/**
 * @file RDKS100.cpp
 * @brief RDK S100 pipeline: camera probe/init, VIN-ISP-PYM/GDC flow setup, frame fetch.
 *
 * Backend of gs130::pipeline::Pipeline for the RDK S100 / Horizon driver stack, built
 * on the node helpers declared in RDKS100.h. It implements the platform-independent
 * interface of pipeline.hpp: the constructor probes both eyes over I2C, init() builds
 * one vflow per eye (camera -> VIN -> ISP, optionally PYM and GDC), start() starts
 * both flows, get_frame() copies frames into caller buffers, and deinit() tears the
 * hardware down again.
 *
 * Ownership: acquired handles and buffers are stored in Impl. deinit() releases a
 * fully initialized stream; see pipeline.hpp for the partial-initialization limitation.
 * Calibration remains caller-owned and its pointer is not retained after init().
 *
 * This file is part of gs130_camera_sdk (https://github.com/D-Robotics/gs130_camera_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "devices/pipeline/RDKS100/RDKS100.h"

#include "devices/pipeline/pipeline.hpp"

#include "base/i2c/i2c.hpp"
#include "base/rectify/rectify.hpp"

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <vector>

namespace gs130 {
namespace pipeline {

namespace {
// SC132GS sensor identification, used to probe a candidate bus + address; the same
// register and value gs130-detect-camera reports
constexpr uint16_t kChipIdReg = 0x3107;
constexpr uint16_t kChipId    = 0x0132;

// S100 resource identifiers; see the file comment for where these come from.
constexpr uint32_t kIspHwId = 0;
constexpr uint32_t kPymHwId = 0;

// First ISP/PYM slot id. S100 hands these out from isp0_next_slot_id, which starts at 4,
// and the platform's sensor table for this sensor carries slot_id 4 too. Starting at 0
// left the nodes unbound.
constexpr uint32_t kIspSlotBase = 4;

/** Convert a driver frame timestamp to nanoseconds. trig_tv (the LPWM exposure trigger)
 *  is preferred; the driver's timestamps field and tv are fallbacks. */
uint64_t frame_ts_ns(const hbn_frame_info_t &info)
{
    if(info.trig_tv.tv_sec != 0 || info.trig_tv.tv_usec != 0)
        return static_cast<uint64_t>(info.trig_tv.tv_sec) * 1000000000ULL +
               static_cast<uint64_t>(info.trig_tv.tv_usec) * 1000ULL;
    if(info.timestamps != 0)
        return info.timestamps;
    return static_cast<uint64_t>(info.tv.tv_sec) * 1000000000ULL +
           static_cast<uint64_t>(info.tv.tv_usec) * 1000ULL;
}


} // namespace

struct Pipeline::Impl {
    bool probed     = false;   // both sensors answered the chip-id probe
    bool inited = false;   // stream built
    bool hb_mem_opened = false;   // hb_mem_module_open() has been called and not closed

    // Per-camera resources, passed one by one when calling the node helpers.
    // Handle 0 means "stage not present" for vflow_build(); the flow owns the nodes
    // it was given, so they are all released by the teardown of that flow.
    struct CamHw {
        camera_handle_t    cam_fd  = 0;
        hbn_vnode_handle_t vin = 0, isp = 0, pym = 0, gdc = 0;
        hbn_vflow_handle_t vflow    = 0;
        hbn_vnode_handle_t output_node = 0;
        uint32_t           output_chn  = 0;
        uint32_t           isp_slot    = 0;   // ISP slot, reused as the PYM slot
        hb_mem_common_buf_t gdc_bin{};   // GDC config binary; only allocated in GDC modes
        int  mipi_rx = -1, reset_gpio = -1, i2c_bus = -1;
        uint8_t i2c_addr = 0;   // 0 = this eye was not found during probing
    } cam[static_cast<std::size_t>(CamIndex::Num)];

    // Geometry parameters, all in pixels: input = sensor frame, mid = PYM output and
    // therefore GDC input, output = size handed to the caller. output_roi is the crop
    // out of the frame the scaling stages read that the output covers; the intrinsics
    // correction at the end of init() is expressed with it.
    uint32_t input_w = 0, input_h = 0, output_w = 0, output_h = 0;
    uint32_t mid_w = 0, mid_h = 0;       /* size PYM writes (and the node after it reads) */
    uint32_t pym_in_w = 0, pym_in_h = 0;  /* size PYM reads: the sensor frame, or the GDC's
                                           * rectified frame on Rect */
    gs130_rect_t output_roi{};
    int install_angle = 0;   // degrees, normalized to [0, 360)
};

Pipeline::Pipeline(
    uint8_t left_addr, uint8_t right_addr,
    const uint8_t *bus_list, std::size_t bus_num)
    : impl_(std::make_unique<Impl>())
{
    // Iterate bus_list: read chip id to confirm an SC132GS on that bus + address.
    // The first bus that answers wins for each eye; the other eye keeps being probed
    // on the remaining buses.
    for(std::size_t i = 0; i < bus_num; ++i){
        const uint8_t bus = bus_list[i];

        // Right camera
        if(impl_->cam[static_cast<std::size_t>(CamIndex::Right)].i2c_addr == 0){
            base::I2cDevice dev(bus, right_addr);
            uint16_t id = 0;
            if(dev && dev.read16(kChipIdReg, &id) == Status::Ok && id == kChipId){
                impl_->cam[static_cast<std::size_t>(CamIndex::Right)].i2c_bus  = bus;
                impl_->cam[static_cast<std::size_t>(CamIndex::Right)].i2c_addr = right_addr;
            }
            dev.close();
        }
        // Left camera
        if(impl_->cam[static_cast<std::size_t>(CamIndex::Left)].i2c_addr == 0){
            base::I2cDevice dev(bus, left_addr);
            uint16_t id = 0;
            if(dev && dev.read16(kChipIdReg, &id) == Status::Ok && id == kChipId){
                impl_->cam[static_cast<std::size_t>(CamIndex::Left)].i2c_bus  = bus;
                impl_->cam[static_cast<std::size_t>(CamIndex::Left)].i2c_addr = left_addr;
            }
            dev.close();
        }
    }

    impl_->probed = (impl_->cam[static_cast<std::size_t>(CamIndex::Right)].i2c_addr != 0 &&
                     impl_->cam[static_cast<std::size_t>(CamIndex::Left)].i2c_addr != 0);
}

Pipeline::~Pipeline()
{
    deinit();
}

Pipeline::operator bool() const { return impl_ && impl_->probed; }

Status Pipeline::init(const PipelineConfig &cfg, StereoImuModel *cal)
{
    if(!impl_->probed)return Status::NotFound;
    if(impl_->inited)return Status::ParamError;

    hb_mem_module_open();   // hold the hb_mem module for the pipeline's whole lifetime
    impl_->hb_mem_opened = true;

    // Geometry parameters. A zero size is rejected up front: aspect_roi() and
    // roi_ratio_exact() divide by the requested width and height, so a zero would fault
    // instead of being reported as a parameter error. Everything after the hb_mem_module_open()
    // above is released by deinit() on the way out.
    if(cfg.sensor_width == 0 || cfg.sensor_height == 0 ||
       cfg.output_width == 0 || cfg.output_height == 0){
        deinit();
        return Status::ParamError;
    }

    impl_->input_w = cfg.sensor_width;
    impl_->input_h = cfg.sensor_height;
    impl_->output_w = cfg.output_width;
    impl_->output_h = cfg.output_height;

    // Raw mode hands the sensor frame through untouched: the requested output size
    // must therefore be the sensor size
    if(cfg.mode == OutputMode::Raw)
        if(cfg.sensor_width != cfg.output_width || cfg.sensor_height != cfg.output_height){
            deinit();
            return Status::ParamError;
        }

    // Determine mipi_rx and reset_gpio; both are keyed by the bus the eye was found on.
    // The same pass fixes each eye's node slot, so the two eyes cannot share one.
    for(std::size_t i = 0; i < static_cast<std::size_t>(CamIndex::Num); i++){
        const int bus = impl_->cam[i].i2c_bus;
        if(bus < 0 || bus >= 32 || cfg.bus_mipi_rx[bus] == 0xFF){
            deinit();
            return Status::ParamError;
        }

        impl_->cam[i].mipi_rx    = cfg.bus_mipi_rx[bus];
        impl_->cam[i].reset_gpio = cfg.bus_reset_gpio[bus];
        impl_->cam[i].isp_slot   = kIspSlotBase + static_cast<uint32_t>(i);
    }

    // Build the stream
    // Camera node
    for(std::size_t i = 0; i < static_cast<std::size_t>(CamIndex::Num); i++){
        // MIPI link parameters for this sensor (link clock, settle and MCLK), fixed
        // here: they are not part of PipelineConfig and do not depend on the geometry.
        // The link clock is the one the board's own dual-sensor configuration states for
        // this module; the X5 backend runs the same sensor at 1200.
        int ret = camera_open(
            &impl_->cam[i].cam_fd,
            impl_->cam[i].i2c_addr,
            impl_->input_w, impl_->input_h, cfg.fps,
            cfg.line_length, cfg.frame_length,
            2400, 20, 1, cfg.tuning_file);

        if(ret){
            deinit();
            return Status::HwError;
        }
    }

    // VIN node
    for(std::size_t i = 0; i < static_cast<std::size_t>(CamIndex::Num); i++){
        int ret = vin_open(
            &impl_->cam[i].vin,
            impl_->cam[i].mipi_rx,
            impl_->input_w, impl_->input_h, cfg.fps);
        if(ret){
            deinit();
            return Status::HwError;
        }
    }

    // Every mode runs VIN through the ISP and uses the offline/DDR YUV420 output on
    // channel 0. Raw reads it directly; Resize binds it to PYM; Rect binds it through GDC.
    for(std::size_t i = 0; i < static_cast<std::size_t>(CamIndex::Num); i++){
        int ret = isp_open(
            &impl_->cam[i].isp,
            impl_->input_w, impl_->input_h,
            kIspHwId, impl_->cam[i].isp_slot, cfg.fps);
        if(ret){
            deinit();
            return Status::HwError;
        }
    }

    // PYM reads pym_in_* and writes mid_*; output_roi is the crop the intrinsics
    // correction below assumes. For Rect the GDC runs first and resamples onto the
    // rectification grid, which stereo_rectify() grows past the sensor size.
    impl_->pym_in_w = impl_->input_w;
    impl_->pym_in_h = impl_->input_h;

    if(cfg.mode == OutputMode::Raw){
        impl_->mid_w = impl_->input_w;
        impl_->mid_h = impl_->input_h;
    }
    else{
        // Resize and Rect both update output intrinsics, so both need calibration.
        if(cal == nullptr)return Status::ParamError;

        // Normalize the install angle to [0, 360) degrees; only a right angle can be
        // expressed by the map rotation below
        impl_->install_angle = ((cal->install_angle % 360) + 360) % 360;
        if(impl_->install_angle % 90 != 0) return Status::ParamError;

        // Rotated width/height: the tables are built in the rotated orientation, so the
        // frame size is swapped for 90 and 270 degrees
        const bool swap = (impl_->install_angle == 90 || impl_->install_angle == 270);

        if(cfg.mode == OutputMode::Rect){
            // Rotation is applied by the GDC, so the rectification is computed for the
            // rotated orientation and the source size is swapped
            const uint32_t src_w = swap ? impl_->input_h : impl_->input_w;
            const uint32_t src_h = swap ? impl_->input_w : impl_->input_h;
            uint32_t rect_w = 0, rect_h = 0;

            // Generate the rectification tables: stereo_rectify() chooses the rectified
            // grid size, one table per eye (index 0 = Right, 1 = Left)
            std::vector<RemapPoint> map[static_cast<std::size_t>(CamIndex::Num)];
            Status st = base::stereo_rectify(cal, src_w, src_h, &rect_w, &rect_h,
                                             &map[static_cast<std::size_t>(CamIndex::Left)],
                                             &map[static_cast<std::size_t>(CamIndex::Right)]);
            if(st != Status::Ok){
                deinit();
                return Status::Unsupported;
            }

            // The GDC rectifies onto the grid stereo_rectify() chose, which is larger than
            // the sensor frame (1088x1280 -> 1088x1536 here); its output size is that grid,
            // not the requested output. This mirrors the X5 path, which also hands the
            // rectified grid on to its scaling stage.
            for(std::size_t i = 0; i < static_cast<std::size_t>(CamIndex::Num); i++){
                int ret = gdc_open(
                    &impl_->cam[i].gdc, &impl_->cam[i].gdc_bin,
                    map[i].data(), impl_->input_w, impl_->input_h,
                    rect_w, rect_h, impl_->install_angle);
                if(ret){
                    deinit();
                    return Status::HwError;
                }
            }

            // PYM now reads the rectified frame and scales it to the requested size; the
            // aspect crop it takes is the crop reported to the caller.
            impl_->pym_in_w = rect_w;
            impl_->pym_in_h = rect_h;
            impl_->mid_w    = impl_->output_w;
            impl_->mid_h    = impl_->output_h;
            impl_->output_roi = aspect_roi(rect_w, rect_h,
                                           impl_->output_w, impl_->output_h);
        }
        else if(impl_->install_angle != 0){
            // Resize with an install rotation: PYM scales and the GDC only rotates. The
            // intermediate is the requested output with its sides swapped for 90 and 270
            // degrees, so rotating it yields exactly the requested size.
            impl_->mid_w = swap ? impl_->output_h : impl_->output_w;
            impl_->mid_h = swap ? impl_->output_w : impl_->output_h;

            // Generate identity tables on the output grid: a rotation resamples nothing
            std::vector<RemapPoint> map[static_cast<std::size_t>(CamIndex::Num)];
            const uint32_t n = impl_->output_w * impl_->output_h;
            for(std::size_t i = 0; i < static_cast<std::size_t>(CamIndex::Num); i++){
                map[i].resize(n);
                for(uint32_t p = 0; p < n; p++){
                    map[i][p].x = p % impl_->output_w;
                    map[i][p].y = p / impl_->output_w;
                }
            }

            // gdc_open per camera; it reads the scaled frame PYM handed over
            for(std::size_t i = 0; i < static_cast<std::size_t>(CamIndex::Num); i++){
                int ret = gdc_open(
                    &impl_->cam[i].gdc, &impl_->cam[i].gdc_bin,
                    map[i].data(), impl_->mid_w, impl_->mid_h,
                    impl_->output_w, impl_->output_h, impl_->install_angle);
                if(ret){
                    deinit();
                    return Status::HwError;
                }
            }
        }
        else{
            // Resize without rotation: PYM does the whole job, there is no GDC
            impl_->mid_w = impl_->output_w;
            impl_->mid_h = impl_->output_h;
        }

        // For Rect the crop was taken from the rectified grid and published above; every
        // other mode crops the sensor frame PYM is given.
        if(cfg.mode != OutputMode::Rect)
            impl_->output_roi = aspect_roi(impl_->input_w, impl_->input_h,
                                           impl_->mid_w, impl_->mid_h);
    }

    // PYM node: reads pym_in_w x pym_in_h (the sensor frame, or the GDC's rectified
    // frame on Rect) and writes mid_w x mid_h
    if(cfg.mode != OutputMode::Raw){
        // The caller chooses the output size; there is no supported-size table here. The
        // aspect crop is the one thing that must line up before the node is opened, since
        // output_roi is derived from it. Any size the hardware cannot serve is left to
        // fail in pym_open(), which fails init.
        if(roi_ratio_exact(impl_->pym_in_w, impl_->pym_in_h,
                           impl_->mid_w, impl_->mid_h) != 0){
            deinit();
            return Status::Unsupported;
        }

        // pym_open per camera
        for(std::size_t i = 0; i < static_cast<std::size_t>(CamIndex::Num); i++){
            int ret = pym_open(
                &impl_->cam[i].pym, impl_->pym_in_w, impl_->pym_in_h,
                impl_->mid_w, impl_->mid_h,
                kPymHwId, impl_->cam[i].isp_slot);
            if(ret){
                deinit();
                return Status::HwError;
            }
        }

        // Write back intrinsics: the crop the frame was taken from and the scale it was
        // written at (scaling-stage coordinates -> output coordinates)
        const gs130_rect_t roi = impl_->output_roi;
        const double sfx = static_cast<double>(impl_->output_w) / roi.w;
        const double sfy = static_cast<double>(impl_->output_h) / roi.h;
        for(CameraIntrinsics *k : {&cal->cam_left, &cal->cam_right}){
            k->fx = k->fx * sfx;
            k->fy = k->fy * sfy;
            k->cx = (k->cx - roi.x) * sfx;
            k->cy = (k->cy - roi.y) * sfy;
            k->K[0] = k->fx; k->K[2] = k->cx;
            k->K[4] = k->fy; k->K[5] = k->cy;
        }
    }

    // Bind each eye's nodes into its own flow and remember the node/channel to fetch frames
    // from (Raw: ISP, Resize: PYM, Rect: PYM, which reads what the GDC rectified)
    const int gdc_before_pym = (cfg.mode == OutputMode::Rect) ? 1 : 0;
    for(std::size_t i = 0; i < static_cast<std::size_t>(CamIndex::Num); i++){
        int ret = vflow_build(
            &impl_->cam[i].vflow, impl_->cam[i].cam_fd,
            impl_->cam[i].vin, impl_->cam[i].isp,
            impl_->cam[i].gdc, impl_->cam[i].pym,
            0, gdc_before_pym,
            &impl_->cam[i].output_node, &impl_->cam[i].output_chn);
        if(ret){
            deinit();
            return Status::HwError;
        }
    }

    impl_->inited = true;
    return Status::Ok;
}

void Pipeline::deinit()
{
    // Runs on every init() failure path, where inited is still false, so an
    // "if(!inited) return" guard would skip cleanup entirely. teardown_cam()
    // ignores zero handles.

    for(std::size_t i = 0; i < static_cast<std::size_t>(CamIndex::Num); i++){
        // teardown_cam() only releases nodes the flow owns, which is true once
        // vflow_build() has added them; before that init() has to close them.
        if(impl_->cam[i].vflow == 0){
            if(impl_->cam[i].vin)hbn_vnode_close(impl_->cam[i].vin);
            if(impl_->cam[i].isp)hbn_vnode_close(impl_->cam[i].isp);
            if(impl_->cam[i].pym)hbn_vnode_close(impl_->cam[i].pym);
            if(impl_->cam[i].gdc)hbn_vnode_close(impl_->cam[i].gdc);
        }
        teardown_cam(impl_->cam[i].vflow, impl_->cam[i].cam_fd,
                     impl_->cam[i].vin, impl_->cam[i].isp,
                     impl_->cam[i].pym, impl_->cam[i].gdc,
                     &impl_->cam[i].gdc_bin, impl_->cam[i].reset_gpio);
        // teardown passes the handles by value, so they are not written back; clear the
        // stored ones here to leave no stale handle behind
        impl_->cam[i].cam_fd = 0;
        impl_->cam[i].vin = impl_->cam[i].isp = impl_->cam[i].pym = impl_->cam[i].gdc = 0;
        impl_->cam[i].vflow = 0;
        impl_->cam[i].output_node = 0;
        impl_->cam[i].output_chn  = 0;
    }

    // Restore each controlled sensor to its normal, detectable state after its flow
    // has been destroyed by running the reset sequence again.
    for(std::size_t i = 0; i < static_cast<std::size_t>(CamIndex::Num); i++){
        if(impl_->cam[i].reset_gpio >= 0)
            sensor_power(impl_->cam[i].reset_gpio, 1);
    }

    impl_->inited = false;
    if(impl_->hb_mem_opened){
        hb_mem_module_close();
        impl_->hb_mem_opened = false;
    }
}

Status Pipeline::start(CamIndex first)
{
    if(!impl_->inited)
        return Status::ParamError;

    const std::size_t a = static_cast<std::size_t>(first);
    const std::size_t b = static_cast<std::size_t>(
        first == CamIndex::Right ? CamIndex::Left : CamIndex::Right);

    if(hbn_vflow_start(impl_->cam[a].vflow) != 0){
        deinit();
        return Status::HwError;
    }
    if(hbn_vflow_start(impl_->cam[b].vflow) != 0){
        deinit();
        return Status::HwError;
    }
    return Status::Ok;
}

void Pipeline::stop()
{
    if(!impl_->inited)
        return;

    for(std::size_t i = 0; i < static_cast<std::size_t>(CamIndex::Num); i++)
        if(impl_->cam[i].vflow != 0)
            hbn_vflow_stop(impl_->cam[i].vflow);
}

Status Pipeline::get_frame(CamIndex idx,
                           uint8_t *y, uint8_t *uv,
                           uint32_t width, uint32_t height,
                           uint32_t y_stride, uint32_t uv_stride,
                           uint64_t *timestamp_ns,
                           uint32_t timeout_ms)
{
    // y, uv and timestamp_ns are caller-owned; the stream must be initialized
    if(y == nullptr || uv == nullptr || timestamp_ns == nullptr || !impl_->inited)
        return Status::ParamError;

    const std::size_t i = static_cast<std::size_t>(idx);
    auto &c = impl_->cam[i];

    /* ISP and PYM publish frame groups on S100. PYM groups its pyramid slots, while the
     * platform also marks direct ISP output as stream_group=1. GDC publishes one image.
     * Asking either grouped producer for a single image starts the flow but never returns a
     * frame. The requested image is entry output_chn in the returned group. */
    hbn_vnode_image_t img;
    hbn_vnode_image_group_t grp;
    memset(&img, 0, sizeof(img));
    memset(&grp, 0, sizeof(grp));

    const bool grouped = (c.output_node == c.pym || c.output_node == c.isp);
    if(grouped){
        if(hbn_vnode_getframe_group(c.output_node, c.output_chn, timeout_ms, &grp) != 0)
            return Status::Timeout;
    }
    else if(hbn_vnode_getframe(c.output_node, c.output_chn, timeout_ms, &img) != 0){
        return Status::Timeout;
    }

    const hb_mem_graphic_buf_t &buf =
        grouped ? grp.buf_group.graph_group[c.output_chn] : img.buffer;

    // The node's buffers are allocated CACHED and written by the hardware, so the CPU's
    // view has to be invalidated before the planes below are read; otherwise a read can
    // hit a stale cache line. The platform's capture path does the same immediately
    // after hbn_vnode_getframe (hobot_mipi_cam, src/s100/hobot_mipi_cap_iml.cpp).
    if(buf.virt_addr[0] != nullptr)
        hb_mem_invalidate_buf_with_vaddr((uint64_t)buf.virt_addr[0], buf.size[0]);
    if(buf.plane_cnt > 1 && buf.virt_addr[1] != nullptr)
        hb_mem_invalidate_buf_with_vaddr((uint64_t)buf.virt_addr[1], buf.size[1]);

    // Check width/height; report error on mismatch, the caller's geometry is not
    // converted or cropped here
    if((uint32_t)buf.width != width || (uint32_t)buf.height != height){
        if(grouped)hbn_vnode_releaseframe_group(c.output_node, c.output_chn, &grp);
        else       hbn_vnode_releaseframe(c.output_node, c.output_chn, &img);
        return Status::ParamError;
    }

    // Copy row by row using each plane's stride. A captured RAW10 frame is a single
    // packed plane (the capture node reports plane_cnt 1 and leaves virt_addr[1] null),
    // so the second plane is only copied when the node actually produced one.
    for(uint32_t r = 0; r < height; ++r)
        memcpy(y + (std::size_t)r * y_stride,
               buf.virt_addr[0] + (std::size_t)r * buf.stride, width);
    if(buf.plane_cnt > 1 && buf.virt_addr[1] != nullptr)
        for(uint32_t r = 0; r < height / 2; ++r)
            memcpy(uv + (std::size_t)r * uv_stride,
                   buf.virt_addr[1] + (std::size_t)r * buf.stride, width);

    *timestamp_ns = frame_ts_ns(grouped ? grp.info : img.info);

    if(grouped)hbn_vnode_releaseframe_group(c.output_node, c.output_chn, &grp);
    else       hbn_vnode_releaseframe(c.output_node, c.output_chn, &img);
    return Status::Ok;
}

} // namespace pipeline
} // namespace gs130
