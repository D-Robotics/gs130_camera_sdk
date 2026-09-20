/**
 * @file RDKS100.cpp
 * @brief RDK S100 pipeline: camera probe/init, VIN-ISP-GDC/PYM flow setup, frame fetch.
 *
 * Backend of gs130::pipeline::Pipeline for the RDK S100 / Horizon driver stack, built
 * on the node helpers declared in RDKS100.h. It implements the platform-independent
 * interface of pipeline.hpp: the constructor probes both eyes over I2C, init() builds
 * one vflow per eye (camera -> VIN -> ISP, optionally GDC and PYM), start() starts
 * both flows, get_frame() copies frames into caller buffers, and deinit() tears the
 * hardware down again.
 *
 * The flow logic is the same as the X5 backend's; what changes is which scaling node is
 * opened (PYM instead of VSE) and the resource identifiers the ISP and PYM nodes need on
 * this generation. Those are assigned here:
 *
 *   - ISP hw_id 0: the platform's camera stack uses ISP0 for a chain without YNR, which
 *     is the case here.
 *   - slot_id = eye index (0 for Right, 1 for Left): the platform allocates slots from a
 *     running counter so two pipelines never collide; fixing one slot per eye reproduces
 *     that for the two pipelines this backend creates, without carrying a counter.
 *   - PYM hw_id 0 and the same slot_id as the eye's ISP: the platform pairs them the same
 *     way in its own ISP-only path.
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

/**
 * @brief Convert a driver frame timestamp to nanoseconds.
 *
 * Preference order: trig_tv (LPWM rising edge = exposure trigger time), then the
 * driver's timestamps field, then tv (the time the frame was ready). The exposure
 * time is what the IMU pairing needs, so the other two are only fallbacks for when
 * the trigger information is missing.
 *
 * S100 has no timestamp switch on the capture node: cim_func_desc_t carries no
 * time_stamp_en / time_stamp_mode / ts_src (those exist only in X5's vin_cfg.h), and the
 * platform's own S100 camera stack reads trig_tv with no such flag set. The preference
 * order below is therefore unchanged from X5, only the trigger timestamp is no longer
 * explicitly enabled anywhere.
 *
 * @param[in] info Frame information returned with the frame.
 * @return Timestamp in nanoseconds on the device clock.
 */
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

    // Geometry parameters, all in pixels: input = sensor, mid = GDC output and PYM
    // input (equal to input when GDC is skipped), output = size handed to the caller
    uint32_t input_w = 0, input_h = 0, output_w = 0, output_h = 0;
    uint32_t mid_w = 0, mid_h = 0;
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

    // Geometry parameters
    impl_->input_w = cfg.sensor_width;
    impl_->input_h = cfg.sensor_height;
    impl_->output_w = cfg.output_width;
    impl_->output_h = cfg.output_height;

    // Raw mode hands the sensor frame through untouched: the requested output size
    // must therefore be the sensor size
    if(cfg.mode == OutputMode::Raw)
        if(cfg.sensor_width != cfg.output_width || cfg.sensor_height != cfg.output_height)
            return Status::ParamError;

    // Determine mipi_rx and reset_gpio; both are keyed by the bus the eye was found on.
    // The same pass fixes each eye's node slot, so the two eyes cannot share one.
    for(std::size_t i = 0; i < static_cast<std::size_t>(CamIndex::Num); i++){
        const int bus = impl_->cam[i].i2c_bus;
        if(bus < 0 || bus >= 32 || cfg.bus_mipi_rx[bus] == 0xFF)return Status::ParamError;

        impl_->cam[i].mipi_rx    = cfg.bus_mipi_rx[bus];
        impl_->cam[i].reset_gpio = cfg.bus_reset_gpio[bus];
        impl_->cam[i].isp_slot   = static_cast<uint32_t>(i);
    }

    // Build the stream
    // Camera node
    for(std::size_t i = 0; i < static_cast<std::size_t>(CamIndex::Num); i++){
        // MIPI link parameters for this sensor (link clock, settle and MCLK), fixed
        // here: they are not part of PipelineConfig and do not depend on the geometry
        int ret = camera_open(
            &impl_->cam[i].cam_fd,
            impl_->cam[i].i2c_addr,
            impl_->input_w, impl_->input_h, cfg.fps,
            cfg.line_length, cfg.frame_length,
            1200, 20, 1, cfg.tuning_file);

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

    // ISP node
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

    // GDC node
    // Rect: rectify + rotation;
    // Resize + install rotation: pure rotation;
    // Raw: skip GDC
    if(cfg.mode != OutputMode::Raw){
        // Resize and Rect both update output intrinsics, so both need calibration.
        if(cal == nullptr)return Status::ParamError;

        // Normalize the install angle to [0, 360) degrees; only a right angle can be
        // expressed by the map rotation below
        impl_->install_angle = ((cal->install_angle % 360) + 360) % 360;
        if(impl_->install_angle % 90 != 0) return Status::ParamError;

        // Handle Rect and rotated Resize cases
        if(cfg.mode == OutputMode::Rect || impl_->install_angle != 0){
            // Rotated width/height: the map is built in the output orientation, so the
            // sensor size is swapped for 90 and 270 degrees
            const bool swap = (impl_->install_angle == 90 || impl_->install_angle == 270);
            const uint32_t src_w = swap ? impl_->input_h : impl_->input_w;
            const uint32_t src_h = swap ? impl_->input_w : impl_->input_h;

            // Generate the GDC map: mid_w x mid_h grid entries, each holding the source
            // pixel to sample; stereo_rectify() chooses the rectified grid size
            std::vector<RemapPoint> map[static_cast<std::size_t>(CamIndex::Num)];
            // Stereo distortion rectification for both cameras
            if(cfg.mode == OutputMode::Rect){
                Status st = base::stereo_rectify(cal, src_w, src_h,
                                                 &impl_->mid_w, &impl_->mid_h,
                                                 &map[static_cast<std::size_t>(CamIndex::Left)],
                                                 &map[static_cast<std::size_t>(CamIndex::Right)]);
                if(st != Status::Ok){
                    deinit();
                    return Status::Unsupported;
                }
            }
            // Rotation-only correction: identity map, so the GDC only rotates and the
            // mid size stays the source size
            else{
                impl_->mid_w = src_w;
                impl_->mid_h = src_h;

                // Generate identity Map
                const uint32_t n = impl_->mid_w * impl_->mid_h;
                for(std::size_t i = 0; i < static_cast<std::size_t>(CamIndex::Num); i++){
                    map[i].resize(n);
                    for(uint32_t p = 0; p < n; p++){
                        map[i][p].x = p % impl_->mid_w;
                        map[i][p].y = p / impl_->mid_w;
                    }
                }
            }

            // gdc_open per camera (map[0]=Right, map[1]=Left)
            for(std::size_t i = 0; i < static_cast<std::size_t>(CamIndex::Num); i++){
                int ret = gdc_open(
                    &impl_->cam[i].gdc, &impl_->cam[i].gdc_bin,
                    map[i].data(), impl_->input_w, impl_->input_h,
                    impl_->mid_w, impl_->mid_h, impl_->install_angle);
                if(ret){
                    deinit();
                    return Status::HwError;
                }
            }
        }
        // Not through GDC: mid takes the sensor size directly (PYM input)
        else{
            impl_->mid_w = impl_->input_w;
            impl_->mid_h = impl_->input_h;
        }
    }

    // PYM node
    if(cfg.mode != OutputMode::Raw){
        // ROI divisibility check (guard against integer-division truncation)
        if(roi_ratio_exact(impl_->mid_w, impl_->mid_h,
                           impl_->output_w, impl_->output_h) != 0){
            deinit();
            return Status::Unsupported;
        }

        // Aspect-ratio-preserving ROI, which pym_open() turns into the source region of
        // the single enabled output slot
        const gs130_rect_t roi = aspect_roi(impl_->mid_w, impl_->mid_h,
                                            impl_->output_w, impl_->output_h);

        // pym_open per camera
        for(std::size_t i = 0; i < static_cast<std::size_t>(CamIndex::Num); i++){
            int ret = pym_open(
                &impl_->cam[i].pym, impl_->mid_w, impl_->mid_h,
                impl_->output_w, impl_->output_h,
                kPymHwId, impl_->cam[i].isp_slot);
            if(ret){
                deinit();
                return Status::HwError;
            }
        }

        // Write back intrinsics: PYM aspect crop + scale (mid coords -> output coords)
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

    // Bind each eye's nodes into its own flow and remember the node/channel to fetch
    // frames from (Raw: ISP, Resize: PYM, Rect: PYM behind GDC)
    for(std::size_t i = 0; i < static_cast<std::size_t>(CamIndex::Num); i++){
        int ret = vflow_build(
            &impl_->cam[i].vflow, impl_->cam[i].cam_fd,
            impl_->cam[i].vin, impl_->cam[i].isp,
            impl_->cam[i].gdc, impl_->cam[i].pym,
            0,
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
    if(!impl_->inited)
        return;

    for(std::size_t i = 0; i < static_cast<std::size_t>(CamIndex::Num); i++){
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
    hb_mem_module_close();
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

    hbn_vnode_image_t img;
    memset(&img, 0, sizeof(img));
    if(hbn_vnode_getframe(c.output_node, c.output_chn, timeout_ms, &img) != 0)
        return Status::Timeout;

    // Check width/height; report error on mismatch, the caller's geometry is not
    // converted or cropped here
    if((uint32_t)img.buffer.width != width || (uint32_t)img.buffer.height != height){
        hbn_vnode_releaseframe(c.output_node, c.output_chn, &img);
        return Status::ParamError;
    }

    // Copy row by row using each plane's stride
    for(uint32_t r = 0; r < height; ++r)
        memcpy(y + (std::size_t)r * y_stride,
               img.buffer.virt_addr[0] + (std::size_t)r * img.buffer.stride, width);
    for(uint32_t r = 0; r < height / 2; ++r)
        memcpy(uv + (std::size_t)r * uv_stride,
               img.buffer.virt_addr[1] + (std::size_t)r * img.buffer.stride, width);

    *timestamp_ns = frame_ts_ns(img.info);

    hbn_vnode_releaseframe(c.output_node, c.output_chn, &img);
    return Status::Ok;
}

} // namespace pipeline
} // namespace gs130
