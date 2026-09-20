/**
 * @file RDKS100.h
 * @brief RDK S100 node helpers: one C function per pipeline stage, plus teardown.
 *
 * Internal header of the RDK S100 backend. It is included by RDKS100.cpp (the platform
 * backend selected at build time) and by the node .c files next to it, never by
 * platform-independent code, because it pulls in the RDK S100 / Horizon driver headers.
 *
 * S100 and X5 share the same Horizon vnode framework (hbn_vnode_open / set_attr /
 * set_ichn_attr / set_ochn_attr / set_ochn_buf_attr and the hbn_vflow_* calls), but the
 * attribute structures and some entry points differ, so this header mirrors RDKX5.h in
 * intent rather than line for line. The differences, all read off the platform's own
 * camera stack (hobot_mipi_cam, src/s100/) and its headers:
 *
 *   - hbn_vnode_set_attr() for VIN takes the wrapper vin_attr_t, not vin_node_attr_t;
 *     the output channel attributes live in vin_attr_t.vin_ochn_attr[VIN_MAIN_FRAME].
 *   - hbn_vnode_set_attr() for ISP takes the wrapper isp_cfg_t, not isp_attr_t.
 *   - VIN's cim_attr has no hdr_mode / time_stamp_en / time_stamp_mode / ts_src; those
 *     are X5-only fields. S100 has mipi_en, cim_pym_flyby, ipi_channels, enable_pattern
 *     and rdma_input instead.
 *   - ISP online/offline is selected with stream_output_mode and axi_output_mode. There
 *     is no input_mode = DDR_MODE, and no FRM_FMT_* / ISP_MAIN_FRAME / CAM_TRUE here.
 *   - GDC uses one gdc_settings_t for all three setters instead of gdc_attr_t +
 *     gdc_ichn_attr_t + gdc_ochn_attr_t.
 *   - The GDC configuration binary is built with hbn_gen_gdc_cfg() and freed with
 *     hbn_free_gdc_cfg(); X5 calls these hbn_gen_gdc_bin() / hbn_free_gdc_bin(), and the
 *     output pointer is uint32_t** there against void** here.
 *   - The scaling node is PYM (HB_PYM) rather than VSE (HB_VSE), and one pym_cfg_t is
 *     passed to all three setters.
 *   - The driver expects the marker value 0x12345678 in the magicNumber field of
 *     vin_node_attr, vin_ochn_attr, vin_attr, pym_cfg and gdc_settings. The platform
 *     headers only declare the field; the value is defined by the platform's camera
 *     stack, so this header defines it (see GS130_S100_MAGIC_NUMBER).
 *
 * There is no shared context: each helper takes the handles, geometry and parameters
 * it needs, so a node can be read (and reused) on its own. The stage helpers all
 * return 0 on success and -1 on failure. Handles and memory-manager buffers are
 * reported through out-parameters and belong to the caller after creation; partially
 * configured handles may remain non-zero on a later setup failure and must be cleaned
 * up by the backend. GDC buffers are freed by teardown_cam().
 *
 * This file is part of gs130_camera_sdk (https://github.com/D-Robotics/gs130_camera_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#ifndef GS130_PIPELINE_RDKS100_H
#define GS130_PIPELINE_RDKS100_H

#include <stdint.h>
#include <stddef.h>

#include <hb_mem_mgr.h>
#include <hb_camera_interface.h>
#include <hbn_vpf_interface.h>   /* vnode / vflow API, hbn_gen_gdc_cfg */
#include <hbn_vin_cfg.h>         /* vin_attr_t and friends */
#include <hbn_isp_cfg.h>         /* isp_cfg_t and friends */
#include <hb_comm_isp.h>         /* ISP enums: AXI_OUTPUT_MODE_*, STREAM_OUTPUT_MODE_*, ... */
#include <hbn_pym_cfg.h>         /* pym_cfg_t, roi_box_t, MAX_DS_NUM */
#include <hb_gdc_cfg.h>          /* gdc_settings_t */
/* gdc_bin_cfg.h and hb_gdc_data_info.h share the include guard HB_GDC_DATA_INFO_H_ and
 * define the same param_t / window_t, so pulling in one of them is enough. */
#include <gdc_bin_cfg.h>         /* param_t, window_t */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Marker value the S100 driver expects in every magicNumber field.
 *
 * The value is not part of the platform headers; the platform's own camera stack defines
 * it locally (hobot_mipi_cam, vp_sensors.h: `#define MAGIC_NUMBER 0x12345678`).
 */
#define GS130_S100_MAGIC_NUMBER 0x12345678u

/**
 * @brief YUV420 output on the ISP's AXI output, i.e. the offline (DDR) frame format.
 *
 * Both platforms hand NV12 to the stages behind the ISP; on S100 that is expressed with
 * axi_output_mode rather than with an FRM_FMT_NV12 input_mode.
 */
#define GS130_S100_ISP_AXI_FORMAT AXI_OUTPUT_MODE_YUV420

/**
 * @brief Run the sensor reset sequence on one GPIO.
 *
 * The GPIO is driven through sysfs (export, direction = out) and then pulsed
 * "1" -> "0" -> "1", holding 30 ms after every step of the sequence. The sensor
 * stays powered throughout: this releases it from reset and restores it to its
 * normal, detectable state.
 *
 * @param[in] gpio Reset GPIO number; negative disables GPIO control and returns 0.
 * @param[in] on   Non-zero runs the reset sequence above, 0 drives the line low.
 * @return 0 when the sequence ran (or GPIO control is disabled), -1 when the GPIO
 *         value file cannot be opened.
 */
int sensor_power(int gpio, int on);

/**
 * @brief Create the camera device for one sensor: MIPI receiver and sensor config.
 *
 * Fills mipi_config_t and camera_config_t for an externally triggered sensor (the
 * frame sync comes from the LPWM generator in vin_open(), so both eyes expose on the
 * same trigger) and calls hbn_camera_create(). The tuning file path is copied into
 * the camera configuration's calibration name.
 *
 * S100's camera_config_t carries three fields X5's does not (eeprom_addr, serial_addr,
 * extra_mode, config_index) and both structures are terminated by an end_flag; the
 * ones without a meaningful value here are left at 0 / spelled out in the definition.
 *
 * @param[out] cam_fd      Receives the camera handle; set to 0 when creation fails.
 * @param[in]  i2c_addr    Sensor I2C address, i.e. the address it was probed at.
 * @param[in]  width,height Sensor output size in pixels.
 * @param[in]  fps         Frame rate in frames per second.
 * @param[in]  line_length Sensor line length in pixel clocks.
 * @param[in]  frame_length Sensor frame length in lines.
 * @param[in]  mipiclk,settle,mclk MIPI link parameters handed to the driver as-is;
 *                         the backend passes fixed values for this sensor.
 * @param[in]  tuning_file ISP tuning file path, borrowed and not retained; NULL
 *                         selects the driver's own "disable" tuning.
 * @return 0 on success, -1 when hbn_camera_create() fails.
 */
int camera_open(
    camera_handle_t *cam_fd,
    uint8_t i2c_addr,
    uint32_t width, uint32_t height, uint32_t fps,
    uint32_t line_length, uint32_t frame_length,
    uint16_t mipiclk, uint16_t settle, uint16_t mclk,
    const char *tuning_file);

/**
 * @brief Open the VIN node: MIPI capture (CIM) plus the LPWM frame trigger.
 *
 * Frames are written to DDR (no ISP fly-by) and all four LPWM channels are configured
 * with the same period, offset and pulse width; board routing determines which outputs
 * drive the cameras and IMU FSYNC.
 *
 * Unlike X5, the configuration is passed as a single vin_attr_t; the output channel
 * attributes are taken from its vin_ochn_attr[VIN_MAIN_FRAME] entry and the buffer
 * count from vin_ochn_buff_attr[VIN_MAIN_FRAME]. The magicNumber markers are filled in
 * here because the driver rejects the node without them.
 *
 * @note The SDK layer uses this frame period as the master time base of the IMU
 *       FSYNC tracker (see src/gs130.cpp), which is why the period is derived from
 *       the configured frame rate.
 *
 * @param[out] vin        Receives the VIN vnode handle; set to 0 only when opening the node fails.
 * @param[in]  mipi_rx    MIPI RX index this sensor is wired to; also the node's hw_id.
 * @param[in]  width,height Frame size in pixels.
 * @param[in]  fps        Frame rate in frames per second; sets the LPWM period.
 * @return 0 on success, -1 when the node cannot be opened or configured.
 */
int vin_open(
    hbn_vnode_handle_t *vin,
    int mipi_rx,
    uint32_t width, uint32_t height, uint32_t fps);

/**
 * @brief Open the ISP node for offline (DDR) processing with NV12 output.
 *
 * The ISP is configured through the isp_cfg_t wrapper. Offline operation is selected on
 * the output channel: stream_output_mode is disabled and axi_output_mode is set to the
 * YUV420 format the later stages consume.
 *
 * @param[out] isp        Receives the ISP vnode handle; set to 0 only when opening the node fails.
 * @param[in]  width,height Frame size in pixels.
 * @param[in]  hw_id      ISP hardware instance; the backend fixes this per platform.
 * @param[in]  slot_id    ISP slot for this eye; the backend assigns one per camera so
 *                        two eyes do not collide. It is the slot PYM must reuse.
 * @param[in]  fps        Frame rate in frames per second, reported to the ISP scheduler.
 * @return 0 on success, -1 when the node cannot be opened or configured.
 */
int isp_open(
    hbn_vnode_handle_t *isp,
    uint32_t width, uint32_t height,
    uint32_t hw_id, uint32_t slot_id, uint32_t fps);

/**
 * @brief Check that the aspect-preserving ROI of an input/output size pair divides exactly.
 *
 * Guards the integer division in aspect_roi(): a crop size that is not an exact
 * multiple of the scale would be truncated, so the caller reports the configuration
 * as unsupported instead of streaming a silently wrong geometry.
 *
 * @param[in] in_w,in_h   Input size in pixels.
 * @param[in] out_w,out_h Output size in pixels.
 * @return 0 if the ROI width/height divide exactly, -1 otherwise.
 */
int roi_ratio_exact(uint32_t in_w, uint32_t in_h,
                    uint32_t out_w, uint32_t out_h);

/**
 * @brief A pixel rectangle in input coordinates, as used by aspect_roi().
 *
 * X5 defines this as common_rect_t in cam_def.h. That header does not exist on S100,
 * so the two fields the backend actually needs are declared here instead of pulling in
 * the whole X5 camera definition header.
 */
typedef struct {
    uint32_t x;
    uint32_t y;
    uint32_t w;
    uint32_t h;
} gs130_rect_t;

/**
 * @brief Crop an ROI from 'in' with the same aspect ratio as 'out', centered.
 *
 * @param[in] in_w,in_h   Input size in pixels.
 * @param[in] out_w,out_h Output size in pixels.
 * @return The crop rectangle in input coordinates: full height or full width, the
 *         other axis centered. Validate it with roi_ratio_exact() before using it.
 */
gs130_rect_t aspect_roi(uint32_t in_w, uint32_t in_h,
                        uint32_t out_w, uint32_t out_h);

/**
 * @brief Open the PYM node: aspect-preserving ROI crop, then scaling.
 *
 * S100's counterpart of the X5 VSE node. The backend keeps the X5 semantics on purpose:
 * aspect_roi() picks a centered crop of the input with the output's aspect ratio, and
 * that crop becomes the PYM source region, so the output is never stretched.
 *
 * This is deliberately not what the platform's own camera stack does. hobot_mipi_cam
 * passes the whole pyramid base layer as the region and lets the requested size be the
 * output, which stretches whenever the aspect ratios differ. Keeping the crop keeps the
 * two platforms' outputs comparable and lets the backend reuse its intrinsics update.
 *
 * Unlike VSE, PYM takes one configuration structure for the node attribute, the input
 * channel and the output channel; the same pym_cfg_t is passed to all three setters.
 *
 * @param[out] pym        Receives the PYM vnode handle; set to 0 only when opening the node fails.
 * @param[in]  in_w,in_h  Input size in pixels.
 * @param[in]  out_w,out_h Output size in pixels.
 * @param[in]  hw_id      PYM hardware instance; the backend fixes this per platform.
 * @param[in]  slot_id    PYM slot for this eye, which is the ISP slot of the same eye.
 * @return 0 on success, -1 when the node cannot be opened or configured.
 */
int pym_open(
    hbn_vnode_handle_t *pym,
    uint32_t in_w, uint32_t in_h,
    uint32_t out_w, uint32_t out_h,
    uint32_t hw_id, uint32_t slot_id);

/**
 * @brief Open the GDC node from a generated map: sub-pixel offset, clamp, rotation.
 *
 * @p map holds grid_w * grid_h RemapPoint values (output pixel -> source sampling
 * coordinate). Every point is offset by +0.5 px (bilinear interpolation
 * compensation), clamped to the source bounds so no black border appears, and then
 * rotated by @p install_angle. The result is encoded into a GDC configuration binary
 * with hbn_gen_gdc_cfg() and handed to the node with the geometry of both sides.
 *
 * The map layout is expected to match the driver's point_t{double x, y}; this is
 * maintained by hand, there is no compile-time check of it. param_t and window_t are
 * shared with X5, so this helper differs from its X5 counterpart only in how the node
 * is configured once the binary exists.
 *
 * @param[out] gdc        Receives the GDC vnode handle; set to 0 when the open fails.
 * @param[out] gdc_bin    Receives the memory-manager buffer holding the configuration
 *                        binary. It belongs to the caller and must be freed (see
 *                        teardown_cam()).
 * @param[in]  map        Borrowed read-only map of grid_w * grid_h points; not retained.
 * @param[in]  in_w,in_h  GDC input size in pixels, i.e. the sensor geometry.
 * @param[in]  grid_w,grid_h Map size in pixels, which is the GDC output geometry. The
 *                        map is generated in the rotated orientation, so for 90/270
 *                        degrees its coordinates span the sensor height instead.
 * @param[in]  install_angle Rotation in degrees; 0, 90, 180 and 270 have a defined
 *                        mapping, any other value leaves the points unrotated.
 * @return 0 on success, -1 on allocation, binary-generation or node-configuration failure.
 */
int gdc_open(
    hbn_vnode_handle_t *gdc,
    hb_mem_common_buf_t *gdc_bin,
    const void *map, uint32_t in_w, uint32_t in_h,
    uint32_t grid_w, uint32_t grid_h, int install_angle);

/**
 * @brief Create the vflow, add and bind the nodes, and report where frames come out.
 *
 * The chain is vin -> isp -> [gdc] -> [pym]; a handle that is 0 is skipped, so Raw
 * (vin -> isp), Resize (vin -> isp -> pym) and Rect (vin -> isp -> gdc -> pym) all
 * use this one helper. The camera is attached to VIN last, once the nodes are bound.
 *
 * @param[out] vflow      Receives the vflow handle; set to 0 when creation fails.
 * @param[in]  cam_fd     Camera handle to attach to VIN.
 * @param[in]  vin,isp,gdc,pym Vnode handles to add and bind; 0 = stage not present.
 * @param[in]  pym_chn    PYM output channel, used when @p pym is present.
 * @param[out] out_node   Receives the node producing frames: pym when present,
 *                        otherwise gdc, otherwise isp. Written on success only.
 * @param[out] out_chn    Receives the channel to read on @p out_node: pym_chn, 0 for
 *                        GDC, or 0 for the direct ISP output. Written on success only.
 * @return 0 on success, -1 when a create, add, bind or attach step fails.
 */
int vflow_build(
    hbn_vflow_handle_t *vflow,
    camera_handle_t cam_fd,
    hbn_vnode_handle_t vin,
    hbn_vnode_handle_t isp,
    hbn_vnode_handle_t gdc,
    hbn_vnode_handle_t pym, uint32_t pym_chn,
    hbn_vnode_handle_t *out_node, uint32_t *out_chn);

/**
 * @brief Tear down one camera: stop and destroy the flow, destroy the camera, free the GDC binary.
 *
 * The vnodes are not closed one by one here: they were added to the flow, which owns
 * them, so destroying the flow releases them.
 *
 * @note The vnode handles and @p reset_gpio are unused (the definition casts them to
 *       void). They stay in the signature so a call site lists the same resources it
 *       passed to the *_open() helpers; the reset GPIO is handled by the caller, which
 *       runs sensor_power() itself once the flow is gone.
 *
 * @param[in] vflow       Flow to stop and destroy; 0 skips the flow.
 * @param[in] cam_fd      Camera handle to destroy; 0 is ignored.
 * @param[in] vin,isp,pym,gdc Unused; see the note above.
 * @param[in,out] gdc_bin Buffer returned by gdc_open(); freed and cleared when its fd is
 *                        set, so a second call cannot free it twice.
 * @param[in] reset_gpio  Unused; see the note above.
 */
void teardown_cam(
    hbn_vflow_handle_t vflow,
    camera_handle_t cam_fd,
    hbn_vnode_handle_t vin, hbn_vnode_handle_t isp,
    hbn_vnode_handle_t pym,
    hbn_vnode_handle_t gdc,
    hb_mem_common_buf_t *gdc_bin, int reset_gpio);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* GS130_PIPELINE_RDKS100_H */
