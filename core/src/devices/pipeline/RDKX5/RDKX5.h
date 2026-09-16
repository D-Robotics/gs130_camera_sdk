/**
 * @file RDKX5.h
 * @brief RDK X5 node helpers: one C function per pipeline stage, plus teardown.
 *
 * Internal header of the RDK X5 backend. It is included by RDKX5.cpp (the platform
 * backend selected at build time) and by the node .c files next to it, never by
 * platform-independent code, because it pulls in the RDK X5 / Horizon driver headers.
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
#ifndef GS130_PIPELINE_RDKX5_H
#define GS130_PIPELINE_RDKX5_H

#include <stdint.h>
#include <stddef.h>

#include <hbn_api.h>
#include <hb_mem_mgr.h>
#include <cam_def.h>
#include <vin_cfg.h>
#include <isp_cfg.h>
#include <vse_cfg.h>
#include <gdc_cfg.h>
#include <hb_camera_interface.h>

#ifdef __cplusplus
extern "C" {
#endif

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
 * Frames are written to DDR (no ISP fly-by) and timestamps are enabled for the IPI
 * VSYNC and trigger events, which is where the frame timestamp comes from. All four
 * LPWM channels are configured with the same period, offset, and pulse width; board
 * routing determines which outputs drive the cameras and IMU FSYNC.
 *
 * @note The SDK layer uses this frame period as the master time base of the IMU
 *       FSYNC tracker (see src/gs130.cpp), which is why the period is derived from
 *       the configured frame rate.
 *
 * @param[out] vin        Receives the VIN vnode handle; set to 0 only when opening the node fails.
 * @param[in]  mipi_rx    MIPI RX index this sensor is wired to.
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
 * The input channel is RAW10 at the given size and the main output channel is NV12
 * (8 bit), so the ISP hands Y/UV planes to the next stage.
 *
 * @param[out] isp        Receives the ISP vnode handle; set to 0 only when opening the node fails.
 * @param[in]  width,height Frame size in pixels.
 * @return 0 on success, -1 when the node cannot be opened or configured.
 */
int isp_open(
    hbn_vnode_handle_t *isp,
    uint32_t width, uint32_t height);

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
 * @brief Crop an ROI from 'in' with the same aspect ratio as 'out', centered.
 *
 * @param[in] in_w,in_h   Input size in pixels.
 * @param[in] out_w,out_h Output size in pixels.
 * @return The crop rectangle in input coordinates: full height or full width, the
 *         other axis centered. Validate it with roi_ratio_exact() before using it.
 */
common_rect_t aspect_roi(uint32_t in_w, uint32_t in_h,
                         uint32_t out_w, uint32_t out_h);

/**
 * @brief Open the VSE node: aspect-preserving ROI crop, then scaling.
 *
 * @param[out] vse        Receives the VSE vnode handle; set to 0 only when opening the node fails.
 * @param[in]  in_w,in_h  Input size in pixels.
 * @param[in]  out_w,out_h Output size in pixels.
 * @param[in]  vse_chn    Output channel to configure, and the channel the caller reads
 *                        frames from later; the caller selects the down- or
 *                        up-scaling channel of this hardware.
 * @return 0 on success, -1 when the node cannot be opened or configured.
 */
int vse_open(
    hbn_vnode_handle_t *vse,
    uint32_t in_w, uint32_t in_h,
    uint32_t out_w, uint32_t out_h,
    uint32_t vse_chn);

/**
 * @brief Open the GDC node from a generated map: sub-pixel offset, clamp, rotation.
 *
 * @p map holds grid_w * grid_h RemapPoint values (output pixel -> source sampling
 * coordinate). Every point is offset by +0.5 px (bilinear interpolation
 * compensation), clamped to the source bounds so no black border appears, and then
 * rotated by @p install_angle. The result is encoded into a GDC configuration binary
 * with hbn_gen_gdc_bin() and handed to the node with the geometry of both sides.
 *
 * The map layout is expected to match the driver's point_t{double x, y}; this is
 * maintained by hand, there is no compile-time check of it.
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
 * The chain is vin -> isp -> [gdc] -> [vse]; a handle that is 0 is skipped, so Raw
 * (vin -> isp), Resize (vin -> isp -> vse) and Rect (vin -> isp -> gdc -> vse) all
 * use this one helper. The camera is attached to VIN last, once the nodes are bound.
 *
 * @param[out] vflow      Receives the vflow handle; set to 0 when creation fails.
 * @param[in]  cam_fd     Camera handle to attach to VIN.
 * @param[in]  vin,isp,gdc,vse Vnode handles to add and bind; 0 = stage not present.
 * @param[in]  vse_chn    VSE output channel, used when @p vse is present.
 * @param[out] out_node   Receives the node producing frames: vse when present,
 *                        otherwise gdc, otherwise isp. Written on success only.
 * @param[out] out_chn    Receives the channel to read on @p out_node: vse_chn, 0 for
 *                        GDC, or ISP_MAIN_FRAME for the direct ISP output. Written on
 *                        success only.
 * @return 0 on success, -1 when a create, add, bind or attach step fails.
 */
int vflow_build(
    hbn_vflow_handle_t *vflow,
    camera_handle_t cam_fd,
    hbn_vnode_handle_t vin,
    hbn_vnode_handle_t isp,
    hbn_vnode_handle_t gdc,
    hbn_vnode_handle_t vse, uint32_t vse_chn,
    hbn_vnode_handle_t *out_node, uint32_t *out_chn);

/**
 * @brief Tear down one camera: stop and destroy the flow, destroy the camera, free the GDC binary.
 *
 * The vnodes are not closed one by one here: they were added to the flow, which owns
 * them, so destroying the flow releases them.
 *
 * @note @p vin, @p isp, @p vse, @p gdc and @p reset_gpio are unused (the definition
 *       casts them to void). They stay in the signature so a call site lists the same
 *       resources it passed to the *_open() helpers; the reset GPIO is handled by the
 *       caller, which runs sensor_power() itself once the flow is gone.
 *
 * @param[in] vflow       Flow to stop and destroy; 0 skips the flow.
 * @param[in] cam_fd      Camera handle to destroy; 0 is ignored.
 * @param[in] vin,isp,vse,gdc Unused; see the note above.
 * @param[in,out] gdc_bin Buffer returned by gdc_open(); freed and cleared when its fd is
 *                        set, so a second call cannot free it twice.
 * @param[in] reset_gpio  Unused; see the note above.
 */
void teardown_cam(
    hbn_vflow_handle_t vflow,
    camera_handle_t cam_fd,
    hbn_vnode_handle_t vin, hbn_vnode_handle_t isp,
    hbn_vnode_handle_t vse,
    hbn_vnode_handle_t gdc,
    hb_mem_common_buf_t *gdc_bin, int reset_gpio);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* GS130_PIPELINE_RDKX5_H */
