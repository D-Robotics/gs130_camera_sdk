/**
 * @file RDKS100.h
 * @brief RDK S100 node helpers: one C function per pipeline stage, plus teardown.
 *
 * Internal header of the RDK S100 backend, included by RDKS100.cpp and the node .c files
 * next to it, never by platform-independent code, because it pulls in the platform's
 * driver headers.
 *
 * Two things are easy to miss on this platform:
 *
 *   - VIN, ISP, PYM and GDC each take one wrapper configuration struct (vin_attr_t,
 *     isp_cfg_t, pym_cfg_t, gdc_settings_t) rather than separate input, output and
 *     buffer-attribute structs.
 *   - Several of those structs carry a magicNumber field the driver checks; see
 *     GS130_S100_MAGIC_NUMBER.
 *
 * Handles and memory-manager buffers are reported through out-parameters and belong to
 * the caller after creation; partially configured handles may remain non-zero
 * on a later setup failure and must be cleaned up by the backend. GDC buffers are freed
 * by teardown_cam().
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
 * The value is not part of the platform headers; the platform's camera stack defines
 * it locally (hobot_mipi_cam, vp_sensors.h: `#define MAGIC_NUMBER 0x12345678`).
 */
#define GS130_S100_MAGIC_NUMBER 0x12345678u

/** Run the sensor reset sequence on one GPIO through sysfs. A negative gpio disables
 *  the control and the call succeeds. */
int sensor_power(int gpio, int on);

/** Create the camera device for one sensor: MIPI receiver and sensor config.
 *  tuning_file is borrowed and not retained; NULL selects "disable". */
int camera_open(
    camera_handle_t *cam_fd,
    uint8_t i2c_addr,
    uint32_t width, uint32_t height, uint32_t fps,
    uint32_t line_length, uint32_t frame_length,
    uint16_t mipiclk, uint16_t settle, uint16_t mclk,
    const char *tuning_file);

/** Open the VIN node: MIPI capture (CIM) plus the LPWM frame trigger. The LPWM period
 *  comes from fps and is the SDK's IMU FSYNC time base. */
int vin_open(
    hbn_vnode_handle_t *vin,
    int mipi_rx,
    uint32_t width, uint32_t height, uint32_t fps);

/** Open the ISP node with offline/DDR YUV420 output on channel 0. */
int isp_open(
    hbn_vnode_handle_t *isp,
    uint32_t width, uint32_t height,
    uint32_t hw_id, uint32_t slot_id, uint32_t fps);

/** Check that the aspect-preserving ROI of an in/out size pair divides exactly, so
 *  the integer division in aspect_roi() truncates nothing. */
int roi_ratio_exact(uint32_t in_w, uint32_t in_h,
                    uint32_t out_w, uint32_t out_h);

/** A pixel rectangle in input coordinates, as used by aspect_roi(). */
typedef struct {
    uint32_t x;
    uint32_t y;
    uint32_t w;
    uint32_t h;
} gs130_rect_t;

/** Crop a centered ROI from 'in' with the aspect ratio of 'out'. Validate the result
 *  with roi_ratio_exact() before using it. */
gs130_rect_t aspect_roi(uint32_t in_w, uint32_t in_h,
                        uint32_t out_w, uint32_t out_h);

/** Open the PYM node: aspect-preserving ROI crop, then scaling. Uses PYM_M2M_MODE
 *  because ISP and GDC hand it DDR-backed frames over direct vnode binds. */
int pym_open(
    hbn_vnode_handle_t *pym,
    uint32_t in_w, uint32_t in_h,
    uint32_t out_w, uint32_t out_h,
    uint32_t hw_id, uint32_t slot_id);

/** Open the GDC node from a generated map: every point offset by +0.5 px, clamped to
 *  the source bounds, then rotated by install_angle. The map has to match the driver's
 *  point_t{double x, y}; nothing checks that at compile time. */
int gdc_open(
    hbn_vnode_handle_t *gdc,
    hb_mem_common_buf_t *gdc_bin,
    const void *map, uint32_t in_w, uint32_t in_h,
    uint32_t grid_w, uint32_t grid_h, int install_angle);

/** Create the vflow, add and bind the nodes, and report where frames come out. A zero
 *  handle skips a stage; gdc_before_pym selects the order of the last two. */
int vflow_build(hbn_vflow_handle_t *vflow, camera_handle_t cam_fd,
                hbn_vnode_handle_t vin, hbn_vnode_handle_t isp,
                hbn_vnode_handle_t gdc, hbn_vnode_handle_t pym,
                uint32_t pym_chn, int gdc_before_pym,
                hbn_vnode_handle_t *out_node, uint32_t *out_chn);

/** Tear down one camera: stop and destroy the flow, destroy the camera, free the GDC
 *  binary. The flow owns the vnodes, so they are not closed individually. */
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
