/**
 * @file isp.c
 * @brief isp node: offline (DDR) ISP, NV12 output.
 *
 * RAW10 frames arriving from VIN in DDR are processed here and handed on as NV12,
 * which is the format every later stage (GDC, VSE) and the caller's buffers use.
 *
 * The function contract (parameters, units, ownership, return value) is documented
 * with the declaration in RDKX5.h.
 *
 * This file is part of gs130_camera_sdk (https://github.com/D-Robotics/gs130_camera_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "RDKX5.h"

#include <string.h>

int isp_open(hbn_vnode_handle_t *isp, uint32_t width, uint32_t height)
{
    isp_attr_t attr = {
        .input_mode = DDR_MODE,      /* offline (DDR): the ISP reads the frames VIN wrote */
        .sensor_mode = ISP_NORMAL_M, /* non-HDR sensor mode */
        .crop = { .x = 0, .y = 0, .w = width, .h = height },   /* full frame, in pixels */
    };

    isp_ichn_attr_t ichn = {
        .width = width,              /* input frame size in pixels */
        .height = height,
        .fmt = FRM_FMT_RAW,          /* RAW10 input, matching VIN and camera.c */
        .bit_width = 10,
    };

    isp_ochn_attr_t ochn = {
        .ddr_en = CAM_TRUE,          /* output is written to DDR */
        .fmt = FRM_FMT_NV12,         /* NV12 out, 8 bit per component */
        .bit_width = 8,
    };

    if (hbn_vnode_open(HB_ISP, 0, AUTO_ALLOC_ID, isp) != 0) {
        *isp = 0;
        return -1;
    }
    if (hbn_vnode_set_attr(*isp, &attr) != 0 ||
        hbn_vnode_set_ochn_attr(*isp, ISP_MAIN_FRAME, &ochn) != 0 ||
        hbn_vnode_set_ichn_attr(*isp, 0, &ichn) != 0)
        return -1;

    hbn_buf_alloc_attr_t alloc = {
        .buffers_num = 5,        /* driver-held output buffers for the ISP main frame */
        .is_contig = 1,
        .flags = HB_MEM_USAGE_CPU_READ_OFTEN |
                 HB_MEM_USAGE_CPU_WRITE_OFTEN |
                 HB_MEM_USAGE_CACHED |
                 HB_MEM_USAGE_GRAPHIC_CONTIGUOUS_BUF,
    };
    if (hbn_vnode_set_ochn_buf_attr(*isp, ISP_MAIN_FRAME, &alloc) != 0)
        return -1;
    return 0;
}
