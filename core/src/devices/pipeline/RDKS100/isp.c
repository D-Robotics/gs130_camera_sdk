/**
 * @file isp.c
 * @brief isp node: NV12 output written to DDR for the downstream node.
 *
 * RAW10 frames arriving from VIN are processed here and handed on as NV12, which is the
 * format every later stage (PYM, GDC) and the caller's buffers use.
 *
 * Every mode reads the offline/DDR output, so stream output is disabled and AXI YUV420 is
 * enabled.
 *
 * The function contract (parameters, units, ownership, return value) is documented
 * with the declaration in RDKS100.h.
 *
 * This file is part of gs130_camera_sdk (https://github.com/D-Robotics/gs130_camera_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "RDKS100.h"

#include <string.h>

/* Number of driver-held output buffers for the ISP main frame; the platform's S100
 * camera stack uses 3 for the offline path. */
#define GS130_ISP_BUF_NUM 3

int isp_open(hbn_vnode_handle_t *isp, uint32_t width, uint32_t height,
             uint32_t hw_id, uint32_t slot_id, uint32_t fps)
{
    isp_cfg_t cfg = { 0 };

    /*
     * Node identity and scheduling. hw_id / slot_id identify which ISP instance and
     * which of its slots this eye uses; ctx_id stays AUTO_ALLOC_ID, the same value
     * passed as the context argument to hbn_vnode_open() below.
     */
    cfg.isp_attr.channel.hw_id   = hw_id;
    cfg.isp_attr.channel.slot_id = slot_id;
    cfg.isp_attr.channel.ctx_id  = AUTO_ALLOC_ID;

    cfg.isp_attr.work_mode  = 0;
    /* Non-HDR (linear) sensor mode: the value the platform's S100 configuration
       sets for this sensor. */
    cfg.isp_attr.hdr_mode   = 1;
    cfg.isp_attr.size.width  = width;
    cfg.isp_attr.size.height = height;
    cfg.isp_attr.frame_rate  = fps;
    cfg.isp_attr.sched_mode  = SCHED_MODE_MANUAL;
    cfg.isp_attr.algo_state  = 1;

    /* The ISP runs on its own, not combined with a second instance. */
    cfg.isp_attr.isp_combine.isp_channel_mode = ISP_CHANNEL_MODE_NORMAL;

    /* Statistics buffers the 3A algorithms read; the platform's S100 configuration
       enables these three and leaves the remaining two off. */
    cfg.isp_attr.isp_sw_ctrl.ae_stat_buf_en    = 1;
    cfg.isp_attr.isp_sw_ctrl.awb_stat_buf_en   = 1;
    cfg.isp_attr.isp_sw_ctrl.ae5bin_stat_buf_en = 1;
    cfg.isp_attr.isp_sw_ctrl.ctx_buf_en        = 0;
    cfg.isp_attr.isp_sw_ctrl.pixel_consistency_en = 0;

    /* Both crop stages are disabled: the ISP processes the whole frame it is given.
       Cropping is the scaling node's job. */
    cfg.ichn_attr.input_crop_cfg.enable  = 0;
    cfg.ichn_attr.in_buf_noclean         = 1;
    cfg.ichn_attr.in_buf_noncached       = 0;

    cfg.ochn_attr.output_crop_cfg.enable = 0;
    cfg.ochn_attr.out_buf_noinvalid      = 1;
    cfg.ochn_attr.out_buf_noncached      = 0;
    cfg.ochn_attr.output_raw_level       = 0;   /* ISP_OUTPUT_RAW_LEVEL_SENSOR_DATA */
    cfg.ochn_attr.buf_num                = GS130_ISP_BUF_NUM;

    cfg.ochn_attr.stream_output_mode = STREAM_OUTPUT_MODE_DISABLE;
    cfg.ochn_attr.axi_output_mode    = AXI_OUTPUT_MODE_YUV420;

    if (hbn_vnode_open(HB_ISP, hw_id, AUTO_ALLOC_ID, isp) != 0) {
        *isp = 0;
        return -1;
    }
    if (hbn_vnode_set_attr(*isp, &cfg) != 0 ||
        hbn_vnode_set_ochn_attr(*isp, 0, &cfg.ochn_attr) != 0 ||
        hbn_vnode_set_ichn_attr(*isp, 0, &cfg.ichn_attr) != 0)
        return -1;

    hbn_buf_alloc_attr_t alloc_attr = { 0 };
    alloc_attr.buffers_num = GS130_ISP_BUF_NUM;
    alloc_attr.is_contig = 1;
    alloc_attr.flags = HB_MEM_USAGE_CPU_READ_OFTEN |
                       HB_MEM_USAGE_CPU_WRITE_OFTEN |
                       HB_MEM_USAGE_CACHED;
    if (hbn_vnode_set_ochn_buf_attr(*isp, 0, &alloc_attr) != 0)
        return -1;

    return 0;
}
