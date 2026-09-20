/**
 * @file pym.c
 * @brief pym node: aspect-preserving crop + scaling.
 *
 * S100's counterpart of the X5 VSE node. PYM is a pyramid scaler: the source layer plus
 * up to MAX_DS_NUM-1 downscaled layers can be described at once, and each enabled output
 * slot names its source layer and the region to take from it. This backend enables a
 * single slot on the source layer (ds_roi_sel 0 / ds_roi_layer 0), so the region is
 * expressed in full-resolution sensor coordinates.
 *
 * The scaling semantics are deliberately the X5 ones: aspect_roi() picks a centered crop
 * of the input with the output's aspect ratio and that crop becomes the source region, so
 * the image is never stretched. The platform's own camera stack instead passes the whole
 * pyramid base layer as the region and lets the requested size be the output, which
 * stretches when the aspect ratios differ; see RDKS100.h for why the crop is kept here.
 *
 * Unlike VSE, PYM takes one configuration structure for the node attribute, the input
 * channel and the output channel; the same pym_cfg_t is passed to all three setters.
 *
 * Buffer counts, handshaking flags and the horizontal/vertical blanking values follow the
 * platform's S100 configuration for this exact sensor (hobot_mipi_cam,
 * src/s100/sensor/sc132gs_linear_1088x1280_raw10_30fps_1lane.c).
 *
 * This file is part of gs130_camera_sdk (https://github.com/D-Robotics/gs130_camera_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "RDKS100.h"

#include <string.h>

/* PYM strides are 16-byte aligned, the same alignment the platform's camera stack uses
 * for GDC and PYM (ALIGN_16 in hobot_mipi_cam). */
#define GS130_ALIGN_16(v) (((v) + 15u) & ~15u)

#define GS130_PYM_BL_MAX_EN      5
#define GS130_PYM_SUFFIX_HB      68
#define GS130_PYM_PREFIX_HB      2
#define GS130_PYM_SUFFIX_VB      20
#define GS130_PYM_PREFIX_VB      2
#define GS130_PYM_OUTPUT_BUF_NUM 6
#define GS130_PYM_FB_BUF_NUM     2
#define GS130_PYM_BUF_NUM        3   /* buffers the caller's output channel is given */

int roi_ratio_exact(uint32_t in_w, uint32_t in_h,
                    uint32_t out_w, uint32_t out_h)
{
    if (in_w * out_h > out_w * in_h) {
        // input is relatively wider: crop left/right, roi.w = in_h * out_w / out_h
        return (in_h * out_w % out_h == 0) ? 0 : -1;
    }
    // crop top/bottom, roi.h = in_w * out_h / out_w
    return (in_w * out_h % out_w == 0) ? 0 : -1;
}

gs130_rect_t aspect_roi(uint32_t in_w, uint32_t in_h,
                        uint32_t out_w, uint32_t out_h)
{
    gs130_rect_t roi;
    if (in_w * out_h > out_w * in_h) {
        // input is relatively wider: crop left/right
        roi.h = in_h;
        roi.w = in_h * out_w / out_h;
        roi.x = (in_w - roi.w) / 2;
        roi.y = 0;
    } else {
        // crop top/bottom
        roi.w = in_w;
        roi.h = in_w * out_h / out_w;
        roi.x = 0;
        roi.y = (in_h - roi.h) / 2;
    }
    return roi;
}

int pym_open(hbn_vnode_handle_t *pym, uint32_t in_w, uint32_t in_h,
             uint32_t out_w, uint32_t out_h, uint32_t hw_id, uint32_t slot_id)
{
    const gs130_rect_t roi = aspect_roi(in_w, in_h, out_w, out_h);

    pym_cfg_t cfg = { 0 };

    /* Node identity: which PYM instance and slot this eye uses, in explicit
       (non-fly-by) mode. */
    cfg.hw_id    = hw_id;
    cfg.slot_id  = slot_id;
    cfg.pym_mode = PYM_MANUAL_MODE;

    /* Buffer counts and handshaking, as in the platform's S100 configuration. */
    cfg.pingpong_ring        = 0;
    cfg.output_buf_num       = GS130_PYM_OUTPUT_BUF_NUM;
    cfg.fb_buf_num           = GS130_PYM_FB_BUF_NUM;
    cfg.timeout              = 0;
    cfg.threshold_time       = 0;
    cfg.layer_num_trans_next = 0;
    cfg.layer_num_share_prev = -1;
    cfg.out_buf_noinvalid    = 1;
    cfg.out_buf_noncached    = 0;
    cfg.in_buf_noclean       = 1;
    cfg.in_buf_noncached     = 0;

    /* Input geometry: the full frame handed over by the ISP or GDC, NV12, 16-byte
       aligned strides. */
    cfg.chn_ctrl.invalid_head_lines  = 0;
    cfg.chn_ctrl.src_in_width        = in_w;
    cfg.chn_ctrl.src_in_height       = in_h;
    cfg.chn_ctrl.src_in_stride_y     = GS130_ALIGN_16(in_w);
    cfg.chn_ctrl.src_in_stride_uv    = GS130_ALIGN_16(in_w);
    cfg.chn_ctrl.suffix_hb_val       = GS130_PYM_SUFFIX_HB;
    cfg.chn_ctrl.prefix_hb_val       = GS130_PYM_PREFIX_HB;
    cfg.chn_ctrl.suffix_vb_val       = GS130_PYM_SUFFIX_VB;
    cfg.chn_ctrl.prefix_vb_val       = GS130_PYM_PREFIX_VB;
    cfg.chn_ctrl.bl_max_layer_en     = GS130_PYM_BL_MAX_EN;
    cfg.chn_ctrl.ds_roi_uv_bypass    = 0;

    /*
     * One output, taken from the source layer. ds_roi_sel 0 / ds_roi_layer 0 means "the
     * full-resolution layer", which is why the region below is given in sensor
     * coordinates; the disabled slots keep their selector/layer at the platform's
     * defaults and are gated off by ds_roi_en.
     */
    cfg.chn_ctrl.ds_roi_sel[0]   = 0;
    cfg.chn_ctrl.ds_roi_layer[0] = 0;
    cfg.chn_ctrl.ds_roi_en       = (uint8_t)(1u << 0);

    roi_box_t *box = &cfg.chn_ctrl.ds_roi_info[0];
    box->start_left    = roi.x;      /* centered crop, the X5 VSE behaviour */
    box->start_top     = roi.y;
    box->region_width  = roi.w;
    box->region_height = roi.h;
    box->out_width     = out_w;
    box->out_height    = out_h;
    box->wstride_y     = GS130_ALIGN_16(out_w);
    box->wstride_uv    = GS130_ALIGN_16(out_w);
    box->vstride       = out_h;

    for (uint32_t i = 1; i < MAX_DS_NUM; i++) {
        cfg.chn_ctrl.ds_roi_sel[i]   = 1;
        cfg.chn_ctrl.ds_roi_layer[i] = (uint8_t)(i - 1);
    }

    cfg.magicNumber = GS130_S100_MAGIC_NUMBER;

    if (hbn_vnode_open(HB_PYM, cfg.hw_id, AUTO_ALLOC_ID, pym) != 0) {
        *pym = 0;
        return -1;
    }
    /*
     * PYM takes one configuration structure for the node attribute, the input channel
     * and the output channel; the platform camera stack passes the same pym_cfg_t to all
     * three. VSE on X5 uses three distinct structures here, so this is deliberately not
     * a line-for-line copy of vse.c.
     */
    if (hbn_vnode_set_attr(*pym, &cfg) != 0 ||
        hbn_vnode_set_ichn_attr(*pym, 0, &cfg) != 0 ||
        hbn_vnode_set_ochn_attr(*pym, 0, &cfg) != 0)
        return -1;

    hbn_buf_alloc_attr_t alloc = {
        .buffers_num = GS130_PYM_BUF_NUM,
        .is_contig = 1,
        .flags = HB_MEM_USAGE_CPU_READ_OFTEN |
                 HB_MEM_USAGE_CPU_WRITE_OFTEN |
                 HB_MEM_USAGE_CACHED |
                 HB_MEM_USAGE_GRAPHIC_CONTIGUOUS_BUF,
    };
    if (hbn_vnode_set_ochn_buf_attr(*pym, 0, &alloc) != 0)
        return -1;
    return 0;
}
