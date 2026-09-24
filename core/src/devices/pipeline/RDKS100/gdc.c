/**
 * @file gdc.c
 * @brief gdc node: apply +0.5/clamp/rotation to the map, encode it as a cfg, then open the GDC vnode.
 *
 * The map handed in comes either from base::stereo_rectify() or from the identity map
 * built in RDKS100.cpp: an array of RemapPoint{double x, y} entries, reinterpreted here
 * as the driver's point_t{double x, y} so it can be passed to the GDC without a
 * conversion pass. The two layouts therefore have to stay identical -- nothing checks
 * that at compile time, so a change to RemapPoint (src/types.hpp) must be mirrored in
 * point_t.
 *
 * This file is part of gs130_camera_sdk (https://github.com/D-Robotics/gs130_camera_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "RDKS100.h"

#include <stdlib.h>
#include <string.h>

/* GDC strides are 16-byte aligned on S100; the platform's camera stack aligns them the
 * same way (ALIGN_16 in hobot_mipi_cam's create_gdc_node()). */
#define GS130_ALIGN_16(v) (((v) + 15u) & ~15u)

/* Number of driver-held output buffers for the GDC output. */
#define GS130_GDC_BUF_NUM 3

int gdc_open(hbn_vnode_handle_t *gdc, hb_mem_common_buf_t *gdc_bin,
             const void *map, uint32_t in_w, uint32_t in_h,
             uint32_t grid_w, uint32_t grid_h, int install_angle)
{
    const uint32_t npts = grid_w * grid_h;
    const point_t *m = (const point_t *)map;

    // xmax/ymax bound the GDC input frame, which is the sensor frame
    const double xmax = (double)in_w - 1.0;
    const double ymax = (double)in_h - 1.0;
    // the map is generated in the rotated orientation, so for a 90/270 degree install
    // rotation its coordinates span the sensor height; the clamp bounds follow that
    const int    swap     = (install_angle == 90 || install_angle == 270);
    const double src_xmax = swap ? ymax : xmax;
    const double src_ymax = swap ? xmax : ymax;

    point_t *pts = (point_t *)malloc((size_t)npts * sizeof(point_t));
    if (!pts)
        return -1;

    for (uint32_t i = 0; i < npts; ++i) {
        /* GDC bilinear-interpolation compensation: +0.5 sub-pixel, clamp to avoid black edges */
        double x = m[i].x + 0.5;
        double y = m[i].y + 0.5;
        if (x > src_xmax) x = src_xmax;
        if (y > src_ymax) y = src_ymax;
        /* the install rotation is applied to the sampling coordinate, mapping it from
           the rotated map orientation back into the sensor frame the GDC reads */
        switch (install_angle) {
        case 90:  pts[i].x = y;        pts[i].y = ymax - x; break;
        case 180: pts[i].x = xmax - x; pts[i].y = ymax - y; break;
        case 270: pts[i].x = xmax - y; pts[i].y = x;        break;
        default:  pts[i].x = x;        pts[i].y = y;        break;
        }
    }

    param_t param = {
        .format = FMT_SEMIPLANAR_420,   /* NV12, matching the ISP output */
        .in = { .w = in_w, .h = in_h },      /* GDC input geometry, in pixels */
        .out = { .w = grid_w, .h = grid_h },  /* output grid = map size, in pixels */
        .fov = 180.0,                   /* generator parameter retained for the custom map */
        .diameter = in_h,               /* generator diameter in input pixels */
    };

    window_t win = {
        .transform = CUSTOM,            /* use the generated map, not a built-in transform */
        .strength = 1.0,
        .strengthY = 1.0,
        .keep_ratio = 1,
        .FOV_h = 90.0,                  /* generator fields used with CUSTOM mapping */
        .FOV_w = 90.0,
        .trapezoid_left_angle = 90.0,
        .trapezoid_right_angle = 90.0,
        .out_r = { .w = grid_w, .h = grid_h },
        .input_roi_r = { .w = in_w, .h = in_h },   /* whole input frame, in pixels */
        .zoom = 1.0,
        .custom = {
            .full_tile_calc = 1,
            .tile_incr_x = 50,
            .tile_incr_y = 50,
            .w = (int32_t)grid_w - 1,       /* grid size minus one, as the map is */
            .h = (int32_t)grid_h - 1,       /* indexed from 0 */
            .centerx = (double)(grid_w / 2),   /* map center, in grid coordinates */
            .centery = (double)(grid_h / 2),
            .points = pts,                  /* generated map, npts entries */
        },
    };

    /* S100 hands the buffer back as void*, X5 as uint32_t*; the mapping itself is built
       by the shared hbn_gen_gdc_cfg()/hbn_gen_gdc_bin() code underneath. */
    void    *cfg_buf  = NULL;
    uint64_t cfg_size = 0;
    if (hbn_gen_gdc_cfg(&param, &win, 1, &cfg_buf, &cfg_size) != 0 ||
        cfg_buf == NULL || cfg_size == 0) {
        /* Freed through the platform's entry point, not free(): the buffer comes
           from the GDC generator, so it must go back to it. */
        if (cfg_buf != NULL)
            hbn_free_gdc_cfg((uint32_t *)cfg_buf);
        free(pts);
        return -1;
    }

    const int64_t flags = HB_MEM_USAGE_MAP_INITIALIZED |
                          HB_MEM_USAGE_PRIV_HEAP_2_RESERVERD |
                          HB_MEM_USAGE_CPU_READ_OFTEN |
                          HB_MEM_USAGE_CPU_WRITE_OFTEN |
                          HB_MEM_USAGE_CACHED;
    memset(gdc_bin, 0, sizeof(*gdc_bin));
    if (hb_mem_alloc_com_buf(cfg_size, flags, gdc_bin) != 0 ||
        gdc_bin->virt_addr == NULL) {
        hbn_free_gdc_cfg((uint32_t *)cfg_buf);
        free(pts);
        return -1;
    }
    memcpy(gdc_bin->virt_addr, cfg_buf, (size_t)cfg_size);
    hbn_free_gdc_cfg((uint32_t *)cfg_buf);
    free(pts);

    if (hb_mem_flush_buf(gdc_bin->fd, 0, cfg_size) != 0)
        return -1;

    /* Node attributes: geometry and the configuration binary, plus the marker the
       driver checks. */
    gdc_settings_t settings = { 0 };
    settings.gdc_config.config_addr   = gdc_bin->phys_addr;
    settings.gdc_config.config_size   = (uint32_t)gdc_bin->size;
    settings.gdc_config.input_width   = in_w;
    settings.gdc_config.input_height  = in_h;
    settings.gdc_config.input_stride  = GS130_ALIGN_16(in_w);
    settings.gdc_config.output_width  = grid_w;
    settings.gdc_config.output_height = grid_h;
    settings.gdc_config.output_stride = GS130_ALIGN_16(grid_w);
    settings.gdc_config.total_planes  = 2;   /* NV12: luma + interleaved chroma */
    settings.binary_ion_id            = gdc_bin->share_id;
    settings.binary_offset            = gdc_bin->offset;
    settings.magicNumber              = GS130_S100_MAGIC_NUMBER;

    if (hbn_vnode_open(HB_GDC, 0, AUTO_ALLOC_ID, gdc) != 0) {
        *gdc = 0;
        return -1;
    }

    /* As with PYM, one structure describes the node and both of its channels. */
    if (hbn_vnode_set_attr(*gdc, &settings) != 0 ||
        hbn_vnode_set_ichn_attr(*gdc, 0, &settings) != 0 ||
        hbn_vnode_set_ochn_attr(*gdc, 0, &settings) != 0)
        return -1;

    hbn_buf_alloc_attr_t alloc = {
        .buffers_num = GS130_GDC_BUF_NUM,
        .is_contig = 1,
        .flags = HB_MEM_USAGE_CPU_READ_OFTEN |
                 HB_MEM_USAGE_CPU_WRITE_OFTEN |
                 HB_MEM_USAGE_CACHED,
    };
    if (hbn_vnode_set_ochn_buf_attr(*gdc, 0, &alloc) != 0)
        return -1;
    return 0;
}
