/**
 * @file flow.c
 * @brief flow node: vflow build/bind and camera teardown helpers.
 *
 * This file is part of gs130_camera_sdk (https://github.com/D-Robotics/gs130_camera_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "RDKS100.h"

#include <string.h>

int vflow_build(hbn_vflow_handle_t *vflow, camera_handle_t cam_fd,
                hbn_vnode_handle_t vin, hbn_vnode_handle_t isp,
                hbn_vnode_handle_t gdc, hbn_vnode_handle_t pym,
                uint32_t pym_chn,
                hbn_vnode_handle_t *out_node, uint32_t *out_chn)
{
    if (hbn_vflow_create(vflow) != 0) {
        *vflow = 0;
        return -1;
    }
    if (hbn_vflow_add_vnode(*vflow, vin) != 0 ||
        hbn_vflow_add_vnode(*vflow, isp) != 0 ||
        (gdc != 0 && hbn_vflow_add_vnode(*vflow, gdc) != 0) ||
        (pym != 0 && hbn_vflow_add_vnode(*vflow, pym) != 0))
        return -1;

    // vin -> isp -> [gdc] -> [pym]
    if(hbn_vflow_bind_vnode(*vflow, vin, 0, isp, 0) != 0)return -1;
    if(gdc != 0 && hbn_vflow_bind_vnode(*vflow, isp, 0, gdc, 0) != 0)return -1;
    if(pym != 0){
        const hbn_vnode_handle_t src = (gdc != 0) ? gdc : isp;
        if(hbn_vflow_bind_vnode(*vflow, src, 0, pym, 0) != 0) return -1;
        *out_node = pym;
        *out_chn  = pym_chn;
    }
    else if(gdc != 0){
        *out_node = gdc;
        *out_chn  = 0;
    }
    else{
        // Direct ISP output. S100 has no ISP_MAIN_FRAME symbol; the ISP's single
        // output channel is channel 0, the same channel the configuration used.
        *out_node = isp;
        *out_chn  = 0;
    }

    if(hbn_camera_attach_to_vin(cam_fd, vin) != 0)return -1;

    return 0;
}

void teardown_cam(hbn_vflow_handle_t vflow, camera_handle_t cam_fd,
                  hbn_vnode_handle_t vin, hbn_vnode_handle_t isp,
                  hbn_vnode_handle_t pym, hbn_vnode_handle_t gdc,
                  hb_mem_common_buf_t *gdc_bin, int reset_gpio)
{
    (void)vin; (void)isp; (void)pym; (void)gdc; (void)reset_gpio;

    if(vflow != 0){
        hbn_vflow_stop(vflow);
        hbn_vflow_destroy(vflow);
    }

    if(cam_fd != 0)hbn_camera_destroy(cam_fd);

    if(gdc_bin != 0 && gdc_bin->fd != 0){
        hb_mem_free_buf(gdc_bin->fd);
        memset(gdc_bin, 0, sizeof(*gdc_bin));
    }
}
