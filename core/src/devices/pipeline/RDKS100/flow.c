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
                uint32_t pym_chn, int gdc_before_pym,
                hbn_vnode_handle_t *out_node, uint32_t *out_chn)
{
    if (hbn_vflow_create(vflow) != 0) {
        *vflow = 0;
        return -1;
    }
    // A handle of 0 means the stage is not part of this flow. Raw has an ISP but no GDC or
    // PYM; Resize has PYM only; Rect has both GDC and PYM.
    if (hbn_vflow_add_vnode(*vflow, vin) != 0 ||
        (isp != 0 && hbn_vflow_add_vnode(*vflow, isp) != 0) ||
        (gdc != 0 && hbn_vflow_add_vnode(*vflow, gdc) != 0) ||
        (pym != 0 && hbn_vflow_add_vnode(*vflow, pym) != 0))
        return -1;

    /*
     * Two orders, chosen by the caller, after vin -> isp:
     *
     *   gdc_before_pym == 0:  isp -> pym -> [gdc]
     *     PYM scales (it is this platform's VSE equivalent; there is no VSE node on S100)
     *     and the GDC, when present, only transforms the frame it is handed. Used by
     *     Resize, including the install-rotation case, and by any flow without a GDC.
     *
     *   gdc_before_pym == 1:  isp -> gdc -> pym
     *     The GDC resamples the sensor frame through the rectification table onto the
     *     rectified grid, which is NOT the sensor size: stereo_rectify() grows the grid
     *     until the black borders disappear (1088x1280 -> 1088x1536 on this module), so
     *     the GDC changes the size. PYM then takes an aspect-preserving window of that
     *     rectified frame and scales it to the requested output, which is what makes PYM
     *     (not the GDC) this path's scaling stage.
     */
    if(isp != 0 && hbn_vflow_bind_vnode(*vflow, vin, 0, isp, 0) != 0)return -1;
    if(pym != 0 && isp != 0 && gdc_before_pym && gdc != 0){
        if(hbn_vflow_bind_vnode(*vflow, isp, 0, gdc, 0) != 0) return -1;
        if(hbn_vflow_bind_vnode(*vflow, gdc, 0, pym, 0) != 0) return -1;
        *out_node = pym;
        *out_chn  = pym_chn;
    }
    else if(pym != 0 && isp != 0){
        /* All modes use the ISP's offline/DDR YUV420 output on channel 0. Resize binds
           it directly to PYM; the install-rotation variant appends GDC after PYM. */
        if(hbn_vflow_bind_vnode(*vflow, isp, 0, pym, 0) != 0) return -1;
        if(gdc != 0){
            if(hbn_vflow_bind_vnode(*vflow, pym, pym_chn, gdc, 0) != 0) return -1;
            *out_node = gdc;
            *out_chn  = 0;
        }
        else{
            *out_node = pym;
            *out_chn  = pym_chn;
        }
    }
    else if(gdc != 0){
        *out_node = gdc;
        *out_chn  = 0;
    }
    else if(isp != 0){
        // Direct ISP output. S100 has no ISP_MAIN_FRAME symbol; the ISP's single
        // output channel is channel 0, the same channel the configuration used.
        *out_node = isp;
        *out_chn  = 0;
    }
    else{
        // No ISP in the flow: this fallback reads the capture node itself. Current public
        // modes all instantiate ISP; Raw normally reaches the ISP branch above.
        *out_node = vin;
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
