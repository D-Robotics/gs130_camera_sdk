/**
 * @file vin.c
 * @brief vin node: MIPI capture (CIM) plus the LPWM exposure trigger.
 *
 * The LPWM channels share one frame period and one pulse shape, so both eyes are
 * triggered in lockstep. That period is also the master time base of the SDK's IMU
 * FSYNC tracker (see src/gs130.cpp), which is why it is derived from the configured
 * frame rate rather than passed in by the caller.
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

/* RAW10, the sensor's output format and therefore VIN's input and output format. */
#define GS130_VIN_RAW10 0x2B

/*
 * VCON bus selection. Taken from the platform's S100 configuration for this exact
 * sensor (hobot_mipi_cam, src/s100/sensor/sc132gs_linear_1088x1280_raw10_30fps_1lane.c),
 * which sets both to 2. The platform's camera stack does not derive these from the device
 * tree, so they have to be stated; whether this board uses the same VCON wiring has not
 * been checked on hardware.
 */
#define GS130_VIN_VCON_BUS_MAIN   2
#define GS130_VIN_VCON_BUS_SECOND 2

/* Number of driver-held capture buffers for the VIN output; the platform's S100
 * configuration uses 6 for this sensor. */
#define GS130_VIN_BUF_NUM 6

/* Bytes per line of the RAW10 output, aligned up to the DMA engine's 32-byte plane
 * boundary. The ISP derives the stride of its own input the same way
 * (AFRAME_ALIGN_PLANE(width * 10 / 8) in the ISP driver's isp_video_reqbufs_src) and
 * refuses to bind when the two disagree, so this has to be the aligned packed size and
 * not a padded bytes-per-pixel count. For the 1088-wide sensor that is 1088 * 10 / 8 =
 * 1360, rounded up to 1376. */
#define GS130_VIN_WSTRIDE_ALIGN 32U

int vin_open(hbn_vnode_handle_t *vin, int mipi_rx,
             uint32_t width, uint32_t height, uint32_t fps)
{
    const uint32_t period_us = 1000000U / fps;   /* LPWM period = one frame, in us */
    /* Packed RAW10 line, then rounded up to the plane alignment the ISP checks against. */
    const uint32_t ochn_wstride =
        (((width * 10U / 8U) + GS130_VIN_WSTRIDE_ALIGN - 1U) / GS130_VIN_WSTRIDE_ALIGN)
        * GS130_VIN_WSTRIDE_ALIGN;

    vin_attr_t attr = { 0 };

    /* Which MIPI/virtual-connection bus this sensor sits on. */
    attr.vin_node_attr.vcon_attr.bus_main   = GS130_VIN_VCON_BUS_MAIN;
    attr.vin_node_attr.vcon_attr.bus_second = GS130_VIN_VCON_BUS_SECOND;

    /* Capture side. mipi_en turns the receiver on, cim_isp_flyby = 0 keeps VIN offline
       (it writes to DDR and the ISP reads from there), and cim_pym_flyby = 0 keeps the
       same for the scaling node. */
    attr.vin_node_attr.cim_attr.mipi_en       = 1;
    attr.vin_node_attr.cim_attr.cim_isp_flyby = 0;
    attr.vin_node_attr.cim_attr.cim_pym_flyby = 0;
    attr.vin_node_attr.cim_attr.mipi_rx       = (uint32_t)mipi_rx;
    attr.vin_node_attr.cim_attr.vc_index      = 0;   /* virtual channel, matching camera.c */
    attr.vin_node_attr.cim_attr.ipi_channels  = 1;
    attr.vin_node_attr.cim_attr.y_uv_swap     = 0;   /* RAW10: no luma/chroma swap */

    /* Frame bookkeeping. The counter starts at 0 and every frame carries an id. */
    attr.vin_node_attr.cim_attr.func.enable_frame_id   = 1;
    attr.vin_node_attr.cim_attr.func.set_init_frame_id = 0;
    attr.vin_node_attr.cim_attr.func.enable_pattern    = 0;   /* test pattern off */

    /* No RDMA input: the frames come from the MIPI receiver, not from memory. */
    attr.vin_node_attr.cim_attr.rdma_input.rdma_en = 0;

    attr.vin_node_attr.magicNumber = GS130_S100_MAGIC_NUMBER;

    /* LPWM exposure trigger: all four channels use the same period and the same raw
       driver offset/duty values, so their outputs share one time base. */
    for (uint32_t ch = 0; ch < 4; ch++) {
        attr.vin_node_attr.lpwm_attr.lpwm_chn_attr[ch].enable        = 1;
        attr.vin_node_attr.lpwm_attr.lpwm_chn_attr[ch].trigger_source = 0;
        attr.vin_node_attr.lpwm_attr.lpwm_chn_attr[ch].trigger_mode   = 0;
        attr.vin_node_attr.lpwm_attr.lpwm_chn_attr[ch].period         = period_us;
        attr.vin_node_attr.lpwm_attr.lpwm_chn_attr[ch].offset         = 10;
        attr.vin_node_attr.lpwm_attr.lpwm_chn_attr[ch].duty_time      = 1000;
        attr.vin_node_attr.lpwm_attr.lpwm_chn_attr[ch].threshold      = 0;
        attr.vin_node_attr.lpwm_attr.lpwm_chn_attr[ch].adjust_step    = 0;
    }

    /* Input channel: the frame the sensor produces. */
    attr.vin_ichn_attr.width  = width;
    attr.vin_ichn_attr.height = height;
    attr.vin_ichn_attr.format = GS130_VIN_RAW10;

    /* Output channel: written to DDR, RAW10 with a 2-byte-per-pixel stride. The number
       of buffers lives in a separate member of the same wrapper. */
    attr.vin_ochn_attr[VIN_MAIN_FRAME].ddr_en = 1;
    attr.vin_ochn_attr[VIN_MAIN_FRAME].vin_basic_attr.format    = GS130_VIN_RAW10;
    attr.vin_ochn_attr[VIN_MAIN_FRAME].vin_basic_attr.wstride   = ochn_wstride;
    attr.vin_ochn_attr[VIN_MAIN_FRAME].vin_basic_attr.pack_mode = 1;
    attr.vin_ochn_attr[VIN_MAIN_FRAME].pingpong_ring = 1;
    attr.vin_ochn_attr[VIN_MAIN_FRAME].magicNumber   = GS130_S100_MAGIC_NUMBER;

    attr.vin_ochn_buff_attr[VIN_MAIN_FRAME].buffers_num = GS130_VIN_BUF_NUM;

    attr.magicNumber = GS130_S100_MAGIC_NUMBER;

    /* The node's hw_id is the MIPI receiver index, as in the platform's camera stack. */
    if (hbn_vnode_open(HB_VIN, (uint32_t)mipi_rx, AUTO_ALLOC_ID, vin) != 0) {
        *vin = 0;
        return -1;
    }
    if (hbn_vnode_set_attr(*vin, &attr) != 0 ||
        hbn_vnode_set_ichn_attr(*vin, 0, &attr.vin_ichn_attr) != 0 ||
        hbn_vnode_set_ochn_attr(*vin, 0, &attr.vin_ochn_attr[VIN_MAIN_FRAME]) != 0)
        return -1;

    hbn_buf_alloc_attr_t alloc = {
        .buffers_num = GS130_VIN_BUF_NUM,
        .is_contig = 1,
        .flags = HB_MEM_USAGE_CPU_READ_OFTEN |
                 HB_MEM_USAGE_CPU_WRITE_OFTEN |
                 HB_MEM_USAGE_CACHED,
    };
    if (hbn_vnode_set_ochn_buf_attr(*vin, 0, &alloc) != 0)
        return -1;
    return 0;
}
