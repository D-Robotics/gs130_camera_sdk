/**
 * @file gs130-imu-info.c
 * @brief Probe the IMU and print its model and details
 *
 * usage: gs130-imu-info <device> <mode> <width> <height> <fps> <odr>
 *
 *   Positional arguments, all required, no flags:
 *
 *   <device>  device model: GS130WI | GS130W | GS130W_NO_EEPROM
 *   <mode>    pipeline mode: raw | resize | rect (anything else means raw)
 *   <width>   output width in pixels
 *   <height>  output height in pixels
 *   <fps>     camera frame rate
 *   <odr>     IMU output data rate in Hz
 *
 *   Normally not called directly: 'imu-info' inside `gs130 shell` passes the
 *   device config locked by the shell in exactly this order. 'imu-info --help'
 *   is answered by the shell, which prints its usage and never starts this
 *   program; outside the shell "--help" would be read as a device name.
 *
 * output: on stdout, the model line followed by the detail block
 *
 *   imu name: <model>
 *   <info>
 *
 *   or a single line when no IMU was detected:
 *
 *   no IMU detected
 *
 *   <info> is whatever gs130_get_imu_info() returns; a NULL detail block prints
 *   as an empty line, and a model name without details prints the name alone.
 *
 * exit status:
 *   0  a model name was reported, or no IMU was detected
 *   1  fewer than six arguments, or gs130_init() failed
 *
 * error reporting: this program prints plain, unprefixed diagnostics on stderr
 * ("init failed"), so a failure that happens in here does not carry the
 * 'gs130-...: <reason>' / try '--help' contract the shell script follows.
 * The same is true of the other sample programs, and of '--help' and argument
 * validation: those live in the gs130 script, not in the samples.
 *
 * This file is part of gs130_camera_sdk (https://github.com/D-Robotics/gs130_camera_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "gs130.h"
#include "gs130_define.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char **argv)
{
    if(argc < 7)return 1;
    const char *device   = argv[1];
    const char *mode_s   = argv[2];
    int w   = atoi(argv[3]);
    int h   = atoi(argv[4]);
    int fps = atoi(argv[5]);
    int odr = atoi(argv[6]);

    gs130_camera_mode_t mode = GS130_CAMERA_MODE_RAW;
    if(!strcmp(mode_s, "resize"))     mode = GS130_CAMERA_MODE_RESIZE;
    else if(!strcmp(mode_s, "rect"))  mode = GS130_CAMERA_MODE_RECT;

    gs130_config_t cfg = GS130_CONFIG(device, mode, w, h, fps, odr);
    gs130_device_t *dev = gs130_create();
    if(gs130_init(dev, &cfg) != GS130_OK){ fprintf(stderr, "init failed\n"); gs130_destroy(dev); return 1; }

    const char *name = gs130_get_imu_name(dev);
    if(!name){
        printf("no IMU detected\n");
    } else {
        const char *info = gs130_get_imu_info(dev);
        printf("imu name: %s\n%s\n", name, info ? info : "");
    }

    gs130_deinit(dev);
    gs130_destroy(dev);
    return 0;
}
