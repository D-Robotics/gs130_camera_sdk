/**
 * @file gs130-run.c
 * @brief Stream the camera and the IMU and print the newest data of each
 *
 * usage: gs130-run <device> <mode> <width> <height> <fps> <odr>
 *
 *   Positional arguments, all required, no flags:
 *
 *   <device>  device model: GS130WI | GS130W | GS130W_NO_EEPROM | GS130WI_20260924
 *   <mode>    pipeline mode: raw | resize | rect (anything else means raw)
 *   <width>   output width in pixels
 *   <height>  output height in pixels
 *   <fps>     camera frame rate
 *   <odr>     IMU output data rate in Hz
 *
 *   Normally not called directly: 'run' inside `gs130 shell` passes the device
 *   config locked by the shell in exactly this order. Runs until SIGINT
 *   (Ctrl-C), which stops capture and exits cleanly; it records nothing.
 *
 * output: on stdout, a dashed separator followed by two lines, refreshed at most
 * every PRINT_US (20 ms, i.e. 50 times per second):
 *
 *   ------------------------------------------------------------------
 *   Camera  [ID: <index> | fps: <rate> | Left timestamp: <s> s | Right timestamp: <s> s]
 *   IMU     [ID: <index> | odr: <rate> Hz | Timestamp: <s> s | Accel <x> <y> <z> m/s^2 | Gyro <x> <y> <z> rad/s | Temp <t> C]
 *
 *   The '------' line above is as wide as the longer of the two lines, so its
 *   length follows the numbers; the two field layouts are exactly as shown.
 *   <index> is 0-based and counts what the stream produced since startup, so it
 *   keeps growing and restarts at 0 on the next run. <rate> is the average over
 *   the last CAM_WINDOW frames / IMU_WINDOW packets and prints as a right-aligned
 *   "--" until that many have arrived. Each <s> is a timestamp_ns converted to
 *   seconds with 6 decimals (divided by 1e9): the capture/sample time on the
 *   camera clock, not seconds since this program started. Accel is m/s^2, gyro
 *   rad/s, temp degrees Celsius, and every number is printed signed (accel and
 *   temp with two and one decimals, gyro with three). Before the first frame or
 *   packet the respective line reads "Camera  [no frame yet]" / "IMU     [no
 *   packet yet]".
 *
 * exit status: 0 after Ctrl-C, 1 when gs130_init() or gs130_start() failed.
 *
 * error reporting: this program prints plain, unprefixed diagnostics on stderr
 * ("init/start failed"), so a failure that happens in here does not carry the
 * 'gs130-...: <reason>' / try '--help' contract the shell script follows. The
 * same is true of the other sample programs, and of '--help' and argument
 * validation: those live in the gs130 script, not in the samples.
 *
 * This file is part of gs130_camera_sdk (https://github.com/D-Robotics/gs130_camera_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "gs130.h"
#include "gs130_define.h"

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define CAM_WINDOW 10
#define IMU_WINDOW 100
#define PRINT_US   20000

static volatile sig_atomic_t stop = 0;
static void on_sigint(int sig){(void)sig; stop = 1;}

static uint64_t now_us(void);
static double cam_fps(uint64_t ts);
static double imu_odr(uint64_t ts);
static void print_block(uint64_t cam_idx, uint64_t imu_idx, double fps, double odr,
                        uint64_t timestamp_left, uint64_t timestamp_right,
                        const gs130_imu_packet_t *imu);

int main(int argc, char **argv)
{
    if(argc < 7)return 1;
    signal(SIGINT, on_sigint);

    gs130_camera_mode_t mode = !strcmp(argv[2], "rect")   ? GS130_CAMERA_MODE_RECT :
                               !strcmp(argv[2], "resize") ? GS130_CAMERA_MODE_RESIZE :
                               GS130_CAMERA_MODE_RAW;

    gs130_config_t cfg = GS130_CONFIG(
        argv[1], mode, atoi(argv[3]), atoi(argv[4]), atoi(argv[5]), atoi(argv[6]));

    /* Create Device Handle */
    gs130_device_t *dev = gs130_create();

    /* Init Device and Start DataFlow */
    int ret = 0;
    if(gs130_init(dev, &cfg) != GS130_OK || gs130_start(dev) != GS130_OK){
        fprintf(stderr, "init/start failed\n");
        ret = 1; goto out;
    }

    gs130_imu_packet_t imu_data = {0};
    double fps = 0, odr = 0;
    uint64_t timestamp_left = 0, timestamp_right = 0;
    uint64_t cam_idx = 0, imu_idx = 0;

    /* Get data */
    while(!stop){
        gs130_image_nv12_t left_image, right_image;
        while(gs130_get_nv12_frame(dev, &left_image, &right_image) == GS130_OK){
            fps = cam_fps(left_image.timestamp_ns);
            timestamp_left = left_image.timestamp_ns;
            timestamp_right = right_image.timestamp_ns;
            cam_idx++;
            free(left_image.data), free(right_image.data);   /* the buffer is ours to free */
        }
        gs130_imu_packet_t p;
        while(gs130_get_imu_packet(dev, &p) == GS130_OK){
            odr = imu_odr(p.timestamp_ns);
            imu_data = p;
            imu_idx++;
        }

        print_block(cam_idx, imu_idx, fps, odr, timestamp_left, timestamp_right, &imu_data);
        usleep(1000);
    }

out:
    gs130_stop(dev);
    gs130_deinit(dev);
    gs130_destroy(dev);
    return ret;
}

static double cam_fps(uint64_t ts)
{
    static uint64_t ring[CAM_WINDOW];
    static size_t pos, n;

    ring[pos] = ts;
    pos = (pos + 1) % CAM_WINDOW;
    if(++n < CAM_WINDOW)return 0.0;

    const uint64_t first = ring[pos], last = ring[(pos + CAM_WINDOW - 1) % CAM_WINDOW];
    return (last > first) ? (double)(CAM_WINDOW - 1) * 1e9 / (double)(last - first) : 0.0;
}

static double imu_odr(uint64_t ts)
{
    static uint64_t ring[IMU_WINDOW];
    static size_t pos, n;

    ring[pos] = ts;
    pos = (pos + 1) % IMU_WINDOW;
    if(++n < IMU_WINDOW)return 0.0;

    const uint64_t first = ring[pos], last = ring[(pos + IMU_WINDOW - 1) % IMU_WINDOW];
    return (last > first) ? (double)(IMU_WINDOW - 1) * 1e9 / (double)(last - first) : 0.0;
}

static uint64_t now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000 + (uint64_t)ts.tv_nsec / 1000;
}

static void print_block(uint64_t cam_idx, uint64_t imu_idx, double fps, double odr,
                        uint64_t timestamp_left, uint64_t timestamp_right,
                        const gs130_imu_packet_t *imu)
{
    static uint64_t next_print = 0;
    const uint64_t now = now_us();
    if(now < next_print)return;
    next_print += PRINT_US;
    if(next_print <= now)next_print = now + PRINT_US;

    /* both rates need a full window, so they stay "--" until it has filled */
    char fps_s[16] = "    --", odr_s[16] = "    --";
    if(cam_idx >= CAM_WINDOW)snprintf(fps_s, sizeof(fps_s), "%6.2f", fps);
    if(imu_idx >= IMU_WINDOW)snprintf(odr_s, sizeof(odr_s), "%6.2f", odr);

    char line[2][288];
    if(cam_idx)snprintf(line[0], sizeof(line[0]),
            "Camera  [ID: %llu | fps: %s | Left timestamp: %.6f s | Right timestamp: %.6f s]",
            (unsigned long long)cam_idx - 1, fps_s, timestamp_left / 1e9, timestamp_right / 1e9);
    else snprintf(line[0], sizeof(line[0]), "Camera  [no frame yet]");

    if(imu_idx)snprintf(line[1], sizeof(line[1]),
            "IMU     [ID: %llu | odr: %s Hz | Timestamp: %.6f s | Accel %+7.2f %+7.2f %+7.2f m/s^2 | "
            "Gyro %+8.3f %+8.3f %+8.3f rad/s | Temp %+5.1f C]",
            (unsigned long long)imu_idx - 1, odr_s, imu->timestamp_ns / 1e9,
            imu->accel[0], imu->accel[1], imu->accel[2],
            imu->gyro[0], imu->gyro[1], imu->gyro[2], imu->temp);
    else snprintf(line[1], sizeof(line[1]), "IMU     [no packet yet]");

    const size_t width = strlen(line[0]) > strlen(line[1]) ? strlen(line[0]) : strlen(line[1]);
    for(size_t i = 0; i < width; i++)putchar('-');
    printf("\n%s\n%s\n", line[0], line[1]);
}
