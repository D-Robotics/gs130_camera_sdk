/**
 * @file gs130_define.h
 * @brief Optional preset configurations for supported GS130 modules.
 *
 * Include gs130.h, string.h, stdio.h, and stdlib.h before this header. These
 * initializers use GNU C designated-range syntax and compound literals; they are
 * intended for the project's GNU C build and are not a portable C++ interface.
 *
 * GS130_CONFIG() selects a preset for the build platform reported by
 * gs130_platform(). GS130_CONFIG_PLATFORM() accepts an explicit platform string.
 * In both macros, device is a model-name string, mode is a gs130_camera_mode_t,
 * w and h are output dimensions in pixels, fps is frames per second, and odr is
 * the IMU output data rate in Hz. Unsupported platform/model pairs print an error
 * and terminate the process.
 *
 * @warning Preset values may change between SDK releases. This header does not
 *          provide a backward-compatible configuration ABI.
 *
 * This file is part of gs130_camera_sdk (https://github.com/D-Robotics/gs130_camera_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#ifndef GS130_DEFINE_H
#define GS130_DEFINE_H

#define GS130_CONFIG(device, mode, w, h, fps, odr) GS130_CONFIG_PLATFORM(gs130_platform(), device, mode, w, h, fps, odr)

#define GS130_CONFIG_PLATFORM(platform, device, mode, w, h, fps, odr) \
    strcmp((platform), "RDKX5") == 0 && strcmp((device), "GS130WI") == 0 ? (gs130_config_t) GS130_CONFIG_RDKX5_GS130WI((mode), (w), (h), (fps), (odr)) : \
    strcmp((platform), "RDKX5") == 0 && strcmp((device), "GS130W")  == 0 ? (gs130_config_t) GS130_CONFIG_RDKX5_GS130W((mode), (w), (h), (fps), (odr))  : \
    (fprintf(stderr, "unsupported platform/device: %s %s\n", (platform), (device)), exit(1), (gs130_config_t){0})

#define GS130_CONFIG_RDKX5_GS130WI(mode_, width_, height_, fps_, odr_) {    \
    .camera_config = {                                                      \
        .bus = {4, 6}, .bus_num = 2,                                        \
        .left_addr = 0x30, .right_addr = 0x32,                              \
        .sensor_width = 1088, .sensor_height = 1280,                        \
        .fps = (fps_), .line_length = 1400, .frame_length = 1500,           \
        .tuning_file = NULL,                                                \
        .output_width = (width_), .output_height = (height_),               \
        .mode = (mode_),                                                    \
        .stereo_layout = GS130_STEREO_LAYOUT_NONE,                          \
        .bus_mipi_rx    = { [0 ... 3] = 0xFF, [4] = 2, [5] = 0xFF, [6] = 0, [7 ... 31] = 0xFF }, \
        .bus_reset_gpio = { [0 ... 3] = -1, [4] = 351, [5] = -1, [6] = 353, [7 ... 31] = -1 },   \
        .fsync_camera = GS130_CAMERA_RIGHT_IDX,                             \
    },                                                                      \
    .imu_config = {                                                         \
        .bus = {4, 6}, .bus_num = 2, .addr = 0x68,                          \
        .odr_hz = (odr_), .accel_fsr_g = 16, .gyro_fsr_dps = 2000,          \
        .accel_bw_sel = 0, .gyro_bw_sel = 0,                                \
    },                                                                      \
    .eeprom_config = { .bus = {4, 6}, .bus_num = 2, .addr = 0x50 },         \
    .camera_fifo = { .depth = 4,    .mode = GS130_FIFO_DROP_OLD },          \
    .imu_fifo    = { .depth = 1024, .mode = GS130_FIFO_DROP_OLD },          \
}

#define GS130_CONFIG_RDKX5_GS130W(mode_, width_, height_, fps_, odr_) {    \
    .camera_config = {                                                      \
        .bus = {4, 6}, .bus_num = 2,                                        \
        .left_addr = 0x30, .right_addr = 0x31,                              \
        .sensor_width = 1088, .sensor_height = 1280,                        \
        .fps = (fps_), .line_length = 1400, .frame_length = 1500,           \
        .tuning_file = NULL,                                                \
        .output_width = (width_), .output_height = (height_),               \
        .mode = (mode_),                                                    \
        .stereo_layout = GS130_STEREO_LAYOUT_NONE,                          \
        .bus_mipi_rx    = { [0 ... 3] = 0xFF, [4] = 2, [5] = 0xFF, [6] = 0, [7 ... 31] = 0xFF }, \
        .bus_reset_gpio = { [0 ... 3] = -1, [4] = 351, [5] = -1, [6] = 353, [7 ... 31] = -1 },   \
        .fsync_camera = GS130_CAMERA_RIGHT_IDX,                             \
    },                                                                      \
    .imu_config = {                                                         \
        .bus_num = 0, .addr = 0x68,                                         \
        .odr_hz = (odr_), .accel_fsr_g = 16, .gyro_fsr_dps = 2000,          \
        .accel_bw_sel = 0, .gyro_bw_sel = 0,                                \
    },                                                                      \
    .eeprom_config = { .bus = {4, 6}, .bus_num = 2, .addr = 0x50 },         \
    .camera_fifo = { .depth = 4,    .mode = GS130_FIFO_DROP_OLD },          \
}

#endif /* GS130_DEFINE_H */
