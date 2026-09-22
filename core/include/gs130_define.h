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
    strcmp((platform), "RDKX5") == 0 && strcmp((device), "GS130W_NO_EEPROM") == 0 ? (gs130_config_t) GS130_CONFIG_RDKX5_GS130W_NO_EEPROM((mode), (w), (h), (fps), (odr)) : \
    strcmp((platform), "RDKS100") == 0 && strcmp((device), "GS130WI") == 0 ? (gs130_config_t) GS130_CONFIG_RDKS100_GS130WI((mode), (w), (h), (fps), (odr)) : \
    strcmp((platform), "RDKS100") == 0 && strcmp((device), "GS130W_NO_EEPROM") == 0 ? (gs130_config_t) GS130_CONFIG_RDKS100_GS130W_NO_EEPROM((mode), (w), (h), (fps), (odr)) : \
    strcmp((platform), "RDKS600") == 0 && strcmp((device), "GS130WI") == 0 ? (gs130_config_t) GS130_CONFIG_RDKS600_GS130WI((mode), (w), (h), (fps), (odr)) : \
    strcmp((platform), "RDKS600") == 0 && strcmp((device), "GS130W_NO_EEPROM") == 0 ? (gs130_config_t) GS130_CONFIG_RDKS600_GS130W_NO_EEPROM((mode), (w), (h), (fps), (odr)) : \
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

/* GS130W stereo module with a readable calibration EEPROM. The camera addresses are the
   ones measured on the module this preset targets: left 0x33 and right 0x32, both
   answering chip-id 0x0132. They differ from the GS130WI module's 0x30/0x31, and pipeline
   construction requires both eyes to be found. The buses are listed as {4, 6} because the
   pipeline probes every listed bus for each address and lets the first one that answers
   win, so which of the two buses carries which eye does not matter. The EEPROM answers on
   bus 6 at 0x50 and is enabled here; its model is registered by the SZYGSJKJ pinhole V1.1
   driver, which also supplies the 90 degree installation angle of this
   landscape-mounted module. */
#define GS130_CONFIG_RDKX5_GS130W(mode_, width_, height_, fps_, odr_) {    \
    .camera_config = {                                                      \
        .bus = {4, 6}, .bus_num = 2,                                        \
        .left_addr = 0x33, .right_addr = 0x32,                              \
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

/* RDK S100. The board wires the module to different I2C buses and MIPI RX ports than
   the X5, so every bus-keyed field differs: the cameras sit on buses 1 and 2, and both
   the EEPROM and the IMU answer on bus 2 alone. The two MIPI RX ports and the sensor
   mode are the ones the board's own dual-sensor tuning configuration states for this
   module (/app/tuning_tool/cfg/matrix/tuning_sc132gs_dual_cim_isp_1088_1280).
   bus_reset_gpio is left disabled: the X5 pulses a reset line through sysfs and this
   board's configuration names no such line, so that value is still unverified. */
#define GS130_CONFIG_RDKS100_GS130WI(mode_, width_, height_, fps_, odr_) {  \
    .camera_config = {                                                      \
        .bus = {1, 2}, .bus_num = 2,                                        \
        .left_addr = 0x30, .right_addr = 0x32,                              \
        .sensor_width = 1088, .sensor_height = 1280,                        \
        .fps = (fps_), .line_length = 1400, .frame_length = 1500,           \
        .tuning_file = "lib_sc132gs_linear.so",                             \
        .output_width = (width_), .output_height = (height_),               \
        .mode = (mode_),                                                    \
        .stereo_layout = GS130_STEREO_LAYOUT_NONE,                          \
        .bus_mipi_rx    = { [0] = 0xFF, [1] = 0, [2] = 1, [3 ... 31] = 0xFF }, \
        .bus_reset_gpio = { [0 ... 31] = -1 },                              \
        .fsync_camera = GS130_CAMERA_RIGHT_IDX,                             \
    },                                                                      \
    .imu_config = {                                                         \
        .bus = {2}, .bus_num = 1, .addr = 0x68,                             \
        .odr_hz = (odr_), .accel_fsr_g = 16, .gyro_fsr_dps = 2000,          \
        .accel_bw_sel = 0, .gyro_bw_sel = 0,                                \
    },                                                                      \
    .eeprom_config = { .bus = {2}, .bus_num = 1, .addr = 0x50 },            \
    .camera_fifo = { .depth = 4,    .mode = GS130_FIFO_DROP_OLD },          \
    .imu_fifo    = { .depth = 1024, .mode = GS130_FIFO_DROP_OLD },          \
}

/* A GS130W-family module with no usable calibration EEPROM, on this platform's
   camera connectors. The board wiring is the GS130WI preset's: the module uses the
   same two connectors, so bus, bus_mipi_rx and bus_reset_gpio are identical, and
   this board names no reset line. What differs is the module: its left camera
   answers at 0x33 rather than 0x30, it has no IMU, and its EEPROM is not usable: a
   chip answers at 0x50 on bus 1, but its contents are an unregistered model variant
   the SDK rejects, so probing it would only add a probe that always fails.
   eeprom_config.bus_num = 0 and imu_config.bus_num = 0 disable both, and with no IMU
   there is no imu_fifo. */
#define GS130_CONFIG_RDKS100_GS130W_NO_EEPROM(mode_, width_, height_, fps_, odr_) { \
    .camera_config = {                                                      \
        .bus = {1, 2}, .bus_num = 2,                                        \
        .left_addr = 0x33, .right_addr = 0x32,                              \
        .sensor_width = 1088, .sensor_height = 1280,                        \
        .fps = (fps_), .line_length = 1400, .frame_length = 1500,           \
        .tuning_file = NULL,                                                \
        .output_width = (width_), .output_height = (height_),               \
        .mode = (mode_),                                                    \
        .stereo_layout = GS130_STEREO_LAYOUT_NONE,                          \
        .bus_mipi_rx    = { [0] = 0xFF, [1] = 0, [2] = 1, [3 ... 31] = 0xFF }, \
        .bus_reset_gpio = { [0 ... 31] = -1 },                              \
        .fsync_camera = GS130_CAMERA_RIGHT_IDX,                             \
    },                                                                      \
    .imu_config = {                                                         \
        .bus_num = 0, .addr = 0x68,                                         \
        .odr_hz = (odr_), .accel_fsr_g = 16, .gyro_fsr_dps = 2000,          \
        .accel_bw_sel = 0, .gyro_bw_sel = 0,                                \
    },                                                                      \
    .eeprom_config = { .bus_num = 0, .addr = 0x50 },                        \
    .camera_fifo = { .depth = 4,    .mode = GS130_FIFO_DROP_OLD },          \
}

/* A GS130W-family module with no usable calibration EEPROM. Differs from
   GS130_CONFIG_RDKX5_GS130W in two places: the camera addresses and the disabled EEPROM.
   The addresses are the ones measured on the module this preset was added for (left on
   bus 4 at 0x33, right on bus 6 at 0x32). That module does not answer at the GS130W
   preset's 0x30/0x31, and pipeline construction requires both addresses to be found, so
   reusing those addresses left the device unable to initialize at all.
   eeprom_config.bus_num = 0 disables EEPROM probing. Its address stays listed the way the
   disabled IMU bus above keeps its own, so the two macros can still be compared field by
   field; bus_num is what the SDK actually reads. */
#define GS130_CONFIG_RDKX5_GS130W_NO_EEPROM(mode_, width_, height_, fps_, odr_) { \
    .camera_config = {                                                      \
        .bus = {4, 6}, .bus_num = 2,                                        \
        .left_addr = 0x33, .right_addr = 0x32,                              \
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
    .eeprom_config = { .bus_num = 0, .addr = 0x50 },                        \
    .camera_fifo = { .depth = 4,    .mode = GS130_FIFO_DROP_OLD },          \
}

/* RDK S600. Both cameras reach the board through the 22-pin MIPI connectors, which are the
   receivers the device tree names vcon@4 and vcon@5; their I2C buses happen to carry the
   same numbers here, but the port value in bus_mipi_rx is the receiver index, not the bus.
   bus_reset_gpio holds each port's camera enable line: pulling it low makes the sensor stop
   answering on I2C, so the SDK drives it high. */
#define GS130_CONFIG_RDKS600_GS130WI(mode_, width_, height_, fps_, odr_) {  \
    .camera_config = {                                                      \
        .bus = {4, 5}, .bus_num = 2,                                        \
        .left_addr = 0x30, .right_addr = 0x32,                              \
        .sensor_width = 1088, .sensor_height = 1280,                        \
        .fps = (fps_), .line_length = 1400, .frame_length = 1500,           \
        .tuning_file = "lib_sc132gs_linear.so",                             \
        .output_width = (width_), .output_height = (height_),               \
        .mode = (mode_),                                                    \
        .stereo_layout = GS130_STEREO_LAYOUT_NONE,                          \
        .bus_mipi_rx    = { [0 ... 3] = 0xFF, [4] = 4, [5] = 5, [6 ... 31] = 0xFF },  \
        .bus_reset_gpio = { [0 ... 3] = -1, [4] = 411, [5] = 412, [6 ... 31] = -1 },  \
        .fsync_camera = GS130_CAMERA_RIGHT_IDX,                             \
    },                                                                      \
    .imu_config = {                                                         \
        .bus = {5}, .bus_num = 1, .addr = 0x68,                             \
        .odr_hz = (odr_), .accel_fsr_g = 16, .gyro_fsr_dps = 2000,          \
        .accel_bw_sel = 0, .gyro_bw_sel = 0,                                \
    },                                                                      \
    .eeprom_config = { .bus = {5}, .bus_num = 1, .addr = 0x50 },            \
    .camera_fifo = { .depth = 4,    .mode = GS130_FIFO_DROP_OLD },          \
    .imu_fifo    = { .depth = 1024, .mode = GS130_FIFO_DROP_OLD },          \
}

/* The GS130W module without a usable calibration EEPROM, on the S600's 22-pin connectors.
   Bus, port and GPIO assignments are the GS130WI preset's because the module occupies the
   same two connectors. The left address is the one that module answers at on the S100; it is
   carried over rather than measured, as no such module was available on an S600. */
#define GS130_CONFIG_RDKS600_GS130W_NO_EEPROM(mode_, width_, height_, fps_, odr_) { \
    .camera_config = {                                                      \
        .bus = {4, 5}, .bus_num = 2,                                        \
        .left_addr = 0x33, .right_addr = 0x32,                              \
        .sensor_width = 1088, .sensor_height = 1280,                        \
        .fps = (fps_), .line_length = 1400, .frame_length = 1500,           \
        .tuning_file = NULL,                                                \
        .output_width = (width_), .output_height = (height_),               \
        .mode = (mode_),                                                    \
        .stereo_layout = GS130_STEREO_LAYOUT_NONE,                          \
        .bus_mipi_rx    = { [0 ... 3] = 0xFF, [4] = 4, [5] = 5, [6 ... 31] = 0xFF },  \
        .bus_reset_gpio = { [0 ... 3] = -1, [4] = 411, [5] = 412, [6 ... 31] = -1 },  \
        .fsync_camera = GS130_CAMERA_RIGHT_IDX,                             \
    },                                                                      \
    .imu_config = {                                                         \
        .bus_num = 0, .addr = 0x68,                                         \
        .odr_hz = (odr_), .accel_fsr_g = 16, .gyro_fsr_dps = 2000,          \
        .accel_bw_sel = 0, .gyro_bw_sel = 0,                                \
    },                                                                      \
    .eeprom_config = { .bus_num = 0, .addr = 0x50 },                        \
    .camera_fifo = { .depth = 4,    .mode = GS130_FIFO_DROP_OLD },          \
}

#endif /* GS130_DEFINE_H */
