/**
 * @file gs130.h
 * @brief GS130xx SDK C API.
 *
 * This file is part of gs130_camera_sdk (https://github.com/D-Robotics/gs130_camera_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#ifndef GS130_H
#define GS130_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum gs130_err_e{
    GS130_OK = 0,          /* success */
    GS130_PARAM_ERROR,     /* invalid parameter (null pointer, bad config, wrong call order) */
    GS130_UNSUPPORTED,     /* hardware cannot do this configuration */
    GS130_NOT_FOUND,       /* device or calibration data not detected */
    GS130_HW_ERROR,        /* low-level communication or driver failure */
    GS130_TIMEOUT,         /* no data available right now (non-blocking reads) */
    GS130_THREAD_CLOSED,   /* threads closed or never started; after a fault, recover via deinit + init */
}gs130_err_t;

typedef enum gs130_fifo_mode_e{
    GS130_FIFO_DROP_NEW,   /* drop the newest data when full */
    GS130_FIFO_DROP_OLD,   /* overwrite the oldest data when full */
}gs130_fifo_mode_t;

typedef struct gs130_fifo_config_s{
    size_t depth;              /* queue depth, >= 2 */
    gs130_fifo_mode_t mode;   /* policy when full */
}gs130_fifo_config_t;

/* ==================== Device Config ==================== */

typedef enum gs130_camera_mode_e{
    GS130_CAMERA_MODE_RAW,     /* CAM -> VIN -> ISP -> OUT */
    GS130_CAMERA_MODE_RESIZE,  /* CAM -> VIN -> ISP -> VSE -> OUT */
    GS130_CAMERA_MODE_RECT,    /* CAM -> VIN -> ISP -> GDC -> VSE -> OUT (needs calibration) */
}gs130_camera_mode_t;

typedef enum gs130_camera_index_e{
    GS130_CAMERA_RIGHT_IDX = 0,
    GS130_CAMERA_LEFT_IDX  = 1,
}gs130_camera_index_t;

/** Stereo stitching layout. */
typedef enum gs130_stereo_layout_e{
    GS130_STEREO_LAYOUT_NONE,          /* no stitching (default): separate per-eye frames */
    GS130_STEREO_LAYOUT_LEFT_RIGHT,    /* horizontal: left eye on the left, right eye on the right */
    GS130_STEREO_LAYOUT_RIGHT_LEFT,    /* horizontal: right eye on the left, left eye on the right */
    GS130_STEREO_LAYOUT_TOP_BOTTOM,    /* vertical: left eye on top, right eye below */
    GS130_STEREO_LAYOUT_BOTTOM_TOP,    /* vertical: right eye on top, left eye below */
}gs130_stereo_layout_t;

typedef struct gs130_camera_config_s{
    uint8_t bus[32]; /* candidate I2C buses, probed in order */
    size_t bus_num;
    uint8_t  left_addr;
    uint8_t  right_addr;

    uint32_t sensor_width, sensor_height;
    uint32_t fps;
    uint32_t line_length, frame_length;
    const char *tuning_file; /* ISP tuning file, nullptr = do not load */

    uint32_t output_width, output_height;   /* Raw mode: must equal the sensor size */
    gs130_camera_mode_t mode;

    /* Stitched output: GS130_STEREO_LAYOUT_NONE (default) = separate per-eye frames;
       any other value = the camera thread fills the frame in that layout directly,
       and the stitched frame is popped zero-copy */
    gs130_stereo_layout_t stereo_layout;

    uint8_t bus_mipi_rx[32]; /* I2C bus -> MIPI RX map; 0xFF = not configured */
    int bus_reset_gpio[32]; /* I2C bus -> reset GPIO map; -1 = do not control */
    /* e.g. bus-4 & gpio-532 & mipi-rx-1 */
    /* memset(bus_mipi_rx, 0xFF, sizeof(bus_mipi_rx)); */
    /* memset(bus_reset_gpio, -1, sizeof(bus_reset_gpio)); */
    /* bus_mipi_rx[4] = 1; */
    /* bus_reset_gpio[4] = 532; */

    gs130_camera_index_t fsync_camera;   /* camera the IMU FSYNC pin is bound to */
}gs130_camera_config_t;

typedef struct gs130_imu_config_s{
    uint8_t bus[32];        /* candidate I2C buses, probed in order */
    size_t bus_num;
    uint8_t addr;
    uint32_t odr_hz;
    uint16_t accel_fsr_g;
    uint16_t gyro_fsr_dps;
    uint8_t accel_bw_sel;   /* accel UI filter setting 0..F, see info() for bandwidths */
    uint8_t gyro_bw_sel;    /* gyro UI filter setting 0..F, see info() for bandwidths */
}gs130_imu_config_t;

/** EEPROM (calibration data) configuration. */
typedef struct gs130_eeprom_config_s{
    uint8_t bus[32];        /* candidate I2C buses, probed in order */
    size_t bus_num;
    uint8_t addr;
}gs130_eeprom_config_t;

/** Top-level device configuration for gs130_init(). */
typedef struct gs130_config_s{
    gs130_camera_config_t camera_config;
    gs130_imu_config_t imu_config;
    gs130_eeprom_config_t eeprom_config;

    /* FIFO queues: camera frames and IMU packets configured separately */
    gs130_fifo_config_t camera_fifo;
    gs130_fifo_config_t imu_fifo;
}gs130_config_t;

/** Opaque device handle. */
typedef struct gs130_device_s gs130_device_t;

/**
 * @brief SDK version string.
 *
 * @return Version string (e.g. "0.0.1"); statically allocated, do not free.
 */
const char *gs130_version(void);

/**
 * @brief Compiled-in platform string.
 *
 * @return Platform name (e.g. "RDKX5"), taken from the platform directory name at
 *         build time; statically allocated, do not free.
 */
const char *gs130_platform(void);

/**
 * @brief Create a device handle (an empty shell; call gs130_init to initialize it).
 *
 * @return Device handle; release it with gs130_destroy() when done.
 */
gs130_device_t *gs130_create();

/**
 * @brief Release a device handle.
 *
 * gs130_deinit() (or at least gs130_stop()) must be called first; destroying the
 * handle while background threads are still running is undefined behaviour.
 *
 * @param[in] dev Device handle from gs130_create().
 */
void gs130_destroy(
    gs130_device_t *dev);

/**
 * @brief Initialize the device: probe EEPROM/IMU along the candidate buses in order,
 *        load the calibration, configure the camera and the IMU (streaming stays off).
 *
 * The EEPROM and the IMU are both optional and the SDK degrades gracefully when they are
 * not detected; GS130_CAMERA_MODE_RECT does need a calibration though, and returns
 * GS130_PARAM_ERROR without an EEPROM.  A handle can only be initialized once; a
 * repeated call returns GS130_PARAM_ERROR.
 *
 * @param[in] dev Device handle from gs130_create().
 * @param[in] cfg Device configuration, see gs130_config_t.
 * @return err code
 */
gs130_err_t gs130_init(
    gs130_device_t *dev,
    const gs130_config_t *cfg);

/**
 * @brief Deinitialize: release all device resources (calls gs130_stop() first if still running).
 *
 * The handle itself stays valid afterwards and must be released by gs130_destroy().
 *
 * @param[in] dev Device handle from gs130_create().
 * @return err code
 */
gs130_err_t gs130_deinit(
    gs130_device_t *dev);

/**
 * @brief Start capture.
 *
 * When an IMU was detected, the camera waits for the IMU FSYNC handshake to complete
 * before streaming; without an IMU it streams immediately.  A repeated call returns
 * GS130_PARAM_ERROR.
 *
 * @param[in] dev Device handle from gs130_create().
 * @return err code
 */
gs130_err_t gs130_start(
    gs130_device_t *dev);

/**
 * @brief Stop capture and wait for the background threads to exit.
 *
 * A no-op when not running; gs130_start() may be called again afterwards.
 *
 * @param[in] dev Device handle from gs130_create().
 */
void gs130_stop(
    gs130_device_t *dev);

/* ==================== Camera Data ==================== */

typedef struct gs130_image_nv12_s{
    uint8_t *data; /* contiguous tightly-packed NV12: Y plane (width*height bytes), then UV plane (width*height/2 bytes) */
    uint32_t width, height;
    uint64_t timestamp_ns;
}gs130_image_nv12_t;

/**
 * @brief Query how many synchronized stereo frame pairs the queue currently holds.
 *
 * Use together with gs130_get_nv12_frame and gs130_get_stereo_nv12_frame: only fetch
 * when the count is greater than 0.
 *
 * @param[in] dev Device handle from gs130_create().
 * @return Number of frame pairs available in the queue.
 */
size_t gs130_available_camera(
    gs130_device_t *dev);

/**
 * @brief Fetch the left and right NV12 frames separately.
 *
 * @param[in]  dev Device handle from gs130_create().
 * @param[out] image_left Left camera frame.
 * @param[out] image_right Right camera frame.
 * @return err code
 *
 * @note The data buffers of image_left / image_right are malloc()ed by the SDK and owned
 *       by the caller, who must free() them.
 * @note Not available in a stitching layout (stereo_layout other than NONE); returns
 *       GS130_UNSUPPORTED.
 */
gs130_err_t gs130_get_nv12_frame(
    gs130_device_t *dev,
    gs130_image_nv12_t *image_left,
    gs130_image_nv12_t *image_right);

/**
 * @brief Fetch a stitched left+right NV12 frame.
 *
 * The stitching layout is fixed by gs130_camera_config_t.stereo_layout at gs130_init
 * time; the camera thread fills the frame in that layout and this function pops it
 * zero-copy.
 *
 * @param[in]  dev Device handle from gs130_create().
 * @param[out] image Single stitched NV12 frame: a horizontal stitch doubles the width,
 *                   a vertical stitch doubles the height.
 * @return err code
 *
 * @note The data buffer of image is malloc()ed by the SDK and owned by the caller, who
 *       must free() it.
 * @note Only available when stereo_layout is not GS130_STEREO_LAYOUT_NONE; otherwise
 *       returns GS130_UNSUPPORTED.
 */
gs130_err_t gs130_get_stereo_nv12_frame(
    gs130_device_t *dev,
    gs130_image_nv12_t *image);

/* ==================== IMU Data ==================== */

typedef struct gs130_imu_packet_s{
    float accel[3]; /* m/s^2 */
    float gyro[3]; /* rad/s */
    float temp; /* degC */
    bool  is_fsync; /* this packet is an FSYNC sync packet */
    uint64_t timestamp_ns; /* corrected absolute timestamp (aligned to the camera clock) */
}gs130_imu_packet_t;

/**
 * @brief Query how many IMU packets the queue currently holds.
 *
 * Use together with gs130_get_imu_packet: only fetch when the count is greater than 0.
 * Returns 0 when no IMU was detected or the IMU has failed.
 *
 * @param[in] dev Device handle from gs130_create().
 * @return Number of packets available in the queue.
 */
size_t gs130_available_imu(
    gs130_device_t *dev);

/**
 * @brief Fetch one IMU packet.
 *
 * @param[in]  dev Device handle from gs130_create().
 * @param[out] out IMU packet.
 * @return err code
 */
gs130_err_t gs130_get_imu_packet(
    gs130_device_t *dev,
    gs130_imu_packet_t *out);

/**
 * @brief Get the model name of the detected IMU.
 *
 * @param[in] dev Device handle from gs130_create().
 * @return IMU model string, or NULL when no IMU was detected.
 */
const char *gs130_get_imu_name(
    gs130_device_t *dev);

/**
 * @brief Get detailed information about the detected IMU (setting/bandwidth tables, etc.).
 *
 * @param[in] dev Device handle from gs130_create().
 * @return IMU information string, or NULL when no IMU was detected.
 */
const char *gs130_get_imu_info(
    gs130_device_t *dev);

/* ==================== EEPROM Data ==================== */

typedef enum gs130_dist_model_e{
    GS130_DIST_PINHOLE,    /* dist_coeffs = [k1,k2,p1,p2,k3,k4,k5,k6] */
    GS130_DIST_FISHEYE,    /* dist_coeffs = [k1,k2,k3,k4,0,0,0,0] */
}gs130_dist_model_t;

typedef struct gs130_camera_intrinsics_s{
    double fx, fy, cx, cy;
    double K[9], dist_coeffs[8];
    gs130_dist_model_t dist_model;
}gs130_camera_intrinsics_t;

typedef struct gs130_imu_intrinsics_s{
    double accel_misalign[9];
    double accel_scale[3];
    double accel_bias[3];
    double accel_noise;          /* m/s^2/sqrt(Hz) */
    double accel_random_walk;    /* m/s^3/sqrt(Hz) */
    double gyro_misalign[9];
    double gyro_scale[3];
    double gyro_bias[3];
    double gyro_noise;           /* rad/s/sqrt(Hz) */
    double gyro_random_walk;     /* rad/s^2/sqrt(Hz) */
}gs130_imu_intrinsics_t;

typedef struct gs130_calibration_s{
    gs130_imu_intrinsics_t imu;
    gs130_camera_intrinsics_t camera_right;
    gs130_camera_intrinsics_t camera_left;

    /** About the extrinsics
     * 1. An extrinsic is the rotation matrix and translation vector that map a point
     *    from that device's frame into the reference frame, i.e. the absolute pose of
     *    the device in the reference frame.
     * 2. Device initialization defines the reference frame (chosen by the EEPROM driver)
     *    and converts every device extrinsic into an absolute pose in that frame.
     * 3. With stereo rectification enabled the reference frame does not change, but the
     *    extrinsics become virtual (parallel stereo), so watch for the frame change when
     *    using them.
     */
    double camera_right_R[9], camera_right_T[3];
    double camera_left_R[9], camera_left_T[3];
    double imu_R[9], imu_T[3];

    int camera_install_angle;
}gs130_calibration_t;

typedef enum gs130_reference_frame_e{
    GS130_REF_CAMERA_RIGHT,
    GS130_REF_CAMERA_LEFT,
    GS130_REF_IMU,
}gs130_reference_frame_t;

/**
 * @brief Get the camera intrinsics.
 *
 * @param[in]  dev        Device handle from gs130_create().
 * @param[in]  cam_idx    Camera index.
 * @param[out] intrinsics Camera intrinsics.
 * @return err code
 */
gs130_err_t gs130_get_camera_intrinsics(
    gs130_device_t *dev,
    gs130_camera_index_t cam_idx,
    gs130_camera_intrinsics_t *intrinsics);

/**
 * @brief Get the IMU intrinsics.
 *
 * @param[in]  dev        Device handle from gs130_create().
 * @param[out] intrinsics IMU intrinsics.
 * @return err code
 */
gs130_err_t gs130_get_imu_intrinsics(
    gs130_device_t *dev,
    gs130_imu_intrinsics_t *intrinsics);

/**
 * @brief Get the relative rotation matrix between two devices.
 *
 * @param[in]  dev        Device handle from gs130_create().
 * @param[in]  from_frame Source frame.
 * @param[in]  to_frame   Target frame.
 * @param[out] R          Rotation matrix, row-major 3x3 (9 doubles).
 * @return err code
 *
 * @note With from_frame == to_frame the identity matrix is expected, but the function
 *       still runs the matrix math, so the returned identity may differ in precision.
 * @note R maps a point from the from_frame frame into the to_frame frame, i.e. the
 *       rotation of from_frame as seen from to_frame.
 */
gs130_err_t gs130_get_relative_R(
    gs130_device_t *dev,
    gs130_reference_frame_t from_frame,
    gs130_reference_frame_t to_frame,
    double *R);

/**
 * @brief Get the relative translation vector between two devices.
 *
 * @param[in]  dev        Device handle from gs130_create().
 * @param[in]  from_frame Source frame.
 * @param[in]  to_frame   Target frame.
 * @param[out] T          Translation vector (3 doubles).
 * @return err code
 *
 * @note With from_frame == to_frame the zero vector is expected, but the function still
 *       runs the matrix math, so the returned zero vector may differ in precision.
 * @note T maps a point from the from_frame frame into the to_frame frame, i.e. the
 *       position of from_frame as seen from to_frame.
 */
gs130_err_t gs130_get_relative_T(
    gs130_device_t *dev,
    gs130_reference_frame_t from_frame,
    gs130_reference_frame_t to_frame,
    double *T);

/**
 * @brief Get the complete stereo (and IMU) calibration.
 *
 * @param[in]  dev         Device handle from gs130_create().
 * @param[out] calibration Stereo (and IMU) calibration structure.
 * @return err code
 *
 * @note Do not confuse the meaning of the extrinsics in gs130_calibration_t; see the
 *       structure definition.
 */
gs130_err_t gs130_get_calibration(
    gs130_device_t *dev,
    gs130_calibration_t *calibration);

/**
 * @brief Change the reference frame of the devices.
 *
 * Sets the given device's pose to the given value and moves every device pose along with
 * it, so the other devices keep their pose relative to that device.
 *
 * @param[in] dev       Device handle from gs130_create().
 * @param[in] ref_frame Reference device.
 * @param[in] ref_R     Rotation matrix of the reference device (9 doubles).
 * @param[in] ref_T     Translation vector of the reference device (3 doubles).
 * @return err code
 *
 * @note This function modifies the calibration stored inside the device, so later
 *       gs130_get_calibration / gs130_get_relative_R / gs130_get_relative_T results
 *       change accordingly.
 * @note Only valid after gs130_init(): init reloads the calibration from the EEPROM and
 *       overwrites any earlier change.  This function only changes the calibration data;
 *       it does not recompute the stereo rectification.
 */
gs130_err_t gs130_convert_calibration(
    gs130_device_t *dev,
    gs130_reference_frame_t ref_frame,
    const double ref_R[9],
    const double ref_T[3]);

/**
 * @brief Get the detected EEPROM header declaration.
 *
 * @param[in] dev Device handle from gs130_create().
 * @return EEPROM header string, or NULL when no EEPROM was detected.
 */
const char *gs130_get_eeprom_name(
    gs130_device_t *dev);

/**
 * @brief Get detailed information about the detected EEPROM (vendor, version, distortion model, etc.).
 *
 * @param[in] dev Device handle from gs130_create().
 * @return EEPROM information string, or NULL when no EEPROM was detected.
 */
const char *gs130_get_eeprom_info(
    gs130_device_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* GS130_H */
