/**
 * @file gs130.h
 * @brief GS130 SDK C API.
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

/** Result codes returned by the public SDK functions. */
typedef enum gs130_err_e{
    GS130_OK = 0,          /**< Operation completed successfully. */
    GS130_PARAM_ERROR,     /**< Invalid argument, configuration, or call order. */
    GS130_UNSUPPORTED,     /**< The selected hardware or mode is not supported. */
    GS130_NOT_FOUND,       /**< A required device or calibration record was not detected. */
    GS130_HW_ERROR,        /**< Low-level communication or driver failure. */
    GS130_TIMEOUT,         /**< A non-blocking data query found no item available. */
    GS130_THREAD_CLOSED,   /**< Capture threads are stopped or have not been started. */
}gs130_err_t;

/** Policy applied when a bounded FIFO is full. */
typedef enum gs130_fifo_mode_e{
    GS130_FIFO_DROP_NEW,   /**< Reject the newly produced item. */
    GS130_FIFO_DROP_OLD,   /**< Discard the oldest item and enqueue the new item. */
}gs130_fifo_mode_t;

/** Configuration for one bounded SDK FIFO. */
typedef struct gs130_fifo_config_s{
    size_t depth;              /**< Capacity; must be at least 2. */
    gs130_fifo_mode_t mode;   /**< Policy applied when the FIFO is full. */
}gs130_fifo_config_t;

/* ==================== Device Config ==================== */

/** Camera processing pipeline selected at initialization. */
typedef enum gs130_camera_mode_e{
    GS130_CAMERA_MODE_RAW,     /**< CAM -> VIN -> ISP -> OUT; no calibration required. */
    GS130_CAMERA_MODE_RESIZE,  /**< CAM -> VIN -> ISP -> VSE -> OUT; requires calibration. */
    GS130_CAMERA_MODE_RECT,    /**< CAM -> VIN -> ISP -> GDC -> VSE -> OUT; requires calibration. */
}gs130_camera_mode_t;

/** Logical camera identifiers used by the SDK. */
typedef enum gs130_camera_index_e{
    GS130_CAMERA_RIGHT_IDX = 0, /**< Right camera; also the default FSYNC camera. */
    GS130_CAMERA_LEFT_IDX  = 1, /**< Left camera. */
}gs130_camera_index_t;

/** Arrangement of the two camera images in a stitched output frame. */
typedef enum gs130_stereo_layout_e{
    GS130_STEREO_LAYOUT_NONE,          /**< No stitching; return separate frames. */
    GS130_STEREO_LAYOUT_LEFT_RIGHT,    /**< Horizontal: left image, then right image. */
    GS130_STEREO_LAYOUT_RIGHT_LEFT,    /**< Horizontal: right image, then left image. */
    GS130_STEREO_LAYOUT_TOP_BOTTOM,    /**< Vertical: left image, then right image. */
    GS130_STEREO_LAYOUT_BOTTOM_TOP,    /**< Vertical: right image, then left image. */
}gs130_stereo_layout_t;

/** Stereo-camera and platform pipeline configuration. */
typedef struct gs130_camera_config_s{
    uint8_t bus[32]; /**< Candidate I2C bus numbers, probed in array order. */
    size_t bus_num; /**< Number of valid entries in bus; maximum 32. */
    uint8_t  left_addr; /**< Left sensor I2C address. */
    uint8_t  right_addr; /**< Right sensor I2C address. */

    uint32_t sensor_width, sensor_height; /**< Native sensor dimensions in pixels. */
    uint32_t fps; /**< Camera frame rate in frames per second. */
    uint32_t line_length, frame_length; /**< Sensor timing values in pixel clocks and lines. */
    const char *tuning_file; /**< ISP tuning file path, or NULL to disable loading. */

    uint32_t output_width, output_height; /**< Output dimensions; RAW requires the native size. */
    gs130_camera_mode_t mode; /**< Processing pipeline to construct. */

    /** NONE returns separate frames; other values produce one prearranged frame. */
    gs130_stereo_layout_t stereo_layout;

    uint8_t bus_mipi_rx[32]; /**< I2C-bus-to-MIPI-RX map; 0xFF means unconfigured. */
    int bus_reset_gpio[32]; /**< I2C-bus-to-reset-GPIO map; -1 disables GPIO control. */

    gs130_camera_index_t fsync_camera; /**< Camera whose trigger is wired to IMU FSYNC. */
}gs130_camera_config_t;

/** IMU discovery and sampling configuration. */
typedef struct gs130_imu_config_s{
    uint8_t bus[32]; /**< Candidate I2C bus numbers, probed in array order. */
    size_t bus_num; /**< Number of valid entries in bus; 0 disables IMU probing. */
    uint8_t addr; /**< IMU I2C address. */
    uint32_t odr_hz; /**< Output data rate in Hz. */
    uint16_t accel_fsr_g; /**< Accelerometer full-scale range in g. */
    uint16_t gyro_fsr_dps; /**< Gyroscope full-scale range in degrees per second. */
    uint8_t accel_bw_sel; /**< Accelerometer UI-filter selector, 0x0..0xF; see gs130_get_imu_info(). */
    uint8_t gyro_bw_sel; /**< Gyroscope UI-filter selector, 0x0..0xF; see gs130_get_imu_info(). */
}gs130_imu_config_t;

/** EEPROM discovery configuration for calibration data. */
typedef struct gs130_eeprom_config_s{
    uint8_t bus[32]; /**< Candidate I2C bus numbers, probed in array order. */
    size_t bus_num; /**< Number of valid entries in bus; 0 disables EEPROM probing. */
    uint8_t addr; /**< EEPROM I2C address. */
}gs130_eeprom_config_t;

/**
 * Top-level configuration passed to gs130_init().
 *
 * Zero-initialize the complete structure before assigning fields. The SDK does
 * not supply implicit defaults for fields left at zero; alternatively, start
 * from one of the presets in gs130_define.h.
 */
typedef struct gs130_config_s{
    gs130_camera_config_t camera_config; /**< Camera and platform pipeline settings. */
    gs130_imu_config_t imu_config; /**< IMU settings. */
    gs130_eeprom_config_t eeprom_config; /**< Calibration EEPROM settings. */
    gs130_fifo_config_t camera_fifo; /**< Synchronized camera-frame FIFO. */
    gs130_fifo_config_t imu_fifo; /**< IMU-packet FIFO; required when IMU probing is enabled. */
}gs130_config_t;

/** Opaque device handle. */
typedef struct gs130_device_s gs130_device_t;

/**
 * @brief SDK version string.
 *
 * @return Version string in major.minor.patch form; statically allocated, do
 *         not free.
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
 * The normal lifecycle is create -> init -> start -> stop -> deinit -> destroy.
 * Capture may be restarted after stop, and a deinitialized handle may be
 * initialized again.
 *
 * @return A new device handle. Release it with gs130_destroy().
 */
gs130_device_t *gs130_create();

/**
 * @brief Release a device handle.
 *
 * gs130_deinit() (or at least gs130_stop()) must be called first; destroying the
 * handle while background threads are still running is undefined behaviour.
 *
 * @param[in] dev Device handle returned by gs130_create(); NULL is accepted.
 */
void gs130_destroy(
    gs130_device_t *dev);

/**
 * @brief Initialize the device: probe EEPROM/IMU along the candidate buses in order,
 *        load the calibration, configure the camera and the IMU (streaming stays off).
 *
 * EEPROM and IMU probing are optional. RAW mode can operate without EEPROM
 * calibration; RESIZE and RECT require it because initialization updates the
 * output calibration. A handle can be initialized only once until deinitialized.
 *
 * @param[in] dev Device handle returned by gs130_create().
 * @param[in] cfg Fully initialized device configuration.
 * @retval GS130_OK Initialization succeeded.
 * @retval GS130_PARAM_ERROR An argument, configuration field, or call order is invalid.
 * @retval GS130_UNSUPPORTED The requested hardware configuration is unsupported.
 * @retval GS130_NOT_FOUND The stereo camera could not be detected.
 * @retval GS130_HW_ERROR Device communication or pipeline setup failed.
 */
gs130_err_t gs130_init(
    gs130_device_t *dev,
    const gs130_config_t *cfg);

/**
 * @brief Deinitialize: release all device resources (calls gs130_stop() first if still running).
 *
 * The handle itself stays valid afterwards and must be released by gs130_destroy().
 *
 * @param[in] dev Device handle returned by gs130_create().
 * @retval GS130_OK Resources were released, or the handle was already idle.
 * @retval GS130_PARAM_ERROR dev is NULL.
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
 * @param[in] dev Initialized device handle.
 * @retval GS130_OK Capture threads were started.
 * @retval GS130_PARAM_ERROR The handle is NULL, uninitialized, or already running.
 * @retval GS130_HW_ERROR The IMU or camera pipeline failed to start.
 */
gs130_err_t gs130_start(
    gs130_device_t *dev);

/**
 * @brief Stop capture and wait for the background threads to exit.
 *
 * A no-op when not running; gs130_start() may be called again afterwards.
 *
 * @param[in] dev Device handle; NULL is accepted and ignored.
 */
void gs130_stop(
    gs130_device_t *dev);

/* ==================== Camera Data ==================== */

/** One tightly packed NV12 image returned by the SDK. */
typedef struct gs130_image_nv12_s{
    uint8_t *data; /**< SDK-allocated buffer: Y plane followed by interleaved UV; caller must free(). */
    uint32_t width, height; /**< Image dimensions in pixels. */
    uint64_t timestamp_ns; /**< Capture timestamp in nanoseconds, aligned to the camera clock. */
}gs130_image_nv12_t;

/**
 * @brief Query how many synchronized stereo frame pairs the queue currently holds.
 *
 * The count is a snapshot and may change before the next call. The frame getters
 * are non-blocking and return GS130_TIMEOUT if another consumer empties the FIFO.
 *
 * @param[in] dev Running device handle.
 * @return Number of frame pairs currently queued, or 0 for an invalid, idle, or
 *         faulted device.
 */
size_t gs130_available_camera(
    gs130_device_t *dev);

/**
 * @brief Fetch the left and right NV12 frames separately.
 *
 * This function is non-blocking. On success, each returned data pointer is owned
 * by the caller and must be released with free().
 *
 * @param[in]  dev Running device handle.
 * @param[out] image_left Left-camera frame.
 * @param[out] image_right Right-camera frame.
 * @retval GS130_OK A synchronized frame pair was returned.
 * @retval GS130_PARAM_ERROR An argument is NULL or the camera FIFO is unavailable.
 * @retval GS130_UNSUPPORTED A stitched output layout is configured.
 * @retval GS130_TIMEOUT The FIFO is currently empty.
 * @retval GS130_THREAD_CLOSED Capture is not running.
 * @retval GS130_HW_ERROR A capture thread terminated after a hardware failure.
 */
gs130_err_t gs130_get_nv12_frame(
    gs130_device_t *dev,
    gs130_image_nv12_t *image_left,
    gs130_image_nv12_t *image_right);

/**
 * @brief Fetch a stitched left+right NV12 frame.
 *
 * The layout is fixed by gs130_camera_config_t.stereo_layout during
 * gs130_init(). This function is non-blocking and transfers ownership of the
 * already assembled pixel buffer to the caller; release image->data with free().
 *
 * @param[in]  dev Running device handle.
 * @param[out] image Stitched NV12 frame. Horizontal layouts double width; vertical
 *                   layouts double height.
 * @retval GS130_OK A stitched frame was returned.
 * @retval GS130_PARAM_ERROR An argument is NULL or the camera FIFO is unavailable.
 * @retval GS130_UNSUPPORTED GS130_STEREO_LAYOUT_NONE is configured.
 * @retval GS130_TIMEOUT The FIFO is currently empty.
 * @retval GS130_THREAD_CLOSED Capture is not running.
 * @retval GS130_HW_ERROR A capture thread terminated after a hardware failure.
 */
gs130_err_t gs130_get_stereo_nv12_frame(
    gs130_device_t *dev,
    gs130_image_nv12_t *image);

/* ==================== IMU Data ==================== */

/** One timestamp-corrected IMU sample. */
typedef struct gs130_imu_packet_s{
    float accel[3]; /**< Acceleration in m/s^2. */
    float gyro[3]; /**< Angular velocity in rad/s. */
    float temp; /**< Temperature in degrees Celsius. */
    bool  is_fsync; /**< True when the sample carries an IMU FSYNC anchor. */
    uint64_t timestamp_ns; /**< Absolute timestamp in ns, aligned to the camera clock. */
}gs130_imu_packet_t;

/**
 * @brief Query how many IMU packets the queue currently holds.
 *
 * The count is a snapshot and may change before the next call. Returns 0 when
 * no IMU is available, capture is idle, or the device has faulted.
 *
 * @param[in] dev Running device handle.
 * @return Number of packets currently queued.
 */
size_t gs130_available_imu(
    gs130_device_t *dev);

/**
 * @brief Fetch one IMU packet.
 *
 * This function is non-blocking.
 *
 * @param[in]  dev Running device handle.
 * @param[out] out Destination for one IMU packet.
 * @retval GS130_OK One packet was returned.
 * @retval GS130_PARAM_ERROR An argument is NULL or no IMU is available.
 * @retval GS130_TIMEOUT The FIFO is currently empty.
 * @retval GS130_THREAD_CLOSED Capture is not running.
 * @retval GS130_HW_ERROR The IMU thread terminated after a hardware failure.
 */
gs130_err_t gs130_get_imu_packet(
    gs130_device_t *dev,
    gs130_imu_packet_t *out);

/**
 * @brief Get the model name of the detected IMU.
 *
 * @param[in] dev Device handle from gs130_create().
 * @return Statically allocated IMU model string, or NULL if no IMU was detected.
 *         The caller must not modify or free the returned string.
 */
const char *gs130_get_imu_name(
    gs130_device_t *dev);

/**
 * @brief Get detailed information about the detected IMU (setting/bandwidth tables, etc.).
 *
 * @param[in] dev Device handle from gs130_create().
 * @return Statically allocated information string, or NULL if no IMU was
 *         detected. The caller must not modify or free the returned string.
 */
const char *gs130_get_imu_info(
    gs130_device_t *dev);

/* ==================== EEPROM Data ==================== */

/** Camera distortion model used by dist_coeffs. */
typedef enum gs130_dist_model_e{
    GS130_DIST_PINHOLE,    /**< OpenCV/radtan order: [k1,k2,p1,p2,k3,k4,k5,k6]. */
    GS130_DIST_FISHEYE,    /**< Equidistant order: [k1,k2,k3,k4,0,0,0,0]. */
}gs130_dist_model_t;

/** Camera intrinsic parameters. Matrices are stored row-major. */
typedef struct gs130_camera_intrinsics_s{
    double fx, fy, cx, cy; /**< Focal lengths and principal point in pixels. */
    /** K is a row-major 3x3 intrinsic matrix; dist_coeffs uses the model order above. */
    double K[9], dist_coeffs[8];
    gs130_dist_model_t dist_model; /**< Model used to interpret dist_coeffs. */
}gs130_camera_intrinsics_t;

/** IMU calibration parameters and noise characteristics. */
typedef struct gs130_imu_intrinsics_s{
    double accel_misalign[9]; /**< 3x3 row-major cross-axis coupling matrix. */
    double accel_scale[3]; /**< Per-axis scale factors. */
    double accel_bias[3]; /**< Per-axis bias in the calibrated sensor units. */
    double accel_noise;          /**< Accelerometer noise density, m/s^2/sqrt(Hz). */
    double accel_random_walk;    /**< Accelerometer random walk, m/s^3/sqrt(Hz). */
    double gyro_misalign[9]; /**< 3x3 row-major cross-axis coupling matrix. */
    double gyro_scale[3]; /**< Per-axis scale factors. */
    double gyro_bias[3]; /**< Per-axis bias in rad/s. */
    double gyro_noise;           /**< Gyroscope noise density, rad/s/sqrt(Hz). */
    double gyro_random_walk;     /**< Gyroscope random walk, rad/s^2/sqrt(Hz). */
}gs130_imu_intrinsics_t;

/** Complete camera, IMU, and extrinsic calibration. */
typedef struct gs130_calibration_s{
    gs130_imu_intrinsics_t imu;
    gs130_camera_intrinsics_t camera_right;
    gs130_camera_intrinsics_t camera_left;

    /**
     * Extrinsics are absolute poses expressed as sensor-to-reference transforms:
     * p_reference = R * p_sensor + T. The EEPROM driver selects the initial
     * reference frame. Rectification preserves that reference but replaces the
     * camera poses with virtual parallel-stereo poses; consumers must account for
     * that change when using rectified calibration.
     */
    double camera_right_R[9], camera_right_T[3];
    double camera_left_R[9], camera_left_T[3];
    double imu_R[9], imu_T[3];

    int camera_install_angle; /**< Installation rotation in degrees; normalized modulo 360 and required to be a multiple of 90. */
}gs130_calibration_t;

/** Frames that may be used as calibration references. */
typedef enum gs130_reference_frame_e{
    GS130_REF_CAMERA_RIGHT, /**< Right camera frame. */
    GS130_REF_CAMERA_LEFT,  /**< Left camera frame. */
    GS130_REF_IMU,          /**< IMU frame. */
}gs130_reference_frame_t;

/**
 * @brief Get the camera intrinsics.
 *
 * This query is valid after initialization, whether capture is running or idle.
 *
 * @param[in]  dev        Initialized device handle.
 * @param[in]  cam_idx    GS130_CAMERA_RIGHT_IDX or GS130_CAMERA_LEFT_IDX.
 * @param[out] intrinsics Destination for the camera intrinsics.
 * @retval GS130_OK Intrinsics were returned.
 * @retval GS130_PARAM_ERROR An argument or camera index is invalid.
 * @retval GS130_NOT_FOUND No calibration EEPROM was detected.
 * @retval GS130_HW_ERROR The device has a latched hardware failure.
 */
gs130_err_t gs130_get_camera_intrinsics(
    gs130_device_t *dev,
    gs130_camera_index_t cam_idx,
    gs130_camera_intrinsics_t *intrinsics);

/**
 * @brief Get the IMU intrinsics.
 *
 * This query is valid after initialization, whether capture is running or idle.
 *
 * @param[in]  dev        Initialized device handle.
 * @param[out] intrinsics Destination for the IMU intrinsics.
 * @retval GS130_OK Intrinsics were returned.
 * @retval GS130_PARAM_ERROR An argument is invalid.
 * @retval GS130_NOT_FOUND No calibration EEPROM was detected.
 * @retval GS130_HW_ERROR The device has a latched hardware failure.
 */
gs130_err_t gs130_get_imu_intrinsics(
    gs130_device_t *dev,
    gs130_imu_intrinsics_t *intrinsics);

/**
 * @brief Get the relative rotation matrix between two devices.
 *
 * @param[in]  dev        Initialized device handle.
 * @param[in]  from_frame Frame in which the input point is expressed.
 * @param[in]  to_frame   Frame in which the output point is expressed.
 * @param[out] R          Row-major 3x3 rotation satisfying p_to = R * p_from.
 * @retval GS130_OK The rotation was returned.
 * @retval GS130_PARAM_ERROR An argument or frame identifier is invalid.
 * @retval GS130_NOT_FOUND No calibration EEPROM was detected.
 * @retval GS130_HW_ERROR The device has a latched hardware failure.
 *
 * @note For identical frames, the result is numerically close to the identity
 *       matrix; it may contain floating-point roundoff.
 */
gs130_err_t gs130_get_relative_R(
    gs130_device_t *dev,
    gs130_reference_frame_t from_frame,
    gs130_reference_frame_t to_frame,
    double *R);

/**
 * @brief Get the relative translation vector between two devices.
 *
 * Use this vector with gs130_get_relative_R() as p_to = R * p_from + T.
 *
 * @param[in]  dev        Initialized device handle.
 * @param[in]  from_frame Frame in which the input point is expressed.
 * @param[in]  to_frame   Frame in which the output point is expressed.
 * @param[out] T          Three-element translation vector expressed in to_frame.
 * @retval GS130_OK The translation was returned.
 * @retval GS130_PARAM_ERROR An argument or frame identifier is invalid.
 * @retval GS130_NOT_FOUND No calibration EEPROM was detected.
 * @retval GS130_HW_ERROR The device has a latched hardware failure.
 *
 * @note For identical frames, the result is numerically close to zero; it may
 *       contain floating-point roundoff.
 */
gs130_err_t gs130_get_relative_T(
    gs130_device_t *dev,
    gs130_reference_frame_t from_frame,
    gs130_reference_frame_t to_frame,
    double *T);

/**
 * @brief Get the complete stereo (and IMU) calibration.
 *
 * The result includes any virtual intrinsics/extrinsics produced by the selected
 * pipeline mode and any reference-frame conversion applied since initialization.
 *
 * @param[in]  dev         Initialized device handle.
 * @param[out] calibration Destination for the complete calibration.
 * @retval GS130_OK Calibration was returned.
 * @retval GS130_PARAM_ERROR An argument is invalid.
 * @retval GS130_NOT_FOUND No calibration EEPROM was detected.
 * @retval GS130_HW_ERROR The device has a latched hardware failure.
 */
gs130_err_t gs130_get_calibration(
    gs130_device_t *dev,
    gs130_calibration_t *calibration);

/**
 * @brief Change the reference frame of the devices.
 *
 * Sets the selected device pose to (ref_R, ref_T) and transforms every other
 * device pose by the same rigid transform, preserving all relative poses.
 *
 * @param[in] dev       Initialized device handle.
 * @param[in] ref_frame Device whose pose is being assigned.
 * @param[in] ref_R     Row-major 3x3 sensor-to-new-reference rotation matrix.
 * @param[in] ref_T     Three-element sensor-to-new-reference translation vector.
 * @retval GS130_OK The in-memory calibration was converted.
 * @retval GS130_PARAM_ERROR An argument or frame identifier is invalid.
 * @retval GS130_NOT_FOUND No calibration EEPROM was detected.
 * @retval GS130_HW_ERROR The device has a latched hardware failure.
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
 * @brief Get the model name of the detected calibration EEPROM.
 *
 * @param[in] dev Initialized device handle.
 * @return Statically allocated EEPROM model string, or NULL if no supported
 *         EEPROM was detected. The caller must not modify or free it.
 */
const char *gs130_get_eeprom_name(
    gs130_device_t *dev);

/**
 * @brief Get details about the detected calibration EEPROM.
 *
 * The returned text may include the vendor, format version, device type, and
 * distortion model.
 *
 * @param[in] dev Initialized device handle.
 * @return Statically allocated information string, or NULL if no supported
 *         EEPROM was detected. The caller must not modify or free it.
 */
const char *gs130_get_eeprom_info(
    gs130_device_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* GS130_H */
