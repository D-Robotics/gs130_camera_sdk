/**
 * @file types.hpp
 * @brief Internal C++ shared types: status codes, configuration, calibration, and
 *        FIFO policy.
 *
 * Every type here is an internal detail of the library; the customer-facing
 * equivalents live in include/gs130.h and are converted at the C API boundary.
 * Conventions used throughout this header: SI units with the unit in the field
 * name, row-major matrices, and OpenCV parameter orders.
 *
 * This file is part of gs130_camera_sdk (https://github.com/D-Robotics/gs130_camera_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#ifndef GS130_TYPES_HPP
#define GS130_TYPES_HPP

#include <cstdint>

namespace gs130 {

// ============================== Error codes ==============================

/**
 * Internal result code, returned by every internal call that can fail and mapped
 * 1:1 onto ::gs130_err_t at the C API boundary.  Callers must compare against
 * Status::Ok rather than testing for a specific failure.
 */
enum class Status {
    Ok,
    ParamError,     // the parameter itself is invalid (null pointer, required field is 0, etc.)
    Unsupported,    // hardware cannot do this configuration; never degrade, round, or substitute
    NotFound,       // device or model not detected
    HwError,        // low-level communication or driver failure
    Timeout,        // wait timed out
    ThreadClosed,   // threads closed or never started
};

// ============================== Camera & output ==============================

/**
 * Logical camera index, also the subscripts of the per-camera arrays.
 * @note These are logical roles fixed by the device wiring, not USB/I2C enumeration
 *       order, and the index identifies a camera only within one pipeline instance.
 */
enum class CamIndex {
    Right = 0,
    Left  = 1,
    Num   = 2,
};

/** Processing pipeline selected for both cameras; the comment traces each stage. */
enum class OutputMode {
    Raw,      // CAM -> VIN -> ISP -> OUT
    Resize,   // CAM -> VIN -> ISP -> VSE -> OUT
    Rect,     // CAM -> VIN -> ISP -> GDC -> VSE -> OUT
};

/**
 * Platform pipeline configuration for one camera pair, already resolved for the
 * target board by the caller.
 *
 * The arrays are indexed by I2C bus number, not by CamIndex. Pointer values are
 * borrowed for the duration of Pipeline::init(); the configuration owns no memory.
 */
struct PipelineConfig {
    // Platform mapping: I2C bus -> MIPI RX / reset GPIO.
    // Looked up by the bus the camera is detected on; buses may be swapped, so key by bus, not camera index.
    // bus_mipi_rx 0xFF = not configured; bus_reset_gpio -1 = do not control.
    uint8_t bus_mipi_rx[32];
    int     bus_reset_gpio[32];

    /** Sensor output size in pixels, i.e. the VIN/ISP input frame. */
    uint32_t sensor_width, sensor_height;

    /** Requested frame rate in frames per second; must match the sensor timing registers. */
    uint32_t fps;
    OutputMode mode;

    /** Output size in pixels; required for Resize/Rect and equal to the sensor size in Raw. */
    uint32_t output_width;
    uint32_t output_height;

    /** Sensor line and frame timing values passed directly to the platform driver. */
    uint32_t line_length;
    uint32_t frame_length;

    const char *tuning_file;   // Borrowed ISP tuning path; nullptr disables loading.
};

// ============================== IMU ==============================

/**
 * One IMU sample as read back from the hardware FIFO, still in raw sensor units:
 * the driver performs no scaling, so the consumer converts with the configured
 * full-scale range and the model's sensitivity.
 *
 * Fields map onto the 16-byte ICM-family FIFO packet as read from the device
 * (big-endian on the wire, already converted to host byte order here):
 * bytes 1..12 are the signed big-endian accel/gyro words, byte 13 the temperature,
 * bytes 14..15 the header-dependent timestamp/delta word.
 */
struct ImuHwFifo16Packet {
    /** Signed raw accelerometer counts, axes in the order and sign of the sensor package. */
    int16_t  accel[3];
    /** Signed raw gyroscope counts, same axis convention as accel. */
    int16_t  gyro[3];
    /** Raw 8-bit temperature code; the scale factor is model-specific. */
    int8_t   temp;
    /** True when the packet carried an FSYNC/timestamp header. */
    bool     is_fsync;
    uint32_t delta_time_us;   // valid only when is_fsync: edge-to-sample offset
};

/**
 * Requested IMU sampling configuration, passed through to the model driver.
 * The driver accepts the hardware encodings verbatim and rejects anything else with
 * Status::Unsupported instead of rounding or substituting a value.
 */
struct ImuConfig {
    /** Output data rate in Hz; supported values are model-specific (see ModelDesc::info). */
    uint32_t odr_hz;
    /** Accelerometer full-scale range in g (1 g = 9.80665 m/s^2). */
    uint16_t accel_fsr_g;
    /** Gyroscope full-scale range in degrees per second. */
    uint16_t gyro_fsr_dps;
    uint8_t  accel_bw_sel;   // accel UI filter setting 0..F; see info() for the bandwidths
    uint8_t  gyro_bw_sel;    // gyro UI filter setting 0..F; see info() for the bandwidths
};

// ============================== Calibration ==============================

/**
 * Distortion model that defines how CameraIntrinsics::dist_coeffs is read; the
 * coefficient order is the OpenCV one, and unused entries are zero.
 */
enum class DistModel {
    Pinhole,        // dist_coeffs = [k1,k2,p1,p2,k3,k4,k5,k6]
    Fisheye,        // dist_coeffs = [k1,k2,k3,k4,0,0,0,0]
};

/**
 * Intrinsics of one camera.  Rectification rewrites these in place, see
 * base::stereo_rectify().
 *
 * All values are in pixels; the principal point is measured from the top-left corner
 * of the image, with x to the right and y downwards.
 */
struct CameraIntrinsics {
    double fx, fy, cx, cy;
    /** Intrinsic matrix, row-major; it is a duplicate of fx/fy/cx/cy with the last row (0,0,1). */
    double K[9], dist_coeffs[8];
    /** Model used to interpret dist_coeffs; left and right cameras must agree. */
    DistModel dist_model = DistModel::Fisheye;
};

/**
 * IMU calibration and noise characteristics as stored in the EEPROM.
 * All three-element vectors are per-axis in sensor axis order, all 3x3 matrices are
 * row-major, and the units follow the field name.
 */
struct ImuIntrinsics {
    double accel_misalign[9];    // cross-axis coupling 3x3
    double accel_scale[3];
    double accel_bias[3];
    double accel_noise;          // m/s^2/sqrt(Hz)
    double accel_random_walk;    // m/s^3/sqrt(Hz)
    double gyro_misalign[9];
    double gyro_scale[3];
    double gyro_bias[3];
    double gyro_noise;           // rad/s/sqrt(Hz)
    double gyro_random_walk;     // rad/s^2/sqrt(Hz)
};

/**
 * Full stereo + IMU calibration. The EEPROM model driver chooses the initial
 * reference frame and supplies all three absolute sensor poses in that frame.
 *
 * @note Each of the three extrinsic sets is a "sensor frame -> common reference
 *       frame" transform, i.e. p_ref = R * p_sensor + T, not a transform between
 *       devices.
 * @note The external API can re-express the same calibration against another device
 *       with gs130_convert_calibration(); that does not recompute the rectification.
 */
struct StereoImuModel {
    ImuIntrinsics    imu;
    CameraIntrinsics cam_right;
    CameraIntrinsics cam_left;

    double cam_right_R[9];
    double cam_right_T[3];
    double cam_left_R[9];
    double cam_left_T[3];
    double imu_R[9];
    double imu_T[3];
    /** Installation rotation in degrees, a multiple of 90; 0 for layouts without rotation. */
    int    install_angle;
};

// ============================== Stereo rectification ==============================

/**
 * Remap coordinate point: output pixel -> source-image sampling coordinate.
 *
 * The RDK X5 backend reinterprets this layout as the Horizon point_t type. No
 * compile-time layout assertion currently enforces that assumption, so both types
 * must remain two consecutive double values.
 */
struct RemapPoint {
    /** Source-image sampling position in pixels; values may be fractional. */
    double x, y;
};

// ============================== FIFO ==============================

/** Policy applied by base::Fifo when the queue is full. */
enum class FifoMode {
    DropNew,   // drop new data when full
    DropOld,   // overwrite oldest data when full
};

} // namespace gs130

#endif // GS130_TYPES_HPP
