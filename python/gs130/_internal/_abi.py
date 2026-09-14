"""ctypes mirror of core/include/gs130.h: internal to the gs130 package.

Declarations follow the header: same names, same order, same fields. Where C
and ctypes spell a type differently the mapping is fixed: uint8_t -> c_uint8,
size_t -> c_size_t, bool -> c_bool, T[N] -> c_type * N, an enum -> c_int, a
const char * -> c_char_p, uint8_t * -> POINTER(c_uint8), and a const T * ->
POINTER(T).
"""

from ctypes import (
    POINTER,
    Structure,
    c_bool,
    c_char_p,
    c_double,
    c_float,
    c_int,
    c_size_t,
    c_uint8,
    c_uint16,
    c_uint32,
    c_uint64,
    c_void_p,
)
from enum import IntEnum


# gs130_err_t
class gs130_err_t(IntEnum):
    GS130_OK = 0
    GS130_PARAM_ERROR = 1
    GS130_UNSUPPORTED = 2
    GS130_NOT_FOUND = 3
    GS130_HW_ERROR = 4
    GS130_TIMEOUT = 5
    GS130_THREAD_CLOSED = 6


# gs130_fifo_mode_t
class gs130_fifo_mode_t(IntEnum):
    GS130_FIFO_DROP_NEW = 0
    GS130_FIFO_DROP_OLD = 1


# gs130_fifo_config_t
class gs130_fifo_config_t(Structure):
    _fields_ = [
        ("depth", c_size_t),
        ("mode", c_int),
    ]


# ==================== Device Config ====================


# gs130_camera_mode_t
class gs130_camera_mode_t(IntEnum):
    GS130_CAMERA_MODE_RAW = 0
    GS130_CAMERA_MODE_RESIZE = 1
    GS130_CAMERA_MODE_RECT = 2


# gs130_camera_index_t
class gs130_camera_index_t(IntEnum):
    GS130_CAMERA_RIGHT_IDX = 0
    GS130_CAMERA_LEFT_IDX = 1


# gs130_stereo_layout_t
class gs130_stereo_layout_t(IntEnum):
    GS130_STEREO_LAYOUT_NONE = 0
    GS130_STEREO_LAYOUT_LEFT_RIGHT = 1
    GS130_STEREO_LAYOUT_RIGHT_LEFT = 2
    GS130_STEREO_LAYOUT_TOP_BOTTOM = 3
    GS130_STEREO_LAYOUT_BOTTOM_TOP = 4


# gs130_camera_config_t
class gs130_camera_config_t(Structure):
    _fields_ = [
        ("bus", c_uint8 * 32),
        ("bus_num", c_size_t),
        ("left_addr", c_uint8),
        ("right_addr", c_uint8),
        ("sensor_width", c_uint32),
        ("sensor_height", c_uint32),
        ("fps", c_uint32),
        ("line_length", c_uint32),
        ("frame_length", c_uint32),
        ("tuning_file", c_char_p),
        ("output_width", c_uint32),
        ("output_height", c_uint32),
        ("mode", c_int),
        ("stereo_layout", c_int),
        ("bus_mipi_rx", c_uint8 * 32),
        ("bus_reset_gpio", c_int * 32),
        ("fsync_camera", c_int),
    ]


# gs130_imu_config_t
class gs130_imu_config_t(Structure):
    _fields_ = [
        ("bus", c_uint8 * 32),
        ("bus_num", c_size_t),
        ("addr", c_uint8),
        ("odr_hz", c_uint32),
        ("accel_fsr_g", c_uint16),
        ("gyro_fsr_dps", c_uint16),
        ("accel_bw_sel", c_uint8),
        ("gyro_bw_sel", c_uint8),
    ]


# gs130_eeprom_config_t
class gs130_eeprom_config_t(Structure):
    _fields_ = [
        ("bus", c_uint8 * 32),
        ("bus_num", c_size_t),
        ("addr", c_uint8),
    ]


# gs130_config_t
class gs130_config_t(Structure):
    _fields_ = [
        ("camera_config", gs130_camera_config_t),
        ("imu_config", gs130_imu_config_t),
        ("eeprom_config", gs130_eeprom_config_t),
        ("camera_fifo", gs130_fifo_config_t),
        ("imu_fifo", gs130_fifo_config_t),
    ]


# typedef struct gs130_device_s gs130_device_t; -- opaque, only ever a handle
gs130_device_t = c_void_p


# ==================== Camera Data ====================


# gs130_image_nv12_t
class gs130_image_nv12_t(Structure):
    _fields_ = [
        ("data", POINTER(c_uint8)),
        ("width", c_uint32),
        ("height", c_uint32),
        ("timestamp_ns", c_uint64),
    ]


# ==================== IMU Data ====================


# gs130_imu_packet_t
class gs130_imu_packet_t(Structure):
    _fields_ = [
        ("accel", c_float * 3),
        ("gyro", c_float * 3),
        ("temp", c_float),
        ("is_fsync", c_bool),
        ("timestamp_ns", c_uint64),
    ]


# ==================== EEPROM Data ====================


# gs130_dist_model_t
class gs130_dist_model_t(IntEnum):
    GS130_DIST_PINHOLE = 0
    GS130_DIST_FISHEYE = 1


# gs130_camera_intrinsics_t
class gs130_camera_intrinsics_t(Structure):
    _fields_ = [
        ("fx", c_double),
        ("fy", c_double),
        ("cx", c_double),
        ("cy", c_double),
        ("K", c_double * 9),
        ("dist_coeffs", c_double * 8),
        ("dist_model", c_int),
    ]


# gs130_imu_intrinsics_t
class gs130_imu_intrinsics_t(Structure):
    _fields_ = [
        ("accel_misalign", c_double * 9),
        ("accel_scale", c_double * 3),
        ("accel_bias", c_double * 3),
        ("accel_noise", c_double),
        ("accel_random_walk", c_double),
        ("gyro_misalign", c_double * 9),
        ("gyro_scale", c_double * 3),
        ("gyro_bias", c_double * 3),
        ("gyro_noise", c_double),
        ("gyro_random_walk", c_double),
    ]


# gs130_calibration_t
class gs130_calibration_t(Structure):
    _fields_ = [
        ("imu", gs130_imu_intrinsics_t),
        ("camera_right", gs130_camera_intrinsics_t),
        ("camera_left", gs130_camera_intrinsics_t),
        ("camera_right_R", c_double * 9),
        ("camera_right_T", c_double * 3),
        ("camera_left_R", c_double * 9),
        ("camera_left_T", c_double * 3),
        ("imu_R", c_double * 9),
        ("imu_T", c_double * 3),
        ("camera_install_angle", c_int),
    ]


# gs130_reference_frame_t
class gs130_reference_frame_t(IntEnum):
    GS130_REF_CAMERA_RIGHT = 0
    GS130_REF_CAMERA_LEFT = 1
    GS130_REF_IMU = 2


# Return type and argument types of every function in gs130.h, in the order the
# header declares them. ctypes needs them at load time, so they live in one
# table instead of next to the declaration each one belongs to.
_SIGNATURES = {
    "gs130_version": (c_char_p, []),
    "gs130_platform": (c_char_p, []),
    "gs130_create": (gs130_device_t, []),
    "gs130_destroy": (None, [gs130_device_t]),
    "gs130_init": (c_int, [gs130_device_t, POINTER(gs130_config_t)]),
    "gs130_deinit": (c_int, [gs130_device_t]),
    "gs130_start": (c_int, [gs130_device_t]),
    "gs130_stop": (None, [gs130_device_t]),
    "gs130_available_camera": (c_size_t, [gs130_device_t]),
    "gs130_get_nv12_frame": (
        c_int,
        [
            gs130_device_t,
            POINTER(gs130_image_nv12_t),
            POINTER(gs130_image_nv12_t),
        ],
    ),
    "gs130_get_stereo_nv12_frame": (
        c_int,
        [gs130_device_t, POINTER(gs130_image_nv12_t)],
    ),
    "gs130_available_imu": (c_size_t, [gs130_device_t]),
    "gs130_get_imu_packet": (
        c_int,
        [gs130_device_t, POINTER(gs130_imu_packet_t)],
    ),
    "gs130_get_imu_name": (c_char_p, [gs130_device_t]),
    "gs130_get_imu_info": (c_char_p, [gs130_device_t]),
    "gs130_get_camera_intrinsics": (
        c_int,
        [gs130_device_t, c_int, POINTER(gs130_camera_intrinsics_t)],
    ),
    "gs130_get_imu_intrinsics": (
        c_int,
        [gs130_device_t, POINTER(gs130_imu_intrinsics_t)],
    ),
    "gs130_get_relative_R": (
        c_int,
        [gs130_device_t, c_int, c_int, POINTER(c_double)],
    ),
    "gs130_get_relative_T": (
        c_int,
        [gs130_device_t, c_int, c_int, POINTER(c_double)],
    ),
    "gs130_get_calibration": (
        c_int,
        [gs130_device_t, POINTER(gs130_calibration_t)],
    ),
    "gs130_convert_calibration": (
        c_int,
        [gs130_device_t, c_int, POINTER(c_double), POINTER(c_double)],
    ),
    "gs130_get_eeprom_name": (c_char_p, [gs130_device_t]),
    "gs130_get_eeprom_info": (c_char_p, [gs130_device_t]),
}
