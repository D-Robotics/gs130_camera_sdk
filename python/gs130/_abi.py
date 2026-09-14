"""Exact ctypes declarations for ``core/include/gs130.h``.

This module contains no Python-facing policy. C enums are represented by
``c_int`` at the ABI boundary, as required by the platform ABI.
"""

import ctypes as C


class FifoConfig(C.Structure):
    _fields_ = [("depth", C.c_size_t), ("mode", C.c_int)]


class CameraConfig(C.Structure):
    _fields_ = [
        ("bus", C.c_uint8 * 32),
        ("bus_num", C.c_size_t),
        ("left_addr", C.c_uint8),
        ("right_addr", C.c_uint8),
        ("sensor_width", C.c_uint32),
        ("sensor_height", C.c_uint32),
        ("fps", C.c_uint32),
        ("line_length", C.c_uint32),
        ("frame_length", C.c_uint32),
        ("tuning_file", C.c_char_p),
        ("output_width", C.c_uint32),
        ("output_height", C.c_uint32),
        ("mode", C.c_int),
        ("stereo_layout", C.c_int),
        ("bus_mipi_rx", C.c_uint8 * 32),
        ("bus_reset_gpio", C.c_int * 32),
        ("fsync_camera", C.c_int),
    ]


class ImuConfig(C.Structure):
    _fields_ = [
        ("bus", C.c_uint8 * 32),
        ("bus_num", C.c_size_t),
        ("addr", C.c_uint8),
        ("odr_hz", C.c_uint32),
        ("accel_fsr_g", C.c_uint16),
        ("gyro_fsr_dps", C.c_uint16),
        ("accel_bw_sel", C.c_uint8),
        ("gyro_bw_sel", C.c_uint8),
    ]


class EepromConfig(C.Structure):
    _fields_ = [
        ("bus", C.c_uint8 * 32),
        ("bus_num", C.c_size_t),
        ("addr", C.c_uint8),
    ]


class Config(C.Structure):
    _fields_ = [
        ("camera_config", CameraConfig),
        ("imu_config", ImuConfig),
        ("eeprom_config", EepromConfig),
        ("camera_fifo", FifoConfig),
        ("imu_fifo", FifoConfig),
    ]


Device = C.c_void_p


class ImageNV12(C.Structure):
    _fields_ = [
        ("data", C.POINTER(C.c_uint8)),
        ("width", C.c_uint32),
        ("height", C.c_uint32),
        ("timestamp_ns", C.c_uint64),
    ]


class ImuPacket(C.Structure):
    _fields_ = [
        ("accel", C.c_float * 3),
        ("gyro", C.c_float * 3),
        ("temp", C.c_float),
        ("is_fsync", C.c_bool),
        ("timestamp_ns", C.c_uint64),
    ]


class CameraIntrinsics(C.Structure):
    _fields_ = [
        ("fx", C.c_double),
        ("fy", C.c_double),
        ("cx", C.c_double),
        ("cy", C.c_double),
        ("K", C.c_double * 9),
        ("dist_coeffs", C.c_double * 8),
        ("dist_model", C.c_int),
    ]


class ImuIntrinsics(C.Structure):
    _fields_ = [
        ("accel_misalign", C.c_double * 9),
        ("accel_scale", C.c_double * 3),
        ("accel_bias", C.c_double * 3),
        ("accel_noise", C.c_double),
        ("accel_random_walk", C.c_double),
        ("gyro_misalign", C.c_double * 9),
        ("gyro_scale", C.c_double * 3),
        ("gyro_bias", C.c_double * 3),
        ("gyro_noise", C.c_double),
        ("gyro_random_walk", C.c_double),
    ]


class Calibration(C.Structure):
    _fields_ = [
        ("imu", ImuIntrinsics),
        ("camera_right", CameraIntrinsics),
        ("camera_left", CameraIntrinsics),
        ("camera_right_R", C.c_double * 9),
        ("camera_right_T", C.c_double * 3),
        ("camera_left_R", C.c_double * 9),
        ("camera_left_T", C.c_double * 3),
        ("imu_R", C.c_double * 9),
        ("imu_T", C.c_double * 3),
        ("camera_install_angle", C.c_int),
    ]


SIGNATURES = {
    "gs130_version": (C.c_char_p, []),
    "gs130_platform": (C.c_char_p, []),
    "gs130_create": (Device, []),
    "gs130_destroy": (None, [Device]),
    "gs130_init": (C.c_int, [Device, C.POINTER(Config)]),
    "gs130_deinit": (C.c_int, [Device]),
    "gs130_start": (C.c_int, [Device]),
    "gs130_stop": (None, [Device]),
    "gs130_available_camera": (C.c_size_t, [Device]),
    "gs130_get_nv12_frame": (
        C.c_int,
        [Device, C.POINTER(ImageNV12), C.POINTER(ImageNV12)],
    ),
    "gs130_get_stereo_nv12_frame": (
        C.c_int,
        [Device, C.POINTER(ImageNV12)],
    ),
    "gs130_available_imu": (C.c_size_t, [Device]),
    "gs130_get_imu_packet": (C.c_int, [Device, C.POINTER(ImuPacket)]),
    "gs130_get_imu_name": (C.c_char_p, [Device]),
    "gs130_get_imu_info": (C.c_char_p, [Device]),
    "gs130_get_camera_intrinsics": (
        C.c_int,
        [Device, C.c_int, C.POINTER(CameraIntrinsics)],
    ),
    "gs130_get_imu_intrinsics": (
        C.c_int,
        [Device, C.POINTER(ImuIntrinsics)],
    ),
    "gs130_get_relative_R": (
        C.c_int,
        [Device, C.c_int, C.c_int, C.POINTER(C.c_double)],
    ),
    "gs130_get_relative_T": (
        C.c_int,
        [Device, C.c_int, C.c_int, C.POINTER(C.c_double)],
    ),
    "gs130_get_calibration": (C.c_int, [Device, C.POINTER(Calibration)]),
    "gs130_convert_calibration": (
        C.c_int,
        [Device, C.c_int, C.POINTER(C.c_double), C.POINTER(C.c_double)],
    ),
    "gs130_get_eeprom_name": (C.c_char_p, [Device]),
    "gs130_get_eeprom_info": (C.c_char_p, [Device]),
}
