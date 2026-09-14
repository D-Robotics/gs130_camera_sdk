import ctypes
import ctypes.util
import os
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
from pathlib import Path

# gs130_err_t
class GS130Err(IntEnum):
    GS130_OK = 0
    GS130_PARAM_ERROR = 1
    GS130_UNSUPPORTED = 2
    GS130_NOT_FOUND = 3
    GS130_HW_ERROR = 4
    GS130_TIMEOUT = 5
    GS130_THREAD_CLOSED = 6


# gs130_fifo_mode_t
class GS130FifoMode(IntEnum):
    GS130_FIFO_DROP_NEW = 0
    GS130_FIFO_DROP_OLD = 1


# gs130_camera_mode_t
class GS130CameraMode(IntEnum):
    GS130_CAMERA_MODE_RAW = 0
    GS130_CAMERA_MODE_RESIZE = 1
    GS130_CAMERA_MODE_RECT = 2


# gs130_camera_index_t
class GS130CameraIndex(IntEnum):
    GS130_CAMERA_RIGHT_IDX = 0
    GS130_CAMERA_LEFT_IDX = 1


# gs130_stereo_layout_t
class GS130StereoLayout(IntEnum):
    GS130_STEREO_LAYOUT_NONE = 0
    GS130_STEREO_LAYOUT_LEFT_RIGHT = 1
    GS130_STEREO_LAYOUT_RIGHT_LEFT = 2
    GS130_STEREO_LAYOUT_TOP_BOTTOM = 3
    GS130_STEREO_LAYOUT_BOTTOM_TOP = 4


# gs130_dist_model_t
class GS130DistModel(IntEnum):
    GS130_DIST_PINHOLE = 0
    GS130_DIST_FISHEYE = 1


# gs130_reference_frame_t
class GS130ReferenceFrame(IntEnum):
    GS130_REF_CAMERA_RIGHT = 0
    GS130_REF_CAMERA_LEFT = 1
    GS130_REF_IMU = 2

# gs130_camera_config_t.bus / gs130_camera_intrinsics_t.dist_coeffs
BUS_MAX = 32
DIST_COEFFS = 8

_LIB_NAMES = ("libgs130.so.0", "libgs130.so")
_HBN_LIBS = ("libvpf.so", "libhbmem.so", "libcam.so")


# gs130_fifo_config_t
class GS130FifoConfig(Structure):
    _fields_ = [
        ("depth", c_size_t),
        ("mode", c_int),
    ]


# gs130_camera_config_t
class GS130CameraConfig(Structure):
    _fields_ = [
        ("bus", c_uint8 * BUS_MAX),
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
        ("bus_mipi_rx", c_uint8 * BUS_MAX),
        ("bus_reset_gpio", c_int * BUS_MAX),
        ("fsync_camera", c_int),
    ]


# gs130_imu_config_t
class GS130ImuConfig(Structure):
    _fields_ = [
        ("bus", c_uint8 * BUS_MAX),
        ("bus_num", c_size_t),
        ("addr", c_uint8),
        ("odr_hz", c_uint32),
        ("accel_fsr_g", c_uint16),
        ("gyro_fsr_dps", c_uint16),
        ("accel_bw_sel", c_uint8),
        ("gyro_bw_sel", c_uint8),
    ]


# gs130_eeprom_config_t
class GS130EepromConfig(Structure):
    _fields_ = [
        ("bus", c_uint8 * BUS_MAX),
        ("bus_num", c_size_t),
        ("addr", c_uint8),
    ]


# gs130_config_t
class GS130Config(Structure):
    _fields_ = [
        ("camera_config", GS130CameraConfig),
        ("imu_config", GS130ImuConfig),
        ("eeprom_config", GS130EepromConfig),
        ("camera_fifo", GS130FifoConfig),
        ("imu_fifo", GS130FifoConfig),
    ]


# gs130_image_nv12_t
class GS130ImageNV12(Structure):
    _fields_ = [
        ("data", c_void_p),
        ("width", c_uint32),
        ("height", c_uint32),
        ("timestamp_ns", c_uint64),
    ]


# gs130_imu_packet_t
class GS130ImuPacket(Structure):
    _fields_ = [
        ("accel", c_float * 3),
        ("gyro", c_float * 3),
        ("temp", c_float),
        ("is_fsync", c_bool),
        ("timestamp_ns", c_uint64),
    ]


# gs130_camera_intrinsics_t
class GS130CameraIntrinsics(Structure):
    _fields_ = [
        ("fx", c_double),
        ("fy", c_double),
        ("cx", c_double),
        ("cy", c_double),
        ("K", c_double * 9),
        ("dist_coeffs", c_double * DIST_COEFFS),
        ("dist_model", c_int),
    ]


# gs130_imu_intrinsics_t
class GS130ImuIntrinsics(Structure):
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
class GS130Calibration(Structure):
    _fields_ = [
        ("imu", GS130ImuIntrinsics),
        ("camera_right", GS130CameraIntrinsics),
        ("camera_left", GS130CameraIntrinsics),
        ("camera_right_R", c_double * 9),
        ("camera_right_T", c_double * 3),
        ("camera_left_R", c_double * 9),
        ("camera_left_T", c_double * 3),
        ("imu_R", c_double * 9),
        ("imu_T", c_double * 3),
        ("camera_install_angle", c_int),
    ]


_C_TYPES = {
    "gs130_fifo_config_t": GS130FifoConfig,
    "gs130_camera_config_t": GS130CameraConfig,
    "gs130_imu_config_t": GS130ImuConfig,
    "gs130_eeprom_config_t": GS130EepromConfig,
    "gs130_config_t": GS130Config,
    "gs130_image_nv12_t": GS130ImageNV12,
    "gs130_imu_packet_t": GS130ImuPacket,
    "gs130_camera_intrinsics_t": GS130CameraIntrinsics,
    "gs130_imu_intrinsics_t": GS130ImuIntrinsics,
    "gs130_calibration_t": GS130Calibration,
}

_C_TYPES_REVERSE = {cls: name for name, cls in _C_TYPES.items()}

_SIGNATURES = {
    "gs130_version": (c_char_p, []),
    "gs130_platform": (c_char_p, []),
    "gs130_create": (c_void_p, []),
    "gs130_destroy": (None, [c_void_p]),
    "gs130_init": (c_int, [c_void_p, POINTER(GS130Config)]),
    "gs130_deinit": (c_int, [c_void_p]),
    "gs130_start": (c_int, [c_void_p]),
    "gs130_stop": (None, [c_void_p]),
    "gs130_available_camera": (c_size_t, [c_void_p]),
    "gs130_get_nv12_frame": (
        c_int,
        [c_void_p, POINTER(GS130ImageNV12), POINTER(GS130ImageNV12)],
    ),
    "gs130_get_stereo_nv12_frame": (c_int, [c_void_p, POINTER(GS130ImageNV12)]),
    "gs130_available_imu": (c_size_t, [c_void_p]),
    "gs130_get_imu_packet": (c_int, [c_void_p, POINTER(GS130ImuPacket)]),
    "gs130_get_imu_name": (c_char_p, [c_void_p]),
    "gs130_get_imu_info": (c_char_p, [c_void_p]),
    "gs130_get_camera_intrinsics": (
        c_int,
        [c_void_p, c_int, POINTER(GS130CameraIntrinsics)],
    ),
    "gs130_get_imu_intrinsics": (c_int, [c_void_p, POINTER(GS130ImuIntrinsics)]),
    "gs130_get_relative_R": (
        c_int,
        [c_void_p, c_int, c_int, POINTER(c_double)],
    ),
    "gs130_get_relative_T": (
        c_int,
        [c_void_p, c_int, c_int, POINTER(c_double)],
    ),
    "gs130_get_calibration": (c_int, [c_void_p, POINTER(GS130Calibration)]),
    "gs130_convert_calibration": (
        c_int,
        [c_void_p, c_int, POINTER(c_double), POINTER(c_double)],
    ),
    "gs130_get_eeprom_name": (c_char_p, [c_void_p]),
    "gs130_get_eeprom_info": (c_char_p, [c_void_p]),
}

# GS130_CONFIG_RDKX5_GS130WI / GS130_CONFIG_RDKX5_GS130W
_PRESETS = {
    "RDKX5": {
        "GS130WI": {
            "left_addr": 0x30,
            "right_addr": 0x32,
            "imu_bus": (4, 6),
            "imu_bus_num": 2,
            "imu_fifo_depth": 1024,
            "imu_fifo_mode": GS130FifoMode.GS130_FIFO_DROP_OLD,
        },
        "GS130W": {
            "left_addr": 0x30,
            "right_addr": 0x31,
            "imu_bus": (),
            "imu_bus_num": 0,
            "imu_fifo_depth": 0,
            "imu_fifo_mode": GS130FifoMode.GS130_FIFO_DROP_NEW,
        },
    },
}

_CAMERA_BUS = (4, 6)
_CAMERA_MIPI_RX = {4: 2, 6: 0}
_CAMERA_RESET_GPIO = {4: 351, 6: 353}

_lib = None


def preset(platform, device, mode, width, height, fps, odr):
    table = _PRESETS.get(platform, {}).get(device)
    if table is None:
        raise ValueError("unsupported platform/device: %s %s" % (platform, device))
    cfg = GS130Config()
    camera = cfg.camera_config
    camera.bus_num = len(_CAMERA_BUS)
    for index, bus in enumerate(_CAMERA_BUS):
        camera.bus[index] = bus
    camera.left_addr = table["left_addr"]
    camera.right_addr = table["right_addr"]
    camera.sensor_width = 1088
    camera.sensor_height = 1280
    camera.fps = fps
    camera.line_length = 1400
    camera.frame_length = 1500
    camera.tuning_file = None
    camera.output_width = width
    camera.output_height = height
    camera.mode = mode
    camera.stereo_layout = GS130StereoLayout.GS130_STEREO_LAYOUT_NONE
    for index in range(BUS_MAX):
        camera.bus_mipi_rx[index] = 0xFF
        camera.bus_reset_gpio[index] = -1
    for bus, mipi_rx in _CAMERA_MIPI_RX.items():
        camera.bus_mipi_rx[bus] = mipi_rx
    for bus, gpio in _CAMERA_RESET_GPIO.items():
        camera.bus_reset_gpio[bus] = gpio
    camera.fsync_camera = GS130CameraIndex.GS130_CAMERA_RIGHT_IDX

    imu = cfg.imu_config
    imu.bus_num = table["imu_bus_num"]
    for index, bus in enumerate(table["imu_bus"]):
        imu.bus[index] = bus
    imu.addr = 0x68
    imu.odr_hz = odr
    imu.accel_fsr_g = 16
    imu.gyro_fsr_dps = 2000
    imu.accel_bw_sel = 0
    imu.gyro_bw_sel = 0

    eeprom = cfg.eeprom_config
    eeprom.bus_num = len(_CAMERA_BUS)
    for index, bus in enumerate(_CAMERA_BUS):
        eeprom.bus[index] = bus
    eeprom.addr = 0x50

    cfg.camera_fifo.depth = 4
    cfg.camera_fifo.mode = GS130FifoMode.GS130_FIFO_DROP_OLD
    cfg.imu_fifo.depth = table["imu_fifo_depth"]
    cfg.imu_fifo.mode = table["imu_fifo_mode"]
    return cfg


def _find_library():
    env = os.environ.get("GS130_LIB")
    if env:
        return env
    root = Path(__file__).resolve().parents[2]
    for candidate in sorted((root / "core" / "out").glob("*/libgs130.so")):
        return str(candidate)
    found = ctypes.util.find_library("gs130")
    if found:
        return found
    raise OSError("libgs130 shared library not found, set GS130_LIB")


def _load_library():
    global _lib
    if _lib is None:
        for name in _HBN_LIBS:
            try:
                ctypes.CDLL(name, mode=ctypes.RTLD_GLOBAL)
            except OSError:
                pass
        lib = ctypes.CDLL(_find_library())
        for name, (restype, argtypes) in _SIGNATURES.items():
            func = getattr(lib, name)
            func.restype = restype
            func.argtypes = argtypes
        _lib = lib
    return _lib


def library_path():
    return _find_library()


def version():
    return _load_library().gs130_version().decode()


def platform():
    return _load_library().gs130_platform().decode()


def layout_report():
    return {
        name: {
            "sizeof": ctypes.sizeof(cls),
            "offsets": {field: getattr(cls, field).offset for field, _ in cls._fields_},
        }
        for name, cls in _C_TYPES.items()
    }
