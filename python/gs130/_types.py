"""Data returned by :class:`gs130.Device`."""

import ctypes
import weakref
from dataclasses import dataclass

import numpy as np

from ._enums import DistModel

_free = ctypes.CDLL(None).free
_free.argtypes = [ctypes.c_void_p]
_free.restype = None


class Image(np.ndarray):
    """Zero-copy NV12 image with a nanosecond camera timestamp."""

    def __new__(cls, raw):
        address = ctypes.cast(raw.data, ctypes.c_void_p).value
        if not address:
            raise RuntimeError("libgs130 returned an empty image buffer")
        size = raw.width * raw.height * 3 // 2
        owner = (ctypes.c_uint8 * size).from_address(address)
        weakref.finalize(owner, _free, address)
        array = np.frombuffer(owner, dtype=np.uint8)
        result = array.reshape(raw.height * 3 // 2, raw.width).view(cls)
        result._timestamp_ns = int(raw.timestamp_ns)
        return result

    def __array_finalize__(self, source):
        self._timestamp_ns = getattr(source, "_timestamp_ns", 0)

    @property
    def timestamp_ns(self):
        return self._timestamp_ns


@dataclass(frozen=True)
class ImuPacket:
    accel: np.ndarray
    gyro: np.ndarray
    temp: float
    is_fsync: bool
    timestamp_ns: int


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    K: np.ndarray
    dist_coeffs: np.ndarray
    dist_model: DistModel


@dataclass(frozen=True)
class ImuIntrinsics:
    accel_misalign: np.ndarray
    accel_scale: np.ndarray
    accel_bias: np.ndarray
    accel_noise: float
    accel_random_walk: float
    gyro_misalign: np.ndarray
    gyro_scale: np.ndarray
    gyro_bias: np.ndarray
    gyro_noise: float
    gyro_random_walk: float


@dataclass(frozen=True)
class Calibration:
    imu: ImuIntrinsics
    camera_left: CameraIntrinsics
    camera_right: CameraIntrinsics
    camera_left_R: np.ndarray
    camera_left_T: np.ndarray
    camera_right_R: np.ndarray
    camera_right_T: np.ndarray
    imu_R: np.ndarray
    imu_T: np.ndarray
    install_angle: int


def _array(values, shape):
    return np.ctypeslib.as_array(values).copy().reshape(shape)


def camera_intrinsics(raw):
    return CameraIntrinsics(
        raw.fx,
        raw.fy,
        raw.cx,
        raw.cy,
        _array(raw.K, (3, 3)),
        _array(raw.dist_coeffs, (8,)),
        DistModel(raw.dist_model),
    )


def imu_intrinsics(raw):
    return ImuIntrinsics(
        _array(raw.accel_misalign, (3, 3)),
        _array(raw.accel_scale, (3,)),
        _array(raw.accel_bias, (3,)),
        raw.accel_noise,
        raw.accel_random_walk,
        _array(raw.gyro_misalign, (3, 3)),
        _array(raw.gyro_scale, (3,)),
        _array(raw.gyro_bias, (3,)),
        raw.gyro_noise,
        raw.gyro_random_walk,
    )


def calibration(raw):
    return Calibration(
        imu_intrinsics(raw.imu),
        camera_intrinsics(raw.camera_left),
        camera_intrinsics(raw.camera_right),
        _array(raw.camera_left_R, (3, 3)),
        _array(raw.camera_left_T, (3,)),
        _array(raw.camera_right_R, (3, 3)),
        _array(raw.camera_right_T, (3,)),
        _array(raw.imu_R, (3, 3)),
        _array(raw.imu_T, (3,)),
        raw.camera_install_angle,
    )
