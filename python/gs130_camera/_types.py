"""The values gs130 hands back to the user.

This module never calls the SDK.  It holds the enums a user compares against
and the data classes a user receives.
"""

from __future__ import annotations

import ctypes
import weakref
from dataclasses import dataclass
from enum import IntEnum

import numpy as np

from . import _abi


_free = ctypes.CDLL(None).free
_free.argtypes = [ctypes.c_void_p]
_free.restype = None


# ---------------------------------------------------------------------------
# Enums.  The names are the user-facing ones; every value is read out of _abi,
# so they cannot drift from gs130.h.
# ---------------------------------------------------------------------------


class ErrorCode(IntEnum):
    """Why a call failed; ``OK`` means it did not."""

    OK = _abi.gs130_err_t.GS130_OK
    PARAM_ERROR = _abi.gs130_err_t.GS130_PARAM_ERROR
    UNSUPPORTED = _abi.gs130_err_t.GS130_UNSUPPORTED
    NOT_FOUND = _abi.gs130_err_t.GS130_NOT_FOUND
    HW_ERROR = _abi.gs130_err_t.GS130_HW_ERROR
    TIMEOUT = _abi.gs130_err_t.GS130_TIMEOUT
    THREAD_CLOSED = _abi.gs130_err_t.GS130_THREAD_CLOSED


class CameraMode(IntEnum):
    """How frames travel through the ISP."""

    RAW = _abi.gs130_camera_mode_t.GS130_CAMERA_MODE_RAW
    RESIZE = _abi.gs130_camera_mode_t.GS130_CAMERA_MODE_RESIZE
    RECT = _abi.gs130_camera_mode_t.GS130_CAMERA_MODE_RECT


class CameraIndex(IntEnum):
    RIGHT = _abi.gs130_camera_index_t.GS130_CAMERA_RIGHT_IDX
    LEFT = _abi.gs130_camera_index_t.GS130_CAMERA_LEFT_IDX


class StereoLayout(IntEnum):
    """How the two eyes are packed into one frame."""

    NONE = _abi.gs130_stereo_layout_t.GS130_STEREO_LAYOUT_NONE
    LEFT_RIGHT = _abi.gs130_stereo_layout_t.GS130_STEREO_LAYOUT_LEFT_RIGHT
    RIGHT_LEFT = _abi.gs130_stereo_layout_t.GS130_STEREO_LAYOUT_RIGHT_LEFT
    TOP_BOTTOM = _abi.gs130_stereo_layout_t.GS130_STEREO_LAYOUT_TOP_BOTTOM
    BOTTOM_TOP = _abi.gs130_stereo_layout_t.GS130_STEREO_LAYOUT_BOTTOM_TOP


class FifoMode(IntEnum):
    """Which sample a full queue throws away."""

    DROP_NEW = _abi.gs130_fifo_mode_t.GS130_FIFO_DROP_NEW
    DROP_OLD = _abi.gs130_fifo_mode_t.GS130_FIFO_DROP_OLD


class DistModel(IntEnum):
    PINHOLE = _abi.gs130_dist_model_t.GS130_DIST_PINHOLE
    FISHEYE = _abi.gs130_dist_model_t.GS130_DIST_FISHEYE


class ReferenceFrame(IntEnum):
    CAMERA_RIGHT = _abi.gs130_reference_frame_t.GS130_REF_CAMERA_RIGHT
    CAMERA_LEFT = _abi.gs130_reference_frame_t.GS130_REF_CAMERA_LEFT
    IMU = _abi.gs130_reference_frame_t.GS130_REF_IMU


# ---------------------------------------------------------------------------
# Camera frames
# ---------------------------------------------------------------------------


class Image(np.ndarray):
    """One NV12 frame, shaped ``(height * 3 // 2, width)``.

    The buffer is allocated by the SDK with ``malloc`` and its ownership passes
    to the caller.  This object frees it once the last reference is gone, so a
    frame stays valid for as long as any view or slice of it is alive.

    Slices and the plane helpers below keep ``timestamp_ns``, but any operation
    that returns a plain ``numpy.ndarray`` (``cv2.cvtColor``, for instance)
    drops it; read the timestamp before converting.
    """

    def __new__(cls, raw):
        address = ctypes.cast(raw.data, ctypes.c_void_p).value
        if not address:
            raise ValueError("libgs130 returned an empty image buffer")
        size = raw.width * raw.height * 3 // 2
        buffer = (ctypes.c_uint8 * size).from_address(address)
        weakref.finalize(buffer, _free, address)
        frame = np.frombuffer(buffer, dtype=np.uint8)
        result = frame.reshape(raw.height * 3 // 2, raw.width).view(cls)
        result._timestamp_ns = int(raw.timestamp_ns)
        return result

    def __array_finalize__(self, source):
        self._timestamp_ns = getattr(source, "_timestamp_ns", 0)

    @property
    def timestamp_ns(self):
        """Camera timestamp in nanoseconds."""
        return self._timestamp_ns

    @property
    def width(self):
        return self.shape[1]

    @property
    def height(self):
        return self.shape[0] * 2 // 3

    def y_plane(self):
        """Luma plane, shape ``(height, width)``; a zero-copy view."""
        return self[: self.height]

    def uv_plane(self):
        """Interleaved chroma plane, shape ``(height // 2, width)``; a view."""
        return self[self.height :]


# ---------------------------------------------------------------------------
# IMU
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ImuPacket:
    """One IMU sample."""

    accel: np.ndarray  # (3,) m/s^2
    gyro: np.ndarray  # (3,) rad/s
    temp: float  # degC
    is_fsync: bool  # this packet carries the camera sync pulse
    timestamp_ns: int  # corrected, aligned to the camera clock


# ---------------------------------------------------------------------------
# Calibration
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    K: np.ndarray  # (3, 3)
    dist_coeffs: np.ndarray  # (8,)
    dist_model: DistModel


@dataclass(frozen=True)
class ImuIntrinsics:
    accel_misalign: np.ndarray  # (3, 3)
    accel_scale: np.ndarray  # (3,)
    accel_bias: np.ndarray  # (3,)
    accel_noise: float  # m/s^2/sqrt(Hz)
    accel_random_walk: float  # m/s^3/sqrt(Hz)
    gyro_misalign: np.ndarray  # (3, 3)
    gyro_scale: np.ndarray  # (3,)
    gyro_bias: np.ndarray  # (3,)
    gyro_noise: float  # rad/s/sqrt(Hz)
    gyro_random_walk: float  # rad/s^2/sqrt(Hz)


@dataclass(frozen=True)
class Calibration:
    """Everything the EEPROM knows about this unit.

    The ``R`` / ``T`` pairs are absolute poses in the reference frame: R maps a
    point from that device's frame into the reference frame, T is the matching
    translation in the same frame.  A device's reference frame is defined by
    the EEPROM driver at init.  Enabling stereo rectification (``RECT`` mode)
    keeps the reference frame but makes the extrinsics virtual (parallel
    stereo), so the pose changes meaning under ``RECT``.
    """

    imu: ImuIntrinsics
    camera_left: CameraIntrinsics
    camera_right: CameraIntrinsics
    camera_left_R: np.ndarray  # (3, 3)
    camera_left_T: np.ndarray  # (3,)
    camera_right_R: np.ndarray  # (3, 3)
    camera_right_T: np.ndarray  # (3,)
    imu_R: np.ndarray  # (3, 3)
    imu_T: np.ndarray  # (3,)
    install_angle: int  # camera mounting angle, degrees
