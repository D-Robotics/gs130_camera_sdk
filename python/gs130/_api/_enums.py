"""The values the gs130 calls take and return, with Python names.

Every member carries the value of the C enumerator it stands for, so a value
from here can be passed straight to the library and a value it returns can be
compared with one of these.
"""

from enum import IntEnum

from .._internal._abi import (
    gs130_camera_index_t,
    gs130_camera_mode_t,
    gs130_dist_model_t,
    gs130_err_t,
    gs130_fifo_mode_t,
    gs130_reference_frame_t,
    gs130_stereo_layout_t,
)


class ErrorCode(IntEnum):
    """What a gs130 call returns. GS130Error carries one of these."""

    OK = gs130_err_t.GS130_OK
    PARAM_ERROR = gs130_err_t.GS130_PARAM_ERROR
    UNSUPPORTED = gs130_err_t.GS130_UNSUPPORTED
    NOT_FOUND = gs130_err_t.GS130_NOT_FOUND
    HW_ERROR = gs130_err_t.GS130_HW_ERROR
    TIMEOUT = gs130_err_t.GS130_TIMEOUT
    THREAD_CLOSED = gs130_err_t.GS130_THREAD_CLOSED


class FifoMode(IntEnum):
    """What a full queue does with the new item."""

    DROP_NEW = gs130_fifo_mode_t.GS130_FIFO_DROP_NEW
    DROP_OLD = gs130_fifo_mode_t.GS130_FIFO_DROP_OLD


class CameraMode(IntEnum):
    """How the camera pipeline is set up."""

    RAW = gs130_camera_mode_t.GS130_CAMERA_MODE_RAW
    RESIZE = gs130_camera_mode_t.GS130_CAMERA_MODE_RESIZE
    RECT = gs130_camera_mode_t.GS130_CAMERA_MODE_RECT


class CameraIndex(IntEnum):
    """Which of the two cameras."""

    RIGHT = gs130_camera_index_t.GS130_CAMERA_RIGHT_IDX
    LEFT = gs130_camera_index_t.GS130_CAMERA_LEFT_IDX


class StereoLayout(IntEnum):
    """How the two eyes are stitched, NONE for two separate frames."""

    NONE = gs130_stereo_layout_t.GS130_STEREO_LAYOUT_NONE
    LEFT_RIGHT = gs130_stereo_layout_t.GS130_STEREO_LAYOUT_LEFT_RIGHT
    RIGHT_LEFT = gs130_stereo_layout_t.GS130_STEREO_LAYOUT_RIGHT_LEFT
    TOP_BOTTOM = gs130_stereo_layout_t.GS130_STEREO_LAYOUT_TOP_BOTTOM
    BOTTOM_TOP = gs130_stereo_layout_t.GS130_STEREO_LAYOUT_BOTTOM_TOP


class DistModel(IntEnum):
    """Which distortion model the coefficients belong to."""

    PINHOLE = gs130_dist_model_t.GS130_DIST_PINHOLE
    FISHEYE = gs130_dist_model_t.GS130_DIST_FISHEYE


class ReferenceFrame(IntEnum):
    """A frame the calibration can be expressed in."""

    CAMERA_RIGHT = gs130_reference_frame_t.GS130_REF_CAMERA_RIGHT
    CAMERA_LEFT = gs130_reference_frame_t.GS130_REF_CAMERA_LEFT
    IMU = gs130_reference_frame_t.GS130_REF_IMU
