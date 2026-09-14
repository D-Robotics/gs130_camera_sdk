"""Public enum values used by the gs130 API."""

from enum import IntEnum


class ErrorCode(IntEnum):
    OK = 0
    PARAM_ERROR = 1
    UNSUPPORTED = 2
    NOT_FOUND = 3
    HW_ERROR = 4
    TIMEOUT = 5
    THREAD_CLOSED = 6


class FifoMode(IntEnum):
    DROP_NEW = 0
    DROP_OLD = 1


class CameraMode(IntEnum):
    RAW = 0
    RESIZE = 1
    RECT = 2


class CameraIndex(IntEnum):
    RIGHT = 0
    LEFT = 1


class StereoLayout(IntEnum):
    NONE = 0
    LEFT_RIGHT = 1
    RIGHT_LEFT = 2
    TOP_BOTTOM = 3
    BOTTOM_TOP = 4


class DistModel(IntEnum):
    PINHOLE = 0
    FISHEYE = 1


class ReferenceFrame(IntEnum):
    CAMERA_RIGHT = 0
    CAMERA_LEFT = 1
    IMU = 2
