"""Python interface for the gs130 stereo camera and IMU."""

from ._api._calibration import Calibration, CameraIntrinsics, ImuIntrinsics
from ._api._config import (
    CameraConfig,
    Config,
    EepromConfig,
    FifoConfig,
    ImuConfig,
)
from ._api._device import Device, ImuPacket
from ._api._enums import (
    CameraIndex,
    CameraMode,
    DistModel,
    ErrorCode,
    FifoMode,
    ReferenceFrame,
    StereoLayout,
)
from ._api._error import GS130Error
from ._internal._version import __version__

__all__ = [
    "__version__",
    "GS130Error",
    "ErrorCode",
    "FifoMode",
    "CameraMode",
    "CameraIndex",
    "StereoLayout",
    "DistModel",
    "ReferenceFrame",
    "Config",
    "CameraConfig",
    "ImuConfig",
    "EepromConfig",
    "FifoConfig",
    "Device",
    "ImuPacket",
    "Calibration",
    "CameraIntrinsics",
    "ImuIntrinsics",
]
