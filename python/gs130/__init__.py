"""Python interface for the GS130 stereo camera and IMU.

Typical use::

    import gs130

    config = gs130.preset(
        "RDKX5", "GS130WI", gs130.CameraMode.RESIZE, 1088, 1280, 30, 200
    )
    with gs130.Device(config) as dev:
        dev.start()
        frame = dev.read_image()

``read_image`` and ``read_imu`` return ``None`` when there is nothing to hand
back, so a capture loop only has to test for that.  Everything else raises
:class:`GS130Error`, carrying the code the SDK reported.
"""

from ._config import config
from ._device import Device, GS130Error
from ._preset import preset
from ._types import (
    Calibration,
    CameraIndex,
    CameraIntrinsics,
    CameraMode,
    DistModel,
    ErrorCode,
    FifoMode,
    Image,
    ImuIntrinsics,
    ImuPacket,
    ReferenceFrame,
    StereoLayout,
)

__all__ = [
    "Device",
    "GS130Error",
    "Image",
    "ImuPacket",
    "CameraIntrinsics",
    "ImuIntrinsics",
    "Calibration",
    "config",
    "preset",
    "ErrorCode",
    "CameraMode",
    "CameraIndex",
    "StereoLayout",
    "FifoMode",
    "DistModel",
    "ReferenceFrame",
]
