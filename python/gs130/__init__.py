"""Python interface for the gs130 stereo camera and IMU."""

from ._internal._version import __version__
from ._ffi import (
    gs130_camera_index_t,
    gs130_camera_mode_t,
    gs130_dist_model_t,
    gs130_err_t,
    gs130_fifo_mode_t,
    gs130_reference_frame_t,
    gs130_stereo_layout_t,
    path as library_path,
    platform,
    version as library_version,
)
from ._api._config import (
    CameraConfig,
    Config,
    EepromConfig,
    FifoConfig,
    ImuConfig,
)


class GS130Error(RuntimeError):
    """Raised when a gs130 call fails.

    code is the gs130_err_t value the library returned and func is the C
    function that returned it, or None.
    """

    def __init__(self, code, func=None):
        self.code = int(code)
        self.func = func
        try:
            reason = gs130_err_t(code).name
        except ValueError:
            reason = "error code %s" % (code,)
        super().__init__(
            "%s() -> %s" % (func, reason) if func else reason
        )


_ENUMS = (
    gs130_err_t,
    gs130_fifo_mode_t,
    gs130_camera_mode_t,
    gs130_camera_index_t,
    gs130_stereo_layout_t,
    gs130_dist_model_t,
    gs130_reference_frame_t,
)

globals().update({member.name: member for enum in _ENUMS for member in enum})

__all__ = [member.name for enum in _ENUMS for member in enum] + [
    "__version__",
    "GS130Error",
    "Config",
    "CameraConfig",
    "ImuConfig",
    "EepromConfig",
    "FifoConfig",
    "library_path",
    "library_version",
    "platform",
]
