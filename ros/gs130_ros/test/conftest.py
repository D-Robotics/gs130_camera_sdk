"""Test setup shared by the package's tests.

The node needs libgs130 and its Python binding.  A machine without them can
still check every pure conversion, which is where most of this package's logic
lives, so the real binding is used when it imports and a faithful stand-in is
installed when it does not.  The stand-in's values come from
``core/include/gs130.h`` and are written out in full there, so a test that
depends on a value failing here means the header moved.
"""

import enum
import sys
import types


def _stand_in():
    module = types.ModuleType("gs130")

    class CameraMode(enum.IntEnum):
        # gs130_camera_mode_t, in declaration order
        RAW = 0
        RESIZE = 1
        RECT = 2

    class StereoLayout(enum.IntEnum):
        # gs130_stereo_layout_t, in declaration order
        NONE = 0
        LEFT_RIGHT = 1
        RIGHT_LEFT = 2
        TOP_BOTTOM = 3
        BOTTOM_TOP = 4

    class DistModel(enum.IntEnum):
        # gs130_dist_model_t, in declaration order
        PINHOLE = 0
        FISHEYE = 1

    module.CameraMode = CameraMode
    module.StereoLayout = StereoLayout
    module.DistModel = DistModel
    return module


try:
    import gs130  # noqa: F401
except Exception:  # noqa: BLE001 - any failure means "use the stand-in"
    sys.modules["gs130"] = _stand_in()
