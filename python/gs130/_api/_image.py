"""The NV12 frames the library hands out, as numpy arrays.

The library allocates the buffer with malloc() and gives up ownership, so the
array frees it when the last view of it dies. The finalizer hangs off the
ctypes array because that object is the root of every view's base chain.
"""

import ctypes
import weakref

import numpy as np

_free = ctypes.CDLL(None).free
_free.argtypes = [ctypes.c_void_p]
_free.restype = None


class Image(np.ndarray):
    """One NV12 frame: (height * 3 // 2, width) uint8, plus timestamp_ns.

    The first height rows are the Y plane, the rest interleaved UV. Slices and
    plain ndarray views share the buffer and keep it alive; a copy does not.
    """

    def __new__(cls, address, width, height, timestamp_ns=0):
        size = width * height * 3 // 2
        buffer = (ctypes.c_uint8 * size).from_address(address)
        weakref.finalize(buffer, _free, address)
        image = np.frombuffer(buffer, dtype=np.uint8)
        obj = image.reshape(size // width, width).view(cls)
        obj._timestamp_ns = timestamp_ns
        return obj

    def __array_finalize__(self, obj):
        self._timestamp_ns = getattr(obj, "_timestamp_ns", 0)

    @property
    def timestamp_ns(self):
        return self._timestamp_ns


def image(raw):
    """Wrap a gs130_image_nv12_t as a zero-copy array."""
    address = ctypes.cast(raw.data, ctypes.c_void_p).value
    return Image(address, raw.width, raw.height, raw.timestamp_ns)
