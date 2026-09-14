"""Finding and loading libgs130: internal to the gs130 package.

Nothing here mirrors a header. Which library to use, and what to say when
there is none, is deployment policy.
"""

import ctypes
import ctypes.util
import os

from . import _abi

# the camera stack has to be in the global namespace before libgs130 runs
_HBN_LIBS = ("libvpf.so", "libhbmem.so", "libcam.so")

_lib = None
_path = None


def _find():
    """$GS130_LIB first, then the system library, else say what to do."""
    configured = os.environ.get("GS130_LIB")
    if configured:
        return configured
    found = ctypes.util.find_library("gs130")
    if found:
        return found
    raise OSError(
        "libgs130 not found: install the package that provides it "
        "(libgs130.so.0) or point GS130_LIB at the library"
    )


def load():
    """The loaded CDLL, with the signatures of gs130.h applied to it.

    :raises OSError: no library was found, or it could not be loaded.
    """
    global _lib, _path
    if _lib is None:
        for name in _HBN_LIBS:
            try:
                ctypes.CDLL(name, mode=ctypes.RTLD_GLOBAL)
            except OSError:
                pass
        _path = _find()
        library = ctypes.CDLL(_path)
        for name, (restype, argtypes) in _abi._SIGNATURES.items():
            function = getattr(library, name)
            function.restype = restype
            function.argtypes = argtypes
        _lib = library
    return _lib


def path():
    """The path or name the library was loaded from."""
    load()
    return _path


def version():
    """The version string of the loaded library."""
    return load().gs130_version().decode()


def platform():
    """The platform string of the loaded library, for example "rdkx5"."""
    return load().gs130_platform().decode()
