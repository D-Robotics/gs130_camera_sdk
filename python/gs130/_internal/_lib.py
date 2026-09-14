"""Finding and loading libgs130: internal to the gs130 package.

Nothing here mirrors a header. Which library to use, what to say when there is
none, and which versions may be mixed are deployment decisions.
"""

import ctypes
import ctypes.util
import os
import warnings

from . import _abi, _version

# the camera stack has to be in the global namespace before libgs130 runs
_HBN_LIBS = ("libvpf.so", "libhbmem.so", "libcam.so")

_lib = None
_path = None
_missing = ()


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


def _numbers(text):
    """The leading dotted numbers of a version, (0, 0, 1) for "0.0.1+rdkx5"."""
    parts = []
    for piece in text.split("+")[0].split("-")[0].split("."):
        if not piece.isdigit():
            break
        parts.append(int(piece))
    return tuple(parts)


def _check_version(library_version):
    """Refuse a library newer than this package; warn about an older one.

    A newer library may have grown a structure this package allocates too
    small, which the library would read past the end of. An older one only
    lacks functions, which is reported when one of them is missing.

    :raises RuntimeError: the library is newer than the package.
    """
    package = _version.__version__
    if package is None:
        return
    ours = _numbers(package)
    theirs = _numbers(library_version)
    if not ours or not theirs or ours == theirs:
        return
    if theirs > ours:
        raise RuntimeError(
            "libgs130 %s is newer than this gs130 package %s: its structures "
            "may be larger than the ones this package builds. Install the "
            "matching gs130 package." % (library_version, package)
        )
    warnings.warn(
        "libgs130 %s is older than this gs130 package %s: only what %s has is "
        "available" % (library_version, package, library_version),
        RuntimeWarning,
        stacklevel=3,
    )


def load():
    """The loaded CDLL, with the signatures of gs130.h applied to it.

    :raises OSError: no library was found, or it could not be loaded.
    :raises RuntimeError: the library is newer than this package.
    """
    global _lib, _path, _missing
    if _lib is None:
        for name in _HBN_LIBS:
            try:
                ctypes.CDLL(name, mode=ctypes.RTLD_GLOBAL)
            except OSError:
                pass
        _path = _find()
        library = ctypes.CDLL(_path)
        _missing = tuple(
            name for name in _abi._SIGNATURES if not hasattr(library, name)
        )
        # the signatures come first: without them ctypes would read
        # gs130_version() as the int it returns by default
        for name, (restype, argtypes) in _abi._SIGNATURES.items():
            if name in _missing:
                continue
            function = getattr(library, name)
            function.restype = restype
            function.argtypes = argtypes
        if _missing:
            warnings.warn(
                "libgs130 %s does not provide %s: calling them fails"
                % (_path, ", ".join(_missing)),
                RuntimeWarning,
                stacklevel=2,
            )
        _check_version(library.gs130_version().decode())
        _lib = library
    return _lib


def missing():
    """The functions of gs130.h the loaded library does not provide."""
    load()
    return _missing


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
