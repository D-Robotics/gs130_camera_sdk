"""Load libgs130 and apply the declarations in :mod:`gs130._abi`."""

import ctypes
import ctypes.util
import importlib.metadata
import os
import warnings

from ._abi import SIGNATURES

_PRELOAD = ("libvpf.so", "libhbmem.so", "libcam.so")
_library = None
_missing = ()


def _package_version():
    try:
        return importlib.metadata.version("gs130")
    except importlib.metadata.PackageNotFoundError:
        return None


__version__ = _package_version()


def _find_library():
    configured = os.environ.get("GS130_LIB")
    if configured:
        return configured
    found = ctypes.util.find_library("gs130")
    if found:
        return found
    raise OSError(
        "libgs130 not found; install libgs130.so.0 or set GS130_LIB"
    )


def _version_tuple(text):
    numbers = []
    for part in text.split("+", 1)[0].split("-", 1)[0].split("."):
        if not part.isdigit():
            break
        numbers.append(int(part))
    return tuple(numbers)


def _check_version(library_version):
    if __version__ is None:
        return
    package = _version_tuple(__version__)
    library = _version_tuple(library_version)
    if not package or not library or package == library:
        return
    if library > package:
        raise RuntimeError(
            "libgs130 %s is newer than gs130 package %s" %
            (library_version, __version__)
        )
    warnings.warn(
        "libgs130 %s is older than gs130 package %s" %
        (library_version, __version__),
        RuntimeWarning,
        stacklevel=3,
    )


def load():
    """Return the process-wide, fully declared libgs130 handle."""
    global _library, _missing
    if _library is not None:
        return _library

    for name in _PRELOAD:
        try:
            ctypes.CDLL(name, mode=ctypes.RTLD_GLOBAL)
        except OSError:
            pass

    path = _find_library()
    library = ctypes.CDLL(path)
    missing = []
    for name, (restype, argtypes) in SIGNATURES.items():
        try:
            function = getattr(library, name)
        except AttributeError:
            missing.append(name)
            continue
        function.restype = restype
        function.argtypes = argtypes

    if missing:
        warnings.warn(
            "libgs130 %s is missing: %s" % (path, ", ".join(missing)),
            RuntimeWarning,
            stacklevel=2,
        )
    if "gs130_version" not in missing:
        value = library.gs130_version()
        if value:
            _check_version(value.decode())

    _missing = tuple(missing)
    _library = library
    return library


def function(library, name):
    """Return a declared function or raise a stable error for an old library."""
    if name in _missing:
        raise RuntimeError("loaded libgs130 does not provide %s()" % name)
    return getattr(library, name)


def library_version():
    """Return the loaded libgs130 version string."""
    value = function(load(), "gs130_version")()
    return value.decode() if value else None

