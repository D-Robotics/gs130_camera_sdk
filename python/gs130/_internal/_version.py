"""The version of this installed package, from its distribution metadata.

Running from a source checkout there is no distribution, so this is None and
the version check in _lib has nothing to compare.
"""

import importlib.metadata


def _read():
    try:
        return importlib.metadata.version("gs130")
    except importlib.metadata.PackageNotFoundError:
        return None


__version__ = _read()
