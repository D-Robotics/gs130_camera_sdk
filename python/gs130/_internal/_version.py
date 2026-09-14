"""The version of this Python package.

It comes from the same VERSION file the C library is built from, so the two can
be compared: an installed package reports its metadata version, a source
checkout reads the file next to core/.
"""

import importlib.metadata
from pathlib import Path

_VERSION_FILE = Path(__file__).resolve().parents[3] / "VERSION"


def _read():
    try:
        return importlib.metadata.version("gs130")
    except importlib.metadata.PackageNotFoundError:
        pass
    if _VERSION_FILE.is_file():
        return _VERSION_FILE.read_text().strip()
    return None


__version__ = _read()
