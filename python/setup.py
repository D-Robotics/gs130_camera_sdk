"""Build configuration for the gs130 package.

The version lives in the repository root because the wheel and libgs130 must
agree on it, so this package is built from the checkout rather than from a
copy of ``python/`` on its own.
"""

from pathlib import Path

from setuptools import find_packages, setup


VERSION_FILE = Path(__file__).resolve().parent.parent / "VERSION"

if not VERSION_FILE.is_file():
    raise RuntimeError(
        "%s is missing; build this package from the repository checkout, "
        "which owns the version" % VERSION_FILE
    )

setup(
    name="gs130",
    version=VERSION_FILE.read_text().strip(),
    description="Python interface for the GS130 stereo camera and IMU",
    license="MIT",
    url="https://github.com/hachi-leaf/gs130_sdk",
    packages=find_packages(),
    python_requires=">=3.10",
    install_requires=["numpy>=1.20"],
    extras_require={"test": ["opencv-python"]},
)
