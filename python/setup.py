"""Build metadata for the gs130_camera Python package.

The package and libgs130 share the repository-level VERSION file because their
ctypes structures must stay in lockstep.  This stays a setup.py rather than a
PEP 621 ``[project]`` table: the supported RDK image ships setuptools 59, which
predates complete PEP 621 support.
"""

from pathlib import Path

from setuptools import setup


HERE = Path(__file__).resolve().parent
VERSION_FILE = HERE.parent / "VERSION"
README_FILE = HERE / "README.md"

if not VERSION_FILE.is_file():
    raise RuntimeError(
        "%s is missing; build this package from the repository checkout, "
        "which owns the version" % VERSION_FILE
    )

setup(
    name="gs130-camera",
    version=VERSION_FILE.read_text(encoding="utf-8").strip(),
    description="Python interface for the GS130 stereo camera and IMU",
    long_description=README_FILE.read_text(encoding="utf-8"),
    long_description_content_type="text/markdown",
    author="D-Robotics",
    license="MIT",
    url="https://github.com/D-Robotics/gs130_camera_sdk",
    project_urls={
        "Source": "https://github.com/D-Robotics/gs130_camera_sdk",
        "Issues": "https://github.com/D-Robotics/gs130_camera_sdk/issues",
    },
    packages=["gs130_camera"],
    package_data={"gs130_camera": ["__init__.pyi", "py.typed"]},
    python_requires=">=3.10",
    install_requires=["numpy>=1.20"],
    # The hardware test prints frames and writes PNGs, so it needs OpenCV.
    extras_require={"test": ["opencv-python>=4.5"]},
    classifiers=[
        "Development Status :: 4 - Beta",
        "License :: OSI Approved :: MIT License",
        "Operating System :: POSIX :: Linux",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.10",
        "Topic :: Scientific/Engineering",
    ],
)
