"""Build configuration for the gs130_ros package.

The version comes from the repository root, because this node and libgs130
have to agree on it: the Python binding refuses a library that is older than
the package, and an out-of-step ROS package would be the same problem one layer
up.  That also means the package is built from the checkout rather than from a
copy of ``ros/gs130_ros`` on its own.
"""

import os
from glob import glob
from pathlib import Path

from setuptools import setup


PACKAGE_NAME = "gs130_ros"

VERSION_FILE = Path(__file__).resolve().parent.parent.parent / "VERSION"

if not VERSION_FILE.is_file():
    raise RuntimeError(
        "%s is missing; build this package from the repository checkout, which "
        "owns the version" % VERSION_FILE
    )

setup(
    name=PACKAGE_NAME,
    version=VERSION_FILE.read_text().strip(),
    description="ROS 2 interface for the GS130 stereo camera and IMU",
    license="MIT",
    url="https://github.com/hachi-leaf/gs130_sdk",
    packages=[PACKAGE_NAME],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        (os.path.join("share", PACKAGE_NAME), ["package.xml"]),
        (os.path.join("share", PACKAGE_NAME, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    python_requires=">=3.10",
    entry_points={"console_scripts": ["gs130_node = gs130_ros.node:main"]},
)
