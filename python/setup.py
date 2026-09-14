"""Build configuration for the gs130 Python package."""

from pathlib import Path

from setuptools import find_packages, setup


ROOT = Path(__file__).resolve().parent.parent

setup(
    name="gs130",
    version=(ROOT / "VERSION").read_text().strip(),
    description="Python interface for the GS130 stereo camera and IMU",
    license="MIT",
    url="https://github.com/hachi-leaf/gs130_sdk",
    packages=find_packages(),
    python_requires=">=3.10",
    install_requires=["numpy>=1.20"],
)
