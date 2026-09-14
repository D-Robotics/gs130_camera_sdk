import pathlib

from setuptools import setup

HERE = pathlib.Path(__file__).resolve().parent

setup(
    name="gs130",
    version=(HERE.parent / "VERSION").read_text(encoding="utf-8").strip(),
    packages=["gs130"],
    python_requires=">=3.8",
    install_requires=["numpy>=1.20"],
)
