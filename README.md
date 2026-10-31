# Project moved

The GS130 Camera SDK is superseded by the **RDK Stereo Camera SDK**:

**https://github.com/D-Robotics/RDK-Stereo-Camera-SDK**

[简体中文](README.zh-CN.md)

## Why a new project

The product line grew from GS130WI / GS130W to GS130WI V2, RDK Stereo A1 and A1P. The old name no
longer described the family: "GS130" encoded the shutter type, the pixel size, the wide angle and
the IMU, which stops holding once the line expands.

## What changed

The new project drops the encoded naming:

- SDK name: **RDK Stereo Camera SDK**
- Native library: `libdstereocam`
- Command-line tool: `dsc`
- Package: `dstereo-cam`
- Version line: restarted at 0.1.0

## What happened to this repository

It is frozen at its last release, **0.1.2-Alpha**, and kept for history only. The `develop` branch
has been deleted and no further changes are accepted here.
