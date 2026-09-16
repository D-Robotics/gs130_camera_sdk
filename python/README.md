# GS130 Python binding

`gs130` is the Python interface to `libgs130`. It exposes the stereo camera,
IMU, calibration data, and the configuration presets defined by the C SDK.

## Requirements

- Python 3.10 or newer
- `numpy`
- a compatible system installation of `libgs130`

The wheel does not bundle the native library. At runtime the binding resolves
`GS130_LIB` first, then `ldconfig`, then `libgs130.so`. The Python package and
native library must use compatible versions because the binding mirrors the C
structures directly.

## Install

From the repository checkout:

```bash
python3 -m pip install ./python
```

To build a wheel with the toolchain already installed on the board:

```bash
cd python
./build-wheel.sh
```

The wheel is written to `python/dist/`.

## Use

```python
import gs130

configuration = gs130.preset(
    "GS130WI",
    gs130.CameraMode.RECT,
    544,
    448,
    30,
    200,
)

with gs130.Device(configuration) as device:
    device.start()
    while device.available_camera() == 0:
        pass
    images = device.read_image()
    left = images["left"]
    right = images["right"]
```

`read_image()` and `read_imu()` return `None` only when the SDK reports that no
sample is available. Other SDK errors raise `GS130Error`.

The model name is not a platform argument: `preset()` reads the platform from
the loaded native library, exactly as the C `GS130_CONFIG` macro does.

## Hardware test

`test/test_gs130.py` is the hardware test. It walks the public API, measures the
queues, and writes the captured frames as PNG:

```bash
cd python
python3 test/test_gs130.py GS130WI rect 544 448 30 200
```

It needs the camera and OpenCV (`python3 -m pip install '.[test]'`), runs
straight from the checkout whether or not the package was installed, and skips
the sections whose hardware this unit does not carry. Use `--output-dir` to pick
the PNG directory; a non-empty directory is only reused with `--overwrite`.
