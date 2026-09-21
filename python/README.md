# GS130 Camera SDK — Python Interface

[简体中文](README.zh-CN.md) | **English**

> A `ctypes` binding that exposes the GS130 stereo camera, its IMU and its calibration as ordinary numpy arrays.

---

## 📖 Overview

`gs130_camera` is the Python interface to `libgs130`. It mirrors the C ABI with `ctypes` and hands back numpy arrays, so a capture loop is a few lines of plain Python. The binding does no image processing, stereo matching or filtering of its own, and it does not bundle the native library.

| Item | Description |
| --- | --- |
| Python | 3.10 or newer |
| Native library | `libgs130.so` (the `gs130-camera` package) |
| Runtime dependency | `numpy` |
| Typing | ships `py.typed` and `__init__.pyi` |
| Platform | RDK X5, RDK S100 (RDK S600 in development) |

## 🧩 Requirements

- Python 3.10 or newer, and `numpy` (>= 1.20).
- An installation of `libgs130` that matches this binding. The wheel does **not** include the native library; the two must use compatible versions because the binding mirrors the C structures directly. The binding resolves it in this order:

  1. the `GS130_LIB` environment variable,
  2. `ldconfig` (via `ctypes.util.find_library`),
  3. `libgs130.so` on the default search path.

  The version recorded in the package and the one the library reports are compared the first time the library is loaded: an older library refuses to load, a newer one only warns. `library_version()` and `library_platform()` report what was actually loaded.

## 🔨 Install

**1. Install the native library first**

```bash
# Check whether it is already installed
dpkg -s gs130-camera

# If not, install the deb built from core/ (`make deb` in the SDK checkout)
sudo dpkg -i gs130-camera_<version>+<platform>_<arch>.deb
```

**2. Build and install the binding**

```bash
# Build on the board (needs setuptools and wheel installed)
cd python && ./build-wheel.sh

# Install the wheel just built
python3 -m pip install dist/gs130_camera-*.whl
```

The script builds with `pip wheel --no-deps --no-build-isolation`, so it uses the `setuptools` and `wheel` already on the board and fetches nothing from the network. The version comes from the repository-level `VERSION` file, so build from the checkout. `./build-wheel.sh clean` removes `build/`, `dist/` and `*.egg-info`.

## 🚀 Quick start

```python
import gs130_camera

config = gs130_camera.preset(
    "GS130WI", gs130_camera.CameraMode.RECT, 544, 448, 30, 200
)

with gs130_camera.Device(config) as device:
    device.start()

    while device.available_camera() == 0:
        pass
    images = device.read_image()
    print(images["left"].shape, images["left"].timestamp_ns)

    packet = device.read_imu()
    if packet is not None:
        print(packet.accel, packet.gyro, packet.temp)
```

`preset()` fills every field for known hardware; `config()` returns the same dict with everything unset if you would rather set the fields yourself. Either way it is an ordinary nested dict you may edit before handing it to `Device`:

```python
config["camera_config"]["stereo_layout"] = gs130_camera.StereoLayout.TOP_BOTTOM
# read_image() then returns {"stitched": Image} instead of {"left": ..., "right": ...}
```

Reading calibration and converting a frame with OpenCV:

```python
calibration = device.calibration()
print(calibration.camera_left.K, calibration.install_angle)

import cv2
bgr = cv2.cvtColor(images["left"], cv2.COLOR_YUV2BGR_NV12)
```

The model name is not a platform argument: `preset()` reads the platform from the loaded native library, exactly as the C `GS130_CONFIG` macro does. Hardware without a preset raises `ValueError`, where the macro prints to stderr and exits.

## 📡 API

`read_image()` and `read_imu()` return `None` only when there is nothing to hand back, so a capture loop just has to test for that. Everything else raises `GS130Error`.

### Module

| Name | Description |
| --- | --- |
| `preset(device, mode, width, height, fps, odr)` | Filled configuration for known hardware |
| `config()` | Empty configuration, every field unset |
| `library_version()` / `library_platform()` | Version and platform of the loaded `libgs130` |
| `package_version()` / `__version__` | Version recorded when the wheel was built, or `None` |

### Device

| Member | Description |
| --- | --- |
| `Device(config)` | Validate the configuration, initialize the hardware, detect the IMU |
| `start()` / `stop()` | Begin and end capture; `start()` returns `self` |
| `close()` | Stop capture and release every resource; safe to call twice |
| `closed`, `stitched` | Whether the device is released, and whether frames are stitched |
| `imu_name`, `imu_info`, `eeprom_name`, `eeprom_info` | Identity strings, or `None` when absent |
| `available_camera()`, `available_imu()` | Queue depths; `0` while capture is not running |
| `read_image()` | `{"stitched": Image}` or `{"left": Image, "right": Image}`, else `None` |
| `read_imu()` | One `ImuPacket`, else `None` |
| `calibration()` | Full stereo and IMU calibration from the EEPROM |
| `camera_intrinsics(camera)`, `imu_intrinsics()` | One eye's intrinsics, or the IMU's |
| `relative_R(from, to)`, `relative_T(from, to)` | Rotation `(3, 3)` and translation `(3,)` between reference frames |
| `convert_calibration(ref_frame, R, T)` | Move the reference frame, keeping every device pose fixed |

`Relative`/`convert_calibration` take a `ReferenceFrame`; `R` must hold 9 values and `T` three, and a wrong length raises `ValueError` before the C call.

### Values

| Type | Description |
| --- | --- |
| `Image` | An `np.ndarray` subclass shaped `(height * 3 // 2, width)`, `uint8`; carries `timestamp_ns`, `width`, `height` and the zero-copy views `y_plane()` / `uv_plane()` |
| `ImuPacket` | `accel` `(3,)` m/s², `gyro` `(3,)` rad/s, `temp` °C, `is_fsync`, `timestamp_ns` |
| `CameraIntrinsics` | `fx`, `fy`, `cx`, `cy`, `K` `(3, 3)`, `dist_coeffs` `(8,)`, `dist_model` |
| `ImuIntrinsics` | Misalignment, scale, bias, noise and random walk for accelerometer and gyroscope |
| `Calibration` | The two camera intrinsics, the `R` / `T` poses, and `install_angle` |
| `GS130Error` | A failed call: `.code`, `.reason`, `.func`; a `RuntimeError` subclass |
| Enums | `ErrorCode`, `CameraMode`, `CameraIndex`, `StereoLayout`, `FifoMode`, `DistModel`, `ReferenceFrame` |

## ⚙️ Configuration

The configuration is a plain nested dict with five sections, mirroring `gs130_config_t` field by field. `preset()` fills all of them for known hardware, so building one by hand is only needed for unusual setups.

| Section | Fields |
| --- | --- |
| `camera_config` | `bus`, `left_addr`, `right_addr`, `sensor_width`, `sensor_height`, `fps`, `line_length`, `frame_length`, `tuning_file`, `output_width`, `output_height`, `mode`, `stereo_layout`, `bus_mipi_rx`, `bus_reset_gpio`, `fsync_camera` |
| `imu_config` | `bus`, `addr`, `odr_hz`, `accel_fsr_g`, `gyro_fsr_dps`, `accel_bw_sel`, `gyro_bw_sel` |
| `eeprom_config` | `bus`, `addr` |
| `camera_fifo` | `depth`, `mode` |
| `imu_fifo` | `depth`, `mode` |

- `bus_mipi_rx` and `bus_reset_gpio` are `{bus: slot}` dicts, since the C arrays behind them are sparse lookup tables. `bus_num` is not a field; it is `len(bus)`.
- `tuning_file` is a `str` path or `None`; `None` means load no tuning file, which is meaningful to the SDK rather than "unset".
- Every other `None` is rejected when the `Device` is constructed, so nothing silently stays at zero.

## ✨ Features

- **numpy-native frames**: a frame is an `np.ndarray` shaped `(height * 3 // 2, width)`, and `y_plane()` / `uv_plane()` are zero-copy views, so no pixel data is copied on the way out of the SDK.
- **Explicit buffer ownership**: the SDK allocates the frame with `malloc` and hands ownership over; the `Image` frees it once the last reference is gone, so a frame stays valid as long as any view or slice of it is alive.
- **Timestamp that survives slicing**: `timestamp_ns` is carried by `Image` and its views. An operation returning a plain `numpy.ndarray` (`cv2.cvtColor`, for instance) drops it, so read the timestamp first.
- **Loop-friendly reads**: `read_image()` and `read_imu()` return `None` instead of raising when nothing is available; a real failure raises `GS130Error` from the other calls, carrying the SDK's own code, reason and function name.
- **Enums read from the loaded library**: every enum value is taken from `libgs130` at import, so the Python names cannot drift from `gs130.h`.
- **Safe lifetime**: `Device` is a context manager and also warns with a `ResourceWarning` if it is left unclosed; a failure while closing never masks the exception already propagating.
- **Typed**: the package ships `py.typed` and `__init__.pyi`, so editors and type checkers see the full API.

Not implemented: image processing, stereo matching, IMU filtering, recording, and any device discovery beyond what `libgs130` itself reports.

## 🧪 Hardware test

`test/test_gs130.py` is a hardware test rather than a unit test: it walks the public API, measures the queues, and writes the captured frames as PNG.

```bash
cd python
python3 -m pip install '.[test]'                     # the test needs OpenCV
python3 test/test_gs130.py GS130WI rect 544 448 30 200
```

The positional arguments are `device`, `mode` (`raw`, `resize` or `rect`), `width`, `height`, `fps` and `odr`. It runs straight from the checkout whether or not the package was installed, and skips the sections whose hardware this unit does not carry instead of failing the run. Frames go to `gs130_images/` by default; use `--output-dir` to choose another directory, and `--overwrite` to write into a non-empty one.

## 📄 License

Released under the [MIT License](../LICENSE).
