# GS130 Camera SDK

[简体中文](README.zh-CN.md) | **English**

The GS130 Camera SDK provides the software components required to operate the GS130 stereo camera and its onboard IMU from an RDK development board. It consists of a native C library and two wrappers built on top of it, one for Python and one for ROS 2.

> **Developer preview (Alpha).** This release is intended for evaluation. Interfaces, configuration presets and behaviour may change without notice. Feedback and contributions are welcome through the [D-Robotics developer community](https://forum.d-robotics.cc/) or by email to [xiaoye.zhang@d-robotics.cc](mailto:xiaoye.zhang@d-robotics.cc).

---

## 📖 Overview

The hardware pipeline is implemented once, in the native library: sensor capture and ISP processing, rectification on the GDC hardware, calibration data read from the onboard EEPROM, and IMU sampling aligned to the camera time base through FSYNC. The three layers listed below expose that pipeline to applications and are delivered separately.

| Layer | Directory | Contents |
| --- | --- | --- |
| Core | `core/` | `libgs130`, the public C headers, the gs130 tools, and Debian packaging |
| Python wrapper | `python/` | The `gs130_camera` package, which binds `libgs130` through `ctypes` |
| ROS 2 wrapper | `ros2/` | The `gs130_camera` package, which publishes camera, IMU and TF data |

Core must be installed before either wrapper can be used.

## 🧩 Requirements

- **Hardware**: an RDK development board and a GS130 series stereo camera. Some camera models include an IMU.
- **Core**: the Horizon multimedia libraries (`libvpf`, `libhbmem` and `libcam`, installed under `/usr/hobot/lib`), the OpenCV 4 development headers, a TBB shared library (`libtbb.so.2` or `libtbb.so.12`, resolved from what the image installs), the OpenGL and LAPACK shared libraries the image's static OpenCV references (`libGL.so.1`, `liblapack.so.3`), and a GCC toolchain with C11 and C++17 support.
- **Python wrapper**: Python 3.10 or later and `numpy`. Building the wheel additionally requires `setuptools` and `wheel`.
- **ROS 2 wrapper**: ROS 2 Humble or Jazzy. The web preview and stereo depth launch files also require the TROS packages `hobot_codec`, `websocket` and `hobot_stereonet`.

The current release implements the RDK X5, RDK S100 and RDK S600 backends.

## 🗂️ Repository layout

```
core/
  include/gs130.h            Public C API
  include/gs130_define.h     Platform configuration presets
  src/                       Library implementation (base, devices, tools)
  samples/                   gs130 front end and sample programs
python/
  gs130_camera/              Python wrapper, with type stubs
  test/test_gs130.py         Hardware test
ros2/src/gs130_camera/       ROS 2 package: node, launch files, documentation
VERSION                      Version shared by Core and the Python wrapper
LICENSE
```

## 🔨 Build and installation

### 1. Core

The library, the tools and the Debian package are built with the GNU Makefile in `core/`. A platform name may be passed to `make`, although it is only honoured as the first goal on the command line.

```bash
cd core
make -j$(nproc)            # library, samples, tools and the Debian package
make lib                   # library only
make RDKX5 samples         # select the platform explicitly
```

Build output is written to `build/<platform>/` and `out/<platform>/`, and the Debian package to `out/`. When `make` runs on a terminal it requests confirmation once before compiling; the question is skipped when standard input is not a terminal.

```bash
# Check whether the SDK is already installed
dpkg -s gs130-camera

# If not, install the Debian package built from core/
sudo dpkg -i out/gs130-camera_<version>+<platform>_<arch>.deb
```

Installing `gs130-camera` is sufficient for the steps that follow.

### 2. Python wrapper

The wheel is built on the board with the toolchain already present.

```bash
cd python && ./build-wheel.sh
python3 -m pip install dist/gs130_camera-*.whl
```

`./build-wheel.sh clean` removes the build directory and the generated metadata. The version is taken from the repository-level `VERSION` file, so the package must be built from a checkout. The wheel does not bundle the native library; an installed `libgs130` is located at run time.

See the [Python README](python/README.md) for details.

### 3. ROS 2 wrapper

```bash
cd ros2
source /opt/ros/humble/setup.bash     # or: source /opt/ros/jazzy/setup.bash
colcon build --packages-select gs130_camera
source install/setup.bash
```

See the [ROS 2 README](ros2/src/gs130_camera/README.md) for details.

## 🚀 Getting started

**Command-line tools**

The gs130 tools are installed together with Core, and the sample sources are kept in [core/samples/](core/samples). They serve both as diagnostic utilities and as reference implementations.

```bash
gs130 help                       # list the available commands
gs130 version                    # SDK version and build platform
gs130 detect <imu|eeprom|camera> -b <bus...> [-a <addr...>]
gs130 shell -d <device> [-m <mode>] [-w <W>] [-h <H>] [-f <fps>] [-o <odr>]
```

`gs130 shell` locks a single device configuration and then accepts the remaining commands interactively.

```console
gs130-shell>> imu-info                                                              # detected IMU model and its details
gs130-shell>> eeprom-info                                                           # EEPROM calibration model and its details
gs130-shell>> calib-export <dir>                                                    # Kalibr YAML: camchain.yaml and imu.yaml
gs130-shell>> run                                                                   # stream camera and IMU, print the newest of each
gs130-shell>> rec [-c <from:to>] [-i <from:to>] -o <dir> [--stitch]                 # record index ranges to disk
```

**C language**

The public API is declared in `core/include/gs130.h`, and the platform presets that describe known hardware are defined in `core/include/gs130_define.h`. A program links against `libgs130`, creates a device handle, applies a configuration, and reads frames and IMU samples from it.

```c
#include "gs130.h"
#include "gs130_define.h"

#include <stdlib.h>
#include <unistd.h>

/* Preset for this camera on the platform the library was built for. */
gs130_config_t config = GS130_CONFIG(
    "GS130WI", GS130_CAMERA_MODE_RECT, 544, 448, 30, 200);

gs130_device_t *device = gs130_create();
if (gs130_init(device, &config) != GS130_OK ||
    gs130_start(device) != GS130_OK) {
    gs130_stop(device);
    gs130_deinit(device);
    gs130_destroy(device);
    return 1;
}

for (;;) {
    gs130_image_nv12_t left, right;
    while (gs130_get_nv12_frame(device, &left, &right) == GS130_OK) {
        /* NV12, width * height * 3 / 2 bytes; the buffer is ours to free. */
        free(left.data);
        free(right.data);
    }

    /* Non-blocking: this drains what has arrived and ends on an empty FIFO.
       A device without an IMU never returns a packet. */
    gs130_imu_packet_t packet;
    while (gs130_get_imu_packet(device, &packet) == GS130_OK) {
        /* packet.accel m/s^2, packet.gyro rad/s, packet.temp degrees Celsius;
           packet.timestamp_ns is on the camera clock. */
    }

    usleep(1000);
}

gs130_stop(device);
gs130_deinit(device);
gs130_destroy(device);
```

Complete programs, including the rate calculation and the interrupt-driven shutdown, are provided in [core/samples/](core/samples).

**Python**

The Python wrapper returns camera frames and IMU samples as NumPy arrays.

```python
import gs130_camera

config = gs130_camera.preset(
    "GS130WI", gs130_camera.CameraMode.RECT, 544, 448, 30, 200
)
with gs130_camera.Device(config) as device:
    device.start()
    while device.available_camera() == 0:
        pass
    left = device.read_image()["left"]

    # None when the queue is empty, and on a device that has no IMU
    packet = device.read_imu()
    if packet is not None:
        # packet.accel m/s^2, packet.gyro rad/s, packet.temp degrees Celsius
        print(packet.accel, packet.gyro, packet.temp)
```

**ROS 2**

The ROS 2 wrapper publishes the camera, the calibration data and the IMU as standard messages. The topic and parameter reference is given in the package documentation.

```bash
ros2 launch gs130_camera gs130.launch.py             # images, calibration, IMU and TF
ros2 launch gs130_camera gs130_websocket.launch.py   # the same, previewed in a browser
ros2 launch gs130_camera gs130_stereonet.launch.py   # stereo depth on the same page
```

## ✨ Features

- **Hardware rectification.** In `rect` mode, undistortion and rectification are performed on the GDC hardware, so no host-side image processing is required.
- **Selectable stereo layouts.** The two eyes may be delivered separately or combined into a single frame, using one of four layouts.
- **Calibration from the EEPROM.** Intrinsics, distortion coefficients, extrinsics and IMU parameters are read from the onboard EEPROM and can be exported as Kalibr YAML. After rectification, virtual intrinsics are written back. The exported `T_cam_imu` is IMU-to-camera; a consumer that reads that key as `T_imu_cam` (OpenVINS does) has to invert it, and the exported file's header says so.
- **FSYNC-aligned IMU.** The camera drives the IMU time base, so both streams are expressed on a single clock.
- **Standard ROS 2 interfaces.** Images, camera information, inertial data and static transforms are published as standard messages; no custom message types are introduced.
- **Consistent versioning.** The native library and the Python wrapper share the version declared in `VERSION`, and the wrapper rejects a library that is older than the package.

The following capabilities are not provided in this release: zero-copy transport (`hbm_img_msgs` over shared memory), custom message types, IMU filtering, run-time reconfiguration of parameters, and synchronisation of multiple devices.

## 📄 License

Released under the [MIT License](LICENSE).
