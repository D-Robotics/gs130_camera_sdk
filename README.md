# GS130 Stereo Camera SDK

[简体中文](README.zh-CN.md) | **English**

> The SDK for the GS130 stereo camera and its IMU, containing a C library, a Python wrapper and a ROS 2 wrapper.
>
> **This is a developer preview (alpha) release**; interfaces and behaviour may still change. Feedback and co-creation are welcome — through the [D-Robotics developer community](https://forum.d-robotics.cc/), or by mail at [xiaoye.zhang@d-robotics.cc](mailto:xiaoye.zhang@d-robotics.cc).

---

## 📖 Overview

This repository is the SDK for the GS130 stereo camera family. A single native library does the hardware work — sensor capture, ISP, GDC rectification, EEPROM calibration and FSYNC-aligned IMU sampling — and three layers sit on top of it:

| Layer | Directory | Contents |
| --- | --- | --- |
| Core | `core/` | `gs130` tools and the C API sources |
| Python wrapper | `python/` | sources for building the `gs130_camera` library |
| ROS 2 wrapper | `ros2/` | sources for building the `gs130_camera` package |

| Item | Description |
| --- | --- |
| Platform | RDK X5 (RDK S100 / RDK S600 in development) |
| License | MIT |
| Languages | C11 & C++17 (library), Python 3.10+ (wrapper), C++17 (ROS 2 node) |

## 🧩 Requirements

- **Hardware**: an RDK development board with a GS130 series stereo camera (some models include an IMU).
- **Core**: the Horizon multimedia libraries (`libvpf`, `libhbmem`, `libcam`, in `/usr/hobot/lib`), OpenCV 4 headers, `libtbb.so.2`, and a GCC toolchain with C11 & C++17.
- **Python wrapper**: Python 3.10 or newer, `numpy`, and `setuptools` plus `wheel` to build the wheel.
- **ROS 2 wrapper**: ROS 2 Humble or Jazzy. The web preview and stereo depth launch files additionally need the TROS packages `hobot_codec`, `websocket` and `hobot_stereonet`.

Every layer needs the native library, so build and install `core/` first.

## 🗂️ Repository layout

```
core/
  include/gs130.h            public C API
  include/gs130_define.h     platform configuration presets
  src/                       library sources (base, devices, tools)
  samples/                   the gs130 front end and the sample programs
python/
  gs130_camera/              the wrapper, with its type stubs
  test/test_gs130.py         hardware test
ros2/src/gs130_camera/       the ROS 2 package: node, launch files, README
VERSION                      version shared by the library and the wrapper
LICENSE
```

## 🔨 Build and install

### 1. Core

```bash
cd core
make -j$(nproc)            # library, samples, tools and the .deb for the default platform
make lib                   # library only
make RDKX5 samples         # a platform name is only honoured as the first goal
```

Artifacts land in `build/<platform>/`, `out/<platform>/`, and in `out/` for the `.deb`. An interactive shell is asked once for confirmation before compiling; non-interactive builds skip the question.

```bash
# Check whether the SDK is already installed
dpkg -s gs130-camera

# If not, install the deb built from core/
sudo dpkg -i out/gs130-camera_<version>+<platform>_<arch>.deb
```

Installing `gs130-camera` is enough.

### 2. Python wrapper

```bash
cd python && ./build-wheel.sh
python3 -m pip install dist/gs130_camera-*.whl
```

See the [Python README](python/README.md) for details.

### 3. ROS 2 wrapper

```bash
cd ros2
source /opt/ros/humble/setup.bash     # or: source /opt/ros/jazzy/setup.bash
colcon build --packages-select gs130_camera
source install/setup.bash
```

See the [ROS 2 README](ros2/src/gs130_camera/README.md) for details.

## 🚀 Quick start

**gs130 tools**

These ship with Core: the `.deb` installs them into `/usr/bin`. Their sources are the files under [core/samples/](core/samples), with the three `detect` tools under [core/src/tools/](core/src/tools) — they are test tools and samples at the same time.

```bash
gs130 help                       # list the commands
gs130 version                    # SDK version and build platform
gs130 detect <imu|eeprom|camera> [-b <bus...>] [-a <addr...>]
gs130 shell -d <device> [-m <mode>] [-w <W>] [-h <H>] [-f <fps>] [-o <odr>]
```

`shell` locks one device configuration, then takes these commands interactively:

```console
gs130-shell>> imu-info                                                              # detected IMU model and its details
gs130-shell>> eeprom-info                                                           # EEPROM calibration model and its details
gs130-shell>> calib-export <dir>                                                    # Kalibr YAML: camchain.yaml and imu.yaml
gs130-shell>> run                                                                   # stream camera and IMU, print the newest of each
gs130-shell>> rec [-c <from:to>] [-i <from:to>] -o <dir> [--stitch]                 # record index ranges to disk
```

**C language**

Link `libgs130` and drive the camera from a program of your own:

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

The complete programs, with the Ctrl-C handling and the rate calculation, are the ones under [core/samples/](core/samples).

**Python**

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

```bash
ros2 launch gs130_camera gs130.launch.py             # images, calibration, IMU and TF
ros2 launch gs130_camera gs130_websocket.launch.py   # the same, previewed in a browser
ros2 launch gs130_camera gs130_stereonet.launch.py   # stereo depth on the same page
```

## ✨ Features

- **Hardware rectification**: in `rect` mode the undistortion and rectification run on the GDC hardware, so no host-side image processing is needed.
- **Five stereo layouts**: separate eyes, left-right, right-left, top-bottom and bottom-top.
- **Calibration from the EEPROM**: intrinsics, distortion, extrinsics and IMU parameters, with a Kalibr YAML export and a virtual-intrinsics write-back after rectification.
- **FSYNC-aligned IMU**: the camera drives the IMU time base, so both streams carry one clock.
- **Standard ROS 2 output**: `sensor_msgs/Image`, `sensor_msgs/CameraInfo`, `sensor_msgs/Imu` and static TF, with no custom messages.
- **One version, checked**: the library and the wrapper share `VERSION`; the wrapper refuses to load a library older than the package and warns when it is newer.

## 📄 License

Released under the [MIT License](LICENSE).
