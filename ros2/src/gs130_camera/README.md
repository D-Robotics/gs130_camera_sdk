# GS130 Stereo Camera SDK — ROS Interface

[简体中文](README.zh-CN.md) | **English**

> A lightweight ROS 2 driver that brings the GS130 stereo camera and its onboard IMU onto ROS 2 topics by calling the GS130 SDK C API directly.

---

## 📖 Overview

`gs130_camera` publishes the GS130 stereo camera and its IMU as **standard ROS 2 messages**. The node calls the GS130 SDK C API directly (linking `libgs130.so`) and performs no image processing, stereo matching or filtering of its own; in `rect` mode the undistortion and rectification are done by the SDK on the GDC hardware.

| Item | Description |
| --- | --- |
| Language | C++17 (node) + C99 (SDK preset bridge) |
| Build | `ament_cmake` / `colcon` |
| Platform | RDK X5 (RDK S100 / RDK S600 in development) |
| Requires | ROS 2 Humble or Jazzy, `libgs130.so` |

## 🧩 Dependencies

- **Hardware**: an RDK series development board with a GS130 series stereo camera (some models include an IMU). Currently supported: **RDK X5**; RDK S100 and RDK S600 support is in development.
- **ROS 2**: Humble or Jazzy. Source the distribution installed on the board, and on a TROS system source the TROS overlay after it:

  ```bash
  source /opt/ros/humble/setup.bash     # or: source /opt/ros/jazzy/setup.bash
  source /opt/tros/humble/setup.bash    # only on TROS, matching the distro above
  ```

- **GS130 SDK**: the `gs130-camera` package, which provides `libgs130.so`, `libgs130.a`, `gs130.h` and `gs130_define.h`. It has to be installed separately.
- **Optional**: the TROS packages `hobot_codec` and `websocket` (web preview), and `hobot_stereonet` (stereo depth).

## 🔨 Build

**1. Install the SDK first**

```bash
# Check whether it is already installed
dpkg -s gs130-camera

# If not, install the deb built from core/ (`make deb` in the SDK checkout)
sudo dpkg -i gs130-camera_<version>+<platform>_<arch>.deb
```

**2. Build the ROS 2 package**

```bash
cd ros2
source /opt/ros/humble/setup.bash     # or: source /opt/ros/jazzy/setup.bash
colcon build --packages-select gs130_camera
source install/setup.bash
```

When building next to a checkout instead of an installed SDK, CMake picks up the headers from `core/include` and locates `libgs130` by name. If the layout differs, point CMake at the two paths explicitly:

```bash
colcon build --packages-select gs130_camera --cmake-args \
  -DGS130_LIBRARY=/path/to/libgs130.so \
  -DGS130_INCLUDE_DIR=/path/to/include
```

## 🚀 Running the node

```bash
# Run the node directly
ros2 run gs130_camera gs130_node --ros-args -p camera_mode:=rect -p stitch:=top_bottom

# Or use the base launch file (recommended)
ros2 launch gs130_camera gs130.launch.py
```

The executable is `gs130_node` and the node name is `gs130_camera`. Invalid parameters or a failed SDK initialization make the node log an error and exit.

## 📡 Topics

With `stitch` other than `none`, both eyes arrive in a single frame:

| Topic | Type | Notes |
| --- | --- | --- |
| `image_combine` | `sensor_msgs/Image` | Combined frame, `nv12` |
| `image_combine/left/camera_info` | `sensor_msgs/CameraInfo` | Left eye intrinsics, sized for one eye |
| `image_combine/right/camera_info` | `sensor_msgs/CameraInfo` | Right eye intrinsics; `P[3]` carries the baseline |
| `image_combine/gray` | `sensor_msgs/Image` | Optional, `mono8` |

With `stitch: none`, each eye gets its own frame:

| Topic | Type | Notes |
| --- | --- | --- |
| `image_left` / `image_right` | `sensor_msgs/Image` | Left / right eye, `nv12` |
| `image_left/camera_info` / `image_right/camera_info` | `sensor_msgs/CameraInfo` | Left / right eye intrinsics |
| `image_left/gray` / `image_right/gray` | `sensor_msgs/Image` | Optional, `mono8` |

The image topic names above are all parameters. The `camera_info` topic names are derived from the image topic they describe and are not parameters.

Either way the node also publishes:

| Topic | Type | Notes |
| --- | --- | --- |
| `/imu_data` | `sensor_msgs/Imu` | Angular velocity and linear acceleration; not published on devices without an IMU |
| `/tf_static` | `tf2_msgs/TFMessage` | `camera_link` → `camera_right_link`, plus `imu_link` when an IMU is present |

`/tf_static` is a reserved ROS topic; remap it if it needs to move.

## ⚙️ Parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| `device` | `GS130WI` | Camera model: `GS130WI` or `GS130W` |
| `camera_mode` | `rect` | `raw` / `resize` / `rect` |
| `stitch` | `none` | `none` / `left_right` / `right_left` / `top_bottom` / `bottom_top` |
| `output_width` | `544` | Output width of one eye, in pixels |
| `output_height` | `448` | Output height of one eye, in pixels |
| `fps` | `30` | Camera frame rate |
| `odr` | `200` | IMU output data rate, in Hz |
| `publish_gray` | `false` | Also publish each image as `mono8` |
| `image_topic` | `image_combine` | Combined frame topic (used when `stitch` is not `none`) |
| `left_image_topic` | `image_left` | Left eye topic (used when `stitch` is `none`) |
| `right_image_topic` | `image_right` | Right eye topic (used when `stitch` is `none`) |
| `imu_topic` | `/imu_data` | IMU topic |
| `timer_period_ms` | `1` | Publish timer period, in milliseconds |
| `frame_id` | `camera_link` | Frame of the left eye and of the combined frame |
| `right_frame_id` | `camera_right_link` | Frame of the right eye |
| `imu_frame_id` | `imu_link` | Frame the IMU samples are stamped with |

`output_width`, `output_height`, `fps` and `odr` must be positive integers, `timer_period_ms` must be positive, and `camera_mode` and `stitch` must be one of the values listed above; otherwise the node fails at startup.

## ✨ Features

- **Hardware rectification**: in `rect` mode the undistortion and rectification are done by the SDK on the GDC hardware; the node itself does no image processing.
- **Selectable stereo layout**: separate eyes plus 4 stitched layouts, 5 in total.
- **Calibration output**: when an EEPROM is present the node publishes both eyes' `CameraInfo` (the right eye carries the baseline in `P[3]`) and the static transforms. RAW mode still produces images without calibration but publishes no `CameraInfo` and no TF; RESIZE and RECT modes require calibration.
- **Optional IMU**: angular velocity and linear acceleration are published when the device has an IMU; `orientation` is marked unavailable following the `sensor_msgs/Imu` convention (first covariance element `-1`).
- **Ordered publishing**: a single timer emits one message per period, interleaving images and IMU samples by their SDK timestamps, which preserves sampling order and keeps a shallow subscriber queue from dropping the tail.
- **Timestamps**: message stamps are the **publication time**; SDK hardware timestamps are used only for internal ordering (the camera clock counts from boot on RDK boards, so it is not useful outside the board).
- **QoS**: image queue depth 5 and IMU queue depth 10, matching the `hobot_mipi_cam` convention.

Not implemented: zero-copy (`hbm_img_msgs` over shared memory), custom messages, IMU filtering, dynamic parameters, and multi-device synchronisation.

## 🖥️ Launch files

### 1. `gs130.launch.py` — base startup and parameter mapping

```bash
ros2 launch gs130_camera gs130.launch.py
ros2 launch gs130_camera gs130.launch.py stitch:=top_bottom publish_gray:=true
ros2 launch gs130_camera gs130.launch.py --show-args      # list every argument and default
```

Mapping rule: **each launch argument corresponds one-to-one to the node parameter of the same name**. The launch file also declares the target type of every argument (integer or boolean), so a value typed on the command line is converted to the type the node expects before it is passed down; the defaults match the table above.

### 2. `gs130_websocket.launch.py` — web preview

```bash
ros2 launch gs130_camera gs130_websocket.launch.py
```

This includes the base launch and additionally starts `hobot_codec` (`nv12` → `jpeg`) and a `websocket` streaming node (which also brings up nginx). Open the page in a browser:

```
http://<board-ip>:8000
```

- `stitch` other than `none`: the combined frame uses channel 0.
- `stitch: none`: the left eye uses channel 0 and the right eye channel 1.

The parameters that shape the preview are:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `stitch` | `none` | `none` previews one channel per eye; any other value previews the combined frame |
| `image_topic` | `image_combine` | Topic previewed when `stitch` is not `none` |
| `left_image_topic` | `image_left` | Left eye topic previewed when `stitch` is `none` |
| `right_image_topic` | `image_right` | Right eye topic previewed when `stitch` is `none` |
| `fps` | `30` | Frame rate handed to the encoder |

It declares no arguments of its own: every camera parameter listed above is accepted and passed straight through to the node.

### 3. `gs130_stereonet.launch.py` — stereo depth

```bash
ros2 launch gs130_camera gs130_stereonet.launch.py
```

This includes the base launch (with `stitch` pinned to `top_bottom`, because `hobot_stereonet` requires the left eye in the top half) and starts the `hobot_stereonet` depth model and its visualisation node. The coloured depth view is likewise watched in the browser:

```
http://<board-ip>:8000
```

It requires `hobot_stereonet` (TROS) to be installed. Its own parameters are:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `stereonet_model` | `DStereoV2.4_int8_544_448.bin` | Model file under `hobot_stereonet/config`, sized for one eye |
| `render_type` | `indoor` | Depth colouring: `indoor`, `outdoor`, `indoor-reverse`, `outdoor-reverse`, `distance`, `distance-reverse` |

The model input is a **single eye** of `544×448`, which matches the default `output_width` / `output_height`, so change the model and the two sizes together. The camera parameters are accepted as well; `image_topic`, `frame_id` and `right_frame_id` are forwarded to `hobot_stereonet`.

## 📄 License

Released under the [MIT License](../../../LICENSE).
