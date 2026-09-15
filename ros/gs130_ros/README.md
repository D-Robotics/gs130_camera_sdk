# gs130_ros

Publishes a GS130 stereo camera and its IMU as standard ROS 2 messages.

The node is a thin mapping of the C API in `core/` onto `sensor_msgs`, so the
streams drop into the official D-Robotics perception stack. It does no image
processing, no stereo matching and no filtering.

## What it needs

- ROS 2 Humble, or TROS (`/opt/tros/humble`), on an RDK X5.
- `libgs130.so` for the platform, installed where the loader finds it.
- The `gs130` Python binding, built from this repository:

      cd gs130_sdk
      pip3 install ./python

  The binding loads `libgs130` and refuses a library older than itself, so
  install the two from the same checkout. Neither is a rosdep key, which is why
  `package.xml` does not list them.

## Build and run

    mkdir -p ~/ws/src && ln -s "$PWD/ros/gs130_ros" ~/ws/src/gs130_ros
    cd ~/ws && source /opt/tros/humble/setup.bash && colcon build
    source install/setup.bash
    ros2 launch gs130_ros gs130.launch.py

The defaults publish the rectified stereo pair, stacked with the left eye on
top, on `image_combine_raw` -- the configuration `hobot_stereonet` expects.
Hardware and interface choices are launch arguments:

    ros2 launch gs130_ros gs130.launch.py device:=GS130W camera_mode:=raw
    ros2 launch gs130_ros gs130.launch.py stereo_layout:=none framerate:=15

To run the official depth pipeline against it:

    ros2 launch gs130_ros gs130.launch.py &
    ros2 launch hobot_stereonet stereonet_model_component_no_web.launch.py

## Topics

| Topic | Type | Notes |
|---|---|---|
| `image_combine_raw` | `sensor_msgs/Image` | The stitched pair, `nv12`. Only when `stereo_layout:=top_bottom`. |
| `image_combine_raw/left/camera_info` | `sensor_msgs/CameraInfo` | The left eye's own size, not the frame's. |
| `image_combine_raw/right/camera_info` | `sensor_msgs/CameraInfo` | This is the one `hobot_stereonet` reads. |
| `image_stereo_raw` | `sensor_msgs/Image` | The stitched pair for any other layout. |
| `image_left_raw`, `image_right_raw` | `sensor_msgs/Image` | The two eyes, when they arrive separately or `publish_left_right` is on. |
| `<image topic>/camera_info` | `sensor_msgs/CameraInfo` | One per eye, on the topic its image is published on. |
| `/imu_data` | `sensor_msgs/Imu` | Absolute name, frame `imu_link`. |
| `/imu_extrinsic` | `geometry_msgs/TransformStamped` | Latched; see below. |

Image and IMU QoS match `hobot_mipi_cam`: depth 5 for images, 10 for the IMU.
camera_info goes out immediately before the image it belongs to, with the same
stamp, so a consumer keyed on the stamp always has the calibration first.

`/tf_static` carries one transform, `camera_link` to `imu_link`. There are no
per-eye optical frames: the device's axis convention has not been checked
against REP-103 yet, and a frame named `*_optical_frame` that is not one
silently reorients everything downstream.

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `platform` | `RDKX5` | Board, selects the preset. |
| `device` | `GS130WI` | Camera model: `GS130WI` or `GS130W`. |
| `camera_mode` | `rect` | `raw`, `resize` or `rect`. Only `rect` yields a stereo pair usable for depth. |
| `image_width`, `image_height` | `1088`, `1280` | Output size of **one** eye. |
| `framerate` | `30` | Camera frames per second. |
| `imu_odr` | `200` | IMU output data rate in Hz. |
| `stereo_layout` | `top_bottom` | `none`, `top_bottom`, `bottom_top`, `left_right`, `right_left`. |
| `frame_id` | `camera_link` | Frame every image is stamped with. |
| `imu_frame_id` | `imu_link` | Frame every IMU sample is stamped with. |
| `publish_imu` | `true` | Publish the IMU when the device has one. |
| `publish_left_right` | `false` | Also publish the two eyes separately when the frames arrive stitched. |
| `only_when_subscribed` | `false` | Stop capturing frames while nothing is subscribed. |
| `timestamp_source` | `auto` | `auto`, `device` or `system`. |
| `topic_*` | see the launch file | Topic names. |

The presets pin `stereo_layout` to `NONE` in C (`gs130_define.h:30`), which is
why the layout is a parameter applied on top of them rather than a preset
choice.

## Where this differs from `hobot_mipi_cam`

The names, sizes and QoS follow the official node. Four things are deliberate:

- **`P[3]` is only set under `rect`.** Under `raw` and `resize` there is no
  virtual parallel pair, so there is no baseline to report and `P` keeps the
  eye's own `K` with a zero translation. Publishing `P[3] = 0` under `rect`
  would make a stereo consumer compute all-zero depth and report success, so
  under `rect` the baseline is always filled in, and if the calibration cannot
  supply one, camera_info is withheld and the reason is logged at ERROR.
- **`CameraInfo.R` is the identity.** `R` means the rotation into the rectified
  frame, not the camera's mounting pose. The official dual-camera path copies
  the device rotation into it (`hobot_mipi_calibration.cpp:1298`); the
  extrinsics belong in `P` and TF.
- **`/imu_extrinsic` is latched.** The official node builds a
  `transient_local` QoS profile and then does not pass it to
  `create_publisher` (`hobot_mipi_node.cpp:470-472`), leaving a race against
  the first IMU sample for late subscribers. Here it is published once, on the
  profile that was meant, and on `/tf_static`.
- **A non-`top_bottom` layout is not published on `image_combine_raw`.** It
  goes to `image_stereo_raw` with a warning instead. `hobot_stereonet` splits
  the frame at `height / 2` and assumes the left eye is on top
  (`stereonet_component.cpp:647`), so a different layout on that topic would
  be silently misread rather than rejected.

## Timestamps

Stamps are the device's own, never the receive time, because a frame's age is
what a fusion consumer cares about and `this->now()` would erase it. `auto`
checks the clock for the first 20 frames -- increasing, on the Unix epoch, and
agreeing with the IMU -- and falls back to the receive time with an ERROR if
any of that fails. `timestamp_source:=device` skips the check and
`:=system` skips the device clock entirely.

**On an RDK X5 the device clock is not the Unix epoch.** The carrier board has
no RTC, and the SDK's camera clock counts nanoseconds since boot, so a working
setup logs this on every start:

    the device clock looks unusable (the first camera timestamp (...) looks like
    a monotonic clock rather than a Unix epoch); stamping every message with the
    receive time instead

That is the expected outcome, not a fault: the device clock is monotonic and
self-consistent, but it cannot be compared with another node's stamps, so the
node uses the system clock and says so once. Set the clock (`sudo date -s ...`,
or NTP) and the message disappears; if the monotonic clock is what you want,
`timestamp_source:=device` keeps it.

Nothing is published until the check is done, so a subscriber never sees
boot-clock stamps followed by a jump of decades.

## Verified on hardware

Measured on an RDK X5 with a GS130WI, `rect`, 1088x1280 per eye, 30 fps:

| What | Measured |
|---|---|
| Rectified baseline | 0.0703162 m, purely along x (off-axis < 1e-14 m) |
| Rectified rotation | identity to 2.3e-14 |
| Rectified intrinsics | fx = fy = 615.5139, cx = 544.0, cy = 640.0, distortion zeroed |
| Output size | 1088x2560 stitched, exactly the configured 1088x1280 per eye |
| Frame rate | 30.2 Hz published, camera_info alongside it at 30.1 Hz |
| IMU | 201.5 Hz at `imu_odr:=200`, FSYNC handshake completed |
| Stamp latency | 3 ms (IMU), 28 ms (image) behind the system clock |
| `hobot_stereonet` | accepts the stream as `nv12` and derives baseline 0.070316 m |

The last row is the whole point: with our camera_info, the official depth
pipeline reports `[fx, fy, cx, cy, baseline(m), doffs]` as
`[362.07, 169.27, 320.00, 176.00, 0.070316, 0.000000]` at its 640x352 model
input -- our per-eye intrinsics rescaled by exactly the model-to-eye ratio, and
a metric baseline taken from `P[3] / P[0]`.

## Still unverified

- Whether the device's axes follow REP-103. The IMU sits at
  `(0.0695, 0.0032, -0.0311)` in the left camera's frame with a rotation of
  about 179 degrees about y, which is either how the IMU is mounted or how the
  SDK reports it -- the two cannot be told apart without tilting the board by
  hand. Until they can, this node publishes no per-eye optical frames.
- Depth values against a measured distance. The pipeline runs end to end and
  the baseline is right, but no target of known range has been placed in front
  of it.

## Tests

    cd ros/gs130_ros
    python3 -m pytest test -q

They import `gs130` when it is available and otherwise install a stand-in with
the enum values copied out of `core/include/gs130.h`, so the conversions can be
checked on a machine that has neither libgs130 nor the binding; on the board the
real one is used.

Under TROS, add `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`. TROS ships a
`launch_testing` pytest plugin whose hooks do not match the pytest on the board,
and pytest fails to start before it collects anything.

## Not implemented

Zero-copy (`hbm_img_msgs` over shared memory), custom messages, IMU filtering,
online reconfiguration, multi-device synchronisation, and any image processing.

## A note on the image payload

`sensor_msgs/Image.data` is filled with an `array.array("B")`, not a `bytes`.
Both are valid, and on an RDK X5 the difference is the whole frame rate: rclpy
fills a `uint8[]` field one element at a time when handed a `bytes`, which costs
3.3 seconds for a 1088x2560 NV12 frame, against 0.8 ms for an `array.array`.
`test_convert.py` asserts the type for that reason.
