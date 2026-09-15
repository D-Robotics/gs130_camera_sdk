# gs130_ros

Publishes a GS130 stereo camera and its IMU as standard ROS 2 messages, by
calling the SDK's C API directly.

C++ with `rclcpp` and `ament_cmake`, linking `libgs130.so`. The node is a thin
mapping of `gs130.h` onto `sensor_msgs`, so the streams drop into the official
D-Robotics perception stack. It does no image processing, no stereo matching
and no filtering.

## What it needs

- ROS 2 Humble, or TROS, on an RDK X5. TROS is an overlay on the plain ROS
  install, and only the overlay has the D-Robotics packages, so source both:

      source /opt/ros/humble/setup.bash    # rclcpp and the build system
      source /opt/tros/humble/setup.bash   # hobot_stereonet and friends

- `libgs130.so` and `gs130.h`. Building from the `gs130_sdk` checkout finds the
  header next to this package; a standalone build needs `GS130_ROOT`, or
  `-DGS130_LIBRARY=` and `-DGS130_INCLUDE_DIR=`. Neither is a rosdep key, which
  is why `package.xml` does not list them.

## Build and run

    mkdir -p ~/ws/src && ln -s "$PWD/ros/gs130_ros" ~/ws/src/gs130_ros
    cd ~/ws
    source /opt/ros/humble/setup.bash
    source /opt/tros/humble/setup.bash
    colcon build --packages-select gs130_ros
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
| `image_width`, `image_height` | `640`, `350` | Output size of **one** eye. See [Feeding hobot_stereonet](#feeding-hobot_stereonet) for why. |
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

`make_config()` in `src/node.cpp` mirrors the `GS130_CONFIG_*` macros field by
field instead of calling them, because it has to: the macros initialise their
arrays with GNU range designators (`[0 ... 3] = 0xFF`), which `g++` rejects
outright -- `gcc -std=gnu99` accepts them, `g++ -std=gnu++17` does not. They
also `exit(1)` on an unknown board, where a launch file needs an error it can
report. The two are kept comparable line by line.

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
| Output size | exactly the configured size per eye, stitched to double the height |
| Frame rate | 30.2 Hz published, camera_info alongside it at 30.1 Hz |
| IMU | 201.5 Hz at `imu_odr:=200`, FSYNC handshake completed |
| Stamp latency | 3 ms (IMU), 28 ms (image) behind the system clock |
| `hobot_stereonet` | 15.0 fps end to end, depth 1.031x the geometric value |

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

## Feeding hobot_stereonet

The official depth pipeline takes a 640x352 input and rescales whatever it is
given to that shape. Three things follow, and the default output size is chosen
to satisfy all of them at once:

- The rescale does not preserve aspect. A 1088x1280 rectified pair became
  640x352 by scaling x by 0.588 and y by 0.275, stretching the picture about
  2.1x across.
- The intrinsics are rescaled by the same two factors, and then rescaled
  *again* in the resize branch (`stereonet_component.cpp:885`), which cost a
  factor of 1.7 in every depth it reported. Measured against an independent
  match of the same frame, its depth came out at 0.607x the geometric value.
- 640x352 exactly is what avoids all of it, and the SDK will not produce it.
  Its aspect-preserving crop demands an exact integer ratio (`vse.c`
  `roi_ratio_exact`), which for a 640-wide output means the height has to be a
  multiple of ten; 352 is not, and `gs130_init` returns `GS130_UNSUPPORTED`.

**640x350 is the answer**, and it is the default. It is the closest height the
SDK accepts at the model's own width, and because the width matches exactly the
horizontal scale factor stereonet applies is 1.0 -- so the second rescaling
leaves `fx` untouched, and the only residue is a 0.57% change in `fy` from the
two-pixel height difference. Its own log, with no parameters set at all:

    => sub rectified  [fx, fy, cx, cy, ...] : [362.067018, 364.135972, 320.0, 176.3, ...]
    => after resize   [fx, fy, cx, cy, ...] : [362.067018, 366.216749, 320.0, 177.4, ...]
                                               ^^^^^^^^^^ unchanged

So the SDK does the rectification on its GDC hardware, the node publishes
640x350 at 30 Hz, and stereonet's stock launch file reads it as it is:

    ros2 launch gs130_ros gs130.launch.py
    ros2 launch hobot_stereonet stereonet_model_no_web.launch.py

Measured that way: 15.0 fps through stereonet, 140-160 ms latency, and depth at
1.031x the geometric value over 165 confident matches. `image_width:=1088
image_height:=1280` still gives the whole sensor field of view for anything
that does not want the model's shape.

### If you would rather stereonet did the rectifying

`gs130_calibration` writes the device's real calibration -- the raw fisheye
intrinsics and distortion of each eye, and the transform between them -- as the
`cam0`/`cam1` YAML that `calib_method:=custom` reads. That path rectifies from
the file instead of resizing, so it needs no particular image size, but it does
the rectification on the CPU and runs at 8.5 fps against 15.0:

    ros2 run gs130_ros gs130_calibration --output ~/gs130_stereo_calib.yaml --fov-scale 0.455

    ros2 launch gs130_ros gs130.launch.py camera_mode:=raw image_height:=1280 stereo_layout:=top_bottom &
    ros2 launch hobot_stereonet stereonet_model_no_web.launch.py use_mipi_cam:=False \
      calib_method:=custom stereo_calib_file_path:=$HOME/gs130_stereo_calib.yaml \
      camera_info_topic:=/gs130_unused_right left_camera_info_topic:=/gs130_unused_left

The two `camera_info_topic` overrides are not optional there: in custom mode
stereonet derives its intrinsics from the file, and its `camera_info_callback`
would overwrite them with ours scaled to model space -- or, if ours arrived
first, the block that builds the rectification maps would never run at all.

The node cannot export that file itself: under `rect` the C layer replaces the
intrinsics and the rotation with virtual ones while initialising the camera, so
the real values are gone before the node is running. They are still in the
EEPROM, and the tool opens the device without rectification to read them,
which is why it is a tool and not part of the node.

`--fov-scale` is worth knowing about. Left out, stereonet picks 0.8, which
rectifies to a 124.7 degree horizontal field of view -- wider than the
fisheye's own 94.8 degrees -- so the picture has black wedges down both sides.
0.455 matches the sensor and removes them; the fisheye's field of view follows
from its equidistant focal length, `r = f * theta`, so with f = 657.65 px and a
half-width of 544 px, theta = 0.827 rad and the horizontal field of view is
94.8 degrees.

## A note on capture threads

Capture runs on two dedicated `std::thread`s, not on `rclcpp` timers, the same
way the official node does it. That is not only a matter of taste: on the TROS
`rclcpp` an `rclcpp` timer bound to an explicitly created callback group never
fires under `MultiThreadedExecutor` -- the node starts, the publisher exists,
and not one frame is published -- while the identical timer on the node's
default callback group works, and `executor.add_callback_group()` for those
groups then reports they were already added. Threads do not depend on the
executor's callback-group bookkeeping at all, so the node keeps working
whatever that behaviour turns out to be.

## Tests

    cd ~/ws && colcon test --packages-select gs130_ros && colcon test-result --all

or run the binary directly:

    ./build/gs130_ros/test_convert

They cover the conversions only, and link just `gs130_ros_convert` and the
generated messages -- never `libgs130`, because the conversions use the C
header's types and call none of it. So they run on a machine with no camera and
no library installed, which is where most of this package's logic belongs.

The packed-frame test builds the stitched NV12 buffer from the offsets in
`gs130.cpp` rather than from `slice_eyes`, so the layout the node writes and the
layout it reads back are checked against each other rather than against
themselves.

## Not implemented

Zero-copy (`hbm_img_msgs` over shared memory), custom messages, IMU filtering,
online reconfiguration, multi-device synchronisation, and any image processing.
The node is a plain executable, not an `rclcpp_components` component, so it
cannot be composed into a container with `hobot_stereonet`.
