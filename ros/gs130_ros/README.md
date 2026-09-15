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
| `image_width`, `image_height` | `1088`, `598` | Output size of **one** eye. See below for why 598. |
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

## Feeding stereonet the real calibration

Everything above hands `hobot_stereonet` a *rectified* camera: `CameraInfo` with
zero distortion and a baseline, on an image the SDK already rectified. With
`calib_method:=none` that is all it gets, and it then resizes the image to its
model input and rescales the intrinsics, which is where both the stretch and
the depth error come from.

Its other mode, `calib_method:=custom`, rectifies from a calibration file
instead, so it never resizes and never rescales -- but it needs the *raw*
fisheye calibration, which the node cannot supply: under `rect` the C layer
replaces the intrinsics and the rotation with virtual ones while initialising
the camera, so by the time the node runs the real values are gone. They are
still in the EEPROM, and opening the device without rectification hands them
back. That is what `gs130_calibration` does:

    ros2 run gs130_ros gs130_calibration --output ~/gs130_stereo_calib.yaml --fov-scale 0.455

It writes the `stereo0/cam0/cam1` YAML stereonet reads: the equidistant
intrinsics and distortion of each eye at the sensor resolution, and
`T_cn_cnm1`, the transform from the left eye to the right.

Then run the camera unrectified at the sensor size, and stereonet from the file:

    ros2 launch gs130_ros gs130.launch.py camera_mode:=raw image_height:=1280 stereo_layout:=top_bottom &
    ros2 launch hobot_stereonet stereonet_model_no_web.launch.py use_mipi_cam:=False \
      calib_method:=custom stereo_calib_file_path:=$HOME/gs130_stereo_calib.yaml \
      camera_info_topic:=/gs130_unused_right left_camera_info_topic:=/gs130_unused_left

The two `camera_info_topic` overrides are not optional. In custom mode
stereonet derives its intrinsics from the calibration file, and its
`camera_info_callback` would overwrite them with ours scaled to model space --
and if ours arrived first, the block that builds the rectification maps would
never run at all. Pointing them at topics nobody publishes keeps the
calibration file authoritative.

With this, stereonet reports `width, height scale: [1, 1]` (the image
resolution matches the calibration's, so nothing is rescaled), rectifies to
`fx = fy = 294.511`, and the intrinsics are never doubled. Measured against an
independent match of stereonet's own rectified images, its depth then comes out
at **1.069x** the geometric value (median of 94 confident matches) where the
`none` path gave 0.607x.

`--fov-scale` is optional and worth understanding. Left out, stereonet picks
0.8, which on a GS130WI rectifies to a 124.7 degree horizontal field of view --
wider than the fisheye's own 94.8 degrees -- so the rectified image has black
wedges down both sides. 0.455 brings it to 94.75 degrees, matching the sensor,
with no black. The fisheye's field of view follows from its equidistant focal
length: `r = f * theta`, so with f = 657.65 px and a half-width of 544 px,
`theta = 0.827 rad` and the horizontal field of view is 94.8 degrees.

## Why the default height is 598

The sensor rectifies to a portrait field of view, 1088x1280, while every depth
model `hobot_stereonet` ships takes a 640x352 landscape input. When the two do
not match, stereonet resizes the image to its model input with a plain
`cv::resize`, which does not preserve aspect: 1088x1280 became 640x352 by
scaling x by 0.588 and y by 0.275, stretching the picture about 2.1x across.
Its own log says so:

    => input image size not match model input size, need resize, [1088 x 1280] -> [640 x 352]

It also rescales the intrinsics by the same two factors, so the depth scale is
wrong by a second effect described below.

`598 = 1088 * 352 / 640` is the height that gives one eye the model's 20:11
aspect, so the resize becomes isotropic -- fx and fy come out 362.07 and 362.31
instead of 362.07 and 169.27, and the picture is no longer distorted. It is also
the nearest height that the SDK accepts: its aspect-preserving crop demands an
exact integer ratio (`vse.c` `roi_ratio_exact`), and 640x352 itself is refused
with `GS130_UNSUPPORTED` because 1088 * 352 is not a multiple of 640.

The cost is field of view: the rectified image is cropped to a centred
landscape band, roughly half its height. Pass `image_height:=1280` to get the
whole sensor back and accept the stretch in stereonet's rendering.

## stereonet reports depth about 1.7x too small

Measured, with the aspect already matched: an independent zero-mean
normalised-cross-correlation match of the same frame, using the baseline the
device reports, gives distances that stereonet's published depth map scales by
a median factor of 0.607 (81 confident matches, ZNCC >= 0.85). Its log shows
why:

    => sub rectified                     [fx, fy, ...] : [362.067, 362.309, ...]
    => after resize, update camera intrinsic [fx, fy, ...] : [212.981, 213.266, ...]

362.067 is `615.514 * 640/1088`, the intrinsics correctly rescaled into its
640x352 model space. 212.981 is that scaled by 0.588 a second time, and
212.981/362.067 = 0.588 is the factor the ratio above shows up as.

The second scaling sits inside the same resize branch and is guarded by
`camera_info_updated_`, which its `camera_info_callback` sets before it
finishes -- so by the time an image is preprocessed the guard should already be
true and the branch should be skipped. It is not, and delaying the camera_info
to arrive after the first frame does not change it, so the guard does not
behave the way the source reads. `stereonet_component.cpp:885-897` is the code
in question.

This is on stereonet's side and cannot be fixed from a publisher without
publishing intrinsics that are wrong for everyone else. It is also why the
`calib_method:=custom` setup above exists: that path rectifies from a
calibration file and never enters this branch, so the depth comes out right.

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
