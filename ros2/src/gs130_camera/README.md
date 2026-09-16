# gs130_camera

Publishes a GS130 stereo camera and its IMU as standard ROS 2 messages, by
calling the SDK's C API directly. C++ with `rclcpp` and `ament_cmake`, linking
`libgs130.so`. It does no image processing, no stereo matching and no filtering.

## What it needs

- ROS 2 Humble, or TROS, on an RDK X5. TROS is an overlay on the plain ROS
  install and only the overlay has the D-Robotics packages, so source both:

      source /opt/ros/humble/setup.bash
      source /opt/tros/humble/setup.bash

- `libgs130.so` and `gs130.h`, from the `gs130_sdk` checkout.

## Build and run

The package sits in the colcon workspace at `ros2/`, next to the SDK it links:

    cd ros2
    source /opt/ros/humble/setup.bash && source /opt/tros/humble/setup.bash
    colcon build --packages-select gs130_camera
    source install/setup.bash
    ros2 launch gs130_camera gs130.launch.py

## Topics

With `stitch` not `none`, both eyes arrive in one frame:

| Topic | Type | Notes |
|---|---|---|
| `image_combine` | `sensor_msgs/Image` | Both eyes, `nv12`, stitched. |
| `image_combine/left/camera_info` | `sensor_msgs/CameraInfo` | One eye's size, not the frame's. |
| `image_combine/right/camera_info` | `sensor_msgs/CameraInfo` | Carries the baseline in `P[3]`; this is the one `hobot_stereonet` reads. |

With `stitch: none`, each eye gets its own frame and its own info:

| Topic | Type | Notes |
|---|---|---|
| `image_left` | `sensor_msgs/Image` | Left eye, `nv12`. |
| `image_left/camera_info` | `sensor_msgs/CameraInfo` | Left intrinsics. |
| `image_right` | `sensor_msgs/Image` | Right eye, `nv12`. |
| `image_right/camera_info` | `sensor_msgs/CameraInfo` | Carries the baseline in `P[3]`. |

Either way:

| Topic | Type | Notes |
|---|---|---|
| `/imu_data` | `sensor_msgs/Imu` | Frame `imu_link`. Absent when the device has no IMU. |
| `/tf_static` | `tf2_msgs/TFMessage` | `camera_link` to `camera_right_link`, plus `imu_link` when there is an IMU. |

With `publish_gray`, each image also goes out as `mono8` (the Y plane of the
NV12 frame) on `<image topic>/gray`.

`/tf_static` is a ROS reserved topic and is not a parameter; remap it if it has
to move.

Image QoS depth 5 and IMU depth 10, matching `hobot_mipi_cam`. `camera_info`
goes out immediately before the image, with the same stamp.

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `device` | `GS130WI` | Camera model: `GS130WI` or `GS130W`. |
| `camera_mode` | `rect` | `raw`, `resize`, or `rect`. |
| `output_width` | `640` | Output width of one eye. |
| `output_height` | `350` | Output height of one eye. |
| `fps` | `30` | Camera frames per second. |
| `odr` | `200` | IMU output data rate in Hz. |
| `publish_gray` | `false` | Enable mono8 grayscale image output. |
| `stitch` | `none` | `none`, `left_right`, `right_left`, `top_bottom`, `bottom_top`. `none` publishes each eye as its own frame; the rest publish one stitched frame. |
| `image_topic` | `image_combine` | Stitched frame topic, when `stitch` is not `none`. |
| `left_image_topic` | `image_left` | Left eye topic, when `stitch` is `none`. |
| `right_image_topic` | `image_right` | Right eye topic, when `stitch` is `none`. |
| `imu_topic` | `/imu_data` | IMU topic. |
| `timer_period_ms` | `1` | Period of the single publish timer, in milliseconds. |
| `frame_id` | `camera_link` | Frame of the left eye and of the stitched frame. |
| `right_frame_id` | `camera_right_link` | Frame of the right eye. |
| `imu_frame_id` | `imu_link` | Frame the IMU samples are stamped with. |

The `camera_info` topics are not parameters: each one is derived from the image
topic it describes, the way `mipi_cam` names them. A stitched frame publishes
`<image_topic>/left/camera_info` and `<image_topic>/right/camera_info`; a
per-eye frame publishes `<left_image_topic>/camera_info` and
`<right_image_topic>/camera_info`.

Only `top_bottom` feeds `hobot_stereonet`: its component checks the top half is the left eye and has no orientation setting, so a horizontal stitch cannot be used with it.

The node passes these values to `GS130_CONFIG(device, mode, width, height, fps, odr)` from `gs130_define.h`, which takes the platform from `gs130_platform()` -- the one `libgs130.so` was built for, so there is no parameter for it. The macro itself is C-only because it uses GNU C99 range initializers, so `src/config.c` expands it and returns the resulting `gs130_config_t` to the C++ node.

## Why 640x350

`hobot_stereonet`'s depth models take a 640x352 input, and it rescales whatever
it is given to that shape without preserving aspect -- a 1088x1280 pair became
640x352 by scaling x by 0.588 and y by 0.275, stretching the picture 2.1x
across. It also rescales the intrinsics by those same factors and then rescales
them again, which cost a factor of 1.7 in every depth it reported: against an
independent match of the same frame its depth came out at 0.607x the geometric
value.

640x352 exactly is what avoids all of it, and the SDK will not produce it. Its
aspect-preserving crop demands an exact integer ratio (`vse.c`
`roi_ratio_exact`), which for a 640-wide output means the height has to be a
multiple of ten; 352 is not, and `gs130_init` returns `GS130_UNSUPPORTED`.

640x350 is the closest height it accepts at the model's own width, and the
width is what the horizontal rescale is computed from, so with the width
matching the factor is 1.0 and `fx` comes through untouched:

    => sub rectified  [fx, fy, cx, cy, ...] : [362.067018, 364.135972, 320.0, 176.3]
    => after resize   [fx, fy, cx, cy, ...] : [362.067018, 366.216749, 320.0, 177.4]
                                               ^^^^^^^^^^ unchanged

Only `fy` moves, by 0.57%, from the two-pixel height difference. So the SDK does
the rectification on its GDC hardware, the node publishes 640x350 at 30 Hz, and
stereonet's stock launch file reads it as it is:

    ros2 launch gs130_camera gs130.launch.py &
    ros2 launch hobot_stereonet stereonet_model_no_web.launch.py

Measured that way: 15.0 fps through stereonet, 140-160 ms latency, and depth at
1.031x the geometric value over 165 confident matches.

`image_width:=1088 image_height:=1280` gives the whole sensor field of view for
anything that does not want the model's shape; stereonet will stretch it.

## Timestamps

`system` (the default) stamps each message with the receive time. `device`
passes the SDK's timestamp through instead, which is the frame's own age, but
on an RDK X5 the camera clock counts nanoseconds since boot rather than since
the epoch, so those stamps are unusable anywhere else. Neither invents a time:
the node never calls `this->now()` to make one up.

## Not implemented

Zero-copy (`hbm_img_msgs` over shared memory), custom messages, IMU filtering,
online reconfiguration, multi-device synchronisation, and any image processing.
A `stereo_layout` of `none` publishes only the left eye.
