"""Start the GS130 camera node.

Every hardware and interface choice is an argument here, so one file covers a
board and a camera model without a rebuild::

    ros2 launch gs130_ros gs130.launch.py
    ros2 launch gs130_ros gs130.launch.py device:=GS130W camera_mode:=raw
    ros2 launch gs130_ros gs130.launch.py stereo_layout:=none framerate:=15

The defaults are the ones that feed D-Robotics' official depth pipeline: the
rectified pair, stacked with the left eye on top, on ``image_combine_raw``.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Every argument is declared with a default and forwarded by name, so the
# launch file and the node's parameter list are read side by side.
ARGUMENTS = (
    ("platform", "RDKX5", "Board the camera is attached to (RDKX5)"),
    ("device", "GS130WI", "Camera model (GS130WI, GS130W)"),
    ("camera_mode", "rect", "ISP path (raw, resize, rect)"),
    (
        "image_width",
        "640",
        "Output width of one eye; 640 is the width the depth models take",
    ),
    (
        "image_height",
        "350",
        "Output height of one eye; 350 is the closest to the models' 352 the "
        "SDK accepts at that width",
    ),
    ("framerate", "30", "Frames per second"),
    ("imu_odr", "200", "IMU output data rate in Hz"),
    (
        "stereo_layout",
        "top_bottom",
        "How the two eyes are packed (none, top_bottom, bottom_top, left_right, right_left)",
    ),
    ("frame_id", "camera_link", "Frame the images are stamped with"),
    ("imu_frame_id", "imu_link", "Frame the IMU samples are stamped with"),
    ("publish_imu", "true", "Publish the IMU when the device has one"),
    (
        "publish_left_right",
        "false",
        "Also publish the two eyes separately when the frames arrive stitched",
    ),
    (
        "only_when_subscribed",
        "false",
        "Skip capturing frames while nothing is subscribed",
    ),
    (
        "timestamp_source",
        "auto",
        "Where stamps come from (auto, device, system)",
    ),
    ("topic_combine", "image_combine_raw", "Stitched top/bottom image topic"),
    ("topic_stereo", "image_stereo_raw", "Stitched image topic for other layouts"),
    ("topic_left", "image_left_raw", "Left eye image topic"),
    ("topic_right", "image_right_raw", "Right eye image topic"),
    ("topic_imu", "/imu_data", "IMU topic, absolute like the official node's"),
    (
        "topic_imu_extrinsic",
        "/imu_extrinsic",
        "Extrinsic topic, absolute like the official node's",
    ),
)


def generate_launch_description():
    parameters = {
        name: LaunchConfiguration(name) for name, _, _ in ARGUMENTS
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(name, default_value=default, description=description)
            for name, default, description in ARGUMENTS
        ]
        + [
            Node(
                package="gs130_ros",
                executable="gs130_node",
                name="gs130_ros",
                parameters=[parameters],
                output="screen",
                emulate_tty=True,
            )
        ]
    )
