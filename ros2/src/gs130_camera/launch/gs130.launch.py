"""Start the GS130 stereo camera node.

    ros2 launch gs130_camera gs130.launch.py
    ros2 launch gs130_camera gs130.launch.py stitch:=top_bottom publish_gray:=true

Every launch argument is passed straight through to the node as a parameter of
the same name, so the list below is the only place argument defaults live.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


# name, default, type, description.  A launch argument is always a string, and the
# type is what tells launch_ros to hand the node an integer or a boolean instead.
# The order matches declare_parameters() in src/gs130_node.cpp.
ARGUMENTS = (
    ("device", "GS130WI", str, "Camera model (GS130WI or GS130W)"),
    ("camera_mode", "rect", str, "Camera mode (raw, rect, or resize)"),
    (
        "stitch",
        "none",
        str,
        "Both eyes in one frame (none, left_right, right_left, top_bottom, bottom_top)",
    ),
    ("image_topic", "image_combine", str, "Stitched frame topic, used when stitch is not none"),
    ("left_image_topic", "image_left", str, "Left eye topic, used when stitch is none"),
    ("right_image_topic", "image_right", str, "Right eye topic, used when stitch is none"),
    ("imu_topic", "/imu_data", str, "IMU topic, used when the device has an IMU"),
    ("output_width", "544", int, "Output width of one eye"),
    ("output_height", "448", int, "Output height of one eye"),
    ("fps", "30", int, "Camera frames per second"),
    ("odr", "200", int, "IMU output data rate, in Hz"),
    (
        "timer_period_ms",
        "1",
        int,
        "Publish period in milliseconds; one message goes out per period, so this "
        "also caps the combined stream rate and has to stay well under it",
    ),
    ("publish_gray", "false", bool, "Also publish each image as mono8"),
    ("frame_id", "camera_link", str, "Frame of the left eye and of the stitched frame"),
    ("right_frame_id", "camera_right_link", str, "Frame of the right eye"),
    ("imu_frame_id", "imu_link", str, "Frame the IMU samples are stamped with"),
)


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, _, description in ARGUMENTS
    ]

    # Without value_type every one of these would reach the node as a string, and
    # declaring "640" as an integer parameter is not something the node can do.
    parameters = {
        name: ParameterValue(LaunchConfiguration(name), value_type=value_type)
        for name, _, value_type, _ in ARGUMENTS
    }

    node = Node(
        package="gs130_camera",
        executable="gs130_node",
        name="gs130_camera",
        parameters=[parameters],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription(arguments + [node])
