"""Bring up the GS130 camera and IMU as ROS topics.

    ros2 launch gs130_ros gs130_camera.launch.py

No web chain here: this is for applications that consume the topics.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from gs130_ros.launch_arguments import CAMERA_ARGUMENTS, camera_parameters


def generate_launch_description():
    declarations = [
        DeclareLaunchArgument(name, default_value=default, description=text)
        for name, default, _, text in CAMERA_ARGUMENTS
    ]
    camera = Node(
        package="gs130_ros",
        executable="camera_node",
        name="gs130_camera",
        output="screen",
        parameters=[camera_parameters()],
    )
    return LaunchDescription(declarations + [camera])
