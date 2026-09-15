"""Bring up the GS130 camera and show it in the D-Robotics TROS web UI.

    ros2 launch gs130_ros gs130_web.launch.py

Then open http://<board-ip>:8000/ in a browser. The image chain reuses the
existing TROS nodes: this node publishes NV12 on /image_combine_raw,
hobot_codec encodes it to /image_combine_jpeg, and websocket serves it.
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gs130_arguments import CAMERA_ARGUMENTS, camera_parameters  # noqa: E402

WEB_ARGUMENTS = [
    ("image_topic", "/image_combine_raw", str, "NV12 topic the camera publishes"),
    ("jpeg_topic", "/image_combine_jpeg", str, "jpeg topic the web UI subscribes to"),
    ("jpg_quality", "85.0", str, "jpeg quality passed to hobot_codec"),
    ("websocket_channel", "0", str, "web channel to serve on"),
]


def generate_launch_description():
    camera_declarations = [
        DeclareLaunchArgument(name, default_value=default, description=text)
        for name, default, _, text in CAMERA_ARGUMENTS
    ]
    web_declarations = [
        DeclareLaunchArgument(name, default_value=default, description=text)
        for name, default, _, text in WEB_ARGUMENTS
    ]

    camera = Node(
        package="gs130_ros",
        executable="camera_node",
        name="gs130_camera",
        output="screen",
        parameters=[camera_parameters()],
    )

    # existing TROS node: NV12 -> jpeg
    codec = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("hobot_codec"),
            "launch/hobot_codec_encode.launch.py")),
        launch_arguments={
            "codec_in_mode": "ros",
            "codec_in_format": "nv12",
            "codec_out_mode": "ros",
            "codec_out_format": "jpeg",
            "codec_sub_topic": LaunchConfiguration("image_topic"),
            "codec_pub_topic": LaunchConfiguration("jpeg_topic"),
            "codec_jpg_quality": LaunchConfiguration("jpg_quality"),
        }.items(),
    )

    # existing TROS node: web UI on port 8000
    web = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("websocket"),
            "launch/websocket.launch.py")),
        launch_arguments={
            "websocket_image_topic": LaunchConfiguration("jpeg_topic"),
            "websocket_channel": LaunchConfiguration("websocket_channel"),
            "websocket_only_show_image": "True",
        }.items(),
    )

    return LaunchDescription(
        camera_declarations + web_declarations + [camera, codec, web]
    )
