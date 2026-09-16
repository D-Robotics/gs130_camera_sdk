"""Start the camera and show it on the web page.

    ros2 launch gs130_camera gs130_websocket.launch.py
    ros2 launch gs130_camera gs130_websocket.launch.py stitch:=top_bottom

The camera comes from gs130.launch.py, included rather than described again,
so this file cannot drift from it: the arguments, their defaults and their
descriptions are that file's, and ``--show-args`` lists them here too.

What it adds is the pair of TROS nodes that carry the picture to a browser.
hobot_codec encodes nv12 as jpeg on an ordinary ROS topic, websocket serves
that topic over its socket, and the websocket launch file starts the nginx
that hands out the page:

    http://<board>:8000

With stitch set, one frame holds both eyes and that frame is channel 0.  With
stitch none there is no such frame, so the left eye is channel 0 and the right
eye is channel 1, each with its own encoder and its own websocket.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


# Two encoders run at once when stitch is none, and hobot_codec's channel is
# the processing channel number, of which there are four, so they take 0 and 1.
CHANNEL_LEFT = 0
CHANNEL_RIGHT = 1

# The node's own default, and enough for a preview.
JPEG_QUALITY = 80.0


def _encode(sub_topic, pub_topic, name, channel):
    """One hobot_codec turning nv12 into jpeg.

    in_mode is stated as ros because the TROS launch file for this node
    overrides the node's own default with shared_mem, which would have it wait
    on hbmem_img.  Nothing here publishes hbmem_img: the camera publishes
    sensor_msgs/Image.
    """
    return Node(
        package="hobot_codec",
        executable="hobot_codec_republish",
        name=name,
        parameters=[
            {
                "channel": channel,
                "in_mode": "ros",
                "in_format": "nv12",
                "out_mode": "ros",
                "out_format": "jpeg",
                "sub_topic": sub_topic,
                "pub_topic": pub_topic,
                "jpg_quality": JPEG_QUALITY,
                "input_framerate": ParameterValue(
                    LaunchConfiguration("fps"), value_type=int
                ),
            }
        ],
        output="screen",
    )


def _websocket(image_topic, channel):
    """A websocket on its own channel, through the package's launch file.

    That file is what starts nginx, and it skips the start when nginx is
    already up, so including it is also how the page gets served.
    """
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("websocket"),
                "launch",
                "websocket.launch.py",
            )
        ),
        launch_arguments={
            "websocket_image_topic": image_topic,
            "websocket_image_type": "mjpeg",
            "websocket_channel": str(channel),
            # Nothing here publishes the AI detection topic this node otherwise
            # subscribes to, and it complains about that every few seconds.
            "websocket_only_show_image": "True",
        }.items(),
    )


def _second_websocket(image_topic, channel):
    """The other eye's channel, as a plain node.

    websocket.launch.py fixes no node name, so including it a second time
    would leave two nodes both called websocket.  This one is named.  nginx is
    not its business: the include above has already started it.
    """
    return Node(
        package="websocket",
        executable="websocket",
        name="websocket_right",
        parameters=[
            {
                "image_topic": image_topic,
                "image_type": "mjpeg",
                # See the note on the include: no AI detection topic is published here.
                "only_show_image": True,
                "output_fps": 0,
                "smart_topic": "/hobot_mono2d_body_detection",
                "channel": channel,
            }
        ],
        output="screen",
    )


def _web(context):
    """Build the web half once stitch is known.

    Which topics exist is a property of the resolved stitch value, and the
    number of encoders and websockets follows from it, so none of this can be
    decided while the launch description is being put together.
    """
    if LaunchConfiguration("stitch").perform(context) != "none":
        image = LaunchConfiguration("image_topic").perform(context)
        return [
            _encode(image, "/image_jpeg", "hobot_codec_web", CHANNEL_LEFT),
            _websocket("/image_jpeg", CHANNEL_LEFT),
        ]

    left = LaunchConfiguration("left_image_topic").perform(context)
    right = LaunchConfiguration("right_image_topic").perform(context)
    return [
        _encode(left, "/image_jpeg_left", "hobot_codec_left", CHANNEL_LEFT),
        _encode(right, "/image_jpeg_right", "hobot_codec_right", CHANNEL_RIGHT),
        _websocket("/image_jpeg_left", CHANNEL_LEFT),
        _second_websocket("/image_jpeg_right", CHANNEL_RIGHT),
    ]


def generate_launch_description():
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gs130_camera"),
                "launch",
                "gs130.launch.py",
            )
        )
    )

    # The include has to come first: it is what declares stitch and the rest,
    # and _web reads them out of the context it fills in.
    return LaunchDescription([camera, OpaqueFunction(function=_web)])
