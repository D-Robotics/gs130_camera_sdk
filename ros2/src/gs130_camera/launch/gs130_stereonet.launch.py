"""Start the camera, the stereo depth model, and the depth view on the web page.

    ros2 launch gs130_camera gs130_stereonet.launch.py
    ros2 launch gs130_camera gs130_stereonet.launch.py render_type:=distance

The camera comes from gs130.launch.py, included rather than described again, so
its arguments, defaults and descriptions are that file's and can be set here just
the same.  stitch is the one exception: hobot_stereonet takes both eyes in a
single frame with the left one on top, so this file pins that value and passing
another one has no effect.

The model named by stereonet_model is the pair to the camera's default 544x448
eye -- both numbers are the size of one eye, and the stitched frame is twice as
tall -- so the two have to be changed together.  The node reads its calibration
from the camera_info topics published next to the stitched image.

The depth view then reaches the browser the same way the raw image does in
gs130_websocket.launch.py: hobot_stereonet's own codec_web_visual.launch.py
encodes /StereoNetNode/stereonet_visual to jpeg and serves it on web channel 0,
with nginx started by the websocket launch file:

    http://<board>:8000
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


# name, default, description.  stereonet_model is a file under the package's
# config directory, and its two numbers are the size of one eye, which is why the
# default here matches the default output_width and output_height of the camera.
ARGUMENTS = (
    (
        "stereonet_model",
        "DStereoV2.4_int8_544_448.bin",
        "Model file under hobot_stereonet/config, sized for one eye",
    ),
    (
        "render_type",
        "indoor",
        "Depth colouring (indoor, outdoor, indoor-reverse, outdoor-reverse, "
        "distance, distance-reverse)",
    ),
)


def _stereonet(context):
    """Point the model at the camera's topics, which are only known by now."""
    image_topic = LaunchConfiguration("image_topic").perform(context)

    model = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("hobot_stereonet"),
                "launch",
                "stereonet_model_no_web.launch.py",
            )
        ),
        launch_arguments={
            # The camera is this node, not mipi_cam.
            "use_mipi_cam": "False",
            # Passed as the substitutions rather than the values they hold now, so
            # that a command line of this launch file still decides them.
            "stereonet_model_file_path": PathJoinSubstitution(
                [
                    get_package_share_directory("hobot_stereonet"),
                    "config",
                    LaunchConfiguration("stereonet_model"),
                ]
            ),
            "render_type": LaunchConfiguration("render_type"),
            # The stitched frame, plus the two camera_info topics published beside
            # it: with calib_method left at none, those are its calibration.
            "stereo_image_topic": image_topic,
            "camera_info_topic": image_topic + "/right/camera_info",
            "left_camera_info_topic": image_topic + "/left/camera_info",
            # Defaults here are camera_link and camera_link_right; ours differ on
            # the right eye, and a frame the node does not publish is no use in tf.
            "stereonet_frame_id": LaunchConfiguration("frame_id").perform(context),
            "stereonet_frame_id_right": LaunchConfiguration("right_frame_id").perform(context),
        }.items(),
    )

    # Starts the codec and the websocket that put the depth view on channel 0.
    view = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("hobot_stereonet"),
                "launch",
                "codec_web_visual.launch.py",
            )
        )
    )

    return [model, view]


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, description in ARGUMENTS
    ]

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gs130_camera"),
                "launch",
                "gs130.launch.py",
            )
        ),
        # A literal, not LaunchConfiguration('stitch'): stereonet's layout is a
        # requirement rather than a preference, so this is what pins it.
        launch_arguments={"stitch": "top_bottom"}.items(),
    )

    # The camera include declares the rest of the arguments and has to run first:
    # _stereonet reads them back out of the context it fills in.
    return LaunchDescription(arguments + [camera, OpaqueFunction(function=_stereonet)])
