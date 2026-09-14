"""Launch arguments shared by the camera and web launch files."""

from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

# name, default, python type, description
CAMERA_ARGUMENTS = [
    ("platform", "RDKX5", str, "SDK platform name"),
    ("device", "GS130WI", str, "SDK device name: GS130WI with IMU, GS130W without"),
    ("mode", "resize", str, "camera mode: raw, resize or rect"),
    ("width", "640", int, "output width per eye"),
    ("height", "480", int, "output height per eye"),
    ("fps", "30", int, "camera frame rate"),
    ("odr", "200", int, "IMU output data rate"),
    ("stereo_layout", "left_right", str,
     "none publishes two eyes, a layout publishes one stitched frame"),
    ("frame_id_camera", "camera_left", str, "frame_id of the left eye and the TF parent"),
    ("frame_id_imu", "imu_link", str, "frame_id of the IMU"),
    ("publish_imu", "true", bool, "publish /imu/data"),
    ("publish_tf", "true", bool, "publish the extrinsics as a static transform"),
    ("start_timeout_s", "10.0", float, "seconds to wait for the first frame"),
]


def camera_parameters(arguments=CAMERA_ARGUMENTS):
    """Declared launch arguments as typed node parameters.

    The type matters: a launch argument is text, and rclpy refuses a string
    override for an integer parameter, so conversion happens here, where a bad
    value fails with a message that names the argument.
    """
    return {
        name: ParameterValue(LaunchConfiguration(name), value_type=kind)
        for name, _, kind, _ in arguments
    }
