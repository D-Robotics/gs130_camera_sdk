"""Mapping from gs130 calibration values to standard ROS messages.

Pure functions: no rclpy node, no hardware, so they are testable on a host.
"""

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo

import gs130


def distortion_model(dist_coeffs, dist_model):
    """The ROS distortion model string and coefficients for these values.

    An already rectified camera reports zeros, and ROS consumers want
    plumb_bob with five zeros in that case. The SDK's fisheye model carries
    the four equidistant coefficients, and its pinhole model carries the
    eight rational polynomial ones.
    """
    values = [float(value) for value in dist_coeffs]
    if not any(values):
        return "plumb_bob", [0.0] * 5
    if dist_model == gs130.DistModel.FISHEYE:
        return "equidistant", values[:4]
    return "rational_polynomial", values


def matrix(values):
    """A 3x3 numpy array as the flat nine-element list ROS expects."""
    return [float(value) for value in values.reshape(-1)]


def camera_info(intrinsics, width, height, frame_id):
    """Build a sensor_msgs/CameraInfo from a gs130 CameraIntrinsics.

    K and D describe one camera at the given output resolution. R stays the
    identity and P restates K, because the SDK exposes no rectification
    matrix; both are documented rather than invented.
    """
    model, coefficients = distortion_model(intrinsics.dist_coeffs, intrinsics.dist_model)
    message = CameraInfo()
    message.header.frame_id = frame_id
    message.width = int(width)
    message.height = int(height)
    message.distortion_model = model
    message.k = matrix(intrinsics.K)
    message.d = coefficients
    message.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    message.p = [
        intrinsics.fx, 0.0, intrinsics.cx, 0.0,
        0.0, intrinsics.fy, intrinsics.cy, 0.0,
        0.0, 0.0, 1.0, 0.0,
    ]
    return message


def transform(device, from_frame, to_frame, parent_frame, child_frame):
    """The static transform that places child_frame in the parent parent_frame.

    gs130 relative_R/T(from, to) describes from_frame as seen in to_frame,
    which is exactly a parent/child transform with to_frame as the parent.
    """
    rotation = device.relative_R(from_frame, to_frame)
    translation = device.relative_T(from_frame, to_frame)
    message = TransformStamped()
    message.header.frame_id = parent_frame
    message.child_frame_id = child_frame
    message.transform.translation.x = float(translation[0])
    message.transform.translation.y = float(translation[1])
    message.transform.translation.z = float(translation[2])
    x, y, z, w = _quaternion(rotation)
    message.transform.rotation.x = x
    message.transform.rotation.y = y
    message.transform.rotation.z = z
    message.transform.rotation.w = w
    return message


def _quaternion(rotation):
    """The quaternion of a 3x3 rotation matrix (Shepperd's method)."""
    m = rotation
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        root = (trace + 1.0) ** 0.5
        w = 0.5 * root
        scale = 0.5 / root
        return ((m[2][1] - m[1][2]) * scale,
                (m[0][2] - m[2][0]) * scale,
                (m[1][0] - m[0][1]) * scale,
                w)
    index = max(range(3), key=lambda i: m[i][i])
    if index == 0:
        root = (1.0 + m[0][0] - m[1][1] - m[2][2]) ** 0.5
        x = 0.5 * root
        scale = 0.5 / root
        return (x,
                (m[0][1] + m[1][0]) * scale,
                (m[0][2] + m[2][0]) * scale,
                (m[2][1] - m[1][2]) * scale)
    if index == 1:
        root = (1.0 + m[1][1] - m[0][0] - m[2][2]) ** 0.5
        y = 0.5 * root
        scale = 0.5 / root
        return ((m[0][1] + m[1][0]) * scale,
                y,
                (m[1][2] + m[2][1]) * scale,
                (m[0][2] - m[2][0]) * scale)
    root = (1.0 + m[2][2] - m[0][0] - m[1][1]) ** 0.5
    z = 0.5 * root
    scale = 0.5 / root
    return ((m[0][2] + m[2][0]) * scale,
            (m[1][2] + m[2][1]) * scale,
            z,
            (m[1][0] - m[0][1]) * scale)
