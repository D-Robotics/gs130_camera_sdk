"""Conversions from gs130 values into ROS messages.

Every function here is pure: one gs130 value in, one message out, no node and
no clock.  ``node`` decides *what* to publish and *when*; this module only
decides what it looks like.

The camera publishes NV12 because that is the only format the C API produces.
``sensor_msgs/Image`` for NV12 uses ``step == width`` and ``height`` equal to
the luma height, so a frame's ``data`` is ``width * height * 3 // 2`` bytes:
``height`` luma rows followed by ``height // 2`` interleaved chroma rows.  A
stitched frame keeps that layout, with the two eyes packed inside it; see
``slice_eyes``.
"""

from __future__ import annotations

import array
import math

import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image, Imu

import gs130


# NV12 in ROS: single channel, one byte per pixel, so the row stride is the width.
NV12_ENCODING = "nv12"

_NS_PER_SEC = 1_000_000_000
_SEC_MAX = 2**31 - 1

_IDENTITY = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]


def to_time(timestamp_ns):
    """Split a device nanosecond timestamp into a ``builtin_interfaces/Time``.

    Raises :class:`ValueError` when the value cannot be represented, rather
    than wrapping around and silently producing a frame from 1970 or 2038.
    """
    seconds, nanoseconds = divmod(int(timestamp_ns), _NS_PER_SEC)
    if seconds > _SEC_MAX:
        raise ValueError(
            "timestamp %d ns is outside the ROS time range" % timestamp_ns
        )
    return Time(sec=seconds, nanosec=nanoseconds)


def nv12_message(data, width, height, frame_id, stamp):
    """Build the ``sensor_msgs/Image`` for one NV12 buffer.

    ``data`` is anything holding ``width * height * 3 // 2`` bytes: a
    :class:`gs130.Image`, a slice of one, or a plain array.  It is copied, so
    the caller may drop the source straight afterwards -- the SDK frees its own
    buffer once the last reference to it goes away.
    """
    message = Image()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.height = int(height)
    message.width = int(width)
    message.encoding = NV12_ENCODING
    message.is_bigendian = 0
    message.step = int(width)
    # An array of bytes rather than a bytes object.  Both are valid values for
    # a uint8[] field, but rclpy fills the field one element at a time when it
    # is handed a bytes, which costs 3.3 seconds for a 1088x2560 NV12 frame on
    # an X5 -- and the camera then runs at 0.2 fps.  array.frombytes copies the
    # buffer in one step: 0.8 ms for the same frame.
    payload = array.array("B")
    payload.frombytes(memoryview(np.ascontiguousarray(data)))
    message.data = payload
    return message


def _distortion(intrinsics, rectified):
    """Return ``(distortion_model, coefficients)`` for a camera.

    Rectification zeroes the coefficients in the C layer, so a rectified frame
    is described exactly by ``plumb_bob`` with nothing to correct.  Otherwise
    the model decides the length: the pinhole table holds the rational
    polynomial's eight terms, the fisheye table the equidistant model's four.
    """
    coefficients = np.asarray(intrinsics.dist_coeffs, dtype=float).reshape(-1)
    if rectified:
        return "plumb_bob", [0.0] * 5
    if intrinsics.dist_model == gs130.DistModel.FISHEYE:
        return "equidistant", [float(value) for value in coefficients[:4]]
    return "rational_polynomial", [float(value) for value in coefficients[:8]]


def projection(intrinsics):
    """The default ``P`` of an eye: its own ``K`` with a zero translation.

    Used for every unrectified mode, where there is no virtual parallel pair
    and therefore no baseline to report.
    """
    return [
        float(intrinsics.fx), 0.0, float(intrinsics.cx), 0.0,
        0.0, float(intrinsics.fy), float(intrinsics.cy), 0.0,
        0.0, 0.0, 1.0, 0.0,
    ]


def stereo_projection(intrinsics, translation):
    """``P`` of a rectified eye from its position in the rectified left frame.

    ``translation`` is that eye's origin in the left camera's frame, so
    ``P = K * [I | t]`` and the two entries downstream consumers read are
    ``P[3] = fx * tx + cx * tz`` (the focal length times the baseline, which is
    all a stereo matcher needs) and ``P[7] = fy * ty + cy * tz``.

    The left eye passes ``(0, 0, 0)`` by definition -- it *is* the frame the
    right eye is measured in -- and gets ``P[3] == 0``, which is the convention
    ``stereoRectify`` and the official dual-camera calibration both produce.
    """
    tx, ty, tz = (float(value) for value in np.asarray(translation).reshape(-1)[:3])
    fx, fy = float(intrinsics.fx), float(intrinsics.fy)
    cx, cy = float(intrinsics.cx), float(intrinsics.cy)
    return [
        fx, 0.0, cx, fx * tx + cx * tz,
        0.0, fy, cy, fy * ty + cy * tz,
        0.0, 0.0, 1.0, 0.0,
    ]


def camera_info(intrinsics, width, height, frame_id, stamp,
                projection_matrix=None, rectified=False):
    """Build the ``sensor_msgs/CameraInfo`` for one eye.

    ``width`` and ``height`` describe **one eye**, which is what the official
    calibration files declare and what downstream consumers scale against; on a
    stitched topic that is half of the image message's size, not all of it.

    ``R`` is the identity: ``CameraInfo.R`` is the rotation that takes a point
    into the rectified frame, and a frame this node publishes is either already
    rectified or not rectified at all.  The device extrinsics are delivered
    through ``P`` and TF instead, never here.
    """
    message = CameraInfo()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.width = int(width)
    message.height = int(height)
    message.distortion_model, message.d = _distortion(intrinsics, rectified)
    message.k = [float(value) for value in np.asarray(intrinsics.K).reshape(-1)[:9]]
    message.r = list(_IDENTITY)
    message.p = (
        projection(intrinsics)
        if projection_matrix is None
        else [float(value) for value in projection_matrix]
    )
    return message


def imu_message(packet, frame_id, stamp):
    """Build the ``sensor_msgs/Imu`` for one sample.

    The orientation is not estimated, so it is marked absent the way
    ``sensor_msgs/Imu`` prescribes: the first covariance entry set to -1.  The
    acceleration and rate covariances stay at zero, meaning "unknown"; the
    noise densities the EEPROM holds are continuous-time densities, and turning
    one into a per-sample variance needs a bandwidth this node does not have.
    """
    message = Imu()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.linear_acceleration.x = float(packet.accel[0])
    message.linear_acceleration.y = float(packet.accel[1])
    message.linear_acceleration.z = float(packet.accel[2])
    message.angular_velocity.x = float(packet.gyro[0])
    message.angular_velocity.y = float(packet.gyro[1])
    message.angular_velocity.z = float(packet.gyro[2])
    message.orientation_covariance[0] = -1.0
    return message


def quaternion(rotation):
    """Rotation matrix to ``(x, y, z, w)``, normalised.

    Uses the largest diagonal term to pick a stable branch, so rotations near
    180 degrees do not divide by a vanishing scale factor.
    """
    m = np.asarray(rotation, dtype=float).reshape(3, 3)
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        x = (m[2, 1] - m[1, 2]) / scale
        y = (m[0, 2] - m[2, 0]) / scale
        z = (m[1, 0] - m[0, 1]) / scale
        w = 0.25 * scale
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        scale = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        x = 0.25 * scale
        y = (m[0, 1] + m[1, 0]) / scale
        z = (m[0, 2] + m[2, 0]) / scale
        w = (m[2, 1] - m[1, 2]) / scale
    elif m[1, 1] > m[2, 2]:
        scale = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        x = (m[0, 1] + m[1, 0]) / scale
        y = 0.25 * scale
        z = (m[1, 2] + m[2, 1]) / scale
        w = (m[0, 2] - m[2, 0]) / scale
    else:
        scale = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        x = (m[0, 2] + m[2, 0]) / scale
        y = (m[1, 2] + m[2, 1]) / scale
        z = 0.25 * scale
        w = (m[1, 0] - m[0, 1]) / scale

    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm == 0.0:
        raise ValueError("rotation is not a usable rotation matrix")
    return x / norm, y / norm, z / norm, w / norm


def transform_message(rotation, translation, parent, child, stamp):
    """Build a ``geometry_msgs/TransformStamped`` describing ``child`` in ``parent``.

    ``rotation`` and ``translation`` are the pair :meth:`gs130.Device.relative_R`
    and :meth:`gs130.Device.relative_T` produce, which already mean "take a
    point in ``from`` into ``to``" -- the direction TF wants for a transform
    from ``to`` to ``from``.  The values are passed through untouched: the
    device's axis convention has not been checked against REP-103, and
    rewriting them here would be a guess that silently reorients every consumer.
    """
    message = TransformStamped()
    message.header.stamp = stamp
    message.header.frame_id = parent
    message.child_frame_id = child
    x, y, z, w = quaternion(rotation)
    message.transform.translation.x = float(translation[0])
    message.transform.translation.y = float(translation[1])
    message.transform.translation.z = float(translation[2])
    message.transform.rotation.x = x
    message.transform.rotation.y = y
    message.transform.rotation.z = z
    message.transform.rotation.w = w
    return message


def eye_size(layout, width, height):
    """The size of **one** eye inside a frame packed with ``layout``."""
    if layout in (gs130.StereoLayout.TOP_BOTTOM, gs130.StereoLayout.BOTTOM_TOP):
        half = height // 2
        return width, half
    if layout in (gs130.StereoLayout.LEFT_RIGHT, gs130.StereoLayout.RIGHT_LEFT):
        half = width // 2
        return half, height
    return width, height


def slice_eyes(frame, layout):
    """Split a stitched NV12 frame into ``(left, right)`` NV12 buffers.

    Each half comes back as its own contiguous ``(eye_height * 3 // 2, eye_width)``
    array, independent of ``frame``, so it stays valid after ``frame`` is
    dropped.

    The buffer is the one ``gs130.cpp:262-301`` lays out, and it is also the one
    ``hobot_stereonet`` reads back (``stereonet_component.cpp:898-905``): the two
    planes are separate buffers, not one image appended to another.  Stacked,
    that means both eyes' luma first, ``[0, w*h)`` and ``[w*h, 2*w*h)``, then
    their chroma at ``2*w*h`` and ``2*w*h + w*h/2`` -- so the split is by whole
    bands.  Side by side the eyes share every row at a stride of ``2*w``, so it
    is a strided take of the luma band and of the chroma band.
    """
    luma_height = frame.height
    chroma_height = luma_height // 2
    stacked = layout in (gs130.StereoLayout.TOP_BOTTOM, gs130.StereoLayout.BOTTOM_TOP)
    first_is_left = layout in (gs130.StereoLayout.TOP_BOTTOM, gs130.StereoLayout.LEFT_RIGHT)

    if stacked:
        split = luma_height // 2
        chroma_split = luma_height + chroma_height // 2
        chroma_end = chroma_split + chroma_height // 2
        halves = [
            (frame[0:split], frame[luma_height:chroma_split]),
            (frame[split:luma_height], frame[chroma_split:chroma_end]),
        ]
    else:
        split = frame.width // 2
        chroma = slice(luma_height, luma_height + chroma_height)
        halves = [
            (frame[0:luma_height, 0:split], frame[chroma, 0:split]),
            (frame[0:luma_height, split:], frame[chroma, split:]),
        ]

    if not first_is_left:
        halves.reverse()

    return tuple(
        np.concatenate(
            [
                np.ascontiguousarray(luma).reshape(-1),
                np.ascontiguousarray(chroma).reshape(-1),
            ]
        )
        for luma, chroma in halves
    )
