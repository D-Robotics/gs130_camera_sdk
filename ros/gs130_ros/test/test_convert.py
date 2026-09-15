"""Tests for the conversions from gs130 values to ROS messages."""

import math

import numpy as np
import pytest
from builtin_interfaces.msg import Time

import gs130
from gs130_ros import convert


# ---------------------------------------------------------------------------
# A stand-in for gs130.Image: the same shape and the same two properties, so
# the packing tests below exercise the real slicing arithmetic.
# ---------------------------------------------------------------------------


class _Frame(np.ndarray):
    def __new__(cls, array):
        return np.asarray(array, dtype=np.uint8).view(cls)

    @property
    def width(self):
        return self.shape[1]

    @property
    def height(self):
        return self.shape[0] * 2 // 3


class _Intrinsics:
    """The fields convert reads off a gs130.CameraIntrinsics."""

    def __init__(self, fx, fy, cx, cy, dist_model=gs130.DistModel.PINHOLE, dist=None):
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
        self.dist_coeffs = np.zeros(8) if dist is None else np.asarray(dist, dtype=float)
        self.dist_model = dist_model


class _Packet:
    def __init__(self, accel, gyro, timestamp_ns):
        self.accel = np.asarray(accel, dtype=float)
        self.gyro = np.asarray(gyro, dtype=float)
        self.temp = 25.0
        self.is_fsync = True
        self.timestamp_ns = timestamp_ns


def _rotation_from_quaternion(x, y, z, w):
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


# ---------------------------------------------------------------------------
# Timestamps
# ---------------------------------------------------------------------------


def test_to_time_splits_seconds_and_nanoseconds():
    moment = convert.to_time(1_500_000_002)
    assert (moment.sec, moment.nanosec) == (1, 500_000_002)
    assert (convert.to_time(0).sec, convert.to_time(0).nanosec) == (0, 0)


def test_to_time_keeps_nanoseconds_in_range_for_negative_values():
    # ROS allows a negative sec as long as nanosec stays in [0, 1e9).
    moment = convert.to_time(-1)
    assert (moment.sec, moment.nanosec) == (-1, 999_999_999)


def test_to_time_refuses_a_value_past_the_ros_range():
    with pytest.raises(ValueError):
        convert.to_time((2**31) * 1_000_000_000)


# ---------------------------------------------------------------------------
# Rotation
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("axis", [0, 1, 2])
@pytest.mark.parametrize("angle", [0.0, 30.0, 90.0, 179.0, 180.0, 270.0])
def test_quaternion_round_trips_through_its_matrix(axis, angle):
    radians = math.radians(angle)
    rotation = np.eye(3)
    rotation[(axis + 1) % 3, (axis + 1) % 3] = math.cos(radians)
    rotation[(axis + 2) % 3, (axis + 2) % 3] = math.cos(radians)
    rotation[(axis + 1) % 3, (axis + 2) % 3] = -math.sin(radians)
    rotation[(axis + 2) % 3, (axis + 1) % 3] = math.sin(radians)

    quaternion = convert.quaternion(rotation)
    assert math.isclose(sum(value * value for value in quaternion), 1.0, abs_tol=1e-12)
    # q and -q are the same rotation, so compare the matrices they build.
    assert np.allclose(_rotation_from_quaternion(*quaternion), rotation, atol=1e-9)


def test_quaternion_refuses_a_non_finite_rotation():
    with pytest.raises(ValueError):
        convert.quaternion(np.full((3, 3), np.nan))


# ---------------------------------------------------------------------------
# Projection
# ---------------------------------------------------------------------------


def test_projection_has_no_baseline_before_rectification():
    projection = convert.projection(_Intrinsics(600.0, 600.0, 544.0, 640.0))
    assert projection[3] == 0.0
    assert projection[7] == 0.0
    assert projection[11] == 0.0


def test_stereo_projection_puts_the_baseline_in_p3():
    intrinsics = _Intrinsics(600.0, 600.0, 544.0, 640.0)
    # The right eye sits a baseline to the left of the left eye's frame.
    projection = convert.stereo_projection(intrinsics, (-0.08, 0.0, 0.0))
    assert projection[3] == pytest.approx(600.0 * -0.08)
    assert projection[7] == 0.0
    assert projection[11] == 0.0
    # What a stereo matcher reads back out of it.
    assert abs(projection[3] / projection[0]) == pytest.approx(0.08)


def test_stereo_projection_accounts_for_an_off_axis_offset():
    intrinsics = _Intrinsics(600.0, 600.0, 544.0, 640.0)
    projection = convert.stereo_projection(intrinsics, (-0.08, 0.0, 0.002))
    assert projection[3] == pytest.approx(600.0 * -0.08 + 544.0 * 0.002)


def test_stereo_projection_of_the_left_eye_has_no_baseline():
    projection = convert.stereo_projection(
        _Intrinsics(600.0, 600.0, 544.0, 640.0), (0.0, 0.0, 0.0)
    )
    assert projection[3] == 0.0


# ---------------------------------------------------------------------------
# CameraInfo
# ---------------------------------------------------------------------------


def test_camera_info_declares_the_eye_size_and_an_identity_r():
    message = convert.camera_info(
        _Intrinsics(600.0, 601.0, 544.0, 640.0), 1088, 1280, "camera_link", Time()
    )
    assert (message.width, message.height) == (1088, 1280)
    assert list(message.k[:3]) == [600.0, 0.0, 544.0]
    assert list(message.r) == [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    assert len(message.p) == 12


def test_camera_info_carries_all_eight_pinhole_coefficients():
    intrinsics = _Intrinsics(600.0, 600.0, 544.0, 640.0, dist=np.arange(8, dtype=float))
    message = convert.camera_info(intrinsics, 1088, 1280, "camera_link", Time())
    assert message.distortion_model == "rational_polynomial"
    assert list(message.d) == list(range(8))


def test_camera_info_carries_four_coefficients_for_a_fisheye():
    intrinsics = _Intrinsics(
        600.0, 600.0, 544.0, 640.0, gs130.DistModel.FISHEYE, dist=np.arange(8, dtype=float)
    )
    message = convert.camera_info(intrinsics, 1088, 1280, "camera_link", Time())
    assert message.distortion_model == "equidistant"
    assert list(message.d) == [0.0, 1.0, 2.0, 3.0]


def test_a_rectified_frame_needs_no_distortion_model():
    intrinsics = _Intrinsics(
        600.0, 600.0, 544.0, 640.0, gs130.DistModel.FISHEYE, dist=np.arange(8, dtype=float)
    )
    message = convert.camera_info(
        intrinsics, 1088, 1280, "camera_link", Time(), rectified=True
    )
    assert message.distortion_model == "plumb_bob"
    assert list(message.d) == [0.0] * 5


# ---------------------------------------------------------------------------
# Packed frames
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "layout",
    [
        gs130.StereoLayout.NONE,
        gs130.StereoLayout.TOP_BOTTOM,
        gs130.StereoLayout.BOTTOM_TOP,
        gs130.StereoLayout.LEFT_RIGHT,
        gs130.StereoLayout.RIGHT_LEFT,
    ],
)
def test_eye_size_is_the_eye_not_the_frame(layout):
    eye_width, eye_height = 128, 96
    frame_width, frame_height = _frame_size(layout, eye_width, eye_height)
    assert convert.eye_size(layout, frame_width, frame_height) == (eye_width, eye_height)


def _frame_size(layout, eye_width, eye_height):
    """The frame the C layer builds around an eye of the given size."""
    if layout in (gs130.StereoLayout.TOP_BOTTOM, gs130.StereoLayout.BOTTOM_TOP):
        return eye_width, eye_height * 2
    if layout in (gs130.StereoLayout.LEFT_RIGHT, gs130.StereoLayout.RIGHT_LEFT):
        return eye_width * 2, eye_height
    return eye_width, eye_height


def _nv12(width, height, fill):
    return np.full(width * height * 3 // 2, fill, dtype=np.uint8)


def _bands(nv12, width, height):
    """Split one NV12 image into its luma rows and its chroma rows."""
    rows = nv12.reshape(-1, width)
    return rows[:height], rows[height:]


def _packed_frame(layout, width, height):
    """Build the buffer gs130.cpp writes for ``layout`` (gs130.cpp:262-301).

    Written from the C offsets, not from ``slice_eyes``: top/bottom puts the
    first eye's luma at 0, the second's at ``w*h``, then their chroma planes at
    ``2*w*h`` and ``2*w*h + w*h/2``; left/right interleaves the two eyes inside
    every row, at a stride of ``2*w``.
    """
    left = _nv12(width, height, 0x11)
    right = _nv12(width, height, 0x22)

    left_luma, left_chroma = _bands(left, width, height)
    right_luma, right_chroma = _bands(right, width, height)

    left_first = layout in (gs130.StereoLayout.TOP_BOTTOM, gs130.StereoLayout.LEFT_RIGHT)
    first = (left_luma, left_chroma) if left_first else (right_luma, right_chroma)
    second = (right_luma, right_chroma) if left_first else (left_luma, left_chroma)

    if layout in (gs130.StereoLayout.TOP_BOTTOM, gs130.StereoLayout.BOTTOM_TOP):
        # Luma of both eyes, then chroma of both: the two planes are separate
        # buffers handed to the hardware, not one image appended to another.
        packed = np.concatenate([first[0], second[0], first[1], second[1]])
    else:
        packed = np.concatenate(
            [
                np.concatenate([first[0], second[0]], axis=1),
                np.concatenate([first[1], second[1]], axis=1),
            ]
        )
    packed = packed.reshape(-1)

    frame_width, frame_height = _frame_size(layout, width, height)
    return _Frame(packed.reshape(frame_height * 3 // 2, frame_width))


@pytest.mark.parametrize(
    "layout",
    [
        gs130.StereoLayout.TOP_BOTTOM,
        gs130.StereoLayout.BOTTOM_TOP,
        gs130.StereoLayout.LEFT_RIGHT,
        gs130.StereoLayout.RIGHT_LEFT,
    ],
)
def test_slice_eyes_recovers_both_eyes(layout):
    width, height = 8, 6
    packed = _packed_frame(layout, width, height)
    left, right = convert.slice_eyes(packed, layout)
    eye_size = width * height * 3 // 2
    assert left.size == eye_size and right.size == eye_size
    assert np.all(left == 0x11)
    assert np.all(right == 0x22)


def test_slice_eyes_returns_arrays_that_outlive_the_frame():
    width, height = 8, 6
    packed = _packed_frame(gs130.StereoLayout.TOP_BOTTOM, width, height)
    left, right = convert.slice_eyes(packed, gs130.StereoLayout.TOP_BOTTOM)
    packed.fill(0)
    assert np.all(left == 0x11)
    assert np.all(right == 0x22)


# ---------------------------------------------------------------------------
# Images, IMU, transforms
# ---------------------------------------------------------------------------


def test_nv12_message_describes_the_buffer_it_carries():
    width, height = 8, 6
    frame = _Frame(_nv12(width, height, 0x33).reshape(height * 3 // 2, width))
    message = convert.nv12_message(frame, width, height, "camera_link", Time(sec=7))
    assert message.encoding == "nv12"
    assert (message.width, message.height, message.step) == (width, height, width)
    assert len(message.data) == width * height * 3 // 2
    assert message.header.frame_id == "camera_link"
    assert message.header.stamp.sec == 7


def test_nv12_message_payload_is_an_array_not_a_bytes():
    """The payload type is a performance contract, not a style choice.

    rclpy fills a ``uint8[]`` field one element at a time when it is given a
    ``bytes``: 3.3 seconds for a 1088x2560 NV12 frame on an RDK X5, against
    0.8 ms for an ``array.array``.  Nothing else about the two differs, so only
    this assertion stops the slow one coming back.
    """
    import array

    width, height = 8, 6
    source = _nv12(width, height, 0x33)
    frame = _Frame(source.reshape(height * 3 // 2, width))
    message = convert.nv12_message(frame, width, height, "camera_link", Time())
    assert isinstance(message.data, array.array)
    assert message.data.typecode == "B"
    assert bytes(message.data) == source.tobytes()


def test_nv12_message_copies_so_the_frame_can_be_dropped():
    width, height = 8, 6
    frame = _Frame(_nv12(width, height, 0x33).reshape(height * 3 // 2, width))
    message = convert.nv12_message(frame, width, height, "camera_link", Time())
    frame.fill(0)
    assert bytes(message.data) == bytes([0x33]) * (width * height * 3 // 2)


def test_imu_message_marks_the_missing_orientation():
    message = convert.imu_message(
        _Packet([1.0, 2.0, 3.0], [4.0, 5.0, 6.0], 9), "imu_link", Time()
    )
    acceleration = message.linear_acceleration
    assert (acceleration.x, acceleration.y, acceleration.z) == (1.0, 2.0, 3.0)
    rate = message.angular_velocity
    assert (rate.x, rate.y, rate.z) == (4.0, 5.0, 6.0)
    assert message.orientation_covariance[0] == -1.0


def test_transform_message_passes_the_device_pose_through():
    rotation = np.eye(3)
    message = convert.transform_message(
        rotation, (1.0, 2.0, 3.0), "camera_link", "imu_link", Time()
    )
    assert message.header.frame_id == "camera_link"
    assert message.child_frame_id == "imu_link"
    translation = message.transform.translation
    assert (translation.x, translation.y, translation.z) == (1.0, 2.0, 3.0)
    rotation = message.transform.rotation
    assert (rotation.x, rotation.y, rotation.z, rotation.w) == (0.0, 0.0, 0.0, 1.0)
