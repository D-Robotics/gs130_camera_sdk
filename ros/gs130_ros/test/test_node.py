"""Tests for the node's decisions that do not need a camera attached.

The numbers the timing tests use are the ones an RDK X5 with a GS130WI actually
produced: a metric baseline of 0.07031615 m between the rectified eyes, one
shared stamp per eye pair, and camera stamps that count nanoseconds since boot
rather than since the epoch (so ``auto`` rejects them).
"""

import pytest

from gs130_ros import node


SYSTEM_TIME = object()


class _MarkerTime:
    """Stands in for an rclpy Time."""

    def to_msg(self):
        return SYSTEM_TIME


class _FixedClock:
    """Stands in for an rclpy Clock."""

    def now(self):
        return _MarkerTime()


# What the device reported, in nanoseconds.
DEVICE_STAMP = 2_122_032_477_000
MEASURED_TRANSLATION = -0.07031615052675907


def _policy(source):
    return node._TimestampPolicy(source, _FixedClock())


def _epoch_run(policy, count=node._TimestampPolicy.PROBE, stream="stitched"):
    """Feed ``count`` increasing stamps that look like a Unix clock."""
    start = node._TimestampPolicy.EPOCH_FLOOR_NS + 1_000_000_000
    stamps = [start + index * 33_000_000 for index in range(count)]
    for stamp in stamps:
        policy.camera(stamp, stream)
    return stamps


def test_system_stamps_are_never_the_device_ones():
    policy = _policy("system")
    assert policy.camera(DEVICE_STAMP, "stitched") is SYSTEM_TIME
    assert not policy.ready()


def test_device_stamps_are_passed_through():
    policy = _policy("device")
    assert policy.camera(1_500_000_002, "stitched").nanosec == 500_000_002


def test_auto_keeps_a_device_clock_that_checks_out():
    policy = _policy("auto")
    stamps = _epoch_run(policy)
    assert policy.ready()
    assert policy.judge(stamps[-1] + 1_000_000_000) == []
    # Still the device's value, now that the probe has finished.
    assert policy.camera(1_500_000_002, "stitched").nanosec == 500_000_002


def test_auto_rejects_the_uptime_clock_the_device_actually_reports():
    policy = _policy("auto")
    for index in range(node._TimestampPolicy.PROBE):
        policy.camera(DEVICE_STAMP + index * 33_000_000, "stitched")
    problems = policy.judge(1_700_000_000_000_000_000)
    assert any("monotonic clock" in problem for problem in problems)
    assert policy.camera(DEVICE_STAMP, "stitched") is SYSTEM_TIME


def test_auto_keeps_an_epoch_clock_even_when_the_system_clock_is_unset():
    # A board with no RTC sits in 2000; that says nothing about the device.
    policy = _policy("auto")
    _epoch_run(policy)
    assert policy.judge(946_686_859_000_000_000) == []


def test_auto_rejects_a_clock_far_ahead_of_a_set_system_clock():
    policy = _policy("auto")
    start = node._TimestampPolicy.EPOCH_FLOOR_NS + 100 * 86_400_000_000_000
    for index in range(node._TimestampPolicy.PROBE):
        policy.camera(start + index * 33_000_000, "stitched")
    problems = policy.judge(node._TimestampPolicy.EPOCH_FLOOR_NS + 1_000_000_000)
    assert any("ahead of the system clock" in problem for problem in problems)


def test_auto_falls_back_when_a_stream_does_not_increase():
    policy = _policy("auto")
    start = node._TimestampPolicy.EPOCH_FLOOR_NS + 1_000_000_000
    for index in range(node._TimestampPolicy.PROBE):
        policy.camera(start + (index % 2) * 1_000_000, "stitched")
    problems = policy.judge(start + 1_000_000_000)
    assert any("stitched timestamps do not increase" in problem for problem in problems)
    assert policy.camera(start, "stitched") is SYSTEM_TIME


def test_two_eyes_sharing_a_stamp_is_not_a_stalled_clock():
    # Measured on the device: left and right carry one identical stamp, so
    # interleaving them must not look like a clock that repeats itself.
    policy = _policy("auto")
    start = node._TimestampPolicy.EPOCH_FLOOR_NS + 1_000_000_000
    for index in range(node._TimestampPolicy.PROBE):
        stamp = start + index * 33_000_000
        policy.camera(stamp, "left")
        policy.camera(stamp, "right")
    assert policy.judge(start + node._TimestampPolicy.PROBE * 33_000_000) == []


def test_auto_rejects_a_clock_that_disagrees_with_the_imu():
    policy = _policy("auto")
    start = node._TimestampPolicy.EPOCH_FLOOR_NS + 1_000_000_000
    for index in range(node._TimestampPolicy.PROBE):
        policy.camera(start + index * 33_000_000, "stitched")
        policy.imu(start + 60_000_000_000 + index * 33_000_000)
    problems = policy.judge(start + 1_000_000_000)
    assert any("clocks differ" in problem for problem in problems)


def test_auto_accepts_a_clock_in_step_with_the_imu():
    policy = _policy("auto")
    start = node._TimestampPolicy.EPOCH_FLOOR_NS + 1_000_000_000
    for index in range(node._TimestampPolicy.PROBE):
        policy.camera(start + index * 33_000_000, "stitched")
        policy.imu(start + index * 33_000_000 + 1_000)
    assert policy.judge(start + 1_000_000_000) == []


def test_auto_does_not_judge_before_the_probe_is_full():
    policy = _policy("auto")
    policy.camera(node._TimestampPolicy.EPOCH_FLOOR_NS + 1, "stitched")
    assert not policy.ready()
    assert policy.probing()


def test_the_extrinsic_profile_is_latched_and_reliable():
    from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

    qos = node._extrinsic_qos()
    assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.depth == 1


def test_every_layout_the_device_has_is_reachable():
    import gs130

    assert set(node._STEREO_LAYOUTS.values()) == set(gs130.StereoLayout)
    assert set(node._CAMERA_MODES.values()) == set(gs130.CameraMode)


def test_only_auto_probes_the_clock():
    # The other two are instructions, not questions, so nothing is held back.
    assert not _policy("device").probing()
    assert not _policy("system").probing()


def test_auto_probes_until_it_judges():
    policy = _policy("auto")
    assert policy.probing()
    stamps = _epoch_run(policy)
    assert policy.ready()
    policy.judge(stamps[-1])
    assert not policy.probing()


def test_probing_still_records_what_it_needs():
    policy = _policy("auto")
    start = node._TimestampPolicy.EPOCH_FLOOR_NS + 1_000_000_000
    for index in range(node._TimestampPolicy.PROBE):
        policy.camera(start + index * 33_000_000, "stitched")
        policy.imu(start + index * 33_000_000)
    assert policy.ready()
    assert policy.judge(start) == []


def test_the_fallback_stamp_advances_with_the_clock():
    """Each message needs its own reading, not the one taken at construction.

    The node used to hand the policy ``get_clock().now().to_msg``, which reads
    like a callable that reports the time and is really a bound method of one
    frozen instant: every message carried the moment the node started.  On the
    board that showed up as a thousand IMU samples sharing one stamp.  The
    policy now takes the Clock itself, so there is nothing to freeze.
    """
    import time

    import rclpy
    from rclpy.clock import Clock

    rclpy.init()
    try:
        policy = node._TimestampPolicy("system", Clock())
        first = policy.camera(1, "stitched")
        time.sleep(0.02)
        second = policy.camera(2, "stitched")
        assert (first.sec, first.nanosec) != (second.sec, second.nanosec)
        assert (second.sec, second.nanosec) > (first.sec, first.nanosec)
    finally:
        rclpy.try_shutdown()


def test_the_stereo_translation_is_the_left_to_right_direction():
    """The measured -0.0703 m must become a negative P[3], as stereoRectify does."""
    from gs130_ros import convert

    intrinsics = _Intrinsics(fx=615.5139302922461, fy=615.5139302922461, cx=544.0, cy=640.0)
    projection = convert.stereo_projection(intrinsics, (MEASURED_TRANSLATION, 0.0, 0.0))
    assert projection[3] == pytest.approx(-43.28057017374667)
    assert abs(projection[3] / projection[0]) == pytest.approx(0.07031615052675907)


class _Intrinsics:
    def __init__(self, fx, fy, cx, cy):
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
