"""A ROS 2 node that publishes the GS130 stereo camera and its IMU.

The node is a thin mapping of the C API onto standard ROS messages: it opens
one :class:`gs130.Device`, drains its frame and IMU queues, and publishes what
comes out.  It does no image processing, no stereo matching and no filtering --
those belong to whatever consumes the topics.

Everything a deployment varies -- board, camera model, mode, size, rate, stereo
packing, topic names -- is a parameter, so one launch file covers the hardware
without a recompile.  The defaults follow D-Robotics' ``hobot_mipi_cam`` so the
streams drop into the official perception stack; see README.md for the details
and for the places where this node deliberately differs.
"""

from __future__ import annotations

import threading
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu

try:
    import gs130
except (ImportError, OSError, RuntimeError) as error:  # pragma: no cover
    raise RuntimeError(
        "the gs130 SDK is not usable from this process (%s); build and install "
        "the binding from this repository with 'pip install ./python', and put "
        "libgs130 where the loader can find it" % error
    ) from error

from . import convert


# hobot_mipi_cam publishes images with this depth (hobot_mipi_node.cpp:29) and
# the IMU with 10 (:463).  Matching them keeps the two nodes interchangeable.
PUB_BUF_NUM = 5
IMU_BUF_NUM = 10

# The IMU queue is drained on a timer; polling faster than this buys nothing.
IMU_POLL_FLOOR_SEC = 0.001

_LEFT = "left"
_RIGHT = "right"
_EYES = (_LEFT, _RIGHT)

# An enum in a launch file spells badly, so the parameters are strings and
# these two tables are the only place the names meet the C values.
_CAMERA_MODES = {
    "raw": gs130.CameraMode.RAW,
    "resize": gs130.CameraMode.RESIZE,
    "rect": gs130.CameraMode.RECT,
}

_STEREO_LAYOUTS = {
    "none": gs130.StereoLayout.NONE,
    "left_right": gs130.StereoLayout.LEFT_RIGHT,
    "right_left": gs130.StereoLayout.RIGHT_LEFT,
    "top_bottom": gs130.StereoLayout.TOP_BOTTOM,
    "bottom_top": gs130.StereoLayout.BOTTOM_TOP,
}


def _extrinsic_qos():
    """QoS for ``/imu_extrinsic``: one sample, kept for late subscribers.

    The official node builds this profile and then does not pass it to
    ``create_publisher`` (hobot_mipi_node.cpp:470-472), so its extrinsic topic
    is a race against the first IMU sample.  An extrinsic is a constant of the
    rig, so here it is latched instead.
    """
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class _TimestampPolicy:
    """Decides what stamp a message gets, and holds that decision.

    ``device`` passes the SDK's value through untouched, ``system`` always uses
    the moment the sample reached this node, and ``auto`` keeps the device
    value only while the first samples look usable: increasing, on the Unix
    epoch and agreeing with the IMU.  A clock that fails any of those would
    place every frame at the wrong time without saying so, which is worse than
    stamping late but correctly.
    """

    PROBE = 20
    EPOCH_FLOOR_NS = 1_577_836_800_000_000_000  # 2020-01-01, past any plausible epoch
    SLACK_NS = 86_400_000_000_000  # a device a day out is wrong, not merely unsynchronised
    SKEW_NS = 1_000_000_000

    def __init__(self, source, clock):
        # A Clock, not a callable: ``get_clock().now().to_msg`` reads like a
        # callable that reports the time and actually freezes at the moment it
        # is read, which is how every message ends up carrying the instant the
        # node started.
        self._clock = clock
        self._use_device = source != "system"
        self._decided = source != "auto"
        self._camera = {}
        self._imu = []
        self._lock = threading.Lock()

    def camera(self, device_ns, stream):
        """Take one camera stamp.  ``stream`` names the topic it belongs to.

        Monotonicity is judged per stream, because the two eyes of one pair
        carry the same stamp: fed through one list they would look like a clock
        that stalls every other sample, and a healthy device would be thrown
        away for it.
        """
        with self._lock:
            if not self._decided:
                self._camera.setdefault(stream, []).append(int(device_ns))
        return self._stamp(device_ns)

    def imu(self, device_ns):
        with self._lock:
            if not self._decided:
                self._imu.append(int(device_ns))
        return self._stamp(device_ns)

    def probing(self):
        """Whether the clock is still being judged, so nothing should publish."""
        return not self._decided

    def _stamp(self, device_ns):
        if self._use_device:
            return convert.to_time(device_ns)
        return self._clock.now().to_msg()

    def ready(self):
        """Whether enough camera samples have arrived to judge the clock."""
        with self._lock:
            if self._decided:
                return False
            return max((len(values) for values in self._camera.values()), default=0) >= self.PROBE

    def judge(self, wall_ns):
        """Latch the decision; return the problems found, empty when none."""
        with self._lock:
            self._decided = True
            camera = {stream: list(values) for stream, values in self._camera.items()}
            imu = list(self._imu)

        problems = []
        for stream in sorted(camera):
            stamps = camera[stream]
            if any(later <= earlier for earlier, later in zip(stamps, stamps[1:])):
                problems.append("the %s timestamps do not increase" % stream)

        stamps = [value for values in camera.values() for value in values]
        first = min(stamps)
        if first < self.EPOCH_FLOOR_NS:
            # A counter that starts near zero is time since boot, not a date.
            problems.append(
                "the first camera timestamp (%d ns, about %.1f s) looks like a "
                "monotonic clock rather than a Unix epoch" % (first, first / 1e9)
            )
        elif wall_ns >= self.EPOCH_FLOOR_NS and first > wall_ns + self.SLACK_NS:
            # Only meaningful against a system clock that is set; an unset one
            # says nothing about the device.
            problems.append(
                "the first camera timestamp (%d ns) is more than a day ahead of the "
                "system clock (%d ns)" % (first, wall_ns)
            )
        if imu:
            nearest = min(abs(one - other) for one in stamps for other in imu)
            if nearest > self.SKEW_NS:
                problems.append(
                    "the camera and IMU clocks differ by %.3f s" % (nearest / 1e9)
                )
        if problems:
            self._use_device = False
        return problems


class Gs130Node(Node):
    """Publishes one GS130 device as ``sensor_msgs`` streams."""

    def __init__(self):
        super().__init__("gs130_ros")
        # Set before anything else can fail, so teardown can tell a real fault
        # from the node simply going away.
        self._closing = False
        self._declare_parameters()

        self._mode_name = self.get_parameter("camera_mode").value
        self._layout_name = self.get_parameter("stereo_layout").value
        self._mode = self._lookup("camera_mode", self._mode_name, _CAMERA_MODES)
        self._layout = self._lookup("stereo_layout", self._layout_name, _STEREO_LAYOUTS)
        self._rectified = self._mode == gs130.CameraMode.RECT
        self._framerate = max(1, int(self.get_parameter("framerate").value))
        self._frame_id = self.get_parameter("frame_id").value
        self._imu_frame_id = self.get_parameter("imu_frame_id").value
        self._only_when_subscribed = bool(self.get_parameter("only_when_subscribed").value)
        self._publish_left_right = bool(self.get_parameter("publish_left_right").value)
        self._timestamps = _TimestampPolicy(
            self.get_parameter("timestamp_source").value, self.get_clock()
        )

        config = self._build_config()
        camera = config["camera_config"]
        self.get_logger().info(
            "opening %s %s: %s mode, %s layout, %dx%d per eye at %d fps"
            % (
                self.get_parameter("platform").value,
                self.get_parameter("device").value,
                self._mode_name,
                self._layout_name,
                camera["output_width"],
                camera["output_height"],
                self._framerate,
            )
        )
        self._device = gs130.Device(config)
        self._stitched = self._device.stitched
        self._camera_drain = max(1, int(config["camera_fifo"]["depth"]))
        self._imu_drain = max(1, int(config["imu_fifo"]["depth"]))

        self._calibration = None
        self._intrinsics = {}
        self._stereo_translation = None
        self._publish_info = False
        self._load_calibration()

        self._stitched_publisher = None
        self._stitched_info = {}
        self._raw_publishers = {}
        self._raw_infos = {}
        self._imu_publisher = None
        self._create_publishers()
        self._publish_extrinsics()

        self._device.start()
        self._create_timers()
        self.get_logger().info(
            "gs130 %s ready, library %s for %s"
            % (
                self._mode_name,
                gs130.library_version() or "unknown",
                gs130.library_platform() or "unknown",
            )
        )

    # -- parameters --------------------------------------------------------

    def _declare_parameters(self):
        """Declare every knob once, so launch files and `ros2 param` agree."""
        self.declare_parameter("platform", "RDKX5")
        self.declare_parameter("device", "GS130WI")
        self.declare_parameter("camera_mode", "rect")
        self.declare_parameter("image_width", 1088)
        self.declare_parameter("image_height", 1280)
        self.declare_parameter("framerate", 30)
        self.declare_parameter("imu_odr", 200)
        self.declare_parameter("stereo_layout", "top_bottom")
        self.declare_parameter("frame_id", "camera_link")
        self.declare_parameter("imu_frame_id", "imu_link")
        self.declare_parameter("publish_imu", True)
        self.declare_parameter("publish_left_right", False)
        self.declare_parameter("only_when_subscribed", False)
        self.declare_parameter("timestamp_source", "auto")
        self.declare_parameter("topic_combine", "image_combine_raw")
        self.declare_parameter("topic_stereo", "image_stereo_raw")
        self.declare_parameter("topic_left", "image_left_raw")
        self.declare_parameter("topic_right", "image_right_raw")
        # Absolute, because the official node declares these two that way and a
        # consumer written against it expects them at the root.
        self.declare_parameter("topic_imu", "/imu_data")
        self.declare_parameter("topic_imu_extrinsic", "/imu_extrinsic")

    def _lookup(self, name, value, table):
        if value not in table:
            raise ValueError(
                "%s must be one of %s, got %r" % (name, ", ".join(sorted(table)), value)
            )
        return table[value]

    def _build_config(self):
        """Turn the parameters into the configuration dict the SDK wants.

        The presets fill in the board wiring, and both of them pin
        ``stereo_layout`` to ``NONE`` (gs130_define.h:30), so the layout has to
        be applied on top -- it is the one field a preset cannot answer.
        """
        try:
            config = gs130.preset(
                self.get_parameter("platform").value,
                self.get_parameter("device").value,
                self._mode,
                int(self.get_parameter("image_width").value),
                int(self.get_parameter("image_height").value),
                self._framerate,
                int(self.get_parameter("imu_odr").value),
            )
        except ValueError as error:
            raise ValueError(
                "%s; the presets that exist are RDKX5/GS130WI and RDKX5/GS130W" % error
            ) from None
        config["camera_config"]["stereo_layout"] = self._layout
        return config

    # -- device ------------------------------------------------------------

    def _load_calibration(self):
        """Read the EEPROM calibration, once, before capture starts.

        Under ``rect`` the C layer has already replaced the intrinsics and the
        rotations with the virtual ones while initialising the camera
        (rdkx5.cpp:194-274), so a single read describes the frames this node is
        about to publish -- provided it happens after ``gs130.Device`` and not
        before it.
        """
        try:
            self._calibration = self._device.calibration()
        except gs130.GS130Error as error:
            self.get_logger().warning(
                "no camera calibration in the EEPROM (%s); images go out without "
                "camera_info, which leaves them unusable for depth" % error
            )
            return

        self._intrinsics = {
            _LEFT: self._calibration.camera_left,
            _RIGHT: self._calibration.camera_right,
        }
        self._publish_info = True
        self._stereo_translation = self._measure_stereo_translation()
        if self._rectified and self._stereo_translation is None:
            # Rectified frames are only meaningful as a stereo pair, and a pair
            # without a baseline is worse than no calibration at all: the
            # consumer computes all-zero depth and reports success.
            self._publish_info = False

    def _measure_stereo_translation(self):
        """The translation that turns a rectified pair into a stereo pair.

        This is ``relative_T(LEFT, RIGHT)``: the left eye's points seen from the
        right eye, which is the direction ``P = K * [I | t]`` wants.  Its x
        component is therefore ``-baseline``, and ``P[3] = fx * tx`` comes out
        negative for a right eye that sits at +x -- the sign ``stereoRectify``
        and the official dual-camera calibration both produce.  Its magnitude
        would be the baseline, and using that instead flips the sign of every
        depth a consumer computes.

        It is read rather than assumed: the calibration may carry a rotation and
        an off-axis offset as well, and a baseline taken as ``|T|`` would then
        be wrong by exactly the part that is not along the axis.
        """
        if not self._rectified:
            return None
        try:
            translation = self._device.relative_T(
                gs130.ReferenceFrame.CAMERA_LEFT, gs130.ReferenceFrame.CAMERA_RIGHT
            )
        except gs130.GS130Error as error:
            self.get_logger().error(
                "no relative extrinsics between the eyes (%s); withholding "
                "camera_info rather than publishing a zero baseline" % error
            )
            return None

        x, y, z = (float(value) for value in translation)
        if x == 0.0:
            self.get_logger().error(
                "the two eyes coincide in the rectified calibration; it holds no "
                "baseline, so camera_info is withheld"
            )
            return None

        off_axis = max(abs(y), abs(z)) / abs(x)
        if off_axis > 0.01:
            self.get_logger().warning(
                "the rectified pair is not parallel: the offset between the eyes is "
                "(%.6g, %.6g, %.6g), so %.1f%% of it is off the baseline axis; P "
                "carries the full fx*tx + cx*tz, but a consumer reading only "
                "P[3]/P[0] loses that part" % (x, y, z, 100.0 * off_axis)
            )
        else:
            self.get_logger().info(
                "baseline %.6g, so the right eye's P[3] is %.6g"
                % (abs(x), float(self._intrinsics[_LEFT].fx) * x)
            )
        return translation

    # -- publishers --------------------------------------------------------

    def _create_publishers(self):
        qos = QoSProfile(depth=PUB_BUF_NUM)

        if self._stitched:
            on_official_topic = self._layout == gs130.StereoLayout.TOP_BOTTOM
            combine_topic = self.get_parameter("topic_combine").value
            stereo_topic = self.get_parameter("topic_stereo").value
            topic = combine_topic if on_official_topic else stereo_topic
            self._stitched_publisher = self.create_publisher(Image, topic, qos)
            if not on_official_topic:
                self.get_logger().warning(
                    "%s is not top_bottom, so the stitched frame goes out on %s and %s "
                    "is left alone; hobot_stereonet assumes top_bottom with the left "
                    "eye on top and cannot read this stream"
                    % (self._layout_name, topic, combine_topic)
                )
            if self._publish_info:
                self._stitched_info = {
                    eye: self.create_publisher(
                        CameraInfo, "%s/%s/camera_info" % (topic, eye), qos
                    )
                    for eye in _EYES
                }

        # Two separate frames need their own topics.  A stitched frame does not,
        # but the official node publishes the pair alongside the combination
        # (hobot_mipi_node.cpp:263-275), so it is available on request.
        if not self._stitched or self._publish_left_right:
            self._raw_publishers = {
                eye: self.create_publisher(Image, self.get_parameter("topic_%s" % eye).value, qos)
                for eye in _EYES
            }
            if self._publish_info:
                self._raw_infos = {
                    eye: self.create_publisher(
                        CameraInfo,
                        "%s/camera_info" % self.get_parameter("topic_%s" % eye).value,
                        qos,
                    )
                    for eye in _EYES
                }

        if self._imu_is_usable():
            self._imu_publisher = self.create_publisher(
                Imu, self.get_parameter("topic_imu").value, IMU_BUF_NUM
            )

    def _imu_is_usable(self):
        if not self.get_parameter("publish_imu").value:
            self.get_logger().info("IMU publishing is off")
            return False
        name = self._device.imu_name
        if name is None:
            self.get_logger().info("no IMU detected on this device; publishing none")
            return False
        self.get_logger().info("IMU detected: %s" % name)
        return True

    def _publish_extrinsics(self):
        """Publish ``camera_link -> imu_link``, as TF and as a latched topic.

        The transform is a constant of the rig, so it goes on the static
        broadcaster and on a topic that keeps its one sample for late
        subscribers, stamped zero.  Neither ``this->now()`` nor an IMU sample
        time describes a constant, and a consumer given one would be invited to
        interpolate it.
        """
        if self._imu_publisher is None:
            return
        try:
            rotation = self._device.relative_R(
                gs130.ReferenceFrame.IMU, gs130.ReferenceFrame.CAMERA_LEFT
            )
            translation = self._device.relative_T(
                gs130.ReferenceFrame.IMU, gs130.ReferenceFrame.CAMERA_LEFT
            )
        except gs130.GS130Error as error:
            self.get_logger().warning(
                "no IMU extrinsics in the EEPROM (%s); not publishing %s"
                % (error, self.get_parameter("topic_imu_extrinsic").value)
            )
            return

        message = convert.transform_message(
            rotation, translation, self._frame_id, self._imu_frame_id, convert.to_time(0)
        )
        try:
            from tf2_ros import StaticTransformBroadcaster
        except ImportError as error:
            self.get_logger().warning(
                "tf2_ros is unavailable (%s); %s -> %s goes out as a topic only"
                % (error, self._frame_id, self._imu_frame_id)
            )
        else:
            self._static_tf = StaticTransformBroadcaster(self)
            self._static_tf.sendTransform(message)

        extrinsic_topic = self.get_parameter("topic_imu_extrinsic").value
        self._extrinsic_publisher = self.create_publisher(
            TransformStamped, extrinsic_topic, _extrinsic_qos()
        )
        self._extrinsic_publisher.publish(message)
        self.get_logger().info(
            "%s -> %s published; the axes are the device's own and are not yet "
            "checked against REP-103" % (self._frame_id, self._imu_frame_id)
        )

    # -- timers ------------------------------------------------------------

    def _create_timers(self):
        # One callback group per timer, so a slow frame does not delay the IMU
        # and a burst of IMU samples does not stall the camera.
        self._camera_timer = self.create_timer(
            1.0 / self._framerate,
            self._camera_tick,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        if self._imu_publisher is None:
            self._imu_timer = None
            return
        period = max(1.0 / max(1, int(self.get_parameter("imu_odr").value)), IMU_POLL_FLOOR_SEC)
        self._imu_timer = self.create_timer(
            period, self._imu_tick, callback_group=MutuallyExclusiveCallbackGroup()
        )

    # -- capture -----------------------------------------------------------

    def _camera_tick(self):
        if self._closing:
            return
        try:
            self._run_camera_tick()
        except Exception as error:  # noqa: BLE001 - one bad frame must not kill the node
            self._report_tick_failure("camera", error)

    def _run_camera_tick(self):
        # The clock is judged before anything is published: a stamp the node is
        # about to disown must never reach a subscriber, because a consumer that
        # sees boot-clock stamps and then epoch ones sees a jump of decades.
        # The cost is a fraction of a second of frames at startup.
        if self._timestamps.probing():
            self._probe_clock()
            return
        if self._only_when_subscribed and not self._any_image_subscriber():
            # Nothing is listening, so nothing needs to be fresh.  The device
            # queue holds at most `depth` frames and drops the oldest, so an
            # idle second does not have to be drained later -- the next tick
            # finds a current frame.  The timer period is the sleep.
            return
        for _ in range(self._camera_drain):
            frames = self._device.read_image()
            if frames is None:
                return
            if "stitched" in frames:
                frame = frames["stitched"]
                self._publish_stitched(
                    frame, self._timestamps.camera(frame.timestamp_ns, "stitched")
                )
            else:
                for eye in _EYES:
                    frame = frames[eye]
                    self._publish_eye(
                        eye, frame, self._timestamps.camera(frame.timestamp_ns, eye)
                    )

    def _probe_clock(self):
        """Drain samples for the clock check, publishing none of them."""
        for _ in range(self._camera_drain):
            frames = self._device.read_image()
            if frames is None:
                break
            if "stitched" in frames:
                frame = frames["stitched"]
                self._timestamps.camera(frame.timestamp_ns, "stitched")
            else:
                for eye in _EYES:
                    frame = frames[eye]
                    self._timestamps.camera(frame.timestamp_ns, eye)
        self._judge_timestamps()

    def _imu_tick(self):
        if self._closing:
            return
        try:
            self._run_imu_tick()
        except Exception as error:  # noqa: BLE001 - one bad packet must not kill the node
            self._report_tick_failure("IMU", error)

    def _run_imu_tick(self):
        probing = self._timestamps.probing()
        if not probing and self._only_when_subscribed:
            if self._imu_publisher.get_subscription_count() == 0:
                return
        for _ in range(self._imu_drain):
            packet = self._device.read_imu()
            if packet is None:
                return
            stamp = self._timestamps.imu(packet.timestamp_ns)
            if not probing:
                self._imu_publisher.publish(
                    convert.imu_message(packet, self._imu_frame_id, stamp)
                )

    def _report_tick_failure(self, what, error):
        if self._closing:
            return
        self.get_logger().error(
            "the %s tick failed: %s" % (what, error), throttle_duration_sec=1.0
        )

    def _any_image_subscriber(self):
        publishers = list(self._raw_publishers.values())
        publishers += list(self._stitched_info.values()) + list(self._raw_infos.values())
        if self._stitched_publisher is not None:
            publishers.append(self._stitched_publisher)
        return any(publisher.get_subscription_count() > 0 for publisher in publishers)

    def _judge_timestamps(self):
        if not self._timestamps.ready():
            return
        problems = self._timestamps.judge(time.time_ns())
        if problems:
            self.get_logger().error(
                "the device clock looks unusable (%s); stamping every message with the "
                "receive time instead, which is late but on the right clock.  Set "
                "timestamp_source:=device to keep the device clock anyway." % "; ".join(problems)
            )
        else:
            self.get_logger().info(
                "using the device timestamps: they increase, are on the Unix epoch "
                "and agree with the IMU"
            )

    # -- publishing --------------------------------------------------------

    def _camera_info(self, eye, width, height, stamp):
        """The ``CameraInfo`` for one eye of one frame.

        ``width`` and ``height`` are the eye's own, taken from the frame that
        was actually returned rather than from the parameters: ``rect`` passes
        its output through a VSE crop and rescale (rdkx5.cpp:249-274), and a
        declared size that disagrees with the pixels mis-scales every intrinsic
        downstream.
        """
        intrinsics = self._intrinsics[eye]
        projection_matrix = None
        if self._rectified and self._stereo_translation is not None:
            # The left eye *is* the frame the right eye is measured in, so its
            # own translation is zero and its P[3] is zero -- what
            # stereoRectify and the official calibration both emit.
            translation = self._stereo_translation if eye == _RIGHT else (0.0, 0.0, 0.0)
            projection_matrix = convert.stereo_projection(intrinsics, translation)
        return convert.camera_info(
            intrinsics,
            width,
            height,
            self._frame_id,
            stamp,
            projection_matrix=projection_matrix,
            rectified=self._rectified,
        )

    def _publish_stitched(self, frame, stamp):
        width, height = int(frame.width), int(frame.height)
        eye_width, eye_height = convert.eye_size(self._layout, width, height)

        # camera_info first, then the image, with one stamp for both -- the
        # order the official node uses, so a consumer keyed on the stamp has
        # the calibration in hand before the frame arrives.
        for eye in _EYES:
            if eye in self._stitched_info:
                self._stitched_info[eye].publish(
                    self._camera_info(eye, eye_width, eye_height, stamp)
                )
        self._stitched_publisher.publish(
            convert.nv12_message(frame, width, height, self._frame_id, stamp)
        )

        if self._raw_publishers:
            for eye, data in zip(_EYES, convert.slice_eyes(frame, self._layout)):
                if eye in self._raw_infos:
                    self._raw_infos[eye].publish(
                        self._camera_info(eye, eye_width, eye_height, stamp)
                    )
                self._raw_publishers[eye].publish(
                    convert.nv12_message(data, eye_width, eye_height, self._frame_id, stamp)
                )

    def _publish_eye(self, eye, frame, stamp):
        width, height = int(frame.width), int(frame.height)
        if eye in self._raw_infos:
            self._raw_infos[eye].publish(self._camera_info(eye, width, height, stamp))
        self._raw_publishers[eye].publish(
            convert.nv12_message(frame, width, height, self._frame_id, stamp)
        )

    # -- teardown ----------------------------------------------------------

    def destroy_node(self):
        # Stop the timers first: a tick that publishes into a torn-down context
        # raises, and an exception raised inside a callback nobody awaits is
        # reported as "never retrieved" rather than as the teardown it is.
        self._closing = True
        # Releasing the camera stops its capture threads, so it has to happen
        # while the node can still report a failure.
        try:
            self._device.close()
        except Exception as error:  # noqa: BLE001 - teardown must not mask the cause
            self.get_logger().error("closing the camera failed: %s" % error)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Gs130Node()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
