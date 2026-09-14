"""The gs130 ROS 2 camera node: stereo frames, IMU and calibration.

The node owns the gs130.Device for its whole life, polls the SDK's
non-blocking reads from timers, and publishes standard ROS messages so that
existing TROS nodes (hobot_codec, websocket) work unchanged.
"""

import array
import signal
import sys
import time

import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu
from tf2_ros import StaticTransformBroadcaster

from .calibration import camera_info, transform

try:
    import gs130
except ImportError as error:
    print("gs130_ros needs the gs130 Python package, which is not importable: %s" % error,
          file=sys.stderr)
    print("build and install it from the gs130_sdk repository: "
          "cd python && ./build-wheel.sh wheel && sudo pip3 install dist/gs130-*.whl",
          file=sys.stderr)
    raise SystemExit(2)

# hobot_codec subscribes reliably, so a best-effort publisher is invisible.
IMAGE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
IMU_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=200,
)
INFO_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

MODES = {
    "raw": gs130.CameraMode.RAW,
    "resize": gs130.CameraMode.RESIZE,
    "rect": gs130.CameraMode.RECT,
}
LAYOUTS = {
    "none": gs130.StereoLayout.NONE,
    "left_right": gs130.StereoLayout.LEFT_RIGHT,
    "right_left": gs130.StereoLayout.RIGHT_LEFT,
    "top_bottom": gs130.StereoLayout.TOP_BOTTOM,
    "bottom_top": gs130.StereoLayout.BOTTOM_TOP,
}
SENSOR_SIZE = (1088, 1280)
MAX_FPS = 33
REPORT_PERIOD_S = 5.0


def _boolean(value):
    """A launch argument arrives as text, so 'false' must not mean true."""
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in ("true", "1", "yes", "on"):
        return True
    if text in ("false", "0", "no", "off"):
        return False
    raise ValueError(value)


class Gs130Camera(Node):
    """Publishes the GS130 stereo camera and IMU as ROS topics."""

    def __init__(self):
        super().__init__("gs130_camera")
        self.declare_parameter("platform", "RDKX5")
        self.declare_parameter("device", "GS130WI")
        self.declare_parameter("mode", "resize")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30)
        self.declare_parameter("odr", 200)
        self.declare_parameter("stereo_layout", "left_right")
        self.declare_parameter("frame_id_camera", "camera_left")
        self.declare_parameter("frame_id_imu", "imu_link")
        self.declare_parameter("publish_imu", True)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("start_timeout_s", 10.0)

        self.platform = self._text_parameter("platform")
        self.device_name = self._text_parameter("device")
        self.mode_name = self._text_parameter("mode")
        self.width = self._number_parameter("width", int)
        self.height = self._number_parameter("height", int)
        self.fps = self._number_parameter("fps", int)
        self.odr = self._number_parameter("odr", int)
        self.layout_name = self._text_parameter("stereo_layout")
        self.frame_camera = self._text_parameter("frame_id_camera")
        self.frame_imu = self._text_parameter("frame_id_imu")
        self.publish_imu = self._number_parameter("publish_imu", _boolean)
        self.publish_tf = self._number_parameter("publish_tf", _boolean)
        self.start_timeout = self._number_parameter("start_timeout_s", float)

        self._validate()
        # The device is stored before any call that can fail on it, so that a
        # failure while starting still runs the cleanup path below.
        self.device = self._create_device()
        try:
            self.device.start()
            self._start()
        except BaseException:
            self.shutdown()
            raise

    def _text_parameter(self, name):
        value = self.get_parameter(name).value
        if not isinstance(value, str) or not value:
            self.fail("parameter %s must be a non-empty string, got %r" % (name, value))
        return value

    def _number_parameter(self, name, convert):
        value = self.get_parameter(name).value
        try:
            return convert(value)
        except (TypeError, ValueError):
            self.fail("parameter %s must be a number, got %r" % (name, value))

    def _start(self):
        """Bring up publishers, calibration and timers on a live device."""
        self.stitched = LAYOUTS[self.layout_name] != gs130.StereoLayout.NONE
        self.offset_ns = 0
        self.frames = 0
        self.packets = 0
        self._last_frames = 0
        self._stalled = 0

        if self.stitched:
            self.image_publisher = self.create_publisher(Image, "/image_combine_raw", IMAGE_QOS)
        else:
            self.left_publisher = self.create_publisher(Image, "/image_left_raw", IMAGE_QOS)
            self.right_publisher = self.create_publisher(Image, "/image_right_raw", IMAGE_QOS)
        self.imu_publisher = None
        if self.publish_imu:
            self.imu_publisher = self.create_publisher(Imu, "/imu/data", IMU_QOS)

        self.publish_calibration()
        self._publish_transforms()
        self._wait_for_first_frame()
        self._report()

        self.create_timer(1.0 / max(2 * self.fps, 10), self._poll_images)
        if self.publish_imu:
            self.create_timer(1.0 / max(2 * self.odr, 20), self._poll_imu)
        self.create_timer(REPORT_PERIOD_S, self._report)
        self.get_logger().info(
            "gs130 camera ready: %s publishes %s, not RGB. Decode with "
            "cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_NV12)."
            % (self._image_topics(), "NV12")
        )

    def _image_topics(self):
        if self.stitched:
            return "/image_combine_raw"
        return "/image_left_raw and /image_right_raw"

    # ---------------------------------------------------------------- setup

    def _validate(self):
        """Reject anything the SDK or the hardware cannot do, before init."""
        if self.mode_name not in MODES:
            self.fail("mode must be one of %s, got '%s'" % (", ".join(MODES), self.mode_name))
        if self.layout_name not in LAYOUTS:
            self.fail("stereo_layout must be one of %s, got '%s'"
                      % (", ".join(LAYOUTS), self.layout_name))
        if self.fps < 1 or self.fps > MAX_FPS:
            self.fail("fps must be between 1 and %d, got %d" % (MAX_FPS, self.fps))
        if self.odr < 1:
            self.fail("odr must be at least 1, got %d" % self.odr)
        if self.width < 1 or self.height < 1:
            self.fail("width and height must be positive, got %dx%d" % (self.width, self.height))
        if self.width % 2 or self.height % 2:
            # The published height is derived from the packed NV12 row count, so
            # an odd value would not survive the round trip and the message
            # geometry would disagree with camera_info.
            self.fail("width and height must be even for NV12, got %dx%d"
                      % (self.width, self.height))
        if self.mode_name == "raw":
            if (self.width, self.height) != SENSOR_SIZE:
                self.fail("mode raw requires width=%d height=%d, got %dx%d"
                          % (SENSOR_SIZE[0], SENSOR_SIZE[1], self.width, self.height))
            if LAYOUTS[self.layout_name] != gs130.StereoLayout.NONE:
                self.fail(
                    "mode raw cannot be combined with stereo_layout=%s: raw output "
                    "must be the %dx%d sensor size, which stitching would double. "
                    "Use mode:=resize or stereo_layout:=none."
                    % (self.layout_name, SENSOR_SIZE[0], SENSOR_SIZE[1])
                )

    def _create_device(self):
        """Build the configuration and initialize the camera."""
        try:
            config = gs130.Config.preset(
                self.platform, self.device_name, MODES[self.mode_name],
                self.width, self.height, self.fps, self.odr,
            )
        except ValueError as error:
            self.fail("unsupported platform/device: %s %s (%s)"
                      % (self.platform, self.device_name, error))
        # The SDK stitches in hardware; the web chain then sees one frame.
        config.camera_config.stereo_layout = LAYOUTS[self.layout_name]
        try:
            return gs130.Device(config)
        except gs130.GS130Error as error:
            self.fail(
                "gs130_init failed (%s). The camera is exclusive: check that no "
                "other process holds it (mipi_cam, another camera_node, or a "
                "leftover script) with 'ps -ef | grep -E \"mipi_cam|camera_node\"'."
                % error,
                code=1,
            )
        except OSError as error:
            self.fail("libgs130 could not be loaded: %s" % error, code=1)

    def fail(self, message, code=2):
        """Log a fatal problem and leave, without leaving device state behind."""
        self.get_logger().fatal(message)
        raise SystemExit(code)

    # --------------------------------------------------------------- output

    def publish_calibration(self):
        """Publish both CameraInfo once; they are latched for late joiners."""
        left_info = self.create_publisher(CameraInfo, "/image_left/camera_info", INFO_QOS)
        right_info = self.create_publisher(CameraInfo, "/image_right/camera_info", INFO_QOS)
        left = self.device.camera_intrinsics(gs130.CameraIndex.LEFT)
        right = self.device.camera_intrinsics(gs130.CameraIndex.RIGHT)
        left_message = camera_info(left, self.width, self.height, self.frame_camera)
        right_message = camera_info(right, self.width, self.height, "camera_right")
        left_message.header.stamp = self.get_clock().now().to_msg()
        right_message.header.stamp = left_message.header.stamp
        left_info.publish(left_message)
        right_info.publish(right_message)
        self.get_logger().info(
            "calibration: left fx=%.2f fy=%.2f cx=%.2f cy=%.2f %s | right fx=%.2f %s"
            % (left.fx, left.fy, left.cx, left.cy, left_message.distortion_model,
               right.fx, right_message.distortion_model)
        )

    def _publish_transforms(self):
        """Publish the extrinsics the EEPROM calibration provides."""
        if not self.publish_tf:
            return
        self.broadcaster = StaticTransformBroadcaster(self)
        messages = [
            transform(self.device, gs130.ReferenceFrame.CAMERA_RIGHT,
                      gs130.ReferenceFrame.CAMERA_LEFT,
                      self.frame_camera, "camera_right"),
        ]
        if self.device.imu_name is not None:
            messages.append(
                transform(self.device, gs130.ReferenceFrame.IMU,
                          gs130.ReferenceFrame.CAMERA_LEFT,
                          self.frame_camera, self.frame_imu)
            )
        now = self.get_clock().now().to_msg()
        for message in messages:
            message.header.stamp = now
        self.broadcaster.sendTransform(messages)
        baseline = messages[0].transform.translation
        self.get_logger().info(
            "static tf from %s to %s (baseline %.6f m)"
            % (self.frame_camera, ", ".join(m.child_frame_id for m in messages),
               (baseline.x ** 2 + baseline.y ** 2 + baseline.z ** 2) ** 0.5)
        )

    # ---------------------------------------------------------------- reads

    def _poll_images(self):
        """Publish every frame that is ready, at most two per tick."""
        for _ in range(2):
            try:
                images = self.device.read_image()
            except gs130.GS130Error as error:
                self.get_logger().error("read_image failed: %s" % error)
                return
            if images is None:
                return
            stamp = self.stamp_of(next(iter(images.values())))
            if self.stitched:
                self._publish_image(self.image_publisher, images["stitched"], "camera", stamp)
            else:
                self._publish_image(self.left_publisher, images["left"], self.frame_camera, stamp)
                self._publish_image(self.right_publisher, images["right"], "camera_right", stamp)
            self.frames += 1
            if self.frames == 1:
                self.get_logger().info("first frame published")

    def _publish_image(self, publisher, frame, frame_id, stamp):
        """Copy an NV12 frame into a message and let the SDK buffer go.

        The SDK owns the buffer through the numpy array and frees it when the
        array dies, so the data must be copied before this returns.
        """
        publisher.publish(image_message(frame, frame_id, stamp))

    def _poll_imu(self):
        """Publish every IMU packet that is ready, at most 64 per tick."""
        for _ in range(64):
            try:
                packet = self.device.read_imu()
            except gs130.GS130Error as error:
                self.get_logger().error("read_imu failed: %s" % error)
                return
            if packet is None:
                return
            self.imu_publisher.publish(self._imu_message(packet))
            self.packets += 1

    def _imu_message(self, packet):
        message = Imu()
        message.header.stamp = self.stamp_of(packet)
        message.header.frame_id = self.frame_imu
        message.angular_velocity.x = float(packet.gyro[0])
        message.angular_velocity.y = float(packet.gyro[1])
        message.angular_velocity.z = float(packet.gyro[2])
        message.linear_acceleration.x = float(packet.accel[0])
        message.linear_acceleration.y = float(packet.accel[1])
        message.linear_acceleration.z = float(packet.accel[2])
        message.orientation.w = 1.0
        # The SDK reports no fused orientation, so it is marked unavailable.
        # The gyro and accelerometer are measured but carry no variance, which
        # ROS reads as "unknown" from the zeroed covariances.
        message.orientation_covariance[0] = -1.0
        return message

    # ----------------------------------------------------------- timestamps

    def _wait_for_first_frame(self):
        """Take the device clock offset from the first frame that arrives."""
        deadline = time.monotonic() + self.start_timeout
        reported = 0.0
        while time.monotonic() < deadline:
            images = self.device.read_image()
            if images is not None:
                # The SDK stamps CLOCK_MONOTONIC nanoseconds since boot, not
                # Unix epoch, so one offset puts every later stamp in the same
                # domain as the rest of ROS while keeping frame spacing intact.
                self.offset_ns = self.get_clock().now().nanoseconds - images[
                    next(iter(images))
                ].timestamp_ns
                self.get_logger().info(
                    "gs130 timestamps are monotonic since boot; using a constant "
                    "offset of %d ns (accuracy within one frame period)"
                    % self.offset_ns
                )
                return
            waited = time.monotonic() - (deadline - self.start_timeout)
            if waited - reported >= 2.0:
                reported = waited
                self.get_logger().info(
                    "waiting for the first frame (%.0f of %.0f s)"
                    % (waited, self.start_timeout)
                )
            time.sleep(0.005)
        self.fail("no frame within %.1f s; check the camera and the IMU FSYNC wiring"
                  % self.start_timeout, code=1)

    def to_ros_ns(self, timestamp_ns):
        return timestamp_ns + self.offset_ns

    def stamp_of(self, frame):
        return _stamp(self.to_ros_ns(int(frame.timestamp_ns)))

    # -------------------------------------------------------------- reports

    def _report(self):
        """Report progress, and notice a stream that another process stole.

        Measured behaviour: the camera pipeline is not shareable, but a second
        opener is not refused, it simply takes the frames. The first node then
        keeps publishing IMU while its images stop, with no error anywhere.
        """
        self.get_logger().info(
            "frames=%d imu=%d | %s | %dx%d %s fps=%d odr=%d"
            % (self.frames, self.packets, self.layout_name, self.width, self.height,
               self.mode_name, self.fps, self.odr)
        )
        if self.frames == self._last_frames:
            self._stalled += 1
            if self._stalled == 2:
                self.get_logger().error(
                    "no camera frames for about %d s while the IMU still streams. "
                    "Another process has most likely taken the camera (mipi_cam or a "
                    "second camera_node); check with "
                    "'ps -ef | grep -E \"mipi_cam|camera_node\"'."
                    % int(2 * REPORT_PERIOD_S)
                )
        else:
            self._stalled = 0
        self._last_frames = self.frames

    def shutdown(self):
        """Release the camera. Never rely on garbage collection for this.

        A second SIGINT arriving mid-cleanup would otherwise abort the release
        and leave the camera held by a dying process, so it is ignored here.
        """
        device, self.device = self.device, None
        if device is None:
            return
        previous = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            try:
                device.stop()
            except Exception as error:
                self.get_logger().warning("stop failed: %s" % error)
            try:
                device.close()
            except Exception as error:
                self.get_logger().error("close failed: %s" % error)
            if device.closed:
                self.get_logger().info("camera released")
        finally:
            signal.signal(signal.SIGINT, previous)


def image_message(frame, frame_id, stamp):
    """An NV12 frame as a sensor_msgs/Image.

    height is the real image height, not the packed NV12 row count: the TROS
    codec reads it as the picture height and crashes on the packed value. The
    data is built as an array.array because assigning bytes makes rclpy
    validate every element in Python, which costs about a second per frame.
    """
    data = array.array("B")
    data.frombytes(frame.tobytes())
    message = Image()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.encoding = "nv12"
    message.is_bigendian = 0
    message.width = int(frame.shape[1])
    message.height = int(frame.shape[0]) * 2 // 3
    message.step = int(frame.shape[1])
    message.data = data
    return message


def _stamp(nanoseconds):
    stamp = Time()
    stamp.sec = int(nanoseconds // 1_000_000_000)
    stamp.nanosec = int(nanoseconds % 1_000_000_000)
    return stamp


def main():
    rclpy.init()
    node = None
    code = 0
    try:
        node = Gs130Camera()
        rclpy.spin(node)
    except KeyboardInterrupt:
        code = 0
    except SystemExit as error:
        code = error.code if isinstance(error.code, int) else 1
    except Exception as error:
        # A SIGINT delivered while the executor is inside its wait set tears the
        # context down and surfaces as an rclpy error instead of a
        # KeyboardInterrupt. That is still a normal shutdown, not a failure.
        if rclpy.ok():
            if node is not None:
                node.get_logger().error("node stopped by an unexpected error: %s" % error)
            code = 1
    finally:
        if node is not None:
            node.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
