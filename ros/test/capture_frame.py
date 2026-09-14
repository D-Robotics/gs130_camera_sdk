"""Capture one stitched frame from the gs130 ROS node and prove it decodes.

Checks the message contract, decodes NV12 to BGR, splits the two eyes and
writes PNGs so the result can be inspected by a human.
"""

import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

OUT = "/tmp/probe/capture"
QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class Capture(Node):
    def __init__(self):
        super().__init__("gs130_capture")
        self.frame = None
        self.info = None
        self.create_subscription(Image, "/image_combine_raw", self.on_image, QOS)
        self.create_subscription(
            CameraInfo,
            "/image_left/camera_info",
            self.on_info,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )

    def on_image(self, message):
        self.frame = message

    def on_info(self, message):
        self.info = message


def main():
    rclpy.init()
    node = Capture()
    for _ in range(600):
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.frame is not None and node.info is not None:
            break

    if node.frame is None:
        print("FAIL no image received")
        return 1

    message = node.frame
    height = int(message.height)
    width = int(message.width)
    expected = width * height * 3 // 2
    print("topic          /image_combine_raw")
    print("encoding       %s" % message.encoding)
    print("width          %d" % width)
    print("height         %d   (real image height)" % height)
    print("step           %d" % message.step)
    print("data length    %d   (expected %d)" % (len(message.data), expected))
    print("frame_id       %s" % message.header.frame_id)
    print("stamp          %d.%09d" % (message.header.stamp.sec, message.header.stamp.nanosec))

    ok = (message.encoding == "nv12" and message.step == width
          and len(message.data) == expected and height * 3 // 2 == height * 3 // 2)
    print("contract       %s" % ("OK" if ok else "MISMATCH"))

    buffer = np.frombuffer(bytes(message.data), dtype=np.uint8)
    nv12 = buffer.reshape(height * 3 // 2, width)
    bgr = cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)
    cv2.imwrite(OUT + "_stitched.png", bgr)
    half = width // 2
    left = bgr[:, :half]
    right = bgr[:, half:]
    cv2.imwrite(OUT + "_left.png", left)
    cv2.imwrite(OUT + "_right.png", right)
    print("stitched png   %s_stitched.png  shape=%s" % (OUT, bgr.shape))
    print("left png       %s_left.png      mean=%.1f std=%.1f" % (OUT, left.mean(), left.std()))
    print("right png      %s_right.png     mean=%.1f std=%.1f" % (OUT, right.mean(), right.std()))
    # Two eyes of the same scene differ where parallax moves an edge, so compare
    # the images rather than their average brightness, which tracks the lighting.
    difference = float(np.abs(left.astype(np.int16) - right.astype(np.int16)).mean())
    print("eye difference %.2f (mean absolute pixel difference)" % difference)
    print("eyes differ    %s" % (difference > 2.0))

    if node.info is not None:
        info = node.info
        print("camera_info    %dx%d model=%s" % (info.width, info.height, info.distortion_model))
        print("  K            fx=%.2f fy=%.2f cx=%.2f cy=%.2f" % (info.k[0], info.k[4], info.k[2], info.k[5]))
        print("  D            %s" % [round(value, 6) for value in info.d])
        print("  P            [%.2f, %.2f, %.2f, %.2f]" % (info.p[0], info.p[2], info.p[5], info.p[6]))
        print("  R            %s" % [round(value, 3) for value in info.r])

    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
