"""Publish a synthetic NV12 stereo image, to validate the TROS reuse chain.

This is a host-side bring-up probe, not part of the deliverable package.
"""

import array
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

WIDTH = 1280
HEIGHT = 720


def make_nv12():
    y = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)
    x = np.arange(WIDTH, dtype=np.int32)[None, :]
    rows = np.arange(HEIGHT, dtype=np.int32)[:, None]
    y[:] = ((x // 8 + rows // 8) % 2 * 200 + 20).astype(np.uint8)
    y[:, : WIDTH // 2] = ((rows * 255 // HEIGHT) % 200 + 30).astype(np.uint8)
    uv = np.full((HEIGHT // 2, WIDTH), 128, dtype=np.uint8)
    return np.concatenate([y.reshape(-1), uv.reshape(-1)])


class Probe(Node):
    def __init__(self):
        super().__init__("gs130_nv12_probe")
        self.publisher = self.create_publisher(
            Image,
            "/image_combine_raw",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.data = make_nv12()
        self.count = 0
        self.previous = time.perf_counter()
        rate = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
        self.create_timer(1.0 / rate, self.publish)

    def publish(self):
        enter = time.perf_counter()
        gap = enter - self.previous
        self.previous = enter
        message = Image()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "camera"
        message.height = HEIGHT
        message.width = WIDTH
        message.encoding = "nv12"
        message.is_bigendian = 0
        message.step = WIDTH
        build = time.perf_counter()
        buffer = array.array("B")
        buffer.frombytes(self.data.tobytes())
        message.data = buffer
        filled = time.perf_counter()
        self.publisher.publish(message)
        sent = time.perf_counter()
        self.count += 1
        if self.count % 30 == 0:
            self.get_logger().info(
                "frame %d: gap %.1f ms | build %.2f | fill %.2f | publish %.2f"
                % (
                    self.count,
                    gap * 1000.0,
                    (build - enter) * 1000.0,
                    (filled - build) * 1000.0,
                    (sent - filled) * 1000.0,
                )
            )


def main():
    rclpy.init()
    node = Probe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
