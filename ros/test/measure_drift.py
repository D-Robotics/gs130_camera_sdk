"""Measure the timestamp offset drift of the running gs130 camera node.

The image topic is far too large to time in Python, but the IMU shares the same
offset and its messages are tiny, so the receipt latency of an IMU message
tracks how the node's mapping to the system clock behaves over time. A constant
latency means the one-time offset is holding; a growing one is drift.
"""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

DURATION_S = float(sys.argv[1]) if len(sys.argv) > 1 else 600.0


class Drift(Node):
    def __init__(self):
        super().__init__("gs130_drift")
        self.samples = []
        self.create_subscription(
            Imu,
            "/imu/data",
            self.on_imu,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=200,
            ),
        )
        self.started = time.monotonic()

    def on_imu(self, message):
        received = time.time_ns()
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        # Sample at about 1 Hz so the measurement cost stays negligible.
        if not self.samples or received - self.samples[-1][0] > 1_000_000_000:
            self.samples.append((received, received - stamp))


def main():
    rclpy.init()
    node = Drift()
    deadline = time.monotonic() + DURATION_S
    while time.monotonic() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)

    samples = node.samples
    print("samples %d over %.0f s" % (len(samples), DURATION_S))
    if len(samples) < 3:
        print("FAIL not enough samples")
        return 1

    first = samples[0][1]
    last = samples[-1][1]
    span_s = (samples[-1][0] - samples[0][0]) / 1e9
    latency_ms = [value / 1e6 for _, value in samples]
    print("latency ms: first %.2f last %.2f min %.2f max %.2f" % (
        latency_ms[0], latency_ms[-1], min(latency_ms), max(latency_ms)))
    drift_ms = (last - first) / 1e6
    print("drift over %.0f s: %.2f ms (%.3f ms/min)" % (span_s, drift_ms, drift_ms / (span_s / 60.0)))

    # Least squares slope in ms per minute.
    n = len(samples)
    xs = [(received - samples[0][0]) / 60e9 for received, _ in samples]
    ys = latency_ms
    mean_x = sum(xs) / n
    mean_y = sum(ys) / n
    numerator = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
    denominator = sum((x - mean_x) ** 2 for x in xs)
    slope = numerator / denominator if denominator else 0.0
    print("fitted slope: %.3f ms/min" % slope)
    print("VERDICT %s" % ("PASS one-time offset holds" if abs(slope) <= 1.0 else "FAIL offset drifts"))

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
