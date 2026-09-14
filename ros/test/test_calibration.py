"""Unit tests for the parts of gs130_ros that need no camera.

Run on the board with the workspace sourced:

    python3 -m unittest discover -s ros/test -v
"""

import unittest

import numpy as np
from builtin_interfaces.msg import Time

import gs130
from gs130_ros.calibration import camera_info, distortion_model, transform
from gs130_ros.camera_node import image_message


class Inner:
    """Stands in for a gs130 CameraIntrinsics."""

    def __init__(self, dist_coeffs, dist_model):
        self.fx = 386.85
        self.fy = 387.12
        self.cx = 304.62
        self.cy = 245.06
        self.K = np.array([[386.85, 0.0, 304.62],
                           [0.0, 387.12, 245.06],
                           [0.0, 0.0, 1.0]])
        self.dist_coeffs = np.array(dist_coeffs, dtype=np.float64)
        self.dist_model = dist_model


class DistortionTests(unittest.TestCase):
    def test_rectified_camera_reports_plumb_bob(self):
        model, values = distortion_model(np.zeros(8), gs130.DistModel.FISHEYE)
        self.assertEqual(model, "plumb_bob")
        self.assertEqual(values, [0.0] * 5)

    def test_fisheye_keeps_four_equidistant_coefficients(self):
        model, values = distortion_model(
            [-0.025173, 0.012189, -0.013019, 0.002486, 0.0, 0.0, 0.0, 0.0],
            gs130.DistModel.FISHEYE,
        )
        self.assertEqual(model, "equidistant")
        self.assertEqual(len(values), 4)
        self.assertAlmostEqual(values[0], -0.025173, places=6)

    def test_pinhole_keeps_eight_coefficients(self):
        model, values = distortion_model(
            [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8],
            gs130.DistModel.PINHOLE,
        )
        self.assertEqual(model, "rational_polynomial")
        self.assertEqual(len(values), 8)


class CameraInfoTests(unittest.TestCase):
    def test_fields_come_from_the_calibration(self):
        message = camera_info(Inner(np.zeros(8), gs130.DistModel.FISHEYE),
                              640, 480, "camera_left")
        self.assertEqual(message.width, 640)
        self.assertEqual(message.height, 480)
        self.assertEqual(message.header.frame_id, "camera_left")
        self.assertEqual(message.distortion_model, "plumb_bob")
        self.assertAlmostEqual(message.k[0], 386.85)
        self.assertAlmostEqual(message.k[4], 387.12)
        self.assertAlmostEqual(message.k[2], 304.62)
        self.assertAlmostEqual(message.k[5], 245.06)
        self.assertEqual([float(value) for value in message.r],
                         [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        self.assertAlmostEqual(message.p[0], 386.85)
        self.assertAlmostEqual(message.p[2], 304.62)
        self.assertAlmostEqual(message.p[5], 387.12)
        self.assertAlmostEqual(message.p[6], 245.06)


class ImageMessageTests(unittest.TestCase):
    def test_nv12_uses_the_real_height(self):
        frame = np.zeros((480 * 3 // 2, 640), dtype=np.uint8)
        stamp = Time(sec=7, nanosec=5)
        message = image_message(frame, "camera_left", stamp)
        self.assertEqual(message.encoding, "nv12")
        self.assertEqual(message.width, 640)
        self.assertEqual(message.height, 480)
        self.assertEqual(message.step, 640)
        self.assertEqual(message.is_bigendian, 0)
        self.assertEqual(len(message.data), 640 * 480 * 3 // 2)
        self.assertEqual(message.header.frame_id, "camera_left")
        self.assertEqual(message.header.stamp.sec, 7)

    def test_stitched_geometry_doubles_the_width_only(self):
        frame = np.zeros((480 * 3 // 2, 1280), dtype=np.uint8)
        message = image_message(frame, "camera", Time())
        self.assertEqual(message.width, 1280)
        self.assertEqual(message.height, 480)
        self.assertEqual(message.step, 1280)
        self.assertEqual(len(message.data), 1280 * 480 * 3 // 2)


def _rotate(x, y, z, w, vector):
    """Rotate a vector by a quaternion, to check the mapping independently."""
    cross = (y * vector[2] - z * vector[1],
             z * vector[0] - x * vector[2],
             x * vector[1] - y * vector[0])
    return (
        vector[0] + 2.0 * (w * cross[0] + (y * cross[2] - z * cross[1])),
        vector[1] + 2.0 * (w * cross[1] + (z * cross[0] - x * cross[2])),
        vector[2] + 2.0 * (w * cross[2] + (x * cross[1] - y * cross[0])),
    )


class FakeDevice:
    def __init__(self, rotation, translation):
        self.rotation = np.array(rotation, dtype=np.float64)
        self.translation = np.array(translation, dtype=np.float64)

    def relative_R(self, from_frame, to_frame):
        return self.rotation

    def relative_T(self, from_frame, to_frame):
        return self.translation


class TransformTests(unittest.TestCase):
    def test_identity_rotation_becomes_a_unit_quaternion(self):
        device = FakeDevice(np.eye(3), [0.070316, 0.0, 0.0])
        message = transform(device, gs130.ReferenceFrame.CAMERA_RIGHT,
                            gs130.ReferenceFrame.CAMERA_LEFT,
                            "camera_left", "camera_right")
        self.assertEqual(message.header.frame_id, "camera_left")
        self.assertEqual(message.child_frame_id, "camera_right")
        self.assertAlmostEqual(message.transform.translation.x, 0.070316, places=6)
        self.assertAlmostEqual(message.transform.rotation.w, 1.0, places=9)
        self.assertAlmostEqual(message.transform.rotation.x, 0.0, places=9)

    def test_half_turn_uses_the_off_diagonal_branch(self):
        device = FakeDevice([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]],
                            [0.0, 0.0, 0.0])
        message = transform(device, gs130.ReferenceFrame.IMU,
                            gs130.ReferenceFrame.CAMERA_LEFT,
                            "camera_left", "imu_link")
        rotation = message.transform.rotation
        self.assertAlmostEqual(rotation.w, 0.0, places=9)
        self.assertAlmostEqual(abs(rotation.x), 1.0, places=9)

    def test_quaternion_reproduces_the_rotation(self):
        import math

        angle = 0.7
        matrix = np.array([[math.cos(angle), -math.sin(angle), 0.0],
                           [math.sin(angle), math.cos(angle), 0.0],
                           [0.0, 0.0, 1.0]])
        device = FakeDevice(matrix, [0.0, 0.0, 0.0])
        message = transform(device, gs130.ReferenceFrame.CAMERA_RIGHT,
                            gs130.ReferenceFrame.CAMERA_LEFT,
                            "camera_left", "camera_right")
        rotation = message.transform.rotation
        x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
        for vector in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)):
            got = _rotate(x, y, z, w, vector)
            expected = matrix @ np.array(vector)
            for value, want in zip(got, expected):
                self.assertAlmostEqual(value, float(want), places=6)


if __name__ == "__main__":
    unittest.main()
