"""Calibration: what the two cameras and the IMU are, and where they sit."""

import numpy as np

from ._enums import DistModel


class CameraIntrinsics:
    """One camera: focal lengths, principal point, distortion.

    K is the (3, 3) matrix, dist_coeffs the 8 coefficients of the model in
    dist_model. In CameraMode.RECT the frames are already rectified, so the
    coefficients are zero and K applies directly.
    """

    def __init__(self, fx, fy, cx, cy, K, dist_coeffs, dist_model):
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.K = K
        self.dist_coeffs = dist_coeffs
        self.dist_model = dist_model

    def __repr__(self):
        return (
            "CameraIntrinsics(fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f, %s)"
            % (self.fx, self.fy, self.cx, self.cy, self.dist_model.name)
        )


class ImuIntrinsics:
    """The IMU: misalignment, scale, bias and noise of both sensors.

    The noise densities are in m/s^2/sqrt(Hz) for the accelerometer and
    rad/s/sqrt(Hz) for the gyroscope, the random walks in m/s^3/sqrt(Hz) and
    rad/s^2/sqrt(Hz).
    """

    def __init__(
        self,
        accel_misalign,
        accel_scale,
        accel_bias,
        accel_noise,
        accel_random_walk,
        gyro_misalign,
        gyro_scale,
        gyro_bias,
        gyro_noise,
        gyro_random_walk,
    ):
        self.accel_misalign = accel_misalign
        self.accel_scale = accel_scale
        self.accel_bias = accel_bias
        self.accel_noise = accel_noise
        self.accel_random_walk = accel_random_walk
        self.gyro_misalign = gyro_misalign
        self.gyro_scale = gyro_scale
        self.gyro_bias = gyro_bias
        self.gyro_noise = gyro_noise
        self.gyro_random_walk = gyro_random_walk

    def __repr__(self):
        return "ImuIntrinsics(accel_noise=%.6f, gyro_noise=%.6f)" % (
            self.accel_noise,
            self.gyro_noise,
        )


class Calibration:
    """Everything the device knows about itself.

    camera_left_T and camera_right_T are (3,) translations in meters and the
    _R are (3, 3) rotations, all expressed in the reference frame the device
    picked when it initialized. install_angle is the rotating installation
    angle in degrees.
    """

    def __init__(
        self,
        imu,
        camera_left,
        camera_right,
        camera_left_R,
        camera_left_T,
        camera_right_R,
        camera_right_T,
        imu_R,
        imu_T,
        install_angle,
    ):
        self.imu = imu
        self.camera_left = camera_left
        self.camera_right = camera_right
        self.camera_left_R = camera_left_R
        self.camera_left_T = camera_left_T
        self.camera_right_R = camera_right_R
        self.camera_right_T = camera_right_T
        self.imu_R = imu_R
        self.imu_T = imu_T
        self.install_angle = install_angle

    def __repr__(self):
        return "Calibration(%s, %s, install_angle=%d)" % (
            self.camera_left,
            self.imu,
            self.install_angle,
        )


def _matrix(values, shape):
    return np.array(values, dtype=np.float64).reshape(shape)


def camera_intrinsics(raw):
    """Build a CameraIntrinsics from a gs130_camera_intrinsics_t."""
    return CameraIntrinsics(
        raw.fx,
        raw.fy,
        raw.cx,
        raw.cy,
        _matrix(raw.K, (3, 3)),
        _matrix(raw.dist_coeffs, (8,)),
        DistModel(raw.dist_model),
    )


def imu_intrinsics(raw):
    """Build an ImuIntrinsics from a gs130_imu_intrinsics_t."""
    return ImuIntrinsics(
        _matrix(raw.accel_misalign, (3, 3)),
        _matrix(raw.accel_scale, (3,)),
        _matrix(raw.accel_bias, (3,)),
        raw.accel_noise,
        raw.accel_random_walk,
        _matrix(raw.gyro_misalign, (3, 3)),
        _matrix(raw.gyro_scale, (3,)),
        _matrix(raw.gyro_bias, (3,)),
        raw.gyro_noise,
        raw.gyro_random_walk,
    )


def calibration(raw):
    """Build a Calibration from a gs130_calibration_t."""
    return Calibration(
        imu_intrinsics(raw.imu),
        camera_intrinsics(raw.camera_left),
        camera_intrinsics(raw.camera_right),
        _matrix(raw.camera_left_R, (3, 3)),
        _matrix(raw.camera_left_T, (3,)),
        _matrix(raw.camera_right_R, (3, 3)),
        _matrix(raw.camera_right_T, (3,)),
        _matrix(raw.imu_R, (3, 3)),
        _matrix(raw.imu_T, (3,)),
        raw.camera_install_angle,
    )
