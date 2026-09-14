"""The camera and the IMU."""

import ctypes

import numpy as np

from .._internal import _abi, _lib
from ._calibration import calibration as _calibration
from ._calibration import camera_intrinsics as _camera_intrinsics
from ._calibration import imu_intrinsics as _imu_intrinsics
from ._config import Config, _to_c
from ._enums import CameraIndex, ErrorCode, ReferenceFrame, StereoLayout
from ._error import GS130Error, check
from ._image import image as _wrap


class ImuPacket:
    """One IMU sample.

    accel is in m/s^2, gyro in rad/s, temp in degC and timestamp_ns in
    nanoseconds on the camera clock. is_fsync marks the last packet of a
    frame's batch.
    """

    def __init__(self, accel, gyro, temp, is_fsync, timestamp_ns):
        self.accel = accel
        self.gyro = gyro
        self.temp = temp
        self.is_fsync = is_fsync
        self.timestamp_ns = timestamp_ns

    def __repr__(self):
        return (
            "ImuPacket(accel=%s, gyro=%s, temp=%.2f, is_fsync=%s, "
            "timestamp_ns=%d)"
            % (
                self.accel.tolist(),
                self.gyro.tolist(),
                self.temp,
                self.is_fsync,
                self.timestamp_ns,
            )
        )


class Device:
    """The camera and the IMU.

        with gs130.Device(config) as dev:
            dev.start()
            images = dev.read_image()
            left = images["left"]
    """

    def __init__(self, config):
        """Initialize the device from config. Streaming does not start yet.

        :param config: a gs130.Config.
        :raises TypeError: config is not a Config.
        :raises GS130Error: the library could not initialize the device, for
            example a rectified mode without calibration data.
        """
        if not isinstance(config, Config):
            raise TypeError(
                "Device() takes a gs130.Config, got %s" % type(config).__name__
            )
        self._config = config
        self._c_config = _to_c(config)
        self._lib = _lib.load()
        self._dev = self._lib.gs130_create()
        if not self._dev:
            raise GS130Error(ErrorCode.HW_ERROR, "gs130_create")
        self._started = False
        self._closed = False
        code = self._lib.gs130_init(self._dev, ctypes.byref(self._c_config))
        if code != ErrorCode.OK:
            self._lib.gs130_destroy(self._dev)
            self._dev = None
            self._closed = True
            raise GS130Error(code, "gs130_init")
        self._has_imu = self._text(self._lib.gs130_get_imu_name(self._dev))

    @staticmethod
    def _text(value):
        return value.decode() if value else None

    def _ensure_open(self):
        if self._closed:
            raise GS130Error(ErrorCode.THREAD_CLOSED, "_ensure_open")

    def _ensure_streaming(self, func):
        if self._closed or not self._started:
            raise GS130Error(ErrorCode.THREAD_CLOSED, func)

    @property
    def running(self):
        """True while streaming."""
        return self._started and not self._closed

    @property
    def closed(self):
        """True once the device has been released."""
        return self._closed

    @property
    def stitched(self):
        """True when one stitched image comes out instead of two."""
        return self._config.camera_config.stereo_layout != StereoLayout.NONE

    @property
    def imu_name(self):
        """IMU model name, or None when there is no IMU."""
        self._ensure_open()
        return self._text(self._lib.gs130_get_imu_name(self._dev))

    @property
    def imu_info(self):
        """IMU details, including the rates and bandwidths it supports."""
        self._ensure_open()
        return self._text(self._lib.gs130_get_imu_info(self._dev))

    @property
    def eeprom_name(self):
        """The EEPROM header string, or None when there is no EEPROM."""
        self._ensure_open()
        return self._text(self._lib.gs130_get_eeprom_name(self._dev))

    @property
    def eeprom_info(self):
        """EEPROM details: manufacturer, calibration version and model."""
        self._ensure_open()
        return self._text(self._lib.gs130_get_eeprom_info(self._dev))

    def start(self):
        """Start streaming. Waiting for the IMU handshake happens here."""
        self._ensure_open()
        check(self._lib.gs130_start(self._dev), "gs130_start")
        self._started = True
        return self

    def stop(self):
        """Stop streaming. Does nothing when it is not running."""
        if self._closed or not self._started:
            return
        self._lib.gs130_stop(self._dev)
        self._started = False

    def close(self):
        """Release the device. Calling it twice is fine."""
        if self._closed:
            return
        self._started = False
        device, self._dev = self._dev, None
        self._closed = True
        code = self._lib.gs130_deinit(device)
        self._lib.gs130_destroy(device)
        check(code, "gs130_deinit")

    def available_camera(self):
        """How many stereo frames are waiting. A faulted device reports 0."""
        self._ensure_open()
        return self._lib.gs130_available_camera(self._dev)

    def available_imu(self):
        """How many IMU packets are waiting. A faulted device reports 0."""
        self._ensure_open()
        return self._lib.gs130_available_imu(self._dev)

    def read_image(self):
        """One frame if one is ready, None when the queue is empty.

        The call never waits. A fault is raised as GS130Error, and a faulted
        device stays faulted: close() it and build a new Device.

        :returns: {"left": image, "right": image}, or {"stitched": image} when
            the configuration asks for a stitched output. Each image is
            (height * 3 // 2, width) uint8 NV12 carrying timestamp_ns; views of
            it keep the frame alive and there is nothing to release.
        :raises GS130Error: the device failed, or streaming was not started.
        """
        self._ensure_streaming("read_image")
        if self.stitched:
            image = _abi.gs130_image_nv12_t()
            code = self._lib.gs130_get_stereo_nv12_frame(
                self._dev, ctypes.byref(image)
            )
            if code == ErrorCode.TIMEOUT:
                return None
            check(code, "gs130_get_stereo_nv12_frame")
            return {"stitched": _wrap(image)}
        left = _abi.gs130_image_nv12_t()
        right = _abi.gs130_image_nv12_t()
        code = self._lib.gs130_get_nv12_frame(
            self._dev, ctypes.byref(left), ctypes.byref(right)
        )
        if code == ErrorCode.TIMEOUT:
            return None
        check(code, "gs130_get_nv12_frame")
        return {"left": _wrap(left), "right": _wrap(right)}

    def read_imu(self):
        """One IMU packet if one is ready, None when the queue is empty.

        Packets are published once per camera frame, so the newest packet can
        be one frame behind the newest image.

        :raises GS130Error: the device failed, or streaming was not started.
        """
        self._ensure_streaming("read_imu")
        if not self._has_imu:
            return None
        packet = _abi.gs130_imu_packet_t()
        code = self._lib.gs130_get_imu_packet(self._dev, ctypes.byref(packet))
        if code == ErrorCode.TIMEOUT:
            return None
        check(code, "gs130_get_imu_packet")
        return ImuPacket(
            np.frombuffer(packet.accel, dtype=np.float32).copy(),
            np.frombuffer(packet.gyro, dtype=np.float32).copy(),
            packet.temp,
            packet.is_fsync,
            packet.timestamp_ns,
        )

    def calibration(self):
        """The full calibration: both cameras, the IMU and the extrinsics."""
        self._ensure_open()
        raw = _abi.gs130_calibration_t()
        check(
            self._lib.gs130_get_calibration(self._dev, ctypes.byref(raw)),
            "gs130_get_calibration",
        )
        return _calibration(raw)

    def camera_intrinsics(self, camera):
        """The intrinsics of one camera.

        :param camera: CameraIndex.LEFT or CameraIndex.RIGHT.
        :raises ValueError: the value is neither of them.
        """
        self._ensure_open()
        camera = CameraIndex(camera)
        raw = _abi.gs130_camera_intrinsics_t()
        check(
            self._lib.gs130_get_camera_intrinsics(
                self._dev, camera, ctypes.byref(raw)
            ),
            "gs130_get_camera_intrinsics",
        )
        return _camera_intrinsics(raw)

    def imu_intrinsics(self):
        """The intrinsics of the IMU."""
        self._ensure_open()
        raw = _abi.gs130_imu_intrinsics_t()
        check(
            self._lib.gs130_get_imu_intrinsics(self._dev, ctypes.byref(raw)),
            "gs130_get_imu_intrinsics",
        )
        return _imu_intrinsics(raw)

    def relative_R(self, from_frame, to_frame):
        """The (3, 3) rotation that takes from_frame to to_frame.

        :param from_frame: a ReferenceFrame.
        :param to_frame: a ReferenceFrame.
        """
        return self._relative("gs130_get_relative_R", from_frame, to_frame, 9)

    def relative_T(self, from_frame, to_frame):
        """The (3,) translation in meters from from_frame to to_frame."""
        return self._relative("gs130_get_relative_T", from_frame, to_frame, 3)

    def _relative(self, func, from_frame, to_frame, size):
        self._ensure_open()
        from_frame = ReferenceFrame(from_frame)
        to_frame = ReferenceFrame(to_frame)
        out = (ctypes.c_double * size)()
        check(
            getattr(self._lib, func)(self._dev, from_frame, to_frame, out),
            func,
        )
        values = np.frombuffer(out, dtype=np.float64).copy()
        return values.reshape(3, 3) if size == 9 else values

    def convert_calibration(self, ref_frame, R, T):
        """Make ref_frame the reference and place it at R, T.

        The device keeps the new reference frame, so later calibration() and
        relative_*() calls report it.

        :param ref_frame: a ReferenceFrame.
        :param R: the (3, 3) rotation of ref_frame.
        :param T: the (3,) translation of ref_frame in meters.
        """
        self._ensure_open()
        ref_frame = ReferenceFrame(ref_frame)
        rotation = np.asarray(R, dtype=np.float64).reshape(9)
        translation = np.asarray(T, dtype=np.float64).reshape(3)
        check(
            self._lib.gs130_convert_calibration(
                self._dev,
                ref_frame,
                (ctypes.c_double * 9)(*rotation),
                (ctypes.c_double * 3)(*translation),
            ),
            "gs130_convert_calibration",
        )

    def __enter__(self):
        return self

    def __exit__(self, kind, value, traceback):
        try:
            self.close()
        except Exception:
            if kind is None:
                raise
        return False

    def __del__(self):
        if not getattr(self, "_closed", True):
            try:
                self.close()
            except Exception:
                pass
