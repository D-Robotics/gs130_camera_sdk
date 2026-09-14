"""The high-level GS130 device."""

import ctypes

import numpy as np

from . import _abi, _runtime
from ._config import Config, to_c
from ._enums import CameraIndex, ErrorCode, ReferenceFrame, StereoLayout
from ._error import GS130Error, check
from ._types import (
    Image,
    ImuPacket,
    calibration as make_calibration,
    camera_intrinsics as make_camera_intrinsics,
    imu_intrinsics as make_imu_intrinsics,
)


class Device:
    """An initialized GS130 camera; call :meth:`start` before reading."""

    def __init__(self, config):
        if not isinstance(config, Config):
            raise TypeError("Device() expects Config, got %s" % type(config).__name__)
        self._c_config = to_c(config)
        self._stitched = (
            self._c_config.camera_config.stereo_layout != StereoLayout.NONE
        )
        self._lib = _runtime.load()
        self._dev = None
        self._started = False
        self._closed = False

        create = self._function("gs130_create")
        self._dev = create()
        if not self._dev:
            self._closed = True
            raise GS130Error(ErrorCode.HW_ERROR, "gs130_create")
        initialized = False
        try:
            check(
                self._call("gs130_init", self._dev, ctypes.byref(self._c_config)),
                "gs130_init",
            )
            initialized = True
            self._has_imu = self.imu_name is not None
        except BaseException:
            if initialized:
                self._call("gs130_deinit", self._dev)
            self._function("gs130_destroy")(self._dev)
            self._dev = None
            self._closed = True
            raise

    def _function(self, name):
        return _runtime.function(self._lib, name)

    def _call(self, name, *args):
        return self._function(name)(*args)

    @staticmethod
    def _text(value):
        return value.decode() if value else None

    def _ensure_open(self, func):
        if self._closed:
            raise GS130Error(ErrorCode.THREAD_CLOSED, func)

    def _ensure_streaming(self, func):
        if self._closed or not self._started:
            raise GS130Error(ErrorCode.THREAD_CLOSED, func)

    @property
    def running(self):
        return self._started and not self._closed

    @property
    def closed(self):
        return self._closed

    @property
    def stitched(self):
        return self._stitched

    @property
    def imu_name(self):
        self._ensure_open("gs130_get_imu_name")
        return self._text(self._call("gs130_get_imu_name", self._dev))

    @property
    def imu_info(self):
        self._ensure_open("gs130_get_imu_info")
        return self._text(self._call("gs130_get_imu_info", self._dev))

    @property
    def eeprom_name(self):
        self._ensure_open("gs130_get_eeprom_name")
        return self._text(self._call("gs130_get_eeprom_name", self._dev))

    @property
    def eeprom_info(self):
        self._ensure_open("gs130_get_eeprom_info")
        return self._text(self._call("gs130_get_eeprom_info", self._dev))

    def start(self):
        self._ensure_open("gs130_start")
        check(self._call("gs130_start", self._dev), "gs130_start")
        self._started = True
        return self

    def stop(self):
        if self._closed or not self._started:
            return
        self._call("gs130_stop", self._dev)
        self._started = False

    def close(self):
        if self._closed:
            return
        device = self._dev
        self._started = False
        self._dev = None
        self._closed = True
        try:
            code = self._call("gs130_deinit", device)
        finally:
            self._call("gs130_destroy", device)
        check(code, "gs130_deinit")

    def available_camera(self):
        self._ensure_open("gs130_available_camera")
        return self._call("gs130_available_camera", self._dev)

    def available_imu(self):
        self._ensure_open("gs130_available_imu")
        return self._call("gs130_available_imu", self._dev)

    def read_image(self):
        self._ensure_streaming("read_image")
        if self.stitched:
            raw = _abi.ImageNV12()
            code = self._call(
                "gs130_get_stereo_nv12_frame", self._dev, ctypes.byref(raw)
            )
            if code == ErrorCode.TIMEOUT:
                return None
            check(code, "gs130_get_stereo_nv12_frame")
            return {"stitched": Image(raw)}

        left, right = _abi.ImageNV12(), _abi.ImageNV12()
        code = self._call(
            "gs130_get_nv12_frame",
            self._dev,
            ctypes.byref(left),
            ctypes.byref(right),
        )
        if code == ErrorCode.TIMEOUT:
            return None
        check(code, "gs130_get_nv12_frame")
        return {"left": Image(left), "right": Image(right)}

    def read_imu(self):
        self._ensure_streaming("read_imu")
        if not self._has_imu:
            return None
        raw = _abi.ImuPacket()
        code = self._call("gs130_get_imu_packet", self._dev, ctypes.byref(raw))
        if code == ErrorCode.TIMEOUT:
            return None
        check(code, "gs130_get_imu_packet")
        return ImuPacket(
            np.ctypeslib.as_array(raw.accel).copy(),
            np.ctypeslib.as_array(raw.gyro).copy(),
            float(raw.temp),
            bool(raw.is_fsync),
            int(raw.timestamp_ns),
        )

    def calibration(self):
        self._ensure_open("gs130_get_calibration")
        raw = _abi.Calibration()
        check(
            self._call("gs130_get_calibration", self._dev, ctypes.byref(raw)),
            "gs130_get_calibration",
        )
        return make_calibration(raw)

    def camera_intrinsics(self, camera):
        self._ensure_open("gs130_get_camera_intrinsics")
        raw = _abi.CameraIntrinsics()
        check(
            self._call(
                "gs130_get_camera_intrinsics",
                self._dev,
                CameraIndex(camera),
                ctypes.byref(raw),
            ),
            "gs130_get_camera_intrinsics",
        )
        return make_camera_intrinsics(raw)

    def imu_intrinsics(self):
        self._ensure_open("gs130_get_imu_intrinsics")
        raw = _abi.ImuIntrinsics()
        check(
            self._call("gs130_get_imu_intrinsics", self._dev, ctypes.byref(raw)),
            "gs130_get_imu_intrinsics",
        )
        return make_imu_intrinsics(raw)

    def relative_R(self, from_frame, to_frame):
        return self._relative("gs130_get_relative_R", from_frame, to_frame, 9)

    def relative_T(self, from_frame, to_frame):
        return self._relative("gs130_get_relative_T", from_frame, to_frame, 3)

    def _relative(self, name, from_frame, to_frame, size):
        self._ensure_open(name)
        values = (ctypes.c_double * size)()
        check(
            self._call(
                name,
                self._dev,
                ReferenceFrame(from_frame),
                ReferenceFrame(to_frame),
                values,
            ),
            name,
        )
        result = np.ctypeslib.as_array(values).copy()
        return result.reshape(3, 3) if size == 9 else result

    def convert_calibration(self, ref_frame, R, T):
        self._ensure_open("gs130_convert_calibration")
        rotation = np.asarray(R, dtype=np.float64).reshape(9)
        translation = np.asarray(T, dtype=np.float64).reshape(3)
        check(
            self._call(
                "gs130_convert_calibration",
                self._dev,
                ReferenceFrame(ref_frame),
                (ctypes.c_double * 9)(*rotation),
                (ctypes.c_double * 3)(*translation),
            ),
            "gs130_convert_calibration",
        )

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        try:
            self.close()
        except Exception:
            if exc_type is None:
                raise
        return False

    def __del__(self):
        if not getattr(self, "_closed", True):
            try:
                self.close()
            except Exception:
                pass
