"""The GS130 device.

This is the only module that calls libgs130.  ``_abi`` holds the C
declarations, ``_types`` holds what the user receives, and everything between
them lives here.
"""

from __future__ import annotations

import ctypes
import warnings

import numpy as np

from . import _abi
from ._config import to_c
from ._types import (
    Calibration,
    CameraIntrinsics,
    DistModel,
    ErrorCode,
    Image,
    ImuIntrinsics,
    ImuPacket,
    StereoLayout,
)


class GS130Error(RuntimeError):
    """A call into libgs130 failed.

    Inherits :class:`RuntimeError`, so a caller that does not care which
    failure it was can catch it the ordinary way.

    ``code`` is an :class:`ErrorCode` when the SDK reported a known code, and a
    plain ``int`` when it did not, so a newer library cannot turn a failure
    into a ``ValueError``.  ``func`` is the C function that failed.

    ``args`` stays ``(code, func)``, which is what keeps the exception
    picklable; ``str()`` renders it as ``"gs130_init() -> NOT_FOUND (3)"``.
    """

    def __init__(self, code, func=None):
        try:
            self.code = ErrorCode(code)
            self.reason = "%s (%d)" % (self.code.name, int(self.code))
        except ValueError:
            self.code = code
            self.reason = "unknown error code %s" % code
        self.func = func
        super().__init__(code, func)

    def __str__(self):
        if self.func:
            return "%s() -> %s" % (self.func, self.reason)
        return self.reason


def _check(code, func):
    """Raise :class:`GS130Error` unless the C call reported ``GS130_OK``."""
    if code != ErrorCode.OK:
        raise GS130Error(code, func)


# ---------------------------------------------------------------------------
# gs130_* structs -> the classes in _types
# ---------------------------------------------------------------------------


def _text(value):
    """Decode a C string, mapping NULL to ``None``."""
    return value.decode() if value else None


def _copy(values, shape):
    """Copy a ctypes array into an independent numpy array of ``shape``."""
    return np.ctypeslib.as_array(values).copy().reshape(shape)


def _imu_packet(raw):
    """Build an :class:`ImuPacket` from a ``gs130_imu_packet_t``."""
    return ImuPacket(
        _copy(raw.accel, (3,)),
        _copy(raw.gyro, (3,)),
        float(raw.temp),
        bool(raw.is_fsync),
        int(raw.timestamp_ns),
    )


def _camera_intrinsics(raw):
    """Build :class:`CameraIntrinsics` from a ``gs130_camera_intrinsics_t``."""
    return CameraIntrinsics(
        float(raw.fx),
        float(raw.fy),
        float(raw.cx),
        float(raw.cy),
        _copy(raw.K, (3, 3)),
        _copy(raw.dist_coeffs, (8,)),
        DistModel(raw.dist_model),
    )


def _imu_intrinsics(raw):
    """Build :class:`ImuIntrinsics` from a ``gs130_imu_intrinsics_t``."""
    return ImuIntrinsics(
        _copy(raw.accel_misalign, (3, 3)),
        _copy(raw.accel_scale, (3,)),
        _copy(raw.accel_bias, (3,)),
        float(raw.accel_noise),
        float(raw.accel_random_walk),
        _copy(raw.gyro_misalign, (3, 3)),
        _copy(raw.gyro_scale, (3,)),
        _copy(raw.gyro_bias, (3,)),
        float(raw.gyro_noise),
        float(raw.gyro_random_walk),
    )


def _calibration(raw):
    """Build :class:`Calibration` from a ``gs130_calibration_t``."""
    return Calibration(
        imu=_imu_intrinsics(raw.imu),
        camera_left=_camera_intrinsics(raw.camera_left),
        camera_right=_camera_intrinsics(raw.camera_right),
        camera_left_R=_copy(raw.camera_left_R, (3, 3)),
        camera_left_T=_copy(raw.camera_left_T, (3,)),
        camera_right_R=_copy(raw.camera_right_R, (3, 3)),
        camera_right_T=_copy(raw.camera_right_T, (3,)),
        imu_R=_copy(raw.imu_R, (3, 3)),
        imu_T=_copy(raw.imu_T, (3,)),
        install_angle=int(raw.camera_install_angle),
    )


# ---------------------------------------------------------------------------
# Device
# ---------------------------------------------------------------------------


class Device:
    """An initialized GS130 camera.

    Build it from a configuration dict (:func:`gs130.config` returns an empty
    one, :func:`gs130.preset` a filled one), then call :meth:`start` before
    reading.  Use it as a context manager so the camera is always released::

        with gs130.Device(config) as dev:
            dev.start()
            frame = dev.read_image()
    """

    # -- construction and lifecycle ---------------------------------------

    def __init__(self, config):
        """Validate ``config``, initialize the hardware, and detect the IMU."""
        # Only gs130_init reads the configuration, and it copies what it needs
        # (the tuning file path included), so this stays a local.
        c_config = to_c(config)
        self._stitched = (
            c_config.camera_config.stereo_layout != StereoLayout.NONE
        )
        self._lib = _abi.load()
        self._dev = self._lib.gs130_create()
        if not self._dev:
            raise MemoryError("gs130_create() returned NULL")

        try:
            _check(
                self._lib.gs130_init(self._dev, ctypes.byref(c_config)),
                "gs130_init",
            )
        except BaseException:
            # The handle was created but never initialized.  Release it, mark
            # the object closed, and let the original failure through.
            self._lib.gs130_destroy(self._dev)
            self._dev = None
            raise

    def _device(self):
        """Return the native handle, rejecting use after :meth:`close`."""
        if self._dev is None:
            raise RuntimeError("gs130.Device is closed")
        return self._dev

    def start(self):
        """Begin capture and return ``self``."""
        _check(self._lib.gs130_start(self._device()), "gs130_start")
        return self

    def stop(self):
        """Stop capture and wait for the background threads to exit.

        Does nothing when capture is not running; :meth:`start` may be called
        again.
        """
        self._lib.gs130_stop(self._device())

    def close(self):
        """Stop capture and release every device resource.

        Calling it again is a no-op.
        """
        device = self._dev
        if device is None:
            return
        self._dev = None
        try:
            # gs130_deinit() stops the threads itself, but stopping explicitly
            # means none can still be running should deinit fail before it gets
            # that far -- destroying a live handle is undefined behaviour.
            self._lib.gs130_stop(device)
            _check(self._lib.gs130_deinit(device), "gs130_deinit")
        finally:
            self._lib.gs130_destroy(device)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        try:
            self.close()
        except Exception:
            # A failure to close must not mask the failure that got us here.
            if exc_type is None:
                raise
            warnings.warn(
                "gs130.Device.close() failed while another exception was "
                "already propagating",
                RuntimeWarning,
                source=self,
            )
        return False

    def __del__(self):
        # Best effort only: __del__ may run during interpreter shutdown, when
        # the library or even the warnings module is already gone.  close() is
        # the reliable path; this warns that it was skipped and then tries.
        if getattr(self, "_dev", None) is not None:
            try:
                warnings.warn(
                    "unclosed gs130.Device; use close() or a with block",
                    ResourceWarning,
                    source=self,
                )
                self.close()
            except Exception:
                pass

    # -- state -------------------------------------------------------------

    @property
    def closed(self):
        """Whether the camera has been released."""
        return self._dev is None

    @property
    def stitched(self):
        """Whether :meth:`read_image` returns one stitched frame or two eyes."""
        return self._stitched

    @property
    def imu_name(self):
        """Detected IMU model, or ``None``."""
        return _text(self._lib.gs130_get_imu_name(self._device()))

    @property
    def imu_info(self):
        """Detected IMU details, including the bandwidth table, or ``None``."""
        return _text(self._lib.gs130_get_imu_info(self._device()))

    @property
    def eeprom_name(self):
        """Detected EEPROM header, or ``None``."""
        return _text(self._lib.gs130_get_eeprom_name(self._device()))

    @property
    def eeprom_info(self):
        """Detected EEPROM details, or ``None``."""
        return _text(self._lib.gs130_get_eeprom_info(self._device()))

    # -- queue depth -------------------------------------------------------

    def available_camera(self):
        """How many synchronized frame pairs are queued right now.

        ``0`` while capture is not running.
        """
        return self._lib.gs130_available_camera(self._device())

    def available_imu(self):
        """How many IMU packets are queued right now.

        ``0`` while capture is not running, or when no IMU was detected.
        """
        return self._lib.gs130_available_imu(self._device())

    # -- reads -------------------------------------------------------------

    def read_image(self):
        """Pop one synchronized frame, or ``None`` when there is none.

        Returns ``{"stitched": Image}`` when a stereo layout is configured and
        ``{"left": Image, "right": Image}`` otherwise.  Each :class:`Image`
        owns its buffer and frees it when dropped.

        Any non-OK code becomes ``None``, so a hardware fault looks the same as
        an empty queue here; this is the loop-friendly contract, and
        :meth:`available_camera` tells the two apart by reporting ``0`` while
        the stream is not running.
        """
        device = self._device()
        if self._stitched:
            raw = _abi.gs130_image_nv12_t()
            code = self._lib.gs130_get_stereo_nv12_frame(
                device, ctypes.byref(raw)
            )
            if code != ErrorCode.OK:
                return None
            return {"stitched": Image(raw)}

        left = _abi.gs130_image_nv12_t()
        right = _abi.gs130_image_nv12_t()
        code = self._lib.gs130_get_nv12_frame(
            device, ctypes.byref(left), ctypes.byref(right)
        )
        if code != ErrorCode.OK:
            return None
        return {"left": Image(left), "right": Image(right)}

    def read_imu(self):
        """Pop one IMU packet, or ``None`` when there is none.

        Also ``None`` on a device without an IMU, which the SDK reports as
        ``PARAM_ERROR`` rather than ``TIMEOUT``; check :attr:`imu_name` if that
        case needs to be told apart from an idle queue.
        """
        raw = _abi.gs130_imu_packet_t()
        code = self._lib.gs130_get_imu_packet(
            self._device(), ctypes.byref(raw)
        )
        if code != ErrorCode.OK:
            return None
        return _imu_packet(raw)

    # -- calibration -------------------------------------------------------

    def calibration(self):
        """The full stereo and IMU calibration loaded from the EEPROM."""
        raw = _abi.gs130_calibration_t()
        _check(
            self._lib.gs130_get_calibration(self._device(), ctypes.byref(raw)),
            "gs130_get_calibration",
        )
        return _calibration(raw)

    def camera_intrinsics(self, camera):
        """Intrinsics of one eye; ``camera`` is a :class:`CameraIndex`."""
        raw = _abi.gs130_camera_intrinsics_t()
        _check(
            self._lib.gs130_get_camera_intrinsics(
                self._device(), camera, ctypes.byref(raw)
            ),
            "gs130_get_camera_intrinsics",
        )
        return _camera_intrinsics(raw)

    def imu_intrinsics(self):
        """IMU intrinsics: misalignment, scale, bias, noise, random walk."""
        raw = _abi.gs130_imu_intrinsics_t()
        _check(
            self._lib.gs130_get_imu_intrinsics(self._device(), ctypes.byref(raw)),
            "gs130_get_imu_intrinsics",
        )
        return _imu_intrinsics(raw)

    def relative_R(self, from_frame, to_frame):
        """Rotation taking a point from ``from_frame`` into ``to_frame``, (3, 3)."""
        values = (ctypes.c_double * 9)()
        _check(
            self._lib.gs130_get_relative_R(
                self._device(), from_frame, to_frame, values
            ),
            "gs130_get_relative_R",
        )
        return np.ctypeslib.as_array(values).copy().reshape(3, 3)

    def relative_T(self, from_frame, to_frame):
        """Translation taking ``from_frame`` into ``to_frame``, (3,)."""
        values = (ctypes.c_double * 3)()
        _check(
            self._lib.gs130_get_relative_T(
                self._device(), from_frame, to_frame, values
            ),
            "gs130_get_relative_T",
        )
        return np.ctypeslib.as_array(values).copy()

    def convert_calibration(self, ref_frame, R, T):
        """Move the reference frame while keeping every device pose fixed.

        ``R`` is the reference rotation in row-major order (9 values, so a 3x3
        array is accepted), ``T`` the reference translation (3 values).

        Results of :meth:`calibration`, :meth:`relative_R` and
        :meth:`relative_T` follow this change.
        """
        # The C API takes bare pointers, so it cannot know how long the caller's
        # buffer is; the size has to be checked here.
        rotation = np.asarray(R, dtype=np.float64).reshape(-1)
        if rotation.size != 9:
            raise ValueError(
                "R must hold 9 values, got %d" % rotation.size
            )
        translation = np.asarray(T, dtype=np.float64).reshape(-1)
        if translation.size != 3:
            raise ValueError(
                "T must hold 3 values, got %d" % translation.size
            )
        _check(
            self._lib.gs130_convert_calibration(
                self._device(),
                ref_frame,
                (ctypes.c_double * 9)(*rotation),
                (ctypes.c_double * 3)(*translation),
            ),
            "gs130_convert_calibration",
        )
