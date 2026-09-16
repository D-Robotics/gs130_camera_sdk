"""Type stubs for the gs130 binding.

The configuration is a plain nested dict because it doubles as the editable
form a user adjusts before handing it to :class:`Device`.  The ``TypedDict``
sections below describe the keys ``config()`` creates and ``to_c()`` accepts.
"""

from enum import IntEnum
from typing import Any, TypedDict

import numpy as np

__version__: str | None
__all__: list[str]


# ---------------------------------------------------------------------------
# Enums
# ---------------------------------------------------------------------------


class ErrorCode(IntEnum):
    OK: int
    PARAM_ERROR: int
    UNSUPPORTED: int
    NOT_FOUND: int
    HW_ERROR: int
    TIMEOUT: int
    THREAD_CLOSED: int


class CameraMode(IntEnum):
    RAW: int
    RESIZE: int
    RECT: int


class CameraIndex(IntEnum):
    RIGHT: int
    LEFT: int


class StereoLayout(IntEnum):
    NONE: int
    LEFT_RIGHT: int
    RIGHT_LEFT: int
    TOP_BOTTOM: int
    BOTTOM_TOP: int


class FifoMode(IntEnum):
    DROP_NEW: int
    DROP_OLD: int


class DistModel(IntEnum):
    PINHOLE: int
    FISHEYE: int


class ReferenceFrame(IntEnum):
    CAMERA_RIGHT: int
    CAMERA_LEFT: int
    IMU: int


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------


class CameraConfig(TypedDict):
    bus: list[int]
    left_addr: int
    right_addr: int
    sensor_width: int
    sensor_height: int
    fps: int
    line_length: int
    frame_length: int
    tuning_file: str | None
    output_width: int
    output_height: int
    mode: CameraMode
    stereo_layout: StereoLayout
    bus_mipi_rx: dict[int, int]
    bus_reset_gpio: dict[int, int]
    fsync_camera: CameraIndex


class ImuConfig(TypedDict):
    bus: list[int]
    addr: int
    odr_hz: int
    accel_fsr_g: int
    gyro_fsr_dps: int
    accel_bw_sel: int
    gyro_bw_sel: int


class EepromConfig(TypedDict):
    bus: list[int]
    addr: int


class FifoConfig(TypedDict):
    depth: int
    mode: FifoMode


class Config(TypedDict):
    camera_config: CameraConfig
    imu_config: ImuConfig
    eeprom_config: EepromConfig
    camera_fifo: FifoConfig
    imu_fifo: FifoConfig


def config() -> Config:
    """An empty configuration: every field unset, ready to be filled in."""
    ...


def preset(
    device: str,
    mode: CameraMode,
    width: int,
    height: int,
    fps: int,
    odr: int,
) -> Config:
    """A filled configuration for known hardware on the loaded platform."""
    ...


# ---------------------------------------------------------------------------
# Values the SDK hands back
# ---------------------------------------------------------------------------


class Image(np.ndarray):
    @property
    def timestamp_ns(self) -> int: ...
    @property
    def width(self) -> int: ...
    @property
    def height(self) -> int: ...
    def y_plane(self) -> Image: ...
    def uv_plane(self) -> Image: ...


class ImuPacket:
    accel: np.ndarray
    gyro: np.ndarray
    temp: float
    is_fsync: bool
    timestamp_ns: int


class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    K: np.ndarray
    dist_coeffs: np.ndarray
    dist_model: DistModel


class ImuIntrinsics:
    accel_misalign: np.ndarray
    accel_scale: np.ndarray
    accel_bias: np.ndarray
    accel_noise: float
    accel_random_walk: float
    gyro_misalign: np.ndarray
    gyro_scale: np.ndarray
    gyro_bias: np.ndarray
    gyro_noise: float
    gyro_random_walk: float


class Calibration:
    imu: ImuIntrinsics
    camera_left: CameraIntrinsics
    camera_right: CameraIntrinsics
    camera_left_R: np.ndarray
    camera_left_T: np.ndarray
    camera_right_R: np.ndarray
    camera_right_T: np.ndarray
    imu_R: np.ndarray
    imu_T: np.ndarray
    install_angle: int


# ---------------------------------------------------------------------------
# Errors and the device
# ---------------------------------------------------------------------------


class GS130Error(RuntimeError):
    code: ErrorCode | int
    reason: str
    func: str | None
    def __init__(self, code: int, func: str | None = ...) -> None: ...


class Device:
    def __init__(self, config: Config) -> None: ...
    def start(self) -> Device: ...
    def stop(self) -> None: ...
    def close(self) -> None: ...
    def __enter__(self) -> Device: ...
    def __exit__(self, exc_type: Any, exc_value: Any, traceback: Any) -> bool: ...
    @property
    def closed(self) -> bool: ...
    @property
    def stitched(self) -> bool: ...
    @property
    def imu_name(self) -> str | None: ...
    @property
    def imu_info(self) -> str | None: ...
    @property
    def eeprom_name(self) -> str | None: ...
    @property
    def eeprom_info(self) -> str | None: ...
    def available_camera(self) -> int: ...
    def available_imu(self) -> int: ...
    def read_image(self) -> dict[str, Image] | None: ...
    def read_imu(self) -> ImuPacket | None: ...
    def calibration(self) -> Calibration: ...
    def camera_intrinsics(self, camera: CameraIndex) -> CameraIntrinsics: ...
    def imu_intrinsics(self) -> ImuIntrinsics: ...
    def relative_R(
        self, from_frame: ReferenceFrame, to_frame: ReferenceFrame
    ) -> np.ndarray: ...
    def relative_T(
        self, from_frame: ReferenceFrame, to_frame: ReferenceFrame
    ) -> np.ndarray: ...
    def convert_calibration(
        self,
        ref_frame: ReferenceFrame,
        R: Any,
        T: Any,
    ) -> None: ...


def package_version() -> str | None: ...
def library_version() -> str | None: ...
def library_platform() -> str | None: ...
