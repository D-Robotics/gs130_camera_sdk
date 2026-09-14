"""Python configuration objects and C conversion."""

from dataclasses import dataclass, field

from . import _abi
from ._enums import CameraIndex, CameraMode, FifoMode, StereoLayout


@dataclass
class FifoConfig:
    depth: int = 0
    mode: FifoMode = FifoMode.DROP_NEW


@dataclass
class CameraConfig:
    bus: list = field(default_factory=list)
    left_addr: int = 0
    right_addr: int = 0
    sensor_width: int = 0
    sensor_height: int = 0
    fps: int = 0
    line_length: int = 0
    frame_length: int = 0
    tuning_file: str | bytes | None = None
    output_width: int = 0
    output_height: int = 0
    mode: CameraMode = CameraMode.RAW
    stereo_layout: StereoLayout = StereoLayout.NONE
    bus_mipi_rx: dict = field(default_factory=dict)
    bus_reset_gpio: dict = field(default_factory=dict)
    fsync_camera: CameraIndex = CameraIndex.RIGHT


@dataclass
class ImuConfig:
    bus: list = field(default_factory=list)
    addr: int = 0
    odr_hz: int = 0
    accel_fsr_g: int = 0
    gyro_fsr_dps: int = 0
    accel_bw_sel: int = 0
    gyro_bw_sel: int = 0


@dataclass
class EepromConfig:
    bus: list = field(default_factory=list)
    addr: int = 0


@dataclass
class Config:
    camera_config: CameraConfig = field(default_factory=CameraConfig)
    imu_config: ImuConfig = field(default_factory=ImuConfig)
    eeprom_config: EepromConfig = field(default_factory=EepromConfig)
    camera_fifo: FifoConfig = field(default_factory=FifoConfig)
    imu_fifo: FifoConfig = field(default_factory=FifoConfig)

    @classmethod
    def preset(cls, platform, device, mode, width, height, fps, odr):
        """Build a supported hardware preset.

        RAW output width and height must equal the 1088 x 1280 sensor size.
        """
        if platform != "RDKX5" or device not in {"GS130WI", "GS130W"}:
            raise ValueError(
                "unsupported platform/device: %s %s" % (platform, device)
            )
        mode = CameraMode(mode)
        has_imu = device == "GS130WI"
        return cls(
            camera_config=CameraConfig(
                bus=[4, 6], left_addr=0x30,
                right_addr=0x32 if has_imu else 0x31,
                sensor_width=1088, sensor_height=1280, fps=fps,
                line_length=1400, frame_length=1500,
                output_width=width, output_height=height, mode=mode,
                stereo_layout=StereoLayout.NONE,
                bus_mipi_rx={4: 2, 6: 0},
                bus_reset_gpio={4: 351, 6: 353},
                fsync_camera=CameraIndex.RIGHT,
            ),
            imu_config=ImuConfig(
                bus=[4, 6] if has_imu else [], addr=0x68, odr_hz=odr,
                accel_fsr_g=16, gyro_fsr_dps=2000,
                accel_bw_sel=0, gyro_bw_sel=0,
            ),
            eeprom_config=EepromConfig(bus=[4, 6], addr=0x50),
            camera_fifo=FifoConfig(4, FifoMode.DROP_OLD),
            imu_fifo=(FifoConfig(1024, FifoMode.DROP_OLD)
                      if has_imu else FifoConfig()),
        )


def _fill_bus(out, values):
    values = list(values)
    if len(values) > len(out.bus):
        raise ValueError("at most %d I2C buses are supported" % len(out.bus))
    out.bus[:len(values)] = values
    out.bus_num = len(values)


def _fill_map(out, name, values, default):
    target = getattr(out, name)
    for index in range(len(target)):
        target[index] = default
    for index, value in values.items():
        if not 0 <= index < len(target):
            raise ValueError("%s bus index out of range: %s" % (name, index))
        target[index] = value


def to_c(config):
    """Convert a :class:`Config` to the exact C structure."""
    if not isinstance(config, Config):
        raise TypeError("expected Config, got %s" % type(config).__name__)
    out = _abi.Config()
    camera = config.camera_config
    _fill_bus(out.camera_config, camera.bus)
    for name in (
        "left_addr", "right_addr", "sensor_width", "sensor_height", "fps",
        "line_length", "frame_length", "output_width", "output_height",
        "mode", "stereo_layout", "fsync_camera",
    ):
        setattr(out.camera_config, name, getattr(camera, name))
    tuning = camera.tuning_file
    if isinstance(tuning, str):
        tuning = tuning.encode()
    if tuning is not None and not isinstance(tuning, bytes):
        raise TypeError("tuning_file must be str, bytes, or None")
    out.camera_config.tuning_file = tuning
    _fill_map(out.camera_config, "bus_mipi_rx", camera.bus_mipi_rx, 0xFF)
    _fill_map(out.camera_config, "bus_reset_gpio", camera.bus_reset_gpio, -1)

    imu = config.imu_config
    _fill_bus(out.imu_config, imu.bus)
    for name in (
        "addr", "odr_hz", "accel_fsr_g", "gyro_fsr_dps",
        "accel_bw_sel", "gyro_bw_sel",
    ):
        setattr(out.imu_config, name, getattr(imu, name))
    eeprom = config.eeprom_config
    _fill_bus(out.eeprom_config, eeprom.bus)
    out.eeprom_config.addr = eeprom.addr
    out.camera_fifo.depth = config.camera_fifo.depth
    out.camera_fifo.mode = config.camera_fifo.mode
    out.imu_fifo.depth = config.imu_fifo.depth
    out.imu_fifo.mode = config.imu_fifo.mode
    return out
