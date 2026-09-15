"""Nested dictionary configuration and C conversion."""

from . import _abi
from ._enums import CameraIndex, CameraMode, FifoMode, StereoLayout


_SECTIONS = {
    "camera_config": {
        "bus", "left_addr", "right_addr", "sensor_width", "sensor_height",
        "fps", "line_length", "frame_length", "tuning_file",
        "output_width", "output_height", "mode", "stereo_layout",
        "bus_mipi_rx", "bus_reset_gpio", "fsync_camera",
    },
    "imu_config": {
        "bus", "addr", "odr_hz", "accel_fsr_g", "gyro_fsr_dps",
        "accel_bw_sel", "gyro_bw_sel",
    },
    "eeprom_config": {"bus", "addr"},
    "camera_fifo": {"depth", "mode"},
    "imu_fifo": {"depth", "mode"},
}


def preset(platform, device, mode, width, height, fps, odr):
    """Return a mutable nested-dict configuration for known hardware."""
    if platform != "RDKX5" or device not in {"GS130WI", "GS130W"}:
        raise ValueError("unsupported platform/device: %s %s" % (platform, device))

    has_imu = device == "GS130WI"
    return {
        "camera_config": {
            "bus": [4, 6],
            "left_addr": 0x30,
            "right_addr": 0x32 if has_imu else 0x31,
            "sensor_width": 1088,
            "sensor_height": 1280,
            "fps": fps,
            "line_length": 1400,
            "frame_length": 1500,
            "tuning_file": None,
            "output_width": width,
            "output_height": height,
            "mode": CameraMode(mode),
            "stereo_layout": StereoLayout.NONE,
            "bus_mipi_rx": {4: 2, 6: 0},
            "bus_reset_gpio": {4: 351, 6: 353},
            "fsync_camera": CameraIndex.RIGHT,
        },
        "imu_config": {
            "bus": [4, 6] if has_imu else [],
            "addr": 0x68,
            "odr_hz": odr,
            "accel_fsr_g": 16,
            "gyro_fsr_dps": 2000,
            "accel_bw_sel": 0,
            "gyro_bw_sel": 0,
        },
        "eeprom_config": {"bus": [4, 6], "addr": 0x50},
        "camera_fifo": {"depth": 4, "mode": FifoMode.DROP_OLD},
        "imu_fifo": (
            {"depth": 1024, "mode": FifoMode.DROP_OLD}
            if has_imu else
            {"depth": 0, "mode": FifoMode.DROP_NEW}
        ),
    }


def _validate(config):
    if not isinstance(config, dict):
        raise TypeError("configuration must be a dict")

    names = set(config)
    expected = set(_SECTIONS)
    if names != expected:
        missing = sorted(expected - names)
        unknown = sorted(names - expected)
        raise ValueError(
            "invalid configuration sections; missing=%s unknown=%s"
            % (missing, unknown)
        )

    for name, fields in _SECTIONS.items():
        values = config[name]
        if not isinstance(values, dict):
            raise TypeError("configuration section %s must be a dict" % name)
        names = set(values)
        if names != fields:
            missing = sorted(fields - names)
            unknown = sorted(names - fields)
            raise ValueError(
                "invalid %s fields; missing=%s unknown=%s"
                % (name, missing, unknown)
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
    """Convert a nested dictionary to the exact C configuration structure."""
    _validate(config)
    out = _abi.Config()

    camera = config["camera_config"]
    _fill_bus(out.camera_config, camera["bus"])
    for name in (
        "left_addr", "right_addr", "sensor_width", "sensor_height", "fps",
        "line_length", "frame_length", "output_width", "output_height",
        "mode", "stereo_layout", "fsync_camera",
    ):
        setattr(out.camera_config, name, camera[name])
    tuning = camera["tuning_file"]
    if isinstance(tuning, str):
        tuning = tuning.encode()
    if tuning is not None and not isinstance(tuning, bytes):
        raise TypeError("tuning_file must be str, bytes, or None")
    out.camera_config.tuning_file = tuning
    _fill_map(out.camera_config, "bus_mipi_rx", camera["bus_mipi_rx"], 0xFF)
    _fill_map(out.camera_config, "bus_reset_gpio", camera["bus_reset_gpio"], -1)

    imu = config["imu_config"]
    _fill_bus(out.imu_config, imu["bus"])
    for name in (
        "addr", "odr_hz", "accel_fsr_g", "gyro_fsr_dps",
        "accel_bw_sel", "gyro_bw_sel",
    ):
        setattr(out.imu_config, name, imu[name])

    eeprom = config["eeprom_config"]
    _fill_bus(out.eeprom_config, eeprom["bus"])
    out.eeprom_config.addr = eeprom["addr"]

    for name in ("camera_fifo", "imu_fifo"):
        target = getattr(out, name)
        target.depth = config[name]["depth"]
        target.mode = config[name]["mode"]
    return out
