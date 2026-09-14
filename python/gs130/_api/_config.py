"""Python objects for the gs130 configuration structs.

_to_c() turns one into a ctypes structure, _from_c() turns one back.
"""

import ctypes

from .._internal import _define
from .._internal._abi import (
    gs130_camera_config_t,
    gs130_config_t,
    gs130_eeprom_config_t,
    gs130_fifo_config_t,
    gs130_imu_config_t,
)

# what an entry of these arrays means when it is not configured
_MAP_DEFAULTS = {"bus_mipi_rx": 0xFF, "bus_reset_gpio": -1}


def _is_struct(field_type):
    return hasattr(field_type, "_fields_")


def _is_array(field_type):
    return (
        isinstance(field_type, type)
        and issubclass(field_type, ctypes.Array)
    )


class FifoConfig:
    """A queue of depth items, dropped according to mode."""

    def __init__(self, depth=0, mode=0):
        self.depth = depth
        self.mode = mode


class CameraConfig:
    """Camera configuration.

    bus is the list of candidate I2C buses; bus_mipi_rx and bus_reset_gpio are
    {bus: value} maps; tuning_file is a path or None.
    """

    def __init__(
        self,
        bus=None,
        left_addr=0,
        right_addr=0,
        sensor_width=0,
        sensor_height=0,
        fps=0,
        line_length=0,
        frame_length=0,
        tuning_file=None,
        output_width=0,
        output_height=0,
        mode=0,
        stereo_layout=0,
        bus_mipi_rx=None,
        bus_reset_gpio=None,
        fsync_camera=0,
    ):
        self.bus = list(bus or ())
        self.left_addr = left_addr
        self.right_addr = right_addr
        self.sensor_width = sensor_width
        self.sensor_height = sensor_height
        self.fps = fps
        self.line_length = line_length
        self.frame_length = frame_length
        self.tuning_file = tuning_file
        self.output_width = output_width
        self.output_height = output_height
        self.mode = mode
        self.stereo_layout = stereo_layout
        self.bus_mipi_rx = dict(bus_mipi_rx or {})
        self.bus_reset_gpio = dict(bus_reset_gpio or {})
        self.fsync_camera = fsync_camera


class ImuConfig:
    """IMU configuration, with the list of candidate I2C buses."""

    def __init__(
        self,
        bus=None,
        addr=0,
        odr_hz=0,
        accel_fsr_g=0,
        gyro_fsr_dps=0,
        accel_bw_sel=0,
        gyro_bw_sel=0,
    ):
        self.bus = list(bus or ())
        self.addr = addr
        self.odr_hz = odr_hz
        self.accel_fsr_g = accel_fsr_g
        self.gyro_fsr_dps = gyro_fsr_dps
        self.accel_bw_sel = accel_bw_sel
        self.gyro_bw_sel = gyro_bw_sel


class EepromConfig:
    """EEPROM configuration, with the list of candidate I2C buses."""

    def __init__(self, bus=None, addr=0):
        self.bus = list(bus or ())
        self.addr = addr


class Config:
    """Everything gs130_init() takes.

    Start from Config.preset() and change what has to change.
    """

    def __init__(
        self,
        camera_config=None,
        imu_config=None,
        eeprom_config=None,
        camera_fifo=None,
        imu_fifo=None,
    ):
        self.camera_config = camera_config or CameraConfig()
        self.imu_config = imu_config or ImuConfig()
        self.eeprom_config = eeprom_config or EepromConfig()
        self.camera_fifo = camera_fifo or FifoConfig()
        self.imu_fifo = imu_fifo or FifoConfig()

    @classmethod
    def preset(cls, platform, device, mode, width, height, fps, odr):
        """The configuration GS130_CONFIG(platform, device, ...) builds.

        :raises ValueError: unknown platform or device.
        """
        return _from_c(
            _define.GS130_CONFIG(
                platform, device, mode, width, height, fps, odr
            )
        )


_TO_C_TYPES = {
    FifoConfig: gs130_fifo_config_t,
    CameraConfig: gs130_camera_config_t,
    ImuConfig: gs130_imu_config_t,
    EepromConfig: gs130_eeprom_config_t,
    Config: gs130_config_t,
}

_FROM_C_TYPES = {c_type: cls for cls, c_type in _TO_C_TYPES.items()}


def _to_c(config, c_type=None):
    """Fill a ctypes structure from a config object.

    The bus count follows the list and the bus maps are filled with their "not
    configured" marker, so an object only names what it wants to change.
    """
    c_type = c_type or _TO_C_TYPES[type(config)]
    field_names = {name for name, _ in c_type._fields_}
    out = c_type()
    for name, field_type in c_type._fields_:
        if name.endswith("_num") and name[: -len("_num")] in field_names:
            continue                      # written along with the array
        if _is_struct(field_type):
            setattr(out, name, _to_c(getattr(config, name), field_type))
        elif name in _MAP_DEFAULTS:
            entries = getattr(out, name)
            for index in range(len(entries)):
                entries[index] = _MAP_DEFAULTS[name]
            for index, item in (getattr(config, name) or {}).items():
                entries[index] = item
        elif _is_array(field_type):
            entries = getattr(out, name)
            values = list(getattr(config, name) or ())
            for index, item in enumerate(values):
                entries[index] = item
            if name + "_num" in field_names:
                setattr(out, name + "_num", len(values))
        elif field_type is ctypes.c_char_p:
            # ctypes keeps the bytes alive through the structure's own _objects
            value = getattr(config, name)
            setattr(
                out,
                name,
                value.encode() if isinstance(value, str) else value,
            )
        else:
            setattr(out, name, getattr(config, name))
    return out


def _from_c(struct):
    """Build the config object a ctypes structure describes."""
    field_names = {name for name, _ in type(struct)._fields_}
    values = {}
    for name, field_type in type(struct)._fields_:
        if name.endswith("_num") and name[: -len("_num")] in field_names:
            continue
        value = getattr(struct, name)
        if _is_struct(field_type):
            values[name] = _from_c(value)
        elif name in _MAP_DEFAULTS:
            values[name] = {
                index: int(item)
                for index, item in enumerate(value)
                if item != _MAP_DEFAULTS[name]
            }
        elif _is_array(field_type):
            count = getattr(struct, name + "_num", len(value))
            values[name] = [int(item) for item in list(value)[:count]]
        elif field_type is ctypes.c_char_p:
            values[name] = value.decode() if value else None
        else:
            values[name] = value
    return _FROM_C_TYPES[type(struct)](**values)
