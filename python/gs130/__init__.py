import ctypes

from . import _ffi
from ._ffi import GS130Err

globals().update({member.name: member for member in GS130Err})

__all__ = [member.name for member in GS130Err] + [
    "Config",
    "CameraConfig",
    "ImuConfig",
    "EepromConfig",
    "FifoConfig",
    "GS130Error",
]


class GS130Error(RuntimeError):
    def __init__(self, code, func=None):
        self.code = code
        self.func = func
        try:
            text = GS130Err(code).name
        except ValueError:
            text = "error code %s" % (code,)
        if func:
            text = "%s() -> %s" % (func, text)
        super().__init__(text)


_MAP_DEFAULTS = {"bus_mipi_rx": 0xFF, "bus_reset_gpio": -1}

_PY_TYPES = {}


# gs130_fifo_config_t
class FifoConfig:
    def __init__(self, depth=0, mode=0):
        self.depth = depth
        self.mode = mode


# gs130_camera_config_t
class CameraConfig:
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
        fsync_camera=0
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


# gs130_imu_config_t
class ImuConfig:
    def __init__(
        self,
        bus=None,
        addr=0,
        odr_hz=0,
        accel_fsr_g=0,
        gyro_fsr_dps=0,
        accel_bw_sel=0,
        gyro_bw_sel=0
    ):
        self.bus = list(bus or ())
        self.addr = addr
        self.odr_hz = odr_hz
        self.accel_fsr_g = accel_fsr_g
        self.gyro_fsr_dps = gyro_fsr_dps
        self.accel_bw_sel = accel_bw_sel
        self.gyro_bw_sel = gyro_bw_sel


# gs130_eeprom_config_t
class EepromConfig:
    def __init__(self, bus=None, addr=0):
        self.bus = list(bus or ())
        self.addr = addr


# gs130_config_t
class Config:
    def __init__(
        self,
        camera_config=None,
        imu_config=None,
        eeprom_config=None,
        camera_fifo=None,
        imu_fifo=None
    ):
        self.camera_config = camera_config if camera_config else CameraConfig()
        self.imu_config = imu_config if imu_config else ImuConfig()
        self.eeprom_config = eeprom_config if eeprom_config else EepromConfig()
        self.camera_fifo = camera_fifo if camera_fifo else FifoConfig()
        self.imu_fifo = imu_fifo if imu_fifo else FifoConfig()

    @classmethod
    def preset(cls, platform, device, mode, width, height, fps, odr):
        return _from_c(_ffi.preset(platform, device, mode, width, height, fps, odr))


_PY_TYPES.update(
    {
        _ffi.GS130FifoConfig: FifoConfig,
        _ffi.GS130CameraConfig: CameraConfig,
        _ffi.GS130ImuConfig: ImuConfig,
        _ffi.GS130EepromConfig: EepromConfig,
        _ffi.GS130Config: Config,
    }
)


def _to_c(config, c_type=None):
    c_type = c_type or _ffi.GS130Config
    out = c_type()
    keepalive = []
    out._keepalive = keepalive
    for name, field_type in c_type._fields_:
        value = getattr(config, name, None)
        if isinstance(field_type, type) and issubclass(field_type, ctypes.Structure):
            setattr(out, name, _to_c(value, field_type))
        elif name in _MAP_DEFAULTS:
            array = getattr(out, name)
            for index in range(len(array)):
                array[index] = _MAP_DEFAULTS[name]
            for index, item in (value or {}).items():
                array[index] = item
        elif isinstance(field_type, type) and issubclass(field_type, ctypes.Array):
            array = getattr(out, name)
            for index, item in enumerate(value or ()):
                array[index] = item
            if hasattr(out, name + "_num"):
                setattr(out, name + "_num", len(value or ()))
        elif field_type is ctypes.c_char_p:
            data = value.encode() if isinstance(value, str) else value
            setattr(out, name, data)
            if data is not None:
                keepalive.append(data)
        elif value is not None:
            setattr(out, name, value)
    return out


def _from_c(struct):
    field_names = {name for name, _ in type(struct)._fields_}
    config = _PY_TYPES[type(struct)]()
    for name, field_type in type(struct)._fields_:
        if name.endswith("_num") and name[: -len("_num")] in field_names:
            continue
        value = getattr(struct, name)
        if isinstance(field_type, type) and issubclass(field_type, ctypes.Structure):
            setattr(config, name, _from_c(value))
        elif name in _MAP_DEFAULTS:
            setattr(
                config,
                name,
                {i: item for i, item in enumerate(value) if item != _MAP_DEFAULTS[name]},
            )
        elif isinstance(field_type, type) and issubclass(field_type, ctypes.Array):
            count = getattr(struct, name + "_num", len(value))
            setattr(config, name, list(value)[:count])
        elif field_type is ctypes.c_char_p:
            setattr(config, name, value.decode() if value else None)
        else:
            setattr(config, name, value)
    return config
