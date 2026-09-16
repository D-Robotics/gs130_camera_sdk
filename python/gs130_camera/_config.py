"""The configuration dict, and its conversion to ``gs130_config_t``.

The dict mirrors ``gs130_config_t`` field by field but holds Python values
only: ints, strings, lists, dicts and the enums from ``_types``.  Two C arrays
that exist in the header purely as sparse lookup tables are dicts here::

    bus_mipi_rx     {bus: mipi_rx}      C default 0xFF
    bus_reset_gpio  {bus: gpio}         C default -1

``bus_num`` is not in the dict; it is ``len(bus)``.

``None`` means "not set" and is rejected, except for ``tuning_file`` (NULL,
load no tuning file), whose unset state is meaningful to the SDK.
"""

from __future__ import annotations

import copy

from . import _abi
from ._types import CameraIndex, CameraMode, FifoMode, StereoLayout


# ---------------------------------------------------------------------------
# Field checkers.  They validate Python types only; whether a value makes
# sense for the hardware is left to the SDK, which reports it as an error code.
# ---------------------------------------------------------------------------


def _int(value):
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError("expected an int, got %s" % type(value).__name__)
    return value


def _enum(cls):
    def check(value):
        if not isinstance(value, cls):
            raise TypeError(
                "expected a %s, got %s" % (cls.__name__, type(value).__name__)
            )
        return value

    return check


def _path(value):
    if value is None or isinstance(value, str):
        return value
    raise TypeError("expected a str or None, got %s" % type(value).__name__)


def _buses(value):
    if not isinstance(value, list):
        raise TypeError("expected a list, got %s" % type(value).__name__)
    for bus in value:
        _int(bus)
    return value


def _slots(value):
    if not isinstance(value, dict):
        raise TypeError("expected a dict, got %s" % type(value).__name__)
    for bus, slot in value.items():
        _int(bus)
        _int(slot)
    return value


# ---------------------------------------------------------------------------
# The shape.  Each entry is (empty value, checker); config() builds the empty
# dict from it and to_c() validates against it, so the two cannot disagree.
# ---------------------------------------------------------------------------


_FIELDS = {
    "camera_config": {
        "bus": ([], _buses),
        "left_addr": (None, _int),
        "right_addr": (None, _int),
        "sensor_width": (None, _int),
        "sensor_height": (None, _int),
        "fps": (None, _int),
        "line_length": (None, _int),
        "frame_length": (None, _int),
        "tuning_file": (None, _path),
        "output_width": (None, _int),
        "output_height": (None, _int),
        "mode": (None, _enum(CameraMode)),
        "stereo_layout": (None, _enum(StereoLayout)),
        "bus_mipi_rx": ({}, _slots),
        "bus_reset_gpio": ({}, _slots),
        "fsync_camera": (None, _enum(CameraIndex)),
    },
    "imu_config": {
        "bus": ([], _buses),
        "addr": (None, _int),
        "odr_hz": (None, _int),
        "accel_fsr_g": (None, _int),
        "gyro_fsr_dps": (None, _int),
        "accel_bw_sel": (None, _int),
        "gyro_bw_sel": (None, _int),
    },
    "eeprom_config": {
        "bus": ([], _buses),
        "addr": (None, _int),
    },
    "camera_fifo": {
        "depth": (None, _int),
        "mode": (None, _enum(FifoMode)),
    },
    "imu_fifo": {
        "depth": (None, _int),
        "mode": (None, _enum(FifoMode)),
    },
}


def config():
    """Return an empty configuration dict.

    Every key of ``gs130_config_t`` is present with an unset value; fill in the
    fields you need and hand the dict to :class:`gs130_camera.Device`.  Unset values
    are rejected at that point, so nothing is silently left at zero.
    """
    return {
        section: {name: copy.deepcopy(empty) for name, (empty, _) in fields.items()}
        for section, fields in _FIELDS.items()
    }


def _validate(values):
    if not isinstance(values, dict):
        raise TypeError(
            "configuration must be a dict, got %s" % type(values).__name__
        )

    given = set(values)
    expected = set(_FIELDS)
    if given != expected:
        raise ValueError(
            "configuration sections; missing=%s unknown=%s"
            % (sorted(expected - given), sorted(given - expected))
        )

    for section, fields in _FIELDS.items():
        section_values = values[section]
        if not isinstance(section_values, dict):
            raise TypeError(
                "%s must be a dict, got %s" % (section, type(section_values).__name__)
            )

        given = set(section_values)
        expected = set(fields)
        if given != expected:
            raise ValueError(
                "%s fields; missing=%s unknown=%s"
                % (section, sorted(expected - given), sorted(given - expected))
            )

        for name, (_, check) in fields.items():
            try:
                check(section_values[name])
            except TypeError as exc:
                raise TypeError("%s.%s: %s" % (section, name, exc)) from None


def _fill_bus(target, buses):
    target.bus[: len(buses)] = buses
    target.bus_num = len(buses)


def _fill_slots(target, slots, default):
    for index in range(len(target)):
        target[index] = default
    for index, value in slots.items():
        # A negative index would silently write another bus's slot instead of
        # failing, so it is rejected here.  The value itself is the SDK's
        # business.
        if index < 0:
            raise ValueError("bus index must not be negative: %s" % index)
        target[index] = value


def to_c(values):
    """Validate a configuration dict and convert it to ``gs130_config_t``."""
    _validate(values)
    out = _abi.gs130_config_t()

    camera = values["camera_config"]
    _fill_bus(out.camera_config, camera["bus"])
    out.camera_config.left_addr = camera["left_addr"]
    out.camera_config.right_addr = camera["right_addr"]
    out.camera_config.sensor_width = camera["sensor_width"]
    out.camera_config.sensor_height = camera["sensor_height"]
    out.camera_config.fps = camera["fps"]
    out.camera_config.line_length = camera["line_length"]
    out.camera_config.frame_length = camera["frame_length"]
    out.camera_config.tuning_file = (
        camera["tuning_file"].encode() if camera["tuning_file"] else None
    )
    out.camera_config.output_width = camera["output_width"]
    out.camera_config.output_height = camera["output_height"]
    out.camera_config.mode = camera["mode"]
    out.camera_config.stereo_layout = camera["stereo_layout"]
    _fill_slots(out.camera_config.bus_mipi_rx, camera["bus_mipi_rx"], 0xFF)
    _fill_slots(out.camera_config.bus_reset_gpio, camera["bus_reset_gpio"], -1)
    out.camera_config.fsync_camera = camera["fsync_camera"]

    imu = values["imu_config"]
    _fill_bus(out.imu_config, imu["bus"])
    out.imu_config.addr = imu["addr"]
    out.imu_config.odr_hz = imu["odr_hz"]
    out.imu_config.accel_fsr_g = imu["accel_fsr_g"]
    out.imu_config.gyro_fsr_dps = imu["gyro_fsr_dps"]
    out.imu_config.accel_bw_sel = imu["accel_bw_sel"]
    out.imu_config.gyro_bw_sel = imu["gyro_bw_sel"]

    eeprom = values["eeprom_config"]
    _fill_bus(out.eeprom_config, eeprom["bus"])
    out.eeprom_config.addr = eeprom["addr"]

    for name in ("camera_fifo", "imu_fifo"):
        fifo = values[name]
        target = getattr(out, name)
        target.depth = fifo["depth"]
        target.mode = fifo["mode"]

    return out
