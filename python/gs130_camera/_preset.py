"""Ready-made configurations for known hardware.

Mirrors ``GS130_CONFIG_*`` in ``core/include/gs130_define.h``.  One function
per C macro, setting exactly what that macro sets, so the two can be compared
line by line.

A field a macro leaves out stays at its C zero-initialisation here as well:
The GS130W-family presets -- ``GS130_CONFIG_RDKX5_GS130W`` and both
``_NO_EEPROM`` variants -- have no ``.imu_fifo``, so that queue keeps the zeroed
``depth = 0`` and ``DROP_NEW``.  The SDK accepts that because its depth check
only applies when ``imu_config.bus_num`` is non-zero.
"""

from ._abi import library_platform
from ._config import config
from ._types import CameraIndex, FifoMode, StereoLayout


def GS130_CONFIG_RDKX5_GS130WI(mode, width, height, fps, odr):
    """Mirror of the ``GS130_CONFIG_RDKX5_GS130WI`` macro."""
    values = config()
    values["camera_config"].update(
        bus=[4, 6],
        left_addr=0x30,
        right_addr=0x32,
        sensor_width=1088,
        sensor_height=1280,
        fps=fps,
        line_length=1400,
        frame_length=1500,
        tuning_file=None,
        output_width=width,
        output_height=height,
        mode=mode,
        stereo_layout=StereoLayout.NONE,
        bus_mipi_rx={4: 2, 6: 0},
        bus_reset_gpio={4: 351, 6: 353},
        fsync_camera=CameraIndex.RIGHT,
    )
    values["imu_config"].update(
        bus=[4, 6],
        addr=0x68,
        odr_hz=odr,
        accel_fsr_g=16,
        gyro_fsr_dps=2000,
        accel_bw_sel=0,
        gyro_bw_sel=0,
    )
    values["eeprom_config"].update(bus=[4, 6], addr=0x50)
    values["camera_fifo"].update(depth=4, mode=FifoMode.DROP_OLD)
    values["imu_fifo"].update(depth=1024, mode=FifoMode.DROP_OLD)
    return values


def GS130_CONFIG_RDKX5_GS130W(mode, width, height, fps, odr):
    """Mirror of the ``GS130_CONFIG_RDKX5_GS130W`` macro."""
    values = config()
    values["camera_config"].update(
        bus=[4, 6],
        left_addr=0x30,
        right_addr=0x31,
        sensor_width=1088,
        sensor_height=1280,
        fps=fps,
        line_length=1400,
        frame_length=1500,
        tuning_file=None,
        output_width=width,
        output_height=height,
        mode=mode,
        stereo_layout=StereoLayout.NONE,
        bus_mipi_rx={4: 2, 6: 0},
        bus_reset_gpio={4: 351, 6: 353},
        fsync_camera=CameraIndex.RIGHT,
    )
    values["imu_config"].update(
        bus=[],
        addr=0x68,
        odr_hz=odr,
        accel_fsr_g=16,
        gyro_fsr_dps=2000,
        accel_bw_sel=0,
        gyro_bw_sel=0,
    )
    values["eeprom_config"].update(bus=[4, 6], addr=0x50)
    values["camera_fifo"].update(depth=4, mode=FifoMode.DROP_OLD)
    values["imu_fifo"].update(depth=0, mode=FifoMode.DROP_NEW)
    return values


def GS130_CONFIG_RDKX5_GS130W_NO_EEPROM(mode, width, height, fps, odr):
    """Mirror of the ``GS130_CONFIG_RDKX5_GS130W_NO_EEPROM`` macro.

    A GS130W-family module with no usable calibration EEPROM. Differs from
    :func:`GS130_CONFIG_RDKX5_GS130W` in two places: the camera addresses and the
    disabled EEPROM.  The addresses are the ones measured on the module this preset
    was added for, which does not answer at the GS130W preset's 0x30/0x31.
    ``eeprom_config.bus`` is empty, which is the C macro's ``bus_num = 0``; its
    ``addr`` stays set for the same reason the disabled IMU bus above keeps its own.
    """
    values = config()
    values["camera_config"].update(
        bus=[4, 6],
        left_addr=0x33,
        right_addr=0x32,
        sensor_width=1088,
        sensor_height=1280,
        fps=fps,
        line_length=1400,
        frame_length=1500,
        tuning_file=None,
        output_width=width,
        output_height=height,
        mode=mode,
        stereo_layout=StereoLayout.NONE,
        bus_mipi_rx={4: 2, 6: 0},
        bus_reset_gpio={4: 351, 6: 353},
        fsync_camera=CameraIndex.RIGHT,
    )
    values["imu_config"].update(
        bus=[],
        addr=0x68,
        odr_hz=odr,
        accel_fsr_g=16,
        gyro_fsr_dps=2000,
        accel_bw_sel=0,
        gyro_bw_sel=0,
    )
    values["eeprom_config"].update(bus=[], addr=0x50)
    values["camera_fifo"].update(depth=4, mode=FifoMode.DROP_OLD)
    values["imu_fifo"].update(depth=0, mode=FifoMode.DROP_NEW)
    return values


def GS130_CONFIG_RDKS100_GS130WI(mode, width, height, fps, odr):
    """Mirror of the ``GS130_CONFIG_RDKS100_GS130WI`` macro."""
    values = config()
    values["camera_config"].update(
        bus=[1, 2],
        left_addr=0x30,
        right_addr=0x32,
        sensor_width=1088,
        sensor_height=1280,
        fps=fps,
        line_length=1400,
        frame_length=1500,
        tuning_file="lib_sc132gs_linear.so",
        output_width=width,
        output_height=height,
        mode=mode,
        stereo_layout=StereoLayout.NONE,
        bus_mipi_rx={1: 0, 2: 1},
        bus_reset_gpio={},
        fsync_camera=CameraIndex.RIGHT,
    )
    values["imu_config"].update(
        bus=[2],
        addr=0x68,
        odr_hz=odr,
        accel_fsr_g=16,
        gyro_fsr_dps=2000,
        accel_bw_sel=0,
        gyro_bw_sel=0,
    )
    values["eeprom_config"].update(bus=[2], addr=0x50)
    values["camera_fifo"].update(depth=4, mode=FifoMode.DROP_OLD)
    values["imu_fifo"].update(depth=1024, mode=FifoMode.DROP_OLD)
    return values


def GS130_CONFIG_RDKS100_GS130W_NO_EEPROM(mode, width, height, fps, odr):
    """Mirror of the ``GS130_CONFIG_RDKS100_GS130W_NO_EEPROM`` macro.

    A GS130W-family module with no usable calibration EEPROM, on this platform's
    camera connectors. The board wiring is :func:`GS130_CONFIG_RDKS100_GS130WI`'s --
    same two connectors, same MIPI receivers, and no reset line. What differs is the
    module: its left camera answers at 0x33, it has no IMU, and its EEPROM is not usable --
    a chip answers at 0x50 on bus 1, but its contents are an unregistered model variant the
    SDK rejects.  ``imu_config.bus`` and ``eeprom_config.bus`` are therefore empty (the C
    macro's ``bus_num = 0`` for both), and ``imu_fifo`` stays at its zeroed depth and
    DROP_NEW.
    """
    values = GS130_CONFIG_RDKS100_GS130WI(mode, width, height, fps, odr)
    values["camera_config"].update(left_addr=0x33, tuning_file=None)
    values["imu_config"].update(bus=[])
    values["eeprom_config"].update(bus=[])
    values["imu_fifo"].update(depth=0, mode=FifoMode.DROP_NEW)
    return values


PRESETS = {
    ("RDKX5", "GS130WI"): GS130_CONFIG_RDKX5_GS130WI,
    ("RDKX5", "GS130W"): GS130_CONFIG_RDKX5_GS130W,
    ("RDKX5", "GS130W_NO_EEPROM"): GS130_CONFIG_RDKX5_GS130W_NO_EEPROM,
    ("RDKS100", "GS130WI"): GS130_CONFIG_RDKS100_GS130WI,
    ("RDKS100", "GS130W_NO_EEPROM"): GS130_CONFIG_RDKS100_GS130W_NO_EEPROM,
}


def preset(device, mode, width, height, fps, odr):
    """Return a filled configuration dict for known hardware.

    ``mode`` is a :class:`CameraMode`, ``width`` / ``height`` the output size,
    ``fps`` the camera frame rate and ``odr`` the IMU output data rate in Hz.
    The result is an ordinary dict: adjust it before handing it to
    :class:`gs130_camera.Device`.

    The platform is the one ``libgs130`` was built for, as it is for the
    ``GS130_CONFIG`` macro, so there is nothing to pass in and no way to ask
    for another board's preset.  Raises :class:`ValueError` for hardware
    without a preset, where the macro prints to stderr and exits.
    """
    platform = library_platform()
    try:
        make = PRESETS[(platform, device)]
    except KeyError:
        raise ValueError(
            "unsupported platform/device: %s %s" % (platform, device)
        ) from None
    return make(mode, width, height, fps, odr)
