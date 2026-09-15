"""Ready-made configurations for known hardware.

Mirrors ``GS130_CONFIG_*`` in ``core/include/gs130_define.h``.  One function
per C macro, setting exactly what that macro sets, so the two can be compared
line by line.

A field a macro leaves out stays at its C zero-initialisation here as well:
``GS130_CONFIG_RDKX5_GS130W`` has no ``.imu_fifo``, so that queue keeps the
zeroed ``depth = 0`` and ``DROP_NEW``.  The SDK accepts that because its depth
check only applies when ``imu_config.bus_num`` is non-zero.
"""

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


PRESETS = {
    ("RDKX5", "GS130WI"): GS130_CONFIG_RDKX5_GS130WI,
    ("RDKX5", "GS130W"): GS130_CONFIG_RDKX5_GS130W,
}


def preset(platform, device, mode, width, height, fps, odr):
    """Return a filled configuration dict for known hardware.

    ``mode`` is a :class:`CameraMode`, ``width`` / ``height`` the output size,
    ``fps`` the camera frame rate and ``odr`` the IMU output data rate in Hz.
    The result is an ordinary dict: adjust it before handing it to
    :class:`gs130.Device`.

    Raises :class:`ValueError` for hardware without a preset, where the
    ``GS130_CONFIG`` macro prints to stderr and exits.
    """
    try:
        make = PRESETS[(platform, device)]
    except KeyError:
        raise ValueError(
            "unsupported platform/device: %s %s" % (platform, device)
        ) from None
    return make(mode, width, height, fps, odr)
