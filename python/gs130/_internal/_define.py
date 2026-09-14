"""Mirror of core/include/gs130_define.h: internal to the gs130 package.

The C macros are compound literals, so each one is a function here that returns
a gs130_config_t initialized the way the macro initializes it. A member the
macro does not list is left untouched, exactly as a C compound literal
zero-initializes it -- which is why GS130_CONFIG_RDKX5_GS130W() never touches
imu_fifo.
"""

from ._abi import (
    gs130_camera_index_t,
    gs130_config_t,
    gs130_fifo_mode_t,
    gs130_stereo_layout_t,
)


def GS130_CONFIG_RDKX5_GS130WI(mode_, width_, height_, fps_, odr_):
    """GS130_CONFIG_RDKX5_GS130WI(mode_, width_, height_, fps_, odr_)."""
    cfg = gs130_config_t()

    camera = cfg.camera_config
    camera.bus[:2] = [4, 6]
    camera.bus_num = 2
    camera.left_addr = 0x30
    camera.right_addr = 0x32
    camera.sensor_width = 1088
    camera.sensor_height = 1280
    camera.fps = fps_
    camera.line_length = 1400
    camera.frame_length = 1500
    camera.output_width = width_
    camera.output_height = height_
    camera.mode = mode_
    camera.stereo_layout = gs130_stereo_layout_t.GS130_STEREO_LAYOUT_NONE
    for index in range(32):
        camera.bus_mipi_rx[index] = 0xFF
        camera.bus_reset_gpio[index] = -1
    camera.bus_mipi_rx[4] = 2
    camera.bus_mipi_rx[6] = 0
    camera.bus_reset_gpio[4] = 351
    camera.bus_reset_gpio[6] = 353
    camera.fsync_camera = gs130_camera_index_t.GS130_CAMERA_RIGHT_IDX

    imu = cfg.imu_config
    imu.bus[:2] = [4, 6]
    imu.bus_num = 2
    imu.addr = 0x68
    imu.odr_hz = odr_
    imu.accel_fsr_g = 16
    imu.gyro_fsr_dps = 2000
    imu.accel_bw_sel = 0
    imu.gyro_bw_sel = 0

    eeprom = cfg.eeprom_config
    eeprom.bus[:2] = [4, 6]
    eeprom.bus_num = 2
    eeprom.addr = 0x50

    cfg.camera_fifo.depth = 4
    cfg.camera_fifo.mode = gs130_fifo_mode_t.GS130_FIFO_DROP_OLD
    cfg.imu_fifo.depth = 1024
    cfg.imu_fifo.mode = gs130_fifo_mode_t.GS130_FIFO_DROP_OLD
    return cfg


def GS130_CONFIG_RDKX5_GS130W(mode_, width_, height_, fps_, odr_):
    """GS130_CONFIG_RDKX5_GS130W(mode_, width_, height_, fps_, odr_)."""
    cfg = gs130_config_t()

    camera = cfg.camera_config
    camera.bus[:2] = [4, 6]
    camera.bus_num = 2
    camera.left_addr = 0x30
    camera.right_addr = 0x31
    camera.sensor_width = 1088
    camera.sensor_height = 1280
    camera.fps = fps_
    camera.line_length = 1400
    camera.frame_length = 1500
    camera.output_width = width_
    camera.output_height = height_
    camera.mode = mode_
    camera.stereo_layout = gs130_stereo_layout_t.GS130_STEREO_LAYOUT_NONE
    for index in range(32):
        camera.bus_mipi_rx[index] = 0xFF
        camera.bus_reset_gpio[index] = -1
    camera.bus_mipi_rx[4] = 2
    camera.bus_mipi_rx[6] = 0
    camera.bus_reset_gpio[4] = 351
    camera.bus_reset_gpio[6] = 353
    camera.fsync_camera = gs130_camera_index_t.GS130_CAMERA_RIGHT_IDX

    imu = cfg.imu_config
    imu.bus_num = 0
    imu.addr = 0x68
    imu.odr_hz = odr_
    imu.accel_fsr_g = 16
    imu.gyro_fsr_dps = 2000
    imu.accel_bw_sel = 0
    imu.gyro_bw_sel = 0

    eeprom = cfg.eeprom_config
    eeprom.bus[:2] = [4, 6]
    eeprom.bus_num = 2
    eeprom.addr = 0x50

    cfg.camera_fifo.depth = 4
    cfg.camera_fifo.mode = gs130_fifo_mode_t.GS130_FIFO_DROP_OLD
    return cfg


def GS130_CONFIG(platform, device, mode, w, h, fps, odr):
    """GS130_CONFIG(platform, device, mode, w, h, fps, odr).

    The C macro calls exit(1) on an unknown platform or device; here it raises.
    """
    if platform == "RDKX5" and device == "GS130WI":
        return GS130_CONFIG_RDKX5_GS130WI(mode, w, h, fps, odr)
    if platform == "RDKX5" and device == "GS130W":
        return GS130_CONFIG_RDKX5_GS130W(mode, w, h, fps, odr)
    raise ValueError(
        "unsupported platform/device: %s %s" % (platform, device)
    )
