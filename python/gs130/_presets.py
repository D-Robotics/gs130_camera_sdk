"""Python copies of the preset macros in core/include/gs130_define.h.

Keep each function in step with the C macro of the same name. Registering a
new platform/device is one entry in PRESETS; preset selection contains no
device-specific branches.
"""

from ._enums import CameraIndex, CameraMode, FifoMode, StereoLayout


def GS130_CONFIG_RDKX5_GS130WI(mode, width, height, fps, odr):
    """Python copy of GS130_CONFIG_RDKX5_GS130WI."""
    return {
        "camera_config": {
            "bus": [4, 6],
            "left_addr": 0x30,
            "right_addr": 0x32,
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
            "bus": [4, 6],
            "addr": 0x68,
            "odr_hz": odr,
            "accel_fsr_g": 16,
            "gyro_fsr_dps": 2000,
            "accel_bw_sel": 0,
            "gyro_bw_sel": 0,
        },
        "eeprom_config": {"bus": [4, 6], "addr": 0x50},
        "camera_fifo": {"depth": 4, "mode": FifoMode.DROP_OLD},
        "imu_fifo": {"depth": 1024, "mode": FifoMode.DROP_OLD},
    }


def GS130_CONFIG_RDKX5_GS130W(mode, width, height, fps, odr):
    """Python copy of GS130_CONFIG_RDKX5_GS130W."""
    return {
        "camera_config": {
            "bus": [4, 6],
            "left_addr": 0x30,
            "right_addr": 0x31,
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
            "bus": [],
            "addr": 0x68,
            "odr_hz": odr,
            "accel_fsr_g": 16,
            "gyro_fsr_dps": 2000,
            "accel_bw_sel": 0,
            "gyro_bw_sel": 0,
        },
        "eeprom_config": {"bus": [4, 6], "addr": 0x50},
        "camera_fifo": {"depth": 4, "mode": FifoMode.DROP_OLD},
        "imu_fifo": {"depth": 0, "mode": FifoMode.DROP_NEW},
    }


PRESETS = {
    ("RDKX5", "GS130WI"): GS130_CONFIG_RDKX5_GS130WI,
    ("RDKX5", "GS130W"): GS130_CONFIG_RDKX5_GS130W,
}
