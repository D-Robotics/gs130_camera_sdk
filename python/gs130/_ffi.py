"""The C boundary of the gs130 package.

Everything that faces the C library is reachable from here: the declarations of
gs130.h, the presets of gs130_define.h and the loaded library. The declarations
live in gs130._internal._abi, a mirror of core/include/gs130.h that is checked
against it field by field, and the loading in gs130._internal._lib.
"""

from ._internal._abi import (
    _SIGNATURES,
    gs130_calibration_t,
    gs130_camera_config_t,
    gs130_camera_index_t,
    gs130_camera_intrinsics_t,
    gs130_camera_mode_t,
    gs130_config_t,
    gs130_device_t,
    gs130_dist_model_t,
    gs130_eeprom_config_t,
    gs130_err_t,
    gs130_fifo_config_t,
    gs130_fifo_mode_t,
    gs130_image_nv12_t,
    gs130_imu_config_t,
    gs130_imu_intrinsics_t,
    gs130_imu_packet_t,
    gs130_reference_frame_t,
    gs130_stereo_layout_t,
)
from ._internal._define import (
    GS130_CONFIG,
    GS130_CONFIG_RDKX5_GS130W,
    GS130_CONFIG_RDKX5_GS130WI,
)
from ._internal._lib import load, path, platform, version

__all__ = [
    "GS130_CONFIG",
    "GS130_CONFIG_RDKX5_GS130W",
    "GS130_CONFIG_RDKX5_GS130WI",
    "_SIGNATURES",
    "gs130_calibration_t",
    "gs130_camera_config_t",
    "gs130_camera_index_t",
    "gs130_camera_intrinsics_t",
    "gs130_camera_mode_t",
    "gs130_config_t",
    "gs130_device_t",
    "gs130_dist_model_t",
    "gs130_eeprom_config_t",
    "gs130_err_t",
    "gs130_fifo_config_t",
    "gs130_fifo_mode_t",
    "gs130_image_nv12_t",
    "gs130_imu_config_t",
    "gs130_imu_intrinsics_t",
    "gs130_imu_packet_t",
    "gs130_reference_frame_t",
    "gs130_stereo_layout_t",
    "load",
    "path",
    "platform",
    "version",
]
