/*
 * The SDK's preset macros expand to a braced initializer that uses GNU range
 * designators, which C++ does not accept in any standard mode. This C shim is
 * the only place that touches them, so the preset values stay defined in
 * gs130_define.h instead of being copied into C++.
 */

#ifndef GS130_ROS_PRESET_H
#define GS130_ROS_PRESET_H

#include "gs130.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Build a preset configuration for a platform and device.
 *
 * Unlike the GS130_CONFIG macro this never calls exit(): an unknown platform
 * or device is reported to the caller instead.
 *
 * @param[in]  platform  platform name, for example "RDKX5".
 * @param[in]  device    device name, for example "GS130WI".
 * @param[in]  mode      a gs130_camera_mode_t value.
 * @param[in]  w,h       per-eye output size.
 * @param[in]  fps       camera frame rate.
 * @param[in]  odr       IMU output data rate.
 * @param[out] out       filled on success.
 * @return 0 on success, -1 when the platform or device is unknown.
 */
int gs130_ros_preset(const char *platform, const char *device, int mode,
                     int w, int h, int fps, int odr, gs130_config_t *out);

#ifdef __cplusplus
}
#endif

#endif /* GS130_ROS_PRESET_H */
