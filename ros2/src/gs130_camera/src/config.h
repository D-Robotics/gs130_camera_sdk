// Copyright (c) 2026 D-Robotics.
// SPDX-License-Identifier: MIT

#ifndef GS130_CAMERA_CONFIG_H_
#define GS130_CAMERA_CONFIG_H_

#include <stdint.h>

#include "gs130.h"

#ifdef __cplusplus
extern "C" {
#endif

gs130_config_t gs130_camera_config_from_define(
  const char * device, gs130_camera_mode_t mode,
  uint32_t width, uint32_t height, uint32_t fps, uint32_t odr,
  gs130_stereo_layout_t layout);

#ifdef __cplusplus
}
#endif

#endif  // GS130_ROS_CONFIG_H_
