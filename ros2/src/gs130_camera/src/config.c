/**
 * @file config.c
 * @brief C implementation of the SDK preset bridge used by the ROS 2 node.
 *
 * This file is compiled as GNU C because the SDK presets use designated-range
 * initializers and compound literals that are not a portable C++ interface.
 *
 * This file is part of gs130_camera_sdk (https://github.com/D-Robotics/gs130_camera_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "gs130_define.h"

gs130_config_t gs130_camera_config_from_define(
  const char * device, gs130_camera_mode_t mode,
  uint32_t width, uint32_t height, uint32_t fps, uint32_t odr,
  gs130_stereo_layout_t layout)
{
  gs130_config_t config = GS130_CONFIG(device, mode, width, height, fps, odr);
  /* SDK presets select separate frames; the ROS parameter overrides that field. */
  config.camera_config.stereo_layout = layout;
  return config;
}
