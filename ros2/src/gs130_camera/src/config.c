// Copyright (c) 2026 D-Robotics.
// SPDX-License-Identifier: MIT

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
  /* The presets hardcode GS130_STEREO_LAYOUT_NONE, so the layout is applied here. */
  config.camera_config.stereo_layout = layout;
  return config;
}
