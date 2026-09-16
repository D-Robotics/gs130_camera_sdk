/**
 * @file config.h
 * @brief C bridge for creating SDK preset configurations from the ROS 2 node.
 *
 * This file is part of gs130_camera_sdk (https://github.com/D-Robotics/gs130_camera_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */

#ifndef GS130_CAMERA_CONFIG_H_
#define GS130_CAMERA_CONFIG_H_

#include <stdint.h>

#include "gs130.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Build a platform preset and apply the requested stereo layout.
 *
 * This C function isolates the GNU C initializer macros in gs130_define.h from
 * the C++ ROS 2 node.
 *
 * @param[in] device SDK device-model string, such as "GS130WI" or "GS130W".
 * @param[in] mode Camera processing mode.
 * @param[in] width Output width of one eye, in pixels.
 * @param[in] height Output height of one eye, in pixels.
 * @param[in] fps Camera frame rate in frames per second.
 * @param[in] odr IMU output data rate in Hz.
 * @param[in] layout Requested separate or stitched output layout.
 * @return A complete SDK configuration value.
 *
 * @note Unsupported platform/model pairs follow GS130_CONFIG() semantics and
 *       terminate the process after printing an error.
 */
gs130_config_t gs130_camera_config_from_define(
  const char * device, gs130_camera_mode_t mode,
  uint32_t width, uint32_t height, uint32_t fps, uint32_t odr,
  gs130_stereo_layout_t layout);

#ifdef __cplusplus
}
#endif

#endif  // GS130_CAMERA_CONFIG_H_
