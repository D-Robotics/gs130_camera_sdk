// Copyright (c) 2026 D-Robotics.
// SPDX-License-Identifier: MIT
//
// The device configuration and the error type, kept apart from the node so
// that tools can use them without pulling in rclcpp.

#ifndef GS130_ROS__CONFIG_HPP_
#define GS130_ROS__CONFIG_HPP_

#include <stdexcept>
#include <string>

#include "gs130.h"

namespace gs130_ros
{

/// A call into libgs130 failed, carrying the code it reported.
class Gs130Error : public std::runtime_error
{
public:
  Gs130Error(gs130_err_t code, const std::string & function);

  gs130_err_t code() const {return code_;}
  const std::string & function() const {return function_;}

private:
  gs130_err_t code_;
  std::string function_;
};

/// Raise Gs130Error unless the C call reported GS130_OK.
void check(gs130_err_t code, const std::string & function);

/// The name of an error code, for a log line.
const char * error_name(gs130_err_t code);

/// The device configuration for a known board and camera model.
///
/// Mirrors the GS130_CONFIG_RDKX5_* macros in gs130_define.h, which this
/// package cannot use: they initialise arrays with GNU range designators,
/// which g++ rejects outright.  The macros also call exit(1) for an unknown
/// board, where a launch file needs an error it can report.
///
/// Throws std::invalid_argument for a board and model with no preset.
gs130_config_t make_config(
  const std::string & platform, const std::string & device,
  gs130_camera_mode_t mode, uint32_t width, uint32_t height, uint32_t fps,
  uint32_t odr, gs130_stereo_layout_t layout);

}  // namespace gs130_ros

#endif  // GS130_ROS__CONFIG_HPP_
