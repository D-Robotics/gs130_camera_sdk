// Copyright (c) 2026 D-Robotics.
// SPDX-License-Identifier: MIT

#include <exception>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "gs130_ros/node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int status = 0;
  try {
    auto node = std::make_shared<gs130_ros::Gs130Node>();
    // Capture runs on the node's own threads, so the executor is only here for
    // services and shutdown.
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("gs130_ros"), "%s", error.what());
    status = 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return status;
}
