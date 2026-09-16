# GS130 ROS 2 camera package.
#
# This file is part of gs130_camera_sdk (https://github.com/D-Robotics/gs130_camera_sdk).
# Copyright (c) 2026 D-Robotics.
# SPDX-License-Identifier: MIT
# See the LICENSE file in the project root for the full license text.

cmake_minimum_required(VERSION 3.8)
project(gs130_camera)

# config.c expands the SDK's GNU C preset macros; gs130_node.cpp uses C++17.
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ROS 2 build system and message/TF interfaces used by gs130_node.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)

# libgs130 does not export a CMake package configuration, so locate its library
# and public header independently. A checkout build finds the sibling core header.
# Standalone builds can use GS130_ROOT/include for headers, while the library must
# be in a default linker search path or supplied through the GS130_LIBRARY cache
# variable; GS130_INCLUDE_DIR may likewise override header discovery.
find_library(GS130_LIBRARY NAMES gs130)
find_path(GS130_INCLUDE_DIR NAMES gs130.h
  HINTS "${CMAKE_CURRENT_SOURCE_DIR}/../../../core/include"
        "$ENV{GS130_ROOT}/include")

if(NOT GS130_LIBRARY)
  message(FATAL_ERROR "libgs130 not found; configure with -DGS130_LIBRARY=/path/to/libgs130.so")
endif()
if(NOT GS130_INCLUDE_DIR)
  message(FATAL_ERROR "gs130.h not found; build from the gs130_sdk checkout or pass -DGS130_INCLUDE_DIR=")
endif()
if(NOT EXISTS "${GS130_INCLUDE_DIR}/gs130_define.h")
  message(FATAL_ERROR "gs130_define.h not found next to gs130.h")
endif()

# Build the C++ node together with the small GNU C preset bridge.
add_executable(gs130_node src/gs130_node.cpp src/config.c)
target_include_directories(gs130_node PRIVATE ${GS130_INCLUDE_DIR})
target_link_libraries(gs130_node ${GS130_LIBRARY})
ament_target_dependencies(gs130_node
  rclcpp sensor_msgs geometry_msgs builtin_interfaces tf2_ros)

# Install the executable where ros2 run expects package executables and copy all
# launch files into the package share directory.
install(TARGETS gs130_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
