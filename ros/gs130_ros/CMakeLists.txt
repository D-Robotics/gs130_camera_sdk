cmake_minimum_required(VERSION 3.8)
project(gs130_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# The version lives in the repository root, because libgs130 and everything
# that talks to it have to agree on it.
get_filename_component(GS130_ROS_PACKAGE_DIR "${CMAKE_CURRENT_SOURCE_DIR}" REALPATH)
set(GS130_ROS_VERSION_FILE "${GS130_ROS_PACKAGE_DIR}/../../VERSION")
if(EXISTS "${GS130_ROS_VERSION_FILE}")
  file(READ "${GS130_ROS_VERSION_FILE}" GS130_ROS_VERSION)
  string(STRIP "${GS130_ROS_VERSION}" GS130_ROS_VERSION)
else()
  set(GS130_ROS_VERSION "0.0.0")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)

# libgs130 is a plain C library with no CMake package of its own, so it is
# located by name and by header.  Building from the checkout finds the header
# next to this package; a standalone build needs GS130_ROOT or the two
# variables below.
find_library(GS130_LIBRARY NAMES gs130)
find_path(GS130_INCLUDE_DIR NAMES gs130.h
  HINTS "${GS130_ROS_PACKAGE_DIR}/../../core/include"
        "$ENV{GS130_ROOT}/include")

if(NOT GS130_LIBRARY)
  message(FATAL_ERROR
    "libgs130 was not found.  Install it where the linker looks, or configure "
    "with -DGS130_LIBRARY=/path/to/libgs130.so")
endif()
if(NOT GS130_INCLUDE_DIR)
  message(FATAL_ERROR
    "gs130.h was not found.  Build this package from the gs130_sdk checkout, "
    "or configure with -DGS130_INCLUDE_DIR=/path/to/include")
endif()
message(STATUS "gs130 ${GS130_ROS_VERSION}: header ${GS130_INCLUDE_DIR}, library ${GS130_LIBRARY}")

# The conversions, in a library of their own so the tests can link them
# without libgs130: they touch the C header's types but call none of it.
add_library(gs130_ros_convert src/convert.cpp)
target_include_directories(gs130_ros_convert PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  ${GS130_INCLUDE_DIR})
ament_target_dependencies(gs130_ros_convert
  sensor_msgs
  geometry_msgs
  builtin_interfaces)

add_executable(gs130_node src/main.cpp src/node.cpp)
target_include_directories(gs130_node PRIVATE ${GS130_INCLUDE_DIR})
target_link_libraries(gs130_node gs130_ros_convert ${GS130_LIBRARY})
ament_target_dependencies(gs130_node
  rclcpp
  sensor_msgs
  geometry_msgs
  builtin_interfaces
  tf2_ros)

install(TARGETS gs130_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_convert test/test_convert.cpp)
  target_link_libraries(test_convert gs130_ros_convert)
  target_include_directories(test_convert PRIVATE ${GS130_INCLUDE_DIR})
endif()

ament_package()
