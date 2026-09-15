cmake_minimum_required(VERSION 3.8)
project(gs130_ros C CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# No pkg-config file ships with the SDK, so the library is located directly.
find_library(GS130_LIBRARY NAMES gs130 REQUIRED)
message(STATUS "libgs130: ${GS130_LIBRARY}")

add_executable(camera_node
  src/camera_node.cpp
  src/conversions.cpp
  src/preset.c)
target_include_directories(camera_node PRIVATE include)
target_link_libraries(camera_node ${GS130_LIBRARY})
ament_target_dependencies(camera_node rclcpp sensor_msgs geometry_msgs tf2_ros)

install(TARGETS camera_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_conversions test/test_conversions.cpp src/conversions.cpp)
  target_include_directories(test_conversions PRIVATE include)
  ament_target_dependencies(test_conversions sensor_msgs geometry_msgs)
endif()

ament_package()
