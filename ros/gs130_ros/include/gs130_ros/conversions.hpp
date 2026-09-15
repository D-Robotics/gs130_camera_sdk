/*
 * Mapping from gs130 C values to ROS messages. Pure functions with no node and
 * no device, so they can be tested with gtest on a host.
 */

#ifndef GS130_ROS_CONVERSIONS_H
#define GS130_ROS_CONVERSIONS_H

#include <cstdint>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "gs130.h"

namespace gs130_ros
{

/// Nanoseconds to a ROS time. The SDK stamps are CLOCK_MONOTONIC since boot,
/// so callers must add the startup offset before calling this.
builtin_interfaces::msg::Time to_stamp(int64_t nanoseconds);

/// Bytes in a tightly packed NV12 buffer of this picture size.
size_t nv12_size(uint32_t width, uint32_t height);

/// One camera frame as a sensor_msgs/Image.
///
/// The height is the REAL picture height: the TROS codec reads it that way and
/// crashes when it is given the packed NV12 row count instead. The data is
/// copied by the caller's message, so the SDK buffer can be released after the
/// returned message owns its payload.
sensor_msgs::msg::Image to_image(const uint8_t * data,
                                 uint32_t width,
                                 uint32_t height,
                                 const std::string & frame_id,
                                 const builtin_interfaces::msg::Time & stamp);

/// One IMU sample as a sensor_msgs/Imu. The SDK reports no fused orientation,
/// so it is marked unavailable, and no variance either, which ROS reads as
/// unknown from the zeroed covariances.
sensor_msgs::msg::Imu to_imu(const gs130_imu_packet_t & packet,
                             const std::string & frame_id,
                             const builtin_interfaces::msg::Time & stamp);

/// The ROS distortion model and its coefficients for these SDK values. An
/// already rectified camera reports zeros, which ROS consumers read as
/// plumb_bob with five zeros; the fisheye model carries four equidistant
/// coefficients and the pinhole model the eight rational polynomial ones.
std::string distortion_model(const gs130_camera_intrinsics_t & intrinsics,
                             std::vector<double> & coefficients);

/// K, D, R and P for one camera at the given output resolution. R stays the
/// identity and P restates K because the SDK exposes no rectification matrix.
sensor_msgs::msg::CameraInfo to_camera_info(const gs130_camera_intrinsics_t & intrinsics,
                                           uint32_t width,
                                           uint32_t height,
                                           const std::string & frame_id);

/// The quaternion of a row-major 3x3 rotation matrix.
void to_quaternion(const double rotation[9], double & x, double & y, double & z, double & w);

/// A static transform placing child_frame in parent_frame. gs130_relative_R/T
/// describe the from frame as seen in the to frame, which is a parent/child
/// transform with the to frame as the parent.
geometry_msgs::msg::TransformStamped to_transform(const double rotation[9],
                                                 const double translation[3],
                                                 const std::string & parent_frame,
                                                 const std::string & child_frame);

}  // namespace gs130_ros

#endif  // GS130_ROS_CONVERSIONS_H
