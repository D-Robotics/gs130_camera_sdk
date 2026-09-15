// Copyright (c) 2026 D-Robotics.
// SPDX-License-Identifier: MIT

#include "gs130_ros/convert.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace gs130_ros
{

const char * const kNv12Encoding = "nv12";

namespace
{

constexpr int64_t kNsPerSec = 1000000000;
constexpr int64_t kSecMax = 2147483647;  // int32, what builtin_interfaces/Time holds

void fill_identity_rotation(std::array<double, 9> & rotation)
{
  rotation = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
}

}  // namespace

builtin_interfaces::msg::Time to_time(int64_t timestamp_ns)
{
  builtin_interfaces::msg::Time stamp;
  int64_t seconds = timestamp_ns / kNsPerSec;
  int64_t nanoseconds = timestamp_ns % kNsPerSec;
  if (nanoseconds < 0) {
    // Keep nanosec inside [0, 1e9) the way ROS defines it.
    nanoseconds += kNsPerSec;
    seconds -= 1;
  }
  if (seconds > kSecMax || seconds < -kSecMax) {
    throw std::out_of_range("timestamp " + std::to_string(timestamp_ns) + " ns is outside the ROS time range");
  }
  stamp.sec = static_cast<int32_t>(seconds);
  stamp.nanosec = static_cast<uint32_t>(nanoseconds);
  return stamp;
}

sensor_msgs::msg::Image nv12_message(
  const uint8_t * data, uint32_t width, uint32_t height,
  const std::string & frame_id, const builtin_interfaces::msg::Time & stamp)
{
  sensor_msgs::msg::Image message;
  message.header.stamp = stamp;
  message.header.frame_id = frame_id;
  message.height = height;
  message.width = width;
  message.encoding = kNv12Encoding;
  message.is_bigendian = 0;
  message.step = width;
  const size_t size = static_cast<size_t>(width) * height * 3 / 2;
  message.data.assign(data, data + size);
  return message;
}

void eye_size(
  gs130_stereo_layout_t layout, uint32_t width, uint32_t height,
  uint32_t * eye_width, uint32_t * eye_height)
{
  switch (layout) {
    case GS130_STEREO_LAYOUT_TOP_BOTTOM:
    case GS130_STEREO_LAYOUT_BOTTOM_TOP:
      *eye_width = width;
      *eye_height = height / 2;
      return;
    case GS130_STEREO_LAYOUT_LEFT_RIGHT:
    case GS130_STEREO_LAYOUT_RIGHT_LEFT:
      *eye_width = width / 2;
      *eye_height = height;
      return;
    default:
      *eye_width = width;
      *eye_height = height;
      return;
  }
}

std::array<std::vector<uint8_t>, 2> slice_eyes(
  const uint8_t * frame, uint32_t width, uint32_t height,
  gs130_stereo_layout_t layout)
{
  const bool stacked = layout == GS130_STEREO_LAYOUT_TOP_BOTTOM ||
    layout == GS130_STEREO_LAYOUT_BOTTOM_TOP;
  const bool first_is_left = layout == GS130_STEREO_LAYOUT_TOP_BOTTOM ||
    layout == GS130_STEREO_LAYOUT_LEFT_RIGHT;

  uint32_t eye_width = 0;
  uint32_t eye_height = 0;
  eye_size(layout, width, height, &eye_width, &eye_height);
  const size_t eye_size_bytes = static_cast<size_t>(eye_width) * eye_height * 3 / 2;
  const size_t luma_bytes = static_cast<size_t>(eye_width) * eye_height;
  const size_t chroma_bytes = luma_bytes / 2;

  std::array<std::vector<uint8_t>, 2> eyes;
  eyes[0].resize(eye_size_bytes);
  eyes[1].resize(eye_size_bytes);

  if (stacked) {
    // Luma of both eyes, then chroma of both: the two planes are separate
    // buffers handed to the hardware, not one image appended to another.
    const uint8_t * luma[2] = {frame, frame + static_cast<size_t>(width) * height / 2};
    const uint8_t * chroma[2] = {
      frame + static_cast<size_t>(width) * height,
      frame + static_cast<size_t>(width) * height + static_cast<size_t>(width) * height / 4};
    if (!first_is_left) {
      std::swap(luma[0], luma[1]);
      std::swap(chroma[0], chroma[1]);
    }
    for (int eye = 0; eye < 2; ++eye) {
      std::memcpy(eyes[eye].data(), luma[eye], luma_bytes);
      std::memcpy(eyes[eye].data() + luma_bytes, chroma[eye], chroma_bytes);
    }
    return eyes;
  }

  // Side by side: the eyes share every row, at a stride of 2 * eye_width, so
  // the split is a strided take of the luma band and of the chroma band.
  const uint32_t row_stride = width;
  for (int eye = 0; eye < 2; ++eye) {
    // Column offset of this eye; index 0 is always the left eye.
    const bool take_left = (eye == 0) == first_is_left;
    const uint32_t column = take_left ? 0 : eye_width;
    for (uint32_t row = 0; row < eye_height; ++row) {
      std::memcpy(
        eyes[eye].data() + static_cast<size_t>(row) * eye_width,
        frame + static_cast<size_t>(row) * row_stride + column, eye_width);
    }
    const uint8_t * chroma = frame + static_cast<size_t>(height) * row_stride;
    for (uint32_t row = 0; row < eye_height / 2; ++row) {
      std::memcpy(
        eyes[eye].data() + luma_bytes + static_cast<size_t>(row) * eye_width,
        chroma + static_cast<size_t>(row) * row_stride + column, eye_width);
    }
  }
  return eyes;
}

Projection k_projection(const gs130_camera_intrinsics_t & intrinsics)
{
  return {
    intrinsics.fx, 0.0, intrinsics.cx, 0.0,
    0.0, intrinsics.fy, intrinsics.cy, 0.0,
    0.0, 0.0, 1.0, 0.0};
}

Projection stereo_projection(
  const gs130_camera_intrinsics_t & intrinsics, const double translation[3])
{
  Projection projection = k_projection(intrinsics);
  projection[3] = intrinsics.fx * translation[0] + intrinsics.cx * translation[2];
  projection[7] = intrinsics.fy * translation[1] + intrinsics.cy * translation[2];
  return projection;
}

sensor_msgs::msg::CameraInfo camera_info(
  const gs130_camera_intrinsics_t & intrinsics, uint32_t width, uint32_t height,
  const std::string & frame_id, const builtin_interfaces::msg::Time & stamp,
  const Projection * projection, bool rectified)
{
  sensor_msgs::msg::CameraInfo message;
  message.header.stamp = stamp;
  message.header.frame_id = frame_id;
  message.width = width;
  message.height = height;

  // Rectification zeroes the coefficients in the C layer, so a rectified frame
  // is described exactly by plumb_bob with nothing to correct.  Otherwise the
  // model decides the length: the pinhole table holds the rational
  // polynomial's eight terms, the fisheye table the equidistant model's four.
  if (rectified) {
    message.distortion_model = "plumb_bob";
    message.d = std::vector<double>(5, 0.0);
  } else if (intrinsics.dist_model == GS130_DIST_FISHEYE) {
    message.distortion_model = "equidistant";
    message.d.assign(intrinsics.dist_coeffs, intrinsics.dist_coeffs + 4);
  } else {
    message.distortion_model = "rational_polynomial";
    message.d.assign(intrinsics.dist_coeffs, intrinsics.dist_coeffs + 8);
  }

  // k and p are fixed-size std::array in the generated message, so they are
  // copied element by element; only d is a variable-length vector.
  std::copy(intrinsics.K, intrinsics.K + 9, message.k.begin());
  fill_identity_rotation(message.r);
  message.p = projection ? *projection : k_projection(intrinsics);
  return message;
}

sensor_msgs::msg::Imu imu_message(
  const gs130_imu_packet_t & packet, const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp)
{
  sensor_msgs::msg::Imu message;
  message.header.stamp = stamp;
  message.header.frame_id = frame_id;
  message.linear_acceleration.x = packet.accel[0];
  message.linear_acceleration.y = packet.accel[1];
  message.linear_acceleration.z = packet.accel[2];
  message.angular_velocity.x = packet.gyro[0];
  message.angular_velocity.y = packet.gyro[1];
  message.angular_velocity.z = packet.gyro[2];
  message.orientation_covariance[0] = -1.0;
  return message;
}

bool quaternion_from_rotation(const double rotation[9], double quaternion[4])
{
  const double m00 = rotation[0], m01 = rotation[1], m02 = rotation[2];
  const double m10 = rotation[3], m11 = rotation[4], m12 = rotation[5];
  const double m20 = rotation[6], m21 = rotation[7], m22 = rotation[8];
  const double trace = m00 + m11 + m22;

  double x = 0.0, y = 0.0, z = 0.0, w = 0.0, scale = 0.0;
  if (trace > 0.0) {
    scale = std::sqrt(trace + 1.0) * 2.0;
    x = (m21 - m12) / scale;
    y = (m02 - m20) / scale;
    z = (m10 - m01) / scale;
    w = 0.25 * scale;
  } else if (m00 > m11 && m00 > m22) {
    scale = std::sqrt(1.0 + m00 - m11 - m22) * 2.0;
    x = 0.25 * scale;
    y = (m01 + m10) / scale;
    z = (m02 + m20) / scale;
    w = (m21 - m12) / scale;
  } else if (m11 > m22) {
    scale = std::sqrt(1.0 + m11 - m00 - m22) * 2.0;
    x = (m01 + m10) / scale;
    y = 0.25 * scale;
    z = (m12 + m21) / scale;
    w = (m02 - m20) / scale;
  } else {
    scale = std::sqrt(1.0 + m22 - m00 - m11) * 2.0;
    x = (m02 + m20) / scale;
    y = (m12 + m21) / scale;
    z = 0.25 * scale;
    w = (m10 - m01) / scale;
  }

  const double norm = std::sqrt(x * x + y * y + z * z + w * w);
  if (!std::isfinite(norm) || norm == 0.0) {
    return false;
  }
  quaternion[0] = x / norm;
  quaternion[1] = y / norm;
  quaternion[2] = z / norm;
  quaternion[3] = w / norm;
  return true;
}

geometry_msgs::msg::TransformStamped transform_message(
  const double rotation[9], const double translation[3],
  const std::string & parent, const std::string & child,
  const builtin_interfaces::msg::Time & stamp)
{
  geometry_msgs::msg::TransformStamped message;
  message.header.stamp = stamp;
  message.header.frame_id = parent;
  message.child_frame_id = child;

  double quaternion[4] = {0.0, 0.0, 0.0, 1.0};
  quaternion_from_rotation(rotation, quaternion);
  message.transform.rotation.x = quaternion[0];
  message.transform.rotation.y = quaternion[1];
  message.transform.rotation.z = quaternion[2];
  message.transform.rotation.w = quaternion[3];
  message.transform.translation.x = translation[0];
  message.transform.translation.y = translation[1];
  message.transform.translation.z = translation[2];
  return message;
}

}  // namespace gs130_ros
