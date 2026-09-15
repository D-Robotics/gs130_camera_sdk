// Copyright (c) 2026 D-Robotics.
// SPDX-License-Identifier: MIT
//
// Tests for the conversions from gs130 values to ROS messages.
//
// These link only gs130_ros_convert and the generated messages, never
// libgs130: the conversions use the C header's types and call none of it, so
// they can be checked on a machine with no hardware and no library.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

#include "gs130_ros/convert.hpp"

namespace gs130_ros
{
namespace
{

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

gs130_camera_intrinsics_t intrinsics(
  double fx, double fy, double cx, double cy,
  gs130_dist_model_t model = GS130_DIST_PINHOLE, double distortion = 0.0)
{
  gs130_camera_intrinsics_t values{};
  values.fx = fx;
  values.fy = fy;
  values.cx = cx;
  values.cy = cy;
  values.dist_model = model;
  values.K[0] = fx;
  values.K[2] = cx;
  values.K[4] = fy;
  values.K[5] = cy;
  values.K[8] = 1.0;
  for (int i = 0; i < 8; ++i) {
    values.dist_coeffs[i] = distortion == 0.0 ? 0.0 : distortion + i;
  }
  return values;
}

std::array<double, 9> rotation_about(int axis, double degrees)
{
  const double radians = degrees * M_PI / 180.0;
  const double c = std::cos(radians);
  const double s = std::sin(radians);
  std::array<double, 9> rotation{1, 0, 0, 0, 1, 0, 0, 0, 1};
  const int a = (axis + 1) % 3;
  const int b = (axis + 2) % 3;
  rotation[a * 3 + a] = c;
  rotation[b * 3 + b] = c;
  rotation[a * 3 + b] = -s;
  rotation[b * 3 + a] = s;
  return rotation;
}

std::array<double, 9> matrix_from_quaternion(const double q[4])
{
  const double x = q[0], y = q[1], z = q[2], w = q[3];
  return {
    1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
    2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
    2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)};
}

std::vector<uint8_t> nv12(uint32_t width, uint32_t height, uint8_t fill)
{
  return std::vector<uint8_t>(static_cast<size_t>(width) * height * 3 / 2, fill);
}

/// Build the buffer gs130.cpp writes for `layout` (gs130.cpp:262-301).
///
/// Written from the C offsets, not from slice_eyes, so the two are checked
/// against each other: top/bottom puts both eyes' luma first, at 0 and w*h,
/// then their chroma at 2*w*h and 2*w*h + w*h/2; left/right interleaves the two
/// eyes inside every row, at a stride of 2*w.
std::vector<uint8_t> packed_frame(
  gs130_stereo_layout_t layout, uint32_t width, uint32_t height)
{
  const std::vector<uint8_t> left = nv12(width, height, 0x11);
  const std::vector<uint8_t> right = nv12(width, height, 0x22);
  const size_t luma = static_cast<size_t>(width) * height;
  const size_t chroma = luma / 2;
  const bool left_first = layout == GS130_STEREO_LAYOUT_TOP_BOTTOM ||
    layout == GS130_STEREO_LAYOUT_LEFT_RIGHT;

  std::vector<uint8_t> packed;
  if (layout == GS130_STEREO_LAYOUT_TOP_BOTTOM || layout == GS130_STEREO_LAYOUT_BOTTOM_TOP) {
    const std::vector<uint8_t> & first = left_first ? left : right;
    const std::vector<uint8_t> & second = left_first ? right : left;
    packed.insert(packed.end(), first.begin(), first.begin() + luma);
    packed.insert(packed.end(), second.begin(), second.begin() + luma);
    packed.insert(packed.end(), first.begin() + luma, first.end());
    packed.insert(packed.end(), second.begin() + luma, second.end());
    return packed;
  }

  const std::vector<uint8_t> & first = left_first ? left : right;
  const std::vector<uint8_t> & second = left_first ? right : left;
  for (size_t row = 0; row < luma / width; ++row) {
    packed.insert(packed.end(), first.begin() + row * width, first.begin() + (row + 1) * width);
    packed.insert(packed.end(), second.begin() + row * width, second.begin() + (row + 1) * width);
  }
  for (size_t row = 0; row < chroma / width; ++row) {
    packed.insert(
      packed.end(), first.begin() + luma + row * width, first.begin() + luma + (row + 1) * width);
    packed.insert(
      packed.end(), second.begin() + luma + row * width,
      second.begin() + luma + (row + 1) * width);
  }
  return packed;
}

// ---------------------------------------------------------------------------
// Timestamps
// ---------------------------------------------------------------------------

TEST(ToTime, SplitsSecondsAndNanoseconds)
{
  const auto stamp = to_time(1500000002);
  EXPECT_EQ(stamp.sec, 1);
  EXPECT_EQ(stamp.nanosec, 500000002u);
  EXPECT_EQ(to_time(0).sec, 0);
}

TEST(ToTime, KeepsNanosecondsInRangeForNegativeValues)
{
  // ROS allows a negative sec as long as nanosec stays in [0, 1e9).
  const auto stamp = to_time(-1);
  EXPECT_EQ(stamp.sec, -1);
  EXPECT_EQ(stamp.nanosec, 999999999u);
}

TEST(ToTime, RefusesAValuePastTheRosRange)
{
  EXPECT_THROW(to_time(static_cast<int64_t>(2147483648LL) * 1000000000LL), std::out_of_range);
}

// ---------------------------------------------------------------------------
// Rotation
// ---------------------------------------------------------------------------

TEST(Quaternion, RoundTripsThroughItsMatrix)
{
  for (int axis = 0; axis < 3; ++axis) {
    for (double degrees : {0.0, 30.0, 90.0, 179.0, 180.0, 270.0}) {
      const std::array<double, 9> rotation = rotation_about(axis, degrees);
      double quaternion[4] = {0, 0, 0, 0};
      ASSERT_TRUE(quaternion_from_rotation(rotation.data(), quaternion))
        << "axis " << axis << " degrees " << degrees;
      const double norm = std::sqrt(
        quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1] +
        quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]);
      EXPECT_NEAR(norm, 1.0, 1e-12);
      // q and -q are the same rotation, so compare the matrices they build.
      const std::array<double, 9> rebuilt = matrix_from_quaternion(quaternion);
      for (int i = 0; i < 9; ++i) {
        EXPECT_NEAR(rebuilt[i], rotation[i], 1e-9) << "axis " << axis << " index " << i;
      }
    }
  }
}

TEST(Quaternion, RefusesANonFiniteRotation)
{
  std::array<double, 9> rotation;
  rotation.fill(std::nan(""));
  double quaternion[4] = {0, 0, 0, 0};
  EXPECT_FALSE(quaternion_from_rotation(rotation.data(), quaternion));
}

// ---------------------------------------------------------------------------
// Projection
// ---------------------------------------------------------------------------

TEST(Projection, HasNoBaselineBeforeRectification)
{
  const auto projection = k_projection(intrinsics(600.0, 600.0, 544.0, 640.0));
  EXPECT_DOUBLE_EQ(projection[3], 0.0);
  EXPECT_DOUBLE_EQ(projection[7], 0.0);
  EXPECT_DOUBLE_EQ(projection[11], 0.0);
}

TEST(Projection, PutsTheBaselineInP3)
{
  const auto values = intrinsics(600.0, 600.0, 544.0, 640.0);
  // relative_T(LEFT, RIGHT): the right eye sits a baseline to the left of the
  // left eye's frame, so tx is negative and P[3] is too.
  const double translation[3] = {-0.08, 0.0, 0.0};
  const auto projection = stereo_projection(values, translation);
  EXPECT_NEAR(projection[3], 600.0 * -0.08, 1e-12);
  EXPECT_DOUBLE_EQ(projection[7], 0.0);
  EXPECT_DOUBLE_EQ(projection[11], 0.0);
  // What a stereo matcher reads back out of it.
  EXPECT_NEAR(std::fabs(projection[3] / projection[0]), 0.08, 1e-12);
}

TEST(Projection, AccountsForAnOffAxisOffset)
{
  const auto values = intrinsics(600.0, 600.0, 544.0, 640.0);
  const double translation[3] = {-0.08, 0.0, 0.002};
  const auto projection = stereo_projection(values, translation);
  EXPECT_NEAR(projection[3], 600.0 * -0.08 + 544.0 * 0.002, 1e-12);
}

TEST(Projection, LeftEyeHasNoBaseline)
{
  const auto values = intrinsics(600.0, 600.0, 544.0, 640.0);
  const double zero[3] = {0.0, 0.0, 0.0};
  EXPECT_DOUBLE_EQ(stereo_projection(values, zero)[3], 0.0);
}

TEST(Projection, MatchesTheMeasuredDevice)
{
  // Measured on an RDK X5 with a GS130WI under rect: relative_T(LEFT, RIGHT)
  // is (-0.07031615052675907, 0, 0), fx = fy = 615.5139302922461, cx = 544.
  const auto values = intrinsics(615.5139302922461, 615.5139302922461, 544.0, 640.0);
  const double translation[3] = {-0.07031615052675907, 0.0, 0.0};
  const auto projection = stereo_projection(values, translation);
  EXPECT_NEAR(projection[3], -43.28057017374667, 1e-9);
  EXPECT_NEAR(std::fabs(projection[3] / projection[0]), 0.07031615052675907, 1e-12);
}

// ---------------------------------------------------------------------------
// CameraInfo
// ---------------------------------------------------------------------------

TEST(CameraInfo, DeclaresTheEyeSizeAndAnIdentityR)
{
  const auto message = camera_info(
    intrinsics(600.0, 601.0, 544.0, 640.0), 1088, 1280, "camera_link", to_time(0), nullptr, false);
  EXPECT_EQ(message.width, 1088u);
  EXPECT_EQ(message.height, 1280u);
  EXPECT_DOUBLE_EQ(message.k[0], 600.0);
  EXPECT_DOUBLE_EQ(message.k[2], 544.0);
  const std::array<double, 9> identity{1, 0, 0, 0, 1, 0, 0, 0, 1};
  for (int i = 0; i < 9; ++i) {
    EXPECT_DOUBLE_EQ(message.r[i], identity[i]);
  }
  EXPECT_EQ(message.p.size(), 12u);
}

TEST(CameraInfo, CarriesAllEightPinholeCoefficients)
{
  const auto message = camera_info(
    intrinsics(600.0, 600.0, 544.0, 640.0, GS130_DIST_PINHOLE, 0.5), 1088, 1280,
    "camera_link", to_time(0), nullptr, false);
  EXPECT_EQ(message.distortion_model, "rational_polynomial");
  EXPECT_EQ(message.d.size(), 8u);
  EXPECT_DOUBLE_EQ(message.d[0], 0.5);
  EXPECT_DOUBLE_EQ(message.d[7], 7.5);
}

TEST(CameraInfo, CarriesFourCoefficientsForAFisheye)
{
  const auto message = camera_info(
    intrinsics(600.0, 600.0, 544.0, 640.0, GS130_DIST_FISHEYE, 0.5), 1088, 1280,
    "camera_link", to_time(0), nullptr, false);
  EXPECT_EQ(message.distortion_model, "equidistant");
  EXPECT_EQ(message.d.size(), 4u);
  EXPECT_DOUBLE_EQ(message.d[3], 3.5);
}

TEST(CameraInfo, ARectifiedFrameNeedsNoDistortionModel)
{
  const auto message = camera_info(
    intrinsics(600.0, 600.0, 544.0, 640.0, GS130_DIST_FISHEYE, 0.5), 1088, 1280,
    "camera_link", to_time(0), nullptr, true);
  EXPECT_EQ(message.distortion_model, "plumb_bob");
  EXPECT_EQ(message.d.size(), 5u);
  EXPECT_DOUBLE_EQ(message.d[0], 0.0);
}

// ---------------------------------------------------------------------------
// Packed frames
// ---------------------------------------------------------------------------

TEST(EyeSize, IsTheEyeNotTheFrame)
{
  uint32_t width = 0;
  uint32_t height = 0;
  eye_size(GS130_STEREO_LAYOUT_NONE, 128, 96, &width, &height);
  EXPECT_EQ(width, 128u);
  EXPECT_EQ(height, 96u);
  eye_size(GS130_STEREO_LAYOUT_TOP_BOTTOM, 128, 192, &width, &height);
  EXPECT_EQ(width, 128u);
  EXPECT_EQ(height, 96u);
  eye_size(GS130_STEREO_LAYOUT_BOTTOM_TOP, 128, 192, &width, &height);
  EXPECT_EQ(width, 128u);
  EXPECT_EQ(height, 96u);
  eye_size(GS130_STEREO_LAYOUT_LEFT_RIGHT, 256, 96, &width, &height);
  EXPECT_EQ(width, 128u);
  EXPECT_EQ(height, 96u);
  eye_size(GS130_STEREO_LAYOUT_RIGHT_LEFT, 256, 96, &width, &height);
  EXPECT_EQ(width, 128u);
  EXPECT_EQ(height, 96u);
}

TEST(SliceEyes, RecoversBothEyesInEveryLayout)
{
  const uint32_t eye_width = 8;
  const uint32_t eye_height = 6;
  const size_t eye_bytes = static_cast<size_t>(eye_width) * eye_height * 3 / 2;

  for (auto layout : {
      GS130_STEREO_LAYOUT_TOP_BOTTOM, GS130_STEREO_LAYOUT_BOTTOM_TOP,
      GS130_STEREO_LAYOUT_LEFT_RIGHT, GS130_STEREO_LAYOUT_RIGHT_LEFT})
  {
    const bool stacked = layout == GS130_STEREO_LAYOUT_TOP_BOTTOM ||
      layout == GS130_STEREO_LAYOUT_BOTTOM_TOP;
    const uint32_t width = stacked ? eye_width : eye_width * 2;
    const uint32_t height = stacked ? eye_height * 2 : eye_height;
    const std::vector<uint8_t> packed = packed_frame(layout, eye_width, eye_height);
    ASSERT_EQ(packed.size(), static_cast<size_t>(width) * height * 3 / 2);

    const auto eyes = slice_eyes(packed.data(), width, height, layout);
    ASSERT_EQ(eyes[0].size(), eye_bytes);
    ASSERT_EQ(eyes[1].size(), eye_bytes);
    EXPECT_TRUE(std::all_of(eyes[0].begin(), eyes[0].end(), [](uint8_t v) {return v == 0x11;}))
      << "left eye wrong for layout " << static_cast<int>(layout);
    EXPECT_TRUE(std::all_of(eyes[1].begin(), eyes[1].end(), [](uint8_t v) {return v == 0x22;}))
      << "right eye wrong for layout " << static_cast<int>(layout);
  }
}

// ---------------------------------------------------------------------------
// Images, IMU, transforms
// ---------------------------------------------------------------------------

TEST(ImageMessage, DescribesTheBufferItCarries)
{
  const uint32_t width = 8;
  const uint32_t height = 6;
  const std::vector<uint8_t> source = nv12(width, height, 0x33);
  const auto message = nv12_message(source.data(), width, height, "camera_link", to_time(0));
  EXPECT_EQ(message.encoding, "nv12");
  EXPECT_EQ(message.width, width);
  EXPECT_EQ(message.height, height);
  EXPECT_EQ(message.step, width);
  EXPECT_EQ(message.data.size(), source.size());
  EXPECT_EQ(message.header.frame_id, "camera_link");
}

TEST(ImageMessage, CopiesSoTheFrameCanBeReleased)
{
  const uint32_t width = 8;
  const uint32_t height = 6;
  std::vector<uint8_t> source = nv12(width, height, 0x33);
  const auto message = nv12_message(source.data(), width, height, "camera_link", to_time(0));
  std::fill(source.begin(), source.end(), 0);
  EXPECT_EQ(message.data[0], 0x33);
  EXPECT_EQ(message.data.back(), 0x33);
}

TEST(ImuMessage, MarksTheMissingOrientation)
{
  gs130_imu_packet_t packet{};
  packet.accel[0] = 1.0f;
  packet.accel[1] = 2.0f;
  packet.accel[2] = 3.0f;
  packet.gyro[0] = 4.0f;
  packet.gyro[1] = 5.0f;
  packet.gyro[2] = 6.0f;
  const auto message = imu_message(packet, "imu_link", to_time(0));
  EXPECT_FLOAT_EQ(message.linear_acceleration.x, 1.0f);
  EXPECT_FLOAT_EQ(message.linear_acceleration.z, 3.0f);
  EXPECT_FLOAT_EQ(message.angular_velocity.y, 5.0f);
  EXPECT_DOUBLE_EQ(message.orientation_covariance[0], -1.0);
}

TEST(TransformMessage, PassesTheDevicePoseThrough)
{
  const std::array<double, 9> rotation{1, 0, 0, 0, 1, 0, 0, 0, 1};
  const double translation[3] = {1.0, 2.0, 3.0};
  const auto message = transform_message(
    rotation.data(), translation, "camera_link", "imu_link", to_time(0));
  EXPECT_EQ(message.header.frame_id, "camera_link");
  EXPECT_EQ(message.child_frame_id, "imu_link");
  EXPECT_DOUBLE_EQ(message.transform.translation.x, 1.0);
  EXPECT_DOUBLE_EQ(message.transform.translation.z, 3.0);
  EXPECT_DOUBLE_EQ(message.transform.rotation.w, 1.0);
}

}  // namespace
}  // namespace gs130_ros
