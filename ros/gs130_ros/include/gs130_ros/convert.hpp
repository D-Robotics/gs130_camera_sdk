// Copyright (c) 2026 D-Robotics.
// SPDX-License-Identifier: MIT
//
// Conversions from gs130 values into ROS messages.
//
// Every function here is pure: one gs130 value in, one message out, no node
// and no clock.  node.cpp decides what to publish and when; this file only
// decides what it looks like.
//
// The camera publishes NV12 because that is the only format the C API
// produces.  sensor_msgs/Image for NV12 uses step == width and height equal to
// the luma height, so a frame's data is width * height * 3 / 2 bytes: height
// luma rows followed by height / 2 interleaved chroma rows.  A stitched frame
// keeps that layout with the two eyes packed inside it; see eye_size and
// slice_eyes.

#ifndef GS130_ROS__CONVERT_HPP_
#define GS130_ROS__CONVERT_HPP_

#include <array>
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

/// NV12 in ROS: one byte per pixel, so the row stride is the width.
extern const char * const kNv12Encoding;

/// A projection matrix, row major, as sensor_msgs/CameraInfo carries it.
using Projection = std::array<double, 12>;

/// Split a device timestamp into a ROS time.
///
/// Throws std::out_of_range when the value does not fit, rather than wrapping
/// around and silently producing a frame from 1970 or 2038.
builtin_interfaces::msg::Time to_time(int64_t timestamp_ns);

/// Build the sensor_msgs/Image for one NV12 buffer.
///
/// `data` is copied, so the caller may release it as soon as this returns.
sensor_msgs::msg::Image nv12_message(
  const uint8_t * data, uint32_t width, uint32_t height,
  const std::string & frame_id, const builtin_interfaces::msg::Time & stamp);

/// The size of one eye inside a frame packed with `layout`.
void eye_size(
  gs130_stereo_layout_t layout, uint32_t width, uint32_t height,
  uint32_t * eye_width, uint32_t * eye_height);

/// Split a stitched NV12 frame into {left, right}, each a complete NV12 image.
///
/// The buffer is the one gs130.cpp lays out, and it is also the one
/// hobot_stereonet reads back: the two eyes' luma planes first, then their
/// chroma planes, when they are stacked; shared rows at a stride of 2 * width
/// when they are side by side.
std::array<std::vector<uint8_t>, 2> slice_eyes(
  const uint8_t * frame, uint32_t width, uint32_t height,
  gs130_stereo_layout_t layout);

/// The default P of an eye: its own K with a zero translation.
///
/// Used for every unrectified mode, where there is no virtual parallel pair
/// and therefore no baseline to report.
Projection k_projection(const gs130_camera_intrinsics_t & intrinsics);

/// P of a rectified eye from its position in the rectified left eye's frame.
///
/// `translation` is the eye's origin in the left camera's frame, so
/// P = K * [I | t] and P[3] = fx * tx + cx * tz, which is what a stereo matcher
/// reads as the baseline.  The left eye passes {0, 0, 0} by definition -- it is
/// the frame the right eye is measured in -- and gets P[3] == 0, the convention
/// stereoRectify and the official dual-camera calibration both produce.
///
/// `translation` must be relative_T(LEFT, RIGHT), the left frame's points seen
/// from the right eye, whose x component is -baseline.  Passing the opposite
/// direction flips the sign of every depth a consumer computes.
Projection stereo_projection(
  const gs130_camera_intrinsics_t & intrinsics, const double translation[3]);

/// Build the sensor_msgs/CameraInfo for one eye.
///
/// `width` and `height` describe **one eye**, which is what the official
/// calibration files declare and what downstream consumers scale against; on a
/// stitched topic that is half of the image message's size, not all of it.
///
/// R is the identity: CameraInfo.R is the rotation that takes a point into the
/// rectified frame, and a frame this node publishes is either already
/// rectified or not rectified at all.  The device extrinsics are delivered
/// through P and TF instead, never here.
sensor_msgs::msg::CameraInfo camera_info(
  const gs130_camera_intrinsics_t & intrinsics, uint32_t width, uint32_t height,
  const std::string & frame_id, const builtin_interfaces::msg::Time & stamp,
  const Projection * projection, bool rectified);

/// Build the sensor_msgs/Imu for one sample.
///
/// The orientation is not estimated, so it is marked absent the way
/// sensor_msgs/Imu prescribes: the first covariance entry set to -1.  The
/// acceleration and rate covariances stay at zero, meaning "unknown"; the
/// noise densities the EEPROM holds are continuous-time densities, and turning
/// one into a per-sample variance needs a bandwidth this node does not have.
sensor_msgs::msg::Imu imu_message(
  const gs130_imu_packet_t & packet, const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp);

/// Rotation matrix to quaternion, normalised.  False when it is not usable.
///
/// Uses the largest diagonal term to pick a stable branch, so rotations near
/// 180 degrees do not divide by a vanishing scale factor.
bool quaternion_from_rotation(const double rotation[9], double quaternion[4]);

/// Build a TransformStamped describing `child` in `parent`.
///
/// `rotation` and `translation` are the pair gs130_get_relative_R and
/// gs130_get_relative_T produce, which already mean "take a point in `from`
/// into `to`" -- the direction TF wants for a transform from `to` to `from`.
/// The values are passed through untouched: the device's axis convention has
/// not been checked against REP-103, and rewriting them here would be a guess
/// that silently reorients every consumer.
geometry_msgs::msg::TransformStamped transform_message(
  const double rotation[9], const double translation[3],
  const std::string & parent, const std::string & child,
  const builtin_interfaces::msg::Time & stamp);

}  // namespace gs130_ros

#endif  // GS130_ROS__CONVERT_HPP_
