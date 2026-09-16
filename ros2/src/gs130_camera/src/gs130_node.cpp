// Copyright (c) 2026 D-Robotics.
// SPDX-License-Identifier: MIT
//
// Maps the GS130 stereo camera and IMU onto standard ROS 2 messages.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

#include "gs130.h"
#include "config.h"

namespace gs130_camera
{
namespace
{

constexpr int kImageQosDepth = 5;   // matches mipi_cam
constexpr int kImuQosDepth = 10;

/// NV12 is a Y plane followed by one interleaved UV plane.
size_t nv12_bytes(const gs130_image_nv12_t & frame)
{
  return static_cast<size_t>(frame.width) * frame.height * 3 / 2;
}

/// The frame is copied out of the SDK's buffer, which the caller still owns.
std::unique_ptr<sensor_msgs::msg::Image> make_image(
  const gs130_image_nv12_t & frame, const char * encoding, size_t bytes,
  const std::string & frame_id, const rclcpp::Time & stamp)
{
  auto image = std::make_unique<sensor_msgs::msg::Image>();
  image->header.stamp = stamp;
  image->header.frame_id = frame_id;
  image->height = frame.height;
  image->width = frame.width;
  image->encoding = encoding;
  image->is_bigendian = false;
  image->step = frame.width;
  image->data.resize(bytes);
  std::memcpy(image->data.data(), frame.data, bytes);
  return image;
}

/// The Y plane of NV12 is already the grayscale image.
std::unique_ptr<sensor_msgs::msg::Image> make_gray(
  const gs130_image_nv12_t & frame, const std::string & frame_id, const rclcpp::Time & stamp)
{
  return make_image(
    frame, "mono8", static_cast<size_t>(frame.width) * frame.height, frame_id, stamp);
}

std::unique_ptr<sensor_msgs::msg::Image> make_nv12(
  const gs130_image_nv12_t & frame, const std::string & frame_id, const rclcpp::Time & stamp)
{
  return make_image(frame, "nv12", nv12_bytes(frame), frame_id, stamp);
}

/// Shepperd's method, for a row-major 3x3 rotation matrix.
geometry_msgs::msg::Quaternion quaternion_from_R(const double R[9])
{
  const double trace = R[0] + R[4] + R[8];
  double w = 0.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  if (trace > 0.0) {
    const double s = std::sqrt(trace + 1.0) * 2.0;
    w = 0.25 * s;
    x = (R[7] - R[5]) / s;
    y = (R[2] - R[6]) / s;
    z = (R[3] - R[1]) / s;
  } else if (R[0] > R[4] && R[0] > R[8]) {
    const double s = std::sqrt(1.0 + R[0] - R[4] - R[8]) * 2.0;
    w = (R[7] - R[5]) / s;
    x = 0.25 * s;
    y = (R[1] + R[3]) / s;
    z = (R[2] + R[6]) / s;
  } else if (R[4] > R[8]) {
    const double s = std::sqrt(1.0 + R[4] - R[0] - R[8]) * 2.0;
    w = (R[2] - R[6]) / s;
    x = (R[1] + R[3]) / s;
    y = 0.25 * s;
    z = (R[5] + R[7]) / s;
  } else {
    const double s = std::sqrt(1.0 + R[8] - R[0] - R[4]) * 2.0;
    w = (R[3] - R[1]) / s;
    x = (R[2] + R[6]) / s;
    y = (R[5] + R[7]) / s;
    z = 0.25 * s;
  }

  geometry_msgs::msg::Quaternion quaternion;
  quaternion.w = w;
  quaternion.x = x;
  quaternion.y = y;
  quaternion.z = z;
  return quaternion;
}

gs130_camera_mode_t camera_mode_of(const std::string & mode)
{
  if (mode == "raw") {
    return GS130_CAMERA_MODE_RAW;
  }
  if (mode == "resize") {
    return GS130_CAMERA_MODE_RESIZE;
  }
  if (mode == "rect") {
    return GS130_CAMERA_MODE_RECT;
  }
  throw std::invalid_argument("camera_mode must be raw, rect, or resize");
}

gs130_stereo_layout_t stitch_of(const std::string & stitch)
{
  if (stitch == "none") {
    return GS130_STEREO_LAYOUT_NONE;
  }
  if (stitch == "left_right") {
    return GS130_STEREO_LAYOUT_LEFT_RIGHT;
  }
  if (stitch == "right_left") {
    return GS130_STEREO_LAYOUT_RIGHT_LEFT;
  }
  if (stitch == "top_bottom") {
    return GS130_STEREO_LAYOUT_TOP_BOTTOM;
  }
  if (stitch == "bottom_top") {
    return GS130_STEREO_LAYOUT_BOTTOM_TOP;
  }
  throw std::invalid_argument(
    "stitch must be none, left_right, right_left, top_bottom, or bottom_top");
}

}  // namespace

class Gs130Node : public rclcpp::Node
{
public:
  Gs130Node();
  ~Gs130Node() override;

  /// Begins capture.  Blocks until the IMU FSYNC handshake completes.
  void start();

private:
  void declare_parameters();
  gs130_config_t build_config() const;
  void open_device();
  void load_calibration();
  void create_publishers();

  /// Runs every timer period: publishes whichever stream has the older sample.
  void tick();
  void publish_camera();
  void publish_imu();
  void publish_transforms();
  void release_frames();

  sensor_msgs::msg::CameraInfo camera_info_for(bool left_eye, const rclcpp::Time & stamp) const;

  // Parameters.
  std::string device_model_;
  std::string camera_mode_;
  std::string stitch_;
  std::string image_topic_;
  std::string left_image_topic_;
  std::string right_image_topic_;
  std::string imu_topic_;
  std::string frame_id_;
  std::string right_frame_id_;
  std::string imu_frame_id_;
  uint32_t eye_width_ = 0;
  uint32_t eye_height_ = 0;
  uint32_t fps_ = 30;
  uint32_t odr_ = 200;
  int64_t timer_period_ms_ = 1;
  bool gray_ = false;
  bool stitched_ = false;

  // Device.
  gs130_device_t * device_ = nullptr;
  bool imu_present_ = false;
  bool calibrated_ = false;
  gs130_camera_intrinsics_t intrinsics_left_ = {};
  gs130_camera_intrinsics_t intrinsics_right_ = {};
  double stereo_T_[3] = {0.0, 0.0, 0.0};         // left -> right, for the baseline in P[3]
  double right_R_[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  double right_T_[3] = {0.0, 0.0, 0.0};          // right in the left frame, for tf
  double imu_R_[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  double imu_T_[3] = {0.0, 0.0, 0.0};            // imu in the left frame, for tf

  /// One pending sample per stream, so the two can be compared by timestamp.
  bool frame_ready_ = false;
  gs130_image_nv12_t frame_left_ = {};
  gs130_image_nv12_t frame_right_ = {};          // unused when stitched
  bool packet_ready_ = false;
  gs130_imu_packet_t packet_ = {};

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr gray_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_gray_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_gray_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

Gs130Node::Gs130Node()
: rclcpp::Node("gs130")
{
  declare_parameters();
  open_device();
  load_calibration();
  create_publishers();
  publish_transforms();

  RCLCPP_INFO(
    get_logger(),
    "GS130 ready: %s %s %s %ux%u fps=%u odr=%u stitch=%s gray=%s imu=%s calibration=%s (sdk %s)",
    gs130_platform(), device_model_.c_str(), camera_mode_.c_str(),
    eye_width_, eye_height_, fps_, odr_, stitch_.c_str(), gray_ ? "on" : "off",
    imu_present_ ? "yes" : "no", calibrated_ ? "yes" : "no", gs130_version());
}

void Gs130Node::declare_parameters()
{
  device_model_ = declare_parameter<std::string>("device", "GS130WI");
  camera_mode_ = declare_parameter<std::string>("camera_mode", "rect");
  stitch_ = declare_parameter<std::string>("stitch", "none");

  // Topic names, one parameter each, defaulting to the names mipi_cam uses.  The
  // camera_info and gray topics are derived from the image topic they belong to.
  image_topic_ = declare_parameter<std::string>("image_topic", "image_combine");
  left_image_topic_ = declare_parameter<std::string>("left_image_topic", "image_left");
  right_image_topic_ = declare_parameter<std::string>("right_image_topic", "image_right");
  imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu_data");

  const int64_t output_width = declare_parameter<int64_t>("output_width", 544);
  const int64_t output_height = declare_parameter<int64_t>("output_height", 448);
  const int64_t fps = declare_parameter<int64_t>("fps", 30);
  const int64_t odr = declare_parameter<int64_t>("odr", 200);
  timer_period_ms_ = declare_parameter<int64_t>("timer_period_ms", 1);
  gray_ = declare_parameter<bool>("publish_gray", false);

  frame_id_ = declare_parameter<std::string>("frame_id", "camera_link");
  right_frame_id_ = declare_parameter<std::string>("right_frame_id", "camera_right_link");
  imu_frame_id_ = declare_parameter<std::string>("imu_frame_id", "imu_link");

  if (output_width <= 0 || output_width > std::numeric_limits<uint32_t>::max() ||
    output_height <= 0 || output_height > std::numeric_limits<uint32_t>::max() ||
    fps <= 0 || fps > std::numeric_limits<uint32_t>::max() ||
    odr <= 0 || odr > std::numeric_limits<uint32_t>::max())
  {
    throw std::invalid_argument("output_width, output_height, fps, and odr must be positive uint32 values");
  }
  if (timer_period_ms_ <= 0) {
    throw std::invalid_argument("timer_period_ms must be positive");
  }

  eye_width_ = static_cast<uint32_t>(output_width);
  eye_height_ = static_cast<uint32_t>(output_height);
  fps_ = static_cast<uint32_t>(fps);
  odr_ = static_cast<uint32_t>(odr);
  stitched_ = stitch_of(stitch_) != GS130_STEREO_LAYOUT_NONE;
}

gs130_config_t Gs130Node::build_config() const
{
  // This C bridge expands GS130_CONFIG from gs130_define.h with GNU C99 syntax.
  return gs130_camera_config_from_define(
    device_model_.c_str(), camera_mode_of(camera_mode_),
    eye_width_, eye_height_, fps_, odr_, stitch_of(stitch_));
}

void Gs130Node::open_device()
{
  const gs130_config_t config = build_config();

  device_ = gs130_create();
  if (device_ == nullptr) {
    throw std::runtime_error("gs130_create failed");
  }

  const gs130_err_t error = gs130_init(device_, &config);
  if (error != GS130_OK) {
    gs130_destroy(device_);
    device_ = nullptr;
    throw std::runtime_error(
      "gs130_init failed: " + std::to_string(static_cast<int>(error)));
  }

  // Both are optional: the SDK degrades instead of failing when it finds neither.
  const char * imu_name = gs130_get_imu_name(device_);
  imu_present_ = imu_name != nullptr && imu_name[0] != '\0';
  const char * eeprom_name = gs130_get_eeprom_name(device_);
  calibrated_ = eeprom_name != nullptr && eeprom_name[0] != '\0';
}

void Gs130Node::load_calibration()
{
  if (!calibrated_) {
    return;
  }

  gs130_get_camera_intrinsics(device_, GS130_CAMERA_LEFT_IDX, &intrinsics_left_);
  gs130_get_camera_intrinsics(device_, GS130_CAMERA_RIGHT_IDX, &intrinsics_right_);
  gs130_get_relative_T(device_, GS130_REF_CAMERA_LEFT, GS130_REF_CAMERA_RIGHT, stereo_T_);
  gs130_get_relative_R(device_, GS130_REF_CAMERA_RIGHT, GS130_REF_CAMERA_LEFT, right_R_);
  gs130_get_relative_T(device_, GS130_REF_CAMERA_RIGHT, GS130_REF_CAMERA_LEFT, right_T_);

  if (imu_present_) {
    gs130_get_relative_R(device_, GS130_REF_IMU, GS130_REF_CAMERA_LEFT, imu_R_);
    gs130_get_relative_T(device_, GS130_REF_IMU, GS130_REF_CAMERA_LEFT, imu_T_);
  }
}

void Gs130Node::create_publishers()
{
  const rclcpp::QoS image_qos = rclcpp::QoS(rclcpp::KeepLast(kImageQosDepth));

  if (stitched_) {
    image_publisher_ = create_publisher<sensor_msgs::msg::Image>(image_topic_, image_qos);
    if (gray_) {
      gray_publisher_ = create_publisher<sensor_msgs::msg::Image>(image_topic_ + "/gray", image_qos);
    }
  } else {
    left_image_publisher_ = create_publisher<sensor_msgs::msg::Image>(left_image_topic_, image_qos);
    right_image_publisher_ = create_publisher<sensor_msgs::msg::Image>(right_image_topic_, image_qos);
    if (gray_) {
      left_gray_publisher_ =
        create_publisher<sensor_msgs::msg::Image>(left_image_topic_ + "/gray", image_qos);
      right_gray_publisher_ =
        create_publisher<sensor_msgs::msg::Image>(right_image_topic_ + "/gray", image_qos);
    }
  }

  // Calibration is what makes camera_info possible at all.
  if (calibrated_) {
    const std::string left_info_topic =
      stitched_ ? image_topic_ + "/left/camera_info" : left_image_topic_ + "/camera_info";
    const std::string right_info_topic =
      stitched_ ? image_topic_ + "/right/camera_info" : right_image_topic_ + "/camera_info";
    left_info_publisher_ =
      create_publisher<sensor_msgs::msg::CameraInfo>(left_info_topic, image_qos);
    right_info_publisher_ =
      create_publisher<sensor_msgs::msg::CameraInfo>(right_info_topic, image_qos);
  }

  if (imu_present_) {
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::QoS(rclcpp::KeepLast(kImuQosDepth)));
  }
}

void Gs130Node::publish_transforms()
{
  if (!calibrated_) {
    return;
  }

  if (tf_broadcaster_ == nullptr) {
    tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
  }

  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  const rclcpp::Time stamp = now();

  geometry_msgs::msg::TransformStamped right;
  right.header.stamp = stamp;
  right.header.frame_id = frame_id_;
  right.child_frame_id = right_frame_id_;
  right.transform.translation.x = right_T_[0];
  right.transform.translation.y = right_T_[1];
  right.transform.translation.z = right_T_[2];
  right.transform.rotation = quaternion_from_R(right_R_);
  transforms.push_back(right);

  // Without an IMU there is no imu frame, so nothing IMU-shaped is broadcast.
  if (imu_present_) {
    geometry_msgs::msg::TransformStamped imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = frame_id_;
    imu.child_frame_id = imu_frame_id_;
    imu.transform.translation.x = imu_T_[0];
    imu.transform.translation.y = imu_T_[1];
    imu.transform.translation.z = imu_T_[2];
    imu.transform.rotation = quaternion_from_R(imu_R_);
    transforms.push_back(imu);
  }

  tf_broadcaster_->sendTransform(transforms);
}

sensor_msgs::msg::CameraInfo Gs130Node::camera_info_for(
  bool left_eye, const rclcpp::Time & stamp) const
{
  const gs130_camera_intrinsics_t & intrinsics = left_eye ? intrinsics_left_ : intrinsics_right_;

  sensor_msgs::msg::CameraInfo info;
  info.header.stamp = stamp;
  info.header.frame_id = left_eye ? frame_id_ : right_frame_id_;
  info.width = eye_width_;
  info.height = eye_height_;
  info.distortion_model =
    intrinsics.dist_model == GS130_DIST_FISHEYE ? "equidistant" : "plumb_bob";
  for (size_t i = 0; i < 9; ++i) {
    info.k[i] = intrinsics.K[i];
  }
  // d is the one camera matrix here that ROS 2 declares as a std::vector rather
  // than a std::array, so it starts empty and has to be given its length before
  // it can be indexed.  Indexing it first writes out of bounds.
  info.d.assign(intrinsics.dist_coeffs, intrinsics.dist_coeffs + 8);

  // The pair is already rectified, so R is identity and the baseline rides in P.
  info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  const double baseline_x = left_eye ? 0.0 : intrinsics.fx * stereo_T_[0] + intrinsics.cx * stereo_T_[2];
  info.p = {
    intrinsics.fx, 0.0, intrinsics.cx, baseline_x,
    0.0, intrinsics.fy, intrinsics.cy, 0.0,
    0.0, 0.0, 1.0, 0.0};
  return info;
}

void Gs130Node::release_frames()
{
  free(frame_left_.data);
  free(frame_right_.data);
  frame_left_ = gs130_image_nv12_t{};
  frame_right_ = gs130_image_nv12_t{};
}

void Gs130Node::publish_camera()
{
  const rclcpp::Time stamp = now();

  if (calibrated_) {
    left_info_publisher_->publish(camera_info_for(true, stamp));
    right_info_publisher_->publish(camera_info_for(false, stamp));
  }

  if (stitched_) {
    image_publisher_->publish(make_nv12(frame_left_, frame_id_, stamp));
    if (gray_) {
      gray_publisher_->publish(make_gray(frame_left_, frame_id_, stamp));
    }
  } else {
    left_image_publisher_->publish(make_nv12(frame_left_, frame_id_, stamp));
    right_image_publisher_->publish(make_nv12(frame_right_, right_frame_id_, stamp));
    if (gray_) {
      left_gray_publisher_->publish(make_gray(frame_left_, frame_id_, stamp));
      right_gray_publisher_->publish(make_gray(frame_right_, right_frame_id_, stamp));
    }
  }

  release_frames();
}

void Gs130Node::publish_imu()
{
  sensor_msgs::msg::Imu message;
  message.header.stamp = now();
  message.header.frame_id = imu_frame_id_;

  // The device reports no orientation: identity plus the first covariance element
  // set to -1 is how sensor_msgs/Imu says "no estimate here".
  message.orientation.w = 1.0;
  message.orientation_covariance[0] = -1.0;

  message.angular_velocity.x = packet_.gyro[0];
  message.angular_velocity.y = packet_.gyro[1];
  message.angular_velocity.z = packet_.gyro[2];

  message.linear_acceleration.x = packet_.accel[0];
  message.linear_acceleration.y = packet_.accel[1];
  message.linear_acceleration.z = packet_.accel[2];

  // All-zero covariance is how sensor_msgs/Imu says "covariance unknown".
  imu_publisher_->publish(message);
}

void Gs130Node::tick()
{
  // Hold one sample per stream, publish the older one, then refill that side, so
  // image and IMU leave in timestamp order and neither overtakes the other.
  for (;;) {
    if (!frame_ready_ && gs130_available_camera(device_) > 0) {
      const gs130_err_t error = stitched_ ?
        gs130_get_stereo_nv12_frame(device_, &frame_left_) :
        gs130_get_nv12_frame(device_, &frame_left_, &frame_right_);
      frame_ready_ = error == GS130_OK;
    }

    if (!packet_ready_ && imu_present_ && gs130_available_imu(device_) > 0) {
      packet_ready_ = gs130_get_imu_packet(device_, &packet_) == GS130_OK;
    }

    if (!frame_ready_) {
      return;
    }
    if (imu_present_ && !packet_ready_) {
      return;  // hold the frame until there is an IMU sample to compare against
    }

    if (!imu_present_ || frame_left_.timestamp_ns <= packet_.timestamp_ns) {
      publish_camera();
      frame_ready_ = false;
    } else {
      publish_imu();
      packet_ready_ = false;
    }
  }
}

void Gs130Node::start()
{
  const gs130_err_t error = gs130_start(device_);
  if (error != GS130_OK) {
    throw std::runtime_error(
      "gs130_start failed: " + std::to_string(static_cast<int>(error)));
  }

  timer_ = create_wall_timer(
    std::chrono::milliseconds(timer_period_ms_), [this]() {tick();});
}

Gs130Node::~Gs130Node()
{
  if (timer_) {
    timer_->cancel();
  }
  release_frames();

  if (device_ != nullptr) {
    // deinit() stops the threads itself, but stopping explicitly means none can
    // still be running should it fail before it gets that far.
    gs130_stop(device_);
    gs130_deinit(device_);
    gs130_destroy(device_);
    device_ = nullptr;
  }
}

}  // namespace gs130_camera

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int status = 0;
  try {
    auto node = std::make_shared<gs130_camera::Gs130Node>();
    node->start();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("gs130_camera"), "%s", error.what());
    status = 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return status;
}
