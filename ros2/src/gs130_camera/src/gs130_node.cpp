/**
 * @file gs130_node.cpp
 * @brief ROS 2 node for publishing GS130 stereo images, calibration, IMU data, and TF.
 *
 * The node owns one SDK device handle, copies SDK-allocated frame buffers into
 * sensor_msgs messages, publishes camera calibration when an EEPROM is present,
 * and broadcasts the calibrated sensor transforms.
 *
 * This file is part of gs130_camera_sdk (https://github.com/D-Robotics/gs130_camera_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */

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

// Keep image QoS compatible with the existing mipi_cam topic convention.
constexpr int kImageQosDepth = 5;
constexpr int kImuQosDepth = 10;

/** @return Size in bytes of a tightly packed NV12 frame. */
size_t nv12_bytes(const gs130_image_nv12_t & frame)
{
  return static_cast<size_t>(frame.width) * frame.height * 3 / 2;
}

/**
 * @brief Copy one SDK frame into a ROS image message.
 *
 * @param[in] frame Source frame; its pixel buffer remains owned by the caller.
 * @param[in] encoding ROS image-encoding string.
 * @param[in] bytes Number of source bytes to copy.
 * @param[in] frame_id Coordinate frame assigned to the message.
 * @param[in] stamp ROS timestamp assigned to the message.
 * @return A complete sensor_msgs/Image message owning its copied pixel data.
 */
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

/** @brief Copy the NV12 luma plane into a mono8 ROS image. */
std::unique_ptr<sensor_msgs::msg::Image> make_gray(
  const gs130_image_nv12_t & frame, const std::string & frame_id, const rclcpp::Time & stamp)
{
  return make_image(
    frame, "mono8", static_cast<size_t>(frame.width) * frame.height, frame_id, stamp);
}

/** @brief Copy a complete tightly packed NV12 frame into a ROS image. */
std::unique_ptr<sensor_msgs::msg::Image> make_nv12(
  const gs130_image_nv12_t & frame, const std::string & frame_id, const rclcpp::Time & stamp)
{
  return make_image(frame, "nv12", nv12_bytes(frame), frame_id, stamp);
}

/**
 * @brief Convert a row-major 3x3 rotation matrix to a quaternion.
 * @param[in] R Proper rotation matrix in row-major order.
 * @return Quaternion representing the same rotation, computed with Shepperd's method.
 */
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

/** Convert a validated ROS parameter string to the SDK camera-mode enum. */
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

/** Convert a validated ROS parameter string to the SDK stereo-layout enum. */
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

/**
 * @brief Owns one GS130 device and publishes its camera and IMU streams.
 *
 * Construction declares and validates parameters, initializes the SDK, reads
 * calibration, creates publishers, and emits static transforms. start() begins
 * capture and installs the publishing timer. The destructor stops capture, frees
 * any pending SDK frame buffers, and destroys the device handle.
 *
 * This class adds no synchronization of its own. Stream state is consumed by the
 * single wall-timer callback created in start().
 */
class Gs130Node : public rclcpp::Node
{
public:
  Gs130Node();
  ~Gs130Node() override;

  /**
   * @brief Start SDK capture and create the publishing timer.
   *
   * gs130_start() launches the SDK worker threads and returns; when an IMU is
   * present, the camera worker waits for the FSYNC handshake in the background.
   */
  void start();

private:
  void declare_parameters();
  gs130_config_t build_config() const;
  void open_device();
  void load_calibration();
  void create_publishers();

  /** Fetch at most one pending item per stream and publish the older SDK timestamp. */
  void timer_callback();
  void publish_camera();
  void publish_imu();
  void publish_transforms();
  void release_frames();

  sensor_msgs::msg::CameraInfo camera_info_for(bool left_eye, const rclcpp::Time & stamp) const;

  // Validated ROS parameters and values derived from them.
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

  // SDK device state and calibration cached after initialization.
  gs130_device_t * device_ = nullptr;
  bool imu_present_ = false;
  bool calibrated_ = false;
  gs130_camera_intrinsics_t intrinsics_left_ = {};
  gs130_camera_intrinsics_t intrinsics_right_ = {};
  double stereo_T_[3] = {0.0, 0.0, 0.0};         // Translation of the left-to-right transform; used by P[3].
  double right_R_[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  double right_T_[3] = {0.0, 0.0, 0.0};          // Right sensor pose expressed in the left-camera frame.
  double imu_R_[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  double imu_T_[3] = {0.0, 0.0, 0.0};            // IMU pose expressed in the left-camera frame.

  /** One caller-owned pending item per stream, retained until timestamp ordering selects it. */
  bool frame_ready_ = false;
  gs130_image_nv12_t frame_left_ = {};
  gs130_image_nv12_t frame_right_ = {};          // Remains empty for stitched output.
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

  // Topic parameters default to the existing mipi_cam naming convention. CameraInfo
  // and grayscale topic names are derived from their corresponding image topic.
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
  // Keep the GNU C preset macro in config.c; this translation unit remains C++17.
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

  // The IMU is optional in every mode. A calibration EEPROM may be absent only
  // when RAW initialization succeeded; RESIZE and RECT require calibration.
  const char * imu_name = gs130_get_imu_name(device_);
  imu_present_ = imu_name != nullptr && imu_name[0] != '\0';
  const char * eeprom_name = gs130_get_eeprom_name(device_);
  calibrated_ = eeprom_name != nullptr && eeprom_name[0] != '\0';
}

void Gs130Node::load_calibration()
{
  // Keep identity/zero defaults when RAW mode initialized without an EEPROM.
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

  // CameraInfo and sensor-frame transforms are published only when calibration exists.
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

  // Do not advertise an IMU child frame when no IMU was detected.
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
  // CameraInfo::d is a variable-length vector; copy all eight SDK coefficients.
  info.d.assign(intrinsics.dist_coeffs, intrinsics.dist_coeffs + 8);

  // No additional ROS-side rectification rotation is applied. P carries the
  // calibrated stereo translation; K and D retain the SDK mode's intrinsics.
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
  // SDK frame getters transfer malloc()-allocated buffers to the caller.
  free(frame_left_.data);
  free(frame_right_.data);
  frame_left_ = gs130_image_nv12_t{};
  frame_right_ = gs130_image_nv12_t{};
}

void Gs130Node::publish_camera()
{
  // ROS messages use publication time. The SDK hardware timestamp is retained only
  // for cross-stream ordering in timer_callback().
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
  // Match camera-message semantics: stamp at publication, use packet timestamp only
  // to preserve device sampling order across the two streams.
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

void Gs130Node::timer_callback()
{
  // One message per call, rather than one per message that happens to be ready.
  // The SDK hands over a frame's worth of IMU samples in a single batch, and
  // publishing the whole batch back to back makes a subscriber with a shallow
  // queue drop its tail.  At one message per period the timer is still fast
  // enough to carry both streams.
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

  // Hold one sample per stream and publish the older one, so image and IMU leave
  // in timestamp order and neither overtakes the other.
  if (!imu_present_ || frame_left_.timestamp_ns <= packet_.timestamp_ns) {
    publish_camera();
    frame_ready_ = false;
  } else {
    publish_imu();
    packet_ready_ = false;
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
    std::chrono::milliseconds(timer_period_ms_), [this]() {timer_callback();});
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
