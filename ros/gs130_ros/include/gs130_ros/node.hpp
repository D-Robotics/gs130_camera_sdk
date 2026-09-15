// Copyright (c) 2026 D-Robotics.
// SPDX-License-Identifier: MIT
//
// A ROS 2 node that publishes the GS130 stereo camera and its IMU.
//
// The node is a thin mapping of the C API onto standard ROS messages: it opens
// one gs130_device_t, drains its frame and IMU queues, and publishes what comes
// out.  It does no image processing, no stereo matching and no filtering --
// those belong to whatever consumes the topics.
//
// Everything a deployment varies -- board, camera model, mode, size, rate,
// stereo packing, topic names -- is a parameter, so one launch file covers the
// hardware without a recompile.  The defaults follow D-Robotics' hobot_mipi_cam
// so the streams drop into the official perception stack; see README.md for the
// details and for the places where this node deliberately differs.

#ifndef GS130_ROS__NODE_HPP_
#define GS130_ROS__NODE_HPP_

#include <atomic>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

#include "gs130.h"

#include "gs130_ros/config.hpp"
#include "gs130_ros/convert.hpp"

namespace gs130_ros
{

/// Decides what stamp a message gets, and holds that decision.
///
/// kDevice passes the SDK's value through untouched, kSystem always uses the
/// moment the sample reached this node, and kAuto keeps the device value only
/// while the first samples look usable: increasing per stream, on the Unix
/// epoch and agreeing with the IMU.  A clock that fails any of those would
/// place every frame at the wrong time without saying so, which is worse than
/// stamping late but correctly.
class TimestampPolicy
{
public:
  static constexpr size_t kProbe = 20;
  static constexpr int64_t kEpochFloorNs = 1577836800000000000LL;  // 2020-01-01
  static constexpr int64_t kSlackNs = 86400000000000LL;            // one day
  static constexpr int64_t kSkewNs = 1000000000LL;                 // one second

  TimestampPolicy(const std::string & source, rclcpp::Clock::SharedPtr clock);

  /// Take one camera stamp.  `stream` names the topic it belongs to.
  ///
  /// Monotonicity is judged per stream, because the two eyes of one pair carry
  /// the same stamp: fed through one list they would look like a clock that
  /// stalls every other sample, and a healthy device would be thrown away.
  rclcpp::Time camera(int64_t device_ns, const std::string & stream);

  rclcpp::Time imu(int64_t device_ns);

  /// Whether the clock is still being judged, so nothing should publish.
  bool probing() const;

  /// Whether enough camera samples have arrived to judge the clock.
  bool ready() const;

  /// Latch the decision; returns the problems found, empty when there are none.
  std::vector<std::string> judge(int64_t wall_ns);

private:
  rclcpp::Time stamp(int64_t device_ns) const;

  rclcpp::Clock::SharedPtr clock_;
  bool use_device_;
  bool decided_;
  std::map<std::string, std::vector<int64_t>> camera_;
  std::vector<int64_t> imu_;
  mutable std::mutex mutex_;
};

class Gs130Node : public rclcpp::Node
{
public:
  Gs130Node();
  ~Gs130Node() override;

private:
  // -- parameters --
  void declare_parameters();
  gs130_config_t build_config();
  gs130_camera_mode_t camera_mode(const std::string & name) const;
  gs130_stereo_layout_t stereo_layout(const std::string & name) const;

  // -- device --
  void open_device();
  void load_calibration();
  void measure_stereo_translation();
  void create_publishers();
  bool imu_is_usable();
  void publish_extrinsics();

  // -- capture --
  void start_capture_threads();
  void stop_capture_threads();
  void camera_loop();
  void imu_loop();
  void sleep_for(double seconds);
  void camera_tick();
  void imu_tick();
  void probe_clock();
  bool any_image_subscriber();
  void judge_timestamps();

  // -- publishing --
  sensor_msgs::msg::CameraInfo camera_info_for(
    const std::string & eye, uint32_t width, uint32_t height,
    const builtin_interfaces::msg::Time & stamp);
  void publish_stitched(const gs130_image_nv12_t & frame, const rclcpp::Time & stamp);
  void publish_eye(
    const std::string & eye, const gs130_image_nv12_t & frame,
    const rclcpp::Time & stamp);

  // -- state --
  gs130_device_t * device_ = nullptr;
  bool closing_ = false;
  bool stitched_ = false;
  bool rectified_ = false;
  bool publish_info_ = false;
  bool only_when_subscribed_ = false;
  bool publish_left_right_ = false;
  size_t camera_drain_ = 4;
  size_t imu_drain_ = 1;
  uint32_t framerate_ = 30;
  std::string mode_name_;
  std::string layout_name_;
  std::string frame_id_;
  std::string imu_frame_id_;
  gs130_stereo_layout_t layout_ = GS130_STEREO_LAYOUT_NONE;

  bool have_calibration_ = false;
  gs130_camera_intrinsics_t intrinsics_[2];  // 0 = left, 1 = right
  bool have_stereo_translation_ = false;
  double stereo_translation_[3] = {0.0, 0.0, 0.0};

  std::unique_ptr<TimestampPolicy> timestamps_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stitched_publisher_;
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> stitched_info_;
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> raw_publishers_;
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> raw_infos_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr extrinsic_publisher_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;

  // Capture runs on its own threads rather than on rclcpp timers.  The
  // official node does the same, and it is not only a matter of taste: on the
  // TROS rclcpp a timer bound to an explicitly created callback group never
  // fires under MultiThreadedExecutor, so every frame is silently dropped,
  // while the same timer on the default group works.  Threads do not depend on
  // the executor at all.
  std::thread camera_thread_;
  std::thread imu_thread_;
  std::atomic<bool> running_{false};
  std::mutex sleep_mutex_;
  std::condition_variable sleep_cv_;
};

}  // namespace gs130_ros

#endif  // GS130_ROS__NODE_HPP_
