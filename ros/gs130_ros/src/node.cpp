// Copyright (c) 2026 D-Robotics.
// SPDX-License-Identifier: MIT

#include "gs130_ros/node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <utility>

#include "rclcpp/callback_group.hpp"
#include "rclcpp/qos.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

namespace gs130_ros
{

namespace
{

// hobot_mipi_cam publishes images with this depth (hobot_mipi_node.cpp:29) and
// the IMU with 10 (:463).  Matching them keeps the two nodes interchangeable.
constexpr int kPubBufNum = 5;
constexpr int kImuBufNum = 10;

// The IMU queue is drained on a timer; polling faster than this buys nothing.
constexpr double kImuPollFloorSec = 0.001;

const char * const kLeft = "left";
const char * const kRight = "right";

const char * const kLayoutNames[] = {"none", "left_right", "right_left", "top_bottom", "bottom_top"};
const char * const kModeNames[] = {"raw", "resize", "rect"};

/// QoS for /imu_extrinsic: one sample, kept for late subscribers.
///
/// The official node builds this profile and then does not pass it to
/// create_publisher (hobot_mipi_node.cpp:470-472), so its extrinsic topic is a
/// race against the first IMU sample.  An extrinsic is a constant of the rig,
/// so here it is latched instead.
rclcpp::QoS extrinsic_qos()
{
  return rclcpp::QoS(1).reliable().transient_local();
}

/// Frees an SDK frame buffer.  The SDK allocates it with malloc() and hands
/// ownership to the caller, so it has to go back to free().
struct FrameBuffer
{
  gs130_image_nv12_t image{};

  FrameBuffer() = default;
  FrameBuffer(const FrameBuffer &) = delete;
  FrameBuffer & operator=(const FrameBuffer &) = delete;

  ~FrameBuffer()
  {
    if (image.data != nullptr) {
      free(image.data);
    }
  }
};

}  // namespace

// ---------------------------------------------------------------------------
// Gs130Error
// ---------------------------------------------------------------------------

namespace
{

const char * error_name(gs130_err_t code)
{
  switch (code) {
    case GS130_OK: return "OK";
    case GS130_PARAM_ERROR: return "PARAM_ERROR";
    case GS130_UNSUPPORTED: return "UNSUPPORTED";
    case GS130_NOT_FOUND: return "NOT_FOUND";
    case GS130_HW_ERROR: return "HW_ERROR";
    case GS130_TIMEOUT: return "TIMEOUT";
    case GS130_THREAD_CLOSED: return "THREAD_CLOSED";
    default: return "UNKNOWN";
  }
}

}  // namespace

Gs130Error::Gs130Error(gs130_err_t code, const std::string & function)
: std::runtime_error(function + "() -> " + error_name(code) + " (" + std::to_string(static_cast<int>(code)) + ")"),
  code_(code),
  function_(function)
{
}

void check(gs130_err_t code, const std::string & function)
{
  if (code != GS130_OK) {
    throw Gs130Error(code, function);
  }
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

gs130_config_t make_config(
  const std::string & platform, const std::string & device,
  gs130_camera_mode_t mode, uint32_t width, uint32_t height, uint32_t fps,
  uint32_t odr, gs130_stereo_layout_t layout)
{
  if (platform != "RDKX5" || (device != "GS130WI" && device != "GS130W")) {
    throw std::invalid_argument(
            "unsupported platform/device: " + platform + " " + device +
            "; the presets that exist are RDKX5/GS130WI and RDKX5/GS130W");
  }
  const bool with_imu = device == "GS130WI";

  gs130_config_t config{};
  // Every entry point the macros leave alone starts at its C zero value here
  // too, so this can be compared with gs130_define.h field by field.

  auto & camera = config.camera_config;
  const uint8_t camera_bus[] = {4, 6};
  std::copy(camera_bus, camera_bus + 2, camera.bus);
  camera.bus_num = 2;
  camera.left_addr = 0x30;
  camera.right_addr = with_imu ? 0x32 : 0x31;
  camera.sensor_width = 1088;
  camera.sensor_height = 1280;
  camera.fps = fps;
  camera.line_length = 1400;
  camera.frame_length = 1500;
  camera.tuning_file = nullptr;
  camera.output_width = width;
  camera.output_height = height;
  camera.mode = mode;
  camera.stereo_layout = layout;
  std::memset(camera.bus_mipi_rx, 0xFF, sizeof(camera.bus_mipi_rx));
  for (size_t i = 0; i < sizeof(camera.bus_reset_gpio) / sizeof(camera.bus_reset_gpio[0]); ++i) {
    camera.bus_reset_gpio[i] = -1;
  }
  camera.bus_mipi_rx[4] = 2;
  camera.bus_mipi_rx[6] = 0;
  camera.bus_reset_gpio[4] = 351;
  camera.bus_reset_gpio[6] = 353;
  camera.fsync_camera = GS130_CAMERA_RIGHT_IDX;

  auto & imu = config.imu_config;
  if (with_imu) {
    std::copy(camera_bus, camera_bus + 2, imu.bus);
    imu.bus_num = 2;
  }
  imu.addr = 0x68;
  imu.odr_hz = odr;
  imu.accel_fsr_g = 16;
  imu.gyro_fsr_dps = 2000;
  imu.accel_bw_sel = 0;
  imu.gyro_bw_sel = 0;

  auto & eeprom = config.eeprom_config;
  std::copy(camera_bus, camera_bus + 2, eeprom.bus);
  eeprom.bus_num = 2;
  eeprom.addr = 0x50;

  config.camera_fifo.depth = 4;
  config.camera_fifo.mode = GS130_FIFO_DROP_OLD;
  // GS130W has no IMU, and its macro has no .imu_fifo at all, so that queue
  // keeps the zeroed depth = 0 and DROP_NEW.
  if (with_imu) {
    config.imu_fifo.depth = 1024;
    config.imu_fifo.mode = GS130_FIFO_DROP_OLD;
  }
  return config;
}

// ---------------------------------------------------------------------------
// TimestampPolicy
// ---------------------------------------------------------------------------

TimestampPolicy::TimestampPolicy(const std::string & source, rclcpp::Clock::SharedPtr clock)
: clock_(std::move(clock)),
  use_device_(source != "system"),
  decided_(source != "auto")
{
  if (source != "auto" && source != "device" && source != "system") {
    throw std::invalid_argument(
            "timestamp_source must be one of auto, device, system, got " + source);
  }
}

rclcpp::Time TimestampPolicy::camera(int64_t device_ns, const std::string & stream)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!decided_) {
      camera_[stream].push_back(device_ns);
    }
  }
  return stamp(device_ns);
}

rclcpp::Time TimestampPolicy::imu(int64_t device_ns)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!decided_) {
      imu_.push_back(device_ns);
    }
  }
  return stamp(device_ns);
}

rclcpp::Time TimestampPolicy::stamp(int64_t device_ns) const
{
  if (use_device_) {
    return rclcpp::Time(device_ns);
  }
  // The clock is read per message.  Holding on to a time_t taken at
  // construction would stamp every message with the moment the node started.
  return clock_->now();
}

bool TimestampPolicy::probing() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return !decided_;
}

bool TimestampPolicy::ready() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (decided_) {
    return false;
  }
  size_t longest = 0;
  for (const auto & entry : camera_) {
    longest = std::max(longest, entry.second.size());
  }
  return longest >= kProbe;
}

std::vector<std::string> TimestampPolicy::judge(int64_t wall_ns)
{
  std::map<std::string, std::vector<int64_t>> camera;
  std::vector<int64_t> imu;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    decided_ = true;
    camera = camera_;
    imu = imu_;
  }

  std::vector<std::string> problems;
  int64_t first = 0;
  bool have_first = false;
  for (const auto & entry : camera) {
    const std::vector<int64_t> & stamps = entry.second;
    for (size_t i = 1; i < stamps.size(); ++i) {
      if (stamps[i] <= stamps[i - 1]) {
        problems.push_back("the " + entry.first + " timestamps do not increase");
        break;
      }
    }
    if (!stamps.empty()) {
      first = have_first ? std::min(first, stamps.front()) : stamps.front();
      have_first = true;
    }
  }

  if (have_first && first < kEpochFloorNs) {
    // A counter that starts near zero is time since boot, not a date.
    problems.push_back(
      "the first camera timestamp (" + std::to_string(first) + " ns, about " +
      std::to_string(first / 1000000000) + " s) looks like a monotonic clock "
      "rather than a Unix epoch");
  } else if (have_first && wall_ns >= kEpochFloorNs && first > wall_ns + kSlackNs) {
    // Only meaningful against a system clock that is set; an unset one says
    // nothing about the device.
    problems.push_back(
      "the first camera timestamp (" + std::to_string(first) +
      " ns) is more than a day ahead of the system clock (" +
      std::to_string(wall_ns) + " ns)");
  }

  if (!imu.empty() && have_first) {
    int64_t nearest = -1;
    for (const auto & entry : camera) {
      for (int64_t one : entry.second) {
        for (int64_t other : imu) {
          const int64_t distance = std::llabs(one - other);
          nearest = nearest < 0 ? distance : std::min(nearest, distance);
        }
      }
    }
    if (nearest > kSkewNs) {
      problems.push_back(
        "the camera and IMU clocks differ by " + std::to_string(nearest / 1000000000) + " s");
    }
  }

  if (!problems.empty()) {
    use_device_ = false;
  }
  return problems;
}

// ---------------------------------------------------------------------------
// Gs130Node
// ---------------------------------------------------------------------------

Gs130Node::Gs130Node()
: rclcpp::Node("gs130_ros")
{
  declare_parameters();

  mode_name_ = get_parameter("camera_mode").as_string();
  layout_name_ = get_parameter("stereo_layout").as_string();
  framerate_ = static_cast<uint32_t>(std::max<int64_t>(1, get_parameter("framerate").as_int()));
  frame_id_ = get_parameter("frame_id").as_string();
  imu_frame_id_ = get_parameter("imu_frame_id").as_string();
  only_when_subscribed_ = get_parameter("only_when_subscribed").as_bool();
  publish_left_right_ = get_parameter("publish_left_right").as_bool();
  timestamps_ = std::make_unique<TimestampPolicy>(
    get_parameter("timestamp_source").as_string(), get_clock());

  const gs130_config_t config = build_config();
  RCLCPP_INFO(
    get_logger(), "opening %s %s: %s mode, %s layout, %ux%u per eye at %u fps",
    get_parameter("platform").as_string().c_str(), get_parameter("device").as_string().c_str(),
    mode_name_.c_str(), layout_name_.c_str(), config.camera_config.output_width,
    config.camera_config.output_height, framerate_);

  open_device();

  camera_drain_ = std::max<size_t>(1, config.camera_fifo.depth);
  imu_drain_ = std::max<size_t>(1, config.imu_fifo.depth);
  load_calibration();
  create_publishers();
  publish_extrinsics();

  check(gs130_start(device_), "gs130_start");
  start_capture_threads();
  RCLCPP_INFO(
    get_logger(), "gs130 %s ready, library %s for %s", mode_name_.c_str(),
    gs130_version(), gs130_platform());
}

Gs130Node::~Gs130Node()
{
  stop_capture_threads();
  closing_ = true;
  if (device_ != nullptr) {
    // Releasing the camera stops its capture threads.  gs130_deinit stops them
    // itself, but stopping explicitly means none can still be running should
    // deinit fail before it gets that far.
    gs130_stop(device_);
    const gs130_err_t code = gs130_deinit(device_);
    if (code != GS130_OK) {
      RCLCPP_ERROR(get_logger(), "closing the camera failed: %s", error_name(code));
    }
    gs130_destroy(device_);
    device_ = nullptr;
  }
}

void Gs130Node::declare_parameters()
{
  declare_parameter("platform", "RDKX5");
  declare_parameter("device", "GS130WI");
  declare_parameter("camera_mode", "rect");
  declare_parameter("image_width", 1088);
  declare_parameter("image_height", 1280);
  declare_parameter("framerate", 30);
  declare_parameter("imu_odr", 200);
  declare_parameter("stereo_layout", "top_bottom");
  declare_parameter("frame_id", "camera_link");
  declare_parameter("imu_frame_id", "imu_link");
  declare_parameter("publish_imu", true);
  declare_parameter("publish_left_right", false);
  declare_parameter("only_when_subscribed", false);
  declare_parameter("timestamp_source", "auto");
  declare_parameter("topic_combine", "image_combine_raw");
  declare_parameter("topic_stereo", "image_stereo_raw");
  declare_parameter("topic_left", "image_left_raw");
  declare_parameter("topic_right", "image_right_raw");
  // Absolute, because the official node declares these two that way and a
  // consumer written against it expects them at the root.
  declare_parameter("topic_imu", "/imu_data");
  declare_parameter("topic_imu_extrinsic", "/imu_extrinsic");
}

gs130_camera_mode_t Gs130Node::camera_mode(const std::string & name) const
{
  for (int i = 0; i < 3; ++i) {
    if (name == kModeNames[i]) {
      return static_cast<gs130_camera_mode_t>(i);
    }
  }
  throw std::invalid_argument("camera_mode must be one of raw, resize, rect, got " + name);
}

gs130_stereo_layout_t Gs130Node::stereo_layout(const std::string & name) const
{
  for (int i = 0; i < 5; ++i) {
    if (name == kLayoutNames[i]) {
      return static_cast<gs130_stereo_layout_t>(i);
    }
  }
  throw std::invalid_argument(
          "stereo_layout must be one of none, left_right, right_left, top_bottom, "
          "bottom_top, got " + name);
}

gs130_config_t Gs130Node::build_config()
{
  layout_ = stereo_layout(layout_name_);
  return make_config(
    get_parameter("platform").as_string(), get_parameter("device").as_string(),
    camera_mode(mode_name_),
    static_cast<uint32_t>(get_parameter("image_width").as_int()),
    static_cast<uint32_t>(get_parameter("image_height").as_int()),
    framerate_,
    static_cast<uint32_t>(get_parameter("imu_odr").as_int()),
    layout_);
}

void Gs130Node::open_device()
{
  const gs130_config_t config = build_config();
  rectified_ = config.camera_config.mode == GS130_CAMERA_MODE_RECT;
  stitched_ = config.camera_config.stereo_layout != GS130_STEREO_LAYOUT_NONE;

  device_ = gs130_create();
  if (device_ == nullptr) {
    throw std::runtime_error("gs130_create() returned null");
  }
  try {
    check(gs130_init(device_, &config), "gs130_init");
  } catch (...) {
    // The handle was created but never initialized.  Release it and let the
    // original failure through.
    gs130_destroy(device_);
    device_ = nullptr;
    throw;
  }
}

void Gs130Node::load_calibration()
{
  // Under rect the C layer has already replaced the intrinsics and the
  // rotations with the virtual ones while initialising the camera, so a single
  // read here describes the frames this node is about to publish -- provided
  // it happens after gs130_init and not before it.
  gs130_calibration_t calibration{};
  const gs130_err_t code = gs130_get_calibration(device_, &calibration);
  if (code != GS130_OK) {
    RCLCPP_WARN(
      get_logger(),
      "no camera calibration in the EEPROM (gs130_get_calibration() -> %s); images go "
      "out without camera_info, which leaves them unusable for depth", error_name(code));
    return;
  }
  have_calibration_ = true;
  intrinsics_[0] = calibration.camera_left;
  intrinsics_[1] = calibration.camera_right;
  publish_info_ = true;
  measure_stereo_translation();
  if (rectified_ && !have_stereo_translation_) {
    // Rectified frames are only meaningful as a stereo pair, and a pair
    // without a baseline is worse than no calibration at all: the consumer
    // computes all-zero depth and reports success.
    publish_info_ = false;
  }
}

void Gs130Node::measure_stereo_translation()
{
  if (!rectified_) {
    return;
  }

  // relative_T(LEFT, RIGHT): the left eye's points seen from the right eye,
  // which is the direction P = K * [I | t] wants.  Its x component is
  // therefore -baseline, and P[3] = fx * tx comes out negative for a right eye
  // that sits at +x -- the sign stereoRectify and the official dual-camera
  // calibration both produce.  Taking the opposite direction, or the
  // magnitude, flips the sign of every depth a consumer computes.
  double translation[3] = {0.0, 0.0, 0.0};
  const gs130_err_t code = gs130_get_relative_T(
    device_, GS130_REF_CAMERA_LEFT, GS130_REF_CAMERA_RIGHT, translation);
  if (code != GS130_OK) {
    RCLCPP_ERROR(
      get_logger(),
      "no relative extrinsics between the eyes (gs130_get_relative_T() -> %s); "
      "withholding camera_info rather than publishing a zero baseline",
      error_name(code));
    return;
  }

  if (translation[0] == 0.0) {
    RCLCPP_ERROR(
      get_logger(),
      "the two eyes coincide in the rectified calibration; it holds no baseline, so "
      "camera_info is withheld");
    return;
  }

  std::copy(translation, translation + 3, stereo_translation_);
  have_stereo_translation_ = true;

  const double off_axis = std::max(
    std::fabs(translation[1]), std::fabs(translation[2])) / std::fabs(translation[0]);
  if (off_axis > 0.01) {
    RCLCPP_WARN(
      get_logger(),
      "the rectified pair is not parallel: the offset between the eyes is "
      "(%.6g, %.6g, %.6g), so %.1f%% of it is off the baseline axis; P carries the "
      "full fx*tx + cx*tz, but a consumer reading only P[3]/P[0] loses that part",
      translation[0], translation[1], translation[2], 100.0 * off_axis);
  } else {
    RCLCPP_INFO(
      get_logger(), "baseline %.6g, so the right eye's P[3] is %.6g",
      std::fabs(translation[0]), intrinsics_[0].fx * translation[0]);
  }
}

void Gs130Node::create_publishers()
{
  const auto qos = rclcpp::QoS(kPubBufNum);

  if (stitched_) {
    const bool on_official_topic = layout_ == GS130_STEREO_LAYOUT_TOP_BOTTOM;
    const std::string combine_topic = get_parameter("topic_combine").as_string();
    const std::string stereo_topic = get_parameter("topic_stereo").as_string();
    const std::string topic = on_official_topic ? combine_topic : stereo_topic;
    stitched_publisher_ = create_publisher<sensor_msgs::msg::Image>(topic, qos);
    if (!on_official_topic) {
      RCLCPP_WARN(
        get_logger(),
        "%s is not top_bottom, so the stitched frame goes out on %s and %s is left "
        "alone; hobot_stereonet assumes top_bottom with the left eye on top and "
        "cannot read this stream", layout_name_.c_str(), topic.c_str(), combine_topic.c_str());
    }
    if (publish_info_) {
      stitched_info_[kLeft] =
        create_publisher<sensor_msgs::msg::CameraInfo>(topic + "/left/camera_info", qos);
      stitched_info_[kRight] =
        create_publisher<sensor_msgs::msg::CameraInfo>(topic + "/right/camera_info", qos);
    }
  }

  // Two separate frames need their own topics.  A stitched frame does not, but
  // the official node publishes the pair alongside the combination
  // (hobot_mipi_node.cpp:263-275), so it is available on request.
  if (!stitched_ || publish_left_right_) {
    const std::string left_topic = get_parameter("topic_left").as_string();
    const std::string right_topic = get_parameter("topic_right").as_string();
    raw_publishers_[kLeft] = create_publisher<sensor_msgs::msg::Image>(left_topic, qos);
    raw_publishers_[kRight] = create_publisher<sensor_msgs::msg::Image>(right_topic, qos);
    if (publish_info_) {
      raw_infos_[kLeft] =
        create_publisher<sensor_msgs::msg::CameraInfo>(left_topic + "/camera_info", qos);
      raw_infos_[kRight] =
        create_publisher<sensor_msgs::msg::CameraInfo>(right_topic + "/camera_info", qos);
    }
  }

  if (imu_is_usable()) {
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
      get_parameter("topic_imu").as_string(), kImuBufNum);
  }
}

bool Gs130Node::imu_is_usable()
{
  if (!get_parameter("publish_imu").as_bool()) {
    RCLCPP_INFO(get_logger(), "IMU publishing is off");
    return false;
  }
  const char * name = gs130_get_imu_name(device_);
  if (name == nullptr) {
    RCLCPP_INFO(get_logger(), "no IMU detected on this device; publishing none");
    return false;
  }
  RCLCPP_INFO(get_logger(), "IMU detected: %s", name);
  return true;
}

void Gs130Node::publish_extrinsics()
{
  if (imu_publisher_ == nullptr) {
    return;
  }

  // camera_link -> imu_link, read from the device rather than from the
  // calibration's absolute poses.
  double rotation[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  double translation[3] = {0, 0, 0};
  const gs130_err_t rotation_code = gs130_get_relative_R(
    device_, GS130_REF_IMU, GS130_REF_CAMERA_LEFT, rotation);
  const gs130_err_t translation_code = gs130_get_relative_T(
    device_, GS130_REF_IMU, GS130_REF_CAMERA_LEFT, translation);
  if (rotation_code != GS130_OK || translation_code != GS130_OK) {
    RCLCPP_WARN(
      get_logger(), "no IMU extrinsics in the EEPROM; not publishing %s",
      get_parameter("topic_imu_extrinsic").as_string().c_str());
    return;
  }

  // The transform is a constant of the rig, so it goes on the static
  // broadcaster and on a topic that keeps its one sample for late
  // subscribers, stamped zero.  Neither now() nor an IMU sample time describes
  // a constant, and a consumer given one would be invited to interpolate it.
  const geometry_msgs::msg::TransformStamped message = transform_message(
    rotation, translation, frame_id_, imu_frame_id_, to_time(0));
  static_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
  static_tf_->sendTransform(message);

  extrinsic_publisher_ = create_publisher<geometry_msgs::msg::TransformStamped>(
    get_parameter("topic_imu_extrinsic").as_string(), extrinsic_qos());
  extrinsic_publisher_->publish(message);
  RCLCPP_INFO(
    get_logger(),
    "%s -> %s published; the axes are the device's own and are not yet checked "
    "against REP-103", frame_id_.c_str(), imu_frame_id_.c_str());
}

void Gs130Node::start_capture_threads()
{
  running_ = true;
  // One thread each, so a slow frame does not delay the IMU and a burst of IMU
  // samples does not stall the camera.
  camera_thread_ = std::thread([this]() {camera_loop();});
  if (imu_publisher_ != nullptr) {
    imu_thread_ = std::thread([this]() {imu_loop();});
  }
}

void Gs130Node::stop_capture_threads()
{
  if (!running_.exchange(false)) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(sleep_mutex_);
    sleep_cv_.notify_all();
  }
  if (camera_thread_.joinable()) {
    camera_thread_.join();
  }
  if (imu_thread_.joinable()) {
    imu_thread_.join();
  }
}

void Gs130Node::sleep_for(double seconds)
{
  std::unique_lock<std::mutex> lock(sleep_mutex_);
  sleep_cv_.wait_for(
    lock, std::chrono::duration<double>(seconds), [this]() {return !running_.load();});
}

void Gs130Node::camera_loop()
{
  const double period = 1.0 / static_cast<double>(framerate_);
  auto next = std::chrono::steady_clock::now();
  while (running_ && rclcpp::ok()) {
    camera_tick();
    next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(period));
    const auto now = std::chrono::steady_clock::now();
    if (now < next) {
      sleep_for(std::chrono::duration<double>(next - now).count());
    } else if (now > next + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(4 * period)))
    {
      // Far behind: the queue holds the newest frames and drops the oldest, so
      // catching up would only publish what is already stale.
      next = now;
    }
  }
}

void Gs130Node::imu_loop()
{
  const auto odr = static_cast<uint32_t>(
    std::max<int64_t>(1, get_parameter("imu_odr").as_int()));
  const double period = std::max(1.0 / static_cast<double>(odr), kImuPollFloorSec);
  while (running_ && rclcpp::ok()) {
    imu_tick();
    sleep_for(period);
  }
}

void Gs130Node::camera_tick()
{
  if (closing_) {
    return;
  }
  try {
    if (timestamps_->probing()) {
      probe_clock();
      return;
    }
    if (only_when_subscribed_ && !any_image_subscriber()) {
      // Nothing is listening, so nothing needs to be fresh.  The device queue
      // holds at most `depth` frames and drops the oldest, so an idle second
      // does not have to be drained later -- the next pass finds a current
      // frame.  The loop's own sleep is the idle wait.
      return;
    }

    for (size_t i = 0; i < camera_drain_; ++i) {
      if (stitched_) {
        FrameBuffer frame;
        if (gs130_get_stereo_nv12_frame(device_, &frame.image) != GS130_OK) {
          return;
        }
        const rclcpp::Time stamp =
          timestamps_->camera(static_cast<int64_t>(frame.image.timestamp_ns), "stitched");
        publish_stitched(frame.image, stamp);
      } else {
        FrameBuffer left;
        FrameBuffer right;
        if (gs130_get_nv12_frame(device_, &left.image, &right.image) != GS130_OK) {
          return;
        }
        const rclcpp::Time left_stamp =
          timestamps_->camera(static_cast<int64_t>(left.image.timestamp_ns), kLeft);
        const rclcpp::Time right_stamp =
          timestamps_->camera(static_cast<int64_t>(right.image.timestamp_ns), kRight);
        publish_eye(kLeft, left.image, left_stamp);
        publish_eye(kRight, right.image, right_stamp);
      }
    }
  } catch (const std::exception & error) {
    if (!closing_) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "the camera tick failed: %s", error.what());
    }
  }
}

void Gs130Node::imu_tick()
{
  if (closing_) {
    return;
  }
  try {
    const bool probing = timestamps_->probing();
    if (!probing && only_when_subscribed_ &&
      imu_publisher_->get_subscription_count() == 0)
    {
      return;
    }
    for (size_t i = 0; i < imu_drain_; ++i) {
      gs130_imu_packet_t packet{};
      if (gs130_get_imu_packet(device_, &packet) != GS130_OK) {
        return;
      }
      const rclcpp::Time stamp = timestamps_->imu(static_cast<int64_t>(packet.timestamp_ns));
      if (!probing) {
        imu_publisher_->publish(imu_message(packet, imu_frame_id_, stamp));
      }
    }
  } catch (const std::exception & error) {
    if (!closing_) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "the IMU tick failed: %s", error.what());
    }
  }
}

void Gs130Node::probe_clock()
{
  // Drain samples for the clock check, publishing none of them: a stamp the
  // node is about to disown must never reach a subscriber, because a consumer
  // that sees boot-clock stamps and then epoch ones sees a jump of decades.
  // The cost is a fraction of a second of frames at startup.
  for (size_t i = 0; i < camera_drain_; ++i) {
    if (stitched_) {
      FrameBuffer frame;
      if (gs130_get_stereo_nv12_frame(device_, &frame.image) != GS130_OK) {
        break;
      }
      timestamps_->camera(static_cast<int64_t>(frame.image.timestamp_ns), "stitched");
    } else {
      FrameBuffer left;
      FrameBuffer right;
      if (gs130_get_nv12_frame(device_, &left.image, &right.image) != GS130_OK) {
        break;
      }
      timestamps_->camera(static_cast<int64_t>(left.image.timestamp_ns), kLeft);
      timestamps_->camera(static_cast<int64_t>(right.image.timestamp_ns), kRight);
    }
  }
  judge_timestamps();
}

bool Gs130Node::any_image_subscriber()
{
  if (stitched_publisher_ && stitched_publisher_->get_subscription_count() > 0) {
    return true;
  }
  for (const auto & entry : raw_publishers_) {
    if (entry.second->get_subscription_count() > 0) {
      return true;
    }
  }
  for (const auto & entry : stitched_info_) {
    if (entry.second->get_subscription_count() > 0) {
      return true;
    }
  }
  for (const auto & entry : raw_infos_) {
    if (entry.second->get_subscription_count() > 0) {
      return true;
    }
  }
  return false;
}

void Gs130Node::judge_timestamps()
{
  if (!timestamps_->ready()) {
    return;
  }
  const auto wall = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  const std::vector<std::string> problems = timestamps_->judge(wall);
  if (problems.empty()) {
    RCLCPP_INFO(
      get_logger(),
      "using the device timestamps: they increase, are on the Unix epoch and agree "
      "with the IMU");
    return;
  }
  std::string joined;
  for (const std::string & problem : problems) {
    joined += joined.empty() ? problem : "; " + problem;
  }
  RCLCPP_ERROR(
    get_logger(),
    "the device clock looks unusable (%s); stamping every message with the receive "
    "time instead, which is late but on the right clock.  Set "
    "timestamp_source:=device to keep the device clock anyway.", joined.c_str());
}

sensor_msgs::msg::CameraInfo Gs130Node::camera_info_for(
  const std::string & eye, uint32_t width, uint32_t height,
  const builtin_interfaces::msg::Time & stamp)
{
  // width and height are the eye's own, taken from the frame that was actually
  // returned rather than from the parameters: rect passes its output through a
  // VSE crop and rescale, and a declared size that disagrees with the pixels
  // mis-scales every intrinsic downstream.
  const gs130_camera_intrinsics_t & intrinsics =
    intrinsics_[eye == kLeft ? 0 : 1];
  Projection projection{};
  const Projection * projection_ptr = nullptr;
  if (rectified_ && have_stereo_translation_) {
    // The left eye *is* the frame the right eye is measured in, so its own
    // translation is zero and its P[3] is zero.
    const double zero[3] = {0.0, 0.0, 0.0};
    projection = stereo_projection(
      intrinsics, eye == kLeft ? zero : stereo_translation_);
    projection_ptr = &projection;
  }
  return camera_info(
    intrinsics, width, height, frame_id_, stamp, projection_ptr, rectified_);
}

void Gs130Node::publish_stitched(
  const gs130_image_nv12_t & frame, const rclcpp::Time & stamp)
{
  const uint32_t width = frame.width;
  const uint32_t height = frame.height;
  uint32_t eye_width = 0;
  uint32_t eye_height = 0;
  eye_size(layout_, width, height, &eye_width, &eye_height);
  const builtin_interfaces::msg::Time time = stamp;

  // camera_info first, then the image, with one stamp for both -- the order the
  // official node uses, so a consumer keyed on the stamp has the calibration in
  // hand before the frame arrives.
  for (const auto & entry : stitched_info_) {
    entry.second->publish(camera_info_for(entry.first, eye_width, eye_height, time));
  }
  stitched_publisher_->publish(
    nv12_message(frame.data, width, height, frame_id_, time));

  if (raw_publishers_.empty()) {
    return;
  }
  const auto eyes = slice_eyes(frame.data, width, height, layout_);
  for (int i = 0; i < 2; ++i) {
    const std::string eye = i == 0 ? kLeft : kRight;
    const auto info = raw_infos_.find(eye);
    if (info != raw_infos_.end()) {
      info->second->publish(camera_info_for(eye, eye_width, eye_height, time));
    }
    const auto publisher = raw_publishers_.find(eye);
    if (publisher != raw_publishers_.end()) {
      publisher->second->publish(
        nv12_message(eyes[i].data(), eye_width, eye_height, frame_id_, time));
    }
  }
}

void Gs130Node::publish_eye(
  const std::string & eye, const gs130_image_nv12_t & frame, const rclcpp::Time & stamp)
{
  const uint32_t width = frame.width;
  const uint32_t height = frame.height;
  const builtin_interfaces::msg::Time time = stamp;
  const auto info = raw_infos_.find(eye);
  if (info != raw_infos_.end()) {
    info->second->publish(camera_info_for(eye, width, height, time));
  }
  raw_publishers_[eye]->publish(nv12_message(frame.data, width, height, frame_id_, time));
}

}  // namespace gs130_ros
