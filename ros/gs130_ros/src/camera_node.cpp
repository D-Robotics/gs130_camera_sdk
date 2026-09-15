/*
 * The gs130 ROS 2 camera node, built directly on the gs130 C API.
 *
 * The node owns the device for its whole life, polls the SDK's non-blocking
 * reads from timers, and publishes standard messages so that the existing TROS
 * nodes (hobot_codec, websocket) work unchanged.
 */

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

#include "gs130.h"

#include "gs130_ros/conversions.hpp"
#include "gs130_ros/preset.h"

namespace gs130_ros
{
namespace
{

/// Measured on hardware: the sensor is 1088x1280 and RAW output must match it.
constexpr int kSensorWidth = 1088;
constexpr int kSensorHeight = 1280;
/// Frames drained per timer tick, and the measured camera ceiling.
constexpr int kMaxFramesPerTick = 2;
constexpr int kMaxPacketsPerTick = 64;
constexpr int kMaxFps = 33;
/// The rates the IMU advertises, and the only ones gs130_init() accepts.
const std::array<int, 2> kOdrRates = {200, 500};
constexpr double kReportPeriodSeconds = 5.0;

const std::array<const char *, 3> kModes = {"raw", "resize", "rect"};
const std::array<const char *, 5> kLayouts = {
    "none", "left_right", "right_left", "top_bottom", "bottom_top"};
/// Resolutions the SDK rejected with GS130_UNSUPPORTED when measured, plus the
/// ones that do work. The list is deliberately permissive: only the measured
/// failures are refused, and the SDK stays the final authority.
const std::array<std::pair<int, int>, 2> kUnsupportedSizes = {{{864, 480}, {1024, 600}}};

gs130_camera_mode_t mode_value(const std::string & name)
{
    return name == "raw" ? GS130_CAMERA_MODE_RAW :
           name == "rect" ? GS130_CAMERA_MODE_RECT :
           GS130_CAMERA_MODE_RESIZE;
}

gs130_stereo_layout_t layout_value(const std::string & name)
{
    if (name == "left_right") return GS130_STEREO_LAYOUT_LEFT_RIGHT;
    if (name == "right_left") return GS130_STEREO_LAYOUT_RIGHT_LEFT;
    if (name == "top_bottom") return GS130_STEREO_LAYOUT_TOP_BOTTOM;
    if (name == "bottom_top") return GS130_STEREO_LAYOUT_BOTTOM_TOP;
    return GS130_STEREO_LAYOUT_NONE;
}

std::string join(const std::array<const char *, 5> & values)
{
    std::string text;
    for (const char * value : values) {
        if (!text.empty()) text += ", ";
        text += value;
    }
    return text;
}

std::string join(const std::array<const char *, 3> & values)
{
    std::string text;
    for (const char * value : values) {
        if (!text.empty()) text += ", ";
        text += value;
    }
    return text;
}

}  // namespace

class CameraNode : public rclcpp::Node
{
public:
    CameraNode()
    : rclcpp::Node("gs130_camera")
    {
        // A constructor that throws does not run the destructor, so a failure
        // after the camera is open would leave it held. Release it here.
        try {
            read_parameters();
            validate();
            open_device();
            start_streaming();
        } catch (...) {
            shutdown();
            throw;
        }
    }

    ~CameraNode() override
    {
        shutdown();
    }

    /// Release the camera. Called from the destructor and from main; the flag
    /// makes it idempotent. Never rely on this alone, as it may not run.
    void shutdown()
    {
        if (released_) return;
        released_ = true;
        if (device_ == nullptr) return;
        gs130_stop(device_);
        const gs130_err_t code = gs130_deinit(device_);
        gs130_destroy(device_);
        device_ = nullptr;
        if (code != GS130_OK) {
            RCLCPP_ERROR(get_logger(), "gs130_deinit() -> %d", static_cast<int>(code));
        } else {
            RCLCPP_INFO(get_logger(), "camera released");
        }
    }

private:
    // ------------------------------------------------------------- parameters

    void read_parameters()
    {
        platform_ = declare_parameter<std::string>("platform", "RDKX5");
        device_name_ = declare_parameter<std::string>("device", "GS130WI");
        mode_name_ = declare_parameter<std::string>("mode", "resize");
        width_ = declare_parameter<int>("width", 640);
        height_ = declare_parameter<int>("height", 480);
        fps_ = declare_parameter<int>("fps", 30);
        odr_ = declare_parameter<int>("odr", 200);
        layout_name_ = declare_parameter<std::string>("stereo_layout", "left_right");
        frame_camera_ = declare_parameter<std::string>("frame_id_camera", "camera_left");
        frame_imu_ = declare_parameter<std::string>("frame_id_imu", "imu_link");
        publish_imu_ = declare_parameter<bool>("publish_imu", true);
        publish_tf_ = declare_parameter<bool>("publish_tf", true);
        start_timeout_s_ = declare_parameter<double>("start_timeout_s", 10.0);
    }

    void fatal(const std::string & message)
    {
        RCLCPP_FATAL(get_logger(), "%s", message.c_str());
        throw std::runtime_error(message);
    }

    /// Reject what the SDK or the hardware cannot do, before opening anything.
    void validate()
    {
        // The device stamps are CLOCK_MONOTONIC since boot and are mapped to
        // the node clock by one constant offset taken at startup. Under
        // simulated time that mapping is meaningless, so refuse rather than
        // publish timestamps that would silently break tf and message_filters.
        if (get_parameter("use_sim_time").as_bool()) {
            fatal("use_sim_time is not supported: the camera stamps are monotonic "
                  "since boot and cannot be mapped onto simulated time. "
                  "Run with use_sim_time:=false.");
        }
        if (std::find(kModes.begin(), kModes.end(), mode_name_) == kModes.end()) {
            fatal("mode must be one of " + join(kModes) + ", got '" + mode_name_ + "'");
        }
        if (std::find(kLayouts.begin(), kLayouts.end(), layout_name_) == kLayouts.end()) {
            fatal("stereo_layout must be one of " + join(kLayouts) +
                  ", got '" + layout_name_ + "'");
        }
        if (fps_ < 1 || fps_ > kMaxFps) {
            fatal("fps must be between 1 and " + std::to_string(kMaxFps) +
                  ", got " + std::to_string(fps_));
        }
        // Measured: the IMU advertises 200 and 500 Hz, and odr=100 makes
        // gs130_init() fail with GS130_UNSUPPORTED on this hardware.
        if (std::find(kOdrRates.begin(), kOdrRates.end(), odr_) == kOdrRates.end()) {
            fatal("odr must be 200 or 500, got " + std::to_string(odr_) +
                  " (the IMU reports these two rates in its info string)");
        }
        if (width_ < 1 || height_ < 1) fatal("width and height must be positive");
        if (width_ % 2 != 0 || height_ % 2 != 0) {
            fatal("width and height must be even for NV12, got " +
                  std::to_string(width_) + "x" + std::to_string(height_));
        }
        for (const auto & size : kUnsupportedSizes) {
            if (width_ == size.first && height_ == size.second) {
                fatal("the camera reports UNSUPPORTED for " + std::to_string(width_) + "x" +
                      std::to_string(height_) + "; measured working sizes include " +
                      "320x240, 640x480, 1280x720 and 1920x1080");
            }
        }
        if (mode_name_ == "raw") {
            if (width_ != kSensorWidth || height_ != kSensorHeight) {
                fatal("mode raw requires width=" + std::to_string(kSensorWidth) +
                      " height=" + std::to_string(kSensorHeight) + ", got " +
                      std::to_string(width_) + "x" + std::to_string(height_));
            }
            if (layout_value(layout_name_) != GS130_STEREO_LAYOUT_NONE) {
                fatal("mode raw cannot be combined with stereo_layout=" + layout_name_ +
                      ": raw output must be the 1088x1280 sensor size, which stitching "
                      "would double. Use mode:=resize or stereo_layout:=none.");
            }
        }
        stitched_ = layout_value(layout_name_) != GS130_STEREO_LAYOUT_NONE;
    }

    // ---------------------------------------------------------------- device

    void open_device()
    {
        gs130_config_t config{};
        if (gs130_ros_preset(platform_.c_str(), device_name_.c_str(),
                             static_cast<int>(mode_value(mode_name_)),
                             width_, height_, fps_, odr_, &config) != 0) {
            fatal("unsupported platform/device: " + platform_ + " " + device_name_);
        }
        config.camera_config.stereo_layout = layout_value(layout_name_);

        device_ = gs130_create();
        if (device_ == nullptr) fatal("gs130_create() returned null");

        const gs130_err_t code = gs130_init(device_, &config);
        if (code != GS130_OK) {
            gs130_destroy(device_);
            device_ = nullptr;
            if (code == GS130_NOT_FOUND) {
                fatal("gs130_init() -> GS130_NOT_FOUND: the camera or its EEPROM was not "
                      "detected. Check the cable, and that the device name matches the "
                      "hardware (device:=" + device_name_ + ").");
            }
            if (code == GS130_UNSUPPORTED) {
                fatal("gs130_init() -> GS130_UNSUPPORTED: the hardware rejected this "
                      "configuration - platform=" + platform_ + " device=" + device_name_ +
                      " mode=" + mode_name_ + " size=" + std::to_string(width_) + "x" +
                      std::to_string(height_) + " fps=" + std::to_string(fps_) +
                      " odr=" + std::to_string(odr_) + ". Measured working sizes are "
                      "320x240, 640x480, 1280x720 and 1920x1080, and the accepted odr "
                      "values are 200 and 500.");
            }
            fatal("gs130_init() -> error " + std::to_string(static_cast<int>(code)) +
                  ". The camera pipeline is not shareable: check that no other process "
                  "holds it (mipi_cam, another camera_node, or a leftover script) with "
                  "'ps -ef | grep -E \"mipi_cam|camera_node\"'.");
        }
        has_imu_ = gs130_get_imu_name(device_) != nullptr;
    }

    void start_streaming()
    {
        const gs130_err_t code = gs130_start(device_);
        if (code != GS130_OK) {
            fatal("gs130_start() -> error " + std::to_string(static_cast<int>(code)));
        }

        const auto image_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
        const auto info_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

        if (stitched_) {
            image_publisher_ = create_publisher<sensor_msgs::msg::Image>("/image_combine_raw", image_qos);
        } else {
            left_publisher_ = create_publisher<sensor_msgs::msg::Image>("/image_left_raw", image_qos);
            right_publisher_ = create_publisher<sensor_msgs::msg::Image>("/image_right_raw", image_qos);
        }
        if (publish_imu_) {
            imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
                "/imu/data", rclcpp::QoS(rclcpp::KeepLast(200)).reliable());
        }
        left_info_publisher_ = create_publisher<sensor_msgs::msg::CameraInfo>(
            "/image_left/camera_info", info_qos);
        right_info_publisher_ = create_publisher<sensor_msgs::msg::CameraInfo>(
            "/image_right/camera_info", info_qos);

        publish_calibration();
        publish_transforms();
        take_clock_offset();
        report();

        const auto image_period = std::chrono::duration<double>(1.0 / std::max(2 * fps_, 10));
        const auto imu_period = std::chrono::duration<double>(1.0 / std::max(2 * odr_, 20));
        image_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(image_period),
            [this]() { poll_images(); });
        if (publish_imu_) {
            imu_timer_ = create_wall_timer(
                std::chrono::duration_cast<std::chrono::nanoseconds>(imu_period),
                [this]() { poll_imu(); });
        }
        report_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(kReportPeriodSeconds)),
            [this]() { report(); });

        RCLCPP_INFO(get_logger(),
                    "gs130 camera ready: %s publish NV12, not RGB. Decode with "
                    "cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_NV12).",
                    stitched_ ? "/image_combine_raw" : "/image_left_raw and /image_right_raw");
    }

    // ----------------------------------------------------------------- output

    void publish_calibration()
    {
        gs130_camera_intrinsics_t left{};
        gs130_camera_intrinsics_t right{};
        if (gs130_get_camera_intrinsics(device_, GS130_CAMERA_LEFT_IDX, &left) != GS130_OK ||
            gs130_get_camera_intrinsics(device_, GS130_CAMERA_RIGHT_IDX, &right) != GS130_OK) {
            RCLCPP_WARN(get_logger(), "camera intrinsics unavailable");
            return;
        }
        auto left_message = to_camera_info(left, width_, height_, frame_camera_);
        auto right_message = to_camera_info(right, width_, height_, "camera_right");
        const auto stamp = now();
        left_message.header.stamp = stamp;
        right_message.header.stamp = stamp;
        left_info_publisher_->publish(left_message);
        right_info_publisher_->publish(right_message);
        RCLCPP_INFO(get_logger(),
                    "calibration: left fx=%.2f fy=%.2f cx=%.2f cy=%.2f %s | right fx=%.2f %s",
                    left.fx, left.fy, left.cx, left.cy, left_message.distortion_model.c_str(),
                    right.fx, right_message.distortion_model.c_str());
    }

    void publish_transforms()
    {
        if (!publish_tf_) return;
        broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
        std::vector<geometry_msgs::msg::TransformStamped> messages;

        double rotation[9] = {0};
        double translation[3] = {0};
        if (gs130_get_relative_R(device_, GS130_REF_CAMERA_RIGHT, GS130_REF_CAMERA_LEFT, rotation) == GS130_OK &&
            gs130_get_relative_T(device_, GS130_REF_CAMERA_RIGHT, GS130_REF_CAMERA_LEFT, translation) == GS130_OK) {
            messages.push_back(to_transform(rotation, translation, frame_camera_, "camera_right"));
        }
        if (has_imu_ &&
            gs130_get_relative_R(device_, GS130_REF_IMU, GS130_REF_CAMERA_LEFT, rotation) == GS130_OK &&
            gs130_get_relative_T(device_, GS130_REF_IMU, GS130_REF_CAMERA_LEFT, translation) == GS130_OK) {
            messages.push_back(to_transform(rotation, translation, frame_camera_, frame_imu_));
        }
        if (messages.empty()) return;
        const auto stamp = now();
        for (auto & message : messages) message.header.stamp = stamp;
        const double baseline = std::sqrt(
            messages[0].transform.translation.x * messages[0].transform.translation.x +
            messages[0].transform.translation.y * messages[0].transform.translation.y +
            messages[0].transform.translation.z * messages[0].transform.translation.z);
        broadcaster_->sendTransform(messages);
        RCLCPP_INFO(get_logger(), "static tf from %s to %zu frame(s), baseline %.6f m",
                    frame_camera_.c_str(), messages.size(), baseline);
    }

    // ------------------------------------------------------------------ reads

    void poll_images()
    {
        for (int i = 0; i < kMaxFramesPerTick; ++i) {
            // The SDK leaves the structs untouched on timeout, so they are
            // cleared first and only freed when the call actually filled them.
            gs130_image_nv12_t left{};
            gs130_image_nv12_t right{};
            gs130_image_nv12_t stitched{};
            gs130_err_t code;
            if (stitched_) {
                code = gs130_get_stereo_nv12_frame(device_, &stitched);
            } else {
                code = gs130_get_nv12_frame(device_, &left, &right);
            }
            if (code == GS130_TIMEOUT) {
                read_fault_ = 0;
                return;
            }
            if (code != GS130_OK) {
                // A fault must not flood the log: one line per fault episode.
                if (read_fault_ != static_cast<int>(code)) {
                    read_fault_ = static_cast<int>(code);
                    RCLCPP_ERROR(get_logger(), "frame read failed with %d", read_fault_);
                }
                return;
            }
            if (stitched_) {
                publish_frame(stitched, "camera", image_publisher_);
                free(stitched.data);
            } else {
                publish_frame(left, frame_camera_, left_publisher_);
                publish_frame(right, "camera_right", right_publisher_);
                free(left.data);
                free(right.data);
            }
            ++frames_;
            if (frames_ == 1) RCLCPP_INFO(get_logger(), "first frame published");
        }
    }

    void publish_frame(const gs130_image_nv12_t & frame,
                       const std::string & frame_id,
                       const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr & publisher)
    {
        if (frame.data == nullptr || frame.width == 0 || frame.height == 0) return;
        const auto stamp = to_stamp(static_cast<int64_t>(frame.timestamp_ns) + offset_ns_);
        // to_image copies the payload, so the SDK buffer is free to be
        // released by the caller the moment this returns.
        publisher->publish(to_image(frame.data, frame.width, frame.height, frame_id, stamp));
    }

    void poll_imu()
    {
        for (int i = 0; i < kMaxPacketsPerTick; ++i) {
            gs130_imu_packet_t packet{};
            const gs130_err_t code = gs130_get_imu_packet(device_, &packet);
            if (code == GS130_TIMEOUT) {
                imu_fault_ = 0;
                return;
            }
            if (code != GS130_OK) {
                if (imu_fault_ != static_cast<int>(code)) {
                    imu_fault_ = static_cast<int>(code);
                    RCLCPP_ERROR(get_logger(), "imu read failed with %d", imu_fault_);
                }
                return;
            }
            const auto stamp = to_stamp(static_cast<int64_t>(packet.timestamp_ns) + offset_ns_);
            imu_publisher_->publish(to_imu(packet, frame_imu_, stamp));
            ++packets_;
        }
    }

    // -------------------------------------------------------------- timestamps

    void take_clock_offset()
    {
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::duration<double>(start_timeout_s_);
        auto last_report = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() < deadline) {
            gs130_image_nv12_t left{};
            gs130_image_nv12_t right{};
            gs130_image_nv12_t stitched{};
            gs130_err_t code;
            if (stitched_) {
                code = gs130_get_stereo_nv12_frame(device_, &stitched);
            } else {
                code = gs130_get_nv12_frame(device_, &left, &right);
            }
            if (code == GS130_OK) {
                const uint64_t stamp = stitched_ ? stitched.timestamp_ns : left.timestamp_ns;
                offset_ns_ = static_cast<int64_t>(now().nanoseconds()) -
                             static_cast<int64_t>(stamp);
                RCLCPP_INFO(get_logger(),
                            "gs130 timestamps are monotonic since boot; using a constant "
                            "offset of %ld ns (accuracy within one frame period)",
                            static_cast<long>(offset_ns_));
                if (stitched_) {
                    free(stitched.data);
                } else {
                    free(left.data);
                    free(right.data);
                }
                return;
            }
            if (std::chrono::steady_clock::now() - last_report > std::chrono::seconds(2)) {
                last_report = std::chrono::steady_clock::now();
                RCLCPP_INFO(get_logger(), "waiting for the first frame");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        fatal("no frame within the start timeout; check the camera and the IMU FSYNC wiring");
    }

    // ----------------------------------------------------------------- reports

    void report()
    {
        RCLCPP_INFO(get_logger(), "frames=%lu imu=%lu | %s | %dx%d %s fps=%d odr=%d",
                    static_cast<unsigned long>(frames_), static_cast<unsigned long>(packets_),
                    layout_name_.c_str(), width_, height_, mode_name_.c_str(), fps_, odr_);
        // Measured behaviour: a second opener is not refused, it simply takes
        // the frames, and this node then stops producing images with no error.
        if (frames_ == last_frames_) {
            if (++stalled_ == 2) {
                RCLCPP_ERROR(get_logger(),
                             "no camera frames for about %.0f s. Another process has most "
                             "likely taken the camera (mipi_cam or a second camera_node), "
                             "which also makes the IMU reads fail; check with "
                             "'ps -ef | grep -E \"mipi_cam|camera_node\"'.",
                             2 * kReportPeriodSeconds);
            }
        } else {
            stalled_ = 0;
        }
        last_frames_ = frames_;
    }

    // ------------------------------------------------------------------ state

    std::string platform_, device_name_, mode_name_, layout_name_;
    std::string frame_camera_, frame_imu_;
    int width_ = 0, height_ = 0, fps_ = 0, odr_ = 0;
    bool publish_imu_ = true, publish_tf_ = true, stitched_ = false, has_imu_ = false;
    bool released_ = false;
    double start_timeout_s_ = 10.0;

    gs130_device_t * device_ = nullptr;
    int64_t offset_ns_ = 0;
    uint64_t frames_ = 0, packets_ = 0, last_frames_ = 0;
    int stalled_ = 0;
    int read_fault_ = 0;
    int imu_fault_ = 0;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_publisher_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
    rclcpp::TimerBase::SharedPtr image_timer_, imu_timer_, report_timer_;
};

}  // namespace gs130_ros

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    int code = 0;
    std::shared_ptr<gs130_ros::CameraNode> node;
    try {
        node = std::make_shared<gs130_ros::CameraNode>();
        rclcpp::spin(node);
    } catch (const std::exception & error) {
        // Parameter and device failures have already been logged with detail.
        code = 2;
    }
    if (node) {
        node->shutdown();
    }
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return code;
}
