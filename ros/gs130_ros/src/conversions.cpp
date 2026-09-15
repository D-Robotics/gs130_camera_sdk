#include "gs130_ros/conversions.hpp"

#include <cmath>

namespace gs130_ros
{

builtin_interfaces::msg::Time to_stamp(int64_t nanoseconds)
{
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
    stamp.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
    return stamp;
}

size_t nv12_size(uint32_t width, uint32_t height)
{
    return static_cast<size_t>(width) * height * 3 / 2;
}

sensor_msgs::msg::Image to_image(const uint8_t * data,
                                 uint32_t width,
                                 uint32_t height,
                                 const std::string & frame_id,
                                 const builtin_interfaces::msg::Time & stamp)
{
    sensor_msgs::msg::Image message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id;
    message.encoding = "nv12";
    message.is_bigendian = 0;
    message.width = width;
    message.height = height;
    message.step = width;
    message.data.assign(data, data + nv12_size(width, height));
    return message;
}

sensor_msgs::msg::Imu to_imu(const gs130_imu_packet_t & packet,
                             const std::string & frame_id,
                             const builtin_interfaces::msg::Time & stamp)
{
    sensor_msgs::msg::Imu message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id;
    message.angular_velocity.x = packet.gyro[0];
    message.angular_velocity.y = packet.gyro[1];
    message.angular_velocity.z = packet.gyro[2];
    message.linear_acceleration.x = packet.accel[0];
    message.linear_acceleration.y = packet.accel[1];
    message.linear_acceleration.z = packet.accel[2];
    message.orientation.w = 1.0;
    message.orientation_covariance[0] = -1.0;
    return message;
}

std::string distortion_model(const gs130_camera_intrinsics_t & intrinsics,
                             std::vector<double> & coefficients)
{
    bool all_zero = true;
    for (int i = 0; i < 8; ++i) {
        if (intrinsics.dist_coeffs[i] != 0.0) {
            all_zero = false;
            break;
        }
    }
    if (all_zero) {
        coefficients.assign(5, 0.0);
        return "plumb_bob";
    }
    if (intrinsics.dist_model == GS130_DIST_FISHEYE) {
        coefficients.assign(intrinsics.dist_coeffs, intrinsics.dist_coeffs + 4);
        return "equidistant";
    }
    coefficients.assign(intrinsics.dist_coeffs, intrinsics.dist_coeffs + 8);
    return "rational_polynomial";
}

sensor_msgs::msg::CameraInfo to_camera_info(const gs130_camera_intrinsics_t & intrinsics,
                                           uint32_t width,
                                           uint32_t height,
                                           const std::string & frame_id)
{
    sensor_msgs::msg::CameraInfo message;
    message.header.frame_id = frame_id;
    message.width = width;
    message.height = height;
    message.distortion_model = distortion_model(intrinsics, message.d);
    for (int i = 0; i < 9; ++i) {
        message.k[i] = intrinsics.K[i];
    }
    message.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    message.p = {intrinsics.fx, 0.0, intrinsics.cx, 0.0,
                 0.0, intrinsics.fy, intrinsics.cy, 0.0,
                 0.0, 0.0, 1.0, 0.0};
    return message;
}

void to_quaternion(const double m[9], double & x, double & y, double & z, double & w)
{
    const double trace = m[0] + m[4] + m[8];
    if (trace > 0.0) {
        const double root = std::sqrt(trace + 1.0);
        w = 0.5 * root;
        const double scale = 0.5 / root;
        x = (m[7] - m[5]) * scale;
        y = (m[2] - m[6]) * scale;
        z = (m[3] - m[1]) * scale;
        return;
    }
    if (m[0] >= m[4] && m[0] >= m[8]) {
        const double root = std::sqrt(1.0 + m[0] - m[4] - m[8]);
        x = 0.5 * root;
        const double scale = 0.5 / root;
        y = (m[1] + m[3]) * scale;
        z = (m[2] + m[6]) * scale;
        w = (m[7] - m[5]) * scale;
        return;
    }
    if (m[4] >= m[8]) {
        const double root = std::sqrt(1.0 + m[4] - m[0] - m[8]);
        y = 0.5 * root;
        const double scale = 0.5 / root;
        x = (m[1] + m[3]) * scale;
        z = (m[5] + m[7]) * scale;
        w = (m[2] - m[6]) * scale;
        return;
    }
    const double root = std::sqrt(1.0 + m[8] - m[0] - m[4]);
    z = 0.5 * root;
    const double scale = 0.5 / root;
    x = (m[2] + m[6]) * scale;
    y = (m[5] + m[7]) * scale;
    w = (m[3] - m[1]) * scale;
}

geometry_msgs::msg::TransformStamped to_transform(const double rotation[9],
                                                 const double translation[3],
                                                 const std::string & parent_frame,
                                                 const std::string & child_frame)
{
    geometry_msgs::msg::TransformStamped message;
    message.header.frame_id = parent_frame;
    message.child_frame_id = child_frame;
    message.transform.translation.x = translation[0];
    message.transform.translation.y = translation[1];
    message.transform.translation.z = translation[2];
    to_quaternion(rotation,
                  message.transform.rotation.x, message.transform.rotation.y,
                  message.transform.rotation.z, message.transform.rotation.w);
    return message;
}

}  // namespace gs130_ros
