/*
 * Tests for the mapping functions that need neither a camera nor a node.
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>

#include "gs130_ros/conversions.hpp"

namespace
{

gs130_camera_intrinsics_t make_intrinsics(const double coefficients[8],
                                            gs130_dist_model_t model)
{
    gs130_camera_intrinsics_t intrinsics{};
    intrinsics.fx = 386.85;
    intrinsics.fy = 387.12;
    intrinsics.cx = 304.62;
    intrinsics.cy = 245.06;
    const double k[9] = {386.85, 0.0, 304.62, 0.0, 387.12, 245.06, 0.0, 0.0, 1.0};
    std::memcpy(intrinsics.K, k, sizeof(k));
    std::memcpy(intrinsics.dist_coeffs, coefficients, sizeof(double) * 8);
    intrinsics.dist_model = model;
    return intrinsics;
}

TEST(DistortionModel, RectifiedCameraReportsPlumbBob)
{
    const double zeros[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    std::vector<double> coefficients;
    const auto intrinsics = make_intrinsics(zeros, GS130_DIST_FISHEYE);
    EXPECT_EQ(gs130_ros::distortion_model(intrinsics, coefficients), "plumb_bob");
    ASSERT_EQ(coefficients.size(), 5u);
    EXPECT_DOUBLE_EQ(coefficients[0], 0.0);
}

TEST(DistortionModel, FisheyeKeepsFourCoefficients)
{
    const double values[8] = {-0.025173, 0.012189, -0.013019, 0.002486, 0, 0, 0, 0};
    std::vector<double> coefficients;
    const auto intrinsics = make_intrinsics(values, GS130_DIST_FISHEYE);
    EXPECT_EQ(gs130_ros::distortion_model(intrinsics, coefficients), "equidistant");
    ASSERT_EQ(coefficients.size(), 4u);
    EXPECT_NEAR(coefficients[0], -0.025173, 1e-9);
}

TEST(DistortionModel, PinholeKeepsEightCoefficients)
{
    const double values[8] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8};
    std::vector<double> coefficients;
    const auto intrinsics = make_intrinsics(values, GS130_DIST_PINHOLE);
    EXPECT_EQ(gs130_ros::distortion_model(intrinsics, coefficients), "rational_polynomial");
    ASSERT_EQ(coefficients.size(), 8u);
}

TEST(CameraInfo, FieldsComeFromTheCalibration)
{
    const double values[8] = {-0.025173, 0.012189, -0.013019, 0.002486, 0, 0, 0, 0};
    const auto intrinsics = make_intrinsics(values, GS130_DIST_FISHEYE);
    const auto message = gs130_ros::to_camera_info(intrinsics, 640, 480, "camera_left");
    EXPECT_EQ(message.width, 640u);
    EXPECT_EQ(message.height, 480u);
    EXPECT_EQ(message.header.frame_id, "camera_left");
    EXPECT_EQ(message.distortion_model, "equidistant");
    EXPECT_NEAR(message.k[0], 386.85, 1e-9);
    EXPECT_NEAR(message.k[4], 387.12, 1e-9);
    EXPECT_NEAR(message.k[2], 304.62, 1e-9);
    EXPECT_NEAR(message.k[5], 245.06, 1e-9);
    EXPECT_DOUBLE_EQ(message.r[0], 1.0);
    EXPECT_DOUBLE_EQ(message.r[4], 1.0);
    EXPECT_DOUBLE_EQ(message.r[8], 1.0);
    EXPECT_DOUBLE_EQ(message.r[1], 0.0);
    EXPECT_NEAR(message.p[0], 386.85, 1e-9);
    EXPECT_NEAR(message.p[2], 304.62, 1e-9);
}

TEST(Image, HeightIsTheRealHeightNotThePackedRowCount)
{
    // The mistakes this guards against: sending the packed NV12 row count as
    // the height makes the TROS codec crash, and the length, step and encoding
    // assertions all still hold in that broken case, so they cannot catch it.
    const uint32_t width = 1280;
    const uint32_t height = 480;
    std::vector<uint8_t> buffer(gs130_ros::nv12_size(width, height), 0x5a);
    const auto message = gs130_ros::to_image(buffer.data(), width, height, "camera",
                                             gs130_ros::to_stamp(1));
    EXPECT_EQ(message.height, height);
    EXPECT_NE(message.height, height * 3 / 2);
    EXPECT_EQ(message.width, width);
    EXPECT_EQ(message.step, width);
    EXPECT_EQ(message.encoding, "nv12");
    EXPECT_EQ(message.is_bigendian, 0);
    EXPECT_EQ(message.data.size(), 1280u * 480u * 3 / 2);
    EXPECT_EQ(message.header.frame_id, "camera");
    EXPECT_EQ(message.data[0], 0x5a);
    EXPECT_EQ(message.data.back(), 0x5a);
}

TEST(Image, Nv12SizeMatchesThePackedLayout)
{
    EXPECT_EQ(gs130_ros::nv12_size(640, 480), 460800u);
    EXPECT_EQ(gs130_ros::nv12_size(1280, 480), 921600u);
    EXPECT_EQ(gs130_ros::nv12_size(1920, 1080), 3110400u);
}

TEST(Imu, UnitsAndCovariancesFollowTheSdk)
{
    gs130_imu_packet_t packet{};
    packet.accel[0] = -0.158f; packet.accel[1] = -9.835f; packet.accel[2] = 0.062f;
    packet.gyro[0] = 0.001f;  packet.gyro[1] = -0.016f; packet.gyro[2] = 0.002f;
    packet.temp = 25.9f;
    packet.is_fsync = false;
    packet.timestamp_ns = 9546850424616ULL;
    // to_stamp takes nanoseconds, so 7 seconds is 7e9.
    const auto message = gs130_ros::to_imu(packet, "imu_link", gs130_ros::to_stamp(7000000000LL));
    EXPECT_EQ(message.header.frame_id, "imu_link");
    EXPECT_EQ(message.header.stamp.sec, 7);
    EXPECT_FLOAT_EQ(message.linear_acceleration.y, -9.835f);
    EXPECT_FLOAT_EQ(message.angular_velocity.y, -0.016f);
    // No fused orientation is available from the SDK.
    EXPECT_DOUBLE_EQ(message.orientation.w, 1.0);
    EXPECT_DOUBLE_EQ(message.orientation_covariance[0], -1.0);
    // The gyro and accelerometer are measured but carry no variance.
    EXPECT_DOUBLE_EQ(message.angular_velocity_covariance[0], 0.0);
    EXPECT_DOUBLE_EQ(message.linear_acceleration_covariance[0], 0.0);
}

TEST(Stamp, SplitsNanoseconds)
{
    const auto stamp = gs130_ros::to_stamp(1789395368711803996LL);
    EXPECT_EQ(stamp.sec, 1789395368);
    EXPECT_EQ(stamp.nanosec, 711803996u);
}

TEST(Stamp, HandlesZero)
{
    const auto stamp = gs130_ros::to_stamp(0);
    EXPECT_EQ(stamp.sec, 0);
    EXPECT_EQ(stamp.nanosec, 0u);
}

TEST(Quaternion, IdentityBecomesUnit)
{
    const double identity[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    double x = 1, y = 1, z = 1, w = 0;
    gs130_ros::to_quaternion(identity, x, y, z, w);
    EXPECT_NEAR(w, 1.0, 1e-12);
    EXPECT_NEAR(x, 0.0, 1e-12);
    EXPECT_NEAR(y, 0.0, 1e-12);
    EXPECT_NEAR(z, 0.0, 1e-12);
}

TEST(Quaternion, HalfTurnUsesTheOffDiagonalBranch)
{
    const double half[9] = {1, 0, 0, 0, -1, 0, 0, 0, -1};
    double x = 0, y = 0, z = 0, w = 1;
    gs130_ros::to_quaternion(half, x, y, z, w);
    EXPECT_NEAR(w, 0.0, 1e-12);
    EXPECT_NEAR(std::fabs(x), 1.0, 1e-12);
}

TEST(Quaternion, ReproducesTheRotationMatrix)
{
    const double angle = 0.7;
    const double m[9] = {std::cos(angle), -std::sin(angle), 0.0,
                         std::sin(angle), std::cos(angle), 0.0,
                         0.0, 0.0, 1.0};
    double x, y, z, w;
    gs130_ros::to_quaternion(m, x, y, z, w);
    const double v[3] = {1.0, 0.0, 0.0};
    // v' = v + 2w(q x v) + 2q x (q x v)
    const double cx = y * v[2] - z * v[1];
    const double cy = z * v[0] - x * v[2];
    const double cz = x * v[1] - y * v[0];
    const double c2x = y * cz - z * cy;
    const double c2y = z * cx - x * cz;
    const double c2z = x * cy - y * cx;
    const double rotated[3] = {v[0] + 2 * w * cx + 2 * c2x,
                               v[1] + 2 * w * cy + 2 * c2y,
                               v[2] + 2 * w * cz + 2 * c2z};
    const double expected[3] = {m[0], m[3], m[6]};
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(rotated[i], expected[i], 1e-9);
    }
}

TEST(Transform, ParentAndChildFollowTheSdkMeaning)
{
    const double identity[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    const double translation[3] = {0.070316, 0.0, 0.0};
    const auto message = gs130_ros::to_transform(identity, translation,
                                                 "camera_left", "camera_right");
    EXPECT_EQ(message.header.frame_id, "camera_left");
    EXPECT_EQ(message.child_frame_id, "camera_right");
    EXPECT_NEAR(message.transform.translation.x, 0.070316, 1e-9);
    EXPECT_NEAR(message.transform.rotation.w, 1.0, 1e-12);
}

}  // namespace
