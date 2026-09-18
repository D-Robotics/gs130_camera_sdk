/**
 * @file SZYGSJKJ_stereo_imu_fisheye_v1p2r0n4.cpp
 * @brief SZYGSJKJ stereo + IMU, fisheye, v1.2, rotation 0 deg, 4 distortion coefficients.
 *
 * This file is self-contained: header, checksum, offsets, and field widths are all local; no code is shared with other models.
 *
 * The EEPROM is read once as a flat byte image, so the offsets below are byte offsets.
 * Floating-point fields are copied directly into host float/double objects; this format
 * therefore assumes the EEPROM encoding matches the target's IEEE-754 byte order.
 *
 * Unlike the UNION layout, this layout stores every field as int/float (4B) and carries no
 * 8B double, so there is no read_double() here; it also stores scale and misalignment
 * combined in one 3x3 matrix per sensor.  Its IMU area has its own checksum byte, whose
 * documented range overshoots the last data byte: summing up to that byte is what the
 * module actually stores, so kImuSumLen covers the data only.
 *
 * This file is part of gs130_camera_sdk (https://github.com/D-Robotics/gs130_camera_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "devices/eeprom/eeprom.hpp"

#include <cstring>

namespace gs130 {
namespace eeprom {
namespace {

// ============================== Identification ==============================
// The identification area is header[0..14], followed by one checksum byte at offset 15.
constexpr uint16_t kHeaderSize = 15;                 // header[0..14]
constexpr uint16_t kChecksumOff = kHeaderSize;       // checksum follows immediately
constexpr uint8_t  kChecksumLen = 14;                // only the first 14 bytes are summed

constexpr uint8_t kHeader[kHeaderSize] = {
    0x53, 0x5a, 0x59, 0x47, 0x53, 0x4a, 0x4b, 0x4a,  // "SZYGSJKJ"
    0x11, 0x01,   // stereo + IMU
    0x01, 0x02,   // v1.2
    0x00,         // rotation 0 deg
    0x04,         // 4 distortion coefficients
    0x00,
};

// The IMU area is announced by the module-type byte of the header.
constexpr uint8_t  kCamTypeStereoImu = 0x11;
constexpr uint16_t kCamTypeOff       = 0x0008;   // module type (char)

// ============================== Layout offsets ==============================
// Camera intrinsics and stereo extrinsics are int/float (4B); IMU intrinsics and IMU
// extrinsics are float (4B)
// These are byte offsets into the flat image, not indices of aligned fields: the image is
// addressed byte-wise, so an offset is not required to be a multiple of its field width.
// Unlike the UNION layout, the two intrinsics blocks begin with the calibration resolution
// (int width, int height), so they are named after the block rather than after fx.

constexpr uint16_t kLBlock        = 0x0010;   // start of the EEPROM L intrinsics block
constexpr uint16_t kRBlock        = 0x0048;   // start of the EEPROM R intrinsics block
constexpr uint16_t kStereoR       = 0x008C;   // stereo rotation 3x3 (float)
constexpr uint16_t kStereoT       = 0x00B0;   // stereo translation 3  (float)
constexpr uint16_t kAccelMisalign = 0x0200;   // accel scale + misalignment 3x3 (float), row-major
constexpr uint16_t kAccelNoise    = 0x0224;   // accel noise density (float)
constexpr uint16_t kAccelWalk     = 0x0228;   // accel random walk (float)
constexpr uint16_t kGyroMisalign  = 0x022C;   // gyro scale + misalignment 3x3 (float), row-major
constexpr uint16_t kGyroNoise     = 0x0250;   // gyro noise density (float)
constexpr uint16_t kGyroWalk      = 0x0254;   // gyro random walk (float)
constexpr uint16_t kImuR          = 0x025C;   // IMU -> left camera rotation (float)
constexpr uint16_t kImuT          = 0x0280;   // IMU -> left camera translation (float)
constexpr uint16_t kSize          = 0x02D8;   // bytes to read for a full calibration

// The IMU area ends with its own checksum byte, immediately after the last data byte.
constexpr uint16_t kImuCheckOff = 0x02D8;     // IMU checksum (char)
constexpr uint16_t kImuSumLen   = kImuCheckOff - kAccelMisalign;   // bytes summed: the data only

// memcpy avoids unaligned typed access; the EEPROM representation must match host byte order.
// EEPROM stores 4-byte floats; widening to double is lossless
double read_float(const uint8_t *buf, uint16_t off)
{
    float v;
    memcpy(&v, &buf[off], sizeof(v));
    return v;
}

// Intrinsics block: fx,fy,cx,cy followed by 4 fisheye distortion coefficients, all float
void read_intrinsics(const uint8_t *buf, CameraIntrinsics *cam, uint16_t off)
{
    cam->fx = read_float(buf, off + 0x08);
    cam->fy = read_float(buf, off + 0x0C);
    cam->cx = read_float(buf, off + 0x10);
    cam->cy = read_float(buf, off + 0x14);

    cam->K[0] = cam->fx;
    cam->K[2] = cam->cx;
    cam->K[4] = cam->fy;
    cam->K[5] = cam->cy;
    cam->K[8] = 1.0f;

    cam->dist_model = DistModel::Fisheye;
    cam->dist_coeffs[0] = read_float(buf, off + 0x18);
    cam->dist_coeffs[1] = read_float(buf, off + 0x1C);
    cam->dist_coeffs[2] = read_float(buf, off + 0x20);
    cam->dist_coeffs[3] = read_float(buf, off + 0x24);
}

// The EEPROM stores the relative pose IMU->left camera; compose it into IMU->reference frame before handing to the caller
void compose_imu_to_ref(StereoImuModel *c, const double *imu_to_left_R,
                        const double *imu_to_left_T)
{
    for(uint8_t r = 0; r < 3; r++){
        for(uint8_t col = 0; col < 3; col++){
            c->imu_R[r * 3 + col] = 0.0;
            for(uint8_t k = 0; k < 3; k++)
                c->imu_R[r * 3 + col] += c->cam_left_R[r * 3 + k] * imu_to_left_R[k * 3 + col];
        }
        c->imu_T[r] = c->cam_left_T[r];
        for(uint8_t k = 0; k < 3; k++)
            c->imu_T[r] += c->cam_left_R[r * 3 + k] * imu_to_left_T[k];
    }
}

bool probe(base::I2cDevice &bus)
{
    // header 15B + checksum 1B
    uint8_t buf[kHeaderSize + 1];
    if(bus.readBurst16(0x0000, buf, sizeof(buf)) != Status::Ok)
        return false;

    // Checksum: sum of the first 14 bytes, (sum % 255) + 1
    uint16_t sum = 0;
    for(uint8_t i = 0; i < kChecksumLen; i++)
        sum = static_cast<uint16_t>(sum + buf[i]);
    if(static_cast<uint8_t>((sum % 255) + 1) != buf[kChecksumOff])
        return false;

    return memcmp(buf, kHeader, kHeaderSize) == 0;
}

Status read(base::I2cDevice &bus, StereoImuModel *out)
{
    if(!out)
        return Status::ParamError;

    uint8_t buf[kSize];
    if(bus.readBurst16(0x0000, buf, sizeof(buf)) != Status::Ok)
        return Status::HwError;

    if(buf[kCamTypeOff] != kCamTypeStereoImu)
        return Status::HwError;

    // This layout carries its own checksum over the IMU area; a mismatch invalidates the read.
    uint32_t imu_sum = 0;
    for(uint16_t i = 0; i < kImuSumLen; i++)
        imu_sum += buf[kAccelMisalign + i];
    uint8_t imu_checksum = 0;
    if(bus.readBurst16(kImuCheckOff, &imu_checksum, 1) != Status::Ok)
        return Status::HwError;
    if(static_cast<uint8_t>((imu_sum % 255) + 1) != imu_checksum)
        return Status::HwError;

    *out = StereoImuModel{};

    // The EEPROM L/R blocks map directly to the module's left/right cameras
    read_intrinsics(buf, &out->cam_left,  kLBlock);
    read_intrinsics(buf, &out->cam_right, kRBlock);

    out->install_angle = static_cast<int8_t>(buf[0x0C]);

    // This layout uses the right camera as the common reference frame, so cam_right is the identity transform
    for(uint8_t i = 0; i < 9; i++){
        out->cam_right_R[i] = (i % 4 == 0) ? 1.0f : 0.0f;
        out->cam_left_R[i]  = read_float(buf, kStereoR + i * 4);
    }
    for(uint8_t i = 0; i < 3; i++){
        out->cam_right_T[i] = 0.0f;
        out->cam_left_T[i]  = read_float(buf, kStereoT + i * 4);
    }

    // Each sensor carries scale and misalignment combined in one matrix, with no separate
    // scale or bias field; the matrix is kept whole and no missing term is invented.
    for(uint8_t i = 0; i < 9; i++){
        out->imu.accel_misalign[i] = read_float(buf, kAccelMisalign + i * 4);
        out->imu.gyro_misalign[i]  = read_float(buf, kGyroMisalign + i * 4);
    }
    out->imu.accel_noise       = read_float(buf, kAccelNoise);
    out->imu.accel_random_walk = read_float(buf, kAccelWalk);
    out->imu.gyro_noise        = read_float(buf, kGyroNoise);
    out->imu.gyro_random_walk  = read_float(buf, kGyroWalk);

    double imu_to_left_R[9];
    double imu_to_left_T[3];
    for(uint8_t i = 0; i < 9; i++)
        imu_to_left_R[i] = read_float(buf, kImuR + i * 4);
    for(uint8_t i = 0; i < 3; i++)
        imu_to_left_T[i] = read_float(buf, kImuT + i * 4);
    compose_imu_to_ref(out, imu_to_left_R, imu_to_left_T);

    return Status::Ok;
}

} // namespace

extern const ModelDesc kSZYGSJKJStereoImuFisheyeV1P2R0N4 = {
    "SZYGSJKJ Stereo-IMU Fisheye V1.2 Rotate-0-deg 4-Distortion-parameters",
    "- Calibration manufacturer:    SZYGSJKJ\n"
    "- Distortion model:            Fisheyes-4-parameters,\n"
    "- Calibration version:         V1.2\n"
    "- Device Type:                 Stereo Camera and IMU\n"
    "- Rotating installation angle: 0 degrees\n",
    probe,
    read,
};

GS130_EEPROM_REGISTER_MODEL(kSZYGSJKJStereoImuFisheyeV1P2R0N4);

} // namespace eeprom
} // namespace gs130
