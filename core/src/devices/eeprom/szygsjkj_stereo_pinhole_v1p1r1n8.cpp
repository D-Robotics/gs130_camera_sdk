/**
 * @file szygsjkj_stereo_pinhole_v1p1r1n8.cpp
 * @brief SZYGSJKJ stereo (no IMU), pinhole, v1.1, rotation index 1 (=90 deg), 8 distortion coefficients.
 *
 * This file is self-contained: header, checksum, offsets, and field widths are all local; no code is shared with other models.
 *
 * The EEPROM is read once as a flat byte image, so the offsets below are byte offsets.
 * Floating-point fields are copied directly into host float/double objects; this format
 * therefore assumes the EEPROM encoding matches the target's IEEE-754 byte order.
 *
 * The layout of this variant is byte-for-byte the same as
 * szygsjkj_stereo_imu_fisheye_v1p2r0n4.cpp for everything up to the stereo extrinsics:
 * both intrinsics blocks are 56 bytes (2 ints, then fx/fy/cx/cy, then eight float
 * coefficient slots) and the right block starts at 0x0048.  The two variants differ in
 * three ways, and each is handled here rather than by reusing that file:
 *
 *   - the identification header (module type, version, rotation, coefficient count);
 *   - the distortion model: this variant fills all eight slots and is interpreted as
 *     OpenCV pinhole/radtan [k1,k2,p1,p2,k3,k4,k5,k6], while the fisheye variant uses
 *     [k1,k2,k3,k4] and leaves the remaining slots untouched;
 *   - there is no IMU: the IMU area of this module reads back as 0xFF, so this driver
 *     neither requires nor interprets it, and the IMU fields are left zeroed.  The
 *     device is registered with its IMU disabled, so those fields are never consumed.
 *
 * The rotation byte is a quarter-turn index, not a degree count: 0 -> 0, 1 -> 90,
 * 2 -> 180, 3 -> 270.  A literal reading of 1 as "one degree" would be rejected by the
 * pipeline, which requires a multiple of 90.  That interpretation is an inference from
 * this module being mounted landscape; it is verified by checking the delivered image
 * orientation against the scene.
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
    0x01, 0x00,   // stereo, no IMU
    0x01, 0x01,   // v1.1
    0x01,         // rotation index 1 (90 deg)
    0x08,         // 8 distortion coefficients
    0x00,
};

// Offset of the rotation byte inside the header.
constexpr uint16_t kRotationOff = 0x000C;
// The rotation byte counts quarter turns; the pipeline only accepts multiples of 90.
constexpr int kQuarterTurnDeg = 90;

// ============================== Layout offsets ==============================
// Camera intrinsics and stereo extrinsics are int/float (4B).
// These are byte offsets into the flat image, not indices of aligned fields: the image is
// addressed byte-wise, so an offset is not required to be a multiple of its field width.
// Each intrinsics block begins with the calibration resolution (int width, int height),
// so the blocks are named after the block rather than after fx.

constexpr uint16_t kLBlock   = 0x0010;   // start of the EEPROM L intrinsics block
constexpr uint16_t kRBlock   = 0x0048;   // start of the EEPROM R intrinsics block
constexpr uint16_t kStereoR  = 0x008C;   // stereo rotation 3x3 (float)
constexpr uint16_t kStereoT  = 0x00B0;   // stereo translation 3 (float)
constexpr uint16_t kSize     = 0x00C0;   // bytes to read: header through the stereo translation

// Distance from a block start to the first distortion coefficient.
constexpr uint16_t kDistOff    = 0x0018;
constexpr uint8_t  kDistCount  = 8;

// memcpy avoids unaligned typed access; the EEPROM representation must match host byte order.
// EEPROM stores 4-byte floats; widening to double is lossless
double read_float(const uint8_t *buf, uint16_t off)
{
    float v;
    memcpy(&v, &buf[off], sizeof(v));
    return v;
}

// Intrinsics block: fx,fy,cx,cy followed by 8 pinhole (radtan) distortion coefficients, all float
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

    cam->dist_model = DistModel::Pinhole;
    for(uint8_t i = 0; i < kDistCount; i++)
        cam->dist_coeffs[i] = read_float(buf, off + kDistOff + i * 4);
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

    // Re-check the header: probe() may have matched a different model, and this driver is
    // also reachable when a caller reads without probing.
    if(memcmp(buf, kHeader, kHeaderSize) != 0)
        return Status::HwError;

    *out = StereoImuModel{};

    // The EEPROM L/R blocks map directly to the module's left/right cameras
    read_intrinsics(buf, &out->cam_left,  kLBlock);
    read_intrinsics(buf, &out->cam_right, kRBlock);

    out->install_angle = static_cast<int>(buf[kRotationOff]) * kQuarterTurnDeg;

    // This layout uses the right camera as the common reference frame, so cam_right is the identity transform
    for(uint8_t i = 0; i < 9; i++){
        out->cam_right_R[i] = (i % 4 == 0) ? 1.0f : 0.0f;
        out->cam_left_R[i]  = read_float(buf, kStereoR + i * 4);
    }
    for(uint8_t i = 0; i < 3; i++){
        out->cam_right_T[i] = 0.0f;
        out->cam_left_T[i]  = read_float(buf, kStereoT + i * 4);
    }

    // No IMU on this module: its area reads back as 0xFF and carries no checksum to verify,
    // so nothing is invented for the IMU fields and the read still succeeds.  The device is
    // registered with imu_config.bus_num = 0, so these fields are not consumed.
    return Status::Ok;
}

} // namespace

extern const ModelDesc kSZYGSJKJStereoPinholeV1P1R1N8 = {
    "SZYGSJKJ Stereo Pinhole V1.1 Rotate-90-deg 8-Distortion-parameters",
    "- Calibration manufacturer:    SZYGSJKJ\n"
    "- Distortion model:            Pinhole-8-parameters [k1,k2,p1,p2,k3,k4,k5,k6],\n"
    "- Calibration version:         V1.1\n"
    "- Device Type:                 Stereo Camera\n"
    "- Rotating installation angle: 90 degrees\n",
    probe,
    read,
};

GS130_EEPROM_REGISTER_MODEL(kSZYGSJKJStereoPinholeV1P1R1N8);

} // namespace eeprom
} // namespace gs130
