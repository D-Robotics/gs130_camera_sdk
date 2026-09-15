// Copyright (c) 2026 D-Robotics.
// SPDX-License-Identifier: MIT
//
// Writes the device's calibration in the format hobot_stereonet expects for
// calib_method:=custom.
//
// The node cannot do this itself: under `rect` the C layer replaces the
// intrinsics and the rotation with virtual ones while initialising the camera,
// so the raw fisheye calibration is gone by the time the node is running.  It
// is still in the EEPROM, and opening the device without rectification hands it
// back -- which is all this tool does, plus writing it out.
//
// stereonet's custom path rectifies from this file instead of resizing the
// rectified image its camera published, so it needs the raw model: the
// equidistant intrinsics and distortion of each eye, and the transform from the
// left eye to the right.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <vector>

#include "gs130.h"

#include "gs130_ros/config.hpp"

namespace
{

void usage(const char * program)
{
  std::fprintf(
    stderr,
    "usage: %s [--output FILE] [--platform NAME] [--device NAME]\n"
    "          [--width W] [--height H] [--fov-scale S]\n"
    "\n"
    "Writes the raw camera calibration (fisheye intrinsics, distortion and the\n"
    "stereo transform) as the cam0/cam1 YAML that hobot_stereonet reads with\n"
    "calib_method:=custom.  The device is opened without rectification, so the\n"
    "width and height must be the sensor size.\n"
    "\n"
    "--fov-scale sets cam1's fov_scale, the divisor stereonet applies to the\n"
    "rectified focal length.  Without it stereonet picks 0.8, which on a GS130WI\n"
    "gives a 124.7 deg horizontal field of view, wider than the fisheye's own\n"
    "94.8 deg, so the rectified image has black wedges down both sides.  Passing\n"
    "0.455 gives a 94.75 deg field of view and no black.  See README.md.\n",
    program);
}

struct Options
{
  std::string output = "gs130_stereo_calib.yaml";
  std::string platform = "RDKX5";
  std::string device = "GS130WI";
  uint32_t width = 1088;
  uint32_t height = 1280;
  double fov_scale = 0.0;  // 0 leaves the key out and lets stereonet choose
};

Options parse(int argc, char ** argv)
{
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string flag = argv[i];
    auto value = [&]() -> std::string {
        if (i + 1 >= argc) {
          throw std::invalid_argument(flag + " needs a value");
        }
        return argv[++i];
      };
    if (flag == "--output") {
      options.output = value();
    } else if (flag == "--platform") {
      options.platform = value();
    } else if (flag == "--device") {
      options.device = value();
    } else if (flag == "--width") {
      options.width = static_cast<uint32_t>(std::stoul(value()));
    } else if (flag == "--height") {
      options.height = static_cast<uint32_t>(std::stoul(value()));
    } else if (flag == "--fov-scale") {
      options.fov_scale = std::stod(value());
    } else if (flag == "-h" || flag == "--help") {
      usage(argv[0]);
      std::exit(0);
    } else {
      throw std::invalid_argument("unknown option: " + flag);
    }
  }
  return options;
}

/// A sequence on one line, full precision: every digit here ends up in a
/// rectification the device cannot re-derive.
void write_sequence(std::ostream & out, const double * values, size_t count, int indent)
{
  out << "[";
  for (size_t i = 0; i < count; ++i) {
    out << (i ? ", " : "") << std::setprecision(17) << values[i];
  }
  out << "]";
  (void)indent;
}

void write_camera(
  std::ostream & out, const char * name, const gs130_camera_intrinsics_t & intrinsics,
  uint32_t width, uint32_t height, const char * indent)
{
  const bool fisheye = intrinsics.dist_model == GS130_DIST_FISHEYE;
  const size_t count = fisheye ? 4 : 8;

  out << indent << name << ":\n";
  out << indent << "  camera_model: pinhole\n";
  out << indent << "  intrinsics: ";
  const double intrinsics4[4] = {intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy};
  write_sequence(out, intrinsics4, 4, 2);
  out << "\n";
  out << indent << "  distortion_model: " << (fisheye ? "equidistant" : "rational_polynomial") << "\n";
  out << indent << "  distortion_coeffs: ";
  write_sequence(out, intrinsics.dist_coeffs, count, 2);
  out << "\n";
  out << indent << "  resolution: [" << width << ", " << height << "]\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  Options options;
  try {
    options = parse(argc, argv);
  } catch (const std::exception & error) {
    std::fprintf(stderr, "%s\n\n", error.what());
    usage(argv[0]);
    return 2;
  }

  gs130_device_t * device = nullptr;
  try {
    // Raw mode is what keeps the real calibration intact: rect would replace
    // the intrinsics with the virtual pair and the rotation with the
    // rectified one.
    const gs130_config_t config = gs130_ros::make_config(
      options.platform, options.device, GS130_CAMERA_MODE_RAW,
      options.width, options.height, 30, 200, GS130_STEREO_LAYOUT_NONE);

    device = gs130_create();
    if (device == nullptr) {
      throw std::runtime_error("gs130_create() returned null");
    }
    gs130_ros::check(gs130_init(device, &config), "gs130_init");

    gs130_calibration_t calibration{};
    gs130_ros::check(gs130_get_calibration(device, &calibration), "gs130_get_calibration");

    // relative_R/T(LEFT, RIGHT) is the transform that takes a point from the
    // left eye's frame into the right eye's, which is what stereonet's
    // T_cn_cnm1 holds: p_cam1 = T_cam1_cam0 * p_cam0, with cam0 = left.
    double rotation[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    double translation[3] = {0, 0, 0};
    gs130_ros::check(
      gs130_get_relative_R(device, GS130_REF_CAMERA_LEFT, GS130_REF_CAMERA_RIGHT, rotation),
      "gs130_get_relative_R");
    gs130_ros::check(
      gs130_get_relative_T(device, GS130_REF_CAMERA_LEFT, GS130_REF_CAMERA_RIGHT, translation),
      "gs130_get_relative_T");

    const char * eeprom = gs130_get_eeprom_name(device);
    std::ofstream out(options.output);
    if (!out) {
      throw std::runtime_error("cannot write " + options.output);
    }

    // OpenCV's FileStorage reader wants the %YAML directive first and rejects
    // the file without it, comments or no comments, so the provenance goes
    // last and the document starts with the directive like every calibration
    // file D-Robotics ships.
    out << "%YAML:1.0\n";
    out << "stereo0:\n";
    write_camera(out, "cam0", calibration.camera_left, options.width, options.height, "  ");
    write_camera(out, "cam1", calibration.camera_right, options.width, options.height, "  ");

    if (options.fov_scale > 0.0) {
      out << "    fov_scale: " << std::setprecision(4) << options.fov_scale << "\n";
    }
    out << "    T_cn_cnm1:\n";
    for (int row = 0; row < 3; ++row) {
      out << "      - [" << std::setprecision(17)
          << rotation[row * 3 + 0] << ", " << rotation[row * 3 + 1] << ", "
          << rotation[row * 3 + 2] << ", " << translation[row] << "]\n";
    }
    out << "      - [0.0, 0.0, 0.0, 1.0]\n";

    out << "#\n";
    out << "# Generated by gs130_calibration from the camera EEPROM.\n";
    out << "# eeprom: " << (eeprom ? eeprom : "unknown") << "\n";
    out << "# install_angle: " << calibration.camera_install_angle << " deg\n";
    out << "# libgs130: " << gs130_version() << " (" << gs130_platform() << ")\n";
    out << "#\n";
    out << "# cam0 is the left eye and cam1 the right, and T_cn_cnm1 is the\n";
    out << "# transform from cam0 to cam1: p_cam1 = T_cn_cnm1 * p_cam0.\n";
    out << "#\n";
    out << "# Resolution is the sensor size, because these are the unrectified\n";
    out << "# intrinsics: run the camera with camera_mode:=raw and hobot_stereonet\n";
    out << "# with calib_method:=custom stereo_calib_file_path:=this file.\n";
    out.close();

    const double baseline = std::sqrt(
      translation[0] * translation[0] + translation[1] * translation[1] +
      translation[2] * translation[2]);
    std::printf(
      "wrote %s\n  cam0/cam1: %.4f x %.4f px, %s\n  baseline (cam0 -> cam1): %.6f m\n",
      options.output.c_str(), calibration.camera_left.fx, calibration.camera_left.fy,
      calibration.camera_left.dist_model == GS130_DIST_FISHEYE ? "equidistant" : "rational_polynomial",
      baseline);
    if (!(options.fov_scale > 0.0)) {
      std::printf(
        "  fov_scale: not written, so stereonet picks its own 0.8 -- a wider field\n"
        "  of view than the fisheye has, with black down the sides.  --fov-scale\n"
        "  0.455 matches the sensor instead.\n");
    }
  } catch (const std::exception & error) {
    std::fprintf(stderr, "gs130_calibration: %s\n", error.what());
    if (device != nullptr) {
      gs130_stop(device);
      gs130_deinit(device);
      gs130_destroy(device);
    }
    return 1;
  }

  gs130_stop(device);
  gs130_deinit(device);
  gs130_destroy(device);
  return 0;
}
