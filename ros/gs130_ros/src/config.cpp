// Copyright (c) 2026 D-Robotics.
// SPDX-License-Identifier: MIT

#include "gs130_ros/config.hpp"

#include <algorithm>
#include <cstring>

namespace gs130_ros
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

Gs130Error::Gs130Error(gs130_err_t code, const std::string & function)
: std::runtime_error(
    function + "() -> " + error_name(code) + " (" + std::to_string(static_cast<int>(code)) + ")"),
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
  // Every entry the macros leave alone starts at its C zero value here too, so
  // this can be compared with gs130_define.h field by field.

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

}  // namespace gs130_ros
