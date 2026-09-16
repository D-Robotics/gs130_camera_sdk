# GS130 Camera SDK

**简体中文** | [English](README.md)

> GS130 双目相机与 IMU 的软件开发包，包含 C 库、Python 封装与 ROS 2 封装。
>
> **当前为开发者预览（alpha）版本**，接口与行为仍可能调整。欢迎提出意见或参与共创：
> [地瓜机器人开发者社区](https://forum.d-robotics.cc/)，或邮件 [xiaoye.zhang@d-robotics.cc](mailto:xiaoye.zhang@d-robotics.cc)。

---

## 📖 简介

本仓库是 GS130 系列双目相机的 SDK。硬件工作由同一个原生库完成——传感器采集、ISP、GDC 校正、EEPROM 标定与 FSYNC 对齐的 IMU 采样——其上分三层对外提供接口：

| 层 | 目录 | 内容 |
| --- | --- | --- |
| Core | `core/` | gs130 tools、C-API 源码 |
| Python 封装 | `python/` | `gs130_camera` 库构建源码 |
| ROS 2 封装 | `ros2/` | `gs130_camera` 功能包构建源码 |

| 项目 | 说明 |
| --- | --- |
| 支持平台 | RDK X5（RDK S100 / RDK S600 开发中） |
| 许可证 | MIT |
| 语言 | C11 & C++17（库）、Python 3.10+（封装）、C++17（ROS 2 节点） |

## 🧩 依赖

- **硬件**：RDK 系列开发板 + GS130 系列双目相机（部分型号带 IMU）。
- **Core**：地平线多媒体库（`libvpf`、`libhbmem`、`libcam`，位于 `/usr/hobot/lib`）、OpenCV 4 头文件、`libtbb.so.2`，以及支持 C11 & C++17 的 GCC 工具链。
- **Python 封装**：Python 3.10 或更高、`numpy`，构建 wheel 还需要 `setuptools` 与 `wheel`。
- **ROS 2 封装**：ROS 2 Humble 或 Jazzy。网页预览与双目深度两个 launch 还需要 TROS 的 `hobot_codec`、`websocket` 与 `hobot_stereonet`。

三层都依赖原生库，因此请先构建并安装 `core/`。

## 🗂️ 仓库结构

```
core/
  include/gs130.h            公共 C API
  include/gs130_define.h     各平台配置预设
  src/                       库实现（base、devices、tools）
  samples/                   gs130 前端与示例程序
python/
  gs130_camera/              封装实现，含类型存根
  test/test_gs130.py         硬件测试
ros2/src/gs130_camera/       ROS 2 包：节点、launch 文件、README
VERSION                      库与封装共用的版本号
LICENSE
```

## 🔨 构建与安装

### 1. Core

```bash
cd core
make -j$(nproc)            # 为默认平台构建库、示例、工具与 .deb
make lib                   # 只构建库
make RDKX5 samples         # 平台名只有放在第一个目标位置才生效
```

产物分别位于 `build/<平台>/`、`out/<平台>/`，`.deb` 则放在 `out/` 下。交互式终端在编译前会询问一次确认；非交互场景自动跳过。

```bash
# 检查是否已安装
dpkg -s gs130-camera

# 若未安装，安装由 core/ 构建出的 deb
sudo dpkg -i out/gs130-camera_<version>+<platform>_<arch>.deb
```

将 `gs130-camera` 安装即可。

### 2. Python 封装

```bash
cd python && ./build-wheel.sh
python3 -m pip install dist/gs130_camera-*.whl
```

详见 [Python README](python/README.zh-CN.md)。

### 3. ROS 2 封装

```bash
cd ros2
source /opt/ros/humble/setup.bash     # 或：source /opt/ros/jazzy/setup.bash
colcon build --packages-select gs130_camera
source install/setup.bash
```

详见 [ROS 2 README](ros2/src/gs130_camera/README.zh-CN.md)。

## 🚀 快速开始

**gs130 tools**

随 Core 一起安装：`.deb` 会把这些程序装到 `/usr/bin`。源码是 [core/samples/](core/samples) 下的文件，三个 `detect` 工具在 [core/src/tools/](core/src/tools)——它们既是测试工具，也是可以直接阅读的示例。

```bash
gs130 help                       # 列出全部命令
gs130 version                    # SDK 版本与构建平台
gs130 detect <imu|eeprom|camera> [-b <bus...>] [-a <addr...>]
gs130 shell -d <device> [-m <mode>] [-w <W>] [-h <H>] [-f <fps>] [-o <odr>]
```

`shell` 固定一份设备配置后进入交互，其中可用：

```console
gs130-shell>> imu-info                                                              # 检测到的 IMU 型号与详情
gs130-shell>> eeprom-info                                                           # EEPROM 标定型号与详情
gs130-shell>> calib-export <dir>                                                    # 导出 Kalibr YAML：camchain.yaml 与 imu.yaml
gs130-shell>> run                                                                   # 同时取流相机与 IMU，打印各自最新数据
gs130-shell>> rec [-c <from:to>] [-i <from:to>] -o <dir> [--stitch]                 # 按索引区间录制到磁盘
```

**C 语言**

链接 `libgs130`，在自己的 C 程序里驱动相机：

```c
#include "gs130.h"
#include "gs130_define.h"

#include <stdlib.h>
#include <unistd.h>

/* 取本平台、本机型对应的预设配置 */
gs130_config_t config = GS130_CONFIG(
    "GS130WI", GS130_CAMERA_MODE_RECT, 544, 448, 30, 200);

gs130_device_t *device = gs130_create();
if (gs130_init(device, &config) != GS130_OK ||
    gs130_start(device) != GS130_OK) {
    gs130_stop(device);
    gs130_deinit(device);
    gs130_destroy(device);
    return 1;
}

for (;;) {
    gs130_image_nv12_t left, right;
    while (gs130_get_nv12_frame(device, &left, &right) == GS130_OK) {
        /* NV12，宽 * 高 * 3 / 2 字节；缓冲区由调用方负责释放 */
        free(left.data);
        free(right.data);
    }
    /* 非阻塞：这里排空已到达的数据，队列空时结束循环。
       没有 IMU 的设备不会返回任何 packet。 */
    gs130_imu_packet_t packet;
    while (gs130_get_imu_packet(device, &packet) == GS130_OK) {
        /* packet.accel 单位 m/s^2，packet.gyro 单位 rad/s，packet.temp 单位 ℃；
           packet.timestamp_ns 基于相机时钟。 */
    }

    usleep(1000);
}

gs130_stop(device);
gs130_deinit(device);
gs130_destroy(device);
```

带 Ctrl-C 处理与速率统计的完整程序，见 [core/samples/](core/samples)。

**Python**

```python
import gs130_camera

config = gs130_camera.preset(
    "GS130WI", gs130_camera.CameraMode.RECT, 544, 448, 30, 200
)
with gs130_camera.Device(config) as device:
    device.start()
    while device.available_camera() == 0:
        pass
    left = device.read_image()["left"]

    # 队列为空时返回 None；没有 IMU 的设备同样返回 None
    packet = device.read_imu()
    if packet is not None:
        # packet.accel 单位 m/s^2，packet.gyro 单位 rad/s，packet.temp 单位 ℃
        print(packet.accel, packet.gyro, packet.temp)
```

**ROS 2**

```bash
ros2 launch gs130_camera gs130.launch.py             # 图像、标定、IMU 与 TF
ros2 launch gs130_camera gs130_websocket.launch.py   # 同上，并在浏览器中预览
ros2 launch gs130_camera gs130_stereonet.launch.py   # 在同一页面上查看双目深度
```

## ✨ 特性

- **硬件级校正**：`rect` 模式的去畸变与校正由 GDC 硬件完成，无需主机侧图像处理。
- **五种双目布局**：不拼接、左右、右左、上下、下上。
- **EEPROM 标定**：内参、畸变、外参与 IMU 参数，支持导出 Kalibr YAML，并在校正后写回虚拟内参。
- **FSYNC 对齐的 IMU**：以相机为时间基准对齐 IMU，两路数据共用一个时钟。
- **标准 ROS 2 输出**：`sensor_msgs/Image`、`sensor_msgs/CameraInfo`、`sensor_msgs/Imu` 与静态 TF，不使用自定义消息。
- **版本可校验**：库与封装共用 `VERSION`，加载到比包版本更旧的库时拒绝加载，更新的库只给出告警。

## 📄 许可证

本项目采用 [MIT 许可证](LICENSE)。
