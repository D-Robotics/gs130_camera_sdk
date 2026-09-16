# GS130 Camera SDK

**简体中文** | [English](README.md)

GS130 Camera SDK 提供在 RDK 系列开发板上使用 GS130 双目相机及板载 IMU 所需的软件组件，由一个原生 C 库，以及构建于其上的 Python 与 ROS 2 两个封装层组成。

> **开发者预览（Alpha）版本。** 本版本供评估使用，接口、配置预设与行为可能随时调整，恕不另行通知。欢迎通过[地瓜机器人开发者社区](https://forum.d-robotics.cc/)或邮件 [xiaoye.zhang@d-robotics.cc](mailto:xiaoye.zhang@d-robotics.cc) 反馈问题、参与共建。

---

## 📖 简介

硬件链路由原生库统一实现，包括传感器采集与 ISP 处理、GDC 硬件校正、从板载 EEPROM 读取标定数据，以及通过 FSYNC 与相机时基对齐的 IMU 采样。以下三个层次分别对外提供接口，并独立发布。

| 层 | 目录 | 内容 |
| --- | --- | --- |
| Core | `core/` | `libgs130`、公共 C 头文件、gs130 工具与 Debian 打包 |
| Python 封装 | `python/` | `gs130_camera` 包，通过 `ctypes` 调用 `libgs130` |
| ROS 2 封装 | `ros2/` | `gs130_camera` 包，发布相机、IMU 与 TF 数据 |

必须先安装 Core，两个封装层方可使用。

## 🧩 依赖

- **硬件**：RDK 系列开发板与 GS130 系列双目相机。部分机型带 IMU。
- **Core**：地平线多媒体库（`libvpf`、`libhbmem` 与 `libcam`，安装于 `/usr/hobot/lib`）、OpenCV 4 开发头文件、`libtbb.so.2`，以及支持 C11 与 C++17 的 GCC 工具链。
- **Python 封装**：Python 3.10 或更高版本与 `numpy`；构建 wheel 还需要 `setuptools` 与 `wheel`。
- **ROS 2 封装**：ROS 2 Humble 或 Jazzy。网页预览与双目深度两个 launch 还需要 TROS 的 `hobot_codec`、`websocket` 与 `hobot_stereonet`。

本版本实现了 RDK X5 后端，RDK S100 与 RDK S600 的支持仍在开发中。

## 🗂️ 仓库结构

```
core/
  include/gs130.h            公共 C API
  include/gs130_define.h     各平台配置预设
  src/                       库实现（base、devices、tools）
  samples/                   gs130 前端与示例程序
python/
  gs130_camera/              Python 封装，含类型存根
  test/test_gs130.py         硬件测试
ros2/src/gs130_camera/       ROS 2 包：节点、launch 文件与文档
VERSION                      Core 与 Python 封装共用的版本号
LICENSE
```

## 🔨 构建与安装

### 1. Core

库、工具与 Debian 包由 `core/` 下的 GNU Makefile 构建。`make` 可接受平台名，但仅当它位于命令行的第一个目标位置时才会生效。

```bash
cd core
make -j$(nproc)            # 构建库、示例、工具与 Debian 包
make lib                   # 仅构建库
make RDKX5 samples         # 显式指定平台
```

构建产物写入 `build/<平台>/` 与 `out/<平台>/`，Debian 包写入 `out/`。在终端中运行 `make` 时，编译前会请求一次确认；标准输入不是终端时不会询问。

```bash
# 检查 SDK 是否已安装
dpkg -s gs130-camera

# 若未安装，安装由 core/ 构建出的 Debian 包
sudo dpkg -i out/gs130-camera_<version>+<platform>_<arch>.deb
```

安装 `gs130-camera` 即可满足后续步骤的要求。

### 2. Python 封装

wheel 在板端使用已安装的工具链构建。

```bash
cd python && ./build-wheel.sh
python3 -m pip install dist/gs130_camera-*.whl
```

`./build-wheel.sh clean` 用于清理构建目录与生成的元数据。版本号取自仓库根目录的 `VERSION` 文件，因此需从源码目录构建。wheel 不包含原生库，运行时将定位已安装的 `libgs130`。

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

**命令行工具**

gs130 工具随 Core 一同安装，示例源码保存在 [core/samples/](core/samples)。这些程序既是诊断工具，也可作为参考实现。

```bash
gs130 help                       # 列出可用命令
gs130 version                    # SDK 版本与构建平台
gs130 detect <imu|eeprom|camera> [-b <bus...>] [-a <addr...>]
gs130 shell -d <device> [-m <mode>] [-w <W>] [-h <H>] [-f <fps>] [-o <odr>]
```

`gs130 shell` 会固定一份设备配置，随后以交互方式接受其余命令。

```console
gs130-shell>> imu-info                                                              # 检测到的 IMU 型号与详情
gs130-shell>> eeprom-info                                                           # EEPROM 标定型号与详情
gs130-shell>> calib-export <dir>                                                    # 导出 Kalibr YAML：camchain.yaml 与 imu.yaml
gs130-shell>> run                                                                   # 同时取流相机与 IMU，打印各自最新数据
gs130-shell>> rec [-c <from:to>] [-i <from:to>] -o <dir> [--stitch]                 # 按索引区间录制到磁盘
```

**C 语言**

公共 API 声明于 `core/include/gs130.h`，描述已知硬件的平台预设定义于 `core/include/gs130_define.h`。程序链接 `libgs130`，创建设备句柄、应用配置，并从中读取图像帧与 IMU 数据。

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

包含速率统计与中断退出处理的完整程序见 [core/samples/](core/samples)。

**Python**

Python 封装以 NumPy 数组返回图像帧与 IMU 数据。

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

ROS 2 封装将相机、标定数据与 IMU 发布为标准消息。话题与参数参考见该包的文档。

```bash
ros2 launch gs130_camera gs130.launch.py             # 图像、标定、IMU 与 TF
ros2 launch gs130_camera gs130_websocket.launch.py   # 同上，并在浏览器中预览
ros2 launch gs130_camera gs130_stereonet.launch.py   # 在同一页面上查看双目深度
```

## ✨ 特性

- **硬件级校正。** 在 `rect` 模式下，去畸变与校正由 GDC 硬件完成，无需主机侧图像处理。
- **可选双目布局。** 双目可分别输出，也可按四种布局合并为单帧。
- **EEPROM 标定。** 内参、畸变系数、外参与 IMU 参数均从板载 EEPROM 读取，并支持导出为 Kalibr YAML；校正后会写回虚拟内参。
- **FSYNC 对齐的 IMU。** 以相机为 IMU 的时间基准，两路数据统一在同一时钟上表示。
- **标准 ROS 2 接口。** 图像、相机信息、惯性数据与静态变换均以标准消息发布，不引入自定义消息类型。
- **版本一致。** 原生库与 Python 封装共用 `VERSION` 中的版本号，封装会拒绝加载版本低于软件包的原生库。

本版本不提供以下能力：零拷贝传输（基于共享内存的 `hbm_img_msgs`）、自定义消息类型、IMU 滤波、参数运行时重配置，以及多设备同步。

## 📄 许可证

本项目采用 [MIT 许可证](LICENSE)。
