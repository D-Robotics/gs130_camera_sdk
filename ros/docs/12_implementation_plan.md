# GS130 ROS 2 (TROS) 接口包实施与迭代计划

文档编号：12
适用仓库：`gs130_sdk`，分支 `develop`，目标包目录 `ros/`
作者角色：DEV-3（implementation）
前置事实：板端环境（TROS humble / `/opt/tros/humble`、Python 3.10、colcon、rclpy 与 sensor_msgs / geometry_msgs / tf2_msgs、OpenCV 4.11、numpy 1.26）以及既有可复用节点（`hobot_codec`、`websocket`、`hobot_shm`、`hobot_stereonet`）均已验证，本文不再重复验证。

---

## 0. 目标与非目标摘要

**目标**：用最少的文件、最少的抽象，把 GS130 的双目 NV12 帧、IMU 数据与 EEPROM 标定数据变成 ROS 2 话题，并复用 D-Robotics 既有节点让**现有 TROS web UI（8000 端口）显示相机画面**。整个包只做“展示（showcase）”：帧能看、IMU 能订、标定能读。

**交付边界**：ROS 侧全部新增代码约 **300 行 Python + 约 100 行配置/脚本**，3 个可执行节点（其中 2 个是真正的节点进程，1 个是可选标定进程），无自定义 msg/srv，无 C++ 节点。

---

## 1. 文件树与逐文件职责

全部位于 `ros/` 之下（`probe_nv12_publisher.py` 是既有的一次性 bring-up 探针，不属于本包，不改动、不移动）。

```
ros/
├── docs/
│   └── 12_implementation_plan.md          # 本文档：实施与迭代计划（约 400 行，仅文档）
├── gs130_ros/
│   ├── package.xml                        # ament 包清单：包名、依赖、许可证（约 30 行）
│   ├── setup.py                           # ament_python 构建脚本：packages、data_files、3 个 entry point（约 35 行）
│   ├── setup.cfg                          # 把 console_scripts 安装到 lib/<pkg>（6 行）
│   ├── resource/
│   │   └── gs130_ros                       # ament 资源标记（空文件，0 行）
│   ├── LICENSE                            # 与仓库根一致的 MIT 文本（复制，不新写）
│   ├── launch/
│   │   ├── camera.launch.py               # 摄像头链路：gs130 节点 + 2×hobot_codec + websocket（约 85 行）
│   │   └── calibration.launch.py          # 只启动标定节点，供单独验证（约 25 行）
│   └── gs130_ros/
│       ├── __init__.py                    # 包标记与版本，无逻辑（约 5 行）
│       ├── camera_node.py                 # 主节点：打开设备、发布双目 NV12 与 IMU（约 170 行）
│       ├── calibration.py                 # 标定数据 -> CameraInfo / TransformStamped 的纯函数（约 55 行）
│       └── calibration_node.py            # 标定节点：发布左右 CameraInfo 与静态 TF（约 80 行）
└── test/
    └── fake_frames.py                     # 主机侧冒烟：未安装 gs130 时注入假帧与假 IMU（约 65 行）
```

合计 10 个文件（含本文档与 LICENSE）。除显而易见的 *节点 / launch / 包元数据* 之外的 4 个文件，逐一说明为什么必须存在：

| 额外文件 | 为什么不能省 |
| --- | --- |
| `gs130_ros/calibration.py` | `CameraInfo` 与 `TransformStamped` 的字段映射（畸变模型选择、行/列主序、单位换算、`calibration()` 结果与 `camera_intrinsics()` 的一致化）是纯计算，放在节点类里会与 rclpy 生命周期纠缠；拆成纯函数后可在主机上无 ROS 直接断言数值。约 55 行，不构成过度设计。 |
| `gs130_ros/calibration_node.py` | 标定必须能在相机被占用/相机启动失败时独立运行（EEPROM 与 Camera 是同一 `gs130_init`，但标定失败不该拖垮图像链路；反之图像链路调试时也不必反复重建 CameraInfo）。独立进程也避免主节点持有两套发布器。 |
| `launch/calibration.launch.py` | `camera.launch.py` 的默认参数不含相机标定开关；用一个 25 行的 launch 明确“只验标定”这条独立路径，避免在 `camera.launch.py` 里塞 `if/else` 分支。 |
| `test/fake_frames.py` | 无板开发者的唯一验证手段，且它不属于安装内容（不写进 `setup.py` 的 `packages`/`data_files`）。见第 6 节。 |

被明确**否决**的文件（防止后续迭代偷偷加回来）：`gs130_ros/device.py`（SDK 包装层，`gs130.Device` 已经是包装层）、`gs130_ros/frame.py`（NV12 尺寸换算只有 3 行，写在原处）、`gs130_ros/params.py`（参数声明一处写完即可）、自定义 `msg/` 与 `srv/`（无新数据类型的必要）、`test/` 下的 pytest 套件（本阶段只保留可手跑的主机冒烟）。

---

## 2. 主节点 `gs130_ros/camera_node.py` 函数级大纲

### 2.1 模块结构

```python
"""ROS 2 interface for the GS130 stereo camera."""

import argparse          # 仅为 main() 的 argv 覆盖
import os                # 仅为 fake 帧开关
import sys
import time              # fake IMU 的时间戳

import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu
```

`gs130`、`rclpy` 本身在模块顶层导入（主机上装了 wheel 即可 import，见第 6 节）；**不**在顶层创建 `Device`、**不**在顶层做任何 I/O，保证 `import gs130_ros.camera_node` 是零副作用的。

### 2.2 函数逐个说明

| # | 签名 | 作用（一句话） | 包装的 SDK 调用 |
| --- | --- | --- | --- |
| 1 | `main(argv=None) -> int` | 解析 argv、`rclpy.init`、构造并 spin 节点、在 finally 中关闭设备并 `rclpy.shutdown`。 | — |
| 2 | `build_device(node) -> gs130.Device` | 由 ROS 参数组装 `gs130.Config.preset(...)` 并构造（未 `start`）`Device`；参数非法时抛 `ValueError`。 | `gs130.Config.preset(platform, device, mode, width, height, fps, odr)`、`gs130.Device(config)` |
| 3 | `default_config(platform, device, mode, width, height, fps, odr) -> gs130.Config` | 在 preset 之上做两条硬性校验（见下），返回可直接交给 `Device` 的配置。 | `gs130.Config.preset` |
| 4 | `class CameraNode(Node).__init__(self)` | 声明参数、取参数、构造 `Device`、`start()`、建发布器、建两个定时器。 | `Device.start()`、`Device.stitched`、`Device.imu_name` |
| 5 | `CameraNode._declare_parameters(self)` | 用一张表声明全部参数及默认值（唯一声明处，无重复代码）。 | — |
| 6 | `CameraNode.publish_images(self)` | 单次轮询：把队列里已有帧对取空并逐对发布；返回本次发布的帧对数（供测试断言）。 | `Device.read_image()`（非阻塞，返回 `dict` 或 `None`） |
| 7 | `CameraNode.publish_imu(self)` | 单次轮询：取空 IMU 队列（上限 `imu_burst` 包），逐包发布 `sensor_msgs/Imu`；返回包数。 | `Device.read_imu()`（非阻塞，返回 `ImuPacket` 或 `None`） |
| 8 | `CameraNode.image_message(self, image, frame_id) -> Image` | 由 NV12 ndarray 构造 `sensor_msgs/Image`：填 `width/height/step/encoding="nv12"`、`frame_id`、时间戳；`data` 为一次性拷贝，随后 SDK 缓冲随 ndarray 释放。 | — （`Image` 的零拷贝地址由 `gs130._types.Image` 持有，Python 层在 `read_image()` 内已 copy + free） |
| 9 | `CameraNode.imu_message(self, packet, frame_id) -> Imu` | 由 `ImuPacket` 构造 `sensor_msgs/Imu`，写入角速度、线加速度与时间戳。 | — |
| 10 | `CameraNode.stamp(self, timestamp_ns) -> rclpy.time.Time` | 把相机纳秒时间戳转成 ROS 时间；时间戳明显不是 Unix 纪元时退化为节点时钟并只告警一次。 | — （数据来自 `Image.timestamp_ns` / `ImuPacket.timestamp_ns`） |
| 11 | `CameraNode.shutdown(self)` | 幂等收尾：`destroy_timer`、`Device.stop()`、`Device.close()`、`destroy_node()`。 | `Device.stop()`、`Device.close()` |
| 12 | `CameraNode._frames(self)` | 唯一的硬件/假数据分叉点：`fake_frames` 打开时返回合成帧对与假 IMU 包，否则转发到设备读取。 | `Device.read_image()` / `Device.read_imu()`（fake 分支不调用） |
| 13 | `create_fake_pair(width, height) -> dict` | 生成一张左亮右暗、带棋盘格的 NV12 帧对，用于主机冒烟（第 6 节）。 | — |
| 14 | `create_fake_imu() -> gs130.ImuPacket` | 生成一个常量重力 + 零角速度的 `ImuPacket`。 | — |

### 2.3 非阻塞读取如何被驱动（timer，单线程）

- 节点持有**一个** rclpy 定时器 `image_timer`，周期 `image_period = 1/(2*fps)` 秒（默认 30 fps → 60 Hz），回调是 `publish_images`。另有一个 `imu_timer`，周期 `1/odr_s`（默认 200 Hz），回调是 `publish_imu`。
- 两个回调都在**同一个 executor、同一个线程**里运行（`rclpy.spin(node)` 单线程），所以 `Device` 上的调用天然串行，不引入锁，也不需要后台线程 —— 这与 `core/samples/gs130-run.c` 的 `while(!stop)` 轮询语义等价，只是把 `usleep` 换成了 ROS 定时器。
- 回调内是“取空为止”的小循环，且**每次回调最多取 `frame_burst`（默认 4）对帧 / `imu_burst`（默认 32）包**，防止一次回调长时间占住 executor 而延迟 Ctrl-C 与参数服务。取空靠 `read_image()` 返回 `None` 判断（SDK 队列可能积压多帧，因为采集线程独立于本节点）。
- 不使用 `while True` 阻塞循环、不使用 `MultiThreadedExecutor`、不使用 `rclpy.timer.Rate`：定时器就是本包的全部调度机制。
- `image_period` 取 2 倍帧率的理由：SDK 队列由采集线程异步填充，2 倍轮询频率把“帧已到达但还没被取走”的平均延迟压到一个采集周期以内，同时保持 `read_image()` 非阻塞（无数据时立即返回 `None`，不空转 CPU）。

### 2.4 帧如何拷进 ROS 消息、SDK 缓冲如何释放

链路（每一跳的所有权都是明确的）：

1. `Device.read_image()` 内部调用 `gs130_get_nv12_frame()`，libgs130 用 `malloc()` 分配 `data`（`core/include/gs130.h` 明确：**所有权归调用方，用后须 `free()`**）。
2. Python 绑定在 `gs130/_types.py` 中用一个 `(ctypes.c_uint8 * size).from_address(addr)` 视图 + `weakref.finalize(owner, libc.free, addr)` 接管该缓冲，再把内容 `copy()` 进 ndarray。因此 **numpy ndarray 一旦被释放，libc `free()` 立即执行**。
3. 节点侧只保留一个引用：局部变量 `image`（来自 `pair["left"] / pair["right"] / pair["stitched"]`）。
4. `image_message()` 里用 `message.data = bytes(image)` 一次性拷出紧凑 NV12（`bytes(ndarray)` 等价于 `tobytes()`，在 C 连续数组上是零额外开销的整体拷贝）。
5. 发布后回调返回，`image` 局部引用消失 → ndarray 释放 → `weakref.finalize` 调用 `free()` → SDK 缓冲归还。**本包不调用 `free()`、不持有跨回调的帧引用、不做帧缓存**。

尺寸与步长（NV12 紧排：Y 平面 `width*height` 字节，其后 UV 平面 `width*height/2` 字节）：

```python
height = int(image.shape[0]) * 2 // 3     # ndarray 是 (height*3//2, width) 的 2D 视图
width  = int(image.shape[1])
message.width, message.height = width, height
message.step = width                      # Y 平面行字节数；UV 平面紧随其后，仍是 width
message.encoding = "nv12"
message.is_bigendian = 0
```

这与既有 `ros/probe_nv12_publisher.py`（已用于验证 `hobot_codec` 链路）的写法保持一致：`height` 是**真实图像高**，不是 `shape[0]`。若把 `shape[0]` 当成 `height` 填进去，`hobot_codec` 会把 UV 平面当亮度解，画面出现错行与绿边 —— 这是本包最容易犯的错，故写死在 `image_message()` 一处。

### 2.5 两条硬性校验（在 `default_config` 中，fail fast）

1. `mode == "raw"` 时 `output_width/height` 必须等于传感器 `1088 x 1280`（`Config.preset` 的 raw 语义要求），否则抛 `ValueError("raw mode requires 1088x1280, got ...")`。
2. `mode` 只接受 `raw / resize / rect`，`device` 只接受 `GS130WI / GS130W`，`odr` 必须大于 0（无 IMU 的 GS130W 允许 `odr=0`，此时不启动 IMU 定时器）。

错误信息必须点名失败的调用（参见第 3 节）：configure 阶段直接暴露 `gs130` 抛出的 `GS130Error("gs130_init() -> HW_ERROR")`，只加一层上下文前缀，例如 `"cannot open GS130: gs130_init() -> HW_ERROR (is another process holding the camera?)"`。

### 2.6 发布的话题

| 话题 | 类型 | QoS | 说明 |
| --- | --- | --- | --- |
| `/gs130/left/image_raw` | `sensor_msgs/Image` (nv12) | `qos_profile_sensor_data` | 左目 NV12，`frame_id=camera_left_optical_frame` |
| `/gs130/right/image_raw` | `sensor_msgs/Image` (nv12) | `qos_profile_sensor_data` | 右目 NV12，`frame_id=camera_right_optical_frame` |
| `/gs130/stereo/image_raw` | `sensor_msgs/Image` (nv12) | `qos_profile_sensor_data` | **仅当** `stereo_layout != NONE` 时创建；`frame_id=camera_left_optical_frame`，图像来自 `pair["stitched"]` |
| `/gs130/imu/data` | `sensor_msgs/Imu` | `qos_profile_sensor_data` | `frame_id=imu_link`，`orientation_covariance[0] = -1`（显式声明不提供姿态） |

左右目与拼接帧三选一由 `Device.stitched` 决定，`read_image()` 返回的键与之对应，节点不自行拼接、不转换像素格式、不缩放。

---

## 3. 代码风格规则

与 `python/gs130/` 保持一致，逐条可检查：

1. **模块 docstring 一句话讲意图**，例如 `"""ROS 2 interface for the GS130 stereo camera."""`，不写“这个文件包含……”。
2. **函数短**：目标 ≤ 25 行；`publish_images` / `publish_imu` 只做“取数 → 转消息 → 发布”，转换逻辑抽到 `*_message()`。
3. **标识符与文件名全英文**，注释也优先英文（与 `python/gs130` 一致）；中文只出现在本文档这类面向评审的说明里。
4. **无投机性通用性**：不引入基类、插件、工厂、抽象 `Device` 接口，不做“以后可能支持其它相机”的预留参数。
5. **无未使用参数**：不用 `**kwargs` 吞参数；不为将来预留形参（`image_message(self, image, frame_id)` 只收它真正需要的两项）。
6. **不刷日志**：正常运行期**每帧都不打日志**。允许的日志只有：启动时一行配置摘要（`info`）、IMU 缺失一行（`info`）、参数校验失败与时间戳退化（`warn`，后者仅一次）、异常（`error`）。帧率统计不做（`ros2 topic hz` 已能替代）。
7. **错误信息点名失败的调用**：沿用 `python/gs130/_error.py` 的 `"gs130_init() -> HW_ERROR"` 形状，本包自己抛出的消息也写清是哪个动作失败，例如 `"gs130_stream_loop: read_image() failed: gs130_get_nv12_frame() -> HW_ERROR"`。不做静默 `except: pass`；唯一允许吞异常的地方是 `shutdown()` 中的 `close()`（设备可能已经处于错误态）。
8. **常量集中**：参数默认值只出现在 `_declare_parameters` 一张表里；话题名与 `frame_id` 用模块级常量 `LEFT_TOPIC` / `RIGHT_TOPIC` / `STEREO_TOPIC` / `IMU_TOPIC`，launch 文件与文档引用同一组字符串。
9. **不写测试专用生产代码**：不设 `if testing:` 分支，不加 `debug` 参数；假数据注入只经由既有的 `fake_frames` 参数，且该参数在正常 launch 中不出现（第 6 节）。
10. **导入顺序**：标准库、第三方（numpy / gs130）、ROS，各段之间空一行；不在函数内做不必要的 import（`gs130` 与 ROS 均在顶层，`test/fake_frames.py` 除外）。

---

## 4. ament_python 打包文件内容

### 4.1 `ros/gs130_ros/package.xml`

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>gs130_ros</name>
  <version>0.0.1</version>
  <description>ROS 2 interface for the GS130 stereo camera and IMU.</description>
  <maintainer email="dev@example.com">gs130_sdk maintainers</maintainer>
  <license>MIT</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>tf2_msgs</exec_depend>
  <exec_depend>hobot_codec</exec_depend>
  <exec_depend>websocket</exec_depend>
  <exec_depend>hobot_shm</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

要点：
- `<version>` 与仓库根 `VERSION`（当前 `0.0.1`）保持一致；本期不自动读取，升级时手工同步（避免 build 时读跨目录文件）。
- 依赖分三类：运行时 ROS 依赖（`rclpy` / `sensor_msgs` / `geometry_msgs` / `tf2_msgs`）、被 launch 复用的 TROS 包（`hobot_codec` / `websocket` / `hobot_shm`）、非 ROS 依赖（`numpy`、`gs130` wheel）。
- **`<exec_depend>numpy</exec_depend>` 也要写**（TROS 基础镜像已带 numpy 1.26，声明它是为了在 `rosdep` 之外留下可读记录）；`gs130` 是本地 wheel、不在 rosdep 索引中，**不能**写成 `<depend>gs130</depend>`，否则 `rosdep install` 会失败 —— 其可用性由 `setup.py` 的 `install_requires` 兜底并在文档中说明。
- **不声明** `<depend>hobot_stereonet</depend>`：本期不启用深度网络（非目标，见第 9 节），仅在文档里记录它可挂到 `/gs130/left/image_raw`。

### 4.2 `ros/gs130_ros/setup.py`

```python
"""Build configuration for the gs130_ros package."""

from glob import glob

from setuptools import setup

PACKAGE = "gs130_ros"

setup(
    name=PACKAGE,
    version="0.0.1",
    description="ROS 2 interface for the GS130 stereo camera and IMU",
    license="MIT",
    packages=[PACKAGE],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE]),
        ("share/" + PACKAGE, ["package.xml"]),
        ("share/" + PACKAGE + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "numpy>=1.20", "gs130"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "gs130_camera_node = gs130_ros.camera_node:main",
            "gs130_calibration_node = gs130_ros.calibration_node:main",
            "gs130_fake_frames = gs130_ros.camera_node:main",
        ],
    },
)
```

要点：
- `packages=[PACKAGE]`（不用 `find_packages()`），源目录 `gs130_ros/gs130_ros/` 与包名同名，是 ament_python 的默认布局。
- `data_files` 三行分别负责：ament 索引资源标记、`package.xml` 安装、launch 目录安装到 `share/gs130_ros/launch`（launch 文件必须通过 `glob("launch/*.launch.py")` 收集，否则 `ros2 launch` 找不到）。
- **entry point 名字**：`gs130_camera_node`（主链路）、`gs130_calibration_node`（标定）、`gs130_fake_frames`（指向与主节点相同的 `main`，仅作为主机冒烟时的显式别名，等价于 `gs130_camera_node --ros-args -p fake_frames:=true`；保留它是因为 `ros2 run` 不接受额外位置参数的写法在某些脚本里不直观）。
- `install_requires` 写 `gs130`：wheel 已用 `python/build-wheel.sh` 构建，若开发者直接 `pip install` 本包，pip 会尝试解析 `gs130`；**构建期**若本地无 wheel 索引，必须在 `PYTHONPATH` 或 `pip install python/dist/gs130-*.whl` 之后构建，或临时用 `--no-deps`。这一点写进第 5 节的 Iteration 1 命令里，避免第一天就卡住。
- `zip_safe=True` 是 ament_python 的惯例，`glob` 收集的 `data_files` 不受影响。

### 4.3 `ros/gs130_ros/setup.cfg`

```ini
[develop]
script_dir=$base/lib/gs130_ros
[install]
install_scripts=$base/lib/gs130_ros
```

作用：colcon 环境下把 `console_scripts` 落到 `lib/gs130_ros/`，使 `ros2 run gs130_ros gs130_camera_node` 能找到可执行文件；缺少它时 `ros2 run` 会报 “No executable found”。

### 4.4 `ros/gs130_ros/resource/gs130_ros`

空文件（0 字节）。它是 ament 索引里的包标记：`data_files` 把它装到 `share/ament_index/resource_index/packages/gs130_ros`，`ros2 pkg` / `ros2 launch` 依靠它发现包。**不要**写入任何内容。

### 4.5 `ros/gs130_ros/__init__.py`

```python
"""ROS 2 interface for the GS130 stereo camera and IMU."""

__version__ = "0.0.1"
```

### 4.6 `ros/gs130_ros/LICENSE`

直接复制仓库根 `LICENSE`（MIT），不改文本；`package.xml` 的 `<license>MIT</license>` 必须与实际文件一致。

---

## 5. 迭代计划

共 4 个迭代，每个都可独立验收。第 1 个迭代是“硬件上能看见画面的最小闭环”。

### Iteration 1 —— 最小可见结果（硬件）

- **目标**：web UI 上出现相机画面。默认参数为 `mode=resize, 640x480, fps=30`（`resize` 走 ISP+VSE，是 1088x1280 双 MIPI 在 30 fps 下最稳的路径，RAW 直出要求输出尺寸严格等于 1088x1280，不适合作为默认展示分辨率）。
- **文件**：`package.xml`、`setup.py`、`setup.cfg`、`resource/gs130_ros`、`LICENSE`、`gs130_ros/__init__.py`、`launch/camera.launch.py`、`gs130_ros/camera_node.py`（仅 `main` / `build_device` / `__init__` / `_declare_parameters` / `publish_images` / `image_message` / `shutdown`）。
- **验收（有板）**：
  1. `colcon build --packages-select gs130_ros && source install/setup.bash`
  2. `ros2 launch gs130_ros camera.launch.py` 无异常退出。
  3. 浏览器打开 `http://<board>:8000`，**能看到左目画面**（棋盘格/场景，非绿屏、非错行）。
  4. `ros2 topic hz /gs130/left/image_raw` 接近 30 Hz（±10%）。
  5. 另开终端 `ros2 topic hz /gs130/left/image_jpeg` 有数据，说明 `hobot_codec` 的 NV12→JPEG 链路成立。
- **状态**：本迭代跑通即证明“SDK → ROS 话题 → 复用节点 → web UI”整条链成立，之后的迭代只加内容不改链路。

### Iteration 2 —— IMU 与参数化

- **目标**：IMU 上话题；分辨率/模式/帧率/ODR 可由 launch 参数覆盖；右目与拼接帧按需发布。
- **文件**：`camera_node.py`（加 `publish_imu`、`imu_message`、`stamp`、`imu_timer`、`stereo` 发布器）、`launch/camera.launch.py`（暴露 `mode/width/height/fps/odr/stereo_layout` 参数；第三个 `hobot_codec` + 第二个 websocket channel 以注释形式给出）。
- **验收**：`ros2 topic echo /gs130/imu/data --once` 打印 `linear_acceleration.z ≈ 9.8`（静止、Z 轴朝上时）且 `orientation_covariance[0] == -1`；`ros2 topic hz /gs130/imu/data` 接近 `odr`（默认 200 Hz，±10%）；用 `ros2 launch gs130_ros camera.launch.py fps:=15` 覆盖参数后，`ros2 topic hz /gs130/left/image_raw` 变为 ≈15 Hz，且启动摘要日志里的帧率与之一致（本期不做运行时重配置，参数只在启动时读取）。

### Iteration 3 —— 标定展示（CameraInfo + TF）

- **目标**：左右目 `CameraInfo` 与相机-相机、相机-IMU 的静态 TF 上话题。
- **文件**：新增 `gs130_ros/calibration.py`、`gs130_ros/calibration_node.py`、`launch/calibration.launch.py`；`package.xml` 已在 Iteration 1 声明 `geometry_msgs` / `tf2_msgs`，无需改动。
- **验收**：
  1. `ros2 launch gs130_ros calibration.launch.py` 后 `ros2 topic echo /gs130/left/camera_info --once` 的 `k[0]`（fx）与 `python/test/test_gs130.py` 打印的 `left.K[0][0]` **完全一致**。
  2. `distortion_model` 为 `equidistant`（`DistModel.FISHEYE`）或 `plumb_bob`（`DistModel.PINHOLE`），并且 `d` 长度与该模型匹配（equidistant 4 项，plumb_bob 5 项）。
  3. `ros2 run tf2_ros tf2_echo camera_left_optical_frame camera_right_optical_frame` 输出的平移模长等于基线（`|T|`，单位米），旋转可读；`tf2_echo camera_left_optical_frame imu_link` 有输出。
  4. 相机被占用时该节点给出点名 `gs130_init()` 的报错并**不**崩溃（说明两个进程的失败域是分开的）。

### Iteration 4 —— 主机侧验证与收尾

- **目标**：无板也能跑语法/导入/消息构造检查；Ctrl-C 释放干净；日志与参数定稿。
- **文件**：新增 `test/fake_frames.py`；微调 `camera_node.py`（`_frames` 的 fake 分支、`shutdown` 幂等、启动摘要日志）；`launch/camera.launch.py`（可选 `fake_frames` 参数，默认 `false`）。
- **验收（主机，无相机）**：
  1. `python3 -m compileall -q ros/gs130_ros ros/test` 通过。
  2. `PYTHONPATH=ros/gs130_ros python3 -c "import gs130_ros.camera_node"` 在装了 wheel + rclpy 的主机上无报错。
  3. `python3 ros/test/fake_frames.py` 打印两帧尺寸与 IMU 字段的断言结果后退出 0（第 6 节）。
- **验收（有板）**：
  1. `ros2 launch gs130_ros camera.launch.py fake_frames:=true` 时 web UI 显示合成棋盘格（左亮右暗），证明发布/编码/显示链路与硬件无关。
  2. `ros2 launch gs130_ros camera.launch.py` 后 Ctrl-C：进程在 2 秒内退出，日志只有一行 “gs130 camera stopped”。
  3. Ctrl-C 后立刻 `python3 python/test/test_gs130.py RDKX5 GS130WI resize 640 480 30 200` 能成功 `gs130_init` —— 证明摄像头已释放（第 7 节 R1）。

### 主机可验证 vs 需要板

| 可在主机验证 | 必须在板上验证 |
| --- | --- |
| `compileall` 语法检查；模块导入（wheel 已装） | `libgs130` 能否找到、`gs130_init/start` 是否成功 |
| 假帧路径下的消息构造：`width/height/step/encoding`、`data` 长度 = `w*h*3/2` | 真实 NV12 步长、`hobot_codec` 能否解码（绿屏/错行只能上板看） |
| `calibration.py` 纯函数的数值映射（用 `_types.py` 里同类数据的构造样本） | EEPROM 实际内容、`CameraInfo` 的 `d` 长度与真实畸变模型 |
| 参数校验与错误信息文本（`ValueError` 分支） | 帧率/ODR 实测、Ctrl-C 后设备释放 |
| launch 文件语法（`ros2 launch --show-args` 需要 ROS 安装，主机装了即可） | web UI 是否真的出图、端口 8000 是否可达 |

---

## 6. 主机侧开发策略（无板可做的一切）

**前置**：`pip install python/dist/gs130-*.whl`（wheel 由 `python/build-wheel.sh` 产出，本期**不修改** `python/` 下任何文件）即可让 `import gs130` 成功 —— 只是 `gs130.Device()` 会因找不到 `libgs130.so` 抛 `OSError`。这正好把“导入检查”和“设备使用”分成两件事。

1. **语法与导入检查**（不需要 libgs130，也不需要拍照）：

   ```bash
   python3 -m compileall -q ros/gs130_ros ros/test
   PYTHONPATH=ros/gs130_ros python3 -c "import gs130_ros.camera_node, gs130_ros.calibration, gs130_ros.calibration_node"
   ```

   成立的前提是第 3 节规则 10 与 2.1 的约定：模块顶层只 import、不建设备、不做 I/O。一旦有人在顶层写 `Device(...)`，这条命令会立刻以 `OSError: libgs130 not found` 失败 —— 这本身就是有价值的守卫。

2. **消息构造冒烟**（注入假帧，不加测试专用生产代码）：

   `test/fake_frames.py` 做三件事，全部通过**覆盖实例方法**实现，不引入生产代码分支：

   ```python
   rclpy.init(args=["gs130_fake_frames"])                 # 覆盖 argv，避免吞掉 pytest/脚本参数
   node = camera_node.CameraNode.__new__(camera_node.CameraNode)   # 不跑 __init__，不碰硬件
   rclpy.node.Node.__init__(node, "gs130_fake_frames")   # 只初始化 ROS 侧
   camera_node.CameraNode._frames = lambda self: camera_node.create_fake_pair(640, 480)
   message = node.image_message(camera_node.create_fake_pair(640, 480)["left"], "camera_left_optical_frame")
   assert message.encoding == "nv12"
   assert (message.width, message.height, message.step) == (640, 480, 640)
   assert len(message.data) == 640 * 480 * 3 // 2
   ```

   同时用 `create_fake_imu()` 走一遍 `imu_message()` 并断言 `orientation_covariance[0] == -1` 与 `linear_acceleration.z`。脚本以退出码表达成功/失败，可直接被 CI 或人工调用。

   **边界**：`create_fake_pair()` 与 `create_fake_imu()` 是本节的唯一支撑代码，它们住在生产模块 `camera_node.py` 里，同时服务 `fake_frames:=true` 的主机演示（Iteration 4 验收项）—— 一个用途是测试、另一个用途是演示，且实现只有 ~15 行，故不违反“不为测试写生产代码”。除此之外**不**新增任何 hook、不注入 `mock` 依赖、不改函数签名。

3. **`calibration.py` 的纯函数断言**：手工构造一个 `gs130.CameraIntrinsics` / `Calibration` 命名元组式样本（`dataclass` 可直接实例化，不需要设备），断言 `to_camera_info()` 的 `k/p/d/distortion_model` 与输入一致。这条不需要 ROS 也不需要相机。

4. **不做什么**：不为本包写完整 pytest 套件、不做 launch 文件的单元测试、不引入 `launch_testing`（第 9 节非目标）。

---

## 7. 风险与缓解

| # | 风险 | 影响 | 缓解措施 | 验证方式 |
| --- | --- | --- | --- | --- |
| R1 | **相机独占**：上一次崩溃的进程仍持有 `/dev/video*` / VIN 资源，新进程 `gs130_init` 失败且原因不明 | 节点起不来，且误判为代码 bug | (a) `build_device()` 捕获 `GS130Error` 与 `OSError`，日志明确写出“camera may be held by another process; check `ps`/`fuser`”，并附上失败调用名；(b) `main()` 的 `finally` 保证任何退出路径都 `stop()+close()`；(c) `shutdown()` 幂等，重复调用安全；(d) 文档给出手工排查命令：`ps -ef \| grep gs130`、`fuser -v /dev/video*`。**不在代码里自动杀进程**（危险且越界） | 板上故意让一个 gs130 进程挂着，再启动节点，确认报错文本正确且进程干净退出；Ctrl-C 后立即可被 `test_gs130.py` 重新打开（Iteration 4 验收） |
| R2 | **NV12 与 `hobot_codec` 不匹配**：`height` 填成 `shape[0]`（= h*3/2）、`step` 填错或 `encoding` 写成 `yuv420`/`nv21`，导致画面错行、绿边、codec 报错 | web UI 看不到正确画面，且极易被误判为驱动问题 | 尺寸换算只在 `image_message()` 一处，写成带注释的三行（2.4 节）；`encoding` 恒为 `"nv12"`；`step = width`（UV 平面紧随 Y 平面，行字节数仍是 width）；Iteration 1 验收即人工看图确认；主机侧断言 `len(data) == w*h*3//2` | 上板看图 + `ros2 topic echo /gs130/left/image_raw --field height`（应为 480 而非 720） |
| R3 | **IMU 速率 vs 图像速率**：IMU 200 Hz 远高于图像 30 Hz，若沿用“每帧取一包”会持续积压并让 `ros2 topic hz` 远低于预期 | IMU 延迟增长、数据丢失 | IMU 用独立定时器（`1/odr`），每次回调**取空队列**（上限 `imu_burst=32`），发布器 QoS 深度设为 200；`odr<=0` 或 `Device.imu_name is None` 时不启动 IMU 定时器并打一行 info | `ros2 topic hz /gs130/imu/data` 与 `odr` 相差 ±10%；连续 60 秒后队列不增长（用 `ros2 topic delay` 或时间戳差观察） |
| R4 | **时间戳单位/时基**：`timestamp_ns` 是纳秒，但如果退化到 `hbn_frame_info_t.timestamps`（内核单调时基）就不是 Unix 纪元，直接塞进 `header.stamp` 会让 `ros2 bag`、`tf2` 的时间比较错乱 | 话题时间戳不可用、TF 查询失败 | `stamp()` 统一转换：`timestamp_ns <= 0` 或 `< 1.577e18`（2020-01-01 的纳秒值，明显早于任何合理纪元时间）时改用节点时钟并**只告警一次**（`warn`，含原始值）；正常路径 `rclpy.time.Time(nanoseconds=ts)`；**绝不**手工做秒/纳秒除法，避免 `float` 精度丢失 | `ros2 topic echo /gs130/left/image_raw --field header.stamp` 与 `date +%s` 对比，偏差应在秒级；主机侧单测 `stamp()` 的两个分支（真值 / 退化值） |
| R5 | **colcon 工作空间与 `PYTHONPATH`（gs130 wheel）**：`gs130` 是本地 wheel，不是 ROS 包；`source install/setup.bash` 后 `import gs130` 可能失败，或在 `PYTHONPATH` 混乱时导入到旧版本 | 节点启动即 `ImportError`，或加载到不匹配的绑定与 `libgs130` | (a) 顺序固定：先 `pip install python/dist/gs130-*.whl`（或 `PYTHONPATH=python`），再 `colcon build`，再 `source install/setup.bash`；(b) `setup.py` 的 `install_requires` 写 `gs130`，让缺失时在构建期就暴露（本地索引缺失时用 `--no-deps` 并在文档说明）；(c) 启动时把 `gs130.__version__` 与 `gs130.library_version()` 打进那一行配置摘要日志，版本不匹配一眼可见（`_runtime.load()` 已有版本校验，会 `warn` 或 `raise`）；(d) 工作空间建在 `~/gs130_ws`（不放进仓库），避免 `src/` 与仓库内 `python/build` 混淆 | 板上 `python3 -c "import gs130; print(gs130.__version__, gs130.library_version())"` 与节点启动日志一致 |
| R6 | **SIGINT 清理**：Ctrl-C 时若设备未 `stop()/close()`，摄像头保持被占用，下一次启动失败（与 R1 复合） | 每轮调试都要重启机器/复位 | `main()` 用 `try/except KeyboardInterrupt/finally`：`finally` 中调用 `shutdown()`（`destroy_timer` → `Device.stop()` → `Device.close()` → `destroy_node()` → `rclpy.shutdown()`）；`Device` 自身的 `__exit__`/`__del__` 是第二道保险；不注册额外 `signal` 处理器（让 rclpy 处理信号，避免双份清理逻辑） | Ctrl-C 后 2 秒内进程消失（`pgrep -f gs130_camera_node` 为空）；紧接着跑 `test_gs130.py` 能 `gs130_init` 成功 |
| R7 | **内存与带宽**：双目 NV12 640x480 每帧 460 KB，30 fps 双路 ≈ 28 MB/s；节点若持有帧引用不释放，`malloc` 缓冲会迅速耗尽 | 数分钟内 OOM 或帧率崩塌 | 不持有跨回调引用（2.4 节），`bytes(image)` 之后局部引用即失效；不缓存、不复用消息对象（rclpy 发布后由中间件接管序列化）；主机侧可用 `fake_frames:=true` 连续跑 10 分钟看 `RSS` 是否平稳（该项可在有 ROS 的无板主机上做） | 板上跑 10 分钟，`ps -o rss` 平稳；`ros2 topic hz` 不衰减 |
| R8 | **参数错配导致的静默错图**：用户把 `width/height` 改成与 `mode` 不匹配的值（尤其 raw 模式的 1088x1280 约束） | 启动失败或画面异常但原因不明 | `default_config()` 做硬校验并在错误信息里写出期望值与实际值；不用“猜一个能跑的尺寸”来自动纠正 | 主机侧对越界参数断言 `ValueError` 文本 |
| R9 | **复用节点的参数/话题拼写**：`hobot_codec` 的 `in_format/out_format`、`websocket` 的 `image_topic/channel/port` 写错则链路静默断 | 节点全部在跑但 UI 黑屏，排查成本高 | launch 文件中这几个参数集中在文件顶部（注释标注来源与含义），只保留一条正确链路；Iteration 1 验收包含“`ros2 topic hz /gs130/left/image_jpeg` 有数据”这一步，把“话题断”和“UI 渲染问题”区分开；`hobot_stereonet` 本期不接入，避免多一条可能出错的边 | 逐步 `ros2 topic hz`：`image_raw` → `image_jpeg` → 看 UI |

---

## 8. Definition of Done（评审者执行的命令）

以下命令构成验收清单。`<board>` 为板端地址，`$WS` 为工作空间（建议 `~/gs130_ws`）。

**A. 主机 / 构建**（不需要相机）

1. `python3 -m compileall -q ros/gs130_ros ros/test` —— 退出码 0。
2. `PYTHONPATH=ros/gs130_ros python3 -c "import gs130_ros.camera_node, gs130_ros.calibration, gs130_ros.calibration_node"` —— 退出码 0（wheel 已安装）。
3. `python3 ros/test/fake_frames.py` —— 打印 NV12 尺寸与 IMU 断言，退出码 0。
4. `cd $WS && colcon build --packages-select gs130_ros` —— 构建成功，无 warning 级以上的 ament 报错。
5. `source $WS/install/setup.bash && ros2 pkg executables gs130_ros` —— 列出 `gs130_camera_node`、`gs130_calibration_node`、`gs130_fake_frames` 三个可执行文件（证明 `setup.cfg` + entry point + resource 标记都装对了）。
6. `ros2 launch gs130_ros camera.launch.py --show-args` —— 打印参数列表且退出码 0（launch 文件语法正确）。

**B. 板端 —— 图像链路**

7. `ros2 launch gs130_ros camera.launch.py` —— 启动无异常。
8. `ros2 topic hz /gs130/left/image_raw` —— ≈30 Hz（±10%）。
9. `ros2 topic echo /gs130/left/image_raw --field height` —— 输出 `480`（默认参数）。
10. `ros2 topic hz /gs130/left/image_jpeg` —— 有稳定帧率（证明 `hobot_codec` 链路）。
11. 浏览器 `http://<board>:8000` —— **看到画面**（本包的核心验收项）。

**C. 板端 —— IMU 与标定**

12. `ros2 topic hz /gs130/imu/data` —— ≈200 Hz（±10%）。
13. `ros2 topic echo /gs130/imu/data --once` —— `linear_acceleration.z` ≈ 9.8（静止），`orientation_covariance[0] == -1`。
14. `ros2 launch gs130_ros calibration.launch.py` —— 启动成功。
15. `ros2 topic echo /gs130/left/camera_info --once` —— `k[0]` 与 `test_gs130.py` 打印的 `left.K[0][0]` 一致，`distortion_model` 为 `equidistant` 或 `plumb_bob`，`len(d)` 与模型匹配。
16. `ros2 run tf2_ros tf2_echo camera_left_optical_frame camera_right_optical_frame` —— 平移模长等于基线（米）。
17. `ros2 run tf2_ros tf2_echo camera_left_optical_frame imu_link` —— 有稳定输出。

**D. 板端 —— 健壮性与清理**

18. Ctrl-C 后 `pgrep -f gs130_camera_node` 为空，且进程 2 秒内退出。
19. 紧接着 `python3 python/test/test_gs130.py RDKX5 GS130WI resize 640 480 30 200` —— 能成功 `gs130_init`（证明相机已释放）。
20. `ros2 launch gs130_ros camera.launch.py fake_frames:=true` —— web UI 显示合成棋盘格，无需相机（证明发布链路与硬件解耦）。
21. 连续运行 10 分钟：`ros2 topic hz` 不衰减，`ps -o rss -p $(pgrep -f gs130_camera_node)` 平稳。

**E. 代码审阅**

22. `wc -l ros/gs130_ros/*.py` 总量 ≤ 330 行；文件数与第 1 节清单完全一致（无新增文件、无 `msg/`、无 `test/` 下的额外模块）。
23. 常规运行日志中**每帧无输出**（用 `ros2 launch ... 2>&1 | grep -c "image"` ≈ 0）。
24. 未修改 `python/` 下任何文件（`git status --short python/` 为空）。
25. 所有错误信息都点名了失败的调用（抽查 `GS130Error` 传播路径与 `ValueError` 分支）。

---

## 9. 明确的非目标（防止过度开发）

以下内容在本期明确**不做**，任何迭代中都不应被偷偷加入：

1. **不写 C++ 节点**、不做 `rclcpp` 版本、不做编译型 `gs130_ros` 混合包（`ament_cmake` + `ament_python` 混合构建）。
2. **不定义自定义消息/服务/动作**（无 `msg/`、`srv/`、`action/`，无独立 interface 包）。“展示全部标定”用三个话题表达：左右 `CameraInfo` + `CameraInfo.p` + 静态 TF，不新增 `Calibration` 消息。
3. **不接入 `hobot_stereonet` 深度网络**、不做点云、不做视差、不做 RGB-D 发布。
4. **不做相机内/外参标定流程**（不跑棋盘格标定、不写 EEPROM、不调用 `convert_calibration()` 回写）。标定只**读**和**展示**。
5. **不做录制/回放节点**（不封装 `ros2 bag`）、不做视频文件输出、不做图像压缩参数调优。
6. **不做像素格式转换**：不把 NV12 转 BGR/RGB 后再发布（`hobot_codec` 已在链路上做这件事），不发布 `compressed` 自定义格式。
7. **不做自我拼接**：左右拼接帧只走 SDK 的 `stereo_layout`（FPGA/相机线程内完成），ROS 侧不 `hconcat`。
8. **不做多相机/多实例支持**：GS130 是独占设备，不支持节点内管理两个设备，也不做设备选择参数。
9. **不做运行时重配置**：参数只在启动时读取；不实现 `ros2 param set` 生效的动态重配（不写 `on_set_parameters` 回调）。
10. **不做硬件同步研究**：不调整 FSYNC、不做时间戳对齐/插值、不做 IMU-图像时间同步算法。
11. **不做生命周期节点（lifecycle node）**、不做 component/`rclcpp_components` 容器化、不写自定义 executor 策略。
12. **不做性能优化**：不引入零拷贝共享内存发布（`hobot_shm` 只用于既有节点的 env）、不做帧池、不做内存复用；`bytes(image)` 的整帧拷贝是本期的既定取舍。
13. **不做单元测试套件/CI**：本期只有第 6 节的主机冒烟脚本；不引入 `launch_testing`、pytest 插件、GitHub Actions。
14. **不做 Python API 变更**：`python/` 与 `core/` 一行都不改；若发现 SDK 缺陷，另开任务，不在本包内绕过。
