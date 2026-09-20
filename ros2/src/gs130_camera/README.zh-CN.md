# GS130 Camera SDK —— ROS 接口

**简体中文** | [English](README.md)

> 把 GS130 双目相机与板载 IMU 接入 ROS 2 的轻量驱动，直接调用 GS130 SDK 的 C API。

---

## 📖 简介

`gs130_camera` 将 GS130 双目相机及其 IMU 发布为**标准 ROS 2 消息**。节点直接调用 GS130 SDK 的 C API（链接 `libgs130.so`），本身不做图像处理、立体匹配或滤波；`rect` 模式下的去畸变与极线校正由 SDK 在 GDC 硬件上完成。

| 项目 | 说明 |
| --- | --- |
| 语言 | C++17（节点）+ C99（SDK 预设桥接） |
| 构建 | `ament_cmake` / `colcon` |
| 支持平台 | RDK X5（RDK S100 / RDK S600 开发中） |
| 依赖 | ROS 2 Humble 或 Jazzy、`libgs130.so` |

## 🧩 依赖

- **硬件**：RDK 系列开发板 + GS130 系列双目相机（部分型号带 IMU）。当前支持 **RDK X5**，RDK S100 / RDK S600 支持开发中。
- **ROS 2**：Humble 或 Jazzy。请 source 板卡上实际安装的发行版；使用 TROS 时，在其后继续 source TROS overlay：

  ```bash
  source /opt/ros/humble/setup.bash     # 或：source /opt/ros/jazzy/setup.bash
  source /opt/tros/humble/setup.bash    # 仅 TROS 环境，与上面的发行版对应
  ```

- **GS130 SDK**：`gs130-camera` 软件包，提供 `libgs130.so`、`libgs130.a`、`gs130.h` 与 `gs130_define.h`，需要单独安装。
- **可选依赖**：TROS 的 `hobot_codec`、`websocket`（网页预览）与 `hobot_stereonet`（双目深度）。

## 🔨 构建

**1. 先安装 SDK**

```bash
# 检查是否已安装
dpkg -s gs130-camera

# 若未安装，安装由 core/ 构建出的 deb（在 SDK 源码目录执行 make deb）
sudo dpkg -i gs130-camera_<version>+<platform>_<arch>.deb
```

**2. 再编译 ROS 2 包**

```bash
cd ros2
source /opt/ros/humble/setup.bash     # 或：source /opt/ros/jazzy/setup.bash
colcon build --packages-select gs130_camera
source install/setup.bash
```

在 SDK 源码目录旁构建（未安装到系统）时，CMake 会自动使用 `core/include` 的头文件，并按名字查找 `libgs130`。若目录结构不同，可显式指定：

```bash
colcon build --packages-select gs130_camera --cmake-args \
  -DGS130_LIBRARY=/path/to/libgs130.so \
  -DGS130_INCLUDE_DIR=/path/to/include
```

## 🚀 运行节点

```bash
# 直接运行节点
ros2 run gs130_camera gs130_node --ros-args -p camera_mode:=rect -p stitch:=top_bottom

# 或使用基础 launch（推荐）
ros2 launch gs130_camera gs130.launch.py
```

可执行文件为 `gs130_node`，节点名为 `gs130_camera`。参数非法或 SDK 初始化失败时，节点会打印错误并退出。

## 📡 话题

`stitch` 不为 `none` 时，双目合并为一帧：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `image_combine` | `sensor_msgs/Image` | 合并帧，`nv12` |
| `image_combine/left/camera_info` | `sensor_msgs/CameraInfo` | 左目内参，尺寸为单目尺寸 |
| `image_combine/right/camera_info` | `sensor_msgs/CameraInfo` | 右目内参，`P[3]` 携带基线 |
| `image_combine/gray` | `sensor_msgs/Image` | 可选，`mono8` |

`stitch: none` 时，每目独立成帧：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `image_left` / `image_right` | `sensor_msgs/Image` | 左 / 右目，`nv12` |
| `image_left/camera_info` / `image_right/camera_info` | `sensor_msgs/CameraInfo` | 左 / 右目内参 |
| `image_left/gray` / `image_right/gray` | `sensor_msgs/Image` | 可选，`mono8` |

以上图像话题名均可通过参数修改；`camera_info` 话题名由对应图像话题派生，不作为参数。

两种情况都会发布：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/imu_data` | `sensor_msgs/Imu` | 角速度与线加速度；无 IMU 的设备不发布 |
| `/tf_static` | `tf2_msgs/TFMessage` | `camera_link` → `camera_right_link`，有 IMU 时追加 `imu_link` |

`/tf_static` 是 ROS 保留话题，需要改名时请使用 remap。

## ⚙️ 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `device` | `GS130WI` | 相机型号：`GS130WI`、`GS130W` 或 `GS130W_NO_EEPROM` |
| `camera_mode` | `rect` | `raw` / `resize` / `rect` |
| `stitch` | `none` | `none` / `left_right` / `right_left` / `top_bottom` / `bottom_top` |
| `output_width` | `544` | 单目输出宽度（像素） |
| `output_height` | `448` | 单目输出高度（像素） |
| `fps` | `30` | 相机帧率 |
| `odr` | `200` | IMU 输出数据率（Hz） |
| `publish_gray` | `false` | 额外发布 `mono8` 灰度图 |
| `image_topic` | `image_combine` | 合并帧话题（`stitch` 不为 `none` 时生效） |
| `left_image_topic` | `image_left` | 左目话题（`stitch` 为 `none` 时生效） |
| `right_image_topic` | `image_right` | 右目话题（`stitch` 为 `none` 时生效） |
| `imu_topic` | `/imu_data` | IMU 话题 |
| `timer_period_ms` | `1` | 发布定时器周期（ms） |
| `frame_id` | `camera_link` | 左目与合并帧的坐标系 |
| `right_frame_id` | `camera_right_link` | 右目坐标系 |
| `imu_frame_id` | `imu_link` | IMU 坐标系 |

`output_width`、`output_height`、`fps`、`odr` 必须为正整数，`timer_period_ms` 必须为正，`camera_mode` 与 `stitch` 必须为上述取值之一，否则节点启动即报错。

## ✨ 特性

- **硬件级校正**：`rect` 模式的去畸变与校正由 SDK 的 GDC 硬件完成，节点不做图像处理。
- **双目布局可选**：不拼接与 4 种拼接方式共 5 种布局，按需选择。
- **标定输出**：EEPROM 存在时发布双目 `CameraInfo`（右目 `P[3]` 携带基线）与静态 TF；RAW 模式在无标定时也能出图，但不发布 `CameraInfo` 与 TF，RESIZE / RECT 模式则需要标定。
- **IMU 可选**：设备带 IMU 时发布角速度与线加速度；`orientation` 按 `sensor_msgs/Imu` 约定标记为不可用（协方差首元素为 `-1`）。
- **有序发布**：单个定时器每周期只发一条消息，图像与 IMU 按 SDK 时间戳交替发出，保证采样顺序并避免下游浅队列丢帧。
- **时间戳**：消息时间戳为**发布时刻**；SDK 硬件时间戳仅用于节点内部排序（RDK 板卡的相机时钟自开机计时，不适合直接对外使用）。
- **QoS**：图像队列深度 5，IMU 队列深度 10，与 `hobot_mipi_cam` 的用法保持一致。

暂不支持：零拷贝（共享内存 `hbm_img_msgs`）、自定义消息、IMU 滤波、动态参数与多设备同步。

## 🖥️ Launch 启动方式

### 1. `gs130.launch.py` — 基础启动与参数映射

```bash
ros2 launch gs130_camera gs130.launch.py
ros2 launch gs130_camera gs130.launch.py stitch:=top_bottom publish_gray:=true
ros2 launch gs130_camera gs130.launch.py --show-args      # 查看全部参数与默认值
```

映射规则：**launch 参数与节点参数同名一一对应**。launch 文件同时声明了每个参数的类型（整数 / 布尔），因此命令行传入的字符串会先转换为节点期望的类型再下发；默认值与上表一致。

### 2. `gs130_websocket.launch.py` — 网页预览

```bash
ros2 launch gs130_camera gs130_websocket.launch.py
```

包含基础 launch，并额外启动 `hobot_codec`（`nv12` → `jpeg`）与 `websocket` 推流节点（同时拉起 nginx）。浏览器打开查看：

```
http://<板卡IP>:8000
```

- `stitch` 不为 `none`：合并帧使用通道 0。
- `stitch: none`：左目使用通道 0，右目使用通道 1。

影响预览的参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `stitch` | `none` | `none` 时每目一个通道；其他取值预览合并帧 |
| `image_topic` | `image_combine` | `stitch` 不为 `none` 时预览的话题 |
| `left_image_topic` | `image_left` | `stitch` 为 `none` 时预览的左目话题 |
| `right_image_topic` | `image_right` | `stitch` 为 `none` 时预览的右目话题 |
| `fps` | `30` | 传给编码器的帧率 |

它自身不声明参数：上表中的全部相机参数都会原样透传给节点。

### 3. `gs130_stereonet.launch.py` — 双目深度

```bash
ros2 launch gs130_camera gs130_stereonet.launch.py
```

包含基础 launch（`stitch` 固定为 `top_bottom`，因为 `hobot_stereonet` 要求左目位于上半帧），并拉起 `hobot_stereonet` 的深度模型与可视化节点，深度伪彩图同样在网页上查看：

```
http://<板卡IP>:8000
```

需要已安装 `hobot_stereonet`（TROS）。它自身的参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `stereonet_model` | `DStereoV2.4_int8_544_448.bin` | `hobot_stereonet/config` 下的模型文件，尺寸对应**单目** |
| `render_type` | `indoor` | 深度着色：`indoor`、`outdoor`、`indoor-reverse`、`outdoor-reverse`、`distance`、`distance-reverse` |

模型输入为**单目** `544×448`，与默认的 `output_width` / `output_height` 对应，两者需同步修改。相机参数同样会被透传，其中 `image_topic`、`frame_id`、`right_frame_id` 会传给 `hobot_stereonet`。

## 📄 许可证

本项目采用 [MIT 许可证](../../../LICENSE)。
