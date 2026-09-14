# GS130 ROS 2 (TROS) 接口冻结书 v0.1.0

> ## ⚠️ 修订通知（评审后生效，优先级高于本文其余内容）
>
> 本文档在**板端实测之前**写成，其中三处规定与实测结果冲突，已由
> `00_verified_platform_facts.md`（实测权威）推翻，**以下述规定为准**：
>
> | 项 | 本文原规定（已作废） | 实测与最终规定 | 依据 |
> |---|---|---|---|
> | 图像 QoS | `BEST_EFFORT` | **`RELIABLE`**：`hobot_codec` 订阅端为 RELIABLE，BEST_EFFORT 会被静默丢弃，页面黑屏且无报错 | E1 |
> | `Image.height` | `H*3//2`（NV12 打包高） | **真实图像高 `H`**（`shape[0]*2//3`）。用打包高会让 codec 记录 `init_pic_h_: 1080, alined_pic_h_: 1088` 后**段错误（-11）** | E2 |
> | 拼接实现 | 驱动侧手工 `concatenate` 左右目 | **使用 SDK 硬件拼接**（`Config.preset()` 后设 `stereo_layout`），不写任何拼接代码 | E6 |
>
> 另有两项补充规定：
> - `Image.data` 必须用 `array.array("B")` + `frombytes` 构造；直接赋 `bytes` 会触发 rclpy 逐元素校验，
>   1.38 MB 一帧耗时 **1103.87 ms**（对比 1.36 ms），实测约 800 倍差距（E3）。
> - `websocket.launch.py` 用 `os.system` 启动的 nginx 在 launch 退出后**仍存活**并占用 8000 端口，
>   关停流程必须显式清理（E4）。
>
> 实际交付的实现见 `ros/gs130_ros/`，用户文档见 `ros/README.md`，实测证据见 `00_verified_platform_facts.md`。
> 本文档其余部分（话题名、消息类型、参数名、launch 名、退出码）与实现一致，继续有效。

- 文档编号：11
- 状态：**FROZEN（冻结）**，版本 v0.1.0
- 适用平台：RDK X5 + TROS Humble（`/opt/tros/humble`）
- 上游依赖：`gs130_sdk` 仓库版本 `0.0.1`（`VERSION` 文件）之 Python 绑定 `python/gs130/`
- 本文档是**外部接口契约**：实现、测试、文档三方均以本文档为唯一依据，不得各自解释。
- 本文档只描述**对外可观察**的接口：包名、节点名、话题、消息字段、时间戳、参数、launch、错误行为、兼容性。

本文档中的每一处「名字 / 默认值 / 单位」只写一次、写死。若实现与本文档冲突，以实现 bug 处理，不改契约。

---

## 0. 范围与非目标（Scope）

### 0.1 v0.1.0 范围

1. 把 GS130 双目相机的 NV12 图像流发布到 ROS 2 话题，**默认链路与 TROS 既有链完全一致**，从而**不修改任何 D-Robotics 既有节点**：
   `gs130_ros`（发布 `/image_combine_raw`，NV12）→ `hobot_codec_republish`（转 `/image_combine_jpeg`）→ `websocket`（nginx:8000 显示）。
2. 在同一节点内发布 IMU 数据（`sensor_msgs/Imu`）。
3. 在启动时发布静态标定（`sensor_msgs/CameraInfo`）。
4. 提供一个带 web 链路的 launch 文件和一个仅相机的 launch 文件。

### 0.2 非目标（明确不做，且不得在未来版本"顺手加上"而不走评审）

| 非目标 | 理由 |
|---|---|
| 不做图像编码 / 转码 / 缩放 / 色彩空间转换 | 交给 `hobot_codec_republish`，重复实现会破坏零拷贝链路 |
| 不做深度计算、不做 `stereonet` 桥接 | 直接使用既有 `hobot_stereonet`，它有自己的输入约定 |
| 不做 `sensor_msgs/Imu` 的姿态解算（AHRS / 互补滤波 / 卡尔曼） | SDK 只给原始 accel/gyro，姿态解算属于下游融合算法，不属于驱动 |
| 不发布温度话题 | SDK 的 `temp`（摄氏度）在 ROS 标准消息中**没有对应字段**；自定义消息会引入非标准依赖。见 §3.3 |
| 不发布 `/tf` 与 `/tf_static` | SDK 的外参语义（参考系在 RECT 模式下会变为"虚拟双目平行系"，见 `gs130.h` 中 `gs130_calibration_t` 的外参说明）需要单独的评审；v0.1.0 不引入半正确的 TF。`tf2_msgs` 不进入依赖表。 |
| 不发布 `PointCloud2` / `disparity` / `image_rect` / `image_color` | 需要额外计算或依赖 `image_proc`，超出"发布 SDK 能给的原始数据"范围 |
| 不做压缩图像话题（`image_transport` 插件） | 压缩由 `hobot_codec_republish` 完成 |
| 不做参数动态重配置（`rcl_interfaces` 动态回调） | 相机参数在 SDK 中由 `gs130_init()` 定死，运行时无法修改；见 §5.3 |
| 不支持多相机 / 多实例 | GS130 是**独占设备**，同一时刻只允许一个进程持有；见 §8 |

---

## 1. 包身份与依赖

### 1.1 身份

| 项 | 值 |
|---|---|
| ROS 包名 | `gs130_ros` |
| 节点名（唯一节点） | `gs130_node` |
| 可执行文件名 | `gs130_node` |
| 包版本 | `0.1.0` |
| 源码布局 | `ros/gs130_ros/`（`package.xml`、`setup.py`、`setup.cfg`、`resource/gs130_ros`、`gs130_ros/`、`launch/`） |
| 构建类型 | `ament_python` |
| `build_type` | `ament_python` |
| 安装后入口点 | `console_scripts: gs130_node = gs130_ros.gs130_node:main` |
| 安装后 launch 路径 | `share/gs130_ros/launch/gs130_web.launch.py`、`share/gs130_ros/launch/gs130_camera.launch.py` |

### 1.2 `package.xml` 字段（逐字）

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>gs130_ros</name>
  <version>0.1.0</version>
  <description>ROS 2 (TROS) interface for the GS130 stereo camera and IMU on the D-Robotics RDK X5.</description>
  <maintainer email="dev@d-robotics.cc">D-Robotics</maintainer>
  <license>MIT</license>
  <url type="repository">https://github.com/hachi-leaf/gs130_sdk</url>

  <buildtool_depend>ament_copyright</buildtool_depend>
  <buildtool_depend>ament_flake8</buildtool_depend>
  <buildtool_depend>ament_pep257</buildtool_depend>
  <buildtool_depend>python3-setuptools</buildtool_depend>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>diagnostic_msgs</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

说明：`maintainer` / `license` 取值与仓库既有约定一致（`core/include/gs130.h` 的 `Copyright (c) 2026 D-Robotics` + `SPDX-License-Identifier: MIT`，仓库根 `LICENSE` 为 MIT）。`geometry_msgs` 是 `sensor_msgs/Imu` 的传递依赖，仍然显式声明（ROS 2 要求直接使用即声明）。`tf2_msgs` **不**声明（§0.2）。

### 1.3 运行时依赖

| 类别 | 依赖 | 版本约束 | 缺失时行为 |
|---|---|---|---|
| ROS | `rclpy` | Humble | 节点无法启动 |
| ROS | `sensor_msgs` | Humble | 同上 |
| ROS | `geometry_msgs` | Humble | 同上（`Quaternion`/`Vector3`） |
| ROS | `std_msgs` | Humble | 同上（`Header`） |
| ROS | `diagnostic_msgs` | Humble | 仅 T8（`/gs130/status`，默认关闭）；缺失时 `publish_status:=true` 会在导入期失败 |
| ROS | `launch`、`launch_ros` | Humble | 仅影响 launch 文件 |
| ROS（可选链路） | `hobot_codec`（节点 `hobot_codec_republish`） | TROS 自带 | 仅 `gs130_web.launch.py` 需要；缺失时该链路启动失败，`gs130_camera.launch.py` 不受影响 |
| ROS（可选链路） | `websocket`（节点 `websocket`） | TROS 自带 | 同上 |
| Python | `numpy` | >= 1.20（随 TROS 提供） | 节点启动即失败 |
| Python | `gs130` | **必须与 `libgs130` 版本一致**，仓库版本 `0.0.1` | 节点启动即失败 |
| 系统 | `libgs130.so.0` | 与 `gs130` 包版本一致 | `gs130` 导入时报错；可用环境变量 `GS130_LIB` 指定绝对路径 |
| 系统 | 运行时 `hobot_shm` 环境 | TROS 自带 | **不强制**；见 §6.5 |

明确**不依赖** `cv2`（OpenCV）：驱动不做任何颜色转换。

---

## 2. 话题表（最终版）

### 2.1 硬性约定

- **链路约定（不可协商）**：stitched NV12 话题名固定为 `/image_combine_raw`，`encoding` 固定为 `"nv12"`。这是 D-Robotics 参考链的入口，只有逐字匹配才能保证 `hobot_codec_republish` + `websocket` **零修改**工作。
- **拼接约定（不可协商）**：stitched 帧为**水平拼接、左目在左、右目在右**。因此 `websocket` 的 `channel:=0` 显示的正是**左目**，与参考链语义一致。
- **时间戳约定（不可协商）**：同一帧的左目与右目时间戳相同；stitched 帧使用**左目**的时间戳。理由：SDK 内部对双目做半帧对齐（同一 `cycle_ns` 内），stitched 的语义是"这一对帧"，取左目即可且可复现。
- QoS 简写：`SD` = `rclpy.qos.qos_profile_sensor_data`，其精确值为
  reliability=`RELIABLE`，durability=`VOLATILE`，history=`KEEP_LAST`，depth=`1`（实测值，E1/E4）。
  下表若 depth 不是 `5`，则不用 `SD`，逐字写出四项。
- 表中所有话题均为**绝对话题名**（以 `/` 开头），节点**不**使用命名空间，**不**提供 `ns` 参数。理由：TROS 既有节点（`hobot_codec_republish`、`websocket`）的默认 `sub_topic`/`image_topic` 都是绝对名，命名空间会直接破坏默认链路。

### 2.2 话题

| # | 话题名 | 类型 | Reliability | Durability | History | Depth | frame_id | 默认发布频率 | v0.1.0 |
|---|---|---|---|---|---|---|---|---|---|
| T1 | `/image_combine_raw` | `sensor_msgs/msg/Image` | `RELIABLE` | `VOLATILE` | `KEEP_LAST` | `1` | `camera`（参数 `frame_id_combine`） | `fps`（默认 30 Hz） | **必需** |
| T2 | `/image_left_raw` | `sensor_msgs/msg/Image` | `RELIABLE` | `VOLATILE` | `KEEP_LAST` | `1` | `camera_left`（参数 `frame_id_left`） | `fps`（默认 30 Hz） | 可选，默认关闭 |
| T3 | `/image_right_raw` | `sensor_msgs/msg/Image` | `RELIABLE` | `VOLATILE` | `KEEP_LAST` | `1` | `camera_right`（参数 `frame_id_right`） | `fps`（默认 30 Hz） | 可选，默认关闭 |
| T4 | `/imu/data` | `sensor_msgs/msg/Imu` | `RELIABLE` | `VOLATILE` | `KEEP_LAST` | `200` | `imu_link`（参数 `frame_id_imu`） | `odr`（默认 200 Hz），见 §3.3 的突发说明 | **必需**（`device:=GS130W` 时不存在） |
| T5 | `/image_left/camera_info` | `sensor_msgs/msg/CameraInfo` | `RELIABLE` | `TRANSIENT_LOCAL` | `KEEP_LAST` | `1` | `camera_left` | 启动时 **1 次**（latched） | **必需** |
| T6 | `/image_right/camera_info` | `sensor_msgs/msg/CameraInfo` | `RELIABLE` | `TRANSIENT_LOCAL` | `KEEP_LAST` | `1` | `camera_right` | 启动时 **1 次**（latched） | **必需** |
| T7 | `/image_combine_jpeg` | `sensor_msgs/msg/CompressedImage` | 由 `hobot_codec_republish` 决定 | 由 `hobot_codec_republish` 决定 | — | — | — | `fps`（默认 30 Hz） | 可选链路（仅 `gs130_web.launch.py`） |
| T8 | `/gs130/status` | `diagnostic_msgs/msg/DiagnosticStatus` | `RELIABLE` | `VOLATILE` | `KEEP_LAST` | `1` | 无（该消息无 header） | 1 Hz | 可选，默认关闭 |

### 2.3 话题逐条论证（不能论证的已删除）

| # | 论证 |
|---|---|
| T1 | 唯一能保证既有 TROS 链零修改工作的入口名。SDK 侧能力来源：`Device.read_image()`（`stereo_layout = NONE` 时返回 `{"left": .., "right": ..}`，两块 NV12）+ `Image.timestamp_ns`。拼接在驱动内以**内存布局**完成（见 §3.1.3），不需要 SDK 的 `StereoLayout`。 |
| T2/T3 | 论证：SDK 原生输出就是**分离双目**（`Config.preset()` 固定 `stereo_layout=StereoLayout.NONE`），因此分离帧是零成本的原始数据，且是 `mode=rect` 双目矫正结果的唯一可用形式（拼接帧的下采样与矫正验证都更麻烦）。删除它们的代价是用户无法得到双目原始对；保留它们不增加硬件成本（同一帧内存切两半）。但**默认关闭**，因为默认链路只允许一个 `image_topic`，开两个只会浪费带宽。 |
| T4 | SDK 提供 `read_imu()`，`ImuPacket.accel` (m/s²)、`gyro` (rad/s)、`timestamp_ns`（已对齐相机时钟）。消费方是下游融合/标定流程，必须发布。 |
| T5/T6 | SDK 提供 `camera_intrinsics(LEFT/RIGHT)`（含 `K`、`dist_coeffs`、`dist_model`），且 SDK 在 `init()` 内**已把内参写回到最终输出分辨率**（`rdkx5.cpp` 的 VSE ROI+scale 回写），因此 K 与发布图像分辨率严格一致，可以发布，且必须发布。 |
| T7 | 由既有 `hobot_codec_republish` 发布；本包只**消费约定**，不自己发。计入本文档是为了说明它是 web 链的一部分。 |
| T8 | 论证：v0.1.0 的错误契约要求"丢帧 / 队列深度 / IMU 缺失"这类**非致命**异常可被外部观测（§7）。若评审认为日志足够，T8 可在 v0.1.0 发布前降级为彻底删除——它是本文档中唯一"可选且默认为关"的新增话题。 |
| 已删除：stitched 的 `camera_info` | stitched 帧含两个视场，**不存在**与之对应的单一内参；发布 `/image_combine/camera_info` 会是错误数据。相关内参由 T5/T6 承担。 |
| 已删除：`/image_combine_raw` 的压缩/rectified 变体 | 压缩由 T7 链路承担；rectified 由 `mode:=rect` 直接产出到 T1，不需要额外话题。 |
| 已删除：温度话题 `/imu/temperature` | 见 §0.2，无标准字段，引入自定义消息不值得。 |
| 已删除：`/tf`、`/tf_static` | 见 §0.2。 |

---

## 3. 消息构造规则（逐字段）

以下规则是**逐字段**的，实现不得"顺便多填"。

### 3.1 `sensor_msgs/msg/Image`（T1 / T2 / T3）

设：
- `ew` = 单目宽度 = 参数 `width`（默认见 §5）
- `eh` = 单目高度 = 参数 `height`
- `cw = 2 * ew`（仅 T1）

#### 3.1.1 T2 / T3（单目）

| 字段 | 值 |
|---|---|
| `header.stamp` | 见 §4 |
| `header.frame_id` | T2: `frame_id_left`；T3: `frame_id_right` |
| `height` | `eh`（真实图像高；**不是** `eh*3//2`，后者会让网编码节点段错误） |
| `width` | `ew` |
| `encoding` | 逐字 `"nv12"`（不允许 `"NV12"`、`"YUV420"`、`"yuv420"`） |
| `is_bigendian` | `0` |
| `step` | `ew` |
| `data` | `len(data) == ew * eh * 3 // 2`，即 `step * height`；内容为 SDK 返回的**紧排 NV12**：先 `ew * eh` 字节 Y 平面，后 `ew * eh // 2` 字节 UV 平面（UV 交错，`U0 V0 U1 V1 ...`） |

#### 3.1.2 ~~为何 `height = eh * 3 // 2`~~（已作废）

**本节结论已被实测推翻，不得实现。** 原文主张按"单平面 YUV420 视作 `width x (h*3/2)`"输出，
理由是 RViz/`image_geometry`/`hobot_codec_republish` 的通行约定。
实测（E2）表明 `hobot_codec_republish` 把 `height` 当作**真实图像高**：
传入 `eh*3//2` 时它记录 `init_pic_h_: 1080, alined_pic_h_: 1088` 并**段错误（退出码 -11）**。
正确约定见上表：`height = eh`、`step = ew`、`len(data) = ew*eh*3//2`。

原文引用的"本地基线探针 `ros/probe_nv12_publisher.py` 按此约定"同样是错的：
该探针初版用 `height = H*3//2`，正是它复现了 codec 的段错误；修正为 `height = H` 后
链路才在 30 fps 正常编码（见 `00_verified_platform_facts.md` E4）。

#### 3.1.3 T1（拼接帧）构造规则（已按 E6 改写）

**不使用手工拼接。** 在 `Config.preset()` 之后设置
`config.camera_config.stereo_layout = StereoLayout.LEFT_RIGHT`，
由 SDK 硬件输出拼接帧；`read_image()` 返回 `{"stitched": frame}`，
其 numpy 形状为 `(eh*3//2, ew*2)`。实测 640x480 配置得到 `(720, 1280)`。
驱动侧不得再写任何 `concatenate` 代码。

原文的 `data = concatenate([L.reshape(-1), R.reshape(-1)])` 已作废：
它既非必要（硬件已支持），其 UV 相位也与 SDK 的拼接布局语义不同。

| 字段 | 值 |
|---|---|
| `header.stamp` | 拼接帧的 `timestamp_ns` 换算结果（§4） |
| `header.frame_id` | `frame_id_combine` |
| `height` | `eh`（真实图像高） |
| `width` | `ew * 2` |
| `encoding` | `"nv12"` |
| `is_bigendian` | `0` |
| `step` | `ew * 2` |
| `data` | `len(data) == ew * 2 * eh * 3 // 2 == step * height` |

**关于 SDK 的 `stereo_layout`**：原文称"不使用 SDK 的 `stereo_layout`，因为 `mode=raw` 时
拼接宽度 2176 与 1088x1280 约束冲突"。该冲突属实，但结论相反：拼接帧使用
`resize`/`rect` 模式（默认 `resize`），`raw` 模式本就不用于拼接。
即 **`stereo_layout != none` 与 `mode=raw` 互斥**，节点对 `raw` + 非原生尺寸会 fail-fast。

### 3.2 发布条件（T1/T2/T3）

- 每轮循环调用 `Device.read_image()`；返回 `None`（SDK 语义：队列为空，`GS130_TIMEOUT`）→ **本轮回调不发消息**，记录一次 `drop` 计数，不报错、不退出、不打日志（`/gs130/status` 与 1 Hz 节流日志除外，§7）。
- 返回字典含 `"left"`/`"right"` → 发布 T1（若 `publish_combine`）；若 `publish_per_eye` 为真，则**同一组数据**再发布 T2/T3（同一 `stamp`，禁止两次读设备凑对）。
- 一次读取绝不重复发布同一帧超过一次。

### 3.3 `sensor_msgs/msg/Imu`（T4）

| 字段 | 值 | 说明 |
|---|---|---|
| `header.stamp` | `ImuPacket.timestamp_ns` 按 §4 换算 | SDK 已把 IMU 时间戳对齐到相机时钟域（`TimestampTracker`），因此与图像时间戳**同一时基、可直接比较** |
| `header.frame_id` | `frame_id_imu` | 加速度与角速度都表达在该坐标系中 |
| `orientation.x/y/z` | `0.0` | **显式决策**：驱动不做姿态解算，四个分量保持消息默认值 |
| `orientation.w` | `1.0` | 消息默认值 |
| `orientation_covariance[0]` | `-1.0` | 逐字 `-1` 表示"本消息不含姿态估计"。**不得填 0**（ROS 约定 `0` = "有估计但协方差未知"，会让下游错误地把单位四元数当作有效姿态） |
| `orientation_covariance[1..8]` | `0.0` | 未使用 |
| `angular_velocity.x/y/z` | `ImuPacket.gyro[0/1/2]` | 单位 **rad/s**（SDK 原样，已按 `gyro_fsr_dps` 与 EEPROM 标定缩放），不做符号翻转、不做轴交换 |
| `angular_velocity_covariance[0]` | `-1.0` | 驱动没有逐样本协方差；用 `-1` 显式声明"无估计"，而不是伪造对角阵 |
| `angular_velocity_covariance[1..8]` | `0.0` | |
| `linear_acceleration.x/y/z` | `ImuPacket.accel[0/1/2]` | 单位 **m/s²**（SDK 原样，已按 `accel_fsr_g` 与 EEPROM 标定缩放），不做符号翻转、不做轴交换 |
| `linear_acceleration_covariance[0]` | `-1.0` | 同上 |
| `linear_acceleration_covariance[1..8]` | `0.0` | |

**约定说明（必须写进用户文档）**：
- `linear_acceleration` 按 ROS 约定**包含重力**（`sensor_msgs/Imu` 的字段语义为 proper acceleration，静止朝上时 `z ≈ +9.81`）。驱动不对重力做任何扣除。
- SDK 的 `ImuPacket.temp`（摄氏度）**没有** `sensor_msgs/Imu` 字段。**决定**：v0.1.0 **丢弃**该值，不做话题、不做自定义消息。理由：为一个诊断量引入自定义消息会污染依赖表；需要温度的用户应直接调用 SDK。若未来需要，只能通过新增话题（**契约变更，需评审**）。
- SDK 的 `ImuPacket.is_fsync` **不发布**。它是 SDK 内部握手/时钟同步的中间量，不是传感器观测。
- **发布时间点与突发性**：SDK 的 IMU 时间戳与数据包是**成组**产出的（仅在 FSYNC 锚点后成组释放，见 `gs130.cpp` 的 `take_ready` 分支），因此 T4 的瞬时发布是"突发 + 静默"，**平均**速率等于 `odr`。契约只保证：**不丢包、不重排、`header.stamp` 严格递增**（时间戳可能因补点而相等时，按 SDK 给出的顺序发布，顺序即时间顺序）。禁止在驱动侧"平滑"或按固定周期重发。
- 由 `ImuPacket.is_fsync` 触发的配对意味着：`odr:=200` 或 `odr:=500`，SDK 只接受这两个值（`icm42688.cpp` 的 `kOdr` 表，其他值返回 `GS130_UNSUPPORTED`，不取整、不替代）。

### 3.4 `sensor_msgs/msg/CameraInfo`（T5 / T6）

数据来源：`Device.camera_intrinsics(CameraIndex.LEFT | CameraIndex.RIGHT)`，字段 `fx, fy, cx, cy, K(3x3), dist_coeffs(8), dist_model`。

**关键前提（已核实）**：SDK 在 `gs130_init()` 内部完成 VSE ROI + scale 的内参回写（`rdkx5.cpp`：`k->fx *= sfx; k->cx = (k->cx - roi.x) * sfx; ...`），`mode=rect` 下先经 `stereo_rectify()` 写入虚拟内参再回写。因此 `camera_intrinsics()` 返回的 K **已经处于最终输出分辨率**。驱动**不得**再乘任何缩放系数。

| 字段 | 值 |
|---|---|
| `header.stamp` | 节点启动时的 `RCL_ROS_TIME` 时刻（标定为静态量，取一次即可）；**不得**填 0 |
| `header.frame_id` | T5: `frame_id_left`；T6: `frame_id_right`（与对应 Image 完全一致，ROS 要求冲突时行为未定义） |
| `height`, `width` | 单目输出高度 `eh`、宽度 `ew`（**不是** `eh*3//2`） |
| `distortion_model` | 见 §3.4.1 |
| `D` | 见 §3.4.1；长度必须与 `distortion_model` 匹配（`plumb_bob` 5 个、`equidistant` 4 个） |
| `K` | `[fx, 0, cx, 0, fy, cy, 0, 0, 1]`，即 SDK 的 `K` 逐元素复制（行主序） |
| `R` | **全 0**（9 个 0.0）。理由：`R` 的语义是"把点从相机坐标旋转到理想双目平行坐标系"的矫正矩阵，ROS 消费者在未标定的单目场景下约定全 0；SDK 的 `stereo_rectify` 只在 `mode=rect` 时内部构造 `rect_R`，**不通过 C API 暴露**，驱动无法给出正确值。填单位阵会是伪造数据。 |
| `P` | **全 0**（12 个 0.0）。理由同上：正确的 `P` 必须在矫正后的公共分辨率与共享焦距下构造，该信息未暴露给调用方；`mode:=rect` 时"图像已矫正"这一事实由 `distortion_model` + `D` 表达，不由 `P` 表达。 |
| `binning_x`, `binning_y` | **0**（ROS 约定 0 = 无 binning） |
| `roi.x_offset`, `roi.y_offset`, `roi.height`, `roi.width` | **0**（0 = 全图） |
| `roi.do_rectify` | `False` |

#### 3.4.1 `distortion_model` 与 `D` 的映射规则（唯一版本）

| 条件 | `distortion_model` | `D` |
|---|---|---|
| 参数 `mode` 的取值为 `raw` 或 `resize`，且 SDK 报 `DistModel.FISHEYE`（GS130 的 EEPROM 标定为渔眼，见 `union_stereo_imu_fisheye_v1p2r0n4.cpp`） | 逐字 `"equidistant"` | `[d0, d1, d2, d3]`，即 SDK `dist_coeffs[0..3]` = `k1 k2 k3 k4`（OpenCV `fisheye` 的 4 参数模型），**只发前 4 个** |
| 参数 `mode` 的取值为 `raw` 或 `resize`，且 SDK 报 `DistModel.PINHOLE` | 逐字 `"plumb_bob"` | `[k1, k2, p1, p2, k3]` = SDK `dist_coeffs[0..4]`，**只发前 5 个** |
| 参数 `mode` 的取值为 `rect` | 逐字 `"plumb_bob"` | `[0.0, 0.0, 0.0, 0.0, 0.0]` |
| 参数 `camera_info_distortion_model` 非 `auto` | 由该参数逐字决定（`plumb_bob` / `equidistant` / `none`） | `none` → 空数组 `[]`；其余按上表对应项取值 |

规则依据（必须写在实现注释里）：
1. SDK 的 `DistModel` 只有两个枚举值，映射为 ROS 字符串是**固定映射**，不存在第三种解释：`FISHEYE → "equidistant"`、`PINHOLE → "plumb_bob"`。
2. `mode=rect` 下 SDK 的 `stereo_rectify()` 会 `memset(k->dist_coeffs, 0, ...)` 做**原地清零**，但 `dist_model` 枚举**仍为 FISHEYE**。此时图像已无畸变，逐字上报 `"equidistant"` 会误导下游再做一次渔眼去畸变。因此 `rect` 模式**强制**上报 `"plumb_bob"` + 全 0 的 5 元 `D`。这是一条**显式覆盖规则**，不是"自动猜测"。
3. SDK 的 `dist_coeffs` 是 8 元；`plumb_bob` 在 ROS 中是 5 元、`equidistant` 是 4 元。**超出的系数（k4..k6 / k4..k6,p1,p2 的尾部）如实丢弃**——ROS 的 `D` 无法表达它们。此丢弃必须写进用户文档，不得静默。
4. 参数 `camera_info_distortion_model:=none` 时 `D` 为空数组，用于确信图像已矫正、不希望下游做任何去畸变的下游栈。

#### 3.4.2 分模式一致性表

| `mode` | 发布图像分辨率 | `camera_info` 的 `height/width` | K 的含义 | `R` / `P` |
|---|---|---|---|---|
| `raw` | `1088 x 1280`（强制，见 §5.2） | `1280 x 1088`（高 x 宽） | 1088x1280 下的原始内参 | 全 0 / 全 0 |
| `resize` | 用户配置的 `height x width` | 与配置值一致 | **已按 VSE ROI+scale 回写**，与发布图像严格对应，无需用户再缩放 | 全 0 / 全 0 |
| `rect` | 用户配置的 `height x width` | 与配置值一致 | 矫正后的虚拟内参（左右焦距统一、主点居中） | 全 0 / 全 0 |

---

## 4. 时间戳规则（唯一版本）

### 4.1 原始时间基准

- 图像：`gs130.Image.timestamp_ns`。SDK 取值优先级为 `trig_tv`（LPWM 上升沿 = 曝光触发时刻）→ `timestamps` → `tv`（`rdkx5.cpp` 的 `frame_ts_ns`）。单位纳秒，`uint64`。
- IMU：`gs130.ImuPacket.timestamp_ns`。SDK 注释明确为 "corrected absolute timestamp (aligned to the camera clock)"（`gs130.h`），即与图像**同域**。
- 因此：**图像与 IMU 的时间戳可以直接相减**，驱动不做任何插值、不做任何重采样、不做任何各自为政的本地打点。

### 4.2 设备时钟 → `header.stamp` 的换算（唯一公式）

设备时钟的绝对纪元（REALTIME 还是 MONOTONIC）**不由本契约假定**。驱动用一个在启动时确定、运行期恒定的偏移量把设备时钟搬进 ROS 时钟：

```
# 启动时（第一次拿到任意有效设备时间戳时）执行一次：
offset_ns = (节点时钟 now_ns) - (device_ts_ns)      # 仅当 stamp_offset_ns == 0
offset_ns = stamp_offset_ns                          # 当 stamp_offset_ns != 0

# 每条消息：
stamp_ns  = device_ts_ns + offset_ns
header.stamp.sec     = stamp_ns // 1_000_000_000
header.stamp.nanosec = stamp_ns %  1_000_000_000
```

性质（必须保持，测试须验证）：
1. `offset_ns` 在一次进程生命周期内**只计算一次**，之后**恒定**。禁止每条消息都用 `now()` 覆盖（会抹掉曝光间隔），禁止用滑动平均（会引入抖动）。
2. 因此**相邻消息的时间差与设备时间差严格相等**，多传感器对齐不受影响。
3. 偏移量一旦确定，**图像与 IMU 共用同一个偏移量**，两者相对关系保持设备原值。
4. 节点时钟使用 `RCL_ROS_TIME`（`self.get_clock()`）。
5. **`use_sim_time` 必须为 `false`**。节点在启动时若检测到 `use_sim_time` 为真，打 `WARN` 并以 `use_sim_time:=false` 的语义继续（即用系统时钟），同时在日志中说明偏移量会因仿真时钟而变化不可复现。

### 4.3 时间戳为零 / 不可用时的规则

| 情形 | 规则 |
|---|---|
| 某帧 `timestamp_ns == 0` | 该帧**仍发布**，`header.stamp` 使用"当前 `offset_ns` + 当前设备时间不可知"，取 **最近一次有效设备时间戳 + `1e9 // fps`**（保持单调递增）；同时 `WARN`（1 Hz 节流）"frame timestamp unavailable" |
| 该帧是**第一帧**且 `timestamp_ns == 0` | `offset_ns` 尚不可计算 → 本帧 `header.stamp` 取节点 `now()`，并**不**据此确定 `offset_ns`；待第一个有效时间戳到来时再确定偏移量（此前已发布的 `stamp` 不回溯修改） |
| IMU 包 `timestamp_ns == 0` | **丢弃该包**（`WARN`，1 Hz 节流）。理由：IMU 时间戳为 0 时无法与图像对齐，发布错误时间戳比丢包更有害 |
| `device_ts_ns + offset_ns` 为负 | 不可能（设备 ns 为无符号），若发生则视为编程错误，`ERROR` 并退出（退出码 1） |
| 时间戳非单调（回退） | IMU 包直接丢弃并 `WARN`；图像帧照发但不修改 `offset_ns` |

### 4.4 IMU 与图像的时间关系（必须写进用户文档）

- 两个话题的时间戳**同一时基**，可直接互比；但 SDK 的 IMU 输出是 FSYNC 锚点后的突发，**IMU 样本的时间戳可以早于最近一帧图像的时间戳**（补点是"过去"的样本），这是 SDK 的补点语义，不是驱动错误。
- 驱动**不**发布"IMU 与图像的时间差"、"同步标志"等派生量。
- 驱动**不**对 IMU 做插值/对齐到图像；需要严格对齐的下游应使用 `message_filters`，本契约只保证时间戳的物理意义正确。

---

## 5. 参数表（最终版）

### 5.1 完整参数表

所有参数在节点启动时用 `declare_parameter` 声明。`Reconfig` 列含义：`No` = 运行期修改无效（节点不会读取新值，修改需重启进程）；`N/A` = 参数只影响当前进程的一次性行为。

| 参数名 | 类型 | 默认值 | 取值 | 作用 | Reconfig |
|---|---|---|---|---|---|
| `platform` | string | `"RDKX5"` | `"RDKX5"`（唯一） | 传给 `Config.preset(platform, ...)`；其他值直接报错退出 | No |
| `device` | string | `"GS130WI"` | `"GS130WI"`、`"GS130W"` | 选择带 IMU 的 WI 或不带 IMU 的 W 模组；决定是否探测 IMU 与 `right_addr`（0x32/0x31） | No |
| `mode` | string | `"resize"` | `"raw"`、`"resize"`、`"rect"` | 相机输出通路：`raw` = CAM→VIN→ISP→OUT（无 VSE/GDC）；`resize` = +VSE（缩放，含安装角度旋转）；`rect` = +GDC（双目矫正，**必须有 EEPROM 标定**） | No |
| `width` | int | `640` | `[16, 1088]`，偶数；`mode:=raw` 时必须 `== 1088` | 单目输出宽度；`resize`/`rect` 下为 VSE 输出宽度（须满足 SDK 的 ROI 整除约束，不满足时启动报 `GS130_UNSUPPORTED`） | No |
| `height` | int | `480` | `[16, 1280]`，偶数；`mode:=raw` 时必须 `== 1280` | 单目输出高度，约束同上 | No |
| `fps` | int | `30` | `[1, 33]` | LPWM 触发频率（`1000000/fps` µs 周期）。**不得超过传感器读出率**（当前 `line_length=1400`、`frame_length=1500` 下约 33 fps），超过会造成帧丢失/异常 | No |
| `odr` | int | `200` | `200`、`500`（SDK 只接受这两个值） | IMU 输出数据率；其他值 SDK 返回 `GS130_UNSUPPORTED`，节点报错退出 | No |
| `frame_id_combine` | string | `"camera"` | 任意非空字符串 | T1 的 `header.frame_id`；与既有链路/基线探针一致 | No |
| `frame_id_left` | string | `"camera_left"` | 任意非空字符串 | T2、T5 的 `header.frame_id` | No |
| `frame_id_right` | string | `"camera_right"` | 任意非空字符串 | T3、T6 的 `header.frame_id` | No |
| `frame_id_imu` | string | `"imu_link"` | 任意非空字符串 | T4 的 `header.frame_id` | No |
| `publish_combine` | bool | `true` | `true`/`false` | 是否发布 T1（`/image_combine_raw`）。**置 false 会断开 TROS web 链路** | No |
| `publish_per_eye` | bool | `false` | `true`/`false` | 是否发布 T2/T3（单目 NV12） | No |
| `publish_imu` | bool | `true` | `true`/`false` | 是否发布 T4。**置 false 只是不发布，SDK 仍会启动 IMU 并完成相机 FSYNC 握手** | No |
| `publish_camera_info` | bool | `true` | `true`/`false` | 是否发布 T5/T6（启动时各一次，latched） | No |
| `publish_status` | bool | `false` | `true`/`false` | 是否发布 T8（`/gs130/status`，1 Hz） | No |
| `stamp_offset_ns` | int | `0` | `0` 或任意整数 | `0` = 使用 §4.2 的自动偏移；非 `0` = 强制使用该偏移量（用于可复现测试/回放） | No |
| `stamp_offset_mode` | string | `"auto"` | `"auto"`、`"device"` | `auto` = §4.2 换算；`device` = 直接使用设备纳秒值（不推荐，见 §5.4） | No |
| `camera_info_distortion_model` | string | `"auto"` | `"auto"`、`"plumb_bob"`、`"equidistant"`、`"none"` | 覆盖 §3.4.1 的畸变模型上报 | No |
| `poll_period_ms` | int | `2` | `[1, 50]` | 相机/IMU 轮询周期。SDK 的 `read_image()`/`read_imu()` 为非阻塞空队列返回 `None`，驱动以该周期轮询 | No |
| `imu_qos_depth` | int | `200` | `[1, 2000]` | T4 的 `KEEP_LAST` 深度。默认 `200` = 在 200 Hz 下可容忍 1 s 的订阅者滞后 | No |
| `image_qos_depth` | int | `1` | `[1, 100]` | T1/T2/T3 的 `KEEP_LAST` 深度 | No |
| `log_fps_period_s` | double | `5.0` | `(0, 600]` | 实际帧率/IMU 速率统计日志的周期；`0` 关闭 | No |
| `start_timeout_s` | double | `10.0` | `[0.1, 600]` | `Device.start()` 未返回时的 `WARN` 周期（该等待不可中断，见 §7.5） | No |
| `codec_channel` | int | `0` | `[0, 255]` | **仅 launch 使用**：`hobot_codec_republish` 的 `channel` | N/A |
| `websocket_channel` | int | `1` | `[0, 255]` | **仅 launch 使用**：`websocket` 的 `channel`；必须与 `codec_channel` 不同 | N/A |
| `web_port` | int | `8000` | `[1024, 65535]` | **仅 launch 使用**：由既有 nginx 配置（port 8000）承载，**本包不启动 nginx**；该参数只作为启动前检查与文档提示 | N/A |
| `web_output_fps` | int | `30` | `[1, 60]` | **仅 launch 使用**：`websocket` 的 `output_fps` | N/A |
| `jpg_quality` | int | `80` | `[1, 100]` | **仅 launch 使用**：`hobot_codec_republish` 的 `jpg_quality` | N/A |
| `smart_topic` | string | `"/image_combine_jpeg"` | 任意话题名 | **仅 launch 使用**：`websocket` 的 `smart_topic`；默认与 `image_topic` 相同，保证既有节点行为不变 | N/A |
| `hobot_shm` | bool | `true` | `true`/`false` | **仅 launch 使用**：是否按 TROS 约定设置零拷贝共享内存环境。本包**不依赖**它（§6.5） | N/A |

### 5.2 参数与 SDK 的对应（实现必须逐字使用）

```
config = gs130.Config.preset(platform, device, mode, width, height, fps, odr)
device = gs130.Device(config)          # platform/device 非法时 preset 抛 ValueError
device.start()                          # 检测到 IMU 时，SDK 内部先等 FSYNC 握手完成
```

- `mode` 的字符串到 `gs130.CameraMode` 的映射在本包内定义：`"raw" → CameraMode.RAW`、`"resize" → CameraMode.RESIZE`、`"rect" → CameraMode.RECT`（与 `python/test/test_gs130.py` 的 `mode_from_text()` 完全一致，不允许第二套拼写）。
- `mode:=raw` 时 `width`/`height` **必须**等于 1088/1280，否则 `Config.preset` 之后的 `gs130_init` 返回 `GS130_PARAM_ERROR`（SDK 硬约束）。节点在**启动参数校验阶段**就拒绝，不等 SDK 报错。
- `mode:=rect` 必须有 EEPROM 标定；无 EEPROM 时 SDK 返回 `GS130_PARAM_ERROR`。
- `device:=GS130W` 时无 IMU：不创建 T4，`odr` 参数被忽略（不报错，但启动日志说明"IMU not present"）。

### 5.3 为什么没有动态重配置（说明）

`gs130.Config` 的所有相机/IMU 字段都在 `gs130_init()` 时下发给硬件（VIN/ISP/VSE/GDC 通路与 IMU 寄存器一次配置），SDK **不提供**运行期改配置的 API。因此本节点**不注册** `rcl_interfaces` 参数回调；`ros2 param set` 会成功设置变量但**不产生任何效果**。参数表 `Reconfig=No` 即此含义。需要改参数 → 重启节点/重新 launch。

### 5.4 `stamp_offset_mode:=device` 的警告

设备时间戳的绝对纪元未由 SDK 声明带有 ROS 语义，直接发布到 `header.stamp` 会使 RViz / TF / `ros2 bag` 的时间轴与系统时间脱节。该模式**仅**用于：① 离线分析设备原始时基；② 需要逐字节复现设备时间戳的测试。使用时节点启动即打 `WARN`。

### 5.5 非目标参数（**故意不提供**，不得实现）

| 未提供的参数 | 理由 |
|---|---|
| `frame_id`（单一参数） | 一个节点有 4 个 frame（stitched/left/right/imu），单参数无法表达；用 4 个独立参数（§5.1） |
| `sensor_width` / `sensor_height` / `line_length` / `frame_length` / `left_addr` / `right_addr` / `bus` / `bus_mipi_rx` / `bus_reset_gpio` / `fsync_camera` / `tuning_file` | 全部由 `Config.preset()` 按模组定死；暴露它们等于允许用户把硬件配置改坏，而 `preset()` 是不支持这些覆写的 |
| `stereo_layout` | v0.1.0 固定为 `NONE`，拼接在驱动侧做（§3.1.3） |
| `accel_fsr_g` / `gyro_fsr_dps` / `accel_bw_sel` / `gyro_bw_sel` | `preset()` 固定为 `16 g` / `2000 dps` / `0` / `0`；改变会改变 SDK 的物理量换算，属于标定级变更 |
| `camera_fifo_depth` / `imu_fifo_depth` | `preset()` 固定为 `4` / `1024`（`DROP_OLD`）；驱动侧不再二次缓存 |
| `use_compressed` / `jpeg_quality` / `image_transport` | 压缩是 `hobot_codec_republish` 的职责（§0.2） |
| `qos_reliability` / `qos_durability` 可调 | QoS 是链路契约的一部分（`hobot_codec_republish` 以 `RELIABLE` 订阅，实测 E1），不允许用户降级成 `BEST_EFFORT` 后破坏兼容；只允许调 depth |
| `enable_depth` / `stereonet_*` | 深度由既有 `hobot_stereonet` 负责（§0.2） |
| `imu_orientation_source` / `enable_ahrs` | 姿态解算是非目标（§3.3） |
| `temperature_topic` | 非目标（§0.2、§3.3） |
| `publish_tf` | 非目标（§0.2）；`tf2_msgs` 不在依赖表 |
| `ns`（命名空间） | 见 §2.1（绝对话题名） |
| `reconnect` / `auto_restart` | SDK 的错误语义要求"出错 → `deinit` + `init` 恢复"（`gs130.h` 的 `GS130_THREAD_CLOSED` 注释）。v0.1.0 明确**不做**自动恢复（§7）。 |

---

## 6. Launch 契约

### 6.1 文件清单（逐字）

| 文件 | 路径 | 作用 |
|---|---|---|
| `gs130_web.launch.py` | `ros/gs130_ros/launch/gs130_web.launch.py` | 启动相机节点 + `hobot_codec_republish` + `websocket`，构成 TROS web 显示链 |
| `gs130_camera.launch.py` | `ros/gs130_ros/launch/gs130_camera.launch.py` | **只**启动相机节点（无 web 链路，不依赖 `hobot_codec` / `websocket` 存在） |

两个 launch 文件都**只**启动 `gs130_node` 这一个本包进程；`gs130_web.launch.py` 额外启动两个既有节点，且**不**修改它们的任何行为。

### 6.2 `gs130_web.launch.py`

启动的节点（顺序固定）：

1. `gs130_node`（本包，`package="gs130_ros"`, `executable="gs130_node"`）— 参数见 §6.4。
2. `hobot_codec_republish`（`package="hobot_codec"`, `executable="hobot_codec_republish"`）— 参数逐字：

| 参数 | 值 |
|---|---|
| `channel` | `LaunchConfiguration("codec_channel")`，默认 `0` |
| `in_mode` | `"ros"` |
| `in_format` | `"nv12"` |
| `out_mode` | `"ros"` |
| `out_format` | `"jpeg"` |
| `sub_topic` | `"/image_combine_raw"` |
| `pub_topic` | `"/image_combine_jpeg"` |
| `jpg_quality` | `LaunchConfiguration("jpg_quality")`，默认 `80` |
| `input_framerate` | `-1`（不抽帧，跟随输入；**不得**设为小于 `fps` 的值，否则会与 web 侧 `output_fps` 叠加抽帧） |
| `output_framerate` | `-1`（不抽帧） |

3. `websocket`（`package="websocket"`, `executable="websocket"`）— 参数逐字：

| 参数 | 值 |
|---|---|
| `image_topic` | `"/image_combine_jpeg"` |
| `image_type` | `"mjpeg"` |
| `only_show_image` | `False` |
| `output_fps` | `LaunchConfiguration("web_output_fps")`，默认 `30` |
| `channel` | `LaunchConfiguration("websocket_channel")`，默认 `1` |
| `smart_topic` | `LaunchConfiguration("smart_topic")`，默认 `"/image_combine_jpeg"`（无智能话题时与 `image_topic` 相同，保证既有节点行为不变） |

显示语义（必须写进用户文档）：`websocket` 的 `channel:=0` 对应 **`/image_combine_raw` 拼接帧的左半部分 = 左目**。

### 6.3 `gs130_camera.launch.py`

只启动 `gs130_node`，参数与 §6.4 中同名参数取相同默认值，**额外**固定：`publish_per_eye` 默认仍为 `false`，`publish_status` 默认仍为 `false`。该文件**不**声明 `codec_channel`、`websocket_channel`、`web_port`、`web_output_fps` 以外的 web 参数。可用 `ros2 launch gs130_ros gs130_camera.launch.py publish_per_eye:=true` 单独取双目原始流。

### 6.4 两个 launch 文件共同声明的参数（launch 参数，不是节点参数）

| launch 参数 | 默认值 | 传给 |
|---|---|---|
| `platform` | `"RDKX5"` | `gs130_node.platform` |
| `device` | `"GS130WI"` | `gs130_node.device` |
| `mode` | `"resize"` | `gs130_node.mode` |
| `width` | `"640"` | `gs130_node.width` |
| `height` | `"480"` | `gs130_node.height` |
| `fps` | `"30"` | `gs130_node.fps` |
| `odr` | `"200"` | `gs130_node.odr` |
| `frame_id_combine` | `"camera"` | `gs130_node.frame_id_combine` |
| `frame_id_left` | `"camera_left"` | `gs130_node.frame_id_left` |
| `frame_id_right` | `"camera_right"` | `gs130_node.frame_id_right` |
| `frame_id_imu` | `"imu_link"` | `gs130_node.frame_id_imu` |
| `publish_combine` | `"true"` | `gs130_node.publish_combine` |
| `publish_per_eye` | `"false"` | `gs130_node.publish_per_eye` |
| `publish_imu` | `"true"` | `gs130_node.publish_imu` |
| `publish_camera_info` | `"true"` | `gs130_node.publish_camera_info` |
| `publish_status` | `"false"` | `gs130_node.publish_status` |
| `stamp_offset_mode` | `"auto"` | `gs130_node.stamp_offset_mode` |
| `camera_info_distortion_model` | `"auto"` | `gs130_node.camera_info_distortion_model` |

`LaunchConfiguration` 在 launch 文件中是**字符串**；launch 实现必须显式类型转换（`int(...)` / `bool` 由 `launch_ros.utilities.evaluate_parameters` 处理），不得依赖隐式转换。

`gs130_web.launch.py` 额外声明：`codec_channel`（`"0"`）、`websocket_channel`（`"1"`）、`web_port`（`"8000"`）、`web_output_fps`（`"30"`）、`jpg_quality`（`"80"`）、`smart_topic`（`"/image_combine_jpeg"`）、`hobot_shm`（`"true"`）。

### 6.5 `hobot_shm` 处理（零拷贝环境）

`gs130_node` **不**依赖共享内存传输（它直接发布堆上的 `data`），因此**不要求** `hobot_shm` 环境。若用户已 `source` 了 `hobot_shm` 提供的零拷贝环境，既有节点可能走 `shm` 传输；此时 `gs130_node` 的 `data` 会被复制一次，功能仍然正确。`hobot_shm` 参数只作为一个**使能开关**在 launch 中暴露（`true` 时按 TROS 约定设置相应的环境变量），不改变本包任何 topic 名字或编码。

### 6.6 必须能跑通的命令（逐字，v0.1.0 验收项）

```bash
# 0) 环境
source /opt/tros/humble/setup.bash

# 1) 构建（在工作空间根目录，或把 ros/gs130_ros 链接/拷贝进 src/）
cd ~/ros2_ws && colcon build --packages-select gs130_ros
source install/setup.bash

# 2) web 链路（与任务书给出的命令逐字一致）
ros2 launch gs130_ros gs130_web.launch.py platform:=RDKX5 device:=GS130WI mode:=resize width:=640 height:=480 fps:=30 odr:=200

# 3) 仅相机（无 web 链路）
ros2 launch gs130_ros gs130_camera.launch.py platform:=RDKX5 device:=GS130WI mode:=resize width:=640 height:=480 fps:=30 odr:=200

# 4) 仅相机 + 双目原始流 + 状态
ros2 launch gs130_ros gs130_camera.launch.py mode:=rect width:=640 height:=480 fps:=30 publish_per_eye:=true publish_status:=true

# 5) raw 模式（必须显式给出 1088x1280）
ros2 launch gs130_ros gs130_camera.launch.py mode:=raw width:=1088 height:=1280 fps:=30 odr:=200

# 6) 直接跑节点（不经 launch，参数必须全部自带默认值）
ros2 run gs130_ros gs130_node

# 7) 验收检查
ros2 topic hz /image_combine_raw
ros2 topic echo /image_combine_raw --once --field encoding      # 期望: nv12
ros2 topic echo /image_left/camera_info --once                  # 期望: latched, D 长度 4 或 5
ros2 topic hz /imu/data
ros2 topic echo /image_combine_jpeg --once --field format | head -1
# 浏览器打开 http://<板卡IP>:8000 ，选择 channel 0 -> 左目画面
```

---

## 7. 错误与诊断契约

### 7.1 SDK 错误码 → ROS 行为（逐条，唯一）

SDK 错误以 `gs130.GS130Error`（携带 `ErrorCode`）抛出。

| 阶段 | SDK `ErrorCode` | 日志级别 | 消息（逐字模板，`{}` 内为值） | 节点是否退出 |
|---|---|---|---|---|
| 参数校验（未触碰设备前） | — | `ERROR` | `invalid parameter {name}={value}: {reason}` | **是**，退出码 `2` |
| `Config.preset()` | `ValueError` | `ERROR` | `unsupported platform/device: {platform} {device}` | **是**，退出码 `2` |
| `Device(config)` / `gs130_init` | `PARAM_ERROR` | `ERROR` | `gs130_init failed: PARAM_ERROR ({hint})`；`hint` 在 `mode=raw` 时逐字为 `RAW mode requires width=1088 height=1280`，`mode=rect` 时逐字为 `RECT mode requires EEPROM calibration` | **是**，退出码 `1` |
| 同上 | `NOT_FOUND` | `ERROR` | `gs130_init failed: NOT_FOUND: camera not detected on the configured I2C buses` | **是**，退出码 `1` |
| 同上 | `UNSUPPORTED` | `ERROR` | `gs130_init failed: UNSUPPORTED: camera cannot produce the requested configuration (mode={mode} {width}x{height}@{fps})` | **是**，退出码 `1` |
| 同上 | `HW_ERROR` | `ERROR` | `gs130_init failed: HW_ERROR` + 若 `/proc` 中存在其他持有者提示则追加 `a camera process may already own the MIPI CSI port; see 11_interface_freeze.md §8` | **是**，退出码 `1` |
| `Device.start()` / `gs130_start` | 任意非 `OK` | `ERROR` | `gs130_start failed: {CODE}` | **是**，退出码 `1` |
| 轮询 `read_image()` | `HW_ERROR` / `THREAD_CLOSED` | `ERROR` | `camera stream failed: {CODE}` | **是**，退出码 `1`（见 §7.2） |
| 轮询 `read_image()` | `TIMEOUT`（Python 侧表现为返回 `None`） | 无日志（仅在 `publish_status` 时计数） | — | 否 |
| 轮询 `read_imu()` | `TIMEOUT`（返回 `None`） | 无日志 | — | 否 |
| 轮询 `read_imu()` | `HW_ERROR` / `THREAD_CLOSED` | `ERROR`（1 次） | `imu stream failed: {CODE}; IMU publishing disabled, camera continues` | **否**：停止发布 T4，相机继续；`/gs130/status` 报 `ERROR` |
| 设备无 IMU（`device:=GS130W` 或 `imu_name is None`） | — | `WARN`（1 次） | `IMU not present; /imu/data will not be published` | 否 |
| 无 EEPROM 标定但 `publish_camera_info:=true` | `NOT_FOUND` | `WARN`（1 次） | `calibration not available; /image_left/camera_info and /image_right/camera_info will not be published` | 否 |
| 时间戳异常（§4.3） | — | `WARN`（1 Hz 节流） | `frame timestamp unavailable` / `IMU timestamp zero: packet dropped` | 否 |
| QoS 不匹配（订阅者 `RELIABLE`） | — | `WARN`（1 Hz 节流） | `image subscriber {topic} requested incompatible QoS` | 否 |

### 7.2 致命 / 非致命的分界（唯一规则）

- **致命（进程退出）**：设备初始化失败、启动失败、**相机**数据通路失败。理由：相机是唯一数据源，通路坏了就没有可发布的内容；SDK 明确要求"出错后走 `deinit` + `init` 恢复"（`gs130.h` 的 `GS130_THREAD_CLOSED` 注释），v0.1.0 不实现自动恢复，因此选择"退出并被 launch/supervisor 看到"，而不是静默停留。
- **非致命（继续运行）**：空队列、IMU 缺失、IMU 通路失败、标定缺失、时间戳异常、丢帧。
- 退出码：`0` = 正常（收到 SIGINT → 干净关闭）；`1` = 运行期致命错误；`2` = 参数/配置错误。**launch 不得**用 `respawn=True` 掩盖退出（会在设备忙时变成重启风暴）。

### 7.3 周期性统计日志

每 `log_fps_period_s`（默认 5 s）打一条 `INFO`，逐字包含：`camera {fps:.2f} Hz, published {n_combine} combine / {n_eye} per-eye, dropped {n_drop}, imu {odr:.2f} Hz, published {n_imu}, offset {offset_ns} ns`。`log_fps_period_s<=0` 时关闭。

### 7.4 关闭契约（进程必须保证）

收到 `SIGINT` / `SIGTERM` 时，节点必须在 **3 秒内**完成以下序列，且以退出码 `0` 结束：

1. 停止发布（先取消/停止 timer，再进入关闭）。
2. `Device.stop()` —— 等待 SDK 后台线程退出。
3. `Device.close()` —— `gs130_deinit()` + `gs130_destroy()`，**释放相机独占**（SDK 在 `deinit()` 里复位并断电 MIPI 传感器）。
4. `node.destroy_node()`、`rclpy.shutdown()`。
5. 关闭过程中**禁止**再调用 `read_image()`/`read_imu()`。

实现必须使用 `try/finally`（或 `rclpy.spin` + `KeyboardInterrupt` 捕获）保证 2–4 在所有退出路径（包括异常路径）上执行。**不得**依赖 `Device.__del__` 完成释放（GC 时机不确定）。若 `close()` 本身抛错：`ERROR` 记录，但仍以退出码 `0` 结束（设备可能已被释放；此时打印 `device close failed: {CODE}; the camera may need a power cycle`）。

### 7.5 启动等待（`start()` 不返回时的契约）

`gs130_start()` 在检测到 IMU 时会**先等 IMU FSYNC 握手完成再开流**（`gs130.h` 的 `gs130_start` 注释），SDK **不提供**该等待的超时参数。因此：

- 节点在调用 `Device.start()` **之前**打一条 `INFO`：`starting the camera; waiting for the IMU FSYNC handshake (this can block if the IMU is not producing FSYNC)`。
- 若 `start()` 超过 `start_timeout_s`（默认 `10.0`，节点参数）仍未返回，节点打 `WARN`：`gs130_start has not returned after {start_timeout_s} s; still waiting for the IMU FSYNC handshake`，并且**每隔 `start_timeout_s` 重复一次该 `WARN`**，但**不**中止 `start()`（SDK 无法安全中断该调用；强行 `close()` 会与 SDK 线程竞争，属于未定义行为）。
- 此阶段收到 `SIGINT`：节点**不能**保证 3 秒退出预算；`WARN` 一次 `shutdown requested while gs130_start is blocking; waiting for the handshake`，待 `start()` 返回后立即执行 §7.4 的关闭序列。
- `device:=GS130W`（无 IMU）时不进入握手，`start()` 立即返回。

### 7.6 `/gs130/status`（T8，可选）

`diagnostic_msgs/msg/DiagnosticStatus`，1 Hz：
- `name = "gs130_ros: gs130_node"`，`hardware_id = "GS130 <device> on RDKX5"`
- `level`：`OK`（相机正常）；`WARN`（有丢帧，或 IMU 未发布）；`ERROR`（IMU 通路失败）
- `message`：人类可读的单行摘要
- `values`：键固定为 `camera_fps`、`imu_rate`、`dropped_frames`、`stamp_offset_ns`、`mode`、`width`、`height`、`imu_present`

---

## 8. 兼容性规则（相机独占）

### 8.1 硬事实

GS130 是**独占设备**：同一时刻**只能有一个进程**持有 MIPI CSI 通路与 I2C。`gs130_node` 与 TROS 既有 `mipi_cam` 节点**不能同时运行**。

### 8.2 同时启动时的行为

| 场景 | 观察到的行为 |
|---|---|
| 先起 `gs130_node`，再起 `mipi_cam` | `mipi_cam` 启流失败（无法拿到 VIN/CSI 资源） |
| 先起 `mipi_cam`，再起 `gs130_node` | `gs130_node` 在 `gs130_init()` 阶段失败 → 打 `ERROR`（`§7.1` 的 `HW_ERROR`/`NOT_FOUND` 消息）→ **退出码 1**；节点**不重试、不静默等待** |
| 两者交替启动 | 前一进程正常退出时 SDK 会在 `deinit()` 内复位传感器，后续进程可正常拿到设备 |

### 8.3 文档必须给出的警告（逐字写进用户文档）

> **警告：GS130 相机是独占资源。** 运行 `gs130_node` 时**不要**同时运行 `mipi_cam` 或任何其他访问 MIPI CSI/I2C 的进程（包括 `gs130_sdk` 的 `core/samples` 示例程序）。若 `gs130_node` 因 `HW_ERROR`/`NOT_FOUND` 启动失败，最常见的原因就是另一个进程仍持有相机；请先停止该进程，再重启本节点。**不要在无人值守场景下使用 `respawn=True`**：设备忙时会造成反复重启。
>
> 本文档中的 `/image_combine_raw` → `hobot_codec_republish` → `websocket` 链路是 **GS130 的替代实现**，用于替换 `mipi_cam` 在 TROS web 显示链中的角色；它**不**与 `mipi_cam` 协同，而是**取代**它。

### 8.4 与既有 TROS 节点的兼容性约束（不可修改既有节点）

本契约**明文禁止**修改 `hobot_codec` / `websocket` / `hobot_shm` / `hobot_stereonet` 的任何参数默认值、话题默认值或源码。若需要它们支持新行为，必须回到本文档走评审并升版本。

---

## 9. 冻结声明（FROZEN, DO NOT CHANGE WITHOUT REVIEW）

以下条目**已冻结**。下游实现、测试与用户文档均以此为唯一依据。任何修改都必须走接口评审并同步升版本（`gs130_ros` 包版本 + 本文档版本），且在本文档中新增变更记录。

1. 包名 `gs130_ros`、节点名 `gs130_node`、包版本 `0.1.0`、`package.xml` 字段与依赖表（§1）。
2. 话题名与类型：`/image_combine_raw`、`/image_left_raw`、`/image_right_raw`、`/imu/data`、`/image_left/camera_info`、`/image_right/camera_info`、`/image_combine_jpeg`、`/gs130/status`（§2）。
3. 各话题的 QoS 四项（reliability/durability/history/depth）、`frame_id` 默认值、必需/可选标记（§2）。
4. 编码字符串 `"nv12"`、`height = eh`（真实图像高）、`width`、`step`、`is_bigendian = 0`、`data` 长度等式，以及 T1 的**左目在左、右目在右**拼接布局（§3.1）。
5. `sensor_msgs/Imu` 的逐字段规则：`orientation` 全零 + `w=1`、三个协方差矩阵的 `[0] = -1`、`gyro` rad/s、`accel` m/s²（含重力）、温度**丢弃**、`is_fsync` **不发布**（§3.3）。
6. `sensor_msgs/CameraInfo` 的逐字段规则：`distortion_model` 的 `"plumb_bob"`/`"equidistant"` 映射、`mode=rect` 强制 `"plumb_bob"` + 5 元全 0 `D`、`D` 的截断规则、`R`/`P`/`binning`/`ROI` 全零（§3.4）。
7. 时间戳公式（启动时一次性 `offset_ns`、运行期恒定）与零/不可用时间戳的降级规则（§4）。
8. 全部参数的名字、类型、默认值、取值域与 `Reconfig=No` 语义（§5.1）；非目标参数清单（§5.5）不得被实现。
9. launch 文件名 `gs130_web.launch.py` / `gs130_camera.launch.py`、它们启动的节点集合、既有节点的逐字参数值、以及 §6.6 中的逐字命令（§6）。
10. 错误分级规则：初始化/启动/相机通路失败 → 退出；空队列/IMU 缺失或失败/标定缺失 → 继续（§7.2）；关闭序列与 3 秒退出预算（§7.4）。
11. 相机独占性行为与 §8.3 的警告文本（§8）。
12. 不修改任何既有 TROS 节点（§8.4）。

---

## 附录 A：实现检查清单（供测试对照，不引入新契约）

| # | 检查项 | 期望 |
|---|---|---|
| A1 | `ros2 topic list` 含 `/image_combine_raw`、`/imu/data`、`/image_left/camera_info`、`/image_right/camera_info` | 出现 |
| A2 | `/image_combine_raw` 的 `encoding` | 逐字 `nv12` |
| A3 | `/image_combine_raw` 的 `width x (height/1.5)` | `2*width_eye x height_eye` |
| A4 | `/image_combine_raw` 的 `data` 长度 | `2*ew*eh*3//2` |
| A5 | 左半帧与 `/image_left_raw`（若开启）字节比较 | 与左目 Y/UV 行交错一致（§3.1.3 布局） |
| A6 | `/imu/data` 的 `orientation_covariance[0]` | `-1.0` |
| A7 | `/imu/data` 的 `angular_velocity_covariance[0]`、`linear_acceleration_covariance[0]` | `-1.0` |
| A8 | `/image_left/camera_info` 的 `D` 长度 | `mode=rect` → 5 且全 0；否则 4（fisheye）或 5（pinhole） |
| A9 | `/image_left/camera_info` 的 `R`、`P` | 分别 9 个 0、12 个 0 |
| A10 | 相邻两帧 `header.stamp` 差 | 与设备时间差相等（误差 0 ns） |
| A11 | `use_sim_time:=true` 启动 | `WARN` 出现，节点仍发布 |
| A12 | 与 `mipi_cam` 同时运行 | `gs130_node` 退出码 1，`ERROR` 日志含相机占用提示 |
| A13 | `Ctrl-C` | 3 秒内退出码 0；随后可立即重新启动节点（设备已释放） |
| A14 | `mode:=raw width:=640 height:=480` | 启动参数校验阶段拒绝，退出码 2 |
| A15 | `odr:=100` | 退出码 1，`ERROR` 含 `UNSUPPORTED` |
| A16 | `device:=GS130W` | 无 `/imu/data`，`WARN` 一次，图像正常 |
| A17 | `publish_per_eye:=true` | 出现 `/image_left_raw`、`/image_right_raw`，与 T1 同 `stamp` |

## 附录 B：变更记录

| 版本 | 日期 | 变更 |
|---|---|---|
| v0.1.0 | 首次冻结 | 建立契约：8 个话题（T1/T4/T5/T6 必需、T2/T3/T7/T8 可选或既有链路）、31 个节点参数（其中 `codec_channel`/`websocket_channel`/`web_port`/`web_output_fps`/`jpg_quality`/`smart_topic`/`hobot_shm` 仅由 launch 使用）、2 个 launch 文件（共同声明 18 个 launch 参数） |
