# gs130_ros — GS130 双目相机与 IMU 的 ROS 2 接口

在 RDK X5 上把 `gs130_sdk` 的双目图像、IMU 与标定参数发布为标准 ROS 2 话题，
并直接复用 D-Robotics TROS 已有的 `hobot_codec` 与 `websocket` 节点在浏览器中查看画面。

节点是 **C++（ament_cmake）**，直接 `#include <gs130.h>` 并链接 `libgs130`，不经过 Python。
本包**不实现**相机驱动、编解码、网页或深度算法：这些分别由 `gs130_sdk`、
`hobot_codec`、`websocket` 和 `hobot_stereonet` 提供。

---

## 1. 依赖与构建

| 依赖 | 说明 |
|---|---|
| RDK X5 + TROS humble | `source /opt/tros/humble/setup.bash` |
| `libgs130` 与 `gs130.h` | 随 gs130_sdk 安装（`/usr/include/gs130.h`、`/lib/aarch64-linux-gnu/libgs130.so`） |
| `rclcpp`、`sensor_msgs`、`geometry_msgs`、`tf2_ros` | TROS 自带 |
| `hobot_codec`、`websocket` | TROS 自带，仅 Web 展示时需要 |

构建（不需要安装任何 Python 包）：

```bash
mkdir -p ~/gs130_ws/src && cp -r ros/gs130_ros ~/gs130_ws/src/
source /opt/tros/humble/setup.bash
cd ~/gs130_ws && colcon build --packages-select gs130_ros
source install/setup.bash
```

单元测试（纯函数，不需要相机）：

```bash
./build/gs130_ros/test_conversions      # 13 个用例
```

## 2. 快速开始

### 2.1 在浏览器中看画面

```bash
ros2 launch gs130_ros gs130_web.launch.py
```

然后打开 **http://<板卡IP>:8000/** ，channel 0 会显示左右目拼接的实时画面。
链路是：本节点发布 NV12 到 `/image_combine_raw` → 既有 `hobot_codec` 编码为
`/image_combine_jpeg` → 既有 `websocket` 推送网页。

### 2.2 只发布话题

```bash
ros2 launch gs130_ros gs130_camera.launch.py
ros2 topic hz /image_combine_raw      # 约等于 fps
ros2 run tf2_ros tf2_echo camera_left camera_right   # 双目基线
```

### 2.3 常用参数

```bash
ros2 launch gs130_ros gs130_web.launch.py \
    platform:=RDKX5 device:=GS130WI mode:=resize \
    width:=640 height:=480 fps:=30 odr:=200 stereo_layout:=left_right
```

## 3. 话题

| 话题 | 类型 | 何时出现 | 说明 |
|---|---|---|---|
| `/image_combine_raw` | `sensor_msgs/Image` | `stereo_layout != none`（默认） | 拼接 NV12，`encoding="nv12"` |
| `/image_left_raw` | `sensor_msgs/Image` | `stereo_layout:=none` | 左目 NV12 |
| `/image_right_raw` | `sensor_msgs/Image` | `stereo_layout:=none` | 右目 NV12 |
| `/imu/data` | `sensor_msgs/Imu` | 有 IMU 时 | rad/s 与 m/s²，含重力 |
| `/image_left/camera_info` | `sensor_msgs/CameraInfo` | 总是 | latched，每目一份 |
| `/image_right/camera_info` | `sensor_msgs/CameraInfo` | 总是 | latched |
| `/tf_static` | `tf2_msgs/TFMessage` | `publish_tf:=true`（默认） | `camera_left`→`camera_right`、`camera_left`→`imu_link` |
| `/image_combine_jpeg` | `sensor_msgs/CompressedImage` | 仅 Web launch | 由既有 `hobot_codec` 产生 |

frame_id：拼接图 `camera`，左目 `camera_left`，右目 `camera_right`，IMU `imu_link`。

**这不是 RGB 图。** NV12 是半平面格式，消息里的 `height` 是**真实图像高**，
`data` 则包含完整的 NV12 缓冲（Y 平面 + UV 平面），长度为 `width*height*3/2`。

默认参数（`width:=640 height:=480`）下：

| 话题 | `width` | `height` | `data` 长度 | numpy 形状 |
|---|---|---|---|---|
| `/image_combine_raw`（拼接） | 1280 | 480 | 921600 | `(720, 1280)` |
| `/image_left_raw`（单目，`stereo_layout:=none`） | 640 | 480 | 460800 | `(720, 640)` |

解码与拆目（拼接模式，`width` 为单目宽 640）：

```python
nv12 = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height * 3 // 2, msg.width)
bgr = cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)   # (480, 1280, 3)
left, right = bgr[:, :640], bgr[:, 640:]           # 每目 (480, 640, 3)
```

拼接帧按 `stereo_layout` 排布：`left_right` 为左目在左、右目在右，每目 `width x height`。

## 4. 参数

| 参数 | 默认 | 取值 | 说明 |
|---|---|---|---|
| `platform` | `RDKX5` | 见 SDK | 传给 `gs130.Config.preset` |
| `device` | `GS130WI` | `GS130WI`（带 IMU）/ `GS130W` | 决定是否有 IMU |
| `mode` | `resize` | `raw` / `resize` / `rect` | `raw` 要求 width/height 为 1088x1280 |
| `width` / `height` | `640` / `480` | 正偶数 | 单目输出尺寸；NV12 要求偶数 |
| `fps` | `30` | 1..33 | 相机帧率 |
| `odr` | `200` | `200` / `500` | IMU 输出速率；实测 100 会被 SDK 拒绝，500 实测 505 Hz |
| `stereo_layout` | `left_right` | `none` / `left_right` / `right_left` / `top_bottom` / `bottom_top` | `none` 发布两目，其余发布一帧拼接图；实测 `left_right` 与 `top_bottom` 解码正确。`mode:=raw` 与任何拼接互斥 |
| `frame_id_camera` | `camera_left` | 字符串 | TF 父坐标系 |
| `frame_id_imu` | `imu_link` | 字符串 | IMU 坐标系 |
| `publish_imu` | `true` | 布尔 | 关闭则不发布 IMU |
| `publish_tf` | `true` | 布尔 | 是否发布外参静态 TF |
| `start_timeout_s` | `10.0` | 秒 | 等首帧超时 |

参数在启动时生效，运行期不支持动态重配置：相机流水线由 `gs130_init()` 定死。

## 5. 标定

`camera_info` 由 EEPROM 标定直接映射：

- `K`：3x3 内参，已经在**当前输出分辨率**下（SDK 会随 RESIZE/RECT 回写）。
- `D` 与 `distortion_model`：全零系数 → `plumb_bob`（5 个零，RECT 模式即此）；
  鱼眼 → `equidistant`（4 个系数）；针孔 → `rational_polynomial`（8 个系数）。
- `R` 为单位阵、`P` 由 `K` 重排：SDK 不提供立体校正矩阵，所以不编造，
  单目消费（如 `image_proc`）可直接使用。

外参静态 TF 来自 `gs130.relative_R/T`，例如：

```bash
ros2 run tf2_ros tf2_echo camera_left camera_right   # Translation 约 [0.070, 0, 0] m
```

注意：`mode:=rect` 时 SDK 会把外参变成虚拟的平行双目外参，参考系随之改变。

## 6. 时间戳

SDK 的时间戳是 **CLOCK_MONOTONIC（开机以来）**，不是 Unix 纪元。
节点在首帧到达时计算一次 `offset_ns`，之后恒定换算到系统时钟域，
因此 `header.stamp` 可与其它 ROS 节点比较；一次性采样误差不超过一个帧周期。
图像与 IMU 共用同一 offset，两者时间戳可直接互相比较
（实测 IMU 包时间戳比同批图像晚约 8.6 ms）。

实测补充（900 s）：该 offset 的漂移拟合为 **0.047 ms/min**（约 2.8 ms/小时），
因此运行期不重算；但 IMU 消息的**到达时刻**可能比其时间戳晚最多约 25 ms
（单线程中图像回调的拷贝与发布排在 IMU 回调之前）。
**消费者必须按 `header.stamp` 对齐，不要按到达顺序对齐。**

IMU 消息不包含姿态：SDK 不提供融合姿态，`orientation_covariance[0] = -1`
表示不可用；角速度与加速度有实测值但无方差，协方差保持 0（未知）。

`use_sim_time` 不受支持：设备时间戳是开机以来的单调时钟，无法映射到仿真时间，
节点会直接拒绝并说明原因。

## 7. 故障排查

| 现象 | 原因与处理 |
|---|---|
| 启动即退出，日志 `gs130_init failed` | 相机被占用或不可用。先确认没有 `mipi_cam`、其它 `camera_node` 或残留脚本：`ps -ef \| grep -E "mipi_cam\|camera_node"` |
| **日志里 `frames=` 不再增长、但 `imu=` 还在增长** | **最典型的"相机被别人抢走"症状**。相机流水线不可共享，但第二个打开者**不会报错**，它会直接拿走帧流，先启动的节点静默停帧。节点会在约 10 s 后打印 ERROR 明确提示。处理：停掉多余进程后重启本节点 |
| 网页黑屏，`/image_combine_raw` 有数据 | publisher 与 `hobot_codec` 的 QoS 不匹配。图像必须以 **RELIABLE** 发布，本包已固定如此；若自行改写发布端请保持该设置 |
| 网页黑屏，端口 8000 被占用 | `websocket.launch.py` 用 `os.system` 启动的 nginx 在 launch 退出后仍会存活。清理：`pkill -f webservice/sbin/nginx` 或 `pkill nginx` |
| `mode:=raw` 报错要求 1088x1280 | RAW 模式必须使用传感器原始尺寸，这是 SDK 的硬约束 |
| `mode:=raw` 与拼接同时使用时被拒绝 | RAW 输出必须是 1088x1280，拼接会让宽度翻倍，二者互斥；拼接请用 `mode:=resize` |
| `width`/`height` 被拒绝 | NV12 要求宽高为偶数；另外 `fps` 上限为 33 |
| 图像花屏或上下错位 | 检查读取端是否把 `height` 当成 NV12 打包高度（`H*3/2`）。本包发布的 `height` 是真实高度 |
| Ctrl-C 后相机仍被占用 | 正常关停会打印 `camera released`。若无该日志，用 `ps` 找到进程并 `SIGINT`（不要 `kill -9` 初始化过程中的进程） |

启动失败时的退出码：参数/组合非法为 `2`，相机或运行库不可用为 `1`，
`Ctrl-C` 正常退出为 `0`。

## 8. 已知边界（v0.1.0 刻意不做）

- 不自建网页、编码器、深度网络；深度请用既有 `hobot_stereonet`（见下）。
- 不做零拷贝：SDK 的帧本身来自 `malloc` 缓冲，节点只做一次必要拷贝。
- 不支持运行期动态参数、多相机实例、ROS 1、自定义消息。
- 上下文里的相机独占约束：本包与 `mipi_cam` 不能同时运行。
  实测补充：相机流水线不可共享，但**第二个打开者不会报错**，它会直接拿走帧流，
  先启动的节点静默停帧（IMU 仍在流）。节点有看门狗会在大约 10 s 后给出 ERROR。
- `hobot_stereonet` 需要在一次 `gs130_init()` 里改成 `top_bottom` 拼接
  （实测该布局解码正确），但它的 launch 默认会拉起 `mipi_cam` 与相机独占冲突，
  因此留作后续版本，v0.1.0 不接线。
