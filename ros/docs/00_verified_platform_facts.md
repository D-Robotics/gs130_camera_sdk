# 00 平台事实核验记录（权威，覆盖推测）

本文档由总架构师在 RDK X5 真机上实测得出，**优先级高于任何未实测的推测**。
凡与本文冲突的设计文档，以本文为准；如需推翻，必须给出同等强度的实测证据。

板卡：RDK X5，Ubuntu 22.04，TROS humble（`/opt/tros/humble`），内核 uptime 2:38。
所有实验均未使用相机以外的资源冲突，测试后均已 `stop()` + `close()` 并清理进程。

---

## E1 相机链路 QoS 必须 RELIABLE（实测）

现象：以 `qos_profile_sensor_data`（BEST_EFFORT）发布 `/image_combine_raw` 时，
`hobot_codec_republish` 日志：

```
New publisher discovered on topic '/image_combine_raw', offering incompatible QoS.
No messages will be sent to it. Last incompatible policy: RELIABILITY_QOS_POLICY
```

结论：`hobot_codec` 的订阅端是 **RELIABLE**。我们的发布端必须 RELIABLE，否则一条消息都不会到达。
`hobot_shm` 的 `shm_fastdds.xml` 并未指定 reliability（只设 ASYNCHRONOUS publishMode 与 data_sharing），
因此 reliability 来自节点代码本身，不可通过环境变量规避。

## E2 `sensor_msgs/Image.height` 必须是真实图像高，不是 NV12 打包高（实测，致命）

以 `height = H*3//2`（NV12 缓冲区行数）发布时，codec 把打包高当成图像高：

```
[HobotVenc]: init_pic_w_: 1280, init_pic_h_: 1080, alined_pic_h_: 1088
[ERROR] process has died [hobot_codec_republish, exit code -11]     # 段错误
```

改为 `height = H`（真实高）后链路立即正常。冻结约定：

| 字段 | 值 |
|---|---|
| `encoding` | `"nv12"` |
| `width` | 真实宽（拼接帧为 `2W`） |
| `height` | **真实高** `H`（不是 `H*3//2`） |
| `step` | `width` |
| `data` 长度 | `width * height * 3 // 2` |

SDK 给的是 numpy 视图 `shape == (H*3//2, W)`，因此换算为：`height = shape[0] * 2 // 3`。

## E3 `Image.data` 赋值有 800 倍性能陷阱（实测）

rclpy 生成的 setter（`sensor_msgs/msg/_image.py`）先看 `value.typecode`；
`bytes` 没有 `typecode`，于是落入 `_check_types()` **逐元素 Python 校验**。

实测（1280x720，1.38 MB）：

| 写法 | 耗时 |
|---|---|
| `msg.data = raw_bytes` | **1103.87 ms** |
| `buf = array.array("B"); buf.frombytes(raw); msg.data = buf` | **1.36 ms + 0.023 ms** |
| `msg.data = array.array("B", raw)` | 1.40 ms |
| `frombytes(memoryview(nparray))` | 1.85 ms |

结论：**必须**用 `array.array("B")` + `frombytes` 构造 data，禁止直接赋 `bytes`。
`tobytes()` 本身只有 0.31 ms，慢的全部是 setter 校验。

## E4 完整链路在 30 fps 端到端跑通（实测）

合成 NV12（1280x720）以 30 Hz 发布 → 既有 `hobot_codec` → 既有 `websocket`：

```
/image_combine_raw   average rate: 29.966   (min 0.024s max 0.043s, window 215)
/image_combine_jpeg  average rate: 29.974   (min 0.026s max 0.042s, window 214)
codec log:           Sub imgRaw fps [29.98] / Pub img fps [29.98]
node timing:         build 0.53 ms | fill 1.28 ms | publish 5.07 ms
```

Web UI：`curl http://127.0.0.1:8000/` → HTTP 200。nginx 由 `websocket.launch.py` 以
`os.system` 启动，launch 退出后 **nginx 仍存活**（孤儿进程），需显式停止。

## E5 SDK 时间戳是 CLOCK_MONOTONIC（开机以来），非 Unix 纪元（实测）

真机 SDK 读取 vs 系统时钟：

```
frame_ts_ns        9546841822000
imu_ts_ns          9546850424616
time.time_ns()     1789394612599936943
monotonic_ns()      9540690105753
frame - monotonic          6151716247
frame - wall       -1789385065758114943
imu - frame                  8602616
```

`monotonic_ns()` 与 uptime（2:38 = 9480 s）一致；`frame_ts` 与 monotonic 同量级，
与 wall clock 相差约 1.79e18 ns。结论：

1. 图像与 IMU 时间戳**同处 CLOCK_MONOTONIC 域**，可直接互相比较（实测 `imu - frame = 8.6 ms`）。
2. **不能**把原始 `timestamp_ns` 直接写进 `header.stamp`：其它 ROS 节点使用系统时钟，
   直接写入会静默破坏 `tf` / `message_filters` / `rosbag`。
3. 正确做法：启动时一次性求 `offset_ns = ros_clock.now() - sdk_ts`，运行期恒定换算，
   从而既落到系统时钟域，又保留曝光间隔与 IMU/图像对齐关系。
   单次采样误差 ≤ 一个帧周期，必须写入文档。

## E6 硬件拼接可用，无需自行拼接（实测）

`Config.preset()` 之后设置 `config.camera_config.stereo_layout = StereoLayout.LEFT_RIGHT`，
真机得到 `read_image()["stitched"]`，shape `(720, 1280)`（真实图 1280x480）。
在 RESIZE 模式下按 `width=2W, height=H` 正好满足 E2 的 `/image_combine_raw` 约定。
因此 **不需要在 Python 侧手工 concat 左右目**，直接使用硬件拼接即可。

注意：RAW 模式要求输出尺寸等于传感器 1088x1280，与拼接宽度翻倍冲突，
因此拼接帧应使用 RESIZE（或 RECT）模式。

## E8 相机流水线不可共享，且第二个打开者不会失败（实测，重要）

同时运行两个持有相机的进程（两个 `camera_node`）时：

```
第一个实例：frames 增长到 391 后**冻结**，imu 继续增长 3051 → 7086（约 200 Hz）
第二个实例：frames 正常增长 154 → 604（30 fps），imu = 0
```

结论：

1. `gs130_init()` **不会**因为相机已被占用而报错，两个实例都能成功初始化。
2. 但帧只归其中一个：后打开者拿走帧流，先打开者**静默停帧**（IMU 仍在流）。
3. 第二实例收不到 IMU 包（`imu_name` 为 None），因此节点日志中不会出现 `imu_link` 的静态 TF。
4. 因此"相机被占用"的典型表现是**静默停帧**，而不是初始化失败；
   仅凭 `gs130_init` 失败来判断占用是不可靠的。

由此在节点中加入了看门狗：连续两次（约 10 s）帧计数不增长而 IMU 仍在增长时，
打印 ERROR 指明很可能有其它进程占用了相机。

## E9 `top_bottom` / `bottom_top` 拼接布局解码正确（实测，推翻评审推测）

以 `stereo_layout:=top_bottom`、640x480 运行，抓帧解码：

```
width 640 height 960 decoded (640, 960, 3)
```

目视该 PNG：上、下两幅为同一场景的左右目视图，颜色正常、边缘清晰、
两半之间存在符合视差的位移。**没有**出现评审猜测的"分平面串联导致彩色错乱"。

同时确认消息几何自洽：SDK 输出的拼接帧 numpy 形状为 `(1440, 640)`，
节点推导 `height = 1440*2//3 = 960`、`width = 640`、`len(data) = 640*960*3//2 = 921600`，
与实际缓冲一致。

因此 `top_bottom`/`bottom_top` 在 v0.1.0 中**保留可用**，
`hobot_stereonet` 需要的正是这类上下堆叠布局（见 `32_integration_expectations.md`）。

## E10 参数校验矩阵（实测）

| 输入 | 结果 |
|---|---|
| `mode:=raw width:=1088 height:=1280 stereo_layout:=left_right` | FATAL 说明 raw 与拼接互斥，退出码 2 |
| `mode:=raw width:=640 height:=480` | FATAL 说明 raw 需要 1088x1280，退出码 2 |
| `width:=640 height:=479`（奇数） | FATAL 说明 NV12 需要偶数，退出码 2 |
| `platform:=RDKX6` | FATAL `unsupported platform/device`，退出码 2 |
| `fps:=120` | FATAL 超出上限 33，退出码 2 |
| `mode:=rgb` | FATAL 列出合法取值，退出码 2 |
| `width:=abc` | launch 期报错，节点未启动（未触碰相机） |
| `publish_imu:=false` | `/imu/data` 话题不存在 |

所有这些失败都发生在**相机被打开之前或立即关闭**，测试结束后无残留进程。


## E7 其它实测约束

- `CameraMode.RAW` 必须 `output_width/height == 1088x1280`，否则 `gs130_init` 返回 PARAM_ERROR。
- 相机独占：任何时刻仅一个进程可持有；`mipi_cam`、`gs130` 脚本、本 ROS 节点互斥。
- `websocket` 用 `channel` 区分多路（既有 132GS launch 用 channel 0/1 复用同一 8000 端口），
  不需要起第二个端口。
- 板端已装：colcon、rclpy、sensor_msgs、geometry_msgs、tf2_msgs、OpenCV 4.11.0、numpy 1.26.4。

---

## E11 C++ 节点实测（`libgs130` C API + rclcpp）

`ros/gs130_ros` 已从 rclpy 改写为 ament_cmake 的 C++ 节点，直接 `#include <gs130.h>`
并链接 `-lgs130`。实测：

```
gtest（无硬件）              13/13 通过
640x480@30 启动              calibration / static tf baseline 0.070316 m / offset 正常
20 s 计数                    frames=600 imu=4049   （30 fps / 202 Hz）
1080p soak 12 min            frames=21450 (30.0 fps) imu=144484 (202 Hz) RSS 持平
端到端 web 链路              /image_combine_raw 29.871 Hz，/image_combine_jpeg 29.750 Hz
QoS offer 侧                 RELIABLE；codec 的 "offering incompatible QoS" 计数 = 0
网页                         HTTP 200；抓帧 nx12 1280x480，左右目像素差 60.41
SIGINT                       "camera released" + "process has finished cleanly"，无残留
```

## E12 C 的 preset 宏无法在 C++ 中使用（实测，构建约束）

`gs130_define.h` 的 `GS130_CONFIG_*` 展开后使用 GNU 区间指定器（`[0 ... 3] = 0xFF`），
**C++ 任何标准模式下都无法编译**（`gnu++17` 与 `gnu++20` 均报
`expected identifier before numeric constant`）；而且它展开成花括号初始化器，
只能用于**初始化**，不能赋值。

因此本仓库用一个 C 翻译单元 `ros/gs130_ros/src/preset.c` 包一层：

```c
const gs130_config_t cfg = GS130_CONFIG_RDKX5_GS130WI(camera_mode, w, h, fps, odr);
*out = cfg;
```

好处是 preset 的数值仍然只定义在 SDK 头文件里，不复制到 C++；并且这一层
在未知 platform/device 时返回 -1 而**不会像宏那样 `exit(1)`**。
实测：`GS130WI` → 1088x1280、mipi[4]=2、gpio[4]=351、imu_fifo=1024；
`GS130W` → right_addr=0x31、imu bus_num=0、imu_fifo=0；未知平台 → -1。

## E13 IMU 的 odr 只有 200 与 500 可用（实测）

| odr | 结果 |
|---|---|
| 100 | `gs130_init()` 返回 `GS130_UNSUPPORTED`，节点启动失败 |
| 200 | 正常，节点内部计数 202 Hz |
| 500 | 正常，节点内部计数 505 Hz |

设备的 `gs130_get_imu_info()` 也自述 `odr: 200 | 500 Hz`。
节点已把 odr 校验为 {200, 500} 并在拒绝时给出可读信息。

注意：`ros2 topic hz /imu/data` 在 odr=200 时只报约 150 Hz，**与节点内部计数不符**；
CLI 订阅端跟不上 200–500 Hz 的小消息。速率判定必须用节点计数或 C++ 订阅端。

## E14 抢流时 IMU 也会失败（实测，补充 E8）

第二个持有者抢流后，先到者不只是图像停：实测其 IMU 计数先冻结，
随后 `gs130_get_imu_packet()` 连续返回 `GS130_HW_ERROR(4)`；
而抢流者拿到帧却拿不到 IMU（`imu=0`）。看门狗在此场景下准确触发一次。

## E15 offset 漂移与 IMU 投递抖动（实测，900 s）

以 IMU 话题（与图像共用同一 offset，消息极小）测量 `接收时刻 − header.stamp`，
1 Hz 采样 900 s：

```
latency ms: first -0.12  last -17.20  min -23.64  max 1.78
fitted slope: 0.047 ms/min        -> 约 2.8 ms/小时
```

结论：

1. **一次性 offset 成立**：拟合漂移 0.047 ms/min，12 分钟会话累计不到 1 ms。
   不要用首末差值（−17 ms）当漂移，那主要是投递抖动；判据必须用拟合斜率。
2. **IMU 投递延迟抖动约 25 ms**：单线程 executor 中，一次图像回调要拷贝并发布
   6.2 MB（1080p 时），IMU 回调排在其后。这不影响 `header.stamp` 的正确性
   （stamp 来自设备时间戳 + 常量 offset），但消费者必须按 stamp 对齐，
   不能按到达顺序对齐。
3. 同一会话内节点保持健康：frames=27450（30 fps）、imu=184899（≈202 Hz），
   SIGINT 后干净释放。

## 由实测得出的冻结决策

| 编号 | 决策 | 依据 |
|---|---|---|
| D1 | 图像发布 QoS = RELIABLE | E1 |
| D2 | `Image.height = shape[0]*2//3`，`step = width` | E2 |
| D3 | `data` 用 `array.array("B").frombytes()` 构造 | E3 |
| D4 | 复用 `hobot_codec` + `websocket`，不写自研 codec/网页 | E4 |
| D5 | `header.stamp` = SDK 时间戳 + 启动时一次性 offset（系统时钟域） | E5 |
| D6 | 拼接帧使用 SDK 硬件拼接（`StereoLayout.LEFT_RIGHT`），不手工拼 | E6 |
| D7 | 默认 `mode=resize`；`raw` 仅用于标定导出 | E6/E7 |
| D8 | launch 退出后必须显式清理 nginx | E4 |
| D9 | 用看门狗把"静默停帧"变成可诊断错误 | E8 |
| D10 | `raw` 与 `stereo_layout != none` 互斥，启动期拒绝 | E6/E7/E10 |
| D11 | `width`/`height` 必须为偶数，`fps <= 33`，启动期校验 | E10 |
| D12 | `top_bottom`/`bottom_top` 保留可用（实测解码正确） | E9 |
