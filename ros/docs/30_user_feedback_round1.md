# GS130 ROS 2 (TROS) 接口包 —— 用户评审第一轮反馈

- 评审人：UX-1（用户评审组，代表 RDK X5 + GS130 的 ROS 2 应用开发者）
- 评审对象：`gs130_sdk` 计划中的 ROS 2 (TROS) 接口包（尚未实现）
- 评审时机：实现前，目的是让反馈仍然能改变设计
- 依据：`python/gs130/__init__.py`、`python/gs130/_device.py`、`python/gs130/_config.py`、`python/gs130/_types.py`、`python/gs130/_enums.py`、`python/test/test_gs130.py`、`ros/probe_nv12_publisher.py`，以及已验证的板端事实（TROS humble、hobot_codec / websocket / hobot_shm / hobot_stereonet / hobot_usb_cam / mipi_cam 均已存在）
- 立场：我是要把 GS130 用进真实项目的人，不是来验收 demo 的。下面按"我会怎么用"而不是"你们想怎么做"来写。

---

## 1. 我作为 ROS 用户的五条最重要期望

### E1（mandatory）零新增节点地接入现有 TROS 显示链路

我期望这个包只提供"数据源"（一个或两个 publisher 节点 + launch + 参数），图像 topic 的 encoding、尺寸语义、`step`、`frame_id` 都做成 `hobot_codec` 能直接吃进去的形式，最差也只是改一个 remap 或一个参数，就能出现在 websocket 节点的 channel 上。
之所以重要：D-Robotics 那条已验证链路（NV12 → `hobot_codec` → jpeg → websocket/web UI）是我唯一能确定能跑通的路。如果接上 GS130 需要我再写一个中转节点、自己拼 NV12、自己管 buffer 生命周期，那这个包的收益就被吃掉了——我直接改 `mipi_cam` 参数更省事。

### E2（mandatory）独占冲突必须显式失败，并且失败信息要指向真实原因

我期望：当 `mipi_cam` 或别的进程已经占着相机时，节点在启动阶段就明确报错退出（非零退出码 + 一条能读懂的错误，附带底层 `ErrorCode`），而不是卡在"等待第一帧"、也不是无限重试、更不是打印一堆无关 log。
之所以重要：GS130 是独占设备。我调不出来的第一个夜晚，99% 的原因是"我忘了上一条 launch 还活着"或"mipi_cam 还在跑"。把这条做成显式诊断，能省掉我几小时；做成静默挂起，我会以为是驱动或权限问题，然后开始翻 `dmesg` 和 `/dev`。

### E3（mandatory）IMU 以独立于相机帧率的速率发布，且时间戳可用

我期望 IMU 有独立的 topic、独立的发布节奏（一个独立线程/定时器），不要"每来一帧图像才发一次 IMU"；并且 `header.stamp` 的取值规则被明确写下来（硬件时间戳还是节点时钟、如何换算），`frame_id` 固定且可参数化。
之所以重要：我要拿 IMU 做 VIO / EKF。看到 `odr_hz=200` 就以为拿到 200 Hz，结果实际只在相机帧边界批量到货，这是会直接毁掉融合效果的设计缺陷。时间戳来源如果不写清楚，我丢进 `message_filters` 或 TF 时会得到静默的错位结果，比报错更难查。

### E4（mandatory）标定必须能直接变成 `camera_info` + 静态 TF，且诚实地表达鱼眼

我期望左右目各有标准 `camera_info`（`K`、`D`、`R`、`P`、`distortion_model`、`width`/`height`、`header.frame_id`/`camera_name` 一致），畸变模型能如实反映 fisheye，而不是一律写成 `plumb_bob`；并且提供一种方式把 `calibration()` 里的左/右/IMU 外参变成静态 TF（至少是可选、默认关闭）。
之所以重要：我拿这套东西的第一个真实用途就是 rectification 和 stereo depth。只有图像没有 `camera_info`，`hobot_stereonet` 和任何 ROS 双目工具链都用不了；`D` 是 8 个系数但标成 `plumb_bob`，会让下游库按错误模型解码，产出的深度图是"看起来能跑但是错的"——这是最坏的一类 bug。

### E5（mandatory）生命周期干净，且 QoS/丢弃策略对我可见、可调

我期望 `Ctrl-C` / SIGTERM 之后节点能真正释放相机（短时间内可以立刻重启），异常退出后不需要重启整机就能恢复；`SIGKILL` 之后的行为要写在文档里。同时图像 topic 的 QoS 和丢帧策略（对应 SDK 的 FIFO `DROP_OLD` / `DROP_NEW` 和 `depth`）要有参数可配，默认值要跟 `hobot_codec` 的订阅端对得上。
之所以重要：调参阶段我一天会重启节点几十次。如果每次崩溃都要 `reboot` 才能拿回相机，这个包在真实项目里不可用。QoS 不匹配则表现为"什么都不显示"且没有任何报错，是典型的浪费时间故障。

### 补充期望（nice-to-have，不作为 v0.1.0 门槛）

- **N1（nice-to-have）** 我期望 `stitched`（拼接）输出作为一个可选开关存在，并明确"横向拼接宽度翻倍 / 纵向拼接高度翻倍"的语义，这样我可以在一个 channel 里同时看到左右目。
- **N2（nice-to-have）** 我期望提供 `imu_only` 这类模式（不占相机、只发 IMU）的可能性——如果不做，请直接说"不支持"，不要留一个半残的开关。
- **N3（nice-to-have）** 我期望提供零拷贝路径（`hobot_shm`）的选项，或至少说明为什么 v0.1.0 不需要它。

---

## 2. 前五分钟我会做什么，以及我会在哪里卡住

### 我的第一猜测（在没有文档的情况下我会敲的命令）

```bash
# 1) 确认包在不在
ros2 pkg list | grep -i gs130

# 2) 看能不能直接起
source /opt/tros/humble/setup.bash
ros2 launch gs130_ros gs130.launch.py
# 或者我猜的名字：
#   ros2 launch gs130_camera gs130_camera.launch.py
#   ros2 launch gs130_bringup stereo.launch.py

# 3) 看 topic 出来了没有
ros2 topic list
ros2 topic hz /image_left_raw
ros2 topic echo /imu/data --once

# 4) 起显示链路（我会照着 D-Robotics 的例子猜 remap）
ros2 run hobot_codec hobot_codec_republish \
  --ros-args -p channel:=0 \
  -p sub_topic:=/image_left_raw \
  -p pub_topic:=/image_left_jpeg \
  -p in_format:=nv12 -p out_format:=jpeg

# 5) 打开浏览器 http://<board-ip>:8000，看 channel 0
```

### 我期望看到什么

- `ros2 topic list` 里有图像 topic、对应的 `camera_info`、以及 IMU topic，名字不需要我查文档就能猜到。
- `ros2 topic hz` 报出接近参数里 fps 的稳定频率，而不是 0.5 Hz 或一堆 `NaN`。
- 浏览器里出现画面；两路图像时间戳差在亚毫秒到毫秒量级。
- 节点日志第一行就把关键配置回显出来（device/mode/width/height/fps/imu_odr/topic 名/QoS）。

### 设计草率时我会卡在哪（按真实概率排序）

1. **topic 名不可猜 + 文档没写全**。`ros2 topic list` 出来一堆 `/gs130_wi_left_image_nv12_raw` 之类的名字，我 ping-pong 于文档和终端之间。→ 我改回 `mipi_cam`。
2. **静默无输出**。节点起来了、退出码 0、没有任何错误，就是没有帧。可能是相机被占、可能是 `hobot_codec` 的 `sub_topic` 写错、可能是 QoS 不匹配、也可能是 encoding 字符串对不上（`nv12` vs `NV12` vs `yuv420`）。这四种原因在现象上完全一样。→ 我会先怀疑这个包，然后放弃。
3. **NV12 打包方式与 `hobot_codec` 期望的不一致**。SDK 给的是"tightly-packed NV12：Y 平面 `width*height`，然后 UV 平面 `width*height/2`"，numpy 侧是 `(height*3//2, width)`。如果 ROS 侧填 `height=1088`（而不是 1088*3/2=1632）或者 `step` 填错，画面会是撕裂/花屏/只有绿灰块。我很难从现象反推是接口填错了。→ 我会认为是 SDK 的 bug。
4. **时间戳对不上**。SDK 的帧时间戳优先取曝光触发时刻（LPWM 上升沿），是相机侧时间基准；IMU 时间戳是"对齐到相机时钟的校正时间戳"。如果节点把它们直接塞进 `header.stamp` 而不做换算，`ros2 topic delay`、TF、`message_filters` 全都会给出无意义的数字，而画面看起来完全正常——我会先怀疑自己的订阅端。
5. **独占错误被当成未知错误**。相机被占时报一个 `HW_ERROR` 或干脆超时，没有告诉我"设备忙，检查是否已有进程占用"。→ 我会去查驱动。
6. **IMU 速率与期望不符**。参数写 `imu_odr:=200`，`ros2 topic hz /imu/data` 出来 30（跟 fps 走）。如果文档没提前说明，我会以为是我的参数没生效。

### 什么会让我直接放弃、改用 `mipi_cam`

明确列出（这是承诺门槛，不是抱怨）：

- 我按文档抄的命令**跑不出画面**，或者需要我自己写胶水节点才能进 web UI。
- 没有可用的 `camera_info`；或者 `camera_info` 的畸变模型与真实标定不符（fisheye 被写成 `plumb_bob`）。
- 相机独占冲突时静默挂起，或每次异常退出都要 `reboot` 才能恢复。
- IMU 只在相机帧边界发布，且没有独立速率选项。
- 参数需要"改代码 / 改 launch 里的硬编码路径（比如某个 tuning 文件的绝对路径）"才能换分辨率或 fps。
- 文档缺失到我要读 C 头文件才能知道 `step` 怎么填。

---

## 3. 命名与接口反馈

### 3.1 跟随 D-Robotics 命名：帮助大于困惑，但必须写清"别名"

**结论：帮助我，前提是文档里有一张"你们的名字 ↔ D-Robotics 例子的名字"对照表。**

理由：我的心智模型、我的 launch 文件、我抄的教程都来自 D-Robotics 的 stereo 例子（`/image_combine_raw` → `/image_combine_jpeg` → web UI channel 0）。名字一致意味着我可以直接复用现有 launch、`hobot_codec` 参数和 websocket 配置，只把"谁发布"换掉。名字不一致则每一次集成都变成翻译工作。

但请注意一个真实的困惑源：D-Robotics 的通用 web UI 链路里 `hobot_codec` 和 websocket 的参数名/通道语义并不总是显式写在例子里。所以：

- 如果你们采用 `/image_combine_raw`，请在文档里明确它是一个**已存在的、被 D-Robotics 例子占用的名字**，以及和 `mipi_cam` 同时运行会发生什么。
- 如果你们采用自己的命名空间（见 3.2），请在文档里给出**最小 remap 片段**，让 `hobot_codec` 的 `sub_topic` 指向你们的名字。这一条比名字本身更重要。

### 3.2 我期望的 topic 命名

我期望有一个可参数化的命名空间 `ns`（默认 `gs130`），并且 topic 名可被 launch 覆盖（`remappings`）。在此前提下，我期望的默认布局：

| 用途 | 我期望的名字 | 类型 | 说明 |
| --- | --- | --- | --- |
| 左目 | `/<ns>/camera_left/image_raw` | `sensor_msgs/Image` | `encoding: nv12` |
| 左目标定 | `/<ns>/camera_left/camera_info` | `sensor_msgs/CameraInfo` | 与图像同 `frame_id` |
| 右目 | `/<ns>/camera_right/image_raw` | `sensor_msgs/Image` | `encoding: nv12` |
| 右目标定 | `/<ns>/camera_right/camera_info` | `sensor_msgs/CameraInfo` | |
| 拼接（可选，N1） | `/<ns>/image_combine_raw` | `sensor_msgs/Image` | 兼容 D-Robotics 链路；语义见下 |
| IMU | `/<ns>/imu/data` | `sensor_msgs/Imu` | 六轴，见 3.3 |
| 温度（可选） | `/<ns>/imu/temperature` | `sensor_msgs/Temperature` | 可选；不做就删掉这一行 |

命名规则意见：

- **不要**把 `left` / `right` 做成 topic 后缀（`/image_raw_left`），也不要用中划线和大小写混排。理由：`camera_info` 必须与图像 topic **同目录同基名**，才能被 `image_proc`、`image_transport`、`camera_calibration`、`depth_image_proc` 这套标准工具自动配对。`/camera_left/image_raw` + `/camera_left/camera_info` 是 ROS 世界里的默认约定，遵循它我的工具链就零配置可用。
- 拼接 topic 的语义必须在文档里写死：横向拼接（`LEFT_RIGHT`/`RIGHT_LEFT`）→ `width` 翻倍、`height` 不变；纵向拼接（`TOP_BOTTOM`/`BOTTOM_TOP`）→ `height` 翻倍、`width` 不变；且 `step` 与打包方式要被明确写出。这条正是"画面看起来能显示但几何全错"的高危点。
- 如果 v0.1.0 只做一种布局（我猜是 `NONE`，即左右目分开），那就**不要**宣称支持 `stitched`；半个功能比没有更糟。

### 3.3 IMU topic

- 一个 `sensor_msgs/Imu` topic，只填 `linear_acceleration`、`angular_velocity`，以及可填的 `orientation_covariance[0] = -1`（表示"本设备不提供姿态"）。**不要**伪造 `orientation`。
- **不要**按 REP 145 之外的私人约定拆成 `/imu/accel` 和 `/imu/gyro` 两个非标准 topic；也不要为了"标准"假装有磁力计（ICM-42688-P 没有）。如果你们坚持拆，请用 `sensor_msgs/Imu` 一个 topic + 文档说明。
- covariance：v0.1.0 可以是全 0 或明确的 `-1` 约定，但**必须**在文档里写清楚你们填了什么，以及 `imu_intrinsics()` 里的 noise / random walk 是否被用上了。我更希望噪声参数以参数形式暴露、由节点填进 covariance，而不是让我自己再去调 API。
- `frame_id`：默认一个固定值（例如 `imu_link`），可参数化；并且**必须**在文档里给出它相对于左右相机光学坐标系的静态 TF（`calibration().imu_R` / `imu_T` 是相机系到 IMU 的外参）。否则我的融合结果再好也无法上 TF 树。

### 3.4 参数命名与默认值

我期望参数名与 SDK 的 `Config.preset(platform, device, mode, width, height, fps, odr)` 语义一一对应，用 snake_case，并且**默认值等于能开箱出画面的那组**（即相当于 `RDKX5 GS130WI` 的预设）：

| 参数 | 类型 | 我期望的默认 | 备注 |
| --- | --- | --- | --- |
| `platform` | string | `RDKX5` | 不支持的值必须报错，不要静默 fallback |
| `device` | string | `GS130WI` | 与 `GS130W`（无 IMU）区分；无 IMU 时 IMU topic 不出现且日志说明 |
| `mode` | string | `resize` | `raw` / `resize` / `rect`；`raw` 时宽高必须等于 1088x1280，否则报错 |
| `width`, `height` | int | 与 `mode` 匹配的一组已知可用值 | 参数表要列出**受支持的组合**，不要只写范围 |
| `fps` | int | 30 | 要说明可选值与 `line_length`/`frame_length` 的关系 |
| `imu_odr` | int | 200 | 同时说明**实际发布速率**（见 4.3） |
| `ns`（命名空间） | string | `gs130` | |
| `publish_stitched` | bool | `false` | N1 |
| `image_qos_reliability` / `image_qos_depth` | string/int | 与 `hobot_codec` 订阅端匹配 | 默认值必须在文档里注明"为什么是这个" |
| `fifo_depth` / `fifo_mode` | int/string | 跟随预设（相机 `DROP_OLD`） | 用 ROS 侧丢帧策略表达 |
| `frame_id_left` / `frame_id_right` / `frame_id_imu` | string | `camera_left_optical` / `camera_right_optical` / `imu_link` | |
| `publish_static_tf` | bool | `false` | 由标定 R/T 生成 |
| `timestamp_source` | string | 见 4.4 | `hardware` / `node_clock` / `hardware_with_offset` |

原则：

- **不启动流就需要的参数**（`platform`/`device`/`mode`/`width`/`height`/`fps`/`imu_odr`）在 v0.1.0 里允许"只能启动时设置"。但必须文档化"改了要重启节点"，并且**不要**给我一个看起来能动态改、实际静默无效的参数。
- **不允许**出现需要绝对路径的参数（例如 tuning 文件）。如果预设里必须带路径，请在包里随文件分发并写相对解析规则。
- 参数默认值不要依赖 `~/.bashrc`、环境变量或 `install/` 下的相对路径。

---

## 4. 对现有 Python API 的诚实评审（ROS 视角）

先说好消息：这个 API 在 ROS 侧的"形状"其实相当合适，因为它的数据模型本身就是"同步的左右目 NV12 + 一个 IMU packet + 一份标定"，这正是 ROS 消息能直接承载的东西；`Config.preset()` 也几乎就是一份 launch 参数清单；`Image` 是 `np.ndarray` 子类且带 `timestamp_ns`，零拷贝，这对做 ROS 发布是有利的（`data` 可以直接 `tobytes()` 或走共享内存）。

下面逐条说会让 ROS 用户困惑的地方，以及 ROS 侧必须做什么来消除它。

### 4.1 `read_image()` 返回 dict，而不是固定的对象

- 现象：`{"left": Image, "right": Image}` 或 `{"stitched": Image}`，取决于 `Config.preset()` 里 `stereo_layout` 的取值；超时返回 `None`。
- 为什么 ROS 用户会困惑：ROS 侧的订阅者永远不知道下一帧是哪种形态；键名集合是运行期决定的、没有类型标注、IDE 不提示。我会写出 `images["left"]` 然后在拼接模式下拿到 `KeyError`。
- ROS 侧文档/接口必须做的：把两种形态**拆成两个明确的节点或两个明确的参数模式**（例如 `stereo_layout:=none` 时发布左右目两路；`stereo_layout:=left_right` 时只发布 `image_combine_raw`），并在文档里给出「模式 → topic 集合」的对照表。绝不要让同一个 topic 在不同模式下改变语义。另外必须把 `None`（超时）的行为写成规范：ROS 节点内部应重试而不是发布空帧，并且把超时计为可观测的统计（日志/MHz）。

### 4.2 NV12 的 numpy 形状与 ROS `Image` 字段的对应

- 现象：`Image` 是 `(height * 3 // 2, width)` 的 `uint8` 视图，内存布局是"Y 平面 `width*height` + UV 平面 `width*height/2`，紧密打包"，`timestamp_ns` 挂在数组属性上。
- 为什么 ROS 用户会困惑：ROS `sensor_msgs/Image` 里，NV12 类编码的**事实约定**是 `height = 行高`、`step = width`，数据长度 `= step * height = width * height * 3/2`，即 `height` 是"总行数"。但很多人（包括我）第一反应会写 `message.height = 1088`。此外 `step` 在 NV12 里表示的是"Y 平面一行字节数"，填错会静默花屏。附带一个问题：`Image` 的 `timestamp_ns` 是 ndarray 的属性，任何 `np.copy` / 切片之外的操作都会丢，ROS 侧代码很容易顺手丢掉时间戳而不自知。
- ROS 侧文档/接口必须做的：在文档里给出一段**可直接抄的 10 行 NV12 → `sensor_msgs/Image` 填充示例**，明确写出 `encoding="nv12"`、`height=frame.shape[0]`、`width=frame.shape[1]`、`step=frame.shape[1]`、`is_bigendian=0`，并显式声明 `data` 是紧密打包（无 padding）。同时写明"`timestamp_ns` 必须在任何拷贝/转换之前取走"。这一条比任何解释性文字都省时间。
- 附加建议：`encoding` 字符串的大小写必须与 `hobot_codec` 的 `in_format` 期望一致，并在文档里给出该字符串的**字面量**。

### 4.3 IMU packet 每个相机帧才到货一次（最严重的一条）

- 现象：从源码看，IMU 只有在收到带 FSYNC 的包（`is_fsync`）时才把缓存的样本配对成 `gs130_imu_packet_t` 并推入 `imu_fifo`："pair only after an FSYNC anchor"。而 FSYNC 绑定在相机上（`fsync_camera`，预设为 RIGHT），也就是说**可读的 IMU 输出节奏被相机帧率门控**。`read_imu()` 每次返回一个 `ImuPacket`，但它的到货是成批的、以帧为节拍的。
- 为什么 ROS 用户会困惑：`ImuConfig.odr_hz=200` 会让我认为 `sensor_msgs/Imu` 会以 200 Hz 到达。如果节点简单地"每帧发一个 IMU"，我会得到约等于 fps 的低频 IMU，融合效果直接不达标；如果我按 200 Hz 轮询 `read_imu()`，我会拿到 `None` 和突发的批量包，时间戳还是历史值。
- ROS 侧文档/接口必须做的：
  1. 文档里写清 **IMU 的实际输出速率上界与相机 fps 的关系**，以及 `imu_odr_hz` 究竟控制什么。
  2. 节点实现上要给 IMU 一条**独立于图像的回调/线程路径**，并按每个 packet 自己的 `timestamp_ns` 设置 `header.stamp`；不能在图像回调里"顺便发一条 IMU"。
  3. 文档要给出**背压策略**：一帧边界上批量到达多个 packet 时，是全部发出（保留真实速率和时间戳）还是丢弃旧样本（保实时）。我倾向"全部按序发出"，但这必须是一个被记录的决定，而不是实现副作用。
  4. `is_fsync` 的含义（这一包是 FSYNC 同步包）必须在文档里解释，并说明它**不**映射到任何 ROS 字段；同时说明 FSYNC 绑定在右目（`fsync_camera=RIGHT`）这一事实对左右目时间戳关系的影响。

### 4.4 时间戳是纳秒整数，但**时钟域未定义**（高危）

- 现象：图像 `timestamp_ns` 优先取曝光触发时刻（`trig_tv`），退回 `timestamps` 或 `tv`；IMU 的 `timestamp_ns` 被注释为"corrected absolute timestamp (aligned to the camera clock)"。两边都是 uint64 纳秒，但**都没有说明基准是系统实时钟（CLOCK_REALTIME）还是单调钟（CLOCK_MONOTONIC）**。
- 为什么 ROS 用户会困惑：ROS 的 `header.stamp` 是 `builtin_interfaces/Time`，默认被当作实时钟。如果我把相机时钟域的值直接写进 `header.stamp`，会出现：`ros2 topic delay` 报出数十年的延迟、TF 查询在"未来"或"过去"失败、`message_filters` 永远配不上对——而**画面完全正常**，所以我不会怀疑是时间戳的问题。
- ROS 侧文档/接口必须做的：
  1. 明确写出两个 `timestamp_ns` 的时钟域，以及它们是否同源。
  2. 提供 `timestamp_source` 参数，至少支持 `node_clock`（用 `get_clock().now()`）与"硬件时间戳 + 可配置 offset"，并把默认值、以及为什么是它，写进文档。
  3. 文档里给出验证手段：我用什么命令/什么方法能自己确认两边时间戳在同一时钟域（例如同时订阅 image 与 imu 后比较 `header.stamp` 与本地时钟差是否稳定）。
  4. 如果无法做时钟域换算，就**明确说不能**，并让 `node_clock` 成为默认，这比给我一个看起来更精确但含义不明的硬件时间戳要好。

### 4.5 没有融合后的姿态（orientation）

- 现象：SDK 只提供 `accel` / `gyro` / `temp` / `is_fsync` / `timestamp_ns`，以及 `ImuIntrinsics`（misalign / scale / bias / noise / random walk）。没有姿态输出。
- 为什么 ROS 用户会困惑：看到 `sensor_msgs/Imu`，很多人默认 `orientation` 有效。
- ROS 侧必须做的：文档里明确"本包不提供 fused orientation，`orientation_covariance[0] = -1`"，并说明 `ImuIntrinsics` 的噪声参数是否可以（或如何）被节点写进 `linear_acceleration_covariance` / `angular_velocity_covariance`。不要为了填字段而伪造姿态。

### 4.6 其他零散但会造成困惑的点

| 现象 | ROS 用户会怎么误读 | ROS 侧必须做什么 |
| --- | --- | --- |
| `CameraIndex.RIGHT = 0`、`LEFT = 1`（0 是右目） | 我会假定 0 是左目，写下 `dev.camera_intrinsics(0)` 并拿到右目的结果 | 文档显式写"0=RIGHT, 1=LEFT"，参数用字符串 `left`/`right` 而不是整数 |
| `Config.preset()` 要求 `RAW` 模式下 `output_width/height` 必须等于 1088x1280 | 我会以为可以任意设置 `raw` 分辨率 | 参数表里为每种 `mode` 单独列出**合法**的宽高组合 |
| `available_camera()` / `available_imu()` / `imu_name` / `eeprom_info` 只是文本/数量 | 我不知道这些信息在 ROS 侧该出现在哪，也不知道有没有用 | 启动时打一条结构化日志（可用相机数、IMU 名称与信息、EEPROM 信息），并说明它们与 topic 是否出现的关系 |
| `calibration()` 里 `install_angle` 是 int，`ReferenceFrame` 有 `CAMERA_RIGHT/CAMERA_LEFT/IMU`，`relative_R/T` 是 3x3/3 | 我不知道 `install_angle` 的单位与用途，也不知道该发哪些 TF | 文档给出"标定字段 → ROS 输出"的映射表（哪些进 `camera_info`，哪些进静态 TF，哪些不进 ROS） |
| `dist_model` 是 `PINHOLE` / `FISHEYE`，`dist_coeffs` 长度 8 | 我会顺手写 `distortion_model: "plumb_bob"` | 文档给出 `PINHOLE → "plumb_bob"`、`FISHEYE → "equidistant"` 的明确映射，并说明 8 个系数如何放进 `D` |
| `read_image()` / `read_imu()` 超时返回 `None`，`test_gs130.py` 用 `time.sleep(0.01)` 忙等 | 我会照抄这个 busy-wait 到我的节点里，制造 CPU 热点 | 文档明确"不得照抄示例的忙等"；节点内部用条件变量/阻塞读取或受控定时器 |
| `test_gs130.py` 只跑 10 帧就退出，且强依赖命令行参数个数与 `cv2` | 我会以为这就是"官方用法"，并照抄它的参数解析 | 文档提供一个**ROS 侧的独立最小示例**，不要让我从 smoke test 里学接口 |

---

## 5. v0.1.0 我会拒绝的东西

### 5.1 三个过度设计（做了我会反对）

1. **自定义消息类型**。`Gs130Frame.msg` / `Gs130Imu.msg` 之类。理由：`sensor_msgs/Image` + `sensor_msgs/CameraInfo` + `sensor_msgs/Imu` 已经完整覆盖我需要的所有信息，自定义类型会把我锁死在你们的生态里，还要我额外 `source` 一个 msg 包。唯一例外是"如果标准类型真的装不下"——目前看装得下。
2. **内建深度 / 点云 / 视差输出**。`hobot_stereonet` 已经存在，我的部署链条是"GS130 出图 → stereonet 出深度"。在 v0.1.0 里塞一个自己的深度节点意味着两份代码、两个 bug 来源、以及"到底该用哪个"的文档负担。请把它明确留到后续包，并在文档里写"如何把本包的 topic 接到 `hobot_stereonet`"。
3. **运行期动态参数重配（含参数回调热切换分辨率/模式/帧率）**。理由：改 `mode`/`width`/`height`/`fps` 在底层意味着重建 VIN-ISP-VSE/GDC 流水线（`mode=raw|resize|rect` 走的是不同的路径），甚至要重新做 FSYNC 握手；运行期热切换是可靠性陷阱，收益却很小——我重启节点只要几秒。请只做"启动时参数 + 明确的 `restart required` 文档"，把 `ros2 param set` 对一个无效参数的处理写成"明确拒绝/明确警告"。

### 5.2 三个交付不足（少了我认为 v0.1.0 不成立）

1. **相机被独占时没有可诊断的失败**（对应 E2）。静默挂起、只打 `HW_ERROR`、或无限重试都算交付不足。必须有：非零退出码、日志里点明"设备可能已被其他进程占用（mipi_cam?）"、以及文档里的排查步骤。
2. **`camera_info` 缺失或与真实标定不符**（对应 E4）。包括：没有 `camera_info`、`R`/`P` 全零、fisheye 被标成 `plumb_bob`、`width`/`height` 与实际图像不一致、`frame_id` 与图像不一致。少任何一条，这套接口在我项目里就是"只能看不能算"。
3. **IMU 与图像的耦合发布、或时间戳语义未定义**（对应 E3/E4.4）。如果 `ros2 topic hz /imu/data` 出来的是 fps、或者 `header.stamp` 的时钟域在文档里找不到，这个包就不能用于任何融合用途，只能当"有个加速度计在动"的演示。

（额外的边界：如果文档不能让我在不读 C 头文件的情况下把 NV12 正确填进 `sensor_msgs/Image`，我也算它为交付不足。）

---

## 6. 我要在文档里看到什么

最小章节集合（按我会翻阅的顺序）：

1. **拓扑总表**：节点名 → 它发布/订阅的每个 topic → 消息类型 → QoS（reliability/durability/depth）→ encoding → `frame_id` → 期望频率。**这是我排障时第一个打开的表。**
2. **参数总表**：参数名 → 类型 → 默认值 → 合法取值/组合 → 启动期还是可动态改 → 改了会发生什么。
3. **快速开始**：从 `source /opt/tros/humble/setup.bash` 到"浏览器里看到画面"的完整命令序列，一行一条，可直接粘贴（包含 web UI 的 `http://<board-ip>:8000` 与 channel 编号说明）。
4. **Web UI / TROS 集成**：`hobot_codec` 的完整参数示例（`channel`、`sub_topic`、`pub_topic`、`in_format`、`out_format`）、websocket 节点如何声明该 channel、以及"用本包替换 D-Robotics 那个 stereo launch 时，需要改哪几行"。
5. **topic / 命名空间 remap 示例**：如何把本包接到 `hobot_stereonet`、如何改 `ns`、如何同时跑其它相机（如果可能）。
6. **标定与 TF**：`camera_info` 字段逐个说明来源（`K`/`D`/`R`/`P`/`distortion_model`/`camera_name`/`frame_id`/`width`/`height`）；静态 TF 的父子关系与轴约定；`PINHOLE`/`FISHEYE` 的映射；`install_angle` 的用途；外参在哪个坐标系之间。
7. **已知限制与依赖**：相机独占（含与 `mipi_cam` 冲突的具体表现）、不提供 fused orientation、IMU 速率与相机 fps 的关系、时间戳时钟域、`SIGKILL` 后的恢复行为、需要的 SDK 版本 / 系统镜像 / TROS 版本。
8. **故障排查**：按"症状 → 可能原因 → 验证命令"组织，至少覆盖：无画面 / 花屏或错色 / `ros2 topic hz` 为 0 或异常 / 打不开设备 / web UI 空白 / `camera_info` 全零 / IMU 无数据 / QoS 不匹配。
9. **SDK 与 ROS 包的职责边界**：哪些能力来自 `libgs130`（例如实际 IMU 速率、时间戳来源），哪些是 ROS 包的策略选择。我要能判断"这是 SDK 的限制还是包的选择"。

### 我会第一个复制的示例

**"一条命令起双目 + 一条命令起 codec + 浏览器看到画面"的最小 launch + 命令序列**（第 3、4 项的合并）。具体形态：

```bash
source /opt/tros/humble/setup.bash
ros2 launch gs130_ros gs130_stereo.launch.py          # 示例，最终以实际包名为准
ros2 run hobot_codec hobot_codec_republish --ros-args \
  -p channel:=0 -p sub_topic:=<文档给出的确切 topic 名> \
  -p pub_topic:=<文档给出的确切 topic 名> -p in_format:=nv12 -p out_format:=jpeg
# 浏览器打开 http://<board-ip>:8000 ，选 channel 0
```

原因：我判断一个包值不值得用的唯一标准是"多快能看到画面"。第二个会复制的示例是 `camera_info` 的 `ros2 topic echo` 输出片段（我要确认它是真的而不是占位零）。

---

## 7. 我希望项目在动手前回答的问题

1. 包名、节点名、默认 `ns` 分别是什么？最终默认的图像 topic 名是 `/image_combine_raw` 系列，还是 `/<ns>/camera_left|right/image_raw`？（我需要一个确定答案，不接受"两者都行"）
2. 帧时间戳（`trig_tv` 优先）和 IMU 时间戳（"aligned to the camera clock"）的**时钟域**分别是什么？能否换算到 `CLOCK_REALTIME`？如果只能给 `node_clock`，请明确写出。
3. IMU 的实际可读速率与相机 fps 是什么关系（源码显示只有 FSYNC 锚点之后才配对并推入 FIFO）？`imu_odr_hz` 参数究竟控制硬件 ODR 还是别的？v0.1.0 承诺的 IMU 发布频率是多少？
4. 图像 topic 的默认 QoS 是什么，为什么？它能否与 `hobot_codec` 的订阅端默认值匹配？不匹配时会在哪里报错（还是完全静默）？
5. `camera_info` 的来源是 EEPROM 标定还是节点参数？`distortion_model` 里 `FISHEYE` 映射成什么字符串，8 个 `dist_coeffs` 如何填入 `D`？`R`/`P` 打算怎么构造？
6. 相机被占用时，节点会怎样失败（退出码、日志文本、是否重试）？以及 `SIGKILL` 之后相机能否在无人工干预的情况下重新打开？
7. 是否提供静态 TF / `robot_state_publisher` 路径？如果提供，TF 的 frame 命名与父子关系是什么；如果不提供，请说明我该拿 `calibration()` 的哪些字段自己做。
8. `mode=rect` 时，整流后的 `camera_info` 是否也要相应变化（`D` 是否应清零、`K`/`P` 是否要改）？这是我最容易踩错的一点，请明确"`rect` 模式下 `camera_info` 的语义"。
9. 是否支持 `GS130W`（无 IMU）？不支持时，`device:=GS130W` 是明确报错，还是静默降级？
10. 受支持的 `width`/`height`/`fps` 组合有哪些（每种 `mode` 分别列出）？超出范围时报错还是自动裁剪？
11. 参数中是否会出现绝对路径或对 `/opt/tros` 的写依赖？安装方式是 `apt`、`colcon build` 还是既有的 SDK 安装流程？
12. 一个最小可用示例（可粘贴的 launch + 命令）能否在**只有本文档**的情况下跑通？谁负责验证这一点，什么时候验证？

---

## 附：我认可现状的部分（避免反馈显得片面）

- `Config.preset()` 的签名几乎可以直接映射成 launch 参数，这是很好的设计；请不要为了"ROS 风格"把它拆散。
- `Image` 作为 `np.ndarray` 子类且零拷贝、带 `timestamp_ns`，为 ROS 发布留了正确的余地（`data` 可直接取字节，未来也能走共享内存）。
- `Device` 是上下文管理器 + `stop()` / `close()` 分明，生命周期语义清晰，能让 ROS 节点把"释放相机"做对——这是 E5 能成立的基础。
- `ErrorCode` 有明确枚举（含 `TIMEOUT` / `HW_ERROR` / `THREAD_CLOSED`），ROS 侧可以据此给出有区分度的诊断，而不是一句"failed"。请在 ROS 侧真的用上它。
