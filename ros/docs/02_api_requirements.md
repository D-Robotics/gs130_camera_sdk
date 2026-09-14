# 02 API Requirement Review — gs130_ros 外部契约（PM-2）

| 项目 | 值 |
| --- | --- |
| 文档角色 | PM-2（interface/API）交付物，接口需求评审记录 |
| 目标版本 | `gs130_ros` v0.1.0 |
| 平台 | RDK X5 + TROS humble（`/opt/tros/humble`），ROS 2 Humble 语义 |
| 上游依赖 | `gs130_sdk` C API / Python binding（`python/gs130`，见 §1.2） |
| 评审方式 | 逐行 accept / reject。本文件定义的名称、类型、字段取值即为**冻结契约**；实现与本文件不一致时以实现为缺陷，除非先修订本文件 |
| 本文件的范围 | 只定义**对外接口**（topic / message / parameter / launch / QoS / frame_id）；不定义节点内部线程模型、不定义 C++ 类结构 |

> 评审提示：§4（topic 表）、§8（parameter 表）、§9（launch 契约）是必须逐行签字的章节；§7（CameraInfo）与 §6（消息映射）是"宁可留空也不许臆造"的红线章节。

---

## 1. 前置事实与上游接口冻结面

### 1.1 已给定事实（不再重新验证）

| # | 事实 | 对接口的后果 |
| --- | --- | --- |
| F1 | 相机独占：同一时刻只有一个进程能持有 GS130 | 本节点与 `mipi_cam` **互斥**，launch 中不得同时启动；启动失败必须报错而不是重试抢设备 |
| F2 | 复用 `hobot_codec`（ROS Image → jpeg）、`websocket`（:8000 web UI）、`hobot_shm`（零拷贝环境） | 本包**不实现**编解码、不实现 web server、不实现零拷贝传输 |
| F3 | D-Robotics 参考链路：camera → NV12 `/image_combine_raw` → `hobot_codec` → `/image_combine_jpeg` → `websocket` channel 0 | `/image_combine_raw` 的名称、类型、NV12 约定**不可改** |
| F4 | GS130 = 双 1088×1280 MIPI 传感器 + ICM-42688-P + EEPROM 鱼眼标定 | 单目分辨率可非 16:9；畸变模型为 fisheye（等距） |
| F5 | `Config.preset(platform, device, mode, width, height, fps, odr)` 只支持 `platform="RDKX5"`、`device∈{GS130WI, GS130W}`；`mode` 取 `CameraMode.RAW/RESIZE/RECT`；`device="GS130W"` 无 IMU | ROS 参数取值域直接继承，不做二次发明（§8） |

### 1.2 上游 Python 接口（本契约唯一允许使用的 API 面）

| API | 返回 | 本契约使用点 |
| --- | --- | --- |
| `Config.preset(platform, device, mode, width, height, fps, odr)` | `Config` | §8 参数 → 设备配置 |
| `Device(config)` / `start()` / `stop()` / `close()` | — | 节点生命周期；`running` / `closed` 用于状态检查 |
| `read_image()` | `{"stitched": Image}` 或 `{"left": Image, "right": Image}` | §4 图像 topic；`Image.timestamp_ns` → `header.stamp` |
| `read_imu()` | `ImuPacket(accel, gyro, temp, is_fsync, timestamp_ns)` | §4 IMU topic |
| `camera_intrinsics(CameraIndex)` | `CameraIntrinsics(fx, fy, cx, cy, K, dist_coeffs[8], dist_model)` | §7 CameraInfo |
| `imu_intrinsics()` | `ImuIntrinsics(*_misalign/scale/bias, accel_noise, accel_random_walk, gyro_noise, gyro_random_walk)` | §6.3 covariance 判定依据 |
| `calibration()` / `relative_R/T(from,to)` / `convert_calibration(ref,R,T)` | 标定 | v0.1.0 **不暴露**（§11 非目标） |
| `imu_name` / `imu_info` / `eeprom_name` / `eeprom_info` | `str \| None` | 启动日志、IMU 是否存在的判定 |
| `available_camera()` / `available_imu()` | `int` | 调度：>0 才调用读取（避免空转） |
| `library_version()` | `str` | 启动日志 |
| `GS130Error`（`PARAM_ERROR` / `UNSUPPORTED` / `NOT_FOUND` / `HW_ERROR` / `TIMEOUT` / `THREAD_CLOSED`） | 异常 | §8.4 失败语义 |

### 1.3 上游语义中已确认、必须写进契约的细节（来自 SDK 源码，不是猜测）

| 编号 | 事实 | 影响 |
| --- | --- | --- |
| S1 | `Image` 是 `height*3//2 × width` 的 `uint8` 视图，`width/height` 是**单目**尺寸，缓冲区紧密打包（Y 平面 `width*height`，其后 UV 平面 `width*height/2`） | §6.1 映射公式的 `height`/`width` 语义 |
| S2 | `stereo_layout=LEFT_RIGHT` 时，单帧内左右目紧邻：`stride = width*2`，`frame.width = width*2`，`frame.height = height` | 拼接图 `message.width = 2*width`、`message.height = height` |
| S3 | 拼接帧的 `timestamp_ns` 取 FSYNC 绑定目（preset 中为右目）的相机时间戳 | combine 与 per-eye 的 stamp 关系需在文档中说明（§6.1） |
| S4 | `camera_intrinsics()` 返回的 `K/fx/fy/cx/cy` **已被管线改写**：RESIZE/RECT 模式下按 VSE 的 ROI 裁剪 + 缩放换算到输出分辨率；RECT 模式下更被立体矫正改写为"虚拟内参"（fx=fy=min、主点居中、`dist_coeffs` 全 0） | §7 必须区分 RESIZE 与 RECT 两种 CameraInfo |
| S5 | RECT 模式下 `dist_model` 仍报告 `FISHEYE`，即使 `dist_coeffs` 已被清零 | §7.3 需要"以 D 全零为准"的规则，否则下游解畸变会错 |
| S6 | `ImuPacket.accel` 已换算为 `m/s^2`（±16g FSR）、`gyro` 已换算为 `rad/s`（±2000dps），`temp` 为 `degC`，`is_fsync` 标记 FSYNC 同步包，`timestamp_ns` 已校正并对齐到相机时钟 | §6.3 不需要单位换算，只需赋值 |
| S7 | 上游**不提供融合姿态**：SDK 只有原始 accel/gyro（+ 确定性 bias/scale/misalign 与噪声密度），无姿态估计 | §6.3 `orientation` 必须明确为"非有效" |
| S8 | `ImuIntrinsics` 给的是**噪声密度**（`m/s^2/√Hz`、`rad/s/√Hz`）与**随机游走**（`m/s^3/√Hz`、`rad/s^2/√Hz`），不是方差 | §6.3 covariance 不能直接填 |
| S9 | EEPROM 标定 `install_angle = 0`；`cam_right` 为参考系单位位姿；`relative_R/T` 返回"从 from 系到 to 系"的绝对位姿 | §6.2 frame 层级；v0.1.0 不做 TF |

---

## 2. 参考链路证据（用于证明"名称不可改"）

本契约的 topic 名称来自 D-Robotics 自己的 launch 源文件，而非本项目臆测：

| 来源 | 关键内容 |
| --- | --- |
| `hobot_mipi_cam/launch/mipi_cam_dual_channel_websocket_132gs_nocal+cal+r90.launch.py` | `codec_sub_topic:=/image_combine_raw`、`codec_pub_topic:=/image_combine_jpeg`、`codec_in_format:=nv12`、`codec_jpg_quality:=85.0`、`websocket_image_topic:=/image_combine_jpeg`、`websocket_channel:=0`、`websocket_only_show_image:=True`、`dual_combine:=2` |
| `hobot_mipi_cam/src/hobot_mipi_node.cpp` | `frame_id_ = "camera_link"`（可被参数 `frame_id` 覆盖）；图像 topic 名 `image_left_raw` / `image_right_raw` / `image_combine_raw`；camera_info topic 名 `image_left_raw/camera_info`、`image_right_raw/camera_info`、`image_combine_raw/left/camera_info`、`image_combine_raw/right/camera_info`；camera_info QoS = `rclcpp::QoS(1).reliable().transient_local()`；IMU topic = `/imu_data`，`Imu.header.frame_id = "imu_link"`，只填 `linear_acceleration` / `angular_velocity`，**不填** orientation 与任何 covariance |
| `hobot_codec/src/hobot_codec_node.cpp` | `#define PUB_QUEUE_NUM 5`；`in_format:=nv12` 时以 `create_subscription<sensor_msgs::msg::Image>(topic, PUB_QUEUE_NUM, ...)` 订阅（KeepLast(5) + 默认可靠性 RELIABLE）；`in_mode:=ros` 时直接把 `msg->height`、`msg->width` 交给编码器，并按 `height*width*3/2` 做内存拷贝 |
| `hobot_mipi_cam/src/hobot_mipi_node.cpp`（续） | `#define PUB_BUF_NUM 5`：图像 publisher 为 `create_publisher<sensor_msgs::msg::Image>(topic, 5)`（KeepLast(5)、RELIABLE、VOLATILE），**不是** `SensorDataQoS` |
| `hobot_websocket/launch/websocket.launch.py` | 参数名 `websocket_image_topic` / `websocket_image_type`（默认 `mjpeg`）/ `websocket_only_show_image` / `websocket_output_fps` / `websocket_smart_topic` / `websocket_channel` |

**由此推导的两条硬结论**（§6 会再次强调）：

- **C1**：`hobot_codec` 的 `ros`+`nv12` 路径把 `msg->height` 当作**真实图像高度**使用（`video_utils::NV12_to_BGR24(msg->data.data(), mPtrIn, msg->width, msg->height)`，随后按 `msg->width * msg->height * 3 / 2` 计算输入长度）。因此 `sensor_msgs/Image.height` 必须是**图像高度**（拼接后仍为单目高度），而**不是** `height*3//2`。这与 ROS 社区某些 NV12 打包写法不同，属于本契约的显式决策。
- **C2**：`/image_combine_raw` 的 `width = 0` 之类的"半分辨率"写法会导致 codec 崩溃或花屏；`width` 必须是拼接后的行像素数（= 单目宽度 × 2）。

---

## 3. 节点与进程契约

| 项 | 冻结值 | 说明 |
| --- | --- | --- |
| 包名 | `gs130_ros` | 全小写下划线，ament_python 包 |
| 可执行名 | `gs130_node` | 相机 + IMU 发布节点（唯一持有相机的进程） |
| 节点默认名 | `gs130_node` | 可被 launch 覆盖 |
| 语言 | Python 3（复用 `python/gs130` binding） | 与 SDK 唯一受支持的 Python 接口一致 |
| 依赖 | `rclpy`、`sensor_msgs`、`std_srvs`(不用)、`python3-numpy`、`gs130` | 不新增第三方依赖 |
| 运行期进程 | `gs130_node`（本包）、`hobot_codec_republish`、`websocket`（+ nginx）、`hobot_shm` 环境 | 后三者由既有节点提供，本包只负责按参数拉起 |

---

## 4. Topic 契约表（逐行评审）

图例：**R** = v0.1.0 必须实现；**O** = v0.1.0 可选（默认关闭）；**—** = 不属于本包。

| # | Topic | 类型 | 含义 | 必需性 | 依据与理由 |
| --- | --- | --- | --- | --- | --- |
| T1 | `/image_combine_raw` | `sensor_msgs/msg/Image` | 左右目横向拼接 NV12（`LEFT_RIGHT`）：`data` 为单张 NV12，左目占据每行的左半、右目占据右半；`height` = 单目高，`width` = 2×单目宽 | **R**（唯一不可缺的图像 topic） | F3 + C1/C2：这是既有 TROS web 链路的入口名，改名/改语义即断链 |
| T2 | `/image_left_raw` | `sensor_msgs/msg/Image` | 左目 NV12 单帧（非拼接） | **O**（默认 `false`） | 名字与 mipi_cam 参考实现一致；stereonet/双目算法常按"每目一 topic"取流。参考 launch 在 `dual_combine:=2` 时**不发布**该 topic，所以它不是 web 链路必需 |
| T3 | `/image_right_raw` | `sensor_msgs/msg/Image` | 右目 NV12 单帧（非拼接） | **O**（默认 `false`） | 同 T2 |
| T4 | `/imu/data` | `sensor_msgs/msg/Imu` | ICM-42688-P 原始测量：`linear_acceleration`(m/s²)、`angular_velocity`(rad/s)；无姿态、无协方差 | **R**（当 `device` 有 IMU，即 `GS130WI`） | 用户显式要求该名字；ROS 生态标准名。**注意**：D-Robotics 自家 mipi_cam 用的是 `/imu_data`（无斜杠），冲突见 §13 OQ-3 |
| T5 | `/imu/data_raw` | `sensor_msgs/msg/Imu` | 与 T4 **逐字段相同**的消息（本节点不做任何补偿） | **O**（默认 `false`） | 仅在需要同时喂给依赖 `data_raw` 的生态节点时打开；重复发布两份相同数据没有信息增益，默认关闭以避免带宽与评审歧义 |
| T6 | `/image_combine_raw/left/camera_info` | `sensor_msgs/msg/CameraInfo` | 左目内参（输出分辨率下、**未校正**鱼眼） | **R** | 名称直接沿用参考实现（`init_DualCalibration(..., "image_combine_raw/left/camera_info", "image_combine_raw/right/camera_info", ...)`） |
| T7 | `/image_combine_raw/right/camera_info` | `sensor_msgs/msg/CameraInfo` | 右目内参 | **R** | 同上。T6/T7 是**双目算法唯一可用的内参来源**（拼接图本身不发 CameraInfo，§7.4） |
| T8 | `/camera_info` | `sensor_msgs/msg/CameraInfo` | 左目内参的**简名别名**（逐字段等于 T6） | **O**（默认 `false`，参数 `publish_camera_info_alias`） | 便于单目消费者无需 remap；默认关闭以免与其它相机节点的同名 topic 冲突 |
| T9 | `/image_combine_jpeg` | `sensor_msgs/msg/CompressedImage` | web 用 JPEG | **—** | 由 `hobot_codec` 发布（F2），本包**禁止**发布（§11） |
| T10 | `/imu_extrinsic` | `geometry_msgs/msg/TransformStamped` | IMU↔相机外参 | **O**（v0.1.0 不做） | mipi_cam 有该 topic；本包 v0.1.0 不暴露外参（§11），列为 v0.2 候选 |
| T11 | TF（`static_transform_publisher`） | `tf2_msgs/TFMessage` | `camera_link` ↔ `imu_link` | **O**（默认不启动） | 不启动 TF 不影响 web 显示与 IMU 数据可用性；启动则在 launch 中显式开启，见 §9 |

**Topic 命名决策（冻结）**

1. T1 名称 `image_combine_raw`、T9 名称 `image_combine_jpeg`、`websocket` 的 `image_topic` = `/image_combine_jpeg`、`channel` = 0 —— 与参考 launch 完全一致，**不得**使用 `/image_raw`、`/image_jpeg`、`/stereo/*` 等替代名（后者会破坏 F3 链路与 web UI 兼容）。
2. T6/T7 采用参考实现的**原样名称** `/image_combine_raw/left/camera_info`、`/image_combine_raw/right/camera_info`，不做"美化改名"。理由：这两个名字是 TROS 生态既有约定（`mipi_cam`、`hobot_stereonet` 类算法按此取内参），改名会让下游必须额外 remap；"挂在图像名之下"虽然读起来像层级，但在 ROS 中只是普通 topic 名，不产生歧义。简名 `/camera_info`（左目）作为**可选别名**由 T8 提供。
3. 拼接图**不发布** `/image_combine_raw/camera_info`：拼接图不满足单相机模型（`P`、`D` 都无意义），发布它等于制造假数据（§7.4）。
4. 本包 v0.1.0 **不发布** `/image_combine_raw/compressed`：`hobot_codec` 已承担（§11）。

---

## 5. frame_id 与帧层级

| frame_id 字符串 | 用于 | 类型 |
| --- | --- | --- |
| `camera_link` | T1/T2/T3 的 `header.frame_id` | 默认值，**与参考实现一致**（`mipi_cam` 参数 `frame_id` 默认 `camera_link`） |
| `imu_link` | T4/T5 的 `header.frame_id` | 默认值，与 mipi_cam IMU 实现一致 |
| `camera_optical_frame` | v0.1.0 不使用 | 需要 TF 才能成立，属非目标 |

两个 frame_id 名均可通过 ROS 参数覆盖（§8，`frame_id` / `frame_id_imu`），但**默认值冻结**为上表。理由：既有 web 链路与下游示例都按 `camera_link` 工作；改名会让已有订阅端的 TF 查询失败。

帧层级（仅文档记录，v0.1.0 不发 TF）：`imu_link` 与 `camera_link` 之间的外参由 `calibration().camera_left_R/T`、`imu_R/T` 描述，参考系为**右目（`GS130_REF_CAMERA_RIGHT`）**；`install_angle = 0`。若下游需要 TF，请在 launch 中显式启用静态 TF 节点并自行提供标定数值。

---

## 6. 消息字段映射规则

### 6.1 `gs130.Image` (NV12) → `sensor_msgs/msg/Image`

记单目输出尺寸为 `W × H`（`W = width` 参数，`H = height` 参数，RESIZE/RECT 模式下即用户请求值）。

| 字段 | 规则 | 备注 |
| --- | --- | --- |
| `header.stamp` | `sec = timestamp_ns // 1e9`，`nanosec = timestamp_ns % 1e9` | 取 SDK 帧时间戳（S1/S3）。**不得**用 `node.get_clock().now()` 替代（那会丢掉相机时间基准） |
| `header.frame_id` | `camera_link` | §5 |
| `height`（T1 拼接） | `H` | **不是** `H*3//2`，见 C1；`H` = 单目高度 |
| `width`（T1 拼接） | `2 * W` | S2：左右目横向紧邻，`stride = 2W` |
| `step`（T1 拼接） | `2 * W` | 紧密打包无 padding；`step` = 每行字节数 |
| `height`（T2/T3 单目） | `H` | S1 |
| `width`（T2/T3 单目） | `W` | S1 |
| `step`（T2/T3 单目） | `W` | S1 |
| `encoding` | 固定字符串 `"nv12"` | 小写，与 `hobot_codec` 的 `in_format:=nv12` 及 README 枚举一致 |
| `is_bigendian` | `0` | 单字节像素，无语义歧义；沿用参考实现 |
| `data` | NV12 视图的**连续字节序列**：Y 平面 `(2W)*H`（或 `W*H`）字节，其后 UV 平面 `(2W)*(H/2)`（或 `W*(H/2)`）字节 | `numpy.ascontiguousarray(img).tobytes()`；`data.size` 必须等于 `width*height*3/2`（对 T1 即 `2W*H*3/2 = 3WH`） |

**拼接图内部布局（冻结）**：`data[0 : 2W*H]` 为 Y 平面，每行 `2W` 字节，行内 `[0:W)` = 左目，`[W:2W)` = 右目；`data[2W*H : 3WH]` 为 UV 平面，每行 `2W` 字节，同样左半/右半分属左右目。该布局与 SDK `GS130_STEREO_LAYOUT_LEFT_RIGHT` 的填充方式（S2）逐字节一致，本节点**不做**重排。

**验收细则**

| ID | 验收点 |
| --- | --- |
| A1 | `ros2 topic echo /image_combine_raw --field height --field width --field step --field encoding --field is_bigendian` 输出 `H`、`2W`、`2W`、`nv12`、`0` |
| A2 | `ros2 topic hz /image_combine_raw` 与请求 `fps` 偏差 ≤ ±10% |
| A3 | `data` 长度 = `width*height*3/2`（由该节点日志断言一次，超限即报错退出） |
| A4 | web UI（`http://<board>:8000`）显示拼接画面，左半为左目、右半为右目 |
| A5 | `header.stamp` 与 IMU 消息 `header.stamp` 处于同一时钟域（同一次运行内单调递增，且 IMU/图像插值误差 ≤ 1 个相机周期） |
| A6 | 相机被占用（mipi_cam 在跑）时，节点在 ≤5s 内以非零码退出并在日志中给出原因（`GS130Error` 的 `HW_ERROR` 或 `NOT_FOUND` 文案），**不得**静默重试 |

### 6.2 T2/T3（可选 per-eye）附加规则

| 项 | 规则 |
| --- | --- |
| 触发 | 参数 `publish_per_eye_images:=false`（默认）→ 不发布；`true` → 发布 |
| 尺寸 | `W × H`，`step = W`，`encoding="nv12"`，`is_bigendian=0` |
| stamp | 与 T1 **同一帧**使用相同 stamp。注意 SDK 在 `LEFT_RIGHT` 模式下只返回一个拼接帧（`read_image()["stitched"]`），左右目时间戳在 SDK 内部已完成对齐（S3 的半步对齐循环），因此**本节点无法也不需要**再比较左右目时间戳；若该帧的 stamp 与上一帧相同或更小（时间戳未前进），记录警告并丢弃该帧 |
| 开销 | 每帧 2 次 `memcpy`（各 `W*H*3/2` 字节）；在 640×480@30 下约 13.8 MB/s，可接受；在 RAW 1088×1280@30 下约 125 MB/s（4.18 MB × 30），需实测（§13 OQ-6） |
| 说明 | `stereo_layout` 恒为 `LEFT_RIGHT`（T1 必需），因此 per-eye 数据来自**同一份 SDK 缓冲区**的两次切片拷贝，仍满足"单次读取"约束 |

### 6.3 `gs130.ImuPacket` → `sensor_msgs/msg/Imu`

| Imu 字段 | 规则 |
| --- | --- |
| `header.stamp` | `timestamp_ns` → `sec`/`nanosec`（S6：该时间戳已由 SDK 校正并对齐到相机时钟） |
| `header.frame_id` | `imu_link`（§5） |
| `linear_acceleration.x/y/z` | `ImuPacket.accel[0/1/2]`，**单位已是 m/s²，不做任何换算**（S6） |
| `angular_velocity.x/y/z` | `ImuPacket.gyro[0/1/2]`，**单位已是 rad/s**（S6） |
| `orientation` | **显式设为无效**：`x=y=z=0.0`，`w=1.0`（单位四元数作为占位） |
| `orientation_covariance[0]` | `-1.0`：ROS 约定"该字段无数据"。**理由**：SDK 不提供融合姿态（S7），本节点不做姿态估计；若填 0 会被下游误认为"姿态恒为单位四元数" |
| `orientation_covariance[1..8]` | `0.0` |
| `angular_velocity_covariance[0]` | `-1.0`（未知），其余 8 项 `0.0` |
| `linear_acceleration_covariance[0]` | `-1.0`（未知），其余 8 项 `0.0` |

**为什么 covariance 一律标记为 -1（未知）而不是用 EEPROM 的噪声参数填值？**

| 理由 | 说明 |
| --- | --- |
| 量纲不匹配（S8） | EEPROM 给的是**连续时间噪声密度**（`accel_noise` = m/s²/√Hz，`gyro_noise` = rad/s/√Hz）与**随机游走**（bias 随机游走）。ROS 的 `covariance` 字段在 Humble 中**未规定**是"离散采样方差"还是"噪声密度/PSD"（社区两种解释并存）。把 m/s²/√Hz 直接写进 `covariance[0]`（单位应为 m²/s⁴）是**量纲错误**，会让 EKF/VIO 后端严重低估噪声 |
| 无法确定采样率换算 | 严格换算需要 `σ_d² = σ_c² × (1/Δt)` 或 `σ_d² = σ_c² × BW` 的约定（Δt 与滤波器带宽相关）。本节点不掌握下游的离散化约定，不能替下游选择 |
| 确定性误差未补偿 | EEPROM 另有 `accel_bias/scale/misalign`、`gyro_bias/scale/misalign`。本节点发布**原始未补偿**测量（与 `imu/data_raw` 语义一致），如果同时给出"补偿后残差的协方差"也是虚假精度 |
| 评审可控 | 填 `-1` 是 ROS 官方允许的"未知"表达，下游必须显式处理（正确的工程信号）。将来若要投递噪声参数，正确做法是**新增独立 topic/diagnostic 或参数文件**，而不是塞进方差 |

**`temp` 与 `is_fsync` 的处置（冻结）**：`sensor_msgs/msg/Imu` 没有温度与同步标志字段。v0.1.0 **不发布**温度与 `is_fsync`（避免自定义消息，§11），只在节点日志中以 ≤1 Hz 的节流频率打印温度用于健康检查。若下游需要温度，走 v0.2 的 `sensor_msgs/msg/Temperature` 独立 topic（§13 OQ-2）。

**频率与调度（冻结）**：节点以"有数据即发"的方式发布，不主动降频。IMU 与图像共用同一个设备句柄，节点必须按"每轮循环先取图像、再排空 IMU"的顺序调度，且 `available_imu()` 为 0 时立即返回（**不得** `usleep` 等待 IMU 而阻塞图像路径）。理由：图像超时会直接丢帧，而 IMU 有 1024 深度 FIFO 缓冲（`preset` 设定），短时排空延迟不会丢数据；`odr` 参数决定硬件 ODR，见 §8。

---

## 7. CameraInfo 映射（"能填的填满、不能填的留零"）

### 7.1 字段来源表

| CameraInfo 字段 | 来源 | 规则 |
| --- | --- | --- |
| `header.stamp` | 节点启动时刻（内参不随时间变化；`transient_local` 保证送达，见 §7.5）。**不逐帧发布** | 避免用 30~200 Hz 广播常量标定，也避免强加 `message_filters` 配对要求 |
| `header.frame_id` | `camera_link` | 与图像 topic 使用同一 frame_id，便于下游库（`image_geometry`）直接比较与解析；因不逐帧发布，不要求 stamp 配对 |
| `height`, `width` | 参数（或 `camera_intrinsics` 对应的输出分辨率） | RESIZE/RECT 下 = 单目输出 `H × W`（S4：`K` 已被改写为输出分辨率下的值，二者必须一致） |
| `distortion_model` | `CameraIntrinsics.dist_model`（`DistModel.FISHEYE` → `"equidistant"`；`DistModel.PINHOLE` → `"plumb_bob"`） | 见 §7.3 的 RECT 例外规则 |
| `d` | `CameraIntrinsics.dist_coeffs`（8 项） | 按模型顺序直传：`FISHEYE` = `[k1,k2,k3,k4,0,0,0,0]`；`PINHOLE` = `[k1,k2,p1,p2,k3,k4,k5,k6]`。**保持长度 8**，不做截断 |
| `k` | `CameraIntrinsics.K`（3×3 行主序） | `K = [fx,0,cx, 0,fy,cy, 0,0,1]`；**已是输出分辨率下的值**（S4），不再二次缩放 |
| `r` | `[1,0,0, 0,1,0, 0,0,1]` | **单位矩阵**，语义 = "本 CameraInfo 已表达在自身坐标系下"，不声称任何立体校正 |
| `p` | **全 0**（12 项） | v0.1.0 **不填**：见 §7.4 |
| `binning_x`, `binning_y` | `0` | 无 binning；`0` 是 ROS 约定的"未指定" |
| `roi` | 全 `0`（`x_offset/y_offset/height/width/do_rectify`） | 无 ROI 裁剪；`0` 约定为"整幅" |

### 7.2 逐项"为什么不填"的理由（评审红线）

| 字段 | 不填的理由 |
| --- | --- |
| `p`（投影矩阵） | `P` 的定义是"**已校正**图像的 3×4 投影矩阵"，其 `P[3]` 项即双目基线 `-fx*Tx`。非 RECT 模式下 GS130 输出的是**未校正鱼眼图**，填 `P` 等于向下游宣称"图已校正、可直接三角化"，会造成深度计算系统性错误。SDK 也没有提供 `P` |
| `p` 的替代品 | RESIZE 模式下的 K 已含裁剪缩放，信息足够；RECT 模式下 K 已被改写为虚拟校正内参（S4），此时 `P` 仍**不填**——因为 per-eye 的 `P` 需要基线（`relative_T(CAMERA_LEFT→CAMERA_RIGHT)`），而 v0.1.0 不暴露外参（§11 非目标），凭 `install_angle`/传感器尺寸**推不出**基线，属于"发明数值" |
| `binning`/`roi` | 管线是 ROI 裁剪 + VSE 缩放，不是硬件 binning；且 SDK 未返回裁剪窗口，填任何非零值都是编造 |

### 7.3 RECT 模式下的 `distortion_model`（必须按此实现）

S5 指出：RECT 模式下 SDK 把 `dist_coeffs` 清零、`K` 改写为虚拟 pinhole，但 `dist_model` **仍报 FISHEYE**。因此规则冻结为：

| 条件 | `distortion_model` 取值 |
| --- | --- |
| `dist_coeffs` 全 0 | `"plumb_bob"`（零畸变的针孔），**忽略** `dist_model` 标签 |
| `dist_coeffs` 非全 0 且 `dist_model == FISHEYE` | `"equidistant"` |
| `dist_coeffs` 非全 0 且 `dist_model == PINHOLE` | `"plumb_bob"` |

补充：`"equidistant"`（等距鱼眼，OpenCV `fisheye` 的 k1..k4）**不在** ROS REP/`sensor_msgs` 的官方枚举里，是 ROS 社区（`image_pipeline`、`camera_calibration`）的既有实践；`image_proc` 会拒绝非 `plumb_bob` 的模型，这是可接受的（下游鱼眼解畸变本就应使用 `cv::fisheye`）。此项列 §13 OQ-4 供评审确认。

### 7.4 拼接图的 CameraInfo

`/image_combine_raw` **不发布** CameraInfo。理由：拼接图是"一张图两个不同相机模型"，`K`/`D`/`P` 无一能表达；发布一个假的"拼接相机"内参会让 `image_proc`、`image_geometry`、stereo 后端静默出错。需要内参的消费者请订阅 T6/T7：其中的 `K`、`D` 均表达在**各自相机的输出分辨率坐标系**（`W × H`）下；若消费者要在拼接坐标系中使用，需自行在 `cx` 上加 `+W`（右目位于右半幅）。这一步属于消费者职责，写在此处以便评审确认不是遗漏。

### 7.5 发布策略

| 项 | 值 |
| --- | --- |
| 发布时机 | 节点启动、拿到内参后**立即发布一次**；随后每 10s 重发一次（防丢、便于晚加入的消费者） |
| 每帧 vs 单次（**必须明确**） | **不逐帧发布 CameraInfo**。理由：内参在节点生命周期内不变，逐帧发布等于用 30~200 Hz 的带宽广播常量，且会把 `message_filters::ApproximateTime` 的配对要求强加给下游。消费者应订阅一次（`transient_local` 保证必然收到），并用同一份内参处理所有帧。这**不同于**参考实现（mipi_cam 逐帧发布 CameraInfo），差异已在 §12 记录 |
| QoS | `rclpy` 等价：`QoSProfile(depth=1, reliability=RELIABLE, durability=TRANSIENT_LOCAL)` —— 与 D-Robotics 参考实现一致（§2 证据），保证晚加入的订阅者能拿到标定 |
| 发布失败降级 | `camera_intrinsics()` 抛 `GS130Error`（如无 EEPROM）时：**不发布** CameraInfo，`RCLCPP_WARN` 明确说明"无标定"，图像与 IMU 继续发布。**禁止**用 `fx=width`、`cx=width/2` 之类的猜测值兜底 |

---

## 8. ROS 参数契约表（逐行评审）

类型沿用 ROS 2 参数类型。所有参数在节点启动时声明；**v0.1.0 不支持运行中修改**（§11）。

### 8.1 设备/流配置参数（必答组）

| 参数 | 类型 | 默认值 | 有效范围 | 作用 | 理由/备注 |
| --- | --- | --- | --- | --- | --- |
| `platform` | string | `RDKX5` | 仅 `RDKX5` | 传给 `Config.preset` | 唯一受支持平台（F5）；保留参数是为了将来扩展，不是发明 |
| `device` | string | `GS130WI` | `GS130WI` \| `GS130W` | 传 `preset`；决定是否有 IMU（`GS130W` 无 IMU → 不发 T4/T5） | 与 SDK 白名单一致；非法值直接报错退出 |
| `mode` | string | `resize` | `raw` \| `resize` \| `rect` | → `CameraMode.RAW/RESIZE/RECT` | 小写字符串便于命令行；`rect` 需要 EEPROM 标定，无标定会 `GS130_PARAM_ERROR`（需在启动日志说明） |
| `width` | int | `640` | `≥ 8`，8 字节对齐（codec 要求）；`raw` 模式**必须** `1088`；`resize`/`rect` 下需满足管线 ROI 整除条件（见下） | 单目输出宽 `W` | 默认 640×480 是"能跑通 web 链路"的最小可用分辨率；`raw` 必须等于传感器尺寸（SDK 强校验）。**管线约束（源码依据，需实测确认）**：RESIZE/RECT 走"保持宽高比的 ROI 裁剪 + 缩放，不补黑边"（`src/devices/pipeline/rdkx5/vse.c::aspect_roi`），且 `roi_ratio_exact()` 要求整除；在 1088×1280 竖幅传感器下输出宽高关系为 `W ≤ (4/3) × H`，因此 `W/H > 4/3` 的组合（如 640×360、1280×360）会请求超出画面的区域。该组合**不在验收范围**；若评审要求支持，需另立需求 |
| `height` | int | `480` | `≥ 8`，8 对齐；`raw` 模式**必须** `1280`；与 `width` 的宽高比约束同上 | 单目输出高 `H` | 同上。注意 `raw` 是 1088(W)×1280(H) 竖幅（`preset` 中 `sensor_width=1088, sensor_height=1280`） |
| `fps` | int | `30` | `> 0`，且为传感器支持的档位（`preset` 只透传，非法值在 `gs130_init` 报错） | 传感器帧率 | 默认 30 匹配 codec 的 `input_framerate` 默认值 30 |
| `odr` | int | `200` | `200` \| `500`（ICM-42688-P 的两档 ODR） | IMU ODR；`GS130W` 时忽略 | 默认 200 与 `preset` 示例一致；500 时需复核 FIFO/带宽（§13 OQ-5） |
| `frame_id` | string | `camera_link` | 任意非空 | 图像 `header.frame_id` | 与参考实现默认值一致（§5） |
| `frame_id_imu` | string | `imu_link` | 任意非空 | IMU `header.frame_id` | 与 mipi_cam IMU 实现一致 |
| `publish_per_eye_images` | bool | `false` | true/false | 是否发布 T2/T3 | 默认关闭：web 链路只需要 T1，避免无谓带宽（§6.2） |
| `publish_imu_raw` | bool | `false` | true/false | 是否额外发布 T5 | 内容与 T4 相同，默认关闭 |
| `imu_topic` | string | `/imu/data` | 任意合法 topic | IMU 消息的发布 topic | 提供它而不是硬编码，是为了在需要与 D-Robotics `mipi_cam` 生态（`/imu_data`）互通时一键切换（§13 OQ-3）；也可直接用 REMAP 达到同样效果 |
| `publish_camera_info_alias` | bool | `false` | true/false | 是否额外发布 T8（`/camera_info` 简名，内容 = 左目内参） | 便于单目消费者免 remap；默认关闭以免与其它相机节点同名 topic 冲突 |
| `publish_images` | bool | `true` | true/false | 是否发布图像（T1/T2/T3）；`false` 时只发布 IMU 与 CameraInfo | 供"只读 IMU"场景使用（§13 OQ-7）；不改变相机配置，只关发布 |
| `queue_depth` | int | `4` | `≥ 2` | → `Config.camera_fifo.depth` | SDK 要求 `depth ≥ 2`；4 与 `preset` 一致 |
| `use_sdk_timestamp` | bool | `true` | true/false | `true` = 用 SDK 的 `timestamp_ns`；`false` = 用节点时钟（`get_clock().now()`） | 默认信任 SDK 时钟（IMU 已对齐到相机时钟，S6）。若实测 SDK 时钟域与 ROS 时钟域不一致（§13 OQ-1），可切到 `false` 作为应急开关，代价是失去硬件时间精度 |
| `log_level` | string | `info` | `debug`…`fatal` | 日志级别 | 便于现场定位；非功能性 |

### 8.2 下游链路透传参数（可选组）

| 参数 | 类型 | 默认值 | 有效范围 | 作用 |
| --- | --- | --- | --- | --- |
| `jpeg_quality` | double | `85.0` | `0.0 ~ 100.0` | 透传给 `hobot_codec` 的 `jpg_quality`；默认 85.0 取自参考 launch |
| `websocket_channel` | int | `0` | `0 ~ 3`（web 页面上的显示通道序号，参考实现用 `0`） | 透传给 `websocket` 的 `channel` |
| `websocket_image_topic` | string | `/image_combine_jpeg` | 任意 topic | 透传给 `websocket` 的 `image_topic` |
| `websocket_output_fps` | int | `0` | `0` = 不控速；`1~30` | web 端限帧，降低浏览器/编码负载 |
| `launch_codec` | bool | `true` | true/false | 是否在 launch 中启动 `hobot_codec`（调试时只跑相机） |
| `launch_websocket` | bool | `true` | true/false | 是否启动 `websocket` + nginx |
| `launch_static_tf` | bool | `false` | true/false | 是否启动 `camera_link ↔ imu_link` 静态 TF（需外部提供标定值时才有意义） |

### 8.3 明确判为"不必要"的参数（非目标，评审可据此否决新增）

| 候选参数 | 判定 | 理由 |
| --- | --- | --- |
| `stereo_layout` | **不需要** | T1 的布局必须恒为 `LEFT_RIGHT`（§4），把它参数化只会制造"发布出去的 `width/height` 与实际布局不一致"的坑。将来若支持上下拼接，属新版本需求并需重新冻结尺寸公式 |
| `publish_combined` | **不需要** | web 链路入口是 T1，关闭它等于关闭本节点主要价值；若确实要只发 per-eye，请另开话题集合的新版本需求 |
| `sub_topic` / `pub_topic`（图像） | **不需要**（仅窄用途） | codec 侧已有参数；本节点侧只允许用 ROS remap（`--ros-args -r /image_combine_raw:=/x`）而非自造参数，避免出现两套改名机制 |
| `smart_topic` | **不需要** | 面向 `ai_msgs::msg::PerceptionTargets`，属算法节点职责；`websocket` 的默认值即可 |
| `orientation_source` / `fuse_imu` | **不需要** | 需要滤波/融合算法，超出"接口包"范围（§11） |
| `output_framerate`（本节点限帧） | **不需要** | 限帧应交给 `hobot_codec` 的 `output_framerate` 或 `websocket` 的 `output_fps`；在发布端丢帧会破坏 IMU/图像时间配对的完整性 |
| `camera_info_url` | **不需要** | D-Robotics 参考实现里该参数默认空；标定直接来自 EEPROM，不存在外部 YAML 文件 |

### 8.4 参数校验与失败语义（冻结）

| 情况 | 行为 |
| --- | --- |
| `platform`/`device`/`mode` 非法 | 启动即 `RCLCPP_ERROR` + 退出码 2（对应 `GS130_PARAM_ERROR`），**不降级** |
| `raw` 模式下 `width/height ≠ 1088/1280` | 同上（SDK 强校验，必须给出可读的中文错误提示） |
| `odr` 非 200/500（且 `device` 有 IMU） | 启动即报错退出（SDK `GS130_UNSUPPORTED`）；这是**硬失败**，因为静默用默认档位会让下游以为按需配置成功 |
| `mode=rect` 且无 EEPROM 标定 | 启动即报错退出，提示需要标定（SDK `GS130_PARAM_ERROR`） |
| 相机被占用 / 硬件未检测到 | 退出码非零，日志含 `HW_ERROR` 或 `NOT_FOUND`，并在文档/启动日志中提示"确认没有 mipi_cam 在运行"（F1）；**不重试** |

---

## 9. Launch 契约

### 9.1 文件清单

| Launch 文件 | 用途 | 启动的节点 | 必需性 |
| --- | --- | --- | --- |
| `gs130_stereo_websocket.launch.py` | **主入口**：相机 + 编码 + web 显示（对齐参考 launch） | `gs130_node`、`hobot_codec`(jpeg)、`websocket`(+nginx)、`hobot_shm` 环境 | **R** |
| `gs130_stereo.launch.py` | 只跑相机（调参/取数/录制） | `gs130_node` | **R**（评审要求：至少有两个入口，便于隔离故障） |
| `gs130_imu.launch.py` | 只跑 IMU（标定/健康检查） | `gs130_node`（以 `publish_images:=false`、`launch_codec:=false`、`launch_websocket:=false` 启动） | **O** |

### 9.2 主 launch 的必需参数（与 §8 同名，值直接透传给节点）

| 参数 | 默认值 |
| --- | --- |
| `platform` | `RDKX5` |
| `device` | `GS130WI` |
| `mode` | `resize` |
| `width` | `640` |
| `height` | `480` |
| `fps` | `30` |
| `odr` | `200` |
| `jpeg_quality` | `85.0` |
| `websocket_channel` | `0` |

### 9.3 必须成功的用户命令（验收命令，逐字符冻结）

```bash
# 主链路（相机 → NV12 → jpeg → web UI）
ros2 launch gs130_ros gs130_stereo_websocket.launch.py \
  platform:=RDKX5 device:=GS130WI mode:=resize width:=640 height:=480 fps:=30 odr:=200

# 仅相机（不开 web）
ros2 launch gs130_ros gs130_stereo.launch.py \
  platform:=RDKX5 device:=GS130WI mode:=resize width:=640 height:=480 fps:=30 odr:=200
```

### 9.4 Launch 内部等价命令（用于评审核对参数名，不作验收命令）

```bash
# 1) 相机（本包）
ros2 run gs130_ros gs130_node --ros-args \
  -p platform:=RDKX5 -p device:=GS130WI -p mode:=resize \
  -p width:=640 -p height:=480 -p fps:=30 -p odr:=200
# 2) 编码（复用既有节点；参数名来自 hobot_codec 自身 README/launch）
ros2 run hobot_codec hobot_codec_republish --ros-args \
  -p channel:=0 -p in_mode:=ros -p in_format:=nv12 \
  -p out_mode:=ros -p out_format:=jpeg \
  -p sub_topic:=/image_combine_raw -p pub_topic:=/image_combine_jpeg \
  -p jpg_quality:=85.0 -p input_framerate:=30
# 3) web（复用既有节点）
ros2 run websocket websocket --ros-args \
  -p image_topic:=/image_combine_jpeg -p image_type:=mjpeg \
  -p only_show_image:=true -p channel:=0
```

### 9.5 Launch 验收判据

| ID | 判据 |
| --- | --- |
| L1 | 上述命令**在相机空闲、无 mipi_cam 运行**的板子上一次成功，无需手工附加参数 |
| L2 | 启动后 ≤10s，`http://<board>:8000` 的 channel 0 显示拼接实时画面（允许单目分辨率非 16:9 出现拉伸） |
| L3 | `ros2 topic list` 恰好出现 `/image_combine_raw`、`/image_combine_jpeg`、`/imu/data`、`/image_combine_raw/left/camera_info`、`/image_combine_raw/right/camera_info`，不出现其它本项目新造的 topic（默认参数下 T2/T3/T5/T8 均不在列表中） |
| L4 | `Ctrl-C` 后相机被释放：再次执行同一条命令仍能成功（验证 `stop()`+`close()` 路径） |
| L5 | `hobot_shm` 零拷贝环境由 launch 引入（`RMW_FASTRTPS_USE_QOS_FROM_XML=1` 等），但不作为 v0.1.0 的强验收项（本包走 `in_mode:=ros`，不依赖 shared_mem） |

### 9.6 Launch 的实现约束（复用而非重写）

| 约束 | 内容 |
| --- | --- |
| 必须复用既有 launch | 编码环节用 `hobot_codec` 的 `launch/hobot_codec_encode.launch.py`（`IncludeLaunchDescription` + `codec_*` 参数名），显示环节用 `websocket` 的 `launch/websocket.launch.py`（`websocket_*` 参数名），零拷贝环境用 `hobot_shm` 的 `launch/hobot_shm.launch.py`。**不得**在 `gs130_ros` 内复制这几个节点的参数声明 |
| node 名冲突 | `hobot_codec_encode.launch.py` 内部用 `uuid4()[:8]` 生成节点名，`websocket.launch.py` 同理；launch 中**不要**给它们指定固定 `name` |
| nginx | `websocket.launch.py` 会检测并启动自带 nginx（`:8000`）。多次 launch 时它不会重复启动；若 8000 被别的 nginx 占用，属环境问题，本包不做处理（N5） |
| 启动顺序 | `gs130_node` 与 `hobot_codec` 可并行启动（codec 会等第一帧）；`websocket` 建议放在 codec 之后（减少"无数据"告警），但**不构成契约** |
| 关闭顺序 | `Ctrl-C` 时 `gs130_node` 必须在 `on_shutdown` 中调用 `Device.stop()` + `Device.close()`（`with` 语义），保证相机被释放（L4） |
| 环境变量 | 若 `launch_static_tf:=true`，由 launch 启动 `tf2_ros static_transform_publisher`；外参数值**必须**来自用户提供的参数，**禁止**用编造值（§11 N8） |

---

## 10. QoS 与频率约定

| Topic | Reliability | Durability | History/Depth | 频率 | 依据 |
| --- | --- | --- | --- | --- | --- |
| `/image_combine_raw` (T1) | `RELIABLE` | `VOLATILE` | `KEEP_LAST(10)` | = `fps`（默认 30） | `hobot_codec` 在 `in_mode=ros` 下用 `keep_last(5)` + 默认可靠性（RELIABLE）订阅；`RELIABLE`+`VOLATILE` 与"BE 或 RELIABLE 订阅者"都兼容，因此是最宽松的选择。参考实现的图像 publisher 是 `create_publisher(topic, 5)`（KeepLast(5)、RELIABLE、VOLATILE），本包取 depth=10 以略增缓冲 |
| `/image_left_raw`, `/image_right_raw` (T2/T3) | `RELIABLE` | `VOLATILE` | `KEEP_LAST(5)` | = `fps` | 同 T1；per-eye 只给"能跟上"的消费者，深度取小 |
| `/imu/data` (T4), `/imu/data_raw` (T5) | `RELIABLE` | `VOLATILE` | `KEEP_LAST(200)` | ≈ `odr`（默认 200 Hz） | `rclcpp::SensorDataQoS()`（BEST_EFFORT、`KEEP_LAST(5)`）对 200 Hz 数据而言缓冲过浅（25 ms）；参考实现的 IMU publisher 用 depth=10 的默认 QoS。本包取 depth=200 ≈ 1 s 数据，兼顾突发与内存。**评审点**：若下游 EKF 明确要求 best-effort，可改为 `SENSOR_DATA`，但必须先修订本文档 |
| `/image_combine_raw/{left,right}/camera_info` (T6/T7)，`/camera_info` (T8, 可选) | `RELIABLE` | **`TRANSIENT_LOCAL`** | `KEEP_LAST(1)` | 启动时 1 次 + 每 10s 重发 | 与 mipi_cam 的 `QoS(1).reliable().transient_local()` 一致（§2 证据）；标定不随时间变化，晚加入的消费者必须拿到 |
| `/image_combine_jpeg` | 由 `hobot_codec` 决定：`SensorDataQoS()`（BEST_EFFORT、`KEEP_LAST(5)`） | `VOLATILE` | — | = `fps`（或受 `output_framerate`/`output_fps` 限制） | 既有节点行为（`hobot_codec_node.cpp` 中 `ros_image_publisher_` 用 `SensorDataQoS()`），本包不干预。注意：此处是 **BEST_EFFORT**，与 T1 的 RELIABLE 不同，属既有链路既有行为 |

**吞吐预算（供评审判断"默认值是否可行"）**

| 分辨率 | NV12 每帧 | `fps=30` 时 T1 带宽 | 说明 |
| --- | --- | --- | --- |
| 640×480（默认） | 0.44 MB（拼接后 0.88 MB） | ≈ 13.8 MB/s（拼接 ≈ 27.6 MB/s） | 与参考链路同量级，可接受 |
| 1088×1280（`raw`） | 2.0 MB（拼接后 4.1 MB） | ≈ 62 MB/s（拼接 ≈ 123 MB/s） | 仅在 `in_mode=ros` 下走 DDS，需实测；建议 `raw` 时用 `hobot_shm`+shared_mem（v0.2，§13 OQ-6） |

**频率验收**：`ros2 topic hz` 实测值与配置值的偏差 ≤ ±10%；IMU 与图像的时间戳必须落在同一时钟域（A5）。

---

## 11. 接口非目标（v0.1.0 明确不做）

| # | 非目标 | 理由 |
| --- | --- | --- |
| N1 | **不定义自定义消息**（无 `.msg`/`.srv`/`.action`） | 只用 `sensor_msgs`。避免下游必须额外 install 消息包；`/imu/data` 无温度字段的缺口用日志而非自定义消息补（§6.3） |
| N2 | **v0.1.0 不提供 service / action** | 生命周期由 launch 与信号控制；参数在启动时固定。没有"运行时标定/重配"的真实需求 |
| N3 | **不支持运行时动态重配管线**（改分辨率/模式/拼接布局必须重启节点） | SDK 的 `stereo_layout`、`output_width/height`、`mode` 都在 `gs130_init` 时定死（S2/S4），运行中改动需 deinit+init，等价于"重启相机"，比直接重启节点更危险（相机独占，F1） |
| N4 | **本节点不发布压缩图像**（无 `/image_combine_raw/compressed`） | `hobot_codec` 是唯一编码器（F2）；两处编码会争抢 VPU 且 web 端只看 `image_combine_jpeg` |
| N5 | **不实现 web server / 不做 nginx 托管** | `websocket` 包已提供（F2） |
| N6 | **不实现零拷贝（shared_mem）发布** | v0.1.0 走 `in_mode:=ros`；`hobot_shm` 环境由 launch 引入以便 v0.2 无缝升级（§13 OQ-6） |
| N7 | **不做标定文件的导入/导出**（`camera_info_url`、YAML 读写） | 标定来自 EEPROM，SDK 已有 `gs130-calib-export` 工具（核心仓库），ROS 包不需要重复 |
| N8 | **不做姿态融合、不做 TF 树发布（默认）** | 需要滤波算法与帧层级设计；`/imu/data` 的 `orientation` 明确为无效（§6.3） |
| N9 | **不做图像录制/回放、不做 RTSP/WebRTC 推流** | 超出接口包范围；`ros2 bag` 已能完成录制 |
| N10 | **不做自动重连/断流自愈** | 相机独占（F1）；SDK 故障后要求 deinit+init 才能恢复（`GS130_THREAD_CLOSED` 语义）。v0.1.0 选择"快速失败 + 明确日志" |
| N11 | **不允许重命名 T1/T9 或改变其 NV12 语义** | 断链风险最高的一条（F3）；改名只能通过 ROS remap 在用户侧做 |
| N12 | **不发布深度/点云** | 属 `hobot_stereonet` 或下游算法节点职责 |

---

## 12. 对既有 TROS 链路的兼容性检查表（评审用）

| 检查项 | 结论 | 依据 |
| --- | --- | --- |
| `hobot_codec` 能否直接订阅 T1 而不改参数 | ✅ 可以（`in_mode=ros`、`in_format=nv12`、`sub_topic=/image_combine_raw`） | T1 类型 = `sensor_msgs/Image`，`encoding="nv12"`，`height` 为真实图像高（C1） |
| `websocket` 能否直接显示 | ✅ 可以（`image_topic=/image_combine_jpeg`、`channel=0`、`only_show_image=true`） | 与参考 launch 一致（§2） |
| 分辨率对齐 | ⚠️ 需保证 `W`、`H` 8 字节对齐 | `hobot_codec` README：编解码要求宽高 8 对齐；默认 640×480 满足 |
| 竖幅画面 | ⚠️ 传感器是 1088(W)×1280(H) 竖幅，`raw` 模式下发到 web 端会显示为竖图 | 不做旋转（N3/A4 允许拉伸）；如需横屏请在 v0.2 用 GDC/`mipi_rotation` 类能力解决 |
| 与 mipi_cam 共存 | ❌ 禁止 | F1：相机独占，两进程抢设备 |
| IMU topic 名与 mipi_cam 一致 | ❌ 不一致（本包 `/imu/data` vs `mipi_cam` `/imu_data`） | 见 §13 OQ-3；评审需裁决 |
| CameraInfo 名称与 mipi_cam 一致 | ✅ 一致（`/image_combine_raw/{left,right}/camera_info`） | §4 T6/T7 |
| CameraInfo 发布节奏与 mipi_cam 一致 | ⚠️ **有意不同**：mipi_cam 逐帧发布，本包启动时发布一次 + 每 10s 重发（`transient_local`） | §7.5；若评审要求严格对齐，需改为逐帧（增加带宽、强加 stamp 配对），请在此处裁决 |
| 是否引入新依赖 | ✅ 无（仅 rclpy / sensor_msgs / numpy / gs130） | §3 |

---

## 13. 待决问题（Open Questions）与建议

| ID | 问题 | 备选 | **建议（PM-2 推荐）** | 若不采纳的后果 |
| --- | --- | --- | --- | --- |
| OQ-1 | SDK 的 `timestamp_ns` 时钟域是否与 ROS 时钟（`rclpy` 的 `system`/`steady`）一致？文档无法证明（板卡时间戳来自 VIN 管线；IMU 已对齐到相机时钟） | (a) 直接用 SDK stamp；(b) 用节点时钟；(c) 提供 `use_sdk_timestamp` 开关 | **采纳 (c)，默认 (a)**；并在 §6.1 A5 加"同域自检"：启动后比较首帧 stamp 与 `now()`，偏差 > 1s 时打 WARN 并建议切换 | 若时钟域不同且强行用 SDK stamp，`message_filters` 与 EKF 会静默丢帧；这是最高风险项，必须实测 |
| OQ-2 | IMU 温度（`ImuPacket.temp`）与 `is_fsync` 无处安放 | (a) 不发布（仅日志）；(b) 自定义消息；(c) v0.1.0 加 `sensor_msgs/Temperature` | **采纳 (a)**，v0.2 再评估 (c)。理由：违背 N1 的成本高于收益；温度是诊断量，不是算法输入 | 下游若需要温度需自行读 SDK，短期可接受 |
| OQ-3 | IMU topic 名：生态标准 `/imu/data` vs D-Robotics 自家 `mipi_cam` 的 `/imu_data` | (a) 只用 `/imu/data`；(b) 只用 `/imu_data`；(c) 默认 `/imu/data` + 参数 `imu_topic` 可切 | **采纳 (c)**（参数默认 `/imu/data`），并在 README 给出 `-r /imu/data:=/imu_data` 的 remap 示例。理由：用户明确要求 `/imu/data`；同时不放弃与 D-Robotics 内部工具的互通 | 若评审认为必须严格对齐 D-Robotics 命名，则改默认为 `/imu_data`，但需先修订本文档 §4 |
| OQ-4 | `distortion_model="equidistant"` 不是 `sensor_msgs` 官方枚举，`image_proc` 会拒绝 | (a) `equidistant`；(b) `plumb_bob`（错标但工具链能跑）；(c) `fisheye` | **采纳 (a)**。理由：(b) 是**错误标注**，会让 `image_proc` 用 Brown 模型解鱼眼系数，产生错误图像（静默错误）。宁可让工具显式拒绝 | 若坚持 (b)，必须在文档中写明"该字段不可信"，并要求下游不按 `D` 解畸变——工程上难以执行 |
| OQ-5 | `odr=500` 是否可行（缓存上限 `kMaxImuCache=1000`，500Hz 下丢 FSYNC 会更快触发 `HW_ERROR`） | (a) 默认只允许 200；(b) 允许 500 但文档告警；(c) 运行时自适应 | **采纳 (b)**：参数允许 200/500，`500` 时打印 INFO 告警"更高 ODR 下 FSYNC 中断的容错窗口更小" | 若只允许 200，VIO 类下游会抱怨；若默认 500，则风险面变大 |
| OQ-6 | `raw`/高分辨率下是否必须走 `shared_mem`（零拷贝） | (a) v0.1.0 全走 `in_mode=ros`；(b) v0.1.0 增加 `in_mode=shared_mem` 路径 | **采纳 (a)**，并把 (b) 列为 v0.2 需求。理由：`hbm_img_msgs` 是 D-Robotics 自有消息，会把接口包与平台消息包绑定，且 `hobot_codec` 的 `shared_mem` 路径要求 `RMW_FASTRTPS_USE_QOS_FROM_XML=1`（README 明确） | 高分辨率默认不可用；文档需写明"640×480/30 是 v0.1.0 的推荐工作点" |
| OQ-7 | 是否提供"只跑 IMU"的干净开关（`publish_images`） | (a) 不做（只能靠停用下游节点间接实现）；(b) 加 `publish_images:=false` | **采纳 (b)**：它是"发布开关"而非"重配管线"（不违反 N3），成本极小，且 `gs130_imu.launch.py` 没有别的干净实现方式 | 若无该开关，`gs130_imu.launch.py` 只能静默发布没人订阅的图像，浪费带宽与 CPU |
| OQ-8 | CameraInfo 的 `width/height` 是"参数值"还是"实测值" | (a) 用参数；(b) 用首帧 `Image` 的真实 `shape` 校验后发布 | **采纳 (a) 但在首帧校验**：不一致时打 ERROR 并按实测值发布（防止参数与 SDK 输出不符时下游解算错） | 若纯用参数，参数写错会静默传播到下游三角化 |

---

## 14. 逐行验收清单（Reviewer Checklist）

| # | 检查行 | Accept 判据 |
| --- | --- | --- |
| 1 | T1 名称 `/image_combine_raw`、类型 `sensor_msgs/Image`、`encoding="nv12"` | 与 §2 证据一致，且 §6.1 公式自洽（`width=2W`, `height=H`, `step=2W`） |
| 2 | T4 名称 `/imu/data`、`orientation` 无效化、三个 covariance 首项 `-1` | §6.3 规则可实现且 §6.3 的"为何不用 EEPROM 噪声"理由成立 |
| 3 | CameraInfo 只填 `K`/`D`/`distortion_model`/`R`/`height`/`width`，`P`/`binning`/`roi` 留零 | §7.1/§7.2 无"发明数值" |
| 4 | 参数表（§8）中每个参数都有：类型、默认、范围、作用、理由 | 缺失任一列即 reject |
| 5 | §8.3 的"不必要参数"清单被接受 | 若评审要加 `stereo_layout`/`publish_combined` 等，须先说明为何 §8.3 的理由不成立 |
| 6 | §9.3 的命令逐字符可用 | 参数名与 §8.1 完全一致（无拼写漂移） |
| 7 | QoS（§10）与 §2 证据一致，camera_info 为 `transient_local` | 与 mipi_cam 行为对齐 |
| 8 | 非目标（§11）中 N1/N3/N4/N11 被接受 | 这四条决定了"接口包"的边界与断链风险 |
| 9 | 所有 OQ（§13）都有明确建议与不采纳后果 | 允许 reject 单个建议，但必须给出替代决策 |
| 10 | §8.1 `width`/`height` 的管线约束（ROI 整除、`W/H ≤ 4/3`）被接受或给出纠正 | 该约束决定"哪些分辨率组合承诺可用"；不接受则须给出替代的支持矩阵与实测数据 |

---

## 15. 变更控制

| 变更类型 | 要求 |
| --- | --- |
| 新增/重命名 topic | 必须修订本文档 §4 并重新评审；**T1/T9 名称禁止变更** |
| 新增参数 | 必须补全 §8 的五列并说明为何不落入 §8.3 |
| 修改消息字段约定 | 必须修订 §6/§7 并给出下游影响分析（尤其 `height` 语义与 covariance 语义） |
| 放宽非目标 | 必须走新版本需求（v0.2+），不得在 v0.1.0 内追加 |
