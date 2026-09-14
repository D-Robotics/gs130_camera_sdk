# GS130 ROS 2 (TROS) 接口包 — 架构设计

- 文档编号：10（Architecture Design）
- 状态：待评审
- 目标版本：`gs130_ros` v0.1.0，对应 `gs130_sdk` VERSION `0.0.1`
- 平台：RDK X5 / Ubuntu 22.04 / TROS humble (`/opt/tros/humble`) / Python 3.10
- 上游契约（不得破坏）：`python/gs130/*`、`python/test/test_gs130.py`、`core/include/gs130.h`
- **接口契约**：`ros/docs/11_interface_freeze.md`（Interface Freeze）是话题名、消息字段、frame_id、QoS、参数名与默认值的**唯一权威**。本文档讨论"为什么这样做、代码怎么组织"；两者冲突时以 `11_interface_freeze.md` 为准（本文档已在 2026-09 对齐其 T1–T8 与参数表）。

---

## 0. 一句话方案

一个 `ament_python` 包 `gs130_ros`，里面只有**一个 `rclpy` 节点 `gs130_node`**：`Config.preset()` 之后把 `stereo_layout` 改为 `LEFT_RIGHT` 让**硬件**输出拼接帧（E6），用一个单线程 executor + 一个 `poll_period_ms` 轮询定时器取 NV12 与 IMU（`None` = 队列空），把 NV12 以 `array.array("B")` 快路径（E3）装进 `sensor_msgs/Image`（`height` = 真实高 `H`，E2）并以 **`RELIABLE`** QoS（E1）发布；下游**继续用 D-Robotics 的 `hobot_codec_republish` 编码成 JPEG、由 `websocket` 包在 nginx:8000 的官方页面显示**。本包不写 codec、不写 web 页面、不写 C++ 驱动、不做手工拼接、不发布 TF（§8.3）。

---

## 1. 目标与硬约束

### 1.1 目标

1. 把 GS130 双目图像、IMU、标定（内参/外参）以标准 ROS 2 话题暴露出来。
2. 图像最终由 D-Robotics 现有 TROS web UI（nginx :8000）在浏览器中显示，双目标清可见。
3. 代码量小、结构直白、能作为"如何在 RDK X5 上把一台非 mipi_cam 相机接入 TROS"的范例。
4. 不引入 SDK 内部改动：不修改 `core/`、不修改 `python/`。

### 1.2 硬约束

| # | 约束 | 对设计的直接后果 |
|---|---|---|
| C1 | 复用 D-Robotics 节点，禁止重实现 | 图像转换只用 `hobot_codec_republish`；网页只用 `websocket`；零拷贝环境只用 `hobot_shm`；不自写 HTTP / MJPEG / 编解码 |
| C2 | 必须复用 `hobot_codec_republish`，不得自写 codec | 我们的节点只发布 NV12 `sensor_msgs/Image`，JPEG 由官方 codec 生成 |
| C3 | 必须复用 `websocket` 包（nginx :8000 + 现有 index.html） | 不自建静态页、不改官方 html、不自写 websocket 推流 |
| C4 | 相机独占 | 本包与 `mipi_cam` 互斥；启动即检测，不提供"共享设备"模式 |
| C5 | Python binding 是 SDK 唯一受支持的入口 | ROS 节点用 `rclpy` + `import gs130`；不写 C++ 直连 `libgs130` |
| C6 | 简洁，不过度开发 | 1 个包、1 个节点、2 个 launch、1 个纯函数转换模块、少量测试 |
| C7 | 不改 `python/`、`core/` | 节点只能使用 `gs130` 已导出的公开 API（见 1.3） |
| C8 | SDK 无阻塞读 | `read_image()` / `read_imu()` 返回 `None` 表示队列空，必须由我们轮询 |
| C9 | `Config.preset()` 把 `stereo_layout` 硬编码为 `StereoLayout.NONE`（`python/gs130/_config.py:79`） | 需在 `preset()` **之后**改写 `config.camera_config.stereo_layout = StereoLayout.LEFT_RIGHT` 才能拿到 T1；这是**硬件拼接**，不是驱动侧手工拼接（E6，§6.1） |
| C10 | **T1 图像 QoS 必须是 `RELIABLE`**（E1 实测） | `hobot_codec` 以 `RELIABLE` 订阅；用 `BEST_EFFORT` 时它拒绝接收（`RELIABILITY_QOS_POLICY` 不兼容），页面永远黑屏。`shm_fastdds.xml` 不设置 reliability，**无绕过手段** |
| C11 | **`Image.height` 必须是真实图像高 `H`**（E2 实测，致命） | 填 `H*3//2` 会让 codec `init_pic_h_: 1080, alined_pic_h_: 1088` 后**段错误（-11）**。约定：`height = shape[0]*2//3`、`step = width`、`len(data) = width*height*3//2` |
| C12 | **`msg.data` 必须用 `array.array("B")` + `frombytes`**（E3 实测） | `msg.data = ndarray.tobytes()` 会走 rclpy `_check_types()` 逐元素校验：1.38 MB 帧实测 **1103.87 ms** 对比 `array.array` 路径 **1.36 ms**（约 800×）。这是节点唯一的"性能陷阱" |
| C13 | 默认 `mode = resize`；`raw` 仅用于标定导出（E7） | `raw` 强制 `width/height == 1088/1280`，且**不能用于拼接**（拼接后宽度会变 2176，与 SDK 硬约束冲突） |
| C14 | nginx 由 `websocket.launch.py` 用 `os.system` 启动，**launch 退出后成为孤儿进程存活**（E4 实测） | 关闭/重启流程必须显式清理（§9.3），否则"重启后端口占用/画面来自旧实例" |

### 1.3 可依赖的 SDK 契约（源码核对 + 板上实测）

**板上实测事实（E1–E7）优先于任何推断**；标注 [实测] 的条目已在 RDK X5 上验证过。

- `gs130.Config.preset("RDKX5", "GS130WI", mode, width, height, fps, odr)` 是唯一受支持的配置构造入口；`sensor_width/height` 固定 1088×1280，**`stereo_layout` 被写死为 `NONE`**。
  - [实测 E6] 在 `preset()` 之后把配置对象改为 `config.camera_config.stereo_layout = StereoLayout.LEFT_RIGHT`，`read_image()` 即返回 `{"stitched": Image}`，`resize` 640×480 配置下 shape 为 `(720, 1280)`。**硬件自行完成拼接，无需手工 `concatenate`。**
  - [实测 E7] `mode=raw` 时 `output_width/height` 必须等于 1088×1280，否则 `gs130_init` 返回 `GS130_PARAM_ERROR`；因此 `raw` **不能**用于拼接（拼接宽度会是 2176）。
- `gs130.Device(config)` → `start()` → `read_image()` / `read_imu()` → `stop()` → `close()`。
- `read_image()` 的返回形态由 `stereo_layout` 决定：`NONE` → `{"left","right"}`；`LEFT_RIGHT` → `{"stitched"}`。`Image` 是 `np.ndarray` 子类，shape `(H*3//2, W)`、dtype `uint8`、带 `timestamp_ns`。
  - 缓冲区由 `libgs130` `malloc`，所有权归调用方，**在 numpy 数组被回收时由 `weakref.finalize(owner, free, addr)` 释放**。
- `read_imu()` 返回 `ImuPacket(accel[3], gyro[3], temp, is_fsync, timestamp_ns)`，无 IMU 时返回 `None`。
  - `timestamp_ns` 是 SDK 对齐到相机时钟后的绝对时间（`core/src/gs130.cpp` 用 `TimestampTracker` 以相机 LPWM 主时钟对齐 IMU FSYNC 从时钟）。
  - [实测 E5] **时钟域是 `CLOCK_MONOTONIC`（自开机起算），不是 Unix epoch**：实测 `frame_ts_ns = 9546841822000`，同时刻 `time.monotonic_ns() = 9540690105753`（与 uptime 9480 s 吻合），而 `frame_ts - time.time_ns() ≈ -1.79e18`。图像与 IMU 同域（`imu_ts - frame_ts ≈ 8.6 ms`）。
    - **后果：`timestamp_ns` 绝不能直接填进 `header.stamp`**（`builtin_interfaces/Time.sec` 是 uint32，monotonic 秒数会变成无意义值）。用启动时一次性偏移（§7）。
- 标定：`device.camera_intrinsics(idx)`、`device.imu_intrinsics()`、`device.calibration()`、`device.relative_R/T(from, to)`、`device.convert_calibration(...)`。
  - **内参坐标系已由 SDK 处理**：`core/src/devices/pipeline/rdkx5/rdkx5.cpp` 在 `init()` 中对非 RAW 模式做 VSE ROI 裁剪+缩放，并把结果写回 `cal->cam_left/cam_right`；`gs130_get_camera_intrinsics()` 读的正是这份写回数据。**因此 `K` 永远是"输出像素坐标系"下的值，我们不做任何缩放。**
- 错误：所有失败以 `gs130.GS130Error`（`code` ∈ `ErrorCode`）抛出；`None` 只表示"当前无数据"，不表示故障。
- 拼接帧内存布局（`core/src/gs130.cpp` 的 `GS130_STEREO_LAYOUT_LEFT_RIGHT` 分支）：`width_out = 2*w`、`height = h`，Y 平面两目按行交错、UV 平面左目块紧跟右目块，总长 `2*w*h*3/2`。
  - [实测 E6] 该布局直接就是 `/image_combine_raw` 需要的"左目在左、右目在右"，且与 NV12 的 `step = 2*w` 约定自洽。**我们不再实现任何手工拼接**（早期设计里基于 `np.concatenate` 的方案已作废，理由见 §12）。
- [实测 E1/E2/E3] 下游 `hobot_codec_republish` 的硬性要求（否则黑屏或段错误）：入图 QoS 必须 `RELIABLE`；`height` 必须是真实高 `H`；`data` 走 `array.array` 快路径。这三条在本文件中作为约束 C10/C11/C12 贯穿全文。

---

## 2. 包形式选型：ament_python（rclpy）vs ament_cmake（rclcpp）

### 2.1 结论

**采用 `ament_python`（`rclpy`）。** 这是本节唯一推荐方案。

### 2.2 理由（按权重排序）

1. **C5 是决定性的**：SDK 只支持 Python 入口。选 `rclcpp` 意味着要么 (a) 在 C++ 里重新链接 `libgs130.so`，绕过 binding 自建一套 `gs130_create/init/...` 生命周期与 malloc 所有权管理，直接违反 C5 与 C7；要么 (b) 在 C++ 节点里嵌 CPython，复杂度远超收益。两者都不可接受。
2. 该节点是**纯粹的格式搬运**：读缓冲 → 构造消息 → publish。没有实时性要求，没有需要 C++ 的算法。
3. 复用链（codec / websocket / shm）全部是独立进程，语言无关；我们的节点用 Python 不影响 C2/C3。
4. 部署简单：`pip install gs130` 的 wheel + `colcon build --symlink-install`，无交叉编译、无 CMake 依赖查找（`libgs130.so` 由 wheel 的运行时加载逻辑负责）。

### 2.3 明确的取舍

| 维度 | ament_python（选） | ament_cmake + rclcpp（弃） |
|---|---|---|
| 与 SDK 契约 | 直接用唯一受支持入口，零重复 | 违反 C5，需自建 C 生命周期封装 |
| 每帧 CPU | 每目多 1 次 ~0.44 MB memcpy（640×480）+ 1 次 rclpy 序列化，两目合计 ~26 MB/s | 可做真零拷贝、可 hold dma-buf |
| 峰值延迟 | 单帧额外百微秒级 | 略低，但瓶颈在 ISP/JPEG，不在此 |
| 代码量 | ~450 行 Python | ≥1200 行 C++（含错误映射、RAII、calibration 结构体转换） |
| 构建/调试 | 改文件即生效（symlink-install），`python3 -m pytest` 可测 | 每次改要重编；板上编译时间显著更长 |
| 依赖 | 需要 `pip install gs130` 的 wheel | 需要 `.deb`/`libgs130.so` + CMake find module |
| 生命周期控制 | rclpy 足够（见 §4.3） | C++ 生命周期节点更规范，但本项目无外部 supervisor |
| 失败模式 | GIL 下 memcpy 阻塞会顶住 executor（可控，见 §6.5） | 无 GIL 问题 |

### 2.4 什么情况下会改这个决定

只要出现以下任一条，就应重新评估并**只针对图像路径**引入 C++（不是整体重写）：

- 实测 Python 侧（读+拷贝+序列化）在 30 fps 下占用单核 > 40%；
- 需要把 SDK 的 `malloc` 帧真正做成 dma-buf 零拷贝直通 `hobot_codec`（Python binding 不暴露该能力，且与 C7 冲突）；
- 需要在同一节点内跑立体匹配 / VIO 等已存在的 C++ 算法（此时更可能是"新增一个 C++ 节点订阅我们的话题"，而不是替换本节点）；
- 项目要求把 ROS 包做成 `.deb` 并进入 D-Robotics 镜像构建流程。

---

## 3. 仓库布局

新增内容全部在 `ros/` 下，`ros/` 目前是未被 git 跟踪的空目录（仅含 `docs/`）。

```
ros/
├── docs/
│   ├── 10_architecture.md              # 本文档
│   └── 11_interface_freeze.md          # 接口契约（他人维护，本文档对齐它）
├── probe_nv12_publisher.py             # 既有：不用相机的 NV12 链路探针（不属于交付包），用于先打通 codec+websocket
└── gs130_ros/                          # ament_python 包
    ├── package.xml                     # 包名/版本/许可证；依赖 rclpy、sensor_msgs、diagnostic_msgs、std_msgs、launch、launch_ros（**不含 tf2_msgs**，freeze §0.2）；gs130 以注释说明为 pip 依赖（无 rosdep key）
    ├── setup.py                        # ament_python 构建；entry_point gs130_node = gs130_ros.gs130_node:main；安装 launch/
    ├── setup.cfg                       # 脚本安装路径（ament_python 固定模板，2 行）
    ├── resource/gs130_ros              # ament 资源索引标记（空文件，包可见性）
    ├── gs130_ros/
    │   ├── __init__.py                 # 空包标记，不导出符号
    │   ├── conversions.py              # 纯函数层：NV12 ndarray→Image（array.array 快路径 + height=shape[0]*2//3）、ImuPacket→Imu、内参→CameraInfo、ns→builtin_interfaces/Time（启动时一次性 offset）。无 rclpy 状态、无设备依赖，可在无硬件主机上单测（不产生 TF，见 §8.3）
    │   ├── gs130_node.py               # 唯一 ROS 节点：参数解析、设备生命周期、两个定时器、故障上报、关闭
    │   └── gs130_probe.py              # 唯一非 ROS 命令：只读打印 libgs130/pip 版本、camera/imu 可用性、imu/eeprom 名称、标定摘要；相机不可用时退出码非零（US-11/AC-11）
    ├── launch/
    │   ├── gs130_camera.launch.py      # 只启动 gs130_node（无 web 链路，不依赖 hobot_codec/websocket 存在）
    │   └── gs130_web.launch.py         # 相机节点 + hobot_shm + hobot_codec_republish + websocket（浏览器看图）
    └── test/
        ├── test_conversions.py         # 主机可跑：消息构造、T1 拼接字节布局、时间换算、TF 方向、CameraInfo 字段
        └── test_gs130_hardware.py      # 板上冒烟：真实取帧/取 IMU，无设备时整体 skip
```

**文件数量说明**：包内 10 个文件（其中 3 个是 ament_python 的样板文件，1 个空 `__init__.py`）。没有单独的 `calibration.py` / `imu.py` / `image.py` 模块——它们各自只有几十行且只被 `gs130_node.py` 使用，拆开只会增加跳转成本。不设 `config/*.yaml`：参数默认值写在 `gs130_node.py` 的参数声明里，launch 只覆盖需要的项（单一事实来源，避免 launch 与 yaml 双份默认值漂移）。

与 `12_implementation_plan.md`、`11_interface_freeze.md` 的一致性：交付**两个** launch（`gs130_camera` / `gs130_web`），与 freeze §6.1 一致；不设第三个 "camera_info only" launch——标定-only 场景用 `ros2 launch gs130_ros gs130_camera.launch.py publish_imu:=false` 即可（`CameraInfo`/TF 在 `start()` 之前发布，本身就不需要取流），多一个 launch 文件没有新的能力。也**不新增 `stereonet` 演示 launch**（§12）。`gs130_web.launch.py` 的 D-Robotics 节点**逐个显式声明**，不 include 它们的 launch（§10.2）。

**包名/节点名**：包 `gs130_ros`，节点 `gs130_node`（与 freeze §1 一致）。不叫 `gs130` 以免与 pip 包 `gs130` 在 `ros2 pkg` / 日志 / import 命名空间中混淆。

---

## 4. 节点设计

### 4.1 标识与生命周期

- 节点名：`gs130_node`（固定，与 `11_interface_freeze.md` §1 一致；不做多实例，故不需要可改名的参数）。
- **使用普通 `rclpy.node.Node`，不使用 lifecycle node**。理由：设备独占（C4），进程内没有"配置态/激活态"分离的需求；引入 lifecycle 需要额外一个 manager 才能启动，属于过度开发（C6）。启停语义由 launch 的启动/`Ctrl+C` 表达。
- **布局在 `Device()` 之前定死**：`stereo_layout` 是 `gs130_init` 时下发给相机的配置（§6.1，E6），因此"拼接 / 单目"的选择发生在**构造 `Config` 与构造 `Device` 之间**，运行期不可切换（与 §4.4 的"参数只读一次"一致）。
- 进程内生命周期（唯一状态机）：

```
  参数解析 ──┬─ 构造 config = Config.preset(...)
             └─ 若 publish_combine：config.camera_config.stereo_layout = LEFT_RIGHT   # E6
                        │ (ValueError/GS130Error → 退出码 2/1)
                        ▼
                   Device(config) ──► 发 T5/T6（latched, RELIABLE）
                        │
                        ▼  start()（可能等 IMU FSYNC 握手）
                    Streaming ──poll_timer 发布 T1/T4──► 30 Hz（E4 实测）
                        │
                        ├── read_* 抛 GS130Error ──► Faulted ──► 退出码 1
                        │
        SIGINT/关闭请求   ▼  stop() ─► close() ─► destroy_node() ─► 退出码 0（3 s 内）
```

### 4.2 设备所有权

- 唯一持有者：节点实例属性 `self._device`，在 `__init__` 中创建，在 `main()` 的 `finally` 中 `stop()` + `close()`。
- **一次 `Device` 覆盖全部功能**：`camera_intrinsics()` / `imu_intrinsics()` / `calibration()` 在设备 idle 时即可调用（源码已验证，`state == ThreadClosed` 也允许）。因此顺序是：先 `Device(config)` → 发 T5/T6（标定，latched）→ 再 `start()` → 起定时器。这样"标定先于图像"在时间上天然成立，不需要等待/重发逻辑。
- 构造失败（无设备/相机被占用/无 EEPROM 而 mode 非 RAW）→ `GS130Error` 直接冒泡到 `main()`，先 `get_logger().error(...)`（文案见 freeze §7.1），再 `rclpy.shutdown()`，退出码 1/2。不重试、不降级到"空跑模式"：相机拿不到就应当立刻失败并报清楚原因（见 §9）。

### 4.3 线程与定时器

**单线程 executor（`rclpy.spin`），两个 `create_timer`，不使用额外 Python 线程。**

| 定时器 | 周期 | 每 tick 工作 |
|---|---|---|
| `poll_timer` | `poll_period_ms`（freeze 默认 2 ms） | ① `read_image()` **一次**：硬件拼接模式下返回 `{"stitched": Image}` → 以 `array.array("B")` + `frombytes` 装进 T1（`/image_combine_raw`，`publish_combine=true` 时）并 `RELIABLE` 发布；返回 `None` → 记一次 `drop`，不发消息 ② drain `read_imu()` 直到 `None` 或 `imu_drain_max`（默认 64）→ 逐个发布 T4 |
| `stats_timer` | `log_fps_period_s`（默认 5 s） | 统计实际 fps / IMU Hz / drop 数；按 `publish_status` 以 1 Hz 发 T8（`/gs130/status`）；检查"连续无帧"超时（`frame_timeout_s`） |

一个 tick 内**只调用一次 `read_image()`**（freeze §3.2 禁止"两次读设备凑对"）。默认（硬件拼接）下一次调用直接给出 T1 所需的整帧；`publish_per_eye=true` 时若同时需要单目，**必须**把 `stereo_layout` 退回 `NONE` 并同时发布 T1+T2+T3（同一次读取派生出三条消息，stamp 相同）——两种布局不能同时存在，这点由 §14 的决策表固定。

实测成本（E4，1280×480 拼接帧）：`build 0.53 ms / fill 1.28 ms / publish 5.07 ms`，整链 30 fps 稳定（`/image_combine_raw` 29.966 Hz、`/image_combine_jpeg` 29.974 Hz）。2 ms 的轮询周期足够容纳单帧处理。

设计要点：

1. **为什么用定时器而不是独占线程**：`read_*` 是非阻塞的（队列空时立即返回 `None`），不需要一个"会阻塞的读"来驱动；定时器由 executor 与 `rclpy.shutdown()` 统一管理，`Ctrl+C` 后没有第三个线程需要 join。语义上它就是"按固定周期轮询"，代码更短。
2. **为什么 IMU 与图像在同一个 tick**：两者共用同一次轮询节拍，代码只有一个回调入口；IMU 侧用 drain 循环吸收定时器抖动——IMU FIFO 深度 1024、`DROP_OLD`，`odr=200` 而 tick=2 ms（500 Hz）时每 tick 约 0.4 包，若不 drain，调度抖动会让队列堆积到 1024 后被 `DROP_OLD` 一次性丢弃，T4 也会从"等间隔"退化成"突发"。`imu_drain_max` 上限防止某次 tick 长时间占用 executor。
3. **发布速率策略**：
   - 图像速率 = 硬件 `fps`（默认 30），**不做重采样、不重复发布同一帧**（freeze §3.2）。
   - 积压时不追赶：一轮 tick 只读一次，队列满时由 SDK 的 `DROP_OLD` 丢弃旧帧。理由：`camera_fifo.depth=4` 且 `DROP_OLD`，节点侧自己再 drain 只会增加拷贝与带宽，而"最新帧优先"已由 SDK 保证。端到端延迟用 `ros2 topic delay` 观测（§11.2）。
   - 功能开关全部沿用 freeze：`publish_combine`（T1，默认 true，**置 false 会断开 web 链路**）、`publish_per_eye`（T2/T3，默认 false）、`publish_imu`（T4，默认 true）、`publish_camera_info`（T5/T6，默认 true，启动时各一次 + latched）、`publish_status`（T8，默认 false）。
4. **为什么 IMU 的 T4 是"逐包发布"而不是攒批**：`sensor_msgs/Imu` 每包一条消息（200 Hz 对 ROS 2 不构成压力），下游可用 `message_filters` 自行对齐；攒批需要自定义消息或 `Imu` 数组话题，属过度开发。
5. **为什么不做"读线程 + 队列"**：那会引入一个我们拥有的队列、一个停止标志和一个 join 点，收益只是让 executor 更空；30 fps / 200 Hz 的量级下不必要（C6）。

### 4.4 参数

**参数名、类型、默认值、取值范围、Reconfig 语义一律以 `11_interface_freeze.md` §5 为准，本文档不重复列举**（避免两套默认值漂移）。与架构实现直接相关的补充说明：

| 参数 | 说明 |
|---|---|
| `poll_period_ms` | 唯一轮询周期（图像与 IMU 共用）；freeze 默认 2 ms。它也是"帧超时"检测的时基 |
| `image_qos_depth` / `imu_qos_depth` | 分别对应 T1–T3 与 T4 的 `KEEP_LAST` 深度（freeze 默认 1 / 200） |
| T1–T3 的 reliability | **固定 `RELIABLE`，不做参数**（E1）：`hobot_codec` 以 `RELIABLE` 订阅，改成 `BEST_EFFORT` 会让它拒收（`RELIABILITY_QOS_POLICY` 不兼容）→ 页面黑屏。freeze 的"不允许用户改 QoS"规则在此有了实测依据 |
| T4 的 reliability | 维持 `BEST_EFFORT`（freeze §2）：IMU 无第二订阅方的兼容性约束，"丢最新比阻塞好" |
| `stamp_offset_mode` / `stamp_offset_ns` | 时间戳换算的控制（§7）；`device` 模式给可复现测试用 |
| `camera_info_distortion_model` | `auto`/`plumb_bob`/`equidistant`/`none`，覆盖畸变模型上报（§8.1） |

本设计**不新增** freeze 之外的接口参数。三个纯实现细节常量（`imu_drain_max=64`、`frame_timeout_s=3.0`、测试用的拷贝计时开关）作为模块级常量写在 `gs130_node.py` 内，**不暴露为 ROS 参数**——它们不是接口，暴露只会扩大契约面。

参数取值**不在 ROS 层复制校验规则**：`Config.preset` 抛 `ValueError`、`Device` 抛 `GS130Error` 时，节点把异常翻译成"参数名 + 原异常文本 + 接受范围"后以非零码退出，避免 SDK 与 ROS 两套真相（`01_requirements_review.md` Q-8/Q-10/Q-12）。

**不注册 `on_set_parameters` 回调**：所有参数都在 `gs130_init` 时下发到硬件（freeze §5.4）。运行期修改需要 stop→deinit→init 全链路重建，属 §12 明确拒绝的动态重配置；`ros2 param set` 会成功但无效果——这一点必须在 README 写明，而不是让用户以为"改了没反应是 bug"。

### 4.5 故障检测与上报

**分级与逐条文案见 freeze §7.1/§7.2；本节给出检测机制。** 三条互相独立的检测线：

| # | 检测 | 触发条件 | 上报 |
|---|---|---|---|
| D1 | **异常（硬故障）** | `read_image()`/`read_imu()` 抛 `GS130Error`（`HW_ERROR`/`THREAD_CLOSED`） | 相机通路 → `ERROR` + 退出码 1（freeze §7.2：相机是唯一数据源，坏了没有可发布内容）；IMU 通路 → `ERROR` 一次，停 T4，相机继续 |
| D2 | **静默丢失（软故障）** | 连续 `frame_timeout_s`（3 s）没有拿到帧，且**没有异常** | `WARN`（1 Hz 节流）+ T8 `level=WARN`；**不退出**（可能只是触发/曝光异常，SDK 未报错） |
| D3 | **资源/健康观测** | `log_fps_period_s`（5 s）周期 | `INFO` 统计行（`camera {fps} Hz, dropped {n}, imu {odr} Hz`，freeze §7.3 的逐字字段）+ T8（`publish_status=true` 时，1 Hz） |

关键设计点：

1. **`None` 不是故障**：`read_image()` 返回 `None` 是 SDK 的"队列空"语义（`GS130_TIMEOUT`）。因此每轮 `None` **不打日志**（2 ms 周期打日志会淹没真实信息），只计数。区分"软故障"的唯一依据是**持续时间**（D2）。
2. **`drop` 计数的含义**：`read_image()` 返回 `None` 的次数。它**不是**丢帧数——SDK 的 `camera_fifo`（深度 4、`DROP_OLD`）在队列满时自己丢旧帧，节点侧看不到。因此 T8 的 `dropped_frames` 语义在文档里定义为"poll 轮次中队列为空的次数"，避免用户拿它当"硬件丢帧"用。
3. **不自动恢复**：`GS130_THREAD_CLOSED` 的 SDK 语义是"出错后要 `deinit`+`init` 才能恢复"，v0.1.0 明确不实现自动重连（freeze §5.5）——出错就退出，让 launch/人看见，而不是在无人值守时假装正常。
4. **T8 用 `DiagnosticStatus`（不是 `DiagnosticArray`）**：freeze §7.6 固定类型与 `values` 键集合（`camera_fps`、`imu_rate`、`dropped_frames`、`stamp_offset_ns`、`mode`、`width`、`height`、`imu_present`）。**不新增键**（温度/fsync 因此不进 T8，§7）。

---

## 5. 数据流（ASCII）

`[HW]` 硬件；`[SDK]` C 库；`[PY]` Python binding；`[OURS]` 本包实现；`[D-R]` D-Robotics 现有组件（复用，不改）。

```
  [HW]  GS130 双目 MIPI (1088x1280)  +  ICM-42688-P IMU
          |
          v
  [SDK] libgs130.so  -- malloc 帧缓冲 (所有权归调用方, ndarray 释放即 free)
          |  stereo_layout=LEFT_RIGHT (preset 后改写) => 硬件拼接
          |  read_image() -> {"stitched": (H*3//2, 2W)} ; read_imu() -> ImuPacket
          |  (非阻塞; None = 队列空)
          v
  ============ 单一进程  gs130_node  [OURS] ============================
          |
          |  poll_timer (poll_period_s, 默认 0.002)
          v
     +------------------------------+   +-----------------------------+
     | 每次 tick 只读一次:           |   | drain read_imu() <= 64 包    |
     |  img = read_image()["stitched"]|  | 逐包发布, 不做滤波/积分       |
     |  height = shape[0]*2//3 (E2)  |   |                              |
     |  data = array.array("B") (E3) |   |                              |
     +------------------------------+   +-----------------------------+
          |                                     |
          | -> T1 (无手工拼接, E6)               | imu_from_packet -> T4
          | 拷进 msg.data; ndarray 释放即 free    | accel m/s^2 (含重力)
          |                                      | gyro rad/s; cov[0] = -1
          v                                     v
  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 发布 (绝对话题名, freeze §2) ~~~~~~~~~~~
    T1 /image_combine_raw   NV12  2*ew x (eh*3/2)   30 Hz   pkg 必需
    T2 /image_left_raw      NV12  ew x (eh*3/2)     默认关
    T3 /image_right_raw     NV12  ew x (eh*3/2)     默认关
    T4 /imu/data            sensor_msgs/Imu         odr (默认 200 Hz)
    T5 /image_left/camera_info   CameraInfo  启动时一次, latched
    T6 /image_right/camera_info  CameraInfo  启动时一次, latched
    T8 /gs130/status        DiagnosticStatus        1 Hz, 默认关
    (无 TF: 见 §8.3)
  =====================================================================
          |
          | T1 (NV12)
          v
  [D-R] hobot_codec_republish   channel=codec_channel
        in_mode=ros  in_format=nv12  out_mode=ros  out_format=jpeg
        sub_topic=/image_combine_raw   pub_topic=/image_combine_jpeg
        jpg_quality=80  input_framerate=-1  output_framerate=-1
          |
          | T7 /image_combine_jpeg  (sensor_msgs/CompressedImage)
          v
  [D-R] websocket 节点   image_topic=/image_combine_jpeg  image_type=mjpeg
        channel=websocket_channel  smart_topic=/image_combine_jpeg
        nginx :8000  (官方 index.html, 不改)
          |
          | MJPEG over HTTP   channel 0 = 左目, channel 1 = 右目
          v
      浏览器  http://<board-ip>:8000   [实测: HTTP 200, 30 fps]

  [D-R] hobot_shm 节点: 按 TROS 约定准备零拷贝环境 (可选前置, 本包不依赖)
  注意: nginx 由 websocket.launch.py 启动, launch 退出后仍作为孤儿进程存活
        => 关闭/重启必须显式清理 (E4, §9.3)
```

话题归属一览（**`11_interface_freeze.md` §2 为权威**；`T#` 编号与 freeze 一致）：

| # | 话题 | 类型 | QoS（实测约束） | 发布者 | 订阅者 | v0.1.0 |
|---|---|---|---|---|---|---|
| T1 | `/image_combine_raw` | `sensor_msgs/Image` (nv12) | **RELIABLE** + VOLATILE + KEEP_LAST(1) | `gs130_node` [OURS] | `hobot_codec_republish` [D-R] + 用户算法 | 必需（默认开） |
| T2 | `/image_left_raw` | `sensor_msgs/Image` (nv12) | **RELIABLE**（同 T1） | `gs130_node` [OURS] | 用户算法 | 可选（默认关） |
| T3 | `/image_right_raw` | `sensor_msgs/Image` (nv12) | **RELIABLE**（同 T1） | `gs130_node` [OURS] | 用户算法 | 可选（默认关） |
| T4 | `/imu/data` | `sensor_msgs/Imu` | BEST_EFFORT + KEEP_LAST(200) | `gs130_node` [OURS] | 用户算法 | 必需（`GS130W` 时不存在） |
| T5 | `/image_left/camera_info` | `sensor_msgs/CameraInfo` | RELIABLE + TRANSIENT_LOCAL(1) | `gs130_node` [OURS] | 用户算法 | 必需 |
| T6 | `/image_right/camera_info` | `sensor_msgs/CameraInfo` | RELIABLE + TRANSIENT_LOCAL(1) | `gs130_node` [OURS] | 用户算法 | 必需 |
| T7 | `/image_combine_jpeg` | `sensor_msgs/CompressedImage` | 由 codec 决定 | `hobot_codec_republish` [D-R] | `websocket` [D-R] | 仅 web launch |
| T8 | `/gs130/status` | `diagnostic_msgs/DiagnosticStatus` | RELIABLE(1) | `gs130_node` [OURS] | 生态工具 | 可选（默认关） |

注意本包**没有**话题级命名空间参数：T1–T7 使用 freeze 固定的绝对话题名（与既有 TROS 链路、`ros/probe_nv12_publisher.py` 的 `/image_combine_raw` 一致），只有 T8 在 `/gs130/` 下。这样 `hobot_codec_republish` 与 `websocket` 的默认参数不需要被猜改；如果要跑第二套，用 `ROS_DOMAIN_ID` 隔离，而不是改话题名（C4 下本来也不允许第二套同时碰相机）。

---

## 6. 帧处理与内存

### 6.1 T1：硬件拼接，不做手工拼接（E6）

**决策：在 `Config.preset()` 之后改写 `config.camera_config.stereo_layout = StereoLayout.LEFT_RIGHT`，由相机线程在 `resize` 模式下直接产出拼接帧。**

```python
config = gs130.Config.preset(platform, device, mode, width, height, fps, odr)
if publish_combine:
    config.camera_config.stereo_layout = gs130.StereoLayout.LEFT_RIGHT   # E6 实测可行
device = gs130.Device(config)
...
frames = device.read_image()      # -> {"stitched": Image}, shape (height*3//2, 2*width)
```

[实测 E6] `resize` + 640×480 配置下，`read_image()` 返回的 stitched 帧 shape 为 `(720, 1280)`，即 `(H*3//2, 2W)`——**恰好满足 `/image_combine_raw` 需要的 `width=2W`、`height=H`、`step=2W` 约定，无需任何手工拼接**。

为什么这是唯一正确做法（取代早期的驱动侧 `np.concatenate` 方案）：

| 维度 | 硬件拼接（选） | 驱动侧手工拼接（弃） |
|---|---|---|
| Python 侧成本 | 0（只有一次 `array.array` 填充） | 每个 tick 多一次 0.88 MB 全帧 `concatenate` |
| 左右同步 | 由相机线程保证（FSYNC 配对后一次填入） | 依赖"同一组 left/right"，仍是同一次读取但要多一次内存搬运 |
| 代码量 | 2 行（改一个字段） | 拼接函数 + 逐字节布局的单元测试 + UV 相位不确定 |
| 与官方链路 | 与 SDK 的 `LEFT_RIGHT` 布局逐字节一致（即参考 launch 的 `/image_combine_raw` 语义） | 与 SDK 布局在 UV 平面语义上有差异（早期草稿的遗留风险 R4，现已消失） |

约束与后果：

1. **拼接只在 `resize` / `rect` 模式下可用**；`mode=raw` 强制 `width/height == 1088×1280`（E7），交给 SDK 拼接会得到 2176 宽，与 SDK 硬约束冲突。因此 **`raw` 只用于标定导出**，web 链路必须用 `resize`（C13）。
2. **`stereo_layout` 与 `publish_per_eye` 互斥**：打开拼接时 `read_image()` 只返回 `{"stitched"}`，拿不到单目帧。若用户要 T2/T3，则节点必须退回 `NONE` 布局——**这是一次启动期的二选一，不做运行期切换**（§14 决策 6 与 §12 的布局规则）。默认：拼接开、`per_eye` 关。
3. 该改写**不使用** `preset()` 之外的未文档化路径：`Config`/`CameraConfig` 是普通 dataclass，`stereo_layout` 是公开字段（`python/gs130/_config.py`），只是 `preset()` 没把它暴露成参数。C7（不改 `python/`）依旧满足。
4. 左目在左、右目在右：`websocket` 的 `channel 0` 显示 `/image_combine_raw` 的左半部分 = 左目（freeze §6.2 的显示语义）。

被拒的替代方案：

| 方案 | 拒绝理由 |
|---|---|
| 驱动侧 `np.concatenate([L.reshape(-1), R.reshape(-1)])` | E6 证明不必要：硬件已经能做，手工版只增加 CPU 与一个需要维护的布局假设 |
| 让 `hobot_codec` 订阅两路单目（两个 codec / 两个 channel） | codec 是单输入模型；官方参考链路是"单路拼接 → 单 codec → 单 channel"，双路会引入未验证的 channel 行为（E7 只确认了 channel 可区分流，未确认双实例行为） |
| 只发布单目（web 只能看一只眼） | 不满足"showcase 双目硬件"的目标 |

### 6.2 ndarray → `sensor_msgs/Image` 的确切约定（E2 实测为准）

```
src: np.ndarray, shape (H*3//2, W), dtype uint8, C 连续, .timestamp_ns

msg.height      = src.shape[0] * 2 // 3   # 真实图像高 H —— 绝不能填 shape[0]（E2）
msg.width       = W                       # 拼接帧时 W = 2*W_eye
msg.encoding    = "nv12"
msg.step        = W                       # Y 行 1 字节/像素；UV 交错平面共享该 stride
msg.is_bigendian= False
msg.data        = array.array("B"); data.frombytes(memoryview(src).cast("B"))   # E3
# 不变量: len(data) == W*H*3//2 == step*height*3//2
```

- **[实测 E2，致命] `height` 必须是真实高 `H`。** 早期设计填 `H*3//2`（"把单平面 YUV420 视作 `W × (H*3/2)` 单通道图"）在真机上被证伪：`hobot_codec` 记录 `init_pic_h_: 1080, alined_pic_h_: 1088` 后**段错误（-11）**。这条推翻了 `11_interface_freeze.md` §3.1.2，**以实测为准，freeze 需要同步修订**。
- 拼接帧映射：`W = 2*W_eye`、`H = H_eye`，即 640×480 配置 → `width=1280, height=480, step=1280, len(data)=1280*480*3//2=921600`；`src.shape == (720, 1280)`，故 `height = 720*2//3 = 480`。
- `shape[0]*2//3` 与"图像高"互为逆运算：**不能用 `shape[0]`**（会得到 1.5 倍高度），也不能硬编码 `H_eye`（`rect`/`resize` 下用户改 `height` 参数就会失配）。用 `shape[0]*2//3` 是唯一与输入的表达式。
- `header.stamp` = `src.timestamp_ns` 经 §7 的启动偏移换算（T1/T2/T3 由同一次读取派生时 stamp 相同）。
- `header.frame_id`：T1 = `frame_id_combine`（默认 `camera`）；T2/T5 = `frame_id_left`；T3/T6 = `frame_id_right`。
- 测试：`test_conversions.py` 必须覆盖 `shape=(720,1280) → height=480` 与 `shape=(720,640) → height=480` 两个方向，并断言 `len(data) == width*height*3//2`（这条断言会在 E2 类 bug 上立刻失败，而不是等到 codec 段错误）。

### 6.3 拷贝与数据填充：必须用 `array.array("B")`（E3 实测）

**[实测 E3] `msg.data = ndarray.tobytes()` 是性能陷阱**：rclpy 的 setter 看到 `bytes` 没有 `.typecode`，落进 `_check_types()` 走**逐元素 Python 校验**。1280×720（1.38 MB）单帧实测：

| 写法 | 耗时 | 说明 |
|---|---|---|
| `msg.data = frame.tobytes()` | **1103.87 ms** | 逐元素 `_check_types()`，约 800× 慢 |
| `msg.data = array.array("B"); frombytes(memoryview(frame).cast("B"))` | **1.36 ms** | 唯一可用写法 |
| 仅 `frame.tobytes()`（不计赋值） | 0.31 ms | 说明瓶颈不在拷贝本身 |

**规定写法（节点内唯一允许的形式）**：

```python
from array import array

data = array("B")
data.frombytes(memoryview(src).cast("B"))   # src 为 C 连续 uint8 ndarray；1.36 ms / 1.38 MB
msg.data = data                             # array.array 有 .typecode，走快路径
```

- 仍然存在一次全帧拷贝（`frombytes` 从 ndarray 的缓冲拷进 `array` 的缓冲）——这是不可避免的（§6.3.1）；**被消除的是 rclpy 的逐元素校验**。
- 若 `src` 不保证 C 连续（当前 SDK 一定连续：`reshape` 出来的 own-data 数组），先 `np.ascontiguousarray`；不要用 `copy=False` 的假设。
- 单测应加一条"数据填充不慢于 X ms/MB"的守卫（宽松阈值，如 1.38 MB < 50 ms），把 800× 级别的回退钉死在 CI 里。

#### 6.3.1 为什么不能零拷贝

1. `sensor_msgs/Image.data` 要求拥有自己的缓冲，无法引用外部 `malloc` 地址；
2. SDK 帧缓冲的所有权是"标量所有权"（numpy 数组死即 `free`），而发布是异步的——若让消息引用该地址，索引器/`hobot_codec` 在另一个进程读到的内存可能已被 `free`（用后即崩）。

### 6.4 SDK 缓冲何时释放

- `Image` 的释放由 `weakref.finalize` 挂在底层 `ctypes` owner 上（`python/gs130/_types.py`）：**只要不再持有该帧的 numpy 视图/引用，缓冲立即 `free`**。
- 节点内的显式规则：`poll_timer` 回调是局部作用域，`frames = device.read_image()` 返回的字典在回调结束时离开作用域；`conversions.image_from_nv12()` 只读 `src`、把字节拷进 `array.array`，消息里**没有任何**对 `src` 的引用。因此每 tick 结束即释放，稳态驻留 = 1 帧 + 已发布的消息缓冲。
- 明确不做的两件事：① 不把 `src` 的 `ctypes` owner 挂到消息或节点上"以后再用"；② 不缓存上一帧做差分/复用。
- 不做 `gc.collect()`；CPython 引用计数足够。测试用一条断言覆盖"转换完成后 `src` 不再被额外引用"（`sys.getrefcount` 对比）。

### 6.5 每帧成本预算（640×480 双目的拼接帧 @30 fps）

[实测 E4] 端到端已跑通且稳定 30 fps：`/image_combine_raw` 29.966 Hz、`/image_combine_jpeg` 29.974 Hz、codec 自报 Sub/Pub 29.98 fps；页面 `http://127.0.0.1:8000` 返回 HTTP 200。单帧节点内成本：

| 步骤 | 实测/估算 | 备注 |
|---|---|---|
| `read_image()`（SDK malloc + 硬件拼接 + 拷出） | — | C 层，非我们控制 |
| `build`（消息对象 + `height/width/step` 计算） | **0.53 ms** | E4 |
| `fill`（`array.array.frombytes`，0.92 MB） | **1.28 ms** | E4 |
| `publish`（rclpy 发布调用） | **5.07 ms** | E4；DDS 侧开销为主 |
| 合计 | ≈ 6.9 ms/帧 | 30 fps → 约 20% 单核占用 |
| IMU（200 Hz，每包 <100 B） | 可忽略 | |

结论：Python 跳板在本配置下有充足余量（C6/§2 的 ament_python 决策在性能维度成立）。**若用 `tobytes()` 写法，单帧 publish 会退化到秒级**，链路直接崩掉——所以 E3 不是优化建议而是功能前提。

### 6.6 `hobot_shm` / hbmem 零拷贝是否值得（v0.1.0）

### 6.6 `hobot_shm` / hbmem 零拷贝是否值得（v0.1.0）

诚实评估：

- **SDK 侧拿不到可共享的 dma-buf**：帧是 `malloc` 出来的普通 DDR 缓冲，不是 `hb_mem` 分配的 ION/dma-buf；而 TROS 的零拷贝依赖 `hb_mem_common_buf_t` 与消息中的共享内存句柄。Python binding 不暴露该句柄，我们要"零拷贝"就必须在 C 层把 malloc 缓冲导出为 dma-buf 或改造流水线直出 `hb_mem` 缓冲——**这直接违反 C5/C7**。
- **`hobot_shm` 的作用被正确理解**：它准备零拷贝的环境（共享内存池/权限），让**支持零拷贝的节点之间**能以共享内存传图。我们的节点不在这个集合里（发布的是普通 `sensor_msgs/Image`），因此 `hobot_shm` 对我们是"为下游 codec/websocket 保持一致性"的启动步骤，而不是我们的性能手段。参考 launch 把它放在最前面的原因即在此。
- **收益量级**：约 0.88 MB/tick × 30 fps ≈ 26 MB/s 的 DDR 吞吐（再加上 rclpy 序列化），在 X5 上（数十 GB/s 级带宽）是小量但不是零；真正的成本在 ISP、单路 1280×480 JPEG 编码与 nginx 推流。

**建议：v0.1.0 不使用零拷贝，但按 freeze `hobot_shm:=true` 的默认值把 `hobot_shm.launch.py` 放进启动顺序**（它是官方链路的常规前置，成本为零，也保证将来若把图像路径换成共享内存方案时环境已经就绪）。 触发重新评估的条件：实测节点进程 CPU 在 30 fps 下 > 40% 单核（板上必须实测，见 §11.2/§13-未知3），或帧率提升到 60 fps 且输出切到 RAW。届时优先手段是**降低输出分辨率**，其次才是 C++ 零拷贝路径（§2.4）。

---

## 7. IMU 设计

字段级细节全部以 `11_interface_freeze.md` §3.3 为准（话题 `/imu/data` = T4，`frame_id_imu` 默认 `imu_link`，QoS best_effort/depth 200）。本节只记录**设计与理由**。

| 项 | 取值 |
|---|---|
| 话题 / 类型 | T4 `/imu/data`，`sensor_msgs/msg/Imu` |
| 速率 | = `odr`（freeze 参数；SDK 只接受 200/500），**不做抽稀/重采样/滤波/积分** |
| 发布条件 | `publish_imu=true` 且探测到 IMU；`device:=GS130W` 时话题不存在（一条 `INFO` 说明，不是故障） |
| 时间戳 | 与图像**同一换算公式与同一偏移量**（见下） |

字段映射与理由（`ImuPacket` → `sensor_msgs/Imu`）：

- `linear_acceleration` ← `packet.accel`（m/s²，SDK 已按 FSR 换算并含 EEPROM 标定，`core/src/gs130.cpp:139`）；**包含重力**，驱动不扣重力（freeze §3.3 要求写进用户文档）。
- `angular_velocity` ← `packet.gyro`（rad/s）；**不做符号翻转、不做轴交换**：SDK 的 IMU 坐标系就是 `frame_id_imu` 的坐标系，任何"顺手调成 REP-103"的改动都会让标定外参失去意义。
- `orientation = (0,0,0,1)` + `orientation_covariance[0] = -1.0`：**不发布姿态**。`w=1` 只是消息默认值；`-1` 是 ROS 唯一的"本消息不含姿态估计"标记。若填 0，下游会把它当作"有姿态、协方差未知"从而真的去用这个单位四元数——这是必须避免的静默错误。
- `angular_velocity_covariance[0] = linear_acceleration_covariance[0] = -1.0`：**与 `01_requirements_review.md` Q-1 的处理不同**，这里采纳 freeze：`sensor_msgs/Imu` 的协方差字段语义是**逐样本估计的协方差**，而 SDK 给的是 Allan 方差噪声密度（`accel_noise` / `gyro_noise`，单位 m/s²/√Hz、rad/s/√Hz）与随机游走；把噪声密度直接写成协方差是**量纲近似、不是真值**，会让下游 Kalman 滤波器吃到错误的权重。驱动没有逐样本协方差 → 用 `-1` 如实声明"无估计"。
  - 若将来有人想用噪声密度，正确的落点是 T8 的新增键（属契约变更）或 README，而不是伪装成协方差。
- `packet.temp` 与 `packet.is_fsync`：**丢弃，不发布、不进 T8**（freeze §3.3 的显式决定：温度无 `sensor_msgs/Imu` 字段位置，`is_fsync` 是 SDK 内部的握手/时钟同步中间量而非观测）。T8 的 `values` 键是 freeze §7.6 固定集合（`camera_fps`/`imu_rate`/`dropped_frames`/`stamp_offset_ns`/`mode`/`width`/`height`/`imu_present`），**不新增键**。需要温度的用户直接调用 SDK；若将来要暴露，属于契约变更，需要重新评审（freeze 同款措辞）。

时间戳换算（`ns` → `builtin_interfaces/Time`）——**唯一公式，以 freeze §4.2 为准，已被 E5 实测证实必要性**：

```
# 启动时（第一次拿到有效设备时间戳时）计算一次，之后恒定：
offset_ns = node_clock.now().nanoseconds - device_ts_ns     # stamp_offset_ns == 0 时
offset_ns = stamp_offset_ns                                  # 非 0 时强制

# 每条消息：
stamp_ns = device_ts_ns + offset_ns
sec = stamp_ns // 10**9 ; nanosec = stamp_ns % 10**9
```

**[实测 E5] 设备时钟是 `CLOCK_MONOTONIC`（自开机起算），不是 Unix epoch**：实测 `frame_ts_ns = 9546841822000`，同时刻 `time.monotonic_ns() = 9540690105753`（与 uptime 9480 s 吻合），而 `frame_ts - time.time_ns() ≈ -1.79e18`。图像与 IMU 同域（`imu_ts - frame_ts ≈ 8.6 ms`）。

因此：

- **绝不能把 `timestamp_ns` 直接写进 `header.stamp`**：monotonic 的 `sec ≈ 9546` 会被当成 1970-01-01 之后的 9546 秒，RViz/`ros2 bag`/`tf` 的时间轴全错位，且不会报错——属于静默错误。这批实测数据把早期"时间基未知，可能有 wall 概率"的猜测变成确定结论。
- `offset_ns` 的作用就是把 monotonic 平移到 ROS 时钟（`RCL_ROS_TIME`）。
- **采样误差必须写进用户文档**：`offset_ns` 由"第一个有效设备时间戳"与"当时的节点时钟"两个采样点决定，两者之间存在**不超过一个帧周期（默认 33 ms）**的偏移误差；它是一次性常量，因此**不影响任何相对量**：相邻消息的时间差仍严格等于设备时间差（这正是 `message_filters`、VIO 需要的东西）。若不接受绝对时刻的这几十毫秒误差，可用 `stamp_offset_ns` 显式指定（配合离线标定）。
- 正因为误差是常量，**禁止**用 `now()` 逐帧覆盖或滑动平均"修正"它——那会把一个常量误差换成一个抖动误差，破坏相对时间关系。

实现约束（"容易写错"的点，评审重点）：

1. 偏移量只算一次、之后恒定；同一个偏移量同时服务图像与 IMU（`conversions.StampOffset` 实例）——SDK 的 `TimestampTracker` 已把 IMU 对齐到相机主时钟，我们只做一次线性平移，因此 **T1/T4 的时间戳可直接比较**（E5 实测 `imu - frame = 8.6 ms`）。
2. `device_ts_ns == 0` 的帧：照发，`stamp` = 最近一次有效值 + `1e9//fps`（保持单调），`WARN` 节流 1 Hz；**不用它去确定 offset**。
3. 时间戳回退（非单调）：IMU 包丢弃 + `WARN`；图像照发但不改 offset。
4. `use_sim_time=true` 时：`WARN` 并按 `use_sim_time:=false` 语义继续（偏移量在仿真时钟下不可复现）。
5. 启动时 `INFO` 打印一次 `stamp_offset_ns=<值> mode=<auto|forced>`；`stamp_offset_mode:=device` 跳过换算（仅用于回放/离线分析，文档标注"时间戳非 ROS 时钟"）。
6. 绝对时刻的合理性可自检：换算后的 `stamp` 与 `node.get_clock().now()` 的差值应在**一个帧周期 + 少量调度抖动**内；若偏差大于 1 s，说明 offset 取错（例如把 IMU 的首个包当成了图像的时间戳基准），此时 `ERROR` 并拒绝继续发布错误时间戳。这条自检在板测里作为断言（§11.2）。

---

## 8. 标定设计

### 8.1 内参 → `CameraInfo`（以 freeze §3.4 为准）

话题 T5 `/image_left/camera_info`、T6 `/image_right/camera_info`，`sensor_msgs/msg/CameraInfo`，QoS **RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1)**（latched：晚到的订阅者也能拿到，这是 ROS 2 的 `camera_info` 惯例）。启动时各发布 **1 次**，不逐帧重发。两条消息由同一个纯函数生成，只差 `CameraIndex` 与 `frame_id`。

| 字段 | 取值 | 理由 |
|---|---|---|
| `header.frame_id` | T5: `frame_id_left`（默认 `camera_left`）；T6: `frame_id_right`（默认 `camera_right`） | 必须与对应 Image 话题的 `frame_id` **逐字相同**，否则 `image_geometry` 的 `CameraInfo`/`Image` 配对失效 |
| `header.stamp` | 启动时刻 | 标定不随时间变化 |
| `height` / `width` | `height` 参数 / `width` 参数（默认 480 / 640） | 单目输出尺寸，与 T2/T3 一致；**不是** T1 的拼接尺寸 |
| `distortion_model` | 见下表 | 固定映射 + `mode=rect` 强制覆盖 + `camera_info_distortion_model` 参数覆盖 |
| `d` | fisheye → `dist_coeffs[0:4]`；pinhole → `dist_coeffs[0:5]`；`rect` → 5 个 0；`camera_info_distortion_model:=none` → `[]` | ROS 的 `equidistant` 是 4 元、`plumb_bob` 是 5 元；**多余系数如实丢弃并写进用户文档** | 
| `k` | SDK `K` 展开（9 个） | 直接来自 SDK，**不做任何缩放**（§1.3 已从源码确认：非 RAW 模式下 SDK 在 `Pipeline::init()` 里按 VSE ROI+scale 把内参写回 `cal->cam_left/right`，`gs130_get_camera_intrinsics()` 读的就是这份写回值） |
| `r` | 9 个 0 | **不是**单位阵：freeze §3.4.2 规定 `R`/`P` 在三个模式下全部上报 0（"本驱动不做立体校正、不提供校正矩阵"） |
| `p` | 12 个 0 | 同上；`tx=0` 也不表达基线 |
| `binning_x` / `binning_y` | 0 | 无 binning |
| `roi` | 全 0，`do_rectify=false` | 无 ROI |

`distortion_model` 与 `d` 的唯一映射（freeze §3.4.1）：

| 条件 | `distortion_model` | `d` |
|---|---|---|
| `mode ∈ {raw, resize}` 且 SDK 报 `DistModel.FISHEYE` | `"equidistant"` | `[k1,k2,k3,k4]`（4 元） |
| `mode ∈ {raw, resize}` 且 SDK 报 `DistModel.PINHOLE` | `"plumb_bob"` | `[k1,k2,p1,p2,k3]`（5 元） |
| `mode = rect` | `"plumb_bob"`（强制覆盖） | `[0,0,0,0,0]` |
| `camera_info_distortion_model := none` | 由参数决定 | `[]`（空数组） |

为什么 `rect` 要强制覆盖：SDK 的 `stereo_rectify()` 会把 `dist_coeffs` **原地清零**，但 `dist_model` 枚举仍是 `FISHEYE`；此时图像已经没有畸变，若照抄 `equidistant`，下游会再去做一次鱼眼去畸变。所以这是一条**显式规则**，不是猜测。实现必须在代码注释里保留这条理由（freeze §3.4.1 的要求）。

**`R`/`P` 全 0 的影响必须让评审知道**：`image_proc` / `stereo_image_proc` 这类依赖 `P` 做投影的节点在 `P` 全 0 时不可用；本包的正确用法是"用 `K` + `D` 自己做去畸变/投影"（T5/T6 的价值就在这里）。这是 freeze 的显式决定（宁可为空也不填半正确的值），本设计不反对，但把它列入 §12 的"已知后果"而不是悄悄带过。

**必须实测的一条**（`01_requirements_review.md` Q-2 的闭环）：`resize` 模式下 `fx` 应与 `width` 同量级（640 宽时约 300~800；1088 宽时约 600~1300）。板测断言即检查这一点（§11.2）。源码已确认 SDK 会回写缩放后的内参，板上这一条是**确认**而不是探索；若实测不符，则本节的"不缩放"结论作废，改为按 `sx=out_w/sensor_w` 缩放后填 `K`，并同步修改 freeze（**这是 v0.1.0 前必须闭环的一个点**）。

### 8.2 T1（拼接帧）为什么没有 `CameraInfo`

`11_interface_freeze.md` §2 已显式删除 stitched 的 `camera_info`：拼接帧含两个视场，**不存在**与之对应的单一相机模型，发布 `/image_combine/camera_info` 会是错误数据。内参由 T5/T6 承担。这与本设计的判断一致（§6.1 也据此选择了"拼接在驱动侧做、内参按单目发布"）。

### 8.3 外参：**不发布 TF**，改为"可读的标定数据 + 探测器命令"

**结论（服从契约）：v0.1.0 不发布 `/tf` 与 `/tf_static`，`tf2_msgs` 不进依赖表。** 依据 `11_interface_freeze.md` §0.2 与 §5.5（`publish_tf` 被列入"故意不提供的参数"）。理由是 freeze 给出的：SDK 的外参语义在 `mode=rect` 下会变成"虚拟平行双目系"（`core/src/gs130.cpp`/`gs130.h` 的外参说明），需要一个单独的评审才能定义"发布哪条边、父是谁、虚拟化时如何标注"，v0.1.0 不引入半正确的 TF。

本设计**接受该决定**，并指出它留下的空白与填补方式——因为"展示标定/外参"是任务书的明确要求（`01_requirements_review.md` US-06/US-07、AC-07）：

| 需求 | v0.1.0 的满足方式 |
|---|---|
| 拿到内外参数值 | T5/T6 的 `CameraInfo`（内参）+ `gs130_probe`（全部外参矩阵） |
| 拿到相机-相机基线 | `gs130_probe` 打印 `relative_T(CAMERA_RIGHT, CAMERA_LEFT)` 与其模长 |
| 拿到 IMU-相机外参 | `gs130_probe` 打印 `relative_R/T(CAMERA_*, IMU)` |
| 拿到 `dist_model` / `install_angle` | `gs130_probe` 打印 + T8（`publish_status`）不涉及 |
| VIO/SLAM 建系 | **v0.1.0 不承诺**：用户可自己从 `gs130_probe` 的数值 + `static_transform_publisher` 建 TF，本包不代劳（不发布半正确的 TF） |

`gs130_probe`（`gs130_ros.gs130_probe`，US-11/AC-11）因此从"可选的诊断工具"升级为**外参需求的正式载体**：

- 输出内容：`platform`、`libgs130` 版本、`gs130` pip 版本、`available_camera()`/`available_imu()`、`imu_name`/`eeprom_name`、左右目 `K`/`dist_model`/`dist_coeffs`、`install_angle`、三对 `relative_R`/`relative_T`（含基线模长）。
- 相机不可用/被占用时：打印 `camera busy` 关键词 + `GS130Error.code`，退出码非零（AC-10/AC-11）。
- 它**确实会短暂持有相机**（`Device()` 会探测 I2C/EEPROM），因此必须在 README 与 `--help` 里写明"独占，勿与 `gs130_node` 同时运行"，并在文档中给出推荐用法：`ros2 run gs130_ros gs130_probe`（节点未运行时）。

如果评审认为"必须有 TF 才算 showcase 外参"，本文档给出的下一步是：先按 §2 的评审流程在 freeze 中新增一节定义 3 条静态边（`imu_link→camera_left`、`imu_link→camera_right`、`camera_left→camera_right`）与 `mode=rect` 的虚拟化标注，再实现。**方向换算的结论先记在这里，避免将来实现时取反**：

- SDK 语义：`relative_R(from, to)` 是"把 *from* 系下的点变换到 *to* 系"的旋转，即 *from* 在 *to* 系中的姿态。
- ROS 语义：`T_parent_child` 是 child 在 parent 中的位姿。
- 因此 `imu_link → camera_left` 这条边应取 `relative_R(CAMERA_LEFT, IMU)` / `relative_T(CAMERA_LEFT, IMU)`，**直接使用，不求逆**；若改取 `relative_R(IMU, CAMERA_LEFT)`，则必须 `R'=Rᵀ`、`T'=-RᵀT`。届时的四元数换算用自写 Shepperd 法（约 15 行，模长归一 + `w ≥ 0`），**不依赖 `tf_transformations`**（该包在 humble 中的提供方式不稳定，属不必要依赖）；v0.1.0 不实现这段代码，因为不发布 TF。

`install_angle`：v0.1.0 **只打印、不参与任何计算**。SDK 头文件只有 `int camera_install_angle;`，没有单位说明（度/弧度）与作用说明，因此不能折算进任何矩阵或旋转。这是对 `01_requirements_review.md` Q-5 的明确结论。

`convert_calibration` 的暴露问题见 §8.4。

## 9. 错误处理与关闭

**错误分级、逐条日志文案与退出码以 `11_interface_freeze.md` §7 为准（唯一版本）**，本节只补充架构层面的机制与理由，不重抄文案。

### 9.1 场景矩阵（架构视角）

| 场景 | 检测点 | 分级（freeze §7.2） | 机制 |
|---|---|---|---|
| `import gs130` 失败（wheel 未装） | 模块顶层 `try/except ImportError` | 参数/环境错误 → 退出码 2 | 打 stderr 可执行指引（`pip3 install gs130`、`source /opt/tros/humble/setup.bash`）；**不创建 ROS 上下文、不导入 rclpy 之外的东西**，因此失败很快、不会留下半初始化状态 |
| `libgs130.so` 找不到（`GS130_LIB` 未设） | `Device()` 抛 `OSError` | 致命 → 退出码 1 | 原样贴出 `_runtime` 的异常文本（含 `GS130_LIB` 提示），不自己猜路径 |
| 参数非法（platform/device/mode/尺寸组合） | `Config.preset` 抛 `ValueError` / `gs130_init` 抛 `PARAM_ERROR` | 参数错误 → 退出码 2 | 翻译成 `invalid parameter {name}={value}: {reason}`；**不在 ROS 层预校验**（避免两套真相） |
| 相机被占用（`mipi_cam` 在跑 / 上次进程未退） | `init` 失败（`NOT_FOUND`/`HW_ERROR`） | 致命 → 退出码 1 | 日志含固定可搜关键词 `camera busy` + `pgrep -af 'gs130\|mipi_cam'` 提示；**不自动 kill 别人的进程、不 respawn** |
| `start()` 卡在 IMU FSYNC 握手 | `start_timeout_s`（默认 10 s） | 不致命 | 每 10 s 打一条 `WARN`，**不中断** `start()`（SDK 无法安全中断，强行 close 属未定义行为）；此阶段 SIGINT 只能等它返回（freeze §7.5） |
| 运行期 `read_image()` 抛 `GS130Error` | 轮询回调内 `except` | 致命 → 退出码 1 | 相机是唯一数据源，通路坏了没有可发布内容；停在原地只会骗人。按 freeze：不实现自动重连（SDK 要求 `deinit`+`init` 才能恢复） |
| 运行期 `read_imu()` 抛 `GS130Error` | 轮询回调内 `except` | 非致命 | `ERROR` 一次 → **停止发布 T4**，相机与 T1 继续（T8 报 `ERROR`） |
| `read_*` 返回 `None` | 正常语义（队列空） | 非致命 | 本轮不发消息，只累加 `drop` 计数；**不打日志**（2 ms 周期打日志会淹掉真实信息），只在 `log_fps_period_s` 的统计行与 T8 里体现 |
| 连续无帧超过 `frame_timeout_s`（3 s，实现常量） | 统计定时器 | 非致命 | `WARN`（1 Hz 节流）"no frames for N s"；这是**观测**，不是退出条件（可能只是曝光/触发异常，SDK 未报错） |
| 无 EEPROM（`publish_camera_info=true`） | `camera_intrinsics` 抛 `NOT_FOUND` | 非致命 | `WARN` 一次，跳过 T5/T6；图像与 IMU 照常 |
| 无 IMU（`GS130W` 或 `imu_name is None`） | 启动探测 | 非致命 | `WARN` 一次 `IMU not present; /imu/data will not be published`；不创建 T4 相关发布者 |

### 9.2 SIGINT / SIGTERM → stop → close

```
main():
  rclpy.init()
  node = None
  try:
      node = Gs130Node()          # Device 在这里创建；失败即 raise（→ 退出码 1/2）
      rclpy.spin(node)            # Ctrl+C → KeyboardInterrupt（rclpy 默认信号处理器）
  except KeyboardInterrupt:
      pass
  finally:
      if node is not None:
          node.shutdown()         # 见下，3 秒预算内完成
      if rclpy.ok():
          rclpy.shutdown()
  return exit_code                 # 正常关闭 = 0（freeze §7.4）
```

`Gs130Node.shutdown()` 的**顺序是硬要求**（`core/include/gs130.h` 明确写"后台线程仍在运行时 `gs130_destroy()` 属未定义行为"），且必须在 3 秒内完成：

1. `self._poll_timer.cancel()`、`self._stats_timer.cancel()`（先让回调不再进入——这一步必须在 `stop()` 之前，否则可能出现"线程已停、回调还在读"）；
2. `self._device.stop()`（`gs130_stop()`：停流并 join 相机/IMU 线程；幂等）；
3. `self._device.close()`（`gs130_deinit()` + `gs130_destroy()`：复位并断电 MIPI 传感器、释放 I2C/EEPROM；幂等）；
4. `self._device = None`；`self.destroy_node()`。

- **显式顺序、不依赖 `__del__`**：`Device.__del__` 与 `__exit__` 都在（`python/gs130/_device.py`），但 GC 时机不确定，`shutdown()` 必须自己走完 2–3 步（freeze §7.4 的明文要求）。
- **关闭过程中禁止再调用 `read_image()`/`read_imu()`**：定时器已 cancel，且用 `self._closing` 标志让任何漏进来的回调直接返回。
- 幂等：`shutdown()` 由 `self._closed` 保护，`finally` 中重复调用是空操作。
- `close()` 自身抛错：`ERROR` 记录但仍以 0 退出（设备可能已释放），并按 freeze 打印"可能需要断电重启相机"的提示。

### 9.3 不残留进程占用相机（C4 的落地）

- **正常路径**：`finally` 保证 `close()` 一定执行。
- **异常路径**：即使 `finally` 未跑到（`os._exit`、SIGKILL），**进程退出即由内核关闭 `camera_handle` 与 MIPI/I2C 文件描述符**，`libgs130` 的线程随进程消亡；`deinit()` 里做的传感器复位/断电不会执行，但下一次启动仍能正常探测（这也是 §8.3 里"残留占用"的真正含义：**上一次的进程还活着**，而不是 fd 泄漏）。
- 运维手段（写进 README，不做自动化 kill）：
  1. 初始化失败时日志给出 `pgrep -af "gs130|mipi_cam"` 供人判断；
  2. launch 文件不使用 `respawn=True`，也不隐藏子进程（让 launch 退出时节点的退出码与日志可见）；
  3. 文档明确"先停 `mipi_cam` 再起 `gs130_node`"。
- **明确不做**：不在节点内定时 `kill` 其他进程、不 `rmmod` 驱动、不为 SIGTERM 之外的信号写特殊处理。SIGTERM 的清理依赖 rclpy 的信号处理链（humble 下会把 SIGTERM 映射为关闭请求，`spin()` 返回后走同一 `finally`）；这一行为将在板测 §11.2 的"关掉再起"步骤中被真实验证。

## 10. Launch 设计

**参数名、默认值、节点组成与逐字参数值以 `11_interface_freeze.md` §6 为准**。本节记录"为什么这样组成"，以及实现时必须守住的几条规则。

### 10.1 两个 launch 及职责

| 文件 | 组成 | 职责 |
|---|---|---|
| `gs130_camera.launch.py` | 只有 `gs130_node` | 无 web 链路；**不依赖 `hobot_codec`/`websocket` 是否存在**（它们是 TROS 可选组件，缺了相机节点照样要能跑） |
| `gs130_web.launch.py` | `hobot_shm`（可选前置）+ `gs130_node` + `hobot_codec_republish` + `websocket` | 浏览器看图：T1 NV12 → T7 JPEG → nginx:8000 官方页面 |

### 10.2 为什么**不 include** D-Robotics 的 launch 文件

`share/hobot_codec/launch/hobot_codec_encode.launch.py`、`share/websocket/launch/websocket.launch.py`、`share/mipi_cam/launch/mipi_cam_dual_channel_websocket_132gs_nocal+cal+r90.launch.py` 都存在，但我们**逐个显式声明 `Node(...)`**，理由：

1. **参数耦合面更小**：include 别人维护的 launch 会把它的参数名/结构定义变成我们的隐式契约，对方改名（或换实现）我们静默失败；而我们只需要 9 个已知参数（`channel/in_mode/in_format/out_mode/out_format/sub_topic/pub_topic/jpg_quality/input_framerate/output_framerate`）。
2. **`mipi_cam` 的 launch 会启动 `mipi_cam` 节点**：那就是同时占用相机（违反 C4），绝对不能 include。
3. **唯一例外是 `hobot_shm.launch.py`**：它是纯环境准备（设置零拷贝相关环境变量），没有业务参数、没有话题，include 它既能满足"按 TROS 约定准备环境"，又不引入耦合——这也正是 D-Robotics 参考 launch 的写法。`hobot_shm` 参数作为开关（默认 `true`）暴露在 `gs130_web.launch.py`。

### 10.3 实现时必须守住的规则（容易写错的点）

1. **`input_framerate` / `output_framerate` 必须是 `-1`**（不抽帧，跟随输入）。设成小于 `fps` 的值会与 web 侧 `output_fps` 叠加抽帧，出现"帧率莫名变低"。freeze §6.2 已固定为 `-1`；[实测 E4] 该组合下 codec 自报 Sub/Pub 29.98 fps、`/image_combine_jpeg` 29.974 Hz，链路已跑通。
1b. **入图 QoS 固定 `RELIABLE`，launch 里不要引入任何会把它降级的东西**（E1）：`hobot_codec` 以 `RELIABLE` 订阅，若我们的发布者用 `BEST_EFFORT`，codec 会打印 `offering incompatible QoS ... RELIABILITY_QOS_POLICY` 并**一条都不收**。`shm_fastdds.xml` 不设置 reliability，没有规避手段——这条**不能**做成可配参数。
2. **`codec_channel ≠ websocket_channel`**（默认 0 / 1）：`hobot_codec_republish` 的 `channel` 是它内部的通道号，`websocket` 的 `channel` 是页面上的显示通道号，两者语义不同但都从 0 起，若取同一个值会出现"页面看不到画面而 topic 有数据"。
3. **`only_show_image=False`**（freeze §6.2）：保持官方页面默认行为（保留页面上的其它元素与智能结果显示位），不自作聪明地改成只显示图像。
4. **`sub_topic`/`pub_topic`/`image_topic`/`smart_topic` 用绝对话题名**，与 T1/T7 逐字一致（`/image_combine_raw`、`/image_combine_jpeg`）。
5. **launch 参数是字符串**：`width/height/fps/odr/poll_period_ms/...` 必须显式转 `int(...)`；`publish_*` 交给 `launch_ros` 的参数求值处理。不做隐式转换依赖。
6. **我们不启动 nginx**：端口 8000 由既有 webservice 承载；`web_port` 只用于启动前提示与文档（freeze §5.1）。但 [实测 E4] `websocket` 的 launch 会通过 `os.system` 拉起 nginx 且它会在 launch 退出后存活——因此"清理 nginx 孤儿"是我们运行/验收流程的一部分（§9.3），即使代码里没有一行启动 nginx 的语句。
7. **不设 `respawn=True`**：设备忙时会变成重启风暴（freeze §7.2）。

### 10.4 启动顺序与前提

1. `source /opt/tros/humble/setup.bash`（提供 `hobot_codec`/`websocket`/`hobot_shm` 与 `rclpy`）；并确保 `gs130` wheel 已安装（`pip3 install gs130`）。
2. 确认 `mipi_cam` 未运行（C4）。
3. `ros2 launch gs130_ros gs130_web.launch.py ...`（逐字命令见 freeze §6.6）。
4. 浏览器 `http://<board-ip>:8000`，`channel 0` = T1 的左半部分 = **左目**；`channel 1` = 右目（E7：`websocket` 用同一个 8000 端口按 `channel` 区分多路流，**不需要第二个端口**，D-Robotics 132GS 参考 launch 就是 channel 0/1）。显示语义必须写进 README（页面不标注哪边是左目，用户会猜）。
5. 停止时按 §9.3 处理 **nginx 孤儿**（E4），再确认相机已释放。

### 10.5 已由实测关闭的 launch 疑点

E4 已把原"待确认清单"里的关键项关闭，实现时按结论写即可，不要再按猜测设计：

| 原疑点 | 实测结论 |
|---|---|
| `hobot_codec` 是否要求共享内存输入 | **不要求**：普通堆上的 `sensor_msgs/Image`（`encoding=nv12`）即可，整链 30 fps、页面 200（E4）。`hobot_shm` 仍按约定 include，但不是功能前提 |
| codec 参数组合是否成立（`input_framerate=-1` 等） | **成立**：E4 下 codec 自报 Sub/Pub 29.98 fps，`/image_combine_jpeg` 29.974 Hz |
| 是否需要第二个端口 | **不需要**：`websocket` 在同一 8000 端口用 `channel` 区分流（E7，132GS 参考 launch 即 channel 0/1） |
| 页面能否出图 | **能**：`curl http://127.0.0.1:8000` 返回 200（E4） |
| nginx 是否需要我们启动/清理 | 我们**不启动**，但必须**清理**它的孤儿进程（E4/C14/§9.3） |

仍未验证但不影响架构的：`jpg_quality=80` 下 1280×480 的实际码率与 CPU 占用（默认 80，实测偏高再降）；`publish_per_eye` 同时开启时的双路开销。

## 11. 测试策略

### 11.1 主机（无硬件、无 TROS）可测：`test/test_conversions.py`

`conversions.py` 被刻意设计为**无 rclpy 状态、无设备依赖的纯函数层**：输入是 numpy 数组与 SDK 的 dataclass（`ImuPacket`/`CameraIntrinsics`/`Calibration` 都是可直接构造的 frozen dataclass，`Image` 是 ndarray 子类，测试里可用等价对象构造），输出是 ROS 消息。**所有"字段对不对"的缺陷都在这一层被拦住**，这是主机测试的主要价值。

覆盖清单（标 [E#] 的是实测回归，必须在 CI 里长期存在）：

1. **T1 几何 [E2]**：输入 shape `(720, 1280)` → `height == 480`、`width == 1280`、`step == 1280`、`len(data) == 921600`；输入 shape `(720, 640)` → `height == 480`、`width == 640`、`len(data) == 460800`。**必须包含"`msg.height != src.shape[0]` 且 `msg.height == src.shape[0]*2//3`"的断言**——E2 的段错误正是这条不变量被破坏的结果。
2. **数据填充路径 [E3]**：断言 `isinstance(msg.data, array.array)`、`msg.data.typecode == "B"`，且实现不出现 `bytes` 赋值路径；加一条性能守卫（1.38 MB 帧构造 < 50 ms，宽松阈值即可把 800× 级回退钉住）。
3. **QoS 不可降级 [E1]**：断言 T1/T2/T3 publisher 的 `reliability == ReliabilityPolicy.RELIABLE`（E1 的不兼容拒绝是静默的：topic 有数据、页面黑屏，所以只能靠断言防回归）。
4. **T2/T3 几何**：`height == H_eye`、`width == W_eye`、`step == W_eye`、`len(data) == W_eye*H_eye*3//2`。
5. **同帧同 stamp**：同一份输入派生 T1/T2/T3 时 `header.stamp` 相等，`frame_id` 分别为 `frame_id_combine`/`frame_id_left`/`frame_id_right`。
6. **内存**：转换完成后 `src` 不被消息引用（引用计数断言），保证 §6.4 的释放契约。
7. **时间戳换算 `StampOffset` [E5]**：首个有效时间戳只确定一次 `offset_ns`；之后 `stamp = device_ts + offset`；**相邻 stamp 之差与设备时间差严格相等**（这条是 E5 的核心保证）；`device_ts == 0` 时用"上次 + `1e9//fps`"且保持单调；回退时只 WARN 不改 offset；`stamp_offset_ns != 0` 时强制使用；`nanosec < 1e9`。
8. **`CameraInfo` 映射**：fisheye → `"equidistant"` + 4 元 `D`；pinhole → `"plumb_bob"` + 5 元 `D`；`mode=rect` → 强制 `"plumb_bob"` + 5 个 0；`camera_info_distortion_model:=none` → `D == []`；`k` 与 SDK `K` 逐元素相等；`r`/`p` 全 0；`binning_*`/`roi` 全 0；T5/T6 的 `frame_id` 不同。
9. **`Imu` 字段**：`orientation == (0,0,0,1)`、三个协方差 `[0] == -1.0`、其余为 0；`angular_velocity`/`linear_acceleration` 与 `ImuPacket` 逐值相等（不做轴交换/符号翻转/单位换算）；`frame_id == frame_id_imu`。
10. **外参数值**：`relative_R(LEFT, IMU)` 近似正交（`‖RᵀR − I‖_∞ < 1e-6`）、`det ≈ +1`；`‖relative_T(RIGHT, LEFT)‖` 落在合理基线范围（数值健全性检查；v0.1.0 不发布 TF，见 §8.3）。

运行：`python3 -m pytest ros/gs130_ros/test/test_conversions.py`。依赖仅 numpy + ROS 消息的 Python 接口；若主机没有 `rclpy`/`sensor_msgs`，模块顶部 `pytest.importorskip` 整体跳过并打印原因（**不静默通过**）。

### 11.2 板测（必须）：`test/test_gs130_hardware.py`

结构：**一个端到端用例 + teardown 断言**，不做单元化拆分（避免为测试引入抽象层）。E1–E7 已把主要未知变成已知，因此板测的职责是**守护这些实测结论不被回归**（尤其是 E2 的段错误与 E1 的静默拒收）。

```
setup_module:  import gs130, rclpy; rclpy.init()
               config = Config.preset(platform, device, mode, width, height, fps, odr)
               config.camera_config.stereo_layout = StereoLayout.LEFT_RIGHT   # E6
               尝试 Device(config)
                 失败（被占用/不在位）→ pytest.skip("camera busy or absent: <GS130Error>")
                 # skip 而非 fail：同一块板会被多人/多轮使用，且这正是 AC-10 的现场
               device 保留在模块作用域（一个进程、一次独占）
test_stream_and_publish（直接调 conversions + 真实 Device，不建 ROS 图）:
    1. device.start()（注意可能等 IMU FSYNC 握手）
    2. 取 30 帧或 5 s 超时：
         frames = device.read_image(); None → sleep(poll_period_ms) 继续
         断言 set(frames) == {"stitched"}（E6：拼接布局下不应出现 left/right）
         断言 img.shape == (H*3//2, 2*W)（E6 实测 640x480 时为 (720, 1280)）
         msg = conversions.combined_image(img, ...)
         断言 §11.1-1 的几何（height == H，绝不是 shape[0]）与 array.array 填充路径
         断言 timestamp_ns 单调不减，且 stamp 与 now() 之差 < 1 s（E5 的 offset 自检）
    3. 实际 fps > 0.8 * 配置 fps（E4 实测 29.97 Hz @30 配置）
    4. IMU（imu_name 非空时）：收 2 s 包，断言 accel/gyro 有限、非全零、
       timestamp_ns 单调、条数 > 0.5 * odr * 2；
       断言 |imu_ts - frame_ts| < 2 * (1e9/fps)（E5 实测同域，差 8.6 ms，不得是秒级）
    5. 标定：camera_intrinsics(LEFT/RIGHT).K 与 freeze 的模式一致性表相符
       （resize 640 宽时 100 < fx < 1200；左右目 K 不同）；
       relative_R(LEFT, IMU) 正交；‖relative_T(RIGHT, LEFT)‖ 在 (0, 0.5] m
    6. 单目路径（可选，需重启设备）：stereo_layout=NONE 时返回 {"left","right"}，
       且 height == H（与 T1 相同的 height 规则）
teardown_module: device.stop(); device.close(); rclpy.shutdown()
    断言 device.closed is True（关闭契约的机器可验证部分）
```

**图级冒烟（人工步骤，逐字写进 README，不自动化）**——E4 已证明该链路可达 30 fps 且页面 200，下列命令是回归检查清单：

```
source /opt/tros/humble/setup.bash
ros2 launch gs130_ros gs130_web.launch.py
ros2 topic hz /image_combine_raw                      # ≈ fps（E4: 29.966 Hz）
ros2 topic echo /image_combine_raw --once --field encoding   # nv12
ros2 topic echo /image_combine_raw --once --field height     # == 真实高（E2；不是 H*3//2）
ros2 topic info -v /image_combine_raw | grep -i reliab        # RELIABLE（E1）
ros2 topic hz /image_combine_jpeg                     # ≈ fps（E4: 29.974 Hz）
ros2 topic hz /imu/data                               # ≈ odr
ros2 topic echo --once /image_left/camera_info        # D 长度/CameraInfo 合理
ros2 topic list | grep -c tf                          # 期望 0：本包不发布 TF（§8.3）
curl -s -o /dev/null -w '%{http_code}' http://127.0.0.1:8000   # 200（E4）
浏览器打开 http://<board-ip>:8000                      # channel 0 = 左目，channel 1 = 右目
# 停止与再启动（同时覆盖相机释放与 nginx 孤儿，E4/C14）
Ctrl+C  → 3 s 内退出、退出码 0；pgrep -af gs130_node 无输出
pgrep -af nginx                                       # 可能仍有孤儿，按 §9.3 清理
ss -ltnp | grep 8000                                  # 需要时确认端口已释放
再次 ros2 launch 必须成功                             # 证明相机已释放（AC-09/AC-13）
```

**最后三条是本项目最重要的验收标准**：同时验证 §9.2 的关闭契约、C4 的独占释放、以及一次真实的"改代码 → 重启 → 页面仍出图"闭环。

### 11.3 主机 / 板上的分工原则

- 主机测"**消息是否被正确构造**"（纯逻辑：字段、几何、单位、映射、时间戳算术）——这是缺陷密度最高的一层，且完全不需要硬件。
- 板上测"**SDK 与硬件是否按文档行为工作**"：帧形状与时间戳单调性、内参坐标系与模式一致性、实机 fps、独占与释放、以及 `hobot_codec`/`websocket` 这条复用链是否真的通。
- **任何需要真实 `Device` 的断言都不放进主机测试**：不用 mock 伪造 SDK（那会掩盖真实契约——`Image` 的 shape、`None` 语义、`GS130Error` 都是我们依赖的契约面）。

## 12. 明确的非目标与被拒方案

契约层面的非目标（`11_interface_freeze.md` §0.2/§5.5）不在下表重复；下表是**架构选型的取舍记录**。

| 非目标 / 被拒方案 | 理由 |
|---|---|
| 写 C++ 驱动（`rclcpp` + 直接链接 `libgs130`） | 违反 C5/C7：绕过唯一受支持入口，重复实现设备生命周期与 malloc 所有权管理；§2.4 给出了改判条件 |
| 自写 web 节点 / 自写 MJPEG 推流 / 自建静态页 | 违反 C1/C3；`websocket` 已提供 nginx:8000 + 官方页面，自建即重复开发且会被镜像升级甩开 |
| 自写 NV12→JPEG 编码（OpenCV / libjpeg） | 违反 C2；`hobot_codec_republish` 就是做这件事的，且走硬件/库加速 |
| 发布 TF（`/tf`、`/tf_static`） | **契约决定**：freeze §0.2 明确 v0.1.0 不发布 TF，`tf2_msgs` 不进依赖表（`mode=rect` 下外参会被虚拟化，需要单独评审）。外参需求由 `gs130_probe` 承担（§8.3）。若评审要求发布，按 §8.3 给出的 3 条边与方向规则补齐后再实现 |
| 自己发布点云 / 自己跑立体匹配 / 自己拼深度 | 违反 C1；`hobot_stereonet` 是官方深度网络包。本包只提供 NV12（T1/T2/T3）与 `CameraInfo`（T5/T6） |
| 在驱动侧手工拼接双目（`np.concatenate` / OpenCV） | **已被 E6 证伪为不必要**：`preset()` 后改写 `stereo_layout=LEFT_RIGHT` 即可让硬件直接产出拼接帧，手工版只增加每帧 0.88 MB 搬运、一段需要维护的布局假设，以及与 SDK 布局的 UV 相位差异。**v0.1.0 不写任何拼接代码** |
| 用 `mode=raw` 跑 web 链路 | E7 + SDK 硬约束：`raw` 强制 1088×1280，且拼接后宽度 2176 与约束冲突；`raw` 只作为标定导出用途 |
| 让 codec 订阅两路单目（两个 codec 实例、两个 web channel） | codec 是单输入模型；官方参考链路是"单路拼接 → 单 codec → 单 channel"。E7 只确认了 `channel` 能在同一端口区分流，未确认双 codec 实例行为，不值得为它承担未验证风险 |
| 用 `msg.data = ndarray.tobytes()` 填数据 | **E3 实测慢约 800×**（1.38 MB 帧 1103.87 ms vs `array.array` 1.36 ms）：rclpy 对 `bytes` 走逐元素 `_check_types()`。这不是"可选优化"，是链路能否跑起来的前提 |
| 图像 QoS 用 `BEST_EFFORT` / 做成可配参数 | **E1 实测**：`hobot_codec` 以 `RELIABLE` 订阅，`BEST_EFFORT` 会被判定 QoS 不兼容而**一条不收**（页面黑屏，topic 侧却看起来正常）。`shm_fastdds.xml` 不设 reliability，无法绕过 |
| 动态参数重配置（`on_set_parameters` + 参数回调） | 所有参数都在 `gs130_init` 时下发硬件，运行期变更需要 stop→deinit→init 全链路重建，语义是"重启节点"而不是"改参数"（freeze §5.3）；引入它只会带来状态机与并发风险 |
| 自动重连 / 自动重启（`respawn=True`、出错后 `deinit`+`init` 重试） | SDK 的错误语义要求显式的 `deinit`+`init` 恢复；v0.1.0 明确"出错 → 退出 → 让 launch/人看见"（freeze §5.5、§7.2）。设备忙时 respawn 会变成重启风暴 |
| 多实例（同节点多设备 / 多进程共享相机） | 违反 C4；硬件独占，SDK 亦无多句柄语义 |
| 生命周期节点（lifecycle node） | §4.1：无外部 supervisor、设备独占，收益为零、样板代码增多 |
| 自己实现零拷贝（`hobot_shm`/`hbmem` 直通） | §6.6：SDK 帧是 malloc 缓冲、binding 不暴露 dma-buf 句柄，要做得改 C 层（违反 C5/C7），收益量级偏小 |
| 发布 `orientation`（姿态）或做 IMU 积分/滤波 | SDK 只提供原始六轴；积分/融合是下游的职责。freeze §3.3 固定为"不发布姿态" |
| 发布 IMU 温度 / `is_fsync`（话题或 T8 字段） | freeze §3.3 显式丢弃：无标准字段位置、且 `is_fsync` 是 SDK 内部中间量。T8 的 `values` 键集合已固定，不新增键 |
| 逐帧重发 `CameraInfo` | 标定不随时间变化；latched 各发一次即可（freeze §2 的 T5/T6） |
| 图像缩放/降采样（用 OpenCV 在节点内 resize） | 缩放应由 SDK 的 `resize` 模式（VSE）完成，且内参会随之正确写回；在 Python 里缩放是纯粹的浪费 |
| 自定义 msg/srv 包 | 标准消息已足够表达；新增 msg 包会带来构建类型与版本管理成本（freeze N-06） |
| `camera_mode=rect` 作为默认 | `resize` 是默认（freeze §5.1）；`rect` 作为可选参数存在，其"外参虚拟化"必须在文档中标注（§8.3） |

### 已知后果（不是缺陷，但必须让用户知道）

1. **`CameraInfo.R`/`P` 全 0**：`image_proc`/`stereo_image_proc` 类依赖 `P` 的节点不可用；用户要用 `K`+`D` 自己投影（§8.1）。
2. **T1 是"内存拼接"而非"几何校正拼接"**：左目在左、右目在右，接缝处不做融合/校正（硬件按 `LEFT_RIGHT` 布局填充，§6.1）。
3. **无 TF**：任何需要 `tf2` 的链路（RViz 的 fixed frame、`message_filters` 的 TF 对齐）在 v0.1.0 不可用；RQT 的图像查看器可用（§8.3）。
4. **`ros2 param set` 无效果**：参数在 `init` 时下发硬件；会在 USER 文档中写明（§4.4）。
5. **绝对时间戳带一次性采样误差**（≤ 一个帧周期）：相对时间严格正确，绝对时刻可能偏几十毫秒（E5/§7）；需要精确绝对时刻时用 `stamp_offset_ns` 标定。
6. **`stereo_layout` 与 `publish_per_eye` 二选一**：默认拼接（web 必需）；要 T2/T3 就用 `NONE` 布局启动，二者不能并存（§6.1）。
7. **`raw` 不适配 web 链路**：`raw` 只用于标定导出（E7/§14 决策 7）。

---

## 13. 风险与缓解

**E1–E7 已消掉了大部分原风险**（原 R2 的 codec 兼容性、R3 的时间基、R4 的 UV 相位、以及"能否 30 fps"的性能未知都已实测关闭）。下表只保留仍然开放的风险。

| # | 风险 | 影响 | 缓解 | 状态 |
|---|---|---|---|---|
| R1 | **相机独占冲突**：`mipi_cam` 或残留进程占用 → 初始化失败 | 启动失败；现场最常见的抱怨 | freeze §7.1 文案 + `camera busy` 关键词 + `pgrep` 指引；README 顶部写明互斥；不用 `respawn` | 开放（环境类） |
| R2 | **QoS 被"顺手"改回 `BEST_EFFORT`**（E1 的失败是静默的：topic 有数据、codec 不收、页面黑屏） | 功能整体失效且难定位（codec 日志里才有一行 incompatible QoS） | T1–T3 的 reliability **写死不可配**（C10）；主机测试断言 publisher 的 `ReliabilityPolicy.RELIABLE`；README 记下 codec 的那行日志作为排障关键词 | 已用断言缓解 |
| R3 | **`height` 被写成 `shape[0]`**（E2：codec 段错误 -11） | 进程崩溃，且崩溃在**下游**（codec），现象会误导到别处 | `height = shape[0]*2//3` 作为唯一表达式；主机测试断言 `msg.height != src.shape[0]`；板测的 `ros2 topic echo --field height` 检查 | 已用断言缓解 |
| R4 | **数据填充回退到 `bytes` 路径**（E3：单帧 publish 从 6.9 ms 退化到秒级） | 链路直接崩（帧率归零、页面卡死） | 规定唯一写法（`array.array("B")` + `frombytes`）；主机测试断言 `typecode == "B"` + 性能守卫 < 50 ms/帧 | 已用断言缓解 |
| R5 | **nginx 孤儿导致"画面来自旧实例"**（E4：launch 退出后 nginx 存活） | 改完代码重启却看不到变化，误判为代码问题 | §9.3 的显式清理流程 + README 停止/重启章节 + 板测"关掉再起"步骤含 `pgrep nginx` / 端口检查 | 已用流程缓解 |
| R6 | **板上环境未就绪**（`pip install gs130` 未做 / `libgs130.so` 路径未配 / 未 `source` TROS） | 节点导入即失败或找不到 `hobot_codec` | §9.1 的导入错误处理给出可执行指引；README"前提"段写明三条（source TROS、装 wheel、确认 `mipi_cam` 未运行） | 开放（部署类） |
| R7 | **1280×480 @30 fps 下的余量**：E4 实测单帧 ≈6.9 ms（≈20% 单核），但未测 `fps=33` 上限与 `publish_per_eye` 同时开启 | 高负载时掉帧 | 默认关闭 `publish_per_eye`；`raw` 不用于 web；若实测吃紧，退化路径是把 `fps` 降到 15 或只编码一路（配置变化，不动代码） | 开放（性能余量） |
| R8 | **SIGTERM 路径**（`docker stop`/`kill`）依赖 rclpy 信号链把关闭请求映射为 `spin()` 返回 | 极端情况下 `deinit()` 未执行（传感器未复位） | 内核会关闭 fd、进程退出即释放；文档要求用 `Ctrl+C` 或 `ros2 launch` 正常退出；板测含"关掉再起" | 开放（平台行为） |
| R9 | **freeze §3.1.2 与 E2 冲突**（freeze 要求 `height = eh*3//2`，实测会段错误） | 若实现照 freeze 写，链路必崩 | **以实测 E2 为准**（`height = H`）；在本文档 §6.2/§14 显式记录"freeze 需同步修订"，评审时把这条列为需回写 freeze 的变更项 | 契约待修订 |
| R10 | **`mode=rect` 下外参被虚拟化**，与物理标定不一致 | 用户用 `gs130_probe` 的数值建 TF 会与实际不符 | `gs130_probe` 在 `mode=rect` 时打印醒目提示"extrinsics are virtual (rectified parallel stereo)"；默认 `resize` | 开放（可选路径） |
| R11 | **IMU FSYNC 握手阻塞 `start()`**（SDK 无超时参数） | 启动看起来"卡住"；此阶段 SIGINT 也不能 3 秒退出 | freeze §7.5：启动前 INFO、每 `start_timeout_s`（10 s）WARN、**不中断调用**；`device:=GS130W` 时不进入握手 | 开放（SDK 行为） |

### 仍然开放的技术未知（按优先级）

E1–E7 关闭了原清单的第 1、2、3 项（codec 兼容性、时间基、Python 跳板性能）。剩下的：

1. **`fps` 上限余量**：`line_length=1400 / frame_length=1500` 下约 33 fps 是传感器读出上限（freeze §5.1），但**拼接 + 两路 JPEG + web 推流**在 `fps=33` 或 `width/height` 更大（如 1088×1280 的 `resize`）时的余量未测。它决定 README 里"已验证组合"能给到多宽。
2. **`publish_per_eye=true` + 拼接布局的取舍**：目前设计为二选一（§6.1）。若下游同时要"web 看双目"和"算法拿单目"，是否需要两个进程/两次启动，还是需要 SDK 侧支持同时输出（可能要动 binding，属 C5/C7 之外的需求）——这是**需求问题**，不是实现问题。
3. **`mode=rect` 的实际表现**：`rect` 下 `R`/`P` 全 0、外参虚拟化、`D` 强制 5 个 0，图像确实无畸变但下游拿不到校正矩阵。是否有用户真的需要 `rect` 作为一等路径（而非 `resize` + 自行去畸变），需要需求侧确认。

---

## 14. 八条冻结决策（由板上实测确定）

以下八条是**实现必须逐条遵守**的结论；每条都标注了依据与"若违反会怎样"。前七条来自 E1–E7 的实测，第八条来自 E4 的进程行为。

| # | 冻结决策 | 依据 | 违反的后果 |
|---|---|---|---|
| 1 | **T1–T3 图像 QoS 用 `RELIABLE`**（VOLATILE、KEEP_LAST(1)），不做成参数 | E1：`hobot_codec` 以 `RELIABLE` 订阅；`shm_fastdds.xml` 不设 reliability，无法绕过 | codec 打印 `offering incompatible QoS ... RELIABILITY_QOS_POLICY` 并**一条不收**；topic 侧看起来正常，页面永久黑屏 |
| 2 | **`Image.height = src.shape[0] * 2 // 3`**（真实高 H），`width` 为真实宽（拼接时 `2W`），`step = width` | E2：填 `H*3//2` 时 codec `init_pic_h_: 1080, alined_pic_h_: 1088` 后**段错误 -11** | 下游 codec 崩溃（-11），且崩溃点不在本包，排障方向会被带偏 |
| 3 | **`msg.data` 用 `array.array("B")` + `frombytes(memoryview(src).cast("B"))`**，禁止 `ndarray.tobytes()` 直接赋值 | E3：1.38 MB 帧 bytes 赋值 **1103.87 ms**，`array.array` 路径 **1.36 ms**（≈800×）；rclpy 对 `bytes` 走 `_check_types()` 逐元素校验 | 单帧 publish 从 6.9 ms 退化到秒级，30 fps 链路直接崩 |
| 4 | **复用既有 `hobot_codec_republish` + `websocket`，不自写 codec/web** | E4：整链实测 30 fps（raw 29.966 Hz / jpeg 29.974 Hz，codec Sub/Pub 29.98 fps），页面 HTTP 200 | 重复开发、失去官方链路的维护与镜像兼容（C2/C3） |
| 5 | **`header.stamp = sdk_ts + offset_ns`**，`offset_ns` 在启动时用"节点时钟 − 首个设备时间戳"算一次并**保持不变** | E5：设备时钟是 `CLOCK_MONOTONIC`（`frame_ts=9546841822000` vs `monotonic_ns=9540690105753`，`frame − wall ≈ −1.79e18`） | 直接把 SDK 纳秒写进 `header.stamp` 会把 monotonic 秒数当成 Unix 时间，RViz/bag/tf 时间轴全错且**不报错**；逐帧 `now()` 覆盖则会抹掉曝光间隔、破坏相对时间 |
| 6 | **T1 用硬件拼接**：`preset()` 后设 `config.camera_config.stereo_layout = StereoLayout.LEFT_RIGHT`，`resize` 模式下 `read_image()` 返回 `{"stitched": (H*3//2, 2W)}`；**不写任何手工拼接代码** | E6：640×480 配置下实测 shape `(720, 1280)`，恰好满足 `width=2W, height=H, step=2W` | 自己 `concatenate` 每帧多搬 0.88 MB，并引入需要维护的 UV 相位假设（E6 证明纯属多余） |
| 7 | **默认 `mode=resize`（640×480，`fps=30`）；`raw` 仅用于标定导出** | E7：`raw` 强制 `output_width/height == 1088×1280`，否则 `gs130_init` 返回 `GS130_PARAM_ERROR`；且拼接后宽度 2176 与 `raw` 约束冲突 | `raw` 下 web 链路无法成立；`rect`/`resize` 之外的模式选择会让 `width/height` 参数被 SDK 拒绝 |
| 8 | **停止/重启必须显式清理 nginx 孤儿**（`websocket.launch.py` 用 `os.system` 启动 nginx，launch 退出后它继续存活） | E4：launch 退出后 nginx 仍在，端口 8000 仍被占用 | 重启后页面可能由**旧 nginx 实例**提供（看起来"改了没生效"），且 `ros2 launch` 不会报错 |

**对 `11_interface_freeze.md` 的回写请求**（本文件不能改那份契约，但评审需要同步）：

1. §3.1.2 的 `height = eh*3//2` → **必须改为 `height = H`**（E2 实测段错误）。T1 的 `width/step/len(data)` 规则不变。
2. §2 的 T1–T3 QoS：`BEST_EFFORT` → **`RELIABLE`**（E1 实测，否则 codec 拒收）。
3. §3.1.3 的手工拼接构造规则 → 改为"**由 SDK 硬件拼接**（`stereo_layout=LEFT_RIGHT`），驱动不做字节拼接"（E6）。
4. §5.5 中"`stereo_layout` v0.1.0 固定为 `NONE`" → 需改为"**默认 `LEFT_RIGHT`（web 链路必需）；`NONE` 是 `publish_per_eye` 的配套布局**"。
5. 新增一条实现级要求：`msg.data` 必须用 `array.array("B")` 填充（E3），否则性能不可接受；以及"停止流程需清理 nginx 孤儿"（E4）。
