# 52 C++ 节点架构（TEAM-AUTHORS/B1 交付，攻击目标）

本文件是 v0.1.0 C++ 节点的**实现契约**：任何一条技术论断都必须可被证伪
（指名线程、函数、缓冲区所有者、实测数字）。与 `00_verified_platform_facts.md`
（E1–E10）冲突之处以 E 系列为准；本文件对 `50_cpp_design.md` 的 3.2/3.3/3.4
三节给出**修正后的结论**，并在 §2.6、§3.4、§5.5 写出被推翻的理由。

标注约定：

- 「实测」= 有板端原始记录（`00_verified_platform_facts.md` / `25_test_report.md`）。
- 「待测」= 本架构要求板端判决性实验给出数字（§10）。
- 「未验证」= 有明确依据的推断，但**尚无**板端证据，不得当作已验证使用。

代码现状（本文件必须与之一致，`ros/gs130_ros/` 已存在未提交的在建实现）：
`src/camera_node.cpp` 531 行、`src/conversions.cpp` 113 行、`src/preset.c` 39 行、
`include/gs130_ros/{conversions.hpp,preset.h}` 92 行、`CMakeLists.txt` 已有雏形。
本文件同时**列出与在建实现的 6 处偏差**（§11），供评审直接对账。

---

## 1. 模块与文件布局

包路径 `ros/gs130_ros/`，ament_cmake，`C`+`CXX`（`preset.c` 是 C 文件，见 §8.3）。

| 文件 | 行数估算 | 一行职责 |
|---|---|---|
| `CMakeLists.txt` | 40 | ament 构建：`find_library(gs130)`、可执行、gtest 注册、安装 launch |
| `package.xml` | 30 | 依赖：`rclcpp` `sensor_msgs` `geometry_msgs` `tf2_ros` `ament_cmake_gtest` |
| `include/gs130_ros/preset.h` | 40 | 声明 `gs130_ros_preset()`：把 `gs130_define.h` 的宏包成可返回错误的 C 函数 |
| `include/gs130_ros/conversions.hpp` | 55 | 声明纯函数：`to_stamp` / `distortion_model` / `to_camera_info` / `to_quaternion` / `to_transform` |
| `include/gs130_ros/frame_buffer.hpp` | 70 | `Nv12Frame`（RAII，持 `malloc` 指针并 `free`）+ `nv12_size()/nv12_height()` 几何纯函数 |
| `src/preset.c` | 45 | 唯一接触 `GS130_CONFIG_RDKX5_*` 宏的文件（宏含 GNU range designator，C++ 任何标准模式都不接受）；不调用 `exit()`，返回 `-1` |
| `src/conversions.cpp` | 120 | 上述纯函数实现（无节点、无设备、无 `gs130_device_t`） |
| `src/frame_buffer.cpp` | 60 | `Nv12Frame` 的 `free` 唯一出口 + 几何换算（`height` 真实高，见 E2） |
| `src/camera_node.cpp` | 560 | `Gs130Camera` 节点：参数、校验、设备生命周期、定时器、发布、看门狗、关停 |
| `src/camera_node_main.cpp` | 70 | `main()`：`rclcpp::init` → 构造 → `spin` → 异常映射为退出码 → `shutdown()` |
| `launch/gs130_camera.launch.py` | 35 | 只起相机节点（参数全部显式声明为 launch 参数） |
| `launch/gs130_web.launch.py` | 85 | 相机 + 既有 `hobot_codec` + 既有 `websocket`；退出时显式清理 nginx（E4/D8） |
| `test/test_conversions.cpp` | 200 | gtest：CameraInfo/畸变模型/四元数/TF 方向/时间戳换算 |
| `test/test_frame_buffer.cpp` | 140 | gtest：`Nv12Frame` 的 `free` 恰一次（用计数 allocator 钩子）、几何契约 |
| `test/test_preset.c` | 60 | gtest（C 文件）：`gs130_ros_preset()` 对未知 platform/device 返回 `-1` 且不退出 |
| `tools/measure_pipeline.cpp` | 180 | 板端测量工具：分段计时（pop/copy/publish/serialize），§10 实验用 |

合计约 **1815 行**（其中 C++ ≈ 1600、C ≈ 105、Python launch ≈ 120）。
行数为**估算**，由在建实现（`camera_node.cpp` 531 行已覆盖本节功能）外推。

**不写的东西**：不做零拷贝/`hb_mem`/loaned message，不做动态参数重配置，
不做自研编码器或网页，不接 `hobot_stereonet`（与 `50_cpp_design.md` §4 一致）。

---

## 2. 执行模型（主攻击目标）

### 2.1 结论

**v0.1.0 使用「1 个执行线程 + 3 个 `create_wall_timer`」，不引入自研采集线程**；
但该结论是**有条件的**（§2.6 给出触发条件与切换路径）。SDK 自身已有 2 个线程，
节点**不自建线程**：

| 线程 | 谁创建 | 干什么 | 阻塞点 |
|---|---|---|---|
| `camera_thread` | SDK `gs130_start()` → `std::thread(camera_thread_func, dev)` | FSYNC 握手后循环：`malloc` 每目缓冲 → `pipeline->get_frame()`（超时 100 ms）→ 半帧对齐（丢弃较旧目并重取）→ `camera_fifo->push()` | `get_frame(...,100)`，最坏 100 ms；`Fifo::push` 的 `std::mutex` |
| `imu_thread` | SDK `gs130_start()` → `std::thread(imu_thread_func, dev)` | 读硬件 FIFO（最多 128 包/次）、FSYNC 握手、`TimestampTracker` 校正、`imu_fifo->push()` | I2C 读；`Fifo::push` 的 `std::mutex` |
| **节点主线程 / executor** | `rclcpp::spin(node)`（`SingleThreadedExecutor`） | 3 个 timer 回调 + `tf2_ros` 静态广播；**所有** `gs130_get_*` 调用都在这里 | 见 §2.4：`publisher->publish()` |

单线程执行器的选择是刻意的：`gs130_get_nv12_frame/_get_stereo_nv12_frame/_get_imu_packet`
都会先取 `dev->mtx`（`core/src/gs130.cpp:561/579/607`），多线程执行器**不会**破坏一致性，
但会让「图像回调」与「IMU 回调」并发，IMU 包的出队顺序与 `header.stamp` 单调性不再由
一个线程保证；`50_cpp_design.md` §3.2 提的「RMW 回调里 memcpy 会阻塞 IMU」在本模型里
不成立，因为**只有一个回调在跑**。

### 2.2 三个定时器（周期与依据）

| 定时器 | 周期 | 依据 |
|---|---|---|
| `image_timer_` | `1/max(2*fps,10)` s → 默认 30 fps 时 **60 Hz**，16.67 ms | 2× 过采样：SDK 的相机 FIFO depth=4 且 `GS130_FIFO_DROP_OLD`（`gs130_define.h:41`），一次 tick 最多 `kMaxFramesPerTick=2` 帧；2 倍余量保证即使 tick 抖动到 33 ms 也只会积压 1 帧而不会丢帧（丢帧只发生在连续 4 个帧周期无 tick，即 133 ms 卡死） |
| `imu_timer_` | `1/max(2*odr,20)` s → 默认 odr=200 时 **400 Hz**，2.5 ms | 同样 2× 过采样；每 tick 最多 `kMaxPacketsPerTick=64` 包，与 Python 一致；`imu_fifo` depth=1024，即使 IMU 回调被图像拷贝拖住 1 s 也不丢包（200 Hz × 1 s = 200 < 1024） |
| `report_timer_` | 5.0 s | 心跳 + E8 看门狗（§7.4） |

**读驱动方式是轮询，不是回调**：SDK 只提供非阻塞读（`GS130_TIMEOUT` = 现在没数据）。
`gs130_available_camera()/gs130_available_imu()`（`gs130.cpp:545/593`）只在需要日志时才查，
热路径直接用 `gs130_get_*` 的返回值驱动循环，省一次加锁。

### 2.3 一次迭代做什么（`Gs130Camera::poll_images()`）

```
for (i = 0; i < 2; ++i) {                      // 最多 2 帧/tick
  Nv12Frame frame;                             // data=nullptr，RAII
  code = stitched ? gs130_get_stereo_nv12_frame(dev_, &frame.raw)
                  : gs130_get_nv12_frame(dev_, &left.raw, &right.raw);
  if (code == GS130_TIMEOUT) return;           // 无事可做
  if (code != GS130_OK)      { 故障处理 §7.3; return; }
  publish_frame(frame);                        // 构造消息 + 拷贝 + publish
}                                              // 出作用域 → free()，恰一次
```

关键实现事实（`gs130.h` 的契约 + `core/src/gs130.cpp:567`）：

- 只有 `GS130_OK` 时结构体才被填充；`GS130_TIMEOUT` 时 SDK 不动 `*image`。
  因此**必须**把结构体清空后再调用，且**只**在 `GS130_OK` 时 `free`（在建实现
  `camera_node.cpp:343` 已如此）。
- 拼接与非拼接是**两个互斥 API**：`stereo_layout != NONE` 时 `gs130_get_nv12_frame`
  返回 `GS130_UNSUPPORTED`（`gs130.h:232`）。

### 2.4 1920x1080@30 的单次迭代预算

拼接帧（`left_right`，`width=1920`）缓冲区大小实测式：
`W*H*3/2 = 1920*1080*3/2 = 3,110,400 B`（单目，`gs130.cpp:252`）；拼接缓冲是
`malloc(width*height*3) = 1920*1080*3 = 6,220,800 B`（`gs130.cpp:264`）。
下文所有 MB 均为十进制 `10^6`，字节数为准。
按 1280x720 实测「build 0.53 ms / fill 1.28 ms / publish 5.07 ms」线性外推
（拷贝量与耗时成正比，这是本轮外推的**唯一假设**）：

| 阶段 | 1280x720 实测 | 1920x1080 外推 | 状态 |
|---|---|---|---|
| `Fifo::pop` + 结构体赋值 | 计入 0.53 ms | ≈0.5 ms | 未验证（锁竞争 +2 线程） |
| 消息构造（`Image` 字段 + `data.resize`） | 0.53 ms | ≈1.2 ms | 未验证 |
| `data.assign(frame.data, +size)`（memcpy 6.22 MB） | 1.28 ms | **≈3.5 ms** | 待测（§10 实验 2） |
| `publisher->publish()`（含 RMW 序列化，再拷 6.22 MB） | 5.07 ms | **≈10 ms** | 待测（§10 实验 1/2） |
| **合计/帧** | **6.88 ms** | **≈15 ms** | 帧周期 33.3 ms 的 45% |
| 30 fps 下每秒占用 | 206 ms | **≈450 ms** | 占空比 45% |

叠加项（必须在板端量，否则上面这张表是乐观的）：

1. **每帧 2 次大 `malloc`**（NONE 布局）或 1 次 6.2 MB `malloc`（拼接布局），
   `free` 在 publish 之后。glibc 默认 `M_MMAP_THRESHOLD`=128 KB，3.1 MB/6.2 MB
   必然走 `mmap`/`munmap`，于是**每帧**都要重新缺页、由内核清零 3.11 MB（单目）
   或 6.22 MB（拼接）。
   推断成本 ≈0.75–1.5 ms/帧（按 2 GB/s 清零），**未验证**。缓解手段：
   `main()` 早于 `gs130_create()` 调 `mallopt(M_MMAP_THRESHOLD, 64<<20)`，
   让 SDK 的 `malloc` 走 `brk` 堆而复用已缺页的页。这是**改动最小的优化**，
   但它修改的是**进程全局** allocator 行为（SDK 与节点共用 libc），
   需要在 §10 实验 2 里 A/B 对比，不能默认打开。
2. IMU 定时器 400 Hz：每秒 400 次回调，其中 200 次真正取到包（odr=200）。
   空回调约 10–30 µs/次（唤醒 + 锁），合计 ≈4–12 ms/s，**未验证**。
   注意 6.2 MB 的 `memcpy` 会把 L2 冲干净，第 2 秒的 IMU 回调会变慢；
   这也是「按需扩大 timer 周期」优先于「加线程」的原因。
3. `publish()` 的隐藏风险见 §2.5。

### 2.5 `publish()` 会阻塞吗（本章最脆弱的一点）

`rclcpp::Publisher::publish()` 在 `SingleThreadedExecutor` 里是**同步**的：
它把消息 CDR 序列化进 RMW 的 buffer，然后交给 DDS 线程/引擎。
`50_cpp_design.md` §3.3 问的「publish 返回是否代表已经发出」——**不代表**，
但**本设计不需要它代表**：消息载荷在序列化后已与我们的缓冲区解耦（§3）。

阻塞场景（必须实测，不能假设）：RELIABLE + `KeepLast(1)` 的 publisher，
慢订阅者（`hobot_codec` 被 JPEG 编码拖慢、或订阅端队列满）时，
RMW 的 `write()` 路径可能阻塞在 DDS 线程的异步队列/`max_blocking_time` 上，
或在消息被 `history=1` 挤掉时做额外工作。
若单次 `publish()` 最坏值 > 33 ms，本 tick 的 IMU 包会积压（depth 1024 兜住），
但**下一帧会迟到**，30 fps 破裂。**阈值判据**：
若 60 s 内 `image_timer` 回调耗时的 p99.9 > 16.67 ms 或最大 > 33 ms → 采用 §2.6 方案 B。

### 2.6 什么时候必须加线程（推翻「单线程一定够」的表述）

`50_cpp_design.md` §3.2 的「单线程 + 两定时器」在 640x480 实测链路（29.997 Hz，E4）
下成立。但那个实测是**脚本合成帧**，不是 SDK `malloc`+拼接 6.2 MB 的真实路径，
所以不能拿它证明 1920x1080 安全。触发条件与切换路径写在这里，避免评审把
「单线程」当成无条件结论：

- **方案 B（推荐的首选升级）**：`image_timer_` 回调只做
  `pop` → `memcpy` 进一个**预分配环形缓冲**（`image_timer_` 不再调 `publish`），
  由 `publish_timer_`（同线程、周期 1/fps）做 `publish`。这仍是单线程，只是把
  「可能阻塞的 publish」与「必须准时的取帧」在时间上解耦，代价是最多 1 帧延迟。
- **方案 C**：`MultiThreadedExecutor` + 两个 `MutuallyExclusive` callback group
  （图像组、IMU 组），+ 1 个自研采集线程专做 `pop`+`memcpy`。
  只在方案 B 仍不够、或需要同时跑第二个订阅者回调时才用。

---

## 3. 帧缓冲所有权（主攻击目标之二）

### 3.1 谁分配、谁持有、何时释放

事实链（全部可在源码中核对）：

1. `camera_thread_func` 每帧 `malloc`：`gs130.cpp:252/253`（NONE，左右各一块
   `width*height*3/2`）、`:264/274/284/294`（四种拼接布局，**一块**
   `width*height*3`，另一目是同块的指针偏移）。`frame[0].data` 恒等于
   **该次 `malloc` 的返回值**，所以 `free(frame.data)` 对四种拼接布局都合法。
2. 帧被 `camera_fifo->push(frame)` 入队（`:335`）。若 push 满了，按
   `GS130_FIFO_DROP_OLD` 走 `disposer_(buf_[tail_])`，即 **FIFO 自己调
   `free_frame_pair()`** 把被顶掉的那对帧 `free` 掉（`fifo.hpp:80`）。
3. `gs130_get_*` **pop** 出来（`gs130.cpp:567/585`）：`Fifo::pop` 只做
   `item = buf_[tail_]` 的拷贝，**不**调 disposer。所以 **pop 之后所有权 100% 归调用方**，
   SDK 侧不再有任何指针能再次 `free` 它（生产者只把新帧写进新 malloc 的地址）。
4. `gs130_deinit()` 里 `dev->camera_fifo.reset()`（`gs130.cpp:506`）触发
   `~Fifo()`（`fifo.hpp:36`）把**仍在队列里**的帧 `free`。生产者/消费者已 join，
   所以这是安全的，且**要求节点不能再持有任何指向队列内帧的指针**。

**因此生命周期精确表述为**：
`malloc`（camera_thread，入队前）→ 有效期至「被 pop」或「被 DROP_OLD 顶掉」或
「deinit 清队」（三者互斥，任一发生即所有权转移或释放）→ 调用方在 pop 成功后
**必须且恰好 `free()` 一次其 `data`** → `free` 之后该指针立即失效，
**不得**再读写、不得二次 `free`、不得跨帧缓存。

### 3.2 C++ 实现方式：RAII，`free` 只有一个出口

```cpp
struct Nv12Frame {                 // frame_buffer.hpp
  gs130_image_nv12_t raw{};        // 清零：SDK 在 TIMEOUT 时不写它
  Nv12Frame() = default;
  ~Nv12Frame() { release(); }
  Nv12Frame(const Nv12Frame&) = delete;            // 禁止隐式拷贝（双 free 的根因）
  Nv12Frame& operator=(const Nv12Frame&) = delete;
  void release() { if (raw.data) { free(raw.data); raw.data = nullptr; } }
};
```

非拼接模式下左/右帧来自**两次独立 malloc**，必须**各自** free（在建实现
`camera_node.cpp:363-364` 正确；`publish_frame` 里如果抛异常会漏掉 free，
这是本轮评审要抓的点，见 §11 偏差 3）。

### 3.3 为什么消息载荷不依赖 SDK 缓冲

`publish_frame()` 的关键三行（在建 `camera_node.cpp:377-388`）：

```cpp
auto message = std::make_unique<sensor_msgs::msg::Image>();
message->data.assign(frame.data, frame.data + size);   // 独立 std::vector<uint8_t>
publisher->publish(std::move(message));                 // 序列化后再交给 RMW
// 调用方 free(frame.data) —— 载荷已与 SDK 缓冲无关
```

`assign` 之后 `message->data` 是**消息自己的**堆内存；`publish()` 返回时
序列化已完成。所以 **`free()` 在 `publish()` 返回之后立刻调用是安全的**，
理由不是「publish 已经发出去了」，而是「我们不再向 RMW 提供指向 SDK 缓冲的指针」。

### 3.4 是否必须自定义 allocator 或拷贝：**必须拷贝**（结论）

- SDK 的 `malloc` 调用点在 `core/src/gs130.cpp` 的 SDL 内部，
  `gs130_config_t` **没有**任何 allocator/deleter 钩子（`gs130.h:63-119` 全字段已核），
  所以**无法**让 SDK 直接分配进 ROS 消息的缓冲区；**自定义 allocator 不可行**。
- `sensor_msgs::msg::Image::data` 是 `std::vector<uint8_t>`（强类型，不能接管外部
  `malloc` 内存），**零拷贝**在当前 API 下不可实现（除非上 `hb_mem`，已列为非目标）。
- 于是每次迭代**必然**至少 1 次 6.22 MB 拷贝（1920x1080 拼接）。这是本设计的
  固有成本，写在 §2.4 预算里，不做「看似零拷贝」的优化。

**我们防的失效模式（三种，都是崩溃级）**：

1. **Use-after-free**：把 `frame.data` 的裸指针交给消息（例如 `data` 用
   `memcpy` 到预分配缓冲后又把 `frame.data` 存起来复用），随后 `free`，
   订阅者/编码器读到已释放内存 → 随机花屏或 SIGSEGV（E2 已证明 `hobot_codec`
   在几何字段错误时会 `exit code -11`，说明它**直接**读缓冲区）。
2. **Double free**：同一指针既被节点 `free`，又被 FIFO 的 `disposer_`
   （DROP_OLD / `~Fifo`）`free`；或 `Nv12Frame` 被拷贝导致两次析构。
   glibc 报 `double free or corruption`，进程 abort，**相机被残留持有**（E8 的
   前车之鉴：节点死亡不影响流水线，下一个进程会静默抢帧）。
3. **跨帧缓存**：把上一帧指针留在成员里「下帧复用」，而 `deinit` 已清队 → 悬垂。

**每帧 errno 检查不适用**：`malloc` 失败时 SDK 走 `goto free_frame`
（`gs130.cpp:255/258`），**不会**返回半空帧；节点侧只需在 `frame.data==nullptr`
时跳过（在建实现 `camera_node.cpp:375` 已做）。

**未验证点**：`publish()` 在 KEEP_LAST(1) + RELIABLE 下是否可能**同步等待**订阅端 ACK
（若会，则「free 在 publish 后」仍然安全，但延迟会跳变）。这是 §10 实验 1 的判据。

---

## 4. 标定路径（CameraInfo 与静态 TF）

### 4.1 用到的 C 调用（全部在 `gs130_start()` 之后、`gs130_init()` 之后调用）

| 调用 | 产出 | 失败码 |
|---|---|---|
| `gs130_get_camera_intrinsics(dev, GS130_CAMERA_LEFT_IDX, &L)` | `fx,fy,cx,cy` + `K[9]` + `dist_coeffs[8]` + `dist_model` | 无 EEPROM → `GS130_NOT_FOUND` |
| `gs130_get_camera_intrinsics(dev, GS130_CAMERA_RIGHT_IDX, &R)` | 同上 | 同上 |
| `gs130_get_relative_R(dev, GS130_REF_CAMERA_RIGHT, GS130_REF_CAMERA_LEFT, R9)` | 3x3 行主序旋转 | `GS130_NOT_FOUND` |
| `gs130_get_relative_T(dev, ..., T3)` | 3 平移（米） | 同上 |
| `gs130_get_relative_R/T(dev, GS130_REF_IMU, GS130_REF_CAMERA_LEFT, ...)` | IMU 外参 | 无 IMU → `GS130_PARAM_ERROR`（`frame_extrinsics` 返回 nullptr 分支） |
| `gs130_get_imu_name(dev)` | `nullptr` = 无 IMU → 不发 `imu_link` TF | — |

**方向已用源码核实**：`gs130_get_relative_R` 实现是 `mat33_t_mul(to,from,R)`
即 `R = R_to^T * R_from`，`gs130.cpp:695` 注释与 `gs130.h:390` 的说明一致：
返回值把 `from_frame` 的点变到 `to_frame`。所以
`gs130_get_relative_R(RIGHT, LEFT)` 描述 **right 在以 left 为参考系下的位姿**，
正好是 `parent=camera_left, child=camera_right` 的 TF。板端实测
`Translation: [0.070, 0.000, -0.002]`（基线 0.070316 m），与「左目为父、右目在 +x」
自洽。**矩阵按行主序存放**；`conversions.cpp` 里用 `m[r*3+c]` 取值，
若改用 `Eigen::Map<Matrix3d>` 必须显式 `RowMajor`，否则转置错误（易错点）。

### 4.2 `CameraInfo` 逐字段映射

设 `W,H = width_, height_`（**单目输出尺寸**，不是拼接宽），`fm = frame_camera_`。

| 字段 | 取值 | 说明 |
|---|---|---|
| `header.stamp` | 发布时的 `now()`（不是帧时间戳；一次性 latched 消息） | 与在建实现一致 |
| `header.frame_id` | 左：`frame_camera_`（默认 `camera_left`）；右：`"camera_right"` | 硬编码，与实测抓到的 `camera_info` 一致 |
| `width`,`height` | `W`,`H` | 未拼接时即单目尺寸；**拼接时仍是单目尺寸**（`camera_info` 描述单目内参） |
| `distortion_model` | 三态，见 §4.3 | |
| `k` | `K[0..8]` **原样**（行主序 → ROS 的 k[9] 也是行主序） | 不得重排 |
| `d` | 见 §4.3（5 个 0 / 8 个 / 4 个） | |
| `r` | `{1,0,0, 0,1,0, 0,0,1}` **恒为单位阵** | SDK **不提供** rectification matrix，不臆造（`25_test_report.md` 实测 `R=[1,0,0,0,1,0,0,0,1]`） |
| `p` | `{fx,0,cx,0, 0,fy,cy,0, 0,0,1,0}` | `Tx=0`：SDK 外参不映射到 P，立体基线**只**通过 `tf_static`（`camera_left→camera_right`）表达；**未验证**下游（`hobot_stereonet`）是否接受这种「P 无基线」的组合 |

`fx/fy/cx/cy` 与 `K` 冗余：`K[0]=fx,K[4]=fy,K[2]=cx,K[5]=cy`（`gs130.cpp:646-648`
逐个赋值 + `memcpy(K)`，两者同源，节点用 `K` 填 `k`、用 `fx..cy` 填 `p`，
并在 §9 的 gtest 里断言二者一致）。

### 4.3 畸变模型选择（**三态**，与 `calibration.py:12-25` 同规则）

```
if (dist_coeffs 全为 0.0)                     → "plumb_bob"            , d = [0,0,0,0,0]
else if (dist_model == GS130_DIST_FISHEYE)    → "equidistant"          , d = dist_coeffs[0..3]
else /* GS130_DIST_PINHOLE */                 → "rational_polynomial"  , d = dist_coeffs[0..7]
```

依据：`gs130.h:310-311` 定义两个模型（`PINHOLE`: `[k1,k2,p1,p2,k3,k4,k5,k6]`；
`FISHEYE`: `[k1..k4,0,0,0,0]`）。实测 GS130WI 是 **equidistant**：
`D=[-0.025173, 0.012189, -0.013019, 0.002486]`、`fx=386.85`。
`rect` 模式实测输出 `plumb_bob` + 5 个零（已矫正内参 `fx=362.07 cx=320 cy=240`），
正是第一条分支。

**保持为 0 的项（明确不填）**：`d` 的补零位（fisheye 的 4 个尾零、
`plumb_bob` 的 5 个零）；`r` 的全部非对角；`p` 的 `p[3]`、`p[7]`（即 `Tx=0`）、
`p[11]`（`1.0` 是约定，不是 0）。`k` 全 9 位都来自 SDK。
另有 `CameraInfo.roi`/`binning_*` 全零（默认值，不设置）。

### 4.4 静态 TF 的四元数

`to_quaternion(R9, x,y,z,w)` 用 Shepperd 分支法（与 `calibration.py:79-114` 等价）：
`trace>0` 走主分支，否则取最大对角元的分支，避免 `sqrt(负小量)`。
4 个易错点：

1. **行主序**：`R9` 是行主序，`m[r*3+c]`；转置会得到共轭四元数（TF 反向，静默错误）。
2. **不归一化就会有漂移累积**：EEPROM 的 R 由标定写入，**未验证**是否严格正交，
   因此在转四元数前做一次 Gram–Schmidt 正交化并对 `(x,y,z,w)` 归一化；
   gtest 里断言 `q` 还原的 R 与输入 R 的 Frobenius 差 < 1e-6。
3. **`w` 符号**：Shepperd 三支各自保证 `w>=0`；ROS 无符号约定，但下游
   (`tf2`) 用最短路径插值，静态 TF 无所谓。**不额外取反**。
4. **三个 TF 的父/子**：
   - `camera_left`（父）← `camera_right`（子），来自 `relative_R/T(RIGHT, LEFT)`；
   - `camera_left`（父）← `imu_link`（子），来自 `relative_R/T(IMU, LEFT)`；
     仅在 `gs130_get_imu_name(dev) != nullptr` 时发布（E8 实测第二实例 `imu_name=None`，
     此时**不应**出现 `imu_link`）。
   - 三个 stamp 用**同一个** `now()`。

**未验证**：`tf2_ros::StaticTransformBroadcaster` 是否会自己声明
`TRANSIENT_LOCAL` durability（Humble 的实现通常是），以及 `tf_static` 量程
（1 个 vs 2 个 transform 的 expected 数量）是否被下游硬编码。§10 实验附注。

---

## 5. 时间戳策略

### 5.1 问题

SDK 的 `timestamp_ns` 是 **CLOCK_MONOTONIC（开机以来）**，图像与 IMU 同域
（E5：`frame=9546841822000`、`imu-frame=8602616` ns，`wall-frame≈1.79e18` ns）。
直接把原始值写进 `header.stamp` 会静默破坏 `tf`/`message_filters`/`rosbag`（E5.2）。

### 5.2 方案（一次性 offset，运行期恒定）

```
启动期（gs130_start() 之后、定时器之前，在 executor 还没 spin 的时候）：
    frame = 第一个成功 pop 的帧（丢弃该帧数据，free 之）
    t_sdk  = frame.timestamp_ns                                  // 与 pop 同刻
    t_host = std::chrono::system_clock::now()                     // 与 pop 同刻
    offset_ns_ = t_host_ns - t_sdk     (int64，存 std::atomic<int64_t>)
运行期：
    header.stamp = to_stamp(static_cast<int64_t>(ts) + offset_ns_)  // 图像与 IMU 共用
```

- **读哪个时钟**：**`std::chrono::system_clock`**（Linux 上即 `CLOCK_REALTIME`，
  也就是 ROS 的 system time），**不是** `node->now()`、也不是 `steady_clock`。
  理由：offset 的语义是「把 SDK 的单调域搬到**墙上时钟域**」，所以参考点必须是墙上时钟。
  `steady_clock` 与 SDK 同域但没有 epoch 信息；`node->now()` 会随
  `use_sim_time` 返回不同东西（见 §5.4），**不能**用于计算 offset。
- **误差来源（必须写进 README 与日志）**：`t_sdk` 是**曝光时刻**，`t_host` 是**读出
  该帧之后**的时刻，两者相差「曝光→入队→pop」的延迟，量级为 1 个帧周期
  （30 fps 时 ≤33 ms，经验值 ≈3–10 ms）。E5 的实测「`imu - frame = 8.6 ms`」
  说明 IMU 与图像在 SDK 内已被校正到同一相位，**因此这个 offset 对图像和 IMU 是同一个**，
  二者相对关系（动捕/松耦合 VIO 最关心的量）**不被该误差破坏**，被破坏的只是
  绝对时刻的常数偏移。**已在 Python 版写入日志，实测为
  `offset = 1789385065931459293 ns`（≈ 2026-09-14 墙上时间）。**
- `report_timer_` 里每次打印一次 `offset_ns_` 与「上次 SDK→host 差值」，用于诊断
  漂移（E5/§10 实验 3）。

### 5.3 溢出与退化

- `frame.timestamp_ns` 是 `uint64_t`，转 `int64_t` **安全**（< 2^63，开机才 1e13 ns 量级），
  但转换必须显式 `static_cast<int64_t>`，否则 `uint64 + int64` 会走无符号算术，
  offset 为负时得到天文数字。
- `to_stamp` 负责 `sec = ns/1e9; nanosec = ns%1e9`，并对 `ns<0` 拒绝（返回
  `RCLCPP_FATAL` + 退出），不产生非法 `Time`（`builtin_interfaces::msg::Time`
  的 `nanosec` 必须 ∈[0,1e9)，Python 版曾因把整数直接赋给 `stamp` 而
  `AssertionError`，见 `25_test_report.md` §5 缺陷 1；C++ 里是编译期类型错误，
  同类错误不会发生，但**必须**防负数取模）。

### 5.4 `use_sim_time` 的影响（**本版明确关闭**）

- rclcpp 的 `node->now()` 返回 `RCL_ROS_TIME`：`use_sim_time=false` 时等于 system time，
  `true` 时等于 `/clock` 的时间，**未收到 `/clock` 前为 0**。
- 若用 `node->now()` 计算 offset，则在 `use_sim_time=true` 且 `/clock` 未到位时
  会算出 `offset = 0 - t_sdk`（巨大负数），运行期再叠加 `/clock` 跳变，
  时间戳**静默错乱**。这正是 `50_cpp_design.md` §3.4 要求反驳的点：**该主张有缺陷**。
- **v0.1.0 决策**：`main()` 里检查 `use_sim_time` 参数（rclcpp 会在
  `--ros-args -p use_sim_time:=true` 时自动声明它，用
  `if (node->has_parameter("use_sim_time")) get_parameter(...) else declare_parameter<bool>("use_sim_time", false)`，
  避免 `ParameterAlreadyDeclaredException`）。**为 `true` 时 FATAL 退出（码 2）**，
  错误信息说明：本节点的时间戳来自 SDK 的 `CLOCK_MONOTONIC`，无法映射到仿真时钟，
  请用 `ros2 bag record` 的 `--use-sim-time` 或在外部做时间转换。
  这比「静默产出错误时间戳」安全。
- **未验证**：`use_sim_time=false` 时 `node->now()` 与 `system_clock` 是否严格同一读数
  （理论上都是 `CLOCK_REALTIME`，但 rclcpp 会做一次 `rcl_clock` 包装）；
  本设计**不依赖**这个等号，因为 offset 只用 `system_clock` 算，
  `node->now()` 只用于 CameraInfo/TF 的 latched stamp。

### 5.5 图像与 IMU 的一致性

- 拼接帧的 `timestamp_ns` = **FSYNC 绑定目**（默认右目）的时间戳（`gs130.cpp:333`，
  `frame[0].timestamp_ns = ts[fi]`）；非拼接时左右**各有**自己的时间戳
  （`frame[R].timestamp_ns=ts[R]` 等），两者相差 ≤ 半帧周期（半帧对齐循环保证
  `|ts_R - ts_L| <= 1e9/fps/2`）。
- 节点对左/右**使用同一个 stamp**（取左目，与 Python 版一致），因为下游把这一对
  当作同步双目；若 `|ts_L - ts_R| > 5 ms`（半帧对齐退化的信号）打印 `WARN`，
  **待测**该情形在 30 fps 下的实际发生率。
- IMU：`is_fsync=true` 的包是**同步脉冲标记**，不是测量值。节点**不发布**这些包
  （只计数 + 每 5 s 报告），避免下游把脉冲当加速度。**未验证**：Python 版
  `_poll_imu` 未过滤 `is_fsync`，因此当前话题上可能混有这类包（`/imu/data` 的
  实际含义需要抓包核对）。这是本 C++ 版对 Python 行为的**一处有意收紧**。
- IMU 的协方差：`orientation_covariance[0] = -1.0`（SDK 无融合姿态，ROS 约定
  「第一个元素 -1 = 该字段不可用」），角速度/加速度 9 个协方差全零（=未知）。
  与 Python 版逐字段一致。

---

## 6. 参数声明与校验（C++ 的 fail-fast 写法）

### 6.1 用才 `declare_parameter<T>("name", default)` 模板重载

**必须**用带默认值的模板重载：

```cpp
width_ = declare_parameter<int>("width", 640);        // 声明即固定类型为 int
imu_   = declare_parameter<bool>("publish_imu", true); // bool
```

理由（这是 §6 的核心论点，可被证伪）：该重载在 `NodeParameters::declare_parameter`
里把 `rclcpp::ParameterValue(default)` 的类型写入 parameter 的**静态类型**；随后
launch/YAML 传来的 override 若类型不同（`ros2 launch ... width:="abc"` 或
`publish_imu:="false"`），在 `rclcpp::init` 之后的参数设置阶段抛
`rclcpp::exceptions::InvalidParameterTypeException`，**在 `gs130_create()` 之前**
被 `main()` 的 `catch (const std::exception&)` 捕获 → `RCLCPP_FATAL` 打印原始消息 →
`return 2`。相机从未被打开（与 E10 的「`width:=abc` 在 launch 期报错、节点未启动」
行为一致）。

**禁止**的写法（会被评审当缺陷指出）：

- `declare_parameter("width")`（无默认值）后 `get_parameter("width").as_int()`：
  类型错时在 `as_int()` 抛异常，位置在**参数读取循环中间**，且错误信息不含参数名。
- `get_parameter("publish_imu").as_bool()` 配 `bool` 转换自由函数：
  rclcpp 的 bool 参数**不做** `"false"`→`false` 的字符串解析（Python 版专门写了
  `_boolean()` 处理 launch 传字符串，见 `camera_node.py:69-78`）。
  C++ 侧的正确做法是让 `declare_parameter<bool>` 把类型钉死：
  `publish_imu:="false"`（**带引号**）会因类型不符而 fail-fast，
  而 `publish_imu:=false`（不带引号，YAML 解析为 bool）正常。
  **待测**：launch_ros 对带引号/不带引号两种写法的实际传递类型要在板端各试一次，
  并把结论写进 README（`25_test_report.md` 只测了 `publish_imu:=false` 这一种）。
- 用 `ParameterDescriptor` 的 `dynamic_typing=true`：故意关掉类型检查，反模式。

数值范围用 `rcl_interfaces::msg::ParameterDescriptor` 的
`integer_range` / `floating_point_range`（`from_value=true`）在**声明期**拒绝越界值，
再在 `validate()` 里做语义校验（两者互补：前者给通用错误，后者给「人话」错误）。

### 6.2 `validate()` 清单（全部在 `gs130_create()` 之前执行，失败 = FATAL + 退出码 2）

| # | 规则 | 依据 |
|---|---|---|
| V1 | `platform ∈ {RDKX5}` | `gs130_define.h:17-19`（未知值 `GS130_CONFIG` 宏会 `exit(1)`，我们用 `gs130_ros_preset()` 改成返回 `-1`） |
| V2 | `device ∈ {GS130WI, GS130W}`；**本机为 GS130WI**，`device:=GS130W` 实测 `NOT_FOUND`（E10）→ 允许传入但启动会 FATAL 并给出「设备名与硬件不符」提示 | E10 |
| V3 | `mode ∈ {raw, resize, rect}`；非法值列出合法取值 | E10 |
| V4 | `stereo_layout ∈ {none,left_right,right_left,top_bottom,bottom_top}`（后两者 E9 实测可用，保留） | E9/E10 |
| V5 | `1 ≤ fps ≤ 33` | E10（`fps:=120` 被拒） |
| V6 | `odr ≥ 1`（默认 200） | Python 版一致 |
| V7 | `width>0 && height>0` | — |
| V8 | `width%2==0 && height%2==0`（NV12 打包要求） | E10（`640x479` 被拒） |
| V9 | `(width,height) ∉ {(864,480),(1024,600)}`（实测 `GS130_UNSUPPORTED`）；**只拒这两个**，其余交给 SDK 当最终裁决者 | E10 |
| V10 | `mode=="raw"` ⟹ `(width,height)==(1088,1280)` | E6/E7/E10 |
| V11 | `mode=="raw"` ⟹ `stereo_layout=="none"`（拼接会翻倍宽度，与 RAW 的传感器尺寸冲突） | E6/E10（D10） |
| V12 | `use_sim_time == false` | §5.4 |
| V13 | `publish_tf` 为 false 时不创建 `StaticTransformBroadcaster`；`publish_imu=false` 时不建 `/imu/data`（E10 实测该话题不存在） | E10 |
| V14 | 目标分辨率必须能被 SDK 接受：以上全部通过后仍由 `gs130_init()` 的返回码兜底（`GS130_UNSUPPORTED` → 打印「该硬件无法做 WxH」） | E7/E10 |
| V15 | `start_timeout_s > 0`（默认 10.0），超时未出首帧 → FATAL（「无帧，检查相机与 IMU FSYNC 接线」） | Python 版一致 |

**实测可用的分辨率白名单（用于错误信息，不用于拦截）**：
`320x240 / 640x480 / 1280x720 / 1920x1080` 均 30 fps（E10）；
`1088x1280 rect` ≈20 fps（E10，**要 WARN 而不是拒绝**，见 §10 实验附注）。

---

## 7. 关停与故障处理

### 7.1 信号处理

- **不安装自研 `SIGINT`/`SIGTERM` handler**。`rclcpp::init()` 已安装
  `rclcpp::SignalHandlerOptions::All`：`SIGINT` → `rclcpp::shutdown()` →
  `spin()` 返回。自建 handler 会与 rclcpp 的 `signal_handler` 抢 `sigaction`，
  且 Python 版必须「在 shutdown 期间忽略第二个 SIGINT」（`camera_node.py:421-434`）
  的原因在 C++ 里被两件事替代：
  1. `gs130_stop()` 的实现是
     `stop_locked()`：`state=ThreadClosed` → `join()` 两个线程 → `pipeline->stop()`
     （`gs130.cpp:344-357`），**不可被信号打断**（信号只会设 `rclcpp` 的原子标志）。
  2. `rclcpp::shutdown()` 之后 `spin()` 立刻返回，**不会**再派发 timer 回调，
     因此**不存在**「回调执行到一半被关停打断」的窗口；`free()` 的 RAII 出口
     保证任何路径（含异常）都恰一次释放。
- `main()` 的骨架（`src/camera_node_main.cpp`）：

```cpp
rclcpp::init(argc, argv);
int code = 0;
std::shared_ptr<CameraNode> node;
try { node = std::make_shared<CameraNode>(); rclcpp::spin(node); }
catch (const std::exception & e) { code = 2; }   // FATAL 已打印细节
if (node) node->shutdown();                      // 幂等，见 §7.2
if (rclcpp::ok()) rclcpp::shutdown();
return code;
```

`CameraNode` 构造函数里任何失败（参数、校验、init、start、首帧超时）都抛
`std::runtime_error`；**构造失败时 `node` 为空，但已创建的 `gs130_device_t*`
必须在构造函数内部自己收拾干净**（见 §7.2 的顺序），否则相机会被残留持有。

### 7.2 释放顺序（唯一合法顺序）

```
gs130_stop(dev)      → 置 ThreadClosed、join camera_thread/imu_thread、pipeline->stop()、imu->stop()
gs130_deinit(dev)    → stop_locked()（空操作，已停）+ pipeline/imu/eeprom/camera_fifo/imu_fifo 全部 reset()
                       其中 camera_fifo.reset() 会 free 掉队列里剩下的帧（FIFO 的 disposer）
gs130_destroy(dev)   → delete dev          （文档明示：后台线程仍在运行时 destroy 是 UB）
dev_ = nullptr       → 幂等标记，二次调用直接返回
```

- **原因**：`gs130_destroy()` 只做 `delete dev`（`gs130.cpp:363`），而
  `~Fifo()` 会用 `disposer_` 访问 `free_frame_pair` 并把队列里的帧 free 掉；
  若先 destroy 后 stop，两个线程仍在跑并持有 `dev` 指针 → use-after-free。
- `gs130_deinit()` 内部**已经**调用 `stop_locked()`，所以 `stop` 在顺序上冗余，
  但保留它可让「停止→解初始化」两个阶段在日志里可分辨（在建实现
  `camera_node.cpp:116-118` 正是三连调用）。`gs130_stop` 对未运行设备是空操作
  （`gs130.h:195-202`），可安全重复。
- **未验证**：`stop` + `deinit` + `destroy` 之后 `librkisp`/`mipi` 的引用计数是否
  完全回落（判断标准：紧接着第二次启动同一节点能出图；`25_test_report.md` 的 L5
  只验证了「无残留进程 + 端口回收」）。

### 7.3 故障处理矩阵（`poll_images`/`poll_imu` 的返回码）

| 返回码 | 含义 | 处理 |
|---|---|---|
| `GS130_OK` | 有数据 | 发布，计数 `++frames_/++packets_` |
| `GS130_TIMEOUT` | 现在没有（**正常路径，非错误**） | 直接 `return`，**不打印**（否则 60 Hz 刷屏） |
| `GS130_HW_ERROR` | 底层通信/驱动失败 | `RCLCPP_ERROR`（节流 1 s）+ 置 `fault_=true`；`poll_*` 继续被调用但不再发布 |
| `GS130_THREAD_CLOSED` | SDK 线程已退出（含**已锁存故障**） | 同上，附加提示「SDK 线程已关闭；需 deinit+init 才能恢复（`gs130.h:28`）」 |
| `GS130_PARAM_ERROR` | 不可能出现（调用序错误/句柄空） | 视为**架构 bug**：FATAL 退出（不尝试恢复） |
| `GS130_UNSUPPORTED` | 不该发生（读接口与布局不匹配） | 视为配置 bug：FATAL，信息里带上 `stereo_layout` 与所选读接口 |

**不做自动重启**：`gs130.h:28` 明确「故障后需 `deinit + init` 恢复」，
而恢复要重建流水线（E8 证明流水线独占且状态脆弱）。v0.1.0 选择**快速失败**：
`fault_=true` 后节点继续存活（IMU 可能仍在流，`/tf_static`/`camera_info` 仍可查），
`report_timer_` 每秒重试**一次**读调用并在恢复时打印「stream recovered」。
若 2 次报告周期（10 s）仍为故障态 → `RCLCPP_FATAL` + 退出码 1，交给 launch
的重启策略（`respawn` 由用户在 launch 里选择，节点不自己 fork）。

### 7.4 E8 静默抢帧的检测（**必须能区分「真停帧」与「真故障」**）

现象（E8 实测）：第二个打开者不会失败，**抢走帧流**；先打开者 `frames` 冻结在 391，
而 `imu` 从 3051 涨到 7086。仅凭 `gs130_init` 失败判断占用是**错的**。

判据（`report_timer_`，周期 5 s）：

```
stall = (frames_ == last_frames_)                 // 本周期没有新帧
if (stall) ++stalled_; else stalled_ = 0;
if (stalled_ == 2)                                // 连续 2 周期 ≈10 s
    ERROR "no camera frames for about 10 s%s"
```

`%s` 必须**按事实分岔**（在建实现 `camera_node.cpp:471-479` 无条件写
「while the IMU still streams」，当 `publish_imu=false` 或 `!has_imu_` 时这是**假陈述**，
是评审可抓的缺陷，见 §11 偏差 4）：

| 条件 | 日志内容 |
|---|---|
| `has_imu_ && publish_imu_ && packets_ > last_packets_ && state==GS130_OK` | 「IMU 仍在增长而图像停帧 → **很可能有其它进程占用了相机**（mipi_cam / 第二个 camera_node）」+ `ps -ef \| grep -E "mipi_cam\|camera_node"` |
| `has_imu_ && packets_ 也不增长` | 「图像与 IMU 同时停 → 优先怀疑 SDK 流水线故障或设备掉线」 |
| `!has_imu_ \|\| !publish_imu_` | 「无 IMU 可供对照：**无法区分**占用与故障」，并要求人工核查 |

此外增加一条**独立**的占用探针（Python 版没有）：连续 3 个周期（15 s）
frame 冻结**且** `gs130_available_camera()` 恒为 0 **且** `state==GS130_OK`
→ 追加「SDK 队列一直为空但状态正常，符合 E8 的『帧被另一进程取走』特征」。
**待测**：被抢占时 `gs130_available_camera()` 是否真的恒为 0（若抢占者也在
用 SDK API，队列会被它 pop 空；若抢占者是 `mipi_cam`，帧可能根本进不了本句柄的
FIFO，两种情形的 available 行为可能不同）。这是 §10 实验附注里的低成本实验。

**同时必须监控**：`report()` 打印 `frames/imu/layout/WxH/mode/fps/odr` 一行
（与 Python 版一致），以及 `|| frames_` 的增长速率（换算成实际 fps 与配置 fps 的偏差），
因为 1088x1280 rect 实测只有 ~20 fps（配置 30），**这是静默的降级**，
必须在日志里显式 WARN：「requested 30 fps, measured X fps」。

---

## 8. 构建系统

### 8.1 `CMakeLists.txt` 要点

```cmake
cmake_minimum_required(VERSION 3.8)
project(gs130_ros C CXX)                      # C 必须存在：src/preset.c

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)                  # 只需要 C++17；不用 C++20（Humble 上 g++11.2）
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra)            # 不加 -Werror：libgs130 头可能有告警

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# 没有 pkg-config、也没有 CMake config：直接用 find_library（见 §8.2）
find_library(GS130_LIBRARY NAMES gs130 REQUIRED)
add_executable(camera_node
  src/camera_node_main.cpp src/camera_node.cpp
  src/conversions.cpp src/frame_buffer.cpp src/preset.c)
target_include_directories(camera_node PRIVATE include)
target_link_libraries(camera_node ${GS130_LIBRARY})
ament_target_dependencies(camera_node rclcpp sensor_msgs geometry_msgs tf2_ros)

install(TARGETS camera_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_conversions test/test_conversions.cpp src/conversions.cpp)
  ament_add_gtest(test_frame_buffer test/test_frame_buffer.cpp src/frame_buffer.cpp)
  # preset.c 是 C 文件：用 ament_add_gtest 时显式让 C 编译器处理它
  ament_add_gtest(test_preset test/test_preset.c src/preset.c)
  target_include_directories(test_conversions PRIVATE include)
  ament_target_dependencies(test_conversions sensor_msgs geometry_msgs)
  target_include_directories(test_frame_buffer PRIVATE include)
endif()
ament_package()
```

`install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})` 的语义：把
`launch/*.py` 装到 `share/gs130_ros/launch/`，于是
`ros2 launch gs130_ros gs130_camera.launch.py` 生效（`ros2 launch` 从
`share/<pkg>/launch` 找文件）。**不要**用 `install(FILES ...)` 逐个列举：
D8 要求 `gs130_web.launch.py` 能带 nginx 清理逻辑一起演进。

### 8.2 怎么找到 `libgs130`

板上**没有** `gs130.pc`，也**没有** `gs130Config.cmake`。事实是：

```
/usr/include/gs130.h        /usr/include/gs130_define.h
/lib/aarch64-linux-gnu/libgs130.so        (VERSION 0.0.1, 实测 `g++ x.cpp -lgs130` 可跑)
```

因此：

1. 头文件：**不需要** `target_include_directories`（`/usr/include` 是编译器默认搜索
   路径）；`#include "gs130.h"` 直接可用。若将来头被装到别处，加
   `find_path(GS130_INCLUDE_DIR NAMES gs130.h REQUIRED)` 兜底。
2. 库：`find_library(GS130_LIBRARY NAMES gs130 REQUIRED)` → 找到
   `/lib/aarch64-linux-gnu/libgs130.so`；**链接时用绝对路径 `${GS130_LIBRARY}`**
   （而不是 `-lgs130`），这样 CMake 会把该 `.so` 记进
   `CMakeFiles/camera_node.dir/link.txt`，构建日志可核对；`REQUIRED` 保证
   缺库时在**配置阶段**报错，而不是链接期一堆 `undefined reference`。
3. **不要**写 `target_link_libraries(camera_node gs130)`（裸 `-lgs130` 也行，
   但错误信息差、且无法在 CMake 里断言版本）。
4. **版本门**：`gs130.h` 不暴露版本宏给预处理器（`gs130_version()` 是运行时函数）。
   运行时在 `main()` 里打印 `gs130_version()`（实测 `"0.0.1"`）与
   `gs130_platform()`（实测 `"rdkx5"`），并在 REPORT 日志里带一次；
   构建期无法静态断言版本，这点写清楚（评审易抓「为什么没有版本检查」）。
5. `CMAKE_BUILD_TYPE` 不显式设置：colcon 默认给出 `-O3 -DNDEBUG`（RelWithDebInfo
   风格），这对 6 MB memcpy 很重要（`-O0` 下 memcpy 会退化成逐字节）。
   **待测**：在 `-O3` 之外试 `-fno-semantic-interposition` 与
   `-march=armv8.2-a` 对 memcpy 的影响（§10 实验 2 的附注）。

### 8.3 为什么必须有一个 `.c` 文件（`src/preset.c`）

`gs130_define.h` 的 `GS130_CONFIG_*` 宏使用 **GNU range designator**
（`.bus_mipi_rx = {[0 ... 3] = 0xFF, [4] = 2, ...}`，`gs130_define.h:31`），
C++ 在任何标准模式（含 `-std=gnu++17`）下都**不接受**该语法；
且 `GS130_CONFIG(...)` 宏在未知 platform/device 时**直接 `exit(1)`**
（`gs130_define.h:19`），节点不能接受这种副作用。

所以：把宏的展开放在 `src/preset.c`（GNU C 接受 range designator），
包成 `int gs130_ros_preset(const char*, const char*, int, int, int, int, int, gs130_config_t*)`，
未知值返回 `-1`，由节点转成 `RCLCPP_FATAL("unsupported platform/device: ...")`
+ 退出码 2（与 E10 的实测行为一致）。

**风险（未验证，评审可攻击）**：宏用的是**位置式**指定初始化
（`.camera_config=..., .imu_config=..., .eeprom_config=..., .camera_fifo=..., .imu_fifo=...`），
若上游改字段顺序/新增字段，`preset.c` 会**静默**编译通过而语义错位。缓解：
`test/test_preset.c` 断言 `gs130_ros_preset("RDKX5","GS130WI",...)` 的关键字段
（`left_addr==0x30`、`bus_mipi_rx[4]==2`、`bus_reset_gpio[4]==351`、`camera_fifo.depth==4`、
`imu_fifo.depth==1024`、`sensor_width==1088`）——**这些是 SDK 侧预设的实测值**，
一旦上游改动会红。另外注意 `GS130_CONFIG_RDKX5_GS130W` **未**初始化 `.imu_config.bus`
（`gs130_define.h:59-63` 只写了 `.bus_num=0`），`-Wall -Wextra` 可能对
`preset.c` 报 `-Wmissing-field-initializers`，允许在该文件局部 `#pragma` 抑制并写明原因。

---

## 9. 可测试性

### 9.1 主机可测（gtest，**不接相机、不需要板子**）

| 目标 | 内容 | 为什么不需要硬件 |
|---|---|---|
| `test_conversions` | 畸变模型三态（全零→plumb_bob+5 零、fisheye→equidistant+4、pinhole→rational_polynomial+8）；`K` 与 `fx..cy` 一致性断言；`r` 恒为单位阵；`p` 精确等于 `{fx,0,cx,0,0,fy,cy,0,0,0,1,0}`；`to_stamp` 的 `sec/nanosec` 拆分与负值拒绝；Shepperd 四支分支（含 `trace<=0` 的最大对角元分支）后 `q` 还原的 R 与输入误差 <1e-6；TF 的 `parent/child` 方向断言（`RIGHT→LEFT` 得到 `parent=camera_left`） | 只依赖 `sensor_msgs`/`geometry_msgs` 头 + `gs130_camera_intrinsics_t`（POD），**不链接 libgs130** |
| `test_frame_buffer` | `Nv12Frame` 析构恰调用一次 `free`（用 `--wrap=free` 或函数指针钩子计数）；`release()` 幂等；拷贝被删除（编译期）；`nv12_size(w,h)==w*h*3/2`；`nv12_height(packed_rows)==rows*2/3`（E2 的契约） | 纯逻辑 |
| `test_preset` | §8.3 的字段断言 + 未知 platform/device 返回 `-1`（**不退出**） | 只链接 `preset.c` 与 `gs130.h` |

主机上**无法**测：QoS 兼容性（E1）、`publish()` 阻塞（§2.5）、malloc/mmap 成本、
时间戳域、帧率。这些必须上板。

### 9.2 需要板子

| 目标 | 方法 | 判据 |
|---|---|---|
| 话题/QoS | `ros2 topic info -v /image_combine_raw` | publisher 与 `hobot_codec` 订阅端**均** RELIABLE（E1） |
| 速率 | `ros2 topic hz` 30 s，以及节点自报 `frames/imu` 的差分 | ≥29.5 fps（1920x1080 也要）；IMU ≈ odr |
| 消息契约 | `ros2 topic echo --once` + 一段 Python/`cv_bridge` 校验脚本 | `encoding=nv12`、`height`=真实高、`step==width`、`len(data)==W*H*3/2`（E2） |
| 时间戳域 | 抓一帧的 `header.stamp` 与 `date +%s` 比 | 同域（差值 = offset 误差，≤1 帧周期）；**不得**出现 1.79e9 s 的偏差（E5） |
| 标定 | `ros2 topic echo /image_left/camera_info --once` 与 `ros2 run tf2_ros tf2_echo camera_left camera_right` | 与 `25_test_report.md` §3.3 的实测值同量级；基线 ≈0.070 m |
| 关停 | `Ctrl-C` 后 `ps -ef \| grep -E "nginx\|websocket\|hobot_codec\|camera_node"` 与 `ss -ltn \| grep :8000` | CLEAN + 端口释放（E4/D8；nginx 是孤儿进程，必须显式清理） |
| 故障路径 | 终端 A 起节点，终端 B 起第二个节点 | 终端 A 在 ≈10 s 内打出 §7.4 的 ERROR，且**不是**「ERR 说 IMU 仍在流」的假陈述（`publish_imu:=false` 时另测一次） |
| 参数校验 | E10 的 8 类非法输入逐条重放 | 全部 FATAL + 退出码 2 + 相机未被打开 |

---

## 10. 风险与三个最需要板端测量的未知量

### 10.1 风险清单（按严重度）

| # | 风险 | 现有依据 | 缓解 |
|---|---|---|---|
| R1 | 1920x1080 下单线程模型被 `publish()` 阻塞打崩（<30 fps 或帧时间抖动） | §2.4 外推 15 ms/帧，**未验证** | §2.6 方案 B（取帧与 publish 解耦） |
| R2 | 每帧 `malloc/free` 大块走 `mmap`，缺页成本吃掉余量 | glibc 默认 128 KB 阈值，**未验证** | `mallopt(M_MMAP_THRESHOLD, 64<<20)`（需 A/B） |
| R3 | E8 抢占被误报/漏报 | 判据在 §7.4，**未验证** available 行为 | 三态分岔日志 + 人工核查提示 |
| R4 | `1088x1280 rect` 只有 ~20 fps 却被静默接受 | E10 实测 | WARN + 实测 fps 上报（不拒绝：用户可能故意要 rect） |
| R5 | 100 ms 级 `malloc` 抖动导致 SDK `camera_thread` 长时间等锁 | 推断 | 无法从节点侧消除；用 `salvage`：DROP_OLD 保证只丢旧帧不卡流水线 |
| R6 | `P` 矩阵 `Tx=0` 使下游立体匹配拿不到基线 | §4.2 | 文档化 + `tf_static` 提供基线；**未验证** `hobot_stereonet` 需求 |
| R7 | 一次性 offset 的长期漂移 | `25_test_report.md` §6「未测」 | §10.2 实验 3 |
| R8 | `imu_link` 的 TF 在无 IMU 时缺失（第二实例 `imu_name=None`，E8） | E8 实测 | `gs130_get_imu_name()` 判定 + 明确日志 |
| R9 | 静态 TF 的 stamp 用了 `now()` 而 `tf_static` 是 latched：`use_sim_time` 下会有 0 时间戳 | 推断 | 已被 V12 关闭 `use_sim_time` 覆盖 |
| R10 | `is_fsync` 包被当作测量值发布（Python 版行为） | 源码 + 推断 | C++ 版过滤（§5.5），需抓包核对 Python 版现状 |

### 10.2 三个判决性实验（每一项都能给出数字，且能推翻上面的论断）

**实验 1：`publish()` 在同线程 RELIABLE/KEEP_LAST(1) 下的最坏阻塞时间**

- 目的：判决 §2.1/§2.5 的单线程结论（R1）。
- 方法：`tools/measure_pipeline.cpp` 用 `-p publish:=true/false` 两档，
  1920x1080@30 各跑 60 s；每帧记录 `t0=pop 前`、`t1=memcpy 后`、`t2=publish 后`，
  输出 p50/p99/max。订阅端用 `ros2 topic hz` + 一个故意慢的回调
  （或 `ros2 bag record` 限速）制造 RELIABLE 反压；再用
  `ROS_DISABLE_LOANED_MESSAGES=1`（保险）与**无订阅者**两种对照。
- 判据：`t2-t1` 的 max ≤ 16.67 ms 且 p99.9 ≤ 8 ms → 单线程通过；
  否则走 §2.6 方案 B。（`t2-t1` 无订阅者时应 ≈0，可校准测量本身的开销。）
- 顺带产出 R2 的数据：对比 `mallopt` 打开/关闭两组 `t1-t0`。

**实验 2：1920x1080@30 全链路 60 s 稳定性与真实帧率分解**

- 目的：判决 §2.4 的 15 ms/帧外推，以及「DROP_OLD 是否真的兜住」。
- 方法：跑 `gs130_web.launch.py`（相机 + 既有 codec + 既有 websocket），
  60 s 内每 5 s 打印 `frames`/`packets`/`available_camera()`/实测调用次数，
  同时 `ros2 topic hz /image_combine_raw`、`/image_combine_jpeg`。
- 判据：`frames` 在任何一个 5 s 窗口内的增量 ≥ 145（≈29 fps）；`available_camera()`
  恒 ≤1；websocket `http_code=200`（E4）。**若出现帧率下降但无 ERROR**，
  说明是 §7.4 的静默降级，需要补判据。
- 附注（低成本、顺手做）：`1088x1280 rect` 的实测 fps（判定 R4 的 WARN 阈值）；
  被第二个进程抢占时 `gs130_available_camera()` 的取值序列（判定 R3）。

**实验 3：`CLOCK_MONOTONIC` 与 `CLOCK_REALTIME` 的相对漂移（一次性 offset 的长期有效性）**

- 目的：判决 §5.2 的「一次性 offset 足够」与 R7（`25_test_report.md` 明确列为未测）。
- 方法：不起相机也能做一半——1 Hz 采样 `frame.timestamp_ns`（需相机，30 s 即可）
  与 `clock_gettime(CLOCK_MONOTONIC)/CLOCK_REALTIME` 的差值；完整版是
  **连续 30 min**，每秒记录 `delta = realtime - (sdk_ts + offset)`，
  对 `t` 做最小二乘拟合。
- 判据：`|斜率| ≤ 1 ms/min` → 一次性 offset 通过（30 min 累计漂移 ≤30 ms，
  优于一个帧周期）；`> 1 ms/min` → 必须改为周期性重算 offset（例如每 60 s 在
  IMU 回调里做一次 `offset += slow_correction`），并写入 README 的精度声明。
- 注意：该实验**只**验证域间漂移，不验证「offset 的绝对精度」（后者受
  §5.2 的「曝光→pop」延迟限制，约 3–10 ms，无法用这个实验测出，
  需要外部触发源，**未验证**且在 v0.1.0 不承诺）。

---

## 11. 与在建实现的偏差（供评审直接对账）

以下 6 处是本文档与 `ros/gs130_ros/src/camera_node.cpp`（当前 531 行、未提交）
不一致或需要加强的点。**本文档为准**，实现侧需按此修改：

1. **`use_sim_time` 未处理**：在建实现的 `take_clock_offset()`
   （`camera_node.cpp:439`）用 `now().nanoseconds()`（= `RCL_ROS_TIME`）算 offset；
   `use_sim_time=true` 且 `/clock` 未到位时 `now()==0`，offset 变成巨大的负数。
   改为 `std::chrono::system_clock::now()`，并按 §5.4 的 V12 在启动期拒绝
   `use_sim_time=true`。
2. **`gs130_device_t*` 的异常安全**：`open_device()`（`:207-228`）在
   `gs130_init` 失败时销毁句柄，但 `start_streaming()`（`:234-237`）在
   `gs130_start` 失败时直接 `fatal()` 抛出，**句柄未 `gs130_destroy()`**；
   而 `main()` 里 `node` 为空 → 没人释放 → 相机被残留持有（正是 E8 的触发条件）。
   改为：`CameraNode` 构造函数用 RAII 持有 `gs130_device_t*`（一个
   `DeviceHandle` 成员，析构即 `stop+deinit+destroy`），任何抛出路径都释放。
3. **`publish_frame` 的 free 出口不唯一**（`:371-389`）：`free` 在调用方
   （`:359/363-364`）而不是 RAII 容器里；`data.assign` 若抛
   `std::bad_alloc`/`std::length_error` 会**跳过** `free`。改用 §3.2 的
   `Nv12Frame` + 作用域内释放。
4. **E8 看门狗日志是假陈述**（`:471-479`）：无条件写「while the IMU still
   streams」，`publish_imu=false` 或 `!has_imu_` 时不成立。按 §7.4 的三态分岔。
   同时缺 `gs130_available_camera()` 恒 0 的独立证据。
5. **缺少 `framing` 校验**（防御 E2 类错误）：发布前应断言
   `frame.width == (stitched_ ? 2*width_ : width_)` 且
   `frame.height == height_`，不一致时 FATAL（错误信息带期望/实际），
   而不是把一个几何错位的帧发出去让 `hobot_codec` 段错误。
6. **`report()` 未上报实测 fps**（`:464-484`）：R4（rect 模式 ~20 fps）需要
   显式 WARN。另外 `imu_timer_` 在 `publish_imu=false` 时不创建（`:267`）——正确，
   但此时 §7.4 的判据分支也要相应退化。

---

## 12. 有待板端确认的「未验证」清单（汇总，便于评审逐个攻击）

| 编号 | 未验证论断 | 验证方式 |
|---|---|---|
| U1 | 1920x1080@30 单线程 `publish()` 最坏 ≤16.67 ms | §10 实验 1 |
| U2 | 1920x1080 单帧总成本 ≈15 ms | §10 实验 2 |
| U3 | `malloc`/`mmap` 缺页成本 0.75–1.5 ms/帧，`mallopt` 能降 | §10 实验 1 附注 |
| U4 | 一次性 offset 的漂移 ≤1 ms/min | §10 实验 3 |
| U5 | EEPROM 的 `R` 是否严格正交（是否需 Gram–Schmidt） | §10 实验 2 抓 `tf2_echo` 的 R 行列式 |
| U6 | `tf2_ros::StaticTransformBroadcaster` 的 durability 与 `tf_static` 的 transform 数量 | 板端 `ros2 topic info -v /tf_static`，与 Python 版对照 |
| U7 | 被抢占时 `gs130_available_camera()` 的行为 | §10 实验 2 附注 |
| U8 | `is_fsync` 包在 Python 版是否已被发到 `/imu/data` | 板端抓 `/imu/data` 的 `odr` 与脉冲计数对照 |
| U9 | launch 传 `publish_imu:="false"`（带引号）在 C++ 侧的类型错误行为 | 板端逐条重放 E10 |
| U10 | `stop+deinit+destroy` 后能否立刻再次启动同一节点（引用计数是否回落） | 连跑两次 launch |
| U11 | `hobot_stereonet` 是否要求 `P` 里带基线（`Tx!=0`） | 读其 launch/参数或试用 |
| U12 | 30 fps 下 `|ts_L - ts_R|` 的实际分布（半帧对齐是否稳定） | 在 `tools/measure_pipeline.cpp` 里打印直方图 |
