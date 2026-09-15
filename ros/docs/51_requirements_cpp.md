# 51 C++ 节点需求与接口契约（v0.2.0，待对抗式评审）

- 文档编号：51
- 角色：TEAM-AUTHORS/A1（需求与接口所有者）
- 状态：**PROPOSED（待评审）**；本文通过的条目在 C++ 实现合入后升为 FROZEN v0.2.0
- 取代关系：本文取代 `11_interface_freeze.md` 的**外部接口部分**；与 `00_verified_platform_facts.md`（E1–E10，真机实测）冲突时**一律以 00 为准**，本文必须被改
- 被攻击对象：`50_cpp_design.md`（架构师设计稿）。本文是它的**需求方**，不是它的复述；两者冲突处以本文的判据为准
- 取证边界：本文作者**未登板、未运行相机、未运行任何 ROS 节点**。所有判据由板端团队执行；本文引用的数值全部来自 `00_verified_platform_facts.md`（E1–E10）与 `25_test_report.md`（Python 版实测）
- 标签：`[P]` = Python 版已实测通过的项（C++ 版必须在同等判据下复现，属回归门禁）；`[C]` = C++ 版新增或**从未被任何实验覆盖**的项

---

## 0. 阅读约定

| 约定 | 含义 |
|---|---|
| 判据 | 一条可在板端执行的命令 + 期望输出。判据不成立的实现即**不达标**，不接受"环境差异""看起来在跑"作为解释 |
| must | 不进 v0.2.0 不得发布 |
| should | 可带已知缺陷发布，但缺陷必须写进 README 与本文件变更记录 |
| may | 允许不做，做了必须按本文判据验收 |
| 排除（exclusion） | 明确**不得**实现的行为；实现它属于契约违反，不属于"顺手加的优化" |
| P/C | 见上文标签。`[C]` 项在板端验证之前，任何文档不得声称它已工作 |

---

## 1. 范围与 C++ 重写的理由

### 1.1 范围（一段）

本包是 GS130 双目相机 + IMU 在 RDK X5 / TROS Humble 上的**唯一 ROS 2 驱动**：一个 ament_cmake 构建的 C++ 节点（包 `gs130_ros`，节点 `gs130_camera`），用 `rclcpp` 直接链接 `libgs130.so` 的 C API，`gs130_create→gs130_init→gs130_start` 持有相机与 IMU，把 SDK 交付的**紧排 NV12** 帧与 IMU 包转成 `sensor_msgs/Image`、`sensor_msgs/Imu`、`sensor_msgs/CameraInfo`，把 EEPROM 外参转成 `/tf_static`，并通过**两个 launch 文件**把默认显示链路接到 D-Robotics 既有节点上（`/image_combine_raw` → 既有 `hobot_codec` → 既有 `websocket` → `http://<board>:8000`）。节点是**独占设备持有者**：同一时刻只有一个进程可以拿相机，节点必须在被第二个打开者抢帧时把它变成**可诊断错误**而不是静默停帧。范围外的一切（编码、网页、深度、姿态解算、零拷贝、动态重配置、多实例、ROS 1）见 §4。

### 1.2 为什么必须是 C++：C API 相对 Python 绑定到底给了什么

前提说明（避免夸大）：Python 侧的 `python/gs130/` 绑定用 ctypes **逐函数手工镜像了几乎整个 C API**（`python/gs130/_abi.py:140-175`），因此"Python 拿不到某个函数"**不是**本文的理由。真正的理由是下面 8 条，全部可核对到文件与行号，且都能转成可证伪的判据。

| # | 维度 | Python 路径（rclpy + ctypes 绑定） | C++ 直连 C API | 证据 |
|---|---|---|---|---|
| J1 | ABI 单一真源 | `_abi.py` 手工抄写 `gs130_*.` 结构体布局；C 侧对 `gs130_calibration_t` 有 `static_assert`（`core/src/gs130.cpp:761-762`），Python 侧**没有任何等价保护**，结构体漂移是静默的 | `#include <gs130.h>`，与 `libgs130.so` 同源编译，漂移在编译期暴露 | `core/src/gs130.cpp:761` |
| J2 | 序列化开销 | E3 实测 `msg.data = bytes` → **1103.87 ms/帧**（rclpy 逐元素校验）；正确写法 1.36 ms；`publish()` 本身 5.07 ms | `sensor_msgs::msg::Image::data` 是 `std::vector<uint8_t>`，赋值即一次连续 memcpy；**语言层面不存在**"逐元素校验"这条代码路径 | E3 |
| J3 | 缓冲所有权 | 依赖 numpy 视图 + `weakref.finalize` 调 `free`（`python/gs130/_types.py:25`），释放时机由 GC 决定 | C API 明确 `malloc` 所有权归调用方（`core/include/gs130.h:231,249`）。C++ 必须**显式** free——这是把隐式生命周期换成可审计的显式生命周期，**不是白拿的好处**，因此本文用 R-21 的 RSS 判据把它变成可证伪项 | `gs130.h:231,249` |
| J4 | 故障与空闲的区分 | 绑定把 `GS130_TIMEOUT` 转成 `None`、其他码转异常；节点在异常层二次判断 | 直接读返回值即可区分"队列空"（`GS130_TIMEOUT`）与"通路故障"（`GS130_HW_ERROR`/`GS130_THREAD_CLOSED`）；且 `gs130_available_*()` 在故障时**返回 0**（`gs130.cpp:550,598`），只有直接读 `gs130_get_*` 的返回值才能识破"故障伪装成空闲" | `gs130.cpp:550,562,580,598,608` |
| J5 | 几何真值 | 拿到的是 numpy 视图 `(H*3//2, W)`，必须做 `shape[0]*2//3` **反推**真实高——E2 那个会让既有节点段错误的 bug，成因正是"几何要从缓冲区字节形状反推" | `gs130_image_nv12_t` 直接给出 `width`/`height`/`timestamp_ns`（`gs130.h:206-210`），C++ 侧照抄即可，**结构上消灭**这一类 bug | E2；`gs130.h:206-210` |
| J6 | 配置面 | 受支持入口只有 `Config.preset()`（`python/gs130/_config.py:53`），bus/addr/tuning_file/fifo/fsync 全部硬编码 | `gs130_config_t` 是完整可控结构（`gs130.h:111-119`）。v0.1.0/v0.2.0 **仍然只用 preset 等价值**（见 §3.3 排除项），但将来调 GDC/tuning/fifo 不需要改绑定层 | `gs130.h:63-119` |
| J7 | 工具链与部署 | 需要 rclpy + numpy + 手写绑定 + `pip3 install` 流程（旧 README 的 `build-wheel.sh` 步骤） | ament_cmake + colcon，与既有 `hobot_codec`/`websocket` 同一 ABI、同一构建系统；板端已核实 `g++ -std=gnu++17 x.cpp -lgs130` 可用 | 平台事实（`/opt/ros/humble`、`colcon`、`libgs130.so 0.0.1`） |
| J8 | 进程内确定性 | 有 GIL、解释器启动、GC 抖动；释放依赖 `__del__`（实测缺陷 2：关停期第二个 SIGINT 会中断释放，`25_test_report.md` §5） | 无解释器；关停是可写死的三步 `gs130_stop → gs130_deinit → gs130_destroy`（`gs130.h:145-153,194-202`），不依赖任何析构 | `25_test_report.md` §5 |

结论：C++ 重写的收益是**去掉一层手工 ABI 镜像 + 去掉一条实测 800× 的慢路径 + 把隐式的内存与故障语义变成显式的、可测的**。它不是"Python 做不到"，因此本文所有 `must` 都必须能用命令证明，而不允许用"语言更好"当理由。

---

## 2. 需求（R-01 … R-51）

### 2.0 实测事实 → 需求的映射（每个实测失败模式都必须有需求防止它被发布）

| 事实 | 内容摘要 | 对应需求 |
|---|---|---|
| E1 | `hobot_codec` 订阅端 RELIABLE；BEST_EFFORT 被静默丢弃 | R-16, R-17 |
| E2 | codec 把 `Image.height` 当真实高；打包高 → 段错误 -11 | R-19, R-20 |
| E3 | rclpy 下 `msg.data = bytes` 慢 800× | J2（语言层面不再适用）；R-20 的 `len(data)` 判据 |
| E4 | 复用 codec+websocket 端到端 29.97 fps、HTTP 200；nginx 退出后成孤儿占 8000 | R-05, R-06, R-07, R-08, R-45 |
| E5 | SDK 时间戳是 CLOCK_MONOTONIC；图像与 IMU 同域；直接透传会破坏 tf/bag | R-35, R-36, R-38 |
| E6 | `stereo_layout` 走 SDK 硬件拼接；RAW 强制 1088x1280 不能拼接 | R-09, R-11, R-42 |
| E8 | 相机不可共享但第二个打开者不被拒绝 → 先打开者**静默停帧**（IMU 仍在流） | R-48, R-49, R-50 |
| E9 | `top_bottom` 拼接帧实测解码正确（640x960） | R-10, R-14 |
| E10 | 参数校验矩阵 + 分辨率矩阵（320x240/640x480/1280x720/1920x1080 可用；864x480/1024x600 被拒） | R-40, R-41, R-42, R-43, R-44 |

### 2.1 校验矩阵（R-40 / R-41 的唯一判据，逐行执行，E10 实测）

统一命令形式：`ros2 launch gs130_ros gs130_camera.launch.py <该行的输入>; echo $?`，并检查该行日志。

| # | 输入 | 期望输出（逐字要点） |
|---|---|---|
| V1 | `mode:=rgb` | 退出码 2；日志列出合法取值（`raw`/`resize`/`rect`）；**未触碰相机** |
| V2 | `mode:=raw width:=640 height:=480` | 退出码 2；日志说明 `raw` 需要 `1088x1280` |
| V3 | `mode:=raw width:=1088 height:=1280 stereo_layout:=left_right` | 退出码 2；说明 raw 与拼接互斥，建议 `stereo_layout:=none` |
| V4 | `width:=640 height:=479` | 退出码 2；说明 NV12 需要偶数 |
| V5 | `width:=abc` | 退出码 **2**；消息指名 `width` 与原值 `abc`；**不得** SIGABRT / 退出码 134 / 未捕获异常栈 |
| V6 | `fps:=120` | 退出码 2；说明上限 33 |
| V7 | `platform:=RDKX6` | 退出码 2；含 `unsupported platform/device` |
| V8 | `odr:=100` | 退出码 2；消息引用 `gs130_get_imu_info()` 的 ODR 列表（`200 \| 500`） |
| V9 | `device:=GS130W` | 退出码 **1**；含 `GS130_NOT_FOUND`（本硬件为 GS130WI，E10 实测） |
| V10 | `publish_imu:=maybe` | 退出码 2；消息指名 `publish_imu` 与原值 `maybe` |

每一行的附加判据（R-41）：`pgrep -f gs130_camera_node` → 空；日志中 `grep -c "frames="` → `0`。

### 组 A：交付形态与依赖

| 编号 | 要求 | 判据（命令 → 期望） | 依据 | 优先级 |
|---|---|---|---|---|
| R-01 | 交付物是 ament_cmake C++ 包 `gs130_ros`，节点名 `gs130_camera`，可执行 `gs130_camera_node`，包版本 `0.2.0`；同时**删除** Python 包（`ros/gs130_ros/{setup.py,setup.cfg,resource,gs130_ros/*.py}` 不再存在） | `colcon build --packages-select gs130_ros` → 退出码 0；`ros2 pkg executables gs130_ros` → 恰好 `gs130_ros gs130_camera_node` | `50_cpp_design.md:41-51` | must |
| R-02 | 可执行文件必须**同时**链接 C API 与 rclcpp，不得自带 SDK 源码副本 | `ldd install/gs130_ros/lib/gs130_ros/gs130_camera_node \| grep -E "libgs130\|librclcpp"` → 两行都在；`grep -rn "rclpy\|import gs130" ros/gs130_ros` → 无输出（退出码 1） | J1/J7 | must `[P]` |
| R-03 | 设备访问**只**通过 `gs130.h` 的 C API：禁止直接 I2C/V4L2/GPIO 操作，禁止复制 SDK 内部逻辑 | `grep -RnE "ioctl\|/dev/i2c\|/dev/video\|sgpio" ros/gs130_ros/src` → 无输出；`nm -D --undefined-only <exe> \| grep -c gs130_` ≥ 8 | J1 | must |
| R-04 | 运行期不得修改既有 TROS 文件 | `touch /tmp/m; <跑一次 web 链路 + Ctrl-C>; find /opt/tros -newer /tmp/m -type f` → 空 | `11:647` | must `[P]` |

### 组 B：单命令 bring-up 与 web 链路

| 编号 | 要求 | 判据（命令 → 期望） | 依据 | 优先级 |
|---|---|---|---|---|
| R-05 | 零参数单命令拉起整条显示链路 | `ros2 launch gs130_ros gs130_web.launch.py` 后 30 s 内：`ros2 topic hz /image_combine_jpeg` → 平均 ≥ **29.0 Hz**；`curl -s -o /dev/null -w '%{http_code}' http://127.0.0.1:8000/` → `200` | E4；`25_test_report.md` §3.2（29.997） | must `[P]` |
| R-06 | web 链路只由**既有**节点构成，本包不实现编码与网页 | `ros2 node list` 同时含 `gs130_camera`、一个 `hobot_codec_encoder_*`、`websocket`；`ros2 topic list` 不含本包自己发的 jpeg/压缩话题 | E4；§4 非目标 | must `[P]` |
| R-07 | `gs130_camera.launch.py` 只拉起相机节点，不依赖 codec/websocket 存在 | `ros2 launch gs130_ros gs130_camera.launch.py` 后 `ros2 node list` → 只有 `gs130_camera`；`ros2 topic hz /image_combine_raw` → ≥ 29.0 Hz | `11:452` | must `[P]` |
| R-08 | 链路退化即判定为回归，不得用"环境差异"解释 | 同时满足：`ros2 topic hz /image_combine_raw` 与 `/image_combine_jpeg` 之差 ≤ 0.5 Hz；codec 日志无 `incompatible QoS`；`ros2 node list` 中 codec 始终存活（无 `process has died ... exit code -11`） | E1/E2/E4 基线 29.966/29.974 | must `[P]` |

### 组 C：图像消息契约（NV12）

| 编号 | 要求 | 判据（命令 → 期望） | 依据 | 优先级 |
|---|---|---|---|---|
| R-09 | 拼接帧逐字段：`encoding="nv12"`、`step=width`、`is_bigendian=0`、`len(data)==width*height*3//2`；`left_right` + 640x480 → `width=1280, height=480, data=921600` | `ros2 topic echo /image_combine_raw --once --field encoding` → `nv12`；`--field width` → `1280`；`--field height` → `480`；`--field step` → `1280`；`--field is_bigendian` → `0`；`--field data` 的长度（`ros2 topic echo --once --field data \| tr -d '[],\n ' \| wc -c`）→ **≥ 921600**（YAML 有额外字符，只允许偏大） | E2/E6；`25_test_report.md` §3.3 | must `[P]` |
| R-10 | 几何**直接取** `gs130_image_nv12_t.width/height`，禁止由缓冲字节数反推（禁止 `shape`/`len(data)` 反解）；`top_bottom` + 640x480 → `width=640, height=960` | `ros2 launch gs130_ros gs130_camera.launch.py mode:=resize width:=640 height:=480 stereo_layout:=top_bottom` 后 `ros2 topic echo /image_combine_raw --once --field width` → `640`，`--field height` → `960` | E9 实测 `(640,960)`；`gs130.h:206-210` | must `[C]`（E9 只测过 Python 版） |
| R-11 | 话题集合由 `stereo_layout` 唯一决定：`none` → `/image_left_raw` + `/image_right_raw`；非 `none` → 只有 `/image_combine_raw`。**排除**：同一进程同时输出拼接帧与单目帧（`stereo_layout` 在 `gs130_init` 时定死，`gs130.h:242-243`），也**排除**驱动侧手工拼接/切分 | `stereo_layout:=none` 时 `ros2 topic list` 有 `/image_left_raw`、`/image_right_raw` 且**无** `/image_combine_raw`；反之亦然 | E6；`gs130.cpp:563,581`（互斥返回 `GS130_UNSUPPORTED`） | must `[P]` |
| R-12 | 一次 `gs130_get_*_nv12_frame` 的返回只发布一次，禁止重复发布凑速率 | `ros2 topic hz /image_combine_raw` 在 `fps:=30` 下 → `average rate` 在 **29.0–31.5** 内（不得出现 ≈60 Hz） | `25:166`（Python 版一次读一次发） | must `[P]` |
| R-13 | 每轮轮询必须**排空**队列（循环直到 `GS130_TIMEOUT`），单轮上限 8 帧；禁止"每周期固定最多 2 帧"的写法 | `fps:=30` 跑 60 s：`ros2 topic hz -w 300 /image_combine_raw` → `average rate` ≥ 29.0，且同一输出的 `min` ≥ **0.017 s**、`max` ≤ **0.067 s**（即无 > 2 帧周期的空洞） | E4 基线 `(min 0.024 max 0.043)`；`50_cpp_design.md:57` 的"每次最多 2 帧"是本文的**攻击点** | must `[C]` |
| R-14 | `left_right` 拼接帧的**左半 = 左目**、右半 = 右目（源码依据：左目写在 buffer 基址、右目写在 `base+width`）；该断言**尚无人登板验证过** | 两次运行（`stereo_layout:=left_right` 与 `:=none`）各抓一帧：左半与左目 Y 平面归一化互相关 > 0.99，与右目 < 0.9 | `gs130.cpp:262-271`（源码），E9 只验证过 `top_bottom` 的上下 | should `[C]`（源码已证、板端未证） |
| R-15 | 发布的必须是**真图像**，不得是合成/灰度噪声 | 抓一帧解码 PNG，`mean`/`std` 与 E4/`25:104-113` 同量级；遮住镜头后重抓，`mean` 下降 > 30%；左右目 `mean` 差值 < 20 | `25:104-113` | should `[P]` |

### 组 D：QoS（E1）

| 编号 | 要求 | 判据（命令 → 期望） | 依据 | 优先级 |
|---|---|---|---|---|
| R-16 | 图像 publisher QoS **逐字** `RELIABLE` + `VOLATILE` + `KEEP_LAST(1)`；IMU `RELIABLE` + `VOLATILE` + `KEEP_LAST(200)`；camera_info `RELIABLE` + `TRANSIENT_LOCAL` + `KEEP_LAST(1)` | `ros2 topic info -v /image_combine_raw` → publisher 与 `hobot_codec_*` subscriber 均 `Reliability: RELIABLE`、`Durability: VOLATILE`；`ros2 topic info -v /imu/data` → `RELIABLE`；`ros2 topic info -v /image_left/camera_info` → subscriber `Durability: TRANSIENT_LOCAL` | E1；`25:67-68` | must `[P]` |
| R-17 | **排除**任何能把图像 QoS 降级的参数（不得提供 `qos_reliability`/`qos_durability`），也不得靠环境变量（`hobot_shm`）"修"QoS | 参数表（§3.3）中不存在该类参数；`ros2 param list /gs130_camera` → 无 `qos*`；任意合法启动下 `ros2 topic info -v` 仍为 `RELIABLE` | E1（`shm_fastdds.xml` 不设 reliability） | must |
| R-18 | camera_info 必须 latched：订阅者**晚于**节点启动也能收到 | 节点启动 30 s 后再开一个终端 `ros2 topic echo /image_left/camera_info --once` → 在 3 s 内收到一条（不依赖重发） | `25:243-252` | must `[P]` |

### 组 E：`Image.height` 必须是真实图像高（E2，崩溃级）

| 编号 | 要求 | 判据（命令 → 期望） | 依据 | 优先级 |
|---|---|---|---|---|
| R-19 | `height` = 真实图像高；**禁止** `H*3//2`（NV12 打包高） | `ros2 topic echo /image_combine_raw --once --field height` → `480`（`left_right` 640x480），**不得**为 `720`；同时 codec 存活、无 `process has died [hobot_codec_republish, exit code -11]` | E2 | must `[P]` |
| R-20 | `data` 必须是紧排 NV12：`len(data) == step*height`，Y 平面在 `[0,W*H)`、UV 紧随其后且逐行无 padding（`step == width`） | R-09 的 `len(data)`/`step`/`height` 三条同时成立；`ros2 topic echo /image_combine_raw --once --field step` → 等于 `--field width` | E2；`gs130.h:207`；`32:49` F7 | must `[P]` |

### 组 F：缓冲所有权与内存（C++ 特有风险）

| 编号 | 要求 | 判据（命令 → 期望） | 依据 | 优先级 |
|---|---|---|---|---|
| R-21 | 每一个 `gs130_get_*_nv12_frame` 返回的缓冲必须**恰好 free 一次**，含：第二目（`image_right`）、错误分支、`stereo_layout≠none` 时被丢弃的 `image_right`；**排除**任何"依赖析构/智能指针自动 free SDK malloc 缓冲"的写法 | `ros2 launch … ` 跑 5 min，每 10 s 采 `grep VmRSS /proc/$(pgrep -f gs130_camera_node)/status`：**末值 − 首值 < 8 MB**（640x480 拼接帧 921600 B × 30 fps ⇒ 泄漏 5 min 会 > 8 GB，必被发现）。**负对照**：故意删掉一处 `free()`，该判据必须 FAIL（否则测试无判别力） | J3；`gs130.h:231,249`；`gs130.cpp:212-218` | must `[C]` |
| R-22 | v0.2.0 **排除**零拷贝/借出内存/自定义 deleter：一律 `memcpy` 进 `msg.data` 后立即 free | 代码审查：`grep -rn "borrow_loaned_message\|unique_ptr<.*Image.*,.*free\|hb_mem" ros/gs130_ros/src` → 无输出；`ros2 topic info -v /image_combine_raw` 的 `Type` 为 `sensor_msgs/msg/Image`（非 `hbm_img_msgs`） | `32:313-321`（真零拷贝需 SDK 导出 hb_mem fd） | must |

### 组 G：IMU

| 编号 | 要求 | 判据（命令 → 期望） | 依据 | 优先级 |
|---|---|---|---|---|
| R-23 | `/imu/data` 存在，平均速率 ≈ `odr` 且**不受 fps 限制**；`odr:=200` → 190–212 Hz | `ros2 topic hz -w 1000 /imu/data` 跑 30 s → `average rate` 在 **190–212** | `25:57`（实测 202.9 Hz）、`25:58` | must `[P]` |
| R-24 | 消息字段逐字冻结：`orientation=(0,0,0,1)`、`orientation_covariance[0]=-1`；`gyro` 单位 rad/s、`accel` 单位 m/s²（**含重力**，静止朝上 z≈+9.81）；`accel/gyro` 协方差全 0（有测量值、方差未知）；**不发** `temp`、**不发** `is_fsync`、不做姿态解算 | `ros2 topic echo /imu/data --once` → `orientation_covariance` 首元素 `-1.0`、其余 0；`angular_velocity_covariance`/`linear_acceleration_covariance` 全 0；静止时 `linear_acceleration.z` 在 9.0–10.6 之间 | `11:247-262`（协方差项按实现修订，见 §3.2 差异表） | must `[P]` |
| R-25 | **排除**在驱动侧过滤 `is_fsync` 包：SDK 交给调用方的每个包都必须发布（发布数 == pop 数）；`publish_imu:=false` 时仍必须把 IMU 队列排空并计数（否则 R-48 的看门狗失去证据） | 取相邻两条 5 s 日志的 `imu=` 差值 ÷ 5，与同期 `ros2 topic hz -w 1000 /imu/data` 的 `average rate` 之差 ≤ 3%；`publish_imu:=false` 启动后 `ros2 topic list` 无 `/imu/data`，但日志 `imu=` 仍单调增长 | `gs130.cpp:177-203`（`is_fsync` 包也进 FIFO）；E10（`publish_imu:=false` → 话题不存在） | must `[C]` |
| R-26 | 无 IMU 时（`gs130_get_imu_name()` 返回 NULL）：不创建 `/imu/data`、不轮询 IMU、打一次 WARN 含 `IMU not present`，节点继续；且**不得**把 `gs130_get_imu_packet` 在无 IMU 时返回的 `GS130_PARAM_ERROR` 当致命错误 | 本板硬件（GS130WI）**无法触发**该路径 ⇒ 本项只能代码审查 + 待 W 模组；判据写成：代码审查确认 `publish_imu` 只影响 publisher、且无 IMU 时不进入轮询 | `gs130.cpp:609`；`11:415` | should `[C]`（**未验证**） |
| R-27 | `device:=GS130W` 在本硬件（GS130WI）上必须**快速失败**：退出码 1，日志含 `GS130_NOT_FOUND`，且不产生 `frames=` 报告行 | `ros2 launch gs130_ros gs130_camera.launch.py device:=GS130W; echo $?` → `1`，日志含 `GS130_NOT_FOUND` | E10 实测 | must `[C]` |

### 组 H：标定 → CameraInfo

| 编号 | 要求 | 判据（命令 → 期望） | 依据 | 优先级 |
|---|---|---|---|---|
| R-28 | 启动时各发**一次** `/image_left/camera_info` 与 `/image_right/camera_info`（latched，见 R-18）；`frame_id` 与对应图像完全一致；`header.stamp` 不得为 0 | `ros2 topic echo /image_left/camera_info --once` → `header.frame_id == camera_left`、`header.stamp.sec > 0`；`/image_right/camera_info` → `camera_right` | `11:279-280`；`25:89` | must `[P]` |
| R-29 | `distortion_model`/`D` 规则（唯一版本）：本硬件 EEPROM 为鱼眼 → `raw`/`resize` 下 `equidistant` + 前 4 个系数（640x480 实测 `[-0.025173, 0.012189, -0.013019, 0.002486]`）；`PINHOLE`→`rational_polynomial` + 8 个系数（**偏离 `11:303` 的"截断到 5 元 plumb_bob"**，理由：ROS 原生支持 8 元 rational 模型、不丢系数）；`rect` 下强制 `plumb_bob` + 5 个 0（SDK 原地清零 D 但枚举仍是 FISHEYE，`rectify.hpp:20-27`）；D 全 0 → `plumb_bob` + 5 个 0 | `--field distortion_model` → `equidistant`；`--field d` → 4 个元素且与上列数值误差 < 1e-4；`mode:=rect` 时 → `plumb_bob` 且 5 个 0 | `25:89-92`、`25:149`；`11:291-304`；`rectify.hpp:20-27` | must `[P]` |
| R-30 | `width`/`height` = **单目输出尺寸**（640x480），不得填拼接尺寸；`K` 逐元素复制 SDK 的 `K`（SDK 已在 `init()` 内按 VSE ROI+scale 回写到输出分辨率，节点**不得**再乘任何缩放） | `--field width` → `640`、`--field height` → `480`；`--field k` → `[386.85, 0, 304.62, 0, 387.12, 245.06, 0, 0, 1]`（±0.5）；`mode:=rect` 640x480 → `fx≈362.07, cx=320, cy=240` | `25:90-92`、`25:149`；`32:55` F13 | must `[P]` |
| R-31 | `R` = 9 元单位阵；`P = [fx,0,cx,0, 0,fy,cy,0, 0,0,1,0]`（即 K 重述、`Tx=0`）；`binning_x=y=0`、`roi` 全 0、`do_rectify=false`。**偏离 `11:285-286` 的"R/P 全 0"**：全 0 会让 `image_geometry` 与 `hobot_stereonet`（要求 `P[0]≠0`）判为无效数据；`Tx=0` 是因为单目无基线，基线由 `/tf_static` 表达 | `--field r` → 9 个 (1,0,0,0,1,0,0,0,1)；`--field p` → 12 个数且 `p[0]=fx`、`p[3]=0`、`p[5]=fy`、`p[6]=cy`、`p[10]=1`；`--field binning_x` → `0`；`--field roi` → 全 0 | `11:285-286`（被本文覆盖，见 §5 D-2）；`32:234`（stereonet 要求 `P[0]≠0`）、`25:92-93` | must（争议项，见 §5 D-2） |
| R-32 | EEPROM 缺失（`GS130_NOT_FOUND`）时：不发布 camera_info、WARN 一次含 `calibration not available`，节点继续 | 本硬件有 EEPROM ⇒ **不可触发**，标为未验证；判据形式：代码审查 + 待无 EEPROM 样机 | `11:580` | may `[C]` |

### 组 I：外参 → 静态变换

| 编号 | 要求 | 判据（命令 → 期望） | 依据 | 优先级 |
|---|---|---|---|---|
| R-33 | 启动时向 `/tf_static` 发两条：`camera_left → camera_right` 与 `camera_left → imu_link`（IMU 不存在时只发前者）；方向规则锁死为 `T_parent_child = relative_T(child, parent)`，即 `camera_left→camera_right` 用 `relative_T(CAMERA_RIGHT, CAMERA_LEFT)`；`publish_tf:=false` 时一条都不发 | `ros2 run tf2_ros tf2_echo camera_left camera_right` → 5 s 内输出 `Translation: [0.070, 0.000, -0.002]`（基线 `0.070316 m`，±0.001）；`tf2_echo camera_left imu_link` → 有输出；`publish_tf:=false` 时 `ros2 topic list` 无 `/tf_static` | `25:47,69`；`32:456`；`gs130.h:392-414` | must `[P]` |
| R-34 | frame 名只用 `camera_left`/`camera_right`/`imu_link`（**排除** `_optical_frame` 后缀），且 README 必须声明"SDK 的相机轴约定未文档化，本节点不声称满足 REP-103/REP-105"；`mode:=rect` 下外参是**虚拟**平行双目系（`gs130.h:341`），必须在 README 与启动 INFO 中点名 | `ros2 topic echo /tf_static --once` → `child_frame_id` 集合 ⊆ {`camera_right`,`imu_link`}；README 含 `REP-105` 与 `virtual` 两个关键词的说明段（文档审查） | `gs130.h:338-345`；`32:458` | should `[P]`（TF 本身已实测，轴约定未验证） |

### 组 J：时间戳

| 编号 | 要求 | 判据（命令 → 期望） | 依据 | 优先级 |
|---|---|---|---|---|
| R-35 | 启动时一次性确定 `offset_ns = now_ns − device_ts_ns`（图像与 IMU **共用**），运行期**恒定**；`header.stamp = device_ts_ns + offset_ns`；**排除**逐消息用 `now()` 覆盖、滑动平均、任何插值 | 日志出现**恰好一次** `using a constant offset of <N> ns`；30 s 内 `ros2 topic echo /image_combine_raw --field header.stamp` 与同时刻 `date +%s%N` 同量级（`|stamp − wall| < 1 s`）；相邻帧 stamp 差在 33.3 ms ± 15% | E5；`25:48,96-97`（实测 offset `1789385065931459293`） | must `[P]` |
| R-36 | 图像与 IMU 时间戳同域、可直接比较；README 必须写明 IMU 是 FSYNC 锚点后的突发、其 stamp 可以早于最近一帧图像（SDK 补点语义） | 同时采一条 `/imu/data` 与一条 `/image_combine_raw` → `|Δ|` ≤ 50 ms（1.5 帧周期）；README 含 `fsync`/补点说明段 | E5（`imu − frame = 8.6 ms`）、`11:358` | must `[P]` |
| R-37 | 两目 stamp **各自独立**（`gs130.cpp:332-333` 各用自身 `ts`，对齐窗口 `cycle_ns = 1e9/fps/2` ⇒ 30 fps 下差 ≤ **16.7 ms**）；**排除**为凑"左右相同"而强制赋同一个值；节点必须每 5 s 打印原始差值以便证伪 | `stereo_layout:=none` 时日志每 5 s 含 `stamp_delta_ms=`；采两目各 30 帧比对 `header.stamp`：对应帧 `|Δ| ≤ 16.7 ms` 且与日志 `stamp_delta_ms` 相符（差异 < 2 ms） | `gs130.cpp:228,316-333`；`11:142`（"两目 stamp 相同"是**错的**，本文修正） | should `[C]` |
| R-38 | 拼接帧 stamp = **FSYNC 绑定目**的 stamp（preset `fsync_camera = GS130_CAMERA_RIGHT_IDX` ⇒ 右目），**不是**左目；节点启动必须打一行说明来源 | 启动日志含 `stitched stamp source: camera_right`；README 同句 | `gs130.cpp:335`；`gs130_define.h:33`；`11:142`（"取左目"是**错的**，本文修正） | should `[C]` |
| R-39 | `use_sim_time:=true` 时不得静默产出错误时间轴：至少打 WARN 说明 offset 会随 `/clock` 变化；若 rclcpp 在无 `/clock` 时使 `now()` 停 0 导致 offset 无意义，则必须 `FATAL` + 退出码 2 | `ros2 launch gs130_ros gs130_camera.launch.py --ros-args -p use_sim_time:=true` → 日志含 `WARN`（或退出码 2 的 FATAL），**不得**出现 stamp 恒为 0 却继续发布 | `11:344`；`50_cpp_design.md:77-79`（开放问题 3） | may `[C]` |

### 组 K：参数与 fail-fast 校验

| 编号 | 要求 | 判据（命令 → 期望） | 依据 | 优先级 |
|---|---|---|---|---|
| R-40 | 参数集合与默认值逐字见 §3.3；所有校验必须在**打开相机之前**完成；非法输入的退出码语义：参数/配置错误 `2`、SDK 初始化/启动/相机通路失败 `1`、正常关停 `0` | **逐行执行 §2.1 的 10 行校验矩阵（V1–V10）**，每行用 `ros2 launch … <该行输入>; echo $?` 记录退出码与 stdout/stderr | E10 全表；`50_cpp_design.md:87-89`（C++ 侧参数类型异常是本文的攻击点） | must `[P]`（V5、V8、V10 三行为 `[C]`） |
| R-41 | 校验失败时不得留下任何残留：无 `frames=` 报告行、无 `first frame` 日志、进程已退出、设备未被打开 | §2.1 每一行执行后：`pgrep -f gs130_camera_node` → 空；`ros2 node list` 中无 `gs130_camera`；`grep -c "frames=" <log>` → `0` | E10 末段（"所有这些失败都发生在相机被打开之前"） | must `[P]` |
| R-42 | 分辨率支持**由 SDK 裁决**，节点不得自建白名单猜测；节点只校验"偶数、>0、raw 精确等于 1088x1280"。**排除**给 `width`/`height` 设上限（E10 实测 1920x1080 可用，已超过传感器宽 1088，`11:375-376` 的 `[16,1088]` 上界与实测冲突） | `width:=1920 height:=1080` → 退出码 0 且 `ros2 topic hz` ≥ 29.0；`width:=864 height:=480` → 退出码 **1** 且日志含 `GS130_UNSUPPORTED`（SDK 裁决，不是节点自造理由）；README 给出 `roi_ratio_exact` 的整除规则（`vse.c:14-23`）作为"哪些尺寸会被拒"的解释，并用 E10 六行做对照用例 | E10；`vse.c:14-23`（该规则可同时解释 320x240/640x480/1280x720/1920x1080 通过、864x480/1024x600 被拒） | must `[C]` |
| R-43 | 校验必须根据参数**类型**区分路径：`int`/`double`/`bool` 参数被 launch 传入字符串时必须先解析再校验，错误消息必须指名参数名与收到的原值；**排除**让 `rclcpp` 的类型异常逃逸到 `std::terminate` | §2.1 的 V5（`width:=abc`）与 V10（`publish_imu:=maybe`）两行 | `50_cpp_design.md:87-89`；`launch_arguments.py:25-34`（Python 版用 `ParameterValue(value_type=…)`，C++ 侧必须有等价且 stdout 可判的失败路径） | must `[C]` |
| R-44 | **排除**动态重配置：`ros2 param set` 可以成功但不产生任何行为变化；README 必须写明"改参数 = 重启节点" | 运行中 `ros2 param set /gs130_camera width 320` 后：`ros2 topic echo /image_combine_raw --once --field width` 仍为原值（如 `1280`）；且 `/image_combine_raw` 速率不变 | `11:419`（SDK 只在 `gs130_init` 下配置硬件） | must `[C]` |

### 组 L：关停与资源释放

| 编号 | 要求 | 判据（命令 → 期望） | 依据 | 优先级 |
|---|---|---|---|---|
| R-45 | 收到 SIGINT：**3 s 内**以退出码 0 结束；日志含 `camera released`；不依赖任何析构/GC；`nginx` 不是本包的进程，本包不得去杀它（README 给出显式清理命令） | `ros2 launch gs130_ros gs130_camera.launch.py & LP=$!; sleep 10; t0=$(date +%s%N); kill -INT $LP; wait $LP; echo "exit=$? ms=$(( ($(date +%s%N)-t0)/1000000 ))"` → `exit=0` 且 `ms < 3000`；日志含 `camera released`；`pgrep -f gs130_camera_node` → 空；web 链路测试后另加 `ss -ltn \| grep :8000` → 空（按 README 清理后） | `25:118-131`；E4（nginx 孤儿） | must `[P]` |
| R-46 | 释放后**立即**可被新进程接管（相机独占已释放） | 紧接上一条再启动一次：30 s 内 `ros2 topic hz /image_combine_raw` ≥ 29.0；不出现"停帧"（R-48 的 ERROR 不出现） | `25:131`（port 8000 free、无残留）、`gs130.h:172-180`（`deinit` 复位并断电传感器） | must `[P]` |
| R-47 | 关停序列在所有退出路径（含 `gs130_init`/`gs130_start` 失败、异常、`SIGTERM`）都必须执行 `gs130_stop → gs130_deinit → gs130_destroy`；**排除**在 `deinit` 之前 `destroy`、**排除**关停期再调用 `gs130_get_*` | 代码审查：`gs130_destroy` 只有一个调用点且其后无设备调用；每条早退路径都经过同一个清理函数；`pkill -TERM -f gs130_camera_node` → 退出码 0 且日志含 `camera released` | `gs130.h:145-153`；实测缺陷 2（`25:141`） | must `[C]` |

### 组 M：失败模式诊断（E8）

| 编号 | 要求 | 判据（命令 → 期望） | 依据 | 优先级 |
|---|---|---|---|---|
| R-48 | **停帧看门狗**：连续 2 个报告周期（默认 10 s）`frames` 不增长而 `imu` 仍在增长 → 打**一次** `ERROR`，文案含 `no camera frames` 与 `another process`，**不退出**、IMU 继续发布 | 起两个实例（`gs130_camera.launch.py` ×2，第二实例用不同 `frame_id_camera` 以区分日志）：先启动者 15 s 内出现该 ERROR；其 `ros2 topic hz /imu/data` 仍 ≥ 190 Hz；后启动者 `ros2 topic hz /image_combine_raw` ≥ 29.0 | E8；`25:168`（该 ERROR 已实测触发） | must `[P]` |
| R-49 | **排除**把"队列空"与"通路故障"混为一谈：轮询循环必须直接调用 `gs130_get_nv12_frame`/`gs130_get_stereo_nv12_frame` 并按返回值分支；**禁止**以 `gs130_available_camera()==0` 作为"无数据"的唯一判据（故障时它返回 0，会把故障伪装成空闲并永久静默） | 代码审查（逐条核对，无命令可判）：① 轮询函数中存在对 `GS130_TIMEOUT` 与"其他非 OK"的分支；② 不存在 `if(gs130_available_camera(...)==0) return;` 形态的唯一门；③ `stereo_layout` 决定调用哪个 getter，另一个 getter 返回的 `GS130_UNSUPPORTED` 必须在启动期被排除（不得进入运行期分支） | `gs130.cpp:550,562,563,580,581,598,608` | must `[C]` |
| R-50 | 致命/非致命分界唯一：相机通路非 `GS130_TIMEOUT` 错误 → `ERROR` + 退出码 1；IMU 通路错误 → 停发 `/imu/data`（`ERROR` 一次）、相机继续、退出码不变；空队列、无 IMU、无标定、时间戳异常 → 非致命 | 故障注入不可在板端廉价复现 ⇒ 判据为代码审查 + 退出码表（R-40）一致性；`kill -9` 类外部杀死不属本项 | `11:584-588`；`gs130.h:28`（`GS130_THREAD_CLOSED` 要求 deinit+init 恢复） | must `[C]`（**仅代码审查可判**） |
| R-51 | 周期日志键名冻结（周期固定 5 s，**不提供**周期参数与关闭开关）：每 5 s 一行，含 `frames=`、`imu=`、`camera_fps=`、`imu_hz=`、`offset_ns=`；`stereo_layout:=none` 时另含 `stamp_delta_ms=`；启动阻塞时每 `start_timeout_s` 打一次 `WARN` 含 `waiting for the IMU FSYNC handshake`，且**不得**中止 `gs130_start()` | `ros2 launch … 2>&1 \| grep -m1 -E "frames=.*imu=.*offset_ns="` → 命中；`grep -c "frames=" <log>` 在 60 s 运行中 ≈ 12 ± 2 | `25:52-54`；`11:608-613`；`gs130.h:185`（start 会等 FSYNC） | should `[P]` |

---

## 3. 外部接口

### 3.1 话题表（本包发布/约定）

| # | 话题 | 类型 | Reliability | Durability | History/Depth | frame_id | 出现条件 | 状态 |
|---|---|---|---|---|---|---|---|---|
| T1 | `/image_combine_raw` | `sensor_msgs/msg/Image` | `RELIABLE` | `VOLATILE` | `KEEP_LAST` 1 | `frame_id_camera`（默认 `camera_left`） | `stereo_layout != none` | **FROZEN** |
| T2 | `/image_left_raw` | `sensor_msgs/msg/Image` | `RELIABLE` | `VOLATILE` | `KEEP_LAST` 1 | `frame_id_camera` | `stereo_layout == none` | **FROZEN** |
| T3 | `/image_right_raw` | `sensor_msgs/msg/Image` | `RELIABLE` | `VOLATILE` | `KEEP_LAST` 1 | 固定 `camera_right` | `stereo_layout == none` | **FROZEN** |
| T4 | `/imu/data` | `sensor_msgs/msg/Imu` | `RELIABLE` | `VOLATILE` | `KEEP_LAST` 200 | `frame_id_imu`（默认 `imu_link`） | 探测到 IMU 且 `publish_imu` | **FROZEN** |
| T5 | `/image_left/camera_info` | `sensor_msgs/msg/CameraInfo` | `RELIABLE` | `TRANSIENT_LOCAL` | `KEEP_LAST` 1 | `frame_id_camera` | 有标定且启动成功 | **FROZEN** |
| T6 | `/image_right/camera_info` | `sensor_msgs/msg/CameraInfo` | `RELIABLE` | `TRANSIENT_LOCAL` | `KEEP_LAST` 1 | `camera_right` | 有标定且启动成功 | **FROZEN** |
| T7 | `/tf_static` | `tf2_msgs/msg/TFMessage` | 继承 `tf2_ros::StaticTransformBroadcaster`（Humble：`RELIABLE`+`TRANSIENT_LOCAL`+`KEEP_LAST 100`），**非本包自选** | — | — | 父 `camera_left` | `publish_tf` | **FROZEN**（QoS 为继承值） |
| T8 | `/image_combine_jpeg` | `sensor_msgs/msg/CompressedImage` | 由 `hobot_codec` 决定 | — | — | — | 仅 `gs130_web.launch.py` | 外部（既有节点发布，**本包不发**） |

**FROZEN（冻结）**：T1–T6 的话题名、消息类型、QoS 四项（reliability/durability/history/depth）；T1 的 `encoding="nv12"`、`step=width`、`height=真实高`、`len(data)=step*height`；T3/T6 的 `camera_right`；T7 的两条边与方向规则。理由：T1 是 D-Robotics 参考链的入口，只有逐字匹配才能让 `hobot_codec` + `websocket` 零修改工作（E1/E2/E4）；T4–T6 已被 Python 版实测通过并被下游消费。

**OPEN（开放，允许改但需评审）**：T1/T2/T5 的 `frame_id` 取值（本文默认 `frame_id_camera`＝`camera_left`；Python 版把 T1 硬编码为 `camera`——见 §5 与 Q1）；T5/T6 的 `P[3]`（本文取 0，见 §5 D-2 / Q2）；T1 的 depth（本文取 1）；`/gs130/status`（本文**不提供**，Python 版亦未实现）。

### 3.2 与已实现 Python 版的接口差异（回归对照，评审不得当成退化）

| 项 | Python 实现（`ros/gs130_ros`） | 本文（C++ v0.2.0） | 理由 |
|---|---|---|---|
| 可执行名 | `camera_node` | `gs130_camera_node` | 语言更换；`ros2 run gs130_ros camera_node` **故意破坏**（README 必须写） |
| `/image_combine_raw` 的 frame_id | 硬编码 `camera` | `frame_id_camera`（`camera_left`） | `camera` 在 TF 树中不存在（TF 父是 `camera_left`），RViz/tf2 无法变换该图 |
| 轮询策略 | 每周期最多 2 帧 | 每轮排空（上限 8） | R-13：固定 2 帧在调度抖动下会积压/丢帧 |
| 校验时机 | `fps`/`mode`/`layout`/尺寸在启动前，`odr` 交 SDK | `odr` 也 fail-fast（退出码 2） | 一致性与 E10 的"不触碰相机"原则 |
| 泄漏判据 | 无 | R-21（RSS + 负对照） | C++ 显式 `free` 是新风险面 |
| 两目 stamp | 未声明（实现里分别取各自 ts） | R-37 明确"可不同，≤16.7 ms"，并要求 `stamp_delta_ms` 日志 | 修正 `11:142` 的错误断言 |
| 拼接帧 stamp | 未声明 | R-38：FSYNC 绑定目（右目） | 修正 `11:142` 的错误断言 |
| camera_info `D`（PINHOLE） | `rational_polynomial` + 8 系数 | 同（**偏离 `11:303` 的截断规则**） | 不丢系数（R-29） |
| IMU `accel/gyro` 协方差 | 全 0 | 同（**偏离 `11:258-261` 的 `-1`**） | 有测量值、方差未知 ⇒ ROS 惯例 0；`-1` 表示"无估计" |
| `/tf_static` 默认 | 发布（`publish_tf:=true`） | 同（**偏离 `11:52`**） | 外参是真实硬件能力，E4/E7 已消除反对理由 |

**已知并行实现的命名冲突（本文交付时已存在，必须由评审裁定）**：`ros/gs130_ros/CMakeLists.txt:23` 当前写的是 `add_executable(camera_node …)`，与 R-01 冻结的 `gs130_camera_node` 冲突。建议实现方改名（R-05/R-45/R-48 的判据与 `pgrep -f gs130_camera_node` 都按新名书写），否则 R-01 判据会直接 FAIL。

### 3.3 参数表

全部为**启动期**参数（`Reconfig = No`：运行期修改无任何效果，见 R-44）。下表 13 个名字为契约参数；`ros2 param list /gs130_camera` 除它们之外只允许出现 rclcpp 自动声明的 `use_sim_time`（由 `ros2 launch/run` 的 `--ros-args -p use_sim_time:=…` 注入，见 R-39），**不得**出现其他名字。

| 参数 | 类型 | 默认 | 取值/范围 | 校验与失败行为 |
|---|---|---|---|---|
| `platform` | string | `"RDKX5"` | 仅 `"RDKX5"` | 其他值 → 退出码 2 |
| `device` | string | `"GS130WI"` | `"GS130WI"`、`"GS130W"` | 本硬件 `GS130W` → 退出码 1（`GS130_NOT_FOUND`） |
| `mode` | string | `"resize"` | `"raw"`、`"resize"`、`"rect"` | 其他值 → 退出码 2；`raw` 必须 `1088x1280` 且 `stereo_layout=none` |
| `width` | int | `640` | 偶数、>0；`raw` 时必须 `1088` | 奇数/非正整数 → 2；**不设上限**（R-42） |
| `height` | int | `480` | 偶数、>0；`raw` 时必须 `1280` | 同上 |
| `fps` | int | `30` | `[1, 33]` | 越界 → 2（`line_length=1400/frame_length=1500` 下读出率上限约 33） |
| `odr` | int | `200` | `200`、`500`（ICM-42688-P，`icm42688.cpp:56-60`） | 其他值 → 2，消息引用 `gs130_get_imu_info()` 字符串 |
| `stereo_layout` | string | `"left_right"` | `"none"`、`"left_right"`、`"right_left"`、`"top_bottom"`、`"bottom_top"` | 其他值 → 2；`mode=raw` + 非 `none` → 2 |
| `frame_id_camera` | string | `"camera_left"` | 非空 | 空串 → 2 |
| `frame_id_imu` | string | `"imu_link"` | 非空 | 空串 → 2 |
| `publish_imu` | bool | `true` | `true`/`false` | 非法布尔 → 2；`false` **只**停发布（SDK 仍配 IMU 并做 FSYNC 握手，R-25） |
| `publish_tf` | bool | `true` | `true`/`false` | 非法布尔 → 2 |
| `start_timeout_s` | double | `10.0` | `[1, 600]` | 越界 → 2；只作首帧等待与 `gs130_start` 阻塞 WARN 的周期 |

**launch 专用参数**（不属于节点契约，改了不影响节点；仅 `gs130_web.launch.py` 声明）：`image_topic`(`/image_combine_raw`)、`jpeg_topic`(`/image_combine_jpeg`)、`jpg_quality`(`85.0`)、`websocket_channel`(`0`)。语义：`websocket_channel:=0` 显示**拼接帧左半 = 左目**（R-14 待板端确认）。

**FROZEN**：13 个参数的名字、类型、默认值、取值域与 `Reconfig=No` 语义；退出码语义（0/1/2）。
**OPEN**：`odr` 取值集合是否应改为从 `gs130_get_imu_info()` 解析（Q7）；是否提供 `stamp_offset_ns` 调试参数（Q8，本文**不提供**）；`width/height` 是否需要"建议值"提示（本文**不做**）。

**明确不提供的参数（排除）**：`sensor_width/height`、`line_length/frame_length`、`left_addr/right_addr`、`bus`、`bus_mipi_rx`、`bus_reset_gpio`、`fsync_camera`、`tuning_file`、`accel_fsr_g`、`gyro_fsr_dps`、`accel_bw_sel`、`gyro_bw_sel`、`camera_fifo_depth`、`imu_fifo_depth`（全部由 preset 定死，`gs130_define.h:21-43`）；`qos_reliability`、`qos_durability`（R-17）；`publish_status`、`publish_combine`、`publish_per_eye`、`publish_camera_info`（话题集合由 `stereo_layout` 决定，R-11）；`use_compressed`、`image_transport`、`enable_depth`、`stereonet_*`、`enable_ahrs`、`temperature_topic`、`ns`、`respawn`/`auto_restart`。

---

## 4. 非目标（明确不做，且说明用户该用什么）

| 非目标 | 一句话出口 |
|---|---|
| 自研图像编码 / 转码 / 缩放 / 色彩空间转换 | 用既有 `hobot_codec_republish`（本包只发 NV12） |
| 自研网页 / HTTP / WebSocket 服务 | 用既有 `websocket` + 其 nginx 的 `:8000`（`channel` 复用同一端口，E7） |
| 深度计算 / `hobot_stereonet` 接线 | 用既有 `hobot_stereonet`；三点约束必须写在 README：它默认拉起 `mipi_cam`（与本节点互斥，必须 `use_mipi_cam:=False`）、它要上下堆叠布局（可由 `stereo_layout:=top_bottom` 单次运行满足，但与 web 链路的 `left_right` **互斥**，R-11）、它要求有效 `CameraInfo.P`（v0.2.0 的 `P[3]=0` 不满足，`32:234`）。因此 v0.2.0 **接不上**，属 follow-up（`32:250-284`） |
| 驱动侧手工拼接 / 切分左右目 | 用 SDK 的 `stereo_layout`（E6 实测可用、零成本） |
| 零拷贝 / `hobot_shm` 零拷贝 / 借出内存 | 用普通堆缓冲 + 一次 memcpy（真零拷贝需 SDK 导出 hb_mem fd，属核心改动，`32:313-321`） |
| 动态参数重配置（`rcl_interfaces` 回调） | 改参数后重启节点（SDK 只在 `gs130_init` 下配置硬件） |
| 多相机 / 多实例 / 与 `mipi_cam` 共存 | 先停掉另一个持有者（GS130 是独占设备，E7/E8）；需要第二路画面用 `websocket` 的 `channel` |
| ROS 1 支持 | 用 ROS 2 Humble / TROS |
| IMU 姿态解算（AHRS / 互补滤波 / 卡尔曼） | 用下游 `imu_filter_madgwick` / `robot_localization`（本包只发原始 accel/gyro） |
| 温度话题（`gs130_imu_packet_t.temp`） | 直接调 SDK 取温度（`sensor_msgs/Imu` 无对应字段，不引入自定义消息） |
| 相机几何/总线参数外露（bus/addr/tuning/fifo/fsr/bw） | 用 `gs130_define.h` 的 preset 或直接写 SDK 程序（暴露它们等于允许用户改坏硬件配置） |
| 自动恢复 / `respawn` / 自动重连 | 手动重启（`GS130_THREAD_CLOSED` 的恢复路径是 `deinit`+`init`，v0.2.0 不实现；`respawn=True` 在设备忙时会变成重启风暴） |
| `/gs130/status`（`diagnostic_msgs`） | 用 5 s 周期日志（R-51 的键名）与退出码（Python 版也未实现） |

---

## 5. 我认为最可能**错**或**做不到**的三条

### D-1 单线程 executor + rclcpp 定时器就能满足 R-08/R-13/R-23（攻击 `50_cpp_design.md:53-62`）

- 为什么可能是错的：① `publish()` 在默认 Fast-DDS 下是**调用内同步**完成的（E3 在 Python 侧实测 `publish` 5.07 ms / 帧；C++ 侧无实测数字），单线程下图像发布与 IMU 轮询**互相阻塞**；② 1920x1080 每目拼接（`left_right` ⇒ 3840x1080 NV12 = 6.2 MB/帧）在 30 fps 下需要 ≈186 MB/s 的拷贝 + DDS 序列化再一份 ≈373 MB/s，R-13 的"排空 FIFO"要求可能让某一轮里连续做 4 次 6.2 MB 拷贝，把 IMU 轮询推迟到 >5 ms；③ SDK 侧所有公共 API 共用 `dev->mtx`（`gs130.cpp:52,549,598`），"两个定时器并发"其实也会串行化。
- 判决性实验：给节点加分段计时（pop/copy/publish）与 IMU 相邻包间隔直方图，跑 `fps:=30` 于 640x480、1280x720、1920x1080 各 60 s。
- 判定阈值：任一组出现（a）`ros2 topic hz -w 300 /image_combine_raw` 平均 < 29.0 Hz，或（b）`/imu/data` 间隔 p99 > 2×(1/odr)（即 10 ms @200 Hz），或（c）单帧 publish+copy > 20 ms ⇒ **单线程模型被否决**，必须改为专用采集线程 + 有界队列（并重新评审 R-21 的 free 时机）。

### D-2 camera_info 的 `R`=单位阵、`P=[K|0]`（R-31）两边都不讨好

- 为什么可能是错的：`P[3]=0` ⇒ 基线 0 m；`hobot_stereonet` 用 `calib_method:=none` 时从 `P` 取 `baseline = abs(p[3]/p[0])`（`32:234`），读到 0 会得到"零基线"的深度（或直接无效）。另一方面 `R`=单位阵 + `D` 非 0（鱼眼 4 系数）在语义上是"已矫正 + 有畸变"的矛盾组合：`raw`/`resize` 的图**并未**矫正。
- 判决性实验：① `ros2 topic echo /image_left/camera_info --once --field p` 与 `tf2_echo camera_left camera_right` 的基线比较；② 用 `hobot_stereonet`（`calib_method:=none`，`use_mipi_cam:=False`，`stereo_image_topic` 指向 `top_bottom` 话题）跑一次，看是否打印 `receive stereo image` 或静默无输出；③ 用 `ros2 run image_proc image_proc`（若板端有）确认不报"invalid CameraInfo"。
- 判定阈值：若 stereonet 因 `P[3]=0` 静默无输出，则 R-31 必须改为 `P[3] = −fx · 0.070316`（并把 `D` 在 `raw`/`resize` 下保留、在 `rect` 下清零的关系写清）；若 stereonet 正常出深度且尺度与卷尺基线一致（±5%），R-31 保持。

### D-3 一次性 offset 的精度承诺"≤ 一个帧周期"（R-35/R-36）

- 为什么可能是错的：① E5 的原始数据里 `frame_ts − monotonic = +6 151 716 247 ns`（**+6.15 s**），说明 SDK 时间戳与 `CLOCK_MONOTONIC` 之间存在一个**来源未解释的常量项**；本方案假定它随时间**恒定**，一旦它随温度/时钟源漂移，"采样一次"就会留下随时间增长的绝对误差；② offset 用"取到帧那一刻的 `now()`"采样，而被取的帧在 SDK FIFO（depth 4，`DROP_OLD`）里可能已积压若干帧 ⇒ 残差上界是**FIFO 年龄 + 轮询延迟**（最坏 ≈ 4/30 s ≈ 133 ms），**不是** 1/fps = 33 ms；③ 长时漂移从未被测（`25:157` 自述未覆盖）。
- 判决性实验：写一个 20 行 C 程序（不经过 ROS）每 100 ms 采样 `(gs130_get_*_nv12_frame().timestamp_ns, clock_gettime(CLOCK_MONOTONIC))`，跑 60 min；统计 `(ts − monotonic)` 的均值、标准差与 1 h 内的漂移。
- 判定阈值：若该差值标准差 > 5 ms 或 1 h 内漂移 > 50 ms ⇒ R-35 的"≤ 一个帧周期"必须撤销并改写为实测值（例如"绝对时刻误差 ≤ X ms，且需定期重锚"）；同时 R-36 的容差（50 ms）按实测重算。

（次要可疑项，未列入三条：shutdown 的 3 s 预算在"SDK 线程正卡在 `get_frame` 的 100 ms 超时或半帧对齐重取循环里"时是否成立——`gs130.cpp:316-325` ——以及 R-39 的 `use_sim_time` 行为。）

---

## 6. 给架构团队的开放问题（每个都有可检查的答案）

| # | 问题 | 可检查的答案形式 |
|---|---|---|
| Q1 | `/image_combine_raw` 的 `frame_id` 取 `frame_id_camera`（本文，默认 `camera_left`）还是保留 Python 版的硬编码 `camera`？ | 答案必须给出：默认 web 链（`hobot_codec` + `websocket`）里是否有任何代码读 `header.frame_id`（`grep` 源码给行号），以及实测把 frame_id 改成 `camera` 后 `ros2 topic hz /image_combine_jpeg` 是否仍 ≥ 29.9。若无人读，则取本文方案（`camera` 在 TF 树中不存在） |
| Q2 | `CameraInfo.P[3]` 取 0（本文）还是 `−fx·baseline`？ | 用一次 `hobot_stereonet`（`calib_method:=none`、`use_mipi_cam:=False`）的实跑结果回答：日志是否出现 `receive stereo image`、是否输出非空 `disparity`/点云。有输出 ⇒ 保持 0 亦可接受；无输出 ⇒ 必须改为 `−fx·baseline` |
| Q3 | `odr` 的合法集合应硬编码 `{200,500}`，还是从 `gs130_get_imu_info()` 的字符串解析？ | 打印一次 `gs130_get_imu_info()` 原文（板端 5 行 C 程序），确认它是否列出全部 ODR；若列出，则实现必须从该字符串校验（避免型号相关硬编码），判据：把实现里的 `{200,500}` 换成"解析 info 字符串"后，`odr:=100` 仍退出码 2 且消息引用 info 原文 |
| Q4 | `mode:=rect width:=1088 height:=1280 fps:=30` 实测只能约 20 fps（E10）：节点应 WARN、拒绝，还是什么也不做？ | 跑 60 s 给出 `ros2 topic hz` 的实测值；若 < 25 Hz，则答案必须是"WARN（不退出）"，且日志文案进 R-51 的键名表；同时确认 `camera_fps=` 日志值与该实测一致（±5%） |
| Q5 | `publish_tf` 默认 `true`（本文采纳实现，偏离 `11:52`）在既有栈中是否会引起 frame 冲突？ | `ros2 topic echo /tf_static --once` 与 `ros2 run tf2_ros tf2_monitor` 的输出：若 `camera_left`/`camera_right`/`imu_link` 各只有一个父，则保持默认 `true`；若与 `mipi_cam`/`robot_state_publisher` 的 frame 冲突，则改为默认 `false` 并写进 README |
| Q6 | `use_sim_time:=true` 时 rclcpp 的 `now()` 在无 `/clock` 时是否停 0？ | 一次实跑：`ros2 launch … --ros-args -p use_sim_time:=true` 的 node 日志与 `ros2 topic echo /image_combine_raw --field header.stamp --once`。若 stamp 恒为 0 ⇒ R-39 必须写"FATAL + 退出码 2"；若 stamp 有效 ⇒ 打 WARN 即可 |
| Q7 | `R-21` 的泄漏阈值（5 min 内 `VmRSS` 增量 < 8 MB）是否足够敏感/是否会误报？ | 两次实测：正常实现一次、故意删除一处 `free()` 一次。答案必须给出两组数字（期望：正常 < 8 MB、故意泄漏 ≫ 8 MB）；若正常实现的噪声 > 8 MB，则阈值改为实测噪声的 3 倍 |
| Q8 | 是否需要 `stamp_offset_ns`（强制偏移）调试参数以支持可复现测试？ | 答案必须给出一个**具体**用例：若无法写出"没有该参数就做不了"的测试，则不提供（本文立场是**不提供**，参数表已收敛为 13 个） |
| Q9 | 是否需要把 EEPROM 的 IMU 噪声密度（`gs130_get_imu_intrinsics()` 的 `accel_noise`/`gyro_noise`，单位 `m/s²/√Hz`、`rad/s/√Hz`）填进 `sensor_msgs/Imu` 的协方差？ | 答案必须说明单位换算规则与下游消费证据（例如 `robot_localization` 日志是否因协方差全 0 而拒绝融合）。若给不出换算规则，则保持全 0（R-24） |
| Q10 | `stereo_layout:=none` 时两目 stamp 的差值上界是 `1e9/fps/2`（16.7 ms，源码 `gs130.cpp:228,316`）——板端是否可复现？ | 采 300 对帧，给 `|Δstamp|` 的 max 与 p99；若 max > 0.5/fps，则 R-37 的阈值改为实测上界，并把"半帧对齐"的说明写进 README |

---

## 附录：本文相对上一版契约的**修正**（针对 11/41 的明确错误）

| 编号 | 被修正的原文 | 本文修正 | 依据 |
|---|---|---|---|
| C1 | `11:142`「同一帧的左目与右目时间戳相同；stitched 使用左目时间戳」 | 两目各自独立、差 ≤ 16.7 ms；stitched 用 **FSYNC 绑定目 = 右目** | `gs130.cpp:228,332-335`；`gs130_define.h:33` |
| C2 | `11:375-376` `width ∈ [16,1088]` | **无上限**（1920x1080 实测可用，已超过传感器 1088 宽）；支持性由 SDK 裁决 | E10；`vse.c:14-23` |
| C3 | `11:285-286` camera_info 的 `R`/`P` 全 0 | `R`=单位阵、`P=[K\|0]` | `32:234`（全 0 会让 stereonet `is_valid()` 为 false 并静默无输出） |
| C4 | `11:303` `D` 截断为 5 元 `plumb_bob` | PINHOLE → `rational_polynomial` + 8 系数（不丢系数） | ROS 原生支持 8 元模型 |
| C5 | `11:258-261` accel/gyro 协方差 `[0]=-1` | 全 0（有测量值、方差未知）；仅 `orientation_covariance[0]=-1` | ROS 协方差语义；Python 版实测值 |
| C6 | `11:52` v0.1.0 不发布 TF | 发布 `/tf_static`（`publish_tf:=true`），方向规则见 R-33 | `41:44`（用户复评 UX-3/UX-4）；`25:69` 实测基线 |
| C7 | `11:415`/`11:570` `device:=GS130W` 会正常跑且"无 IMU" | 本硬件上 `GS130W` 直接 `GS130_NOT_FOUND` 退出码 1；"无 IMU 继续运行"路径**未经任何实测**，标为未验证 | E10 |
| C8 | `41:42` 收敛为 13 个参数但未区分"开关参数" | 话题集合由 `stereo_layout` 唯一决定，进一步删除 `publish_combine`/`publish_per_eye`/`publish_camera_info` 的隐含开关语义 | R-11 |
