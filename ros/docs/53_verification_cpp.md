# 53 C++ 节点验证方案（对抗式评审目标）

| 项 | 值 |
|---|---|
| 文档编号 | gs130_ros / 53_verification_cpp |
| 被测对象 | `ros/gs130_ros` C++ 重写版（`ament_cmake`，链接 `libgs130` C API），替换已删除的 Python 节点 |
| 编写人 | TEAM-AUTHORS/C1（验证负责人） |
| 依据 | `core/include/gs130.h`（C API 与所有权语义）、`00_verified_platform_facts.md`（E1–E10，最高权威）、`50_cpp_design.md` §3.2/§3.3/§3.4、`20_test_plan.md`/`25_test_report.md`、`ros/test/test_calibration.py`、`ros/test/capture_frame.py` |
| 架构文档状态 | **`ros/docs/52_architecture_cpp.md` 在本文编写时不存在**。本文的符号名以**已落地的实现**为准（`include/gs130_ros/conversions.hpp`、`include/gs130_ros/preset.h`、`src/conversions.cpp`、`src/preset.c`、`src/camera_node.cpp`、`test/test_conversions.cpp`，编写时均为未提交工作区文件）。若架构落地后符号改名，**只允许改符号名，不允许改判据**（见 §6.4） |
| 编写时的实现状态（只读观察，未运行） | `test/test_conversions.cpp` 有 **10 个 gtest 用例**，只覆盖「纯函数」的一个子集：`distortion_model`(3)、`to_camera_info`(1)、`to_stamp`(2)、`to_quaternion`(3)、`to_transform`(1)。**没有任何** NV12 几何、参数校验、IMU 消息构造、QoS profile、帧缓冲所有权的主机测试；`validate()` 是 `camera_node.cpp` 的**成员函数**（依赖 `declare_parameter` 与 `fatal()`），不是纯函数，主机不可直接调用。→ TC-03 把「现状基线」与「必须补齐」分开判，**不允许**用"10 个用例全绿"充当本方案的 L1 证据 |
| 执行者 | 本文作者未连接板卡、未运行相机。所有"期望值"来自冻结事实、头文件语义与 SDK 源码；标注为**提案**的数值见 §6.2 |
| 判据来源纪律 | 期望值冲突时优先级：`00_verified_platform_facts.md` > `gs130.h` > 本文。**不得**为了通过而放宽判据；放宽必须附实测证据 |

---

## 1. 无硬件可验证 / 不可验证的边界

### 1.1 可在主机（开发机）验证 —— 每项都指定被测函数

**符号名现状（编写时实读）**：`conversions.hpp` 已声明 `to_stamp(int64_t)`、`distortion_model(const gs130_camera_intrinsics_t&, std::vector<double>&)`、`to_camera_info(const gs130_camera_intrinsics_t&, uint32_t, uint32_t, const std::string&)`、`to_quaternion(const double[9], double&,double&,double&,double&)`、`to_transform(const double[9], const double[3], const std::string&, const std::string&)`。下表中标注 **[missing]** 的是本方案要求新增（或抽取为纯函数）的主机可测目标 —— 它们是 §3 中 D2/参数校验之所以能"响亮失败"的前提。

| H | 被测项 | 被测函数 / 目标（**实读名**优先） | 主机可行性 | 主机可证明什么 | 主机**不能**证明什么 |
|---|---|---|---|---|---|
| H1 | 纯函数：NV12 消息几何 | **`conversions::nv12_geometry(uint32_t w, uint32_t h, gs130_stereo_layout_t, Nv12Geometry*)` [missing]** —— 现状是把几何直接写在 `publish_frame()` 里，**没有**独立函数可测 | 可（只需 `<gs130.h>` 枚举，不需要 `libgs130.so`） | `height == SDK 真实高`（来自 `gs130_image_nv12_t.height`，**不是**从 `data_size` 反推）、`step == width`、`len == w*h*3/2`、`left_right` 只翻倍 width、`top_bottom` 只翻倍 height | 真实帧的 `image.width/height` 与这里一致（TC-07 在板端证） |
| H2 | 纯函数：时间戳换算 | `conversions::to_stamp(int64_t nanoseconds)`（**已存在**，语义 = `ns → builtin_interfaces::msg::Time`）+ **[missing]** 独立的 `monotonic_to_ros_ns(uint64_t device_ts_ns, int64_t offset_ns)` | 可 | `sec = ns/1e9`、`nanosec = ns%1e9`、负数/零/进位（`1e9-1`、`1e9`、`-1`）不出错；`offset=0` 时等于原值 | offset 是否真的来自 `RCL_SYSTEM_TIME`（TC-21） |
| H3 | 纯函数：CameraInfo 组装 | `conversions::to_camera_info(const gs130_camera_intrinsics_t&, uint32_t w, uint32_t h, const std::string& frame_id)`（**已存在**） | 可（手填 `gs130_*` 结构体，无需设备） | `K/D/R/P` 逐元素、`width/height` 用**单目**尺寸、`frame_id`、`binning`/`roi` 清零、`R` 是否为单位阵 | EEPROM 里的真实数值（TC-14） |
| H4 | 纯函数：畸变模型映射 | `conversions::distortion_model(const gs130_camera_intrinsics_t&, std::vector<double>& coefficients)`（**已存在**）；**注**：`mode:=rect` 覆盖为 `plumb_bob`+5 个 0、以及 `camera_info_distortion_model` 覆盖（若保留该参数）应在**同一函数或调用点**可测 | 可 | `FISHEYE`→`equidistant`+4；`PINHOLE`→`plumb_bob`+5；全零→`plumb_bob`+5 个 0；`rect`→`plumb_bob`+5 个 0 | 真机走的是哪条分支（TC-14/TC-11） |
| H5 | 纯函数：旋转矩阵→四元数 | `conversions::to_quaternion(const double rotation[9], double& x, double& y, double& z, double& w)`（**已存在**） | 可 | `trace>0` 与三条 `trace<=0` 分支（含 180°、近 180°、退化 `R=I` 与 `R=-I`）都能反算回 `R`；`|q|=1` | 数值与 EEPROM 外参一致（TC-16） |
| H6 | 纯函数：静态 TF 组装 | `conversions::to_transform(const double rotation[9], const double translation[3], const std::string& parent, const std::string& child)`（**已存在**） | 可 | `parent/child` 方向、`T` 逐元素、四元数一致性、**不**做 `tf2` 广播 | SDK `relative_R/T` 的真实方向语义（TC-16 用 SDK 交叉验证） |
| H7 | 纯函数：IMU 消息构造 | **`conversions::to_imu_msg(const gs130_imu_packet_t&, const std::string& frame_id, int64_t offset_ns)` [missing]** —— 现状在 `imu_timer()` 内联构造 | 可 | `accel→linear_acceleration`、`gyro→angular_velocity` **单位不做任何换算**、`orientation=(0,0,0,1)`、三个 `covariance[0]=-1.0`（长度各 9）、`temp`/`is_fsync` **不被发布** | 数值单位正确性（TC-18 用重力判据证） |
| H8 | 纯函数：参数校验 | **`validate_parameters(const NodeParams&)` [missing]** —— 现状 `camera_node.cpp::validate()` 是成员函数，依赖 `declare_parameter`/`fatal()`，主机不可测 | 可（零 ROS 依赖） | 每个非法输入的**拒绝理由文本**与"合法组合不被拒绝"；`kUnsupportedSizes`（864x480/1024x600）、`raw` 必须 1088x1280、`raw` 与拼接互斥、奇数、`fps∈[1,33]`、`odr`、非法 mode/layout **全部**在**不接触设备**的分支里 | 真实退出码与"未触碰相机"（TC-25） |
| H9 | QoS 常量与匹配语义 | **`qos_profile_for_image()` / `qos_profile_for_camera_info()` [missing]**（现状 `rclcpp::QoS(rclcpp::KeepLast(1)).reliable()` 内联在 `start_streaming()`）+ **[missing]** IMU publisher 的 `imu_qos_depth` 参数 | 可（需要 rclcpp 头 + `rclcpp::QoS` 值语义，**不需要** DDS 发现） | 代码里声明的 QoS 是 `RELIABLE`/`KEEP_LAST`/`depth`/`VOLATILE`（`camera_info` 为 `TRANSIENT_LOCAL`） | 板端 DDS 实际 offer 的 QoS 与订阅端匹配（TC-05，**唯一决定性证据**） |
| H10 | 构建 | `colcon build --packages-select gs130_ros`；`CMakeLists.txt` 的 `ament_target_dependencies` + `target_link_libraries(gs130_node gs130)`；`src/preset.c` 作为 C 源参与编译 | 可交叉编译（`--cmake-args -DCMAKE_TOOLCHAIN_FILE=... aarch64`）或仅在板端编 | 编译期无错（含 C/C++ 混合编译）、`#include <gs130.h>` 可解析、链接行含 `-lgs130`、无 `undefined reference` | 运行期真的加载到 `libgs130.so.0.0.1`（TC-01 用 `ldd` 证） |
| H11 | 帧缓冲 RAII 所有权语义 | **`FrameGuard` / `nv12_frame_ptr` [missing]**（现状 `publish_frame()` 之后在 `image_timer()` 里手工 `free(stitched.data)` / `free(left.data)` / `free(right.data)`，共 2 处 **3 个 free 点**）在**桩设备**下每帧恰好 `free` 一次 | 可（ASan + 计数型桩） | 单次 free、move 后不自留悬垂、异常路径也 free；拼接模式下**只** free 返回的那一个指针（SDK 的 `frame[1].data == nullptr`） | SDK FIFO 丢弃路径也在 free（TC-38） |


**主机链路的具体命令（提案，文件名随架构落地调整）：**

```bash
# H1..H9：纯函数，主机即可
cd $REPO/ros/gs130_ros && rm -rf build_host && cmake -S . -B build_host -DBUILD_TESTING=ON \
  -DGS130_HOST_TESTS=ON > /tmp/host_cmake.log 2>&1 \
  && cmake --build build_host -j"$(nproc)" > /tmp/host_build.log 2>&1 \
  && ctest --test-dir build_host --output-on-failure | tee /tmp/host_ctest.log
# H10：交叉编译，证明 -lgs130 在链接行里
cmake --build build_host --target gs130_node --verbose 2>&1 | grep -E "gs130|rclcpp" | tee /tmp/host_link.log
# H11：桩 + ASan（见 §4.2）
```

### 1.2 主机不可验证（必须在板端）

| N | 项目 | 为什么必须板端 |
|---|---|---|
| N1 | `gs130_init()` 的 I2C 探测、ISP/VSE/GDC/VIN 配置结果 | 依赖 GS130WI 实际连接与内核驱动 |
| N2 | 真实帧率（E10 的分辨率矩阵、1088x1280 约 20 fps） | 硬件流水线决定 |
| N3 | IMU 数值与单位、`odr`、FSYNC 突发特性 | 依赖 ICM-42688-P 与 SDK 时间戳跟踪器 |
| N4 | EEPROM 标定数值（K/D/外参基线） | 依赖样机 EEPROM |
| N5 | E8 静默抢流（第二个打开者） | 平台独占行为，无法桩化 |
| N6 | 与 `hobot_codec` 的 QoS 兼容性（E1）、真实 30 fps 端到端 | 依赖既有 D-Robotics 节点 |
| N7 | SIGINT 释放、端口回收、`libgs130` 卸载 | 进程/端口/驱动层 |
| N8 | 内存正确性（真实 malloc 路径） | 主机桩无法覆盖 SDK 的 FIFO 丢弃与析构路径 |
| N9 | 温度、CPU 占用、长时间稳定性 | 板卡运行时特性 |

**本文所有 TC 均标注级别：`H`=主机、`L2`=板端无相机（参数/构建）、`L3`=板端相机、`L4`=端到端 web、`L5`=可靠性/资源。**

---

## 2. 测试用例表（TC-01 … TC-43）

通用前置（每条 TC 都适用，下文不再重复）：

| 编号 | 前置 |
|---|---|
| P1 | `export REPO=/home/leaf-jammy/rdkx5_work/gs130_sdk; export WS=~/gs130_ros_ws; export EVID=$REPO/ros/docs/evidence/$(date +%Y%m%d_%H%M%S); mkdir -p $EVID/img` |
| P2 | `source /opt/tros/humble/setup.bash && source $WS/install/setup.bash`，`export ROS_DOMAIN_ID=42`（隔离同网段其它实验） |
| P3 | 无相机占用：`pgrep -af "mipi_cam|gs130_node|gs130-run"` 为空 |
| P4 | `export NODE="timeout 300 ros2 run gs130_ros gs130_node --ros-args"`（`timeout` 保证任一用例不会留下永久进程） |
| P5 | 每条命令 `2>&1 | tee $EVID/TC-XX.log`，末尾 `echo rc=$? | tee -a`；时间量一律 `date +%s.%N` 前后夹取 |
| P6 | 话题断言脚本统一用 `ros/test/probe.py`（见 §2.10）：`timeout 40 python3 $REPO/ros/test/probe.py --topic <t> --seconds 30 --qos <profile> --json $EVID/TC-XX.json`，输出 `rate_hz / std / max_gap_ms / count / jumps / height / width / step / data_len / stamp_domain`，**判据全部落在 JSON 字段上**，不靠肉眼读 `ros2 topic hz` |

### 2.1 构建、链接与单元测试

| TC | 目标 | 级别 | 前置 | 命令（P2 之后） | 期望可观测 | 通过判据（数字/文本） | 通过**不**证明 |
|---|---|---|---|---|---|---|---|
| TC-01 | 构建 + 真的链接到 `libgs130` | L2 | P1,P2，已 `ln -sfn $REPO/ros/gs130_ros $WS/src/gs130_ros` | `cd $WS && colcon build --packages-select gs130_ros 2>&1 \| tee $EVID/TC-01-build.log; echo rc=$? \| tee -a $EVID/TC-01-build.log`；`ldd $WS/install/gs130_ros/lib/gs130_ros/gs130_node \| tee $EVID/TC-01-ldd.log`；`nm -D $WS/install/gs130_ros/lib/gs130_ros/gs130_node \| grep -c gs130_ \| tee -a $EVID/TC-01-ldd.log`；`ros2 pkg executables gs130_ros \| tee -a $EVID/TC-01-ldd.log` | 构建成功；`ldd` 含 `libgs130.so.0 => /lib/aarch64-linux-gnu/libgs130.so.0`；`nm -D` 计数 ≥ 8（`gs130_create/init/start/get_stereo_nv12_frame/get_imu_packet/get_calibration/get_camera_intrinsics/get_relative_R/get_relative_T/deinit/destroy/stop`）；`ros2 pkg executables` 含 `gs130_ros: gs130_node` | `rc=0`；`ldd` 行**必须**出现 `libgs130.so.0`（出现 `not found` 即 FAIL）；`nm -D` 计数 ≥ 8 且实际出现在输出里的符号名包含 `gs130_get_stereo_nv12_frame`；`ls $WS/install/gs130_ros/share/gs130_ros/launch/` 含两个 launch | 运行期不崩、动态加载顺序正确、SDK 版本兼容（TC-02/TC-03 补） |
| TC-02 | 库版本与平台自检 | L2 | P1,P2 | `$WS/install/gs130_ros/lib/gs130_ros/gs130_node --version`（若未实现该开关，则 `strings $(ldd ... \| awk '/libgs130/{print $3}') \| grep 0.0.1` 替代）；`nm -D /lib/aarch64-linux-gnu/libgs130.so.0 \| grep -E "gs130_version\|gs130_platform"` | 打印/可见 `0.0.1`；库导出 `gs130_version`、`gs130_platform` | 版本字符串逐字 `0.0.1`；两个符号都在 `nm -D` 输出中 | 头文件与 .so 版本一致（需 TC-01 的能跑通才能间接说明） |
| TC-03 | 主机纯函数 gtest **①现状基线 + ②必须补齐** | H | P1 | `cd $REPO/ros/gs130_ros && cmake -S . -B build_host -DGS130_HOST_TESTS=ON && cmake --build build_host -j"$(nproc)" && ctest --test-dir build_host --output-on-failure --verbose \| tee $EVID/TC-03-host.log` | `test_conversions` 二进制运行并列出用例名 | **① 现状基线（必须不退步）**：`100% tests passed` 且测试数 `≥ 10`（编写时实测 10 个：DistortionModel×3、CameraInfo×1、Stamp×2、Quaternion×3、Transform×1）。**② 必须补齐（缺任一即为 L1 门禁 FAIL，不是 WARN）**：H1 `nv12_geometry`、H7 `to_imu_msg`、H8 `validate_parameters`、H9 QoS profile 四个目标各自 ≥ 3 个 case，且 `nv12_geometry` 的用例必须**至少一个**断言"传入 `data_size = w*h*3` 而 `height` 输出仍等于入参 height"（即**证明几何不依赖缓冲长度**）。总用例数 `≥ 24` | 与真机数值一致、板端分支真的被走到、**参数校验在板端真的执行**（TC-25）；也**不能**证明"现有 10 个用例覆盖了关键契约" |
| TC-04 | 主机 ASan 桩测试（RAII 所有权） | H | P1 | 见 §4.2 逐字命令 | ASan 静默退出 | `ASAN_OPTIONS` 下 `rc=0`，stderr 中**无** `ERROR: AddressSanitizer`、无 `LeakSanitizer: detected memory leaks`；桩计数断言"每帧 free 恰好 1 次"通过 | 真实 `libgs130` 的 free 行为（TC-38） |

### 2.2 话题、类型、QoS —— E1 的决定性验证

| TC | 目标 | 级别 | 前置 | 命令 | 期望可观测 | 通过判据 | 通过**不**证明 |
|---|---|---|---|---|---|---|---|
| TC-05 | **QoS 真的匹配 codec（不只是"话题存在"）** | L3+L4 | P1–P4；`hobot_codec_republish` 可用 | ① `$NODE -p mode:=resize -p width:=640 -p height:=480 & sleep 8`；② `ros2 topic info /image_combine_raw --verbose \| tee $EVID/TC-05-qos.log`；③ 另起终端：`ros2 launch gs130_ros gs130_web.launch.py codec_channel:=0 2>&1 \| tee $EVID/TC-05-codec.log` 跑 30 s；④ `grep -c "offering incompatible QoS" $EVID/TC-05-codec.log`；⑤ `timeout 20 ros2 topic info /image_combine_jpeg --verbose \| tee -a $EVID/TC-05-qos.log` | ② 中 Publisher 段的 `Reliability: RELIABLE`；④ 计数 `0`；⑤ `/image_combine_jpeg` 存在且 `hobot_codec` 是**订阅者**（`Node name: hobot_codec_encoder_*`），其 `Reliability: RELIABLE` | **四个条件全满足**：Publisher 行逐字 `Reliability: RELIABLE`；`grep -c` 输出 `0`；jpeg 话题 `Node name:` 含 `hobot_codec`；`ros2 topic hz /image_combine_jpeg` 平均值 ≥ 0.9×fps。**反向证据**：若 `grep -c` ≥ 1 且 `ros2 topic hz /image_combine_jpeg` 无输出（codec 一条都没收到），判 FAIL 并立案 —— 这正是 E1 的回归 | 只跑"我自己的 subscriber 收到帧"**不构成 QoS 证据**：BEST_EFFORT 发布者与 BEST_EFFORT 订阅者是兼容的，`ros2 topic echo` 默认也常是 BEST_EFFORT。**必须**靠 `--verbose` 的 offer 值 + codec 端到端 |
| TC-06 | 话题集合、类型、`frame_id` | L3 | TC-05 的节点运行中 | `timeout 20 ros2 topic list \| tee $EVID/TC-06-list.log`；`timeout 20 ros2 topic info /imu/data --verbose \| tee $EVID/TC-06-imu.log`；`timeout 20 ros2 topic info /image_left/camera_info --verbose \| tee $EVID/TC-06-info.log` | `/image_combine_raw`（`sensor_msgs/msg/Image`）、`/imu/data`（`sensor_msgs/msg/Imu`）、`/image_left/camera_info`、`/image_right/camera_info`（`CameraInfo`）；`/tf_static` 存在（`publish_tf:=true` 默认）；默认**无** `/image_left_raw`、`/image_right_raw` | 四个必需话题存在且类型逐字匹配；`/image_left/camera_info` 为 `RELIABLE` + `TRANSIENT_LOCAL`；`/imu/data` depth 与 `imu_fifo` 一致（在证据中记录实测 depth 值，判据是"与 `ros2 param get /gs130_node imu_qos_depth` 相同"）；默认关闭的两个 per-eye 话题**不存在**；`probe.py` 读到的 `frame_id`：`/image_combine_raw`=`camera_left`、`/imu/data`=`imu_link` | 发布者没把消息丢掉（TC-05/TC-07 补）、话题内容正确 |
| TC-07 | **E2 字段契约（决定性）** | L3 | TC-05 节点运行中 | `timeout 40 python3 $REPO/ros/test/capture_frame.py 2>&1 \| tee $EVID/TC-07.log`；另加断言脚本 `ros/test/check_contract.py --topic /image_combine_raw --real-width 640 --real-height 480 --json $EVID/TC-07.json` | `encoding=nv12 width=1280 height=480 step=1280 len=921600`；`camera_info 640x480` | **全部严格相等**：`encoding=="nv12"`、`width==2*单目宽`、`step==width`、`len(data)==width*height*3/2`、`frame_id` 非空；**且 `height` 必须等于 SDK 报告的真实高**（`height == sdk_output_height`，由节点启动日志同一行打印的 `geometry WxH` 提供，差必须为 0）。**禁止**把 `height == len(data)*2/(3*width)` 当作判据：打包高 `H*3/2 = 720` 时该式同样成立（`1280*720*3/2 == 1382400 == len(data)`），**长度恒等式对 E2 缺陷完全没有鉴别力**；唯一有鉴别力的比较对象是 SDK 的 `gs130_image_nv12_t.height` | TLS 传输正确、codec 不崩（TC-32 补） |
| TC-08 | 帧内容真实性 + NV12 平面/Orientation 检查 | L3 | TC-05 节点运行中 | `timeout 60 python3 $REPO/ros/test/capture_frame.py`（存 PNG）；`timeout 60 python3 $REPO/ros/test/check_planes.py --topic /image_combine_raw --json $EVID/TC-08.json` | PNG 为真实办公场景，左右半场景相同、有视差位移；平面检查：`Y` 平面 `std>20`、`UV` 平面 `std<Y`、`U/V` 均值落在 `[60,200]`、上半（Y）与下半（UV）无整段恒定填充值 | 20 张 PNG 存盘且 `480×640×3`；`10<mean<245` 且 `std>5`；平面检查四项全过；左右目平均绝对像素差 `> 2.0`（E-事实里的视差判据）；**额外**：对 PNG 顶部 32 行与底部 32 行做 `cv2.Canny` 边缘密度比，判据 `edge_top/edge_bottom ∈ [0.25, 4.0]`（UV 被误当图像会表现为上半极平滑） | 标定正确性、codec 质量 |
| TC-09 | 逐帧成本与丢帧（**性能主判据，板端可自动断言**） | L3 | TC-05 节点运行中，`publish_status:=true` | `timeout 70 ros2 topic echo /gs130/status --once \| tee $EVID/TC-09-status.log`（节点内部逐帧打点，见 §5.1） | `/gs130/status` 的 `values` 含 `frame_cost_ms_median / frame_cost_ms_p99 / frame_cost_ms_max / frames / dropped / fifo_depth_max / imu_queue_max` | 640x480@30：`frame_cost_ms_median ≤ 4.0`、`p99 ≤ 10.0`、`max ≤ 33.3`；`dropped == 0`；`fifo_depth_max ≤ 1`。**判据出处**：`camera_fifo.depth = 4`（`python/gs130/_config.py` `preset()` 实测值）→ `fifo_depth_max < 4` 意味着队列从未被填满，即"节点吞吐 ≥ 相机吞吐"的**直接**证明 | 1920x1080 下的同样结论（TC-10）；`publish()` 返回后的内存安全（TC-38/TC-40） |

### 2.3 帧率、分辨率与模式

| TC | 目标 | 级别 | 前置 | 命令 | 期望可观测 | 通过判据 | 通过**不**证明 |
|---|---|---|---|---|---|---|---|
| TC-10 | 多分辨率帧率（含 E10 拒绝项） | L3 | P1–P4，每档独立启动，`sleep 8` 后测 30 s | 对 `(320,240) (640,480) (1280,720) (1920,1080)`：`$NODE -p mode:=resize -p width:=W -p height:=H -p fps:=30` + `probe.py --topic /image_combine_raw --seconds 30`；对 `(864,480) (1024,600)`：`$NODE ... -p width:=864 -p height:=480 2>&1 \| tee $EVID/TC-10-864.log; echo rc=$?` | 四档 `rate_hz`；两档报错退出 | 四档 `rate_hz ∈ [0.90×30, 1.05×30] = [27.0, 31.5]`（**提案**，见 §6.2）；`std < 3.0 ms`；`max_gap_ms < 200`；`jumps == 0`（stamp 不回退）。两档 `rc=2` 且日志含逐字 `UNSUPPORTED`（或节点自己的 `invalid parameter` + 原因），**且 `sleep 3` 后 `pgrep -af gs130_node` 为空**。1088x1280（`mode:=rect`）单列：`rate_hz ≥ 18.0` 且在证据中登记实测值（E10 实测约 20 fps，**不得**当作 FAIL） | 图像内容、其它 fps、`fps>30` 的行为（`fps ≤ 33` 另测） |
| TC-11 | `mode:=rect` / `mode:=raw` 的合法路径 | L3 | 同上 | `$NODE -p mode:=rect -p width:=1088 -p height:=1280` + `probe.py`；`$NODE -p mode:=raw -p width:=1088 -p height:=1280 -p stereo_layout:=none` + `probe.py` | rect 出图、`distortion_model=plumb_bob`、`D=[0,0,0,0,0]`；raw 单目 `width=1088 height=1280 step=1088 len=2088960` | rect：`rate_hz ≥ 18.0` 且 `camera_info` 为 `plumb_bob` + 5 个 0；raw：三个尺寸字段严格等于 `1088/1280/1088/2088960`，`rate_hz > 0`，10 s 内 `max_gap_ms < 500` | 标定精度、rect 的几何正确性（需人工目视直线变直） |
| TC-12 | 拼接布局切换生效 | L3 | P1–P4 | `stereo_layout:=left_right` → `probe.py`；`stereo_layout:=top_bottom` → `probe.py` + `capture_frame.py` | `left_right`：`width=1280 height=480`；`top_bottom`：`width=640 height=960`；两者 `len == w*h*3/2` | 两种布局的 `width/height/len` 严格匹配；`top_bottom` 的 PNG 上/下两半为同一场景（目视 + `eyes differ > 2.0` 复用 E9 判据）；`stereo_layout:=bogus` → `rc=2` | 视差量、`right_left`/`bottom_top` 的目视正确性（需人工检查或跳帧检查） |
| TC-13 | `stereo_layout:=none` 双话题成对 | L3 | P1–P4 | `$NODE -p stereo_layout:=none -p publish_per_eye:=true`；`probe.py --topic /image_left_raw`；`probe.py --topic /image_right_raw`；`python3 $REPO/ros/test/check_pair.py --json $EVID/TC-13.json` | 两话题都存在、各自 `width=640 height=480`；成对帧的 `header.stamp` 完全相同 | 两话题 `rate_hz` 都 ≥ 27.0；30 对中 `delta_stamp_ns == 0` 的比例 = 30/30（SDK 同一帧对共用 `ts[fi]`/各自 `ts`，见 `core/src/gs130.cpp` 的 FIFO 推送结构——若架构选择"两目各自取时间戳"，判据放宽为 `|ts_left-ts_right| ≤ 1/fps/2`，**必须在架构文档里写明采用哪一种**） | 两话题的图像内容是左右目而非同目（`check_pair.py` 必须做 `mean abs diff > 2.0` 的左右目差异检查） |

### 2.4 标定与 TF

| TC | 目标 | 级别 | 前置 | 命令 | 期望可观测 | 通过判据 | 通过**不**证明 |
|---|---|---|---|---|---|---|---|
| TC-14 | `CameraInfo` 对 EEPROM 真值 | L3 | P1–P4，节点运行中 | `timeout 20 ros2 topic echo /image_left/camera_info --once \| tee $EVID/TC-14-left.log`（右侧同）；`timeout 60 python3 $REPO/ros/test/sdk_truth.py --json $EVID/TC-14-sdk.json`（独立进程 `gs130_create/init/get_camera_intrinsics` 后立即 `deinit/destroy`，**不** start，不抢流） | 左侧 `K fx=386.85 fy=387.12 cx=304.62 cy=245.06`，`model=equidistant`，`D` 4 元；`width=640 height=480` | 与 `sdk_truth.py` 导出的 SDK 原值**逐元素差 ≤ 1e-9**（`K` 全部 9 元、`D` 前 4 位）；`abs(fx-386.85)≤0.5`、`abs(fy-387.12)≤0.5`、`abs(cx-304.62)≤0.5`、`abs(cy-245.06)≤0.5`（真值出处：`25_test_report.md` §3.3 实测）；`width/height` 为**单目**尺寸（不是 `2W`，不是 `H*3/2`）；`binning_x/y==0`；`roi` 全 0；`do_rectify==false` | 标定本身准确（只证"SDK 报什么，ROS 报什么"）；`P` 是否该由 `stereo_rectify` 生成 |
| TC-15 | 内参与输出分辨率同步缩放 | L3 | 两档分辨率各启动一次 | 对 `640x480` 与 `320x240` 各取 `camera_info` + `sdk_truth.py` | 两档的内参与 SDK 各自分辨率的输出一致 | 两档都满足 TC-14 的 `1e-9` 判据；`fx_320/fx_640 ∈ [0.45,0.55]`；**若 SDK 在 320x240 下返回未缩放的 `K`，本 TC 判为"记录实测 + 立案"，不得静默通过** | 缩放是否由 ISP 完成 |
| TC-16 | 静态 TF 与 SDK 外参一致 | L3 | 节点运行中（`publish_tf:=true`） | `timeout 20 ros2 topic echo /tf_static --once \| tee $EVID/TC-16-tf.log`；`timeout 60 python3 $REPO/ros/test/sdk_truth.py --extrinsics --json $EVID/TC-16-sdk.json` | `camera_left→camera_right` 的 `translation=[0.070,0.000,-0.002]`、`rotation` 为单位四元数；`camera_left→imu_link` 存在 | TF 的 `translation` 与 SDK `camera_right_T`（换算到同一 parent/child 方向后）逐元素差 ≤ 1e-6；**基线模长** `|T| ∈ [0.0695, 0.0711]`（中心值 0.070316 m，出处 `25_test_report.md`）；旋转矩阵正交性 `max|R^T R - I| < 1e-6`；`frame_id`/`child_frame_id` 逐字匹配 `frame_id_camera`/`frame_id_imu` 参数 | TF 树的连通性（用 `ros2 run tf2_ros tf2_echo camera_left camera_right` 输出 `Translation` 且无 `Invalid frame ID` 才补上这一条）；外参语义在 rect 模式下变化（E-事实/`gs130.h` 注：rect 会变虚拟平行系）——**rect 模式下 TF 应被禁用或显式告警**，本 TC 只在 `resize` 下判 PASS |
| TC-17 | 无 EEPROM 时的降级契约 | L3 | 需无标定样机；无样机时**必须**在证据里写"未执行" | 拔 EEPROM 或使用无标定样机后启动默认参数 | `WARN`：标定不可用 → 不发布 `camera_info`，相机继续出图；`mode:=rect` 时 `rc=1` | 默认参数下进程存活 ≥ 30 s、`/image_combine_raw` 有数据、两个 `camera_info` 话题**不存在**（不得发布全零 `CameraInfo`）；`rect` 下 `rc=1` 且日志含 `PARAM_ERROR`。**无样机时本 TC 记为 BLOCKED，不得记为 PASS** | rect 在有 EEPROM 时的行为（TC-11） |

### 2.5 IMU

| TC | 目标 | 级别 | 前置 | 命令 | 期望可观测 | 通过判据 | 通过**不**证明 |
|---|---|---|---|---|---|---|---|
| TC-18 | IMU 速率、单位、静止量级 | L3 | 板卡水平静置（用水平仪或已知倾角并记录），节点运行中 | `timeout 40 python3 $REPO/ros/test/probe_imu.py --seconds 30 --json $EVID/TC-18.json` | `count / rate_hz / accel_mean / accel_std / gyro_mean / accel_norm` | `abs(accel_norm - 9.81) ≤ 0.30 m/s²`（出处：SDK `accel_scale = fsr_g*9.80665/32768`，静止应等于 1g；`20_test_plan.md` 用 9.8±0.5，本文收紧到 ±0.30 **提案**）；`abs(gyro_mean) < 0.05 rad/s`；`accel_std < 0.30`；`sensor_msgs/Imu` 的 `linear_acceleration_covariance[0] == -1.0`、`angular_velocity_covariance[0] == -1.0`、`orientation_covariance[0] == -1.0`（各长度 9）；`orientation == (0,0,0,1)`；`frame_id == "imu_link"`；**不存在**温度话题、`is_fsync` 话题 | IMU 标定精度（`accel_misalign`/`bias` 未应用）、姿态解算 |
| TC-19 | `odr` 生效（200 与 500） | L3 | P1–P4，两次独立启动 | `odr:=200` → `probe_imu.py --seconds 30 --json ...-200.json`；`odr:=500` → 同法 | 两次 `rate_hz` | 各自 `rate_hz ∈ [0.85×odr, 1.15×odr]`（**提案**；`25_test_report.md` 实测 202.9 Hz @ odr=200）；比值 `rate_500/rate_200 ∈ [2.0, 3.0]`；`odr:=100/0/250` → `rc=2` | 突发模式下的瞬时抖动、FSYNC 与图像的对齐质量 |
| TC-20 | 不丢包、不重排、突发结构可解释 | L3 | 节点运行中，`publish_status:=true` | `probe_imu.py --seconds 60 --json $EVID/TC-20.json`；`ros2 topic echo /gs130/status --once` | 60 s 内 IMU 包计数与节点 `imu` 计数；stamp 序列 | 60 s 内实测总数 `N` 满足 `N ≥ 0.85 × odr × 60`（id=200 → ≥ 10200）；`jumps == 0`（严格递增，允许相等但统计为 `equal`，`equal ≤ 1%`）；`max_gap_ms ≤ 200`（FSYNC 突发；若 SDK 侧 burst 更长，实测值入证据并**修改本判据需附 SDK 源码引用**）；`fifo_depth_max ≤ 1024×0.5`（`imu_fifo.depth=1024`，实测值） | 数值正确性（TC-18 补）、`is_fsync` 语义（非目标） |
| TC-21 | 时间戳域（E5）与 offset 单调 | L3 | 节点运行中 | `probe.py --topic /image_combine_raw --domain-check`；`probe.py --topic /imu/data --domain-check`；`ros2 topic echo /imu/data --once --field header.stamp` | 图像与 IMU 的 `stamp.sec` | `stamp.sec > 1.6e9`（**系统时钟域**；若直接透传 SDK MONOTONIC，`sec ≈ 9546`，判据立刻失败 —— 这就是 E5 缺陷的探针）；`abs(stamp.sec - $(date +%s)) ≤ 5`（采样误差上限）；图像与 IMU 的 `stamp` 差 `|ts_img - ts_imu| < 1.0 s`（同一域，出处 E5 实测 `imu - frame = 8.6 ms`）；节点启动日志**逐字**含 `offset ... ns` | offset 的长期漂移（TC-22）、`use_sim_time` 下的行为（TC-22 附加） |
| TC-22 | offset 长期稳定性 + `use_sim_time` 边界 | L3 | 运行 5 min 的节点 | 每 30 s 采一次图像与 IMU 的 `stamp` 差，共 10 次；随后 `$NODE --ros-args -p use_sim_time:=true` 启动 | 10 组 `|ts_img - ts_imu|`；sim time 下的行为 | 10 组差值极差 `≤ 10 ms`（**提案**，依据单次采样误差 ≤ 1 帧周期 33 ms 的 E5 结论；若实测更大，登记真实漂移曲线并上报）；`use_sim_time:=true` 时节点**必须**在日志中打印 `WARN` 说明 offset 不可靠（`RCL_ROS_TIME` 在无 `/clock` 时冻结），且**不得**静默发布冻结的 stamp —— 判据：有 `WARN` 或启动失败，二者之一 | offset 绝对精度（只证稳定性）、温度导致的时钟漂移 |

### 2.6 E8 静默抢流的可检测性

| TC | 目标 | 级别 | 前置 | 命令 | 期望可观测 | 通过判据 | 通过**不**证明 |
|---|---|---|---|---|---|---|---|
| TC-23 | **第二个打开者必须被诊断出来（不是静默停帧）** | L3 | P1–P4，无其它占用 | ① `$NODE -p width:=640 -p height:=480 2>&1 \| tee $EVID/TC-23-a.log & A=$!; sleep 20`；② `timeout 20 ros2 topic info /image_combine_raw --verbose \| tee $EVID/TC-23-before.log`；③ 第二个终端 `$NODE -p width:=640 -p height:=480 2>&1 \| tee $EVID/TC-23-b.log; echo rc=$?`；④ 立刻 `timeout 25 ros2 topic info /image_combine_raw --verbose \| tee $EVID/TC-23-after.log`；⑤ 观察 `$EVID/TC-23-a.log` 25 s | `$EVID/TC-23-b.log` 的 `rc`；`before/after` 的发布者数；A 的日志 | **三种结果都必须被显式归因，无第四种**：(a) B `rc=1` 且日志含 `NOT_FOUND`/`HW_ERROR` → 平台事实 E8 被推翻，**必须**在证据里写明并更新 `00_verified_platform_facts.md`；(b) B 存活且 A 的日志在 15 s 内出现 `ERROR` 含逐字 `no camera frames for` 与 `another process`（看门狗）→ PASS（D9 生效）；(c) B 存活且 A 无任何 ERROR → **FAIL（回归：静默抢流）**。附加判据：`after` 里 `/image_combine_raw` 的 Publisher count 由 2 变 1（B 退出）或保持 1（B 未成功），**出现 2 且 A 继续出图** = A 被抢流的直接证据。**结束后** `pgrep -af gs130_node` 为空 | 看门狗的误报率（TC-24 测）、看门狗在 IMU 也停时是否工作 |
| TC-24 | 看门狗不误报 | L3 | P1–P4 | 单节点正常运行 300 s，`probe.py --seconds 300 --json`；`grep -c "no camera frames" $EVID/TC-24.log` | 日志 | `grep -c` 输出 `0`（正常运行时一次都不许报）；300 s 内 `rate_hz ≥ 27.0`；`dropped == 0`。**若在无占用时误报，判 FAIL**（看门狗只有在帧停 + IMU 仍流时才能触发） | 看门狗在长时间真占用下的持久诊断 |

### 2.7 参数 fail-fast

| TC | 目标 | 级别 | 前置 | 命令（每条独立执行，`timeout 30`） | 期望可观测 | 通过判据 | 通过**不**证明 |
|---|---|---|---|---|---|---|---|
| TC-25 | 非法参数 fail-fast 矩阵（未触碰相机） | L2 | P1–P4，**相机可占用**（用错误路径不需要相机） | 见下表 12 条命令 | 每条 `rc` + 日志文本 | **每条** `rc=2`（参数/配置类）；日志含 `FATAL`（`camera_node.cpp::fatal()` 用 `RCLCPP_FATAL` + 抛异常，实测行为 —— 判据按 `FATAL` 匹配，若实现改用 `ERROR` 则**实现方需说明**，判据同时匹配 `FATAL\|ERROR`）+ 被拒参数名 + 取值 + 原因；**不得**出现 `camera ready`/`first frame published`；每条启动到退出 `≤ 10 s`；退出后 `pgrep -af gs130_node` 为空；`ros2 topic list` 快照里**没有**任何本节点话题（无"半启动"）；**不得**出现未捕获异常回溯（`terminate called after throwing an instance of 'std::runtime_error'` 即 FAIL —— `validate()` 的异常必须被 `main` 捕获并转成退出码 2） | 这些拒绝发生在 `gs130_init` 之前（`20_test_plan.md` E10 已实测，但 C++ 重写后需重测；用 `strace -f -e trace=openat` 观察是否 open `/dev/i2c-*` 才能证明"未触碰相机"——**列为可选强化**） |
| TC-26 | 缺库 / 设备名的资源失败路径 | L2 | P1–P4 | `LD_PRELOAD=/nonexistent.so $NODE 2>&1 \| tee ...; echo rc=$?`；`$NODE -p device:=GS130W 2>&1 \| tee ...; echo rc=$?`；`$NODE -p platform:=RDKX6 2>&1 \| tee ...; echo rc=$?` | 各自 `rc` 与文本 | 缺库：`rc≠0`（期望 1）且 stderr 含 `libgs130` 或 `cannot open shared object file`，**无** core dump；`device:=GS130W`：`rc=1` 且日志含 `NOT_FOUND`（真值出处：E10 `GS130W fails NOT_FOUND on this WI hardware`），**不得**是"启动成功但永远无数据"；`platform:=RDKX6`：`rc=2` 且文本含 `unsupported platform` | 真机缺相机时的行为（TC-36） |

TC-25 的 12 条命令（逐字）：

| # | 命令参数 | 期望原因文本包含（现状 `validate()` 的逐字输出，实测于 `src/camera_node.cpp`） |
|---|---|---|
| 1 | `-p mode:=rgb` | `mode must be one of` |
| 2 | `-p mode:=raw -p width:=640 -p height:=480` | `mode raw requires width=1088 height=1280` |
| 3 | `-p mode:=raw -p width:=1088 -p height:=1280 -p stereo_layout:=left_right` | `mode raw cannot be combined with stereo_layout=` |
| 4 | `-p width:=640 -p height:=479` | `must be even for NV12` |
| 5 | `-p width:=641 -p height:=480` | `must be even for NV12` |
| 6 | `-p width:=864 -p height:=480` | `reports UNSUPPORTED for 864x480`（E10） |
| 7 | `-p width:=1024 -p height:=600` | `reports UNSUPPORTED for 1024x600`（E10） |
| 8 | `-p fps:=0` | `fps must be between 1 and 33` |
| 9 | `-p fps:=120` | `fps must be between 1 and 33`（E10） |
| 10 | `-p odr:=100` | **⚠ 契约缺口，必须实测并立案**：冻结书 §0.2 规定 `odr` 只接受 200/500，而现状 `validate()` 只拒绝 `odr < 1`（实测源码 `if (odr_ < 1) fatal(...)`）。因此 `odr:=100` 很可能**不** fail-fast。判据保留"必须 `rc=2` 且文本含 200"；若实测为 `rc=0`（带 100 Hz 启动），记 `GS130ROS-nnn` 缺陷（契约 vs 实现不一致，**不得**当场放宽判据）。`odr:=0` 必须 `rc=2`（现状可满足） |
| 11 | `-p stereo_layout:=front_back` | `stereo_layout must be one of` |
| 12 | `-p width:=abc` | **必须** `rc=2` + 明确文本；**不得**以 `terminate called after throwing` 未捕获异常退出（`50_cpp_design.md` §3.5 的开放问题：`declare_parameter<int>` 在 launch 传字符串时的类型不匹配）。**允许** launch/参数层提前拒绝（E10 实测行为），但必须把"launch 层拒绝"与"节点层拒绝"两类实测结果都写进证据 |

### 2.8 关停、端口、重复起停

| TC | 目标 | 级别 | 前置 | 命令 | 期望可观测 | 通过判据 | 通过**不**证明 |
|---|---|---|---|---|---|---|---|
| TC-27 | SIGINT/SIGTERM 干净释放 | L3+L4 | `gs130_web.launch.py` 前台运行中（相机+codec+websocket） | `PID=$(pgrep -f "gs130_node" \| head -1); T0=$(date +%s.%N); kill -INT $PID; while kill -0 $PID 2>/dev/null; do :; done; T1=$(date +%s.%N); echo "shutdown=$(echo "$T1-$T0" \| bc)"`；随后 `pgrep -af "gs130_node\|hobot_codec\|websocket" \| tee $EVID/TC-27-ps.log`；`ss -ltnp \| grep :8000 \| tee -a $EVID/TC-27-ps.log` | 退出时间、残留、端口 | 从信号到进程消失 `≤ 3.0 s`；退出码 0（`time kill -INT` 或用 `wait` 取码）；`pgrep` 输出**为空**（`nginx` 例外，见 TC-28）；日志含 `camera released`/等价的一次性释放行；**`SIGINT` 与 `SIGTERM` 两种都要过**；`timeout 60 python3 $REPO/ros/test/reopen.py` 打印 `reopen ok`（相机可被立刻重新打开）。**禁止**用 `pkill -9` 清理后判 PASS | 内存是否回收（TC-38）、端口是否被 OS 回收（TC-28） |
| TC-28 | 端口回到基线 | L4 | TC-27 之后 | 启动前基线：`ss -ltn > $EVID/TC-28-baseline.txt`；停止后：`sleep 5; ss -ltn > $EVID/TC-28-after.txt`；`diff <(grep -c LISTEN $EVID/TC-28-baseline.txt) <(...)`；`ss -ltnp \| grep -E ":8000\|:8001" \| tee -a $EVID/TC-28.log` | 端口列表差异 | `:8000` 在 after 中**不存在**；基线中存在的每个 LISTEN 端口在 after 中仍存在或已释放（无新增监听）；若 `nginx` 孤儿仍在（E4 已知），`ss` 输出必须显示其 PID，且判据降级为"`nginx` 的 PID 与启动前基线的 `nginx` PID 相同或不存在"——**判据不允许改成"忽略 nginx"**，必须是"PID 集合回到基线集合" | 内核 socket 泄漏（由 after/baseline 的 LISTEN 集合比较覆盖） |
| TC-29 | 重复起停 20 轮 | L5 | P1–P4 | 逐字脚本见 §2.9 | 20 轮日志 | 20/20 轮在 `sleep 12` 内出图（每轮 `probe.py --seconds 5 --json` 的 `count > 100`）；每轮 `kill -INT` 后 `rc=0` 且 `sleep 5` 后 `pgrep -af gs130_node` 为空；第 20 轮单轮耗时 `≤ 第 1 轮 × 1.5`；每 5 轮跑一次 `reopen.py` 打印 `reopen ok`；20 轮中任一失败即整体 FAIL 并保留该轮日志 | 长时间 soak（TC-30）、内存泄漏（TC-38 用 counters 覆盖） |

### 2.9 可靠性、内存、性能、端到端

| TC | 目标 | 级别 | 前置 | 命令 | 期望可观测 | 通过判据 | 通过**不**证明 |
|---|---|---|---|---|---|---|---|
| TC-30 | 60 min soak | L5 | P1–P4 | `timeout 3900 ros2 launch gs130_ros gs130_web.launch.py ... &`；并行采样循环（每 30 s 采 `rate_hz`(raw/jpeg/imu)、`RSS`、`/sys/class/thermal/thermal_zone0/temp`）120 次 | 采样表 | `rate_hz` 每点 `≥ 0.9×fps`；**无**连续两点 `< 0.5×fps`；RSS 第 5 min 到结束增长 `≤ 20 MB` 且无单调上升；SoC `≤ 85 °C`；`dmesg -T \| tail -50` 无新 `vpf/vinc/isp` 报错；结束时 `Ctrl-C` 满足 TC-27 | 泄漏根因（只看趋势）、高温环境可靠性 |
| TC-31 | 启动时间与首帧延迟 | L3 | P1–P4 | `T0=$(date +%s.%N); $NODE -p width:=640 -p height:=480 & ; timeout 30 python3 $REPO/ros/test/first_frame.py --json $EVID/TC-31.json`（脚本内记录收到首帧的本地单调时刻，减去传入的 `T0`） | `startup_to_first_frame_s` | 640x480：`≤ 8.0 s`（**提案**，见 §6.2）；1920x1080：`≤ 10.0 s`（提案）；节点日志中出现 `first frame published` 的时刻与脚本测得差 `≤ 0.5 s` | 首帧**可用**（TC-07/TC-08 补）、冷启动 vs 热启动差异（须在证据里记录 uptime） |
| TC-32 | **端到端 web 链路** | L4 | P1–P4 | ① `timeout 180 ros2 launch gs130_ros gs130_web.launch.py platform:=RDKX5 device:=GS130WI mode:=resize width:=1280 height:=720 fps:=30 odr:=200 2>&1 \| tee $EVID/TC-32.log &`；② 30 s 后 `timeout 30 python3 $REPO/ros/test/probe.py --topic /image_combine_raw --seconds 25 --json $EVID/TC-32-raw.json`；③ 同法 `--topic /image_combine_jpeg`；④ `curl -sS -o /dev/null -w "%{http_code}" http://127.0.0.1:8000/`；⑤ 抓两帧间隔 2 s 的 JPEG 存 PNG 比对 | 三个节点的存在、两话题速率、HTTP 码、两帧差异 | `ros2 node list` 含 `gs130_node`、`hobot_codec_encoder_*`、`websocket`；两话题 `rate_hz ≥ 27.0`（出处 E4 实测 29.997 Hz）；HTTP `200`；两帧 JPEG 的像素差 `> 2.0`（**排除冻帧**）；`grep -c "offering incompatible QoS"` 为 `0`；codec 日志含 `Sub imgRaw fps` 与 `Pub img fps` 且都 `≥ 27` | 画面语义正确（由 TC-08 的 PNG 保证）、网页前端 JS 逻辑 |
| TC-33 | `hobot_shm:=false` 下同样通过 | L4 | 同上 | 同 TC-32，加 `hobot_shm:=false` | 同上 | 同 TC-32 全部判据；额外要求 raw 与 jpeg 速率差 `≤ 1.0 Hz` | 零拷贝机制本身（非目标） |
| TC-34 | 内存：无泄漏、无双重释放、无 UAF（**逐字命令见 §4**） | L3 | P1–P4 | §4.3 的 `LD_PRELOAD` 全过程 + §4.4 的 Valgrind 子集 | 分配/释放计数器、mtrace 日志、Valgrind 摘要 | 见 §4.5 的"干净运行"判据（逐条） | 未被执行的代码路径（`rect`/`raw` 需各自跑一次，`stereo_layout:none` 的 `gs130_get_nv12_frame` 路径需单独覆盖） |
| TC-35 | 大数据量下 `publish()` 后立即 `free()` 的所有权安全性 | L3 | 节点运行中（1920x1080@30，`image_qos_depth` 默认） | `LD_PRELOAD` counters 开启 `MALLOC_PERTURB_=165`；同时 `probe.py` 对 `/image_combine_raw` 做帧内容校验（`check_planes.py`）；持续 300 s | 计数器 + 内容校验 | 300 s 内：`allocs - frees` 的摆动 `≤ camera_fifo.depth(4) + 4`；帧内容校验**零**次失败（`MALLOC_PERTURB_` 把已 free 内存填成 `0xA5`，若 `publish()` 之后才序列化被 free 的缓冲，NV12 平面会在 probe 侧出现成片 `0xA5` 的 Y/UV 异常，`check_planes.py` 的 `U/V` 均值判据会失败）；`dropped == 0` | 慢订阅者导致的 RMW 重传路径（见 §6.3 的失效条件） |
| TC-36 | 相机缺失/初始化失败 | L3 | 需可断排线的样机；**无样机则记 BLOCKED** | 断开排线后 `$NODE -p width:=640 -p height:=480 2>&1 \| tee ...; echo rc=$?` | `rc` 与文本 | `rc=1` 且文本含 `gs130_init failed` 与错误码名（`NOT_FOUND`/`HW_ERROR`）；`timeout 15 ros2 topic list \| grep image_combine_raw` 无输出；`ros2 node list` 无本节点；无"启动成功但永远无数据" | 真实 I2C 竞争（需 `mipi_cam` 同跑，本版非目标） |
| TC-37 | `use_sim_time` / `ros2 param set` 边界（非目标但必须不崩） | L3 | 节点运行中 | `timeout 20 ros2 param set /gs130_node mode rect \| tee ...`；随后重读 `camera_info` 与尺寸 | 行为 | 节点**不崩**、`ros2 node list` 仍含本节点、尺寸与 `mode` 相关输出**不变**（运行期不可重配置）；日志**不得**声称已重配置 | 参数是否被正确解析（TC-25 补） |
| TC-38 | 原生 `gs130_get_*` 所有权路径（**桩包一层**） | L3 | P1–P4 | `timeout 90 python3 $REPO/ros/test/frame_ownership.py --mode none --json $EVID/TC-38-none.json`、`--mode left_right`、`--mode top_bottom` | 每模式的 `malloc`/`free` 计数与重复 free 检测 | 三个模式各自：`gs130_get_nv12_frame`（`none`）对 `image_left.data`/`image_right.data` 各 free **恰好一次**；`gs130_get_stereo_nv12_frame`（拼接）对返回的**唯一**指针 free **恰好一次**，且**不得**对 `image_right`/第二个结构体做 free（SDK 在拼接模式下只填一个结构体，另一个是零初始化 —— 见 `core/src/gs130.cpp` 的 LEFT_RIGHT 分支：`frame[0].data = y[L]`，`frame[1].data == nullptr`）；`stop()+deinit()` 后计数器回到 0（FIFO 里残留的帧由 SDK 的 `free_frame_pair` disposer 释放） | ROS 发布路径的所有权（TC-35 补）；SDK 内部的 `malloc` 失败注入 |
| TC-39 | 快速 start/stop（`gs130_start` 的幂等性契约） | L3 | P1–P4 | `timeout 90 python3 $REPO/ros/test/restart.py --rounds 10 --json $EVID/TC-39.json` | 每轮首帧延迟与错误码 | 10 轮 `gs130_stop → gs130_start`（**不** deinit）全部成功取到帧，首帧延迟 `≤ 5 s`；`gs130_start` 在已运行状态下返回 `GS130_PARAM_ERROR`（头文件契约）；`stop()` 在未运行时为 no-op（不崩） | 硬件层面的重复开关（SDK 内部行为） |
| TC-40 | 慢订阅者 / 大消息下的 `free()` 时机（§3.3 的攻击面） | L3 | 1920x1080@30 节点运行中 | 起一个**故意慢**的订阅者：`python3 $REPO/ros/test/slow_sub.py --topic /image_combine_raw --qos reliable --depth 50 --delay-ms 40`；同时 `LD_PRELOAD` counters + `MALLOC_PERTURB_=165` 跑 120 s | 计数器摆动、节点 RSS、`dropped` | `allocs - frees` 摆动 `≤ depth(50) + camera_fifo.depth(4) + 8`（RMW 会持有消息，**这是允许的**）；节点 RSS 增长 `≤ 50 MB`；`probe` 端帧内容校验零失败；`dropped` 允许 `> 0` 但必须在证据中登记（慢订阅者下丢帧是**正确**行为，`camera_fifo` 的 `DROP_OLD` 语义）；**若 RSS 单调增长超过 50 MB 或内容校验失败 → FAIL**，说明 `free()` 与 RMW 序列化存在竞态或重复释放 | 真实慢网络（只覆盖本地 DDS 背压） |
| TC-41 | WiFi/负载下的速率保持（可选强化） | L3 | 同上 | `stress-ng --cpu 2 --timeout 60s &` 同时测 1920x1080@30 | `rate_hz` | `rate_hz ≥ 27.0`（CPU 争用下仍达标）；记录 `frame_cost_ms_p99` 与温度 | 温度长稳（TC-30） |
| TC-42 | `camera_info` 的 latched 语义 | L3 | 节点已运行 ≥ 10 s | 后加入的订阅者 `timeout 15 ros2 topic echo /image_left/camera_info --once` | 是否立刻收到 | 后启动的订阅者 `≤ 5 s` 内收到（`TRANSIENT_LOCAL` 生效）；`ros2 topic info --verbose` 的 Durability 为 `TRANSIENT_LOCAL` | 数值正确性（TC-14 补） |
| TC-43 | QoS 不兼容订阅者的可见性（D13 类） | L3 | 节点运行中 | 记录节点日志行数；起一个 `--qos-reliability best_effort --qos-durability volatile` 的订阅者 30 s；再比日志行数 | 日志增长 | 节点日志增长 `≤ 5` 行（不得刷屏）；**允许**不告警（E1 场景的兼容方向不需要告警），但**不得**崩溃或丢帧 | 订阅者 QoS 事件回调的实现质量 |

### 2.10 需要的验证辅助文件（提案）

| 文件 | 用途 | 关键断言 |
|---|---|---|
| `ros/test/probe.py` | 统一话题探针（替代 `ros2 topic hz` 作为判据来源） | 输出 JSON：`rate_hz/std/max_gap_ms/count/jumps/equal/width/height/step/data_len/stamp_min/stamp_max/frame_id`；`--qos {reliable,best_effort}`、`--json` |
| `ros/test/check_contract.py` | NV12 字段契约 | `encoding=="nv12"`、`step==width`、`len==w*h*3/2`、**`height==--real-height`**（与 SDK 值比对，不接受从 `len` 反推） |
| `ros/test/check_planes.py` | 平面/内容校验（含 `MALLOC_PERTURB_` 检测） | `Y.std>20`、`U/V` 均值 ∈ [60,200]、无成片 `0xA5`/`0x5A`、边缘密度比 ∈ [0.25,4.0] |
| `ros/test/check_pair.py` | 左右目成对 | `delta_stamp_ns` 分布 + `mean abs diff > 2.0` |
| `ros/test/probe_imu.py` | IMU 统计 | `count/rate_hz/accel_mean/std/norm/gyro_mean/stamp` 序列 |
| `ros/test/sdk_truth.py` | SDK 真值导出（独立进程，只 `init/deinit`） | 内参、外参、`install_angle`、EEPROM 串 |
| `ros/test/first_frame.py`、`reopen.py`、`restart.py`、`frame_ownership.py`、`slow_sub.py` | 见对应 TC | 见对应 TC |
| `ros/test/malloc_count.c` | `LD_PRELOAD` 分配计数器 + 重复 free 断言（见 §4.3） | `allocs/frees/dup_free/realloc` 计数，`SIGUSR1` dump 到文件 |
| `ros/test/profiling` 宏 `GS130_NODE_PROFILE` | 逐帧打点（`frame_cost_ms_*`），经 `/gs130/status` 暴露 | 见 §5.1 |

统一执行脚本骨架（TC-29）：

```bash
for i in $(seq 1 20); do
  t0=$(date +%s.%N)
  timeout 45 ros2 launch gs130_ros gs130_web.launch.py > $EVID/round_$i.log 2>&1 &
  p=$!
  sleep 12
  timeout 15 python3 $REPO/ros/test/probe.py --topic /image_combine_raw --seconds 5 \
      --json $EVID/round_$i.json >> $EVID/round_$i.log 2>&1
  timeout 15 python3 $REPO/ros/test/probe.py --topic /image_combine_jpeg --seconds 5 \
      --json $EVID/round_${i}_jpg.json >> $EVID/round_$i.log 2>&1
  kill -INT $p; wait $p; echo "round $i rc=$? dur=$(echo "$(date +%s.%N)-$t0" | bc)" >> $EVID/rounds.txt
  sleep 5
  pgrep -af "gs130_node|hobot_codec|websocket" >> $EVID/leftover_$i.log
  [ $((i % 5)) -eq 0 ] && timeout 60 python3 $REPO/ros/test/reopen.py >> $EVID/rounds.txt 2>&1
done
```

---

## 3. 三个实测缺陷的"复活即响亮失败"设计

| 缺陷 | 原症状（`25_test_report.md` / E1/E2/E8） | 若在 C++ 版本复活 | 哪个 TC 会让它**响亮**失败 | 为什么该判据有鉴别力（**没有它就会静默通过**的原因） |
|---|---|---|---|---|
| D1：`RELIABLE` QoS（E1） | `hobot_codec` 日志 `offering incompatible QoS ... RELIABILITY_QOS_POLICY`，**一条消息都不达**，页面黑屏但**无任何崩溃** | 发布端回到 `qos_profile_sensor_data`（BEST_EFFORT） | **TC-05**（`ros2 topic info --verbose` 的 `Reliability: RELIABLE` + codec 日志 `grep -c "offering incompatible QoS" == 0` + `/image_combine_jpeg` 速率 ≥ 27）；**TC-32/TC-33** 作为端到端兜底 | **"订阅者能收到帧"完全不能证明 RELIABLE**：DDS 允许 RELIABLE 发布者匹配 BEST_EFFORT 订阅者，且 `ros2 topic echo` 默认常是 BEST_EFFORT，BEST_EFFORT 发布者与它匹配得**完美**。唯一鉴别力来自 offer 侧的值（`--verbose`）与"RELIABLE 订阅者是否收不到"（codec 端到端）。缺 TC-05 的话，整个方案会在 D1 上全绿放过 |
| D2：`Image.height` 用 NV12 打包高（E2） | codec `init_pic_h_: 1080, alined_pic_h_: 1088` 后**段错误 exit -11** | 节点从 `len(data)` 反推 `height = len*2/(3*width)`（数学上"自洽"，任何长度恒等式都通不过鉴别） | **TC-07**：除长度恒等式外，强制断言 `height == SDK output_height`（由节点启动日志的 `geometry WxH` 提供，差必须为 0）；**TC-32** 的 codec 存活作为兜底（`process has died` / `exit code -11` 直接 FAIL） | `len(data) == width*height*3/2`、`step == width`、`encoding == "nv12"` 这三条在打包高下**全部成立**。上一版 Python 的 9 个单测里 `test_nv12_uses_the_real_height` 之所以有效，是因为它把 `height` 与**输入 shape 的高**对比（`frame.shape[0]*2//3`）。C++ 版本的等价做法只有一个：`height` 必须来自 **SDK 的 `gs130_image_nv12_t.height`**，任何"从字节数反推"的实现都必须在单测里被显式钉住（H1 的输入是 `(w, h, layout)` 而不是 buffer 长度） |
| D3：静默抢流（E8） | 第二个进程不报错、抢走帧流，第一个进程**帧冻结在 391 而 IMU 升到 7086**，无任何 ERROR | C++ 看门狗未实现，或看门狗条件写成"帧停就报"（会误报）/条件过严（永远不报） | **TC-23**：三种结果必须显式归因，(c) 分支（B 存活 + A 无 ERROR）直接 FAIL；同时用 `ros2 topic info --verbose` 的 Publisher count 与速率交叉验证"A 真的停帧了"；**TC-24** 反向防误报（正常 300 s 内 `grep -c == 0`） | 这个缺陷**不会**以任何错误码或崩溃形式出现：`gs130_init` 成功、话题存在、`ros2 topic list` 正常、IMU 继续发布。**唯一**的探针是"帧计数与 IMU 计数的相对变化"。只测"单实例能出图"的用例在此缺陷上 100% 全绿。另外必须把 E8 的**两种可能**都写成判据：(a) 平台行为变了（B 失败退出）必须当场登记并更新 `00_verified_platform_facts.md`，而不是判 FAIL 或忽略 |

---

## 4. 内存正确性验证（C++ 特有）

### 4.1 必须被证明的四个命题

| 命题 | 失效后果 | 证据工具 |
|---|---|---|
| M1 每帧 `malloc` 恰好对应一次 `free`（无泄漏） | 6.22 MB/帧 × 30 fps → 数秒内 OOM | 分配计数器（§4.3）+ Valgrind（§4.4）+ ASan（§4.2） |
| M2 无双重释放 | `free(): double free detected` / 段错误；SDK FIFO 丢弃路径与节点 `free()` **是同一块内存的两个释放点**（`free_frame_pair` disposer 见 `core/src/gs130.cpp:212`） | 计数器 `dup_free` + Valgrind + `MALLOC_CHECK_` |
| M3 无 use-after-free | 发布出去的是已释放内存 → 花屏/`0xA5` 成片/随机崩溃 | ASan（桩）、`MALLOC_PERTURB_=165` + 内容校验（TC-35/TC-40） |
| M4 `free()` 的时机相对 RMW 序列化是安全的（§3.3 的攻击面） | RELIABLE + 大消息 + 慢订阅者下重传读到已释放内存 | TC-35/TC-40 的计数器摆动 + 内容校验 |

### 4.2 主机：ASan + 桩设备（覆盖 M1/M2/M3 的**代码层**）

```bash
ASAN_OPTIONS=detect_leaks=1:detect_stack_use_after_return=1:strict_string_checks=1:abort_on_error=0:halt_on_error=0 \
UBSAN_OPTIONS=print_stacktrace=1:halt_on_error=0 \
ctest --test-dir $REPO/ros/gs130_ros/build_host -R test_frame_ownership --output-on-failure \
  | tee /tmp/asan_frame_ownership.log
```

桩设备（`test/stub_device.cpp`）的契约：

* 每个"帧"返回 `malloc(w*h*3/2)`（拼接模式为 `malloc(w*h*3)` 的**单个**指针，第二个结构体 `data == nullptr`，复刻 `core/src/gs130.cpp` 的 LEFT_RIGHT 分支）；
* 桩里记账 `alloc_count / free_count / free_ptr 集合`，`free` 时若指针已在集合里 → 立即 `FAIL()` 并打印双 free 的调用栈；
* 构造 5 类路径：正常帧、发布抛异常、`publish()` 返回后立即析构、`pop` 得到 `GS130_TIMEOUT`、`stop()` 时 FIFO 里仍有 4 帧（`DROP_OLD` 丢弃路径）。

**注意 ASan 的覆盖边界（必须写进证据）**：`libgs130.so` 未用 `-fsanitize=address` 编译，所以 ASan **看不到** `libgs130` 内部的堆写越界与内存错误，只能看到节点自身的坏 `free`、双重释放、以及后续对已释放块的访问。要覆盖 SDK 内部，用 §4.4 的 Valgrind（无需重新编译）。

### 4.3 板端：`LD_PRELOAD` 分配计数器（覆盖 M1/M2/M4，全分辨率可用，开销低）

`ros/test/malloc_count.c`（逐字，可直接落盘）：

```c
#define _GNU_SOURCE
#include <dlfcn.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>

static void *(*real_malloc)(size_t) = NULL;
static void  (*real_free)(void *)   = NULL;
static __thread int in_hook = 0;
static _Atomic long allocs = 0, frees = 0, dup_free = 0, peak_live = 0, live = 0, bad_ptr = 0;
#define TRACK_MAX 65536
static void *tracked[TRACK_MAX];
static pthread_mutex_t track_mtx = PTHREAD_MUTEX_INITIALIZER;

static void init_real(void) {
    if (!real_malloc) real_malloc = (void *(*)(size_t))dlsym(RTLD_NEXT, "malloc");
    if (!real_free)   real_free   = (void (*)(void *))dlsym(RTLD_NEXT, "free");
}
static void track_add(void *p) {
    pthread_mutex_lock(&track_mtx);
    for (int i = 0; i < TRACK_MAX; i++) if (!tracked[i]) { tracked[i] = p; break; }
    pthread_mutex_unlock(&track_mtx);
}
static int track_del(void *p) {           /* returns 0 when the pointer was NOT live */
    int found = 0;
    pthread_mutex_lock(&track_mtx);
    for (int i = 0; i < TRACK_MAX; i++) if (tracked[i] == p) { tracked[i] = NULL; found = 1; break; }
    pthread_mutex_unlock(&track_mtx);
    return found;
}
void *malloc(size_t n) {
    init_real();
    if (in_hook) return real_malloc(n);
    in_hook = 1;
    void *p = real_malloc(n);
    in_hook = 0;
    if (p) { allocs++; live++; if (live > peak_live) peak_live = live; track_add(p); }
    return p;
}
void free(void *p) {
    init_real();
    if (!p) return;
    if (in_hook) { real_free(p); return; }
    in_hook = 1;
    if (!track_del(p)) { dup_free++; fprintf(stderr, "[malloc_count] DOUBLE FREE or foreign pointer %p\n", p); }
    real_free(p);
    in_hook = 0;
    frees++; live--;
}
void *calloc(size_t nmemb, size_t size) {
    init_real();
    void *p = real_malloc(nmemb * size);
    if (p) { memset(p, 0, nmemb * size); allocs++; live++; if (live > peak_live) peak_live = live; track_add(p); }
    return p;
}
void *realloc(void *p, size_t n) { init_real(); if (p && track_del(p)) live--; void *q = real_malloc(n); if (q) { allocs++; live++; track_add(q); } if (p) real_free(p); return q; }
static void dump(int sig) {
    (void)sig;
    char buf[512];
    int n = snprintf(buf, sizeof buf,
        "allocs=%ld frees=%ld live=%ld peak_live=%ld dup_free=%ld\n",
        (long)allocs, (long)frees, (long)live, (long)peak_live, (long)dup_free);
    int fd = open(getenv("MALLOC_COUNT_OUT") ? getenv("MALLOC_COUNT_OUT") : "/tmp/mc.txt",
                  O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd >= 0) { if (write(fd, buf, n) < 0) {} close(fd); }
}
__attribute__((constructor)) static void setup(void) { signal(SIGUSR1, dump); }
```

（编译：`gcc -shared -fPIC -O2 -o /tmp/libmc.so ros/test/malloc_count.c -ldl -lpthread`。）

运行（相机全速，开销可忽略）：

```bash
gcc -shared -fPIC -O2 -o /tmp/libmc.so $REPO/ros/test/malloc_count.c -ldl -lpthread || exit 1
P=$(pgrep -f gs130_node | head -1)
MALLOC_COUNT_OUT=$EVID/TC-34-mc-start.txt kill -USR1 "$P"; sleep 1
echo "== 60 s 全速运行 =="; sleep 60
MALLOC_COUNT_OUT=$EVID/TC-34-mc-end.txt kill -USR1 "$P"; sleep 1
MALLOC_CHECK_=3 MALLOC_PERTURB_=165 LD_PRELOAD=/tmp/libmc.so \
  timeout 120 ros2 run gs130_ros gs130_node --ros-args -p mode:=resize -p width:=640 -p height:=480 \
  2>&1 | tee $EVID/TC-34-run.log
```

（本 TC 需要进程以 `LD_PRELOAD` 启动，因此不能用 `pgrep` 注入；把 `LD_PRELOAD` 与 `MALLOC_COUNT_OUT` 放在 `ros2 run` 前面，采样用两次 `kill -USR1` 夹住 60 s。`MALLOC_PERTURB_=165` 让 `free` 后的字节变 `0xA5`，`MALLOC_CHECK_=3` 让 glibc 的堆一致性检查失效时**立即 abort** 并打印 `free(): invalid pointer`。）

### 4.4 板端：Valgrind memcheck（唯一能覆盖 `libgs130` 内部的手段）

```bash
sudo apt-get install -y valgrind   # 若未预装；不改仓库、不改 SDK
LOG=$EVID/TC-34-valgrind.log
valgrind --tool=memcheck --leak-check=full --show-leak-kinds=definite,indirect \
         --errors-for-leak-kinds=definite --track-origins=yes --num-callers=40 \
         --error-exitcode=99 --log-file=$LOG \
  timeout 90 ros2 run gs130_ros gs130_node --ros-args \
    -p mode:=resize -p width:=320 -p height:=240 -p fps:=15
echo "valgrind rc=$?  (99 == valgrind 报告了错误；管道不可用，否则 rc 会被 tee 覆盖)" | tee -a $LOG
```

**为什么必须降规格**：Valgrind 串行化执行、无 JIT，全分辨率 30 fps 下 SDK 的取帧超时（`get_frame(...,100)`）会频繁失败，产生的 TIMEOUT/丢帧会被误判成缺陷。本用例的**目的**是内存正确性而不是速率，所以用 320x240@15 并把运行时长压到 90 s（≥ 300 帧，足以覆盖 FIFO 丢弃路径）。**速率判据在 TC-10/TC-09 里，本用例不做速率断言** —— 这一点必须写在证据里，防止有人拿 Valgrind 的慢速去否定设计。

### 4.5 干净运行长什么样（逐条判据）

| 工具 | 干净输出 | 出现即 FAIL |
|---|---|---|
| ASan（§4.2） | 进程正常退出，`rc=0`，stderr 无 `ERROR: AddressSanitizer`、无 `LeakSanitizer: detected memory leaks` | `heap-use-after-free` / `double-free` / `heap-buffer-overflow` / `LeakSanitizer: detected memory leaks` / `SUMMARY: AddressSanitizer: <N> byte(s)` |
| malloc_count（§4.3） | 60 s 前后 `live` 都 `≤ camera_fifo.depth + imu_fifo 未消费量 + DDS 待发消息数`（经验上 `≤ 64`），且 `live_end - live_start` 的绝对值 `≤ 16`；`dup_free == 0`；`free` 日志中**无** `DOUBLE FREE or foreign pointer` | 任何一行 `DOUBLE FREE`；`live` 随采样单调增长（60 s 内 `live_end - live_start > 64`）；`live` 为负 |
| mtrace（可选） | `mtrace` 日志经 `mtrace -l` 后无 `Memory not freed` 块，或仅有一次性初始化分配 | 与帧数成正比的 `malloc` 数量未被 `free` |
| Valgrind（§4.4） | `ERROR SUMMARY: 0 errors from 0 contexts`；`definitely lost: 0 bytes in 0 blocks`；`indirectly lost: 0 bytes`；`possibly lost` 允许非零但必须在证据中列出并逐个归因到 DDS/SDK 的一次性分配（附 `--num-callers=40` 的栈） | `Invalid free() / delete / delete[]`；`Invalid read/write of size N`；`definitely lost` 与帧数成正比（如 300 帧 × 3.11 MB ≈ 933 MB）；`ERROR SUMMARY` 非 0 且栈落在 `gs130_get_*_frame` / `free_frame_pair` / 节点的 `free()` 上 |
| 进程级 | 全程无 `free(): invalid pointer` / `corrupted size vs. prev_size` / `munmap_chunk(): invalid pointer`；`dmesg -T` 无新的 OOM/`vinc` 报错 | 上述任一字符串；`exit code -11`/`-6` |

**必须分别在三种布局下各跑一次 §4.3**：`stereo_layout:=left_right`（走 `gs130_get_stereo_nv12_frame`，单指针）、`stereo_layout:=none -p publish_per_eye:=true`（走 `gs130_get_nv12_frame`，双指针）、`stereo_layout:=top_bottom`（拼接的另一种内存切分）。**只跑一种布局的干净结果不能外推到另一种** —— 三者在 `core/src/gs130.cpp` 里走三条不同的 `malloc`/切片分支。

---

## 5. 性能判据（含数字与测法）

### 5.1 逐帧成本预算 @1920x1080@30 —— 怎么测

节点在 `GS130_NODE_PROFILE=ON` 编译时，对每帧记录三段单调时钟（`std::chrono::steady_clock`，纳秒）：

| 段 | 起 | 止 | 期望量级 |
|---|---|---|---|
| `fetch_ms` | 调用 `gs130_get_stereo_nv12_frame` 前 | 返回 `GS130_OK` 后 | SDK 已把数据写入 malloc 缓冲，本段只做 FIFO 弹出（零拷贝）→ `≤ 0.5 ms` |
| `build_ms` | 开始 `memcpy` 到 `Image::data` | `memcpy` 结束 | 6.22 MB memcpy；按 ~4 GB/s 有效带宽估 `≈ 1.6 ms` |
| `publish_ms` | 调用 `publish()` 前 | `publish()` 返回 | rclcpp 序列化 + RMW 入队；`25_test_report.md` 实测 Python 侧 `publish 5.07 ms`（含 DDS 序列化）→ 提案 `p99 ≤ 26.7 ms` |
| `free_ms` | `free(data)` 前 | `free` 后 | `≤ 0.1 ms` |
| `frame_cost_ms` | `fetch` 起 | `free` 止 | **主判据** |

暴露方式（判据必须可自动断言，不能只靠日志）：

* 每 1 s 更新 `/gs130/status`（`diagnostic_msgs/DiagnosticStatus`），`values` 含 `frame_cost_ms_median / frame_cost_ms_p95 / frame_cost_ms_p99 / frame_cost_ms_max / dropped / fifo_depth_max / imu_queue_max / frames / imu`；
* 同时每 5 s 打一行 `INFO` 汇总（供人读）；
* 采样窗口 = 30 s（≈ 900 帧），窗口内**所有**帧都进直方图（不抽样）。

| 指标 | 判据（**提案**，出处见 §6.2） | 失败含义 |
|---|---|---|
| `frame_cost_ms_median` @1080p | `≤ 8.0 ms` | 半周期 `1/(2×30) = 16.7 ms` 的一半；超过说明存在非必要的拷贝或分配 |
| `frame_cost_ms_p99` @1080p | `≤ 26.7 ms` | 80% 的帧周期；超过就会让图像定时器迟到、`available_camera()` 抬升 |
| `frame_cost_ms_max` @1080p | `≤ 33.3 ms` | 超过一个周期 = 至少丢掉一帧的时隙 |
| `dropped` @1080p、30 s | `== 0` | 直接反证单线程定时器模型 |
| `fifo_depth_max` @1080p | `≤ 1` | `camera_fifo.depth == 4`；持续 `≥ 3` 说明消费慢于生产（`50_cpp_design.md` §3.2 的判决性指标） |
| 节点 CPU 占用 | `≤ 60%` 单核（`pidstat -p <pid> 1 30 \| awk 'NR>3{sum+=$8;n++}END{print sum/n}'`） | 说明存在忙等或过度拷贝；**此判据是提案，须在首次实测后按实测值校准** |
| 速率保真 @1080p | `rate_hz ∈ [27.0, 31.5]`；`std ≤ 3.0 ms`；`max_gap_ms ≤ 200` | 与 TC-10 相同判据在最高分辨率下再验一次 |

**注意**：`publish()` 返回**不代表** RMW 已完成序列化（`50_cpp_design.md` §3.3 的核心质疑）。上面的 `frame_cost_ms` 只测到 `free()` 为止，因此 TC-35/TC-40 用计数器与内存扰动独立证明"提前 `free()` 是安全的"；两者不可互相替代。

### 5.2 速率容差（统一口径）

| 场景 | 下界 | 上界 | 出处 |
|---|---|---|---|
| `resize` 320x240 / 640x480 / 1280x720 / 1920x1080 @30 | `0.90×fps = 27.0` | `1.05×fps = 31.5` | E10 实测全部 30 fps；容差沿用 `20_test_plan.md` TC-06 |
| `rect` 1088x1280 | `18.0` | `31.5` | E10 实测约 20 fps；**不得**按 30 判 FAIL |
| `/image_combine_jpeg` | `≥ 0.9 ×` 源话题速率 | — | E4 实测两话题同为 29.997 Hz |
| `/imu/data` @odr=200 | `170` | `230` | `25_test_report.md` 实测 202.9 Hz |
| `/imu/data` @odr=500 | `425` | `575` | 提案（相同比例） |
| 窗口长度 | `ros2 topic hz` 一律 `--window 300`（30 fps 下 ≈ 10 s）；`probe.py` 一律 `--seconds 30` | | 避免窗口过小导致的假 P99 |

### 5.3 启动与关停时间

| 指标 | 定义（怎么测） | 判据（提案） | 依据 |
|---|---|---|---|
| `startup_to_first_frame_s` | `T0 = date +%s.%N` 紧接在 `ros2 run` 之前；`first_frame.py` 在收到首帧时用 `time.monotonic_ns()` 记 `T1`（两者同机时钟）；`T1 - T0` | 640x480 ≤ **8.0 s**；1920x1080 ≤ **10.0 s** | E5 记录 SDK 需等 IMU FSYNC 握手才开流；上一版 Python 在 30 s 内出图（`25_test_report.md` §3.1 的 30 s 内含 752 帧）。8 s 是**提案**，首次实测后校准 |
| `startup_to_first_frame_s`（`device:=GS130W`，无 IMU） | 同上 | ≤ **4.0 s** | 无 IMU 时 SDK 跳过握手（`core/src/gs130.cpp` 的 `camera_on = true` 分支）→ 应显著更快；若两者相同，说明握手逻辑没有被跳过 |
| `startup_to_ready`（节点自己的 `first frame published` 日志） | `T0` 到日志行出现 | 与 `startup_to_first_frame_s` 差 `≤ 0.5 s` | 交叉验证探针与节点日志一致 |
| `shutdown_s` | `kill -INT <pid>` 到 `kill -0` 失败 | `≤ 3.0 s`，`rc=0` | 沿用 `20_test_plan.md` §0.2 的冻结 SIGINT 契约（3 秒） |
| `shutdown_s`（SIGTERM） | 同上 | `≤ 3.0 s`，`rc=0` | 同上 |
| `reopen_gap_s` | 上一进程退出到 `reopen.py` 打印 `reopen ok` | `≤ 5.0 s` | 相机独占释放的可观测延迟 |
| 端口回到基线 | `ss -ltn` 的 LISTEN 集合 | 5 s 内 `:8000` 不在集合中；LISTEN 集合 = 基线集合 | E4 的 nginx 孤儿问题 |

---

## 6. 本方案的局限

### 6.1 本方案**不能**证明的东西

| 不能证明 | 为什么 | 谁能证 |
|---|---|---|
| 图像内容语义正确（谁在画面里、标定绝对精度） | 自动化只能证"可解码、非恒定、有视差" | 人工目视 + 标定评审 |
| 标定（内参/外参）的绝对准确度 | 只比对"SDK 报什么 ROS 报什么" | 重新标定 / 第三方标定工具（非目标） |
| 图像与 IMU 的真实时间对齐质量 | 只证"同一时基 + 差值在量级内" | 需要运动激励的联合标定 |
| `libgs130` 内部的内存正确性（除 Valgrind 子集外） | ASan 看不到未插桩的库；Valgrind 无法覆盖全速率全分辨率 | SDK 自己的测试 + `core/samples` |
| 长时间（>60 min）与温度循环下的行为 | 只做 60 min 常温 soak | 老化测试（非目标） |
| 慢网络、跨机订阅、远程 `rosbag record` 的 QoS 重传行为 | 只在板内 DDS 覆盖 | 现场集成测试 |
| `mode:=raw` 的 20 fps 上限是否"可接受" | 平台结论，不是本节点的问题 | 需求方 |
| 任何硬件的**机械**问题（排线、散热） | 超出软件可观测范围 | 硬件团队 |
| 第二个打开者场景中"谁"偷了流 | 只能诊断"帧停了"，无法归因到具体 PID 的相机占用 | 需要 SDK/驱动层提供 owner 信息（建议作为 SDK 需求提出） |
| 看门狗的报告延迟上限 | 只验"15 s 内报出"，未验"最坏多久" | 需要注入式测试（人为暂停取帧） |

### 6.2 哪些是提案而非实测事实

| 编号 | 提案值 | 来源 | 状态 |
|---|---|---|---|
| P-1 | 速率容差 `[0.90, 1.05]×fps` | 沿用 `20_test_plan.md` TC-06 | 提案（沿用，尚未在 C++ 版实测） |
| P-2 | `frame_cost_ms_median ≤ 8.0`、`p99 ≤ 26.7`、`max ≤ 33.3` @1080p | 本文按半周期 16.7 ms 推导 | **提案**；首次实测后必须按实测分布重新标定（若实测 median 为 3 ms，则应收紧到 6 ms 才有回归价值） |
| P-3 | 节点 CPU `≤ 60%` 单核 | 本文估计 | **提案**；须校准 |
| P-4 | `startup_to_first_frame_s ≤ 8.0 s`（有 IMU）/`≤ 4.0 s`（无 IMU） | 依据 E5 的 FSYNC 握手与上一版 Python 的 30 s 内出图 | **提案** |
| P-5 | IMU `abs(accel_norm - 9.81) ≤ 0.30` | 比 `20_test_plan.md` 的 ±0.5 更紧 | **提案**；若样机本身偏差大，改为登记实测值 |
| P-6 | IMU 速率窗口 `[0.85, 1.15]×odr`；`max_gap_ms ≤ 200` | 由 FSYNC 突发特性推得 | **提案**；须实测确认 burst 长度 |
| P-7 | `live` 摆动 `≤ 64`、RSS 增长 `≤ 20 MB`/`50 MB` | 本文估计（camera_fifo 4 + DDS 待发） | **提案**；用 `--depth` 与实际 DDS 队列长度校准 |
| P-8 | 看门狗窗口 10 s、30 s 内不重复刷屏 | E8 的看门狗描述 | 提案（沿用 Python 版行为） |
| P-9 | `fifo_depth_max ≤ 1` @1080p | 由 `camera_fifo.depth == 4` 实测值导出 | 判据是**推导**，非实测 |
| P-10 | `use_sim_time:=true` 时必须 `WARN` 或拒绝启动 | `50_cpp_design.md` §3.4 的开放问题 | **提案**；两条路径都可以，但必须二选一并在文档写明 |

**已实测的真值**（可直接当判据用，不需再校准）：E10 的分辨率矩阵、E2 的字段契约、E1 的 QoS 兼容方向、E5 的 CLOCK_MONOTONIC 与 8.6 ms 图像/IMU 差、E4 的 29.997 Hz 与 HTTP 200、`25_test_report.md` 的 752 帧/30 s ≈ 30.1 fps、内参 `386.85/387.12/304.62/245.06`、基线 `0.070316 m`、IMU 202.9 Hz @odr=200。

### 6.3 什么证据会**推翻**一次运行（判 FAIL 或作废）

| 失效条件 | 处置 |
|---|---|
| 命令被改写（加参数、改分辨率、改 `ROS_DOMAIN_ID`、去掉 `timeout`） | 该 TC 证据作废 |
| 判据被就地放宽（"大约 29 Hz 也算过"、"height 差 240 也算过"） | 作废 + 立案（`GS130ROS-nnn`） |
| 用 `pkill -9`、`ros2 daemon stop` 掩盖残留进程或端口问题 | TC-27/TC-28 判 FAIL |
| 测试前 `pgrep -af "mipi_cam\|gs130"` 不为空 | 该次全部 L3/L4 结果作废（E8 的静默抢流会使一切速率结果不可信） |
| 用 Valgrind/ASan 的慢速去否定性能判据，或用性能达标去否定内存判据 | 两类判据互不覆盖，混用即作废 |
| 只跑单实例就断言 E8 相关结论 | TC-23 未执行则"无静默抢流"结论作废 |
| 只跑 `stereo_layout:=left_right` 就断言内存正确 | 见 §4.5 末段，另两种布局未跑则结论范围受限，必须写在结论里 |
| 节点日志出现 `dropped > 0` 却仍判性能 PASS | 作废（`dropped == 0` 是 @1080p 的硬判据） |
| 相机被占用/被第二个进程抢流后仍采集的速率数据 | 作废 |
| 证据缺 `rc`、缺完整 stdout/stderr、缺时间戳 | 该 TC 记 BLOCKED（不是 PASS） |
| `git status --porcelain` 非空（测的是未提交代码） | 整轮作废 |
| `00_verified_platform_facts.md` 与实测冲突（如 `device:=GS130W` 居然成功） | **不是**判 FAIL：当场登记差异、更新事实文档、该 TC 记 BLOCKED 并重新设计 |

### 6.4 交付物与判据的分离

本文的**判据**（数字、文本、字段关系）不依赖 C++ 的符号名与文件布局。`52_architecture_cpp.md` 落地后若函数名/目录变化，只需替换 §1.1 与 §2.10 的符号名；**任何判据的放宽都属于契约修订，必须走评审**，不得由实现方在 PR 里顺手改掉。同理，"按 `len(data)` 反推 `height`"、"用 `ros2 topic echo` 收到帧就断言 QoS"、"只测单实例就断言无抢流"这三种做法即使在实现里"看起来能过"，也必须按 §3 判 FAIL。

---

## 7. 最可能暴露真实缺陷的三个用例

| 排序 | TC | 为什么最可能抓到真 bug | 若它的判据被弱化会漏掉什么 |
|---|---|---|---|
| 1 | **TC-09 + TC-10 的 `fifo_depth_max`/`dropped` 与 1080p 速率**（`50_cpp_design.md` §3.2 的攻击目标） | 单线程 + 两个 rclcpp 定时器的模型在 640x480 下几乎必然通过，**只有在 1920x1080@30 才可能暴露**：6.22 MB/帧的 `memcpy` + 序列化 + RELIABLE 传输叠加在同一个 executor 上，且 SDK 的 `camera_fifo`（depth 4）在节点跟不上时走 `DROP_OLD` **静默**丢帧。`dropped`/`fifo_depth_max`/`frame_cost_ms_p99` 是**唯一**能把这三种失败区分开的可观测（速率下降=消费慢；`dropped>0`=队列满；`p99` 尖峰=GC/分配抖动）。这是设计者自己列为"请攻击"的点，也是最可能有真实缺陷的地方 | 只测 640x480 会用"一切正常"掩盖设计缺陷；只看平均速率会把"偶发 300 ms 停顿 + 补帧"误判为健康 |
| 2 | **TC-23（+TC-24）E8 静默抢流的诊断** | 这个缺陷的**全部**症状是"没有任何症状"：初始化成功、话题正常、IMU 继续发。C++ 重写最容易在这里退回原状（看门狗漏移植、条件写错、日志级别用 `WARN` 导致 `grep ERROR` 失败）。而它又是**现场最常见**的故障形态（用户误起 `mipi_cam` 或第二个节点），一旦退回，用户看到的是"相机哑了但没有错误"。TC-23 的 (b)/(c) 二分支加上 TC-24 的反向防误报，同时钉住"该报必须报"和"不该报不许报" | 只测单实例 → 100% 全绿；只检查"进程没崩" → 全绿；把看门狗判据写成"帧停即报" → 正常负载抖动时刷屏，最终被运维静音，等于没有看门狗 |
| 3 | **TC-05 + TC-07 的 QoS 与 `height` 契约** | 这两个是 E1/E2 的实测缺陷，而它们的**正确判据是最容易被写错的**：E1 的正确判据在 offer 侧（`--verbose` + codec 端到端），而"订阅者收到帧"这个直觉判据对 BEST_EFFORT **同样成立**；E2 的正确判据是"`height == SDK 的真实高"，而所有长度恒等式（`len == w*h*3/2`、`step == w`）在打包高下**全部成立**。因此这两个 TC 是最容易"假 PASS"的，也最可能在评审中暴露"测试写错了而不是代码错了" | 用 `ros2 topic echo` 收帧判 QoS → D1 复活时全绿；用长度恒等式判 `height` → D2 复活时全绿，且残余风险不是"图像慢"而是 **codec 段错误（exit -11）导致整条 web 链路死掉** |

---

## 附录 A：用例计数与覆盖对照

| 组 | TC | 数量 |
|---|---|---|
| 构建/主机纯函数（H/L2） | TC-01 … TC-04 | 4（其中 H 级 2：TC-03、TC-04） |
| 话题/QoS/消息契约 | TC-05 … TC-09 | 5 |
| 帧率/分辨率/模式/布局 | TC-10 … TC-13 | 4 |
| 标定/TF/降级 | TC-14 … TC-17 | 4 |
| IMU | TC-18 … TC-22 | 5 |
| E8 抢流 | TC-23 … TC-24 | 2 |
| 参数 fail-fast | TC-25 … TC-26（TC-25 内含 12 条子命令） | 2 |
| 关停/端口/重复起停 | TC-27 … TC-29 | 3 |
| 内存/可靠性/性能/端到端 | TC-30 … TC-43 | 14 |
| **合计** | | **43**（其中**主机可执行 2 个 TC + H1–H11 共 11 个函数/目标**） |

必须覆盖的 14 项（任务书要求）对照：

| 要求 | 对应用例 |
|---|---|
| 构建并链接 `libgs130` | TC-01、TC-02 |
| 话题存在与类型 | TC-06 |
| `Image` 字段契约 | TC-07、TC-08 |
| QoS 真的匹配 codec（不只是存在） | **TC-05**（+TC-32/33 兜底） |
| 多分辨率帧率（含被拒项） | TC-10、TC-11 |
| IMU 速率与单位 | TC-18、TC-19、TC-20 |
| `CameraInfo` 对 EEPROM | TC-14、TC-15 |
| 静态变换 | TC-16 |
| E8 静默抢流 | **TC-23**、TC-24 |
| 每个非法参数 fail-fast | **TC-25**（12 条）、TC-26 |
| SIGINT 释放 + 无残留 + 端口回基线 | TC-27、TC-28 |
| web 链路端到端 | TC-32、TC-33（+TC-05 的 codec 段） |
| 重复起停循环 | TC-29（+TC-30 soak） |
| 内存正确性（泄漏/双 free/UAF） | TC-34、TC-35、TC-38、TC-40（§4 全套工具） |
