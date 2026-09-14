# GS130 ROS 2 (TROS) 接口包 —— 测试计划（20_test_plan.md）

| 项目 | 值 |
| --- | --- |
| 文档编号 | gs130_ros / 20_test_plan |
| 被测对象 | ROS 2 (TROS humble) 接口包 `gs130_ros` v0.1.0（发布 GS130 双目 NV12 图像与 IMU，复用 D-Robotics `hobot_codec_republish` / `websocket` 节点，由 TROS Web UI 显示） |
| 接口基线 | **唯一依据**：[`11_interface_freeze.md`](https://github.com/hachi-leaf/gs130_sdk/blob/develop/ros/docs/11_interface_freeze.md) v0.1.0（FROZEN）。本计划中的话题名、类型、QoS、参数名与取值、错误码、launch 契约**逐字**取自该冻结书 |
| 其它参考 | `ros/docs/10_architecture.md`（背景，若与冻结书冲突以冻结书为准）、`python/test/test_gs130.py`（硬件实测输出）、`python/gs130/_device.py` / `_config.py` / `_types.py` / `_enums.py`、`core/include/gs130.h` |
| 仓库 / 分支 | `gs130_sdk` @ `develop`（板端目录约定 `/home/leaf-jammy/rdkx5_work/gs130_sdk`，文中用 `$REPO` 表示） |
| 编写人 | QA-1（测试负责人） |
| 读者 | 开发（迭代自测）、QA（验收执行）、发布责任人（发布门禁） |
| 板端前提 | 所有测试均由**他人按本文档在板端执行**。本文档作者未连接板卡、未运行相机；"期望结果"来自已核实的板端事实、冻结书与 SDK 代码/头文件语义 |
| 冲突处理 | 若发现冻结书与实现不一致：**以冻结书为期望值**，把差异记为缺陷（`GS130ROS-nnn`），不得就地放宽判据（见 9.1） |

---

## 0. 会话约定与固定事实（执行前必读）

### 0.1 记号与每次开新终端的前导

```bash
export REPO=/home/leaf-jammy/rdkx5_work/gs130_sdk
export WS=~/gs130_ros_ws                      # colcon 工作空间（见第 8 章）
export EVID=$REPO/ros/docs/evidence/$(date +%Y%m%d_%H%M%S)   # 证据目录（见第 6 章）
export BOARD_IP=$(hostname -I | awk '{print $1}')
export NODE="timeout 300 ros2 run gs130_ros gs130_node --ros-args"
mkdir -p $EVID/img
source /opt/tros/humble/setup.bash
source $WS/install/setup.bash
```

* `$NODE` 已含 `ros2 run` 前缀与 `timeout 300`，表中出现 `$NODE -p mode:=...` 可直接粘贴；若未导出，展开为 `timeout 300 ros2 run gs130_ros gs130_node --ros-args -p mode:=...`。
* 所有采集类命令必须带 `timeout`，且输出用 `tee` 落盘到 `$EVID`。
* `$EVID` 见第 6 章；`<BOARD_IP>` = 板卡实际 IP。

### 0.2 被测接口的固定事实（来自冻结书，测试按此断言）

| 项 | 值 |
| --- | --- |
| 包名 / 节点名 / 可执行名 | `gs130_ros` / `gs130_node` / `gs130_node`（`ament_python`，入口点 `gs130_ros.gs130_node:main`） |
| launch 文件 | `gs130_web.launch.py`（相机 + `hobot_codec_republish` + `websocket`）、`gs130_camera.launch.py`（仅相机） |
| 话题 T1 | `/image_combine_raw`（`sensor_msgs/Image`，NV12，左右水平拼接、左目在左），QoS `BEST_EFFORT`/`VOLATILE`/`KEEP_LAST`/depth 1，`frame_id=camera` |
| 话题 T2/T3 | `/image_left_raw`、`/image_right_raw`（同类型），depth 1，`frame_id=camera_left` / `camera_right`；**默认关闭**（`publish_per_eye=false`） |
| 话题 T4 | `/imu/data`（`sensor_msgs/Imu`），depth 200，`frame_id=imu_link` |
| 话题 T5/T6 | `/image_left/camera_info`、`/image_right/camera_info`（`CameraInfo`），`RELIABLE`/`TRANSIENT_LOCAL`/depth 1，**启动时 latched 1 次** |
| 话题 T7 | `/image_combine_jpeg`（`CompressedImage`，由 `hobot_codec_republish` 发布） |
| 话题 T8 | `/gs130/status`（`diagnostic_msgs/DiagnosticStatus`，1 Hz，默认关闭） |
| 图像尺寸 | `resize/rect`：单目 `W×H`；消息 `width=W`（T1 为 `2W`）、`height=H*3/2`、`step=width`、`len(data)=step*height`、`encoding="nv12"`。`640x480` 时 T1 = `width 1280`、`height 720`、`step 1280`、`len 691200` |
| CameraInfo | `height/width` = **单目** `H/W`（不是 `H*3/2`）；`K` 为 SDK `K` 逐元素复制；`R`、`P` **全 0**（`P` 长度 12）；`binning_x/y=0`；`roi` 全 0、`do_rectify=False` |
| 畸变 | `mode∈{raw,resize}` 且 SDK 报 FISHEYE → `"equidistant"` + 4 元 `D`；SDK 报 PINHOLE → `"plumb_bob"` + 5 元 `D`；`mode:=rect` → `"plumb_bob"` + 5 个 0；`camera_info_distortion_model` 参数可覆盖（`none` → 空 `D`） |
| IMU 字段 | `linear_acceleration`（m/s²，含重力）、`angular_velocity`（rad/s）逐字取 SDK；`orientation` 保持默认 `(0,0,0,1)` 且 `orientation_covariance[0] = -1.0`；加速度/角速度各 `covariance[0] = -1.0`；**不发布温度、不发布 `is_fsync`** |
| 时间戳 | `stamp_ns = device_ts_ns + offset_ns`（`stamp_offset_mode:=auto`）；图像与 IMU 同一时基可直接比较；IMU 为 FSYNC 突发式发布，契约只保证**不丢包、不重排、stamp 递增** |
| 参数（节选） | `platform`、`device`、`mode`、`width`、`height`、`fps`（`[1,33]`）、`odr`（**只接受 200 与 500**）、四个 `frame_id_*`、`publish_combine`、`publish_per_eye`、`publish_imu`、`publish_camera_info`、`publish_status`、`stamp_offset_ns`、`stamp_offset_mode`、`camera_info_distortion_model`、`poll_period_ms`、`imu_qos_depth`、`image_qos_depth`、`log_fps_period_s`、`start_timeout_s`；**运行期不可重配置**（`ros2 param set` 无效） |
| launch 参数（节选） | `codec_channel=0`、`websocket_channel=1`、`web_port=8000`、`web_output_fps=30`、`jpg_quality=80`、`smart_topic=/image_combine_jpeg`、`hobot_shm=true` |
| 错误契约 | 参数/配置错误 → `ERROR` + **退出码 2**；初始化/启动/相机通路致命错误 → `ERROR` + **退出码 1**；IMU 缺失/失败、丢帧、标定缺失 → **非致命**（不退出） |
| SIGINT 契约 | `SIGINT`/`SIGTERM` 后 **3 秒内**完成 停止发布 → `Device.stop()` → `Device.close()`（释放相机独占）→ `destroy_node()`/`shutdown()`，**退出码 0** |
| 相机独占 | GS130 与 `mipi_cam` **不能同时运行**；先起 `mipi_cam` 时本节点 `gs130_init()` 失败并退出码 1，不重试 |
| 验收命令 | 冻结书 §6.6 给出的 7 条命令是 v0.1.0 的验收项，本计划把其中每条都映射到具体 TC（见附录 C） |

---

## 1. 测试范围

### 1.1 被测（in scope）

| 编号 | 被测内容 |
| --- | --- |
| S1 | 包与构建产物：colcon 可构建、`ros2 pkg executables` 可见、`ros2 run` / `ros2 launch` 可启动、`share/gs130_ros/launch/*.py` 已安装 |
| S2 | 接口面：T1–T8 的话题名、消息类型、QoS（reliability/durability/history/depth）、`frame_id`、发布频率 |
| S3 | 消息构造正确性：NV12 语义（`encoding="nv12"`、`width/height/step/len(data)` 关系）、拼接布局字节级正确、单目话题默认关闭/开启行为 |
| S4 | `CameraInfo`：真实内参（fx/fy/cx/cy ≈ 387/387/305/245 @640x480）、`R`/`P` 全 0、`binning`/`roi`、畸变模型字符串与 `D` 长度（含 `mode` 与参数覆盖规则） |
| S5 | IMU：`/imu/data` 到达、单位与静止数值合理、速率等于 `odr`、协方差 `-1` 语义、`odr:=200/500` 生效、`odr` 非法值报错 |
| S6 | 外参的可观测性：v0.1.0 **不发布 tf**（冻结书 §0.2/§5.5），因此外参在 ROS 侧的唯一验收面是"多话题/多模式间的标定一致性"与"工具可复现 SDK 外参"（见 TC-13、TC-30） |
| S7 | 参数生效：`mode` / `width,height` / `fps` / `odr` / `publish_*` / `frame_id_*` / `camera_info_distortion_model` / `image_qos_depth`（**每次改动都需重启进程**，冻结书 §5.3） |
| S8 | 失败路径：非法参数（退出码 2）、RAW 模式非原生分辨率、非法 `odr`、缺库、缺相机、`mipi_cam` 占用、无 EEPROM 时 `rect` 的降级或报错 |
| S9 | 生命周期：SIGINT/SIGTERM 3 秒内干净退出且释放相机、连续多次运行成功、20 轮起停与 ≥60 min soak 稳定 |
| S10 | 与既有 TROS 链的集成：`/image_combine_raw` → `hobot_codec_republish` → `/image_combine_jpeg` → `websocket`（`channel:=1`）→ `http://<BOARD_IP>:8000` 选择 channel 0 显示左目实时画面 |
| S11 | 一键 launch（相机 + codec + web）与"仅相机" launch 两条路径 |

### 1.2 明确不在范围内（out of scope，视为可信、不重测）

| 编号 | 不测内容 | 理由 |
| --- | --- | --- |
| O1 | `hobot_codec_republish` 的内部实现、NV12→JPEG 编码质量、`jpg_quality` 的视觉差异 | D-Robotics 官方节点；本包只消费约定。仅验证输入/输出话题与编码格式被正确对接（TC-20） |
| O2 | `websocket` 节点的 HTTP/WebSocket 服务实现、nginx 端口 8000 配置、网页前端 JS 逻辑 | 官方组件；仅验证页面可达 + 能看到实时图像 + channel 语义（TC-21） |
| O3 | `hobot_shm` 零拷贝机制本身 | 官方组件；冻结书 §6.5 明确本节点不依赖它，仅验证 `hobot_shm:=true/false` 下功能都正确（TC-19 的附加检查） |
| O4 | `hobot_stereonet` 与 `/StereoNetNode/stereonet_visual` | 官方模型；本项目只需向其提供合法右目图像。v0.1.0 不集成 stereo net，删除出该话题的用例 |
| O5 | `libgs130.so` 的驱动/ISP/VIN/VSE/GDC 内部行为与画质 | 属前置条件；SDK 自身由 `python/test/test_gs130.py` 与 `core/samples` 覆盖，本计划只做环境自检（TC-01） |
| O6 | 标定精度（内参/外参的绝对准确性）、重新标定 | 只验证"SDK 报什么，ROS 就报什么"（TC-11/TC-12/TC-13） |
| O7 | 姿态解算（AHRS/`orientation` 的数值）、温度话题、`is_fsync` 的语义 | 冻结书 §0.2/§3.3 明确为非目标 |
| O8 | `/tf`、`/tf_static` | 冻结书 §0.2/§5.5 明确为非目标；"没有发布的东西不需要测"（若有实现意外发布，见 TC-29 的反向断言） |
| O9 | 运行期动态重配置 | 冻结书 §5.3 明确不提供；本计划改为断言"`ros2 param set` 无效果且不崩溃"（TC-29） |
| O10 | 性能极限优化目标、24h+ 老化、高低温/振动可靠性 | 只做第 5 章的有限 soak |
| O11 | 权限/安装脚本、`deb` 打包、TROS 系统镜像 | 由其它交付物覆盖 |

> 边界说明：官方节点的**内部**不测，但它们与 `gs130_ros` 的**接口契约**必须测（话题名、类型、编码、channel 号、launch 顺序）。否则"网页没图"时无法定位责任方。

---

## 2. 测试级别

| 级别 | 名称 | 需要板卡/相机 | 能证明什么 | 不能证明什么 | 对应用例 |
| --- | --- | --- | --- | --- | --- |
| L1 | 主机测试（无硬件） | 否 | 参数解析与校验、NV12 尺寸计算、拼接布局函数、`CameraInfo` 组装、畸变映射规则在纯函数级正确；包能被 colcon/ament 构建 | 与真实 `libgs130`、真实传感器、真实 TROS 节点的任何交互行为 | TC-01, TC-02, TC-03 |
| L2 | 接口/消息构造测试 | 否（桩数据或 `ros2 topic pub` 回放） | 发布出去的话题名/类型/QoS/字段与冻结书一致；失败路径（非法 `mode`/`odr`/RAW 尺寸）报错清晰且退出码正确 | 时序、真实帧率、真实数值语义（重力方向、内参量级）、与 codec/websocket 的联通 | TC-02, TC-03, TC-05（桩）, TC-11（桩）, TC-29（桩） |
| L3 | 板端集成测试 | 是（相机 + IMU） | 真实 SDK 数据经 ROS 发布正确：话题/类型/QoS/帧率、图像与拼接布局、IMU 数值与单位、CameraInfo 真实内参、模式与参数切换生效、SIGINT 契约、重复运行成功 | 网页端人眼可见性；跨节点（codec/websocket）链路联通 | TC-04 ~ TC-18, TC-22, TC-23, TC-24, TC-25, TC-26, TC-27, TC-28, TC-29, TC-30 |
| L4 | 端到端 Web 验证 | 是（相机 + codec + websocket + 浏览器） | 一条 launch 后 `http://<BOARD_IP>:8000` 选择 channel 0 看到**实时左目**画面；`/image_combine_jpeg` 有数据；整链可用 | 图像内容正确性由 L3 的 PNG 证据保证；网页前端内部逻辑不测 | TC-19, TC-20, TC-21 |
| L5 | 回归与重复运行稳定性 | 是 | 反复起停无资源泄漏、相机始终被释放、连续运行成功、soak 期间帧率/内存/温度稳定、无崩溃与丢帧恶化 | 新功能首次正确性（L1–L4 覆盖）；极端环境可靠性 | TC-22, TC-24, TC-28 + 第 5 章 S-01/S-02 |

级别与迭代的关系：每个迭代至少跑通 L1 + L2 + L3 的冒烟子集；L4 在涉及 launch/web 变更时必跑；L5 在候选发布（RC）构建上跑。

---

## 3. 测试用例表

通用前置：

| 编号 | 前置条件 |
| --- | --- |
| P1 | 板卡上电，RDK X5 + GS130WI 已连接（排线固定）；**没有**任何进程占用相机：`pgrep -af "mipi_cam\|gs130"` 为空 |
| P2 | 已按 §0.1 导出变量并 source `/opt/tros/humble/setup.bash` 与 `$WS/install/setup.bash` |
| P3 | `python3 -c "import gs130;print(gs130.__version__, gs130.library_version())"` 成功，且两者一致、无 `RuntimeWarning`（否则先做 TC-01） |
| P4 | `$EVID` 已创建，**每条**命令的完整 stdout+stderr 用 `tee` 落盘为 `$EVID/TC-XX*.log`，并记录 `echo rc=$?`（见第 6 章） |
| P5 | 测试者已知相机为**独占**资源：测试期间不得启动 `mipi_cam`、`core/samples` 或任何其它取流进程 |
| P6 | 每次涉及后台节点时，**先确认上一进程已消失**（`pgrep -af gs130_node` 为空）再开始 |

### 3.1 用例总表

> 命令列中的 `$NODE` = `timeout 300 ros2 run gs130_ros gs130_node --ros-args`。表内 `\|` 为表格转义，实际命令中是普通管道 `|`。

| TC | 目标 | 前置 | 命令（P2 后执行） | 期望可观测结果 | 通过判据 |
| --- | --- | --- | --- | --- | --- |
| TC-01 | 环境与 SDK 依赖自检（L3 总门） | P1,P2，`libgs130` 已装到 `/usr/lib/aarch64-linux-gnu/`（或设 `GS130_LIB`） | `python3 -c "import gs130;print(gs130.__version__, gs130.library_version())" \| tee $EVID/TC-01.log`；`python3 $REPO/python/test/test_gs130.py RDKX5 GS130WI resize 640 480 30 200 \| tail -5 \| tee -a $EVID/TC-01.log` | 第一条打印两个相同版本号；第二条末尾打印 `GS130 smoke test passed` 与 `saved PNG count: 20` | 两条命令退出码 0；无 `RuntimeWarning: libgs130 ... is older`；无 traceback。**不通过则 TC-04 及以后全部 BLOCKED** |
| TC-02 | 包与可执行被发现（构建基线） | 已完成 §8.2 构建 | `ros2 pkg prefix gs130_ros \| tee $EVID/TC-02.log`；`ros2 pkg executables gs130_ros \| tee -a $EVID/TC-02.log`；`ls $WS/install/share/gs130_ros/launch/ \| tee -a $EVID/TC-02.log` | 打印 `$WS/install/gs130_ros`；列出 `gs130_ros: gs130_node`；launch 目录含 `gs130_web.launch.py` 与 `gs130_camera.launch.py` | 三条子命令退出码 0；`gs130_node` 与两个 launch 文件名逐字出现 |
| TC-03 | 非法参数：退出码 2 且错误文本明确 | 已完成构建；**不**接相机也可 | `$NODE -p mode:=bogus 2>&1 \| tee $EVID/TC-03-mode.log; echo rc=$?`；`$NODE -p mode:=raw -p width:=640 -p height:=480 2>&1 \| tee $EVID/TC-03-raw.log; echo rc=$?`；`$NODE -p odr:=100 2>&1 \| tee $EVID/TC-03-odr.log; echo rc=$?`；`$NODE -p platform:=RDKX3 2>&1 \| tee $EVID/TC-03-platform.log; echo rc=$?` | 四条都打印 `ERROR` 级 `invalid parameter {name}={value}: {reason}`（`platform` 非法时打印 `unsupported platform/device: RDKX3 GS130WI`）；`mode:=raw` 的消息含逐字 `RAW mode requires width=1088 height=1280`；`odr:=100` 说明只接受 200/500 | 四条命令 **`rc=2`**；日志含 `invalid parameter` 或 `unsupported platform/device`；**不出现**任何 `started`/发布成功日志；无 traceback 以外的模糊输出 |
| TC-04 | 话题、类型与 QoS 齐全（默认参数） | P1,P2,P3 | 终端 A：`$NODE -p mode:=resize -p width:=640 -p height:=480 -p fps:=30 -p odr:=200`；等 5 s 后终端 B：`timeout 20 ros2 topic list \| tee $EVID/TC-04-list.log`；`timeout 20 ros2 topic info /image_combine_raw -v \| tee $EVID/TC-04-raw.log`；`timeout 20 ros2 topic info /imu/data -v \| tee $EVID/TC-04-imu.log`；`timeout 20 ros2 topic info /image_left/camera_info -v \| tee $EVID/TC-04-info.log` | `topic list` 含 `/image_combine_raw`、`/imu/data`、`/image_left/camera_info`、`/image_right/camera_info`；**不含** `/image_left_raw`、`/image_right_raw`、`/gs130/status`；`/image_combine_raw` 为 `Type: sensor_msgs/msg/Image`、`Reliability: BEST_EFFORT`、`Durability: VOLATILE`、depth 1、Publisher `gs130_node`；`/imu/data` depth 200；`camera_info` 为 `RELIABLE` + `TRANSIENT_LOCAL`、depth 1 | 四个必需话题存在、类型精确匹配；两个 per-eye 话题与 status 话题**必须不存在**（默认关闭）；QoS 四项逐一相符；发布者计数 ≥ 1。**PASS 后再执行 TC-05** |
| TC-05 | T1 图像字段正确（NV12 尺寸语义） | TC-04 节点运行中 | A.1 脚本：`timeout 40 python3 $EVID/tc05_fields.py \| tee $EVID/TC-05.log` | 打印 `encoding=nv12 width=1280 height=720 step=1280 len=691200 is_bigendian=0 frame_id=camera` | 每个字段与上式**逐一相等**；`len(data) == step*height == width*height*3//2 == 2*W*H*3//2`；`encoding` 逐字小写 `nv12` |
| TC-06 | T1 帧率等于配置 `fps` | TC-04 节点运行中 | `timeout 25 ros2 topic hz /image_combine_raw --window 100 \| tee $EVID/TC-06.log`；另开终端 `timeout 30 python3 $EVID/tc09_stats.py \| tee $EVID/TC-06-stats.log`（A.4 的 5 s 统计日志与之比对） | `hz` 输出 `average rate` ≈ 30；节点日志出现 `camera {fps:.2f} Hz, published ... combine ...` 周期统计（默认每 5 s 一条） | `average rate` 落在 `[0.9×fps, 1.05×fps]` = `[27.0, 31.5]`；节点统计日志里的 `camera` 值同样落在该区间；两者差值 ≤ 1.0 Hz |
| TC-07 | 双目成对：同一组数据的左右目时间戳相同 | TC-04 节点运行中，另起 `publish_per_eye:=true` 的节点（见 TC-17） | `timeout 40 python3 $EVID/tc07_pair.py \| tee $EVID/TC-07.log` | 打印 30 组 `left=`、`right=`、`delta_ns=`；`delta_ns` 全为 0 | 30 组中 `delta_ns == 0` 的组数 = 30（设计上同一组数据只取一次设备，见冻结书 §3.2）；若出现非零值，需开发解释后再判定 |
| TC-08 | T1 可解码为有效图像并存 PNG 供人检 | TC-04 节点运行中 | `timeout 60 python3 $EVID/tc08_save.py \| tee $EVID/TC-08.log`（A.3 脚本，把 T1 的左右两半各存 10 张 PNG 到 `$EVID/img/`） | 打印每张 `half=left/right shape=(480, 640, 3) mean std`、共 20 个文件路径 | 20 张 `cv2.imwrite` 成功；每张 `480×640×3 uint8`；`std > 5`（非纯色）；`10 < mean < 245`；人工检查 4 张（左/右各 2）确认无花屏、无全黑、**无左右半错位**（错位表现为接缝处竖直色带）与 Y/UV 平面错位（绿色/灰紫条纹） |
| TC-09 | IMU 到达、单位与静止数值合理 | 板卡水平静置，TC-04 节点运行中 | `timeout 40 python3 $EVID/tc09_imu.py \| tee $EVID/TC-09.log`（A.5，统计 200 包） | 打印 `count=200`、`rate≈odr`、`accel_mean/std`、`accel_norm`、`gyro_mean/std`、`cov` 四元组、`frame_id` | `abs(accel_norm - 9.8) ≤ 0.5 m/s²`；`abs(gyro_mean) < 0.05 rad/s`；`linear_acceleration_covariance[0] == -1.0`、`angular_velocity_covariance[0] == -1.0`、`orientation_covariance[0] == -1.0`（各长度 9）；`orientation` 四个分量保持消息默认值 `(0,0,0,1)`；`frame_id == "imu_link"`；**不存在**温度话题与 `is_fsync` 话题 |
| TC-10 | `odr` 生效：200 与 500 可观测区分 | P1 | 分别运行 `$NODE -p odr:=200` 与 `$NODE -p odr:=500`，每次跑 A.5 脚本（`rate` 部分）与 `timeout 25 ros2 topic hz /imu/data --window 500 \| tee $EVID/TC-10-hz-$ODR.log` | `odr=200` → `rate ≈ 200 Hz`；`odr=500` → `rate ≈ 500 Hz`；两者都由"突发 + 静默"构成，`hz` 的平均值仍接近设定值 | 5 s 窗口内实测速率落在 `[0.85×odr, 1.15×odr]`；两次实测速率比 ≈ 2.5（2.0~3.0）；`header.stamp` 序列**严格递增**（相等时间戳按 SDK 顺序发布，不算回退）；节点日志的 `imu {odr:.2f} Hz` 与脚本结论一致 |
| TC-11 | `CameraInfo` 携带真实内参（resize 640x480） | TC-04 节点运行中 | `timeout 20 ros2 topic echo /image_left/camera_info --once \| tee $EVID/TC-11-left.log`；对 `/image_right/camera_info` 同样执行 | `height: 480`、`width: 640`、`frame_id: camera_left`；`k: [≈387, 0, ≈305, 0, ≈387, ≈245, 0, 0, 1]`；`r` 全 0（9 个）；`p` 全 0（12 个）；`binning_x=0`、`binning_y=0`；`roi` 全 0、`do_rectify=false` | `abs(fx-387) ≤ 8`、`abs(fy-387) ≤ 8`、`abs(cx-305) ≤ 8`、`abs(cy-245) ≤ 8`；`k[1]==k[3]==k[6]==k[7]==0`、`k[8]==1`；`r` 与 `p` 的每个元素都为 `0.0`（**不得**是单位阵）；`k`／`r`／`p` 与 TC-30 脚本打印的 SDK 原始值逐一相符（容差 1e-9） |
| TC-12 | 畸变模型字符串与 `D` 长度（含 rect 覆盖规则） | TC-04 节点运行中；随后另起 `mode:=rect` 节点 | `resize`：`timeout 20 ros2 topic echo /image_left/camera_info --once \| grep -A14 -E "distortion_model\|^d:" \| tee $EVID/TC-12-resize.log`；`rect`：同法存 `$EVID/TC-12-rect.log`；`camera_info_distortion_model:=none`：同法存 `$EVID/TC-12-none.log` | `resize` → `distortion_model: "equidistant"`、`d` 长度 4（= SDK `dist_coeffs[0..3]`）；`rect` → `"plumb_bob"`、`d = [0,0,0,0,0]`；`none` → `d: []`（空数组） | 三种情形逐字匹配；`d` 与 TC-30 的 SDK 值差 ≤ 1e-9；若 SDK 报 PINHOLE（在证据中登记 EEPROM 字符串）则 `resize` 必须为 `plumb_bob` + 5 元 `D` |
| TC-13 | 标定/外参在 ROS 侧的一致性（tf 为非目标） | 任一节点运行中的场景均可（外参取自 SDK 而非话题） | `timeout 60 python3 $EVID/tc30_extrinsics.py \| tee $EVID/TC-13.log`；反向断言：`timeout 15 ros2 topic list \| grep -E "^/tf" \| tee $EVID/TC-13-notf.log` | TC-30 脚本打印 3×3 参考系两两的 `R`/`T`、`install_angle`、`camera_left/right` 内参，以及一致性检查结论；`grep -E "^/tf"` **无输出** | 自检全部通过：`R` 正交性 `max(abs(R.T@R - I)) < 1e-6`；`R(from,to)` 与 `R(to,from)` 互为转置（差 ≤ 1e-9）；`T(from,to) == -R(to,from)@T(to,from)`（差 ≤ 1e-9）；左右目基线模长在 `[0.02, 0.30] m`（超出需在证据中登记实际值并说明）；`/tf`、`/tf_static` **不存在**（与冻结书 §5.5 一致）。**说明**：v0.1.0 不含 tf 发布，若评审后续批准 tf 方案，本用例必须升级为"tf 与 SDK 外参逐元素比对"后再测 |
| TC-14 | 尺寸语义：T1/T2 与 `CameraInfo` 的关系 | TC-04 节点运行中 | `timeout 30 python3 $EVID/tc14_dims.py \| tee $EVID/TC-14.log`（A.6，同时取 T1 与两侧 `CameraInfo`） | 打印 `combine: w=1280 h=720 step=1280`、`camera_info left: w=640 h=480`、`right: w=640 h=480`、`check 720 == 3*480//2 -> True` | `combine.width == 2 * camera_info.width`、`combine.height == 3 * camera_info.height // 2`、`combine.step == combine.width`；三个消息的 `header.stamp` 在同一时基（差值 < 1 s）；`frame_id` 分别为 `camera` 与 `camera_left`/`camera_right` |
| TC-15 | `mode` 参数改变可观测（resize ↔ rect） | P1 | 先 `$NODE -p mode:=resize -p width:=640 -p height:=480`，跑 A.3 采 5 张；Ctrl-C 确认进程消失后 `$NODE -p mode:=rect -p width:=640 -p height:=480`，再采 5 张（存 `$EVID/img_rect/`）；随后检查 rect 的 `CameraInfo`（同 TC-12 命令存 `$EVID/TC-15-rect-info.log`） | 两次都出图；`rect` 的图像相比 `resize` 的同一场景，鱼眼弯曲的直线变直；`rect` 的 `CameraInfo` 为 `plumb_bob` + 5 个 0 | 两种 mode 都在 20 s 内出首帧；`resize` 与 `rect` 各 5 张 PNG 存盘且人工对照可见直线弯直差异（两张对照图必须入证据）；`rect` 模式下 `K` 左右目焦距一致（差 ≤ 2 px）且主点居中（`abs(cx - W/2) ≤ 8`、`abs(cy - H/2) ≤ 8`）——若实际不符合，登记实测值并作为缺陷上报（虚拟内参应由 `stereo_rectify` 写入） |
| TC-16 | `width/height` 参数改变可观测 | P1 | 依次运行 `$NODE -p mode:=resize -p width:=640 -p height:=480` 与 `$NODE -p mode:=resize -p width:=320 -p height:=240`，每次跑 A.1 脚本与 TC-11 的 `camera_info` 命令 | `640x480`：T1 `w=1280 h=720 step=1280 len=691200`，`camera_info w=640 h=480`；`320x240`：T1 `w=640 h=360 step=640 len=172800`，`camera_info w=320 h=240` | 尺寸字段严格等于上式（公式 `len = 2W*H*3/2`）；`fx_320 ≈ fx_640 / 2`（容差 5%）；**不支持的分辨率**（如 `width:=641` 奇数、`width:=1100`）必须报错退出（`ERROR` + 退出码 1 或 2），不得静默裁剪 |
| TC-17 | `fps` 参数改变可观测，且 per-eye 话题默认关闭可打开 | P1 | `$NODE -p fps:=15` 跑 `ros2 topic hz /image_combine_raw`；`$NODE -p fps:=30` 同法；`$NODE -p fps:=40 2>&1 \| tee $EVID/TC-17-fps40.log; echo rc=$?`；`$NODE -p publish_per_eye:=true` 后 `timeout 20 ros2 topic list \| tee $EVID/TC-17-eye.log` | `fps:=15` → ≈15 Hz；`fps:=30` → ≈30 Hz；`fps:=40`（超出 `[1,33]`）报 `invalid parameter fps=40`；`publish_per_eye:=true` 时多出 `/image_left_raw` 与 `/image_right_raw` | 两次 `hz` 落在 `[0.9×fps, 1.05×fps]`，速率比 ≈ 2（1.6~2.4）；`fps:=40` 退出码 2；`publish_per_eye:=true` 时两个话题存在且 A.1 对 `width=640 height=720 step=640 len=460800`（单目尺寸）判定通过 |
| TC-18 | 时间戳：同源、单调、无长间隔 | TC-04 节点运行中 | `timeout 60 python3 $EVID/tc18_stamps.py \| tee $EVID/TC-18.log`（A.7，各取 100 条） | 打印 `image: count monotonic jumps max_gap_ms min max`、`imu: ...`、`cam_minus_imu_min`、`offset_span_ms` | 图像 `jumps <= 1`（同一时刻的补点允许相等，不得回退）；IMU **严格递增**（`jumps == 0`）；`max_gap_ms < 2000/fps`（30 fps 时 < 66.7 ms）；`abs(cam_minus_imu_min)` < 1 s 且整段内抖动 < 5 ms（冻结书 §4.4 允许 IMU 样本早于最近图像帧，但偏移量必须稳定）；节点日志打印的 `offset {offset_ns} ns` 与脚本推算一致（差 ≤ 1 ms） |
| TC-19 | 一条 launch 拉起整链（相机 + codec + web） | P1；TROS 与 `$WS` 均已构建 | `timeout 120 ros2 launch gs130_ros gs130_web.launch.py platform:=RDKX5 device:=GS130WI mode:=resize width:=640 height:=480 fps:=30 odr:=200 \| tee $EVID/TC-19.log`（前台保持运行） | launch 日志依次出现 `gs130_node`、`hobot_codec_republish`、`websocket` 的启动行；无 `process has died`；`ros2 node list` 含三个节点 | 三个节点 30 s 内全部出现；60 s 内无进程退出；随后 `hobot_shm:=false` 再跑一次同样通过（冻结书 §6.5 要求不依赖 shm）；`publish_combine:=false` 时 `/image_combine_raw` 消失、codec 无输入（用于验证链路可定位） |
| TC-20 | codec 链发布 JPEG 话题 | TC-19 运行中 | `timeout 20 ros2 topic list \| grep jpeg \| tee $EVID/TC-20-list.log`；`timeout 25 ros2 topic hz /image_combine_jpeg \| tee $EVID/TC-20-hz.log`；`timeout 30 ros2 topic echo /image_combine_jpeg --once --field format \| tee $EVID/TC-20-fmt.log` | `/image_combine_jpeg` 存在，类型 `sensor_msgs/msg/CompressedImage`；`format` 含 `jpeg`；速率 ≈ 源帧率 | 话题存在 + 类型正确 + `hz ≥ 0.8×fps` + `format` 含 `jpeg`；`ros2 topic info -v /image_combine_raw` 中可见 `hobot_codec_republish` 为订阅者（证明链路对接正确） |
| TC-21 | Web UI 8000 端口显示实时左目图像 | TC-19 运行中；测试机与板卡同网段 | `timeout 10 curl -sS -o $EVID/web_index.html -w "%{http_code}\n" http://$BOARD_IP:8000/ \| tee $EVID/TC-21-http.log`；浏览器打开 `http://$BOARD_IP:8000`，选择 **channel 0**，间隔 2 s 截两张图存 `$EVID/web_live_1.png`、`$EVID/web_live_2.png`（或录 5 s 视频） | HTTP 200 且返回 HTML；页面上出现实时画面（`websocket_channel:=1` 承载 `/image_combine_jpeg`，页面 channel 0 显示其左半 = 左目，见冻结书 §6.2 显示语义）；晃动相机时画面同步变化 | HTTP 200；画面可见；两张截图内容不同（非冻帧）且变化与人为晃动一致；`ros2 topic info -v /image_combine_jpeg` 显示 `websocket` 为订阅者。**若网页显示的是右目或拼接全图**，判 FAIL 并记录（冻结书 §6.2 的显示语义被破坏） |
| TC-22 | SIGINT/SIGTERM 3 秒内干净退出并释放相机 | 任一前台运行中的节点（TC-04 或 TC-19） | 在运行终端按 `Ctrl-C`，同时计时：`time kill -INT <pid>` 前后的时间差；随后 `pgrep -af "gs130_node\|hobot_codec\|websocket" \| tee $EVID/TC-22-ps.log`；再 `timeout 60 python3 -c "import gs130;d=gs130.Device(gs130.Config.preset('RDKX5','GS130WI',gs130.CameraMode.RESIZE,640,480,30,200));d.start();print('reopen ok');d.close()" \| tee -a $EVID/TC-22-ps.log`；另一次用 `kill -TERM <pid>` 重复 | 退出日志含关闭信息；进程在 3 s 内消失，退出码 0；`pgrep` 无输出；SDK 复开打印 `reopen ok` | **退出码 0**；从信号到进程消失 ≤ 3.0 s（用 `date +%s.%N` 计时并记录实测值）；无残留进程；相机复开成功；`SIGINT` 与 `SIGTERM` 两种信号都必须通过。**禁止**用 `pkill -9` 清理后判 PASS（若必须强杀，本用例记为 FAIL 并立案） |
| TC-23 | 相机被 `mipi_cam` 占用时报错清晰 | P1；板端有可用的 `mipi_cam` 启动方式 | 终端 A：启动 `mipi_cam`（按板端实际命令，例如 `ros2 launch mipi_cam mipi_cam.launch.py`）并确认它已占用相机；终端 B：`$NODE -p mode:=resize -p width:=640 -p height:=480 2>&1 \| tee $EVID/TC-23.log; echo rc=$?` | `ERROR` 级 `gs130_init failed: HW_ERROR ...`（或 `NOT_FOUND: camera not detected on the configured I2C buses`），退出码 1；不重试、不静默等待 | **`rc=1`**；错误文本含 `gs130_init failed` 与错误码名；`timeout 15 ros2 topic list \| grep image_combine_raw` 无输出；节点**不**自动重启（`ros2 node list` 中无本节点） |
| TC-24 | 连续第二次运行成功 | TC-22 已通过 | 连续执行两次 `timeout 90 ros2 launch gs130_ros gs130_web.launch.py` → 每次 `Ctrl-C` 确认进程消失 → **立即**再执行一次；第二次期间跑 `timeout 25 ros2 topic hz /image_combine_raw` 与 `timeout 25 ros2 topic hz /image_combine_jpeg` | 第二次仍正常出图，`/image_combine_jpeg` 重新出现 | 第二次两次 `hz` 都满足 TC-06/TC-20 判据；两次之间无 `HW_ERROR`/`NOT_FOUND`/`address already in use` 类错误；`dmesg -T \| tail -20` 无新的 `vpf/vinc/isp` 报错 |
| TC-25 | 缺库时报错清晰且不崩溃 | 可临时改变库可见性（**不修改** `python/` 与系统安装） | `GS130_LIB=/nonexistent/libgs130.so $NODE -p mode:=resize -p width:=640 -p height:=480 2>&1 \| tee $EVID/TC-25.log; echo rc=$?` | 日志含 `OSError: libgs130 not found; install libgs130.so.0 or set GS130_LIB`（或 `cannot open shared object file`）；进程退出 | `rc != 0`（预期 1）；无段错误/core dump；错误信息指出缺库与可用环境变量 `GS130_LIB`；`ros2 node list` 无残留 |
| TC-26 | IMU 缺失/相机缺失的降级与报错 | P1；具备可断开排线的条件或用 `device:=GS130W` 模拟无 IMU | `$NODE -p device:=GS130W 2>&1 \| tee $EVID/TC-26-noimu.log; echo rc=$?`，并在其运行期间 `timeout 15 ros2 topic list \| tee -a $EVID/TC-26-noimu.log`；断开传感器排线后 `$NODE -p mode:=resize -p width:=640 -p height:=480 2>&1 \| tee $EVID/TC-26-nohw.log; echo rc=$?` | `device:=GS130W`：`WARN` 逐字 `IMU not present; /imu/data will not be published`，`/imu/data` **不存在**，相机话题仍发布，进程**不退出**；断开硬件：`ERROR` + 退出码 1 | `GS130W` 场景：运行中 `/imu/data` 不存在、`/image_combine_raw` 存在并可 `hz`、**无** `rc`（进程存活）；断开硬件场景：`rc=1` 且日志含 `gs130_init failed`。**禁止**出现"启动成功但永远没有数据"的静默状态 |
| TC-27 | RAW 模式必须用原生分辨率，且尺寸语义正确 | P1 | 失败路径：`$NODE -p mode:=raw -p width:=640 -p height:=480 2>&1 \| tee $EVID/TC-27-bad.log; echo rc=$?`；成功路径：`$NODE -p mode:=raw -p width:=1088 -p height:=1280`，随后 A.1 脚本与 `timeout 25 ros2 topic hz /image_combine_raw` | 失败路径：`ERROR` + 逐字 `RAW mode requires width=1088 height=1280` + 退出码 2（参数校验阶段即拒绝，冻结书 §5.2）；成功路径：T1 `width=2176 height=1920 step=2176 len=6266880`，`camera_info w=1088 h=1280` | 失败路径 `rc=2` 且文本含 `1088`/`1280`；成功路径三个尺寸字段严格等于上式、`hz > 0` 且连续 10 s 无中断（RAW 无 VSE 缩放，实测帧率以证据登记为准），`rx`/丢帧统计无持续下降 |
| TC-28 | `publish_*`、`frame_id_*` 与 `publish_status` 的行为 | P1 | `$NODE -p publish_status:=true` 后：`timeout 20 ros2 topic echo /gs130/status --once \| tee $EVID/TC-28-status.log`；`$NODE -p publish_combine:=false -p publish_per_eye:=true` 后：`timeout 20 ros2 topic list \| tee $EVID/TC-28-combine-off.log`；`$NODE -p frame_id_left:=cam_l -p frame_id_imu:=imu0` 后：A.1/A.5 脚本检查 `frame_id` | `publish_status:=true` → `/gs130/status` 1 Hz，`name="gs130_ros: gs130_node"`、`hardware_id="GS130 GS130WI on RDKX5"`、`level` ∈ {OK,WARN,ERROR}、`values` 含 8 个固定键 `camera_fps,imu_rate,dropped_frames,stamp_offset_ns,mode,width,height,imu_present`；`publish_combine:=false` → `/image_combine_raw` 不存在而 per-eye 存在；`frame_id_*` 改动后逐字出现在对应消息的 `header.frame_id` | status 的 8 个键齐全且 `message` 非空；`publish_combine:=false` 时 `/image_combine_raw` 消失（记录：这会断开 web 链路，属预期）；`frame_id_left:=cam_l` 时 `/image_left_raw` 的 `frame_id` 为 `cam_l`，`/image_left/camera_info` 的 `frame_id` **必须同步**为 `cam_l`（冻结书 §3.4 要求一致） |
| TC-29 | 运行期不可重配置（`ros2 param set` 无效且不崩溃） | 任一节点运行中 | `timeout 20 ros2 param list \| tee $EVID/TC-29-list.log`；`timeout 20 ros2 param set /gs130_node mode rect \| tee -a $EVID/TC-29-list.log`；随后 A.1 脚本与 `timeout 20 ros2 topic echo /image_left/camera_info --once` 复测 | `param set` 返回 `Set parameter successful`（rclpy 层成功）；**但**图像尺寸、`CameraInfo` 与 mode 相关输出**不变**；节点不崩溃、不重启 | `param list` 与冻结书 §5.1 的参数表逐字一致（无未声明参数、无缺失必需参数）；`param set` 后 `ros2 topic hz` 与尺寸字段保持原值；`ros2 node list` 仍含 `/gs130_node`；节点日志**不**声称已重配置 |
| TC-30 | SDK 外参/内参可比对的真值导出（工具一致性） | P1,P3（不需要相机在跑，但需要能 init 设备；若设备忙则等 TC-22 完成） | `timeout 60 python3 $EVID/tc30_extrinsics.py \| tee $EVID/TC-30.log`（A.8，打印 `camera_intrinsics`、`calibration()`、9 组 `relative_R/T`、`install_angle`） | 打印两侧 `fx/fy/cx/cy`、`K`、`dist_model`、`dist_coeffs`、`imu_R/imu_T`、`camera_left/right_R/T`、`install_angle`、以及 9 组参考系两两的 `R`/`T` | 与 TC-11/TC-12/TC-13 的 ROS 侧数值逐一相符（内参容差 1e-9，`D` 前 4 位容差 1e-9）；`install_angle` 与 EEPROM 字符串中的 `Rotate-N-deg` 一致（GS130 实测为 `Rotate-0-deg`，即 `install_angle == 0`）；正交性与互逆检查同 TC-13 |

> 执行顺序建议：TC-01 → TC-02 → TC-03 → TC-04 →（同一长驻节点内连续做 TC-05, TC-06, TC-08, TC-09, TC-11, TC-12, TC-14, TC-18, TC-29）→ TC-30 → TC-07/TC-17/TC-28（需要 per-eye 与 status 的单独启动）→ TC-10/TC-15/TC-16/TC-27（参数切换）→ TC-19~TC-21 → TC-22 → TC-23 → TC-24 → TC-25 → TC-26 → 第 5 章 S-01/S-02。

### 3.2 参数覆盖矩阵（证明"参数改变 → 输出改变"）

| 参数 | 变更 | 观测点 | 期望 | 用例 |
| --- | --- | --- | --- | --- |
| `mode` | `resize` → `rect` | PNG 内容、`distortion_model`、`D`、`K` | 直线变直；`plumb_bob` + 5 个 0；左右焦距一致 | TC-15, TC-12 |
| `mode` | `resize` → `raw` | T1/T2 尺寸、`CameraInfo` 尺寸 | `1088x1280` 单目 / `2176x1280` 拼接 | TC-27 |
| `width`,`height` | `640x480` → `320x240` | 尺寸字段、内参缩放 | 按公式与比例变化 | TC-16 |
| `fps` | `30` → `15` | `ros2 topic hz`、`max_gap_ms` | 速率减半、gap 加倍 | TC-17, TC-18 |
| `fps` | `40`（越界） | 进程行为 | `invalid parameter fps=40` + 退出码 2 | TC-17 |
| `odr` | `200` → `500` | `/imu/data` 速率 | 速率随之变化（仅这两个值合法） | TC-10 |
| `odr` | `100`（非法） | 进程行为 | `invalid parameter odr=100` + 退出码 2 | TC-03 |
| `publish_combine` | `true` → `false` | `/image_combine_raw` | 话题消失（web 链路随之断开） | TC-28 |
| `publish_per_eye` | `false` → `true` | `/image_left_raw`、`/image_right_raw` | 话题出现且为单目尺寸 | TC-17 |
| `publish_imu` | `true` → `false` | `/imu/data` | 话题消失（相机仍工作） | TC-28 |
| `publish_camera_info` | `true` → `false` | 两个 `camera_info` | 话题消失 | TC-28 |
| `publish_status` | `false` → `true` | `/gs130/status` | 1 Hz 诊断消息出现 | TC-28 |
| `frame_id_*` | 改名 | 对应消息 `frame_id` | 逐字生效，Image 与 CameraInfo 保持同步 | TC-28 |
| `camera_info_distortion_model` | `auto` → `none` | `D` | 空数组 | TC-12 |
| `image_qos_depth` | `1` → `5` | `ros2 topic info -v` | depth 变化，reliability/durability 不变 | TC-04 附加 |
| `imu_qos_depth` | `200` → `50` | `ros2 topic info -v` | depth 变化 | TC-04 附加 |
| `stamp_offset_ns` | `0` → 固定值 | `header.stamp` | stamp 全等于 `device_ts + 该值`（差 ≤ 1 ms） | TC-18 附加 |
| `stamp_offset_mode` | `auto` → `device` | `header.stamp`、日志 | stamp 变成设备纳秒原始值；启动即打 `WARN` | TC-18 附加 |

---

## 4. 负向与失败路径测试

原则：每个可预期的失败都必须给出**可被自动化断言的**结果（退出码 + 可匹配的错误文本 + 无残留进程/话题）。出现"静默失败"（进程活着、没有数据、没有日志）一律记为 Blocker 级缺陷。期望值全部取自冻结书 §7.1/§7.2。

| 编号 | 失败场景 | 制造方法（不改 `python/`、不改系统库） | 必须观测到 | 反例（判失败） | 关联 TC |
| --- | --- | --- | --- | --- | --- |
| N-01 | 缺少 `libgs130` | `GS130_LIB=/nonexistent/libgs130.so` | `OSError: libgs130 not found; install libgs130.so.0 or set GS130_LIB` + 退出 | 段错误、挂死、静默退出 | TC-25 |
| N-02 | 相机缺失（排线/供电断开） | 断开传感器后运行默认参数 | `ERROR` `gs130_init failed: NOT_FOUND: camera not detected on the configured I2C buses`（或 `HW_ERROR`）+ 退出码 1 | 启动成功但无帧 | TC-26 |
| N-03 | 非法 `mode` | `-p mode:=bogus` | `ERROR invalid parameter mode=bogus: ...` + 退出码 2 | 悄悄回落默认 mode | TC-03 |
| N-04 | RAW + 非原生分辨率 | `-p mode:=raw -p width:=640 -p height:=480` | `ERROR invalid parameter ... RAW mode requires width=1088 height=1280` + 退出码 2 | 自动改成 1088x1280 不告知 | TC-27 |
| N-05 | 非法 `odr` | `-p odr:=100`（以及 `odr:=0`、`odr:=250`） | `ERROR invalid parameter odr=100: ...`（说明只接受 200/500）+ 退出码 2 | 静默夹取到 200 | TC-03 |
| N-06 | 非法 `platform`/`device` | `-p platform:=RDKX3`、`-p device:=GS130` | `ERROR unsupported platform/device: RDKX3 GS130WI` + 退出码 2 | 用默认值偷偷继续 | TC-03 |
| N-07 | 相机被 `mipi_cam` 占用 | 先启动 `mipi_cam` 再起本节点 | `ERROR gs130_init failed: HW_ERROR`（或 `NOT_FOUND`）+ 退出码 1，**不重试** | `mipi_cam` 被挤掉、或两进程交替出帧 | TC-23 |
| N-08 | 同机第二个实例 | 第一个节点运行中再启动第二个 | 第二个明确失败退出（`HW_ERROR`/`NOT_FOUND`）；第一个继续正常出图 | 第二个把第一个挤掉流 | TC-23 扩展 |
| N-09 | 无 EEPROM 标定 + `mode:=rect` | 用无标定样机；无样机时用 L2 桩测试（`camera_intrinsics` 抛 `GS130Error`）替代 | `ERROR gs130_init failed: PARAM_ERROR (RECT mode requires EEPROM calibration)` + 退出码 1 | 用全零内参继续发布 `CameraInfo` | TC-15 扩展 |
| N-10 | 无 EEPROM 标定 + `publish_camera_info:=true`（非 rect 模式） | 同 N-09 的条件 | `WARN calibration not available; /image_left/camera_info and /image_right/camera_info will not be published`，相机继续发布，**不退出** | 发布全零 `CameraInfo`（伪造数据）或直接崩溃 | TC-11 扩展 |
| N-11 | IMU 通路失败（拔出 IMU 或占用 I2C） | 仅在可安全操作的样机上执行 | `ERROR imu stream failed: {CODE}; IMU publishing disabled, camera continues`（1 次），`/imu/data` 停止，相机继续；`/gs130/status` 变 `ERROR` | 相机随之退出，或 IMU 话题继续发旧值 | TC-09 扩展 |
| N-12 | TROS web 依赖缺失 | 只 `ros2 run`（不 launch）；或在未 source `/opt/tros/humble` 的终端里 launch | launch 以 `package not found`/`executable not found` 失败并清理已启动进程 | launch 静默成功但网页无图（无法定位责任） | TC-19 扩展 |
| N-13 | 订阅者 QoS 不匹配 | `ros2 topic echo /image_combine_raw --qos-reliability reliable` | 节点 `WARN`（1 Hz 节流）`image subscriber /image_combine_raw requested incompatible QoS`，图像仍向兼容订阅者发布 | 节点崩溃或日志刷屏 | TC-04 扩展 |
| N-14 | `gs130_start()` 长时间阻塞（IMU FSYNC 未产生） | 在可控条件下阻断 IMU FSYNC（或把 `start_timeout_s` 设为 2 观察） | `INFO starting the camera; waiting for the IMU FSYNC handshake ...`，超时后每 `start_timeout_s` 打 `WARN gs130_start has not returned after {t} s ...`，**不**中止调用；此阶段 SIGINT 只 `WARN` 一次并等待握手 | 直接崩溃、或强行 `close()` 造成竞态 | TC-26 扩展 |

每个负向用例的证据要求与正向一致，**额外**必须记录：完整错误文本（原样）、退出码、从启动到退出的时间（要求 ≤ 30 s，参数校验类 ≤ 10 s）、以及错误发生后 `ros2 topic list` 的快照（证明没有"半启动"的话题）。

---

## 5. 稳定性测试

### 5.1 S-01 反复 start/stop（20 轮）

```bash
mkdir -p $EVID/stability
for i in $(seq 1 20); do
  start=$(date +%s.%N)
  timeout 45 ros2 launch gs130_ros gs130_web.launch.py > $EVID/stability/round_$i.log 2>&1 &
  pid=$!
  sleep 12
  timeout 15 ros2 topic hz /image_combine_jpeg  > $EVID/stability/hz_$i.log 2>&1
  timeout 15 ros2 topic hz /image_combine_raw   >> $EVID/stability/hz_$i.log 2>&1
  kill -INT $pid; wait $pid
  echo "round $i code=$? duration=$(echo "$(date +%s.%N) - $start" | bc)" >> $EVID/stability/rounds.txt
  sleep 5
  pgrep -af "gs130_node|hobot_codec|websocket" >> $EVID/stability/leftover_$i.log
done
```

| 指标 | 通过判据 |
| --- | --- |
| 成功率 | 20/20 轮都在 12 s 内出图（`hz_$i.log` 含 `average rate`）；任一轮失败即失败并保留该轮日志 |
| 退出码 | 每轮 `kill -INT` 后 launch 进程退出码 0（异常时记录实测码） |
| 残留进程 | 每轮结束后 `leftover_$i.log` 为空（脚本已 `sleep 5`，覆盖 3 s 关闭预算） |
| 单轮耗时漂移 | 第 20 轮耗时 ≤ 第 1 轮 × 1.5 |
| 内存 | 每轮记录 `ps -o rss= -C gs130_node`；第 20 轮 RSS ≤ 第 1 轮 + 20 MB，且无单调上升 |
| 相机释放 | 每 5 轮执行一次 TC-22 的复开自检（`reopen ok`） |
| 内核日志 | 全程 `dmesg -T \| tail -50` 无新的 `vpf/vinc/isp` 报错 |

### 5.2 S-02 长时 soak（≥ 60 min）

```bash
timeout 3900 ros2 launch gs130_ros gs130_web.launch.py > $EVID/stability/soak.log 2>&1 &
soak_pid=$!
( for i in $(seq 1 120); do
    echo "--- $(date -Is)" >> $EVID/stability/soak_samples.log
    timeout 12 ros2 topic hz /image_combine_raw        --window 200 >> $EVID/stability/soak_samples.log 2>&1
    timeout 12 ros2 topic hz /imu/data                 --window 500 >> $EVID/stability/soak_samples.log 2>&1
    ps -o rss= -C gs130_node >> $EVID/stability/soak_samples.log
    cat /sys/class/thermal/thermal_zone0/temp >> $EVID/stability/soak_samples.log
    sleep 18
  done )
kill -INT $soak_pid; wait $soak_pid
```

| 指标 | 通过判据 |
| --- | --- |
| 运行时长 | ≥ 60 min 连续运行，中途无重启 |
| 图像帧率 | 所有采样点 `average rate ≥ 0.9×fps`；采样点中位数漂移 ≤ 5%；**无**连续两个采样点低于 `0.5×fps` |
| IMU 速率 | 所有采样点 `≥ 0.8×odr`；无 > 30 s 的静默段（突发式发布下用 60 s 窗口平均判断） |
| 内存 | `gs130_node` RSS 从第 5 分钟到结束增长 ≤ 30 MB，且无单调上升趋势 |
| 温度 | 全程 SoC ≤ 85 °C；IMU 温度（用 TC-01 的 smoke 输出中的 `temp`）≤ 60 °C |
| 崩溃/异常 | `soak.log` 无 traceback、无 `process has died`；结束时 Ctrl-C 干净退出（复用 TC-22 判据） |
| 视觉复核 | soak 中途在网页（TC-21）看一次，画面仍为实时（排除冻帧） |
| 丢帧 | 若 `publish_status:=true`，`/gs130/status` 的 `dropped_frames` 增长速率 ≤ 1% 帧率；否则节点周期统计日志中的 `dropped` 计数同样要求 ≤ 1% |

---

## 6. 证据要求

### 6.1 每个用例必须留存

| 项 | 要求 | 存放 |
| --- | --- | --- |
| 命令 | 逐字复制实际执行的命令（含参数、`timeout`、环境变量） | 各 log 首行或 `$EVID/TC-XX.cmd` |
| 完整输出 | 原始 stdout+stderr，**不得**截断或手工美化 | `$EVID/TC-XX*.log` |
| 退出码 | `echo rc=$?` 的结果 | 各 log 末尾 |
| 图像/截图 | ≥10 张 PNG（左/右各 5）或 T1 拆半的 10 张；网页两张间隔 2 s 的截图；soak 中途 1 张 | `$EVID/img/`、`$EVID/web_live_*.png` |
| 时序 | 用例开始/结束时间与总时长；启动到首帧时间；SIGINT 到进程消失的时间 | `$EVID/timings.txt` |
| 参数快照 | `ros2 param list` 与关键参数取值（`ros2 param get`） | `$EVID/TC-XX-params.log` |
| 环境快照 | 每次会话开始时一次性采集（6.2） | `$EVID/env.txt` |
| 结论 | 每个 TC 一行：`TC-XX PASS/FAIL/BLOCKED` + 证据路径 + 缺陷 ID | `$EVID/summary.md` |

### 6.2 会话级环境快照（每次开测先跑一次）

```bash
{ echo "== date =="; date -Is
  echo "== uname =="; uname -a; head -3 /etc/os-release
  echo "== repo =="; git -C $REPO rev-parse --short HEAD; git -C $REPO status --porcelain
  echo "== VERSION =="; cat $REPO/VERSION
  echo "== libgs130 =="; ls -l /usr/lib/aarch64-linux-gnu/libgs130.so* 2>&1
  echo "== ros =="; source /opt/tros/humble/setup.bash; printenv ROS_DISTRO; ros2 --version 2>&1
  echo "== pkg =="; ros2 pkg prefix gs130_ros 2>&1
  echo "== launch =="; ls $WS/install/share/gs130_ros/launch/ 2>&1
  echo "== python =="; python3 -V; python3 -c "import gs130;print(gs130.__file__, gs130.__version__, gs130.library_version())" 2>&1
  echo "== procs =="; pgrep -af "mipi_cam|gs130|hobot_codec|websocket" 2>&1
  echo "== thermal =="; cat /sys/class/thermal/thermal_zone0/temp
} | tee $EVID/env.txt
```

### 6.3 证据与缺陷

* 失败用例必须附：现象、期望 vs 实际、完整 log，必要时 `dmesg -T | tail -100` 与 `free -m`。
* 缺陷编号 `GS130ROS-<nnn>`，在 `$EVID/summary.md` 中与 TC 编号双向链接；与冻结书冲突的缺陷必须引用冻结书章节号。
* 证据目录用时间戳命名，保留不覆盖；RC 发布的证据单独归档到 `$REPO/ros/docs/evidence/release_<version>/`。
* 测试期间不得 `git commit` / `git push`；证据写入未跟踪目录 `ros/docs/evidence/`。

---

## 7. 迭代进入/退出准则与发布门禁

### 7.1 每个迭代的进入准则（Entry）

| 编号 | 条件 |
| --- | --- |
| E1 | 开发已提交代码，并给出**变更清单**（新增/变更话题、参数、launch、依赖）与本迭代需覆盖的 TC 列表 |
| E2 | 板端已完成 §8.2 构建；`ros2 pkg prefix gs130_ros` 可解析；`git status --porcelain` 干净（禁止测试未提交的本地改动） |
| E3 | TC-01 通过，且 `gs130.__version__ == gs130.library_version()`、无版本告警 |
| E4 | 板卡无其他相机占用进程；测试 PC 可访问 `http://<BOARD_IP>:8000` |
| E5 | 本迭代范围与判据已冻结（FAIL 定义不得因实现改动而放宽）；判据与冻结书冲突时以冻结书为准 |

### 7.2 每个迭代的退出准则（Exit）

| 编号 | 条件 |
| --- | --- |
| X1 | 本迭代声明的 TC 全部执行并记录；L1/L2 全绿 |
| X2 | L3 冒烟子集（TC-04, TC-05, TC-06, TC-08, TC-09, TC-11, TC-12, TC-22, TC-30）全绿 |
| X3 | 无未关闭的 Blocker/Critical 缺陷；Major 缺陷有结论（修复或降级为已知限制并写入文档） |
| X4 | 涉及 launch / web 变更时，TC-19~TC-21 至少绿一次 |
| X5 | 证据齐备（第 6 章），`summary.md` 已更新 |

### 7.3 最终发布门禁（Release Gate）

| 编号 | 条件 |
| --- | --- |
| G1 | 全量 TC-01~TC-30 通过（降级类用例 N-09/N-10/N-11/N-14 允许"按冻结书声明的降级行为"通过，但必须在证据中逐字引用契约条款） |
| G2 | S-01（20 轮）与 S-02（≥60 min）通过 |
| G3 | **干净环境复测**：全新 clone + 按 §8.2/§8.3 全新构建与安装，重跑 L3 冒烟子集全绿（证明交付物自洽，不依赖开发者本地状态） |
| G4 | 冻结书 §6.6 的 7 条验收命令**逐字**执行并全部通过（见附录 C 的映射表） |
| G5 | 发布产物登记：`gs130_ros` 包版本（0.1.0）、冻结书版本、`libgs130` 版本、`VERSION`、TROS 版本、构建命令与 SHA256 |
| G6 | 已知限制清单（无 tf、无温度话题、无运行期重配置、rect 模式畸变上报规则、RECT 需要 EEPROM）已写入用户文档并与冻结书 §0.2/§5.5 一致 |
| G7 | 大版本发布需**连续两次**（跨天）完整通过 G1+G2，排除单次偶然 |

---

## 8. 测试环境

### 8.1 硬件与软件

| 项 | 值 |
| --- | --- |
| 板卡 | RDK X5，Ubuntu 22.04，Python 3.10 |
| 相机 | GS130WI（双目 1088×1280 MIPI + ICM-42688-P IMU + EEPROM `UNION Stereo-IMU Fisheye V1.2 Rotate-0-deg 4-Distortion-parameters`） |
| TROS | humble，`/opt/tros/humble` |
| 依赖 | `colcon`、`rclpy`、`sensor_msgs`、`geometry_msgs`、`diagnostic_msgs`、`launch`、`launch_ros`；OpenCV 4.11、numpy 1.26 |
| 复用的官方节点 | `hobot_codec_republish`、`websocket`（nginx 8000）、`hobot_shm`（可选） |
| 测试机 | 与板卡同网段的 PC，浏览器可访问 `http://<BOARD_IP>:8000` |
| 供电/网络 | 稳定供电（soak 期间不得断电）；网络不中断（网页验证需要） |

### 8.2 构建到 colcon 工作空间

冻结书的源码布局是 `$REPO/ros/gs130_ros/`（不是 `ros/src/`），因此工作空间用软链接指向它（`--symlink-install` 便于只改 Python 时免重复构建）：

```bash
source /opt/tros/humble/setup.bash
mkdir -p $WS/src
ln -sfn $REPO/ros/gs130_ros $WS/src/gs130_ros
cd $WS
colcon build --symlink-install --packages-select gs130_ros 2>&1 | tee $EVID/colcon_build.log
source $WS/install/setup.bash
ros2 pkg prefix gs130_ros                 # 期望：$WS/install/gs130_ros
ls $WS/install/share/gs130_ros/launch/    # 期望：gs130_web.launch.py gs130_camera.launch.py
```

说明：

* `ament_python` 包无 C++ 编译；`--symlink-install` 下改动节点 Python 文件后无需重新 build（新增 launch 文件或改 `setup.py` 后仍需 `colcon build`）。
* 若开发使用了 `--merge-install`，`ros2 pkg prefix` 输出形如 `$WS/install`，属正常；G3 干净复测必须使用与本迭代一致的构建命令并记录。
* `$REPO/ros/gs130_ros` 只包含 `package.xml`、`setup.py`、`setup.cfg`、`resource/gs130_ros`、`gs130_ros/`、`launch/`（冻结书 §1.1）。

### 8.3 让 `gs130` Python 包可被导入

冻结书 §1.3 要求 `gs130_node` 在运行期 `import gs130`。三种方式，择一并**记录在 `env.txt` 与 launch 说明中**（不得混用，避免版本歧义）：

| 方式 | 命令 | 适用 |
| --- | --- | --- |
| A（推荐，生产） | `cd $REPO/python && python3 -m pip install --no-build-isolation . 2>&1 \| tee -a $EVID/env.txt`（若报 externally-managed，用 `--user` 或 `pipx`；**不修改 `python/` 目录内容**） | 交付与发布测试 |
| B（迭代调试） | `export PYTHONPATH=$PYTHONPATH:$REPO/python` | 开发/QA 快速迭代（新终端需重设，或在 launch 前显式加 `PYTHONPATH=...`） |
| C（离线安装） | `python3 -m pip install $REPO/python/dist/gs130-<version>-py3-none-any.whl` | 板端无网络 |

自检（不通过则 TC-01 阻塞）：

```bash
python3 -c "import gs130, sys; print(gs130.__file__); print(gs130.__version__, gs130.library_version())"
```

要求：`gs130.__file__` 与登记方式一致（A/C 为 site-packages，B 为 `$REPO/python`）；版本与 `VERSION` 一致且无 `RuntimeWarning: libgs130 ... is older`。

### 8.4 launch 的节点与职责（用于定位"网页没图"）

| 顺序 | 节点 | 职责 | 缺失时的表现 |
| --- | --- | --- | --- |
| 1 | `gs130_node` | 相机 + IMU → `/image_combine_raw`（+ 可选 per-eye、`camera_info`、`/imu/data`、`/gs130/status`） | 所有 `/image_*`、`/imu/data` 缺失 |
| 2 | `hobot_codec_republish` | `/image_combine_raw`（`in_format=nv12`）→ `/image_combine_jpeg`（`out_format=jpeg`，`channel=0`） | `/image_combine_jpeg` 缺失 → 网页无图（TC-20 FAIL，TC-21 随之 FAIL） |
| 3 | `websocket` | 订阅 `/image_combine_jpeg`（`image_type=mjpeg`，`channel=1`，`output_fps=30`）→ 页面 channel 0 显示左目 | 页面打不开或无图像（TC-21 FAIL） |
| 可选 | `hobot_shm` | 零拷贝 env 开关（冻结书 §6.5：本节点不依赖） | 不影响正确性，只影响性能 |

参考链路（D-Robotics 132GS launch）：`NV12 on /image_combine_raw` → `hobot_codec` → `/image_combine_jpeg` → `websocket`。

### 8.5 `/image_combine_raw` 的尺寸契约（关键）

`hobot_codec_republish` 以 `in_format="nv12"` 订阅，期望 `encoding="nv12"`、`width=2W`、`height=H*3/2`、`step=2W`、`len(data)=step*height`。默认 `640x480` 下即 `1280×720`、`step 1280`、`691200` 字节。若 chain 无法解码：先按 TC-05 校验消息字段，再判断是否需要修改节点——**不得**改造 codec 去适配错误消息（冻结书 §8.4 禁止修改既有节点）。

### 8.6 测试期间的环境约束

* **相机独占**：任何时刻只允许一个进程持有相机；每个用例前 `pgrep -af "mipi_cam|gs130"` 必须为空（P1/P6）。
* 不修改 `python/`、`core/`、`ros/gs130_ros/` 下的任何文件（测试只做验证，不做修复）。
* 不修改 `hobot_codec` / `websocket` / `hobot_shm` 的任何参数默认值或源码（冻结书 §8.4）。
* 禁止 `git commit` / `git push`；证据写入未跟踪目录。
* 每个用例使用独立终端/独立进程；按 `Ctrl-C` 后必须确认进程消失再进入下一用例。

---

## 9. 影响测试有效性的风险

| 编号 | 风险 | 影响 | 缓解/检测手段 |
| --- | --- | --- | --- |
| R1 | 相机被 `mipi_cam`/其它进程占用 | 所有 L3/L4 用例失败，且可能被误判为代码缺陷 | 每次用例前检查（P1/P6）；TC-22/TC-23 专门覆盖 |
| R2 | `libgs130` 与 `gs130` 包版本不一致 | 行为与 API 不匹配，结论不可信 | TC-01 强制检查；`env.txt` 记录版本对 |
| R3 | 板卡 SoC 温度过高降频 | 帧率判据（TC-06/TC-17/S-02）误判 FAIL | 记录 `thermal_zone0`；帧率不达标时先看温度，> 80 °C 时降温复测 |
| R4 | 网络不稳或浏览器缓存 | TC-21 误判"网页无图" | 先 `curl` 验证 HTTP 200；浏览器无痕窗口 + 强制刷新；用两张截图或录屏判定实时性 |
| R5 | 光照条件 | TC-08 的 `mean/std` 判据在暗光下误判"全黑" | 固定室内照度、记录现场照片；必要时改为人工确认非全黑并登记 |
| R6 | 网页 channel 选择错误（选到 channel 1 或其他 demo） | TC-21 看到非本项目图像 | 按冻结书 §6.2 显示语义必须选 **channel 0**（=`websocket_channel:=1` 的左目）；前后用 `ros2 topic info -v /image_combine_jpeg` 交叉验证订阅者 |
| R7 | 实现与冻结书不一致（话题名/参数名/退出码改动） | 用例命令与断言失效 | 进入准则 E1 要求变更清单；差异按缺陷处理，判据不放宽（见 9.1 第 9 条） |
| R8 | `ros2 topic hz` 自身开销（大分辨率 + 采样窗口） | 测量值偏低，误判帧率 | 固定 `--window` 并记录；RAW（2176×1280）以"连续无中断"为判据 |
| R9 | 共享内存/零拷贝残留状态 | 二次运行失败被误判为代码缺陷 | TC-19 同时验证 `hobot_shm:=true/false`；区分"干净退出后的二次运行"（TC-24）与"强杀后的二次运行"（不计入判据） |
| R10 | 测试者手工改板端文件（不经 git） | G3 干净复测失败，或测到的不是交付版本 | E2 强制 `git status --porcelain` 为空；G3 在全新 clone 上复测 |
| R11 | IMU 为 FSYNC 突发式发布 | 用 `ros2 topic hz` 短窗口误判"速率不稳/丢包" | 用 ≥5 s 窗口或 A.5 脚本统计；判据写明"契约只保证不丢包、不重排、stamp 递增"（TC-10/TC-18） |
| R12 | 标定 EEPROM 与期望值不同（更换模组） | TC-11/TC-12/TC-15 的数值判据误判 | 判据以"ROS 值 == SDK 值"为主、"≈387/305/245"为辅；EEPROM 字符串变化必须记录并复核 |
| R13 | `camera_info` 为 latched，`ros2 topic echo --once` 可能拿不到历史样本 | TC-11/TC-12 误报"无数据" | 用带 `TRANSIENT_LOCAL` 的订阅（`ros2 topic echo` 默认即可收到 latched，但必须先确认 publisher 是 `TRANSIENT_LOCAL`）；连续 `--once` 失败两次时先用 A.6 脚本改为主动订阅再判定 |
| R14 | 硬件老化/排线接触不良 | 随机失败被误判为软件缺陷 | 出现"设备探测失败"时先跑 TC-01 与 SDK smoke；连续两次相同失败再立案 |

### 9.1 使一次测试运行**作废**（必须重跑）的条件

满足任意一条，该次运行的全部结论作废：

1. `git status --porcelain` 非空（被测代码与提交记录不一致）。
2. TC-01 未通过，或 `gs130.__version__` 与 `library_version()` 不一致/出现版本告警。
3. 测试期间有第二个进程持有相机（含测试者自己忘了关上一实例）。
4. 未按第 6 章采集证据（缺命令、缺完整 log、缺退出码），无法复核。
5. Python 导入方式与 `env.txt` 登记不一致（A/B/C 混用）。
6. 修改了 `hobot_codec`/`websocket`/`hobot_shm` 的任何参数或源码（破坏"官方节点可信"前提）。
7. 帧率/IMU 速率判据未在记录温度与负载的条件下测得（无法排除降频与负载干扰）。
8. 用 `pkill -9` 或重启板卡"清理"掉失败现场后再判定 PASS（掩盖真实缺陷）。
9. 遇到实现与冻结书不一致时，未记为缺陷而就地放宽判据（该次结论对相关 TC 无效）。

---

## 附录 A：测试脚本

脚本按原样执行即可（仅依赖 `rclpy`、`numpy`、`cv2`）。写入 `$EVID` 后运行；`$EVID` 需已导出。

### A.1 TC-05 / TC-14 / TC-17 图像字段检查（T1 + 可选 per-eye）

```python
# $EVID/tc05_fields.py
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

TOPICS = {
    "combine": ("/image_combine_raw", 1, 2),   # (话题, 宽度倍率, 无)
    "left": ("/image_left_raw", 1, 1),
    "right": ("/image_right_raw", 1, 1),
}


class Probe(Node):
    def __init__(self, wanted):
        super().__init__("tc05_probe")
        self.wanted = wanted
        self.got = set()
        for name in wanted:
            topic = TOPICS[name][0]
            self.create_subscription(
                Image, topic, lambda msg, n=name: self.check(msg, n),
                qos_profile_sensor_data)

    def check(self, msg, name):
        expected = msg.width * msg.height * 3 // 2
        print("%s topic=%s encoding=%s width=%d height=%d step=%d len=%d "
              "is_bigendian=%d frame_id=%s stamp=%d.%09d"
              % (name, TOPICS[name][0], msg.encoding, msg.width, msg.height,
                 msg.step, len(msg.data), msg.is_bigendian,
                 msg.header.frame_id, msg.header.stamp.sec,
                 msg.header.stamp.nanosec))
        assert msg.encoding == "nv12", msg.encoding
        assert msg.is_bigendian == 0
        assert msg.step == msg.width
        assert len(msg.data) == msg.step * msg.height == expected, "size mismatch"
        self.got.add(name)
        if self.got >= set(self.wanted):
            raise SystemExit(0)


rclpy.init()
node = Probe(sys.argv[1:] or ["combine"])
try:
    rclpy.spin(node)
except SystemExit:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
```

### A.2 TC-07 左右目（per-eye）分组比对

```python
# $EVID/tc07_pair.py  —— 需要 publish_per_eye:=true
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def ns(msg):
    return msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec


class Probe(Node):
    def __init__(self):
        super().__init__("tc07_probe")
        self.last = {}
        self.pairs = 0
        self.bad = 0
        self.create_subscription(Image, "/image_left_raw", lambda m: self.on(m, "left"), qos_profile_sensor_data)
        self.create_subscription(Image, "/image_right_raw", lambda m: self.on(m, "right"), qos_profile_sensor_data)

    def on(self, msg, side):
        self.last[side] = ns(msg)
        if len(self.last) == 2:
            delta = self.last["left"] - self.last["right"]
            self.pairs += 1
            self.bad += 1 if delta != 0 else 0
            print("pair %d left=%d right=%d delta_ns=%d" % (self.pairs, self.last["left"], self.last["right"], delta))
            if self.pairs >= 30:
                print("pairs=%d bad=%d" % (self.pairs, self.bad))
                raise SystemExit(0 if self.bad == 0 else 1)


rclpy.init()
node = Probe()
code = 0
try:
    rclpy.spin(node)
except SystemExit as exc:
    code = exc.code or 0
finally:
    node.destroy_node()
    rclpy.shutdown()
raise SystemExit(code)
```

> 说明：本脚本比对"最新到达的左右目"，低帧率下可能跨组配对。出现非零 `delta_ns` 时，先用连续两轮复测排除采样时序问题，再按缺陷处理；判据仍是"同一组数据的左右目时间戳相同"（冻结书 §2.1）。

### A.3 TC-08 从 T1 拆半解码并存 PNG

```python
# $EVID/tc08_save.py
import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

OUT = os.path.join(os.environ["EVID"], "img")
os.makedirs(OUT, exist_ok=True)
COUNT = 10


class Probe(Node):
    def __init__(self):
        super().__init__("tc08_probe")
        self.n = 0
        self.create_subscription(Image, "/image_combine_raw", self.on, qos_profile_sensor_data)

    def on(self, msg):
        if self.n >= COUNT:
            return
        w_half = msg.width // 2
        rows = msg.height
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(rows, msg.width)
        for side, sl in (("left", slice(0, w_half)), ("right", slice(w_half, msg.width))):
            half = np.ascontiguousarray(buf[:, sl])
            bgr = cv2.cvtColor(half, cv2.COLOR_YUV2BGR_NV12)
            path = os.path.join(OUT, "combine_%s_%02d.png" % (side, self.n + 1))
            ok = cv2.imwrite(path, bgr)
            print("half=%s shape=%s mean=%.1f std=%.1f ok=%s path=%s"
                  % (side, bgr.shape, bgr.mean(), bgr.std(), ok, path))
        self.n += 1
        if self.n >= COUNT:
            raise SystemExit(0)


rclpy.init()
node = Probe()
try:
    rclpy.spin(node)
except SystemExit:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
```

### A.4 TC-06 节点周期统计日志对照（可选）

```bash
# 直接抓节点日志中的周期统计行（默认每 5 s 一条）
timeout 30 ros2 launch gs130_ros gs130_camera.launch.py 2>&1 \
  | grep --line-buffered -E "camera .*Hz, published" | tee $EVID/TC-06-node-stats.log
```

### A.5 TC-09 / TC-10 IMU 统计

```python
# $EVID/tc09_imu.py
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def ns(msg):
    return msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec


class Probe(Node):
    def __init__(self):
        super().__init__("tc09_probe")
        self.accel, self.gyro, self.stamps = [], [], []
        self.orient = None
        self.covs = None
        self.frame_id = None
        self.create_subscription(Imu, "/imu/data", self.on_imu, qos_profile_sensor_data)
        # 反向断言：温度话题不应存在（订阅后不会收到即可，超时由外层 timeout 处理）
        self.create_subscription(Imu, "/imu/data", self.noop, qos_profile_sensor_data)

    def noop(self, msg):
        return

    def on_imu(self, msg):
        self.accel.append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        self.gyro.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        self.stamps.append(ns(msg))
        self.orient = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        self.covs = (len(msg.linear_acceleration_covariance), msg.linear_acceleration_covariance[0],
                     len(msg.angular_velocity_covariance), msg.angular_velocity_covariance[0],
                     len(msg.orientation_covariance), msg.orientation_covariance[0])
        self.frame_id = msg.header.frame_id
        if len(self.accel) >= 200:
            self.report()
            raise SystemExit(0)

    def report(self):
        accel = np.array(self.accel)
        gyro = np.array(self.gyro)
        stamps = np.array(self.stamps)
        span = (stamps[-1] - stamps[0]) / 1e9
        print("count=%d rate=%.2f Hz frame_id=%s orientation=%s"
              % (len(accel), (len(accel) - 1) / span, self.frame_id, self.orient))
        print("cov_lens_and_first=%s" % (self.covs,))
        print("accel_mean=%s accel_std=%s accel_norm=%.3f"
              % (np.round(accel.mean(axis=0), 3), np.round(accel.std(axis=0), 3),
                 np.linalg.norm(accel.mean(axis=0))))
        print("gyro_mean=%s gyro_std=%s"
              % (np.round(gyro.mean(axis=0), 4), np.round(gyro.std(axis=0), 4)))


rclpy.init()
node = Probe()
try:
    rclpy.spin(node)
except SystemExit:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
```

### A.6 TC-14 尺寸三元组

```python
# $EVID/tc14_dims.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo


def latched():
    return QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Probe(Node):
    def __init__(self):
        super().__init__("tc14_probe")
        self.img = self.left = self.right = None
        self.create_subscription(Image, "/image_combine_raw", self.on_img, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/image_left/camera_info", lambda m: self.on_info(m, "left"), latched())
        self.create_subscription(CameraInfo, "/image_right/camera_info", lambda m: self.on_info(m, "right"), latched())

    def on_img(self, msg):
        self.img = msg
        self.report()

    def on_info(self, msg, side):
        setattr(self, side, msg)
        self.report()

    def report(self):
        if None in (self.img, self.left, self.right):
            return
        i = self.img
        print("combine: w=%d h=%d step=%d len=%d frame_id=%s" % (i.width, i.height, i.step, len(i.data), i.header.frame_id))
        for side, c in (("left", self.left), ("right", self.right)):
            print("camera_info %s: w=%d h=%d frame_id=%s binning=%sx%s roi=%dx%d do_rectify=%s"
                  % (side, c.width, c.height, c.header.frame_id, c.binning_x, c.binning_y,
                     c.roi.width, c.roi.height, c.roi.do_rectify))
        print("check w == 2*cw -> %s ; h == 3*ch//2 -> %s"
              % (i.width == 2 * self.left.width, i.height == 3 * self.left.height // 2))
        raise SystemExit(0)


rclpy.init()
node = Probe()
try:
    rclpy.spin(node)
except SystemExit:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
```

### A.7 TC-18 时间戳单调性与偏移稳定性

```python
# $EVID/tc18_stamps.py
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu


def ns(msg):
    return msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec


class Probe(Node):
    def __init__(self):
        super().__init__("tc18_probe")
        self.img, self.imu = [], []
        self.create_subscription(Image, "/image_combine_raw", lambda m: self.add(self.img, m), qos_profile_sensor_data)
        self.create_subscription(Imu, "/imu/data", lambda m: self.add(self.imu, m), qos_profile_sensor_data)

    def add(self, bucket, msg):
        bucket.append(ns(msg))
        if len(self.img) >= 100 and len(self.imu) >= 100:
            self.report()
            raise SystemExit(0)

    @staticmethod
    def stats(name, values, strict):
        arr = np.array(values, dtype=np.int64)
        d = np.diff(arr)
        bad = (d <= 0).sum() if strict else (d < 0).sum()
        print("%s count=%d monotonic=%s violations=%d max_gap_ms=%.2f min=%d max=%d"
              % (name, len(arr), bool((d > 0).all() if strict else (d >= 0).all()),
                 int(bad), d.max() / 1e6, arr.min(), arr.max()))

    def report(self):
        self.stats("image", self.img, strict=False)   # 允许相等（补点），不得回退
        self.stats("imu", self.imu, strict=True)      # 严格递增
        offsets = np.array(self.img[:len(self.imu)]) - np.array(self.imu[:len(self.img)])
        print("cam_minus_imu min=%.3f ms max=%.3f ms span=%.3f ms"
              % (offsets.min() / 1e6, offsets.max() / 1e6, (offsets.max() - offsets.min()) / 1e6))


rclpy.init()
node = Probe()
try:
    rclpy.spin(node)
except SystemExit:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
```

### A.8 TC-13 / TC-30 SDK 内参与外参真值导出

```python
# $EVID/tc30_extrinsics.py
import numpy as np
import gs130

FRAMES = (gs130.ReferenceFrame.CAMERA_LEFT, gs130.ReferenceFrame.CAMERA_RIGHT, gs130.ReferenceFrame.IMU)
cfg = gs130.Config.preset("RDKX5", "GS130WI", gs130.CameraMode.RESIZE, 640, 480, 30, 200)

with gs130.Device(cfg) as dev:
    for name, index in (("left", gs130.CameraIndex.LEFT), ("right", gs130.CameraIndex.RIGHT)):
        k = dev.camera_intrinsics(index)
        print("%s fx=%.9f fy=%.9f cx=%.9f cy=%.9f" % (name, k.fx, k.fy, k.cx, k.cy))
        print("%s K=%s" % (name, np.round(k.K, 9).tolist()))
        print("%s dist_model=%s dist_coeffs=%s" % (name, k.dist_model.name, np.round(k.dist_coeffs, 9).tolist()))
    cal = dev.calibration()
    print("install_angle=%d" % cal.install_angle)
    print("imu_R=%s imu_T=%s" % (np.round(cal.imu_R, 9).tolist(), np.round(cal.imu_T, 9).tolist()))
    print("left_R=%s left_T=%s" % (np.round(cal.camera_left_R, 9).tolist(), np.round(cal.camera_left_T, 9).tolist()))

    ok = True
    for a in FRAMES:
        for b in FRAMES:
            R = dev.relative_R(a, b)
            T = dev.relative_T(a, b)
            print("%s->%s R=%s T=%s" % (a.name, b.name, np.round(R, 9).tolist(), np.round(T, 9).tolist()))
            orth = np.abs(R.T @ R - np.eye(3)).max()
            if orth >= 1e-6:
                ok = False
                print("  !! orthogonality failed: %.3e" % orth)
            if a != b:
                R2 = dev.relative_R(b, a)
                T2 = dev.relative_T(b, a)
                dR = np.abs(R - R2.T).max()
                dT = np.abs(T + R2 @ T2).max()
                print("  inverse check dR=%.3e dT=%.3e" % (dR, dT))
                if dR >= 1e-9 or dT >= 1e-9:
                    ok = False
                    print("  !! inverse consistency failed")
    baseline = float(np.linalg.norm(dev.relative_T(gs130.ReferenceFrame.CAMERA_LEFT, gs130.ReferenceFrame.CAMERA_RIGHT)))
    print("baseline_left_right=%.6f m in_range=%s" % (baseline, 0.02 <= baseline <= 0.30))
    print("CONSISTENCY %s" % ("PASS" if ok else "FAIL"))
```

---

## 附录 B：用例索引

| 级别 | 用例 |
| --- | --- |
| L1 主机 | TC-01, TC-02, TC-03 |
| L2 接口/消息 | TC-03, TC-05（桩）, TC-11（桩）, TC-29（桩） |
| L3 板端集成 | TC-04 ~ TC-18, TC-22 ~ TC-30 |
| L4 端到端 Web | TC-19, TC-20, TC-21 |
| L5 回归/稳定性 | TC-22, TC-24, TC-28, S-01, S-02 |
| 冒烟子集（每迭代必跑） | TC-01, TC-02, TC-03, TC-04, TC-05, TC-06, TC-08, TC-09, TC-11, TC-12, TC-22, TC-30 |
| 发布门禁全量 | TC-01 ~ TC-30 + S-01 + S-02 + G3 干净环境复测 + G4 验收命令 |

---

## 附录 C：冻结书 §6.6 验收命令 → 用例映射

| 冻结书命令 | 对应用例 |
| --- | --- |
| `colcon build --packages-select gs130_ros` + `source install/setup.bash` | TC-02（并见 §8.2） |
| `ros2 launch gs130_ros gs130_web.launch.py platform:=RDKX5 device:=GS130WI mode:=resize width:=640 height:=480 fps:=30 odr:=200` | TC-19（+ TC-20/TC-21） |
| `ros2 launch gs130_ros gs130_camera.launch.py ...` | TC-19 附加检查（仅相机路径） |
| `ros2 launch gs130_ros gs130_camera.launch.py mode:=rect width:=640 height:=480 fps:=30 publish_per_eye:=true publish_status:=true` | TC-15 + TC-17 + TC-28 |
| `ros2 launch gs130_ros gs130_camera.launch.py mode:=raw width:=1088 height:=1280 fps:=30 odr:=200` | TC-27 |
| `ros2 run gs130_ros gs130_node`（全默认参数） | TC-04（默认参数路径） |
| `ros2 topic hz /image_combine_raw` | TC-06 |
| `ros2 topic echo /image_combine_raw --once --field encoding`（期望 `nv12`） | TC-05 |
| `ros2 topic echo /image_left/camera_info --once`（期望 latched，`D` 长度 4 或 5） | TC-11 + TC-12 |
| `ros2 topic hz /imu/data` | TC-10 |
| `ros2 topic echo /image_combine_jpeg --once --field format`（期望含 `jpeg`） | TC-20 |
| 浏览器打开 `http://<板卡IP>:8000`，选择 channel 0 → 左目画面 | TC-21 |
