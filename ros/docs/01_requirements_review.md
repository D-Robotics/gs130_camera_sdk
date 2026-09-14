# 01 Requirements Review — GS130 ROS 2 (TROS) 接口

| 项 | 值 |
| --- | --- |
| 项目 | gs130_sdk / ROS 2 接口包 |
| 文档 | Requirements Review (PM-1) |
| 目标平台 | RDK X5 (Ubuntu 22.04) + TROS humble (`/opt/tros/humble`) |
| 基线版本 | gs130_sdk 0.0.1（`VERSION`），Python 绑定 `python/gs130/` |
| 目标版本 | ROS 包 `gs130_ros` v0.1.0 |
| 状态 | 待工程评审（第 6 节开放问题需回填结论） |
| 输入来源 | 已核实板卡事实 + 现有 `python/gs130` 公开 API（本文件不重复验证硬件） |

## 1. 目标与问题陈述

- **现状**：GS130（双目 1088x1280 + ICM-42688-P IMU + 带鱼眼标定的 EEPROM）目前只能通过 C/C++ SDK 与 `python/gs130` 绑定使用。ROS 2 用户必须自己写进程、自己管生命周期、自己拼消息。
- **矛盾点**：
  1. 相机是**独占资源**（同一时刻只能被一个进程持有），任何"随手跑个脚本"都会与正在运行的节点互相踩踏。
  2. 板卡上已有 TROS 生态（`hobot_codec`、`websocket` web UI、`hobot_shm`、`hobot_stereonet`），而现有可视化入口是围绕 `mipi_cam` 的 NV12 话题 `/image_combine_raw` 设计的，GS130 没有任何话题可以接进去。
  3. 标定/外参（`camera_intrinsics`、`relative_R/T`、`calibration`）只存在于 SDK 里，ROS 侧没有对应的 `CameraInfo` / TF，导致任何 SLAM、VIO、深度下游都要重复造轮子。
- **要做的事**：提供一个薄封装 ROS 2 包，把 SDK 的既有能力（取图、IMU、标定、参数预设、生命周期）映射为**标准 ROS 消息与标准启动方式**，并**复用**已有 TROS 节点完成编码与 Web 显示，而不是新写驱动或新写前端。
- **判断标准**：薄。ROS 包只做"SDK ↔ ROS 消息/生命周期"的翻译；任何 SDK 已经能做的事不在 ROS 层重做（见第 4 节非目标）。

## 2. 目标用户

| 用户 | 诉求 | 主要对应故事 |
| --- | --- | --- |
| ROS 2 应用开发者（RDK X5） | 用标准话题拿到双目图像、IMU、内外参，直接喂给自研算法 | US-03 ~ US-08, US-14 |
| D-Robotics TROS 用户 | 一条 launch 起相机，在现有 `http://<board-ip>:8000` web UI 里看到实时画面，不写一行代码 | US-01, US-02, US-13 |
| 现场调试/集成人员 | 相机缺失或被占用时立刻知道原因；重复启动不残留 | US-09, US-10, US-11 |

非用户（v0.1.0 明确不考虑）：ROS 1 用户、非 RDK X5 平台用户、想同时跑 `mipi_cam` 的多相机用户。

## 3. 用户故事（User Stories）

SDK 能力列表示该故事**依赖的既有 API**，不新增 C/C++ 代码。

| ID | 用户故事 | 依赖的 SDK 能力 |
| --- | --- | --- |
| US-01 | 作为 TROS 用户，我希望用一条 launch 命令启动 GS130 双目取流，这样我不必了解 SDK 的 I2C/GPIO 细节。 | `Config.preset`、`Device.start` |
| US-02 | 作为 TROS 用户，我希望在现有 TROS web UI 里看到 GS130 实时画面，这样我不需要新的前端。 | 复用 `hobot_codec` + `websocket` |
| US-03 | 作为 ROS 开发者，我希望双目图像以 `sensor_msgs/Image`（NV12）发布在独立话题上，这样我能直接接入自己的算法。 | `read_image()` → `{"left","right"}` |
| US-04 | 作为 ROS 开发者，我希望 IMU 以 `sensor_msgs/Imu` 发布（无 IMU 设备时自动不发布），这样我能直接接入融合算法。 | `read_imu()`、`ImuPacket`、`imu_name` |
| US-05 | 作为 ROS 开发者，我希望相机内参以 `sensor_msgs/CameraInfo` 发布（左右各一），这样我能做去畸变/投影。 | `camera_intrinsics(LEFT/RIGHT)` |
| US-06 | 作为 ROS 开发者，我希望拿到相机-相机与 IMU-相机外参（TF 或话题），这样我能搭 VIO/SLAM 坐标系。 | `calibration()`、`relative_R/relative_T`、`ReferenceFrame` |
| US-07 | 作为 ROS 开发者，我希望 `dist_model`/`install_angle` 等非标准标定信息可获取，这样我能正确处理鱼眼模型与安装角。 | `Calibration`、`DistModel`、`imu_intrinsics()` |
| US-08 | 作为 TROS 用户，我希望 platform/device/mode/width/height/fps/odr 都由 ROS 参数选择，这样同一份 launch 能覆盖 RAW/RESIZE/RECT 与不同帧率。 | `Config.preset(platform, device, mode, width, height, fps, odr)` |
| US-09 | 作为集成者，我希望 ctrl-c 后节点干净退出、相机立即释放，这样我可以马上再次启动或让 `mipi_cam` 使用硬件。 | `Device.stop/close`、上下文管理器 |
| US-10 | 作为集成者，我希望相机缺失或被占用时得到**明确原因**而不是超时或段错误，这样我能快速定位。 | `GS130Error.code/func`、`ErrorCode` |
| US-11 | 作为调试人员，我希望有一个只读探测命令打印库版本、相机/IMU 可用性、标定摘要，这样我在起节点前就能确认链路。 | `library_version`、`available_camera/imu`、`eeprom_*` |
| US-12 | 作为支持人员，我希望运行期能查到 SDK 库版本与包版本，这样我能判断兼容性。 | `gs130.library_version()`、`gs130.__version__` |
| US-13 | 作为 ROS 用户，我希望用 `ros2 topic hz` 验证实际帧率与参数一致，这样我能确认没有静默降速。 | `read_image()` 节拍 |
| US-14 | 作为 ROS 开发者，我希望每条消息都带 `header.stamp` 与 `header.frame_id`，这样我能做时间同步与 TF 关联。 | `Image.timestamp_ns`、`ImuPacket.timestamp_ns` |

**故事粒度约束**：每条故事必须可被第 5 节的一条可观测语句验收；不满足者拆故事或降级为非目标。

## 4. 非目标（NON-GOALS，v0.1.0 明确不做）

| # | 非目标 | 理由 |
| --- | --- | --- |
| N-01 | 自研 web UI / web 节点 / HTTP 服务 | 已存在 `websocket` 包与端口 8000 的 nginx 前端，重写即重复开发 |
| N-02 | 在 ROS 包内用 C++ 重写相机驱动（V4L2/VI/ISP） | SDK 已提供；重写会与 SDK 抢占独占硬件并产生两套真相 |
| N-03 | 自研深度 / 点云 / 视差计算 | 属 `hobot_stereonet` 职责；且本 SDK 预设 `stereo_layout=NONE`，不提供拼接图（见 Q-1） |
| N-04 | ROS 1（roscpp/rospy）支持 | 目标平台只有 TROS humble |
| N-05 | 支持无法测试的硬件（非 RDK X5、非 GS130W/WI 传感器、其它 IMU） | `Config.preset` 只接受 `RDKX5` + `GS130W`/`GS130WI` |
| N-06 | 自动生成消息包（`add_message_files` 自定义 msg） | 全部使用标准 `sensor_msgs`，避免下游额外编译依赖（见 Q-9） |
| N-07 | 运行期动态重配相机管线（`mode/resolution/fps` 热切换） | SDK 需 `deinit`→`init` 重建；v0.1.0 只读启动参数，运行期修改仅告警不改行为 |
| N-08 | 多相机实例 / 多节点同时占用 | 硬件独占；v0.1.0 单实例单命名空间 |
| N-09 | 修改 `python/` 下的绑定实现 | 本阶段冻结 Python 绑定，仅消费其公开 API |
| N-10 | 替代 `mipi_cam` 的通用相机框架（V4L2 抽象、`camera_info_manager` 全量功能） | 超出"GS130 接口包"范围 |

## 5. 验收标准（Acceptance Criteria）

| ID | 可观测、可检查的验收语句 |
| --- | --- |
| AC-01 | `ros2 launch gs130_ros gs130_camera.launch.py` 后 3 秒内 `ros2 node list` 出现相机节点，且 `ros2 topic list` 含左右图像话题；无参数时使用文档默认值（`platform=RDKX5`、`device=GS130WI`、`mode=RAW`、`width=1088`、`height=1280`、`fps=30`、`odr=100`）。 |
| AC-02 | 在浏览器打开 `http://<board-ip>:8000` 可见实时画面；同时 `ros2 topic hz /image_jpeg`（或所选 JPEG 话题）有稳定输出。 |
| AC-03 | `ros2 topic list` 含 `/gs130/image_left_raw`、`/gs130/image_right_raw`；`ros2 topic echo --once /gs130/image_left_raw --field encoding` 返回 `nv12`；`--field width/height` 等于配置的 `width/height`（RAW 模式等于 1088/1280）。 |
| AC-04 | `device:=GS130WI` 时 `/gs130/imu` 有输出且 `ros2 topic echo --once /gs130/imu` 的 `linear_acceleration`/`angular_velocity` 为 3 元素数值；`device:=GS130W` 时该话题**不出现**且日志有一条"no IMU"说明（不报错退出）。 |
| AC-05 | `/gs130/left/camera_info`、`/gs130/right/camera_info` 各至少发布一次；`K[0]`(fx)、`K[4]`(fy)、`K[2]`(cx)、`K[5]`(cy) 与 `camera_intrinsics()` 返回值一致，`width/height` 与图像一致，`distortion_model` 与 `DistModel` 对应（`plumb_bob` 或 `equidistant`）。 |
| AC-06 | `ros2 run tf2_ros tf2_echo gs130_camera_left_frame gs130_camera_right_frame` 打印出与 `relative_R/relative_T` 一致的平移；`ros2 run tf2_ros tf2_echo gs130_camera_left_frame gs130_imu_frame` 在 `publish_tf:=true` 且设备为 `GS130WI` 时可用。 |
| AC-07 | `ros2 param get` / 节点启动日志中可读到 `platform, device, mode, width, height, fps, odr` 的实际生效值，以及 `dist_model`、`install_angle`（来自 `calibration()`），无需额外工具即可确认鱼眼模型与安装角。 |
| AC-08 | `odr:=200 fps:=15 width:=544 height:=640 mode:=RESIZE`（或其它 `preset` 接受组合）启动后，AC-03 的 `width/height` 与话题频率随参数变化；`mode:=RECT` 时图像话题照常发布。非法组合（如 `mode:=RESIZE` 但 width/height 为 0、`device:=GS130X`）时节点以非零码退出并给出参数名与接受范围；`fps:=0` 时不发布图像但 IMU 仍发布。 |
| AC-09 | 对节点发 `SIGINT`（ctrl-c）后 3 秒内进程退出，`ros2 node list` 不再包含该节点，`pgrep -f gs130_ros` 无残留；**紧接着再次执行 AC-01 的 launch 能成功**（证明相机已释放）。 |
| AC-10 | 相机被占用场景（另一个进程持有 GS130）启动节点：日志出现固定前缀的可读原因（例如 `[gs130_ros] camera busy`）并带 `GS130Error.code`/`func`，节点非零退出；相机不在位的场景同理。异常路径下无 segfault、无 `hw error` 之外的模糊信息。 |
| AC-11 | `ros2 run gs130_ros gs130_probe` 打印 `library_version`、`gs130.__version__`、`available_camera()`、`available_imu()`、`imu_name/eeprom_name` 与标定摘要；相机不可用时退出码非零。（该命令会短暂持有相机，需在文档中标注"独占、勿与节点同跑"。） |
| AC-12 | 节点日志首行包含 `libgs130 <library_version>` 与 `gs130 <__version__>`；两者不一致时（含 SDK 自身版本检查告警）在日志中可见。 |
| AC-13 | `ros2 topic hz /gs130/image_left_raw` 报告值在配置 `fps` 的 ±10% 内持续 10 秒以上（例：`fps:=30` 时约 30 Hz）。 |
| AC-14 | 对任意一帧图像消息与同一时刻的 IMU 消息，`header.frame_id` 非空且属于文档列出的 frame 名集合；`header.stamp` 单调不减；用 `ros2 run tf2_ros tf2_echo` 能解析到该 frame_id。时间戳语义在文档中写明（见 Q-3）。 |

## 6. 开放问题与风险（需工程给出结论）

| ID | 问题 | 影响 | 建议（PM 推荐） |
| --- | --- | --- | --- |
| Q-1 | `Config.preset` 在 Python 绑定里硬编码 `stereo_layout=StereoLayout.NONE`，因此 `read_image()` 走 `{"left","right"}` 分支，**SDK 公开路径拿不到拼接 NV12**；而 D-Robotics 参考 launch 的 web 链路是围绕 `/image_combine_raw`（拼接图）设计的。同时 `Device.stitched`/`Image` 尺寸计算都跟随 `output_*`，拼接假设一旦被破坏会算出错误 buffer 长度。 | 直接决定 US-02/US-03 的话题设计，以及 `hobot_stereonet` 能否接入 | **v0.1.0 不依赖拼接**：相机侧发布左右两路 NV12，web 链路让 `hobot_codec` 编码**其中一路**（`sub_topic=/gs130/image_left_raw`），web UI 显示单目实时画面即满足 US-02；不要在本包内手写拼接，也不要绕过 `Config.preset` 去改 `stereo_layout`。若后续确实需要拼接/立体网络，作为独立需求单独立项（可能要动绑定层，属 N-09 变更）。 |
| Q-2 | EEPROM 内参的**参考分辨率**：`CameraIntrinsics` 只有 `fx/fy/cx/cy/K/dist_coeffs/dist_model`，无参考宽高字段。RAW 是 1088x1280，RESIZE 是自定义尺寸，RECT 是去畸变输出。 | `CameraInfo.K` 在非 RAW 模式下若是错的，下游标定/测距全错 | 工程实测：RAW 与 RESIZE 同场景下比较 `K`。若 `K` 不随输出尺寸变化，则按 `sx=out_w/sensor_w, sy=out_h/sensor_h` 缩放 `fx/fy/cx/cy` 后再填 `CameraInfo`，并在文档标注缩放公式；若已随尺寸变化则直接透传。**必须先测再写代码**，禁止假设。 |
| Q-3 | `timestamp_ns` 的**时钟域**未定义（CLOCK_MONOTONIC？steady？是否与主板墙钟同源）。ROS 侧 `header.stamp` 若填错时钟域，`tf`/`message_filters` 会集体错位。 | US-14 与所有时间同步下游 | 启动时同时采样 `timestamp_ns` 与 `node.get_clock().now()`，若差值在 ±100 ms 内则直接使用 SDK 时间戳（图像与 IMU 同源，优先保一致性）；否则用 ROS 时钟并在日志给出明确警告。要求工程确认 SDK 时间戳来源，并把结论写入 README。 |
| Q-4 | `dist_coeffs` 长度固定 8；`DistModel.FISHEYE` 时 8 个分量的语义（OpenCV `equidistant` 常用 4 个）与 `CameraInfo` 的映射未定义。 | US-05/AC-05 的 `distortion_model` 正确性 | `PINHOLE` → `plumb_bob`（取 k1..k3, p1, p2，其余补 0）；`FISHEYE` → `equidistant`（取前 4 项），并在 README 记录原始 8 项与截断规则；如实测语义不符，只改映射表，不改绑定。 |
| Q-5 | `relative_R/relative_T` 的**方向约定**（`ReferenceFrame(from) → to` 是 `T_to_from` 还是 `T_from_to`），以及 `install_angle` 单位（度/弧度）与作用。 | TF 方向错了会让 VIO/SLAM 直接发散，属高危 | 采用 `Q-6` 的命名约定并在实机上验证方向（用一次已知平移/旋转检查符号）；验证结论固化成一个单元校验用例写在文档里。`install_angle` v0.1.0 仅上报为参数/日志，不折算进 TF，直到单位被确认。 |
| Q-6 | TF frame 命名与发布方式：是否需要 TF、发静态还是动态、frame 名规范。 | US-06 与下游 TF 消费者 | 发**静态 TF**（`static_transform_publisher` 语义，同硬件刚性固定）：`<ns>/camera_left_frame` ↔ `<ns>/camera_right_frame`，`<ns>/imu_frame` → left（`GS130WI` 且 `publish_tf:=true` 时）；图像/IMU 的 `header.frame_id` 用同一套名字，光学坐标系命名为 `<...>_optical_frame`（REP-105：z 前、x 右、y 下）。IMU 外参除 TF 外另以 `ros2 param`/话题输出数值，便于离线使用。 |
| Q-7 | 图像传输路径：能否用带共享内存（hbmem/`hobot_shm`）的零拷贝 NV12 发布，还是 rclpy 必须 memcpy。 | CPU 占用与 fps 上限（双路 1088x1280@30 约 2 MB/帧） | 首选零拷贝 NV12 发布（`encoding=nv12`）；工程验证 rclpy 下是否可用共享内存分配器，不可用则退回 memcpy，并把实测 CPU/帧率写进 README。不要为此引入 C++ 节点（N-02 精神：能做但 v0.1.0 不值当）。 |
| Q-8 | RAW/RESIZE/RECT 三种 `mode` 在 ROS 侧是否都作为一等公民支持，`RESIZE` 的允许尺寸与 `fps` 允许值范围未文档化。 | AC-08 的参数校验边界；也可能出现"参数被接受但采集失败" | v0.1.0 **不在 ROS 层复制校验规则**，把参数交给 `Config.preset` 与 `Device.start`，将其 `ValueError`/`GS130Error` 原样翻译成参数名 + 报错文本（避免两套真相）。README 只列已在实机验证通过的组合。 |
| Q-9 | IMU 的 `temp` 与 `is_fsync` 在 `sensor_msgs/Imu` 中无处安放。 | 是否打破"不新建消息包"（N-06） | 保持只发标准 `sensor_msgs/Imu`；`temp` 通过参数 `publish_imu_temp:=true` 时以 `diagnostic_msgs`/话题外方式暴露（或仅日志），`is_fsync` v0.1.0 仅按需打日志。**不为两个字段新建 msg 包**。 |
| Q-10 | `odr` 的合法取值集合与 `fps` 的关系（`preset` 的 `odr` 直接写进 `ImuConfig.odr_hz`，`fps` 写进 `CameraConfig.fps`，均无范围校验）。 | 用户填了硬件不支持的组合 → 运行期失败而非参数期失败 | 采纳 Q-8 的同一条策略（透传给 SDK 并翻译错误），同时在 README 明确列出一组实测可用组合；若实测发现明显不安全（如整机挂死），升级为 ROS 层白名单校验。 |
| Q-11 | 与 `mipi_cam` 的互斥：GS130 独占。启动时若 `mipi_cam` 在跑（或反之）。 | 现场最常见的"起不来"抱怨 | 不写复杂互斥逻辑：节点启动失败时在日志给出固定可搜关键词 `camera busy` 并在 README 顶部写明"GS130 与 mipi_cam 不可同时运行"；launch 内注释提醒。 |
| Q-12 | 相机复用与 QoS：图像与 IMU 的话题 QoS（可靠/最佳努力、队列深度）、FIFO 丢帧策略（`FifoMode.DROP_OLD`，`camera_fifo.depth` 预设为 4）。 | 高负载下延迟累积或丢帧，web 画面卡顿 | 图像/IMU 默认 `SensorDataQoS`（best effort, depth 5）并做成参数；`camera_fifo.depth` 暴露为 `frame_queue_depth`。高延迟场景推荐 `DROP_OLD`（SDK 已是默认），在 README 给出"实时性优先"参数模板。 |
| Q-13 | `python/gs130` 的加载前置：`_runtime.load()` 预加载 `libvpf.so/libhbmem.so/libcam.so`，依赖 `GS130_LIB`/系统库路径；ROS 环境变量在 `source` 后可能覆盖 `LD_LIBRARY_PATH`。 | 节点起来后报 "libgs130 not found" | 在 README 与 launch 头注释中明确"先 `source /opt/tros/humble/setup.bash`"，并把库查找失败的错误原样打印（含被检查的路径/`GS130_LIB` 值），不做自动修复。 |
| Q-14 | 客户端 Web UI 的 `channel` 与 JPEG 话题绑定关系，以及 `hobot_codec` 的 `input_framerate/output_framerate` 与相机 fps 的联动。 | US-02 可能出现"节点正常但网页黑屏" | launch 中把 `input_framerate` 绑定到 `fps` 参数，`image_topic` 指向我们发布的 `/gs130/image_jpeg`，`channel` 固定为 0 并在 README 说明"黑屏先查 `ros2 topic hz /gs130/image_jpeg`"，与 D-Robotics 参考 launch（132GS）保持同构。 |

**风险汇总（高 → 低）**：Q-5（TF 方向错，下游发散）> Q-2/Q-3（内参与时间戳语义错，静默错误）> Q-1（拼接路径缺失，功能预期落差）> Q-7/Q-12（性能）> Q-14/Q-11/Q-13（集成体验）。

## 7. 范围总结（v0.1.0 交付内容）

**交付**

- 一个 ament_python 包 `gs130_ros`（**不新增 C++ 源码**）：
  - 单节点：相机取流 + IMU 取流 + 标定/CameraInfo + 静态 TF，参数即 `Config.preset` 的入参。
  - 一个只读诊断命令 `gs130_probe`（US-11）。
- 三个 launch：`gs130_camera.launch.py`（仅相机）、`gs130_websocket.launch.py`（相机 + `hobot_codec` + `websocket`，复用已有 launch 文件的 include）、`gs130_camera_info_only.launch.py`（仅标定/TF，无取流，供离线标定用途）。
- 标准消息与话题（命名空间 `<ns>` 默认 `gs130`）：
  - `/gs130/image_left_raw`、`/gs130/image_right_raw`：`sensor_msgs/Image`，`encoding=nv12`。
  - `/gs130/image_jpeg`：由既有 `hobot_codec` 产生（非本包代码）。
  - `/gs130/imu`：`sensor_msgs/Imu`（`GS130WI` 才发布）。
  - `/gs130/left/camera_info`、`/gs130/right/camera_info`：`sensor_msgs/CameraInfo`。
  - 静态 TF：left ↔ right（+ imu → left，条件发布）。
- 文档：README（安装、`source /opt/tros/humble/setup.bash`、与 `mipi_cam` 互斥说明、参数表、话题表、已知限制、实测可用参数组合、错误关键词 `camera busy`）。
- 测试：一个不需要相机的 launch/参数翻译单元测试；一个需相机的冒烟测试清单（CHECKLIST，人工执行）。

**明确不交付（对应第 4 节）**

- 自研 web 前端或 web 节点；相机驱动重写；深度/点云/视差；ROS 1；不可测硬件；自定义 msg 包；运行期相机管线热重配；多相机实例；`python/` 绑定改动。

**下一版候选（不在 v0.1.0，需重新走需求评审）**

- 拼接 NV12 与 `hobot_stereonet` 接入（依赖 Q-1 结论，可能触及绑定层）、拼接图的方图/同步精度提升、多实例命名空间支持、IMU 温度与 `is_fsync` 的正式暴露方式。
