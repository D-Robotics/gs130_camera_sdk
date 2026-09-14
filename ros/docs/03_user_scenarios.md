# 03 用户场景与兼容性分析（PM-3 交付物）

- 项目：gs130_sdk（分支 develop，SDK 版本 0.0.1）
- 目标：为 GS130 双目相机 + IMU 提供 ROS 2 / TROS（humble，`/opt/tros/humble`）接口包 `gs130_ros`，并通过 D-Robotics 现有 TROS Web UI 展示
- 本文用途：需求评审的场景与兼容性依据。本文只描述接口行为，不承诺未经板端验证的能力
- 待建包路径（约定）：`ros/gs130_ros/`（`launch/gs130_web.launch.py`、`launch/gs130.launch.py`、节点源码）

## 0. 前提、约束与本文的证据边界

### 0.1 已确认事实（不再验证，直接使用）

| 项 | 事实 |
| --- | --- |
| TROS | humble，`source /opt/tros/humble/setup.bash`；`colcon` 位于 `/usr/local/bin/colcon` |
| 复用节点 | `hobot_codec`（`hobot_codec_republish`：ROS image → jpeg，参数 `in_mode` / `in_format` / `out_mode` / `out_format` / `sub_topic` / `pub_topic` / `jpg_quality`）、`websocket`（nginx 8000，参数 `image_topic` 默认 `/image_jpeg`、`only_show_image`、`channel`、`smart_topic`）、`hobot_shm`（零拷贝环境）、`hobot_stereonet`（输出 `/StereoNetNode/stereonet_visual`） |
| 参考链路 | camera NV12 `/image_combine_raw` → `hobot_codec` → `/image_combine_jpeg` → `websocket` channel 0 |
| 硬件 | 双目 1088x1280 MIPI（left/right）、ICM-42688-P IMU、EEPROM 鱼眼标定 |
| 独占性 | 相机同一时刻只能被一个进程持有；**不得与 mipi_cam 同时运行** |

### 0.2 SDK 行为事实（读代码得到，场景分析依赖它们）

| 事实 | 出处 | 对 ROS 场景的含义 |
| --- | --- | --- |
| 有 IMU 时，相机开流前必须完成 IMU FSYNC 握手；`gs130_start()` 不返回错误，但相机线程会一直等 `camera_on` | `core/src/gs130.cpp:222`、`:523-530` | FSYNC 不来时表现为"进程活着、话题为空"，节点必须自己报超时 |
| IMU 数据只在 FSYNC 包到达时成批压入 FIFO，一批约 `odr_hz / fps` 个包 | `core/src/gs130.cpp:177-203` | 见 SC-03：IMU 是"每相机帧发布一次"的批量流 |
| 未检测到 IMU 时直接开流（`camera_on = true`） | `core/src/gs130.cpp:527-529` | IMU 缺失不阻塞出图（但节点应默认要求 IMU，见 M-3） |
| IMU 线程出错（FIFO 满、FSYNC 长时间丢失）会把设备置为故障态，此后 `available_camera()` / `available_imu()` 均返回 0 | `core/src/gs130.cpp:149`、`:158`、`:550`、`:598` | IMU 故障会**同时中断图像话题**，不是"只有 IMU 掉线" |
| 拼接布局在 `gs130_init()` 时定死；非拼接下 `get_stereo_nv12_frame` 返回 `UNSUPPORTED`，拼接下 `get_nv12_frame` 返回 `UNSUPPORTED` | `core/include/gs130.h:232`、`:250`；`core/src/gs130.cpp:563`、`:581` | 切换 NONE ↔ 拼接需要 stop + close + 重新 init（约 1-2 s 停机） |
| 相机线程已做半帧对齐（左右目时间戳差 ≤ 半帧周期否则重取） | `core/src/gs130.cpp:316-329` | 左右目视为同步对；但仍有残差，见 SC-02 |
| 拼接帧的时间戳取 FSYNC 绑定目（默认右目） | `core/src/gs130.cpp:335` | 拼接帧只有一个时间戳，分左右后要自行记住布局 |
| 图像缓冲：NV12 紧密打包（Y 平面 + UV 平面），缓冲区所有权归调用方（Python 侧为 numpy） | `core/include/gs130.h:206-210` | ROS `sensor_msgs/Image` 需 `step` 与 NV12 语义见 SC-01 |
| 标定：`RECT`/`RESIZE` 模式下读出的内参与外参是**矫正/缩放后**的；只有 `RAW` 模式读出的是 EEPROM 原始标定 | `core/samples/gs130-calib-export.c:52-58`；`core/include/gs130.h:338-342`（开启立体矫正后外参变为虚拟平行双目） | 见 M-1：标定必须在 RAW 模式导出 |
| 无 EEPROM 时 `camera_intrinsics` / `imu_intrinsics` / `calibration` / `relative_R/T` / `convert_calibration` 返回 `NOT_FOUND`；`RECT` 模式 init 直接 `PARAM_ERROR` | `core/include/gs130.h:160`；`core/src/gs130.cpp:484`、`:642`、`:663`、`:689`、`:709`、`:731`、`:749` | 标定类场景在无 EEPROM 硬件上是硬失败，需明确报错 |
| 错误码：`TIMEOUT`=暂时无数据、`THREAD_CLOSED`=未启动/已停止、其余为故障（需 deinit+init 恢复） | `core/include/gs130.h:21-29`；`python/gs130/_enums.py` | 节点区分"空队列"与"设备故障" |

### 0.3 未经板端验证、评审必须当作 open question 的点

1. `hobot_codec` 实际接受的 `in_format` / `out_format` 取值集合，以及 `websocket` 期望的 JPEG 话题消息类型（本文按官方参考链路写为 `nv12 → jpeg`，需板端确认）。
2. `hobot_stereonet` 的 launch 是否会连带启动 mipi_cam（会与 gs130_ros 抢相机），以及它对标定文件的格式/路径要求。
3. 相机被残留进程占用时，`gs130_init()` 返回的确切错误码（本文按参数/硬件/未找到三类分别描述）。
4. `hobot_shm` 是否覆盖 NV12 大帧的传输；v0.1.0 不依赖它，默认走标准 DDS 传输。

---

## 1. 场景清单

### 场景—需求映射总表

| 编号 | 场景 | v0.1.0 定位 | 依赖 |
| --- | --- | --- | --- |
| SC-01 | 首次点亮 + Web 实时看拼接双目 | 必须 | `gs130_ros` + `hobot_codec` + `websocket` |
| SC-02 | 应用消费 left/right 并跑自研双目算法 | 必须 | `gs130_ros`（`stereo_layout:=none`） |
| SC-03 | 应用同时消费 IMU 与图像（时间戳） | 必须 | `gs130_ros` |
| SC-04 | 读标定/内外参搭建矫正或深度流水线 | 必须（导出工具 + 参数一次导出，无新 ROS 话题） | `gs130-calib-export`（已有）+ 节点 |
| SC-05 | 用我们的图像喂 `hobot_stereonet` | 延后到 v0.2.0（v0.1.0 仅做"能订阅"烟囱验证） | 见 SC-05 |
| SC-06 | 不用 Web UI，用外部 ROS 工具看数据 | 必须（文档 + 命令行验收） | 无 |
| SC-07 | 干净退出与重启（含相机被占用的处理） | 必须 | 节点信号处理 |

### 1.1 接口约定（本文引用的名字，评审冻结后才能改动）

| 名字 | 值 | 说明 |
| --- | --- | --- |
| 包 / 节点 | `gs130_ros` / `gs130_camera` | 节点源码建议 Python（`rclpy` + 已有 `python/gs130`），纯 wrapper，不引入 C++ 编译链 |
| 拼接图像 | `/image_combine_raw` | 沿用 D-Robotics 约定，直接接 `hobot_codec`，不另造名字 |
| 分目图像 | `/gs130/left/image_raw`、`/gs130/right/image_raw` | 仅 `stereo_layout:=none` 时发布 |
| IMU | `/gs130/imu` | `sensor_msgs/Imu`，`frame_id: gs130_imu_link` |
| 编码 | `sensor_msgs/Image`，`encoding: nv12`（默认），`is_bigendian: 0`，`step = width`（Y 与 UV 连续） | 见 M-1；不默认转 BGR，避免裁剪/畸变处理与 CPU 拷贝 |
| 布局默认 | `stereo_layout:=left_right`（默认即拼接，服务 SC-01） | 分目场景显式 `stereo_layout:=none` |
| 默认模式/尺寸/帧率 | `mode:=resize`、`640x480`、`fps:=30`、`odr_hz:=200` | `mode:=rect` 用于矫正后输出；`mode:=raw` 仅用于导标定 |
| launch 文件 | `launch/gs130.launch.py`（只有节点）、`launch/gs130_web.launch.py`（节点 + codec + websocket） | — |

---

### SC-01 首次点亮：Web UI 实时查看拼接双目画面

**目标**：插上相机、3 分钟内在这台机器的 8000 端口网页上看到左右目拼接的实时画面，不需要自己写 codec、不需要自己写网页。

**前置条件**
1. 相机已接好（I2C bus 4/6、MIPI RX、复位 GPIO 由 `Config.preset("RDKX5", "GS130WI", ...)` 提供，无需用户手工填写）。
2. 设备上**没有** mipi_cam 或其他占用相机的进程（`ps -ef | grep -E "mipi_cam|gs130"` 应为空）。
3. 已安装 SDK 与 Python 包（`python/` 的 wheel 或 `pip install -e python`），`python3 -c "import gs130; print(gs130.library_version())"` 成功。

**命令**
```bash
source /opt/tros/humble/setup.bash
# 终端 A：相机 + codec + websocket（一个 launch 起全链路）
ros2 launch gs130_ros gs130_web.launch.py
# 终端 A 中另开终端，确认话题与链路
source /opt/tros/humble/setup.bash
ros2 topic hz /image_combine_raw     # 期望 ≈30 Hz
ros2 topic hz /image_combine_jpeg    # 期望 ≈30 Hz
# 浏览器（本机或同网段 PC）
# http://<RDK_X5_IP>:8000
```

**期望的可观察结果**
1. `ros2 topic hz /image_combine_raw` ≈ `fps`（30 Hz ±10%）。
2. `/image_combine_jpeg` 与前者同频率；`websocket` 默认 `image_topic:=/image_jpeg`，由 launch 显式改写为本链路的 JPEG 话题，无需用户手填。
3. 网页看到 1280x480（640x480 横向拼接）的连续画面，左右目内容正确、无撕裂；遮挡左目时画面左半边变化、右半边不动（这是"拼接方向正确"的判据）。
4. 节点日志：设备版本、IMU 型号（`ICM-42688-P` 之类）、EEPROM 名称、实际生效的 layout/mode/分辨率/fps、以及"设备已就绪，开始发布"。

**失败行为**
| 现象 | 原因 | 用户处理 |
| --- | --- | --- |
| 节点启动但 `/image_combine_raw` 一直 0 Hz，日志停在"等待 IMU FSYNC 握手" | FSYNC 未建立（IMU 未探测到 / FSYNC 未接） | 节点在 `startup_timeout_s`（默认 5 s）后 logger.fatal 并退出，提示"未收到 IMU FSYNC，相机未开流，请检查 IMU 与排线"；不要让它静默等待 |
| 有 `/image_combine_raw` 但网页黑屏 | `hobot_codec` 的 `in_format` 与 `encoding: nv12` 不匹配，或 `websocket` 的 `image_topic` 未指向本链路 JPEG | 见 0.3 第 1 条，需板端确认后固定 launch 参数；文档给出一行 `ros2 param get` 自查命令 |
| 网页出现绿色/花屏画面 | 浏览器拿到的是未解码 NV12 当 JPEG 显示 | 明确文档："网页只能看 JPEG 链路，不能直接订阅 NV12" |
| `3000` 端口有画面但 `8000` 没有 | 看错端口或看错通道 | 文档固定写"nginx 8000，channel 0" |
| 启动即报相机被占用 | 上一条 mipi_cam / 上一次 gs130 残留 | 转 SC-07 |

---

### SC-02 应用消费 left/right，跑自己的双目算法

**目标**：拿到左右目独立帧（各自有时间戳），自己算视差/深度，不用我们的任何深度结果。

**前置条件**
1. 相机空闲；算法程序在 RDK X5 本机或同网段 PC。
2. 明确本场景**不需要**拼接，因此 launch 必须显式给 `stereo_layout:=none`（默认是拼接，见 M-1）。

**命令**
```bash
source /opt/tros/humble/setup.bash
ros2 launch gs130_ros gs130.launch.py stereo_layout:=none mode:=resize width:=640 height:=480 fps:=30
# 应用侧（示例：Python + 自己的算法）
ros2 topic hz /gs130/left/image_raw
ros2 topic hz /gs130/right/image_raw
ros2 topic info /gs130/left/image_raw --verbose   # 看 encoding= nv12, step= 640
```

**期望的可观察结果**
1. 两个话题都以 `fps` 发布；`/image_combine_raw` 不再发布（布局为 NONE 时该话题无输出，节点日志明确说明"当前为分目模式，无拼接话题"）。
2. 同一 `header.stamp` 附近可配对的左右帧，其 `header.stamp` 差值应在半帧周期以内（30 fps 时 < 16.7 ms，SDK 已在相机线程做半帧对齐）。
3. 每帧 `encoding: nv12`、`step == width`、`width x height == 640x480`（或用户指定值）；应用自行做 NV12→BGR/灰度。
4. 应用输出的视差图主观正确（左右目内容同步，同一物体左右位置差与深度趋势一致）。

**失败行为**
1. 忘记 `stereo_layout:=none`：节点以拼接模式运行，`read_image()` 走 `get_stereo_nv12_frame`，`/gs130/left|right` 不发布；节点日志 warn 明确写"当前为拼接模式，未发布分目话题，如需分目请 `stereo_layout:=none`"。
2. 运行中改布局：不支持热切换，必须重启 launch（stop + close + init，约 1-2 s）；节点对 `stereo_layout` 参数设 `read_only` 语义，运行中 `ros2 param set` 返回失败并提示重启。
3. 应用太慢导致丢帧：SDK 默认 `camera_fifo = FifoConfig(4, DROP_OLD)`（`python/gs130/_config.py:90`），丢的是最旧的整对帧，表现为时间戳跳变而非错位；节点应发布累计丢帧数并在丢帧率 > 5% 时 warn。
4. 应用假设左右目时间戳完全相等：不是，SDK 允许 ≤ 半帧残差；文档要求按时间戳配对而不是按到达顺序配对。

---

### SC-03 应用同时消费 IMU 与图像（时间戳语义）

**目标**：在 ROS 里同时拿到图像与 IMU，并正确处理两者的时间关系。

**前置条件**：相机空闲；`GS130WI` 版本（含 IMU）；默认 `odr_hz:=200`。

**命令**
```bash
source /opt/tros/humble/setup.bash
ros2 launch gs130_ros gs130.launch.py stereo_layout:=none odr_hz:=200
ros2 topic hz /gs130/imu                  # 期望 ≈200 Hz
ros2 topic echo /gs130/imu --once         # 看 header.stamp / frame_id / linear_acceleration / angular_velocity
```

**期望的可观察结果**
1. `/gs130/imu` 平均频率 ≈ `odr_hz`（200 Hz），`frame_id: gs130_imu_link`；`sensor_msgs/Imu` 字段填充 `linear_acceleration`（m/s²）、`angular_velocity`（rad/s）、`temperature`（`core/include/gs130.h:258-264`）。
2. **SDK 事实（必须写进 README 和节点日志）**：IMU 数据包只在相机帧的 FSYNC 到达时成批发布，一批约 `odr_hz / fps` 个包（30 fps + 200 Hz 时约 6-7 个/批，间隔 5 ms，每帧一批，批间隔 33.3 ms）。因此
   - 时间上：IMU 是"每相机帧一批"的突发流，而不是等间隔连续流；
   - 最新一包 IMU 相对最新一帧图像，最多可能落后**一个相机帧周期**（30 fps 时约 33 ms）；在 FSYNC 握手完成前，一包 IMU 都不会发布；
   - 对 ROS 用户的含义：**不要按到达顺序把 IMU 包与图像一一配对**，也不要假设"图像一到，对应的 IMU 就已经到齐"；按 `header.stamp` 做时间对齐（插值/外推由用户的融合算法负责），并接受最长约一帧周期的 IMU 新鲜度上限；
   - 如果确实需要"图像到位时 IMU 必须已经覆盖到该时刻"，用户要么自己延迟图像 N 个周期，要么把 `fps`/`odr_hz` 一起调大以缩小这个上限。
3. 节点对每个 IMU 包写 `header.stamp`（来自 `timestamp_ns`，已对齐相机时钟），不做重排序、不丢包、不做插值——节点是搬运工，融合归用户。

**失败行为**
1. 未探测到 IMU：`imu_name is None` → 节点默认 `require_imu:=true` 时 logger.fatal 退出（提示"未检测到 IMU，本场景需要 IMU"）；显式 `require_imu:=false` 时才降级为只出图，且日志必须 warn"IMU 未启用，/gs130/imu 不会发布"。
2. IMU 中途故障（硬件 FIFO 满 / FSYNC 长时间丢失）：设备进入故障态，**图像与 IMU 一起停**；节点检测 `device.running` 变 false 或连续 `TIMEOUT` 超时，报 fatal 并退出，不能表现为"话题静默但节点健康"。
3. 用户把 IMU 直接灌进 EKF 而忽略约一帧周期的批量延迟：滤波器可能出现滞后/发散，文档在 SC-03 明确写"IMU 延迟上限 = 1/fps，需在融合中显式补偿"。
4. `odr_hz` 设得远大于 `fps`：单批包变多，单帧周期内的 IMU 延迟上限不变（仍由 `fps` 决定）；文档说明"提高 ODR 只增加批内密度，不减小批延迟"。

---

### SC-04 读取标定/内外参，搭建矫正或深度流水线

**目标**：拿到左右目内参 + 畸变模型 + IMU 内参 + 相对外参，用于自己的矫正/深度流水线。

**前置条件**
1. 相机空闲。本场景**不需要开流**：`gs130-calib-export` 只做 `create/init/get_calibration/deinit/destroy`（`core/samples/gs130-calib-export.c`），不占流、不耗帧。
2. EEPROM 可读；无 EEPROM 时本场景硬失败（`NOT_FOUND` / `RECT` 模式 init 返回 `PARAM_ERROR`）。

**命令**
```bash
# 1) 导出标定（RAW 模式！这是唯一能拿到 EEPROM 原始标定的模式）
mkdir -p /tmp/gs130_calib
core/samples/gs130-calib-export /tmp/gs130_calib RDKX5 GS130WI raw 640 480 30 200
# 产出：/tmp/gs130_calib/camchain.yaml 与 /tmp/gs130_calib/imu.yaml
cat /tmp/gs130_calib/camchain.yaml
# 2) 在同一终端里用 Python 读（更细的字段，如 dist_model / 噪声）
python3 - <<'PY'
import gs130
cfg = gs130.Config.preset("RDKX5", "GS130WI", gs130.CameraMode.RAW, 1088, 1280, 30, 200)
dev = gs130.Device(cfg)          # init 即可读标定，不必 start
cal = dev.calibration()
print(dev.eeprom_name, dev.eeprom_info)
print(dev.camera_intrinsics(gs130.CameraIndex.LEFT).fx)
print(dev.camera_intrinsics(gs130.CameraIndex.RIGHT).dist_model)
print(dev.relative_R(gs130.ReferenceFrame.CAMERA_RIGHT, gs130.ReferenceFrame.CAMERA_LEFT))
print(dev.relative_T(gs130.ReferenceFrame.CAMERA_RIGHT, gs130.ReferenceFrame.CAMERA_LEFT))
print(dev.relative_R(gs130.ReferenceFrame.IMU, gs130.ReferenceFrame.CAMERA_RIGHT))
dev.close()
PY
```

**期望的可观察结果**
1. `camchain.yaml` 含 `cam0`（右目）与 `cam1`（左目）：`camera_model: pinhole`、`distortion_model: equidistant|radtan`、4 个畸变系数、`intrinsics: [fx, fy, cx, cy]`、`resolution`、`T_cam_imu`、`T_cn_cnm1`（`core/samples/gs130-calib-export.c:73-120`）。
2. Python 侧同理，且能拿到 `dist_coeffs[8]`、`accel/gyro` 的 `misalign/scale/bias/noise/random_walk`。
3. 外参语义必须按头文件说明理解：`R`/`T` 把 `from_frame` 下的点变换到 `to_frame`（即 `to_frame` 下 `from_frame` 的位姿），`p_to = R * p_from + T`；`camera_install_angle` 一并导出。
4. 参考系默认为 EEPROM 驱动定义的设备参考系；需要时用 `convert_calibration()` 重新锚定（如以右目为基准），之后所有 `calibration/relative_*` 返回值同步改变。

**失败行为**
1. 在 `resize`/`rect` 模式下导出标定：内参是输出分辨率下的、外参在矫正模式下是"虚拟平行双目"的（`core/include/gs130.h:341`）。工具本身只会打印黄色 warning（`gs130-calib-export.c:52-56`），**文档与 SC-04 命令强制使用 `raw`**；误用会导致重复矫正、深度不准（这是本文列的三大误用之一，见 §6 M-1）。
2. 无 EEPROM：`camera_intrinsics` / `calibration` / `relative_R/T` 抛 `GS130Error(NOT_FOUND)`；`mode:=rect` 的 launch 直接 init 失败。节点与文档明确提示"无标定，无法提供矫正/标定类能力"，不做静默降级。
3. 用 `test_gs130.py` 之类的流式样例去读标定（必须 `start()` 才读）会白占相机：文档固定推荐 `gs130-calib-export` 或"init 不 start"的 8 行片段。
4. `convert_calibration` 在 `init` 之前调用无效（init 会重新从 EEPROM 载入并覆盖），文档给出"init 之后、start 之前调用"的顺序要求（`core/include/gs130.h:441`）。

---

### SC-05 把我们的图像喂给 D-Robotics 双目深度节点（诚实评估）

**目标**：用 `hobot_stereonet` 直接消费 GS130 的图像得到深度/可视化（`/StereoNetNode/stereonet_visual`），从而不必自己写深度网络。

**前置条件（缺一不可）**
1. `gs130_ros` 以拼接布局运行，且 `output_width`/`output_height` 与 stereonet 期望的输入分辨率、左右顺序一致。
2. 有与 stereonet 输入分辨率匹配的**正确标定文件**（stereonet 通常从相机标定读取内参/外参，而我们的标定在 EEPROM 里，是鱼眼模型 + 已由 ISP 用 GDC 做过矫正/未矫正两种情形）。
3. stereonet 的 launch **不会**连带启动 mipi_cam，或我们能关闭那一项（否则两个进程抢相机，必失败）。
4. 板端实测过分辨率/布局/标定这一组合。

**命令（若在 v0.2.0 之后启用）**
```bash
source /opt/tros/humble/setup.bash
# 终端 A：只起我们的相机链路（不含 mipi_cam）
ros2 launch gs130_ros gs130.launch.py stereo_layout:=left_right mode:=rect width:=640 height:=480 fps:=30 require_imu:=false
ros2 topic hz /image_combine_raw
# 终端 B：起 stereonet，指向我们的相机话题（具体参数名以待验证的 stereonet launch 为准）
ros2 launch hobot_stereonet hobot_stereonet.launch.py \
  <此处替换为已验证的"禁用 mipi_cam + 指定标定文件"参数>
ros2 topic echo /StereoNetNode/stereonet_visual --once
```

**诚实评估（v0.1.0 是否现实）**

| 维度 | 结论 |
| --- | --- |
| 图像链路可行性 | 可行。我们输出 NV12 拼接帧到 `/image_combine_raw`，与 D-Robotics 参考链路是同一形状（横向拼接），图像侧不构成障碍 |
| 标定链路可行性 | **未验证，是主要风险**。GS130 的标定来源是 EEPROM + 我们导出的 camchain/imu YAML；stereonet 期望的标定文件格式、路径、分辨率是否与之一致没有板端证据。若 stereonet 内部自己做矫正，而我们的 `mode:=rect` 已经用 GDC 矫正过，会**二次矫正**导致深度错误 |
| 进程/话题冲突 | **未验证**。若 stereonet 的 launch 自带 mipi_cam，两个进程同时持有相机必然失败；必须先确认可关闭 |
| 谁拥有深度 | 深度结果、点云、可视化统统由 D-Robotics 节点产出，属于**复用**，不算我们的工作量，也不作为我们的验收指标 |
| v0.1.0 结论 | **延后**。v0.1.0 只做"烟囱验证"：证明 stereonet 能订阅到我们的 `/image_combine_raw`（`ros2 topic info /image_combine_raw --verbose` 能看到 stereonet 节点作为订阅者），或在不能关闭 mipi_cam 时明确记录为**阻塞项**并写进已知限制。深度输出不作为 v0.1.0 验收项 |
| 延后到 v0.2.0 的解锁条件 | ① 确认 stereonet 的 mipi_cam 开关；② 确认其标定文件格式（用 `gs130-calib-export` 产出后能否直接使用，或需要一层转换）；③ `mode` 与 stereonet 内部矫正的归属（谁做矫正）二选一；④ board 上跑通一次并记录命令 |

**失败行为**
1. stereonet 未收到图像：先查它在不在订阅者列表，再查分辨率/编码是否被它的 QoS 或格式检查拒绝。
2. 收到图像但深度全零/错乱：优先怀疑标定来源（二次矫正、分辨率不匹配），而不是先怀疑网络。
3. 两个进程抢相机：表现为其中一个 init 失败或直接硬件错误；节点应给出"相机被占用，请先停止 mipi_cam / 上一次进程"的可执行提示（见 SC-07）。
4. 相机为独占资源：**不允许**为了 stereonet 而并存 mipi_cam，这一条写进 §2 与 §4。

---

### SC-06 不用 Web UI：用外部 ROS 工具查看数据

**目标**：不依赖 nginx 网页，用 ROS 2 原生命令或 RViz / Foxglove 检查数据是否正常。

**前置条件**：`gs130_ros` 已运行；外部工具与相机节点在同一 `ROS_DOMAIN_ID` / RMW 环境下（TROS 的 `/opt/tros/humble/setup.bash` 必须 source）。

**命令**
```bash
source /opt/tros/humble/setup.bash
ros2 topic list
ros2 topic hz /image_combine_raw
ros2 topic hz /gs130/imu
ros2 topic echo /gs130/imu --once
ros2 topic info /image_combine_raw --verbose      # 看 encoding / step / 订阅者
# 可视化（PC 端需与板卡同 ROS_DOMAIN_ID）
rviz2        # Add -> Image -> /gs130/left/image_raw（topic 列表里应能看到）
foxglove-studio
```

**期望的可观察结果**
1. `ros2 topic list` 能看到 §1.1 表中的话题（按布局发布相应子集）。
2. `/gs130/imu` 可用 `ros2 topic echo --once` 直接读；图像话题**不要**对整帧做 `echo`（NV12 大数组，输出是二进制洪流，应以 `hz` + `topic info` 为准，这一点写进文档）。
3. RViz 的 Image 显示：`encoding: nv12` 时 RViz 不能正确显示（RViz 原生支持 mono8/rgb8/bgr8/32FC1 等），因此**外部可视化看的是 `hobot_codec` 的 JPEG 输出**（`sensor_msgs/CompressedImage`，RViz 用 CompressedImage 显示，Foxglove 直接在 Image 面板选该话题）。
4. 因此 v0.1.0 的定位是：原始数据用 `ros2` CLI + `hobot_codec` 后的 JPEG 做可视化；节点**不**额外提供 BGR 转换话题（见 §4 非目标），除非评审要求再加一个 `encoding:=bgr8` 调试参数。

**失败行为**
1. `ros2 topic list` 为空：几乎总是环境不一致（未 source TROS、`ROS_DOMAIN_ID` 不同、跨机 RMW 不通）。文档给出"在启动 launch 的同一终端环境里执行"的要求。
2. RViz 添加 `nv12` 图像显示报错：预期行为，文档明写"RViz 不显示 NV12，请看 JPEG 话题"。
3. 跨机看不到话题：DDS discovery 受限，文档给出"先在本机 `ros2 topic list` 验证，再排查跨机/域号"的排查顺序。

---

### SC-07 干净退出与重启（含"上一次没关干净"）

**目标**：Ctrl-C 后相机被释放，可以立刻再次启动；进程被强杀或相机被别的进程占用时，用户能看到可执行的提示而不是莫名失败。

**命令**
```bash
# 正常退出
ros2 launch gs130_ros gs130_web.launch.py     # Ctrl-C
sleep 2
ros2 launch gs130_ros gs130_web.launch.py     # 立即重启，应正常出图
# 检查占用
ps -ef | grep -E "mipi_cam|gs130" | grep -v grep
# 强制清理（确认是残留进程后）
pkill -f gs130_camera ; sleep 2
```

**期望的可观察结果**
1. Ctrl-C（SIGINT）→ 节点捕获信号，调用 `device.stop()` 与 `device.close()`（`close()` 内部是 `gs130_deinit` + `gs130_destroy`：停止线程、关闭流、恢复 sensor 上电状态、关 I2C），日志输出"设备已释放，可以重新启动"，进程在 1 s 内退出（正常路径无需等待 `startup_timeout_s`）。
2. SIGTERM 同样处理；无论哪种信号，`close()` 必须只被调用一次（SDK 侧重复 close 是空操作，见 `python/gs130/_device.py:118-129`）。
3. `ps -ef` 中不残留节点进程；`ros2 topic list` 中相机话题消失。
4. 立即重启成功，行为与首次启动一致（Web 链路约 3-4 s 内出图，含 FSYNC 握手）。

**失败行为（相机被占用 / 上一次残留）**
| 现象 | 原因 | 期望的节点行为 |
| --- | --- | --- |
| 启动时 `gs130_init()` 失败（`PARAM_ERROR` / `HW_ERROR` / `NOT_FOUND` / `TIMEOUT`，确切错误码待板端确认） | 相机被 mipi_cam、上一次残留进程或别的示例占用 | 节点**不重试**、不静默，日志 fatal 打印：原始错误码 + "相机为独占资源，请先 `ps -ef \| grep -E 'mipi_cam\|gs130'`，确认后 `pkill -f mipi_cam` / `pkill -f gs130_camera`，必要时重启模块"，然后退出码非 0 |
| 测试脚本异常退出后（未走信号路径）再启动失败 | 进程未释放相机、内核侧流未停 | 同上提示，并额外说明"SDK 未做跨进程恢复，若 pkill 无效需重启相机模块/整机"（不夸大，不承诺自动恢复） |
| `SIGKILL`（`kill -9`）后立刻重启 | 释放不是进程主动做的 | 文档明确：`kill -9` 后建议先 `sleep 2` 再启动，并把"必须 pkill 残留进程"写进排查步骤 |
| FSYNC 一直不来导致启动挂住 | IMU 未探测/接线问题 | `startup_timeout_s`（默认 5 s）到时 fatal 退出，保证"挂住"不会表现为假健康 |

**验收判据**：连续 3 次"启动 → 看画面 → Ctrl-C → 立即重启"不出现相机占用错误；第 3 次之后 `ps -ef` 无残留。

---

## 2. 与现有 TROS mipi_cam 链路的能力对照

| 维度 | 现状：mipi_cam 链路 | 本包：gs130_ros | 是否可混用 |
| --- | --- | --- | --- |
| 相机驱动 | D-Robotics `mipi_cam`，面向 RDK 官方 MIPI 模组 | GS130 SDK（本仓 `core/` + `python/gs130`），专属双 1088x1280 传感器 | **不可混用**：同一时刻只能有一个进程持有相机 |
| 图像话题 | NV12 `/image_combine_raw`（拼接） | NV12 `/image_combine_raw`（拼接，默认 left_right）+ `/gs130/left\|right/image_raw`（分目） | 名字故意一致：**下游 codec/websocket 无需改动**；但也因此两个驱动互相覆盖话题，绝不能并行 |
| Web 展示 | `/image_combine_raw` → `hobot_codec` → JPEG → `websocket` → nginx 8000 | 同一链路、同一 codec、同一 websocket、同一端口 | 可混用（下游是共用的），但上游只能有一个相机源 |
| IMU | 无（mipi_cam 不出 IMU 话题） | `/gs130/imu`，200 Hz，时间戳已对齐相机时钟，`sensor_msgs/Imu` | 不冲突（mipi_cam 侧无对应能力）；但把两者当同一时间基准使用是错的 |
| 双目同步 | 依赖传感器同步，SDK 侧无时间戳对齐保证 | 相机线程半帧对齐 + 拼接帧用绑定目时间戳；每帧带 `timestamp_ns` | 不可混用同一套配对逻辑 |
| 标定 | 用户自备标定文件/参数，与相机无绑定 | EEPROM 内标定，可导出 camchain/imu YAML，并可直接查询内参/外参 | 不可混用：mipi_cam 的标定对应它的模组与矫正设置，换到 GS130 上无意义 |
| 深度（stereonet） | 官方参考组合，已在线路上验证 | 图像侧兼容，标定与 launch 归属未验证（SC-05） | 需满足 §SC-05 前置条件；**不得**为此外挂 mipi_cam |
| 独占资源 | camera I2C（bus 4/6）、MIPI RX、复位 GPIO、ISP/GDC | 同上 | **硬约束**：`mipi_cam` 与 `gs130_ros` 同时运行 = 未定义行为（init 失败或硬件错误） |
| 验收口径 | 官方链路可用 | 见 §3："充分展示硬件"只算双目帧 + IMU + 标定 | — |

**必须写进文档的一行结论**：下游（`hobot_codec` / `websocket` / `hobot_shm` / RViz / Foxglove）可以与我们共存；上游相机驱动（`mipi_cam`）与我们**互斥**。

---

## 3. "充分展示硬件"的定义（限定范围）

本项目的"充分展示硬件"仅指以下三项**从 SDK 真机产出**的能力，且都以可复现命令 + 可观察结果为准：

| 编号 | 能力 | 判据 | 证据形式 |
| --- | --- | --- | --- |
| D-1 | 双目帧 | 左右目各自独立、内容正确、时间戳可在半帧周期内配对；拼接模式方向正确（遮左目只动左半边）；帧率 ≈ `fps` | `ros2 topic hz` + Web 实时画面 + 一次遮挡手势 |
| D-2 | IMU 数据流 | `/gs130/imu` 平均 ≈ `odr_hz`，`linear_acceleration`/`angular_velocity`/`temperature` 有合理物理响应（静止 ≈ 1g；转动时角速度符号正确） | `ros2 topic hz` + `ros2 topic echo` 采样 + 一次手动转动 |
| D-3 | 标定/内参/外参 | 能导出并读出左右目内参 + 畸变模型 + 相对外参（双目与 IMU-相机），且数值来自本机 EEPROM | `core/samples/gs130-calib-export` 产出的 `camchain.yaml` / `imu.yaml` + Python 读取输出 |

明确**不算**本项目工作、也不进入验收的内容：
1. 深度图、点云、SLAM 结果——若由 `hobot_stereonet` 等 D-Robotics 节点产出，属**复用**，只统计"能否订阅到我们的图像"这一件事（且按 SC-05 延后）。
2. 任何需要自己训练/移植网络的能力。
3. 需要额外硬件（如标定板、外部触发源）才能证明的指标。

---

## 4. 非目标与绝对不做的事

1. **不写自己的 Web 页面 / 不写前端服务**：展示一律用现成 `websocket` + nginx 8000；不改它的前端资源。
2. **不写自己的编解码**：JPEG 编解码一律用 `hobot_codec`；不在节点里做 NV12→JPEG，也不引入软件编码器。
3. **不写自己的深度网络 / 不做点云**：深度能力只以复用 D-Robotics 节点的方式考虑（SC-05）。
4. **不支持 ROS 1**：只做 TROS humble（ROS 2）；不提供 rospy/rosbag v1 兼容层。
5. **不引入无法测试的硬件能力**：没有真机、没有 IMU、没有 EEPROM 的配置不写进验收路径；无法在板上验证的功能（如 SC-05 的深度闭环）不承诺、只记录为已知限制或延后项。
6. **不做与 mipi_cam 的运行时共存/仲裁**：不做自动抢占、不做"谁先启动谁赢"的调度；只做启动前检查与清晰报错。
7. **不修改 `python/` 下的绑定与 `core/` 的 C/C++ 实现**：本包是 wrapper。若发现 SDK 行为不满足 ROS 需求（例如需要"无 IMU 也能开流"的开关），走 SDK 变更流程，不在 ROS 层绕过。
8. **不暴露 SDK 内部纹理**：不新增私有消息类型（除非评审明确要求），优先用标准 `sensor_msgs`。
9. **不做标定自动重算/矫正算法**：矫正由 SDK 的 `mode:=rect`（GDC）或用户自己的流水线负责。

---

## 5. 优先级与验收范围

| 优先级 | 场景 | 理由 | v0.1.0 验收判据 |
| --- | --- | --- | --- |
| P0（必须） | SC-01 首次点亮 + Web 实时拼接画面 | 这是"能用"的最低门槛，也是唯一能一眼看出硬件正常的路径；完全复用现成链路，风险最低 | 按 SC-01 命令在 3 分钟内看到 1280x480@30fps 画面；遮挡左目验证方向 |
| P0（必须） | SC-02 消费 left/right | 没有分目数据，SDK 就只是"又一个相机驱动"；也是自研双目算法的唯一入口 | 两个话题稳定出帧、`encoding/step/尺寸` 正确、配对残差 ≤ 半帧 |
| P0（必须） | SC-03 IMU + 图像（时间戳语义） | IMU 是 GS130 相对 mipi_cam 的核心差异点；批量发布语义必须写进文档 | `/gs130/imu` ≈200 Hz、字段合理；文档含"IMU 每相机帧一批、最新包可能落后一帧"的明确说明 |
| P0（必须） | SC-04 标定导出与读取 | 深度/矫正流水线的前置，且已有导出工具，成本低收益高 | `camchain.yaml`/`imu.yaml` 可从 EEPROM 导出；Python 能读到内参与相对外参；命令强制 `raw` 模式 |
| P0（必须） | SC-06 外部 ROS 工具可见 | 决定这个包是不是"真 ROS 接口"；成本几乎为 0 | `ros2 topic list/hz/echo` 可用；文档写明 RViz/Foxglove 需看 JPEG 话题 |
| P0（必须） | SC-07 干净退出与重启 | 相机独占 + FSYNC 握手让"重启失败"极易发生；不做就是每次演示都靠运气 | 连续 3 次启动/退出无占用错误；占用时有可执行报错，启动挂起有 5 s 超时 |
| P1（延后 v0.2.0） | SC-05 stereonet 深度链路 | ① 标定格式与"谁做矫正"未验证；② stereonet 的 mipi_cam 开关未验证；③ 深度归 D-Robotics，不算我们的硬件展示证据。风险高、收益可由复用替代 | v0.1.0 只验收"stereonet 能订阅到 `/image_combine_raw`"或把阻塞项记录为已知限制 |
| P2（明确不在 v0.1.0） | IMU 与图像的时间同步输出（插值后的对齐流）、BGR 调试话题、rosbag 录制工具、多相机实例、`hobot_shm` 零拷贝优化 | 都是"锦上添花"或需要额外验证，且可以由用户侧完成 | — |

**一句话范围**：v0.1.0 = 相机（拼接/分目）+ IMU + 标定导出 + 现成 Web 展示 + 干净退出；深度类能力只做复用与记录，不做承诺。

---

## 6. 最可能的三个误用/困惑及预防

### M-1 混淆"拼接 vs 分目"，以及用错模式导出标定

- **误用**：默认以为能同时拿到拼接话题和分目话题；或直接 `read_image()["stitched"]` 后按分目代码去切；或在 `rect`/`resize` 模式下导出标定拿去搭深度流水线。
- **后果**：话题不存在（`UNSUPPORTED`）、图像被错误裁剪、深度/矫正结果系统性错误（二次矫正）。
- **预防**：
  1. launch 默认显式 `stereo_layout:=left_right`，并在启动日志里用一行中文写清"当前布局：left_right（左目在左，右目在右），拼接话题 `/image_combine_raw`；如需分目请 `stereo_layout:=none` 并重启"。
  2. `stereo_layout` 不接受运行中修改（改参数返回失败并提示重启），避免"改了没生效"的假象。
  3. SC-04 的命令与文档统一使用 `raw` 模式导标定，并在导出工具/文档中重复 SDK 的警告语义："rect/resize 模式导出的是矫正后/缩放后的标定"。

### M-2 把 IMU 当成等间隔连续流，并按到达顺序与图像配对

- **误用**：以为 IMU 是 200 Hz 均匀流、图像一到对应 IMU 就到齐；或用"每帧一个 IMU"的假设写融合代码。
- **后果**：时间对齐错误、融合结果滞后或发散；还会被"IMU 是突发批"误判为丢包/驱动有问题。
- **预防**：
  1. 文档在 SC-03 与 README 顶部各写一次："IMU 每相机帧成批发布，约 `odr_hz/fps` 包/批；最新 IMU 包最多落后最新图像一个帧周期（30 fps → 约 33 ms）；请按 `header.stamp` 对齐，不要按到达顺序配对。"
  2. 节点启动日志打印实际 `fps` / `odr_hz` 与由此推出的"批大小 ≈ N 包/帧、IMU 最大延迟 ≈ 33 ms"。
  3. 节点提供（可选）批次计数/频率统计日志，让用户能自证是批量而非丢包。

### M-3 "相机被占用/未握手"被误读为驱动坏了

- **误用**：上一次进程没退干净或 mipi_cam 还在跑就启动；或 FSYNC 没来（IMU 未接/未探测）导致话题一直空；或 `kill -9` 后马上重启。
- **后果**：init 失败或长时间无数据，用户以为硬件/驱动故障，反复重插相机。
- **预防**：
  1. 启动时先做占用检查（`ps -ef | grep -E "mipi_cam|gs130"`），发现疑似占用时打印**可执行的**处理命令（`pkill -f ...`），而不是只抛错误码。
  2. `startup_timeout_s` 默认 5 s：FSYNC 握手未完成即 fatal 退出，绝不以"进程活着但无数据"的形式呈现。
  3. 默认 `require_imu:=true`：IMU 未探测到时直接失败并说明"本设备无 IMU 或 IMU 未接，若确定要只出图请显式 `require_imu:=false`"，避免"相机没开流"被误判为相机故障。
  4. 文档给出排查顺序：① 有无残留/mipi_cam → ② 有无 FSYNC/IMU → ③ 话题与编码 → ④ 才怀疑硬件；并在 SC-07 明确 `kill -9` 后先 `sleep 2` 的建议。

---

## 附：评审待决问题（需要 freeze 的接口决策）

1. 节点默认分辨率/帧率：本文取 `resize 640x480@30`（Web 链路带宽可控）——是否改为 `raw 1088x1280@30`？
2. 默认编码是否保持 `nv12`（需 `hobot_codec` 板端确认 `in_format`），还是额外加一个可选 `bgr8` 调试输出（代价：每帧 CPU 转换）？
3. `require_imu` 默认值：本文取 `true`（早失败、语义清晰），是否接受"无 IMU 时默认降级为只出图"？
4. SC-05 是否允许在 v0.1.0 内做"仅验证订阅关系"的烟囱测试，还是完全移出 v0.1.0？
