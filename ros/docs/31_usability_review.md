# 31 可用性评审（UX-2）

- Reviewer: UX-2（可用性 / 开发者体验）
- Scope: 新增 ROS 2 (TROS) 接口包 `gs130_ros` 的首次运行体验、命令行与 launch 人体工学、
  topic 命名、错误文案、文档最小集、验收检查点
- 基准: `humble` (`/opt/tros/humble`), Python 3.10, `colcon`, 复用 `hobot_codec` /
  `websocket` / `hobot_shm` / `hobot_stereonet`
- 结论标记: **[blocking]** 必须在冻结接口前解决；**[must]** 一版内必须做；**[nice]** 可延后

This file is part of gs130_sdk (https://github.com/hachi-leaf/gs130_sdk).
Copyright (c) 2026 D-Robotics.
SPDX-License-Identifier: MIT
See the LICENSE file in the project root for the full license text.

---

## 0. 结论摘要（可执行清单）

| # | 决定 | 级别 | 交付物 |
|---|------|------|--------|
| 1 | 首次运行 = 一条命令 `ros2 launch gs130_ros gs130_web.launch.py`，零必填参数 | blocking | launch 文件 + 默认参数 |
| 2 | 首行输出 ≤1 s，Web UI URL ≤3 s，`first frame ok` ≤10 s | blocking | 启动日志与帧确认日志 |
| 3 | 首跑只允许 0 个必填参数，其余全部有板端默认值 | must | launch 参数表 |
| 4 | 参数错误一律 fail-fast，退出码 2，打印 `usage:` 一行 | must | 参数校验 |
| 5 | 单目图像用 `/<ns>/camera_left\|right/image_raw`，IMU 用 `/<ns>/imu/data`，内参用同名 `camera_info` | blocking | topic 命名表 §3.2 |
| 6 | 拼接帧沿用 `/image_combine_raw`（仅在方案 B 下启用，默认不发布） | blocking | §3.1 裁决 |
| 7 | 六类失败各有一条固定文案（原因 + 下一步命令） | must | 错误处理 §4 |
| 8 | 六份文档，NV12 陷阱写在启动日志与 `docs/02_topics.md` | must | 文档集 §5 |
| 9 | 首版删除深度、点云、IMU 融合、RGB 发布、动态参数 | must | 范围控制 §7 |
| 10 | 与 `01_requirements_review.md` 的三处命名/默认值冲突，冻结前必须裁决并回写该文档 | blocking | §2.2、§3.1、§3.3 |

---

## 1. 首次运行体验（first run）

### 1.1 唯一必须可用的命令

```bash
source /opt/tros/humble/setup.bash
ros2 launch gs130_ros gs130_web.launch.py
```

约束：

- 不要求 `--ros-args`，不要求设备型号，不要求分辨率，不要求 `sudo`。
- 一条命令内必须同时拉起：`gs130_camera`（出流）、`hobot_codec`（NV12→JPEG）、
  `websocket`（Web UI），用户只需打开浏览器。
- 禁止要求用户先 `ros2 run` 三个终端再自己配 remap —— 那是"能跑"，不是"首次运行体验"。
- 关闭旧栈：本命令独占相机，若 `mipi_cam` 或上一条本命令仍在运行必须明确报错（见 §4.3），
  不得静默卡住。
- **推荐 launch 文件名 `gs130_web.launch.py`**（web 链路是首跑目标的直接体现），
  同时在包内保留一个别名 `gs130_camera.launch.py` 指向同一实现，避免与 `01_requirements_review.md`
  AC-01 的命令互相打架。两个文件名都必须能在 README 里被搜到。

### 1.2 时间预算（超时即视为失败）

| 时点 | 必须出现的内容 | 上限 |
|------|----------------|------|
| 第 1 行日志 | 版本 + 即将使用的参数 | 1 s |
| Web UI URL 行 | `http://<board-ip>:8000` | 3 s |
| 首帧确认行 | `first frame ok` | 10 s |
| 浏览器可见画面 | Web UI channel 0 有图像 | 点击后 5 s |
| 运行稳定 | 连续 60 s 无丢帧告警、无崩溃 | 60 s |

> 10 s 是**硬上限**：初始化失败必须在 10 s 内变成一条错误文案，绝不能"永远转圈"。
> 若某个阶段确实可能更慢，必须在此之前打印 `waiting for first frame (n/10 s)` 之类的进度行。

### 1.3 成功输出形状（照抄实现）

```text
[gs130_ros] gs130_ros 0.1.0 (gs130_sdk 0.0.1, platform rdkx5)
[gs130_ros] device GS130WI  mode resize  output 1280x720  fps 30  imu 200 Hz
[gs130_ros] topics: /gs130/camera_left/image_raw (nv12) -> hobot_codec -> /gs130/image_jpeg
[gs130_ros] web ui: http://192.168.1.10:8000  channel 0     <- open this in a browser
[gs130_ros] note: channel 0 shows the LEFT eye only (publish_stitched:=false)
[gs130_ros] waiting for first frame ...
[gs130_camera] first frame ok: 1280x720 nv12, stamp 1738291200.123456789
[gs130_camera] streaming: 30.1 fps, published 30 frames, dropped 0
[gs130_camera] imu: 198.4 Hz, 40 packets per image (200/30)
```

要求：

- 每行 ≤ 96 字符，前缀固定 `[节点名]`，便于 `grep gs130_ros`。
- 第 4 行的 URL 必须是**可点击复制的完整地址**，不能只给 `port 8000`。
- 第 5 行的"只看左目"提示是**必须**的：否则用户会以为双目拼接没生效是 bug（对应 §3.1 方案 A）。
- `first frame ok` 一行必须打印真实分辨率、`nv12` 字样与时间戳，便于用户确认"不是合成图"。
- `imu:` 一行必须显示"每帧图像对应多少个 IMU 包"（见 §3.4、§5.3），这是最容易被误读的量。

### 1.4 三个最可能的失败（固定文案）

**失败 A：相机被占用（最常见，`mipi_cam` 或上一条命令未退出）**

```text
[gs130_camera] ERROR camera is busy: /dev/video0 is held by another process
[gs130_camera] ERROR likely holder: mipi_cam (pid 1183)
[gs130_camera] ERROR next: stop the other camera stack, then relaunch
[gs130_camera] ERROR   pkill -f mipi_cam        # or: ros2 node kill /mipi_cam
[gs130_camera] ERROR only one process may own the GS130 camera at a time
```

**失败 B：设备未检测到（未插好 / I2C 未上电 / 排线反接）**

```text
[gs130_camera] ERROR device not detected: no GS130 on i2c bus 4 or 6, no EEPROM at 0x50
[gs130_camera] ERROR next: check the FPC cable and the 12V supply, then scan the buses
[gs130_camera] ERROR   gs130 detect camera -b 4 6
[gs130_camera] ERROR   gs130 detect eeprom -b 4 6
```

**失败 C：没有帧（超时 10 s）**

```text
[gs130_camera] ERROR no frame within 10 s: the sensor opened but never delivered a frame
[gs130_camera] ERROR next: confirm no other process holds the camera, then retry in raw mode
[gs130_camera] ERROR   gs130 shell -p RDKX5 -d GS130WI -m raw -w 1088 -h 1280 -f 30 -o 200
[gs130_camera] ERROR   (inside the shell) run
[gs130_camera] ERROR report this with the log: ~/.ros/log/latest/gs130_camera_*.log
```

共同规则（对全部失败文案生效）：**[blocking]**

1. 每条错误都写全 `原因` + `下一步动作（可直接复制的命令）`，禁止只输出错误码。
2. 错误输出到 `stderr`，`ERROR` 前缀便于 grep；同一原因最多 4 行。
3. 失败后进程必须**非零退出并释放相机**，不得留下占用 `/dev/video0` 的僵尸进程。
4. 不得把 `GS130_TIMEOUT` 直接抛给用户（`gs130_get_nv12_frame() -> TIMEOUT` 不是用户语言）。

---

## 2. 命令行与 launch 人体工学

### 2.1 参数预算

| 类别 | 允许数量 | 规则 |
|------|----------|------|
| 首跑必填参数 | **0** | 任何必填参数都是首次运行的失败点 |
| 常见可调参数 | 3（`device` / `width`+`height` / `fps`） | 必须有板端默认值 |
| 高级参数 | 不限，但不得出现在 usage 首屏 | 仅记录在 `docs/03_launch_options.md` |
| 布尔开关 | 3（`ui` / `publish_imu` / `publish_camera_info`） | `true`/`false` 小写，默认 `true` |

### 2.2 默认值表（板端 D-Robotics 惯例）

| 参数 | 默认值 | 为什么是默认 |
|------|--------|--------------|
| `device` | `GS130WI` | 板端出货配置；无 IMU 时用户只改这一项 |
| `mode` | `resize` | `raw` 1088x1280 对 Web UI 与链路都过重 |
| `width` `height` | `1280` `720` | 与现有 `hobot_codec` / Web UI 链路一致 |
| `fps` | `30` | 与 `gs130 shell` 示例一致 |
| `imu_odr` | `200` | 与 SDK preset 实例一致（`30_user_feedback_round1.md` §3.4 同值） |
| `ns` | `gs130` | topic 可预测、可成组清理，也便于同时跑两套 |
| `publish_stitched` | `false` | 方案 A：公开 API 拿不到拼接帧，默认不宣称支持（见 §3.1） |
| `ui` | `true` | 首跑目标就是"浏览器里看到图" |
| `publish_imu` | `true`（`GS130W` 自动置 false） | GS130W 无 IMU，不能因此报错 |
| `publish_camera_info` | `true` | 有标定就发，缺了才像没做完 |

> **[blocking] 与 `01_requirements_review.md` AC-01 的冲突必须裁决**：AC-01 写的是
> `mode=RAW width=1088 height=1280 fps=30 odr=100`。若照此作为默认，首跑要跑 1088x1280@30
> 的 RAW NV12（约 62 MB/s/眼，双目 125 MB/s），web 链路与 CPU 都吃不消，首跑体验必然退化。
> 本评审坚持 **默认 `mode=resize 1280x720 fps=30 odr=200`**，把 `raw` 留给需要原图的人显式指定；
> `odr` 统一 200（`Config.preset` 示例与用户反馈文档同值，避免出现两套默认）。

### 2.3 必须显式给出的参数

只有以下三类必须显式，且仅在**偏离默认**时：

| 情形 | 必须显式 | 原因 |
|------|----------|------|
| 无 IMU 的 GS130W | `device:=GS130W` | 否则 IMU 相关节点起不来，报错要可读 |
| 需要原始分辨率 | `mode:=raw` | 与 `width/height` 必须等于 1088x1280，改小即非法 |
| 独占相机的其它进程共存 | 不允许 | 只能改流程，不给"共享相机"参数 |

### 2.4 错误参数：必须 fail-fast + 可执行提示

**[blocking]** 错误参数一律在**参数解析阶段**失败，退出码 `2`，且不初始化相机。
文案格式沿用仓库 `gs130` 前端的约定（`<command>: <reason>` + `try '<command> --help'`）：

```text
$ ros2 launch gs130_ros gs130_web.launch.py mode:=rgb
gs130_ros: bad value for 'mode': 'rgb' (expected raw | resize | rect)
try 'ros2 launch gs130_ros gs130_web.launch.py -s'

$ ros2 launch gs130_ros gs130_web.launch.py fps:=0
gs130_ros: bad value for 'fps': 0 (expected an integer in [1, 30])

$ ros2 launch gs130_ros gs130_web.launch.py mode:=raw width:=640
gs130_ros: 'raw' mode requires width=1088 height=1280, got 640x720
gs130_ros: use mode:=resize for a smaller image
```

规则：

- 报错必须回显**用户给的坏值**（`'rgb'`），只报"invalid mode"不合格。
- 必须给出合法取值集合或区间，并给一条"该改成什么"的替代方案。
- `raw` + 非 1088x1280 是最常见的误配，必须单独一条消息（如上）。

### 2.5 参数命名：好与坏

启动参数：

| 好 | 坏 | 原因 |
|----|----|------|
| `device:=GS130W` | `dev:=1` | 型号名可读、可 grep，与 `gs130 shell -d` 一致 |
| `width:=1280 height:=720` | `size:=1280x720` | 单值字符串无法做类型校验 |
| `fps:=30` | `rate:=30` | 与 SDK / C API / 文档统一用 `fps` |
| `imu_odr:=200` | `imu:=200` | ODR 是单位 Hz 的率参数，名字要说清 |
| `mode:=rect` | `rectify:=true` | 三态用 enum，不用布尔组合 |
| `publish_stitched:=true` | `combine:=1` | 布尔用 `true/false`，名字说明"发的是拼接图" |
| `ns:=gs130` | `prefix:=/gs130/` | 命名空间给不带斜杠的名字，避免 `//` 拼接错误 |
| `ui:=true` | `websocket:=1` | 用户心智是"要不要 Web UI"，不是中间件名字 |
| `publish_imu:=false` | `no_imu:=1` | 正能量命名，避免双重否定 |

节点/可执行文件命名（注意不要与 D-Robotics 现有名字冲突）：

| 好 | 坏 | 原因 |
|----|----|------|
| `gs130_camera` | `camera` / `image_publisher` | 碰撞风险高，`ros2 node list` 无法区分 |
| `gs130_web.launch.py` | `camera.launch.py` | launch 名要带包语义 |
| `gs130_nv12_probe`（已有） | `probe` | 保留现状 |

---

## 3. Topic 命名评审

### 3.1 是否沿用 `/image_combine_raw`：**argue for，但只在拼接帧上** [blocking]

| 支持沿用 | 反对沿用 |
|----------|----------|
| `hobot_codec` / `hobot_stereonet` 的默认订阅名就是它，沿用即零 remap | 名字含 `combine`，而 GS130 默认是**双目分离帧**，语义不吻合 |
| Web UI channel 0 与现有 stereo 示例一一对应，文档可直接复用 | 全项目唯一的 `combine` 词根会让人以为相机天然输出拼接图 |
| 不沿用就必须改 D-Robotics 节点的配置，破坏"复用现有节点"的目标 | `hobot_stereonet` 也可能占用该名，需要显式确认归属 |

**裁决**：拼接帧（`stereo_layout != NONE`）沿用 `/image_combine_raw`，**不沿用**的三个理由
都不足以抵消重映射成本。但必须在 `docs/02_topics.md` 第一句写明"这里的 combine 由
`stereo_layout` 参数决定，不是硬件属性"。**[must]**

**v0.1.0 的现实约束（必须写进 README，不许含糊）**：`python/gs130/_config.py` 的
`Config.preset()` 目前把 `stereo_layout` 硬编码为 `StereoLayout.NONE`，`read_image()` 因此只走
`{"left","right"}` 分支 —— **公开 API 拿不到拼接帧**（与 `01_requirements_review.md` Q-1 同结论）。
于是二选一，且必须由工程在冻结前定死一条：

| 方案 | 做法 | 代价 | 我的建议 |
|------|------|------|----------|
| A（首版默认） | 只发左右目 NV12，web 链路 remap 成 `hobot_codec sub_topic:=/<ns>/camera_left/image_raw`，Web UI 显示**单目** | 网页不是左右拼接，与 D-Robotics stereo 例子的观感不同 | **采纳**：不动绑定层，首版风险最低 |
| B（可选，N1） | 给 `Config.preset()` 增加 `stereo_layout` 参数，发布 `/<ns>/image_combine_raw` 拼接帧，Web UI 与 D-Robotics 例子**逐字节同构** | 要改 Python 绑定（本次评审范围内不允许我改） | 作为独立变更单，接口名沿用本文件 |

**[blocking]** 若最终选 B：`/<ns>/image_combine_raw` 必须与 `/image_combine_raw` 二选一，
并在 launch 中默认 `remappings` 到无命名空间名，否则用户抄 D-Robotics 的 launch 会接不上。

### 3.2 推荐最终命名

命名空间参数 `ns`，默认 `gs130`；下表写作 `/<ns>/...`，同时给出默认值下的完整名字。
**与 `30_user_feedback_round1.md` §3.2 的差异已在此收敛**（该文档写的是 `/<ns>/camera_left/image_raw`，
采纳它，因为它同时满足"`camera_info` 与图像同目录同基名"和"`left/right` 不做后缀"）。

| 数据 | 推荐 topic（默认 `ns=gs130`） | 类型 | 一行理由 |
|------|------------------------------|------|----------|
| 左目 | `/gs130/camera_left/image_raw` | `sensor_msgs/Image`, `nv12` | 与 `camera_info` 同基名，image_pipeline 零配置配对 |
| 右目 | `/gs130/camera_right/image_raw` | `sensor_msgs/Image`, `nv12` | 与左目严格对称，stereo 工具可自动配对 |
| 左目内参 | `/gs130/camera_left/camera_info` | `sensor_msgs/CameraInfo` | 与图像同 `frame_id`、同尺寸，缺一不可 |
| 右目内参 | `/gs130/camera_right/camera_info` | `sensor_msgs/CameraInfo` | 同上 |
| IMU | `/gs130/imu/data` | `sensor_msgs/Imu` | 与 `30_user_feedback_round1.md` §3.3 一致，采纳其决定，只填 accel+gyro |
| 拼接图像（方案 B，默认关闭） | `/image_combine_raw`（无 `ns`，或 `remappings` 到此名） | `sensor_msgs/Image`, `nv12` | 沿用 D-Robotics 惯例，`hobot_codec` 零配置直连 |
| 拼接帧内参 | 不发布 | — | 拼接帧没有单一内参，发布一份就是错误信息 |
| JPEG（`hobot_codec` 产出） | `/image_combine_jpeg` 或 `/<ns>/image_jpeg` | `sensor_msgs/CompressedImage` | 由 `hobot_codec` 的 `pub_topic` 决定，本包只文档化、不发布 |

`frame_id` 约定（与 `30_user_feedback_round1.md` §3.4 对齐）：
`camera_left_optical` / `camera_right_optical` / `imu_link`，全部可参数化。
首版**不发布** TF（见 §7），但 `docs/02_topics.md` 必须给出这四个 `frame_id` 的父子关系，
说明它们暂为孤立坐标系，避免下游误以为存在 TF 树。

### 3.3 会碰撞或被误认的名字 [blocking]

| 名字 | 风险 | 处理 |
|------|------|------|
| `/image_combine_raw` 由 `mipi_cam` 同时发布 | 两个发布者抢同一 topic，Web UI 图像随机闪烁 | 启动时检测已有发布者，命中即报 `§4.3` 文案并退出 |
| `/image_raw` / `/camera/image_raw` / `/image_combine_jpeg` | 与 `mipi_cam` 的 MIPI 相机话题混淆，用户无法判断图像来自哪台相机 | **禁止**本包使用无 `/<ns>/` 前缀的通用名（唯一例外是 §3.2 的 `/image_combine_raw`，且必须有裁决记录） |
| `/gs130/image_left_raw`（`01_requirements_review.md` AC-03 的写法） | 后缀式 `_left` 破坏 `camera_info` **同基名**约定，`image_proc` 配不上对 | **改为** `/gs130/camera_left/image_raw`；AC-03 需同步修订 |
| `/gs130/imu`（`01_requirements_review.md` AC-04 的写法） | 与 `30_user_feedback_round1.md` 的 `/gs130/imu/data` 不一致，团队会各写一半 | 统一取 `/gs130/imu/data`；AC-04 需同步修订 |
| `/imu/data`（无 `ns`） | 板载 IMU 或其它节点可能占用，且不符合本节 `ns` 规则 | 只用 `/gs130/imu/data` |
| `/hobot_stereonet/...` | 属于现有 D-Robotics 节点的输出，本包若同名会遮蔽 | 只订阅、只文档化，绝不同名发布 |
| `/left/image_raw`、`/right/image_raw`、`/camera_left/...`（无前缀） | 多相机系统下必然冲突 | 全部加 `/<ns>/` 前缀 |

---

## 4. 错误文案（逐字实现）

统一模板：`[节点] ERROR <现象>: <原因>` 紧跟 `[节点] ERROR next: <动作>（+ 可复制命令）`。
全部走 `stderr`，全部导致非零退出并释放相机。

### 4.1 库未找到

```text
[gs130_camera] ERROR libgs130 not found: gs130 python package is installed but the shared library is missing
[gs130_camera] ERROR next: install the SDK or point at the library explicitly
[gs130_camera] ERROR   sudo dpkg -i gs130_0.0.1+rdkx5_arm64.deb
[gs130_camera] ERROR   export GS130_LIB=/path/to/libgs130.so.0
```

原因：`libgs130.so.0` 未安装（`ctypes.util.find_library('gs130')` 为空），
或 `GS130_LIB` 指向的文件不存在/不是库。文案必须点名**实际查找过的东西**。

### 4.2 设备未检测到

```text
[gs130_camera] ERROR device not detected: no GS130 sensor on i2c bus 4 or 6, no EEPROM at 0x50
[gs130_camera] ERROR next: check the FPC cable and the 12V supply, then scan the buses
[gs130_camera] ERROR   gs130 detect camera -b 4 6
[gs130_camera] ERROR   gs130 detect eeprom -b 4 6
```

原因：I2C 探测全部失败（`GS130_NOT_FOUND`）。设备未插、排线反接、未上电、总线编号不符。

### 4.3 相机被其它进程占用

```text
[gs130_camera] ERROR camera is busy: /dev/video0 is held by another process
[gs130_camera] ERROR likely holder: mipi_cam (pid 1183)
[gs130_camera] ERROR next: stop the other camera stack, then relaunch
[gs130_camera] ERROR   pkill -f mipi_cam
[gs130_camera] ERROR only one process may own the GS130 camera at a time
```

原因：GS130 相机是独占资源。必须打印**建议的占用者与 pid**（从 `/proc/*/fd` 或
`fuser /dev/video0` 取），否则用户无从下手。若无法确定 pid，退化为
`ERROR   fuser -v /dev/video0`。

### 4.4 不支持的模式或分辨率

```text
[gs130_camera] ERROR unsupported mode/resolution: 'raw' requires width=1088 height=1280, got 640x720
[gs130_camera] ERROR next: use mode:=resize for a smaller image, or match the sensor size
[gs130_camera] ERROR   ros2 launch gs130_ros gs130_web.launch.py mode:=resize width:=640 height:=480
```

```text
[gs130_camera] ERROR unsupported mode: 'rect' needs EEPROM calibration, none was detected
[gs130_camera] ERROR next: use mode:=resize, or fix the EEPROM and relaunch
[gs130_camera] ERROR   gs130 detect eeprom -b 4 6
```

原因：`GS130_UNSUPPORTED` / `GS130_PARAM_ERROR`。必须**同时**给出错的值、合法值、替代命令。

### 4.5 超时无帧

```text
[gs130_camera] ERROR no frame within 10 s: the sensor opened but delivered no frame
[gs130_camera] ERROR next: confirm no other process holds the camera, then retry in raw mode
[gs130_camera] ERROR   gs130 shell -p RDKX5 -d GS130WI -m raw -w 1088 -h 1280 -f 30 -o 200
[gs130_camera] ERROR   (inside the shell) run
[gs130_camera] ERROR log: ~/.ros/log/latest/gs130_camera_*.log
```

原因：相机已 init/start，但 `read_image()` 在 10 s 内始终返回 `None`。
必须打印**已等待秒数**与日志路径，禁止无限等待（[blocking]）。

### 4.6 流运行中关闭

```text
^C
[gs130_camera] stopping: caught SIGINT, draining the pipeline
[gs130_camera] stopped: 412 frames in 13.7 s (30.1 fps), imu 2904 packets, dropped 0
[gs130_camera] camera released, exit 0
```

要求：

- 关闭是**成功路径**，不是错误路径：不得打印 traceback、不得留下 `Error` 字样。
- 必须打印帧数/时长/实际 fps/IMU 包数/丢帧数，用户据此判断本次采集是否可信。
- 第二次 Ctrl-C 立即强杀，文案 `forcing exit, camera may stay busy for a moment`。
- **[blocking]** 三条：不打印 traceback、不残留 `/dev/video0` 占用、退出码为 0。

---

## 5. 文档评审

### 5.1 最小文档集

| 文件 | 必须包含的章节 | 读者 |
|------|----------------|------|
| `ros/README.md` | 一句话定位、依赖、安装（`colcon build` + `source install/setup.bash`）、§1.1 那条命令、成功输出、`Ctrl-C` 退出、下一步链接 | 首次接触的用户 |
| `ros/docs/01_install.md` | TROS humble 前置检查、`libgs130` 安装（deb / `GS130_LIB`）、`colcon build` 步骤、验证命令（`ros2 pkg prefix gs130_ros`） | 部署者 |
| `ros/docs/02_topics.md` | §3.2 命名表（topic/type/encoding/rate/frame_id）、**NV12 说明**、拼接语义、QoS（`SensorDataQoS` / best effort）、带宽估算、D-Robotics 话题归属表 | 集成者 |
| `ros/docs/03_launch_options.md` | §2.2 默认值表、每个参数的类型/取值/默认值、`ui:=false` 只用话题的示例、remap 到 `hobot_stereonet` 的示例 | 集成者 |
| `ros/docs/04_troubleshooting.md` | §4 六条失败文案 + 排查命令 + `ros2 topic hz` / `ros2 topic echo --once` 自检步骤 | 现场用户 |
| `ros/docs/05_view_in_browser.md` | 板端 IP 获取（`hostname -I`）、`http://<ip>:8000`、选择 channel 0、画面为**左右拼接**、手机同一局域网访问、看不到图时的三条检查 | 演示/验收者 |

不再多写：首版禁止新增第 7 份文档；内容重复比缺文档更伤可用性。

### 5.2 四个已知陷阱的**指定安放位置** [blocking]

| 陷阱 | 指定位置（必须逐字出现） |
|------|--------------------------|
| NV12 不是 RGB | `docs/02_topics.md` 顶部 `## Encoding` 一节；同时 `README.md` 的 topic 表内以括注 `encoding=nv12, not RGB` 出现；`gs130_camera` 启动日志在 topic 行内打印 `(nv12)` |
| 每帧 IMU 节奏 | `docs/02_topics.md` 的 IMU 小节 `## IMU pacing`：`imu_odr / fps` 个包对应 1 帧，IMU 独立定时器发布，**不得**逐帧捆绑发布；并说明 `read_imu()` 返回 `None` 是正常空读 |
| 相机与 `mipi_cam` 互斥 | `docs/04_troubleshooting.md` 第一条（对应 §4.3）；`README.md` 的 Requirements 一行"the camera is exclusive: do not run `mipi_cam` at the same time"；`docs/03_launch_options.md` 也有交叉引用 |
| 浏览器看图 | `docs/05_view_in_browser.md` 全文；`README.md` 首次运行段落末尾一行直达链接 `See docs/05_view_in_browser.md` |

### 5.3 NV12 与 IMU 的两段必写文字（照抄）

```text
Encoding: images are NV12 (Y plane then interleaved UV plane), NOT RGB/BGR.
  width x height in the message header is the visible size (720 for a 1280x720 image);
  1.5 * width * height bytes of data are expected, so the row step is width.
  To decode:
    cv2.cvtColor(buf.reshape(h * 3 // 2, w), cv2.COLOR_YUV2BGR_NV12)
  A wrong encoding shows as washed-out green/grey or wrong-aspect stripes, not as an error.
```

```text
IMU pacing: IMU packets arrive at imu_odr (default 200 Hz) while images arrive at fps
  (default 30 Hz), so about 40 IMU packets correspond to one image.
  The IMU is published by its own timer at imu_rate, and each packet carries the
  timestamp aligned to the camera clock: pair an IMU packet with a frame by
  timestamp, never by arrival order.
```

---

## 6. 十个可用性检查点（通过 / 失败）

由评审人在项目末期逐条走查，每条只回答 PASS/FAIL，不写"基本可以"。

| # | 检查点（问句） | PASS 判据 |
|---|----------------|-----------|
| 1 | 全新终端执行 `source /opt/tros/humble/setup.bash && ros2 launch gs130_ros gs130_web.launch.py`，**10 s 内**是否打印 `first frame ok`？ | 从回车到该行的墙钟时间 ≤10 s |
| 2 | **不查文档**打开 `http://<board-ip>:8000` 并选 channel 0，是否 5 s 内看到清晰画面，且页面上的"只看左目"提示可被理解？ | 浏览器可见图，无绿灰偏色、无拉伸条纹 |
| 3 | `ros2 topic hz /gs130/camera_left/image_raw` 是否稳定在 30±1 Hz，且 60 s 内无丢帧告警？ | 输出均值与 `dropped 0` 一致 |
| 4 | 是否发布了双目 `camera_info`，且 `camera_info.width/height` 与同目录 `image_raw` 完全一致？ | 两侧四项数值一致；`ros2 topic echo --once --field encoding` 返回 `nv12` |
| 5 | 是否发布 `/gs130/imu/data`，其 `header.stamp` 与最近一帧图像时间戳之差是否 < 一个 IMU 周期？ | 差值 < 5 ms 且与 `first frame ok` 的时间戳同源 |
| 6 | 把 `/gs130/camera_left/image_raw` 当 RGB 用是否**明显出错**（即文档已警告）？ | README 与 `docs/02_topics.md` 都写明 `encoding=nv12` |
| 7 | 在 `mipi_cam` 已运行时启动本包，是否在 **5 s 内**报"camera is busy"并给出占用者与释放命令？ | 文案与 §4.3 一致，退出非零 |
| 8 | 拔掉相机后启动，是否在 **10 s 内**报"device not detected"并给出 `gs130 detect` 命令？ | 无 traceback，无无限重试 |
| 9 | 输入 `mode:=rgb fps:=0`、`mode:=raw width:=640`，是否**立即**（未初始化相机）给出可执行提示并退出码 2？ | 三条 §2.4 文案齐备 |
| 10 | 流运行中 `Ctrl-C`，是否打印本次帧数/耗时/实际 fps/IMU 包数，退出码 0，且 `mipi_cam` 或下一次启动能立刻拿到相机？ | 无 traceback，无残留占用 |

附加（不计入十条，但一版应满足）：空闲时 `gs130_camera` 单核 CPU < 35%；
`ros2 topic list` 中所有本包 topic 都能在 `docs/02_topics.md` 表里找到。

---

## 7. 范围建议：删除与补充

### 7.1 首版应删除（keep the first release simple）

| 删除项 | 理由 |
|--------|------|
| `hobot_stereonet` 深度/点云接入 | 属于第二个里程碑；首版只要"能看到图" |
| IMU 融合 / `sensor_msgs/Imu` 之外的姿态输出 | 融合是用户侧的事，发布 `data_raw` 即可 |
| 同时发布左右目 + 拼接三路 | 三路同带宽翻倍，首版默认只发拼接帧；单目由参数开启 |
| 逐帧捆绑 IMU 发布 | 违反 200/30 节奏，制造 30 Hz 的假 IMU 率 |
| RGB/BGR/JPEG 二次编码 | `hobot_codec` 已负责 JPEG，重复编码浪费 CPU 且造成两套图像话题 |
| 运行时动态参数（`ros2 param set` 改分辨率/模式） | 分辨率与拼接布局在 `gs130_init` 时定死，动态改必然不一致 |
| 首版发布 TF 树 | 标定已能从 EEPROM 读出，但外参参考系变换复杂（见 `gs130.h` 注释），首版盲发会误导下游 |
| 自定义消息（`.msg`） | 现有 `sensor_msgs` 足够，新增消息会拖慢构建与验收 |

### 7.2 首版应补充（缺了就像没做完）

| 补充项 | 理由 |
|--------|------|
| `gs130_ros/launch/gs130_web.launch.py`（带 §2.2 默认值） | §1.1 单命令成立的前提 |
| `gs130_ros/launch/gs130_topics_only.launch.py`（`ui:=false`） | 集成用户不想要 Web UI 时不要改 launch |
| `camera_info` 发布 | 双目相机不发内参，包看起来只有一半；数据 EEPROM 里已有 |
| 启动自检：发布者冲突检测 + 相机可用性探测 | 决定 §4.3 能否在 5 s 内触发 |
| 首帧确认与运行统计（`first frame ok` / `streaming: N fps` / `stopped: ...`） | 没有它，用户只能靠"浏览器有没有图"判断成败 |
| 每条 topic 的 QoS 显式声明（`SensorDataQoS`） | 与 `hobot_codec` / Web UI 的 best-effort 匹配，默认 reliable 会静默丢配 |
| `README.md` 顶部一行 `status: experimental, interfaces may change` | 与 `gs130_define.h` 中"preset values may change"的既有免责风格一致 |
| `.gitignore` 增加 `ros/build/ ros/install/ ros/log/`（colcon 在包目录内构建时的产物路径） | 首次 `colcon build` 就会产出，否则仓库立刻变脏 |
| `VERSION` 联动：`gs130_ros` 版本读同一份 `VERSION` | 避免出现两个版本号互相矛盾 |
| 把已有的 `ros/probe_nv12_publisher.py` 移到 `ros/tools/` 并在文件头注明"bring-up 用，不随包发布" | 顶层散落脚本会让用户误以为它是包的一部分（其 docstring 已如此声明） |
