# GS130 ROS 2 (TROS) 接口包 —— 验收标准与发布门禁（21_acceptance_criteria.md）

- 文档编号：21（Acceptance Criteria / Release Gate）
- 角色：QA-2（验收与质量门禁负责人）
- 被验收物：`ros/gs130_ros/**`（代码 + launch + 用户文档），发布标识 **v0.1.0**
- 目标平台：RDK X5 / Ubuntu 22.04 / TROS humble（`/opt/tros/humble`）/ Python 3.10 / GS130WI（双 1088×1280 MIPI + ICM-42688-P IMU + EEPROM 鱼眼标定）
- 上游文档：`11_interface_freeze.md`（**FROZEN，接口唯一权威**）、`01_requirements_review.md`、`10_architecture.md`、`12_implementation_plan.md`、`20_test_plan.md`
- 本文件不引入任何新的接口名称或默认值：§2 是 `11_interface_freeze.md` 的**验收镜像**（逐字一致，便于第三方比对）

---

## 0. 本文件的作用、权威性与判定规则

### 0.1 权威链

1. 对外接口（包名/节点名/话题/字段/参数/launch/错误行为）以 `11_interface_freeze.md`（状态 FROZEN）为基础；本文件 §2 是它的验收镜像，用于逐项比对。
2. **实测证据优先（CHIEF ARCHITECT 在 RDK X5 上的实测结论 E1–E7，见 §2.0）**：实测与冻结书冲突处**以实测为准**。§2 已按 E1（图像 QoS 必须 `RELIABLE`）与 E2（`Image.height` 必须是**真实高度 H**）修正；冻结书必须在发布前同步修正（阻塞项 `B-03`、`B-09`）。
3. 本文件的**判据、阈值、优先级、放行规则**由 QA-2 定义，是 v0.1.0 的**唯一验收依据**。判据不得由实现方单方面放宽；与实测冲突的判据必须整条替换，不得保留"或者"式的软化表述。
4. §11 列出**阻塞性澄清项**（`B-01`…`B-11`）：冻结书自身的自洽性问题、与实测证据的冲突、或与既有 D-Robotics 实现行为的冲突，**关闭前不得发布**。关闭方式 = 修改 `11_interface_freeze.md` + 本表 + `20_test_plan.md`（同一变更单），并以板上实测证据登记。
5. 四份协作文档（`01/10/12/20`）与冻结书/本文件冲突之处，一律以本文件与冻结书修正后的条文为准，且必须在验收开始前修正（`AC-IF-01` 阻塞项）。
6. 主观描述（"画面正常""基本稳定""感觉延迟低"）不作为任何判定依据。

| 级别 | 含义 | 放行要求 |
| --- | --- | --- |
| **P0** | 阻塞项 | 必须**全部通过**；任一 P0 FAIL ⇒ 不发布 |
| **P1** | 非阻塞项 | 允许 FAIL，但必须写入 `release_notes` 与用户文档"已知限制"，并给出后续计划 |

### 0.2 判定与证据规则

- 每条判据必须能被**未参与编码的第三方**在板上复现，只依据：本文件的命令、被验收物本身、板端 TROS/系统工具。
- 证据必须落盘到 `$EVID`（命令原文 + 完整 stdout/stderr + `rc=$?` + 截图/PNG 等），判定只看落盘证据。
- 阈值不可改写；命令因环境差异改写时必须记录改写内容与理由。
- 相机**独占**：任意时刻只允许一个进程持有；测试期间不得启动 `mipi_cam`（`11_interface_freeze.md` §8）。
- 每个用例结束后必须做清理自检：`pgrep -af "gs130_node|hobot_codec|websocket"` 为空；相机可被 SDK 复开（`AC-PERF-07` ③）。必须用 `pkill -9` 才能清理的用例**判 FAIL**（禁止用强杀掩盖残留缺陷）。

### 0.3 环境变量与通用前置（下文命令均假定已执行）

```bash
export REPO=$HOME/rdkx5_work/gs130_sdk
export WS=$HOME/ros2_ws                       # colcon 工作空间（工作区外，不进仓库）
export EVID=$REPO/ros/docs/evidence/release_v0.1.0
export BOARD_IP=<板卡 IP>
mkdir -p "$EVID"
source /opt/tros/humble/setup.bash
source $WS/install/setup.bash
```

```bash
# 前置自检（不通过则后续全部阻塞）
python3 -c "import gs130;print(gs130.__version__, gs130.library_version())"   # 无 RuntimeWarning
pgrep -af "mipi_cam|gs130|gs130_node"                                          # 期望：无输出
ros2 pkg prefix gs130_ros                                                      # 期望：$WS/install
ros2 pkg executables gs130_ros                                                 # 期望：含 gs130_node
```

---

## 1. 发布门禁清单（v0.1.0 Checklist）

### 1.1 G0 构建、版本、可复现
- [ ] `AC-BLD-01` 构建成功、包可被发现（P0）
- [ ] `AC-BLD-02` 入口 `gs130_node` 存在（P0）
- [ ] `AC-BLD-03` 干净 clone 复现通过（P0）
- [ ] `AC-BLD-04` 发布候选提交固定、工作区干净、`python/`+`core/` 未改（P0）
- [ ] `AC-BLD-05` 两个 launch 文件语法与参数集合正确（P0）
- [ ] `AC-BLD-06` 主机侧语法/导入检查通过（无硬件）（P0）
- [ ] `AC-BLD-07` 包版本与许可证字段正确（P0）

### 1.2 G1 功能
- [ ] `AC-FUN-01` 单命令 bring-up 拉起 相机+codec+web（P0）
- [ ] `AC-FUN-02` T1/T2/T3 图像消息字段逐一正确（`height=H`，E2）（P0）
- [ ] `AC-FUN-03` T1 为硬件拼接、几何正确、左右目位置正确（遮挡法）（P0）
- [ ] `AC-FUN-04` 图像可解码、非纯色、无绿边/错行（P0）
- [ ] `AC-FUN-05` IMU 静止数值物理合理、协方差约定正确（P0）
- [ ] `AC-FUN-06` IMU 单位/轴向与 SDK 直读一致（P0）
- [ ] `AC-FUN-07` CameraInfo 逐字段正确、与 SDK K 一致、latch 可达（P0）
- [ ] `AC-FUN-08` 不发布 TF（契约决定）且文档说明外参获取方式（P0）
- [ ] `AC-FUN-09` 现有 web UI（:8000）显示实时且正确的画面；codec 实收且**不崩溃**（P0）
- [ ] `AC-FUN-10` 参数默认值可读、统计日志逐字符合契约（P0）
- [ ] `AC-FUN-11` `device:=GS130W` 优雅降级（P0）
- [ ] `AC-FUN-12` `/gs130/status` 按契约工作（P1）

### 1.3 G2 定量
- [ ] `AC-PERF-01` T1 速率 `[29,31] Hz`（fps=30，窗口 ≥200 样本；E3）（P0）
- [ ] `AC-PERF-02` T1 与 T7 速率均 `[29,31] Hz`，且 T2/T3（若开启）一致（E3/E1）（P0）
- [ ] `AC-PERF-03` IMU 平均速率在 `odr` 容差内且不丢包（P0）
- [ ] `AC-PERF-04` 启动到首帧 ≤20 s、到网页 ≤30 s（P0）
- [ ] `AC-PERF-05` 端到端时间戳延迟在界内（E5）（P0）
- [ ] `AC-PERF-06` 时间戳单调、图像与 IMU 同源（同域）（P0）
- [ ] `AC-PERF-07` SIGINT ≤3 s 退出码 0、相机 ≤10 s 可复开、**无残留进程、端口回基线**（E4）（P0）
- [ ] `AC-RES-01` 内存/句柄/线程不泄漏（P0）
- [ ] `AC-RES-02` CPU 在界内（P0）
- [ ] `AC-RES-03` ≥60 min soak 稳定（P1）
- [ ] `AC-RES-04` 无订阅者时无积压、无劣化（P0）

### 1.4 G3 代码质量
- [ ] `AC-SRC-01` 模块与公开函数均有 docstring（P0）
- [ ] `AC-SRC-02` 模块/函数体积在限内（P0）
- [ ] `AC-SRC-03` 无未使用参数/导入/局部变量；launch 参数全部被使用（P0）
- [ ] `AC-SRC-04` 无死代码、无静默吞异常（P0）
- [ ] `AC-SRC-05` 未自研 web/codec/depth 组件（P0）
- [ ] `AC-SRC-06` D-Robotics 节点按契约参数启动、不改既有节点（P0）
- [ ] `AC-SRC-07` 无每帧日志；统计日志按契约周期与格式（P0）
- [ ] `AC-SRC-08` 错误信息与退出码逐字符合契约（P0）
- [ ] `AC-SRC-09` 未写入/未 monkeypatch SDK（P0）
- [ ] `AC-SRC-10` 未实现非目标消息/参数（无 TF、无温度话题、无自定义 msg）（P0）
- [ ] `AC-SRC-11` 无非目标参数、无 `respawn=True`（P0）
- [ ] `AC-SRC-12` 纯函数层主机侧单测通过（P1）
- [ ] `AC-SRC-13` `data` 赋值形式为整块缓冲（E3 佐证）（P1）

### 1.5 G4 接口契约
- [ ] `AC-IF-01` §11 阻塞项全部关闭、四份文档与冻结书一致（阻塞）（P0）
- [ ] `AC-IF-02` 话题名/类型/QoS 四项/`frame_id` 与冻结表逐字一致（P0）
- [ ] `AC-IF-03` 节点参数与 launch 参数的名字、默认值、类型一致（P0）
- [ ] `AC-IF-04` 每个参数都有可观测效应（P0）
- [ ] `AC-IF-05` 消息逐字段冻结项（Imu/CameraInfo/Image）全部满足（P0）
- [ ] `AC-IF-06` 时间戳公式与降级规则按契约实现（P0）
- [ ] `AC-IF-07` launch 启动的节点集合与既有节点参数逐字一致（P0）

### 1.6 G5 鲁棒性
- [ ] `AC-RB-01` 相机被占用：退出码 1 + 点名错误，不影响已运行实例（P0）
- [ ] `AC-RB-02` 缺库：明确报错、非 0 退出、无段错误（P0）
- [ ] `AC-RB-03` 非法参数：退出码 2/1 与逐字错误文本（P0）
- [ ] `AC-RB-04` `raw` + 非原生分辨率：退出码 2 + 逐字提示（P0）
- [ ] `AC-RB-05` 20 轮启停全部成功、无残留、相机释放（P0）
- [ ] `AC-RB-06` 启动中途 SIGINT：按契约告警并最终干净退出（P0）
- [ ] `AC-RB-07` SIGKILL 后按文档恢复（P1）
- [ ] `AC-RB-08` 运行期相机通路故障：退出码 1；IMU 通路故障：非致命（P0，注入）
- [ ] `AC-RB-09` 标定缺失 / `rect` 无标定：降级与退出码正确（P0）
- [ ] `AC-RB-10` 无订阅者 60 s：无积压、无劣化（P0）
- [ ] `AC-RB-11` 无 IMU：图像正常、无 IMU 话题、WARN 一次（P0）
- [ ] `AC-RB-12` 时间戳为 0 / 非单调：按契约降级（P0，注入）
- [ ] `AC-RB-13` `use_sim_time:=true`：WARN 且继续发布（P0）
- [ ] `AC-RB-14` 图像字段错误（`height=1.5H`）场景下 codec 段错误为**硬失败**，且实现不得产生该场景（P0）
- [ ] `AC-RB-15` 关停后无孤儿进程、端口 8000 占用回基线（E4）（P0）

### 1.7 G6 文档
- [ ] `AC-DOC-01` 用户文档齐备（安装/启动/参数/话题/web/排查）（P0）
- [ ] `AC-DOC-02` 第三方按文档 30 min 内看到画面、提问次数 = 0（P0）
- [ ] `AC-DOC-03` 文档命令逐字可执行（抽查 ≥10 条全通过）（P0）
- [ ] `AC-DOC-04` 文档参数表/话题表与冻结书逐字一致（P0）
- [ ] `AC-DOC-05` 故障排查覆盖 ≥9 个最可能失败场景（P0）
- [ ] `AC-DOC-06` 必写的契约说明齐备（独占警告、显示语义、时间戳、丢弃项、非目标）（P0）
- [ ] `AC-DOC-07` 中文说明 + 英文标识符；命令 source 顺序统一（P1）

---

## 2. 冻结接口契约（验收镜像，以 `11_interface_freeze.md` 为基础 + 实测修正）

> 本节的每一个值都是判定基准。判定时以本节为准；若与冻结书不一致，说明该处已被 §2.0 的实测证据修正，冻结书须同步。

### 2.0 实测证据（E1–E7，CHIEF ARCHITECT，RDK X5 实机）与由此产生的契约修正

| # | 实测结论（可直接作为判据依据） | 对本契约的影响 |
| --- | --- | --- |
| **E1** | **图像话题必须是 `RELIABLE`**：`BEST_EFFORT` 时既有 `hobot_codec` 打印 `offering incompatible QoS ... RELIABILITY_QOS_POLICY` 并**收到 0 条消息** | §2.2 中 T1/T2/T3 的 reliability 由 `BEST_EFFORT` **改为 `RELIABLE`**；判据必须同时验证"publisher QoS = RELIABLE"与"codec 实际收到消息"（`AC-IF-02`、`AC-FUN-09`⑦、`AC-PERF-02`） |
| **E2** | **`sensor_msgs/Image.height` 必须是真实高度 `H`**（不是打包高度 `H*3//2`）：填 `1.5H` 时 codec 打印 `init_pic_h_: 1080, alined_pic_h_: 1088` 并**段错误（退出码 `-11`）** | §2.3 的 Image 字段表按 `height=H`、`step=width`、`len=width*height*3//2` 修正；新增"codec 不得崩溃（`-11`/`139` 为硬失败）"判据（`AC-FUN-09`⑥、`AC-RB-14`） |
| **E3** | **发布路径性能**：把字节逐个赋给 `msg.data` 时 **1103.87 ms/帧**（1.38 MB）；用 `array.array("B") + frombytes` 时 **1.36 ms/帧**。实测链路速率：`/image_combine_raw` **29.966 Hz**、`/image_combine_jpeg` **29.974 Hz**（窗口 >200 样本，配置 30 fps） | §5 的图像速率判据由 `[27.0, 31.5]` **收紧为 `[29, 31] Hz`（fps=30）**，并**同时适用于 T1 与 T7**（`AC-PERF-01`、`AC-PERF-02`）；新增 `AC-SRC-13`（数据赋值形式）作为 P1 佐证 |
| **E4** | **完整复用链在 30 fps 下工作**，`http://<board-ip>:8000/` 返回 **HTTP 200**；但 `websocket.launch.py` 通过 `os.system` 启动的 **nginx 在 launch 退出后成为孤儿进程存活** | §2.6 增加备注；`AC-PERF-07` 增加"关停后无残留进程 + 端口 8000 占用回到启动前基线"判据（`AC-RB-15`） |
| **E5** | **SDK 时间戳是开机以来的 `CLOCK_MONOTONIC`，不是 Unix 纪元**（`frame_ts=9546841822000` vs monotonic `9540690105753`，与 uptime 9480 s 相符；与墙钟差 `-1.79e18` ns）。图像与 IMU **同域**（`imu - frame = 8.6 ms`） | §2.5 补充实测值与"≤ 一个帧周期的不确定性"文档要求；`AC-IF-06` 增加"默认配置下 `header.stamp` 必须落在系统时钟域（与 `date +%s` 差 ≤ 60 s）"与"`stamp_offset_mode:=device` 时明显不同（证明未透传）"两条判据；`AC-DOC-06` 增加文档要求 |
| **E6** | **硬件拼接可用**：在 `Config.preset()` 之后设置 `stereo_layout = StereoLayout.LEFT_RIGHT`，640×480 配置下得到形状 `(720, 1280)` 的拼接帧，正好映射到 `width=2W, height=H` 约定 | §2.3 的拼接布局改为"**使用 SDK 硬件拼接**"；**不得要求也不得奖励任何手写拼接代码**；`AC-FUN-03` 改为"几何 + 左右目位置（遮挡法）"判据；`B-01` 关闭（冻结书的 `concatenate` 示例必须删除） |
| **E7** | **RAW 模式必须输出 1088×1280，否则 `gs130_init` 返回 `PARAM_ERROR`**；websocket 的 `channel` 参数在同一端口 8000 上区分多路流，**不需要第二个端口**；板端已装 colcon / rclpy / sensor_msgs / tf2_msgs / OpenCV 4.11.0 / numpy 1.26.4 | §4/§7 的拼接几何、速率、web 判据一律在 `mode=resize`（或 `rect`）下执行，`raw` 只验证 1088×1280 约束与退出码；`B-02` 收敛为"文档给出 channel 映射即可，不需第二个端口" |


### 2.1 包、节点与入口

| 项 | 冻结值 |
| --- | --- |
| 包名 | `gs130_ros`（`ament_python`） |
| 节点名 / 可执行名 | `gs130_node`（唯一节点、唯一可执行） |
| 包版本 | `0.1.0` |
| 源码布局 | `ros/gs130_ros/`（`package.xml`、`setup.py`、`setup.cfg`、`resource/gs130_ros`、`gs130_ros/`、`launch/`） |
| 一键启动（web 链路） | `ros2 launch gs130_ros gs130_web.launch.py` |
| 仅相机 | `ros2 launch gs130_ros gs130_camera.launch.py` |
| 直接运行节点 | `ros2 run gs130_ros gs130_node`（所有参数自带默认值） |
| 依赖 | `rclpy`、`sensor_msgs`、`diagnostic_msgs`；**不依赖** `tf2_msgs`、`geometry_msgs` |

### 2.2 话题表（8 条，含必需/可选）

| # | 话题 | 类型 | Reliability | Durability | History | Depth | `frame_id`（默认） | 频率 | v0.1.0 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| T1 | `/image_combine_raw` | `sensor_msgs/msg/Image` | **`RELIABLE`**（E1 实测修正） | `VOLATILE` | `KEEP_LAST` | `1` | `camera` | `fps` | **必需** |
| T2 | `/image_left_raw` | `sensor_msgs/msg/Image` | **`RELIABLE`**（E1） | `VOLATILE` | `KEEP_LAST` | `1` | `camera_left` | `fps` | 可选，默认关闭（`publish_per_eye:=true`） |
| T3 | `/image_right_raw` | `sensor_msgs/msg/Image` | **`RELIABLE`**（E1） | `VOLATILE` | `KEEP_LAST` | `1` | `camera_right` | `fps` | 可选，默认关闭 |
| T4 | `/imu/data` | `sensor_msgs/msg/Imu` | `BEST_EFFORT` | `VOLATILE` | `KEEP_LAST` | `200`（参数 `imu_qos_depth`） | `imu_link` | `odr`（突发） | **必需**（`device:=GS130W` 时不存在） |
| T5 | `/image_left/camera_info` | `sensor_msgs/msg/CameraInfo` | `RELIABLE` | `TRANSIENT_LOCAL` | `KEEP_LAST` | `1` | `camera_left` | 启动时 1 次（latched） | **必需** |
| T6 | `/image_right/camera_info` | `sensor_msgs/msg/CameraInfo` | `RELIABLE` | `TRANSIENT_LOCAL` | `KEEP_LAST` | `1` | `camera_right` | 启动时 1 次（latched） | **必需** |
| T7 | `/image_combine_jpeg` | `sensor_msgs/msg/CompressedImage` | 由 `hobot_codec_republish` 决定 | 同左 | — | — | — | `fps` | 复用链路（仅 `gs130_web.launch.py`） |
| T8 | `/gs130/status` | `diagnostic_msgs/msg/DiagnosticStatus` | `RELIABLE` | `VOLATILE` | `KEEP_LAST` | `1` | 无 | 1 Hz | 可选，默认关闭（`publish_status:=true`） |

**E1 说明（必须体现在实现与判据中）**：图像三话题的 reliability **必须**为 `RELIABLE`。若发布为 `BEST_EFFORT`，既有 `hobot_codec_republish` 会打印 `offering incompatible QoS ... RELIABILITY_QOS_POLICY` 且收到 0 条消息——此时 web 页面必然无图。因此 `AC-IF-02` 必须断言 `RELIABLE`，`AC-FUN-09`⑦ / `AC-PERF-02` 必须断言 codec **实际收到**消息（仅"话题存在"不算通过）。
**T4 说明**：`/imu/data` 的 QoS 未被 codec 链路实测覆盖（冻结值 `BEST_EFFORT` + depth 200）；订阅方必须使用匹配 QoS（`ros2 topic echo` 会自动适配）。该值若在下游出现兼容性问题，按 §13 `K-10` 登记，不作为 v0.1.0 阻塞项。


### 2.3 消息字段冻结项（`AC-IF-05` 的判定基准）

**Image（T1/T2/T3）**：`ew`=单目宽、`eh`=单目高（**真实高度**）、T1 的 `cw = 2*ew`

| 字段 | T1 `/image_combine_raw` | T2/T3 单目 |
| --- | --- | --- |
| `height` | **`eh`（默认 `480`）**——真实高度，**不是** `eh*3//2`（E2 实测修正） | 同左（默认 `480`） |
| `width` | `2*ew`（默认 `1280`） | `ew`（默认 `640`） |
| `step` | `2*ew`（`1280`） | `ew`（`640`） |
| `encoding` | 逐字 `"nv12"`（不得 `NV12`/`yuv420`） | 同左 |
| `is_bigendian` | `0` | `0` |
| `len(data)` | `2*ew*eh*3//2`（`921600`） | `ew*eh*3//2`（`460800`） |
| `header.stamp` | 同帧**左目**时间戳（按 §2.5 换算到系统时钟域） | 各自同帧时间戳 |
| `header.frame_id` | `frame_id_combine`（`camera`） | `frame_id_left`/`frame_id_right` |
| 发布条件 | 每次读到的帧对**只发一次**；`read_image()` 返回 `None` 时不发、不报错、不退出 | 同一份帧数据、同一 `stamp`，禁止二次读设备凑对 |

**`data` 赋值方式（E3，性能判据 `AC-PERF-01` 的实现前提）**：必须一次性提供整块缓冲（`array.array("B")` + `frombytes`，或 `bytes()`/`tobytes()`）。**禁止**逐元素字节赋值（实测 1103.87 ms/帧 ⇒ 30 fps 不可能达标）。见 `AC-SRC-13`（P1 佐证）。

**拼接布局（T1，E6）**：**必须使用 SDK 硬件拼接**——在 `Config.preset()` 之后设置 `camera_config.stereo_layout = StereoLayout.LEFT_RIGHT`；640×480 配置下 SDK 返回形状 `(720, 1280)` 的拼接帧，其几何与本节一致：`width=2*ew=1280`、`height=eh=480`、`step=1280`、`len=2*ew*eh*3//2=921600`。语义为**左目在左、右目在右**。
- **不得要求、也不得奖励任何手写拼接代码**（`hstack`/`concatenate` 拼 NV12）。若实现自行重组像素，必须自行承担 `AC-PERF-01` 的速率判据与 `AC-FUN-03` 的位置判据，并在文档中说明理由（风险登记，不作为加分项）。
- 左右目位置用**遮挡法**判定（`AC-FUN-03`）：遮挡左目时拼接图左半幅平均亮度下降 ≥50%，右半幅变化 ≤10%。


**Imu（T4）**：

| 字段 | 冻结值 |
| --- | --- |
| `orientation.x/y/z` | `0.0`；`orientation.w` = `1.0` |
| `orientation_covariance[0]` | **`-1.0`**（`[1..8]` = 0.0） |
| `angular_velocity.x/y/z` | SDK `gyro[0/1/2]`，单位 **rad/s**，符号与轴向**原样** |
| `angular_velocity_covariance[0]` | **`-1.0`**（`[1..8]` = 0.0） |
| `linear_acceleration.x/y/z` | SDK `accel[0/1/2]`，单位 **m/s²**，**含重力**，符号与轴向原样 |
| `linear_acceleration_covariance[0]` | **`-1.0`**（`[1..8]` = 0.0） |
| `header.stamp` | `ImuPacket.timestamp_ns` 按 §2.5 换算（与图像同域） |
| 温度 | **丢弃**（不发布话题、不发自定义消息） |
| `is_fsync` | **不发布** |
| 发布时序 | FSYNC 锚点后**突发**；契约只保证不丢包、不重排、`stamp` 非递减。**不得**据此判定"等间隔" |

**CameraInfo（T5/T6）**：

| 字段 | 冻结值 |
| --- | --- |
| `width` / `height` | `ew` / `eh`（默认 `640`/`480`，**不是** `eh*3//2`） |
| `K` | `[fx,0,cx, 0,fy,cy, 0,0,1]`，逐元素复制 SDK `camera_intrinsics()` 的 `K`；驱动**不得**再乘任何缩放系数 |
| `distortion_model` / `D` | `raw`/`resize` + FISHEYE ⇒ `"equidistant"` + `D=[d0..d3]`；`raw`/`resize` + PINHOLE ⇒ `"plumb_bob"` + `D=[k1,k2,p1,p2,k3]`；`mode=rect` ⇒ **强制** `"plumb_bob"` + `D=[0,0,0,0,0]`；`camera_info_distortion_model:=none` ⇒ `D=[]` |
| `R` | 9 个 `0.0` |
| `P` | 12 个 `0.0` |
| `binning_x` / `binning_y` | `0` / `0` |
| `roi.*` | 全 `0`；`roi.do_rectify` = `False` |
| `header.stamp` | 启动时刻（**不得为 0**） |
| `header.frame_id` | 与对应 Image 完全一致（`camera_left` / `camera_right`） |
| 发布方式 | 启动时各 1 次，latched（`RELIABLE` + `TRANSIENT_LOCAL`） |

### 2.4 参数表（节点参数，`Reconfig=No`：运行期修改无效）

| 参数 | 类型 | 默认 | 取值 |
| --- | --- | --- | --- |
| `platform` | string | `RDKX5` | 仅 `RDKX5` |
| `device` | string | `GS130WI` | `GS130WI` / `GS130W` |
| `mode` | string | `resize` | `raw` / `resize` / `rect` |
| `width` | int | `640` | `[16,1088]` 偶数；`raw` 时必须 `1088` |
| `height` | int | `480` | `[16,1280]` 偶数；`raw` 时必须 `1280` |
| `fps` | int | `30` | `[1,33]` |
| `odr` | int | `200` | 仅 `200`、`500` |
| `frame_id_combine` | string | `camera` | 非空 |
| `frame_id_left` | string | `camera_left` | 非空 |
| `frame_id_right` | string | `camera_right` | 非空 |
| `frame_id_imu` | string | `imu_link` | 非空 |
| `publish_combine` | bool | `true` | 关闭将断开 web 链路 |
| `publish_per_eye` | bool | `false` | 控制 T2/T3 |
| `publish_imu` | bool | `true` | 关闭仅停止发布（SDK 仍启动 IMU 完成 FSYNC 握手） |
| `publish_camera_info` | bool | `true` | 控制 T5/T6 |
| `publish_status` | bool | `false` | 控制 T8 |
| `stamp_offset_ns` | int | `0` | `0` = 自动；非 0 = 强制偏移 |
| `stamp_offset_mode` | string | `auto` | `auto` / `device`（不推荐） |
| `camera_info_distortion_model` | string | `auto` | `auto` / `plumb_bob` / `equidistant` / `none` |
| `poll_period_ms` | int | `2` | `[1,50]` |
| `imu_qos_depth` | int | `200` | `[1,2000]` |
| `image_qos_depth` | int | `1` | `[1,100]` |
| `log_fps_period_s` | double | `5.0` | `(0,600]`；`0` 关闭 |
| `start_timeout_s` | double | `10.0` | `[0.1,600]` |

**launch 参数（`gs130_web.launch.py` 额外声明，`11` §6.4）**：`codec_channel=0`、`websocket_channel=1`、`web_port=8000`、`web_output_fps=30`、`jpg_quality=80`、`smart_topic=/image_combine_jpeg`、`hobot_shm=true`。
**两个 launch 共同声明的 18 个 launch 参数**：`platform`、`device`、`mode`、`width`、`height`、`fps`、`odr`、`frame_id_combine`、`frame_id_left`、`frame_id_right`、`frame_id_imu`、`publish_combine`、`publish_per_eye`、`publish_imu`、`publish_camera_info`、`publish_status`、`stamp_offset_mode`、`camera_info_distortion_model`（默认值与 §2.4 同名参数一致）。

### 2.5 时间戳公式（唯一版本，含 E5 实测依据）

**实测（E5）**：SDK 的 `timestamp_ns` 是**开机以来的 `CLOCK_MONOTONIC`**，不是 Unix 纪元——`frame_ts = 9546841822000` ns（≈9547 s）与系统 monotonic `9540690105753`、uptime 9480 s 相符，与墙钟相差 `-1.79e18` ns。图像与 IMU **同域**（实测 `imu - frame = 8.6 ms`）。因此**直接透传 SDK 纳秒值到 `header.stamp` 是不可接受的**。

```
启动时（首次拿到有效设备时间戳）：
  offset_ns = (节点时钟 now_ns) - device_ts_ns        # stamp_offset_ns == 0
  offset_ns = stamp_offset_ns                         # stamp_offset_ns != 0
每条消息：
  header.stamp = device_ts_ns + offset_ns             # 运行期 offset_ns 恒定
```
- 图像与 IMU **共用同一偏移**；相邻消息时间差 == 设备时间差（不因换算而改变）。
- **不确定性（必须写入用户文档）**：`offset_ns` 由"首次有效设备时间戳"与"当时的节点时钟"一次性确定，因此 `header.stamp` 与真实曝光/采样时刻之间存在 **≤ 一个帧周期**（默认 30 fps ⇒ ≤33 ms）的对齐不确定性；该不确定性在运行期恒定，不影响图像与 IMU 的相对关系。
- 节点时钟 = `RCL_ROS_TIME`；`use_sim_time` 必须为 `false`（检测到 `true` 时 `WARN` 并按系统时钟继续）。
- 异常降级：某帧 `timestamp_ns==0` → 仍发布，取"最近有效设备时间戳 + `1e9//fps`"，`WARN`（1 Hz 节流）`frame timestamp unavailable`；**首帧**为 0 → 用 `now()` 且不据此确定偏移；IMU 包 `timestamp_ns==0` → 丢弃并 `WARN`；IMU 时间戳回退 → 丢弃并 `WARN`。

### 2.6 launch 契约与复用节点参数（逐字）

`gs130_web.launch.py` 启动 3 个节点（顺序固定）：`gs130_node`、`hobot_codec_republish`、`websocket`。

| 节点 | 参数（逐字） |
| --- | --- |
| `hobot_codec_republish` | `channel=codec_channel(0)`、`in_mode="ros"`、`in_format="nv12"`、`out_mode="ros"`、`out_format="jpeg"`、`sub_topic="/image_combine_raw"`、`pub_topic="/image_combine_jpeg"`、`jpg_quality=jpg_quality(80)`、`input_framerate=-1`、`output_framerate=-1` |
| `websocket` | `image_topic="/image_combine_jpeg"`、`image_type="mjpeg"`、`only_show_image=False`、`output_fps=web_output_fps(30)`、`channel=websocket_channel(1)`、`smart_topic=smart_topic("/image_combine_jpeg")` |

显示语义（冻结书 §6.2 + E7）：`websocket` 的 `channel` 参数在**同一端口 8000** 上区分多路流，**不需要第二个端口**；文档必须给出"channel 号 ↔ 话题"的映射，并在该 channel 上以截图验证画面（`AC-FUN-09`⑤）。冻结书 §6.2 的一句话（`channel 0` = 拼接帧左半 = 左目）与其默认值 `websocket_channel=1` 必须统一（见 §11 `B-02`）。

**E4 备注（关停与孤儿进程）**：`websocket.launch.py` 通过 `os.system` 启动 nginx，**该 nginx 会在 launch 退出后继续存活**。因此验收必须比对"启动前基线"，要求本 launch 不引入任何残留进程与端口占用（`AC-PERF-07`⑤⑥、`AC-RB-15`）；若基线中 nginx 已存在，则关停后允许其继续存在，但占用者集合必须与基线一致。

**禁止**：修改 `hobot_codec` / `websocket` / `hobot_shm` / `hobot_stereonet` 的任何源码或参数默认值；使用 `respawn=True`；自研编解码、web 服务、深度计算。

### 2.7 错误与退出码契约（判定基准）

| 阶段/情形 | 退出码 | 必须出现的日志（逐字要点） |
| --- | --- | --- |
| 参数校验失败（如 `mode:=bogus`、`width:=641`） | `2` | `invalid parameter {name}={value}: {reason}` |
| `platform`/`device` 不支持 | `2` | `unsupported platform/device: {platform} {device}` |
| `gs130_init` `PARAM_ERROR`（含 `raw` 尺寸错、`rect` 无标定） | `1` | `gs130_init failed: PARAM_ERROR (...)`，`raw` 时含 `RAW mode requires width=1088 height=1280`，`rect` 时含 `RECT mode requires EEPROM calibration` |
| `gs130_init` `NOT_FOUND` | `1` | `gs130_init failed: NOT_FOUND: camera not detected on the configured I2C buses` |
| `gs130_init` `UNSUPPORTED`（含 `odr:=100`） | `1` | `gs130_init failed: UNSUPPORTED: ...` |
| `gs130_init` `HW_ERROR`（被占用） | `1` | `gs130_init failed: HW_ERROR` + 占用提示 |
| `gs130_start` 失败 | `1` | `gs130_start failed: {CODE}` |
| 运行期相机通路 `HW_ERROR`/`THREAD_CLOSED` | `1` | `camera stream failed: {CODE}` |
| 运行期 IMU 通路失败 | 不退出 | `imu stream failed: {CODE}; IMU publishing disabled, camera continues`（1 次） |
| 无 IMU | 不退出 | `IMU not present; /imu/data will not be published`（`WARN` 1 次） |
| 无 EEPROM 标定但 `publish_camera_info:=true` | 不退出 | `calibration not available; /image_left/camera_info and /image_right/camera_info will not be published`（`WARN` 1 次） |
| 时间戳异常 | 不退出 | `frame timestamp unavailable` / `IMU timestamp zero: packet dropped`（1 Hz 节流） |
| SIGINT/SIGTERM 正常关闭 | `0` | 关闭序列在 **3 s** 内完成（停 timer → `Device.stop()` → `Device.close()` → `destroy_node()` → `rclpy.shutdown()`） |
| `start()` 长时间未返回（等 FSYNC 握手） | 不退出 | 启动前 `INFO` 说明可能阻塞；每 `start_timeout_s`（默认 10 s）`WARN` 一次；**不中止** `start()`；此阶段收到 SIGINT 打 `shutdown requested while gs130_start is blocking; waiting for the handshake` |

**统计日志（`log_fps_period_s`，默认 5 s，`INFO`）**：逐字包含 `camera {fps:.2f} Hz`、`published {n_combine} combine / {n_eye} per-eye`、`dropped {n_drop}`、`imu {odr:.2f} Hz`、`published {n_imu}`、`offset {offset_ns} ns`。

**`/gs130/status`（T8，`publish_status:=true` 时）**：`name="gs130_ros: gs130_node"`、`hardware_id="GS130 <device> on RDKX5"`、`level ∈ {OK,WARN,ERROR}`、`values` 键固定为 `camera_fps`、`imu_rate`、`dropped_frames`、`stamp_offset_ns`、`mode`、`width`、`height`、`imu_present`。

---

## 3. G0 构建、版本与可复现

| AC | 级 | 判据 | 测量/判定命令 | TC |
| --- | --- | --- | --- | --- |
| `AC-BLD-01` | P0 | `colcon build` 退出码 0，日志无 `Failed`/`error:`（`warning` 允许）；`ros2 pkg prefix gs130_ros` 退出码 0 | `cd $WS && colcon build --packages-select gs130_ros 2>&1 \| tee $EVID/bld.log; echo rc=$?; grep -ciE 'failed\|error:' $EVID/bld.log; ros2 pkg prefix gs130_ros` | TC-02 |
| `AC-BLD-02` | P0 | `ros2 pkg executables gs130_ros` 输出恰为 `gs130_ros: gs130_node` | `ros2 pkg executables gs130_ros` | TC-02 |
| `AC-BLD-03` | P0 | 全新 `git clone` + 按用户文档构建 + 一键启动，冒烟子集（`AC-FUN-01/02/04/05/07/09`）全绿 | 逐字执行用户文档 | §7.3 G3 |
| `AC-BLD-04` | P0 | `git rev-parse HEAD` 记入记录；`git status --porcelain` 为空；`git status --porcelain python/ core/` 为空 | 三条命令 | ★TC-29 |
| `AC-BLD-05` | P0 | 两个 launch 文件均：`--show-args` 退出码 0；`gs130_camera.launch.py` 列出 §2.4 的 18 个 launch 参数；`gs130_web.launch.py` 额外列出 7 个 web 参数；名称与默认值逐字一致 | `ros2 launch gs130_ros gs130_camera.launch.py --show-args \| tee $EVID/args_cam.log`；`... gs130_web.launch.py --show-args` | TC-19 前置 |
| `AC-BLD-06` | P0 | 无硬件主机：`python3 -m compileall -q` 退出码 0；`PYTHONPATH=... python3 -c "import gs130_ros.gs130_node"` 退出码 0（证明顶层无 I/O、无设备构造） | 见命令 | ★TC-30 |
| `AC-BLD-07` | P0 | `package.xml` 的 `<name>`=`gs130_ros`、`<version>`=`0.1.0`、`<license>`=`MIT`（且仓库存在 `LICENSE`）、`<build_type>`=`ament_python`；`setup.py` 的 entry point 为 `gs130_node = gs130_ros.gs130_node:main` | `ros2 pkg xml gs130_ros \| head -20`；`grep -n entry_points -A4 $REPO/ros/gs130_ros/setup.py` | TC-02 |

---

## 4. G1 功能验收

| AC | 级 | 判据 | 测量/判定命令 | TC |
| --- | --- | --- | --- | --- |
| `AC-FUN-01` | P0 | 执行冻结书 §6.6 的 web 启动命令后 ≤20 s：`ros2 node list` 含 `gs130_node` 与两个既有节点；`ros2 topic list` 含 T1/T4/T5/T6（`gs130_web.launch.py` 下还含 T7）；launch 日志 60 s 内无 `process has died`、无 traceback | `ros2 launch gs130_ros gs130_web.launch.py platform:=RDKX5 device:=GS130WI mode:=resize width:=640 height:=480 fps:=30 odr:=200 2>&1 \| tee $EVID/launch.log`；另终端 `sleep 20; ros2 node list; ros2 topic list -t; grep -c "process has died" $EVID/launch.log` | TC-19 |
| `AC-FUN-02` | P0 | 默认参数（`mode=resize 640×480`）下 T1 首帧字段逐一相等：`encoding=nv12`、`width=1280`、**`height=480`**（真实高度，E2）、`step=1280`、`len(data)=921600`、`is_bigendian=0`、`frame_id=camera`；T2/T3：`width=640`、**`height=480`**、`step=640`、`len=460800`、`frame_id=camera_left`/`camera_right`；所有 `header.stamp` 非 0；`len(data) == width*height*3//2` 且 `step == width` | `ros2 topic echo /image_combine_raw --once --field height`（期望 `480`）；`timeout 30 python3 $EVID/frame_fields.py`（附录 A.4） | TC-05（期望值按 §11 `B-04` 修正）、E2 |
| `AC-FUN-03` | P0 | ①T1 由 **SDK 硬件拼接**产生（`stereo_layout=LEFT_RIGHT` 设置于 `Config.preset()` 之后）：`width=2*ew`、`height=eh`、`len=2*ew*eh*3//2`，且与 SDK 拼接帧形状 `(eh*3//2, 2*ew)` = `(720,1280)` 一致；②**左右目位置正确（遮挡法）**：遮挡左目 5 s，T1 解码图**左半幅**平均亮度下降 ≥50%、**右半幅**变化 ≤10%；遮挡右目时相反；③`publish_per_eye:=true` 时 T2/T3 存在且与 T1 **同一帧对**（`Δt ≤ 1 ms`）；若实现选择"切换到 SDK 单目模式"从而 T1 不发布，则文档必须声明该组合行为且 `ros2 topic list` 与声明一致。**本项不要求、也不奖励任何手写拼接/切分代码** | `ros2 topic echo --once /image_combine_raw --field width --field height`；遮挡法脚本（保存 4 张 PNG + 左右半幅 mean 统计，附录 A.10） | ★TC-31（改写）、★TC-46、E6 |
| `AC-FUN-04` | P0 | T2/T3 各解出 `(480,640,3) uint8`（`cv2.COLOR_YUV2BGR_NV12`），`std>5`、`10<mean<245`；20 张 PNG 全部写入成功；抽查 4 张无绿边/上下错位/花屏 | `timeout 60 python3 $EVID/save_png.py` | TC-08 |
| `AC-FUN-05` | P0 | 静止 200 包：①`9.3 ≤ \|accel\| ≤ 10.3 m/s²` ②主轴分量 `\|a_i\| ≥ 9.3`、其余两轴 `≤ 1.0`（轴向与符号以实机实测登记后固化）③`\|gyro\| ≤ 0.05 rad/s` ④`orientation = (0,0,0,1)` ⑤`orientation_covariance[0] == -1.0`、`angular_velocity_covariance[0] == -1.0`、`linear_acceleration_covariance[0] == -1.0`（各 9 元素，其余位 0） | `timeout 30 python3 $EVID/imu_stats.py` | TC-09、`11` A6/A7 |
| `AC-FUN-06` | P0 | 静止时 T4 的 200 包均值 vs SDK 直读（`python3 $REPO/python/test/test_gs130.py RDKX5 GS130WI resize 640 480 30 200` 打印的 `accel`/`gyro`）逐轴：`\|Δaccel\| ≤ 0.5 m/s²`、`\|Δgyro\| ≤ 0.02 rad/s`（两次运行串行；相机独占） | 节点侧脚本落盘 JSON → Ctrl-C → 跑 SDK 冒烟 → 比对 | ★TC-32 |
| `AC-FUN-07` | P0 | T5/T6：①启动后 ≤60 s 各收到一次；②节点启动 10 s 后的**迟到订阅者**仍能收到（latch 生效）；③`width/height == ew/eh`（640/480）；④`K` 与 SDK `camera_intrinsics()` 的 `K` **逐元素相等**（相对误差 ≤1e-9，禁止额外缩放），结构为 `[fx,0,cx,0,fy,cy,0,0,1]`；⑤`distortion_model`/`D` 按 §2.3（`equidistant`+4 项 / `plumb_bob`+5 项 / `rect` 强制 `plumb_bob`+5 个 0）；⑥`R` 全 0（9）、`P` 全 0（12）、`binning=0`、`roi` 全 0 且 `do_rectify=False`；⑦`stamp` 非 0；⑧`frame_id` 与对应 Image 相同。**附加区分性检查**：分别以 `mode:=resize width:=640 height:=480` 与 `mode:=raw width:=1088 height:=1280` 直读 SDK `camera_intrinsics()`；若两者 K **完全相同**，则冻结书 §3.4 的前提（SDK 已按输出分辨率回写）不成立 ⇒ 记阻塞缺陷（`B-03` 同类），CameraInfo 必须按文档化规则换算 | `ros2 topic echo --once /image_left/camera_info`；SDK 侧 `python3 - <<'PY'` + 附录 A.6 比对 | TC-11/TC-12/TC-16（按 §11 `B-04` 修正）、`11` A8/A9 |
| `AC-FUN-08` | P0 | **不发布 TF**（契约决定）：`ros2 topic list \| grep -E '^/tf'` 无输出；用户文档写明"本期不发布 `/tf`、`/tf_static`；需要外参请直接调用 SDK"并给出可复制的 SDK 命令（`relative_R`/`relative_T`/`calibration()`）；文档说明 EEPROM 外参参考系在 `rect` 模式下会变为虚拟平行双目 | `ros2 topic list \| grep -E '^/tf'; grep -n 'tf_static\|relative_T' <用户文档>` | `11` §0.2/§2.3 |
| `AC-FUN-09` | P0 | ①`curl -o /dev/null -w '%{http_code}' http://$BOARD_IP:8000/` == `200` 且返回 HTML（E4 实测）；②`/image_combine_jpeg` 类型为 `sensor_msgs/msg/CompressedImage`，`format` 含 `jpeg`，20 s 窗口内平均速率 **∈ [29,31] Hz（fps=30，窗口 ≥200 样本；E3 实测 29.974 Hz）**；③**解码该 JPEG 得到 `1280 × 480`（`2*ew × eh`）** 且画面为左右两目并排、无绿边/错行/UV 当亮度（E2 的区分性判据）；④`ros2 node info`（websocket 节点）显示订阅 `/image_combine_jpeg`；⑤页面在**文档声明的 channel** 上显示实时画面：间隔 5 s 的两张截图差异比例 ≥5%（存档 `$EVID/web_before.png`、`web_after.png`）；⑥**codec 不崩溃**：60 s 观测期内 `hobot_codec_republish` 进程持续存在、`ros2 node list` 中不消失、launch 日志无 `process has died`、无退出码 `-11`/`139`、无 `init_pic_h_`/`alined_pic_h_` 异常行；⑦**codec 实际收到消息**：`ros2 topic info -v /image_combine_jpeg` 的 Publisher count ≥1 且 `/image_combine_jpeg` 速率达标（E1 的区分性判据：`BEST_EFFORT` 时该话题不会有任何数据） | ①②④⑥⑦ 命令行判定；③ 用 `ros2 topic echo --once /image_combine_jpeg` 落盘后用 `cv2.imdecode` 断言 `shape==(480,1280,3)`（附录 A.7）；⑤ 截图 + 附录 A.7 | TC-20/TC-21、E1/E2/E3/E4 |
| `AC-FUN-10` | P0 | ①`ros2 param get /gs130_node <param>` 对 §2.4 每个参数返回默认值（或 launch 覆盖值）；②运行 ≥15 s 后日志中出现 ≥2 条统计 `INFO`，逐字含 `camera <f> Hz`、`published <n> combine / <n> per-eye`、`dropped <n>`、`imu <r> Hz`、`published <n>`、`offset <ns> ns`；③`ros2 node list` 节点名为 `/gs130_node` | `for p in mode width height fps odr publish_per_eye; do ros2 param get /gs130_node $p; done; grep -c 'camera .* Hz, published' $EVID/launch.log` | ★TC-33、`11` §7.3 |
| `AC-FUN-11` | P0 | `device:=GS130W`：T1 正常发布；`/imu/data` **不出现**；日志恰一条 `IMU not present; /imu/data will not be published`；退出码 0（不因缺 IMU 退出）；`start()` 不进入 FSYNC 等待（首帧时间满足 `AC-PERF-04`） | `ros2 launch gs130_ros gs130_camera.launch.py device:=GS130W ...`；`ros2 topic list`；`grep` 日志 | TC-26、`11` A16 |
| `AC-FUN-12` | P1 | `publish_status:=true` 时：`/gs130/status` 1 Hz；`name="gs130_ros: gs130_node"`；`hardware_id="GS130 GS130WI on RDKX5"`；`values` 键集合恰为 `camera_fps,imu_rate,dropped_frames,stamp_offset_ns,mode,width,height,imu_present` | `ros2 topic echo /gs130/status --once` | `11` §7.6 |

---

## 5. G2 定量验收

> 全部针对**文档默认配置**（`mode=resize`、`640×480`、`fps=30`、`odr=200`）。若实测无法达标，**唯一允许的处置是把默认值改为实测可持续值**（同时改冻结书 + 本表 + `20_test_plan.md`，见 §0.1），**不得放宽阈值**。

| AC | 级 | 判据（阈值） | 测量/判定命令 | TC |
| --- | --- | --- | --- | --- |
| `AC-PERF-01` | P0 | `ros2 topic hz /image_combine_raw`：窗口 ≥**200** 样本、采样 ≥20 s，平均速率 **∈ [29, 31] Hz**（配置 `fps=30`；E3 实测 29.966 Hz）。其它 `fps` 值时判据为 `[fps-1, fps+1] Hz`（`fps ≥ 5`）。**该判据是发布路径性能的端到端陷阱检测项**：逐元素字节赋值（实测 1103.87 ms/帧）必然不达标。附加：`max delta ≤ 0.5 s` | `timeout 75 ros2 topic hz /image_combine_raw --window 200 \| tee $EVID/hz_combine.log` | TC-06（容差按 §11 `B-04` 收紧）、E3 |
| `AC-PERF-02` | P0 | 同一时间窗内：①`/image_combine_raw` 与 `/image_combine_jpeg` 平均速率均为 **∈ [29,31] Hz**（E3 实测 29.966 / 29.974）；②`publish_per_eye:=true` 时 T2/T3 与 T1 平均速率差 ≤ 0.5 Hz；③**codec 实收**：`/image_combine_jpeg` 有数据即证明 codec 收到了 T1（配合 `AC-IF-02` 的 `RELIABLE` 断言构成 E1 的完整判据） | 对四个话题分别执行 `timeout 75 ros2 topic hz <topic> --window 200` | ★TC-34、E1/E3 |
| `AC-PERF-03` | P0 | `/imu/data`：①≥30 s 平均速率 ∈ `[0.95×odr, 1.05×odr]`（200 ⇒ `[190,210]`）；②累计包数 == `odr × 时长`（±1%）；③60 s 内每个 1 s 窗口包数 ∈ `[0.8,1.2]×odr` 的窗口占比 ≥95%。**不得**用最大包间隔判定（契约规定突发） | `timeout 75 ros2 topic hz /imu/data --window 2000`；窗口统计用附录 A.5 的 `imu_windows.py` | TC-10（容差按 §11 `B-04` 统一） |
| `AC-PERF-04` | P0 | ①从发出 web 启动命令到订阅者收到首帧 T1 ≤ **20 s**；②到 `curl :8000` 返回 200 且 T7 首帧 ≤ **30 s**。若 `start()` 因 FSYNC 握手阻塞（日志出现对应 `INFO`/`WARN`），本项改为断言 `AC-RB-06` 的握手行为，并记录实测等待时长 | `T0=$(date +%s.%N); (ros2 launch gs130_ros gs130_web.launch.py >$EVID/launch.log 2>&1 &); timeout 40 ros2 topic echo --once /image_combine_raw --field header.stamp >/dev/null; T1=$(date +%s.%N); echo "$T0 $T1" \| awk '{print $2-$1}'` | ★TC-35 |
| `AC-PERF-05` | P0 | `ros2 topic delay`（同机同钟）：T1 平均 ≤ **150 ms**、最大 ≤ **300 ms**；`/imu/data` 平均 ≤ **50 ms**、最大 ≤ **100 ms**。该判据同时证明 §2.5 的偏移换算使 `header.stamp` 落在**系统时钟域**（若透传 SDK 的 monotonic 纳秒值，延迟会是 `-1.79e18` 量级的负值或秒级错位，E5） | `timeout 70 ros2 topic delay /image_combine_raw \| tail -5`；同命令换 `/imu/data` | ★TC-36、E5 |
| `AC-PERF-06` | P0 | ①100 帧/包：图像 `stamp` 严格递增、IMU `stamp` 非递减、回跳数 = 0；②相邻帧差 ≈ `1/fps`（均值偏差 ≤10%，抖动 ≤1 ms）——对应契约 A10「相邻差 == 设备时间差」；③图像与 IMU **同域**：`abs(imu_stamp − 最近图像 stamp)` 的中位数 ≤ 一个帧周期（33 ms；E5 实测 `imu − frame = 8.6 ms`），且两者差值在整段内抖动 < 5 ms | `timeout 60 python3 $EVID/stamp_check.py` | TC-18、E5 |
| `AC-PERF-07` | P0 | 启动前先记录基线：`pgrep -af "nginx\|hobot_codec\|websocket\|gs130_node"`、`ss -ltnp \| grep :8000`、`ros2 node list`。SIGINT 后：①节点 **≤3 s** 退出且退出码 **0**；②launch 进程树 ≤10 s 全部消失；③`pgrep -af "gs130_node\|hobot_codec\|websocket"` 无输出；④≤10 s 内 SDK 复开成功（打印 `reopen ok`）；⑤**无孤儿进程**：`pgrep -af nginx` 的结果集合与启动前基线**逐行一致**（E4 实测：既有 `websocket.launch.py` 用 `os.system` 启动的 nginx 会在 launch 退出后存活，本包必须处理或明确不引入新增占用）；⑥**端口回基线**：`ss -ltn \| grep :8000` 的占用者集合与启动前一致 | 基线采集命令 + `ros2 launch ... & LP=$!; sleep 30; T0=$(date +%s.%N); kill -INT $LP; wait $LP; echo rc=$?; echo $(date +%s.%N)-$T0 \| bc; pgrep -af "gs130_node\|hobot_codec\|websocket"; pgrep -af nginx; ss -ltn \| grep :8000; timeout 60 python3 -c "import gs130,time; d=gs130.Device(gs130.Config.preset('RDKX5','GS130WI',gs130.CameraMode.RESIZE,640,480,30,200)); d.start(); print('reopen ok'); d.close()"` | TC-22、★TC-48、E4 |
| `AC-RES-01` | P0 | 连续运行 10 min：①节点 RSS ≤ **300 MB**；②第 5→10 min 增长 ≤ **30 MB**；③`/proc/<pid>/fd` 条目增长 ≤ **10**；④`Threads` ≤ **32** | `PID=$(pgrep -f gs130_node \| head -1); grep -E 'VmRSS\|Threads' /proc/$PID/status; ls /proc/$PID/fd \| wc -l`（第 1/5/10 min 各一次） | S-02 |
| `AC-RES-02` | P0 | 60 s 采样平均：①`gs130_node` ≤ **50%** 单核；②整链（`gs130_node`+`hobot_codec_republish`+`websocket`）合计 ≤ **200%** 单核 | `PID=$(pgrep -f gs130_node \| head -1); top -b -d 1 -n 60 -p $PID \| awk '$12 ~ /gs130_node/ {c+=$9; n++} END {printf "%.1f\n", c/n}'`（`top -b` 的 `%CPU` 为第 9 列；命令原文与表头须存档） | ★TC-37 |
| `AC-RES-03` | P1 | ≥60 min soak：所有采样点 T1 速率 ≥ `0.9×fps`，中位数漂移 ≤5%，无连续两点 `< 0.5×fps`；IMU 各点 ≥ `0.8×odr`，无 `> 30 s` 中断；RSS 增长 ≤ 30 MB；无 traceback、无 `process has died`；SoC 温度 ≤ 85 °C | 按 `20_test_plan.md` §5.2 脚本 | S-02 |
| `AC-RES-04` | P0 | 无任何订阅者持续 60 s：T1/T4 速率仍满足 `AC-PERF-01/03`；RSS 增长 ≤ 30 MB（图像 depth=1、IMU depth=200 下不得无限积压） | 关闭全部订阅终端后重复采样 | N-10 |

---

## 6. G3 代码质量验收

| AC | 级 | 判据 | 测量/判定命令 | TC |
| --- | --- | --- | --- | --- |
| `AC-SRC-01` | P0 | 每个 `.py` 模块有模块 docstring；每个公开函数/方法/类（不以 `_` 开头）有 docstring；两个 launch 文件有模块 docstring。脚本输出为空 | `python3 $EVID/check_docstrings.py $REPO/ros/gs130_ros`（附录 A.1） | ★TC-38 |
| `AC-SRC-02` | P0 | ①单 `.py` ≤ **250** 行；②单函数/方法 ≤ **40** 行；③`ros/gs130_ros/gs130_ros/*.py` 合计 ≤ **700** 行；④单 launch ≤ **150** 行 | `wc -l`；函数长度用附录 A.2 | ★TC-38 |
| `AC-SRC-03` | P0 | ①静态检查零告警（未使用 import/局部变量/参数）；②launch 中每个 `DeclareLaunchArgument` 至少被 `LaunchConfiguration` 引用一次 | `python3 -m ruff check --select F,ARG $REPO/ros/gs130_ros`（不可用时附录 A.2）；附录 A.3 | ★TC-38 |
| `AC-SRC-04` | P0 | ①无连续 ≥5 行被注释掉的代码（注释行中 ≥3 行含 `=`/`(`/`def `/`return`）；②无 `if False:`/`if 0:`；③除 `close()` 路径外无 `except: pass`/裸 `except`（该例外须有注释说明理由）；④无孤儿模块（每模块至少被 import 或被 `entry_points` 引用） | 附录 A.8 + 附录 A.2 | ★TC-38 |
| `AC-SRC-05` | P0 | `ros/gs130_ros/` 内不得出现：JPEG/MJPEG 编码（`imencode`、`turbojpeg`、`Mjpeg`）；HTTP/WebSocket 服务（`http.server`、`socketserver`、`socket.socket`、`flask`、`aiohttp`、`tornado`）；深度/视差/点云（`stereonet`、`disparity`、`PointCloud2`）；相机驱动直连（`/dev/video`、`V4L2`、`ioctl`）。命中即 FAIL（文档字符串中的说明性提及逐条豁免并登记） | `grep -rniE 'imencode\|turbojpeg\|http\.server\|socketserver\|socket\.socket\|flask\|aiohttp\|tornado\|stereonet\|disparity\|PointCloud2\|/dev/video\|V4L2\|ioctl' $REPO/ros/gs130_ros --include='*.py'` | 冻结书 §0.2/§2.6 |
| `AC-SRC-06` | P0 | ①`gs130_web.launch.py` 启动既有节点时使用的参数名与值逐字符合 §2.6 表；②仓库内不对既有包做任何补丁（无 `hobot_codec/`、`websocket/`、`hobot_shm/` 目录，无 patch 文件）；③运行时 `ros2 node list` 含官方节点 | `grep -nE "in_format\|out_format\|sub_topic\|pub_topic\|image_topic\|image_type" $REPO/ros/gs130_ros/launch/gs130_web.launch.py`；`find $REPO/ros -name '*.patch'` | TC-19、★TC-38 |
| `AC-SRC-07` | P0 | ①正常运行期**无每帧/每包日志**：启动完成后的 60 s 内日志新增行数 ≤ 统计日志应有条数 + 2（`log_fps_period_s=5` ⇒ ≤14 行）；②统计日志格式逐字符合 §2.7；③`log_fps_period_s:=0` 时无统计日志 | `wc -l $EVID/launch.log`（第 5 s 与第 65 s 各一次）；`grep -c 'Hz, published' $EVID/launch.log` | `11` §7.3 |
| `AC-SRC-08` | P0 | §2.7 表中每一行的日志文本与退出码可被逐字匹配（抽样执行 `AC-RB-01/02/03/04/08/09/12` 后比对）；错误信息必须点名失败调用或参数，不得只有裸 traceback | `grep -nE 'invalid parameter\|gs130_init failed\|camera stream failed\|imu stream failed\|not present\|calibration not available' $EVID/*.log` | TC-03/23/25/26/27 |
| `AC-SRC-09` | P0 | `git status --porcelain python/ core/` 为空；`grep -rnE 'setattr\(gs130\|gs130\.Config\.preset *=\|gs130\.Device\.[A-Za-z_]+ *=' $REPO/ros/gs130_ros` 无输出 | 两条命令 | ★TC-29 |
| `AC-SRC-10` | P0 | 契约非目标未被实现：无 `msg/`、`srv/`、`action/` 目录；无自定义消息依赖（`setup.py`/`package.xml` 无 `message_generation`/`rosidl`）；无 `Temperature`、`PointCloud2`、`ImageMarker` 引用；无 `/tf` 发布（`tf2_ros`、`TransformStamped` 不出现） | `find $REPO/ros/gs130_ros -type d -name 'msg' -o -name 'srv' -o -name 'action'`；`grep -rniE 'Temperature\|PointCloud2\|TransformStamped\|tf2_ros' $REPO/ros/gs130_ros` | 冻结书 §0.2/§9 |
| `AC-SRC-11` | P0 | ①`ros2 param list /gs130_node` 的参数集合 == §2.4 的 **24** 个节点参数（多出即 FAIL，对应冻结书 §5.5 的非目标参数；launch-only 的 6 个参数 `codec_channel`/`websocket_channel`/`web_port`/`web_output_fps`/`jpg_quality`/`smart_topic` 也不得出现，见 §11 `B-08`）；②launch 文件不含 `respawn=True`；③launch 与节点不设置 `use_sim_time:=true` | `ros2 param list /gs130_node`；`grep -n respawn $REPO/ros/gs130_ros/launch/*.py` | ★TC-39 |
| `AC-SRC-12` | P1 | 纯函数层（消息构造、时间换算、CameraInfo 映射、拼接几何）在无硬件主机上单测通过 | `python3 -m pytest $REPO/ros/gs130_ros/test -q`（若提供该目录） | `10` §11.1 |
| `AC-SRC-13` | P1 | `data` 赋值必须是整块缓冲形式（`array.array("B")` + `frombytes`、`bytes(...)`、`tobytes()`）；**不得**出现逐元素赋值（`msg.data = list(...)`、`bytearray(...)` 逐项 append、`for i: msg.data[i] = ...`）。E3 实测：逐元素赋值 1103.87 ms/帧 vs 整块 1.36 ms/帧；本项是 `AC-PERF-01` 的静态佐证 | `grep -rnE 'msg\.data *= *(list\|bytearray)\(\|\.data\[[a-z_]+\] *=' $REPO/ros/gs130_ros --include='*.py'`（期望无输出）；`grep -rnE 'array\(.B.\)\|frombytes\|tobytes\(\)\|bytes\(' $REPO/ros/gs130_ros --include='*.py'`（期望至少一处） | ★TC-38、E3 |

---

## 7. G4 接口契约验收

| AC | 级 | 判据 | 测量/判定命令 | TC |
| --- | --- | --- | --- | --- |
| `AC-IF-01` | P0 | **阻塞**：§11 的 `B-01`…`B-11` 全部关闭；`01/10/12/20` 与冻结书在包名、节点名、话题名、类型、`frame_id`、参数名/默认值、launch 名上**逐字一致**（不接受"等价但不同"） | 人工逐项比对（清单存档）；任一不一致 ⇒ FAIL | — |
| `AC-IF-02` | P0 | ①`ros2 topic list -t` 中 T1/T4/T5/T6（及 `gs130_web.launch.py` 下的 T7）名称与类型逐字一致；②`ros2 topic info -v` 的四项与 §2.2 一致，其中**图像三话题的 Reliability 必须是 `RELIABLE`**（E1；若为 `BEST_EFFORT` ⇒ FAIL，因为 codec 收不到消息）；③`frame_id` 默认值逐字一致；④默认不出现 T2/T3/T8；⑤E1 的因果验证：`RELIABLE` 下 `/image_combine_jpeg` 有数据；若把发布者改成 `BEST_EFFORT` 复测，`/image_combine_jpeg` 必须无数据（证明该判据真的在测 QoS 兼容性，可选但建议留证） | `ros2 topic list -t`；`ros2 topic info -v /image_combine_raw /imu/data /image_left/camera_info` | TC-04（按 §11 `B-04` 修正）、★TC-45、E1 |
| `AC-IF-03` | P0 | ①`--show-args` 的名称集合与默认值 == §2.4（见 `AC-BLD-05`）；②以非默认值启动后 `ros2 param get` 返回该值；③launch 到节点的类型转换正确（`width:=320` 得到 int 320，不是字符串） | `ros2 launch gs130_ros gs130_camera.launch.py width:=320 height:=240; ros2 param get /gs130_node width` | ★TC-40 |
| `AC-IF-04` | P0 | 每个参数的可观测效应（**拼接几何、速率、web 判据一律在 `mode=resize` 或 `rect` 下执行，E7**）：`width:=320 height:=240` ⇒ T1 `640×240`、`len=230400`，T2/T3 `320×240`、`len=115200`；`fps:=15` ⇒ T1 平均速率 ∈ `[14,16] Hz`；`odr:=500` ⇒ 平均 ≈500（`[475,525]`）；`publish_combine:=false` ⇒ 无 T1；`publish_per_eye:=true` ⇒ 出现 T2/T3；`publish_imu:=false` ⇒ 无 T4；`publish_camera_info:=false` ⇒ 无 T5/T6；`camera_info_distortion_model:=none` ⇒ `D` 为空；`publish_status:=true` ⇒ 出现 T8；`log_fps_period_s:=0` ⇒ 无统计日志；`mode:=raw width:=1088 height:=1280` ⇒ T2/T3 `1088×1280`、`len=2088960`（`raw` 下**不得**用其它尺寸，见 `AC-RB-04`），且 T1 的行为（若可用则 `2176×1280`、`len=4177920`）必须与文档声明一致（若 SDK 在 `raw` 下无法拼接，文档必须声明该组合的处理方式与退出码，并作为 §13 `K-14` 登记） | 每项一次独立启动，判定同 §4/§5 | TC-10/15/16/17/27 + 参数矩阵、E7 |
| `AC-IF-05` | P0 | §2.3 的全部冻结字段（Image/Imu/CameraInfo）逐项满足；其中 **`Image.height` 必须等于真实高度 `eh`**（E2；不得为 `eh*3//2`）、`orientation_covariance[0]`、`angular_velocity_covariance[0]`、`linear_acceleration_covariance[0]` 必须为 `-1.0`（不得为 0）、CameraInfo 的 `R`/`P`/`binning`/`roi` 必须全 0 | `ros2 topic echo`/脚本逐字段断言（附录 A.4/A.6） | `11` A2–A9 + E2 |
| `AC-IF-06` | P0 | §2.5 的时间戳规则（E5）：①`stamp_offset_ns:=0` 时 `offset_ns` 在运行期恒定（统计日志中 `offset` 值 60 s 内不变）；②**`header.stamp` 落在系统时钟域**：默认配置下 `abs(stamp.sec − $(date +%s)) ≤ 60`（该帧到达时刻与消息时间戳之差在分钟级内）；③**未透传设备时间戳**：以 `stamp_offset_mode:=device` 启动时 `abs(stamp.sec − $(date +%s)) > 1e6`（≈1970 年，E5 的 monotonic 量级），证明两种模式可区分且默认模式确实做了换算；④`stamp_offset_ns:=<已知值>` 时 `header.stamp == device_ts + 该值`（用 SDK 直读的 `timestamp_ns` 比对，误差 0 ns）；⑤`use_sim_time` 检查见 `AC-RB-13` | 统计日志 `offset` 字段采样；`ros2 topic echo --once /image_combine_raw --field header.stamp` 与 `date +%s` 比对；SDK 直读比对（附录 A.6） | ★TC-47、`11` §4、E5 |
| `AC-IF-07` | P0 | `gs130_web.launch.py` 启动的节点集合恰为 3 个（`gs130_node` + `hobot_codec_republish` + `websocket`），既有节点参数逐字符合 §2.6；`gs130_camera.launch.py` **只**启动 `gs130_node`（`ros2 node list` 中无 codec/websocket） | `ros2 launch ... gs130_camera.launch.py` 后 `ros2 node list` | TC-19 |

---

## 8. G5 鲁棒性验收

> 每个用例结束必须执行 §0.2 的清理自检。注入类用例（假设备/桩）必须在仓库外或测试目录内实现，不得为测试在生产代码中加分支。

| AC | 级 | 触发方式 | 必须观测到 | TC |
| --- | --- | --- | --- | --- |
| `AC-RB-01` | P0 | 实例 A（`gs130_node`）运行中再启动实例 B | B：`ERROR` 含 `gs130_init failed:` 与 `HW_ERROR`（或 `NOT_FOUND`）+ 占用提示，退出码 **1**，≤10 s 退出；A：速率仍满足 `AC-PERF-01`，B 退出前后 10 s 内 A 无 `> 0.5 s` 帧间隔，A 不在 B 之后退出 | TC-23（注入方式按 §11 `B-04` 改为本包第二实例）、`11` A12 |
| `AC-RB-02` | P0 | `GS130_LIB=/nonexistent/libgs130.so ros2 run gs130_ros gs130_node` | ≤10 s 退出；退出码 ≠0 且非段错误（≠139/`-11`）；stderr 含 `libgs130`（或 `cannot open shared object file`）并提及 `GS130_LIB`；无 core dump；`ros2 node list` 无残留 | TC-25 / N-01 |
| `AC-RB-03` | P0 | `-p mode:=bogus`；`-p width:=641`；`-p width:=0`；`-p fps:=0`；`-p fps:=99`；`-p odr:=100`；`-p platform:=X`；`-p device:=GS130X` | 前六类：退出码 **2**（参数校验阶段）且日志含 `invalid parameter {name}={value}` 与原因；`odr:=100` 与非法 `platform/device`：退出码必须等于**文档（冻结书 §7.1）声明的值**（`UNSUPPORTED` ⇒ 1；`platform/device` ⇒ 2），日志文本逐字匹配。判定只比较「实测 == 文档声明」，不接受任何未声明的取值；期间不得发布任何图像话题 | TC-03 / TC-27、`11` A15、E7 |
| `AC-RB-04` | P0 | `-p mode:=raw -p width:=640 -p height:=480`（E7：`gs130_init` 必须拒绝） | 进程退出且**退出码等于文档声明的值**（冻结书 §7.1 的 `PARAM_ERROR` ⇒ 1；若实现在参数校验阶段拦截，则必须在文档中声明为 2），日志逐字含 `RAW mode requires width=1088 height=1280`；不进入取流、不发布任何图像话题；随后 `mode:=raw width:=1088 height:=1280` 必须能正常启动（见 `AC-IF-04`） | `11` A14、E7 |
| `AC-RB-05` | P0 | 20 轮：启动 → 12 s → `ros2 topic hz` → SIGINT → 等待 → 下一轮 | 20/20 轮均在 12 s 内出现有效 `average rate`；每轮退出码 0 且 ≤3 s；每轮后无残留进程；第 20 轮耗时 ≤ 第 1 轮 ×1.5；节点 RSS 增长 ≤20 MB；每 5 轮复开自检 `reopen ok` | S-01 |
| `AC-RB-06` | P0 | ①启动后 2 s 内发 SIGINT（首帧之前）；②模拟 `start()` 长阻塞（`start_timeout_s:=2` 且 IMU 不产 FSYNC，或代码级桩） | ①若未阻塞：≤3 s 退出码 0、无残留、相机 ≤10 s 可复开、紧接着重新启动仍满足 `AC-FUN-01`；②若阻塞：出现启动前 `INFO`、每 `start_timeout_s` 一条 `WARN`、SIGINT 时一条 `shutdown requested while gs130_start is blocking; waiting for the handshake`，`start()` 返回后 ≤3 s 退出码 0（节点**不得**强行中止 `start()`） | ★TC-41 |
| `AC-RB-07` | P1 | `pkill -9 -f gs130_node` 后立即重新启动 | 60 s 内重新启动成功；否则**文档化的恢复步骤**（README 必须给出，如等待/重启命令）在 60 s 内使其成功 | ★TC-42 |
| `AC-RB-08` | P0 | 代码级注入 `read_image()` 抛 `HW_ERROR` / `THREAD_CLOSED`；再注入 `read_imu()` 抛同类 | 相机通路：`ERROR` 含 `camera stream failed: {CODE}`，≤5 s 退出码 **1**，无残留、相机可复开。IMU 通路：`ERROR` 一次含 `imu stream failed: {CODE}; IMU publishing disabled, camera continues`，**不退出**，T1 继续发布且速率达标，T4 停止发布 | ★TC-43、`11` §7.1/§7.2 |
| `AC-RB-09` | P0 | ①`mode:=rect` 且无标定（无 EEPROM 样机或代码级桩）②`mode:=resize` 且无标定 | ①退出码 **1**，日志逐字含 `RECT mode requires EEPROM calibration`；②不退出，`WARN` 一次含 `calibration not available;`，T5/T6 不发布（不得伪造内参），T1 正常发布 | N-06、`11` §7.1 |
| `AC-RB-10` | P0 | 只启动节点、无任何订阅者 60 s | 同 `AC-RES-04` | N-10 |
| `AC-RB-11` | P0 | `device:=GS130W` | 同 `AC-FUN-11`（图像正常、无 `/imu/data`、WARN 一次、退出码 0 语义） | `11` A16 |
| `AC-RB-12` | P0 | 代码级注入：图像 `timestamp_ns==0`；IMU `timestamp_ns==0`；IMU 时间戳回退 | 图像：帧仍发布，`stamp` 单调递增（用上一有效值 + `1e9//fps`），`WARN` 含 `frame timestamp unavailable`（1 Hz 节流）；IMU：包被丢弃（T4 计数不含该包），`WARN` 含 `IMU timestamp zero: packet dropped`；回退包被丢弃并 `WARN`；节点不退出 | `11` §4.3 |
| `AC-RB-13` | P0 | `--ros-args -p use_sim_time:=true` 启动 | 出现 `WARN`（说明偏移量语义变化），节点**仍发布** T1/T4，退出码 0；`ros2 param get /gs130_node use_sim_time` 为 true | `11` A11 |
| `AC-RB-14` | P0 | **图像字段错误的失败模式必须被显式检测**：①在验收中（人为把发布的 `height` 改成 `1.5*H` 的对照实验，或直接引用 E2 的实测记录）确认 `hobot_codec` 会打印 `init_pic_h_`/`alined_pic_h_` 并**段错误（退出码 `-11`）**；该情形记 **Blocker** 级缺陷（不是"链路不通"）；②被测实现在整个验收过程中 `hobot_codec_republish` 的退出码**不得**出现 `-11`/`139`，`dmesg -T` 无新的 segfault 记录 | 对照实验：任一发布者以 `height=720` 发布 1280×480 的 NV12（例如用 `ros/probe_nv12_publisher.py` 改一处），观察 codec 退出码与日志；被测实现侧：`ros2 node list` 持续存在 + `dmesg -T \| tail -30` | ★TC-45、E2 |
| `AC-RB-15` | P0 | **关停后无残留进程与端口占用**（与 `AC-PERF-07`⑤⑥ 同一证据）：启动前记录 `pgrep -af "nginx\|hobot_codec\|websocket"` 与 `ss -ltnp \| grep :8000`；SIGINT 后 10 s 内该两组输出必须与基线**逐行一致**（不得新增）；连续 3 轮启停后仍满足，且 `/image_combine_jpeg` 不再有发布者 | 基线采集 + 3 轮循环脚本（每轮记录两组命令输出） | ★TC-48、E4 |

---

## 9. G6 文档验收

用户文档位置：`ros/README.md` 或 `ros/docs/40_user_guide.md`（README 顶部须给出明确路径）。

| AC | 级 | 判据 | 判定方法 | TC |
| --- | --- | --- | --- | --- |
| `AC-DOC-01` | P0 | 文档含 9 个章节：①环境与安装（`source /opt/tros/humble/setup.bash`、`gs130` Python 包安装、构建）②冻结书 §6.6 的逐字启动命令 ③参数表（名称/默认/范围/`Reconfig=No` 说明）④话题表（名称/类型/QoS 四项/`frame_id`/含义/必需或可选）⑤web UI 使用（URL、channel 号与话题的对应）⑥故障排查 ≥9 条 ⑦已知限制 ⑧v0.1.0 非目标 ⑨与 `mipi_cam` 的独占互斥警告 | 勾选清单存档 | ★TC-44 |
| `AC-DOC-02` | P0 | 由未参与编码的测试者从干净环境按文档操作，**30 min 内**在网页看到画面；向作者提问次数 = **0** | 计时记录 + 提问清单（须为空） | ★TC-44 |
| `AC-DOC-03` | P0 | 随机抽取 ≥10 条文档命令逐字执行，与文档描述一致，通过 10/10 | 逐条 `rc=$?` 存档 | ★TC-44 |
| `AC-DOC-04` | P0 | 文档的参数表/话题表与冻结书 §2/§5/§6 逐字一致（名称、默认值、类型、QoS、`frame_id`、必需性） | `diff` 人工比对 | `AC-IF-01` |
| `AC-DOC-05` | P0 | 故障排查覆盖：①网页黑屏/无图 ②图像花屏、绿边、上下错位 ③`HW_ERROR`/相机被占（含"不要用 `respawn`"） ④`libgs130 not found` ⑤IMU 无数据 ⑥启动参数被拒（`invalid parameter`） ⑦`raw` 模式尺寸错 ⑧`Ctrl-C` 后设备未释放、二次启动失败 ⑨`:8000` 打不开。每条为"现象 → 判定命令（可复制）→ 处理动作" | 人工核对 | ★TC-44 |
| `AC-DOC-06` | P0 | 必写内容全部出现：①独占警告（冻结书 §8.3 的文本要点）②显示语义（哪个 channel 显示哪路画面；同一端口 8000 靠 channel 区分，无需第二个端口）③**时间戳语义**：SDK 时间戳为开机以来的 `CLOCK_MONOTONIC`、`header.stamp` 经一次性常量偏移换算到系统时钟域、**且必须写明 `header.stamp` 与真实曝光/采样时刻之间存在 ≤ 一个帧周期（默认 ≤33 ms）的不确定性**（E5）④丢弃项（IMU 温度、`is_fsync` 不发布；`D` 的截断规则）⑤外参不发布及替代做法 ⑥IMU 加速度含重力、单位 ⑦运行期参数修改无效 ⑧非目标清单（含"NV12 不能直接用于 `rqt_image_view`"，需经 codec）⑨图像 `height` 为真实高度、`len(data)=width*height*3//2` 的说明（便于用户自行校验） | 人工核对（逐条打勾，缺失即 FAIL） | `11` §8.3、§4.4、§3.3、§3.4 + E2/E5/E7 |
| `AC-DOC-07` | P1 | 中文说明 + 英文标识符/话题名/命令；所有 ROS 命令前置统一为 `source /opt/tros/humble/setup.bash` + `source $WS/install/setup.bash` | 抽样审阅 | — |

---

## 10. 明确非验收范围（NON-ACCEPTANCE：v0.1.0 不要求，任何人不得据此阻塞发布）

| # | 不做 / 不要求 | 说明 |
| --- | --- | --- |
| NA-01 | 深度图、视差、点云、`hobot_stereonet` 接入 | 冻结书 §0.2 明确非目标 |
| NA-02 | 由本包计算深度/视差的任何实现 | 同 NA-01 |
| NA-03 | `/tf`、`/tf_static`、外参发布、URDF | 冻结书 §0.2 明确不发布；只需文档说明替代做法（`AC-FUN-08`） |
| NA-04 | IMU 温度话题与 `is_fsync` 发布 | 冻结书 §3.3 明确丢弃；只需文档说明 |
| NA-05 | IMU 姿态解算（AHRS/互补滤波/卡尔曼） | `orientation` 固定 `(0,0,0,1)` + `covariance[0]=-1` |
| NA-06 | ROS 1 / `ros1_bridge` | 目标平台只有 TROS humble |
| NA-07 | 运行期动态重配置（`ros2 param set` 生效、`on_set_parameters`） | `Reconfig=No`；文档写明即可 |
| NA-08 | 多相机 / 多实例 / 多命名空间；与 `mipi_cam` 协同运行 | 硬件独占；只要求第二个实例明确报错退出 |
| NA-09 | 自定义 `msg`/`srv`/`action`、interface 包 | 只用标准消息 |
| NA-10 | lifecycle node、component 容器、自定义 executor | 普通 `rclpy.node.Node` |
| NA-11 | C++/`rclcpp` 节点、`ament_cmake` 混合包、自研 V4L2/ISP/VSE 驱动 | SDK 是唯一驱动路径 |
| NA-12 | 自研 web 前端 / HTTP / MJPEG / WebSocket；修改官方前端与既有节点 | 必须复用 `websocket` + nginx `:8000` |
| NA-13 | 自研 JPEG 编解码、NV12→BGR/RGB 发布、`image_transport` 插件 | 由 `hobot_codec_republish` 承担 |
| NA-14 | 零拷贝/hbmem 发布路径优化、帧池、内存复用 | 本期明确接受一次 `memcpy` |
| NA-15 | 标定写入 / `convert_calibration()` 回写 / 棋盘格标定流程 / EEPROM 写 | 只读、只展示 |
| NA-16 | 标定精度、外参精度、VIO/SLAM 精度评估 | 只要求"与 SDK 返回值一致" |
| NA-17 | `ros2 bag` 封装、录制/回放节点、视频文件输出 | bag 仅作测试工具（可用 `ros2 bag record` 抽检） |
| NA-18 | `rqt_image_view` 直接显示 NV12 | 生态不支持；文档说明需经 codec → JPEG |
| NA-19 | 比 §5 更严的性能指标（60 fps、零丢帧、RAW@30fps 确定性） | 只验收 §5 阈值 |
| NA-20 | 非 RDK X5 平台、非 GS130W/WI 传感器、非 TROS 发行版 | `platform` 仅接受 `RDKX5` |
| NA-21 | 打包发布（`.deb`、apt 源、`rosdep` key、CI/CD） | 手工构建 + 文档安装 |
| NA-22 | 英文文档、教程/视频/营销材料 | 只需中文用户文档 |
| NA-23 | `python/`、`core/` 的任何修改 | SDK 冻结 |
| NA-24 | 主观判定（画质、美观、"感觉流畅"） | 一律不进门禁 |

---

## 11. 阻塞性澄清与文档修正登记（`AC-IF-01` 的关闭清单）

> 全部为**发布阻塞项**。关闭方式：修改 `11_interface_freeze.md`（走接口评审并记录变更）+ 本表 + `20_test_plan.md`，或在板上取得实测证据后维持原契约并在本节登记证据路径。

| ID | 问题 | 现状 | 必须的处置 |
| --- | --- | --- | --- |
| `B-01` | **~~拼接布局的代码示例与文字意图不自洽~~ → 已由 E6 关闭** | 冻结书 §3.1.3 给出的 `data = concatenate([L.reshape(-1), R.reshape(-1)])` 手写拼接与其"左目在左、右目在右"的文字意图不自洽 | **关闭**：E6 实测表明 `Config.preset()` 之后设置 `stereo_layout=StereoLayout.LEFT_RIGHT` 即可获得硬件拼接帧（640×480 ⇒ 形状 `(720,1280)`），正合 `width=2W, height=H` 约定。冻结书必须删除该手写拼接示例，改为"使用 SDK 硬件拼接"；验收不要求也不奖励手写拼接（`AC-FUN-03`） |
| `B-02` | **显示 channel 号不一致 → 按 E7 收敛** | 冻结书 §6.2 的显示语义说 `channel:=0` 显示拼接帧左半（左目），但 §6.2/§6.4 的 launch 默认是 `websocket_channel=1` | 同一端口 8000 用 `channel` 区分多路流，**不需要第二个端口**（E7）。必须把"channel 号 ↔ 话题"写成用户文档中的一张映射表，并使冻结书 §6.2 的文字与 `websocket_channel` 默认值一致；验收以 `AC-FUN-09`⑤（截图 + `ros2 param get` 的 channel 值）判定 |
| `B-03` | **~~NV12 `height` 语义冲突~~ → 已由 E2 关闭（结论：`height` = 真实高度 H）** | 冻结书 §3.1.2 冻结 `height = eh*3//2`；实测（E2）填 `1.5H` 会使 codec 打印 `init_pic_h_: 1080, alined_pic_h_: 1088` 并**段错误（退出码 `-11`）** | **关闭**：`height` 必须是真实高度 `H`，`step = width`，`len(data) = width*height*3//2`。冻结书 §3.1/§3.1.2 与附录 A3 必须修正；`20_test_plan.md` 的 TC-05/TC-14/TC-16/TC-27 期望值同步修正（见 `B-04`）；`AC-FUN-02`/`AC-IF-05`/`AC-RB-14` 为判定项 |
| `B-04` | **`20_test_plan.md` 与冻结书/实测不一致** | 话题集合（TC-04 期望的 6 话题与名称）、launch 名（TC-19/24/S-01/S-02 用 `gs130_web.launch.py`）、可执行名（`gs130_node`）、`frame_id`（`camera`/`camera_left`/`camera_right`/`imu_link`）、**NV12 `height` 期望值（TC-05/14/16/27 必须改为真实高度 `480`，E2）**、**速率容差（TC-06/TC-17/TC-20 收紧为 `[29,31] Hz`@30fps 或 `[fps-1,fps+1]`，E3）**、**QoS 期望（图像 `RELIABLE`，E1）**、per-eye 默认开关、期望各用例的**退出码**、**新增 `-11`/段错误 断言（E2）**、**新增"关停后无残留进程/端口回基线"（E4）**、**新增 channel 映射与时钟域断言（E5/E7）**、TC-13（TF 用例应改为"确认无 TF + 文档说明"）、TC-11 的固定内参期望值（应改为与 SDK 逐元素比对） | 按冻结书修正版 + E1–E7 逐条修正测试计划；新增本文件 §12 列出的 ★TC |
| `B-05` | **`10_architecture.md` / `12_implementation_plan.md` 与冻结书冲突** | 话题名（`/gs130/image_nv12_*`、`/gs130/left/image_raw`、`/gs130/right/image_raw`、`/gs130/imu`）、launch 名、TF 发布、温度话题、节点名、默认参数 | 两份文档必须以冻结书为准改写（或明确标注为历史提案、不具约束力），避免实现依据错误文档 |
| `B-06` | **`01_requirements_review.md` 的验收条款与冻结书冲突** | AC-01 默认 `raw 1088×1280 / odr=100`；AC-04 `/gs130/imu`；AC-05 `/gs130/left/camera_info`；AC-06 要求 TF 可用；AC-11 `gs130_probe` | 以冻结书为准更新（默认 `resize 640×480 / odr=200`、`/imu/data`、`/image_left/camera_info`、无 TF、无 `gs130_probe`），否则验收标准自相矛盾 |
| `B-07` | **`11` 未定义"页面 channel ↔ 话题"的可验证判据** | §6.2 只有一句话，且与 `websocket_channel` 默认值冲突（见 `B-02`） | 用户文档必须给出映射表，且 `AC-FUN-09` ⑤ 以截图判定 |
| `B-08` | **`codec_channel`/`websocket_channel`/`web_port`/`web_output_fps`/`jpg_quality`/`smart_topic` 的参数归属歧义** | `11` §5.1 把它们列入"节点参数表"但注明"仅 launch 使用"，§6.4 又称其为 launch 参数 | 明确为 **launch-only**：不得出现在 `ros2 param list /gs130_node`（影响 `AC-SRC-11` ①的参数集合断言） |
| `B-09` | **图像 QoS 必须为 `RELIABLE`（E1 实测）** | `11` §2.2/§9 冻结图像三话题为 `BEST_EFFORT`；实测 `BEST_EFFORT` 时 codec 打印 `offering incompatible QoS ... RELIABILITY_QOS_POLICY` 且收到 0 条消息 | 冻结书 §2.2/§9 必须把 T1/T2/T3 的 reliability 改为 `RELIABLE`；`20_test_plan.md` 若有 QoS 期望同步修正；验收以 `AC-IF-02`②（`topic info -v` = `RELIABLE`）+ `AC-PERF-02`③（codec 实收）双重判定 |
| `B-10` | **`publish_per_eye` 与硬件拼接互斥（E6/E2 推论）** | SDK 的选择是二选一：`stereo_layout=LEFT_RIGHT` 时 `read_image()` 只给 `{"stitched": ...}`（`gs130_get_nv12_frame` 返回 `UNSUPPORTED`）；`stereo_layout=NONE` 时只给 `{"left","right"}` | 冻结书必须明确 `publish_per_eye:=true` 的实现方式：要么同时开启 `stereo_layout` 切换（此时 T1 不存在，文档必须声明），要么由实现从拼接帧分离（此时**不奖励**该做法，且必须满足 `AC-FUN-03`③ 与 `AC-PERF-01`）。禁止"两次读设备凑对"（`§2.3` 发布条件） |
| `B-11` | **关停后的 nginx 孤儿（E4 实测）** | 既有 `websocket.launch.py` 通过 `os.system` 启动 nginx，launch 退出后 nginx 存活；冻结书 §7.4 的关闭契约只约束 `gs130_node` | 必须在冻结书中补充"进程树与端口"要求：本 launch 不得引入新增残留进程与端口占用（允许与启动前基线一致）；验收判定 `AC-PERF-07`⑤⑥ 与 `AC-RB-15`。若清理手段需要改动既有节点（§8.4 禁止），则必须在用户文档中给出显式清理步骤并在 §13 登记 |

---

## 12. 追溯表（AC ↔ TC）

**TC 编号来源**：`20_test_plan.md` 的 `TC-01`…`TC-27`、`N-01`…`N-10`、`S-01`/`S-02`。带 ★ 的为本文件新增、**必须由测试计划补入**的用例。

| AC | TC | 判据位置 |
| --- | --- | --- |
| `AC-BLD-01`/`AC-BLD-02`/`AC-BLD-07` | TC-02 | §3 |
| `AC-BLD-03` | `20_test_plan.md` §7.3 G3 | §3 |
| `AC-BLD-04`、`AC-SRC-09` | ★TC-29 | §3、§6 |
| `AC-BLD-06` | ★TC-30 | §3 |
| `AC-BLD-05` | TC-19 前置、★TC-40 | §3、§7 |
| `AC-FUN-01` | TC-19 | §4 |
| `AC-FUN-02` | TC-05（按 `B-04` 修正） | §4 |
| `AC-FUN-03` | ★TC-31（改写为遮挡法）、★TC-46 | §4 |
| `AC-FUN-04` | TC-08 | §4 |
| `AC-FUN-05` | TC-09 | §4 |
| `AC-FUN-06` | ★TC-32 | §4 |
| `AC-FUN-07` | TC-11/TC-12/TC-16（按 `B-04` 改为与 SDK 逐元素比对） | §4 |
| `AC-FUN-08` | TC-13（改为"确认无 TF + 文档说明"） | §4 |
| `AC-FUN-09` | TC-20/TC-21、★TC-45 | §4 |
| `AC-FUN-10` | ★TC-33 | §4 |
| `AC-FUN-11`、`AC-RB-11` | TC-26 | §4、§8 |
| `AC-FUN-12` | `11` §7.6 | §4 |
| `AC-PERF-01` | TC-06 | §5 |
| `AC-PERF-02` | ★TC-34 | §5 |
| `AC-PERF-03` | TC-10（容差按 `B-04` 统一） | §5 |
| `AC-PERF-04` | ★TC-35 | §5 |
| `AC-PERF-05` | ★TC-36 | §5 |
| `AC-PERF-06` | TC-18 | §5 |
| `AC-PERF-07` | TC-22、★TC-48 | §5 |
| `AC-RES-01`、`AC-RES-03` | S-02 | §5 |
| `AC-RES-02` | ★TC-37 | §5 |
| `AC-RES-04`、`AC-RB-10` | N-10 | §5、§8 |
| `AC-SRC-01`…`AC-SRC-04`、`AC-SRC-06` | ★TC-38 | §6 |
| `AC-SRC-05`、`AC-SRC-10` | 冻结书 §0.2/§9 | §6 |
| `AC-SRC-07` | `11` §7.3 | §6 |
| `AC-SRC-08` | TC-03/23/25/26/27 | §6 |
| `AC-SRC-11` | ★TC-39 | §6 |
| `AC-SRC-12` | `10` §11.1 | §6 |
| `AC-SRC-13` | ★TC-38 | §6、E3 |
| `AC-IF-01` | 无（文档评审项） | §7、§11 |
| `AC-IF-02` | TC-04（按 `B-04` 修正）、★TC-45 | §7、E1 |
| `AC-IF-03` | ★TC-40 | §7 |
| `AC-IF-04` | TC-10/15/16/17/27 | §7 |
| `AC-IF-05` | `11` A2–A9（并入 TC-05/09/11/12） | §7 |
| `AC-IF-06` | ★TC-47 + TC-18 | §7、E5 |
| `AC-IF-07` | TC-19 | §7 |
| `AC-RB-01` | TC-23（注入方式改本包第二实例）、N-08、`11` A12 | §8 |
| `AC-RB-02` | TC-25、N-01 | §8 |
| `AC-RB-03` | TC-03、`11` A15 | §8 |
| `AC-RB-04` | TC-27、`11` A14 | §8 |
| `AC-RB-05` | S-01 | §8 |
| `AC-RB-06` | ★TC-41 | §8 |
| `AC-RB-07` | ★TC-42 | §8 |
| `AC-RB-08` | ★TC-43 | §8 |
| `AC-RB-09` | N-06、TC-15 | §8 |
| `AC-RB-12` | `11` §4.3（并入 ★TC-43 的注入框架） | §8 |
| `AC-RB-13` | `11` A11 | §8 |
| `AC-RB-14` | ★TC-45 | §8、E2 |
| `AC-RB-15` | ★TC-48 | §8、E4 |
| `AC-DOC-01`…`AC-DOC-06` | ★TC-44 | §9 |
| `AC-DOC-07` | 无（审阅项，P1） | §9 |

**必须新增的 TC（测试计划待补）**：TC-29（发布候选与工作区基线）、TC-30（主机侧语法/导入）、TC-31（**硬件拼接几何 + 遮挡法左右目位置**）、TC-32（IMU 与 SDK 直读一致）、TC-33（参数默认值 + 统计日志 + offset 恒定）、TC-34（**T1/T7 速率 `[29,31] Hz`** + T2/T3 一致性）、TC-35（启动到首帧计时）、TC-36（端到端延迟）、TC-37（CPU）、TC-38（静态质量检查：docstring/体积/死代码/复用/`data` 赋值形式）、TC-39（参数集合与非目标参数）、TC-40（参数传递与类型转换生效）、TC-41（启动中断与 FSYNC 阻塞告警）、TC-42（SIGKILL 恢复）、TC-43（故障/时间戳注入：相机通路致命、IMU 通路非致命、时间戳降级）、TC-44（文档可用性）、**TC-45（E1/E2 共同判据：图像 `RELIABLE` + codec 实收 + codec 不崩溃 + 解码尺寸 1280×480 + `-11` 为硬失败）**、**TC-46（遮挡法：T1 左半幅=左目、右半幅=右目）**、**TC-47（E5 时钟域：默认模式下 `abs(stamp.sec − date +%s) ≤ 60`、`stamp_offset_mode:=device` 时 > 1e6、offset 恒定）**、**TC-48（E4 关停：无孤儿进程、端口 8000 占用回基线，连续 3 轮）**。

---

## 13. 已知风险与本次验收无法闭合的项（必须知情）

| # | 项 | 处置 |
| --- | --- | --- |
| K-1 | 真实硬件故障（MIPI/I2C 掉线）无稳定可重复的注入手段，且验收方不得改动板端硬件 | `AC-RB-08`/`AC-RB-12` 以**代码级注入**为 P0 判定；真实故障仅机会性观测（P1）。"驱动层真实故障行为"在 v0.1.0 属未完全验证，写入已知限制 |
| K-2 | `SIGKILL` 后 MIPI/VIN/ISP 资源回收依赖内核/驱动，可能超过 60 s 或需复位 | `AC-RB-07` 为 P1；文档必须给出可执行的恢复步骤，否则须写明"崩溃后请重启板卡" |
| K-3 | ~~NV12 `height` 约定冲突~~ → **已由 E2 关闭** | 结论：`height` = **真实高度 `H`**（填 `1.5H` 会让 codec 段错误，退出码 `-11`）。剩余风险仅为冻结书/测试计划旧文本未同步（`B-03`/`B-04`），由 `AC-IF-01` 阻塞 |
| K-4 | CameraInfo 的 `K` 是否真的"已按输出分辨率回写"（冻结书 §3.4 的前提） | `AC-FUN-07` 的附加区分性检查（`raw` 与 `resize` 下 SDK `K` 是否相同）判定；若相同 ⇒ 阻塞缺陷并需按文档化规则换算 |
| K-5 | 默认参数的可持续性：**图像速率部分已由 E3 实测支持**（29.966 / 29.974 Hz），**IMU 速率（`odr=200`）与 `raw` 模式未实测** | 若实测不达标，唯一处置是改默认值并同步三份文档；不得放宽 §5 阈值 |
| K-6 | IMU 的 FSYNC 突发语义使"瞬时速率"无意义 | 判据只针对 30–60 s 平均与 1 s 窗口占比（`AC-PERF-03`）；**禁止**用最大包间隔判定 |
| K-7 | `web_port=8000` 由既有 nginx 承载，本包不启动 nginx | 若环境缺少既有 nginx 配置，`AC-FUN-09`① 失败属**环境问题**；须保存 nginx 进程与端口占用证据以区分"本包缺陷"与"环境未就绪"（E4） |
| K-8 | `start()` 可无限阻塞（等 IMU FSYNC 握手，SDK 不可中断） | `AC-RB-06`② 按冻结书 §7.5 判定（告警 + 不中止 + 最终干净退出），此时 `AC-PERF-04` 的 20 s 上限不适用并须在证据中登记实测等待时长 |
| K-9 | `hobot_shm` 使能开关的实际效果（是否真的影响既有节点的传输路径）未在本包内可观测 | `hobot_shm:=true/false` 只作 P1 观察项；功能正确性不受影响，不得据此判 FAIL |
| K-10 | `/imu/data` 的 QoS（`BEST_EFFORT` + depth 200）**未经过类似 codec 的消费者兼容性实测**（E1 只覆盖图像） | 若下游出现 `RELIABILITY_QOS_POLICY` 类不兼容，按契约变更处理；v0.1.0 不作为阻塞项，但用户文档必须写明"IMU 订阅方需匹配 QoS" |
| K-11 | 图像 `RELIABLE` + `KEEP_LAST depth=1` 在订阅者滞后时的背压行为（发布阻塞或丢帧）未被实测 | `AC-PERF-01/02` 在慢订阅者场景下可能暴露；若出现发布阻塞，按变更控制调整 `image_qos_depth`（同步冻结书），不得静默修改 |
| K-12 | **nginx 孤儿清理与"不得修改既有节点"（冻结书 §8.4）存在张力**（E4） | `AC-RB-15` 要求"关停后占用回基线"；若无法在不改动既有节点的前提下清理，须由接口评审裁决：或由本包承担清理、或在用户文档给出显式清理步骤（`B-11`） |
| K-13 | `publish_per_eye` 与硬件拼接互斥（`B-10`）可能造成"开启后 T1 消失"的用户困惑 | 用户文档必须给出组合矩阵（`stereo_layout` × `publish_per_eye` × 实际存在的话题），并由 `AC-FUN-03`③ 判定声明与实测一致 |
| K-14 | `mode:=raw`（必须 `1088×1280`，E7）与默认 `stereo_layout=left_right` 的组合行为未实测：硬件拼接可能使输出宽度变成 2176，与 RAW 的尺寸约束冲突 | 冻结书必须声明 `raw` 下拼接的允许性与处理方式；`AC-IF-04` 只要求"实测与文档声明一致"；若不支持，实现必须给出明确退出码与文本（不得静默降级为不发布 T1） |

---

## 附录 A：自包含判定脚本（放入 `$EVID`，逐字执行）

### A.1 模块与公开函数 docstring（`$EVID/check_docstrings.py`）
```python
import ast, pathlib, sys
root = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else "ros/gs130_ros")
bad = []
for path in sorted(root.rglob("*.py")):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    if not ast.get_docstring(tree):
        bad.append("%s: module" % path)
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            if not node.name.startswith("_") and not ast.get_docstring(node):
                bad.append("%s:%d %s" % (path, node.lineno, node.name))
print("\n".join(bad))
sys.exit(1 if bad else 0)
```

### A.2 函数长度 / 未使用参数 / 孤儿模块（`$EVID/check_source.py`）
```python
import ast, pathlib, sys
root = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else "ros/gs130_ros")
problems = []
for path in sorted(root.rglob("*.py")):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            length = (node.end_lineno or node.lineno) - node.lineno + 1
            if length > 40:
                problems.append("%s:%d %s is %d lines" % (path, node.lineno, node.name, length))
            used = {n.id for n in ast.walk(node) if isinstance(n, ast.Name)}
            for arg in list(node.args.args) + list(node.args.kwonlyargs):
                if arg.arg not in ("self", "cls") and arg.arg not in used:
                    problems.append("%s:%d %s unused arg %s" % (path, node.lineno, node.name, arg.arg))
print("\n".join(problems))
sys.exit(1 if problems else 0)
```

### A.3 launch 参数是否全部被使用（`$EVID/check_launch_args.py`）
```python
import ast, pathlib, sys
problems = []
for path in sorted(pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else "ros/gs130_ros/launch").glob("*.py")):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    declared, used = set(), set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            fname = getattr(node.func, "id", "")
            if fname == "DeclareLaunchArgument" and node.args:
                declared.add(getattr(node.args[0], "value", None))
            if fname == "LaunchConfiguration" and node.args:
                used.add(getattr(node.args[0], "value", None))
    for item in sorted(x for x in declared - used if x):
        problems.append("%s: declared but unused: %s" % (path, item))
print("\n".join(problems))
sys.exit(1 if problems else 0)
```

### A.4 图像字段断言（`$EVID/frame_fields.py`）
```python
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

TOPICS = ["/image_combine_raw", "/image_left_raw", "/image_right_raw"]
EXPECT = {  # 默认 resize 640x480：T1 拼接 / T2,T3 单目（height = 真实高度 H，E2）
    "/image_combine_raw": (1280, 480, 1280, 921600, "camera"),
    "/image_left_raw":    (640, 480, 640, 460800, "camera_left"),
    "/image_right_raw":   (640, 480, 640, 460800, "camera_right"),
}

class Probe(Node):
    def __init__(self):
        super().__init__("frame_fields")
        self.seen = {}
        for topic in TOPICS:
            self.create_subscription(Image, topic,
                                     lambda m, t=topic: self.seen.setdefault(t, m),
                                     qos_profile_sensor_data)

rclpy.init()
node = Probe()
while rclpy.ok() and len(node.seen) < len(TOPICS):
    rclpy.spin_once(node, timeout_sec=0.5)
fail = 0
for topic in TOPICS:
    msg = node.seen.get(topic)
    if msg is None:
        print("%s MISSING (publish_per_eye:=true 时才要求 T2/T3)" % topic); fail += 1; continue
    got = (msg.width, msg.height, msg.step, len(msg.data), msg.header.frame_id)
    ok = got == EXPECT[topic] and msg.encoding == "nv12" and msg.is_bigendian == 0
    print("%s %s got=%s expect=%s" % ("PASS" if ok else "FAIL", topic, got, EXPECT[topic]))
    fail += 0 if ok else 1
node.destroy_node(); rclpy.shutdown()
sys.exit(1 if fail else 0)
```

### A.5 帧对配对与 IMU 窗口统计（`$EVID/pair_stamps.py` / `$EVID/imu_windows.py`）
```python
# pair_stamps.py（要点）
# 1) 订阅 /image_combine_raw、/image_left_raw、/image_right_raw（sensor_data QoS）
# 2) 按 header.stamp 配对（|Δt| <= 1ms 视为同一帧对）；统计 100 对：
#    输出 satisfied/total、max_dt_ns
# 3) 注意：不要求对拼接帧做像素级切分/重组（E6：使用 SDK 硬件拼接，不奖励手写拼接）；
#    左右目位置与几何正确性分别由 AC-FUN-02（字段）与 AC-FUN-03（遮挡法，附录 A.10）判定
```

```python
# imu_windows.py（要点）
# 订阅 /imu/data，采集 >= 60 s；按 1 s 窗口统计包数 w_i
# 判据：mean(w_i) 落在 [0.95*odr, 1.05*odr]；sum(w_i) 与 odr*duration 偏差 <= 1%；
#       w_i 落在 [0.8*odr, 1.2*odr] 的窗口占比 >= 95%
# 注意：契约规定 IMU 突发，禁止使用"最大包间隔"
```

```python
# imu_windows.py（要点）
# 订阅 /imu/data，采集 >= 60 s；按 1 s 窗口统计包数 w_i
# 判据：mean(w_i) 落在 [0.95*odr, 1.05*odr]；sum(w_i) 与 odr*duration 偏差 <= 1%；
#       w_i 落在 [0.8*odr, 1.2*odr] 的窗口占比 >= 95%
# 注意：契约规定 IMU 突发，禁止使用"最大包间隔"
```

### A.6 SDK 标定/时间戳比对（`$EVID/calib_check.py`）
```python
# 1) 读消息：ros2 topic echo --once /image_left/camera_info（K/D/distortion_model/R/P/width/height/stamp）
# 2) SDK 直读（会占用相机，必须与节点串行）：
#      cfg = gs130.Config.preset("RDKX5","GS130WI",gs130.CameraMode.RESIZE,640,480,30,200)
#      dev = gs130.Device(cfg); ci = dev.camera_intrinsics(gs130.CameraIndex.LEFT); dev.close()
# 3) 断言消息 K == ci.K 逐元素（相对误差 <= 1e-9），width/height == 640/480
# 4) 断言 R 全 0（9）、P 全 0（12）、binning 0、roi 全 0、do_rectify False
# 5) 断言 distortion_model/D 组合符合冻结书 §3.4.1
# 6) 时间戳：同时对同一帧记录 SDK Image.timestamp_ns 与消息 header.stamp，
#    断言 stamp == timestamp_ns + offset_ns（offset 取自统计日志）
```

### A.7 JPEG 解码尺寸与截图差异（`$EVID/jpeg_dims.py` / `$EVID/img_diff.py`）
```python
# jpeg_dims.py：ros2 topic echo --once /image_combine_jpeg 保存 data 到 $EVID/frame.jpg
#   再从 CompressedImage 消息反序列化，用 cv2.imdecode 解出图像，断言 shape == (480, 1280, 3)
#   （这是 B-03 的区分性判据：若 hobot_codec 把 UV 当亮度，尺寸/内容会不符）
```

```python
import sys
import numpy as np
from PIL import Image
a = np.asarray(Image.open(sys.argv[1]).convert("L"), dtype=np.int16)
b = np.asarray(Image.open(sys.argv[2]).convert("L"), dtype=np.int16)
h = min(a.shape[0], b.shape[0]); w = min(a.shape[1], b.shape[1])
print("diff_ratio=%.3f" % (np.abs(a[:h, :w] - b[:h, :w]) > 8).mean())
```

### A.8 死代码与静默异常（shell）
```bash
grep -rnE '^\s*#.*[=(]' $REPO/ros/gs130_ros --include='*.py' | head -40   # 人工确认无"连续>=5行被注释的代码"
grep -rnE 'if (False|0):' $REPO/ros/gs130_ros --include='*.py'
grep -rnE 'except\s*:' $REPO/ros/gs130_ros --include='*.py'                # 裸 except
grep -rnE 'except[^:]*:\s*pass' $REPO/ros/gs130_ros --include='*.py'       # 静默吞异常（仅 close() 路径允许，须有注释）
grep -rn 'respawn' $REPO/ros/gs130_ros/launch/*.py                         # 必须为空
```

### A.9 `data` 赋值形式（E3 佐证，`AC-SRC-13`）
```bash
# 必须无输出（禁止逐元素赋值）
grep -rnE 'msg\.data *= *(list|bytearray)\(|\.data\[[A-Za-z_]+\] *=' $REPO/ros/gs130_ros --include='*.py'
# 必须至少有一处整块缓冲赋值
grep -rnE 'array\(.B.\)|frombytes|tobytes\(\)|bytes\(' $REPO/ros/gs130_ros --include='*.py'
# 端到端判据：AC-PERF-01（29–31 Hz@30fps）。实测逐元素赋值 1103.87 ms/帧，必然不达标
```

### A.10 遮挡法左右目位置（`$EVID/occlusion_check.py`，`AC-FUN-03`）
```python
# 步骤（每个状态各保存 3 张 PNG 到 $EVID/occ/<state>/）：
#   1) 无遮挡 -> 3 张；2) 遮挡左目 5 s -> 3 张；3) 遮挡右目 5 s -> 3 张
# 对 /image_combine_raw 每帧解码（cv2.COLOR_YUV2BGR_NV12），按 width//2 切成左右半幅，
# 计算每个状态的半幅灰度均值 mean_L / mean_R（各状态取 3 帧平均）：
#   判据：mean_L(遮挡左) <= 0.5 * mean_L(无遮挡) 且 mean_R(遮挡左) >= 0.9 * mean_R(无遮挡)
#         mean_R(遮挡右) <= 0.5 * mean_R(无遮挡) 且 mean_L(遮挡右) >= 0.9 * mean_L(无遮挡)
# 输出：三组 (mean_L, mean_R) 与 PASS/FAIL
```

### A.11 关停基线（`$EVID/shutdown_baseline.sh`，`AC-PERF-07`⑤⑥ / `AC-RB-15`）
```bash
# 启动前（基线）
{ pgrep -af "nginx|hobot_codec|websocket"; echo ---; ss -ltn | grep :8000; } > $EVID/base.txt
# ... 启动、运行 30 s、SIGINT、等待 10 s 后
{ pgrep -af "nginx|hobot_codec|websocket"; echo ---; ss -ltn | grep :8000; } > $EVID/after.txt
diff <(sort $EVID/base.txt) <(sort $EVID/after.txt)   # 期望：无新增行（== 通过）
# 连续 3 轮，每轮都要 diff 通过
```

---

## 附录 B：证据、签核与放行

### B.1 必须归档的证据
| 项 | 内容 |
| --- | --- |
| 基线 | `git rev-parse HEAD`、`git status --porcelain`（须为空）、`cat VERSION`、`gs130.__version__`/`library_version()`、TROS 版本、冻结书版本 |
| 命令与输出 | 每条 AC 的命令原文（含 `timeout` 与环境变量）+ 完整 stdout/stderr + `rc=$?` |
| 结构性快照 | `ros2 topic list -t`、`ros2 topic info -v`（全部必需话题）、`ros2 node list`、`ros2 param list /gs130_node`、两个 launch 的 `--show-args` |
| 量化 | `topic hz`/`topic delay` 原始输出、IMU 1 s 窗口统计、RSS/fd/线程采样表、CPU 采样表 |
| 图像 | ≥10 张 PNG（左右各 5）+ 拼接帧解码图 + 网页截图 2 张（挥手前后） |
| 结论 | `$EVID/summary.md`：每个 AC 一行 `PASS/FAIL/BLOCKED` + 证据路径 + 缺陷号 `GS130ROS-<nnn>` |
| 偏离 | `$EVID/deviations.md`：§0.1 变更控制下的每一次契约/判据修改 |

### B.2 放行判据（最终）
1. §1 中全部 **P0** 项 `PASS`；
2. §11 的 `B-01`…`B-11` 全部关闭；
3. `AC-RB-05`（20 轮）与 `AC-PERF-07`（干净退出 + 相机释放）通过；
4. `AC-FUN-03` 与 `AC-FUN-09` ③ 通过（拼接布局与 NV12 语义的正确性锚点）；
5. P1 项若 FAIL，已在用户文档"已知限制"与发布说明中登记；
6. §13 的 K-1…K-9 已按各自处置写入已知限制。

### B.3 签核
| 角色 | 职责 | 签署 |
| --- | --- | --- |
| DEV（实现） | 提供变更清单、修复缺陷、保证可复现构建 | |
| QA-1（测试执行） | 执行 TC 并提交证据 | |
| QA-2（本文件作者） | 判定门禁、关闭/驳回例外、维护判据与冻结镜像 | |
| 接口评审（架构/PM） | 裁决 §11 的契约澄清与 §13 的风险接受 | |
