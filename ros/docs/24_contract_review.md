# 24 契约合规评审报告（REV-2）

- 文档编号：24（Contract Compliance Review）
- 评审角色：REV-2（契约合规评审，独立于实现方与测试执行方）
- 评审对象：`ros/gs130_ros/**`（交付实现）+ `ros/docs/00…32`（规范文档集）
- 权威链（本次评审使用的唯一裁决规则）：
  1. **实测事实最高**（本次任务给定的板端实测结果 + `00_verified_platform_facts.md` 的 E1–E7/D1–D8）；
  2. 其次是 `11_interface_freeze.md`（FROZEN，含其在 22:18 加入的"修订通知"，该通知自称优先级高于本文其余内容）；
  3. 再次是 `21_acceptance_criteria.md` 的 §2 验收镜像与 §2.0 的 E1–E7 修正；
  4. `01/02/03/10/12/20/22/30/31/32` 为提案/背景文档，与 1–3 冲突时一律以 1–3 为准。
- 评审方法：只读代码 + 只读文档 + 只读 git 状态；**未 ssh 板卡、未运行相机、未运行任何 ROS 命令、未修改除本文件外的任何文件**。
- 快照口径：`ros/` 在本次评审期间被其它角色并发修改了**两轮**（第二轮把 launch 参数抽成 `gs130_ros/launch_arguments.py`、并调整了 `camera_node.py`/`README.md`）。**本报告的全部行号与哈希对应下表快照（最后核对时间 `2026-09-14 22:30`）**；两轮改动均未改变本报告的结论，只改变了行号。

## 0. 评审快照（sha256，用于复现判定）

| 文件 | sha256（前 16 位） | 行数 |
|---|---|---|
| `ros/docs/00_verified_platform_facts.md` | `b2de9990c0476347` | 132 |
| `ros/docs/11_interface_freeze.md` | `2bdad2a3f9fca8db` | 684 |
| `ros/docs/21_acceptance_criteria.md` | `3e157f1d79ec01dd` | 826 |
| `ros/gs130_ros/gs130_ros/camera_node.py` | `4f1248fe606dade3` | 442 |
| `ros/gs130_ros/gs130_ros/calibration.py` | `fa8ab3deb95d8793` | 114 |
| `ros/gs130_ros/gs130_ros/launch_arguments.py` | `0daf333bda384969` | 35 |
| `ros/gs130_ros/launch/gs130_camera.launch.py` | `983ab6e6ab38c196` | 27 |
| `ros/gs130_ros/launch/gs130_web.launch.py` | `12a8e8b3d5001908` | 77 |
| `ros/gs130_ros/setup.py` | `b534240ca2d06966` | 22 |
| `ros/gs130_ros/package.xml` | `f39323c2727630c9` | 18 |
| `ros/README.md` | `8066caaf46d2c21d` | 161 |
| `ros/test/capture_frame.py` | `289c23261ef18d36` | 107 |
| `ros/test/test_calibration.py` | `d71e88ea51764ccb` | 167 |

补充事实（只读 git）：

```
$ git rev-parse HEAD        -> efa7abc4e9de132d3b4762536f3c8204490e537e
$ git status --porcelain    -> " M .gitignore" 与 "?? ros/"
$ git status --porcelain python/ core/  -> （空）
```

即：`ros/` 全部内容**未纳入版本控制**，工作区不干净（`AC-BLD-04` 的判定对象）。

评审快照期间新增（不在本次规定评审范围内，仅记录）：`ros/docs/25_test_report.md`、`ros/docs/33_user_review_round2.md`、`ros/docs/40_process_record.md`、`ros/test/test_calibration.py`、`ros/gs130_ros/gs130_ros/launch_arguments.py`。

## 0.1 一分钟结论

| 项 | 结果 |
|---|---|
| 合规矩阵行数 / 不符合行数 | 107 行冻结项中 **47 行 MATCHES、38 行 DIVERGES、22 行 NOT IMPLEMENTED**（共 **60 行不符合**） |
| 文档互相矛盾条数 | **21 条**（D-01 … D-21） |
| 交付包能否通过 `21_acceptance_criteria.md` 发布门禁 | **不能**。72 个 AC 中 **8 PASS / 44 FAIL / 20 UNVERIFIED**（P0 66 项中 6 PASS / 42 FAIL / 18 UNVERIFIED）；§B.2 的 6 条放行判据全部不满足 |
| 最危险的剩余分歧 | 冻结书正文仍冻结 `Image.height = eh*3//2`（`11_interface_freeze.md:195/220/646`），而实测（E2）与该值会让既有 `hobot_codec_republish` **段错误（-11）**；现实与实现都已改为 `height = eh` |

---

## 1. 合规矩阵（`11_interface_freeze.md` 冻结项 vs 交付实现）

判定用词只有三种：**MATCHES**（逐字/逐值一致）、**DIVERGES**（实现与冻结值不同）、**NOT IMPLEMENTED**（冻结项在实现中不存在）。
凡冻结书正文与它自己的"修订通知（L3–L21，优先级更高）"或与实测冲突的行，**按修订通知/实测判定**，正文的失效文本在 §2 记为 D-xx，不重复计入本矩阵。

### 1.1 §1 包身份与依赖（冻结书 §1.1–1.3）

| # | 冻结项（值） | 判定 | 证据（文件:行） |
|---|---|---|---|
| C01 | ROS 包名 `gs130_ros` | MATCHES | `ros/gs130_ros/package.xml:4`、`ros/gs130_ros/setup.py:3` |
| C02 | 节点名 `gs130_node` | DIVERGES | `camera_node.py:83` `super().__init__("gs130_camera")`；`launch/gs130_camera.launch.py:23`、`launch/gs130_web.launch.py:42` `name="gs130_camera"` |
| C03 | 可执行名 `gs130_node` | DIVERGES | `setup.py:21` `"camera_node = gs130_ros.camera_node:main"`；两个 launch `executable="camera_node"`（`gs130_camera.launch.py:22`、`gs130_web.launch.py:41`） |
| C04 | 包版本 `0.1.0` | MATCHES | `package.xml:5`、`setup.py:7` |
| C05 | 构建类型/layout `ament_python` + `package.xml`/`setup.py`/`setup.cfg`/`resource/gs130_ros`/`gs130_ros/`/`launch/` | MATCHES | `package.xml:16`；目录齐备（多出内部模块 `calibration.py`，不改对外接口） |
| C06 | 安装后入口点 `console_scripts: gs130_node = gs130_ros.gs130_node:main` | DIVERGES | `setup.py:21`（模块名与入口名都不同） |
| C07 | 安装后 launch 路径 `share/gs130_ros/launch/{gs130_web,gs130_camera}.launch.py` | MATCHES | `setup.py:14-17` |
| C08 | `package.xml` 的 `<description>` 逐字 | DIVERGES | `package.xml:6` 为 `ROS 2 interface for the GS130 stereo camera and IMU on RDK X5.`（冻结书 §1.2 为 `ROS 2 (TROS) interface … on the D-Robotics RDK X5.`） |
| C09 | `maintainer email="dev@d-robotics.cc"` + `D-Robotics` | DIVERGES | `package.xml:7` `dev@example.com` / `gs130_sdk maintainers`（`<license>MIT</license>` 一致） |
| C10 | `<url type="repository">` | NOT IMPLEMENTED | `package.xml` 全文无 `<url>` |
| C11 | 4 个 `<buildtool_depend>`（`ament_copyright`/`ament_flake8`/`ament_pep257`/`python3-setuptools`） | NOT IMPLEMENTED | `package.xml:10-13` 只有 `<depend>` |
| C12 | `exec_depend`：`rclpy`/`std_msgs`/`sensor_msgs`/`geometry_msgs`/`diagnostic_msgs`/`launch`/`launch_ros` | DIVERGES | `package.xml:10-13` 声明 `rclpy`/`sensor_msgs`/`geometry_msgs`/**`tf2_ros`**；缺 `std_msgs`/`diagnostic_msgs`/`launch`/`launch_ros`；`tf2_ros` 是 §0.2 明文排除的依赖 |
| C13 | 4 个 `<test_depend>` | NOT IMPLEMENTED | `package.xml` 无 `<test_depend>` |
| C14 | §1.3 运行时依赖含 `numpy`（"节点启动即失败"） | DIVERGES | 实现不 import numpy（`camera_node.py:8-20`）；README:18 仍列为依赖 |

### 1.2 §2 话题表（冻结书 §2.1–§2.3）

| # | 冻结项（值） | 判定 | 证据 |
|---|---|---|---|
| C15 | T1 名/类型 `/image_combine_raw` + `sensor_msgs/msg/Image` | MATCHES | `camera_node.py:144` |
| C16 | T1 reliability = `RELIABLE`（修订通知 L10 = E1 = 21 §2.2） | MATCHES | `camera_node.py:33-38` `IMAGE_QOS` |
| C17 | T1 durability/history/depth = `VOLATILE`/`KEEP_LAST`/`1` | MATCHES | `camera_node.py:33-38` |
| C18 | T1 `frame_id` 默认值 `camera` | MATCHES | `camera_node.py:275`（硬编码 `"camera"`） |
| C19 | T1 `frame_id` 由参数 `frame_id_combine` 提供 | NOT IMPLEMENTED | 无该参数（`camera_node.py:84-96`） |
| C20 | T1 频率 = `fps` | MATCHES | `camera_node.py:157`（每 tick 最多 2 帧）；实测 29.997 Hz |
| C21 | T2/T3 由参数 `publish_per_eye`（默认 `false`）控制，可与 T1 同帧并存 | DIVERGES | `camera_node.py:144-147`、`camera_node.py:274-278`：T2/T3 只在 `stereo_layout=none` 时创建，与 T1 **互斥**；无 `publish_per_eye` 参数 |
| C22 | T2/T3 `frame_id` 由参数 `frame_id_left`/`frame_id_right` 提供 | NOT IMPLEMENTED | `camera_node.py:277-278`：左目用 `frame_id_camera`（默认 `camera_left`），右目硬编码 `"camera_right"` |
| C23 | T2/T3 与 T1 共用同一帧数据、同一 `stamp`，禁止二次读设备凑对 | MATCHES | `camera_node.py:265-278`（一次 `read_image()` 的同一 dict） |
| C24 | T4 名/类型 `/imu/data` + `sensor_msgs/msg/Imu` | MATCHES | `camera_node.py:150` |
| C25 | T4 reliability = `BEST_EFFORT`（§2.2 L155、21 §2.2 L185） | DIVERGES | `camera_node.py:39-44` `IMU_QOS` 为 **`RELIABLE`**（实测未覆盖该话题，21 `K-10`） |
| C26 | T4 depth = `200` | MATCHES | `camera_node.py:43` |
| C27 | T4 depth 由参数 `imu_qos_depth` 提供 | NOT IMPLEMENTED | 无该参数 |
| C28 | T4 `frame_id` 默认 `imu_link`（参数 `frame_id_imu`） | MATCHES | `camera_node.py:93`、`305` |
| C29 | `device:=GS130W` 时 **不创建** T4，且 `WARN` 一次 `IMU not present; /imu/data will not be published` | DIVERGES | `camera_node.py:150` 无条件创建 publisher；全文除 TF 分支（`camera_node.py:244`）外不检查 `imu_name`，无该 WARN |
| C30 | T4 不丢包/不重排/不"平滑"重发 | MATCHES | `camera_node.py:291-302`（每 tick drain 至多 64 包，按 SDK 顺序发布）；实测 ≈203 Hz |
| C31 | T5/T6 名与类型 `/image_left/camera_info`、`/image_right/camera_info` | MATCHES | `camera_node.py:218-219` |
| C32 | T5/T6 QoS = `RELIABLE` + `TRANSIENT_LOCAL` + `KEEP_LAST(1)`，启动时各 1 次（latched） | MATCHES | `camera_node.py:45-50`、`214-225`；实测 latched |
| C33 | T5/T6 `frame_id` 与对应 Image 一致（`camera_left`/`camera_right`） | MATCHES | `camera_node.py:222-223` |
| C34 | T5/T6 `R` = 9 个 0.0 | DIVERGES | `calibration.py:48` 写入单位阵；实测现实亦为单位阵 |
| C35 | T5/T6 `P` = 12 个 0.0 | DIVERGES | `calibration.py:49-53` 由 K 重排填充；实测现实亦已填充 |
| C36 | T5/T6 `D`/`distortion_model` 映射（FISHEYE→`equidistant`+4；PINHOLE→`plumb_bob`+5；`rect`→强制 `plumb_bob`+5 个 0；参数 `camera_info_distortion_model` 覆盖；`none`→`[]`） | DIVERGES | `calibration.py:12-25`：全零→`plumb_bob`+5 个 0 ✓；FISHEYE→`equidistant`+4 ✓；**PINHOLE→`rational_polynomial`+8 元**（冻结值为 `plumb_bob`+5 元）；无 `camera_info_distortion_model`/`none` 分支。默认 `resize` 下实测 `equidistant` + D 长度 4 ✓ |
| C37 | T7 `/image_combine_jpeg` 由既有 codec 发布 | MATCHES | `launch/gs130_web.launch.py:48-61`（参数名不逐字，见 C63/C64） |
| C38 | T8 `/gs130/status`（`diagnostic_msgs/msg/DiagnosticStatus`，1 Hz，默认关闭） | NOT IMPLEMENTED | 实现无该话题、无 `publish_status`、`package.xml` 无 `diagnostic_msgs` |
| C39 | §2.1 绝对话题名、不使用命名空间、无 `ns` 参数 | MATCHES | 全部话题字面量为绝对名（`camera_node.py:144-148`、`216-217`） |

### 1.3 §3 消息构造与字段

| # | 冻结项（值） | 判定 | 证据 |
|---|---|---|---|
| C40 | `encoding` 逐字 `"nv12"` | MATCHES | `camera_node.py:406`；实测 `nv12` |
| C41 | `is_bigendian = 0` | MATCHES | `camera_node.py:407` |
| C42 | `step = width` | MATCHES | `camera_node.py:410`；实测 1280 / 640 |
| C43 | `height`（冻结正文 `eh*3//2`；修订通知 L11 = E2 = 21 §2.3 = 实测） | DIVERGES（对正文）/ MATCHES（对修订通知与实测） | `camera_node.py:409` `height = shape[0]*2//3`；实测 480 |
| C44 | T1 `width = 2*ew`、`len(data) = 2*ew*eh*3//2` | MATCHES | `camera_node.py:408-411`；实测 `width=1280`、`len=921600` |
| C45 | T1 左目在左、右目在右，由 SDK 硬件拼接产生（修订通知 L12 = E6） | MATCHES | `camera_node.py:196` 在 `Config.preset()` 之后设 `stereo_layout`；无任何 `concatenate`；实测网页链路显示拼接图 |
| C46 | 左右目位置（遮挡法，21 AC-FUN-03②） | MATCHES（未实测） | 由 SDK `LEFT_RIGHT` 语义保证；遮挡法证据未提供 |
| C47 | §3.2 `read_image()` 返回 `None` → 不发消息、计一次 `drop`、不报错退出 | DIVERGES | `camera_node.py:271-272` 直接 `return`，无 drop 计数 |
| C48 | §3.3 `orientation.x/y/z=0.0`、`w=1.0` | MATCHES | `camera_node.py:314` |
| C49 | §3.3 `orientation_covariance[0] = -1.0` | MATCHES | `camera_node.py:318` |
| C50 | §3.3 `angular_velocity_covariance[0] = -1.0` | DIVERGES | 未赋值，保持 `0.0`（`camera_node.py:305-319`）；21 AC-FUN-05⑤ 明文"不得为 0" |
| C51 | §3.3 `linear_acceleration_covariance[0] = -1.0` | DIVERGES | 同上 |
| C52 | §3.3 `gyro` rad/s、`accel` m/s²（含重力），不翻转/不换轴 | MATCHES | `camera_node.py:308-313` |
| C53 | §3.3 温度丢弃、`is_fsync` 不发布 | MATCHES | 无相关话题/字段 |
| C54 | §3.4 `height/width = ew/eh`（不是 `eh*3//2`） | MATCHES | `camera_node.py:222-223`；实测 640x480 |
| C55 | §3.4 `K` = SDK `K` 逐元素、驱动不得再缩放 | MATCHES | `calibration.py:46`；实测 `fx=386.85 fy=387.12 cx=304.62 cy=245.06` |
| C56 | §3.4 `D` 长度与 `distortion_model` 匹配 | MATCHES（默认配置） | 实测 `equidistant` + D 长度 4 |
| C57 | §3.4 `binning=0`、`roi` 全 0、`roi.do_rectify=False` | MATCHES | 未写入，保持消息默认值（`calibration.py:41-54`） |
| C58 | §3.4 `header.stamp` 非 0（启动时刻） | MATCHES | `camera_node.py:224` |

### 1.4 §4 时间戳

| # | 冻结项 | 判定 | 证据 |
|---|---|---|---|
| C59 | 启动时一次性计算 `offset_ns`，运行期恒定 | MATCHES | `camera_node.py:323-341` |
| C60 | 图像与 IMU 共用同一 offset，不做本地打点/插值 | MATCHES | `camera_node.py:353-357`（`stamp_of` 同时服务两者） |
| C61 | 参数 `stamp_offset_ns`（强制偏移） | NOT IMPLEMENTED | 无该参数 |
| C62 | 参数 `stamp_offset_mode`（`auto`/`device`） | NOT IMPLEMENTED | 无该参数；实现只有 auto 一种行为 |
| C63 | §4.3 零/不可用/回退时间戳的降级与 `WARN` | NOT IMPLEMENTED | 无任何时间戳校验分支（`camera_node.py:353-357`） |
| C64 | §4.2 第 5 条：检测 `use_sim_time=True` 时 `WARN` 并以系统时钟继续 | NOT IMPLEMENTED | 无该检测 |
| C65 | 节点时钟使用 `RCL_ROS_TIME` | MATCHES | `camera_node.py:224`（rclpy 默认时钟） |

### 1.5 §5 参数表

| # | 冻结项 | 判定 | 证据 |
|---|---|---|---|
| C66 | §5.1 中实现的 10 个参数名与默认值（`platform`/`device`/`mode`/`width`/`height`/`fps`/`odr`/`frame_id_imu`/`publish_imu`/`start_timeout_s`） | MATCHES | `camera_node.py:84-96` 与冻结书 §5.1 逐值一致（`RDKX5`/`GS130WI`/`resize`/`640`/`480`/`30`/`200`/`imu_link`/`True`/`10.0`） |
| C67 | 其余 14 个节点参数：`frame_id_combine`、`frame_id_left`、`frame_id_right`、`publish_combine`、`publish_per_eye`、`publish_camera_info`、`publish_status`、`stamp_offset_ns`、`stamp_offset_mode`、`camera_info_distortion_model`、`poll_period_ms`、`imu_qos_depth`、`image_qos_depth`、`log_fps_period_s` | NOT IMPLEMENTED | `camera_node.py:84-96` 只有 13 个 `declare_parameter` |
| C68 | §5.5 非目标参数不得实现 | DIVERGES | 多出 3 个：`stereo_layout`（§5.5 L419 明列禁止）、`frame_id_camera`（冻结书无此名）、`publish_tf`（§5.5 L427 明列禁止） |
| C69 | `width`/`height` 取值域 `[16,1088]`/`[16,1280]` 且必须偶数 | DIVERGES | `camera_node.py:183-184` 只判 `>= 1`；`width:=641` 会通过校验 |
| C70 | `fps` 取值域 `[1,33]` | DIVERGES | `camera_node.py:181-182` 只判 `>= 1`；`fps:=99` 会通过校验 |
| C71 | `odr` 仅接受 `200`/`500` | DIVERGES | 无校验（`camera_node.py:174-187`），`odr:=100` 直达 `Config.preset()` |
| C72 | `mode` 映射与 `mode:=raw` 必须 1088x1280（启动参数校验阶段拒绝，退出码 2） | MATCHES | `camera_node.py:52-56`、`183-185`、`207-210`；实测拒绝且退出码 2 |
| C73 | `Config.preset()` 非法 platform/device → `ERROR` + 退出码 2 | NOT IMPLEMENTED | `camera_node.py:189-207` 未捕获 `ValueError`（只捕获 `GS130Error`/`OSError`） |
| C74 | `Reconfig=No`（不注册动态回调） | MATCHES | 无 `on_set_parameters`；README:111 已说明 |
| C75 | launch-only 参数 `codec_channel`/`websocket_channel`/`web_port`/`web_output_fps`/`jpg_quality`/`smart_topic`/`hobot_shm`（不得出现在节点参数表） | DIVERGES | 未进节点参数表 ✓；但 launch 只声明了 `jpg_quality`（默认 `"85.0"`，冻结值 `80`）与 `websocket_channel`（默认 `"0"`，冻结值 `1`），其余 5 个未声明（`gs130_web.launch.py:21-26`） |

### 1.6 §6 Launch 契约

| # | 冻结项 | 判定 | 证据 |
|---|---|---|---|
| C76 | 文件名 `gs130_web.launch.py` / `gs130_camera.launch.py` | MATCHES | `ros/gs130_ros/launch/` |
| C77 | `gs130_camera.launch.py` **只**启动相机节点 | MATCHES | `gs130_camera.launch.py:20-26`（唯一 `Node`） |
| C78 | `gs130_web.launch.py` 启动 `gs130_node` + `hobot_codec_republish` + `websocket`，既有节点参数逐字 | DIVERGES | `gs130_web.launch.py:48-73` 以 `IncludeLaunchDescription` 引入 `hobot_codec_encode.launch.py` 与 `websocket.launch.py`，使用 `codec_in_mode`/`codec_sub_topic`/`codec_jpg_quality`/`websocket_image_topic`/`websocket_only_show_image` 等 launch 参数名，而冻结书 §6.2 逐字规定的是节点参数 `in_mode`/`sub_topic`/`jpg_quality`/`image_topic`/`image_type`/`only_show_image`/`output_fps`/`smart_topic`/`input_framerate=-1`/`output_framerate=-1`；且 `websocket_only_show_image: "True"` 与冻结值 `only_show_image=False` 相反 |
| C79 | 两个 launch 共同声明 18 个 launch 参数 | DIVERGES | `launch_arguments.py:7-22` 声明 13 个；`launch_arguments.py:7-22` 13 个（缺 5 个：`publish_combine`/`publish_per_eye`/`publish_camera_info`/`publish_status`/`stamp_offset_mode`/`camera_info_distortion_model` 中除已有项外的全部） |
| C80 | `gs130_web.launch.py` 额外声明 7 个 web 参数（含默认值） | DIVERGES | `gs130_web.launch.py:21-26` 只声明 4 个（`image_topic`/`jpeg_topic`/`jpg_quality=85.0`/`websocket_channel=0`），缺 `codec_channel`/`web_port`/`web_output_fps`/`smart_topic`/`hobot_shm` |
| C81 | 显示语义：`websocket` `channel:=0` = 拼接帧左半 = 左目 | MATCHES | `gs130_web.launch.py:25` 默认 `0`；README:39 说明 |
| C82 | §6.6 逐字命令（`ros2 run gs130_ros gs130_node`、`publish_per_eye:=true`、`publish_status:=true`） | DIVERGES | 可执行名是 `camera_node`（C03）；`publish_per_eye`/`publish_status` 未声明，按命令执行会报未知参数 |
| C83 | §6.5 `hobot_shm` 开关 | NOT IMPLEMENTED | 无该 launch 参数 |

### 1.7 §7 错误与诊断契约

| # | 冻结项 | 判定 | 证据 |
|---|---|---|---|
| C84 | 参数校验失败：`ERROR` + 逐字 `invalid parameter {name}={value}: {reason}` + 退出码 2 | DIVERGES | 退出码 2 ✓（`camera_node.py:209-212`）；文本为 `mode must be one of raw, resize, rect, got 'x'` 等（`camera_node.py:177-187`、`:126`、`:134`），非逐字模板 |
| C85 | `Config.preset()` 非法 platform/device → 逐字 `unsupported platform/device: {platform} {device}` + 退出码 2 | NOT IMPLEMENTED | 见 C73（未捕获，traceback + 退出码 1） |
| C86 | `gs130_init` `PARAM_ERROR` 逐字含 `RAW mode requires width=1088 height=1280` / `RECT mode requires EEPROM calibration` | DIVERGES | `camera_node.py:186-187` 的文本为 `mode raw requires width=1088 height=1280, got 640x480`（大小写不同、带附加值）；`rect` 无标定无专门提示 |
| C87 | `gs130_init` `NOT_FOUND` 逐字 `gs130_init failed: NOT_FOUND: camera not detected on the configured I2C buses` | DIVERGES | `camera_node.py:199-205` 统一输出 `gs130_init failed ({error}). The camera is exclusive: …` |
| C88 | `gs130_init` `HW_ERROR` + 占用提示 + 退出码 **1** | DIVERGES | 同一分支（`camera_node.py:199-205`）退出码 **2**；文本不同 |
| C89 | `gs130_init` `UNSUPPORTED` 逐字 `gs130_init failed: UNSUPPORTED: camera cannot produce the requested configuration (mode=… )` | DIVERGES | 同上统一文本 |
| C90 | `gs130_start` 失败 → 逐字 `gs130_start failed: {CODE}` + 退出码 1 | NOT IMPLEMENTED | `camera_node.py:117` `self.device.start()` 无 try/except；异常上抛为 traceback |
| C91 | 轮询 `read_image()` `HW_ERROR`/`THREAD_CLOSED` → 逐字 `camera stream failed: {CODE}` + 退出码 1 | DIVERGES | `camera_node.py:268-270` 记 `read_image failed: %s` 后 **继续运行**（不退出） |
| C92 | 轮询 `read_image()` `TIMEOUT`（返回 `None`）→ 无日志、不退出 | MATCHES | `camera_node.py:271-272` |
| C93 | 轮询 `read_imu()` 失败 → 逐字 `imu stream failed: {CODE}; IMU publishing disabled, camera continues` + 停止发布 T4 | DIVERGES | `camera_node.py:296-298` 记 `read_imu failed: %s` 后继续轮询，未停发 |
| C94 | 无 IMU 的 `WARN` 逐字 `IMU not present; /imu/data will not be published` | NOT IMPLEMENTED | 见 C29 |
| C95 | 无标定 `WARN` 逐字 `calibration not available; …`，且不伪造内参 | NOT IMPLEMENTED | `camera_node.py:220-221` 无条件调用 `camera_intrinsics()`，无 try/except |
| C96 | 时间戳异常 `WARN` 逐字 `frame timestamp unavailable` / `IMU timestamp zero: packet dropped` | NOT IMPLEMENTED | 见 C63 |
| C97 | QoS 不匹配 `WARN` | NOT IMPLEMENTED | 无订阅端 QoS 事件处理 |
| C98 | §7.2 退出码语义：`0` 正常 / `1` 运行期致命 / `2` 参数错误 | DIVERGES | 初始化失败与"无首帧"都走 `fail()`→`2`（`camera_node.py:209-212`、`:350-351`）；运行期相机通路故障不退出（C91）；`1` 只出现在未捕获异常的 traceback |
| C99 | §7.3 统计日志逐字格式与 `log_fps_period_s` 周期 | DIVERGES | `camera_node.py:361-366` 输出 `frames=%d imu=%d` + ` \| ` 分隔的布局摘要（冻结格式为 `camera {fps:.2f} Hz, published {n_combine} combine / {n_eye} per-eye, dropped {n_drop}, imu {odr:.2f} Hz, published {n_imu}, offset {offset_ns} ns`）；周期固定 5 s（`camera_node.py:160`），无参数开关 |
| C100 | §7.4 关闭序列（停 timer → `stop()` → `close()` → `destroy_node()` → `rclpy.shutdown()`）、3 s、退出码 0、`camera released` | MATCHES | `camera_node.py:368-390`（`stop`+`close`+`camera released`）、`camera_node.py:422-438`（`KeyboardInterrupt` → 0、`finally` 释放）；实测"camera released + 无残留进程" |
| C101 | §7.4 "所有退出路径（含异常路径）都必须释放设备" | MATCHES（附注） | `camera_node.py:115-121`：设备在 `start()` **之前**赋给 `self.device`，且 `start()` 与 `_start()` 同处 `try/except BaseException: self.shutdown()` 内 → `start()` 阻塞期间到达的 `SIGINT`、`_start()` 内的任何异常都会走 `stop()+close()`。唯一残留窗口是 `gs130.Device(config)` 构造返回前后（`self.device` 尚未赋值时异常则句柄丢失），属极窄窗口，建议登记为已知边界 |
| C102 | §7.4 `close()` 失败文本 `device close failed: {CODE}; the camera may need a power cycle` | DIVERGES | `camera_node.py:386` 输出 `close failed: %s` |
| C103 | §7.5 启动等待契约（启动前 `INFO`、每 `start_timeout_s` 一条 `WARN`、SIGINT 时 `shutdown requested while gs130_start is blocking; waiting for the handshake`） | NOT IMPLEMENTED | `camera_node.py:117` 前无 `INFO`；无周期性 `WARN`；`_wait_for_first_frame` 的 2 s `INFO`（`camera_node.py:342-348`）不是该契约文本，且发生在 `start()` 之后 |
| C104 | §7.6 `/gs130/status` 的 `name`/`hardware_id`/`values` 键 | NOT IMPLEMENTED | 见 C38 |

### 1.8 §8 兼容性（相机独占）

| # | 冻结项 | 判定 | 证据 |
|---|---|---|---|
| C105 | 相机被占用：`gs130_init` 失败 → `ERROR`（`HW_ERROR`/`NOT_FOUND` 文本）+ **退出码 1** | DIVERGES | `camera_node.py:199-205` → 退出码 2、文本不同（同 C87/C88） |
| C106 | §8.3 警告文本要点写进用户文档 | DIVERGES | README:146/158 覆盖"独占、勿与 `mipi_cam` 同跑"，但缺"不要在无人值守场景使用 `respawn=True`"与 §8.3 的整段措辞 |
| C107 | §8.4 不修改任何既有 TROS 节点 | MATCHES | 无 patch、无 vendor 目录改动；但 launch 通过 `websocket_only_show_image: "True"` 改变了既有节点的行为（见 C78） |

### 1.9 计数

| 判定 | 行数 |
|---|---|
| MATCHES | 47 |
| DIVERGES | 38 |
| NOT IMPLEMENTED | 22 |
| **合计（冻结项行数）** | **107** |
| **不符合行数（DIVERGES + NOT IMPLEMENTED）** | **60** |

按冻结书章节看不符合密度（不符合 / 该章行数）：§1 身份与依赖 **10/14**（C02/C03/C06/C08/C09/C10/C11/C12/C13/C14）、§2 话题 **10/25**、§3 消息字段 **4/19**、§4 时间戳 **4/7**、§5 参数 **7/10**、§6 launch **5/8**、§7 错误契约 **18/21**、§8 独占 **2/3**。**§7 错误与诊断契约是最大的缺口**：21 行里只有 3 行逐字匹配（C92、C100、C101）。

---

## 2. 文档一致性矩阵

### 2.1 矛盾总表

裁决规则：实测（本次给定结果 + `00`）> `11`（含修订通知）> `21` §2/§2.0 > 其余。

| ID | 矛盾内容 | 冲突文档与行 | 胜出方 | 必须修正的那一句 / 那一行 |
|---|---|---|---|---|
| D-01 | 图像三话题 reliability | `11:152-154` 写 `BEST_EFFORT`；`11:423` 写"`hobot_codec_republish` 以 `BEST_EFFORT` 订阅"；`11:570` 写"QoS 不匹配（订阅者 `RELIABLE`）"；`20:42`、`20:136` 期望 `BEST_EFFORT`；`22:340` 写"参考链为 `BEST_EFFORT`" ↔ `11:10`（修订通知）、`00:11-23`、`00:125`、`21:182-184`、`10:40/299-301/843`、`32:119/505/681`、实现 `camera_node.py:33-38`、实测"Image QoS is RELIABLE" | **00 / E1** | `11` §2.2 三行的 reliability 单元格改 `RELIABLE`；删 `11:423` 的括注与 `11:570` 整行；`20:42`、`20:136` 的 QoS 期望改 `RELIABLE`；`22:340` 改 `RELIABLE` |
| D-02 | `Image.height` 语义 | `11:195`、`11:220` 写 `eh * 3 // 2`；`11:202` 标题"为何 `height = eh * 3 // 2`（不改变上表）"；`11:204` 明写"**不得**输出 `height = eh`…"；`11:646`（§9.4）、`11:664`（A3）同义 ↔ `11:11`（修订通知）、`00:25-44`、`00:126`、`21:201/206`、`10:41/367-368/842`、`02:74/143`、`32:82/120/134-135`、`20:48/137/148/418`、实现 `camera_node.py:409`、实测 `height=480` | **00 / E2** | 见 §4 的最小修复（`11:195/220/646/664` + 删除 §3.1.2） |
| D-03 | 是否手工拼接左右目 | `11:208-212` 给出 `data = concatenate([L.reshape(-1), R.reshape(-1)]).tobytes()` 与"拼接在驱动侧完成"；`11:227` "**不使用** SDK 的 `stereo_layout`"；`22:294-300` 按行交错手工拼接 `≈4.18 MB/帧` ↔ `00:101-109`、`00:130`（D6）、`21:213-214`（"不得要求也不得奖励任何手写拼接代码"）、`10:836`、`32:73`、实现无拼接代码 | **00 / E6** | 删除 `11:208-212` 的手工拼接算法与示例；删 `11:227` 整段；`22:300` 表行作废 |
| D-04 | `stereo_layout` 参数是否允许 | `11:227`、`11:419` 把 `stereo_layout` 列为"固定 `NONE`/非目标参数不得实现" ↔ `00:101-109`（E6 要求在 `preset()` 后设置它）、`21:213`（"必须使用 SDK 硬件拼接"）、`32:73/242`、实现 `camera_node.py:196`、README:104 | **00 / E6** | `11:419` 的 `stereo_layout` 行改为"v0.1.0 提供，取值 `none`/`left_right`/`right_left`/`top_bottom`/`bottom_top`，默认 `left_right`" |
| D-05 | 是否发布 `/tf`、`/tf_static` | `11:52`（§0.2 非目标）、`11:113`（"`tf2_msgs` **不**声明"）、`11:427`（§5.5 `publish_tf` 非目标）、`21:361`（AC-FUN-08 P0"不发布 TF"）、`21:402`（AC-SRC-10）、`21:469`（NA-03） ↔ `32:303`（"`/tf_static` **v0.1.0 建议做**，与冻结书相反"）、`32:448-459`、`32:734`、实现 `camera_node.py:95/234-259`、`package.xml:13`、README:69、实测 `/tf_static` 已发布、`25_test_report.md:16`（把 TF 记为 PASS） | **11**（00 未记载）→ 实现 **DIVERGES**；但实测与现实支持 32 一侧，故须走接口评审改 11 | 二选一，见 §4 备注：①实现把 `publish_tf` 默认改 `False`；或②`11:52` 删除该非目标行、`11:427` 删除、`11:113` 改为声明 `tf2_msgs`、§2.2 增补 T9 `/tf_static`（`RELIABLE`+`TRANSIENT_LOCAL`+`KEEP_LAST(1)`），并同步 `21:361/402/469` |
| D-06 | 节点参数集合与默认值 | `11` §5.1 `360-390`（24 项）↔ 实现 `camera_node.py:84-96`（13 项，含未走评审的 `stereo_layout`/`frame_id_camera`/`publish_tf`）；`01:75` 默认 `mode=RAW width=1088 height=1280 odr=100`；`03:105` `startup_timeout_s` 默认 5 s；`03:154` 参数名 `odr_hz`；`02:327` 参数 `publish_images`/`launch_codec`/`launch_websocket`；`10:209` `imu_drain_max=64`/`frame_timeout_s=3.0` 不暴露 | **11** | 实现补齐 14 项或由接口评审把其中不作要求的项明确移出 §5.1；同时把 3 个新参数**补写进** §5.1（否则属"未评审就扩契约"）；`01:75`、`03:105`、`03:154`、`02:327` 按 11 改写 |
| D-07 | launch 文件名 | `11:439-440`（`gs130_web.launch.py`/`gs130_camera.launch.py`）↔ `02:325-327`（`gs130_stereo_websocket.launch.py`/`gs130_stereo.launch.py`/`gs130_imu.launch.py`）、`03:6/70`（`gs130.launch.py`）、`12:276/288`（`launch/camera.launch.py`）、`01:118`（`gs130_websocket.launch.py`/`gs130_camera_info_only.launch.py`） | **11** | `02:325-327`、`03:6`、`03:70`、`12:276`、`12:288`、`01:118` 全部改成两个冻结名（21 `B-05`/`B-06` 早已要求，至今未执行） |
| D-08 | launch 里既有节点的参数名与取值 | `11:453/460/461/468-473`（逐字节点参数）与 `10:631-637`（明确**不 include** 厂商 launch、逐个显式 `Node()`）↔ 实现 `gs130_web.launch.py:48-73`（include 两个厂商 launch，用 `codec_*`/`websocket_*` 参数名，`only_show_image=True`、`jpg_quality=85.0`、`websocket_channel=0`） | **11 §6.2 + 10 §10.2** | 实现改为显式 `Node(hobot_codec_republish)`/`Node(websocket)` 并逐字传 §6.2 参数；若坚持 include，则必须先在 `11` §6.2 增加"经厂商 launch 传递"的等价映射表并把 `websocket_channel` 默认值统一（`B-02` 仍未关闭） |
| D-09 | 话题名 | `11` §2.2 `152-159` ↔ `01:77-79`（`/gs130/image_left_raw`、`/gs130/imu`、`/gs130/left/camera_info`）、`02:403-404`（`/imu/data_raw`、`/image_combine_raw/{left,right}/camera_info`）、`03:64-66`（`/gs130/left/image_raw`、`/gs130/imu`）、`12:139-142`（`/gs130/left/image_raw`、`/gs130/stereo/image_raw`）、`30:124-130`（`/<ns>/camera_left/image_raw`）、`31:257-262`（`/gs130/camera_left/image_raw`） | **11**（实现与 11 一致） | `01:77-79`、`02:403-404`、`03:64-66`、`12:139-142`、`30:124-130`、`31:257-262` 全部按 §2.2 改写或整表标注"历史提案、不具约束力" |
| D-10 | 图像 QoS 的 depth | `11:152`、`21:182` 规定 depth `1` ↔ `32:119/505/681`（"depth ≥ 5，与 codec `PUB_QUEUE_NUM=5` 对齐"）、`02:401`（`KEEP_LAST(10)`） | **11**（实现 depth=1） | `32:681`、`32:505`、`02:401` 的 depth 声明改为 `1`，并说明"publisher depth 只需 ≥1，codec 侧自带 depth 5" |
| D-11 | `/imu/data` reliability | `11:155`、`21:185`、`10:205/302` 规定 `BEST_EFFORT` ↔ `02:403`、`32:506`（允许统一 `RELIABLE`）、实现 `camera_node.py:39-44`（`RELIABLE`） | **11** | 实现改回 `BEST_EFFORT`，或走评审把 `11:155`+`21:185`+`10:302` 一并改为 `RELIABLE` 并在 `00` 增补决策（安全影响低：RELIABLE 发布端对 BEST_EFFORT 订阅端是兼容方向） |
| D-12 | CameraInfo 的 `R`/`P` | `11:273-274`、`11:648`（§9.6）、`11:670`（A9）、`21:240-241`、`20:49` 规定 `R`=9 个 0、`P`=12 个 0 ↔ 实现 `calibration.py:48-53`、`test_calibration.py:69-74`、README:120-121、`32:256/491`（stereonet 需要非零 `P`）、实测"R identity, P filled" | **11**（`00` 无记载）→ 实现 **DIVERGES**；实测支持实现一侧，须在 `00` 补一条实测决策后改 11/21 | `11:273`、`11:274`、`11:648`、`11:670`、`21:240`、`21:241`、`20:49` 改为"`R` = 单位阵；`P` = `[fx,0,cx,0, 0,fy,cy,0, 0,0,1,0]`（由 `K` 重排），并在用户文档声明 SDK 不提供立体校正矩阵" |
| D-13 | IMU 协方差约定 | `11:245-250`、`11:647`（§9.5）、`11:668`（A7）、`21:225-227`、`21:358`（AC-FUN-05⑤"不得为 0"）↔ 实现 `camera_node.py:314-318`（仅 `orientation_covariance[0] = -1`）、README:139-140（"协方差保持 0（未知）"） | **11** | 实现补两行 `angular_velocity_covariance[0] = -1.0`、`linear_acceleration_covariance[0] = -1.0`；否则必须改 `11`/`21` 的判据并说明 ROS 语义 |
| D-14 | 节点名 / 可执行名 | `11:67-73`（`gs130_node`）、`21:170/341/363`、`10:138/146`、`02:84-85` 均为 `gs130_node` ↔ `12:225-236/249/390`（`gs130_camera_node`、`gs130_calibration_node`、`gs130_fake_frames`）、实现 `setup.py:21` + `camera_node.py:83`（`camera_node`/`gs130_camera`） | **11** | 实现改 entry point 与节点名（最小 2 处），或走评审同步改 `11:67-73`+`21`+`10`+`02`；`12:225-236` 必须整段作废 |
| D-15 | `device:=GS130W` 时的 `/imu/data` | `11:155/403/567`、`21:364`（AC-FUN-11）、`21:439`（AC-RB-11）↔ 实现 `camera_node.py:150`（无条件创建 publisher） | **11** | 实现按 `imu_name` 条件创建 T4 并补 WARN；`11:403` 的"不创建 T4"因此成为可测判据 |
| D-16 | 时间戳降级与 `stamp_offset_*` | `11:312-342`（§4.2/§4.3）、`11:376-377`（§5.1）、`21:294`、`21:418`（AC-IF-06）、`11:672`（A11）↔ 实现 `camera_node.py:140/323-357`（只有一次性 offset） | **11** | 实现补齐参数与降级分支，或把 `11` §4.3 整表 + `11:376-377` + A11 明确标注"v0.2.0 待实现"并同步 `21:418` |
| D-17 | `mode:=raw` 与拼接的组合行为 | `21:416`（AC-IF-04 要求"必须与文档声明一致"）、`21:598`（K-14）↔ README:100（只写尺寸约束）、实现允许 `raw` + 默认 `stereo_layout=left_right`、`22:350` 断言 `width` 应为 `2176` | **11 + 21**（要求文档声明） | README §4 与 `11` §5 增补"`raw` 下 `stereo_layout != none` 的实测行为与退出码"；`22:350` 的 `2176` 断言需复核 |
| D-18 | 跨文档行号引用漂移 | `32:116` 引 `11_interface_freeze.md:438`、`32:256` 引 `11:254`（11 在 22:18 插入 20 行修订通知后全部 +20，现已指向无关内容） | **11** | `32:116`、`32:256` 等改写为小节号（§6.2、§3.4），全文档集禁用裸行号引用 |
| D-19 | `20_test_plan.md` 的期望值 | `20:42`（BEST_EFFORT）、`20:48`（`height=H*3/2`、`640x480 ⇒ 720 / 691200`）、`20:137`（TC-05 期望 `height=720 len=691200`）、`20:148`、`20:418` ↔ `00` E1/E2、`21:156-162` | **00** | `20:42/48/137/148/418` 按 `height=eh`、`len=width*height*3//2`、图像 `RELIABLE` 修正（21 `B-04` 早已登记，未执行） |
| D-20 | 测试报告与冻结书对 TF 的判定 | `25_test_report.md:16` 把"话题、速率、QoS、消息字段、时间戳、IMU、标定、**TF**"整体记 PASS ↔ `11:52`、`21:361/402` | **11** | `25_test_report.md:16` 的 TF 项改为"发布 TF（与 `11` §0.2 冲突，待接口评审裁决）"，不得记为契约 PASS |
| D-21 | 冻结书引用的"基线探针"与探针实际行为相反 | `11:204` 称"本地基线探针 `ros/probe_nv12_publisher.py` 都按此约定（`width x (h*3/2)`）" ↔ 该探针实际 `HEIGHT = 720`（`probe_nv12_publisher.py:17`）、`message.height = HEIGHT`（`:51`）、`message.step = WIDTH`（`:55`），即**真实高度**写法（E2 一侧） | **00 / E2** | 删 `11:204` 中"…基线探针…都按此约定"整句（保留的是一句与自身证据相反的话，最容易被后人当权威引用） |

### 2.2 用户点名的七项逐条结论

| 检查项 | 冻结书现状 | 实现/实测 | 结论 |
|---|---|---|---|
| 冻结书的图像 QoS | 正文 `BEST_EFFORT`（`11:152-154`），修订通知 `RELIABLE`（`11:10`） | 实现与实测均 `RELIABLE` | 实现合规；**正文三行必须改**（D-01） |
| 冻结书的 `Image.height` 约定 | 正文 `eh*3//2`（`11:195/220/646`），修订通知 + E2 为 `eh` | 实测 `height=480`（真实高），实现 `shape[0]*2//3` | 实现合规；**正文四处 + §3.1.2 必须改**（D-02，见 §4） |
| 冻结书是否要求手工拼接两目 | 要求（`11:208-212` 的 `concatenate` 示例），并禁止 `stereo_layout`（`11:227`） | E6 硬件拼接可用；实现用硬件拼接，无拼接代码 | 实现合规；**该示例与禁令必须删除/反转**（D-03、D-04） |
| 冻结书的参数表与默认值 | 24 个节点参数（`11:360-383`） | 实现 13 个（10 个一致、14 个缺失、3 个未评审新增） | 实现**不合规**（C67、C68）；方向须由接口评审裁定 |
| 冻结书的 launch 文件名 | `gs130_web.launch.py` / `gs130_camera.launch.py`（`11:439-440`） | 实现一致 | 合规；但**launch 参数与既有节点参数不合规**（C78-C80、C83） |
| 冻结书的 TF 决策 | 不发布 `/tf`、`/tf_static`（`11:52/113/427`） | 实现默认发布 `/tf_static`，实测确有 `/tf_static` | 实现**不合规**（C68 + 实测）；须裁决（D-05） |
| 冻结书的话题名 | `11:152-159` 八个话题 | 实现与 11 一致（除 T8 未实现） | 合规（缺 T8）；**其余文档的旧话题名仍与之冲突**（D-09） |

---

## 3. 发布门禁判定（`21_acceptance_criteria.md`）

判定只用：本次给定的板端实测结果 + 只读代码可确定的事实。**无法确定的一律记 UNVERIFIED，不记 PASS。**

### 3.1 G0 构建、版本、可复现

| AC | 级 | 判定 | 依据 / 待补证据 |
|---|---|---|---|
| `AC-BLD-01` 构建成功、包可发现 | P0 | UNVERIFIED | 需 `colcon build --packages-select gs130_ros` 的退出码与日志 |
| `AC-BLD-02` `ros2 pkg executables` 恰为 `gs130_ros: gs130_node` | P0 | **FAIL** | `setup.py:21` 装出的是 `camera_node`（C03） |
| `AC-BLD-03` 干净 clone 复现 | P0 | UNVERIFIED | 需在干净 clone 上按 README 走通冒烟子集 |
| `AC-BLD-04` 候选提交固定、工作区干净 | P0 | **FAIL** | `git status --porcelain` = ` M .gitignore` + `?? ros/`（`ros/` 未提交；`python/`/`core/` 干净 ✓） |
| `AC-BLD-05` 两个 launch 的参数集合正确 | P0 | **FAIL** | 13 + 4 ≠ 18 + 7（C79、C80） |
| `AC-BLD-06` 主机侧导入检查 `import gs130_ros.gs130_node` | P0 | **FAIL** | 模块名为 `gs130_ros.camera_node`（C06） |
| `AC-BLD-07` 包版本/许可证字段 + entry point | P0 | **FAIL** | `name`/`version`/`license`/`build_type` 正确；entry point 为 `camera_node`（C06） |

### 3.2 G1 功能

| AC | 级 | 判定 | 依据 / 待补证据 |
|---|---|---|---|
| `AC-FUN-01` 一条命令拉起相机+codec+web | P0 | **FAIL** | 链路本身由实测支持（HTTP 200 + 实时画面），但判据含 `ros2 node list` 必须出现 `gs130_node`，实际为 `gs130_camera`（C02） |
| `AC-FUN-02` T1/T2/T3 字段逐一正确 | P0 | **FAIL** | T1 全部字段由实测支持（`nv12`/1280/480/1280/921600/`frame_id=camera` ✓）；T2/T3 无法与 T1 并存（C21），`publish_per_eye` 不存在 |
| `AC-FUN-03` T1 硬件拼接 + 遮挡法左右目位置 + T2/T3 同帧对 | P0 | **FAIL** | ① 硬件拼接 ✓（`camera_node.py:196`，实测帧形状与几何一致）；② 遮挡法**未实测**；③ `publish_per_eye:=true` 不存在 → 子项失败 |
| `AC-FUN-04` T2/T3 可解码（`(480,640,3)`） | P0 | UNVERIFIED | 需在 `stereo_layout:=none` 下抓帧并 `cv2.COLOR_YUV2BGR_NV12` 解码、`std>5`、写 PNG |
| `AC-FUN-05` IMU 数值物理合理 + 协方差约定 | P0 | **FAIL** | ⑤ 明文"不得为 0"，实现 `angular_velocity_covariance[0]`/`linear_acceleration_covariance[0]` 均为 `0.0`（C50/C51）；①–④ 需静止 200 包统计 |
| `AC-FUN-06` IMU 与 SDK 直读逐轴一致 | P0 | UNVERIFIED | 实现是原样透传（`camera_node.py:308-313`），需节点侧落盘 JSON 与 `test_gs130.py` 串行比对 |
| `AC-FUN-07` CameraInfo 逐字段正确 + latch | P0 | **FAIL** | ②③⑤⑦⑧ 由实测支持（latched、640x480、`equidistant`+D 长度 4）；⑥ 要求 `R` 全 0、`P` 全 0，实现为单位阵 + `P` 填充（C34/C35）。④"K 与 SDK 逐元素相等"需 SDK 直读比对 |
| `AC-FUN-08` 不发布 TF + 文档给出替代做法 | P0 | **FAIL** | 实测存在 `/tf_static`，默认 `publish_tf=true`（C68） |
| `AC-FUN-09` 网页实时正确画面 + codec 不崩溃 + 实收 | P0 | UNVERIFIED | ① `curl` HTTP 200 ✓实测；⑤ 实时拼接画面 ✓实测；⑥⑦ codec 存活并收到数据（由画面推出）✓；② T7 速率、③ JPEG 解码尺寸 `(480,1280,3)`、④ `ros2 node info` 订阅关系**未实测** |
| `AC-FUN-10` 参数默认值 + 统计日志 + 节点名 | P0 | **FAIL** | 节点名（C02）；参数集合与 §2.4 不符（C67/C68）；统计日志文本不符（C99） |
| `AC-FUN-11` `device:=GS130W` 优雅降级 | P0 | **FAIL** | `/imu/data` 恒被创建，无 `IMU not present; …` WARN（C29） |
| `AC-FUN-12` `/gs130/status` | P1 | **FAIL** | 未实现（C38） |

### 3.3 G2 定量

| AC | 级 | 判定 | 依据 / 待补证据 |
|---|---|---|---|
| `AC-PERF-01` T1 ∈ `[29,31] Hz` | P0 | **PASS** | 实测 29.997 Hz（窗口需存档） |
| `AC-PERF-02` T1/T7 均达标 + per-eye 一致 + codec 实收 | P0 | UNVERIFIED | T1 ✓实测；T7 速率未实测；② per-eye 无法开启 → 该子项 **FAIL** |
| `AC-PERF-03` IMU ∈ `[190,210] Hz`、不丢包 | P0 | UNVERIFIED | 实测"about 203 Hz"支持 ①；②③（累计包数、1 s 窗口占比）需 `imu_windows.py` 采集 ≥60 s |
| `AC-PERF-04` 首帧 ≤20 s、网页 ≤30 s | P0 | UNVERIFIED | 需计时证据；实现侧 `start_timeout_s=10.0` 只是超限即退出（`camera_node.py:350`） |
| `AC-PERF-05` 端到端延迟 T1 ≤150 ms / IMU ≤50 ms | P0 | UNVERIFIED | 需 `ros2 topic delay` 原始输出 |
| `AC-PERF-06` 时间戳单调、同域、相邻差 ≈1/fps | P0 | UNVERIFIED | 设计上同域（一次性 offset，`camera_node.py:323-357`）；需 100 帧统计 |
| `AC-PERF-07` SIGINT ≤3 s 退出码 0、无残留、相机可复开 | P0 | UNVERIFIED | 实测"`camera released` + 无残留进程"支持 ①③；④"≤10 s 复开"、⑤⑥ 基线比对（nginx 孤儿）未实测 |
| `AC-RES-01`/`AC-RES-02`/`AC-RES-03`/`AC-RES-04` | P0×3 + P1 | UNVERIFIED | 需 RSS/fd/线程/CPU 采样与 60 min soak、无订阅者场景 |

### 3.4 G3 代码质量

| AC | 级 | 判定 | 依据 |
|---|---|---|---|
| `AC-SRC-01` docstring 齐备 | P0 | **FAIL** | `camera_node.py:422 main()`、`:353 to_ros_ns()`、`:356 stamp_of()`、两个 launch 的 `generate_launch_description()` 均无 docstring |
| `AC-SRC-02` 体积限内（单文件 ≤250 行） | P0 | **FAIL** | `camera_node.py` 440 行；`calibration.py` 114；单函数 ≤40 ✓；launch ≤150 ✓ |
| `AC-SRC-03` 无未使用 import/参数；launch 参数全被引用 | P0 | **PASS** | 逐个核对 `camera_node.py:8-20` 与 `calibration.py:6-9` 均被使用；launch 每个 `DeclareLaunchArgument` 都进入 `parameters` |
| `AC-SRC-04` 无死代码/裸 except/静默吞异常 | P0 | **PASS**（附注） | `except Exception` 均带日志（`camera_node.py:381-386`）；附注：`ros/test/capture_frame.py:76` 有恒真断言 `height*3//2 == height*3//2`（不属本包 `ros/gs130_ros/`，但应清理） |
| `AC-SRC-05` 未自研 web/codec/depth | P0 | **PASS** | `ros/gs130_ros/` 内无 `imencode`/HTTP/socket/`stereonet`/`PointCloud2`/V4L2 命中 |
| `AC-SRC-06` 既有节点参数逐字、无补丁 | P0 | **FAIL** | `gs130_web.launch.py:48-73` 用 include + 厂商 launch 参数名，非 §2.6 逐字；`websocket_only_show_image="True"` 与 `only_show_image=False` 相反（C78） |
| `AC-SRC-07` 无每帧日志 + 统计日志逐字 + `log_fps_period_s` | P0 | **FAIL** | 无每帧日志 ✓；统计日志文本与周期参数均不符（C99） |
| `AC-SRC-08` 错误文本与退出码逐字 | P0 | **FAIL** | 见 C84–C98、C105 |
| `AC-SRC-09` 未 monkeypatch SDK | P0 | **PASS** | 无对 `gs130` 的赋值/`setattr`；`git status python/ core/` 为空 |
| `AC-SRC-10` 未实现非目标消息/参数（无 TF/温度/自定义 msg） | P0 | **FAIL** | `tf2_ros`（`camera_node.py:18`）、`TransformStamped`（`calibration.py:6`）、`/tf_static` 实际发布 |
| `AC-SRC-11` 参数集合 == 24、无非目标参数 | P0 | **FAIL** | 13 个参数，含 `stereo_layout`/`publish_tf`（C67/C68）；无 `respawn` ✓、无 `use_sim_time:=true` ✓ |
| `AC-SRC-12` 纯函数层主机侧单测 | P1 | UNVERIFIED | `ros/test/test_calibration.py` 已存在（9 个 unittest），但 AC 的路径是 `ros/gs130_ros/test`（不存在），且需板端 rclpy/gs130 环境执行 |
| `AC-SRC-13` `data` 为整块缓冲赋值 | P1 | **PASS** | `camera_node.py:401-402` `array.array("B")` + `frombytes`（E3 快路径） |

### 3.5 G4 接口契约

| AC | 级 | 判定 | 依据 |
|---|---|---|---|
| `AC-IF-01` §11 阻塞项全关 + 四份文档与冻结书逐字一致 | P0 | **FAIL** | `B-02`/`B-03`/`B-04`/`B-05`/`B-06`/`B-08`/`B-09` 在文本层面均未闭合（见 §2）；`01/02/03/12` 未改写 |
| `AC-IF-02` 话题名/类型/QoS/`frame_id` 逐字 | P0 | **FAIL** | 图像 `RELIABLE`+depth 1 ✓；`/imu/data` reliability 与 §2.2 不符（C25）；T5/T6 `frame_id` ✓；T2/T3/T8 默认不出现 ✓；⑤ codec 实收 ✓ |
| `AC-IF-03` 节点/launch 参数名、默认值、类型一致 | P0 | **FAIL** | C67–C71、C79、C80 |
| `AC-IF-04` 每个参数都有可观测效应 | P0 | **FAIL** | 参数矩阵中的 `publish_combine`/`publish_per_eye`/`publish_camera_info`/`publish_status`/`camera_info_distortion_model`/`log_fps_period_s` 不存在；`mode:=raw` 的 T1 行为未文档化（D-17） |
| `AC-IF-05` 消息逐字段冻结项 | P0 | **FAIL** | `height=eh` ✓；三个协方差 `[0] = -1.0` ✗（C50/C51）；CameraInfo `R`/`P` ✗（C34/C35） |
| `AC-IF-06` 时间戳公式与降级规则 | P0 | **FAIL** | 一次性 offset ✓；`stamp_offset_ns`/`stamp_offset_mode` 不存在 → ③"两种模式可区分"无法执行；①②④ 需实测 |
| `AC-IF-07` launch 节点集合与既有节点参数 | P0 | **FAIL** | C78；`gs130_camera.launch.py` 只起本包节点 ✓ |

### 3.6 G5 鲁棒性

| AC | 级 | 判定 | 依据 / 待补证据 |
|---|---|---|---|
| `AC-RB-01` 相机被占用 → 退出码 1 + 点名错误 | P0 | **FAIL** | 实现退出码 2、文本不同（C105） |
| `AC-RB-02` 缺库 → 明确报错、非 0、非段错误 | P0 | **FAIL** | 非法退出 ✓（模块级 `ImportError` → `SystemExit(2)`，`camera_node.py:22-30`）；但文本既不提 `GS130_LIB`，`OSError`（`libgs130.so` 缺失）路径只在 `Device()` 构造处兜底（`camera_node.py:206-207`），导入期 `OSError` 未被捕获 |
| `AC-RB-03` 非法参数退出码与逐字文本 | P0 | **FAIL** | `width:=641`、`fps:=99` 通过校验（C69/C70）；文本非 `invalid parameter …`；`platform:=X` 未捕获 `ValueError`；`odr:=100` 实测路径与文档声明的退出码不一致（文档 1 vs 实现 2） |
| `AC-RB-04` `raw` + 错误尺寸 → 文档声明的退出码 + 逐字提示 | P0 | **FAIL** | 行为与退出码 2 由实测支持；文本为 `mode raw requires … got 640x480`（非逐字），README 未声明"校验阶段 → 退出码 2" |
| `AC-RB-05` 20 轮启停 | P0 | UNVERIFIED | 需 20 轮脚本记录 |
| `AC-RB-06` 启动中途 SIGINT + FSYNC 阻塞告警 | P0 | **FAIL** | 判据②要求的逐字日志全部缺失：`start()` 之前无 `starting the camera; waiting for the IMU FSYNC handshake…` 的 `INFO`，无每 `start_timeout_s` 一条的 `WARN`，无 `shutdown requested while gs130_start is blocking; waiting for the handshake`（C103）。①"≤3 s 退出码 0 + 无残留 + 可复开"需实测（实现侧 `camera_node.py:115-121` 已保证 `SIGINT` 会走 `stop()+close()`） |
| `AC-RB-07` SIGKILL 后按文档恢复 | P1 | **FAIL** | README 未给出 SIGKILL 后的恢复步骤（§7 只有"不要 `kill -9`"） |
| `AC-RB-08` 运行期相机通路致命 / IMU 通路非致命 | P0 | **FAIL** | 相机通路失败只记 `read_image failed` 并继续（C91）；IMU 文本不符且未停发（C93） |
| `AC-RB-09` 标定缺失 / `rect` 无标定 | P0 | **FAIL** | 无 `calibration not available;` 分支、无 `publish_camera_info`；`camera_intrinsics()` 无条件调用（C95） |
| `AC-RB-10` 无订阅者 60 s | P0 | UNVERIFIED | 同 `AC-RES-04` |
| `AC-RB-11` `device:=GS130W` | P0 | **FAIL** | 同 `AC-FUN-11`（C29） |
| `AC-RB-12` 零/回退时间戳注入 | P0 | **FAIL** | 未实现（C63） |
| `AC-RB-13` `use_sim_time:=true` → WARN 且继续 | P0 | **FAIL** | 无检测（C64）；`use_sim_time=true` 且无 `/clock` 时 `get_clock().now()` 返回 0，`offset_ns` 会被算成 `-device_ts`，`header.stamp` 落到 1970 年域且无任何告警 |
| `AC-RB-14` 图像字段错误场景下 codec 段错误为硬失败、实现不得产生 | P0 | **PASS** | 实现发布 `height = eh`（`camera_node.py:409`），实测 codec 未崩溃 |
| `AC-RB-15` 关停后无残留进程、端口回基线 | P0 | **FAIL** | `gs130_web.launch.py` include 的 `websocket.launch.py` 用 `os.system` 起 nginx，`00:74-75`（E4）实测其"launch 退出后仍存活"；本包未做任何清理 → 占用集合与基线不一致（README:148 只给人工 `pkill` 步骤） |

### 3.7 G6 文档

| AC | 级 | 判定 | 依据 |
|---|---|---|---|
| `AC-DOC-01` 用户文档 9 章齐备 | P0 | **FAIL** | README 有安装/启动/参数/话题/标定/时间戳/排查/边界，但话题表**缺 QoS 四项**、**缺 channel ↔ 话题映射表**、故障排查只有 **6 条**（要求 ≥9）、未收录 §6.6 的逐字命令 |
| `AC-DOC-02` 第三方 30 min 内看到画面、零提问 | P0 | UNVERIFIED | 需未参与编码者的实测记录 |
| `AC-DOC-03` 文档命令逐字可执行（≥10 条） | P0 | UNVERIFIED | README 的命令与实现自洽，但需逐条执行留证 |
| `AC-DOC-04` 文档参数表/话题表与冻结书逐字一致 | P0 | **FAIL** | README:96-109 的参数表是实现的 13 项，与 `11` §5.1 的 24 项不一致；话题表缺 `mode=raw` 组合行为 |
| `AC-DOC-05` 故障排查 ≥9 条 | P0 | **FAIL** | README:144-151 共 6 条 |
| `AC-DOC-06` 必写契约说明齐备 | P0 | **FAIL** | 已写：独占要点、时间戳（含 ≤ 一帧周期不确定性）、IMU 含重力；未写：channel↔话题映射表、`temp`/`is_fsync` 丢弃项、`D` 的截断规则（§5 反而写"针孔 → 8 个系数"，与冻结书"只发 5 个"相反）、"NV12 不能直接用于 `rqt_image_view`"、IMU 订阅方需匹配 QoS（`K-10`） |
| `AC-DOC-07` 中文说明 + 英文标识符 | P1 | **PASS** | README 全篇符合 |

### 3.8 门禁结论（对照 §B.2）

| §B.2 放行判据 | 结果 |
|---|---|
| 1. §1 中全部 **P0** 项 PASS | **否**（P0 共 66 项：**6 PASS / 42 FAIL / 18 UNVERIFIED**） |
| 2. §11 的 `B-01`…`B-11` 全部关闭 | **否**（文本层面至少 `B-02`、`B-03`、`B-04`、`B-05`、`B-06`、`B-08`、`B-09` 未闭合） |
| 3. `AC-RB-05`（20 轮）与 `AC-PERF-07` 通过 | **否**（均 UNVERIFIED，且 `AC-RB-15` 相关项 FAIL） |
| 4. `AC-FUN-03` 与 `AC-FUN-09`③ 通过 | **否**（`AC-FUN-03`③ FAIL；`AC-FUN-09`③ 未实测） |
| 5. P1 项 FAIL 已在"已知限制"登记 | **否**（`AC-FUN-12` 未实现且未登记；`AC-RB-07` 缺恢复步骤） |
| 6. §13 的 `K-1`…`K-9` 已按处置写入已知限制 | **否**（`K-5`/`K-10`/`K-11`/`K-14` 的文档要求未落地） |

**判定：`gs130_ros` v0.1.0 不通过发布门禁。** 全量 72 项 AC：**8 PASS / 44 FAIL / 20 UNVERIFIED**（P0 66 项：6 PASS / 42 FAIL / 18 UNVERIFIED；P1 6 项：2 PASS / 2 FAIL / 2 UNVERIFIED）。
必须说明的是：其中相当一部分 FAIL 的直接原因不是行为错误，而是**行为与冻结书的字面约定不同**（C02/C03/C67/C68/C78/C79/C80、C34/C35、C50/C51、C84–C98）。
因此存在两条互斥的收敛路径：**(a)** 实现向冻结书对齐（补齐参数/错误文本/退出码，去掉 TF 或改为默认关闭）；**(b)** 走接口评审承认已交付接口并同步改写 `11`+`21`+`20`+其它文档。**在任一路径完成之前，门禁不能通过，也不应通过。**

---

## 4. 最危险的剩余分歧与最小修复

### 4.1 最危险项：冻结书正文仍冻结"打包高"，而打包高会让既有节点段错误

- 文档一侧的现状：`11_interface_freeze.md` 的**正文**在两处表格里冻结 `height = eh * 3 // 2`（`:195` 单目、`:220` 拼接），第 `:202` 行的小节标题是"为何 `height = eh * 3 // 2`（说明，**不改变上表**）"，第 `:204` 行还用一整段论证"因此**不得**输出 `height = eh`、`step = ew`…"，第 `:646` 行把它写进"冻结声明"第 4 条，附录 `:664` 的 A3 也按打包高自洽。
- 现实一侧：`00` E2（`:25-44`）实测该写法让既有 `hobot_codec_republish` 打印 `init_pic_h_: 1080, alined_pic_h_: 1088` 后**段错误（退出码 -11）**；实测的现实是 `height=480`（真实高）、`step=1280`、`len=921600`；实现也已按修订通知写成 `shape[0]*2//3`（`camera_node.py:409`）。
- 为什么它是"最危险"而不是其它分歧：
  1. 它是**唯一一处"照冻结书实现就会让别人的节点崩溃"**的条款（其余分歧最多是收不到数据、字段语义不同或门禁判据不匹配）；
  2. 它现在被文档**自身**分成了两半——第 `:10-21` 行的修订通知说改为 `H`，正文说 `eh*3//2` 且"不改变上表"。后来的读者若从 §3.1 入口读，会直接读到"不得输出 `height = eh`"，正好与交付实现相反；
  3. 该节还引用 `ros/probe_nv12_publisher.py` 作为"打包高约定"的证据，而该探针自己就写着 `message.height = HEIGHT`（真实高，`probe_nv12_publisher.py:17/51/55`）——即冻结书**引用了一份与自身结论相反的证据**（D-21），这会进一步让人相信正文。

### 4.2 最小修复（改 1 个文件、5 处逐字替换 + 1 节删除）

在 `ros/docs/11_interface_freeze.md` 内：

| 位置 | 现在 | 改为 |
|---|---|---|
| `:195`（单目表） | `height` 行的值单元格：`eh * 3 // 2` | `height` 行的值单元格：`eh`（真实图像高，E2 实测） |
| `:220`（拼接表） | `height` 行的值单元格：`eh * 3 // 2` | `height` 行的值单元格：`eh` |
| `:202`+`:204` | 标题"为何 `height = eh * 3 // 2`（说明，不改变上表）"及其论证段 | **删除该小节**（或改写为"为何 `height = eh`：`eh*3//2` 会让 `hobot_codec_republish` 段错误，见 E2"），并**删除** `:204` 中"因此**不得**输出 `height = eh`…"整句 |
| `:646` | `height = eh*3//2` | `height = eh` |
| `:664`（A3） | `/image_combine_raw` 的 `width x (height/1.5)` | `/image_combine_raw` 的 `width x height` = `2*width_eye x height_eye` |

同一变更单里同步 `20_test_plan.md:48/137/148/418`（期望值 `720/691200` → `480/921600`），即 21 `B-03`/`B-04` 早已登记的关闭动作。

### 4.3 并列的第二、第三危险项（不改变上面的结论，但需要在同一评审会上裁决）

1. **TF**（D-05）：冻结书 `§0.2`/`§5.5` 明文禁止，现实与实现都发布 `/tf_static`；读者若照冻结书自建静态 TF，会出现同一父/子边的**双发布者**，下游 tf2 会拿到抖动/冲突的几何。最小处置二选一：实现把 `camera_node.py:95` 的 `publish_tf` 默认改为 `False`（1 行 + 两个 launch 默认值），或在冻结书新增 T9 并同步 `21` 的 `AC-FUN-08`/`AC-SRC-10`/`NA-03`。
2. **`R`/`P`**（D-12）：冻结书要求全 0，现实是单位阵 + 填充。最小处置：在 `00` 增补一条实测决策行，然后改 `11:273/274/648/670` 与 `21:240/241`、`20:49`。

---

## 5. 建议显式作废（RETIRE）的规范条目

以下条目在文档集中与实现/实测冲突，且**不应通过"改实现"来满足**（它们是早期提案、已被实测或接口裁决推翻）。建议在每条原位置标注"已作废（v0.1.0，见 24 §5）"而不是删除，以免后人重新引用：

| # | 作废项 | 位置 | 理由 |
|---|---|---|---|
| R-01 | 图像三话题 `BEST_EFFORT` | `11:152-154` | E1 实测：codec 端为 RELIABLE，BEST_EFFORT 静默丢弃（修订通知已改口） |
| R-02 | `hobot_codec_republish` 以 `BEST_EFFORT` 订阅 / "QoS 不匹配（订阅者 RELIABLE）" | `11:423`、`11:570` | 方向写反 |
| R-03 | `height = eh*3//2` 及其"不得输出 `height = eh`"论证、A3 | `11:195/202-204/220/646/664` | E2 实测段错误（见 §4） |
| R-04 | 驱动侧手工 `concatenate` 拼接与"不使用 SDK `stereo_layout`" | `11:208-212/227`、`11:419`、`22:294-300` | E6 实测硬件拼接可用且更便宜 |
| R-05 | `R`/`P` 全 0 | `11:273-274/648/670`、`21:240-241`、`20:49` | 实测现实为单位阵 + 填充；`32:256` 指出 stereonet 需要非零 `P`（须先在 `00` 增补实测决策） |
| R-06 | `codec_channel`/`web_port`/`web_output_fps`/`smart_topic`/`hobot_shm`/`input_framerate=-1`/`output_framerate=-1`/`image_type="mjpeg"`/`only_show_image=False` 等 launch-only 参数与既有节点参数表 | `11:384-390/453-473/506`、`21:277/302-303` | 实现走厂商 launch include，既不声明这些参数也不逐字传值；若确定采用 include 路线，本表须整体替换为映射表（否则 `B-08` 永远无法关闭） |
| R-07 | `publish_per_eye` 与"T2/T3 默认关闭、可与 T1 并存" | `11:372`、`11:479`、`11:678`（A17）、`21:264/356`、`32:591` | SDK 二选一（`B-10`）；实现以 `stereo_layout:=none` 表达同一能力 |
| R-08 | `/gs130/status`（T8）及其字段清单 | `11:159/375/603-610`、`21:365`、`AC-FUN-12` | 未实现且为可选；`11:170` 自己已注明"可降级为彻底删除" |
| R-09 | `camera_info_distortion_model` 覆盖参数与 `none` 分支、PINHOLE→`plumb_bob`+5 元 | `11:378/286`、`21:239/270` | 实现采用 `rational_polynomial`+8 元与"全零即 plumb_bob"三分支；须由评审统一 |
| R-10 | `stamp_offset_ns`/`stamp_offset_mode` 与 §4.3 的六条降级规则 | `11:338/376-377`、`21:294/418`、`11:672(A11)` | 未实现；若 v0.1.0 确定不做，应标 v0.2.0 |
| R-11 | `poll_period_ms`/`imu_qos_depth`/`image_qos_depth`/`log_fps_period_s`/`publish_combine`/`publish_camera_info`/`publish_status`/`frame_id_combine`/`frame_id_left`/`frame_id_right` | `11:367-382` | 未实现的 14 项中的其余部分；v0.1.0 的实际契约是"`stereo_layout` + `frame_id_camera` + `publish_tf`" |
| R-12 | §6.6 中依赖 `gs130_node` 可执行名与 `publish_per_eye`/`publish_status` 的逐字命令 | `11:528-529/534-535` | 与实现的可执行名/参数集合不符 |
| R-13 | `/tf` 非目标条目与 `tf2_msgs` 不声明 | `11:52/113/427`、`21:361/402/469` | 见 D-05（须裁决，不能默认保留） |
| R-14 | `01` 的 `AC-01`（`raw 1088x1280`/`odr=100`）、`AC-02`（`/image_jpeg`）、`AC-03`（`/gs130/image_left_raw`）、`AC-04`（`/gs130/imu`）、`AC-05`（`/gs130/left/camera_info`）、`AC-06`（TF 可用）、`AC-11`（`gs130_probe`） | `01:75-87/117-123` | 与冻结书逐项冲突（21 `B-06` 已登记） |
| R-15 | `gs130_probe` 诊断命令、`/gs130/stereo/image_raw`、`/<ns>/camera_left/image_raw`、`/gs130/imu/data` 旧命名、`odr_hz`、`startup_timeout_s`、`require_imu` | `10:125/537-547`、`12:139-142/225-236`、`30:124-130`、`31:257-262`、`03:64-70/105/154/169` | 实现不存在这些名字 |
| R-16 | 三 launch / 别名 launch 方案（`gs130_stereo_websocket.launch.py`、`gs130_imu.launch.py`、`gs130_camera_info_only.launch.py`、`gs130.launch.py`、`gs130_websocket.launch.py`） | `02:325-327`、`03:6/70`、`01:118`、`12:276/288` | 冻结书只有两个 launch（21 `B-05`/`B-06`） |
| R-17 | 图像 QoS depth ≥5 / `KEEP_LAST(10)` | `32:119/505/681`、`02:401` | 冻结值与实现均为 depth 1 |
| R-18 | `20_test_plan.md` 的打包高期望值与 `BEST_EFFORT` 期望 | `20:42/48/137/148/418` | 已被 E1/E2 推翻（21 `B-04`） |
| R-19 | 冻结书引用的"探针按打包高写法" | `11:204` 整句 | 探针实际是真实高写法（D-21） |
| R-20 | 裸行号交叉引用 | `32:116/256` 等 | 11 插入修订通知后全部漂移（D-18） |

---

## 附录 A：UNVERIFIED 项各自需要的证据（一次性清单）

| UNVERIFIED 项 | 需要的证据 |
|---|---|
| `AC-BLD-01`/`AC-BLD-03`/`AC-BLD-06` | `colcon build` 日志（`rc=$?`）、干净 clone 上的冒烟子集、`python3 -c "import gs130_ros.<模块>"` |
| `AC-FUN-04`/`AC-FUN-06` | `stereo_layout:=none` 下的 PNG 落盘与解码统计；节点 JSON 与 `python/test/test_gs130.py` 的串行比对 |
| `AC-FUN-03`② | 遮挡左/右目各 5 s 的视频/NV12 抓帧 + 左右半幅灰度均值（`21` 附录 A.10） |
| `AC-FUN-09`②③④ | `ros2 topic hz /image_combine_jpeg`（窗口 ≥200）、`cv2.imdecode` 断言 `(480,1280,3)`、`ros2 node info websocket` |
| `AC-PERF-02`②③/`AC-PERF-03`②③ | 四个话题的同窗 `topic hz`；`imu_windows.py` ≥60 s 的 1 s 窗口统计 |
| `AC-PERF-04`/`AC-PERF-05` | 启动计时脚本、`ros2 topic delay` 原始输出 |
| `AC-PERF-06`/`AC-IF-06`①④ | 100 帧 `stamp` 序列 + SDK `timestamp_ns` 直读比对（需先实现 `stamp_offset_ns`） |
| `AC-PERF-07`④⑤⑥/`AC-RB-15` | 启动前基线（`pgrep -af` 匹配 `nginx`/`hobot_codec`/`websocket`、`ss -ltn` 过滤 `:8000`）与关停后逐行 diff，连续 3 轮 |
| `AC-RES-01`…`AC-RES-04`/`AC-RB-05`/`AC-RB-10` | `/proc/<pid>/status` 的 RSS/Threads、`fd` 计数、`top -b -d 1 -n 60` 采样、20 轮启停脚本、无订阅者 60 s 复测 |
| `AC-SRC-12` | `ros/gs130_ros/test`（或 `ros/test`）在板端 `pytest`/`unittest` 的完整输出 |
| `AC-DOC-02`/`AC-DOC-03` | 未参与编码者的 30 min 计时记录与 ≥10 条命令的 `rc=$?` 存档 |

## 附录 B：本报告的判定口径声明

1. 本报告**不修改**任何被评审文件，所有修复建议均以"位置 + 现在的内容 + 应改为"的形式给出，交由实现方或接口评审执行。
2. `MATCHES` 指**逐字/逐值**一致；"语义等价但文本不同"按 `DIVERGES` 计（冻结书 §0.2 与 21 `AC-IF-01` 都要求"不接受等价但不同"）。
3. 凡冻结书正文与其自身修订通知冲突之处，按修订通知（= 实测）判定，正文文本记为文档缺陷（D-xx），**不重复计入**合规矩阵的不符合行数。
4. 本报告的快照时间与哈希见 §0；若评审后 `ros/` 再次变更，本报告的合规矩阵与门禁判定需按新快照重跑。
