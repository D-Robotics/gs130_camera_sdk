# 23 代码评审报告（REV-1，对抗式正确性评审）

评审对象：`ros/`（v0.1.0 交付包）。**评审期间作者在持续改动**，本报告的行号锚定于 2026-09-14 22:36（CST）的快照；每条缺陷都同时给出**函数名/标识符**，即使行号漂移也能定位。评审基线（本次评审实际读取的修订）：

| 文件 | sha256（前 16 位） |
|---|---|
| `ros/gs130_ros/gs130_ros/camera_node.py` | `e2eabed42359a57c` |
| `ros/gs130_ros/gs130_ros/calibration.py` | `fa8ab3deb95d8793` |
| `ros/gs130_ros/gs130_ros/launch_arguments.py` | `0daf333bda384969` |
| `ros/gs130_ros/launch/gs130_camera.launch.py` | `983ab6e6ab38c196` |
| `ros/gs130_ros/launch/gs130_web.launch.py` | `12a8e8b3d5001908` |
| `ros/README.md` | `8066caaf46d2c21d` |

同时阅读了被判定的底层实现：`python/gs130/_device.py`、`_config.py`、`_types.py`、`_enums.py`、
`_abi.py`、`_error.py`、`_runtime.py`、`core/include/gs130.h`，以及
`core/src/gs130.cpp`、`core/src/devices/pipeline/rdkx5/rdkx5.cpp`（判据需要，仅阅读未运行相机）。
平台事实以 `ros/docs/00_verified_platform_facts.md`（E1–E7）为准。

判据来源：`ros/docs/11_interface_freeze.md`（接口冻结契约）、`21_acceptance_criteria.md`、
`00_verified_platform_facts.md`。凡“契约/README 说了但代码做不到”的，按缺陷处理；
凡仅属风格、命名、排版的不报。

---

## 1. 缺陷清单（按严重度排序）

严重度定义：**Blocker** = 崩溃/存越界/必然产生错误数据或必然违反冻结契约；
**Critical** = 特定但常见的输入组合下产生错误数据或错误退出码，且无任何告警；
**Major** = 明确的可复现错误行为或契约偏差，影响面较窄；
**Minor** = 影响有限但确属缺陷；**Nit** = 死代码/文档措辞等文档级瑕疵。

| ID | 严重度 | 文件:行 | 触发场景（输入与状态） | 证据与推理 | 最小修复 |
|---|---|---|---|---|---|
| **B-01** | Blocker | `ros/gs130_ros/gs130_ros/camera_node.py:192-197`（`_create_device` 内 `gs130.Config.preset` 在 `try` 之外，`try` 在其后 200 行处） | `ros2 launch gs130_ros gs130_camera.launch.py device:=GS130X`（或 `platform:=RDKX5x`、`device:=gs130wi` 等任何非法拼写） | `Config.preset()` 对非法 platform/device **抛 `ValueError`**（`python/gs130/_config.py:66-69`）。该调用在 `try:` 之前，因此既不落入 `except gs130.GS130Error` 也不落入 `except OSError`，直接冒泡出 `Gs130Camera.__init__`：`main()` 只捕获 `KeyboardInterrupt` 与 `SystemExit`，未捕获的 `ValueError` 一路冒到 `__main__`，Python 打印原始 traceback 并以 **1** 退出（`finally` 里 `node is None`，所以连一句针对用户的提示日志都没有）。契约 §7.1 要求该情形逐字 `ERROR unsupported platform/device: {platform} {device}` 且**退出码 2**；附录 A 亦列此条。当前行为：用户看到 traceback，无 “参数错误” 语义，日志级别也不是 ERROR。 | 把 `Config.preset(...)` 与 `LOWER` 一起纳入 `try`：`try: config = gs130.Config.preset(...) except ValueError as e: self.fail("unsupported platform/device: %s %s" % (self.platform, self.device_name))`（`fail()` 已保证退出码 2）。 |
| **B-02** | Blocker | `camera_node.py:177-190`（`_validate`）、`:198-200`（`_create_device` 写入 `stereo_layout`） | `ros2 launch ... mode:=raw width:=1088 height:=1280 stereo_layout:=left_right`（默认 layout 即为 left_right，故 `mode:=raw` + 默认值就会命中） | `_validate` 只校验 raw 模式的宽高等于 1088x1280，**不校验 raw 与拼接互斥**。E6 明确：RAW 模式输出固定 1088x1280，与拼接宽度翻倍冲突，**RAW 不能拼接**。C++ 侧拼接分支（`core/src/gs130.cpp:249-268`）分配 `width*height*3` 并把 `frame[0].width = width*2`、`frame[0].height = height`（RAW 下即 2176x1280）。Python 侧 `Image.__new__`（`_types.py:23-27`）按 `raw.width*raw.height*3//2` 建视图 → `(raw.height*3//2, raw.width)` = `(2176*3//2=3264, 1280)`，而节点用 `height = shape[0]*2//3 = 2176`（= 实际`H*3//2` 型“打包高度”）、`width = 1280`（实际应为 2176）。消息**自洽但几何全错**：声明 1280x1088 的图，实为一张 1280x2176 的缓冲（`len(data)=1280*2176*3/2=4177920`），恰好等于声明长度，所以连“长度不符”这条防线都不会报警。下游 codec 收到 2176 高的“图像”，即 E2 实测会 **段错误（exit -11）** 的那类输入。 | 在 `_validate` 增加一条：`if self.mode_name == "raw" and LAYOUTS[self.layout_name] != gs130.StereoLayout.NONE: self.fail("mode raw cannot stitch; use mode:=resize with stereo_layout:=none")`。 |
| **B-03** | Blocker | `camera_node.py:432`（`image_message` 的 `height = shape[0]*2//3`）配 `_validate:186-190` | `... height:=479` 或 `height:=481`（README §4 写“正整数”，校验也只要求 `> 0`） | 该公式只在“打包行数 == 1.5×真实高”时成立。实算（numpy 复现）：`H=479` → 打包行 718 → 发布 `height=478`、`len(data)=459520`，而 `width*height*3//2=458880`，两者相差 640 字节；`H=481` → 打包行 721 → 发布 `height=480`、`len=461440` vs 460800。违反自己 README §3 表格（`data` 长度必须 `= width*height*3/2`）与冻结契约 §3.1。消费端按 `height*3//2` 行 reshape 会得到与发布端不同的形状，即“花屏/上下错位”。 | 校验阶段直接拒绝非偶数与超界维度：`if self.width % 2 or self.height % 2: self.fail(...)`，并保留现公式（偶数时公式恒等成立，已复算 480/960/1088 均 OK）。 |
| **B-04** | Blocker | `camera_node.py:58-64`（LAYOUTS 暴露 `top_bottom`/`bottom_top`）、`:279`（统一按“打包 NV12”发布）、`README.md:104` | `... stereo_layout:=top_bottom`（或 `bottom_top`），默认 640x480 | C++ 布局用**分平面**偏移（`gs130.cpp:262-277`）：`y[L]=buf; y[R]=buf+W*H; uv[L]=buf+2*W*H; uv[R]=uv[L]+W*H/2`，即前 1.5 个平面是左目的完整 NV12、后 1.5 个是右目的完整 NV12；而 Python 侧把它 reshape 成 `(raw.height*3//2, raw.width)`（480 时即 `(1440, 640)`）当作“单张打包 NV12”。按该解释，Y 平面 = 行 0..959，其中行 480..719 实际是左目 UV、行 720..959 是右目 Y，行 960..1439 才被当作 UV。结果：Y/UV 全部错位，codec 解出的图必然是彩色错乱的垃圾帧，而节点日志与 README 都不提示。 | 二选一，且不要引入新特性：① 在 `_validate` 拒绝 `top_bottom`/`bottom_top`（只保留 `none`/`left_right`/`right_left` 三个已验证布局），并在 README 注明原因；② 若必须支持，则按 `raw.height` 拆成“每目一张单目 Image”发布，而不是伪装成一张打包 NV12。 |
| **B-05** | Blocker | `camera_node.py:184-190`（`_validate` 缺 `fps` 上界、缺 `odr` 值域、缺偶数校验）；对照冻结契约 §5.1（`fps [1,33]`、`odr ∈ {200,500}`、`width/height 偶数且 [16,1088]/[16,1280]`） | `fps:=200`；或 `odr:=150`；或 `width:=641` | ① `fps` 上界不校验：契约注明“不得超过传感器读出率（约 33 fps），超过会造成帧丢失/异常”；`fps:=200` 时图像 tick 变 400 Hz，每 tick 最多 2 帧 0.92 MB 的 `memcpy`+`tobytes`+`frombytes`+DDS 序列化，而 `rclpy.spin(node)` 用的是 `SingleThreadedExecutor`，IMU 与 5 s 上报定时器会排在图像回调之后被周期性饥饿。② `odr:=150` 会被 SDK 拒（`GS130_UNSUPPORTED` → `GS130Error`），但 `_create_device` 的 `except gs130.GS130Error` 分支（`:202-209`）不分错误码一律 `fail(..., code=1)`，对“参数值非法”这一类给出的退出码是 1 而非契约的 2（把参数错误与硬件错误合并了）。③ 奇数宽高见 B-03。 | 在 `_validate` 补齐契约值域校验（`fps` 1..33、`odr ∈ {200,500}`、宽高偶数且范围内），非法组合在**启动校验阶段**就以退出码 2 拒绝；若确实想让 SDK 去拒绝 `odr`，则按 `error.code` 分流退出码。 |
| **C-01** | Critical | `camera_node.py:267-274`（`_poll_images` 的 `except`）与 `:327-340`（`_wait_for_first_frame` 无 `except`） | 运行期相机通路故障：`read_image()` 返回 `HW_ERROR`/`THREAD_CLOSED`（拔线、ISP/VIN 掉流、SDK 线程异常退出）；或启动等待首帧期间发生同样故障 | 契约 §7.2 把“相机数据通路失败”列为**致命**：必须 `ERROR camera stream failed: {CODE}` 后**退出码 1**，理由写得很清楚（SDK 要求 `deinit`+`init` 恢复，v0.1.0 不自动恢复，应让 supervisor 看见）。当前 `_poll_images` 只 `get_logger().error(...)` 然后 `return`，节点会**永远**以 60 Hz（`1/(2*30)`）继续空转并重复打日志，既没有恢复也没有退出；`_wait_for_first_frame` 里的 `read_image()` 同样可能抛 `GS130Error`，会冒泡出 `__init__`（`finally` 里 `node is None`，进程以未捕获异常退出，退出码 1 但没有任何 `camera stream failed` 语义日志与清理提示）。 | 两处统一：捕获后 `self.get_logger().error("camera stream failed: %s" % error)`，随后 `raise SystemExit(1)`（`main()` 已支持），并确保 `_wait_for_first_frame` 的调用被 `__init__` 的 `try/except BaseException: self.shutdown()` 覆盖（把 `self.device.start()` 之前的 `_wait_for_first_frame` 之前的调用一并纳入即可，或在该函数内自捕获后 `fail`/`SystemExit(1)`）。 |
| **C-02** | Critical | `camera_node.py:279`（`images["stitched"]` 用字面量 `"camera"`）对照 `:281`（单目用 `self.frame_camera`）、`:226`（CameraInfo 用 `self.frame_camera`） | `ros2 launch ... frame_id_camera:=my_cam`（README §4 把该参数描述为“TF 父坐标系”，用户会认为它决定图像 frame） | 该参数只影响 `/image_left/camera_info`（`frame_camera`）与两条静态 TF 的父坐标系，**拼接图像仍写死 `"camera"`**。默认值下 README 行 72（拼接图 `camera`）与行 105（`frame_id_camera` 默认 `camera_left`，描述为 TF 父坐标系）互不一致，读者无法判断图像帧到底叫什么；一旦用户改参数，图像 frame 与 TF 父 frame 分属两个坐标系名，`tf2` 与 `image_geometry` 查 `camera`→`camera_left` 会失败（无该边）。 | 把 `image_message` 的 frame_id 参数改为 `self.frame_camera` 并在 README 明确“该参数同时是拼接图 frame_id 与 TF 父坐标系”；或者新增独立参数并把 README 的两处写清（不改语义的前提下二选一）。 |
| **C-03** | Critical | `camera_node.py:337-339`（offset 计算不做有效性判断）、`:360-361`（`stamp_of` 无条件换算）、`:310`（IMU 同一路径） | 首帧 `timestamp_ns == 0`（SDK `frame_ts_ns` 在 `trig_tv`/`timestamps`/`tv` 三者皆 0 时返回 0，见 `core/src/devices/pipeline/rdkx5/rdkx5.cpp:32-41`）；或 IMU 包 `timestamp_ns == 0` | 契约 §4.3 明确规定：帧时间戳为 0 时**不得**据此确定偏移量，该帧 stamp 取节点 `now()`；IMU 包为 0 时**丢弃并 WARN**。当前代码会把 0 当作有效值：`offset_ns = now_ns - 0 = now_ns`（≈1.79e18），随后每帧 stamp = 设备小值 + 1.79e18 ≈ 当前时刻，看似“正常”，但 **offset 被永久污染**（首帧之后所有帧的真实曝光间隔仍在，可 offset 本身已错）；若 0 出现在运行期某帧，`stamp = offset + 0 ≈ now` 尚可，而 IMU 的 0 包会算出与图像同域的“当前时刻”，把一个无时间信息的样本冒充成有效同步样本，直接破坏下游 `message_filters` 对齐。 | `_wait_for_first_frame`：`ts = images[...].timestamp_ns; if ts: self.offset_ns = now - ts` 否则继续等待（不返回）；`stamp_of`：`ts==0` 时返回上一次有效 stamp + `1e9//fps` 并 1 Hz 节流 WARN；`_poll_imu`：`packet.timestamp_ns == 0` 时跳过发布并 WARN。 |
| **C-04** | Critical | `camera_node.py:169`（`except KeyboardInterrupt`）配 `:391-407`（`shutdown` 只忽略 SIGINT） | 用 SIGTERM 结束节点（`ros2 launch` 正常退出、`timeout`、systemd、`kill <pid>`），且在 `device.stop()/close()` 执行期间再来一次 SIGTERM/SIGINT | `rclpy.init()` 默认 `SignalHandlerOptions.ALL`，SIGINT 与 SIGTERM 都由 rclpy 的 C++ 处理器接管（`rclpy/__init__.py:80`、`rclpy/signals.py`）：收到信号时置 guard condition 让 `spin()` 返回，**不会**抛 Python `KeyboardInterrupt`。因此 `except KeyboardInterrupt` 实际上不可达（死分支），而 `signal.signal(SIGINT, SIG_IGN)` 只压制 SIGINT——第二次 SIGTERM 仍会按 rclpy 处理器语义终止进程，`device.close()` 可能未执行完，相机被持有到进程死亡（SDK 靠 `deinit()` 复位并断电传感器）。契约 §7.4 要求关闭序列在 3 秒内完成且**不得**依赖 `Device.__del__`。 | 在 `shutdown()` 内同时忽略两个信号（`for s in (signal.SIGINT, signal.SIGTERM): signal.signal(s, signal.SIG_IGN)`，并保存/恢复原值），或更稳妥地用 `signal.pthread_sigmask(SIG_BLOCK, {SIGINT, SIGTERM})` 包住 stop/close。 |
| **C-05** | Critical | `calibration.py:20-25`（`distortion_model` 的 pinhole 分支返回 8 个系数） | EEPROM 报 `DistModel.PINHOLE` 且系数非全零（`mode:=resize`/`raw`） | 冻结契约 §3.4.1 与附录 A8 明确：`plumb_bob` 只能是 **5 个**系数 `[k1,k2,p1,p2,k3]`，SDK 的 8 元 `dist_coeffs` 尾部必须**如实丢弃**；`rational_polynomial` 才允许 8 个（按 `k1,k2,p1,p2,k3,k4,k5,k6` 顺序）。契约同时禁止未评审地改接口。当前实现返回 8 元并声明 `rational_polynomial`，与冻结契约冲突。注意 `ros/README.md:119` 与 `ros/test/test_calibration.py:48-54` 与实现一致——即**实现+文档+测试同时偏离冻结契约**，改哪边都必须走接口评审，属于需要决策的冲突而非单纯笔误（这也是我把它列在“发布前必须定夺”的原因）。 | 若以冻结契约为准：pinhole 分支改为 `return "plumb_bob", values[:5]`，同步改 README §5 与 `test_calibration.py`，并在 `11_interface_freeze.md` 附录 B 记录变更；若以当前实现为准，则必须显式更新冻结契约（不得两边并存）。 |
| **M-01** | Major | `camera_node.py:360-361`（`stamp_of` 无条件 `to_ros_ns`）、`:140`（`offset_ns` 初值 0） | `publish_imu:=true` 且 `device:=GS130W`（无 IMU）；或任何设备在 `_wait_for_first_frame` 尚未确定 offset 前的窗口 | `_stamp(0 + 0)` 会产出 `sec=0, nanosec=0`，即 **1970-01-01**。虽然 IMU 定时器在 offset 确定之后才创建（`_start` 中 `_wait_for_first_frame` 在 `create_timer` 之前），但 `stamp_of` 本身对“未知时间戳”没有任何防线：任何一次 `offset_ns` 未初始化/被清零（例如未来在 `_start` 失败重入、或有人复用该函数）都会静默发出 epoch 0 的时间戳，而契约 §4.3 对这种情况有明确降级规则。 | 在 `stamp_of` 中判空：`if not timestamp_ns or self.offset_ns == 0: <降级规则>`；至少 `assert`/`WARN` 一次，杜绝 1970 时间戳流出。 |
| **M-02** | Major | `camera_node.py:295-306`（`_poll_imu` 无丢包统计）、`:162`（tick 周期 `1/max(2*odr,20)`） | `odr:=500`（SDK 允许的另一档），或 200 Hz 下 DDS 序列化被图像回调挤压 | 容量本身安全：每 tick 64 包 / tick 间隔 `1/(2*odr)` ⇒ 200 Hz 下 25600 包/s、500 Hz 下 64000 包/s，均远超产出率，所以**不会**因单 tick 取不空而无界增长（SDK 侧 FIFO 也有界：imu 1024 + `DROP_OLD`）。真正的问题是**丢包完全静默**：SDK 的 IMU 是 FSYNC 后成组突发产出（契约 §3.3），一旦某次回调被长阻塞（一帧 0.92 MB 序列化、DDS 拥塞），溢出由 SDK `DROP_OLD` 直接丢弃且无通知 API，节点 `self.packets` 只统计“取到的包”，于是契约 §7.3 要求的 `dropped {n_drop}` 永远缺失，运维无法发现 IMU 掉数据。 | 在 `_report` 用 `gs130.available_imu()` 的差分（配合时间戳间隔）估算丢包并在日志中输出；`publish_imu:=false` 时该队列会被 SDK 自然丢弃，不属缺陷。 |
| **M-03** | Major | `camera_node.py:365-375`（`_report`） | 运行 5 s 后看日志 | 契约 §7.3 要求每 `log_fps_period_s` 输出**速率**：`camera {fps:.2f} Hz, published {n} combine / {n_eye} per-eye, dropped {n_drop}, imu {odr:.2f} Hz, published {n_imu}, offset {offset_ns} ns`。当前 `_report` 打印的是**累计** `self.frames`/`self.packets` 且从不重置，无法据此算出真实帧率（E4 的 29.98 Hz 结论在代码里没有任何支撑点）；`offset_ns` 也不打印。日志内容与契约不符，是“验收靠人肉算差值”的缺陷。 | 在 `_report` 中记录上次计数与时刻，输出差值/间隔得到的 Hz + offset，或按契约文本逐字补齐。 |
| **M-04** | Major | `camera_node.py:278-282`（按 `self.stitched` 决定话题）与 `README.md:63-65`、`gs130_web.launch.py:22`（`image_topic` 默认 `/image_combine_raw`） | `ros2 launch gs130_ros gs130_web.launch.py stereo_layout:=none` | 该组合下节点只发 `/image_left_raw`、`/image_right_raw`，而 hobot_codec 仍被要求订阅 `/image_combine_raw`（`launch_arguments` 直接传 `LaunchConfiguration("image_topic")`）：网页永久黑屏，`ros2 topic list` 也没有 `/image_combine_jpeg`。README §3 靠“何时出现”一列隐含说明，但没有“web launch 必须保持拼接”的显式警告，也没有启动期一致性检查。 | 在 `gs130_web.launch.py` 中显式声明/校验该组合（例如把 `stereo_layout` 的 default 固定为拼接并在 README §2.1 加一句“`stereo_layout:=none` 会断开 web 链路”）。 |
| **M-05** | Major | `camera_node.py:238-263`（`_publish_transforms` 无条件发布 `device.relative_R/T` 的结果）与 `:177-190`（`_validate` 不校验 `mode` 与 `layout` 的组合） | `... mode:=rect`（默认 `stereo_layout:=left_right`） | `mode=rect` 下 SDK 先做双目矫正，外参被替换为**虚拟平行双目外参**（C++ `rdkx5.cpp:194-215` 调 `stereo_rectify()`；`00_verified_platform_facts.md` 与 README §5 都写明“参考系随之改变”），而节点照样把 `relative_R/T` 的结果当成几何真值发到 `/tf_static`，且 **`_validate` 不校验、启动日志不提示**。下游（`hobot_stereonet`、点云拼接、手眼标定）拿到的是虚拟基线/虚拟姿态却无从得知，只有读过 README 第 5 节的人才会发现。 | 启动时对该组合打一条明确的 WARN（例如 `rect mode: /tf_static extrinsics are virtual (parallel stereo), not the physical mounting`），或在 `_publish_transforms` 内按 `self.mode_name == "rect"` 分支提示；不引入新特性、不改消息内容。 |
| **M-06** | Major | `camera_node.py:137-150`（视频通路故障后不退出）与 `:267-285`（`self.frames` 只统计成功帧） | 相机通路故障后（见 C-01），或 SDK FIFO `DROP_OLD`（`Config.preset` 中 `camera_fifo=FifoConfig(4, DROP_OLD)`）丢帧时 | 帧丢失是**静默**的：SDK 队列深 4、策略 DROP_OLD，一旦节点两次 tick 之间积压 >4 帧，旧帧被 SDK 直接 `free()` 掉且无任何通知 API（`gs130_available_camera()` 只给“当前可取数”，不给“累计丢弃数”）。因此“每 tick 最多 2 帧”这一设计（`:269`）只在 `fps <= 2*定时器频率` 时无损（30 fps 下 60 Hz tick × 2 = 60 帧/s，安全；`fps:=33` 时 66 帧/s，仍安全），但一旦 tick 被长阻塞（大消息序列化、DDS 拥塞、GC）就会静默丢帧且日志无痕。 | 在 `_poll_images` 用 `self.device.available_camera()` 在取帧前检测积压（`>2` 即 WARN 节流），并在 `_report` 输出累计丢帧估计。 |
| **m-01** | Minor | `camera_node.py:186-187` 与 `Image.__new__`（`python/gs130/_types.py:23-27`） | `height:=479`/`481` 等奇数（见 B-03） | 除高度公式外，偶数校验的缺失还让 CameraInfo 与图像维度不一致（CameraInfo 用配置值，图像用 shape 反算），即“同一话题的几何自相矛盾”；这是一处、两处表现，修复同 B-03 的校验。 | 同 B-03（校验阶段拒绝奇数维度）。 |
| **m-02** | Minor | `camera_node.py:321-322`（注释“ROS reads as unknown from the zeroed covariances”）对照冻结契约 §3.3 与附录 A7 | 任何 `/imu/data` 消息 | 契约要求 `angular_velocity_covariance[0] = -1`、`linear_acceleration_covariance[0] = -1`（“驱动没有逐样本协方差，用 -1 显式声明无估计”）。当前代码只设 `orientation_covariance[0] = -1`，另两个保持 0，而消息默认值为 0，故**行为上违反契约**（附录 A7 会判失败）；注释还把这个偏离描述成“ROS 读作 unknown”，属于把契约违反写成特性。 | 补两行 `message.angular_velocity_covariance[0] = -1.0`、`message.linear_acceleration_covariance[0] = -1.0`，并同步 README §6 措辞。 |
| **m-03** | Minor | `camera_node.py:400`（`signal.signal(signal.SIGINT, signal.SIG_IGN)`） | 在非主线程中调用 `shutdown()`（例如将来把该节点放进 `MultiThreadedExecutor`+回调线程、或测试里从线程调用） | `signal.signal()` 只能在主线程调用，否则抛 `ValueError: signal only works in main thread`。当前调用点在 `main()` 主线程，故现状安全；但该函数是公开可复用的关停入口，且异常会在 `finally: signal.signal(...)` 恢复处再次抛出，可能在设备已 `close()` 后把异常抛给调用者。 | 用 `try/except ValueError` 包住信号操作，或改用 `pthread_sigmask`（不依赖主线程）。 |
| **m-04** | Minor | `camera_node.py:287-293`（`_publish_image` 的 docstring 在 `image_message` 已不再负责缓冲区生命周期） | 阅读/维护代码时 | `_publish_image` 现在只做 `publisher.publish(image_message(...))`，其 docstring 却详细描述“必须在返回前拷贝、否则 SDK 缓冲被释放”。真正的拷贝发生在 `image_message`（`:424-425`），该函数没有对应说明。这是误导性 docstring（评审项 9 明确点名），且会诱导后续维护者在错误的位置加/去拷贝。 | 把该 docstring 移到 `image_message`，`_publish_image` 只保留一行说明。 |
| **m-05** | Minor | `camera_node.py:424-425`（`data.frombytes(frame.tobytes())`） | 每帧拷贝（E3/E4 已验证可用） | 正确性与 E3 结论一致（`array.array('B')` + `frombytes`），但 `tobytes()` 会额外申请一份 0.92 MB 临时缓冲（E3 实测 0.31 ms），随后 `frombytes` 再拷一次；`frombytes(memoryview(frame))` 是 E3 表里 1.85 ms 的等价写法，可省一次中间分配。不是缺陷，只是可省的内存峰值（对 board 端 1 GB 级内存有意义）。 | 可选：`data.frombytes(memoryview(frame).cast("B"))`，需按 E3 方式复测后采用。 |
| **n-01** | Nit | `camera_node.py:14` 与 `:439`（`_stamp` 内重复 `from builtin_interfaces.msg import Time`） | 静态阅读 | 顶层已导入 `Time`，函数内再次局部导入，属重复导入/死代码风格问题。 | 删除 `_stamp` 内的局部 import。 |
| **n-02** | Nit | `camera_node.py:452-453`（`except KeyboardInterrupt: code = 0`） | SIGINT | 如 C-04 所述，rclpy 的信号处理器不抛 `KeyboardInterrupt`，该分支不可达。保留无害，但会让读者误以为 SIGINT 路径已被覆盖。 | 改为注释说明“rclpy 用 guard condition 处理信号，spin() 正常返回”，或保留分支但加注解。 |
| **n-03** | Nit | `camera_node.py:158`（`_report()` 在 `_wait_for_first_frame` 之后调用） | 启动日志 | `_wait_for_first_frame` 会消费掉首帧（用于求 offset 且丢弃），紧接着的 `_report()` 打印 `frames=0`，容易被误读为“没出图”。 | 在 `_report` 增加 `offset_ns` 并注明首帧已用于求偏移，或先报再等。 |
| **n-04** | Nit | `camera_node.py:143`/`:283`（`self.frames` 语义）与 `README.md:47`（`ros2 topic hz ... # 约等于 fps`） | `stereo_layout:=none` 时 | `self.frames` 在单目模式下每帧加 1（一对左右目算 1），语义是“帧对”，README 的 hz 说明未区分“拼接帧率”与“帧对率”。 | README §2.2 补一句说明。 |
| **n-05** | Nit | `ros/gs130_ros/setup.py:12`（`packages=find_packages(exclude=["test"])`）与 `ros/test/*` | `colcon build` / 安装后 | `ros/test/` 位于包目录之外、也未在 `setup.py` 中安装，因此板端 `python3 -m unittest discover -s ros/test` 依赖人工拷贝（README 未提），测试实际不会随包分发。 | 在 README 的“构建”步骤里显式给出测试命令与路径，或把测试纳入安装清单（二选一，不引入新依赖）。 |

---

## 1a. 与既有契约评审的交叉引用（避免重复裁决）

评审期间发现本仓库已存在并行的契约评审文档，其中若干条目正是“实现与冻结契约不一致”类问题。为避免同一问题被两处重复裁决，本节只做**交叉引用**，不再独立立案；仍需发布前定夺的实现相关项已在 §1 表中保留。

| 本报告项 | 与之对应的既有条目 | 说明 |
|---|---|---|
| C-05（PINHOLE → 8 元 `rational_polynomial`） | `ros/docs/24_contract_review.md:104`（C36）、`:436`（R-09） | 既有评审已登记为 `DIVERGES` 并注明“须由评审统一”。本报告在 §1 中只保留“必须二选一并同步实现/README/测试”这一定夺动作，不重复论证。 |
| m-02（`angular_velocity_covariance[0]` / `linear_acceleration_covariance[0]` 未置 `-1`） | 冻结契约 §3.3、`21_acceptance_criteria.md` 附录 A7 | 属实现侧可一行修复的行为偏差，仍列在 §1。 |
| 契约文本与实现相反或未实现的其余条目（`height=eh*3//2`、图像 `BEST_EFFORT`、禁止 `stereo_layout`、禁止 `/tf_static`、`R/P` 全 0、`publish_per_eye`、`/gs130/status`、`stamp_offset_*` 与 §4.3 降级规则、`camera_info_distortion_model`、launch-only 参数表等） | `ros/docs/24_contract_review.md` §4/§5（D-01…D-12、R-01…R-13） | 须由评审会裁决“契约文本过期”还是“实现缺失”。本报告按任务范围（找实现缺陷、不提重设计）不在 §1 重复列出，但在 C-03/C-04/M-01/M-03 中引用了其中与运行期行为直接相关的降级规则作为判据。 |

两点需要团队注意：
1. `24_contract_review.md` 已确认 `11_interface_freeze.md` 的 `height = eh*3//2` 条目应作废（E2 实测段错误），即本报告 B-03 依据的“真实高”约定与最终契约方向一致；B-03 的独立价值在于**奇数高度下该公式本身失效**，这一点既有评审未覆盖。
2. 契约文本与实现“相反”的条目里，真正会影响运行期行为的是时间戳降级规则（C-03/M-01）与相机通路故障必须退出（C-01）；其余多为文档口径问题。

---

## 2. 已检查且判定为正确的部分（评审覆盖面）

以下项经逐行对照 SDK 源码/头文件语义与实测事实核验，**未发现缺陷**，供团队判断本轮评审的边界：

**资源所有权与生命周期**
- `__init__`（`:115-123`）先 `self.device = self._create_device()` 再 `try: start(); _start()`，`except BaseException: self.shutdown(); raise`：**异常/SystemExit 路径都不会留下已打开的设备**；`shutdown()` 用 `device, self.device = self.device, None` 做原子接管，重复调用是安全 no-op（`:397-399`）。
- `Device.__init__`（`_device.py:23-54`）自身在 `gs130_init` 失败时执行 `deinit`+`destroy`，并把 `_closed` 置位；节点在 `gs130_init` 失败路径上不持有任何需要清理的句柄，`fail()` → `SystemExit(2)` 的路径无泄漏。
- `_validate`/参数解析（`:113`、`:124-135`）全部发生在 `_create_device()`（接触硬件）之前；`fail()` 只在设备已打开之后的运行期被调用（`_wait_for_first_frame`），且该调用点被 `__init__` 的 `except BaseException` 覆盖 → `shutdown()` 必然执行。

**缓冲区生命周期（发布路径）**
- `image_message`（`:416-436`）：`frame.tobytes()` 产生独立 bytes → `array.array('B')` 再拷一份 → 赋给 `message.data`（走 `sensor_msgs/_image.py` 的 `array.array` 快路径，不做逐元素校验）。`publisher.publish()` 返回后才离开函数，numpy 视图随后析构触发 `_types.py:25` 的 `weakref.finalize(owner, _free, address)`。
  - 因此：**不会发布指向已释放内存的数据**（拷贝在返回前完成）、**不会重复 free**（finalizer 只挂在 `owner` 上，`Image` 只是视图）、**不会因提前析构而丢帧**。E3 的性能结论（1.36 ms vs 1103.87 ms）在本实现中被正确采用。
- `_poll_images`（`:263-281`）在 `images` 存活期间取 `stamp`（纯 int，不保留引用）、发布后立即丢弃数组，无“先删数组再发布”的顺序错误。

**消息几何与契约**
- 每目 640x480：`width=shape[1]`、`height=shape[0]*2//3=480`、`step=shape[1]`、`len(data)=460800`；拼接 `left_right` 640x480：1280/480/1280/921600——与 README §3 表格和 E2 约定逐字一致（我用 numpy 复算了 480/960/1088 等偶数情形，全部自洽）。
- `left_right`/`right_left` 两个横向布局的数据排布与 C++ 侧 `get_frame(..., y_stride=uv_stride=width*2)` 的行交错方式一致，`hobot_codec` 按“一张 2W×H 的打包 NV12”解释是正确的（这是默认链路，E4 29.98 Hz 亦印证）。
- QoS：图像 RELIABLE、depth=1、VOLATILE（E1 要求）；CameraInfo `TRANSIENT_LOCAL`（latched，`depth=1`，晚加入者可拿）；IMU depth=200 与契约 §5.1 一致。

**不启动硬件即可判定的标定映射**
- `transform(device, from, to, parent, child)` 的方向约定**正确**：`gs130.h:390/408` 明确 `relative_R/T(from,to)` 是“把 from 下的点变换到 to”，即以 `to` 为参考系描述 `from` 的位姿，故 `parent=to`、`child=from` 成立。节点用 `(CAMERA_RIGHT, CAMERA_LEFT)` + `parent=frame_camera, child="camera_right"`，得到 `T ≈ [+0.070, 0, -0.002]`（正基线），与实测一致——方向**没有**反号（若反了会是 `-0.070`）。
- `_quaternion`（`calibration.py:79-114`）四个分支逐一核对 Shepperd 公式：`trace>0`、`index=0/1/2` 的分量顺序与符号（如 `x=(m[2][1]-m[1][2])·s`、`w=(m[1][0]-m[0][1])·s`）全部正确；三处分母分别由 `1+trace`、`1+m00-m11-m22` 等构成，在对应分支下恒为正（对角最大元素保证），不存在对负数开方或除零。`test_calibration.py` 用旋转向量回代校验属有效验证。
- 畸变模型选择：全零 → `plumb_bob` + 5 个零（RECT 的正确行为）；FISHEYE → `equidistant` + 前 4 个；PINHOLE → 见 C-05（与冻结契约的 8 vs 5 冲突，其余映射正确）。
- `K`（行主序 9 元）、`R=单位阵`、`P=[fx,0,cx,0, 0,fy,cy,0, 0,0,1,0]` 的字段顺序与长度均正确；`matrix()` 的 `reshape(-1)` 与 `_types.py` 的 `K` 形状 `(3,3)` 匹配。
- HFOV 无关项：`camera_info` 的 `width/height` 取配置值而非 `shape`，与“K 已在输出分辨率下（`rdkx5.cpp:267-273` 回写）”的前提一致。

**时间戳（除 C-03/M-01 外）**
- 一次性 offset 的设计与 E5 一致：图像与 IMU 共用 `to_ros_ns`（`:357-361`、`:310`），两者时间差保持设备原值（实测 8.6 ms），未做插值/重采样，符合契约 §4.2 的“运行期恒定、禁滑动平均”。
- `_stamp` 的 `sec/nanosec` 拆分（整除 + 取模）对正数正确；`stamp_of` 用 `int()` 而非 `float()` 避免精度丢失。

**定时器与速率（除 M-02/M-03/M-06 外）**
- 图像 tick = `1/max(2*fps,10)`（`:160`）、每 tick ≤2 帧：默认 30 fps 时容量 60 帧/s ≥ 30，**不会**让 SDK 深 4 的队列溢出；`fps=1` 时退化为 10 Hz tick，仍 ≥ 2 帧/s。
- IMU tick = `1/max(2*odr,20)`（`:162`）、每 tick ≤64 包：200/500 Hz 下均有 >10 倍容量裕量，不会因单 tick 未取空而无界增长；因为 SDK 侧 FIFO 有界（camera 4 / imu 1024，均 `DROP_OLD`），最坏情况是丢数据而非内存增长。
- 两个 drain 循环都在“空队列/故障”时立即 `return`，单次回调最多 2+64 次非阻塞调用，**不会**阻塞 executor 到影响 5 s 上报定时器（`_report` 只做字符串格式化与日志）。

**错误信息**
- `_create_device` 的“相机被占用”提示（`:202-211`）包含**可执行**的自查命令，且 `ps -ef | grep -E "mipi_cam|camera_node"` 与 README §7 中给出的字符串一致（README 已由作者同步为 `camera_node`），属可操作信息。

**launch 文件（已按 refactor 后的 `launch_arguments.py` 复核）**
- 两个 launch 都改用共享的 `gs130_ros/gs130_ros/launch_arguments.py`：`camera_parameters()` 回传 `ParameterValue(LaunchConfiguration(name), value_type=int|bool|float|str)`，**参数类型在这里被显式固定**，不再依赖 launch_ros 的 YAML 猜测；因此 `width`/`height`/`fps`/`odr` 一定是 int、`publish_imu`/`publish_tf` 一定是 bool，节点侧 `_boolean`（`:68-77`）与 `_number_parameter` 只是第二道防线。**`publish_imu:=false` 这类文本布尔在两个 launch 与 `--ros-args -p publish_imu:=false` 三条路径下都能正确得到 `False`**，不存在“字符串 false 被当成真”的经典陷阱。
- 模块安装正确：`gs130_ros/gs130_ros/` 含 `__init__.py`，`setup.py` 用 `find_packages(exclude=["test"])` 收集，launch 文件 `from gs130_ros.launch_arguments import ...` 在 `source install/setup.bash` 后可用；`camera_parameters()` 返回的字典在列表里只出现一次，节点侧不会收到重复参数定义。
- 未修正但不构成缺陷：`WEB_ARGUMENTS` 里 `jpg_quality`（默认 `"85.0"`）与 `websocket_channel`（默认 `"0"`）的“类型”字段声明为 `str`，而它们只经 `LaunchConfiguration` 原样透传给既有 TROS 节点，`camera_parameters()` 不消费它们，故当前无实际影响（若将来有人复制该表用于 `ParameterValue` 会立刻暴露类型不符）。
- `gs130_web.launch.py` 只创建一个 `camera_node` 实例（`:39-45`），include 的 `hobot_codec`/`websocket` 不含相机节点，**不会出现两个相机持有者**；M-04 是“节点话题与 web 链路参数不一致”，不是双持有。
- 节点未使用 `respawn=True`（契约 §7.2 禁止）。

**不可在本机验证、故未列入缺陷表的项（诚实声明）**
- `hobot_codec_encode.launch.py` / `websocket.launch.py` 的参数名与取值域（`websocket_channel:=0` 是否合法、`jpg_quality` 类型、nginx 孤儿行为）本机无 TROS 包，无法核对；`websocket_only_show_image: "True"` 属既有节点参数，未验证。
- 奇数宽高、`fps > 33` 是否会被 SDK/VSE 提前拒绝（`rdkx5.cpp` 只有 `roi_ratio_exact()` 校验），未在真机确认；B-03/B-05 的价值在于“节点必须自己给出契约一致的消息与退出码”，与该问题无关。

---

## 3. 发布前必须先修的三个缺陷

1. **B-02（raw + 拼接导致 codec 段错误）**——`mode:=raw` 配默认 `stereo_layout:=left_right` 是**默认值就能踩中**的组合：节点会把一张实际为 1280x2176 的缓冲按 `height=2176` 发布，正是 E2 实测导致 `hobot_codec` 段错误（exit -11）的那类几何。一行 `_validate` 校验即可闭合，收益/成本比最高。
2. **B-01（非法 platform/device 走成 traceback + 退出码 1）**——任何拼写错误都会让用户看到 Python traceback 而不是契约要求的 `ERROR unsupported platform/device: ...`（退出码 2）；这条同时是“验收附录 A 逐条对照”的失败项，且修复只是把 `Config.preset()` 挪进 `try`。
3. **B-03 + B-05（维度/参数值域校验缺失）**——奇数 `height` 会让 `height`、`data` 长度与 CameraInfo 三者自相矛盾（README §3 的等式当场失效），`fps`/`odr`/奇数宽高都属于“启动时就能判定的非法组合”。补齐 `_validate` 即可，且能顺带把 C-01 的退出码分级（参数错=2 / 硬件不支持=1）一次改对。

其次建议立刻处理 **B-04（纵向拼接的 NV12 排布错误）**：它是唯一会让默认之外的合法参数静默产出垃圾图像的问题，且 README 目前把 `top_bottom`/`bottom_top` 宣传为可用值。
