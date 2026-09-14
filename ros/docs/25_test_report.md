# 25 测试报告 — gs130_ros v0.1.0

- 被测版本：`ros/gs130_ros` v0.1.0（commit 见仓库），依赖 `gs130` Python 包 0.0.1
- 环境：RDK X5，Ubuntu 22.04，TROS humble（`/opt/tros/humble`），kernel uptime 2h38m
- 执行者：总架构师（真机执行），命令与输出均为实际记录
- 相机：GS130WI（双目 1088x1280 + ICM-42688-P IMU + EEPROM 鱼眼标定）

---

## 1. 结果汇总

| 级别 | 用例 | 结果 |
|---|---|---|
| L1 主机/纯函数 | 9 项 unittest（畸变模型、CameraInfo、Image 契约、四元数、TF 方向） | **9/9 PASS** |
| L2 构建 | `colcon build --packages-select gs130_ros` | **PASS**（约 6.4 s） |
| L3 板端集成 | 话题、速率、QoS、消息字段、时间戳、IMU、标定、TF | **PASS** |
| L4 端到端 Web | 既有 codec + 既有 websocket + 浏览器画面 + 抓帧解码 | **PASS** |
| L5 关停/资源 | SIGINT 释放相机、无残留进程、端口回收 | **PASS** |

## 2. L1 纯函数单元测试

```
$ python3 -m unittest discover -s test -p "test_*.py" -v
test_fields_come_from_the_calibration ... ok
test_fisheye_keeps_four_equidistant_coefficients ... ok
test_pinhole_keeps_eight_coefficients ... ok
test_rectified_camera_reports_plumb_bob ... ok
test_nv12_uses_the_real_height ... ok
test_stitched_geometry_doubles_the_width_only ... ok
test_half_turn_uses_the_off_diagonal_branch ... ok
test_identity_rotation_becomes_a_unit_quaternion ... ok
test_quaternion_reproduces_the_rotation ... ok
Ran 9 tests in 0.011s
OK
```

覆盖的契约点：`height = shape[0]*2//3`、拼接只翻倍宽度、`step == width`、
`len(data) == w*h*3//2`、`encoding="nv12"`、畸变模型三态映射、
四元数在 trace>0 与 trace<=0 两条分支上都能复现旋转矩阵。

## 3. L3 板端集成测试

### 3.1 只启动相机节点（`gs130_camera.launch.py`）

```
[camera_node-1] calibration: left fx=386.85 fy=387.12 cx=304.62 cy=245.06 equidistant | right fx=388.52 equidistant
[camera_node-1] static tf from camera_left to camera_right, imu_link (baseline 0.070316 m)
[camera_node-1] gs130 timestamps are monotonic since boot; using a constant offset of 1789385065931459293 ns (accuracy within one frame period)
[camera_node-1] frames=0 imu=0 | left_right | 640x480 resize fps=30 odr=200
[camera_node-1] gs130 camera ready
[camera_node-1] first frame published
[camera_node-1] frames=152 imu=1030 | left_right | 640x480 resize fps=30 odr=200
[camera_node-1] frames=302 imu=2041 | ...
[camera_node-1] frames=752 imu=5072 | left_right | 640x480 resize fps=30 odr=200
```

结论：30 s 内 **752 帧 / 5072 个 IMU 包** ≈ **30.1 fps / 202.9 Hz**。
IMU 速率由 `odr` 决定，**不受相机帧率限制**（推翻了评审中"IMU 按帧成批、被 fps 门控"的推测）。

### 3.2 端到端 Web 链路（`gs130_web.launch.py`）

```
== topics ==            /image_combine_jpeg /image_combine_raw /image_left/camera_info
                        /image_right/camera_info /imu/data /tf_static
== image hz ==          29.997   (min 0.029s max 0.037s std 0.002, window 273)
== jpeg hz ==           29.997   (min 0.029s max 0.038s std 0.002, window 275)
== raw qos ==           gs130_camera              Reliability: RELIABLE
                        hobot_codec_encoder_7dd06e87  Reliability: RELIABLE
== tf ==                Translation: [0.070, 0.000, -0.002]
== web ==               http_code=200
```

结论：我们自己发布的 NV12 被**既有** `hobot_codec` 正常编码、被**既有** `websocket`
正常推送到网页；两侧 QoS 均为 RELIABLE（E1 的硬要求）。

### 3.3 消息契约（从话题实抓一帧）

```
topic          /image_combine_raw
encoding       nv12
width          1280
height         480            (真实图像高)
step           1280
data length    921600         (expected 921600)
frame_id       camera
stamp          1789395368.711803996
contract       OK

camera_info    640x480 model=equidistant
  K            fx=386.85 fy=387.12 cx=304.62 cy=245.06
  D            [-0.025173, 0.012189, -0.013019, 0.002486]
  P            [386.85, 304.62, 387.12, 245.06]
  R            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
```

`stamp.sec = 1789395368` 与板卡墙上时间同域（E5 的 offset 生效），
若直接透传 SDK 时间戳该值会小 1.79e9 秒。

### 3.4 图像真实性（人工目视）

抓帧解码并存 PNG：

```
stitched png  capture_stitched.png  shape=(480, 1280, 3)
left png      mean=120.7 std=64.7
right png     mean=110.3 std=63.9
eyes differ   True
```

总架构师目视该 PNG：画面为办公桌场景，左半与右半为同一场景的左右目视图，
椅子、显示器、桌面物体存在符合视差的水平位移，左右目亮度差 10.4
（与基线 0.070 m 的轻微曝光差异一致）。

**结论：不是花屏、不是错位、不是灰度噪声，是真实可用的双目图像。**

## 4. L5 关停与资源释放

```
[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
[camera_node-1] camera released
[camera_node-1] process has finished cleanly [pid 17811]
```

关停后核查：

```
$ ps -ef | grep -E "nginx|websocket|hobot_codec|camera_node" | grep -v grep
CLEAN
$ ss -ltn | grep :8000
port 8000 free
```

注意：`nginx` 由既有 `websocket.launch.py` 用 `os.system` 启动，**不会**随 launch 退出而结束，
必须显式清理；这一点已写入 README 与冻结书修订通知（E4）。

## 5. 迭代中发现并修复的真实缺陷

| # | 缺陷 | 现象 | 修复 |
|---|---|---|---|
| 1 | `header.stamp` 被赋值为整数 | 首个 IMU 包触发 `AssertionError: The 'stamp' field must be a sub message of type 'Time'`，节点退出码 1 | 改为经 `_stamp()` 构造 `builtin_interfaces/Time`；IMU 与图像共用同一换算 |
| 2 | 关停期间第二个 SIGINT 会中断释放 | SIGINT 落在 `device.stop()` 中抛 `KeyboardInterrupt`，进程 -2 退出，相机可能被残留持有 | `shutdown()` 期间忽略 SIGINT，并分别捕获 stop/close 异常，保证 `close()` 一定执行 |

两处都是真机迭代才暴露的问题，均已复测通过。

## 6. 未覆盖与待验证

| 项 | 状态 | 说明 |
|---|---|---|
| `mode:=rect` | **已测 PASS** | 451 帧 / 15 s（30 fps）；`camera_info` 为 `plumb_bob` + 5 个零、fx=362.07 cx=320 cy=240（已矫正内参） |
| `mode:=raw` + 单目 | **已测 PASS** | `stereo_layout:=none`，452 帧 / 15 s，`width=1088`（`height=1280`） |
| `mode:=raw` + 拼接 | **已测（拒绝）** | 组合非法，节点 FATAL 拒绝，退出码 2 |
| 参数校验矩阵 | **已测 PASS** | 见 `00_verified_platform_facts.md` E10，8 类非法输入全部按预期拒绝 |
| 相机被第二个进程抢占 | **已测（发现）** | 见 E8：第二个进程不报错但抢走帧流，先到者静默停帧；已加看门狗并实测触发 |
| `stereo_layout:=none` 两目话题 | **已测 PASS** | `/image_left_raw` 29.993 Hz；`/image_right_raw` 同时存在 |
| `top_bottom` 布局 | **已测 PASS** | 抓帧 `640x960` 解码目视为正确上下堆叠双目图（E9） |
| 其它分辨率/更高 fps | 未测 | 上游源码提示部分分辨率因整除约束不可用；`fps` 已限制 ≤33 |
| 长时间运行漂移 | 未测 | 一次性 offset 的长期漂移（温度/时钟）未评估 |
| `hobot_stereonet` 联调 | 不做 | 需其 launch 不拉 `mipi_cam`，v0.1.0 明确不接线 |

### 6.1 关于"相机占用"的更正

验证过程中发现（E8）：相机流水线**不可共享，但第二个打开者不会被拒绝**。
两个 `camera_node` 同时运行时，后启动者正常出图，先启动者**静默停帧**（IMU 仍在流）。
因此本报告早先关于"占用会以 `gs130_init` 失败暴露"的假设**不成立**；
真正的占用症状是停帧，已在节点中加入看门狗并实测触发：

```
[camera_node-1] [ERROR] no camera frames for about 10 s while the IMU still streams.
Another process has most likely taken the camera (mipi_cam or a second camera_node);
check with 'ps -ef | grep -E "mipi_cam|camera_node"'.
```

上述未测项不得被解读为"已验证"；发布说明中按未验证对待。
