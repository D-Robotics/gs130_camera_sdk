# 33 GS130 ROS 2 接口包 —— 用户评审第二轮（实施后）

- 评审人：UX-4（用户评审组，代表 RDK X5 + GS130 的 ROS 2 应用开发者；与前两轮的 UX-1 / UX-2 / UX-3 是同一类用户）
- 评审对象：**已实施**的 `ros/gs130_ros` v0.1.0（分支 `develop`）
- 评审时机：实施后、发布前
- 我方取证范围：只读本仓库交付物 —— `ros/README.md`、`ros/gs130_ros/launch/*.launch.py`、`ros/gs130_ros/gs130_ros/camera_node.py`、`ros/gs130_ros/gs130_ros/calibration.py`、`ros/gs130_ros/{package.xml,setup.py,setup.cfg}`、`ros/test/capture_frame.py`、`python/gs130/_device.py`、`python/gs130/_config.py`、`python/gs130/_runtime.py`、`python/gs130/_types.py`、`ros/docs/30_user_feedback_round1.md`、`ros/docs/31_usability_review.md`、`ros/docs/00_verified_platform_facts.md`、`ros/docs/32_integration_expectations.md`
- 板端实测结果（由总架构师提供，本评审当作事实采信，**未登板**）：web launch 起链路、8000 端口 HTTP 200、`/image_combine_raw` 与 `/image_combine_jpeg` 各 29.997 Hz、IMU 203 Hz（odr=200）、encoding `nv12` / 1280x480 / step 1280 / len 921600 / frame_id `camera`、左目 camera_info 为 640x480 equidistant fx=386.85 fy=387.12 cx=304.62 cy=245.06 / D 长 4 / R 单位阵 / P 已填、`camera_left→camera_right` 平移 `[0.070, 0.000, -0.002]`（baseline 0.070316 m）、Ctrl-C 打印 `camera released` 且无残留、抓帧可解出有视差的 1280x480 立体对、启动日志含标定摘要/tf 行/时间戳 offset 行/每 5 秒 `frames=N imu=M | left_right | 640x480 resize fps=30 odr=200`/`gs130 camera ready`/`first frame published`
- 本地可执行验证（不在板上跑相机）：用一份临时 ament_python 探针包在 `/opt/ros/humble` 上确认了 ①`console_scripts` 入口点确实 `sys.exit(main())`，非零返回码会传到进程（`return 2` → 退出码 2）；②`LaunchDescription.visit()` 不会清除 CLI 传入但未 `DeclareLaunchArgument` 的配置（不报错、不警告）
- 立场：我是要把这套东西接进真实项目的人。下面每一条判断都指到文件/行或实测数据；不接受的表述是"在我们机器上是好的"。

---

## 0. 一分钟结论

**方向对了，细节还欠一遍"第一次用的人"的摩擦。** 与前两轮相比，实质性缺陷（QoS、NV12 的 `height` 语义、时间戳时钟域、fisheye 被写成 `plumb_bob`、IMU 被帧率门控、Ctrl-C 不释放相机）**全部被真正修掉了**，而且有板端数据支撑，这一点必须承认。但作为"今天就要接手的人"，我仍然有 **3 个必须先修的项**（§6.1）和 **5 个可以带着文档发布的限制**（§6.2）。

最关键的一条不是代码，而是：**`ros/` 整个目录在 `develop` 上没有被 git 跟踪**（`git ls-files ros/` 为空，`git status` 显示 `?? ros/`）。也就是说"按 README 走一遍"这件事在**当前仓库状态下一次都做不成**。这条对"能不能发给别的团队"是决定性的。

---

## 1. 按 README 顺序走一遍（PASS / FAIL / UNCLEAR）

前提：仓库在 `/home/leaf-jammy/rdkx5_work/gs130_sdk`，板端已装 TROS humble、colcon、numpy 1.26.4、OpenCV 4.11.0（`00_verified_platform_facts.md` E7）。

| # | README 步骤 | 判定 | 依据与理由 |
|---|---|---|---|
| 1 | §1 依赖表：RDK X5 + TROS humble / `gs130` Python 包 / `hobot_codec`+`websocket` / numpy+OpenCV | **PASS** | `README.md:11-18`。四项与板端事实一致（E7 已确认 colcon/rclpy/numpy/OpenCV 在板）。缺一项：`libgs130.so.0` 本身（Python 包是 ctypes 绑定，`python/gs130/_runtime.py:26-35` 靠 `ctypes.util.find_library("gs130")` 或 `GS130_LIB` 找库）。板上已装，所以现在不痛；换一块板就会痛。 |
| 2 | §1 构建 wheel：`cd python && ./build-wheel.sh wheel && sudo pip3 install dist/gs130-*.whl` | **PASS（带前提）** | `python/build-wheel.sh` 存在且可执行（git mode 100755），脚本自己做 `pip wheel --no-deps .` 并打印 `ls -lh dist/gs130-*.whl`；`dist/gs130-0.0.1-py3-none-any.whl` 已在树里。前提：**必须在仓库根目录执行**，README 没有写"从这里开始，假设 cwd=仓库根"，而命令本身混用了 `cd python`（相对）和 `ros/gs130_ros`（相对根），读者得自己推断起点。 |
| 3 | §1 `mkdir -p ~/gs130_ws/src && cp -r ros/gs130_ros ~/gs130_ws/src/` | **FAIL（今天）** | 命令本身正确（`setup.py:11-18` 声明的 `launch/`、`package.xml`、resource 都在包目录内，整目录拷贝可用）。FAIL 的原因是**交付状态**：`ros/` 在 `develop` 上未被跟踪，任何 `git clone` 出来的人都拿不到这个目录。 |
| 4 | §1 `source /opt/tros/humble/setup.bash` + `colcon build --packages-select gs130_ros` + `source install/setup.bash` | **PASS** | `package.xml` 的 `<export><build_type>ament_python</build_type>`、`setup.cfg` 的 `script_dir=$base/lib/gs130_ros` 配置正确；缺 `setup.py` 里的 `tests_require`/`pytest` 不影响。README 没写"build 失败先看 `log/latest_build`"，但这属于常识，不计。 |
| 5 | §2.1 首跑 `ros2 launch gs130_ros gs130_web.launch.py` | **PASS（实测）** | 板端已实测：一条命令拉起 camera + codec + websocket，无必填参数。`gs130_web.launch.py:41-88` 的 12 个 launch 参数都有默认值。 |
| 6 | §2.1 打开 `http://<板卡IP>:8000/`，channel 0 看到左右拼接画面 | **PASS（实测）** | HTTP 200 + channel 0 live image 已实测。**但 README 没有给"怎么得到板卡 IP"**（UX-2 §5.1 要求 `hostname -I`），也没写 nginx 由 `websocket.launch.py` 用 `os.system` 起、launch 退出后变孤儿（README §7 只在"端口 8000 被占用"一行提了处置）。 |
| 7 | §2.2 只看话题：`ros2 launch gs130_ros gs130_camera.launch.py` | **PASS** | 与 web launch 共用同一个 `camera_node`，只是不带 codec/websocket。 |
| 8 | §2.2 `ros2 topic hz /image_combine_raw` | **PASS** | 实测 29.997 Hz（fps=30）。README 写"约等于 fps"，准确。 |
| 9 | §2.2 `ros2 run tf2_ros tf2_echo camera_left camera_right` | **PASS** | `/opt/ros/humble/share/tf2_ros` 与 `tf2_msgs` 存在（本机核对）；静态 TF 实测 `[0.070, 0.000, -0.002]`，README §5 写的"约 [0.070, 0, 0] m"与实测一致（z 的 -0.002 在"约"字覆盖范围内，可接受）。 |
| 10 | §2.3 常用参数那一整段（platform/device/mode/width/height/fps/odr/stereo_layout） | **PASS** | 这 8 个名字全部在 `gs130_web.launch.py:19-30` 声明，类型与默认值一致。实测的日志行 `640x480 resize fps=30 odr=200` 正是这组默认值。 |
| 11 | §3 从话题消费一帧（自写订阅者） | **UNCLEAR** | README 只给了字段语义与两段转换片段，**没有给一个可运行的订阅示例**。仓库里唯一能抄的是 `ros/test/capture_frame.py`，但 README 从头到尾没有提到这个文件的存在。而且这个文件本身有两个瑕疵：`capture_frame.py:76` 的 `height * 3 // 2 == height * 3 // 2` 恒真（想校验的东西没校验），它也**不校验 `frame_id`**（而 `frame_id` 是实测契约的一部分：`camera`）。用户能不能消费一帧？能，但要自己拼 QoS（RELIABLE + depth 1 已实测可用）。
| 12 | §3 NV12 不是 RGB（`encoding="nv12"`，height 是真实高，len=W*H*3/2） | **PASS（文档层）** | `README.md:74-80` 明写"**这不是 RGB 图**"，给出 `cv2.COLOR_YUV2BGR_NV12` 与 reshape 形状，并说明 640x480 → 460800 字节。拼接切半也给了一行（`README.md:86`）。实测 encoding/step/len 三者与文档完全一致。**启动日志层不 PASS**，见 §3。 |
| 13 | §3 读 IMU `/imu/data` | **PASS** | 实测 203 Hz @ odr=200、`orientation_covariance[0] = -1`（`camera_node.py:284`），README §6 写明了"不提供姿态、协方差为 0 表示未知、实测 IMU 比同批图像晚约 8.6 ms"。这是我上一轮最担心的点，现在文档和实现都到位。 |
| 14 | §3 读标定 `/image_left/camera_info` | **PASS** | README §5 给出来源映射（K 已在输出分辨率、D/distortion_model 的三种情况、R 单位阵、P 由 K 重排）。实测：640x480、equidistant、D 长 4、R 单位阵、P 已填，逐条对上。README 写了"latched"（`:67-68`），对应 `INFO_QOS` 的 `TRANSIENT_LOCAL`（`camera_node.py:36-41`），后启动的订阅者能拿到。 |
| 15 | §5 读外参 TF + `mode:=rect` 的注意事项 | **PASS** | `README.md:118-124`，并明确提示 `rect` 下外参变成虚拟平行双目、参考系改变 —— 这正是我上一轮问的第 8 个问题，答复到位。 |
| 16 | §6 读时间戳语义 | **PASS** | `README.md:126-135`：CLOCK_MONOTONIC、启动时一次性 `offset_ns`、精度不超过一个帧周期、图/IMU 共用 offset 且可互比。实测启动日志里确实有这条 offset 行。**这是全文档质量最高的一节。** |
| 17 | §7 故障排查表 | **UNCLEAR** | 六个现象覆盖了（相机占用 / QoS / 8000 端口孤儿 nginx / raw 分辨率 / 花屏 / Ctrl-C 占用），每行都有处置命令，方向正确。UNCLEAR 的原因是**第 1 行给的自检命令与节点自己打印的命令不一致**：README 写 `ps -ef \| grep -E "mipi_cam\|camera_node"`，节点日志写 `ps -ef \| grep -E "mipi_cam\|gs130"`（`camera_node.py:154`）；两者都会漏掉对方能抓到的进程（`camera_node` 抓不到 `python3 xxx.py` 里跑 gs130 的残留脚本，`gs130` 抓不到已重命名的进程）。现场排障时这两条的差异会真的浪费时间。 |
| 18 | §8 已知边界 | **PASS** | 8 条边界与代码一致，没有一条是"宣称做了其实没做"（我逐条核对了：不做零拷贝 ✓ `camera_node.py:236-255` 只做一次拷贝且注释解释了原因；不做动态参数 ✓ 参数只在 `__init__` 读一次；不做多实例 ✓ 无 ns 参数；stereonet 不接线 ✓）。 |

---

## 2. README 里每一条可复制命令的逐条判定

规则：**能在板上原样跑通** = "verbatim 可跑"；**预期输出描述准确** = "描述准确"。

| 命令（README 行号） | verbatim 可跑？ | 预期输出描述准确？ | 缺的、读者猜不到的步骤 |
|---|---|---|---|
| `cd python && ./build-wheel.sh wheel && sudo pip3 install dist/gs130-*.whl`（`:23`） | 是（cwd=仓库根） | 大致准确：脚本会打印 `Built wheel: ... dist/gs130-0.0.1-py3-none-any.whl`，README 没把这段输出写出来 | ① 起点目录未说明；② `pip3 install` 只装 Python 包，**不装 `libgs130.so.0`**，而 README 依赖表里没有这一行；③ `dist/gs130-*.whl` 的 glob 在 `/bin/sh` 下不展开，只在 bash/zsh 下有效（README 用 bash，可接受，但值得写明） |
| `mkdir -p ~/gs130_ws/src && cp -r ros/gs130_ros ~/gs130_ws/src/`（`:25`） | **否（当前仓库）** | — | `ros/` 未被 git 跟踪。修法：把 `ros/` 纳入版本控制（见 §6.1 B1） |
| `source /opt/tros/humble/setup.bash`（`:26`） | 是 | 无输出即成功，README 没写"无输出是对的" | 无 |
| `cd ~/gs130_ws && colcon build --packages-select gs130_ros`（`:27`） | 是 | README 没给预期输出（`Summary: 1 package finished`） | 首次构建若失败，用户需要 `log/latest_build/gs130_ros/stderr.log`，README 未提（次要） |
| `source install/setup.bash`（`:28`） | 是 | 无输出即成功 | **这一步是必须的，但 README 没有说"每个新终端都要 source 这一行 + `/opt/tros/humble/setup.bash`"**。对 ROS 老手是常识，对"从 SDK 过来的第一次用 ROS 的人"不是 —— 而 `gs130_sdk` 的主要用户恰恰是后者 |
| `ros2 launch gs130_ros gs130_web.launch.py`（`:36`） | 是（实测） | README 只说"然后打开浏览器"，**没有给一条应该看到的日志样例**。UX-2 §1.3 要求过"成功输出形状照抄实现"，这一条没做 | 无（命令本身完整） |
| `ros2 launch gs130_ros gs130_camera.launch.py`（`:46`） | 是 | 未描述 | 无 |
| `ros2 topic hz /image_combine_raw`（`:47`） | 是 | 准确（"约等于 fps"；实测 29.997 @ fps=30） | 无 |
| `ros2 run tf2_ros tf2_echo camera_left camera_right`（`:48`、`:121`） | 是 | 准确（实测平移 `[0.070, 0.000, -0.002]`，README 写约 `[0.070, 0, 0]`） | 无 |
| `ros2 launch gs130_ros gs130_web.launch.py platform:=... device:=... mode:=... width:=... height:=... fps:=... odr:=... stereo_layout:=...`（`:54-56`） | 是（8 个参数都在 launch 里声明了） | 未描述输出 | 无 |
| `bgr = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_NV12)   # frame 形状 (720, 640)`（`:79`） | 否（片段，非可执行） | 描述准确，但**注释写的是 `(720, 640)`，默认参数下拼接线应该是 `(720, 1280)` 的真实形状 `(480*3//2, 1280)`**。这是文档内部的一个真实矛盾：§2.3/§4 的默认输出是 640x480/眼，拼接后宽 1280，而示例按 640 宽写。读者会先困惑一次 | 无（示例性质） |
| `left, right = bgr[:, :width // 2], bgr[:, width // 2:]`（`:86`） | 否（片段） | 准确（实测拼接帧 1280x480、每目 640x480） | 无 |
| `ros2 launch gs130_ros gs130_web.launch.py mode:=raw`（§7 表里提到 raw 需要 1088x1280，未给完整命令） | 不适用 | README 只表里点名"RAW 要求 1088x1280"，**参数表也只写了这一条硬约束**；我上一轮问的"每种 mode 的合法 width/height 组合"至今没有答案 | 需要在 §4 参数表补一行：`resize`/`rect` 下合法尺寸受 SDK 整除约束（`ros/docs/32_integration_expectations.md` F11：`(out_w*h) % out_h == 0` 或 `(mid_w*out_h) % out_w == 0`），错误表现为 `gs130_init` 失败 |

**被 README 漏掉、但读者一定会需要的命令（三条，都不是"过度设计"）：**

1. `ros2 topic list | grep -E "image|imu|camera_info|tf"` —— 确认话题集合（README §3 的表是查的，不是看的）。
2. `ros2 topic echo --once /image_right/camera_info` —— 确认标定不是占位零（我上一轮说"这是我第一个会复制的第二个示例"，README 仍然没有）。
3. `hostname -I` —— 拿板卡 IP（README 写 `<板卡IP>` 占位符）。

---

## 3. 启动日志评审判定

实测日志行（板端）：

```text
calibration: left fx=386.85 fy=387.12 cx=304.62 cy=245.06 equidistant | right fx=... equidistant
static tf from camera_left to camera_right, imu_link (baseline 0.070316 m)
gs130 timestamps are monotonic since boot; using a constant offset of ... ns (accuracy within one frame period)
frames=N imu=M | left_right | 640x480 resize fps=30 odr=200      # 每 5 秒
gs130 camera ready
first frame published
```

**逐问回答：**

1. **"能不能看出它成功了"——基本能，但判据要靠猜。** `gs130 camera ready` 与 `first frame published` 是明确的正信号，且比 UX-2 要求的 `waiting for first frame ...` 更干脆。问题在于**顺序**：日志里 `gs130 camera ready` 出现在首帧之后（`camera_node.py:115` 的 `_wait_for_first_frame()` 在 `:122` 的 `ready` 之前），所以如果只 grep 一次日志尾巴，看到 `first frame published` 时并不知道"还没 ready"。这不是 bug，但值得在 `ready` 行里带上帧数/分辨率。
2. **web URL 与话题清单可发现吗——不可发现。** 整个启动过程**没有任何一行**提到 `http://<board-ip>:8000` 或 channel 0，也没有话题清单。用户必须回头看 README §2.1。UX-2 §1.2 把"Web UI URL ≤3 s 必须出现"列为 blocking，这一条**没有实现**。我认为对 v0.1.0 而言"URL 必须出现在日志里"这条 blocking 定级偏重（launch 文件顶部 docstring 和 README 都写了），但**至少应该在首跑那一条命令的输出里能看见 URL**，否则"一条命令 + 看 README"仍是两步。
3. **NV12 不是 RGB 这个陷阱，暴露得够早吗——不够。** 启动日志里**没有任何一行**提到 `nv12` 或"不是 RGB"。UX-2 §5.2 指定的安放位置是"topic 行内打印 `(nv12)`"。当前用户看到 `first frame published` 后的第一个自然动作是 `ros2 topic echo --once` 或自己写订阅者，此时 README §3 的警告在文档里（位置正确、措辞清楚），**但不在他正在看的终端里**。从实测看，最坏后果是花屏/偏色而不是崩溃，所以我不认为这是"发布阻断"，但它是一个真实的、可以用一行日志消掉的坑。

**我要求改动/新增的具体行（只改已有行，或补真实缺失的行）：**

| 类型 | 现在 | 改成 / 新增 |
|---|---|---|
| 改已有行（`camera_node.py:122`） | `gs130 camera ready` | `gs130 camera ready: /image_combine_raw nv12 1280x480 (not RGB; step=1280) + /imu/data + /image_left|camera_right/camera_info` |
| 新增（`_start()` 开头，publishers 建好之后） | —（无） | `gs130 studio: open http://<board-ip>:8000/ and pick channel 0 (web launch only; use 'hostname -I' for the IP)` |
| 新增（`_wait_for_first_frame()` 进循环前） | —（无，且循环期间完全静默） | `waiting for first frame (up to %.1f s) ...` |
| 改已有行（`_report()`，`:320-324`） | `frames=N imu=M \| left_right \| 640x480 resize fps=30 odr=200` | `frames=N (+%.1f fps) imu=M (+%.1f Hz) \| left_right \| 640x480 resize fps=30 odr=200` —— 现在是**累计计数**，不能当速率看；要判断"IMU 到底有没有 200 Hz"用户必须去敲 `ros2 topic hz /imu/data`，而这是本包最容易被误读的一个量（上一轮 E3） |
| 改已有行（`_open_device()`，`:151-156`） | `... with 'ps -ef \| grep -E "mipi_cam\|gs130"'` | 拆成两行，且命令去掉内层引号以便复制：`... next: stop the other stack, then relaunch` / `  pkill -f mipi_cam` + `  ps -ef \| grep -E mipi_cam\|camera_node\|gs130` |

另外一条一致性缺陷（不新增行、但必须修）：`frame_id_camera` 可参数化，但只有**TF 父坐标系**和**左目图像**用它，右目图像固定 `"camera_right"`（`camera_node.py:176, 231`）。用户把 `frame_id_camera:=foo` 之后，`/image_left_raw` 的 `frame_id` 变成 `foo`、右目仍是 `camera_right`、TF 父坐标系也是 `foo`，三者不再自洽。要么把右目也接到同一个参数（`frame_id_camera_right`），要么在 README 里禁止改这个参数。

---

## 4. 五类失败体验的判定

判定标准：**只看包本身（日志 + README + 退出码）能否定位到原因**。

### 4.1 相机被占用 —— 可诊断，但"下一步"不到位

- 代码路径：`gs130.Device(config)` 抛 `GS130Error` → `_open_device()` → `fail()` → `FATAL` + `SystemExit(2)`（`camera_node.py:148-165`）。
- 优点：**启动期就失败，不卡在等首帧**；文案点出了真因（"The camera is exclusive"）并指向 `mipi_cam` / 另一个 node / 残留脚本；退出码是 2 而不是 0（`main()` 返回 `error.code`，入口点 `sys.exit()` 转发，已用本地探针验证）。
- 缺点：① 没有占用者 PID（UX-2 §4.3 明确要求 `fuser -v /dev/video0` 或 `/proc/*/fd`）；② 没有可复制的释放命令（`pkill -f mipi_cam`）；③ 给的是 `ps -ef | grep ...` 而不是 `fuser`；④ `ros2 launch` 的退出码是 1（节点退出码被 launch 吞掉），用户如果只看 `echo $?` 会以为是 launch 自身的问题；⑤ 日志用 `get_logger().fatal()`，输出到 stdout 还是 stderr 取决于 rclpy 配置，UX-2 要求"全部走 stderr"未验证。
- **最小修复**：日志补两行（`  fuser -v /dev/video0` 与 `  pkill -f mipi_cam`），并把 README §7 第一行的自检命令与节点文案统一。

### 4.2 缺 `gs130` Python 包 —— 完全不可诊断

- 代码路径：`camera_node.py:14` 的 `import gs130` 在模块导入期执行。缺包 → `ModuleNotFoundError` traceback → rclpy 尚未 init、无 ROS 日志、无 `fail()` 文案、无退出码约定。
- 用户看到的是 Python 栈，而不是任何一句"请先装 gs130 Python 包"。README §1 依赖表里写了这一条，所以**能自己找到答案，但只能靠回头翻文档**，属于"从包本身诊断不出"。
- **最小修复**（3 行）：

```python
try:
    import gs130
except ImportError as error:
    print("gs130_ros: the gs130 Python package is missing: %s" % error, file=sys.stderr)
    print("gs130_ros: build and install it from the SDK: "
          "cd python && ./build-wheel.sh wheel && sudo pip3 install dist/gs130-*.whl", file=sys.stderr)
    sys.exit(2)
```

### 4.3 libgs130 共享库缺失（边界情况） —— 基本可诊断

- `_open_device()` 捕获 `OSError` → `fail("libgs130 could not be loaded: %s")`（`camera_node.py:157-158`）。真因来自 `_runtime._find_library()` 的文案（"install libgs130.so.0 or set GS130_LIB"），会随 `%s` 一起打出来，**已经点了 `GS130_LIB`**。
- 缺点：没有 deb 安装命令（UX-2 §4.1 要求 `sudo dpkg -i gs130_...deb`），README 依赖表也没提这个库。**可接受的 v0.1.0 缺口**。

### 4.4 错误的 mode / 分辨率 —— 可诊断，且是最规范的一类

- `_validate()` 在**打开设备之前**跑（`camera_node.py:91-92`），`mode:=rgb` → `mode must be one of raw, resize, rect, got 'rgb'`；`mode:=raw width:=640` → `mode raw requires width=1088 height=1280, got 640x480`；`fps:=0` → `fps must be at least 1, got 0`。全部 `FATAL` + `SystemExit(2)`。
- 我实机验证过 `return 2` 会真的变成进程退出码 2（本地探针），所以"非零退出"这条是真的成立，不是纸面承诺。
- 缺点：① 文案没有"替代命令"（UX-2 §2.4 要求 `use mode:=resize ...`），照抄 UX-2 的模板加一行即可；② **非数值参数会裸抛 `ValueError`**：`width:=abc` 时 `int(self.get_parameter("width").value)`（`camera_node.py:80`）直接崩 traceback，没有 `fail()` 文案 —— 这是真实的体验缺口，和 §4.2 一样属于"输入错误 → Python 栈"；③ `top_bottom`/`bottom_top` 在 640x480 下是否满足 SDK 的整除约束，参数表没有给合法组合表，用户只能靠 `gs130_init` 失败来发现。
- **最小修复**：把 `int()`/`float()` 包一层，异常走 `fail()`；README §4 补合法尺寸组合表（至少写清"resize/rect 的宽高受 SDK 整除约束，推荐成对使用 640x480 / 1280x720"）。

### 4.5 超时无帧（10 s） —— 文案到位，过程静默，且有一条竞态

- 文案不差：`no frame within 10.0 s; check the camera and the IMU FSYNC wiring`（`camera_node.py:308-309`），点了 FSYNC 这个真因，`SystemExit(2)` 且 `self.device` 已经赋值所以**相机会被释放**（`main()` 的 `finally` → `shutdown()`）。
- 缺点 ①：**10 秒内没有任何进度输出**，UX-2 §1.2 明确要求 `waiting for first frame (n/10 s)`，且把"10 s 硬上限+必须有进度行"列为 blocking。这是 0.1.0 应该补的一行。
- 缺点 ②（**真实 bug，必须修**）：`_start()` 的步骤顺序是"建 publisher → 发标定 → 发 TF → 等首帧 → 建定时器"，而 `main()` 的清理依赖 `node is not None`：

```python
node = Gs130Camera()      # 若这里抛异常，node 仍是 None
...
finally:
    if node is not None:
        node.shutdown()   # ← 不会被调用
```

`self.device = self._open_device()`（`:92`）成功之后，如果 `_start()` 期间失败（等首帧超时 **或** 用户在启动窗口内按 Ctrl-C），`Gs130Camera.__init__` 里的 `except BaseException: self.shutdown()` 会兜住（`:93-97`），所以这条路径**是安全的**。但**恰好在 `_open_device()` 内部失败**（`device.start()` 抛 `GS130Error`，即相机已 open/init 但 `start()` 失败）时，`self.device` 从未被赋值，设备只在 `Device.__del__()` 里被回收，而 `__del__` 在解释器关停期是否执行是不保证的 —— 表现就是"偶发启动失败后相机被占，必须重启才能恢复"。这个窗口很窄，但正是我一整天都在重启节点的场景。**最小修复**：把 `except BaseException` 的兜底从 `_start()` 扩到整个构造过程，或让 `_open_device()` 在 `start()` 失败时自己 `close()`。

### 4.6 启动过程中 Ctrl-C —— 与 UX-2 §4.6 的约定不符

- 正常路径（首帧之后 Ctrl-C）：实测 `camera released`，无残留，退出码 0，**PASS**。
- 启动窗口内 Ctrl-C：见 §4.5 缺点 ②，先走 `shutdown()` 再传播 `KeyboardInterrupt`，`main()` 里 `node is None` → 不重复释放，行为正确。
- 不符约定的一条：`shutdown()` 把 SIGINT 设为 `SIG_IGN`（`camera_node.py:335`）并在 `finally` 里恢复（`:348`）。UX-2 §4.6 要求"第二次 Ctrl-C 立即强杀，文案 `forcing exit, camera may stay busy for a moment`"。当前实现是：清理期间第二次 Ctrl-C 被忽略，清理结束就退出，所以**不会出现"击键无效"的观感**（清理通常很快），但没有那条文案，也没有刻意处理第二次 Ctrl-C。我判定这是**文档层面的小缺口**：README §7 应该写清"第二次 Ctrl-C 可能被忽略一小会儿，别用 `kill -9`"，因为 README 现在只在最后一格写了"不要 kill -9 初始化过程中的进程"却没解释为什么。
- 另外 README 完全没提 **SIGKILL 之后的行为**（我上一轮问了、UX-2 §5.1 也列了）。这是文档题，不是代码题。

---

## 5. 第一轮反馈（`30`）与可用性评审（`31`）逐条对照现实

### 5.1 被honoured的（我逐条核对过实现，不是只看文档）

| 条目 | 证据 | 判定 |
|---|---|---|
| E1 零新增节点接入 TROS 链路 | `gs130_web.launch.py:61-86` 只 Include `hobot_codec` 与 `websocket` 的既有 launch；实测 HTTP 200 + channel 0 有图 | **完全兑现** |
| E2 独占冲突显式失败（不静默挂起） | `_open_device()` 的 `GS130Error` → `fail()` → `SystemExit(2)`，文案点明 mipi_cam | **兑现，文案还需补 PID/命令** |
| E3 IMU 独立速率 | `_poll_imu()` 独立定时器（`1/max(2*odr,20)` ≈ 2.5 ms），每包用自己的 `timestamp_ns`；实测 203 Hz @ odr=200 | **完全兑现**（这是我上一轮最严重的一条，修得很好） |
| E4 诚实的 fisheye / camera_info | `calibration.py:11-24`：全零→`plumb_bob`(5 零)、FISHEYE→`equidistant`(4 系数)、PINHOLE→`rational_polynomial`(8 系数)；实测 equidistant + D 长 4 | **完全兑现** |
| E4 外参静态 TF | `_publish_transforms()`（`camera_node.py:187-212`），实测 `camera_left→camera_right` 与 `imu_link` 均在 `/tf_static` | **兑现**（UX-2 §7.1 曾建议首版不发 TF，实现选择发，我认为是更好的选择，但 `32` F12 提示 `rect` 模式会改写外参，README 已用一句话提示参考系变化） |
| E5 干净生命周期 | `shutdown()` 直接 `stop()`+`close()`，不依赖 GC；实测 `camera released`、无残留 | **兑现** |
| E5 QoS 与 hobot_codec 匹配 | `IMAGE_QOS` RELIABLE（`camera_node.py:24-29`），与 `00_verified_platform_facts.md` F5 的 `PUB_QUEUE_NUM=5` RELIABLE 订阅端匹配 | **兑现** |
| 时间戳时钟域写清楚 | README §6 + 启动日志 offset 行 | **完全兑现** |
| 4.1 `read_image()` 的 dict 形态 → 模式/话题对照表 | README §3 表格里写明了 `stereo_layout != none` / `:= none` 各自的话题 | **兑现** |
| 4.5 不伪造 orientation | `orientation.w = 1.0` + `orientation_covariance[0] = -1` + README 说明 | **兑现** |
| 4.6 忙等不得照抄 | 节点用受控定时器（`create_timer`），无 `time.sleep` 轮询（除首帧等待的 5 ms 循环，可接受） | **兑现** |
| 5.1 三个过度设计一律没做 | 无自定义 msg、无深度/点云、无动态参数（README §4 末尾明写"运行期不支持动态重配置"） | **完全兑现** |
| N1 拼接作为可选开关 | `stereo_layout` 参数 + SDK 硬件拼接（`camera_node.py:147`），README §3 写了 left_right 语义与切半方法 | **兑现**（实测 1280x480 立体对，工程上比 Python 侧手工拼更对） |
| 3.1 沿用 `/image_combine_raw` 并说明它是既有名字 | README §3 表 + §2.1 链路说明 | **兑现** |

### 5.2 未被honoured的，以及我的判级

| 条目 | 现状 | 影响 v0.1.0？ |
|---|---|---|
| **UX-2 #5 / UX-1 3.2：`ns` 命名空间（默认 `gs130`）** | 未实现。话题是裸名 `/image_combine_raw`、`/imu/data`、`/image_left/camera_info`。README §3 如实写了裸名，没有撒谎 | **不阻断。** 裸名是和 D-Robotics 例子对齐的刻意选择（`31` §3.1 的裁决也承认 remap 成本 > 语义洁癖），而且 `camera_info` 与图像同基名的约定在 `camera_left`/`camera_right` 这一层是满足的。但"多相机系统下 `imu/data` 必然冲突"是真实的：**判为可接受的 v0.1.0 范围决定**，只需在 README §8 补一句"不支持多实例，`/imu/data` 是全局名" |
| **UX-1 4.1：`/image_left_raw` + `/image_left/camera_info` 的配对约定** | 图像是 `_raw` 后缀，`camera_info` 在 `camera_left/` 下，两者**不同基名不同目录** | **不阻断但值得改文档。** 后果是实现细节：`image_proc` / `depth_image_proc` 的自动配对（`.../image_raw` + `.../camera_info`）用不上，用户需要显式 remap。README 应把这个事实写出来（"本包图像话题名沿用 TROS 例子的 `_raw` 后缀，`camera_info` 在 `/image_left|camera_right/` 下，标准工具链需手动 remap"），因为它是"看起来能用其实配不上对"的典型 |
| **UX-2 §1.2/§1.3：启动日志规格（首行参数回显、URL ≤3 s、`first frame ok` 行含分辨率与时间戳、逐行 ≤96 字符带节点前缀）** | 未实现，见 §3 | **不阻断，但要求补 §3 表格里的 3 行**（nv12/分辨率行、URL 行、进度行）。理由：日志里加三行几乎零成本，收益是"一条命令 + 看终端"就能自证成功 |
| **UX-2 §4 六类固定失败文案** | 只实现了参数校验类（`_validate`）与相机占用类（`fail()` 内联文案）；"设备未检测到""库未找到""超时"的"原因+可复制命令"格式不完整 | **不阻断。** 相机占用与无帧这两类**已经能定位**（§4.1/§4.5），缺的是"更贴心"。列为应当补，不作为发布阻断 |
| **UX-2 §5.1 六份文档** | 只交付了一份 `ros/README.md`（无 `01_install` / `02_topics` / `03_launch_options` / `04_troubleshooting` / `05_view_in_browser`） | **不阻断，判为合理的范围决定。** README 已把六份文档的实质内容压进九节，且质量高于预期（§6 时间戳、§5 标定两节尤其好）。UX-2 自己也写了"内容重复比缺文档更伤可用性"。**唯一我不同意的删减是"浏览器看图"没有独立成节**：首跑目标就是看图，而 README 只有两行（IP 怎么来、孤儿 nginx 怎么清都缺）——建议在 README 内补齐，不必新建文件 |
| **UX-2 §5.3 两段"照抄"文字（Encoding / IMU pacing）** | Encoding 段的实质内容在 README §3（措辞不同但更清楚）；IMU pacing 的实质内容在 §4 参数表 + §6（`odr` 与实测 203 Hz、8.6 ms），但**没有写"每帧图像对应多少个 IMU 包"这个 UX-2 专门点名的量** | **不阻断。** 加一句 `200/30 ≈ 6.7 包/帧` 即可 |
| **UX-1 4.3.4：`is_fsync` 的含义与不映射到 ROS 字段** | 未文档化。IMU 消息里确实没有它（`_imu_message` 只填 accel/gyro），但用户读 `_types.ImuPacket` 时会问 | **不阻断。** 一行文档题 |
| **UX-1 4.4.2：`timestamp_source` 参数（`hardware`/`node_clock`/`hardware_with_offset`）** | 未实现，只有一种行为（offset 换算）。README §6 写清了唯一行为与误差 | **不阻断，反而是好事。** 单一行为 + 诚实文档比我上一轮要的三态参数更好；这是"采纳了问题、拒绝了方案"的正确做法 |
| **UX-1 3.4 / E5：QoS 与 FIFO 丢弃策略可配** | 硬编码 RELIABLE/depth 1（图像）与 depth 200（IMU），SDK FIFO 的 `DROP_OLD` 完全没暴露 | **不阻断。** 实测链路正常，且"默认值必须对得上订阅端"这条比"可配"重要。但 README §4 参数表应补一句"图像 QoS 固定 RELIABLE depth 1，IMU depth 200，不可配，原因见 §7"——现在 README 只在 §7 表里间接提到 RELIABLE |
| **UX-1 4.6：CameraIndex.RIGHT=0 / LEFT=1 的说明** | README 未提 | **不阻断。** ROS 侧全部用字符串参数，用户不会遇到这个整数陷阱 |
| **UX-1 6.3：从零到看见画面的"一行一条"完整序列** | README §1+§2.1 合起来就是这条序列，但被 §1 的"依赖表 + 安装块"打断 | **不阻断。** 建议把 §2 的快速开始提到 §1 之前（先给能跑的两条命令，再解释依赖）——这是排版问题，不是内容问题 |
| **UX-2 §7.2：`ros2 launch -s` 可直接列参数、`ui:=false` 的独立 launch** | `ros2 launch gs130_ros gs130_web.launch.py -s` 可用（仅 web launch 会打印 12 个参数）；没有独立 `gs130_topics_only.launch.py`，但 `gs130_camera.launch.py` 已覆盖该用途 | **不阻断。** 命名不同、能力齐备 |
| **UX-2 §7.2：`VERSION` 联动** | `gs130_ros/setup.py:7` 硬编码 `0.1.0`，SDK `VERSION=0.0.1` | **不阻断。** 但值得在 README 里写明两者版本号的关系（现在读者会以为是同一个数） |
| **UX-2 §7.2：`probe_nv12_publisher.py` 移到 `ros/tools/`** | 仍在 `ros/` 顶层（docstring 已声明"不是交付物的一部分"） | **不阻断。** 文件名带 `probe` 且有声明，够了 |
| **README 顶部 `status: experimental, interfaces may change`** | 没有 | **不阻断。** SDK 的 `core/include/gs130.h` 有同类免责风格，建议一句 |
| **`camera_name` 未填** | `calibration.py:camera_info()` 没设 `camera_name`（`camera_left`/`camera_right` 是 frame_id 而非 camera_name） | **不阻断。** 标准做法是填同名，但空值不会破坏任何工具链；列入文档说明即可 |

### 5.3 一个必须记下的"文档内部矛盾"

README 的可复制片段之间不一致（不影响运行，但会被用户当场发现）：

1. `README.md:75` 的 `height=480`、`data` 长度 `460800` 是**单目 640x480** 的说法，而 §3 表里 `/image_combine_raw` 是**拼接帧**（实测 1280x480 / 921600 字节）。同一节里 `:79` 的注释又写 `frame 形状 (720, 640)`。三处数字分别对应"单目""拼接""又一个单目"，读者必须自己拼出"拼接帧先切半再 reshape"。建议这一节改成两句：拼接帧 `(720, 1280)` / 921600 字节；切半后每目 `(720, 640)` / 460800 字节。
2. 节点文案 `grep -E "mipi_cam|gs130"` 与 README §7 的 `grep -E "mipi_cam|camera_node"` 不一致（见 §1 #17）。

---

## 6. 我会拒绝发布的东西 / 我会接受的文档化限制

前提声明：**过度设计是不可接受的**。凡是我列在 §6.2 的，我明确表态"0.1.0 不做是对的，只要写进文档"。

### 6.1 我会拒绝带着发布的三项

**B1. `ros/` 不在版本控制里（对"发给另一个团队"是阻断项）**

事实：`git ls-files ros/` 为空，`git status --short` 只有 ` M .gitignore` 与 `?? ros/`。任何别的团队 `git clone` 后**一条 README 命令都跑不成** —— 不是"某台机器不行"，是"这份交付物不存在于版本库"。这不是代码缺陷，是交付流程缺陷，但它的后果比任何代码缺陷都严重。

最小修复：把 `ros/`（含 `docs/`）纳入跟踪并提交；`.gitignore` 里 `ros/build/ install/ log/` 的规则已经写好，不会把构建产物带进来。

**B2. `_open_device()` 内 `start()` 失败时的清理窗口（`camera_node.py:92, 159-160`）**

`self.device` 只在 `_open_device()` 正常返回后赋值；`device.start()` 抛异常时 `close()` 无人负责，只能指望 `Device.__del__()`，而 `main()` 的 `finally` 因为 `node is None` 也不会兜。对一个**显式标记为独占资源**的设备，这是唯一一条"失败后可能需要重启才能恢复"的路径（UX-1 §E5 与我上一轮的门槛都点了这条）。窗口窄，但我一天重启几十次，窄窗口迟早命中。

最小修复（二选一）：`_open_device()` 里 `try: device.start() except BaseException: device.close(); raise`；或把 `_open_device()` 的结果先存到一个局部变量、失败时在 `__init__` 的 `except` 里关掉。

**B3. 输入错误的处理缺口（`width:=abc` 裸 traceback + 缺 `import gs130` 文案）**

`camera_node.py:80-89` 的 `int()`/`float()` 转换和 `:14` 的顶层 `import gs130`，都会在参数错误/缺包时给用户一个 Python 栈。前者是"用户输错一个字符"，后者是"新用户第一步就会踩"。两者加起来 10 行代码，收益是"错误信息可读"。我把它列为发布阻断，理由是：**一个包连自己最常见的两种安装/输入错误的诊断都不给，就不该叫 v0.1.0**。

### 6.2 我会接受、但要求写进文档的限制

| 限制 | 我的表态 |
|---|---|
| 无 `ns` 命名空间、`/imu/data` 是全局名 | **接受。** 与 TROS 例子对齐的价值大于命名洁癖；README §8 加一句"不支持多实例"即可 |
| QoS 与 SDK FIFO 不可配（固定 RELIABLE / depth 1 / depth 200） | **接受。** 实测链路正常，可配是调优期的需求，不是首版需求。README 需补一句"QoS 固定，原因见 §7" |
| 无 `timestamp_source` 参数 | **接受，且比我要的方案好。** 单一行为 + 明确文档胜过三态参数 |
| 无 `imu_only` 模式（不占相机只发 IMU） | **接受。** 代码里没有半残开关，README 也没有宣称；`publish_imu:=false` 是反向能力，够了 |
| 无零拷贝（`hobot_shm`） | **接受。** README §8 明写了"不做零拷贝"并解释了原因。UX-1 把它列为 nice-to-have 是对的 |
| `hobot_stereonet` 不接线 | **接受。** README §8 最后一条把"需要 top_bottom 布局 + 非零 P + mipi_cam 抢占"三件事都点了，v0.1.0 不接线是正确的范围决定 |
| 一份 README 代替六份文档 | **接受**（§5.2 已述），但"浏览器看图"的信息（板卡 IP、channel 0、孤儿 nginx）要在 README 内补齐 |
| 无 `camera_name`、无 `is_fsync` 文档、无 SIGKILL 行为说明、无"每帧多少 IMU 包"、无 per-mode 合法尺寸表 | **全部接受为文档欠账**，逐条都是 1–3 行文字。不做才是问题 |

### 6.3 我明确不要求的东西（防止被过度设计）

- 不要自定义 `.msg`（`sensor_msgs` 够用）。
- 不要 Python 侧手工拼接（SDK 硬件拼接已实测可用，手工拼只会引入 `top_bottom` 平面顺序的坑）。
- 不要为 v0.1.0 加 `ros2 param set` 动态重配（分辨率/布局在 `gs130_init` 定死，热切换是可靠性陷阱）。
- 不要为 `imu_only` / 零拷贝 / 多相机实例预留"看起来能开的开关"。留半个开关比不做更糟。
- 不要把 `ros/test/capture_frame.py` 提升成"官方示例"并原样发布 —— 它现在连 `frame_id` 都不校验、`contract` 那行是恒真表达式。要么修好它并在 README 里引用，要么明确标注"这是内部验收脚本"。

---

## 7. 最终采纳结论

**今天要不要发给另一个团队？—— 不要，但差得不远：三处修完即可发。**

具体理由：方向、架构、接口选择全部正确，最难的几件事（QoS、NV12 的 `height` 语义、时间戳 offset、fisheye 的诚实映射、IMU 独立速率与时间戳、干净释放、复用 TROS 链路）**都已按实测口径做实**，并且有一条我特别看重的质量信号：`00_verified_platform_facts.md` 里 E1–E7 的每条结论都能在实现里找到对应代码，没有"文档写了但代码没做"的情况。我上一轮列的三条交付不足（独占失败、camera_info 真实性、IMU/时间戳）现在都不成立。

但"发给别人"意味着对面只有 `git clone` + README，所以：

**发布的两个条件：**

1. **`ros/` 进版本控制，并且让一个没参与开发的人在干净容器/另一块板子上只按 README 走一遍**（`git clone` → §1 四段 → §2.1 → 浏览器看图 → `ros2 topic hz` → Ctrl-C）。这条不是为了形式，是因为 B1 的存在说明"按 README 走一遍"这件事本身没有被当作交付物验证过。
2. **修掉 §6.1 的 B2（`start()` 失败时的释放）与 B3（参数/缺包的可读错误）**，并补上 §3 里那三行启动日志（nv12+分辨率、web URL、等首帧进度）。前者是可靠性，后者是"用户能不能自己确认成功"。

**作为条件 2 的一部分，我会同时要求的两条最小文档改动：** ① README §7 第一行与节点文案的命令统一（`mipi_cam|camera_node|gs130` 三者都抓）并给出 `fuser -v /dev/video0` 与 `pkill -f mipi_cam`；② README §8 补三句限制说明（无 `ns`/多实例、QoS 不可配、图像话题名与 `camera_info` 目录不同基名需手动 remap 才能被 `image_proc` 自动配对）。

**不构成发布条件、但我强烈建议在 0.1.1 做的**：`_report()` 打印区间速率而不是累计计数（`frames=N (+29.9 fps) imu=M (+203.1 Hz)`）；`capture_frame.py` 修掉恒真表达式并加 `frame_id` 校验；`frame_id_camera` 与右目 frame_id 的自洽性。

**一句话**：这套东西我**愿意用**，但今天**不愿意替你们向别人担保**——因为担保的第一步（`git clone` 就能拿到 `ros/`）现在还不成立。把 §6.1 三项修掉、补三行日志，我就签字发出去。
