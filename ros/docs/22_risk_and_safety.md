# 22 风险与安全 — GS130 ROS 2 (TROS) 接口

| 项 | 值 |
| --- | --- |
| 项目 | gs130_sdk / ROS 2 接口包 `gs130_ros` |
| 文档 | Risk & Safety / 资源所有权分析（QA-3） |
| 目标平台 | RDK X5 + TROS humble（`/opt/tros/humble`），共享开发板，真机相机已连接 |
| 基线 | gs130_sdk 0.0.1；`python/gs130/`（只读消费，本文件不修改它） |
| 上游契约 | `01_requirements_review.md`、`10_architecture.md`、`11_interface_freeze.md`、`20_test_plan.md` |
| 前提 | 本文**不重新验证**板卡事实；本文不要求任何人 ssh 或提前跑相机。所有命令均为可在板卡终端直接粘贴的命令级步骤 |

> **本文件的定位**：`11_interface_freeze.md` 定义"发什么"，本文件定义"**谁拥有硬件与内存、出事怎么收场**"。冲突时以本文件的操作安全规则为准（它涉及板卡存活），接口语义以 freeze 为准。

---

## 0. 十条不可协商的规则（先读这一节）

| # | 规则 | 违反的后果 |
| --- | --- | --- |
| M1 | **同一时刻全板只能有一个进程持有 GS130。** 本节点、`mipi_cam`、`gs130_probe`、`gs130-run`、`gs130-rec`、任何 Python 脚本都算"持有者" | 初始化失败、ISP 状态错乱、之前出现过的"卡死相机" |
| M2 | **绝不用 `kill -9` 打断相机初始化 / 开流过程**（`gs130_init` → `gs130_start` → 首帧之间） | 传感器停在未定义状态，需要硬件级复位才能恢复 |
| M3 | **重启（`sudo reboot`）永远是最后手段**，且必须作为"记入日志的失败"上报，不得默认为第一反应 | 共享板上他人工作中断；且掩盖了真正的资源泄漏根因 |
| M4 | **任何时候只允许一个进程以 `./sbin/nginx -p .` 运行**（端口 8000） | 第二个 nginx 端口冲突；launch 报错或 web 页面黑屏 |
| M5 | **ROS 消息的 `data` 必须拥有自己的内存**：在发布返回前，产生它的 numpy 视图必须保持存活；发布返回后显式释放 | 悬垂指针 → `hobot_codec` 编码到已 free 的内存 → 花屏 / 段错误 |
| M6 | **不得依赖 GC / `__del__` / `weakref.finalize` 的时机来释放相机与缓冲** | 相机不释放、进程退出时未 deinit、缓冲释放时机不确定 |
| M7 | **运行期间浏览器页面不得无人看守地长期打开**；测试结束必须关页面、停 websocket、核对相机已释放 | nginx / websocket 残留进程 + 相机被持续占用 |
| M8 | **每次运行必须可被 `timeout` 限定**（默认 `timeout 300`），不得放后台过夜 | 无人看守的长时间占用，出问题时无人知情 |
| M9 | **不得写入任何 GPIO / I2C / sysfs**（`/sys/class/gpio/*`、`i2cset`）来"救"相机；硬件复位只能通过 SDK 的正常 `close()` 路径完成 | 与 SDK 的 sensor 电源时序冲突，可能永久损伤传感器状态 |
| M10 | **任何测试前的第一步不是启动，而是确认相机空闲**（§8/§9 检查清单） | 90% 的"起不来"都是上一次运行的残留 |

---

## 1. 风险总览（按"最坏后果 × 发生概率"排序）

| ID | 风险 | 概率 | 影响 | 检测手段 | 缓解 | 免重启恢复 | 残余 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| H-1 | 相机被上一个进程占用，`gs130_init` 失败且原因不明 | 高 | 中 | §3.3 诊断命令 | 启动前自检 + 单一持有者规则 | §3.4 R-1/R-2 | R2 |
| H-2 | 开流中途被 `kill`，传感器停在未定义态 | 中 | 高 | 是否出现"初始化中"退出记录 | M2 + 宽限信号序列 | §3.4 R-3 | R2 |
| H-3 | ISP / VPU 全局资源未释放 | 低 | 高 | 重启后同一进程立即失败 | 只走正常 `close()`；不并发持有 | §3.4 R-4 | R1 |
| H-4 | RAW 模式输出分辨率 ≠ 1088×1280 → `PARAM_ERROR` | 中 | 中 | 启动日志 / `ERROR` 码 | 参数白名单，交给 `Config.preset` | 改参数重启节点 | — |
| P-1 | `fps=0` → C 侧 `1000000000ULL / fps` 整数除零 → 相机线程异常 | 低 | 高（进程崩溃） | 启动日志；`fps` 参数校验 | 节点侧禁止 `fps<=0` 进入 SDK | 重启节点 | R3 |
| P-2 | 相机线程"热自旋"（`get_frame` 立刻返回非 OK，`goto free_frame` 无休眠）→ 每循环 malloc/free 2 MiB，CPU 打满 | 低 | 高（板卡变卡、风控） | CPU 占用 100% 且 `ros2 topic hz` 为 0 | `timeout` + CPU 监控 + 立即 `SIGINT` | §3.4 R-3 | R3 |
| P-3 | IMU 线程 latch `HW_ERROR`（`imu->full()` 或 cache > `kMaxImuCache`） | 低 | 中 | 节点日志 `HW_ERROR` | 只允许 `imu_fifo.depth=1024` 预设，不改小 | `deinit`+`init` | — |
| P-4 | `hbn_vnode_getframe` 尺寸与 `output_w/h` 不一致 → `PARAM_ERROR` | 低 | 中 | 日志 + 无帧 | 参数白名单；`mode=rect` 时单独验证 | 重启节点 | — |
| M-1 | 泄漏（每帧 2 MiB × 2） | 中 | 高 | RSS 监控（§5.4） | M5/M6 + 只在 `OK` 时才认为拿到 buffer | 重启节点 | — |
| M-2 | double free | 低 | 高（进程崩溃） | 崩溃回溯；`ErrorCode` 序列异常 | "一次 `OK` 只 free 一次"规则（§5.3） | 重启节点 | — |
| M-3 | use-after-free | 中 | 高（花屏/崩溃） | 花屏随机出现、`dmesg` 段错误 | M5 生命周期规则 | 重启节点 | — |
| R-1 | QoS 不匹配（我们 BEST_EFFORT，订阅者 RELIABLE） | 中 | 中（web 黑屏） | `ros2 topic info -v` 的匹配数 | 与 freeze §2.2 完全一致的 QoS | 改 QoS 重启 | — |
| R-2 | 慢订阅者导致队列/内存增长 | 中 | 中 | RSS、`ros2 topic hz`、丢帧计数 | image depth=1，imu depth=200 | 调 depth | — |
| R-3 | 时间戳时钟域判断错 | 中 | 中（下游静默错位） | 启动自检（§4.4） | 一次性 `offset_ns` 策略 | 改参数重启 | R1 |
| L-1 | 两个 launch 同时起，nginx/codec 重复 | 中 | 中 | `ps ax` 检查 | 启动前 `pgrep` 互斥检查 | §6.3 清理 | — |
| L-2 | 误启动 `mipi_cam` 抢相机 | 低 | 高 | 启动日志 | 不 include 任何含 `mipi_cam` 的 launch；只用 `Node(package=..., executable=...)` | §6.3 | R2 |
| L-3 | `hobot_codec.launch.py` 内部的 `hobot_shm` 被重复引入 | 中 | 低 | launch 日志 | 只用 `hobot_codec_encode.launch.py`，或自行起 `hobot_codec_republish` 节点 | 重启 launch | — |

---

## 2. 硬件与平台风险（逐项：描述 / 概率 / 影响 / 检测 / 缓解 / 免重启恢复）

### H-1 相机独占资源被占用

| 项 | 内容 |
| --- | --- |
| 描述 | GS130 的两颗 MIPI sensor（I2C bus 4 @ `0x30`、bus 6 @ `0x32`，reset GPIO 351/353）由 SDK 通过 `/dev` 层的 D-Robotics camera 句柄 + `hbn_vflow` 持有。同一时刻只允许一个进程执行 `gs130_init`+`gs130_start`。 |
| 概率 | **高**。共享板上任何一次遗留的 `gs130-run`、`python -c`、`mipi_cam`、上一轮测试的僵尸节点都会造成 |
| 影响 | 中：本节点初始化失败（`NOT_FOUND` / `HW_ERROR` / `PARAM_ERROR`），不损坏硬件，但会浪费一轮测试并诱发"重启试试"的错误动作 |
| 检测 | §3.3 的四条命令：进程扫描、`/proc/*/fd` 设备占用、`fuser`、只读探测工具 |
| 缓解 | ① M1 单一持有者；② 节点启动时做"空闲自检"并在日志打固定关键词 `camera busy`（freeze §7 的错误契约）；③ 每次测试结束执行 §9 后置清单 |
| 免重启恢复 | **R-1**（礼貌停止）→ **R-2**（强制收尾）。见 §3.4。**不需要重启**：`hbn_vflow_destroy()` + `hbn_camera_destroy()` 在持有者进程退出时由 SDK 路径执行，内核随之回收；随后 `sensor_power(gpio, 1)` 会把 sensor 恢复到可探测状态 |

### H-2 开流中途被 kill（最危险的操作）

| 项 | 内容 |
| --- | --- |
| 描述 | 在 `gs130_init`（EEPROM/IMU 探测、sensor 上电、`hbn_camera_attach_to_vin`）与 `gs130_start`（`hbn_vflow_start`、IMU FSYNC 握手）之间强杀进程，sensor / ISP / MIPI RX 可能停在"已上电但未成流"的半途状态。"卡死相机要求重启"就是这一条的历史实例 |
| 概率 | 中（默认用 ctrl-c 时是低，但开发中"起不来就 `kill -9`"很常见） |
| 影响 | **高**：可能需要硬件级恢复，且恢复过程中相机不可用 |
| 检测 | 进程退出后：`pgrep -af gs130` 为空 **但** §3.3 的只读探测仍不通过 → 判为 H-2 而非 H-1 |
| 缓解 | M2：只发 `SIGINT`（ctrl-c）；`SIGINT` 在 5 s 内没退出时发 `SIGTERM` 再等 5 s；**`SIGKILL` 只在 `gs130_start()` 已完成且日志出现第一帧之后才允许** |
| 免重启恢复 | §3.4 **R-3**（sensor 上电恢复循环）。SDK 的 `Pipeline::deinit()` 会对 `reset_gpio` 重新执行 `sensor_power(gpio, 1)`（写 `1,0,1` 并各延时 30 ms），这本身**就是一次硬件复位**——所以正常 `close()` 就是复位手段，不需要断电 |

### H-3 ISP / VPU 全局资源未释放

| 项 | 内容 |
| --- | --- |
| 描述 | 部分 MIPI/ISP 资源是**全局**的。`hb_mem_module_close()` 是进程级调用，但底层 VPU/ISP 句柄若在异常路径未被 `hbn_vflow_destroy()`，会在内核侧保留引用 |
| 概率 | 低（SDK 的 `teardown_cam()` 覆盖了正常路径；`Pipeline::start()` 失败时会主动 `deinit()`） |
| 影响 | 高：此后任何进程初始化都失败，表现为"相机彻底没了" |
| 检测 | 持有者进程已不存在、`/proc/*/fd` 无设备占用、只读探测仍失败、`dmesg` 无新增错误 |
| 缓解 | 只走 SDK 的 `Device.close()`；严禁并发持有；严禁在未知状态下反复快速试错（每次失败后必须做 §3.3 确认） |
| 免重启恢复 | §3.4 **R-4**。**若 R-4 无效，重启是唯一的最后手段**——因为失去用户态句柄后无法从用户态调用 `hbn_vflow_destroy()`，而 ISP/VPU 的引用只在进程/内核对象销毁时释放。此时必须：(a) 先保存 `dmesg` 与初始化日志；(b) 在测试记录中写明"已尝试 R-1~R-4 全部失败"；(c) 由板卡负责人执行重启 |

### H-4 RAW 模式分辨率约束

| 项 | 内容 |
| --- | --- |
| 描述 | RAW 要求 `output_width/height == sensor 1088×1280`，否则初始化报参数错误（SDK/BSP 已验证事实） |
| 概率 | 中（用户随手写 1280×720） |
| 影响 | 中：启动失败，无硬件损伤 |
| 检测 | 启动日志中的 `GS130_PARAM_ERROR` + 参数名 |
| 缓解 | 节点**不复制校验规则**（需求 freeze Q-8）：参数原样交给 `Config.preset` / `Device.start`，把 `ValueError`/`GS130Error` 翻译成"参数名 + 原因"后非零退出 |
| 免重启恢复 | 修正参数重启节点即可，无需任何硬件动作 |

### P-1 `fps=0` 整数除零

| 项 | 内容 |
| --- | --- |
| 描述 | C 侧 `cycle_ns = 1000000000ULL / dev->fps / 2`、`TimestampTracker(1000000000ULL / dev->fps)`。`fps=0` 是除零，属未定义行为，可能直接崩溃相机线程 |
| 概率 | 低（但 `fps:=0` 是很容易敲出的"关掉图像"意图） |
| 影响 | **高**（进程异常终止 → 落回 H-2 场景） |
| 检测 | 启动日志 + 参数校验 |
| 缓解 | 节点侧对 `fps < 1` 直接参数报错退出（**不进入 SDK**）；文档写明"想只测 IMU 请用 `fps:=1` 或不要启动本节点" |
| 免重启恢复 | 无需恢复，进程根本没碰相机 |

### P-2 相机线程热自旋

| 项 | 内容 |
| --- | --- |
| 描述 | `camera_thread_func` 中，`pipeline->get_frame()` 返回非 OK、或尺寸校验 `PARAM_ERROR`、或 `camera_fifo->push()` 失败时走 `goto free_frame`，**循环体内无 `usleep`**。若错误是持续性的，就会变成"每轮 malloc 2×2 MiB → free → 再 malloc"的热循环 |
| 概率 | 低，但一旦发生后果放大（板卡变卡、温度上升、日志刷屏） |
| 影响 | 高：板卡整体响应变慢，影响共享板上的其他人 |
| 检测 | ① `top -b -n1 \| head -20` 中本节点 CPU ≈ 100%；② `ros2 topic hz /image_combine_raw` 为 0；③ `grep VmRSS /proc/$(pgrep -f gs130_node)/status` 不增长；④ 无新日志 |
| 缓解 | `timeout` 限时；测试期间**必须有人看 CPU**；发现上述四条同时成立立刻 `SIGINT`（此时已过 `start()`，允许 `SIGTERM`，8 s 不退出才 `SIGKILL`） |
| 免重启恢复 | §3.4 **R-3**（因为很可能已经走到 `PARAM_ERROR` 后仍持有 pipeline）。**不需要重启** |

### P-3 IMU 线程 latch `HW_ERROR`

| 项 | 内容 |
| --- | --- |
| 描述 | `imu->full()`（硬件 FIFO 溢出）或 FSYNC 锚点长期缺失导致 `cache.size() > kMaxImuCache` 时，IMU 线程把状态置为 `HW_ERROR` 并返回；此后 `gs130_available_imu()` 返回 0、`read_imu()` 抛 `GS130Error(HW_ERROR)`，且 `gs130_start()` 无法直接重启（需先 `deinit`） |
| 概率 | 低；改小 `imu_fifo.depth` 或降低 `odr` 会显著提高 |
| 影响 | 中：图像也可能随之停止（相机线程状态检查） |
| 检测 | 日志 `GS130Error: gs130_get_imu_packet() -> HW_ERROR` |
| 缓解 | 只用 `Config.preset`（`imu_fifo.depth=1024`, `DROP_OLD`）；不给用户暴露缩小 IMU 队列的参数 |
| 免重启恢复 | SDK 契约：**`deinit` + `init`** 可清 latch（`gs130_deinit` 显式 `state.store(Status::ThreadClosed)`）。节点侧实现为"捕获 `HW_ERROR` → 完整 `close()` → 重新 `Device(...)`"，不重启板卡 |

### P-4 帧尺寸校验失败

| 项 | 内容 |
| --- | --- |
| 描述 | `Pipeline::get_frame()` 校验 VPU 返回的 `img.buffer.width/height` 是否等于 `output_w/h`，不等则 `releaseframe` + 返回 `PARAM_ERROR`，该帧被丢弃 |
| 概率 | 低（RAW/RESIZE 走正常路径时一致；`mode=rect` 首次验证时值得留意） |
| 影响 | 中：表现为"没有帧"，且可能触发 P-2 |
| 检测 | 日志 + `ros2 topic hz` = 0 |
| 缓解 | 每种 `mode` 首次上板时先用单帧验证（`fps:=1`），确认收到帧再加到 30 |
| 免重启恢复 | 改参数重启节点 |

---

## 3. 资源所有权规则（本节点）

### 3.1 谁拥有什么

| 资源 | 拥有者（唯一） | 获取时机 | 释放时机 | 异常退出时的行为 |
| --- | --- | --- | --- | --- |
| GS130 sensor / MIPI RX / ISP / GDC / VSE 管线 | **`gs130_ros` 相机节点进程** | `gs130.Device(Config.preset(...))` → `gs130_init` | `Device.close()` → `gs130_deinit`（内部先 `stop`）→ `gs130_destroy` | 见 §3.2（进程退出行为） |
| EEPROM（I2C bus 4/6 @ `0x50`）读句柄 | 同上（同一 `Device`） | `gs130_init` 内 | `deinit` 内 `eeprom.reset()` | 进程退出关闭 fd |
| IMU（I2C @ `0x68`）句柄 | 同上 | `gs130_init` 内 | `deinit` 内 `imu.reset()` | 进程退出关闭 fd |
| NV12 帧缓冲（`malloc`） | **调用方**（= Python 侧 `Image` 对象持有的 ctypes owner） | `gs130_get_nv12_frame` 返回 `OK` | `weakref.finalize` → `free(address)`（§6） | 进程退出时全部丢弃（内核回收）；这不是泄漏源 |
| ROS 消息 `data` | **消息自己**（`array.array`） | 构造消息时 | 消息被回收时 | — |
| nginx（端口 8000） | **`websocket` 包的 launch**，不是本包 | include `websocket.launch.py` | **没有任何人负责**——它不随 launch 退出而死（§7） | 会残留 |
| `hobot_codec_republish` 进程 | 本包 launch 的 `Node` action | launch | launch 退出（需正常关闭） | 可能残留 |
| `/dev/shm` 的 DDS 共享内存段 | FastDDS / `hobot_shm` | 运行时 | 正常退出 | 残留段会导致"新一轮启动读到旧段" |

### 3.2 获取与释放的顺序契约（必须照此实现）

```python
# 获取（顺序不可交换）
node 起来
  → 1. 前置自检：是否已有持有者（§3.3）        失败 ⇒ 日志 "camera busy" + exit 1
  → 2. cfg = Config.preset(platform, device, mode, w, h, fps, odr)   # 唯一的配置入口
  → 3. dev = Device(cfg)                       # gs130_init：探测 + 上电 + 建流（不开流）
  → 4. dev.start()                             # gs130_start：vflow_start + FSYNC 握手
  → 5. 起 timer / 开始 publish

# 释放（顺序不可交换，且必须在进程结束前完成）
收到 SIGINT
  → 1. 停 timer                                   # 不再取新帧
  → 2. dev.stop()                                 # 等相机/IMU 线程退出
  → 3. del 最后一帧的 numpy 视图                   # 显式，不靠 GC
  → 4. dev.close()                                # deinit（会再 stop 一次，幂等）+ destroy
  → 5. 记录 close 结果；异常只 WARN，不能让进程带不明确状态退出
  → 6. rclpy.shutdown() / destroy_node()
```

要点：

- `Device.stop()` / `close()` 在 SDK 侧是**幂等**的（`stop_locked` 用 `compare_exchange_strong`；`close()` 由 `_closed` 短路），重复调用安全，**但 `close()` 必须被调用**。
- **禁止只靠 `__del__`**：`Device.__del__` 里 `except Exception: pass` 会**静默吞掉 deinit 失败**；进程紧接着退出，日志里什么都没有，现场只剩"相机好像被占了"。M6 因此成立。
- 任何异常路径都要用 `try/finally` 或在 `__exit__`（`with Device(cfg) as dev:`) 内完成释放。

### 3.3 判断"相机是否被持有 / 被谁持有"的确切命令

> 全部为只读命令；除最后一条外不触碰相机。请在**任何**启动前执行。

```bash
# ① 有没有明确知道的相机进程（我们自己的 + TROS 的 + SDK 示例）
pgrep -af 'gs130_node|gs130_probe|gs130_ros|mipi_cam|hobot_stereonet|gs130-run|gs130-rec|gs130-calib-export'
pgrep -af 'python3.*(gs130|probe_nv12)'

# ② 谁打开了相机 / 视频类设备节点（最可靠的一条）
#    逐个内核设备找持有者，不假设节点名
for d in /dev/video* /dev/videocapture* /dev/hb_* /dev/dri/*; do
  [ -e "$d" ] && sudo fuser -v "$d" 2>&1
done

# ③ 不依赖设备名的全进程扫描：谁 open 了相机相关节点
sudo find /proc/[0-9]*/fd -lname '*video*' -o -lname '*videocapture*' -o -lname '*hbmem*' 2>/dev/null \
  | sed 's#/proc/\([0-9]*\)/fd/.*#\1#' | sort -u | xargs -r ps -o pid,ppid,etime,cmd -p

# ④ I2C 侧旁证：sensor 是否还在响应（0x30 在 bus 4、0x32 在 bus 6）
for b in 4 6; do echo "--- i2c-$b"; sudo i2cdetect -y -r $b | grep -E '30|32|50|68' ; done

# ⑤ GPIO 状态旁证（SDK 用 351=bus4 / 353=bus6；-1 表示还没被 SDK export）
for g in 351 353; do
  [ -d /sys/class/gpio/gpio$g ] && echo "gpio$g: $(cat /sys/class/gpio/gpio$g/direction) $(cat /sys/class/gpio/gpio$g/value)" \
                               || echo "gpio$g: not exported"
done

# ⑥ 决定性判据：用只读探测工具（会短暂持有相机，必须独占执行！见下）
gs130 detect camera -b 4 6        # 期望：两个地址都命中 chip-id 0x0132
```

**判读表**：

| ① 有进程 | ②/③ 有设备持有者 | 判定 | 动作 |
| --- | --- | --- | --- |
| 有 | 有 | **明确的持有者** | §3.4 R-1（礼貌停止那个进程） |
| 有 | 无 | 进程在但已不持设备（可能正在初始化/正在退出） | **等 5 s 再测一次**；仍如此 → R-2 |
| 无 | 无 | 没有用户态持有者 | 走 ⑤⑥；⑥ 通过即可启动 |
| 无 | 无，⑥ 失败 | **H-2 / H-3 的签名** | §3.4 R-3 → R-4 |
| 无 | 有（孤儿 fd 指向某个进程） | 用 ③ 的输出点名该 PID | R-2 |

**关于 `gs130 detect camera`**：它是**内部 bring-up 工具**，会走 I2C 探测（等价于 `i2cdetect` 命中判断），风险低，但仍是"接触硬件"。**绝不能与节点同时执行**。如果不想碰硬件，可以只做 ①~⑤，把 ⑥ 留给"准备启动之前"的最后一秒。

### 3.4 免重启恢复流程（按顺序尝试，任一步成功即停）

**R-1 礼貌停止（首选）**

```bash
pgrep -af gs130            # 记下 PID
kill -INT <pid>            # 等价 ctrl-c：走正常 stop/close
for i in $(seq 10); do kill -0 <pid> 2>/dev/null || break; sleep 1; done
pgrep -af gs130 || echo "RELEASED"
```

**R-2 强制收尾（进程已无响应且 **已确认 `gs130_start()` 已完成**）**

```bash
# 只在"日志里已经出现过帧"之后才允许；初始化中不得使用（M2）
kill -TERM <pid>; sleep 5
kill -0 <pid> 2>/dev/null && kill -KILL <pid>
sleep 2
# 立刻确认没有孤儿 fd
sudo find /proc/[0-9]*/fd -lname '*video*' 2>/dev/null | head
```

**R-3 sensor 上电恢复循环（H-2 / P-2 的恢复；不需重启，不做任何手工 GPIO 写）**

原理：`Pipeline::deinit()` 会对 `reset_gpio` 重新调用 `sensor_power(gpio, 1)`，即对两颗 sensor 各做一次 `1 → 0 → 1`（每步 30 ms 延时）。**正常 `close()` 就是硬件复位**，因此用 SDK 自己完成一次干净的 init/close 循环即可。

```bash
# 1) 确认没有任何持有者（§3.3 ①②③ 全空）
# 2) 用 SDK 示例做一次"完整 init + 立即 close"，即完整走一遍上电/复位/下电/再上电路径
timeout 30 gs130-run RDKX5 GS130WI raw 1088 1280 30 100
#    预期：打印 fps/odr 后 ctrl-c 退出；或直接正常退出
# 3) 再确认
gs130 detect camera -b 4 6
# 4) 起节点，先用低帧率验证单帧
ros2 launch gs130_ros gs130_camera.launch.py fps:=1
```

**R-4 资源兜底（H-3）**

```bash
sudo dmesg | tail -60                       # 保存证据
pgrep -af 'gs130|mipi_cam' || echo "no owner"
sudo fuser -v /dev/video* /dev/videocapture* 2>&1   # 必须为空
sudo ss -lntp | grep -E '8000|11311'        # 顺带确认没有遗留服务

# 若④⑤⑥ 仍失败：
#   → 重新插拔相机排线（物理断电，由现场人员执行；这是"重启"的轻量替代）
#   → 仍失败：记录 dmesg + 日志，交由板卡负责人决定是否重启
```

**重启的门槛（必须同时满足）**：R-1、R-2、R-3、R-4 全部失败；`dmesg` / 初始化日志已保存；重启已作为"异常事件"写入测试记录。**理由**：重启会销毁内核侧的 VPU/ISP 引用，是唯一能清掉"用户态已无句柄但全局资源仍被引用"的手段；但它同时会中断共享板上其他人的工作，并且**掩盖根因**（下次还会复现）。

---

## 4. ROS 级风险

### 4.1 消息数据拷贝开销（RAW 1088×1280，30 Hz）

单帧字节数：`1088 × 1280 × 3/2 = 2,088,960 B ≈ 1.99 MiB`。
按 freeze §2.1 的拼接约定（水平拼接、左目在左），`/image_combine_raw` 的每帧是 `2 × 2,088,960 = 4,177,920 B ≈ 3.98 MiB`。

| 环节 | 拷贝次数 | 单帧量（30 Hz 下带宽） | 备注 |
| --- | --- | --- | --- |
| SDK `Pipeline::get_frame` | 每帧 1920 次 `memcpy`（每行 1088 B） | ≈ 4.18 MB / 帧 | 已是 SDK 行为，不在本包优化范围 |
| Python `Image` 构造 | **0**（`np.frombuffer` 包装同一块 malloc 内存） | 0 | 这是 SDK 的零拷贝优势，必须保住 |
| 拼接成 `/image_combine_raw` | 1 次（按行交错 + UV 拼接） | ≈ 4.18 MB / 帧 ≈ 125 MB/s | 目的是让 `hobot_codec` 零修改工作（freeze §2.1） |
| 构造 `sensor_msgs/Image.data` | **≥ 1 次**（见下） | ≈ 4.18 MB / 帧 ≈ 125 MB/s | 必须的：消息要拥有数据 |
| DDS 序列化 | 1 次（FastDDS 内部） | 同量级 | 无法避免 |

**关键实现规则（可实测的差异）**：

- `sensor_msgs/Image.data` 的 setter 在 rclpy 中是：若是 `array.array`（typecode `'B'`）则**直接持有**；否则 `self._data = array.array('B', value)`（逐元素拷贝）。因此：
  - ❌ `msg.data = frame.tobytes()` → bytes → 再被拷进 `array.array`，**两次拷贝**。
  - ✅ `data = array.array('B'); data.frombytes(memoryview(frame).cast('B'))` → **一次拷贝**，且 `data` 自己拥有内存（不是视图），生命周期与相机缓冲**解耦**。
  - ✅ 更省的做法：复用预分配的 `array.array('B', bytes(total))`，按行交错填 `Y`，再拼 `UV`——仍是 1 次写入，无中间对象。
- 结论：**发布前做一次显式拷贝，把所有权从 SDK 缓冲转移到消息**。这是在 rclpy 下唯一能同时保证正确性与可接受开销的方案（TROS 的 `hobot_shm` 零拷贝只服务于 D-Robotics 自家 C++ 节点，rclpy 无法借道）。
- 允许的"下一步实验"（必须在文档中标注**未验证**）：尝试用 `hobot_shm` 环境让 FastDDS 走 `shm` 传输，看 `ros2 topic hz` / CPU 是否改善；不建议为了这个引入 C++ 节点（freeze N-02）。

### 4.2 发布队列深度与慢订阅者

| 话题 | depth | 理由 | 慢订阅者时的行为 |
| --- | --- | --- | --- |
| `/image_combine_raw` | **1** | 实时性优先：旧帧对新帧无价值，积压只会增加延迟与内存 | 队列丢旧帧；SDK 侧 `camera_fifo` 是 `DROP_OLD`（depth 4），两头都"保新" |
| `/image_left_raw`、`/image_right_raw` | 1 | 同上（默认关闭） | 同上 |
| `/imu/data` | **200** | IMU 以突发形式成批到达（FSYNC 锚点后补点），突发量可能几十个包；depth 太小会丢包并破坏下游对齐 | 200 Hz 下可容忍 1 s 订阅者滞后 |
| camera_info | 1（`TRANSIENT_LOCAL`） | latched，一次 | — |

**内存增长最坏情况估算**（depth=1 时）：

- 图像：1 条在队 + 1 条正在序列化 + FastDDS 发送缓冲 ≈ 3 帧 ≈ **12.5 MB 稳态**；**不随运行时间增长**。若观测到 RSS 单调增长，几乎一定是 §5.2 的泄漏，而不是队列。
- 若误把 `depth` 设成 10：队列里会同时存在 10 个各持 4 MiB 的消息 → **≈ 40 MB 常驻 + 队列延迟 ≈ 333 ms**，web 页面表现为"明显延迟、操作不同步"。
- IMU：200 × 约 40 B 有效载荷（`sensor_msgs/Imu` 实际序列化约 300 B）≈ 60 KB，可忽略。

**发布实现的三条硬规则**：

1. **每帧新建一条消息对象**，禁止复用同一条 `Image` 消息反复改写 `data`（复用会让队列里的多条 entry 指向同一块内存，序列化时内容全变）。
2. 记录**丢帧计数**：`available_camera()` 与索取次数之差、"请求发布但队列满"的计数，1 Hz 汇总到日志（freeze T8 可选话题）。**没有计数就没法判断是"慢"还是"坏"。**
3. 不在回调里做任何阻塞操作（编码、写盘、等锁）；回调只做"取帧 → 拷贝 → publish"。

### 4.3 QoS 与 D-Robotics 节点的匹配

| 对端 | 它的 QoS | 我们必须是 | 不匹配的症状 |
| --- | --- | --- | --- |
| `hobot_codec_republish`（订阅 NV12） | `BEST_EFFORT` | `BEST_EFFORT` + `VOLATILE` + `KEEP_LAST` | publisher RELIABLE ↔ subscriber BEST_EFFORT **可以**匹配；反向（pub BEST_EFFORT ↔ sub RELIABLE）**永远不匹配** |
| `websocket`（订阅 jpeg） | 由 codec 决定 | 我们不经手 | 网页黑屏 |
| `hobot_stereonet`（订阅 `/image_combine_raw`） | 参考链为 `BEST_EFFORT` | 同 T1 | 节点起得来但收不到帧 |

**规则**：QoS 是**链路契约**的一部分，不是用户偏好。只暴露 `image_qos_depth` / `imu_qos_depth`（freeze §5.1），**不允许**把 reliability 改成 `RELIABLE`。

**诊断命令（黑屏 / 收不到帧时的第一步）**：

```bash
ros2 topic info -v /image_combine_raw     # 看 Type / Publisher count / Subscription count 与各自的 QoS
ros2 topic hz /image_combine_raw          # 有没有真的在发
ros2 topic echo --once /image_combine_raw --field encoding   # 应为 nv12
ros2 topic echo --once /image_combine_raw --field width      # 应为 2176（=2×1088）
ros2 topic info -v /image_combine_jpeg    # codec 有没有产出
```

`Subscription count = 0` 且对端进程明明在跑 → **就是 QoS 不匹配**，不要怀疑相机。

### 4.4 时间戳单位与时钟域

| 陷阱 | 具体表现 | 规则 |
| --- | --- | --- |
| **单位错** | `header.stamp.nanosec` 被填成整秒数、或 `sec` 里塞了纳秒 | 一律 `sec = ns // 1_000_000_000`，`nanosec = ns % 1_000_000_000`；`sec` 是 `int32`，**不要塞 epoch 纳秒**（会溢出） |
| **时钟域错（最隐蔽）** | SDK `timestamp_ns` 来自 VPU 帧信息（优先 `trig_tv` 曝光触发沿，回退 `timestamps` / `tv`），**其时钟域未经核实**；若直接用 `ns` 拆成 `sec/nanosec`，下游看到的是"1970 年"或"开机 3 小时"，`tf`/`message_filters` 集体错位 | 采用 freeze `§4.2` 的**一次性偏移量**策略：启动时用第一帧有效设备时间戳算一次 `offset_ns = (节点 now_ns) - device_ts_ns`，此后**恒定**，图像与 IMU **共用同一个偏移量**。禁止每帧用 `now()` 覆盖（会抹掉曝光间隔），禁止滑动平均（引入抖动） |
| **`use_sim_time`** | 为真时 `now()` 变成仿真时间，偏移量不可复现 | 启动时检测为真 → `WARN` 并按 `use_sim_time=false` 语义继续 |
| **0 / 回退时间戳** | 单帧 `timestamp_ns == 0`；或 IMU 包时间戳非单调 | 图像：照发，用"上次有效 + `1e9//fps`"保持单调并 `WARN`（1 Hz 节流）；IMU：**丢弃并 WARN**（错的时间戳比丢包更有害） |

**启动自检（必须实现，10 秒内能给出结论）**：

```python
# 采 100 个有效帧时间戳与本地时钟，打印二者差值的中位数
# 判定：|median(diff)| < 10 ms 且 |diff| < 1e9 → 设备时钟与节点时钟同源（可直接用，offset≈0）
#       否则                                → 必须走 offset_ns 方案，并在日志打出 offset 值
```

```bash
# 板卡上人工复核（不需要相机也能看时钟域是否可信）
python3 -c "import time;print('mono',time.monotonic_ns());print('real',time.time_ns())"
# 若帧时间戳与 mono 相差在一个常量内、与 real 相差数十年 → 设备时钟是单调时钟，offset 方案必需
```

### 4.5 IMU 与图像不同频

| 项 | 事实 | 规则 |
| --- | --- | --- |
| 频率 | 图像 `fps`（默认 30 Hz）；IMU `odr`（默认 100 Hz，可按需 200 Hz） | 两个独立 timer/回调，**互不等待** |
| 突发性 | SDK 的 IMU 时间戳在 FSYNC 锚点后"补点"，一帧周期内可能一次性吐出多个包，且**时间戳可以早于最近一帧图像** | 不要把"最近收到的 IMU"当作"这一帧的 IMU" |
| 时间关系 | 图像与 IMU 时间戳经 `TimestampTracker` 对齐到**同一时基**（IMU 校正到相机时钟），可直接互比 | 保留物理语义，**不做插值/对齐**（freeze §4.4） |
| 对齐责任 | 下游 | 文档明确要求下游用 `message_filters.ApproximateTimeSynchronizer`，容差 ≥ 2 × IMU 周期（100 Hz → ≥ 20 ms） |
| 常见错误 | 在图像回调里"顺手发一次 IMU"，或把 IMU 采样率当成图像帧率的整数倍去假设对齐 | 禁止：IMU 一包一发；不得跨话题复用同一 `stamp` 变量 |
| `device=GS130W` | 无 IMU：`read_imu()` 恒 `None`，`imu_name` 为 `None` | 不发布 `/imu/data`，日志一条"no IMU"说明，**不报错退出**（freeze AC-04） |

---

## 5. 内存与缓冲所有权分析（ROS 路径）

### 5.1 完整所有权链（从上到下）

```
core/src/gs130.cpp  camera_thread_func()
  y[R] = malloc(width*height*3/2);  y[L] = malloc(width*height*3/2)
        │  所有权：SDK 内部 → 入队成功即转移给"调用方"
        ▼
  camera_fifo->push(frame)                 # FIFO 只存 2 个指针，深拷贝=0
        │  · 队列满(DROP_OLD)：disposer_ = free_frame_pair() 释放被覆盖的旧帧
        │  · Fifo 析构：disposer_ 释放仍在队列里的帧
        ▼
  gs130_get_nv12_frame()  →  *image_left = f[Left]; *image_right = f[Right]
        │  头文件契约：data 由 SDK malloc，所有权归调用方，用完自行 free()
        ▼
python/gs130/_device.py  read_image()
        │  · 返回 None（TIMEOUT）：**没有 buffer**，绝不能 free
        │  · 返回 dict：每个 Image 拥有一个 malloc 指针
        ▼
python/gs130/_types.py  Image.__new__()
  owner = (ctypes.c_uint8 * size).from_address(address)
  weakref.finalize(owner, _free, address)      # ← 唯一的释放点
  array = np.frombuffer(owner, dtype=np.uint8).reshape(h*3//2, w)
        │  · numpy 只是视图；真正的"生命周期锚点"是 owner(ctypes) 对象
        │  · 最后一条引用（owner）死亡 ⇒ CPython 引用计数归零 ⇒ finalize 回调 free(address)
        ▼
gs130_ros 节点：拼成 /image_combine_raw
        │  必须在这里把所有权"抄"给消息（§5.3）
        ▼
sensor_msgs/Image.data （array.array('B')，自己拥有内存）
        │
        ▼
rclpy 序列化 → DDS → hobot_codec_republish → jpeg → websocket → 浏览器
```

**必须记住的一句话**：`free()` 的唯一触发点是 `owner` 的最后一条引用消失。**谁持有 `owner`（直接或间接），谁就推迟释放；谁丢了它，谁就提前释放。**

### 5.2 泄漏 / double free / use-after-free 的完整枚举

#### A. 泄漏（leak）

| # | 场景 | 机理 | 防止规则 |
| --- | --- | --- | --- |
| A1 | `read_image()` 拿到 `OK` 后抛异常 | `Image.__new__` 在 `address == 0` 时 `RuntimeError`；此时**左右两块 buffer 都已由 C 侧分配**，无人 free | 在 `read_image()` 调用点外层用 `try/finally`；拿到 dict 后**先**把引用存进 `self._pending`，再处理；`Image` 构造失败时立刻让 `dev` 走 `close()`（`deinit` 会销毁 `camera_fifo`，FIFO 析构释放队列内残余帧） |
| A2 | 只处理了 `left`，`right` 被丢弃 | `{"left","right"}` 两个值都持有 malloc 指针；只留一个，另一个随 dict 死亡而 free——**这不算泄漏**；但如果代码把 `right.data` 塞进某处长期存活的结构（如缓存、日志缓冲）就会 | 明确：`right` 不用就让它自然离开作用域（引用计数归零即释放）；**禁止**把 `Image` 存进长期容器 |
| A3 | 拼接/转换产生中间 numpy 视图并长期持有 | `arr.reshape(...)`、切片都是**视图**，会延长 `owner` 的寿命——不是泄漏但是**延迟释放**（内存占用被推高） | 拼接结果必须是**拥有自己内存**的 `array.array`；旧帧的视图在 publish 返回后 `del` |
| A4 | 在异常路径上跳过 `close()` | 相机句柄与线程未释放；`camera_fifo` 里最多 4 条帧对（≈ 8 MiB）也留着 | M6：`try/finally` + `with Device(...)`；节点 `on_shutdown` 显式 `stop()`+`close()` |
| A5 | 崩溃/`kill -9` 导致帧丢失 | 进程退出，用户态内存由内核回收——**不是真泄漏**；真正的损失是相机状态（H-2） | 不适用；重点是 H-2 |
| A6 | C 侧理论上不会泄漏（已核对） | `gs130.cpp`：第二个 `malloc` 失败 → `free_frame_pair` 后 `continue`；`push` 失败 → `goto free_frame`；`Fifo` 析构 → `disposer_` 释放残余；`get_nv12_frame` 的 `TIMEOUT` 路径不写结构体 | **不要试图在 Python 里"补偿"一个不存在的 C 侧泄漏**（那会造成 double free） |

#### B. double free

| # | 场景 | 机理 | 防止规则 |
| --- | --- | --- | --- |
| B1 | 同一次 `OK` 结果被释放两次 | 复制了 `Image` 对象（例如 `a = img; b = img[:]`）本身不重复 free（同一个 `owner`）；但若把 `address` 抄进第二个 `from_address` + `finalize`，就会 free 两次 | **一个 `OK` 结果只允许一个释放点**。禁止在 Python 侧手工构造第二个 owner / 调用 `_free` |
| B2 | 复用了 `ImageNV12` 结构体再 free | `read_image()` 在 `TIMEOUT` 时**不写** `raw`，结构体保留**上一次的旧指针**。任何"缓存 callback 参数、稍后再 free"的代码都会 free 一个早已释放的地址 | `_abi.ImageNV12()` 必须在每次调用前**新建**（现有实现已经是）；**禁止**跨调用复用结构体、禁止把结构体当"帧句柄"缓存 |
| B3 | 进程崩溃后的"补偿式 free" | 有人为了"保险"在 finally 里对旧地址再 free 一次 | 同上：释放点唯一 |
| B4 | 拼接模式下按眼 free | `GS130_STEREO_LAYOUT_*` 分支是**一次 malloc 覆盖两只眼**（`y[R] = y[L] + width`），按眼 free 会崩。当前 preset 固定 `stereo_layout=NONE`，**不可达**，但若将来有人绕过 preset 打开拼接到 SDK 层就会踩到 | 保持 `stereo_layout=NONE`（freeze 决策）；任何绕过 preset 手工构造 `CameraConfig` 的做法视为禁止操作 |

#### C. use-after-free

| # | 场景 | 机理 | 防止规则 |
| --- | --- | --- | --- |
| C1 | **发布还没完成，`owner` 已死** | `publish()` 返回前，rclpy 需要读 `msg.data`。若 `data` 是**指向相机缓冲的视图/`memoryview`**，而 `owner` 已因作用域结束被回收（或被上一帧赋值覆盖），序列化读到的就是已 free 的内存 | **M5 的核心**：`msg.data` 必须是**自己拥有内存**的 `array.array`（§5.3 的写法），生命周期与相机缓冲解耦 |
| C2 | 用"上一帧的 `data`"顶替 | 为省拷贝而复用 `array.array` 并被 `frombytes` 重填，而队列里还挂着用同一对象构造的消息 | 每帧新建消息**与**新建 `data`（或改成"拥有内存 + 双缓冲轮换、且确信队列 depth=1"） |
| C3 | 计时器回调重叠/异步 | 若把 `publish` 放进 `Executor` 的多线程回调或 `create_task`，两个回调可能同时操作同一个缓冲 | 相机取帧→发布只允许在**单线程 Mutex 回调组**里串行执行；禁止在回调里 `await`/`sleep` |
| C4 | 把指针传出进程 | 任何把 `data` 地址（整数）写进共享内存、发给别的进程的做法都是无意义的（对方的堆里没有这块内存） | 禁止；跨进程只走 ROS 消息（那正是拷贝存在的理由） |
| C5 | 退出时 `_free` 已不可用 | 解释器关闭期间 `_types._free`（模块全局）可能已被置空 → finalize 抛 `TypeError` 被吞掉；极端情况下在 `libc` 之后析构 | 退出路径**显式**丢掉最后一帧引用（`del`），不要留给解释器关闭阶段的 GC |

### 5.3 numpy 数组释放前必须发生的事（可执行清单）

**必须**（顺序不能反）：

1. **先决定这一帧用不用**：`None`（TIMEOUT）⇒ 到此为止，**没有任何 buffer**，不得 free、不得复用结构体。
2. **先算长度再拷贝**：`size = width * height * 3 // 2 == len(data)`，NV12 的 `Y` 是前 `width*height` 字节，`UV` 是后 `width*height/2` 字节。长度不符直接判为 bug 并丢弃该帧（绝不 publish 短消息：`hobot_codec` 会按 `width/height` 语义读越长越好，导致越界读）。
3. **把数据抄进一个自己拥有内存的容器**：
   ```python
   from array import array
   W, H = 2 * w, h                      # 拼接后 2176 × 1280
   total = W * H * 3 // 2               # 4,177,920 B
   out = array('B', bytes(total))       # 拥有自己的内存（可预分配一次、每帧覆写）
   mv  = memoryview(out)                # 可写视图，指向 out 自己的内存

   y_view = np.frombuffer(mv[:W * H], dtype=np.uint8).reshape(H, W)   # Y 平面
   y_view[:, :w] = left [:h, :]         # left 是 shape (h*3//2, w) 的 Image 视图
   y_view[:, w:] = right[:h, :]         # Y 行交错：左目偶数列区、右目奇数列区

   uv = (H // 2) * W                    # UV 平面字节数
   uv_view = np.frombuffer(mv[W * H:W * H + uv], dtype=np.uint8).reshape(H // 2, W)
   uv_view[:, :w] = left [h:, :]        # UV 左段 = 左目 UV
   uv_view[:, w:] = right[h:, :]        # UV 右段 = 右目 UV
   # 一次写入，无中间 bytes；msg.data = out 时 rclpy 不再拷贝
   ```
   一次性拷贝，无中间 `bytes`（`tobytes()` 之后会被 setter 再拷一次，是两次）。
4. **构造消息并赋值** `msg.data = out`（`array('B')` 被 rclpy **直接持有**，不再拷贝）。
5. **publish**。
6. **publish 返回之后**才允许 `del` 帧引用（`left`/`right`/任何视图）。若第 3 步已经完成拷贝，则**第 5 步其实不依赖**旧缓冲，但**仍按此顺序写**（因为这是唯一对未来改动免疫的顺序）。

**禁止**：

- ❌ `msg.data = frame.data`、`msg.data = memoryview(frame)`、`msg.data = np.array(..., copy=False)` 之类**共享内存**的赋值。
- ❌ 依赖 `gc.collect()` 或 `__del__` 决定释放时机（M6）。
- ❌ 手工再构造一个 `from_address` owner。
- ❌ 复用 `_abi.ImageNV12` 结构体跨调用。
- ❌ 在 `publish` 之前 `del` 掉视图——即使当前实现是拷贝，这个习惯会在未来改成零拷贝时立刻变成 UAF。

**一句话规则**：**`numpy` 视图（含 `owner`）必须在"数据的完整内容已被复制进消息"之后才允许释放；而这个"复制完成"的时点，在 rclpy 里是 `publish()` 返回之后。**

### 5.4 内存监控（唯一的客观判据）

```bash
PID=$(pgrep -f gs130_node | head -1)
watch -n 2 "grep -E 'VmRSS|VmSize' /proc/$PID/status"
# 合格：30 秒到 5 分钟内 VmRSS 在 ±10% 内波动（峰值 ≈ 30~60 MB 量级，取决于 depth）
# 不合格：单调上升且不回落到基线 → 泄漏（先查 depth，再查 §5.2 的 A 系列）
```

配套：`ros2 topic hz /image_combine_raw` 与 `ros2 topic info -v` 的 subscription count 一起看；**只有 RSS 涨、hz 正常**⇒ 泄漏；**hz=0、CPU 满**⇒ P-2 热自旋；**hz 正常但网页花屏**⇒ 极可能是 UAF（C1）。

---

## 6. Launch 级风险

### 6.1 `websocket.launch.py` 的真实行为（已核对源码，必须按此设计）

```python
# 该文件在 generate_launch_description() 里，也就是 launch **构造阶段**就执行：
processes = subprocess.check_output(['ps', 'ax'])
processes = [p for p in processes.split('\n') if './sbin/nginx -p .' in p]
if len(processes) > 0: "webserver has launch"      # 跳过启动
else: os.chdir(<websocket_prefix>/lib/websocket/webservice); os.system('./sbin/nginx -p .')
```

由此产生四个必须接受的后果：

| # | 后果 | 风险 | 我们的对策 |
| --- | --- | --- | --- |
| W1 | 探测是**命令行字符串子串匹配**，对 `nginx -p .`、绝对路径启动、`-g` 参数等**都会漏判** | 漏判 ⇒ 再起一个 nginx ⇒ **端口 8000 冲突**，或 nginx 以错误工作目录运行导致 web 资源 404 | 我们的 launch 在启动前用 `pgrep -af nginx` 检查并在日志/报错里给出明确提示；**不修改** TROS 的 launch |
| W2 | `os.system()` 在 launch 构造阶段就**同步**执行并返回（`nginx` 自身 daemonize），nginx 成为 `ros2 launch` 的子进程，**launch 框架不管理它** | launch 退出后 nginx **残留**（孤儿进程），端口 8000 一直占着 | §6.3 的显式收尾；**必须写进 README** |
| W3 | 若我们**先**用 `websocket_service.launch.py` 起了服务，再 include `websocket.launch.py`，后者会检测到并跳过 | 正常；但两条路径的探测字符串相同，任一处漏判就双起 | 一个 launch 只 include 其中**一个**；本包默认 include `websocket.launch.py`（它自带服务启动） |
| W4 | 我们在**自己的 launch 里再 include 一次 `hobot_shm.launch.py`** | 与 `hobot_codec.launch.py` 内部的 include **重复**（`hobot_codec.launch.py` 第 73 行已 include `hobot_shm.launch.py`） | 二选一：只用 `hobot_codec_encode.launch.py`（它是否 include `hobot_shm` 需在板上一眼确认），或**不 include 任何 hobot 的 launch，直接在 `Node()` 里逐个起 `hobot_codec_republish` / `websocket`**（本包推荐：可控、可传参、无隐藏 include） |

### 6.2 两个 launch 同时启动

| 冲突对 | 症状 | 预防 |
| --- | --- | --- |
| 我们 + 我们 | 第二个节点 `gs130_init` 失败（H-1）；两个 `hobot_codec_republish` 抢同一个 channel（0~3） | 启动前 `pgrep -af gs130_node`；`hobot_codec` 的 `channel` 参数固定并文档化 |
| 我们 + `mipi_cam` | 两个相机持有者（L-2，最严重） | 见 §6.4 |
| 我们 + `websocket` 手工起的 | nginx 双起（W1/W2） | 启动前 `pgrep -af nginx` |
| 我们 + `hobot_stereonet` 的 web launch | 两个 websocket 订阅不同的 jpeg 话题，网页显示谁不确定 | 一次只验证一条链路；stereonet 需要拼接输入，**不在 v0.1.0 范围**（freeze C9） |

### 6.3 孤立的 nginx / codec 进程

| 进程 | 什么时候会残留 | 检测 | 清理（§6.3 统一执行） |
| --- | --- | --- | --- |
| `./sbin/nginx -p .` | 每次 web launch 之后（W2） | `pgrep -af nginx`、`ss -lntp \| grep 8000` | `pkill -f './sbin/nginx'`；失败则 `sudo pkill -f nginx` + 确认端口释放 |
| `hobot_codec_republish` | launch 非正常退出 | `pgrep -af hobot_codec` | `pkill -f hobot_codec_republish`（**不含相机，安全**） |
| `websocket` 节点 | 同上 | `pgrep -af 'websocket'` | `pkill -f 'websocket --ros-args'` |
| 我们自己的节点 | 异常退出 | `pgrep -af gs130_node` | **先 SIGINT，再判定**（M2） |

**注意**：`pkill -f nginx` 会杀掉别人起的 nginx（共享板！）。因此默认只杀**我们自己模式**的进程（`-f './sbin/nginx'`），要杀宽范围必须先确认没有其他人在用 8000。

### 6.4 误启动 `mipi_cam` 抢相机

**为什么会发生**：D-Robotics 的参考 launch（`mipi_cam.launch.py` / `hobot_stereonet` 的 web launch / 各种 demo）里 `mipi_cam` 是隐式启动的；照抄参考 launch 做"最小修改"就会把相机抢走。

**硬规则**：

1. 本包 launch **只允许** `Node(package='gs130_ros', ...)` + `Node(package='hobot_codec', executable='hobot_codec_republish', ...)` + `Node(package='websocket', executable='websocket', ...)`。
2. **禁止** include `mipi_cam`、`hobot_stereonet`、`hobot_usb_cam` 的任何 launch。
3. 若确实要 include `hobot_codec.launch.py` 或 `websocket.launch.py`，必须在文件头注释写明："本 include **不包含** `mipi_cam`"，并在 launch 构造阶段加一道自检：

```python
import subprocess, sys
busy = subprocess.check_output(['ps', 'ax'], universal_newlines=True)
if 'mipi_cam' in busy:
    print('[gs130_ros] FATAL: mipi_cam is running; GS130 and mipi_cam cannot share the camera. '
          'Stop it first (see ros/docs/22_risk_and_safety.md 8.4)', file=sys.stderr)
    raise RuntimeError('mipi_cam is running')
```

4. **禁止** `ros2 run mipi_cam mipi_cam` 手工验证"顺便试试"。

### 6.5 `websocket.launch.py` 的自检代码在 launch 构造阶段运行 ⇒ 我们的检查也必须在构造阶段

因为 TROS 的 launch 在 `generate_launch_description()` 里就 `os.system` 起 nginx，**任何"放到 `OnProcessStart` 之后再检查"的写法都太晚了**。我们的检查（持有者 / mipi_cam / nginx / 端口）必须放在 `generate_launch_description()` 顶部，违反时直接 `raise`，让 launch 在动任何东西之前失败。

---

## 7. 开发过程的操作安全规则

### 7.1 允许在板卡上运行的

| 允许 | 备注 |
| --- | --- |
| `ros2 launch/run gs130_ros ...`（本包节点） | 唯一被授权持有相机的进程 |
| `ros2 run gs130_ros gs130_probe` | **独占、勿与节点同跑**；会短暂持有相机 |
| `ros2 topic list/info/echo/hz`、`ros2 node list`、`ros2 param get` | 只读，安全 |
| `pgrep/ps/fuser/lsof/ss/dmesg/top/free` | 只读，安全 |
| `i2cdetect -y -r 4/6` | 只读探测；**不与相机进程同时执行** |
| `colcon build --packages-select gs130_ros` | 可，但**不要在相机跑的时候**做大编译（CPU 与内存会拖垮取流） |
| `gs130 detect camera -b 4 6` | 内部 bring-up 工具，独占执行 |

### 7.2 禁止在板卡上运行的

| 禁止 | 原因 |
| --- | --- |
| `mipi_cam` 的任何形式（`ros2 run/launch mipi_cam`） | M1：抢相机 |
| `hobot_stereonet` 的 web launch | 需要拼接输入（本包 v0.1.0 不提供，freeze C9）+ 会隐式起 `mipi_cam`/websocket |
| 两个持有者同时跑（哪怕"就一下"） | M1；历史上"卡死相机"的来源 |
| `gs130-run` / `gs130-rec` / `gs130-calib-export` 与节点同时 | 同上（都是持有者） |
| `python3 -c` 里临时 `import gs130; Device(...)` | 同上，且无日志、无清理 |
| 手工写 `/sys/class/gpio/*`、`i2cset`、`devmem` | M9：与 SDK 电源时序冲突 |
| `rmmod/modprobe` 相机/VPU 内核模块 | 可能让板卡进入不可恢复态 |
| `sudo reboot` 作为第一反应 | M3 |
| `kill -9` 相机初始化中 | M2 |
| 无人看守地跑（过夜、`&`、`nohup`） | M8 |

### 7.3 干净地停止一个正在运行的系统（固定顺序）

```bash
# ① 相机节点：先 SIGINT，给足宽限（这才是 SDK 的 deinit 路径）
pkill -INT -f gs130_node ; sleep 3
pgrep -af gs130_node && { sleep 5; pgrep -af gs130_node; }   # 仍在 ⇒ 见 R-2（先确认已出过帧）
pgrep -af gs130_node || echo "camera node released"

# ② codec / websocket（不含相机，随时可停）
pkill -f hobot_codec_republish 2>/dev/null; pkill -f 'websocket --ros-args' 2>/dev/null; sleep 1

# ③ 关掉浏览器页面（否则 websocket 会重连，下一轮的现象会互相污染）——人工步骤

# ④ nginx（只杀我们自己模式的）
pgrep -af nginx
pkill -f './sbin/nginx' ; sleep 1
ss -lntp | grep 8000 || echo "port 8000 free"

# ⑤ 最终确认相机空闲（§3.3 ①②③ + 可选⑤⑥）
pgrep -af 'gs130|mipi_cam' || echo "no camera owner"
```

### 7.4 永远不要做（与"不要做"的原因）

| 不要做 | 后果 |
| --- | --- |
| 把重启当第一反应 | 中断共享板上他人工作；掩盖泄漏根因；下次照旧 |
| 在初始化窗口 kill -9 | sensor 半初始化 → 可能需要插拔排线/重启（H-2） |
| 同时跑两个相机拥有者 | 之前出现过的"卡死相机" |
| 无人看守地跑流 | 出问题时没人能及时 SIGINT |
| 在相机跑的时候做大编译/跑大模型 | CPU/内存争抢 → 丢帧 → 误判为"驱动 bug" |
| 手工 GPIO/I2C 复位 | M9 |
| 修改 `python/` 下的绑定来"绕过"问题 | 本阶段冻结（freeze N-09）；会制造两套真相 |
| 为省一次拷贝而让消息共享相机缓冲 | §5.2：UAF |

---

## 8. 运行前检查清单（PRE-RUN，逐项勾选，任一项不过就停止）

> 下表中含 `|` 的命令必须放在反引号里整条执行；为免歧义，另有 §8.1 的"一键版"可直接粘贴。

| # | 检查项 | 命令 | 通过判据 |
| --- | --- | --- | --- |
| 1 | 没有已知相机持有者 | `pgrep -af 'gs130\|mipi_cam\|hobot_stereonet'` | 无输出 |
| 2 | 没有进程打开相机设备 | `for d in /dev/video* /dev/videocapture*; do sudo fuser -v "$d"; done` | 无输出 |
| 3 | 无残留 nginx / 端口空闲 | `pgrep -af nginx` 与 `ss -lntp \| grep 8000` | 无输出（或明确知道是谁的） |
| 4 | 无残留 codec / websocket | `pgrep -af 'hobot_codec\|websocket'` | 无输出 |
| 5 | GPIO 状态正常（未接管/已恢复） | `ls /sys/class/gpio/ \| grep -E '351|353'` | 与上次一致（不存在或已 export 且 value 可读） |
| 6 | I2C 上能看到 sensor / EEPROM / IMU | `sudo i2cdetect -y -r 4; sudo i2cdetect -y -r 6` | 出现 `30/32`(sensor)、`50`(EEPROM)、`68`(IMU) |
| 7 | 库版本一致 | `python3 -c "import gs130;print(gs130.__version__, gs130.library_version())"` | 两者一致（不一致会有 `RuntimeWarning`） |
| 8 | TROS 环境已 source | `echo $ROS_DISTRO; ls /opt/tros/humble/setup.bash` | `humble`；文件存在 |
| 9 | 电源/温度余量 | `vcgencmd measure_temp 2>/dev/null \|\| cat /sys/class/thermal/thermal_zone0/temp` | < 70 ℃（数值/1000） |
| 10 | 参数合法 | `ros2 param get` 或 node `--ros-args -p mode:=raw -p width:=1088 -p height:=1280 -p fps:=30 -p odr:=100` | RAW 必须是 1088×1280；`fps>=1`（P-1） |
| 11 | 运行有超时保护 | 命令前加 `timeout 300` | 必须有 |
| 12 | 有人看着（CPU/日志/网页） | — | 必须有人 |
| 13 | 测试记录已开（时间、参数、期望） | — | 必须有 |
| 14 | 决定失败判定 | 例如"`hz` 掉出 ±10% 或 RSS 单调增长 30 s" | 必须有 |

### 9.1 PRE-RUN 一键版（可直接粘贴）

```bash
echo "== owners =="        ; pgrep -af 'gs130|mipi_cam|hobot_stereonet' || echo none
echo "== devices =="       ; for d in /dev/video* /dev/videocapture*; do [ -e "$d" ] && sudo fuser -v "$d"; done
echo "== web =="           ; pgrep -af nginx || echo none; ss -lntp | grep 8000 || echo "port 8000 free"
echo "== codec =="         ; pgrep -af 'hobot_codec|websocket' || echo none
echo "== i2c =="           ; for b in 4 6; do echo "-- bus $b"; sudo i2cdetect -y -r $b | grep -E ' 30| 32| 50| 68'; done
echo "== lib =="           ; python3 -c "import gs130;print(gs130.__version__, gs130.library_version())"
echo "== temp =="          ; cat /sys/class/thermal/thermal_zone0/temp
```

## 9. 运行后检查清单（POST-RUN，必须全部完成才算一轮结束）

| # | 检查项 | 命令 | 通过判据 |
| --- | --- | --- | --- |
| 1 | 相机节点已退出 | `pgrep -af gs130_node` | 无输出（否则 §6.3 ①②） |
| 2 | 退出是**正常**路径 | 日志里能看到 `deinit`/`close` 相关记录或至少没有非零信号退出 | 是 |
| 3 | 无相机设备持有者 | `for d in /dev/video* /dev/videocapture*; do sudo fuser -v "$d"; done` | 无输出 |
| 4 | codec / websocket / nginx 已停 | `pgrep -af 'hobot_codec|websocket|nginx'` | 无输出 |
| 5 | 端口 8000 已释放 | `ss -lntp \| grep 8000` | 无输出 |
| 6 | 浏览器页面已关闭 | 人工 | 已关闭 |
| 7 | 相机确实可再次获取 | `gs130 detect camera -b 4 6` | 两个地址都命中 |
| 8 | 收集证据 | `dmesg \| tail -30`、节点日志、`hz` 输出、RSS 曲线 | 已归档 |
| 9 | 内存无泄漏结论 | §5.4 的 RSS 观察结果 | 已记录"合格/不合格" |
| 10 | 记录本轮参数与结论 | 写入测试记录 | 已写 |
| 11 | 若出现过异常：写清恢复路径 | R-1/R-2/R-3/R-4 走到哪一步 | 已写（**不允许**"重启了一下"这种无信息记录） |

---

## 10. 三个无法完全缓解的残余风险

### R1 — SDK `timestamp_ns` 的时钟域无法在本阶段被确证

| 项 | 内容 |
| --- | --- |
| 为什么无法完全缓解 | 值来自 VPU 帧信息（优先 `trig_tv` 曝光触发沿，回退 `timestamps` / `tv`），其时钟源在 BSP 内部；我们**不能**通过读 SDK 源码确定它是不是 `CLOCK_MONOTONIC`。freeze Q-3 要求"实测确认"，但实测本身依赖板卡与相机，且不同 BSP 版本可能不同 |
| 后果 | 若判错，`header.stamp` 系统性偏移，下游 `tf`/`message_filters` **静默**错位（不报错、只是结果不对） |
| 补偿措施（必须做） | ① 采用一次性 `offset_ns` 方案（freeze §4.2）：无论时钟域是什么，**只要求它能算出一个恒定偏移**，图像与 IMU 的相对关系永远正确；② 节点启动时打印 `offset_ns` 的数值与判定依据（同源 / 非同源），并在日志里给出"设备时钟与节点时钟的关系"一句话结论；③ 前置检查清单第 12 项要求人工确认这条日志存在且数值合理（非同源时 `offset_ns` 应为常数级而非抖动） |
| 余留风险 | 若设备时钟**频率**与节点时钟有漂移（不是固定偏移），长时间运行后偏移会缓慢增长——**这一条无法自动补偿**，只能靠"单次测试不超过 5 分钟 + 长测时人工对比首尾 `stamp` 与 `now()` 的差"来发现 |

### R2 — "上一次异常退出留下的相机状态"没有 100% 可靠的探针

| 项 | 内容 |
| --- | --- |
| 为什么无法完全缓解 | 用户态能看到的信息有限：`/proc/*/fd` 只能看到**还活着的**持有者；`i2cdetect` 只能证明 sensor 在 I2C 上应答，**不能**证明 MIPI/ISP 侧空闲（这正是 H-2/H-3 的盲区）；而 D-Robotics 的句柄不落在 V4L2 的 `/dev/video*` 上，设备节点名需上板确认，无法预先写死一条"一定正确"的命令 |
| 后果 | 最坏情况判为 H-3，需要重启（M3 的最后手段） |
| 补偿措施（必须做） | ① PRE-RUN 第 1~6 项作为**分层证据**一起看（任一异常都不启动）；② 每轮开始前固定跑一次 `gs130 detect camera -b 4 6`（只读、低风险）作为"可获取"判据；③ 每轮结束后**无条件**执行 §9 后置清单——"用得干净"是防止 R2 变成真实事故的最有效手段；④ 遇到 H-2/H-3 时**按 R-1→R-2→R-3→R-4 逐级**尝试并逐级记录，禁止跳级重启 |
| 余留风险 | 存在一种情形（用户态无持有者 + I2C 应答正常 + MIPI/ISP 全局引用残留）只能靠重启解决。**能做的只是让它的概率尽量低，并保证它发生时我们手上有完整证据** |

### R3 — C 库内部的"错误路径行为"（fps=0 除零 / 尺寸不匹配热自旋）不在本包可控范围

| 项 | 内容 |
| --- | --- |
| 为什么无法完全缓解 | 这些是 `core/` 的 C++ 实现细节：`1000000000ULL / fps` 与 `goto free_frame` 后无休眠的循环。本阶段的硬约束是**不修改 `python/`**，也不在本包内重写驱动（freeze N-02）。节点侧只能"避免进入"，不能"修复" |
| 后果 | 最坏是进程崩溃（落回 H-2）或 CPU 打满（影响共享板上其他人） |
| 补偿措施（必须做） | ① 节点侧参数校验兜住最明确的坑：`fps >= 1`、RAW 必须 1088×1280（越界**不进 SDK**）；② 每次运行前 `timeout`，运行中**必须有人看 `top`**；③ 定义并写入测试记录的"热自旋判据"：**CPU≈100% + `hz`=0 + RSS 不增长 + 无新日志** → 立刻 SIGINT，之后按 §3.4 的 R-3 恢复；④ 测试轮次遵守"新参数组合先用 `fps:=1` 单帧验证，再加到 30"的分阶段纪律；⑤ 把这两个 C 侧问题作为**独立问题单**上报给 SDK 维护方（本文件只做记录与规避，不在此修改代码） |
| 余留风险 | 若真的崩溃，恢复依赖 §3.4 的 R-3；若 R-3 无效则退回 R2 的路径 |

---

## 11. 附：本文引用的关键代码事实（便于复核）

| 事实 | 出处 |
| --- | --- |
| 帧缓冲由 `malloc` 分配，所有权归调用方，用完 `free()` | `core/include/gs130.h:231,249` |
| NV12 紧排布局：`Y` 后接 `UV`，`UV` 尺寸 `width*height/2` | `core/include/gs130.h:207` |
| `TIMEOUT` 表示"当前没数据"，不写输出结构体 | `core/src/gs130.cpp:554-572`、`gs130.h:27` |
| C 侧错误路径不泄漏（`free_frame` / FIFO disposer / `Fifo` 析构） | `core/src/gs130.cpp:213-215,346-350`、`core/src/base/fifo/fifo.hpp:36-41,77-81` |
| Python 侧唯一的释放点：`owner` 的 `weakref.finalize(owner, _free, address)` | `python/gs130/_types.py:24-26` |
| `Image` 是 `np.frombuffer` 视图（零拷贝），shape 为 `(h*3//2, w)` | `python/gs130/_types.py:26-27` |
| `read_image()`：`TIMEOUT` → `None`；否则 `{"left","right"}` 或 `{"stitched"}` | `python/gs130/_device.py:139-161` |
| `stop()`/`close()` 幂等；`close()` 走 `deinit` + `destroy` | `python/gs130/_device.py:112-129`；`core/src/gs130.cpp:492-541` |
| `gs130_deinit` 会清 latch，因此 `HW_ERROR` 无需重启即可恢复 | `core/src/gs130.cpp:500` |
| `deinit` 会重新给 sensor 上电（`sensor_power(gpio,1)` = 写 `1,0,1`）——这就是免重启的硬件恢复手段 | `core/src/devices/pipeline/rdkx5/rdkx5.cpp:313-317`、`camera.c:17-42` |
| RAW 模式输出必须等于 sensor 尺寸 | 板卡已验证事实 + `python/gs130/_config.py:61-66` |
| `fps` 参与整数除法（`1e9/fps`、`1e9/fps/2`） | `core/src/gs130.cpp:228,465` |
| 相机线程错误路径无休眠（热自旋可能） | `core/src/gs130.cpp:214-216,346-350` |
| 帧时间戳来源：`trig_tv` → `timestamps` → `tv`（时钟域未定义） | `core/src/devices/pipeline/rdkx5/rdkx5.cpp:30-41` |
| preset 固定 `stereo_layout=NONE`，SDK 公开路径只出左右分离帧 | `python/gs130/_config.py:79` |
| `hobot_codec` 参数：`in_mode/out_mode`(`ros`/`shared_mem`)、`in_format/out_format`、`sub_topic`、`pub_topic`、`channel`、`input_framerate`；要求宽高 8 对齐 | D-Robotics `hobot_codec` README；1088 / 1280 均满足 8 对齐 |
| `websocket` 参数：`image_topic`、`image_type`(`mjpeg`)、`only_show_image`、`channel` | D-Robotics `hobot_websocket` README |
| `websocket.launch.py` 在 launch 构造阶段用 `ps ax` 子串匹配 + `os.system('./sbin/nginx -p .')`，且 nginx 不被 launch 管理 | D-Robotics `hobot_websocket/launch/websocket.launch.py` |
| `hobot_codec.launch.py` 内部已 include `hobot_shm.launch.py` | D-Robotics `hobot_codec/launch/hobot_codec.launch.py` |
| `hobot_shm` 的产物就是 DDS 零拷贝配置（`shm_fastdds.xml` + launch） | D-Robotics `hobot_shm` README |
| `sensor_msgs/Image.data` setter：`array.array('B')` 直接持有，其它输入 `array.array('B', value)` 逐元素拷贝 | ROS 2 humble `sensor_msgs/msg/_image.py:249-270` |
| `qos_profile_sensor_data` = `BEST_EFFORT` / `VOLATILE` / `KEEP_LAST` / depth 5 | `rclpy/qos.py:430`；本包按 freeze §2.2 逐话题覆写 depth |

---

*本文件只描述规则与恢复流程，不修改 `python/`、不修改 `core/`、不新增 ROS 代码。文中的命令全部为可粘贴形式；标注为"上板确认"的项（相机设备节点名、`hobot_codec_encode.launch.py` 是否 include `hobot_shm`、温度节点名）必须在首次执行时由执行者核对并回填到本文件的表格中。*
