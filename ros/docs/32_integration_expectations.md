# GS130 ROS 2 (TROS) 生态集成期望评估

- 文档编号：32（Ecosystem & Integration Expectations）
- 角色：UX-3，用户评估组 · 生态与集成评估
- 状态：**实施前交付**（Pre-implementation evaluation）；**第 2 版：已并入主架构师在 RDK X5 上的实测结论 E1–E7**
- 评估对象：`gs130_ros` v0.1.0（拟议包），上游 `gs130_sdk` 仓库 `VERSION = 0.0.1`
- 平台：RDK X5 + TROS Humble（`/opt/tros/humble`），ROS 2 Humble
- 上游契约：`ros/docs/11_interface_freeze.md`（FROZEN v0.1.0）、`ros/docs/10_architecture.md`、`ros/docs/21_acceptance_criteria.md`
- 平台侧只读证据：`core/include/gs130.h`、`core/src/devices/pipeline/rdkx5/*`、`python/gs130/*`、`python/test/test_gs130.py`

> **本评估的取证边界（必须先读）**：
> **第 1 版**由 UX-3 撰写，**未登板、未运行相机、未运行任何 ROS 节点**，全部结论基于源码级证据，端到端结论一律标 `[待板端验证]`。
> **第 2 版**并入主架构师在真实 RDK X5 上执行的实测结论（E1–E7）。这些实测**关闭了第 1 版列出的三个冻结前问题中的两个**（Q2 QoS、Q3 时钟域），
> 并**推翻了一条源码推断**（NV12 的 `height` 语义，见 X3）。凡由实测关闭的条目，本文一律改标 `[实测-板端]` 并在
> §0.4 保留原始数值；**凡是实测没有覆盖的结论，仍然保持 `[待板端验证]`，不得因为"相关实验成功了"而被顺带当作已验证。**
>
> 阅读本文时的正确姿势：`[实测-板端]` = 有测量数据支撑的**集成契约**；`[已验证-源码]` = 代码确实如此；`[推断]` = 逻辑推导；
> `[待板端验证]` = **尚未被任何实验覆盖**。缺任何一类标记的地方都可以质疑。

---

## 0. 证据等级标记与全局事实表

### 0.1 标记约定

| 标记 | 含义 |
|---|---|
| `[实测-板端]` | **在真实 RDK X5 上测量得到**（主架构师实验 E1–E7）；含原始数值，是本文最高等级证据 |
| `[已验证-源码]` | 在本地仓库源码中逐行核对过（给出文件:行） |
| `[已验证-上游源码]` | 在 D-Robotics 官方公开仓库源码中逐行核对过（给出仓库路径/行） |
| `[已验证-官方文档]` | 官方文档 / README 明文写出 |
| `[推断]` | 由已验证事实推导，逻辑链完整但未直接观察 |
| `[待板端验证]` | **必须**在板卡上实测才能确认；本文给出命令与判定条件 |
| `[已解决]` | 第 1 版的开放问题/冲突，已被 E1–E7 的实测关闭 |
| `[已被实测推翻]` | 第 1 版的结论与实测相反，本文已改正 |
| `[冲突]` | 本项目文档之间、或本文与既有文档之间的**明确矛盾**，必须由接口评审裁定 |
| `[不可得]` | 本评估环境无法获取的证据 |

### 0.2 与集成直接相关的、已被源码确认的事实

| # | 事实 | 证据 |
|---|---|---|
| F1 | SDK 支持 4 种拼接布局 `GS130_STEREO_LAYOUT_{LEFT_RIGHT,RIGHT_LEFT,TOP_BOTTOM,BOTTOM_TOP}`，但 Python 受支持入口 `Config.preset()` **硬编码 `StereoLayout.NONE`**（双目分离帧） | `core/include/gs130.h:54-61`；`python/gs130/_config.py:79` |
| F2 | `TOP_BOTTOM` 布局的**内存平面顺序**为：`Y_left \| Y_right \| UV_left \| UV_right`，且 `uv[L]=base+2*w*h`、`uv[R]=uv[L]+w*h/2` | `core/src/gs130.cpp:282-291` |
| F3 | 官方 `mipi_cam` 的 `dual_combine` 拼接帧使用**完全相同**的平面顺序（先全部 Y 再全部 UV），尺寸 `w × (n*h)`，即**上下堆叠** | `hobot_mipi_cam/src/hobot_mipi_cap.cpp:125-145` |
| F4 | 官方双目启动脚本 `hobot_stereonet/script/run_cam.sh` 用 `image_width=1280 image_height=1088 rotation=90 gdc_enable=False dual_combine=1`，即每目 `1280×1088`、拼接帧 `1280×2176` | `hobot_stereonet/script/run_cam.sh` |
| F5 | `hobot_codec_republish` 在 `in_mode=ros` 时用 `create_subscription<sensor_msgs::msg::Image>(sub_topic, PUB_QUEUE_NUM, cb)`，`PUB_QUEUE_NUM = 5`，**即默认 QoS（RELIABLE + KEEP_LAST depth 5）**；只有 `in_mode=shared_mem` 才用 `SensorDataQoS()`（BEST_EFFORT） | `hobot_codec/src/hobot_codec_node.cpp:25,366-383,406-412` |
| F6 | `websocket` 只接受 `sensor_msgs/msg/CompressedImage`（mjpeg）或 `hbm_img_msgs/msg::HbmMsg1080P`（mjpeg_shared_mem），监听 nginx `:8000` | `hobot_websocket/README_cn.md:80,96-98,257` |
| F7 | SDK 输出帧是**紧排 NV12（无 stride padding）**：`Pipeline::get_frame()` 按行 `memcpy`，行 stride 由调用方给定为 `width` | `core/src/devices/pipeline/rdkx5/rdkx5.cpp:377-383`；`core/src/gs130.cpp:250-261` |
| F8 | SDK 帧缓冲由 `malloc()` 分配、所有权交给调用方，Python 侧在 numpy 数组被回收时 `free()` | `core/include/gs130.h:231,249`；`python/gs130/_types.py:11-25`；`core/src/gs130.cpp:429-432` |
| F9 | EEPROM 标定为**鱼眼 4 参数**模型（`k1..k4`，`DistModel.FISHEYE`），外参为 `camera_left/right/imu` 在参考系下的绝对位姿 `R,T`，`T` 单位米 | `core/src/devices/eeprom/union_stereo_imu_fisheye_v1p2r0n4.cpp:17-23,51-54`；`core/include/gs130.h:309-348` |
| F10 | `install_angle` 由 EEPROM 驱动写为 **0**，因此 `mode=resize` 时 GDC 节点**不启用**，`mid_w×mid_h` 直接等于 `sensor_width×sensor_height = 1088×1280`（**纵向**） | `union_stereo_imu_fisheye_v1p2r0n4.cpp:139`；`rdkx5.cpp:178-247` |
| F11 | 非 RAW 模式要求 `mid` 与输出满足**整除**关系，否则 `Status::Unsupported → GS130_UNSUPPORTED`：`(out_w*h) % out_h == 0`（当 `mid` 相对更宽）或 `(mid_w*out_h) % out_w == 0` | `core/src/devices/pipeline/rdkx5/vse.c:13-25`；`rdkx5.cpp:240-247` |
| F12 | `mode=rect` 下 `stereo_rectify()` 会**原地改写**标定的内参与外参（虚拟双目平行系、虚拟 K），并先做 GDC Map 搜索再走 VSE ROI/缩放 | `core/src/base/rectify/rectify.hpp:20-27`；`rdkx5.cpp:185-235,263-272` |
| F13 | SDK 在 `init()` 阶段已把 VSE ROI+scale 写回 `fx,fy,cx,cy,K`；`camera_intrinsics()` 返回的 K 已处于**最终输出像素坐标系** | `rdkx5.cpp:263-272`；`python/gs130/_device.py:189-201` |
| F14 | IMU 时间戳已由 SDK 对齐到相机时钟域，可直接与图像时间戳相减 | `core/include/gs130.h:263`；`core/src/gs130.cpp:309-310,464-465` |
| F15 | `hbm_img_msgs/msg/HbmMsg1080P` 的载荷上限为 `uint8[6220800]`（=1920×1080×3），官方说明为"1080P RGB24 位以下分辨率" | `hobot_msgs/hbm_img_msgs/msg/HbmMsg1080P.msg`；`hobot_msgs/hbm_img_msgs/README_cn.md` |
| F16 | `hobot_stereonet` 用 `cv::FileStorage` 读 Kalibr 风格 YAML（`cam0/cam1` 的 `intrinsics/resolution/distortion_model/distortion_coeffs`、`T_cn_cnm1`），`custom` 模式必须提供该文件 | `hobot_stereonet/src/stereo_rectify.cpp:21,187-310`；`stereonet_component.cpp:165-184` |
| F17 | `hobot_stereonet` 的 NV12 输入被当作 **`width × (height/2)` 单目、上下堆叠**：`single_img_w = msg->width; single_img_h = msg->height/2`，且上下两半各自独立取 Y/UV（`uv_base = data + width*height`） | `stereonet_component.cpp:869-887,1036-1063` |

### 0.3 板端实测事实（E1–E7，本文的**最高等级证据**）

> 下表是主架构师在 RDK X5 上实际执行得到的结论，**取代本文第 1 版中对应的源码推断与开放问题**。
> 每条都保留了可核对的原始数值；本文后续所有推荐均以此表为前提。

| # | 实测结论 | 原始数据 / 证据 | 在本文中的作用 |
|---|---|---|---|
| **E1** | **与 `hobot_codec` 的集成要求我方图像 publisher 必须是 `RELIABLE`。** 用 `BEST_EFFORT` 时 codec 打印 `New publisher discovered ... offering incompatible QoS. No messages will be sent to it. Last incompatible policy: RELIABILITY_QOS_POLICY`，并**静默收不到任何数据** | codec 日志原文；无数据到达 | **`[已解决]` Q2 / X2**；推翻"图像一律用 `SensorDataQoS`"的 ROS 惯例；成为本文最强约束 M2 |
| **E2** | **codec 把 `sensor_msgs/Image.height` 解释为真实图像高度。** 若按"打包后的 NV12 高度 `H*3//2`"发布，codec 打印 `init_pic_h_: 1080, alined_pic_h_: 1088` 后 **SEGFAULT** | codec 日志 + 崩溃 | **`[已解决]` X3**，判定为 **`height = H`（真实高度）**；本文第 1 版此处的"两种语义各自自洽"论述被实测推翻，见 §1.2 |
| **E3** | **零拷贝问题已关闭：v0.1.0 不使用 `hobot_shm` 零拷贝。** 正确路径的开销很小：`array.array("B") + frombytes` 处理 1.38 MB 耗时 **1.36 ms**；节点整帧成本 **build 0.53 ms / fill 1.28 ms / publish 5.07 ms @30 fps**。而朴素写法 `msg.data = frame.tobytes()` 耗时 **1103.87 ms**（rclpy 逐元素校验）。另外 `hobot_shm` 的 `shm_fastdds.xml` **不设置 reliability**，因此**不能指望它修 QoS** | 上述毫秒级测量 | **`[已解决]` Q（§4）**；给出 §4.3 的可测量差异表；明确 `hobot_shm` 与 E1 无关 |
| **E4** | **复用链路在全速率下工作**：我方 publisher → 既有 `hobot_codec` → 既有 `websocket`，`/image_combine_raw` **29.966 Hz**、`/image_combine_jpeg` **29.974 Hz**，codec 日志 `Sub imgRaw fps 29.98` / `Pub img fps 29.98`；Web UI `http://127.0.0.1:8000/` 返回 **HTTP 200** | 频率与 HTTP 状态码 | M9 从"必须验证"升级为**已有基线证据**；同时给出 nginx 孤儿进程的清理要求（§7.5） |
| **E5** | **时间戳：SDK 返回 `CLOCK_MONOTONIC`（自开机），不是 Unix epoch。** 实测 frame `9546841822000` vs monotonic `9540690105753`（约 9480 s uptime），相对墙钟差 **-1.79e18 ns**；图像与 IMU **同域**（`imu - frame = 8.6 ms`）。因此把原始 SDK 时间戳直接塞进 `header.stamp` 会**静默破坏 tf / message_filters / rosbag** | 上列数值 | **`[已解决]` Q3（时钟域部分）**；确立 M4 的换算规则与"至多一个帧周期"的残余不确定度；直接决定 §5 的 TF 决策 |
| **E6** | **硬件拼接可用且便宜**：在 `Config.preset()` 之后设置 `stereo_layout = StereoLayout.LEFT_RIGHT`，SDK 直接返回拼接帧，`640×480` 配置下为 **(720, 1280)** 的 numpy 形状，**与 `/image_combine_raw` 的 `width=2W` 约定完全吻合**。**RAW 模式不能用于拼接**（RAW 强制输出 `1088×1280`） | 实测形状 `(720,1280)` | 网页链路**无需在驱动侧手工拼接**；`[已解决]` 第 1 版 §1.2 关于"手工拼接 1 次拷贝"的必要性讨论；但**引入两条新约束**，见 §1.2 与 M11 |
| **E7** | **`websocket` 用同一端口 8000 上的 `channel` 参数区分多路**（D-Robotics 的 132GS launch 用 channel 0 和 1），**不需要第二个端口**；板端环境已具备 `colcon`、`rclpy`、`sensor_msgs`、`geometry_msgs`、`tf2_msgs`、OpenCV 4.11.0、numpy 1.26.4 | 板端环境清单 | 修正第 1 版"新链路要换 8080 端口"的表述（§2.5 保留但降级为"另一套 viewer 的端口"）；`tf2_msgs` 已存在 ⇒ §5 的 TF 建议**无新增依赖成本** |

### 0.4 契约级冲突状态（实测后更新）

| # | 冲突 | 状态 | 结论 |
|---|---|---|---|
| X1 | `/image_combine_raw` 的**拼接布局** | **`[已解决]`（E6）** | 实测证明 `StereoLayout.LEFT_RIGHT` 的输出形状与 `width=2W` 约定吻合。第 1 版所说的"与 `hobot_stereonet` 期望的上下堆叠相反"**依然成立**（那是 stereonet 的独立约束，见 §2），但它**不再是阻塞默认链路的问题**——默认链路（codec + websocket）与水平拼接完全相容（E4 已端到端验证） |
| X2 | 图像话题 **QoS** | **`[已解决]`（E1）** | **必须 `RELIABLE`**。冻结书 §2.1 的 `BEST_EFFORT` 是**错的**，必须改；`22_risk_and_safety.md` §4.3 表中"codec 订阅为 BEST_EFFORT"一行也**是错的**（与该文档自己的"pub BEST_EFFORT ↔ sub RELIABLE 永不匹配"规则自相矛盾） |
| X3 | `sensor_msgs/Image.height` 的 NV12 语义 | **`[已解决]`（E2）** | **`height = H`（真实高度）**，与 `21_acceptance_criteria.md` 的 D-01 一致，与冻结书 §3.1.1/§3.1.3 的 `height = eh*3//2` 冲突。**按冻结书写会 SEGFAULT 既有节点**，因此这不是取舍问题，是必须改的 bug |
| X4 | 外参静态 TF：冻结书禁 vs 验收书 P0 要求 | **`[部分解决]`（E5/E7）** | 第 1 版列出的两条反对理由中，"时间戳时钟域不确定"已被 E5 关闭，"新增依赖成本"已被 E7 关闭（`tf2_msgs` 已在板端）。本文因此**上调建议**：见 §5.5 |
| X5 | frame 命名不一致 | `[冲突]` 仍开放 | 建议与 X4 同批评审修正 |
| X6 | 话题名空间不一致 | `[冲突]` 仍开放 | 同上 |

> **X1–X6 不是"文档瑕疵"，而是集成接口本身的分叉。** 其中 X2/X3 已经**由实测给出唯一答案**（不再是选择题）；
> X4 的两条反对理由已被实测削弱；X5/X6 建议作为同一次评审的一揽子修正，不在本文展开。


---

## 1. 现有 D-Robotics 节点集成评估矩阵（对应要求 1）

### 1.1 总表

| # | 既有节点 | 与本包的耦合点 | 我方必须提供什么 | v0.1.0 是否在范围内 | 证据等级 |
|---|---|---|---|---|---|
| I1 | `hobot_codec`（`hobot_codec_republish`） | 订阅 NV12 图像 → 发布 jpeg | 话题逐字 `/image_combine_raw`、`sensor_msgs/Image`、`encoding` 逐字 `"nv12"`、**`height` = 真实高**、**QoS = `RELIABLE`** | **在范围内**（默认链路，**E4 已端到端验证**） | `[实测-板端]` E1/E2/E4 |
| I2 | `websocket` | 订阅 jpeg → nginx `:8000` 网页 | **不直接对接**：只要求 I1 产出的 jpeg 分辨率/时间戳自洽 | **在范围内**（默认链路，经 codec 间接；**E4 已验证 HTTP 200**） | `[实测-板端]` E4 + `[已验证-官方文档]` F6 |
| I3 | `hobot_shm` | 设置 Fast-DDS 共享内存传输环境 | 不要求我方改动；作为 launch 开关暴露 | **在范围内（仅作为环境开关）** | `[已验证-官方文档]` |
| I4 | `hobot_stereonet` | 订阅双面 stereo 图像 → 深度/点云 | **上下堆叠布局 + 有效 `CameraInfo.P` 或 Kalibr YAML** | **不在范围内**（见 §2 结论） | F16/F17 + §2 |
| I5 | `hobot_yolo_world`（本评估选定的感知节点） | 订阅 `sensor_msgs/Image` → 推理 → `ai_msgs` | 一路普通图像 + 正确 `frame_id`；`ros_img_sub_topic_name` 可指向我方话题 | **不在范围内（v0.1.0）**，属 follow-up | `[已验证-官方文档]` |
| I6 | `hobot_cv` | NV12 处理库（resize/rotate/crop），供第三方调用 | 无需对接（库级，不是节点） | **不在范围内** | `[已验证-官方文档]` |
| I7 | `hobot_hdmi` / `hobot_visualization` | 显示 / ai_msgs 可视化 | 本包不产出 `ai_msgs`，无可对接 | **不在范围内（明确非目标）** | `[已验证-官方文档]` |
| I8 | `mipi_cam` | **不组合，互斥** | 无（见 §6） | **互斥**，不在范围内 | `[已验证-源码]` §6 |

### 1.2 I1 `hobot_codec` — 在范围内，且是**唯一的强制链路**

**它是什么**：`hobot_codec_republish` 是一个通用转码节点，`in_mode`/`out_mode`/`in_format`/`out_format`/`sub_topic`/`pub_topic` 全部参数化（`hobot_codec/README_cn.md`），并非只服务于 `mipi_cam`。

**它对我们的要求（逐条，实测后的最终版）**：

| 维度 | 要求 | 我方必须的实现 | 判定 |
|---|---|---|---|
| 话题名 | `sub_topic` 逐字等于我方图像话题（launch 固定为 `"/image_combine_raw"`） | 逐字同名，绝对名、无命名空间 | ✅ `11_interface_freeze.md:438` 一致 |
| 消息类型 | `in_format != jpeg/h264/h265` 时订阅 `sensor_msgs/msg/Image` | 发布 `sensor_msgs/Image` | ✅ |
| 编码字符串 | `in_format: "nv12"`；消息 `encoding` 逐字 `"nv12"` | 逐字小写 `"nv12"` | ✅ |
| **QoS** | **`RELIABLE`**（**E1 实测**：`BEST_EFFORT` 时 codec 报 `offering incompatible QoS ... Last incompatible policy: RELIABILITY_QOS_POLICY` 并**静默收不到**） | 图像话题必须 `RELIABLE` + `VOLATILE` + `KEEP_LAST`，depth ≥ 5（与 codec 的 `PUB_QUEUE_NUM = 5` 对齐） | ❌ **冻结书 §2.1 的 `BEST_EFFORT` 必须改**（X2 已由 E1 关闭） |
| **`height` 语义** | **真实图像高度 `H`**（**E2 实测**：按 `H*3//2` 发布 → codec 打 `init_pic_h_: 1080, alined_pic_h_: 1088` 后 **SEGFAULT**） | `width = 真实宽`、`height = 真实高`、`step = width`、`len(data) = width*height*3//2` | ❌ **冻结书 §3.1 的 `height = eh*3//2` 必须改**（X3 已由 E2 关闭，且属**崩溃级 bug**） |
| 像素布局 | 紧排 NV12 单平面视图：Y 平面在 `[0, width*height)`，UV 平面紧随其后；`uv_offset = width*height` | 由 SDK 紧排输出 +（可选）SDK 硬件拼接直接满足（F7、E6） | ✅ |
| 分辨率 | ROS 模式无硬上限；`shared_mem` 模式受 `HbmMsg1080P`=`1920×1080×3` 上限 | `640×480` 拼接后 `width=1280,height=480,len=921,600` | ✅（§4.4） |
| 时间戳 | codec 转码后 jpeg 是否保留 `header.stamp` | **`[待板端验证]`**：E4 只证明了频率与 HTTP 200，**没有验证 jpeg 的 stamp 与图像一致** | ⚠️ 未覆盖 |
| 对齐 | codec 内部有 `alined_pic_h_` 概念（E2 日志可见），说明存在行对齐处理 | 建议文档给出"宽度取 16 的倍数"的经验规则 | `[待板端验证]` |

**E2 的完整含义（必须写进实现注释与用户文档）**：

```
✅ 正确：width = 640, height = 480, step = 640, len(data) = 640*480*3//2 = 460800
❌ 错误：width = 640, height = 720, step = 640, len(data) = 460800   ← codec 认为图高 720，SEGFAULT
✅ 拼接：width = 1280, height = 480, step = 1280, len(data) = 1280*480*3//2 = 921600
```

> **本文第 1 版在这里的论述已被实测推翻**：第 1 版认为"`height=eh*3//2` 与 `height=H` 两种语义**各自自洽**，可以由评审取舍"。
> E2 证明**并非各自自洽**——`height=eh*3//2` 会让既有节点崩溃，因此 `height=H` 是**唯一正确**的实现。
> 第 1 版那条"数学自洽性"论证只在"UV 偏移量"这一个点上成立，**掩盖了 codec 还有独立的高度校验**，这是一个只看源码无法发现的教训。

**关于 nginx 与 codec 的生命周期（E4 附带结论）**：`websocket.launch.py` 用 `os.system` 启动 nginx，**launch 退出后 nginx 变成孤儿进程继续存活**并占用 `:8000`。因此：
- 用户文档必须给出显式清理命令（见 §7.5）；
- 测试脚本在"第二次运行"前必须先清理，否则会被误判为"端口占用/启动失败"。

### 1.3 I2 `websocket` — 在范围内，但**零直接耦合**

**它是什么**：Web 服务（nginx `:8000`），订阅 jpeg（`sensor_msgs/CompressedImage` 或 `hbm_img_msgs`）与 `ai_msgs/PerceptionTargets`，按 `header.stamp` 做图文匹配后推送给浏览器。

**E4 实测基线（这是 v0.1.0 唯一的对外可见交付，已有证据）**：

| 观测量 | 实测值 |
|---|---|
| `/image_combine_raw` 频率 | **29.966 Hz** |
| `/image_combine_jpeg` 频率 | **29.974 Hz** |
| codec 日志 | `Sub imgRaw fps 29.98` / `Pub img fps 29.98` |
| Web UI | `http://127.0.0.1:8000/` → **HTTP 200** |

⇒ **"整链不掉帧"这一点不再是假设**，而是基线事实。任何回归如果频率明显低于 ~29.97 Hz 或 HTTP 非 200，即为**实现退化**，不是"环境差异"。

**它对我们的要求**：
1. **无直接要求**——它只订阅 I1 产出的 `/image_combine_jpeg`。本包**不得**自建 HTTP/WebSocket 服务（冻结书 §0.2、`21_acceptance_criteria.md` `AC-SRC-05`）。
2. 间接要求：**发布频率与时间戳必须稳定**（它按 `header.stamp` 匹配图文）。E5 已给出时钟域的正确答案；文档必须给出可验证性质：
   ```bash
   ros2 topic echo /image_combine_raw --field header.stamp --once   # 连续采样
   # 要求：stamp 严格单调递增；相邻差 ≈ 1/fps（±10%）；与 `date +%s%N` 同量级（E5 的偏移换算生效）
   ```
3. **`channel` 参数在同一端口 8000 上区分多路（E7 实测）**：D-Robotics 132GS 的 launch 用 channel 0 与 1。因此：
   - **不需要第二个端口**；第 1 版 §2.5 中"网页端口从 8000 变成 8080"的说法只适用于**另一套 viewer**（`pointcloud_web_viewer`），**不是**我们接入 stereonet 的必要代价，本文已相应下调该论述的措辞。
   - 我们的文档必须写清：`channel 0` = `/image_combine_raw` 拼接帧的**左半部分 = 左目**（与冻结书 §6.2 一致）。
4. **`only_show_image` / `smart_topic` 语义**：无智能话题时，`smart_topic` 应与 `image_topic` 相同（冻结书 `11:453` 已如此处理），否则 websocket 会等待一个永不出现的话题。

**风险（`[待板端验证]`）**：`websocket` 的 `output_fps`（launch 写 `30`）与 `hobot_codec` 的 `output_framerate: -1` 叠加时，实际网页帧率 = `min(fps, output_fps)`。`fps:=15` 的用户必须同时把 `web_output_fps` 调低，文档需写清这条"两个抽帧点"。

**E4 附带的生命周期陷阱（必须写进文档与测试脚本）**：nginx 由 `websocket.launch.py` 通过 `os.system` 启动，**launch 退出后作为孤儿进程继续存活并占用 `:8000`**。清理见 §7.5。

### 1.4 I3 `hobot_shm` — 在范围内，但只是**环境开关**，且**不能用来修 QoS**

**它是什么**：`hobot_shm` 不发布话题、不处理图像，它只提供 `config/shm_fastdds.xml` 与 `launch/hobot_shm.launch.py`，用于把 Fast-DDS 配置为共享内存传输（`hobot_shm/README.md`）。

**E3 实测结论（直接回答第 1 版的零拷贝问题）**：
- `hobot_shm` 的 `shm_fastdds.xml` **不设置 reliability** ⇒ **它不能、也从未被指望用来修复 E1 的 QoS 问题**。E1 的修复方式是**在 publisher 上显式声明 `RELIABLE`**，与 `hobot_shm` 无关。
- 我们的帧是普通 CPU 缓冲（F8），**零拷贝不可达**；正确的 plain-ROS 路径开销很小（见 §4.3 的完整数字）。
- **结论：v0.1.0 不使用 `hobot_shm` 零拷贝**，`hobot_shm` 仅作为既有 C++ 节点之间的传输环境开关保留。

**对我们的要求**：**无**。冻结书 §6.5 的判断（"本节点不依赖共享内存传输"）**与源码事实和 E3 实测一致**。

**副作用提醒（`[推断]`）**：共享内存传输生效时，像素数据会多一次"复制进 shm 段"的动作。它**不会**让链路零拷贝，因此用户若以"省 CPU"为目的开启它，**可能观察到 CPU 持平或略升**。

### 1.5 I5 `hobot_yolo_world` — 本评估选定的感知节点；**不在 v0.1.0 范围**

**选它的理由**：它是 D-Robotics 生态中**唯一直接吃普通图像话题**、且对 `frame_id`/分辨率没有 hbmem 硬约束的感知节点，因而最能检验"我们的普通图像消息是否能被第三方感知节点消费"这一最基础的集成能力。相对地：
- `tros_mot_node` 订阅的是 `ai_msgs/PerceptionTargets`（**检测结果**，不是图像），与相机包没有直接接口（`mot/README_cn.md:76-92`），不适合作为集成对端；
- `hobot_stereonet` 有布局强约束（§2），不适合作为"通用感知节点"样本；
- `hobot_visualization` 只做 `ai_msgs` → `visualization_msgs`，与本包零交集。

**它对我们要求什么**（`hobot_yolo_world/README_cn.md:114-133`）：

| 维度 | 要求 | 我方状态 |
|---|---|---|
| 话题 | `ros_img_sub_topic_name`（默认 `/image`）可指向我方话题；需 `feed_type:=1` | ✅ 可指向 `/image_left_raw`（需 `publish_per_eye:=true`） |
| 消息类型 | `sensor_msgs/msg/Image` | ✅ |
| 编码 | 非共享内存订阅路径需 BGR8/RGB8（`is_shared_mem_sub:=0`）；共享内存订阅**只支持 NV12** | ⚠️ 我们只发 NV12。→ 走 ROS 路径时**需要先经 `hobot_codec` 转 bgr8**（`out_format:=bgr8`），或用其 `is_shared_mem_sub:=1`（但那要求 `HbmMsg1080P`，我们发不出） |
| 分辨率 | 模型固定输入（通常 640×640 附近），节点自行 resize | ✅ |
| QoS | `[不可得]`：本次无法取得其订阅 QoS 源码 | `[待板端验证]`，用 `ros2 topic info -v` 看兼容性 |
| `frame_id` | 它会下发自己的 `ai_msgs`；图像与结果均需一致 `frame_id` 供 `websocket` 匹配 | ⚠️ 我方 `frame_id` 与 yolo_world 输出必须一致，需文档说明 |

**结论**：**不在 v0.1.0 范围内**。理由是它引入一条"NV12 → bgr8 → 推理"的额外链路，其正确性完全属于 `hobot_codec` 的能力范围，本包不应为它新增话题或编码（否则违反冻结书 §0.2"不做转码"）。建议列为 follow-up，并在用户文档中给出一条**可直接复制的组合命令**（见 §5.7）。

---

## 2. `hobot_stereonet` 深度评估（对应要求 2）

> 本节结论：**v0.1.0 不应尝试接通 `hobot_stereonet`**；应作为 follow-up，并先在板端完成一组**最小验证**。
> 理由不是"实现太难"，而是**输入布局与冻结契约不一致**，且该项已被冻结书明文列为非目标。

### 2.1 它到底消费什么（源码事实，不是猜测）

`hobot_stereonet`（RDK X5，`develop` 分支）的输入契约如下，全部来自源码逐行核对：

**(a) 话题与参数**
- `stereo_image_topic` 默认 `/image_combine_raw`，`camera_info_topic` 默认 `/image_combine_raw/right/camera_info`，`left_camera_info_topic` 默认 `/image_combine_raw/left/camera_info`（`launch/stereonet_model.launch.py:49-51`）。
- 订阅：`create_subscription<sensor_msgs::msg::Image>(stereo_image_topic_, 1, cb)`（`stereonet_component.cpp:558-559`）→ **默认 QoS（`RELIABLE`）+ depth 1**。
  **`[已解决]`（E1 + E7）**：第 1 版曾把"我方 `BEST_EFFORT` 与 stereonet 的 `RELIABLE` 不相容"列为阻塞项；E1 实测已把我方契约改为 `RELIABLE`，因此**这条阻塞自动消失**——两侧都是 `RELIABLE`，可以匹配。
  ⚠️ **但不要过早乐观**：E1/E7 的实测环境**没有启动过 `hobot_stereonet`**，因此"两侧 QoS 相容"是**由两条独立的实测事实推导**（我方=RELIABLE；stereonet=源码默认 RELIABLE）而**不是直接观察**。标记为 `[推断]`，需在 follow-up 的第一次实跑中确认。

**(b) 图像布局：上下堆叠，不是左右拼接**
- `int single_img_w = msg->width; int single_img_h = msg->height / 2;`（`stereonet_component.cpp:871-872`）
- 上下两半各自独立取 Y/UV：`uv_base = stereo_data + stereo_msg->width * stereo_msg->height`，`left_uv = uv_base`，`right_uv = uv_base + uv_size`（`resize_stereo_nv12_image` L1014-1030、`split_stereo_nv12_image` L1036-1063）
- 图像预处理始终按**行切分**：`left_bgr = stereo_bgr.rowRange(0, single_img_h)`、`right_bgr = stereo_bgr.rowRange(single_img_h, height)`（L883-886、L936-941）
- 左右顺序由一次 ORB 判定自动纠正（`judge_top_is_left_by_ORB`，第 5 帧，L986-989）→ **顺序不是硬约束**，但**布局是硬约束**。

**(c) 支持的编码**：`"nv12"`、`"rgb8"`、`"bgr8"`；其他编码直接 `RCLCPP_ERROR("unsupported image encoding")`（L928-982）。

**(d) 标定输入：两条互斥路径**
| `calib_method` | 输入 | 细节 |
|---|---|---|
| `none`（默认） | 订阅两路 `CameraInfo` | `fx=msg->p[0]`、`fy=msg->p[5]`、`cx=msg->p[2]`、`cy=msg->p[6]`、**`baseline = abs(msg->p[3]/msg->p[0])`**、`doffs=msg->p[11]`（L708-716）→ **`P` 必须非 0，否则 `CameraIntrinsic::is_valid()` 为 false，回调直接 `return`，节点静默无输出** |
| `custom` | Kalibr 风格 YAML（`stereo_calib_file_path`） | `cv::FileStorage` 读 `cam0/cam1` 的 `intrinsics/resolution/distortion_model/distortion_coeffs`、`T_cn_cnm1`；支持 `radtan` / `equidistant` / `mei`（`stereo_rectify.cpp:21,187-310`） |

**(e) 模型输入尺寸**：由 `stereonet_model_file_path` 指向的模型决定（`config/` 下有 `640_480`、`544_448`、`320_256`、`1280_704` 等多种），节点内部自动 resize 到模型输入。

**结论 (a)-(e)**：`hobot_stereonet` 期望的是 D-Robotics 自己的拼接约定 = **上下堆叠（`width = 单目宽`、`height = 2 × 单目高`）**。这与 F3/F4（`mipi_cam dual_combine` 上下堆叠）完全一致；与我们**冻结的水平拼接**（`width = 2 × 单目宽`）不一致。

> **`[已解决]`（E6）—— 这一条比第 1 版乐观得多，但方向要说清楚。** E6 实测证明两件事：
> 1. **SDK 能在 RESIZE 模式下直接输出拼接帧**（设置 `stereo_layout` 即可，无需驱动侧手工拼接、无需额外拷贝）；
> 2. **RAW 模式不能拼接**（RAW 强制输出 `1088×1280`）。
>
> 由此得到一个第 1 版**没有看到**的结论：**stereonet 所需的上下堆叠帧，用 `StereoLayout.TOP_BOTTOM` + `mode=resize` 可以由 SDK 拼出来**——因为"上下堆叠"正是 SDK 支持的另一种布局，而 RESIZE 模式允许拼接。于是：
> - **web 链路用 `LEFT_RIGHT`**（E6/E4 已验证，`width = 2W`）；
> - **stereonet 链路用 `TOP_BOTTOM`**（`width = W`、`height = 2H`），由 SDK 拼接，**不需要一次额外的 90° 旋转**（第 1 版 §2.3 理由 4 的判断需要修正：纵向口径本身就是 stereonet 想要的朝向，问题只是**选哪种布局**，而不是"朝向不对"）。
> - ⚠️ **但 `LEFT_RIGHT` 与 `TOP_BOTTOM` 是同一次 `gs130_init()` 里定死的**（`stereo_layout` 在 init 时固定，`gs130.h:242-243`），**同一个进程不能同时输出两种布局**。因此这需要**第二路话题 + 独立的一次 SDK 读帧语义**（详见 §2.4 路径 A′），而不是"改一个字段就同时满足两边"。

### 2.2 用我们的图像喂它，必须同时为真的 7 个条件

| # | 条件 | 状态（E1–E7 之后） | 验证方式 |
|---|---|---|---|
| C1 | 输入话题上的图像是 **上下堆叠**（`width = 单目宽`、`height = 2 × 单目高`） | ⚠️ **`[已解决]` 可行性（E6）**：`StereoLayout.TOP_BOTTOM` + `mode=resize` 可产出；但**与默认的 `LEFT_RIGHT` web 链路互斥**（同一次 init 只能选一种布局），需第二路话题 | `ros2 topic echo <stereo_topic> --once --field width` ⇒ 期望 `单目宽` |
| C2 | `encoding` 为 `nv12`（或 bgr8） | ✅ 已是 `nv12` | `--field encoding` |
| C3 | **至少一路有效 `CameraInfo`**，且 `P[0] = fx ≠ 0`、`P[3] = -fx*baseline_m` | ❌ **仍开放**：冻结书规定 `P` **全 0**（`11:254`）；E1–E7 **未覆盖**此路径 | `ros2 topic echo --once /image_left/camera_info --field p` |
| C4 | baseline 必须来自 **`‖T_left - T_right‖`**（米），且与图像分辨率下的 `fx` 同坐标系、符号正确 | ❌ **仍开放**：符号与坐标系规则未定义，E1–E7 **未覆盖** | 用 SDK `relative_T(CAMERA_LEFT, CAMERA_RIGHT)` 算模长，与 `abs(p[3]/p[0])` 对比 |
| C5 | 每目分辨率与模型输入的比例合理 | ⚠️ 部分改善（E6）：`mode=resize` 下每目分辨率可自由选（如 `640×480`、`544×640`），不再被 RAW 的 `1088×1280` 锁死 | 实测视差是否可用 |
| C6 | QoS 相容 | ✅ **`[已解决]`（E1 + E7）**：E1 已把我方改为 `RELIABLE`，stereonet 订阅为默认 `RELIABLE` ⇒ 相容。属 `[推断]`（两侧事实各自实测/源码核对，但**未同场实跑**） | `ros2 topic info -v` + stereonet 日志 "receive stereo image" |
| C7 | 与 `mipi_cam` 不抢资源（stereonet launch 默认会拉起它） | ⚠️ **仍开放，且是硬风险**：官方 launch 默认 `use_mipi_cam=True`（`stereonet_model_web_visual.launch.py:41-44`）；E1–E7 **未验证**其在本板的行为 | 必须显式 `use_mipi_cam:=False`，并用 §7.2 的命令确认没有 `mipi_cam` 进程 |

### 2.3 为什么 v0.1.0 仍不应尝试（修订后的三条理由）

1. **主链路已达标，动它不划算**：默认链路（`LEFT_RIGHT` + codec + websocket）**已被 E4 端到端验证在全速率工作**。接通 stereonet 需要**第二种拼接布局**（与主链路互斥）、**第二路话题**、**非 0 的 `CameraInfo.P`**（与冻结书冲突）以及 **baseline 符号规则**。为一个被冻结书明文列为非目标的特性，去动一个已经跑通的链路，代价与收益不匹配。
2. **三个前置条件仍未被任何实测覆盖**（C3 `P`、C4 baseline 语义、C7 `mipi_cam` 抢占）。E1–E7 关闭了布局与 QoS，但**没有关闭这三个**，因此 stereonet 仍然**不能**被声称"基本可用"。
3. **验收书与冻结书仍互相矛盾**（`20_test_plan.md` O4 说不集成 vs `21_acceptance_criteria.md` 的 `AC-FUN-08`/`AC-DOC-06` 要求 TF/标定）。在 X4/X5/X6 裁定之前，stereonet 实现无法确定自己的"正确"标准。

> **修正记录**：第 1 版 §2.3 的"理由 4（需要 90° 旋转）"**被 E6 削弱**，本文已把它从阻断性理由降级为"布局互斥"这一实现约束（见 C1）。这是 E6 带来的**风险下调**。

### 2.4 如果以后要做：推荐路径（follow-up 的施工顺序，E6 修订版）

**推荐路径 A′（首选，E6 修订）：用 SDK 的 `TOP_BOTTOM` 拼接产出第二路话题**
- 新增参数（例如 `stereo_topic_layout:=top_bottom` + 第二路话题如 `/gs130/stereo_tb_raw`），默认关闭。
- **由 SDK 硬件拼接**（E6 已证明可用），驱动侧**不做手工拼接**（避免第 1 版设想的额外拷贝）。
- 布局必须在**启动时**决定（`stereo_layout` 在 `gs130_init()` 时定死），**不支持运行期切换**。
- 同一参数下发布 `/gs130/stereo_tb_raw/left/camera_info`、`/right/camera_info`，`P` 非 0。
- 优点：不改默认链路、无手工拼接拷贝、可独立开关、失败可回退。
- 缺点：**同一时刻只能有一种布局**（web 与 stereonet 不能同时用 SDK 拼接）；第二路话题带来额外的 publish 开销。

**推荐路径 B：导出 Kalibr YAML，用 `calib_method:=custom`**
- 依据 F16，用 SDK 的 `camera_intrinsics()`（`equidistant`，4 参数 = `k1..k4`）+ `relative_R/T` 生成 YAML。
- 这样**绕开 C3（`P` 全 0）**，是"最小改动"的接法；**但仍受 C1（布局）与 C4（baseline 语义）约束**。

**路径 A′+B 组合是完整方案**，且必须在板端实测通过后才写进文档。**C7 必须在第一次实跑前就用 `use_mipi_cam:=False` 处理掉**，否则违反相机独占。

### 2.5 `hobot_stereonet` 与既有 TROS web 链路的关系（E7 修正）

官方 X5 链路是：`mipi_cam` → `/image_combine_raw` → `hobot_stereonet` → `/StereoNetNode/stereonet_visual`（+ `pointcloud2`）；显示侧新版用 `pointcloud_web_viewer`（端口 **8080**，`stereonet_model_web_visual.launch.py:38-40`），旧版仍可走 `hobot_codec` → `websocket`。

> **E7 修正**：`websocket` 用**同一端口 8000** 上的 `channel` 参数区分多路（D-Robotics 132GS launch 用 channel 0/1），因此**"再开一个画面"不需要新端口**。第 1 版把"网页端口从 8000 变成 8080"写成接入 stereonet 的必要代价，这是**不准确的**：8080 只是另一套 viewer（`pointcloud_web_viewer`，用于显示点云）的端口，而我们的 jpeg 链路始终在 8000 上按 channel 区分。若将来接入 stereonet 并显示点云，文档需要说明的是"点云走另一套 viewer/端口"，而不是"原来的网页换端口了"。

---

## 3. 结论速览（给评审用）

| 集成项 | 结论 | 一句话理由 |
|---|---|---|
| `hobot_codec` | **v0.1.0 必须做；QoS 必须是 `RELIABLE`，`height` 必须是真实高** | `[实测-板端]` E1（BEST_EFFORT ⇒ 静默收不到）、E2（`H*3//2` ⇒ SEGFAULT）；**X2/X3 已关闭** |
| `websocket` | **v0.1.0 必须做**，零直接耦合；`channel` 复用 8000 端口 | `[实测-板端]` E4（29.97 Hz + HTTP 200）；E7（无需第二端口） |
| `hobot_shm` | **v0.1.0 保留为环境开关，不承诺收益，且不能修 QoS** | `[实测-板端]` E3：缓冲是普通 CPU 缓冲；`shm_fastdds.xml` 不设 reliability |
| `hobot_stereonet` | **v0.1.0 不做**；follow-up 风险已下调（布局可行、QoS 相符），但 `P`/baseline/`mipi_cam` 三项仍未验证 | E6 降低了布局门槛；C3/C4/C7 仍开放 |
| `hobot_yolo_world` | **v0.1.0 不做**；文档给出组合示例 | 需要 NV12→bgr8 转换，属既有节点职责 |
| `/tf_static` | **v0.1.0 建议做**（与冻结书"不做"相反）；E5/E7 已消除两条反对理由 | 外参已在 SDK 中，`tf2_msgs` 已在板端；不做会让下游"看起来能用但没有几何关系" |
| 零拷贝 | **不做**，维持堆缓冲 + 正确的一次拷贝 | `[实测-板端]` E3：正确路径 1.36 ms / 1.38 MB；真零拷贝需 SDK 导出 hb_mem fd |

---

## 4. `hobot_shm` 与零拷贝路径（对应要求 3）—— **已由 E3 关闭**

> 第 1 版把本节写成"开放问题 + 建议"。**E3 已给出实测答案**，本节改为"结论 + 数据 + 用户可见差异"。

### 4.1 事实链：我们的帧**不可能**零拷贝（源码级论证）

1. SDK 的 `Pipeline::get_frame()` **总是**把 SDK 内部 `hb_mem` 缓冲**逐行 `memcpy`** 到调用方提供的指针（`rdkx5.cpp:377-383`）。也就是说，数据到我们的地址空间时**已经至少被拷贝过一次**——这一步与 ROS 无关，是 SDK 的既有行为。
2. 目标缓冲是 `malloc()` 的普通堆内存（`gs130.cpp:252-253`），所有权交给调用方（`gs130.h:231`），Python 侧在数组析构时 `free()`（`_types.py:25`）。
3. `hobot_shm` 只改 **Fast-DDS 传输层**（`shm_fastdds.xml` + `RMW_FASTRTPS_USE_QOS_FROM_XML`），它无法把 `malloc` 的堆内存变成可跨进程共享的 DMA-BUF。**且 E3 实测发现 `shm_fastdds.xml` 并不设置 reliability** ⇒ 它既解决不了零拷贝，也解决不了 E1 的 QoS。
4. D-Robotics 真正的零拷贝路径是 `hbm_img_msgs::msg::HbmMsg1080P`（`in_mode=shared_mem`）：发布端把 **hb_mem 缓冲的 fd** 传出去，接收端 `mmap`。要从我们的进程走这条路，必须**由 SDK 导出 hb_mem 的 fd/handle 并把生命周期交给 ROS**，也就是改动 `core/`（冻结书 C7 明确禁止）。
5. 退一步：即使我们自己 `hb_mem_alloc` 一块共享缓冲，也**必须先把 SDK 的 malloc 帧 `memcpy` 进去**（F8）——拷贝次数与"直接拷进消息"完全相同，只是目的地不同。**净收益 ≈ 0，复杂度显著上升。**

> **结论（源码 + 实测双重支持）**：本包在任何配置下都不是零拷贝。任何声称"启用 `hobot_shm` 即零拷贝"的文档都是错的。
> **v0.1.0 决定：不使用 `hobot_shm` 零拷贝。** `hobot_shm` 仅作为既有 C++ 节点之间的传输环境开关保留。

### 4.2 推荐方案

| 项 | 推荐 | 理由 |
|---|---|---|
| 发布路径 | **普通 `sensor_msgs/Image`（堆内存）+ 发布前 1 次显式拷贝** | 与冻结书 §6.5、`22_risk_and_safety.md` §4.1 一致；rclpy 下唯一能保证所有权解耦的做法；E3 实测开销可接受 |
| `hobot_shm` | **保留为 launch 开关**，但**不承诺性能收益**，且**不得**用它解释/修复 QoS 问题 | E3：它只影响 C++ 节点间传输；E1：QoS 必须在 publisher 上显式声明 |
| 拷贝写法 | **`array.array('B')` + `frombytes(memoryview(frame).cast('B'))`**；**禁止** `msg.data = frame.tobytes()` | E3：正确写法 **1.36 ms / 1.38 MB**；朴素写法 **1103.87 ms**（rclpy 逐元素校验） |
| 真正的零拷贝 | **follow-up，且必须先做 SDK 侧可行性评审**（导出 hb_mem handle） | 属核心改动，不应在 v0.1.0 出现 |

### 4.3 用户可测量的差异（**E3 实测数字，直接写入发布说明**）

**单帧成本（E3，节点内部分段计时）**：

| 阶段 | 实测耗时 | 说明 |
|---|---|---|
| `build`（构造消息头/字段） | **0.53 ms** | 可忽略 |
| `fill`（填充 `data`） | **1.28 ms** | 与"`array.array` + `frombytes` = 1.36 ms / 1.38 MB"一致 |
| `publish` | **5.07 ms** | 含 DDS 序列化与传输 |
| **合计** | **≈ 6.9 ms / 帧** | `30 fps` 的帧周期是 **33.3 ms** ⇒ 约占 **21%**；`AC-RES-02` 的 50% 单核上限有充足余量 |

**必须避免的陷阱（E3 实测，量化）**：

| 写法 | 实测耗时 | 结论 |
|---|---|---|
| `array.array("B") + frombytes(...)`（1.38 MB） | **1.36 ms** | ✅ 采用 |
| `msg.data = frame.tobytes()` | **1103.87 ms** | ❌ **绝对禁止**：rclpy 对 `bytes`/序列做逐元素校验，慢约 **800×**；这一条会直接让节点"看起来卡死" |

> **给实现者的硬规则（建议逐字写进代码注释与 code review 清单）**：
> `sensor_msgs/Image.data` 赋值必须走 `array.array('B')` + `frombytes`，**不得**用 `tobytes()`、不得用 Python list、不得把 numpy 数组直接赋给 `.data`。
> 这条规则的理由是实测的 **1103.87 ms vs 1.36 ms**，不是风格偏好。

**持续带宽（按 E2 的正确 `height` 语义重算）**：

| 场景 | 单帧字节（拼接帧） | 30 fps 带宽 | 备注 |
|---|---|---|---|
| `640×480`（`width=1280, height=480`） | 921,600 B | **≈ 27.6 MB/s** | 目标默认配置 |
| `544×640`（`width=1088, height=640`） | 1,044,480 B | ≈ 31.3 MB/s | 纵向原生比例 |
| `1088×1280` RAW 拼接（`width=2176, height=1280`） | 4,177,920 B | **≈ 125.3 MB/s** | RAW 模式；`AC-RES-02` 的主要风险点 |

**判定命令（若评审仍想看 `hobot_shm` 的实际影响）**：

```bash
# A: 不带 hobot_shm
ros2 launch gs130_ros gs130_camera.launch.py mode:=resize width:=640 height:=480 fps:=30 &
PID=$(pgrep -f gs130_node | head -1)
top -b -d 1 -n 60 -p "$PID" | awk '$12 ~ /gs130_node/ {c+=$9; n++} END {printf "avg CPU%%: %.1f\n", c/n}'

# B: 带 hobot_shm（仅环境差异）；同样采样，对比 avg CPU% 与 `ros2 topic hz`
ros2 launch gs130_ros gs130_web.launch.py hobot_shm:=true ... &
```

> 预期：**A 与 B 相近**（差异在噪声量级）。因为真正的拷贝发生在"SDK → 消息"这一步，与 DDS 用什么传输无关。**若 B 明显更差，属正常**，不应被当作回归。

**文档必须写的一段话（建议逐字）**：
> `hobot_shm` 只优化 D-Robotics 自家 C++ 节点之间的图像传输；`gs130_node` 的像素数据是普通内存缓冲，**不参与零拷贝**，开启它**不会**降低本节点 CPU。它**也不能**修复 QoS 问题——图像话题必须以 `RELIABLE` 发布（见"与 hobot_codec 的集成要求"）。真正的零拷贝需要 SDK 导出硬件缓冲句柄，v0.1.0 不支持。

### 4.4 与 `hbm_img_msgs` 尺寸上限的关系（顺带澄清一个常见误解）

`HbmMsg1080P` 的 `data` 上限是 6,220,800 B（F15）。按 E2 确定的正确语义，我们的 RAW 拼接帧是 `2176 × 1280` 的 NV12 = **4,177,920 B**，**小于上限**。所以"RAW 太大发不进 hbmem"这个说法在 `1088×1280` 上不成立；真正的原因仍然是 §4.1 的 fd 问题。

---

## 5. 外参、标定与 TF 期望（对应要求 4）

### 5.1 时间戳时钟域：**E5 已关闭，实现规则照此执行**

**E5 实测事实**：SDK 返回的时间戳是 **`CLOCK_MONOTONIC`（自开机），不是 Unix epoch**：

| 观测量 | 实测值 |
|---|---|
| 一帧的设备时间戳 | `9546841822000` ns |
| 同一时刻的 monotonic 时钟 | `9540690105753` ns（≈ 9480 s 开机时间） |
| 相对墙钟的差 | **≈ -1.79e18 ns** |
| IMU 与图像的关系 | `imu - frame = 8.6 ms`（**同一时钟域**，可直接比较） |

**后果（E5 明确指出）**：把原始 SDK 时间戳直接写进 `header.stamp` 会**静默破坏** `tf`、`message_filters`、`rosbag`——它们都假定时间戳在 ROS 时钟域内。表现为"看起来在跑，但 TF 查询总是超时、bag 回放时间轴错乱、`message_filters` 永远配不上"。

**实现规则（唯一版本，照此执行）**：

```
# 启动时（第一次拿到有效设备时间戳时）一次性确定偏移：
offset_ns = (本机系统时钟 now_ns) - (device_ts_ns)      # stamp_offset_mode=auto
offset_ns = stamp_offset_ns                              # stamp_offset_mode=manual

# 每条消息（图像与 IMU 用同一个 offset）：
stamp_ns = device_ts_ns + offset_ns
header.stamp.sec     = stamp_ns // 1_000_000_000
header.stamp.nanosec = stamp_ns %  1_000_000_000
```

**必须写进用户文档的两条**：
1. **残余不确定度 ≤ 一个帧周期**（`1/fps`，默认 `33.3 ms`）。它与"图像-IMU 相对时间"无关（两者同域，E5），只影响"绝对时刻"的准确性。需要更精确绝对时间（例如与外部传感器比对）的用户必须自行使用外部同步（硬件触发/PTP）。
2. **`use_sim_time` 与 `stamp_offset_mode` 的交互**必须明确：`use_sim_time:=true` 时节点时钟来自 `/clock`，偏移会在启动时按当时的 `/clock` 计算——这是"可预期但容易误解"的行为，文档需要点名。

**`[待板端验证]`（E5 未覆盖的部分）**：
- 偏移量在长时间运行（数小时）下的**漂移**（monotonic 与墙钟的漂移，受 NTP 步进影响）；
- `mode=raw` 与 `mode=rect` 下时间戳来源是否一致（`frame_ts_ns` 的 `trig_tv → timestamps → tv` 优先级，`rdkx5.cpp:385`）。

### 5.2 用户到底需要什么样的变换

一个 ROS 用户拿到双目相机时，会依次需要三样东西：

1. **`camera_info`（已冻结，必要）**：每个目一个，`K/D/distortion_model` 用于去畸变；`P` 用于立体/深度。
2. **左右目之间的静态变换（TF 或 `P`）**：无论是 `stereo_image_proc`、自研深度、还是 `hobot_stereonet`，都需要**一个基线**。ROS 生态里有两个等价载体：
   - `CameraInfo.P[3] = -fx * baseline`（stereonet 走这条，F16/§2.1(d)）；
   - `tf_static`: `left_frame → right_frame` 的平移（`stereo_image_proc` 等走这条）。
3. **相机与 IMU 之间的静态变换**：任何 IMU-视觉融合（VIO、`robot_localization`、`imu_filter_madgwick` + 相机外参）都需要 `imu_frame → camera_frame`。

**E5 给这一节带来的前置结论（必须先看 §5.1）**：静态 TF 的**内容**（旋转/平移）与**时间戳**是两件事。E5 已经解决了时间戳那一半：SDK 给的是 `CLOCK_MONOTONIC`，必须换算到系统时钟域，且换算后的残余不确定度**至多一个帧周期**。这对"静态 TF"尤其友好——静态变换本身不随时间变化，时间戳只影响"从哪一个时刻开始有效"，因此**残余不确定度对一个 latched 静态变换几乎无害**（唯一影响是"变换的生效时刻可能偏早/偏晚一帧"，对静态关系无几何影响）。这消除了第 1 版把"时间戳时钟域不确定"当作 TF 反对理由的合理性。

### 5.3 我们可以给出什么（SDK 能力核对）

| 需要的东西 | SDK 提供 | 换算 |
|---|---|---|
| 左目内参 `K,D` | ✅ `camera_intrinsics(LEFT)`，`dist_model=FISHEYE`，`D = [k1..k4]` | 直接映射 `equidistant`（冻结书 §3.4.1） |
| 右目内参 | ✅ 同上 | 同上 |
| 基线 | ✅ 可算：`baseline = ‖T_left - T_right‖`，`T` 单位米（F9、`gs130.h:408`） | **需要契约定义符号与坐标轴** |
| 左右相对旋转 `R_lr` | ✅ `relative_R(CAMERA_LEFT, CAMERA_RIGHT)`（`gs130.h:392-396`） | 直接转四元数 |
| IMU↔相机 | ✅ `relative_R/T(CAMERA_LEFT, IMU)` | 同上 |
| IMU 内参（噪声、随机游走、bias） | ✅ `imu_intrinsics()` | 可写入 `sensor_msgs/Imu` 的协方差（当前冻结书选择 `-1`，见 §5.5） |

**关键前提（必须在契约里写明）**：`mode=rect` 时 SDK 的 `stereo_rectify()` 会把外参**改写为虚拟双目平行系**（`rectify.hpp:20-27`、`gs130.h:341`）。也就是说：
- `mode=raw/resize`：`relative_R/T` 是**物理**外参，TF 表达物理安装关系；
- `mode=rect`：`relative_R/T` 是**虚拟**外参，TF 表达的是"矫正后的虚拟相机"。

**因此**：`/tf_static` 的语义**随 `mode` 变化**。这不是缺陷，但**必须写进文档**，否则用户会在 `mode:=rect` 下用 TF 做 VIO 而得到错误结果。

### 5.4 推荐的 TF 约定（若采纳）

| 项 | 建议 |
|---|---|
| frame 名 | 沿用冻结书的 `camera_left` / `camera_right` / `imu_link`（`11:130-137`），**不要**用 `_optical_frame` 后缀（除非真的满足 REP-105 光轴约定） |
| 树形 | 以 **`camera_left` 为根**（左目为主参考），发布 `camera_left → camera_right`、`camera_left → imu_link` |
| 方向 | `T_parent_child`：把子系中的点变到父系。SDK 的 `relative_T(from, to)` 给的是"源原点在目标系中的坐标" = `t_to_from`，因此 `T_camera_left_camera_right = relative_T(CAMERA_RIGHT, CAMERA_LEFT)`；`T_camera_left_imu = relative_T(IMU, CAMERA_LEFT)`。**符号必须用一条单元测试锁死**：`‖T_camera_left_camera_right‖` 与 `‖relative_T(CAMERA_LEFT,CAMERA_RIGHT)‖` 必须相等（模长与方向无关，可作为无硬件单测） |
| 旋转 | 旋转矩阵 → 四元数，要求 `det(R) > 0`、`‖R Rᵀ - I‖ < 1e-6`；不满足则**不发布**并报 `WARN`（宁可缺 TF，不可发错 TF） |
| 轴约定 | **`[待板端验证]`**：SDK 的相机坐标轴（X 右/Y 下/Z 前？）在源码与文档中**没有明文**，只有"参考系由 EEPROM 驱动定义"。**在验证之前不要声称满足 REP-103/REP-105** |
| QoS | latching：`RELIABLE` + `TRANSIENT_LOCAL` + depth ≥ 1（标准 `/tf_static`） |
| 频率 | 启动时一次（latched），运行期不变 |
| **时间戳（E5）** | `header.stamp` 用 §5.1 的换算结果（系统时钟域）。**静态变换对时间戳不敏感**（它不随时间变化），但**必须**落在 ROS 时钟域内，否则 `tf2` 的静态缓存会把"未来/过去 1.79e18 ns"的条目判为无效 |

### 5.5 如果我们什么都不发布，消费者会怎样（必须诚实写出）

| 消费者 | 缺 `P`/TF 时的实际表现 |
|---|---|
| `hobot_stereonet` | `CameraIntrinsic::is_valid()` 为 false → 回调**直接 return**，节点**不报错**、无输出。用户看到"节点在跑、话题是空的" |
| `stereo_image_proc` / `image_proc` | `image_proc` 只需要 `K/D` + `CameraInfo`，可以工作；`disparity_node` 需要 `P`，会给出无意义结果或直接失败 |
| `rviz2` 点云 / `robot_state_publisher` 下游 | 缺 frame → `Fixed Frame` 报错，点云无法显示 |
| VIO / `robot_localization` | 无法建立相机-IMU 观测模型，等于少一个传感器 |
| 只用图像做检测/跟踪的用户 | **完全不受影响** |

### 5.6 建议：**v0.1.0 就做 `[建议偏离]`**（与冻结书 §0.2 相反；**E5/E7 后本建议已加强**）

冻结书 §0.2 把 `/tf` 列为非目标，理由是"外参语义需要单独评审，不引入半正确的 TF"。该理由**成立且谨慎**，但本文建议**用一个受控的方式做**，因为：

1. **代价极低，且 E7 已证明无新增依赖成本**：`relative_R/T` + `camera_intrinsics()` 已在 SDK 公开 API 内，换算是一段纯函数（可无硬件单测）。第 1 版曾把"需要新增 `tf2_msgs`"算作成本；**E7 实测板端已装 `tf2_msgs`**，因此这条成本**归零**。
2. **收益明确**：不做 TF，双目包对 ROS 生态的可用性只剩"两路 NV12 图像"，任何几何下游都无法使用。
3. **E5 消除了"时间戳不可靠"这条反对理由**：静态变换的时间戳只要落在 ROS 时钟域即可，而 E5 已给出确定性的换算规则与"≤ 一个帧周期"的残余不确定度；对静态关系**几何上无影响**（见 §5.1、§5.4）。
4. **风险可控**：把"不确定"的部分**显式挡住**——轴约定未验证前**不使用** `_optical_frame` 命名、`R` 非法时不发布、`mode=rect` 时在文档与日志中标注"虚拟外参"。

**若评审不同意**（认为语义风险 > 收益），则必须同时满足：
- 文档里**明确写出**"本包不提供任何几何关系，`P`/`R` 为 0 是**有意为之**，不是 bug"；
- 在 `README` 的"已知限制"中给出**用户自行发布的示例代码**（从 SDK 取外参 → `static_transform_publisher`），否则用户会认为相机坏了。

### 5.7 与 `CameraInfo.P` 的关系（一个必须做的取舍；**E1–E7 未覆盖，仍待裁定**）

- `P` 全 0 是当前冻结决定（`11:254`）。
- `hobot_stereonet` 的 `none` 路径**必须** `P` 非 0（§2.1(d)）。
- 因此：**"`P` 全 0" 与 "stereonet 可用" 不可同时成立**。这是 §2.2 的 C3，**E1–E7 没有覆盖**，必须在做 stereonet 之前裁定。
- 折中方案（若要做 stereonet）：**只在新话题上发布带 `P` 的 `CameraInfo`**，既有 `/image_left/camera_info` 保持全 0。这样默认契约不变，stereonet 走新链路。

---

## 6. 标准 ROS 开发者的操作路径（对应要求 5）

> 前提：下面所有命令都建立在"节点已在跑"之上；QoS 相关结论依赖 §6.0 的裁定。

### 6.0 通用前提（E1/E2 之后的**修正版**）

**第 1 版这里写的是"我们的图像/IMU 是 `BEST_EFFORT`"，该前提已被 E1 推翻。** E1 实测要求图像话题必须是 `RELIABLE`。因此：

| 话题 | QoS（E1 之后） | 对 CLI 工具的含义 |
|---|---|---|
| `/image_combine_raw`、`/image_left_raw`、`/image_right_raw` | **`RELIABLE`** + `VOLATILE` + `KEEP_LAST`（depth ≥ 5，与 codec 对齐） | CLI 默认与显式 `--qos-reliability reliable` **都能收到**；显式写 `best_effort` **也能收到**（RELIABLE pub ↔ BEST_EFFORT sub 是相容方向） |
| `/imu/data` | 可保持 `BEST_EFFORT`（保留冻结书选择）或与图像统一为 `RELIABLE` | 两种都能被 CLI 的 auto 模式收到 |
| `/image_left/camera_info` 等 | `RELIABLE` + `TRANSIENT_LOCAL` | 迟到订阅者仍能收到（latch）；拿**历史**那次需 `--qos-durability transient_local` |

**兼容方向速查（E1 的直接推论，务必写进文档）**：

| publisher | subscriber | 是否相容 |
|---|---|---|
| `RELIABLE` | `RELIABLE` | ✅ |
| `RELIABLE` | `BEST_EFFORT` | ✅ |
| `BEST_EFFORT` | `RELIABLE` | ❌ **永远不匹配**（E1 实测：codec 的报错就是这一格） |
| `BEST_EFFORT` | `BEST_EFFORT` | ✅ |

> **结论：图像发布必须是 `RELIABLE`。** 这是唯一能让"既有 `hobot_codec`"与"任意默认 QoS 的第三方订阅者"同时工作的选择。

**文档必须给出的"三件套"自检**（缺失任何一条，用户都会认为是相机问题）：

```bash
ros2 topic list | grep -E 'image_combine_raw|imu/data|camera_info'
ros2 topic info -v /image_combine_raw      # 看 Publisher/Subscription count 与两侧 QoS（必须 RELIABLE）
ros2 topic hz /image_combine_raw           # 看是否真的在发
```

### 6.1 `ros2 topic echo`

| 项 | 要求 / 陷阱 |
|---|---|
| 能工作 | ✅ 消息是标准 `sensor_msgs`，无自定义类型 |
| **NV12 专属陷阱 1** | **不要** echo 整个图像：NV12 是 `uint8[]`，默认会把几十万到几百万个数字逐行打印，终端会刷屏并显著占用 CPU。**必须**用 `--field`：<br>`ros2 topic echo /image_combine_raw --once --field encoding`（期望 `nv12`）<br>`--field width` / `--field height` / `--field step` / `--field header.stamp` |
| **NV12 专属陷阱 2（E2 之后的新判据）** | `--field height` 的期望值是**真实高**：`mode=resize width:=640 height:=480` ⇒ **`height: 480`**（**不是 720**）、`width: 1280`（拼接）、`step: 1280`。<br>若看到 `height = 720`，说明实现按"打包高度"填了字段——**这会让 codec SEGFAULT（E2）**，属 P0 缺陷，应立即报 bug 而不是"大概没事" |
| **NV12 专属陷阱 3** | `--field data` 会输出 `array('B', [...])` 巨型数组；要检查数据请用 Python 订阅者而不是 CLI |
| 陷阱 4 | `--once` 对 latched `CameraInfo` 有效；对图像话题在无发布时**会一直等**，建议 `timeout 10 ros2 topic echo ...` |
| 陷阱 5 | 显式 `--qos-reliability best_effort` 读**图像**仍可收到（相容方向），但**不要**据此认为"BEST_EFFORT 就够"——那只对订阅者成立，对 codec 不成立（E1） |

### 6.2 `ros2 topic hz` / `ros2 topic bw`

| 项 | 要求 / 陷阱 |
|---|---|
| 能工作 | ✅ |
| 陷阱 1 | 默认跟随发布者 QoS；显式指定 `reliable` 会收不到 |
| 陷阱 2 | **它用 `header.stamp` 的差值估算频率**。我们的 `header.stamp` 由**设备单调时钟 + 启动偏移**换算而来（§5.1，E5）。**判据**：`hz` 平均值应 ≈ `fps`（容差 ±5%）。**关键排查提示**：若实现忘了做 §5.1 的偏移换算，`hz` 仍会显示正确频率（它只看差值），但 `bag`/`tf2_echo`/`message_filters` 会全线失效——**"hz 正常"不能证明时间戳正确**，必须用 `--field header.stamp` 与 `date +%s%N` 比对量级（E5：错误实现会差约 1.79e18 ns） |
| 陷阱 3 | `/imu/data` 是**突发式**发布（FSYNC 后成组，`11:236`）。`hz` 的窗口平均 ≈ `odr`，但**瞬时**看起来是一阵一阵的。`/imu/data` 上看到"忽高忽低"**不是** bug，文档必须提前说明，否则会被报为缺陷 |
| 带宽 | `ros2 topic bw /image_combine_raw`：按 E2 的正确语义，`640×480` 拼接帧是 `1280×480` NV12 = **921,600 B/帧 ≈ 27.6 MB/s**；`1088×1280` RAW 拼接帧是 `2176×1280` NV12 = **4,177,920 B/帧 ≈ 125.3 MB/s**。**文档给出这个量级**，让用户预先判断网络/磁盘是否够用 |

### 6.3 `ros2 bag record`

| 项 | 要求 / 陷阱 |
|---|---|
| 能工作 | ✅ 消息是标准类型；**E1 之后图像为 `RELIABLE`，与 `ros2 bag record` 的默认 `RELIABLE` 订阅天然相容**（第 1 版担心的"录不到任何图像帧"在修正 QoS 后**不再存在**） |
| **陷阱 1（改为 durability）** | 带 `TRANSIENT_LOCAL` 的 `camera_info` 是 **latched**：若在节点启动**之后**才开始录制，`bag` 里**不会**有那次已发布的历史消息（默认订阅是 `VOLATILE`）。要录到它，必须 `--qos-profile-overrides-path` 指定 `durability: transient_local`，或**先起录制再起节点** |
| **陷阱 2（NV12 专属，体积）** | bag 存的是 NV12 原始字节：`640×480` 拼接 ≈ **0.92 MB/帧 ≈ 27.6 MB/s**；`1088×1280` RAW 拼接 ≈ **4.18 MB/帧 ≈ 125.3 MB/s**。**1 分钟 RAW 约 7.5 GB**。文档必须写明磁盘预算，并建议录 `640×480`。 |
| 陷阱 3 | 回放后 `header.stamp` 是**录制时的时间**（除非 `--use-sim-time`），与当前系统时钟脱节；`ros2 topic hz` 回放时看起来正常，但与其他在线话题混合分析会错位 |
| **陷阱 4（E5 相关）** | bag 里的时间戳只有在**写入时已换算到 ROS 时钟域**才有意义。若实现漏了 §5.1 的换算，录出来的 bag 时间轴会落在"开机后 9500 秒"，`ros2 bag info` 与回放都会表现为异常起点 |
| 用途 | bag 是**最有价值的调试手段**（`hobot_codec` 不在场时也能复现网页黑屏）。文档应给出"录 10 秒 640×480 + IMU"的一行命令与配套 `qos.yaml` |

### 6.4 RViz2

| 项 | 要求 / 陷阱 |
|---|---|
| **NV12 图像** | ❌ **无法直接显示**。`rviz2` 的 Image / Camera 显示插件要求 `rgb8`/`bgr8`/`mono8`/`mono16`/`depth`，不认识 `encoding: "nv12"`。**这是本包最大的"看起来能用但看不到图"陷阱。** |
| 推荐做法 | 显示 `/image_combine_jpeg`（`CompressedImage`）：用 `rqt_image_view` 或 Foxglove 更简单；RViz 需 `image_transport` 的 compressed 插件 |
| 要显示原始话题 | 必须经 `image_transport republish`：<br>`ros2 run image_transport republish raw in:=/image_left_raw compressed out:=/image_left_compressed`（或借 `hobot_codec` 转 bgr8） |
| CameraInfo | ✅ 可作为文本显示 |
| 点云/深度 | 本包**不提供**（非目标），因此 RViz 无法显示深度 |
| TF | 若无 `/tf_static`，`Fixed Frame` 只能选固定的 `map`/`camera`，图像仍可显示（无几何含义）；**但任何 `PointCloud2`/`LaserScan` 都无法显示** |

### 6.5 Foxglove Studio

| 项 | 要求 / 陷阱 |
|---|---|
| 连接 | 通过 `foxglove_bridge` 或 rosbridge（TROS 不一定自带，需用户自行安装 → **文档应注明"生态自带/web 链路不依赖 Foxglove"**） |
| 图像 | `CompressedImage`（jpeg）✅；**NV12 不支持**（同 §6.4） |
| 时间戳 | 依赖 `header.stamp` 单调；我们的设备时钟偏移在启动后恒定，满足 |
| 优势 | 可同时看 `/image_combine_jpeg` 与 `/imu/data` 曲线，是**排查"图示正常但数据不对"最好的工具** |

### 6.6 `rqt_image_view`

| 项 | 要求 / 陷阱 |
|---|---|
| NV12 | ❌ 同一原因无法显示（`image_transport` 无 NV12 插件） |
| jpeg | ✅ 直接选 `/image_combine_jpeg` 即可，**这是推荐给用户查看画面的方式** |

### 6.7 一条可直接复制的组合示例（写进用户文档）

```bash
# 终端 A：相机 + 普通 ROS 图像
ros2 launch gs130_ros gs130_camera.launch.py mode:=resize width:=640 height:=480 fps:=30 publish_per_eye:=true

# 终端 B：把左目 NV12 转 bgr8（复用既有 hobot_codec，不自研）
ros2 run hobot_codec hobot_codec_republish --ros-args \
  -p channel:=0 -p in_mode:=ros -p in_format:=nv12 -p out_mode:=ros -p out_format:=bgr8 \
  -p sub_topic:=/image_left_raw -p pub_topic:=/image_left_bgr

# 终端 C：按 D-Robotics 既有用法跑一个感知节点（示例，需实测）
ros2 run hobot_yolo_world hobot_yolo_world --ros-args \
  -p feed_type:=1 -p ros_img_sub_topic_name:=/image_left_bgr -p texts:="bottle,person"

# 终端 D：网页查看（官方 websocket，channel 0 = 左目）
ros2 launch websocket websocket.launch.py websocket_image_topic:=/image_combine_jpeg websocket_only_show_image:=true
```

> 上述 B/C/D 三条**必须逐条实测**后才能写进正式文档（`[待板端验证]`）。本评估只能证明它们**在接口上互相匹配**（话题名、类型、编码字符串），不能证明它们跑得通。

---

## 7. 与 TROS 生态的共存规则（对应要求 6）

### 7.1 绝不能与 `gs130_node` 同时运行的进程（完整清单）

| 类别 | 具体对象 | 为什么冲突 | 用户如何发现 |
|---|---|---|---|
| **D-Robotics 相机节点** | `mipi_cam`、`hobot_stereo_mipi_cam`、`hobot_rgbd_cam`、`hobot_zed_cam`、`hobot_usb_cam`（USB 相机通常不冲突，但 `mipi_cam` 一定冲突） | 抢占 MIPI CSI / VIN / I2C 与 ISP/VPU 全局资源 | `ros2 node list`、`pgrep -af 'mipi_cam\|usb_cam'` |
| **`hobot_stereonet` 的官方 launch** | `stereonet_model_web_visual.launch.py` 默认 `use_mipi_cam:=True` → 会拉起 `mipi_cam` | 间接抢相机 | 启动命令里出现 `mipi_cam_dual_channel.launch.py` 即冲突；必须显式 `use_mipi_cam:=False` |
| **SDK 自带示例/工具** | `core/samples/gs130-run`、`gs130-rec`、`gs130-calib-export`、`core/src/tools/gs130-detect-*` | 直接打开同一个 I2C/CSI 设备 | `pgrep -af gs130` 除了 `gs130_node` 还有别的命中 |
| **手工 I2C 访问** | `i2cdetect -y 4` / `-y 6`（对 `0x30/0x31/0x32` 的**读**通常无害，但 `i2cset`/寄存器写**有害**） | 可能改变 sensor 状态或干扰 SDK 的探测 | 用户手册中明确禁止写操作 |
| **另一个 `gs130_node`** | 同一节点启动两份 | 独占设备 | `pgrep -c -f gs130_node` > 1 |

### 7.2 用户如何检测冲突（必须写进文档的命令）

```bash
# 1) 有没有别的相机进程
pgrep -af 'mipi_cam|gs130|usb_cam|stereonet' | grep -v grep

# 2) ROS 层有没有别的相机节点
ros2 node list

# 3) 我方节点的启动日志（唯一权威判据）
#    相机被占用时：ERROR + GS130_NOT_FOUND/GS130_HW_ERROR + 退出码 1（冻结书 §8.2）
echo $?     # 或看 launch 输出
```

**判定语义（必须逐字写进文档与错误消息）**：
- `GS130_NOT_FOUND` ⇒ **最可能**是相机被别的进程持有（也可能是没接好/EEPROM 未探测到）；
- `GS130_HW_ERROR` ⇒ 底层通信失败（同样可能是占用，也可能是硬件问题）；
- **不要**把二者混淆为"驱动 bug"。

### 7.3 文档中的警告文本（建议在冻结书 §8.3 基础上补充四条）

1. **相机独占**（守冻结书 §8.3 原文，不删改）。
2. **`hobot_stereonet` 的默认 launch 会拉起 `mipi_cam`**，因此在 GS130 上使用 stereonet 必须显式关闭 `use_mipi_cam`；在 v0.1.0 中我们**不支持**该组合（§2）。**C7 至今未被实测覆盖**，第一次实跑前必须先确认没有 `mipi_cam` 进程。
3. **不要用 `respawn=True` 无人值守运行**（守冻结书 §8.3 原文）；补充原因：设备被占用时会导致高频重启 + 反复 `GS130_HW_ERROR` 日志，且每次失败都会尝试复位 GPIO（`bus_reset_gpio` F10 中的 351/353）。
4. **nginx 是孤儿进程（E4 实测）**：`websocket.launch.py` 用 `os.system` 启动 nginx，**launch 退出后 nginx 继续存活并占用 `:8000`**。因此"关掉 launch"不等于"关掉网页服务"。文档必须给出显式清理命令（见 §7.5）。

### 7.4 生态层的非冲突项（明确写清，避免用户过度担惊）

以下**可以**同时运行，且是推荐组合：`hobot_codec`、`websocket`（nginx `:8000`）、`hobot_shm` 环境、`hobot_yolo_world`（消费我方输出）、`foxglove_bridge`、任意只读订阅者（`ros2 topic echo`/`bag`）。**前提**：它们不试图自己打开相机。

---

### 7.5 启动/停止与清理（E4 修订，必须写进用户文档与测试脚本）

```bash
# 正常停止
#   Ctrl-C 停 launch；然后**必须**确认孤儿进程已清理：
pgrep -af nginx
pgrep -af 'hobot_codec|websocket|gs130_node'

# 清理（按顺序）
pkill -f gs130_node
pkill -f hobot_codec_republish
pkill -f 'websocket'
sudo pkill nginx        # nginx 由 websocket.launch.py 以 os.system 启动，常为孤儿
ss -ltnp | grep :8000   # 确认端口已释放（否则下次启动会失败）
```

**为什么这条是 must-have 而不是 nice-to-have**：
- 第二次运行若 `:8000` 仍被孤儿 nginx 占用，`websocket.launch.py` 会启动失败 → 用户会报"改成 RELIABLE 之后又坏了"这类**误归因**的 bug；
- `AC-RB-04` 一类"重复启动"用例若不做清理，会把**环境残留**判成**实现缺陷**。

## 8. 集成需求优先级清单（对应要求 7）

### 8.1 Must-have（v0.1.0 必须满足，缺一条即认为集成目标未达成）

| # | 需求 | 一句话理由（含证据等级） |
|---|---|---|
| **M1** | **图像 `height` 必须是真实高 `H`**：`width`/`height`/`step`/`len(data)` 四者满足 `step == width`、`len(data) == width*height*3//2` | **`[实测-板端]` E2：按 `H*3//2` 发布会让 `hobot_codec` SEGFAULT**（`init_pic_h_: 1080, alined_pic_h_: 1088`）。这是**崩溃级**契约，不是风格 |
| **M2** | **图像话题必须以 `RELIABLE` 发布**（其余三项 `VOLATILE` / `KEEP_LAST` / depth ≥ 5） | **`[实测-板端]` E1：`BEST_EFFORT` ⇒ codec 报 `RELIABILITY_QOS_POLICY` 不兼容并静默收不到任何帧** |
| **M3** | `/image_combine_raw` 上 `encoding` 逐字 `"nv12"`、紧排（无 stride padding）、`uv_offset == width*height` | `hobot_codec` 以 `width*height` 定位 UV；E4 已证明在正确字段下整链跑通 |
| **M4** | **`header.stamp` 必须由设备单调时钟 + 启动一次性偏移换算到 ROS 时钟域**，图像与 IMU 用同一偏移 | **`[实测-板端]` E5：SDK 给 `CLOCK_MONOTONIC`（相对墙钟差 ≈ -1.79e18 ns），直接填会静默破坏 tf / message_filters / rosbag**；文档需写明残余不确定度 ≤ 一个帧周期 |
| **M5** | `/image_left/camera_info`、`/image_right/camera_info` 存在且 `K/D/distortion_model` 与 SDK 一致（`equidistant` + 4 元 `D`） | 没有内参，用户无法去畸变，双目包只剩"看画面" |
| **M6** | `/imu/data` 发布 `sensor_msgs/Imu`，单位 rad/s、m/s²，协方差 `[0] = -1` | 单位或协方差填错会让下游融合算法静默产出错误结果 |
| **M7** | **帧缓冲所有权必须与 SDK 解耦**（发布前显式拷贝，禁止把 SDK `malloc` 缓冲的视图交给消息） | SDK 在 Python 数组析构时 `free()`（F8）；任何"零拷贝外借"都会变成 use-after-free |
| **M8** | **`data` 赋值必须走 `array.array('B')` + `frombytes(memoryview(...))`；禁止 `tobytes()`、禁止直接赋 numpy 数组** | **`[实测-板端]` E3：正确写法 1.36 ms / 1.38 MB，朴素 `tobytes()` 写法 1103.87 ms（≈800×）**，会让节点"看起来卡死" |
| **M9** | `mipi_cam` 等冲突进程的启动失败必须 `ERROR` + 非 0 退出 + 明确提示相机被占用，**不得重试/静默等待** | 静默重试会掩盖真实的设备占用，用户无从判断 |
| **M10** | 用户文档必须包含：与 `mipi_cam` 互斥警告、`hobot_shm` 不带来零拷贝且**不能修 QoS**、`/image_combine_jpeg` 是唯一可在 RViz/rqt 中查看的话题、**nginx 孤儿进程清理命令** | 这四条是用户最容易误判为"驱动坏了"的地方；E4 证明孤儿 nginx 会造成误归因 |
| **M11** | 与 `hobot_codec` + `websocket` 的默认链路**必须保持 E4 的基线**（≈30 Hz、HTTP 200），任何回归需定位 | **`[实测-板端]` E4：`/image_combine_raw` 29.966 Hz、`/image_combine_jpeg` 29.974 Hz、`curl :8000` HTTP 200** —— 已有基线，不再是"待验证目标" |
| **M12** | **拼接帧由 SDK 输出（`StereoLayout.LEFT_RIGHT`，`mode != raw`），驱动侧不手工拼接** | **`[实测-板端]` E6：SDK 拼接在 `640×480` 下给出形状 `(720, 1280)`，与 `width=2W` 约定完全吻合**；省掉一次全帧拷贝，也避免重复实现（冻结书 C1/C2） |
| **M13** | **RAW 模式与拼接互斥必须 fail-fast**：`mode:=raw` 时不得尝试拼接（RAW 强制 `1088×1280`） | **`[实测-板端]` E6：RAW 模式无法用于拼接**；静默降级会让用户拿到尺寸错误的帧 |
| **M14** | `mode`/`width`/`height` 组合必须在启动阶段校验，非法组合 fail-fast 并点名参数与建议值 | `[已验证-源码]` F11：非整除组合让 SDK 返回 `GS130_UNSUPPORTED`；必须让用户在启动时看到原因 |

> **M1/M2 的地位**：这两条是**实测确认过的、会分别导致"崩溃"与"静默无数据"的契约**，应放在实现检查清单的最前面，并在 code review 中作为硬性门禁。

> **M14 的补充证据（重要，请评审注意）**：`mid` 在 `mode=resize` 下等于 `sensor 尺寸 1088×1280`（F10，**纵向、高大于宽**），非 RAW 模式要求精确整除（F11）。因此**并非任意分辨率都可用**，且**裁剪方向是纵向**（因为 `mid` 是纵向的，输出要求更宽时按高度裁）。可用性判据（`[已验证-源码]`，由 `vse.c` 的 `roi_ratio_exact()` 逐字推导）：
>
> ```
> 当 1088*out_h > out_w*1280  （输出比 mid 更宽）: 要求 (1280*out_w) % out_h == 0
> 否则（输出比 mid 更窄/等高）:                   要求 (1088*out_h) % out_w == 0
> ```
>
> 用该判据核对的常见分辨率（`mid = 1088×1280`，`roi = aspect_roi(mid, out)`）：

| 目标 `width×height` | 可用 | `roi`（裁剪区域） | 视野损失 |
|---|---|---|---|
| `1088×1280` | ✅ | `0,0,1088,1280` | 无（原生纵向） |
| `544×640` | ✅ | `0,0,1088,1280` | 无（纯缩放 1/2） |
| `272×320` | ✅ | `0,0,1088,1280` | 无（纯缩放 1/4） |
| `640×480` | ✅ | `0,232,1088,816` | 纵向裁掉 ≈ 36%（上下各 232 行） |
| `320×240` | ✅ | `0,232,1088,816` | 同上（同一 ROI 再缩放） |
| `800×600` | ✅ | `0,232,1088,816` | 同上 |
| `1280×960` | ✅ | `0,232,1088,816` | 同上 |
| `1920×1080` | ✅ | `0,334,1088,612` | 纵向裁掉 ≈ 52% |
| `640×360` | ✅ | `0,334,1088,612` | 同上 |
| `1280×720` | ✅ | `0,334,1088,612` | 同上 |
| `640×640` | ✅ | `0,96,1088,1088` | 纵向裁掉 ≈ 15% |
| `1088×480` | ✅ | `0,400,1088,480` | 纵向裁掉 ≈ 62% |
| `848×480` | ❌ | — | 非整除 ⇒ `GS130_UNSUPPORTED` |
| `1024×600` | ❌ | — | 非整除 |
| `864×480` / `1120×480` / `1008×480` | ❌ | — | 非整除（**这几个"看起来很正常"的值实际不可用**） |
| `640×512` | ❌ | — | 非整除 |

> **结论**：冻结书与验收书使用的 `640×480`/`320×240` 是可行值；但**用户极易踩到"看似正常却不可用"的分辨率**（如 `864×480`、`1008×480`、`1024×600`）。因此文档**必须**写出上面的判据与可用值表，并把 `GS130_UNSUPPORTED` 的日志改成**直接打印判据与建议值**（例如"`864×480` 不满足 `(1088*480) % 864 == 0`，请改用 `816×480` 或 `640×480`"）。此外，`[待板端验证]` 该裁剪方向（纵向）与裁剪量：用 `mode=resize width:=640 height:=480` 拍一张已知图案，确认上下被裁而非左右被裁。

### 8.2 Follow-up（v0.1.0 之后，需评审）

| # | 需求 | 一句话理由 |
|---|---|---|
| F1 | 提供一路**上下堆叠**的拼接话题（用 SDK 的 `StereoLayout.TOP_BOTTOM` + `mode=resize`，见 §2.4 路径 A′），用于对接 `hobot_stereonet` | 官方 stereo 链路只认上下堆叠（F3/F17）；**E6 已证明 SDK 能直接拼出该布局**，门槛从"需要旋转"降为"需要第二路话题" |
| F2 | 导出 Kalibr 风格 YAML（`cam0/cam1` + `T_cn_cnm1`） | `hobot_stereonet` 的 `calib_method:=custom` 的唯一输入（F16），可绕开 `P` 全 0 冲突（§2.2 C3） |
| F3 | 在带 `P` 的 `CameraInfo` 中发布 `baseline`（并按契约定义符号与坐标系） | 没有 baseline 就没有深度；符号错误等于深度全错（§2.2 C4 **仍开放**） |
| F4 | `/tf_static` 外参（见 §5.6；本评估建议**提前到 v0.1.0**，若评审不采纳则留在此处） | 几何下游（VIO、点云、TF 依赖工具）需要；E5/E7 已消除两条反对理由 |
| F5 | 把 SDK 的 IMU 噪声/随机游走写入 `sensor_msgs/Imu` 协方差（替代 `-1`） | 下游融合算法需要真实的噪声模型才能给出可信状态估计 |
| F6 | 真正的零拷贝发布（需 SDK 导出 hb_mem handle + 生命周期契约） | **E3 已证明 plain-ROS 路径开销可接受（≈6.9 ms/帧）**，因此本项属**优化而非缺陷**，优先级低于第 1 版 |
| F7 | 与 `hobot_yolo_world` 的官方组合示例（NV12→bgr8→推理→网页） | 证明"非 mipi_cam 相机也能进 TROS 感知生态" |
| F8 | 多相机/多实例支持 | GS130 是独占设备，除非硬件方案变化，否则**明确不做**（建议长期非目标） |

### 8.3 建议删除的非目标（避免范围膨胀）

`PointCloud2` / `disparity` / `image_rect` / `image_color` / 自研压缩插件 / `hobot_hdmi` / `hobot_visualization`——全部属既有节点职责，本包不应出现（与冻结书 §0.2 一致）。

---

## 9. 冻结前的问题：**哪些已被 E1–E7 关闭，哪些仍然开放**（对应要求 8）

> 第 1 版在此提出三个问题（Q1 朝向/分辨率、Q2 QoS、Q3 `rect` 外参语义）。**E1–E7 关闭了 Q2，并部分关闭了 Q1**（分辨率可用性判据已由源码推导 + E6 证明拼接可行）。
> 本节改为"已关闭项 + 仍然开放项"，并指出**新的开放项**——它们恰恰是因为实测才暴露出来的。

### 9.1 已关闭（不再需要评审裁定，直接按实测实现）

| 原问题 | 关闭依据 | 定论 |
|---|---|---|
| **Q2：codec 要 `BEST_EFFORT` 还是 `RELIABLE`？** | **E1** | **`RELIABLE`**。冻结书 §2.1 与 `22_risk_and_safety.md` §4.3 的 `BEST_EFFORT` 相关表述必须改。`hobot_shm` 不能替代（E3：不设 reliability） |
| **Q1（部分）：`height` 用哪种语义？** | **E2** | **`height = 真实高 H`**。按 `H*3//2` 填会让 codec SEGFAULT。冻结书 §3.1 必须改 |
| **零拷贝是否值得做？** | **E3** | **不做**。正确路径 1.36 ms / 1.38 MB、整帧 ≈6.9 ms（30 fps 下 21%），余量充足；真零拷贝需改 `core/` |
| **复用链路是否真的能跑通？** | **E4** | **能，且已达标**（29.966 / 29.974 Hz，HTTP 200）。M11 从此是**回归基线** |
| **时间戳时钟域 / TF 是否因此不可做？** | **E5** | **可做**。设备时钟为 `CLOCK_MONOTONIC`，用启动一次性偏移换算到系统时钟域；残余不确定度 ≤ 一个帧周期。TF 的反对理由被消除 |
| **拼接帧是否必须在驱动侧手工拼？** | **E6** | **不必**。SDK 的 `StereoLayout.LEFT_RIGHT` 在 `mode != raw` 下直接给出与 `width=2W` 约定吻合的帧；RAW 模式**不能**拼接，必须 fail-fast |
| **是否需要第二个 web 端口来显示多路？** | **E7** | **不需要**。`websocket` 的 `channel` 参数在同一 `:8000` 上区分。第 1 版 §2.5 的"端口从 8000 变 8080"表述已修正 |

### 9.2 仍然开放（需要在冻结/做 stereonet 之前回答）

| # | 问题 | 为什么仍然开放 | 怎么回答 | 答案如何改变期望 |
|---|---|---|---|---|
| **Q-A** | **`CameraInfo.P` 是否允许非 0？（原 C3）** | E1–E7 **未覆盖**；冻结书规定 `P` 全 0（`11:254`），而 stereonet 的 `calib_method:=none` 必须有 `P[3]/P[0] = baseline` | `ros2 topic echo --once /image_left/camera_info --field p` 看当前实现；再决定是否在**新话题**上发布非 0 `P` | 若允许**只在新话题**上非 0 ⇒ stereonet 可走 `none` 路径，F2（YAML）降为可选；若一律全 0 ⇒ stereonet 必须走 `calib_method:=custom` + F2 |
| **Q-B** | **baseline 的符号与坐标系（原 C4）** | E1–E7 **未覆盖**；SDK 只给 `T`（源原点在参考系中的坐标），从它到 `P[3]` 的换算规则（含符号、是否取左目为参考）**没有契约** | 跑 `python/test/test_gs130.py ... raw` 打印 `relative_T(CAMERA_LEFT,CAMERA_RIGHT)` 与 `relative_T(CAMERA_RIGHT,CAMERA_LEFT)`，与 `fx` 一起构造 `P`，再用 stereonet 的日志对比 `[fx,fy,cx,cy,baseline,doffs]` 是否合理 | 符号定错 ⇒ 深度符号全反（表现为"近处为负/图全黑"）；这条**必须**用一次实跑闭合，不能靠推理 |
| **Q-C** | **`mode=rect` 下外参是否为虚拟值（原 Q3）** | E1–E7 **未覆盖**；F12/E3 与 `gs130.h:341` 都提示 `stereo_rectify()` 会就地改写外参 | 用 `test_gs130.py` 的 `Extrinsics` 段对比 `raw` 与 `rect` 两种模式的 `‖T_left - T_right‖` | 若二者一致（≤1 mm）⇒ TF/`P` 可跨 `mode` 统一发布；若不一致 ⇒ 必须按 `mode` 分支，且文档要写明"`rect` 下外参不是物理安装关系" |
| **Q-D** | **`hobot_stereonet` 的 launch 是否真的会拉起 `mipi_cam` 并抢占设备（原 C7）** | E1–E7 **未覆盖**；源码显示默认 `use_mipi_cam=True`，但**没有在本板验证** | 在**不接相机**的情况下先跑一次 `use_mipi_cam:=False`，确认不产生 `mipi_cam` 进程；再决定是否需要包装一个"安全 launch" | 若默认就抢设备 ⇒ 必须提供我们自己的 launch 包装（显式 `use_mipi_cam:=False`），否则任何用户按官方文档操作都会踩相机独占 |
| **Q-E（新增）** | **偏移换算在长时间运行/`use_sim_time` 下的漂移与语义** | E5 只证明了"一次偏移 + 单帧测量"成立，**未覆盖**数小时漂移与 `use_sim_time:=true` 的组合 | 长跑 2–4 h 记录 `(stamp - wall)` 的漂移；另起一次 `use_sim_time:=true` 观察 `WARN` 与时间戳来源 | 若漂移显著（> 一个帧周期）⇒ 文档必须给出"长时间运行需重启节点"或提供周期性重同步；若 `use_sim_time` 下行为不可预期 ⇒ 该参数应被文档标为"不推荐/实验性" |
| **Q-F（新增）** | **`/imu/data` 的 QoS 是否也要从 `BEST_EFFORT` 改为 `RELIABLE`** | E1 只测了**图像**话题；IMU 没有 RELIABLE 强制消费者（E1 的机制不适用），但"图像 RELIABLE / IMU BEST_EFFORT"的不一致会给用户在 `ros2 topic info` 时造成困惑 | 检查下游是否有 IMU 的 RELIABLE 消费者；若无，可保留 `BEST_EFFORT` 并在文档写明"图像与 IMU 的 QoS 故意不同" | 若统一为 RELIABLE ⇒ 契约更简单、用户的 `--qos-reliability` 心智负担更低；若保持差异 ⇒ 文档必须**显式**列出两者不同，否则会被当成 bug 报告 |

---

## 附录 A：把本文的"待验证"变成证据的最小命令集

> 全部在板端执行，**每条一次启动、一次采集**。执行前先确认没有 `mipi_cam` 在跑（§7.2）。
> **状态列**说明本项在 E1–E7 之后是否还需要做：`✅已覆盖` 表示已有实测证据，保留此行仅为回归对照；`⬜待做` 表示**仍未被任何实验覆盖**。

| # | 目的 | 命令 | 判定 | 状态 |
|---|---|---|---|---|
| V1 | 链路是否存在 | `ros2 topic list \| grep -E 'image_combine_raw\|image_combine_jpeg\|imu/data'` | 4 个话题都在 | ⬜待做（E4 只覆盖了 raw 与 jpeg 两路） |
| V2 | 编码与尺寸 | `--field encoding` / `--field width` / `--field height` / `--field step` | `nv12`；**`height` 必须等于真实高**（`640×480` ⇒ `height: 480`、`width: 1280`、`step: 1280`） | **✅已覆盖（E2）**，作为 P0 回归项保留 |
| V3 | QoS 方向 | `ros2 topic info -v /image_combine_raw` | Publisher **必须 `RELIABLE`**；与 codec 订阅相容 | **✅已覆盖（E1）**，作为 P0 回归项保留 |
| V4 | 网页链路 | `curl -s -o /dev/null -w '%{http_code}\n' http://127.0.0.1:8000` + `ros2 topic hz /image_combine_jpeg` | 200；`/image_combine_raw` ≈29.97 Hz、jpeg ≈29.97 Hz | **✅已覆盖（E4）**，作为回归基线（M11） |
| V5 | 时间戳 | `--field header.stamp` 与 `date +%s%N` 比对，连续 10 帧 | 严格单调；相邻差 ≈ 1/fps（±10%）；**且量级与墙钟一致**（不是 9.5e12 那种开机时间）；差 ≈ 墙钟而非 -1.79e18 | ⬜待做（E5 测的是**原始 SDK 值**，**换算后**的正确性需在实现上验证） |
| V6 | 标定一致性 | `--once /image_left/camera_info` 与 SDK `camera_intrinsics(LEFT).K` 对比 | 逐元素一致（相对误差 ≤ 1e-6） | ⬜待做 |
| V7 | 分辨率可用性 | 依次试 `544×640`、`640×480`、`800×600`、`864×480` | 前三个成功；`864×480` fail-fast 且错误点名判据与建议值 | ⬜待做（判据来自源码 F11，未实测） |
| V8 | **外参语义**（原 Q3 → §9.2 Q-C） | 跑 `test_gs130.py ... raw ...` 与 `... rect ...`，比较 Extrinsics 段 | `‖T_left-T_right‖` 两次之差 ≤ 1 mm（→ 可统一）或明显不同（→ 必须分模式） | ⬜**待做，仍开放** |
| V9 | `hobot_shm` 实际影响 | §4.3 的 A/B 对照 | 记录 CPU 与 `hz`，写入文档"实测值" | ⬜待做（E3 已给出**结论**：不使用；此对照仅供好奇者） |
| V10 | 冲突行为 | 先起 `mipi_cam`，再起 `gs130_node` | 非 0 退出 + `ERROR` 含相机占用提示 | ⬜待做 |
| **V11** | **baseline 符号/坐标系**（§9.2 Q-B） | 打印 `relative_T(CAMERA_LEFT,CAMERA_RIGHT)` 与反向，构造 `P`；与 stereonet 日志的 `[fx,fy,cx,cy,baseline,doffs]` 对照 | 符号正确、`baseline` 与 `‖T‖`（米）一致 | ⬜**待做，仍开放** |
| **V12** | **`CameraInfo.P` 策略**（§9.2 Q-A） | `--once /image_left/camera_info --field p` | 按裁定结果（全 0 或非 0）逐元素一致 | ⬜**待做，仍开放** |
| **V13** | **stereonet 是否拉起 `mipi_cam`**（§9.2 Q-D） | 不接相机跑一次 `use_mipi_cam:=False` 的 stereonet launch；`pgrep -af mipi_cam` | 无 `mipi_cam` 进程；stereonet 能收到我们的（`TOP_BOTTOM`）话题 | ⬜**待做，仍开放** |
| **V14** | **时钟偏移长跑漂移 / `use_sim_time`**（§9.2 Q-E） | 长跑 2–4 h 记录 `(stamp - wall)`；另起一次 `use_sim_time:=true` | 漂移 ≤ 一个帧周期；`use_sim_time` 行为与文档一致 | ⬜**待做，仍开放** |
| **V15** | **`/imu/data` 的 QoS 取值**（§9.2 Q-F） | `ros2 topic info -v /imu/data` + 检查下游是否有 RELIABLE 消费者 | 与契约一致；若与图像不同需在文档显式说明 | ⬜**待做，仍开放** |

## 附录 B：本文引用的上游证据位置

| 证据 | 位置 |
|---|---|
| SDK 拼接布局与内存平面顺序 | `core/src/gs130.cpp:249-304` |
| SDK 紧排 NV12 取帧 | `core/src/devices/pipeline/rdkx5/rdkx5.cpp:377-383` |
| SDK 模式/分辨率约束 | `rdkx5.cpp:109-125`、`178-247`、`240-272`；`vse.c:13-41` |
| SDK 标定与外参语义 | `core/include/gs130.h:309-448`；`core/src/base/rectify/rectify.hpp:20-27` |
| EEPROM 鱼眼 4 参数与 `install_angle=0` | `core/src/devices/eeprom/union_stereo_imu_fisheye_v1p2r0n4.cpp:17-23,51-54,139` |
| Python 缓冲所有权 | `python/gs130/_types.py:11-36`；`python/gs130/_config.py:79` |
| `hobot_codec` 订阅 QoS + 参数表 | `hobot_codec/src/hobot_codec_node.cpp:25,279-290,366-428`；`hobot_codec/README_cn.md` |
| `websocket` 输入与端口 | `hobot_websocket/README_cn.md:80,96-98,251-261` |
| `hobot_shm` 作用范围 | `hobot_shm/README.md`；`hobot_codec_node.cpp:384-400` |
| `hobot_stereonet` 输入契约 | `stereonet_component.cpp:53-184,558-559,641-720,869-989,1036-1063`；`launch/stereonet_model.launch.py:44-109`；`include/stereo_rectify.h`；`src/stereo_rectify.cpp:21-310`；`script/run_cam.sh`；`script/run_stereo.sh` |
| `mipi_cam` 拼接实现 | `hobot_mipi_cam/src/hobot_mipi_cap.cpp:125-145`；`src/hobot_mipi_node.cpp:270-320`；`launch/mipi_cam_dual_channel.launch.py` |
| `hbm_img_msgs` 上限 | `hobot_msgs/hbm_img_msgs/msg/HbmMsg1080P.msg`；`README_cn.md` |
| `hobot_yolo_world` 输入 | `hobot_yolo_world/README_cn.md:114-133` |
| `mot` 的输入是 `ai_msgs` | `mot/README_cn.md:76-92` |
| `hobot_hdmi` / `hobot_visualization` / `hobot_cv` | 各自 `README_cn.md` |

---

## 附录 C：板端实测证据原始记录（E1–E7，**本文最高等级证据的来源**）

| # | 结论一句话 | 原始观测 | 覆盖了第 1 版的什么 |
|---|---|---|---|
| E1 | 集成 `hobot_codec` **必须** `RELIABLE` publisher | codec 日志：`New publisher discovered ... offering incompatible QoS. No messages will be sent to it. Last incompatible policy: RELIABILITY_QOS_POLICY`；`BEST_EFFORT` 下**零数据到达** | 关闭 Q2 / X2；推翻"图像用 `SensorDataQoS`"的惯例 |
| E2 | codec 把 `height` 当**真实高**；按 `H*3//2` 发布 → **SEGFAULT** | codec 日志 `init_pic_h_: 1080, alined_pic_h_: 1088` + 崩溃 | 关闭 X3；**推翻**第 1 版"两种语义各自自洽、可取舍"的论述 |
| E3 | **不用** `hobot_shm` 零拷贝；正确拷贝路径开销小 | `array.array("B")+frombytes` = **1.36 ms / 1.38 MB**；节点整帧 `build 0.53 ms / fill 1.28 ms / publish 5.07 ms @30 fps`；朴素 `msg.data = frame.tobytes()` = **1103.87 ms**；`shm_fastdds.xml` **不设 reliability** | 关闭 §4 的开放问题；给出 M8 与"不能靠 shm 修 QoS"的结论 |
| E4 | 复用链路全速率工作 | `/image_combine_raw` **29.966 Hz**；`/image_combine_jpeg` **29.974 Hz**；codec `Sub imgRaw fps 29.98` / `Pub img fps 29.98`；`http://127.0.0.1:8000/` → **HTTP 200**；nginx 由 `websocket.launch.py` 经 `os.system` 启动且**launch 退出后成为孤儿进程** | 把 M9 从"待验证目标"升级为**回归基线**（M11）；新增 §7.5 清理要求 |
| E5 | SDK 时间戳是 **`CLOCK_MONOTONIC`**（非 Unix epoch），图像与 IMU 同域 | frame `9546841822000` vs monotonic `9540690105753`（≈9480 s uptime）；相对墙钟 **≈ -1.79e18 ns**；`imu - frame = 8.6 ms` | 关闭"时钟域不确定"；确立 M4 与 §5.1；消除 TF 的一条反对理由 |
| E6 | **硬件拼接可用且便宜**，RAW 模式**不能**拼接 | `stereo_layout = StereoLayout.LEFT_RIGHT`（`Config.preset()` 之后）→ `640×480` 配置下形状 **(720, 1280)**，与 `width=2W` 约定吻合；RAW 强制 `1088×1280` | 关闭 X1 的默认链路部分；新增 M12/M13；把 stereonet 的布局门槛从"需要旋转"降为"需要第二路话题"（§2.4 A′） |
| E7 | `websocket` 用**同一 :8000 端口**上的 `channel` 区分多路；板端已具备所需依赖 | 132GS launch 用 channel 0/1；板端有 `colcon`、`rclpy`、`sensor_msgs`、`geometry_msgs`、`tf2_msgs`、OpenCV 4.11.0、numpy 1.26.4 | 修正 §2.5 的"端口变更"表述；**消除 TF 建议的依赖成本**（`tf2_msgs` 已装） |

> **明确没有覆盖的范围（不得当作已验证）**：`hobot_stereonet` 从未被启动；`CameraInfo.P`/baseline 语义未被验证；`mode=rect` 的外参语义未被验证；`hobot_yolo_world` 未被启动；`/imu/data` 的 QoS 未单独测量；长时间运行的时钟漂移未测量。这些构成 §9.2 的开放问题 Q-A…Q-F。

## 附录 D：变更记录

| 版本 | 日期 | 变更 |
|---|---|---|
| **v0.2（当前）** | 收到主架构师板端实测 E1–E7 后的修订 | ① 新增 `[实测-板端]` 证据等级、§0.3 实测事实表、附录 C 原始记录；② **关闭 X2（QoS 必须 `RELIABLE`）与 X3（`height` 必须为真实高，否则 codec SEGFAULT）**；③ **修正被实测推翻的论述**：NV12 `height` "两种语义各自自洽"（E2）、"图像用 `SensorDataQoS`"（E1）、"stereonet 需要 90° 旋转"（E6）、"接入 stereonet 需要换 8080 端口"（E7）、"零拷贝是待验证的优化"（E3）、"`tobytes()` 只是多一次拷贝"（E3：实际慢约 800×）；④ must-have 由 M1–M10 扩为 **M1–M14**（新增 `height`、`RELIABLE`、`array.array` 拷贝、E4 回归基线、SDK 硬件拼接、RAW×拼接 fail-fast）；⑤ §4 由开放问题改为结论 + 实测数字；⑥ 新增 §5.1 时间戳时钟域规则（E5）；⑦ §6 CLI 工具前提按 E1/E2 全面修正（含 bag 的 durability 陷阱、`hz` 无法暴露时间戳错误）；⑧ 新增 §7.5 孤儿 nginx 清理；⑨ §9 由"三个开放问题"改为"已关闭 7 项 + 仍开放 6 项（Q-A…Q-F）"；⑩ TF 建议加强（E5/E7 消除两条反对理由） |
| v0.1 | 第 32 号文档首次交付 | 建立生态集成期望：7 项集成评估、stereonet 深度评估与 follow-up 路径、零拷贝结论、TF 建议、CLI 工具前提与陷阱、共存规则、M1–M10 must-have 与 F1–F8 follow-up、Q1–Q3 冻结前问题 |
