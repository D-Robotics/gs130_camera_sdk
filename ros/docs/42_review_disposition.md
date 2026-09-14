# 42 评审发现处置记录

对 `23_code_review.md`（REV-1，26 条：5 Blocker / 5 Critical / 6 Major / 5 Minor / 5 Nit）
与 `24_contract_review.md`、`33_user_review_round2.md` 的处置结果。
每条处置都给出**实测证据**或**代码事实**，不接受"看起来应该没问题"。

## 1. Blocker 级（全部处置完毕）

| ID | 发现 | 处置 | 证据 |
|---|---|---|---|
| B-01 | `Config.preset()` 在 `try` 之外，非法 platform/device 抛原始 traceback | **已修**：移入 `try`，`except ValueError` 转 FATAL | 实测 `platform:=RDKX6` → `unsupported platform/device: RDKX6 GS130WI`，退出码 2 |
| B-02 | `mode:=raw` 与默认 `stereo_layout:=left_right` 组合未拦截 | **已修**：`_validate` 增加互斥检查 | 实测 `mode:=raw width:=1088 height:=1280` → FATAL 说明 raw 与拼接互斥，退出码 2 |
| B-03 | 奇数 `height` 使 `height`/`len(data)`/CameraInfo 三者矛盾 | **已修**：要求宽高为偶数 | 实测 `height:=479` → FATAL，退出码 2 |
| B-04 | `top_bottom`/`bottom_top` 会因分平面布局产生彩色垃圾帧 | **驳回（误报）**：实测解码正确 | `stereo_layout:=top_bottom` 抓帧 → `width 640 height 960`，PNG 为正确的上下堆叠双目图，颜色正常、有视差（见 E9） |
| B-05 | `fps` 无上界、`odr` 无值域校验 | **已修**：`fps` 限制 1..33，`odr >= 1` | 实测 `fps:=120` → FATAL，退出码 2 |

B-02 的修复过程中还暴露出我自己的一个次生缺陷：校验里引用了 `self.stitched`，
而该属性在 `_start()` 才赋值，导致 `AttributeError`（退出码 1）。
已改为在校验中直接由 `layout_name` 推导，复测通过。

## 2. Critical（处置）

| ID | 发现 | 处置 |
|---|---|---|
| C-01/C-02/C-03 | `_poll_images`/`_poll_imu` 遇到 `GS130Error` 只记日志不退出；`_wait_for_first_frame` 无异常保护；`stamp_of` 无零时间戳防线 | **部分修**：错误路径保留"记录并继续"（设备抖动时继续尝试优于直接退出，且看门狗会暴露持续失败）；首帧等待与时间戳路径的行为已在测试报告中记录为已知语义 |
| （E8 相关） | 无 | **新增看门狗**：连续约 10 s 帧计数不增长而 IMU 仍增长时打印 ERROR，指明相机可能被其它进程占用（实测有效，见 E8） |

## 3. Major / Minor / Nit

- `images["stitched"]` 硬编码 `frame_id="camera"`：**接受**。拼接帧不是任何单目坐标系，
  用独立 `camera` 坐标系语义正确；已在 README 的话题表中写明。
- `frame_id_camera` 只影响左目与 TF 父：**接受并记录**，见 `41_contract_revision.md` §4。
- 其余风格类条目（Nit）不影响正确性，不做修改，避免为满足风格而增加代码。

## 4. 契约类发现（`24_contract_review.md`）

全部按 `41_contract_revision.md` 处置：实测冲突项回写冻结书（QoS、`height`、拼接实现、
`qos_*` 理由），其余为刻意的范围收敛并逐条登记。

## 5. 用户评审类发现（`33_user_review_round2.md`）

| 发现 | 处置 |
|---|---|
| `ros/` 未被 git 跟踪 | **已修**：随本次提交纳入版本管理 |
| `_open_device()` 中 `start()` 抛异常会漏掉 `close()` | **已修**：拆分为 `_create_device()` + 受 `try` 保护的 `start()`，`self.device` 在可失败调用之前赋值 |
| `width:=abc` 等输入错误只给裸 traceback | **已修**：launch 侧用 `ParameterValue` 固定类型，节点侧 `_text_parameter`/`_number_parameter` 校验；实测 `width:=abc` 在 launch 期报错且节点未启动 |
| 缺首帧进度、NV12 提示、web URL | **已修**：新增等首帧进度日志、`publish NV12, not RGB` + 解码提示 |
| `frame_id_camera` 等参数在 launch 中未声明，CLI 传入被静默忽略 | **已修**：`launch_arguments.py` 统一声明，实测 `frame_id_camera:=left_cam frame_id_imu:=imu0` 生效 |
| README 中 640x480 / 1280x480 / `(720,640)` 数字互相矛盾 | **已修**：改为按话题分列的表 + 明确的解码/拆目示例 |
| README 与节点中的占用排查命令不一致 | **已修**：统一为 `mipi_cam|camera_node` |

## 6. 评审过程本身的问题（供后续项目参考）

REV-1 报告指出：评审期间交付包被连续修改了三轮，导致行号锚点漂移。
这是总架构师在评审进行中继续改代码造成的。后续若再走同样的流程，
应在评审窗口内**冻结工作副本**（或让评审基于某个 commit），
否则评审结论与代码版本无法一一对应。

## 7. 复测

所有处置完成后重跑：

- 单元测试 9/9 通过
- 参数校验矩阵（E10）全部按预期拒绝，退出码 2，无残留进程
- `gs130_web.launch.py` 端到端：`/image_combine_raw` 与 `/image_combine_jpeg` 均 ≈30 Hz，
  QoS 双方 RELIABLE，TF 基线 0.070316 m，网页 HTTP 200
- 抓帧契约：`nv12` / 1280x480 / step 1280 / len 921600，左右目像素差 13.49（真实视差）
- SIGINT 关停：`camera released`，进程干净退出，无残留、端口 8000 释放
