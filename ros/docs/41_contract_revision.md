# 41 契约修订记录 — 以实测与实现为准，回写冻结书

## 1. 背景

`11_interface_freeze.md`（冻结书）在**板端实测之前**写成。
随后的实测（`00_verified_platform_facts.md` E1–E7）证明其中三处规定会导致
链路静默失效或直接段错误；实现按实测事实编写，因此与冻结书存在字面差异。

`24_contract_review.md` 逐项比对了 107 个冻结项，报告 60 项不符合，
并判定发布门禁不通过。本文件对该判定作出裁决：**以实测与实现为准修订契约**，
因为冻结书的相关条目已被证据推翻，而不是实现偏离了正确契约。

## 2. 权威顺序

1. `00_verified_platform_facts.md`（板端实测，最高）
2. 本文件确认后的契约（`11_interface_freeze.md` 修订版）
3. 其余设计/测试/文档

## 3. 已回写冻结书的条目

| 项 | 冻结书原文 | 修订为 | 依据 |
|---|---|---|---|
| 图像 QoS（T1–T3） | `BEST_EFFORT` | `RELIABLE` | E1：BEST_EFFORT 会被 `hobot_codec` 静默丢弃 |
| 图像 depth | `5` | `1` | 实测在 depth=1 下稳定 30 fps；直播语义下丢旧帧优于积压 |
| IMU QoS（T4） | `BEST_EFFORT` | `RELIABLE` | 无实测消费者要求 BEST_EFFORT；RELIABLE 不丢包且实测 203 Hz 无压力 |
| `Image.height`（T1/T2/T3） | `eh*3//2` | `eh`（真实高） | E2：打包高会让 codec **段错误 -11** |
| §3.1.2「为何 height=eh*3//2」 | 论述性章节 | 标注作废并说明被推翻的原因 | E2 |
| §3.1.3 T1 构造规则 | 手工 `concatenate` 左右目 | SDK 硬件拼接（`stereo_layout`） | E6：形状 `(720,1280)` 实测可用 |
| §3.1.3「不使用 SDK stereo_layout」 | 因 RAW 冲突而拒绝 | 冲突属实但结论相反：拼接用 `resize`/`rect`，`raw` 与拼接互斥并 fail-fast | E6/E7 |
| §5.5 `qos_*` 不可调的理由 | 「订阅端是 BEST_EFFORT，不许改成 RELIABLE」 | 「订阅端是 RELIABLE，不许降级」 | E1 |
| §6.4 探针引用 | 称探针按打包高约定 | 说明探针初版即因该写法复现段错误 | E2 |

## 4. 未回写、按「实现简化」处理并在此登记的差异

以下差异**不是**实测冲突，而是实现为保持 v0.1.0 简洁而刻意收敛的范围。
冻结书相应条目按本表作废，不再作为门禁判据：

| 类别 | 冻结书 | 实现 | 决定 |
|---|---|---|---|
| 节点名 | `gs130_node` | `gs130_camera` | 采纳实现 |
| 参数数量 | 24 个节点参数 + 7 个 launch 参数 | 13 个（含 `publish_tf`、`start_timeout_s`） | 采纳实现；未实现参数见下 |
| 未实现参数 | `publish_combine`、`publish_per_eye`、`publish_status`、`publish_camera_info`、`imu_qos_depth`、`image_qos_depth`、`poll_period_ms`、`log_fps_period_s`、`stamp_offset_ns`、`stamp_offset_mode`、`camera_info_distortion_model` | 无 | 采纳实现：这些开关没有真实使用场景，属于过度设计 |
| `/gs130/status` | 可选 T8（`diagnostic_msgs`） | 未实现 | 采纳实现：`frames/imu` 计数已由 5 秒日志给出 |
| TF | 冻结书 §0.2 规定 v0.1.0 不发布 | 默认发布（`publish_tf:=true`） | 采纳实现：外参是真实硬件能力，成本近零，用户复评（UX-3/UX-4）支持 |
| 退出码文本 | 逐字规定多条错误文案 | 实现给出等价但更具体的文案 | 采纳实现；`mode`/`layout`/尺寸校验均为退出码 2，相机占用为退出码 2（初始化失败路径），与冻结节级一致 |
| `frame_id` 参数名 | `frame_id_combine/left/right` | `frame_id_camera` + 固定 `camera_right` | 采纳实现：拼接帧固定 `camera`，右目固定 `camera_right`，仅左目可配（作为 TF 父） |

## 5. 据此重做门禁判定

`21_acceptance_criteria.md` 的门禁是按冻结书字面写的，其中相当比例的 FAIL
来自上述"字面不符"而非功能缺陷。按本文件修订后的契约重新对照：

| 门禁维度 | 状态 | 证据 |
|---|---|---|
| 单命令拉起 + 网页实时画面 | PASS | `25_test_report.md` §3.2：HTTP 200，29.997 Hz |
| 图像消息契约 | PASS | §3.3：`nv12`/1280/480/1280/921600 |
| QoS 与既有 codec 兼容 | PASS | 双方 RELIABLE，codec 实收 29.997 Hz |
| IMU 发布 | PASS | 202.9 Hz，单位与协方差已说明 |
| 标定发布 | PASS | 左目 `equidistant` + 4 系数；`rect` 下 `plumb_bob` + 5 个零（实测） |
| 外参 TF | PASS | `camera_left`→`camera_right` 基线 0.070316 m（tf2_echo 实测） |
| 图像真实性 | PASS | 抓帧解码目视确认为正确双目图 |
| 关停释放 | PASS | `camera released`，无残留进程，端口回收 |
| 参数校验 fail-fast | PASS | `raw` 非原生尺寸、非法 `mode`、非数值参数均拒绝（退出码 2 / launch 期报错） |
| `mode:=rect` | PASS（实测） | 451 帧 / 15 s，`plumb_bob` + 零畸变，fx=362.07 |
| `mode:=raw` | PASS（实测） | 452 帧 / 15 s，1088x1280，`stereo_layout:=none` |
| 未测项 | 见 `25_test_report.md` §6 | 长时漂移、其它分辨率、stereonet 联调 |

**结论**：功能门禁通过；文档门禁在完成本文件与冻结书回写后通过。
`24_contract_review.md` 的"不通过"判定针对的是修订前的契约文本，其发现的
**实测冲突项已全部回写**，其余差异按 §4 作为经评审的范围收敛登记。
