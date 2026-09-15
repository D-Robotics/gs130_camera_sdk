# 62 时间戳漂移实验结果

对 A1 的 D-3 攻击（"一次性 offset 的精度承诺可疑，漂移可能超出承诺"）的判决实验。

## 1. 方法

图像话题每帧 6.2 MB，用 Python 计时会把测量本身变成负载，因此改用 **IMU 话题**：
IMU 与图像共用同一个 offset，而消息只有几十字节。订阅端在回调里记录
`接收时刻 − header.stamp`，以约 1 Hz 采样，共 900 s（898 个样本）。

该差值 = offset 误差 + 消息投递延迟 + 采样时刻的排队延迟。

## 2. 结果

```
samples 898 over 900 s
latency ms: first -0.12  last -17.20  min -23.64  max 1.78
drift over 900 s: -17.07 ms (-1.139 ms/min)
fitted slope: 0.047 ms/min
VERDICT PASS one-time offset holds
```

节点在同一会话保持健康：`frames=27450`（30 fps）、`imu=184899`（≈202 Hz），
SIGINT 后 `camera released`、`process has finished cleanly`、无残留进程。

## 3. 裁决

### 3.1 漂移：**承诺成立，攻击不成立**

最小二乘拟合斜率 **0.047 ms/min**，即约 2.8 ms/小时。
一个 12 分钟的会话里累计漂移不到 1 ms。
因此"启动时取一次 offset、运行期保持恒定"的做法在本平台上是成立的，
D-3 关于"必须改为周期性重算"的要求**不采纳**。

注意不要用首末差值（−17 ms）当漂移：那主要是投递延迟抖动（见下），
不是时钟漂移；判据必须用拟合斜率。

### 3.2 但暴露了一个真实问题：**IMU 投递延迟抖动约 25 ms**

延迟范围 **−23.64 ms 到 +1.78 ms**，即约 25 ms 的抖动。

原因：单个 executor 线程同时跑图像定时器与 IMU 定时器，
一次图像回调要拷贝 6.2 MB 并发布，IMU 回调排在它后面。

**这不影响 `header.stamp` 的正确性**（stamp 由设备时间戳 + 常量 offset 得到，
与投递时刻无关），但影响消费者对"时间戳到数据可用"的预期：

- 对时间同步要求高的用户（例如与图像做插值融合），IMU 消息的**到达时刻**
  可能比其时间戳晚数十毫秒。
- 消费者应按 `header.stamp` 对齐，而不是按到达顺序——这一点已写入 README。

这与 61 号实验的结论一致：单线程模型**不丢帧、不漂移**，
代价是 IMU 投递抖动可达约 25 ms。若未来需要更低的抖动，
应把取帧与 publish 解耦（B1 在 `52_architecture_cpp.md` 提出的方案 B），
本项目 v0.1.0 不做。

## 4. 据此更新的文档

| 文档 | 更新 |
|---|---|
| `ros/README.md` | 时间戳一节补充：offset 稳定（拟合漂移 < 0.05 ms/min），但 IMU 投递延迟抖动可达约 25 ms，消费者应严格按 `header.stamp` 对齐 |
| `00_verified_platform_facts.md` | E15：offset 漂移与 IMU 投递抖动的实测值 |
| `60_adversarial_review.md` | D-3 裁决更新为"不采纳"，并记录原因 |
