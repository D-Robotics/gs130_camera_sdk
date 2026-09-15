# 50 C++ 节点设计（待对抗式评审的目标）

本文件是**被攻击对象**。四个团队必须针对本文的具体论断提出可证伪的反对意见，
不得只做补充说明。每条反对意见必须包含：被引用的原文、你认为它会怎样失败、
以及一个能在板端执行的判决性实验。

## 1. 背景与纠正

上一轮的 ROS 接口是 rclpy（Python）节点，包了一层 `python/gs130` 绑定。
这**不是本项目应有的形态**：TROS 是 C/C++ 生态，节点应当直接用 `rclcpp`
链接 `libgs130` 的 C API。板端环境已核实：

```
/usr/include/gs130.h                  # C API 头
/usr/include/gs130_define.h
/lib/aarch64-linux-gnu/libgs130.so    # 0.0.1
/opt/ros/humble/include/rclcpp        # rclcpp 头
/opt/ros/humble/share/ament_cmake
/usr/local/bin/colcon
```

`g++ -std=gnu++17 x.cpp -lgs130` 实测可编译运行。因此 Python 节点整体废弃，
改为 ament_cmake 的 C++ 包。

## 2. 必须遵守的实测事实（来自 00_verified_platform_facts.md）

| 编号 | 事实 | 对 C++ 实现的要求 |
|---|---|---|
| E1 | `hobot_codec` 订阅端为 RELIABLE | 图像 publisher QoS 必须 RELIABLE |
| E2 | codec 把 `Image.height` 当真实图像高 | `height = 真实高`；按打包高会段错误 |
| E4 | 复用 codec+websocket 链路可达 30 fps | 不自研编码器/网页 |
| E5 | SDK 时间戳是 CLOCK_MONOTONIC | 启动时一次性 offset 换算到系统时钟域 |
| E6 | SDK 硬件拼接可用 | 设置 `stereo_layout`，不手工拼帧 |
| E8 | 相机流水线不可共享，第二个打开者不报错只抢帧 | 需要停帧看门狗 |
| E10 | 参数校验矩阵 | 启动期校验并给出明确错误 |

## 3. 本文提出的设计（请攻击）

### 3.1 包结构

```
ros/gs130_ros/
├── CMakeLists.txt          ament_cmake，链接 gs130 / rclcpp / sensor_msgs / tf2_ros
├── package.xml
├── include/gs130_ros/conversions.hpp   纯函数：标定→CameraInfo、时间戳、NV12 几何
├── src/conversions.cpp
├── src/camera_node.cpp                 节点主体
├── launch/gs130_camera.launch.py
├── launch/gs130_web.launch.py
└── test/test_conversions.cpp           gtest，无硬件
```

### 3.2 线程模型（**重点攻击对象**）

主张：**单线程 + 两个 rclcpp 定时器**即可，不需要专用采集线程。

- 图像定时器：周期 `1/(2*fps)`，每次最多取 2 帧（`gs130_get_nv12_frame`）
- IMU 定时器：周期 `1/(2*odr)`，每次最多取 64 包
- 理由：SDK 的读取是非阻塞的，C++ 侧单帧拷贝（~1.4 MB memcpy）量级在百微秒，
  30 fps 下占用可忽略；RMW 回调里做 memcpy 不会阻塞到影响 IMU。

请给出你的反驳：什么条件下这个模型会丢帧、会阻塞、会让 IMU 时间戳抖动？

### 3.3 帧内存所有权（**重点攻击对象**）

主张：在发布回调内 `memcpy` 到 `sensor_msgs::msg::Image::data`，然后立刻
`free(image.data)`。理由是 `gs130_get_nv12_frame` 返回 `malloc` 缓冲、所有权归调用方。

请给出你的反驳：能否真正做到"发布后立即 free"？`publish()` 返回是否代表
RMW 已经完成序列化？在 RELIABLE + 大消息 + 慢订阅者时会发生什么？

### 3.4 时间戳

主张：首帧到达时计算 `offset_ns = now_ns - frame.timestamp_ns`，之后
`stamp = frame.timestamp_ns + offset_ns`，图像与 IMU 共用。

请给出你的反驳：C++ 侧 `now()` 用哪个时钟？`RCL_ROS_TIME` 与
`RCL_SYSTEM_TIME` 的区别会不会让这个 offset 在 `use_sim_time` 下出错？

### 3.5 参数与校验

与 Python 版一致：`platform/device/mode/width/height/fps/odr/stereo_layout/
frame_id_camera/frame_id_imu/publish_imu/publish_tf/start_timeout_s`。
启动期校验并拒绝：非法 mode/layout、奇数宽高、`fps` 超出 1..33、
`mode=raw` 非 1088x1280、`raw` 与拼接互斥。

请给出你的反驳：C++ 侧参数声明用 `declare_parameter` 的哪种形式能在
launch 传入字符串时依然 fail-fast 而不是抛未捕获异常？

### 3.6 关停

主张：`rclcpp::ok()` 循环退出后依次 `gs130_stop` → `gs130_deinit` →
`gs130_destroy`；不依赖任何析构或 GC。

请给出你的反驳：`rclcpp::shutdown()` 与 signal handler 的交互下，
回调正在执行时被中断会怎样？如何保证 `free()` 不遗漏？

## 4. 明确的非目标（不要在这里提需求）

- 不自研编码器、网页、深度网络
- 不做零拷贝 / hb_mem / loaned message（v0.1.0）
- 不做动态参数重配置、多相机实例、ROS 1
- 不实现 `hobot_stereonet` 接线

## 5. 需要团队给出判决性实验的开放问题

1. C++ 单线程定时器模型在 1920x1080@30 下是否稳定，还是必须专用线程？
2. `publish()` 后立即 `free()` 是否安全，还是必须用带自定义 deleter 的缓冲区？
3. `use_sim_time` 打开时 offset 方案是否失效？
4. 1088x1280 的 `rect` 模式实测只有约 20 fps，是否需要限制/告警？
