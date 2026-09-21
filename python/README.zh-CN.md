# GS130 Camera SDK —— Python 接口

**简体中文** | [English](README.md)

> 用 `ctypes` 封装 GS130 双目相机、IMU 与标定数据，对外直接返回 numpy 数组。

---

## 📖 简介

`gs130_camera` 是 `libgs130` 的 Python 接口。它用 `ctypes` 对应 C ABI，把结果转成 numpy 数组，因此一个采集循环只需几行普通 Python。绑定本身不做图像处理、立体匹配或滤波，也不自带 `libgs130`。

| 项目 | 说明 |
| --- | --- |
| Python | 3.10 或更高 |
| 原生库 | `libgs130.so`（`gs130-camera` 软件包） |
| 运行时依赖 | `numpy` |
| 类型支持 | 自带 `py.typed` 与 `__init__.pyi` |
| 支持平台 | RDK X5、RDK S100（RDK S600 开发中） |

## 🧩 依赖

- Python 3.10 或更高，以及 `numpy`（>= 1.20）。
- 与本绑定版本匹配的 `libgs130`。wheel **不包含**原生库；由于绑定直接对应 C 结构体，两者版本必须兼容。绑定的查找顺序为：

  1. 环境变量 `GS130_LIB`，
  2. `ldconfig`（通过 `ctypes.util.find_library`），
  3. 默认搜索路径下的 `libgs130.so`。

  首次加载原生库时，会比较包中记录的版本与库自己报告的版本：库更旧会拒绝加载，库更新只给出告警。`library_version()` 与 `library_platform()` 会返回实际加载的版本与平台。

## 🔨 安装

**1. 先安装原生库**

```bash
# 检查是否已安装
dpkg -s gs130-camera

# 若未安装，安装由 core/ 构建出的 deb（在 SDK 源码目录执行 make deb）
sudo dpkg -i gs130-camera_<version>+<platform>_<arch>.deb
```

**2. 构建并安装绑定**

```bash
# 在板端构建（需要已安装 setuptools 与 wheel）
cd python && ./build-wheel.sh

# 安装构建出来的 wheel
python3 -m pip install dist/gs130_camera-*.whl
```

脚本以 `pip wheel --no-deps --no-build-isolation` 构建，使用板卡上已有的 `setuptools` 与 `wheel`，不联网下载。版本号来自仓库根目录的 `VERSION` 文件，因此需要从源码目录构建；`./build-wheel.sh clean` 会清理 `build/`、`dist/` 与 `*.egg-info`。

## 🚀 快速开始

```python
import gs130_camera

config = gs130_camera.preset(
    "GS130WI", gs130_camera.CameraMode.RECT, 544, 448, 30, 200
)

with gs130_camera.Device(config) as device:
    device.start()

    while device.available_camera() == 0:
        pass
    images = device.read_image()
    print(images["left"].shape, images["left"].timestamp_ns)

    packet = device.read_imu()
    if packet is not None:
        print(packet.accel, packet.gyro, packet.temp)
```

`preset()` 会为已知硬件填好全部字段；若想自己设置字段，可用 `config()` 得到同样结构、但全部未设置的字典。两种方式得到的都是普通嵌套字典，可以在交给 `Device` 之前直接修改：

```python
config["camera_config"]["stereo_layout"] = gs130_camera.StereoLayout.TOP_BOTTOM
# 之后 read_image() 返回 {"stitched": Image}，而不再是 {"left": ..., "right": ...}
```

读取标定、以及用 OpenCV 转换一帧：

```python
calibration = device.calibration()
print(calibration.camera_left.K, calibration.install_angle)

import cv2
bgr = cv2.cvtColor(images["left"], cv2.COLOR_YUV2BGR_NV12)
```

相机型号不是平台参数：`preset()` 与 C 的 `GS130_CONFIG` 宏一样，从已加载的原生库读取平台。没有预设的硬件会抛出 `ValueError`，而宏的行为是打印到 stderr 并退出。

## 📡 API

`read_image()` 与 `read_imu()` 仅在"当前没有数据可取"时返回 `None`，因此采集循环只需判断这一点；其他失败一律抛出 `GS130Error`。

### 模块级

| 名称 | 说明 |
| --- | --- |
| `preset(device, mode, width, height, fps, odr)` | 为已知硬件生成填好的配置 |
| `config()` | 生成空配置，全部字段未设置 |
| `library_version()` / `library_platform()` | 已加载 `libgs130` 的版本与平台 |
| `package_version()` / `__version__` | 构建 wheel 时记录的版本，未安装时为 `None` |

### Device

| 成员 | 说明 |
| --- | --- |
| `Device(config)` | 校验配置、初始化硬件并探测 IMU |
| `start()` / `stop()` | 开始与结束采集；`start()` 返回 `self` |
| `close()` | 结束采集并释放全部资源；重复调用安全 |
| `closed`、`stitched` | 是否已释放，以及帧是否为拼接帧 |
| `imu_name`、`imu_info`、`eeprom_name`、`eeprom_info` | 识别信息字符串，不存在时为 `None` |
| `available_camera()`、`available_imu()` | 队列深度；未开始采集时为 `0` |
| `read_image()` | 返回 `{"stitched": Image}` 或 `{"left": Image, "right": Image}`，无数据时为 `None` |
| `read_imu()` | 返回一个 `ImuPacket`，无数据时为 `None` |
| `calibration()` | 从 EEPROM 读取的完整双目与 IMU 标定 |
| `camera_intrinsics(camera)`、`imu_intrinsics()` | 单目内参，或 IMU 内参 |
| `relative_R(from, to)`、`relative_T(from, to)` | 两个参考系之间的旋转 `(3, 3)` 与平移 `(3,)` |
| `convert_calibration(ref_frame, R, T)` | 更换参考系，同时保持各设备位姿不变 |

`relative_*` 与 `convert_calibration` 使用 `ReferenceFrame` 枚举；`R` 必须为 9 个值、`T` 为 3 个值，长度不对会在调用 C 之前抛出 `ValueError`。

### 返回值类型

| 类型 | 说明 |
| --- | --- |
| `Image` | `np.ndarray` 子类，形状 `(height * 3 // 2, width)`，`uint8`；带 `timestamp_ns`、`width`、`height`，以及零拷贝视图 `y_plane()` / `uv_plane()` |
| `ImuPacket` | `accel` `(3,)` m/s²、`gyro` `(3,)` rad/s、`temp` ℃、`is_fsync`、`timestamp_ns` |
| `CameraIntrinsics` | `fx`、`fy`、`cx`、`cy`、`K` `(3, 3)`、`dist_coeffs` `(8,)`、`dist_model` |
| `ImuIntrinsics` | 加速度计与陀螺仪的失准、刻度、零偏、噪声与随机游走 |
| `Calibration` | 双目内参、各组 `R` / `T` 位姿，以及 `install_angle` |
| `GS130Error` | 调用失败：`.code`、`.reason`、`.func`；继承 `RuntimeError` |
| 枚举 | `ErrorCode`、`CameraMode`、`CameraIndex`、`StereoLayout`、`FifoMode`、`DistModel`、`ReferenceFrame` |

## ⚙️ 配置

配置是一个普通嵌套字典，分为 5 个区段，逐字段对应 `gs130_config_t`。`preset()` 会为已知硬件填好全部字段，只有特殊场景才需要手工构造。

| 区段 | 字段 |
| --- | --- |
| `camera_config` | `bus`、`left_addr`、`right_addr`、`sensor_width`、`sensor_height`、`fps`、`line_length`、`frame_length`、`tuning_file`、`output_width`、`output_height`、`mode`、`stereo_layout`、`bus_mipi_rx`、`bus_reset_gpio`、`fsync_camera` |
| `imu_config` | `bus`、`addr`、`odr_hz`、`accel_fsr_g`、`gyro_fsr_dps`、`accel_bw_sel`、`gyro_bw_sel` |
| `eeprom_config` | `bus`、`addr` |
| `camera_fifo` | `depth`、`mode` |
| `imu_fifo` | `depth`、`mode` |

- `bus_mipi_rx` 与 `bus_reset_gpio` 是 `{bus: 槽位}` 形式的字典，因为 C 中对应的数组是稀疏查找表。`bus_num` 不是字段，等于 `len(bus)`。
- `tuning_file` 为路径字符串或 `None`；`None` 表示不加载 tuning 文件，这对 SDK 是有意义的取值，而不是"未设置"。
- 其余字段为 `None` 时，会在构造 `Device` 时报错，不会静默保持为 0。

## ✨ 特性

- **numpy 原生帧**：一帧就是形状 `(height * 3 // 2, width)` 的 `np.ndarray`，`y_plane()` / `uv_plane()` 是零拷贝视图，从 SDK 取出时不复制像素。
- **明确的缓冲区所有权**：帧由 SDK 用 `malloc` 分配并把所有权交给调用方；`Image` 在最后一个引用消失时释放它，因此只要还有视图或切片存活，帧就一直有效。
- **时间戳随切片保留**：`timestamp_ns` 由 `Image` 及其视图携带。返回普通 `numpy.ndarray` 的操作（例如 `cv2.cvtColor`）会丢掉它，需要先读时间戳。
- **便于写循环的读取语义**：没有数据时 `read_image()` / `read_imu()` 返回 `None` 而不抛异常；真正的失败由其他接口抛出 `GS130Error`，携带 SDK 返回的错误码、描述与函数名。
- **枚举取自已加载的库**：所有枚举值在 import 时从 `libgs130` 读出，因此 Python 侧的名称不会与 `gs130.h` 脱节。
- **安全的生命周期**：`Device` 支持 `with` 语句；未关闭时给出 `ResourceWarning` 提示，且关闭过程中的失败不会掩盖正在传播的异常。
- **带类型信息**：包内提供 `py.typed` 与 `__init__.pyi`，编辑器与类型检查器可识别完整 API。

暂不支持：图像处理、立体匹配、IMU 滤波、录制，以及 `libgs130` 自身报告之外的设备发现。

## 🧪 硬件测试

`test/test_gs130.py` 是硬件测试而非单元测试：它会走一遍公开 API，统计队列深度，并把采集到的帧保存为 PNG。

```bash
cd python
python3 -m pip install '.[test]'                     # 测试需要 OpenCV
python3 test/test_gs130.py GS130WI rect 544 448 30 200
```

位置参数依次为 `device`、`mode`（`raw`、`resize` 或 `rect`）、`width`、`height`、`fps`、`odr`。无论是否安装过本包，都可以直接从源码目录运行；本机没有的硬件相关小节会跳过并提示，而不是中断整个测试。帧默认写入 `gs130_images/`，可用 `--output-dir` 指定其他目录，用 `--overwrite` 写入非空目录。

## 📄 许可证

本项目采用 [MIT 许可证](../LICENSE)。
