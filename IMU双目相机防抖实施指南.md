# IMU 双目相机防抖实施指南

本文面向当前 `xr_teleoperate` 项目中的头部双目相机，说明如何利用相机内置 IMU 或外部 IMU 实现数字防抖（EIS），以及在条件允许时如何配合硬件防抖。文档以 PC2 上运行的 `teleimager` 为主要实施位置。

## 1. 目标与适用范围

目标是降低机器人头部运动、机械振动和高频抖动对远程视觉画面的影响，同时保持双目图像的几何一致性、低延迟和足够视场角。

当前项目的图像链路为：

```text
双目相机采集
    -> 图像/IMU 时间同步
    -> 姿态估计
    -> 双目同步防抖
    -> 裁剪、缩放、左右目分流
    -> JPEG/H.264 编码
    -> ZMQ/WebRTC 传输
    -> TeleVuer/VR 显示
```

防抖应放在采集端、编码和网络传输之前。不要只在远端显示端处理，否则网络抖动、丢帧和额外延迟会破坏图像与 IMU 的对应关系。

## 2. 当前代码结构

当前 `teleimager` 已支持 UVC、OpenCV 和 RealSense 相机，但没有通用的 IMU 读取和防抖模块。

主要位置如下：

| 功能 | 文件或类 |
| --- | --- |
| 默认头部双目配置 | `teleop/teleimager/cam_config_server.yaml` |
| RealSense 配置 | `teleop/teleimager/cam_config_server_realsense.yaml` |
| UVC 图像采集 | `OpenCVCamera` |
| UVC 后台取帧 | `OpenCVCamera._capture_loop()` |
| UVC 图像处理和发布 | `OpenCVCamera._update_frame()` |
| RealSense 图像采集 | `RealSenseCamera._update_frame()` |
| 网络发布 | ZMQ/WebRTC publisher |

当前外置双目 UVC 配置采用左右拼接图像，典型参数为：

```yaml
capture_image_shape: [1200, 4000]
image_shape: [720, 2400]
binocular: true
stereo_sbs: true
stereo_split_webrtc: true
fps: 30
```

这类输入应先拆分为左目和右目，再使用同一个时间点的防抖变换分别处理，最后重新拼接或分别发布。

## 3. 首先确认 IMU 数据来源

### 3.1 相机内置 IMU

需要从相机厂商 SDK 获取：

- 陀螺仪角速度 `gyro`，单位统一为 `rad/s`
- 加速度计 `accel`
- IMU 硬件时间戳
- 左右目图像时间戳或曝光时间
- IMU 与图像之间的同步关系
- 相机坐标系定义和轴方向
- 相机与 IMU 之间的安装外参

优先使用硬件时间戳。不要直接用图像线程中的 Python `time.time()` 与 IMU 时间进行匹配，因为线程调度和 USB 缓冲会产生不可忽略的偏差。

### 3.2 Intel RealSense D435i

RealSense D435i 带有 IMU，但当前 `RealSenseCamera` 只启用了彩色流。需要在启动配置时增加 gyro 和 accel 流：

```python
config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
config.enable_stream(rs.stream.gyro)
config.enable_stream(rs.stream.accel)
```

在 `pipeline.wait_for_frames()` 后读取运动帧：

```python
frames = self.pipeline.wait_for_frames()

gyro_frame = frames.first_or_default(rs.stream.gyro)
accel_frame = frames.first_or_default(rs.stream.accel)

if gyro_frame:
    gyro_data = gyro_frame.as_motion_frame().get_motion_data()
    gyro_timestamp_ms = gyro_frame.get_timestamp()

if accel_frame:
    accel_data = accel_frame.as_motion_frame().get_motion_data()
    accel_timestamp_ms = accel_frame.get_timestamp()
```

实际集成时必须验证彩色帧和 IMU 帧的时间基准。RealSense 的 IMU 采样频率通常高于彩色帧率，应将 IMU 样本缓存起来，在每个图像曝光时刻插值或积分。

### 3.3 外置 UVC 双目相机

UVC 图像接口本身通常不能说明设备提供 IMU。当前 `OpenCVCamera` 只接收视频帧；如果双目相机没有公开 IMU 接口，需要增加：

- USB IMU
- 串口 IMU
- SPI/I2C/CAN IMU
- 相机厂商专用 SDK
- 或更换为带同步 IMU 的双目相机

独立 IMU 可以用于 EIS，但必须标定相机到 IMU 的旋转外参，并测量图像和 IMU 的时间偏移。

## 4. 推荐的总体方案

建议分三阶段实现：

### 阶段一：纯陀螺仪旋转防抖

只补偿相机旋转，不处理平移。该方案实现简单、延迟低，适合先验证数据链路、时间同步和坐标系方向。

```text
gyro -> 去零偏 -> 四元数积分 -> 平滑姿态 -> 图像旋转补偿
```

### 阶段二：陀螺仪 + 加速度计姿态融合

使用 Mahony、Madgwick 或 EKF 约束重力方向，降低长期姿态漂移。加速度计主要用于估计重力方向，不应直接对线性加速度进行两次积分来估计相机位移。

### 阶段三：IMU + 视觉或深度辅助

当画面包含明显近景、机器人存在平移或相机安装点距离旋转中心较远时，仅旋转补偿会产生残余视差。此时增加：

- 光流或特征点运动估计
- 双目深度
- RGB-D 深度图
- 视觉惯性里程计 VIO

可对旋转使用 IMU，对残余平移和局部运动使用视觉或深度估计。

## 5. IMU 预处理

### 5.1 单位与坐标系

统一以下约定：

- 角速度：`rad/s`
- 加速度：`m/s^2` 或明确使用 `g`
- 时间：秒
- 右手坐标系
- 明确相机光轴、水平轴和垂直轴方向

建议在程序启动时打印一组静止状态数据，人工确认：

```text
静止时 gyro 应接近零
静止时 accel 的模长应接近 9.81 m/s^2
绕相机 X/Y/Z 轴旋转时，只有预期轴的角速度显著变化
```

### 5.2 陀螺仪零偏

启动时保持相机静止 2 到 5 秒，计算零偏：

```text
gyro_bias = mean(gyro_samples_during_static_period)
gyro_corrected = gyro_raw - gyro_bias
```

运行过程中可以缓慢更新零偏，但不能在剧烈运动时更新。建议设置静止检测条件，例如角速度和加速度变化均低于阈值时才更新。

### 5.3 异常值与丢样本

需要处理以下情况：

- 时间戳倒退
- 时间间隔过大
- USB 或 SDK 丢帧
- 角速度超出传感器量程
- 重复时间戳
- IMU 队列积压

发现时间间隔异常时，不应使用超大的 `dt` 继续积分。可以丢弃该样本并以最近可靠姿态继续输出。

## 6. 姿态估计

### 6.1 四元数积分

对角速度进行离散积分：

```text
delta_angle = gyro_corrected * dt
delta_q = quaternion_from_rotation_vector(delta_angle)
q_current = normalize(q_current * delta_q)
```

四元数比欧拉角更适合连续旋转，避免俯仰接近 90 度时的万向节锁。

### 6.2 加速度计融合

推荐使用 Mahony 或 Madgwick 作为第一版融合算法。基本原则是：

- 陀螺仪负责短期动态响应
- 加速度计用于低频重力方向修正
- 快速运动或碰撞时降低加速度计权重
- 加速度模长明显偏离重力时暂时拒绝加速度计更新

加速度计不能可靠区分重力和机器人线性加速度，因此不应在所有时刻强制把加速度方向当作重力方向。

## 7. 生成平滑姿态

EIS 不能简单地把所有画面锁定到启动时的方向，否则机器人转头时画面会明显拖拽。

应从当前姿态生成平滑目标姿态：

```text
q_smooth = low_pass_filter(q_current)
R_correction = R_smooth * inverse(R_current)
```

可选方法：

- 一阶低通滤波：低延迟，适合第一版
- 指数平滑：参数简单，易于运行时调节
- Savitzky-Golay：离线或允许固定窗口延迟时使用
- 滑动窗口轨迹优化：效果较好，但会增加延迟和计算量

建议将平滑参数设计为配置项，例如：

```yaml
stabilization:
  enabled: false
  mode: gyro
  smoothing_alpha: 0.92
  max_correction_deg: 8.0
  crop_ratio: 0.12
  rolling_shutter: false
```

`smoothing_alpha` 越大，画面越平滑，但响应越慢。建议从 `0.90` 到 `0.97` 范围开始测试。

## 8. 从姿态补偿到图像变换

### 8.1 纯旋转近似

对于远景和小幅抖动，可使用单应性矩阵：

```text
H = K * R_correction * inverse(K)
```

其中 `K` 是相机内参矩阵。对图像使用：

```python
stabilized = cv2.warpPerspective(
    frame,
    H,
    (frame.shape[1], frame.shape[0]),
    flags=cv2.INTER_LINEAR,
    borderMode=cv2.BORDER_REPLICATE,
)
```

### 8.2 畸变模型

如果镜头广角明显，应先使用去畸变映射，再进行防抖，或将防抖单应性合并到去畸变映射中。不要直接把强畸变图像当作理想针孔模型处理，否则画面边缘会出现更明显的拉伸。

### 8.3 有限视场角

防抖变换会产生黑边。常见处理方式：

- 预留 8% 到 15% 的裁剪边界
- 对变换矩阵增加适度缩放
- 使用 `BORDER_REPLICATE` 或边缘填充作为临时方案
- 当修正角度超过上限时降低补偿比例

建议优先使用裁剪加缩放，而不是依赖复制边界，因为复制边界在 VR 双目画面中容易被察觉。

## 9. 双目防抖的关键约束

双目不能把左右目当成两个独立单目画面随意处理，否则会造成视差变化、双眼不一致和观看疲劳。

### 9.1 同一时刻使用同一个全局旋转

左右目应使用同一个相机刚体姿态和同一个平滑目标姿态：

```text
left_stabilized  = warp(left,  H_left)
right_stabilized = warp(right, H_right)
```

`H_left` 和 `H_right` 可以由各自的内参计算，但旋转补偿来源必须相同。

### 9.2 不要独立估计左右目运动

左右目独立使用光流估计并分别平滑，可能导致：

- 左右画面水平视差不一致
- 垂直视差增加
- 物体边缘出现双影
- VR 观看产生眼疲劳

如果必须使用视觉辅助，应先估计双目共同的刚体运动，再将该运动应用到左右目。

### 9.3 左右目内参和外参

需要保存：

- 左目内参 `K_left`
- 右目内参 `K_right`
- 左右目畸变参数
- 左右目基线和相对旋转
- 相机与 IMU 的旋转外参

防抖变换不能破坏原始双目校正关系。处理后应使用棋盘格或双目标定板检查水平极线是否仍然对齐。

### 9.4 当前 SBS 输入的处理顺序

对于当前 `4000 x 1200` 左右拼接输入，推荐顺序：

```text
读取 SBS 原始帧
  -> 按原始宽度拆成 left/right
  -> 左右目分别去畸变或使用预计算 map
  -> 使用同一姿态补偿进行 warp
  -> 统一裁剪区域
  -> resize
  -> hconcat 或分别写入 WebRTC buffer
```

不要先把整张 SBS 图像作为一个普通单目图像进行旋转变换，因为左右目各自的主点和视场中心不同。

## 10. 滚动快门问题

若相机使用 rolling shutter，整帧只使用一个姿态会在快速运动时残留倾斜和果冻效应。

改进方式是按图像行时间进行姿态插值：

```text
每个图像行的曝光时刻
    -> 插值 IMU 姿态
    -> 计算该行对应的变换
    -> 分块或逐行重映射
```

第一版可先采用整帧姿态，因为它实现简单且适合低速抖动。确认系统存在明显 rolling shutter 伪影后，再增加行级补偿。

## 11. 建议的代码集成方式

### 11.1 新增独立稳定器模块

建议新增类似以下模块：

```text
teleop/teleimager/src/teleimager/imu_stabilizer.py
```

模块职责保持单一：

- 接收带时间戳的 IMU 样本
- 维护零偏和姿态
- 根据图像时间戳计算补偿矩阵
- 对单目或双目帧执行变换
- 输出诊断信息

不要把滤波器、相机采集和网络发布全部堆在 `image_server.py` 的一个函数中。

### 11.2 统一数据结构

可以使用以下数据结构表达时间戳数据：

```python
from dataclasses import dataclass
import numpy as np


@dataclass
class ImuSample:
    timestamp_s: float
    gyro_rad_s: np.ndarray
    accel_m_s2: np.ndarray | None = None


@dataclass
class ImageSample:
    timestamp_s: float
    frame_bgr: np.ndarray
```

Python 3.8 环境不支持 `np.ndarray | None` 语法时，应改用 `Optional[np.ndarray]`。

### 11.3 调整 `OpenCVCamera` 的处理位置

当前 `OpenCVCamera._update_frame()` 中大致流程为：

```text
读取最新帧
  -> stereo_sbs 或 stereo_fusion
  -> stereo_shift_x
  -> stereo_split_webrtc
  -> ZMQ/WebRTC 输出
```

建议调整为：

```text
读取带时间戳的最新帧
  -> 拆分左右目
  -> IMU 防抖
  -> stereo_sbs 或 stereo_fusion
  -> stereo_shift_x
  -> stereo_split_webrtc
  -> ZMQ/WebRTC 输出
```

如果当前 UVC 接口不能提供硬件图像时间戳，应在采集线程成功读取图像的第一时间记录时间，并明确这是近似时间戳。不要在编码完成后再记录时间。

### 11.4 低延迟策略

当前采集线程已经采用“只保留最新帧”的策略。防抖模块也应遵循相同原则：

- IMU 队列只保留覆盖当前图像时间的必要窗口
- 图像处理不能等待旧帧
- 处理过慢时丢弃旧图像，而不是增加队列长度
- WebRTC 继续使用单帧或极小队列
- 记录每阶段耗时和端到端延迟

## 12. RealSense 实施注意事项

如果使用带 IMU 的 RealSense 设备，建议在 `RealSenseCamera` 内部完成以下工作：

1. 启用 color、gyro、accel 流。
2. 使用 SDK 时间戳缓存 IMU 样本。
3. 读取一帧彩色图像时，取覆盖该图像时间的 IMU 样本区间。
4. 对该区间积分得到曝光时刻姿态。
5. 对彩色图像执行防抖。
6. 继续按现有接口写入 WebRTC 和 ZMQ buffer。

RealSense 的深度图若同时使用，也必须考虑防抖后的彩色图与深度图几何关系。只防抖彩色图而继续使用未变换的深度图，会造成彩色和深度不对齐。

## 13. 硬件辅助防抖方案

数字防抖不能消除运动模糊，也不能恢复曝光期间丢失的细节。若机械振动较强，应优先从硬件侧降低输入振动。

### 13.1 被动隔振

适合处理高频小振动：

- 橡胶减振柱
- 弹性隔振垫
- 阻尼支架
- 增加支架刚度
- 缩短相机悬臂
- 减少线缆拉扯

隔振结构不能过软，否则会产生低频摆动，反而增加 EIS 负担。

### 13.2 主动云台或音圈执行器

适合需要更大补偿角度或更高画质的场景：

- 两轴或三轴云台
- 微型音圈执行器
- 压电或 MEMS 光学防抖机构

控制链路应为：

```text
IMU -> 姿态/角速度控制器 -> 执行器 -> 相机姿态稳定
                         \-> EIS 残差补偿
```

硬件防抖负责大幅、低频或中频姿态变化，数字防抖负责高频残差和执行器误差。机械系统需要明确最大角度、带宽、延迟和失效安全状态。

### 13.3 曝光与快门设置

减小曝光时间通常比单纯提高 EIS 强度更能改善运动模糊，但会带来噪声增加。建议：

- 尽量使用较高帧率
- 避免自动曝光把曝光时间拉得过长
- 在允许的噪声范围内提高增益或补光
- 评估 LED 灯光下的频闪和 rolling shutter 条纹

## 14. 配置建议

建议在头部相机配置中增加独立的防抖配置段，而不是复用现有 stereo shift 参数：

```yaml
head_camera:
  enable_zmq: true
  enable_webrtc: true
  type: opencv
  capture_image_shape: [1200, 4000]
  image_shape: [720, 2400]
  binocular: true
  stereo_sbs: true
  stereo_split_webrtc: true
  fps: 30

  stabilization:
    enabled: false
    imu_source: none
    mode: gyro
    smoothing_alpha: 0.92
    max_correction_deg: 8.0
    crop_ratio: 0.12
    rolling_shutter: false
    calibration_file: null
```

推荐的 `imu_source` 值：

| 值 | 含义 |
| --- | --- |
| `none` | 禁用防抖 |
| `realsense` | 从 RealSense SDK 读取 IMU |
| `serial` | 从串口读取外部 IMU |
| `sdk` | 使用相机厂商 SDK |
| `file` | 读取离线记录，用于测试 |

首次上线时应将 `enabled` 默认设置为 `false`，通过配置逐步启用，确保 IMU 数据异常时不会阻塞图像服务。

## 15. 标定流程

### 15.1 相机内参

分别标定左右目：

- 焦距和主点
- 畸变参数
- 图像分辨率对应关系

如果使用相机驱动提供的内参，必须确认其对应当前分辨率、裁剪方式和 binning 设置。

### 15.2 双目外参

使用棋盘格或 Charuco 标定板获得：

- 左右目旋转
- 左右目平移
- 基线长度
- 极线校正参数

### 15.3 相机到 IMU 外参

至少需要相机和 IMU 之间的旋转外参：

```text
R_camera_imu
```

可通过厂商提供的标定参数、手眼标定或相机-IMU 联合标定获取。安装完成后不要随意改变 IMU 与相机的相对位置。

### 15.4 时间偏移

记录图像和 IMU 时间，估计固定偏移：

```text
t_image_corrected = t_image + time_offset
```

时间偏移误差会直接表现为防抖过补偿或欠补偿。快速旋转测试时，哪怕数毫秒偏差也可能可见。

## 16. 验证与验收指标

### 16.1 静态测试

相机固定在支架上，保持无运动：

- 画面不能自行漂移
- IMU 积分姿态不能持续快速旋转
- 陀螺仪零偏应稳定
- 左右目不能出现垂直漂移

### 16.2 单轴旋转测试

分别绕相机 X、Y、Z 轴缓慢和快速旋转：

- 验证轴方向是否正确
- 验证补偿方向是否正确
- 验证补偿角度是否正确
- 检查左右目视差是否保持稳定

### 16.3 高频振动测试

将相机安装到真实机器人头部，执行不同速度的头部动作：

- 低速转头
- 快速点头和摇头
- 行走或机械臂运动引起的结构振动
- 急停和碰撞保护动作

### 16.4 量化指标

建议记录以下指标：

| 指标 | 说明 |
| --- | --- |
| 稳定前后角点轨迹方差 | 衡量画面残余抖动 |
| 光流高频能量 | 衡量高频运动残差 |
| 左右目垂直视差 | 衡量双目几何一致性 |
| 黑边或裁剪比例 | 衡量视场损失 |
| 端到端延迟 | 采集到显示的总延迟 |
| CPU/GPU 占用 | 评估 PC2 实时性 |
| 丢帧率 | 评估采集和发布稳定性 |
| 运动模糊长度 | EIS 无法消除的曝光模糊 |

建议验收目标：

- 防抖增加延迟不超过 1 帧
- 正常运动下左右目垂直视差不明显增加
- 防抖后有效视场损失控制在 10% 到 15% 内
- 采集到发布链路不因 IMU 异常阻塞
- IMU 断开时自动降级为原始图像或视觉防抖

## 17. 故障处理与降级策略

防抖必须是可选模块，不能成为图像服务的单点故障。

建议的降级逻辑：

```text
IMU 正常 + 时间同步正常
    -> 使用 IMU EIS

IMU 暂时丢样本
    -> 使用最近可靠姿态，降低补偿权重

IMU 长时间不可用
    -> 关闭 EIS，输出原始图像

标定文件无效
    -> 关闭几何防抖，记录错误日志

处理耗时超过帧周期
    -> 丢弃旧帧，保持最新帧优先
```

不要在检测到单个异常 IMU 样本时直接停止整个 `teleimager` 服务。

## 18. 推荐实施顺序

1. 确认相机和 IMU 的硬件接口、时间戳和坐标系。
2. 记录原始图像、gyro、accel 和时间戳，暂不做防抖。
3. 实现 IMU 零偏估计和四元数积分。
4. 使用离线数据验证坐标轴、时间偏移和旋转方向。
5. 新增独立 `imu_stabilizer.py` 模块。
6. 在 `OpenCVCamera` 中先实现单目或左目离线防抖验证。
7. 扩展到左右目共同补偿，检查双目视差。
8. 接入实时 WebRTC/ZMQ 链路，测量 CPU、帧率和延迟。
9. 增加滚动快门补偿或视觉残差补偿。
10. 根据机械振动结果决定是否增加被动隔振或主动云台。

## 19. 最小可行版本

如果需要尽快验证效果，可以先实现以下最小版本：

```text
相机图像时间戳
  + 1 kHz 左右 gyro
  -> 启动静止校准
  -> 四元数积分
  -> 一阶低通获得平滑姿态
  -> 计算 H = K * R * K^-1
  -> 左右目分别 warpPerspective
  -> 裁剪 12%
  -> 继续现有 SBS/WebRTC/ZMQ 发布
```

该版本不处理平移、深度和 rolling shutter，但能够验证最核心的 IMU 到图像补偿链路。后续再逐步加入加速度计融合、视觉辅助和硬件稳定机构。

## 20. 结论

对当前项目，最实用的路线是：

1. 在 PC2 的 `teleimager` 采集端集成 IMU。
2. 先使用陀螺仪实现低延迟旋转 EIS。
3. 对左右目使用同一刚体姿态补偿，不独立平滑左右图像。
4. 使用真实硬件时间戳完成图像与 IMU 对齐。
5. 通过裁剪和缩放控制黑边，预留约 10% 到 15% 视场余量。
6. 对强振动和运动模糊，增加机械隔振、缩短曝光时间或使用主动云台。
7. 为 IMU 断开、时间戳异常和处理超时提供自动降级。

这样可以在不改变 `televuer` 显示协议的前提下，把防抖功能透明地加入现有图像服务链路。
