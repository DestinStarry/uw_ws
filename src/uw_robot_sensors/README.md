# 传感器包

本包提供 IMU、MS5837 深度计与双目相机，不处理手柄或执行器控制。

| 文件 | 作用 |
| --- | --- |
| `imu/driver.py` | SDK 初始化、真实收帧时间追踪、上电零点、线程安全快照 |
| `imu/imu_node.py` | 新鲜样本发布到 `imu/data`；`make_imu_message` 负责格式与单位转换 |
| `depth/ms5837.py` | 设备层：I2C 复位/读 PROM/CRC4/补偿运算；`compute` 等为纯函数 |
| `depth/driver.py` | 服务层：采集线程、零点标定状态机、失效判定、变化率低通 |
| `depth/depth_node.py` | ROS 层：发布 `depth/state`（与 STM32 版同构），含心跳帧 |
| `camera/capture.py` | 唯一 VideoCapture 所有者，采集/编码线程与最新帧缓存 |
| `camera/camera_node.py` | 参数、健康检查、按订阅需求发布左右目 Image |
| `camera/stream.py` | HTTP 健康状态、截图和 MJPEG 预览，不占有相机设备 |

上表的 Python 路径相对于 `uw_robot_sensors/` 模块目录。

## 深度

MS5837 经 I2C 直连主控，不再使用商家的 I2C→串口解算板。总线号随 Jetson 型号
与载板变化，先用 `i2cdetect -l` 列出、再用 `i2cdetect -y -r N` 寻到 `0x76`。
传感器为 3.3 V 器件，接 5 V 会损坏；上拉也只能上拉到 3.3 V。

`model: auto` 在启动时用同一组原始值按 02BA/30BA 各算一次，取更接近水面气压者：
错误型号会把压力算成约二十分之一，偏差上千 mbar，因此判别很稳。**前提是开机时
传感器在空气中**；若残差超过 50 mbar（例如已在水下），节点直接拒绝启动并要求
显式写 `02ba` / `30ba`，避免带着 10 倍误差下潜。30BA 最高支持 OSR 4096，
02BA 可到 8192。

`surface_pressure_mbar: 0` 表示上电后按 `zero_time_s` 采样取中位数自动标定水面零点。
天气变化带来的 ±10 mbar 相当于约 ±10 cm 深度，因此每次下水都应重新标定，而不是
沿用上一次的常数。

标定状态机：`ZEROING → READY`。若零点落在 700~1200 mbar 之外（相当于已在水下
超过约 1.9 m），重新标定；累计失败 `zero_attempts` 次后退回
`expected_surface_mbar` 并告警，保证仍能输出深度，而不是永久卡在标定。

失效处理与 IMU 同构：读失败不更新时间戳，`data_age_s` 持续增长，`valid` 置 false；
消费端 `control_node` 还会用 1.5 s 超时再判一次。

无效期间（标定中/读失败/未接传感器）仍以 `heartbeat_hz` 发布 `valid=false` 的
心跳帧，界面和日志能明确看到"深度计无数据"，而不是话题静默。

`depth/state` 只能有一个发布者：默认 `depth_node` 发布、`hardware_node.publish_depth`
为 false；`robot.launch.py` 的 `depth` 参数默认已改为 true。

深度变化率用一阶低通（α=0.2），与 `hardware_node` 的原有口径一致。

## IMU

`_open_imu()` 挂接 SDK 解析回调，按实际 raw/quaternion 帧时间判断新鲜度，
避免 getter 缓存掩盖 USB 断线。`imu_reader()` 保留原有 2 s 校准规则和四元数半球对齐。
坏样本在零点建立前重新开始连续校准；建立后保留零点，不因每次解锁而重建。
控制器的目标姿态捕获发生在控制包，与这个上电零点不同。

SDK 四元数顺序 wxyz，角速度 rad/s，加速度 g；ROS Imu 加速度转换为 m/s²。
样本使用主机实际接收时间推算 ROS 时间戳，重复或过期样本不重新发布。

## 双目

默认整幅 2560×720，每目 1280×720；请求采集 30 FPS，ROS 发布上限 15 Hz。
实际帧率取决于设备和负载，参数不是实际帧率保证。
`swap_eyes` 用于 ROS 单目和单目截图；整幅 HTTP JPEG 保持摄像头原始左右线序，
兼容已有 Windows UI 的左右目交换。图像时间戳为主机接收时刻，不是硬件曝光时刻。

采集和 JPEG 编码均只缓存最新帧，避免消费慢造成历史帧队列。
无订阅者时不序列化 ROS 原始 Image；HTTP 可独立读取最新 JPEG。
共享图像按只读使用，要绘制检测结果时先复制。

HTTP：`/?action=stream` 和 `/stereo.mjpg` 为整幅流；`/left.jpg`、`/right.jpg` 为单目截图；
`/snapshot.jpg`、`/stereo.jpg` 为整幅截图；`/health` 为状态。
截图无新鲜数据返回 503；采集失败会唤醒等待线程。
