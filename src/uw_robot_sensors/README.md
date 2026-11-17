# 传感器包

本包提供 IMU 与双目相机，不处理手柄或执行器控制。

| 文件 | 作用 |
| --- | --- |
| `imu/driver.py` | SDK 初始化、真实收帧时间追踪、上电零点、线程安全快照 |
| `imu/imu_node.py` | 新鲜样本发布到 `imu/data`；`make_imu_message` 负责格式与单位转换 |
| `camera/capture.py` | 唯一 VideoCapture 所有者，采集/编码线程与最新帧缓存 |
| `camera/camera_node.py` | 参数、健康检查、按订阅需求发布左右目 Image |
| `camera/stream.py` | HTTP 健康状态、截图和 MJPEG 预览，不占有相机设备 |

上表的 Python 路径相对于 `uw_robot_sensors/` 模块目录。

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
