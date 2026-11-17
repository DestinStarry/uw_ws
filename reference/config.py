# -*- coding: utf-8 -*-
"""
机载电脑配置 — 所有可调参数集中在这里
"""

# ============================================================
# 网络
# ============================================================
TCP_HOST = "0.0.0.0"           # 监听所有网口
TCP_PORT = 8888                 # 与笔记本约定的端口

# ============================================================
# 串口 (机载电脑 UART5 <-> STM32 USART6)
# ============================================================
SERIAL_PORT = "/dev/ttyAS5"    # 机载电脑 UART5 设备路径
                                # OrangePi Zero 3: /dev/ttyAS5
                                # 树莓派: /dev/ttyAMA1
                                # Jetson: /dev/ttyTHS1 (视引脚而定)
SERIAL_BAUDRATE = 115200        # 波特率, 与 STM32 一致
SERIAL_BYTESIZE = 8
SERIAL_STOPBITS = 1
SERIAL_PARITY = "N"             # 无校验 (STM32 USART6: UART_PARITY_NONE)

# ============================================================
# 新 IMU (YbImu, 香橙派 USB 串口)
# ============================================================
YBUART_PORT = "/dev/ttyUSB0"     # 新 IMU 串口设备路径
IMU_OUTPUT_RATE = 100             # IMU硬件串口输出频率 (Hz)
IMU_ALGORITHM = 6                 # 先用六轴融合，避开推进器磁场干扰
IMU_READ_INTERVAL = 0.01          # 香橙派状态快照频率 (秒), 100Hz
IMU_REPORT_INTERVAL = 0.1         # 向Windows回传显示数据 (秒), 10Hz
IMU_STALE_TIMEOUT = 0.1           # 控制端允许的最大IMU数据年龄 (秒)
IMU_ZERO_CALIBRATION_TIME = 2.0    # 上电后保持机器人水平静止，用这段数据建立姿态零点

# 深度计实测约 2Hz，即每 500ms 才有一帧新测量，偶有较大拖尾。
# 连续丢失 3 个采样(1.5s)后标记失效；控制器不会因此自动关闭定深模式。
# 注意：STM32 在深度断流时不再发送 Depth 字段（只发其余信息），
# 所以这里的时间判定是唯一依据，不会把旧值误当成新采样。
DEPTH_STALE_TIMEOUT = 1.5
# 相邻两次深度采样允许的最短间隔。低于此值只可能是重复帧或时间戳
# 错乱，用它求速度会把误差放大（定深 PID 的 KD=12），必须丢弃该次。
DEPTH_RATE_MIN_DT = 0.3

# ============================================================
# 飞行日志（每次运行生成 CSV 到 orange/logs/）
# ============================================================
FLIGHT_LOG_ENABLED = True

# ============================================================
# OpenCV同步双目（单路UVC输出左右拼接帧）
# ============================================================
CAMERA_DEVICE = "/dev/video0"
CAMERA_FRAME_WIDTH = 1280         # 左右拼接后的整帧宽度
CAMERA_FRAME_HEIGHT = 480
CAMERA_FPS = 30
CAMERA_HTTP_PORT = 8080
CAMERA_JPEG_QUALITY = 90          # MJPEG编码质量；双目整幅像素多，80会明显发糊
CAMERA_SWAP_EYES = False          # 左右画面相反时改为True
