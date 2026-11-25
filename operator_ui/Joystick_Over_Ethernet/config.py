"""
配置文件 - 网络参数、按键映射等
根据实际环境修改此处配置
"""

# ====== 网络配置 ======
# 机载电脑的 IP 地址
ONBOARD_IP = "192.168.1.10"
# 命令发送端口（TCP，与机载电脑约定）
CMD_PORT = 8888

# ====== 视频配置 ======
# Orin 原双目相机左右拼接整帧（默认 2560x720），UI 自适应缩放。
VIDEO_STREAM_URL = f"http://{ONBOARD_IP}:8080/stereo.mjpg"

# ====== 显示配置 ======
WINDOW_WIDTH = 1600
WINDOW_HEIGHT = 800
VIDEO_WIDTH = 1536
VIDEO_HEIGHT = round(((VIDEO_WIDTH - 16) // 2) * 9 / 16)  # 单目16:9，避免补黑边
FPS = 30

# ====== 摇杆死区 ======
# 摇杆在中心附近的小幅度抖动会被忽略
AXIS_DEADZONE = 0.05

# 超过此时间未收到ARMED运行状态，关闭连接并重连（秒）
STATUS_TIMEOUT = 2.0


# 左右镜头安装顺序相反时设为 True
VIDEO_SWAP_EYES = True


# ====== 指令成形：摇杆 -> 机体系六轴速度 ======
# 满杆量程；必须与控制端 robot.yaml 的 damping、rate_full_scale_rad_s 对应。
CMD_SURGE_FULL = 1.0       # u 满杆 m/s（满杆对应 30 N）
CMD_SWAY_FULL = 1.0        # v 满杆 m/s（满杆对应 10 N，约 0.24 m/s）
CMD_HEAVE_FULL = 1.0       # w 满杆（三个线轴统一 1.0；满杆对应 10 N）
CMD_ROLL_FULL_DEG = 25.0
CMD_PITCH_FULL_DEG = 25.0
CMD_YAW_FULL_DEG = 25.0
# 指令成形。默认与参考实现一致：不做输入滤波，只由控制端的 150 N/s 六维限幅兜底。
# 想要更软的手感，把 CMD_FILTER_ALPHA 调到 0.3~0.5，并把变化率限到 0.8/2.5。
CMD_FILTER_ALPHA = 1.0
CMD_RATE_LINEAR = 50.0     # m/s^2
CMD_RATE_ANGULAR = 100.0   # rad/s^2      
# 显示超时：仅影响 UI，不改变控制端看门狗或控制指令。
TELEMETRY_DISPLAY_TIMEOUT_S = 3.0
VIDEO_STALE_TIMEOUT_S = 2.0
