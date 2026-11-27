"""
Joystick Over Ethernet — 主程序

架构：
  笔记本（本程序）         机载电脑
  ┌────────────────┐      ┌──────────────────┐
  │  pygame 读取手柄 │────▶│  TCP 发送控制指令  │
  │  TCP 发送控制量  │     │  处理并控制机器人   │
  │  接收视频流      │◀────│  MJPEG 视频流服务  │
  │  接收深度数据    │◀────│  TCP 回传 DEPTH    │
  └────────────────┘      └──────────────────┘
        通过网线连接

控制帧：CMD:u,v,w,p,q,r,arm,estop,depth,loop（约30Hz）。
左摇杆平移、扳机升沉、右摇杆俯仰/偏航、LB/RB横滚；A解锁、B急停、X定深、Y切换开闭环。
遥测与双目视频只影响显示，不参与本地控制决策。
"""
import sys
import math
import cv2
import numpy as np
import pygame
import threading
import time
from collections import deque

from config import *
from network import CommandClient
from command_mapper import CommandMapper
from camera_pan import CameraPan
from telemetry_display import TelemetryView


# ============================================================
# 全局状态
# ============================================================
# ============================================================
# Pygame 辅助
# ============================================================
class TextPrint:
    def __init__(self):
        self.reset()
        # Windows 支持中文的字体列表（按优先级）
        self.font = pygame.font.SysFont("microsoftyahei,simhei,dengxian,simsun,stkaiti", 22)

    def print(self, screen, text, color=(0, 0, 0)):
        bitmap = self.font.render(text, True, color)
        screen.blit(bitmap, [self.x, self.y])
        self.y += self.line_height

    def reset(self):
        self.x = 15
        self.y = 15
        self.line_height = 30

    def indent(self):
        self.x += 15

    def unindent(self):
        self.x -= 15


def format_number(value, decimals=3):
    """遥测可能缺失或无效；只格式化显示，不替代控制器数据。"""
    try:
        number = float(value)
    except (TypeError, ValueError):
        return "--"
    return f"{number:.{decimals}f}" if math.isfinite(number) else "--"


def draw_sensor_panel(screen, text_print, client, info_y):
    """All sensor values honor validity, source age and independent receive timeout."""
    values, ages = client.telemetry_snapshot()
    view = TelemetryView(values, ages, TELEMETRY_DISPLAY_TIMEOUT_S)
    text_print.x, text_print.y = 1086, info_y
    grey = (130, 140, 150)
    normal = (0, 0, 0)

    def row(label, value, unit=""):
        suffix = " " + unit if unit and value != "--" else ""
        text_print.print(screen, label + value + suffix, grey if value == "--" else normal)

    depth = view.number("DEPTH", 3, "DEPTH_VALID", "DEPTH_AGE_MS", timeout_s=1.5)
    row("[传感器] 深度: ", depth, "m")
    yaw, pitch, roll = (view.number(key, 1, "IMU_OK", timeout_s=1.0)
                        for key in ("YAW", "PITCH", "ROLL"))
    text_print.print(screen, f"  姿态: Y{yaw}° P{pitch}° R{roll}°",
                     grey if "--" in (yaw, pitch, roll) else normal)
    row("  电池电压: ", view.number("VOLT", 2, "VOLT_VALID", "VOLT_AGE_MS"), "V")
    row("  Orin CPU 温度: ", view.cpu_temperature(), "°C")
    text_print.print(screen, "  推进器转速 [RPM]")
    rpm = [view.rpm(i) for i in range(4)]
    for start in (0, 2):
        text_print.print(screen, f"  {start+1}号: {rpm[start]}    {start+2}号: {rpm[start+1]}",
                         grey if rpm[start] == rpm[start+1] == "--" else normal)
    text_print.print(screen, "  -- 表示未收到、无效或数据超时", grey)


def get_control_mode(client):
    """优先读取统一文本字段，同时兼容旧 ROS2 的布尔回传字段。"""
    mode = client.get("CONTROL_MODE")
    if mode in ("CLOSED", "OPEN"):
        return mode
    return {"1": "CLOSED", "0": "OPEN"}.get(client.get("CLOSED_LOOP"))


def apply_deadzone(value, threshold):
    """中心死区并重新映射剩余行程，消除起步跳变且保留满量程。"""
    magnitude = abs(value)
    if magnitude <= threshold:
        return 0.0
    return np.sign(value)*(magnitude-threshold)/(1.0-threshold)


# ============================================================
# 视频接收线程（手动解析 MJPEG 流，低延迟）
# ============================================================
class VideoReceiver(threading.Thread):
    def __init__(self, stream_url):
        super().__init__(daemon=True)
        self.stream_url = stream_url
        self.running = False
        self.latest_surface = None
        self._last_frame_at = None
        self._frame_times = deque(maxlen=300)
        self._lock = threading.Lock()

    def run(self):
        import urllib.request
        self.running = True

        try:
            stream = urllib.request.urlopen(self.stream_url, timeout=15)
            print(f"[视频] 已连接到视频流: {self.stream_url}")
        except Exception as e:
            print(f"[视频] 无法连接到视频流: {e}")
            return

        # 读取 boundary
        boundary = b""
        while self.running:
            line = stream.readline()
            if not line:
                break
            if line.startswith(b"--"):
                boundary = line.strip()
                break

        if not boundary:
            print("[视频] 无法解析 MJPEG boundary")
            stream.close()
            return

        while self.running:
            try:
                # 读头部 -> Content-Length
                content_length = 0
                while True:
                    line = stream.readline()
                    if not line:
                        raise ConnectionError("视频流已关闭")
                    if line == b"\r\n" or line == b"\n":
                        break
                    if line.lower().startswith(b"content-length:"):
                        content_length = int(line.split(b":")[1].strip())

                if content_length <= 0:
                    continue

                # 读取精确长度的 JPEG 数据
                jpeg_data = b""
                while len(jpeg_data) < content_length:
                    chunk = stream.read(content_length - len(jpeg_data))
                    if not chunk:
                        raise ConnectionError("流中断")
                    jpeg_data += chunk

                stream.readline()  # 跳过结尾 \r\n

                # 解码 + 缩放 + 旋转 + 翻转（在视频线程一次完成）
                arr = np.frombuffer(jpeg_data, dtype=np.uint8)
                img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if img is not None:
                    # 服务端必须返回左右拼接整帧；两目共享一次采集。
                    height, width = img.shape[:2]
                    if width % 2 or width < height * 1.8:
                        raise ValueError("需要左右拼接双目流，请检查 /stereo.mjpg 和相机输出格式")
                    middle = width // 2
                    eyes = [img[:, :middle], img[:, middle:]]
                    if VIDEO_SWAP_EYES:
                        eyes.reverse()
                    panel_width = (VIDEO_WIDTH - 16) // 2
                    panel_height = VIDEO_HEIGHT
                    canvas = np.full((VIDEO_HEIGHT, VIDEO_WIDTH, 3), 24, dtype=np.uint8)
                    for index, eye in enumerate(eyes):
                        eh, ew = eye.shape[:2]
                        scale = min(panel_width / ew, panel_height / eh)
                        dw, dh = max(1, round(ew * scale)), max(1, round(eh * scale))
                        resized = cv2.resize(eye, (dw, dh))
                        x = index * (panel_width + 16) + (panel_width - dw) // 2
                        y = (panel_height - dh) // 2
                        canvas[y:y+dh, x:x+dw] = resized
                    rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
                    surf = pygame.image.frombuffer(rgb.tobytes(),
                                                   (VIDEO_WIDTH, VIDEO_HEIGHT), 'RGB').copy()
                    with self._lock:
                        self.latest_surface = surf
                        self._last_frame_at = time.monotonic()
                        self._frame_times.append(self._last_frame_at)

            except Exception as e:
                print(f"[视频] 读取错误: {e}")
                break

        stream.close()
        print("[视频] 视频流已断开")

    def get_surface(self):
        with self._lock:
            if (self._last_frame_at is None or
                    time.monotonic() - self._last_frame_at > VIDEO_STALE_TIMEOUT_S):
                return None
            return self.latest_surface

    def get_fps(self):
        """最近一秒成功接收并解码的帧率；无新画面超过一秒则为零。"""
        now = time.monotonic()
        with self._lock:
            while self._frame_times and now - self._frame_times[0] > 1.0:
                self._frame_times.popleft()
            if len(self._frame_times) < 2:
                return 0.0
            elapsed = now - self._frame_times[0]
            return (len(self._frame_times) - 1) / elapsed if elapsed > 0 else 0.0

    def stop(self):
        self.running = False


# ============================================================
# 手柄读取 — 六轴与按键原始状态
# ============================================================
def read_controls(joystick):
    """返回六轴和按键原始状态；A解锁、B急停、X定深、Y切换开闭环。"""
    axis=lambda i: joystick.get_axis(i) if joystick.get_numaxes()>i else 0.
    button=lambda i: int(bool(joystick.get_button(i))) if joystick.get_numbuttons()>i else 0
    lx,ly,rx,ry=(apply_deadzone(axis(i),AXIS_DEADZONE) for i in range(4))
    lt=(axis(4)+1.)/2.; rt=(axis(5)+1.)/2.
    return lx,ly,rx,ry,lt,rt,button(0),button(1),button(2),button(3),button(4),button(5)


# ============================================================
# 主函数
# ============================================================
def main():
    # 1. 初始化 Pygame
    pygame.init()
    # 在固定逻辑画布绘制，再整体缩放至屏幕内，避免系统缩放导致窗口被裁掉。
    desktop_w, desktop_h = pygame.display.get_desktop_sizes()[0]
    initial_scale = min(1.0, (desktop_w - 80) / WINDOW_WIDTH,
                        (desktop_h - 100) / WINDOW_HEIGHT)
    display = pygame.display.set_mode(
        (max(640, int(WINDOW_WIDTH * initial_scale)),
         max(400, int(WINDOW_HEIGHT * initial_scale))), pygame.RESIZABLE)
    screen = pygame.Surface((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption("Joystick Over Ethernet - ROV 控制")
    clock = pygame.time.Clock()
    text_print = TextPrint()

    # 2. 初始化手柄（可选，没手柄也能打开）
    joystick = None
    pygame.joystick.init()
    if pygame.joystick.get_count() > 0:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"[手柄] 已连接: {joystick.get_name()}")
    else:
        print("[手柄] 未检测到手柄，仅显示视频画面")

    # 3. 连接机载电脑（仅尝试一次，不阻塞主循环）
    cmd_client = CommandClient(ONBOARD_IP, CMD_PORT, STATUS_TIMEOUT)
    cmd_client.connect()

    # 4. 启动视频接收线程
    video_receiver = VideoReceiver(VIDEO_STREAM_URL)
    video_receiver.start()

    # 5. 主循环
    mapper = CommandMapper()
    camera_pan = CameraPan(step_deg=10, limit_deg=90)
    last_cmd_time = time.monotonic()
    running = True
    while running:
        # ----- 事件处理 -----
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.JOYHATMOTION and event.hat == 0:
                camera_pan.update(event.value[1])

        now = time.monotonic()
        cmd_dt = min(max(now - last_cmd_time, 0.0), 0.2)
        last_cmd_time = now

        # ----- 读取手柄 -> 控制量 -----
        if joystick is not None:
            try:
                controls = read_controls(joystick)
            except pygame.error:
                controls = (0.0,)*6 + (0,)*6
        else:
            controls = (0.0,)*6 + (0,)*6
        lx,ly,rx,ry,lt,rt,a,b,x,y,lb,rb=controls
        try:
            hat_y = joystick.get_hat(0)[1] if joystick is not None and joystick.get_numhats() else 0
        except pygame.error:
            hat_y = 0
        camera_pan.update(hat_y)

        # ----- 发送控制指令（每帧发送，约 33ms/次 = 30FPS） -----
        # 手柄轴在这里换算成机体系六轴速度，控制端只跟踪物理量。
        velocity = mapper.update(lx, ly, rx, ry, lt, rt, lb, rb, cmd_dt)
        cmd_str = mapper.frame(velocity, a, b, x, y, camera_pan.angle_deg)
        # send() 会在断线时触发限频重连。重连成功后的下一帧会立即恢复
        # 发送，手柄无需重启程序。
        cmd_client.send(cmd_str)

        # ----- 终端输出控制信息（用于调试） -----
        depth = cmd_client.get("DEPTH")
        depth_str = f" 深度={depth}m" if depth is not None else ""
        print(f"\r[发送] {cmd_str}{depth_str}", end="", flush=True)

        # ----- 绘制画面 -----
        screen.fill((245, 247, 250))
        # 视频占上部，下方三栏排列状态、控制和传感器。
        info_y = VIDEO_HEIGHT + 90
        for column in range(3):
            card = pygame.Rect(20 + column * 526, info_y - 10, 508,
                               WINDOW_HEIGHT - info_y - 10)
            pygame.draw.rect(screen, (255, 255, 255), card, border_radius=10)
            pygame.draw.rect(screen, (218, 225, 232), card, 1, border_radius=10)
        text_print.reset()
        text_print.x, text_print.y = 34, info_y

        conn_status = cmd_client.connection_status
        text_print.print(screen, f"网络 [{conn_status}]",
                         (0, 140, 0) if cmd_client.connected else (190, 0, 0))
        text_print.print(screen, "")

        # 以香橙派控制器回传为准，界面不根据本地按键自行猜测模式。
        armed = cmd_client.get("ARMED")
        control_mode = get_control_mode(cmd_client)
        depth_hold = cmd_client.get("DEPTH_HOLD")
        depth_target = cmd_client.get("DEPTH_TARGET")
        imu_ok = cmd_client.get("IMU_OK")
        imu_zeroed = cmd_client.get("IMU_ZEROED")
        depth_valid = cmd_client.get("DEPTH_VALID")
        depth_age_ms = cmd_client.get("DEPTH_AGE_MS")
        status_color = lambda value: (0, 140, 0) if value == "1" else (190, 0, 0)
        status_text = lambda value, yes, no: yes if value == "1" else (no if value == "0" else "未知")
        text_print.print(screen, "[运行状态]")
        mode_text = ("姿态闭环" if control_mode == "CLOSED" else
                     ("六维开环" if control_mode == "OPEN" else "未知"))
        text_print.print(screen, "  模式: " + mode_text,
                         (0, 100, 190) if control_mode == "CLOSED" else (190, 110, 0))
        text_print.print(screen, "  控制: " + status_text(armed, "已解锁", "未解锁"),
                         status_color(armed))
        depth_mode_text = status_text(depth_hold, "已开启", "未开启")
        if depth_hold == "1" and depth_target is not None:
            depth_mode_text += f"  目标 {format_number(depth_target)} m"
        text_print.print(screen, "  定深: " + depth_mode_text,
                         status_color(depth_hold))
        text_print.print(screen, "  IMU: " + status_text(imu_ok, "正常", "未就绪/失联")
                         + "  零点: " + status_text(imu_zeroed, "已建立", "标定中"),
                         status_color(imu_ok))
        depth_age_text = f"  数据年龄: {depth_age_ms} ms" if depth_age_ms is not None else ""
        text_print.print(screen, "  深度计: " + status_text(depth_valid, "正常", "无效/失联")
                         + depth_age_text,
                         status_color(depth_valid))
        text_print.print(screen, "")

        # 控制量：下方中栏。
        text_print.x, text_print.y = 560, info_y
        text_print.print(screen, "[控制量]")
        text_print.print(screen, f"  前后={-ly:+5.3f}  横移={-lx:+5.3f}")
        text_print.print(screen, f"  偏航={rx:+5.3f}  俯仰={-ry:+5.3f}")
        text_print.print(screen, "  A 解锁/锁定    B 急停")
        text_print.print(screen, "  X 定深    Y 开环/闭环")
        text_print.print(screen, "  LB / RB 横滚")
        text_print.print(screen, f"  十字键↑/↓ 摄像头 ±10° 目标:{camera_pan.angle_deg:+.0f}°")
        text_print.print(screen, "")

        draw_sensor_panel(screen, text_print, cmd_client, info_y)

        # 视频和标签始终绘制，未收到画面时保留双目占位框。
        video_x = (WINDOW_WIDTH - VIDEO_WIDTH) // 2
        panel_width = (VIDEO_WIDTH - 16) // 2
        surf = video_receiver.get_surface()
        if surf is not None:
            screen.blit(surf, (video_x, 10))
        video_fps = video_receiver.get_fps()
        for index, label in enumerate(("左目", "右目")):
            panel = pygame.Rect(video_x + index * (panel_width + 16),
                                10, panel_width, VIDEO_HEIGHT)
            if surf is None:
                pygame.draw.rect(screen, (240, 243, 247), panel)
                wait_surface = text_print.font.render(
                    "等待" + label + "视频流...", True, (128, 128, 128))
                screen.blit(wait_surface, wait_surface.get_rect(center=panel.center))
            pygame.draw.rect(screen, (170, 180, 190), panel, 1)
            label_surface = text_print.font.render(
                f"{label}  {video_fps:.1f} FPS", True, (50, 50, 50))
            screen.blit(label_surface, label_surface.get_rect(
                midtop=(panel.centerx, panel.bottom + 10)))

        # ----- 刷新画面 -----
        # 整体等比缩放，窗口拖动大小后画面和文字均完整显示。
        display_w, display_h = display.get_size()
        scale = min(display_w / WINDOW_WIDTH, display_h / WINDOW_HEIGHT)
        render_size = (max(1, round(WINDOW_WIDTH * scale)),
                       max(1, round(WINDOW_HEIGHT * scale)))
        display.fill((235, 239, 244))
        display.blit(pygame.transform.smoothscale(screen, render_size),
                     ((display_w - render_size[0]) // 2,
                      (display_h - render_size[1]) // 2))
        pygame.display.flip()
        clock.tick(FPS)

    # ----- 清理资源 -----
    print("[退出] 正在关闭...")
    video_receiver.stop()
    video_receiver.join(timeout=2)
    cmd_client.send(mapper.frame((0.0,) * 6, 0, 1, 0, 0))
    cmd_client.close()
    pygame.quit()


if __name__ == "__main__":
    main()
