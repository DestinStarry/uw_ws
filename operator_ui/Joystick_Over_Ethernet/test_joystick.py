"""
手柄测试脚本 — 仅读取手柄并打印数据到终端
不需要 cv2 / 网络 / 视频
"""
import pygame
import time
import sys

# 死区阈值
AXIS_DEADZONE = 0.3

def apply_deadzone(value, threshold):
    return 0.0 if abs(value) < threshold else value

def main():
    pygame.init()
    # Windows 下需要显示窗口才能正常读取手柄
    screen = pygame.display.set_mode((400, 100))
    pygame.display.set_caption("手柄测试")
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("❌ 未检测到手柄，请连接手柄后重试")
        pygame.quit()
        return

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    name = joystick.get_name()
    num_axes = joystick.get_numaxes()
    num_buttons = joystick.get_numbuttons()
    print(f"✅ 手柄已连接: {name}")
    print(f"   轴数: {num_axes}")
    print(f"   按键数: {num_buttons}")
    print("-" * 70)
    print("所有轴原始数据 (移动摇杆观察变化)  Ctrl+C 退出:")
    print("-" * 70)

    servo_open = False
    prev_servo_btn = False
    last_print = ""

    try:
        while True:
            pygame.event.pump()  # 更新手柄状态

            # 读取所有轴原始值
            axes_raw = [joystick.get_axis(i) for i in range(num_axes)]

            # 根据轴数映射 (常见手柄布局)
            if num_axes >= 4:
                raw_v     = axes_raw[1]   # 左 Y
                raw_yaw   = axes_raw[0]   # 左 X
                raw_roll  = axes_raw[2]   # 右 X
                raw_pitch = axes_raw[3]   # 右 Y
            elif num_axes >= 2:
                raw_v     = axes_raw[1]
                raw_yaw   = axes_raw[0]
                raw_roll  = 0.0
                raw_pitch = 0.0
            else:
                raw_v = raw_yaw = raw_roll = raw_pitch = 0.0

            v     = apply_deadzone(raw_v,     AXIS_DEADZONE)
            yaw   = apply_deadzone(raw_yaw,   AXIS_DEADZONE)
            roll  = apply_deadzone(raw_roll,  AXIS_DEADZONE)
            pitch = apply_deadzone(raw_pitch, AXIS_DEADZONE)

            # 按键
            buttons = [joystick.get_button(i) for i in range(num_buttons)]
            # A 键 (Button 0) — 舵机 toggle
            servo_btn = bool(buttons[0]) if num_buttons > 0 else False
            if servo_btn and not prev_servo_btn:
                servo_open = not servo_open
            prev_servo_btn = servo_btn

            cmd_str = f"CTRL:{v:.3f},{yaw:.3f},{roll:.3f},{pitch:.3f},{1 if servo_open else 0}"

            # 所有轴显示
            axes_str = "  ".join([f"A{i}={axes_raw[i]:+6.3f}" for i in range(num_axes)])

            # 按键状态（只显示按下的键）
            btn_str = ""
            pressed = [str(i) for i in range(num_buttons) if buttons[i]]
            if pressed:
                btn_str = "  按键按下: [" + ",".join(pressed) + "]"

            status = (
                f"v={v:+6.3f} yaw={yaw:+6.3f} roll={roll:+6.3f} pitch={pitch:+6.3f} "
                f"舵机={'开' if servo_open else '关'} | {cmd_str}"
            )

            # 清空行并打印
            sys.stdout.write("\033[K")  # 清除当前行
            print(f"\r{status}", end="", flush=True)

            # 每 2 秒打印一行所有轴数据
            now = time.time()
            if int(now * 2) > int((now - 0.033) * 2):
                print(f"\n[调试] 轴: {axes_str}{btn_str}", end="", flush=True)

            time.sleep(0.033)  # ~30fps

    except KeyboardInterrupt:
        print("\n\n🛑 测试结束")

    pygame.quit()


if __name__ == "__main__":
    main()
