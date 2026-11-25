"""
极简手柄测试 - 所有错误都会打印
"""
import pygame
import time
import sys
import traceback

try:
    print("初始化 pygame...")
    pygame.init()
    
    print("创建窗口...")
    screen = pygame.display.set_mode((300, 100))
    pygame.display.set_caption("手柄测试")
    
    print("初始化 joystick 模块...")
    pygame.joystick.init()
    
    count = pygame.joystick.get_count()
    print(f"检测到手柄数量: {count}")
    
    if count == 0:
        print("没有检测到手柄!")
        pygame.quit()
        sys.exit(1)
    
    joy = pygame.joystick.Joystick(0)
    joy.init()
    
    name = joy.get_name()
    axes = joy.get_numaxes()
    buttons = joy.get_numbuttons()
    hats = joy.get_numhats()
    
    print(f"名称: {name}")
    print(f"轴数: {axes}")
    print(f"按键数: {buttons}")
    print(f"帽子(Hat)数: {hats}")
    print("开始读取数据 (移动摇杆/按按键观察变化)...")
    print("=" * 60)
    
    while True:
        pygame.event.pump()
        
        # 读取所有数据（Y 轴取反，上为正）
        vals = []
        for i in range(axes):
            val = joy.get_axis(i)
            if i == 1 or i == 3:  # Y 轴取反
                val = -val
            vals.append(f"A{i}={val:+.3f}")
        
        btn_vals = []
        for i in range(buttons):
            if joy.get_button(i):
                btn_vals.append(str(i))
        
        hat_vals = []
        for i in range(hats):
            h = joy.get_hat(i)
            if h != (0, 0):
                hat_vals.append(f"H{i}=({h[0]},{h[1]})")
        
        # 终端输出
        line = "  ".join(vals)
        if btn_vals:
            line += "  按键:[" + ",".join(btn_vals) + "]"
        if hat_vals:
            line += "  " + " ".join(hat_vals)
        
        sys.stdout.write(f"\r\033[K{line}")
        sys.stdout.flush()
        
        # 处理退出事件
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                print("\n窗口关闭，退出")
                pygame.quit()
                sys.exit(0)
        
        time.sleep(0.05)

except Exception as e:
    print(f"\n发生错误: {e}")
    traceback.print_exc()
    pygame.quit()
    sys.exit(1)
