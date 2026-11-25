# Orin 实物部署与遥测

完整仓库同时保留 ROS 控制/传感器/通信、MuJoCo 仿真、电脑 UI 与 STM32 固件。ROS 构建会跳过 firmware/ 与 operator_ui/，分别按各自 README 构建运行。

## 已验证实物配置

| 设备 | 当前配置 |
|---|---|
| STM32 UART | /dev/ttyTHS2，115200 8N1 |
| USB IMU | /dev/ttyCH341USB0，100Hz |
| MS5837-30BA | I2C-7，固定地址 0x76，model=30ba，OSR4096 |
| 原双目相机 | 默认 /dev/video0，2560×720；HTTP 8080 /stereo.mjpg |
| 电脑控制 TCP | Orin 8888；电脑 config.py 设置实际 IP |

深度计实际复位/PROM 读写成功即可确认通信，不把 i2cdetect -r 是否显示 0x76 作为必要条件。传感器放空气中稳定约 3秒建立零点；水下上电需提供已知水面气压，不能把水下位置作为零深度。具体步骤见 MS5837直连Orin.md。

## 编译与启动

复制/clone 源码，保留设备驱动环境，不复制 Windows 或其它机器的 build/install/log。以下命令不执行系统或内核升级：

```bash
cd ~/uw_ws
python3 scripts/verify_tree.py
bash scripts/check.sh
source install/setup.bash
```

完整仿真、控制一致性检查仍可使用原 tools/check.sh，仿真参数和 simulation.launch.py 保留。

安全观察模式（不打开 MCU 串口）：

```bash
ros2 launch uw_robot_bringup robot.launch.py dry_run:=true imu:=true depth_source:=i2c camera:=true
```

实物 UART 联调用 dry_run:=false，这会启用执行器通信，首次检查先断开推进器动力；不要同时打开 CuteCom 或重复启动节点。

## 新增遥测

system_monitor_node 按 /sys/class/thermal/thermal_zone*/type 识别 CPU，读取 temp 除以 1000，默认 1Hz 发布 /system/cpu_temperature。读取失败发布 NaN，监测不是控制链必需节点，可用 monitor:=false 关闭。

hardware_node 将 MCU 原始行发到 /hardware/rx；TCP 桥严格提取 VOLT 与 CH0~CH3，并订阅 CPU 温度，默认每秒上传一次。非法或超过 3秒有效期上传 -- 与 VALID:0。CPU_TEMP 为 CPU 摄氏温度，TEMP 为兼容旧 UI 的同值别名，绝不转发 MCU 旧板温。转速和电压 AGE_MS 在桥接端表示接收/缓存年龄，不完全等同于 MCU 的原始采样年龄。

low_control3 常规 10Hz 行为 VOLT/CH0~CH3，诊断另以 1Hz DIAG 行发送。现有桥只提取测量字段，DIAG 不应刷新有效测量。CH_READY:0/SPI_ERRORS 增长及串口接收故障仍是待实物排查项，本次 Git 同步没有声称修复它们。

```bash
ros2 topic echo /system/cpu_temperature
ros2 topic echo /hardware/tx --full-length
ros2 topic echo /hardware/rx --full-length
python3 tools/telemetry_smoke.py
```

电脑 UI 标明 Orin CPU 温度，并对过期/无效数据显示 --；使用 operator_ui/Joystick_Over_Ethernet。下位机源码在 firmware/low_control3，保留零推力恢复、中位、看门狗、反馈失效保护；烧录步骤按其 README，不能把离线测试代替带动力验收。
