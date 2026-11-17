# MuJoCo仿真节点

`uw_robot_mujoco`作为额外的仿真后端包，不改变三个业务包的职责。原始auv-v4目录未修改，assets复制进包内并随colcon安装。

```text
CMD TCP → control_node → allocation_node → mujoco_node
                 ↑                         ↓
                 └──── imu/data、depth/state ┘
```

## 启动

新终端：

```bash
source /opt/ros/humble/setup.bash
cd ~/uw_ws
source install/setup.bash
# 有WSLg桌面时打开MuJoCo交互窗口，合成双目同时可用
ros2 launch uw_robot_bringup simulation.launch.py headless:=false
# 无图形环境：只跑物理、合成双目和HTTP预览
ros2 launch uw_robot_bringup simulation.launch.py
```

交互窗口与合成双目共用同一进程的GLFW。相机渲染线程必须在viewer完成GLFW初始化之后再启动，
否则两者并发glfwInit会在WSLg下触发GLFW的X11断言并崩溃；mujoco_node.start_camera()已按此顺序调用。
只保留其中之一时，用camera:=false（仅窗口）或headless:=true（仅双目）。

默认TCP8888与现有Windows CMD发送端一致，可设置tcp_port:=18889或tcp:=false。
不要同时启动robot.launch.py：仿真入口不启动真实hardware_node、imu_node、相机，也不打开/dev/ttyAS5或/dev/ttyUSB0。
Ctrl+C退出整组节点；关键节点退出触发整组关闭。

## 映射与边界

- 订阅actuators/command，内部推力顺序[1上,3下,2右,4左]，单位N，按执行器gear=30转换MuJoCo控制量；两舵角单位rad。
- 发布imu/data、depth/state，以及simulation/pose。
- 深度从启动高度起算，向下为正；depth_rate=-世界Z速度。
- IMU是模型固定机体传感器，非真实SDK；模拟采样率、延迟和噪声，不模拟USB协议。
- 仿真launch对control_node单独覆盖moment_sign=[1,1,1]。原XML按右手坐标真实施力，不套用实物三轴反相；robot.yaml未修改。
- 模型椭球水动力、质量、惯量、净重力、舵机关节及原线性/角向水阻保持。原UDP接收器、PID和推力分配不导入，不与ROS2控制器重复运行。
- 命令超时、禁用或非有限/越界指令会停止输出；停输出不意味着艇体立即静止。
- 物理步长使用XML的0.002s。以墙钟推进，积压有上限；机器太慢时可能落后实时，不是硬实时系统。没有发布/clock，控制和消息采用ROS墙钟。
- 夹爪在原模型中没有机构，只忽略该字段。合成双目经HTTP预览发布，MuJoCo交互窗口不嵌入Windows UI。
- 默认模型随包安装，可用ros2 run uw_robot_mujoco mujoco_node --ros-args -p model_path:=绝对路径 -p headless:=false自定义。

## 依赖和验证

需要在ROS Python环境能import mujoco、numpy；本机实际安装的MuJoCo为3.11.0，原目录名3.3.7不表示当前Python库版本。
图形窗口需WSLg/OpenGL，headless测试不依赖显示环境。

```bash
colcon build --symlink-install --packages-select uw_robot_mujoco uw_robot_bringup
source install/setup.bash
colcon test --packages-select uw_robot_mujoco --event-handlers console_direct+
python3 tools/mujoco_integration.py
```

模型和测试均不代表已标定的水下真实动力学。实际艇体的推力曲线、舵机速度、浮心等仍需实机测量。

## 全浸没水下物理与被动扶正

当前使用真实重力 9.80665 m/s²，由 MuJoCo 对各部件施加；Python 只施加浮力和附加阻尼，
不会再以“缩小重力”代替浮力，也不直接修改姿态或角速度。
浮力为 rho*g*V，在固定机体浮心处施加；浮心偏移相对于初始整机重心。
力矩以 base 惯性质心为参考点，与 MuJoCo xfrc_applied 的世界系约定一致。

默认淡水密度 1000 kg/m³、动力黏度 0.001 Pa·s；V=0 表示自动取整机质量/密度以中性悬浮。
默认浮心在初始重心上方 0.02 m，仅为待实物标定的初值，不是测量值。
这些值可在 simulation.launch.py 的 sim 参数字典中调整：
gravity_mps2、water_density_kg_m3、water_viscosity_pa_s、displaced_volume_m3、
buoyancy_offset_m、water_current_mps（世界系恒定水流）。
转动阻尼及水动力椭球保留，水流同时用于 MuJoCo 流体模型和附加相对速度阻尼。
这些阻尼与流体参数是工程近似，需通过实物滑行、转动衰减实验标定。

本模型假设机器人完全浸没，没有水面穿越、部分浸没排水体积变化、波浪、推进器尾流、
空化或真实 CFD。中性浮力只平衡总竖直力，不消除浮心偏置产生的扶正力矩。
启动仿真后把机器人转到倾角，再按 B 停机，观察零推力下的自然回正。
闭环松杆仍锁定当前角度，可能主动抵抗扶正；验证被动恢复应按 B 或关闭姿态闭环。
