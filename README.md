# uw_ws：按控制、传感器、通信组织的ROS2工程

核心 ROS 工程包含 **3个业务包＋2个支撑包**，另保留 MuJoCo 仿真包和既有示例包。仓库同时收录电脑 UI 和 STM32 下位机源码。

| 包 | 节点/内容 | 边界 |
|---|---|---|
| `uw_robot_control` | `control_node`、`allocation_node` | 从输入和状态得到目标力矩，再分配推进器与舵角；不打开串口、TCP或摄像头 |
| `uw_robot_sensors` | `imu_node`、`depth_node`、`camera_node`、`system_monitor_node` | 读取IMU和OpenCV双目，提供带时间信息的传感器数据及HTTP预览 |
| `uw_robot_communication` | `tcp_bridge_node`、`hardware_node` | 手柄TCP、STM32串口、协议转换和通信超时；不做姿态PID或推力分配 |
| `uw_robot_interfaces` | 消息和共用数据类型 | 只定义接口，不实现算法和驱动 |
| `uw_robot_bringup` | launch、robot.yaml | 统一装配节点和参数，不实现业务逻辑 |

```mermaid
flowchart LR
  UI[Windows UI] -->|TCP CMD 六轴速度| TCP[communication / tcp_bridge_node]
  TCP -->|control/cmd| C[control / control_node]
  IMU[sensors / imu_node] -->|imu/data| C
  DEPTH[sensors / depth_node] -->|depth/state| C
  C -->|control/wrench| A[control / allocation_node]
  A -->|actuators/command| HW
  HW -->|UART CTRL/OFF| STM32[STM32]
  CAM[sensors / camera_node] -->|HTTP双目视频| UI
  CAM -->|左右ROS图像| VISION[后续识别/测距]
```

先读 [直接运行](docs/直接运行.md)，再读 [结构和接口](docs/结构和接口.md)。
相机细节见 [OpenCV双目相机](docs/OpenCV双目相机.md)。

此次只调整包、目录、依赖、工具和文档；控制参数、算法公式、话题名、节点名、可执行文件名和STM32协议保持原样。
旧8包源码及构建产物移到 `~/uw_ws_backups/before_business_grouping_*`，避免旧install残留让系统继续找到已合并的包。
历史说明保存在 `docs/history/before_package_grouping/`，不是当前启动指南。

已有的UI遥测字段不完整、WSL跨进程图像DDS异常、香橙派算力和水下闭环稳定性仍需分别验收；合包不会自动解决这些问题。

## 当前实物与源码目录（2026-11-25 同步）

| 目录 | 内容 |
|---|---|
| src/ | ROS 实物与仿真包；保留原控制/分配算法和仿真对象 |
| operator_ui/Joystick_Over_Ethernet/ | 电脑 UI；CPU 温度、有效性/超时显示、原双目视频 |
| firmware/low_control3/ | STM32F407 执行层；中位与超时保护、转速重同步、精简遥测 |
| tools/YbImuLib/ | IMU SDK 源码，COLCON_IGNORE 避免当作 ROS 包构建 |

实物配置：STM32 /dev/ttyTHS2，IMU /dev/ttyCH341USB0，深度计 I2C-7 / 0x76 / 30ba，原双目 2560×720。你原有 robot.yaml 控制参数与配置注释予以保留。

详细启动与遥测见 [Orin 部署](docs/Orin部署.md)，本次提交范围与验证见 [同步说明](docs/实物同步与提交说明.md)。UI 和 MCU 是本次从仓库外复制的当前源码快照，后续建议在仓库内对应目录维护；如果仍修改外部原路径，要同步后再提交。

## 指令接口

手柄轴映射、死区、滤波和量程换算都在**发送端**完成，发送端直接下发机体系六轴速度：

- TCP 帧：`CMD:u,v,w,p,q,r,arm,estop,depth,loop\n`（10 个字段）。
- ROS 话题：`control/cmd`，消息 `uw_robot_interfaces/ControlCommand`。
- 满杆量程：`u = v = w = 1.0`、`p/q/r = 25/25/25 °/s`（三个线轴同一标尺）。
- 满杆推力：`u = 30 N`、`v = 10 N`、`w = 10 N`，故 `damping = 30/10/10 N/(m/s)`；
  定深目标变化率由 `depth_change_mps = 0.25` 单独给出（同参考实现）。
- 注意：闭环平移增益是手感调校值，不再等于参考实现的 `MAX_BODY_FORCE(20/6/10)`；姿态与角速度环仍与参考实现逐项一致。
- 控制节点只做六轴速度跟踪、解锁/急停/超时仲裁和最终限幅，不再解析摇杆轴。

## MuJoCo仿真后端

新增第6个包`uw_robot_mujoco`，替代实物串口和IMU接入同一套ROS控制链。见[MuJoCo仿真](docs/MuJoCo仿真.md)。原3业务包＋2支撑包结构保持，仿真包作为可选后端。

实物迁移：当前以 docs/实物迁移与标定.md 为准；两种后端共用 robot.yaml，模拟对象参数位于 plant.yaml。
