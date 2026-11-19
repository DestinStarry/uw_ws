# 手柄/上位机 → 控制节点：机体系六轴速度指令与离散命令。
#
# 设计边界：发送端（Windows UI）已完成轴映射、死区与曲线整形，控制节点只负责
# 六轴速度跟踪、离散命令仲裁（边沿/电平）、解锁与超时保护。本消息不含任何
# 执行器输出，也不含摇杆原始轴值。
#
# 机体坐标按 ROS REP-103：+X 前进、+Y 左舷、+Z 向上（右手系）。
std_msgs/Header header
# 发送端自增序号；控制节点用于丢弃乱序帧（回绕时按差值判断）。
uint32 sequence

# 机体系六轴速度 [u, v, w, p, q, r]：
#   线速度 m/s：u 前进为正、v 左舷为正、w 上浮为正
#   角速度 rad/s：p 右滚为正、q 低头为正、r 逆时针（左转）为正
float64[6] velocity

# 离散命令。arm/depth_toggle/loop_toggle 按**边沿**生效，estop 为电平且优先级最高：
#   arm          解锁/上锁切换
#   estop        急停；为 true 时立刻停机，需重新解锁
#   depth_toggle 定深模式切换
#   loop_toggle  未解锁时切换开环/闭环
bool arm
bool estop
bool depth_toggle
bool loop_toggle
