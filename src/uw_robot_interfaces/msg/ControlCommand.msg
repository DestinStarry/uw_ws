# 控制指令：发送端已经解算好的机体系六轴速度与离散命令。
# 本消息不含执行器输出；解锁、急停、超时和限幅仲裁仍在控制节点内完成。
# 机体轴按 ROS REP-103：+X 前进、+Y 左舷、+Z 向上（右手系）。
std_msgs/Header header
uint32 sequence

# 机体系六轴速度：
#   [u,v,w] 线速度 m/s：u 前进为正、v 左舷为正、w 上浮为正
#   [p,q,r] 角速度 rad/s：p 右滚为正、q 低头为正、r 逆时针（左转）为正
float64[6] velocity

# 离散命令，边沿由控制节点判定：
# arm 解锁/上锁切换；estop 急停（电平有效且优先）；depth_toggle 定深切换；
# loop_toggle 未解锁时切换开环/闭环。
bool arm
bool estop
bool depth_toggle
bool loop_toggle
