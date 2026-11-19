# 控制节点 → 分配器：机体系期望合力/力矩与会话控制。
#
# 本消息只描述"要多少力"，不包含任何推进器指令；分配器负责求解执行器。
# session/epoch 用来让过期的异步规划失效（控制重启、停机、重新解锁、模式切换）。
std_msgs/Header header
uint32 sequence
# 控制节点每次启动/重连时递增；分配器据此丢弃上一会话的规划结果。
uint64 session
# 停机、重新解锁、模式切换时递增；分配器据此使旧舵角规划失效。
uint64 epoch
# false 表示停机；分配器应输出零推力并请求回中。
bool enabled
# 闭环/开环模式，仅用于诊断与门控策略。
bool closed_loop
# 请求分配器把舵角回中（松杆或停机时）。
bool center_servos
# 松杆快速回中阶段：直接发零角度并暂停推力；阶段结束后允许闭环重新倾转。
bool recenter_servos
# 机体系六维力/力矩 [Fx,Fy,Fz,Mx,My,Mz]，单位 N、N·m。
float64[6] wrench
