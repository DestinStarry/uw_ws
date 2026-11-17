# 机体系期望合力/力矩；center_servos 请求分配器将倾转目标回中。
# 新会话或 epoch 用来使先前的异步规划失效。
# Desired body-frame wrench. This message contains NO thruster commands.
std_msgs/Header header
uint32 sequence
# Session changes on controller restart; epoch changes on stop/rearm/mode switch.
uint64 session
uint64 epoch
bool enabled
bool closed_loop
bool center_servos
# 松杆快速回中阶段：直接发送零角度，暂停推力；阶段结束后允许闭环重新倾转。
bool recenter_servos
# [Fx,Fy,Fz,Mx,My,Mz], N and N*m
float64[6] wrench
