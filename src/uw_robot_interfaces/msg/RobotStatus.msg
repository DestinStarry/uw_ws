# 控制器状态与六维目标；周期单位 s，目标单位 N、N·m。
# 这些状态不代表已经测得推进器真实输出。
std_msgs/Header header
bool armed
bool closed_loop
bool depth_hold
# 控制器当前实际使用的定深目标，m、向下为正；仅 depth_hold=true 时生效。
float64 depth_target_m
bool imu_valid
bool depth_valid
float64 cycle_dt_s
float64[6] target_wrench
string state
