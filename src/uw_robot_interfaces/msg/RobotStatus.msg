# 控制节点 → 上位机/诊断：状态机、目标六维力与健康标志。
#
# 这里的 wrench 是**控制器下发的目标**，不是推进器实测输出；
# 实测/可实现值请看 AllocationStatus。
std_msgs/Header header
# 是否已解锁（armed）；解锁后收到 estop/超时/IMU 失效会自动上锁。
bool armed
# 当前是否闭环；与开环（手动直通）互斥。
bool closed_loop
# 是否处于定深模式。
bool depth_hold
# 控制器当前使用的定深目标，m，向下为正；仅 depth_hold=true 时生效。
float64 depth_target_m
bool imu_valid
bool depth_valid
# 最近一次控制周期耗时，s；用于判断控制是否跟得上 50 Hz。
float64 cycle_dt_s
# 机体系六维目标 [Fx,Fy,Fz,Mx,My,Mz]，单位 N、N·m。
float64[6] target_wrench
# 状态机名称：DISARMED / ATTITUDE_HOLD / DEPTH_HOLD / CMD_STALE / ESTOP ...
string state
