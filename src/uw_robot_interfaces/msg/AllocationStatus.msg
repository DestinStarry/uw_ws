# 分配诊断；六维顺序 [Fx,Fy,Fz,Mx,My,Mz]，单位 N、N·m。
# achievable_wrench 是门控降额后的目标；achieved_wrench 是模型计算值，非测量值。
std_msgs/Header header
bool planner_valid
float64 planner_ms
float64[6] achievable_wrench
float64[6] achieved_wrench
string state

# 控制器下发的原始六维目标（与 attainable/achieved 对比用）。
float64[6] target_wrench
# 四条推进器目标推力，顺序 [1上,3下,2右,4左]，N。
float64[4] thruster_forces
# 规划器给出的舵角目标与当前估计角度，顺序 [1/3组, 2/4组]，rad。
float64[2] servo_target_rad
float64[2] servo_estimated_rad
# 门控系数（0=不发力，1=全量）。
float64[2] gate
