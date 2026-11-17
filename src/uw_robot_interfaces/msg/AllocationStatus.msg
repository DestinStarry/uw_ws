# 分配诊断；六维顺序 [Fx,Fy,Fz,Mx,My,Mz]，单位 N、N·m。
# achievable_wrench 是门控降额后的目标；achieved_wrench 是模型计算值，非测量值。
std_msgs/Header header
bool planner_valid
float64 planner_ms
float64[6] achievable_wrench
float64[6] achieved_wrench
string state
