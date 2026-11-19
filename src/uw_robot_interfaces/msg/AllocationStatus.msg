# 分配器 → 诊断：目标、可实现、模型实际与执行器状态。
#
# 六维顺序统一为 [Fx,Fy,Fz,Mx,My,Mz]，单位 N、N·m。
# achievable 是门控降额后真正参与求解的目标；achieved 是**模型正算值**，不是测量值。
std_msgs/Header header
bool planner_valid
float64 planner_ms
# 门控降额后参与分配的目标。
float64[6] achievable_wrench
# 按当前舵角与推力模型正算得到的六维结果（非测量）。
float64[6] achieved_wrench
string state

# 控制器下发的原始六维目标，用于与 achievable/achieved 对比。
float64[6] target_wrench
# 四条推进器目标推力，内部顺序 [1 上, 3 下, 2 右, 4 左]，单位 N。
float64[4] thruster_forces
# 规划器给出的舵角目标与当前估计角度，顺序 [1/3 组, 2/4 组]，rad。
float64[2] servo_target_rad
float64[2] servo_estimated_rad
# 门控系数：0=不发力，1=全量；用于诊断降额发生在哪一组。
float64[2] gate
