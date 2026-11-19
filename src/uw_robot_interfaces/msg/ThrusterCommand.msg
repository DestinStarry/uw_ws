# 分配器 → 执行器：四条推进器推力与两组舵角目标。
#
# enabled=false 表示必须停机（控制节点停机、急停、超时都会走这里）。
# 推力单位 N，舵角单位 rad；这些是**目标值**，不是 PWM，也不是转速或角度测量。
std_msgs/Header header
uint32 sequence
bool enabled
# 四条推进器目标推力，内部顺序 [1 上, 3 下, 2 右, 4 左]，单位 N。
float64[4] thrust_newtons
# 两组舵角目标，顺序 [1/3 组, 2/4 组]，单位 rad。
# 字段名沿用历史命名（theta12/theta34），实际含义是"1/3 倾转组"与"2/4 倾转组"。
float64 theta12_rad
float64 theta34_rad
bool gripper_closed
