# 执行器目标；enabled=false 必须停机。
# theta12_rad/theta34_rad 沿用现有字段名，实际对应内部的 1/3 与 2/4 倾转组。
# 推力为 N，倾转为 rad；不是 PWM，也不是转速测量。
std_msgs/Header header
uint32 sequence
bool enabled
# Internal order: 1 top, 3 bottom, 2 right, 4 left. Units N.
float64[4] thrust_newtons
float64 theta12_rad
float64 theta34_rad
bool gripper_closed
