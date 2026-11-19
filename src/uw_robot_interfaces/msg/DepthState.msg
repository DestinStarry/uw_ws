# 深度传感器 → 控制/上位机：相对深度与新鲜度。
#
# 深度为相对值（向下为正），零点是水面标定值，不是绝对海拔。
# 发布时刻不等于采样时刻：valid 与 data_age_s 一起描述"这条数据还能不能用"，
# 消费端应同时检查两者（控制节点另有 1.5 s 超时兜底）。
std_msgs/Header header
# 相对深度，m，向下为正。
float64 depth_m
# 深度变化率，m/s，向下为正；一阶低通后的值。
float64 depth_rate_mps
# 本帧是否有效；无效帧（标定中/读取失败）仍会以低频发布，便于诊断。
bool valid
# 采样到发布的时延，s；用于跨节点判断新鲜度，不能用 header.stamp 代替。
float64 data_age_s
