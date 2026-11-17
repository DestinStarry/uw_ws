# 相对深度 m，向下为正；变化率 m/s。
# valid 与 data_age_s 共同描述新鲜度，发布时刻不等于采样时刻。
std_msgs/Header header
float64 depth_m
float64 depth_rate_mps
bool valid
float64 data_age_s
