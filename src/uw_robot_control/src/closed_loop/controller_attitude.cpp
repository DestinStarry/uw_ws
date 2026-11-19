/// @file controller_attitude.cpp
/// @brief 姿态/角速度级联与无扰交接（Controller::attitude_moment）。
///
/// 由 controller.cpp 拆出：外环 P+I、速率环 PID、逐轴打杆/松杆判定、
/// 积分抗饱和与切换瞬间的力矩连续，都集中在这一个函数里，单独成文件
/// 便于阅读与修改。

#include "uw_robot_control/closed_loop/controller.hpp"
namespace uw {
// 外环：姿态误差 -> 目标角速度；内环：角速度 PI + 误差微分 -> 力矩。
// rate_ref 按 [横滚、俯仰、偏航] 排列，直接来自指令，单位 rad/s。
// 手动打杆/松杆按三轴独立判定：某轴松杆时只交接该轴，其它仍在操作的轴不受影响。
V3 Controller::attitude_moment(const V3 &rate_ref, const Imu &imu,
                               const Eigen::Quaterniond &q, double dt) {
  V3 manual = rate_ref;
  // 逐轴打杆判据：本轴指令超过满杆的 5% 才算有效操作。
  std::array<bool, 3> angular{false, false, false};
  for (int i = 0; i < 3; ++i)
    angular[i] = std::abs(manual[i]) > cfg.rate_full_scale[i] * .05;

  // 逐轴松杆边沿。姿态目标用四元数整体保持，任一轴从有指令变为中立都要重新捕获
  // 当前姿态，避免目标姿态停留在松杆前积分出的位置上；但积分交接只针对松杆轴。
  std::array<bool, 3> release{false, false, false};
  bool target_needs_capture = false;
  for (int i = 0; i < 3; ++i) {
    release[i] = was_angular[i] && !angular[i];
    target_needs_capture = target_needs_capture || release[i];
  }
  if (target_needs_capture)
    target_q = q;
  was_angular = angular;

  for (int i = 0; i < 3; ++i) {
    const double alpha = i == 0 ? cfg.roll_gyro_filter_alpha : cfg.gyro_filter_alpha;
    filtered_gyro[i] += alpha * (imu.gyro[i] - filtered_gyro[i]);
  }
  // 目标姿态按三轴手动角速度积分：打杆轴由内环直接跟踪 rate_ref，不依赖目标姿态；
  // 中立轴的目标姿态则随载体转动一起走（body frame 耦合），这是姿态保持的原有语义。
  Eigen::Quaterniond dq(1, .5 * manual[0] * dt, .5 * manual[1] * dt, .5 * manual[2] * dt);
  target_q = (target_q * dq.normalized()).normalized();
  Eigen::Quaterniond qe = q.conjugate() * target_q;
  // q 与 -q 表示相同姿态；统一符号，选择短路径误差。
  if (qe.w() < 0)
    qe.coeffs() *= -1;
  V3 error = 2 * qe.vec();
  for (int i = 0; i < 3; ++i) {
    if (cfg.smooth_deadband || (i == 0 && cfg.roll_smooth_deadband))
      error[i] =
          std::copysign(std::max(0., std::abs(error[i]) - cfg.attitude_deadzone[i]), error[i]);
    else if (std::abs(error[i]) < cfg.attitude_deadzone[i])
      error[i] = 0;
  }
  if (!cfg.roll_hold)
    error[0] = 0;
  if (!cfg.yaw_hold)
    error[2] = 0;

  // 姿态外环积分单独管理：只积分未打杆轴的姿态误差，限幅 + 抗饱和（误差反向时
  // 允许先退出饱和，同向时冻结）。松杆不再清零，保留已学得的恒定扰动补偿。
  for (int i = 0; i < 3; ++i) {
    if (angular[i])
      continue;
    const double candidate = clamp(attitude_integral[i] + error[i] * dt,
                                   -cfg.attitude_integral_limit, cfg.attitude_integral_limit);
    const double unsat = cfg.attitude_kp[i] * error[i] + cfg.attitude_ki[i] * candidate;
    const bool pushing_out = (unsat > cfg.inner_rate[i] && error[i] > 0) ||
                             (unsat < -cfg.inner_rate[i] && error[i] < 0);
    if (!pushing_out)
      attitude_integral[i] = candidate;
  }
  V3 feedback = cfg.attitude_kp.cwiseProduct(error) +
                cfg.attitude_ki.cwiseProduct(attitude_integral);
  feedback = feedback.cwiseMax(-cfg.inner_rate).cwiseMin(cfg.inner_rate);
  // 速率模式：有旋转指令的轴直接把指令角速度作为内环目标，不做姿态外环。
  if (cfg.manual_rate_mode) {
    for (int i = 0; i < 3; ++i)
      if (angular[i])
        feedback[i] = rate_ref[i];
  }
  V3 rate_error = feedback - filtered_gyro;
  for (int i = 0; i < 3; ++i) {
    if (cfg.smooth_deadband || (i == 0 && cfg.roll_smooth_deadband))
      rate_error[i] = std::copysign(std::max(0., std::abs(rate_error[i]) - cfg.rate_deadzone[i]),
                                    rate_error[i]);
    else if (std::abs(rate_error[i]) < cfg.rate_deadzone[i])
      rate_error[i] = 0;
  }

  // 无扰交接：保存积分更新前的状态；"切换前的实际力矩"就是上一周期已限幅输出
  // prev_moment。松杆轴反算积分初值，使本周期力矩与切换前连续；其余轴正常积分。
  const V3 integral_pre = integral;
  integral = bounded<V3>(integral + rate_error * dt, -cfg.rate_integral_limit,
                         cfg.rate_integral_limit);
  for (int i = 0; i < 3; ++i) {
    if (!release[i] || cfg.rate_ki[i] <= 1e-12)
      continue;
    // prev_moment = kp*e + ki*I_new + kd*d  =>  I_new = (prev_moment - kp*e - kd*d)/ki
    const double keep = (prev_moment[i] - cfg.rate_kp[i] * rate_error[i] -
                         cfg.rate_kd[i] * rate_error_dot[i]) /
                        cfg.rate_ki[i];
    integral[i] = clamp(keep, -cfg.rate_integral_limit, cfg.rate_integral_limit);
    // 交接后重建该轴微分历史：本周期不把目标跳变当成角加速度尖峰。
    previous_rate_error[i] = rate_error[i];
    rate_error_dot[i] = 0;
  }
  if (!cfg.roll_hold)
    integral[0] = 0;
  if (!cfg.yaw_hold)
    integral[2] = 0;
  // 内环抗饱和：常规积分（非交接轴）若把力矩继续往限幅外推则冻结该轴积分；
  // 误差反向时照常积分以便及时退出饱和。交接轴保留反算值维持力矩连续性。
  for (int i = 0; i < 3; ++i) {
    if (release[i] || cfg.rate_ki[i] <= 1e-12)
      continue;
    const double unsat = cfg.rate_kp[i] * rate_error[i] + cfg.rate_ki[i] * integral[i] +
                         cfg.rate_kd[i] * rate_error_dot[i];
    const bool pushing_out = (unsat > cfg.moment[i] && rate_error[i] > 0) ||
                             (unsat < -cfg.moment[i] && rate_error[i] < 0);
    if (pushing_out)
      integral[i] = integral_pre[i];
  }
  // 微分项：对死区后的角速度误差求导并低通，给速率环加相位超前。
  // 首个周期不计算，避免解锁瞬间的微分冲击。
  if (cfg.rate_kd.squaredNorm() > 0) {
    if (rate_error_seen) {
      for (int i = 0; i < 3; ++i) {
        if (release[i])
          continue; // 交接轴本周期刚重置差分起点
        const double raw = (rate_error[i] - previous_rate_error[i]) / dt;
        rate_error_dot[i] += cfg.rate_kd_filter * (raw - rate_error_dot[i]);
      }
    }
    previous_rate_error = rate_error;
    rate_error_seen = true;
  }
  // 手柄前馈已移除：力矩 = kp*误差 + ki*积分 + kd*误差导数。
  V3 moment = cfg.rate_kp.cwiseProduct(rate_error) + cfg.rate_ki.cwiseProduct(integral) +
              cfg.rate_kd.cwiseProduct(rate_error_dot);
  moment = moment.cwiseMax(-cfg.moment).cwiseMin(cfg.moment);
  prev_moment = moment;

  return moment;
}

} // namespace uw
