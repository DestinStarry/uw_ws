#include "uw_robot_control/closed_loop/controller.hpp"
namespace uw {
void Controller::stop() {
  armed = false;
  depth_hold = false;
  ++epoch;
  integral.setZero();
  attitude_integral.setZero();
  filtered_gyro.setZero();
  rate_error_dot.setZero();
  previous_rate_error.setZero();
  rate_error_seen = false;
  target.setZero();
  depth_i = 0;
  centering = false;
  manual_was_active = false;
  recenter_until = -1;
  was_angular = false;
  was_vertical = false;
  depth_was_valid = false;
}

// 外环：姿态误差 -> 目标角速度；内环：角速度 PI + 指令前馈 -> 力矩。
// rate_ref 按 [横滚、俯仰、偏航] 排列，直接来自指令，单位 rad/s。
V3 Controller::attitude_moment(const V3 &rate_ref, const Imu &imu,
                               const Eigen::Quaterniond &q, double dt) {
  V3 manual = rate_ref;
  bool angular = (manual.cwiseAbs().array() > (cfg.rate_full_scale * .05).array()).any();
  // 松开角度指令时捕获当前姿态，并清除角速度积分。
  if (was_angular && !angular) {
    target_q = q;
    integral.setZero();
    attitude_integral.setZero();
  }
  was_angular = angular;
  for (int i = 0; i < 3; ++i) {
    const double alpha = i == 0 ? cfg.roll_gyro_filter_alpha : cfg.gyro_filter_alpha;
    filtered_gyro[i] += alpha * (imu.gyro[i] - filtered_gyro[i]);
  }
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
  // 角度误差积分：恒定扰动（浮心扶正、装配偏置）由它吸收，不依赖前馈常数。
  attitude_integral = bounded<V3>(attitude_integral + error * dt,
                                  -cfg.attitude_integral_limit, cfg.attitude_integral_limit);
  V3 feedback = cfg.attitude_kp.cwiseProduct(error) +
                cfg.attitude_ki.cwiseProduct(attitude_integral);
  feedback = feedback.cwiseMax(-cfg.inner_rate).cwiseMin(cfg.inner_rate);
  // 速率模式：有旋转指令的轴直接把指令角速度作为内环目标，不做姿态外环。
  // 三轴同理——目标姿态若继续往前积分，绕过 90° 后必然失稳（实测俯仰/横滚都会）。
  if (cfg.manual_rate_mode) {
    for (int i = 0; i < 3; ++i)
      if (std::abs(rate_ref[i]) > cfg.rate_full_scale[i] * .05)
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
  integral = bounded<V3>(integral + rate_error * dt, -.8, .8);
  if (!cfg.roll_hold)
    integral[0] = 0;
  if (!cfg.yaw_hold)
    integral[2] = 0;
  V3 moment = cfg.rate_kp.cwiseProduct(rate_error) + cfg.rate_ki.cwiseProduct(integral) +
              cfg.rate_ff.cwiseProduct(manual);
  moment = moment.cwiseMax(-cfg.moment).cwiseMin(cfg.moment);

  return moment;
}

// 深度向下为正；世界系竖直力转换到机体系后叠加到平移力。
// 松开升沉指令、或深度数据恢复有效时，重新捕获当前深度。
void Controller::apply_depth_hold(V3 &force, double w_ref, const Depth &depth,
                                  const Eigen::Quaterniond &q, bool depth_ok, double dt) {
  bool vertical_active = std::abs(w_ref) > .01;
  if (depth_hold && depth_ok) {
    if (!depth_was_valid || (was_vertical && !vertical_active)) {
      depth_target = depth.value;
      depth_i = 0;
    } else
      depth_target -= w_ref * cfg.depth_change * dt;
    double error_depth = depth_target - depth.value;
    depth_i = clamp(depth_i + error_depth * dt, -1., 1.);
    double world_fz =
        clamp(-(cfg.depth_kp * error_depth + cfg.depth_ki * depth_i - cfg.depth_kd * depth.rate),
              -cfg.depth_force, cfg.depth_force);
    force[2] = 0;
    force += q.conjugate() * V3(0, 0, world_fz);
  } else if (depth_hold)
    depth_i = 0;
  was_vertical = vertical_active;
  depth_was_valid = depth_ok;
}

WrenchTarget Controller::update(const Command &cmd, const Imu &imu, const Depth &depth,
                                double now, bool cmd_valid) {
  cycle_dt = last_time < 0 ? .02 : now - last_time;
  last_time = now;
  if (!std::isfinite(cycle_dt) || cycle_dt <= 0)
    cycle_dt = .02;
  double dt = clamp(cycle_dt, .005, .05);
  // 先记录按钮边沿，即使本周期停机，持续按住解锁也不会自动重新解锁。
  bool rise_arm = cmd.arm && !previous_arm, rise_depth = cmd.depth_toggle && !previous_depth;
  bool rise_loop = cmd.loop_toggle && !previous_loop;
  previous_arm = cmd.arm;
  previous_depth = cmd.depth_toggle;
  previous_loop = cmd.loop_toggle;
  // 急停电平有效且优先于解锁，包括两者同时按下的情况。
  if (!cmd_valid || !cmd.finite() || cmd.estop) {
    stop();
    state = cmd.estop ? "ESTOP" : "CMD_STALE";
    return {};
  }
  bool imu_ok = imu.valid && imu.q.coeffs().allFinite() && imu.gyro.allFinite() &&
                imu.q.norm() > .9 && imu.q.norm() < 1.1;
  Eigen::Quaterniond q = imu_ok ? imu.q.normalized() : Eigen::Quaterniond::Identity();
  if (rise_loop && !armed) {
    stop();
    closed_loop = !closed_loop;
  }
  if (rise_arm) {
    if (armed) {
      stop();
      state = "DISARMED";
      return {};
    }
    if (closed_loop && !imu_ok) {
      state = "WAIT_IMU";
      return {};
    }
    stop();
    armed = true;
    target_q = q;
  }
  if (!armed) {
    state = "DISARMED";
    return {};
  }
  if (closed_loop && !imu_ok) {
    stop();
    state = "IMU_STALE";
    return {};
  }
  bool depth_ok = depth.valid && std::isfinite(depth.value) && std::isfinite(depth.rate);
  if (rise_depth && closed_loop && depth_ok) {
    depth_hold = !depth_hold;
    depth_target = depth.value;
    depth_i = 0;
    depth_was_valid = true;
  }
  // 全部六轴指令归一立时，发起一次快速回中；不等待姿态水平或反馈力矩变小。
  // 新的手动输入立即结束回中；持续中立不会反复触发。
  const bool manual_active = !neutral(cmd.velocity);
  if (manual_was_active && !manual_active)
    recenter_until = now + cfg.release_center_time;
  if (manual_active)
    recenter_until = -1;
  manual_was_active = manual_active;
  bool recenter = now < recenter_until;
  V3 rate_ref(cmd.velocity[3], cmd.velocity[4], cmd.velocity[5]);
  V3 moment, force;
  const double w_ref = cmd.velocity[2];
  if (closed_loop) {
    moment = attitude_moment(rate_ref, imu, q, dt);
    // 静水扶正力矩前馈：系数 = 浮心偏移 × ρVg，来自整车物理参数（不是拟合值）。
    // 它抵消已知的恒定扶正力矩，剩余的不确定性由 attitude_ki 积分吸收。
    const V3 hydrostatic = cfg.hydrostatic_coeff.cross(q.conjugate() * V3::UnitZ());
    moment = (moment - hydrostatic).cwiseMax(-cfg.moment).cwiseMin(cfg.moment);
    force = cfg.damping.cwiseProduct(cmd.velocity.head<3>());
    apply_depth_hold(force, w_ref, depth, q, depth_ok, dt);
  } else {
    depth_hold = false;
    force = cfg.open_damping.cwiseProduct(cmd.velocity.head<3>());
    // 开环力矩按满杆量纲还原，保证与旧归一化摇杆指令等值。
    moment = cfg.open_moment.cwiseProduct(rate_ref.cwiseQuotient(cfg.rate_full_scale));
  }
  const double upright = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
  // 自动控制需要力矩/定深时，不能进入强制零推力回中窗口，否则会先跌落再接管。
  // 平稳中立仍保留直接回中；开环仍按原窗口停推；急停/失联在上方优先处理。
  if (cfg.prioritize_hold_on_release && closed_loop &&
      (depth_hold || upright < .99985 || moment.cwiseAbs().maxCoeff() > .02 ||
       imu.gyro.norm() > .03)) {
    recenter = false;
    recenter_until = -1;
  }
  // 力矩方向适配只在出口执行；实物和仿真的符号由参数决定。
  V6 raw;
  raw << force, cfg.moment_sign.cwiseProduct(moment);
  // 按各轴变化率平滑六维目标，单位分别为 N/s、N·m/s。
  V6 step = cfg.wrench_rate * dt;
  target += (raw - target).cwiseMax(-step).cwiseMin(step);
  // 回中采用迟滞：进入阈值保持原值，退出稍放宽，避免小反馈在边界反复切换。
  // 有手动输入、定深或较大姿态纠正时立即交回正常分配。
  const double upright_threshold = centering ? .992 : .996;
  const double moment_threshold = centering ? .05 : .02;
  centering = !depth_hold && neutral(cmd.velocity) && upright > upright_threshold &&
              (!closed_loop || moment.cwiseAbs().maxCoeff() < moment_threshold);
  const bool center = centering;
  state = closed_loop ? (depth_hold ? "DEPTH_HOLD" : "ATTITUDE_HOLD") : "OPEN_LOOP";
  return {target, armed, closed_loop, center || recenter, 0, epoch, recenter};
}
} // namespace uw
