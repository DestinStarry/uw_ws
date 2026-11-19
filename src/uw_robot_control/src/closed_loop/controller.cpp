/// @file controller.cpp
/// @brief 闭环实现：姿态级联、速度→力映射、定深 PID、限幅与无扰交接。
///
/// 控制周期 50 Hz；所有积分都有限幅与抗饱和，切模式时保持力矩连续；
/// 打杆轴走速率环（目标姿态不累积），松杆轴回到姿态保持。

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
  // 急停/失联/解锁等安全停机：逐轴手动状态与无扰交接基准一并清零。
  was_angular.fill(false);
  prev_moment.setZero();
  was_vertical = false;
  depth_was_valid = false;
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
