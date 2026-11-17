#include "uw_robot_control/allocation/allocation_controller.hpp"
namespace uw {
void AllocationController::stop() {
  ++epoch;
  planner_valid = false;
  planner_ms = 0;
  forces.setZero();
  solver_angles.setZero();
  estimated.setZero();
  gate.setOnes();
  target.setZero();
  attainable.setZero();
  achieved.setZero();
  previous_wrench.setZero();
  pending.reset();
  last_plan = -1e9;
  last_valid_plan = -1e9;
  gate_until = 0;
  was_centering = false;
  returning_to_center = false;
  state = "DISABLED";
}
std::optional<Request> AllocationController::take_request() {
  auto out = pending;
  pending.reset();
  return out;
}

// 规划结果只属于创建它的 epoch；停机/来源切换后，旧结果不可复用。
// 还要同时检查时间和目标差异，避免采用已经过时的舵机角度。
void AllocationController::update_plan(bool center, double now, const std::optional<Plan> &result) {
  if (center) {
    solver_angles.setZero();
    planner_valid = false;
    // 只在进入回中时使旧规划失效，避免每个周期重置规划代次。
    if (!was_centering)
      ++epoch;
    returning_to_center = true;
    pending.reset();
    last_plan = -1e9;
  } else {
    if (result && result->valid && result->epoch == epoch &&
        now - result->started <= cfg.plan_max_age && now >= result->started) {
      V6 scale;
      scale << 10, 10, 10, 2, 2, 2;
      if (((target - result->target).cwiseAbs().cwiseQuotient(scale)).maxCoeff() <= 1.5) {
        solver_angles = bounded<V2>(result->angles, -servo_limit, servo_limit);
        planner_valid = true;
        returning_to_center = false;
        planner_ms = result->ms;
        last_valid_plan = result->started;
      }
    }
    if (now - last_plan >= cfg.plan_period) {
      pending = Request{target, estimated, forces, now, epoch};
      last_plan = now;
    }
  }
  was_centering = center;
  if (now - last_valid_plan > cfg.plan_max_age)
    planner_valid = false;
}

// 闭环直接放行；开环保留原有的目标跳变等待和角度接近门控。
// previous_wrench 仍逐周期更新，保持原有模式切换行为。
void AllocationController::update_gate(bool closed_loop, double now) {
  double jump = (target - previous_wrench).cwiseAbs().maxCoeff();
  previous_wrench = target;
  if (closed_loop)
    gate.setOnes();
  else {
    if (!planner_valid || jump > 1)
      gate_until = now + 1.;
    if (now < gate_until)
      gate.setZero();
    else {
      V2 approach = bounded<V2>(
          V2::Ones() - (estimated - solver_angles).cwiseAbs() / (15 * pi / 180), 0., 1.);
      for (int i = 0; i < 2; ++i)
        gate[i] = approach[i] < gate[i] ? approach[i] : gate[i] + .1 * (approach[i] - gate[i]);
    }
  }
}

Actuators AllocationController::update(const WrenchTarget &command, double now, bool input_valid,
                                       const std::optional<Plan> &result) {
  cycle_dt = last_time < 0 ? .02 : now - last_time;
  last_time = now;
  if (!std::isfinite(cycle_dt) || cycle_dt <= 0)
    cycle_dt = .02;
  const double dt = clamp(cycle_dt, .005, .05);
  if (!input_valid || !command.enabled || !command.wrench.allFinite()) {
    stop();
    state = input_valid ? "DISABLED" : "WRENCH_STALE";
    return {};
  }
  if (!source_seen || source_session != command.session || source_epoch != command.epoch) {
    stop();
    source_seen = true;
    source_session = command.session;
    source_epoch = command.epoch;
  }
  target = command.wrench;
  const bool center = command.center || command.recenter, closed_loop = command.closed_loop;
  update_plan(center, now, result);
  if (command.recenter) {
    // 明确的松杆回中命令优先于规划与普通角度限速，机械执行仍由舵机本身完成。
    // 零推力避免快速转向时沿旧方向继续发力；旧规划已在 update_plan 中失效。
    // 目标可立即为零，估计机械角度不能瞬移；仍只靠名义舵速推算。
    const double return_step = cfg.servo_rate * dt;
    estimated += (-estimated).cwiseMax(-return_step).cwiseMin(return_step);
    forces.setZero();
    attainable.setZero();
    achieved.setZero();
    previous_wrench = target;
    state = "RECENTERING";
    return {forces, V2::Zero(), true, false};
  }
  // 回中限速独立于推力放行：等待规划时继续回中，不继续执行过期的非零角度目标。
  // 有效新规划可以立即接管；输入失效/停机仍在上方优先处理。
  if (center || planner_valid || returning_to_center) {
    const double servo_step = cfg.servo_rate * dt;
    estimated += (solver_angles - estimated).cwiseMax(-servo_step).cwiseMin(servo_step);
  }
  // Missing/failed planner never releases thrust. Zero force bypasses normal slew limiting.
  if (!center && !planner_valid) {
    forces.setZero();
    // Diagnostics describe the zero-force command actually sent this cycle.
    attainable.setZero();
    achieved.setZero();
    state = "WAIT_PLANNER";
    return {forces, estimated, true, false};
  }
  update_gate(closed_loop, now);
  // 横移/升沉按对应倾转组降额；横滚取两组可用能力的较大值。
  V2 raw_sin = estimated.array().sin().abs().matrix();
  V2 attain = gate.cwiseProduct(raw_sin.cwiseMax(.4));
  attainable = target;
  attainable[1] *= attain[0];
  attainable[2] *= attain[1];
  // 闭环所需横滚力矩不能再次按 sin(舵角) 缩小：几何矩阵已包含真实力臂。
  // 开环仍保留原来的转向发力门控；固定角度求解负责力限幅和可达性。
  if (!closed_loop)
    attainable[3] *= gate.cwiseProduct(raw_sin).maxCoeff();
  Allocation out =
      fixed_allocate(estimated, attainable, forces, cfg.thrust_rate * dt, cfg.geometry);
  forces = out.forces;
  achieved = out.achieved;
  state = "ALLOCATING";
  return {forces, estimated, true, false};
}
} // namespace uw
