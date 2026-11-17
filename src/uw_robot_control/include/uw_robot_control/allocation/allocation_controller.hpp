#pragma once
#include "uw_robot_control/allocation/allocator.hpp"
#include <optional>
namespace uw {
// 推力限速 N/s，舵机限速 rad/s，规划周期/有效期 s。
struct AllocationConfig {
  AllocationGeometry geometry;
  double plan_min_improvement = 0;
  double thrust_rate = 10, servo_rate = pi, plan_period = .5, plan_max_age = 1.;
};
// Converts one current, enabled body wrench into physically bounded actuators.
class AllocationController {
public:
  explicit AllocationController(AllocationConfig config = {}) : cfg(config) {}
  Actuators update(const WrenchTarget &, double now, bool input_valid,
                   const std::optional<Plan> &result = std::nullopt);
  void stop();
  std::optional<Request> take_request();
  bool planner_valid = false;
  double planner_ms = 0, cycle_dt = .02;
  uint64_t epoch = 0;
  V6 target = V6::Zero(), attainable = V6::Zero(), achieved = V6::Zero();
  std::string state = "DISABLED";

private:
  void update_plan(bool center, double now, const std::optional<Plan> &result);
  void update_gate(bool closed_loop, double now);
  AllocationConfig cfg;
  V2 solver_angles = V2::Zero(), estimated = V2::Zero(), gate = V2::Ones();
  V4 forces = V4::Zero();
  V6 previous_wrench = V6::Zero();
  double last_time = -1, last_plan = -1e9, gate_until = 0, last_valid_plan = -1e9;
  uint64_t source_session = 0, source_epoch = 0;
  bool source_seen = false;
  // 回中请求消失后仍可在零推力下完成回中，直到有效新规划接管。
  bool was_centering = false, returning_to_center = false;
  std::optional<Request> pending;
};
} // namespace uw
