#include "uw_robot_control/allocation/allocation_controller.hpp"
#include <iostream>
#include <stdexcept>
static void check(bool v, const char *reason) {
  if (!v)
    throw std::runtime_error(reason);
}
int main() {
  using namespace uw;
  Actuators a;
  a.forces.setConstant(5);
  a.angles.setZero();
  check((forward(a).head<3>() - V3(20, 0, 0)).norm() < 1e-10, "forward mapping");
  V6 simple = V6::Zero();
  simple[0] = 10;
  auto inverse = analytic_inverse(simple);
  check(inverse.ok && (inverse.achieved - simple).cwiseAbs().maxCoeff() < 2e-3, "analytic inverse");
  V6 target;
  target << 50, 10, 20, 3, 4, 5;
  V4 ref = V4::Constant(1);
  auto allocated = fixed_allocate(V2(.3, -.5), target, ref, .2);
  check((allocated.forces - ref).cwiseAbs().maxCoeff() < .2000001, "QP slew bounds");
  allocated = fixed_allocate(V2(.3, -.5), target);
  check(allocated.forces.minCoeff() >= reverse_limit &&
            allocated.forces.maxCoeff() <= forward_limit,
        "physical bounds");
  AllocationController controller;
  WrenchTarget command;
  command.wrench[0] = 5;
  command.enabled = true;
  command.session = 100;
  command.epoch = 1;
  auto output = controller.update(command, 1, true);
  check(output.forces.isZero(), "no thrust before first plan");
  auto request = controller.take_request();
  check(request.has_value(), "planner request");
  auto plan = plan_angles(*request);
  output = controller.update(command, 1.02, true, plan);
  check(controller.planner_valid && output.enabled, "fresh plan");
  check(output.forces.cwiseAbs().maxCoeff() <= .2000001, "output slew limit");
  check(!controller.update(command, 1.04, false).enabled, "upstream wrench timeout must disable");
  command.epoch = 2;
  output = controller.update(command, 1.06, true, plan);
  check(!controller.planner_valid && output.forces.isZero(), "old result after rearm");
  command.session = 200;
  output = controller.update(command, 1.08, true, plan);
  check(!controller.planner_valid && output.forces.isZero(), "old result after controller restart");
  request = controller.take_request();
  check(request.has_value(), "restart planning");
  plan = plan_angles(*request);
  controller.update(command, 1.10, true, plan);
  output = controller.update(command, 2.5, true);
  check(!controller.planner_valid && output.forces.isZero(), "expired plan must zero thrust");
  check(controller.achieved.isZero() && controller.attainable.isZero(),
        "expired plan diagnostics must describe zero-force output");

  // 回中期间暂时失去 center 请求/规划时仍连续回中，且绝不释放推力。
  AllocationController homing;
  WrenchTarget home_command;
  home_command.enabled = true;
  home_command.session = 10;
  home_command.wrench[0] = 1;
  homing.update(home_command, 10, true);
  auto home_request = homing.take_request();
  Plan home_plan;
  home_plan.valid = true;
  home_plan.epoch = home_request->epoch;
  home_plan.started = home_request->started;
  home_plan.target = home_request->target;
  home_plan.angles = V2(1.2, -.9);
  Actuators previous;
  for (int i = 1; i <= 20; ++i)
    previous = homing.update(home_command, 10 + .02 * i, true,
                             i == 1 ? std::optional<Plan>(home_plan) : std::nullopt);
  home_command.wrench.setZero();
  home_command.center = true;
  auto home_output = homing.update(home_command, 10.42, true);
  const auto home_epoch = homing.epoch;
  previous = home_output;
  home_output = homing.update(home_command, 10.44, true);
  check(homing.epoch == home_epoch, "continuous centering must not reset planner epoch");
  check(home_output.angles.cwiseAbs().sum() < previous.angles.cwiseAbs().sum(),
        "continuous return");
  for (int i = 0; i < 10; ++i) {
    previous = home_output;
    home_command.center = i % 3 == 0;
    home_output = homing.update(home_command, 10.46 + .02 * i, true);
    check(home_output.angles.cwiseAbs().sum() < previous.angles.cwiseAbs().sum(),
          "planner wait must not freeze return motion");
    check((home_output.angles - previous.angles).cwiseAbs().maxCoeff() <= pi * .02 + 1e-9,
          "return must respect servo rate");
    if (!home_command.center)
      check(home_output.forces.isZero(), "planner wait must keep zero thrust");
  }
  // 有效新规划立即接管；过期的非回中目标仍保持原有安全规则。
  home_command.center = false;
  homing.update(home_command, 10.68, true);
  auto next_request = homing.take_request();
  check(next_request.has_value(), "new planning after return");
  Plan next_plan;
  next_plan.valid = true;
  next_plan.epoch = next_request->epoch;
  next_plan.started = next_request->started;
  next_plan.target = next_request->target;
  next_plan.angles = V2(1.2, .9);
  home_output = homing.update(home_command, 10.70, true, next_plan);
  check(homing.planner_valid, "fresh plan must take over return");
  previous = home_output;
  home_output = homing.update(home_command, 12, true);
  check(!homing.planner_valid && home_output.forces.isZero(), "expired plan zero thrust");
  check((home_output.angles - previous.angles).norm() < 1e-12,
        "do not move toward expired nonzero plan");
  check(!homing.update(home_command, 12.02, false).enabled, "invalid input overrides return");

  // 快速回中绕过普通限速和旧规划，直接输出两个零角度及零推力。
  AllocationController direct;
  WrenchTarget direct_command;
  direct_command.enabled = true;
  direct_command.session = 99;
  direct_command.wrench[2] = 4;
  direct.update(direct_command, 40, true);
  auto direct_request = direct.take_request();
  Plan direct_plan;
  direct_plan.valid = true;
  direct_plan.epoch = direct_request->epoch;
  direct_plan.started = direct_request->started;
  direct_plan.target = direct_request->target;
  direct_plan.angles = V2(1, -1);
  Actuators tilted;
  for (int i = 1; i <= 15; ++i)
    tilted = direct.update(direct_command, 40 + .02 * i, true,
                           i == 1 ? std::optional<Plan>(direct_plan) : std::nullopt);
  check(tilted.angles.norm() > .5, "test requires substantial tilt");
  direct_command.recenter = true;
  auto reset = direct.update(direct_command, 40.32, true, direct_plan);
  check(reset.angles.isZero() && reset.forces.isZero() && reset.enabled,
        "direct zero target and zero thrust");
  check(direct.state == "RECENTERING", "direct return diagnostic");
  direct_command.recenter = false;
  reset = direct.update(direct_command, 40.34, true, direct_plan);
  check(reset.forces.isZero() && !direct.planner_valid, "old plan must not undo direct return");
  direct_request = direct.take_request();
  check(direct_request.has_value(), "automatic allocation must restart after direct return");
  direct_plan.epoch = direct_request->epoch;
  direct_plan.started = direct_request->started;
  direct_plan.target = direct_request->target;
  reset = direct.update(direct_command, 40.36, true, direct_plan);
  check(direct.planner_valid && reset.angles.norm() > 0, "fresh automatic plan may tilt again");

  // 无舵角反馈：零目标不等于机械到位。退出快速回中后保留按限速推算的角度。
  AllocationController estimate_only;
  WrenchTarget ec;
  ec.enabled = true;
  ec.session = 2000;
  ec.wrench[0] = 1;
  estimate_only.update(ec, 60, true);
  auto er = estimate_only.take_request();
  Plan ep;
  ep.valid = true;
  ep.epoch = er->epoch;
  ep.started = er->started;
  ep.target = er->target;
  ep.angles = V2(1, -1);
  Actuators eo;
  for (int i = 1; i <= 16; ++i)
    eo = estimate_only.update(ec, 60 + .02 * i, true,
                              i == 1 ? std::optional<Plan>(ep) : std::nullopt);
  ec.recenter = true;
  eo = estimate_only.update(ec, 60.34, true);
  check(eo.angles.isZero(), "direct command target is zero");
  ec.recenter = false;
  eo = estimate_only.update(ec, 60.36, true);
  check(eo.angles.cwiseAbs().maxCoeff() > .7,
        "estimated mechanical angle must not teleport to zero");

  Request stable_request;
  stable_request.angles = V2(.6, -.8);
  stable_request.forces = V4::Constant(1);
  stable_request.target = wrench_matrix(stable_request.angles) * stable_request.forces;
  auto stable_plan = plan_angles(stable_request, {}, .01);
  check((stable_plan.angles - stable_request.angles).norm() < 1e-12,
        "small optimizer benefit must retain existing angle");
  Request translation_request;
  translation_request.target[1] = 10;
  auto translation_plan = plan_angles(translation_request, {}, .01);
  check(translation_plan.valid && translation_plan.angles.norm() > .1,
        "substantial translation demand must still replan angles");

  Request idle_group;
  idle_group.angles = V2(0, .4);
  idle_group.target[4] = .3;
  auto idle_plan = plan_angles(idle_group, {}, .01);
  check(std::abs(idle_plan.angles[1] - .4) < 1e-12,
        "near-zero thrust group must not roam among equivalent angles");
  std::cout << "allocation_test passed\n";
}
