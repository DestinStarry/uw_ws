#include "uw_robot_control/closed_loop/controller.hpp"
#include <iostream>
#include <stdexcept>
static void check(bool v, const char *reason) {
  if (!v)
    throw std::runtime_error(reason);
}
int main() {
  using namespace uw;
  Controller control;
  Command cmd;
  Imu imu;
  imu.valid = true;
  Depth depth;
  check(!control.update(cmd, imu, depth, 1, true).enabled, "default must disarm");
  cmd.arm = true;
  check(control.update(cmd, imu, depth, 1.02, true).enabled, "valid arm");
  cmd.arm = false;
  cmd.velocity[0] = .2;
  auto output = control.update(cmd, imu, depth, 1.04, true);
  check(output.wrench[0] > 0, "controller must output target force");
  check(output.wrench.tail<3>().norm() < 1e-8, "no unsolicited rotation");
  const auto armed_epoch = control.epoch;
  imu.valid = false;
  check(!control.update(cmd, imu, depth, 1.06, true).enabled, "IMU watchdog");
  check(control.epoch != armed_epoch, "stop must invalidate allocation session");
  imu.valid = true;
  cmd.arm = true;
  control.update(cmd, imu, depth, 1.08, true);
  check(!control.update(cmd, imu, depth, 1.10, false).enabled, "CMD watchdog");
  check(!control.update(cmd, imu, depth, 1.12, true).enabled, "held arm must not rearm");
  cmd.arm = false;
  control.update(cmd, imu, depth, 1.14, true);
  cmd.arm = true;
  cmd.estop = true;
  check(!control.update(cmd, imu, depth, 1.16, true).enabled, "estop dominates arm");
  // 遥测必须对应实际目标：开启时捕获，升沉时更新，松杆时重新捕获。
  Controller depth_control;
  Command depth_cmd;
  Imu depth_imu;
  depth_imu.valid = true;
  Depth sample{1.25, 0, true};
  depth_cmd.arm = true;
  depth_control.update(depth_cmd, depth_imu, sample, 2.0, true);
  depth_cmd.arm = false;
  depth_cmd.depth_toggle = true;
  depth_control.update(depth_cmd, depth_imu, sample, 2.02, true);
  check(depth_control.depth_hold, "depth hold enabled");
  check(std::abs(depth_control.depth_target_m() - 1.25) < 1e-12, "capture actual depth target");
  depth_cmd.depth_toggle = false;
  depth_cmd.velocity[2] = .25;
  depth_control.update(depth_cmd, depth_imu, sample, 2.04, true);
  check(std::abs(depth_control.depth_target_m() - 1.245) < 1e-12, "report changing depth target");
  depth_cmd.velocity[2] = 0;
  sample.value = 1.3;
  depth_control.update(depth_cmd, depth_imu, sample, 2.06, true);
  check(std::abs(depth_control.depth_target_m() - 1.3) < 1e-12, "report recaptured depth target");

  // 回中迟滞：小反馈不反复退出，较大纠正和人工输入仍立即接管。
  ControlConfig home_config;
  home_config.attitude_kp.setZero();
  home_config.rate_ki.setZero();
  Controller home_control(home_config);
  Command home_cmd;
  Imu home_imu;
  Depth home_depth{1, 0, true};
  home_imu.valid = true;
  home_cmd.arm = true;
  check(home_control.update(home_cmd, home_imu, home_depth, 20, true).center, "enter centering");
  home_cmd.arm = false;
  home_imu.gyro[2] = .064;
  check(home_control.update(home_cmd, home_imu, home_depth, 20.02, true).center,
        "small feedback must not interrupt centering");
  home_imu.gyro[2] = .5;
  check(!home_control.update(home_cmd, home_imu, home_depth, 20.04, true).center,
        "large feedback must leave centering");
  home_imu.gyro.setZero();
  WrenchTarget centered;
  for (int i = 0; i < 40; ++i)
    centered = home_control.update(home_cmd, home_imu, home_depth, 20.06 + .02 * i, true);
  check(centered.center, "reenter when feedback settles");
  home_cmd.velocity[0] = .2;
  check(!home_control.update(home_cmd, home_imu, home_depth, 20.86, true).center,
        "manual command overrides centering");
  home_cmd.velocity[0] = 0;
  home_cmd.depth_toggle = true;
  auto release_with_depth = home_control.update(home_cmd, home_imu, home_depth, 20.88, true);
  check(release_with_depth.recenter, "release must immediately center even with depth hold");
  auto resumed_depth = home_control.update(home_cmd, home_imu, home_depth, 21.2, true);
  check(!resumed_depth.recenter && !resumed_depth.center && home_control.depth_hold,
        "depth hold must resume allocation after return window");

  // 逐项覆盖横移、升沉、横滚、偏航/俯仰的松杆回中，不能依赖水平或小力矩。
  for (int axis = 0; axis < 8; ++axis) {
    Controller release_control;
    Command release_cmd;
    Imu release_imu;
    Depth release_depth{1, 0, true};
    release_imu.valid = true;
    release_cmd.arm = true;
    release_imu.q = Eigen::AngleAxisd(.3, V3::UnitY());
    release_imu.gyro = V3(.2, .2, .2);
    release_control.update(release_cmd, release_imu, release_depth, 30, true);
    release_cmd.arm = false;
    if (axis == 0)
      release_cmd.velocity[0] = .4;
    if (axis == 1)
      release_cmd.velocity[1] = -.4;
    if (axis == 2)
      release_cmd.velocity[2] = .1;
    if (axis == 3)
      release_cmd.velocity[2] = -.1;
    if (axis == 4)
      release_cmd.velocity[3] = .436;
    if (axis == 5)
      release_cmd.velocity[3] = -.436;
    if (axis == 6)
      release_cmd.velocity[4] = .115;
    if (axis == 7)
      release_cmd.velocity[5] = .105;
    check(!release_control.update(release_cmd, release_imu, release_depth, 30.02, true).recenter,
          "held manual input must not force center");
    release_cmd = Command{};
    auto released = release_control.update(release_cmd, release_imu, release_depth, 30.04, true);
    check(released.recenter && released.center, "every released input must directly center");
    check(release_control.update(release_cmd, release_imu, release_depth, 30.20, true).recenter,
          "return window gives physical servo time to move");
    check(!release_control.update(release_cmd, release_imu, release_depth, 30.40, true).recenter,
          "neutral must not retrigger return window");
    release_cmd.velocity[1] = .4;
    release_control.update(release_cmd, release_imu, release_depth, 30.42, true);
    release_cmd = Command{};
    release_control.update(release_cmd, release_imu, release_depth, 30.44, true);
    release_cmd.velocity[1] = .4;
    check(!release_control.update(release_cmd, release_imu, release_depth, 30.46, true).recenter,
          "new manual input must cancel return window");
    check(!release_control.update(release_cmd, release_imu, release_depth, 30.48, false).enabled,
          "stale command must still stop output");
  }

  // 水下稳姿优先：倾斜松杆不能强制停推，静态浮力力矩由前馈抵消。
  ControlConfig wet_config;
  wet_config.moment_sign.setOnes();
  wet_config.prioritize_hold_on_release = true;
  wet_config.buoyancy_moment_coeff = V3(0, 0, 1.828);
  wet_config.attitude_kp.setZero();
  wet_config.rate_ki.setZero();
  Controller wet(wet_config);
  Command wet_cmd;
  Imu wet_imu;
  Depth wet_depth{1, 0, true};
  wet_imu.valid = true;
  wet_imu.q = Eigen::AngleAxisd(.3, V3::UnitY());
  wet_cmd.arm = true;
  wet.update(wet_cmd, wet_imu, wet_depth, 50, true);
  wet_cmd.arm = false;
  wet_cmd.velocity[4] = .115;
  wet.update(wet_cmd, wet_imu, wet_depth, 50.02, true);
  wet_cmd = Command{};
  auto wet_output = wet.update(wet_cmd, wet_imu, wet_depth, 50.04, true);
  check(!wet_output.recenter, "tilted closed loop must not cut thrust on release");
  for (int i = 0; i < 20; ++i)
    wet_output = wet.update(wet_cmd, wet_imu, wet_depth, 50.06 + i * .02, true);
  check(std::abs(wet_output.wrench[4] - 1.828 * std::sin(.3)) < 1e-8,
        "static restoring moment compensation");
  wet_cmd.depth_toggle = true;
  wet.update(wet_cmd, wet_imu, wet_depth, 50.48, true);
  wet_cmd.depth_toggle = false;
  wet_cmd.velocity[2] = .075;
  wet.update(wet_cmd, wet_imu, wet_depth, 50.50, true);
  wet_cmd.velocity[2] = 0;
  check(!wet.update(wet_cmd, wet_imu, wet_depth, 50.52, true).recenter,
        "depth hold must not lose support during release");
  wet_cmd.estop = true;
  check(!wet.update(wet_cmd, wet_imu, wet_depth, 50.54, true).enabled,
        "estop must override static compensation");

  ControlConfig soft_cfg;
  soft_cfg.smooth_deadband = true;
  soft_cfg.rate_deadzone.setZero();
  soft_cfg.attitude_deadzone = V3::Constant(.5 * pi / 180);
  soft_cfg.attitude_kp.setOnes();
  soft_cfg.rate_kp.setOnes();
  soft_cfg.rate_ki.setZero();
  soft_cfg.moment_sign.setOnes();
  Controller soft_control(soft_cfg);
  Command soft_cmd;
  Imu soft_imu;
  Depth soft_depth;
  soft_imu.valid = true;
  soft_cmd.arm = true;
  soft_control.update(soft_cmd, soft_imu, soft_depth, 70, true);
  soft_cmd.arm = false;
  soft_imu.q = Eigen::AngleAxisd(.51 * pi / 180, V3::UnitY());
  auto soft_out = soft_control.update(soft_cmd, soft_imu, soft_depth, 70.02, true);
  check(std::abs(soft_out.wrench[4]) < .001 && std::abs(soft_out.wrench[4]) > 0,
        "crossing attitude deadband must not introduce a finite torque step");
  std::cout << "control_test passed\n";
}
