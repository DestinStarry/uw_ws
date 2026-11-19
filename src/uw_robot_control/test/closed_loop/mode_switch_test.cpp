/// @file mode_switch_test.cpp
/// @brief 打杆/松杆无扰交接测试：力矩连续、逐轴独立、积分限幅与停机清零。
///

// 手动模式逐轴打杆/松杆的无扰交接测试。
// 关注点：松杆前后机体力矩连续、姿态目标被重新捕获、一个轴松杆不影响其它轴、
// 复合姿态下切换稳定，以及积分限幅/抗饱和与安全停机清零。
#include "uw_robot_control/closed_loop/controller.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace uw;

namespace {
void check(bool v, const char *reason) {
  if (!v)
    throw std::runtime_error(reason);
}
constexpr double kRateEps = 0.05; // 打杆判据：满杆 5%

// 测试配置：力矩 sign 取正方便断言；关闭 D 项（生产配置亦为 0）隔离交接公式；
// 放大积分限幅避免测试被限幅掩盖；静水前馈关闭。
ControlConfig test_config() {
  ControlConfig c;
  c.moment_sign.setOnes();
  c.rate_kd.setZero();
  c.hydrostatic_coeff.setZero();
  c.rate_integral_limit = 1e6;
  c.attitude_integral_limit = 1e6;
  c.inner_rate = V3::Constant(100);
  c.moment = V3::Constant(1e6); // 解除力矩饱和，直接观测 PID 输出连续性
  c.wrench_rate = V6::Constant(1e6); // 目标不做斜率限制，观测原始力矩
  c.rate_full_scale = V3::Constant(1.0);
  return c;
}

struct Fixture {
  Controller ctrl{test_config()};
  Command cmd;
  Imu imu;
  Depth depth{1, 0, true};
  double t = 0;
  Fixture() {
    imu.valid = true;
    cmd.arm = true;
    step();          // 解锁
    cmd.arm = false;
    step();          // 释放解锁沿
  }
  WrenchTarget step(double dt = .02) {
    t += dt;
    return ctrl.update(cmd, imu, depth, t, true);
  }
  V3 moment() const { return ctrl.last_moment(); }
  void set_rate(int axis, double value) {
    cmd.velocity[3 + axis] = value;
  }
};

// 打杆一段时间让积分建立稳定补偿值。
void run(WrenchTarget &out, Fixture &f, int cycles) {
  for (int i = 0; i < cycles; ++i)
    out = f.step();
}
} // namespace

int main() {
  using namespace uw;

  // ---- 1. 单轴松杆：力矩在切换点连续 ----
  {
    Fixture f;
    f.set_rate(1, .5); // pitch 打杆
    WrenchTarget out;
    run(out, f, 60);
    const V3 before = f.moment();
    check(std::abs(before[1]) > .05, "pitch stick must build a non-trivial moment");
    f.set_rate(1, 0); // 松杆
    out = f.step();
    const V3 after = f.moment();
    check(std::abs(after[1] - before[1]) < 1e-6,
          "pitch release must keep moment continuous (bumpless)");
  }

  // ---- 2. 三轴同时操作后逐轴松杆：松杆动作只影响本轴 ----
  // 仍在打杆的轴力矩本来就会随积分逐帧变化，故用"平行对照"：A 松某轴、B 保持该轴，
  // 比较两者在其它两轴上的力矩是否一致，从而隔离松杆动作本身的影响。
  {
    Fixture a, b;
    auto drive = [](Fixture &f) {
      f.set_rate(0, .4);
      f.set_rate(1, -.5);
      f.set_rate(2, .6);
    };
    drive(a);
    drive(b);
    WrenchTarget out;
    run(out, a, 80);
    run(out, b, 80);
    check(a.moment().cwiseAbs().minCoeff() > .03, "all three axes must be active");

    // A 松 pitch，B 继续打 pitch；两者 roll/yaw 应完全一致
    a.set_rate(1, 0);
    a.step();
    b.step();
    check(std::abs(a.moment()[0] - b.moment()[0]) < 1e-9,
          "pitch release must not affect roll");
    check(std::abs(a.moment()[2] - b.moment()[2]) < 1e-9,
          "pitch release must not affect yaw");

    // A 再松 roll，B 继续打 roll
    a.set_rate(0, 0);
    a.step();
    b.step();
    check(std::abs(a.moment()[2] - b.moment()[2]) < 1e-9,
          "roll release must not affect yaw");
  }

  // ---- 3. 复合姿态下切换：姿态目标重新捕获，松杆后能稳定保持 ----
  {
    Fixture f;
    // 复合倾斜：roll 25° + pitch 20° + yaw 15°
    Eigen::Quaterniond q = Eigen::AngleAxisd(15 * pi / 180, V3::UnitZ()) *
                           Eigen::AngleAxisd(20 * pi / 180, V3::UnitY()) *
                           Eigen::AngleAxisd(25 * pi / 180, V3::UnitX());
    f.imu.q = q;
    f.imu.gyro = V3(.05, -.04, .03);
    f.step();
    f.set_rate(0, .45);
    f.set_rate(2, -.35);
    WrenchTarget out;
    run(out, f, 70);
    const V3 before = f.moment();

    // 同时松 roll 与 yaw
    f.set_rate(0, 0);
    f.set_rate(2, 0);
    out = f.step();
    const V3 after = f.moment();
    check(std::abs(after[0] - before[0]) < 1e-6, "roll bumpless under compound attitude");
    check(std::abs(after[2] - before[2]) < 1e-6, "yaw bumpless under compound attitude");

    // 松杆后保持一段，力矩不应发散（姿态外环重新接管）
    f.imu.gyro.setZero();
    for (int i = 0; i < 50; ++i)
      out = f.step();
    check(out.enabled && std::isfinite(f.moment().norm()),
          "compound-attitude hold must remain stable and finite");
    check(f.moment().cwiseAbs().maxCoeff() < 10,
          "hold moment must stay bounded after bumpless handover");
  }

  // ---- 3b. 三轴同时松杆：三轴都连续 ----
  {
    Fixture f;
    f.set_rate(0, .4);
    f.set_rate(1, -.5);
    f.set_rate(2, .6);
    WrenchTarget out;
    run(out, f, 80);
    const V3 before = f.moment();
    f.set_rate(0, 0);
    f.set_rate(1, 0);
    f.set_rate(2, 0);
    out = f.step();
    const V3 after = f.moment();
    check((after - before).cwiseAbs().maxCoeff() < 1e-6,
          "simultaneous three-axis release must be bumpless on every axis");
  }

  // ---- 4. 积分限幅与抗饱和 ----
  {
    ControlConfig c = test_config();
    c.rate_integral_limit = .8; // 与生产 YAML 一致
    c.attitude_integral_limit = .5;
    c.moment = V3::Constant(1e6);
    Controller ctrl(c);
    Command cmd;
    Imu imu;
    Depth depth{1, 0, true};
    imu.valid = true;
    cmd.arm = true;
    ctrl.update(cmd, imu, depth, 1, true);
    cmd.arm = false;

    // 持续大姿态误差：外环积分必须被限幅且输出不发散
    imu.q = Eigen::AngleAxisd(1.2, V3::UnitX());
    for (int i = 0; i < 200; ++i)
      ctrl.update(cmd, imu, depth, 1.02 + .02 * i, true);
    check(ctrl.rate_integral().cwiseAbs().maxCoeff() <= .8 + 1e-9,
          "rate integral must respect configured limit");
    check(std::isfinite(ctrl.last_moment().norm()), "moment must stay finite under saturation");

    // 抗饱和：力矩上限很低、积分限幅放大时，积分应在"刚好把力矩顶到上限"处冻结，
    // 而不是一直累积到限幅值（limit=1000 远大于合理值，只有抗饱和才能挡住）。
    ControlConfig anti = test_config();
    anti.moment = V3::Constant(.2);
    anti.rate_integral_limit = 1000;
    anti.moment_sign.setOnes();
    Controller anti_ctrl(anti);
    Command anti_cmd;
    Imu anti_imu;
    Depth anti_depth{1, 0, true};
    anti_imu.valid = true;
    anti_cmd.arm = true;
    anti_ctrl.update(anti_cmd, anti_imu, anti_depth, 100, true);
    anti_cmd.arm = false;
    anti_imu.gyro[1] = -1.0; // 大角速度误差，力矩必饱和
    for (int i = 0; i < 300; ++i)
      anti_ctrl.update(anti_cmd, anti_imu, anti_depth, 100.02 + .02 * i, true);
    check(std::abs(anti_ctrl.rate_integral()[1]) < 1.0,
          "anti-windup must freeze integral near torque limit, not reach 1000");
  }

  // ---- 5. 安全状态：急停/失联/解除武装仍清零积分 ----
  {
    ControlConfig c = test_config();
    c.moment = V3::Constant(1e6);
    Controller ctrl(c);
    Command cmd;
    Imu imu;
    Depth depth{1, 0, true};
    imu.valid = true;
    cmd.arm = true;
    ctrl.update(cmd, imu, depth, 10, true);
    cmd.arm = false;
    cmd.velocity[4] = .5;
    for (int i = 0; i < 40; ++i)
      ctrl.update(cmd, imu, depth, 10.02 + .02 * i, true);
    check(ctrl.rate_integral().cwiseAbs().maxCoeff() > .01,
          "integral must be built up before safety checks");

    // 急停
    cmd.estop = true;
    ctrl.update(cmd, imu, depth, 11, true);
    check(ctrl.rate_integral().cwiseAbs().maxCoeff() < 1e-12, "estop must clear integrals");
    check(std::abs(ctrl.last_moment().norm()) < 1e-12, "estop must clear moment history");
    cmd.estop = false;

    // 重新解锁后重新建立积分，再验证通信失联
    cmd.arm = true;
    ctrl.update(cmd, imu, depth, 11.02, true);
    cmd.arm = false;
    cmd.velocity[4] = .5;
    for (int i = 0; i < 40; ++i)
      ctrl.update(cmd, imu, depth, 11.04 + .02 * i, true);
    ctrl.update(cmd, imu, depth, 12, false); // cmd stale
    check(ctrl.rate_integral().cwiseAbs().maxCoeff() < 1e-12, "cmd stale must clear integrals");

    // 重新解锁后验证解除武装
    cmd.arm = true;
    ctrl.update(cmd, imu, depth, 12.02, true);
    cmd.arm = false;
    cmd.velocity[4] = .5;
    for (int i = 0; i < 40; ++i)
      ctrl.update(cmd, imu, depth, 12.04 + .02 * i, true);
    cmd.arm = true;
    ctrl.update(cmd, imu, depth, 13, true); // disarm
    check(ctrl.rate_integral().cwiseAbs().maxCoeff() < 1e-12, "disarm must clear integrals");
  }

  std::cout << "mode_switch_test passed\n";
  return 0;
}
