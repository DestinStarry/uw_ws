#pragma once
#include "uw_robot_interfaces/robot_types.hpp"
namespace uw {
// 三维参数顺序为 x/y/z；角度用 rad，力用 N，力矩用 N·m。
struct ControlConfig {
  // 六轴速度指令到机体力/力矩的静态增益：N/(m/s) 与 N·m/(rad/s)。
  // 数值沿用原摇杆量纲迁移后的等价值：20/6/40 对应满杆 20/6/10 N。
  V3 damping{20, 6, 40}, open_damping{60, 20, 120};
  // rate_ff 是角速度指令的前馈增益；open_moment 是开环模式下满杆角速度对应的力矩。
  V3 rate_ff{1.3, 1.5, 1.5}, open_moment{5, 5, 5};
  // 发送端满杆角速度：只用于中立判据和开环量纲还原，不参与闭环运算。
  V3 rate_full_scale{25 * pi / 180, 22 * pi / 180, 20 * pi / 180};
  V3 moment{3, 5, 5};
  V3 attitude_kp{.8, 1.2, 1.2}, attitude_deadzone{8 * pi / 180, 3 * pi / 180, 5 * pi / 180};
  V3 rate_kp{1, 1.5, 1.5}, rate_ki{.06, .20, .20};
  V3 inner_rate{pi / 2, pi / 2, pi / 3};
  V3 moment_sign{-1, -1, -1};
  V6 wrench_rate = (V6() << 150, 150, 150, 8, 8, 8).finished();
  double depth_kp = 20, depth_ki = 2, depth_kd = 2, depth_force = 8;
  // 松杆后给舵机的无推力回中时间，之后恢复正常自动分配。
  double release_center_time = .25;
  // 浮力大小 × 浮心相对重心偏移，Nm；默认零，不改变实物旧参数。
  V3 buoyancy_moment_coeff = V3::Zero();
  bool prioritize_hold_on_release = false;
  // 默认兼容旧算法；共享配置启用连续死区和较低延迟的角速度滤波。
  bool smooth_deadband = false, roll_smooth_deadband = false;
  double roll_gyro_filter_alpha = .25;
  double gyro_filter_alpha = .25;
  V3 rate_deadzone = V3::Constant(.5 * pi / 180);
  bool yaw_hold = true, roll_hold = true;
};
// Feedback control ends at a desired body wrench. No actuator solver here.
class Controller {
public:
  explicit Controller(ControlConfig config = {}) : cfg(config) {}
  WrenchTarget update(const Command &, const Imu &, const Depth &, double now, bool cmd_valid);
  void stop();
  // 只读暴露定深目标用于遥测；不改变目标捕获或控制运算。
  double depth_target_m() const {
    return depth_target;
  }

  bool armed = false, closed_loop = true, depth_hold = false;
  double cycle_dt = .02;
  uint64_t epoch = 0;
  std::string state = "DISARMED";
  V6 target = V6::Zero();

private:
  // 六轴都接近中立时返回 true；角速度阈值按发送端满杆量程的 1% 缩放。
  bool neutral(const V6 &v) const {
    return std::abs(v[0]) <= .01 && std::abs(v[1]) <= .01 && std::abs(v[2]) <= .0025 &&
           std::abs(v[3]) <= .01 * cfg.rate_full_scale[0] &&
           std::abs(v[4]) <= .01 * cfg.rate_full_scale[1] &&
           std::abs(v[5]) <= .01 * cfg.rate_full_scale[2];
  }
  V3 attitude_moment(const V3 &rate_ref, const Imu &imu, const Eigen::Quaterniond &q, double dt);
  void apply_depth_hold(V3 &force, double w_ref, const Depth &depth, const Eigen::Quaterniond &q,
                        bool depth_ok, double dt);
  ControlConfig cfg;
  Eigen::Quaterniond target_q = Eigen::Quaterniond::Identity();
  V3 integral = V3::Zero(), filtered_gyro = V3::Zero();
  double depth_i = 0, depth_target = 0, last_time = -1;
  bool previous_arm = false, previous_depth = false, previous_loop = false;
  bool was_angular = false, was_vertical = false, depth_was_valid = false;
  bool centering = false, manual_was_active = false;
  double recenter_until = -1;
};
} // namespace uw
