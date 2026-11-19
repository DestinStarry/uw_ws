/// @file config.cpp
/// @brief 控制参数装载与校验（实现见 config.hpp 的约定）。
#include "uw_robot_control/closed_loop/config.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace uw {
namespace {

/// 读取三维向量参数：必须恰好 3 项且有限，否则抛异常。
void read_v3(rclcpp::Node &node, const std::string &key, V3 &value) {
  auto v = node.declare_parameter<std::vector<double>>(
      key, {value[0], value[1], value[2]});
  if (v.size() != 3)
    throw std::runtime_error(key + " must have 3 entries");
  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(v[i]))
      throw std::runtime_error(key + " nonfinite");
    value[i] = v[i];
  }
}

} // namespace

ControlSettings load_control_settings(rclcpp::Node &node) {
  ControlSettings s;
  ControlConfig &c = s.control;

  // ---- 平移与开环 -------------------------------------------------------
  read_v3(node, "damping", c.damping);
  read_v3(node, "open_damping", c.open_damping);
  read_v3(node, "rate_ff", c.rate_ff);
  read_v3(node, "open_moment", c.open_moment);
  read_v3(node, "rate_full_scale_rad_s", c.rate_full_scale);
  read_v3(node, "body_moment", c.moment);

  // ---- 姿态外环 ---------------------------------------------------------
  read_v3(node, "attitude_kp", c.attitude_kp);
  read_v3(node, "attitude_deadzone_rad", c.attitude_deadzone);
  read_v3(node, "attitude_ki", c.attitude_ki);

  // ---- 静水扶正前馈：由物理量算，不用拟合常数 ---------------------------
  const double rho = node.declare_parameter("water_density_kg_m3", 1000.0);
  const double volume = node.declare_parameter("displaced_volume_m3", 0.0);
  const double scale = node.declare_parameter("hydrostatic_comp_scale", 1.0);
  auto offset = node.declare_parameter<std::vector<double>>(
      "buoyancy_offset_m", {0.0, 0.0, 0.0});
  if (offset.size() != 3)
    throw std::runtime_error("buoyancy_offset_m requires 3 entries");
  for (double v : offset)
    if (!std::isfinite(v))
      throw std::runtime_error("buoyancy_offset_m nonfinite");
  if (!std::isfinite(rho) || rho <= 0 || !std::isfinite(volume) || volume < 0 ||
      !std::isfinite(scale) || scale < 0)
    throw std::runtime_error("invalid hydrostatic parameters");
  c.hydrostatic_coeff =
      V3(offset[0], offset[1], offset[2]) * (rho * volume * 9.80665 * scale);

  // ---- 速率内环 ---------------------------------------------------------
  read_v3(node, "rate_kp", c.rate_kp);
  read_v3(node, "rate_ki", c.rate_ki);
  read_v3(node, "rate_kd", c.rate_kd);
  c.rate_kd_filter = node.declare_parameter("rate_kd_filter", c.rate_kd_filter);
  c.rate_integral_limit =
      node.declare_parameter("rate_integral_limit", c.rate_integral_limit);
  if (!std::isfinite(c.rate_kd_filter) || c.rate_kd_filter <= 0 ||
      c.rate_kd_filter > 1)
    throw std::runtime_error("invalid rate_kd_filter");
  if (!std::isfinite(c.rate_integral_limit) || c.rate_integral_limit <= 0)
    throw std::runtime_error("invalid rate_integral_limit");
  read_v3(node, "rate_deadzone_rad_s", c.rate_deadzone);
  c.smooth_deadband = node.declare_parameter("smooth_deadband", false);
  c.roll_smooth_deadband = node.declare_parameter("roll_smooth_deadband", false);

  // ---- 反馈滤波 ---------------------------------------------------------
  c.roll_gyro_filter_alpha =
      node.declare_parameter("roll_gyro_filter_alpha", c.gyro_filter_alpha);
  if (!std::isfinite(c.roll_gyro_filter_alpha) ||
      c.roll_gyro_filter_alpha <= 0 || c.roll_gyro_filter_alpha > 1)
    throw std::runtime_error("invalid roll gyro filter");
  c.gyro_filter_alpha =
      node.declare_parameter("gyro_filter_alpha", c.gyro_filter_alpha);
  if (!std::isfinite(c.gyro_filter_alpha) || c.gyro_filter_alpha <= 0 ||
      c.gyro_filter_alpha > 1 || (c.rate_deadzone.array() < 0).any())
    throw std::runtime_error("invalid feedback filter/deadzone");

  // ---- 输出限幅与模式 ---------------------------------------------------
  read_v3(node, "moment_sign", c.moment_sign);
  c.prioritize_hold_on_release =
      node.declare_parameter("prioritize_hold_on_release", false);
  read_v3(node, "inner_rate_rad_s", c.inner_rate);
  auto wr = node.declare_parameter<std::vector<double>>(
      "wrench_rate", {150, 150, 150, 8, 8, 8});
  if (wr.size() != 6)
    throw std::runtime_error("wrench_rate requires six entries");
  for (int i = 0; i < 6; ++i) {
    if (!std::isfinite(wr[i]) || wr[i] <= 0)
      throw std::runtime_error("invalid wrench rate");
    c.wrench_rate[i] = wr[i];
  }

  // ---- 定深 -------------------------------------------------------------
  c.depth_kp = node.declare_parameter("depth_kp", c.depth_kp);
  c.depth_ki = node.declare_parameter("depth_ki", c.depth_ki);
  c.depth_kd = node.declare_parameter("depth_kd", c.depth_kd);
  c.depth_force = node.declare_parameter("depth_force", c.depth_force);
  c.depth_change = node.declare_parameter("depth_change_mps", c.depth_change);
  c.release_center_time =
      node.declare_parameter("release_center_time_s", c.release_center_time);
  c.yaw_hold = node.declare_parameter("yaw_hold", true);
  c.roll_hold = node.declare_parameter("roll_hold", true);
  c.manual_rate_mode = node.declare_parameter("manual_rate_mode", true);

  // ---- 超时 -------------------------------------------------------------
  s.cmd_timeout = node.declare_parameter("cmd_timeout_s", .35);
  s.imu_timeout = node.declare_parameter("imu_timeout_s", .1);
  s.depth_timeout = node.declare_parameter("depth_timeout_s", 1.5);
  for (double value : {c.depth_kp, c.depth_ki, c.depth_kd, c.depth_force,
                       c.depth_change, c.release_center_time, s.cmd_timeout,
                       s.imu_timeout, s.depth_timeout})
    if (!std::isfinite(value) || value <= 0)
      throw std::runtime_error("invalid scalar parameter");

  // ---- 向量参数的统一量纲校验 -------------------------------------------
  for (int i = 0; i < 3; ++i) {
    if (c.attitude_ki[i] < 0 || c.damping[i] <= 0 || c.open_damping[i] <= 0 ||
        c.moment[i] <= 0 || c.rate_ff[i] < 0 || c.open_moment[i] <= 0 ||
        c.rate_full_scale[i] <= 0 || c.attitude_kp[i] < 0 || c.rate_kp[i] < 0 ||
        c.rate_ki[i] < 0 || c.rate_kd[i] < 0 || c.inner_rate[i] <= 0 ||
        std::abs(c.moment_sign[i]) != 1)
      throw std::runtime_error("invalid vector gain/sign");
  }
  return s;
}

} // namespace uw
