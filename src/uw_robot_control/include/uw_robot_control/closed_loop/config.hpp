/// @file config.hpp
/// @brief 控制节点的参数装载：把 ROS 参数读成 ``ControlConfig`` + 三路超时。
///
/// 从 ``control_node.cpp`` 抽出来的原因：参数声明与校验有 90 行，属于"配置面"
/// 而不是运行逻辑；单独成库后单元测试不必链接 ROS 参数系统。
#pragma once

#include <rclcpp/rclcpp.hpp>

#include "uw_robot_control/closed_loop/controller.hpp"

namespace uw {

/// 控制节点启动参数：控制器名义增益 + 指令/IMU/深度三路超时。
struct ControlSettings {
  ControlConfig control{};
  double cmd_timeout = .35;   ///< 指令帧超时，s。
  double imu_timeout = .1;    ///< IMU 超时，s。
  double depth_timeout = 1.5; ///< 深度超时，s。
};

/// 从节点参数读取并校验全部控制参数。
///
/// 约定：**任一参数非法即抛 ``std::runtime_error``**，由 main 打印并退出——
/// 带着错误参数下潜比启动失败危险得多。缺省值与 ``robot.yaml`` 保持一致，
/// 新增参数时三处（yaml / 本文件 / 包 README）必须同步。
ControlSettings load_control_settings(rclcpp::Node &node);

} // namespace uw
