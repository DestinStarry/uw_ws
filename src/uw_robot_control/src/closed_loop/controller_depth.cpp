/// @file controller_depth.cpp
/// @brief 定深闭环（Controller::apply_depth_hold）。
///
/// 深度向下为正；世界系竖直力转换到机体系后叠加到平移力。
/// 松杆或深度恢复有效时重新捕获目标深度，避免积分带着旧偏差。

#include "uw_robot_control/closed_loop/controller.hpp"
namespace uw {
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

} // namespace uw
