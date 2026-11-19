/// @file allocator.hpp
/// @brief 舵角规划器接口：搜索两组倾转角，使给定 wrench 的加权误差最小。
///
/// 纯计算、无 ROS、无锁，可在单元测试中直接调用。

#pragma once
#include "uw_robot_interfaces/robot_types.hpp"

namespace uw {
// 默认沿用实物固定作用点模型；仿真由 XML 提供旋转铰点、质心和组件质量。
// 所有位置均在机体系，长度 m，mass_fractions 为组件质量/整机质量。
struct AllocationGeometry {
  bool rotating_sites = false;
  V3 com{-.01336829, .00002950, -.00000930};
  std::array<V3, 4> sites{V3(-.223, 0, .123), V3(-.223, 0, -.123), V3(-.223, -.123, 0),
                          V3(-.223, .123, 0)};
  std::array<V3, 4> hinges{V3::Zero(), V3::Zero(), V3::Zero(), V3::Zero()};
  std::array<V3, 4> axes{V3::UnitZ(), -V3::UnitZ(), -V3::UnitY(), -V3::UnitY()};
  std::array<V3, 4> child_centers{V3::Zero(), V3::Zero(), V3::Zero(), V3::Zero()};
  V4 mass_fractions = V4::Zero();
};
struct Plan {
  V2 angles = V2::Zero();
  V6 target = V6::Zero();
  double started = 0, ms = 0;
  uint64_t epoch = 0;
  bool valid = false;
};
struct Request {
  V6 target = V6::Zero();
  V2 angles = V2::Zero();
  V4 forces = V4::Zero();
  double started = 0;
  uint64_t epoch = 0;
};
struct Allocation {
  V4 forces = V4::Zero();
  V2 angles = V2::Zero();
  V6 achieved = V6::Zero();
  bool ok = false;
};

M64 wrench_matrix(const V2 &angles, const AllocationGeometry &geometry = {});
V6 forward(const Actuators &a, const AllocationGeometry &geometry = {});
// Ideal symmetric inverse is a seed/diagnostic, never a replacement for actual geometry.
Allocation analytic_inverse(const V6 &target);
Allocation fixed_allocate(const V2 &angles, const V6 &target, const V4 &reference = V4::Zero(),
                          double force_step = -1, const AllocationGeometry &geometry = {});
Plan plan_angles(const Request &request, const AllocationGeometry &geometry = {},
                 double min_improvement = 0);
} // namespace uw
