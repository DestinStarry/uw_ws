#include "uw_robot_control/allocation/allocator.hpp"
#include <Eigen/Cholesky>
#include <Eigen/QR>
#include <limits>
#include <stdexcept>
#include <vector>

namespace uw {
// 误差权重顺序 [Fx,Fy,Fz,Mx,My,Mz]，优先保证姿态力矩。
static V6 weights() {
  V6 w;
  w << 1, 1, 4, 24, 24, 24;
  return w;
}
// 列顺序为 [1上、3下、2右、4左]，对应执行器消息的内部推力顺序。
// 每列 = [推力单位方向；相对质心力臂 × 推力方向]。
M64 wrench_matrix(const V2 &angles, const AllocationGeometry &geometry) {
  M64 matrix;
  V3 com = geometry.com;
  auto mounts = geometry.sites;
  std::array<V3, 4> dirs{V3(std::cos(angles[0]), std::sin(angles[0]), 0),
                         V3(std::cos(angles[0]), -std::sin(angles[0]), 0),
                         V3(std::cos(angles[1]), 0, std::sin(angles[1])),
                         V3(std::cos(angles[1]), 0, std::sin(angles[1]))};
  if (geometry.rotating_sites) {
    for (int i = 0; i < 4; ++i) {
      const Eigen::Matrix3d rotation =
          Eigen::AngleAxisd(angles[i < 2 ? 0 : 1], geometry.axes[i]).toRotationMatrix();
      dirs[i] = rotation * V3::UnitX();
      mounts[i] = geometry.hinges[i] + rotation * (geometry.sites[i] - geometry.hinges[i]);
      // 小质量转动组件同样改变整机质心，不能直接沿用旧 CAD 的固定质心。
      com += geometry.mass_fractions[i] *
             (rotation * geometry.child_centers[i] - geometry.child_centers[i]);
    }
  }
  for (int i = 0; i < 4; ++i) {
    matrix.col(i).head<3>() = dirs[i];
    matrix.col(i).tail<3>() = (mounts[i] - com).cross(dirs[i]);
  }
  return matrix;
}
V6 forward(const Actuators &a, const AllocationGeometry &geometry) {
  return wrench_matrix(a.angles, geometry) * a.forces;
}

static V2 polar(double x, double y) {
  double r = std::hypot(x, y), angle = std::atan2(y, x);
  if (r <= 1e-8)
    return V2::Zero();
  if (std::abs(y) > 1e-8 && std::abs(x) <= .2 * std::abs(y))
    return V2(std::abs(y), std::copysign(servo_limit, y));
  if (angle > servo_limit)
    return V2(-r, angle - pi);
  if (angle < -servo_limit)
    return V2(-r, angle + pi);
  return V2(r, angle);
}
// 理想对称几何的解析逆：用于诊断，不替代实际几何下的约束分配。
Allocation analytic_inverse(const V6 &t) {
  if (!t.allFinite())
    throw std::invalid_argument("nonfinite inverse input");
  const double h = .123, d = .223 - .01336829;
  const V2 q = polar((t[4] - d * t[2]) / h, t[1]);
  double s = std::sin(q[1]), c = std::cos(q[1]);
  double a = (t[5] + d * t[1]) / h, b = -t[3] / h;
  double c2 = c * s, c1 = -t[0] * s - c * b, c0 = t[0] * b - t[2] * a;
  std::vector<double> roots;
  if (std::abs(c2) > 1e-8) {
    double disc = c1 * c1 - 4 * c2 * c0;
    if (disc < -1e-9)
      throw std::runtime_error("unreachable analytic target");
    double v = std::sqrt(std::max(0., disc));
    roots = {(-c1 + v) / (2 * c2), (-c1 - v) / (2 * c2)};
  } else if (std::abs(c1) > 1e-8)
    roots = {-c0 / c1};
  else if (std::abs(c0) <= 1e-9)
    roots = {std::abs(c) > 1e-8 ? .5 * t[0] / c : 0., 0.};
  else
    throw std::runtime_error("degenerate inverse");
  Allocation best;
  double score = std::numeric_limits<double>::infinity();
  for (double p : roots) {
    double xs = t[0] - p * c, yd = b - p * s, sum, diff, theta;
    if (std::hypot(xs, t[2]) > 1e-8 || std::hypot(a, yd) <= 1e-8) {
      V2 v = polar(xs, t[2]);
      sum = v[0];
      theta = v[1];
      diff = std::abs(std::cos(theta)) > 1e-8 ? a / std::cos(theta) : yd / std::sin(theta);
    } else {
      V2 v = polar(a, yd);
      diff = v[0];
      theta = v[1];
      sum = 0;
    }
    Allocation out;
    out.angles = V2(q[1], theta);
    out.forces << .5 * (p + q[0]), .5 * (p - q[0]), .5 * (sum + diff), .5 * (sum - diff);
    out.achieved = wrench_matrix(out.angles) * out.forces;
    double error = (out.achieved - t).cwiseAbs().maxCoeff();
    double rank = error + 1e-8 * out.forces.squaredNorm();
    if (rank < score) {
      score = rank;
      best = out;
    }
  }
  best.ok = (best.achieved - t).cwiseAbs().maxCoeff() <= 2e-3 &&
            best.forces.minCoeff() >= reverse_limit && best.forces.maxCoeff() <= forward_limit;
  if (!best.ok)
    throw std::runtime_error("inverse outside limits or geometry residual");
  return best;
}

// 固定角度下的有界二次规划：跟踪目标 + 推力正则 + 相对上周期变化惩罚。
Allocation fixed_allocate(const V2 &angles, const V6 &target, const V4 &reference, double step,
                          const AllocationGeometry &geometry) {
  if (!angles.allFinite() || !target.allFinite() || !reference.allFinite() || !std::isfinite(step))
    throw std::invalid_argument("nonfinite allocation input");
  V4 ref = bounded<V4>(reference, reverse_limit, forward_limit);
  V4 lower = V4::Constant(reverse_limit), upper = V4::Constant(forward_limit);
  if (step > 0) {
    lower = lower.cwiseMax(ref - V4::Constant(step));
    upper = upper.cwiseMin(ref + V4::Constant(step));
  }
  M64 matrix = wrench_matrix(angles, geometry), a = weights().asDiagonal() * matrix;
  Eigen::Matrix4d h = a.transpose() * a + (.03 * .03 + .025 * .025) * Eigen::Matrix4d::Identity();
  V4 g = a.transpose() * weights().asDiagonal() * target + .025 * .025 * ref;
  // Enumerate 3^4 active sets. Small, deterministic, exact box-QP KKT solve.
  // Equivalent objective to Python BVLS; avoids an extra optimizer dependency.
  double best_score = std::numeric_limits<double>::infinity();
  V4 best = V4::Zero();
  bool found = false;
  for (int code = 0; code < 81; ++code) {
    int state = code;
    std::vector<int> free;
    V4 f = V4::Zero();
    std::array<int, 4> active{};
    for (int i = 0; i < 4; ++i) {
      active[i] = state % 3;
      state /= 3;
      if (active[i] == 0)
        free.push_back(i);
      else
        f[i] = active[i] == 1 ? lower[i] : upper[i];
    }
    int n = static_cast<int>(free.size());
    if (n) {
      Eigen::MatrixXd hh(n, n);
      Eigen::VectorXd rhs(n);
      for (int i = 0; i < n; ++i) {
        rhs[i] = g[free[i]] - h.row(free[i]).dot(f);
        for (int j = 0; j < n; ++j)
          hh(i, j) = h(free[i], free[j]);
      }
      Eigen::VectorXd x = hh.ldlt().solve(rhs);
      for (int i = 0; i < n; ++i)
        f[free[i]] = x[i];
    }
    if (!f.allFinite() || (f.array() < lower.array() - 1e-7).any() ||
        (f.array() > upper.array() + 1e-7).any())
      continue;
    V4 grad = h * f - g;
    bool kkt = true;
    for (int i = 0; i < 4; ++i) {
      if ((active[i] == 1 && grad[i] < -1e-7) || (active[i] == 2 && grad[i] > 1e-7))
        kkt = false;
    }
    if (!kkt)
      continue;
    double score = .5 * f.dot(h * f) - g.dot(f);
    if (score < best_score) {
      best_score = score;
      best = f;
      found = true;
    }
  }
  if (!found)
    throw std::runtime_error("box QP failed");
  Allocation out;
  out.forces = best.cwiseMax(lower).cwiseMin(upper);
  out.angles = angles;
  out.achieved = matrix * out.forces;
  out.ok = true;
  return out;
}

// 22 维残差：六维目标、角度/角度变化、推力/推力变化、接近推力饱和惩罚。
static Eigen::Matrix<double, 22, 1> residual(const V2 &angles, const Request &r,
                                             const AllocationGeometry &geometry) {
  auto allocation = fixed_allocate(angles, r.target, r.forces, -1, geometry);
  const V4 f = allocation.forces;
  Eigen::Matrix<double, 22, 1> out;
  out.head<6>() = weights().asDiagonal() * (allocation.achieved - r.target);
  out.segment<2>(6) = .05 * angles;
  out.segment<2>(8) = .10 * (angles - r.angles);
  out.segment<4>(10) = .03 * f;
  out.segment<4>(14) = .025 * (f - r.forces);
  for (int i = 0; i < 4; ++i)
    out[18 + i] = 2. * std::max(0., f[i] >= 0 ? f[i] - .8 * forward_limit
                                              : -f[i] - .8 * std::abs(reverse_limit));
  return out;
}
// 以上周期角度和目标方向分别起步，有限差分迭代后选择残差较小的结果。
Plan plan_angles(const Request &r, const AllocationGeometry &geometry, double min_improvement) {
  if (!r.target.allFinite() || !r.angles.allFinite() || !r.forces.allFinite())
    throw std::invalid_argument("invalid plan request");
  V2 ref = bounded<V2>(r.angles, -servo_limit, servo_limit);
  V2 lo = (ref - V2::Constant(servo_limit)).cwiseMax(-servo_limit);
  V2 hi = (ref + V2::Constant(servo_limit)).cwiseMin(servo_limit);
  V2 direction(std::abs(r.target[1]) > .1 ? std::copysign(servo_limit, r.target[1]) : 0,
               std::abs(r.target[2]) > .1 ? std::copysign(servo_limit, r.target[2]) : 0);
  if (std::abs(r.target[3]) > .05)
    direction[0] = servo_limit;
  Plan best;
  double cost = std::numeric_limits<double>::infinity();
  for (V2 x : {ref, direction}) {
    x = x.cwiseMax(lo).cwiseMin(hi);
    double damping = .001;
    for (int iteration = 0; iteration < 18; ++iteration) {
      auto e = residual(x, r, geometry);
      Eigen::Matrix<double, 22, 2> j;
      for (int axis = 0; axis < 2; ++axis) {
        V2 plus = x, minus = x;
        plus[axis] = std::min(hi[axis], x[axis] + 1e-5);
        minus[axis] = std::max(lo[axis], x[axis] - 1e-5);
        j.col(axis) = (residual(plus, r, geometry) - residual(minus, r, geometry)) /
                      (plus[axis] - minus[axis]);
      }
      Eigen::Matrix2d h = j.transpose() * j + damping * Eigen::Matrix2d::Identity();
      V2 delta = -h.ldlt().solve(j.transpose() * e);
      if (!delta.allFinite())
        break;
      V2 trial = (x + delta).cwiseMax(lo).cwiseMin(hi);
      if (residual(trial, r, geometry).squaredNorm() < e.squaredNorm()) {
        x = trial;
        damping = std::max(1e-9, damping * .3);
        if (delta.norm() < 1e-8)
          break;
      } else
        damping = std::min(1e12, damping * 10.);
    }
    double score = residual(x, r, geometry).squaredNorm();
    if (score < cost) {
      cost = score;
      best.angles = x;
    }
  }
  // 同一目标先比较保持当前名义角度的代价。微小噪声收益不足时保持角度，
  // 仍由 50Hz 固定角度分配纠正推力，避免低推力/近零空间中的舵机来回选解。
  if (!std::isfinite(min_improvement) || min_improvement < 0)
    throw std::invalid_argument("invalid angle-plan improvement threshold");
  if (min_improvement > 0 && std::isfinite(cost)) {
    const double current_cost = residual(ref, r, geometry).squaredNorm();
    if (current_cost - cost <= min_improvement)
      best.angles = ref;
  }
  // 低推力组的角度接近不可观测：噪声可能让等价解在大角度间漂移。
  // 保留当前角度，但必须检查不会损失有意义的六维跟踪能力。
  if (min_improvement > 0 && std::isfinite(cost)) {
    const auto current = fixed_allocate(ref, r.target, r.forces, -1, geometry);
    const auto candidate = fixed_allocate(best.angles, r.target, r.forces, -1, geometry);
    V2 proposed = best.angles;
    for (int group = 0; group < 2; ++group) {
      if (current.forces.segment<2>(2 * group).cwiseAbs().maxCoeff() < .2 &&
          candidate.forces.segment<2>(2 * group).cwiseAbs().maxCoeff() < .2 &&
          std::abs(r.target[group + 1]) < .1 && std::abs(r.target[3]) < .02)
        proposed[group] = ref[group];
    }
    const auto held = fixed_allocate(proposed, r.target, r.forces, -1, geometry);
    const double candidate_error =
        (weights().asDiagonal() * (candidate.achieved - r.target)).squaredNorm();
    const double held_error = (weights().asDiagonal() * (held.achieved - r.target)).squaredNorm();
    if (held_error <= candidate_error + min_improvement)
      best.angles = proposed;
  }
  best.target = r.target;
  best.started = r.started;
  best.epoch = r.epoch;
  best.valid = best.angles.allFinite() && std::isfinite(cost);
  return best;
}
} // namespace uw
