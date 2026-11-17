#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <string>

// 纯 C++ 算法数据类型，不包含 ROS 节点或串口逻辑。
// 力 N、力矩 N·m、角度 rad；三维向量按 xyz 排列。
namespace uw {
using V2 = Eigen::Vector2d;
using V3 = Eigen::Vector3d;
using V4 = Eigen::Vector4d;
using V6 = Eigen::Matrix<double, 6, 1>;
using M64 = Eigen::Matrix<double, 6, 4>;
constexpr double pi = 3.14159265358979323846;
constexpr double servo_limit = pi / 2.0;
constexpr double reverse_limit = -24.5;
constexpr double forward_limit = 33.0;
inline double clamp(double x, double lo, double hi) {
  return std::max(lo, std::min(hi, x));
}
template <class V> V bounded(const V &x, double lo, double hi) {
  return x.cwiseMax(lo).cwiseMin(hi);
}
// 发送端已完成轴映射与整形；控制侧只做六轴速度跟踪和离散命令仲裁。
// 单位：线速度 m/s（+X 前进、+Y 左舷、+Z 上浮为正），角速度 rad/s（REP-103 右手系）。
struct Command {
  V6 velocity = V6::Zero();
  bool arm = false;          // 解锁/上锁切换，边沿有效。
  bool estop = false;        // 急停，电平有效且优先于其他输入。
  bool depth_toggle = false; // 定深切换，边沿有效。
  bool loop_toggle = false;  // 未解锁时切换开环/闭环，边沿有效。
  bool finite() const {
    for (int i = 0; i < 6; ++i)
      if (!std::isfinite(velocity[i]))
        return false;
    return true;
  }
};
struct Imu {
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  V3 gyro = V3::Zero();
  bool valid = false;
};
struct Depth {
  double value = 0, rate = 0;
  bool valid = false;
};
// 推力顺序 [1上,3下,2右,4左]，两角度对应 1/3、2/4 倾转组。
struct Actuators {
  V4 forces = V4::Zero();
  V2 angles = V2::Zero();
  bool enabled = false, gripper = false;
};
struct WrenchTarget {
  V6 wrench = V6::Zero();
  bool enabled = false, closed_loop = true, center = false;
  uint64_t session = 0, epoch = 0;
  bool recenter = false; // 松杆回中阶段：直接零角度、零推力。
};
} // namespace uw
