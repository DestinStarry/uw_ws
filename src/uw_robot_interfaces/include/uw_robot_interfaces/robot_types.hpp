#pragma once
/// @file robot_types.hpp
/// @brief 纯 C++ 算法数据类型（不依赖 ROS 运行时）。
///
/// 约定
/// ====
/// * 力 N、力矩 N·m、角度 rad、线速度 m/s、角速度 rad/s；
/// * 三维向量按 (x, y, z) 排列，机体轴符合 ROS REP-103：
///   +X 前进、+Y 左舷、+Z 向上；p 右滚、q 低头、r 左转为正；
/// * 本头文件只放 POD 与常量，不含节点、串口、socket 逻辑，
///   因此可以在单元测试里直接构造（control_test / allocation_test 就是这样用的）。

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>

namespace uw {

// ---------------------------------------------------------------------------
// 基础类型与常量
// ---------------------------------------------------------------------------
using V2 = Eigen::Vector2d;
using V3 = Eigen::Vector3d;
using V4 = Eigen::Vector4d;
using V6 = Eigen::Matrix<double, 6, 1>;
using M64 = Eigen::Matrix<double, 6, 4>;

constexpr double pi = 3.14159265358979323846;
/// 舵角机械限位（rad）；分配器与规划器共用。
constexpr double servo_limit = pi / 2.0;
/// 单推进器推力上下限（N），与执行器模型一致：前向 33 N、反向 24.5 N。
constexpr double reverse_limit = -24.5;
constexpr double forward_limit = 33.0;

/// 标量限幅；NaN 会原样传出（调用方需自行检查有限性）。
inline double clamp(double x, double lo, double hi) {
  return std::max(lo, std::min(hi, x));
}

/// 逐元素限幅。
template <class V> V bounded(const V &x, double lo, double hi) {
  return x.cwiseMax(lo).cwiseMin(hi);
}

// ---------------------------------------------------------------------------
// 输入侧
// ---------------------------------------------------------------------------

/// 控制指令：发送端已完成轴映射与整形，控制侧只做六轴速度跟踪与离散命令仲裁。
/// 单位：线速度 m/s（+X 前进、+Y 左舷、+Z 上浮），角速度 rad/s（REP-103）。
struct Command {
  V6 velocity = V6::Zero();
  bool arm = false;          ///< 解锁/上锁切换，边沿有效。
  bool estop = false;        ///< 急停，电平有效且优先于其他输入。
  bool depth_toggle = false; ///< 定深切换，边沿有效。
  bool loop_toggle = false;  ///< 未解锁时切换开环/闭环，边沿有效。

  /// 六轴速度是否全部有限；协议层与控制器都用它拒绝 NaN/Inf 帧。
  bool finite() const {
    for (int i = 0; i < 6; ++i)
      if (!std::isfinite(velocity[i]))
        return false;
    return true;
  }
};

/// IMU 采样：姿态四元数 + 机体系角速度 + 有效性。
struct Imu {
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  V3 gyro = V3::Zero();
  bool valid = false;
};

/// 深度采样：相对深度（m，向下为正）与变化率（m/s）。
struct Depth {
  double value = 0, rate = 0;
  bool valid = false;
};

// ---------------------------------------------------------------------------
// 输出侧
// ---------------------------------------------------------------------------

/// 执行器目标。推力顺序 [1 上, 3 下, 2 右, 4 左]；两角度对应 1/3、2/4 倾转组。
struct Actuators {
  V4 forces = V4::Zero();
  V2 angles = V2::Zero();
  bool enabled = false, gripper = false;
};

/// 控制节点输出的机体系六维目标及其会话信息。
/// session/epoch 用于让分配器丢弃过期的异步规划结果。
struct WrenchTarget {
  V6 wrench = V6::Zero();
  bool enabled = false, closed_loop = true, center = false;
  uint64_t session = 0, epoch = 0;
  bool recenter = false; ///< 松杆回中阶段：直接零角度、零推力。
};

} // namespace uw
