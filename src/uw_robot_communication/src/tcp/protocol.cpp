/// @file protocol.cpp
/// @brief TCP 文本协议实现（见同名头文件的帧格式说明）。
///
/// 解析与格式化都保持纯函数：不碰 socket、不碰 ROS，便于单元测试。

#include "uw_robot_communication/tcp/protocol.hpp"
#include <cmath>
#include <locale>
#include <sstream>
#include <vector>

namespace uw {
namespace {
// 防呆上限：只用于拒绝明显算错的帧，真实量程由发送端和 robot.yaml 决定。
constexpr double max_linear_mps = 1.5;
constexpr double max_angular_rad_s = 1.5;
} // namespace

// CMD: 六个物理量（机体系线速度与角速度）+ 四个按钮；按钮兼容 0/1 和 0.0/1.0。
// 使用固定小数点语言环境；字段数不符、非有限值或越界时整帧丢弃，不截断，
// 交给控制节点按指令超时停推。
std::optional<Command> parse_command(const std::string &line) {
  if (line.rfind("CMD:", 0) != 0 || line.size() > 256)
    return std::nullopt;
  std::stringstream stream(line.substr(4));
  stream.imbue(std::locale::classic());
  std::vector<double> v;
  std::string token;
  while (std::getline(stream, token, ',')) {
    std::istringstream s(token);
    s.imbue(std::locale::classic());
    double x;
    if (!(s >> x) || !std::isfinite(x))
      return std::nullopt;
    s >> std::ws;
    if (!s.eof())
      return std::nullopt;
    v.push_back(x);
  }
  if ((v.size() != 10 && v.size() != 11) || line.back() == ',')
    return std::nullopt;
  for (int i = 0; i < 3; ++i)
    if (std::abs(v[i]) > max_linear_mps)
      return std::nullopt;
  for (int i = 3; i < 6; ++i)
    if (std::abs(v[i]) > max_angular_rad_s)
      return std::nullopt;
  for (int i = 6; i < 10; ++i)
    if (v[i] != 0 && v[i] != 1)
      return std::nullopt;
  if (v.size()==11 && std::abs(v[10])>90) return std::nullopt;
  Command c;
  c.camera_command=v.size()==11;
  if (c.camera_command) c.camera_angle_deg=v[10];
  for (int i = 0; i < 6; ++i)
    c.velocity[i] = v[i];
  c.arm = v[6];
  c.estop = v[7];
  c.depth_toggle = v[8];
  c.loop_toggle = v[9];
  return c;
}
} // namespace uw
