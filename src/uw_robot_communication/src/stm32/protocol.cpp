#include "uw_robot_communication/stm32/protocol.hpp"
#include <iomanip>
#include <locale>
#include <sstream>
#include <vector>

namespace uw {
// 接受独立 DEPTH 字段和 STM32 混合遥测中的 Depth:xxm。
// 此处只解析深度；其它电压、温度等字段仍由原始遥测话题保留。
std::optional<double> parse_depth(const std::string &line) {
  std::istringstream stream(line);
  std::string token;
  while (stream >> token) {
    auto index = token.find(':');
    if (index == std::string::npos)
      continue;
    std::string key = token.substr(0, index);
    if (key != "DEPTH" && key != "Depth")
      continue;
    std::string value = token.substr(index + 1);
    if (!value.empty() && (value.back() == 'm' || value.back() == 'M'))
      value.pop_back();
    std::istringstream s(value);
    s.imbue(std::locale::classic());
    double x;
    if (!(s >> x) || !std::isfinite(x))
      return std::nullopt;
    s >> std::ws;
    if (!s.eof())
      return std::nullopt;
    return x;
  }
  return std::nullopt;
}
// 内部推力 [1上,3下,2右,4左] -> 串口物理顺序 [1,2,3,4]。
// 正反转最大推力不同，分别归一化；舵机角度除以 pi/2。
std::string encode_stm32(const Actuators &a) {
  if (!a.enabled || !a.forces.allFinite() || !a.angles.allFinite())
    return "OFF\n";
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << "CTRL:" << std::fixed << std::setprecision(4);
  const std::array<int, 4> physical{0, 2, 1, 3};
  for (int index : physical) {
    double f = a.forces[index];
    out << clamp(f / (f >= 0 ? forward_limit : std::abs(reverse_limit)), -1., 1.) << ',';
  }
  out << clamp(a.angles[0] / servo_limit, -1., 1.) << ','
      << clamp(a.angles[1] / servo_limit, -1., 1.) << ',';
  out << (a.gripper ? .5 : -.5) << '\n';
  return out.str();
}
} // namespace uw
