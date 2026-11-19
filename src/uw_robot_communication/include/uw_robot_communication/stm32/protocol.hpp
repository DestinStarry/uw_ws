/// @file protocol.hpp
/// @brief STM32 串口文本协议：执行器帧格式化、遥测与深度行解析。
///
/// 下行：CTRL/OFF 等以换行结尾的 ASCII 帧；
/// 上行：兼容独立 DEPTH 字段与混合遥测中的 Depth:xx 字段。

#pragma once
#include "uw_robot_interfaces/robot_types.hpp"
#include <optional>
namespace uw {
std::optional<double> parse_depth(const std::string &line);
std::string encode_stm32(const Actuators &command);
} // namespace uw
