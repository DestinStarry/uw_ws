#pragma once
#include "uw_robot_interfaces/robot_types.hpp"
#include <optional>
namespace uw {
std::optional<double> parse_depth(const std::string &line);
std::string encode_stm32(const Actuators &command);
} // namespace uw
