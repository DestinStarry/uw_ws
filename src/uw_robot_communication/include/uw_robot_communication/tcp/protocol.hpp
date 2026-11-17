#pragma once
#include "uw_robot_interfaces/robot_types.hpp"
#include <optional>
namespace uw {
// 解析 "CMD:u,v,w,p,q,r,arm,estop,depth,loop"；非法帧返回空。
std::optional<Command> parse_command(const std::string &line);
}
