/// @file protocol.hpp
/// @brief TCP 文本协议：解析手柄指令帧、格式化遥测帧。
///
/// 帧格式（发送端 Windows UI 生成）：
///   CMD:u,v,w,p,q,r,arm,estop,depth,loop[,camera_deg]
/// 解析失败（字段数不符、非有限值、超出量程）时整帧丢弃，不做部分采纳；
/// 量程只做防呆，真实满杆与曲线由发送端和 robot.yaml 决定。

#pragma once
#include "uw_robot_interfaces/robot_types.hpp"
#include <optional>
namespace uw {
// 解析 "CMD:u,v,w,p,q,r,arm,estop,depth,loop[,camera_deg]"；非法帧返回空。
std::optional<Command> parse_command(const std::string &line);
}
