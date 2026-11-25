/// @file hardware_node.cpp
/// @brief STM32 串口节点：下发执行器指令，接收遥测与（可选的）深度。
///
/// 串口读写、半帧续写与洪泛限流在 ``stm32/serial_port`` 中；本节点只做
/// "ROS 消息 ↔ 文本协议"的翻译与安全策略：
/// * 指令帧时间戳过期、数值越界或 ``enabled=false`` 一律翻译为 ``OFF``；
/// * 写失败或超过 100 ms 写不出去立即停机（抛异常由 main 处理）；
/// * 深度计改为 Orin I2C 直连后，``publish_depth=false`` 时本节点不再发布
///   ``depth/state``，避免与 ``depth_node`` 的失效帧互相覆盖。

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <uw_robot_interfaces/msg/depth_state.hpp>
#include <uw_robot_interfaces/msg/thruster_command.hpp>

#include <chrono>
#include <stdexcept>

#include "uw_robot_communication/stm32/protocol.hpp"
#include "uw_robot_communication/stm32/serial_port.hpp"

using namespace std::chrono_literals;
namespace msg = uw_robot_interfaces::msg;

namespace {

double monotonic() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

} // namespace

class HardwareNode : public rclcpp::Node {
public:
  HardwareNode() : Node("hardware_node") {
    dry_ = declare_parameter("dry_run", true);
    device_ = declare_parameter<std::string>("port", "/dev/ttyTHS2");
    const int baud = declare_parameter("baud", 115200);
    timeout_ = declare_parameter("command_timeout_s", .15);
    depth_timeout_ = declare_parameter("depth_timeout_s", 1.5);
    publish_depth_ = declare_parameter("publish_depth", true);
    if (baud != 115200 || timeout_ <= 0 || depth_timeout_ <= 0)
      throw std::runtime_error("requires 115200 baud and positive timeouts");

    tx_pub_ = create_publisher<std_msgs::msg::String>("hardware/tx", 1);
    rx_pub_ = create_publisher<std_msgs::msg::String>("hardware/rx", 1);
    if (publish_depth_)
      depth_pub_ = create_publisher<msg::DepthState>("depth/state", 1);

    // 时间戳与物理范围先校验；非法指令立即把输出拉回 OFF。
    sub_ = create_subscription<msg::ThrusterCommand>(
        "actuators/command", 1,
        [this](msg::ThrusterCommand::ConstSharedPtr m) {
          const double age = (now() - rclcpp::Time(m->header.stamp)).seconds();
          if (age < -.05 || age > timeout_)
            return;
          uw::Actuators a;
          a.enabled = m->enabled;
          a.gripper = m->gripper_closed;
          for (int i = 0; i < 4; ++i)
            a.forces[i] = m->thrust_newtons[i];
          a.angles = uw::V2(m->theta12_rad, m->theta34_rad);
          if (!a.forces.allFinite() || !a.angles.allFinite() ||
              a.forces.minCoeff() < uw::reverse_limit - 1e-6 ||
              a.forces.maxCoeff() > uw::forward_limit + 1e-6 ||
              a.angles.cwiseAbs().maxCoeff() > uw::servo_limit + 1e-6) {
            latest_ = "OFF\n";
            last_command_ = -1e9;
            return;
          }
          // disabled 帧天然就是停机帧，encode_stm32 会输出 OFF。
          latest_ = uw::encode_stm32(a);
          last_command_ = monotonic();
        });

    if (!dry_)
      serial_.open(device_);
    timer_ = create_wall_timer(20ms, [this] { tick(); });
    RCLCPP_INFO(get_logger(), "STM32 %s",
                dry_ ? "dry_run (serial unopened)" : device_.c_str());
  }

private:
  /// 遥测行：先原样转发（诊断用），再尝试解析深度。
  void on_line(const std::string &line) {
    std_msgs::msg::String raw;
    raw.data = line;
    rx_pub_->publish(raw);

    const auto value = uw::parse_depth(line);
    if (!value)
      return;
    const double t = monotonic(), dt = t - depth_time_;
    if (depth_time_ > 0 && dt > .3 && dt < depth_timeout_)
      rate_ += .2 * ((*value - depth_) / dt - rate_);
    else
      rate_ = 0;
    depth_ = *value;
    depth_time_ = t;
  }

  /// 无新数据也发布无效帧：接收方据此判失效，不能把旧深度当新采样。
  void publish_depth() {
    msg::DepthState d;
    d.header.stamp = now();
    d.header.frame_id = "base_link";
    d.depth_m = depth_;
    d.depth_rate_mps = rate_;
    d.data_age_s = depth_time_ > 0 ? std::max(0., monotonic() - depth_time_) : 1e9;
    d.valid = depth_time_ > 0 && d.data_age_s <= depth_timeout_;
    depth_pub_->publish(d);
  }

  void tick() {
    const double t = monotonic();
    const std::string wanted = t - last_command_ <= timeout_ ? latest_ : "OFF\n";

    std_msgs::msg::String tx;
    tx.data = wanted;
    tx_pub_->publish(tx);

    if (serial_.is_open()) {
      serial_.write_line(wanted, t);
      serial_.read_lines([this](const std::string &line) { on_line(line); });
    }
    if (publish_depth_)
      publish_depth();
  }

  bool dry_ = true;
  bool publish_depth_ = true;
  std::string device_, latest_ = "OFF\n";
  double timeout_ = .15, depth_timeout_ = 1.5;
  double last_command_ = -1e9, depth_time_ = -1, depth_ = 0, rate_ = 0;

  uw::SerialPort serial_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<msg::ThrusterCommand>::SharedPtr sub_;
  rclcpp::Publisher<msg::DepthState>::SharedPtr depth_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tx_pub_, rx_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<HardwareNode>());
  } catch (const std::exception &e) {
    fprintf(stderr, "hardware_node: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
