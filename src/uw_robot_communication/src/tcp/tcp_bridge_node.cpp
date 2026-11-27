/// @file tcp_bridge_node.cpp
/// @brief TCP 服务端节点：把手柄指令转成 control/cmd，并把遥测回传给客户端。
///
/// socket 生命周期、接包/拆包与发送队列上限在 ``tcp/tcp_server`` 中；
/// 本节点只做"文本协议 ↔ ROS 消息"的翻译与安全策略：
/// * 客户端断开、队列溢出或显式 OFF 一律发布急停（需要重新解锁）；
/// * 绝不允许任意 TCP 文本直接变成执行器输出；
/// * 遥测按固定抽稀比回传（姿态每 10 帧、深度每 5 帧），避免占满链路。

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/string.hpp>
#include <uw_robot_interfaces/msg/control_command.hpp>
#include <uw_robot_interfaces/msg/depth_state.hpp>
#include <uw_robot_interfaces/msg/robot_status.hpp>

#include <chrono>
#include <algorithm>
#include <stdexcept>
#include <cmath>
#include <sstream>

#include "uw_robot_communication/tcp/protocol.hpp"
#include "uw_robot_communication/tcp/tcp_server.hpp"
#include "uw_robot_communication/tcp/telemetry.hpp"

using namespace std::chrono_literals;
namespace msg = uw_robot_interfaces::msg;

class TcpBridge : public rclcpp::Node {
public:
  TcpBridge() : Node("tcp_bridge_node") {
    const auto host = declare_parameter<std::string>("bind_address", "0.0.0.0");
    const int port = declare_parameter("port", 8888);

    const double telemetry_hz = declare_parameter("telemetry_rate_hz", 1.0);
    cpu_timeout_ = declare_parameter("cpu_temp_timeout_s", 3.0);
    voltage_timeout_ = declare_parameter("voltage_timeout_s", 3.0);
    rpm_timeout_ = declare_parameter("rpm_timeout_s", 3.0);
    legacy_temp_ = declare_parameter("publish_legacy_temp", true);
    if (!std::isfinite(telemetry_hz) || telemetry_hz < .1 || telemetry_hz > 10 ||
        !std::isfinite(cpu_timeout_) || cpu_timeout_ <= 0 ||
        !std::isfinite(voltage_timeout_) || voltage_timeout_ <= 0 ||
        !std::isfinite(rpm_timeout_) || rpm_timeout_ <= 0)
      throw std::runtime_error("invalid auxiliary telemetry rate/timeout");
    pub_ = create_publisher<msg::ControlCommand>("control/cmd", 1);
    camera_pub_=create_publisher<std_msgs::msg::Float64>("camera/servo_target_deg",1);

    // 断线（对端关闭/写失败/队列溢出）统一翻译成急停。
    server_.set_disconnect_handler([this] { publish_stop(); });
    server_.set_line_handler([this](const std::string &line) { on_line(line); });
    server_.open(host, port);

    status_ = create_subscription<msg::RobotStatus>(
        "robot/status", 1, [this](msg::RobotStatus::ConstSharedPtr s) {
          // 同时保留旧 CLOSED_LOOP 字段，与现有 UI 的 CONTROL_MODE 协议兼容。
          reply("ARMED:" + std::to_string(s->armed) +
                "\nCLOSED_LOOP:" + std::to_string(s->closed_loop) +
                "\nCONTROL_MODE:" + (s->closed_loop ? "CLOSED" : "OPEN") +
                "\nDEPTH_HOLD:" + std::to_string(s->depth_hold) +
                "\nDEPTH_TARGET:" + std::to_string(s->depth_target_m) +
                "\nIMU_OK:" + std::to_string(s->imu_valid) + "\n");
        });
    depth_ = create_subscription<msg::DepthState>(
        "depth/state", 1, [this](msg::DepthState::ConstSharedPtr s) {
          if (++depth_count_ % 5 == 0)
            reply("DEPTH:" + std::to_string(s->depth_m) +
                  "\nDEPTH_VALID:" + std::to_string(s->valid) + "\n");
        });
    imu_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Imu::ConstSharedPtr s) {
          if (++imu_count_ % 10 != 0)
            return;
          reply_euler(s->orientation);
        });
    hardware_rx_ = create_subscription<std_msgs::msg::String>(
        "hardware/rx", 100, [this](std_msgs::msg::String::ConstSharedPtr message) {
          const auto field = uw::parse_voltage(message->data);
          if (field.present)
            voltage_ = {field.value, monotonic()};
          const auto fields = uw::parse_rpm(message->data);
          for (size_t i = 0; i < fields.size(); ++i)
            if (fields[i].present)
              rpm_[i] = {fields[i].value, monotonic()};
          // Only validated VOLT and CH0..CH3 are accepted, never ARMED/OFF/TEMP.
        });
    cpu_temperature_ = create_subscription<sensor_msgs::msg::Temperature>(
        "system/cpu_temperature", 1,
        [this](sensor_msgs::msg::Temperature::ConstSharedPtr message) {
          const double age = (now() - rclcpp::Time(message->header.stamp)).seconds();
          const double value = message->temperature;
          if (!std::isfinite(age) || age < -.05 || age > cpu_timeout_ ||
              !std::isfinite(value) || value < -100 || value > 200) {
            cpu_ = {std::nullopt, monotonic()};
            return;
          }
          cpu_ = {value, monotonic() - std::max(0., age)};
        });
    timer_ = create_wall_timer(5ms, [this] { server_.poll(); });
    telemetry_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1. / telemetry_hz)), [this] {
          reply(uw::encode_aux_telemetry(cpu_, voltage_, monotonic(), cpu_timeout_,
                                         voltage_timeout_, legacy_temp_) +
                uw::encode_rpm_telemetry(rpm_, monotonic(), rpm_timeout_));
        });
  }

private:
  static double monotonic() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  /// 断线与 OFF 都转成急停；不向执行器转发任意文本。
  void publish_stop() {
    msg::ControlCommand stop;
    stop.header.stamp = now();
    stop.sequence = sequence_++;
    stop.estop = true;
    pub_->publish(stop);
  }

  void reply(const std::string &value) { server_.queue(value); }

  /// 四元数转欧拉角（度）回传给 UI；非法/未归一化的样本直接丢弃。
  void reply_euler(const geometry_msgs::msg::Quaternion &q) {
    Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
    if (!quat.coeffs().allFinite() || quat.norm() < .9)
      return;
    quat.normalize();
    const double w = quat.w(), x = quat.x(), y = quat.y(), z = quat.z();
    const double roll =
        std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)) * 180 / uw::pi;
    const double pitch =
        std::asin(uw::clamp(2 * (w * y - z * x), -1., 1.)) * 180 / uw::pi;
    const double yaw =
        std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) * 180 / uw::pi;
    reply("ROLL:" + std::to_string(roll) + "\nPITCH:" + std::to_string(pitch) +
          "\nYAW:" + std::to_string(yaw) + "\n");
  }

  /// 一帧手柄文本：OFF 直接急停，其余交给协议解析。
  void on_line(const std::string &line) {
    if (line == "OFF" || line == "OFF\r") {
      publish_stop();
      return;
    }
    const auto command = uw::parse_command(line);
    if (!command)
      return;
    if (command->camera_command) {
      std_msgs::msg::Float64 camera;camera.data=command->camera_angle_deg;
      camera_pub_->publish(camera);
    }
    msg::ControlCommand m;
    m.header.stamp = now();
    m.sequence = sequence_++;
    for (int i = 0; i < 6; ++i)
      m.velocity[i] = command->velocity[i];
    m.arm = command->arm;
    m.estop = command->estop;
    m.depth_toggle = command->depth_toggle;
    m.loop_toggle = command->loop_toggle;
    pub_->publish(m);
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr camera_pub_;
  int depth_count_ = 0, imu_count_ = 0;
  uint32_t sequence_ = 0;
  uw::TcpServer server_;
  uw::TimedReading cpu_, voltage_;
  std::array<uw::TimedReading, 4> rpm_;
  double cpu_timeout_ = 3., voltage_timeout_ = 3., rpm_timeout_ = 3.;
  bool legacy_temp_ = true;
  rclcpp::TimerBase::SharedPtr timer_, telemetry_timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr hardware_rx_;
  rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr cpu_temperature_;
  rclcpp::Publisher<msg::ControlCommand>::SharedPtr pub_;
  rclcpp::Subscription<msg::RobotStatus>::SharedPtr status_;
  rclcpp::Subscription<msg::DepthState>::SharedPtr depth_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<TcpBridge>());
  } catch (const std::exception &e) {
    fprintf(stderr, "tcp_bridge_node: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
