/// @file tcp_bridge_node.cpp
/// @brief TCP 服务端节点：把手柄指令转成 control/cmd，并把遥测回传给客户端。
///
/// 线程模型：单线程、非阻塞 socket，跟随 ROS executor 周期轮询；
/// 安全策略：客户端断开、握手失败或 estop 帧一律发布急停，需要重新解锁；
/// 回传缓冲有上限，客户端不读会主动断开（避免拖垮控制链路）。

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <uw_robot_interfaces/msg/control_command.hpp>
#include <uw_robot_interfaces/msg/robot_status.hpp>
#include <uw_robot_interfaces/msg/depth_state.hpp>
#include "uw_robot_communication/tcp/protocol.hpp"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <cerrno>
#include <sstream>

using namespace std::chrono_literals;
namespace msg = uw_robot_interfaces::msg;
class TcpBridge : public rclcpp::Node {
public:
  TcpBridge() : Node("tcp_bridge_node") {
    auto host = declare_parameter<std::string>("bind_address", "0.0.0.0");
    int port = declare_parameter("port", 8888);
    pub_ = create_publisher<msg::ControlCommand>("control/cmd", 1);
    open_server(host, port);
    status_ = create_subscription<msg::RobotStatus>(
        "robot/status", 1, [this](msg::RobotStatus::ConstSharedPtr s) {
          // 同时保留旧 CLOSED_LOOP 字段，与现有 UI 的 CONTROL_MODE 文本协议兼容。
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
        "imu/data", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Imu::ConstSharedPtr s) {
          if (++imu_count_ % 10 != 0)
            return;
          const auto &q = s->orientation;
          Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
          if (!quat.coeffs().allFinite() || quat.norm() < .9)
            return;
          quat.normalize();
          double w = quat.w(), x = quat.x(), y = quat.y(), z = quat.z();
          double roll = std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)) * 180 / uw::pi;
          double pitch = std::asin(uw::clamp(2 * (w * y - z * x), -1., 1.)) * 180 / uw::pi;
          double yaw = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) * 180 / uw::pi;
          reply("ROLL:" + std::to_string(roll) + "\nPITCH:" + std::to_string(pitch) +
                "\nYAW:" + std::to_string(yaw) + "\n");
        });
    timer_ = create_wall_timer(5ms, [this] { tick(); });
  }
  ~TcpBridge() override {
    if (client_ >= 0)
      ::close(client_);
    if (server_ >= 0)
      ::close(server_);
  }

private:
  // 断线和 OFF 均转换为急停请求，不直接向执行器发送任意 TCP 文本。
  void publish_stop() {
    msg::ControlCommand stop;
    stop.header.stamp = now();
    stop.sequence = sequence_++;
    stop.estop = true;
    pub_->publish(stop);
  }
  // 非阻塞单客户端服务端；不在 ROS 回调中等待连接。
  void open_server(const std::string &host, int port) {
    server_ = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (server_ < 0)
      throw std::runtime_error("socket failed");
    int yes = 1;
    setsockopt(server_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (port < 1 || port > 65535 || inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1 ||
        bind(server_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0 ||
        listen(server_, 1) < 0) {
      ::close(server_);
      server_ = -1;
      throw std::runtime_error("invalid address or TCP bind/listen failed");
    }
  }
  void disconnect() {
    if (client_ >= 0)
      ::close(client_);
    client_ = -1;
    rx_.clear();
    tx_.clear();
    // Immediate stop plus controller-side timeout. Never turn an arbitrary TCP line into actuator
    // output.
    publish_stop();
  }
  void reply(const std::string &value) {
    if (client_ < 0)
      return;
    if (tx_.size() + value.size() > 8192) {
      disconnect();
      return;
    }
    tx_ += value;
  }
  // recv 的块不等于一帧；只有遇到换行才交给协议解析。
  void handle_line(const std::string &line) {
    if (line == "OFF" || line == "OFF\r") {
      publish_stop();
      return;
    }
    auto command = uw::parse_command(line);
    if (!command)
      return;
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
  // send 可只写出一部分；剩余内容保留到下一次 tick。
  void flush_reply() {
    if (client_ >= 0 && !tx_.empty()) {
      ssize_t n = send(client_, tx_.data(), tx_.size(), MSG_NOSIGNAL);
      if (n > 0)
        tx_.erase(0, static_cast<size_t>(n));
      else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
        disconnect();
    }
  }
  // 每 5 ms 最多读取 4 批，避免输入洪泛占满单线程执行器。
  void tick() {
    if (client_ < 0) {
      client_ = accept4(server_, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
      if (client_ < 0)
        return;
      rx_.clear();
      tx_.clear();
    }
    char bytes[1024];
    for (int batch = 0; batch < 4; ++batch) {
      ssize_t n = recv(client_, bytes, sizeof(bytes), 0);
      if (n == 0) {
        disconnect();
        return;
      }
      if (n < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
          disconnect();
        break;
      }
      rx_.append(bytes, static_cast<size_t>(n));
      size_t index;
      while ((index = rx_.find('\n')) != std::string::npos) {
        std::string line = rx_.substr(0, index);
        rx_.erase(0, index + 1);
        handle_line(line);
      }
      if (rx_.size() > 2048) {
        disconnect();
        return;
      }
    }
    flush_reply();
  }
  int server_ = -1, client_ = -1, depth_count_ = 0, imu_count_ = 0;
  uint32_t sequence_ = 0;
  std::string rx_, tx_;
  rclcpp::TimerBase::SharedPtr timer_;
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
