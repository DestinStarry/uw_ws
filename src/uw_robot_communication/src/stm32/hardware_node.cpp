#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <uw_robot_interfaces/msg/thruster_command.hpp>
#include <uw_robot_interfaces/msg/depth_state.hpp>
#include "uw_robot_communication/stm32/protocol.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>
#include <chrono>
#include <stdexcept>

using namespace std::chrono_literals;
namespace msg = uw_robot_interfaces::msg;
static double monotonic() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
class HardwareNode : public rclcpp::Node {
public:
  HardwareNode() : Node("hardware_node") {
    dry_ = declare_parameter("dry_run", true);
    device_ = declare_parameter<std::string>("port", "/dev/ttyAS5");
    int baud = declare_parameter("baud", 115200);
    timeout_ = declare_parameter("command_timeout_s", .15);
    depth_timeout_ = declare_parameter("depth_timeout_s", 1.5);
    if (baud != 115200 || timeout_ <= 0 || depth_timeout_ <= 0)
      throw std::runtime_error("requires 115200 baud and positive timeouts");
    tx_pub_ = create_publisher<std_msgs::msg::String>("hardware/tx", 1);
    rx_pub_ = create_publisher<std_msgs::msg::String>("hardware/rx", 1);
    depth_pub_ = create_publisher<msg::DepthState>("depth/state", 1);
    // 时间戳及物理范围先校验，非法指令立即使输出回到 OFF。
    sub_ = create_subscription<msg::ThrusterCommand>(
        "actuators/command", 1, [this](msg::ThrusterCommand::ConstSharedPtr m) {
          double age = (now() - rclcpp::Time(m->header.stamp)).seconds();
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
          // A disabled frame is always a stop frame.
          latest_ = uw::encode_stm32(a);
          last_command_ = monotonic();
        });
    if (!dry_)
      open_serial();
    timer_ = create_wall_timer(20ms, [this] { tick(); });
    RCLCPP_INFO(get_logger(), "STM32 %s", dry_ ? "dry_run (serial unopened)" : device_.c_str());
  }
  ~HardwareNode() override {
    if (fd_ >= 0) {
      // Complete any partially transmitted line, then send an explicit OFF.
      std::string end = pending_.substr(offset_) + "\nOFF\n";
      size_t sent = 0;
      double deadline = monotonic() + .1;
      while (sent < end.size() && monotonic() < deadline) {
        ssize_t n = ::write(fd_, end.data() + sent, end.size() - sent);
        if (n > 0)
          sent += static_cast<size_t>(n);
        else
          usleep(1000);
      }
      ::close(fd_);
    }
  }

private:
  void open_serial() {
    fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0)
      throw std::runtime_error("cannot open " + device_);
    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
      ::close(fd_);
      fd_ = -1;
      throw std::runtime_error("tcgetattr failed");
    }
    cfmakeraw(&tty);
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      ::close(fd_);
      fd_ = -1;
      throw std::runtime_error("tcsetattr failed");
    }
    tcflush(fd_, TCIOFLUSH);
  }
  // 深度向下为正；仅在原有采样间隔范围内估计并低通深度变化率。
  void line_received(const std::string &line) {
    std_msgs::msg::String raw;
    raw.data = line;
    rx_pub_->publish(raw);
    auto value = uw::parse_depth(line);
    if (!value)
      return;
    double t = monotonic(), dt = t - depth_time_;
    if (depth_time_ > 0 && dt > .3 && dt < depth_timeout_)
      rate_ += .2 * ((*value - depth_) / dt - rate_);
    else
      rate_ = 0;
    depth_ = *value;
    depth_time_ = t;
  }
  // 一个周期只尝试一次写出；半帧必须写完整后才能切换下一条命令。
  void write_serial(const std::string &wanted, double t) {
    // Only one in-flight line; once a prefix was written it must finish intact.
    if (offset_ == 0)
      pending_ = wanted;
    if (pending_.empty()) {
      pending_ = wanted;
      offset_ = 0;
    }
    ssize_t n = ::write(fd_, pending_.data() + offset_, pending_.size() - offset_);
    if (n > 0) {
      offset_ += static_cast<size_t>(n);
      if (offset_ == pending_.size()) {
        pending_.clear();
        offset_ = 0;
        blocked_since_ = 0;
      }
    } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      RCLCPP_FATAL(get_logger(), "serial write failed");
      throw std::runtime_error("serial write failed");
    }
    if (!pending_.empty()) {
      if (blocked_since_ == 0)
        blocked_since_ = t;
      if (t - blocked_since_ > .1)
        throw std::runtime_error("serial output blocked >100ms");
    }
  }
  // 每周期最多读取 8 批；按换行拆帧，拒绝过长行并限制缓存大小。
  void read_serial() {
    char bytes[512];
    // Bounded work per tick, even if a broken device floods the port.
    for (int batch = 0; batch < 8; ++batch) {
      ssize_t count = ::read(fd_, bytes, sizeof(bytes));
      if (count <= 0) {
        if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
          throw std::runtime_error("serial read failed");
        break;
      }
      rx_.append(bytes, static_cast<size_t>(count));
      size_t pos;
      while ((pos = rx_.find('\n')) != std::string::npos) {
        std::string line = rx_.substr(0, pos);
        rx_.erase(0, pos + 1);
        if (line.size() <= 1024)
          line_received(line);
      }
      if (rx_.size() > 2048)
        rx_.clear();
    }
  }
  // 无新数据仍发布无效/过期状态；接收方不能把旧深度当成新采样。
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
    double t = monotonic();
    std::string wanted = t - last_command_ <= timeout_ ? latest_ : "OFF\n";
    std_msgs::msg::String tx;
    tx.data = wanted;
    tx_pub_->publish(tx);
    if (fd_ >= 0) {
      write_serial(wanted, t);
      read_serial();
    }
    publish_depth();
  }
  bool dry_;
  std::string device_, latest_ = "OFF\n", pending_, rx_;
  size_t offset_ = 0;
  int fd_ = -1;
  double timeout_, depth_timeout_, last_command_ = -1e9, depth_time_ = -1, depth_ = 0, rate_ = 0,
                                   blocked_since_ = 0;
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
