/// @file control_node.cpp
/// @brief 控制节点：订阅指令/IMU/深度，按 50 Hz 输出 wrench 与状态。
///
/// 仲裁：arm 边沿、estop 电平、指令/IMU/深度超时，任一失效即停机；
/// 参数集中在 load_config()，启动时一次性校验，非法即退出。

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <sensor_msgs/msg/imu.hpp>
#include <uw_robot_interfaces/msg/control_command.hpp>
#include <uw_robot_interfaces/msg/depth_state.hpp>
#include <uw_robot_interfaces/msg/wrench_command.hpp>
#include <uw_robot_interfaces/msg/robot_status.hpp>
#include "uw_robot_control/closed_loop/config.hpp"
#include "uw_robot_control/closed_loop/controller.hpp"
#include <random>

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
static double seconds() {
  return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}
namespace msg = uw_robot_interfaces::msg;
class ControlNode : public rclcpp::Node {
public:
  ControlNode() : Node("control_node") {
    const auto settings = uw::load_control_settings(*this);
    controller_ = std::make_unique<uw::Controller>(settings.control);
    cmd_timeout_ = settings.cmd_timeout;
    imu_timeout_ = settings.imu_timeout;
    depth_timeout_ = settings.depth_timeout;
    command_pub_ = create_publisher<msg::WrenchCommand>("control/wrench", 1);
    status_pub_ = create_publisher<msg::RobotStatus>("robot/status", 1);
    cmd_sub_ = create_subscription<msg::ControlCommand>(
        "control/cmd", 1, [this](msg::ControlCommand::ConstSharedPtr m) {
          if (!fresh(m->header.stamp, cmd_timeout_))
            return;
          uw::Command command;
          for (int i = 0; i < 6; ++i)
            command.velocity[i] = m->velocity[i];
          command.arm = m->arm;
          command.estop = m->estop;
          command.depth_toggle = m->depth_toggle;
          command.loop_toggle = m->loop_toggle;
          if (!command.finite())
            return;
          // 只做防呆上限校验；真实量程由发送端和 robot.yaml 决定。
          for (int i = 0; i < 6; ++i)
            if (std::abs(command.velocity[i]) > 2.)
              return;
          cmd_ = command;
          cmd_time_ = seconds();
        });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {
          if (!fresh(m->header.stamp, imu_timeout_))
            return;
          imu_.q = Eigen::Quaterniond(m->orientation.w, m->orientation.x, m->orientation.y,
                                      m->orientation.z);
          imu_.gyro = uw::V3(m->angular_velocity.x, m->angular_velocity.y, m->angular_velocity.z);
          imu_.valid = m->orientation_covariance[0] >= 0;
          imu_time_ = seconds();
        });
    depth_sub_ = create_subscription<msg::DepthState>(
        "depth/state", 1, [this](msg::DepthState::ConstSharedPtr m) {
          depth_.value = m->depth_m;
          depth_.rate = m->depth_rate_mps;
          depth_.valid = m->valid && std::isfinite(m->data_age_s) && m->data_age_s >= 0 &&
                         m->data_age_s <= depth_timeout_ && fresh(m->header.stamp, .2);
          depth_time_ = seconds() - m->data_age_s;
        });
    // 50 Hz 控制；状态每 5 个周期发布一次（10 Hz）。
    timer_ = create_wall_timer(20ms, [this] { tick(); });
  }

private:
  bool fresh(const builtin_interfaces::msg::Time &stamp, double timeout) {
    double age = (now() - rclcpp::Time(stamp)).seconds();
    return age >= -.05 && age <= timeout;
  }
  void tick() {
    double t = seconds();
    uw::Imu imu = imu_;
    uw::Depth depth = depth_;
    imu.valid = imu.valid && t - imu_time_ <= imu_timeout_;
    depth.valid = depth.valid && t - depth_time_ <= depth_timeout_;
    uw::WrenchTarget output;
    try {
      output = controller_->update(cmd_, imu, depth, t, t - cmd_time_ <= cmd_timeout_);
    } catch (const std::exception &e) {
      controller_->stop();
      controller_->state = "CONTROL_ERROR";
      RCLCPP_ERROR(get_logger(), "Control: %s", e.what());
    }
    msg::WrenchCommand cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = "base_link";
    cmd.sequence = sequence_++;
    cmd.session = source_session_;
    cmd.epoch = controller_->epoch;
    cmd.enabled = output.enabled;
    cmd.closed_loop = output.closed_loop;
    cmd.center_servos = output.center;
    cmd.recenter_servos = output.recenter;
    for (int i = 0; i < 6; ++i)
      cmd.wrench[i] = output.wrench[i];
    command_pub_->publish(cmd);
    if (sequence_ % 5 == 0) {
      msg::RobotStatus s;
      s.header = cmd.header;
      s.armed = controller_->armed;
      s.closed_loop = controller_->closed_loop;
      s.depth_hold = controller_->depth_hold;
      s.depth_target_m = controller_->depth_target_m();
      s.imu_valid = imu.valid;
      s.depth_valid = depth.valid;
      s.cycle_dt_s = controller_->cycle_dt;
      for (int i = 0; i < 6; ++i) {
        s.target_wrench[i] = controller_->target[i];
      }
      s.state = controller_->state;
      status_pub_->publish(s);
    }
  }
  std::unique_ptr<uw::Controller> controller_;
  uw::Command cmd_;
  uw::Imu imu_;
  uw::Depth depth_;
  double cmd_time_ = -1e9, imu_time_ = -1e9, depth_time_ = -1e9, cmd_timeout_, imu_timeout_,
         depth_timeout_;
  uint32_t sequence_ = 0;
  const uint64_t source_session_ =
      (uint64_t(std::random_device{}()) << 32) | std::random_device{}();
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<msg::WrenchCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<msg::RobotStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<msg::ControlCommand>::SharedPtr cmd_sub_;
  rclcpp::Subscription<msg::DepthState>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ControlNode>());
  } catch (const std::exception &e) {
    fprintf(stderr, "control_node: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
