#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <sensor_msgs/msg/imu.hpp>
#include <uw_robot_interfaces/msg/control_command.hpp>
#include <uw_robot_interfaces/msg/depth_state.hpp>
#include <uw_robot_interfaces/msg/wrench_command.hpp>
#include <uw_robot_interfaces/msg/robot_status.hpp>
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
    const auto c = load_config();
    controller_ = std::make_unique<uw::Controller>(c);
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
  // 参数承载控制器名义增益；发送端只给物理量指令，不再传摇杆轴。
  uw::ControlConfig load_config() {
    uw::ControlConfig c;
    auto v3 = [this](const std::string &key, uw::V3 &value) {
      auto v = declare_parameter<std::vector<double>>(key, {value[0], value[1], value[2]});
      if (v.size() != 3)
        throw std::runtime_error(key + " must have 3 entries");
      for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(v[i]))
          throw std::runtime_error(key + " nonfinite");
        value[i] = v[i];
      }
    };
    v3("damping", c.damping);
    v3("open_damping", c.open_damping);
    v3("rate_ff", c.rate_ff);
    v3("open_moment", c.open_moment);
    v3("rate_full_scale_rad_s", c.rate_full_scale);
    v3("body_moment", c.moment);
    v3("attitude_kp", c.attitude_kp);
    v3("attitude_deadzone_rad", c.attitude_deadzone);
    v3("attitude_ki", c.attitude_ki);
    // 静水扶正前馈：用物理量算，不用拟合常数
    const double rho = declare_parameter("water_density_kg_m3", 1000.0);
    const double volume = declare_parameter("displaced_volume_m3", 0.0);
    const double scale = declare_parameter("hydrostatic_comp_scale", 1.0);
    auto offset = declare_parameter<std::vector<double>>("buoyancy_offset_m", {0.0, 0.0, 0.0});
    if (offset.size() != 3)
      throw std::runtime_error("buoyancy_offset_m requires 3 entries");
    for (double v : offset)
      if (!std::isfinite(v))
        throw std::runtime_error("buoyancy_offset_m nonfinite");
    if (!std::isfinite(rho) || rho <= 0 || !std::isfinite(volume) || volume < 0 ||
        !std::isfinite(scale) || scale < 0)
      throw std::runtime_error("invalid hydrostatic parameters");
    c.hydrostatic_coeff =
        uw::V3(offset[0], offset[1], offset[2]) * (rho * volume * 9.80665 * scale);
    v3("rate_kp", c.rate_kp);
    v3("rate_ki", c.rate_ki);
    v3("rate_deadzone_rad_s", c.rate_deadzone);
    c.smooth_deadband = declare_parameter("smooth_deadband", false);
    c.roll_smooth_deadband = declare_parameter("roll_smooth_deadband", false);
    c.roll_gyro_filter_alpha = declare_parameter("roll_gyro_filter_alpha", c.gyro_filter_alpha);
    if (!std::isfinite(c.roll_gyro_filter_alpha) || c.roll_gyro_filter_alpha <= 0 ||
        c.roll_gyro_filter_alpha > 1)
      throw std::runtime_error("invalid roll gyro filter");
    c.gyro_filter_alpha = declare_parameter("gyro_filter_alpha", c.gyro_filter_alpha);
    if (!std::isfinite(c.gyro_filter_alpha) || c.gyro_filter_alpha <= 0 ||
        c.gyro_filter_alpha > 1 || (c.rate_deadzone.array() < 0).any())
      throw std::runtime_error("invalid feedback filter/deadzone");
    v3("moment_sign", c.moment_sign);
    c.prioritize_hold_on_release = declare_parameter("prioritize_hold_on_release", false);
    v3("inner_rate_rad_s", c.inner_rate);
    auto wr = declare_parameter<std::vector<double>>("wrench_rate", {150, 150, 150, 8, 8, 8});
    if (wr.size() != 6)
      throw std::runtime_error("wrench_rate requires six entries");
    for (int i = 0; i < 6; ++i) {
      if (!std::isfinite(wr[i]) || wr[i] <= 0)
        throw std::runtime_error("invalid wrench rate");
      c.wrench_rate[i] = wr[i];
    }
    c.depth_kp = declare_parameter("depth_kp", c.depth_kp);
    c.depth_ki = declare_parameter("depth_ki", c.depth_ki);
    c.depth_kd = declare_parameter("depth_kd", c.depth_kd);
    c.depth_force = declare_parameter("depth_force", c.depth_force);
    c.depth_change = declare_parameter("depth_change_mps", c.depth_change);
    c.release_center_time = declare_parameter("release_center_time_s", c.release_center_time);
    c.yaw_hold = declare_parameter("yaw_hold", true);
    c.roll_hold = declare_parameter("roll_hold", true);
    c.manual_rate_mode = declare_parameter("manual_rate_mode", true);
    cmd_timeout_ = declare_parameter("cmd_timeout_s", .35);
    imu_timeout_ = declare_parameter("imu_timeout_s", .1);
    depth_timeout_ = declare_parameter("depth_timeout_s", 1.5);
    for (double value : {c.depth_kp, c.depth_ki, c.depth_kd, c.depth_force, c.depth_change,
                         c.release_center_time,
                         cmd_timeout_, imu_timeout_, depth_timeout_})
      if (!std::isfinite(value) || value <= 0)
        throw std::runtime_error("invalid scalar parameter");
    for (int i = 0; i < 3; ++i) {
      if (c.attitude_ki[i] < 0 || c.damping[i] <= 0 || c.open_damping[i] <= 0 || c.moment[i] <= 0 || c.rate_ff[i] <= 0 ||
          c.open_moment[i] <= 0 || c.rate_full_scale[i] <= 0 || c.attitude_kp[i] < 0 ||
          c.rate_kp[i] < 0 || c.rate_ki[i] < 0 || c.inner_rate[i] <= 0 ||
          std::abs(c.moment_sign[i]) != 1)
        throw std::runtime_error("invalid vector gain/sign");
    }
    return c;
  }
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
