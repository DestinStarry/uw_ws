#include <rclcpp/rclcpp.hpp>
#include <uw_robot_interfaces/msg/wrench_command.hpp>
#include <uw_robot_interfaces/msg/thruster_command.hpp>
#include <uw_robot_interfaces/msg/allocation_status.hpp>
#include "uw_robot_control/allocation/allocation_controller.hpp"
#include <condition_variable>
#include <mutex>
#include <thread>

using namespace std::chrono_literals;
namespace msg = uw_robot_interfaces::msg;
static double seconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

class AllocationNode : public rclcpp::Node {
public:
  AllocationNode() : Node("allocation_node") {
    uw::AllocationConfig config;
    config.servo_rate = declare_parameter("servo_rate_rad_s", config.servo_rate);
    config.thrust_rate = declare_parameter("thrust_rate_nps", config.thrust_rate);
    config.plan_period = declare_parameter("plan_period_s", config.plan_period);
    config.plan_max_age = declare_parameter("plan_max_age_s", config.plan_max_age);
    plan_min_improvement_ = declare_parameter("plan_min_improvement", config.plan_min_improvement);
    if (!std::isfinite(plan_min_improvement_) || plan_min_improvement_ < 0)
      throw std::runtime_error("invalid angle plan improvement threshold");
    timeout_ = declare_parameter("wrench_timeout_s", .15);
    for (double value : {config.servo_rate, config.thrust_rate, config.plan_period,
                         config.plan_max_age, timeout_}) {
      if (!std::isfinite(value) || value <= 0)
        throw std::runtime_error("invalid allocation parameter");
    }
    config.geometry = load_geometry();
    geometry_ = config.geometry;
    allocator_ = std::make_unique<uw::AllocationController>(config);
    command_pub_ = create_publisher<msg::ThrusterCommand>("actuators/command", 1);
    status_pub_ = create_publisher<msg::AllocationStatus>("allocation/status", 1);
    input_sub_ = create_subscription<msg::WrenchCommand>(
        "control/wrench", 1, [this](msg::WrenchCommand::ConstSharedPtr message) {
          const double age = (now() - rclcpp::Time(message->header.stamp)).seconds();
          if (age < -.05 || age > timeout_)
            return;
          uw::WrenchTarget value;
          for (int i = 0; i < 6; ++i)
            value.wrench[i] = message->wrench[i];
          if (!value.wrench.allFinite()) {
            received_ = -1e9;
            return;
          }
          value.enabled = message->enabled;
          value.closed_loop = message->closed_loop;
          value.center = message->center_servos;
          value.recenter = message->recenter_servos;
          value.session = message->session;
          value.epoch = message->epoch;
          input_ = value;
          // Include transmission delay in the monotonic freshness budget.
          received_ = seconds() - std::max(0., age);
        });
    worker_ = std::thread([this] { work(); });
    timer_ = create_wall_timer(20ms, [this] { tick(); });
  }

  ~AllocationNode() override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      quit_ = true;
    }
    cv_.notify_all();
    if (worker_.joinable())
      worker_.join();
  }

private:
  uw::AllocationGeometry load_geometry() {
    uw::AllocationGeometry g;
    g.rotating_sites = declare_parameter("rotating_thrust_sites", false);
    auto values = [this](const std::string &key, std::vector<double> defaults) {
      auto v = declare_parameter<std::vector<double>>(key, defaults);
      if (v.size() != defaults.size())
        throw std::runtime_error(key + " size mismatch");
      for (double x : v)
        if (!std::isfinite(x))
          throw std::runtime_error(key + " nonfinite");
      return v;
    };
    auto center = values("geometry_center_of_mass", {g.com[0], g.com[1], g.com[2]});
    g.com = uw::V3(center[0], center[1], center[2]);
    auto vectors = [&](const std::string &key, std::array<uw::V3, 4> &target) {
      std::vector<double> defaults;
      for (const auto &v : target)
        for (int j = 0; j < 3; ++j)
          defaults.push_back(v[j]);
      auto result = values(key, defaults);
      for (int i = 0; i < 4; ++i)
        target[i] = uw::V3(result[3 * i], result[3 * i + 1], result[3 * i + 2]);
    };
    vectors("geometry_sites", g.sites);
    vectors("geometry_hinges", g.hinges);
    vectors("geometry_axes", g.axes);
    vectors("geometry_child_centers", g.child_centers);
    auto fractions = values("geometry_mass_fractions", {0, 0, 0, 0});
    for (int i = 0; i < 4; ++i) {
      if (std::abs(g.axes[i].norm() - 1) > 1e-6 || fractions[i] < 0 || fractions[i] > 1)
        throw std::runtime_error("invalid geometry axis or mass fraction");
      g.mass_fractions[i] = fractions[i];
    }
    if (g.mass_fractions.sum() > 1)
      throw std::runtime_error("geometry mass fractions exceed total mass");
    return g;
  }
  // 耗时的角度规划在后台运行；pending_ 只保留最新请求。
  void work() {
    while (true) {
      uw::Request request;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] { return quit_ || pending_.has_value(); });
        if (quit_)
          return;
        request = *pending_;
        pending_.reset();
      }
      try {
        const double started = seconds();
        auto result = uw::plan_angles(request, geometry_, plan_min_improvement_);
        result.ms = (seconds() - started) * 1000.;
        std::lock_guard<std::mutex> lock(mutex_);
        result_ = result;
      } catch (const std::exception &e) {
        RCLCPP_ERROR(get_logger(), "Angle planner: %s", e.what());
      }
    }
  }

  // 50 Hz 执行分配；锁内仅交换数据，不运行优化求解。
  void tick() {
    std::optional<uw::Plan> result;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      result = result_;
      result_.reset();
    }
    const double time = seconds();
    uw::Actuators output;
    try {
      output = allocator_->update(input_, time, time - received_ <= timeout_, result);
    } catch (const std::exception &e) {
      allocator_->stop();
      allocator_->state = "ALLOCATION_ERROR";
      RCLCPP_ERROR(get_logger(), "Allocation: %s", e.what());
    }
    if (auto request = allocator_->take_request()) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        pending_ = request;
      }
      cv_.notify_one();
    }
    msg::ThrusterCommand command;
    command.header.stamp = now();
    command.header.frame_id = "base_link";
    command.sequence = sequence_++;
    command.enabled = output.enabled;
    for (int i = 0; i < 4; ++i)
      command.thrust_newtons[i] = output.forces[i];
    command.theta12_rad = output.angles[0];
    command.theta34_rad = output.angles[1];
    command.gripper_closed = output.gripper;
    command_pub_->publish(command);
    if (sequence_ % 5 == 0) {
      msg::AllocationStatus status;
      status.header = command.header;
      status.planner_valid = allocator_->planner_valid;
      status.planner_ms = allocator_->planner_ms;
      for (int i = 0; i < 6; ++i) {
        status.achievable_wrench[i] = allocator_->attainable[i];
        status.achieved_wrench[i] = allocator_->achieved[i];
        status.target_wrench[i] = allocator_->target[i];
      }
      for (int i = 0; i < 4; ++i)
        status.thruster_forces[i] = allocator_->thruster_forces()[i];
      for (int i = 0; i < 2; ++i) {
        status.servo_target_rad[i] = allocator_->servo_target()[i];
        status.servo_estimated_rad[i] = allocator_->servo_estimated()[i];
        status.gate[i] = allocator_->gate_state()[i];
      }
      status.state = allocator_->state;
      status_pub_->publish(status);
    }
  }

  uw::AllocationGeometry geometry_; // 初始化后只读，后台规划和实时分配共用同一几何。
  double plan_min_improvement_ = 0;
  std::unique_ptr<uw::AllocationController> allocator_;
  uw::WrenchTarget input_;
  double received_ = -1e9, timeout_ = .15;
  uint32_t sequence_ = 0;
  std::mutex mutex_;
  std::condition_variable cv_;
  bool quit_ = false;
  std::thread worker_;
  std::optional<uw::Request> pending_;
  std::optional<uw::Plan> result_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<msg::WrenchCommand>::SharedPtr input_sub_;
  rclcpp::Publisher<msg::ThrusterCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<msg::AllocationStatus>::SharedPtr status_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<AllocationNode>());
  } catch (const std::exception &e) {
    fprintf(stderr, "allocation_node: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
