/// @file control_cli.cpp
/// @brief 控制算法命令行工具：读入 JSON/参数并打印 wrench，用于与参考实现对齐。
///

#include "uw_robot_control/closed_loop/controller.hpp"
#include <iomanip>
#include <iostream>
// 输入列：t u v w p q r arm estop depth loop qw qx qy qz gx gy gz imu_ok depth depth_rate depth_ok
int main() {
  uw::Controller controller;
  double t;
  uw::Command cmd;
  uw::Imu imu;
  uw::Depth depth;
  int arm, estop, depth_toggle, loop_toggle, imu_ok, depth_ok;
  double qw, qx, qy, qz;
  std::cout << std::setprecision(17);
  while (std::cin >> t >> cmd.velocity[0] >> cmd.velocity[1] >> cmd.velocity[2] >> cmd.velocity[3] >>
         cmd.velocity[4] >> cmd.velocity[5] >> arm >> estop >> depth_toggle >> loop_toggle >> qw >>
         qx >> qy >> qz >> imu.gyro[0] >> imu.gyro[1] >> imu.gyro[2] >> imu_ok >> depth.value >>
         depth.rate >> depth_ok) {
    cmd.arm = arm;
    cmd.estop = estop;
    cmd.depth_toggle = depth_toggle;
    cmd.loop_toggle = loop_toggle;
    imu.q = Eigen::Quaterniond(qw, qx, qy, qz);
    imu.valid = imu_ok;
    depth.valid = depth_ok;
    controller.update(cmd, imu, depth, t, true);
    for (int i = 0; i < 6; ++i)
      std::cout << controller.target[i] << ' ';
    std::cout << controller.armed << ' ' << controller.closed_loop << ' ' << controller.depth_hold
              << '\n';
  }
}
