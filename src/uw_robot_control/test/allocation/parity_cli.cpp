/// @file parity_cli.cpp
/// @brief 分配器命令行工具：输出给定 wrench 下的执行器结果，用于一致性对比。
///

#include "uw_robot_control/allocation/allocator.hpp"
#include <iostream>
#include <iomanip>

int main() {
  using namespace uw;
  std::cout << std::setprecision(17);
  double a, b, step;
  V6 target;
  V4 ref;
  while (std::cin >> a >> b >> step) {
    for (int i = 0; i < 6; ++i)
      std::cin >> target[i];
    for (int i = 0; i < 4; ++i)
      std::cin >> ref[i];
    try {
      auto out = fixed_allocate(V2(a, b), target, ref, step);
      auto matrix = wrench_matrix(V2(a, b));
      for (int i = 0; i < 4; ++i)
        std::cout << out.forces[i] << ' ';
      for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 4; ++j)
          std::cout << matrix(i, j) << ' ';
      std::cout << '\n';
    } catch (const std::exception &e) {
      std::cerr << e.what() << '\n';
      return 1;
    }
  }
}
