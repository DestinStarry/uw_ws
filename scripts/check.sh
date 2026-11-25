#!/usr/bin/env bash
# 编译 + 单元测试。提交前必跑；上机前也跑一次确认环境完好。
#
#   bash scripts/check.sh              # 编译 + 测试
#   bash scripts/check.sh --build-only # 只编译
#   bash scripts/check.sh --clean      # 先删 build/install/log 再编译
#
# 不触碰任何硬件：所有测试都是纯算法或伪设备，不打开串口/I2C/相机。
set -euo pipefail

cd "$(dirname "$0")/.."

MODE="${1:-all}"
if [[ "$MODE" == "--clean" ]]; then
  echo "== 清理旧构建 =="
  rm -rf build install log
  MODE="all"
fi

# shellcheck disable=SC1091
# ROS 的 setup.bash 会引用未定义变量，set -u 下会直接退出；只在这一句前后关闭 nounset。
set +u
source /opt/ros/humble/setup.bash
set -u

# Orin 上并行度过高会因内存不足被杀（8GB 型号尤其明显）。
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
export MAKEFLAGS="-j${CMAKE_BUILD_PARALLEL_LEVEL}"

echo "== 构建 =="
colcon build --symlink-install --executor sequential \
  --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON

# shellcheck disable=SC1091
set +u
source install/setup.bash
set -u

if [[ "$MODE" == "--build-only" ]]; then
  echo "== 构建完成（跳过测试）=="
  exit 0
fi

echo "== 单元测试 =="
# C++：控制器、松杆交接、分配器、STM32 协议、TCP 协议；Python：IMU/深度/相机。
colcon test --packages-select uw_robot_control uw_robot_communication uw_robot_sensors uw_robot_logging \
  --event-handlers console_direct+
colcon test-result --verbose

echo "== Python 测试（不经 colcon，便于看单条失败）=="
if python3 -c "import pytest" >/dev/null 2>&1; then
  python3 -m pytest src/uw_robot_sensors/test src/uw_robot_logging/test -q || {
    echo "Python 测试失败；若只是缺少硬件库（YbImuLib/smbus2），"
    echo "请先跑 bash scripts/install_deps.sh 再重试。"
    exit 1
  }
else
  echo "  未安装 pytest，跳过：sudo apt install python3-pytest"
fi

echo
echo "全部通过。下一步：bash scripts/preflight.sh 体检设备。"
