#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch uw_robot_bringup robot.launch.py --show-args
python3 tools/integration.py
