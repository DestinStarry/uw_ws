#!/usr/bin/env bash
# 一次性安装 Orin 上的运行依赖（Ubuntu 22.04 + ROS 2 Humble）。
#
#   bash scripts/install_deps.sh              # 装系统依赖与 ROS 依赖
#   bash scripts/install_deps.sh --check-only # 只体检，不改系统
#
# 前提：ROS 2 Humble 已安装（/opt/ros/humble/setup.bash 存在）。
# 不负责安装 YbImuLib（IMU 的第三方 SDK），见文件末尾提示与 docs/Orin部署.md。
set -euo pipefail

cd "$(dirname "$0")/.."
TARGET_USER="${USER:-$(id -un)}"
CHECK_ONLY=0
[[ "${1:-}" == "--check-only" ]] && CHECK_ONLY=1

ok()   { printf '  \033[32m✓\033[0m %s\n' "$1"; }
warn() { printf '  \033[33m!\033[0m %s\n' "$1"; }
bad()  { printf '  \033[31m✗\033[0m %s\n' "$1"; }

echo "== 1. ROS 2 环境 =="
DISTRO="${ROS_DISTRO:-humble}"
SETUP="/opt/ros/${DISTRO}/setup.bash"
if [[ -f "$SETUP" ]]; then
  ok "找到 $SETUP"
else
  bad "未找到 $SETUP"
  echo "    请先安装 ROS 2 ${DISTRO}：https://docs.ros.org/en/humble/Installation.html"
  exit 1
fi
# shellcheck disable=SC1090
# ROS setup scripts may read unset optional variables.
set +u
source "$SETUP"
set -u

echo "== 2. apt 依赖 =="
APT_PKGS=(
  build-essential cmake git curl
  libeigen3-dev
  python3-pip python3-dev python3-setuptools
  i2c-tools v4l-utils
  ros-dev-tools
)
if [[ $CHECK_ONLY -eq 1 ]]; then
  for p in libeigen3-dev i2c-tools v4l-utils; do
    dpkg -s "$p" >/dev/null 2>&1 && ok "$p 已装" || warn "$p 未装"
  done
else
  sudo apt-get update
  sudo apt-get install -y --no-install-recommends "${APT_PKGS[@]}"
  ok "apt 依赖安装完成"
fi

echo "== 3. rosdep 依赖（按 package.xml 解析）=="
if [[ $CHECK_ONLY -eq 1 ]]; then
  rosdep check --from-paths src --ignore-src >/dev/null 2>&1 \
    && ok "rosdep 依赖齐备" || warn "rosdep 报告缺失依赖，去掉 --check-only 重跑"
else
  if ! rosdep check --from-paths src --ignore-src >/dev/null 2>&1; then
    sudo apt-get install -y python3-rosdep
    [[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]] || sudo rosdep init
    rosdep update
    rosdep install --from-paths src --ignore-src -y --rosdistro "$DISTRO"
  fi
  ok "rosdep 依赖完成"
fi

echo "== 4. Python 依赖 =="
PY_PKGS=(smbus2)
if [[ $CHECK_ONLY -eq 1 ]]; then
  python3 -c "import smbus2" 2>/dev/null && ok "smbus2" || warn "smbus2 未装（深度节点需要）"
  python3 -c "import serial" 2>/dev/null && ok "pyserial" || warn "pyserial 未装（IMU 需要）"
  python3 -c "import cv2" 2>/dev/null && ok "opencv" || warn "opencv 未装（相机需要）"
  python3 -c "import YbImuLib" 2>/dev/null && ok "YbImuLib" \
    || warn "YbImuLib 未装（IMU 需要，见文末）"
else
  # smbus2 优先于 python3-smbus：异常类型清晰，且 arm64 有 wheel。
  pip3 install --user "${PY_PKGS[@]}"
  ok "pip 依赖完成（smbus2）"
fi

echo "== 5. 用户组 =="
NEED_GROUPS=()
id -nG "$TARGET_USER" | grep -qw i2c || NEED_GROUPS+=(i2c)
id -nG "$TARGET_USER" | grep -qw dialout || NEED_GROUPS+=(dialout)
id -nG "$TARGET_USER" | grep -qw video || NEED_GROUPS+=(video)
if [[ ${#NEED_GROUPS[@]} -eq 0 ]]; then
  ok "i2c / dialout / video 已就绪"
elif [[ $CHECK_ONLY -eq 1 ]]; then
  warn "缺少组：${NEED_GROUPS[*]}"
else
  sudo usermod -aG "$(IFS=,; echo "${NEED_GROUPS[*]}")" "$TARGET_USER"
  warn "已加入 ${NEED_GROUPS[*]}，需重新登录（或重启）后生效"
fi

echo
echo "== 后续步骤 =="
cat <<'EOF'
  1. IMU 使用轮趣 YbImu SDK，不在本仓库内，需单独安装：
       pip3 install --user /path/to/YbImuLib      # 目录内含 setup.py
     或把 YbImuLib 源码目录加入 PYTHONPATH。
     装不上时先运行 imu:=false 启动其余节点。
  2. 重新登录使 i2c/dialout/video 组生效：newgrp dialout 或注销重登。
  3. 运行设备体检：bash scripts/preflight.sh
  4. 编译：bash scripts/check.sh
  详见 docs/Orin部署.md。
EOF

if [[ $CHECK_ONLY -eq 0 ]]; then
  echo
  echo "若 smbus2 装到了 ~/.local 但 python3 仍找不到，请确认 ~/.local/bin 与"
  echo "site-packages 在 PATH/PYTHONPATH 中（Ubuntu 默认已包含 ~/.local/lib）。"
fi
