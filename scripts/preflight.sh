#!/usr/bin/env bash
# 上机前设备体检：不改配置、不启动节点、不下发任何指令。
#
#   bash scripts/preflight.sh
#
# 检查项：ROS 环境 / I2C 深度计 / 串口设备与权限 / 相机 / Python 模块 / 深度发布者配置。
# 任何一项 ✗ 都不影响继续查看其余项，但上水前必须全部转 ✓。
set -uo pipefail

cd "$(dirname "$0")/.."
YAML="src/uw_robot_bringup/config/robot.yaml"
# systemd/容器里可能没有 $USER，set -u 下直接展开会中断。
TARGET_USER="${USER:-$(id -un)}"

ok()   { printf '  \033[32m✓\033[0m %s\n' "$1"; }
warn() { printf '  \033[33m!\033[0m %s\n' "$1"; }
bad()  { printf '  \033[31m✗\033[0m %s\n' "$1"; }
info() { printf '    %s\n' "$1"; }

# 取某节点段下标量参数：yaml_scalar <段名> <键名>
yaml_scalar() {
  awk -v sec="$1" -v key="$2" '
    $0 ~ "^"sec":" {insec=1; next}
    insec && /^[a-z_]+:/ {insec=0}
    insec && $0 ~ "^[ \t]+"key":" {
      sub(/^[ \t]*[a-z_]+:[ \t]*/, ""); sub(/[ \t]+#.*$/, ""); print; exit
    }' "$YAML"
}

# 取某节点段下列表参数的前 n 项：yaml_list <段名> <键名> [n]
yaml_list() {
  awk -v sec="$1" -v key="$2" -v want="${3:-3}" '
    $0 ~ "^"sec":" {insec=1; next}
    insec && /^[a-z_]+:/ {insec=0; grab=0}
    insec && $0 ~ "^[ \t]+"key":" {grab=1; next}
    grab && /^[ \t]+-[ \t]*/ {
      line=$0; sub(/^[ \t]+-[ \t]*/, "", line); sub(/[ \t]+#.*$/, "", line)
      printf "%s%s", (n++ ? "," : ""), line
      if (n >= want) exit
    }
    grab && $0 !~ /^[ \t]/ {exit}' "$YAML"
}

FAIL=0

echo "== 1. ROS 2 环境 =="
DISTRO="${ROS_DISTRO:-humble}"
if [[ -f "/opt/ros/${DISTRO}/setup.bash" ]]; then
  ok "ROS ${DISTRO} 已安装"
  # 某些 ROS 发行版的 setup.bash 会引用未定义变量（如 AMENT_TRACE_SETUP_FILES），
  # set -u 下会直接中断整个体检；只在这一句前后临时关闭 nounset。
  # shellcheck disable=SC1090
  set +u
  source "/opt/ros/${DISTRO}/setup.bash"
  set -u
else
  bad "未找到 /opt/ros/${DISTRO}/setup.bash"
  FAIL=1
fi
if command -v ros2 >/dev/null 2>&1; then
  ok "ros2 CLI 可用"
else
  bad "ros2 命令不可用"
  FAIL=1
fi
if [[ -f install/setup.bash ]]; then
  ok "工作空间已构建（install/setup.bash 存在）"
else
  warn "尚未构建，先跑 bash scripts/check.sh"
fi

echo "== 2. I2C 深度计（MS5837，地址 0x76）=="
if ! command -v i2cdetect >/dev/null 2>&1; then
  bad "i2cdetect 未安装：sudo apt install i2c-tools"
  FAIL=1
else
  BUSES=$(i2cdetect -l 2>/dev/null | sed -n 's/^i2c-\([0-9]\+\).*/\1/p' | sort -n)
  if [[ -z "$BUSES" ]]; then
    bad "系统没有可用 I2C 总线（检查 dtb/overlay 与接线）"
    FAIL=1
  else
    info "可用总线：$(echo "$BUSES" | tr '\n' ' ')"
    FOUND=""
    for b in $BUSES; do
      # 无 i2c 组权限时用 sudo 兜底，避免把权限问题误判成设备不在。
      OUT=$(i2cdetect -y -r "$b" 2>/dev/null) || OUT=$(sudo -n i2cdetect -y -r "$b" 2>/dev/null) || continue
      if echo "$OUT" | grep -qi "76"; then FOUND="$FOUND $b"; fi
    done
    if [[ -n "$FOUND" ]]; then
      ok "在 i2c-$(echo "$FOUND" | awk '{print $1}') 上发现 0x76"
      CFG_BUS=$(yaml_scalar depth_node bus)
      info "robot.yaml 中 depth_node.bus = ${CFG_BUS:-未设置}"
      if [[ -n "${CFG_BUS:-}" ]] && [[ " $FOUND " != *" $CFG_BUS "* ]]; then
        bad "配置的总线 ${CFG_BUS} 不是检测到的那条，请改 depth_node.bus"
        FAIL=1
      fi
    else
      bad "所有总线都没扫到 0x76（未接线 / 供电非 3.3V / 上拉缺失）"
      FAIL=1
    fi
  fi
fi

echo "== 3. 串口设备 =="
mapfile -t PORTS < <(ls /dev/ttyUSB* /dev/ttyACM* /dev/ttyTHS* /dev/ttyAS* 2>/dev/null)
if [[ ${#PORTS[@]} -eq 0 ]]; then
  bad "没有串口设备节点（STM32 与 IMU 都需要）"
  FAIL=1
else
  for p in "${PORTS[@]}"; do
    info "$p  属主 $(stat -c '%U:%G %a' "$p" 2>/dev/null)"
  done
  if [[ -e /dev/serial/by-id ]]; then
    echo "    按 ID 的稳定路径："
    for l in /dev/serial/by-id/*; do info "  $l -> $(readlink -f "$l")"; done
  fi
  # 配置里的硬件串口
  HW_PORT=$(yaml_scalar hardware_node port)
  IMU_PORT=$(yaml_scalar imu_node port)
  [[ -n "${HW_PORT:-}" ]] && { [[ -e "$HW_PORT" ]] && ok "hardware_node.port=$HW_PORT 存在" \
      || { bad "hardware_node.port=$HW_PORT 不存在"; FAIL=1; }; }
  [[ -n "${IMU_PORT:-}" ]] && { [[ -e "$IMU_PORT" ]] && ok "imu_node.port=$IMU_PORT 存在" \
      || warn "imu_node.port=$IMU_PORT 不存在（不启 IMU 可忽略）"; }
fi
id -nG "$TARGET_USER" | grep -qw dialout && ok "当前用户在 dialout 组" || { bad "当前用户不在 dialout 组"; FAIL=1; }

echo "== 4. 相机 =="
if command -v v4l2-ctl >/dev/null 2>&1; then
  if v4l2-ctl --list-devices 2>/dev/null | grep -q video; then
    ok "检测到视频设备"
    v4l2-ctl --list-devices 2>/dev/null | sed 's/^/    /' | head -8
  else
    warn "未检测到视频设备（camera:=false 可忽略）"
  fi
else
  warn "v4l2-ctl 未安装：sudo apt install v4l-utils"
fi
id -nG "$TARGET_USER" | grep -qw video && ok "当前用户在 video 组" || warn "当前用户不在 video 组（相机需要）"

echo "== 5. Python 模块 =="
for m in "smbus2:深度节点必需" "serial:IMU 必需" "cv2:相机必需" "YbImuLib:IMU 必需, 第三方 SDK"; do
  mod="${m%%:*}"; why="${m##*:}"
  if python3 -c "import $mod" 2>/dev/null; then ok "$mod"; else
    case "$mod" in
      YbImuLib|cv2|serial) warn "$mod 缺失（$why）";;
      *) bad "$mod 缺失（$why）"; FAIL=1;;
    esac
  fi
done

echo "== 6. 深度发布者配置 =="
PUB_DEPTH=$(yaml_scalar hardware_node publish_depth)
info "hardware_node.publish_depth = ${PUB_DEPTH:-未设置}"
info "实际由 launch 的 depth_source 覆盖：i2c→depth_node 独占，stm32→hardware_node 发布"
if [[ "${PUB_DEPTH:-}" == "true" ]]; then
  warn "yaml 里为 true，但 robot.launch.py 会按 depth_source 覆盖，不必改"
fi

echo "== 7. 控制参数状态 =="
info "moment_sign（实物应为 -1,-1,-1）= $(yaml_list control_node moment_sign 3)"
info "rate_kd（默认 0）= $(yaml_list control_node rate_kd 3)"
info "hydrostatic_comp_scale = $(yaml_scalar control_node hydrostatic_comp_scale)"
warn "以上为仿真调校值，未经水下实测定标"

echo
if [[ $FAIL -eq 0 ]]; then
  printf '\033[32m体检通过\033[0m：可以执行 bash scripts/check.sh，再按 docs/Orin部署.md 分步上水。\n'
else
  printf '\033[31m存在阻塞项\033[0m：先解决上面标 ✗ 的项目。\n'
  exit 1
fi
