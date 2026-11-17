#!/usr/bin/env python3
"""六自由度自动测试：逐轴施加满杆速度指令，记录 MuJoCo 运动并给出指标。

直接向 control/cmd 发 ROS 消息（不经过 TCP 桥），避免网络抖动污染测量；
TCP 协议另由 tools/integration.py 覆盖。

每轴流程：回平 -> 中立 -> 满杆保持 ->（角轴）反向回绕 -> 回中观察。
用法：
    python3 tools/six_dof_test.py
    python3 tools/six_dof_test.py --no-launch --port 18899   # 连接已运行的仿真
"""
import argparse
import json
import math
import os
import secrets
import signal
import subprocess
import time
from collections import deque
from pathlib import Path

os.environ['ROS_DOMAIN_ID'] = str(20 + secrets.randbelow(40))
os.environ['ROS_LOCALHOST_ONLY'] = '0'
os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(Path(__file__).with_name('dds_test.xml'))

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from uw_robot_interfaces.msg import ControlCommand, DepthState, RobotStatus, WrenchCommand

ROOT = Path(__file__).resolve().parents[1]
FULL = (1.0, 1.0, 1.0, math.radians(25.0), math.radians(25.0), math.radians(25.0))
AXES = ('u 纵荡', 'v 横荡', 'w 升沉', 'p 横滚', 'q 俯仰', 'r 偏航')
ANGULAR = (False, False, False, True, True, True)


def euler(q):
    w, x, y, z = q
    return (math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
            math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))),
            math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Sampler:
    def __init__(self, node):
        self.samples = deque(maxlen=8000)
        self.status = None
        self._pos = None
        self._quat = (1.0, 0.0, 0.0, 0.0)
        self._gyro = (0.0, 0.0, 0.0)
        self._depth = None
        self.wrenches = deque(maxlen=4000)
        self._wrench = None
        node.create_subscription(WrenchCommand, 'control/wrench', self._wrench_cb, 10)
        node.create_subscription(PoseStamped, 'simulation/pose', self._pose, 10)
        node.create_subscription(Imu, 'imu/data', self._imu, qos_profile_sensor_data)
        node.create_subscription(DepthState, 'depth/state', self._depth_cb, 10)
        node.create_subscription(RobotStatus, 'robot/status', self._status, 10)

    def _pose(self, m):
        p, q = m.pose.position, m.pose.orientation
        self._pos = (p.x, p.y, p.z)
        self._quat = (q.w, q.x, q.y, q.z)

    def _imu(self, m):
        g = m.angular_velocity
        self._gyro = (g.x, g.y, g.z)

    def _depth_cb(self, m):
        self._depth = m.depth_m

    def _status(self, m):
        self.status = m

    def _wrench_cb(self, m):
        self._wrench = list(m.wrench)
        self.wrenches.append((time.monotonic(), self._wrench))

    def record(self):
        if self._pos is not None:
            self.samples.append((time.monotonic(), self._pos, self._quat,
                                 self._gyro, self._depth))

    def at(self, when):
        for s in reversed(self.samples):
            if s[0] <= when:
                return s
        return None

    def tail_mean(self, t0, t1, field):
        rows = [s for s in self.samples if t0 <= s[0] <= t1]
        if not rows:
            return [0.0, 0.0, 0.0]
        return [sum(r[field][k] for r in rows) / len(rows) for k in range(3)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--hold', type=float, default=3.0)
    ap.add_argument('--angular-hold', type=float, default=2.0)
    ap.add_argument('--settle', type=float, default=2.0)
    ap.add_argument('--rate', type=float, default=30.0)
    ap.add_argument('--port', type=int, default=18899)
    ap.add_argument('--no-launch', action='store_true')
    ap.add_argument('--out', default=str(ROOT / 'validation/six_dof.json'))
    args = ap.parse_args()

    log = (ROOT / 'validation/six_dof_launch.log').open('w') if not args.no_launch else None
    proc = subprocess.Popen(
        ['ros2', 'launch', 'uw_robot_bringup', 'simulation.launch.py',
         'headless:=true', 'camera:=false', 'publish_truth:=true', 'tcp_port:=%d' % args.port],
        stdout=log, stderr=subprocess.STDOUT) if not args.no_launch else None
    rclpy.init()
    node = rclpy.create_node('six_dof_test')
    sampler = Sampler(node)
    pub = node.create_publisher(ControlCommand, 'control/cmd', 10)
    seq = [0]
    result = {'config': {'linear_hold_s': args.hold, 'angular_hold_s': args.angular_hold,
                         'settle_s': args.settle, 'rate_hz': args.rate,
                         'full_scale': {'u': 1.0, 'v': 1.0, 'w': 1.0,
                                        'p_dps': 25.0, 'q_dps': 25.0, 'r_dps': 25.0}},
              'axes': []}
    try:
        deadline = time.monotonic() + 40
        while time.monotonic() < deadline:
            if proc is not None and proc.poll() is not None:
                raise RuntimeError('仿真启动失败，见 validation/six_dof_launch.log')
            rclpy.spin_once(node, timeout_sec=.05)
            sampler.record()
            if sampler.status is not None and pub.get_subscription_count() > 0:
                break

        def emit(vel, arm=False, estop=False, depth=False, loop=False):
            m = ControlCommand()
            m.header.stamp = node.get_clock().now().to_msg()
            m.header.frame_id = 'base_link'
            m.sequence = seq[0]
            seq[0] += 1
            m.velocity = list(vel)
            m.arm = arm
            m.estop = estop
            m.depth_toggle = depth
            m.loop_toggle = loop
            pub.publish(m)

        def hold(seconds, vel, arm=False):
            end = time.monotonic() + seconds
            period = 1.0 / args.rate
            while time.monotonic() < end:
                emit(vel, arm)
                rclpy.spin_once(node, timeout_sec=.002)
                sampler.record()
                time.sleep(period)

        zero = (0.0,) * 6
        # 等 IMU 有效（仿真有 2 s 零点校准）
        end = time.monotonic() + 25
        while time.monotonic() < end:
            hold(.1, zero)
            if sampler.status is not None and sampler.status.imu_valid:
                break
        # 解锁：先确保 arm=0 若干拍，再连续发 arm=1，然后轮询状态直到确认
        armed_any = False
        for attempt in range(5):
            hold(.06, zero)                       # 保证上升沿
            hold(.12, zero, arm=True)             # 连续几拍 arm=1
            poll_end = time.monotonic() + 1.0
            while time.monotonic() < poll_end:
                hold(.05, zero)
                if sampler.status is not None and sampler.status.armed:
                    armed_any = True
                    break
            if armed_any:
                break
        print('  [arm] subs=%d imu=%s armed=%s state=%s' % (
            pub.get_subscription_count(),
            sampler.status.imu_valid if sampler.status else None,
            armed_any,
            sampler.status.state if sampler.status else None), flush=True)
        hold(.4, zero)
        result['armed'] = armed_any
        result['imu_valid'] = bool(sampler.status and sampler.status.imu_valid)

        def level(max_seconds=10.0, tol_deg=3.0):
            gains = math.radians(8.0)
            limit = math.radians(20.0)
            end = time.monotonic() + max_seconds
            while time.monotonic() < end:
                r, p_, _ = euler(sampler._quat)
                if abs(math.degrees(r)) < tol_deg and abs(math.degrees(p_)) < tol_deg:
                    break
                roll_rate = max(-limit, min(limit, -gains * r))
                pitch_rate = max(-limit, min(limit, -gains * p_))
                hold(.08, (0.0, 0.0, 0.0, roll_rate, pitch_rate, 0.0))
            hold(.3, zero)
            r, p_, _ = euler(sampler._quat)
            return math.degrees(r), math.degrees(p_)

        for index, name in enumerate(AXES):
            angular = ANGULAR[index]
            roll0, pitch0 = level()
            hold(1.5, zero)                      # 让上一轴的残余速度衰减
            t0 = time.monotonic()
            s0 = sampler.at(t0)
            target = list(zero)
            target[index] = FULL[index]
            hold(args.angular_hold if angular else args.hold, tuple(target))
            t1 = time.monotonic()
            s1 = sampler.at(t1)
            if angular:
                hold(args.angular_hold, tuple(-v for v in target))
            hold(args.settle, zero)
            t2 = time.monotonic()
            s2 = sampler.at(t2)

            dt = t1 - t0
            dp = [s1[1][k] - s0[1][k] for k in range(3)]
            e0, e1 = euler(s0[2]), euler(s1[2])
            de = [math.degrees(wrap(e1[k] - e0[k])) for k in range(3)]
            gyro_tail = sampler.tail_mean(t1 - .3, t1, 3)
            prev = sampler.at(t1 - .5)
            v_tail = [(s1[1][k] - prev[1][k]) / .5 for k in range(3)]
            drift = [s2[1][k] - s1[1][k] for k in range(3)]
            prev0 = sampler.at(t0 - .3)
            v_start = [0.0, 0.0, 0.0] if prev0 is None else [
                (s0[1][k] - prev0[1][k]) / .3 for k in range(3)]
            wr = [w for (ts, w) in sampler.wrenches if t1 - .5 <= ts <= t1]
            wrench_tail = [round(sum(w[k] for w in wr) / len(wr), 3) for k in range(6)] if wr else None
            entry = {
                'axis': name, 'angular': angular, 'duration_s': round(dt, 3),
                'speed_at_start': [round(v, 4) for v in v_start],
                'wrench_tail': wrench_tail,
                'level_start_deg': [round(roll0, 2), round(pitch0, 2)],
                'delta_pos_m': [round(v, 4) for v in dp],
                'mean_speed_mps': [round(v / dt, 4) for v in dp],
                'tail_speed_mps': [round(v, 4) for v in v_tail],
                'delta_euler_deg': [round(v, 2) for v in de],
                'tail_gyro_dps': [round(math.degrees(v), 2) for v in gyro_tail],
                'settle_drift_m': [round(v, 4) for v in drift],
                'state_after': sampler.status.state if sampler.status else '',
            }
            if sampler.status is not None and sampler.status.armed:
                armed_any = True
            result['axes'].append(entry)
            print('%-9s dt=%.2f dPos=%-24s vTail=%-24s dEuler=%-20s gyro=%s' % (
                name, dt, entry['delta_pos_m'], entry['tail_speed_mps'],
                entry['delta_euler_deg'], entry['tail_gyro_dps']), flush=True)
            hold(.3, zero)
        result['ok'] = True
    finally:
        try:
            for _ in range(3):
                emit((0.0,) * 6, estop=True)
                rclpy.spin_once(node, timeout_sec=.02)
        except Exception:
            pass
        if proc is not None and proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
        if log is not None:
            log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    Path(args.out).write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    print('\n报告: ' + args.out)
    return 0 if result.get('ok') else 1


if __name__ == '__main__':
    raise SystemExit(main())
