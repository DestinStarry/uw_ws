#!/usr/bin/env python3
"""六自由度自动测试：逐轴施加满杆速度指令，记录 MuJoCo 运动并给出指标。

每轴流程：回平 -> 中立 -> 满杆保持 -> （角轴）反向回绕 -> 回中观察。
线轴测位移与稳态速度；角轴测角速度与转角。测试前用 P 控制把姿态拉回水平，
避免上一轴的大倾角污染下一轴。

用法：
    python3 tools/six_dof_test.py                # 自启无头仿真（端口 18899）
    python3 tools/six_dof_test.py --no-launch     # 连接已在运行的仿真
"""
import argparse
import json
import math
import os
import secrets
import signal
import socket
import subprocess
import time
from collections import deque
from pathlib import Path

os.environ['ROS_DOMAIN_ID'] = str(20 + secrets.randbelow(40))
os.environ['ROS_LOCALHOST_ONLY'] = '0'
os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(
    Path(__file__).with_name('dds_test.xml'))

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from uw_robot_interfaces.msg import DepthState, RobotStatus

ROOT = Path(__file__).resolve().parents[1]
FULL = (1.0, 1.0, 0.25, math.radians(25.0), math.radians(22.0), math.radians(20.0))
AXES = ('u 纵荡', 'v 横荡', 'w 升沉', 'p 横滚', 'q 俯仰', 'r 偏航')
ANGULAR = (False, False, False, True, True, True)


def frame(vel, arm=0, estop=0, depth=0, loop=0):
    body = ','.join('%.6f' % v for v in vel)
    return 'CMD:%s,%d,%d,%d,%d' % (body, arm, estop, depth, loop)


def euler(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


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

    def record(self):
        if self._pos is not None:
            self.samples.append((time.monotonic(), self._pos, self._quat,
                                 self._gyro, self._depth))

    def at(self, when):
        best = None
        for s in reversed(self.samples):
            if s[0] <= when:
                best = s
                break
        return best

    def tail_mean(self, t0, t1, field):
        rows = [s for s in self.samples if t0 <= s[0] <= t1]
        if not rows:
            return [0.0, 0.0, 0.0]
        return [sum(r[field][k] for r in rows) / len(rows) for k in range(3)]


def launch_sim(port, log):
    return subprocess.Popen(
        ['ros2', 'launch', 'uw_robot_bringup', 'simulation.launch.py',
         'headless:=true', 'camera:=false', 'publish_truth:=true', 'tcp_port:=%d' % port],
        stdout=log, stderr=subprocess.STDOUT)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--hold', type=float, default=3.0, help='线轴满杆保持秒数')
    ap.add_argument('--angular-hold', type=float, default=0.8, help='角轴满杆保持秒数')
    ap.add_argument('--settle', type=float, default=2.0, help='回中观察秒数')
    ap.add_argument('--rate', type=float, default=30.0, help='指令发送频率 Hz')
    ap.add_argument('--port', type=int, default=18899)
    ap.add_argument('--no-launch', action='store_true')
    ap.add_argument('--out', default=str(ROOT / 'validation/six_dof.json'))
    args = ap.parse_args()

    log_path = ROOT / 'validation/six_dof_launch.log'
    log = log_path.open('w') if not args.no_launch else None
    proc = launch_sim(args.port, log) if not args.no_launch else None
    rclpy.init()
    node = rclpy.create_node('six_dof_test')
    sampler = Sampler(node)
    client = None
    result = {'config': {'linear_hold_s': args.hold, 'angular_hold_s': args.angular_hold,
                         'settle_s': args.settle, 'rate_hz': args.rate,
                         'full_scale': {'u_mps': 1.0, 'v_mps': 1.0, 'w_mps': 0.25,
                                        'p_dps': 25.0, 'q_dps': 22.0, 'r_dps': 20.0}},
              'axes': []}
    try:
        deadline = time.monotonic() + 40
        while time.monotonic() < deadline:
            if proc is not None and proc.poll() is not None:
                raise RuntimeError('仿真启动失败，见 ' + str(log_path))
            if client is None:
                try:
                    client = socket.create_connection(('127.0.0.1', args.port), timeout=.2)
                    client.setblocking(False)
                except OSError:
                    pass
            rclpy.spin_once(node, timeout_sec=.02)
            sampler.record()
            if client and sampler.status is not None:
                break
        if not client:
            raise RuntimeError('TCP 未连上 127.0.0.1:%d' % args.port)

        link = {'n': 0, 'ready': False, 'stale': None}

        def connect():
            nonlocal client
            try:
                if client is not None:
                    client.close()
            except OSError:
                pass
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline:
                try:
                    s = socket.create_connection(('127.0.0.1', args.port), timeout=.5)
                    s.setblocking(False)
                    client = s
                    link['n'] += 1
                    print('  [link] 重新连接 #%d' % link['n'], flush=True)
                    return True
                except OSError:
                    time.sleep(.1)
            return False

        def arm():
            for _ in range(3):
                try:
                    client.sendall((frame((0.0,) * 6, 1) + '\n').encode())
                except (BlockingIOError, OSError):
                    return
                rclpy.spin_once(node, timeout_sec=.01)
                time.sleep(.03)

        def send(vel, arm_value=0):
            # 必须同时排空回传：桥的发送缓冲到 8 KB 会主动断开并急停。
            now = time.monotonic()
            last = link.get('last_send')
            if last is not None and now - last > .2:
                print('  [gap] 发送间隔 %.2f s（上一状态 %s）' % (
                    now - last, states['last']), flush=True)
            link['last_send'] = now
            try:
                client.sendall((frame(vel, arm_value) + '\n').encode())
            except (BlockingIOError, OSError):
                if link['ready'] and connect():
                    arm()
                return
            try:
                client.recv(8192)
            except (BlockingIOError, OSError):
                pass

        states = {'last': None, 't': time.monotonic()}

        def hold(seconds, vel, arm_value=0):
            end = time.monotonic() + seconds
            period = 1.0 / args.rate
            while time.monotonic() < end:
                send(vel, arm_value)
                rclpy.spin_once(node, timeout_sec=.002)
                sampler.record()
                st = sampler.status
                if st is not None and st.state != states['last']:
                    print('  [state] %-12s -> %-13s at %6.2f s' % (
                        states['last'], st.state, time.monotonic() - states['t']), flush=True)
                    states['last'] = st.state
                # 只在确认未解锁时补按 A；A 是边沿触发，状态回传仅 10 Hz，
                # 用旧状态重复按会把已经解锁的控制器又锁上。
                if (link['ready'] and st is not None and not st.armed
                        and st.state != 'WAIT_IMU'
                        and time.monotonic() > link.get('cooldown', 0.0)):
                    if link['stale'] is None:
                        link['stale'] = time.monotonic()
                    elif time.monotonic() - link['stale'] > .3:
                        if st.state == 'CMD_STALE':
                            connect()
                        arm()
                        link['cooldown'] = time.monotonic() + .6
                        link['stale'] = None
                else:
                    link['stale'] = None
                time.sleep(period)

        zero = (0.0,) * 6

        def level(max_seconds=8.0, tol_deg=3.0):
            """用 P 控制把 roll/pitch 拉回水平，返回最终倾角。"""
            end = time.monotonic() + max_seconds
            gains = math.radians(8.0)   # 每度误差给 8°/s 目标角速度
            while time.monotonic() < end and sampler._quat is not None:
                roll, pitch, _ = euler(sampler._quat)
                if abs(math.degrees(roll)) < tol_deg and abs(math.degrees(pitch)) < tol_deg:
                    break
                rate = [-gains * roll, -gains * pitch, 0.0]
                limit = math.radians(20.0)
                rate = [max(-limit, min(limit, v)) for v in rate]
                hold(.08, tuple(rate))
            hold(.3, zero)
            roll, pitch, _ = euler(sampler._quat)
            return math.degrees(roll), math.degrees(pitch)

        # 仿真 IMU 有 2 s 零点校准：必须等 imu_valid 再解锁，否则会被 WAIT_IMU 拦下。
        end = time.monotonic() + 20
        while time.monotonic() < end:
            hold(.1, zero)
            if sampler.status is not None and sampler.status.imu_valid:
                break
        for _ in range(3):
            send(zero, 1)
            rclpy.spin_once(node, timeout_sec=.01)
            time.sleep(.02)
        hold(.5, zero)
        # 解锁失败重试，避免单帧丢失导致整轮测试无效。
        if not (sampler.status and sampler.status.armed):
            for _ in range(2):
                hold(.1, zero)
                send(zero, 1)
                time.sleep(.05)
            hold(.3, zero)
        link['ready'] = True
        result['armed'] = bool(sampler.status and sampler.status.armed)
        result['imu_valid'] = bool(sampler.status and sampler.status.imu_valid)

        for index, name in enumerate(AXES):
            angular = ANGULAR[index]
            roll0, pitch0 = level()
            hold(.4, zero)
            t0 = time.monotonic()
            s0 = sampler.at(t0)
            target = list(zero)
            target[index] = FULL[index]
            hold(args.angular_hold if angular else args.hold, tuple(target))
            t1 = time.monotonic()
            s1 = sampler.at(t1)
            if angular:
                hold(args.angular_hold, tuple(-v for v in target))   # 反向回绕
            hold(args.settle, zero)
            t2 = time.monotonic()
            s2 = sampler.at(t2)

            dt = t1 - t0
            dp = [s1[1][k] - s0[1][k] for k in range(3)]
            e0, e1, e2 = euler(s0[2]), euler(s1[2]), euler(s2[2])
            de = [math.degrees(wrap(e1[k] - e0[k])) for k in range(3)]
            gyro_tail = sampler.tail_mean(t1 - .3, t1, 3)
            v_tail = [(s1[1][k] - sampler.at(t1 - .5)[1][k]) / .5 for k in range(3)]
            drift = [s2[1][k] - s1[1][k] for k in range(3)]
            depth_delta = (s1[4] - s0[4]) if s1[4] is not None and s0[4] is not None else None

            entry = {
                'axis': name,
                'angular': angular,
                'duration_s': round(dt, 3),
                'level_start_deg': [round(roll0, 2), round(pitch0, 2)],
                'delta_pos_m': [round(v, 4) for v in dp],
                'mean_speed_mps': [round(v / dt, 4) for v in dp],
                'tail_speed_mps': [round(v, 4) for v in v_tail],
                'delta_euler_deg': [round(v, 2) for v in de],
                'tail_gyro_dps': [round(math.degrees(v), 2) for v in gyro_tail],
                'settle_drift_m': [round(v, 4) for v in drift],
                'depth_delta_m': None if depth_delta is None else round(depth_delta, 4),
                'state_after': sampler.status.state if sampler.status else '',
            }
            result['axes'].append(entry)
            print('%-10s dt=%.2f  dPos=%-28s tailV=%-26s dEuler=%-24s gyro=%s' % (
                name, dt, entry['delta_pos_m'], entry['tail_speed_mps'],
                entry['delta_euler_deg'], entry['tail_gyro_dps']), flush=True)
            hold(.3, zero)

        result['ok'] = True
    finally:
        if client:
            try:
                client.sendall((frame((0.0,) * 6, 0, 1) + '\n').encode())
            except OSError:
                pass
            client.close()
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

    Path(args.out).write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n',
                              encoding='utf-8')
    print('\n报告: ' + args.out)
    return 0 if result.get('ok') else 1


if __name__ == '__main__':
    raise SystemExit(main())
