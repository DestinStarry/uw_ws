#!/usr/bin/env python3
"""松杆保持测试：给一段角速度指令后松开，测量姿态漂移与残余角速度。

用法：
    python3 tools/attitude_hold_test.py                    # 三轴各 3 次
    python3 tools/attitude_hold_test.py --config /tmp/v.yaml
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
from uw_robot_interfaces.msg import ControlCommand, RobotStatus

ROOT = Path(__file__).resolve().parents[1]
AXES = (('p 横滚', 3, 0), ('q 俯仰', 4, 1), ('r 偏航', 5, 2))


def euler(q):
    w, x, y, z = q
    return (math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
            math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))),
            math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--trials', type=int, default=3)
    ap.add_argument('--rate-deg', type=float, default=20.0)
    ap.add_argument('--drive', type=float, default=2.0, help='转多久后松手')
    ap.add_argument('--observe', type=float, default=5.0, help='松手后观察多久')
    ap.add_argument('--config', default=None)
    ap.add_argument('--out', default=str(ROOT / 'validation/attitude_hold.json'))
    args = ap.parse_args()

    config = args.config or str(ROOT / 'src/uw_robot_bringup/config/robot.yaml')
    log = (ROOT / 'validation/attitude_hold_launch.log').open('w')
    proc = subprocess.Popen(
        ['ros2', 'launch', 'uw_robot_bringup', 'simulation.launch.py',
         'headless:=true', 'camera:=false', 'publish_truth:=true',
         'tcp_port:=18890', 'config:=' + config],
        stdout=log, stderr=subprocess.STDOUT)
    rclpy.init()
    node = rclpy.create_node('attitude_hold_test')
    pub = node.create_publisher(ControlCommand, 'control/cmd', 10)
    st = {'status': None, 'quat': (1.0, 0.0, 0.0, 0.0), 'gyro': (0.0, 0.0, 0.0)}
    node.create_subscription(RobotStatus, 'robot/status', lambda m: st.update(status=m), 10)
    node.create_subscription(Imu, 'imu/data',
                             lambda m: st.update(gyro=(m.angular_velocity.x,
                                                       m.angular_velocity.y,
                                                       m.angular_velocity.z)),
                             qos_profile_sensor_data)
    node.create_subscription(PoseStamped, 'simulation/pose',
                             lambda m: st.update(quat=(m.pose.orientation.w, m.pose.orientation.x,
                                                       m.pose.orientation.y, m.pose.orientation.z)),
                             10)
    seq = [0]
    result = {'config_file': config, 'trials': []}
    trace = deque(maxlen=20000)
    try:
        end = time.monotonic() + 40
        while time.monotonic() < end:
            if proc.poll() is not None:
                raise RuntimeError('仿真启动失败')
            rclpy.spin_once(node, timeout_sec=.05)
            if st['status'] is not None and pub.get_subscription_count() > 0 and st['status'].imu_valid:
                break

        def emit(v, arm=False):
            m = ControlCommand()
            m.header.stamp = node.get_clock().now().to_msg()
            m.header.frame_id = 'base_link'
            m.sequence = seq[0]
            seq[0] += 1
            m.velocity = list(v)
            m.arm = arm
            pub.publish(m)

        def hold(seconds, v, arm=False, record=False):
            t_end = time.monotonic() + seconds
            while time.monotonic() < t_end:
                emit(v, arm)
                rclpy.spin_once(node, timeout_sec=.002)
                if record:
                    tr = euler(st['quat'])
                    trace.append((time.monotonic(), [math.degrees(x) for x in tr],
                                  [math.degrees(g) for g in st['gyro']]))
                time.sleep(.025)

        zero = (0.0,) * 6
        for _ in range(6):
            emit(zero)
            rclpy.spin_once(node, timeout_sec=.01)
            time.sleep(.03)
        for _ in range(4):
            hold(.12, zero, arm=True)
            hold(.15, zero)
            if st['status'] is not None and st['status'].armed:
                break
        hold(.4, zero)
        if not (st['status'] and st['status'].armed):
            raise RuntimeError('解锁失败')

        gains, limit = math.radians(8.0), math.radians(20.0)

        def level(max_seconds=12.0, tol_deg=2.0):
            end = time.monotonic() + max_seconds
            while time.monotonic() < end:
                r, p_, _ = euler(st['quat'])
                if abs(math.degrees(r)) < tol_deg and abs(math.degrees(p_)) < tol_deg:
                    break
                rr = max(-limit, min(limit, -gains * r))
                pp = max(-limit, min(limit, -gains * p_))
                hold(.08, (0.0, 0.0, 0.0, rr, pp, 0.0))
            hold(.3, zero)

        rate = math.radians(args.rate_deg)
        for name, vidx, eidx in AXES:
            for trial in range(args.trials):
                level()
                hold(1.0, zero)
                trace.clear()
                cmd = [0.0] * 6
                cmd[vidx] = rate
                hold(args.drive, tuple(cmd), record=True)
                rel_t = time.monotonic()
                rel_angle = math.degrees(euler(st['quat'])[eidx])
                hold(args.observe, zero, record=True)
                samples = [s for s in trace if s[0] >= rel_t]
                end_angle = math.degrees(euler(st['quat'])[eidx])
                tail = [s for s in samples if s[0] >= rel_t + args.observe - 2.0]
                drift = end_angle - rel_angle
                if tail:
                    span = tail[-1][0] - tail[0][0]
                    drift_rate = (tail[-1][1][eidx] - tail[0][1][eidx]) / max(1e-6, span)
                    resid = sum(abs(s[2][eidx]) for s in tail) / len(tail)
                else:
                    drift_rate = resid = 0.0
                entry = {'axis': name, 'trial': trial + 1,
                         'angle_at_release_deg': round(rel_angle, 2),
                         'drift_after_release_deg': round(drift, 2),
                         'drift_rate_dps': round(drift_rate, 2),
                         'residual_rate_dps': round(resid, 2)}
                result['trials'].append(entry)
                print('  %-8s 第%d次 松手时 %.1f°  5 s 后漂 %.2f°（%.2f °/s）  残余 %.2f °/s'
                      % (name, trial + 1, rel_angle, drift, drift_rate, resid), flush=True)
    finally:
        try:
            for _ in range(3):
                m = ControlCommand()
                m.header.stamp = node.get_clock().now().to_msg()
                m.velocity = [0.0] * 6
                m.estop = True
                pub.publish(m)
                rclpy.spin_once(node, timeout_sec=.02)
        except Exception:
            pass
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
        log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print('\n汇总（松手 5 s 内漂移，越小越好）:')
    for name, _, _ in AXES:
        vals = [t['drift_after_release_deg'] for t in result['trials'] if t['axis'] == name]
        rates = [t['drift_rate_dps'] for t in result['trials'] if t['axis'] == name]
        res = [t['residual_rate_dps'] for t in result['trials'] if t['axis'] == name]
        if vals:
            print('  %-8s 平均漂移 %+.2f°  最大 %+.2f°  末段净速率 %+.2f °/s  残余摆动 %.2f °/s'
                  % (name, sum(vals) / len(vals), max(vals, key=abs),
                     sum(rates) / len(rates), sum(res) / len(res)))
    Path(args.out).write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    print('报告: ' + args.out)


if __name__ == '__main__':
    raise SystemExit(main())
