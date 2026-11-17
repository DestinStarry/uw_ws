#!/usr/bin/env python3
"""横滚专项测试：重复多次同样的横滚指令，量化稳态、波动与起动时间。

用法：
    python3 tools/roll_test.py                       # 当前 robot.yaml，4 次
    python3 tools/roll_test.py --trials 6 --hold 3
    python3 tools/roll_test.py --config /tmp/variant.yaml   # 换一份控制参数
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
from uw_robot_interfaces.msg import ControlCommand, RobotStatus, WrenchCommand

ROOT = Path(__file__).resolve().parents[1]


def euler(q):
    w, x, y, z = q
    return (math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
            math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))),
            math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--trials', type=int, default=4)
    ap.add_argument('--rate-deg', type=float, default=25.0)
    ap.add_argument('--hold', type=float, default=3.0)
    ap.add_argument('--rest', type=float, default=1.5)
    ap.add_argument('--rate-hz', type=float, default=40.0)
    ap.add_argument('--config', default=None, help='控制参数 yaml（默认 robot.yaml）')
    ap.add_argument('--out', default=str(ROOT / 'validation/roll_test.json'))
    args = ap.parse_args()

    config = args.config or str(ROOT / 'src/uw_robot_bringup/config/robot.yaml')
    log = (ROOT / 'validation/roll_test_launch.log').open('w')
    proc = subprocess.Popen(
        ['ros2', 'launch', 'uw_robot_bringup', 'simulation.launch.py',
         'headless:=true', 'camera:=false', 'publish_truth:=true',
         'tcp_port:=18892', 'config:=' + config],
        stdout=log, stderr=subprocess.STDOUT)
    rclpy.init()
    node = rclpy.create_node('roll_test')
    pub = node.create_publisher(ControlCommand, 'control/cmd', 10)
    st = {'status': None, 'quat': (1.0, 0.0, 0.0, 0.0), 'gyro': (0.0, 0.0, 0.0),
          'wrench': None, 'pos': (0.0, 0.0, 0.0)}
    node.create_subscription(RobotStatus, 'robot/status', lambda m: st.update(status=m), 10)
    node.create_subscription(Imu, 'imu/data',
                             lambda m: st.update(gyro=(m.angular_velocity.x,
                                                       m.angular_velocity.y,
                                                       m.angular_velocity.z)),
                             qos_profile_sensor_data)
    node.create_subscription(PoseStamped, 'simulation/pose',
                             lambda m: st.update(
                                 quat=(m.pose.orientation.w, m.pose.orientation.x,
                                       m.pose.orientation.y, m.pose.orientation.z),
                                 pos=(m.pose.position.x, m.pose.position.y, m.pose.position.z)), 10)
    node.create_subscription(WrenchCommand, 'control/wrench',
                             lambda m: st.update(wrench=list(m.wrench)), 10)
    seq = [0]
    trace = deque(maxlen=20000)
    result = {'config_file': config, 'trials': [], 'cmd_dps': args.rate_deg}
    try:
        end = time.monotonic() + 40
        while time.monotonic() < end:
            if proc.poll() is not None:
                raise RuntimeError('仿真启动失败，见 validation/roll_test_launch.log')
            rclpy.spin_once(node, timeout_sec=.05)
            if st['status'] is not None and pub.get_subscription_count() > 0 and st['status'].imu_valid:
                break

        def emit(v, arm=False, estop=False):
            m = ControlCommand()
            m.header.stamp = node.get_clock().now().to_msg()
            m.header.frame_id = 'base_link'
            m.sequence = seq[0]
            seq[0] += 1
            m.velocity = list(v)
            m.arm = arm
            m.estop = estop
            pub.publish(m)

        period = 1.0 / args.rate_hz

        def hold(seconds, v, arm=False, record=False):
            t_end = time.monotonic() + seconds
            while time.monotonic() < t_end:
                emit(v, arm)
                rclpy.spin_once(node, timeout_sec=.002)
                if record:
                    tr = euler(st['quat'])
                    w = st['wrench'] or [0.0] * 6
                    trace.append((time.monotonic(),
                                  (math.degrees(tr[0]), math.degrees(tr[1]), math.degrees(tr[2])),
                                  (math.degrees(st['gyro'][0]), math.degrees(st['gyro'][1]),
                                   math.degrees(st['gyro'][2])),
                                  (w[3], w[4], w[5]),
                                  st['pos']))
                time.sleep(period)

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
            raise RuntimeError('解锁失败: state=' + str(st['status'].state if st['status'] else None))
        print('已解锁，开始 %d 次横滚测试（指令 %.1f °/s，每次 %.1f s）'
              % (args.trials, args.rate_deg, args.hold), flush=True)

        gains = math.radians(8.0)
        limit = math.radians(20.0)

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
            return math.degrees(euler(st['quat'])[0])

        rate = math.radians(args.rate_deg)
        for trial in range(args.trials):
            roll0 = level()
            hold(args.rest, zero)
            trace.clear()
            t0 = time.monotonic()
            hold(args.hold, (0.0, 0.0, 0.0, rate, 0.0, 0.0), record=True)
            t1 = time.monotonic()
            samples = [s for s in trace if t0 <= s[0] <= t1]
            dt = t1 - t0
            roll_end = math.degrees(euler(st['quat'])[0])
            win = [s for s in samples if s[0] >= t1 - 1.0]
            gx = [s[2][0] for s in win]
            gy = [s[2][1] for s in win]
            gz = [s[2][2] for s in win]
            mx = [s[3][0] for s in win]
            my = [s[3][1] for s in win]
            mz = [s[3][2] for s in win]
            if samples:
                a0, a1 = samples[0][1], samples[-1][1]
                dep = a1[1] - a0[1]
                dey = a1[2] - a0[2]
                der = a1[0] - a0[0]
                p0, p1 = samples[0][4], samples[-1][4]
                dpos = [p1[k] - p0[k] for k in range(3)]
            else:
                dep = dey = der = 0.0
                dpos = [0.0, 0.0, 0.0]
            if len(win) > 1:
                dw = win[-1][0] - win[0][0]
                vel = [(win[-1][4][k] - win[0][4][k]) / max(1e-6, dw) for k in range(3)]
            else:
                vel = [0.0, 0.0, 0.0]
            mean = sum(gx) / len(gx) if gx else 0.0
            var = sum((g - mean) ** 2 for g in gx) / len(gx) if gx else 0.0
            rise = None
            for s in samples:
                if abs(s[2][0]) >= 0.9 * args.rate_deg:
                    rise = s[0] - t0
                    break
            crossings = 0
            for a, b in zip(gx, gx[1:]):
                if (a - mean) * (b - mean) < 0:
                    crossings += 1
            entry = {
                'trial': trial + 1,
                'roll_start_deg': round(roll0, 2),
                'roll_end_deg': round(roll_end, 2),
                'delta_roll_deg': round(roll_end - roll0, 2),
                'mean_rate_dps': round(mean, 2),
                'std_rate_dps': round(math.sqrt(var), 2),
                'peak_rate_dps': round(max((abs(g) for g in gx), default=0.0), 2),
                'min_rate_dps': round(min(gx), 2) if gx else 0.0,
                'rise_time_s': None if rise is None else round(rise, 2),
                'mean_mx_nm': round(sum(mx) / len(mx), 3) if mx else None,
                'std_mx_nm': round(math.sqrt(sum((m - sum(mx) / len(mx)) ** 2 for m in mx) / len(mx)), 3) if mx else None,
                'rate_crossings': crossings,
                'delta_roll_deg': round(der, 2),
                'delta_pitch_deg': round(dep, 2),
                'delta_yaw_deg': round(dey, 2),
                'mean_gx_dps': round(mean, 2),
                'mean_gy_dps': round(sum(gy) / len(gy), 2) if gy else 0.0,
                'mean_gz_dps': round(sum(gz) / len(gz), 2) if gz else 0.0,
                'mean_my_nm': round(sum(my) / len(my), 3) if my else 0.0,
                'mean_mz_nm': round(sum(mz) / len(mz), 3) if mz else 0.0,
                'delta_pos_m': [round(v, 3) for v in dpos],
                'mean_vel_mps': [round(v, 3) for v in vel],
                'duration_s': round(dt, 2),
            }
            result['trials'].append(entry)
            print('  第%d次 roll %+.1f° 速率 %.1f±%.1f 峰%.1f 起动%s'
                  ' | 角度耦合 p%+.1f y%+.1f | 位移 x%+.3f y%+.3f z%+.3f m'
                  ' | Mx %.2f My %.2f Mz %.2f'
                  % (entry['trial'], entry['delta_roll_deg'], entry['mean_rate_dps'],
                     entry['std_rate_dps'], entry['peak_rate_dps'], entry['rise_time_s'],
                     entry['delta_pitch_deg'], entry['delta_yaw_deg'],
                     entry['delta_pos_m'][0], entry['delta_pos_m'][1], entry['delta_pos_m'][2],
                     entry['mean_mx_nm'] or 0, entry['mean_my_nm'], entry['mean_mz_nm']),
                  flush=True)
            hold(args.hold, (0.0, 0.0, 0.0, -rate, 0.0, 0.0))
            hold(.4, zero)
            result['ok'] = True
    finally:
        try:
            for _ in range(3):
                emit((0.0,) * 6, estop=True)
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

    if result['trials']:
        means = [t['mean_rate_dps'] for t in result['trials']]
        stds = [t['std_rate_dps'] for t in result['trials']]
        result['summary'] = {
            'trials': len(means),
            'mean_rate_dps': round(sum(means) / len(means), 2),
            'spread_between_trials_dps': round(max(means) - min(means), 2),
            'mean_std_within_trial_dps': round(sum(stds) / len(stds), 2),
            'cmd_dps': args.rate_deg,
        }
        print('\n汇总: %d 次  稳态均值 %.1f °/s（指令 %.1f）  次间极差 %.1f  次内波动 %.1f'
              % (len(means), result['summary']['mean_rate_dps'], args.rate_deg,
                 result['summary']['spread_between_trials_dps'],
                 result['summary']['mean_std_within_trial_dps']))
    Path(args.out).write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    print('报告: ' + args.out)


if __name__ == '__main__':
    raise SystemExit(main())
