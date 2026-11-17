"""圆形 / 螺旋线闭环轨迹验证（仅仿真，绝不打开实物）。

用法：
    source install/setup.bash
    python3 tools/trajectory_validation.py --kind both
    python3 tools/trajectory_validation.py --kind helix --radius 0.2 --turns 2

流程：headless 仿真 + allocation_node + trajectory_node，跑满设定时长，
读取节点自己落盘的 JSON（时间对齐后的跟踪误差、姿态、执行器统计）。

收尾口径：判定成功只看「报告是否生成」，不看 launch 进程是否自己退出。
ros2 launch 在子节点 rclpy.shutdown() 后仍可能卡在事件循环，之前把它记成
“超时”会让一次有效的运行看起来失败。
"""
import argparse
import json
import os
import secrets
import signal
import subprocess
import sys
import time
from pathlib import Path

root = Path(__file__).resolve().parents[1]
os.environ['ROS_DOMAIN_ID'] = str(20 + secrets.randbelow(40))
os.environ['ROS_LOCALHOST_ONLY'] = '0'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(root / 'tools/dds_test.xml')
for key in ('ROS_DISCOVERY_SERVER', 'ROS_SUPER_CLIENT', 'SKIP_DEFAULT_XML'):
    os.environ.pop(key, None)

DEFAULTS = {
    'circle': dict(radius=0.25, period=16.0, turns=1.0, z_rate=0.0),
    'helix': dict(radius=0.20, period=12.0, turns=2.0, z_rate=0.08),
}


def terminate(process):
    """整组退出：ros2 launch 会派生子进程，只 terminate 父进程会留下悬空执行器。"""
    if process.poll() is not None:
        return
    for sig in (signal.SIGINT, signal.SIGKILL):
        try:
            os.killpg(os.getpgid(process.pid), sig)
        except Exception:
            try:
                process.terminate()
            except Exception:
                pass
        try:
            process.wait(timeout=8.0)
            return
        except subprocess.TimeoutExpired:
            continue


def run(kind, args):
    spec = dict(DEFAULTS[kind])
    for key in ('radius', 'period', 'turns', 'z_rate'):
        value = getattr(args, key, None)
        if value is not None:
            spec[key] = value
    blend = args.blend_s
    duration = blend + spec['turns'] * spec['period'] + args.tail_s
    report = root / 'validation' / ('trajectory_%s.json' % kind)
    log_path = root / 'validation' / ('trajectory_%s.log' % kind)
    command = ['ros2', 'launch', 'uw_robot_validation', 'trajectory_validation.launch.py',
               'kind:=' + kind, 'radius:=%g' % spec['radius'], 'period:=%g' % spec['period'],
               'turns:=%g' % spec['turns'], 'blend_s:=%g' % blend, 'z_rate:=%g' % spec['z_rate'],
               'z_floor:=%g' % args.z_floor,
               'duration:=%g' % duration, 'settle_s:=%g' % args.settle_s,
               'direction:=%d' % args.direction, 'report:=' + str(report)]
    print('[%s] %s' % (kind, ' '.join(command)))
    if report.exists():
        report.unlink()
    deadline = time.time() + duration + args.startup_s
    with log_path.open('w') as log:
        process = subprocess.Popen(command, cwd=str(root), stdout=log,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        while time.time() < deadline:
            if report.exists() and report.stat().st_size > 0:
                break
            if process.poll() is not None:
                break
            time.sleep(0.5)
        produced = report.exists() and report.stat().st_size > 0
        if produced:
            try:
                process.wait(timeout=4.0)
            except subprocess.TimeoutExpired:
                terminate(process)
        else:
            timed_out = time.time() >= deadline and process.poll() is None
            terminate(process)
            if timed_out:
                print('[%s] 超时（%.0fs 内未生成报告），日志：%s'
                      % (kind, duration + args.startup_s, log_path))
    if not report.exists():
        tail = log_path.read_text(errors='replace').splitlines()[-25:]
        print('[%s] 未生成报告。日志尾部：' % kind)
        print('\n'.join(tail))
        return 1
    data = json.loads(report.read_text())
    print_summary(kind, data)
    return 0 if data.get('passed') else 2


def print_summary(kind, data):
    if 'error' in data:
        print('[%s] 报告异常：%s' % (kind, data['error']))
        return
    error, path = data['error_3d'], data['path']
    attitude, command = data['attitude'], data['command']
    actuator = data.get('actuator', {})
    sensors = data.get('sensors', {})
    print('[%s] 样本 %d，窗口 %s' % (kind, data['samples'], data['window_s']))
    print('  跟踪误差  rms=%.3f m  peak=%.3f m  p95=%.3f m' %
          (error['rms'], error['max'], error['p95']))
    print('  半径误差  rms=%.3f m  实测半径=%.3f m  速度=%.3f/%.3f m/s' %
          (path['radial']['rms'], path['radius_mean'], path['speed_mean'], path['speed_ref_mean']))
    print('  姿态      roll=%.2f°  pitch=%.2f°  yaw=%.2f°  峰值倾角=%.2f°' %
          (attitude['roll_deg']['rms'], attitude['pitch_deg']['rms'],
           attitude['yaw_deg']['rms'], attitude['max_tilt_deg']))
    print('  指令      力峰值=%.2f N  实现力均值=%.2f N  力矩峰值=%.2f N·m' %
          (command['force']['max'], command['realized_force']['mean'], command['moment']['max']))
    print('  分配      未实现=%.1f%%  等待规划=%.1f%%  规划有效=%.1f%%' %
          (100.0 * command['allocator_clip_fraction'],
           100.0 * command.get('wait_planner_fraction', float('nan')),
           100.0 * command.get('planner_valid_fraction', float('nan'))))
    print('  执行器    推力峰值=%.1f N  舵角峰值=%.0f°  推力变化率峰值=%.0f N/s' %
          (actuator.get('max_thrust_n', float('nan')),
           actuator.get('max_theta_deg', float('nan')),
           actuator.get('max_thrust_rate_nps', float('nan'))))
    if 'min_z_m' in data:
        note = ''
        if data.get('descent_clamped'):
            note = '（z_rate %.3f→%.4f，受 z_floor=%.2f 限制）' % (
                data['trajectory']['z_rate'], data['trajectory']['z_rate_used'],
                data['trajectory'].get('z_floor', 0.0))
        print('  深度      min_z=%.3f m  陀螺取 IMU=%.0f%%%s' %
              (data['min_z_m'], 100.0 * sensors.get('imu_fraction', float('nan')), note))
    failed = [name for name, ok in data['checks'].items() if not ok]
    print('  结论      %s%s' % ('通过' if data['passed'] else '未通过',
                               '' if not failed else '；未过项：' + ', '.join(failed)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--kind', choices=('circle', 'helix', 'both'), default='both')
    parser.add_argument('--radius', type=float)
    parser.add_argument('--period', type=float)
    parser.add_argument('--turns', type=float)
    parser.add_argument('--z-rate', dest='z_rate', type=float)
    parser.add_argument('--z-floor', dest='z_floor', type=float, default=0.25,
                        help='参考深度下限（m）；scene.xml 的 floor 在 z=0，默认留 0.25 余量')
    parser.add_argument('--blend-s', dest='blend_s', type=float, default=4.0)
    parser.add_argument('--settle-s', dest='settle_s', type=float, default=2.0)
    parser.add_argument('--tail-s', dest='tail_s', type=float, default=1.0)
    parser.add_argument('--direction', type=int, default=1, choices=(1, -1))
    parser.add_argument('--startup-s', dest='startup_s', type=float, default=25.0,
                        help='启动与收尾的额外等待预算（秒）')
    args = parser.parse_args()
    kinds = ['circle', 'helix'] if args.kind == 'both' else [args.kind]
    codes = [run(kind, args) for kind in kinds]
    return 0 if all(code == 0 for code in codes) else 1


if __name__ == '__main__':
    sys.exit(main())
