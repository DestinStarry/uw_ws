"""离线自查：用 9.32 kg + 15/40/40 阻尼的质点模型替代 MuJoCo，验证轨迹与控制律。

不需要 ROS、不需要 colcon build，只要有 numpy：
    python3 tools/trajectory_offline_check.py

它不能替代 tools/trajectory_validation.py（后者才跑真实的分配器、舵机与推力限速），
但能在编译或仿真环境出问题时，第一时间区分"数学错了"还是"链路错了"。
"""
import sys
from pathlib import Path

import numpy as np

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / 'src/uw_robot_validation'))

from uw_robot_validation.controller import TrackingController  # noqa: E402
from uw_robot_validation.metrics import summarize  # noqa: E402
from uw_robot_validation.trajectory import Trajectory  # noqa: E402

MASS = 9.32
DRAG = np.array([15.0, 40.0, 40.0])
CASES = (('circle', 0.25, 16.0, 1.0, 0.0, 20.0), ('helix', 0.20, 12.0, 2.0, 0.08, 28.0))


def simulate(kind, radius, period, turns, z_rate, duration, dt=0.02):
    trajectory = Trajectory(kind=kind, radius=radius, period=period, turns=turns,
                            blend_s=4.0, z_rate=z_rate)
    controller = TrackingController(mass=MASS, drag=DRAG)
    position, velocity = np.zeros(3), np.zeros(3)
    quaternion = np.array([1.0, 0.0, 0.0, 0.0])
    records, t = [], 0.0
    while t < duration:
        reference = trajectory.sample(t)
        wrench, info = controller.step(reference, position, quaternion, np.zeros(3), dt)
        acceleration = (wrench[:3] - DRAG * velocity) / MASS
        velocity = velocity + acceleration * dt
        position = position + velocity * dt
        if t >= 4.0:
            records.append(dict(t=t, position=position.copy(), reference=reference['position'].copy(),
                                origin=np.zeros(3), quaternion=quaternion.copy(),
                                yaw_ref=reference['yaw'], wrench=wrench.copy(),
                                velocity_estimate=info['velocity_estimate'].copy(),
                                velocity_ref=reference['velocity'].copy(), clipped=False,
                                saturated=info['saturated_force']))
        t += dt
    return summarize(records, (6.0, duration), radius,
                     dict(max_rms=0.05, max_peak=0.12, max_radial_rms=0.05,
                          max_tilt_deg=5.0, max_clip_fraction=0.05))


def main():
    failed = 0
    for spec in CASES:
        report = simulate(*spec)
        if 'error' in report:
            print('%s: %s' % (spec[0], report['error']))
            failed += 1
            continue
        print('%s  rms=%.4f m  peak=%.4f m  radial=%.4f m  force=%.2f N  %s' % (
            spec[0], report['error_3d']['rms'], report['error_3d']['max'],
            report['path']['radial']['rms'], report['command']['force']['max'],
            '通过' if report['passed'] else '未通过'))
        failed += 0 if report['passed'] else 1
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
