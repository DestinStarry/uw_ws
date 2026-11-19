"""控制器目标 wrench 与参考实现的差异报告（allocation 已隔离）。

自 v0.6 起控制律主动移除了手柄前馈（rate_ff），与 reference/ctrl_solver.py
不再逐位等价，因此这里**不做 1e-9 断言**，只输出最大偏差供人工判断；
真正的回归门禁是 control_test / mode_switch_test 与 six_dof 验证。
状态机（armed/closed_loop/depth_hold）仍要求完全一致。
"""
from pathlib import Path
import json
import subprocess
import sys
import numpy as np

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / 'reference'))
import ctrl_solver

# Disable the original background optimizer and allocation, to compare control
# equations independently of the deliberately changed nonlinear planner.
ctrl_solver.HardwareController._solver_worker = lambda self: None
original = ctrl_solver.HardwareController()
original._allocate = lambda *args, **kwargs: None

# 发送端满杆量程：必须与外部 CMD 发送端和 robot.yaml 的 gain 定义一致。
LINEAR_FULL = 1.0   # u, v 满杆 m/s
HEAVE_FULL = 1.0    # w 满杆（三个线轴统一 1.0）
RATE_FULL = (np.deg2rad(25.0), np.deg2rad(22.0), np.deg2rad(20.0))

lines = []
expected = []
for i in range(180):
    t = 1 + i * .02
    j = dict(lx=.1*np.sin(i*.1), ly=-.15, rx=.1, ry=-.05, lt=0., rt=.1,
             a=i in (1,130,140), b=False, x=i in (35,110), y=i==135,
             lb=False, rb=15<=i<25)
    if 60<=i<100:
        j.update(rx=0.,ry=0.,rb=False,rt=0.)
    angle=.03*np.sin(i*.04)
    q=np.array([np.cos(angle/2),0.,np.sin(angle/2),0.])
    gyro=np.array([.01*np.sin(i),.01,0.])
    imu=dict(quaternion=q,gyro=gyro,ok=True)
    depth=dict(depth=1+.01*np.sin(i*.1),depth_rate=.002,valid=not (70<=i<80))
    original.update(j,imu,depth,now=t)
    expected.append([*original._limited_wrench,original.armed,original.closed_loop,original.depth_hold])
    # 归一化摇杆指令换算成发送端会发出的六轴物理量，再喂给新的 control_cli。
    velocity=[-j['ly']*LINEAR_FULL,
              -j['lx']*LINEAR_FULL,
              (j['rt']-j['lt'])*HEAVE_FULL,
              (j['rb']-j['lb'])*RATE_FULL[0],
              -j['ry']*RATE_FULL[1],
              -j['rx']*RATE_FULL[2]]
    flags=[j['a'],j['b'],j['x'],j['y']]
    values=[t,*velocity,*flags,*q,*gyro,True,depth['depth'],depth['depth_rate'],depth['valid']]
    lines.append(' '.join(str(int(v)) if isinstance(v,(bool,np.bool_)) else str(v) for v in values))
output=subprocess.run([str(root/'build/uw_robot_control/control_cli')],input='\n'.join(lines)+'\n',
                      text=True,capture_output=True,check=True)
actual=np.loadtxt(output.stdout.splitlines())
expected=np.asarray(expected,dtype=float)
error=float(np.max(np.abs(actual[:,:6]-expected[:,:6])))
assert np.array_equal(actual[:,6:],expected[:,6:]), 'mode state mismatch'
report=dict(samples=len(lines),max_target_wrench_error=error,
            scope='attitude/rate/depth/open-loop target equations; allocation isolated',
            note='v0.6 起移除手柄前馈，该偏差属于设计差异；回归由单测与 six_dof 覆盖')
(root/'validation/control_parity.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
