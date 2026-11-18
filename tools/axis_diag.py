#!/usr/bin/env python3
"""逐轴诊断：单轴指令下记录目标/实现 wrench、推力、舵角滞后，找出哪个自由度产生假力矩。"""
import argparse, json, math, os, secrets, signal, subprocess, time
from pathlib import Path
os.environ['ROS_DOMAIN_ID'] = str(20 + secrets.randbelow(40))
os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(Path(__file__).with_name('dds_test.xml'))
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from uw_robot_interfaces.msg import AllocationStatus, ControlCommand, RobotStatus
ROOT = Path(__file__).resolve().parents[1]
IDX = {'u':0,'v':1,'w':2,'p':3,'q':4,'r':5}
FULL = {'u':1.0,'v':1.0,'w':1.0,'p':math.radians(25),'q':math.radians(25),'r':math.radians(25)}
NAMES = ['Fx','Fy','Fz','Mx','My','Mz']

def euler(q):
    w,x,y,z = q
    return (math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),
            math.asin(max(-1.0,min(1.0,2*(w*y-z*x)))),
            math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--axis', required=True, choices=list(IDX))
    ap.add_argument('--hold', type=float, default=2.5)
    args = ap.parse_args()
    log = (ROOT/('validation/ax_%s.log' % args.axis)).open('w')
    proc = subprocess.Popen(['ros2','launch','uw_robot_bringup','simulation.launch.py',
        'headless:=true','camera:=false','publish_truth:=true','tcp_port:=18874'],
        stdout=log, stderr=subprocess.STDOUT)
    rclpy.init()
    node = rclpy.create_node('axis_diag')
    pub = node.create_publisher(ControlCommand, 'control/cmd', 10)
    st = {'s':None,'a':None,'q':(1,0,0,0),'g':(0,0,0)}
    node.create_subscription(RobotStatus,'robot/status',lambda m: st.update(s=m),10)
    node.create_subscription(AllocationStatus,'allocation/status',lambda m: st.update(a=m),10)
    node.create_subscription(Imu,'imu/data',
        lambda m: st.update(g=(m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z)),
        qos_profile_sensor_data)
    node.create_subscription(PoseStamped,'simulation/pose',
        lambda m: st.update(q=(m.pose.orientation.w,m.pose.orientation.x,
                               m.pose.orientation.y,m.pose.orientation.z)),10)
    seq=[0]; rows=[]
    def emit(v,arm=False):
        m=ControlCommand(); m.header.stamp=node.get_clock().now().to_msg()
        m.header.frame_id='base_link'; m.sequence=seq[0]; seq[0]+=1
        m.velocity=list(v); m.arm=arm; pub.publish(m)
    def hold(sec,v,arm=False,rec=False,t0=0.0):
        end=time.monotonic()+sec
        while time.monotonic()<end:
            emit(v,arm); rclpy.spin_once(node,timeout_sec=.002)
            if rec and st['s'] and st['a']:
                a=st['a']
                rows.append({
                  't':round(time.monotonic()-t0,3),
                  'target':[round(x,3) for x in a.target_wrench],
                  'achieved':[round(x,3) for x in a.achieved_wrench],
                  'thrusts':[round(x,2) for x in a.thruster_forces],
                  'servo_t':[round(math.degrees(x),1) for x in a.servo_target_rad],
                  'servo_e':[round(math.degrees(x),1) for x in a.servo_estimated_rad],
                  'gate':[round(x,2) for x in a.gate],
                  'gyro_dps':[round(math.degrees(g),1) for g in st['g']],
                })
            time.sleep(.02)
    try:
        end=time.monotonic()+40
        while time.monotonic()<end:
            if proc.poll() is not None: raise RuntimeError('启动失败')
            rclpy.spin_once(node,timeout_sec=.05)
            if st['s'] and pub.get_subscription_count()>0 and st['s'].imu_valid: break
        zero=(0.0,)*6
        for _ in range(6):
            emit(zero); rclpy.spin_once(node,timeout_sec=.01); time.sleep(.03)
        for _ in range(5):
            hold(.12,zero,arm=True); hold(.2,zero)
            if st['s'] and st['s'].armed: break
        hold(1.0,zero)
        cmd=[0.0]*6; cmd[IDX[args.axis]]=FULL[args.axis]
        t0=time.monotonic(); hold(args.hold,tuple(cmd),rec=True,t0=t0)
    finally:
        try:
            for _ in range(3):
                m=ControlCommand(); m.header.stamp=node.get_clock().now().to_msg()
                m.velocity=[0.0]*6; m.estop=True; pub.publish(m)
                rclpy.spin_once(node,timeout_sec=.02)
        except Exception: pass
        proc.send_signal(signal.SIGINT)
        try: proc.wait(timeout=10)
        except subprocess.TimeoutExpired: proc.kill(); proc.wait()
        log.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    Path(ROOT/('validation/ax_%s.json' % args.axis)).write_text(json.dumps(rows, indent=1), encoding='utf-8')
    if not rows:
        print('%s: 无数据' % args.axis); return
    spur = [max(abs(r['achieved'][i] - r['target'][i]) for r in rows) for i in range(6)]
    lag = max(max(abs(a-b) for a,b in zip(r['servo_t'], r['servo_e'])) for r in rows)
    asym2 = max(abs(r['thrusts'][2]-r['thrusts'][3]) for r in rows)
    asym1 = max(abs(r['thrusts'][0]-r['thrusts'][1]) for r in rows)
    gyr = [max(abs(r['gyro_dps'][i]) for r in rows) for i in range(3)]
    print('%-2s 假力矩 F[%5.2f %5.2f %5.2f] M[%5.2f %5.2f %5.2f] | 舵角滞后 %5.1f° | 推力差 2/4=%5.2f 1/3=%5.2f | 最大角速度 p%5.1f q%5.1f r%5.1f'
          % (args.axis, spur[0], spur[1], spur[2], spur[3], spur[4], spur[5],
             lag, asym2, asym1, gyr[0], gyr[1], gyr[2]))

if __name__ == '__main__':
    raise SystemExit(main())
