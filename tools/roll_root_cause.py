#!/usr/bin/env python3
"""滚转根因追踪：只打升沉，50 Hz 记录推力/舵角/目标与实现力矩，定位 Mx 来源。"""
import json, math, os, secrets, signal, subprocess, time
from pathlib import Path
os.environ['ROS_DOMAIN_ID'] = str(20 + secrets.randbelow(40))
os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(Path(__file__).with_name('dds_test.xml'))
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from uw_robot_interfaces.msg import AllocationStatus, ControlCommand, RobotStatus
ROOT = Path(__file__).resolve().parents[1]

def euler(q):
    w, x, y, z = q
    return (math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
            math.asin(max(-1.0, min(1.0, 2*(w*y-z*x)))),
            math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))

def main():
    log = (ROOT/'validation/rc_launch.log').open('w')
    proc = subprocess.Popen(['ros2','launch','uw_robot_bringup','simulation.launch.py',
        'headless:=true','camera:=false','publish_truth:=true','tcp_port:=18875'],
        stdout=log, stderr=subprocess.STDOUT)
    rclpy.init()
    node = rclpy.create_node('roll_root_cause')
    pub = node.create_publisher(ControlCommand, 'control/cmd', 10)
    st = {'s': None, 'a': None, 'q': (1,0,0,0), 'g': (0,0,0)}
    node.create_subscription(RobotStatus, 'robot/status', lambda m: st.update(s=m), 10)
    node.create_subscription(AllocationStatus, 'allocation/status', lambda m: st.update(a=m), 10)
    node.create_subscription(Imu, 'imu/data',
        lambda m: st.update(g=(m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z)),
        qos_profile_sensor_data)
    node.create_subscription(PoseStamped, 'simulation/pose',
        lambda m: st.update(q=(m.pose.orientation.w, m.pose.orientation.x,
                               m.pose.orientation.y, m.pose.orientation.z)), 10)
    seq=[0]; rows=[]
    def emit(v, arm=False):
        m = ControlCommand()
        m.header.stamp = node.get_clock().now().to_msg()
        m.header.frame_id='base_link'; m.sequence=seq[0]; seq[0]+=1
        m.velocity=list(v); m.arm=arm; pub.publish(m)
    def hold(sec, v, arm=False, rec=False, t0=0.0):
        end=time.monotonic()+sec
        while time.monotonic()<end:
            emit(v, arm); rclpy.spin_once(node, timeout_sec=.002)
            if rec and st['s'] and st['a']:
                a=st['a']; e=euler(st['q'])
                rows.append({'t':round(time.monotonic()-t0,3),
                    'roll_deg':round(math.degrees(e[0]),3),
                    'roll_dps':round(math.degrees(st['g'][0]),2),
                    'cmd_mx':round(a.target_wrench[3],3),
                    'ach_mx':round(a.achieved_wrench[3],3),
                    'cmd_my':round(a.target_wrench[4],3),
                    'ach_my':round(a.achieved_wrench[4],3),
                    'cmd_fz':round(a.target_wrench[2],2),
                    'ach_fz':round(a.achieved_wrench[2],2),
                    'thrusts':[round(x,2) for x in a.thruster_forces],
                    'servo_t':[round(math.degrees(x),1) for x in a.servo_target_rad],
                    'servo_e':[round(math.degrees(x),1) for x in a.servo_estimated_rad],
                    'gate':[round(x,2) for x in a.gate]})
            time.sleep(.02)
    try:
        end=time.monotonic()+40
        while time.monotonic()<end:
            if proc.poll() is not None: raise RuntimeError('仿真启动失败')
            rclpy.spin_once(node, timeout_sec=.05)
            if st['s'] and pub.get_subscription_count()>0 and st['s'].imu_valid: break
        zero=(0.0,)*6
        for _ in range(6):
            emit(zero); rclpy.spin_once(node, timeout_sec=.01); time.sleep(.03)
        for _ in range(5):
            hold(.12, zero, arm=True); hold(.2, zero)
            if st['s'] and st['s'].armed: break
        hold(1.0, zero)
        cmd=[0.0]*6; cmd[2]=1.0
        t0=time.monotonic()
        hold(2.5, tuple(cmd), rec=True, t0=t0)
    finally:
        try:
            for _ in range(3):
                m=ControlCommand(); m.header.stamp=node.get_clock().now().to_msg()
                m.velocity=[0.0]*6; m.estop=True; pub.publish(m)
                rclpy.spin_once(node, timeout_sec=.02)
        except Exception: pass
        proc.send_signal(signal.SIGINT)
        try: proc.wait(timeout=10)
        except subprocess.TimeoutExpired: proc.kill(); proc.wait()
        log.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    Path(ROOT/'validation/roll_root_cause.json').write_text(json.dumps(rows, indent=1), encoding='utf-8')
    print('%6s %8s %9s %8s %8s %8s %8s %-22s %-16s %s' % (
        't','roll°','rate°/s','cmdMx','achMx','cmdFz','achFz','推力[1,3,2,4]','舵角目标/估计','gate'))
    for target_t in [round(0.05*i,2) for i in range(0,31,2)]:
        r=min(rows, key=lambda x: abs(x['t']-target_t)) if rows else None
        if r is None: break
        print('%6.2f %8.2f %9.1f %8.2f %8.2f %8.2f %8.2f %-22s %-16s %s' % (
            r['t'], r['roll_deg'], r['roll_dps'], r['cmd_mx'], r['ach_mx'],
            r['cmd_fz'], r['ach_fz'], str(r['thrusts']),
            '%s / %s' % (r['servo_t'], r['servo_e']), r['gate']))
    print('\n报告: validation/roll_root_cause.json')

if __name__ == '__main__':
    raise SystemExit(main())
