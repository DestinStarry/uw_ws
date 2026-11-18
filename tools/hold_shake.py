#!/usr/bin/env python3
"""滚转到位后松手，测保持期间的抖动幅度（用于对比 D 项效果）。"""
import json, math, os, secrets, signal, subprocess, time
from pathlib import Path
os.environ['ROS_DOMAIN_ID'] = str(20 + secrets.randbelow(40))
os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(Path(__file__).with_name('dds_test.xml'))
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from uw_robot_interfaces.msg import ControlCommand, RobotStatus
ROOT = Path(__file__).resolve().parents[1]

def euler(q):
    w,x,y,z = q
    return (math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),
            math.asin(max(-1.0,min(1.0,2*(w*y-z*x)))),
            math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))

def main():
    log = (ROOT/'validation/shake_launch.log').open('w')
    proc = subprocess.Popen(['ros2','launch','uw_robot_bringup','simulation.launch.py',
        'headless:=true','camera:=false','publish_truth:=true','tcp_port:=18872'],
        stdout=log, stderr=subprocess.STDOUT)
    rclpy.init()
    node = rclpy.create_node('hold_shake')
    pub = node.create_publisher(ControlCommand, 'control/cmd', 10)
    st = {'s':None,'q':(1,0,0,0),'g':(0,0,0)}
    node.create_subscription(RobotStatus,'robot/status',lambda m: st.update(s=m),10)
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
            if rec:
                e=euler(st['q'])
                rows.append({'t':round(time.monotonic()-t0,3),
                             'roll_deg':round(math.degrees(e[0]),3),
                             'roll_dps':round(math.degrees(st['g'][0]),2),
                             'pitch_deg':round(math.degrees(e[1]),3)})
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
        rate=math.radians(25.0)
        hold(3.0,(0.0,0.0,0.0,rate,0.0,0.0))
        t0=time.monotonic()
        hold(6.0,zero,rec=True,t0=t0)   # 松手保持 6 s
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
    Path(ROOT/'validation/hold_shake.json').write_text(json.dumps(rows, indent=1), encoding='utf-8')
    late=[r for r in rows if r['t']>=4.0]
    if not late:
        print('无数据'); return
    rates=[abs(r['roll_dps']) for r in late]
    angs=[r['roll_deg'] for r in late]
    rms=math.sqrt(sum(x*x for x in rates)/len(rates))
    print('保持段(4~6 s)：滚转角 %.1f~%.1f°  滚转率 |max|=%.1f  rms=%.1f °/s  角度摆幅=%.2f°'
          % (min(angs), max(angs), max(rates), rms, max(angs)-min(angs)))

if __name__ == '__main__':
    raise SystemExit(main())
