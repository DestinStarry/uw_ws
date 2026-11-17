"""Headless ROS control chain test, never opens robot hardware."""
import os
from pathlib import Path
import secrets
root = Path('/home/zy/uw_ws')
os.environ['ROS_DOMAIN_ID'] = str(20 + secrets.randbelow(40))
os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(root/'tools/dds_test.xml')
os.environ['ROS_LOCALHOST_ONLY'] = '0'
import json
import signal
import socket
import subprocess
import time
import rclpy
from geometry_msgs.msg import PoseStamped
from uw_robot_interfaces.msg import ThrusterCommand, DepthState, RobotStatus

rclpy.init()
node = rclpy.create_node('mujoco_integration_test')
commands, poses, depths, statuses = [], [], [], []
subscriptions = [
    node.create_subscription(ThrusterCommand,'actuators/command',commands.append,10),
    node.create_subscription(PoseStamped,'simulation/pose',poses.append,10),
    node.create_subscription(DepthState,'depth/state',depths.append,10),
    node.create_subscription(RobotStatus,'robot/status',statuses.append,10)]
log_path = root/'validation/mujoco_launch.log'
client = None
with log_path.open('w') as log:
    process = subprocess.Popen(['ros2','launch','uw_robot_bringup','simulation.launch.py',
                                'headless:=true','publish_truth:=true','tcp_port:=18889'],
                               stdout=log,stderr=subprocess.STDOUT)
    try:
        deadline=time.monotonic()+20
        while time.monotonic()<deadline:
            assert process.poll() is None,log_path.read_text()
            if client is None:
                try:client=socket.create_connection(('127.0.0.1',18889),timeout=.2)
                except OSError:pass
            rclpy.spin_once(node,timeout_sec=.05)
            # 控制器闭环解锁要求 IMU 有效；仿真 IMU 有 zero_time_s 零点校准，必须等它出现。
            if client and len(poses)>2 and commands and statuses and depths and statuses[-1].imu_valid:break
        assert client and poses and depths and statuses,'ROS discovery or simulator startup failed'
        assert not commands[-1].enabled,'Must start disarmed'
        def drive(seconds,arm=0,u=0):
            end=time.monotonic()+seconds
            while time.monotonic()<end:
                client.sendall(('CMD:%s,0,0,0,0,0,%d,0,0,0\n'%(u,arm)).encode())
                rclpy.spin_once(node,timeout_sec=.02)
                time.sleep(.01)
        drive(.3)
        drive(.2,arm=1)
        drive(.3)
        assert any(s.armed for s in statuses),'TCP arm failed'
        start=len(commands)
        x0=poses[-1].pose.position.x
        drive(3.0,u=.3)
        assert any(m.enabled and max(m.thrust_newtons)>0.01 for m in commands[start:])
        assert poses[-1].pose.position.x>x0,'No forward motion'
        end=time.monotonic()+1
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
        assert not commands[-1].enabled,'CMD timeout did not stop output'
        assert all(d.valid for d in depths[-5:])
        result=dict(default_disarmed=True,tcp_arm=True,actuator_motion=True,
                    forward_motion=True,imu_depth_feedback=True,cmd_timeout_off=True)
    finally:
        if client:client.close()
        if process.poll() is None:process.send_signal(signal.SIGINT)
        try:process.wait(timeout=10)
        except subprocess.TimeoutExpired:process.kill();process.wait()
        node.destroy_node();rclpy.shutdown()
assert process.returncode==0,log_path.read_text()
assert 'hardware_node' not in log_path.read_text(),'Unexpected hardware node'
result['clean_shutdown']=True
(root/'validation/mujoco_integration.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
