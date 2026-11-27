"""Direct CTRL/OFF integration using real ROS nodes and a PTY; no physical devices."""
import os,secrets
os.environ["ROS_DOMAIN_ID"]=str(20+secrets.randbelow(40))
os.environ["ROS_LOCALHOST_ONLY"]="1"
os.environ["RMW_IMPLEMENTATION"]="rmw_fastrtps_cpp"
for key in ("FASTRTPS_DEFAULT_PROFILES_FILE","FASTDDS_DEFAULT_PROFILES_FILE","ROS_DISCOVERY_SERVER","ROS_SUPER_CLIENT","SKIP_DEFAULT_XML"):
    os.environ.pop(key,None)
import pty,signal,subprocess,tempfile,time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from uw_robot_interfaces.msg import ControlCommand,RobotStatus
from ament_index_python.packages import get_package_share_directory

def main():
    rclpy.init();node=rclpy.create_node("simple_control_test")
    commands=node.create_publisher(ControlCommand,"control/cmd",10)
    imu=node.create_publisher(Imu,"imu/data",qos_profile_sensor_data)
    states=[]
    node.create_subscription(RobotStatus,"robot/status",lambda m:states.append((m.armed,m.state)),100)
    master,slave=pty.openpty();os.set_blocking(master,False)
    processes=[];wire=[];buffer=b"";sequence=0
    control={"arm":False,"estop":False,"u":0.,"send":True,"feedback":False}
    def pump(seconds):
        nonlocal sequence,buffer
        end=time.monotonic()+seconds;reported=0
        while time.monotonic()<end:
            stamp=node.get_clock().now().to_msg()
            m=Imu();m.header.stamp=stamp;m.orientation.w=1.;imu.publish(m)
            if control["send"]:
                c=ControlCommand();c.header.stamp=stamp;c.sequence=sequence;sequence+=1
                c.arm=control["arm"];c.estop=control["estop"];c.velocity[0]=control["u"];commands.publish(c)
            for _ in range(4):rclpy.spin_once(node,timeout_sec=0)
            try:buffer+=os.read(master,65536)
            except BlockingIOError:pass
            while b"\n" in buffer:
                line,buffer=buffer.split(b"\n",1);wire.append(line.decode().strip())
            if control["feedback"] and time.monotonic()-reported>.1:
                os.write(master,b"VOLT:24.00 CH0:-- CH1:-- CH2:-- CH3:--\r\n")
                reported=time.monotonic()
            time.sleep(.01)
    def nonzero(lines):
        return any(x.startswith("CTRL:") and any(abs(float(v))>.0001 for v in x[5:].split(",")[:4]) for x in lines)
    with tempfile.TemporaryDirectory() as directory:
        config=Path(get_package_share_directory("uw_robot_bringup"))/"config/robot.yaml"
        def start(package,name,params):
            log=(Path(directory)/(name+".log")).open("w")
            proc=subprocess.Popen(["ros2","run",package,name,"--ros-args","--params-file",str(config),*params],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            processes.append((proc,log))
        try:
            start("uw_robot_control","control_node",[])
            start("uw_robot_control","allocation_node",[])
            start("uw_robot_communication","hardware_node",["-p","dry_run:=false","-p","port:="+os.ttyname(slave),"-p","publish_depth:=false"])
            deadline=time.monotonic()+20
            while not states and time.monotonic()<deadline:
                assert all(proc.poll() is None for proc,_ in processes),"node exited during discovery"
                pump(.3)
            assert states,"no state discovered"
            pump(.3)
            assert wire and set(wire)=={"OFF:0.0000"},wire[-10:]
            # No MCU feedback/handshake exists. A normal explicit arm must suffice.
            control.update(arm=True);pump(.3);control.update(arm=False,u=.4)
            offset=len(wire);pump(1.)
            assert states[-1][0] and nonzero(wire[offset:]),"direct control blocked without MCU heartbeat"
            assert not any(x.startswith("SYNC:") for x in wire),"obsolete handshake sent"
            control.update(u=-.4);offset=len(wire);pump(.8)
            assert nonzero(wire[offset:]) and states[-1][0],"second joystick direction failed"
            # Missing RPM remains informational; valid telemetry does not gate control.
            control.update(feedback=True,u=.2);offset=len(wire);pump(.7)
            assert states[-1][0] and nonzero(wire[offset:]),"missing RPM gated motion"
            control.update(estop=True);pump(.5)
            assert not states[-1][0] and all(x=="OFF:0.0000" for x in wire[-8:]),"estop failed"
            control.update(estop=False,arm=False,u=0.);pump(.3)
            control.update(arm=True);pump(.3);control.update(arm=False,u=.2);pump(.6)
            assert states[-1][0],"explicit rearm failed"
            control.update(send=False);pump(.9)
            assert not states[-1][0] and all(x=="OFF:0.0000" for x in wire[-8:]),"command timeout failed"
            print("PASS: direct CTRL/OFF without SYNC/MCU heartbeat/RPM gate; forward/reverse, estop, rearm and command-loss stop")
        except Exception:
            for _,log in processes:log.flush();print(Path(log.name).read_text())
            print(states[-10:],wire[-15:]);raise
        finally:
            for proc,log in processes:
                if proc.poll() is None:
                    os.killpg(proc.pid,signal.SIGINT)
                    try:proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
                log.close()
            os.close(master);os.close(slave);node.destroy_node();rclpy.shutdown()
if __name__=="__main__":main()
