"""UI frame -> TCP -> ROS -> actual hardware_node -> PTY; never opens real hardware."""
import os,secrets
os.environ['ROS_DOMAIN_ID']=str(60+secrets.randbelow(40))
os.environ['ROS_LOCALHOST_ONLY']='1'
os.environ['RMW_IMPLEMENTATION']='rmw_fastrtps_cpp'
for k in ('FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE','ROS_DISCOVERY_SERVER'):
    os.environ.pop(k,None)
import math,pty,signal,socket,subprocess,tempfile,time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from uw_robot_interfaces.msg import RobotStatus
from ament_index_python.packages import get_package_share_directory

def main():
    rclpy.init();node=rclpy.create_node('camera_pan_integration')
    imu=node.create_publisher(Imu,'imu/data',qos_profile_sensor_data)
    states=[];targets=[]
    node.create_subscription(RobotStatus,'robot/status',lambda m:states.append((m.armed,m.state)),100)
    node.create_subscription(Float64,'camera/servo_target_deg',lambda m:targets.append(m.data),100)
    master,slave=pty.openpty();os.set_blocking(master,False)
    reservation=socket.socket();reservation.bind(('127.0.0.1',0));port=reservation.getsockname()[1];reservation.close()
    client=None;processes=[];wire=[];buffer=b''
    inputs={'angle':0.,'arm':0,'estop':0,'u':0.,'send':True}
    def pump(duration):
        nonlocal buffer
        end=time.monotonic()+duration
        while time.monotonic()<end:
            m=Imu();m.header.stamp=node.get_clock().now().to_msg();m.orientation.w=1.;imu.publish(m)
            if client and inputs['send']:
                text='CMD:%.6f,0,0,0,0,0,%d,%d,0,0,%.1f\n'%(inputs['u'],inputs['arm'],inputs['estop'],inputs['angle'])
                client.sendall(text.encode())
            for _ in range(5):rclpy.spin_once(node,timeout_sec=0)
            try:buffer+=os.read(master,65536)
            except BlockingIOError:pass
            while b'\n' in buffer:
                line,buffer=buffer.split(b'\n',1);wire.append(line.decode().strip())
            if client:
                try:client.recv(65536)
                except BlockingIOError:pass
            time.sleep(.02)
    def recent_stop(angle):
        return wire and all(x=='OFF:%.4f'%(angle/90) for x in wire[-5:])
    with tempfile.TemporaryDirectory() as directory:
        config=Path(get_package_share_directory('uw_robot_bringup'))/'config/robot.yaml'
        def start(package,name,args):
            log=(Path(directory)/(name+'.log')).open('w')
            proc=subprocess.Popen(['ros2','run',package,name,'--ros-args','--params-file',str(config),*args],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            processes.append((proc,log))
        try:
            start('uw_robot_control','control_node',[])
            start('uw_robot_control','allocation_node',[])
            start('uw_robot_communication','hardware_node',['-p','dry_run:=false','-p','port:='+os.ttyname(slave),'-p','publish_depth:=false'])
            start('uw_robot_communication','tcp_bridge_node',['-p','bind_address:=127.0.0.1','-p','port:='+str(port)])
            deadline=time.monotonic()+20
            while client is None and time.monotonic()<deadline:
                try:client=socket.create_connection(('127.0.0.1',port),timeout=.1);client.setblocking(False)
                except OSError:pump(.1)
                assert all(p.poll() is None for p,_ in processes),'node exited'
            assert client,'TCP server unavailable'
            while (not states or not targets) and time.monotonic()<deadline:pump(.2)
            assert states and targets,'ROS discovery failed'
            pump(.5);assert recent_stop(0),wire[-8:]
            inputs['angle']=10;pump(.5);assert recent_stop(10) and targets[-1]==10,'locked camera +10 lost'
            inputs['angle']=20;pump(.5);assert recent_stop(20),'second camera tap lost'
            inputs['angle']=10;pump(.5);assert recent_stop(10),'camera reverse step lost'
            for angle in (-90,90,0,10):
                inputs['angle']=angle;pump(.3);assert recent_stop(angle),'camera target limit propagation'
            inputs['arm']=1;pump(.4);inputs['arm']=0;inputs['u']=.3;pump(1.)
            assert states[-1][0],'arm failed'
            controls=[x for x in wire[-20:] if x.startswith('CTRL:')]
            assert controls and all(abs(float(x.split(',')[-1])-10/90)<.0001 for x in controls),'unlock changed camera target'
            assert any(any(abs(float(v))>.0001 for v in x[5:].split(',')[:4]) for x in controls),'normal thrust lost'
            inputs['estop']=1;pump(.5);assert not states[-1][0] and recent_stop(10),'estop motor stop/camera hold failed'
            inputs['send']=False;pump(.8);assert recent_stop(10),'command loss changed camera or failed motor stop'
            print('PASS: actual TCP/ROS/PTY chain; camera 0, +/-10 steps, +/-90 limits, locked movement, unlock, thrust, estop and command loss')
        except Exception:
            for _,log in processes:log.flush();print(Path(log.name).read_text())
            print('states',states[-10:],'targets',targets[-10:],'wire',wire[-15:]);raise
        finally:
            if client:client.close()
            for proc,log in processes:
                if proc.poll() is None:
                    os.killpg(proc.pid,signal.SIGINT)
                    try:proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
                log.close()
            os.close(master);os.close(slave);node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
