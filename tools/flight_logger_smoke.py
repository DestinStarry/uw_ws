"""Offline in-process ROS graph: fake messages, real logger/services/files; no devices."""
import os
import secrets
os.environ["ROS_DOMAIN_ID"]=str(20+secrets.randbelow(40))
os.environ["ROS_LOCALHOST_ONLY"]="1"
os.environ["RMW_IMPLEMENTATION"]="rmw_fastrtps_cpp"
for key in ("FASTRTPS_DEFAULT_PROFILES_FILE","FASTDDS_DEFAULT_PROFILES_FILE","ROS_DISCOVERY_SERVER","ROS_SUPER_CLIENT"):
    os.environ.pop(key,None)
import csv
import json
from pathlib import Path
import tempfile
import time
import rclpy
from std_msgs.msg import String
from sensor_msgs.msg import Imu, Temperature
from std_srvs.srv import Trigger
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from uw_robot_logging.node import FlightLoggerNode
from uw_robot_interfaces.msg import ControlCommand, RobotStatus, DepthState, WrenchCommand, AllocationStatus, ThrusterCommand

node=None
logger=None
executor=None
with tempfile.TemporaryDirectory() as directory:
    root=Path(directory)
    config=root/"config.yaml"
    config.write_text("flight_logger_node:\n  ros__parameters:\n    sample_hz: 20.0\n",encoding="utf-8")
    try:
        rclpy.init(domain_id=int(os.environ["ROS_DOMAIN_ID"]),args=["--ros-args",
            "-p","output_dir:="+str(root/"sessions"),"-p","config_path:="+str(config),
            "-p","min_free_mb:=0.0","-p","max_session_mb:=0.0","-p","sample_hz:=20.0",
            "-p","parameter_nodes:=[/log_fake_robot,/flight_logger_node]"])
        executor=SingleThreadedExecutor()
        logger=FlightLoggerNode()
        executor.add_node(logger)
        node=rclpy.create_node("log_fake_robot")
        executor.add_node(node)
        node.declare_parameter("test_gain",3.25)
        node.declare_parameter("test_flags",[True,False])
        pubs={}
        for topic,typ in [("imu/data",Imu),("depth/state",DepthState),("control/cmd",ControlCommand),
                          ("robot/status",RobotStatus),("control/wrench",WrenchCommand),("allocation/status",AllocationStatus),
                          ("actuators/command",ThrusterCommand),("system/cpu_temperature",Temperature),
                          ("hardware/rx",String),("hardware/tx",String)]:
            pubs[topic]=node.create_publisher(typ,topic,10)
        status=[]
        node.create_subscription(String,"logging/status",lambda m:status.append(json.loads(m.data)),10)
        def run(seconds,invalid=False):
            end=time.monotonic()+seconds
            while time.monotonic()<end:
                assert not logger.writer.finished.is_set(),logger.writer.status()
                stamp=node.get_clock().now().to_msg()
                imu=Imu();imu.header.stamp=stamp;imu.orientation.w=1.;imu.angular_velocity.x=.1
                pubs["imu/data"].publish(imu)
                depth=DepthState();depth.header.stamp=stamp;depth.depth_m=1.25;depth.valid=not invalid;depth.data_age_s=.01
                pubs["depth/state"].publish(depth)
                cmd=ControlCommand();cmd.header.stamp=stamp;cmd.sequence=42;cmd.estop=True
                pubs["control/cmd"].publish(cmd)
                state=RobotStatus();state.header.stamp=stamp;state.state="ESTOP";state.depth_target_m=1.5;state.depth_valid=not invalid;state.imu_valid=True;state.cycle_dt_s=.02
                pubs["robot/status"].publish(state)
                wrench=WrenchCommand();wrench.header.stamp=stamp;wrench.enabled=False;wrench.session=7;wrench.epoch=2
                pubs["control/wrench"].publish(wrench)
                allocation=AllocationStatus();allocation.header.stamp=stamp;allocation.planner_valid=True;allocation.planner_ms=3.;allocation.state="STOPPED"
                pubs["allocation/status"].publish(allocation)
                actuator=ThrusterCommand();actuator.header.stamp=stamp;actuator.enabled=False
                pubs["actuators/command"].publish(actuator)
                cpu=Temperature();cpu.header.stamp=stamp;cpu.temperature=float("nan") if invalid else 42.5
                pubs["system/cpu_temperature"].publish(cpu)
                pubs["hardware/tx"].publish(String(data="OFF\n"))
                pubs["hardware/rx"].publish(String(data="VOLT:15.23 CH0:1200 CH1:0 CH2:-- CH3:--"))
                # Service the ten subscriptions and timers before the next batch.
                # One spin_once per ten publications creates a test-only backlog.
                for _ in range(24): executor.spin_once(timeout_sec=0)
                time.sleep(.01)
        def trigger(name):
            client=node.create_client(Trigger,"logging/"+name)
            assert client.wait_for_service(timeout_sec=5),name+" unavailable"
            future=client.call_async(Trigger.Request())
            deadline=time.monotonic()+8
            while not future.done() and time.monotonic()<deadline:executor.spin_once(timeout_sec=.05)
            assert future.done(),name+" timed out"
            response=future.result();node.destroy_client(client)
            assert response.success,response.message
        deadline=time.monotonic()+20
        while not status and time.monotonic()<deadline:
            assert not logger.writer.finished.is_set(),logger.writer.status()
            executor.spin_once(timeout_sec=.1)
        assert status,"logger status discovery timed out; domain="+str(node.context.get_domain_id())+" nodes="+str(node.get_node_names())+" publishers="+str(node.get_publishers_info_by_topic("logging/status"))
        run(4.)
        assert status and status[-1]["recording"],status
        node.set_parameters([Parameter("test_gain",value=4.5)])
        run(1.,invalid=True)
        trigger("stop")
        directories=sorted((root/"sessions").iterdir())
        assert len(directories)==1,directories
        first=directories[0]
        with (first/"flight.csv").open(newline="",encoding="utf-8") as stream:rows=list(csv.DictReader(stream))
        assert len(rows)>30,len(rows)
        assert any(r["cmd_sequence"]=="42" and r["depth_m"]=="1.25" and r["rpm_0"]=="1200.0" for r in rows),"message fields missing"
        assert any(r["cpu_temperature_c"]=="42.5" for r in rows)
        assert any(r["cpu_temperature_c"]=="" and r["depth_log_usable"]=="False" for r in rows[-15:]),"invalid values not preserved"
        summary=json.loads((first/"summary.json").read_text())
        assert summary["clean_close"] and summary["csv_rows"]==len(rows),summary
        parameters=json.loads((first/"parameters.json").read_text())
        assert parameters["/log_fake_robot"]["test_gain"]==4.5,parameters
        assert parameters["/log_fake_robot"]["test_flags"]==[True,False],parameters
        assert (first/"robot.yaml").read_bytes()==config.read_bytes()
        events=[json.loads(line) for line in (first/"events.jsonl").read_text().splitlines()]
        assert {"command","tx","rx","status_change","session_stop","parameters","parameter_change"}<={e["kind"] for e in events}
        assert not [p for p in node.get_publishers_info_by_topic("actuators/command") if p.node_name=="flight_logger_node"],"logger must never publish actuator commands"
        trigger("start");run(1.);trigger("stop")
        assert len(list((root/"sessions").iterdir()))==2,"restart must create new directory"
        trigger("start");run(.7)
        executor.remove_node(logger)
        logger.destroy_node()
        logger=None
        third=sorted((root/"sessions").iterdir())[-1]
        assert json.loads((third/"summary.json").read_text())["clean_close"],"node shutdown must drain and finalize"
        print("PASS: all ROS fields, CSV/events, config/runtime parameters, NaN/invalid handling, stop/start segmentation, node shutdown finalization; logger has no actuator publisher")
    except Exception:
        if logger:print("Logger status:",logger.writer.status())
        raise
    finally:
        if logger:logger.destroy_node()
        if node:node.destroy_node()
        if executor:executor.shutdown()
        if rclpy.ok():rclpy.shutdown()
