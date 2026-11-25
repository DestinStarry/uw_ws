"""Independent ROS flight logger; no command publisher and no control-loop IO."""
import json
import signal
import math
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data, qos_profile_parameter_events
from rcl_interfaces.msg import ParameterEvent
from rcl_interfaces.srv import ListParameters, GetParameters
from std_msgs.msg import String
from sensor_msgs.msg import Imu, Temperature
from std_srvs.srv import Trigger
from uw_robot_interfaces.msg import ControlCommand, RobotStatus, DepthState, WrenchCommand, AllocationStatus, ThrusterCommand
from .schema import SOURCES, Snapshot
from .adapters import adapt, stamp_seconds
from .writer import SessionWriter, utc_now

TYPES = dict(command=ControlCommand,robot=RobotStatus,imu=Imu,depth=DepthState,wrench=WrenchCommand,
             allocation=AllocationStatus,actuator=ThrusterCommand,cpu=Temperature,tx=String,rx=String)
PARAMETER_NODES = ["/control_node","/allocation_node","/hardware_node","/imu_node","/depth_node",
                   "/camera_node","/tcp_bridge_node","/system_monitor_node","/flight_logger_node"]


def parameter_value(value):
    names = {1:"bool_value",2:"integer_value",3:"double_value",4:"string_value",5:"byte_array_value",
             6:"bool_array_value",7:"integer_array_value",8:"double_array_value",9:"string_array_value"}
    field = names.get(value.type)
    if field is None: return None
    result = getattr(value,field)
    if value.type >= 5: return [None if isinstance(v,float) and not math.isfinite(v) else v for v in result]
    return None if isinstance(result,float) and not math.isfinite(result) else result


class FlightLoggerNode(Node):
    def __init__(self):
        super().__init__("flight_logger_node")
        defaults = dict(output_dir="~/uw_logs",session_label="",sample_hz=50.,flush_interval_s=1.,
                        queue_size=4096,min_free_mb=512.,max_session_mb=2048.,fsync=True,auto_start=True,
                        config_path="",run_mode="external",hardware_dry_run=True,hardware_mode_known=False,record_imu_events=False,
                        parameter_nodes=PARAMETER_NODES,parameter_capture_timeout_s=12.)
        for key,value in defaults.items(): self.declare_parameter(key,value)
        self.rate = float(self.get_parameter("sample_hz").value)
        if not math.isfinite(self.rate) or not 1 <= self.rate <= 200: raise ValueError("sample_hz must be in [1,200]")
        if int(self.get_parameter("queue_size").value) < 16: raise ValueError("queue_size must be at least 16")
        for key in ("flush_interval_s","parameter_capture_timeout_s"):
            value = float(self.get_parameter(key).value)
            if not math.isfinite(value) or value <= 0: raise ValueError(key+" must be finite and positive")
        for key in ("min_free_mb","max_session_mb"):
            value=float(self.get_parameter(key).value)
            if not math.isfinite(value) or value < 0: raise ValueError(key+" must be finite and nonnegative")
        self.model = Snapshot()
        self.writer = None
        self.last_session = {}
        self.recording = False
        self.status_signature = None
        self.source_frames = {}
        self.parameter_clients = {}
        self.parameter_requested = set()
        self.parameter_complete = set()
        self.parameter_deadline = 0.
        self.subscriptions_ = []
        for source,(topic,_) in SOURCES.items():
            qos = qos_profile_sensor_data if source == "imu" else (100 if source in ("rx","tx","command") else 10)
            self.subscriptions_.append(self.create_subscription(TYPES[source],topic,
                lambda message,key=source:self.observe(key,message),qos))
        self.parameter_events = self.create_subscription(ParameterEvent,"/parameter_events",self.parameter_changed,qos_profile_parameter_events)
        self.status_pub = self.create_publisher(String,"logging/status",1)
        self.start_service = self.create_service(Trigger,"logging/start",self.start_service_callback)
        self.stop_service = self.create_service(Trigger,"logging/stop",self.stop_service_callback)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(1./self.rate,self.sample,clock=self.steady_clock)
        self.status_timer = self.create_timer(1.,self.health,clock=self.steady_clock)
        if self.get_parameter("auto_start").value: self.start_session()

    def event(self, kind, **fields):
        if self.writer and self.recording and not self.writer.finished.is_set():
            self.writer.event(dict(kind=kind,wall_time_utc=utc_now(),elapsed_s=time.monotonic()-self.writer.started,
                                   ros_time_s=self.get_clock().now().nanoseconds*1e-9,**fields))

    def observe(self, source, message):
        try:
            data = adapt(source,message)
            received=time.monotonic()
            stamp=stamp_seconds(message)
            self.model.observe(source,data,received,stamp)
            frame = message.header.frame_id if hasattr(message,"header") else ""
            if source not in self.source_frames or self.source_frames[source] != frame:
                self.source_frames[source] = frame
                self.event("source_seen",source=source,topic=SOURCES[source][0],frame_id=frame)
            if source in ("command","tx","rx"):
                if source != "rx" or data.get("hardware_rx"): self.event(source,topic=SOURCES[source][0],stamp_s=stamp,data=data)
            elif source == "robot":
                signature=tuple(data.get(k) for k in ("armed","closed_loop","depth_hold","robot_imu_valid","robot_depth_valid","robot_state"))
                if signature != self.status_signature:
                    self.status_signature=signature
                    self.event("status_change",data=data,stamp_s=stamp)
            elif source == "imu" and self.get_parameter("record_imu_events").value:
                self.event("imu",data=data,stamp_s=stamp)
        except Exception as exc:
            self.get_logger().warning("Log message rejected: "+str(exc),throttle_duration_sec=5.)

    def start_session(self):
        if self.writer and not self.writer.finished.is_set(): return False,"A session is recording or still draining: "+str(self.writer.path)
        p=lambda key:self.get_parameter(key).value
        metadata=dict(config_path=str(Path(p("config_path")).expanduser()) if p("config_path") else "",
                      run_mode=p("run_mode"),hardware_dry_run=bool(p("hardware_dry_run")) if p("hardware_mode_known") else None,sample_hz=self.rate,topics={k:v[0] for k,v in SOURCES.items()},
                      receive_timeout_s={k:v[1] for k,v in SOURCES.items()},
                      snapshot_note="Latest available messages, not hardware synchronized; achieved_model and estimated servos are not measurements.",
                      imu_events=bool(p("record_imu_events")))
        self.writer=SessionWriter(p("output_dir"),metadata,label=p("session_label"),capacity=int(p("queue_size")),
                                  flush_interval=float(p("flush_interval_s")),min_free_mb=float(p("min_free_mb")),
                                  max_session_mb=float(p("max_session_mb")),fsync=bool(p("fsync")))
        self.recording=True
        self.model=Snapshot()
        self.status_signature=None
        self.source_frames={}
        self.parameter_requested=set()
        self.parameter_complete=set()
        self.parameter_deadline=time.monotonic()+float(p("parameter_capture_timeout_s"))
        self.event("session_start")
        self.get_logger().info("Flight log session: "+str(self.writer.path))
        return True,"Session starting: "+str(self.writer.path)

    def stop_session(self):
        if not self.writer: return False,"No session has been started"
        if self.recording: self.event("session_stop")
        self.recording=False
        success=self.writer.close(timeout=3.)
        self.last_session=self.writer.status()
        return success,json.dumps(self.last_session,ensure_ascii=False)

    def start_service_callback(self, request, response):
        try: response.success,response.message=self.start_session()
        except Exception as exc: response.success,response.message=False,str(exc)
        return response

    def stop_service_callback(self, request, response):
        response.success,response.message=self.stop_session()
        return response

    def sample(self):
        if not self.recording or not self.writer or self.writer.finished.is_set(): return
        state=self.writer.status()
        row=self.model.sample(time.monotonic()-self.writer.started,time.monotonic(),
             self.get_clock().now().nanoseconds*1e-9,queue_depth=state["queue_depth"],
             dropped_csv=state["dropped_csv"],dropped_events=state["dropped_events"])
        self.writer.sample(row)

    def parameter_changed(self, message):
        allowed={name if name.startswith("/") else "/"+name for name in self.get_parameter("parameter_nodes").value}
        if message.node not in allowed: return
        values={p.name:parameter_value(p.value) for p in [*message.new_parameters,*message.changed_parameters]}
        self.event("parameter_change",node=message.node,values=values,
                   deleted=[p.name for p in message.deleted_parameters],
                   source_stamp_s=message.stamp.sec+message.stamp.nanosec*1e-9)

    def capture_parameters(self):
        if not self.recording or time.monotonic()>self.parameter_deadline: return
        session=self.writer
        for node in self.get_parameter("parameter_nodes").value:
            if not node.startswith("/"): node="/"+node
            if node in self.parameter_requested: continue
            if node not in self.parameter_clients:
                self.parameter_clients[node]=(self.create_client(ListParameters,node+"/list_parameters"),
                                               self.create_client(GetParameters,node+"/get_parameters"))
            listing,getting=self.parameter_clients[node]
            if not listing.service_is_ready() or not getting.service_is_ready(): continue
            self.parameter_requested.add(node)
            request=ListParameters.Request()
            request.depth=0
            def listed(future,node_name=node,client=getting,session_writer=session):
                if session_writer is not self.writer or not self.recording: return
                try:
                    names=list(future.result().result.names)
                    request=GetParameters.Request();request.names=names
                    def got(result,names=names,name=node_name,writer=session_writer):
                        if writer is not self.writer or not self.recording: return
                        try:
                            values={name:parameter_value(v) for name,v in zip(names,result.result().values)}
                            self.event("parameters",node=name,values=values)
                            self.parameter_complete.add(name)
                        except Exception as exc:self.event("parameter_error",node=name,error=str(exc))
                    client.call_async(request).add_done_callback(got)
                except Exception as exc:self.event("parameter_error",node=node_name,error=str(exc))
            listing.call_async(request).add_done_callback(listed)

    def health(self):
        if self.writer:
            status=self.writer.status()
            if self.writer.finished.is_set(): self.recording=False
            if status["error"]:
                self.get_logger().error("Flight logger "+status["error"]+"; robot control is independent",throttle_duration_sec=5.)
        else: status=dict(state="idle",session_dir="",error="")
        self.capture_parameters()
        status.update(recording=self.recording,parameter_nodes_captured=sorted(self.parameter_complete))
        self.status_pub.publish(String(data=json.dumps(status,ensure_ascii=False,allow_nan=False)))

    def destroy_node(self):
        if self.writer and not self.writer.finished.is_set():
            success,message=self.stop_session()
            if not success:self.get_logger().warning("Log shutdown incomplete: "+message)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node=None
    try:
        node=FlightLoggerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException): pass
    finally:
        # launch and the terminal may both deliver SIGINT during the same stop.
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
