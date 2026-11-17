"""ROS plant node: actuators/command -> simulation -> IMU and depth."""
from pathlib import Path
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Imu, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from uw_robot_interfaces.msg import ThrusterCommand, DepthState

from .sim_engine import Simulation
from .actuator_model import ActuatorModel
from .sensor_model import SensorModel


class MujocoNode(Node):
    def __init__(self):
        super().__init__('mujoco_node')
        default = str(Path(get_package_share_directory('uw_robot_mujoco')) / 'assets/scene.xml')
        defaults = dict(model_path=default, headless=True, command_timeout_s=0.15,
                        publish_hz=50.0, linear_drag=[15.0, 40.0, 40.0],
                        angular_drag=[0.2, 0.7, 0.7], gravity_mps2=9.80665,
                        water_density_kg_m3=1000.0, water_viscosity_pa_s=0.001,
                        displaced_volume_m3=0.0, buoyancy_offset_m=[0.0, 0.0, 0.02],
                        water_current_mps=[0.0, 0.0, 0.0],
                        actuator_delay_s=.03, pwm_hz=50., motor_time_constant_s=.12,
                        motor_deadzone=0., motor_curve_exponent=1., servo_rate_rad_s=3.1415926536,
                        servo_time_constant_s=.05, servo_backlash_rad=.00523598776,
                        imu_sample_hz=100., depth_sample_hz=2.,
                        imu_latency_s=.02, depth_latency_s=.05, imu_zero_time_s=2.,
                        imu_angle_noise_deg=.15, imu_gyro_noise_rad_s=.002,
                        imu_gyro_bias_rad_s=[.005,-.003,.004], depth_noise_m=.005,
                        noise_seed=1, publish_truth=False,
                        camera_enabled=True, camera_width=640, camera_height=360,
                        camera_fps=10., camera_position_m=[.285, 0., 0.],
                        camera_baseline_m=.06, camera_fovy_deg=60.,
                        camera_jpeg_quality=80, camera_http_enabled=True,
                        camera_http_host='0.0.0.0', camera_http_port=8080)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        p = lambda key: self.get_parameter(key).value
        self.timeout = float(p('command_timeout_s'))
        hz = float(p('publish_hz'))
        if not 0 < self.timeout <= 10 or not 0 < hz <= 500:
            raise ValueError('Invalid command timeout or publish rate')
        self.actuators = ActuatorModel(
            delay=p('actuator_delay_s'), pwm_hz=p('pwm_hz'), motor_tau=p('motor_time_constant_s'),
            motor_deadzone=p('motor_deadzone'), motor_exponent=p('motor_curve_exponent'),
            servo_rate=p('servo_rate_rad_s'), servo_tau=p('servo_time_constant_s'),
            servo_backlash=p('servo_backlash_rad'))
        self.sensors = SensorModel(
            imu_hz=p('imu_sample_hz'), depth_hz=p('depth_sample_hz'),
            imu_delay=p('imu_latency_s'), depth_delay=p('depth_latency_s'),
            zero_time=p('imu_zero_time_s'), angle_noise_deg=p('imu_angle_noise_deg'),
            gyro_noise=p('imu_gyro_noise_rad_s'), gyro_bias=p('imu_gyro_bias_rad_s'),
            depth_noise=p('depth_noise_m'), seed=p('noise_seed'))
        self.publish_truth = p('publish_truth')
        self.sim = Simulation(
            p('model_path'), p('linear_drag'), p('angular_drag'),
            gravity=p('gravity_mps2'), water_density=p('water_density_kg_m3'),
            water_viscosity=p('water_viscosity_pa_s'), displaced_volume=p('displaced_volume_m3'),
            buoyancy_offset=p('buoyancy_offset_m'), water_current=p('water_current_mps'),
            actuator_model=self.actuators)
        self.camera = self.preview = None
        self.camera_error_reported = False
        self.camera_period = 1. / float(p('camera_fps')) if float(p('camera_fps')) > 0 else 0.
        if not 0 < self.camera_period <= 10:
            raise ValueError('camera_fps must be in [0.1, 100]')
        self.last_camera = -1e9
        self.configure_camera(p)
        self.publish_sensor_transforms()
        self.viewer = None
        if not p('headless'):
            import mujoco.viewer
            self.viewer = mujoco.viewer.launch_passive(self.sim.model, self.sim.data)
        # 相机线程必须在 viewer 的 GLFW 初始化之后启动，避免并发 glfwInit。
        self.start_camera()
        self.last_command = -1e9
        self.last_wall = time.monotonic()
        self.accumulator = 0.0
        self.last_publish = self.last_wall
        self.last_view = self.last_wall
        self.period = 1.0 / hz
        self.imu_pub = self.create_publisher(Imu, 'imu/data', qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(DepthState, 'depth/state', 1)
        self.pose_pub = self.create_publisher(PoseStamped, 'simulation/pose', 1)
        self.sub = self.create_subscription(ThrusterCommand, 'actuators/command', self.receive, 1)
        self.timer = self.create_timer(0.002, self.tick)
        self.get_logger().info('MuJoCo plant started; no serial/USB IMU opened; gripper not modeled')

    def receive(self, msg):
        """入口检查 ROS 时间戳，持续超时检查使用单调时钟。"""
        age = (self.get_clock().now().nanoseconds -
               (msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec)) / 1e9
        if age < -0.05 or age > self.timeout:
            return
        if self.sim.command(msg.thrust_newtons, [msg.theta12_rad, msg.theta34_rad], msg.enabled):
            self.last_command = time.monotonic()
        else:
            self.last_command = -1e9

    def tick(self):
        """墙钟驱动物理步进；限制补帧数量，避免阻塞后集中执行旧命令。"""
        now = time.monotonic()
        elapsed = min(max(now - self.last_wall, 0.0), 0.05)
        self.last_wall = now
        self.accumulator += elapsed
        dt = float(self.sim.model.opt.timestep)
        steps = min(int(self.accumulator / dt), 10)
        if now - self.last_command > self.timeout:
            self.sim.stop()
        if steps:
            self.accumulator -= steps * dt
            # Bound catch-up: never run many seconds of queued physics steps.
            self.accumulator = min(self.accumulator, 0.05)
            self.sim.step(steps)
        if now - self.last_publish >= self.period:
            self.last_publish = now
            self.publish_state()
        observations = self.sensors.poll(now, self.get_clock().now().nanoseconds, self.sim.state())
        if 'imu' in observations:
            self.publish_imu(observations['imu'])
        if 'depth' in observations:
            self.publish_depth(observations['depth'], now)
        if self.camera:
            if self.camera.error and not self.camera_error_reported:
                self.get_logger().error('Stereo camera failed: ' + self.camera.error +
                                        '; physics and IMU continue. Check OpenGL/EGL.')
                self.camera_error_reported = True
            if now-self.last_camera >= self.camera_period:
                self.last_camera = now
                self.camera.submit(self.sim.data, self.get_clock().now().nanoseconds)
        self.update_viewer(now)

    def update_viewer(self, now):
        """显示按 30 Hz 刷新；关闭窗口时退出节点，由 launch 联动停止其余节点。"""
        if self.viewer and now - self.last_view >= 1 / 30:
            self.last_view = now
            if self.viewer.is_running():
                self.viewer.sync()
            else:
                self.sim.stop()
                raise KeyboardInterrupt

    @staticmethod
    def sample_header(message, stamp_ns, frame_id='base_link'):
        message.header.stamp.sec = stamp_ns // 1000000000
        message.header.stamp.nanosec = stamp_ns % 1000000000
        message.header.frame_id = frame_id

    def publish_imu(self, sample):
        imu = Imu()
        self.sample_header(imu, sample['stamp_ns'], 'imu_frame')
        q = sample['quaternion']
        imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z = map(float, q)
        for vector, values in ((imu.angular_velocity, sample['gyro']),
                               (imu.linear_acceleration, sample['acceleration'])):
            vector.x, vector.y, vector.z = map(float, values)
        self.imu_pub.publish(imu)

    def publish_depth(self, sample, now):
        depth = DepthState()
        # Match hardware: publication stamp is fresh, data age tracks original pressure sample.
        depth.header.stamp = self.get_clock().now().to_msg()
        depth.header.frame_id = 'base_link'
        depth.depth_m = float(sample['depth'])
        depth.depth_rate_mps = float(sample['depth_rate'])
        depth.data_age_s = max(0., now-sample['received_at'])
        depth.valid = depth.data_age_s <= 1.5
        self.depth_pub.publish(depth)

    def publish_state(self):
        # Truth is a diagnostic opt-in. No controller/allocation subscription uses this topic.
        if not self.publish_truth:
            return
        state = self.sim.state()
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'world'
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, state['position'])
        q = state['quaternion']
        pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z = map(float,q)
        self.pose_pub.publish(pose)

    def configure_camera(self, p):
        """复用实物图像话题和 HTTP 预览协议；不打开 USB 摄像头。"""
        if not p('camera_enabled'):
            return
        from sensor_msgs.msg import Image
        from .stereo_camera import StereoCamera, configure_stereo, pinhole_parameters
        configure_stereo(self.sim.model, p('camera_position_m'), p('camera_baseline_m'),
                         p('camera_fovy_deg'))
        width, height = int(p('camera_width')), int(p('camera_height'))
        # offscreen 缓冲区必须容纳单目分辨率，在线程启动前完成模型修改。
        # WSL/Mesa 预览降低阴影和多重采样缓冲开销。
        self.sim.model.vis.quality.shadowsize = 1024
        self.sim.model.vis.quality.offsamples = 0
        self.sim.model.vis.global_.offwidth = max(width, self.sim.model.vis.global_.offwidth)
        self.sim.model.vis.global_.offheight = max(height, self.sim.model.vis.global_.offheight)
        k, pl, pr = pinhole_parameters(width, height, p('camera_fovy_deg'), p('camera_baseline_m'))
        self.camera_outputs = []
        for eye, projection in (('left', pl), ('right', pr)):
            info = CameraInfo()
            info.header.frame_id = 'camera_' + eye + '_optical_frame'
            info.width, info.height = width, height
            info.distortion_model = 'plumb_bob'
            info.d = [0.] * 5
            info.k, info.p = k, projection
            info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
            self.camera_outputs.append((
                self.create_publisher(Image, 'camera/' + eye + '/image_raw', qos_profile_sensor_data),
                self.create_publisher(CameraInfo, 'camera/' + eye + '/camera_info', qos_profile_sensor_data),
                info))
        self.camera = StereoCamera(self.sim.model, width=width, height=height,
            quality=p('camera_jpeg_quality'), on_frame=self.publish_camera)
        # 线程启动推迟到 viewer 完成 GLFW 初始化之后，见 start_camera()。
        self.camera_fps = p('camera_fps')
        self.camera_baseline = p('camera_baseline_m')
        self.camera_http = (p('camera_http_enabled'), p('camera_http_host'), p('camera_http_port'))

    def start_camera(self):
        """相机线程必须晚于交互窗口：两者共用 GLFW，并发 glfwInit 会触发 X11 断言。"""
        if not self.camera:
            return
        from uw_robot_sensors.camera.stream import PreviewServer
        enabled, host, port = self.camera_http
        try:
            if enabled:
                self.preview = PreviewServer(self.camera, host, port)
                self.preview.start()
            self.camera.start()
        except Exception:
            if self.preview:
                self.preview.stop()
            self.camera.close()
            raise
        self.get_logger().info('Stereo: {}x{} per eye, {:.1f}Hz, baseline {:.0f}mm; HTTP port {}'.format(
            self.camera.width, self.camera.height, self.camera_fps,
            1000*self.camera_baseline, port if self.preview else 'disabled'))

    def publish_camera(self, frame):
        """渲染线程发布；图像/内参使用同一个快照时间，不把旧帧重新标为当前时间。"""
        from uw_robot_sensors.camera.camera_node import make_image_message
        for image, (publisher, info_pub, info) in zip((frame.left, frame.right), self.camera_outputs):
            info.header.stamp.sec = frame.ros_stamp_ns // 1000000000
            info.header.stamp.nanosec = frame.ros_stamp_ns % 1000000000
            if publisher.get_subscription_count():
                publisher.publish(make_image_message(image, info.header.frame_id, frame.ros_stamp_ns))
            if info_pub.get_subscription_count():
                info_pub.publish(info)

    def publish_sensor_transforms(self):
        """机体 FLU -> IMU/相机光学系；安装偏移与仿真模型保持一致。"""
        import mujoco
        self.sensor_tf = StaticTransformBroadcaster(self)
        transforms = []
        imu_id = mujoco.mj_name2id(self.sim.model, mujoco.mjtObj.mjOBJ_SITE, 'imu_site')
        mounts = [('imu_frame', self.sim.model.site_pos[imu_id], (0., 0., 0., 1.))]
        if self.camera:
            for eye in ('left', 'right'):
                camera_id = mujoco.mj_name2id(self.sim.model, mujoco.mjtObj.mjOBJ_CAMERA, 'stereo_' + eye)
                # 光学 X=-机体Y，光学Y=-机体Z，光学Z=机体X。
                mounts.append(('camera_' + eye + '_optical_frame',
                               self.sim.model.cam_pos[camera_id], (-.5, .5, -.5, .5)))
        for child, position, quaternion in mounts:
            tf = TransformStamped()
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id, tf.child_frame_id = 'base_link', child
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, position)
            tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = quaternion
            transforms.append(tf)
        self.sensor_tf.sendTransform(transforms)

    def close(self):
        self.sim.stop()
        if self.camera:
            self.camera.close()
        if self.preview:
            self.preview.stop()
        if self.viewer:
            self.viewer.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MujocoNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if node:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
