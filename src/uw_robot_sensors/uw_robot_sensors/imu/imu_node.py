"""IMU ROS 层：把 SDK 快照发布成 ``sensor_msgs/Imu``。

* 只发布**真实新鲜**样本：SDK 帧时间与主机接收时间共同判断，重复/过期样本不重发；
* 时间戳回填到采样时刻，而不是定时器触发时刻，便于下游做时间对齐；
* 单位与坐标：四元数 wxyz → ROS xyzw，角速度 rad/s，加速度 g → m/s²。
"""
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from . import driver


def make_imu_message(state, frame_id, stamp_ns):
    """把 SDK 快照转换为 ROS Imu；wxyz、rad/s、g -> 对应字段及 m/s²。"""
    msg = Imu()
    msg.header.stamp.sec = stamp_ns // 1_000_000_000
    msg.header.stamp.nanosec = stamp_ns % 1_000_000_000
    msg.header.frame_id = frame_id
    q = state['quaternion']  # SDK wxyz -> ROS fields xyzw
    msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z = q
    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = map(float, state['gyro'])
    # Original SDK acceleration unit is g; ROS Imu requires m/s^2.
    acc = tuple(float(x) * 9.80665 for x in state['acceleration'])
    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = acc
    # All-zero covariance means unknown, not an asserted noise model.
    return msg


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        driver.YBUART_PORT = self.declare_parameter('port', '/dev/ttyCH341USB0').value
        driver.IMU_OUTPUT_RATE = self.declare_parameter('rate_hz', 100).value
        driver.IMU_ALGORITHM = self.declare_parameter('algorithm', 6).value
        driver.IMU_ZERO_CALIBRATION_TIME = self.declare_parameter('zero_time_s', 2.0).value
        driver.IMU_STALE_TIMEOUT = self.declare_parameter('stale_timeout_s', 0.1).value
        self.frame = self.declare_parameter('frame_id', 'base_link').value
        if driver.IMU_OUTPUT_RATE <= 0 or driver.IMU_STALE_TIMEOUT <= 0:
            raise ValueError('IMU rate and timeout must be positive')
        driver.IMU_READ_INTERVAL = 1.0 / driver.IMU_OUTPUT_RATE
        if not driver.YB_IMU_AVAILABLE:
            raise RuntimeError('YbImuLib missing: install the same SDK used on the OrangePi')
        self.publisher = self.create_publisher(Imu, 'imu/data', qos_profile_sensor_data)
        self.stop_event = threading.Event()
        self.reader = threading.Thread(target=driver.imu_reader, args=(self.stop_event,), daemon=True)
        self.reader.start()
        self.last_sample = 0.0
        self.timer = self.create_timer(driver.IMU_READ_INTERVAL, self.publish_sample)

    def publish_sample(self):
        state = driver.get_imu_state()
        if not state['ok'] or state['timestamp'] <= self.last_sample:
            return
        self.last_sample = state['timestamp']
        age = max(0.0, time.monotonic() - state['timestamp'])
        # Stamp reflects actual received sample, not a newly stamped stale getter cache.
        stamp_ns = self.get_clock().now().nanoseconds - int(age * 1e9)
        msg = make_imu_message(state, self.frame, stamp_ns)
        self.publisher.publish(msg)

    def close(self):
        self.stop_event.set()
        self.reader.join(timeout=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ImuNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
