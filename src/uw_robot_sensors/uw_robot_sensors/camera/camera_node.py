"""相机 ROS 接口：参数、按订阅需求发布 Image 和健康检查，不执行控制算法。"""
import signal
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from .capture import Settings, StereoCapture
from .stream import PreviewServer


def make_image_message(image, frame_id, stamp_ns):
    """构造 BGR8 消息；两目共用一帧时间戳，保留各自光学坐标系。"""
    msg = Image()
    msg.header.stamp.sec = stamp_ns // 1000000000
    msg.header.stamp.nanosec = stamp_ns % 1000000000
    msg.header.frame_id = frame_id
    msg.height, msg.width = image.shape[:2]
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = image.tobytes()
    return msg


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        defaults = dict(device='/dev/video0', width=2560, height=720, fps=30.0,
                        jpeg_quality=80, swap_eyes=True, stale_timeout_s=2.0,
                        synthetic=False, http_host='0.0.0.0', http_port=8080,
                        publish_rate_hz=15.0, left_frame_id='camera_left_optical_frame',
                        right_frame_id='camera_right_optical_frame')
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        p = lambda name: self.get_parameter(name).value
        rate = float(p('publish_rate_hz'))
        if not 0 < rate <= 240:
            raise ValueError('publish_rate_hz must be in (0, 240]')
        settings = Settings(device=p('device'), width=p('width'), height=p('height'),
                            fps=p('fps'), quality=p('jpeg_quality'), swap_eyes=p('swap_eyes'),
                            stale_timeout=p('stale_timeout_s'), synthetic=p('synthetic'))
        self.capture = StereoCapture(settings, lambda: self.get_clock().now().nanoseconds)
        self.preview = PreviewServer(self.capture, p('http_host'), p('http_port'))
        self._closed = False
        self._sequence = 0
        self._last_error = ''
        self.left_frame = p('left_frame_id')
        self.right_frame = p('right_frame_id')
        self.left_pub = self.create_publisher(Image, 'camera/left/image_raw', qos_profile_sensor_data)
        self.right_pub = self.create_publisher(Image, 'camera/right/image_raw', qos_profile_sensor_data)
        try:
            self.capture.start()
            self.preview.start()
            self.timer = self.create_timer(1.0 / rate, self.publish_images)
            self.health_timer = self.create_timer(1.0, self.check_health)
        except BaseException:
            self.close()
            raise
        self.get_logger().info('OpenCV stereo {}x{} @ {} requested FPS; HTTP :{} /?action=stream{}'.format(
            settings.width, settings.height, settings.fps, p('http_port'),
            ' [SYNTHETIC TEST IMAGES]' if settings.synthetic else ''))

    def publish_images(self):
        # No subscriber => no raw frame serialization or DDS image bandwidth.
        if not (self.left_pub.get_subscription_count() or self.right_pub.get_subscription_count()):
            return
        frame = self.capture.latest_frame()
        if frame is None or frame.sequence == self._sequence:
            return
        self._sequence = frame.sequence
        for publisher, image, frame_id in ((self.left_pub, frame.left, self.left_frame),
                                            (self.right_pub, frame.right, self.right_frame)):
            if not publisher.get_subscription_count():
                continue
            msg = make_image_message(image, frame_id, frame.ros_stamp_ns)
            publisher.publish(msg)

    def check_health(self):
        error = self.capture.error
        if error and error != self._last_error:
            self.get_logger().error('Camera stopped: ' + error)
            self._last_error = error

    def close(self):
        if self._closed:
            return
        self._closed = True
        self.preview.stop()
        self.capture.stop()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # launch与进程组可能同时发送退出信号，清理期间不重复中断资源释放。
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
