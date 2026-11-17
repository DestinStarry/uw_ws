"""Validate ROS Image wire serialization without depending on WSL multicast."""
import time

import rclpy
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Image

from uw_robot_sensors.camera.camera_node import CameraNode


class Receiver:
    def __init__(self):
        self.messages = []

    def get_subscription_count(self):
        return 1

    def publish(self, message):
        self.messages.append(deserialize_message(serialize_message(message), Image))


def test_full_size_ros_image_adapter_and_stereo_stamp():
    rclpy.init(args=['--ros-args', '-p', 'synthetic:=true', '-p', 'http_port:=0'])
    node = None
    try:
        node = CameraNode()
        left, right = Receiver(), Receiver()
        node.left_pub, node.right_pub = left, right
        deadline = time.monotonic() + 3
        while node.capture.latest_frame() is None and time.monotonic() < deadline:
            time.sleep(0.01)
        node.publish_images()
        assert left.messages and right.messages
        a, b = left.messages[0], right.messages[0]
        assert a.header.stamp == b.header.stamp
        assert a.header.frame_id == 'camera_left_optical_frame'
        assert b.header.frame_id == 'camera_right_optical_frame'
        for message in (a, b):
            assert (message.width, message.height, message.step) == (1280, 720, 3840)
            assert message.encoding == 'bgr8'
            assert len(message.data) == 1280 * 720 * 3
    finally:
        if node:
            node.close()
            node.destroy_node()
        rclpy.shutdown()
