"""Verify freshness and ROS units without opening a USB device."""
from types import SimpleNamespace

import pytest
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Imu

from uw_robot_sensors.imu import driver
from uw_robot_sensors.imu.imu_node import make_imu_message


def test_cached_sample_becomes_stale_and_snapshot_is_independent(monkeypatch):
    state = dict(driver.imu_state, timestamp=10.0, ok=True)
    monkeypatch.setattr(driver, 'imu_state', state)
    monkeypatch.setattr(driver, 'time', SimpleNamespace(monotonic=lambda: 10.2))
    snapshot = driver.get_imu_state()
    assert not snapshot['ok']
    assert snapshot['age'] == pytest.approx(0.2)
    assert state['ok'], 'Snapshot freshness must not mutate the shared SDK state'


def test_imu_ros_serialization_preserves_axes_units_and_sample_stamp():
    state = dict(quaternion=(0.5, -0.5, 0.5, -0.5),
                 gyro=(0.1, -0.2, 0.3), acceleration=(1.0, -2.0, 0.5))
    msg = make_imu_message(state, 'base_link', 12345678901)
    msg = deserialize_message(serialize_message(msg), Imu)
    assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == (12, 345678901)
    assert msg.header.frame_id == 'base_link'
    assert (msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z) == state['quaternion']
    assert (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z) == state['gyro']
    assert (msg.linear_acceleration.x, msg.linear_acceleration.y,
            msg.linear_acceleration.z) == pytest.approx((9.80665, -19.6133, 4.903325))
