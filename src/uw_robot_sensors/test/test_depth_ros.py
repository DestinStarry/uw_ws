"""实际 ROS 节点 + 真实设备/服务层；仅把 SMBus 换成寄存器级替身。"""
import time
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.serialization import serialize_message, deserialize_message
from uw_robot_interfaces.msg import DepthState
from uw_robot_sensors.depth import driver, ms5837
from uw_robot_sensors.depth.depth_node import DepthNode
from test_depth import _prom, _d1_for_pressure, _d2_for_celsius


class Bus:
    def __init__(self, model=ms5837.MODEL_02BA):
        self.model = model
        self.prom = _prom()
        self.command = 0
        self.failed = False
        self.closed = False
        self.pressure = 1013.25

    def write_byte(self, address, command):
        if self.failed:
            raise OSError("I2C disconnected")
        assert address == 0x76
        self.command = command

    def read_i2c_block_data(self, address, register, length):
        if self.failed:
            raise OSError("I2C disconnected")
        if register >= 0xA0:
            word = self.prom[(register - 0xA0) // 2]
            return [word >> 8, word & 255]
        value = (_d1_for_pressure(self.prom, self.pressure, self.model)
                 if self.command < 0x50 else _d2_for_celsius(self.prom, 20.0))
        return [(value >> 16) & 255, (value >> 8) & 255, value & 255]

    def close(self):
        self.closed = True


class Receiver:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(deserialize_message(serialize_message(message), DepthState))


@pytest.mark.parametrize("model,device_model,osr", [
    ("auto", ms5837.MODEL_02BA, 4096),
    ("02ba", ms5837.MODEL_02BA, 4096),
    ("30ba", ms5837.MODEL_30BA, 8192),
])
def test_real_depth_node_calibration_disconnect_recovery(monkeypatch, model, device_model, osr):
    bus = Bus(device_model)
    monkeypatch.setattr(ms5837, "smbus", SimpleNamespace(SMBus=lambda _: bus))
    rclpy.init(args=['--ros-args', '-p', 'model:=' + model, '-p', 'zero_time_s:=0.2',
                     '-p', 'rate_hz:=50.0', '-p', 'heartbeat_hz:=1.0',
                     '-p', 'oversampling:=' + str(osr)])
    node = None
    try:
        node = DepthNode()  # also regresses use_sim_time redeclaration
        receiver = Receiver()
        node.publisher = receiver

        def spin_until(predicate):
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.01)
                if predicate():
                    return
            raise AssertionError("depth node did not reach expected state")

        spin_until(lambda: any(m.valid for m in receiver.messages))
        assert not receiver.messages[0].valid
        assert node.sensor._client.model == device_model
        valid = next(m for m in receiver.messages if m.valid)
        assert valid.depth_m == pytest.approx(0., abs=.01)
        assert valid.header.frame_id == 'base_link'
        assert 0 <= valid.data_age_s < .2
        stamp = valid.header.stamp.sec * 10**9 + valid.header.stamp.nanosec
        assert 0 <= node.get_clock().now().nanoseconds - stamp < 500_000_000
        zero = node.sensor.snapshot().surface_mbar
        bus.failed = True
        failed_at = time.monotonic()
        count = len(receiver.messages)
        spin_until(lambda: any(not m.valid for m in receiver.messages[count:]))
        assert time.monotonic() - failed_at < .3  # 不等 1 秒心跳
        bus.pressure += 100.0
        bus.failed = False
        count = len(receiver.messages)
        spin_until(lambda: any(m.valid for m in receiver.messages[count:]))
        recovered = next(m for m in receiver.messages[count:] if m.valid)
        assert recovered.depth_m == pytest.approx(1.0228, abs=.02)
        assert node.sensor.snapshot().surface_mbar == zero
        assert recovered.depth_rate_mps == pytest.approx(0., abs=.01)
    finally:
        if node:
            node.close()
            node.destroy_node()
        rclpy.shutdown()
    assert bus.closed


def test_real_node_rejects_sim_clock_before_opening_device():
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    try:
        with pytest.raises(driver.ConfigError, match='use_sim_time'):
            DepthNode()
    finally:
        rclpy.shutdown()


def test_empty_prom_is_rejected_and_bus_closed(monkeypatch):
    bus = Bus()
    bus.prom = [0] * 7
    monkeypatch.setattr(ms5837, "smbus", SimpleNamespace(SMBus=lambda _: bus))
    device = ms5837.MS5837(1)
    with pytest.raises(ms5837.SensorError, match='calibration'):
        device.open()
    assert bus.closed
