"""Orin CPU temperature: sysfs -> ROS Temperature at 1 Hz; no shell/watch process.

Only CPU-named thermal zones are accepted. Missing/invalid readings publish NaN,
so the TCP bridge can invalidate its cached value without stopping robot control.
"""
import math
from pathlib import Path
import re

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Temperature


def read_cpu_temperature(thermal_root, cpu_zone="auto"):
    """Return (degrees Celsius, zone path); never label a GPU/SOC zone as CPU."""
    root = Path(thermal_root)
    if cpu_zone != "auto" and not re.fullmatch(r"thermal_zone[0-9]+", cpu_zone):
        raise ValueError("cpu_zone must be auto or thermal_zoneN")
    zones = sorted(root.glob("thermal_zone*")) if cpu_zone == "auto" else [root / cpu_zone]
    errors = []
    for zone in zones:
        try:
            kind = (zone / "type").read_text(encoding="utf-8").strip().lower()
            if "cpu" not in kind:
                continue
            value = float((zone / "temp").read_text(encoding="utf-8").strip()) / 1000.0
            if not math.isfinite(value) or not -100.0 <= value <= 200.0:
                raise ValueError("CPU temperature is not a plausible finite Celsius value")
            return value, str(zone)
        except (OSError, ValueError) as exc:
            errors.append(str(exc))
    raise ValueError("No readable CPU thermal zone: " + ("; ".join(errors) or str(root)))


class SystemMonitorNode(Node):
    def __init__(self):
        super().__init__("system_monitor_node")
        self.thermal_root = self.declare_parameter("thermal_root", "/sys/class/thermal").value
        self.cpu_zone = self.declare_parameter("cpu_zone", "auto").value
        rate = float(self.declare_parameter("rate_hz", 1.0).value)
        if not math.isfinite(rate) or not .1 <= rate <= 10.0:
            raise ValueError("system monitor rate_hz must be in [0.1, 10]")
        if self.cpu_zone != "auto" and not re.fullmatch(r"thermal_zone[0-9]+", self.cpu_zone):
            raise ValueError("cpu_zone must be auto or thermal_zoneN")
        if self.get_parameter("use_sim_time").value:
            raise ValueError("system_monitor_node requires the real system clock")
        self.source = None
        self.publisher = self.create_publisher(Temperature, "system/cpu_temperature", 1)
        self.timer = self.create_timer(1.0 / rate, self.publish_temperature)
        self.publish_temperature()

    def publish_temperature(self):
        message = Temperature()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "cpu"
        message.variance = 0.0  # Unknown variance, not a claim of zero noise.
        try:
            message.temperature, source = read_cpu_temperature(self.thermal_root, self.cpu_zone)
            if source != self.source:
                self.get_logger().info("CPU temperature source: " + source)
                self.source = source
        except (OSError, ValueError) as exc:
            message.temperature = float("nan")
            self.get_logger().warning("CPU temperature unavailable: " + str(exc),
                                      throttle_duration_sec=10.0)
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SystemMonitorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
