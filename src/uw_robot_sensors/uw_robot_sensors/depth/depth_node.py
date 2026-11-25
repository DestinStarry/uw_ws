"""MS5837 深度节点（ROS 层）：把采集快照发布成 ``DepthState``。

与硬件链路的衔接
================
``hardware_node`` 原来从 STM32 串口解析 ``DEPTH/Depth`` 再发布 ``depth/state``；
改成 I2C 直连 Orin 后由本节点**独占**发布该话题，因此：

* ``control_node`` 不需要任何改动（话题名、消息字段完全一致）；
* 必须把 ``hardware_node.publish_depth`` 置为 ``false``，否则两个发布者会打架；
* ``robot.launch.py`` 的 ``depth`` 参数负责决定是否启动本节点。

发布策略
========
* **有新样本**：按采样率发布，``valid=true``，时间戳回填到采样时刻；
* **无新样本**（标定中 / I2C 故障 / 传感器未连）：以 ``heartbeat_hz`` 低频发布
  ``valid=false`` 的"心跳帧"，并把已失效的 ``data_age_s`` 一并带上。

第二条是刻意设计的：如果失效时主题完全静默，下游只能靠超时猜"是没启动还是坏了"，
操作者也无法从界面上看出深度计掉了。心跳帧让"没数据"变成一条明确的消息。
"""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from uw_robot_interfaces.msg import DepthState

from . import driver, ms5837


def make_depth_message(snapshot: driver.Snapshot, frame_id: str, stamp_ns: int,
                       valid: bool) -> DepthState:
    """把采集快照转换为 ``DepthState``；时间戳为真实采样时刻。"""
    msg = DepthState()
    msg.header.stamp.sec = stamp_ns // 1_000_000_000
    msg.header.stamp.nanosec = stamp_ns % 1_000_000_000
    msg.header.frame_id = frame_id
    msg.depth_m = float(snapshot.depth_m)
    msg.depth_rate_mps = float(snapshot.rate_mps)
    msg.valid = bool(valid)
    msg.data_age_s = float(snapshot.age)
    return msg


class DepthNode(Node):
    """参数校验 → 启动采集线程 → 按策略发布 ``depth/state``。"""

    def __init__(self):
        super().__init__("depth_node")
        self._config = self._declare_config()
        self._frame = self.declare_parameter("frame_id", "base_link").value
        heartbeat = float(self.declare_parameter("heartbeat_hz", 1.0).value)
        if not math.isfinite(heartbeat) or heartbeat <= 0:
            raise driver.ConfigError("heartbeat_hz must be finite and positive")
        self._heartbeat_period = 1.0 / heartbeat
        self._last_sample = 0.0
        self._last_publish = 0.0
        self._was_valid = False

        # 打不开总线就让节点启动失败：深度是定深闭环的输入，静默降级更危险。
        self.sensor = driver.Sensor(self._config, warn=self.get_logger().warning)
        # 与 hardware_node 的发布端一致使用可靠传输，QoS 才能与 control_node 匹配。
        self.publisher = self.create_publisher(DepthState, "depth/state", 10)
        self.sensor.start()
        self.timer = self.create_timer(1.0 / self._config.rate_hz, self.publish_sample)
        self.get_logger().info(
            "MS5837 %s on i2c-%d addr 0x%02X, OSR=%d, density=%.0f, %s, %.1f Hz"
            % (ms5837.MODEL_NAMES[self.sensor.model], self._config.bus,
               self._config.address, 2 ** (8 + self._config.osr),
               self._config.fluid_density,
               "零点固定 %.2f mbar" % self._config.surface_pressure_mbar
               if self._config.surface_pressure_mbar > 0
               else "启动时自动标定零点",
               self._config.rate_hz))

    # -- 参数 ---------------------------------------------------------------

    def _declare_config(self) -> driver.SensorConfig:
        """声明并校验全部参数；任何一个不合法都在启动阶段失败。"""
        declare = self.declare_parameter
        samples = int(declare("oversampling", 4096).value)
        if samples not in ms5837.OSR_BY_SAMPLES:
            raise driver.ConfigError(
                "oversampling 必须是 %s 之一" % sorted(ms5837.OSR_BY_SAMPLES))
        if self.get_parameter("use_sim_time").value:
            raise driver.ConfigError(
                "depth_node 依赖真实 I2C 与单调时钟，不能在 use_sim_time 下运行")
        config = driver.SensorConfig(
            bus=int(declare("bus", 1).value),
            address=int(declare("address", ms5837.DEFAULT_ADDRESS).value),
            model=str(declare("model", "auto").value),
            osr=ms5837.OSR_BY_SAMPLES[samples],
            fluid_density=float(declare("fluid_density",
                                        ms5837.DENSITY_FRESHWATER).value),
            surface_pressure_mbar=float(
                declare("surface_pressure_mbar", 0.0).value),
            expected_surface_mbar=float(
                declare("expected_surface_mbar", 1013.25).value),
            zero_time_s=float(declare("zero_time_s", 3.0).value),
            zero_attempts=int(declare("zero_attempts", 3).value),
            allow_zero_fallback=bool(declare("allow_zero_fallback", False).value),
            rate_hz=float(declare("rate_hz", 20.0).value),
            stale_timeout_s=float(declare("stale_timeout_s", 0.5).value),
            max_failures=int(declare("max_failures", 5).value),
        )
        return config.validate()

    # -- 发布 ---------------------------------------------------------------

    def publish_sample(self) -> None:
        """有新样本就发有效帧；否则按心跳周期发 invalid 帧。"""
        snapshot = self.sensor.snapshot()
        if snapshot.ok and snapshot.phase == driver.PHASE_READY and snapshot.timestamp > self._last_sample:
            self._last_sample = snapshot.timestamp
            self._publish(snapshot, valid=True)
            return
        if self._was_valid and snapshot.ok and snapshot.phase == driver.PHASE_READY:
            return  # 尚无新采样不是故障，不重复发布同一帧。
        if self._was_valid or time.monotonic() - self._last_publish >= self._heartbeat_period:
            if snapshot.phase == driver.PHASE_ZEROING:
                reason = "零点标定中"
            elif snapshot.phase == driver.PHASE_UNCALIBRATED:
                reason = "零点标定失败，需重新标定"
            elif snapshot.failures:
                reason = "读取失败 %d 次" % snapshot.failures
            else:
                reason = "无有效样本"
            self.get_logger().warning("深度无效（%s），发布心跳帧" % reason,
                                      throttle_duration_sec=5.0)
            self._publish(snapshot, valid=False)

    def _publish(self, snapshot: driver.Snapshot, valid: bool) -> None:
        """时间戳回填到采样时刻，而不是发布时刻（OSR 4096 下差约 21 ms）。"""
        if not valid and snapshot.timestamp <= 0.0:
            # 从未成功采样过：没有可用时间戳，直接标记为旧数据。
            stamp_ns = self.get_clock().now().nanoseconds
        else:
            stamp_ns = self.get_clock().now().nanoseconds - int(snapshot.age * 1e9)
        self.publisher.publish(
            make_depth_message(snapshot, self._frame, stamp_ns, valid))
        self._last_publish = time.monotonic()
        self._was_valid = valid

    # -- 生命周期 -----------------------------------------------------------

    def close(self) -> None:
        self.sensor.stop()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DepthNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover - 便于直接运行调试
    main()
