"""MS5837 深度传感器：设备层 ``ms5837``、采集服务 ``driver``、ROS 节点 ``depth_node``。

约定
====
* ``ms5837``：只碰寄存器与数学，可在无 I2C、无 ROS 的环境下离线测试；
* ``driver``：采集线程、零点标定状态机、失效判定，不依赖 ROS；
* ``depth_node``：ROS 接口，按需导入（避免离线测试被迫依赖 rclpy）。

因此这里只导入前两层，坐标定 ``depth_node`` 请显式 ``from .depth_node import DepthNode``。
"""

from . import driver, ms5837

__all__ = ["driver", "ms5837"]
