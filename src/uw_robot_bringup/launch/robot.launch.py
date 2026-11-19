"""实物/串口组合：控制、分配、通信与可选传感器（不启动仿真器）。

节点
----
control_node / allocation_node / hardware_node（STM32 串口）/ tcp_bridge_node；
可选 imu_node、depth_node（MS5837，I2C）、camera_node。

launch 参数（默认值）
---------------------
dry_run=true   hardware_node 只打印不打开串口，用于地面调试
imu=false      启动 IMU（/dev/ttyUSB0）
depth=true     启动 MS5837（I2C）；必须与 hardware_node.publish_depth 互斥，
               两者恰好一个深度源，否则 depth/state 会无人发布或双发布
tcp=true       启动 TCP 桥（手柄输入 + 遥测回传）
camera=false   启动双目相机与 HTTP 预览

行为
----
必需节点退出会联动停止整组；相机退出不牵连控制链（单独策略）。
"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_config = str(Path(get_package_share_directory('uw_robot_bringup')) / 'config' / 'robot.yaml')
    config = LaunchConfiguration('config')
    control = Node(package='uw_robot_control', executable='control_node', parameters=[config], output='screen')
    allocation = Node(package='uw_robot_control', executable='allocation_node', parameters=[config], output='screen')
    hardware = Node(package='uw_robot_communication', executable='hardware_node', parameters=[
        config, {'dry_run': ParameterValue(LaunchConfiguration('dry_run'), value_type=bool)}], output='screen')
    imu = Node(package='uw_robot_sensors', executable='imu_node', parameters=[config],
               condition=IfCondition(LaunchConfiguration('imu')), output='screen')
    depth = Node(package='uw_robot_sensors', executable='depth_node', parameters=[config],
                 condition=IfCondition(LaunchConfiguration('depth')), output='screen')
    tcp = Node(package='uw_robot_communication', executable='tcp_bridge_node', parameters=[config],
               condition=IfCondition(LaunchConfiguration('tcp')), output='screen')
    camera = Node(package='uw_robot_sensors', executable='camera_node', parameters=[config],
                  condition=IfCondition(LaunchConfiguration('camera')), output='screen')
    handlers = [RegisterEventHandler(OnProcessExit(target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason='Required robot node exited'))]))
        for node in (control, allocation, hardware, imu, depth, tcp)]
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('imu', default_value='false'),
        # 深度默认启动：hardware_node.publish_depth 已为 false，
        # 两者必须恰好有一个深度源，否则 depth/state 要么无人发布、要么双发布。
        DeclareLaunchArgument('depth', default_value='true',
                              description='Start MS5837 over I2C; requires hardware_node.publish_depth=false'),
        DeclareLaunchArgument('tcp', default_value='true'),
        DeclareLaunchArgument('camera', default_value='false',
                              description='Start OpenCV stereo capture and HTTP preview'),
        *handlers, control, allocation, hardware, imu, depth, tcp, camera,
    ])
