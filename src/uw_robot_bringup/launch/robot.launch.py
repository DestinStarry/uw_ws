"""实物/串口组合：控制、分配、通信与可选传感器（不启动仿真器）。

节点
----
control_node / allocation_node / hardware_node（STM32 串口）/ tcp_bridge_node；
可选 imu_node、depth_node（MS5837，I2C）、camera_node。

launch 参数（默认值）
---------------------
dry_run=true   hardware_node 只打印不打开串口，用于地面调试
imu=false      启动 IMU（/dev/ttyCH341USB0）
depth_source=i2c  深度源 i2c/stm32/none；launch 强制互斥，none 为无设备调试
depth=true     兼容旧参数，仅用于关闭 I2C 深度节点
tcp=true       启动 TCP 桥（手柄输入 + 遥测回传）
camera=false   启动双目相机与 HTTP 预览
monitor=true   启动 CPU 温度监测；监测失败不牵连控制链
logging=true   独立记录 CSV/事件日志，log_dir 默认 ~/uw_logs；可设 log_label
日志节点不属于必需控制节点，退出不会联动停止控制链。

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
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_config = str(Path(get_package_share_directory('uw_robot_bringup')) / 'config' / 'robot.yaml')
    config = LaunchConfiguration('config')
    source = LaunchConfiguration('depth_source')
    control = Node(package='uw_robot_control', executable='control_node', parameters=[config], output='screen')
    allocation = Node(package='uw_robot_control', executable='allocation_node', parameters=[config], output='screen')
    hardware = Node(package='uw_robot_communication', executable='hardware_node', parameters=[
        config, {'dry_run': ParameterValue(LaunchConfiguration('dry_run'), value_type=bool),
                 'publish_depth': ParameterValue(PythonExpression(["'", source, "' == 'stm32'"]), value_type=bool)}], output='screen')
    imu = Node(package='uw_robot_sensors', executable='imu_node', parameters=[config],
               condition=IfCondition(LaunchConfiguration('imu')), output='screen')
    depth = Node(package='uw_robot_sensors', executable='depth_node', parameters=[config],
                 condition=IfCondition(PythonExpression(["'", source, "' == 'i2c' and '",
                     LaunchConfiguration('depth'), "'.lower() in ('true', '1')"])), output='screen')
    tcp = Node(package='uw_robot_communication', executable='tcp_bridge_node', parameters=[config],
               condition=IfCondition(LaunchConfiguration('tcp')), output='screen')
    camera = Node(package='uw_robot_sensors', executable='camera_node', parameters=[config],
                  condition=IfCondition(LaunchConfiguration('camera')), output='screen')
    monitor = Node(package='uw_robot_sensors', executable='system_monitor_node', parameters=[config],
                   condition=IfCondition(LaunchConfiguration('monitor')), output='screen')
    logger = Node(package='uw_robot_logging', executable='flight_logger_node', parameters=[config, {
        'output_dir': ParameterValue(LaunchConfiguration('log_dir'), value_type=str),
        'session_label': ParameterValue(LaunchConfiguration('log_label'), value_type=str),
        'config_path': ParameterValue(config, value_type=str),
        'run_mode': 'hardware', 'hardware_mode_known': True,
        'hardware_dry_run': ParameterValue(LaunchConfiguration('dry_run'), value_type=bool)}],
        condition=IfCondition(LaunchConfiguration('logging')), output='screen')
    handlers = [RegisterEventHandler(OnProcessExit(target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason='Required robot node exited'))]))
        for node in (control, allocation, hardware, imu, depth, tcp)]
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('imu', default_value='false'),
        DeclareLaunchArgument('logging', default_value='true', description='Record flight CSV/events independently'),
        DeclareLaunchArgument('log_dir', default_value='~/uw_logs'),
        DeclareLaunchArgument('log_label', default_value=''),
        DeclareLaunchArgument('monitor', default_value='true',
                              description='Publish Orin CPU temperature; optional, not a control dependency'),
        # 数据源由 launch 统一选择，覆盖 YAML 中的串口发布开关以避免双发布。
        DeclareLaunchArgument('depth_source', default_value='i2c', choices=['i2c', 'stm32', 'none'],
                              description='Exclusive depth publisher; default is direct Orin I2C'),
        DeclareLaunchArgument('depth', default_value='true',
                              description='Enable I2C depth node when depth_source=i2c (legacy switch)'),
        DeclareLaunchArgument('tcp', default_value='true'),
        DeclareLaunchArgument('camera', default_value='false',
                              description='Start OpenCV stereo capture and HTTP preview'),
        *handlers, control, allocation, hardware, imu, depth, tcp, camera, monitor, logger,
    ])
