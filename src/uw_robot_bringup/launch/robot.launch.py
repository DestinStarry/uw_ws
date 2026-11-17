"""实物/串口空运行组合：控制、分配、通信及可选传感器。

默认 dry_run=true，不打开 STM32 串口；启用真实设备需显式覆盖。
必需节点退出会联动停止整组；相机退出仍保持原有的独立策略。
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
    tcp = Node(package='uw_robot_communication', executable='tcp_bridge_node', parameters=[config],
               condition=IfCondition(LaunchConfiguration('tcp')), output='screen')
    camera = Node(package='uw_robot_sensors', executable='camera_node', parameters=[config],
                  condition=IfCondition(LaunchConfiguration('camera')), output='screen')
    handlers = [RegisterEventHandler(OnProcessExit(target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason='Required robot node exited'))]))
        for node in (control, allocation, hardware, imu, tcp)]
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('imu', default_value='false'),
        DeclareLaunchArgument('tcp', default_value='true'),
        DeclareLaunchArgument('camera', default_value='false',
                              description='Start OpenCV stereo capture and HTTP preview'),
        *handlers, control, allocation, hardware, imu, tcp, camera,
    ])
