"""仅启动 Orin I2C 深度采集，供上电标定和接线验收，不启动执行器链。"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = str(Path(get_package_share_directory('uw_robot_bringup')) / 'config' / 'robot.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config),
        Node(package='uw_robot_sensors', executable='depth_node',
             parameters=[LaunchConfiguration('config')], output='screen'),
    ])
