"""只启动双目相机与 HTTP 预览；``synthetic:=true`` 时用合成图案，可脱离 USB 调试。

参数：synthetic（默认 false）、http_port（8080）、device（/dev/video0）。
"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = str(Path(get_package_share_directory('uw_robot_bringup')) / 'config' / 'robot.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config),
        DeclareLaunchArgument('synthetic', default_value='false'),
        DeclareLaunchArgument('http_port', default_value='8080'),
        Node(package='uw_robot_sensors', executable='camera_node', output='screen', parameters=[
            LaunchConfiguration('config'),
            {'synthetic': ParameterValue(LaunchConfiguration('synthetic'), value_type=bool),
             'http_port': ParameterValue(LaunchConfiguration('http_port'), value_type=int)}]),
    ])
