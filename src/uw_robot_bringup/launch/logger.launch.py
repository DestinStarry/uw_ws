"""Standalone recorder: subscribe only, never starts control/hardware nodes."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = LaunchConfiguration('config')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(Path(get_package_share_directory('uw_robot_bringup'))/'config'/'robot.yaml')),
        DeclareLaunchArgument('log_dir', default_value='~/uw_logs'),
        DeclareLaunchArgument('log_label', default_value=''),
        Node(package='uw_robot_logging', executable='flight_logger_node', parameters=[config, {
            'output_dir': ParameterValue(LaunchConfiguration('log_dir'), value_type=str),
            'session_label': ParameterValue(LaunchConfiguration('log_label'), value_type=str),
            'config_path': ParameterValue(config, value_type=str)}], output='screen'),
    ])
