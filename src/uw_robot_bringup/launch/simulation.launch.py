"""Same nominal controller/allocation profile as hardware; only the plant is simulated."""
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
    share = Path(get_package_share_directory('uw_robot_bringup'))
    config, plant = LaunchConfiguration('config'), LaunchConfiguration('plant')
    # Sole backend convention adapter: preserve existing hardware -1 signs; model axes use +1.
    # Gains, deadzones, estimated geometry and hydrostatic compensation all come from config.
    control = Node(package='uw_robot_control', executable='control_node',
                   parameters=[config, {'moment_sign': [1.,1.,1.]}], output='screen')
    allocation = Node(package='uw_robot_control', executable='allocation_node',
                      parameters=[config], output='screen')
    sim = Node(package='uw_robot_mujoco', executable='mujoco_node', parameters=[plant, {
        'headless': ParameterValue(LaunchConfiguration('headless'), value_type=bool),
        'publish_truth': ParameterValue(LaunchConfiguration('publish_truth'), value_type=bool),
        'camera_enabled': ParameterValue(LaunchConfiguration('camera'), value_type=bool),
        'camera_http_port': ParameterValue(LaunchConfiguration('camera_port'), value_type=int),
    }], output='screen')
    tcp = Node(package='uw_robot_communication', executable='tcp_bridge_node',
               parameters=[config, {'port': ParameterValue(LaunchConfiguration('tcp_port'), value_type=int)}],
               condition=IfCondition(LaunchConfiguration('tcp')), output='screen')
    nodes = [control, allocation, sim, tcp]
    handlers = [RegisterEventHandler(OnProcessExit(target_action=n,
                on_exit=[EmitEvent(event=Shutdown(reason='Simulation node exited'))])) for n in nodes]
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(share/'config/robot.yaml')),
        DeclareLaunchArgument('plant', default_value=str(share/'config/plant.yaml')),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('publish_truth', default_value='false'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('camera_port', default_value='8080'),
        DeclareLaunchArgument('tcp', default_value='true'),
        DeclareLaunchArgument('tcp_port', default_value='8888'),
        *handlers, *nodes,
    ])
