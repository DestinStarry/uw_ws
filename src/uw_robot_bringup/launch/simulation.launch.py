"""仿真组合：控制器/分配器参数与实物一致，只把被控对象换成 MuJoCo。

与实物的差异
------------
* 不启动 hardware_node，也不使用真实 IMU/深度计；仿真节点直接发布
  ``imu/data``、``depth/state``、``simulation/pose``；
* ``moment_sign`` 由 launch 覆盖为 +1（仿真几何约定），实物为 -1；
* ``headless``、``camera``、``publish_truth``、``tcp_port`` 仅仿真使用。

参数：config（控制/分配参数）、plant（对象参数）、headless、camera、publish_truth、tcp_port。
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
    logger = Node(package='uw_robot_logging', executable='flight_logger_node', parameters=[config, {
        'output_dir': ParameterValue(LaunchConfiguration('log_dir'), value_type=str),
        'session_label': ParameterValue(LaunchConfiguration('log_label'), value_type=str),
        'config_path': ParameterValue(config, value_type=str), 'run_mode': 'simulation', 'hardware_dry_run': True, 'hardware_mode_known': True}],
        condition=IfCondition(LaunchConfiguration('logging')), output='screen')
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
        DeclareLaunchArgument('logging', default_value='false'),
        DeclareLaunchArgument('log_dir', default_value='~/uw_logs'),
        DeclareLaunchArgument('log_label', default_value='simulation'),
        *handlers, *nodes, logger,
    ])
