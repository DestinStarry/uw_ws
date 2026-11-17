"""Offline ROS graph + TCP + pseudo-serial test. Never opens a physical device."""
import os
import secrets
from pathlib import Path
os.environ['ROS_DOMAIN_ID'] = str(20 + secrets.randbelow(40))
os.environ['ROS_LOCALHOST_ONLY'] = '0'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(Path(__file__).with_name('dds_test.xml'))
for key in ('ROS_DISCOVERY_SERVER', 'ROS_SUPER_CLIENT', 'SKIP_DEFAULT_XML'):
    os.environ.pop(key, None)
import json
import pty
import select
import signal
import socket
import subprocess
import time

import rclpy
from sensor_msgs.msg import Imu
from uw_robot_interfaces.msg import RobotStatus, DepthState
from std_msgs.msg import String

root = Path(__file__).resolve().parents[1]
processes = []
logs = []
master, slave = pty.openpty()
slave_path = os.ttyname(slave)
os.set_blocking(master, False)
report = {}
client = None
node = None


def start(package, executable, extra):
    log = open(root / 'validation' / (executable + '.log'), 'w')
    logs.append(log)
    config = str(root / 'src/uw_robot_bringup/config/robot.yaml')
    process = subprocess.Popen(['ros2', 'run', package, executable, '--ros-args', '--params-file', config, *extra],
                               stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    processes.append(process)


def receive_serial():
    data = b''
    while select.select([master], [], [], 0)[0]:
        data += os.read(master, 4096)
    return data.decode(errors='replace')


try:
    (root / 'validation').mkdir(exist_ok=True)
    start('uw_robot_control', 'control_node', [])
    start('uw_robot_communication', 'hardware_node', ['-p', 'dry_run:=false', '-p', 'port:=' + slave_path])
    start('uw_robot_communication', 'tcp_bridge_node', ['-p', 'port:=18888', '-p', 'bind_address:=127.0.0.1'])
    start('uw_robot_control', 'allocation_node', [])
    rclpy.init()
    node = rclpy.create_node('offline_integration_test')
    publisher = node.create_publisher(Imu, 'imu/data', 10)
    statuses = []
    depths = []
    raw_lines = []
    subscription = node.create_subscription(RobotStatus, 'robot/status', statuses.append, 10)
    depth_subscription = node.create_subscription(DepthState, 'depth/state', depths.append, 10)
    raw_subscription = node.create_subscription(String, 'hardware/rx', raw_lines.append, 10)
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        try:
            client = socket.create_connection(('127.0.0.1', 18888), timeout=.2)
            client.setblocking(False)
            break
        except OSError:
            rclpy.spin_once(node, timeout_sec=.05)
    assert client, 'TCP bridge unavailable'

    def run(duration, arm=0, estop=0, u=0.0, send_imu=True, send_cmd=True):
        end = time.monotonic() + duration
        wire = ''
        while time.monotonic() < end:
            if send_imu:
                imu = Imu()
                imu.header.stamp = node.get_clock().now().to_msg()
                imu.orientation.w = 1.0
                publisher.publish(imu)
            if send_cmd:
                client.sendall(f'CMD:{u},0,0,0,0,0,{arm},{estop},0,0\n'.encode())
            try:
                client.recv(8192)
            except BlockingIOError:
                pass
            os.write(master, b'Depth:1.250m DEPTH_MS:100\n')
            for _ in range(8):
                rclpy.spin_once(node, timeout_sec=0)
            wire += receive_serial()
            time.sleep(.02)
        return wire

    run(2.5)
    discovery_deadline = time.monotonic() + 8
    while (not statuses or not statuses[-1].depth_valid or not statuses[-1].imu_valid) and time.monotonic() < discovery_deadline:
        assert all(p.poll() is None for p in processes), 'node failed; inspect validation/*.log'
        run(.25)
    print('Discovered:', node.get_node_names(), 'statuses:', len(statuses), flush=True)
    print('Depth diagnostic:', str(depths[-1]) if depths else 'none',
          'raw:', [v.data for v in raw_lines[-3:]], flush=True)
    assert statuses and not statuses[-1].armed
    assert statuses[-1].depth_valid, 'STM32 pseudo-serial depth not received after DDS discovery'
    report['serial_depth_parse'] = True
    initial = receive_serial() + run(.1)
    assert 'CTRL:' not in initial, initial
    report['default_off'] = True
    assert publisher.get_subscription_count() > 0, 'IMU input has no matching subscriber'
    run(.3, arm=1)
    run(.3)
    active = run(2.0, u=.2)
    assert statuses[-1].armed and 'CTRL:' in active, str(statuses[-1]) + '\n' + active[-500:]
    commands = [line for line in active.splitlines() if line.startswith('CTRL:')]
    assert any(any(abs(float(v)) > .0001 for v in line[5:].split(',')[:4]) for line in commands)
    report['arm_tcp_control_serial'] = True
    run(.3, u=.2, send_imu=False)
    assert not statuses[-1].armed
    assert 'OFF' in run(.1, send_imu=False)
    report['imu_timeout_off'] = True
    run(.3)
    run(.15, arm=1)
    run(.15)
    run(.6, u=.2)
    run(.6, send_cmd=False)
    assert not statuses[-1].armed
    report['cmd_timeout_off'] = True
    # Stop the controller during a real nonzero command, not after an earlier stop.
    run(.4)
    run(.3, arm=1)
    run(.3)
    moving = run(1.2, u=.2)
    assert statuses[-1].armed, 'controller did not rearm for upstream watchdog test'
    moving_commands = [line for line in moving.splitlines() if line.startswith('CTRL:')]
    assert any(any(abs(float(v)) > .0001 for v in line[5:].split(',')[:4])
               for line in moving_commands), moving[-500:]
    receive_serial()
    # Keep allocation and hardware alive while stopping only the control process.
    os.killpg(processes[0].pid, signal.SIGINT)
    time.sleep(.4)
    tail = receive_serial()
    assert tail.strip().endswith('OFF'), tail
    report['control_wrench_watchdog_off'] = True
    assert all(p.poll() is None for p in processes[1:]), 'hardware/TCP node exited unexpectedly'
    os.killpg(processes[3].pid, signal.SIGINT)
    processes[3].wait(timeout=5)
    # Inject only into the pseudo-serial hardware test after the allocator exited.
    from uw_robot_interfaces.msg import ThrusterCommand
    direct = node.create_publisher(ThrusterCommand, 'actuators/command', 1)
    match_deadline = time.monotonic() + 5
    while direct.get_subscription_count() == 0 and time.monotonic() < match_deadline:
        rclpy.spin_once(node, timeout_sec=.05)
    assert direct.get_subscription_count() > 0, 'hardware command subscriber absent'
    for _ in range(20):
        frame = ThrusterCommand()
        frame.header.stamp = node.get_clock().now().to_msg()
        frame.enabled = True
        frame.thrust_newtons = [.2, .2, .2, .2]
        direct.publish(frame)
        rclpy.spin_once(node, timeout_sec=0)
        time.sleep(.02)
    active_wire = receive_serial()
    assert 'CTRL:' in active_wire, active_wire
    time.sleep(.35)
    assert receive_serial().strip().endswith('OFF')
    report['allocation_exit_hardware_watchdog_off'] = True
    (root / 'validation/integration.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
finally:
    if client:
        client.close()
    for p in processes:
        if p.poll() is None:
            os.killpg(p.pid, signal.SIGINT)
    for p in processes:
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)
            p.wait()
    if node:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    for log in logs:
        log.close()
    os.close(master)
    os.close(slave)
