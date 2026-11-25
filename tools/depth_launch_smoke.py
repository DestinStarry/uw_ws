"""验证 none/stm32 启动时的深度发布者与消息；全部 dry-run，不打开串口。"""
import json
import os
from pathlib import Path
import secrets
import signal
import subprocess
import tempfile
import time


def check_source(source, domain):
    # 每轮独立发现域，避免退出节点的 DDS 缓存影响发布者计数。
    os.environ['ROS_DOMAIN_ID'] = str(domain)
    os.environ['ROS_LOCALHOST_ONLY'] = '0'
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(Path(__file__).with_name('dds_test.xml'))
    for key in ('ROS_DISCOVERY_SERVER', 'ROS_SUPER_CLIENT', 'SKIP_DEFAULT_XML'):
        os.environ.pop(key, None)
    import rclpy
    from uw_robot_interfaces.msg import DepthState
    rclpy.init(domain_id=domain)
    node = rclpy.create_node('depth_source_smoke')
    print('Depth source:', source, 'domain:', node.context.get_domain_id(), flush=True)
    received = []
    node.create_subscription(DepthState, 'depth/state', received.append, 10)
    log = tempfile.TemporaryFile(mode='w+')
    process = subprocess.Popen([
        'ros2', 'launch', 'uw_robot_bringup', 'robot.launch.py',
        'depth_source:=' + source, 'dry_run:=true', 'tcp:=false',
        'imu:=false', 'camera:=false'], stdout=log, stderr=subprocess.STDOUT,
        start_new_session=True)
    try:
        deadline = time.monotonic() + 20
        stable_since = None
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise AssertionError('bringup exited early')
            rclpy.spin_once(node, timeout_sec=.05)
            names = set(node.get_node_names())
            if not {'hardware_node', 'control_node', 'allocation_node'} <= names:
                continue
            if source == 'none':
                if stable_since is None:
                    stable_since = time.monotonic()
                if time.monotonic() - stable_since < 1:
                    continue
                assert not node.get_publishers_info_by_topic('depth/state')
                assert not received
            else:
                if not received:
                    continue
                publishers = node.get_publishers_info_by_topic('depth/state')
                assert [p.node_name for p in publishers] == ['hardware_node']
                assert not any(m.valid for m in received), 'dry-run must not invent depth'
            assert 'depth_node' not in names
            return dict(source=source, passed=True)
        raise AssertionError('depth source did not become ready; nodes=' + repr(node.get_node_names()))
    except Exception:
        log.seek(0)
        print(log.read())
        raise
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        node.destroy_node()
        rclpy.shutdown()
        log.close()


if __name__ == '__main__':
    domain = 20 + secrets.randbelow(35)
    print(json.dumps([check_source('none', domain), check_source('stm32', domain + 1)], indent=2))
