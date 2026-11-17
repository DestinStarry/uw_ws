"""Check package grouping, entry points and dependency boundaries."""
from pathlib import Path
import json
import subprocess
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[1]
expected = {
    'uw_robot_control': {'control_node', 'allocation_node'},
    'uw_robot_sensors': {'imu_node', 'camera_node'},
    'uw_robot_communication': {'tcp_bridge_node', 'hardware_node'},
    'uw_robot_mujoco': {'mujoco_node'},
    'uw_robot_interfaces': set(),
    'uw_robot_bringup': set(),
}
actual = {p.parent.name for p in (root / 'src').glob('*/package.xml')}
assert actual == set(expected), actual
for package, names in expected.items():
    result = subprocess.run(['ros2', 'pkg', 'executables', package],
                            capture_output=True, text=True, check=True)
    found = {line.split()[1] for line in result.stdout.splitlines() if line.strip()}
    assert found == names, (package, found, names)
    prefix = subprocess.check_output(['ros2', 'pkg', 'prefix', package], text=True).strip()
    assert Path(prefix).resolve().is_relative_to(root / 'install'), prefix
    tree = ET.parse(root / 'src' / package / 'package.xml')
    dependencies = {e.text for e in tree.getroot() if e.tag.endswith('depend')}
    if package in ('uw_robot_sensors', 'uw_robot_communication'):
        assert 'uw_robot_control' not in dependencies
    if package == 'uw_robot_interfaces':
        assert not (dependencies & (set(expected) - {'uw_robot_interfaces'}))
old = {'uw_robot_allocation', 'uw_robot_hardware', 'uw_robot_teleop',
       'uw_robot_camera', 'uw_robot_imu'}
for package in old:
    result = subprocess.run(['ros2', 'pkg', 'prefix', package], capture_output=True, text=True)
    assert result.returncode != 0, 'Old package still discoverable: ' + package
report = dict(packages=6, business_packages=3, nodes=7,
              no_stale_packages=True, entry_points=True, dependency_boundaries=True)
(root / 'validation').mkdir(exist_ok=True)
(root / 'validation/package_layout.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(report, indent=2))
