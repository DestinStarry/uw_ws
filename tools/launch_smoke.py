"""Check bringup process lifecycle; topic behavior is tested by integration.py."""
from pathlib import Path
import json
import os
import signal
import subprocess
import time

root = Path(__file__).resolve().parents[1]
(root / 'validation').mkdir(exist_ok=True)
path = root / 'validation/launch.log'
with path.open('w') as log:
    process = subprocess.Popen(
        ['ros2', 'launch', 'uw_robot_bringup', 'robot.launch.py', 'tcp:=false', 'depth_source:=none', 'monitor:=false', 'logging:=false'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            assert process.poll() is None, 'bringup exited early'
            content = path.read_text()
            if 'STM32 dry_run (serial unopened)' in content and '[control_node-1]: process started' in content:
                break
            time.sleep(.1)
        else:
            raise AssertionError('bringup did not start control and hardware nodes')
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
assert process.returncode == 0, f'launch exit code: {process.returncode}'
content = path.read_text()
assert 'process has finished cleanly' in content
assert 'process has died' not in content
report = dict(bringup_started=True, default_dry_run=True, clean_shutdown=True,
              scope='process lifecycle only; topic behavior tested separately by integration.py',
              wsl_external_topic_discovery_intermittent=True)
(root / 'validation/launch.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(report, indent=2))
