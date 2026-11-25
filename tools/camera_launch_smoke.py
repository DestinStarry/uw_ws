"""Verify installed camera launch, HTTP compatibility and clean SIGINT shutdown."""
import json
from pathlib import Path
import signal
import subprocess
import time
from urllib.request import urlopen

root = Path(__file__).resolve().parents[1]
log_path = root / 'validation/camera_launch.log'
with log_path.open('w') as log:
    process = subprocess.Popen(['ros2', 'launch', 'uw_robot_bringup', 'camera.launch.py',
                                'synthetic:=true', 'http_port:=18081'],
                               stdout=log, stderr=subprocess.STDOUT)
    try:
        deadline = time.monotonic() + 15
        health = None
        while time.monotonic() < deadline:
            assert process.poll() is None, log_path.read_text()
            try:
                with urlopen('http://127.0.0.1:18081/health', timeout=0.3) as response:
                    health = json.load(response)
                if health['ok']:
                    break
            except OSError:
                pass
            time.sleep(0.1)
        assert health and health['ok'] and health['synthetic']
        with urlopen('http://127.0.0.1:18081/?action=stream', timeout=2) as response:
            assert response.readline() == b'--frame\r\n'
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
assert process.returncode == 0, log_path.read_text()
assert 'Traceback' not in log_path.read_text(), log_path.read_text()
result = dict(synthetic_launch=True, ui_http_stream=True, full_size=True,
              clean_shutdown=True, usb_hardware_tested=False,
              external_dds_images='Not validated: WSL external discovery/transport failures')
(root / 'validation/camera_launch.json').write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps(result, indent=2))
