"""Offline fake sysfs + pseudo-UART -> actual hardware_node -> ROS -> TCP."""
import os
from pathlib import Path
import secrets
os.environ["ROS_DOMAIN_ID"] = str(20 + secrets.randbelow(40))
os.environ["ROS_LOCALHOST_ONLY"] = "1"
os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
os.environ.pop("FASTRTPS_DEFAULT_PROFILES_FILE", None)
os.environ.pop("FASTDDS_DEFAULT_PROFILES_FILE", None)
for key in ("ROS_DISCOVERY_SERVER", "ROS_SUPER_CLIENT", "SKIP_DEFAULT_XML"):
    os.environ.pop(key, None)
import pty
import select
import socket
import subprocess
import tempfile
import time
import signal

processes = []
client = None
master, slave = pty.openpty()
os.set_blocking(master, False)
slave_path = os.ttyname(slave)
with tempfile.TemporaryDirectory() as directory:
    root = Path(directory)
    zone = root / "thermal_zone1"
    zone.mkdir()
    (zone / "type").write_text("CPU-therm")
    (zone / "temp").write_text("42500")
    with socket.socket() as reservation:
        reservation.bind(("127.0.0.1", 0))
        port = reservation.getsockname()[1]
    def start(package, executable, params):
        log = open(root / (executable + ".log"), "w")
        p = subprocess.Popen(["ros2", "run", package, executable, "--ros-args", *params],
                             stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append((p, log))
        return p
    try:
        monitor = start("uw_robot_sensors", "system_monitor_node", ["-p", "thermal_root:="+str(root), "-p", "rate_hz:=5.0"])
        start("uw_robot_communication", "tcp_bridge_node", ["-p", "port:="+str(port), "-p", "bind_address:=127.0.0.1", "-p", "telemetry_rate_hz:=5.0", "-p", "cpu_temp_timeout_s:=1.0", "-p", "voltage_timeout_s:=1.0", "-p", "rpm_timeout_s:=1.0"])
        start("uw_robot_communication", "hardware_node", ["-p", "dry_run:=false", "-p", "port:="+slave_path,
                                                           "-p", "publish_depth:=false"])
        deadline = time.monotonic()+15
        while client is None and time.monotonic()<deadline:
            try:
                client=socket.create_connection(("127.0.0.1",port),.2)
                client.setblocking(False)
            except OSError:
                time.sleep(.1)
        assert client is not None, "TCP startup failed"
        def collect(duration, voltage=None):
            end=time.monotonic()+duration
            wire=""
            while time.monotonic()<end:
                if voltage is not None:
                    os.write(master,(voltage+"\r\n").encode())
                # Drain the node's OFF output; never attach to a physical UART.
                while select.select([master],[],[],0)[0]:
                    os.read(master,65536)
                try: wire+=client.recv(65536).decode()
                except BlockingIOError: pass
                time.sleep(.02)
            return wire
        data=collect(5,"VOLT:15.25 TEMP:99 CH0:0 CH1:1234 CH2:-321 CH3:65000 ARMED:1")
        assert "CPU_TEMP:42.50\nCPU_TEMP_VALID:1" in data,data
        assert "VOLT:15.25\nVOLT_VALID:1" in data,data
        assert "\nTEMP:42.50\n" in data and "TEMP:99" not in data,data
        assert "ARMED:1" not in data,data
        for key,value in [("CH0",0),("CH1",1234),("CH2",-321),("CH3",65000)]:
            assert f"{key}:{value}\n{key}_VALID:1" in data,data
        (zone/"temp").unlink()
        data=collect(2)
        assert "CPU_TEMP:--\nCPU_TEMP_VALID:0" in data,data
        assert "VOLT:--\nVOLT_VALID:0" in data,data
        for key in ("CH0","CH1","CH2","CH3"):
            assert f"{key}:--\n{key}_VALID:0" in data,data
        (zone/"temp").write_text("45000")
        data=collect(1.5,"VOLT:16.00 CH0:222 CH1:bad CH2:12.5 CH3:1 CH3:2")
        assert "CPU_TEMP:45.00\nCPU_TEMP_VALID:1" in data,data
        assert "VOLT:16.00\nVOLT_VALID:1" in data,data
        assert "CH0:222\nCH0_VALID:1" in data,data
        for key in ("CH1","CH2","CH3"):
            assert f"{key}:--\n{key}_VALID:0" in data,data
        data=collect(.7,"VOLT:broken")
        assert "VOLT:--\nVOLT_VALID:0" in data,data
        os.killpg(monitor.pid,signal.SIGINT)
        data=collect(2)
        assert "CPU_TEMP:--\nCPU_TEMP_VALID:0" in data,data
        print("PASS: CPU/voltage/RPM forwarding, compatibility, injection rejection, invalidation, recovery, publisher timeout")
    except Exception:
        for p,log in processes:
            log.flush()
            print(Path(log.name).read_text())
        raise
    finally:
        if client: client.close()
        for p,log in processes:
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGINT)
                try:p.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid,signal.SIGKILL);p.wait()
            log.close()
        os.close(master)
        os.close(slave)
