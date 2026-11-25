"""Verify logger storage failure and process exit never stop dry-run control bringup."""
import os
import csv
import json
import secrets
os.environ["ROS_DOMAIN_ID"]=str(20+secrets.randbelow(40))
os.environ["ROS_LOCALHOST_ONLY"]="1"
os.environ["RMW_IMPLEMENTATION"]="rmw_fastrtps_cpp"
for key in ("FASTRTPS_DEFAULT_PROFILES_FILE","FASTDDS_DEFAULT_PROFILES_FILE","ROS_DISCOVERY_SERVER","ROS_SUPER_CLIENT"):
    os.environ.pop(key,None)
import signal
import re
import subprocess
import tempfile
import time
from pathlib import Path
import yaml

root=Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory() as directory:
    directory=Path(directory)
    params=yaml.safe_load((root/"src/uw_robot_bringup/config/robot.yaml").read_text())
    params["flight_logger_node"]["ros__parameters"].update(min_free_mb=1e12,fsync=False)
    config=directory/"config.yaml"
    config.write_text(yaml.safe_dump(params),encoding="utf-8")
    log_path=directory/"launch.log"
    with log_path.open("w") as output:
        process=subprocess.Popen(["ros2","launch","uw_robot_bringup","robot.launch.py",
            "config:="+str(config),"dry_run:=true","depth_source:=none","imu:=false",
            "camera:=false","tcp:=false","monitor:=false","logging:=true","log_dir:="+str(directory/"logs")],
            stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            deadline=time.monotonic()+20
            while time.monotonic()<deadline:
                assert process.poll() is None,"control group exited on logger disk limit"
                content=log_path.read_text()
                if "Flight logger disk_low" in content and "STM32 dry_run (serial unopened)" in content:break
                time.sleep(.1)
            else:raise AssertionError("bringup/logger startup timed out")
            logger=re.search(r'\[flight_logger_node-\d+\]: process started with pid \[(\d+)\]',content)
            assert logger,"logger process absent"
            os.kill(int(logger.group(1)),signal.SIGTERM)
            time.sleep(.8)
            assert process.poll() is None,"logger exit stopped the control group"
            assert "Required robot node exited" not in log_path.read_text()
        except Exception:
            print(log_path.read_text());raise
        finally:
            if process.poll() is None:process.send_signal(signal.SIGINT)
            try:process.wait(timeout=10)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
    print("PASS: disk limit and logger process termination do not stop dry-run control/hardware nodes")

# Normal group: real C++ state/serial publishers must reach the separate logger.
with tempfile.TemporaryDirectory() as temporary:
    directory=Path(temporary)
    params=yaml.safe_load((root/"src/uw_robot_bringup/config/robot.yaml").read_text())
    params["flight_logger_node"]["ros__parameters"].update(min_free_mb=0.,sample_hz=20.,fsync=True)
    config=directory/"config.yaml";config.write_text(yaml.safe_dump(params))
    path=directory/"launch.log"
    session=None
    with path.open("w") as output:
        process=subprocess.Popen(["ros2","launch","uw_robot_bringup","robot.launch.py",
            "config:="+str(config),"dry_run:=true","depth_source:=none","imu:=false",
            "camera:=false","tcp:=false","monitor:=false","logging:=true","log_dir:="+str(directory/"logs")],
            stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            deadline=time.monotonic()+25
            while time.monotonic()<deadline:
                assert process.poll() is None,"normal logging bringup exited"
                for csv_path in (directory/"logs").glob("*/flight.csv"):
                    with csv_path.open(newline="") as stream:rows=list(csv.DictReader(stream))
                    if any(row.get("robot_state") and row.get("hardware_tx")=="OFF" for row in rows):
                        session=csv_path.parent;break
                if session:break
                time.sleep(.2)
            assert session,"control/serial topics not recorded across processes"
        except Exception:
            print(path.read_text());raise
        finally:
            if process.poll() is None:process.send_signal(signal.SIGINT)
            try:process.wait(timeout=10)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
    summary=json.loads((session/"summary.json").read_text())
    assert summary["clean_close"] and summary["csv_rows"]>0,summary
    metadata=json.loads((session/"metadata.json").read_text())
    assert metadata["run_mode"]=="hardware" and metadata["hardware_dry_run"] is True,metadata
    print("PASS: separate logger records real dry-run control/hardware topics; launch SIGINT saves final summary")
