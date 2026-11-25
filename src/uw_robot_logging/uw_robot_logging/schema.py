"""CSV schema and latest-message snapshots. No ROS, devices or file IO."""
import math
from datetime import datetime, timezone

AXES = ("fx", "fy", "fz", "mx", "my", "mz")
FORCES = ("up_1", "down_3", "right_2", "left_4")
SOURCES = {
    "command": ("control/cmd", .5), "robot": ("robot/status", .5),
    "imu": ("imu/data", .2), "depth": ("depth/state", .5),
    "wrench": ("control/wrench", .5), "allocation": ("allocation/status", .5),
    "actuator": ("actuators/command", .5), "cpu": ("system/cpu_temperature", 3.),
    "tx": ("hardware/tx", .5), "rx": ("hardware/rx", 1.5),
}
FIELDS = {
    "command": ["cmd_sequence"] + ["cmd_"+k for k in ("u", "v", "w", "p", "q", "r")] +
               ["cmd_"+k for k in ("arm", "estop", "depth_toggle", "loop_toggle")],
    "robot": ["armed", "closed_loop", "depth_hold", "depth_target_m", "robot_imu_valid",
              "robot_depth_valid", "cycle_dt_s", "robot_state"] + ["robot_target_"+a for a in AXES],
    "imu": ["imu_q"+a for a in ("w", "x", "y", "z")] +
           ["imu_"+a+"_deg" for a in ("roll", "pitch", "yaw")] +
           ["imu_gyro_"+a+"_rad_s" for a in "xyz"] + ["imu_accel_"+a+"_m_s2" for a in "xyz"],
    "depth": ["depth_m", "depth_rate_mps", "depth_valid", "depth_data_age_s"],
    "wrench": ["wrench_sequence", "wrench_session", "wrench_epoch", "wrench_enabled",
               "wrench_closed_loop", "center_servos", "recenter_servos"] + ["target_"+a for a in AXES],
    "allocation": ["planner_valid", "planner_ms", "allocation_state"] +
                  [prefix+a for prefix in ("allocation_target_", "achievable_", "achieved_model_") for a in AXES] +
                  ["allocated_force_"+k+"_n" for k in FORCES] +
                  ["servo_"+stage+"_"+group+"_deg" for stage in ("target", "estimated") for group in ("13", "24")] +
                  ["gate_13", "gate_24", "allocation_force_error_model_n", "allocation_moment_error_model_nm"],
    "actuator": ["actuator_sequence", "actuator_enabled"] + ["force_"+k+"_n" for k in FORCES] +
                ["servo_command_13_deg", "servo_command_24_deg", "gripper_closed"],
    "cpu": ["cpu_temperature_c"],
    "tx": ["hardware_tx", "serial_command"] + ["ctrl_"+k for k in ("f1", "f2", "f3", "f4", "s1", "s2", "gripper")],
    "rx": ["hardware_rx"],
}
MCU_KEYS = {"VOLT": "mcu_voltage_v", **{"CH"+str(i): "rpm_"+str(i) for i in range(4)},
            "MCU_SEQ": "mcu_sequence", "MCU_MS": "mcu_uptime_ms", "CTRL_ACTIVE": "mcu_control_active",
            "NEUTRAL_REQUIRED": "mcu_neutral_required", "CH_READY": "mcu_ch_ready",
            "STOP_REASON": "mcu_stop_reason", "RX_REJECT": "mcu_rx_reject", "SPI_ERRORS": "mcu_spi_errors"}
for key in ("VOLT", "CH0", "CH1", "CH2", "CH3"):
    MCU_KEYS[key+"_VALID"] = ("mcu_voltage" if key == "VOLT" else "rpm_"+key[2])+"_reported_valid"
    MCU_KEYS[key+"_AGE_MS"] = ("mcu_voltage" if key == "VOLT" else "rpm_"+key[2])+"_reported_age_ms"


def number(value):
    try:
        n = float(value)
        return n if math.isfinite(n) else None
    except (ValueError, TypeError, OverflowError):
        return None


def euler_degrees(w, x, y, z):
    q = [number(v) for v in (w, x, y, z)]
    if any(v is None for v in q): return (None,)*3
    norm = math.sqrt(sum(v*v for v in q))
    if not math.isfinite(norm) or norm < 1e-6: return (None,)*3
    w, x, y, z = [v/norm for v in q]
    return (math.degrees(math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))),
            math.degrees(math.asin(max(-1., min(1., 2*(w*y-z*x))))),
            math.degrees(math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))))


def header():
    cols = ["wall_time_utc", "elapsed_s", "ros_time_s", "log_queue_depth", "log_dropped_csv", "log_dropped_events"]
    for source in SOURCES:
        cols += [source+suffix for suffix in ("_receive_age_s", "_stamp_s", "_source_age_s", "_fresh")]
        cols += FIELDS[source]
    cols += ["depth_log_usable", "depth_error_m", "cpu_log_usable"]
    for key, column in MCU_KEYS.items():
        cols += [column, column+"_receive_age_s"]
    assert len(cols) == len(set(cols))
    return cols


class Snapshot:
    def __init__(self):
        self.latest = {}
        self.mcu = {}

    def observe(self, source, data, received, stamp=None):
        self.latest[source] = (dict(data), received, stamp)
        if source == "rx":
            tokens = {}
            for token in data.get("hardware_rx", "").split():
                if ":" not in token: continue
                key, value = token.split(":", 1)
                if key not in MCU_KEYS: continue
                tokens[key] = None if key in tokens else number(value)
            for key, value in tokens.items():
                self.mcu[key] = (value, received)

    def sample(self, elapsed, now, ros_time, wall_time=None, queue_depth=0, dropped_csv=0, dropped_events=0):
        row = dict.fromkeys(header())
        row.update(wall_time_utc=wall_time or datetime.now(timezone.utc).isoformat(), elapsed_s=elapsed,
                   ros_time_s=ros_time, log_queue_depth=queue_depth,
                   log_dropped_csv=dropped_csv, log_dropped_events=dropped_events)
        for source, (_, timeout) in SOURCES.items():
            row[source+"_fresh"] = False
            if source not in self.latest: continue
            data, received, stamp = self.latest[source]
            age = now-received
            row[source+"_receive_age_s"] = age
            row[source+"_stamp_s"] = stamp
            source_age = ros_time-stamp if stamp is not None else None
            row[source+"_source_age_s"] = source_age
            row[source+"_fresh"] = 0 <= age <= timeout and (source_age is None or -.05 <= source_age <= timeout)
            row.update(data)
        depth_age = row.get("depth_data_age_s")
        row["depth_log_usable"] = bool(row["depth_fresh"] and row["depth_valid"] and row["depth_m"] is not None and
            depth_age is not None and 0 <= depth_age + (row["depth_receive_age_s"] or 0) <= SOURCES["depth"][1])
        row["cpu_log_usable"] = bool(row["cpu_fresh"] and row["cpu_temperature_c"] is not None)
        if row["depth_log_usable"] and row["robot_fresh"] and row["depth_hold"] and row["depth_target_m"] is not None:
            row["depth_error_m"] = row["depth_target_m"] - row["depth_m"]
        for key, (value, received) in self.mcu.items():
            row[MCU_KEYS[key]] = value
            row[MCU_KEYS[key]+"_receive_age_s"] = now-received
        return row
