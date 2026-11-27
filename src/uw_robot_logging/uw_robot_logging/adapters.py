"""ROS message fields -> plain scalar dicts. Units follow uw_robot_interfaces."""
import math
from .schema import AXES, FORCES, euler_degrees, number


def stamp_seconds(message):
    if not hasattr(message, "header"): return None
    stamp = message.header.stamp
    return stamp.sec + stamp.nanosec*1e-9 if stamp.sec or stamp.nanosec else None


def vector(prefix, values, names):
    return {prefix+key: number(value) for key, value in zip(names, values)}


def adapt(source, m):
    if source == "command":
        return {"cmd_sequence": m.sequence, **vector("cmd_", m.velocity, ("u","v","w","p","q","r")),
                "cmd_arm": m.arm, "cmd_estop": m.estop, "cmd_depth_toggle": m.depth_toggle,
                "cmd_loop_toggle": m.loop_toggle}
    if source == "robot":
        return {"armed": m.armed, "closed_loop": m.closed_loop, "depth_hold": m.depth_hold,
                "depth_target_m": number(m.depth_target_m), "robot_imu_valid": m.imu_valid,
                "robot_depth_valid": m.depth_valid, "cycle_dt_s": number(m.cycle_dt_s),
                "robot_state": m.state, **vector("robot_target_", m.target_wrench, AXES)}
    if source == "imu":
        q, g, a = m.orientation, m.angular_velocity, m.linear_acceleration
        angles = euler_degrees(q.w,q.x,q.y,q.z)
        return {**vector("imu_q", (q.w,q.x,q.y,q.z), "wxyz"),
                **dict(zip(("imu_roll_deg","imu_pitch_deg","imu_yaw_deg"), angles)),
                **dict(zip(("imu_gyro_"+k+"_rad_s" for k in "xyz"), map(number,(g.x,g.y,g.z)))),
                **dict(zip(("imu_accel_"+k+"_m_s2" for k in "xyz"), map(number,(a.x,a.y,a.z))))}
    if source == "depth":
        return {"depth_m": number(m.depth_m), "depth_rate_mps": number(m.depth_rate_mps),
                "depth_valid": m.valid, "depth_data_age_s": number(m.data_age_s)}
    if source == "wrench":
        return {"wrench_sequence": m.sequence, "wrench_session": m.session, "wrench_epoch": m.epoch,
                "wrench_enabled": m.enabled, "wrench_closed_loop": m.closed_loop,
                "center_servos": m.center_servos, "recenter_servos": m.recenter_servos,
                **vector("target_", m.wrench, AXES)}
    if source == "allocation":
        result = {"planner_valid": m.planner_valid, "planner_ms": number(m.planner_ms), "allocation_state": m.state,
                  **vector("allocation_target_", m.target_wrench, AXES),
                  **vector("achievable_", m.achievable_wrench, AXES),
                  **vector("achieved_model_", m.achieved_wrench, AXES),
                  **dict(zip(("allocated_force_"+k+"_n" for k in FORCES), map(number,m.thruster_forces))),
                  **dict(zip(("servo_target_13_deg","servo_target_24_deg"), [number(math.degrees(v)) for v in m.servo_target_rad])),
                  **dict(zip(("servo_estimated_13_deg","servo_estimated_24_deg"), [number(math.degrees(v)) for v in m.servo_estimated_rad])),
                  "gate_13": number(m.gate[0]), "gate_24": number(m.gate[1])}
        for suffix, indices in (("force_error_model_n",range(3)),("moment_error_model_nm",range(3,6))):
            differences = [number(m.achievable_wrench[i]-m.achieved_wrench[i]) for i in indices]
            result["allocation_"+suffix] = number(math.hypot(*differences)) if all(v is not None for v in differences) else None
        return result
    if source == "actuator":
        return {"actuator_sequence": m.sequence, "actuator_enabled": m.enabled,
                **dict(zip(("force_"+k+"_n" for k in FORCES),map(number,m.thrust_newtons))),
                "servo_command_13_deg": number(math.degrees(m.theta12_rad)),
                "servo_command_24_deg": number(math.degrees(m.theta34_rad)), "gripper_closed": m.gripper_closed}
    if source == "cpu": return {"cpu_temperature_c": number(m.temperature)}
    if source == "tx":
        raw = m.data.strip()[:2048]
        values = [None]*7
        kind = "OFF" if raw == "OFF" or raw.startswith("OFF:") else ("CTRL" if raw.startswith("CTRL:") else ("SYNC" if raw.startswith("SYNC:") else "OTHER"))
        if kind == "CTRL":
            parts = raw[5:].split(",")
            if len(parts) == 7: values = list(map(number,parts))
        if kind == "OFF" and raw.startswith("OFF:"):
            camera=number(raw[4:])
            if camera is not None and -1 <= camera <= 1: values[6]=camera
        return {"hardware_tx": raw, "serial_command": kind,
                "tx_sync_token": number(raw[5:]) if kind=="SYNC" else None,
                **dict(zip(("ctrl_"+k for k in ("f1","f2","f3","f4","s1","s2","gripper")),values))}
    if source == "rx": return {"hardware_rx": m.data.strip()[:2048]}
    raise ValueError("unknown source: "+source)
