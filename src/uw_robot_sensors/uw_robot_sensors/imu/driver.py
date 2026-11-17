# -*- coding: utf-8 -*-
"""YbImu SDK 采集、上电零点和线程安全快照；ROS 发布位于 imu_node。"""
import math
import time
import threading

YBUART_PORT = '/dev/ttyUSB0'
IMU_ALGORITHM = 6
IMU_OUTPUT_RATE = 100
IMU_READ_INTERVAL = 0.01
IMU_STALE_TIMEOUT = 0.1
IMU_ZERO_CALIBRATION_TIME = 2.0


try:
    from YbImuLib import YbImuSerial
    YB_IMU_AVAILABLE = True
except ImportError:
    YbImuSerial = None
    YB_IMU_AVAILABLE = False

imu_state = {
    "quaternion": (1.0, 0.0, 0.0, 0.0),   # [w,x,y,z]
    "gyro": (0.0, 0.0, 0.0),              # rad/s
    "acceleration": (0.0, 0.0, 0.0),       # g
    "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
    "timestamp": 0.0, "ok": False, "zeroed": False,
}
imu_lock = threading.Lock()


def _normalize_quaternion(q):
    norm = math.sqrt(sum(value*value for value in q))
    if norm <= 1e-9:
        return (1.0, 0.0, 0.0, 0.0)
    return tuple(value/norm for value in q)


def _quaternion_multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw*bw-ax*bx-ay*by-az*bz,
            aw*bx+ax*bw+ay*bz-az*by,
            aw*by-ax*bz+ay*bw+az*bx,
            aw*bz+ax*by-ay*bx+az*bw)


def _relative_quaternion(reference, current):
    """令上电水平姿态为单位四元数；四元数顺序均为[w,x,y,z]。"""
    conjugate = (reference[0], -reference[1], -reference[2], -reference[3])
    return _normalize_quaternion(_quaternion_multiply(conjugate, current))


def _quaternion_to_euler_deg(q):
    w, x, y, z = q
    roll = math.atan2(2.0*(w*x+y*z), 1.0-2.0*(x*x+y*y))
    pitch_term = max(-1.0, min(1.0, 2.0*(w*y-z*x)))
    pitch = math.asin(pitch_term)
    yaw = math.atan2(2.0*(w*z+x*y), 1.0-2.0*(y*y+z*z))
    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


def get_imu_state():
    """线程安全快照；控制循环应检查ok，不能直接使用超时数据。"""
    with imu_lock:
        state = imu_state.copy()
    state["age"] = time.monotonic()-state["timestamp"]
    state["ok"] = bool(state["ok"] and state["age"] <= IMU_STALE_TIMEOUT)
    return state


def _open_imu():
    """连接 SDK 并追踪真实收帧时间；getter 缓存不能作为链路存活依据。"""
    imu = YbImuSerial(YBUART_PORT, debug=False)
    # 驱动getter只返回缓存，无法判断USB串口是否仍在更新。挂接解析
    # 回调，使用真正收到有效IMU帧的时刻做失联检测。
    frame_time = {"raw": 0.0, "quaternion": 0.0, "euler": 0.0}
    original_parse = imu._parse_data

    def tracked_parse(ext_type, ext_data):
        original_parse(ext_type, ext_data)
        now = time.monotonic()
        if ext_type == imu.FUNC_REPORT_IMU_RAW:
            frame_time["raw"] = now
        elif ext_type == imu.FUNC_REPORT_IMU_QUAT:
            frame_time["quaternion"] = now
        elif ext_type == imu.FUNC_REPORT_IMU_EULER:
            frame_time["euler"] = now

    imu._parse_data = tracked_parse
    imu.create_receive_threading()
    imu.set_report_rate(IMU_OUTPUT_RATE)
    imu.set_algo_type(IMU_ALGORITHM)
    print(f"[IMU] 新 IMU 已连接: {YBUART_PORT}")
    print(f"[IMU] 请保持机器人水平静止 {IMU_ZERO_CALIBRATION_TIME:.1f}s，正在建立姿态零点")
    return imu, frame_time


def imu_reader(stop_event):
    """读取四元数、角速度、加速度和欧拉角。"""
    if not YB_IMU_AVAILABLE:
        print("[IMU] 未安装 YbImuLib, 姿态回传禁用")
        return
    try:
        imu, frame_time = _open_imu()
    except Exception as e:
        print(f"[IMU] 初始化失败: {e}")
        return
    # 上电零点只建立一次；它与控制器每次解锁/松杆捕获的目标姿态不同。
    reference = None
    gyro_bias = (0.0, 0.0, 0.0)
    calibration_start = 0.0
    calibration_samples = []
    gyro_samples = []
    while not stop_event.is_set():
        try:
            roll, pitch, yaw = imu.get_imu_attitude_data(ToAngle=True)
            quaternion = tuple(imu.get_imu_quaternion_data())
            gyro = tuple(imu.get_gyroscope_data())
            acceleration = tuple(imu.get_accelerometer_data())
            q_norm = math.sqrt(sum(value*value for value in quaternion))
            finite = all(math.isfinite(value) for value in
                         quaternion+gyro+acceleration+(roll, pitch, yaw))
            now = time.monotonic()
            raw_time = frame_time["raw"]
            quaternion_time = frame_time["quaternion"]
            sample_time = min(raw_time, quaternion_time)
            # 原始帧和四元数帧必须都新鲜，且数据有限、四元数接近单位长度。
            valid = (finite and 0.9 <= q_norm <= 1.1
                     and raw_time > 0.0 and quaternion_time > 0.0
                     and now-raw_time <= IMU_STALE_TIMEOUT
                     and now-quaternion_time <= IMU_STALE_TIMEOUT)
            if valid:
                quaternion = tuple(value/q_norm for value in quaternion)
                if reference is None:
                    if calibration_start <= 0.0:
                        calibration_start = now
                    # q和-q表示同一姿态，平均前先统一半球，避免相互抵消。
                    if calibration_samples and sum(a*b for a, b in zip(
                            calibration_samples[0], quaternion)) < 0.0:
                        quaternion = tuple(-value for value in quaternion)
                    calibration_samples.append(quaternion)
                    gyro_samples.append(tuple(float(value) for value in gyro))
                    if (now-calibration_start >= IMU_ZERO_CALIBRATION_TIME
                            and len(calibration_samples) >= 10):
                        reference = _normalize_quaternion(tuple(
                            sum(q[i] for q in calibration_samples)
                            for i in range(4)))
                        gyro_bias = tuple(sum(sample[i] for sample in gyro_samples)
                                          / len(gyro_samples) for i in range(3))
                        print("[IMU] 姿态零点建立完成")
                if reference is not None:
                    quaternion = _relative_quaternion(reference, quaternion)
                    gyro = tuple(float(value) - gyro_bias[i]
                                 for i, value in enumerate(gyro))
                    roll, pitch, yaw = _quaternion_to_euler_deg(quaternion)
                else:
                    valid = False
            # 零点尚未建立时，坏数据会重新开始连续校准；建立后保留原零点。
            elif reference is None:
                calibration_start = 0.0
                calibration_samples = []
                gyro_samples = []
            with imu_lock:
                imu_state.update(
                    quaternion=quaternion, gyro=gyro,
                    acceleration=acceleration,
                    roll=roll, pitch=pitch, yaw=yaw,
                    timestamp=sample_time, ok=valid,
                    zeroed=reference is not None)
        except Exception as exc:
            with imu_lock:
                imu_state["ok"] = False
            print(f"[IMU] 读取异常: {exc}")
            time.sleep(0.1)
        time.sleep(IMU_READ_INTERVAL)

