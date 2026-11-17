# -*- coding: utf-8 -*-
"""实物 AUV 的 50 Hz 开环/闭环控制与动态推力分配。

控制链：
    手柄输入 -> 机体系目标合力
    -> 姿态外环（四元数误差 -> 目标角速度）
    -> 角速度内环（角速度误差 -> 目标力矩）
    -> 六维目标 [Fx,Fy,Fz,Mx,My,Mz]
    -> 两组舵角和四台推进器推力 -> STM32 归一化 CTRL 指令

坐标系约定：机体系 +X 前进；四元数为 [w,x,y,z]；角速度单位 rad/s。
Y、Z 正方向必须在首次实机测试时与 IMU 和推进器逐轴核对。

调参：只改下面的"参数区"，顺序与"症状→参数"速查见 FinsROV_调参指南.md。
"""
import math
import time
import threading

import numpy as np

from thruster_solver import (
    MuJoCoAllocator,
    THRUSTER_FORWARD_LIMIT,
    THRUSTER_REVERSE_LIMIT,
)


# ===========================================================================
# 参数区（调参只改这段）
#   单位：力 N、力矩 N·m、角速度 rad/s（标 °/s 的除外）、角度 rad、时间 s
#   铁律：一次只改一档、改完上水测；别动已经正常的轴。
# ===========================================================================

# ---- 控制周期 ----
CONTROL_DT = 0.02  # 标称 50Hz；实际周期用 update() 里测到的 dt
# 实测 dt 限幅：调度被一次长暂停（相机、日志刷盘）打断时，不能让积分和
# 限幅步长按几十毫秒去走，否则一次抖动等于一次大跳变。
DT_MIN = 0.005
DT_MAX = 0.050

# ---- 手柄平移力上限 [Fx 前后, Fy 横移, Fz 升沉]，N ----
# 闭环用 MAX_BODY_FORCE；开环另有一套（无反馈，量程更大，纯靠手感）。
MAX_BODY_FORCE = np.array([20.0, 6.0, 10.0])        # 闭环：前后 20 / 横移 6 / 升沉 10
MAX_OPEN_LOOP_FORCE = np.array([60.0, 20.0, 30.0])  # 开环：前后 60 / 横移 20 / 升沉 30

# ---- 安全限幅：六维目标最大变化率 [Fx,Fy,Fz, Mx,My,Mz]，力 N/s、力矩 N·m/s ----
# 任何来源的目标跳变（舵角分支跳、IMU 尖峰、门控、fast_center）都先经此限幅，
# 保证 1~2 拍内不会变成"猛冲"。力 150、力矩 8 → 约 0.4s 到满。
WRENCH_RATE_LIMIT = np.array([150.0, 150.0, 150.0, 8.0, 8.0, 8.0])

# ---- 手柄旋转最大角速度 [roll,pitch,yaw]，°/s ----
# 右摇杆/肩键 → 目标机体角速度的上限（积分成目标姿态，所以按住=连续转）。
MAX_MANUAL_RATE = np.radians([25.0, 22.0, 20.0])

# ---- 串级姿态环：姿态误差 → 目标角速度(外环) → 机体系力矩(内环) ----
# 增益必须"慢"到和执行器的大延迟匹配（舵机转 90° 约需 1s）。原值
# [2.5,3.2,3.2]/[1.0,3.0,3.0] 比执行器快约 3 倍，水下振荡发散。
#
# 外环：姿态误差 × ATTITUDE_KP = 目标角速度。
#   调大→回正快、易振荡；调小→更稳、允许偏差大。
ATTITUDE_KP = np.array([0.8, 1.2, 1.2])          # 滚转 1.2→0.8（降滚转纠正力度、少往别的轴漏耦合）
# 外环输入死区 [roll,pitch,yaw]，rad：误差小于它 → 0。
#   调大→水里更"安静"、不纠静态小倾斜；调小→纠得更精、易抖。
ATTITUDE_DEADZONE = np.radians([8.0, 3.0, 5.0])   # 滚转 8；俯仰 5→3；偏航 5
#
# 内环：角速度误差 × RATE_KP = 力矩（阻尼主力）。
#   调大→刹得住，但过高会自激振荡；调小→刹不住、易过冲。
RATE_KP = np.array([1.0, 1.5, 1.5])               # 滚转曾提到 1.8 → 自激振荡(限制环)，回退 1.0
RATE_KI = np.array([0.06, 0.20, 0.20])            # 内环积分（消稳态角速度差）；调大消差快、易振
RATE_INTEGRAL_LIMIT = 0.8                         # 积分限幅，防饱和
MAX_INNER_RATE = np.radians([90.0, 90.0, 60.0])   # 内环目标角速度上限 [roll,pitch,yaw]

# ---- 手柄旋转"前馈"力矩增益 [roll,pitch,yaw] ----
#   目标力矩里 + MANUAL_MOMENT_KP × 手柄角速度。与反馈用的 RATE_KP 分开，只调手感、
#   不影响反馈整定。调大→按 RB 更有劲、易冲过头；调小→更软、易"没劲"。
MANUAL_MOMENT_KP = np.array([1.3, 1.5, 1.5])      # 滚转试过 4.5(过头)/2.5，最终 1.3
# 判断"松开旋转输入"的摇杆死区（归一化，0~1）：低于它算 0。
#   防止摇杆噪声让"松手重锁目标姿态"永不触发。
MANUAL_INPUT_DEADZONE = 0.05

# ---- 力矩总上限 [Mx 滚转, My 俯仰, Mz 偏航]，N·m ----
#   含反馈+前馈，最终 np.clip 到这里。调大→纠偏/失控都更猛；调小→全变柔、纠偏差。
MAX_BODY_MOMENT = np.array([3, 5, 5])         # 滚转 3.5→2.5，限住"失控转圈"
MAX_OPEN_LOOP_MOMENT = np.array([5.0, 5.0, 5.0])    # 开环（无反馈，限制更保守，防陆地测试大风）

# ---- 偏航模式开关 ----
#   True  = 正常航向保持（默认；六轴实测 yaw 基本不漂）
#   False = 只做角速度阻尼、不锁航向（万一水下 yaw 漂移大就改回）
YAW_HOLD_HEADING = True

# ---- 滚转姿态保持开关 ----
#   True  = 正常（姿态误差 → 目标角速度 → 力矩）
#   False = 只做角速度阻尼、不纠滚转角度：松杆后不自带回正力，
#           但也不会一直和干扰较劲。横移带滚转排查时用这个。
ROLL_HOLD_ATTITUDE = True

# ---- 机体系与力矩正方向（右手定则）----
# 机体轴：+X 前进、+Y 左舷、+Z 向上（右手系，同 ROS REP-103）。
# 力矩一律按右手定则定义：
#   +Mx = 左舷上抬（向右横滚）
#   +My = 低头（机头下俯）
#   +Mz = 逆时针（机头向左，俯视）
# 姿态反馈与手柄前馈都在这一套约定下计算，两者符号天然一致。
#
# PLANT_MOMENT_SIGN：实物推进器实际输出的机体力矩相对该模型"整体反号"时取 -1。
# 这是"物理通道"的符号——反馈与手柄前馈共用同一条力矩通道，必须在两者合并的
# 唯一入口（update() 里的 target_wrench）一起反号；力 Fx/Fy/Fz 不乘此符号。
# 三轴实测均反号 -> [-1,-1,-1]。
#
# ===== 开环逐轴标定（换机/换线后做一次）=====
#   1. 地面固定好机器人，按 Y 切到开环（不读 IMU），A 解锁。
#   2. 每次只打一个输入，对照下面的期望转向：
#        右摇杆 左推 -> 逆时针          (Mz)
#        右摇杆 上推 -> 低头            (My)
#        RB          -> 向右横滚        (Mx)
#   3. 若某轴实物转向相反，只把 PLANT_MOMENT_SIGN 对应分量取反即可，
#      不要改 _manual_angular_rate（那里的映射已是空间真值）。
#   4. 三轴各自标完后，反馈会自动同向、重新变为负反馈，无需再单独调反馈符号。
PLANT_MOMENT_SIGN = np.array([-1.0, -1.0, -1.0])

# ---- 定深环（X 键切换）。深度读数约定为向下增大 ----
DEPTH_KP = 20.0
DEPTH_KI = 2.0
DEPTH_KD = 2.0
DEPTH_INTEGRAL_LIMIT = 1.0
MAX_DEPTH_FORCE = 8.0             # N，定深输出的最大竖直力
MAX_DEPTH_CHANGE_RATE = 0.25      # m/s，按住 LT/RT 时移动深度目标的速率


# ===========================================================================
# 执行器与分配（一般不用动）
# ===========================================================================

# ---- 舵机（实测空载 360°/1.6s ≈ 225°/s，带载取 80%）----
SERVO_LIMIT = math.pi/2.0                  # 舵角上下限 ±90°
# 命令速率：发出去的位置指令按这个速率爬升。原来取 90°/s，是按"保守估计"
# 填的，实测空载 225°/s——等于我们自己把舵机限速了 2.5 倍，转 90° 要 1 秒。
# 取 180°/s：留 20% 余量给水阻和舵面负载。
SERVO_COMMAND_RATE = math.radians(180.0)
SERVO_STEP = SERVO_COMMAND_RATE*CONTROL_DT
SERVO_RETURN_RATE = math.radians(180.0)    # 松手回中同样按这个速率
SERVO_RETURN_STEP = SERVO_RETURN_RATE*CONTROL_DT
# 舵角目标直接=解（不再低通）。低通会让指令被每 0.5s 的重解反复推、走走停停；
# 直接给到目标，舵机就做一次干净匀速转动，平滑由舵机自身 90°/s 斜率保证。
SERVO_TARGET_BLEND = 1.0
SERVO_TARGET_SNAP = math.radians(2.0)      # 接近目标就一次吸附到位，防微动/卡顿
SERVO_TARGET_DEADBAND = math.radians(1.5)

# ---- 重规划与开环发力门控 ----
FULL_ALLOCATION_PERIOD = 0.5               # s，每 0.5s 重解一次舵角（求解放后台线程，不阻塞控制环）
# 后台舵角计划的有效期与"目标变了多少就算过期"。
# 求解本身可能比 0.5s 还慢，所以不能只看编号；这里按目标差判，留足余量。
SOLVER_RESULT_MAX_AGE_S = 1.0
SOLVER_TARGET_DELTA_SCALE = np.array([10., 10., 10., 2., 2., 2.])
SOLVER_TARGET_DELTA_MAX = 1.5
GATE_MARGIN = math.radians(15.0)           # 门控"到位余量"：估计舵角离目标 15° 内才发力到满
GATE_HOLD_S = 1.0                          # 开环新动作后强制关门时长（s），够舵机转到新位置

# ---- 推力 ----
THRUST_RATE_LIMIT = 10.0                   # N/s，单台推力变化率上限
# 零舵角附近基本无法产生 Mx；低于此值才允许闭环松杆快速回中。
ROLL_MOMENT_CENTER_THRESHOLD = 0.35        # N·m


def normalize_quaternion(quaternion):
    """返回单位四元数；无效零向量回退到单位姿态。"""
    value = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(value)
    return value/norm if norm > 1e-9 else np.array([1.0, 0.0, 0.0, 0.0])


def quaternion_multiply(left, right):
    """Hamilton 积，输入输出顺序均为 [w,x,y,z]。"""
    w1, x1, y1, z1 = left
    w2, x2, y2, z2 = right
    return np.array([
        w1*w2-x1*x2-y1*y2-z1*z2,
        w1*x2+x1*w2+y1*z2-z1*y2,
        w1*y2-x1*z2+y1*w2+z1*x2,
        w1*z2+x1*y2-y1*x2+z1*w2,
    ])


def quaternion_to_rotation(quaternion):
    """四元数转换为“机体系向量 -> 世界系向量”的旋转矩阵。"""
    w, x, y, z = normalize_quaternion(quaternion)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


class HardwareController:
    """把手柄、IMU 和深度状态转换为 STM32 的 CTRL 指令。

    两种模式（按 Y 切换，未解锁时）：
      闭环 closed_loop=True ：读 IMU，跑姿态/定深闭环，输出六维目标；
      开环 closed_loop=False：六维目标直接由手柄生成，不读 IMU/深度。
    按键（rising edge）：B=锁定停机，A=解锁/上锁，X=定深，Y=切换闭环/开环。
    """

    def __init__(self):
        self.allocator = MuJoCoAllocator(None)
        # 后台线程专用：不和控制线程共用 last_x，避免停机清零时正好撞上
        # 求解入口的 self.last_x.copy()。
        self._full_allocator = MuJoCoAllocator(None)
        self.armed = False
        # True=姿态/定深闭环；False=六维手柄指令直接解算，不依赖 IMU。
        self.closed_loop = True
        self.depth_hold = False
        self.gripper_closed = False
        self.last_button = {key: False for key in ("a", "b", "x", "y")}

        # 姿态/积分状态
        self.target_attitude = np.array([1.0, 0.0, 0.0, 0.0])  # 目标姿态（松手时会重锁到当前）
        self.rate_integral = np.zeros(3)      # 角速度内环积分
        self.depth_integral = 0.0
        self.depth_target = 0.0
        self.depth_was_valid = False

        # 舵角三级状态：solver=解算器给的方向；desired=最终命令；estimated=按额定舵速
        # 推算的"当前真实角度"（实物无舵角反馈，用它做分配和输出）。
        self.desired_servo_angles = np.zeros(2)
        self.solver_servo_angles = np.zeros(2)
        self.estimated_servo_angles = np.zeros(2)
        # 分配器内部顺序：[1上,3下,2右,4左]，单位 N。
        self.filtered_thruster_forces = np.zeros(4)
        self.last_full_allocation_time = 0.0
        self.was_vertical_active = False
        self.was_angular_active = False       # 上一拍是否有旋转输入（用于松手重锁）
        self.filtered_gyro = np.zeros(3)

        # 后台选解线程：solve_full 单次耗时可达 150~300ms，绝不能放在 50Hz
        # 控制循环里同步执行，否则会卡出"转到一半突然回去再转"的极限环。
        # 控制循环只投递请求、非阻塞取回最近结果。
        self._solver_lock = threading.Lock()
        self._solve_request = None      # (target_wrench, reference)
        self._solve_result = None       # 最近一次解出的 [θ12, θ34]
        self._solved = False            # 是否已拿到第一个解（用于开环发力门控）
        self._prev_servo_angles = np.zeros(2)   # 上一拍舵角，用于算角速度
        self._prev_target_wrench = np.zeros(6)  # 上一拍目标力/矩，用于检测新动作
        self._limited_wrench = np.zeros(6)      # 限幅后的六维目标（上一拍），用于变化率限幅
        self._gate_hold_until = 0.0     # 开环：新动作后保持关门的截止时刻
        self._gate = np.ones(2)         # 发力门控（低通后）
        self._alloc_debug_time = 0.0    # 闭环调试打印节流
        # 诊断量（只被日志读取，不参与控制）
        self._last_update_time = None
        self._dt = CONTROL_DT
        self.last_cycle_dt_s = CONTROL_DT
        self.last_cycle_work_ms = 0.0
        self.last_controller_update_ms = 0.0
        self.last_fixed_solver_ms = 0.0
        self.last_target_wrench = np.zeros(6)
        self.last_wrench_raw = np.zeros(6)
        self.last_wrench_for_allocator = np.zeros(6)
        self.last_achieved_wrench = np.zeros(6)
        self.last_moment_unclipped = np.zeros(3)
        self.last_moment_clipped = np.zeros(3)
        self.last_moment_limit_ratio = np.zeros(3)
        self.last_feedback_moment = np.zeros(3)
        self.last_manual_moment = np.zeros(3)
        self.last_force_error = math.inf
        self.last_moment_error = math.inf
        self.last_solver_result_age_ms = math.inf
        self.last_allocated_forces = np.zeros(4)
        self.last_allocation_saturated = False
        self.last_allocation_near_saturation = False
        self.last_solver_ok = False
        self.last_allocation_achievable = False
        # 定深环诊断
        self.last_depth_target = 0.0
        self.last_depth_error = 0.0
        self.last_world_vertical_force = 0.0
        # 耗时诊断
        self.last_full_solver_ms = 0.0
        self.last_log_write_ms = 0.0
        threading.Thread(target=self._solver_worker, daemon=True).start()

    def _solver_worker(self):
        """后台线程：有请求就跑一次全局选解，把舵角写入 _solve_result。"""
        while True:
            with self._solver_lock:
                request = self._solve_request
                self._solve_request = None
            if request is None:
                time.sleep(0.005)
                continue
            target, reference = request
            started = time.monotonic()
            try:
                self._full_allocator.last_x = np.asarray(reference, dtype=float)
                allocation = self._full_allocator.solve_full(target)
                angles = np.array([allocation.theta12, allocation.theta34],
                                  dtype=float)
                if not np.all(np.isfinite(angles)):
                    raise ValueError("non-finite servo angles")
                with self._solver_lock:
                    self.last_full_solver_ms = (time.monotonic()-started)*1000.0
                    self._solve_result = (angles, started,
                                          np.asarray(target, dtype=float).copy())
            except Exception as exc:
                self.last_full_solver_ms = (time.monotonic()-started)*1000.0
                print(f"[分配] 求解异常: {exc}")

    def _invalidate_solver(self):
        """作废在途的舵角计划：停机/切模式后，旧结果不能再被取走。"""
        with self._solver_lock:
            self._solve_request = None
            self._solve_result = None
            self._solved = False

    def _rising_edge(self, joystick, key):
        """按键上升沿检测（当前按下且上一拍未按下）。"""
        current = bool(joystick.get(key, False))
        rising = current and not self.last_button[key]
        self.last_button[key] = current
        return rising

    def stop(self):
        """锁定并返回停机命令；STM32 收到 OFF 后应输出 1500 us。"""
        self.armed = False
        self.depth_hold = False
        self.rate_integral[:] = 0.0
        self.filtered_thruster_forces[:] = 0.0
        self._limited_wrench[:] = 0.0
        # STM32 收到 OFF 会立即把舵机回中，估计角度也必须同步回中。
        self.desired_servo_angles[:] = 0.0
        self.solver_servo_angles[:] = 0.0
        self.estimated_servo_angles[:] = 0.0
        self.allocator.last_x = None
        # 后台可能还在算停机前那一份，结果到了不能再用。
        self._invalidate_solver()
        self.was_angular_active = False
        self.filtered_gyro[:] = 0.0
        self.depth_integral = 0.0
        return "OFF"

    def _handle_mode_buttons(self, joystick, attitude, depth):
        """处理 B/A/X/Y 四个按键（均取上升沿）。"""
        if self._rising_edge(joystick, "b"):
            self.stop()
            return

        if self._rising_edge(joystick, "y") and not self.armed:
            # Y：未解锁时在 闭环/开环 之间切，切模式清空解算状态。
            self.closed_loop = not self.closed_loop
            self.depth_hold = False
            self.rate_integral[:] = 0.0
            self.depth_integral = 0.0
            self.desired_servo_angles[:] = 0.0
            self.solver_servo_angles[:] = 0.0
            self.estimated_servo_angles[:] = 0.0
            self.filtered_thruster_forces[:] = 0.0
            self.allocator.last_x = None
            self._limited_wrench[:] = 0.0

        if self._rising_edge(joystick, "a"):
            # A：解锁/上锁。闭环必须有有效 IMU；开环允许 IMU 未连接时解锁。
            if self.closed_loop and attitude is None:
                self.armed = False
                return
            self.armed = not self.armed
            self.depth_hold = False
            if attitude is not None:
                # 解锁瞬间以当前姿态为目标姿态。
                self.target_attitude = attitude.copy()
            self.rate_integral[:] = 0.0
            self.allocator.last_x = None
            self.filtered_thruster_forces[:] = 0.0
            self._limited_wrench[:] = 0.0

        if (self._rising_edge(joystick, "x") and self.closed_loop and self.armed
                and depth.get("valid", False)):
            # X：切换定深；以当前深度为新的深度目标。
            self.depth_hold = not self.depth_hold
            self.depth_target = float(depth["depth"])
            self.depth_integral = 0.0
            self.depth_was_valid = True

    def _manual_angular_rate(self, joystick):
        """把右摇杆/肩键映射为目标机体角速度（正方向按右手定则，单位 rad/s）。

        空间含义（固定，与实物期望一致）：
          LB / RB       -> 横滚：RB = 向右横滚(+Mx)，LB = 向左横滚(-Mx)
          右摇杆 上 / 下 -> 俯仰：上 = 低头(+My)，下 = 抬头(-My)
          右摇杆 左 / 右 -> 偏航：左 = 逆时针(+Mz)，右 = 顺时针(-Mz)
        pygame 上推、左推给负值，故 Y 与 X 都要取负才能对上上面的正方向。
        """
        roll = float(joystick.get("rb", False))-float(joystick.get("lb", False))
        pitch = -float(joystick.get("ry", 0.0))   # 上推(ry<0) -> +My 低头
        yaw = -float(joystick.get("rx", 0.0))     # 左推(rx<0) -> +Mz 逆时针
        return MAX_MANUAL_RATE*np.array([roll, pitch, yaw])

    def _manual_body_moment(self, joystick):
        """开环：手柄直接给机体系力矩 [Mx,My,Mz]，方向与闭环手柄完全一致。"""
        roll = float(joystick.get("rb", False))-float(joystick.get("lb", False))
        pitch = -float(joystick.get("ry", 0.0))
        yaw = -float(joystick.get("rx", 0.0))
        return MAX_OPEN_LOOP_MOMENT*np.array([roll, pitch, yaw])

    def _attitude_moment(self, attitude, gyro, manual_rate):
        """姿态外环 + 角速度 PI 内环，输出机体系力矩 [Mx,My,Mz]，N·m。

        外环：目标姿态(Q) 与当前姿态的误差 -> 目标角速度；
        内环：目标角速度与实测角速度的误差 -> 力矩；
        再叠加手柄前馈（MANUAL_MOMENT_KP）。全程保持原符号（右手定则），
        最终在 update() 里统一乘 PLANT_MOMENT_SIGN。
        """
        # 松手重锁：从"有旋转输入"变"无"的那一拍，把目标姿态锁到当前姿态，
        # 避免继续追逐操作期间累积但未达到的旧目标。
        angular_active = bool(np.any(
            np.abs(manual_rate) > MAX_MANUAL_RATE*MANUAL_INPUT_DEADZONE))
        if self.was_angular_active and not angular_active:
            self.target_attitude = attitude.copy()
            self.rate_integral[:] = 0.0
        self.was_angular_active = angular_active

        self.filtered_gyro += 0.25*(gyro-self.filtered_gyro)
        # 目标姿态按手柄角速度积分，所以持续按住摇杆可连续转动。
        small_rotation = normalize_quaternion(
            np.r_[1.0, 0.5*manual_rate*self._dt])
        self.target_attitude = normalize_quaternion(
            quaternion_multiply(self.target_attitude, small_rotation))

        attitude_conjugate = np.array([
            attitude[0], -attitude[1], -attitude[2], -attitude[3]])
        error_quaternion = quaternion_multiply(
            attitude_conjugate, self.target_attitude)
        # q 和 -q 表示同一姿态，固定到最短旋转分支。
        if error_quaternion[0] < 0.0:
            error_quaternion = -error_quaternion

        attitude_error = 2.0*error_quaternion[1:]
        # 姿态死区：小于阈值的误差直接置零，避免小误差一直被纠。
        attitude_error[np.abs(attitude_error) < ATTITUDE_DEADZONE] = 0.0
        # 偏航是否锁航向由 YAW_HOLD_HEADING 决定（False 时置零 yaw 误差 → 只做角速度阻尼）。
        if not YAW_HOLD_HEADING:
            attitude_error[2] = 0.0
        # 滚转同理：关掉后只保留角速度阻尼，不再追姿态角。
        if not ROLL_HOLD_ATTITUDE:
            attitude_error[0] = 0.0

        feedback_rate = np.clip(
            ATTITUDE_KP*attitude_error,
            -MAX_INNER_RATE, MAX_INNER_RATE)
        feedback_error = feedback_rate-self.filtered_gyro
        feedback_error[np.abs(feedback_error) < math.radians(0.5)] = 0.0
        self.rate_integral = np.clip(
            self.rate_integral+feedback_error*self._dt,
            -RATE_INTEGRAL_LIMIT, RATE_INTEGRAL_LIMIT)
        # 偏航只做阻尼时不用积分环：积分 ∫(−角速度) 等于"偏转了多少角"，会变相锁角度。
        if not YAW_HOLD_HEADING:
            self.rate_integral[2] = 0.0
        if not ROLL_HOLD_ATTITUDE:
            self.rate_integral[0] = 0.0

        feedback_moment = (RATE_KP*feedback_error
                           + RATE_KI*self.rate_integral)
        manual_moment = MANUAL_MOMENT_KP*manual_rate
        # 分段记录：raw 是控制器真正想要的力矩，limit_ratio 接近 1 才说明
        # MAX_BODY_MOMENT 成了瓶颈；否则调大上限没有任何作用。
        raw_moment = feedback_moment+manual_moment
        self.last_feedback_moment = feedback_moment.copy()
        self.last_manual_moment = manual_moment.copy()
        self.last_moment_unclipped = raw_moment.copy()
        self.last_moment_limit_ratio = np.abs(raw_moment)/MAX_BODY_MOMENT
        clipped = np.clip(raw_moment, -MAX_BODY_MOMENT, MAX_BODY_MOMENT)
        self.last_moment_clipped = clipped.copy()
        # 反馈与手柄前馈在此保持原符号（右手定则），统一由 update() 的
        # PLANT_MOMENT_SIGN 处理，确保二者方向一致。
        return clipped

    def _body_force(self, joystick, attitude, depth):
        """生成机体系平移力 [Fx,Fy,Fz]；定深模式下叠加世界竖直闭环力。"""
        left_x = float(joystick.get("lx", 0.0))
        left_y = float(joystick.get("ly", 0.0))
        left_trigger = max(0.0, float(joystick.get("lt", 0.0)))
        right_trigger = max(0.0, float(joystick.get("rt", 0.0)))
        vertical_input = right_trigger-left_trigger

        body_force = MAX_BODY_FORCE*np.array([-left_y, -left_x, vertical_input])
        vertical_active = abs(vertical_input) > 0.01

        depth_valid = bool(depth.get("valid", False))
        self.last_depth_target = float(self.depth_target)
        self.last_depth_error = 0.0
        self.last_world_vertical_force = 0.0
        if self.depth_hold and depth_valid:
            # 深度数据从失效恢复时，以恢复后的当前深度重新锁定；不能继续
            # 追逐中断前的旧目标，否则会突然输出较大的升沉推力。
            if not self.depth_was_valid:
                self.depth_target = float(depth["depth"])
                self.depth_integral = 0.0
            # 松开扳机时从实测深度重新锁定，避免继续追逐旧目标。
            elif self.was_vertical_active and not vertical_active:
                self.depth_target = float(depth["depth"])
                self.depth_integral = 0.0
            else:
                self.depth_target -= (
                    vertical_input*MAX_DEPTH_CHANGE_RATE*self._dt)

            depth_error = self.depth_target-float(depth["depth"])
            self.last_depth_target = float(self.depth_target)
            self.last_depth_error = float(depth_error)
            self.depth_integral = float(np.clip(
                self.depth_integral+depth_error*self._dt,
                -DEPTH_INTEGRAL_LIMIT, DEPTH_INTEGRAL_LIMIT))
            depth_rate = float(depth.get("depth_rate", 0.0))
            world_vertical_force = np.clip(
                -(DEPTH_KP*depth_error
                  + DEPTH_KI*self.depth_integral
                  - DEPTH_KD*depth_rate),
                -MAX_DEPTH_FORCE, MAX_DEPTH_FORCE)
            self.last_world_vertical_force = float(world_vertical_force)

            # 世界竖直定深力转回机体系后再进入六维分配器。
            body_force[2] = 0.0
            body_force += quaternion_to_rotation(attitude).T@np.array(
                [0.0, 0.0, world_vertical_force])
        elif self.depth_hold:
            # 短暂失效不自动关闭 X 定深模式：暂停深度修正、清除积分。
            # 此时仍允许 LT/RT 直接产生手动竖直力；数据恢复后自动重锁。
            self.depth_integral = 0.0

        self.was_vertical_active = vertical_active
        self.depth_was_valid = depth_valid
        return body_force

    def _open_loop_body_force(self, joystick):
        """开环模式：平移手柄直接给机体系 [Fx,Fy,Fz]。"""
        left_x = float(joystick.get("lx", 0.0))
        left_y = float(joystick.get("ly", 0.0))
        left_trigger = max(0.0, float(joystick.get("lt", 0.0)))
        right_trigger = max(0.0, float(joystick.get("rt", 0.0)))
        return MAX_OPEN_LOOP_FORCE*np.array([
            -left_y, -left_x, right_trigger-left_trigger])

    @staticmethod
    def _manual_motion_is_neutral(joystick):
        """判断六自由度手动输入是否全部松开（按键模式量不参与）。"""
        axes = ("lx", "ly", "rx", "ry", "lt", "rt")
        return (all(abs(float(joystick.get(key, 0.0))) <= 0.01
                    for key in axes)
                and not bool(joystick.get("lb", False))
                and not bool(joystick.get("rb", False)))

    def _allocate(self, target_wrench, now, fast_center=False):
        """低频搜索最优舵角；每拍按 estimated 舵角重新算推力，输出交给 _format。

        流程：①每 FULL_ALLOCATION_PERIOD 把六维目标投给后台线程选舵角；
        ②取回舵角驱动 desired/estimated；③按估计舵角算"可达目标"（|sinθ| 保护）；
        ④在当前舵角下解出发给四台推进器的推力（带变化率限幅）。
        """
        if fast_center:
            # 开环手柄全部松开时不再等下一次优化，也不经过目标低通。
            self.solver_servo_angles[:] = 0.0
            self.desired_servo_angles[:] = 0.0
            self.allocator.last_x = None
        elif now-self.last_full_allocation_time >= FULL_ALLOCATION_PERIOD:
            # 只投递请求；真正的求解放到后台线程，绝不阻塞控制循环。
            with self._solver_lock:
                self._solve_request = (
                    np.asarray(target_wrench, dtype=float).copy(),
                    np.r_[self.estimated_servo_angles,
                          self.filtered_thruster_forces])
            self.last_full_allocation_time = now

        # 非阻塞取回最近一次后台求解的舵角。只在"够新 + 目标没大变"时采用，
        # 否则沿用上一组角度——换向途中算完的旧计划直接用会出力方向错。
        with self._solver_lock:
            solved = self._solve_result
            self._solve_result = None
        if solved is not None:
            angles, request_time, requested_target = solved
            age_s = max(0.0, now-request_time)
            target_delta = float(np.max(
                np.abs(target_wrench-requested_target)
                / SOLVER_TARGET_DELTA_SCALE))
            self.last_solver_result_age_ms = age_s*1000.0
            if (age_s <= SOLVER_RESULT_MAX_AGE_S
                    and target_delta <= SOLVER_TARGET_DELTA_MAX):
                self.solver_servo_angles = np.clip(
                    angles, -SERVO_LIMIT, SERVO_LIMIT)
                self._solved = True

        # 目标角在每个 50Hz 周期连续趋近 solver 方向，避免舵机"转一下、停一下"。
        if not fast_center:
            angle_error = self.solver_servo_angles-self.desired_servo_angles
            self.desired_servo_angles += SERVO_TARGET_BLEND*angle_error
            # 接近目标就一次吸附到位，避免无限逼近造成舵机持续微动/卡顿。
            close = (np.abs(self.solver_servo_angles
                            - self.desired_servo_angles)
                     <= SERVO_TARGET_SNAP)
            self.desired_servo_angles[close] = self.solver_servo_angles[close]

        # estimated 按额定舵速逼近 desired，用于估算"尚在转动中的真实角度"。
        servo_rate = SERVO_RETURN_RATE if fast_center else SERVO_COMMAND_RATE
        servo_step = servo_rate*self._dt
        self.estimated_servo_angles += np.clip(
            self.desired_servo_angles-self.estimated_servo_angles,
            -servo_step, servo_step)

        # ---- 发力门控："先转到位置、再发力"，**只在开环用** ----
        # 闭环里必须关掉：门控会把 Fz/Fy/Mx 目标反复清零 →"升沉没力、舵角没人管乱跳"。
        # 闭环要连续出力；起步猛冲由下面的 |sinθ| 保护负责。
        wrench_jump = float(np.max(np.abs(
            np.asarray(target_wrench, dtype=float) - self._prev_target_wrench)))
        self._prev_target_wrench = np.asarray(target_wrench, dtype=float).copy()
        if self.closed_loop:
            self._gate[:] = 1.0
        else:
            if (not self._solved) or wrench_jump > 1.0:
                # 检测到新动作 → 强制关门 GATE_HOLD_S，够舵机转到位。
                self._gate_hold_until = now + GATE_HOLD_S
            if now < self._gate_hold_until:
                self._gate[:] = 0.0
            else:
                # 到位程度 = 估计舵角离目标角还差多少；差得远不出力、接近才放开。
                approach = np.clip(
                    1.0 - np.abs(self.estimated_servo_angles
                                 - self.solver_servo_angles)/GATE_MARGIN, 0.0, 1.0)
                self._gate = np.where(
                    approach < self._gate,
                    approach,
                    self._gate + 0.1*(approach - self._gate))
        gate = self._gate

        # ---- 舵角保护 |sinθ| ----
        # 舵角很小时 Fy/Fz/Mx 根本产生不出来；若强求完整目标，求解器会用"巨大且互相
        # 抵消的推力"（四路顶满、净力≈0，机器人却乱冲）。故按 |sinθ| 压住这些分量：
        #   Fy 用 θ12、Fz 用 θ34，带 0.4 下限（别把升沉/滚转压太狠）；
        #   Mx 特殊——θ=0 时物理上完全产生不出 Mx，用**无下限**的 |sin|、两对取较大：
        #      θ≈0 时 Mx 目标→0（先等舵角转起来），避免分配器四路饱和硬凑 →"突然冲出去"。
        raw_sin = np.abs(np.sin(self.estimated_servo_angles))
        sin_attn = np.maximum(raw_sin, 0.4)
        attain = gate*sin_attn
        roll_attn = float(np.max(gate*raw_sin))
        attainable_wrench = np.asarray(target_wrench, dtype=float).copy()
        attainable_wrench[1] *= attain[0]               # 横移 Fy <- θ12
        attainable_wrench[2] *= attain[1]               # 升沉 Fz <- θ34
        attainable_wrench[3] *= roll_attn               # 横滚 Mx <- 两对取较大，无下限

        # 调试：闭环每 0.5s 打印目标力/矩 + 门控 + 舵角，用于定位"失控/猛冲"。
        # wrch 是"原始目标"（未打折）；servo 是估计舵角（看它有没有上到大角度）。
        if self.closed_loop and now-self._alloc_debug_time >= 0.5:
            self._alloc_debug_time = now
            print(f"[闭环] wrch=({target_wrench[0]:+.1f},{target_wrench[1]:+.1f},"
                  f"{target_wrench[2]:+.1f} | {target_wrench[3]:+.2f},"
                  f"{target_wrench[4]:+.2f},{target_wrench[5]:+.2f})  "
                  f"gate=({gate[0]:.2f},{gate[1]:.2f})  "
                  f"servo=({np.degrees(self.estimated_servo_angles[0]):+.0f},"
                  f"{np.degrees(self.estimated_servo_angles[1]):+.0f})°")

        # 当前舵角下解出四台推力。推力变化率作为**求解边界**传进去，
        # 而不是先解再截断——这样解出来就是这一拍物理上执行得到的。
        thrust_step = THRUST_RATE_LIMIT*self._dt
        fixed_started = time.perf_counter()
        allocation = self.allocator.solve_fixed_angles(
            *self.estimated_servo_angles, attainable_wrench,
            reference_forces=self.filtered_thruster_forces,
            force_step=thrust_step)
        self.last_fixed_solver_ms = (time.perf_counter()-fixed_started)*1000.0
        self.last_allocated_forces = allocation.forces().copy()
        self.last_allocation_saturated = bool(allocation.saturated)
        self.last_allocation_near_saturation = bool(allocation.near_saturation)
        self.last_solver_ok = bool(allocation.solver_ok)
        self.last_allocation_achievable = bool(allocation.achievable)
        self.filtered_thruster_forces += np.clip(
            allocation.forces()-self.filtered_thruster_forces,
            -thrust_step, thrust_step)
        self.filtered_thruster_forces[:] = np.clip(
            self.filtered_thruster_forces,
            THRUSTER_REVERSE_LIMIT, THRUSTER_FORWARD_LIMIT)

        # 诊断：用估计舵角 + 限速后的推力正解回来，和送分配器的目标比较。
        self.last_target_wrench = np.asarray(target_wrench, dtype=float).copy()
        self.last_wrench_for_allocator = attainable_wrench.copy()
        final_vector = np.r_[self.estimated_servo_angles,
                             self.filtered_thruster_forces]
        self.last_achieved_wrench = self.allocator.wrench(final_vector)
        final_error = self.last_achieved_wrench-attainable_wrench
        self.last_force_error = float(np.max(np.abs(final_error[:3])))
        self.last_moment_error = float(np.max(np.abs(final_error[3:])))

    def _format_ctrl_command(self):
        """转换为 CTRL:f1,f2,f3,f4,s13,s24,gripper。"""
        # 内部[1上,3下,2右,4左] -> STM32 物理编号[1,2,3,4]。
        physical_order = self.filtered_thruster_forces[[0, 2, 1, 3]]

        # STM32 中 -1/0/+1 对应 1100/1500/1900 us。正反向额定推力不同，
        # 必须分别除以 +33 N 和 24.5 N，不能统一除以 30 N。
        normalized_thrust = np.where(
            physical_order >= 0.0,
            physical_order/THRUSTER_FORWARD_LIMIT,
            physical_order/abs(THRUSTER_REVERSE_LIMIT))
        normalized_thrust = np.clip(normalized_thrust, -1.0, 1.0)
        # 实物无舵角反馈，发送按额定 90°/s 推算的 estimated，而不是瞬间跳到
        # 优化器 desired；否则相邻解变化会直接表现为舵机左右猛转。
        normalized_servo = np.clip(
            self.estimated_servo_angles/SERVO_LIMIT, -1.0, 1.0)
        gripper = 0.5 if self.gripper_closed else -0.5
        values = np.r_[normalized_thrust, normalized_servo, gripper]
        return "CTRL:"+",".join(f"{value:.4f}" for value in values)

    def update(self, joystick, imu, depth, now=None):
        """执行一个 50 Hz 周期并返回 CTRL 或 OFF 字符串。

        joystick: lx,ly,rx,ry,lt,rt,a,b,x,y,lb,rb
        imu:      ok, quaternion[w,x,y,z], gyro[rad/s]
        depth:    valid, depth[m，向下为正], depth_rate[m/s]
        """
        now = time.monotonic() if now is None else float(now)
        # ---- 实测周期：所有积分和变化率步长都按它算 ----
        # 之前固定按 CONTROL_DT=0.02 算，实际循环 24.4ms（41Hz），等于所有
        # "按拍"的限制都松了 20%，姿态积分也慢 20%。
        if self._last_update_time is None:
            raw_dt = CONTROL_DT
        else:
            raw_dt = now-self._last_update_time
        self._last_update_time = now
        if not math.isfinite(raw_dt) or raw_dt <= 0.0:
            raw_dt = CONTROL_DT
        self.last_cycle_dt_s = float(raw_dt)
        self._dt = float(np.clip(raw_dt, DT_MIN, DT_MAX))
        imu_ok = bool(imu.get("ok", False))
        attitude = (normalize_quaternion(imu["quaternion"])
                    if imu_ok else None)
        self._handle_mode_buttons(joystick, attitude, depth)
        if not self.armed:
            return "OFF"

        if self.closed_loop:
            if not imu_ok:
                return self.stop()
            gyro = np.asarray(imu["gyro"], dtype=float)
            manual_rate = self._manual_angular_rate(joystick)
            body_moment = self._attitude_moment(attitude, gyro, manual_rate)
            body_force = self._body_force(joystick, attitude, depth)
        else:
            # 纯开环：六维目标完全由手柄生成，不读取 IMU 或深度计。
            self.depth_hold = False
            body_force = self._open_loop_body_force(joystick)
            body_moment = self._manual_body_moment(joystick)

        # 唯一符号入口：实物力矩相对模型整体反号（三轴皆然）。
        # 反馈与手柄前馈在此合并后一起反号，二者方向因此天然一致。
        target_wrench = np.r_[body_force, PLANT_MOMENT_SIGN*body_moment]

        # 六维目标变化率限幅（防猛冲）：任何来源的跳变（舵角分支跳、IMU 尖峰、
        # 门控开合、fast_center）都先经过这里，1~2 拍内不会直接变成"猛冲"。
        wrench_step = WRENCH_RATE_LIMIT*self._dt
        self._limited_wrench = self._limited_wrench + np.clip(
            target_wrench-self._limited_wrench, -wrench_step, wrench_step)
        target_wrench = self._limited_wrench

        # 非定深状态、且六自由度手动输入全松开、且**姿态真的接近水平**，
        # 才把舵角收回中位。
        # 只看"力矩小"是不行的：松手瞬间目标姿态会重锁到当前姿态，误差归零、
        # 力矩也归零，于是判据成立、舵角收中位、sinθ 归零、四台推力全塌——
        # 表现出来就是"先停转，然后姿态掉下来"。所以加一条"真的水平了"。
        manual_neutral = self._manual_motion_is_neutral(joystick)
        if attitude is not None:
            # R33 = 1-2(x²+y²)，即重力方向在机体系 z 轴的投影；1.0 = 完全水平
            upright = 1.0-2.0*(attitude[1]*attitude[1]
                               + attitude[2]*attitude[2])
        else:
            upright = 1.0
        fast_center = (not self.depth_hold and manual_neutral
                       and upright > 0.996          # 倾角小于 5°
                       and (not self.closed_loop
                            or np.max(np.abs(body_moment)) < 0.02))
        self._allocate(target_wrench, now, fast_center=fast_center)
        return self._format_ctrl_command()
