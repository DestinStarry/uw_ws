"""AUV 正解、解析反解和基于 MuJoCo 几何的精确分配。"""

from dataclasses import dataclass
import math
import numpy as np
from scipy.optimize import least_squares

# ===========================================================================
# 机构参数修改区
# 更换重心、推进器安装位置、舵机方向或推进器型号时，只修改本区域。
# 运行时不读取MuJoCo XML；下面所有位置单位为m，均在机体系中表达。
# 坐标系：+X前进；当前模型约定+Y横向、+Z垂向。
# 内部推进器顺序固定为：[1上, 3下, 2右, 4左]。
# ===========================================================================

# 机体坐标原点下的实际重心位置。实物完成配重后应重新测量此值。
BODY_CENTER_OF_MASS = np.array([
    -0.01336829,
     0.00002950,
    -0.00000930,
])

# 四个推进器发力点在机体原点坐标系中的位置，而不是相对重心的位置。
THRUSTER_MOUNT_POSITIONS = np.array([
    [-0.223,  0.000,  0.123],  # 1号：上
    [-0.223,  0.000, -0.123],  # 3号：下
    [-0.223, -0.123,  0.000],  # 2号：右
    [-0.223,  0.123,  0.000],  # 4号：左
])

# 正解使用的真实力臂：r_i = 推进器发力点 - 重心。
# 力矩由 M_i = r_i × F_i 自动计算；不要再手工重复修改此数组。
THRUSTER_LEVER_ARMS = THRUSTER_MOUNT_POSITIONS-BODY_CENTER_OF_MASS

# 零舵角时，每个推进器都沿机体系+X方向发力。
THRUSTER_ZERO_DIRECTIONS = np.array([
    [1.0, 0.0, 0.0],  # 1上
    [1.0, 0.0, 0.0],  # 3下
    [1.0, 0.0, 0.0],  # 2右
    [1.0, 0.0, 0.0],  # 4左
])

# 以两组控制舵角为自变量后的等效旋转轴。
# 1/3号实际反向转动；2/4号实际同向转动。
THRUSTER_ROTATION_AXES = np.array([
    [0.0,  0.0,  1.0],  # 1上：绕+Z
    [0.0,  0.0, -1.0],  # 3下：绕-Z
    [0.0, -1.0,  0.0],  # 2右：绕-Y
    [0.0, -1.0,  0.0],  # 4左：绕-Y
])

# 由上面四个数组派生的 Rodrigues 预计算项（不要单独维护）：
#   direction(θ) = u0·cosθ + (k×u0)·sinθ + k·(k·u0)·(1-cosθ)
# 本机构里 k ⟂ u0，第三项恒为 0；仍保留，将来改舵轴方向也不会失配。
THRUSTER_ROTATION_CROSS = np.cross(
    THRUSTER_ROTATION_AXES, THRUSTER_ZERO_DIRECTIONS)
THRUSTER_AXIS_PROJECTION = np.einsum(
    "ij,ij->i", THRUSTER_ROTATION_AXES, THRUSTER_ZERO_DIRECTIONS)
THRUSTER_ANGLE_AXIS_TERM = (
    THRUSTER_ROTATION_AXES*THRUSTER_AXIS_PROJECTION[:, None])

# 解析反解用的“理想对称力臂” **从上面的推进器位置/重心自动派生**——这样只需维护
# THRUSTER_MOUNT_POSITIONS 和 BODY_CENTER_OF_MASS 一处，不会再出现“位置改了、这里的
# 力臂忘了改”导致两套模型对不上。前提是结构对称：上下那对在 z=±H、左右那对在 y=±H、
# 四台推进器纵向都在同一个 x。
SYMMETRIC_RADIAL_ARM = abs(THRUSTER_MOUNT_POSITIONS[0, 2])          # H：横向偏移(上下对的 z)
SYMMETRIC_LONGITUDINAL_ARM = abs(THRUSTER_MOUNT_POSITIONS[0, 0]
                                - BODY_CENTER_OF_MASS[0])            # D：纵向力臂

# 执行器物理限制。
SERVO_LIMIT = math.pi/2.0             # 两组舵机均限制在±90°
THRUSTER_REVERSE_LIMIT = -24.5        # 单台反向最大静推力，N
THRUSTER_FORWARD_LIMIT = 33.0         # 单台正向最大静推力，N

# ===========================================================================
# 求解器权重：通常无需因机械安装变化而修改
# ===========================================================================
# 各维跟踪权重。Fx/Fy 取 1：不强行猛抵消前后/左右分量——提到 4 时求解器会
# “较劲”式地互相顶着抵消，表现为转动途中推力很大、到位后突然掉下来。
WRENCH_WEIGHTS = np.array([1., 1., 4., 24., 24., 24.])
# 动态分配代价权重。这些偏好直接参加非线性求解，而不是求解后硬切分支。
ANGLE_ZERO_REG = 0.05   # 偏向小舵角的权重（标量，两对相同=不偏袒）。
# 2026-10-16 一度改成 [0.02,0.05] 想逼滚转优先用上下对，但那会把升沉/俯仰耦合进来、
# 加剧振荡，回退为标量。整体偏小 → 解走大舵角(~70°)，升沉不被 |sinθ| 打折。
ANGLE_CONTINUITY_REG = 0.10    # 连续性正则。取 0.75 时会把解往“当前角度”拽，配合
                               # 有限的迭代次数，导致每拍只前进一点点——舵角目标被
                               # 慢慢拖上去、舵机显慢，且“先到位再发力”的门控失效
                               # （估计角≈目标角，门一直开着）。松开后解得以及时跳到最优。
FORCE_ZERO_REG = 0.030         # 最小推力/能耗
FORCE_CONTINUITY_REG = 0.025   # 推力连续性
SATURATION_REG = 2.00          # 强烈避免超过额定推力80%
# 解析反解只是"理想对称模型"的种子，真实几何（重心偏置）会让它差 1.3e-3 左右。
ANALYTIC_VALIDATION_TOL = 2e-3
EPS = 1e-8

# 舵角信赖域半宽。取到 SERVO_LIMIT 即“不额外限制”：真正的限速交给舵机自身
# 的 90°/s 斜率，分支稳定性交给 BRANCH_SWITCH_HYSTERESIS 与热启动。
# 原来取 45° 时，配合重规划周期会让舵角目标“挪 45°→停→再挪 45°”，
# 表现为滚转时舵机“中途停几次才到位”；取消后目标一次给到最优、平滑滑过去。
ANGLE_TRUST_REGION = SERVO_LIMIT

# 分支切换滞回：0 = 每次直接取最优种子（求解器会一步跳到最优舵角）。
# 之前取 0.10 时，"维持当前分支"导致解贴着当前角度慢慢爬、舵机不到位、门控失效。
BRANCH_SWITCH_HYSTERESIS = 0.0

@dataclass
class Allocation:
    # 内部顺序：上、下、右、左。
    f1: float = 0.0
    f2: float = 0.0
    f3: float = 0.0
    f4: float = 0.0
    theta12: float = 0.0
    theta34: float = 0.0
    error: float = math.inf
    # 诊断字段：只记录，不参与控制。
    force_error: float = math.inf
    moment_error: float = math.inf
    solver_ok: bool = False
    saturated: bool = False
    near_saturation: bool = False
    achievable: bool = False

    def forces(self):
        return np.array([self.f1, self.f2, self.f3, self.f4], dtype=float)


def _polar(x, y, default_theta=0.0):
    radius = math.hypot(x, y)
    if radius <= EPS:
        return 0.0, float(np.clip(default_theta, -SERVO_LIMIT, SERVO_LIMIT))
    # 横移/升沉的奇异区：x 在零附近受闭环噪声影响变号时，普通 atan2
    # 会让舵角从 +90° 突然跳到 -90°，同时把推力整体反号。锁住该分支。
    if abs(y) > EPS and abs(x) <= 0.20*abs(y):
        keep_previous = abs(default_theta) > math.pi/4
        branch_sign = (math.copysign(1., default_theta) if keep_previous
                       else math.copysign(1., y))
        return y/branch_sign, branch_sign*math.pi/2
    angle = math.atan2(y, x)
    if angle > SERVO_LIMIT:
        return -radius, angle-math.pi
    if angle < -SERVO_LIMIT:
        return -radius, angle+math.pi
    return radius, angle


def thruster_directions(theta12, theta34):
    """给定两对舵角，返回四台推进器的单位推力方向（4×3，内部顺序上/下/右/左）。"""
    angles = np.array([theta12, theta12, theta34, theta34], dtype=float)
    cosines = np.cos(angles)[:, None]
    sines = np.sin(angles)[:, None]
    return (THRUSTER_ZERO_DIRECTIONS*cosines
            + THRUSTER_ROTATION_CROSS*sines
            + THRUSTER_ANGLE_AXIS_TERM*(1.0-cosines))


def wrench_matrix(theta12, theta34):
    """力/矩到推力的唯一映射：W(θ)·[f1,f2,f3,f4] -> [Fx,Fy,Fz,Mx,My,Mz]。

    第 i 列 = 第 i 台推进器单位推力产生的六维力/矩（含真实力臂 r_i × F_i）。
    全项目只此一处做正解，别处不要重复写解析式。
    """
    directions = thruster_directions(theta12, theta34)
    moments = np.cross(THRUSTER_LEVER_ARMS, directions)
    return np.vstack([directions.T, moments.T])


def _wrench_from_vector(vector):
    """唯一正解：x = [θ12, θ34, f1上, f2下, f3右, f4左] -> [Fx..Mz]。"""
    theta12, theta34 = vector[0], vector[1]
    forces = np.asarray(vector[2:], dtype=float)
    return wrench_matrix(theta12, theta34)@forces


def forward_kinematics(a):
    """[四推力, 两舵角] -> [Fx,Fy,Fz,Mx,My,Mz]（转发给唯一正解）。

    以前这里是理想对称解析式，而分配器用的是真实力臂，两套模型并存。
    统一之后差小于 1.3e-3，且只有一处需要维护。
    """
    return _wrench_from_vector(
        (a.theta12, a.theta34, a.f1, a.f2, a.f3, a.f4))


def inverse_kinematics(target):
    """理想对称模型解析反解，保留二次方程两个分支并择优。"""
    fx, fy, fz, mx, my, mz = np.asarray(target, dtype=float)
    q, theta12 = _polar(
        (my-SYMMETRIC_LONGITUDINAL_ARM*fz)/SYMMETRIC_RADIAL_ARM, fy)
    s12, c12 = math.sin(theta12), math.cos(theta12)
    a_dir = ((mz+SYMMETRIC_LONGITUDINAL_ARM*fy)
             / SYMMETRIC_RADIAL_ARM)
    b_dir = -mx/SYMMETRIC_RADIAL_ARM
    c2 = c12*s12
    c1 = -fx*s12-c12*b_dir
    c0 = fx*b_dir-fz*a_dir
    candidates = []
    if abs(c2) > EPS:
        delta = c1*c1-4*c2*c0
        if delta < -1e-9:
            raise ValueError(f"不可达：判别式={delta:.6g}")
        root = math.sqrt(max(0., delta))
        candidates = [(-c1+root)/(2*c2), (-c1-root)/(2*c2)]
    elif abs(c1) > EPS:
        candidates = [-c0/c1]
    elif abs(c0) <= 1e-9:
        candidates = [0.5*fx/c12 if abs(c12) > EPS else 0., 0.]
    else:
        raise ValueError("退化情况下目标不可达")

    best = None
    for p in candidates:
        xs, yd = fx-p*c12, b_dir-p*s12
        if math.hypot(xs, fz) > EPS or math.hypot(a_dir, yd) <= EPS:
            s, theta34 = _polar(xs, fz)
            c34, s34 = math.cos(theta34), math.sin(theta34)
            diff = a_dir/c34 if abs(c34) > EPS else yd/s34
        else:
            diff, theta34 = _polar(a_dir, yd)
            s = 0.
        a = Allocation(0.5*(p+q), 0.5*(p-q), 0.5*(s+diff),
                       0.5*(s-diff), theta12, theta34)
        a.error = float(np.max(np.abs(forward_kinematics(a)-target)))
        rank = a.error+1e-8*np.dot(a.forces(), a.forces())
        if best is None or rank < best[0]:
            best = (rank, a)
    result = best[1]
    if (result.error > ANALYTIC_VALIDATION_TOL or
            np.any(result.forces() < THRUSTER_REVERSE_LIMIT) or
            np.any(result.forces() > THRUSTER_FORWARD_LIMIT)):
        raise ValueError("解析反解校验失败或推力超过 [-24.5, +33.0] N")
    return result


def _solve_box_qp(hessian, gradient, lower, upper, max_iter=24):
    """min 0.5 x'Hx - g'x  s.t. lower <= x <= upper，四变量 BVLS active-set。

    规模只有 4 维，写成小循环比调通用求解器快两个数量级。
    """
    tol = 1e-8
    try:
        unconstrained = np.linalg.solve(hessian, gradient)
    except np.linalg.LinAlgError:
        unconstrained = np.linalg.lstsq(hessian, gradient, rcond=None)[0]
    force = np.clip(unconstrained, lower, upper)
    active = np.zeros(force.size, dtype=np.int8)
    active[force <= lower + tol] = -1
    active[force >= upper - tol] = 1
    force[active < 0] = lower[active < 0]
    force[active > 0] = upper[active > 0]

    converged = False
    for _ in range(max_iter):
        fixed = np.flatnonzero(active != 0)
        free = np.flatnonzero(active == 0)
        if free.size:
            rhs = gradient[free]
            if fixed.size:
                rhs = rhs - hessian[np.ix_(free, fixed)] @ force[fixed]
            h_ff = hessian[np.ix_(free, free)]
            try:
                free_solution = np.linalg.solve(h_ff, rhs)
            except np.linalg.LinAlgError:
                free_solution = np.linalg.lstsq(h_ff, rhs, rcond=None)[0]

            below = lower[free] - free_solution
            above = free_solution - upper[free]
            violations = np.maximum(below, above)
            if np.any(violations > tol):
                local = int(np.argmax(violations))
                idx = int(free[local])
                if below[local] >= above[local]:
                    active[idx] = -1
                    force[idx] = lower[idx]
                else:
                    active[idx] = 1
                    force[idx] = upper[idx]
                continue
            force[free] = np.clip(free_solution, lower[free], upper[free])

        # KKT：边界上的变量若梯度指向可行域内部，就放出来。
        grad = hessian @ force - gradient
        lower_bad = np.flatnonzero((active < 0) & (grad < -tol))
        upper_bad = np.flatnonzero((active > 0) & (grad > tol))
        candidates = []
        if lower_bad.size:
            idx = int(lower_bad[np.argmin(grad[lower_bad])])
            candidates.append((abs(float(grad[idx])), idx))
        if upper_bad.size:
            idx = int(upper_bad[np.argmax(grad[upper_bad])])
            candidates.append((abs(float(grad[idx])), idx))
        if candidates:
            _, release_idx = max(candidates)
            active[release_idx] = 0
            continue
        converged = True
        break

    if not np.all(np.isfinite(force)):
        raise ValueError("box QP returned non-finite force")
    return np.clip(force, lower, upper), converged


def _bounded_weighted_lsq(matrix, target, reference_forces=None, force_step=None):
    """带上下界的加权最小二乘：跟踪六维目标 + 推力连续 + 最小推力。

    和"先无约束求解、再逐台 np.clip"的区别：边界进入优化本身，
    所以一台顶到 +33N 时其余三台会重新分配，力矩不会被截断破坏。

    force_step 不为 None 时，每台的边界额外收窄到"这一拍最多能变多少"
    （= 推力变化率 × dt）。这样解出来的四台推力**本身就是可执行的**，
    不会出现"优化假设 10N、实际只跟到 0.2N"那种解与执行脱节。
    """
    matrix = np.asarray(matrix, dtype=float)
    target = np.asarray(target, dtype=float).reshape(6)
    if matrix.shape != (6, 4):
        raise ValueError("wrench matrix must have shape (6,4)")
    if not np.all(np.isfinite(matrix)) or not np.all(np.isfinite(target)):
        raise ValueError("non-finite allocation input")

    if reference_forces is None:
        reference = np.zeros(4, dtype=float)
    else:
        reference = np.asarray(reference_forces, dtype=float).reshape(4)
        if not np.all(np.isfinite(reference)):
            reference = np.zeros(4, dtype=float)
        reference = np.clip(reference, THRUSTER_REVERSE_LIMIT,
                            THRUSTER_FORWARD_LIMIT)

    lower = np.full(4, THRUSTER_REVERSE_LIMIT, dtype=float)
    upper = np.full(4, THRUSTER_FORWARD_LIMIT, dtype=float)
    if force_step is not None:
        step = abs(float(force_step))
        if math.isfinite(step) and step > 0.0:
            lower = np.maximum(lower, reference-step)
            upper = np.minimum(upper, reference+step)

    weighted_a = WRENCH_WEIGHTS[:, None]*matrix
    weighted_b = WRENCH_WEIGHTS*target
    zero_reg2 = float(FORCE_ZERO_REG)**2
    continuity_reg2 = float(FORCE_CONTINUITY_REG)**2
    hessian = (weighted_a.T@weighted_a
               + (zero_reg2+continuity_reg2)*np.eye(4))
    gradient = weighted_a.T@weighted_b + continuity_reg2*reference
    force, converged = _solve_box_qp(hessian, gradient, lower, upper)
    return force, converged


def _fill_allocation_diagnostics(allocation, matrix, target, solver_ok=True):
    """用同一块正解矩阵回算这一拍的力/矩误差，供日志和回归测试使用。"""
    achieved = np.asarray(matrix, dtype=float)@allocation.forces()
    residual = achieved-np.asarray(target, dtype=float)
    allocation.force_error = float(np.max(np.abs(residual[:3])))
    allocation.moment_error = float(np.max(np.abs(residual[3:])))
    allocation.error = float(np.max(np.abs(residual)))
    forces = allocation.forces()
    allocation.solver_ok = bool(solver_ok)
    allocation.saturated = bool(
        np.any(forces <= THRUSTER_REVERSE_LIMIT+1e-4)
        or np.any(forces >= THRUSTER_FORWARD_LIMIT-1e-4))
    allocation.near_saturation = bool(
        np.any(forces <= -0.80*abs(THRUSTER_REVERSE_LIMIT))
        or np.any(forces >= 0.80*THRUSTER_FORWARD_LIMIT))
    allocation.achievable = bool(
        allocation.force_error <= 0.5 and allocation.moment_error <= 0.1)
    return allocation


class MuJoCoAllocator:
    """使用代码顶部固定机构参数计算正解；名称保留以兼容现有入口。"""
    def __init__(self, model):
        del model  # 接口兼容；实物与仿真均不再依赖 XML 几何。
        self.last_x = None
        self.last_branch = None

    @staticmethod
    def to_vector(a):
        return np.array([a.theta12, a.theta34, a.f1, a.f2, a.f3, a.f4])

    @staticmethod
    def from_vector(x):
        return Allocation(x[2], x[3], x[4], x[5], x[0], x[1])

    def wrench(self, x):
        """x = [θ12, θ34, f1, f2, f3, f4] -> 六维力/矩（转发给唯一正解）。"""
        return _wrench_from_vector(x)

    def solve_full(self, target):
        target = np.asarray(target, dtype=float)
        reference = (self.last_x.copy() if self.last_x is not None
                     else np.zeros(6))
        angle_seeds = [reference[:2].copy()]
        labels = ["current"]
        # 按目标力方向补一个物理意义明确的种子，连同当前/零位/解析种子
        # 足以覆盖主要分支，避免旧版大量六维搜索阻塞仿真线程。
        a1 = (math.copysign(SERVO_LIMIT, target[1])
              if abs(target[1]) > 0.1 else 0.)
        a2 = (math.copysign(SERVO_LIMIT, target[2])
              if abs(target[2]) > 0.1 else 0.)
        # 纯横滚在零舵角处对 Mx 的一阶梯度为零，必须提供非零舵角
        # 种子才能离开奇异点。推力正负由线性内层自行选择。
        if abs(target[3]) > 0.05:
            a1 = SERVO_LIMIT
        angle_seeds.append(np.array([a1, a2]))
        labels.append("direction")

        def force_at(angles):
            # 和 50Hz 路径用同一个**有界**求解器，只是不施加推力变化率：
            # 规划器要回答的是"稳态下这个舵角能出多大力"，所以边界取推力
            # 硬限（±33/−24.5），不取每拍步长。这样它和真正执行的算法一致，
            # 不会出现"规划以为能出 10N、执行器只给 4N"。
            forces, _ = _bounded_weighted_lsq(
                wrench_matrix(angles[0], angles[1]), target,
                reference_forces=reference[2:])
            return forces

        def residual_angles(angles):
            forces = force_at(angles)
            tracking = WRENCH_WEIGHTS*(
                wrench_matrix(angles[0], angles[1])@forces-target)
            minimum_angle = ANGLE_ZERO_REG*angles
            angle_continuity = ANGLE_CONTINUITY_REG*(angles-reference[:2])
            minimum_force = FORCE_ZERO_REG*forces
            force_continuity = FORCE_CONTINUITY_REG*(forces-reference[2:])
            # 正反向分别在各自额定能力的 80% 后增加饱和惩罚。
            saturation = SATURATION_REG*np.where(
                forces >= 0.,
                np.maximum(forces-0.80*THRUSTER_FORWARD_LIMIT, 0.),
                np.maximum(-forces-0.80*abs(THRUSTER_REVERSE_LIMIT), 0.))
            return np.r_[tracking, minimum_angle, angle_continuity,
                         minimum_force, force_continuity, saturation]

        # 信赖域：以“当前舵角”reference[:2] 为中心，单次求解只在附近邻域内搜索。
        # 这是让序贯求解稳定的关键——不设界时，连续项权重稍弱或目标稍有扰动，
        # 解就会在等价分支间跳变（±90° 互换），舵机随之猛转/抖动。
        theta_ref = np.clip(reference[:2], -SERVO_LIMIT, SERVO_LIMIT)
        lower = np.maximum(theta_ref - ANGLE_TRUST_REGION, -SERVO_LIMIT)
        upper = np.minimum(theta_ref + ANGLE_TRUST_REGION, SERVO_LIMIT)
        angle_seeds = [np.clip(s, lower, upper) for s in angle_seeds]

        # 内层改用有界求解后每次评估更贵，迭代上限从 30 收到 18；
        # 实测若 full_solver_ms 仍超过 400ms，再往下收。
        results = [least_squares(
            residual_angles,
            np.clip(s, lower+1e-10, upper-1e-10),
            bounds=(lower, upper), max_nfev=18,
            ftol=1e-8, xtol=1e-8, gtol=1e-8) for s in angle_seeds]

        scores = np.array([float(residual_angles(r.x)@residual_angles(r.x))
                           for r in results])
        best_index = int(np.argmin(scores))
        # 滞回：除非另一分支的代价明显更低，否则维持当前分支（seed0）。
        # 否则代价接近时 argmin 会在等价解之间来回挑，舵机随之抖动。
        if (best_index != 0
                and scores[best_index]
                > (1.0-BRANCH_SWITCH_HYSTERESIS)*scores[0]):
            best_index = 0
        result = results[best_index]
        # 达到迭代上限时 least_squares 可能 success=False，但当前有限解
        # 仍是可用的最小代价结果；只有出现非有限数才判定失败。
        if not np.all(np.isfinite(result.x)):
            raise ValueError("动态分配未收敛")
        # 最终方案用和 50Hz 路径完全相同的有界求解器再算一次，并补齐
        # solver_ok / achievable / force_error 等诊断字段——
        # 上层控制器只取两个舵角，但这些字段必须可信，否则"规划是否成功"
        # 就无从判断。
        matrix = wrench_matrix(result.x[0], result.x[1])
        forces, bounded_ok = _bounded_weighted_lsq(
            matrix, target, reference_forces=reference[2:])
        vector = np.r_[result.x, forces]
        allocation = self.from_vector(vector)
        _fill_allocation_diagnostics(
            allocation, matrix, target,
            solver_ok=bool(result.success and bounded_ok))
        self.last_x = vector.copy()
        self.last_branch = labels[best_index]
        return allocation

    def solve_fixed_angles(self, theta12, theta34, target,
                           reference_forces=None, force_step=None):
        """当前舵角下的有界推力分配（50Hz 路径，每拍调用）。

        边界进入优化本身：一台顶到 ±33/24.5 N 时其余三台会重新分配，
        不会再出现"逐台 clip 把力矩截断"。

        force_step 传 `THRUST_RATE_LIMIT×dt` 时，每台的解被限制在
        "这一拍物理上变得到"的范围内——解出来就是可执行的，
        不再需要事后截断。
        """
        target = np.asarray(target, dtype=float).reshape(6)
        matrix = wrench_matrix(float(theta12), float(theta34))
        forces, ok = _bounded_weighted_lsq(
            matrix, target, reference_forces=reference_forces,
            force_step=force_step)
        allocation = Allocation(*forces, float(theta12), float(theta34))
        return _fill_allocation_diagnostics(allocation, matrix, target, ok)
