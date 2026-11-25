"""摇杆原始量 -> 机体系六轴速度指令（CMD 帧）。

控制端（uw_ws 的 control_node）只跟踪物理量：轴映射、一阶滤波和变化率限制
都在发送端完成。帧格式：

    CMD:u,v,w,p,q,r,arm,estop,depth,loop

单位：线速度 m/s（x 前、y 右、z 向下），角速度 rad/s（绕 x、y、z）。
"""
import math

import config


def _deg(value):
    return math.radians(value)


class CommandMapper:
    """把归一化摇杆量换算成六轴速度，并做一阶滤波与变化率限制。"""

    def __init__(self, alpha=None, full_scale=None, rate_limit=None):
        self.alpha = config.CMD_FILTER_ALPHA if alpha is None else alpha
        self.full_scale = full_scale or (
            config.CMD_SURGE_FULL, config.CMD_SWAY_FULL, config.CMD_HEAVE_FULL,
            _deg(config.CMD_ROLL_FULL_DEG), _deg(config.CMD_PITCH_FULL_DEG),
            _deg(config.CMD_YAW_FULL_DEG))
        self.rate_limit = rate_limit or (
            config.CMD_RATE_LINEAR, config.CMD_RATE_LINEAR, config.CMD_RATE_LINEAR,
            config.CMD_RATE_ANGULAR, config.CMD_RATE_ANGULAR, config.CMD_RATE_ANGULAR)
        self.state = [0.0] * 6

    def reset(self):
        self.state = [0.0] * 6

    def update(self, lx, ly, rx, ry, lt, rt, lb, rb, dt):
        f = self.full_scale
        # 与旧 JOY 映射逐项一致：左摇杆平移、扳机升沉、右摇杆俯仰偏航、LB/RB 横滚。
        target = (
            -ly * f[0],
            -lx * f[1],
            (rt - lt) * f[2],
            (int(bool(rb)) - int(bool(lb))) * f[3],
            -ry * f[4],
            -rx * f[5],
        )
        step_dt = min(max(float(dt), 0.0), 0.2)
        for i in range(6):
            want = self.state[i] + self.alpha * (target[i] - self.state[i])
            step = self.rate_limit[i] * step_dt
            self.state[i] = max(self.state[i] - step, min(self.state[i] + step, want))
        return tuple(self.state)

    @staticmethod
    def frame(velocity, arm, estop, depth, loop):
        """生成一行 CMD 帧（不含换行，send() 会补）。"""
        body = ','.join('%.6f' % value for value in velocity)
        return 'CMD:%s,%d,%d,%d,%d' % (body, int(bool(arm)), int(bool(estop)),
                                       int(bool(depth)), int(bool(loop)))