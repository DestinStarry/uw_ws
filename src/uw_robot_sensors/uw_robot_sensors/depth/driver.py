"""MS5837 采集服务：后台线程、水面零点标定、失效判定与快照。

为什么单独一层
==============
单次转换在 OSR_4096 下约 21 ms，直接在 ROS 定时器里做 I2C 会阻塞 executor，
所以这里用一个后台线程持续采样，节点只通过 :meth:`Sensor.snapshot` 取副本。
采集线程还负责三件事：**水面零点标定**、**失效计数与重连**、**深度变化率滤波**。

状态机
======
::

    ZEROING ──(标定成功)──────────> READY
        │
        ├──(标定失败，未超次数)──> ZEROING（重新采样）
        └──(标定失败，超过次数)──> READY（回退到 expected_surface_mbar）

标定失败最典型的场景是**传感器已经在水下**上电：此时绝压远高于水面气压
（每 1.9 m 约 +187 mbar），不能当作零点。回退值来自配置的
``expected_surface_mbar``，同时会告警提示"深度零点未标定"。
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from statistics import median

from . import ms5837

#: 深度变化率的一阶低通系数，与 hardware_node 的滤波口径保持一致。
RATE_ALPHA = 0.2

#: 合理的水面气压范围（mbar），用于拒绝把水下压力当作零点。
#: 1200 mbar 约等于水下 1.9 m，所以"水下开机"一定落在范围外。
SURFACE_RANGE_MBAR = (700.0, 1200.0)

#: 合理的绝对压力范围（mbar）：30BA 满量程 30 bar。
PRESSURE_RANGE_MBAR = (200.0, 40000.0)

#: 零点标定至少需要的样本数。
ZERO_MIN_SAMPLES = 3

#: 标定期间允许的压力波动（mbar），超过只告警不重标。
ZERO_MAX_SPREAD_MBAR = 5.0

#: 连续失败多少次触发一次 reopen。
REOPEN_EVERY = 25

#: 快照的相位。
PHASE_ZEROING = "zeroing"
PHASE_READY = "ready"

#: 零点来源。
ZERO_CONFIG = "config"
ZERO_MEASURED = "measured"
ZERO_FALLBACK = "fallback"


class ConfigError(ValueError):
    """参数组合不合法。启动失败优于带着错误配置下潜。"""


@dataclass(frozen=True)
class SensorConfig:
    """采集线程的全部可调项；字段与 ``robot.yaml`` 中的 ``depth_node`` 一一对应。"""

    bus: int = 1
    address: int = ms5837.DEFAULT_ADDRESS
    model: str = "auto"  # auto / 02ba / 30ba
    osr: int = ms5837.OSR_4096
    fluid_density: float = ms5837.DENSITY_FRESHWATER
    #: >0 表示跳过标定、直接把它当水面零点（mbar）。
    surface_pressure_mbar: float = 0.0
    #: ``model: auto`` 时的参考气压，也是标定失败的回退零点。
    expected_surface_mbar: float = 1013.25
    #: 单次零点标定的采样时长（s）。
    zero_time_s: float = 3.0
    #: 标定失败后最多重试几轮，超过则回退 ``expected_surface_mbar``。
    zero_attempts: int = 3
    rate_hz: float = 20.0
    stale_timeout_s: float = 0.5
    max_failures: int = 5

    def validate(self) -> "SensorConfig":
        """集中校验，错误信息一次列全，避免"改一个报一个"。"""
        problems = []
        if not 0 <= self.address <= 0x7F:
            problems.append("address 必须在 0..0x7F")
        if self.model not in ("auto", "02ba", "30ba"):
            problems.append("model 必须是 auto / 02ba / 30ba")
        if self.osr not in range(6):
            problems.append("osr 必须是 0..5（OSR_256..OSR_8192）")
        if self.fluid_density <= 0:
            problems.append("fluid_density 必须为正")
        if self.surface_pressure_mbar != 0.0 and not (
                500.0 < self.surface_pressure_mbar < 1500.0):
            problems.append("surface_pressure_mbar 必须为 0（自动标定）或 500..1500")
        if not 800.0 < self.expected_surface_mbar < 1200.0:
            problems.append("expected_surface_mbar 必须是合理海平面气压")
        if self.zero_time_s < 0:
            problems.append("zero_time_s 不能为负")
        if self.zero_attempts < 1:
            problems.append("zero_attempts 至少为 1")
        if self.rate_hz <= 0:
            problems.append("rate_hz 必须为正")
        if self.stale_timeout_s <= 0:
            problems.append("stale_timeout_s 必须为正")
        if self.max_failures < 1:
            problems.append("max_failures 至少为 1")
        if problems:
            raise ConfigError("; ".join(problems))
        return self


@dataclass
class Snapshot:
    """某一时刻的采集状态副本；``age`` 以当前时刻计算，不是采样时刻的值。"""

    ok: bool
    phase: str
    timestamp: float  # monotonic 秒；0 表示尚无样本
    depth_m: float
    rate_mps: float
    pressure_mbar: float
    temperature_c: float
    surface_mbar: float
    zero_source: str
    failures: int
    age: float


def _blank_snapshot() -> Snapshot:
    return Snapshot(ok=False, phase=PHASE_ZEROING, timestamp=0.0, depth_m=0.0,
                    rate_mps=0.0, pressure_mbar=0.0, temperature_c=0.0,
                    surface_mbar=0.0, zero_source="", failures=0, age=1e9)


class Sensor:
    """一颗 MS5837 的采集服务；``start()`` 后由后台线程维护快照。"""

    def __init__(self, config: SensorConfig, warn=None, client=None):
        self._cfg = config.validate()
        self._warn = warn or (lambda message: None)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._snapshot = _blank_snapshot()
        # client 仅供离线测试注入；生产路径一律自建并立即打开。
        self._client = client if client is not None else ms5837.MS5837(
            self._cfg.bus, self._cfg.address)
        if client is None:
            # 打不开就让节点启动失败：深度是定深闭环的输入，静默降级更危险。
            self._client.open()
        self.model = self._select_model()

    # -- 只读属性 -----------------------------------------------------------

    @property
    def prom(self):
        return self._client.prom

    # -- 生命周期 -----------------------------------------------------------

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._loop, name="ms5837", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        self._client.close()

    def snapshot(self) -> Snapshot:
        """返回独立副本，并按当前时刻判定新鲜度。"""
        with self._lock:
            state = Snapshot(**vars(self._snapshot))
        if state.timestamp > 0.0:
            state.age = max(0.0, time.monotonic() - state.timestamp)
        else:
            state.age = 1e9
        state.ok = state.ok and state.age <= self._cfg.stale_timeout_s
        return state

    # -- 型号 ---------------------------------------------------------------

    def _select_model(self) -> int:
        """确定型号：配置优先；``auto`` 时按残差判别，残差太大则拒绝启动。"""
        wanted = self._cfg.model
        if wanted == "02ba":
            return ms5837.MODEL_02BA
        if wanted == "30ba":
            return ms5837.MODEL_30BA
        d1, d2 = self._client.read_raw(ms5837.OSR_4096)
        model, residual = ms5837.detect_model(
            d1, d2, self._client.prom, self._cfg.expected_surface_mbar)
        if not ms5837.model_detection_confident(residual):
            raise ConfigError(
                "型号自动判别不可信（残差 %.0f mbar，阈值 %.0f）：传感器可能已在水下，"
                "或表面气压偏离 %.0f mbar。请在空气中上电，或显式设置 model: 02ba/30ba"
                % (residual, ms5837.MODEL_DETECT_TOLERANCE_MBAR,
                   self._cfg.expected_surface_mbar))
        self._warn("型号自动判别为 %s（残差 %.1f mbar）"
                   % (ms5837.MODEL_NAMES[model], residual))
        return model

    # -- 后台线程 -----------------------------------------------------------

    def _loop(self) -> None:
        cfg = self._cfg
        period = 1.0 / cfg.rate_hz
        # 配置了固定零点就不再标定；否则进入标定状态机。
        surface = cfg.surface_pressure_mbar if cfg.surface_pressure_mbar > 0 else None
        zero_source = ZERO_CONFIG if surface is not None else ZERO_MEASURED
        zero_attempts = 0
        samples = []
        zero_deadline = time.monotonic() + cfg.zero_time_s
        last_time = 0.0
        last_depth = 0.0
        rate = 0.0
        failures = 0

        while not self._stop.is_set():
            started = time.monotonic()
            try:
                pressure, temperature = self._client.read(cfg.osr)
                if not PRESSURE_RANGE_MBAR[0] < pressure < PRESSURE_RANGE_MBAR[1]:
                    raise ms5837.SensorError(
                        "pressure %.1f mbar 超出合理范围" % pressure)
                if failures:
                    self._warn("MS5837 在 %d 次失败后恢复" % failures)
                    failures = 0
                now = time.monotonic()

                if surface is None:
                    samples.append(pressure)
                    if now < zero_deadline:
                        # 标定期间不输出深度：此时没有可信的零点。
                        self._store_sample(now, pressure, temperature,
                                           phase=PHASE_ZEROING, depth=0.0, rate=0.0,
                                           surface=0.0, source="")
                        self._pause(started, period)
                        continue
                    measured = self._finish_zero(samples)
                    if measured is None:
                        zero_attempts += 1
                        if zero_attempts < cfg.zero_attempts:
                            samples = []
                            zero_deadline = now + cfg.zero_time_s
                            self._pause(started, period)
                            continue
                        # 重试用尽：退回标称水面气压，保证仍能输出深度。
                        surface = cfg.expected_surface_mbar
                        zero_source = ZERO_FALLBACK
                        self._warn(
                            "零点标定 %d 次失败，回退到 %.2f mbar（请确认上电时传感器在水面）"
                            % (zero_attempts, surface))
                    else:
                        surface = measured
                        zero_source = ZERO_MEASURED
                        self._warn("水面零点标定为 %.2f mbar" % surface)

                depth = ms5837.depth_from_pressure(
                    pressure, surface, cfg.fluid_density)
                delta_t = now - last_time
                if 0.005 < delta_t < 0.5:
                    rate += RATE_ALPHA * ((depth - last_depth) / delta_t - rate)
                else:
                    rate = 0.0
                last_time, last_depth = now, depth
                self._store_sample(now, pressure, temperature, phase=PHASE_READY,
                                   depth=depth, rate=rate, surface=surface,
                                   source=zero_source)
            except (ms5837.SensorError, ValueError, RuntimeError) as error:
                failures += 1
                self._mark_failed(error, failures)
            self._pause(started, period)

    # -- 内部工具 -----------------------------------------------------------

    def _store_sample(self, now, pressure, temperature, *, phase, depth, rate,
                      surface, source) -> None:
        with self._lock:
            self._snapshot = Snapshot(
                ok=True, phase=phase, timestamp=now, depth_m=depth, rate_mps=rate,
                pressure_mbar=pressure, temperature_c=temperature,
                surface_mbar=surface, zero_source=source, failures=0, age=0.0)

    def _mark_failed(self, error, failures: int) -> None:
        # 保留最后读数，但 ok=False 且 timestamp 不推进：age 会持续增长，
        # 消费端据此判失效，不会把旧深度当成新样本。
        with self._lock:
            self._snapshot.ok = False
            self._snapshot.failures = failures
        if failures == self._cfg.max_failures:
            self._warn("MS5837 连续读取失败 %d 次：%s" % (failures, error))
        if failures and failures % REOPEN_EVERY == 0:
            try:
                self._client.reopen()
                self._warn("MS5837 在第 %d 次失败后重新打开" % failures)
            except Exception as exc:  # 重连失败继续计数，不抛出线程
                self._warn("MS5837 重连失败：%s" % exc)

    def _finish_zero(self, samples):
        """从标定样本中取中位数作为零点；不可信时返回 None。"""
        if len(samples) < ZERO_MIN_SAMPLES:
            return None
        surface = float(median(samples))
        if not SURFACE_RANGE_MBAR[0] < surface < SURFACE_RANGE_MBAR[1]:
            self._warn("标定得到 %.1f mbar，超出水面范围 %s；重新标定"
                       % (surface, SURFACE_RANGE_MBAR))
            return None
        spread = max(samples) - min(samples)
        if spread > ZERO_MAX_SPREAD_MBAR:
            self._warn("标定期间压力波动 %.1f mbar，请保持传感器静止于空气中" % spread)
        return surface

    def _pause(self, started: float, period: float) -> None:
        remaining = period - (time.monotonic() - started)
        if remaining > 0:
            self._stop.wait(remaining)
