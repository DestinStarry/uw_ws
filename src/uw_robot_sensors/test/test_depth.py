"""MS5837 深度链路的离线测试：不依赖 I2C 与 ROS 运行时。

覆盖三层：
* 设备层（``ms5837``）：CRC、一阶/二阶补偿、型号判别、压力→深度；
* 服务层（``driver``）：零点标定状态机、水下上电回退、读取失败与快照新鲜度；
* 配置校验：参数组合不合法时必须在启动阶段失败。
"""

import time

import pytest

from uw_robot_sensors.depth import driver, ms5837

# 结构合法的 PROM：C1..C6 为典型量级，C0 由 crc4 回填。
_CAL = [0, 40000, 38000, 20000, 20000, 30000, 20000]


def _prom(words=_CAL):
    prom = list(words)
    prom[0] = 0
    prom[0] = ms5837.crc4(prom) << 12
    return prom


def _d2_for_celsius(prom, celsius):
    """反解出对应温度所需的 D2（温度单位 0.01 °C）。"""
    d_t = (celsius * 100.0 - 2000.0) * 8388608.0 / prom[6]
    return int(round(prom[5] * 256.0 + d_t))


def _d1_for_pressure(prom, pressure_mbar, model=ms5837.MODEL_02BA):
    """反解出对应压力所需的 D1（要求 dT = 0，一阶结果即终值）。"""
    c1, c2 = prom[1], prom[2]
    if model == ms5837.MODEL_02BA:
        sens, off, divisor, scale = c1 * 65536.0, c2 * 131072.0, 32768.0, 100.0
    else:
        sens, off, divisor, scale = c1 * 32768.0, c2 * 65536.0, 8192.0, 10.0
    target = pressure_mbar * scale * divisor
    return int(round((target + off) * 2097152.0 / sens))


# ---------------------------------------------------------------------------
# 设备层
# ---------------------------------------------------------------------------


def test_crc4_is_self_consistent_and_does_not_mutate_input():
    prom = _prom()
    snapshot = list(prom)
    assert ms5837.crc4(prom) == (prom[0] >> 12) & 0x0F
    assert prom == snapshot, "CRC must not rewrite the caller PROM array"


def test_02ba_pressure_round_trips_at_20c():
    prom = _prom()
    d2 = _d2_for_celsius(prom, 20.0)
    d1 = _d1_for_pressure(prom, 1013.25)
    assert d1 < 2 ** 24, "synthetic raw value must fit the 24-bit ADC"
    pressure, temperature = ms5837.compute(d1, d2, prom, ms5837.MODEL_02BA)
    assert pressure == pytest.approx(1013.25, abs=0.1)
    assert temperature == pytest.approx(20.0, abs=0.01)


def test_02ba_second_order_term_only_applies_below_20c():
    prom = _prom()

    def first_order(d1, d2):
        c1, c2, c3, c4, c5, c6 = prom[1:7]
        d_t = d2 - c5 * 256.0
        sens = c1 * 65536.0 + c3 * d_t / 128.0
        off = c2 * 131072.0 + c4 * d_t / 64.0
        return ((d1 * sens / 2097152.0 - off) / 32768.0) / 100.0

    d1 = _d1_for_pressure(prom, 1013.25)
    warm_d2 = _d2_for_celsius(prom, 20.0)
    warm = ms5837.compute(d1, warm_d2, prom, ms5837.MODEL_02BA)[0]
    assert warm == pytest.approx(first_order(d1, warm_d2), abs=1e-9)

    cold_d2 = _d2_for_celsius(prom, -5.0)
    cold = ms5837.compute(d1, cold_d2, prom, ms5837.MODEL_02BA)[0]
    assert abs(cold - first_order(d1, cold_d2)) > 1e-3, \
        "low-temperature compensation must engage"


def test_detect_model_picks_02ba_for_02ba_raw_values():
    prom = _prom()
    d1 = _d1_for_pressure(prom, 1013.25, ms5837.MODEL_02BA)
    d2 = _d2_for_celsius(prom, 20.0)
    model, residual = ms5837.detect_model(d1, d2, prom)
    assert model == ms5837.MODEL_02BA
    assert residual < 10.0, "wrong model would be off by hundreds of mbar"
    assert ms5837.model_detection_confident(residual)


def test_model_detection_is_not_confident_underwater():
    """1300 mbar（水下约 3 m）时两种型号都远离水面气压，不能自动判别。"""
    prom = _prom()
    d1 = _d1_for_pressure(prom, 1300.0, ms5837.MODEL_02BA)
    d2 = _d2_for_celsius(prom, 20.0)
    _, residual = ms5837.detect_model(d1, d2, prom)
    assert not ms5837.model_detection_confident(residual)


def test_both_models_support_8192_and_reject_undefined_osr():
    with pytest.raises(ValueError):
        ms5837.validate_osr(ms5837.MODEL_30BA, 6)
    ms5837.validate_osr(ms5837.MODEL_30BA, ms5837.OSR_8192)
    ms5837.validate_osr(ms5837.MODEL_02BA, ms5837.OSR_8192)
    ms5837.validate_osr(ms5837.MODEL_30BA, ms5837.OSR_4096)


def test_depth_conversion_matches_hydrostatic_pressure():
    assert ms5837.depth_from_pressure(1013.25, 1013.25) == pytest.approx(0.0)
    # 100 mbar ≈ 1.023 m 淡水柱
    assert ms5837.depth_from_pressure(1113.25, 1013.25) == pytest.approx(1.0228, abs=1e-3)
    assert ms5837.depth_from_pressure(1113.25, 1013.25,
                                      ms5837.DENSITY_SALTWATER) == pytest.approx(
        0.9910, abs=1e-3)


def test_depth_rejects_invalid_density():
    with pytest.raises(ValueError):
        ms5837.depth_from_pressure(1100.0, 1000.0, 0.0)


def test_sensor_error_is_an_oserror():
    """上层用 ``except OSError`` 兜 I2C 异常时不能漏掉驱动自己的错误类型。"""
    assert issubclass(ms5837.SensorError, OSError)


# ---------------------------------------------------------------------------
# 服务层：零点标定状态机
# ---------------------------------------------------------------------------


class FakeClient:
    """最小 I2C 替身：按脚本返回压力，可选让第 N 次之后开始失败。"""

    def __init__(self, pressures, prom=None, fail_from=None):
        self.prom = prom or _prom()
        self.model = ms5837.MODEL_02BA
        self._pressures = list(pressures)
        self._index = 0
        self._fail_from = fail_from
        self.closed = False
        self.reopened = 0

    def open(self):
        return self.prom

    def reopen(self):
        self.reopened += 1

    def close(self):
        self.closed = True

    def read_raw(self, osr=None):
        d2 = _d2_for_celsius(self.prom, 20.0)
        d1 = _d1_for_pressure(self.prom, self._pressures[0], ms5837.MODEL_02BA)
        return d1, d2

    def read(self, osr=None):
        if self._fail_from is not None and self._index >= self._fail_from:
            self._index += 1
            raise ms5837.SensorError("simulated I2C failure")
        pressure = self._pressures[min(self._index, len(self._pressures) - 1)]
        self._index += 1
        return pressure, 20.0


def _config(**overrides):
    base = dict(model="02ba", zero_time_s=0.02, zero_attempts=2, rate_hz=200.0,
                stale_timeout_s=0.5, max_failures=3)
    base.update(overrides)
    return driver.SensorConfig(**base)


def _wait_for(predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return False


def test_zeroing_in_air_uses_measured_surface_pressure():
    client = FakeClient([1013.2, 1013.3, 1013.1, 1013.2])
    sensor = driver.Sensor(_config(), client=client)
    try:
        sensor.start()
        assert _wait_for(lambda: sensor.snapshot().phase == driver.PHASE_READY)
        snapshot = sensor.snapshot()
        assert snapshot.zero_source == driver.ZERO_MEASURED
        assert snapshot.surface_mbar == pytest.approx(1013.2, abs=0.2)
        assert snapshot.depth_m == pytest.approx(0.0, abs=0.01)
    finally:
        sensor.stop()


def test_zeroing_underwater_falls_back_to_expected_pressure():
    """水下上电：绝压远超水面范围，重试后必须回退而不是永久卡住。"""
    client = FakeClient([1300.0, 1301.0, 1300.5, 1300.2])
    sensor = driver.Sensor(_config(allow_zero_fallback=True), client=client)
    try:
        sensor.start()
        assert _wait_for(lambda: sensor.snapshot().phase == driver.PHASE_READY)
        snapshot = sensor.snapshot()
        assert snapshot.zero_source == driver.ZERO_FALLBACK
        assert snapshot.surface_mbar == pytest.approx(1013.25, abs=0.01)
        # 1300 mbar 相对 1013.25 mbar 约为 2.93 m 淡水
        assert snapshot.depth_m == pytest.approx(2.93, abs=0.05)
        assert snapshot.ok
    finally:
        sensor.stop()


def test_fixed_surface_pressure_skips_zeroing():
    config = _config(surface_pressure_mbar=1015.0)
    client = FakeClient([1115.0])
    sensor = driver.Sensor(config, client=client)
    try:
        sensor.start()
        assert _wait_for(lambda: sensor.snapshot().ok)
        snapshot = sensor.snapshot()
        assert snapshot.zero_source == driver.ZERO_CONFIG
        assert snapshot.depth_m == pytest.approx(1.0228, abs=0.02)
    finally:
        sensor.stop()


def test_read_failure_marks_invalid_and_increments_failures():
    client = FakeClient([1013.2], fail_from=0)
    sensor = driver.Sensor(_config(), client=client)
    try:
        sensor.start()
        assert _wait_for(lambda: sensor.snapshot().failures >= 2)
        snapshot = sensor.snapshot()
        assert not snapshot.ok
        assert snapshot.age > 0.0
    finally:
        sensor.stop()


def test_auto_model_refuses_underwater_startup():
    """水下上电 + model:auto：残差过大必须拒绝启动，而不是猜一个型号。"""
    client = FakeClient([1300.0])
    with pytest.raises(driver.ConfigError):
        driver.Sensor(_config(model="auto"), client=client)


# ---------------------------------------------------------------------------
# 配置校验
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("overrides", [
    {"rate_hz": 0.0},
    {"stale_timeout_s": 0.0},
    {"zero_attempts": 0},
    {"max_failures": 0},
    {"fluid_density": 0.0},
    {"address": 0x80},
    {"model": "40ba"},
    {"osr": 9},
    {"expected_surface_mbar": 2000.0},
    {"surface_pressure_mbar": 200.0},
])
def test_invalid_config_is_rejected(overrides):
    with pytest.raises(driver.ConfigError):
        driver.SensorConfig(**overrides).validate()


@pytest.mark.parametrize("name", ["fluid_density", "surface_pressure_mbar",
                                 "expected_surface_mbar", "zero_time_s",
                                 "rate_hz", "stale_timeout_s"])
@pytest.mark.parametrize("value", [float('nan'), float('inf')])
def test_nonfinite_config_rejected(name, value):
    with pytest.raises(driver.ConfigError):
        _config(**{name: value}).validate()


def test_selected_model_written_to_device_and_failed_selection_closes_it():
    client = FakeClient([1013.25])
    client.model = None
    sensor = driver.Sensor(_config(), client=client)
    assert client.model == ms5837.MODEL_02BA
    sensor.stop()
    client = FakeClient([1300.0])
    with pytest.raises(driver.ConfigError):
        driver.Sensor(_config(model="auto"), client=client)
    assert client.closed


def test_calibrating_sample_is_invalid():
    sensor = driver.Sensor(_config(zero_time_s=1.0), client=FakeClient([1013.25]))
    try:
        sensor.start()
        assert _wait_for(lambda: sensor.snapshot().timestamp > 0)
        assert sensor.snapshot().phase == driver.PHASE_ZEROING
        assert not sensor.snapshot().ok
    finally:
        sensor.stop()


def test_zero_failure_does_not_silently_use_estimated_pressure():
    sensor = driver.Sensor(_config(), client=FakeClient([1300.0]))
    try:
        sensor.start()
        assert _wait_for(lambda: sensor.snapshot().phase == driver.PHASE_UNCALIBRATED)
        state = sensor.snapshot()
        assert not state.ok and state.surface_mbar == 0.0
    finally:
        sensor.stop()


def test_unstable_zero_is_rejected():
    sensor = driver.Sensor(_config(), client=FakeClient([1013.25]))
    try:
        assert sensor._finish_zero([1000., 1010., 1020.]) is None
    finally:
        sensor.stop()


def test_recovery_retains_zero_and_resets_depth_derivative():
    client = FakeClient([1113.25], fail_from=2)
    sensor = driver.Sensor(_config(surface_pressure_mbar=1013.25), client=client)
    try:
        sensor.start()
        assert _wait_for(lambda: sensor.snapshot().failures >= 2)
        old = sensor.snapshot().timestamp
        assert not sensor.snapshot().ok
        client._pressures = [1213.25]
        client._fail_from = None
        assert _wait_for(lambda: sensor.snapshot().ok)
        state = sensor.snapshot()
        assert state.timestamp > old
        assert state.surface_mbar == 1013.25
        assert state.rate_mps == pytest.approx(0.)
    finally:
        sensor.stop()


# Numerical fixtures from the supplied 2023 MS5837 driver (_calculate).
@pytest.mark.parametrize("model,d1,d2,pressure,temperature",
[(0, 7000000, 5792563, 1013.2173467617089, -36.4048071853639),
 (0, 7000000, 6631424, 1078.2520826597465, -8.52),
 (0, 7000000, 7680000, 1150.2880859375, 20.0),
 (0, 7000000, 8518861, 1203.8025029932032, 40.00000476837158),
 (1, 7000000, 5792563, 20375.1860120721, -37.44160740509047),
 (1, 7000000, 6631424, 21623.180956486613, -8.84),
 (1, 7000000, 7680000, 23005.76171875, 20.0),
 (1, 7000000, 8518861, 24079.101819131756, 39.89760471954345)])
def test_compensation_matches_supplied_reference(model, d1, d2, pressure, temperature):
    actual = ms5837.compute(d1, d2, _CAL, model)
    assert actual == pytest.approx((pressure, temperature), rel=1e-12, abs=1e-9)
