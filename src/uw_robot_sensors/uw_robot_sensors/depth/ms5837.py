"""MS5837-02BA / 30BA 压力传感器驱动（Orin I2C 直连，不经商家解算板）。

分层约定
========
本文件只做 **设备层**：寄存器访问、PROM/CRC 校验、数据手册解算、压力→深度换算。
它不依赖 ROS、不启动线程、不保存采集状态；这些分别由 ``driver.py``（服务层）
与 ``depth_node.py``（ROS 层）负责。

单位约定
========
压力 ``mbar``，温度 ``°C``，深度 ``m``（向下为正），密度 ``kg/m³``。

解算依据
========
MS5837-30BA/02BA 数据手册 "PRESSURE AND TEMPERATURE CALCULATION"：
一阶补偿得到 ``dT / SENS / OFF / TEMP``，再按温度区间做二阶补偿 ``Ti / OFFi / SENSi``。
02BA 的满量程是 2 bar（分辨率 0.01 mbar），30BA 是 30 bar（0.1 mbar），
两者的一阶系数与输出标度不同，**用错型号压力会差约 10 倍**——这就是
``detect_model()`` 存在的原因。

与 Blue Robotics 参考实现的差异
==============================
1. 补偿运算抽成纯函数 ``compute()``，可离线单测，也用于型号自检；
2. ``detect_model()`` 用同一组原始值按两种模型各算一次，取更接近水面气压者；
3. ``crc4()`` 在副本上校验，不就地改写调用方数组；
4. 读写失败抛 ``SensorError``，由调用方决定重试或置失效，不返回 False；
5. 转换等待使用数据手册给出的最大转换时间，避免读到上一次结果；
6. 按型号校验 OSR，拒绝下发未定义档位。
"""

from __future__ import annotations

import time

try:  # smbus2 优先：异常类型清晰；退回 python3-smbus。
    import smbus2 as smbus
except ImportError:  # pragma: no cover - 取决于目标机安装的是哪一个
    try:
        import smbus
    except ImportError:
        smbus = None

# ---------------------------------------------------------------------------
# 常量
# ---------------------------------------------------------------------------

#: 型号标识。写入 ``MS5837.model`` 后 ``read()`` 才知道用哪套系数。
MODEL_02BA = 0
MODEL_30BA = 1
MODEL_NAMES = {MODEL_02BA: "02BA", MODEL_30BA: "30BA"}

#: 过采样档位。命令字 = 0x40/0x50 + 2 × 档位。
OSR_256 = 0
OSR_512 = 1
OSR_1024 = 2
OSR_2048 = 3
OSR_4096 = 4
OSR_8192 = 5

#: 参数文件里用采样数（256…8192）更直观，这里做一次映射。
OSR_BY_SAMPLES = {
    256: OSR_256,
    512: OSR_512,
    1024: OSR_1024,
    2048: OSR_2048,
    4096: OSR_4096,
    8192: OSR_8192,
}

#: 各型号支持的最大档位（8192 仅 02BA 提供）。
MAX_OSR = {MODEL_02BA: OSR_8192, MODEL_30BA: OSR_4096}

#: 数据手册给出的最大转换时间（秒），索引即档位；已含约 20% 余量。
_CONVERSION_S = (0.0007, 0.0014, 0.0027, 0.0052, 0.0104, 0.0207)

# I2C 寄存器 / 命令字
_RESET = 0x1E
_ADC_READ = 0x00
_PROM_READ = 0xA0
_CONVERT_D1 = 0x40
_CONVERT_D2 = 0x50

#: 出厂默认地址（模块 CSB 拉低为 0x76，拉高为 0x77）。
DEFAULT_ADDRESS = 0x76

DENSITY_FRESHWATER = 997.0
DENSITY_SALTWATER = 1029.0
GRAVITY = 9.80665

#: 型号自动判别的可信阈值（mbar）。正确型号残差通常 < 10 mbar，
#: 错误型号会差数百 mbar；超过该阈值说明传感器不在空气中或型号无法确定。
MODEL_DETECT_TOLERANCE_MBAR = 50.0

__all__ = [
    "MODEL_02BA", "MODEL_30BA", "MODEL_NAMES",
    "OSR_256", "OSR_512", "OSR_1024", "OSR_2048", "OSR_4096", "OSR_8192",
    "OSR_BY_SAMPLES", "MAX_OSR",
    "DEFAULT_ADDRESS", "DENSITY_FRESHWATER", "DENSITY_SALTWATER", "GRAVITY",
    "MODEL_DETECT_TOLERANCE_MBAR",
    "SensorError", "crc4", "validate_osr", "compute", "detect_model",
    "depth_from_pressure", "MS5837",
]


class SensorError(OSError):
    """I2C 通信或 PROM 校验失败。

    继承 ``OSError`` 是为了让上层既能按业务语义捕获 ``SensorError``，
    也能用 ``except OSError`` 统一兜住 I2C 异常。
    """


# ---------------------------------------------------------------------------
# 纯函数：CRC、解算、换算
# ---------------------------------------------------------------------------


def crc4(prom) -> int:
    """按数据手册校验 PROM 的 CRC4，返回 0…15。

    输入不会被修改：参考实现会就地清掉 ``prom[0]`` 的高 4 位并追加一个 0，
    调用方若复用自己的数组会被悄悄改掉。
    """
    if len(prom) < 7:
        raise ValueError("PROM must contain at least 7 words")
    words = [(prom[0] & 0x0FFF)] + [prom[i] for i in range(1, 7)] + [0]
    remainder = 0
    for i in range(16):
        if i % 2:
            remainder ^= words[i >> 1] & 0x00FF
        else:
            remainder ^= words[i >> 1] >> 8
        for _ in range(8):
            if remainder & 0x8000:
                remainder = ((remainder << 1) ^ 0x3000) & 0xFFFF
            else:
                remainder = (remainder << 1) & 0xFFFF
    return (remainder >> 12) & 0x000F


def validate_osr(model: int, osr: int) -> None:
    """校验档位是否被该型号支持，否则抛 ``ValueError``。"""
    if not 0 <= osr < len(_CONVERSION_S):
        raise ValueError("oversampling must be one of %s" % (tuple(OSR_BY_SAMPLES),))
    if model in MAX_OSR and osr > MAX_OSR[model]:
        raise ValueError(
            "%s supports oversampling up to %d"
            % (MODEL_NAMES[model], 2 ** (8 + MAX_OSR[model]))
        )


def compute(d1: int, d2: int, prom, model: int = MODEL_30BA):
    """一阶 + 二阶温度补偿，返回 ``(压力 mbar, 温度 °C)``。

    :param d1: 压力 ADC 原始值 ``D1``
    :param d2: 温度 ADC 原始值 ``D2``
    :param prom: 7 个字的 PROM（``prom[0]`` 含 CRC，``prom[1:7]`` 为 C1…C6）
    :param model: ``MODEL_02BA`` 或 ``MODEL_30BA``

    说明：两个型号的**一阶**公式与标度不同（02BA 除以 32768 后还要 /100，
    30BA 除以 8192 后 /10 才得到 mbar），二阶修正的系数也不同。
    """
    if model not in MAX_OSR:
        raise ValueError("model must be MODEL_02BA or MODEL_30BA")
    c1, c2, c3, c4, c5, c6 = prom[1:7]

    # ---- 一阶补偿 ---------------------------------------------------------
    d_t = d2 - c5 * 256.0
    if model == MODEL_02BA:
        sens = c1 * 65536.0 + c3 * d_t / 128.0
        off = c2 * 131072.0 + c4 * d_t / 64.0
        pressure = (d1 * sens / 2097152.0 - off) / 32768.0
    else:
        sens = c1 * 32768.0 + c3 * d_t / 256.0
        off = c2 * 65536.0 + c4 * d_t / 128.0
        pressure = (d1 * sens / 2097152.0 - off) / 8192.0
    temperature = 2000.0 + d_t * c6 / 8388608.0

    # ---- 二阶补偿（按温度区间） -------------------------------------------
    t_i = off_i = sens_i = 0.0
    if model == MODEL_02BA:
        # 02BA 只在 20 °C 以下做二阶修正。
        if temperature < 2000.0:
            t_i = 11.0 * d_t * d_t / 34359738368.0
            off_i = 31.0 * (temperature - 2000.0) ** 2 / 8.0
            sens_i = 63.0 * (temperature - 2000.0) ** 2 / 32.0
    else:
        if temperature < 2000.0:
            t_i = 3.0 * d_t * d_t / 8589934592.0
            off_i = 3.0 * (temperature - 2000.0) ** 2 / 2.0
            sens_i = 5.0 * (temperature - 2000.0) ** 2 / 8.0
            if temperature < -1500.0:  # -15 °C 以下的附加项
                off_i += 7.0 * (temperature + 1500.0) ** 2
                sens_i += 4.0 * (temperature + 1500.0) ** 2
        else:
            t_i = 2.0 * d_t * d_t / 137438953472.0
            off_i = (temperature - 2000.0) ** 2 / 16.0
            sens_i = 0.0

    off -= off_i
    sens -= sens_i
    temperature -= t_i

    # ---- 二阶修正后的压力与标度 -------------------------------------------
    if model == MODEL_02BA:
        pressure = ((d1 * sens / 2097152.0 - off) / 32768.0) / 100.0
    else:
        pressure = ((d1 * sens / 2097152.0 - off) / 8192.0) / 10.0
    return pressure, temperature / 100.0


def detect_model(d1: int, d2: int, prom, expected_mbar: float = 1013.25):
    """用同一组原始值按两种型号各算一次，返回 ``(更可能的型号, 残差 mbar)``。

    **前提：传感器处于空气中。** 水下时两种型号算出的压力都远高于水面气压，
    残差都很差，判别会退化成"随便挑一个"；此时应改用固定 ``model`` 配置，
    或让调用方在残差超过 :data:`MODEL_DETECT_TOLERANCE_MBAR` 时拒绝自动判别。
    """
    best, best_error = MODEL_02BA, None
    for model in (MODEL_02BA, MODEL_30BA):
        pressure, _ = compute(d1, d2, prom, model)
        error = abs(pressure - expected_mbar)
        if best_error is None or error < best_error:
            best, best_error = model, error
    return best, best_error


def model_detection_confident(residual_mbar: float) -> bool:
    """残差是否小到可以相信自动判别的结果。"""
    return residual_mbar <= MODEL_DETECT_TOLERANCE_MBAR


def depth_from_pressure(
    pressure_mbar: float,
    surface_mbar: float,
    density: float = DENSITY_FRESHWATER,
) -> float:
    """压差（mbar）换算为深度（m，向下为正）：``P = ρgh``。

    ``surface_mbar`` 是水面零点（绝压），必须在传感器处于水面时标定，
    否则深度会整体偏移。
    """
    if density <= 0:
        raise ValueError("fluid density must be positive")
    return (pressure_mbar - surface_mbar) * 100.0 / (density * GRAVITY)


# ---------------------------------------------------------------------------
# 传输层：只负责 I2C
# ---------------------------------------------------------------------------


class MS5837:
    """MS5837 的 I2C 传输对象：复位、读 PROM、触发转换、读 ADC。

    这里**不做**补偿与标定，也不保存采样历史——只把寄存器操作封成几个方法，
    便于在离线测试中替换（测试可传入一个模拟 ``smbus`` 的对象）。
    """

    def __init__(self, bus: int, address: int = DEFAULT_ADDRESS):
        if not 0 <= address <= 0x7F:
            raise ValueError("I2C address must be 0..0x7F")
        self._bus_number = int(bus)
        self._address = int(address)
        self._bus = None
        #: 7 个字的 PROM，``open()`` 之后可用。
        self.prom = None
        #: 由调用方设置（``driver`` 选型后写入），``read()`` 依赖它选择公式。
        self.model = None

    # -- 生命周期 -----------------------------------------------------------

    def open(self):
        """打开总线、复位传感器、读 PROM 并校验 CRC。返回 PROM。"""
        if smbus is None:
            raise SensorError(
                "python3-smbus / smbus2 missing: sudo apt install python3-smbus2"
            )
        self.close()
        bus = smbus.SMBus(self._bus_number)
        self._bus = bus
        try:
            bus.write_byte(self._address, _RESET)
            time.sleep(0.010)  # 数据手册：复位后至少等 1.5 ms
            self.prom = self._read_prom()
        except Exception as exc:  # 把第三方库的异常统一成语义明确的类型
            self.close()
            raise SensorError("PROM read failed at 0x%02X: %s" % (self._address, exc))
        expected = (self.prom[0] >> 12) & 0x000F
        actual = crc4(self.prom)
        if actual != expected:
            self.close()
            raise SensorError(
                "PROM CRC mismatch at 0x%02X: expected %X, got %X"
                % (self._address, expected, actual)
            )
        return self.prom

    def reopen(self):
        """重新复位并读 PROM；保留已确定的型号，避免水下重新判别。"""
        model = self.model
        self.open()
        self.model = model

    def close(self):
        bus, self._bus = self._bus, None
        if bus is not None:
            try:
                bus.close()
            except Exception:  # pragma: no cover - 部分 smbus 版本没有 close
                pass

    # -- 寄存器访问 ---------------------------------------------------------

    def _read_prom(self):
        prom = []
        for index in range(7):
            raw = self._bus.read_i2c_block_data(self._address, _PROM_READ + 2 * index, 2)
            prom.append((raw[0] << 8) | raw[1])
        return prom

    def _read_adc(self) -> int:
        raw = self._bus.read_i2c_block_data(self._address, _ADC_READ, 3)
        return (raw[0] << 16) | (raw[1] << 8) | raw[2]

    # -- 采样 ---------------------------------------------------------------

    def read_raw(self, osr: int = OSR_4096):
        """采一组 ``(D1, D2)``；按数据手册最大转换时间等待。"""
        if self._bus is None:
            raise SensorError("sensor is not open")
        validate_osr(self.model if self.model is not None else MODEL_30BA, osr)
        delay = _CONVERSION_S[osr]
        try:
            self._bus.write_byte(self._address, _CONVERT_D1 + 2 * osr)
            time.sleep(delay)
            d1 = self._read_adc()
            self._bus.write_byte(self._address, _CONVERT_D2 + 2 * osr)
            time.sleep(delay)
            d2 = self._read_adc()
        except SensorError:
            raise
        except Exception as exc:
            raise SensorError("ADC read failed at 0x%02X: %s" % (self._address, exc))
        return d1, d2

    def read(self, osr: int = OSR_4096):
        """返回 ``(压力 mbar, 温度 °C)``；型号未确定时拒绝计算。"""
        if self.model is None:
            raise SensorError("model not selected")
        d1, d2 = self.read_raw(osr)
        return compute(d1, d2, self.prom, self.model)
