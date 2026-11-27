/*
 * CH9434.cpp
 * SPI 转 4 路 UART 芯片 CH9434 驱动 — SPI2 轮询方式
 * 参考沁恒 CH9434EVT 官方例程（CH32F103 标准库）移植到 STM32F407 HAL
 */

#include "CH9434.h"
#include <cstdio>
#include "BoardConfig.h"

CH9434 CH9434::ch9434;

/* ===== 底层 SPI（SPI2 轮询） ===== */

void CH9434::cs(uint8_t level)
{
    HAL_GPIO_WritePin(CH9434_CS_GPIO_Port, CH9434_CS_Pin,
                      level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint8_t CH9434::spi_byte(uint8_t tx)
{
    uint8_t rx = 0;
    if (!_io_ok) return 0xFF; // Fail fast for this poll only; no control interlock.
    ++_spi_calls;
    const HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1, kSpiTimeoutMs);
    if (status != HAL_OK) { ++_spi_errors; _last_spi_error = uint8_t(status); _io_ok=false; }
    spi_delay();
    return rx;
}

/* Conservative 10 us minimum gap; Cortex-M4 cycle counter, not a loop estimate.
 * Host tests validate transactions, not physical timing. */
void CH9434::spi_delay(void)
{
#if defined(__arm__) || defined(__thumb__)
    const uint32_t start = DWT->CYCCNT;
    const uint32_t cycles = (SystemCoreClock / 1000000U) * 10U;
    while (uint32_t(DWT->CYCCNT - start) < cycles) { __NOP(); }
#endif
}

uint8_t CH9434::reg_read(uint8_t addr)
{
    uint8_t rx;
    cs(0);
    spi_delay();           /* CS 建立时间 */
    spi_byte(addr & 0x7F); /* 读命令 */
    rx = spi_byte(0xFF);   /* 读回数据 */
    spi_delay(); // Hold CS after final byte.
    cs(1);
    spi_delay();           /* 帧间隔 */
    return rx;
}

void CH9434::reg_write(uint8_t addr, uint8_t val)
{
    cs(0);
    spi_delay();           /* CS 建立时间 */
    spi_byte(addr | 0x80); /* 写命令 */
    spi_byte(val);         /* 写入数据 */
    spi_delay(); // Hold CS after final byte.
    cs(1);
    spi_delay();           /* 帧间隔 */
}

/* ===== 芯片配置（参考 CH9434EVT） ===== */

void CH9434::clk_init(void)
{
    /* 使用芯片内部 32MHz 时钟（不依赖外部晶振），不分频 */
    reg_write(CH9434_CLK_CTRL_CFG_ADD, 0x00);
    _sys_frequency = 32000000;
    HAL_Delay(50);   /* 时钟切换需延时（官方约 50ms） */
}

void CH9434::uart_config(uint8_t idx, uint32_t bps)
{
    uint8_t addr = 0x10 * idx;
    uint8_t lcr;
    uint32_t x;

    /* 波特率分频：x = 10 * sys_freq / 8 / bps，取整 */
    x = 10 * _sys_frequency / 8 / bps;
    x = (x + 5) / 10;

    /* Deterministic 8N1: clear BREAK and stale format bits before enabling DLAB. */
    lcr = 0x03 | CH9434_UARTx_BIT_DLAB;
    reg_write(addr + CH9434_UARTx_LCR_ADD, lcr);

    /* 写 DLL / DLM（波特率） */
    reg_write(addr + CH9434_UARTx_DLL_ADD, x & 0xFF);
    reg_write(addr + CH9434_UARTx_DLM_ADD, (x >> 8) & 0xFF);

    /* 清 DLAB */
    lcr &= ~CH9434_UARTx_BIT_DLAB;
    reg_write(addr + CH9434_UARTx_LCR_ADD, lcr);
}

void CH9434::uart_fifo(uint8_t idx)
{
    /* 使能 FIFO，触发等级 1280（level=3） */
    reg_write(0x10 * idx + CH9434_UARTx_FCR_ADD, 0x01 | (3 << 6));
}

void CH9434::uart_flow(uint8_t idx, uint8_t en)
{
    uint8_t mcr = reg_read(0x10 * idx + CH9434_UARTx_MCR_ADD);
    mcr &= ~(1 << 5);
    if (en) mcr |= (1 << 5);
    reg_write(0x10 * idx + CH9434_UARTx_MCR_ADD, mcr);
}

void CH9434::uart_irq(uint8_t idx)
{
    /* 使能线路状态 + 发送 + 接收中断，关闭 modem 中断 */
    uint8_t ier = (1<<2) | (1<<1) | (1<<0);
    reg_write(0x10 * idx + CH9434_UARTx_IER_ADD, ier);
}

void CH9434::uart_irq_open(uint8_t idx)
{
    /* MCR OUT2(bit3)：使能 INT# 中断输出 */
    uint8_t mcr = reg_read(0x10 * idx + CH9434_UARTx_MCR_ADD);
    mcr |= (1 << 3);
    reg_write(0x10 * idx + CH9434_UARTx_MCR_ADD, mcr);
}

/* ===== 收发 ===== */

uint16_t CH9434::get_rx_fifo_len(uint8_t idx)
{
    reg_write(CH9434_FIFO_CTRL_ADD, idx & 0x0F);
    uint8_t l = reg_read(CH9434_FIFO_CTRL_L_ADD);
    uint8_t h = reg_read(CH9434_FIFO_CTRL_H_ADD);
    return (uint16_t)((h << 8) | l);
}

void CH9434::push_rx(uint8_t idx, uint8_t value)
{
    ++rx_total[idx];
    decoders_[idx].Feed(value, HAL_GetTick());
}
int32_t CH9434::GetRPM(uint8_t idx) { return idx<4 ? decoders_[idx].Rpm() : 0; }
bool CH9434::RpmValid(uint8_t idx, uint32_t now) const {
    return idx<4 && Ready() && decoders_[idx].Valid(now);
}
uint32_t CH9434::RpmAge(uint8_t idx, uint32_t now) const {
    return idx<4 ? decoders_[idx].Age(now) : UINT32_MAX;
}

/* UART0 SCR (0x07) is scratch storage. Restore it after probing;
 * do not touch LCR, baud settings, or PWM control. */
uint8_t CH9434::writeback_test(void)
{
    const uint32_t errors_before = _spi_errors;
    const uint8_t original = reg_read(0x07);
    const uint8_t patterns[5] = {0x55, 0xAA, 0x00, 0xFF, 0x5A};
    uint8_t mask = 0;
    for (uint8_t i = 0; i < 5; ++i) {
        reg_write(0x07, patterns[i]);
        _probe_read[i] = reg_read(0x07);
        if (_probe_read[i] == patterns[i]) mask |= uint8_t(1U << i);
    }
    reg_write(0x07, original);
    _probe_mask = mask;
    ++_probe_rounds;
    if (mask == 0x1F && errors_before == _spi_errors) ++_probe_pass;
    else ++_probe_fail;
    return _probe_read[4];
}

/* Startup-only verification. DLL/DLM share RBR/IER addresses: select DLAB
 * while reading divisors, then restore the configured 8N1 LCR immediately.
 * FCR is write-only: FIFO enable is checked through IIR[7:6], not FCR readback. */
bool CH9434::configure_and_verify(void)
{
    const uint32_t errors_before = _spi_errors;
    clk_init();
    _clock_read = reg_read(CH9434_CLK_CTRL_CFG_ADD);
    _cfg_mask = 0;
    const uint16_t divisor = uint16_t((_sys_frequency + 4U*CH9434_UART_BAUD) /
                                      (8U*CH9434_UART_BAUD));
    for (uint8_t ch=0; ch<4; ++ch) {
        const uint8_t base = uint8_t(ch*0x10);
        uart_config(ch, CH9434_UART_BAUD);
        uart_fifo(ch);
        // Explicitly disable loopback/flow control; keep OUT2 for existing IRQ setup.
        reg_write(base + CH9434_UARTx_MCR_ADD, 0x08);
        uart_irq(ch);
        UartConfigSnapshot &c = _cfg[ch];
        c.lcr = reg_read(base + CH9434_UARTx_LCR_ADD);
        reg_write(base + CH9434_UARTx_LCR_ADD, 0x83);
        c.dll = reg_read(base + CH9434_UARTx_DLL_ADD);
        c.dlm = reg_read(base + CH9434_UARTx_DLM_ADD);
        reg_write(base + CH9434_UARTx_LCR_ADD, 0x03);
        // A second read verifies DLAB really returned to normal receive mode.
        const uint8_t restored_lcr = reg_read(base + CH9434_UARTx_LCR_ADD);
        c.ier = reg_read(base + CH9434_UARTx_IER_ADD);
        c.mcr = reg_read(base + CH9434_UARTx_MCR_ADD);
        c.iir = reg_read(base + CH9434_UARTx_IIR_ADD);
        c.lsr = reg_read(base + CH9434_UARTx_LSR_ADD);
        c.mask = 0;
        if (c.lcr==0x03 && restored_lcr==0x03) c.mask |= 0x01;
        if (c.dll==uint8_t(divisor)) c.mask |= 0x02;
        if (c.dlm==uint8_t(divisor>>8)) c.mask |= 0x04;
        if (c.ier==0x07) c.mask |= 0x08;
        if ((c.mcr & 0x3F)==0x08) c.mask |= 0x10;
        if ((c.iir & 0xC0)==0xC0) c.mask |= 0x20;
        if (c.mask==0x3F) _cfg_mask |= uint8_t(1U<<ch);
    }
    _init_error = _clock_read!=0 ? 2 : (_cfg_mask!=0x0F ? 3 : 0);
    if (_spi_errors!=errors_before) _init_error=1;
    return _init_error==0;
}

/* ===== Device 接口 ===== */

void CH9434::Init()
{
    GPIO_InitTypeDef gpio = {};

#if defined(__arm__) || defined(__thumb__)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif
    /* 配置 CS 输出（默认拉高，空闲） */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = CH9434_CS_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(CH9434_CS_GPIO_Port, &gpio);
    HAL_GPIO_WritePin(CH9434_CS_GPIO_Port, CH9434_CS_Pin, GPIO_PIN_SET);

    /* 配置 INT# 输入（上拉，低电平有效） */
    gpio.Pin = CH9434_INT_Pin;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(CH9434_INT_GPIO_Port, &gpio);

    for (uint8_t ch=0; ch<4; ++ch) decoders_[ch]=low::RpmDecoder(kEscPolePairs[ch]);
    _started = true;
    _initialized = false;
    _init_attempts = 0;
    _init_error = 0;
    _clock_read = 0xFF;
    for (auto &snapshot : _cfg) snapshot = UartConfigSnapshot{};
    _cfg_mask = 0;
    // Allow module power-up to settle. Bounded retries happen only at startup,
    // never insert clock-settling delays into the running motor/control loop.
    HAL_Delay(100);
    for (uint8_t attempt=0; attempt<3; ++attempt) {
        _io_ok=true;
        ++_init_attempts;
        const uint32_t errors_before = _spi_errors;
        _loopback = writeback_test();
        _probe_ok=(_probe_mask==0x1F && _spi_errors==errors_before);
        if (_probe_mask!=0x1F || _spi_errors!=errors_before) {
            _init_error=1;
        } else if (configure_and_verify()) {
            _initialized=true;
            break;
        }
        if (attempt<2) HAL_Delay(50);
    }
    _last_probe_ms = HAL_GetTick();

}

void CH9434::Handle()
{
    if (!_started) return;
    const uint32_t now=HAL_GetTick();
    if (_have_poll && uint32_t(now-_last_poll_ms)>_poll_max_ms) _poll_max_ms=uint32_t(now-_last_poll_ms);
    _last_poll_ms=now; _have_poll=true;
    _io_ok=true; // Retry bounded I/O next poll, without a blocking reinitialization.
    if (uint32_t(now-_last_probe_ms)>=1000U) {
        const uint32_t errors_before=_spi_errors;
        _loopback=writeback_test();
        _probe_ok=(_probe_mask==0x1F && _spi_errors==errors_before);
        _last_probe_ms=now;
    }
    if (!Ready()) { for (auto &d:decoders_) d.Reset(); return; }
    // Match proven low2: poll every channel, max 128 bytes/channel/cycle.
    // Feed each received byte to the decoder even when FIFO is backlogged.
    for (uint8_t ch=0; ch<4 && _io_ok; ++ch) {
        const uint16_t pending=get_rx_fifo_len(ch);
        if (!_io_ok) break;
        if (pending>4096) { _io_ok=false; ++_fifo_errors; break; }
        const uint16_t count=pending>128 ? 128 : pending;
        for (uint16_t i=0; i<count && _io_ok; ++i) {
            const uint8_t value=reg_read(uint8_t(ch*0x10));
            if (_io_ok) push_rx(ch,value);
        }
    }
    if (!_io_ok) for (auto &d:decoders_) d.Reset();
}

void CH9434::Receive(uint8_t *data, uint16_t len)
{
    (void)data; (void)len;   /* 上位机通用命令不归 CH9434 处理，预留 */
}

/* ===== 用户 API ===== */

uint16_t CH9434::Send(uint8_t idx, const uint8_t *data, uint16_t len)
{
    if (!Ready() || idx > 3 || (!data && len)) return 0;
    uint8_t addr = CH9434_REG_OP_WRITE | (0x10 * idx + CH9434_UARTx_THR_ADD);
    for (uint16_t i = 0; i < len; i++) {
        reg_write(addr, data[i]);
    }
    return len;
}

uint32_t CH9434::GetRxCount(uint8_t idx)
{
    if (idx > 3) return 0;
    return rx_total[idx];
}

uint16_t CH9434::GetRxFifoLenDbg(uint8_t idx)
{
    if (idx > 3) return 0;
    return get_rx_fifo_len(idx);
}

uint8_t CH9434::GetLSRDbg(uint8_t idx)
{
    if (idx > 3) return 0;
    /* 线路状态寄存器: 正常空闲 ~0x60(THR空+移位空), 读回 0xFF 说明 SPI 完全不通 */
    return reg_read(0x10 * idx + CH9434_UARTx_LSR_ADD);
}

uint8_t CH9434::GetWritebackDbg(void)
{
    return _loopback;
}

// Cached compact diagnostic; no SPI operations or control gating here.
size_t CH9434::FormatRpmSummary(char *buffer, size_t capacity) const
{
    if (!buffer || !capacity) return 0;
    const int n=std::snprintf(buffer,capacity,
        "RDBG INIT:%s TRY:%u ERR:%u CFG:%X MASK:%02X SPI_ERR:%lu FIFO_ERR:%lu RX:%lu,%lu,%lu,%lu BAD:%lu,%lu,%lu,%lu\r\n",
        _initialized ? "READY" : "FAILED",unsigned(_init_attempts),unsigned(_init_error),
        unsigned(_cfg_mask),unsigned(_probe_mask),(unsigned long)_spi_errors,(unsigned long)_fifo_errors,
        (unsigned long)rx_total[0],(unsigned long)rx_total[1],(unsigned long)rx_total[2],(unsigned long)rx_total[3],
        (unsigned long)decoders_[0].RejectedWindows(),(unsigned long)decoders_[1].RejectedWindows(),
        (unsigned long)decoders_[2].RejectedWindows(),(unsigned long)decoders_[3].RejectedWindows());
    if (n<0 || size_t(n)>=capacity) { buffer[0]='\0'; return 0; }
    return size_t(n);
}
