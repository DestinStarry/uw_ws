/*
 * CH9434.cpp
 * SPI 转 4 路 UART 芯片 CH9434 驱动 — SPI2 轮询方式
 * 参考沁恒 CH9434EVT 官方例程（CH32F103 标准库）移植到 STM32F407 HAL
 */

#include "CH9434.h"

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
    if (!_io_ok) return 0xff;
    if (HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1, kSpiTimeoutMs) != HAL_OK) {
        _io_ok = false; ++_spi_errors; return 0xff;
    }
    spi_delay();
    return rx;
}

/* 帧内字节间小延时（对应官方例程 1us 量级），保证 CH9434 时序稳定 */
void CH9434::spi_delay(void)
{
    volatile uint32_t i;
    for (i = 0; i < 300; i++) { __NOP(); }   /* 168MHz 下约 1.8us */
}

uint8_t CH9434::reg_read(uint8_t addr)
{
    uint8_t rx;
    cs(0);
    spi_delay();           /* CS 建立时间 */
    spi_byte(addr & 0x7F); /* 读命令 */
    rx = spi_byte(0xFF);   /* 读回数据 */
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

    /* 读 LCR，置 DLAB，配置 8N1 */
    lcr = reg_read(addr + CH9434_UARTx_LCR_ADD);
    lcr |= CH9434_UARTx_BIT_DLAB;
    lcr &= ~0x03; lcr |= 0x03;        /* 8 位数据 */
    lcr &= ~(1<<2);                   /* 1 停止位 */
    lcr &= ~(1<<3); lcr &= ~(3<<4);   /* 无校验 */
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

uint16_t CH9434::get_rx_fifo_data(uint8_t idx, uint8_t *buf, uint16_t len)
{
    uint8_t addr = CH9434_REG_OP_READ | (0x10 * idx + CH9434_UARTx_RBR_ADD);
    const uint32_t started = HAL_GetTick();
    uint16_t got = 0;
    while (got < len && _io_ok && uint32_t(HAL_GetTick() - started) < kSpiPollBudgetMs) {
        cs(0); spi_byte(addr); buf[got] = spi_byte(0xff); cs(1);
        if (_io_ok) ++got;
    }
    return got;
}

void CH9434::push_rx(uint8_t idx, uint8_t d)
{
    uint16_t next = (uint16_t)((rx_head[idx] + 1) % RX_BUF_SIZE);
    if (next != rx_tail[idx]) {   /* 非满 */
        rx_buf[idx][rx_head[idx]] = d;
        rx_head[idx] = next;

    }

    ++rx_total[idx];
    decoders_[idx].Feed(d, HAL_GetTick());
}
int32_t CH9434::GetRPM(uint8_t idx) { return idx < 4 ? decoders_[idx].Rpm() : 0; }
bool CH9434::RpmValid(uint8_t idx, uint32_t now) const {
    return idx < 4 && Ready() && decoders_[idx].Valid(now);
}
uint32_t CH9434::RpmAge(uint8_t idx, uint32_t now) const {
    return idx < 4 ? decoders_[idx].Age(now) : UINT32_MAX;
}

/* SPI 写读回自检 (无需硬件短接):
 *   对 UART0 的 LCR 寄存器 先读原值 → 写 0x5A → 读回 → 恢复原值.
 *   读回 0x5A → SPI 写/读双向都通 (问题在别处)
 *   读回 0xFF → SPI 读不通 (芯片无响应 / CS/MISO 接线 / 供电问题)
 */
uint8_t CH9434::writeback_test(void)
{
    uint8_t addr = 0x10 * 0 + CH9434_UARTx_LCR_ADD;   /* UART0 LCR = 0x03 */
    uint8_t orig = reg_read(addr);
    reg_write(addr, 0x5A);
    uint8_t chk = reg_read(addr);
    reg_write(addr, orig);   /* 恢复原值 */
    return chk;
}

/* ===== Device 接口 ===== */

void CH9434::Init()
{
    _retry_tick = HAL_GetTick();
    _initialized = false; _io_ok = true;
    for (uint8_t idx = 0; idx < 4; ++idx) {
        decoders_[idx] = low::RpmDecoder(kEscPolePairs[idx]);
        rx_head[idx] = rx_tail[idx] = 0;
    }
    GPIO_InitTypeDef gpio = {};

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

    /* 初始化芯片时钟 */
    clk_init();

    /* 配置 4 路 UART：115200/8N1/FIFO/中断 */
    for (uint8_t idx = 0; idx < 4; idx++) {
        uart_config(idx, CH9434_UART_BAUD);
        uart_fifo(idx);
        uart_flow(idx, 0);        /* 默认关闭硬件流控 */
        uart_irq(idx);
        uart_irq_open(idx);
    }

    /* 上电写读回自检 (0x5A=SPI双向通; 0xFF=读不通) */
    _loopback = writeback_test();

    _initialized = _io_ok && _loopback == 0x5a;
}

void CH9434::Handle()
{
    if (!Ready()) {
        for (auto &decoder : decoders_) decoder.Reset();
        if (uint32_t(HAL_GetTick() - _retry_tick) >= 1000) Init();
        return;
    }
    // Round robin: one channel per 10ms slot; bounded FIFO draining and SPI time.
    const uint8_t idx = _next_channel;
    _next_channel = uint8_t((_next_channel + 1) % 4);
    const uint16_t pending = get_rx_fifo_len(idx);
    if (pending > 4096) { _io_ok = false; ++_spi_errors; }
    if (_io_ok && pending) {
        uint8_t bytes[128];
        const uint16_t wanted = pending > sizeof(bytes) ? sizeof(bytes) : pending;
        const uint16_t got = get_rx_fifo_data(idx, bytes, wanted);
        if (pending > sizeof(bytes)) {
            // Backlogged data cannot be asserted fresh: discard while catching up.
            decoders_[idx].Reset();
        } else if (_io_ok) {
            for (uint16_t i = 0; i < got; ++i) push_rx(idx, bytes[i]);
        }
    }
    if (!_io_ok) for (auto &decoder : decoders_) decoder.Reset();
}

void CH9434::Receive(uint8_t *data, uint16_t len)
{
    (void)data; (void)len;   /* 上位机通用命令不归 CH9434 处理，预留 */
}

/* ===== 用户 API ===== */

uint16_t CH9434::Send(uint8_t idx, const uint8_t *data, uint16_t len)
{
    if (!_initialized || idx > 3) return 0;
    uint8_t addr = CH9434_REG_OP_WRITE | (0x10 * idx + CH9434_UARTx_THR_ADD);
    for (uint16_t i = 0; i < len; i++) {
        cs(0);
        spi_byte(addr);
        spi_byte(data[i]);
        cs(1);
    }
    return len;
}

uint16_t CH9434::Recv(uint8_t idx, uint8_t *buf, uint16_t maxlen)
{
    uint16_t n = 0;
    if (idx > 3) return 0;
    while (n < maxlen && rx_tail[idx] != rx_head[idx]) {
        buf[n++] = rx_buf[idx][rx_tail[idx]];
        rx_tail[idx] = (uint16_t)((rx_tail[idx] + 1) % RX_BUF_SIZE);
    }
    return n;
}

uint16_t CH9434::Available(uint8_t idx)
{
    if (idx > 3) return 0;
    return (uint16_t)((rx_head[idx] - rx_tail[idx] + RX_BUF_SIZE) % RX_BUF_SIZE);
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
