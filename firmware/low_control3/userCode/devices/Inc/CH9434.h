/*
 * CH9434.h
 * SPI 转 4 路 UART 芯片 CH9434 驱动 — SPI2 轮询方式
 * 参考沁恒 CH9434EVT 官方例程移植到 STM32F407 HAL
 *
 * 硬件连接（按实际接线修改）：
 *   SPI2_SCK   = PB13
 *   SPI2_MISO  = PB14   (注意: 不是 PI2!)
 *   SPI2_MOSI  = PB15
 *   CH9434_CS  = PB12   (片选，软件控制)
 *   CH9434_INT#= PA0    (中断查询，输入上拉)
 */

#ifndef CH9434_H
#define CH9434_H

#include "usermain.h"
#include "spi.h"
#include "RpmDecoder.h"
#include "BoardConfig.h"

/* ===== CH9434 引脚（按实际接线修改） ===== */
#define CH9434_CS_Pin        GPIO_PIN_12
#define CH9434_CS_GPIO_Port  GPIOB
#define CH9434_INT_Pin       GPIO_PIN_0
#define CH9434_INT_GPIO_Port GPIOA

/* ===== 串口参数 ===== */
#define CH9434_UART_BAUD     115200   /* 波特率（内部 32MHz 时钟） */
#define CH9434_UART_8N1      1        /* 固定 8N1 */

/* ===== 电调遥测协议解析 ===== */
#define CH9434_FRAME_LEN   10   /* 电调遥测帧固定 10 字节 */
#define ESC_POLE_PAIRS     7    /* 电机极对数 (机械转速 = ERpm/极对数×100) */

/* ===== 测试回显：CH9434 收到数据后经 UART6 上报（1=开，0=关） ===== */
#define CH9434_ECHO_UART6   0   /* 已取消: 调试用十六进制回显 */

/* ===== 寄存器定义（参考 CH9434EVT/CH9434.H） ===== */
#define CH9434_REG_OP_WRITE     0x80
#define CH9434_REG_OP_READ      0x00

#define CH9434_UARTx_RBR_ADD    0
#define CH9434_UARTx_THR_ADD    0
#define CH9434_UARTx_IER_ADD    1
#define CH9434_UARTx_IIR_ADD    2
#define CH9434_UARTx_FCR_ADD    2
#define CH9434_UARTx_LCR_ADD    3
#define CH9434_UARTx_BIT_DLAB   (1<<7)
#define CH9434_UARTx_MCR_ADD    4
#define CH9434_UARTx_LSR_ADD    5
#define CH9434_UARTx_MSR_ADD    6
#define CH9434_UARTx_DLL_ADD    0
#define CH9434_UARTx_DLM_ADD    1

#define CH9434_FIFO_CTRL_ADD    0x42
#define CH9434_FIFO_CTRL_TR     (1<<4)
#define CH9434_FIFO_CTRL_L_ADD  0x43
#define CH9434_FIFO_CTRL_H_ADD  0x44
#define CH9434_CLK_CTRL_CFG_ADD 0x48

/* IIR 中断标识 */
#define CH9434_IIR_MODEM        0x00
#define CH9434_IIR_THR          0x02
#define CH9434_IIR_RX_DATA      0x04
#define CH9434_IIR_LINE         0x06
#define CH9434_IIR_RX_TIMEOUT   0x0C

class CH9434 : public Device {
public:
    void Init() override;
    void Handle() override;
    void Receive(uint8_t *data, uint16_t len) override;

    /* ===== 用户 API ===== */
    uint16_t Send(uint8_t idx, const uint8_t *data, uint16_t len);  /* 向指定串口发送 */
    uint16_t Recv(uint8_t idx, uint8_t *buf, uint16_t maxlen);      /* 从指定串口取接收数据 */
    uint16_t Available(uint8_t idx);                                /* 指定串口待读字节数 */
    uint32_t GetRxCount(uint8_t idx);                               /* 指定串口累计接收字节数 */
    bool RpmValid(uint8_t idx, uint32_t now) const;
    uint32_t RpmAge(uint8_t idx, uint32_t now) const;
    bool Ready() const { return _initialized && _io_ok; }
    uint32_t SpiErrors() const { return _spi_errors; }
    int32_t  GetRPM(uint8_t idx);                                   /* 最近一帧机械转速 (RPM) */
    uint16_t GetRxFifoLenDbg(uint8_t idx);   /* 调试：直接读指定串口 RxFIFO 长度 */
    uint8_t  GetLSRDbg(uint8_t idx);         /* 调试：读指定串口线路状态寄存器 (SPI 是否通的判据) */
    uint8_t  GetWritebackDbg(void);          /* 调试：SPI 写读回自检结果 (0x5A=双向通, 0xFF=读不通) */

    static CH9434 ch9434;

private:
    /* 底层 */
    void cs(uint8_t level);
    uint8_t spi_byte(uint8_t tx);
    uint8_t reg_read(uint8_t addr);
    void reg_write(uint8_t addr, uint8_t val);
    void spi_delay(void);

    /* 芯片配置 */
    void clk_init(void);
    void uart_config(uint8_t idx, uint32_t bps);
    void uart_fifo(uint8_t idx);
    void uart_flow(uint8_t idx, uint8_t en);
    void uart_irq(uint8_t idx);
    void uart_irq_open(uint8_t idx);

    /* 收发 */
    uint16_t get_rx_fifo_len(uint8_t idx);
    uint16_t get_rx_fifo_data(uint8_t idx, uint8_t *buf, uint16_t len);
    void push_rx(uint8_t idx, uint8_t d);

    /* 电调遥测帧解析 */
    low::RpmDecoder decoders_[4];

    /* SPI 写读回自检 */
    uint8_t writeback_test(void);

    /* 每路串口接收环形缓冲 */
    static const uint16_t RX_BUF_SIZE = 256;
    uint8_t  rx_buf[4][RX_BUF_SIZE];
    uint16_t rx_head[4] = {0, 0, 0, 0};
    uint16_t rx_tail[4] = {0, 0, 0, 0};
    uint32_t rx_total[4] = {0, 0, 0, 0};   /* 累计接收字节数 */

    uint32_t _sys_frequency = 32000000;
    bool _initialized = false, _io_ok = true;
    uint32_t _spi_errors = 0, _retry_tick = 0;
    uint8_t _next_channel = 0;
    uint8_t _loopback = 0xFF;   /* 上电时写读回自检结果 */
};

#endif /* CH9434_H */
