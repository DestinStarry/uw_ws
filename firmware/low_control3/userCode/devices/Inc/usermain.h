/*
 * usermain.h
 * Device 抽象基类 + 全局声明
 * 参考 Lower_Level_Controller 风格
 */

#ifndef USERMAIN_H
#define USERMAIN_H

#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "usart.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* ===== 系统参数 ===== */
#define CONTROL_FREQUENCY       100
#define LED_PERIOD              500

#ifndef PI
#define PI 3.14159265358979323846f
#endif

#ifdef __cplusplus

/* ===== Device 抽象基类 ===== */
class Device {
public:
    virtual void Init() = 0;
    virtual void Handle() = 0;
    virtual void Receive(uint8_t *data, uint16_t len) = 0;
    virtual ~Device() {}
};

#endif /* __cplusplus */

/* ===== 全局 ===== */
extern volatile bool system_init_flag;

#ifdef __cplusplus
extern "C" {
#endif

/* ===== 串口回调 ===== */
void System_UART_RxCallback(uint8_t *data, uint16_t len);

/* ===== CubeMX 生成函数 ===== */
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_ADC1_Init(void);
void MX_ADC3_Init(void);
void MX_I2C3_Init(void);
void MX_SPI1_Init(void);
void MX_SPI2_Init(void);
void MX_TIM1_Init(void);
void MX_TIM4_Init(void);
void MX_TIM5_Init(void);
void MX_TIM8_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART6_UART_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* USERMAIN_H */
