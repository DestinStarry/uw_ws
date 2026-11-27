// low_control3: Orin execution layer only. No depth/board-IMU polling.
#include "usermain.h"
#include "adc.h"
#include "spi.h"
#include "LED.h"
#include "Buzzer.h"
#include "Motor.h"
#include "Servo.h"
#include "CommUART1.h"
#include "CH9434.h"
#include "VoltageMonitor.h"
#include "TelemetryService.h"
#include "Safety.h"
#include "BoardConfig.h"
volatile bool system_init_flag=false;
static uint8_t rx_byte;
static volatile bool rearm_rx=false;
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart) {
    if (uart->Instance != USART6) return;
    // HAL_UART_IRQHandler may call RxCplt BEFORE ErrorCallback for RXNE+error.
    // Receive_IT clears ErrorCode, which would hide ORE/FE/NE/PE and change
    // the HAL error branch. Drop this byte and let HAL finish error handling;
    // the main loop rearms only after ErrorCallback has captured the flags.
    if (uart->ErrorCode != HAL_UART_ERROR_NONE) {
        rearm_rx = true;
        return;
    }
    if (system_init_flag) g_comm.RxByteISR(rx_byte);
    if (HAL_UART_Receive_IT(&huart6,&rx_byte,1) != HAL_OK) {
        rearm_rx=true;g_comm.RxErrorISR(uart->ErrorCode, true);
    }
}
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart) {
    if (uart->Instance == USART6) { g_comm.RxErrorISR(uart->ErrorCode);rearm_rx=true; }
}
extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart) {
    if (uart->Instance == USART6) g_telemetry.TxCompleteISR();
}
extern "C" void System_UART_RxCallback(uint8_t *,uint16_t) { /* Obsolete depth broadcast removed. */ }
int main() {
    HAL_Init();SystemClock_Config();
    MX_GPIO_Init();MX_TIM1_Init();MX_TIM8_Init();
    g_motor.Init();g_servo.Init();Safety_Init();Watchdog_Init();
    MX_ADC1_Init();MX_ADC3_Init();MX_SPI2_Init();
    MX_USART6_UART_Init();MX_TIM4_Init();MX_TIM5_Init();
    HAL_NVIC_SetPriority(USART6_IRQn,1,0);
    // SysTick can preempt UART floods and apply the independent deadman stop.
    HAL_NVIC_SetPriority(SysTick_IRQn,0,0);
    g_led.Init();g_buzzer.Init();g_comm.Init();g_voltage.Init();
    CH9434::ch9434.Init();
    g_buzzer.RequestBeep(2); // Nonblocking startup indication.
    system_init_flag=true;
    if (HAL_UART_Receive_IT(&huart6,&rx_byte,1) != HAL_OK) rearm_rx=true;
    uint32_t last_slow=HAL_GetTick();
    while (true) {
        if (rearm_rx) {
            const uint32_t mask=__get_PRIMASK();__disable_irq();
            HAL_UART_AbortReceive(&huart6);
            __HAL_UART_CLEAR_OREFLAG(&huart6);
            rearm_rx=HAL_UART_Receive_IT(&huart6,&rx_byte,1) != HAL_OK;
            if (rearm_rx) g_comm.RxErrorISR(huart6.ErrorCode, true);
            __set_PRIMASK(mask);
        }
        // Highest priority task every loop. Only valid complete CTRL feeds deadman.
        g_comm.Handle();
        const uint32_t now=HAL_GetTick();
        if (uint32_t(now-last_slow) >= kMainCycleMs) {
            last_slow=now;
            CH9434::ch9434.Handle();
            g_comm.Handle();
            g_voltage.Handle();
            g_comm.Handle();
            g_buzzer.Handle();g_led.Handle();
        }
        g_telemetry.Handle();
        Watchdog_Refresh(); // If any task hangs or IRQs stay disabled, IWDG resets.
    }
}
