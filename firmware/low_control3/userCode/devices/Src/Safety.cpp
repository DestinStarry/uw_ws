#include "Safety.h"
#include "usermain.h"
#include "BoardConfig.h"
#include "ControlProtocol.h"
static volatile bool ready = false, active = false;
static volatile uint32_t accepted_tick = 0, pending_fault = 0, last_reason = SAFETY_BOOT;
#if LOW_CONTROL_ENABLE_IWDG
static IWDG_HandleTypeDef watchdog;
#endif
// No HAL calls, formatting, allocation or parsing in the IRQ stop path.
static void neutral_pwm() {
    if (!ready) return;
    TIM1->CCR1 = 1500; TIM1->CCR2 = 1500; TIM1->CCR3 = 1500; TIM1->CCR4 = 1500;
    TIM8->CCR1 = 1500; TIM8->CCR2 = 1500; TIM8->CCR3 = 1500;
}
extern "C" void Safety_Init() {
    active = false; pending_fault = 0; last_reason = SAFETY_BOOT;
    ready = true; neutral_pwm();
}
extern "C" void Safety_Accept(uint32_t tick) { accepted_tick = tick; active = true; }
extern "C" void Safety_Stop(uint32_t reason) {
    active = false; last_reason = reason; neutral_pwm();
}
extern "C" void Safety_FaultISR(uint32_t reason) {
    const uint32_t mask = __get_PRIMASK(); __disable_irq();
    pending_fault |= reason; Safety_Stop(reason);
    __DMB(); __set_PRIMASK(mask);
}
extern "C" void Safety_Tick(uint32_t now) {
    if (ready && active && uint32_t(now - accepted_tick) > low::kControlTimeoutMs)
        Safety_FaultISR(SAFETY_TIMEOUT);
}
extern "C" uint32_t Safety_TakeFault() {
    const uint32_t result = pending_fault; pending_fault = 0; return result;
}
extern "C" uint32_t Safety_PendingFault() { return pending_fault; }
extern "C" uint32_t Safety_LastReason() { return last_reason; }
extern "C" void Watchdog_Init() {
#if LOW_CONTROL_ENABLE_IWDG
    // Nominal ~2 s at 32 kHz LSI. Actual timeout depends on oscillator tolerance.
    watchdog.Instance = IWDG;
    watchdog.Init.Prescaler = IWDG_PRESCALER_64;
    watchdog.Init.Reload = 999;
    if (HAL_IWDG_Init(&watchdog) != HAL_OK) Error_Handler();
    // Do not freeze the watchdog at debug breakpoints with live actuators.
#endif
}
extern "C" void Watchdog_Refresh() {
#if LOW_CONTROL_ENABLE_IWDG
    HAL_IWDG_Refresh(&watchdog); // Main loop only, never a timer/IRQ.
#endif
}
