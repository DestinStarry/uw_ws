/*
 * Buzzer.cpp
 * 蜂鸣器 — 仅开机响，常态静音
 *   PB8 -> TIM4_CH3
 */

#include "Buzzer.h"

extern TIM_HandleTypeDef htim4;

Buzzer g_buzzer;

/* ===== 开/关 ===== */
float Buzzer::bsp_BuzzerOn(float _freq, float _targetVolPct)
{
    /* 计算 PSC 使 ARR 落在 ~10000（保证 16 位不溢出） */
    uint32_t psc = BUZZER_CLOCK_FREQUENCY / (uint32_t)_freq / 10000;
    if (psc > 65535) psc = 65535;
    uint16_t arr = (uint16_t)(BUZZER_CLOCK_FREQUENCY / ((psc + 1) * (uint32_t)_freq));
    if (arr == 0) arr = 1;
    uint16_t cpr = arr / 2;     /* 50% 占空比 → 最大音量 */

    /* 用 HAL 配置并启动 PWM */
    HAL_TIM_PWM_Stop(&BUZZER_CLOCK, BUZZER_CLOCK_CHANNEL);
    __HAL_TIM_SET_PRESCALER(&BUZZER_CLOCK, (uint16_t)psc);
    __HAL_TIM_SET_AUTORELOAD(&BUZZER_CLOCK, arr);
    __HAL_TIM_SET_COUNTER(&BUZZER_CLOCK, 0);
    HAL_TIM_Base_Start(&BUZZER_CLOCK);
    HAL_TIM_PWM_Start(&BUZZER_CLOCK, BUZZER_CLOCK_CHANNEL);
    __HAL_TIM_SET_COMPARE(&BUZZER_CLOCK, BUZZER_CLOCK_CHANNEL, cpr);

    buzzerWorkingFlag = 1;
    return (float)cpr / (float)arr * 2.0f;
}

void Buzzer::bsp_BuzzerOff()
{
    if (buzzerWorkingFlag) {
        buzzerWorkingFlag = 0;
        HAL_TIM_PWM_Stop(&BUZZER_CLOCK, BUZZER_CLOCK_CHANNEL);
    }
}

/* ===== Device 接口 ===== */
void Buzzer::Init()
{
    BuzzerFlag = 0;
    HAL_TIM_Base_Start(&BUZZER_CLOCK);
    HAL_TIM_PWM_Stop(&BUZZER_CLOCK, BUZZER_CLOCK_CHANNEL);
}

void Buzzer::Handle()
{
    /* 非阻塞驱动鸣响 */
    if (beep_request == 0) return;

    uint32_t now = HAL_GetTick();

    if (beep_phase == 1) {
        /* 正在响 — 等待 120ms 后关闭 */
        if (now - beep_tick >= 120) {
            bsp_BuzzerOff();
            beep_phase = 2;
            beep_tick = now;
        }
    } else if (beep_phase == 2) {
        /* 已关闭 — 等待 80ms 后下一次 */
        if (now - beep_tick >= 80) {
            beep_count++;
            if (beep_count < beep_request) {
                /* 继续下一次 */
                bsp_BuzzerOn(800, 1);
                beep_phase = 1;
                beep_tick = now;
            } else {
                /* 全部完成 */
                beep_request = 0;
                beep_count = 0;
                beep_phase = 0;
            }
        }
    }
}

void Buzzer::Receive(uint8_t *data, uint16_t len)
{
    (void)data; (void)len;
}

/* ===== 开机响几声（阻塞，仅开机用） ===== */
void Buzzer::StartupBeep()
{
    for (int i = 0; i < 4; i++) {
        bsp_BuzzerOn(800, 1);
        HAL_Delay(120);
        bsp_BuzzerOff();
        HAL_Delay(80);
    }
}

/* ===== 非阻塞请求鸣响（在 Handle 中执行） ===== */
void Buzzer::RequestBeep(uint8_t times)
{
    if (times == 0) return;
    beep_request = times;
    beep_count = 0;
    beep_phase = 1;
    beep_tick = HAL_GetTick();
    bsp_BuzzerOn(800, 1);
}

/* ===== C 接口 ===== */
void Buzzer_StartupBeep(void)
{
    g_buzzer.StartupBeep();
}
