/*
 * LED.cpp
 * RGB LED — 常态绿灯，开机闪一下
 *
 * TIM5_CH3 = Red (PH12), CH2 = Green (PH11), CH1 = Blue (PH10)
 */

#include "LED.h"

extern TIM_HandleTypeDef htim5;

LED g_led;

/* ===== 显示 ARGB 颜色 ===== */
void LED::aRGB_led_show(uint32_t aRGB)
{
    uint8_t alpha = (aRGB & 0xFF000000) >> 24;
    uint16_t red   = ((aRGB & 0x00FF0000) >> 16) * alpha;
    uint16_t green = ((aRGB & 0x0000FF00) >> 8)  * alpha;
    uint16_t blue  = ((aRGB & 0x000000FF) >> 0)  * alpha;

    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, red);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, green);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, blue);
}

/* ===== Device 接口 ===== */
void LED::Init()
{
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);

    /* 默认绿灯 */
    aRGB_led_show(0xFF002200);
}

void LED::Handle()
{
    /* PWM 自动保持，无需操作 */
}

void LED::Receive(uint8_t *data, uint16_t len)
{
    (void)data; (void)len;
}

/* ===== 开机演示：红→绿→蓝→绿 ===== */
void LED::StartupDemo()
{
    aRGB_led_show(0xFFFF0000); HAL_Delay(300);   // 红
    aRGB_led_show(0xFF00FF00); HAL_Delay(300);   // 绿
    aRGB_led_show(0xFF0000FF); HAL_Delay(300);   // 蓝
    aRGB_led_show(0xFF002200);                    // 回到绿灯（低亮度）
}

/* ===== C 接口 ===== */
void LED_StartupDemo(void) { g_led.StartupDemo(); }
