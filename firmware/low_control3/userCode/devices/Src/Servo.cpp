/*
 * Servo.cpp
 * 360°位置舵机 — TIM8 PWM；推进器转向只使用中心左右各90°。
 *
 * TIM8: 50Hz, Prescaler=167, Period=19999
 *   168MHz / (167+1) / (19999+1) = 50Hz
 *   脉宽 500~2500 → 占空比 500~2500
 */

#include "Servo.h"
#include <cmath>

extern TIM_HandleTypeDef htim8;

Servo g_servo;

/* ===== 设 PWM ===== */
void Servo::SetPWM(uint8_t ch, int32_t pulse)
{
    if (ch >= SERVO_NUM) return;
    if (pulse < SERVO_MIN) pulse = SERVO_MIN;
    if (pulse > SERVO_MAX) pulse = SERVO_MAX;
    data[ch] = pulse;
}

/* ===== 设逻辑角度：0°=PWM1500，±90°=PWM1000/2000 ===== */
void Servo::SetAngle(uint8_t ch, float angle_deg)
{
    if (ch >= SERVO_NUM) return;
    if (!std::isfinite(angle_deg)) angle_deg = 0.f;
    if (angle_deg > 90.0f) angle_deg = 90.0f;
    if (angle_deg < -90.0f) angle_deg = -90.0f;
    data[ch] = SERVO_MID + (int32_t)(angle_deg * (500.0f / 90.0f));
}

/* ===== Device 接口 ===== */
void Servo::Init()
{
    /* 三个舵机初始都停止 */
    for (int i = 0; i < SERVO_NUM; i++)
        data[i] = SERVO_MID;

    __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,data[0]);
    __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,data[1]);
    __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,data[2]);
    if (HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1)!=HAL_OK ||
        HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2)!=HAL_OK ||
        HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3)!=HAL_OK) Error_Handler();

}

void Servo::Handle()
{
    /* 刷新三个通道 */
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, data[0]);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, data[1]);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, data[2]);
}

void Servo::Receive(uint8_t *rx, uint16_t len)
{
    (void)rx; (void)len;
    /* 预留：串口命令控制 */
}
