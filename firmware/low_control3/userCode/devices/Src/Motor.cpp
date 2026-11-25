/*
 * Motor.cpp
 * 推进器 4 路 — TIM1 PWM
 *
 * TIM1: 50Hz, Prescaler=167, Period=19999
 *   168MHz / (167+1) / (19999+1) = 50Hz
 *   脉宽 1100~1900
 */

#include "Motor.h"
#include "BoardConfig.h"
#include <cmath>

extern TIM_HandleTypeDef htim1;

Motor g_motor;

// Direction inversion is centralized in BoardConfig.h.
/* ===== 设 PWM（原始脉宽，不做方向修正，供单通道测试用） ===== */
void Motor::SetPWM(uint8_t ch, int32_t pulse)
{
    if (ch >= MOTOR_NUM) return;
    if (pulse < MOTOR_MIN) pulse = MOTOR_MIN;
    if (pulse > MOTOR_MAX) pulse = MOTOR_MAX;
    data[ch] = pulse;
}

/* ===== 设推力 ===== */
void Motor::SetThrust(uint8_t ch, float thrust)
{
    if (ch >= MOTOR_NUM) return;
    if (!std::isfinite(thrust)) thrust = 0.f;
    if (kMotorInvert[ch]) thrust = -thrust;   /* 电调反相接反修正 */
    if (thrust > 1.0f)  thrust = 1.0f;
    if (thrust < -1.0f) thrust = -1.0f;
    /* thrust: -1.0~1.0 → PWM: 1100~1900 */
    data[ch] = MOTOR_MID + (int32_t)(thrust * 400.0f);
}

/* ===== Device 接口 ===== */
void Motor::Init()
{
    /* 四个推进器都停止 */
    for (int i = 0; i < MOTOR_NUM; i++)
        data[i] = MOTOR_MID;

    // Set neutral BEFORE enabling the first output channel.
    TIM1->CCR1 = data[0];TIM1->CCR2 = data[1];TIM1->CCR3 = data[2];TIM1->CCR4 = data[3];
    if (HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1)!=HAL_OK ||
        HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2)!=HAL_OK ||
        HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3)!=HAL_OK ||
        HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4)!=HAL_OK) Error_Handler();

}

void Motor::Handle()
{
    /* 刷新四通道 */
    TIM1->CCR1 = data[0];
    TIM1->CCR2 = data[1];
    TIM1->CCR3 = data[2];
    TIM1->CCR4 = data[3];
}

void Motor::Receive(uint8_t *rx, uint16_t len)
{
    (void)rx; (void)len;
    /* 预留：串口命令控制 */
}
