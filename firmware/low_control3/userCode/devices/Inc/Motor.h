/*
 * Motor.h
 * 推进器 4 路 — TIM1 PWM
 *   PE9  -> TIM1_CH1 (电机1)
 *   PE11 -> TIM1_CH2 (电机2)
 *   PE13 -> TIM1_CH3 (电机3)
 *   PE14 -> TIM1_CH4 (电机4)
 *
 * PWM: 50Hz, 1100~1900μs
 *   1100 = 反向最大推力
 *   1500 = 停止
 *   1900 = 正向最大推力
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "usermain.h"

#define MOTOR_NUM       4
#define MOTOR_MID       1500    // 停止
#define MOTOR_MIN       1100    // 反向最大
#define MOTOR_MAX       1900    // 正向最大

class Motor : public Device {
public:
    int32_t data[MOTOR_NUM];    // 当前 PWM 值

    void Init() override;
    void Handle() override;
    void Receive(uint8_t *data, uint16_t len) override;

    /** 直接设 PWM 1100~1900 */
    void SetPWM(uint8_t ch, int32_t pulse);
    /** 设推力 -1.0~1.0（负=反向，正=正向，0=停） */
    void SetThrust(uint8_t ch, float thrust);
};

#ifdef __cplusplus
extern Motor g_motor;
#endif

#endif /* MOTOR_H */