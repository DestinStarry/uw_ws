/*
 * Servo.h
 * 位置舵机（角度控制）— TIM8 PWM
 *   PC6 -> TIM8_CH1 (舵机1)
 *   PI6 -> TIM8_CH2 (舵机2)
 *   PI7 -> TIM8_CH3 (舵机3)
 *
 * PWM: 50Hz, 500~2500μs 对应 0°~360°
 *   500  = 0°
 *   1500 = 180°（中点）
 *   2500 = 360°
 *   PWM = 500 + (角度/360)*2000
 */

#ifndef SERVO_H
#define SERVO_H

#include "usermain.h"

#define SERVO_NUM       3
#define SERVO_MID       1500    // 中点（停止）
#define SERVO_MIN       500     // 最小脉宽
#define SERVO_MAX       2500    // 最大脉宽

class Servo : public Device {
public:
    int32_t data[SERVO_NUM];    // 当前 PWM 值

    void Init() override;
    void Handle() override;
    void Receive(uint8_t *data, uint16_t len) override;

    /** 直接设 PWM 值 500~2500 */
    void SetPWM(uint8_t ch, int32_t pulse);
    /** 前两路转向舵机：逻辑角度-90~90度，对应PWM 1000~2000。 */
    void SetAngle(uint8_t ch, float angle_deg);
};

#ifdef __cplusplus
extern Servo g_servo;
#endif

#endif /* SERVO_H */
