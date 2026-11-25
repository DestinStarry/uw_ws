/*
 * Buzzer.h
 * 频率可控 PWM 蜂鸣器 — 抄 Lower_Level_Controller
 *   PD14 -> TIM4_CH3
 */

#ifndef BUZZER_H
#define BUZZER_H

#include "usermain.h"

#define BUZZER_CLOCK_FREQUENCY  84000000
#define BUZZER_CLOCK            htim4
#define BUZZER_CLOCK_CHANNEL    TIM_CHANNEL_3
#define REFERENCE_MAX_VOL_CCR   5000

class Buzzer : public Device {
public:
    uint32_t BuzzerFlag = 0;
    uint8_t  buzzerWorkingFlag = 0;

    void Init() override;
    void Handle() override;
    void Receive(uint8_t *data, uint16_t len) override;

    float bsp_BuzzerOn(float _freq, float _targetVolPct);
    void  bsp_BuzzerOff();

    /** 开机演示：哔哔哔三声 */
    void StartupBeep(void);

    /** 非阻塞请求哔几声（由 Handle 驱动） */
    void RequestBeep(uint8_t times);

private:
    uint8_t  beep_request = 0;     /* 请求鸣响次数 */
    uint8_t  beep_count  = 0;      /* 已完成次数 */
    uint32_t beep_tick   = 0;      /* 计时 */
    uint8_t  beep_phase  = 0;      /* 0=空闲, 1=响, 2=停 */
};

/* ===== C 接口 ===== */
#ifdef __cplusplus
extern "C" {
#endif
void Buzzer_StartupBeep(void);
#ifdef __cplusplus
}

extern Buzzer g_buzzer;
#endif

#endif /* BUZZER_H */
