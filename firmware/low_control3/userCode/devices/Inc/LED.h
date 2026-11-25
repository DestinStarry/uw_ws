/*
 * LED.h
 * RGB LED — 常态绿灯
 *   PH10 -> TIM5_CH1 (Blue)
 *   PH11 -> TIM5_CH2 (Green)
 *   PH12 -> TIM5_CH3 (Red)
 */

#ifndef LED_H
#define LED_H

#include "usermain.h"

class LED : public Device {
public:
    void Init() override;
    void Handle() override;
    void Receive(uint8_t *data, uint16_t len) override;

    void aRGB_led_show(uint32_t aRGB);
    void StartupDemo(void);
};

#ifdef __cplusplus
extern "C" {
#endif
void LED_StartupDemo(void);
#ifdef __cplusplus
}

extern LED g_led;
#endif

#endif /* LED_H */
