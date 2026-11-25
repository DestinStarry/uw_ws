#pragma once
#include "usermain.h"
#include "adc.h"
class VoltageMonitor : public Device {
public:
    void Init() override;
    void Handle() override;
    void Receive(uint8_t *, uint16_t) override {}
    float voltage = 0.f;
    bool Valid(uint32_t now) const { return valid_ && uint32_t(now-last_sample_) <= 500; }
    uint32_t Age(uint32_t now) const { return sampled_ ? uint32_t(now-last_sample_) : UINT32_MAX; }
private:
    bool Read(ADC_HandleTypeDef *adc, uint32_t channel, uint16_t &value);
    bool valid_ = false, sampled_ = false;
    uint32_t last_sample_ = 0, last_run_ = 0;
};
extern VoltageMonitor g_voltage;
