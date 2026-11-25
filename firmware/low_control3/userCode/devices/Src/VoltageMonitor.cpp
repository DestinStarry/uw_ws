#include "VoltageMonitor.h"
#include "BoardConfig.h"
#include <cmath>
VoltageMonitor g_voltage;
bool VoltageMonitor::Read(ADC_HandleTypeDef *adc, uint32_t channel, uint16_t &value) {
    ADC_ChannelConfTypeDef config = {};
    config.Channel = channel; config.Rank = 1;
    config.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    if (HAL_ADC_ConfigChannel(adc, &config) != HAL_OK) return false;
    if (HAL_ADC_Start(adc) != HAL_OK) return false;
    const bool success = HAL_ADC_PollForConversion(adc, 2) == HAL_OK;
    if (success) value = uint16_t(HAL_ADC_GetValue(adc));
    const bool stopped = HAL_ADC_Stop(adc) == HAL_OK;
    return success && stopped;
}
void VoltageMonitor::Init() { valid_ = sampled_ = false; last_run_ = HAL_GetTick() - kVoltagePeriodMs; }
void VoltageMonitor::Handle() {
    const uint32_t now = HAL_GetTick();
    if (uint32_t(now-last_run_) < kVoltagePeriodMs) return;
    last_run_ = now;
    uint16_t ref = 0, raw = 0;
    valid_ = false;
    // Discard first VREF reading to allow internal reference/channel settling.
    if (!Read(&hadc1, ADC_CHANNEL_VREFINT, ref) ||
        !Read(&hadc1, ADC_CHANNEL_VREFINT, ref) || !ref || ref >= 4095) return;
    // Preserve original board calibration assumption (nominal VREFINT=1.2V).
    const float scale = 1.2f / float(ref);
    const float supply = scale * 4095.f;
    if (supply < 2.4f || supply > 3.6f) return;
    uint32_t sum = 0;
    for (unsigned i = 0; i < 4; ++i) {
        if (!Read(&hadc3, ADC_CHANNEL_8, raw) || raw >= 4095) return;
        sum += raw;
    }
    voltage = float(sum) * .25f * scale * (222.f / 22.f);
    if (!std::isfinite(voltage) || voltage < 0 || voltage > 40) return;
    last_sample_ = HAL_GetTick(); sampled_ = valid_ = true;
}
