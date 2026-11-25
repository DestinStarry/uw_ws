/*
 * DepthSensor.h
 * 深度传感器 — 解算板通过 UART1 发送
 * 格式: "Depth:XX.XXm Temp:XX.XXC\r\n"
 */

#ifndef DEPTHSENSOR_H
#define DEPTHSENSOR_H

#include "usermain.h"

class DepthSensor : public Device {
public:
    float depth = 0.0f;    // 深度 (m)
    float temp  = 0.0f;    // 温度 (°C)
    bool  fresh = false;   // 是否有新数据
    uint32_t last_update_ms = 0;

    /* 深度计实测约 2Hz（500ms 一帧）。取值与上位机的
     * orange/config.py:DEPTH_STALE_TIMEOUT 保持一致（3 个采样周期），
     * 避免两侧对“有效帧”的判定标准不一致。
     * 注意：UART6 已不再上报 DEPTH_VALID，当前无调用者，仅供内部使用。 */
    bool IsValid(uint32_t now, uint32_t timeout_ms = 1500) const {
        return fresh && (uint32_t)(now - last_update_ms) <= timeout_ms;
    }

    void Init() override;
    void Handle() override;
    void Receive(uint8_t *data, uint16_t len) override;
};

extern DepthSensor g_depth;

#endif
