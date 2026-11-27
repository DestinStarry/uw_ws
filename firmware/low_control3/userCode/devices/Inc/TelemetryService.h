#pragma once
#include "usermain.h"
class TelemetryService {
public:
    void Handle();
    void TxCompleteISR() { busy_ = false; }
private:
    uint8_t buffer_[512] = {};
    volatile bool busy_ = false;
    bool resync_ = false;
    uint32_t sent_at_ = 0, last_report_ = 0, last_diagnostics_ = 0, last_rpm_summary_ = 0, sequence_ = 0;
};
extern TelemetryService g_telemetry;
