#include "TelemetryService.h"
#include "TelemetryProtocol.h"
#include "BoardConfig.h"
#include "VoltageMonitor.h"
#include "CH9434.h"
#include "CommUART1.h"
#include "Safety.h"
#include <cmath>
TelemetryService g_telemetry;
void TelemetryService::Handle() {
    const uint32_t now=HAL_GetTick();
    if (busy_) {
        if (uint32_t(now-sent_at_) <= 100) return;
        // Only abort TX; never cancel the independent command RX.
        HAL_UART_AbortTransmit(&huart6);
        busy_=false;resync_=true;
    }
    if (uint32_t(now-last_report_) < kTelemetryPeriodMs) return;
    low::TelemetrySnapshot sample;
    sample.voltage.valid=g_voltage.Valid(now);
    if (sample.voltage.valid) sample.voltage.value=int32_t(std::lround(g_voltage.voltage*100.f));
    sample.voltage.age=g_voltage.Age(now);
    for (uint8_t i=0;i<4;++i) {
        sample.rpm[i].valid=CH9434::ch9434.RpmValid(i,now);
        sample.rpm[i].value=CH9434::ch9434.GetRPM(i);
        sample.rpm[i].age=CH9434::ch9434.RpmAge(i,now);
    }
    sample.sequence=sequence_;sample.now=now;
    sample.active=g_comm.Active() && !Safety_PendingFault();
    sample.needs_neutral=g_comm.NeedsNeutral() || Safety_PendingFault();
    sample.stop_reason=Safety_LastReason();sample.rejected=g_comm.Rejected();
    sample.ch_ready=CH9434::ch9434.Ready();
    sample.spi_errors=CH9434::ch9434.SpiErrors();
    size_t length=0;
    if (resync_) { buffer_[length++]='\r';buffer_[length++]='\n'; }
    const size_t payload=low::FormatTelemetry(reinterpret_cast<char *>(buffer_+length),sizeof(buffer_)-length,sample);
    if (!payload) return;
    length+=payload;
    const bool diagnostics_due=uint32_t(now-last_diagnostics_) >= kDiagnosticsPeriodMs;
    bool diagnostics_added=false;
    if (diagnostics_due) {
        const size_t extra=low::FormatDiagnostics(reinterpret_cast<char *>(buffer_+length),sizeof(buffer_)-length,sample);
        if (extra) { length+=extra;diagnostics_added=true; }
    }
    sent_at_=now;busy_=true;
    if (HAL_UART_Transmit_IT(&huart6,buffer_,uint16_t(length)) == HAL_OK) {
        last_report_=now;++sequence_;resync_=false;
        if (diagnostics_added) last_diagnostics_=now;
    } else {
        busy_=false; // Failed HAL start must never wedge the TX busy flag.
    }
}
