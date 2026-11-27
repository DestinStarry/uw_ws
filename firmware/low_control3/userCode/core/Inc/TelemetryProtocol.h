#pragma once
#include <cstdint>
#include <cstddef>
namespace low {
struct TelemetryValue { bool valid = false; int32_t value = 0; uint32_t age = UINT32_MAX; };
struct TelemetrySnapshot {
    TelemetryValue voltage; // centivolts; RPM below is integer mechanical RPM.
    TelemetryValue rpm[4];
    uint32_t sequence = 0, now = 0, stop_reason = 1, rejected = 0, spi_errors = 0;
    bool active = false, ch_ready = false;
};
// Returns 0 on truncation; never transmit a partial or unterminated line.
size_t FormatTelemetry(char *buffer, size_t capacity, const TelemetrySnapshot &snapshot);
size_t FormatDiagnostics(char *buffer, size_t capacity, const TelemetrySnapshot &snapshot);
}
