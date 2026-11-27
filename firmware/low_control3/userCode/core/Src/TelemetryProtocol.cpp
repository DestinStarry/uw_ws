#include "TelemetryProtocol.h"
#include <cstdio>
#include <cstdarg>
namespace low {
namespace {
bool append(char *buffer, size_t capacity, size_t &used, const char *format, ...) {
    if (used >= capacity) return false;
    va_list args; va_start(args, format);
    const int length = std::vsnprintf(buffer+used, capacity-used, format, args);
    va_end(args);
    if (length < 0 || size_t(length) >= capacity-used) return false;
    used += size_t(length); return true;
}
bool append_age(char *buffer, size_t capacity, size_t &used, uint32_t age) {
    // nano.specs printf does not need 64-bit integer formatting.
    return age == UINT32_MAX ? append(buffer,capacity,used,"-1") :
                               append(buffer,capacity,used,"%lu",(unsigned long)age);
}
}
size_t FormatTelemetry(char *buffer, size_t capacity, const TelemetrySnapshot &s) {
    if (!buffer || !capacity) return 0;
    buffer[0] = '\0'; size_t used = 0;
    bool ok = true;
    if (s.voltage.valid)
        ok = append(buffer,capacity,used,"VOLT:%ld.%02ld",long(s.voltage.value/100),long(s.voltage.value%100));
    else ok = append(buffer,capacity,used,"VOLT:--");
    for (unsigned i=0;i<4 && ok;++i) {
        if (s.rpm[i].valid) ok=append(buffer,capacity,used," CH%u:%ld",i,long(s.rpm[i].value));
        else ok=append(buffer,capacity,used," CH%u:--",i);
    }
    ok=ok && append(buffer,capacity,used,"\r\n");
    if (!ok) { buffer[0]='\0'; return 0; }
    return used;
}
size_t FormatDiagnostics(char *buffer, size_t capacity, const TelemetrySnapshot &s) {
    if (!buffer || !capacity) return 0;
    buffer[0]='\0'; size_t used=0;
    bool ok=append(buffer,capacity,used,
        "DIAG MCU_SEQ:%lu MCU_MS:%lu CTRL_ACTIVE:%u CH_READY:%u STOP_REASON:%lu RX_REJECT:%lu SPI_ERRORS:%lu VOLT_VALID:%u VOLT_AGE_MS:",
        (unsigned long)s.sequence,(unsigned long)s.now,unsigned(s.active),unsigned(s.ch_ready),
        (unsigned long)s.stop_reason,(unsigned long)s.rejected,(unsigned long)s.spi_errors,unsigned(s.voltage.valid));
    ok=ok && append_age(buffer,capacity,used,s.voltage.age);
    for (unsigned i=0;i<4 && ok;++i) {
        ok=append(buffer,capacity,used," CH%u_VALID:%u CH%u_AGE_MS:",i,unsigned(s.rpm[i].valid),i);
        ok=ok && append_age(buffer,capacity,used,s.rpm[i].age);
    }
    ok=ok && append(buffer,capacity,used,"\r\n");
    if (!ok) { buffer[0]='\0'; return 0; }
    return used;
}

}
