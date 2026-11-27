#include "ControlProtocol.h"
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <cerrno>
namespace low {
Command ParseCommand(const char *line) {
    Command result;
    if (!line) return result;
    if (std::strcmp(line, "OFF") == 0) { result.type = CommandType::Off; return result; }
    if (std::strncmp(line,"OFF:",4)==0) {
        result.type=CommandType::Off;
        char *end=nullptr;errno=0;
        const float camera=std::strtof(line+4,&end);
        if (end!=line+4 && *end=='\0' && errno!=ERANGE && std::isfinite(camera) && camera>=-1.f && camera<=1.f) {
            result.camera_target=true;result.values[6]=camera;
        }
        return result; // Malformed camera target must never suppress the motor stop.
    }
    if (std::strcmp(line, "BUZZ") == 0) { result.type = CommandType::Buzz; return result; }
    // Legacy low2 bench commands: physical channel 1..4 motors, 5..7 servos.
    // Raw PWM does not invert direction and does not feed the CTRL watchdog.
    if (line[0]>='1' && line[0]<='8' && line[1]==' ') {
        const char *p=line+2;
        while (*p==' ') ++p;
        const char *digits=p;
        if (*digits=='+' || *digits=='-') ++digits;
        if (*digits<'0' || *digits>'9') return result;
        char *end=nullptr; errno=0;
        const long value=std::strtol(p,&end,10);
        if (errno==ERANGE || *end || value<INT32_MIN || value>INT32_MAX) return result;
        result.pwm_id=uint8_t(line[0]-'0'); result.pwm_us=int32_t(value);
        result.type=result.pwm_id==8 ? CommandType::Buzz : CommandType::RawPwm;
        return result;
    }
    if (std::strncmp(line, "CTRL:", 5) != 0) return result;
    const char *p = line + 5;
    for (size_t i = 0; i < kChannels; ++i) {
        // Only decimal/exponent syntax, no whitespace, nan, inf or hex-floats.
        const char *start = p;
        if (*p == '+' || *p == '-') ++p;
        unsigned digits = 0;
        while (*p >= '0' && *p <= '9') { ++p; ++digits; }
        if (*p == '.') { ++p; while (*p >= '0' && *p <= '9') { ++p; ++digits; } }
        if (!digits) return Command{};
        if (*p == 'e' || *p == 'E') {
            ++p; if (*p == '+' || *p == '-') ++p;
            const char *exponent = p;
            while (*p >= '0' && *p <= '9') ++p;
            if (p == exponent) return Command{};
        }
        if ((i < kChannels - 1 && *p != ',') || (i == kChannels - 1 && *p != '\0'))
            return Command{};
        char *end = nullptr;
        errno = 0;
        const float value = std::strtof(start, &end);
        if (end != p || errno == ERANGE || !std::isfinite(value) || value < -1 || value > 1)
            return Command{};
        result.values[i] = value;
        if (i < kChannels - 1) ++p;
    }
    result.type = CommandType::Control;
    return result;
}
bool ControlGuard::Accept(const Command &command, uint32_t now, uint32_t received) {
    if (command.type != CommandType::Control || uint32_t(now - received) > kRxMaxAgeMs)
        return false;
    active_ = true;
    last_ = received; // Never renew an old command at its processing time.
    return true;
}
bool ControlGuard::Expire(uint32_t now) {
    if (active_ && uint32_t(now - last_) > kControlTimeoutMs) { Stop(); return true; }
    return false;
}
void ControlGuard::Stop() { active_ = false; }
void LineInbox::Fault() { fault_ = true; dropping_ = true; used_ = 0; ++dropped_; }
void LineInbox::Push(uint8_t byte, uint32_t now) {
    if (byte == '\r' || byte == '\n') {
        if (dropping_) { dropping_ = false; used_ = 0; return; }
        if (!used_) return;
        const uint8_t next = (head_ + 1) % kSlots;
        if (next == tail_) { ++overflow_; Fault(); dropping_ = false; return; }
        partial_[used_] = '\0';
        std::memcpy(queue_[head_].text, partial_, used_ + 1);
        queue_[head_].received = started_;
        head_ = next; used_ = 0;
        return;
    }
    if (dropping_) return;
    if (byte < 0x20 || byte > 0x7e) { ++bad_byte_; Fault(); return; }
    if (used_ >= kLineSize - 1) { ++too_long_; Fault(); return; }
    if (used_ && uint32_t(now - started_) > kRxMaxAgeMs) { ++partial_timeout_; Fault(); return; }
    if (!used_) started_ = now;
    partial_[used_++] = char(byte);
}
bool LineInbox::Pop(Line &line) {
    if (tail_ == head_) return false;
    line = queue_[tail_]; tail_ = (tail_ + 1) % kSlots; return true;
}
bool LineInbox::TakeFault() { bool value = fault_; fault_ = false; return value; }
void LineInbox::Reset(bool discard_until_delimiter) {
    // Drop the remainder of a potentially interrupted line before accepting new data.
    head_ = tail_ = 0; used_ = 0; dropping_ = discard_until_delimiter; fault_ = false;
}
}
