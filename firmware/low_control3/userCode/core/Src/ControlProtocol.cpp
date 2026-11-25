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
    if (std::strcmp(line, "BUZZ") == 0) { result.type = CommandType::Buzz; return result; }
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
bool NeutralThrust(const Command &command) {
    if (command.type != CommandType::Control) return false;
    for (size_t i = 0; i < 4; ++i) if (std::fabs(command.values[i]) > 0.0001f) return false;
    return true;
}
bool ControlGuard::Accept(const Command &command, uint32_t now, uint32_t received) {
    if (command.type != CommandType::Control || uint32_t(now - received) > kRxMaxAgeMs)
        return false;
    if (needs_neutral_ && !NeutralThrust(command)) return false;
    needs_neutral_ = false; active_ = true;
    last_ = received; // Never renew an old command at its processing time.
    return true;
}
bool ControlGuard::Expire(uint32_t now) {
    if (active_ && uint32_t(now - last_) > kControlTimeoutMs) { Stop(); return true; }
    return false;
}
void ControlGuard::Stop() { active_ = false; needs_neutral_ = true; }
void LineInbox::Fault() { fault_ = true; dropping_ = true; used_ = 0; ++dropped_; }
void LineInbox::Push(uint8_t byte, uint32_t now) {
    if (byte == '\r' || byte == '\n') {
        if (dropping_) { dropping_ = false; used_ = 0; return; }
        if (!used_) return;
        const uint8_t next = (head_ + 1) % kSlots;
        if (next == tail_) { Fault(); dropping_ = false; return; }
        partial_[used_] = '\0';
        std::memcpy(queue_[head_].text, partial_, used_ + 1);
        queue_[head_].received = started_;
        head_ = next; used_ = 0;
        return;
    }
    if (dropping_) return;
    if (byte < 0x20 || byte > 0x7e || used_ >= kLineSize - 1) { Fault(); return; }
    if (used_ && uint32_t(now - started_) > kRxMaxAgeMs) { Fault(); return; }
    if (!used_) started_ = now;
    partial_[used_++] = char(byte);
}
bool LineInbox::Pop(Line &line) {
    if (tail_ == head_) return false;
    line = queue_[tail_]; tail_ = (tail_ + 1) % kSlots; return true;
}
bool LineInbox::TakeFault() { bool value = fault_; fault_ = false; return value; }
void LineInbox::Reset() {
    // Drop the remainder of a potentially interrupted line before accepting new data.
    head_ = tail_ = 0; used_ = 0; dropping_ = true; fault_ = false;
}
}
