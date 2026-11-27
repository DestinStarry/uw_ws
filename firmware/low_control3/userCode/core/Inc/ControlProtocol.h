#pragma once
#include <cstddef>
#include <cstdint>
namespace low {
static const size_t kChannels = 7;
static const size_t kLineSize = 128;
static const uint32_t kControlTimeoutMs = 200;
static const uint32_t kRxMaxAgeMs = 100;
enum class CommandType { Invalid, Control, Off, Buzz, RawPwm };
struct Command { CommandType type = CommandType::Invalid; float values[kChannels] = {}; uint8_t pwm_id=0; int32_t pwm_us=0; bool camera_target=false; };
Command ParseCommand(const char *line);
// Called only in the main loop (under a short IRQ lock at output commit).
class ControlGuard {
public:
    bool Accept(const Command &command, uint32_t now, uint32_t received);
    bool Expire(uint32_t now);
    void Stop();
    bool Active() const { return active_; }
private:
    uint32_t last_ = 0;
    bool active_ = false;
};
struct Line { char text[kLineSize] = {}; uint32_t received = 0; };
// Single IRQ producer / main consumer. Main Pop/Reset/TakeFault MUST mask IRQs.
class LineInbox {
public:
    void Push(uint8_t byte, uint32_t now);
    bool Pop(Line &line);
    bool TakeFault();
    bool HasFault() const { return fault_; }
    void Fault();
    void Reset(bool discard_until_delimiter=true);
    bool AtLineBoundary() const { return !used_ && !dropping_; }
    uint32_t Dropped() const { return dropped_; }
    uint32_t Overflow() const { return overflow_; }
    uint32_t BadByte() const { return bad_byte_; }
    uint32_t TooLong() const { return too_long_; }
    uint32_t PartialTimeout() const { return partial_timeout_; }
private:
    static const uint8_t kSlots = 8;
    Line queue_[kSlots];
    char partial_[kLineSize] = {};
    uint16_t used_ = 0;
    uint32_t started_ = 0;
    uint8_t head_ = 0, tail_ = 0;
    bool dropping_ = false, fault_ = false;
    uint32_t dropped_ = 0, overflow_ = 0, bad_byte_ = 0, too_long_ = 0, partial_timeout_ = 0;
};
}
