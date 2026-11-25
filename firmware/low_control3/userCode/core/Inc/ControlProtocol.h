#pragma once
#include <cstddef>
#include <cstdint>
namespace low {
static const size_t kChannels = 7;
static const size_t kLineSize = 128;
static const uint32_t kControlTimeoutMs = 200;
static const uint32_t kRxMaxAgeMs = 100;
enum class CommandType { Invalid, Control, Off, Buzz };
struct Command { CommandType type = CommandType::Invalid; float values[kChannels] = {}; };
Command ParseCommand(const char *line);
bool NeutralThrust(const Command &command);
// Called only in the main loop (under a short IRQ lock at output commit).
class ControlGuard {
public:
    bool Accept(const Command &command, uint32_t now, uint32_t received);
    bool Expire(uint32_t now);
    void Stop();
    bool Active() const { return active_; }
    bool NeedsNeutral() const { return needs_neutral_; }
private:
    uint32_t last_ = 0;
    bool active_ = false, needs_neutral_ = true;
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
    void Reset();
    uint32_t Dropped() const { return dropped_; }
private:
    static const uint8_t kSlots = 8;
    Line queue_[kSlots];
    char partial_[kLineSize] = {};
    uint16_t used_ = 0;
    uint32_t started_ = 0;
    uint8_t head_ = 0, tail_ = 0;
    bool dropping_ = false, fault_ = false;
    uint32_t dropped_ = 0;
};
}
