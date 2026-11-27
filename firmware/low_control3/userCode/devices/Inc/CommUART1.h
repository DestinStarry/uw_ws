#pragma once
#include "usermain.h"
#include "ControlProtocol.h"
// UART6 Orin link. IRQ only buffers bytes; main loop owns commands.
class CommUART1 : public Device {
public:
    void Init() override;
    void Handle() override;
    void Receive(uint8_t *data, uint16_t len) override;
    void RxByteISR(uint8_t byte);
    void RxErrorISR(uint32_t errors = 0, bool rearm = false);
    uint32_t RxUartErrors() const { return uart_errors_; }
    uint32_t RxUartFlags() const { return uart_flags_; }
    uint32_t RxRearmErrors() const { return rearm_errors_; }
    uint32_t RxByteOverflow() const { return byte_overflow_; }
    uint32_t RxLineOverflow() const { return inbox_.Overflow(); }
    uint32_t RxBadByte() const { return inbox_.BadByte(); }
    uint32_t RxTooLong() const { return inbox_.TooLong(); }
    uint32_t RxPartialTimeout() const { return inbox_.PartialTimeout(); }
    uint32_t RxStale() const { return stale_; }
    uint32_t RxInvalid() const { return invalid_; }
    bool Active() const { return guard_.Active(); }
    uint32_t Rejected() const { return rejected_; }
    uint32_t Accepted() const { return accepted_; }
private:
    // IRQ producer only enqueues one byte and its original receive tick.
    struct RxByte { uint32_t tick; uint8_t value; };
    static const uint16_t kByteSlots = 1024;
    RxByte bytes_[kByteSlots];
    volatile uint16_t byte_head_ = 0, byte_tail_ = 0;
    volatile bool rx_reset_ = false;
    volatile uint32_t uart_errors_ = 0, uart_flags_ = 0, rearm_errors_ = 0, byte_overflow_ = 0;
    uint32_t stale_ = 0, invalid_ = 0;
    low::LineInbox inbox_;
    low::ControlGuard guard_;
    uint32_t rejected_ = 0, accepted_ = 0;
    void Stop(uint32_t reason);
    void Flush(bool uncertain=false);
};
extern CommUART1 g_comm;
