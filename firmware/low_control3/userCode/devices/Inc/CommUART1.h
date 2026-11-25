#pragma once
#include "usermain.h"
#include "ControlProtocol.h"
// UART6 Orin link. IRQ only assembles bounded lines; main loop owns commands.
class CommUART1 : public Device {
public:
    void Init() override;
    void Handle() override;
    void Receive(uint8_t *data, uint16_t len) override;
    void RxByteISR(uint8_t byte);
    void RxErrorISR();
    bool Active() const { return guard_.Active(); }
    bool NeedsNeutral() const { return guard_.NeedsNeutral(); }
    uint32_t Rejected() const { return rejected_; }
    uint32_t Accepted() const { return accepted_; }
private:
    low::LineInbox inbox_;
    low::ControlGuard guard_;
    uint32_t rejected_ = 0, accepted_ = 0;
    void Stop(uint32_t reason);
    void Flush();
};
extern CommUART1 g_comm;
