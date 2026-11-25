#include "CommUART1.h"
#include "MotionController.h"
#include "Motor.h"
#include "Servo.h"
#include "Buzzer.h"
#include "Safety.h"
namespace {
struct IrqLock {
    uint32_t previous;
    IrqLock() : previous(__get_PRIMASK()) { __disable_irq(); __DMB(); }
    ~IrqLock() { __DMB(); __set_PRIMASK(previous); }
};
}
CommUART1 g_comm;
void CommUART1::Init() { g_motion.Init(); guard_.Stop(); }
void CommUART1::RxByteISR(uint8_t byte) {
    inbox_.Push(byte, HAL_GetTick());
    if (inbox_.HasFault()) Safety_FaultISR(SAFETY_RX);
}
void CommUART1::RxErrorISR() { inbox_.Fault(); Safety_FaultISR(SAFETY_RX); }
void CommUART1::Receive(uint8_t *, uint16_t) {
    // Deliberately no generic broadcast RX: only UART6 RxByteISR is an input.
}
void CommUART1::Flush() { IrqLock lock; inbox_.Reset(); }
void CommUART1::Stop(uint32_t reason) {
    IrqLock lock;
    guard_.Stop(); g_motion.EmergencyStop(); Safety_Stop(reason);
}
void CommUART1::Handle() {
    uint32_t fault;
    {
        IrqLock lock;
        fault = Safety_TakeFault();
        if (inbox_.TakeFault()) fault |= SAFETY_RX;
    }
    if (fault) { Stop(fault); Flush(); return; }
    if (guard_.Expire(HAL_GetTick())) { Stop(SAFETY_TIMEOUT); Flush(); return; }
    low::Line lines[7];
    size_t count = 0;
    {
        IrqLock lock;
        while (count < 7 && inbox_.Pop(lines[count])) ++count;
    }
    // OFF wins over ALL CTRL commands already queued in this batch.
    for (size_t i = 0; i < count; ++i) {
        if (low::ParseCommand(lines[i].text).type == low::CommandType::Off) {
            Stop(SAFETY_OFF); Flush(); return;
        }
    }
    for (size_t i = 0; i < count; ++i) {
        const auto command = low::ParseCommand(lines[i].text);
        if (command.type == low::CommandType::Invalid ||
            uint32_t(HAL_GetTick() - lines[i].received) > low::kRxMaxAgeMs) {
            ++rejected_; Stop(SAFETY_BAD_COMMAND); Flush(); return;
        }
        if (command.type == low::CommandType::Buzz) { g_buzzer.RequestBeep(3); continue; }
        IrqLock lock;
        // Fault may arrive while the main loop was parsing this batch.
        if (Safety_PendingFault()) return;
        if (!guard_.Accept(command, HAL_GetTick(), lines[i].received)) { ++rejected_; continue; }
        g_motion.Apply(command.values);
        Safety_Accept(lines[i].received);
        ++accepted_;
    }
}
