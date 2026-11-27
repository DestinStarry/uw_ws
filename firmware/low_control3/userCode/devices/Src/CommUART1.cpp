#include "CommUART1.h"
#include "MotionController.h"
#include "Motor.h"
#include "Servo.h"
#include "Buzzer.h"
#include "Safety.h"
#include "BoardConfig.h"
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
    const uint16_t next = uint16_t((byte_head_ + 1) % kByteSlots);
    if (next == byte_tail_) { ++byte_overflow_; rx_reset_ = true; return; }
    bytes_[byte_head_].tick = HAL_GetTick(); bytes_[byte_head_].value = byte;
    __DMB(); byte_head_ = next;
}
void CommUART1::RxErrorISR(uint32_t errors, bool rearm) {
    if (rearm) ++rearm_errors_; else ++uart_errors_;
    uart_flags_ |= errors;
    rx_reset_ = true;
}
void CommUART1::Receive(uint8_t *, uint16_t) {
    // Deliberately no generic broadcast RX: only UART6 RxByteISR is an input.
}
void CommUART1::Flush(bool uncertain) {
    IrqLock lock;
    bool boundary=inbox_.AtLineBoundary();
    if (byte_tail_!=byte_head_) {
        const uint8_t last=bytes_[(byte_head_+kByteSlots-1)%kByteSlots].value;
        boundary=(last=='\r' || last=='\n');
    }
    inbox_.Reset(uncertain || !boundary);
    byte_tail_=byte_head_;
}
void CommUART1::Stop(uint32_t reason) {
    IrqLock lock;
    guard_.Stop(); g_motion.EmergencyStop(); Safety_Stop(reason);
}
void CommUART1::Handle() {
    uint32_t fault; bool reset;
    {
        IrqLock lock;
        fault = Safety_TakeFault();
        reset = rx_reset_ || inbox_.TakeFault(); rx_reset_ = false;
    }
    if (fault) { Stop(fault); Flush(); return; }
    if (guard_.Expire(HAL_GetTick())) { Stop(SAFETY_TIMEOUT); Flush(); return; }
    // Corrupt data never updates the last-valid timestamp. Hold the last valid
    // output only within the same 200ms deadman window, as in low_control2.
    if (reset) { Flush(true); return; }
    // Parse with interrupts enabled; avoid a whole-line memcpy inside UART IRQ.
    unsigned complete = 0;
    for (unsigned drained = 0; drained < 1023 && complete < 7; ++drained) {
        RxByte byte;
        { IrqLock lock;
          if (byte_tail_ == byte_head_) break;
          byte = bytes_[byte_tail_]; byte_tail_ = uint16_t((byte_tail_ + 1) % kByteSlots);
        }
        inbox_.Push(byte.value, byte.tick);
        if (inbox_.HasFault()) {
            inbox_.TakeFault(); ++rejected_; Flush(true); return;
        }
        // Consume lines before filling the seven-line parser queue.
        if (byte.value == '\n') ++complete;
    }
    if (guard_.Expire(HAL_GetTick())) { Stop(SAFETY_TIMEOUT); Flush(); return; }
    low::Line lines[7];
    size_t count = 0;
    while (count < 7 && inbox_.Pop(lines[count])) ++count;
    // OFF wins over ALL CTRL commands already queued in this batch.
    for (size_t i = 0; i < count; ++i) {
        const auto stop=low::ParseCommand(lines[i].text);
        if (stop.type == low::CommandType::Off) {
            IrqLock lock;
            Stop(SAFETY_OFF);
            if (stop.camera_target) g_motion.SetCamera(stop.values[6]);
            Flush(); return;
        }
    }
    for (size_t i = 0; i < count; ++i) {
        const auto command = low::ParseCommand(lines[i].text);
        if (command.type == low::CommandType::Invalid ||
            uint32_t(HAL_GetTick() - lines[i].received) > low::kRxMaxAgeMs) {
            if (command.type == low::CommandType::Invalid) ++invalid_; else ++stale_;
            ++rejected_; continue;
        }
        if (command.type == low::CommandType::Buzz) { g_buzzer.RequestBeep(3); continue; }
        IrqLock lock;
        // Fault may arrive while the main loop was parsing this batch.
        if (Safety_PendingFault()) return;
        if (command.type==low::CommandType::RawPwm) {
            if (!kLegacyPwmEnabled) { ++rejected_; continue; }
            if (command.pwm_id<=4) { g_motor.SetPWM(command.pwm_id-1,command.pwm_us);g_motor.Handle(); }
            else if (command.pwm_id==7) g_motion.SetCamera((float(command.pwm_us)-1500.f)/500.f);
            else { g_servo.SetPWM(command.pwm_id-5,command.pwm_us);g_servo.Handle(); }
            // Match low2: no renewed watchdog, and no automatic mode/arm transition.
            ++accepted_; continue;
        }
        if (!guard_.Accept(command, HAL_GetTick(), lines[i].received)) { ++rejected_; continue; }
        g_motion.Apply(command.values);
        Safety_Accept(lines[i].received);
        ++accepted_;
    }
}
