#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
enum SafetyReason { SAFETY_BOOT=1, SAFETY_OFF=2, SAFETY_TIMEOUT=4,
                    SAFETY_RX=8, SAFETY_BAD_COMMAND=16, SAFETY_FATAL=32 };
void Safety_Init(void);
void Safety_Accept(uint32_t received_tick);
void Safety_Stop(uint32_t reason);
void Safety_FaultISR(uint32_t reason);
void Safety_Tick(uint32_t now);
uint32_t Safety_TakeFault(void); // IRQs must be masked.
uint32_t Safety_PendingFault(void);
uint32_t Safety_LastReason(void);
void Watchdog_Init(void);
void Watchdog_Refresh(void);
#ifdef __cplusplus
}
#endif
