#pragma once
#include <stdint.h>
// Preserve tested low_control2 actuator mappings. Verify each channel on hardware.
static const uint8_t kMotorInvert[4] = {1, 1, 1, 1};
static const uint16_t kEscPolePairs[4] = {7, 7, 7, 7};
static const uint32_t kTelemetryPeriodMs = 100; // 10 Hz, independent of depth
static const bool kDiagnosticsEnabled = false; // verbose DIAG off; compact RDBG has an independent 5s switch
static const uint32_t kDiagnosticsPeriodMs = 1000; // 1 Hz diagnostics
static const uint32_t kVoltagePeriodMs = 100;
// Legacy bench PWM holds until OFF/new command when no CTRL session is active.
static const bool kLegacyPwmEnabled = true;
static const bool kRpmSummaryEnabled = true;
static const uint32_t kRpmSummaryPeriodMs = 5000;
static const uint32_t kSpiTimeoutMs = 2;
static const uint32_t kMainCycleMs = 10;
#ifndef LOW_CONTROL_ENABLE_IWDG
#define LOW_CONTROL_ENABLE_IWDG 1
#endif
