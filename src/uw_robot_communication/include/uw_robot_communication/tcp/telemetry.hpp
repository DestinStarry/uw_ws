/// @file telemetry.hpp
/// @brief Whitelisted hardware telemetry and monotonic freshness; no ROS or IO.
#pragma once
#include <array>
#include <optional>
#include <string>

namespace uw {
struct VoltageField {
  bool present = false;
  std::optional<double> value;
};
// Ignore unrelated STM32 fields. Duplicate, malformed, nonfinite voltages are invalid.
VoltageField parse_voltage(const std::string &line);

// CH0..CH3 are independently validated signed integer RPM fields.
std::array<VoltageField, 4> parse_rpm(const std::string &line);

struct TimedReading {
  std::optional<double> value;
  double sample_time = -1;
  bool valid(double now, double timeout) const;
};
// TEMP is an optional compatibility alias for Orin CPU_TEMP, never STM32 TEMP.
std::string encode_aux_telemetry(const TimedReading &cpu, const TimedReading &voltage,
                                 double now, double cpu_timeout, double voltage_timeout,
                                 bool legacy_temp);
std::string encode_rpm_telemetry(const std::array<TimedReading, 4> &rpm,
                                  double now, double timeout);
} // namespace uw
