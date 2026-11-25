/// @file telemetry.cpp
/// @brief Strict VOLT decoding and bounded-rate text telemetry formatting.
#include "uw_robot_communication/tcp/telemetry.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>

namespace uw {
static VoltageField parse_field(const std::string &line, const std::string &key,
                                double minimum, double maximum, bool integer) {
  VoltageField result;
  if (line.size() > 1024) {
    result.present = true; // An oversized input must not extend validity.
    return result;
  }
  std::istringstream fields(line);
  fields.imbue(std::locale::classic());
  std::string token;
  while (fields >> token) {
    if (token.rfind(key, 0) != 0)
      continue;
    if (result.present) {
      result.value.reset();
      return result;
    }
    result.present = true;
    std::istringstream number(token.substr(key.size()));
    number.imbue(std::locale::classic());
    double value;
    if (!(number >> value) || !std::isfinite(value) || value < minimum || value > maximum ||
        (integer && std::trunc(value) != value))
      continue;
    number >> std::ws;
    if (number.eof())
      result.value = value;
  }
  return result;
}

VoltageField parse_voltage(const std::string &line) {
  return parse_field(line, "VOLT:", 0, 1000, false);
}

std::array<VoltageField, 4> parse_rpm(const std::string &line) {
  std::array<VoltageField, 4> result;
  for (size_t i = 0; i < result.size(); ++i)
    result[i] = parse_field(line, "CH" + std::to_string(i) + ":", -1000000, 1000000, true);
  return result;
}

bool TimedReading::valid(double now, double timeout) const {
  const double age = now - sample_time;
  return value && std::isfinite(*value) && sample_time >= 0 &&
         std::isfinite(age) && age >= 0 && age <= timeout;
}

static std::string number(const TimedReading &reading, double now, double timeout) {
  if (!reading.valid(now, timeout))
    return "--";
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::fixed << std::setprecision(2) << *reading.value;
  return out.str();
}
static long long age_ms(const TimedReading &reading, double now) {
  const double age = now - reading.sample_time;
  if (reading.sample_time < 0 || !std::isfinite(age) || age < 0)
    return -1;
  return static_cast<long long>(std::min(age * 1000., 2147483647.));
}
std::string encode_aux_telemetry(const TimedReading &cpu, const TimedReading &voltage,
                                 double now, double cpu_timeout, double voltage_timeout,
                                 bool legacy_temp) {
  const auto temperature = number(cpu, now, cpu_timeout);
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << "CPU_TEMP:" << temperature
      << "\nCPU_TEMP_VALID:" << int(cpu.valid(now, cpu_timeout))
      << "\nCPU_TEMP_AGE_MS:" << age_ms(cpu, now)
      << "\nVOLT:" << number(voltage, now, voltage_timeout)
      << "\nVOLT_VALID:" << int(voltage.valid(now, voltage_timeout))
      << "\nVOLT_AGE_MS:" << age_ms(voltage, now) << '\n';
  if (legacy_temp)
    out << "TEMP:" << temperature << '\n';
  return out.str();
}
std::string encode_rpm_telemetry(const std::array<TimedReading, 4> &rpm,
                                  double now, double timeout) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  for (size_t i = 0; i < rpm.size(); ++i) {
    const auto key = "CH" + std::to_string(i);
    const bool valid = rpm[i].valid(now, timeout);
    out << key << ":";
    if (valid) out << static_cast<long long>(*rpm[i].value);
    else out << "--";
    out << '\n' << key << "_VALID:" << int(valid)
        << '\n' << key << "_AGE_MS:" << age_ms(rpm[i], now) << '\n';
  }
  return out.str();
}
} // namespace uw
