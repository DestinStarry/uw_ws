"""Read-only telemetry formatting; never creates or changes control commands."""
import math


class TelemetryView:
    def __init__(self, values, ages, timeout_s=3.0):
        self.values = values
        self.ages = ages
        self.timeout_s = timeout_s

    def _fresh(self, key, timeout):
        age = self.ages.get(key)
        return age is not None and math.isfinite(age) and 0 <= age <= timeout

    def number(self, key, decimals=2, valid_key=None, age_key=None, timeout_s=None):
        timeout = self.timeout_s if timeout_s is None else timeout_s
        if not self._fresh(key, timeout):
            return "--"
        if valid_key in self.values:
            if self.values[valid_key] != "1" or not self._fresh(valid_key, timeout):
                return "--"
        if age_key in self.values:
            try:
                source_age = float(self.values[age_key]) / 1000.0
                received_age = self.ages[age_key]
                if (not math.isfinite(source_age) or source_age < 0 or
                        not self._fresh(age_key, timeout) or source_age + received_age > timeout):
                    return "--"
            except (TypeError, ValueError, KeyError):
                return "--"
        try:
            value = float(self.values[key])
        except (TypeError, ValueError, KeyError):
            return "--"
        return f"{value:.{decimals}f}" if math.isfinite(value) else "--"

    def cpu_temperature(self):
        # An invalid/stale primary field must NOT fall back to an older TEMP value.
        key = "CPU_TEMP" if "CPU_TEMP" in self.values else "TEMP"
        return self.number(key, 1, "CPU_TEMP_VALID", "CPU_TEMP_AGE_MS")

    def rpm(self, channel):
        key = f"CH{channel}"
        return self.number(key, 0, key + "_VALID", key + "_AGE_MS")
