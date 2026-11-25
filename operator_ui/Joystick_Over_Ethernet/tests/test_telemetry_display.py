import math
import socket
import threading
import time
import unittest

from telemetry_display import TelemetryView
from network import CommandClient


def view(values, ages=None):
    return TelemetryView(values, {key: 0.1 for key in values} if ages is None else ages)


class DisplayTests(unittest.TestCase):
    def test_missing_and_zero_rpm_are_distinct(self):
        self.assertEqual(view({}).rpm(0), "--")
        self.assertEqual(view({"CH0": "0", "CH0_VALID": "1"}).rpm(0), "0")

    def test_four_channels_and_negative_rpm(self):
        v = view({"CH0": "1200", "CH1": "0", "CH2": "--", "CH3": "-321"})
        self.assertEqual([v.rpm(i) for i in range(4)], ["1200", "0", "--", "-321"])

    def test_invalid_flag_hides_numeric_value(self):
        self.assertEqual(view({"CH0": "1200", "CH0_VALID": "0"}).rpm(0), "--")
        self.assertEqual(view({"VOLT": "15.2", "VOLT_VALID": "0"}).number("VOLT", valid_key="VOLT_VALID"), "--")

    def test_bad_numbers(self):
        for value in ["nan", "inf", "-inf", "--", "bad", None]:
            self.assertEqual(view({"CH0": value}).rpm(0), "--")

    def test_receive_timeout_even_with_connection_alive(self):
        self.assertEqual(view({"CH0": "1200"}, {"CH0": 3.01}).rpm(0), "--")
        self.assertEqual(view({"CH0": "1200"}, {"CH0": -1}).rpm(0), "--")

    def test_source_age_plus_receive_age(self):
        values = {"CH0": "1200", "CH0_VALID": "1", "CH0_AGE_MS": "2950"}
        self.assertEqual(view(values).rpm(0), "--")
        values["CH0_AGE_MS"] = "200"
        self.assertEqual(view(values).rpm(0), "1200")
        for age in ["-1", "nan", "bad"]:
            values["CH0_AGE_MS"] = age
            self.assertEqual(view(values).rpm(0), "--")

    def test_stale_validity_flag(self):
        values = {"CH0": "1200", "CH0_VALID": "1"}
        self.assertEqual(view(values, {"CH0": 0, "CH0_VALID": 4}).rpm(0), "--")

    def test_cpu_primary_and_legacy_alias(self):
        self.assertEqual(view({"CPU_TEMP": "42.5", "TEMP": "99"}).cpu_temperature(), "42.5")
        self.assertEqual(view({"TEMP": "43.25"}).cpu_temperature(), "43.2")

    def test_invalid_primary_cannot_fall_back(self):
        values = {"CPU_TEMP": "--", "CPU_TEMP_VALID": "0", "TEMP": "99"}
        self.assertEqual(view(values).cpu_temperature(), "--")
        values["CPU_TEMP"] = "42.5"
        self.assertEqual(view(values).cpu_temperature(), "--")
        values["CPU_TEMP_VALID"] = "1"
        self.assertEqual(view(values, {"CPU_TEMP": 4, "CPU_TEMP_VALID": 0, "TEMP": 0}).cpu_temperature(), "--")

    def test_legacy_fields_without_flags(self):
        self.assertEqual(view({"VOLT": "15.23"}).number("VOLT", valid_key="VOLT_VALID"), "15.23")

    def test_depth_and_attitude_invalid(self):
        self.assertEqual(view({"DEPTH": "1.25", "DEPTH_VALID": "0"}).number("DEPTH", valid_key="DEPTH_VALID"), "--")
        self.assertEqual(view({"YAW": "12", "IMU_OK": "0"}).number("YAW", valid_key="IMU_OK"), "--")


class NetworkSnapshotTests(unittest.TestCase):
    def test_real_receive_timestamps_heartbeat_and_disconnect(self):
        local, peer = socket.socketpair()
        local.settimeout(.05)
        c = CommandClient("unused", 0)
        c.sock = local
        c._connected_at = time.monotonic()
        worker = threading.Thread(target=c._recv_loop, args=(local,))
        worker.start()
        def wait_for(predicate):
            deadline = time.monotonic() + 1
            while time.monotonic() < deadline:
                if predicate(): return
                time.sleep(.005)
            self.fail("receiver did not update")
        try:
            peer.sendall(b"VOLT:15.")
            peer.sendall(b"23\nCH0:0\nCH0_VALID:1\n")
            wait_for(lambda: "VOLT" in c._received_at)
            self.assertFalse(c.connected, "sensor data must not fake a control heartbeat")
            self.assertEqual(c.telemetry_snapshot(), ({}, {}))
            peer.sendall(b"ARMED:1\nCPU_TEMP:42.5\nCPU_TEMP_VALID:1\n")
            wait_for(lambda: c.get("CPU_TEMP_VALID") == "1")
            values, ages = c.telemetry_snapshot()
            self.assertEqual(values["VOLT"], "15.23")
            self.assertGreaterEqual(ages["VOLT"], 0)
            v = TelemetryView(values, ages)
            self.assertEqual(v.rpm(0), "0")
            self.assertEqual(v.cpu_temperature(), "42.5")
            peer.close()
            worker.join(timeout=1)
            self.assertFalse(worker.is_alive())
            self.assertEqual(c.telemetry_snapshot(), ({}, {}))
            self.assertEqual(c._received_at, {})
        finally:
            c.close()
            peer.close()
            worker.join(timeout=1)


if __name__ == "__main__":
    unittest.main()
