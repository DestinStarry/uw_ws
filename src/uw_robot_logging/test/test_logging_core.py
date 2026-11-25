import csv
import json
import math
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

from uw_robot_logging.schema import Snapshot, header, euler_degrees
from uw_robot_logging.adapters import adapt
from uw_robot_logging.writer import SessionWriter
from uw_robot_logging.summarize import summarize


class SnapshotTests(unittest.TestCase):
    def test_empty_snapshot_does_not_invent_zero_measurements(self):
        row=Snapshot().sample(0,10,100)
        self.assertEqual(set(row),set(header()))
        for key in ("depth_m","rpm_0","mcu_voltage_v","cpu_temperature_c","cmd_u"):
            self.assertIsNone(row[key])
        self.assertFalse(row["depth_log_usable"])
        self.assertFalse(row["imu_fresh"])

    def test_freshness_includes_source_age_and_sample_age(self):
        model=Snapshot()
        model.observe("depth",{"depth_m":1.2,"depth_rate_mps":0.,"depth_valid":True,"depth_data_age_s":.1},10,100)
        model.observe("robot",{"depth_hold":True,"depth_target_m":1.5},10,100)
        row=model.sample(0,10.1,100.1)
        self.assertTrue(row["depth_log_usable"])
        self.assertAlmostEqual(row["depth_error_m"],.3)
        self.assertFalse(model.sample(2,12,102)["depth_log_usable"])
        model.observe("depth",{"depth_m":1.2,"depth_valid":True,"depth_data_age_s":2.},12,102)
        self.assertFalse(model.sample(2,12,102)["depth_log_usable"])

    def test_nan_depth_is_not_usable(self):
        model=Snapshot();data=adapt("depth",NS(depth_m=float("nan"),depth_rate_mps=0.,valid=True,data_age_s=0.))
        model.observe("depth",data,1,1)
        self.assertFalse(model.sample(0,1,1)["depth_log_usable"])

    def test_raw_diagnostics_do_not_refresh_rpm(self):
        model=Snapshot();model.observe("rx",{"hardware_rx":"VOLT:15.23 CH0:0 CH1:--"},10)
        model.observe("rx",{"hardware_rx":"DIAG CH0_VALID:1 CH0_AGE_MS:12 SPI_ERRORS:2"},11)
        row=model.sample(2,12,100)
        self.assertEqual(row["rpm_0"],0)
        self.assertEqual(row["rpm_0_receive_age_s"],2)
        self.assertIsNone(row["rpm_1"])
        self.assertEqual(row["mcu_spi_errors"],2)
        model.observe("rx",{"hardware_rx":"CH0:100 CH0:200 VOLT:bad"},12)
        self.assertIsNone(model.sample(2,12,100)["rpm_0"])
        self.assertIsNone(model.sample(2,12,100)["mcu_voltage_v"])

    def test_euler_identity_and_bad_quaternion(self):
        self.assertEqual(euler_degrees(1,0,0,0),(0,0,0))
        self.assertEqual(euler_degrees(0,0,0,0),(None,)*3)
        self.assertEqual(euler_degrees(float("nan"),0,0,0),(None,)*3)

    def test_ctrl_exact_parse_and_off_not_fake_thrust(self):
        data=adapt("tx",NS(data="CTRL:0.1,0,0,0,1,-1,-0.5\n"))
        self.assertEqual(data["ctrl_f1"],.1)
        self.assertEqual(data["ctrl_gripper"],-.5)
        self.assertIsNone(adapt("tx",NS(data="OFF\n"))["ctrl_f1"])
        self.assertIsNone(adapt("tx",NS(data="CTRL:0,0"))["ctrl_f1"])

    def test_snapshot_is_independent_and_ros_clock_jump_is_visible(self):
        model=Snapshot();model.observe("cpu",{"cpu_temperature_c":40},10,100)
        before=model.sample(0,10,100)
        model.observe("cpu",{"cpu_temperature_c":50},11,101)
        self.assertEqual(before["cpu_temperature_c"],40)
        row=model.sample(1,11,90)
        self.assertEqual(row["cpu_source_age_s"],-11)
        self.assertFalse(row["cpu_fresh"])

    def test_allocation_error_is_model_error(self):
        m=NS(planner_valid=True,planner_ms=2.,state="ok",target_wrench=[1.]*6,
             achievable_wrench=[1.,2.,3.,4.,5.,6.],achieved_wrench=[1.,2.,2.,4.,5.,4.],
             thruster_forces=[1.,2.,3.,4.],servo_target_rad=[0.,math.pi/2],
             servo_estimated_rad=[0.,0.],gate=[1.,.5])
        data=adapt("allocation",m)
        self.assertEqual(data["allocation_force_error_model_n"],1)
        self.assertEqual(data["allocation_moment_error_model_nm"],2)
        self.assertAlmostEqual(data["servo_target_24_deg"],90)


class WriterTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.revision=patch("uw_robot_logging.writer.revision",return_value={"revision":"test","dirty":False})
        self.revision.start();self.addCleanup(self.revision.stop)

    def writer(self, cls=SessionWriter, **options):
        writer=cls(self.temp.name,{},min_free_mb=0,fsync=False,**options)
        self.assertTrue(writer.ready.wait(2),"writer startup timed out")
        return writer

    def test_csv_events_metadata_and_orderly_close(self):
        writer=self.writer()
        model=Snapshot()
        model.observe("rx",{"hardware_rx":"VOLT:15.2 CH0:0"},10)
        for i in range(30):self.assertTrue(writer.sample(model.sample(i*.02,10+i*.02,100+i*.02)))
        writer.event({"kind":"parameters","node":"/control_node","values":{"gain":1.2}})
        self.assertTrue(writer.close())
        with (writer.path/"flight.csv").open(newline="",encoding="utf-8") as f: rows=list(csv.DictReader(f))
        self.assertEqual(len(rows),30)
        self.assertEqual(rows[0]["rpm_0"],"0.0")
        self.assertEqual(rows[0]["depth_m"],"")
        self.assertEqual(json.loads((writer.path/"parameters.json").read_text())["/control_node"]["gain"],1.2)
        summary=json.loads((writer.path/"summary.json").read_text())
        self.assertEqual(summary["csv_rows"],30)
        self.assertTrue(summary["clean_close"])
        self.assertEqual(summarize(writer.path)["rows"],30)

    def test_parameter_changes_are_recorded(self):
        writer=self.writer()
        writer.event({"kind":"parameters","node":"/control_node","values":{"gain":1.,"old":2.}})
        writer.event({"kind":"parameter_change","node":"/control_node","values":{"gain":2.},"deleted":["old"]})
        self.assertTrue(writer.close())
        values=json.loads((writer.path/"parameters.json").read_text())["/control_node"]
        self.assertEqual(values,{"gain":2.})

    def test_queue_overflow_is_bounded_and_reported(self):
        entered,release=threading.Event(),threading.Event()
        class SlowWriter(SessionWriter):
            def _write_record(self,*args):
                entered.set();release.wait(2);return super()._write_record(*args)
        writer=self.writer(SlowWriter,capacity=2)
        try:
            writer.event({"kind":"first"});self.assertTrue(entered.wait(1))
            started=time.monotonic()
            writer.event({"kind":"a"});writer.event({"kind":"b"})
            self.assertFalse(writer.sample(Snapshot().sample(0,1,1)))
            self.assertLess(time.monotonic()-started,.2)
            self.assertLessEqual(writer.queue.qsize(),2)
            self.assertEqual(writer.status()["dropped_csv"],1)
        finally:release.set();writer.close()
        self.assertEqual(json.loads((writer.path/"summary.json").read_text())["dropped_csv"],1)

    def test_write_failure_is_not_reported_as_success(self):
        class FailingWriter(SessionWriter):
            def _write_record(self,*args):raise OSError("injected disk failure")
        writer=self.writer(FailingWriter)
        writer.event({"kind":"test"})
        self.assertTrue(writer.finished.wait(2))
        self.assertFalse(writer.close())
        self.assertEqual(writer.status()["state"],"failed")
        self.assertIn("disk failure",writer.status()["error"])
        self.assertFalse(json.loads((writer.path/"summary.json").read_text())["clean_close"])

    def test_session_names_are_unique_and_label_cannot_escape(self):
        a=self.writer(label="../../dive");b=self.writer(label="../../dive")
        self.assertNotEqual(a.path,b.path)
        self.assertEqual(a.path.parent,Path(self.temp.name).resolve())
        self.assertTrue(a.close());self.assertTrue(b.close())

    def test_config_snapshot_is_the_supplied_file(self):
        config=Path(self.temp.name)/"config.yaml";config.write_text("control_node: {}\n")
        writer=SessionWriter(self.temp.name,{"config_path":str(config)},min_free_mb=0,fsync=False)
        self.assertTrue(writer.ready.wait(2));self.assertTrue(writer.close())
        self.assertEqual((writer.path/"robot.yaml").read_bytes(),config.read_bytes())
        self.assertTrue(json.loads((writer.path/"metadata.json").read_text())["configuration"]["copied"])

    def test_low_disk_and_size_limit_are_explicit(self):
        low=SessionWriter(self.temp.name,{},min_free_mb=1e15,fsync=False)
        self.assertTrue(low.finished.wait(2))
        self.assertEqual(low.status()["state"],"limited")
        self.assertEqual(low.status()["error"],"disk_low")
        small=self.writer(max_session_mb=.00001)
        self.assertTrue(small.finished.wait(2))
        self.assertEqual(small.status()["error"],"session_size_limit")

    def test_close_timeout_is_visible_and_can_finish_later(self):
        entered,release=threading.Event(),threading.Event()
        class SlowWriter(SessionWriter):
            def _write_record(self,*args):entered.set();release.wait(2);return super()._write_record(*args)
        writer=self.writer(SlowWriter)
        writer.event({"kind":"test"});self.assertTrue(entered.wait(1))
        self.assertFalse(writer.close(timeout=.01))
        release.set();self.assertTrue(writer.close())

    def test_partial_run_summary_marks_incomplete(self):
        directory=Path(self.temp.name)
        with (directory/"flight.csv").open("w",newline="") as f:csv.DictWriter(f,fieldnames=header()).writeheader()
        self.assertEqual(summarize(directory)["completion"],"incomplete_or_still_recording")
        with (directory/"flight.csv").open("a") as stream:stream.write("2026-11-25T00:00:00Z")
        self.assertEqual(summarize(directory)["rows"],1)
        (directory/"summary.json").write_text("{broken")
        self.assertEqual(summarize(directory)["completion"],"incomplete")


if __name__=="__main__":unittest.main()
