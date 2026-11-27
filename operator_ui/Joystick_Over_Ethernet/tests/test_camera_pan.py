import math
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from camera_pan import CameraPan
from command_mapper import CommandMapper

class CameraPanTest(unittest.TestCase):
    def test_edge_hold_and_limits(self):
        pan = CameraPan()
        self.assertEqual(pan.angle_deg, 0)
        self.assertEqual(pan.update(1), 10)
        for _ in range(100): self.assertEqual(pan.update(1), 10)
        pan.update(0); self.assertEqual(pan.update(1), 20)
        pan.update(0); self.assertEqual(pan.update(-1), 10)
        for _ in range(30): pan.update(0); pan.update(1)
        self.assertEqual(pan.angle_deg, 90)
        for _ in range(30): pan.update(0); pan.update(-1)
        self.assertEqual(pan.angle_deg, -90)
        pan.update(0); self.assertEqual(pan.update(0), -90)

    def test_absolute_target_frame(self):
        args=((0.,)*6, False, False, False, False)
        self.assertEqual(len(CommandMapper.frame(*args).split(',')), 10)
        self.assertTrue(CommandMapper.frame(*args, camera_angle_deg=10).endswith(',10.0'))
        self.assertEqual(len(CommandMapper.frame(*args, camera_angle_deg=0).split(',')), 11)
        for angle in (91, -91, math.nan, math.inf):
            with self.assertRaises(ValueError): CommandMapper.frame(*args, camera_angle_deg=angle)

if __name__ == '__main__': unittest.main()
