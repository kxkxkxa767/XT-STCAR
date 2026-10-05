"""No physical devices: clearance faults and session fencing regression checks."""
import importlib.util
from pathlib import Path
import unittest

SPEC = importlib.util.spec_from_file_location('autonomy_live', Path(__file__).parents[1] / 'autonomy_live.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class ProbeClearanceTests(unittest.TestCase):
    def setUp(self):
        self.scan = {'seq': 1, 'frame_id': 'lidar_origin_coarse_body_heading', 'ranges': [3.] * 360}
        self.ages = {'camera': .01, 'lidar': .01, 'control': .01}

    def test_unknown_front_and_near_obstacle_block_motion(self):
        for index, value in [(0, None), (359, .99), (270, .39)]:
            with self.subTest(index=index):
                self.scan['ranges'][index] = value
                with self.assertRaises(ValueError): MODULE.probe_clearance(self.scan, self.ages)
                self.scan['ranges'][index] = 3.

    def test_invalid_heading_stale_frames_and_nonfinite_values(self):
        self.scan['frame_id'] = 'unknown'
        with self.assertRaises(ValueError): MODULE.probe_clearance(self.scan, self.ages)
        self.scan['frame_id'] = 'lidar_origin_coarse_body_heading'
        for key, value in [('camera', .25), ('lidar', .15), ('control', .08), ('lidar', -1), ('lidar', float('nan'))]:
            with self.subTest(key=key, value=value):
                ages = {**self.ages, key: value}
                with self.assertRaises(ValueError): MODULE.probe_clearance(self.scan, ages)
        for value in [float('inf'), float('nan'), False, -1]:
            self.scan['ranges'][120] = value
            with self.assertRaises(ValueError): MODULE.probe_clearance(self.scan, self.ages)

    def test_probe_never_promotes_navigation_calibration(self):
        self.scan['navigation_validated'] = False
        MODULE.probe_clearance(self.scan, self.ages)
        self.assertFalse(self.scan['navigation_validated'])
        with self.assertRaises(ValueError): MODULE.probe_parameters({'pwm': 1531, 'duration_ms': 400})
        with self.assertRaises(ValueError): MODULE.probe_parameters({'pwm': 1530, 'duration_ms': 501})
        with self.assertRaises(ValueError): MODULE.probe_parameters({'pwm': True, 'duration_ms': 400})


if __name__ == '__main__':
    unittest.main()
