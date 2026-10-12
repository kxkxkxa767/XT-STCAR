"""Round 59 (Claude): earlier second-role confirmation and a lag-predicted
lane-heading limit that also bounds the task-left path correction.
Real trial58 frames: fixtures/route-58-second-heading.json (saved scans only)."""
import copy
import json
import math
from pathlib import Path
import unittest

import test_maneuver_route as route
import compact_target as ct
from maneuver_sequence import ManeuverSequence, SECOND_ACQUISITION_MIN_POINTS

FIXTURE = json.loads((Path(__file__).parent/'fixtures'/'route-58-second-heading.json').read_text())


def real(seq):
    scan = copy.deepcopy(FIXTURE['scans'][str(seq)])
    scan['received_at'] = scan['at_ms']/1000
    return scan


class SecondAcquisitionTests(unittest.TestCase):
    def test_second_role_tracker_acquires_three_returns_first_tracker_does_not(self):
        self.assertEqual(SECOND_ACQUISITION_MIN_POINTS, 3)
        second = ManeuverSequence._new_second_tracker()
        first = ct.CompactTargetTracker(maintenance_bearing_rad=(-math.pi, math.pi), select_associated=True,
                                        unconfirmed_boundary_gap=True, tolerant=True)
        for seq in (425, 427):  # trial58: 3 returns at 2.66 / 2.46 m ahead
            scan = real(seq)
            found = ct._candidates(scan['ranges'], bearing_range_rad=(-math.pi/2, math.pi/2), tolerant=True,
                                   min_points=3)
            self.assertTrue(any(c['point_count'] == 3 and c['range_m'] > 2.3 for c in found))
            self.assertFalse(any(c['range_m'] > 2.3 for c in ct._candidates(
                scan['ranges'], bearing_range_rad=(-math.pi/2, math.pi/2), tolerant=True)))
        tracked = [second.update(real(seq), real(seq)['received_at']) for seq in (423, 425, 427)]
        self.assertTrue(all(t is not None for t in tracked))
        self.assertEqual(len({t['track_id'] for t in tracked}), 1)  # one identity, no restart
        self.assertIsNone(first.update(real(425), real(425)['received_at']))

    def test_acquisition_minimum_is_validated(self):
        for bad in (2, 6, True, 3.):
            with self.assertRaises(ValueError):
                ct.CompactTargetTracker(tolerant=True, acquisition_min_points=bad)
        with self.assertRaises(ValueError):
            ct.CompactTargetTracker(acquisition_min_points=3)  # tolerant only


class HeadingLimitTests(unittest.TestCase):
    def test_real58_limit_tightens_with_the_yaw_rate(self):
        motion = ManeuverSequence(0., continue_route=True)
        limits = {}
        previous = None
        for seq in (439, 442, 445):
            scan = real(seq)
            _, lane = motion._current_lane(scan, scan['received_at'])
            self.assertIsNotNone(lane)
            rate = None if previous is None else (lane['heading_left_rad']-previous[1])/(scan['at_ms']/1000-previous[0])
            _, limits[seq] = motion._lane_left_limit(scan, lane['heading_left_rad'], rate)
            previous = (scan['at_ms']/1000, lane['heading_left_rad'])
        self.assertLess(limits[442], 1632)  # recorded command (path override above the stage)
        self.assertLess(limits[445], 1500)  # recorded 1646: now straighten/return
        self.assertGreater(limits[439], limits[442])

    def test_task_left_path_correction_cannot_exceed_the_frame_limit(self):
        motion = route.RoutePathCheckTests.motion('second_orbit', right=True)
        motion._current_scan = route.RoutePathCheckTests.scan(436, {i: .70 for i in range(6, 17)})
        free = motion._route_path_target(1500)
        self.assertGreater(free, 1500)  # unchanged 52-class left bypass without a limit
        motion._path_cache = None
        motion._task_left_upper = (436, 1540)
        self.assertLessEqual(motion._route_path_target(1500), 1540)
        motion._path_cache = None
        motion._task_left_upper = (435, 1540)  # another frame's limit is ignored
        self.assertEqual(motion._route_path_target(1500), free)


if __name__ == '__main__':
    unittest.main()
