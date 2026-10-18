"""Round 72 (2026-10-18): leaving the start lane the left divider ended, the
native corridor vanished (a 12-bin null run > 5 deg), the right-front rays met
the right board at grazing incidence (bins 6-17 null) and the car stopped on
front_sparse while the first cone (4 returns, 3.03 m, 31.5 deg left) was just
beyond the 3 m acquisition. User: “左侧的挡板没了可以参考右侧的”."""
import json
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).parents[1]))
import autonomy_live as al  # noqa: E402
import compact_target as ct  # noqa: E402
import maneuver_sequence as ms  # noqa: E402

FRAMES = json.loads((Path(__file__).parent/'fixtures/route-72-start-lane-exit.json').read_text())['frames']


def board_scan(right=.47, left=None, nulls=(), opening_at=None):
    """Right board y=right (bin frame, y right); optional left board; nulls."""
    ranges = [None]*360
    for a in range(360):
        c, s = math.cos(math.radians(a)), math.sin(math.radians(a))
        r = None
        if s > 1e-6:
            r = right/s
            if opening_at is not None and r*c > opening_at:
                r = (right+.6)/s  # beyond the board line: an opening
        elif s < -1e-6 and left is not None:
            r = -left/s
        if r is None or r > 8.:
            r = 7./max(.2, abs(c)) if abs(c) > .2 else None
        ranges[a] = None if r is None else round(min(r, 11.), 4)
    for a in nulls:
        ranges[a % 360] = None
    return ranges


class SideWallFit(unittest.TestCase):
    def test_right_board_fits_and_mixed_left_does_not(self):
        fit = al.side_wall_fit(board_scan(), 'right')
        self.assertIsNotNone(fit)
        self.assertAlmostEqual(fit['intercept_m'], .47, places=2)
        self.assertAlmostEqual(fit['slope'], 0., places=2)
        self.assertIsNone(al.side_wall_fit(board_scan(), 'left'))

    def test_real_exit_frames(self):
        for seq in ('1545', '1547'):
            fit = al.side_wall_fit(FRAMES[seq]['ranges'], 'right')
            self.assertIsNotNone(fit)
            self.assertAlmostEqual(fit['intercept_m'], .485, delta=.02)
            self.assertIsNone(al.side_wall_fit(FRAMES[seq]['ranges'], 'left'))


class SideWallProjection(unittest.TestCase):
    def test_grazing_nulls_on_the_board_count_as_surface(self):
        ranges = board_scan(nulls=range(6, 18))
        plain = al.front_gap_evidence(ranges, bridge=True)
        self.assertEqual(plain['largest_open_gap_deg'], 12)
        evidence = al.front_gap_evidence(ranges, bridge=True, side_walls=True)
        self.assertEqual(evidence['front_open_unknown_bins'], [])
        self.assertEqual(evidence['front_wall_projected_bins'], list(range(6, 18)))

    def test_an_opening_stops_the_board(self):
        # The board ends 2 m ahead (real returns beyond its line at 6-13 deg):
        # nulls before it (14-17 deg, 1.5-1.9 m) are surface, beyond (3-8, too
        # long for the short-gap bridge) unknown.
        ranges = board_scan(nulls=(3, 4, 5, 6, 7, 8, 14, 15, 16, 17), opening_at=2.)
        evidence = al.front_gap_evidence(ranges, bridge=True, side_walls=True)
        projection = evidence['front_wall_projection']
        self.assertLess(projection['known_opening_limit_forward_m']['right'], 2.1)
        self.assertEqual(evidence['front_wall_projected_bins'], [14, 15, 16, 17])
        self.assertEqual(evidence['front_open_unknown_bins'], [3, 4, 5, 6, 7, 8])

    def test_round_72_stop_frame(self):
        ranges = FRAMES['1547']['ranges']
        self.assertEqual(al.front_gap_evidence(ranges, bridge=True)['largest_open_gap_deg'], 12)
        evidence = al.front_gap_evidence(ranges, bridge=True, side_walls=True)
        self.assertEqual(evidence['front_open_unknown_bins'], [])
        self.assertEqual(evidence['front_wall_projected_bins'], list(range(6, 18)))

    def test_probe_scope(self):
        scan = {'seq': 7, 'at_ms': 700, 'frame_id': 'lidar_origin_coarse_body_heading', 'ranges': [2.]*360}
        ages = {'lidar': .05, 'control': .01, 'camera': 0.}
        with self.assertRaises(ValueError):
            al.probe_clearance(scan, ages, camera_required=False, front_side_walls=True)  # straight profile
        with self.assertRaises(ValueError):
            al.probe_clearance(scan, ages, camera_required=False, clearance_profile='maneuver',
                               front_side_walls=1)
        self.assertTrue(al.probe_clearance(scan, ages, camera_required=False, clearance_profile='maneuver',
                                           front_side_walls=True)['motion_ready'])


class SideWallSteering(unittest.TestCase):
    def motion(self):
        motion = ms.ManeuverSequence(0., continue_route=True, max_drive_s=40.)
        motion._start_lane_width = .928
        return motion

    def test_lane_centre_from_the_right_board(self):
        scan = {'seq': 1547, 'ranges': FRAMES['1547']['ranges']}
        walls = self.motion()._start_side_wall(scan, None)
        self.assertEqual(walls['side_wall'], 'right')
        self.assertAlmostEqual(walls['offset_right_m'], .481-.928/2, delta=.01)
        self.assertAlmostEqual(walls['width_m'], .928)

    def test_board_must_run_along_the_compass_axis(self):
        scan = {'seq': 1545, 'ranges': FRAMES['1545']['ranges']}
        motion = self.motion()
        self.assertIsNotNone(motion._start_side_wall(scan, 0.))
        self.assertIsNone(motion._start_side_wall(scan, math.radians(8)))
        motion._start_lane_width = None
        self.assertIsNone(motion._start_side_wall(scan, 0.))


class FirstConeRange(unittest.TestCase):
    def test_round_72_cone_is_a_candidate_only_with_the_route_range(self):
        ranges = FRAMES['1547']['ranges']
        kw = dict(bearing_range_rad=ms.FIRST_ACQUIRE_BEARING_RAD, tolerant=True)
        self.assertEqual(ct._candidates(ranges, **kw), [])
        found = ct._candidates(ranges, max_range_m=ct.EXTENDED_MAX_RANGE_M, **kw)
        self.assertEqual(len(found), 1)
        self.assertAlmostEqual(found[0]['range_m'], 3.03, delta=.03)
        self.assertEqual(found[0]['point_count'], 4)

    def test_bounds(self):
        tracker = ct.CompactTargetTracker(tolerant=True)
        with self.assertRaises(ValueError):
            tracker.extend_acquisition_range(4.)
        with self.assertRaises(ValueError):
            ct._candidates([None]*360, max_range_m=2.)
        tracker.extend_acquisition_range(3.5)


if __name__ == '__main__':
    unittest.main()
