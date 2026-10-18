"""Current native-lane wall projections explain grazing nulls, never free space."""
import copy
import json
import math
from pathlib import Path
import sys
import subprocess
import unittest

sys.path.insert(0, str(Path(__file__).parents[1]))
import autonomy_live as live
from maneuver_sequence import ManeuverSequence
import server


def lane(heading=0., width=.9, offset=0.):
    return {'heading_left_rad': heading, 'center_offset_left_m': offset, 'width_m': width,
            'left_wall_points': 60, 'right_wall_points': 60, 'support_span_m': .8,
            'fit_error_m': .01, 'origin_between_walls': True, 'candidate_only': True,
            'turn_path_certified': False}


def scan_with_lane(nulls=(), geometry=None):
    geometry = geometry or lane()
    # Ray casting of two side walls and a far end wall. Geometry is in the
    # native x-forward/y-left convention; bins are clockwise from forward.
    heading = geometry['heading_left_rad']
    ranges = []
    for i in range(360):
        angle = -math.radians(i)-heading
        normal, along = math.sin(angle), math.cos(angle)
        rho = geometry['center_offset_left_m'] + (geometry['width_m']/2 if normal > 0 else -geometry['width_m']/2)
        side = rho/normal if abs(normal) > 1e-10 else 12.
        end = 8./abs(along) if abs(along) > 1e-10 else 12.
        ranges.append(min(side, end, 12.))
    for i in nulls:
        ranges[i % 360] = None
    return {'seq': 10, 'at_ms': 1000, 'frame_id': 'lidar_origin_coarse_body_heading',
            'ranges': ranges, 'corridor_candidates': [geometry]}


class LaneWallProjection(unittest.TestCase):
    def clearance(self, scan, **kwargs):
        return live.probe_clearance(scan, {'lidar': .01, 'control': .01}, camera_required=False,
                                    clearance_profile='maneuver', **kwargs)

    def test_long_grazing_gap_is_split_by_actual_wall_intersections(self):
        # With a -6 deg lane axis the +6..17 deg rays graze its right wall;
        # only 11..17 meet that wall within 6 m. The nearly parallel five
        # rays stay unknown and retain the existing bounded route allowance.
        scan = scan_with_lane(range(6, 18), lane(heading=-math.radians(6)))
        before = copy.deepcopy(scan)
        raw = self.clearance(scan)
        result = self.clearance(scan, front_lane=scan['corridor_candidates'][0])
        self.assertFalse(raw['front_sparse_tolerable'])
        self.assertEqual(result['front_wall_projected_bins'], list(range(11, 18)))
        self.assertEqual(result['front_open_unknown_bins'], list(range(6, 11)))
        self.assertTrue(result['front_sparse_tolerable'])
        self.assertEqual(result['front_unknown_bins'], list(range(6, 18)))
        self.assertEqual(result['front_bridged_bins'], raw['front_bridged_bins'])
        self.assertEqual(result['body_proximity'], raw['body_proximity'])
        self.assertFalse(result['front_wall_projection']['free_space_certified'])
        self.assertEqual(scan, before)

    def test_front_ray_and_rays_beyond_six_metres_remain_unknown(self):
        scan = scan_with_lane(range(-4, 5))
        evidence = live.front_gap_evidence(scan['ranges'], front_lane=scan['corridor_candidates'][0])
        self.assertEqual(evidence['front_open_unknown_bins'], list(range(-4, 5)))
        self.assertEqual(evidence['front_wall_projected_bins'], [])

    def test_six_metre_boundary_and_left_right_symmetry(self):
        for sign in (-1, 1):
            with self.subTest(side=sign):
                on = lane(width=2*6*math.sin(math.radians(10)))
                scan = scan_with_lane([sign*10], on)
                evidence = live.front_gap_evidence(scan['ranges'], front_lane=on)
                self.assertEqual(evidence['front_wall_projected_bins'], [sign*10])
                beyond = lane(width=on['width_m']+.001)
                scan = scan_with_lane([sign*10], beyond)
                evidence = live.front_gap_evidence(scan['ranges'], front_lane=beyond)
                self.assertEqual(evidence['front_open_unknown_bins'], [sign*10])

    def test_observed_open_end_rejects_projection_beyond_it(self):
        scan = scan_with_lane([6, 7, 8, 9, 11, 12])
        scan['ranges'][10] = 8.  # Current beam goes through the right-wall plane.
        evidence = live.front_gap_evidence(scan['ranges'], front_lane=scan['corridor_candidates'][0])
        self.assertEqual(evidence['front_wall_projected_bins'], [11, 12])
        self.assertEqual(evidence['front_open_unknown_bins'], [6, 7, 8, 9])
        limit = evidence['front_wall_projection']['known_opening_limit_along_lane_m']['right']
        self.assertAlmostEqual(limit, .45/math.tan(math.radians(10)))

    def test_close_returns_stale_lidar_and_raw_coverage_are_not_exempted(self):
        scan = scan_with_lane(range(6, 18))
        native = scan['corridor_candidates'][0]
        stale = live.probe_clearance(scan, {'lidar': .3, 'control': .01}, camera_required=False,
            clearance_profile='maneuver', front_lane=native)
        self.assertIn('lidar_late', stale['quality_issues'])
        self.assertNotIn('front_wall_projected_bins', stale)
        scan['ranges'][0] = .27
        with self.assertRaises(live.ProbeClearanceError) as raised:
            self.clearance(scan, front_lane=native)
        self.assertEqual(str(raised.exception), 'probe_obstacle_close_body')
        self.assertEqual(raised.exception.clearance['body_proximity']['stop_trigger_point']['angle_deg'], 0)
        scan['ranges'][0] = 8.
        scan['ranges'][150:180] = [None]*30
        result = self.clearance(scan, front_lane=native)
        self.assertIn('scan_incomplete', result['quality_issues'])
        self.assertFalse(result['scan_incomplete_tolerable'])

    def test_old_bridge_and_both_quality_thresholds_stay_unchanged(self):
        self.assertEqual((live.FRONT_MAX_UNKNOWN, live.FRONT_MAX_GAP), (6, 3))
        self.assertEqual((live.MANEUVER_TOLERABLE_FRONT_UNKNOWN, live.MANEUVER_TOLERABLE_FRONT_RUN), (15, 5))
        scan = scan_with_lane(range(-2, 3))
        raw = live.front_gap_evidence(scan['ranges'], bridge=True)
        with_lane = live.front_gap_evidence(scan['ranges'], bridge=True, front_lane=scan['corridor_candidates'][0])
        self.assertEqual(raw, with_lane)  # Existing equal-range bridge takes precedence.

    def test_wrong_scan_lane_or_straight_profile_cannot_enable_projection(self):
        scan = scan_with_lane(range(6, 18))
        with self.assertRaisesRegex(ValueError, 'probe_invalid_front_lane'):
            self.clearance(scan, front_lane=lane(width=1.))
        with self.assertRaisesRegex(ValueError, 'probe_invalid_front_lane'):
            live.probe_clearance(scan, {'lidar': .01, 'control': .01}, camera_required=False,
                                front_lane=scan['corridor_candidates'][0])
        self.assertNotIn('front_wall_projected_bins', self.clearance(scan))


class LaneProjectionServiceScope(unittest.TestCase):
    def setUp(self):
        self.console = server.Console.__new__(server.Console)
        self.console.scan, self.console.scan_at = scan_with_lane(range(6, 18)), 1.
        self.motion = ManeuverSequence(0., continue_route=True, max_drive_s=40.)
        self.motion.entry_kind, self.motion.route_stage, self.motion.phase = 'lane', 'start_lane', 'drive'
        self.motion.drive_since = .1
        self.session = {'turn_trial': True, 'continue_route': True, 'turn_motion': self.motion, 'phase': 'drive'}

    def selected(self):
        return self.console.turn_front_lane(self.session, 1.01)

    def test_only_two_driving_open_course_lane_stages_qualify(self):
        for stage in ('start_lane', 'light_lane', 'first_target', 'open_transfer', 'second_loop', 'light_entry'):
            self.motion.route_stage = stage
            self.assertEqual(self.selected() is not None, stage in ('start_lane', 'light_lane'), stage)
        self.motion.route_stage = 'light_lane'
        self.motion.entry_kind = 'junction'
        self.assertIsNone(self.selected())
        self.motion.entry_kind = 'lane'
        for flag in ('turn_trial', 'continue_route'):
            self.session[flag] = False
            self.assertIsNone(self.selected())
            self.session[flag] = True
        for phase in ('presteer', 'coast'):
            self.session['phase'] = self.motion.phase = phase
            self.assertIsNone(self.selected())
        self.session['phase'] = self.motion.phase = 'quality_wait'
        self.assertIsNotNone(self.selected())

    def test_missing_ambiguous_crosswise_and_bad_geometry_are_not_a_lane(self):
        original = self.console.scan['corridor_candidates']
        cases = ([], [lane(math.pi/2)], [lane(), lane(.4)], [lane(), {'candidate_only': True}])
        for candidates in cases:
            self.console.scan['corridor_candidates'] = candidates
            self.assertIsNone(self.selected())
        self.console.scan['corridor_candidates'] = original+[lane(.01)]
        self.assertIsNotNone(self.selected())  # Existing native near-duplicate merge rule.

    def test_stale_or_regressed_scan_does_not_refresh_any_motion_state(self):
        before = copy.deepcopy(self.motion.__dict__)
        self.assertIsNotNone(self.selected())
        for key in ('last_seq', 'last_publication', 'last_receive', 'corridor'):
            self.assertEqual(getattr(self.motion, key), before[key])
        self.console.scan_at = .7
        self.assertIsNone(self.selected())
        self.console.scan_at = 1.
        self.motion.last_seq = 11
        self.assertIsNone(self.selected())
        self.motion.last_seq = 10
        self.motion.last_publication = 1001
        self.assertIsNone(self.selected())

    def test_same_scan_candidate_type_changes_are_revalidated_every_call(self):
        candidate = self.console.scan['corridor_candidates'][0]
        self.assertIs(self.selected(), candidate)
        for field, bad in (('left_wall_points', 60.), ('right_wall_points', True),
                           ('width_m', True), ('origin_between_walls', 1)):
            with self.subTest(field=field):
                before = candidate[field]
                candidate[field] = bad
                self.assertIsNone(self.selected())
                candidate[field] = before
                self.assertIs(self.selected(), candidate)
        # Invalid candidates are rejected even if a different valid candidate
        # still defines a unique forward lane. The internal validated path
        # must never turn filtering into permission to ignore invalid input.
        self.console.scan['corridor_candidates'].append(lane(math.pi))
        self.console.scan['corridor_candidates'][-1]['left_wall_points'] = 60.
        self.assertIsNone(self.selected())

    def test_evidence_is_recomputed_with_same_current_lane_and_rejects_forgery(self):
        scan = self.console.scan
        native = self.selected()
        evidence = live.front_gap_evidence(scan['ranges'], bridge=True, front_lane=native)
        self.assertEqual(self.console.turn_front_evidence_matches(scan, evidence, native), evidence)
        self.assertIsNone(self.console.turn_front_evidence_matches(scan, evidence))
        forged = copy.deepcopy(evidence)
        forged['front_wall_projection']['hits'][0]['range_m'] += 1.
        self.assertIsNone(self.console.turn_front_evidence_matches(scan, forged, native))


ROOT = Path(__file__).resolve().parents[3]
TOOL = ROOT/'tools'/'scan-geometry'/'target'/'release'/'xt-stcar-scan-geometry'


@unittest.skipUnless(TOOL.exists(), 'tools/scan-geometry not built')
class NativeLaneAvailability(unittest.TestCase):
    def native(self, nulls):
        scan = scan_with_lane(nulls)
        result = subprocess.run([str(TOOL)], input=json.dumps(scan)+'\n',
                                capture_output=True, text=True, check=True)
        scan.update(json.loads(result.stdout))
        return scan

    def test_native_short_grazing_gap_is_explained_from_same_scan(self):
        scan = self.native(range(6, 11))
        lanes = ManeuverSequence._forward_lanes(scan['corridor_candidates'])
        self.assertEqual(len(lanes), 1)
        evidence = live.front_gap_evidence(scan['ranges'], bridge=True, front_lane=lanes[0])
        self.assertEqual(evidence['front_wall_projected_bins'], list(range(6, 11)))
        self.assertEqual(evidence['front_open_unknown_bins'], [])

    def test_native_long_gap_removes_lane_so_it_cannot_be_explained(self):
        # Rust's unchanged full-scan validation rejects >5 missing degrees.
        # The proposed wall projection cannot rescue the real 12-bin failure
        # if that same scan no longer publishes a native lane.
        scan = self.native(range(6, 18))
        self.assertEqual(scan['corridor_candidates'], [])
        evidence = live.front_gap_evidence(scan['ranges'], bridge=True)
        self.assertEqual(evidence['front_open_unknown_bins'], list(range(6, 18)))
        self.assertEqual(evidence['largest_open_gap_deg'], 12)

    def test_native_opening_outside_front_sector_still_refutes_wall_extension(self):
        # The wall fit near the origin remains valid when a forward ray at
        # 31 deg sees through a board end. It must refute extension just as
        # the same observation at the front-sector boundary would.
        for angle in (30, 31, -30, -31):
            sign = 1 if angle > 0 else -1
            scan = scan_with_lane([sign*i for i in range(6, 11)])
            scan['ranges'][angle % 360] = 8.
            result = subprocess.run([str(TOOL)], input=json.dumps(scan)+'\n',
                                    capture_output=True, text=True, check=True)
            scan.update(json.loads(result.stdout))
            lanes = ManeuverSequence._forward_lanes(scan['corridor_candidates'])
            self.assertEqual(len(lanes), 1)
            evidence = live.front_gap_evidence(scan['ranges'], bridge=True, front_lane=lanes[0])
            self.assertEqual(evidence['front_wall_projected_bins'], [], angle)
            self.assertEqual(evidence['front_open_unknown_bins'], sorted(sign*i for i in range(6, 11)))


if __name__ == '__main__':
    unittest.main()
