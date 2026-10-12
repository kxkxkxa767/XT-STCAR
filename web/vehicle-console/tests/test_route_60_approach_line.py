"""Round 60 (Claude): steer the exit alignment/corridor following toward the
second-support bypass side (left of the lane center) before that support is
confirmed, and bound the second orbit by its own stage deadline.
Real trial59 frames: fixtures/route-59-second-board.json (saved scans only)."""
import copy
import json
import math
from pathlib import Path
import unittest

import test_maneuver_route as route
from test_maneuver_route import observation, feedback
from test_maneuver_sequence import corridor
from maneuver_sequence import (ManeuverSequence, SECOND_APPROACH_LEFT_M, SECOND_APPROACH_WALL_MARGIN_M,
                               SECOND_BYPASS_SUPPORT_GAP_M, SECOND_ORBIT_MAX_S, ORBIT_ENTRY_MAX_S,
                               SIDE_BODY_EXTENT_M, MANEUVER_BODY_CLEARANCE_M, LEFT_TRIAL_GAIN)

FIXTURE = json.loads((Path(__file__).parent/'fixtures'/'route-59-second-board.json').read_text())


def real(seq):
    scan = copy.deepcopy(FIXTURE['scans'][str(seq)])
    scan['received_at'] = scan['at_ms']/1000
    return scan


def staged(stage, *, route_mode=True, passed=True):
    harness = route.ManeuverRouteTests()
    harness.setUp()
    motion = harness.route(stage)
    motion.continue_route = route_mode
    if not passed:
        motion.first_pass_evidence = None
    motion.right_exit_geometry = None
    motion._right_exit_last_receive = motion._right_exit_last_publication = None
    return motion


class ApproachLineTests(unittest.TestCase):
    def test_offset_is_the_bypass_gap_and_only_between_first_pass_and_second_handover(self):
        self.assertEqual(SECOND_APPROACH_LEFT_M, SECOND_BYPASS_SUPPORT_GAP_M)
        lane = corridor(0.)
        lane['width_m'] = 2.4
        for stage in ('right_align', 'corridor_follow'):
            self.assertEqual(staged(stage)._approach_offset(lane), SECOND_APPROACH_LEFT_M)
            self.assertEqual(staged(stage, route_mode=False)._approach_offset(lane), 0.)
            self.assertEqual(staged(stage, passed=False)._approach_offset(lane), 0.)
        for stage in ('first_target', 'second_orbit', 'light_turn', 'light_lane'):
            self.assertEqual(staged(stage)._approach_offset(lane), 0.)

    def test_offset_is_bounded_by_the_measured_width(self):
        motion = staged('corridor_follow')
        margin = SIDE_BODY_EXTENT_M+MANEUVER_BODY_CLEARANCE_M+SECOND_APPROACH_WALL_MARGIN_M
        for width, expected in ((2.4, SECOND_APPROACH_LEFT_M), (1.2, .6-margin), (2*margin, 0.), (.5, 0.)):
            lane = corridor(0.)
            lane['width_m'] = width
            self.assertAlmostEqual(motion._approach_offset(lane), expected)
            # The steered line keeps the wall margin on the left side.
            self.assertLessEqual(motion._approach_offset(lane)+SIDE_BODY_EXTENT_M+MANEUVER_BODY_CLEARANCE_M,
                                 max(width/2-SECOND_APPROACH_WALL_MARGIN_M,
                                     SIDE_BODY_EXTENT_M+MANEUVER_BODY_CLEARANCE_M)+1e-9)

    def test_real59_corridor_steers_toward_the_bypass_side_before_the_second_support(self):
        # Trial59 482 (corridor following): measured lane 2.08 m, car ~.08 m
        # left of center, heading ~3 deg left of the axis; recorded 1493.
        biased = staged('corridor_follow')
        center = staged('second_orbit')  # same measurement, no approach offset
        target, rejection = biased._right_exit_target(real(482), real(482)['received_at'])
        plain, _ = center._right_exit_target(real(482), real(482)['received_at'])
        self.assertIsNone(rejection)
        geometry = biased.right_exit_geometry
        self.assertEqual(geometry['approach_offset_left_m'], SECOND_APPROACH_LEFT_M)
        self.assertEqual(center.right_exit_geometry['approach_offset_left_m'], 0.)
        self.assertLess(plain, 1500)  # toward the center line: what trial59 did
        self.assertGreater(target, 1500)  # toward the bypass side, gently
        expected = 1500+round(LEFT_TRIAL_GAIN*(geometry['heading_left_rad'] + math.atan2(
            geometry['center_offset_left_m']+SECOND_APPROACH_LEFT_M, geometry['lookahead_m'])))
        self.assertEqual(target, expected)
        self.assertLess(target, 1560)

    def test_corridor_follow_update_uses_the_approach_line_and_keeps_its_bounds(self):
        motion = staged('corridor_follow')
        # On the approach line, straight along the lane: neutral.
        result = motion.update(observation(5, heading=0., offset=-SECOND_APPROACH_LEFT_M), 0., .5,
                               feedback(5, motion.servo, 1550))
        self.assertEqual(result['reason'], 'route_corridor_follow_current_geometry')
        self.assertEqual(result['steering_target'], 1500)
        # On the center line: a bounded left toward the approach line.
        motion = staged('corridor_follow')
        result = motion.update(observation(5, heading=0., offset=0.), 0., .5, feedback(5, motion.servo, 1550))
        self.assertGreater(result['steering_target'], 1500)
        self.assertLessEqual(result['steering_target'], 1555)

    def test_second_orbit_lane_measurement_is_not_offset(self):
        motion = staged('second_orbit')
        _, lane = motion._current_lane(real(486), real(486)['received_at'])
        self.assertIsNotNone(lane)
        self.assertEqual(lane['approach_offset_left_m'], 0.)


class SecondOrbitDeadlineTests(unittest.TestCase):
    def test_second_orbit_has_its_own_longer_stage_bound(self):
        self.assertEqual(ORBIT_ENTRY_MAX_S, 3.0)  # first orbit unchanged
        self.assertEqual(SECOND_ORBIT_MAX_S, 6.0)
        harness = route.ManeuverRouteTests()
        harness.setUp()
        motion, result = harness.second_handover()
        since = motion.second_orbit_since
        reasons = {}
        for seq in range(9, 75):
            # The support stays ahead-right (no pass); the stage must not end
            # at the first orbit's 3 s bound, only at its own.
            result = harness.step(motion, seq, second=-45., heading=0., motor=1550)
            reasons[seq] = (round(seq/10-since, 2), result['route_stage'], result['reason'])
            if result['route_stage'] != 'second_orbit':
                break
        elapsed, stage, reason = reasons[seq]
        self.assertEqual(reason, 'second_target_entry_timeout_back_to_corridor')
        self.assertGreaterEqual(elapsed, SECOND_ORBIT_MAX_S-1e-6)
        self.assertTrue(all(r[1] == 'second_orbit' for s, r in reasons.items() if r[0] < SECOND_ORBIT_MAX_S-1e-6))
        self.assertEqual(result['second_orbit_max_s'], SECOND_ORBIT_MAX_S)


if __name__ == '__main__':
    unittest.main()
