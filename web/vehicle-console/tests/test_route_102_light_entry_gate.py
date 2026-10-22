"""Round 102 (Claude): a blocked light-entry line moves to a free parallel line
(the traffic-light gantry's legs in the light-lane mouth, car run 101) and a
light-entry pause steps its lane-axis hold at the open-stage steps. Synthetic
scans; no physical claim."""
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).parents[1]))
import maneuver_sequence as ms  # noqa: E402
from maneuver_sequence import (ManeuverSequence, OPEN_PWM_STEP, OPEN_RELEASE_PWM_STEP,  # noqa: E402
                               SIDE_BODY_EXTENT_M, OPEN_APPROACH_MARGIN_M)
from turn_motion import NEUTRAL, PWM_STEP  # noqa: E402

POST_RADIUS = .025
OUTER = ((-1., .55), (4., .55))
# Car run 101, seq 115 (axis frame): the outer board .55 m left, the gantry
# legs 2.8 m ahead at +.19 / -.77 (.96 m apart).
GATE = ((2.8, .19), (2.8, -.77))
LANE = {'outer_lateral_m': .55, 'inner_lateral_m': None, 'inner_start_along_m': None,
        'width_m': .89, 'center_lateral_m': .55-.89/2}


def cast(posts=(), boards=(), heading=0.):
    """Ranges (bin i = i deg clockwise) of posts and boards given in the lane-axis
    frame from the origin, the car yawed ``heading`` left of the axis."""
    ranges = []
    for i in range(360):
        a = heading-math.radians(i)
        dx, dy = math.cos(a), math.sin(a)
        best = 8.
        for cx, cy in posts:
            b = cx*dx+cy*dy
            disc = b*b-(cx*cx+cy*cy-POST_RADIUS**2)
            if disc >= 0 and b-math.sqrt(disc) > 0:
                best = min(best, b-math.sqrt(disc))
        for (x1, y1), (x2, y2) in boards:
            ex, ey = x2-x1, y2-y1
            den = dx*ey-dy*ex
            if abs(den) < 1e-12:
                continue
            t, u = (x1*ey-y1*ex)/den, (x1*dy-y1*dx)/den
            if t > 1e-6 and 0 <= u <= 1:
                best = min(best, t)
        ranges.append(round(best, 4))
    return ranges


def scan(seq, ranges):
    return {'seq': seq, 'at_ms': 100*seq, 'received_at': seq/10, 'ranges': ranges}


class FreeLateralTests(unittest.TestCase):
    def test_line_through_the_outer_leg_moves_into_the_gate(self):
        for heading in (0., math.radians(29)):
            motion = ManeuverSequence(0., continue_route=True)
            value = scan(5, cast(GATE, (OUTER,), heading))
            # The measured line itself meets the outer leg.
            point = (1.4*math.cos(-heading)-LANE['center_lateral_m']*math.sin(-heading),
                     1.4*math.sin(-heading)+LANE['center_lateral_m']*math.cos(-heading))
            self.assertIsNotNone(motion._open_entry_path_hit(value, point, heading, 2.89))
            lateral = motion._open_entry_free_lateral(value, heading, LANE, 2.89)
            with self.subTest(heading=heading):
                for _, y in GATE:  # body side to each leg
                    self.assertGreater(abs(lateral-y)-POST_RADIUS-SIDE_BODY_EXTENT_M, .25)

    def test_shift_returns_the_free_pursuit_and_its_evidence(self):
        motion = ManeuverSequence(0., continue_route=True)
        value = scan(5, cast(GATE, (OUTER,)))
        with patch.object(motion, '_open_lane_target', return_value=dict(LANE)):
            nominal = motion._open_light_pursuit(value, 0.)
            moved, ahead, evidence = motion._open_entry_shift(value, 0., dict(LANE), None, 2.89, None, 1.)
        self.assertIsNone(ahead)
        self.assertLess(moved[3], nominal[3])  # more right, toward the gate
        self.assertEqual(evidence['line_lateral_m'], LANE['center_lateral_m'])
        self.assertAlmostEqual(evidence['lateral_m'], -.25, delta=.02)
        self.assertEqual(evidence['blocked_front_range_m'], 1.)

    def test_a_narrow_slot_beside_the_board_loses_to_the_lane(self):
        # A post .5 m inside the outer board: the slot between them is free but
        # narrower than OPEN_LANE_SHIFT_MIN_GAP_M; the line passes inside it.
        motion = ManeuverSequence(0., continue_route=True)
        post = (2.5, .05)
        lateral = motion._open_entry_free_lateral(scan(5, cast((post,), (OUTER,))), 0., LANE, 2.89)
        half = SIDE_BODY_EXTENT_M+OPEN_APPROACH_MARGIN_M
        self.assertLess(lateral, post[1]-POST_RADIUS-half)

    def test_board_across_the_lane_leaves_no_line(self):
        motion = ManeuverSequence(0., continue_route=True)
        end = ((2., -1.5), (2., 1.5))
        self.assertIsNone(motion._open_entry_free_lateral(scan(5, cast((), (OUTER, end))), 0., LANE, 2.89))
        self.assertIsNone(motion._open_entry_free_lateral({'seq': 5}, 0., LANE, 2.89))

    def test_never_outside_the_outer_board(self):
        motion = ManeuverSequence(0., continue_route=True)
        lateral = motion._open_entry_free_lateral(scan(5, cast(((2.5, -.2),), (OUTER,))), 0., LANE, 2.89)
        self.assertLessEqual(lateral, LANE['outer_lateral_m']-SIDE_BODY_EXTENT_M-OPEN_APPROACH_MARGIN_M)


class EntryStageTests(unittest.TestCase):
    def entry(self, value, free=True):
        motion = ManeuverSequence(0., continue_route=True)
        motion.route_stage, motion.phase = 'light_entry', 'drive'
        holds = []
        patches = [patch.object(motion, '_route_adopted', return_value=True),
                   patch.object(motion.compass, 'current', return_value=0.),
                   patch.object(motion, '_open_lane_target', side_effect=lambda *a: dict(LANE)),
                   patch.object(motion, '_open_front_stop', return_value=(None, None, 2.89)),
                   patch.object(motion, '_begin_recovery_hold', side_effect=lambda cause, *a: holds.append(cause)),
                   patch.object(ms, 'corridor_walls', return_value=None)]
        if not free:
            patches.append(patch.object(motion, '_open_entry_free_lateral', return_value=None))
        for p in patches:
            p.start()
        try:
            motion._open_light_entry_update(value, 1., {'servo': NEUTRAL, 'motor': 1550})
        finally:
            for p in reversed(patches):
                p.stop()
        return motion, holds

    def test_blocked_line_steers_through_the_gate_instead_of_holding(self):
        motion, holds = self.entry(scan(5, cast(GATE, (OUTER,))))
        self.assertEqual(holds, [])
        self.assertEqual(motion.reason, 'open_light_entry')
        shift = motion.light_feedback['line_shift']
        self.assertAlmostEqual(shift['lateral_m'], -.25, delta=.02)
        self.assertIsNone(motion.light_feedback['front_range_m'])
        self.assertLess(motion.natural_steering_target, NEUTRAL)

    def test_without_a_free_line_the_stage_holds_as_before(self):
        motion, holds = self.entry(scan(5, cast(GATE, (OUTER,))), free=False)
        self.assertEqual(holds, ['open_board_ahead_before_light_lane'])
        self.assertIsNone(motion.light_feedback['line_shift'])

    def test_a_free_measured_line_is_unchanged(self):
        motion, holds = self.entry(scan(5, cast((), (OUTER,))))
        self.assertEqual(holds, [])
        self.assertIsNone(motion.light_feedback['line_shift'])

    def test_pause_resumes_where_the_drive_takes_the_free_line(self):
        for free in (True, False):
            motion = ManeuverSequence(0., continue_route=True)
            motion.route_stage, motion.neutral_pause_kind = 'light_entry', 'recovery'
            with patch.object(motion.compass, 'current', return_value=0.), \
                 patch.object(motion, '_open_front_stop', return_value=(None, 0., 2.89)), \
                 patch.object(motion, '_open_lane_target', side_effect=lambda *a: dict(LANE)), \
                 patch.object(ms, 'corridor_walls', return_value=None):
                if not free:
                    patch.object(motion, '_open_entry_free_lateral', return_value=None).start()
                ready = motion._pause_stage_ready(scan(5, cast(GATE, (OUTER,))), 1.)
                patch.stopall()
            with self.subTest(free=free):
                self.assertEqual(ready, free)


class PauseStepTests(unittest.TestCase):
    def paused(self, servo, hold):
        motion = ManeuverSequence(0., continue_route=True)
        motion.phase, motion.route_stage = 'quality_wait', 'light_entry'
        motion.first_pass_evidence = {'track_id': 1, 'source_seq': 4}
        motion.servo, motion._pause_steer_target = servo, hold
        motion.steering_target = hold if hold is not None else 1580
        motion.last_change = -1.
        return motion

    def test_light_entry_pause_steps_at_the_open_stage_steps(self):
        motion = self.paused(1421, 1580)
        motion._slew(1.)
        self.assertEqual(motion.servo, min(NEUTRAL, 1421+OPEN_RELEASE_PWM_STEP))  # release to centre first
        motion.servo, motion.last_change = NEUTRAL, -1.
        motion._slew(1.)
        self.assertEqual(motion.servo, NEUTRAL+OPEN_PWM_STEP)

    def test_other_waits_keep_the_slow_step(self):
        motion = self.paused(1421, None)
        motion._slew(1.)
        self.assertEqual(motion.servo, 1421+PWM_STEP)
        motion = self.paused(1421, 1580)
        motion.route_stage = 'second_loop'
        motion._slew(1.)
        self.assertEqual(motion.servo, 1421+PWM_STEP)


if __name__ == '__main__':
    unittest.main()
