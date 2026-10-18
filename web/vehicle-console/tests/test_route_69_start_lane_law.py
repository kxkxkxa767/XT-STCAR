"""Start-lane wall law (2026-10-18). Rounds 64/66/67/68 weaved +-8 deg in the
start lane: the old law (CorridorSteering, ~200 nominal PWM per rad behind a
10 PWM/100 ms ramp, no lead) is past its stability limit when the real
near-centre steering answers 1.5-3x the table about .35 s late. The new law
turns toward the lane by the measured error, led by the measured wall-heading
trend, with a small gain; the start-lane speed needs three closing rates."""
import importlib.util
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).parents[1]))
import maneuver_sequence as ms  # noqa: E402
from maneuver_sequence import (ManeuverSequence, LaneWallSteering, LANE_WALL_CAP_PWM, LANE_WALL_GAIN_PWM_PER_RAD,
                               LANE_WALL_LEAD_S, LANE_WALL_LOOKAHEAD_M, NEUTRAL)  # noqa: E402
from autonomy_live import CorridorSteering  # noqa: E402

ROOT = Path(__file__).parents[3]


def walls(heading_deg=0., offset=0., width=.93):
    return {'heading_right_deg': heading_deg, 'offset_right_m': offset,
            'slope': math.tan(math.radians(heading_deg)), 'width_m': width}


class LaneWallLaw(unittest.TestCase):
    def test_turns_toward_the_lane_by_the_error(self):
        law = LaneWallSteering()
        self.assertLess(law.update(walls(2.), 1, 0., True, 100), NEUTRAL)  # lane turns right of the nose
        law = LaneWallSteering()
        self.assertGreater(law.update(walls(-2.), 1, 0., True, 100), NEUTRAL)
        law = LaneWallSteering()
        self.assertLess(law.update(walls(0., .05), 1, 0., True, 100), NEUTRAL)  # centre to the right
        law = LaneWallSteering()
        expected = round(LANE_WALL_GAIN_PWM_PER_RAD*(math.radians(1.)+.03/LANE_WALL_LOOKAHEAD_M))
        self.assertEqual(law.update(walls(1., .03), 1, 0., True, 100), NEUTRAL-expected)

    def test_capped(self):
        law = LaneWallSteering()
        self.assertEqual(law.update(walls(12., .2), 1, 0., True, 100), NEUTRAL-LANE_WALL_CAP_PWM)
        law = LaneWallSteering()
        self.assertEqual(law.update(walls(-12., -.2), 1, 0., True, 100), NEUTRAL+LANE_WALL_CAP_PWM)

    def test_lead_releases_before_the_error_is_gone(self):
        # Heading 4 -> 3 -> 2 deg over .2 s: the car is already turning back
        # at 10 deg/s, so with the lead the command is neutral or opposite
        # while 2 deg of error remain (the old law still pushed right).
        law = LaneWallSteering()
        for k, h in enumerate((4., 3., 2.)):
            out = law.update(walls(h), k+1, 0., True, 100*(k+1))
        self.assertGreaterEqual(out, NEUTRAL)
        self.assertAlmostEqual(law.evidence['trend_rad_s'], math.radians(-10.))
        self.assertAlmostEqual(law.evidence['predicted_rad'], math.radians(2.)+LANE_WALL_LEAD_S*math.radians(-10.))
        old = CorridorSteering()
        for k, h in enumerate((4., 3., 2.)):
            old_out = old.update(walls(h, .03), k+1, k*.1, True, 100*(k+1))
        self.assertLessEqual(old_out, NEUTRAL)

    def test_trend_needs_three_consistent_publications(self):
        law = LaneWallSteering()
        law.update(walls(4.), 1, 0., True, 100)
        law.update(walls(3.), 2, 0., True, 200)
        self.assertEqual(law.evidence['trend_rad_s'], 0.)
        law = LaneWallSteering()
        law.update(walls(4.), 1, 0., True, 100)
        law.update(walls(3.), 2, 0., True, 200)
        law.update(walls(2., .3), 3, 0., True, 300)  # offset jump: another fit, restart
        self.assertEqual((law.evidence['history'], law.evidence['trend_rad_s']), (1, 0.))
        law = LaneWallSteering()
        law.update(walls(4.), 1, 0., True, 100)
        law.update(walls(3.), 2, 0., True, 200)
        law.update(walls(2.), 3, 0., True, 700)  # .5 s gap: not one trend
        self.assertEqual((law.evidence['history'], law.evidence['trend_rad_s']), (1, 0.))

    def test_same_scan_holds_and_bad_order_releases(self):
        law = LaneWallSteering()
        first = law.update(walls(3.), 5, 0., True, 500)
        self.assertEqual(law.update(walls(-9.), 5, .05, True, 500), first)
        self.assertEqual(law.update(walls(3.), 4, .1, True, 600), NEUTRAL)  # seq went back
        law = LaneWallSteering()
        law.update(walls(3.), 5, 0., True, 500)
        self.assertEqual(law.update(walls(3.), 6, .1, True, 500), NEUTRAL)  # publication did not advance
        self.assertEqual(law.update(None, 7, .2, True, 700), NEUTRAL)
        self.assertEqual(law.update(walls(3.), 8, .3, False, 800), NEUTRAL)

    def test_only_the_start_lane_uses_it(self):
        motion = ManeuverSequence(0., continue_route=True, max_drive_s=40.)
        self.assertIsInstance(motion._lane_start_steering, LaneWallSteering)
        self.assertIsInstance(motion._lane_steering, CorridorSteering)


class StartLaneSpeed(unittest.TestCase):
    def feed(self, motion, ranges, **kw):
        out = []
        for k, ahead in enumerate(ranges):
            scan = {'at_ms': 1000+100*k, 'received_at': 10.+.1*k}
            out.append(motion._lane_speed(scan, ahead, **kw))
        return out

    def test_default_unchanged_three_rates_for_the_start_lane(self):
        motion = ManeuverSequence(0., continue_route=True, max_drive_s=40.)
        default = self.feed(motion, [7., 6.9, 6.8, 6.7, 6.6])
        self.assertEqual(default[:2], [None, None])
        self.assertAlmostEqual(default[2], 1.)
        motion = ManeuverSequence(0., continue_route=True, max_drive_s=40.)
        start = self.feed(motion, [7., 6.9, 6.8, 6.7, 6.6], min_rates=3)
        self.assertEqual(start[:4], [None]*4)
        self.assertAlmostEqual(start[4], 1.)

    def test_one_jump_of_the_far_ray_is_rejected(self):
        # 2026-10-18 simulation: 7.29 -> 7.02 -> 7.05 at ~.9 m/s gave 2.65 m/s
        # from two rates and cut the start lane.
        motion = ManeuverSequence(0., continue_route=True, max_drive_s=40.)
        self.assertGreater(self.feed(motion, [7.29, 7.02, 7.05])[-1], 2.)
        motion = ManeuverSequence(0., continue_route=True, max_drive_s=40.)
        speeds = self.feed(motion, [7.29, 7.02, 7.05, 6.96, 6.87], min_rates=3)
        self.assertLess(speeds[-1], 1.)


class ClosedLoopWeave(unittest.TestCase):
    """course-sim, .94 m start lane, real-like steering (wheels start on the
    right side of the play as in round 68, near-centre gain 2, delay .3)."""
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location('course_sim_69_law', ROOT/'scripts/course-sim.py')
        cls.cs = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.cs)

    def run_lane(self, params, delay):
        cs = self.cs
        course = cs.Course(**cs.layout(8.1, 4.15, .94, 3.2))
        summary, log, _ = cs.run(course, max_s=8., vehicle='measured', delay=delay, vehicle_params=params)
        rows = [r for r in log if r[7] == 'start_lane' and r[8] == 'drive' and r[1] <= 3.2]
        head = [r[3] for r in rows]
        return summary, rows, max(head)-min(head)

    def test_start_lane_swing_small_without_a_lane_halt(self):
        for params, delay in (({'play_pwm': 50., 'near_center_gain': 2., 'initial_wheel_pwm': 1450.}, .3),
                              ({'play_pwm': 50., 'near_center_gain': 3., 'initial_wheel_pwm': 1550.}, .25),
                              ({}, .15)):
            with self.subTest(params=params, delay=delay):
                summary, rows, swing = self.run_lane(params, delay)
                self.assertGreater(rows[-1][1], 3.)  # left the start lane by driving, not a stop
                self.assertLess(swing, 8.)
                self.assertGreater(min(r[10] for r in rows), .15)
                self.assertEqual(summary['contacts'], [])

    def test_initial_wheel_must_lie_inside_the_play(self):
        with self.assertRaises(ValueError):
            self.cs.MeasuredVehicle(0., 0., 0., play_pwm=20., initial_wheel_pwm=1450.)


if __name__ == '__main__':
    unittest.main()
