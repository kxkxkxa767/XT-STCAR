"""Round 107 (Claude): the open course's first loop sends its left commands to
the steering output divided by FIRST_LOOP_STEER_GAIN (car runs 104-106-8
curved 1.2-1.4 times the calibration there and passed the first support at
.05-.095 m). The nominal command, the planners and every other stage are
unchanged. Synthetic states; no physical claim."""
from pathlib import Path
import sys
import types
import unittest

sys.path.insert(0, str(Path(__file__).parents[1]))
import maneuver_sequence as ms  # noqa: E402
import server  # noqa: E402
import steer_calibration as sc  # noqa: E402


def console(motion):
    return types.SimpleNamespace(auto_session={'steer_output': sc.SteerOutput(), 'turn_motion': motion},
                                 steer_sent={})


def motion(stage='first_target', phase='drive', entry='lane', route=True):
    m = ms.ManeuverSequence.__new__(ms.ManeuverSequence)
    m.continue_route, m.entry_kind, m.route_stage, m.phase = route, entry, stage, phase
    m.reason, m.start_lane_coast_steering = 'open_first_loop', None
    return m


def physical(m, servo, seq=1):
    c = console(m)
    out = server.Console.output_servo(c, 'drive', servo, seq)
    return out, c.steer_sent[seq]


class FirstLoopSteerGain(unittest.TestCase):
    def test_only_the_open_first_loop_drive(self):
        self.assertTrue(motion().first_loop_output_active())
        for kwargs in ({'stage': 'start_lane'}, {'stage': 'open_transfer'}, {'stage': 'second_loop'},
                       {'phase': 'coast'}, {'entry': 'corridor'}, {'route': False}):
            with self.subTest(**kwargs):
                self.assertFalse(motion(**kwargs).first_loop_output_active())

    def test_left_commands_reach_the_output_divided_by_the_gain(self):
        for servo in (1560, 1660, 1700, 1720):
            scaled = ms.NEUTRAL+round((servo-ms.NEUTRAL)/ms.FIRST_LOOP_STEER_GAIN)
            out, sent = physical(motion(), servo)
            with self.subTest(servo=servo):
                self.assertEqual(out, sc.SteerOutput().physical(scaled))
                self.assertLess(out, sc.SteerOutput().physical(servo))
                self.assertEqual(sent, (servo, out))  # feedback still maps to the nominal command

    def test_right_centre_and_other_stages_are_unchanged(self):
        for servo in (1308, 1450, 1500):
            out, _ = physical(motion(), servo)
            with self.subTest(servo=servo):
                self.assertEqual(out, sc.SteerOutput().physical(servo))
        for stage in ('open_transfer', 'second_loop', 'light_entry'):
            out, _ = physical(motion(stage=stage), 1660)
            with self.subTest(stage=stage):
                self.assertEqual(out, sc.SteerOutput().physical(1660))

    def test_the_gain_puts_a_commanded_circle_where_the_car_curved(self):
        # A .95 m circle (1.05 /m, nominal 1668) on a car curving
        # FIRST_LOOP_STEER_GAIN times the calibration reaches the curvature
        # the planner meant.
        nominal = ms.NEUTRAL+round(ms.ROUTE_PWM_PER_CURVATURE/.95)
        scaled = ms.NEUTRAL+round((nominal-ms.NEUTRAL)/ms.FIRST_LOOP_STEER_GAIN)
        wheel = sc.wheel_for_curvature((scaled-ms.NEUTRAL)/sc.NOMINAL_PWM_PER_CURVATURE)
        self.assertAlmostEqual(ms.FIRST_LOOP_STEER_GAIN*sc.curvature_for_wheel(wheel), 1/.95, delta=.02)


if __name__ == '__main__':
    unittest.main()
