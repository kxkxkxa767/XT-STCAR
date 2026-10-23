"""Round 104 (Claude): a new start-lane 1560 push keeps its own full ESC-lag
rise under the allowed speed (car run 103: one 1560 scan at 1.23 m/s, allowed
1.66, cut the next four scans). Values from run 103's start_lane_feedback;
no physical claim."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).parents[1]))
import maneuver_sequence as ms  # noqa: E402
from maneuver_sequence import ManeuverSequence  # noqa: E402
from turn_motion import NEUTRAL  # noqa: E402

gov = ManeuverSequence._start_governor
FULL_RISE = ms.LANE_POWER_ACCEL_MPS2*ms.ESC_LAG_S


class StartPush(unittest.TestCase):
    def test_run_103_scan_37_holds_1550_instead_of_a_push_it_must_cut(self):
        # Seq 37: 1.23 m/s at 1550 (no rise), allowed 1.66; the old band
        # rule pushed (1.23 < 1.26), and seq 38 read 1.31 + .43 >= 1.62.
        self.assertEqual(gov(1.23, 0., 1.66, True), ms.ROUTE_SLOW_MOTOR)
        self.assertEqual(gov(1.31, .43, 1.62, True), NEUTRAL)  # the cut it caused
        self.assertEqual(gov(1.31, 0., 1.62, True), ms.ROUTE_SLOW_MOTOR)  # no push, no cut

    def test_a_push_whose_rise_fits_still_goes(self):
        # Seq 33: 1.27 m/s, allowed 1.78: 1.27 + .45 < 1.78.
        self.assertEqual(gov(1.27, 0., 1.78, True), ms.TRIAL_MOTOR)
        self.assertEqual(gov(.6, 0., 1.8, True), ms.TRIAL_MOTOR)

    def test_a_push_under_way_keeps_the_band_rule(self):
        # Launch: rise near its full value, so the push rule adds nothing.
        for speed in (.3, .6, .9):
            for rise in (.40, FULL_RISE):
                with self.subTest(speed=speed, rise=rise):
                    band = speed+rise < 1.8-ms.LANE_START_PUSH_BAND_MPS
                    self.assertEqual(gov(speed, rise, 1.8, True), ms.TRIAL_MOTOR if band else ms.ROUTE_SLOW_MOTOR)

    def test_restart_and_launch_are_unchanged(self):
        self.assertEqual(gov(.05, 0., 1.5, True), ms.START_LAUNCH_MOTOR)
        self.assertEqual(gov(.1, 0., .5, True), ms.TRIAL_MOTOR)  # below LANE_SPEED_RESTART_MPS
        self.assertEqual(gov(None, 0., 1.5), ms.TRIAL_MOTOR)
        self.assertEqual(gov(1.0, .7, 1.6, True), NEUTRAL)


if __name__ == '__main__':
    unittest.main()
