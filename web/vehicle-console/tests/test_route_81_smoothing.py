"""Round 81: car run 80 smoothing (first-loop radius, light-lane centre ACK).
Synthetic scans; no physical claims."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).parents[1]))
import maneuver_sequence as ms
import steer_calibration as sc
import test_maneuver_route as route


class FirstLoopRadius(unittest.TestCase):
    def test_left_circle_is_drivable_at_the_physical_limit(self):
        # Physical 1720 leaves the wheels at 1720-play; that curvature's radius.
        tightest = 1/sc.curvature_for_wheel(sc.PHYSICAL_RANGE[1]-sc.STEER_PLAY_PWM)
        self.assertGreater(ms.OPEN_LEFT_RADIUS_M[0], tightest)
        self.assertLessEqual(ms.OPEN_LEFT_RADIUS_M[0], ms.OPEN_LEFT_RADIUS_M[1])
        self.assertTrue(0 < ms.OPEN_TURN_IN_LEAD <= 1)


class LightLaneCenterAck(unittest.TestCase):
    """Car run 80: the slow light lane coasted at 1500 while right needed a
    centre ACK that is recorded only under drive power."""
    def test_blocked_right_gets_one_drive_pulse_then_steers(self):
        t = route.LightLaneTests('test_off_center_lane_is_not_aligned_and_steers_back_to_center')
        motion = t.light()
        for seq in (5, 6, 7, 8, 9):
            result = t.step(motion, seq, route.lane())
            if result['route_stage'] == 'light_lane':
                break
        self.assertEqual(result['route_stage'], 'light_lane')
        motion._route_center_ack = None
        motion.lane_motor, motion._lane_slow_zone = ms.NEUTRAL, True
        motion._lane_speed = lambda scan, ahead: 1.  # above the slow band: the lane coasts
        motor, rows = ms.NEUTRAL, []
        for seq in range(seq+1, seq+7):
            value = route.observation(seq, heading=0.)
            value['ranges'] = route.segment_ranges(route.lane(offset=-.5))  # centre to the right
            value['corridor_candidates'] = []
            result = motion.update(value, 0., seq/10, route.feedback(seq, motion.servo, motor))
            rows.append((result['motor'], result['servo'], result['lane_feedback'].get('center_ack_pulse')))
            motor = result['motor']
        self.assertEqual(rows[0], (ms.ROUTE_SLOW_MOTOR, ms.NEUTRAL, True))  # one pulse at centre
        self.assertEqual([r[0] for r in rows[1:]], [ms.NEUTRAL]*5)  # then the lane coasts again
        self.assertTrue(any(servo < ms.NEUTRAL for _, servo, _ in rows))  # right once ACKed

    def test_no_pulse_where_the_coast_would_not_stop_short(self):
        t = route.LightLaneTests('test_off_center_lane_is_not_aligned_and_steers_back_to_center')
        motion = t.light()
        for seq in (5, 6, 7, 8, 9):
            if t.step(motion, seq, route.lane())['route_stage'] == 'light_lane':
                break
        motion._route_center_ack = None
        motion.lane_motor, motion._lane_slow_zone = ms.NEUTRAL, True
        motion._lane_speed = lambda scan, ahead: 1.
        motion._lane_safe_gap = lambda v, reaction: 99.  # every gap is inside the weakest coast
        value = route.observation(seq+1, heading=0.)
        value['ranges'] = route.segment_ranges(route.lane(offset=-.5))
        value['corridor_candidates'] = []
        result = motion.update(value, 0., (seq+1)/10, route.feedback(seq+1, motion.servo, ms.NEUTRAL))
        self.assertEqual(result['motor'], ms.NEUTRAL)
        self.assertIsNone(result['lane_feedback'].get('center_ack_pulse'))


if __name__ == '__main__':
    unittest.main()
