"""Rounds 81/82: car runs 80/81 smoothing (first-loop radius, light-lane centre
ACK while coasting, measured coast decel). Synthetic scans; no physical claims."""
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
    """Car runs 80/81: the light lane coasted at 1500 while right needed a
    centre ACK that was recorded only under drive power (round 82: coasting
    in the light lane proves centre too)."""
    def lane_motion(self):
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
        return motion, seq

    def test_coasting_centre_proves_centre_and_right_follows(self):
        motion, seq = self.lane_motion()
        rows = []
        for seq in range(seq+1, seq+6):
            value = route.observation(seq, heading=0.)
            value['ranges'] = route.segment_ranges(route.lane(offset=-.5))  # centre to the right
            value['corridor_candidates'] = []
            result = motion.update(value, 0., seq/10, route.feedback(seq, motion.servo, ms.NEUTRAL))
            rows.append((result['motor'], result['servo']))
        self.assertTrue(all(motor == ms.NEUTRAL for motor, _ in rows))  # no power needed
        self.assertEqual(motion._route_center_ack['motor'], ms.NEUTRAL)
        self.assertTrue(any(servo < ms.NEUTRAL for _, servo in rows))

    def test_other_stages_still_need_drive_power(self):
        motion = ms.ManeuverSequence(0., continue_route=True)
        motion.route_stage, motion.servo = 'open_transfer', ms.NEUTRAL
        control = {'servo': ms.NEUTRAL, 'motor': ms.NEUTRAL, 'command_acked': True, 'tick': 1, 'seq': 1}
        motion._route_adopted(control, 1.)
        self.assertIsNone(motion._route_center_ack)


class LightLaneCoastDecel(unittest.TestCase):
    def test_measured_coast_decel_replaces_the_weakest_within_bounds(self):
        motion = ms.ManeuverSequence(0., continue_route=True)
        motion.lane_motor, motion._power_cmd_at = ms.NEUTRAL, None
        weakest = motion._lane_safe_gap(1.)
        for k, v in enumerate((.9, .85, .8, .74, .7, .64)):  # ~.53 m/s^2
            motion._lane_coast_observe(k*.1, v)
        self.assertAlmostEqual(motion._lane_coast_decel, ms.LANE_STOP_DECEL_MPS2)  # clamped to nominal
        self.assertLess(motion._lane_safe_gap(1.), weakest)
        slow = ms.ManeuverSequence(0., continue_route=True)
        slow.lane_motor, slow._power_cmd_at = ms.NEUTRAL, None
        for k, v in enumerate((.9, .89, .88, .87, .86, .85)):  # .1 m/s^2: below the weakest
            slow._lane_coast_observe(k*.1, v)
        self.assertIsNone(slow._lane_coast_decel)  # drop under LANE_COAST_MIN_DROP_MPS
        for k, v in enumerate((.85, .84, .83, .82, .81, .74)):
            slow._lane_coast_observe(.6+k*.1, v)
        self.assertEqual(slow._lane_coast_decel, ms.LANE_STOP_DECEL_MIN_MPS2)

    def test_power_or_a_jump_restarts_the_coast(self):
        motion = ms.ManeuverSequence(0., continue_route=True)
        motion.lane_motor, motion._power_cmd_at = ms.NEUTRAL, None
        motion._lane_coast_observe(0., .9)
        motion._lane_coast_observe(.1, 1.2)  # jump up: not a coast
        self.assertEqual(len(motion._lane_coast_track), 1)
        motion.lane_motor = ms.ROUTE_SLOW_MOTOR
        motion._lane_coast_observe(.2, 1.1)
        self.assertEqual(motion._lane_coast_track, [])


if __name__ == '__main__':
    unittest.main()
