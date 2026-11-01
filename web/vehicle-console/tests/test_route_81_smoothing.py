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
        self.assertFalse(hasattr(ms, 'OPEN_TURN_IN_LEAD'))  # removed in round 86 (first-cone gaps)


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
        motion.lane_motor = ms.NEUTRAL
        # Round 101: fast enough that the aimed coast reaches the end board: the lane coasts.
        motion._lane_speed = lambda scan, ahead: 3.4
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
    """Round y15: a least-squares parabola through the positions on the board
    ahead and the board behind (rounds 82-101: a line through the median-rate
    speed; tests/test_route_y15_coast_fit.py has the rest)."""
    @staticmethod
    def coasting(power_cmd_at=None):
        motion = ms.ManeuverSequence(0., continue_route=True)
        motion.lane_motor, motion._power_cmd_at, motion.servo = ms.NEUTRAL, power_cmd_at, ms.NEUTRAL
        return motion

    @staticmethod
    def observe(motion, t, travel, speed=1.):
        motion._lane_front_range = lambda scan: 3.-travel
        motion._lane_rear_range = lambda scan: 1.+travel
        motion._lane_coast_observe({'at_ms': round(t*1000)}, t, speed)

    def test_measured_coast_decel_is_a_least_squares_fit_within_bounds(self):
        motion = self.coasting()
        for k in range(6):
            t = k*.1
            self.observe(motion, t, 1.2*t-.55*t*t/2)
        self.assertAlmostEqual(motion._lane_coast_decel, .55, places=6)
        self.assertLess(motion._lane_coast_decel_se, ms.COAST_FIT_MODEL_SE_MPS2)
        aim, guard, strong = motion._lane_decels()
        self.assertEqual(aim, motion._lane_coast_decel)
        self.assertAlmostEqual(guard, ms.LANE_COAST_GUARD_RATIO*aim)
        self.assertAlmostEqual(strong, aim/ms.LANE_COAST_GUARD_RATIO)
        weak, strong_car = self.coasting(), self.coasting()
        for k in range(6):
            t = k*.1
            self.observe(weak, t, .9*t-.1*t*t/2)
            self.observe(strong_car, t, 1.5*t-2.*t*t/2)
        self.assertEqual(weak._lane_coast_decel, ms.LANE_COAST_DECEL_FLOOR_MPS2)
        self.assertEqual(strong_car._lane_coast_decel, ms.LANE_COAST_DECEL_CAP_MPS2)
        self.assertEqual(self.coasting()._lane_decels(), (ms.LANE_STOP_DECEL_MPS2, ms.LANE_STOP_DECEL_MIN_MPS2, None))

    def test_a_noisy_fit_waits_and_its_error_widens_the_bounds(self):
        motion = self.coasting()
        noise = (.004, -.006, .005, -.004, .006, -.005, .004, -.006, .005, -.004, .006, -.005)
        for k, e in enumerate(noise[:5]):
            t = k*.1
            self.observe(motion, t, 1.2*t-.5*t*t/2+e)
        self.assertIsNone(motion._lane_coast_decel)  # standard error above the limit
        for k, e in enumerate(noise[5:], start=5):
            t = k*.1
            self.observe(motion, t, 1.2*t-.5*t*t/2+e)
        self.assertIsNotNone(motion._lane_coast_decel)
        se = motion._lane_coast_decel_se
        self.assertGreater(se, ms.COAST_FIT_MODEL_SE_MPS2)
        self.assertLessEqual(se, ms.LANE_COAST_DECEL_SE_MAX_MPS2)
        aim, guard, strong = motion._lane_decels()
        self.assertAlmostEqual(guard, max(ms.LANE_COAST_DECEL_FLOOR_MPS2, min(ms.LANE_COAST_GUARD_RATIO*aim, aim-2*se)))
        self.assertAlmostEqual(strong, max(aim/ms.LANE_COAST_GUARD_RATIO, aim+2*se))

    def test_power_steering_or_rest_restarts_the_coast(self):
        motion = self.coasting()
        for k in range(3):
            self.observe(motion, k*.1, k*.1)
        self.assertEqual((motion._coast_fit_front.n, motion._coast_fit_rear.n), (3, 3))
        motion.lane_motor = ms.ROUTE_SLOW_MOTOR
        self.observe(motion, .3, .3)
        self.assertEqual((motion._coast_fit_front.n, motion._coast_fit_rear.n), (0, 0))
        motion.lane_motor, motion.servo = ms.NEUTRAL, ms.NEUTRAL+ms.COAST_FIT_STRAIGHT_PWM+1  # turning
        self.observe(motion, .4, .4)
        self.assertEqual(motion._coast_fit_front.n, 0)
        motion.servo = ms.NEUTRAL
        self.observe(motion, .5, .5, speed=ms.LANE_SPEED_RESTART_MPS-.01)  # at or near rest
        self.assertEqual(motion._coast_fit_front.n, 0)
        self.observe(motion, .6, .6, speed=None)  # an unknown speed does not restart it
        self.assertEqual(motion._coast_fit_front.n, 1)

    def test_nothing_of_the_push_may_be_left_in_the_pipeline(self):
        motion = self.coasting(power_cmd_at=1.)
        self.observe(motion, 1.+ms.ESC_LAG_S+ms.LANE_ESC_SKEW_S-.01, 0.)
        self.assertEqual(motion._coast_fit_rear.n, 0)
        self.observe(motion, 1.+ms.ESC_LAG_S+ms.LANE_ESC_SKEW_S, 0.)
        self.assertEqual(motion._coast_fit_rear.n, 1)
        motion._lane_rear_range = lambda scan: ms.COAST_FIT_REAR_MAX_M+.1  # too far behind
        motion._lane_coast_observe({'at_ms': 1600}, 1.6, 1.)
        self.assertEqual((motion._coast_fit_front.n, motion._coast_fit_rear.n), (2, 1))


class LightLaneAim(unittest.TestCase):
    """Round 101: the predicted stop, the ESC pipeline and the present speed."""
    def test_stop_gap_travels_reaction_coast_power_then_coasts(self):
        stop = ms.ManeuverSequence._lane_stop
        self.assertAlmostEqual(stop(2., 1., .5, 0., 0., 0., 0.), 2.-1./(2*.5))
        self.assertAlmostEqual(stop(2., 1., .5, .2, 0., 0., 0.), 2.-.2-1.)
        # .2 s coasting (to .9 m/s), .3 s of power at .5 (to 1.05), then a coast.
        self.assertAlmostEqual(stop(3., 1., .5, 0., .2, .3, .5), 3.-.95*.2-1.05*.3-1.05**2/(2*.5))
        # Power never takes the car above LANE_PUSH_MAX_MPS (nor slows a faster one).
        self.assertAlmostEqual(stop(5., 2., .5, 0., 0., 1., 1.), 5.-2.*1.-2.**2/(2*.5))
        # A car that stops while coasting never gets the power.
        self.assertAlmostEqual(stop(1., .1, .5, 0., .5, 1., 1.), 1.-.1**2/(2*.5))

    def test_pipeline_power_between_onset_and_offset(self):
        motion = ms.ManeuverSequence(0., continue_route=True)
        self.assertEqual(motion._lane_pipeline(1., .3, .3), (0., 0.))
        motion._power_since, motion._power_cmd_at = .9, 1.  # a .1 s push just sent
        coast, power = motion._lane_pipeline(1., .3, .3)
        self.assertAlmostEqual(coast, .2)
        self.assertAlmostEqual(power, .1)
        coast, power = motion._lane_pipeline(1.25, .3, .3)  # on the wheels from 1.2 to 1.3
        self.assertEqual(coast, 0.)
        self.assertAlmostEqual(power, .05)
        self.assertEqual(motion._lane_pipeline(1.3, .3, .3), (0., 0.))
        # Pushing on extends the run; after it has left the pipeline, a new one.
        coast, power = motion._lane_pipeline(1.1, .25, .4, .1)
        self.assertAlmostEqual(coast, .05)
        self.assertAlmostEqual(power, 1.2+.4-1.15)
        coast, power = motion._lane_pipeline(2., .25, .4, .1)
        self.assertAlmostEqual(coast, .25)
        self.assertAlmostEqual(power, .1+.4-.25)

    def test_power_runs_are_tracked_by_the_outputs(self):
        motion = ms.ManeuverSequence(0., continue_route=True)
        motion.route_stage, motion.entry_kind, motion.phase = 'light_lane', 'lane', 'drive'
        motion._route_motor = lambda: motion.lane_motor
        outputs = []
        for now, motor in ((1., ms.TRIAL_MOTOR), (1.1, ms.TRIAL_MOTOR), (1.2, ms.NEUTRAL), (1.5, ms.TRIAL_MOTOR)):
            motion.lane_motor = motor
            outputs.append(motion._result(now)['motor'])
            if now == 1.1:
                self.assertEqual((motion._power_since, motion._power_cmd_at, motion._power_prev_end), (1., 1.1, None))
        self.assertEqual((motion._power_since, motion._power_cmd_at, motion._power_prev_end), (1.5, 1.5, 1.1))
        self.assertEqual(outputs, [ms.TRIAL_MOTOR, ms.TRIAL_MOTOR, ms.NEUTRAL, ms.TRIAL_MOTOR])

    def test_present_speed_from_the_median_rate(self):
        motion = ms.ManeuverSequence(0., continue_route=True)
        motion._lane_ranges = [(t/10, t/10, 3.) for t in range(10, 17)]  # a .6 s window: .3 s old
        self.assertAlmostEqual(motion._lane_speed_now(1., 2., .3, .3, .5, .35), 1.-.5*.3)
        motion._power_since, motion._power_cmd_at = 0., 1.6  # powered throughout
        self.assertAlmostEqual(motion._lane_speed_now(1., 1.7, .3, .3, .5, .35), 1.+.35*.3)
        # Cut .1 s before the window's middle: .2 s of it still powered (lag .3).
        motion._power_cmd_at = 1.2
        self.assertAlmostEqual(motion._lane_speed_now(1., 1.7, .3, .3, .5, .35), 1.+.35*.1-.5*.2)

    def test_guard_takes_the_worse_lag(self):
        motion = ms.ManeuverSequence(0., continue_route=True)
        motion._lane_ranges = [(1., 1., 3.)]*3
        motion._power_since, motion._power_cmd_at = 0., 1.
        worst = motion._lane_worst_gap(3., 1., 1.02, .5, .5, ms.LANE_REACTION_BASE_S)
        values = []
        for lag in (ms.LANE_ESC_LAG_MIN_S, ms.ESC_LAG_S):
            onset, offset = lag-ms.LANE_ESC_SKEW_S, lag+ms.LANE_ESC_SKEW_S
            coast, power = motion._lane_pipeline(1.02, onset, offset)
            values.append(ms.ManeuverSequence._lane_stop(3., 1.+ms.LANE_SPEED_GUARD_MARGIN_MPS, .5,
                                                         ms.LANE_REACTION_BASE_S, coast, power, .5))
        self.assertEqual(worst, min(values))
        self.assertEqual(worst, values[1])  # the longer lag keeps more power in the pipeline


class LoopPlanReuse(unittest.TestCase):
    """Round 83 (car run 82: a .111 s resume tick): unchanged plan inputs
    (same support, free memory, car within 1 cm) reuse the plan."""
    def test_second_call_with_the_same_inputs_skips_the_planner(self):
        import copy, json, math
        from unittest.mock import patch
        frame = json.loads((Path(__file__).resolve().parent/'fixtures/route-69-loop-plan-jump.json').read_text())
        slot = frame['previous_slot']
        slot['key'] = tuple(slot['key'])
        slot['free'] = {float(k): v for k, v in slot['free'].items()}
        motion = ms.ManeuverSequence(0., continue_route=True)
        motion._open_slots[frame['side']] = copy.deepcopy(slot)
        motion._open_slots[frame['side']]['approaching'] = True  # replanned every frame
        motion.compass.current = lambda scan: frame['compass_heading']
        motion.compass.rate = 0.
        real = ms._plan_loop
        with patch.object(ms, '_plan_loop', side_effect=real) as planner:
            first = motion._open_loop_geometry(frame['scan'], frame['target'], frame['identity'], frame['side'], frame['axis'])
            second = motion._open_loop_geometry(frame['scan'], frame['target'], frame['identity'], frame['side'], frame['axis'])
        self.assertEqual(planner.call_count, 1)
        self.assertEqual(first['plan'], second['plan'])
        self.assertEqual(first['target_pwm'], second['target_pwm'])


if __name__ == '__main__':
    unittest.main()
