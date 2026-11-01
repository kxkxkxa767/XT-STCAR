"""Round y15 (Claude; user 11-01: 根据雷达测距的数据动态调整滑行距离; 过红绿的你可以用前方+
后方辅助的方式来拟合; 在过红绿灯后，你的补的推力可以适当大点; 根据上一次补的力度反馈的速度来决定
下一次的补推力度; 注意实时性和cpu开销). The coast decel from positions (CoastFit, running
sums): the crosswalk stop on the board behind, the light lane on the board ahead
and behind, carried on to the light and the finish; after the light the finish
tops up at 1560 where that push's guard passes, each top-up's response measured
for the next. Synthetic positions and states; no physical claim."""
import math
import unittest

import maneuver_sequence as ms
from turn_motion import NEUTRAL, TRIAL_MOTOR
import test_route_58_light_lane as lane
import test_route_y5_crosswalk as crosswalk

FRONT = ms.FRONT_BODY_EXTENT_M


def coast(t, v0=1., decel=.6):
    return v0*t-decel*t*t/2


class Fit(unittest.TestCase):
    def test_a_parabola_by_running_sums(self):
        fit = ms.CoastFit()
        for k in range(4):
            fit.add(10.+k*.1, 2.+coast(k*.1))
        self.assertIsNone(fit.fit())  # under COAST_FIT_MIN_POINTS
        fit.add(10.4, 2.+coast(.4))
        decel, se, speed = fit.fit()
        self.assertAlmostEqual(decel, .6, places=8)
        self.assertLess(se, 1e-4)
        self.assertAlmostEqual(speed, 1.-.6*.4, places=8)
        short = ms.CoastFit()
        for k in range(6):
            short.add(k*.07, coast(k*.07))
        self.assertIsNone(short.fit())  # .35 s, under LANE_COAST_MIN_SPAN_S

    def test_a_spike_is_dropped_a_step_or_a_gap_restarts(self):
        fit = ms.CoastFit()
        for k in range(6):
            fit.add(k*.1, coast(k*.1))
        fit.add(.6, coast(.6)+.2)  # one reading off another surface
        fit.add(.7, coast(.7))
        self.assertEqual(fit.n, 7)
        self.assertAlmostEqual(fit.fit()[0], .6, places=8)
        fit.add(.8, coast(.8)+.3)  # the reference moved for good
        fit.add(.9, coast(.9)+.3)
        self.assertEqual((fit.n, fit.t0), (1, .9))
        fit.add(1.3, coast(1.3)+.3)  # over COAST_FIT_GAP_S later
        self.assertEqual((fit.n, fit.t0), (1, 1.3))

    def test_sources_weighted_by_variance_and_spread(self):
        self.assertIsNone(ms._coast_fit_combined(None, None))
        decel, se, speed = ms._coast_fit_combined(None, (.6, .03, .5))
        self.assertAlmostEqual(decel, .6)
        self.assertAlmostEqual(se, math.sqrt(.03**2+ms.COAST_FIT_MODEL_SE_MPS2**2))
        decel, se, speed = ms._coast_fit_combined((.6, .02, .5), (.7, .02, .6))
        self.assertAlmostEqual((decel, speed), (.65, .55))
        self.assertAlmostEqual(se, .05)  # half the spread, over the weighted error
        decel, _, _ = ms._coast_fit_combined((.6, .01, .5), (.9, .2, .5))
        self.assertLess(decel, .61)  # the precise source dominates


class LaneFrontAndRear(unittest.TestCase):
    def test_a_jumping_board_ahead_leaves_the_rear_fit(self):
        motion = ms.ManeuverSequence(0., continue_route=True)
        motion.lane_motor, motion.servo = NEUTRAL, NEUTRAL
        for k in range(8):
            x = coast(k*.1, 1.2)
            front = 3.-x+(.3 if k >= 3 else 0.)  # another surface from the fourth scan on
            motion._lane_front_range = lambda scan, front=front: front
            motion._lane_rear_range = lambda scan, x=x: 1.+x
            motion._lane_coast_observe({'at_ms': k*100}, k*.1, 1.)
        self.assertEqual((motion._coast_fit_front.n, motion._coast_fit_rear.n), (4, 8))
        self.assertAlmostEqual(motion._lane_coast_decel, .6, places=6)


class CrosswalkCoast(unittest.TestCase):
    def coasting(self):
        motion = crosswalk.start_motion()
        motion.crosswalk_state, motion._crosswalk_board_m, motion._crosswalk_rear = 'approach', 3., True
        motion._crosswalk_coasting, motion._power_cmd_at, motion.servo = True, 0., NEUTRAL
        motion._start_motor = NEUTRAL
        return motion

    def test_the_crosswalk_coast_measures_the_floor_for_the_light(self):
        motion = self.coasting()
        for k in range(6, 12):  # from .6 s after the cut, on the board behind
            t = k*.1
            motion._crosswalk_coast_observe(crosswalk.scan(k, t), t, -(.5+coast(t, .9, 1.2)), .5)
        self.assertAlmostEqual(motion._lane_coast_decel, 1.2, places=6)
        aim, guard, strong = motion._lane_decels()  # the light lane's stops take it
        self.assertAlmostEqual(aim, 1.2, places=6)
        self.assertGreater(guard, ms.LANE_STOP_DECEL_MIN_MPS2)
        motion._power_cmd_at = 1.2  # power again: the fits restart, the measured coast stays
        motion._crosswalk_coast_observe(crosswalk.scan(13, 1.3), 1.3, -2., .5)
        self.assertEqual(motion._coast_fit_rear.n, 0)
        self.assertAlmostEqual(motion._lane_coast_decel, 1.2, places=6)

    def test_a_grippy_floor_is_topped_up_while_rolling(self):
        # Coasting at .5 m/s .9 m before the target's edge: the aim (.6 m/s2 after
        # a .3 s lag) foresees ~.54 m but the guard's .6 forbids a push; the fit
        # (1.5 m/s2) foresees ~.83 m and its weak end admits one.
        gap = .9
        track = 3.+FRONT+gap
        plain = self.coasting()
        plain._crosswalk_control(crosswalk.scan(1, 2.), 2., {}, track, .5, .5)
        self.assertEqual(plain._start_motor, NEUTRAL)
        fitted = self.coasting()
        fitted._lane_coast_decel, fitted._lane_coast_decel_se = 1.5, .02
        fitted._coast_fit = (1.5, .02, .45)
        feedback = fitted._crosswalk_control(crosswalk.scan(1, 2.), 2., {}, track, .5, .5)
        self.assertEqual(fitted._start_motor, ms.ROUTE_SLOW_MOTOR)
        self.assertEqual(feedback['coast_decel_mps2'], 1.5)


class AfterTheLight(unittest.TestCase):
    lane_motion = lane.LightLaneTests.lane_motion

    def finish(self, restarted=True):
        motion = self.lane_motion()
        motion.light_stop_evidence = {'go': {}} if restarted else None
        motion.lane_motor, motion.servo = NEUTRAL, NEUTRAL
        motion._lane_coast_decel, motion._lane_coast_decel_se = .6, .03
        motion._power_cmd_at, motion._power_since, motion._power_cmd_motor = 0., 0., ms.ROUTE_SLOW_MOTOR
        return motion

    @staticmethod
    def step(motion, now, gap, speed):
        motion._lane_speed = lambda scan, ahead, **_: speed
        scan = {'seq': round(now*10), 'at_ms': round(now*1000), 'received_at': now, 'ranges': [3.]*360}
        before = motion.lane_motor
        basis, feedback = motion._lane_goal_control(scan, FRONT+gap, now, ms.FINISH_GOAL)
        if motion.lane_motor in ms.DRIVE_MOTORS:  # the output's bookkeeping (_result)
            if before not in ms.DRIVE_MOTORS:
                motion._power_since, motion._power_prev_end = now, motion._power_cmd_at
            motion._power_cmd_at, motion._power_cmd_motor = now, motion.lane_motor
        return feedback

    def test_a_short_coast_after_the_light_tops_up_at_1560_unchopped(self):
        motion = self.finish()
        self.step(motion, 1., 2.8, .8)
        self.assertEqual((motion.lane_motor, motion._lane_topup_motor), (TRIAL_MOTOR, TRIAL_MOTOR))
        for k in range(1, 5):  # past LANE_KICK_FRAMES: a push, not a kick
            feedback = self.step(motion, 1.+k*.1, 2.8-k*.08, .8)
            self.assertEqual(motion.lane_motor, TRIAL_MOTOR)
        self.assertEqual(feedback['tail_lag_s'], ms.LANE_FINISH_ESC_LAG_S)
        self.assertEqual(feedback['guard_accel_mps2'], ms.LANE_KICK_ACCEL_MAX_MPS2)
        plain = self.finish(restarted=False)  # no light restart: 1550 as before
        self.step(plain, 1., 2.8, .8)
        self.assertEqual((plain.lane_motor, plain._lane_topup_motor), (ms.ROUTE_SLOW_MOTOR, ms.ROUTE_SLOW_MOTOR))

    def test_the_launch_keeps_the_long_tail_until_its_power_has_left(self):
        # simulation, 1.3x car: the launch's 1550 tail guarded with the short lag
        # while its 1560 still acted met the 8 cm gate
        motion = self.finish()
        motion._power_cmd_at, motion._power_cmd_motor = .8, ms.ROUTE_SLOW_MOTOR  # the launch, cut .2 s ago
        motion.lane_motor = NEUTRAL
        feedback = self.step(motion, 1., 2.8, .8)
        self.assertEqual(feedback['tail_lag_s'], ms.ESC_LAG_S)
        feedback = self.step(motion, 1.5, 2.8, .8)  # nothing of it left
        self.assertEqual(feedback['tail_lag_s'], ms.LANE_FINISH_ESC_LAG_S)

    def test_the_launch_tail_is_guarded_at_the_1560_bound(self):
        # simulation, 1.3x car: the launch's 1550 tail guarded at 1550's bound
        # (the measured decel above the prior) cut it late into the 8 cm gate
        motion = self.finish()
        motion._light_launch_from = .5  # the launch's powered run, now at 1550
        motion.lane_motor, motion._power_cmd_at, motion._power_cmd_motor = ms.ROUTE_SLOW_MOTOR, .9, ms.ROUTE_SLOW_MOTOR
        feedback = self.step(motion, 1., 3.2, 1.)
        self.assertEqual(feedback['guard_accel_mps2'], ms.LANE_KICK_ACCEL_MAX_MPS2)

    def test_where_1560_would_reach_the_board_1550(self):
        motion = self.finish()
        self.step(motion, 1., 1.3, .8)
        self.assertEqual((motion.lane_motor, motion._lane_topup_motor), (ms.ROUTE_SLOW_MOTOR, ms.ROUTE_SLOW_MOTOR))

    def test_the_last_push_response_sets_the_next_push(self):
        motion = self.finish()
        motion._lane_push_probe = [TRIAL_MOTOR, 1., .6, None, None]
        motion.lane_motor = TRIAL_MOTOR
        for t, v in ((1.1, .58), (1.2, .57)):
            motion._lane_push_observe(t, v)
        motion.lane_motor, motion._power_cmd_at = NEUTRAL, 1.2  # cut after the 1.2 output
        for t, v in ((1.3, .62), (1.4, .75), (1.5, .82), (1.6, .85), (1.7, .84), (1.9, .8), (2.0, .78)):
            motion._lane_push_observe(t, v)
        self.assertIsNotNone(motion._lane_push_probe)
        motion._lane_push_observe(2.1, .76)  # LANE_PUSH_PROBE_S after the last output
        self.assertIsNone(motion._lane_push_probe)
        self.assertAlmostEqual(motion._lane_push_rate_strong, (.85-.57)/(.2+ms.LANE_PUSH_MIN_S))
        motion._lane_topup_motor, motion._power_cmd_motor = TRIAL_MOTOR, TRIAL_MOTOR
        feedback = self.step(motion, 2.2, 3.5, .8)  # the strong push's guard now from its response
        self.assertEqual(feedback['guard_accel_mps2'],
                         max(ms.LANE_STRONG_PUSH_ACCEL_MIN_MPS2, ms.LANE_PUSH_RATE_MARGIN*motion._lane_push_rate_strong))
        quick = self.finish()
        quick._lane_push_probe = [ms.ROUTE_SLOW_MOTOR, 1., .6, None, None]
        quick.lane_motor = ms.ROUTE_SLOW_MOTOR
        quick._lane_push_observe(1.1, .6)
        quick.lane_motor, quick._power_cmd_at = NEUTRAL, 1.1
        quick._lane_push_observe(1.2, .62)
        quick.lane_motor = TRIAL_MOTOR  # the next push before LANE_PUSH_PROBE_MIN_S: dropped
        quick._lane_push_observe(1.3, .62)
        self.assertEqual((quick._lane_push_probe, quick._lane_push_rate_slow), (None, None))


if __name__ == '__main__':
    unittest.main()
