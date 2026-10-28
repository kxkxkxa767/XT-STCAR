"""Round y3 (Claude): the light stop facing the light and leaving on time.

Car run y2-2 (10-27): stopped .14 m left of the gantry's centre ~.3 m before
it, the camera had the red lamp at the frame's top edge and the green one
beyond its right edge; race4 saw the red in 36 frames and no green while it
was lit, and the car left on the 12 s wait cap 6 s after green. Now a red seen
from the stop that has gone from the newest fresh frames releases the car, and
green never seen it goes 8 s after the first red (user: the rules' red lasts
3-10 s, not moving 5 s after green fails the run); while stopping the light
lane pursuit follows the gantry's centre line. Synthetic states and scans, one
simulated run; no physical claim."""
import math
import unittest

import maneuver_sequence as ms
import test_route_58_light_lane as lane
import test_route_y2_light_stop as y2


class LightRelease(unittest.TestCase):
    lane_motion = lane.LightLaneTests.lane_motion
    scan = y2.LightWait.scan
    waiting = y2.LightWait.waiting

    def observe(self, motion, frame, captured, red, green=0.):
        motion._light_observe({'light': {'frame': frame, 'captured_at': captured, 'red': red, 'green': green}})

    def test_a_red_seen_from_the_stop_and_gone_from_three_fresh_frames_releases(self):
        motion = self.waiting([(1, .7, .9, 0.), (2, 1.2, .8, 0.), (3, 1.5, .8, 0.), (4, 1.8, .8, 0.)])
        self.assertEqual((motion._light_red_frames, motion._light_stop_red_frames, motion._light_red_at), (4, 3, .7))
        self.observe(motion, 5, 2.1, 0.)
        self.observe(motion, 6, 2.4, 0.)
        self.assertFalse(motion._light_wait(self.scan(25), 2.5, {}))  # two frames without red
        self.observe(motion, 7, 2.7, 0.)
        self.assertTrue(motion._light_wait(self.scan(28), 2.8, {}))
        self.assertEqual(motion.light_stop_state, 'released')
        go = motion.light_stop_evidence['go']
        self.assertEqual(go['basis'], 'red_gone_from_fresh_frames_after_the_stop')
        self.assertEqual((go['red_frames_since_stop'], go['red_frames_since_gantry']), (3, 4))
        self.assertEqual([f[0] for f in go['frames']], [5, 6, 7])
        self.assertAlmostEqual(go['red_since_s'], 2.1)

    def test_missed_red_frames_in_a_steady_red_keep_the_wait(self):
        motion = self.waiting([(1, 1.2, .8, 0.), (2, 1.5, .8, 0.), (3, 1.8, .8, 0.), (4, 2.1, 0., 0.),
                               (5, 2.4, 0., 0.), (6, 2.7, .8, 0.)])
        self.assertFalse(motion._light_wait(self.scan(28), 2.8, {}))
        self.observe(motion, 7, 3.0, 0.)
        self.observe(motion, 8, 3.3, 0.)
        self.assertFalse(motion._light_wait(self.scan(34), 3.4, {}))  # the red at 2.7 is still among the newest three

    def test_stale_frames_do_not_count_as_gone(self):
        motion = self.waiting([(1, 1.2, .8, 0.), (2, 1.5, .8, 0.), (3, 1.8, .8, 0.), (4, 2.1, 0., 0.),
                               (5, 2.4, 0., 0.), (6, 2.7, 0., 0.)])
        self.assertFalse(motion._light_wait(self.scan(32), 3.2, {}))  # frame 4 more than 1 s old: two fresh

    def test_a_weak_red_from_the_stop_waits_for_the_red_time(self):
        motion = self.waiting([(1, .7, .9, 0.), (2, 1.2, .3, 0.), (3, 1.5, .3, 0.), (4, 1.8, .3, 0.),
                               (5, 2.1, 0., 0.), (6, 2.4, 0., 0.), (7, 2.7, 0., 0.)])
        self.assertEqual(motion._light_stop_red_frames, 0)  # below LIGHT_GREEN_MIN_SCORE: the lamp may be cut off
        self.assertFalse(motion._light_wait(self.scan(28), 2.8, {}))
        self.assertFalse(motion._light_wait(self.scan(86), 8.6, {}))
        self.assertTrue(motion._light_wait(self.scan(88), 8.8, {}))
        self.assertEqual(motion.light_stop_evidence['go']['basis'], 'green_not_seen_red_go_time')

    def test_a_red_still_in_view_holds_past_8_s_until_11_s(self):
        motion = self.waiting([(1, .7, .9, 0.)])
        k = 2
        for t in (1.2, 1.5):
            self.observe(motion, k, t, .8)
            k += 1
        for check in (8.8, 10.0, 11.6):
            self.observe(motion, k, check-.1, .8)
            k += 1
            self.assertFalse(motion._light_wait(self.scan(round(check*10)), check, {}), check)
        self.observe(motion, k, 11.7, .8)
        self.assertTrue(motion._light_wait(self.scan(118), 11.8, {}))
        self.assertEqual(motion.light_stop_state, 'released')
        self.assertEqual(motion.light_stop_evidence['go']['basis'], 'red_still_seen_red_max_time')

    def test_green_still_releases_first(self):
        motion = self.waiting([(1, 1.2, .8, 0.), (2, 1.5, .8, 0.), (3, 1.8, .8, 0.), (4, 2.1, 0., .9)])
        self.assertFalse(motion._light_wait(self.scan(22), 2.2, {}))
        self.observe(motion, 5, 2.4, 0., .9)
        self.assertTrue(motion._light_wait(self.scan(25), 2.5, {}))
        self.assertEqual(motion.light_stop_evidence['go']['basis'], 'green_in_fresh_frames_after_the_stop')

    def test_no_frames_at_all_after_the_first_red(self):
        motion = self.waiting([(1, .7, .9, 0.)])  # the camera process died
        self.assertFalse(motion._light_wait(self.scan(86), 8.6, {}))
        self.assertTrue(motion._light_wait(self.scan(88), 8.8, {}))
        self.assertEqual(motion.light_stop_evidence['go']['basis'], 'green_not_seen_red_go_time')


class LightFacing(unittest.TestCase):
    lane_motion = lane.LightLaneTests.lane_motion
    LANE = {'outer_lateral_m': .43, 'inner_lateral_m': -.60, 'center_lateral_m': -.085, 'inner_start_along_m': None}

    def test_the_gantry_centre_less_the_lane_centre_bounded(self):
        motion = self.lane_motion()
        offset = motion._light_center_offset
        self.assertAlmostEqual(offset([[2., .31], [2.05, -.60]], self.LANE, 0.), -.06, places=9)  # car run y2-2
        self.assertEqual(offset([[2., .9], [2., -.1]], self.LANE, 0.), ms.LIGHT_CENTER_MAX_SHIFT_M)
        self.assertEqual(offset([[2., .31]], self.LANE, .04), .04)
        self.assertEqual(offset([[2., .31], [2., -.6]], None, .04), .04)

    def pursuit(self, motion, heading, lateral=None):
        motion._open_lane_target = lambda scan, h: dict(self.LANE)
        return motion._open_light_pursuit({'seq': 1, 'ranges': [3.]*360}, heading, None, lateral)

    def test_the_pursuit_follows_the_gantry_centre_only_while_stopping_and_aligned(self):
        motion = self.lane_motion()
        motion._light_center_shift_m = -.06
        self.assertAlmostEqual(self.pursuit(motion, 0.)[1][1], -.085)  # no light stop
        motion.light_stop_state = 'approach'
        self.assertAlmostEqual(self.pursuit(motion, 0.)[1][1], -.145)
        motion.light_stop_state = 'waiting'
        self.assertAlmostEqual(self.pursuit(motion, 0.)[1][1], -.145)
        turning = self.pursuit(motion, math.radians(25))
        self.assertAlmostEqual(turning[1][0]*math.sin(math.radians(25))+turning[1][1]*math.cos(math.radians(25)),
                               -.085)  # the entry turn: the lane's line
        self.assertAlmostEqual(self.pursuit(motion, 0., lateral=.1)[1][1], .1)  # an explicit line (entry shift)
        motion.light_stop_state = 'released'
        self.assertAlmostEqual(self.pursuit(motion, 0.)[1][1], -.085)

    def test_both_legs_measured_set_the_shift(self):
        motion = self.lane_motion()
        lane_ = {**y2.LightLegs.LANE, 'center_lateral_m': .05}
        motion._light_line = {'along_m': 2.0, 'at_s': 0.9, 'source': 'legs', 'legs': [[2.0, .45], [2.0, -.45]],
                              'speed_mps': None, 'board_m': None, 'leg_offsets_m': [0., 0.]}
        motion._light_line_ahead(y2.legs_scan(10, (1.95, .45), (1.95, -.45)), 0., lane_, 1.)
        legs = motion._light_line['legs']
        self.assertAlmostEqual(motion._light_center_shift_m, (legs[0][1]+legs[1][1])/2-.05, places=12)
        before = motion._light_center_shift_m
        motion._light_line_ahead(y2.legs_scan(11, (1.9, .45)), 0., {**lane_, 'center_lateral_m': -.3}, 1.1)
        self.assertEqual(motion._light_center_shift_m, before)  # one leg: kept


class LightLaunch(unittest.TestCase):
    """Car run y3-2: .64 s into the launch (1560 since 26.55 s, a .9 s rate window)
    the median rate read .39 m/s with 3.25 m to the board; the guard put the
    stop past the board and ended the launch at once."""
    lane_motion = lane.LightLaneTests.lane_motion
    scan = y2.LightWait.scan
    waiting = y2.LightWait.waiting

    def guard(self, launch_from):
        motion = self.lane_motion()
        motion._power_since, motion._power_cmd_at, motion._power_prev_end = 26.55, 27.17, None
        motion._lane_ranges = [(26.29+.1*k, 26.29+.1*k, 3.6) for k in range(10)]
        motion._light_launch_from = launch_from
        return motion._lane_worst_gap(3.25, .39, 27.19, ms.LANE_STOP_DECEL_MIN_MPS2, ms.LANE_KICK_ACCEL_MAX_MPS2,
                                      ms.LANE_REACTION_BASE_S)

    def test_from_rest_the_kick_bound_keeps_the_launch_going(self):
        self.assertLess(self.guard(None), ms.LANE_FINISH_MIN_GAP_M)  # as on the car: cut
        bounded = self.guard(26.55)
        self.assertGreater(bounded, ms.LANE_FINISH_MIN_GAP_M)
        self.assertLess(bounded, 1.3)  # still a 1.2 m/s peak at the weakest coast
        self.assertEqual(self.guard(20.), self.guard(None))  # long powered: the rate itself

    def test_the_launch_sets_the_bound_and_lasts_longer(self):
        self.assertEqual(ms.LIGHT_LAUNCH_S, .6)
        motion = self.waiting([(1, .8, .9, 0.), (2, 1.2, 0., .9), (3, 1.5, 0., .9)])
        self.assertTrue(motion._light_wait(self.scan(16), 1.6, {}))
        self.assertEqual(motion._light_launch_from, 1.6)
        self.assertAlmostEqual(motion._light_launch_until, 1.6+ms.ESC_LAG_S+ms.LIGHT_LAUNCH_S)


class SimulatedFacing(unittest.TestCase):
    def test_green_out_of_view_at_the_stop_the_car_leaves_when_the_red_goes(self):
        cs = y2.course_sim()
        course = cs.Course(**cs.LAYOUTS['user8'], gantry=True)
        vehicle = {'steer_tau': .25, 'play_pwm': 40., 'speed_scale': 1., 'decel': .6, 'gain_scale': 1.}
        # Round y5: the crosswalk where the user's track has it. Its stop (search at
        # .5 m/s, kicks; this car's 1550 fades at low speed) costs this .3 s-delay
        # car ~7 s more drive time in the start lane, and the 40 s budget may end
        # the run after the light (car run y4 drove ~26.5 s): the light is checked.
        summary, _, _ = cs.run(course, max_s=75., vehicle='measured', delay=.3, vehicle_params=vehicle,
                               light={'red_s': 10., 'green_min_ahead_m': .6, 'crosswalk_m': 2.46})
        light = summary['light']
        self.assertIn(summary['end_reason'], ('route_finish_zone_stop', 'maneuver_cumulative_drive_timeout'))
        self.assertIsNotNone(light['crossed_at'], light)
        self.assertTrue(light['stopped_in_zone'], light)
        self.assertFalse(light['went_before_green'], light)
        self.assertFalse(light['went_late'], light)
        self.assertEqual(light['penalty_s'], 0)
        self.assertLess(light['crossed_at']-light['green_at'], 2.5, light)


if __name__ == '__main__':
    unittest.main()
