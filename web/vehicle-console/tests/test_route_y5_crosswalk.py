"""Round y5 (Claude): the crosswalk stop in the start lane and its fallback in
the first turn. Round y10: the row-to-distance fit of the 10-29 car runs
(rows 424/468/441/435 at 2.34/1.06/1.60/1.92 m by the board behind, anchored
by the user's tape at the start), fixed from frames at the standstill.
Synthetic frames and states; no physical claim."""
import unittest

import maneuver_sequence as ms
import test_route_58_light_lane as lane
from turn_motion import NEUTRAL, TRIAL_MOTOR


def start_motion():
    motion = lane.LightLaneTests.lane_motion(None)
    motion.route_stage, motion.light_vision = 'start_lane', True
    motion.first_approach = None
    motion.drive_since = 0.
    motion._power_cmd_at = None  # not moved yet
    return motion


def scan(seq, at):
    return {'seq': seq, 'at_ms': round(at*1000), 'received_at': at}


def frame(motion, k, captured, score, row):
    motion._light_observe({'light': {'frame': k, 'captured_at': captured, 'red': 0., 'green': 0.,
                                     'crosswalk': score, 'crosswalk_row': row}})


def distance(row):
    return ms.CROSSWALK_ROW_SCALE/(row-ms.CROSSWALK_ROW_HORIZON_PX)-ms.CROSSWALK_ROW_OFFSET_M


def board(place, row):
    return place-ms.FRONT_BODY_EXTENT_M-(distance(row)-2*ms.CROSSWALK_DISTANCE_SIGMA_M)


class CrosswalkFix(unittest.TestCase):
    def test_the_rows_fit_the_car_runs(self):
        for row, metres in ((424., 2.34), (468.1, 1.06), (441., 1.60), (434.7, 1.92)):
            self.assertAlmostEqual(distance(row), metres, delta=.15)  # the fit's largest errors .29-.32

    def test_frames_at_the_standstill_fix_it_before_the_car_moves(self):
        motion = start_motion()
        for k, t in enumerate((.1, .4, .7)):
            frame(motion, k+1, t, .27, 424.)
        motion._crosswalk_observe(scan(10, 1.0), -.32)  # the board behind .32 m
        self.assertEqual(motion.crosswalk_state, 'approach')
        self.assertAlmostEqual(motion._crosswalk_board_m, board(-.32, 424.), places=9)
        self.assertEqual(motion._crosswalk_rest, (1.0, -.32))

    def test_three_frames_within_the_window(self):
        motion = start_motion()
        frame(motion, 1, .1, .27, 424.)
        frame(motion, 2, .4, .27, 424.)
        motion._crosswalk_observe(scan(10, 1.0), -.32)
        self.assertIsNone(motion._crosswalk_board_m)
        motion2 = start_motion()
        for k, t in enumerate((.1, .4, 2.0)):
            frame(motion2, k+1, t, .27, 424.)
        motion2._crosswalk_observe(scan(10, 2.2), -.32)
        self.assertIsNone(motion2._crosswalk_board_m)  # 1.9 s apart
        frame(motion, 3, .7, .27, 424.)
        frame(motion, 4, 1.0, .27, 405.)  # out of the rows: seen only
        motion._crosswalk_observe(scan(11, 1.1), -.32)
        self.assertIsNotNone(motion._crosswalk_board_m)

    def test_later_frames_only_bring_it_nearer(self):
        motion = start_motion()
        for k, t in enumerate((.1, .4, .7)):
            frame(motion, k+1, t, .27, 424.)
        motion._crosswalk_observe(scan(10, 1.0), -.32)
        fixed = motion._crosswalk_board_m
        motion._power_cmd_at = 1.0
        track = {k: -.32-.5*(k-10)*.1 for k in range(10, 30)}  # .5 m/s away from the board behind
        for k in range(11, 20):
            motion._crosswalk_observe(scan(k, k/10), track[k])
        for n, t in enumerate((1.3, 1.6, 1.9)):
            frame(motion, 10+n, t, .4, 428.)  # farther than placed
        motion._crosswalk_observe(scan(20, 2.0), track[20])
        self.assertEqual(motion._crosswalk_board_m, fixed)
        for n, t in enumerate((2.2, 2.5, 2.8)):
            frame(motion, 20+n, t, .8, 470.)  # nearer
        for k in range(21, 30):
            motion._crosswalk_observe(scan(k, k/10), track[k])
        self.assertGreater(motion._crosswalk_board_m, fixed)

    def test_frames_outside_the_start_lane_are_ignored(self):
        motion = start_motion()
        motion.route_stage = 'first_target'
        frame(motion, 1, 1., .8, 460.)
        self.assertEqual(motion._crosswalk_raw, [])


class CrosswalkStop(unittest.TestCase):
    def approach(self, board=3.):
        motion = start_motion()
        motion.crosswalk_state, motion._crosswalk_board_m = 'approach', board
        motion._start_motor = TRIAL_MOTOR
        motion._power_cmd_at = .95  # powered until now
        return motion

    def test_the_aim_cuts_then_a_stop_in_the_zone_waits_and_goes(self):
        motion = self.approach()
        # 1.5 m to the edge at .5 m/s: drives on
        fb = motion._crosswalk_control(scan(1, 1.), 1., {}, 3.+ms.FRONT_BODY_EXTENT_M+1.5, .5, .5)
        self.assertEqual((motion._start_motor, fb['state']), (TRIAL_MOTOR, 'approach'))
        # .55 m at .5 m/s: the coast from here reaches the target
        motion._crosswalk_control(scan(2, 2.), 2., {}, 3.+ms.FRONT_BODY_EXTENT_M+.55, .5, .5)
        self.assertEqual(motion._start_motor, NEUTRAL)
        motion._power_cmd_at = 2.
        motion._start_motor = NEUTRAL
        fb = motion._crosswalk_control(scan(3, 3.), 3., {}, 3.+ms.FRONT_BODY_EXTENT_M+.3, 0., 0.)
        self.assertEqual(fb['state'], 'waiting')
        self.assertEqual(motion.crosswalk_evidence['stop']['basis'], 'stopped_before_the_crosswalk')
        motion._start_motor = TRIAL_MOTOR
        motion._crosswalk_control(scan(4, 6.4), 6.4, {}, 3.5, 0., 0.)
        self.assertEqual((motion.crosswalk_state, motion._start_motor), ('waiting', NEUTRAL))
        motion._crosswalk_control(scan(5, 6.5), 6.5, {}, 3.5, 0., 0.)
        self.assertEqual(motion.crosswalk_state, 'done')
        self.assertAlmostEqual(motion.crosswalk_wait_credit_s, 3.5)
        self.assertAlmostEqual(motion.drive_since, 3.5)
        self.assertIn('crosswalk_go', motion.route_events)
        self.assertIsNone(motion._crosswalk_control(scan(6, 6.6), 6.6, {}, 3.5, 0., 0.))

    def test_after_the_cut_it_coasts_to_a_standstill(self):
        motion = self.approach()
        motion._crosswalk_control(scan(1, 1.), 1., {}, 3.+ms.FRONT_BODY_EXTENT_M+.55, .5, .5)
        self.assertEqual((motion._start_motor, motion._crosswalk_coasting), (NEUTRAL, True))
        motion._start_motor = ms.ROUTE_SLOW_MOTOR  # the cruise would push again
        motion._crosswalk_control(scan(2, 1.1), 1.1, {}, 3.+ms.FRONT_BODY_EXTENT_M+1.5, .3, .3)
        self.assertEqual(motion._start_motor, NEUTRAL)  # aimed short now, still coasting
        motion._power_cmd_at = 0.
        motion._start_motor = TRIAL_MOTOR
        motion._crosswalk_control(scan(3, 3.), 3., {}, 3.+ms.FRONT_BODY_EXTENT_M+1.5, 0., 0.)
        self.assertEqual((motion._start_motor, motion._crosswalk_coasting), (TRIAL_MOTOR, False))  # stopped short: a kick

    def test_the_guard_cuts_a_fast_car(self):
        motion = self.approach()
        motion._crosswalk_control(scan(1, 1.), 1., {}, 3.+ms.FRONT_BODY_EXTENT_M+1.5, 1.4, 1.4)
        self.assertEqual(motion._start_motor, NEUTRAL)

    def test_stopped_short_of_the_zone_kicks_rested_apart(self):
        motion = self.approach()
        at = 3.+ms.FRONT_BODY_EXTENT_M+1.3
        fb = motion._crosswalk_control(scan(1, 1.5), 1.5, {}, at, 0., 0.)
        self.assertEqual((fb['state'], motion._start_motor), ('approach', NEUTRAL))  # not rested
        motors = []
        for k, now in enumerate((2., 2.1, 2.2, 2.3, 2.4)):
            motion._start_motor = TRIAL_MOTOR  # the start lane's own launch
            motion._crosswalk_control(scan(2+k, now), now, {}, at, 0., 0.)
            motors.append(motion._start_motor)
            if motion._start_motor == TRIAL_MOTOR:
                motion._power_cmd_at = now
        self.assertEqual(motors, [TRIAL_MOTOR]*ms.LANE_KICK_FRAMES+[NEUTRAL]*2)
        motion._start_motor = TRIAL_MOTOR
        motion._crosswalk_control(scan(9, 3.3), 3.3, {}, at, 0., 0.)
        self.assertEqual(motion._start_motor, TRIAL_MOTOR)  # rested: the next kick

    def test_no_kick_without_its_room(self):
        motion = self.approach()
        motion._start_motor = TRIAL_MOTOR
        motion._power_cmd_at = None
        motion.crosswalk_state = 'approach'
        # .45 m of kick room missing: the zone (1 m) already holds it, so a stop
        # there waits; between the zone and the room nothing is left
        fb = motion._crosswalk_control(scan(1, 5.), 5., {}, 3.+ms.FRONT_BODY_EXTENT_M+.9, 0., 0.)
        self.assertEqual(fb['state'], 'waiting')

    def test_lost_odometry_stops_where_it_is(self):
        motion = self.approach()
        motion._crosswalk_control(scan(1, 1.), 1., {}, None, .4, .4)
        self.assertEqual((motion.crosswalk_state, motion._start_motor), ('approach', NEUTRAL))
        motion._crosswalk_control(scan(2, 2.), 2., {}, None, 0., 0.)
        self.assertEqual(motion.crosswalk_evidence['stop']['basis'], 'stopped_with_the_crosswalk_odometry_lost')

    def test_the_first_turn_before_the_stop_falls_back_to_a_stop_there(self):
        for state in ('search', 'approach'):
            motion = self.approach()
            motion.crosswalk_state = state
            motion.first_approach = {'track_id': 1}
            fb = motion._crosswalk_control(scan(1, 1.), 1., {}, 5., 1.2, 1.2)
            self.assertEqual((fb['state'], motion._start_motor), ('fallback', NEUTRAL))
            self.assertIn('crosswalk_fallback', motion.route_events)
            motion._power_cmd_at = .9
            motion._crosswalk_control(scan(2, 2.), 2., {}, 4.5, .6, .6)
            self.assertEqual(motion.crosswalk_state, 'fallback')
            motion._crosswalk_control(scan(3, 3.), 3., {}, 4.2, 0., 0.)
            self.assertEqual(motion.crosswalk_state, 'waiting')
            self.assertEqual(motion.crosswalk_evidence['stop']['basis'], 'stopped_in_the_first_turn_fallback')

    def test_neutral_long_enough_counts_as_stopped(self):
        motion = self.approach()
        motion.crosswalk_state = 'fallback'
        motion._power_cmd_at = 1.
        motion._crosswalk_control(scan(1, 3.9), 3.9, {}, None, None, None)
        self.assertEqual(motion.crosswalk_state, 'fallback')
        motion._crosswalk_control(scan(2, 4.), 4., {}, None, None, None)
        self.assertEqual(motion.crosswalk_state, 'waiting')

    def test_no_light_vision_no_crosswalk(self):
        motion = start_motion()
        motion.light_vision = False
        motion._crosswalk_observe(scan(1, 1.), 5.)
        self.assertIsNone(motion.crosswalk_state)
        self.assertIsNone(motion._crosswalk_control(scan(1, 1.), 1., {}, 5., 1., 1.))


if __name__ == '__main__':
    unittest.main()
