"""Round y5 (Claude): the crosswalk stop in the start lane and its fallback in
the first turn.

Bench 10-28 on the car (front 1.0/1.5/2.0/2.46 m before the stripes' near
edge): race4's crosswalk box bottom rows 476/456/444/437.5. Synthetic frames
and states; no physical claim."""
import unittest

import maneuver_sequence as ms
import test_route_58_light_lane as lane
from turn_motion import NEUTRAL, TRIAL_MOTOR


def start_motion():
    motion = lane.LightLaneTests.lane_motion(None)
    motion.route_stage, motion.light_vision = 'start_lane', True
    motion.first_approach = None
    motion.drive_since = 0.
    return motion


def scan(seq, at):
    return {'seq': seq, 'at_ms': round(at*1000), 'received_at': at}


def frame(motion, k, captured, score, row):
    motion._light_observe({'light': {'frame': k, 'captured_at': captured, 'red': 0., 'green': 0.,
                                     'crosswalk': score, 'crosswalk_row': row}})


def distance(row):
    return ms.CROSSWALK_ROW_SCALE/(row-ms.CROSSWALK_ROW_HORIZON_PX)-ms.CROSSWALK_ROW_OFFSET_M


class CrosswalkFix(unittest.TestCase):
    def test_the_rows_fit_the_bench(self):
        for row, metres in ((476, 1.0), (456, 1.5), (444, 2.0), (437.5, 2.46)):
            self.assertAlmostEqual(distance(row), metres, delta=.02)

    def test_two_frames_one_in_range_fix_the_edge_nearer_by_the_lag_and_two_errors(self):
        motion = start_motion()
        track = {k: 6.-.6*(k-10)*.1 for k in range(10, 31)}  # .6 m/s toward the board ahead
        motion._crosswalk_observe(scan(10, 1.0), track[10])
        frame(motion, 1, 1.05, .30, 444.)  # weak: seen, not ranged
        motion._crosswalk_observe(scan(11, 1.1), track[11])
        self.assertEqual((motion.crosswalk_state, motion._crosswalk_board_m), ('search', None))
        for k in range(12, 20):
            motion._crosswalk_observe(scan(k, k/10), track[k])
        frame(motion, 2, 1.85, .60, 456.)
        motion._crosswalk_observe(scan(20, 2.0), track[20])
        self.assertEqual(motion.crosswalk_state, 'approach')
        at = 1.85-ms.CROSSWALK_CAMERA_LAG_S
        place = 6.-.6*(at-1.)
        sigma = ms.CROSSWALK_ROW_SCALE*ms.CROSSWALK_ROW_SIGMA_PX/(456.-ms.CROSSWALK_ROW_HORIZON_PX)**2
        self.assertAlmostEqual(motion._crosswalk_board_m,
                               place-ms.FRONT_BODY_EXTENT_M-(distance(456.)-2*sigma), places=9)
        self.assertIn('crosswalk_fixed', motion.route_events)

    def test_one_frame_or_weak_frames_only_do_not_fix_it(self):
        motion = start_motion()
        for k in range(10, 20):
            motion._crosswalk_observe(scan(k, k/10), 6.-.05*k)
        frame(motion, 1, 1.6, .7, 460.)
        motion._crosswalk_observe(scan(20, 2.0), 5.)
        self.assertIsNone(motion._crosswalk_board_m)  # one frame
        motion2 = start_motion()
        for k in range(10, 20):
            motion2._crosswalk_observe(scan(k, k/10), 6.-.05*k)
            frame(motion2, k, k/10, .35, 440.)
        self.assertIsNone(motion2._crosswalk_board_m)  # never in range
        self.assertEqual(motion2.crosswalk_state, 'search')

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
