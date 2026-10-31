"""Round y5 (Claude): the crosswalk stop in the start lane and its fallback in
the first turn. Round y10: fixed from frames at the standstill. Round y11: the
distance comes from the race program's YOLO process by the calibrated camera
model (scripts/vision-log.py crosswalk_front_m, checked on the track 10-31:
box bottoms 330/372 at 2.31-2.34/1.00-1.03 m); a sharper fix replaces the
edge. Synthetic frames and states; no physical claim."""
import importlib.util
from pathlib import Path
import unittest

import maneuver_sequence as ms
import test_route_58_light_lane as lane
from turn_motion import NEUTRAL, TRIAL_MOTOR

ROOT = Path(__file__).resolve().parents[3]


def start_motion():
    motion = lane.LightLaneTests.lane_motion(None)
    motion.route_stage, motion.light_vision = 'start_lane', True
    motion.first_approach = None
    motion.drive_since = 0.
    motion._power_cmd_at = None  # not moved yet
    return motion


def scan(seq, at):
    return {'seq': seq, 'at_ms': round(at*1000), 'received_at': at}


def frame(motion, k, captured, score, metres, row=None):
    motion._light_observe({'light': {'frame': k, 'captured_at': captured, 'red': 0., 'green': 0.,
                                     'crosswalk': score, 'crosswalk_row': row, 'crosswalk_m': metres}})


def sigma(metres):
    return ms.CROSSWALK_SIGMA_M+ms.CROSSWALK_SIGMA_PER_M2*metres*metres


def board(place, metres):
    return place-ms.FRONT_BODY_EXTENT_M-(metres-2*sigma(metres))


class CameraModel(unittest.TestCase):
    def test_the_track_checks_of_10_31(self):
        spec = importlib.util.spec_from_file_location('vision_log', ROOT/'scripts'/'vision-log.py')
        vision = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(vision)
        # race4's boxes at the start (tape 2.34, board behind 2.31) and at 1.00 m (1.028)
        self.assertAlmostEqual(vision.crosswalk_front_m([281., 314.5, 462., 330.]), 2.32, delta=.03)
        self.assertAlmostEqual(vision.crosswalk_front_m([187., 341., 550., 372.]), 1.015, delta=.02)
        # front .21 m: the stripes run out of the frame, the box ends at its bottom
        self.assertIsNone(vision.crosswalk_front_m([30., 389., 640., 476.5]))
        # the bottom corners lie outside the lens model's iteration
        self.assertIsNone(vision.crosswalk_front_m([0., 400., 60., 465.]))
        # an edge still in view near the bottom
        self.assertAlmostEqual(vision.crosswalk_front_m([200., 380., 500., 470.]), .39, delta=.01)


class CrosswalkFix(unittest.TestCase):
    def test_frames_at_the_standstill_fix_it_before_the_car_moves(self):
        motion = start_motion()
        for k, t in enumerate((.1, .4, .7)):
            frame(motion, k+1, t, .4, 2.32, 330.)
        motion._crosswalk_observe(scan(10, 1.0), -.32)  # the board behind .32 m
        self.assertEqual(motion.crosswalk_state, 'approach')
        self.assertAlmostEqual(motion._crosswalk_board_m, board(-.32, 2.32), places=9)
        self.assertAlmostEqual(motion._crosswalk_board_sigma, sigma(2.32), places=12)
        self.assertEqual(motion._crosswalk_rest, (1.0, -.32))

    def test_three_frames_within_the_window(self):
        motion = start_motion()
        frame(motion, 1, .1, .4, 2.32)
        frame(motion, 2, .4, .4, 2.32)
        motion._crosswalk_observe(scan(10, 1.0), -.32)
        self.assertIsNone(motion._crosswalk_board_m)
        motion2 = start_motion()
        for k, t in enumerate((.1, .4, 2.0)):
            frame(motion2, k+1, t, .4, 2.32)
        motion2._crosswalk_observe(scan(10, 2.2), -.32)
        self.assertIsNone(motion2._crosswalk_board_m)  # 1.9 s apart
        frame(motion, 3, .7, .4, 2.32)
        frame(motion, 4, 1.0, .5, None)  # the edge out of view: seen only
        frame(motion, 5, 1.05, .3, 3.8)  # beyond the range: seen only
        motion._crosswalk_observe(scan(11, 1.1), -.32)
        self.assertIsNotNone(motion._crosswalk_board_m)
        self.assertEqual(len(motion._crosswalk_fixes), 3)

    def test_a_sharper_fix_replaces_the_edge_a_blunter_one_only_brings_it_nearer(self):
        motion = start_motion()
        for k, t in enumerate((.1, .4, .7)):
            frame(motion, k+1, t, .4, 2.32)
        motion._crosswalk_observe(scan(10, 1.0), -.32)
        fixed = motion._crosswalk_board_m
        motion._power_cmd_at = 1.0
        track = {k: -.32-.5*(k-10)*.1 for k in range(10, 40)}  # .5 m/s away from the board behind
        for k in range(11, 20):
            motion._crosswalk_observe(scan(k, k/10), track[k])
        # the car closing in: the same edge (2.32 m from the start), seen sharper
        for n, t in enumerate((1.3, 1.6, 1.9)):
            frame(motion, 10+n, t, .6, 2.32-.5*(t-ms.CROSSWALK_CAMERA_LAG_S-1.))
        motion._crosswalk_observe(scan(20, 2.0), track[20])
        self.assertLess(motion._crosswalk_board_m, fixed)  # farther: the start's margin was wider
        self.assertAlmostEqual(motion._crosswalk_board_sigma, sigma(2.32-.5*(1.6-ms.CROSSWALK_CAMERA_LAG_S-1.)))
        self.assertAlmostEqual(motion._crosswalk_board_m, board(-.32, 2.32)-2*sigma(2.32)+2*motion._crosswalk_board_sigma)
        sharp, sharp_sigma = motion._crosswalk_board_m, motion._crosswalk_board_sigma
        # blunter frames placing it farther: ignored
        for n, t in enumerate((2.2, 2.3, 2.4)):
            frame(motion, 20+n, t, .4, 2.6)
        for k in range(21, 26):
            motion._crosswalk_observe(scan(k, k/10), track[k])
        self.assertEqual((motion._crosswalk_board_m, motion._crosswalk_board_sigma), (sharp, sharp_sigma))
        # blunter frames placing it nearer (captured early, placed late): taken, the sigma kept
        for n, t in enumerate((1.15, 1.2, 1.25)):
            frame(motion, 30+n, t, .4, 2.2)
        motion._crosswalk_observe(scan(26, 2.6), track[26])
        self.assertGreater(motion._crosswalk_board_m, sharp)
        self.assertEqual(motion._crosswalk_board_sigma, sharp_sigma)

    def test_frames_outside_the_start_lane_are_ignored(self):
        motion = start_motion()
        motion.route_stage = 'first_target'
        frame(motion, 1, 1., .8, 1.0)
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
