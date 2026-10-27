"""Round y2 (Claude): the traffic-light stop. With control['light'] (the race
program's read-only YOLO process) the gantry's legs set the light line, the
light lane's goal control stops the whole car in the .70 m zone before it and
the car waits there for green in fresh frames; without it nothing changes.
Car run 108's light-entry scans (fixture), synthetic scans and states, one
simulated run; no physical claim."""
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import threading
import time
import unittest

import maneuver_sequence as ms
from turn_motion import NEUTRAL, TRIAL_MOTOR
import test_route_58_light_lane as lane

ROOT = Path(__file__).resolve().parents[3]
FIXTURE = Path(__file__).parent/'fixtures'/'light_gantry_run108_frames.json'
FRONT = ms.FRONT_BODY_EXTENT_M


def ranges_with(posts=(), board=3., none=()):
    """A board ``board`` m ahead across the front half, posts as {bin: range}."""
    ranges = [board if i <= 90 or i >= 270 else 1. for i in range(360)]
    for i, r in dict(posts).items():
        ranges[i] = r
    for i in none:
        ranges[i] = None
    return ranges


class ThinPostsAndGantry(unittest.TestCase):
    def test_run_108_frames_match_the_fixture(self):
        # The Rust engine's test compares itself with these same results.
        frames = json.loads(FIXTURE.read_text())['frames']
        motion = ms.ManeuverSequence.__new__(ms.ManeuverSequence)
        found = 0
        for frame in frames:
            with self.subTest(seq=frame['seq']):
                python = frame['python']
                self.assertEqual([list(p) for p in ms._thin_posts(frame['ranges'])], python['posts'])
                gantry = motion._light_gantry({'ranges': frame['ranges']}, frame['compass_heading_rad'],
                                              frame['lane_target'])
                self.assertEqual(gantry, python['gantry'])
                found += gantry is not None
        self.assertEqual(found, 7)

    def test_a_thin_post_in_front_of_the_board(self):
        posts = ms._thin_posts(ranges_with({20: 1.5, 21: 1.51}))
        self.assertEqual(len(posts), 1)
        x, y = posts[0]
        self.assertAlmostEqual(math.hypot(x, y), 1.505, places=2)
        self.assertLess(y, 0)  # bins count clockwise: right of the car

    def test_board_pieces_seam_and_dropouts_are_not_posts(self):
        self.assertEqual(ms._thin_posts(ranges_with({20: 1.5, 21: 1.5, 22: 1.5, 23: 1.5, 24: 1.5, 25: 1.5,
                                                     26: 1.5, 27: 1.5, 28: 1.5})), [])  # 9 bins: a board
        self.assertEqual(ms._thin_posts(ranges_with({0: 1.5})), [])  # on the scan's seam
        self.assertEqual(ms._thin_posts(ranges_with({20: 1.5}, none=(19,))), [])  # beside a lost return
        self.assertEqual(ms._thin_posts(ranges_with({20: 2.95})), [])  # not in front of what lies behind

    def test_two_legs_across_the_lane_make_a_gantry_the_nearer_one_is_the_line(self):
        def bin_of(along, lateral):
            return round(-math.degrees(math.atan2(lateral, along))) % 360
        a, b = (2.0, .45), (2.1, -.5)
        ranges = ranges_with({bin_of(*a): math.hypot(*a), bin_of(*b): math.hypot(*b)}, board=4.)
        lane_target = {'outer_lateral_m': .57, 'inner_lateral_m': -.57}
        motion = ms.ManeuverSequence.__new__(ms.ManeuverSequence)
        gantry = motion._light_gantry({'ranges': ranges}, 0., lane_target)
        self.assertEqual(gantry['basis'], 'two_legs')
        self.assertAlmostEqual(gantry['along_m'], 2.0, delta=.03)
        self.assertGreater(gantry['legs'][0][1], gantry['legs'][1][1])  # left first
        one = ranges_with({bin_of(*a): math.hypot(*a)}, board=4.)
        single = motion._light_gantry({'ranges': one}, 0., lane_target)
        self.assertEqual(single['basis'], 'one_leg_inside_a_board')
        self.assertIsNone(motion._light_gantry({'ranges': one}, math.radians(50), lane_target))


class LightWait(unittest.TestCase):
    lane_motion = lane.LightLaneTests.lane_motion

    def scan(self, seq, board=3.):
        return {'seq': seq, 'at_ms': seq*100, 'received_at': seq/10, 'ranges': ranges_with(board=board)}

    def waiting(self, frames, stop_at=1.):
        """A car stopped before the light at ``stop_at``, the gantry confirmed .5 s earlier."""
        motion = self.lane_motion()
        motion.light_vision = True
        motion.light_stop_state, motion._light_gantry_since = 'approach', stop_at-.5
        motion._light_line = {'along_m': FRONT+.15, 'source': 'legs'}
        motion.light_stop_evidence = {'gantry': {}}
        for frame, captured, red, green in frames:
            if captured >= stop_at and motion.light_stop_state == 'approach':
                motion.lane_feedback = {}
                motion._light_stop_reached(self.scan(round(stop_at*10)), stop_at, {}, 'stopped')
            motion._light_observe({'light': {'frame': frame, 'captured_at': captured, 'red': red, 'green': green}})
        if motion.light_stop_state == 'approach':
            motion.lane_feedback = {}
            motion._light_stop_reached(self.scan(round(stop_at*10)), stop_at, {}, 'stopped')
        motion.drive_since = 0.
        return motion

    def test_frames_are_recorded_once_and_only_valid_ones(self):
        motion = self.lane_motion()
        motion._light_observe({})
        self.assertFalse(motion.light_vision)
        for light in ({'frame': 1, 'captured_at': .5, 'red': .9, 'green': None}, {'frame': 1, 'captured_at': .6},
                      {'frame': True, 'captured_at': .7}, {'frame': 2.0, 'captured_at': .7},
                      {'frame': 3, 'captured_at': float('nan')}, {'frame': 4, 'captured_at': .8, 'green': 'x'}):
            motion._light_observe({'light': light})
        self.assertTrue(motion.light_vision)
        self.assertEqual(motion._light_frames, [(1, .5, .9, 0.), (4, .8, 0., 0.)])
        for k in range(5, 12):
            motion._light_observe({'light': {'frame': k, 'captured_at': k/10}})
        self.assertEqual([f[0] for f in motion._light_frames], [7, 8, 9, 10, 11])  # round y3: 5 kept

    def test_green_in_two_fresh_frames_after_red_releases_and_the_wait_is_not_drive_time(self):
        motion = self.waiting([(1, .8, .9, 0.), (2, 1.2, .9, 0.), (3, 1.5, .9, 0.)])
        self.assertEqual(motion._light_red_frames, 3)
        self.assertFalse(motion._light_wait(self.scan(20), 2., {}))
        self.assertEqual((motion.drive_since, motion.light_wait_credit_s), (1., 1.))
        motion._light_observe({'light': {'frame': 4, 'captured_at': 2.1, 'red': 0., 'green': .88}})
        self.assertFalse(motion._light_wait(self.scan(22), 2.2, {}))  # one green frame
        motion._light_observe({'light': {'frame': 5, 'captured_at': 2.3, 'red': 0., 'green': .88}})
        self.assertTrue(motion._light_wait(self.scan(24), 2.4, {}))
        self.assertEqual(motion.light_stop_state, 'released')
        go = motion.light_stop_evidence['go']
        self.assertEqual(go['basis'], 'green_in_fresh_frames_after_the_stop')
        self.assertEqual(go['red_frames_since_gantry'], 3)
        self.assertTrue(go['launch'])  # 2.8 m to the end board
        self.assertAlmostEqual(motion.light_wait_credit_s, 1.4)
        self.assertAlmostEqual(motion.drive_since, 1.4)

    def test_red_or_stale_green_keeps_waiting(self):
        motion = self.waiting([(1, .8, .9, 0.), (2, .95, 0., .9), (3, 1.2, .3, .9)])
        self.assertFalse(motion._light_wait(self.scan(14), 1.4, {}))  # stale green, then red with green
        motion._light_observe({'light': {'frame': 4, 'captured_at': 1.5, 'red': 0., 'green': .9}})
        self.assertFalse(motion._light_wait(self.scan(16), 1.6, {}))
        motion._light_observe({'light': {'frame': 5, 'captured_at': 1.7, 'red': 0., 'green': .9}})
        self.assertFalse(motion._light_wait(self.scan(29), 2.9, {}))  # frame 4 is more than 1 s old by now

    def test_no_light_since_the_gantry_goes_after_4_s_and_looks_again(self):
        motion = self.waiting([(1, .8, 0., 0.), (2, 1.2, .1, .2)])
        self.assertFalse(motion._light_wait(self.scan(49), 4.9, {}))
        self.assertTrue(motion._light_wait(self.scan(50), 5., {}))
        self.assertIsNone(motion.light_stop_state)
        self.assertEqual(motion.light_stop_evidence['go']['basis'], 'no_light_seen_since_the_gantry_looking_again')
        self.assertEqual(len(motion.light_stop_evidence['rearmed']), 1)
        self.assertEqual(motion._light_rearm_min_m, ms.LIGHT_REARM_MIN_M)

    def test_red_seen_on_the_way_in_keeps_the_wait_8_s_from_the_first_red(self):
        # Stopped, the lamps may be out of the camera's view (~.35 m ahead);
        # round y3: green never seen, the car goes LIGHT_RED_GO_S after the first red.
        motion = self.waiting([(1, .7, .9, 0.), (2, 1.1, 0., 0.)])
        self.assertFalse(motion._light_wait(self.scan(60), 6., {}))
        self.assertFalse(motion._light_wait(self.scan(86), 8.6, {}))
        self.assertTrue(motion._light_wait(self.scan(88), 8.8, {}))
        self.assertEqual(motion.light_stop_state, 'released')
        self.assertEqual(motion.light_stop_evidence['go']['basis'], 'green_not_seen_red_go_time')

    def test_a_weak_green_alone_waits_until_the_max(self):
        motion = self.waiting([(1, .8, 0., .3), (2, 1.2, 0., .3)])
        self.assertTrue(motion._light_seen)
        self.assertFalse(motion._light_wait(self.scan(129), 12.9, {}))
        self.assertTrue(motion._light_wait(self.scan(130), 13., {}))
        self.assertEqual(motion.light_stop_state, 'released')
        self.assertEqual(motion.light_stop_evidence['go']['basis'], 'green_not_seen_light_wait_max')

    def test_green_without_any_red_since_the_gantry_is_not_the_light(self):
        motion = self.waiting([(1, .8, 0., .9), (2, 1.2, 0., .9), (3, 1.4, 0., .9)])
        self.assertTrue(motion._light_wait(self.scan(15), 1.5, {}))
        self.assertIsNone(motion.light_stop_state)
        self.assertEqual(motion.light_stop_evidence['go']['basis'], 'green_without_red_since_the_gantry_looking_again')
        self.assertEqual(motion._light_rearm_min_m, ms.LIGHT_REARM_MIN_M)

    def test_frames_before_the_gantry_do_not_count(self):
        motion = self.waiting([(1, .4, .9, 0.)])  # the gantry at .5
        self.assertEqual((motion._light_red_frames, motion._light_seen), (0, False))


class LightGoal(unittest.TestCase):
    lane_motion = lane.LightLaneTests.lane_motion

    def goal(self, ahead, goal, speed=0.):
        motion = self.lane_motion()
        motion._lane_speed = lambda scan, ahead, **_: speed
        scan = {'seq': 9, 'at_ms': 900, 'received_at': .9, 'ranges': ranges_with()}
        return motion, motion._lane_goal_control(scan, ahead, 1., goal)

    def test_stop_bases_name_the_goal(self):
        _, (basis, feedback) = self.goal(FRONT+.2, ms.LIGHT_GOAL)
        self.assertEqual(basis, 'stopped_with_the_whole_car_in_the_light_zone')
        self.assertEqual((feedback['goal'], feedback['speed_basis']), ('light', 'closing_rate_on_light_line_axis'))
        _, (basis, feedback) = self.goal(FRONT+.2, ms.FINISH_GOAL)
        self.assertEqual(basis, 'stopped_with_the_whole_car_in_the_finish_zone')
        self.assertEqual(feedback['goal'], 'finish')

    def test_the_light_zone_is_shorter_than_the_finish_zone(self):
        self.assertAlmostEqual(ms.LIGHT_STOP_BODY_GAP_M, .26)
        self.assertLess(ms.LIGHT_STOP_BODY_GAP_M+FRONT+ms.REAR_BODY_EXTENT_M, ms.LIGHT_ZONE_M)
        _, (basis, _) = self.goal(FRONT+.3, ms.LIGHT_GOAL)  # rear out of the light zone: creep
        self.assertIsNone(basis)
        _, (basis, _) = self.goal(FRONT+.3, ms.FINISH_GOAL)
        self.assertEqual(basis, 'stopped_with_the_whole_car_in_the_finish_zone')

    def test_no_line_on_a_scan_coasts(self):
        motion, (basis, feedback) = self.goal(None, ms.LIGHT_GOAL, speed=None)
        self.assertIsNone(basis)
        self.assertEqual(motion.lane_motor, NEUTRAL)

    def test_the_launch_after_green_and_its_end(self):
        motion = self.lane_motion()
        motion._lane_speed = lambda scan, ahead, **_: 0.
        motion._light_launch_until = 1.5
        scan = {'seq': 9, 'at_ms': 900, 'received_at': .9, 'ranges': ranges_with()}
        self.assertIsNone(motion._lane_goal_control(scan, FRONT+3., 1., ms.FINISH_GOAL)[0])
        self.assertEqual((motion.lane_motor, motion._lane_pulse), (TRIAL_MOTOR, 0))
        motion._lane_speed = lambda scan, ahead, **_: .8
        motion._lane_goal_control(scan, FRONT+3., 1.6, ms.FINISH_GOAL)
        self.assertEqual(motion.lane_motor, ms.ROUTE_SLOW_MOTOR)
        self.assertIsNone(motion._light_launch_until)


class LightApproach(unittest.TestCase):
    lane_motion = lane.LightLaneTests.lane_motion

    def approaching(self, ahead, source):
        motion = self.lane_motion()
        motion.light_vision, motion.light_stop_state = True, 'approach'
        motion.light_stop_evidence = {'gantry': {}}
        motion._light_line = {'along_m': ahead, 'source': source}

        def line_ahead(scan, heading, lane_target, now):
            motion._light_line['source'] = source
            return ahead
        motion._light_line_ahead = line_ahead
        scan = {'seq': 9, 'at_ms': 900, 'received_at': .9, 'ranges': ranges_with()}
        return motion, motion._light_stop_track(scan, 1., {}, 0., {})

    def test_a_front_measured_past_the_line_ends_the_approach(self):
        motion, ahead = self.approaching(FRONT-ms.LIGHT_PASSED_M-.01, 'legs')
        self.assertIsNone(ahead)
        self.assertIsNone(motion.light_stop_state)
        self.assertEqual(len(motion.light_stop_evidence['passed']), 1)
        self.assertEqual(motion._light_rearm_min_m, ms.LIGHT_REARM_MIN_M)

    def test_a_held_line_or_the_front_at_the_line_goes_on(self):
        for ahead, source in ((FRONT-ms.LIGHT_PASSED_M-.01, 'held'), (FRONT-ms.LIGHT_PASSED_M+.01, 'legs')):
            with self.subTest(source=source):
                motion, value = self.approaching(ahead, source)
                self.assertEqual(value, ahead)
                self.assertEqual(motion.light_stop_state, 'approach')

    def test_without_light_vision_nothing_starts(self):
        motion = self.lane_motion()
        scan = {'seq': 9, 'at_ms': 900, 'received_at': .9, 'ranges': ranges_with()}
        self.assertIsNone(motion._light_stop_track(scan, 1., {}, 0., {'outer_lateral_m': .5, 'inner_lateral_m': -.5}))
        self.assertIsNone(motion.light_stop_state)


def bin_of(along, lateral):
    return round(-math.degrees(math.atan2(lateral, along))) % 360


def legs_scan(seq, *legs):
    return {'seq': seq, 'at_ms': seq*100, 'received_at': seq/10,
            'ranges': ranges_with({bin_of(*leg): math.hypot(*leg) for leg in legs}, board=4.)}


class LightLegs(unittest.TestCase):
    """The gantry's legs while the car turns in the light entry (simulation,
    fast car: a line on the outer leg alone was lost while the inner one stayed
    in view and the car ran through the light)."""
    lane_motion = lane.LightLaneTests.lane_motion
    LANE = {'outer_lateral_m': .57, 'inner_lateral_m': -.57, 'center_lateral_m': 0.}

    def test_a_line_on_one_leg_takes_the_pair_it_is_part_of(self):
        motion = self.lane_motion()
        outer = ms._thin_posts(legs_scan(9, (2.0, .45))['ranges'])[0]
        motion._light_line = {'along_m': outer[0]+.05, 'at_s': .8, 'source': 'legs', 'legs': [[outer[0]+.05, outer[1]]],
                              'speed_mps': None, 'board_m': None, 'leg_offsets_m': [0.]}
        ahead = motion._light_line_ahead(legs_scan(9, (2.0, .45), (2.1, -.5)), 0., self.LANE, .9)
        line = motion._light_line
        self.assertEqual(len(line['legs']), 2)
        self.assertEqual(ahead, min(leg[0] for leg in line['legs']))  # the nearer leg
        self.assertEqual(line['leg_offsets_m'], [leg[0]-ahead for leg in line['legs']])
        # the outer leg gone, the inner one alone carries the line
        inner = line['legs'][1]
        later = motion._light_line_ahead(legs_scan(10, (inner[0]-.1, inner[1])), 0., self.LANE, 1.)
        self.assertEqual(motion._light_line['source'], 'legs')
        self.assertAlmostEqual(later, ahead-.1, delta=.03)

    def test_one_post_goes_to_the_leg_nearest_its_lateral(self):
        # A second without legs: both lateral windows reach the inner leg's
        # post; it is the inner leg's, and the outer one moves rigidly.
        motion = self.lane_motion()
        motion._light_line = {'along_m': 2.0, 'at_s': 0., 'source': 'legs', 'legs': [[2.0, .45], [2.0, -.45]],
                              'speed_mps': None, 'board_m': None, 'leg_offsets_m': [0., 0.]}
        post = ms._thin_posts(legs_scan(10, (1.9, -.4))['ranges'])[0]
        ahead = motion._light_line_ahead(legs_scan(10, (1.9, -.4)), 0., self.LANE, 1.)
        outer, inner = motion._light_line['legs']
        self.assertEqual(inner, list(post))
        self.assertEqual(ahead, post[0])
        self.assertAlmostEqual(outer[1], .45+(post[1]+.45), places=9)  # not the inner post's lateral
        self.assertEqual(ms._light_leg_matches([(1.9, -.4)], [[2.0, .45], [2.0, -.45]], 4., 1.15), {1: (1.9, -.4)})

    def test_before_confirmation_the_last_pair_is_followed_before_one_leg(self):
        motion = self.lane_motion()
        self.assertIsNone(motion._light_line_ahead(legs_scan(9, (3.0, .45), (3.05, -.5)), 0., self.LANE, .9))
        self.assertEqual(motion._light_gantry_seen[-1]['basis'], 'two_legs')
        ahead = motion._light_line_ahead(legs_scan(10, (2.9, .45)), 0., self.LANE, 1.)
        self.assertIsNotNone(ahead)  # the pair and one more sighting of it
        self.assertEqual(motion._light_gantry_seen[-1]['basis'], 'leg_followed')
        self.assertEqual(len(motion._light_line['legs']), 2)


def course_sim():
    spec = importlib.util.spec_from_file_location('course_sim_y2', ROOT/'scripts'/'course-sim.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class SimulatedLight(unittest.TestCase):
    def test_measured_car_stops_in_the_zone_waits_for_green_and_finishes(self):
        cs = course_sim()
        course = cs.Course(**cs.LAYOUTS['user8'], gantry=True)
        vehicle = {'steer_tau': .25, 'play_pwm': 40., 'speed_scale': 1., 'decel': .6, 'gain_scale': 1.}
        summary, _, _ = cs.run(course, max_s=75., vehicle='measured', delay=.3, vehicle_params=vehicle,
                               light={'red_s': 10.})
        light = summary['light']
        self.assertTrue(summary['route_completed'], summary['end_reason'])
        self.assertTrue(light['stopped_in_zone'], light)
        self.assertFalse(light['went_before_green'], light)
        self.assertEqual(light['penalty_s'], 0)
        self.assertGreaterEqual(light['stationary_s'], 7.)  # the red less the approach after the trigger


ENGINE = ROOT/'route-engine'/'target'/'release'/'route-engine'
GEOMETRY = ROOT/'tools'/'scan-geometry'/'target'/'release'/'xt-stcar-scan-geometry'
BRIDGE = ROOT/'scripts'/'sim-vehicle-bridge.py'


@unittest.skipUnless(ENGINE.is_file() and GEOMETRY.is_file(), 'route-engine or tools/scan-geometry not built')
class RaceProgramLightState(unittest.TestCase):
    def test_the_light_state_file_rides_on_the_control_lines(self):
        with tempfile.TemporaryDirectory() as temp:
            temp = Path(temp)
            (temp/'shared').mkdir()
            state = temp/'light-state.json'
            stop = threading.Event()

            def write():
                frame = 0
                while not (temp/'run').exists() and not stop.is_set():
                    time.sleep(.01)  # from the program's start on
                while not stop.is_set():
                    now = time.monotonic()
                    text = (json.dumps({'frame': frame, 'captured_s': now-.25, 'written_s': now, 'red': .9, 'green': 0.})
                            if frame != 3 else '{"frame": "x"}')
                    tmp = temp/'light-state.tmp'
                    tmp.write_text(text)
                    os.replace(tmp, state)
                    frame += 1
                    time.sleep(.25)
            writer = threading.Thread(target=write)
            writer.start()
            try:
                p = subprocess.run([str(ENGINE), 'race', '--bridge', str(BRIDGE), '--demo', '--out', str(temp/'run'),
                                    '--status-s', '5', '--start', 'ready', '--ready-hold-s', '.3', '--max-drive-s', '2',
                                    '--light-state', str(state)],
                                   env={**os.environ, 'SIM_SHARED': str(temp/'shared'), 'SIM_LAYOUT': 'user8'},
                                   capture_output=True, text=True, timeout=60, stdin=subprocess.DEVNULL)
            finally:
                stop.set()
                writer.join()
            self.assertEqual(p.returncode, 0, p.stdout+p.stderr)
            summary = json.loads((temp/'run'/'summary.json').read_text())
            self.assertGreater(summary['light']['updates'], 5)
            self.assertEqual(summary['light']['invalid'], 1)
            rows = [json.loads(line) for line in (temp/'run'/'light.jsonl').read_text().splitlines()]
            self.assertEqual(len(rows), summary['light']['updates'])
            for row in rows:
                self.assertAlmostEqual(row['received_at']-row['light']['captured_at'], .25, delta=1e-6)
            feedback = [json.loads(line) for line in (temp/'run'/'feedback.jsonl').read_text().splitlines()]
            lights = [r['state']['status']['control'].get('light') for r in feedback]
            self.assertTrue(any(isinstance(light, dict) and light['red'] == .9 for light in lights))


if __name__ == '__main__':
    unittest.main()
