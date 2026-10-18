"""Start-lane wall law (2026-10-18). Rounds 64/66/67/68 weaved +-8 deg in the
start lane: the old law (CorridorSteering, ~200 nominal PWM per rad behind a
10 PWM/100 ms ramp, no lead) is past its stability limit when the real
near-centre steering answers 1.5-3x the table about .35 s late. The new law
turns toward the lane by the measured error, led by the measured wall-heading
trend, with a small gain; the start-lane speed needs three closing rates."""
import importlib.util
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).parents[1]))
import maneuver_sequence as ms  # noqa: E402
from maneuver_sequence import (ManeuverSequence, LaneWallSteering, LANE_WALL_CAP_PWM, LANE_WALL_GAIN_PWM_PER_RAD,
                               LANE_WALL_LEAD_S, LANE_WALL_LOOKAHEAD_M, NEUTRAL)  # noqa: E402
from autonomy_live import CorridorSteering  # noqa: E402

ROOT = Path(__file__).parents[3]


def walls(heading_deg=0., offset=0., width=.93):
    return {'heading_right_deg': heading_deg, 'offset_right_m': offset,
            'slope': math.tan(math.radians(heading_deg)), 'width_m': width}


class LaneWallLaw(unittest.TestCase):
    def test_turns_toward_the_lane_by_the_error(self):
        law = LaneWallSteering()
        self.assertLess(law.update(walls(2.), 1, 0., True, 100), NEUTRAL)  # lane turns right of the nose
        law = LaneWallSteering()
        self.assertGreater(law.update(walls(-2.), 1, 0., True, 100), NEUTRAL)
        law = LaneWallSteering()
        self.assertLess(law.update(walls(0., .05), 1, 0., True, 100), NEUTRAL)  # centre to the right
        law = LaneWallSteering()
        expected = round(LANE_WALL_GAIN_PWM_PER_RAD*(math.radians(1.)+.03/LANE_WALL_LOOKAHEAD_M))
        self.assertEqual(law.update(walls(1., .03), 1, 0., True, 100), NEUTRAL-expected)

    def test_capped(self):
        law = LaneWallSteering()
        self.assertEqual(law.update(walls(12., .2), 1, 0., True, 100), NEUTRAL-LANE_WALL_CAP_PWM)
        law = LaneWallSteering()
        self.assertEqual(law.update(walls(-12., -.2), 1, 0., True, 100), NEUTRAL+LANE_WALL_CAP_PWM)

    def test_lead_releases_before_the_error_is_gone(self):
        # Heading 4 -> 3 -> 2 deg over .2 s: the car is already turning back
        # at 10 deg/s, so with the lead the command is neutral or opposite
        # while 2 deg of error remain (the old law still pushed right).
        law = LaneWallSteering()
        for k, h in enumerate((4., 3., 2.)):
            out = law.update(walls(h), k+1, 0., True, 100*(k+1))
        self.assertGreaterEqual(out, NEUTRAL)
        self.assertAlmostEqual(law.evidence['trend_rad_s'], math.radians(-10.))
        self.assertAlmostEqual(law.evidence['predicted_rad'], math.radians(2.)+LANE_WALL_LEAD_S*math.radians(-10.))
        old = CorridorSteering()
        for k, h in enumerate((4., 3., 2.)):
            old_out = old.update(walls(h, .03), k+1, k*.1, True, 100*(k+1))
        self.assertLessEqual(old_out, NEUTRAL)

    def test_trend_needs_three_consistent_publications(self):
        law = LaneWallSteering()
        law.update(walls(4.), 1, 0., True, 100)
        law.update(walls(3.), 2, 0., True, 200)
        self.assertEqual(law.evidence['trend_rad_s'], 0.)
        law = LaneWallSteering()
        law.update(walls(4.), 1, 0., True, 100)
        law.update(walls(3.), 2, 0., True, 200)
        law.update(walls(2., .3), 3, 0., True, 300)  # offset jump: another fit, restart
        self.assertEqual((law.evidence['history'], law.evidence['trend_rad_s']), (1, 0.))
        law = LaneWallSteering()
        law.update(walls(4.), 1, 0., True, 100)
        law.update(walls(3.), 2, 0., True, 200)
        law.update(walls(2.), 3, 0., True, 700)  # .5 s gap: not one trend
        self.assertEqual((law.evidence['history'], law.evidence['trend_rad_s']), (1, 0.))

    def test_same_scan_holds_and_bad_order_releases(self):
        law = LaneWallSteering()
        first = law.update(walls(3.), 5, 0., True, 500)
        self.assertEqual(law.update(walls(-9.), 5, .05, True, 500), first)
        self.assertEqual(law.update(walls(3.), 4, .1, True, 600), NEUTRAL)  # seq went back
        law = LaneWallSteering()
        law.update(walls(3.), 5, 0., True, 500)
        self.assertEqual(law.update(walls(3.), 6, .1, True, 500), NEUTRAL)  # publication did not advance
        self.assertEqual(law.update(None, 7, .2, True, 700), NEUTRAL)
        self.assertEqual(law.update(walls(3.), 8, .3, False, 800), NEUTRAL)

    def test_only_the_start_lane_uses_it(self):
        motion = ManeuverSequence(0., continue_route=True, max_drive_s=40.)
        self.assertIsInstance(motion._lane_start_steering, LaneWallSteering)
        self.assertIsInstance(motion._lane_steering, CorridorSteering)


class StartLaneSpeed(unittest.TestCase):
    def feed(self, motion, ranges, **kw):
        out = []
        for k, ahead in enumerate(ranges):
            scan = {'at_ms': 1000+100*k, 'received_at': 10.+.1*k}
            out.append(motion._lane_speed(scan, ahead, **kw))
        return out

    def test_default_unchanged_three_rates_for_the_start_lane(self):
        motion = ManeuverSequence(0., continue_route=True, max_drive_s=40.)
        default = self.feed(motion, [7., 6.9, 6.8, 6.7, 6.6])
        self.assertEqual(default[:2], [None, None])
        self.assertAlmostEqual(default[2], 1.)
        motion = ManeuverSequence(0., continue_route=True, max_drive_s=40.)
        start = self.feed(motion, [7., 6.9, 6.8, 6.7, 6.6], min_rates=3)
        self.assertEqual(start[:4], [None]*4)
        self.assertAlmostEqual(start[4], 1.)

    def test_one_jump_of_the_far_ray_is_rejected(self):
        # 2026-10-18 simulation: 7.29 -> 7.02 -> 7.05 at ~.9 m/s gave 2.65 m/s
        # from two rates and cut the start lane.
        motion = ManeuverSequence(0., continue_route=True, max_drive_s=40.)
        self.assertGreater(self.feed(motion, [7.29, 7.02, 7.05])[-1], 2.)
        motion = ManeuverSequence(0., continue_route=True, max_drive_s=40.)
        speeds = self.feed(motion, [7.29, 7.02, 7.05, 6.96, 6.87], min_rates=3)
        self.assertLess(speeds[-1], 1.)


class ClosedLoopWeave(unittest.TestCase):
    """course-sim, .94 m start lane, real-like steering (wheels start on the
    right side of the play as in round 68, near-centre gain 2, delay .3)."""
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location('course_sim_69_law', ROOT/'scripts/course-sim.py')
        cls.cs = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.cs)

    def run_lane(self, params, delay):
        cs = self.cs
        course = cs.Course(**cs.layout(8.1, 4.15, .94, 3.2))
        summary, log, _ = cs.run(course, max_s=8., vehicle='measured', delay=delay, vehicle_params=params)
        rows = [r for r in log if r[7] == 'start_lane' and r[8] == 'drive' and r[1] <= 3.2]
        head = [r[3] for r in rows]
        return summary, rows, max(head)-min(head)

    def test_start_lane_swing_small_without_a_lane_halt(self):
        for params, delay in (({'play_pwm': 50., 'near_center_gain': 2., 'initial_wheel_pwm': 1450.}, .3),
                              ({'play_pwm': 50., 'near_center_gain': 3., 'initial_wheel_pwm': 1550.}, .25),
                              ({}, .15)):
            with self.subTest(params=params, delay=delay):
                summary, rows, swing = self.run_lane(params, delay)
                self.assertGreater(rows[-1][1], 3.)  # left the start lane by driving, not a stop
                self.assertLess(swing, 8.)
                self.assertGreater(min(r[10] for r in rows), .15)
                self.assertEqual(summary['contacts'], [])

    def test_initial_wheel_must_lie_inside_the_play(self):
        with self.assertRaises(ValueError):
            self.cs.MeasuredVehicle(0., 0., 0., play_pwm=20., initial_wheel_pwm=1450.)


if __name__ == '__main__':
    unittest.main()


class StartLaneSpeedTrack(unittest.TestCase):
    """The nearest return can switch surfaces; the speed follows directions."""
    def scan(self, k, ranges):
        return {'seq': k, 'at_ms': 1000+100*k, 'received_at': 10.+.1*k, 'ranges': ranges}

    def wall(self, distance, near=None):
        # A board ahead perpendicular to the axis; optionally bins -1/-2 meet a
        # nearer surface (a depth step) that may or may not be in the sector.
        ranges = [None]*360
        for i in range(-6, 7):
            ranges[i % 360] = distance/math.cos(math.radians(i))
        if near is not None:
            for i in (-2, -1):
                ranges[i % 360] = near/math.cos(math.radians(i))
        return ranges

    def motion(self):
        motion = ManeuverSequence(0., continue_route=True, max_drive_s=40.)
        motion.compass.current = lambda scan: 0.
        return motion

    def test_track_ignores_the_nearest_return_switching_surface(self):
        motion = self.motion()
        values = []
        for k in range(6):
            d = 7.-.1*k
            near = d-.15 if k % 2 else None  # the step flickers in and out
            ranges = self.wall(d, near)
            ahead = min(r for r in ranges[-3:]+ranges[:4] if r is not None)
            values.append(motion._lane_front_track(self.scan(k, ranges), ahead))
        steps = [a-b for a, b in zip(values, values[1:])]
        for step in steps:
            self.assertAlmostEqual(step, .1, places=6)

    def test_least_squares_speed_after_four_points(self):
        motion = self.motion()
        speeds = [motion._track_speed(self.scan(k, []), 7.-.12*k) for k in range(6)]
        self.assertEqual(speeds[:3], [None]*3)
        self.assertAlmostEqual(speeds[3], 1.2)
        self.assertAlmostEqual(speeds[5], 1.2)
        self.assertIsNone(motion._track_speed(self.scan(6, []), None))  # lost: restart

    def test_unknown_speed_after_a_measured_one_cruises(self):
        gov = ManeuverSequence._start_governor
        self.assertEqual(gov(None, 0., 1.5), ms.TRIAL_MOTOR)  # launch
        self.assertEqual(gov(None, 0., 1.5, True), ms.ROUTE_SLOW_MOTOR)
        self.assertEqual(gov(None, 0., .5, True), NEUTRAL)


class LaneWallPlay(unittest.TestCase):
    def test_play_override_bounds_and_default(self):
        import steer_calibration as sc
        a, b = sc.SteerOutput(), sc.SteerOutput()
        for nominal in (1500, 1520, 1510, 1490, 1500):
            self.assertEqual(a.physical(nominal), b.physical(nominal, sc.STEER_PLAY_PWM))
        with self.assertRaises(ValueError):
            sc.SteerOutput().physical(1520, sc.STEER_PLAY_PWM+1)
        c = sc.SteerOutput()
        self.assertEqual(c.physical(1520, 0), round(sc.wheel_for_curvature(20/160)))

    def test_only_wall_law_commands_are_marked(self):
        motion = ManeuverSequence(0., continue_route=True, max_drive_s=40.)
        motion.route_stage, motion.phase, motion.reason = 'start_lane', 'drive', 'start_lane_lane_walls'
        self.assertTrue(motion.lane_wall_output_active())
        motion.reason = 'start_lane_first_support_approach'
        self.assertFalse(motion.lane_wall_output_active())
        motion.phase, motion.start_lane_coast_steering = 'coast', {'target_pwm': 1500}
        self.assertTrue(motion.lane_wall_output_active())
        motion.route_stage = 'first_target'
        self.assertFalse(motion.lane_wall_output_active())


class CoastKeepsTheLane(unittest.TestCase):
    """A start-lane coast (board ahead forced at scan 18) on a car whose real
    play is smaller than the calibration's: the old coast held the calibrated
    'neutral' (physical 1450 after a left correction) into the right board."""
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location('course_sim_69_coast', ROOT/'scripts/course-sim.py')
        cls.cs = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.cs)

    def run_coast(self, steer):
        cs = self.cs
        orig_range, orig_update = ms.ManeuverSequence._lane_front_range, ms.ManeuverSequence._start_lane_update

        def front(self, scan):
            value = orig_range(self, scan)
            return 1.2 if self.route_stage == 'start_lane' and scan['seq'] >= 18 and value is not None else value

        def update(self, scan, *a, **k):
            if self.phase == 'coast' and not steer:
                saved = ms.corridor_walls
                ms.corridor_walls = lambda scan: None
                try:
                    return orig_update(self, scan, *a, **k)
                finally:
                    ms.corridor_walls = saved
            return orig_update(self, scan, *a, **k)
        course = cs.Course(**cs.layout(8.1, 4.15, .94, 3.2))
        x, y, psi = course.start
        course.start = (x, y-.04, psi+math.radians(2))
        try:
            ms.ManeuverSequence._lane_front_range, ms.ManeuverSequence._start_lane_update = front, update
            summary, log, _ = cs.run(course, max_s=6., vehicle='measured', delay=.25,
                                     vehicle_params={'play_pwm': 0., 'near_center_gain': 2.})
        finally:
            ms.ManeuverSequence._lane_front_range, ms.ManeuverSequence._start_lane_update = orig_range, orig_update
        coast = [r for r in log if r[8] == 'coast']
        return summary, coast

    def test_coast_follows_the_walls(self):
        summary, coast = self.run_coast(True)
        self.assertTrue(coast)
        self.assertEqual(summary['contacts'], [])
        heading = [abs(r[3]) for r in coast]
        held_summary, held = self.run_coast(False)
        self.assertLess(max(heading), max(abs(r[3]) for r in held))
        self.assertGreater(min(r[10] for r in coast), min(r[10] for r in held))


class RearStartZone(unittest.TestCase):
    """User 2026-10-18: the start zone may put the car against the start-end
    wall; returns behind the rear face do not block a forward route start."""
    def scan(self, extra):
        import autonomy_live as al  # noqa: F401
        ranges = [2.]*360
        for angle, r in extra.items():
            ranges[angle] = r
        return {'seq': 7, 'at_ms': 700, 'frame_id': 'lidar_origin_coarse_body_heading', 'ranges': ranges}

    def probe(self, scan, **kw):
        from autonomy_live import probe_clearance
        return probe_clearance(scan, {'lidar': .05, 'control': .01, 'camera': 0.}, camera_required=False,
                               clearance_profile='maneuver', **kw)

    def test_wall_behind_blocks_only_without_the_start_exemption(self):
        from autonomy_live import ProbeClearanceError
        wall = {a: .223/abs(math.cos(math.radians(a))) for a in range(160, 201)}  # 4 cm behind the rear
        with self.assertRaises(ProbeClearanceError):
            self.probe(self.scan(wall))
        result = self.probe(self.scan(wall), rear_start=True)
        self.assertTrue(result['motion_ready'])
        self.assertTrue(result['clearance_gates']['rear_start_exemption_active'])
        self.assertTrue(result['body_proximity']['rear_start_exempt_points'])
        self.assertTrue(all(p['forward_x_m'] < -.15 for p in result['body_proximity']['rear_start_exempt_points']))
        touching = {a: .182/abs(math.cos(math.radians(a)))+(-.004 if a % 2 else .004) for a in range(150, 211)}
        self.assertTrue(self.probe(self.scan(touching), rear_start=True)['motion_ready'])  # bumper on the wall
        # Behind and beside the rear corner (x < -.18): still behind the rear face.
        self.assertTrue(self.probe(self.scan({135: .27}), rear_start=True)['motion_ready'])

    def test_sides_and_front_keep_the_gate(self):
        from autonomy_live import ProbeClearanceError
        for extra in ({90: .19}, {270: .19}, {0: .25}, {110: .2}):  # right, left, front, beside the rear
            with self.subTest(extra=extra), self.assertRaises(ProbeClearanceError):
                self.probe(self.scan(extra), rear_start=True)

    def test_only_the_maneuver_profile_and_a_bool(self):
        from autonomy_live import probe_clearance
        with self.assertRaises(ValueError):
            probe_clearance(self.scan({}), {'lidar': .05, 'control': .01, 'camera': 0.}, camera_required=False,
                            rear_start=True)
        with self.assertRaises(ValueError):
            self.probe(self.scan({}), rear_start=1)

    def test_server_scope_is_the_route_start_lane(self):
        import server
        motion = ManeuverSequence(0., continue_route=True, max_drive_s=40.)
        session = {'turn_trial': True, 'continue_route': True, 'turn_motion': motion}
        motion.route_stage, motion.phase = 'start_lane', 'drive'
        self.assertTrue(server.Console.turn_rear_start(session))
        motion.phase = 'coast'
        self.assertTrue(server.Console.turn_rear_start(session))
        motion.route_stage = 'first_target'
        self.assertFalse(server.Console.turn_rear_start(session))
        motion.route_stage = 'start_lane'
        self.assertFalse(server.Console.turn_rear_start({**session, 'continue_route': False}))
