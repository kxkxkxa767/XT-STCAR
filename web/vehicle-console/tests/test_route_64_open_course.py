"""Round 64 (Claude): open-course route for the correctly laid course (3.2 m
side straights): wall compass, loop plan/pursuit, axis lines, rotation-
compensated tracking, axis front range, and closed-loop course simulations
(scripts/course-sim.py: real Console + ManeuverSequence on ray-cast scans with
the vehicle bridge's Rust geometry). Synthetic data only; no physical claim."""
import importlib.util
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).parents[1]))
import maneuver_sequence as ms  # noqa: E402
from maneuver_sequence import (ManeuverSequence, WallCompass, _axis_frame, _support_center, _probe_free,
                               _loop_extent, _plan_loop, _loop_carrot, _pursuit_pwm, _strip_clear,
                               _axis_lines, _segment_distance, OPEN_LEFT_RADIUS_M, OPEN_RIGHT_RADIUS_M,
                               OPEN_INNER_GAP_M, OPEN_OUTER_GAP_M, OPEN_PROBE_DIRECTIONS_RAD,
                               SIDE_BODY_EXTENT_M, FRONT_BODY_EXTENT_M, ROUTE_PWM_PER_CURVATURE,
                               RIGHT_EXIT_MIN_PWM, POST_PASS_STAGES, OPEN_STAGES, TASK_LEFT_BYPASS_STAGES)
from compact_target import CompactTargetTracker, rotated_observation
from turn_motion import NEUTRAL, SERVO_MAX

ROOT = Path(__file__).resolve().parents[3]
SIM = ROOT/'scripts'/'course-sim.py'
TOOL = ROOT/'tools'/'scan-geometry'/'target'/'release'/'xt-stcar-scan-geometry'


def wall(heading, rho, start, end, span=None, points=60):
    """A native finite wall candidate in the car frame (left +)."""
    return {'heading_left_rad': heading, 'rho_left_m': rho,
            'support_start_left_m': {'x_m': start[0], 'y_m': start[1]},
            'support_end_left_m': {'x_m': end[0], 'y_m': end[1]},
            'support_span_m': span if span is not None else math.dist(start, end),
            'points': points, 'fit_error_m': .005, 'candidate_only': True}


def box_walls(yaw):
    """Four walls of a box around the car, the box rotated by -yaw (car yawed +yaw)."""
    walls = []
    for angle, rho in ((0., 1.), (math.pi/2, 2.), (0., -1.), (math.pi/2, -3.)):
        h = math.atan2(math.sin(angle-yaw), math.cos(angle-yaw))
        n = (-math.sin(h), math.cos(h))
        d = (math.cos(h), math.sin(h))
        mid = (n[0]*rho, n[1]*rho)
        walls.append(wall(h, rho, (mid[0]-d[0], mid[1]-d[1]), (mid[0]+d[0], mid[1]+d[1])))
    return walls


def arc_ranges(center, radius, background=None):
    """Ranges of a lone circle (support) seen from the origin; ``background`` elsewhere."""
    ranges = [background]*360
    for i in range(360):
        dx, dy = math.cos(math.radians(i)), -math.sin(math.radians(i))
        b = -(center[0]*dx+center[1]*dy)
        disc = b*b-(center[0]**2+center[1]**2-radius*radius)
        if disc >= 0 and -b-math.sqrt(disc) > 0:
            ranges[i] = -b-math.sqrt(disc)
    return ranges


class WallCompassTests(unittest.TestCase):
    def test_heading_follows_a_full_left_turn_without_wrapping(self):
        compass = WallCompass()
        heading = None
        for k in range(0, 73):
            yaw = math.radians(5*k)  # 0 .. 360 deg in 5 deg steps at 10 Hz
            heading = compass.update({'at_ms': 1000+100*k, 'wall_candidates': box_walls(yaw)})
            self.assertAlmostEqual(heading, yaw, delta=math.radians(.5))
        self.assertGreater(heading, 2*math.pi-.01)  # continuous, never wrapped

    def test_no_heading_after_the_stale_limit(self):
        compass = WallCompass()
        compass.update({'at_ms': 1000, 'wall_candidates': box_walls(.1)})
        self.assertIsNotNone(compass.current({'at_ms': 1900}))
        compass.update({'at_ms': 2200, 'wall_candidates': []})
        self.assertIsNone(compass.current({'at_ms': 2200}))

    def test_short_or_rough_walls_do_not_vote(self):
        compass = WallCompass()
        walls = [wall(0., 1., (-.2, 1.), (.2, 1.)), {**wall(0., -1., (-1, -1), (1, -1)), 'fit_error_m': .05}]
        compass.update({'at_ms': 1000, 'wall_candidates': walls})
        self.assertIsNone(compass.heading)


class GeometryTests(unittest.TestCase):
    def test_axis_frame_is_orthonormal_and_turn_side_signed(self):
        for heading in (0., .7, math.pi, -2.):
            for side in (1, -1):
                e1, e2 = _axis_frame(0., heading, side)
                self.assertAlmostEqual(math.hypot(*e1), 1.)
                self.assertAlmostEqual(e1[0]*e2[0]+e1[1]*e2[1], 0.)
                self.assertAlmostEqual(e1[0]*e2[1]-e1[1]*e2[0], side)  # +1: e2 left of e1

    def test_support_center_from_its_visible_arc(self):
        center, radius = (2., .8), .1
        ranges = arc_ranges(center, radius)
        bins = [i for i, r in enumerate(ranges) if r is not None]
        estimate, r = _support_center(ranges, bins, 2*radius)
        self.assertAlmostEqual(r, radius)
        self.assertLess(math.dist(estimate, center), .03)  # 2r/pi model of a near-half arc

    def test_probe_skips_the_support_and_finds_the_next_return(self):
        points = [(2., .5+k*.01) for k in range(-5, 6)]  # the support itself
        points += [(2.+k*.05, 1.6) for k in range(-10, 11)]  # a board 1.1 m beyond, across the probe
        self.assertAlmostEqual(_probe_free(points, (2., .5), (0., 1.), .18), 1.1, places=6)
        self.assertEqual(_probe_free(points, (2., .5), (0., -1.), .18), math.inf)

    def test_loop_extent_of_a_centred_circle_is_its_radius(self):
        for theta in OPEN_PROBE_DIRECTIONS_RAD:
            self.assertAlmostEqual(_loop_extent(0., 0., .9, theta), .9)
        self.assertAlmostEqual(_loop_extent(0., -.2, .9, math.pi/2), .7)
        self.assertAlmostEqual(_loop_extent(0., -.2, .9, -math.pi/2), 1.1)

    def test_plan_centres_the_body_between_support_and_a_near_board(self):
        support = .1
        lo = support+SIDE_BODY_EXTENT_M+OPEN_INNER_GAP_M
        # Board 1.1 m beyond the support on the return side, open elsewhere.
        free = [(t, 1.1 if abs(t-math.pi/2) < 1e-9 else math.inf) for t in OPEN_PROBE_DIRECTIONS_RAD]
        plan = _plan_loop(free, support, OPEN_LEFT_RADIUS_M)
        self.assertLess(plan['offset_side_m'], 0.)  # centre moved toward the approach side
        hi = 1.1-SIDE_BODY_EXTENT_M-OPEN_OUTER_GAP_M
        extent = _loop_extent(plan['offset_axis_m'], plan['offset_side_m'], plan['radius_m'], math.pi/2)
        self.assertGreaterEqual(extent, lo-1e-9)
        self.assertLessEqual(extent, hi+1e-9)
        self.assertTrue(OPEN_LEFT_RADIUS_M[0] <= plan['radius_m'] <= OPEN_LEFT_RADIUS_M[1])

    def test_plan_without_obstacles_keeps_the_widest_centred_loop(self):
        free = [(t, math.inf) for t in OPEN_PROBE_DIRECTIONS_RAD]
        plan = _plan_loop(free, .1, OPEN_RIGHT_RADIUS_M)
        self.assertAlmostEqual(plan['radius_m'], OPEN_RIGHT_RADIUS_M[1])
        self.assertLessEqual(math.hypot(plan['offset_axis_m'], plan['offset_side_m']), .05+1e-9)

    def test_carrot_on_the_approach_line_then_on_the_circle(self):
        e1, e2 = (1., 0.), (0., 1.)
        # Circle (anticlockwise) centred 3 m ahead, .5 m left: approach line y = -.4.
        point, mode, before = _loop_carrot((3., .5), .9, 1, 1.1, e1, e2)
        self.assertEqual(mode, 'approach_line')
        self.assertAlmostEqual(point[1], -.4)
        self.assertAlmostEqual(point[0], 1.1)
        self.assertAlmostEqual(before, 3.)
        # On the circle, moving along it: pure pursuit asks for its curvature.
        point, mode, _ = _loop_carrot((0., .9), .9, 1, 1.1, e1, e2)
        curvature, _ = _pursuit_pwm(point)
        self.assertAlmostEqual(curvature, 1/.9, places=6)
        # Clockwise mirror.
        point, mode, _ = _loop_carrot((0., -1.2), 1.2, -1, 1.1, e1, (0., -1.))
        self.assertAlmostEqual(_pursuit_pwm(point)[0], -1/1.2, places=6)

    def test_carrot_takes_the_tangent_when_the_line_path_passes_the_support(self):
        e1, e2 = (1., 0.), (0., 1.)
        # Approach line 1.8 m right; the support sits on the straight way to it.
        support = ((.55, -.9), .1+SIDE_BODY_EXTENT_M+.1)
        point, mode, _ = _loop_carrot((1.2, -.9), .9, 1, 1.1, e1, e2)
        self.assertEqual(mode, 'approach_line')
        self.assertLess(_segment_distance(support[0], point), support[1])
        point, mode, _ = _loop_carrot((1.2, -.9), .9, 1, 1.1, e1, e2, support=support)
        self.assertTrue(mode.startswith('tangent'))

    def test_pursuit_pwm_maps_curvature_and_clamps(self):
        self.assertEqual(_pursuit_pwm((1., 0.))[1], NEUTRAL)
        curvature, pwm = _pursuit_pwm((1., .2))
        self.assertEqual(pwm, NEUTRAL+round(ROUTE_PWM_PER_CURVATURE*curvature))
        self.assertEqual(_pursuit_pwm((.1, 1.))[1], SERVO_MAX)
        self.assertEqual(_pursuit_pwm((.1, -1.))[1], RIGHT_EXIT_MIN_PWM)

    def test_strip_clear(self):
        self.assertTrue(_strip_clear([(1., .5)], (1.2, 0.), None, 0.))
        self.assertFalse(_strip_clear([(1., .1)], (1.2, 0.), None, 0.))
        self.assertTrue(_strip_clear([(1., .1)], (1.2, 0.), (1., .1), .1))  # the support itself

    def test_axis_lines_find_boards_parallel_to_the_axis_not_supports(self):
        heading = math.radians(20)  # car yawed 20 deg left of the axis
        c, s = math.cos(-heading), math.sin(-heading)

        def car(along, lateral):  # axis frame -> car frame
            return (c*along-s*lateral, s*along+c*lateral)
        points = [car(.5+.1*k, 1.3) for k in range(20)]           # board 1.3 m left, 2 m long
        points += [car(2.+.02*k, -.8+.02*k) for k in range(6)]    # compact support
        lines = _axis_lines(points, heading)
        self.assertEqual(len(lines), 1)
        self.assertAlmostEqual(lines[0]['lateral_m'], 1.3, places=6)
        self.assertAlmostEqual(lines[0]['along_m'][0], .5, places=6)


class AxisLineRunTests(unittest.TestCase):
    def test_a_stray_return_at_the_same_offset_does_not_stretch_a_board(self):
        # Board ahead from 2.4 m on (lateral -.56); one return of a crossing
        # wall 2.4 m behind at the same offset (simulation: made the divider
        # look alongside and started the light lane early).
        points = [(2.4+.1*k, -.56) for k in range(25)]+[(-2.36, -.56), (-2.36, -.53)]
        lines = _axis_lines(points, 0.)
        self.assertEqual(len(lines), 1)
        self.assertAlmostEqual(lines[0]['along_m'][0], 2.4, places=6)

    def test_grazing_spacing_far_ahead_keeps_one_board(self):
        # A board .5 m aside seen at a grazing angle: bin spacing grows ~r^2/lateral.
        points = []
        for i in range(1, 30):
            a = math.radians(i)
            points.append((.5/math.tan(a), .5))
        points = [p for p in points if p[0] <= 6.]
        lines = _axis_lines(points, 0.)
        self.assertEqual(len(lines), 1)
        self.assertGreater(lines[0]['along_m'][1], 4.)


class RotationCompensationTests(unittest.TestCase):
    def test_rotated_observation_turns_the_point_opposite_to_the_yaw(self):
        observation = {'point_left_m': [2., 0.], 'bearing_left_rad': 0.}
        turned = rotated_observation(observation, math.radians(10))
        self.assertAlmostEqual(turned['bearing_left_rad'], math.radians(-10))
        self.assertAlmostEqual(math.hypot(*turned['point_left_m']), 2.)
        self.assertIs(rotated_observation(observation, None), observation)

    def test_tracker_keeps_identity_through_fast_yaw_only_with_the_heading(self):
        def scan(seq, yaw):
            # A support 2.4 m away at bearing 40 deg in the world, car yawing.
            angle = math.radians(40)-yaw
            ranges = arc_ranges((2.4*math.cos(angle), 2.4*math.sin(angle)), .1, background=5.)
            return {'frame_id': 'lidar_origin_coarse_body_heading', 'seq': seq, 'at_ms': 100*seq,
                    'received_at': seq/10, 'ranges': ranges}
        for compensated in (True, False):
            tracker = CompactTargetTracker(acquisition_bearing_rad=(-math.pi/2, math.pi/2), tolerant=True,
                                           select_associated=True, maintenance_bearing_rad=(-math.pi, math.pi))
            ids = set()
            for seq in range(1, 9):
                yaw = math.radians(8*seq)  # 80 deg/s
                target = tracker.update(scan(seq, yaw), seq/10, acquire=True,
                                        heading_rad=yaw if compensated else None)
                if target is not None:
                    ids.add(target['track_id'])
            with self.subTest(compensated=compensated):
                if compensated:
                    self.assertEqual(ids, {1})
                else:
                    self.assertGreater(len(ids), 1)  # the .30 m gate breaks at this yaw rate

    def test_tracker_without_heading_is_unchanged(self):
        tracker = CompactTargetTracker()
        self.assertIsNone(tracker.update({'frame_id': 'x'}, 0.))
        with self.assertRaises(ValueError):
            tracker.update({'frame_id': 'x'}, 0., heading_rad='a')


class StageWiringTests(unittest.TestCase):
    def test_open_stages_are_post_pass_and_task_left_bypass(self):
        for stage in OPEN_STAGES:
            self.assertIn(stage, POST_PASS_STAGES)
        self.assertIn('open_transfer', TASK_LEFT_BYPASS_STAGES)
        self.assertIn('second_loop', TASK_LEFT_BYPASS_STAGES)
        self.assertNotIn('light_entry', TASK_LEFT_BYPASS_STAGES)

    def test_front_range_is_measured_along_the_compass_axis_on_the_open_course(self):
        motion = ManeuverSequence(0., continue_route=True)
        motion.entry_kind = 'lane'
        yaw = math.radians(20)
        motion.compass.update({'at_ms': 1000, 'wall_candidates': box_walls(yaw)})
        # End board 3 m ahead along the axis; the car's own straight ray meets
        # a side board 1 m away (angled entry, round-63 early cut).
        ranges = []
        for i in range(360):
            a = -math.radians(i)+yaw  # ray angle in the axis frame
            dx, dy = math.cos(a), math.sin(a)
            best = 12.
            if dx > 1e-9:
                best = min(best, 3./dx)
            if dy < -1e-9:
                best = min(best, 1./-dy)
            if dy > 1e-9:
                best = min(best, 1./dy)
            ranges.append(best)
        scan = {'at_ms': 1000, 'ranges': ranges}
        self.assertAlmostEqual(motion._lane_front_range(scan), 3., places=2)
        motion.entry_kind = 'junction'
        self.assertLess(motion._lane_front_range(scan), 3.)  # old path: the car's own ray


@unittest.skipUnless(TOOL.exists(), 'tools/scan-geometry not built (cargo build --release)')
class CourseSimulationTests(unittest.TestCase):
    """Closed loop through the real service on rules-shaped courses."""
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location('course_sim_for_tests', SIM)
        cls.sim = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.sim)

    def run_layout(self, name, seed=0):
        summary, _, _ = self.sim.run(self.sim.Course(**self.sim.LAYOUTS[name]), seed=seed)
        return summary

    def test_document_example_course_completes_without_contact(self):
        for seed in (0, 1):
            summary = self.run_layout('doc8x4', seed)
            with self.subTest(seed=seed):
                self.assertTrue(summary['success'], summary)
                self.assertEqual(summary['end_reason'], 'route_finish_zone_stop')
                self.assertEqual(summary['contacts'], [])
                self.assertGreater(summary['min_board_gap_m'], .1)
                self.assertGreater(summary['min_cone_gap_m'], .1)
                stages = [s[1] for s in summary['stages']]
                for stage in ('start_lane', 'first_target', 'second_loop', 'light_entry', 'light_lane'):
                    self.assertIn(stage, stages)

    def test_re_laid_practice_and_other_sizes_complete(self):
        for name in ('user8', 'wide8x6', 'mid64', 'short5'):
            summary = self.run_layout(name)
            with self.subTest(layout=name):
                self.assertTrue(summary['success'], summary)
                self.assertEqual(summary['contacts'], [])


if __name__ == '__main__':
    unittest.main()
