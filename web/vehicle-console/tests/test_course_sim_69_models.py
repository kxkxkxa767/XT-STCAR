"""Optional simulation hypotheses; never change the default or the real control."""
import importlib.util
import math
from pathlib import Path
import random
import unittest

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location('course_sim_69_models', ROOT/'scripts/course-sim.py')
cs = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cs)


class GrazingModelTests(unittest.TestCase):
    def course(self):
        c = cs.Course(**cs.LAYOUTS['user8'])
        c.segments = [((0., 0.), (10., 0.))]
        c.cones = []
        return c

    def test_angle_is_to_tangent_not_normal_and_cones_are_excluded(self):
        c = self.course()
        r, a = c.cast(1., 1., math.radians(-10), surface=True)
        self.assertAlmostEqual(a, 10.)
        self.assertAlmostEqual(r, 1/math.sin(math.radians(10)))
        self.assertAlmostEqual(c.cast(1., 1., -math.pi/2, surface=True)[1], 90.)
        self.assertEqual(c.cast(1., 1., 0., surface=True), (None, None))
        c.cones = [(2., 1., .1)]
        self.assertEqual(c.cast(1., 1., 0., surface=True)[1], None)
        self.assertAlmostEqual(c.cast(1., 1., 0., surface=True)[0], .9)

    def test_nearest_occluder_and_wall_segment_direction_do_not_change_angle(self):
        c = self.course()
        expected = c.cast(1., 1., math.radians(-10), surface=True)
        c.segments[0] = tuple(reversed(c.segments[0]))
        actual = c.cast(1., 1., math.radians(-10), surface=True)
        self.assertAlmostEqual(actual[0], expected[0]); self.assertAlmostEqual(actual[1], expected[1])
        c.segments += [((2., -1.), (2., 2.))]
        self.assertAlmostEqual(c.cast(1., 1., math.radians(-10), surface=True)[1], 80.)

    def test_independent_randomness_only_removes_eligible_existing_returns(self):
        noise = random.Random(6); untouched = random.Random(6)
        ranges = [noise.random() for _ in range(5)]
        source = [untouched.random() for _ in range(5)]
        cs.grazing_dropouts(ranges, [3., 12., None, 10., 0.], angle_deg=10., probability=1., rng=random.Random(9))
        self.assertEqual(ranges, [None, source[1], source[2], source[3], None])
        self.assertEqual(noise.getstate(), untouched.getstate())
        ranges = list(source)
        cs.grazing_dropouts(ranges, [3.]*5, angle_deg=10., probability=0., rng=random.Random(9))
        self.assertEqual(ranges, source)


class NearCenterGainTests(unittest.TestCase):
    def settle(self, servo, **kwargs):
        car = cs.MeasuredVehicle(0., 0., 0., drift_per_mps=0., **kwargs)
        for k in range(150):
            car.step(k*.02, .02, 1500, servo)
        return car

    def test_optional_gain_tapers_to_measured_large_turns(self):
        for servo in (1470, 1530):
            regular = self.settle(servo, play_pwm=0.)
            strong = self.settle(servo, play_pwm=0., near_center_gain=3.)
            self.assertGreater(abs(strong.kappa), abs(regular.kappa)*1.5)
            self.assertEqual(math.copysign(1., strong.kappa), math.copysign(1., servo-1500))
        for servo in (1200, 1720):
            self.assertEqual(self.settle(servo).kappa, self.settle(servo, near_center_gain=3.).kappa)

    def test_warped_response_is_monotonic_on_both_sides_and_meets_endpoints(self):
        for gain in (.5, 3., 8.):
            car = cs.MeasuredVehicle(0., 0., 0., play_pwm=0., near_center_gain=gain,
                                     delay=0., steer_tau=.02, drift_per_mps=0.)
            values = []
            for servo in range(1300, 1701):
                car.step(0., .02, 1500, servo)
                values.append(car.kappa)
            self.assertTrue(all(a <= b for a, b in zip(values, values[1:])))
            for servo in (1400, 1500, 1600):
                self.assertAlmostEqual(values[servo-1300], cs._interp(cs.MEASURED_STEER_TABLE, servo))

    def test_gain_does_not_magic_away_real_play(self):
        for servo in (1470, 1530):
            car = self.settle(servo, play_pwm=70., near_center_gain=3.)
            self.assertEqual(car.wheel_pwm, 1500.)
            self.assertEqual(car.kappa, 0.)

    def test_default_matches_explicit_disabled_and_rejects_invalid_parameters(self):
        a, b = self.settle(1585), self.settle(1585, near_center_gain=1.)
        self.assertEqual((a.x,a.y,a.psi,a.kappa,a.wheel_pwm), (b.x,b.y,b.psi,b.kappa,b.wheel_pwm))
        for kwargs in ({'near_center_gain':0.}, {'near_center_gain':math.nan}, {'near_center_width_pwm':0.}):
            with self.assertRaises(ValueError):
                cs.MeasuredVehicle(0.,0.,0.,**kwargs)
