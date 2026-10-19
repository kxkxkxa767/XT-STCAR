"""Round 79: track steering calibration and second-role recovery moved with the
car. Synthetic scans; no physical claims."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).parents[1]))
import maneuver_sequence as ms
import steer_calibration as sc
from compact_target import CompactTargetTracker
import test_maneuver_route as route
from test_compact_target import compact_scan
from test_compact_target_roles import add_object
from test_maneuver_sequence import corridor
import test_route_identity_recovery as recovery


class TrackCalibration(unittest.TestCase):
    """Rounds 70-78 fit: play 40, ~155 PWM per 1/m both sides (turn79 record)."""
    def test_nominal_curvature_is_delivered_linearly_with_the_measured_play(self):
        self.assertEqual(sc.STEER_PLAY_PWM, 40)
        self.assertEqual(ms.LANE_WALL_PLAY_PWM, sc.STEER_PLAY_PWM)
        for nominal in (1340, 1400, 1460, 1540, 1600, 1660):
            kappa = (nominal-1500)/160
            self.assertAlmostEqual(sc.wheel_for_curvature(kappa), 1500+155*kappa, delta=1.)
            out = sc.SteerOutput()
            play = 40 if nominal > 1500 else -40
            self.assertAlmostEqual(out.physical(nominal), 1500+155*kappa+play, delta=1.)
            # Back to centre crosses the play the other way; the wheels end centred.
            self.assertEqual(out.physical(1500), 1500-play)
            self.assertAlmostEqual(out.wheel, 1500., delta=.5)

    def test_planning_limits(self):
        self.assertEqual(sc.NOMINAL_RIGHT_LIMIT_PWM, 1308)
        self.assertEqual(ms.RIGHT_EXIT_MIN_PWM, 1308)
        out = sc.SteerOutput()
        self.assertEqual(out.physical(1308), 1274)  # wheels at -1.2 /m
        out = sc.SteerOutput()
        self.assertEqual(out.physical(1720), 1720)  # the physical end; wheels ~1.16 /m
        self.assertAlmostEqual(sc.curvature_for_wheel(out.wheel), 1.16, delta=.01)


class OnlySupportBins(unittest.TestCase):
    def two(self, seq):
        scan = compact_scan(seq, seq*100, seq/10, bearing=-40.)
        return add_object(scan, bearing=-100.)

    def test_restriction_removes_the_other_support_as_an_ambiguity(self):
        plain = CompactTargetTracker(acquisition_bearing_rad=(-math.pi, math.pi/2), tolerant=True)
        self.assertIsNone(plain.update(self.two(1), .11))
        self.assertEqual(plain.reason, 'compact_target_ambiguous')
        tracker = CompactTargetTracker(acquisition_bearing_rad=(-math.pi, math.pi/2), tolerant=True)
        bins = [(100+k) % 360 for k in range(-4, 5)]
        result = None
        for seq in (1, 2, 3, 4):
            result = tracker.update(self.two(seq), seq/10+.01, only_support_bins=bins)
        self.assertTrue(result['confirmed'])
        self.assertAlmostEqual(math.degrees(result['bearing_left_rad']), -100., delta=1.)
        for bad in ([], [360], ['1'], 5):
            with self.assertRaises(ValueError):
                tracker.update(self.two(5), .51, only_support_bins=bad)


def car_frame(world, pose):
    """World point in the car frame at pose (x, y, heading left +)."""
    x, y, psi = pose
    dx, dy = world[0]-x, world[1]-y
    c, s = math.cos(-psi), math.sin(-psi)
    return c*dx-s*dy, s*dx+c*dy


def object_scan(seq, points):
    """Scan with ~.15 m supports at car-frame points (3 m background)."""
    scan = None
    for x, y in points:
        distance, bearing = math.hypot(x, y), math.degrees(math.atan2(y, x))
        size = max(5, min(15, round(.15/(distance*math.pi/180))+1))
        if scan is None:
            scan = compact_scan(seq, seq*100, seq/10, bearing=bearing, distance=distance, size=size)
        else:
            add_object(scan, bearing=bearing, distance=distance, size=size)
    c = corridor(0.)
    c['center_offset_left_m'], c['width_m'] = 0., 2.4
    scan['corridor_candidates'] = [c]
    return scan


class SecondRecoveryMovedWithTheCar(unittest.TestCase):
    """Round 78: the car rolled on in the pause; the second support re-seen
    ~1.4 m from where the rotation-only prior put it."""
    FIRST, SECOND = (0., 1.), (.7071, -.7071)  # world = car frame at the anchor
    END = (1.4, -.3, math.radians(-30.))

    def run_pause(self, *, landmark):
        helper = recovery.RouteIdentityRecoveryTests('test_ambiguous_recovery_cannot_confirm_or_rebind')
        helper.setUp()
        motion, start, _ = helper.enter_second_wait()
        motion.entry_kind = 'lane'  # open-course association (compass)
        headings = {}
        motion.compass.current = lambda scan: headings.get(scan.get('seq'))
        anchor = motion._second_role_anchor
        self.assertIn('first_landmark_left_m', anchor)
        anchor['compass_heading_rad'] = 0.
        if not landmark:
            del anchor['first_landmark_left_m']
        steps = 14
        result = None
        for k in range(1, steps+12):
            seq = start+k
            f = min(1., k/steps)
            pose = (self.END[0]*f, self.END[1]*f, self.END[2]*f)
            headings[seq] = pose[2]
            points = [car_frame(self.FIRST, pose)]
            if k > steps:  # at rest; the second support is seen again
                points.append(car_frame(self.SECOND, pose))
            scan = object_scan(seq, points)
            result = motion.update(scan, 0., seq/10, route.feedback(seq, motion.servo, 1500, neutral_acked=True),
                                   quality_clear=True, quality_resume_ready=True)
            if motion.second_rebind_evidence is not None:
                break
        return motion, result

    def test_rotation_only_prior_misses_by_more_than_its_gate(self):
        x, y = car_frame(self.SECOND, self.END)
        c, s = math.cos(-self.END[2]), math.sin(-self.END[2])
        rotated = (c*self.SECOND[0]-s*self.SECOND[1], s*self.SECOND[0]+c*self.SECOND[1])
        self.assertGreater(math.dist((x, y), rotated), .3+1.5*ms.SECOND_RECOVERY_MAX_SLACK_S)
        motion, result = self.run_pause(landmark=False)
        self.assertIsNone(motion.second_rebind_evidence)
        self.assertEqual(result['second_acquisition_reason'], 'second_role_recovery_current_support_unproven')

    def test_first_support_landmark_recovers_it(self):
        motion, result = self.run_pause(landmark=True)
        self.assertIsNotNone(motion.second_rebind_evidence)
        self.assertEqual(motion.second_recovery_prior['basis'], 'first_support_landmark')
        x, y = car_frame(self.SECOND, self.END)
        self.assertLess(math.dist(motion.second_recovery_prior['point_left_m'], (x, y)), .05)
        self.assertLess(math.dist(motion.second_target['point_left_m'], (x, y)), .05)
        self.assertIsNone(result['second_pass_evidence'])


if __name__ == '__main__':
    unittest.main()
