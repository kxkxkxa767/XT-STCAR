"""Round 66 car run (2026-10-17): 0.4 s after the start a 5-bin front gap
(returns lost at the edge between the divider's open end and the end wall
~7 m away) stopped the start lane with turn_perception_unavailable. The
bounded-gap allowance the route already has after the handover now also
covers the start lane while this scan's native geometry is current (its lane,
or side boards measuring the compass). Closed loop with the simulator's
optional depth-step dropout model; no physical claim."""
import copy
import importlib.util
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).parents[1]))
import maneuver_sequence as ms  # noqa: E402
from maneuver_sequence import ManeuverSequence, WallCompass  # noqa: E402

ROOT = Path(__file__).resolve().parents[3]
SIM = ROOT/'scripts'/'course-sim.py'
TOOL = ROOT/'tools'/'scan-geometry'/'target'/'release'/'xt-stcar-scan-geometry'


def wall(heading, span=1.5, points=60, fit=.01):
    return {'heading_left_rad': heading, 'rho_left_m': .45,
            'support_start_left_m': {'x_m': 0., 'y_m': .45}, 'support_end_left_m': {'x_m': span, 'y_m': .45},
            'support_span_m': span, 'points': points, 'fit_error_m': fit, 'candidate_only': True}


class CompassMeasurable(unittest.TestCase):
    def test_measurable_matches_update_and_changes_nothing(self):
        compass = WallCompass()
        compass.update({'at_ms': 1000, 'wall_candidates': [wall(0.), wall(math.pi)]})
        for walls, expected in (([wall(.02), wall(math.pi+.01)], True),
                                ([wall(.02, span=.6)], False),           # .6 m support < 1 m
                                ([wall(.9)], False),                      # outside the 30 deg gate
                                ([wall(.02, fit=.08), wall(.01, fit=.08)], False)):  # poor fits
            scan = {'at_ms': 1100, 'wall_candidates': walls}
            before = copy.deepcopy(compass.__dict__)
            self.assertIs(compass.measurable(scan), expected)
            self.assertEqual(compass.__dict__, before)
            probe = copy.deepcopy(compass)
            probe.update(scan)
            self.assertIs(probe.measured_at_s == 1.1, expected)

    def test_uninitialised_compass_is_not_evidence(self):
        self.assertFalse(WallCompass().measurable({'at_ms': 0, 'wall_candidates': [wall(0.), wall(math.pi)]}))


class StartLaneEvidence(unittest.TestCase):
    def test_only_a_driving_start_lane_with_current_geometry_qualifies(self):
        motion = ManeuverSequence(0., continue_route=True, max_drive_s=40.)
        motion.compass.update({'at_ms': 1000, 'wall_candidates': [wall(0.), wall(math.pi)]})
        scan = {'at_ms': 1100, 'corridor_candidates': [], 'wall_candidates': [wall(0.), wall(math.pi)]}
        self.assertFalse(motion.start_lane_quality_geometry(scan))  # not driving the start lane yet
        motion.entry_kind, motion.route_stage, motion.phase, motion.drive_since = 'lane', 'start_lane', 'drive', 0.
        self.assertTrue(motion.start_lane_quality_geometry(scan))
        self.assertFalse(motion.start_lane_quality_geometry({**scan, 'wall_candidates': []}))
        motion.route_stage = 'first_target'
        self.assertFalse(motion.start_lane_quality_geometry(scan))


@unittest.skipUnless(TOOL.exists(), 'tools/scan-geometry not built')
class StartLaneDropoutClosedLoop(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location('course_sim_r66', SIM)
        cls.sim = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.sim)

    def run_user8(self):
        summary, _, _ = self.sim.run(self.sim.Course(**self.sim.LAYOUTS['user8']), vehicle='measured',
                                     seed=0, edge_dropout=.1)
        return summary

    def test_start_lane_continues_through_bounded_dropouts(self):
        summary = self.run_user8()
        stages = [s[1] for s in summary['stages']]
        self.assertIn('first_target', stages)
        self.assertNotEqual(summary['end_reason'], 'turn_perception_unavailable')

    def test_without_the_allowance_the_same_dropouts_stop_the_start_lane(self):
        with patch.object(ms.ManeuverSequence, 'start_lane_quality_geometry', lambda self, scan: False):
            summary = self.run_user8()
        self.assertEqual(summary['end_reason'], 'turn_perception_unavailable')
        self.assertNotIn('first_target', [s[1] for s in summary['stages']])


if __name__ == '__main__':
    unittest.main()
