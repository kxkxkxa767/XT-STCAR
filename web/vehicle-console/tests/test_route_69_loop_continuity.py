"""Round 69: preserve a feasible loop when replanning reverses its task direction.

The fixture is a captured synthetic-course frame and its preceding planner
state, not physical vehicle evidence. The actual-frame test uses real support
checks, ranges, probes, planning and pursuit; only captured compass inputs are
injected. Other tests isolate the decision contracts around replanning.
"""
import copy
import json
import math
import sys
import unittest
from pathlib import Path
from unittest.mock import patch
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import maneuver_sequence as ms
FRAME = json.loads((HERE / 'fixtures/route-69-loop-plan-jump.json').read_text())
FIXTURE = {'previous': {'plan': FRAME['previous_slot']['plan']}, 'current': FRAME['old_result']}

class ReplanIntent(unittest.TestCase):

    def setUp(self):
        self.m = ms.ManeuverSequence(0.0, continue_route=True)
        self.row = copy.deepcopy(FIXTURE['current'])
        self.old = copy.deepcopy(FIXTURE['previous']['plan'])
        self.new = copy.deepcopy(self.row['plan'])
        self.m.compass.current = lambda scan: self.row['compass_heading_rad']
        self.m.compass.rate = 0.0
        self.m._full_target_points = lambda *args: [self.row['support_center_left_m']]
        self.radius = self.row['plan']['inner_bound_m'] - 0.14 - 0.15
        self.free = {math.radians(float(k)): v for k, v in self.row['free_m'].items()}
        self.slot = {'plan': copy.deepcopy(self.old), 'free': dict(self.free), 'key': (2, 0.09), 'radius': self.radius, 'approaching': False}
        self.m._open_slots[-1] = self.slot
        self.scan = {'seq': 157, 'at_ms': 15600, 'ranges': [5.0] * 360}
        self.target = {'support_bins': [90, 91], 'diameter_m': self.radius * 2}

    def call(self, identity=2):
        with patch.object(ms, '_support_center', return_value=(self.row['support_center_left_m'], self.radius)), \
             patch.object(ms, '_probe_free', side_effect=[self.free[t] + 1e-08 for t in ms.OPEN_PROBE_DIRECTIONS_RAD]), \
             patch.object(ms, '_plan_loop', return_value=copy.deepcopy(self.new)) as planner:
            result = self.m._open_loop_geometry(self.scan, self.target, identity, -1, math.pi)
            return (result, planner.call_count)

    def test_feasible_clockwise_circle_does_not_jump_to_counterclockwise_carrot(self):
        out, calls = self.call()
        self.assertEqual(calls, 1)
        self.assertEqual(out['plan'], self.old)
        self.assertLess(out['pursuit_curvature_per_m'], 0)

    def test_captured_failure_frame_uses_real_probe_planner_and_carrot(self):
        x = copy.deepcopy(FRAME)
        slot = x['previous_slot']
        slot['key'] = tuple(slot['key'])
        slot['free'] = {float(k): v for k, v in slot['free'].items()}
        motion = ms.ManeuverSequence(0.0, continue_route=True)
        motion._open_slots[x['side']] = copy.deepcopy(slot)
        motion.compass.current = lambda scan: x['compass_heading']
        motion.compass.rate = x['compass_rate']
        out = motion._open_loop_geometry(x['scan'], x['target'], x['identity'], x['side'], x['axis'])
        self.assertIsNotNone(out)
        self.assertEqual(out['plan'], slot['plan'])
        self.assertLess(out['pursuit_curvature_per_m'], 0.0)
        self.assertLess(out['target_pwm'], 1500)

    def test_selection_metrics_and_current_rejection_have_distinct_provenance(self):
        self.slot['plan_source_seq'] = 140
        out, calls = self.call()
        self.assertEqual(out['plan_selection_source_seq'], 140)
        self.assertEqual(out['plan_metrics_basis'], 'selection_time_optimizer_score_and_probe_margins')
        update = out['plan_update']
        self.assertEqual(update['checked_source_seq'], 157)
        self.assertTrue(update['rejected_task_direction_reversal'])
        self.assertEqual(update['constraint_basis'], 'existing_low_yaw_probe_history_running_minimum')
        self.assertTrue(all((x > 0 for x in update['prior_plan_margins_m'].values())))
        self.assertGreater(update['candidate_curvature_per_m'], 0.0)
        self.assertLess(update['prior_curvature_per_m'], 0.0)
        self.assertFalse(update['swept_path_certified'])

    def test_same_direction_replan_still_occurs(self):
        self.new['offset_axis_m'] = -0.2
        out, calls = self.call()
        self.assertEqual(calls, 1)
        self.assertEqual(out['plan'], self.new)

    def test_approach_can_change_plan(self):
        self.slot['approaching'] = True
        out, calls = self.call()
        self.assertEqual(out['plan'], self.new)

    def test_changed_identity_cannot_reuse_circle(self):
        out, calls = self.call(identity=3)
        self.assertEqual(out['plan'], self.new)
        self.assertEqual(self.slot['key'][0], 3)

    def test_new_blocking_constraint_forces_replan_even_if_opposite(self):
        self.free = {t: 0.7 for t in self.free}
        out, calls = self.call()
        self.assertEqual(out['plan'], self.new)

    def test_old_circle_must_itself_steer_the_task_direction(self):
        self.old = self.new.copy()
        self.slot['plan'] = self.old.copy()
        self.new['offset_axis_m'] = 0.3
        out, calls = self.call()
        self.assertEqual(out['plan'], self.new)

    def test_identical_no_change_geometry_does_not_replan(self):
        self.slot['key'] = (2, round(self.radius, 2))
        out, calls = self.call()
        self.assertEqual(calls, 0)
        self.assertEqual(out['plan'], self.old)
if __name__ == '__main__':
    unittest.main(verbosity=2)
