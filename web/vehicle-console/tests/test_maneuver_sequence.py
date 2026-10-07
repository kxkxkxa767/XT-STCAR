"""Pure control-sequence checks; mock observations are not target detection QA."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).parents[1]))
from maneuver_sequence import ManeuverSequence


def corridor(heading):
    return {'heading_left_rad': math.radians(heading), 'center_offset_left_m': 0.,
            'width_m': 1.2, 'left_wall_points': 40, 'right_wall_points': 45,
            'support_span_m': .8, 'fit_error_m': .02, 'origin_between_walls': True,
            'candidate_only': True, 'turn_path_certified': False}


def scan(seq, now):
    return {'seq': seq, 'at_ms': seq*100, 'received_at': now,
            'frame_id': 'lidar_origin_coarse_body_heading', 'ranges': [3.2]*360,
            'corridor_candidates': [corridor(60), corridor(-120)]}


def target(value, bearing=90., distance=1., identity=1, confirmed=True):
    theta = math.radians(bearing)
    return {'kind': 'lidar_compact_object', 'semantic_class': 'unknown',
            'candidate_only': True, 'confirmed': confirmed, 'track_id': identity,
            'source_seq': value['seq'], 'source_at_ms': value['at_ms'],
            'range_m': distance, 'bearing_left_rad': theta,
            'point_left_m': [distance*math.cos(theta), distance*math.sin(theta)]}


class Observations:
    def __init__(self):
        self.factory = lambda value: None

    def update(self, value, now):
        return self.factory(value)


class Harness:
    def __init__(self, max_drive=10., initial=1700):
        self.motion = ManeuverSequence(0., max_drive_s=max_drive, initial_presteer_pwm=initial)
        self.observer = self.motion.target_tracker = Observations()
        self.seq, self.now, self.servo, self.motor = 0, 0., 1500, 1500
        self.latest_scan = self.latest_control = None

    def step(self, **kwargs):
        self.seq += 1
        self.now = self.seq/10
        self.latest_scan = scan(self.seq, self.now)
        self.latest_control = {'seq': self.seq, 'tick': self.seq*100,
                               'armed': True, 'motor': self.motor, 'servo': self.servo}
        result = self.motion.update(self.latest_scan, 0., self.now, self.latest_control, **kwargs)
        self.servo, self.motor = result['servo'], result['motor']
        return result

    def drive(self):
        for _ in range(49):
            result = self.step()
            if result['phase'] == 'drive':
                return result
        raise AssertionError('presteer never matured')

    def orbit(self, **fields):
        self.drive()
        self.observer.factory = lambda value: target(value, **fields)
        result = self.step()
        assert result['turn_stage'] == 'orbit_entry'
        return result


class ManeuverSequenceTests(unittest.TestCase):
    def test_presteer_remains_neutral_until_real_software_adoption_allowance(self):
        h = Harness()
        steps = []
        for _ in range(49):
            steps.append(h.step())
            if steps[-1]['phase'] == 'drive':
                break
        self.assertTrue(all(v['motor'] == 1500 for v in steps[:-1]))
        self.assertEqual(steps[-1]['motor'], 1560)
        self.assertEqual(steps[-1]['servo'], 1700)
        self.assertGreaterEqual(steps[-1]['steering_settle_feedback_ticks'], 3)
        self.assertGreaterEqual(steps[-1]['steering_settle_elapsed_s']+1e-9, 1.2)
        self.assertTrue(all(abs(a['servo']-b['servo']) <= 10 for a, b in zip(steps, steps[1:])))

    def test_side_target_can_handover_while_wall_is_not_forward(self):
        h = Harness()
        result = h.orbit(bearing=105.)
        self.assertEqual(result['turn_stage'], 'orbit_entry')
        self.assertTrue(result['handover_observed'])
        self.assertAlmostEqual(result['corridor']['heading_left_rad'], math.radians(60))
        self.assertFalse(result['completed'])
        self.assertFalse(result['competition_supported'])
        self.assertIsNone(result['passed_cones'])
        self.assertFalse(result['object_semantic_verified'])
        self.assertFalse(result['test_sequence_finished'])

    def test_unconfirmed_or_stale_source_target_does_not_take_over(self):
        for incorrect in ('unconfirmed', 'wrong_seq', 'wrong_time', 'point_inconsistent', 'semantic_claim'):
            with self.subTest(incorrect=incorrect):
                h = Harness()
                h.drive()
                def observation(value):
                    found = target(value)
                    if incorrect == 'unconfirmed': found['confirmed'] = False
                    if incorrect == 'wrong_seq': found['source_seq'] -= 1
                    if incorrect == 'wrong_time': found['source_at_ms'] -= 1
                    if incorrect == 'point_inconsistent': found['point_left_m'][0] += .2
                    if incorrect == 'semantic_claim': found['semantic_class'] = 'blue_cone'
                    return found
                h.observer.factory = observation
                self.assertFalse(h.step()['handover_observed'])

    def test_relative_feedback_changes_target_and_keeps_entry_range(self):
        h = Harness()
        first = h.orbit(distance=.9)
        h.observer.factory = lambda value: target(value, bearing=70., distance=.8)
        second = h.step()
        h.observer.factory = lambda value: target(value, bearing=110., distance=1.05)
        third = h.step()
        self.assertLess(second['steering_target'], first['steering_target'])
        self.assertGreater(third['steering_target'], second['steering_target'])
        self.assertEqual(third['orbit_reference_range_m'], .9)
        self.assertTrue(1500 <= second['servo'] <= 1720)
        self.assertLessEqual(abs(third['servo']-second['servo']), 10)

    def test_target_loss_and_identity_jump_latch_coast_without_restart(self):
        for lost in ('none', 'unconfirmed', 'identity'):
            with self.subTest(lost=lost):
                h = Harness()
                h.orbit()
                h.observer.factory = (lambda value: None) if lost == 'none' else (
                    lambda value: target(value, confirmed=lost != 'unconfirmed', identity=2))
                result = h.step()
                self.assertEqual(result['phase'], 'coast')
                self.assertEqual(result['motor'], 1500)
                first_coast_at = h.motion.coast_since
                h.observer.factory = lambda value: target(value)
                for _ in range(7):
                    result = h.step()
                    self.assertEqual(result['motor'], 1500)
                    self.assertEqual(result['phase'], 'coast')
                self.assertEqual(h.motion.coast_since, first_coast_at)

    def test_orbit_entry_bound_and_five_second_coast_do_not_report_completion(self):
        h = Harness()
        h.orbit()
        for _ in range(30):
            result = h.step()
        self.assertEqual(result['phase'], 'coast')
        self.assertEqual(result['reason'], 'first_relative_object_entry_trial_timeout')
        for _ in range(50):
            result = h.step()
        self.assertEqual(result['phase'], 'locked')
        self.assertEqual(result['motor'], 1500)
        self.assertEqual(result['servo'], 1500)
        self.assertFalse(result['completed'])
        self.assertEqual(h.step()['phase'], 'locked')

    def test_drive_budget_is_shared_with_left_turn(self):
        h = Harness(max_drive=.4)
        h.drive()
        h.step()  # actual left-turn drive uses this part of the budget
        h.observer.factory = lambda value: target(value)
        result = h.step()
        original_since = h.motion.drive_since
        self.assertTrue(result['handover_observed'])
        for _ in range(3): result = h.step()
        self.assertEqual(result['phase'], 'coast')
        self.assertEqual(result['reason'], 'maneuver_cumulative_drive_timeout')
        self.assertEqual(h.motion.drive_since, original_since)

    def test_duplicate_scan_neither_moves_servo_nor_replays_observation(self):
        h = Harness()
        h.orbit()
        h.observer.factory = lambda value: target(value, bearing=110.)
        result = h.motion.update(h.latest_scan, .1, h.now+.1, h.latest_control)
        self.assertEqual(result['servo'], h.servo)
        self.assertEqual(result['phase'], 'drive')
        stale = h.motion.update(h.latest_scan, .3, h.now+.3, h.latest_control)
        self.assertEqual(stale['phase'], 'locked')

    def test_changed_duplicate_reordered_or_invalid_actual_inputs_lock(self):
        for invalid in ('changed_ranges', 'reordered_tick', 'unarmed', 'missing_receive', 'invalid_range'):
            with self.subTest(invalid=invalid):
                h = Harness()
                h.orbit()
                value, feedback, now = scan(h.seq+1, h.now+.1), dict(h.latest_control), h.now+.1
                feedback['seq'] += 1
                feedback['tick'] += 100
                if invalid == 'changed_ranges':
                    value = dict(h.latest_scan, ranges=[3.1]*360)
                if invalid == 'reordered_tick': feedback['tick'] -= 200
                if invalid == 'unarmed': feedback['armed'] = False
                if invalid == 'missing_receive': value.pop('received_at')
                if invalid == 'invalid_range': value['ranges'][0] = float('nan')
                result = h.motion.update(value, .1, now, feedback)
                self.assertEqual(result['phase'], 'locked')
                self.assertEqual(result['motor'], 1500)

    def test_safe_failure_immediately_centers_and_locks(self):
        h = Harness()
        h.orbit()
        result = h.step(safe=False)
        self.assertEqual((result['phase'], result['motor'], result['servo']), ('locked', 1500, 1500))

    def test_wall_ambiguity_hold_never_renews_geometry_or_matures_alignment(self):
        h = Harness()
        h.drive()
        old_geometry = (h.motion.last_geometry_receive, h.motion.last_geometry_publication,
                        h.motion.last_geometry_seq)
        old_servo = h.servo
        def ambiguous(*args):
            raise ValueError('left_turn_outer_wall_ambiguous')
        h.motion._measurement = ambiguous
        for _ in range(3):
            result = h.step()
            self.assertEqual(result['reason'], 'left_turn_wall_ambiguity_hold')
            self.assertEqual(result['servo'], old_servo)
            self.assertEqual(result['motor'], 1560)
            self.assertEqual(result['alignment_confirmations'], 0)
            self.assertFalse(result['handover_observed'])
            self.assertEqual((h.motion.last_geometry_receive, h.motion.last_geometry_publication,
                              h.motion.last_geometry_seq), old_geometry)
        result = h.step()
        self.assertEqual(result['phase'], 'locked')
        self.assertEqual(result['reason'], 'left_turn_outer_wall_ambiguous')

    def test_actual_confirmed_object_takes_over_without_reclassifying_ambiguous_wall(self):
        h = Harness()
        h.drive()
        def ambiguous(*args):
            raise ValueError('left_turn_outer_wall_ambiguous')
        h.motion._measurement = ambiguous
        h.step()
        h.observer.factory = lambda value: target(value, bearing=100.)
        result = h.step()
        self.assertEqual(result['turn_stage'], 'orbit_entry')
        self.assertEqual(result['motor'], 1560)
        self.assertFalse(result['wall_ambiguity_hold'])
        self.assertEqual(result['compact_target']['semantic_class'], 'unknown')

    def test_stronger_left_wall_response_is_dynamic_and_bounded(self):
        h = Harness()
        h.drive()
        h.motion.last_error = None
        strong = h.motion._target(corridor(60), .1)
        h.motion.last_error = None
        lower = h.motion._target(corridor(30), .1)
        h.motion.last_error = None
        centered = h.motion._target(corridor(0), .1)
        h.motion.last_error = None
        saturated = h.motion._target(corridor(149), .1)
        self.assertEqual(strong, 1688)
        self.assertTrue(centered < lower < strong <= saturated)
        self.assertEqual(centered, 1500)
        self.assertEqual(saturated, 1720)

    def test_trend_can_release_only_twenty_percent_of_current_error(self):
        h = Harness()
        h.drive()
        h.motion.last_error = math.radians(69)
        target_pwm = h.motion._target(corridor(60), .1)
        self.assertEqual(target_pwm, 1651)
        h.motion.last_error = math.radians(5)
        smaller = h.motion._target(corridor(0), .1)
        self.assertEqual(smaller, 1500)


if __name__ == '__main__':
    unittest.main()
