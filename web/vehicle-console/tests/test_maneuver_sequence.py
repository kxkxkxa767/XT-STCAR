"""Pure control-sequence checks; mock observations are not target detection QA."""
import json
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).parents[1]))
from maneuver_sequence import (ManeuverSequence, PRESTEER_MAX_S, DEFAULT_INITIAL_PWM,
                               validate_maneuver_initial_pwm)
from turn_motion import TurnMotion


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
        self.assertEqual(saturated, 1700)

    def test_trend_can_release_only_twenty_percent_of_current_error(self):
        h = Harness()
        h.drive()
        h.motion.last_error = math.radians(69)
        target_pwm = h.motion._target(corridor(60), .1)
        self.assertEqual(target_pwm, 1651)
        h.motion.last_error = math.radians(5)
        smaller = h.motion._target(corridor(0), .1)
        self.assertEqual(smaller, 1500)

    def test_neutral_presteer_ambiguity_holds_adopted_servo_without_maturing(self):
        h = Harness()
        h.step()
        source = (h.motion.last_geometry_receive, h.motion.last_geometry_publication,
                  h.motion.last_geometry_seq)
        servo = h.servo
        def ambiguous(*args):
            raise ValueError('left_turn_outer_wall_ambiguous')
        h.motion._measurement = ambiguous
        h.motion.settle_since = h.now
        h.motion.settle_feedback_ticks = 8
        result = h.step()
        self.assertEqual((result['phase'], result['motor'], result['servo']), ('presteer', 1500, servo))
        self.assertEqual(result['reason'], 'left_turn_presteer_wall_ambiguity_hold')
        self.assertEqual(result['turn_stage'], 'presteer_wait')
        self.assertFalse(result['start_ready'])
        self.assertIsNone(h.motion.settle_since)
        self.assertEqual(result['steering_settle_feedback_ticks'], 0)
        self.assertIsNone(result['natural_steering_target'])
        self.assertEqual((h.motion.last_geometry_receive, h.motion.last_geometry_publication,
                          h.motion.last_geometry_seq), source)

    def test_neutral_hold_original_eight_second_budget_expires_and_cannot_restart(self):
        h = Harness()
        h.step()
        original = h.motion._measurement
        def ambiguous(*args):
            raise ValueError('left_turn_outer_wall_ambiguous')
        h.motion._measurement = ambiguous
        while h.now < 7.9:
            result = h.step()
            self.assertEqual(result['phase'], 'presteer')
            self.assertEqual(result['motor'], 1500)
        result = h.step()
        self.assertEqual(result['reason'], 'left_turn_presteer_timeout')
        self.assertEqual(result['phase'], 'locked')
        h.motion._measurement = original
        self.assertEqual(h.step()['phase'], 'locked')

    def test_unique_real_geometry_recovery_restarts_full_presteer_allowance(self):
        h = Harness()
        h.step()
        original = h.motion._measurement
        def ambiguous(*args):
            raise ValueError('left_turn_outer_wall_ambiguous')
        h.motion._measurement = ambiguous
        held = h.step()
        self.assertEqual(held['motor'], 1500)
        h.motion._measurement = original
        result = h.step()
        self.assertEqual(result['phase'], 'presteer')
        self.assertEqual(result['servo'], held['servo']+10)
        self.assertEqual(result['steering_settle_feedback_ticks'], 0)
        while result['phase'] == 'presteer':
            result = h.step()
        self.assertEqual(result['phase'], 'drive')
        self.assertGreaterEqual(result['steering_settle_feedback_ticks'], 3)
        self.assertGreaterEqual(result['steering_settle_elapsed_s']+1e-9, 1.2)

    def test_neutral_ambiguity_cannot_hold_unknown_or_unadopted_feedback(self):
        for invalid in ('no_left', 'motor', 'servo', 'ack', 'unarmed', 'old_tick', 'stale_scan'):
            with self.subTest(invalid=invalid):
                h = Harness()
                if invalid != 'no_left': h.step()
                def ambiguous(*args):
                    raise ValueError('left_turn_outer_wall_ambiguous')
                h.motion._measurement = ambiguous
                value = scan(h.seq+1, h.now+.1)
                feedback = {'armed': True, 'motor': 1500, 'servo': h.servo,
                            'seq': h.seq+1, 'tick': (h.seq+1)*100, 'command_acked': True}
                if invalid == 'motor': feedback['motor'] = 1560
                if invalid == 'servo': feedback['servo'] -= 10
                if invalid == 'ack': feedback['command_acked'] = False
                if invalid == 'unarmed': feedback['armed'] = False
                if invalid == 'old_tick': feedback['tick'] -= 100
                result = h.motion.update(value, .3 if invalid == 'stale_scan' else 0., h.now+.1, feedback)
                self.assertEqual(result['phase'], 'locked')
                self.assertEqual(result['motor'], 1500)

    def test_default_turn_motion_still_locks_neutral_wall_ambiguity(self):
        motion = TurnMotion(0., initial_presteer_pwm=1700)
        control = {'armed': True, 'motor': 1500, 'servo': 1500, 'seq': 1, 'tick': 100}
        first = motion.update(scan(1, .1), 0., .1, control)
        def ambiguous(*args):
            raise ValueError('left_turn_outer_wall_ambiguous')
        motion._measurement = ambiguous
        control.update(servo=first['servo'], seq=2, tick=200)
        result = motion.update(scan(2, .2), 0., .2, control)
        self.assertEqual(result['phase'], 'locked')
        self.assertEqual(result['reason'], 'left_turn_outer_wall_ambiguous')

    def test_default_1690_is_fully_adopted_before_any_forward_output(self):
        h = Harness(initial=1690)
        h.motion = ManeuverSequence(0.)
        self.assertEqual(h.motion.initial_presteer_pwm, DEFAULT_INITIAL_PWM)
        earlier = []
        while True:
            result = h.step()
            if result['phase'] == 'drive': break
            earlier.append(result)
        self.assertTrue(any(v['servo'] == 1680 for v in earlier))
        self.assertTrue(all(v['motor'] == 1500 for v in earlier))
        self.assertEqual(result['servo'], 1690)
        self.assertEqual(result['motor'], 1560)
        self.assertGreaterEqual(result['steering_settle_feedback_ticks'], 3)
        self.assertGreaterEqual(result['steering_settle_elapsed_s']+1e-9, 1.2)

    def test_actual_scan637_holds_neutral_then_scan639_recovers_geometry(self):
        # Real recorded geometry, with mock fresh neutral adoption feedback.
        # Actual run08 was already locked after637; this tests the repaired
        # state machine, not a physical trajectory or automatic rearm.
        fixture = json.loads((Path(__file__).parent/'fixtures'/'left-cone-08-neutral-ambiguity.json').read_text())
        initial = fixture['initial']
        def seed(cls):
            motion = cls(0., initial_presteer_pwm=1700)
            motion.selected = True
            motion.servo = fixture['held_servo']
            motion.steering_target = 1700
            motion.last_change = 0.
            motion.corridor = initial['corridor']
            motion.outer_wall = initial['outer_wall']
            motion.turn_goal = initial['turn_goal']
            motion.geometry_mode = initial['geometry_mode']
            motion.last_seq = motion.last_geometry_seq = initial['geometry_source_seq']
            motion.last_publication = motion.last_geometry_publication = initial['geometry_source_at_ms']
            motion.last_receive = motion.last_geometry_receive = 0.
            return motion
        motion = seed(ManeuverSequence)
        motion._last_actual_left_target = fixture['last_actual_left_target']
        first, second = fixture['frames']
        now = (first['at_ms']-initial['geometry_source_at_ms'])/1000
        control = {'armed': True, 'motor': 1500, 'servo': fixture['held_servo'],
                   'seq': 12, 'tick': 63797, 'command_acked': True}
        held = motion.update({**first, 'received_at': now}, 0., now, control)
        self.assertEqual(held['reason'], 'left_turn_presteer_wall_ambiguity_hold')
        self.assertEqual((held['phase'], held['motor'], held['servo']), ('presteer', 1500, 1540))
        self.assertEqual(held['geometry_source_seq'], 636)
        self.assertEqual(held['steering_settle_feedback_ticks'], 0)
        self.assertFalse(held['start_ready'])
        previous = seed(TurnMotion).update({**first, 'received_at': now}, 0., now, control)
        self.assertEqual(previous['phase'], 'locked')
        self.assertEqual(previous['reason'], 'left_turn_outer_wall_ambiguous')
        now = (second['at_ms']-initial['geometry_source_at_ms'])/1000
        control.update(seq=13, tick=64000)
        recovery = motion.update({**second, 'received_at': now}, 0., now, control)
        self.assertEqual((recovery['phase'], recovery['motor'], recovery['servo']), ('presteer', 1500, 1550))
        self.assertEqual(recovery['geometry_source_seq'], 639)
        self.assertEqual(recovery['steering_target'], 1700)
        self.assertEqual(recovery['steering_settle_feedback_ticks'], 0)

    def test_neutral_hold_duplicate_frames_cannot_renew_scan_lease(self):
        h = Harness()
        h.step()
        def ambiguous(*args):
            raise ValueError('left_turn_outer_wall_ambiguous')
        h.motion._measurement = ambiguous
        held = h.step()
        self.assertEqual(held['phase'], 'presteer')
        receive = h.motion.last_receive
        control = dict(h.latest_control)
        control.update(seq=control['seq']+1, tick=control['tick']+100)
        result = h.motion.update(h.latest_scan, 0., h.now+.2, control)
        self.assertEqual(result['phase'], 'presteer')
        self.assertEqual(h.motion.last_receive, receive)
        control.update(seq=control['seq']+1, tick=control['tick']+100)
        result = h.motion.update(h.latest_scan, 0., h.now+.31, control)
        self.assertEqual(result['phase'], 'locked')
        self.assertEqual(result['reason'], 'turn_scan_not_advancing')

    def test_new_presteer_budget_keeps_five_seconds_neutral_then_allows_full_settling(self):
        h = Harness(initial=1720)
        while h.now < 5.0:
            h.servo = 1500  # Mock feedback has not adopted the final command.
            result = h.step()
            self.assertEqual(result['phase'], 'presteer')
            self.assertEqual(result['motor'], 1500)
        self.assertEqual(result['presteer_max_s'], 8.)
        self.assertEqual(result['steering_settle_feedback_ticks'], 0)
        h.servo = 1720
        result = h.step()
        settled_from = h.motion.settle_since
        self.assertAlmostEqual(settled_from, 5.1)
        while result['phase'] == 'presteer':
            self.assertEqual(result['motor'], 1500)
            result = h.step()
        self.assertEqual(result['phase'], 'drive')
        self.assertEqual(result['motor'], 1560)
        self.assertAlmostEqual(h.now-settled_from, 1.2)
        self.assertGreaterEqual(result['steering_settle_feedback_ticks'], 3)
        self.assertLess(h.now, PRESTEER_MAX_S)

    def test_eight_second_exact_timeout_does_not_emit_forward_or_extend_budget(self):
        h = Harness(initial=1720)
        for _ in range(79):
            h.servo = 1500
            result = h.step()
            self.assertEqual((result['phase'], result['motor']), ('presteer', 1500))
        self.assertEqual(h.now, 7.9)
        result = h.step()
        self.assertEqual(h.now, 8.)
        self.assertEqual((result['phase'], result['motor'], result['servo']), ('locked', 1500, 1500))
        self.assertEqual(result['reason'], 'left_turn_presteer_timeout')
        self.assertEqual(h.motion.started_at, 0.)
        self.assertEqual(h.motion.max_presteer_s, PRESTEER_MAX_S)
        self.assertEqual(h.step()['phase'], 'locked')

    def test_original_turn_motion_keeps_five_second_presteer_limit(self):
        motion = TurnMotion(0., initial_presteer_pwm=1700)
        for seq in range(1, 51):
            now = seq/10
            control = {'armed': True, 'motor': 1500, 'servo': 1500, 'seq': seq, 'tick': seq*100}
            result = motion.update(scan(seq, now), 0., now, control)
            if seq < 50:
                self.assertEqual((result['phase'], result['motor']), ('presteer', 1500))
        self.assertEqual(motion.max_presteer_s, 5.)
        self.assertEqual(result['phase'], 'locked')
        self.assertEqual(result['reason'], 'left_turn_presteer_timeout')

    def test_new_initial_candidate_range_is_strict_and_old_turn_range_is_preserved(self):
        for value in range(1690, 1721):
            self.assertEqual(validate_maneuver_initial_pwm(value), value)
            self.assertEqual(ManeuverSequence(0., initial_presteer_pwm=value).initial_presteer_pwm, value)
        for value in [None, True, False, 1690., '1690', 1689, 1650, 1721, float('nan')]:
            with self.subTest(value=value):
                with self.assertRaisesRegex(ValueError, 'invalid_maneuver_initial_presteer_pwm'):
                    validate_maneuver_initial_pwm(value)
                with self.assertRaisesRegex(ValueError, 'invalid_maneuver_initial_presteer_pwm'):
                    ManeuverSequence(0., initial_presteer_pwm=value)
        self.assertEqual(TurnMotion(0., initial_presteer_pwm=1650).initial_presteer_pwm, 1650)

    def test_selected_candidate_caps_left_drive_and_current_error_still_releases(self):
        for selected in [1690, 1700, 1720]:
            with self.subTest(selected=selected):
                h = Harness(initial=selected)
                h.drive()
                targets = []
                for heading in [60, 50, 40, 30, 20, 10, 0]:
                    h.seq += 1
                    h.now = h.seq/10
                    value = scan(h.seq, h.now)
                    value['corridor_candidates'] = [corridor(heading), corridor(heading-180)]
                    control = {'armed': True, 'motor': h.motor, 'servo': h.servo,
                               'seq': h.seq, 'tick': h.seq*100}
                    result = h.motion.update(value, 0., h.now, control)
                    h.servo, h.motor = result['servo'], result['motor']
                    targets.append(result['steering_target'])
                    self.assertEqual(result['phase'], 'drive')
                    self.assertTrue(1500 <= result['servo'] <= selected)
                    self.assertTrue(1500 <= result['steering_target'] <= selected)
                    self.assertEqual(result['left_turn_servo_cap'], selected)
                self.assertLess(targets[-1], targets[0])
                self.assertEqual(targets[-1], 1500)

    def test_confirmed_orbit_feedback_can_exceed_left_turn_candidate_cap(self):
        h = Harness(initial=1690)
        h.drive()
        h.observer.factory = lambda value: target(value, bearing=90.)
        first = h.step()
        self.assertEqual(first['turn_stage'], 'orbit_entry')
        self.assertEqual(first['steering_target'], 1690)
        h.observer.factory = lambda value: target(value, bearing=120.)
        second = h.step()
        self.assertEqual(second['steering_target'], 1720)
        self.assertGreater(second['servo'], 1690)
        self.assertLessEqual(second['servo']-first['servo'], 10)
        self.assertTrue(second['handover_observed'])


if __name__ == '__main__':
    unittest.main()
