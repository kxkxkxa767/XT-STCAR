"""Pure control-sequence checks; mock observations are not target detection QA."""
import json
import copy
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
                               'armed': True, 'motor': self.motor, 'servo': self.servo,
                               'command_acked': True}
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
    def pass_step(self, h, x, *, target_present=True, partial=False, corridors=None, ack=True):
        from compact_target import _candidates
        from test_compact_target import compact_scan
        h.seq += 1
        h.now = h.seq/10
        value = scan(h.seq, h.now)
        value['ranges'] = compact_scan(bearing=math.degrees(math.atan2(.5, x)),
                                        distance=math.hypot(x, .5))['ranges']
        value['corridor_candidates'] = ([corridor(-35), corridor(145)] if corridors is None else corridors)
        candidates = _candidates(value['ranges'])
        observed = None
        if target_present and candidates:
            c = candidates[0]
            observed = {**target(value), **c, 'tracking_only_boundary_gap': partial}
        h.observer.factory = lambda _: observed
        feedback = {'armed': True, 'motor': h.motor, 'servo': h.servo,
                    'tick': h.seq*100, 'seq': h.seq, 'command_acked': ack}
        result = h.motion.update(value, 0., h.now, feedback)
        h.servo, h.motor = result['servo'], result['motor']
        return result

    def pass_harness(self):
        h = Harness(initial=1670)
        h.orbit(bearing=80.)
        outputs = [self.pass_step(h, x) for x in [.4, .35, .28, .18, .1, 0., -.1, -.2, -.3]]
        self.assertTrue(outputs[-1]['first_pass_evidence'])
        return h, outputs

    def test_first_pass_prepares_before_rear_then_requires_adopted_center_for_right(self):
        h, outputs = self.pass_harness()
        preparation = next(r for r in outputs if r['first_pass_preparing'])
        self.assertGreater(preparation['first_pass_progress']['observed_frontmost_x_m'], -.18)
        self.assertIsNone(preparation['first_pass_evidence'])
        self.assertFalse(preparation['orbit_left_entry_boost'])
        for _ in range(19):
            outputs.append(self.pass_step(h, -.3, target_present=False))
        right = [r for r in outputs if r['servo'] < 1500]
        self.assertTrue(right)
        self.assertEqual(min(r['servo'] for r in outputs), 1390)
        self.assertTrue(all(r['servo'] >= 1350 for r in outputs))
        self.assertTrue(all(r['right_exit_center_ack']['servo'] == 1500 for r in right))
        self.assertTrue(all(r['first_pass_evidence'] is not None for r in right))
        self.assertTrue(all(r['turn_stage'] == 'right_exit' for r in right))
        for a, b in zip(outputs, outputs[1:]):
            limit = 20 if a['servo'] > 1500 and b['servo'] < a['servo'] else 10
            self.assertLessEqual(abs(b['servo']-a['servo']), limit)
            if a['servo'] > 1500:
                self.assertGreaterEqual(b['servo'], 1500)
        self.assertTrue(all(r['motor'] == 1560 for r in outputs))
        self.assertFalse(outputs[-1]['second_target_handover_observed'])
        self.assertFalse(outputs[-1]['completed'])

    def test_pass_preparation_needs_full_current_approaching_target_and_both_clocks(self):
        for invalid in ('partial', 'receding', 'unacked'):
            with self.subTest(invalid=invalid):
                h = Harness(initial=1670)
                h.orbit()
                xs = [.01, .04, .08, .12, .18] if invalid == 'receding' else [.4, .3, .2, .1, 0.]
                for x in xs:
                    result = self.pass_step(h, x, partial=invalid=='partial', ack=invalid!='unacked')
                    self.assertFalse(result['first_pass_preparing'])
                    self.assertIsNone(result['first_pass_evidence'])

    def test_target_loss_before_rear_evidence_still_coasts_and_never_restarts(self):
        h = Harness(initial=1670)
        h.orbit()
        for x in [.4, .3, .2, .1]:
            result = self.pass_step(h, x)
        self.assertTrue(result['first_pass_preparing'])
        self.assertIsNone(result['first_pass_evidence'])
        stopped = self.pass_step(h, 0., target_present=False)
        self.assertEqual((stopped['phase'], stopped['motor']), ('coast', 1500))
        self.assertEqual(self.pass_step(h, -.3)['motor'], 1500)

    def test_right_exit_missing_geometry_cannot_renew_or_advance_command(self):
        h, _ = self.pass_harness()
        adopted = h.servo
        for _ in range(2):
            result = self.pass_step(h, -.3, target_present=False, corridors=[])
            self.assertEqual(result['motor'], 1560)
            self.assertEqual(result['servo'], adopted)
        result = self.pass_step(h, -.3, target_present=False, corridors=[])
        self.assertEqual((result['phase'], result['motor']), ('coast', 1500))
        self.assertEqual(result['reason'], 'right_exit_corridor_missing')
        self.assertEqual(self.pass_step(h, -.3)['motor'], 1500)

    def test_right_exit_ambiguity_identity_jump_and_budget_still_stop(self):
        for failure in ('ambiguity', 'identity', 'invalid', 'budget'):
            with self.subTest(failure=failure):
                h, _ = self.pass_harness()
                values = [corridor(-35), corridor(-10)] if failure == 'ambiguity' else (
                    [corridor(20)] if failure == 'identity' else [{'fake': True}] if failure == 'invalid' else None)
                if failure == 'budget':
                    h.motion.right_exit_since = h.now-3
                result = self.pass_step(h, -.3, corridors=values)
                self.assertEqual((result['phase'], result['motor']), ('coast', 1500))

    def test_recorded29_geometry_prepares_before_observed_target_reaches_rear(self):
        from compact_target import _candidates
        fixture = json.loads((Path(__file__).parent/'fixtures'/'left-cone-29-pass-exit.json').read_text())
        motion = ManeuverSequence(0.)
        motion.orbit_track_id = 1
        motion.servo = 1700  # Mock adopted command, not a changed physical replay.
        first_prepare = None
        for raw in fixture['frames']:
            now = raw['at_ms']/1000
            value = {**raw, 'received_at': now}
            c = _candidates(raw['ranges'])[0]
            motion.compact_target = {**c, 'confirmed': True, 'track_id': 1}
            motion._observe_first_pass(value, now, {'motor': 1560, 'servo': 1700, 'command_acked': True})
            if motion.first_pass_preparing and first_prepare is None:
                first_prepare = raw['seq']
        self.assertEqual(first_prepare, 613)
        self.assertEqual(motion.first_pass_evidence['source_seq'], 620)
        goal, reason = motion._right_exit_target(value, now)
        self.assertIsNone(reason)
        self.assertTrue(1350 <= goal < 1500)
        self.assertFalse(motion.first_pass_evidence['physical_cone_pass_certified'])

    def test_right_exit_command_bound_and_cumulative_drive_budget(self):
        h, _ = self.pass_harness()
        h.motion.right_exit_geometry = None
        value = scan(h.seq+1, h.now+.1)
        value['corridor_candidates'] = [corridor(-80)]
        self.assertEqual(h.motion._right_exit_target(value, h.now+.1)[0], 1350)
        h.motion.max_drive_s = h.now-h.motion.drive_since
        result = self.pass_step(h, -.3)
        self.assertEqual((result['phase'], result['motor']), ('coast', 1500))
        self.assertEqual(result['reason'], 'maneuver_cumulative_drive_timeout')

    def test_right_exit_waits_for_each_adopted_command_then_coasts_on_timeout(self):
        h, _ = self.pass_harness()
        original = h.servo
        for _ in range(3):
            result = self.pass_step(h, -.3, ack=False)
            self.assertEqual(result['servo'], original)
            self.assertIsNone(result['right_exit_center_ack'])
        stopped = self.pass_step(h, -.3, ack=False)
        self.assertEqual((stopped['phase'], stopped['motor']), ('coast', 1500))
        self.assertEqual(stopped['reason'], 'right_exit_adoption_unconfirmed')

    def test_first_pass_preparation_cannot_use_source_time_for_receive_maturity(self):
        from compact_target import _candidates
        fixture = json.loads((Path(__file__).parent/'fixtures'/'left-cone-29-pass-exit.json').read_text())
        motion = ManeuverSequence(0.)
        motion.orbit_track_id, motion.servo = 1, 1700
        for i, raw in enumerate(fixture['frames']):
            receive = i*.01  # Deliberately compressed; no250ms receipt evidence.
            motion.compact_target = {**_candidates(raw['ranges'])[0], 'confirmed': True, 'track_id': 1}
            motion._observe_first_pass({**raw, 'received_at': receive}, receive,
                {'motor': 1560, 'servo': 1700, 'command_acked': True})
        self.assertFalse(motion.first_pass_preparing)
        self.assertIsNone(motion.first_pass_evidence)

    def test_actual24_farther_endpoint_reduces_preparation_relative_to16(self):
        fixture = Path(__file__).with_name('fixtures')/'left-cone-24-entry-endpoint.json'
        caps = {}
        for record in json.loads(fixture.read_text())['records']:
            value = record['scan']
            motion = ManeuverSequence(0.)
            value = dict(value, received_at=.1)
            result = motion.update(value, 0., .1, {'armed': False, 'motor': 1500,
                'servo': 1500, 'tick': 100, 'seq': 0, 'command_acked': True})
            self.assertTrue(result['start_ready'])
            self.assertTrue(result['entry_bearing_required'])
            self.assertFalse(result['entry_bearing_released'])
            self.assertFalse(result['entry_bearing']['swept_path_certified'])
            self.assertEqual(result['entry_bearing']['source_seq'], value['seq'])
            self.assertEqual(result['motor'], 1500)
            caps[record['trial']] = result['steering_target']
        self.assertTrue(1500 < caps[24] < caps[16] < 1670, caps)

    def test_endpoint_limited_preparation_still_waits_full_adoption_allowance(self):
        fixture = Path(__file__).with_name('fixtures')/'left-cone-24-entry-endpoint.json'
        original = json.loads(fixture.read_text())['records'][1]['scan']
        motion = ManeuverSequence(0.)
        servo, motor, first_adopted = 1500, 1500, None
        for seq in range(1, 61):
            now = seq/10
            value = dict(original, seq=seq, at_ms=seq*100, received_at=now)
            adopted = {'armed': True, 'servo': servo, 'motor': motor,
                       'tick': seq*100, 'seq': seq, 'command_acked': True}
            result = motion.update(value, 0., now, adopted)
            servo, motor = result['servo'], result['motor']
            if adopted['servo'] == result['steering_target'] and first_adopted is None:
                first_adopted = now
            if motor == 1560:
                self.assertTrue(1500 < servo < 1670)
                self.assertGreaterEqual(now-first_adopted+1e-9, 1.2)
                self.assertGreaterEqual(result['steering_settle_feedback_ticks'], 3)
                self.assertFalse(result['entry_bearing_released'])
                break
        else:
            self.fail('limited preparation never adopted')

    def test_endpoint_release_requires_advancing_evidence_then_keeps_left_demand(self):
        from test_turn_motion import opening_scan
        motion = ManeuverSequence(0.)
        servo = 1500
        for seq in range(1, 5):
            # Still ahead of the front projection, but its bearing permits left.
            value = dict(opening_scan(seq, endpoint_index=305), received_at=seq/10)
            adopted = {'armed': True, 'servo': servo, 'motor': 1500,
                       'tick': seq*100, 'seq': seq, 'command_acked': True}
            result = motion.update(value, 0., seq/10, adopted)
            servo = result['servo']
            self.assertEqual(result['entry_bearing_released'], seq == 4)
            duplicate = motion.update(value, 0., seq/10+.001, adopted)
            self.assertEqual(duplicate['entry_bearing']['confirmation_count'], seq)
        motion.phase, motion.drive_since = 'drive', .4
        motion.turn_goal['target_point_left_m'] = {'x_m': 1., 'y_m': .3}
        requested = motion._target(corridor(30.), .1)
        self.assertTrue(motion.left_turn_feedback['opening_left_continuation_active'])
        self.assertGreaterEqual(motion.left_turn_feedback['nominal_target_pwm'], 1670)
        self.assertGreater(requested, 1500)
        self.assertFalse(motion.entry_bearing['endpoint_passed_proven'])

    def test_recorded25_front_projection_releases_before_complete_opening_loss(self):
        fixture = Path(__file__).with_name('fixtures')/'left-cone-25-entry-endpoint-state.json'
        observations = json.loads(fixture.read_text())['observations']
        motion = ManeuverSequence(0.)
        motion.entry_bearing_required = True
        for value in observations:
            motion.last_now = value['at_ms']/1000
            motion._endpoint_observation(value)  # Geometry was already validated in saved state.
        self.assertTrue(motion.entry_bearing_released)
        self.assertLess(motion.entry_bearing['forward_gap_m'], 0)
        self.assertGreater(motion.entry_bearing['lateral_gap_m'], 0)
        self.assertLess(motion.entry_bearing['confirmation_count'], 3)
        self.assertEqual(motion.entry_bearing['release_basis'],
                         'current_endpoint_at_body_front_projection')
        self.assertFalse(motion.entry_bearing['endpoint_passed_proven'])
        motion._endpoint_observation({'seq': 605, 'at_ms': observations[-1]['at_ms']+100,
                                      'left_turn_goal': None})
        self.assertTrue(motion.entry_bearing_released)
        self.assertEqual(motion._entry_target(1670), 1670)

    def test_front_projection_does_not_release_without_lateral_gap(self):
        from test_turn_motion import opening_scan
        value = opening_scan(1, endpoint_index=270)
        value['ranges'][270] = .21
        support = value['left_turn_goal']['incoming_left_end_support']
        support['range_m'] = .21
        support['point_left_m'] = {'x_m': 0., 'y_m': .21}
        value['left_turn_goal']['incoming_left_end_m'] = 0.
        motion = ManeuverSequence(0.)
        result = motion.update(value, 0., .1, {'armed': False, 'motor': 1500,
            'servo': 1500, 'seq': 0, 'tick': 100, 'command_acked': True})
        self.assertFalse(result['entry_bearing_released'])
        self.assertFalse(result['start_ready'])
        self.assertEqual(result['motor'], 1500)

    def test_missing_endpoint_cannot_handover_or_extend_early_approach(self):
        from test_turn_motion import opening_scan
        motion = ManeuverSequence(0.)
        original = opening_scan(1, endpoint_index=325)
        original['received_at'] = .1
        adopted = {'armed': True, 'servo': 1500, 'motor': 1500,
                   'tick': 100, 'seq': 1, 'command_acked': True}
        motion.update(original, 0., .1, adopted)
        motion.phase, motion.drive_since, motion.servo = 'drive', .1, 1590
        value = copy.deepcopy(original)
        value['left_turn_goal'] = None
        for seq in range(2, 5):
            now = seq/10
            value.update(seq=seq, at_ms=seq*100, received_at=now)
            adopted.update(servo=1590, motor=1560, seq=seq, tick=seq*100)
            result = motion.update(value, 0., now, adopted)
            self.assertIsNone(result['entry_bearing'])
            self.assertFalse(result['entry_bearing_released'])
            self.assertFalse(motion._usable_target(target(value), value))
            if seq < 4:
                self.assertEqual(result['motor'], 1560)
                self.assertEqual(result['servo'], 1590)
            else:
                self.assertEqual(result['motor'], 1500)
                self.assertEqual(result['terminal_reason'], 'left_turn_entry_endpoint_lost')

    def test_actual21_same_wall_moderate_width_change_continues_but_old_turn_rejects(self):
        fixture = Path(__file__).with_name('fixtures')/'left-cone-21-exit-width.json'
        scans = json.loads(fixture.read_text())['scans']
        previous, current = scans[:2]
        for cls in [ManeuverSequence, TurnMotion]:
            with self.subTest(controller=cls.__name__):
                motion = cls(0.)
                motion.corridor = motion._measurement(previous, [], .1)
                motion.selected, motion.phase = True, 'drive'
                dt = (current['at_ms']-previous['at_ms'])/1000
                if cls is TurnMotion:
                    with self.assertRaisesRegex(ValueError, 'left_turn_exit_width_jump'):
                        motion._measurement(current, [], dt)
                else:
                    result = motion._measurement(current, [], dt)
                    self.assertEqual(result['width_m'], current['left_turn_goal']['width_m'])
                    self.assertTrue(motion.exit_width_change['accepted'])
                    self.assertAlmostEqual(motion.exit_width_change['absolute_change_m'], .16073534680188284)
                    self.assertAlmostEqual(motion.exit_width_change['relative_change'], .08523824014010425)
                    self.assertEqual(motion.exit_width_change['source_seq'], 579)

    def test_width_limit_scales_but_large_jump_uses_current_wall_without_new_width(self):
        from test_turn_motion import opening_scan
        def opening(seq, width):
            value = opening_scan(seq)
            goal = value['left_turn_goal']
            goal['width_m'] = width
            goal['center_offset_left_m'] = goal['outer_wall']['rho_left_m']+width/2
            ray = goal['target_support_ray']
            angle, heading = ray['angle_left_rad'], goal['heading_left_rad']
            distance = goal['center_offset_left_m']/math.sin(angle-heading)
            ray['target_range_m'] = distance
            goal['target_point_left_m'] = {'x_m': distance*math.cos(angle), 'y_m': distance*math.sin(angle)}
            return value
        for width in [1.2, 2., 3.]:
            limit = max(.15, width*.1)
            for sign in [-1, 1]:
                for extra in [0., .0001]:
                    with self.subTest(width=width, sign=sign, excess=extra):
                        motion = ManeuverSequence(0.)
                        previous = opening(1, width)
                        motion.corridor = motion._measurement(previous, [], .1)
                        motion.selected, motion.phase = True, 'drive'
                        current = opening(2, width+sign*(limit+extra))
                        result = motion._measurement(current, [], .1)
                        self.assertEqual(result['width_m'], width if extra else current['left_turn_goal']['width_m'])
                        self.assertEqual(motion.exit_width_change['action'],
                            'track_current_wall_keep_previous_width' if extra else 'adopt_current_width')
                        if extra:
                            self.assertEqual(motion.turn_goal['observation_type'], 'tracked_actual_outer_wall')
                            self.assertEqual(motion.turn_goal['outer_wall'], current['left_turn_goal']['outer_wall'])
                            self.assertIsNone(motion.turn_goal['target_support_ray'])
                            motion.phase = 'presteer'
                            with self.assertRaisesRegex(ValueError, 'left_turn_exit_width_jump'):
                                motion._measurement(current, [], .1)
                        self.assertAlmostEqual(motion.exit_width_change['allowed_change_m'], limit)

    def test_small_width_change_does_not_bypass_missing_or_invalid_wall_evidence(self):
        from test_turn_motion import opening_scan
        for fault in ['missing', 'identity_jump', 'invalid_ray']:
            with self.subTest(fault=fault):
                motion = ManeuverSequence(0.)
                previous = opening_scan(1)
                motion.corridor = motion._measurement(previous, [], .1)
                motion.selected, motion.phase = True, 'drive'
                current = copy.deepcopy(opening_scan(2))
                if fault == 'missing':
                    current['left_turn_goal'], current['wall_candidates'] = None, []
                elif fault == 'identity_jump':
                    motion.outer_wall['rho_left_m'] -= 1.
                else:
                    current['ranges'][current['left_turn_goal']['target_support_ray']['index']] = None
                with self.assertRaises(ValueError):
                    motion._measurement(current, [], .1)
                self.assertIsNone(motion.exit_width_change)

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
        self.assertTrue(all(abs(a['servo']-b['servo']) <= 20 for a, b in zip(steps, steps[1:])))

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

    def test_entry_feedback_preserves_left_demand_then_increases_for_lag_or_range(self):
        h = Harness()
        first = h.orbit(distance=.9)
        h.observer.factory = lambda value: target(value, bearing=70., distance=.8)
        second = h.step()
        h.observer.factory = lambda value: target(value, bearing=110., distance=1.05)
        third = h.step()
        self.assertEqual(second['steering_target'], first['steering_target'])
        self.assertGreater(third['steering_target'], second['steering_target'])
        self.assertEqual(third['orbit_reference_range_m'], .9)
        self.assertTrue(1500 <= second['servo'] <= 1720)
        self.assertLessEqual(abs(third['servo']-second['servo']), 10)

    def test_weak_handover_is_continuous_then_slews_toward_prepared_left_baseline(self):
        h = Harness(initial=1670)
        h.drive()
        h.servo = h.motion.servo = 1598  # Mock adopted feedback after the wall turn.
        h.observer.factory = lambda value: target(value, bearing=69.5, distance=.965)
        first = h.step()
        self.assertEqual(first['servo'], 1598)
        self.assertEqual(first['steering_target'], 1598)
        self.assertEqual(first['handover_control']['servo'], 1598)
        h.observer.factory = lambda value: target(value, bearing=70.5, distance=.901)
        second = h.step()
        self.assertTrue(1670 < second['steering_target'] <= 1720)
        self.assertEqual(second['orbit_feedback']['nominal_target_pwm'], 1720)
        self.assertEqual(second['servo'], 1608)
        self.assertEqual(second['orbit_feedback']['adopted_handover_bias_pwm'], 1598)
        self.assertEqual(second['orbit_feedback']['entry_base_pwm'], 1670)
        self.assertTrue(second['orbit_feedback']['left_entry_boost_active'])
        self.assertLess(second['orbit_feedback']['bearing_term_pwm'], 0)
        self.assertLess(second['orbit_feedback']['range_term_pwm'], 0)
        self.assertEqual(second['orbit_feedback']['applied_entry_correction_pwm'], 0)
        h.observer.factory = lambda value: target(value, bearing=105., distance=.9)
        third = h.step()
        self.assertGreater(third['steering_target'], 1598)
        self.assertLessEqual(third['servo']-second['servo'], 10)

    def test_ahead_target_entry_continues_left_then_releases_boost_on_current_abeam(self):
        h = Harness(initial=1670)
        first = h.orbit(bearing=78., distance=1.1)
        self.assertEqual(first['servo'], first['handover_control']['servo'])
        previous = first['servo']
        for _ in range(7):
            result = h.step()
            self.assertTrue(result['orbit_left_entry_boost'])
            self.assertEqual(result['orbit_feedback']['nominal_target_pwm'], 1720)
            self.assertTrue(1670 < result['steering_target'] <= 1720)
            self.assertTrue(0 <= result['servo']-previous <= 10)
            previous = result['servo']
        self.assertTrue(1710 <= previous <= 1720)
        h.observer.factory = lambda value: target(value, bearing=95., distance=1.1)
        for i in range(4):
            result = h.step()
            self.assertEqual(result['orbit_left_entry_boost'], i < 3)
        self.assertLess(result['steering_target'], 1720)
        self.assertGreaterEqual(result['steering_target'], 1670)
        self.assertFalse(result['completed'])

    def test_ahead_entry_boost_keeps_inside_release_and_target_loss_stop(self):
        h = Harness(initial=1670)
        h.orbit(bearing=78., distance=1.1)
        value = scan(h.seq+1, h.now+.1)
        value['ranges'][300] = .3
        h.motion._current_scan = value
        nominal = h.motion._relative_target_pwm(target(value, bearing=78., distance=1.))
        self.assertEqual(nominal, 1720)
        self.assertLess(h.motion._limit_left_for_known_points(nominal, 1.), nominal)
        h.observer.factory = lambda value: None
        stopped = h.step()
        self.assertEqual(stopped['motor'], 1500)
        self.assertEqual(stopped['phase'], 'coast')
        h.observer.factory = lambda value: target(value, bearing=78., distance=1.)
        self.assertEqual(h.step()['motor'], 1500)

    def test_entry_holding_left_does_not_disable_current_inside_edge_release(self):
        h = Harness()
        h.orbit()
        value = scan(h.seq+1, h.now+.1)
        value['ranges'][300] = .3
        h.motion._current_scan = value
        nominal = h.motion._relative_target_pwm(target(value, bearing=70., distance=.9))
        limited = h.motion._limit_left_for_known_points(nominal, 1.)
        self.assertEqual(nominal, h.motion.orbit_bias_pwm)
        self.assertLess(limited, nominal)
        self.assertEqual(h.motion.inner_clearance['source_seq'], value['seq'])

    def test_opening_alignment_cannot_erase_prepared_left_before_handover(self):
        h = Harness(initial=1670)
        h.drive()
        h.motion.geometry_mode = 'opening_wall'
        h.motion.turn_goal = {'target_point_left_m': {'x_m': .5, 'y_m': .1}}
        h.motion.last_error = None
        earlier = h.motion._target(corridor(40.), .1)
        self.assertLess(earlier, 1670)
        self.assertFalse(h.motion.left_turn_feedback['opening_left_continuation_active'])
        # Left exit heading with a rightward projected alignment point: the
        # failure pattern from trial19, varied without any fixed course lookup.
        for heading, y in [(28., -.4), (23., -.35), (10., -.1)]:
            with self.subTest(heading=heading):
                h.motion.last_error = None
                h.motion.turn_goal = {'target_point_left_m': {'x_m': .5, 'y_m': y}}
                requested = h.motion._target(corridor(heading), .1)
                self.assertLess(h.motion.left_turn_feedback['alignment_target_pwm'], 1670)
                self.assertEqual(h.motion.left_turn_feedback['nominal_target_pwm'], 1670)
                self.assertEqual(requested, 1670)
        h.motion.last_error = None
        h.motion.turn_goal = {'target_point_left_m': {'x_m': .5, 'y_m': .01}}
        self.assertEqual(h.motion._target(corridor(23.), .1), 1670)
        self.assertFalse(h.motion.left_turn_feedback['handover_preparation_trigger_current'])
        self.assertTrue(h.motion.left_turn_feedback['opening_left_continuation_active'])
        h.motion.last_error = None
        h.motion.turn_goal = {'target_point_left_m': {'x_m': .5, 'y_m': .8}}
        stronger = h.motion._target(corridor(90.), .1)
        self.assertGreater(stronger, 1670)
        self.assertLessEqual(stronger, 1720)

    def test_opening_continuation_keeps_live_inside_edge_release(self):
        h = Harness(initial=1670)
        h.drive()
        h.motion.geometry_mode = 'opening_wall'
        h.motion.turn_goal = {'target_point_left_m': {'x_m': .5, 'y_m': -.4}}
        h.motion._current_scan = scan(h.seq+1, h.now+.1)
        h.motion._current_scan['ranges'][300] = .3
        requested = h.motion._target(corridor(25.), .1)
        self.assertEqual(h.motion.left_turn_feedback['nominal_target_pwm'], 1670)
        self.assertTrue(1500 < requested < 1670)
        self.assertTrue(h.motion.inner_clearance['active'])

    def test_prepared_orbit_baseline_uses_this_runs_selected_pwm(self):
        for initial in [1670, 1690, 1720]:
            with self.subTest(initial=initial):
                h = Harness(initial=initial)
                h.drive()
                h.servo = h.motion.servo = 1538
                h.observer.factory = lambda value: target(value, bearing=72.)
                first = h.step()
                self.assertEqual(first['servo'], 1538)
                second = h.step()
                self.assertEqual(second['orbit_feedback']['entry_base_pwm'], initial)
                self.assertEqual(second['orbit_feedback']['nominal_target_pwm'], 1720)
                self.assertEqual(second['servo'], 1548)

    def test_target_loss_keeps_exact_decision_scan_after_recovery(self):
        h = Harness()
        prior = h.orbit()
        h.observer.factory = lambda value: None
        h.observer.reason = 'compact_target_missing'
        stopped = h.step()
        evidence = copy.deepcopy(stopped['compact_loss_evidence'])
        self.assertEqual(evidence['scan'], {k: h.latest_scan[k] for k in
            ('frame_id', 'seq', 'at_ms', 'received_at', 'ranges')})
        self.assertEqual(evidence['previous_confirmed_target'], prior['compact_target'])
        self.assertEqual(evidence['tracker_reason'], 'compact_target_missing')
        self.assertEqual(evidence['control'], h.latest_control)
        self.assertEqual(stopped['motor'], 1500)
        stopped['compact_loss_evidence']['scan']['ranges'][0] = 99
        h.latest_scan['ranges'][1] = 99
        h.observer.factory = lambda value: target(value, identity=2)
        recovered = h.step()
        self.assertEqual(recovered['compact_loss_evidence'], evidence)
        self.assertEqual(recovered['phase'], 'coast')
        self.assertEqual(recovered['motor'], 1500)

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
        self.assertEqual(result['servo'], held['servo']+20)
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

    def test_default_1670_is_fully_adopted_before_any_forward_output(self):
        h = Harness(initial=1670)
        h.motion = ManeuverSequence(0.)
        self.assertEqual(h.motion.initial_presteer_pwm, DEFAULT_INITIAL_PWM)
        earlier = []
        while True:
            result = h.step()
            if result['phase'] == 'drive': break
            earlier.append(result)
        self.assertTrue(any(v['servo'] == 1660 for v in earlier))
        self.assertTrue(all(v['motor'] == 1500 for v in earlier))
        self.assertEqual(result['servo'], 1670)
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
        self.assertEqual((recovery['phase'], recovery['motor'], recovery['servo']), ('presteer', 1500, 1560))
        self.assertEqual(recovery['geometry_source_seq'], 639)
        self.assertEqual(recovery['steering_target'], 1670)  # Current endpoint bounds this early preparation.
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
        for value in range(1670, 1721):
            self.assertEqual(validate_maneuver_initial_pwm(value), value)
            self.assertEqual(ManeuverSequence(0., initial_presteer_pwm=value).initial_presteer_pwm, value)
        for value in [None, True, False, 1670., '1670', 1669, 1650, 1721, float('nan')]:
            with self.subTest(value=value):
                with self.assertRaisesRegex(ValueError, 'invalid_maneuver_initial_presteer_pwm'):
                    validate_maneuver_initial_pwm(value)
                with self.assertRaisesRegex(ValueError, 'invalid_maneuver_initial_presteer_pwm'):
                    ManeuverSequence(0., initial_presteer_pwm=value)
        self.assertEqual(TurnMotion(0., initial_presteer_pwm=1650).initial_presteer_pwm, 1650)

    def test_all_initial_candidates_keep_global_left_cap_and_error_still_releases(self):
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
                    self.assertTrue(1500 <= result['servo'] <= 1720)
                    self.assertTrue(1500 <= result['steering_target'] <= 1720)
                    self.assertEqual(result['left_turn_servo_cap'], 1720)
                self.assertLess(targets[-1], targets[0])
                self.assertEqual(targets[-1], 1500)

    def test_confirmed_orbit_feedback_uses_adopted_bias_with_global_cap(self):
        h = Harness(initial=1690)
        h.drive()
        h.observer.factory = lambda value: target(value, bearing=90.)
        first = h.step()
        self.assertEqual(first['turn_stage'], 'orbit_entry')
        self.assertEqual(first['steering_target'], 1690)
        h.observer.factory = lambda value: target(value, bearing=120.)
        second = h.step()
        self.assertEqual(second['orbit_feedback']['nominal_target_pwm'], 1720)
        self.assertTrue(1690 < second['steering_target'] <= 1720)
        self.assertGreater(second['servo'], 1690)
        self.assertLessEqual(second['servo']-first['servo'], 10)
        self.assertTrue(second['handover_observed'])

    def test_1670_preparation_matures_then_drive_can_gradually_request_1720(self):
        motion = ManeuverSequence(0.)
        servo, motor = 1500, 1500
        earlier = []
        seq = 0
        while True:
            seq += 1
            value = scan(seq, seq/10)
            value['corridor_candidates'] = [corridor(90), corridor(-90)]
            control = {'armed': True, 'motor': motor, 'servo': servo,
                       'seq': seq, 'tick': seq*100, 'command_acked': True}
            result = motion.update(value, 0., seq/10, control)
            servo, motor = result['servo'], result['motor']
            if result['phase'] == 'drive': break
            earlier.append(result)
        self.assertTrue(all(v['motor'] == 1500 for v in earlier))
        self.assertEqual(result['initial_presteer_pwm'], 1670)
        self.assertEqual(result['servo'], 1670)
        self.assertEqual(result['motor'], 1560)
        self.assertGreaterEqual(result['steering_settle_feedback_ticks'], 3)
        self.assertGreaterEqual(result['steering_settle_elapsed_s']+1e-9, 1.2)
        previous_servo = servo
        for _ in range(5):
            seq += 1
            value = scan(seq, seq/10)
            value['corridor_candidates'] = [corridor(90), corridor(-90)]
            control = {'armed': True, 'motor': motor, 'servo': servo,
                       'seq': seq, 'tick': seq*100, 'command_acked': True}
            result = motion.update(value, 0., seq/10, control)
            self.assertEqual(result['steering_target'], 1720)
            self.assertEqual(result['left_turn_servo_cap'], 1720)
            self.assertLessEqual(result['servo']-previous_servo, 10)
            previous_servo = servo = result['servo']
        self.assertEqual(servo, 1720)


if __name__ == '__main__':
    unittest.main()
