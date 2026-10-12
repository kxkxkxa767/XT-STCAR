"""Exact current-role contact exemptions; all unrelated returns remain hard stops."""
import copy
import math
import unittest
from unittest.mock import patch

import test_route_control as route


class RouteContactControlTests(unittest.TestCase):
    setUp = route.RouteControlTests.setUp
    tearDown = route.RouteControlTests.tearDown
    scan = staticmethod(route.RouteControlTests.scan)
    tick = route.RouteControlTests.tick
    observe = route.RouteControlTests.observe
    start_compact_trial = route.RouteControlTests.start_compact_trial
    drive_compact_trial = route.RouteControlTests.drive_compact_trial
    enter_compact_orbit = route.RouteControlTests.enter_compact_orbit
    compact_scan = route.RouteControlTests.compact_scan
    target_bins = route.RouteControlTests.target_bins
    reset_case = route.RouteControlTests.reset_case

    def contact_scan(self, seq):
        value = self.compact_scan(seq)
        value['ranges'][268:273] = [.17]*5
        value['received_at'] = self.now
        return value

    @staticmethod
    def evidence(value, role='first_target'):
        support = list(range(268, 273))
        return {'role': role, 'track_id': 1, 'confirmed': True, 'full_observation': True,
                'source_seq': value['seq'], 'source_at_ms': value['at_ms'],
                'source_received_at': value['received_at'], 'support_bins': support,
                'support_ranges_m': [value['ranges'][i] for i in support],
                'semantic_class': 'unknown'}

    def clearance(self, value, evidence=None, **extra):
        return self.server.probe_clearance(value, {'lidar': .01, 'camera': .01, 'control': .01},
            True, camera_required=False, clearance_profile='maneuver',
            body_contact_target=evidence, **extra)

    def test_only_current_role_support_can_cross_body_distance_gate(self):
        value = self.contact_scan(10)
        with self.assertRaisesRegex(ValueError, 'probe_obstacle_close_body'):
            self.clearance(value)
        for role in ('first_target', 'second_target'):
            with self.subTest(role=role):
                current = self.clearance(value, self.evidence(value, role))
                body = current['body_proximity']
                self.assertTrue(body['target_contact_tolerated'])
                self.assertFalse(body['stop_requested'])
                self.assertLess(body['current_known_min_distance_m'], .04)
                self.assertEqual(body['default_stop_distance_m'], .08)
                self.assertEqual(len(body['target_contact_points']), 5)
                self.assertEqual(body['target_contact_evidence']['role'], role)
                self.assertEqual(value['ranges'][268:273], [.17]*5)

    def test_known_wall_next_to_target_still_stops_with_raw_contact_diagnostics(self):
        value = self.contact_scan(10)
        target = self.evidence(value)
        value['ranges'][0] = .25  # A separate wall point, five cm beyond the front.
        with self.assertRaisesRegex(ValueError, 'probe_obstacle_close_body') as caught:
            self.clearance(value, target)
        body = caught.exception.clearance['body_proximity']
        self.assertTrue(body['stop_requested'])
        self.assertTrue(body['target_contact_tolerated'])
        self.assertEqual(body['stop_trigger_point']['angle_deg'], 0)
        self.assertEqual(len(body['target_contact_points']), 5)

    def test_unproven_or_mismatched_witness_is_rejected(self):
        value = self.contact_scan(10)
        cases = {'wrong_role': {'role': 'wall'}, 'unconfirmed': {'confirmed': False},
                 'partial': {'full_observation': False}, 'wrong_id_type': {'track_id': True},
                 'missing_id': {'track_id': 0}, 'old_seq': {'source_seq': 9},
                 'old_publication': {'source_at_ms': value['at_ms']-1},
                 'old_receive': {'source_received_at': value['received_at']+.1},
                 'duplicates': {'support_bins': [268, 269, 270, 271, 271]},
                 'false_range': {'support_ranges_m': [.18]*5},
                 'too_few': {'support_bins': [268, 269], 'support_ranges_m': [.17]*2},
                 'bool_bin': {'support_bins': [True, 269, 270, 271, 272]},
                 'false_semantic': {'semantic_class': 'cone'}}
        for fault, change in cases.items():
            with self.subTest(fault=fault), self.assertRaisesRegex(ValueError, 'probe_invalid_body_contact_target'):
                self.clearance(value, {**self.evidence(value), **change})

    def test_contact_witness_cannot_extend_stale_scan_or_enable_straight_mode(self):
        value = self.contact_scan(10)
        target = self.evidence(value)
        for ages, profile in (({'lidar': .3, 'camera': .01, 'control': .01}, 'maneuver'),
                              ({'lidar': .01, 'camera': .01, 'control': .01}, 'straight')):
            with self.subTest(profile=profile), self.assertRaisesRegex(ValueError, 'probe_invalid_body_contact_target'):
                self.server.probe_clearance(value, ages, True, camera_required=False,
                    clearance_profile=profile, body_contact_target=target)

    def test_service_queries_contact_provider_only_in_explicit_route(self):
        for enabled in (False, True):
            with self.subTest(route=enabled):
                self.reset_case()
                _, session = self.enter_compact_orbit(continue_route=enabled)
                motion = session['turn_motion']
                value = self.contact_scan(self.console.scan['seq']+1)
                decision = copy.deepcopy(session['report']['turn'])
                decision.update(motor=1550 if enabled else 1560, servo=motion.servo,
                    phase='drive', stop_requested=False, lock_requested=False)
                # The provider's identity proof is tested with the controller;
                # here isolate the service opt-in and exact-return body gate.
                with patch.object(motion, 'current_contact_target', side_effect=lambda scan, now:
                                  self.evidence(scan), create=True) as provider, \
                     patch.object(motion, 'update', return_value=decision):
                    self.tick(round(self.now+.02, 6), value)
                if enabled:
                    self.assertIs(self.console.auto_session, session)
                    self.assertTrue(provider.called)
                    self.assertEqual(session['report']['target_contact_tolerated_count'], 1)
                    self.assertTrue(session['report']['clearance_current']['body_proximity']['target_contact_tolerated'])
                else:
                    self.assertIsNone(self.console.auto_session)
                    provider.assert_not_called()
                    self.assertEqual(self.outputs[-1]['op'], 'stop')

    def test_service_without_current_target_or_with_wall_still_halts(self):
        for fault in ('target_unknown', 'wall_point', 'forged_identity'):
            with self.subTest(fault=fault):
                self.reset_case()
                _, session = self.enter_compact_orbit(continue_route=True)
                motion = session['turn_motion']
                value = self.contact_scan(self.console.scan['seq']+1)
                if fault == 'wall_point':
                    value['ranges'][0] = .25
                def witness(scan, now):
                    target = self.evidence(scan)
                    if fault == 'target_unknown':
                        return None
                    if fault == 'forged_identity':
                        target['source_seq'] -= 1
                    return target
                with patch.object(motion, 'current_contact_target', side_effect=witness, create=True):
                    self.tick(round(self.now+.02, 6), value)
                self.assertIsNone(self.console.auto_session)
                self.assertEqual(self.outputs[-1]['op'], 'stop')

    def test_service_maintains_real_bound_target_into_contact_then_stops_for_wall(self):
        _, session = self.enter_compact_orbit(continue_route=True)
        original_id = session['turn_motion'].orbit_track_id
        for distance in (.78, .55, .36, .25, .17):
            value = self.compact_scan(self.console.scan['seq']+1, observed=False)
            # Keep a roughly 12 cm observed component as its angular span grows.
            half_span = round(math.degrees(math.atan2(.06, distance)))
            value['ranges'][270-half_span:271+half_span] = [distance]*(2*half_span+1)
            self.observe(round(self.now+.1, 6), value)
            self.assertIs(self.console.auto_session, session, self.console.auto_result)
            self.assertEqual(session['turn_motion'].orbit_track_id, original_id)
        body = session['report']['clearance_current']['body_proximity']
        self.assertTrue(body['target_contact_tolerated'])
        self.assertEqual(body['target_contact_evidence']['track_id'], original_id)
        self.assertLess(body['current_known_min_distance_m'], .04)
        value = copy.deepcopy(value)
        value['seq'] += 1
        value['at_ms'] += 100
        value['ranges'][0] = .25
        self.tick(round(self.now+.02, 6), value)
        self.assertIsNone(self.console.auto_session)
        self.assertEqual(self.outputs[-1]['op'], 'stop')


if __name__ == '__main__':
    unittest.main()
