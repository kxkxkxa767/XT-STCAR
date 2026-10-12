"""Route quality continuity from current scans; no hardware or timing sleeps."""
import copy
import unittest

import test_route_control as route


class RouteQualityContinuityTests(unittest.TestCase):
    setUp = route.RouteControlTests.setUp
    tearDown = route.RouteControlTests.tearDown
    scan = staticmethod(route.RouteControlTests.scan)
    tick = route.RouteControlTests.tick
    observe = route.RouteControlTests.observe
    start_compact_trial = route.RouteControlTests.start_compact_trial
    drive_compact_trial = route.RouteControlTests.drive_compact_trial
    enter_compact_orbit = route.RouteControlTests.enter_compact_orbit
    compact_scan = route.RouteControlTests.compact_scan
    incomplete = route.RouteControlTests.incomplete
    target_bins = route.RouteControlTests.target_bins
    reset_case = route.RouteControlTests.reset_case

    def short_gap(self, seq, *, geometry=True):
        value = self.scan(seq)
        value['ranges'][5:9] = [None]*4
        value['ranges'][9] = 2.  # A real range step, not a bridged surface.
        if not geometry:
            value.update(corridor_candidates=[], wall_candidates=[], left_turn_goal=None)
        return value

    def mixed_gaps(self, seq):
        value = self.compact_scan(seq)
        value['ranges'][5:9] = [None]*4  # Equal adjacent returns: occupied bridge.
        value['ranges'][15:21] = [None]*6  # Six still-unknown bins: neutral only.
        return value

    def test_started_left_turn_uses_current_geometry_for_short_front_gap(self):
        self.drive_compact_trial(continue_route=True)
        session = self.console.auto_session
        self.assertFalse(session['turn_motion'].handover_observed)
        value = self.short_gap(self.console.scan['seq']+1)
        self.tick(round(self.now+.02, 6), value)
        self.assertIs(self.console.auto_session, session)
        self.assertEqual(session['phase'], 'drive')
        self.assertEqual(self.outputs[-1]['motor'], 1560)
        self.assertEqual(session['report']['quality_issues'], [])
        current = session['report']['clearance_current']
        self.assertTrue(current['front_sparse_tolerated'])
        self.assertEqual(current['quality_tolerance_scope'], 'left_turn_current_geometry')
        self.assertEqual(current['front_open_unknown_bins'], [5, 6, 7, 8])

    def test_started_left_turn_uses_existing_incomplete_scan_envelope(self):
        self.drive_compact_trial(continue_route=True)
        session = self.console.auto_session
        value = self.scan(self.console.scan['seq']+1)
        value['ranges'][150:161] = [None]*11
        value['ranges'][200:211] = [None]*11
        self.tick(round(self.now+.02, 6), value)
        self.assertIs(self.console.auto_session, session)
        self.assertEqual(session['phase'], 'drive')
        self.assertEqual(session['report']['quality_issues'], [])
        self.assertTrue(session['report']['clearance_current']['scan_incomplete_tolerated'])

    def test_missing_current_geometry_still_pauses_and_never_refreshes_lease(self):
        self.drive_compact_trial(continue_route=True)
        session = self.console.auto_session
        motion = session['turn_motion']
        old_geometry = (motion.last_geometry_seq, motion.last_geometry_publication,
                        motion.last_geometry_receive, copy.deepcopy(motion.corridor))
        self.tick(round(self.now+.02, 6), self.short_gap(self.console.scan['seq']+1, geometry=False))
        self.assertIs(self.console.auto_session, session)
        self.assertEqual(session['phase'], 'quality_wait')
        self.assertEqual(self.outputs[-1]['motor'], 1500)
        self.assertEqual((motion.last_geometry_seq, motion.last_geometry_publication,
                          motion.last_geometry_receive, motion.corridor), old_geometry)
        self.assertNotIn('front_sparse_tolerated', session['report']['clearance_current'])

    def test_geometry_probe_does_not_mutate_live_adapter(self):
        self.drive_compact_trial(continue_route=True)
        session = self.console.auto_session
        motion = session['turn_motion']
        self.console.scan = self.short_gap(self.console.scan['seq']+1)
        self.console.scan_at = self.now
        before = copy.deepcopy(motion.__dict__)
        # Compare serializable geometry/control state; tracker identities must
        # also stay the original objects rather than being replaced by probes.
        tracker = motion.target_tracker
        self.assertEqual(self.console.turn_route_quality_scope(session, self.now),
                         'left_turn_current_geometry')
        for key in ('last_geometry_seq', 'last_geometry_publication', 'last_geometry_receive',
                    'corridor', 'outer_wall', 'turn_goal', 'entry_bearing_required',
                    'servo', 'steering_target', 'route_events'):
            self.assertEqual(getattr(motion, key), before[key], key)
        self.assertIs(motion.target_tracker, tracker)

    def test_presteer_and_default_mode_do_not_gain_route_tolerance(self):
        self.start_compact_trial(continue_route=True)
        session = self.console.auto_session
        self.assertEqual(session['phase'], 'presteer')
        self.assertIsNone(self.console.turn_route_quality_scope(session, self.now))
        self.tick(round(self.now+.02, 6), self.short_gap(self.console.scan['seq']+1))
        self.assertFalse(any(row['motor'] > 1500 for row in self.outputs))
        self.reset_case()
        self.drive_compact_trial()
        self.tick(round(self.now+.02, 6), self.short_gap(self.console.scan['seq']+1))
        self.assertIsNone(self.console.auto_session)
        self.assertEqual(self.outputs[-1]['op'], 'stop')

    def test_long_unbridged_gap_and_known_body_point_still_stop(self):
        for fault in ('seven_unknown', 'body_close', 'stale_scan'):
            with self.subTest(fault=fault):
                self.reset_case()
                self.drive_compact_trial(continue_route=True)
                value = self.short_gap(self.console.scan['seq']+1)
                if fault == 'seven_unknown':
                    value['ranges'][5:12] = [None]*7
                elif fault == 'body_close':
                    value['ranges'][0] = .27  # Seven cm ahead of the body.
                else:
                    self.console.scan, self.console.scan_at = value, self.now-.31
                self.tick(round(self.now+.02, 6), value)
                self.assertIsNone(self.console.auto_session)
                self.assertEqual(self.outputs[-1]['op'], 'stop')

    def test_bridge_accounting_admits_neutral_wait_across_route_stages(self):
        for stage in ('left_turn', 'first_target', 'right_align', 'corridor_follow',
                      'second_orbit', 'light_turn'):
            with self.subTest(stage=stage):
                self.reset_case()
                if stage == 'left_turn':
                    self.drive_compact_trial(continue_route=True)
                    session = self.console.auto_session
                else:
                    _, session = self.enter_compact_orbit(continue_route=True)
                motion = session['turn_motion']
                if stage not in ('left_turn', 'first_target'):
                    motion.first_pass_evidence = {'source_seq': self.console.scan['seq']}
                    motion.right_exit_center_ack = {'servo': 1500}
                    motion.route_stage = stage
                    motion.right_exit_since = self.now
                self.tick(round(self.now+.02, 6), self.mixed_gaps(self.console.scan['seq']+1))
                self.assertIs(self.console.auto_session, session, self.console.auto_result)
                self.assertEqual(session['phase'], 'quality_wait')
                self.assertEqual(self.outputs[-1]['motor'], 1500)
                current = session['report']['clearance_current']
                self.assertEqual(len(current['front_unknown_bins']), 10)
                self.assertEqual(current['front_bridged_bins'], [5, 6, 7, 8])
                self.assertEqual(current['front_open_unknown_bins'], list(range(15, 21)))
                self.assertFalse(current['motion_ready'])
                self.assertIsNone(session.get('quality_resume_sequence'))
                # The very same unbridged count applies before the first ACK.
                control = self.console.status['control']
                control.update(motor=1560, seq=session['quality_wait_neutral_sequence']-1)
                self.assertTrue(self.console.turn_quality_wait_eligible(session))

    def test_forged_bridge_evidence_does_not_admit_neutral_hold(self):
        _, session = self.enter_compact_orbit(continue_route=True)
        value = self.mixed_gaps(self.console.scan['seq']+1)
        self.console.scan = value
        current = self.server.probe_clearance(value, {'lidar': .01, 'control': .01},
            True, camera_required=False, clearance_profile='maneuver')
        session['report'].update(clearance_current=current, quality_issues=current['quality_issues'])
        self.assertTrue(self.console.turn_quality_hold_eligible(session, route_entry=True))
        for key in ('front_unknown_bins', 'front_bridged_bins', 'front_open_unknown_bins'):
            original = current[key]
            current[key] = []
            self.assertFalse(self.console.turn_quality_hold_eligible(session, route_entry=True), key)
            current[key] = original
        current['scan_seq'] -= 1
        self.assertFalse(self.console.turn_quality_hold_eligible(session, route_entry=True))

    def test_entry_uses_the_same_324_return_floor_as_neutral_wait(self):
        _, session = self.enter_compact_orbit(continue_route=True)
        value = self.compact_scan(self.console.scan['seq']+1)
        value['ranges'][90:127] = [None]*37
        value.update(valid_fraction=1., coverage=1.)
        self.tick(round(self.now+.02, 6), value)
        self.assertIsNone(self.console.auto_session)
        self.assertEqual(self.outputs[-1]['op'], 'stop')
        self.assertFalse(session['turn_motion'].quality_wait_used)

    def test_left_wait_recovers_with_current_geometry_and_bounded_short_gap(self):
        self.drive_compact_trial(continue_route=True)
        session = self.console.auto_session
        self.tick(round(self.now+.02, 6), self.short_gap(self.console.scan['seq']+1, geometry=False))
        self.assertEqual(session['phase'], 'quality_wait')
        start = self.now
        resumed = False
        for _ in range(10):
            self.observe(round(self.now+.1, 6), self.short_gap(self.console.scan['seq']+1))
            self.assertIs(self.console.auto_session, session, self.console.auto_result)
            if session.get('quality_resume_sequence') is not None:
                self.assertGreaterEqual(self.now-start, .3)
                self.assertTrue(session['report']['quality_wait_neutral_ack'])
                resumed = True
                break
            self.assertEqual(self.outputs[-1]['motor'], 1500)
        self.assertTrue(resumed)
        self.assertEqual(sum(row['op'] == 'arm' for row in self.outputs), 1)


if __name__ == '__main__':
    unittest.main()
