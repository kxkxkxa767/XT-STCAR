"""Explicit continuing-route service/CLI gates; synthetic clocks, no hardware."""
import copy
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import test_turn_control as base


class RouteControlTests(unittest.TestCase):
    setUp = base.TurnControlTests.setUp
    tearDown = base.TurnControlTests.tearDown
    scan = staticmethod(base.TurnControlTests.scan)
    tick = base.TurnControlTests.tick
    observe = base.TurnControlTests.observe
    start_compact_trial = base.TurnControlTests.start_compact_trial
    drive_compact_trial = base.TurnControlTests.drive_compact_trial
    enter_compact_orbit = base.TurnControlTests.enter_compact_orbit
    # Wait/resume mechanics below keep the synthetic target ahead of the
    # measured body front (about 80 deg, frontmost ~.24 m), so the trial40
    # release-only branch stays inactive unless a test moves it abeam.
    target_bins = slice(276, 285)

    def compact_scan(self, seq, observed=True, distance=1.):
        value = self.scan(seq)
        value['ranges'] = [3.]*360
        if observed:
            value['ranges'][self.target_bins] = [distance]*9
        return value

    def incomplete(self, seq, front_gap=0):
        value = self.compact_scan(seq)
        value['ranges'][150:170] = [None]*20
        if front_gap:
            value['ranges'][5:5+front_gap] = [None]*front_gap
        return value

    def test_trial50_short_front_gap_keeps_orbit_drive_but_longer_gap_waits(self):
        # Trial50: a 4-bin front null run beside a range step (dropout band)
        # started the wait; <= 5 bins cannot hide a cone closer than ~2.3 m.
        _, session = self.enter_compact_orbit(continue_route=True)
        value = self.compact_scan(self.console.scan['seq']+1)
        value['ranges'][5:9] = [None]*4
        value['ranges'][9] = 2.
        self.observe(round(self.now+.1, 6), value)
        self.assertIs(self.console.auto_session, session)
        self.assertEqual(session['phase'], 'drive')
        self.assertEqual(session['report']['quality_issues'], [])
        self.assertTrue(session['report']['clearance_current']['front_sparse_tolerated'])
        self.assertEqual(session['report']['front_sparse_tolerated_count'], 1)
        value = self.compact_scan(self.console.scan['seq']+1)
        value['ranges'][5:11] = [None]*6
        value['ranges'][11] = 2.
        self.observe(round(self.now+.1, 6), value)
        self.assertIs(self.console.auto_session, session)
        self.assertEqual(session['phase'], 'quality_wait')

    def test_trial50_wait_steering_step_is_accepted_while_unacked(self):
        # Route waits may steer while rolling; the service accepts the one
        # pending step either way and keeps the motor at neutral.
        session, held = self.wait()
        motion = session['turn_motion']
        motion.quality_wait_previous_servo, motion.quality_wait_servo = held, held-20
        control = dict(self.console.status['control'], servo=held, seq=self.console.sequence-1)
        session['turn_servo_sequence'] = self.console.sequence
        self.assertTrue(self.console.turn_wait_servo_matches(session, control))
        motion.quality_wait_servo = held+20
        self.assertTrue(self.console.turn_wait_servo_matches(session, control))
        control['seq'] = self.console.sequence
        self.assertFalse(self.console.turn_wait_servo_matches(session, control))  # ACKed: must equal held
        motion.quality_wait_servo = 1490
        control['seq'] = self.console.sequence-1
        self.assertFalse(self.console.turn_wait_servo_matches(session, control))  # orbit wait never below center

    def test_trial49_tolerable_incomplete_scan_keeps_orbit_drive_after_handover(self):
        # Trial49: 336/337 returns from 9-11 bin open runs started a coasting
        # wait mid-orbit; after handover such a scan keeps the stage steering.
        _, session = self.enter_compact_orbit(continue_route=True)
        value = self.compact_scan(self.console.scan['seq']+1)
        value['ranges'][150:161] = [None]*11
        value['ranges'][200:211] = [None]*11
        self.observe(round(self.now+.1, 6), value)
        self.assertIs(self.console.auto_session, session)
        self.assertEqual(session['phase'], 'drive')
        self.assertEqual(session['report']['quality_issues'], [])
        self.assertTrue(session['report']['clearance_current']['scan_incomplete_tolerated'])
        self.assertEqual(session['report']['scan_incomplete_tolerated_count'], 1)
        self.assertEqual(self.outputs[-1]['motor'], 1550)
        # A 13-bin unknown run is still incomplete and takes the existing wait.
        value = self.compact_scan(self.console.scan['seq']+1)
        value['ranges'][150:163] = [None]*13
        value['ranges'][200:206] = [None]*6
        self.observe(round(self.now+.1, 6), value)
        self.assertIs(self.console.auto_session, session)
        self.assertEqual(session['phase'], 'quality_wait')

    def wait(self):
        _, session = self.enter_compact_orbit(continue_route=True)
        held = self.console.status['control']['servo']
        self.observe(round(self.now+.1, 6), self.incomplete(self.console.scan['seq']+1))
        self.assertIs(self.console.auto_session, session)
        self.assertEqual(session['phase'], 'quality_wait')
        self.assertEqual((self.outputs[-1]['motor'], self.outputs[-1]['servo']), (1500, held))
        self.assertNotIn('coast_deadline', session)
        self.assertNotIn('coast_motion', session['report'].get('quality_wait_snapshot', {}))
        return session, held

    def resume(self):
        session, held = self.wait()
        for _ in range(8):
            self.observe(round(self.now+.1, 6), self.compact_scan(self.console.scan['seq']+1))
            self.assertIs(self.console.auto_session, session)
            if session.get('quality_resume_sequence') is not None:
                break
        self.assertIsNotNone(session.get('quality_resume_sequence'))
        self.assertEqual((self.outputs[-1]['motor'], self.outputs[-1]['servo']), (1560, held))
        return session, held

    def reset_case(self):
        if self.outputs:
            self.tearDown()
            self.setUp()

    def test_route_is_strict_explicit_and_preview_never_arms(self):
        for value in (None, 1, 0, 'true', [], {}):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, 'invalid_continue_route'):
                self.start_compact_trial(continue_route=value)
        with patch.object(self.server.time, 'monotonic', return_value=0.):
            for value in (True, False):
                with self.assertRaisesRegex(ValueError, 'invalid_continue_route'):
                    self.console.autonomy_command({'op': 'turn_left_start', 'boot': self.console.boot,
                        'epoch': 0, 'tick': 1000, 'placement_confirmed': True, 'continue_route': value})
            preview = self.console.state(trial_mode='turn-cone', continue_route=True)['autonomy']
            default = self.console.state(trial_mode='turn-cone')['autonomy']
        self.assertIs(preview['continue_route'], True)
        self.assertIs(preview['turn_preview']['continue_route'], True)
        self.assertIs(default['continue_route'], False)
        self.assertEqual(self.outputs, [])
        self.assertIsNone(self.console.owner)

    def test_http_route_query_requires_one_literal_bool_and_turn_cone_mode(self):
        captured, responses = {}, []
        cases = [('trial_mode=turn-cone&continue_route=true', 200),
                 ('trial_mode=turn-cone&continue_route=false', 200),
                 ('continue_route=true', 400), ('trial_mode=turn-left&continue_route=false', 400),
                 ('trial_mode=turn-cone&continue_route=1', 400),
                 ('trial_mode=turn-cone&continue_route=True', 400),
                 ('trial_mode=turn-cone&continue_route=', 400),
                 ('trial_mode=turn-cone&continue_route=true&continue_route=false', 400)]
        class FakeServer:
            def __init__(self, address, handler): captured['handler'] = handler
            def server_close(self): pass
        def query():
            for suffix, expected in cases:
                handler = object.__new__(captured['handler'])
                handler.path = '/api/state?'+suffix
                handler.headers = {'X-Control-Token': self.console.token}
                handler.reply = lambda status, body, content=None: responses.append((status, body))
                handler.do_GET()
                self.assertEqual(responses[-1][0], expected)
            self.console.stop.set()
        args = base.types.SimpleNamespace(bind='127.0.0.1', lan_bind=None, port=8081,
                                          access_file=str(Path(self.temp.name)/'access.json'))
        with patch.object(self.server, 'Console', return_value=self.console),\
             patch.object(self.server, 'ThreadingHTTPServer', FakeServer),\
             patch.object(self.console, 'start', side_effect=query), patch.object(self.console, 'close'),\
             patch.object(self.server.signal, 'signal'), patch.object(self.server.time, 'monotonic', return_value=0.),\
             patch.object(sys, 'stdout', io.StringIO()):
            self.server.serve(args)
        self.assertIs(responses[0][1]['autonomy']['continue_route'], True)
        self.assertIs(responses[1][1]['autonomy']['continue_route'], False)
        self.assertEqual(self.outputs, [])

    def test_wait_resume_keeps_one_arm_original_budgets_and_exact_bounded_evidence(self):
        session, held = self.wait()
        motion = session['turn_motion']
        original = (session['deadline'], motion.drive_since, motion.orbit_since, self.console.arm_sequence,
                    self.console.owner, self.console.control_epoch)
        snapshot = copy.deepcopy(session['report']['quality_wait_snapshot'])
        self.assertEqual(snapshot['scan']['ranges'], self.console.scan['ranges'])
        self.assertEqual(snapshot['scan']['received_at'], self.console.scan_at)
        self.assertEqual(snapshot['control']['motor'], 1550)  # orbit at the slow motor
        self.assertTrue(snapshot['control']['command_acked'])
        for _ in range(8):
            self.observe(round(self.now+.1, 6), self.compact_scan(self.console.scan['seq']+1))
            self.assertIs(self.console.auto_session, session)
            if 'quality_resume_snapshot' in session['report']:
                break
        self.assertIn('quality_resume_snapshot', session['report'])
        recovered = session['report']['quality_resume_snapshot']
        self.assertEqual(recovered['control']['motor'], 1560)
        self.assertIs(recovered['control']['resume_acked'], True)
        self.assertEqual(recovered['control']['servo'], held)
        self.assertEqual(snapshot, session['report']['quality_wait_snapshot'])
        self.assertEqual(original, (session['deadline'], motion.drive_since, motion.orbit_since,
            self.console.arm_sequence, self.console.owner, self.console.control_epoch))
        self.assertEqual(sum(row['op'] == 'arm' for row in self.outputs), 1)
        self.assertFalse(session['report']['completed'])
        self.assertFalse(session['report']['competition_supported'])

    def test_second_quality_loss_starts_another_neutral_wait_without_rearming(self):
        # Self-recovery: no single-use limit; budgets/deadline still bound it.
        session, held = self.resume()
        self.observe(round(self.now+.1, 6), self.compact_scan(self.console.scan['seq']+1))
        servo = self.console.status['control']['servo']
        self.observe(round(self.now+.1, 6), self.incomplete(self.console.scan['seq']+1))
        self.assertIs(self.console.auto_session, session)
        self.assertEqual(session['phase'], 'quality_wait')
        self.assertEqual(session['turn_motion'].recovery_count, 2)
        self.assertEqual((self.outputs[-1]['motor'], self.outputs[-1]['servo']), (1500, servo))
        self.assertNotIn('quality_coast_trigger', session['report'])
        self.assertEqual(sum(row['op'] == 'arm' for row in self.outputs), 1)

    def test_partial_target_trigger_frame_still_starts_the_neutral_wait(self):
        # Trial41/47: the incomplete trigger frame hid the support edge; the
        # previous fresh full isolation anchors the wait instead of a coast.
        _, session = self.enter_compact_orbit(continue_route=True)
        servo = self.console.status['control']['servo']
        value = self.incomplete(self.console.scan['seq']+1)
        # Nine edge nulls: beyond the confirmed-maintenance allowance (8, round 55).
        value['ranges'][self.target_bins.start-9:self.target_bins.start] = [None]*9
        self.observe(round(self.now+.1, 6), value)
        self.assertIs(self.console.auto_session, session)
        self.assertEqual(session['phase'], 'quality_wait')
        self.assertEqual(session['turn_motion'].quality_wait_anchor['basis'], 'previous_fresh_full_isolation')
        self.assertNotIn('quality_coast_trigger', session['report'])
        self.assertEqual((self.outputs[-1]['motor'], self.outputs[-1]['servo']), (1500, servo))
        self.assertEqual(sum(row['op'] == 'arm' for row in self.outputs), 1)

    def test_refused_wait_with_unacked_step_still_halts(self):
        _, session = self.enter_compact_orbit(continue_route=True)
        motion = session['turn_motion']
        self.assertFalse(motion.begin_route_quality_coast(self.now,
            {**self.console.status['control'], 'command_acked': False}))
        self.assertFalse(motion.begin_route_quality_coast(self.now,
            {**self.console.status['control'], 'servo': motion.servo-10, 'command_acked': True}))
        self.assertEqual(motion.phase, 'drive')

    def test_front_sparse_only_extends_actual_neutral_wait_and_never_restores(self):
        session, held = self.wait()
        # 20 unknown elsewhere plus 16 front unknown leaves exactly the
        # existing 324-return neutral floor. This never passes drive quality.
        # Gaps up to FRONT_BRIDGE_MAX_BINS between equal returns are bridged
        # (maneuver profile); these stay unbridged front_sparse.
        for front_gap in (6, 7, 12, 16):
            value = self.incomplete(self.console.scan['seq']+1, front_gap=front_gap)
            value.update(coverage=0., valid_fraction=0.)  # Not admission evidence.
            self.observe(round(self.now+.1, 6), value)
            self.assertIs(self.console.auto_session, session)
            self.assertEqual((self.outputs[-1]['motor'], self.outputs[-1]['servo']), (1500, held))
            self.assertIsNone(session.get('quality_resume_sequence'))
            self.assertFalse(session['report']['clearance_current']['motion_ready'])
            self.assertIn('front_sparse', session['report']['quality_issues'])
        self.observe(round(self.now+.1, 6), self.incomplete(self.console.scan['seq']+1, front_gap=17))
        self.assertIsNone(self.console.auto_session)
        self.assertEqual(self.outputs[-1]['op'], 'stop')

    def test_actual35_668_only_changes_neutral_wait_eligibility_not_drive_quality(self):
        session, _ = self.wait()
        fixture = json.loads((Path(__file__).with_name('fixtures')/
            'left-cone-35-neutral-wait-front-gap.json').read_text())
        value = copy.deepcopy(fixture['scan'])
        self.assertEqual(value['seq'], 668)
        self.assertEqual(fixture['prior_controller_target_summary']['source_seq'], 667)
        self.assertEqual(sum(r is not None for r in value['ranges']), 337)
        # Only eligibility is evaluated here. The recorded target667 summary
        # is not fabricated into controller target668 or a full predecessor scan.
        self.console.scan = value
        control = copy.deepcopy(fixture['observed_control'])
        self.console.status['control'] = control
        session['quality_wait_neutral_sequence'] = control['seq']  # Mock prior command ACK boundary.
        session['turn_motion'].quality_wait_servo = control['servo']
        clearance = self.server.probe_clearance(value, {'camera': .01, 'lidar': .01, 'control': .01},
            True, camera_required=False, clearance_profile='maneuver')
        session['report'].update(clearance_current=clearance, quality_issues=clearance['quality_issues'])
        self.assertEqual(clearance['front_unknown_bins'], list(range(-30, -18)))
        self.assertEqual(clearance['quality_issues'], ['scan_incomplete', 'front_sparse'])
        self.assertFalse(clearance['motion_ready'])
        before = len(self.outputs)
        self.assertTrue(self.console.turn_quality_wait_eligible(session))
        self.assertEqual(len(self.outputs), before)
        # Round 55 (Claude): a still-pending neutral may keep waiting on any
        # evidence-matched front gap (the alternative halt is the same neutral,
        # recentered); power after the neutral ACK never counts as waiting.
        control['motor'] = 1560
        session['quality_wait_neutral_sequence'] = control['seq']+1
        self.assertTrue(self.console.turn_quality_wait_eligible(session))
        control['motor'] = 1500
        session['quality_wait_neutral_sequence'] = control['seq']
        for fault in ('old_clearance', 'false_unknowns', 'power_after_neutral_ack', 'drive'):
            with self.subTest(fault=fault):
                original = copy.deepcopy(clearance)
                if fault == 'old_clearance': clearance['scan_seq'] -= 1
                if fault == 'false_unknowns': clearance['front_unknown_bins'] = []
                if fault == 'power_after_neutral_ack':
                    control['motor'] = 1560
                if fault == 'drive': session['phase'] = 'drive'
                self.assertFalse(self.console.turn_quality_wait_eligible(session))
                clearance.clear(); clearance.update(original)
                control['motor'] = 1500
                session['quality_wait_neutral_sequence'] = control['seq']
                session['phase'] = 'quality_wait'

    def test_neutral_wait_floor_rejects_323_returns_even_without_front_sparse_and_blindness(self):
        for fault in ('323_elsewhere', 'front_blind', 'all_blind'):
            with self.subTest(fault=fault):
                self.reset_case()
                self.wait()
                value = self.compact_scan(self.console.scan['seq']+1)
                if fault == '323_elsewhere': value['ranges'][90:127] = [None]*37
                if fault == 'front_blind':
                    for angle in range(-30, 31): value['ranges'][angle % 360] = None
                if fault == 'all_blind': value['ranges'] = [None]*360
                value.update(valid_fraction=1., coverage=1.)  # Cannot hide actual missing returns.
                before = len(self.outputs)
                self.observe(round(self.now+.1, 6), value)
                self.assertIsNone(self.console.auto_session)
                self.assertEqual(self.outputs[-1]['op'], 'stop')
                self.assertFalse(any(row['motor'] > 1500 for row in self.outputs[before:]))

    def test_neutral_wait_ranges_keep_original_finite_range_definition(self):
        session, _ = self.wait()
        value = self.incomplete(self.console.scan['seq']+1, front_gap=12)
        clearance = self.server.probe_clearance(value, {'camera': .01, 'lidar': .01, 'control': .01},
            True, camera_required=False, clearance_profile='maneuver')
        self.console.scan = value
        session['report'].update(clearance_current=clearance, quality_issues=clearance['quality_issues'])
        original = value['ranges'][130]
        for invalid in (False, True, '1.0', float('nan'), float('inf'), .019, 12.001):
            with self.subTest(invalid=invalid):
                value['ranges'][130] = invalid
                self.assertFalse(self.console.turn_quality_wait_eligible(session))
        for valid in (.02, 12., original):
            value['ranges'][130] = valid
            self.assertTrue(self.console.turn_quality_wait_eligible(session))

    def test_front_sparse_wait_still_requires_full_stable_quality_before_one_resume(self):
        session, held = self.wait()
        for _ in range(3):
            self.observe(round(self.now+.1, 6), self.incomplete(self.console.scan['seq']+1, front_gap=12))
            self.assertIs(self.console.auto_session, session)
            self.assertEqual((self.outputs[-1]['motor'], self.outputs[-1]['servo']), (1500, held))
        clear_since = round(self.now+.1, 6)
        for _ in range(8):
            self.observe(round(self.now+.1, 6), self.compact_scan(self.console.scan['seq']+1))
            self.assertIs(self.console.auto_session, session)
            if session.get('quality_resume_sequence') is not None:
                self.assertGreaterEqual(self.now-clear_since+1e-9, .30)
                break
            self.assertEqual(self.outputs[-1]['motor'], 1500)
        self.assertIsNotNone(session.get('quality_resume_sequence'))
        self.assertEqual((self.outputs[-1]['motor'], self.outputs[-1]['servo']), (1560, held))
        self.assertEqual(sum(row['op'] == 'arm' for row in self.outputs), 1)

    def test_front_sparse_from_drive_and_unacked_or_forged_neutral_cannot_wait(self):
        # Front_sparse from drive now takes the held-steering motor cut (trial42),
        # never the wait; see the RouteQualityCoast tests below.
        for fault in ('missing_sequence', 'unacked', 'positive', 'servo_changed', 'bridge_locked'):
            with self.subTest(fault=fault):
                self.reset_case()
                session, _ = self.wait()
                control = self.console.status['control']
                if fault == 'missing_sequence': session['quality_wait_neutral_sequence'] = None
                if fault == 'unacked': session['quality_wait_neutral_sequence'] = control['seq']+1
                if fault == 'positive': control['motor'] = 1560
                if fault == 'servo_changed': control['servo'] -= 1
                if fault == 'bridge_locked': control['armed'] = False
                session['report']['quality_wait_neutral_ack'] = True
                self.tick(round(self.now+.02, 6), self.incomplete(self.console.scan['seq']+1, front_gap=5))
                self.assertIsNone(self.console.auto_session)
                self.assertEqual(self.outputs[-1]['op'], 'stop')

    def front_sparse(self, seq, gap, base=None):
        value = self.scan(seq) if base is None else base
        value['ranges'][5:5+gap] = [None]*gap
        return value

    def assert_held_coast(self, session, servo, frames):
        for value in frames:
            self.observe(round(self.now+.1, 6), value)
            self.assertIs(self.console.auto_session, session, self.console.auto_result)
            self.assertEqual(session['phase'], 'coast')
            self.assertEqual((self.outputs[-1]['motor'], self.outputs[-1]['servo']), (1500, servo))
        self.assertFalse(any(row.get('motor', 1500) > 1500 for row in self.outputs[self.coast_start:]))
        self.assertEqual(sum(row['op'] == 'arm' for row in self.outputs), 1)

    def left_turn_pause(self):
        # A six-bin gap exceeds bounded live-geometry tolerance before handover.
        self.drive_compact_trial(continue_route=True)
        session = self.console.auto_session
        motion = session['turn_motion']
        self.assertFalse(motion.handover_observed)
        servo = self.console.status['control']['servo']
        self.assertGreater(servo, 1500)
        self.observe(round(self.now+.1, 6), self.front_sparse(self.console.scan['seq']+1, 6))
        self.assertIs(self.console.auto_session, session)
        self.assertEqual(session['phase'], 'quality_wait')
        self.assertEqual(motion.phase, 'quality_wait')
        self.assertTrue(motion.left_turn_pause_used)
        self.assertFalse(motion.quality_wait_used)  # the orbit wait stays available
        trigger = session['report']['left_turn_pause_trigger']
        self.assertEqual(trigger['quality_issues'], ['front_sparse'])
        self.assertEqual(trigger['front_unknown_bins'], [5, 6, 7, 8, 9, 10])
        self.assertNotIn('quality_coast_trigger', session['report'])
        self.assertEqual((self.outputs[-1]['motor'], self.outputs[-1]['servo']), (1500, servo))
        return session, motion, servo

    def test_left_turn_front_sparse_pauses_then_resumes_with_held_left(self):
        session, motion, servo = self.left_turn_pause()
        pause_start = len(self.outputs)
        resumed = None
        for i in range(1, 9):
            self.observe(round(self.now+.1, 6), self.scan(self.console.scan['seq']+1))
            self.assertIs(self.console.auto_session, session, self.console.auto_result)
            if session.get('quality_resume_sequence') is not None and resumed is None:
                resumed = len(self.outputs)-1
            if resumed is not None and not motion.quality_resume_pending:
                break
        self.assertIsNotNone(resumed)
        # Neutral, unchanged steering until clean quality and current geometry
        # matured; the first powered output still holds the paused command.
        self.assertTrue(all((row['motor'], row['servo']) == (1500, servo)
                            for row in self.outputs[pause_start:resumed]))
        self.assertEqual((self.outputs[resumed]['motor'], self.outputs[resumed]['servo']), (1560, servo))
        self.assertFalse(motion.quality_resume_pending)
        self.assertEqual(motion.phase, 'drive')
        self.assertIn('resume_ack', motion.left_turn_pause_evidence)
        self.assertIn('left_turn_pause_resume_snapshot', session['report'])
        self.assertNotIn('quality_resume_snapshot', session['report'])
        self.assertEqual(sum(row['op'] == 'arm' for row in self.outputs), 1)
        # Self-recovery: the next bounded fault pauses again (no single use).
        held = self.console.status['control']['servo']
        self.observe(round(self.now+.1, 6), self.front_sparse(self.console.scan['seq']+1, 6))
        self.assertEqual(session['phase'], 'quality_wait')
        self.assertEqual(motion.recovery_count, 2)
        self.assertEqual((self.outputs[-1]['motor'], self.outputs[-1]['servo']), (1500, held))

    def test_left_turn_pause_never_resumes_without_geometry_and_only_cumulative_budget_ends_it(self):
        session, motion, servo = self.left_turn_pause()
        start = self.now
        alive_past_old_pause_clock = False
        while self.console.auto_session is session and self.now < start+20:
            bare = self.scan(self.console.scan['seq']+1)
            bare.update(corridor_candidates=[], wall_candidates=[], left_turn_goal=None)
            self.observe(round(self.now+.1, 6), bare)
            if self.console.auto_session is session:
                self.assertIsNone(session.get('quality_resume_sequence'))
                alive_past_old_pause_clock |= self.now > start+5.5
        # Trial50: no 5 s pause clock; the cumulative drive budget ends it.
        self.assertTrue(alive_past_old_pause_clock)
        self.assertIsNone(self.console.auto_session)
        self.assertIn(self.console.auto_result['reason'], ('route_left_turn_pause_original_budget_expired',
                                                           'maneuver_cumulative_drive_timeout'))
        self.assertFalse(any(row.get('motor', 1500) > 1500 for row in self.outputs[-40:]))

    def test_orbit_front_sparse_uses_single_neutral_wait(self):
        _, session = self.enter_compact_orbit(continue_route=True)
        servo = self.console.status['control']['servo']
        self.observe(round(self.now+.1, 6), self.incomplete(self.console.scan['seq']+1, front_gap=5))
        self.assertEqual(session['phase'], 'quality_wait')
        self.assertTrue(session['turn_motion'].quality_wait_used)
        self.assertEqual(session['quality_wait_kind'], 'orbit_wait')
        self.assertEqual((self.outputs[-1]['motor'], self.outputs[-1]['servo']), (1500, servo))
        for i in (1, 2):
            self.observe(round(self.now+.1, 6), self.incomplete(self.console.scan['seq']+1, front_gap=5))
            self.assertIs(self.console.auto_session, session)
            self.assertEqual((self.outputs[-1]['motor'], self.outputs[-1]['servo']), (1500, servo))

    def test_route_front_sparse_above_count_limit_waits_without_power(self):
        # Round 55 (Claude): explicit route, trial52 scan399 class. A front gap
        # above the old count limit enters the neutral wait (power off, steering
        # held) instead of a recentering halt; it never powers until clean.
        self.drive_compact_trial(continue_route=True)
        session = self.console.auto_session
        servo = self.console.status['control']['servo']
        for _ in range(4):
            self.observe(round(self.now+.1, 6), self.front_sparse(self.console.scan['seq']+1, 7))
            self.assertIs(self.console.auto_session, session)
            self.assertEqual(session['phase'], 'quality_wait')
            self.assertEqual((self.outputs[-1]['motor'], self.outputs[-1]['servo']), (1500, servo))
        self.assertEqual(sum(row['op'] == 'arm' for row in self.outputs), 1)

    def test_default_sequence_front_sparse_in_drive_still_halts(self):
        self.drive_compact_trial()
        self.observe(round(self.now+.1, 6), self.front_sparse(self.console.scan['seq']+1, 4))
        self.assertIsNone(self.console.auto_session)
        self.assertEqual(self.outputs[-1]['op'], 'stop')

    def test_route_coast_with_unacked_step_still_halts(self):
        self.drive_compact_trial(continue_route=True)
        motion = self.console.auto_session['turn_motion']
        self.assertFalse(motion.begin_route_quality_coast(self.now,
            {**self.console.status['control'], 'command_acked': False}))
        self.assertEqual(motion.phase, 'drive')

    def test_wait_hard_faults_stop_and_original_deadline_wins_over_recovery(self):
        for fault in ('body_near', 'control_stale', 'lidar_stale', 'loop_gap', 'heartbeat',
                      'operator_stop', 'total_deadline', 'drive_deadline'):
            with self.subTest(fault=fault):
                self.reset_case()
                session, _ = self.wait()
                value = self.compact_scan(self.console.scan['seq']+1)
                now = round(self.now+.02, 6)
                if fault == 'body_near': value['ranges'][0] = .21
                if fault in ('control_stale', 'lidar_stale'):
                    key = fault.split('_')[0]
                    self.console.sensor_ages = lambda now: {'camera': .01, 'lidar': .01, 'control': .01, key: .31}
                if fault == 'total_deadline': session['deadline'] = now
                if fault == 'drive_deadline': session['turn_motion'].drive_since = now-session['turn_motion'].max_drive_s
                if fault == 'operator_stop': self.console.command({'op': 'stop'})
                elif fault == 'heartbeat':
                    self.console.owner_at = now-self.server.HEARTBEAT_S
                    self.console.autonomy_tick(now)
                else:
                    self.tick(round(self.now+.121, 6) if fault == 'loop_gap' else now, value)
                self.assertIsNone(self.console.auto_session)
                self.assertEqual((self.outputs[-1]['op'], self.outputs[-1]['motor']), ('stop', 1500))
                self.assertEqual(sum(row['op'] == 'arm' for row in self.outputs), 1)

    def test_invalid_wait_or_resume_output_is_rejected_before_emit(self):
        for fault in ('powered_wait', 'wrong_resume_servo'):
            with self.subTest(fault=fault):
                self.reset_case()
                session, held = self.wait()
                decision = copy.deepcopy(session['report']['turn'])
                decision.update(stop_requested=False, lock_requested=False)
                if fault == 'powered_wait': decision.update(phase='quality_wait', motor=1560, servo=held)
                else: decision.update(phase='drive', motor=1560, servo=held-1, quality_resume_pending=True)
                before = len(self.outputs)
                with patch.object(session['turn_motion'], 'update', return_value=decision):
                    self.tick(round(self.now+.02, 6), self.incomplete(self.console.scan['seq']+1))
                self.assertIsNone(self.console.auto_session)
                self.assertEqual([row['op'] for row in self.outputs[before:]], ['stop'])

    def test_healthy_frames_without_neutral_ack_never_restore(self):
        session, _ = self.wait()
        # A forged report cannot make current feedback acknowledge our command.
        session['report']['quality_wait_neutral_ack'] = True
        session['quality_wait_neutral_sequence'] = self.console.sequence+1000
        self.console.status['control']['motor'] = 1560
        before = len(self.outputs)
        self.tick(round(self.now+.02, 6), self.compact_scan(self.console.scan['seq']+1))
        self.assertFalse(any(row['op'] == 'drive' and row['motor'] == 1560 for row in self.outputs[before:]))

    def test_resume_pending_never_accepts_bridge_lock_or_changed_adoption(self):
        for fault in ('locked', 'arm_seq_lost', 'servo_changed', 'invalid_motor'):
            with self.subTest(fault=fault):
                self.reset_case()
                session, _ = self.resume()
                control = self.console.status['control']
                if fault == 'locked': control['armed'] = False
                if fault == 'arm_seq_lost': control['seq'] = self.console.arm_sequence-1
                if fault == 'servo_changed': control['servo'] -= 1
                if fault == 'invalid_motor': control['motor'] = 1550
                before = len(self.outputs)
                self.tick(round(self.now+.02, 6), self.compact_scan(self.console.scan['seq']+1))
                self.assertIsNone(self.console.auto_session)
                self.assertEqual([row['op'] for row in self.outputs[before:]], ['stop'])
                # Once halted, a later healthy/armed feedback cannot restore it.
                control.update(armed=True, motor=1500)
                self.tick(round(self.now+.02, 6), self.compact_scan(self.console.scan['seq']+1))
                self.assertEqual([row['op'] for row in self.outputs[before:]], ['stop'])

    def test_resume_waits_for_actual_ack_and_times_out_without_steering_or_rearm(self):
        session, held = self.resume()
        control = self.console.status['control']
        control.update(seq=session['quality_resume_sequence']-1, motor=1500, servo=held)
        session['report']['quality_resume_ack'] = True
        original_emit = self.console.emit
        def no_adoption(*args, **kwargs):
            previous = dict(control)
            original_emit(*args, **kwargs)
            if args[0] == 'drive':
                control.update(previous)
        self.console.emit = no_adoption
        before = len(self.outputs)
        for _ in range(5):
            self.observe(round(self.now+.1, 6), self.compact_scan(self.console.scan['seq']+1))
            if self.console.auto_session is None:
                break
            self.assertEqual(self.outputs[-1]['servo'], held)
            self.assertIs(session['report']['quality_resume_ack'], False)
        self.assertIsNone(self.console.auto_session)
        self.assertEqual(self.console.auto_result['reason'], 'route_quality_resume_ack_timeout')
        self.assertNotIn('quality_resume_snapshot', self.console.auto_result)
        self.assertTrue(all(row['servo'] == held for row in self.outputs[before:] if row['op'] == 'drive'))
        self.assertEqual(sum(row['op'] == 'arm' for row in self.outputs), 1)

    def test_current_target_identity_and_full_boundary_are_required_to_resume(self):
        # A missing/changed/partial support holds the neutral wait (no halt,
        # no recentering) and never restores power while it lasts.
        for fault in ('missing', 'changed', 'partial'):
            with self.subTest(fault=fault):
                self.reset_case()
                session, held = self.wait()
                before = len(self.outputs)
                for _ in range(4):
                    value = self.compact_scan(self.console.scan['seq']+1,
                        observed=fault != 'missing', distance=2. if fault == 'changed' else 1.)
                    if fault == 'partial':  # beyond the 8-bin maintenance allowance (round 55)
                        value['ranges'][self.target_bins.start-9:self.target_bins.start] = [None]*9
                    self.observe(round(self.now+.1, 6), value)
                    self.assertIs(self.console.auto_session, session)
                    self.assertEqual(session['phase'], 'quality_wait')
                self.assertIsNone(session.get('quality_resume_sequence'))
                self.assertTrue(all((row['motor'], row['servo']) == (1500, held) for row in self.outputs[before:]))

    def test_duplicate_reordered_or_old_received_scans_cannot_mature_recovery(self):
        for fault in ('duplicate', 'reordered', 'old_received'):
            with self.subTest(fault=fault):
                self.reset_case()
                session, held = self.wait()
                self.observe(round(self.now+.1, 6), self.compact_scan(self.console.scan['seq']+1))
                before = len(self.outputs)
                if fault == 'duplicate':
                    for _ in range(10):
                        self.tick(round(self.now+.02, 6), self.console.scan)
                    self.assertIs(self.console.auto_session, session)
                elif fault == 'reordered':
                    self.tick(round(self.now+.02, 6), self.compact_scan(self.console.scan['seq']-1))
                    self.assertIsNone(self.console.auto_session)
                else:
                    self.console.scan_at = self.now-.31
                    self.tick(round(self.now+.02, 6), self.console.scan)
                    self.assertIsNone(self.console.auto_session)
                self.assertFalse(any(row['motor'] > 1500 for row in self.outputs[before:]))

    def test_target_loss_in_orbit_holds_neutral_then_resumes_once_reacquired(self):
        # Self-recovery: the orbit target vanishes for a frame; hold at neutral,
        # re-bind the same support (new ID at the same place), then resume once.
        _, session = self.enter_compact_orbit(continue_route=True)
        held = self.console.status['control']['servo']
        # One missing frame (tolerant tracker hold) keeps driving the command.
        self.observe(round(self.now+.1, 6), self.compact_scan(self.console.scan['seq']+1, observed=False))
        self.assertEqual(session['phase'], 'drive')
        self.assertEqual(session['turn_motion'].reason, 'first_target_missing_frame_hold')
        self.assertEqual(self.outputs[-1]['motor'], 1550)
        # Missing beyond the .30 s lease starts the recovery wait.
        for _ in range(4):
            held = self.console.status['control']['servo']
            self.observe(round(self.now+.1, 6), self.compact_scan(self.console.scan['seq']+1, observed=False))
            if session['phase'] == 'quality_wait':
                break
        self.assertEqual(session['phase'], 'quality_wait')
        self.assertEqual(session['quality_wait_kind'], 'orbit_wait')
        self.assertEqual(session['turn_motion'].reason, 'route_orbit_target_recovery_wait')
        self.assertEqual((self.outputs[-1]['motor'], self.outputs[-1]['servo']), (1500, held))
        for _ in range(12):
            self.observe(round(self.now+.1, 6), self.compact_scan(self.console.scan['seq']+1))
            if session.get('quality_resume_sequence') is not None:
                break
        self.assertIsNotNone(session.get('quality_resume_sequence'))
        self.assertIsNotNone(session['turn_motion'].quality_wait_rebind)
        self.assertEqual(sum(row['op'] == 'arm' for row in self.outputs), 1)

    def test_route_right_permission_needs_phase_and_continuous_actual_center_proof(self):
        for fault in ('no_center', 'wrong_center_servo', 'future_center_seq', 'wrong_stage', 'feedback_left', 'valid'):
            with self.subTest(fault=fault):
                self.reset_case()
                _, session = self.enter_compact_orbit(continue_route=True)
                motion = session['turn_motion']
                motion.first_pass_evidence = {'source_seq': self.console.scan['seq']}
                motion.right_exit_center_ack = {'servo': 1500}
                motion.route_stage = 'right_align'
                if fault != 'feedback_left': self.console.status['control']['servo'] = 1500
                decision = copy.deepcopy(session['report']['turn'])
                proof = {**self.console.status['control'], 'servo': 1500}
                if fault == 'no_center': proof = None
                if fault == 'wrong_center_servo': proof['servo'] = 1670
                if fault == 'future_center_seq': proof['seq'] += 1000
                decision.update(servo=1490, motor=1560, phase='drive', route_stage=(
                    'first_target' if fault == 'wrong_stage' else 'right_align'),
                    route_right_output_authorized=True, route_center_ack=proof,
                    quality_resume_pending=False, stop_requested=False, lock_requested=False)
                before = len(self.outputs)
                with patch.object(motion, 'update', return_value=decision):
                    self.tick(round(self.now+.02, 6), self.compact_scan(self.console.scan['seq']+1))
                if fault == 'valid':
                    self.assertIs(self.console.auto_session, session)
                    self.assertEqual((self.outputs[-1]['motor'], self.outputs[-1]['servo']), (1560, 1490))
                else:
                    self.assertIsNone(self.console.auto_session)
                    self.assertEqual([row['op'] for row in self.outputs[before:]], ['stop'])

    def test_slow_motor_only_after_first_pass_including_light_turn(self):
        # Round 53: also the first-target orbit after handover; never the left turn.
        cases = (('right_align', 1500, True), ('light_turn', 1500, True), ('light_turn', 1480, True),
                 ('first_target', 1600, True), ('left_turn', 1600, False))
        for stage, servo, accepted in cases:
            with self.subTest(stage=stage, servo=servo):
                self.reset_case()
                _, session = self.enter_compact_orbit(continue_route=True)
                motion = session['turn_motion']
                if stage == 'left_turn':
                    motion.handover_observed, stage = False, 'first_target'
                if stage != 'first_target':
                    motion.first_pass_evidence = {'source_seq': self.console.scan['seq']}
                    motion.right_exit_center_ack = {'servo': 1500}
                motion.route_stage = stage
                self.console.status['control']['servo'] = 1500
                decision = copy.deepcopy(session['report']['turn'])
                proof = {**self.console.status['control'], 'servo': 1500, 'motor': 1550}
                decision.update(servo=servo, motor=1550, phase='drive', route_stage=stage,
                    route_right_output_authorized=servo < 1500, route_center_ack=proof,
                    quality_resume_pending=False, stop_requested=False, lock_requested=False)
                before = len(self.outputs)
                with patch.object(motion, 'update', return_value=decision):
                    self.tick(round(self.now+.02, 6), self.compact_scan(self.console.scan['seq']+1))
                if accepted:
                    self.assertIs(self.console.auto_session, session)
                    self.assertEqual((self.outputs[-1]['motor'], self.outputs[-1]['servo']), (1550, servo))
                else:
                    self.assertIsNone(self.console.auto_session)
                    self.assertEqual([row['op'] for row in self.outputs[before:]], ['stop'])

    def test_post_pass_scan_incomplete_pauses_instead_of_halting(self):
        _, session = self.enter_compact_orbit(continue_route=True)
        motion = session['turn_motion']
        motion.first_pass_evidence = {'source_seq': self.console.scan['seq']}
        motion.right_exit_center_ack = {'servo': 1500}
        motion.route_stage, motion.right_exit_since = 'right_align', self.now
        self.observe(round(self.now+.1, 6), self.incomplete(self.console.scan['seq']+1))
        self.assertIs(self.console.auto_session, session)
        self.assertEqual(session['phase'], 'quality_wait')
        self.assertEqual(session['quality_wait_kind'], 'post_pass_pause')
        self.assertEqual(self.outputs[-1]['motor'], 1500)
        self.assertIn('post_pass_pause_trigger', session['report'])

    def test_post_pass_pause_waits_briefly_for_an_in_flight_steering_ack(self):
        # Round 76: the bad frame arrived while the last 40 PWM step was
        # pending; the pause needs an ACKed command and the route halted.
        _, session = self.enter_compact_orbit(continue_route=True)
        motion = session['turn_motion']
        motion.first_pass_evidence = {'source_seq': self.console.scan['seq']}
        motion.right_exit_center_ack = {'servo': 1500}
        motion.route_stage, motion.right_exit_since = 'right_align', self.now
        session['turn_servo_sequence'] = self.console.sequence+1000  # never ACKed here
        session['turn_servo_sent_at'] = round(self.now+.05, 6)
        self.observe(round(self.now+.1, 6), self.incomplete(self.console.scan['seq']+1))
        self.assertIs(self.console.auto_session, session)
        self.assertEqual(session['phase'], 'drive')
        self.assertIn('quality_pause_ack_wait', session['report'])
        self.observe(round(self.now+.2, 6), self.incomplete(self.console.scan['seq']+1))
        self.assertIsNone(self.console.auto_session)  # grace over, still un-ACKed: the old halt
        self.assertEqual(self.console.auto_result['reason'], 'turn_perception_unavailable')

    def test_route_later_stage_and_resume_pending_use_full_eight_cm_body_gate(self):
        for stage in ('right_align', 'corridor_follow', 'second_orbit', 'resume_pending'):
            with self.subTest(stage=stage):
                self.reset_case()
                _, session = self.enter_compact_orbit(continue_route=True)
                motion = session['turn_motion']
                if stage == 'resume_pending': motion.quality_resume_pending = True
                else: motion.route_stage = stage
                value = self.compact_scan(self.console.scan['seq']+1)
                value['ranges'][270] = .20  # 6 cm from measured left body boundary.
                before = len(self.outputs)
                self.tick(round(self.now+.02, 6), value)
                self.assertIsNone(self.console.auto_session)
                self.assertIn('body', self.console.auto_result['reason'])
                self.assertEqual([row['op'] for row in self.outputs[before:]], ['stop'])


class RouteCliTests(unittest.TestCase):
    module = base.TurnCliTests.module

    def test_continue_route_rejected_on_other_commands_before_io(self):
        cli = self.module()
        for command in ('status', 'turn-left', 'straight', 'stop'):
            with self.subTest(command=command), patch.object(sys, 'argv', ['autonomy-control.py', command,
                    '--continue-route']), patch.object(sys, 'stderr', io.StringIO()), self.assertRaises(SystemExit) as caught:
                cli.main()
            self.assertEqual(caught.exception.code, 2)
        for value in (1, None, 'true'):
            with self.assertRaisesRegex(ValueError, 'invalid_continue_route'):
                cli.first_compact_target_trial(None, None, 10, True, continue_route=value)

    def test_continue_route_preview_is_explicit_query_and_readonly(self):
        cli, requests, output = self.module(), [], io.StringIO()
        state = {'healthy': True, 'status': {'control': {'armed': False, 'motor': 1500, 'servo': 1500}},
                 'autonomy': {'trial_mode': 'turn-cone', 'continue_route': True, 'turn_ready': True}}
        class Opener:
            def open(self, request, timeout=None):
                requests.append(request)
                return io.BytesIO(json.dumps(state).encode())
        with tempfile.TemporaryDirectory() as directory:
            access = Path(directory)/'access.json'
            access.write_text(json.dumps({'port': 8081, 'token': 'offline'}))
            with patch.object(sys, 'argv', ['autonomy-control.py', 'turn-cone', '--continue-route',
                     '--access-file', str(access)]), patch.object(cli.urllib.request, 'build_opener',
                     return_value=Opener()), patch.object(sys, 'stdout', output):
                cli.main()
        self.assertEqual(len(requests), 1)
        self.assertEqual(requests[0].get_method(), 'GET')
        self.assertIn('continue_route=true', requests[0].full_url)
        self.assertIs(json.loads(output.getvalue())['continue_route'], True)
        self.assertFalse(json.loads(output.getvalue())['motion_requested'])

    def test_explicit_route_reaches_start_payload_as_bool(self):
        cli, requests = self.module(), []
        state = {'boot': 'test', 'autonomy': {'epoch': 0}, 'status': {'control': {'tick': 1}}}
        def request(path, data=None):
            requests.append((path, data))
            raise RuntimeError('bounded payload observation')
        with self.assertRaisesRegex(RuntimeError, 'bounded payload'):
            cli.run_probe(request, state, 1560, 18000, turn_trial=True, compact_target_trial=True,
                          placement_confirmed=True, continue_route=True)
        self.assertEqual(len(requests), 1)
        self.assertIs(requests[0][1]['continue_route'], True)
        self.assertEqual(requests[0][1]['op'], 'turn_cone_start')


if __name__ == '__main__':
    unittest.main()
