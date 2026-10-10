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
        self.assertEqual(snapshot['control']['motor'], 1560)
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

    def test_second_quality_loss_halts_without_rearming_or_new_wait(self):
        session, _ = self.resume()
        self.observe(round(self.now+.1, 6), self.compact_scan(self.console.scan['seq']+1))
        snapshot = copy.deepcopy(session['report']['quality_wait_snapshot'])
        self.observe(round(self.now+.1, 6), self.incomplete(self.console.scan['seq']+1))
        self.assertIsNone(self.console.auto_session)
        self.assertEqual(self.outputs[-1]['op'], 'stop')
        self.assertEqual(self.console.auto_result['quality_wait_snapshot'], snapshot)
        self.assertEqual(sum(row['op'] == 'arm' for row in self.outputs), 1)

    def test_front_sparse_only_extends_actual_neutral_wait_and_never_restores(self):
        session, held = self.wait()
        # 20 unknown elsewhere plus 16 front unknown leaves exactly the
        # existing 324-return neutral floor. This never passes drive quality.
        for front_gap in (5, 7, 12, 16):
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
        for fault in ('old_clearance', 'false_unknowns', 'pending_neutral', 'drive'):
            with self.subTest(fault=fault):
                original = copy.deepcopy(clearance)
                if fault == 'old_clearance': clearance['scan_seq'] -= 1
                if fault == 'false_unknowns': clearance['front_unknown_bins'] = []
                if fault == 'pending_neutral':
                    control['motor'] = 1560
                    session['quality_wait_neutral_sequence'] = control['seq']+1
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
        for fault in ('drive', 'missing_sequence', 'unacked', 'positive', 'servo_changed', 'bridge_locked'):
            with self.subTest(fault=fault):
                self.reset_case()
                if fault == 'drive':
                    self.enter_compact_orbit(continue_route=True)
                else:
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

    def test_current_target_identity_and_full_boundary_are_required_during_wait(self):
        for fault in ('missing', 'changed', 'partial'):
            with self.subTest(fault=fault):
                self.reset_case()
                self.wait()
                value = self.compact_scan(self.console.scan['seq']+1,
                    observed=fault != 'missing', distance=2. if fault == 'changed' else 1.)
                if fault == 'partial': value['ranges'][self.target_bins.start-1] = None
                before = len(self.outputs)
                self.observe(round(self.now+.1, 6), value)
                self.assertIsNone(self.console.auto_session)
                self.assertFalse(any(row['motor'] > 1500 for row in self.outputs[before:]))

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

    def test_target_loss_terminal_coast_cannot_reopen_even_with_route_enabled(self):
        _, session = self.enter_compact_orbit(continue_route=True)
        self.observe(round(self.now+.1, 6), self.compact_scan(self.console.scan['seq']+1, observed=False))
        self.assertEqual(session['phase'], 'coast')
        before = len(self.outputs)
        for _ in range(7):
            self.observe(round(self.now+.1, 6), self.compact_scan(self.console.scan['seq']+1))
        self.assertFalse(any(row['motor'] > 1500 for row in self.outputs[before:]))
        self.assertNotIn('quality_wait_snapshot', session['report'])
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
