"""Opt-in route checks. Synthetic targets/ACKs are not a physical S-run replay."""
import copy
import json
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).parents[1]))
from maneuver_sequence import ManeuverSequence, SECOND_ORBIT_PWM_STEP, SECOND_BYPASS_SUPPORT_GAP_M
from compact_target import _candidates
from test_compact_target import compact_scan
from test_compact_target_roles import add_object
from test_maneuver_sequence import corridor


def observation(seq, *, first=90., second=None, heading=-25., offset=0., received=None):
    now = seq/10 if received is None else received
    scan = compact_scan(seq, seq*100, now, bearing=first)
    if second is not None:
        add_object(scan, bearing=second)
    c = corridor(heading)
    c['center_offset_left_m'] = offset
    c['width_m'] = 2.4
    scan['corridor_candidates'] = [c]
    return scan


def feedback(seq, servo=1670, motor=1560, **extra):
    return {'armed': True, 'servo': servo, 'motor': motor, 'seq': seq, 'tick': seq*100,
            'command_acked': True, **extra}


class ManeuverRouteTests(unittest.TestCase):
    # Default synthetic target is exactly abeam (at the measured body front).
    # Wait/resume mechanics tests that must not trigger the trial40 release use
    # HOLD_BEARING, whose current support stays ahead of the body front.
    HOLD_BEARING = 80.

    def setUp(self):
        self.bearing = 90.

    def edge(self):
        # Range bin just outside the synthetic support (partial boundary).
        return (round(-self.bearing) % 360)-5

    def seeded(self, *, servo=1670):
        # Mock a previously adopted initial turn; target maturation itself uses
        # the real tracker and explicit synthetic scan publication/receipt times.
        motion = ManeuverSequence(0., continue_route=True)
        for seq in (1, 2, 3, 4):
            scan = observation(seq, first=self.bearing)
            motion.compact_target = motion.target_tracker.update(scan, seq/10)
        motion.phase = 'drive'
        motion.drive_since = motion.orbit_since = .4
        motion.handover_observed = True
        motion.orbit_track_id = motion.compact_target['track_id']
        motion.servo = motion.steering_target = servo
        motion.orbit_bias_pwm = servo
        motion.orbit_reference_range_m = 1.
        motion._current_scan = scan
        return motion

    def begin_wait(self):
        motion = self.seeded()
        self.assertTrue(motion.begin_quality_wait(.5, feedback(5), scan=observation(5, first=self.bearing)))
        return motion

    def step(self, motion, seq, *, servo=None, motor=None, quality=False, ready=False,
             first=None, second=None, heading=-25., offset=0., **control):
        first = self.bearing if first is None else first
        value = observation(seq, first=first, second=second, heading=heading, offset=offset)
        result = motion.update(value, 0., seq/10,
            feedback(seq, motion.servo if servo is None else servo,
                     1500 if motor is None and motion.phase == 'quality_wait' else
                     1560 if motor is None else motor, **control),
            quality_clear=quality, quality_resume_ready=ready)
        return result

    def route(self, stage='right_align', servo=1500):
        motion = self.seeded(servo=servo)
        motion.first_pass_evidence = {'track_id': motion.orbit_track_id, 'source_seq': 4}
        motion.first_pass_preparing = True
        motion.right_exit_since = .4
        motion.route_stage = stage
        ack = feedback(4, 1500)
        motion.right_exit_center_ack = {k: ack[k] for k in ('servo', 'motor', 'tick', 'seq')}
        motion._route_center_ack = copy.deepcopy(motion.right_exit_center_ack)
        return motion

    def test_opt_in_is_explicit_and_terminal_coast_cannot_wait(self):
        with self.assertRaises(ValueError):
            ManeuverSequence(0., continue_route=1)
        motion = self.seeded()
        motion.continue_route = False
        self.assertFalse(motion.begin_quality_wait(.5, feedback(5), scan=observation(5)))
        motion.continue_route = True
        motion.begin_coast('old_terminal_stop', .5)
        self.assertFalse(motion.begin_quality_wait(.6, feedback(6), scan=observation(6)))
        self.assertEqual(self.step(motion, 6)['motor'], 1500)

    def test_wait_requires_current_full_original_target_and_actual_left_ack(self):
        for invalid in ('ack', 'motor', 'servo', 'partial', 'identity', 'old', 'passed'):
            with self.subTest(invalid=invalid):
                motion = self.seeded()
                value, control = observation(5), feedback(5)
                if invalid == 'ack': control['command_acked'] = False
                if invalid == 'motor': control['motor'] = 1500
                if invalid == 'servo': control['servo'] = 1660
                if invalid == 'partial': value['ranges'][265] = None
                if invalid == 'identity': value = observation(5, first=120.)
                if invalid == 'old': value['received_at'] = .1
                if invalid == 'passed': motion.first_pass_evidence = {'track_id': 1}
                self.assertFalse(motion.begin_quality_wait(.5, control, scan=value))

    def test_quality_wait_matures_new_frames_then_holds_until_actual_resume_ack(self):
        self.bearing = self.HOLD_BEARING
        motion = self.begin_wait()
        for seq in (5, 6, 7):
            result = self.step(motion, seq, quality=True, ready=True, neutral_acked=True)
            self.assertEqual((result['phase'], result['motor'], result['servo']), ('quality_wait', 1500, 1670))
        result = self.step(motion, 8, quality=True, ready=True, neutral_acked=True)
        self.assertEqual((result['motor'], result['servo']), (1560, 1670))
        self.assertTrue(result['quality_resume_pending'])
        result = self.step(motion, 9, motor=1500)
        self.assertEqual(result['servo'], 1670)
        self.assertTrue(result['quality_resume_pending'])
        result = self.step(motion, 10, motor=1560, resume_acked=True)
        self.assertEqual(result['servo'], 1670)
        self.assertFalse(result['quality_resume_pending'])
        self.assertEqual((motion.drive_since, motion.orbit_since), (.4, .4))
        self.assertFalse(motion.begin_quality_wait(1., feedback(10), scan=observation(10)))

    def test_bad_quality_resets_maturity_and_service_permission_is_required(self):
        motion = self.begin_wait()
        for seq in (5, 6):
            self.step(motion, seq, quality=True, neutral_acked=True)
        self.step(motion, 7, quality=False, neutral_acked=True)
        for seq in (8, 9, 10, 11):
            result = self.step(motion, seq, quality=True, ready=False, neutral_acked=True)
            self.assertEqual(result['motor'], 1500)
        self.assertEqual(self.step(motion, 12, quality=True, ready=True, neutral_acked=True)['motor'], 1560)

    def test_duplicate_quality_frame_cannot_mature_or_resume(self):
        motion = self.begin_wait()
        value = observation(5)
        for now in (.5, .55, .6, .7):
            result = motion.update(value, now-.5, now, feedback(5, 1670, 1500, neutral_acked=True),
                                   quality_clear=True, quality_resume_ready=True)
            self.assertEqual(result['motor'], 1500)
        self.assertEqual(len(motion._quality_good_history), 1)

    def test_wait_can_reconfirm_actual_rear_pass_but_needs_unique_current_corridor(self):
        for ambiguous in (False, True):
            motion = self.begin_wait()
            for seq, bearing in ((5, 90.), (6, 105.), (7, 120.), (8, 120.), (9, 120.)):
                value = observation(seq, first=bearing)
                if ambiguous: value['corridor_candidates'].append(corridor(-20.))
                result = motion.update(value, 0., seq/10,
                                       feedback(seq, motion.servo, 1500, neutral_acked=True),
                                       quality_clear=True, quality_resume_ready=True)
            if ambiguous:
                self.assertEqual(result['reason'], 'route_quality_wait_pass_corridor_unavailable')
                self.assertIsNone(result['first_pass_evidence'])
            else:
                # The abeam 5/6 supports already released left in the wait.
                self.assertEqual(result['phase'], 'drive')
                self.assertTrue(1500 <= result['servo'] < 1670)
                self.assertEqual(result['quality_wait_release']['source_seq'], 6)
                self.assertTrue(result['quality_resume_pending'])
                self.assertEqual(result['route_stage'], 'right_align')
                self.assertLessEqual(result['first_pass_evidence']['observed_frontmost_x_m'], -.18)

    def test_pending_resume_target_loss_or_partial_cuts_power_even_on_ack(self):
        self.bearing = self.HOLD_BEARING
        for failure in ('missing', 'partial', 'identity'):
            for ack in (False, True):
                motion = self.begin_wait()
                for seq in (5, 6, 7, 8):
                    self.step(motion, seq, quality=True, ready=True, neutral_acked=True)
                value = observation(9, first=self.bearing)
                if failure == 'missing': value['ranges'] = [3.]*360
                elif failure == 'partial': value['ranges'][self.edge()] = None
                else: value = observation(9, first=120.)
                result = motion.update(value, 0., .9,
                    feedback(9, 1670, 1560 if ack else 1500, resume_acked=ack))
                self.assertEqual(result['motor'], 1500)
                self.assertEqual(result['reason'], 'route_quality_resume_target_unconfirmed')

    def test_rear_pass_resume_keeps_current_corridor_observations_while_ack_pending(self):
        motion = self.begin_wait()
        for seq, bearing in ((5, 90.), (6, 105.), (7, 120.), (8, 120.), (9, 120.)):
            self.step(motion, seq, first=bearing, quality=True, ready=True, neutral_acked=True)
        self.step(motion, 10, first=120., motor=1500)
        self.step(motion, 11, first=120., motor=1560, resume_acked=True)
        result = self.step(motion, 12, first=120., motor=1560)
        self.assertEqual(result['phase'], 'drive')
        self.assertEqual(result['right_exit_geometry']['source_seq'], 12)
        self.assertLess(result['servo'], 1670)

    def test_wait_hard_fault_partial_or_identity_loss_cannot_resume(self):
        for invalid in ('partial', 'missing', 'identity', 'servo', 'unsafe'):
            with self.subTest(invalid=invalid):
                motion = self.begin_wait()
                value, control = observation(5), feedback(5, motor=1500, neutral_acked=True)
                if invalid == 'partial': value['ranges'][265] = None
                if invalid == 'missing': value['ranges'] = [3.]*360
                if invalid == 'identity': value = observation(5, first=120.)
                if invalid == 'servo': control['servo'] = 1660
                result = motion.update(value, 0., .5, control, safe=invalid!='unsafe',
                                       quality_clear=True, quality_resume_ready=True)
                self.assertTrue(result['lock_requested'])
                self.assertEqual(result['motor'], 1500)

    def test_neutral_and_resume_ack_deadlines_and_original_orbit_budget(self):
        self.bearing = self.HOLD_BEARING
        motion = self.begin_wait()
        for seq in (5, 6, 7, 8):
            result = self.step(motion, seq, motor=1560)
        self.assertEqual(result['reason'], 'route_quality_wait_neutral_ack_timeout')
        motion = self.begin_wait()
        for seq in (5, 6, 7, 8):
            self.step(motion, seq, quality=True, ready=True, neutral_acked=True)
        for seq in (9, 10, 11, 12):
            result = self.step(motion, seq, motor=1500)
        self.assertEqual(result['reason'], 'route_quality_resume_ack_timeout')
        motion = self.begin_wait()
        motion.orbit_since = -2.5
        result = self.step(motion, 5, neutral_acked=True)
        self.assertEqual(result['reason'], 'route_quality_wait_original_budget_expired')

    def test_right_alignment_uses_heading_not_cancelling_offset(self):
        motion = self.route()
        result = self.step(motion, 5, heading=-23., offset=.54)
        self.assertEqual(result['servo'], 1490)
        self.assertTrue(result['route_right_output_authorized'])
        self.assertLess(result['right_exit_geometry']['steering_target_pwm'], 1500)

    def test_alignment_matures_both_clocks_then_follows_without_fixed_distance(self):
        motion = self.route()
        for seq in (5, 6, 7):
            result = self.step(motion, seq, heading=-2.)
            self.assertEqual(result['route_stage'], 'right_align')
        result = self.step(motion, 8, heading=-2.)
        self.assertEqual(result['route_stage'], 'corridor_follow')
        self.assertFalse(result['route_alignment_evidence']['physical_alignment_certified'])
        result = self.step(motion, 9, heading=-1.)
        self.assertEqual(result['motor'], 1560)
        self.assertFalse(result['second_target_handover_observed'])

    def test_ambiguous_corridor_and_right_budget_still_stop(self):
        for failure in ('ambiguous', 'budget'):
            motion = self.route()
            value = observation(5)
            if failure == 'ambiguous': value['corridor_candidates'].append(corridor(-24.))
            else: motion.right_exit_since = -2.5
            result = motion.update(value, 0., .5, feedback(5, 1500))
            self.assertEqual((result['phase'], result['motor']), ('coast', 1500))

    def test_corridor_left_then_right_requires_new_actual_center_ack(self):
        motion = self.route('corridor_follow')
        first = self.step(motion, 5, heading=4., offset=.3)
        self.assertEqual(first['servo'], 1510)
        self.assertIsNone(first['route_center_ack'])
        # Stay inside existing offset association gate while changing correction.
        self.step(motion, 6, heading=-4., offset=.01)
        result = self.step(motion, 7, heading=-4., offset=-.1, servo=1510, command_acked=False)
        self.assertGreaterEqual(result['servo'], 1500)
        self.assertFalse(result['route_right_output_authorized'])
        result = self.step(motion, 8, heading=-4., offset=-.1, servo=1500)
        self.assertEqual(result['servo'], 1490)
        self.assertEqual(result['route_center_ack']['seq'], 8)

    def test_unexpected_actual_left_command_revokes_right_continuity(self):
        for stage in ('right_align', 'corridor_follow'):
            motion = self.route(stage, servo=1450)
            result = self.step(motion, 5, servo=1600, heading=0.)
            self.assertEqual(result['motor'], 1500)
            self.assertIsNone(result['route_center_ack'])
            self.assertFalse(result['route_right_output_authorized'])

    def test_second_role_needs_one_support_ahead_inside_corridor_each_maturity_frame(self):
        motion = self.route('corridor_follow')
        for seq in (5, 6):
            result = self.step(motion, seq, second=-45., heading=0.)
            self.assertFalse(result['second_target_handover_observed'])
        # A second support ahead in the same corridor is ambiguous and
        # discards the whole maturity interval.
        value = add_object(observation(7, second=-45., heading=0.), bearing=30.)
        result = motion.update(value, 0., .7, feedback(7, motion.servo))
        self.assertIsNone(result['second_target'])
        for seq in (8, 9, 10):
            result = self.step(motion, seq, second=-45., heading=0.)
            self.assertFalse(result['second_target_handover_observed'])
        result = self.step(motion, 11, second=-45., heading=0.)
        self.assertTrue(result['second_target_handover_observed'])
        self.assertEqual(result['route_stage'], 'second_orbit')
        evidence = result['role_distinct_evidence']
        self.assertTrue(evidence['first_support_visible'])
        self.assertGreater(evidence['actual_support_separation_m'], .35)

    def test_partial_first_no_longer_discards_second_role_after_first_pass(self):
        motion = self.route('corridor_follow')
        for seq in (5, 6, 7, 8):
            value = observation(seq, second=-45., heading=0.)
            if seq == 7: value['ranges'][265] = None  # first partial; second stays full
            result = motion.update(value, 0., seq/10, feedback(seq, motion.servo))
        self.assertTrue(result['second_target_handover_observed'])

    def test_support_outside_current_corridor_is_never_second_role(self):
        motion = self.route('corridor_follow')
        for seq in (5, 6, 7, 8, 9):
            value = observation(seq, heading=0.)  # corridor half width 1.2, offset 0
            add_object(value, bearing=-70., distance=2.)  # normal about -1.88 m
            result = motion.update(value, 0., seq/10, feedback(seq, motion.servo))
        self.assertFalse(result['second_target_handover_observed'])
        self.assertEqual(result['second_acquisition_reason'], 'second_role_current_distinct_support_missing')

    def test_trial43_left_front_second_support_gets_left_bypass_then_right_after_center_ack(self):
        motion = self.route('corridor_follow')
        for seq in (5, 6, 7, 8):
            result = self.step(motion, seq, second=15., heading=0.)
        self.assertTrue(result['second_target_handover_observed'])
        self.assertEqual(result['role_distinct_evidence']['second_side'], 'left')
        self.assertEqual(result['servo'], 1500)
        result = self.step(motion, 9, second=15., heading=0.)
        self.assertEqual(result['reason'], 'second_target_left_of_support_entry')
        self.assertEqual(result['servo'], 1500+SECOND_ORBIT_PWM_STEP)
        feedback_ = result['second_feedback']
        self.assertAlmostEqual(feedback_['relative_bypass_point_left_m'][1],
                               feedback_['current_support_left_edge_y_m']+SECOND_BYPASS_SUPPORT_GAP_M)
        self.assertGreater(result['steering_target'], 1500)
        # Once the support is right of the bypass line the law asks for right:
        # first release to an ACKed center, never a direct left-to-right step.
        servos = []
        for seq, bearing in enumerate((5., -5., -15., -25., -35., -45., -55., -65.), 10):
            result = self.step(motion, seq, second=bearing, heading=0.)  # tracked, ~.17 m/frame
            self.assertEqual(result['route_stage'], 'second_orbit')
            servos.append(result['servo'])
        self.assertIn(1500, servos)
        first_right = next(i for i, v in enumerate(servos) if v < 1500)
        self.assertEqual(servos[first_right-1], 1500)
        self.assertIsNotNone(result['second_center_ack'])

    def test_real43_left_front_second_support_has_role_proof_on_each_saved_frame(self):
        # Actual trial43 scans; the old right-half rule never produced a proof.
        # Saved frames are every other scan (~.36 m apart at ~1.8 m/s), beyond
        # the .30 m tracker association gate, so only the per-frame role proof
        # is replayed here, not live confirmation.
        fixture = json.loads((Path(__file__).parent/'fixtures'/'left-cone-43-second-left-front.json').read_text())
        motion = self.route('corridor_follow')
        motion.compact_target = None  # the first support had left view
        proofs = {}
        for frame in fixture['frames']:
            scan = {**frame, 'received_at': frame['at_ms']/1000}
            motion.right_exit_geometry = dict(frame['controller_corridor'])
            motion._second_role_observation(scan, scan['received_at'])
            proofs[frame['seq']] = motion.role_distinct_evidence
        self.assertIsNone(proofs[2564])
        for seq in (2566, 2568, 2570, 2572, 2575):
            self.assertEqual(proofs[seq]['second_side'], 'left')
            self.assertFalse(proofs[seq]['first_support_visible'])
            self.assertLess(abs(proofs[seq]['second_normal_m']-.35), .05)

    def test_second_pass_coast_holds_acked_clockwise_command(self):
        motion, _ = self.second_handover()
        bearings = (-60., -70., -80., -90., -100., -110., -120., -130., -140., -150.)
        for seq, bearing in enumerate(bearings, 9):
            result = self.step(motion, seq, second=bearing, heading=0.)  # tracked, ~.17 m/frame
            if result['two_target_observed_pass_complete']:
                break
        self.assertTrue(result['two_target_observed_pass_complete'])
        held = result['route_end_coast_servo']
        self.assertIsNotNone(held)
        self.assertLess(held, 1500)
        self.assertEqual((result['motor'], result['servo']), (1500, held))
        for extra in (1, 2):
            result = self.step(motion, seq+extra, second=-150., heading=0.)
            self.assertEqual((result['motor'], result['servo']), (1500, held))

    def second_handover(self, servo=1500):
        motion = self.route('corridor_follow', servo=servo)
        if servo > 1500: motion._route_center_ack = None
        for seq in (5, 6, 7, 8):
            result = self.step(motion, seq, second=-45., heading=0.)
        self.assertTrue(result['second_target_handover_observed'])
        return motion, result

    def test_second_entry_keeps_handover_command_then_clockwise_bounded_steps(self):
        motion, result = self.second_handover()
        self.assertEqual(result['servo'], 1500)
        result = self.step(motion, 9, second=-45., heading=0.)
        self.assertEqual(result['servo'], 1500-SECOND_ORBIT_PWM_STEP)
        self.assertTrue(result['route_right_output_authorized'])
        self.assertTrue(1350 <= result['steering_target'] < 1500)

    def test_second_geometry_feedback_far_near_bearing_and_inside_release(self):
        def pwm(x, y):
            motion = self.route('second_orbit')
            motion.second_track_id, motion.second_orbit_since = 2, .4
            motion.second_center_ack = copy.deepcopy(motion._route_center_ack)
            value = observation(5)
            value['ranges'] = compact_scan(bearing=math.degrees(math.atan2(y, x)),
                                           distance=math.hypot(x, y),
                                           size=5 if math.hypot(x, y) > 2 else 9)['ranges']
            value['ranges'] = [6. if r == 3. else r for r in value['ranges']]
            c = _candidates(value['ranges'], bearing_range_rad=(-math.pi, 0.))[0]
            motion.second_target = {**c, 'track_id': 2, 'confirmed': True,
                'source_seq': 5, 'source_at_ms': 500, 'source_received_at': .5}
            result = motion._second_orbit_update(value, .5, feedback(5, 1500))
            self.assertFalse(result['second_feedback']['swept_path_certified'])
            return result['steering_target']
        self.assertGreater(pwm(2.8, -.65), pwm(.6, -.65))
        self.assertGreater(pwm(.8, -.25), pwm(.8, -.8))
        # Trial43: the bypass line keeps SECOND_BYPASS_SUPPORT_GAP_M from the
        # support's left edge, so a support just right of center needs left.
        self.assertGreater(pwm(.8, -.2), 1500)
        self.assertNotEqual(pwm(math.cos(math.radians(25)), -math.sin(math.radians(25))),
                            pwm(math.cos(math.radians(60)), -math.sin(math.radians(60))))

    def test_second_after_adopted_left_requires_new_center_ack(self):
        motion = self.route('second_orbit', servo=1520)
        motion.second_track_id, motion.second_orbit_since = 2, .4
        motion.second_center_ack = None
        motion._route_center_ack = None
        for seq, actual, ack in ((5, 1520, True), (6, 1520, False), (7, 1500, True)):
            value = observation(seq, second=-45.)
            c = _candidates(value['ranges'], bearing_range_rad=(-math.pi/2, 0.))[0]
            motion.second_target = {**c, 'track_id': 2, 'confirmed': True,
                'source_seq': seq, 'source_at_ms': seq*100, 'source_received_at': seq/10}
            result = motion._second_orbit_update(value, seq/10, feedback(seq, actual, command_acked=ack))
            if seq < 7:
                self.assertGreaterEqual(result['servo'], 1500)
                self.assertIsNone(result['second_center_ack'])
        self.assertEqual(result['second_center_ack']['seq'], 7)
        self.assertEqual(result['servo'], 1500-SECOND_ORBIT_PWM_STEP)

    def test_second_loss_is_terminal_and_does_not_reacquire(self):
        motion, _ = self.second_handover()
        result = self.step(motion, 9, heading=0.)
        self.assertEqual(result['reason'], 'second_target_lost_or_identity_changed')
        result = self.step(motion, 10, second=-45., heading=0.)
        self.assertEqual(result['motor'], 1500)
        self.assertFalse(result['two_target_observed_pass_complete'])

    def test_second_complete_observed_support_requires_three_frames_and_rear(self):
        motion, _ = self.second_handover()
        for seq, bearing in enumerate((-60., -75., -90., -105., -120., -120., -120., -120.), 9):
            result = self.step(motion, seq, second=bearing, heading=0.)
        self.assertEqual(result['motor'], 1500)
        self.assertTrue(result['two_target_observed_pass_complete'])
        self.assertFalse(result['completed'])
        self.assertFalse(result['second_pass_evidence']['physical_cone_pass_certified'])

    def test_first_lost_then_reappearing_beside_is_never_second_role_proof(self):
        # The lost first support may reappear under a new numeric ID; beside or
        # behind the body front it is never a second-role candidate.
        motion = self.route('corridor_follow')
        value = observation(5, heading=0.)
        for i in range(266, 275): value['ranges'][i] = 3.
        motion.update(value, 0., .5, feedback(5, 1500))
        for seq in (6, 7, 8, 9):
            result = self.step(motion, seq, heading=0.)
        self.assertIsNone(result['compact_target'])
        self.assertIsNone(result['second_target'])
        self.assertFalse(result['second_target_handover_observed'])

    def test_first_lost_single_support_ahead_is_second_after_recorded_first_pass(self):
        # Trial43: the first support left view before the second appeared.
        motion = self.route('corridor_follow')
        for seq in (5, 6, 7, 8):
            value = observation(seq, first=200., second=-45., heading=0.)  # first behind, not tracked
            result = motion.update(value, 0., seq/10, feedback(seq, motion.servo))
        self.assertTrue(result['second_target_handover_observed'])
        evidence = result['role_distinct_evidence']
        self.assertFalse(evidence['first_support_visible'])
        self.assertIsNone(evidence['actual_support_separation_m'])
        self.assertEqual(evidence['basis'], 'single_support_ahead_inside_current_corridor_after_first_pass')

    def test_real34_wait_recovery_retains_original_rear_gate_and_no_second_claim(self):
        fixture = json.loads((Path(__file__).parent/'fixtures'/'left-cone-34-route-observations.json').read_text())
        frames = {s['seq']: s for s in fixture['frames']}
        motion = ManeuverSequence(0., continue_route=True)
        for seq in (585, 586, 588, 590):
            raw = frames[seq]
            value = {**raw, 'received_at': raw['at_ms']/1000}
            motion.compact_target = motion.target_tracker.update(value, value['received_at'])
        # Mock prior turn/ACK and receive timestamps, not a physical replay.
        motion.phase, motion.servo = 'drive', 1690
        motion.steering_target = 1690
        motion.drive_since, motion.orbit_since = 55., 58.5
        motion.handover_observed = True
        motion.orbit_track_id = motion.compact_target['track_id']
        motion.orbit_bias_pwm, motion.orbit_reference_range_m = 1690, 1.
        start = {**frames[591], 'received_at': frames[591]['at_ms']/1000}
        self.assertTrue(motion.begin_quality_wait(start['received_at'], feedback(591, 1690), scan=start))
        for seq in (591, 593, 595, 597, 598, 601):
            value = {**frames[seq], 'received_at': frames[seq]['at_ms']/1000}
            # Mock bridge reports each released wait command as already ACKed.
            result = motion.update(value, 0., value['received_at'],
                                   feedback(seq, motion.servo, 1500, neutral_acked=True),
                                   quality_clear=seq>=595, quality_resume_ready=True)
        # Actual saved 591/593 supports reach the measured body front; trial40
        # releases left only (never right) during the neutral wait.
        self.assertEqual(result['quality_wait_release']['source_seq'], 593)
        self.assertEqual(result['motor'], 1560)
        self.assertTrue(1500 <= result['servo'] < 1690)
        self.assertTrue(result['quality_resume_pending'])
        self.assertFalse(result['route_right_output_authorized'])
        self.assertIsNone(result['first_pass_evidence'])
        self.assertFalse(result['second_target_handover_observed'])
        end = frames[640]
        target = _candidates(end['ranges'])[0]
        front = max(end['ranges'][i]*math.cos(math.radians(i)) for i in target['support_bins'])
        self.assertGreater(front, -.18)

    def test_real34_current_corridor_can_request_right_but_alias_ambiguity_remains(self):
        fixture = json.loads((Path(__file__).parent/'fixtures'/'left-cone-34-route-observations.json').read_text())
        frames = {s['seq']: s for s in fixture['frames']}
        for seq in (602, 604, 607):
            motion = ManeuverSequence(0., continue_route=True)
            value = {**frames[seq], 'received_at': frames[seq]['at_ms']/1000}
            pwm, reason = motion._right_exit_target(value, value['received_at'])
            if seq == 604:
                self.assertIsNone(pwm)
                self.assertEqual(reason, 'right_exit_corridor_ambiguous')
            else:
                self.assertTrue(1350 <= pwm < 1500)
                self.assertIsNone(reason)

    def test_real34_saved_frames_have_no_independent_second_full_component(self):
        fixture = json.loads((Path(__file__).parent/'fixtures'/'left-cone-34-route-observations.json').read_text())
        for raw in fixture['frames']:
            if raw['seq'] < 591: continue
            observed = _candidates(raw['ranges'], bearing_range_rad=(-math.pi, math.pi))
            self.assertEqual(len(observed), 1)
            self.assertGreater(observed[0]['point_left_m'][1], 0.)

    def exit_observation(self, motion, seq, *, bearing=97., receipt=None, partial=False,
                         identity=None, **control):
        """Inject a full current target / prior handover, not a powered replay."""
        value = observation(seq, first=bearing, received=receipt)
        value['corridor_candidates'] = []
        c = _candidates(value['ranges'])[0]
        motion.compact_target = {**c, 'track_id': motion.orbit_track_id if identity is None else identity,
            'confirmed': True, 'source_seq': value['seq'], 'source_at_ms': value['at_ms'],
            'source_received_at': value['received_at'], 'tracking_only_boundary_gap': partial}
        adopted = feedback(seq, motion.servo)
        adopted.update(control)
        motion._observe_first_pass(value, value['received_at'], adopted)
        return value

    def test_geometric_exit_prepares_without_corridor_closing_rate_or_rear_pass(self):
        motion = self.seeded()
        self.exit_observation(motion, 5)
        self.assertFalse(motion.first_pass_preparing)
        # Trial40: two strictly fresh current full-support frames, not .25 s.
        self.exit_observation(motion, 6)
        self.assertTrue(motion.first_pass_preparing)
        self.assertEqual(motion.first_exit_prepare_evidence['observation_count'], 2)
        self.assertFalse(motion.orbit_left_entry_boost)
        self.assertFalse(motion.first_pass_progress['prediction_prepare_condition'])
        self.assertTrue(motion.first_pass_progress['geometric_exit_prepare_condition'])
        self.assertIsNone(motion.first_pass_evidence)
        self.assertIsNone(motion.right_exit_center_ack)
        self.assertFalse(motion._result(.8)['route_right_output_authorized'])
        self.assertEqual(motion.first_exit_prepare_evidence['action'], 'release_left_toward_neutral_only')
        self.assertFalse(motion.first_exit_prepare_evidence['observed_rear_pass'])

    def test_geometric_exit_default_mode_still_uses_original_predictive_rule(self):
        motion = self.seeded()
        motion.continue_route = False
        for seq in range(5, 12): self.exit_observation(motion, seq)
        self.assertFalse(motion.first_pass_preparing)
        self.assertIsNone(motion.first_exit_prepare_evidence)

    def test_geometric_exit_needs_handover_current_ack_and_actual_command(self):
        for fault in ('handover', 'ack', 'servo', 'motor'):
            motion = self.seeded()
            if fault == 'handover': motion.handover_observed = False
            for seq in range(5, 10):
                extra = ({'command_acked': False} if fault == 'ack' else
                         {'servo': 1660} if fault == 'servo' else
                         {'motor': 1500} if fault == 'motor' else {})
                self.exit_observation(motion, seq, **extra)
            self.assertFalse(motion.first_pass_preparing, fault)
            self.assertIsNone(motion.first_exit_prepare_evidence)

    def test_geometric_exit_window_breaks_on_partial_identity_source_or_stale(self):
        for fault in ('partial', 'identity', 'source', 'stale'):
            motion = self.seeded()
            self.exit_observation(motion, 6)
            value = observation(7, first=97.)
            c = _candidates(value['ranges'])[0]
            motion.compact_target = {**c, 'track_id': 9 if fault == 'identity' else motion.orbit_track_id,
                'confirmed': True, 'source_seq': 6 if fault == 'source' else 7,
                'source_at_ms': 700, 'source_received_at': .7,
                'tracking_only_boundary_gap': fault == 'partial'}
            motion._observe_first_pass(value, 1.01 if fault == 'stale' else .7, feedback(7))
            self.assertEqual(motion._first_exit_prepare_history, [])
            self.assertFalse(motion.first_pass_preparing)
            self.exit_observation(motion, 8)
            self.assertFalse(motion.first_pass_preparing)
            self.exit_observation(motion, 9)
            self.assertTrue(motion.first_pass_preparing)

    def test_geometric_exit_duplicate_or_short_receive_span_cannot_mature(self):
        motion = self.seeded()
        for _ in range(6): self.exit_observation(motion, 5)
        self.assertFalse(motion.first_pass_preparing)
        self.assertEqual(len(motion._first_exit_prepare_history), 1)
        for frames in (((5, .5), (6, .82)), ((5, .5), (6, .35)), ((5, .5), (8, .69))):
            # Receipt gap at the scan age limit, reordered receipt, or a
            # receipt/publication mismatch restarts the two-frame window.
            motion = self.seeded()
            for seq, receipt in frames:
                self.exit_observation(motion, seq, receipt=receipt)
            self.assertFalse(motion.first_pass_preparing, frames)

    def test_quality_wait_clears_geometric_window_and_latched_prepare_keeps_neutral_limit(self):
        motion = self.seeded()
        for seq in (5, 6):
            value = self.exit_observation(motion, seq)
            motion.target_tracker.update(value, seq/10)
        self.assertTrue(motion._first_exit_prepare_history)
        # Actual tracker still owns seed4; a current same-ID90-degree target
        # supplies the independent quality-wait entry, not the injected history.
        self.assertTrue(motion.begin_quality_wait(.7, feedback(7), scan=observation(7)))
        self.assertEqual(motion._first_exit_prepare_history, [])
        # Separate integrated sequence verifies the normal controller releases
        # in bounded steps and never gets right authorization from preparation.
        motion = self.seeded()
        outputs = [self.step(motion, seq, first=97.) for seq in range(5, 18)]
        prepared = [r for r in outputs if r['first_pass_preparing']]
        self.assertTrue(prepared)
        self.assertTrue(all(r['servo'] >= 1500 and not r['route_right_output_authorized'] for r in prepared))
        self.assertTrue(all(r['first_pass_evidence'] is None for r in prepared))
        for a, b in zip(prepared, prepared[1:]):
            self.assertGreaterEqual(a['servo']-b['servo'], 0)
            self.assertLessEqual(a['servo']-b['servo'], 40)
        self.assertEqual(prepared[-1]['servo'], 1500)

    def test_real37_current_support_prepares_without_fabricating_missing_raw1964(self):
        fixture = json.loads((Path(__file__).parent/'fixtures'/'left-cone-37-first-exit-prepare.json').read_text())
        summaries = fixture['controller_summaries']
        self.assertEqual([v['controller_scan_seq'] for v in summaries], [1961, 1963, 1964])
        progress = [v['derived_current_support_progress'] for v in summaries]
        self.assertTrue(all(v['center_x_m'] <= 0 and v['observed_frontmost_x_m'] <= .20
                            and v['observed_left_body_gap_m'] >= .08 for v in progress))
        self.assertGreaterEqual((progress[-1]['source_at_ms']-progress[0]['source_at_ms'])/1000, .25)
        self.assertGreaterEqual(progress[-1]['received_at']-progress[0]['received_at'], .25)
        self.assertNotIn(1964, [v['seq'] for v in fixture['raw_frames']])
        motion = self.seeded()
        for raw in fixture['raw_frames']:
            # These are MOCK receipt/ACK/handover values. Raw ranges, sequences
            # and publication clocks are the actual saved1961/1963/1965 frames.
            now = raw['at_ms']/1000
            value = {**raw, 'received_at': now}
            c = _candidates(value['ranges'])[0]
            motion.compact_target = {**c, 'track_id': motion.orbit_track_id, 'confirmed': True,
                'source_seq': raw['seq'], 'source_at_ms': raw['at_ms'], 'source_received_at': now}
            motion._observe_first_pass(value, now, feedback(raw['seq'], motion.servo))
        self.assertTrue(motion.first_pass_preparing)
        # Trial40 two-frame rule: actual 1961 and 1963 suffice; 1964 is still
        # absent and never fabricated.
        self.assertEqual(motion.first_exit_prepare_evidence['source_seq'], 1963)
        self.assertEqual(motion.first_exit_prepare_evidence['first_source_seq'], 1961)
        self.assertIsNone(motion.first_pass_evidence)
        self.assertFalse(motion._result(now)['route_right_output_authorized'])
        self.assertIn('first_exit_prepare', motion.route_events)

    def test_body_front_exit_can_prepare_while_current_center_is_still_ahead(self):
        motion = self.seeded()
        for seq in (5, 6, 7, 8): self.exit_observation(motion, seq, bearing=85.)
        self.assertTrue(motion.first_pass_preparing)
        evidence = motion.first_exit_prepare_evidence
        self.assertGreater(evidence['center_x_m'], 0.)
        self.assertLessEqual(evidence['observed_frontmost_x_m'], .20)
        self.assertEqual(evidence['basis'], 'current_first_target_complete_support_behind_body_front')
        self.assertFalse(motion.first_pass_progress['prediction_prepare_condition'])
        self.assertIsNone(motion.first_pass_evidence)
        self.assertFalse(motion._result(.8)['route_right_output_authorized'])

    def test_wait_release_holds_resume_output_then_continues_after_actual_ack(self):
        motion = self.begin_wait()
        outputs = [self.step(motion, seq, quality=True, ready=True, neutral_acked=True)
                   for seq in (5, 6, 7, 8)]
        # Two fresh abeam supports decide a release-only intention in the wait.
        self.assertEqual([r['servo'] for r in outputs[:3]], [1670, 1630, 1590])
        self.assertTrue(all(r['motor'] == 1500 for r in outputs[:3]))
        self.assertEqual(outputs[1]['quality_wait_release']['source_seq'], 6)
        self.assertFalse(outputs[1]['quality_wait_release']['right_steering_authorized'])
        # The powered resume request itself holds the ACKed released command.
        self.assertEqual((outputs[3]['motor'], outputs[3]['servo']), (1560, 1590))
        self.assertTrue(outputs[3]['quality_resume_pending'])
        self.assertFalse(outputs[3]['first_pass_preparing'])
        value = observation(8)
        result = motion.update(value, .05, .85, feedback(9, 1590, motor=1500), quality_clear=True)
        self.assertTrue(result['quality_resume_pending'])
        self.assertEqual(result['servo'], 1590)
        result = motion.update(value, .10, .90, feedback(10, 1590, resume_acked=True), quality_clear=True)
        self.assertFalse(result['quality_resume_pending'])
        self.assertTrue(result['first_pass_preparing'])
        self.assertEqual(result['servo'], 1590)  # Actual ACK output still holds.
        result = motion.update(value, .11, .91, feedback(11, 1590), quality_clear=True)
        self.assertEqual((result['motor'], result['servo']), (1560, 1550))
        self.assertIsNone(result['first_pass_evidence'])
        self.assertFalse(result['route_right_output_authorized'])

    def test_wait_release_needs_actual_neutral_ack_and_previous_step_ack(self):
        motion = self.begin_wait()
        # No actual neutral ACK: no release decision even with abeam support.
        for seq in (5, 6):
            result = motion.update(observation(seq), 0., seq/10, feedback(seq, 1670, 1500))
        self.assertIsNone(result['quality_wait_release'])
        self.assertEqual(result['servo'], 1670)
        motion = self.begin_wait()
        self.step(motion, 5, neutral_acked=True)
        result = self.step(motion, 6, neutral_acked=True)
        self.assertEqual(result['servo'], 1630)
        # The bridge still reports the previous command: wait, never stack steps.
        result = motion.update(observation(7), 0., .7,
            feedback(7, 1670, 1500, neutral_acked=True, command_acked=False))
        self.assertEqual((result['phase'], result['servo']), ('quality_wait', 1630))
        # Any other actual steering is a changed adoption and locks.
        result = motion.update(observation(8), 0., .8,
            feedback(8, 1610, 1500, neutral_acked=True))
        self.assertTrue(result['lock_requested'])
        self.assertEqual(result['reason'], 'route_quality_wait_feedback_changed')

    def test_wait_release_never_crosses_center_or_outputs_right(self):
        motion = self.begin_wait()
        servos = [self.step(motion, seq, neutral_acked=True)['servo'] for seq in range(5, 12)]
        self.assertEqual(servos[-1], 1500)
        self.assertTrue(all(v >= 1500 for v in servos))
        self.assertTrue(all(a-b <= 40 for a, b in zip(servos, servos[1:])))
        self.assertEqual(motion.phase, 'quality_wait')

    def test_release_latched_before_wait_continues_release_inside_wait(self):
        motion = self.seeded()
        for seq in (5, 6): self.step(motion, seq, first=85.)
        self.assertTrue(motion.first_pass_preparing)
        servo = motion.servo
        value = observation(7, first=85.)
        self.assertTrue(motion.begin_quality_wait(.7, feedback(7, servo), scan=value))
        self.assertEqual(motion.quality_wait_release['basis'], 'release_latched_before_neutral_wait')
        result = motion.update(observation(8, first=85.), 0., .8,
            feedback(8, servo, 1500, neutral_acked=True))
        self.assertEqual((result['phase'], result['motor'], result['servo']),
                         ('quality_wait', 1500, servo-40))

    def test_wait_bad_quality_duplicate_and_unacked_neutral_clear_body_window(self):
        motion = self.begin_wait()
        for seq in (5, 6, 7, 8):
            self.step(motion, seq, quality=True, neutral_acked=True)
        self.assertTrue(motion._quality_exit_prepare_ready)
        motion.update(observation(8), .01, .81, feedback(9, motion.servo, motor=1500, neutral_acked=True),
                      quality_clear=False, quality_resume_ready=True)
        self.assertEqual(motion._first_exit_prepare_history, [])
        self.assertFalse(motion._quality_exit_prepare_ready)
        for seq in (9, 10, 11):
            result = self.step(motion, seq, quality=True, ready=True, neutral_acked=True)
            self.assertEqual(result['phase'], 'quality_wait')
            self.assertFalse(motion._quality_exit_prepare_ready)
        self.assertEqual(self.step(motion, 12, quality=True, ready=True, neutral_acked=True)['motor'], 1560)
        motion = self.begin_wait()
        self.step(motion, 5, quality=True, neutral_acked=True)
        motion.update(observation(5), .01, .51, feedback(6, motion.servo, motor=1500, neutral_acked=False),
                      quality_clear=True)
        self.assertEqual(motion._first_exit_prepare_history, [])

    def test_pending_body_or_quality_mismatch_discards_wait_intent_before_ack(self):
        for fault in ('body_front', 'quality'):
            motion = self.begin_wait()
            for seq in (5, 6, 7, 8):
                self.step(motion, seq, quality=True, ready=True, neutral_acked=True)
            result = self.step(motion, 9, motor=1500, first=80. if fault == 'body_front' else 90.,
                               quality=fault != 'quality')
            # The healthy pre-resume window is still discarded on mismatch.
            self.assertEqual(result['servo'], 1590)
            self.assertTrue(result['quality_resume_pending'])
            self.assertEqual(motion._first_exit_prepare_history, [])
            self.assertFalse(motion._quality_exit_prepare_ready)
            # Trial40: a release already decided in the wait stays release-only;
            # it never becomes a pass, a right command or a new left demand.
            result = self.step(motion, 10, motor=1560, quality=True, resume_acked=True)
            self.assertTrue(result['first_pass_preparing'])
            self.assertEqual(result['first_exit_prepare_evidence']['basis'],
                             'neutral_wait_release_continued_at_resume_ack')
            self.assertEqual(result['servo'], 1590)
            self.assertIsNone(result['first_pass_evidence'])
            self.assertFalse(result['route_right_output_authorized'])

    def test_repeat_scan_only_releases_latched_prepare_at_100ms_without_catchup(self):
        motion = self.seeded()
        self.step(motion, 5, first=85.)
        before = motion.servo
        motion.update(observation(5, first=85.), .1, .6, feedback(6, before))
        self.assertFalse(motion.first_pass_preparing)
        self.assertEqual(motion.servo, before)
        self.assertEqual(len(motion._first_exit_prepare_history), 1)
        # A separate genuinely mature window, then repeated control ticks.
        motion = self.seeded()
        for seq in (5, 6): self.step(motion, seq, first=85.)
        self.assertTrue(motion.first_pass_preparing)
        value, before = observation(6, first=85.), motion.servo
        self.assertGreaterEqual(before, 1600)
        prep, passed = copy.deepcopy(motion._first_exit_prepare_history), copy.deepcopy(motion._first_pass_history)
        result = motion.update(value, .09, .69, feedback(7, before))
        self.assertEqual(result['servo'], before)
        result = motion.update(value, .10, .70, feedback(8, before))
        self.assertEqual(result['servo'], before-40)
        result = motion.update(value, .29, .89, feedback(9, before-40))
        self.assertEqual(result['servo'], before-80)  # One step, not catch-up.
        self.assertEqual((motion._first_exit_prepare_history, motion._first_pass_history), (prep, passed))
        self.assertIsNone(result['first_pass_evidence'])
        self.assertFalse(result['route_right_output_authorized'])

    def test_repeat_release_needs_actual_previous_command_ack_and_opt_in(self):
        for fault in ('ack', 'servo', 'motor', 'identity', 'default'):
            motion = self.seeded()
            for seq in (5, 6, 7, 8): self.step(motion, seq, first=85.)
            before = motion.servo
            control = feedback(9, before)
            if fault == 'ack': control['command_acked'] = False
            if fault == 'servo': control['servo'] = before+10
            if fault == 'motor': control['motor'] = 1500
            if fault == 'identity': motion.orbit_track_id += 1
            if fault == 'default': motion.continue_route = False
            result = motion.update(observation(8, first=85.), .1, .9, control)
            self.assertEqual(result['servo'], before, fault)
            self.assertIsNone(result['first_pass_evidence'])

    def test_repeat_release_rejects_bad_receipt_and_preserves_original_orbit_budget(self):
        for fault in ('future', 'stale', 'changed_receipt', 'budget'):
            motion = self.seeded()
            for seq in (5, 6, 7, 8): self.step(motion, seq, first=85.)
            before, value = motion.servo, observation(8, first=85.)
            if fault == 'future': value['received_at'] = 1.
            if fault == 'stale': value['received_at'] = .5
            if fault == 'changed_receipt': value['received_at'] = .81
            if fault == 'budget': motion.orbit_since = -2.1
            result = motion.update(value, .1, .9, feedback(9, before))
            if fault == 'budget':
                self.assertEqual(result['reason'], 'first_relative_object_entry_trial_timeout')
                self.assertEqual(result['motor'], 1500)
            elif fault == 'changed_receipt':
                self.assertIsNone(result['compact_target'])
                self.assertEqual(result['servo'], before)
            else:
                self.assertEqual(result['phase'], 'locked')
                self.assertEqual(result['motor'], 1500)
            self.assertIsNone(result['first_pass_evidence'])

    def test_real38_saved_wait_support_carries_body_front_prepare_with_explicit_mock_acks(self):
        fixture = json.loads((Path(__file__).parent/'fixtures'/'left-cone-38-wait-exit-prepare.json').read_text())
        self.assertEqual([r['seq'] for r in fixture['raw_frames']], [599, 601, 603])
        actual = fixture['recorded_quality_wait_evidence']
        self.assertEqual((actual['resume_seq'], actual['resume_ack']['tick']), (603, 60479))
        motion = self.begin_wait()
        motion.servo = motion.steering_target = motion.quality_wait_servo = 1705
        # Prior confirmed same-ID handover and target summaries are MOCKED.
        # Actual raw ranges/pub/seq remain unchanged; receive clocks are
        # explicitly injected from publication deltas, not vehicle timestamps.
        def current_target(scan, now, **unused):
            candidate = _candidates(scan['ranges'])[0]
            return {**candidate, 'track_id': motion.orbit_track_id, 'confirmed': True,
                    'source_seq': scan['seq'], 'source_at_ms': scan['at_ms'],
                    'source_received_at': scan['received_at']}
        motion.target_tracker.update = current_target
        origin = fixture['raw_frames'][0]['at_ms']
        for raw in fixture['raw_frames']:
            now = .6+(raw['at_ms']-origin)/1000
            value = {**raw, 'received_at': now, 'corridor_candidates': []}
            # Mock bridge reports each released wait command as already ACKed.
            result = motion.update(value, 0., now,
                feedback(raw['seq'], motion.servo, 1500, neutral_acked=True),
                quality_clear=True, quality_resume_ready=True)
        # Actual 599/601 supports are at the body front: trial40 releases left
        # inside the neutral wait (never right), then resumes on that command.
        self.assertEqual(result['quality_wait_release']['source_seq'], 601)
        self.assertEqual(result['quality_wait_release']['first_source_seq'], 599)
        self.assertTrue(result['quality_resume_pending'])
        self.assertFalse(result['first_pass_preparing'])
        self.assertEqual((result['motor'], result['servo']), (1560, 1665))
        result = motion.update(value, .02, now+.02,
            feedback(604, 1665, 1560, resume_acked=True), quality_clear=True)
        self.assertTrue(result['first_pass_preparing'])
        self.assertEqual(result['servo'], 1665)
        result = motion.update(value, .03, now+.12, feedback(605, 1665), quality_clear=True)
        self.assertEqual(result['servo'], 1625)
        self.assertEqual(result['first_exit_prepare_evidence']['first_source_seq'], 599)
        self.assertEqual(result['first_exit_prepare_evidence']['source_seq'], 603)
        self.assertIsNone(result['first_pass_evidence'])
        self.assertFalse(result['route_right_output_authorized'])


    # ---- trial40: release-only exit timing and same-frame wall-window pairs ----

    @staticmethod
    def real39():
        fixture = json.loads((Path(__file__).parent/'fixtures'/'left-cone-39-exit-release-pairs.json').read_text())
        return {f['seq']: f for f in fixture['frames']}

    def test_real39_wall_window_pairs_match_native_and_fill_only_missing(self):
        frames = self.real39()
        for seq in (482, 484, 486):
            native = [c for c in frames[seq]['corridor_candidates'] if abs(c['heading_left_rad']) < math.pi/2]
            paired, ambiguous = ManeuverSequence._paired_window_corridor(frames[seq])
            self.assertFalse(ambiguous)
            self.assertEqual(len(native), 1)
            self.assertLess(abs(paired['heading_left_rad']-native[0]['heading_left_rad']), math.radians(2))
            self.assertLess(abs(paired['width_m']-native[0]['width_m']), .1)
        # Actual 489: native fit empty with the first target in a side sector;
        # its two parallel native windows still give one current corridor.
        value = {**frames[489], 'received_at': frames[489]['at_ms']/1000}
        self.assertFalse([c for c in value['corridor_candidates'] if abs(c['heading_left_rad']) < math.pi/2])
        motion = ManeuverSequence(0., continue_route=True)
        pwm, reason = motion._right_exit_target(value, value['received_at'])
        self.assertIsNone(reason)
        self.assertEqual(motion.right_exit_geometry['source_kind'], 'native_wall_window_pair')
        self.assertTrue(1350 <= pwm < 1500)
        self.assertFalse(motion.right_exit_geometry['turn_path_certified'])
        # Actual 478 has no pair with enough shared support: still missing.
        value = {**frames[478], 'received_at': frames[478]['at_ms']/1000}
        self.assertEqual(ManeuverSequence(0., continue_route=True)._right_exit_target(value, value['received_at']),
                         (None, 'right_exit_corridor_missing'))
        # Default mode never uses the new fallback.
        value = {**frames[489], 'received_at': frames[489]['at_ms']/1000}
        self.assertEqual(ManeuverSequence(0.)._right_exit_target(value, value['received_at']),
                         (None, 'right_exit_corridor_missing'))

    def test_wall_window_pairs_with_distinct_axes_are_ambiguous_not_chosen(self):
        def window(heading_deg, rho, start, end):
            return {'candidate_only': True, 'heading_left_rad': math.radians(heading_deg), 'rho_left_m': rho,
                    'support_span_m': math.dist(start, end), 'fit_error_m': .02, 'points': 24,
                    'support_start_left_m': {'x_m': start[0], 'y_m': start[1]},
                    'support_end_left_m': {'x_m': end[0], 'y_m': end[1]}}
        walls = [window(0., 1., (-.5, 1.), (1., 1.)), window(0., -1., (-.5, -1.), (1., -1.))]
        paired, ambiguous = ManeuverSequence._paired_window_corridor({'wall_candidates': walls})
        self.assertFalse(ambiguous)
        self.assertAlmostEqual(paired['width_m'], 2.)
        self.assertAlmostEqual(paired['center_offset_left_m'], 0.)
        # A second, differently oriented parallel pair is a competing axis.
        c, s_ = math.cos(math.radians(40)), math.sin(math.radians(40))
        rot = lambda x, y: (c*x-s_*y, s_*x+c*y)
        walls += [window(40., .8, rot(-.5, .8), rot(1., .8)), window(40., -.8, rot(-.5, -.8), rot(1., -.8))]
        self.assertEqual(ManeuverSequence._paired_window_corridor({'wall_candidates': walls}), (None, True))
        # Too narrow, non-overlapping or same-side windows never pair.
        for bad in ([window(0., .3, (-.5, .3), (1., .3)), window(0., -.3, (-.5, -.3), (1., -.3))],
                    [window(0., 1., (1., 1.), (2., 1.)), window(0., -1., (-1., -1.), (.5, -1.))],
                    [window(0., 1., (-.5, 1.), (1., 1.)), window(0., 2., (-.5, 2.), (1., 2.))]):
            self.assertEqual(ManeuverSequence._paired_window_corridor({'wall_candidates': bad}), (None, False))

    def test_right_exit_missing_corridor_releases_left_only_then_coasts_after_lease(self):
        motion = self.route(servo=1574)
        motion.right_exit_center_ack = motion._route_center_ack = None
        motion._right_exit_last_receive, motion._right_exit_last_publication = .45, 450
        value = observation(5)
        value['corridor_candidates'] = []
        result = motion.update(value, 0., .5, feedback(5, 1574))
        self.assertEqual(result['reason'], 'right_exit_corridor_missing_release_left')
        self.assertEqual((result['motor'], result['servo']), (1560, 1534))
        self.assertFalse(result['route_right_output_authorized'])
        value = observation(6)
        value['corridor_candidates'] = []
        result = motion.update(value, 0., .6, feedback(6, 1534))
        self.assertEqual(result['servo'], 1500)  # Clamped at neutral, never right.
        value = observation(8)
        value['corridor_candidates'] = []
        result = motion.update(value, 0., .8, feedback(8, 1500))
        self.assertEqual((result['phase'], result['motor']), ('coast', 1500))
        self.assertGreaterEqual(result['servo'], 1500)

    def test_first_pass_frame_lease_only_releases_until_a_current_corridor_exists(self):
        motion = self.route(servo=1614)
        motion.right_exit_center_ack = motion._route_center_ack = None
        motion.first_pass_evidence = {'track_id': motion.orbit_track_id, 'source_seq': 5,
                                      'source_at_ms': 500, 'received_at': .5}
        value = observation(6)
        value['corridor_candidates'] = []
        result = motion.update(value, 0., .6, feedback(6, 1614))
        self.assertEqual((result['phase'], result['servo']), ('drive', 1574))
        value = observation(7)
        value['corridor_candidates'] = []
        result = motion.update(value, 0., .7, feedback(7, 1574))
        self.assertEqual((result['phase'], result['servo']), ('drive', 1534))
        value = observation(8)
        value['corridor_candidates'] = []
        result = motion.update(value, 0., .8, feedback(8, 1534))
        self.assertEqual((result['phase'], result['motor']), ('coast', 1500))

    def test_real39_actual_frames_release_inside_wait_before_resume(self):
        """Actual saved 466..489 ranges/publication; ACKs/receipt/wait entry MOCKED.

        Actual trial39 entered its wait on unsaved scan473; this command replay
        enters on the first saved incomplete frame 474. It shows command
        decisions only, never a counterfactual vehicle path or pass.
        """
        frames = self.real39()
        motion = ManeuverSequence(0., continue_route=True)
        for seq in (466, 467, 469):
            value = {**frames[seq], 'received_at': frames[seq]['at_ms']/1000}
            motion.compact_target = motion.target_tracker.update(value, value['received_at'])
        self.assertTrue(motion.compact_target['confirmed'])
        motion.phase, motion.servo = 'drive', frames[469]['recorded_control']['servo']
        motion.steering_target = motion.servo
        motion.drive_since, motion.orbit_since = 44.9, 46.5
        motion.handover_observed = True
        motion.orbit_track_id = motion.compact_target['track_id']
        motion.orbit_bias_pwm, motion.orbit_reference_range_m = motion.servo, 1.
        motion.last_seq, motion.last_publication = 469, frames[469]['at_ms']
        motion.last_receive = frames[469]['at_ms']/1000
        motion.last_change = motion.last_receive
        outputs = {}
        for seq in (471, 472, 474, 476, 478, 480, 482):
            value = {**frames[seq], 'received_at': frames[seq]['at_ms']/1000}
            now = value['received_at']
            if seq == 474:
                self.assertEqual(frames[seq]['recorded_quality_issues'], ['scan_incomplete'])
                self.assertTrue(motion.begin_quality_wait(now, feedback(seq, motion.servo), scan=value))
            wait = motion.phase == 'quality_wait'
            outputs[seq] = motion.update(value, 0., now,
                feedback(seq, motion.servo, 1500 if wait else 1560, neutral_acked=wait,
                         resume_acked=motion.quality_resume_pending),
                quality_clear=seq >= 476, quality_resume_ready=seq >= 476)
        # 474/476 actual supports are at the measured body front.
        self.assertEqual(outputs[476]['quality_wait_release']['source_seq'], 476)
        self.assertEqual(outputs[476]['quality_wait_release']['first_source_seq'], 474)
        self.assertLess(outputs[476]['servo'], frames[476]['recorded_control']['servo'])
        released = [outputs[s]['servo'] for s in (476, 478, 480, 482)]
        self.assertTrue(all(1500 <= v for v in released))
        self.assertTrue(all(a >= b for a, b in zip(released, released[1:])))
        self.assertLess(released[-1], frames[482]['recorded_control']['servo'])
        self.assertTrue(all(not r['route_right_output_authorized'] for r in outputs.values()))
        self.assertTrue(all(r['first_pass_evidence'] is None for r in outputs.values()))

class RouteEntryReleaseTests(unittest.TestCase):
    """Trial40: two fresh permitting endpoint bearings, then the opening vanished."""

    def run_frames(self, frames, continue_route=True):
        from test_turn_motion import opening_scan
        motion = ManeuverSequence(0., continue_route=continue_route)
        servo, released = 1500, []
        for seq, received, endpoint_index in frames:
            value = dict(opening_scan(seq, endpoint_index=endpoint_index),
                         at_ms=round(received*1000), received_at=received)
            result = motion.update(value, 0., received, {'armed': True, 'servo': servo,
                'motor': 1500, 'tick': seq*100, 'seq': seq, 'command_acked': True})
            servo = result['servo']
            released.append(result['entry_bearing_released'])
        return motion, released

    def test_route_releases_on_second_fresh_permitting_frame(self):
        motion, released = self.run_frames([(1, .1, 305), (2, .2, 305), (3, .3, 305)])
        self.assertEqual(released, [False, True, True])
        self.assertEqual(motion.entry_bearing['route_fresh_frames'], 2)
        self.assertFalse(motion.entry_bearing['endpoint_passed_proven'])

    def test_default_mode_keeps_three_frame_quarter_second_maturity(self):
        _, released = self.run_frames([(seq, seq/10, 305) for seq in range(1, 5)],
                                      continue_route=False)
        self.assertEqual(released, [False, False, False, True])

    def test_clock_gap_or_non_permitting_frame_restarts(self):
        # .30 s receipt gap is no longer consecutive.
        _, released = self.run_frames([(1, .1, 305), (2, .4, 305)])
        self.assertEqual(released, [False, False])
        # A non-permitting bearing between them clears the chain.
        motion, released = self.run_frames([(1, .1, 305), (2, .2, 320), (3, .3, 305)])
        self.assertEqual(released, [False, False, False])
        self.assertEqual(motion.entry_bearing['route_fresh_frames'], 1)

    def test_missing_endpoint_clears_chain(self):
        from test_turn_motion import opening_scan
        motion = ManeuverSequence(0., continue_route=True)
        motion.entry_bearing_required = True
        first = dict(opening_scan(1, endpoint_index=305), at_ms=100, received_at=.1)
        motion.last_now = .1
        motion._endpoint_observation(first)
        motion.last_now = .2
        motion._endpoint_observation({'seq': 2, 'at_ms': 200, 'received_at': .2,
                                      'left_turn_goal': None, 'wall_candidates': [],
                                      'ranges': [None]*360})
        self.assertEqual(motion._entry_ready_history, [])
        self.assertFalse(motion.entry_bearing_released)


if __name__ == '__main__':
    unittest.main()
