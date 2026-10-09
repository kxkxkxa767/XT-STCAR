"""Opt-in route checks. Synthetic targets/ACKs are not a physical S-run replay."""
import copy
import json
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).parents[1]))
from maneuver_sequence import ManeuverSequence
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
    def seeded(self, *, servo=1670):
        # Mock a previously adopted initial turn; target maturation itself uses
        # the real tracker and explicit synthetic scan publication/receipt times.
        motion = ManeuverSequence(0., continue_route=True)
        for seq in (1, 2, 3, 4):
            scan = observation(seq)
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
        self.assertTrue(motion.begin_quality_wait(.5, feedback(5), scan=observation(5)))
        return motion

    def step(self, motion, seq, *, servo=None, motor=None, quality=False, ready=False,
             first=90., second=None, heading=-25., offset=0., **control):
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
                result = motion.update(value, 0., seq/10, feedback(seq, 1670, 1500, neutral_acked=True),
                                       quality_clear=True, quality_resume_ready=True)
            if ambiguous:
                self.assertEqual(result['reason'], 'route_quality_wait_pass_corridor_unavailable')
                self.assertIsNone(result['first_pass_evidence'])
            else:
                self.assertEqual((result['phase'], result['servo']), ('drive', 1670))
                self.assertTrue(result['quality_resume_pending'])
                self.assertEqual(result['route_stage'], 'right_align')
                self.assertLessEqual(result['first_pass_evidence']['observed_frontmost_x_m'], -.18)

    def test_pending_resume_target_loss_or_partial_cuts_power_even_on_ack(self):
        for failure in ('missing', 'partial', 'identity'):
            for ack in (False, True):
                motion = self.begin_wait()
                for seq in (5, 6, 7, 8):
                    self.step(motion, seq, quality=True, ready=True, neutral_acked=True)
                value = observation(9)
                if failure == 'missing': value['ranges'] = [3.]*360
                elif failure == 'partial': value['ranges'][265] = None
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

    def test_second_needs_current_distinct_first_support_each_maturity_frame(self):
        motion = self.route('corridor_follow')
        for seq in (5, 6):
            result = self.step(motion, seq, second=-45., heading=0.)
            self.assertFalse(result['second_target_handover_observed'])
        value = observation(7, second=-45., heading=0.)
        value['ranges'][265] = None  # first is partial; second itself stays full
        result = motion.update(value, 0., .7, feedback(7, motion.servo))
        self.assertIsNone(result['second_target'])
        for seq in (8, 9, 10):
            result = self.step(motion, seq, second=-45., heading=0.)
            self.assertFalse(result['second_target_handover_observed'])
        result = self.step(motion, 11, second=-45., heading=0.)
        self.assertTrue(result['second_target_handover_observed'])
        self.assertEqual(result['route_stage'], 'second_orbit')
        self.assertGreater(result['role_distinct_evidence']['actual_support_separation_m'], .35)

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
        self.assertEqual(result['servo'], 1490)
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
        self.assertEqual(pwm(.8, -.2), 1500)
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
        self.assertEqual(result['servo'], 1490)

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

    def test_first_lost_then_new_numeric_id_cannot_be_second_role_proof(self):
        motion = self.route('corridor_follow')
        value = observation(5, second=-45., heading=0.)
        for i in range(266, 275): value['ranges'][i] = 3.
        motion.update(value, 0., .5, feedback(5, 1500))
        for seq in (6, 7, 8, 9):
            result = self.step(motion, seq, second=-45., heading=0.)
        self.assertIsNone(result['compact_target'])
        self.assertIsNone(result['second_target'])
        self.assertFalse(result['second_target_handover_observed'])

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
            result = motion.update(value, 0., value['received_at'], feedback(seq, 1690, 1500, neutral_acked=True),
                                   quality_clear=seq>=595, quality_resume_ready=True)
        self.assertEqual((result['motor'], result['servo']), (1560, 1690))
        self.assertTrue(result['quality_resume_pending'])
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


if __name__ == '__main__':
    unittest.main()
