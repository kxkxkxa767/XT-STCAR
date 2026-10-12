"""Route recovery from current evidence; synthetic scans, no physical claims."""
import copy
import math
import unittest

import test_maneuver_route as route


class RouteIdentityRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.helper = route.ManeuverRouteTests()
        self.helper.setUp()

    def second(self, rear=False):
        motion, _ = self.helper.second_handover()
        seq, bearing = 8, -45.
        if rear:
            for seq, bearing in ((9, -60.), (10, -75.), (11, -90.), (12, -105.)):
                self.helper.step(motion, seq, second=bearing, heading=0.)
        self.assertIsNone(motion.second_pass_evidence)
        return motion, seq, bearing

    def update(self, motion, seq, *, second=None, first=90., quality=True, ready=True, **extra):
        scan = route.observation(seq, first=first, second=second, heading=0.)
        result = motion.update(scan, 0., seq/10,
            route.feedback(seq, motion.servo, 1500 if motion.phase == 'quality_wait' else 1560,
                           neutral_acked=motion.phase == 'quality_wait', **extra),
            quality_clear=quality, quality_resume_ready=ready)
        return result, scan

    def enter_second_wait(self, rear=False):
        motion, seq, bearing = self.second(rear)
        seq += 1
        self.assertTrue(motion.begin_post_pass_quality_pause(seq/10,
            route.feedback(seq, motion.servo), scan=route.observation(seq, heading=0.)))
        return motion, seq, bearing

    def test_second_pause_reconfirms_front_and_right_rear_after_identity_loss(self):
        for rear in (False, True):
            with self.subTest(rear=rear):
                motion, start, bearing = self.enter_second_wait(rear)
                original_id, original_drive = motion.second_track_id, motion.drive_since
                for seq in range(start, start+5):
                    result, _ = self.update(motion, seq)
                    self.assertEqual(result['motor'], 1500)
                self.assertTrue(motion._second_recovering)
                for seq in range(start+5, start+16):
                    result, _ = self.update(motion, seq, second=bearing)
                    if motion.second_rebind_evidence is None:
                        self.assertEqual(result['motor'], 1500)
                    if result['quality_resume_pending']:
                        break
                self.assertTrue(result['quality_resume_pending'])
                self.assertEqual(result['route_stage'], 'second_orbit')
                self.assertNotEqual(motion.second_track_id, original_id)
                self.assertGreaterEqual(motion.second_target['confirmation_count'], 3)
                self.assertEqual(motion.drive_since, original_drive)
                self.assertIsNone(motion.second_pass_evidence)
                self.assertFalse(motion.second_rebind_evidence['observed_rear_pass'])

    def test_drive_loss_reacquires_right_rear_without_changing_it_into_a_new_front_role(self):
        motion, seq, bearing = self.second(rear=True)
        original = motion.second_track_id
        for seq in range(seq+1, seq+5):
            result, _ = self.update(motion, seq)
        self.assertEqual(result['route_stage'], 'corridor_follow')
        self.assertTrue(motion._second_recovering)
        for seq in range(seq+1, seq+7):
            result, _ = self.update(motion, seq, second=bearing)
            if result['route_stage'] == 'second_orbit':
                break
        self.assertEqual(result['route_stage'], 'second_orbit')
        self.assertNotEqual(motion.second_track_id, original)
        self.assertIsNone(result['second_pass_evidence'])

    def test_anchor_is_only_prior_and_its_gate_growth_is_capped(self):
        # Round 55 (Claude): the gate widens with time since the last full
        # observation (the car keeps moving) but stops at
        # SECOND_RECOVERY_MAX_SLACK_S (<= 1.2 m / 80 deg); far objects never bind.
        motion, start, bearing = self.enter_second_wait()
        for seq in range(start, start+15):
            self.update(motion, seq)
        for seq in range(start+15, start+19):
            result, _ = self.update(motion, seq, second=-170.)
            self.assertIsNone(motion.second_rebind_evidence)
            self.assertEqual(result['motor'], 1500)
        for seq in range(start+19, start+24):
            result, _ = self.update(motion, seq, second=bearing)
        self.assertIsNotNone(motion.second_rebind_evidence)
        self.assertIsNone(result['second_pass_evidence'])

    def moved_second(self, shift):
        """Second support at -45 deg/1 m, lost for 4 frames while driving, then
        seen again where it would be after the car moved ``shift`` m forward."""
        motion, _ = self.helper.second_handover()
        for seq in range(9, 13):
            scan = route.observation(seq, first=90., heading=0.)
            motion.update(scan, 0., seq/10, route.feedback(seq, motion.servo, 1550),
                          quality_clear=True, quality_resume_ready=True)
        x, y = .7071-shift, -.7071
        return motion, math.degrees(math.atan2(y, x)), math.hypot(x, y)

    def test_drive_loss_recovers_a_support_the_car_has_moved_past(self):
        # Round 55 (Claude): with a fixed .30 m/20 deg gate and the corridor
        # proof disabled during recovery, a support re-seen after >~.3 m of
        # travel was never recovered (shift .35/.50 m stayed corridor_follow).
        for shift in (0., .2, .35, .5):
            with self.subTest(shift=shift):
                motion, bearing, distance = self.moved_second(shift)
                self.assertTrue(motion._second_recovering)
                for seq in range(13, 25):
                    scan = route.observation(seq, first=90., heading=0.)
                    route.add_object(scan, bearing=bearing, distance=distance)
                    result = motion.update(scan, 0., seq/10, route.feedback(seq, motion.servo, 1550),
                                           quality_clear=True, quality_resume_ready=True)
                self.assertEqual(result['route_stage'], 'second_orbit')
                self.assertFalse(motion._second_recovering)
                self.assertIsNone(result['second_pass_evidence'])

    def test_recovery_cannot_take_current_first_target_even_when_anchor_matches(self):
        motion, seq, _ = self.second()
        scan = route.observation(seq+1, first=80., heading=0.)
        first = motion.target_tracker.update(scan, (seq+1)/10, acquire=False)
        self.assertIsNotNone(first)
        motion.compact_target = first
        motion._second_role_anchor = copy.deepcopy(first)
        motion.second_tracker._forget('test_lost')
        motion._start_second_identity_recovery()
        for seq in range(seq+2, seq+7):
            result, _ = self.update(motion, seq, first=80.)
            self.assertIsNone(motion.second_rebind_evidence)
            self.assertIsNone(motion.second_target)
        self.assertFalse(result['two_target_observed_pass_complete'])

    def test_ambiguous_recovery_cannot_confirm_or_rebind(self):
        motion, start, _ = self.enter_second_wait()
        for seq in range(start, start+5):
            self.update(motion, seq)
        for seq in range(start+5, start+11):
            scan = route.observation(seq, second=-45., heading=0.)
            route.add_object(scan, bearing=-30.)
            result = motion.update(scan, 0., seq/10,
                route.feedback(seq, motion.servo, 1500, neutral_acked=True),
                quality_clear=True, quality_resume_ready=True)
            self.assertEqual(result['motor'], 1500)
            self.assertIsNone(motion.second_rebind_evidence)

    def test_first_pass_wait_retries_missing_corridor_then_resumes_from_fresh_geometry(self):
        motion = self.helper.begin_wait()
        for seq, bearing in ((5, 90.), (6, 105.), (7, 120.), (8, 120.), (9, 120.)):
            scan = route.observation(seq, first=bearing)
            scan['corridor_candidates'] = []
            result = motion.update(scan, 0., seq/10,
                route.feedback(seq, motion.servo, 1500, neutral_acked=True),
                quality_clear=True, quality_resume_ready=True)
        self.assertEqual(result['phase'], 'quality_wait')
        self.assertEqual(result['reason'], 'route_quality_wait_pass_corridor_unavailable')
        self.assertIsNone(result['first_pass_evidence'])
        self.assertEqual(result['motor'], 1500)
        result = self.helper.step(motion, 10, first=120., heading=0.,
                                  quality=True, ready=True, neutral_acked=True)
        self.assertEqual(result['phase'], 'drive')
        self.assertEqual(result['route_stage'], 'right_align')
        self.assertTrue(result['quality_resume_pending'])
        self.assertEqual(result['right_exit_geometry']['source_seq'], 10)

    def test_first_held_gap_keeps_history_without_counting_or_resuming(self):
        self.helper.bearing = 80.
        motion = self.helper.begin_wait()
        for seq in (5, 6):
            self.helper.step(motion, seq, quality=True, ready=False, neutral_acked=True)
        before = list(motion._quality_good_history)
        scan = route.observation(7, first=80.)
        scan['ranges'] = [3.]*360
        result = motion.update(scan, 0., .7, route.feedback(7, motion.servo, 1500, neutral_acked=True),
                               quality_clear=True, quality_resume_ready=True)
        self.assertEqual(motion._quality_good_history, before)
        self.assertEqual(result['motor'], 1500)
        self.assertEqual(result['phase'], 'quality_wait')
        for seq in (8, 9, 10):
            scan = route.observation(seq, first=80.)
            scan['ranges'] = [3.]*360
            motion.update(scan, 0., seq/10, route.feedback(seq, motion.servo, 1500, neutral_acked=True),
                          quality_clear=True, quality_resume_ready=True)
        self.assertEqual(motion._quality_good_history, [])

    def test_second_held_gap_keeps_history_but_quality_fault_clears_it(self):
        motion, start, bearing = self.enter_second_wait()
        for seq in (start, start+1):
            self.update(motion, seq, second=bearing, ready=False)
        before = list(motion._quality_good_history)
        result, _ = self.update(motion, start+2)
        self.assertEqual(motion._quality_good_history, before)
        self.assertEqual(result['motor'], 1500)
        self.update(motion, start+3, quality=False)
        self.assertEqual(motion._quality_good_history, [])

    def test_alternating_held_and_complete_frames_do_not_count_as_stable_recovery(self):
        self.helper.bearing = 80.
        motion = self.helper.begin_wait()
        for seq in (5, 6, 7, 8, 9):
            scan = route.observation(seq, first=80.)
            if seq in (6, 8):
                scan['ranges'] = [3.]*360
            result = motion.update(scan, 0., seq/10,
                route.feedback(seq, motion.servo, 1500, neutral_acked=True),
                quality_clear=True, quality_resume_ready=True)
            self.assertEqual(result['motor'], 1500)
        self.assertEqual(len(motion._quality_good_history), 3)
        result = self.helper.step(motion, 10, first=80., quality=True, ready=True, neutral_acked=True)
        self.assertTrue(result['quality_resume_pending'])

    def test_corridor_gap_keeps_history_only_within_both_observation_clocks(self):
        for stage in ('right_align', 'corridor_follow'):
            with self.subTest(stage=stage):
                motion = self.helper.route(stage)
                scan = route.observation(5, heading=0.)
                self.assertTrue(motion.begin_post_pass_quality_pause(.5, route.feedback(5, motion.servo), scan=scan))
                for seq in (5, 6):
                    self.update(motion, seq, ready=False)
                before = list(motion._quality_good_history)
                self.assertTrue(before)
                for seq in (7, 8, 9, 10):
                    scan = route.observation(seq, heading=0.)
                    scan['corridor_candidates'] = []
                    result = motion.update(scan, 0., seq/10,
                        route.feedback(seq, motion.servo, 1500, neutral_acked=True),
                        quality_clear=True, quality_resume_ready=True)
                    if seq == 7:
                        self.assertEqual(motion._quality_good_history, before)
                    self.assertEqual(result['motor'], 1500)
                self.assertEqual(motion._quality_good_history, [])

    def test_observation_lease_rejects_publication_receive_disagreement(self):
        motion = route.ManeuverSequence(0., continue_route=True)
        scan = route.observation(7)
        self.assertTrue(motion._observation_lease(scan, .7, 600, .6))
        self.assertFalse(motion._observation_lease(scan, .7, 450, .69))
        self.assertFalse(motion._observation_lease(scan, .9, 600, .6))

    def test_first_target_on_straight_still_uses_orbit_intent(self):
        motion = self.helper.seeded()
        motion.first_pass_preparing = False
        result = self.helper.step(motion, 5, first=80., heading=0.)
        self.assertEqual(result['route_stage'], 'first_target')
        self.assertEqual(result['reason'], 'first_relative_object_left_orbit_entry')
        self.assertGreaterEqual(result['steering_target'], 1500)
        self.assertIsNone(result['first_pass_evidence'])

    def test_wait_still_expires_at_original_cumulative_budget(self):
        motion, start, _ = self.enter_second_wait()
        motion.max_drive_s = 1.
        for seq in range(start, 16):
            result, _ = self.update(motion, seq)
            if result['phase'] == 'locked':
                break
        self.assertEqual(result['phase'], 'locked')
        self.assertEqual(result['reason'], 'route_post_pass_pause_original_budget_expired')

    def contact_motion(self):
        motion = route.ManeuverSequence(0., continue_route=True)
        for seq in (1, 2, 3, 4):
            motion.compact_target = motion.target_tracker.update(route.observation(seq), seq/10)
        motion.phase, motion.drive_since = 'drive', .4
        motion.servo = motion.steering_target = 1670
        motion.update(route.observation(5), 0., .5, route.feedback(5, 1670))
        self.assertTrue(motion.handover_observed)
        for seq, distance, size in ((6, .8, 9), (7, .6, 11), (8, .4, 15), (9, .3, 21)):
            scan = route.observation(seq)
            scan['ranges'] = route.compact_scan(seq, seq*100, seq/10,
                                               bearing=90., distance=distance, size=size)['ranges']
            motion.update(scan, 0., seq/10, route.feedback(seq, motion.servo))
        scan = route.observation(10)
        scan['ranges'] = route.compact_scan(10, 1000, 1., bearing=90., distance=.2, size=21)['ranges']
        return motion, scan

    def test_contact_evidence_uses_current_exact_returns_without_advancing_real_tracker(self):
        # User 2026-10-12: a cone may be hit and the run continues (round 55,
        # Claude): every cone-like support in the fresh scan is a witness.
        motion, scan = self.contact_motion()
        before = copy.deepcopy(motion.target_tracker.__dict__)
        evidence = motion.current_contact_target(scan, 1.)
        self.assertIsNotNone(evidence)
        self.assertEqual(evidence['role'], 'cone_like_support')
        self.assertIsNone(evidence['track_id'])
        self.assertEqual(evidence['source_seq'], 10)
        self.assertEqual(evidence['orbit_track_id'], motion.orbit_track_id)
        self.assertEqual(evidence['semantic_class'], 'unknown')
        self.assertEqual(evidence['support_ranges_m'], [scan['ranges'][i] for i in evidence['support_bins']])
        self.assertEqual(motion.target_tracker.__dict__, before)
        decision = motion.update(scan, 0., 1., route.feedback(10, motion.servo))
        self.assertEqual(decision['phase'], 'drive')
        self.assertEqual(decision['motor'], 1550)
        motion.phase = 'quality_wait'
        self.assertIsNotNone(motion.current_contact_target(scan, 1.))
        motion.first_pass_evidence = {'source_seq': 9}  # a cone is still a cone after its pass
        self.assertIsNotNone(motion.current_contact_target(scan, 1.))

    def test_contact_permission_excludes_stale_default_inactive_phases_and_board_ends(self):
        motion, scan = self.contact_motion()
        partial = copy.deepcopy(scan)
        partial['ranges'][259] = None  # a bounded edge null keeps the witness, never as support
        evidence = motion.current_contact_target(partial, 1.)
        self.assertNotIn(259, evidence['support_bins'])
        self.assertIsNone(motion.current_contact_target(scan, 1.31))
        board = copy.deepcopy(scan)
        # Incoming board end seen past the rear-left corner (trial32 scan546
        # shape): a straight piece receding along the line of sight.
        board['ranges'] = [3.]*360
        board['ranges'][228:234] = [6.77, .29, .30, .43, .56, 4.29]
        self.assertIsNone(motion.current_contact_target(board, 1.))
        for phase in ('coast', 'locked', 'presteer'):
            motion.phase = phase
            self.assertIsNone(motion.current_contact_target(scan, 1.))
        motion.phase = 'drive'
        motion.continue_route = False
        self.assertIsNone(motion.current_contact_target(scan, 1.))

    def test_second_contact_witness_in_every_post_pass_stage(self):
        motion, seq, bearing = self.second()
        for seq, distance, size in ((9, .8, 9), (10, .6, 11), (11, .4, 15), (12, .3, 21)):
            scan = route.observation(seq, heading=0.)
            route.add_object(scan, bearing=bearing, distance=distance, size=size)
            motion.update(scan, 0., seq/10, route.feedback(seq, motion.servo))
        scan = route.observation(13, heading=0.)
        route.add_object(scan, bearing=bearing, distance=.2, size=21)
        evidence = motion.current_contact_target(scan, 1.3)
        self.assertIsNotNone(evidence)
        self.assertEqual(evidence['second_track_id'], motion.second_track_id)
        self.assertEqual(evidence['role'], 'cone_like_support')
        motion.second_pass_evidence = {'source_seq': 12}
        self.assertIsNotNone(motion.current_contact_target(scan, 1.3))
        motion.second_pass_evidence = None
        motion.route_stage = 'corridor_follow'
        self.assertIsNotNone(motion.current_contact_target(scan, 1.3))
