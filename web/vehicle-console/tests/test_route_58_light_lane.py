"""Round 58 (Claude): next-lane turn after the second support, light lane and
finish stop (round 63: straight law and coasting profile replace the .45-.70
m/s slow band; round 101: the coast aimed at a whole-car stop, pushed again
while rolling). Synthetic scans; no traffic-light vision; no physical claim."""
import copy
import math
import unittest

import test_maneuver_route as route
from maneuver_sequence import (ManeuverSequence, LANE_STOP_DECEL_MPS2, ROUTE_POST_PASS_PWM_STEP,
                               ROUTE_SLOW_MOTOR, ROUTE_MAX_DRIVE_S, FRONT_BODY_EXTENT_M,
                               LANE_FINISH_BODY_GAP_M, LANE_STOP_DECEL_MIN_MPS2)
from turn_motion import TRIAL_MOTOR
import maneuver_sequence as ms

HALF = 1.0  # lane half width (m)


def box(ahead, *, half=HALF):
    """Ranges inside a straight lane (walls at +-half) ending in a board ``ahead`` m away."""
    ranges = []
    for i in range(360):
        a = -math.radians(i)
        dx, dy = math.cos(a), math.sin(a)
        best = 6.
        if dx > 1e-9:
            best = min(best, ahead/dx)
        if abs(dy) > 1e-9:
            best = min(best, half/abs(dy))
        ranges.append(round(best, 3))
    return ranges


def ms_zone():
    import maneuver_sequence
    return maneuver_sequence.LANE_FINISH_ZONE_GAP_M


class LightLaneTests(unittest.TestCase):
    def lane_motion(self):
        helper = route.ManeuverRouteTests()
        helper.setUp()
        motion = helper.route('second_orbit')
        motion.route_stage, motion.light_since = 'light_lane', .4
        motion.second_pass_evidence = {'track_id': 2, 'source_seq': 4}
        return motion

    def scan(self, seq, ahead, corridor=True):
        value = route.observation(seq, heading=0.)
        value['ranges'] = box(ahead)
        value['corridor_candidates'][0].update(center_offset_left_m=0., width_m=2*HALF)
        if not corridor:
            value['corridor_candidates'] = []
        return value

    def drive(self, motion, speeds, start=4., seq0=5):
        """Feed scans closing on the board at the given per-step speeds (m/s)."""
        ahead, results, motor = start, [], ROUTE_SLOW_MOTOR
        for k, v in enumerate(speeds):
            seq = seq0+k
            ahead -= v*.1
            result = motion.update(self.scan(seq, ahead), 0., seq/10, route.feedback(seq, motion.servo, motor))
            motor = result['motor']
            results.append(result)
            if result['phase'] != 'drive':
                break
        return results

    def test_far_lane_drives_1550_near_lane_coasts_and_a_stop_restarts(self):
        # Round 63: far from the end 1.0 m/s keeps 1550 (was coasted above .70).
        # Round 101: "far" = the aimed coast stops short of the target and the
        # guard (the weakest LANE_STOP_DECEL_MIN_MPS2 before a coast is measured)
        # still keeps the board; within it the lane coasts; a car too slow for
        # 1550 to act coasts on, and from rest gets a 1560 kick.
        motion = self.lane_motion()
        far = self.drive(motion, [1.0]*5, start=5.9)
        self.assertEqual(far[0]['motor'], ROUTE_SLOW_MOTOR)   # speed not yet known
        self.assertEqual(far[-1]['motor'], ROUTE_SLOW_MOTOR)  # the straight: 1550
        self.assertGreater(far[-1]['lane_feedback']['predicted_stop_gap_m'], ms.LANE_FINISH_TARGET_GAP_M)
        near = self.drive(motion, [1.0]*3, start=2.5, seq0=10)
        self.assertTrue(all(r['motor'] == 1500 for r in near))
        self.assertLessEqual(near[0]['lane_feedback']['worst_stop_gap_m'], ms.LANE_FINISH_MIN_GAP_M)  # the guard cut
        # A measured coasting car too slow for 1550 to act (LANE_PUSH_ROLLING_MPS).
        motion._lane_speed = lambda scan, ahead, **_: .3
        slow = self.drive(motion, [.3]*4, start=3.5, seq0=14)
        self.assertTrue(all(r['motor'] == 1500 for r in slow))
        self.assertGreater(slow[-1]['lane_feedback']['predicted_stop_gap_m'],
                           ms.LANE_FINISH_TARGET_GAP_M+ms.LANE_TOPUP_BAND_M)
        motion._lane_speed = lambda scan, ahead, **_: .05
        crawl = self.drive(motion, [.05]*4, start=3.4, seq0=18)
        # At rest: a 1560 kick once LANE_KICK_REST_S has passed since the last push.
        self.assertIn(TRIAL_MOTOR, [r['motor'] for r in crawl])
        self.assertTrue(all(r['phase'] == 'drive' for r in far+near+slow+crawl))
        self.assertTrue(all(r['reason'] == 'light_lane_follow_straight' for r in near+slow+crawl))

    def test_finish_ends_stopped_with_the_whole_car_in_the_zone(self):
        # Round 101 (rules: all four wheels in the 80 cm zone at the stop): the
        # stage ends stopped with the front within LANE_FINISH_BODY_GAP_M, or
        # still coasting once a coast at the strong measured decel stops there.
        motion = self.lane_motion()
        speeds = [max(0., .9-.055*k) for k in range(25)]+[0.]*5
        results = self.drive(motion, speeds, start=1.3)
        last = results[-1]
        self.assertEqual(last['phase'], 'coast')
        self.assertEqual(last['reason'], 'route_finish_zone_stop')
        evidence = last['finish_evidence']
        self.assertIn(evidence['basis'], ('stopped_with_the_whole_car_in_the_finish_zone',
                                          'coast_stops_the_whole_car_in_the_finish_zone'))
        if evidence['basis'].startswith('stopped'):
            self.assertLessEqual(evidence['front_gap_m'], LANE_FINISH_BODY_GAP_M)
        else:
            self.assertLessEqual(evidence['front_gap_m']-evidence['speed_aim_mps']**2/(2*evidence['strong_decel_mps2']),
                                 LANE_FINISH_BODY_GAP_M)
        self.assertEqual(evidence['front_gap_m'], evidence['front_range_m']-FRONT_BODY_EXTENT_M)
        self.assertEqual(last['motor'], 1500)
        self.assertEqual(last['servo'], last['route_end_coast_servo'])
        self.assertFalse(evidence['finish_zone_certified'])

    def test_stop_with_only_the_front_in_the_zone_kicks_unless_that_could_reach_the_board(self):
        # Round 101: a restart kick from rest moved the car .08-.36 m (runs
        # 98-100); within LANE_FINISH_MIN_GAP_M+LANE_KICK_TRAVEL_M no kick.
        # Round 105: there a shorter creep instead (test_route_105_finish_creep).
        for start, frames in ((.2+.47, ms.LANE_CREEP_FRAMES), (.2+.7, ms.LANE_KICK_FRAMES)):
            motion = self.lane_motion()
            results = self.drive(motion, [0.]*6, start=start)
            with self.subTest(gap=start-.2):
                self.assertEqual(results[-1]['phase'], 'drive')
                self.assertEqual([r['motor'] for r in results].count(TRIAL_MOTOR), frames)

    def test_unknown_speed_coasts_near_the_board_and_decides_once_measured(self):
        # Round 64: an unmeasured speed (1.3 m/s fallback) within the weakest
        # coast distance gives no power and no cut; round 101: the stage then
        # ends once the measured car has stopped in the zone.
        motion = self.lane_motion()
        result = motion.update(self.scan(5, 2.0), 0., .5, route.feedback(5, motion.servo, ROUTE_SLOW_MOTOR))
        self.assertIsNone(result['finish_evidence'])
        self.assertEqual(result['phase'], 'drive')
        self.assertEqual(result['motor'], 1500)
        self.assertFalse(result['lane_feedback']['speed_known'])
        speeds = [max(0., 1.1-.055*k) for k in range(25)]+[0.]*5
        results = self.drive(motion, speeds, start=2., seq0=6)
        self.assertEqual(results[-1]['reason'], 'route_finish_zone_stop')
        self.assertNotIn(TRIAL_MOTOR, [r['motor'] for r in results])
        self.assertLess(LANE_STOP_DECEL_MIN_MPS2, LANE_STOP_DECEL_MPS2)

    def test_lane_starts_with_the_speed_measured_while_aligning(self):
        helper = route.ManeuverRouteTests()
        helper.setUp()
        motion = helper.route('second_orbit')
        motion.route_stage, motion.light_since = 'light_turn', .4
        motion.light_turn_started = -.6  # the turn already ran (round 63 entry)
        motion.second_pass_evidence = {'track_id': 2, 'source_seq': 4}
        ahead = 3.2
        for seq in range(5, 12):
            ahead -= .06
            result = motion.update(self.scan(seq, ahead), 0., seq/10,
                                   route.feedback(seq, motion.servo, ROUTE_SLOW_MOTOR))
            if result['route_stage'] == 'light_lane':
                break
        self.assertEqual(result['route_stage'], 'light_lane')
        result = motion.update(self.scan(seq+1, ahead-.06), 0., (seq+1)/10,
                               route.feedback(seq+1, motion.servo, ROUTE_SLOW_MOTOR))
        self.assertAlmostEqual(result['lane_feedback']['speed_mps'], .6, delta=.05)

    def test_coasting_pulse_is_adopted_and_records_a_light_lane_center_ack(self):
        # Round 82 (car runs 80/81): in the light lane an ACKed centre while
        # coasting is the centre proof too (was: only under drive power, which
        # left right blocked while the lane coasted to the finish).
        motion = self.lane_motion()
        motion._route_center_ack = None
        motion.servo = motion.steering_target = 1500
        result = motion.update(self.scan(5, 4.), 0., .5, route.feedback(5, 1500, 1500))
        self.assertEqual(result['phase'], 'drive')
        self.assertNotEqual(result['reason'], 'route_waiting_adopted_command')
        self.assertEqual(motion._route_center_ack['motor'], 1500)
        motion.update(self.scan(6, 3.9), 0., .6, route.feedback(6, 1500, ROUTE_SLOW_MOTOR))
        self.assertEqual(motion._route_center_ack['motor'], ROUTE_SLOW_MOTOR)

    def test_lane_steering_walls_then_measured_lane_then_free_space(self):
        # Round 63: two fitted side boards -> the start-straight wall law;
        # without them the measured lane (+-55), else free space.
        motion = self.lane_motion()
        result = motion.update(self.scan(5, 4.), 0., .5, route.feedback(5, motion.servo, ROUTE_SLOW_MOTOR))
        self.assertEqual(result['lane_feedback']['steering_source'], 'straight_lane_walls')
        self.assertEqual(result['lane_feedback']['nominal_target_pwm'], 1500)  # centered, aligned
        motion = self.lane_motion()
        value = self.scan(5, 4.)
        for i in range(225, 316):
            value['ranges'][i] = None  # no left board fit
        value['corridor_candidates'][0].update(center_offset_left_m=.3)
        result = motion.update(value, 0., .5, route.feedback(5, motion.servo, ROUTE_SLOW_MOTOR))
        self.assertEqual(result['lane_feedback']['steering_source'], 'measured_lane')
        self.assertGreater(result['lane_feedback']['nominal_target_pwm'], 1500)
        self.assertLessEqual(result['lane_feedback']['nominal_target_pwm'], 1555)
        value = self.scan(6, 3.9, corridor=False)
        for i in range(225, 316):
            value['ranges'][i] = None
        result = motion.update(value, 0., .6, route.feedback(6, motion.servo, ROUTE_SLOW_MOTOR))
        self.assertEqual(result['lane_feedback']['steering_source'], 'free_space')

    def test_route_budget_covers_the_whole_route(self):
        self.assertEqual(ROUTE_MAX_DRIVE_S, 40.)  # round 64: whole lap from the start zone
        ManeuverSequence(0., max_drive_s=40., continue_route=True)
        with self.assertRaises(ValueError):
            ManeuverSequence(0., max_drive_s=40.1, continue_route=True)


class NextLaneTurnTests(unittest.TestCase):
    def test_light_turn_uses_the_post_pass_step(self):
        helper = route.LightLaneTests()
        motion = helper.light()
        motion.servo = motion.steering_target = 1500
        result = helper.step(motion, 5, route.junction(-1.))  # opening on the right
        self.assertEqual(result['servo'], 1500-ROUTE_POST_PASS_PWM_STEP)

    def second(self, bearing, distance, *, opening_right=True):
        helper = route.ManeuverRouteTests()
        helper.setUp()
        motion = helper.route('second_orbit')
        motion.second_track_id, motion.second_orbit_since = 2, .4
        motion.second_center_ack = copy.deepcopy(motion._route_center_ack)
        value = route.observation(5, heading=0.)
        value['ranges'] = route.segment_ranges(route.junction(-1. if opening_right else 1.))
        route.add_object(value, bearing=bearing, distance=distance, size=15)
        value['corridor_candidates'] = []
        c = [c for c in route._candidates(value['ranges'], bearing_range_rad=(-math.pi, math.pi), tolerant=True)
             if abs(math.degrees(c['bearing_left_rad'])-bearing) < 5][0]
        motion.second_target = {**c, 'track_id': 2, 'confirmed': True, 'source_seq': 5,
                                'source_at_ms': 500, 'source_received_at': .5}
        return motion, motion._second_orbit_update(value, .5, route.feedback(5, motion.servo, 1550))

    def test_support_beside_the_body_turns_into_the_open_next_lane(self):
        _, result = self.second(-95., .5)
        lane = result['second_feedback']['next_lane_free_space']
        self.assertIsNotNone(lane)
        self.assertLess(lane['best_heading_rad'], 0.)
        self.assertLessEqual(result['natural_steering_target'], lane['target_pwm'])

    def test_support_still_ahead_or_next_lane_on_the_left_never_forces_right(self):
        _, ahead = self.second(-30., .8)
        self.assertNotIn('next_lane_free_space', ahead['second_feedback'])
        _, left = self.second(-95., .5, opening_right=False)
        free = left['second_feedback']['next_lane_free_space']
        self.assertFalse(free['applied'])  # only ever takes a target further right


if __name__ == '__main__':
    unittest.main()


import test_route_control as service  # noqa: E402
from unittest.mock import patch  # noqa: E402


class LightLaneServiceTests(unittest.TestCase):
    setUp = service.RouteControlTests.setUp
    tearDown = service.RouteControlTests.tearDown
    scan = staticmethod(service.RouteControlTests.scan)
    tick = service.RouteControlTests.tick
    observe = service.RouteControlTests.observe
    start_compact_trial = service.RouteControlTests.start_compact_trial
    drive_compact_trial = service.RouteControlTests.drive_compact_trial
    enter_compact_orbit = service.RouteControlTests.enter_compact_orbit
    compact_scan = service.RouteControlTests.compact_scan
    incomplete = service.RouteControlTests.incomplete
    target_bins = service.RouteControlTests.target_bins
    reset_case = service.RouteControlTests.reset_case

    def light_lane(self):
        _, session = self.enter_compact_orbit(continue_route=True)
        motion = session['turn_motion']
        motion.first_pass_evidence = {'source_seq': self.console.scan['seq']}
        motion.right_exit_center_ack = {'servo': 1500}
        motion.second_pass_evidence = {'source_seq': self.console.scan['seq']}
        motion.route_stage, motion.light_since = 'light_lane', self.now
        return session, motion

    def test_light_lane_motor_outputs_are_accepted(self):
        for motor in (1500, ROUTE_SLOW_MOTOR, TRIAL_MOTOR):
            with self.subTest(motor=motor):
                self.reset_case()
                session, motion = self.light_lane()
                decision = copy.deepcopy(session['report']['turn'])
                decision.update(servo=motion.servo, motor=motor, phase='drive', route_stage='light_lane',
                                quality_resume_pending=False, stop_requested=False, lock_requested=False)
                with patch.object(motion, 'update', return_value=decision):
                    self.tick(round(self.now+.02, 6), self.compact_scan(self.console.scan['seq']+1))
                self.assertIs(self.console.auto_session, session)
                self.assertEqual(self.outputs[-1]['motor'], motor)

    def test_quality_fault_during_a_coasting_pulse_waits_without_power(self):
        session, motion = self.light_lane()
        motion.lane_motor = 1500
        self.console.status['control']['motor'] = 1500  # adopted neutral coasting pulse
        self.observe(round(self.now+.1, 6), self.incomplete(self.console.scan['seq']+1))
        self.assertIs(self.console.auto_session, session)
        self.assertEqual(session['phase'], 'quality_wait')
        self.assertEqual(session['quality_wait_kind'], 'post_pass_pause')
        self.assertIs(session['quality_wait_entry_neutral'], True)
        self.assertEqual(self.outputs[-1]['motor'], 1500)
        # Its own neutral command is still pending while the motor reads 1500.
        self.observe(round(self.now+.1, 6), self.incomplete(self.console.scan['seq']+1))
        self.assertIs(self.console.auto_session, session)
        self.assertEqual(session['phase'], 'quality_wait')
