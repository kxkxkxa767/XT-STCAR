"""Current second-support return geometry; no vehicle, fabricated pass, or yaw timer."""
import copy
import json
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).parents[1]))
from autonomy_live import ProbeClearanceError, probe_clearance
from maneuver_sequence import ManeuverSequence
from test_maneuver_sequence import corridor


FIXTURE = Path(__file__).parent/'fixtures'/'route-55-second-exit-board.json'


def scene(seq, *, cone=(.1, -.45), heading=-25., offset=-.2, width=2.4,
          world_translation=(0., 0.), world_rotation=0., present=True):
    """Raycast a round support and two walls from one consistent local scene.

    Rigid world transforms move the car, walls and cone together. Their local
    observations must stay equivalent; cone placement and corridor direction
    are separate parameters, not a remembered practice-course location.
    """
    rotation = math.radians(world_rotation)
    co, si = math.cos(rotation), math.sin(rotation)
    world_cone = (world_translation[0]+co*cone[0]-si*cone[1],
                  world_translation[1]+si*cone[0]+co*cone[1])
    delta = (world_cone[0]-world_translation[0], world_cone[1]-world_translation[1])
    center = (co*delta[0]+si*delta[1], -si*delta[0]+co*delta[1])
    angle = math.radians(heading)
    normal = (-math.sin(angle), math.cos(angle))
    ranges = []
    for index in range(360):
        ray = (math.cos(math.radians(index)), -math.sin(math.radians(index)))
        choices = [8.]
        dot = normal[0]*ray[0]+normal[1]*ray[1]
        if abs(dot) > 1e-12:
            choices += [distance for edge in (offset-width/2, offset+width/2)
                        if (distance := edge/dot) > 0]
        projection = center[0]*ray[0]+center[1]*ray[1]
        discriminant = projection*projection-(center[0]**2+center[1]**2-.09**2)
        if present and discriminant >= 0:
            distance = projection-math.sqrt(discriminant)
            if distance > .02:
                choices.append(distance)
        ranges.append(min(choices))
    geometry = corridor(heading)
    geometry.update(center_offset_left_m=offset, width_m=width)
    return {'seq': seq, 'at_ms': seq*100, 'received_at': seq/10,
            'frame_id': 'lidar_origin_coarse_body_heading', 'ranges': ranges,
            'corridor_candidates': [geometry], 'wall_candidates': []}


def adopted(seq, servo, *, ack=True):
    return {'armed': True, 'motor': 1550, 'servo': servo,
            'tick': seq*100, 'seq': seq, 'command_acked': ack}


def seed_second(scans, *, servo=1580):
    """Seed only an already performed first pass and adopted second handover.

    The second identity itself is confirmed by the real tracker on three
    supplied scans, not by setting confirmed=True on a manufactured target.
    Every later command and pass is produced by public update().
    """
    motion = ManeuverSequence(0., continue_route=True)
    for scan in scans:
        target = motion.second_tracker.update(scan, scan['received_at'])
    if target is None or not target['confirmed']:
        raise AssertionError('fixture did not naturally confirm the second support')
    now = scans[-1]['received_at']
    motion.phase, motion.route_stage = 'drive', 'second_orbit'
    motion.drive_since = motion.orbit_since = motion.second_orbit_since = now
    motion.handover_observed = motion.second_target_handover_observed = True
    motion.orbit_track_id = 7
    motion.first_pass_evidence = {'track_id': 7, 'source_seq': 0,
        'basis': 'synthetic_precondition_first_support_already_passed'}
    motion.second_target = target
    motion.second_track_id = target['track_id']
    motion.second_tracker.enable_close_maintenance(target['track_id'])
    motion._second_role_anchor = copy.deepcopy(target)
    motion.servo = motion.steering_target = servo
    motion.last_change = now
    return motion


class SecondOrbitExitPreparationTests(unittest.TestCase):
    def seeded(self, *, servo=1580, **geometry):
        return seed_second([scene(seq, **geometry) for seq in (1, 2, 4)], servo=servo)

    def step(self, motion, scan, *, actual=None, ack=True, age=0.):
        return motion.update(scan, age, scan['received_at']+age,
            adopted(scan['seq'], motion.servo if actual is None else actual, ack=ack))

    def reach_center(self, motion, *, start=5, **geometry):
        result = None
        for seq in range(start, start+6):
            before = motion.servo
            result = self.step(motion, scene(seq, **geometry))
            self.assertEqual(result['phase'], 'drive')
            self.assertTrue(result['second_return_preparing'])
            self.assertGreaterEqual(result['servo'], 1500)
            self.assertLessEqual(result['servo'], before)
            self.assertIsNone(result['second_pass_evidence'])
            if result['servo'] == 1500:
                return seq, result
        self.fail('current clear return geometry never released the adopted left command')

    def test_real55_prepares_center_then_right_before_the_recorded_board_contact(self):
        frames = json.loads(FIXTURE.read_text())['frames']
        scans = []
        for frame in frames:
            scan = copy.deepcopy(frame['scan'])
            # Explicit offline receipt model preserves publication intervals;
            # these are not claimed to be the old run's exact host timestamps.
            scan['received_at'] = (scan['at_ms']-frames[0]['scan']['at_ms'])/1000
            scans.append(scan)
        self.assertEqual([s['seq'] for s in scans[:3]], [680, 682, 684])
        motion = seed_second(scans[:3], servo=frames[2]['recorded_control']['servo'])
        first_pass = copy.deepcopy(motion.first_pass_evidence)
        decisions = {}
        for scan in scans[3:]:
            if scan['seq'] > 698:
                break  # changed commands cannot claim the later old path as a new physical result
            decisions[scan['seq']] = self.step(motion, scan)
        self.assertTrue(any(r['second_return_preparing'] for seq, r in decisions.items() if seq <= 694))
        self.assertLessEqual(decisions[696]['servo'], 1500)
        self.assertLess(decisions[698]['servo'], 1500)
        self.assertIsNotNone(decisions[698]['second_center_ack'])
        self.assertTrue(decisions[698]['route_right_output_authorized'])
        # The saved stream is 5 Hz. Its missing intermediate scan makes the
        # 696 -> 698 bearing jump exceed identity association: 698 may only
        # hold the right command already earned from current support at 696.
        self.assertIsNone(decisions[698]['second_target'])
        self.assertEqual(decisions[698]['second_return_evidence']['source_seq'], 696)
        self.assertEqual(decisions[698]['servo'], decisions[696]['servo'])
        self.assertTrue(all(r['second_pass_evidence'] is None for r in decisions.values()))
        self.assertEqual(motion.first_pass_evidence, first_pass)

    def test_new_return_waits_for_actual_center_ack_before_any_right_command(self):
        motion = self.seeded()
        seq, _ = self.reach_center(motion)
        # A neutral command alone is not neutral adoption. Neither stale left
        # feedback nor a neutral value with no command ACK permits right PWM.
        for i, (actual, ack) in enumerate(((1540, False), (1500, False)), start=1):
            result = self.step(motion, scene(seq+i), actual=actual, ack=ack)
            self.assertEqual(result['servo'], 1500)
            self.assertFalse(result['route_right_output_authorized'])
            self.assertIsNone(result['second_pass_evidence'])
        result = self.step(motion, scene(seq+3), actual=1500, ack=True)
        self.assertLess(result['servo'], 1500)
        self.assertTrue(result['route_right_output_authorized'])
        self.assertEqual(result['second_center_ack']['servo'], 1500)

    def test_return_follows_current_geometry_across_layouts_and_world_transforms(self):
        layouts = [(-15., -.1, (.1, -.45)), (-30., -.25, (.15, -.5)),
                   (-45., -.35, (.05, -.8)), (-20., -.15, (1.1, -.5))]
        for heading, offset, cone in layouts:
            outputs = []
            for translation, rotation in (((0., 0.), 0.), ((4.8, -2.1), 73.),
                                          ((-6.4, 3.2), -118.)):
                geometry = dict(heading=heading, offset=offset, cone=cone,
                                world_translation=translation, world_rotation=rotation)
                with self.subTest(geometry=geometry):
                    motion = self.seeded(**geometry)
                    seq, _ = self.reach_center(motion, **geometry)
                    result = self.step(motion, scene(seq+1, **geometry))
                    self.assertLessEqual(result['servo'], 1500)
                    self.assertIsNone(result['second_pass_evidence'])
                    outputs.append(result['servo'])
            if len(outputs) == 3:
                self.assertEqual(outputs, [outputs[0]]*3)

    def test_no_current_corridor_cannot_prepare_or_invent_a_right_exit(self):
        motion = self.seeded()
        for seq in range(5, 9):
            scan = scene(seq)
            scan['corridor_candidates'] = []
            result = self.step(motion, scan)
            self.assertFalse(result['second_return_preparing'])
            self.assertIsNone(result['second_pass_evidence'])
            self.assertFalse(result['two_target_observed_pass_complete'])

    def test_neutral_corridor_preserves_a_current_safe_clockwise_bypass(self):
        geometry = dict(heading=0., offset=0., cone=(.1, -.6))
        motion = self.seeded(servo=1500, **geometry)
        result = self.step(motion, scene(5, **geometry))
        self.assertTrue(result['second_return_preparing'])
        self.assertEqual(result['second_return_evidence']['corridor_target_pwm'], 1500)
        self.assertLess(result['servo'], 1500)
        self.assertTrue(result['route_right_output_authorized'])
        self.assertEqual(result['second_center_ack']['servo'], 1500)
        self.assertIsNone(result['second_pass_evidence'])

    def test_far_forward_support_cannot_be_cut_off_by_a_short_clear_right_arc(self):
        # The support is beyond the 0.7 m obstacle horizon. Each available
        # right arc passes on its wrong side; straight keeps it on the right.
        geometry = dict(cone=(1.5, -.4))
        motion = self.seeded(servo=1500, **geometry)
        for seq in range(5, 8):
            result = self.step(motion, scene(seq, **geometry))
            evidence = result['second_return_evidence']
            self.assertGreater(evidence['observed_frontmost_x_m'], .7)
            self.assertLess(evidence['corridor_target_pwm'], 1500)
            self.assertEqual(evidence['clear_candidates'], [1500])
            self.assertEqual(result['servo'], 1500)
            self.assertIsNone(result['second_pass_evidence'])

    def test_left_support_and_ambiguous_corridor_do_not_create_preparation(self):
        motion = self.seeded(cone=(.1, .45))
        result = self.step(motion, scene(5, cone=(.1, .45)))
        self.assertFalse(result['second_return_preparing'])
        self.assertGreater(result['servo'], 1500)
        motion = self.seeded()
        for seq in range(5, 8):
            scan = scene(seq)
            scan['corridor_candidates'].append(corridor(25.))
            result = self.step(motion, scan)
            self.assertFalse(result['second_return_preparing'])
            self.assertIsNone(result['second_return_evidence'])
            self.assertIsNone(result['second_pass_evidence'])

    def test_missing_corridor_after_prepare_only_releases_and_does_not_renew_proof(self):
        motion = self.seeded(servo=1620)
        result = self.step(motion, scene(5))
        self.assertTrue(result['second_return_preparing'])
        source = result['second_return_evidence']['source_seq']
        for seq in range(6, 9):
            before = motion.servo
            scan = scene(seq)
            scan['corridor_candidates'] = []
            result = self.step(motion, scan)
            self.assertTrue(1500 <= result['servo'] <= before)
            self.assertEqual(result['second_return_evidence']['source_seq'], source)
            self.assertIsNone(result['second_pass_evidence'])

    def test_right_command_cannot_grow_when_current_corridor_disappears(self):
        motion = self.seeded(servo=1500)
        result = self.step(motion, scene(5))
        self.assertLess(result['servo'], 1500)
        source = result['second_return_evidence']['source_seq']
        for seq in range(6, 9):
            before = motion.servo
            scan = scene(seq)
            scan['corridor_candidates'] = []
            result = self.step(motion, scan)
            self.assertGreaterEqual(result['servo'], before)
            self.assertLessEqual(result['servo'], 1500)
            self.assertEqual(result['second_return_evidence']['source_seq'], source)

    def test_held_support_never_renews_return_proof_or_increases_right(self):
        motion = self.seeded(servo=1500)
        result = self.step(motion, scene(5))
        before = result['servo']
        evidence = copy.deepcopy(result['second_return_evidence'])
        result = self.step(motion, scene(6, present=False))
        self.assertIsNone(result['second_target'])
        self.assertTrue(result['second_return_preparing'])
        self.assertGreaterEqual(result['servo'], before)
        self.assertLessEqual(result['servo'], 1500)
        self.assertEqual(result['second_return_evidence'], evidence)
        self.assertIsNone(result['second_pass_evidence'])

    def test_held_release_requires_adoption_and_original_ack_timeout_still_coasts(self):
        motion = self.seeded(servo=1620)
        result = self.step(motion, scene(5))
        requested = result['servo']
        self.assertTrue(1500 < requested < 1620)
        result = self.step(motion, scene(6, present=False), actual=1620, ack=False)
        self.assertEqual(result['servo'], requested)
        self.assertIsNotNone(motion.right_exit_adoption_wait_since)
        # The next full frames preserve identity, but do not erase the
        # unadopted-command deadline or authorize another release step.
        for seq in (7, 8):
            result = self.step(motion, scene(seq), actual=1620, ack=False)
            self.assertEqual(result['servo'], requested)
            self.assertEqual(result['phase'], 'drive')
        for seq in (9, 10):
            result = self.step(motion, scene(seq), actual=1620, ack=False)
            if result['phase'] != 'drive':
                break
        self.assertEqual(result['phase'], 'coast')
        self.assertEqual(result['motor'], 1500)
        self.assertIn('adoption_unconfirmed', result['reason'])
        self.assertIsNone(result['second_pass_evidence'])

    def test_preparation_does_not_replace_the_complete_rear_pass_window(self):
        motion = self.seeded(servo=1500, cone=(.65, -.55))
        first_pass = copy.deepcopy(motion.first_pass_evidence)
        rear_rows = []
        positions = (.5, .35, .2, .05, -.1, -.25, -.35, -.45, -.5, -.55)
        for seq, x in enumerate(positions, start=5):
            scan = scene(seq, cone=(x, -.55))
            result = self.step(motion, scan)
            target = result['second_target']
            self.assertIsNotNone(target)
            self.assertEqual(target['source_seq'], seq)
            points = [(scan['ranges'][i]*math.cos(math.radians(i)),
                       -scan['ranges'][i]*math.sin(math.radians(i)))
                      for i in target['support_bins']]
            if max(x for x, _ in points) <= -.18 and min(-y for _, y in points)-.14 >= .08:
                rear_rows.append(scan)
            else:
                rear_rows.clear()
            mature = (len(rear_rows) >= 3
                      and scan['at_ms']-rear_rows[0]['at_ms'] >= 250)
            if not mature:
                self.assertIsNone(result['second_pass_evidence'])
                self.assertEqual(result['route_stage'], 'second_orbit')
            else:
                self.assertIsNotNone(result['second_pass_evidence'])
                self.assertEqual(result['route_stage'], 'light_turn')
                self.assertFalse(result['second_return_preparing'])
            self.assertEqual(motion.first_pass_evidence, first_pass)
        self.assertTrue(mature)

    def test_quality_pause_releases_only_after_neutral_and_servo_adoption(self):
        motion = self.seeded(servo=1660)
        prepared = self.step(motion, scene(5))
        evidence = copy.deepcopy(prepared['second_return_evidence'])
        self.assertTrue(motion.begin_post_pass_quality_pause(
            .5, adopted(5, motion.servo), scan=scene(5)))
        for seq in range(6, 16):
            before = motion.servo
            scan = scene(seq)
            # A right-rear return outside the 8 cm body gate would ordinarily
            # ask proximity avoidance to add left. It must not defeat this
            # stage's neutral release or create right while power is neutral.
            scan['ranges'][120] = .26
            control = adopted(seq, before)
            control.update(motor=1500, neutral_acked=seq != 6)
            if seq == 8:
                control.update(servo=motion.quality_wait_previous_servo,
                               command_acked=False)
            result = motion.update(scan, 0., seq/10, control,
                                   quality_clear=True, quality_resume_ready=False)
            self.assertEqual(result['phase'], 'quality_wait')
            self.assertEqual(result['motor'], 1500)
            self.assertEqual(result['route_avoid']['side'], 'right')
            self.assertFalse(result['route_right_output_authorized'])
            self.assertTrue(1500 <= result['servo'] <= before)
            if seq in (6, 8):
                self.assertEqual(result['servo'], before)
            self.assertEqual(result['second_return_evidence'], evidence)
            self.assertIsNone(result['second_pass_evidence'])
        self.assertEqual(result['servo'], 1500)

    def test_lost_second_identity_clears_preparation_without_fabricating_a_pass(self):
        motion = self.seeded()
        self.assertTrue(self.step(motion, scene(5))['second_return_preparing'])
        first_pass = copy.deepcopy(motion.first_pass_evidence)
        for seq in range(6, 11):
            result = self.step(motion, scene(seq, present=False))
        self.assertFalse(result['second_return_preparing'])
        self.assertIsNone(result['second_pass_evidence'])
        self.assertEqual(motion.first_pass_evidence, first_pass)

    def test_stale_scan_and_real_outer_board_still_refuse_motion(self):
        motion = self.seeded()
        self.assertTrue(self.step(motion, scene(5))['second_return_preparing'])
        stale = self.step(motion, scene(6), age=.3)
        self.assertTrue(stale['lock_requested'])
        self.assertEqual((stale['motor'], stale['servo']), (1500, 1500))
        frames = json.loads(FIXTURE.read_text())['frames']
        scan = copy.deepcopy(next(f['scan'] for f in frames if f['scan']['seq'] == 704))
        scan['received_at'] = 1.
        witness_motion = ManeuverSequence(0., continue_route=True)
        witness_motion.phase = 'drive'
        witness = witness_motion.current_contact_target(scan, 1.)
        with self.assertRaisesRegex(ProbeClearanceError, 'probe_obstacle_close_body'):
            probe_clearance(scan, {'control': .01, 'lidar': .01}, camera_required=False,
                            clearance_profile='maneuver', body_contact_target=witness)


if __name__ == '__main__':
    unittest.main()
