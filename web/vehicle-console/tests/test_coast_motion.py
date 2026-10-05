"""Offline geometry and publication-clock checks; never opens a device or socket."""
import importlib.util
import json
import math
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location('coast_motion_test', Path(__file__).parents[1]/'coast_motion.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def room_scan(seq, x=0., y=0., yaw=0., at_ms=None, front=4., rear=-3., half_width=1.):
    ranges = []
    for angle in range(360):
        dx, dy = math.cos(math.radians(angle)+yaw), math.sin(math.radians(angle)+yaw)
        choices = []
        if abs(dx) > 1e-9:
            choices.extend(r for r in [(front-x)/dx, (rear-x)/dx] if r > 0)
        if abs(dy) > 1e-9:
            choices.extend(r for r in [(half_width-y)/dy, (-half_width-y)/dy] if r > 0)
        distance = min(choices)
        ranges.append(distance if distance <= 12 else None)
    return {'seq': seq, 'at_ms': seq*100 if at_ms is None else at_ms,
            'frame_id': 'lidar_origin_coarse_body_heading', 'ranges': ranges}


def parallel_scan(seq):
    ranges = []
    for angle in range(360):
        sine = abs(math.sin(math.radians(angle)))
        distance = .5/sine if sine > 1e-9 else float('inf')
        ranges.append(distance if distance <= 12 else None)
    return {'seq': seq, 'at_ms': seq*100,
            'frame_id': 'lidar_origin_coarse_body_heading', 'ranges': ranges}


class CoastMotionTests(unittest.TestCase):
    def setUp(self):
        self.motion = MODULE.CoastMotion()

    def test_requires_two_scans_then_half_second_and_five_pairs(self):
        first = self.motion.update(room_scan(1), .01, .1)
        self.assertFalse(first['observable'])
        self.assertFalse(first['stationary'])
        for seq in range(2, 6):
            result = self.motion.update(room_scan(seq), .01, seq*.1)
            self.assertTrue(result['observable'])
            self.assertFalse(result['stationary'])
        result = self.motion.update(room_scan(6), .01, .6)
        self.assertTrue(result['stationary'])
        self.assertGreaterEqual(result['stable_ms'], 500)
        self.assertGreaterEqual(result['stable_publication_ms'], 500)
        self.assertGreaterEqual(result['stable_receive_ms'], 500)
        self.assertGreaterEqual(result['stable_pairs'], 5)
        self.assertGreaterEqual(result['matched_planes'], 2)
        self.assertTrue(result['low_motion'])
        self.assertEqual(result['evidence_type'], 'uncalibrated_lidar_low_motion')
        self.assertEqual(result['stable_translation_m'], 0.)
        self.assertEqual(result['stable_rotation_rad'], 0.)
        self.assertEqual(result['software_thresholds']['min_receive_stable_s'], .5)

    def test_buffered_publications_cannot_fast_forward_receive_stability(self):
        reasons = []
        for index in range(6):
            result = self.motion.update(room_scan(index+1), .01, 10+index*.02)
            reasons.append(result['reason'])
            self.assertFalse(result['stationary'], result)
            self.assertLess(result['stable_receive_ms'], 500)
        self.assertIn('stable_clock_progress_disagreement', reasons)
        self.assertEqual(result['stable_pairs'], 1)

    def test_both_elapsed_clocks_are_required_even_with_acceptable_progress_difference(self):
        for index in range(6):
            result = self.motion.update(room_scan(index+1), .01, 10+index*.08)
        self.assertTrue(result['low_motion'])
        self.assertEqual(result['stable_publication_ms'], 500)
        self.assertEqual(result['stable_receive_ms'], 400)
        self.assertFalse(result['stationary'])

    def test_receive_progress_alone_cannot_satisfy_publication_window(self):
        for index in range(6):
            result = self.motion.update(room_scan(index+1, at_ms=100+index*80), .01, 10+index*.10)
        self.assertTrue(result['low_motion'])
        self.assertEqual(result['stable_publication_ms'], 400)
        self.assertEqual(result['stable_receive_ms'], 500)
        self.assertFalse(result['stationary'])

    def test_low_motion_is_explicit_even_when_not_physically_stationary(self):
        for seq in range(1, 8):
            result = self.motion.update(room_scan(seq, x=(seq-1)*.002), .01, seq*.1)
        self.assertTrue(result['observable'])
        self.assertTrue(result['low_motion'])
        self.assertTrue(result['stationary'])
        self.assertGreater(result['speed_mps'], 0.)
        self.assertGreater(result['stable_translation_m'], .01)
        self.assertEqual(result['evidence_type'], 'uncalibrated_lidar_low_motion')

    def test_forward_and_lateral_translation_are_observed(self):
        self.motion.update(room_scan(1), .01, .1)
        result = self.motion.update(room_scan(2, x=.15, y=.02), .01, .2)
        self.assertTrue(result['observable'])
        self.assertFalse(result['stationary'])
        self.assertFalse(result['low_motion'])
        self.assertAlmostEqual(result['translation_m'], math.hypot(.15, .02), delta=.0005)
        self.assertAlmostEqual(result['speed_mps'], math.hypot(.15, .02)/.1, delta=.005)

    def test_rotation_uses_signed_rigid_motion_and_publication_time(self):
        self.motion.update(room_scan(1), .01, .15)
        result = self.motion.update(room_scan(2, yaw=-.02), .01, .27)
        self.assertTrue(result['observable'])
        self.assertFalse(result['stationary'])
        self.assertAlmostEqual(result['rotation_rad'], -.02, places=5)
        self.assertAlmostEqual(result['yaw_rate_rps'], -.2, places=5)

    def test_parallel_corridor_never_certifies_stationary(self):
        # Translating along infinite parallel walls gives identical scans.
        for seq in range(1, 20):
            result = self.motion.update(parallel_scan(seq), .01, seq*.1)
            self.assertFalse(result['observable'])
            self.assertFalse(result['stationary'])
            self.assertIsNone(result['speed_mps'])

    def test_rotationally_symmetric_curve_is_not_a_plane_identity(self):
        for seq in range(1, 8):
            scan = dict(room_scan(seq), ranges=[3.]*360)
            result = self.motion.update(scan, .01, seq*.1)
            self.assertFalse(result['stationary'])

    def test_repeated_scan_does_not_mature_stability(self):
        scan = room_scan(1)
        self.motion.update(scan, .01, .1)
        for now in [.2, .3, .6, 1.0]:
            result = self.motion.update(scan, .01, now)
            self.assertFalse(result['stationary'])
            self.assertEqual(result['stable_pairs'], 0)
        result = self.motion.update(scan, .30, 1.1)
        self.assertFalse(result['observable'])
        self.assertEqual(result['reason'], 'lidar_stale_or_invalid_age')

    def test_sequence_and_publication_regressions_clear_stability(self):
        for bad in [room_scan(3, at_ms=450), room_scan(5, at_ms=400)]:
            with self.subTest(scan=bad['seq']):
                motion = MODULE.CoastMotion()
                for seq in range(1, 5): motion.update(room_scan(seq), .01, seq*.1)
                result = motion.update(bad, .01, .45)
                self.assertFalse(result['observable'])
                self.assertFalse(result['stationary'])
                self.assertEqual(result['stable_pairs'], 0)

    def test_duplicate_identity_with_changed_content_is_unknown(self):
        scan = room_scan(1)
        self.motion.update(scan, .01, .1)
        scan['ranges'][0] += .01
        result = self.motion.update(scan, .01, .12)
        self.assertFalse(result['observable'])
        self.assertEqual(result['reason'], 'duplicate_sequence_changed_ranges')

    def test_long_publication_gap_restarts_reference_and_stability(self):
        for seq in range(1, 5): self.motion.update(room_scan(seq), .01, seq*.1)
        result = self.motion.update(room_scan(5, at_ms=750), .01, .75)
        self.assertFalse(result['observable'])
        self.assertFalse(result['stationary'])
        self.assertEqual(result['reason'], 'scan_gap_or_rate_invalid')
        result = self.motion.update(room_scan(6, at_ms=850), .01, .85)
        self.assertTrue(result['observable'])
        self.assertFalse(result['stationary'])
        self.assertEqual(result['stable_pairs'], 1)

    def test_sensor_age_or_receive_clock_fault_never_certifies_stationary(self):
        for bad_age in [.30, -1., float('nan'), True]:
            result = MODULE.CoastMotion().update(room_scan(1), bad_age, .1)
            self.assertFalse(result['observable'])
            self.assertFalse(result['stationary'])
        self.motion.update(room_scan(1), .01, .1)
        result = self.motion.update(room_scan(2), .01, .05)
        self.assertFalse(result['observable'])
        self.assertEqual(result['reason'], 'invalid_receive_clock')

    def test_publication_and_receive_clock_disagreement_is_unknown(self):
        self.motion.update(room_scan(1), .01, .1)
        result = self.motion.update(room_scan(2), .01, .4)
        self.assertFalse(result['observable'])
        self.assertEqual(result['reason'], 'publication_receive_clock_disagreement')

    def test_invalid_ranges_unknown_frame_and_sparse_scan_are_unknown(self):
        for change in [{'frame_id': 'world'}, {'ranges': [3.]*359},
                       {'ranges': [None]*360}, {'ranges': [float('nan')]+[3.]*359}]:
            result = MODULE.CoastMotion().update(dict(room_scan(1), **change), .01, .1)
            self.assertFalse(result['observable'])
            self.assertFalse(result['stationary'])

    def test_inconsistent_planes_and_motion_jumps_do_not_mean_static(self):
        for scan in [room_scan(2, front=3.4), room_scan(2, x=1.0)]:
            motion = MODULE.CoastMotion()
            motion.update(room_scan(1), .01, .1)
            result = motion.update(scan, .01, .2)
            self.assertFalse(result['observable'])
            self.assertFalse(result['stationary'])

    def test_motion_after_stable_evidence_clears_stationary(self):
        for seq in range(1, 8): result = self.motion.update(room_scan(seq), .01, seq*.1)
        self.assertTrue(result['stationary'])
        result = self.motion.update(room_scan(8, x=.02), .01, .8)
        self.assertTrue(result['observable'])
        self.assertFalse(result['stationary'])
        self.assertEqual(result['stable_pairs'], 0)

    def test_recorded_run08_never_claims_early_stationary(self):
        path = ROOT/'work/vehicle-upload-20261005/straight-start-08/after/junction-run.json'
        if not path.exists():
            self.skipTest('Private recorded scans are not part of the source checkout')
        data = json.loads(path.read_text())
        checked = 0
        for sample in data['samples']:
            result = self.motion.update(sample['scan'], sample.get('ages', {}).get('lidar', .01), sample['elapsed_s'])
            if sample['scan']['at_ms'] <= 60292:
                checked += 1
                self.assertFalse(result['stationary'], (sample['scan']['seq'], result))
        self.assertGreater(checked, 40)


if __name__ == '__main__':
    unittest.main()
