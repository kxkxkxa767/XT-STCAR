"""Unknown-object admission, association and original observation leases."""
import copy
import importlib.util
import math
from pathlib import Path
import unittest

SPEC = importlib.util.spec_from_file_location('compact_target', Path(__file__).parents[1]/'compact_target.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def compact_scan(seq=1, published=0, received=0., bearing=90., distance=1., size=9):
    ranges = [3.]*360
    middle = round(-bearing) % 360
    for offset in range(-(size//2), size-size//2):
        ranges[(middle+offset) % 360] = distance
    return {'frame_id': MODULE.FRAME, 'seq': seq, 'at_ms': published,
            'received_at': received, 'ranges': ranges}


class CompactTargetTests(unittest.TestCase):
    def confirmed(self):
        tracker = MODULE.CompactTargetTracker()
        result = None
        for seq, published, received in [(1, 0, 0.), (2, 100, .1), (3, 300, .3)]:
            result = tracker.update(compact_scan(seq, published, received), received+.01)
        self.assertTrue(result['confirmed'])
        return tracker, result

    def test_current_side_object_is_unknown_until_confirmed(self):
        result = MODULE.CompactTargetTracker().update(compact_scan(), .01)
        self.assertFalse(result['confirmed'])
        self.assertEqual(result['semantic_class'], 'unknown')
        self.assertEqual(result['kind'], 'lidar_compact_object')
        self.assertFalse(result['physical_identity_verified'])
        self.assertTrue(result['candidate_only'])
        self.assertEqual(result['point_count'], 9)
        self.assertAlmostEqual(result['bearing_left_rad'], math.pi/2)
        self.assertAlmostEqual(result['point_left_m'][0], 0.)
        self.assertGreater(result['point_left_m'][1], 0.)
        self.assertEqual((result['source_seq'], result['source_at_ms']), (1, 0))

    def test_three_distinct_frames_and_both_clocks_confirm(self):
        tracker = MODULE.CompactTargetTracker()
        first = tracker.update(compact_scan(), .01)
        second = tracker.update(compact_scan(2, 100, .1), .11)
        third = tracker.update(compact_scan(3, 200, .2), .21)
        fourth = tracker.update(compact_scan(4, 300, .3), .31)
        self.assertFalse(second['confirmed'])
        self.assertFalse(third['confirmed'])
        self.assertTrue(fourth['confirmed'])
        self.assertEqual(first['track_id'], fourth['track_id'])
        self.assertEqual(fourth['confirmation_count'], 4)

    def test_receive_clock_cannot_be_replaced_by_fast_source_time(self):
        tracker = MODULE.CompactTargetTracker()
        for seq, source, receive in [(1, 0, 0.), (2, 150, .1), (3, 300, .2)]:
            result = tracker.update(compact_scan(seq, source, receive), receive+.01)
        self.assertFalse(result['confirmed'])

    def test_repeated_read_does_not_confirm_or_renew(self):
        tracker = MODULE.CompactTargetTracker()
        scan = compact_scan()
        for now in [.01, .10, .20, .29]:
            result = tracker.update(scan, now)
            self.assertEqual(result['confirmation_count'], 1)
            self.assertFalse(result['confirmed'])
        self.assertIsNone(tracker.update(scan, .30))
        self.assertEqual(tracker.reason, 'compact_target_stale')

    def test_confirmed_repeat_retains_original_lease_and_source(self):
        tracker, target = self.confirmed()
        result = tracker.update(compact_scan(3, 300, .3), .5)
        self.assertEqual(result, target)
        self.assertIsNone(tracker.update(compact_scan(3, 300, .3), .61))

    def test_same_source_cannot_change_content_or_receive_time(self):
        for mutation in ['ranges', 'at_ms', 'received_at']:
            with self.subTest(mutation=mutation):
                tracker, _ = self.confirmed()
                scan = compact_scan(3, 300, .3)
                if mutation == 'ranges': scan['ranges'][270] = .99
                else: scan[mutation] += .01 if mutation == 'received_at' else 1
                self.assertIsNone(tracker.update(scan, .4))
                self.assertIsNone(tracker.update(compact_scan(3, 300, .3), .41))

    def test_older_sequence_or_clock_never_reacquires(self):
        for scan in [compact_scan(2, 200, .35), compact_scan(4, 200, .35), compact_scan(4, 400, .2)]:
            with self.subTest(scan=scan['seq']):
                tracker, _ = self.confirmed()
                self.assertIsNone(tracker.update(scan, .4))

    def test_observation_gap_restarts_confirmation_with_new_identity(self):
        tracker, target = self.confirmed()
        result = tracker.update(compact_scan(4, 800, .8), .81)
        self.assertFalse(result['confirmed'])
        self.assertEqual(result['confirmation_count'], 1)
        self.assertNotEqual(result['track_id'], target['track_id'])

    def test_inconsistent_clock_progress_rejects_current_frame(self):
        tracker = MODULE.CompactTargetTracker()
        tracker.update(compact_scan(), .01)
        self.assertIsNone(tracker.update(compact_scan(2, 250, .05), .06))
        self.assertEqual(tracker.reason, 'compact_target_clock_gap')

    def test_absent_or_ambiguous_current_object_discards_old_target(self):
        for ambiguous in [False, True]:
            with self.subTest(ambiguous=ambiguous):
                tracker, target = self.confirmed()
                scan = compact_scan(4, 400, .4)
                if ambiguous:
                    for i in range(246, 255): scan['ranges'][i] = 1.5
                else:
                    scan['ranges'] = [3.]*360
                self.assertIsNone(tracker.update(scan, .41))
                recovered = tracker.update(compact_scan(5, 500, .5), .51)
                self.assertFalse(recovered['confirmed'])
                self.assertNotEqual(recovered['track_id'], target['track_id'])

    def test_null_neighbor_is_unknown_not_isolation(self):
        for index in [265, 275]:
            scan = compact_scan()
            scan['ranges'][index] = None
            self.assertIsNone(MODULE.CompactTargetTracker().update(scan, .01))

    def test_nearer_neighbor_is_not_far_background(self):
        scan = compact_scan()
        scan['ranges'][265] = .5
        self.assertIsNone(MODULE.CompactTargetTracker().update(scan, .01))

    def test_fewer_than_five_actual_returns_are_rejected(self):
        for size in [1, 2, 3, 4]:
            self.assertIsNone(MODULE.CompactTargetTracker().update(compact_scan(size=size), .01))

    def test_small_large_and_long_whole_objects_are_rejected(self):
        for distance, size in [(1., 3), (1., 25), (1., 80), (3., 129)]:
            self.assertIsNone(MODULE.CompactTargetTracker().update(compact_scan(distance=distance, size=size), .01))

    def test_wall_end_is_never_cropped_by_search_sector(self):
        scan = compact_scan()
        for i in range(220, 310): scan['ranges'][i] = 1.
        self.assertIsNone(MODULE.CompactTargetTracker().update(scan, .01))

    def test_range_filter_never_crops_a_long_wall_into_object(self):
        scan = compact_scan()
        for i in range(250, 291): scan['ranges'][i] = .3+abs(i-270)*.015
        self.assertIsNone(MODULE.CompactTargetTracker().update(scan, .01))

    def test_enclosure_and_wrap_components_are_not_sliced(self):
        scan = compact_scan()
        scan['ranges'] = [1.]*360
        self.assertIsNone(MODULE.CompactTargetTracker().update(scan, .01))
        points = [None]*360
        for i in [358, 359, 0, 1, 2]:
            points[i] = (math.cos(math.radians(i)), math.sin(math.radians(i)))
        self.assertEqual(MODULE._clusters(points), [[358, 359, 0, 1, 2]])

    def test_wrong_side_and_outside_sector_are_not_selected(self):
        for bearing in [-90, 0, 30, 150, 180]:
            self.assertIsNone(MODULE.CompactTargetTracker().update(compact_scan(bearing=bearing), .01))
        for bearing in [50, 80, 110, 130]:
            self.assertIsNotNone(MODULE.CompactTargetTracker().update(compact_scan(bearing=bearing), .01))

    def test_outside_trial_range_is_rejected(self):
        for distance in [.30, 3.1]:
            self.assertIsNone(MODULE.CompactTargetTracker().update(compact_scan(distance=distance), .01))

    def test_association_jump_is_a_new_unconfirmed_track(self):
        tracker, old = self.confirmed()
        result = tracker.update(compact_scan(4, 400, .4, bearing=60.), .41)
        self.assertFalse(result['confirmed'])
        self.assertNotEqual(result['track_id'], old['track_id'])

    def test_invalid_ranges_and_envelopes_return_none(self):
        for change in [{'ranges': [1.]*359}, {'ranges': 'bad'}, {'frame_id': 'unmapped'},
                       {'seq': True}, {'seq': -1}, {'at_ms': 1.5}, {'received_at': None},
                       {'received_at': .2}, {'received_at': float('nan')}]:
            with self.subTest(change=str(change)[:25]):
                scan = compact_scan()
                scan.update(change)
                self.assertIsNone(MODULE.CompactTargetTracker().update(scan, .01))
        for value in [True, float('nan'), float('inf'), .019, 12.1, -1., '1']:
            scan = compact_scan()
            scan['ranges'][0] = value
            self.assertIsNone(MODULE.CompactTargetTracker().update(scan, .01))

    def test_receive_clock_regression_and_missing_stamp_return_none(self):
        tracker, _ = self.confirmed()
        self.assertIsNone(tracker.update(compact_scan(4, 400, .4), .2))
        scan = compact_scan()
        del scan['received_at']
        self.assertIsNone(MODULE.CompactTargetTracker().update(scan, .01))

    def test_returned_diagnostics_cannot_mutate_tracker_evidence(self):
        tracker, expected = self.confirmed()
        copied = copy.deepcopy(expected)
        expected['point_left_m'][1] = 999
        expected['support_bins'].clear()
        repeated = tracker.update(compact_scan(3, 300, .3), .4)
        self.assertEqual(repeated, copied)


if __name__ == '__main__':
    unittest.main()
