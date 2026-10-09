"""Unknown-object admission, association and original observation leases."""
import copy
import importlib.util
import json
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


def double_gap_scan(seq=4, published=400, received=.4, *, bearing=90., gaps=(1, 1)):
    scan = compact_scan(seq, published, received, bearing=bearing)
    middle = round(-bearing) % 360
    for direction, count in zip((-1, 1), gaps):
        for offset in range(count):
            scan['ranges'][(middle+direction*(5+offset)) % 360] = None
    return scan


def projected_gap_scan(seq=4, published=400, received=.4, *, gap_count=4, side=1,
                       bearing=90., distance=1., size=9):
    scan = compact_scan(seq, published, received, bearing=bearing, distance=distance, size=size)
    middle = round(-bearing) % 360
    edge = middle-size//2 if side == -1 else middle+size-size//2-1
    for offset in range(1, gap_count+1):
        scan['ranges'][(edge+side*offset) % 360] = None
    return scan


class CompactTargetTests(unittest.TestCase):
    def confirmed(self, *, distance=1., size=9):
        tracker = MODULE.CompactTargetTracker()
        result = None
        for seq, published, received in [(1, 0, 0.), (2, 100, .1), (3, 300, .3)]:
            result = tracker.update(compact_scan(seq, published, received, distance=distance, size=size), received+.01)
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

    def test_confirmed_current_shape_can_cross_one_boundary_null(self):
        tracker, old = self.confirmed()
        scan = compact_scan(4, 400, .4)
        scan['ranges'][275] = None
        result = tracker.update(scan, .41)
        self.assertTrue(result['confirmed'])
        self.assertEqual(result['track_id'], old['track_id'])
        self.assertEqual(result['source_seq'], 4)
        self.assertEqual(result['confirmation_count'], old['confirmation_count'])
        self.assertEqual(result['boundary_unknown_bins'], [275])
        self.assertNotIn(275, result['support_bins'])
        self.assertEqual(result['last_full_isolation_seq'], 3)
        self.assertEqual(result['last_full_isolation_at_ms'], 300)
        self.assertEqual(result['last_full_isolation_received_at'], .3)
        self.assertEqual(result['point_count'], 9)
        self.assertEqual(tracker.reason, 'compact_target_tracked_with_boundary_gap')
        expected = copy.deepcopy(result)
        result['boundary_unknown_bins'].clear()
        result['boundary_far_returns'][0]['range_m'] = 99
        self.assertEqual(tracker.update(scan, .5), expected)
        self.assertIsNone(tracker.update(scan, .71))

    def test_boundary_null_never_confirms_new_track_or_renews_partial_chain(self):
        tracker = MODULE.CompactTargetTracker()
        tracker.update(compact_scan(), .01)
        scan = compact_scan(2, 100, .1)
        scan['ranges'][275] = None
        self.assertIsNone(tracker.update(scan, .11))
        tracker, old = self.confirmed()
        for seq in [4, 5, 6]:
            scan = compact_scan(seq, seq*100, seq/10)
            scan['ranges'][275] = None
            result = tracker.update(scan, seq/10+.01)
            if seq < 6:
                self.assertEqual(result['track_id'], old['track_id'])
                self.assertEqual(result['confirmation_count'], old['confirmation_count'])
                self.assertEqual(result['last_full_isolation_at_ms'], 300)
                self.assertEqual(result['last_full_isolation_received_at'], .3)
            else:
                self.assertIsNone(result)
                self.assertEqual(tracker.reason, 'compact_target_boundary_gap_expired')

    def test_partial_duplicate_expires_with_full_isolation_not_current_scan(self):
        tracker, _ = self.confirmed()
        scan = compact_scan(4, 500, .5)
        scan['ranges'][275:278] = [None]*3
        self.assertIsNotNone(tracker.update(scan, .51))
        self.assertIsNotNone(tracker.update(scan, .59))
        self.assertIsNone(tracker.update(scan, .61))  # Current scan is only 110 ms old.
        self.assertEqual(tracker.reason, 'compact_target_boundary_gap_expired')

    def test_partial_lease_requires_both_original_clocks(self):
        for published, received in [(600, .5), (500, .6)]:
            with self.subTest(published=published, received=received):
                tracker, _ = self.confirmed()
                partial = compact_scan(4, 400, .4)
                partial['ranges'][275] = None
                self.assertIsNotNone(tracker.update(partial, .41))
                partial.update(seq=5, at_ms=published, received_at=received)
                self.assertIsNone(tracker.update(partial, received+.01))
                self.assertEqual(tracker.reason, 'compact_target_boundary_gap_expired')

    def test_full_boundaries_restore_isolation_lease(self):
        tracker, original = self.confirmed()
        partial = compact_scan(4, 500, .5)
        partial['ranges'][275:278] = [None]*3
        self.assertIsNotNone(tracker.update(partial, .51))
        full = tracker.update(compact_scan(5, 550, .55), .56)
        self.assertEqual(full['track_id'], original['track_id'])
        self.assertEqual(full['last_full_isolation_seq'], 5)
        partial.update(seq=6, at_ms=650, received_at=.65)
        result = tracker.update(partial, .66)
        self.assertEqual(result['track_id'], original['track_id'])
        self.assertEqual(result['last_full_isolation_at_ms'], 550)
        self.assertEqual(result['last_full_isolation_received_at'], .55)

    def test_tracking_boundary_gap_requires_bounded_real_background_and_same_shape(self):
        for change in ('excessive_span', 'both_edges_one_plus_two', 'near_background', 'jump', 'ambiguous'):
            with self.subTest(change=change):
                tracker, _ = self.confirmed()
                scan = compact_scan(4, 400, .4, bearing=60. if change == 'jump' else 90.)
                boundary = 305 if change == 'jump' else 275
                scan['ranges'][boundary] = None
                if change == 'excessive_span': scan['ranges'][boundary:boundary+11] = [None]*11
                if change == 'both_edges_one_plus_two': scan['ranges'][264:266] = [None]*2
                if change == 'near_background': scan['ranges'][boundary+1] = .9
                if change == 'ambiguous':
                    for index in range(246, 255): scan['ranges'][index] = 1.5
                self.assertIsNone(tracker.update(scan, .41))

    def test_double_single_null_keeps_only_current_confirmed_support_and_original_lease(self):
        tracker, old = self.confirmed()
        result = tracker.update(double_gap_scan(), .41)
        self.assertTrue(result['confirmed'])
        self.assertEqual(result['track_id'], old['track_id'])
        self.assertEqual(result['confirmation_count'], old['confirmation_count'])
        self.assertEqual(result['source_seq'], 4)
        self.assertEqual(result['support_bins'], list(range(266, 275)))
        self.assertEqual(result['point_count'], 9)
        self.assertEqual(result['boundary_unknown_bins'], [265, 275])
        self.assertEqual(result['boundary_far_returns'],
                         [{'index': 264, 'range_m': 3.}, {'index': 276, 'range_m': 3.}])
        self.assertTrue(result['tracking_only_boundary_gap'])
        self.assertEqual(result['last_full_isolation_seq'], 3)
        self.assertEqual(result['last_full_isolation_at_ms'], 300)
        self.assertEqual(result['last_full_isolation_received_at'], .3)
        self.assertFalse(result['physical_identity_verified'])
        self.assertEqual(tracker.reason, 'compact_target_tracked_with_boundary_gap')

    def test_double_single_null_never_initializes_or_confirms_a_target(self):
        for unconfirmed_seed in (False, True):
            with self.subTest(unconfirmed_seed=unconfirmed_seed):
                tracker = MODULE.CompactTargetTracker()
                if unconfirmed_seed:
                    self.assertFalse(tracker.update(compact_scan(3, 300, .3), .31)['confirmed'])
                self.assertIsNone(tracker.update(double_gap_scan(), .41))
                self.assertEqual(tracker.reason, 'compact_target_missing')
                recovered = tracker.update(compact_scan(5, 500, .5), .51)
                self.assertFalse(recovered['confirmed'])
                self.assertEqual(recovered['confirmation_count'], 1)

    def test_double_boundary_gap_rejects_more_than_one_null_on_either_side(self):
        for gaps in ((1, 2), (2, 1), (2, 2), (1, 3), (3, 1)):
            with self.subTest(gaps=gaps):
                tracker, _ = self.confirmed()
                self.assertIsNone(tracker.update(double_gap_scan(gaps=gaps), .41))
                self.assertEqual(tracker.reason, 'compact_target_missing')

    def test_double_boundary_gap_requires_real_far_return_beyond_each_null(self):
        for index in (264, 276):
            for value in (None, .9, 1.18):
                with self.subTest(index=index, value=value):
                    tracker, _ = self.confirmed()
                    scan = double_gap_scan()
                    scan['ranges'][index] = value
                    self.assertIsNone(tracker.update(scan, .41))
                    self.assertEqual(tracker.reason, 'compact_target_missing')

    def test_double_boundary_gap_rejects_identity_jump_and_current_ambiguity(self):
        for change in ('jump', 'ambiguous'):
            with self.subTest(change=change):
                tracker, _ = self.confirmed()
                scan = double_gap_scan(bearing=60. if change == 'jump' else 90.)
                if change == 'ambiguous':
                    for index in range(246, 255): scan['ranges'][index] = 1.5
                self.assertIsNone(tracker.update(scan, .41))
                self.assertEqual(tracker.reason, 'compact_target_boundary_gap_unassociated'
                                 if change == 'jump' else 'compact_target_ambiguous')

    def test_double_boundary_chain_cannot_renew_either_original_clock(self):
        for published, received in ((600, .5), (500, .6)):
            with self.subTest(published=published, received=received):
                tracker, old = self.confirmed()
                partial = tracker.update(double_gap_scan(), .41)
                self.assertEqual(partial['confirmation_count'], old['confirmation_count'])
                self.assertIsNone(tracker.update(double_gap_scan(5, published, received), received+.01))
                self.assertEqual(tracker.reason, 'compact_target_boundary_gap_expired')

    def test_double_boundary_repeat_cannot_extend_full_isolation_lease(self):
        tracker, _ = self.confirmed()
        scan = double_gap_scan(4, 500, .5)
        result = tracker.update(scan, .51)
        self.assertIsNotNone(result)
        self.assertEqual(tracker.update(scan, .59), result)
        self.assertIsNone(tracker.update(scan, .61))  # Partial scan is only 110 ms old.
        self.assertEqual(tracker.reason, 'compact_target_boundary_gap_expired')

    def test_projected_single_gap_keeps_current_support_without_renewing_full_lease(self):
        for side in (-1, 1):
            with self.subTest(side=side):
                tracker, old = self.confirmed()
                result = tracker.update(projected_gap_scan(side=side), .41)
                self.assertIsNotNone(result)
                self.assertTrue(result['confirmed'])
                self.assertEqual(result['track_id'], old['track_id'])
                self.assertEqual(result['confirmation_count'], old['confirmation_count'])
                self.assertEqual(result['support_bins'], old['support_bins'])
                self.assertEqual(result['point_count'], old['point_count'])
                self.assertEqual(result['diameter_m'], old['diameter_m'])
                self.assertEqual(result['last_full_isolation_seq'], 3)
                self.assertEqual(result['last_full_isolation_at_ms'], 300)
                self.assertEqual(result['last_full_isolation_received_at'], .3)
                self.assertEqual(len(result['boundary_unknown_bins']), 4)
                self.assertAlmostEqual(result['boundary_gap_projected_span_m'], math.radians(5))
                self.assertTrue(result['tracking_only_boundary_gap'])
                self.assertFalse(result['physical_identity_verified'])
                self.assertEqual(tracker.reason, 'compact_target_tracked_with_boundary_gap')

    def test_projected_gap_never_initializes_or_matures_unconfirmed_target(self):
        for unconfirmed_seed in (False, True):
            with self.subTest(unconfirmed_seed=unconfirmed_seed):
                tracker = MODULE.CompactTargetTracker()
                if unconfirmed_seed:
                    tracker.update(compact_scan(3, 300, .3), .31)
                self.assertIsNone(tracker.update(projected_gap_scan(), .41))
                self.assertEqual(tracker.reason, 'compact_target_missing')

    def test_projected_gap_rejects_span_beyond_point_gap_or_visible_diameter_or_total_limit(self):
        cases = [(2., 9, 5, 'point_gap'), (1., 5, 4, 'visible_diameter'), (1., 17, 4, 'total')]
        for distance, size, gap_count, exceeded in cases:
            with self.subTest(exceeded=exceeded):
                tracker, old = self.confirmed(distance=distance, size=size)
                span = distance*math.radians(gap_count+1)
                if exceeded == 'point_gap':
                    self.assertGreater(span, MODULE.MAX_POINT_GAP_M)
                elif exceeded == 'visible_diameter':
                    self.assertGreater(span, old['diameter_m'])
                    self.assertLess(span, MODULE.MAX_POINT_GAP_M)
                    self.assertLess(span+old['diameter_m'], MODULE.MAX_DIAMETER_M)
                else:
                    self.assertGreater(span+old['diameter_m'], MODULE.MAX_DIAMETER_M)
                    self.assertLess(span, min(MODULE.MAX_POINT_GAP_M, old['diameter_m']))
                self.assertIsNone(tracker.update(projected_gap_scan(distance=distance, size=size,
                                                                    gap_count=gap_count), .41))
                self.assertEqual(tracker.reason, 'compact_target_missing')

    def test_projected_gap_requires_immediate_opposite_and_real_far_outer_return(self):
        for change in ('opposite_null', 'opposite_long_gap', 'outer_near', 'outer_beyond_span'):
            with self.subTest(change=change):
                tracker, _ = self.confirmed()
                scan = projected_gap_scan()
                if change == 'opposite_null': scan['ranges'][265] = None
                if change == 'opposite_long_gap': scan['ranges'][262:266] = [None]*4
                if change == 'outer_near': scan['ranges'][279] = 1.18
                if change == 'outer_beyond_span': scan['ranges'][279:] = [None]*(360-279)
                self.assertIsNone(tracker.update(scan, .41))
                self.assertEqual(tracker.reason, 'compact_target_missing')

    def test_projected_gap_rejects_unassociated_or_ambiguous_current_shape(self):
        for change in ('jump', 'ambiguous'):
            with self.subTest(change=change):
                tracker, _ = self.confirmed()
                scan = projected_gap_scan(bearing=60. if change == 'jump' else 90.)
                if change == 'ambiguous':
                    for index in range(246, 255): scan['ranges'][index] = 1.5
                self.assertIsNone(tracker.update(scan, .41))
                self.assertEqual(tracker.reason, 'compact_target_boundary_gap_unassociated'
                                 if change == 'jump' else 'compact_target_ambiguous')

    def test_projected_gap_cannot_renew_either_original_clock_or_duplicate_lease(self):
        for published, received in ((600, .5), (500, .6)):
            with self.subTest(published=published, received=received):
                tracker, _ = self.confirmed()
                self.assertIsNotNone(tracker.update(projected_gap_scan(), .41))
                self.assertIsNone(tracker.update(projected_gap_scan(5, published, received), received+.01))
                self.assertEqual(tracker.reason, 'compact_target_boundary_gap_expired')
        tracker, _ = self.confirmed()
        scan = projected_gap_scan(4, 500, .5)
        result = tracker.update(scan, .51)
        self.assertIsNotNone(result)
        self.assertEqual(tracker.update(scan, .59), result)
        self.assertIsNone(tracker.update(scan, .61))
        self.assertEqual(tracker.reason, 'compact_target_boundary_gap_expired')

    def test_trial31_recorded_full_loss_and_returned_boundaries_keep_seeded_identity(self):
        fixture = json.loads((Path(__file__).parent/'fixtures'/'left-cone-31-projected-boundary-gap.json').read_text())
        tracker = MODULE.CompactTargetTracker()
        seed = copy.deepcopy(fixture['validated_state_target599'])
        full, loss, following = fixture['raw_frames']
        # Unlike trial30, raw599 WAS saved. Its geometry must match the recorded
        # target; only earlier confirmation bookkeeping is injected here.
        full_candidates = MODULE._candidates(full['ranges'])
        self.assertEqual(len(full_candidates), 1)
        for key in ('point_left_m', 'diameter_m', 'support_bins', 'boundary_far_returns'):
            self.assertEqual(full_candidates[0][key], seed[key])
        tracker._target = seed
        tracker._last_seq, tracker._last_at = seed['source_seq'], seed['source_at_ms']
        tracker._last_received = tracker._last_now = seed['source_received_at']
        tracker._signature = tuple(full['ranges'])
        tracker._first_at, tracker._first_received = seed['source_at_ms']-300, seed['source_received_at']-.3
        tracker._next_id = seed['track_id']+1
        self.assertEqual(tracker.update(full, full['received_at']+.001), seed)
        result = tracker.update(loss, fixture['loss_decision_at'])
        self.assertIsNotNone(result)
        self.assertEqual(MODULE._candidates(loss['ranges']), [])
        self.assertEqual(result['track_id'], seed['track_id'])
        self.assertTrue(result['confirmed'])
        self.assertTrue(result['tracking_only_boundary_gap'])
        self.assertEqual(result['confirmation_count'], seed['confirmation_count'])
        self.assertEqual(result['last_full_isolation_seq'], 599)
        self.assertEqual(result['last_full_isolation_at_ms'], seed['source_at_ms'])
        self.assertEqual(result['last_full_isolation_received_at'], seed['source_received_at'])
        self.assertEqual(result['support_bins'], list(range(280, 291)))
        self.assertEqual(result['boundary_unknown_bins'], list(range(291, 300)))
        self.assertAlmostEqual(result['boundary_gap_projected_span_m'], .901*math.radians(10))
        self.assertEqual(result['boundary_far_returns'],
                         [{'index': 279, 'range_m': loss['ranges'][279]}, {'index': 300, 'range_m': 4.714}])
        # Raw601 points/time are recorded. This receive interval is mocked, not
        # evidence of resumed actuation or a physically identical whole cone.
        scan = {**following, 'received_at': loss['received_at']+(following['at_ms']-loss['at_ms'])/1000}
        restored = tracker.update(scan, scan['received_at']+.001)
        self.assertEqual(restored['track_id'], seed['track_id'])
        self.assertTrue(restored['confirmed'])
        self.assertFalse(restored['tracking_only_boundary_gap'])
        self.assertEqual(restored['confirmation_count'], seed['confirmation_count']+1)
        self.assertEqual(restored['last_full_isolation_seq'], 601)
        self.assertFalse(restored['physical_identity_verified'])

    def test_trial30_exact_loss_frame_and_next_raw_frame_maintain_seeded_identity(self):
        fixture = json.loads((Path(__file__).parent/'fixtures'/'left-cone-30-double-boundary-gaps.json').read_text())
        tracker = MODULE.CompactTargetTracker()
        seed = copy.deepcopy(fixture['validated_state_target939'])
        # Raw scan939 was NOT captured. This is its recorded target summary;
        # prior maturity is injected bookkeeping, not a fabricated raw frame.
        tracker._target = seed
        tracker._last_seq = seed['source_seq']
        tracker._last_at = seed['source_at_ms']
        tracker._last_received = tracker._last_now = seed['source_received_at']
        tracker._first_at = seed['source_at_ms']-300
        tracker._first_received = seed['source_received_at']-.3
        tracker._next_id = seed['track_id']+1
        loss_frame = fixture['raw_frames'][0]
        for raw in fixture['raw_frames']:
            scan = copy.deepcopy(raw)
            if raw['seq'] == 940:
                now = fixture['loss_decision_at']  # Both receive and decision clocks are recorded.
            else:
                # Only this receive interval is mocked; raw941 points/source time are recorded.
                scan['received_at'] = loss_frame['received_at']+(raw['at_ms']-loss_frame['at_ms'])/1000
                now = scan['received_at']+.001
            result = tracker.update(scan, now)
            self.assertIsNotNone(result)
            self.assertEqual(MODULE._candidates(raw['ranges']), [])
            self.assertEqual(result['track_id'], seed['track_id'])
            self.assertEqual(result['confirmation_count'], seed['confirmation_count'])
            self.assertTrue(result['confirmed'])
            self.assertTrue(result['tracking_only_boundary_gap'])
            self.assertFalse(result['physical_identity_verified'])
            self.assertEqual(result['source_seq'], raw['seq'])
            self.assertEqual(result['last_full_isolation_seq'], 939)
            self.assertEqual(result['last_full_isolation_at_ms'], seed['source_at_ms'])
            self.assertEqual(result['last_full_isolation_received_at'], seed['source_received_at'])
            self.assertEqual(result['support_bins'], list(range(276, 287)) if raw['seq'] == 940
                             else list(range(273, 286)))
            self.assertEqual(result['boundary_unknown_bins'], [275, 287] if raw['seq'] == 940 else [286, 287])
        self.assertIsNone(tracker.update(scan, seed['source_received_at']+.301))
        self.assertEqual(tracker.reason, 'compact_target_boundary_gap_expired')

    def test_trial27_current_components_keep_seeded_confirmed_identity(self):
        fixture = json.loads((Path(__file__).parent/'fixtures'/'left-cone-27-boundary-gaps.json').read_text())
        tracker = MODULE.CompactTargetTracker()
        seed = copy.deepcopy(fixture['validated_state_target791'])
        # Raw scan791 was NOT captured. Seed its recorded confirmed state;
        # mocked prior maturity/receipt intervals are unit inputs, not actuation.
        seed.update(last_full_isolation_at_ms=seed['source_at_ms'],
                    last_full_isolation_received_at=seed['source_received_at'])
        tracker._target = seed
        tracker._last_seq = seed['source_seq']
        tracker._last_at = seed['source_at_ms']
        tracker._last_received = tracker._last_now = seed['source_received_at']
        tracker._first_at = seed['source_at_ms']-300
        tracker._first_received = seed['source_received_at']-.3
        tracker._next_id = seed['track_id']+1
        for raw in fixture['raw_frames']:
            receive = seed['source_received_at']+(raw['at_ms']-seed['source_at_ms'])/1000
            result = tracker.update({**raw, 'received_at': receive}, receive+.001)
            self.assertIsNotNone(result)
            self.assertEqual(MODULE._candidates(raw['ranges']), [])
            self.assertEqual(result['track_id'], seed['track_id'])
            self.assertEqual(result['confirmation_count'], seed['confirmation_count'])
            self.assertEqual(result['source_seq'], raw['seq'])
            self.assertEqual(result['last_full_isolation_seq'], 791)
            self.assertEqual(result['boundary_unknown_bins'], [297, 298, 299] if raw['seq'] == 792 else [283])
            self.assertEqual(result['point_count'], 12 if raw['seq'] == 792 else 13)

    def test_real_trial18_boundary_null_keeps_current_identity_then_strictly_recovers(self):
        fixture = json.loads((Path(__file__).parent/'fixtures'/'left-cone-18-boundary-null.json').read_text())
        tracker = MODULE.CompactTargetTracker()
        identity = None
        first_at = fixture['frames'][0]['at_ms']
        for raw in fixture['frames']:
            now = (raw['at_ms']-first_at)/1000  # Mock receipt clock, not recorded actuation.
            scan = {**raw, 'received_at': now}
            result = tracker.update(scan, now+.01)
            self.assertIsNotNone(result)
            if identity is None: identity = result['track_id']
            self.assertEqual(result['track_id'], identity)
            self.assertEqual(result['source_seq'], raw['seq'])
            if raw['seq'] == 618:
                self.assertEqual(MODULE._candidates(raw['ranges']), [])
                self.assertTrue(result['confirmed'])
                self.assertEqual(result['point_count'], 13)
                self.assertEqual(result['boundary_unknown_bins'], [272])
                self.assertEqual(result['last_full_isolation_seq'], 617)
            if raw['seq'] == 620:
                self.assertTrue(result['confirmed'])
                self.assertFalse(result['tracking_only_boundary_gap'])
                self.assertEqual(result['last_full_isolation_seq'], 620)

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
