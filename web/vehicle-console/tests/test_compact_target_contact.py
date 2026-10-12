"""Explicit role-bound near maintenance, never near acquisition or cone semantics.

Round 55 (Claude): near maintenance uses the tolerant rules (see
compact_target._close_candidates); cone-like contact supports are tested in
test_route_contact_control.
"""
import copy
import math
import unittest

from test_compact_target import MODULE, compact_scan
from test_compact_target_roles import add_object


class CompactTargetContactTests(unittest.TestCase):
    @staticmethod
    def tracker():
        return MODULE.CompactTargetTracker(
            acquisition_bearing_rad=(-math.pi, math.pi),
            maintenance_bearing_rad=(-math.pi, math.pi),
            select_associated=True, unconfirmed_boundary_gap=True, tolerant=True)

    def bound(self, *, bearing=90., enabled=True):
        tracker = self.tracker()
        for seq, at in ((1, 0.), (2, .1), (3, .3)):
            target = tracker.update(compact_scan(seq, round(at*1000), at,
                                                bearing=bearing, distance=.4, size=21), at+.01)
        self.assertTrue(target['confirmed'])
        if enabled:
            self.assertTrue(tracker.enable_close_maintenance(target['track_id']))
        return tracker, target

    @staticmethod
    def near(seq=4, at=.4, *, bearing=90., distance=.25, size=21):
        return compact_scan(seq, round(at*1000), at, bearing=bearing, distance=distance, size=size)

    def test_default_acquisition_and_unbound_confirmed_tracks_keep_35cm_minimum(self):
        scan = self.near()
        self.assertEqual(MODULE._candidates(scan['ranges']), [])
        self.assertIsNone(self.tracker().update(scan, .41))
        tracker, old = self.bound(enabled=False)
        self.assertIsNone(tracker.update(scan, .41))
        self.assertEqual(tracker.held['track_id'], old['track_id'])
        self.assertFalse(tracker.held['close_maintenance_observation'])

    def test_enabling_requires_the_current_confirmed_full_original_identity(self):
        tracker = self.tracker()
        self.assertFalse(tracker.enable_close_maintenance(1))
        first = tracker.update(compact_scan(distance=.4, size=21), .01)
        self.assertFalse(first['confirmed'])
        self.assertFalse(tracker.enable_close_maintenance(first['track_id']))
        tracker, target = self.bound(enabled=False)
        for wrong in (True, None, '1', target['track_id']+1):
            self.assertFalse(tracker.enable_close_maintenance(wrong))
        self.assertTrue(tracker.enable_close_maintenance(target['track_id']))

    def test_bound_role_maintains_current_support_through_contact_without_relabeling(self):
        # Front, right, left and rear are all original clockwise lidar bins;
        # no sector exemption or assumed camera direction is involved.
        for bearing, contact_distance in ((0., .20), (-90., .14), (90., .14), (180., .18)):
            with self.subTest(bearing=bearing):
                tracker, old = self.bound(bearing=bearing)
                for seq, distance, size in ((4, .32, 21), (5, .25, 21), (6, contact_distance, 27)):
                    at = seq/10
                    scan = self.near(seq, at, bearing=bearing, distance=distance, size=size)
                    current = tracker.update(scan, at+.01, acquire=False)
                    self.assertIsNotNone(current)
                    self.assertEqual(current['track_id'], old['track_id'])
                    self.assertTrue(current['confirmed'])
                    self.assertTrue(current['close_maintenance_observation'])
                    self.assertFalse(current['tracking_only_boundary_gap'])
                    self.assertEqual(current['source_seq'], seq)
                    self.assertEqual(current['source_at_ms'], round(at*1000))
                    self.assertEqual(current['source_received_at'], at)
                    self.assertEqual(current['last_full_isolation_seq'], seq)
                    self.assertEqual(current['boundary_unknown_bins'], [])
                    self.assertEqual(current['semantic_class'], 'unknown')
                    self.assertFalse(current['physical_identity_verified'])
                    self.assertEqual(len(current['support_bins']), size)
                    self.assertTrue(all(scan['ranges'][i] == distance for i in current['support_bins']))

    def test_near_bounded_edge_and_internal_gaps_keep_the_bound_identity(self):
        # Round 55 (Claude): the real cone in trial54 scan328 (.33-.38 m) had
        # one internal null and .42-.49 m mixed edge returns; the earlier
        # strict near rule dropped it. Near maintenance now uses the tolerant
        # rules: bridged internal nulls, size-bounded unknown edges with a
        # real farther return beyond. Null bins are never support.
        for fault in ('left_edge_null', 'right_edge_null', 'internal_null', 'mixed_edge', 'mixed_internal'):
            with self.subTest(fault=fault):
                tracker, old = self.bound()
                scan = self.near()
                index = {'left_edge_null': 259, 'right_edge_null': 281,
                         'internal_null': 270, 'mixed_edge': 259, 'mixed_internal': 270}[fault]
                scan['ranges'][index] = .40 if fault.startswith('mixed') else None
                current = tracker.update(scan, .41, acquire=False)
                self.assertIsNotNone(current)
                self.assertEqual(current['track_id'], old['track_id'])
                self.assertTrue(current['close_maintenance_observation'])
                self.assertFalse(current['tracking_only_boundary_gap'])
                self.assertTrue(all(scan['ranges'][i] is not None for i in current['support_bins']))
                if fault.startswith('mixed'):
                    # A real return within the 18 cm link is support, not unknown.
                    self.assertIn(index, current['support_bins'])
                if fault in ('left_edge_null', 'right_edge_null', 'internal_null'):
                    self.assertNotIn(index, current['support_bins'])

    def test_near_edge_gap_beyond_the_maintenance_bound_is_not_maintained(self):
        tracker, old = self.bound()
        scan = self.near()
        for index in range(251, 260):  # nine unknown bins at one edge
            scan['ranges'][index] = None
        self.assertIsNone(tracker.update(scan, .41, acquire=False))
        self.assertEqual(tracker.held['source_seq'], old['source_seq'])

    def test_recorded52_near_contour_keeps_its_full_support_and_real_edge_depths(self):
        # Trial52 scan440: these are the exact support ranges at bins9..33
        # and their measured immediate backgrounds8/34. Other bins below are
        # synthetic distant background; this is a component regression, not
        # a replay of the complete physical scene or proof of cone identity.
        ranges = [3.]*360
        ranges[8:35] = [
            2.421, .378, .33099999999999996, .316, .315, .298, .283,
            .283, .283, .298, .283, .299, .299, .299, .316, .315,
            .315, .315, .32999999999999996, .32999999999999996,
            .32999999999999996, .32999999999999996, .346, .346,
            .346, .363, 3.088]
        self.assertEqual(MODULE._candidates(ranges, bearing_range_rad=(-math.pi, math.pi)), [])
        candidates = MODULE._close_candidates(ranges)
        self.assertEqual(len(candidates), 1)
        target = candidates[0]
        self.assertEqual(target['support_bins'], list(range(9, 34)))
        self.assertEqual(target['boundary_unknown_bins'], [])
        # Genuine measured curvature/depth changes (largest radial step 47 mm)
        # stay inside the support.
        self.assertGreater(max(ranges[i] for i in target['support_bins']), .35)
        self.assertLess(min(ranges[i] for i in target['support_bins']), .30)

    def test_two_near_associations_are_ambiguous_instead_of_chosen(self):
        tracker, old = self.bound()
        scan = self.near(bearing=80., size=15)
        add_object(scan, bearing=100., distance=.25, size=15)
        self.assertIsNone(tracker.update(scan, .41, acquire=False))
        self.assertEqual(tracker.reason, 'compact_target_ambiguous_held')
        self.assertEqual(tracker.held['track_id'], old['track_id'])

    def test_competing_far_and_near_associations_are_also_ambiguous(self):
        tracker, _ = self.bound()
        scan = self.near(bearing=73., size=15)
        add_object(scan, bearing=107., distance=.4, size=11)
        self.assertIsNone(tracker.update(scan, .41, acquire=False))
        self.assertEqual(tracker.reason, 'compact_target_ambiguous_held')

    def test_near_association_uses_the_same_elapsed_time_gate_as_far(self):
        # Round 55 (Claude): the car keeps moving while a near support is
        # passed; a fixed gate dropped it after one missed frame.
        tracker, old = self.bound()
        scan = self.near(at=.5, bearing=115.)
        candidate = MODULE._candidates(scan['ranges'], min_range_m=.02)[0]
        self.assertTrue(MODULE._associated(candidate, old, .1))
        # Round 57 (Claude): .20 m of motion near the sensor (.25-.40 m) is a
        # 25 deg bearing change; the range-dependent bearing gate accepts it.
        self.assertLess(math.dist(candidate['point_left_m'], old['point_left_m']), .30)
        self.assertTrue(MODULE._associated(candidate, old))
        current = tracker.update(scan, .51, acquire=False)
        self.assertEqual(current['track_id'], old['track_id'])
        self.assertTrue(current['close_maintenance_observation'])
        far = self.near(5, .6, bearing=-90.)  # far outside even the widened gate
        self.assertIsNone(tracker.update(far, .61, acquire=False))

    def test_near_support_cannot_be_cut_out_of_a_wall(self):
        tracker, _ = self.bound()
        scan = self.near()
        # Continuous side wall beyond the compact diameter limit, including
        # the existing role bearing. Full components are formed before range
        # filtering; no range/sector crop can manufacture a target from it.
        for angle in range(205, 336):
            scan['ranges'][angle] = .25 / -math.sin(math.radians(angle))
        self.assertIsNone(tracker.update(scan, .41, acquire=False))

    def test_near_contact_with_a_joined_close_point_is_not_trimmed_to_old_support(self):
        tracker, _ = self.bound()
        scan = self.near()
        for angle in range(281, 346):
            scan['ranges'][angle] = .25
        self.assertIsNone(tracker.update(scan, .41, acquire=False))

    def test_near_repeated_reads_keep_source_identity_and_never_refresh_lease(self):
        tracker, _ = self.bound()
        scan = self.near()
        current = tracker.update(scan, .41, acquire=False)
        repeated = tracker.update(scan, .50, acquire=False)
        self.assertEqual(repeated, current)
        self.assertIsNone(tracker.update(scan, .701, acquire=False))
        self.assertEqual(tracker.reason, 'compact_target_stale')
        self.assertIsNone(tracker.update(self.near(5, .8), .81, acquire=False))

    def test_invalid_duplicate_and_clock_gap_revoke_near_permission(self):
        for fault in ('duplicate_changed', 'source_order', 'receive_order', 'observation_gap'):
            with self.subTest(fault=fault):
                tracker, _ = self.bound()
                scan = self.near()
                self.assertIsNotNone(tracker.update(scan, .41, acquire=False))
                if fault == 'duplicate_changed':
                    scan['ranges'][270] = .24
                else:
                    scan = self.near(5, .5)
                    if fault == 'source_order': scan['at_ms'] = 399
                    if fault == 'receive_order': scan['received_at'] = .399
                    if fault == 'observation_gap':
                        scan['at_ms'], scan['received_at'] = 710, .71
                self.assertIsNone(tracker.update(scan, .72, acquire=False))
                self.assertIsNone(tracker.update(self.near(6, .8), .81, acquire=False))

    def test_new_far_identity_does_not_inherit_old_role_near_permission(self):
        tracker, old = self.bound()
        # Expire the old observation, then acquire a different identity under
        # unchanged normal acquisition rules. Explicit role binding is needed.
        for seq, at in ((4, .7), (5, .8), (6, 1.)):
            current = tracker.update(compact_scan(seq, round(at*1000), at,
                                                 distance=.4, size=21), at+.01)
        self.assertTrue(current['confirmed'])
        self.assertNotEqual(current['track_id'], old['track_id'])
        self.assertIsNone(tracker.update(self.near(7, 1.1), 1.11, acquire=False))

    def test_far_tolerant_edges_and_internal_gaps_stay_identical_when_enabled(self):
        plain, _ = self.bound(enabled=False)
        enabled, _ = self.bound()
        for seq, missing in ((4, [259, 258]), (5, [270]), (6, [281, 282])):
            at = seq/10
            scan = compact_scan(seq, round(at*1000), at, distance=.4, size=21)
            for index in missing: scan['ranges'][index] = None
            actual = enabled.update(copy.deepcopy(scan), at+.01)
            self.assertEqual(actual, plain.update(scan, at+.01))
            self.assertIsNotNone(actual)
            self.assertFalse(actual['close_maintenance_observation'])

    def test_candidate_minimum_range_override_is_validated_and_not_global(self):
        scan = self.near()
        self.assertEqual(len(MODULE._candidates(scan['ranges'], min_range_m=.02)), 1)
        self.assertEqual(MODULE._candidates(scan['ranges']), [])
        for invalid in (True, None, '0.02', .019, .351, float('nan'), float('inf')):
            with self.subTest(invalid=invalid):
                with self.assertRaises(ValueError):
                    MODULE._candidates(scan['ranges'], min_range_m=invalid)


if __name__ == '__main__':
    unittest.main()
