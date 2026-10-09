"""Optional role sectors retain real support, association gates and clock leases."""
import copy
import math
import unittest

from test_compact_target import MODULE, compact_scan, double_gap_scan


def add_object(scan, *, bearing, distance=1., size=9):
    middle = round(-bearing) % 360
    for offset in range(-(size//2), size-size//2):
        scan['ranges'][(middle+offset) % 360] = distance
    return scan


class CompactTargetRoleTests(unittest.TestCase):
    def first_tracker(self):
        return MODULE.CompactTargetTracker(maintenance_bearing_rad=(-math.pi, math.pi),
                                            select_associated=True)

    def second_tracker(self):
        return MODULE.CompactTargetTracker(acquisition_bearing_rad=(-math.pi/2, 0.),
                                            maintenance_bearing_rad=(-3*math.pi/4, 0.),
                                            select_associated=True)

    def confirm(self, tracker, *, bearing=90.):
        result = None
        for seq, published, received in ((1, 0, 0.), (2, 100, .1), (3, 300, .3)):
            result = tracker.update(compact_scan(seq, published, received, bearing=bearing), received+.01)
        self.assertTrue(result['confirmed'])
        return result

    def test_configuration_rejects_nonfinite_unordered_or_outside_intervals(self):
        invalid = (None, (), (0.,), (0., 1., 2.), 'right', (0., 0.), (1., 0.),
                   (-math.pi-.01, 0.), (0., math.pi+.01), (True, 1.),
                   (0., float('inf')), (float('nan'), 0.))
        for interval in invalid:
            with self.subTest(interval=interval):
                with self.assertRaises(ValueError):
                    MODULE.CompactTargetTracker(acquisition_bearing_rad=interval)
                with self.assertRaises(ValueError):
                    MODULE._candidates([3.]*360, bearing_range_rad=interval)
                if interval is not None:
                    with self.assertRaises(ValueError):
                        MODULE.CompactTargetTracker(maintenance_bearing_rad=interval)

    def test_boolean_flags_do_not_accept_truthy_values(self):
        for value in (0, 1, None, 'false', [], {}):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    MODULE.CompactTargetTracker(select_associated=value)
                with self.assertRaises(ValueError):
                    MODULE.CompactTargetTracker().update(compact_scan(), .01, acquire=value)
                with self.assertRaises(ValueError):
                    MODULE._candidates([3.]*360, allow_boundary_gap=value)

    def test_options_are_copied_and_do_not_mutate_default_sector(self):
        interval = [-math.pi/2, 0.]
        right = MODULE.CompactTargetTracker(acquisition_bearing_rad=interval)
        interval[:] = [math.pi/4, 3*math.pi/4]
        self.assertIsNotNone(right.update(compact_scan(bearing=-45.), .01))
        self.assertIsNone(MODULE.CompactTargetTracker().update(compact_scan(bearing=-45.), .01))
        self.assertIsNotNone(MODULE.CompactTargetTracker().update(compact_scan(), .01))
        self.assertEqual(MODULE._candidates(compact_scan(bearing=-45.)['ranges']), [])

    def test_default_still_discards_two_candidates_while_role_keeps_first_identity(self):
        default, role = MODULE.CompactTargetTracker(), self.first_tracker()
        self.confirm(default)
        first = self.confirm(role)
        scan = add_object(compact_scan(4, 400, .4), bearing=110., distance=1.5)
        self.assertIsNone(default.update(scan, .41))
        self.assertEqual(default.reason, 'compact_target_ambiguous')
        current = role.update(scan, .41, acquire=False)
        self.assertEqual(current['track_id'], first['track_id'])
        self.assertEqual(current['source_seq'], 4)
        self.assertEqual(current['support_bins'], first['support_bins'])

    def test_full_circle_maintenance_keeps_first_identity_past_left_sector(self):
        tracker = self.first_tracker()
        first = self.confirm(tracker)
        for seq, bearing in enumerate((105., 120., 135., 150., 165., 179., -177.), start=4):
            received = seq/10
            scan = add_object(compact_scan(seq, seq*100, received, bearing=bearing),
                              bearing=-45., distance=1.5)
            current = tracker.update(scan, received+.01, acquire=False)
            self.assertEqual(current['track_id'], first['track_id'])
            self.assertTrue(current['confirmed'])
            self.assertEqual(current['source_seq'], seq)
            self.assertAlmostEqual(current['bearing_left_rad'], math.radians(bearing))

    def test_association_wraps_rear_bearing_without_expanding_twenty_degree_gate(self):
        for bearing, accepted in ((-179., True), (-155., False)):
            with self.subTest(bearing=bearing):
                tracker = MODULE.CompactTargetTracker(acquisition_bearing_rad=(math.radians(170), math.pi),
                                                       maintenance_bearing_rad=(-math.pi, math.pi),
                                                       select_associated=True)
                first = self.confirm(tracker, bearing=179.)
                current = tracker.update(compact_scan(4, 400, .4, bearing=bearing), .41, acquire=False)
                if accepted:
                    self.assertEqual(current['track_id'], first['track_id'])
                else:
                    self.assertIsNone(current)

    def test_acquire_false_cannot_initialize_or_rebuild_consumed_identity(self):
        tracker = self.first_tracker()
        self.assertIsNone(tracker.update(compact_scan(), .01, acquire=False))
        self.assertEqual(tracker._next_id, 1)
        tracker = self.first_tracker()
        first = self.confirm(tracker)
        missing = compact_scan(4, 400, .4)
        missing['ranges'] = [3.]*360
        self.assertIsNone(tracker.update(missing, .41, acquire=False))
        for seq in (5, 6, 7):
            self.assertIsNone(tracker.update(compact_scan(seq, seq*100, seq/10), seq/10+.01, acquire=False))
        self.assertEqual(tracker._next_id, first['track_id']+1)

    def test_acquire_false_cannot_replace_unconfirmed_identity_after_jump(self):
        tracker = self.first_tracker()
        first = tracker.update(compact_scan(), .01)
        self.assertFalse(first['confirmed'])
        self.assertIsNone(tracker.update(compact_scan(2, 100, .1, bearing=60.), .11, acquire=False))
        self.assertEqual(tracker.reason, 'compact_target_acquisition_disabled')

    def test_confirmed_association_loss_does_not_switch_to_an_unrelated_candidate(self):
        tracker = self.first_tracker()
        first = self.confirm(tracker)
        self.assertIsNone(tracker.update(compact_scan(4, 400, .4, bearing=60.), .41))
        recovered = tracker.update(compact_scan(5, 500, .5, bearing=60.), .51)
        self.assertFalse(recovered['confirmed'])
        self.assertNotEqual(recovered['track_id'], first['track_id'])

    def test_two_current_associated_components_are_ambiguous(self):
        tracker = self.first_tracker()
        first = self.confirm(tracker)
        scan = add_object(compact_scan(4, 400, .4, bearing=80.), bearing=100.)
        candidates = MODULE._candidates(scan['ranges'])
        self.assertEqual(len(candidates), 2)
        self.assertTrue(all(MODULE._associated(candidate, first) for candidate in candidates))
        self.assertIsNone(tracker.update(scan, .41, acquire=False))
        self.assertEqual(tracker.reason, 'compact_target_ambiguous')
        self.assertIsNone(tracker.update(compact_scan(5, 500, .5), .51, acquire=False))

    def test_second_role_acquires_forward_right_and_maintains_confirmed_right_rear(self):
        for bearing, accepted in ((90., False), (-120., False), (-45., True)):
            with self.subTest(bearing=bearing):
                result = self.second_tracker().update(compact_scan(bearing=bearing), .01)
                self.assertEqual(result is not None, accepted)
        tracker = self.second_tracker()
        second = self.confirm(tracker, bearing=-75.)
        for seq, bearing in enumerate((-90., -105., -120.), start=4):
            current = tracker.update(compact_scan(seq, seq*100, seq/10, bearing=bearing), seq/10+.01)
            self.assertEqual(current['track_id'], second['track_id'])
        self.assertIsNone(tracker.update(compact_scan(7, 700, .7, bearing=-140.), .71))

    def test_maintenance_sector_does_not_admit_new_or_unconfirmed_identity(self):
        tracker = self.first_tracker()
        self.assertIsNone(tracker.update(compact_scan(bearing=150.), .01))
        tracker = self.second_tracker()
        self.assertIsNotNone(tracker.update(compact_scan(bearing=-80.), .01))
        self.assertIsNone(tracker.update(compact_scan(2, 100, .1, bearing=-95.), .11))
        tracker = MODULE.CompactTargetTracker(maintenance_bearing_rad=(-math.pi, math.pi))
        self.confirm(tracker)
        self.assertIsNone(tracker.update(compact_scan(4, 400, .4, bearing=-45.), .41))
        self.assertEqual(tracker.reason, 'compact_target_outside_acquisition_sector')

    def test_role_configuration_preserves_whole_component_near_far_and_null_gates(self):
        for change in ('long', 'near', 'far', 'null', 'few_points', 'near_boundary'):
            with self.subTest(change=change):
                tracker = self.second_tracker()
                scan = compact_scan(bearing=-45.)
                if change == 'long':
                    for index in range(0, 90): scan['ranges'][index] = 1.
                if change == 'near': scan = compact_scan(bearing=-45., distance=.3)
                if change == 'far': scan = compact_scan(bearing=-45., distance=3.1)
                if change == 'null': scan['ranges'][50] = None
                if change == 'few_points': scan = compact_scan(bearing=-45., size=4)
                if change == 'near_boundary': scan['ranges'][50] = .5
                self.assertIsNone(tracker.update(scan, .01))

    def test_partial_old_support_can_coexist_without_maturing_or_renewing_lease(self):
        tracker = self.first_tracker()
        first = self.confirm(tracker)
        for seq in (4, 5):
            scan = add_object(double_gap_scan(seq, seq*100, seq/10), bearing=-45., distance=1.5)
            current = tracker.update(scan, seq/10+.01, acquire=False)
            self.assertEqual(current['track_id'], first['track_id'])
            self.assertEqual(current['confirmation_count'], first['confirmation_count'])
            self.assertEqual(current['last_full_isolation_seq'], 3)
            self.assertEqual(current['support_bins'], first['support_bins'])
        self.assertIsNone(tracker.update(double_gap_scan(6, 600, .6), .61, acquire=False))
        self.assertEqual(tracker.reason, 'compact_target_boundary_gap_expired')
        self.assertIsNone(tracker.update(compact_scan(7, 700, .7), .71, acquire=False))

    def test_role_partial_cannot_acquire_or_mature_and_both_clocks_still_confirm(self):
        for seeded in (False, True):
            with self.subTest(seeded=seeded):
                tracker = self.second_tracker()
                if seeded:
                    tracker.update(compact_scan(bearing=-45.), .01)
                self.assertIsNone(tracker.update(double_gap_scan(2, 100, .1, bearing=-45.), .11))
        tracker = self.second_tracker()
        for seq, published, received in ((1, 0, 0.), (2, 150, .1), (3, 300, .2)):
            current = tracker.update(compact_scan(seq, published, received, bearing=-45.), received+.01)
        self.assertFalse(current['confirmed'])

    def test_stale_and_observation_gap_cannot_reacquire_consumed_target(self):
        for published, received, now in ((400, .4, .71), (800, .8, .81)):
            with self.subTest(published=published):
                tracker = self.first_tracker()
                first = self.confirm(tracker)
                self.assertIsNone(tracker.update(compact_scan(4, published, received), now, acquire=False))
                self.assertEqual(tracker._next_id, first['track_id']+1)

    def test_explicit_default_options_preserve_entire_output_and_reasons(self):
        implicit = MODULE.CompactTargetTracker()
        explicit = MODULE.CompactTargetTracker(acquisition_bearing_rad=(math.pi/4, 3*math.pi/4),
                                               maintenance_bearing_rad=None, select_associated=False)
        scans = [compact_scan(1, 0, 0.), compact_scan(2, 100, .1), compact_scan(3, 300, .3),
                 double_gap_scan(), compact_scan(5, 500, .5, bearing=60.),
                 add_object(compact_scan(6, 600, .6), bearing=110., distance=1.5)]
        for scan in scans:
            self.assertEqual(implicit.update(copy.deepcopy(scan), scan['received_at']+.01),
                             explicit.update(copy.deepcopy(scan), scan['received_at']+.01, acquire=True))
            self.assertEqual(implicit.reason, explicit.reason)


if __name__ == '__main__':
    unittest.main()
