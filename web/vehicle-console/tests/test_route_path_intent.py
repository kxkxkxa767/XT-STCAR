"""Obstacle corrections preserve the task's direction and adopted-center protocol."""
import math
import unittest
from unittest.mock import patch

import test_maneuver_route as route


class RoutePathIntentTests(unittest.TestCase):
    STAGES = ('right_align', 'corridor_follow', 'second_orbit', 'light_turn')

    def motion(self, stage, target, *, authorized=True):
        motion = route.RoutePathCheckTests.motion(stage, right=authorized)
        if stage == 'second_orbit' and authorized:
            motion.second_center_ack = {'servo': 1500}
        motion.steering_target = motion.natural_steering_target = target
        motion.servo = 1500
        wall = {i: .8/math.cos(math.radians(i)) for i in range(-60 % 360, 360)}
        wall.update({i: .8/math.cos(math.radians(i)) for i in range(0, 6)})
        motion._current_scan = route.RoutePathCheckTests.scan(3, wall)
        return motion

    def test_only_opposite_clear_arc_does_not_reverse_any_post_pass_stage(self):
        for stage in self.STAGES:
            for target in (1610, 1420):
                with self.subTest(stage=stage, target=target):
                    motion = self.motion(stage, target)
                    if target < 1500:
                        motion._current_scan['ranges'] = [
                            motion._current_scan['ranges'][-i % 360] for i in range(360)]
                    self.assertEqual(motion._route_path_target(target), target)
                    self.assertFalse(motion.path_evidence['overridden'])

    def test_neutral_stage_cannot_be_turned_by_path_or_proximity(self):
        for stage in self.STAGES:
            for authorized in (False, True):
                with self.subTest(stage=stage, authorized=authorized):
                    motion = self.motion(stage, 1500, authorized=authorized)
                    motion.natural_steering_target = 1350
                    self.assertEqual(motion._route_path_target(1500), 1500)
                    for side in ('right', 'left', 'front_left_open', 'front_right_open'):
                        motion._avoid = {'side': side}
                        self.assertEqual(motion._avoid_target(1500), 1500)

    def test_clear_same_direction_arc_still_corrects_a_stage(self):
        for stage in self.STAGES:
            with self.subTest(stage=stage):
                motion = self.motion(stage, 1450)
                chosen = motion._route_path_target(1450)
                self.assertLess(chosen, 1500)
                self.assertGreaterEqual(motion._path_free_distance(
                    motion._path_points(motion._current_scan), chosen), .7)

    def test_waiting_right_command_is_not_reversed_by_proximity(self):
        for stage in self.STAGES:
            with self.subTest(stage=stage):
                motion = self.motion(stage, 1420)
                motion.phase, motion.neutral_pause_kind = 'quality_wait', 'post_pass'
                motion._avoid = {'side': 'right'}
                self.assertLessEqual(motion._avoid_target(1420), 1500)
                motion._avoid = None
                self.assertEqual(motion._avoid_target(1420), 1420)

    def test_first_orbit_release_cannot_be_undone_by_either_layer(self):
        motion = self.motion('first_target', 1600, authorized=False)
        motion.servo = 1690
        motion.first_pass_preparing = True
        # The only clear arc turns harder left than the requested release.
        with patch.object(motion, '_path_free_distance', side_effect=lambda _, pwm: 1. if pwm == 1660 else .2):
            self.assertEqual(motion._route_path_target(1600), 1600)
        motion._avoid = {'side': 'right'}
        self.assertEqual(motion._avoid_target(1600), 1600)
        motion.phase = 'quality_wait'
        motion.quality_wait_release = {'basis': 'current_support'}
        self.assertEqual(motion._avoid_target(1600), 1600)

    def test_release_cache_changes_when_current_servo_changes(self):
        motion = self.motion('first_target', 1600, authorized=False)
        with patch.object(motion, '_path_free_distance', side_effect=lambda _, pwm: 1. if pwm == 1660 else .2):
            motion.servo = 1550  # stage currently asks to increase left
            self.assertEqual(motion._route_path_target(1600), 1660)
            motion.servo = 1690  # same scan/target now requests release
            self.assertEqual(motion._route_path_target(1600), 1600)

    def test_first_target_release_limit_does_not_block_exit_left(self):
        motion = self.motion('right_align', 1610)
        motion.servo = 1690
        motion.first_pass_preparing = True
        with patch.object(motion, '_path_free_distance', side_effect=lambda _, pwm: 1. if pwm == 1660 else .2):
            self.assertEqual(motion._route_path_target(1610), 1660)


if __name__ == '__main__':
    unittest.main()
