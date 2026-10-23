"""Round 106 (Claude): in the transfer the second support standing on the axis
was itself the return ahead, so the confirmation grace (a tracked support
nearer than the board) never applied and the board-ahead hold stopped the car
just before the support's confirmation (car runs 98/99/101/105, .4-.7 s).
The board is now the nearest axis return that is not one of the support's."""
import json
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).parents[1]))
import maneuver_sequence as ms  # noqa: E402

FIXTURE = Path(__file__).parent/'fixtures'/'run105_seq89_transfer_axis_support.json'


def transfer(heading, target, role_seq):
    motion = ms.ManeuverSequence(0., continue_route=True)
    motion.entry_kind, motion.route_stage = 'lane', 'open_transfer'
    motion.compass.current = lambda scan: heading
    motion.second_target = dict(target)
    motion.role_distinct_evidence = {'source_seq': role_seq}
    return motion


class AxisSupport(unittest.TestCase):
    def setUp(self):
        self.fx = json.loads(FIXTURE.read_text())
        self.scan = {'seq': self.fx['seq'], 'ranges': self.fx['ranges']}

    def test_run_105_seq_89_support_is_the_return_ahead_and_matures(self):
        fx = self.fx
        motion = transfer(fx['compass_heading_rad'], fx['second_target'], fx['role_distinct_source_seq'])
        ahead = motion._lane_front_range(self.scan)
        stop = ms.FRONT_BODY_EXTENT_M+ms.LANE_FINISH_GAP_M+ms.LANE_SPEED_UNKNOWN_MPS**2/(
            2*ms.LANE_START_STOP_DECEL_MPS2)+ms.LANE_SPEED_UNKNOWN_MPS*ms.LANE_REACTION_S
        target = fx['second_target']
        self.assertAlmostEqual(ahead, target['range_m'], delta=.02)  # the support's own face
        self.assertGreaterEqual(target['range_m'], ahead)  # so the old "nearer than the board" failed
        board = motion._lane_front_range(self.scan, exclude=set(target['support_bins']))
        self.assertGreater(board, 5.)  # the end board behind it
        self.assertTrue(motion._open_second_maturing(self.scan, ahead, stop))
        # Inside the grace's end the hold applies as before.
        self.assertFalse(motion._open_second_maturing(self.scan, stop-ms.OPEN_SECOND_CONFIRM_GRACE_M, stop))

    def test_a_board_nearer_than_the_support_still_holds(self):
        fx = self.fx
        ranges = list(fx['ranges'])
        board_bin = min(fx['second_target']['support_bins'])-2  # an axis bin beside the support
        ranges[board_bin % 360] = 2.6
        scan = {'seq': fx['seq'], 'ranges': ranges}
        motion = transfer(fx['compass_heading_rad'], fx['second_target'], fx['role_distinct_source_seq'])
        ahead = motion._lane_front_range(scan)
        self.assertLess(ahead, fx['second_target']['range_m'])
        self.assertFalse(motion._open_second_maturing(scan, ahead, 2.885))

    def test_without_support_bins_or_ranges_the_old_rule_stands(self):
        fx = self.fx
        target = {k: v for k, v in fx['second_target'].items() if k != 'support_bins'}
        motion = transfer(fx['compass_heading_rad'], target, fx['role_distinct_source_seq'])
        ahead = motion._lane_front_range(self.scan)
        self.assertFalse(motion._open_second_maturing(self.scan, ahead, 2.885))
        motion = transfer(fx['compass_heading_rad'], fx['second_target'], fx['role_distinct_source_seq'])
        self.assertFalse(motion._open_second_maturing({'seq': fx['seq']}, ahead, 2.885))


class RecordedHolds(unittest.TestCase):
    def test_every_recorded_hold_start_matures_now(self):
        # Runs 98/99/101/105: the support was the return ahead (old rule: hold).
        frames = json.loads((Path(__file__).parent/'fixtures'/'transfer_axis_support_frames.json').read_text())['frames']
        self.assertEqual([f['run'] for f in frames], [98, 99, 101, 105])
        for f in frames:
            motion = transfer(f['compass_heading_rad'], f['second_target'], f['role_distinct_source_seq'])
            scan = {'seq': f['seq'], 'at_ms': f['at_ms'], 'ranges': f['ranges']}
            ahead, stop = motion._lane_front_range(scan), f['python']['stop_at']
            with self.subTest(run=f['run']):
                if f['run'] != 105:  # 105: the frame before its hold
                    self.assertLessEqual(ahead, stop)
                self.assertGreaterEqual(f['second_target']['range_m'], ahead)
                self.assertTrue(motion._open_second_maturing(scan, ahead, stop))
                self.assertEqual(ahead, f['python']['ahead'])  # the values the Rust test checks too


if __name__ == '__main__':
    unittest.main()
