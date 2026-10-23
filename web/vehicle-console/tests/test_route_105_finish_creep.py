"""Round 105 (Claude): a car stopped with the front in the finish zone and the
rear out, too close for the 3-frame restart kick (car runs 103/104: .404/.459
m from the end board), creeps in with LANE_CREEP_FRAMES 1560 frames, at most
LANE_CREEP_MAX times. Synthetic scans; no physical claim."""
import unittest

import maneuver_sequence as ms
from turn_motion import NEUTRAL, TRIAL_MOTOR
import test_route_58_light_lane as lane

FRONT = ms.FRONT_BODY_EXTENT_M


class FinishCreep(unittest.TestCase):
    lane_motion, scan, drive = (lane.LightLaneTests.lane_motion, lane.LightLaneTests.scan,
                                lane.LightLaneTests.drive)

    def runs(self, results):
        """(first index, length) of each run of 1560 outputs."""
        out, start = [], None
        for k, r in enumerate(results+[{'motor': NEUTRAL}]):
            if r['motor'] == TRIAL_MOTOR and start is None:
                start = k
            elif r['motor'] != TRIAL_MOTOR and start is not None:
                out.append((start, k-start))
                start = None
        return out

    def test_run_104_stop_creeps_and_ends_after_the_last_creep_if_it_never_moves(self):
        motion = self.lane_motion()
        results = self.drive(motion, [0.]*60, start=FRONT+.459)
        pulses = self.runs(results)
        self.assertEqual([n for _, n in pulses], [ms.LANE_CREEP_FRAMES]*ms.LANE_CREEP_MAX)
        # Each creep waits LANE_KICK_REST_S after the previous power.
        self.assertTrue(all(b[0]-a[0] >= 10 for a, b in zip(pulses, pulses[1:])), pulses)
        last = results[-1]
        self.assertEqual(last['reason'], 'route_finish_zone_stop')
        self.assertEqual(last['finish_evidence']['basis'], 'stopped_front_in_finish_zone_creeps_used')
        self.assertGreaterEqual(results.index(last)-pulses[-1][0], 10)  # ended once the last creep rested

    def test_a_creep_that_moves_the_car_ends_with_the_whole_car_in_the_zone(self):
        motion = self.lane_motion()
        stopped = self.drive(motion, [0.]*6, start=FRONT+.43)
        self.assertEqual([n for _, n in self.runs(stopped)], [ms.LANE_CREEP_FRAMES])
        self.assertEqual(stopped[-1]['lane_feedback']['creeps'], 1)
        # ~.1 m of creep (the drive helper closes the board by v x .1 s a scan).
        moved = self.drive(motion, [.2, .3, .3, .2, .1]+[0.]*8, start=FRONT+.43, seq0=11)
        last = moved[-1]
        self.assertEqual(last['finish_evidence']['basis'], 'stopped_with_the_whole_car_in_the_finish_zone')
        self.assertLessEqual(last['finish_evidence']['front_gap_m'], ms.LANE_FINISH_BODY_GAP_M)
        self.assertNotIn(TRIAL_MOTOR, [r['motor'] for r in moved])

    def test_kick_room_and_whole_car_stops_are_unchanged(self):
        motion = self.lane_motion()
        kicked = self.drive(motion, [0.]*6, start=FRONT+ms.LANE_FINISH_MIN_GAP_M+ms.LANE_KICK_TRAVEL_M+.02)
        self.assertEqual([n for _, n in self.runs(kicked)], [ms.LANE_KICK_FRAMES])
        self.assertEqual(kicked[-1]['lane_feedback']['creeps'], 0)
        motion = self.lane_motion()
        inside = self.drive(motion, [0.]*4, start=FRONT+.38)
        self.assertEqual(inside[-1]['finish_evidence']['basis'], 'stopped_with_the_whole_car_in_the_finish_zone')
        self.assertNotIn(TRIAL_MOTOR, [r['motor'] for r in inside])

    def test_the_creep_cannot_reach_the_board_on_the_guards_own_model(self):
        # Two frames from rest at the kick's accel bound, the ESC skew on both
        # ends and the weakest coast: within LANE_CREEP_TRAVEL_M (3 frames: the
        # kick's LANE_KICK_TRAVEL_M scale).
        a, d, skew = ms.LANE_KICK_ACCEL_MAX_MPS2, ms.LANE_STOP_DECEL_MIN_MPS2, 2*ms.LANE_ESC_SKEW_S
        def travel(frames):
            t = frames*.1+skew
            v = ms.LANE_SPEED_GUARD_MARGIN_MPS+a*t
            return v*v/(2*d)
        self.assertLessEqual(travel(ms.LANE_CREEP_FRAMES)*ms.LANE_KICK_TRAVEL_M/travel(ms.LANE_KICK_FRAMES),
                             ms.LANE_CREEP_TRAVEL_M)
        self.assertGreater(ms.LANE_FINISH_BODY_GAP_M-ms.LANE_CREEP_TRAVEL_M, ms.LANE_FINISH_MIN_GAP_M+.05)


if __name__ == '__main__':
    unittest.main()
