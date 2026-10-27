"""Round 108 (Claude): the light pursuit aims at the divider's open end no
nearer than OPEN_LANE_PURSUIT_MIN_M (car run 107: a .906 m horizon at
1.47 m/s handed over to the light lane at 1382 and the car swung 38 deg past
the axis into the divider). The near-line shortening, the lane admission and
every other pursuit keep their .8 m floor. Synthetic lane targets; no
physical claim."""
import math
from pathlib import Path
import sys
import unittest
import unittest.mock

sys.path.insert(0, str(Path(__file__).parents[1]))
import maneuver_sequence as ms  # noqa: E402


def pursuit(start, centre, heading_deg=10.):
    m = ms.ManeuverSequence.__new__(ms.ManeuverSequence)
    lane = {'outer_lateral_m': centre+.55, 'inner_lateral_m': None if start is None else centre-.55,
            'inner_start_along_m': start, 'width_m': 1.1, 'center_lateral_m': centre}
    m._open_lane_target = lambda scan, heading: lane
    m.light_stop_state = None  # round y3: the pursuit reads the light stop
    heading = math.radians(heading_deg)
    _, point, _, pwm = m._open_light_pursuit({}, heading)
    u = (math.cos(-heading), math.sin(-heading))
    return point[0]*u[0]+point[1]*u[1], pwm


class LightPursuitFloor(unittest.TestCase):
    def test_constants(self):
        self.assertEqual(ms.OPEN_LANE_LOOKAHEAD_M, (.8, 1.6))
        self.assertEqual(ms.OPEN_LANE_PURSUIT_MIN_M, 1.4)

    def test_run_107_hand_over_aims_farther_and_turns_less(self):
        # Seq 144: divider end .906 m ahead, centre line .157 m right, 10 deg left of the axis.
        along, pwm = pursuit(.906, -.157)
        self.assertAlmostEqual(along, ms.OPEN_LOOKAHEAD_M+.157, places=6)  # the near-line shortening still bounds it
        self.assertEqual(pwm, 1426)
        with unittest.mock.patch.object(ms, 'OPEN_LANE_PURSUIT_MIN_M', ms.OPEN_LANE_LOOKAHEAD_M[0]):
            along, pwm = pursuit(.906, -.157)
            self.assertAlmostEqual(along, .906, places=6)
            self.assertEqual(pwm, 1382)  # the old floor: the car's command at seq 145

    def test_far_divider_end_and_no_divider_are_unchanged(self):
        self.assertAlmostEqual(pursuit(1.5, -.05)[0], 1.15, places=6)
        self.assertAlmostEqual(pursuit(1.5, -.5)[0], 1.5, places=6)
        self.assertAlmostEqual(pursuit(None, -.6)[0], 1.6, places=6)
        self.assertAlmostEqual(pursuit(.1, -.05)[0], 1.15, places=6)  # end behind the body front

    def test_divider_end_nearer_than_the_floor(self):
        for start in (.5, .8, 1.2):
            with self.subTest(start=start):
                self.assertAlmostEqual(pursuit(start, -.4)[0], ms.OPEN_LANE_PURSUIT_MIN_M, places=6)


if __name__ == '__main__':
    unittest.main()
