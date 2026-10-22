"""Round 103 (Claude): a start-lane stopping range that reaches the board ahead
before the first support cuts the power for that scan instead of ending the
run (car run 102: a closing speed read off the end board ~7 m ahead, 1.77 m/s
at ~1.3 m/s, ended the start lane .3 m after the start). Closed loop through
scripts/course-sim.py; synthetic, no physical claim."""
import importlib.util
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).parents[1]))
import maneuver_sequence as ms  # noqa: E402

ROOT = Path(__file__).parents[3]
TOOL = ROOT/'tools'/'scan-geometry'/'target'/'release'/'xt-stcar-scan-geometry'
FORCED = (18, 30)  # scans whose board ahead is forced to 1.2 m (t 1.7-2.9 s)


@unittest.skipUnless(TOOL.exists(), 'tools/scan-geometry not built (cargo build --release)')
class StartLaneCut(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location('course_sim_103', ROOT/'scripts/course-sim.py')
        cls.cs = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.cs)

    def run_course(self, course, *, forced=None, max_s=45.):
        orig_range, orig_event = ms.ManeuverSequence._lane_front_range, ms.ManeuverSequence._route_event
        events = []

        def front(self, scan):
            value = orig_range(self, scan)
            if (forced and self.route_stage == 'start_lane' and forced[0] <= scan['seq'] <= forced[1]
                    and value is not None):
                return 1.2
            return value

        def event(self, name, scan, control, evidence=None):
            events.append((name, scan['seq'], evidence))
            return orig_event(self, name, scan, control, evidence)
        try:
            ms.ManeuverSequence._lane_front_range, ms.ManeuverSequence._route_event = front, event
            summary, log, _ = self.cs.run(course, max_s=max_s, vehicle='measured')
        finally:
            ms.ManeuverSequence._lane_front_range, ms.ManeuverSequence._route_event = orig_range, orig_event
        return summary, log, events

    def test_a_board_ahead_cuts_the_power_and_the_lane_drives_on(self):
        summary, log, events = self.run_course(self.cs.Course(**self.cs.LAYOUTS['user8']), forced=FORCED)
        cuts = [seq for name, seq, _ in events if name == 'start_lane_board_ahead_cut']
        self.assertTrue(cuts and FORCED[0] <= cuts[0] <= FORCED[1], cuts)
        window = [r for r in log if FORCED[0]+1 <= round(r[0]*10)+1 <= FORCED[1]]
        self.assertTrue(window)
        for row in window:  # still the start lane's drive, unpowered, on the lane law
            self.assertEqual((row[5], row[7], row[8]), (1500, 'start_lane', 'drive'), row)
            self.assertTrue(row[9].startswith('start_lane_'), row)
        self.assertTrue(any(r[5] > 1500 and r[7] == 'start_lane' for r in log if round(r[0]*10)+1 > FORCED[1]))
        self.assertNotIn('coast', [r[8] for r in log if r[7] == 'start_lane'])
        self.assertTrue(summary['route_completed'], summary)
        self.assertEqual(summary['contacts'], [])

    def test_without_a_support_the_car_stops_short_of_the_board_and_waits(self):
        spec = dict(self.cs.LAYOUTS['user8'], cones=[(-3., -3.), (-4., -4.)])  # outside the field
        course = self.cs.Course(**spec)
        summary, log, events = self.run_course(course, max_s=20.)
        self.assertEqual(summary['contacts'], [])
        self.assertGreater(summary['min_board_gap_m'], .15)
        self.assertIsNone(summary['end_reason'])  # still waiting when the simulation stops
        self.assertEqual(log[-1][7:9], ('start_lane', 'drive'))
        self.assertLess(log[-1][4], .01)
        self.assertLess(summary['final_front_x'], course.L-ms.LANE_START_SEARCH_M/2)


if __name__ == '__main__':
    unittest.main()
