"""Round 87: control-thread load and gap telemetry. Synthetic scans and
clocks; no physical claims."""
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).parents[1]))
import autonomy_live as al
from test_autonomy_live import wall_scan
import test_turn_control as control


AGES = {'camera': .01, 'lidar': .01, 'control': .01}


def maneuver(**extra):
    return dict(camera_required=False, clearance_profile='maneuver', **extra)


class Counting:
    def __init__(self):
        self.calls = 0

    def __call__(self, *args, **kwargs):
        self.calls += 1
        return al.probe_clearance(*args, **kwargs)


class ClearanceReuseTests(unittest.TestCase):
    def setUp(self):
        self.scan = wall_scan(width=1.2)
        self.reuse, self.check = al.ClearanceReuse(), Counting()

    def probe(self, scan=None, ages=AGES, **kwargs):
        return self.reuse.probe(self.check, self.scan if scan is None else scan, ages, False, True,
                                **(kwargs or maneuver()))

    def test_unchanged_scan_and_inputs_reuse_an_equal_copy(self):
        first = self.probe()
        first['quality_issues'] = ['changed by the caller']
        again = self.probe(dict(self.scan), dict(AGES, lidar=.2))
        self.assertEqual(self.check.calls, 1)
        self.assertEqual(self.reuse.reused, 1)
        self.assertEqual(again, al.probe_clearance(self.scan, AGES, False, True, **maneuver()))
        self.assertIsNot(again, first)

    def test_any_changed_input_checks_again(self):
        changes = [
            lambda: self.probe(ages=dict(AGES, lidar=al.LIDAR_LATE_S)),       # lidar age limit crossed
            lambda: self.probe(**maneuver(relaxed_turn_left=True)),            # gate changed with the servo
            lambda: self.probe(**maneuver(rear_start=True)),
            lambda: self.probe(**maneuver(front_side_walls=True)),
            lambda: self.scan['ranges'].__setitem__(180, 2.5) or self.probe(),  # the same list changed in place
        ]
        for change in changes:
            self.probe()
            calls = self.check.calls
            self.probe()
            self.assertEqual(self.check.calls, calls)
            change()
            self.assertEqual(self.check.calls, calls+1)

    def test_types_are_compared_too(self):
        self.probe(**maneuver(rear_start=True))
        with self.assertRaises(ValueError):
            self.probe(**maneuver(rear_start=1))  # equal, refused by the probe itself
        self.assertEqual(self.check.calls, 2)

    def test_a_refused_check_is_never_reused(self):
        close = wall_scan(width=.34)  # boards beside the body: inside the gate
        for _ in range(3):
            with self.assertRaises(al.ProbeClearanceError):
                self.probe(close)
        self.assertEqual((self.check.calls, self.reuse.reused), (3, 0))
        for ages in (dict(AGES, control=al.CONTROL_AGE_LIMIT_S), dict(AGES, lidar=math.nan), {}):
            with self.assertRaises(ValueError):
                self.probe(ages=ages)
        self.assertEqual(self.check.calls, 6)

    def test_camera_limits_matter_only_when_the_camera_is_required(self):
        straight = {'clearance_profile': 'straight'}
        scan = wall_scan(width=1.6)
        self.probe(scan, **straight)
        self.probe(scan, dict(AGES, camera=.6), **straight)
        self.probe(scan, dict(AGES, camera=1.), **straight)
        self.assertEqual(self.check.calls, 3)
        self.probe(scan, dict(AGES, camera=5.))
        self.probe(scan, dict(AGES, camera=.01))
        self.assertEqual(self.check.calls, 4)


class TickGapBreakdown(unittest.TestCase):
    def test_gap_splits_into_tick_idle_late_wake_and_lock_wait(self):
        harness = control.TurnControlTests('test_watchdog_terminal_gap_result_keeps_measured_lock_and_tick_wall_time')
        harness.setUp()
        self.addCleanup(harness.temp.cleanup)
        harness.start_compact_trial()
        console = harness.console
        session = console.auto_session
        clock = {'t': 1., 'waits': 0}
        walls, lates, locks = iter([.088, .010, .030]), iter([0., .030, .002]), iter([.001, .016, .004])

        class Lock:
            def __enter__(self):
                clock['t'] += next(locks)

            def __exit__(self, *exc):
                return False

        def wait(timeout):
            clock['waits'] += 1
            if clock['waits'] > 3:
                return True
            clock['t'] += timeout+next(lates)
            return False

        def tick(now):
            clock['t'] += next(walls)
        console.lock = Lock()
        with patch.object(console.stop, 'wait', wait), patch.object(console, 'autonomy_tick', tick), \
             patch.object(harness.server.time, 'monotonic', lambda: clock['t']):
            console.autonomy_watch()
        report = session['report']
        worst = report['max_gap_breakdown']
        self.assertAlmostEqual(worst['gap_s'], .088+.005+.030+.016)
        self.assertAlmostEqual(worst['previous_tick_wall_s'], .088)
        self.assertAlmostEqual(worst['planned_idle_s'], .005)
        self.assertAlmostEqual(worst['wake_late_s'], .030)
        self.assertAlmostEqual(worst['lock_wait_s'], .016)
        self.assertAlmostEqual(sum(worst[k] for k in ('previous_tick_wall_s', 'planned_idle_s', 'wake_late_s',
                                                      'lock_wait_s')), worst['gap_s'])
        self.assertAlmostEqual(report['max_wake_late_s'], .030)
        self.assertGreaterEqual(report['max_tick_cpu_s'], 0.)
        self.assertEqual(report['tick_cpu_basis'], 'control_thread_cpu_time')
        self.assertEqual(type(report['control_thread_native_id']), int)


class PlanMarginReuse(unittest.TestCase):
    def test_kept_margins_give_the_same_plan(self):
        import maneuver_sequence as ms
        free = [(t, d) for t, d in zip(ms.OPEN_PROBE_DIRECTIONS_RAD, (1.2, 2.05, math.inf, 1.6, .95))]
        plans = []
        for approach in (None, (-1.4, .3), (-1.3, .28), (-1.4, .3)):
            ms._PLAN_MARGINS.clear()
            cold = ms._plan_loop(free, .1, ms.OPEN_RIGHT_RADIUS_M, approach)
            plans.append(cold)
        ms._PLAN_MARGINS.clear()
        warm = [ms._plan_loop(free, .1, ms.OPEN_RIGHT_RADIUS_M, a)
                for a in (None, (-1.4, .3), (-1.3, .28), (-1.4, .3))]
        self.assertEqual(warm, plans)
        self.assertEqual(len(ms._PLAN_MARGINS), 1)


class SecondConfirmGrace(unittest.TestCase):
    def test_hold_waits_only_for_a_tracked_unconfirmed_support_nearer_than_the_board(self):
        import maneuver_sequence as ms
        motion = ms.ManeuverSequence(0., continue_route=True)
        scan = {'seq': 7}
        motion.second_target = {'confirmed': False, 'track_id': 1, 'source_seq': 7, 'range_m': 2.6}
        motion.role_distinct_evidence = {'source_seq': 7}
        stop = 2.89
        self.assertTrue(motion._open_second_maturing(scan, 2.8, stop))
        self.assertFalse(motion._open_second_maturing(scan, stop-ms.OPEN_SECOND_CONFIRM_GRACE_M, stop))
        self.assertFalse(motion._open_second_maturing(scan, 2.5, stop))  # support behind the board
        for change in ({'confirmed': True}, {'source_seq': 6}, {'track_id': None}):
            motion.second_target = {'confirmed': False, 'track_id': 1, 'source_seq': 7, 'range_m': 2.6, **change}
            self.assertFalse(motion._open_second_maturing(scan, 2.8, stop))
        motion.second_target = {'confirmed': False, 'track_id': 1, 'source_seq': 7, 'range_m': 2.6}
        motion.role_distinct_evidence = {'source_seq': 6}
        self.assertFalse(motion._open_second_maturing(scan, 2.8, stop))
        self.assertFalse(motion._open_second_maturing(scan, None, stop))


class LightEntryPauseHold(unittest.TestCase):
    def test_rolling_turn_toward_the_axis_counter_steers_before_crossing_it(self):
        import maneuver_sequence as ms
        motion = ms.ManeuverSequence(0., continue_route=True)
        motion.compass.current = lambda scan: math.radians(6.6)
        motion.servo, motion.compass.rate = 1432, math.radians(-30.)  # car run 90, 15.15 s
        plain = motion._open_heading_hold(math.radians(6.6), 0.)
        self.assertLess(plain, ms.NEUTRAL)
        self.assertGreaterEqual(motion._light_entry_pause_hold({}), ms.NEUTRAL)
        motion.compass.rate = 0.
        self.assertEqual(motion._light_entry_pause_hold({}), plain)
        motion.route_stage, motion._pause_steer_target, motion.quality_wait_servo = 'light_entry', 1510, 1580
        self.assertEqual(motion._pause_wait_base('post_pass'), 1510)
        motion.compass.current = lambda scan: None
        self.assertIsNone(motion._light_entry_pause_hold({}))


class ActiveReportView(unittest.TestCase):
    def test_live_state_summarizes_heavy_evidence_without_changing_the_report(self):
        import copy, json
        server = control.TurnControlTests('test_watchdog_terminal_gap_result_keeps_measured_lock_and_tick_wall_time')
        server.setUp()
        self.addCleanup(server.temp.cleanup)
        scan = {'seq': 9, 'ranges': [1.]*360}
        events = {'start_lane': {'scan': scan, 'control': {}, 'evidence': {}},
                  'first_pass': {'scan': {**scan, 'seq': 12}, 'control': {}, 'evidence': {}}}
        report = {'route_events': events, 'quality_wait_snapshot': {'scan': scan, 'control': {}},
                  'turn': {'route_events': events, 'avoid_evidence': [{'k': i} for i in range(9)], 'reason': 'x'},
                  'phase': 'drive'}
        original = copy.deepcopy(report)
        view = server.server.active_report_view(report)
        self.assertEqual(report, original)
        self.assertEqual(view['route_events'], {'summarized': True, 'events': {
            'start_lane': {'scan_seq': 9}, 'first_pass': {'scan_seq': 12}}})
        self.assertEqual(view['turn']['route_events'], view['route_events'])
        self.assertEqual(view['quality_wait_snapshot'], {'summarized': True, 'keys': ['control', 'scan'], 'scan_seq': 9})
        self.assertEqual(view['turn']['avoid_evidence'], {'summarized': True, 'count': 9, 'last': [{'k': 7}, {'k': 8}]})
        self.assertEqual((view['phase'], view['turn']['reason']), ('drive', 'x'))
        self.assertLess(len(json.dumps(view)), len(json.dumps(report)))


class ApproachConvergence(unittest.TestCase):
    def test_far_approach_line_is_joined_gently_up_to_half_way(self):
        import maneuver_sequence as ms
        # Clockwise loop, axis straight ahead; tangent point 3 m ahead, .7 m right.
        center, radius = (3., -1.75), 1.05
        point, mode, before = ms._loop_carrot(center, radius, -1, ms.OPEN_LOOKAHEAD_M, (1., 0.), (0., -1.))
        self.assertEqual(mode, 'approach_line')
        self.assertAlmostEqual(before, 3.)
        self.assertAlmostEqual(point[0], min(.5*before, ms.OPEN_APPROACH_CONVERGE_RATIO*.7))
        self.assertAlmostEqual(point[1], -.7)
        near, _, _ = ms._loop_carrot((1.6, -1.75), radius, -1, ms.OPEN_LOOKAHEAD_M, (1., 0.), (0., -1.))
        self.assertAlmostEqual(near[0], ms.OPEN_LOOKAHEAD_M)  # never shorter than the base lookahead


if __name__ == '__main__':
    unittest.main()
