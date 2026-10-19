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


if __name__ == '__main__':
    unittest.main()
