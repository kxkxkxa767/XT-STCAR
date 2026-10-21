"""Round 97 (part B): the standalone control program `route-engine race`.

Runs the Mac build against scripts/sim-vehicle-bridge.py (course-sim's
measured vehicle and course in real time, the bridge's own teleop guard) in
demo mode: argument refusals, a start that needs a terminal, a closed-loop
start -> drive -> cumulative budget halt with the trial helper's evidence
files, a lidar bridge that ends (bridge_failure), and an interrupted program
whose bridges hold neutral themselves. The decisions are checked exactly by
scripts/route-engine-supervisor-parity.py; these tests cover the program's
own I/O. Real time (about 40 s in all); skipped without the builds.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[3]
ENGINE = ROOT/'route-engine'/'target'/'release'/'route-engine'
GEOMETRY = ROOT/'tools'/'scan-geometry'/'target'/'release'/'xt-stcar-scan-geometry'
BRIDGE = ROOT/'scripts'/'sim-vehicle-bridge.py'


@unittest.skipUnless(ENGINE.is_file() and GEOMETRY.is_file(), 'route-engine or tools/scan-geometry not built')
class RaceProgramTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.dir = Path(self.temp.name)
        self.shared = self.dir/'shared'
        self.shared.mkdir()
        self.out = self.dir/'evidence'

    def tearDown(self):
        self.temp.cleanup()

    def command(self, *extra):
        return [str(ENGINE), 'race', '--bridge', str(BRIDGE), '--demo', '--out', str(self.out),
                '--status-s', '5', *extra]

    def env(self, **extra):
        return {**os.environ, 'SIM_SHARED': str(self.shared), 'SIM_LAYOUT': 'user8', **extra}

    def run_program(self, *extra, timeout=60, **env):
        return subprocess.run(self.command(*extra), env=self.env(**env), capture_output=True, text=True,
                              timeout=timeout, stdin=subprocess.DEVNULL)

    def test_refusals_before_any_bridge(self):
        self.out.mkdir()
        p = self.run_program('--start', 'ready')
        self.assertEqual(p.returncode, 64)
        self.assertIn('already exists', p.stderr)
        self.out.rmdir()
        for extra in (('--max-drive-s', '41'), ('--max-drive-s', '0.5'), ('--start', 'now'), ('--frobnicate',)):
            with self.subTest(extra=extra):
                p = self.run_program(*extra)
                self.assertEqual(p.returncode, 64, p.stderr)
                self.assertFalse(self.out.exists())
        self.assertFalse((self.shared/'command.json').exists())  # no bridge was started

    def test_enter_start_needs_a_terminal(self):
        p = self.run_program('--start', 'enter')
        self.assertEqual(p.returncode, 64, p.stdout+p.stderr)
        self.assertIn('terminal input closed', p.stdout)
        self.assertFalse((self.out/'feedback.jsonl').exists())  # never started
        final = json.loads((self.shared/'control-final.json').read_text())
        self.assertEqual((final['armed'], final['motor'], final['servo'], final['reason']),
                         (False, 1500, 1500, 'input_closed'))

    def test_closed_loop_start_drive_and_budget_halt(self):
        p = self.run_program('--start', 'ready', '--ready-hold-s', '.3', '--max-drive-s', '3')
        self.assertEqual(p.returncode, 0, p.stdout+p.stderr)
        for name in ('before.json', 'before-lidar.json', 'feedback.jsonl', 'after.json', 'summary.json',
                     'cli-result.json'):
            self.assertTrue((self.out/name).is_file(), name)
        summary = json.loads((self.out/'summary.json').read_text())
        result = summary['result']
        self.assertEqual(summary['reason'], 'maneuver_cumulative_drive_timeout')
        self.assertEqual(result['route_controller'], 'rust_program')
        self.assertTrue(result['observed_armed'])
        self.assertGreater(result['drive_ticks'], 50)
        self.assertLess(summary['max_actual_gap_s'], .12)
        rows = [json.loads(line) for line in (self.out/'feedback.jsonl').read_text().splitlines()]
        self.assertGreater(len(rows), 10)
        # The fields trial-timeline/trial-map read.
        for row in rows:
            self.assertIn('control', row['state']['status'])
            self.assertIsInstance(row['lidar']['scan']['ranges'], list)
            self.assertIsInstance(row['elapsed_s'], float)
        self.assertTrue(any((r['state']['autonomy']['active'] or {}).get('phase') == 'drive' for r in rows))
        before = json.loads((self.out/'before.json').read_text())
        self.assertTrue(before['autonomy']['turn_ready'])
        after = json.loads((self.out/'after.json').read_text())
        self.assertEqual(len(after), 6)
        self.assertFalse(after[-1]['state']['status']['control']['armed'])
        final = json.loads((self.shared/'control-final.json').read_text())
        self.assertEqual((final['armed'], final['motor']), (False, 1500))

    def test_lidar_bridge_end_halts_the_run(self):
        p = self.run_program('--start', 'ready', '--ready-hold-s', '.3', SIM_LIDAR_END_S='1')
        self.assertEqual(p.returncode, 0, p.stdout+p.stderr)
        summary = json.loads((self.out/'summary.json').read_text())
        self.assertEqual(summary['reason'], 'bridge_failure')
        state = json.loads((self.out/'after.json').read_text())[-1]['state']
        self.assertEqual(state['errors'].get('lidar'), 'lidar bridge ended')
        self.assertFalse(state['status']['control']['armed'])

    def test_interrupted_program_leaves_the_bridges_at_neutral(self):
        proc = subprocess.Popen(self.command('--start', 'ready', '--ready-hold-s', '.3'), env=self.env(),
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, text=True)
        try:
            deadline = time.monotonic()+30
            while not (self.shared/'driving').exists() and time.monotonic() < deadline:
                time.sleep(.05)
            self.assertTrue((self.shared/'driving').exists(), 'the run never drove')
            time.sleep(.5)
            proc.send_signal(signal.SIGINT)  # the program only: the bridges have their own process group
            proc.wait(timeout=5)
        finally:
            if proc.poll() is None:
                proc.kill()
                proc.wait()
            proc.stdout.close()
        deadline = time.monotonic()+5
        while not (self.shared/'control-final.json').exists() and time.monotonic() < deadline:
            time.sleep(.05)
        final = json.loads((self.shared/'control-final.json').read_text())
        self.assertEqual((final['armed'], final['motor'], final['servo'], final['reason']),
                         (False, 1500, 1500, 'input_closed'))
        # Neither bridge outlives the program (no orphan holding a device).
        for mode in ('control', 'lidar'):
            pid = int((self.shared/f'pid-{mode}').read_text())
            deadline = time.monotonic()+5
            while time.monotonic() < deadline:
                try:
                    os.kill(pid, 0)
                except ProcessLookupError:
                    break
                time.sleep(.05)
            else:
                self.fail(f'{mode} bridge {pid} still running')


if __name__ == '__main__':
    unittest.main()
