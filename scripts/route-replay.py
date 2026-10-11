#!/usr/bin/env python3
"""Open-loop decision replay of a recorded trial through the current controller.

Usage (from the repository root):
    python3 -I scripts/route-replay.py work/vehicle-dynamic-deploy-20261008/left-cone-trial-20261011-52
    python3 -I scripts/route-replay.py <trial-dir> --from 8.5 --to 10.3

Feeds the trial's saved scans (with the car's own native geometry fields) at
their recorded timing into server.Console + ManeuverSequence exactly as in the
working tree (unit-test harness, instant ACKs) and prints, per scan, what the
CURRENT code decides: phase, route stage, reason, commanded motor/servo, first
and second target IDs, path-check overrides. The session starts so that its
drive begins when the car's recorded drive began.

OPEN LOOP: the scans are what the car saw while running the OLD code. Decisions
are meaningful up to the moment they differ from what the car did; after that
the real scans no longer follow the new commands. Use it to check perception
and stage logic on real data at the moments that failed (e.g. does the second
target get acquired and does the steering go left before the trial-52 contact).
The saved scans are about every other one (~5 Hz).
"""
import argparse
import importlib.util
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
TESTS = ROOT/'web'/'vehicle-console'/'tests'
HARNESS_PRESTEER_S = 2.1  # instant-ACK presteer in the harness (2.08 s measured)


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def recorded(trial):
    scans, drive_at, seen = [], None, set()
    for line in (Path(trial)/'feedback.jsonl').read_text().splitlines():
        row = json.loads(line)
        scan = (row.get('lidar') or {}).get('scan')
        control = ((row.get('state') or {}).get('status') or {}).get('control') or {}
        if isinstance(scan, dict) and scan['seq'] not in seen:
            seen.add(scan['seq'])
            scans.append(scan)
        if drive_at is None and control.get('motor', 1500) > 1500 and isinstance(scan, dict):
            drive_at = scan['at_ms']
    scans.sort(key=lambda s: s['seq'])
    return scans, drive_at


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('trial_dir')
    parser.add_argument('--from', dest='start', type=float, default=0.)
    parser.add_argument('--to', dest='end', type=float, default=99.)
    parser.add_argument('--max-drive-s', type=float, default=16.)
    args = parser.parse_args()
    scans, drive_at = recorded(args.trial_dir)
    if drive_at is None:
        sys.exit('no recorded drive in this trial')
    static = [s for s in scans if s['at_ms'] < drive_at]
    moving = [s for s in scans if s['at_ms'] >= drive_at]
    if not static:
        sys.exit('no recorded scan before the drive')
    sys.path.insert(0, str(TESTS))
    mod = load_module('replay_harness', TESTS/'test_turn_control.py')
    h = mod.TurnControlTests('test_compact_trial_preview_default_1670_and_placement_admission_keep_neutral')
    h.setUp()
    seq = [static[-1]['seq']]

    def feed(scan, t):
        seq[0] += 1
        value = {**scan, 'seq': seq[0], 'at_ms': round(t*1000), 'frame_id': 'lidar_origin_coarse_body_heading'}
        h.observe(t, value)

    h.now = h.console.scan_at = h.console.control_at = .1
    h.console.scan = {**static[-1], 'seq': seq[0], 'at_ms': 100, 'frame_id': 'lidar_origin_coarse_body_heading'}
    data = {'op': 'turn_cone_start', 'boot': h.console.boot, 'epoch': h.console.control_epoch,
            'tick': h.console.status['control']['tick'], 'placement_confirmed': True,
            'continue_route': True, 'max_drive_s': args.max_drive_s}
    with mod.patch.object(h.server.time, 'monotonic', return_value=h.now):
        started = h.console.autonomy_command(data)
    print('start', {k: started.get(k) for k in ('ok', 'error')} if isinstance(started, dict) else started)
    # Phase 1: the car was stationary; cycle its recorded stationary scans
    # until the replayed controller enters drive (presteer adoption).
    t, k = .1, 0
    while t < 8.:
        t = round(t+.1, 3)
        feed(static[k % len(static)], t)
        k += 1
        motion = (h.console.auto_session or {}).get('turn_motion')
        if motion is None or motion.phase != 'presteer':
            break
    drive_t = t
    print(f'replayed drive starts at {drive_t:.2f} s; recorded drive scans follow with their own timing')
    print('  t(s) rec  seq  phase        stage           M    S    1st      2nd      reason')
    for scan in moving:
        rec_t = (scan['at_ms']-drive_at)/1000
        t = round(drive_t+rec_t+.1, 3)
        if t <= h.now:
            continue
        feed(scan, t)
        session = h.console.auto_session
        motion = (session or {}).get('turn_motion')
        control = h.console.status['control']
        if motion is None:
            result = h.console.auto_result or {}
            print(f"{t:6.2f} {rec_t:5.2f} {scan['seq']:5d} ENDED {result.get('reason')}")
            break
        if not args.start <= t <= args.end:
            continue
        first_t = motion.compact_target or {}
        second_t = motion.second_target or {}
        fmt = lambda target: (f"{target.get('track_id')}/{target.get('confirmation_count')}"  # noqa: E731
                              f"{'c' if target.get('confirmed') else ''}" if target else '-')
        override = ''
        if motion.path_evidence and motion.path_evidence.get('source_seq') == scan['seq']:
            e = motion.path_evidence
            override = f" | PATH {e['stage_target']}->{e['chosen']} free {e['stage_free_m']:.2f}"
        cause = ''
        if motion.phase == 'quality_wait' and isinstance(motion.recovery_evidence, dict):
            cause = f" | cause {motion.recovery_evidence.get('cause')}"
        print(f"{t:6.2f} {rec_t:5.2f} {scan['seq']:5d} {motion.phase[:12]:12s} {motion.route_stage[:15]:15s} "
              f"{control.get('motor', 1500) if control.get('armed') else 1500:4d} {control.get('servo'):4d} "
              f"{fmt(first_t):8s} {fmt(second_t):8s} {motion.reason}{override}{cause}")
    h.tearDown()


if __name__ == '__main__':
    main()
