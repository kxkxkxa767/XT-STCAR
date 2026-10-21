#!/usr/bin/env python3
"""coast_motion.CoastMotion against the Rust engine's port (route-engine/src/coast.rs).

  python3 scripts/route-engine-coast-parity.py [--engine PATH] [--seed S] [TRIAL_DIR ...]

Each recorded trial (by default every one from round 64 on) gives its saved
scans in order (feedback.jsonl, then after.json, about 5 Hz) to a fresh
estimator in Python and in the engine: as recorded, with receive clocks from
the publication clock plus jitter, and with random changes (repeated scans,
changed duplicates, reordering, gaps, late or invalid ages and clocks, invalid
ranges, missing returns). Synthetic stationary runs (the at-rest scans again
at 10 Hz with small range noise) reach the stationary evidence. Every result
must match exactly (types and float bits, as in route-engine-parity.py). The
first difference is printed and the run exits with status 3. Offline.
"""
import argparse
import copy
import importlib.util
import json
from pathlib import Path
import random
import sys

ROOT = Path(__file__).resolve().parents[1]
CONSOLE = ROOT/'web'/'vehicle-console'
TRIALS = ROOT/'work'/'vehicle-dynamic-deploy-20261008'
sys.path.insert(0, str(CONSOLE))
import coast_motion  # noqa: E402
import route_engine  # noqa: E402

# One request carries a whole sequence (up to a few hundred fits, about 15 ms
# each on the car), so the console's per-tick answer limit does not apply here.
route_engine.ENGINE_TIMEOUT_S = 60.

spec = importlib.util.spec_from_file_location('route_engine_parity', ROOT/'scripts'/'route-engine-parity.py')
parity = importlib.util.module_from_spec(spec)
spec.loader.exec_module(parity)


def saved_scans(trial):
    scans, seen = [], set()
    rows = [json.loads(line) for line in (trial/'feedback.jsonl').read_text().splitlines()]
    after = trial/'after.json'
    if after.is_file() and after.stat().st_size:  # the car's copy of run 78 left it empty
        rows += json.loads(after.read_text())
    for row in rows:
        scan = (row.get('lidar') or {}).get('scan')
        if isinstance(scan, dict) and type(scan.get('seq')) is int and scan['seq'] not in seen:
            seen.add(scan['seq'])
            scans.append(scan)
    return sorted(scans, key=lambda s: s['seq'])


def recorded_steps(scans, rng, offset=50.):
    return [[scan, rng.uniform(.005, .12), round(scan['at_ms']/1000+offset+rng.uniform(0, .02), 6)] for scan in scans]


def changed_steps(scans, rng):
    steps = recorded_steps(scans, rng)
    out = []
    for step in steps:
        pick = rng.random()
        if pick < .08 and out:
            out.append(copy.deepcopy(out[-1]))  # the same scan again, later
            if type(out[-1][2]) is float:
                out[-1][2] = round(out[-1][2]+.02, 6)
        elif pick < .1 and out:
            again = copy.deepcopy(out[-1])
            ranges = again[0]['ranges']
            if isinstance(ranges, list) and ranges:
                ranges[rng.randrange(len(ranges))] = None if rng.random() < .5 else 3.
            out.append(again)
        elif pick < .12 and out:
            again = copy.deepcopy(out[-1])
            again[0]['at_ms'] += 1
            out.append(again)
        elif pick < .14 and len(out) > 1:
            again = copy.deepcopy(out[-2])  # reordered, received later
            if type(out[-1][2]) is float:
                again[2] = round(out[-1][2]+.03, 6)
            out.append(again)
        elif pick < .17:
            continue  # gap
        step = copy.deepcopy(step)
        scan = step[0]
        if rng.random() < .03:
            step[1] = rng.choice([.3, .31, -.01, float('nan'), True])
        if rng.random() < .02:
            last = out[-1][2] if out and type(out[-1][2]) is float else 1.
            step[2] = rng.choice([last-.01, -1., float('inf'), 'x'])
        if rng.random() < .02:
            scan['ranges'][rng.randrange(360)] = rng.choice([.01, 13., float('inf'), True])
        if rng.random() < .05:
            for _ in range(rng.choice([10, 30, 45])):
                scan['ranges'][rng.randrange(360)] = None
        if rng.random() < .01:
            scan['frame_id'] = 'lidar_raw'
        if rng.random() < .01:
            scan['seq'] = rng.choice([-1, 'x', True])
        if rng.random() < .01:
            scan['ranges'] = scan['ranges'][:300]
        out.append(step)
    return out


def stationary_steps(scan, rng, count=14):
    steps, base = [], copy.deepcopy(scan)
    for k in range(count):
        s = copy.deepcopy(base)
        s['seq'], s['at_ms'] = base['seq']+k, base['at_ms']+100*k
        s['ranges'] = [None if r is None else round(r+rng.gauss(0, .002), 4) for r in s['ranges']]
        steps.append([s, rng.uniform(.005, .05), round(s['at_ms']/1000+20+rng.uniform(0, .01), 6)])
    return steps


def rotating_steps(scan, rng, step_bins=5, dt_ms=50, count=8):
    """The scene turning by step_bins degrees per dt_ms (motion limits)."""
    steps = []
    for k in range(count):
        s = copy.deepcopy(scan)
        shift = k*step_bins
        s['ranges'] = scan['ranges'][-shift % 360:]+scan['ranges'][:-shift % 360] if shift else list(scan['ranges'])
        s['seq'], s['at_ms'] = scan['seq']+k, scan['at_ms']+dt_ms*k
        steps.append([s, .02, round(s['at_ms']/1000+30, 6)])
    return steps


def drifting_steps(scan, rng, count=12, receive_step=.13):
    """At rest, but the receive clock advances faster than publication."""
    steps = stationary_steps(scan, rng, count)
    for k, step in enumerate(steps):
        step[2] = round(steps[0][2]+receive_step*k, 6)
        step[1] = .02
    return steps


def python_sequence(steps):
    motion, out = coast_motion.CoastMotion(), []
    for scan, age, now in steps:
        out.append(motion.update(scan, age, now))
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--engine', default=str(parity.DEFAULT_ENGINE))
    parser.add_argument('--seed', type=int, default=97)
    parser.add_argument('trials', nargs='*')
    options = parser.parse_args()
    trials = [Path(t) for t in options.trials] or sorted(
        (p for p in TRIALS.glob('left-cone-trial-*-*') if p.is_dir() and p.name.rsplit('-', 1)[1].isdigit()
         and int(p.name.rsplit('-', 1)[1]) >= 64 and (p/'feedback.jsonl').is_file()),
        key=lambda p: int(p.name.rsplit('-', 1)[1]))
    rng = random.Random(options.seed)
    engine = route_engine.RouteEngine(options.engine)
    generation, _ = engine.open_session()
    results, reasons = 0, {}
    try:
        for trial in trials:
            scans = saved_scans(trial)
            runs = [('recorded', recorded_steps(scans, rng))]
            runs += [(f'changed-{k}', changed_steps(scans, rng)) for k in range(3)]
            runs += [(f'stationary-{k}', stationary_steps(rng.choice(scans[-6:]), rng)) for k in range(3)]
            rest = rng.choice(scans[-6:])
            runs += [('rotating-fast', rotating_steps(rest, rng)), ('rotating-slow', rotating_steps(rest, rng, 1, 100)),
                     ('drifting-clock', drifting_steps(rest, rng))]
            for name, steps in runs:
                python = parity.normalize(python_sequence(steps))
                response = engine.request(generation, {'op': 'fn', 'name': 'coast_motion', 'args': [steps]})
                if not response.get('ok'):
                    print(f'\nCOAST ENGINE ERROR ({trial.name} {name}): {response}', file=sys.stderr)
                    sys.exit(3)
                found = parity.difference(python, response['r'], '$')
                if found:
                    path, py, rust = found
                    print(f'\nCOAST MISMATCH ({trial.name} {name}) at {path}\n  python: {repr(py)[:600]}\n'
                          f'  rust:   {repr(rust)[:600]}', file=sys.stderr)
                    sys.exit(3)
                results += len(python)
                for result in python:
                    reasons[result['reason']] = reasons.get(result['reason'], 0)+1
            print(f'{trial.name}: {len(scans)} scans', flush=True)
    finally:
        engine.close()
    print(f'coast parity: {results} results matched; reasons {dict(sorted(reasons.items(), key=lambda kv: -kv[1]))}')


if __name__ == '__main__':
    main()
