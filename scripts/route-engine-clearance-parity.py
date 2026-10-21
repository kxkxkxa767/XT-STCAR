#!/usr/bin/env python3
"""autonomy_live.probe_clearance against the Rust engine's port, call by call.

  python3 scripts/route-engine-clearance-parity.py [--engine PATH] [--per-scan N] [--seed S] [TRIAL_DIR ...]

Real car scans (by default every recorded trial from round 64 on) with random
argument combinations and synthetic changes: missing and invalid returns,
returns around the body (stop, boundary, cone-like witness, start-end wall
behind the car), front null runs (bridged, lane and side-board projection),
late/invalid sensor ages, invalid arguments and witnesses. Each result or
error (type, message, a body stop's attached clearance) must match exactly:
types and float bits, as in route-engine-parity.py. A second pass sends the
scans in order through ClearanceReuse and the engine's clearance_probe (one
session; repeated frames, ages across the limits, changed arguments) and
compares results and reuse counts. The first difference is printed and the
run exits with status 3. Offline; never contacts the car.
"""
import argparse
import copy
import importlib.util
import json
import math
from pathlib import Path
import random
import sys

ROOT = Path(__file__).resolve().parents[1]
CONSOLE = ROOT/'web'/'vehicle-console'
TRIALS = ROOT/'work'/'vehicle-dynamic-deploy-20261008'
sys.path.insert(0, str(CONSOLE))
import autonomy_live as al  # noqa: E402
import compact_target as ct  # noqa: E402
import route_engine  # noqa: E402

spec = importlib.util.spec_from_file_location('route_engine_parity', ROOT/'scripts'/'route-engine-parity.py')
parity = importlib.util.module_from_spec(spec)
spec.loader.exec_module(parity)


def trial_scans(trial):
    scans, seen = [], set()
    for line in (trial/'feedback.jsonl').read_text().splitlines():
        scan = (json.loads(line).get('lidar') or {}).get('scan')
        if isinstance(scan, dict) and scan.get('seq') not in seen:
            seen.add(scan.get('seq'))
            scans.append(scan)
    return sorted(scans, key=lambda s: s['seq'])


def cone(ranges, rng):
    """A compact convex support (radius .08-.12 m) .25-1.5 m away."""
    radius, distance, bearing = rng.uniform(.08, .12), rng.uniform(.25, 1.5), rng.randrange(360)
    for k in range(-25, 26):
        theta = math.radians(k)
        inner = radius**2-(distance*math.sin(theta))**2
        if inner > 0:
            r = distance*math.cos(theta)-math.sqrt(inner)
            if .02 <= r <= 12:
                ranges[(bearing+k) % 360] = r


def body_returns(ranges, rng):
    """Returns near each face, some exactly at the stop distance."""
    for _ in range(rng.randint(1, 4)):
        angle = rng.choice([0, 1, 2, 358, 359, 90, 270, 89, 91, 269, 271, 180, 179, 181, 45, 135, 225, 315,
                            rng.randrange(360)])
        face = (al.FRONT_BODY_EXTENT_M if angle <= 45 or angle >= 315 else al.REAR_BODY_EXTENT_M
                if 135 <= angle <= 225 else al.SIDE_BODY_EXTENT_M)
        gap = rng.choice([0., .01, .039, .04, .041, .07, .079, .08, .081, .1, .2, rng.uniform(0, .3)])
        ranges[angle] = max(.02, min(12., face+gap))


def mutate(scan, rng):
    s = copy.deepcopy(scan)
    s['received_at'] = round(1000+rng.random(), 6)
    ranges = s['ranges']
    if rng.random() < .5:  # a front null run
        start, length = rng.randint(-30, 30), rng.randint(1, 9)
        for angle in range(start, start+length):
            ranges[angle % 360] = None
    if rng.random() < .25:  # scattered nulls (incomplete scans)
        for _ in range(rng.choice([5, 18, 25, 36, 50])):
            ranges[rng.randrange(360)] = None
    if rng.random() < .35:
        body_returns(ranges, rng)
    if rng.random() < .35:
        cone(ranges, rng)
    if rng.random() < .1:  # integer ranges keep their type in the result
        for _ in range(5):
            i = rng.randrange(360)
            if ranges[i] is not None:
                ranges[i] = max(1, min(12, round(ranges[i])))
    if rng.random() < .04:
        ranges[rng.randrange(360)] = rng.choice([.01, 12.5, float('inf'), float('nan'), True, 'x', -1])
    if rng.random() < .02:
        s['frame_id'] = 'lidar_raw'
    if rng.random() < .01:
        s['seq'] = rng.choice([True, '7', 7.0])
    if rng.random() < .01:
        s['ranges'] = ranges[:359]
    return s


def ages(rng):
    pick = rng.random()
    if pick < .8:
        return {'camera': rng.uniform(0, .4), 'lidar': rng.uniform(0, .25), 'control': rng.uniform(0, .15)}
    choice = rng.choice([
        {'camera': .1, 'lidar': .3, 'control': .01}, {'camera': .1, 'lidar': .2999, 'control': .01},
        {'camera': .5, 'lidar': .02, 'control': .01}, {'camera': 1, 'lidar': .02, 'control': .01},
        {'camera': .1, 'lidar': .02, 'control': .2}, {'camera': .1, 'lidar': .02, 'control': .1999},
        {'camera': .1, 'lidar': .45, 'control': .05}, {'camera': 0, 'lidar': 0, 'control': 0},
        {'camera': .1, 'lidar': -.01, 'control': .01}, {'camera': .1, 'lidar': True, 'control': .01},
        {'lidar': .02, 'control': .01}, {'camera': float('nan'), 'lidar': .02, 'control': .01},
        {'camera': .1, 'lidar': float('inf'), 'control': .01}])
    return dict(choice)


def witness(scan, rng):
    """Contact targets as current_contact_target builds them, and broken ones."""
    ranges = scan.get('ranges')
    try:
        supports = ct.contact_supports(ranges) if isinstance(ranges, list) and len(ranges) == 360 else []
    except (ValueError, TypeError, KeyError):
        supports = []
    if not supports:
        return None if rng.random() < .95 else {'role': 'cone_like_support'}
    bins = sorted({i for support in supports for i in support['support_bins']})
    target = {'role': 'cone_like_support', 'track_id': None, 'confirmed': False, 'full_observation': True,
              'source_seq': scan['seq'], 'source_at_ms': scan.get('at_ms'), 'source_received_at': scan['received_at'],
              'support_bins': bins, 'support_ranges_m': [ranges[i] for i in bins], 'semantic_class': 'unknown',
              'objects': [{'support_bins': list(s['support_bins']), 'range_m': s['range_m']} for s in supports],
              'orbit_track_id': 3, 'second_track_id': None}
    if rng.random() < .2:  # a confirmed role binds one support only
        one = list(supports[0]['support_bins'])
        target.update(role=rng.choice(['first_target', 'second_target']), track_id=rng.choice([1, 4]),
                      confirmed=True, support_bins=one, support_ranges_m=[ranges[i] for i in one])
    if rng.random() < .1:
        change = rng.randrange(9)
        if change == 0:
            target['source_seq'] = target['source_seq']+1 if type(target['source_seq']) is int else 1
        elif change == 1 and len(target['support_bins']) > 4:
            target['support_bins'] = target['support_bins'][1:]
            target['support_ranges_m'] = target['support_ranges_m'][1:]
        elif change == 2:
            target['support_ranges_m'] = [r+.001 for r in target['support_ranges_m']]
        elif change == 3:
            target['confirmed'] = None
        elif change == 4:
            target['source_received_at'] = target['source_received_at']+1e-6
        elif change == 5:
            target['support_bins'] = target['support_bins']+target['support_bins'][:1]
            target['support_ranges_m'] = target['support_ranges_m']+target['support_ranges_m'][:1]
        elif change == 6:
            target['semantic_class'] = 'blue_cone'
        elif change == 7:
            target['track_id'] = 0
        else:
            target['source_received_at'] = int(target['source_received_at'])
    return target


def arguments(scan, rng):
    profile = rng.choices(['maneuver', 'straight', 'bogus', None], [80, 17, 2, 1])[0]
    maneuver = profile == 'maneuver'
    kw = {'rear_launch': rng.random() < .3, 'camera_required': rng.choices([True, False, 1], [30, 67, 3])[0],
          'clearance_profile': profile,
          'relaxed_turn_left': rng.random() < .3 and (maneuver or rng.random() < .05),
          'rear_start': rng.random() < .3 and (maneuver or rng.random() < .05),
          'front_side_walls': False, 'front_lane': None}
    if rng.random() < .03:
        kw['relaxed_turn_left'] = 1
    if rng.random() < .03:
        kw['rear_start'] = 'yes'
    candidates = scan.get('corridor_candidates') if isinstance(scan.get('corridor_candidates'), list) else []
    pick = rng.random()
    if pick < .3 and candidates:
        kw['front_lane'] = copy.deepcopy(rng.choice(candidates))  # equal content, as the server passes it
    elif pick < .33 and candidates:
        lane = copy.deepcopy(rng.choice(candidates))
        lane['heading_left_rad'] = rng.choice([lane['heading_left_rad']+1e-9, 2.0])
        if rng.random() < .5:
            scan['corridor_candidates'] = candidates+[lane]
        kw['front_lane'] = lane
    elif pick < .6:
        kw['front_side_walls'] = True
    elif pick < .62:
        kw['front_side_walls'] = rng.choice([1, None])
    if rng.random() < .01 and kw['front_lane'] is not None:
        scan['corridor_candidates'] = None
    kw['body_contact_target'] = witness(scan, rng) if maneuver or rng.random() < .05 else None
    return kw


FEATURES = {
    'lane_projection_hits': lambda c: bool((c.get('front_wall_projection') or {}).get('hits'))
    and 'lane' in c['front_wall_projection'],
    'side_board_projection_hits': lambda c: bool((c.get('front_wall_projection') or {}).get('hits'))
    and 'fits' in c['front_wall_projection'],
    'bridged': lambda c: bool(c.get('front_bridged_bins')),
    'front_sparse': lambda c: 'front_sparse' in c['quality_issues'],
    'front_sparse_tolerable': lambda c: c['front_sparse_tolerable'],
    'scan_incomplete_tolerable': lambda c: c['scan_incomplete_tolerable'],
    'cone_contact_tolerated': lambda c: c['body_proximity']['target_contact_tolerated'],
    'rear_start_exempt': lambda c: bool(c['body_proximity']['rear_start_exempt_points']),
    'relaxed_nearest': lambda c: (c['body_proximity']['nearest_point'] or {}).get('relaxed_turn_left') is True,
    'int_range_nearest': lambda c: type(c['nearest_m']) is int,
}


def features(outcome, counts):
    kind, value = outcome
    clearance = value if kind == 'ok' else value[2] if value[0] == 'ProbeClearanceError' else None
    for name, test in FEATURES.items():
        if clearance is not None and test(clearance):
            counts[name] = counts.get(name, 0)+1


def python_outcome(call):
    try:
        return 'ok', parity.normalize(call())
    except al.ProbeClearanceError as error:
        return 'error', ['ProbeClearanceError', str(error), parity.normalize(error.clearance)]
    except (ValueError, TypeError, KeyError) as error:
        return 'error', [type(error).__name__, '' if isinstance(error, TypeError) else str(error), None]


def engine_function(engine, generation, args):
    response = engine.request(generation, {'op': 'fn', 'name': 'probe_clearance', 'args': args})
    if not response.get('ok'):
        exc = response.get('exc')
        return 'error', [exc, '' if exc == 'TypeError' else response.get('msg'), None]
    result = response['r']
    if 'body_stop' in result:
        return 'error', ['ProbeClearanceError', 'probe_obstacle_close_body', result['body_stop']]
    return 'ok', result['clearance']


def report(where, found, args):
    path, py, rust = found
    print(f'\nCLEARANCE MISMATCH ({where}) at {path}\n  python: {repr(py)[:600]}\n  rust:   {repr(rust)[:600]}',
          file=sys.stderr)
    out = ROOT/'work'/'clearance-parity-mismatch.json'
    out.write_text(json.dumps(args, default=repr))
    print(f'  arguments: {out}', file=sys.stderr)
    sys.exit(3)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--engine', default=str(parity.DEFAULT_ENGINE))
    parser.add_argument('--per-scan', type=int, default=6)
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
    calls, outcomes, seen = 0, {}, {}
    try:
        for trial in trials:
            scans = trial_scans(trial)
            # Pass 1: single calls, random arguments and changes.
            for scan in scans:
                for _ in range(options.per_scan):
                    s = mutate(scan, rng)
                    a, kw = ages(rng), arguments(s, rng)
                    demo, centering = rng.random() < .1, rng.random() < .3
                    args = [s, a, demo, centering, kw]
                    py = python_outcome(lambda: al.probe_clearance(s, a, demo, centering, **kw))
                    rust = engine_function(engine, generation, args)
                    calls += 1
                    key = py[0] if py[0] == 'ok' else py[1][1] or py[1][0]
                    outcomes[key] = outcomes.get(key, 0)+1
                    features(py, seen)
                    found = parity.difference(py[0], rust[0], '$.outcome') or parity.difference(py[1], rust[1], '$')
                    if found:
                        report(f'{trial.name} seq {scan.get("seq")}', found, args)
            # Pass 2: the frames in order through ClearanceReuse and the session.
            motion = route_engine.RustManeuverSequence(engine, 0., continue_route=True)
            reuse = al.ClearanceReuse()
            kw = {'rear_launch': False, 'camera_required': False, 'clearance_profile': 'maneuver',
                  'relaxed_turn_left': False, 'front_lane': None, 'front_side_walls': True, 'rear_start': False}
            for scan in scans:
                s = {**scan, 'received_at': round(1000+scan['seq']*.1, 6)}
                for _ in range(rng.choice([1, 2, 3, 3, 4])):
                    a = {'camera': .1, 'lidar': rng.choice([.02, .1, .29, .3, .31]), 'control': rng.choice([.01, .19, .2])}
                    if rng.random() < .15:
                        kw = {**kw, 'front_side_walls': not kw['front_side_walls'],
                              'relaxed_turn_left': rng.random() < .3}
                    if rng.random() < .05:
                        s = {**s, 'ranges': list(s['ranges'])}  # equal content, another list
                    py = python_outcome(lambda: (lambda c: [c, reuse.reused])(
                        reuse.probe(al.probe_clearance, s, a, False, False, body_contact_target=None, **kw)))
                    rust = parity.outcome(lambda: motion.clearance_probe(s, 0., False, a, False, False, **kw))
                    if rust[0] == 'ok':
                        rust = 'ok', parity.normalize([rust[1][1], rust[1][2]])
                    else:
                        error = rust[1]
                        rust = 'error', [type(error).__name__, '' if isinstance(error, TypeError) else str(error),
                                         parity.normalize(getattr(error, 'clearance', None))]
                    calls += 1
                    found = parity.difference(py[0], rust[0], '$.outcome') or parity.difference(py[1], rust[1], '$')
                    if found:
                        report(f'{trial.name} reuse seq {scan.get("seq")}', found, [s, a, kw])
            print(f'{trial.name}: {len(scans)} scans, reused {reuse.reused}', flush=True)
    finally:
        engine.close()
    print(f'clearance parity: {calls} calls matched; outcomes {dict(sorted(outcomes.items(), key=lambda kv: -kv[1]))};'
          f' paths {seen}')


if __name__ == '__main__':
    main()
