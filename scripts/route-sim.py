#!/usr/bin/env python3
"""EXPERIMENTAL closed-loop route simulation through the real console/route code.

Usage (from the repository root; builds the host geometry tool once):
    cargo build --release -q --manifest-path tools/scan-geometry/Cargo.toml
    python3 -I scripts/route-sim.py work/vehicle-dynamic-deploy-20261008/left-cone-trial-20261011-52 \
        --seeds 5 --png /tmp/sim.png

World: the course rebuilt from a recorded trial's scans (scripts/trial-map.py ICP;
start pose = that trial's start). Sensor: z-buffered ray casting of the map with
range noise, the recurring 5-17 deg right dropout band and edge nulls at depth
steps (measured on the car, trials 50-52). Geometry: the same Rust junction code
as the vehicle bridge (tools/scan-geometry; matches the
car's recorded fields exactly). Car: lidar-path kinematics with speeds/decel and
curvature-per-PWM measured from rebuilt paths (1560 ~1.4 m/s, 1550 ~.9 m/s,
neutral ~.5 m/s^2, kappa ~ (servo-1500)/160 1/m, ~.12 s steering lag).
Control: server.Console + ManeuverSequence exactly as deployed, through the
unit-test harness (instant ACKs).

It is a regression check, not a certificate: the map only covers what the
recorded trial saw (usually not past the second target), the dynamics are rough
and ACK timing is idealized. A change that fails here will fail on the car; a
pass only means the car test is worth running.
"""
import argparse
import importlib.util
import json
import math
from pathlib import Path
import random
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
TESTS = ROOT/'web'/'vehicle-console'/'tests'
GEOMETRY = ROOT/'tools'/'scan-geometry'/'target'/'release'/'xt-stcar-scan-geometry'
FRONT, REAR, SIDE = .20, .18, .14
SPEED = {1560: 1.4, 1550: .9}
ACCEL, COAST_DECEL, PWM_PER_KAPPA, STEER_TAU = 1.5, .5, 160., .12


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def world_map(trial, min_hits=4):
    """3 cm voxels seen in >= min_hits registered frames (drops ghost walls from
    imperfect registration during fast turns: trial52 median sim-vs-real range
    error 10 cm with every voxel, ~3 cm at 4 hits)."""
    tm = load_module('trial_map', ROOT/'scripts'/'trial-map.py')
    frames = tm.load_frames(Path(trial))
    poses = tm.track(frames)
    hits = {}
    for est in poses:
        if not est['good']:
            continue
        seen = set()
        for p in est['points']:
            w = tm.to_world(est['pose'], p)
            key = (round(w[0]/.03), round(w[1]/.03))
            if key in seen:
                continue
            seen.add(key)
            hits.setdefault(key, [0, w])[0] += 1
    return [w for count, w in hits.values() if count >= min_hits]


def extract_shapes(points, rng):
    """Boards as line segments (RANSAC, 3 cm inliers, split at 15 cm gaps) and the
    remaining compact clusters (<= .35 m) as cones (circle, ~.08 m radius)."""
    remaining = list(points)
    segments = []
    for _ in range(60):
        if len(remaining) < 12:
            break
        best = None
        for _ in range(250):
            a, b = rng.sample(remaining, 2)
            length = math.dist(a, b)
            if length < .15:
                continue
            ux, uy = (b[0]-a[0])/length, (b[1]-a[1])/length
            inliers = [p for p in remaining if abs((p[0]-a[0])*uy-(p[1]-a[1])*ux) < .03]
            if best is None or len(inliers) > len(best[2]):
                best = (a, (ux, uy), inliers)
        if best is None or len(best[2]) < 12:
            break
        a, (ux, uy), inliers = best
        # refit by least squares (principal axis) and split along the line
        mx = sum(p[0] for p in inliers)/len(inliers)
        my = sum(p[1] for p in inliers)/len(inliers)
        sxx = sum((p[0]-mx)**2 for p in inliers)
        syy = sum((p[1]-my)**2 for p in inliers)
        sxy = sum((p[0]-mx)*(p[1]-my) for p in inliers)
        angle = .5*math.atan2(2*sxy, sxx-syy)
        ux, uy = math.cos(angle), math.sin(angle)
        proj = sorted(((p[0]-mx)*ux+(p[1]-my)*uy, p) for p in inliers)
        used = set()
        run = [proj[0]]
        for item in proj[1:]+[None]:
            if item is not None and item[0]-run[-1][0] <= .15:
                run.append(item)
                continue
            if len(run) >= 8 and run[-1][0]-run[0][0] >= .2:
                t0, t1 = run[0][0], run[-1][0]
                segments.append(((mx+ux*t0, my+uy*t0), (mx+ux*t1, my+uy*t1)))
                used.update(id(q) for _, q in run)
            if item is not None:
                run = [item]
        if not used:
            break
        remaining = [p for p in remaining if id(p) not in used]
    cones = []
    while remaining:
        seed = remaining.pop()
        cluster, frontier = [seed], [seed]
        while frontier:
            q = frontier.pop()
            near = [p for p in remaining if math.dist(p, q) < .08]
            for p in near:
                remaining.remove(p)
            cluster.extend(near)
            frontier.extend(near)
        if len(cluster) >= 6:
            span = max(math.dist(a, b) for a in cluster for b in cluster)
            if .05 <= span <= .35:
                cx = sum(p[0] for p in cluster)/len(cluster)
                cy = sum(p[1] for p in cluster)/len(cluster)
                cones.append(((cx, cy), max(.06, span/2)))
    return segments, cones


def recorded_static_scans(trial):
    """Calibrated ranges of the trial's scans before the car moved."""
    rows = []
    for line in (Path(trial)/'feedback.jsonl').read_text().splitlines():
        row = json.loads(line)
        control = ((row.get('state') or {}).get('status') or {}).get('control') or {}
        scan = (row.get('lidar') or {}).get('scan')
        if isinstance(scan, dict) and control.get('motor', 1500) == 1500:
            rows.append(scan['ranges'])
        elif control.get('motor', 1500) != 1500:
            break
    return rows


class Lidar:
    def __init__(self, shapes, rng, dropout=True):
        self.segments, self.cones = shapes
        self.rng, self.dropout = rng, dropout

    def scan(self, pose, moving=True):
        px, py, th = pose
        bins = [None]*360
        for k in range(360):
            a = th-math.radians(k)  # bin k: k degrees clockwise from forward
            dx, dy = math.cos(a), math.sin(a)
            best = None
            for (ax, ay), (bx, by) in self.segments:
                ex, ey = bx-ax, by-ay
                den = dx*ey-dy*ex
                if abs(den) < 1e-9:
                    continue
                t = ((ax-px)*ey-(ay-py)*ex)/den
                u = ((ax-px)*dy-(ay-py)*dx)/den
                if t > .02 and -1e-9 <= u <= 1+1e-9 and (best is None or t < best):
                    best = t
            for (cx, cy), radius in self.cones:
                fx, fy = px-cx, py-cy
                b = fx*dx+fy*dy
                disc = b*b-(fx*fx+fy*fy-radius*radius)
                if disc >= 0:
                    t = -b-math.sqrt(disc)
                    if t > .02 and (best is None or t < best):
                        best = t
            if best is not None and best < 8:
                bins[k] = best
        ranges = [None if r is None else round(r+self.rng.gauss(0, .008), 3) for r in bins]
        if self.dropout and moving:  # start scans on the car were nearly complete
            for k in range(360):  # beam split at depth steps: far side often missing
                a, b = ranges[k], ranges[(k+1) % 360]
                if a is not None and b is not None and abs(a-b) > .3 and self.rng.random() < .35:
                    ranges[(k+1) % 360 if b > a else k] = None
            k = 5
            while k <= 17:  # recurring right-front dropout band
                if self.rng.random() < .12:
                    run = self.rng.randint(1, 4)
                    for j in range(k, min(18, k+run)):
                        ranges[j] = None
                    k += run
                k += 1
        return ranges


def body_gap(shapes, pose):
    """Clearance of the body box to boards/cones (sampled along each shape)."""
    px, py, th = pose
    c, s = math.cos(th), math.sin(th)
    segments, cones = shapes
    samples = []
    for (ax, ay), (bx, by) in segments:
        n = max(2, int(math.dist((ax, ay), (bx, by))/.02))
        samples.extend(((ax+(bx-ax)*i/n, ay+(by-ay)*i/n), 0.) for i in range(n+1))
    samples.extend((centre, radius) for centre, radius in cones)
    best = math.inf
    for (wx, wy), radius in samples:
        dx, dy = wx-px, wy-py
        if abs(dx) > 1.2 or abs(dy) > 1.2:
            continue
        lx, ly = c*dx+s*dy, -s*dx+c*dy
        best = min(best, math.hypot(max(-REAR-lx, 0, lx-FRONT), max(abs(ly)-SIDE, 0))-radius)
    return best


def run(points, seed, args, geometry):
    rng = random.Random(seed)
    sys.path.insert(0, str(TESTS))
    harness_mod = load_module('test_turn_control_sim', TESTS/'test_turn_control.py')
    harness = harness_mod.TurnControlTests('test_compact_trial_preview_default_1670_and_placement_admission_keep_neutral')
    harness.setUp()
    lidar = Lidar(points, rng, dropout=not args.no_dropout)  # points = (segments, cones)
    # The car is stationary until the drive starts: replay the trial's own
    # recorded stationary scans there (ground truth), ray-cast once it moves.
    static = recorded_static_scans(args.trial_dir)
    x, y, th = 0., 0., 0.
    v, steer = 0., 1500.
    seq = 1

    def make_scan(now):
        nonlocal seq
        seq += 1
        moved = math.hypot(x, y) > .005 or abs(th) > .002
        ranges = (lidar.scan((x, y, th), moving=v > .2) if moved or not static
                  else list(static[seq % len(static)]))
        row = {'seq': seq, 'at_ms': round(now*1000), 'ranges': ranges}
        geometry.stdin.write(json.dumps(row)+'\n')
        geometry.stdin.flush()
        full = json.loads(geometry.stdout.readline())
        full['frame_id'] = 'lidar_origin_coarse_body_heading'
        return full

    harness.console.scan = make_scan(0.)
    harness.console.scan_at = 0.
    data = {'op': 'turn_cone_start', 'boot': harness.console.boot, 'epoch': harness.console.control_epoch,
            'tick': harness.console.status['control']['tick'], 'placement_confirmed': True,
            'continue_route': True, 'max_drive_s': args.max_drive_s}
    with harness_mod.patch.object(harness.server.time, 'monotonic', return_value=0.):
        start = harness.console.autonomy_command(data)
    trace, stages, min_gap, contact_at = [], [], math.inf, None
    now, last_scan = 0., 0.
    while now < args.max_seconds:
        now = round(now+.02, 6)
        control = harness.console.status['control']
        motor = control.get('motor', 1500) if control.get('armed') else 1500
        servo = control.get('servo', 1500)
        steer += (servo-steer)*min(1., .02/STEER_TAU)
        target_v = SPEED.get(motor, 0.)
        if v < target_v:
            v = min(target_v, v+ACCEL*.02)
        elif motor == 1500:
            v = max(0., v-COAST_DECEL*.02)
        else:
            v = max(target_v, v-ACCEL*.02)
        kappa = (steer-1500)/PWM_PER_KAPPA
        x += v*.02*math.cos(th)
        y += v*.02*math.sin(th)
        th += v*.02*kappa
        gap = body_gap(points, (x, y, th))
        min_gap = min(min_gap, gap)
        if gap <= 0 and contact_at is None:
            contact_at = now
        if now-last_scan >= .1-1e-9:
            last_scan = now
            harness.tick(now, make_scan(now))
        else:
            harness.tick(now, harness.console.scan)
        session = harness.console.auto_session
        motion = (session or {}).get('turn_motion')
        stage = None if motion is None else (motion.phase, motion.route_stage)
        if stage and (not stages or stages[-1][1] != stage):
            stages.append((round(now, 2), stage))
        trace.append((now, x, y, th, v, motor, servo, None if motion is None else motion.reason))
        if session is None and v < .02:
            break
    result = harness.console.auto_result or {}
    turn = (result.get('turn') or {}) if isinstance(result, dict) else {}
    harness.tearDown()
    sys.path.remove(str(TESTS))
    return {'seed': seed, 'start': start, 'end_reason': result.get('reason') if isinstance(result, dict) else None,
            'end_at': round(now, 2), 'stages': stages, 'min_body_gap_m': round(min_gap, 3), 'contact_at': contact_at,
            'first_pass': bool(turn.get('first_pass_evidence')),
            'second_handover': bool(turn.get('second_target_handover_observed')),
            'second_pass': bool(turn.get('two_target_observed_pass_complete')),
            'trace': trace}


def render(points, results, path):
    tm = load_module('trial_map_png', ROOT/'scripts'/'trial-map.py')
    segments, cones = points
    ends = [e for seg in segments for e in seg]+[c for c, _ in cones]
    xs = [p[0] for p in ends]+[t[1] for r in results for t in r['trace']]
    ys = [p[1] for p in ends]+[t[2] for r in results for t in r['trace']]
    xmin, xmax, ymin, ymax = min(xs)-.3, max(xs)+.3, min(ys)-.3, max(ys)+.3
    scale = min(1000/(xmax-xmin), 1000/(ymax-ymin), 150)
    img = tm.Image(int((ymax-ymin)*scale)+1, int((xmax-xmin)*scale)+1)

    def uv(x, y):
        return int((ymax-y)*scale), int((xmax-x)*scale)
    for a, b in segments:
        img.line(uv(*a), uv(*b), (120, 120, 120))
    for (cx, cy), radius in cones:
        img.dot(*uv(cx, cy), (220, 0, 0), max(1, int(radius*scale)))
    colors = [(0, 150, 0), (0, 60, 230), (220, 0, 0), (240, 140, 0), (150, 60, 200), (0, 150, 150)]
    for n, r in enumerate(results):
        color = colors[n % len(colors)]
        for a, b in zip(r['trace'], r['trace'][1:]):
            img.line(uv(a[1], a[2]), uv(b[1], b[2]), color)
        if r['contact_at'] is not None:
            t = next(t for t in r['trace'] if t[0] >= r['contact_at'])
            img.dot(*uv(t[1], t[2]), (0, 0, 0), 4)
    img.save(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('trial_dir', help='recorded trial whose rebuilt map is the world')
    parser.add_argument('--seeds', type=int, default=3)
    parser.add_argument('--max-seconds', type=float, default=20.)
    parser.add_argument('--max-drive-s', type=float, default=16.)
    parser.add_argument('--start-jitter-m', type=float, default=.03)
    parser.add_argument('--start-jitter-deg', type=float, default=2.)
    parser.add_argument('--no-dropout', action='store_true')
    parser.add_argument('--min-hits', type=int, default=4)
    parser.add_argument('--png')
    args = parser.parse_args()
    if not GEOMETRY.exists():
        sys.exit('build first: cargo build --release -q --manifest-path tools/scan-geometry/Cargo.toml')
    points = extract_shapes(world_map(args.trial_dir, args.min_hits), random.Random(7))
    print('world:', len(points[0]), 'boards,', len(points[1]), 'cones', [tuple(round(v, 2) for v in c) for c, _ in points[1]])
    geometry = subprocess.Popen([str(GEOMETRY)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
    results = []
    try:
        for seed in range(args.seeds):
            r = run(points, seed, args, geometry)
            results.append(r)
            print(f"seed {seed}: end {r['end_reason']} at {r['end_at']}s | first pass {r['first_pass']} | "
                  f"2nd handover {r['second_handover']} | 2nd pass {r['second_pass']} | "
                  f"min body gap {r['min_body_gap_m']} m | contact {r['contact_at']}")
            print('    stages:', ' -> '.join(f'{t}s {p}/{s}' for t, (p, s) in r['stages']))
    finally:
        geometry.stdin.close()
        geometry.wait()
    if args.png:
        render(points, results, args.png)
        print('png', args.png)


if __name__ == '__main__':
    main()
