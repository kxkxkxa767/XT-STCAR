#!/usr/bin/env python3
"""Pass rate of the Rust race controller over the rule-book track range. Offline, no vehicle.

    cargo build --release --manifest-path tools/scan-geometry/Cargo.toml --example surface_sim
    python3 scripts/track-spec-sim.py --out work/track-spec-NEW [--n 1500] [--jobs 8]

Every layout is a random draw from the rule figures (rules Fig. 5, the table the user confirmed
on 2026-11-08). The real Supervisor drives; surface_sim supplies lidar, camera, IMU and the car.
A run is judged on the true pose by the rule book, not by the controller's own "finished" flag:

    clean    finished with no penalty
    penalty  finished, with penalties (crosswalk 10/20/30, light 20/30/50, finish 5)
    fail     did not finish: refused at the start, touched a board or cone, stopped by the
             controller, ran out of its drive budget, or stood still over 10 s

Contact ends a simulated run, so a cone touch counts as a failure here although the rules
would only add 5 s; the car's own 8 cm body guard normally ends such a run anyway.
"""
import argparse
import collections
import concurrent.futures as cf
import copy
import json
import math
import random
import shutil
import statistics
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BIN = ROOT / 'tools/scan-geometry/target/release/examples/surface_sim'
BASE = json.loads((ROOT / 'config/track-spec-sim-20261108.json').read_text())

CAR_WIDTH_M = 0.28
CONE_BASE_HALF_M = 0.14
STRIPE_DEPTH_M = 0.297       # the 29.7 cm side of the paper runs along the lane (rules Fig. 1)
FINISH_ZONE_M = 0.80
CROSSWALK_AREA_FROM = 0.55   # of the start divider, measured from rules Fig. 1 and 5
LIGHT_AREA_TO = 0.62         # of the light divider, measured from rules Fig. 1 and 5
MIN_CONE_SPACING_M = 1.5
KEEP_FAILURES = 12           # full traces kept per failure class

# Passage left beside a cone: the car, its share of margin and the cone base.
MIN_PASSAGE_M = CONE_BASE_HALF_M + CAR_WIDTH_M + 0.10


def draw_layout(rng, lanes):
    """Fig. 5 ranges; anything a builder could not actually lay out is drawn again."""
    while True:
        length, width = rng.uniform(5, 8), rng.uniform(4, 6)
        lane, light = (0.9, 0.9) if lanes == '0.9' else (rng.uniform(1, 2), rng.uniform(1, 2))
        start_div, light_div = rng.uniform(3, 6), rng.uniform(3, 6)
        right_gap, left_gap = rng.uniform(1, 3), rng.uniform(1, 3)
        cone1_x, cone2_x = length - right_gap, left_gap
        cone_y = width / 2  # the build guide puts the cones on the middle line; 2-3 m from the top wall is W/2
        # The right cone lies past the start divider's end and the left cone before the light
        # divider's start (Fig. 1, 5); a cone beside a divider needs room for the car to pass.
        if start_div > cone1_x or length - light_div < cone2_x:
            continue
        if cone1_x - cone2_x < MIN_CONE_SPACING_M:
            continue
        if cone_y - lane < MIN_PASSAGE_M or cone_y - light < MIN_PASSAGE_M:
            continue
        return length, width, lane, light, start_div, light_div, cone1_x, cone2_x, cone_y


def case_config(rng, seed, layout, placed=True):
    length, width, lane, light, start_div, light_div, cone1_x, cone2_x, cone_y = layout
    cfg = copy.deepcopy(BASE)
    cfg['seed'] = seed
    near = max(CROSSWALK_AREA_FROM * start_div, FINISH_ZONE_M + 0.3)
    stripe = rng.uniform(near, start_div - STRIPE_DEPTH_M) if placed else start_div - 0.25
    # The beam and the gantry both stand inside the light area.
    gantry = rng.uniform(length - light_div + 1., length - (1 - LIGHT_AREA_TO) * light_div)
    cfg['layout'] = {
        'length_m': length, 'width_m': width, 'lane_width_m': lane, 'light_width_m': light,
        'start_divider_m': start_div, 'light_divider_m': light_div,
        'stripe_x_m': stripe, 'gantry_x_m': gantry, 'beam_gap_m': 1.0,
        'cone1_x_m': cone1_x, 'cone2_x_m': cone2_x, 'cone1_y_m': cone_y, 'cone2_y_m': cone_y,
        'start_x_m': rng.uniform(0.30, 0.50), 'start_y_m': lane / 2 + rng.uniform(-0.05, 0.05),
        'start_yaw_rad': rng.uniform(-0.035, 0.035),
    }
    cfg['camera']['red_duration_s'] = rng.uniform(3, 10)
    cfg['camera']['red_on_delay_s'] = rng.uniform(0.2, 1.3)  # 1.0-1.3 s on the practice lamp
    cfg['sensors']['imu_drift_mps2'] = rng.uniform(-0.135, 0.04)  # standstill residuals y57-y66
    return cfg


def judge(s):
    """Rule book section 2.4.2, applied to the true pose."""
    crosswalk, light = s.get('crosswalk_stop'), s.get('light_stop')
    if s.get('start_error'):
        return 'fail', 'start_refused', {}
    if s.get('contact'):
        return 'fail', f"contact_{s['contact']['kind']}", {}
    if not s.get('controller_finish'):
        return 'fail', f"no_finish:{s.get('terminal_reason')}", {}
    if s.get('stopped_elsewhere_s', 0) > 10:
        return 'fail', 'stopped_over_10s', {}
    penalties = {}
    if crosswalk is None:
        penalties['crosswalk_no_stop'] = 30
    elif crosswalk['true_gap_m'] < 0:
        penalties['crosswalk_over_line'] = 20
    elif s['crosswalk_stationary_s'] + 1e-9 < 3:
        penalties['crosswalk_short_wait'] = 10
    if light is None:
        penalties['light_no_stop'] = 50
    else:
        if s.get('light_crossed_before_green'):
            penalties['light_before_green'] = 30
        if not s.get('light_stopped_in_zone'):
            penalties['light_wheel_outside'] = 20
    if not s.get('finish_stopped_in_zone'):
        penalties['finish_outside_zone'] = 5
    return ('penalty', ','.join(sorted(penalties)), penalties) if penalties else ('clean', '', {})


def run_case(job):
    name, cfg, out = job
    folder = out / name
    folder.parent.mkdir(parents=True, exist_ok=True)
    if folder.exists():
        shutil.rmtree(folder)
    config_path = out / f'{name}.config.json'
    config_path.write_text(json.dumps(cfg))
    done = subprocess.run([str(BIN), str(config_path), str(folder)], capture_output=True, text=True)
    if done.returncode != 0:
        return {'name': name, 'cfg': cfg, 'kind': 'fail', 'why': 'simulator_crash', 'penalties': {},
                'crash': done.stderr[-400:]}
    s = json.loads((folder / 'summary.json').read_text())
    kind, why, penalties = judge(s)
    last = s['stage_changes'][-1] if s['stage_changes'] else {}
    events = s.get('route_events') or {}
    wait = s.get('light_wait_s') or 0.
    start, finish = s.get('start_line_at'), s.get('finish_line_at')
    rule_time = None if kind == 'fail' or start is None or finish is None else finish - start - wait + sum(penalties.values())
    gap = lambda d, k: None if d is None else d.get(k)
    row = {
        'name': name, 'cfg': cfg, 'kind': kind, 'why': why, 'penalties': penalties,
        'terminal': s.get('terminal_reason'), 'stage': last.get('stage'), 'contact': s.get('contact'),
        'events': sorted(events), 'internal_s': s.get('simulated_effective_s'), 'rule_s': rule_time,
        'light_wait_s': wait, 'wall_gap': s.get('min_wall_gap_m'), 'cone_gap': s.get('min_cone_gap_m'),
        'crosswalk_gap': gap(s.get('crosswalk_stop'), 'true_gap_m'), 'crosswalk_still': s.get('crosswalk_stationary_s'),
        'light_gap': gap(s.get('light_stop'), 'true_gap_m'), 'light_rear': gap(s.get('light_stop'), 'rear_past_beam_m'),
        'finish_gap': gap(s.get('finish_stop'), 'true_gap_m'), 'finish_rear': gap(s.get('finish_stop'), 'rear_past_zone_m'),
        'beam_at': s.get('beam_trigger_at'), 'stopped_elsewhere_s': s.get('stopped_elsewhere_s'),
        'start_error': s.get('start_error'), 'traction': s.get('max_traction_demand_ratio'),
    }
    return row


def wilson(k, n, z=1.96):
    if n == 0:
        return (0., 0.)
    p = k / n
    d = 1 + z * z / n
    c = p + z * z / (2 * n)
    m = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return ((c - m) / d, (c + m) / d)


def tally(rows):
    n = len(rows)
    count = collections.Counter(r['kind'] for r in rows)
    finished = count['clean'] + count['penalty']
    times = [r['rule_s'] for r in rows if r['kind'] == 'clean' and r['rule_s'] is not None]
    return {
        'n': n, 'clean': count['clean'], 'penalty': count['penalty'], 'fail': count['fail'],
        'clean_rate': count['clean'] / n if n else None, 'clean_ci95': wilson(count['clean'], n),
        'finish_rate': finished / n if n else None, 'finish_ci95': wilson(finished, n),
        'median_clean_rule_s': statistics.median(times) if times else None,
    }


def binned(rows, label, value, edges):
    out = []
    for lo, hi in zip(edges, edges[1:]):
        picked = [r for r in rows if lo <= value(r) < hi or (hi == edges[-1] and value(r) == hi)]
        if picked:
            out.append({'axis': label, 'from': lo, 'to': hi, **tally(picked)})
    return out


def show(title, entry):
    print(f"{title:36s} n={entry['n']:5d}  零罚时完赛 {100*entry['clean_rate']:5.1f}%  "
          f"完赛(含罚时) {100*entry['finish_rate']:5.1f}%  失败 {entry['fail']:4d}"
          + (f"  中位规则用时 {entry['median_clean_rule_s']:.2f}s" if entry['median_clean_rule_s'] else ''))


def causes(rows, top=14):
    return collections.Counter((r['why'], r.get('stage') or '') for r in rows if r['kind'] != 'clean').most_common(top)


def layout_of(r):
    return r['cfg']['layout']


AXES = [
    ('L 场地长', lambda r: layout_of(r)['length_m'], [5, 6, 7, 8]),
    ('W 场地宽', lambda r: layout_of(r)['width_m'], [4, 4.5, 5, 5.5, 6]),
    ('起步道宽', lambda r: layout_of(r)['lane_width_m'], [1, 1.25, 1.5, 1.75, 2]),
    ('灯道宽', lambda r: layout_of(r)['light_width_m'], [1, 1.25, 1.5, 1.75, 2]),
    ('起步隔板', lambda r: layout_of(r)['start_divider_m'], [3, 4, 5, 6]),
    ('灯道隔板', lambda r: layout_of(r)['light_divider_m'], [3, 4, 5, 6]),
    ('右桶距端墙', lambda r: layout_of(r)['length_m'] - layout_of(r)['cone1_x_m'], [1, 1.5, 2, 2.5, 3]),
    ('左桶距端墙', lambda r: layout_of(r)['cone2_x_m'], [1, 1.5, 2, 2.5, 3]),
    ('两桶间距', lambda r: layout_of(r)['cone1_x_m'] - layout_of(r)['cone2_x_m'], [1.5, 2, 2.5, 3, 4, 6]),
    ('起步隔板末端到右端墙', lambda r: layout_of(r)['length_m'] - layout_of(r)['start_divider_m'], [1, 1.5, 2, 2.5, 3, 3.5, 4, 5.5]),
    ('右桶超出起步隔板末端', lambda r: layout_of(r)['cone1_x_m'] - layout_of(r)['start_divider_m'], [0, 0.5, 1, 1.5, 2, 3, 5.5]),
    ('左桶早于灯道入口', lambda r: layout_of(r)['length_m'] - layout_of(r)['light_divider_m'] - layout_of(r)['cone2_x_m'], [0, 0.5, 1, 1.5, 2, 3, 5.5]),
    ('桶到灯道隔板', lambda r: layout_of(r)['cone1_y_m'] - layout_of(r)['light_width_m'], [0.5, 0.75, 1, 1.25, 1.5, 2.2]),
    ('桶到起步隔板', lambda r: layout_of(r)['cone1_y_m'] - layout_of(r)['lane_width_m'], [0.5, 0.75, 1, 1.25, 1.5, 2.2]),
    ('斑马线距起点', lambda r: layout_of(r)['stripe_x_m'], [1, 2, 3, 4, 5, 6]),
    ('门架距灯道入口', lambda r: layout_of(r)['gantry_x_m'] - (layout_of(r)['length_m'] - layout_of(r)['light_divider_m']), [1, 1.5, 2, 2.5, 3, 4]),
    ('红灯时长', lambda r: r['cfg']['camera']['red_duration_s'], [3, 5, 7, 10]),
]


def sweep_jobs(out):
    """One factor at a time around the rule book's 8x4 example."""
    jobs = []
    def add(axis, value, overrides, seeds=5):
        for seed in range(seeds):
            rng = random.Random(f'sweep-{axis}-{value}-{seed}')
            length = overrides.get('L', 8.)
            width = overrides.get('W', 4.)
            lane = overrides.get('lane', 0.9)
            light = overrides.get('light', lane)
            start_div = overrides.get('D1', 3.2)
            light_div = overrides.get('D2', 3.2)
            right_gap = overrides.get('c1', 0.25 * length)
            left_gap = overrides.get('c2', 0.25 * length)
            layout = (length, width, lane, light, start_div, light_div, length - right_gap, left_gap, width / 2)
            cfg = case_config(rng, 900000 + seed, layout)
            cfg['camera']['red_duration_s'] = 5.
            if 'stripe_frac' in overrides:
                cfg['layout']['stripe_x_m'] = CROSSWALK_AREA_FROM * start_div + overrides['stripe_frac'] * (start_div - STRIPE_DEPTH_M - CROSSWALK_AREA_FROM * start_div)
            if 'gantry_frac' in overrides:
                low, high = length - light_div + 1., length - (1 - LIGHT_AREA_TO) * light_div
                cfg['layout']['gantry_x_m'] = low + overrides['gantry_frac'] * (high - low)
            if 'span' in overrides:
                cfg['layout']['gantry_post_span_m'] = overrides['span']
            if overrides.get('practice_gantry'):
                cfg['layout']['gantry_x_m'] = length - light_div - 0.025
            jobs.append((f'sweep/{axis}={value}/s{seed}', cfg, out))
    # The practice field as built (gantry at the divider's start), and the guide's 8x4 example
    # with the crosswalk and light drawn from their areas.
    for seed in range(200):
        rng = random.Random(f'practice-{seed}')
        cfg = case_config(rng, 700000 + seed, (8., 4., 0.9, 0.9, 3.2, 3.2, 6., 2., 2.))
        cfg['layout']['stripe_x_m'] = 2.95
        cfg['layout']['gantry_x_m'] = 8. - 3.2 - 0.025
        cfg['camera']['red_duration_s'] = 10.
        cfg['camera']['red_on_delay_s'] = rng.uniform(1.0, 1.3)
        jobs.append((f'sweep/practice_field/s{seed}', cfg, out))
    for seed in range(200):
        rng = random.Random(f'guide-{seed}')
        cfg = case_config(rng, 800000 + seed, (8., 4., 0.9, 0.9, 3.2, 3.2, 6., 2., 2.))
        jobs.append((f'sweep/guide_8x4_random_areas/s{seed}', cfg, out))
    # The rule book's own drawing (Fig. 5 pixel proportions: field 438 x 260 px, start divider to
    # 288 px, right cone at 298 px, light divider from 150 px, left cone at 120 px).
    def fig5(length, width, lane):
        return (length, width, lane, lane, 288 / 438 * length, 288 / 438 * length,
                298 / 438 * length, 120 / 438 * length, width / 2)
    for name, layout in (('fig5_L8_W4.75_lane0.9', fig5(8, 4.75, 0.9)), ('fig5_L8_W4.75_lane1.1', fig5(8, 4.75, 1.1)),
                         ('fig5_L7_W4.2_lane0.9', fig5(7, 4.2, 0.9)), ('fig5_L6_W4_lane0.9', fig5(6, 4, 0.9)),
                         ('fig5_L5_W4_lane0.9', fig5(5, 4, 0.9))):
        for seed in range(100):
            rng = random.Random(f'{name}-{seed}')
            jobs.append((f'sweep/{name}/s{seed}', case_config(rng, 600000 + seed, layout), out))
    for v in (5, 5.5, 6, 6.5, 7, 7.5, 8):
        add('L', v, {'L': v, 'D1': min(3.2, v - 0.25 * v), 'D2': min(3.2, v - 0.25 * v)})
    for v in (4, 4.5, 5, 5.5, 6):
        add('W', v, {'W': v})
    for width in (4, 5, 6):
        for v in (0.9, 1.0, 1.2, 1.4, 1.5, 1.6, 1.8, 2.0):
            if width / 2 - v >= MIN_PASSAGE_M:
                add(f'lane@W{width}', v, {'W': width, 'lane': v})
    for width in (5, 6):
        for v in (0.9, 1.0, 1.2, 1.4, 1.5, 1.6, 1.8, 2.0):
            if width / 2 - v >= MIN_PASSAGE_M:
                add(f'start_lane_only@W{width}', v, {'W': width, 'lane': v, 'light': 0.9})
                add(f'light_lane_only@W{width}', v, {'W': width, 'lane': 0.9, 'light': v})
    for v in (3, 3.5, 4, 4.5, 5, 5.5, 6):
        add('D1', v, {'D1': v})
        add('D2', v, {'D2': v})
    for v in (1, 1.5, 2, 2.5, 3):
        add('c1', v, {'c1': v})
        add('c2', v, {'c2': v})
    for v in (0, 0.25, 0.5, 0.75, 1):
        add('stripe', v, {'stripe_frac': v})
        add('gantry', v, {'gantry_frac': v})
    add('practice_gantry', 1, {'practice_gantry': True}, seeds=20)
    for lane in (0.9, 1.2, 1.5, 1.8):
        add('gantry_0.9m_beam@lane', lane, {'W': 6, 'lane': 0.9, 'light': lane, 'span': 0.72}, seeds=40)
        add('gantry_lane_edge@lane', lane, {'W': 6, 'lane': 0.9, 'light': lane}, seeds=40)
    return jobs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--n', type=int, default=1500)
    ap.add_argument('--jobs', type=int, default=8)
    ap.add_argument('--seed', default='20261108')
    ap.add_argument('--report', help='write the compact JSON here')
    a = ap.parse_args()
    out = Path(a.out).resolve()
    out.mkdir(parents=True, exist_ok=False)

    jobs = []
    for mode in ('0.9', '1-2'):
        for i in range(a.n):
            rng = random.Random(f'{a.seed}-{mode}-{i}')
            layout = draw_layout(rng, mode)
            jobs.append((f'lane{mode}/{i:05d}', case_config(rng, 1000 + i, layout), out))
    jobs += sweep_jobs(out)
    print(f'{len(jobs)} runs on {a.jobs} workers')

    rows, kept = [], collections.Counter()
    with cf.ThreadPoolExecutor(a.jobs) as pool, (out / 'results.jsonl').open('w') as log:
        for done, row in enumerate(pool.map(run_case, jobs), 1):
            rows.append(row)
            log.write(json.dumps(row, ensure_ascii=False) + '\n')
            folder = out / row['name']
            config_path = out / f"{row['name']}.config.json"
            key = (row['name'].split('/')[0], row['why'])
            keep = row['kind'] != 'clean' and kept[key] < KEEP_FAILURES
            if keep:
                kept[key] += 1
            else:
                shutil.rmtree(folder, ignore_errors=True)
                config_path.unlink(missing_ok=True)
            if done % 500 == 0:
                print(done, 'done', flush=True)

    modes = {m: [r for r in rows if r['name'].startswith(f'lane{m}/')] for m in ('0.9', '1-2')}
    report = {'runs': len(rows), 'modes': {}}
    for mode, chosen in modes.items():
        total = tally(chosen)
        print()
        show(f'车道 {mode} m 全部', total)
        report['modes'][mode] = {'total': total, 'axes': [], 'causes': [[list(k), v] for k, v in causes(chosen, 30)]}
        for label, value, edges in AXES:
            if mode == '0.9' and label in ('起步道宽', '灯道宽'):
                continue
            entries = binned(chosen, label, value, edges)
            report['modes'][mode]['axes'] += entries
            for e in entries:
                show(f"  {label} {e['from']}-{e['to']}", e)
        print('  未零罚时完赛的原因:')
        for (why, stage), count in causes(chosen):
            print(f'    {count:5d}  {why}  @ {stage}')

    # Where the car is dependable (0.9 m lanes): the loosest simple box that holds at least 90 %.
    box = [r for r in modes['0.9'] if layout_of(r)['length_m'] - layout_of(r)['start_divider_m'] >= 3
           and layout_of(r)['length_m'] - layout_of(r)['cone1_x_m'] >= 1.25 and layout_of(r)['cone2_x_m'] >= 1.25
           and layout_of(r)['cone1_x_m'] - layout_of(r)['start_divider_m'] >= 1
           and layout_of(r)['length_m'] - layout_of(r)['light_divider_m'] - layout_of(r)['cone2_x_m'] >= 1]
    report['reliable_box_0.9'] = tally(box)
    show('0.9 m 可靠区（见报告）', report['reliable_box_0.9'])

    print('\n单因素扫描（8×4 示例为基准，每点 5 次）')
    sweep = collections.defaultdict(list)
    for r in rows:
        if r['name'].startswith('sweep/'):
            sweep[r['name'].split('/')[1]].append(r)
    report['sweep'] = {}
    for key in sweep:
        entry = tally(sweep[key])
        report['sweep'][key] = {**entry, 'causes': [[list(k), v] for k, v in causes(sweep[key], 6)]}
        show(key, entry)
    if a.report:
        Path(a.report).write_text(json.dumps(report, ensure_ascii=False, indent=1) + '\n')


if __name__ == '__main__':
    main()
