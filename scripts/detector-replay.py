#!/usr/bin/env python3
"""Replay saved trial scans through compact-object trackers (read-only analysis).

Usage (from the repository root):
    python3 -I scripts/detector-replay.py work/vehicle-dynamic-deploy-20261008/left-cone-trial-2026101*-*

For every trial directory, feeds the saved scans (feedback.jsonl, in scan order)
through several tracker configurations and prints, per sector:
  seen   = frames with a returned (observed) target
  conf   = frames with a confirmed target
  ids    = identities created (each reset of the same object adds one)
  first  = time (s from the first saved scan) of the first confirmation
A robust detector shows few ids per real object and an early first confirmation.

Caveats: the observer saved about every other scan, so the replay sees ~5 Hz
instead of the controller's ~10 Hz (gaps are twice as long); results are a
conservative comparison between configurations, not a controller replay.
"""
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'web'/'vehicle-console'))
from compact_target import CompactTargetTracker  # noqa: E402

SECTORS = {
    # first target: the route's first tracker (acquisition 45..135 deg left)
    'first': dict(maintenance_bearing_rad=(-math.pi, math.pi), select_associated=True),
    # objects ahead (second-target acquisition sector)
    'ahead': dict(acquisition_bearing_rad=(-math.pi/2, math.pi/2),
                  maintenance_bearing_rad=(-math.pi, math.pi/2), select_associated=True),
}
CONFIGS = {
    'strict(51)': {},
    'gap(52)': {'unconfirmed_boundary_gap': True},
    'tolerant': {'unconfirmed_boundary_gap': True, 'tolerant': True},
}


def scans(base):
    rows, last = [], None
    for line in (base/'feedback.jsonl').read_text().splitlines():
        row = json.loads(line)
        scan = (row.get('lidar') or {}).get('scan')
        if not isinstance(scan, dict) or not isinstance(scan.get('ranges'), list):
            continue
        if last is not None and scan['seq'] <= last:
            continue
        last = scan['seq']
        rows.append(scan)
    return rows


def replay(rows, sector, extra):
    tracker = CompactTargetTracker(**SECTORS[sector], **extra)
    t0 = rows[0]['at_ms']
    seen = conf = 0
    ids, first = set(), None
    for scan in rows:
        value = {**scan, 'received_at': scan['at_ms']/1000}
        target = tracker.update(value, scan['at_ms']/1000)
        if target is None:
            continue
        seen += 1
        ids.add(target['track_id'])
        if target['confirmed']:
            conf += 1
            if first is None:
                first = (scan['at_ms']-t0)/1000
    return seen, conf, len(ids), first


def main():
    bases = [Path(p) for p in sys.argv[1:]]
    if not bases:
        print(__doc__)
        return
    totals = {(s, c): [0, 0, 0] for s in SECTORS for c in CONFIGS}
    for base in bases:
        if not (base/'feedback.jsonl').exists():
            continue
        rows = scans(base)
        if len(rows) < 3:
            continue
        parts = []
        for sector in SECTORS:
            for name, extra in CONFIGS.items():
                seen, conf, ids, first = replay(rows, sector, extra)
                total = totals[(sector, name)]
                total[0] += seen
                total[1] += conf
                total[2] += ids
                parts.append(f"{sector}/{name}: seen {seen:3d} conf {conf:3d} ids {ids:2d} first "
                             f"{'-' if first is None else f'{first:5.2f}'}")
        print(base.name, f'({len(rows)} scans)')
        for part in parts:
            print('   ', part)
    print('TOTAL')
    for (sector, name), (seen, conf, ids) in totals.items():
        print(f'    {sector}/{name}: seen {seen} conf {conf} ids {ids}')


if __name__ == '__main__':
    main()
