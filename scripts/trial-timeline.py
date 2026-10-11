#!/usr/bin/env python3
"""Print one vehicle trial as a readable timeline (read-only analysis).

Usage (from the repository root):
    python3 -I scripts/trial-timeline.py work/vehicle-dynamic-deploy-20261008/left-cone-trial-20261011-46
    python3 -I scripts/trial-timeline.py <trial-dir> --from 5.0 --objects

Reads before.json, feedback.jsonl, cli-result.json and after.json written by
the left-cone-trial-once-*.py helper. It never contacts the vehicle.

Caveats that matter when reading the output:
- feedback.jsonl is the observer's poll (~every 150 ms), so it usually holds
  every other lidar scan. The controller saw every scan; a decision can come
  from a scan that was never saved.
- Use the LIVE fields: state.autonomy.active.quality_issues and
  state.autonomy.active.turn. Keys such as quality_coast_trigger,
  quality_wait_trigger or left_turn_pause_trigger are frozen snapshots of
  the event that started them, not per-frame quality.
- Corridor angles are relative to the car: heading_left > 0 means the
  corridor axis is to the LEFT of the car heading; center_offset_left > 0
  means the corridor center line is to the car's left.
- Body gap is the distance from a known return to the measured body box
  (front .20, rear .18, sides .14 m from the lidar). Lidar bin i is the
  angle i degrees clockwise from forward (bins 1..179 are on the right).
"""
import argparse
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'web'/'vehicle-console'))
from compact_target import _candidates  # noqa: E402

FRONT, REAR, SIDE = .20, .18, .14


def deg(value):
    return None if value is None else round(math.degrees(value), 1)


def body_gap(x, y):
    return math.hypot(max(-REAR-x, 0, x-FRONT), max(abs(y)-SIDE, 0))


def nearest(ranges):
    best = None
    for i, r in enumerate(ranges):
        if r is None:
            continue
        x, y = r*math.cos(math.radians(i)), -r*math.sin(math.radians(i))
        g = body_gap(x, y)
        if best is None or g < best[0]:
            best = (g, i, r)
    return best


def start_geometry(state):
    preview = state['autonomy'].get('turn_preview') or {}
    turn = preview.get('turn') if isinstance(preview.get('turn'), dict) else preview
    goal, entry = turn.get('turn_goal') or {}, turn.get('entry_bearing') or {}
    return {'front_wall_m': goal.get('front_wall_m'), 'incoming_board_end_m': goal.get('incoming_left_end_m'),
            'exit_width_m': goal.get('width_m'), 'exit_heading_deg': deg(goal.get('heading_left_rad')),
            'incoming_heading_deg': deg(goal.get('incoming_heading_left_rad')),
            'entry_lateral_gap_m': entry.get('lateral_gap_m'), 'presteer_cap_pwm': entry.get('steering_cap_pwm'),
            'turn_ready': state['autonomy'].get('turn_ready'), 'turn_rejection': state['autonomy'].get('turn_rejection')}


def fmt(value):
    if isinstance(value, float):
        return round(value, 3)
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('trial_dir')
    parser.add_argument('--from', dest='start', type=float, default=0., help='first elapsed second to print')
    parser.add_argument('--objects', action='store_true', help='also list all compact lidar objects per frame')
    args = parser.parse_args()
    base = Path(args.trial_dir)

    before = json.loads((base/'before.json').read_text())
    print('START', {k: fmt(v) for k, v in start_geometry(before).items()})

    for line in (base/'feedback.jsonl').read_text().splitlines():
        row = json.loads(line)
        if 'state' not in row:
            print(round(row.get('elapsed_s', 0), 2), 'OBSERVER', row)
            continue
        if row['elapsed_s'] < args.start:
            continue
        scan = row['lidar']['scan']
        control = row['state']['status']['control']
        active = row['state']['autonomy'].get('active')
        near = nearest(scan['ranges'])
        head = f"{row['elapsed_s']:6.2f} scan {scan['seq']}"
        tail = f"| near gap {near[0]:.3f} bin {near[1]}" if near else ''
        if not isinstance(active, dict):
            print(head, 'LOCKED', 'M', control.get('motor'), 'S', control.get('servo'), control.get('reason'), tail)
            continue
        turn = active.get('turn') or {}
        clearance = active.get('clearance_current') or {}
        corridor = turn.get('right_exit_geometry') or {}
        first = turn.get('compact_target') or {}
        second = turn.get('second_target') or {}
        parts = [head, (turn.get('phase') or '')[:6], (turn.get('route_stage') or '')[:11],
                 (turn.get('reason') or '')[:44], 'M', control.get('motor'), 'S', control.get('servo'),
                 'tgt', turn.get('steering_target'), '| q', active.get('quality_issues')]
        if clearance.get('front_unknown_bins'):
            parts += ['nulls', clearance.get('front_unknown_bins'), 'bridged', clearance.get('front_bridged_bins')]
        if corridor:
            parts += ['| corr', (deg(corridor.get('heading_left_rad')), fmt(corridor.get('center_offset_left_m')),
                                 fmt(corridor.get('width_m')))]
        if first:
            parts += ['| 1st', first.get('track_id'), first.get('confirmation_count'),
                      fmt(first.get('range_m')), deg(first.get('bearing_left_rad'))]
        if second:
            parts += ['| 2nd', second.get('track_id'), second.get('confirmed'),
                      fmt(second.get('range_m')), deg(second.get('bearing_left_rad'))]
        if turn.get('handover_observed'):
            parts += ['HO']
        if turn.get('first_pass_evidence'):
            parts += ['PASS1']
        print(*parts, tail)
        if args.objects:
            try:
                objects = _candidates(scan['ranges'], bearing_range_rad=(-math.pi, math.pi))
            except (ValueError, TypeError, KeyError) as error:
                objects, note = [], type(error).__name__
            else:
                note = ''
            print('        objects (bearing_left_deg, range_m, diameter_m):',
                  [(deg(o['bearing_left_rad']), fmt(o['range_m']), fmt(o['diameter_m'])) for o in objects], note)

    result = json.loads((base/'cli-result.json').read_text())
    turn = result.get('turn') or {}
    print('END reason', result.get('reason'), '| turn terminal', turn.get('terminal_reason'),
          '| stage', turn.get('turn_stage'), '| route', turn.get('route_stage'),
          '| handover', turn.get('handover_observed'), '| left-turn pause', turn.get('left_turn_pause_used'),
          '| orbit wait', turn.get('quality_wait_used'), '| 2nd handover', turn.get('second_target_handover_observed'),
          '| 2nd pass', turn.get('two_target_observed_pass_complete'))
    after = json.loads((base/'after.json').read_text())
    rest = nearest(after[-1]['lidar']['scan']['ranges'])
    print('REST nearest body gap', round(rest[0], 3), 'bin', rest[1], 'range', rest[2],
          '(gap ~0 at rest suggests contact; confirm with the operator)')


if __name__ == '__main__':
    main()
