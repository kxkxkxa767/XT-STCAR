#!/usr/bin/env python3
"""Conditional second-orbit replay using saved raw scans, never a device.

Usage: python3 -I scripts/second-orbit-replay.py <trial-dir> [--source-root ROOT]
       [--json NEW_OUTPUT.json]

Find the first recorded second-orbit handover, then warm the real second
tracker on saved scans until it confirms a current support. Initialize the
stage explicitly from the corresponding recorded state. Later ACKs are an
instant-adoption model, not the car's old replies to different commands.
Publication times/ranges/geometry are unchanged; the receive clock follows
publication intervals. Repeated 20 ms control ticks reuse the same scan, never
interpolate a new scan. Runtime freshness and center ACK remain active. Body
clearance uses the conservative 8 cm maneuver reference, with the runtime's
current cone-shaped support exclusions but without the service's conditional
4 cm left-turn relaxation. It is not a full service gate reconstruction.
Unhandled scan-quality faults stop this limited phase replay.

This is NOT a whole-trial reconstruction or a prediction of the new physical
trajectory. The old scans stop following the proposed commands once they
diverge. A missing raw frame remains missing. JSON records initialization
provenance, input/source hashes, every control tick and scan-boundary results.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import sys


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def report_of(row):
    autonomy = (row.get('state') or {}).get('autonomy') or {}
    return autonomy.get('active') or autonomy.get('last_result') or {}


def load_trial(path):
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    scans, sources, handover = {}, {}, None
    for index, row in enumerate(rows):
        scan = (row.get('lidar') or {}).get('scan')
        if isinstance(scan, dict) and isinstance(scan.get('ranges'), list):
            if scan['seq'] in scans and scan != scans[scan['seq']]:
                raise ValueError('same recorded scan sequence has different content')
            scans[scan['seq']] = scan
            sources[scan['seq']] = f'feedback.jsonl:{index+1}.lidar.scan'
        turn = report_of(row).get('turn') or {}
        event = (turn.get('route_events') or {}).get('second_handover')
        if handover is None and turn.get('route_stage') == 'second_orbit' and event:
            handover = (event, index)
    if handover is None:
        raise ValueError('no recorded second_orbit with a second_handover event')
    return rows, sorted(scans.values(), key=lambda s: s['seq']), sources, handover


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trial_dir', type=Path)
    parser.add_argument('--source-root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--json', type=Path, help='New evidence file; an existing file is never overwritten')
    args = parser.parse_args()
    if args.json is not None and args.json.exists():
        parser.error('JSON output already exists')
    source = args.source_root.resolve()/'web'/'vehicle-console'
    sys.path.insert(0, str(source))
    from maneuver_sequence import ManeuverSequence, SCAN_AGE_S
    from compact_target import _associated
    from autonomy_live import probe_clearance, ProbeClearanceError, MANEUVER_BODY_CLEARANCE_M

    feedback_path = args.trial_dir.resolve()/'feedback.jsonl'
    rows, scans, scan_sources, (handover, event_index) = load_trial(feedback_path)
    window = [s for s in scans if s['at_ms'] >= handover['scan']['at_ms']]
    tracker = ManeuverSequence._new_second_tracker()
    warmup, confirmed = [], None
    for scan in window:
        value = {**scan, 'received_at': scan['at_ms']/1000}
        target = tracker.update(value, value['received_at'])
        warmup.append({'seq': scan['seq'], 'at_ms': scan['at_ms'], 'source': scan_sources[scan['seq']],
                       'reason': tracker.reason, 'target': copy.deepcopy(target)})
        if target is not None and target.get('confirmed') and not target.get('tracking_only_boundary_gap'):
            confirmed = (value, target)
            break
    if confirmed is None:
        raise ValueError('saved window cannot naturally confirm a second target')
    initial_scan, initial_target = confirmed
    initial_now = initial_scan['received_at']
    eligible = []
    for index, row in enumerate(rows):
        report = report_of(row)
        turn = report.get('turn') or {}
        target = turn.get('second_target') or {}
        if (turn.get('phase') == 'drive' and turn.get('route_stage') == 'second_orbit'
                and turn.get('publication_at_ms', float('inf')) <= initial_scan['at_ms']
                and target.get('confirmed') is True and report.get('reason') is None):
            eligible.append((turn['publication_at_ms'], index, row, turn))
    if not eligible:
        raise ValueError('no recorded adopted second-orbit state at confirmed-window start')
    _, seed_index, seed_row, recorded_turn = max(eligible, key=lambda item: item[:2])
    seed_control = copy.deepcopy(seed_row['state']['status']['control'])
    recorded_target = recorded_turn['second_target']
    dt = (initial_scan['at_ms']-recorded_target['source_at_ms'])/1000
    if not 0 <= dt < SCAN_AGE_S or not _associated(initial_target, recorded_target, max(0., dt-.1)):
        raise ValueError('naturally confirmed support does not match the recorded second role')
    if (seed_control.get('armed') is not True or seed_control.get('motor') not in (1550, 1560)
            or recorded_turn.get('handover_observed') is not True
            or not isinstance(recorded_turn.get('first_pass_evidence'), dict)):
        raise ValueError('recorded seed has no adopted drive or first-pass evidence')

    motion = ManeuverSequence(initial_now, max_drive_s=16., continue_route=True)
    first_pass = copy.deepcopy(recorded_turn['first_pass_evidence'])
    first_pass['received_at'] = first_pass['source_at_ms']/1000
    first_drive = next(row['lidar']['scan'] for row in rows
                       if row['state']['status']['control'].get('motor', 1500) > 1500)
    provenance = {}

    def seed(name, value, origin):
        setattr(motion, name, value)
        provenance[name] = {'value': copy.deepcopy(value), 'source': origin}

    turn_origin = f'feedback.jsonl:{seed_index+1}.state.autonomy.active.turn'
    control_origin = f'feedback.jsonl:{seed_index+1}.state.status.control'
    seed('phase', 'drive', turn_origin+'.phase')
    seed('route_stage', 'second_orbit', turn_origin+'.route_stage')
    seed('handover_observed', True, turn_origin+'.handover_observed')
    seed('first_pass_evidence', first_pass, turn_origin+'.first_pass_evidence; receive clock mapped to publication seconds')
    seed('orbit_track_id', first_pass['track_id'], turn_origin+'.first_pass_evidence.track_id')
    seed('drive_since', first_drive['at_ms']/1000, 'first saved positive-motor scan publication; approximate drive origin')
    seed('second_orbit_since', handover['scan']['at_ms']/1000,
         f'feedback.jsonl:{event_index+1}.state.autonomy.active.turn.route_events.second_handover.scan.at_ms / 1000')
    seed('servo', seed_control['servo'], control_origin+'.servo (adopted value, not pending command)')
    seed('steering_target', seed_control['servo'], 'initial hold of recorded adopted steering')
    seed('natural_steering_target', seed_control['servo'], 'initial hold; recomputed from next update')
    seed('last_change', initial_now, 'conservative model start; elapsed prior steering-step time is unknown')
    seed('second_track_id', initial_target['track_id'], 'naturally confirmed replay tracker ID; not a physical identity claim')
    seed('second_target', copy.deepcopy(initial_target), 'natural warmup final observation')
    seed('_second_role_anchor', copy.deepcopy(initial_target), 'natural warmup final full observation')
    seed('second_target_handover_observed', True, 'recorded second_handover event, conditional initialization')
    for name, key in (('_route_center_ack', 'route_center_ack'),
                      ('right_exit_center_ack', 'right_exit_center_ack'),
                      ('second_center_ack', 'second_center_ack')):
        seed(name, copy.deepcopy(recorded_turn.get(key)), turn_origin+'.'+key)
    motion.second_tracker = tracker
    tracker.enable_close_maintenance(initial_target['track_id'])
    provenance['second_tracker'] = {'source': 'new runtime tracker updated only with listed warmup scans; maintenance enabled after role binding'}

    result = {'tool': 'conditional_second_orbit_replay', 'source_root': str(args.source_root.resolve()),
              'input': str(feedback_path), 'input_sha256': sha256(feedback_path),
              'tool_sha256': sha256(Path(__file__)),
              'source_sha256': {p.name: sha256(p) for p in sorted(source.glob('*.py'))},
              'warmup': warmup, 'seed': provenance, 'seed_control': seed_control,
              'scan_sources': scan_sources, 'ticks': [], 'frames': [], 'protocol_violations': [],
              'body_gate_reference': {'profile': 'maneuver', 'relaxed_turn_left': False,
                  'stop_distance_m': MANEUVER_BODY_CLEARANCE_M,
                  'meaning': 'Conservative body reference; service conditional left-turn relaxation is not modeled.'},
              'limitations': ['Conditional stage initialization, not whole-trial reconstruction.',
                  'Only raw saved scans; no interpolation or repair of missing frames.',
                  'Publication-clock intervals are the modeled receive clock; later ACKs are instant simulated adoption.',
                  'Runtime freshness; conservative 8 cm raw body reference with current support exclusions, not service conditional 4 cm relaxation.',
                  'Other unhandled quality faults stop this limited replay.',
                  'Old observations do not predict the new physical path once commands differ.']}
    command = {'motor': seed_control['motor'], 'servo': seed_control['servo']}
    control_seq, control_tick = seed_control['seq'], seed_control['tick']
    tick_origin, current_scan = initial_now, initial_scan
    center_seen = seed_control['servo'] == 1500 or recorded_turn.get('second_center_ack') is not None
    stopped = False
    print('CONDITIONAL second_orbit; actual saved scans, publication receive clock, instant modeled ACK')
    print('body reference: conservative 8 cm maneuver gate; service conditional 4 cm relaxation not modeled')
    print('seed', initial_scan['seq'], 'recorded control', seed_control['motor'], seed_control['servo'],
          'naturally confirmed', initial_target['track_id'], initial_target['confirmation_count'])
    print(' rec(s) seq stage          natural command prepare centerACK passed body reason')

    def tick(now, fresh):
        nonlocal command, control_seq, center_seen, stopped
        age = max(0., now-current_scan['received_at'])
        control_seq += 1
        control = {**seed_control, 'seq': control_seq,
                   'tick': control_tick+round((now-tick_origin)*1000), 'armed': True,
                   **command, 'command_acked': True, 'neutral_acked': command['motor'] == 1500,
                   'resume_acked': command['motor'] > 1500}
        if control['servo'] > 1500:
            center_seen = False
        elif control['servo'] == 1500:
            center_seen = True
        row = {'now': now, 'rec_s': (current_scan['at_ms']-first_drive['at_ms'])/1000,
               'seq': current_scan['seq'], 'fresh_scan': fresh, 'control': copy.deepcopy(control)}
        contact = motion.current_contact_target(current_scan, now)
        try:
            clearance = probe_clearance(current_scan, {'lidar': age, 'control': 0.},
                camera_required=False, clearance_profile='maneuver', relaxed_turn_left=False,
                body_contact_target=contact)
        except ProbeClearanceError as error:
            row['body_gate'] = error.clearance
            row['stop'] = str(error)
        except ValueError as error:
            row['stop'] = str(error)
        else:
            row['body_gate'] = clearance
            # This tool deliberately does not reconstruct the service's
            # quality-wait state from an incomplete event log.
            issues = clearance['quality_issues']
            if issues:
                row['stop'] = 'conditional_replay_quality_halt:'+','.join(issues)
            else:
                decision = motion.update(current_scan, age, now, control,
                                         quality_clear=True, quality_resume_ready=True)
                row['decision'] = decision
                if decision['servo'] < 1500 and control['servo'] >= 1500 and not center_seen:
                    result['protocol_violations'].append({'seq': current_scan['seq'], 'now': now,
                                                         'reason': 'new_right_without_adopted_center'})
                command = {'motor': decision['motor'], 'servo': decision['servo']}
                if decision.get('lock_requested') or decision.get('stop_requested'):
                    row['stop'] = decision.get('terminal_reason') or decision.get('reason')
                elif decision['phase'] != 'drive' or decision['route_stage'] != 'second_orbit':
                    row['window_complete'] = 'controller_left_second_orbit'
        if 'stop' in row:
            command = {'motor': 1500, 'servo': 1500}
        row['output_command'] = copy.deepcopy(command)
        result['ticks'].append(row)
        if fresh or 'stop' in row or 'window_complete' in row:
            result['frames'].append(row)
            d = row.get('decision') or {}
            print(f"{row['rec_s']:7.3f} {row['seq']:4d} {d.get('route_stage','-'):14s} "
                  f"{str(d.get('natural_steering_target','-')):>7s} {command['motor']}/{command['servo']} "
                  f"{bool(d.get('second_return_preparing'))!s:7s} {bool(d.get('second_center_ack'))!s:9s} "
                  f"{bool(d.get('second_pass_evidence'))!s:6s} "
                  f"{'STOP' if 'stop' in row else 'ok':4s} {row.get('stop') or d.get('reason')}")
        stopped = 'stop' in row or 'window_complete' in row

    tick(initial_now, True)
    now = initial_now
    for scan in window[len(warmup):]:
        if stopped:
            break
        incoming_now = scan['at_ms']/1000
        while round(now+.02, 9) < incoming_now-1e-9 and not stopped:
            now = round(now+.02, 9)
            tick(now, False)
        if stopped:
            break
        current_scan = {**scan, 'received_at': incoming_now}
        now = incoming_now
        tick(now, True)
    result['final'] = {'stop': result['ticks'][-1].get('stop'),
                       'window_complete': result['ticks'][-1].get('window_complete'),
                       'seq': current_scan['seq'], 'output_command': command}
    # Independently audit every remaining real scan even when decimation has
    # already ended the controller window. This does not resume control or
    # claim a trajectory: it asks only what the conservative 8 cm reference
    # would do with that exact frame and its current shape witnesses. No
    # assumed steering is used to grant the service's left-turn relaxation.
    body_auditor = ManeuverSequence(initial_now, max_drive_s=16., continue_route=True)
    body_auditor.phase = 'drive'
    body_auditor.drive_since = motion.drive_since
    body_audit = []
    for scan in window:
        value = {**scan, 'received_at': scan['at_ms']/1000}
        entry = {'seq': scan['seq'], 'source': scan_sources[scan['seq']]}
        try:
            contact = body_auditor.current_contact_target(value, value['received_at'])
            entry['clearance'] = probe_clearance(value, {'lidar': 0., 'control': 0.},
                camera_required=False, clearance_profile='maneuver', relaxed_turn_left=False,
                body_contact_target=contact)
        except ProbeClearanceError as error:
            entry['clearance'], entry['stop'] = error.clearance, str(error)
        except ValueError as error:
            entry['stop'] = str(error)
        body_audit.append(entry)
    result['independent_route_drive_body_audit'] = body_audit
    result['source_hashes_unchanged'] = all(sha256(source/name) == digest
        for name, digest in result['source_sha256'].items())
    result['input_hash_unchanged'] = sha256(feedback_path) == result['input_sha256']
    print('independent conservative body-reference stops', [(entry['seq'], entry['stop'])
                                             for entry in body_audit if 'stop' in entry])
    if args.json is not None:
        with args.json.open('x') as stream:
            json.dump(result, stream, indent=2)
            stream.write('\n')
    if result['protocol_violations']:
        raise SystemExit('right-before-center protocol violation in replay')
    if not result['source_hashes_unchanged'] or not result['input_hash_unchanged']:
        raise SystemExit('source or input changed during replay')


if __name__ == '__main__':
    main()
