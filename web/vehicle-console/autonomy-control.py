#!/usr/bin/env python3
"""Vehicle terminal entry: status, bounded motion trials, target registration, or latched stop."""
import argparse
import json
import math
from pathlib import Path
import signal
import time
import urllib.error
import urllib.request
from autonomy_live import MIN_FORWARD_PWM, MAX_PWM, COAST_MAX_S, CONTROL_AGE_LIMIT_S
from turn_motion import parse_trial_goal, TRIAL_MOTOR, MAX_DRIVE_S, MAX_PRESTEER_S


class ProbeInterrupted(RuntimeError):
    def __init__(self, state, run):
        self.state, self.run = state, run
        result = state['autonomy']['last_result']
        super().__init__('probe interrupted: ' + str(result.get('reason') if result else 'no result'))


def run_probe(request, state, pwm, duration_ms, centering=False, stop_left_junction=False, formal_goal=False,
              turn_trial=False, placement_confirmed=False, max_turn_s=MAX_DRIVE_S, trial_goal_id=None):
    run = None
    try:
        op = 'turn_left_start' if turn_trial else 'stop_goal_start' if formal_goal else 'to_left_junction_start' if stop_left_junction else 'straight_start' if centering else 'probe_start'
        start = {'op': op, 'boot': state['boot'],
                      'epoch': state['autonomy']['epoch'], 'tick': state['status']['control']['tick'],
                      'pwm': pwm}
        if turn_trial:
            start.pop('pwm')
            start.update(placement_confirmed=placement_confirmed, max_drive_s=max_turn_s)
            if trial_goal_id is not None:
                start['goal_id'] = trial_goal_id
        elif not formal_goal:
            start['duration_ms'] = duration_ms
        run = request('/api/autonomy', start)
        sequence = 0
        end = time.monotonic() + duration_ms / 1000 + (COAST_MAX_S if centering else 0) + 1
        while time.monotonic() < end:
            latest = request('/api/state')
            active = latest['autonomy']['active']
            result = latest['autonomy']['last_result']
            if not active or active['run_id'] != run['run_id']:
                if latest['autonomy']['mode'] == 'manual':
                    raise RuntimeError('manual operator took control; automatic output cancelled')
                control = latest['status']['control']
                control_age = latest.get('ages', {}).get('control')
                if (latest['autonomy']['mode'] != 'locked' or control.get('armed') is not False
                        or control.get('motor') != 1500 or control.get('servo') != 1500
                        or type(control_age) not in (int, float) or not math.isfinite(control_age)
                        or not 0 <= control_age < CONTROL_AGE_LIMIT_S):
                    time.sleep(.02)
                    continue
                if not result or result.get('run_id') != run['run_id'] or not result.get('completed'):
                    raise ProbeInterrupted(latest, run)
                return latest, run
            sequence += 1
            try:
                if formal_goal:
                    # This reads the configured upstream file; the source's
                    # timestamps remain authoritative, independent of heartbeat.
                    request('/api/autonomy', {'op': 'stop_goal_refresh', 'boot': state['boot'], 'epoch': run['epoch']})
                request('/api/autonomy', {'op': 'heartbeat', 'boot': state['boot'],
                        'epoch': run['epoch'], 'run_id': run['run_id'], 'seq': sequence})
            except RuntimeError:
                latest = request('/api/state')
                if latest['autonomy']['active'] is None:
                    result = latest['autonomy']['last_result']
                    if result and result.get('run_id') == run['run_id']:
                        time.sleep(.02)
                        continue  # Any stop raced heartbeat; still await fresh neutral feedback.
                    raise ProbeInterrupted(latest, run) from None
                raise
            time.sleep(.04)
        raise RuntimeError('probe did not confirm neutral output')
    finally:
        if run is not None:
            try:
                request('/api/autonomy', {'op': 'cancel', 'boot': state['boot'],
                        'epoch': run['epoch'], 'run_id': run['run_id']})
            except Exception:
                pass  # Cannot cancel a newer manual owner; independent deadlines remain.


def left_turn_trial(request, state, max_seconds, placement_confirmed, goal_id=None):
    """One mid-segment trial; alignment evidence never becomes a cone task completion."""
    try:
        latest, run = run_probe(request, state, TRIAL_MOTOR, round((MAX_PRESTEER_S+max_seconds)*1000),
            centering=True, turn_trial=True, placement_confirmed=placement_confirmed, max_turn_s=max_seconds,
            trial_goal_id=goal_id)
    except ProbeInterrupted as error:
        latest, run = error.state, error.run
    result = latest['autonomy']['last_result']
    return {'reason': result['reason'], 'completed': False, 'turn': result.get('turn'),
            'control': latest['status']['control'], 'fresh_neutral_locked_confirmed': True,
            'operator_placement_confirmed': placement_confirmed, 'last_run_id': run['run_id'],
            'entry_confirmed': result.get('entry_confirmed', False),
            'observed_alignment': result.get('observed_alignment', False),
            'physical_steering_confirmed': False, 'physical_standstill_verified': False,
            'competition_navigation': False}


def straight_segment(request, state, pwm, max_seconds, expected_run_id=None, stop_left_junction=False):
    """One supervised session; a human/fault stop never causes another arm."""
    if expected_run_id is not None:
        old = state['autonomy']['last_result']
        if (not old or old.get('run_id') != expected_run_id or not old.get('completed')
                or old.get('reason') != 'probe_complete' or type(old.get('epoch')) is not int
                or old['epoch'] + 1 != state['autonomy']['epoch']
                or (state.get('last_stop') or {}).get('reason') != 'probe_complete'):
            raise RuntimeError('previous phase was stopped or restarted; refusing continuation')
    try:
        latest, run = run_probe(request, state, pwm, round(max_seconds*1000), centering=True, stop_left_junction=stop_left_junction)
        result = latest['autonomy']['last_result']
        reason = 'left_junction_reached' if result.get('reason') == 'left_junction_reached' else 'time_limit'
    except ProbeInterrupted as error:
        result = error.state['autonomy']['last_result']
        stops = {'probe_obstacle_in_straight_corridor': 'forward_clearance_limit',
                 'front_boundary_stop': 'front_boundary_stop'}
        if not result or result.get('reason') not in stops:
            raise
        run, reason = error.run, stops[result['reason']]
    return {'reason': reason, 'completed': result.get('completed', False),
            'motion_ticks': result.get('motion_ticks', 0), 'recovery_ticks': result.get('recovery_ticks', 0),
            'perception_recovering': result.get('perception_recovering', False),
            'coast_ticks': result.get('coast_ticks', 0), 'standstill_confirmed': result.get('standstill_confirmed', False),
            'coast_motion': result.get('coast_motion'),
            'steering_changes': result.get('steering_changes', 0), 'walls': result.get('walls'),
            'junction_confirmations': result.get('junction_confirmations', 0),
            'junction_geometry': result.get('junction_geometry'),
            'current_pwm': result.get('current_pwm'), 'approach_front_m': result.get('approach_front_m'),
            'boundary_closing_mps': result.get('boundary_closing_mps'),
            'boundary_time_to_clearance_s': result.get('boundary_time_to_clearance_s'),
            'pwm_ramp_changes': result.get('pwm_ramp_changes', 0),
            'last_run_id': run['run_id'], 'competition_navigation': False, 'wheel_motion_measured': False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['status', 'probe', 'straight', 'to-left-junction', 'turn-left',
        'trial-goal-status', 'trial-goal-register', 'goal-status', 'goal-straight', 'stop'])
    parser.add_argument('--pwm', type=int, default=MAX_PWM)
    parser.add_argument('--duration-ms', type=int, default=400)
    parser.add_argument('--max-seconds', type=float, help='drive limit: straight 1..30s, turn-left 1..10s')
    parser.add_argument('--expected-run-id', help='fence a new monitored phase to the previous normal stop')
    parser.add_argument('--execute', action='store_true', help='explicit motion request; motion commands otherwise only query state')
    parser.add_argument('--placement-confirmed', action='store_true', help='operator confirms stopped mid-segment placement for this one left trial')
    parser.add_argument('--goal-file', type=Path, help='Local target-only JSON for explicit trial-goal-register')
    parser.add_argument('--trial-goal-id', help='Use a registered trial intent; arbitrary point execution requires real pose')
    parser.add_argument('--access-file', type=Path, default=Path.home() / 'xt-stcar-console/access.json')
    args = parser.parse_args()
    access = json.loads(args.access_file.read_text())
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def request(path, data=None):
        headers = {'X-Control-Token': access['token']}
        if data is not None:
            headers['Content-Type'] = 'application/json'
        req = urllib.request.Request('http://127.0.0.1:' + str(access['port']) + path,
                                     headers=headers, data=json.dumps(data).encode() if data is not None else None)
        try:
            with opener.open(req, timeout=.15) as response:
                return json.load(response)
        except urllib.error.HTTPError as error:
            with error:
                detail = json.load(error).get('error', 'HTTP ' + str(error.code))
            raise RuntimeError(detail) from None

    if args.command == 'stop':
        request('/api/control', {'op': 'stop'})
        print('STOP requested; control locked. No automatic resume.')
        return
    state = request('/api/state')
    if 'autonomy' not in state:
        raise RuntimeError('vehicle console has no live autonomy interface')
    if args.command == 'trial-goal-status':
        print(json.dumps({'trial_goal': state['autonomy'].get('trial_goal'), 'motion_requested': False}, ensure_ascii=False, indent=2))
        return
    if args.command == 'trial-goal-register':
        if args.goal_file is None:
            raise RuntimeError('trial-goal-register requires explicit --goal-file')
        with args.goal_file.open('rb') as handle:
            source = handle.read(2049)
        if len(source) > 2048:
            raise RuntimeError('target-only registration must fit the existing HTTP limit')
        goal = parse_trial_goal(json.loads(source))
        # Only producer fields cross the wire; diagnostic parser fields do not.
        fields = {'schema_version', 'goal_id', 'source_kind', 'goal_type', 'frame',
                  'coordinate_convention', 'max_seconds', 'target_point_left_m'}
        data = {'op': 'trial_goal_register', 'boot': state['boot'], 'epoch': state['autonomy']['epoch'],
                'goal': {key: value for key, value in goal.items() if key in fields}}
        if len(json.dumps(data).encode()) > 2048:
            raise RuntimeError('target-only registration exceeds existing HTTP limit')
        print(json.dumps(request('/api/autonomy', data), ensure_ascii=False, indent=2))
        return
    if args.command == 'turn-left':
        if not args.execute:
            print(json.dumps({'healthy': state['healthy'], 'control': state['status']['control'],
                'turn_ready': state['autonomy'].get('turn_ready', False),
                'rejection': state['autonomy'].get('turn_rejection', 'turn_trial_interface_unavailable'),
                'preview': state['autonomy'].get('turn_preview'), 'motion_requested': False,
                'start_scope': 'operator_placed_mid_segment'}, ensure_ascii=False, indent=2))
            return
        if not args.placement_confirmed:
            raise RuntimeError('turn trial requires --placement-confirmed for this stopped mid-segment placement')
        if args.trial_goal_id is not None:
            registered = state['autonomy'].get('trial_goal')
            if not registered or registered.get('goal_id') != args.trial_goal_id:
                raise RuntimeError('trial_goal_not_registered')
            if not registered.get('execution_ready'):
                raise RuntimeError(registered.get('execution_rejection') or 'trial goal not ready')
        if not state['autonomy'].get('turn_ready'):
            raise RuntimeError(state['autonomy'].get('turn_rejection') or 'left turn trial not ready')
        limit = MAX_DRIVE_S if args.max_seconds is None else args.max_seconds
        if not 1 <= limit <= MAX_DRIVE_S:
            raise RuntimeError('left turn drive limit must be 1..10 seconds')
        def turn_interrupted(*_):
            raise KeyboardInterrupt
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, turn_interrupted)
        result = left_turn_trial(request, state, limit, args.placement_confirmed, args.trial_goal_id)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        raise SystemExit(1)  # No cone-entry/task completion has been certified.
    if args.command in ('goal-status', 'goal-straight'):
        if args.command == 'goal-status' or not args.execute:
            print(json.dumps({'healthy': state['healthy'], 'formal_ready': state['autonomy'].get('formal_ready', False),
                'rejection': state['autonomy'].get('formal_rejection', 'formal_interface_unavailable'),
                'final_stop_goal': state['autonomy'].get('final_stop_goal'), 'motion_requested': False}, ensure_ascii=False, indent=2))
            return
        request('/api/autonomy', {'op': 'stop_goal_refresh', 'boot': state['boot'], 'epoch': state['autonomy']['epoch']})
        state = request('/api/state')
        if not state['autonomy'].get('formal_ready'):
            raise RuntimeError(state['autonomy'].get('formal_rejection') or 'formal source not ready')
        duration = state['autonomy'].get('formal_max_drive_ms')
        if type(duration) is not int or not 1 <= duration <= 30000:
            raise RuntimeError('formal source has no bounded driving budget')
        def goal_interrupted(*_):
            raise KeyboardInterrupt
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, goal_interrupted)
        latest, _ = run_probe(request, state, args.pwm, duration, centering=True, formal_goal=True)
        print(json.dumps({'result': latest['autonomy']['last_result'], 'control': latest['status']['control'],
                         'competition_navigation': False}, ensure_ascii=False, indent=2))
        return
    if args.command == 'status' or not args.execute:
        print(json.dumps({'healthy': state['healthy'], 'control': state['status']['control'],
                          'settings': state['settings'], 'ages': state['ages'],
                          'autonomy': state['autonomy'], 'motion_requested': False}, ensure_ascii=False, indent=2))
        return
    if not MIN_FORWARD_PWM <= args.pwm <= MAX_PWM or not 100 <= args.duration_ms <= 500:
        raise RuntimeError(f'probe requires PWM {MIN_FORWARD_PWM}..{MAX_PWM} and duration 100..500 ms')
    if not state['autonomy']['probe_ready']:
        if args.command not in ('straight', 'to-left-junction'):
            raise RuntimeError(state['autonomy']['rejection'] or 'probe not ready')

    def interrupted(*_):
        raise KeyboardInterrupt

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, interrupted)
    if args.command in ('straight', 'to-left-junction'):
        if args.max_seconds is None:
            args.max_seconds = 30 if args.command == 'to-left-junction' else 1
        if not 1 <= args.max_seconds <= 30:
            raise RuntimeError('straight total limit must be 1..30 seconds')
        result = straight_segment(request, state, args.pwm, args.max_seconds,
                                  expected_run_id=args.expected_run_id, stop_left_junction=args.command == 'to-left-junction')
        print(json.dumps(result, ensure_ascii=False, indent=2))
    else:
        print('Bounded straight probe requested; this is NOT competition navigation.', flush=True)
        latest, _ = run_probe(request, state, args.pwm, args.duration_ms)
        print(json.dumps({'result': latest['autonomy']['last_result'], 'control': latest['status']['control'],
                          'healthy': latest['healthy']}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Interrupted; probe cancellation requested.')
        raise SystemExit(130)
    except Exception as error:
        print('CONTROL ERROR:', type(error).__name__, str(error))
        raise SystemExit(1)
