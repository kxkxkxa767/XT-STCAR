#!/usr/bin/env python3
"""Vehicle-side one-shot low-speed steering arc, then return to 1500 (not yet run).

Purpose: rolling curvature at one steering PWM (1200 right is still
uncalibrated) and the residual curvature after commanding 1500 on the ground
(user, 2026-10-13: after a turn, 1500 leaves the wheels toward that turn;
lifted they are straight). The curvature itself is measured afterwards with
scripts/trial-map.py on the copied evidence directory (it reads before.json,
before-lidar.json, feedback.jsonl, after.json).

This drives through the manual control path, so the route's autonomy gates
(8 cm body gate, bridge lock checks of the route, ...) do not apply. Hence:
the operator clears about 3 m round the car, watches, and confirms placement
with --placement-confirmed; forward only (no reverse key); powered time at most
4.5 s; any lidar return closer than the guard in the front +-60 deg stops the
run; deadline and Ctrl-C stop; the PWM settings in force before the run are
restored afterwards. Run on the vehicle (stdin over the SSH socket):
    python3 - --run-id steer-arc-YYYYMMDD-N --steer right --pwm 1200 \
        --placement-confirmed --execute < scripts/steering-arc-check.py
Without --execute it only prints a read-only state check.
"""
import argparse
import json
from pathlib import Path
import re
import secrets
import signal
import time
import urllib.request

MANUAL_LIMITS = {'left': (1520, 1800), 'right': (1200, 1480)}
FORWARD_RANGE = (1540, 1560)
GUARD_M = 1.4          # ~1 m coasting at 1 m/s and .5 m/s^2, plus the .2 m front
GUARD_HALF_DEG = 60


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--steer', choices=('left', 'right', 'straight'), required=True)
    parser.add_argument('--pwm', type=int, help='steering PWM for left/right')
    parser.add_argument('--forward', type=int, default=1550)
    parser.add_argument('--turn-s', type=float, default=1.2)
    parser.add_argument('--center-s', type=float, default=1.5)
    parser.add_argument('--placement-confirmed', action='store_true')
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    if not re.fullmatch(r'[a-z0-9-]{8,80}', args.run_id):
        parser.error('run-id must be a unique lowercase evidence directory name')
    if args.steer == 'straight':
        if args.pwm is not None:
            parser.error('straight takes no --pwm')
    elif args.pwm is None or not MANUAL_LIMITS[args.steer][0] <= args.pwm <= MANUAL_LIMITS[args.steer][1]:
        parser.error('--pwm outside %s %s' % (args.steer, MANUAL_LIMITS[args.steer]))
    if not FORWARD_RANGE[0] <= args.forward <= FORWARD_RANGE[1]:
        parser.error('--forward outside %s' % (FORWARD_RANGE,))
    if not (.3 <= args.turn_s <= 2.5 and .5 <= args.center_s <= 2. and args.turn_s+args.center_s <= 4.5):
        parser.error('turn .3-2.5 s, centre .5-2 s, powered total <= 4.5 s')
    if args.execute and not args.placement_confirmed:
        parser.error('--execute needs --placement-confirmed (operator: area clear, watching)')
    base = Path.home()/'xt-stcar-console'
    access_path = base/'access.json'
    if access_path.stat().st_mode & 0o077:
        raise RuntimeError('access file must be private')
    access = json.loads(access_path.read_text())
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def request(path, data=None):
        req = urllib.request.Request(
            'http://127.0.0.1:'+str(access['port'])+path,
            headers={'X-Control-Token': access['token'], 'Content-Type': 'application/json'},
            data=None if data is None else json.dumps(data).encode())
        with opener.open(req, timeout=1) as response:
            return json.load(response)

    def locked(s):
        c = s['status']['control']
        assert s['healthy'] and s['owner'] is None
        assert s['autonomy']['active'] is None and s['autonomy']['mode'] == 'locked'
        assert not c['armed'] and (c['motor'], c['servo']) == (1500, 1500)
        assert 0 <= s['ages']['control'] < .2 and 0 <= s['ages']['lidar'] < .3

    def front_min(lidar):
        ranges = lidar['scan']['ranges']
        values = [ranges[i % 360] for i in range(-GUARD_HALF_DEG, GUARD_HALF_DEG+1)]
        values = [r for r in values if isinstance(r, (int, float)) and r > .02]
        return min(values) if values else None

    before = request('/api/state')
    locked(before)
    lidar = request('/api/lidar')
    front = front_min(lidar)
    print(json.dumps({'read_only': not args.execute, 'settings': before['settings'],
                      'front_min_m': front, 'guard_m': GUARD_M}), flush=True)
    if not args.execute:
        return
    if front is None or front < GUARD_M+.5:
        raise RuntimeError('front sector not clear before start; nothing sent')
    output = base/args.run_id
    output.mkdir(mode=0o700)  # Existing directory means this run is consumed.
    (output/'before.json').write_text(json.dumps(before, indent=2)+'\n')
    (output/'before-lidar.json').write_text(json.dumps(lidar)+'\n')
    original = dict(before['settings'])
    settings = dict(original, forward=args.forward)
    if args.steer != 'straight':
        settings[args.steer] = args.pwm
    servo_turn = args.pwm if args.steer != 'straight' else 1500
    turn_keys = ['up'] if args.steer == 'straight' else ['up', args.steer]
    result = {'completed': False, 'steer': args.steer, 'servo_turn': servo_turn, 'motor': args.forward,
              'turn_s': args.turn_s, 'center_s': args.center_s, 'phases': [],
              'curvature_measured_here': False, 'analysis': 'scripts/trial-map.py on this directory'}
    client = 'steer-arc-'+secrets.token_hex(12)
    boot = before['boot']
    seq = 0
    started = time.monotonic()
    log = (output/'feedback.jsonl').open('x')

    def command(op, s, keys=()):
        nonlocal seq
        assert set(keys) <= {'up', 'left', 'right'}  # never the reverse key
        seq += 1
        return request('/api/control', {'op': op, 'boot': boot, 'client': client,
                       'seq': seq, 'tick': s['status']['control']['tick'], 'keys': list(keys)})

    def observe():
        s = request('/api/state')
        scan = request('/api/lidar')
        log.write(json.dumps({'elapsed_s': time.monotonic()-started, 'state': s, 'lidar': scan})+'\n')
        log.flush()
        assert s['boot'] == boot and s['healthy'], 'restart or stale sensor'
        assert s['owner'] == client and s['autonomy']['mode'] == 'manual', 'control ownership lost'
        assert 0 <= s['ages']['control'] < .2 and 0 <= s['ages']['lidar'] < .3
        return s, scan

    def hold(name, keys, motor, servo, seconds, allow_arm_pending=False):
        begin = time.monotonic()
        adopted_at = None
        while True:
            s, scan = observe()
            now = time.monotonic()
            c = s['status']['control']
            assert c['armed'] or (allow_arm_pending and now-begin < .25), 'bridge disarmed'
            if 'up' in keys:
                near = front_min(scan)
                if near is None or near < GUARD_M:
                    raise RuntimeError('front guard: return at %s m' % near)
            if c['armed'] and c['servo'] == servo and c['motor'] == motor:
                if adopted_at is None:
                    adopted_at = now
                    result['phases'].append({'name': name, 'motor': motor, 'servo': servo,
                                             'adopted_elapsed_s': now-started, 'adopted_scan_seq': scan['scan']['seq']})
            elif adopted_at is not None:
                raise RuntimeError('adopted command changed unexpectedly')
            elif now-begin > 1:
                raise RuntimeError('command adoption timeout')
            if adopted_at is not None and now-adopted_at >= seconds:
                result['phases'][-1].update(end_elapsed_s=now-started, end_scan_seq=scan['scan']['seq'])
                return s
            command('drive', s, keys)
            time.sleep(.04)

    def expired(*_):
        raise RuntimeError('steering arc deadline or interrupt')

    for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGALRM):
        signal.signal(sig, expired)
    signal.setitimer(signal.ITIMER_REAL, 12)
    try:
        assert request('/api/settings', settings) == settings
        fresh = request('/api/state')
        locked(fresh)
        assert fresh['boot'] == boot, 'server restarted before arm'
        command('arm', fresh)
        hold('initial_center', (), 1500, 1500, .3, allow_arm_pending=True)
        hold('turn', turn_keys, args.forward, servo_turn, args.turn_s)
        hold('center_rolling', ['up'], args.forward, 1500, args.center_s)
        result['completed'] = True
    except BaseException as error:
        result['error'] = type(error).__name__+': '+str(error)
        raise
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        try:
            request('/api/control', {'op': 'stop'})
            time.sleep(1.5)  # let it coast to rest before the at-rest samples
            after = []
            for _ in range(6):
                time.sleep(.18)
                s = request('/api/state')
                locked(s)
                if after:
                    assert s['status']['control']['tick'] > after[-1]['state']['status']['control']['tick']
                after.append({'state': s, 'lidar': request('/api/lidar')})
            (output/'after.json').write_text(json.dumps(after, indent=2)+'\n')
            result['fresh_locked_samples'] = len(after)
        except BaseException as error:
            result['completed'] = False
            result['stop_verification_error'] = type(error).__name__+': '+str(error)
        try:
            restored = request('/api/settings', original)
            result['settings_restored'] = restored == original
        except BaseException as error:
            result['settings_restored'] = False
            result['settings_restore_error'] = type(error).__name__+': '+str(error)
        log.close()
        (output/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result), flush=True)
    if not result['completed']:
        raise RuntimeError('steering arc incomplete; see evidence')


if __name__ == '__main__':
    main()
