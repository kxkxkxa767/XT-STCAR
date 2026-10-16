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
4.5 s; any lidar return in the body-wide band ahead stops the run (straight
ahead, plus the nominal arc while turning); deadline and Ctrl-C stop; the PWM
settings in force before the run are restored afterwards. A +-60 deg sector
(first run, 2026-10-13) also fired on marker posts the path clears by .7 m. Run on the vehicle (stdin over the SSH socket):
    python3 - --run-id steer-arc-YYYYMMDD-N --steer right --pwm 1200 \
        --placement-confirmed --execute < scripts/steering-arc-check.py
Without --execute it only prints a read-only state check.
"""
import argparse
import json
import math
from pathlib import Path
import re
import secrets
import signal
import time
import urllib.request

MANUAL_LIMITS = {'left': (1520, 1800), 'right': (1200, 1480)}
FORWARD_RANGE = (1540, 1600)  # user raised the cap to 1600 (2026-10-13)
GUARD_M = 2.0          # 1560 reached ~2 m/s on tile (run 3, 2026-10-13); stop plus coast
GUARD_HALF_WIDTH_M = .14+.25  # body side plus margin
# Along the arc only: turning speed is ~.6 m/s at 1550 (2026-10-16), and a 2 m arc
# look-ahead reached posts half a circle ahead that a short turn never meets.
ARC_REACH_M = 1.2
NOMINAL_PWM_PER_CURVATURE = 160  # agent.md section 5; only shapes the guard band


def guard_hit(ranges, turn, reach):
    """First return in the band the body sweeps within ``reach`` (car frame).

    Straight ahead always; while turning also the nominal arc (turn =
    (side +1 left / -1 right, radius)). The real radius lies between the two.
    """
    for i, r in enumerate(ranges):
        if not isinstance(r, (int, float)) or not .02 < r < 12:
            continue
        x, y = r*math.cos(math.radians(i)), -r*math.sin(math.radians(i))
        if .05 < x <= reach and abs(y) <= GUARD_HALF_WIDTH_M:
            return round(x, 3), round(y, 3)
        if turn is not None:
            side, radius = turn
            d = math.hypot(x, y-side*radius)
            ahead = (side*(math.atan2(y-side*radius, x)-math.atan2(-side*radius, 0.))) % (2*math.pi)
            if abs(d-radius) <= GUARD_HALF_WIDTH_M and ahead*radius <= min(reach, ARC_REACH_M):
                return round(x, 3), round(y, 3)
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--steer', choices=('left', 'right', 'straight'), required=True)
    parser.add_argument('--pwm', type=int, help='steering PWM for left/right')
    parser.add_argument('--forward', type=int, default=1550)
    parser.add_argument('--turn-s', type=float, default=1.2)
    parser.add_argument('--center-s', type=float, default=1.5)
    parser.add_argument('--kick-pwm', type=int, help='right runs only: brief opposite servo before 1500')
    parser.add_argument('--kick-s', type=float, default=0.)
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
    if args.kick_pwm is not None and not (args.steer == 'right' and 1500 < args.kick_pwm <= 1650
                                          and .05 <= args.kick_s <= .5):
        parser.error('kick: right runs only, 1501-1650, .05-.5 s (user: left deadband ends at 1580)')
    if args.turn_s+args.center_s+args.kick_s > 4.5:
        parser.error('powered total <= 4.5 s')
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

    before = request('/api/state')
    locked(before)
    lidar = request('/api/lidar')
    global GUARD_M
    GUARD_M += max(0, args.forward-1560)*.0125  # 2.5 m at 1600: faster, longer stop
    turn = (None if args.steer == 'straight' else
            ({'left': 1, 'right': -1}[args.steer], NOMINAL_PWM_PER_CURVATURE/abs(args.pwm-1500)))
    blocked = guard_hit(lidar['scan']['ranges'], turn, GUARD_M+.5)
    print(json.dumps({'read_only': not args.execute, 'settings': before['settings'],
                      'start_band_return': blocked, 'guard_m': GUARD_M, 'turn_band': turn}), flush=True)
    if not args.execute:
        return
    if blocked is not None:
        raise RuntimeError('swept band not clear before start; nothing sent')
    output = base/args.run_id
    output.mkdir(mode=0o700)  # Existing directory means this run is consumed.
    (output/'before.json').write_text(json.dumps(before, indent=2)+'\n')
    (output/'before-lidar.json').write_text(json.dumps(lidar)+'\n')
    original = dict(before['settings'])
    settings = dict(original, forward=args.forward)
    if args.steer != 'straight':
        settings[args.steer] = args.pwm
    if args.kick_pwm is not None:
        settings['left'] = args.kick_pwm
    servo_turn = args.pwm if args.steer != 'straight' else 1500
    turn_keys = ['up'] if args.steer == 'straight' else ['up', args.steer]
    result = {'completed': False, 'steer': args.steer, 'servo_turn': servo_turn, 'motor': args.forward,
              'turn_s': args.turn_s, 'center_s': args.center_s, 'kick_pwm': args.kick_pwm,
              'kick_s': args.kick_s if args.kick_pwm is not None else 0., 'phases': [],
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

    def hold(name, keys, motor, servo, seconds, allow_arm_pending=False, band=None):
        begin = time.monotonic()
        adopted_at = None
        while True:
            s, scan = observe()
            now = time.monotonic()
            c = s['status']['control']
            assert c['armed'] or (allow_arm_pending and now-begin < .25), 'bridge disarmed'
            if 'up' in keys:
                hit = guard_hit(scan['scan']['ranges'], band, GUARD_M)
                if hit is not None:
                    raise RuntimeError('swept-band guard: return at %s' % (hit,))
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
        hold('turn', turn_keys, args.forward, servo_turn, args.turn_s, band=turn)
        if args.kick_pwm is not None:
            hold('kick', ['up', 'left'], args.forward, args.kick_pwm, args.kick_s)
        hold('center_rolling', ['up'], args.forward, 1500, args.center_s)
        result['completed'] = True
    except BaseException as error:
        result['error'] = type(error).__name__+': '+str(error)
        raise
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        try:
            request('/api/control', {'op': 'stop'})
            # Keep recording the coast to rest (decel for the simulator model).
            coast_end = time.monotonic()+1.8
            while time.monotonic() < coast_end:
                try:
                    log.write(json.dumps({'elapsed_s': time.monotonic()-started, 'state': request('/api/state'),
                                          'lidar': request('/api/lidar'), 'after_stop': True})+'\n')
                    log.flush()
                except Exception as error:
                    log.write(json.dumps({'elapsed_s': time.monotonic()-started, 'observer_error': type(error).__name__})+'\n')
                time.sleep(.05)
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
