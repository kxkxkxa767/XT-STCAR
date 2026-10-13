#!/usr/bin/env python3
"""Vehicle-side one-shot manual steering check; never requests motor power."""
import argparse
import json
from pathlib import Path
import re
import secrets
import signal
import time
import urllib.request


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    if not re.fullmatch(r'[a-z0-9-]{8,80}', args.run_id):
        parser.error('run-id must be a unique lowercase evidence directory name')
    base = Path.home() / 'xt-stcar-console'
    access_path = base / 'access.json'
    if access_path.stat().st_mode & 0o077:
        raise RuntimeError('access file must be private')
    access = json.loads(access_path.read_text())
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def request(path, data=None):
        req = urllib.request.Request(
            'http://127.0.0.1:' + str(access['port']) + path,
            headers={'X-Control-Token': access['token'], 'Content-Type': 'application/json'},
            data=None if data is None else json.dumps(data).encode())
        with opener.open(req, timeout=1) as response:
            return json.load(response)

    def compact(s):
        return {k: s[k] for k in ('boot', 'healthy', 'owner', 'ages', 'settings', 'status')} | {
            'mode': s['autonomy']['mode'], 'active': s['autonomy']['active']}

    def locked(s):
        c = s['status']['control']
        assert s['healthy'] and s['owner'] is None
        assert s['autonomy']['active'] is None and s['autonomy']['mode'] == 'locked'
        assert not c['armed'] and (c['motor'], c['servo']) == (1500, 1500)
        assert 0 <= s['ages']['control'] < .2 and 0 <= s['ages']['lidar'] < .3

    before = request('/api/state')
    locked(before)
    if not args.execute:
        print(json.dumps({'read_only': True, 'state': compact(before)}), flush=True)
        return
    output = base / args.run_id
    output.mkdir(mode=0o700)  # Existing directory means this run is consumed.
    (output / 'before.json').write_text(json.dumps(compact(before), indent=2) + '\n')
    result = {'completed': False, 'motor_pwm': 1500, 'left_pwm': 1800,
              'right_pwm': 1200, 'seconds_each': 10, 'phases': [],
              'physical_wheel_angle_measured': False}
    client = 'static-steering-' + secrets.token_hex(12)
    boot = before['boot']
    seq = 0
    started = time.monotonic()
    log = (output / 'feedback.jsonl').open('x')

    def command(op, s, keys=()):
        nonlocal seq
        assert set(keys) <= {'left', 'right'}  # No forward/reverse keys.
        seq += 1
        return request('/api/control', {'op': op, 'boot': boot, 'client': client,
                       'seq': seq, 'tick': s['status']['control']['tick'], 'keys': list(keys)})

    def observe():
        s = request('/api/state')
        log.write(json.dumps({'elapsed_s': time.monotonic() - started, **compact(s)}) + '\n')
        log.flush()
        assert s['boot'] == boot and s['healthy'], 'restart or stale sensor'
        assert s['owner'] == client and s['autonomy']['mode'] == 'manual', 'control ownership lost'
        c = s['status']['control']
        assert c['motor'] == 1500, 'unexpected motor output'
        assert 0 <= s['ages']['control'] < .2 and 0 <= s['ages']['lidar'] < .3
        return s

    def hold(name, keys, pwm, seconds, allow_arm_pending=False):
        begin = time.monotonic()
        adopted_at = None
        last_tick = None
        ticks = 0
        while True:
            s = observe()
            now = time.monotonic()
            c = s['status']['control']
            assert c['armed'] or (allow_arm_pending and now - begin < .25), 'bridge disarmed'
            if c['armed'] and c['servo'] == pwm:
                if adopted_at is None:
                    adopted_at = now
                    print(name + ': adopted ' + str(pwm), flush=True)
                if last_tick != c['tick']:
                    ticks += 1
                last_tick = c['tick']
            elif adopted_at is not None:
                raise RuntimeError('adopted steering changed unexpectedly')
            elif now - begin > 1:
                raise RuntimeError('steering adoption timeout')
            if adopted_at is not None and now - adopted_at >= seconds:
                result['phases'].append({'name': name, 'servo': pwm,
                    'adopted_duration_s': now - adopted_at, 'fresh_ticks': ticks})
                return s
            command('drive', s, keys)
            time.sleep(.04)

    def expired(*_):
        raise RuntimeError('static check deadline or interrupt')

    for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGALRM):
        signal.signal(sig, expired)
    signal.setitimer(signal.ITIMER_REAL, 32)
    try:
        settings = {**before['settings'], 'left': 1800, 'right': 1200}
        assert request('/api/settings', settings) == settings
        fresh = request('/api/state')
        locked(fresh)
        assert fresh['boot'] == boot, 'server restarted before arm'
        command('arm', fresh)
        hold('initial_center', (), 1500, .3, allow_arm_pending=True)
        hold('left_10s', ('left',), 1800, 10)
        hold('center_between', (), 1500, 1)
        hold('right_10s', ('right',), 1200, 10)
        hold('final_center', (), 1500, .3)
        result['completed'] = True
    except BaseException as error:
        result['error'] = type(error).__name__ + ': ' + str(error)
        raise
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        try:
            request('/api/control', {'op': 'stop'})
            after = []
            for _ in range(6):
                time.sleep(.18)
                s = request('/api/state')
                locked(s)
                if after:
                    assert s['status']['control']['tick'] > after[-1]['status']['control']['tick']
                after.append(compact(s))
            (output / 'after.json').write_text(json.dumps(after, indent=2) + '\n')
            result['fresh_locked_samples'] = len(after)
        except BaseException as error:
            result['completed'] = False
            result['stop_verification_error'] = type(error).__name__ + ': ' + str(error)
        log.close()
        (output / 'summary.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result), flush=True)
    if not result['completed']:
        raise RuntimeError('static check incomplete; see evidence')


if __name__ == '__main__':
    main()
