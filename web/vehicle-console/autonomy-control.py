#!/usr/bin/env python3
"""Vehicle terminal entry: status, bounded straight probe, or latched stop."""
import argparse
import json
from pathlib import Path
import signal
import time
import urllib.error
import urllib.request


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['status', 'probe', 'stop'])
    parser.add_argument('--pwm', type=int, default=1530)
    parser.add_argument('--duration-ms', type=int, default=400)
    parser.add_argument('--execute', action='store_true', help='explicit physical probe; otherwise read-only')
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
            detail = json.load(error).get('error', 'HTTP ' + str(error.code))
            raise RuntimeError(detail) from None

    if args.command == 'stop':
        request('/api/control', {'op': 'stop'})
        print('STOP requested; control locked. No automatic resume.')
        return
    state = request('/api/state')
    if 'autonomy' not in state:
        raise RuntimeError('vehicle console has no live autonomy interface')
    if args.command == 'status' or not args.execute:
        print(json.dumps({'healthy': state['healthy'], 'control': state['status']['control'],
                          'settings': state['settings'], 'ages': state['ages'],
                          'autonomy': state['autonomy'], 'motion_requested': False}, ensure_ascii=False, indent=2))
        return
    if not 1501 <= args.pwm <= 1530 or not 100 <= args.duration_ms <= 500:
        raise RuntimeError('probe requires PWM 1501..1530 and duration 100..500 ms')
    if not state['autonomy']['probe_ready']:
        raise RuntimeError(state['autonomy']['rejection'] or 'probe not ready')
    run = None

    def interrupted(*_):
        raise KeyboardInterrupt

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, interrupted)
    try:
        # Retain the preflight epoch. A stop between read and start must reject start.
        run = request('/api/autonomy', {'op': 'probe_start', 'boot': state['boot'],
                      'epoch': state['autonomy']['epoch'], 'tick': state['status']['control']['tick'],
                      'pwm': args.pwm, 'duration_ms': args.duration_ms})
        print('Bounded straight probe started; this is NOT competition navigation.', flush=True)
        sequence = 0
        end = time.monotonic() + args.duration_ms / 1000 + 1
        while time.monotonic() < end:
            latest = request('/api/state')
            active = latest['autonomy']['active']
            if not active or active['run_id'] != run['run_id']:
                result = latest['autonomy']['last_result']
                # Wait for serial owner confirmation, not just the HTTP stop acknowledgement.
                if latest['autonomy']['mode'] == 'manual':
                    raise RuntimeError('manual operator took control; automatic output cancelled')
                if latest['status']['control']['armed']:
                    time.sleep(.02)
                    continue
                print(json.dumps({'result': result, 'control': latest['status']['control'],
                                  'healthy': latest['healthy']}, ensure_ascii=False, indent=2))
                if not result or result.get('run_id') != run['run_id'] or not result.get('completed'):
                    raise RuntimeError('probe stopped without successful completion')
                return
            sequence += 1
            request('/api/autonomy', {'op': 'heartbeat', 'boot': state['boot'],
                    'epoch': run['epoch'], 'run_id': run['run_id'], 'seq': sequence})
            time.sleep(.04)
        raise RuntimeError('probe did not confirm neutral output')
    finally:
        if run is not None:
            # Compare-and-cancel cannot stop a newer manual controller after takeover.
            try:
                request('/api/autonomy', {'op': 'cancel', 'boot': state['boot'],
                        'epoch': run['epoch'], 'run_id': run['run_id']})
            except Exception:
                pass  # Independent vehicle and Rust deadlines remain active.


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Interrupted; probe cancellation requested.')
        raise SystemExit(130)
    except Exception as error:
        print('CONTROL ERROR:', type(error).__name__, str(error))
        raise SystemExit(1)
