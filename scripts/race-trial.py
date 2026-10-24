#!/usr/bin/env python3
"""One route run of the standalone program `route-engine race` on the car.

  python3 race-trial.py --evidence NEW_DIR --program PATH --bridge PATH [--max-drive-s 40]
      [--initial-presteer-pwm 1670] [--ready-hold-s 1] [--program-sha256 H] [--bridge-sha256 H]
      [--check-only]

Runs on the car, as often as wanted (user, 10-24: “不限次数”), one run at a
time (BASE/race-trial.lock). Each run gets a new evidence directory: NEW_DIR,
else NEW_DIR-2, NEW_DIR-3, ... (an existing one is never reused); its path is
the first line printed. A round's packaged copy has FIXED_ARGS set by its
builder and takes no arguments. The
program and bridge must match the given hashes before anything else. Before: the
console must report a fresh locked neutral and its own (Python) start check
for the explicit route must pass. Then the console service is stopped (the
program needs /dev/car and /dev/laser to itself), both devices must be free,
and the program runs with --start ready (or --start never with --check-only)
and writes the trial helper's evidence files into NEW_DIR/run/; its terminal
output goes to NEW_DIR/race-stdout.txt. Afterwards the console service is
started again whatever happened, and six fresh locked-neutral samples are
required (state in NEW_DIR/after-console.json). The result is
NEW_DIR/helper-result.json. The console's access token is read here and
never printed.
"""
import argparse
import fcntl
import hashlib
import itertools
import json
from pathlib import Path
import subprocess
import sys
import time
import urllib.request

BASE = Path('/home/bianbu/xt-stcar-console')
SERVICE = 'xt-stcar-console.service'
FIXED_ARGS = None  # a round's packaged copy: its arguments, set by the builder


def fresh_dir(base):
    """Create and return the first of BASE, BASE-2, BASE-3, ... that does not exist yet."""
    for n in itertools.count(1):
        out = base if n == 1 else base.with_name(f'{base.name}-{n}')
        try:
            out.mkdir(mode=0o700)
            return out
        except FileExistsError:
            continue


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--evidence', required=True)
    parser.add_argument('--program', required=True)
    parser.add_argument('--bridge', required=True)
    parser.add_argument('--max-drive-s', default='40')
    parser.add_argument('--initial-presteer-pwm', default='1670')
    parser.add_argument('--ready-hold-s', default='1')
    parser.add_argument('--program-sha256')
    parser.add_argument('--bridge-sha256')
    parser.add_argument('--check-only', action='store_true')
    if FIXED_ARGS is not None and sys.argv[1:]:
        raise SystemExit("this round's helper takes no arguments")
    args = parser.parse_args(FIXED_ARGS)
    for path, expected in ((args.program, args.program_sha256), (args.bridge, args.bridge_sha256)):
        if expected is not None and hashlib.sha256(Path(path).read_bytes()).hexdigest() != expected:
            raise SystemExit(f'{path} does not match its expected sha256; nothing started')
    lock = (BASE/'race-trial.lock').open('w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise SystemExit('another run is in progress; nothing started') from None
    out = fresh_dir(Path(args.evidence))
    print(json.dumps({'evidence_dir': str(out)}), flush=True)
    access = json.loads((BASE/'access.json').read_text())
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def state(query=''):
        req = urllib.request.Request(f"http://127.0.0.1:{access['port']}/api/state{query}",
                                     headers={'X-Control-Token': access['token']})
        with opener.open(req, timeout=2) as r:
            return json.load(r)

    def neutral(s):
        c = s['status']['control']
        return (s['healthy'] is True and s['owner'] is None and s['autonomy']['active'] is None
                and s['autonomy']['mode'] == 'locked'
                and c['armed'] is False and c['motor'] == c['servo'] == 1500 and 0 <= s['ages']['control'] < .2)

    def busy():
        return [d for d in ('/dev/car', '/dev/laser')
                if subprocess.run(['fuser', d], capture_output=True).returncode != 1]

    pre = state(f'?trial_mode=turn-cone&initial_presteer_pwm={args.initial_presteer_pwm}&continue_route=true')
    (out/'before-console.json').write_text(json.dumps(pre, indent=2)+'\n')
    if not neutral(pre):
        raise SystemExit('console not at a fresh locked neutral; nothing started')
    if not args.check_only and not (pre['autonomy'].get('turn_ready') is True
                                    and pre['autonomy'].get('continue_route') is True
                                    and (pre['autonomy'].get('turn_preview') or {}).get('continue_route') is True):
        raise SystemExit(f"console start check not ready ({pre['autonomy'].get('turn_rejection')}); nothing started")
    result = {'evidence_dir': str(out), 'check_only': args.check_only}
    subprocess.run(['systemctl', '--user', 'stop', SERVICE], check=True, timeout=30)
    try:
        deadline = time.monotonic()+15
        while busy() and time.monotonic() < deadline:
            time.sleep(.2)
        result['devices_busy_before_run'] = busy()
        if result['devices_busy_before_run']:
            raise RuntimeError('devices still busy after stopping the console; not started')
        command = [args.program, 'race', '--bridge', args.bridge, '--out', str(out/'run'),
                   '--max-drive-s', args.max_drive_s, '--initial-presteer-pwm', args.initial_presteer_pwm,
                   '--status-s', '.5', '--max-wait-s', '20']
        command += ['--start', 'never'] if args.check_only else ['--start', 'ready', '--ready-hold-s', args.ready_hold_s]
        started = time.monotonic()
        with (out/'race-stdout.txt').open('w') as stdout:
            p = subprocess.run(command, stdout=stdout, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, timeout=120)
        result.update(program_exit=p.returncode, program_elapsed_s=time.monotonic()-started)
        time.sleep(1)
        result['devices_busy_after_run'] = busy()
    finally:
        subprocess.run(['systemctl', '--user', 'start', SERVICE], timeout=30)
        final, deadline = [], time.monotonic()+90
        while len(final) < 6 and time.monotonic() < deadline:
            try:
                s = state()
                if neutral(s) and (not final or s['status']['control']['tick'] > final[-1]['status']['control']['tick']):
                    final.append(s)
                elif not neutral(s):
                    final = []
            except Exception:  # noqa: BLE001 - the service is still starting
                pass
            time.sleep(.3)
        (out/'after-console.json').write_text(json.dumps(final, indent=2)+'\n')
        summary = out/'run'/'summary.json'
        run = json.loads(summary.read_text()) if summary.is_file() else {}
        result.update(console_neutral_samples=len(final), console_healthy=final[-1]['healthy'] if final else None,
                      reason=run.get('reason'), max_actual_gap_s=run.get('max_actual_gap_s'),
                      max_tick_compute_s=run.get('max_tick_compute_s'),
                      service_active=subprocess.run(['systemctl', '--user', 'is-active', SERVICE],
                                                    capture_output=True, text=True).stdout.strip())
        (out/'helper-result.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result))
        if len(final) != 6:
            raise SystemExit('console fresh neutral lock not confirmed after the run; check the car')


if __name__ == '__main__':
    main()
