#!/usr/bin/env python3
"""One route run of the standalone program `route-engine race` on the car.

  python3 race-trial.py --evidence NEW_DIR --program PATH --bridge PATH [--max-drive-s 40]
      [--initial-presteer-pwm 1670] [--ready-hold-s 1] [--program-sha256 H] [--bridge-sha256 H]
      [--check-only] [--vision-camera /dev/video20]

Runs on the car, as often as wanted (user, 10-24: “不限次数”), one run at a
time (BASE/race-trial.lock). Each run gets a new evidence directory: NEW_DIR,
else NEW_DIR-2, NEW_DIR-3, ... (an existing one is never reused); its path is
the first line printed. A round's packaged copy has FIXED_ARGS set by its
builder and takes no arguments. The
program and bridge must match the given hashes before anything else. Before: the
console must report a fresh locked neutral. Road admission is owned by the Rust
race program with its live sensor stream. Then the console service is stopped (the
program needs /dev/car and /dev/laser to itself), both devices must be free,
and the program runs with --start ready (or --start never with --check-only)
and writes the trial helper's evidence files into NEW_DIR/run/; its terminal
output goes to NEW_DIR/race-stdout.txt. Afterwards the console service is
started again whatever happened, and six fresh locked-neutral samples are
required (state in NEW_DIR/after-console.json). The result is
NEW_DIR/helper-result.json. The console's access token is read here and
never printed.

The Rust capture-camera process publishes raw frames to the start-lane and
traffic-light workers. Camera acquisition and race admission start concurrently.
"""
import argparse
import fcntl
import hashlib
import itertools
import json
import math
from pathlib import Path
import subprocess
import os
import sys
import time
import urllib.request

BASE = Path('/home/bianbu/xt-stcar-console')
SERVICE = 'xt-stcar-console.service'
FIXED_ARGS = None  # a round's packaged copy: its arguments, set by the builder
VISION_STOP_S = 10.


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
    parser.add_argument('--ready-hold-s', default='3')
    parser.add_argument('--program-sha256')
    parser.add_argument('--bridge-sha256')
    parser.add_argument('--check-only', action='store_true')
    # Older packaged helpers may carry unused arguments for their former image source.
    for option in ('--vision-script','--vision-script-sha256','--vision-model','--vision-model-sha256'):
        parser.add_argument(option, help=argparse.SUPPRESS)
    parser.add_argument('--vision-camera', default='/dev/video20')
    parser.add_argument('--light-stop', action='store_true')
    parser.add_argument('--ros2-setup', help='source this ROS2 setup.bash before launching the Rust program')
    if FIXED_ARGS is not None and sys.argv[1:]:
        raise SystemExit("this round's helper takes no arguments")
    args = parser.parse_args(FIXED_ARGS)
    checks = [(args.program, args.program_sha256), (args.bridge, args.bridge_sha256)]
    for path, expected in checks:
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

    devices = ('/dev/car', '/dev/laser')

    def busy():
        return [d for d in devices if subprocess.run(['fuser', d], capture_output=True).returncode != 1]

    pre = state(f'?trial_mode=turn-cone&initial_presteer_pwm={args.initial_presteer_pwm}&continue_route=true')
    (out/'before-console.json').write_text(json.dumps(pre, indent=2)+'\n')
    if not neutral(pre):
        raise SystemExit('console not at a fresh locked neutral; nothing started')
    # The race program owns admission using the fresh driver stream. The
    # console's older road preview is retained as evidence, not a second gate.
    result = {'evidence_dir': str(out), 'check_only': args.check_only,
              'start_admission': 'rust_race_live_sensors'}
    vision = None
    frame_file = (Path('/dev/shm') / f'xt-stcar-race-{os.getpid()}.bgr') if Path('/dev/shm').is_dir() else out/'camera.bgr'
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
        # scripts/t-ground-drag.py --apply: this floor's measured load, the learner's starting point.
        surface_initial = BASE/'surface-initial.json'
        if surface_initial.is_file():
            result['surface_initial'] = json.loads(surface_initial.read_text())
            command += ['--surface-initial-load', str(float(result['surface_initial']['extra_load_mps2']))]
        command += ['--frame-shm', str(frame_file)]
        camera_command = [args.program, 'capture-camera', args.vision_camera, str(frame_file), str(out/'vision')]
        try:
            with (out/'vision-stderr.txt').open('w') as stderr:
                vision = subprocess.Popen(camera_command, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                          stderr=stderr, text=True)
        except OSError as error:
            result['camera_spawn_error'] = str(error)
        result.update(vision_backend='rust_v4l2', light_stop=True)
        started = time.monotonic()
        result['program_started_monotonic_s'] = started
        if args.ros2_setup:
            # Startup environment only. Control and hardware nodes remain Rust.
            # Values are positional shell arguments, never interpolated code.
            setup = Path(args.ros2_setup)
            if not setup.is_file():
                raise RuntimeError(f'ROS2 setup file missing: {setup}')
            command = ['bash', '-c',
                       'source "$1" || exit; export ROS_DOMAIN_ID=217 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOG_DIR="$2"; shift 2; exec "$@"',
                       'stcar-ros2', str(setup), str(out/'ros-log'), *command]
            result['ros2_setup'] = str(setup)
        with (out/'race-stdout.txt').open('w') as stdout:
            p = subprocess.run(command, stdout=stdout, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, timeout=120)
        result.update(program_exit=p.returncode, program_elapsed_s=time.monotonic()-started)
        time.sleep(1)
        result['devices_busy_after_run'] = busy()
    finally:
        if vision is not None:
            if vision.poll() is None:
                vision.terminate()
                try:
                    vision.wait(VISION_STOP_S)
                except subprocess.TimeoutExpired:
                    vision.kill()
                    vision.wait(5)
            tail = [json.loads(line) for line in vision.stdout.read().splitlines() if line.startswith('{')]
            result['vision_ready'] = next((row for row in tail if row.get('kind') == 'ready'), None)
            result.update(vision_exit=vision.returncode,
                          vision_summary=next((row for row in tail if row.get('kind') == 'summary'), None))
        for path in (frame_file, frame_file.with_suffix('.bgr.tmp'), frame_file.with_suffix('.bgr.lamp.sock')):
            try:
                path.unlink(missing_ok=True)
            except OSError as error:
                result['camera_cleanup_error'] = str(error)
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
