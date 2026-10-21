#!/usr/bin/env python3
"""A stand-in for vehicle-bridge (Mac tests of `route-engine race`; never the car).

  fake-vehicle-bridge.py control | lidar

Environment: FAKE_TRIAL (a recorded trial directory, required for lidar) and
FAKE_SHARED (a scratch directory both modes share).

control: the bridge's own teleop guard (crates/robot-core/src/teleop.rs):
arm/drive/stop with its sequence, tick and PWM checks, the 150 ms gap and
300 ms command lease, stop when the input closes; a status row every 20 ms on
its own millisecond clock. The first powered drive is noted in FAKE_SHARED.
lidar: the trial's saved scans (feedback.jsonl), renumbered and re-stamped on
this clock at 10 Hz: the scans before its recorded drive loop until the control
side has driven, then the recorded drive scans follow at their recorded
spacing (open loop, like route-replay.py). Offline only.
"""
import json
import os
from pathlib import Path
import sys
import threading
import time

START = time.monotonic()


def now_ms():
    return int((time.monotonic()-START)*1000)


def emit(row):
    sys.stdout.write(json.dumps(row)+'\n')
    sys.stdout.flush()


def control():
    shared = Path(os.environ.get('FAKE_SHARED', '.'))
    status = {'tick': 0, 'seq': 0, 'armed': False, 'motor': 1500, 'servo': 1500, 'reason': 'locked'}
    state = {'received': 0, 'last': 0, 'stopped_at': None, 'closed': False}
    lock = threading.Lock()

    def stop(reason):
        state['stopped_at'] = status['tick']
        status.update(armed=False, motor=1500, servo=1500, reason=reason)

    def advance(now):
        status['tick'] = now
        if now < state['last'] or (status['armed'] and now-state['last'] > 150):
            stop('control_gap')
        state['last'] = now
        if status['armed'] and now-state['received'] >= 300:
            stop('heartbeat_timeout')

    def apply(req, now):
        advance(now)
        if req.get('op') == 'stop':
            status['seq'] = max(status['seq'], req['seq'])
            stop('operator_stop')
            return
        if req['seq'] <= status['seq'] or req['tick'] > now or now-req['tick'] >= 250:
            stop('stale_request')
            return
        status['seq'] = req['seq']
        if req['motor'] > 1620 or req['motor'] < 1500 or not 1200 <= req['servo'] <= 1800:
            stop('pwm_out_of_range')
            return
        if (req['op'] == 'arm' and req['motor'] == 1500 and req['servo'] == 1500 and not status['armed']
                and (state['stopped_at'] is None or req['tick'] > state['stopped_at'])):
            status.update(armed=True, reason='armed')
        elif req['op'] == 'drive' and status['armed']:
            status.update(motor=req['motor'], servo=req['servo'], reason='manual')
            if req['motor'] > 1500:
                (shared/'driving').touch()
        else:
            stop('not_armed_or_invalid_request')
            return
        state['received'] = now

    def read():
        for line in sys.stdin:
            req = json.loads(line)
            with lock:
                apply(req, now_ms())
        with lock:
            state['closed'] = True

    threading.Thread(target=read, daemon=True).start()
    while True:
        with lock:
            advance(now_ms())
            if state['closed']:
                stop('input_closed')
                emit({'control': dict(status), 'physical_output': False, 'reverse_enabled': False})
                return
            emit({'control': dict(status), 'physical_output': False, 'reverse_enabled': False})
        time.sleep(.02)


def lidar():
    trial = Path(os.environ['FAKE_TRIAL'])
    shared = Path(os.environ.get('FAKE_SHARED', '.'))
    scans, drive_at, seen = [], None, set()
    for line in (trial/'feedback.jsonl').read_text().splitlines():
        row = json.loads(line)
        scan = (row.get('lidar') or {}).get('scan')
        control = ((row.get('state') or {}).get('status') or {}).get('control') or {}
        if isinstance(scan, dict) and scan['seq'] not in seen:
            seen.add(scan['seq'])
            scans.append(scan)
        if drive_at is None and control.get('motor', 1500) > 1500 and isinstance(scan, dict):
            drive_at = scan['at_ms']
    scans.sort(key=lambda s: s['seq'])
    static = [s for s in scans if drive_at is None or s['at_ms'] < drive_at] or scans[:1]
    moving = [s for s in scans if drive_at is not None and s['at_ms'] >= drive_at]
    seq, k = 1, 0
    while not (shared/'driving').exists():
        emit({**static[k % len(static)], 'seq': seq, 'at_ms': now_ms(), 'frame_id': 'lidar_origin_coarse_body_heading'})
        seq, k = seq+1, k+1
        time.sleep(.1)
    began, first = time.monotonic(), (moving[0]['at_ms'] if moving else 0)
    for scan in moving:
        delay = (scan['at_ms']-first)/1000-(time.monotonic()-began)
        if delay > 0:
            time.sleep(delay)
        emit({**scan, 'seq': seq, 'at_ms': now_ms(), 'frame_id': 'lidar_origin_coarse_body_heading'})
        seq += 1
    last = moving[-1] if moving else static[-1]
    while True:  # the last scan again: the car has stopped there
        time.sleep(.1)
        emit({**last, 'seq': seq, 'at_ms': now_ms(), 'frame_id': 'lidar_origin_coarse_body_heading'})
        seq += 1


if __name__ == '__main__':
    try:
        {'control': control, 'lidar': lidar}[sys.argv[1]]()
    except (BrokenPipeError, KeyboardInterrupt):
        pass
