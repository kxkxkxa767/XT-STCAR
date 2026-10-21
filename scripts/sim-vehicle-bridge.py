#!/usr/bin/env python3
"""A closed-loop simulated vehicle-bridge (Mac tests of `route-engine race`; never the car).

  sim-vehicle-bridge.py control | lidar

Environment: SIM_SHARED (a scratch directory both modes share), SIM_LAYOUT
(course-sim layout, default user8), SIM_SEED, SIM_LIDAR_END_S (tests: the
lidar side ends this many seconds after the first powered drive).

control: fake-vehicle-bridge.py's copy of the bridge's teleop guard; the
adopted motor/servo go to SIM_SHARED/command.json every 20 ms.
lidar: course-sim's measured vehicle model (physical servo, as the bridge
receives it) and course, integrated in real time from command.json; a 10 Hz
scan cast from the current pose with course-sim's range noise and the same
native geometry (tools/scan-geometry) the car's bridge adds. The true pose,
speed and body gaps go to SIM_SHARED/truth.jsonl. Real time and open to OS
scheduling, so not bit-reproducible; route-engine-supervisor-parity.py is
the exact check of the decisions. Offline only.
"""
import importlib.util
import json
import math
import os
from pathlib import Path
import random
import sys
import time

ROOT = Path(__file__).resolve().parents[1]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


fake = load('fake_vehicle_bridge', ROOT/'scripts'/'fake-vehicle-bridge.py')


def control():
    shared = Path(os.environ['SIM_SHARED'])
    inner = fake.emit

    def emit(row):
        inner(row)
        status = row['control']
        temp = shared/'command.json.tmp'
        temp.write_text(json.dumps({'armed': status['armed'], 'motor': status['motor'], 'servo': status['servo']}))
        os.replace(temp, shared/'command.json')
    fake.emit = emit
    os.environ.setdefault('FAKE_SHARED', str(shared))
    fake.control()


def lidar():
    shared = Path(os.environ['SIM_SHARED'])
    cs = load('course_sim_bridge', ROOT/'scripts'/'course-sim.py')
    course = cs.Course(**cs.LAYOUTS[os.environ.get('SIM_LAYOUT', 'user8')])
    seed = int(os.environ.get('SIM_SEED', '0'))
    car = cs.MeasuredVehicle(*course.start, seed=seed)
    geometry = cs.Geometry()
    rng = random.Random(seed)
    truth = (shared/'truth.jsonl').open('a')
    start = time.monotonic()
    t, seq, next_scan = 0., 1, 0.
    end_after = float(os.environ['SIM_LIDAR_END_S']) if os.environ.get('SIM_LIDAR_END_S') else None
    driving_at = None
    try:
        while True:
            now = time.monotonic()-start
            if end_after is not None and driving_at is None and (shared/'driving').exists():
                driving_at = now
            if driving_at is not None and now-driving_at >= end_after:
                return
            try:
                command = json.loads((shared/'command.json').read_text())
            except (OSError, ValueError):
                command = {'armed': False, 'motor': 1500, 'servo': 1500}
            motor = command['motor'] if command['armed'] else 1500
            servo = command['servo'] if command['armed'] else 1500
            while t+.02 <= now:
                t = round(t+.02, 6)
                car.step(t, .02, motor, servo)
            if now >= next_scan:
                ranges = []
                for i in range(360):
                    r = course.cast(car.x, car.y, car.psi-math.radians(i))
                    ranges.append(None if r is None or r > 12 or r < .02 or rng.random() < .002
                                  else round(r+rng.gauss(0, .008), 4))
                scan = geometry({'seq': seq, 'at_ms': fake.now_ms(), 'frame_id': 'lidar_origin_coarse_body_heading',
                                 'ranges': ranges})
                fake.emit(scan)
                wall, cone = course.body_gaps(car.x, car.y, car.psi)
                truth.write(json.dumps({'t': round(t, 3), 'seq': seq, 'x': car.x, 'y': car.y, 'psi': car.psi,
                                        'v': car.v, 'motor': motor, 'servo': servo, 'wall_gap': wall,
                                        'cone_gap': cone})+'\n')
                truth.flush()
                seq += 1
                next_scan += .1
            time.sleep(.005)
    finally:
        geometry.close()


if __name__ == '__main__':
    (Path(os.environ['SIM_SHARED'])/f'pid-{sys.argv[1]}').write_text(str(os.getpid()))
    try:
        {'control': control, 'lidar': lidar}[sys.argv[1]]()
    except (BrokenPipeError, KeyboardInterrupt):
        pass
