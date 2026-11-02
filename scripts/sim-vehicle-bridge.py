#!/usr/bin/env python3
"""A closed-loop simulated vehicle-bridge (Mac tests of `route-engine race`; never the car).

  sim-vehicle-bridge.py control | lidar

Environment: SIM_SHARED (a scratch directory both modes share), SIM_LAYOUT
(course-sim layout, default user8), SIM_SEED, SIM_LIDAR_END_S (tests: the
lidar side ends this many seconds after the first powered drive).
With SIM_RACE4=1 use the gantry, crosswalk, simulated calibrated IMU, measured
tile response, and one side-ray dropout on the crosswalk restart. Pass
--light-state SIM_SHARED/light-state.json to the Rust race program in demo mode.

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
    fake.control((1000, 2000) if os.environ.get('SIM_RACE4') == '1' else (1500, 1620))


class RaceSensors:
    """Virtual camera and photo beam on the Mac. Pose is never given to the controller."""
    def __init__(self, cs, course, shared):
        self.cs, self.course, self.shared = cs, course, shared
        self.frame, self.next_frame, self.pending = 0, 0., None
        self.trigger = None
        self.stopped_at_stripes = self.dropout_sent = False
        self.crosswalk_m = 2.28

    def update(self, t, car):
        course = self.course
        c, s = math.cos(car.psi), math.sin(car.psi)
        sx, sy, psi0 = course.start
        to_stripes = self.crosswalk_m - ((car.x-sx)*math.cos(psi0)+(car.y-sy)*math.sin(psi0))
        turn = abs(math.atan2(math.sin(car.psi-psi0), math.cos(car.psi-psi0)))
        if turn < math.pi/6 and abs(to_stripes) < .9 and car.v < .05:
            self.stopped_at_stripes = True
        front = car.x + self.cs.FRONT*c
        if self.trigger is None and car.y >= course.W-course.light_lane-.1 and front >= course.gantry_x-1.10:
            self.trigger = t
        if self.pending is not None and t >= self.pending['captured_s'] + .24:
            self.pending['written_s'] = t
            temp = self.shared/'light-state.tmp'
            temp.write_text(json.dumps(self.pending))
            os.replace(temp, self.shared/'light-state.json')
            self.pending = None
        if t < self.next_frame:
            return
        self.next_frame += 1/3.6
        dx, dy = course.gantry_x-car.x, course.light_y-car.y
        ahead, lateral = dx*c+dy*s, -dx*s+dy*c
        visible = .60 <= ahead <= 6. and abs(math.atan2(lateral,ahead)) <= math.radians(44)
        elapsed = t-self.trigger if self.trigger is not None else -1.
        self.pending = {'frame':self.frame, 'captured_s':t, 'red':.90 if visible and .8 <= elapsed < 10. else 0.,
                        'green':.90 if visible and elapsed >= 10. else 0., 'crosswalk':0.}
        stripe = self.cs.crosswalk_frame(to_stripes) if turn <= math.pi/6 else None
        if stripe is not None:
            self.pending['crosswalk'], self.pending['crosswalk_m'] = stripe
        self.frame += 1


def lidar():
    shared = Path(os.environ['SIM_SHARED'])
    cs = load('course_sim_bridge', ROOT/'scripts'/'course-sim.py')
    race_sensors = os.environ.get('SIM_RACE4') == '1'
    course = cs.Course(**cs.LAYOUTS[os.environ.get('SIM_LAYOUT', 'user8')], gantry=race_sensors)
    seed = int(os.environ.get('SIM_SEED', '0'))
    tile = dict(decel=.60, esc_lag_s=.20, accel_max_mps2=4.0, brake_decel_mps2=.8,
                speed_tau_s=.70, gain_scale=1.2) if race_sensors else {}
    car = cs.MeasuredVehicle(*course.start, seed=seed, **tile)
    sensors = RaceSensors(cs, course, shared) if race_sensors else None
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
                if sensors: sensors.update(t, car)
            if now >= next_scan:
                ranges = []
                for i in range(360):
                    r = course.cast(car.x, car.y, car.psi-math.radians(i))
                    ranges.append(None if r is None or r > 12 or r < .02 or rng.random() < .002
                                  else round(r+rng.gauss(0, .008), 4))
                dropout = False
                if sensors and sensors.stopped_at_stripes and not sensors.dropout_sent and motor == 1600 and car.v >= .45:
                    for i in (*range(-27,-23), *range(-20,-14)):
                        ranges[i % 360] = None
                    sensors.dropout_sent = dropout = True
                scan = geometry({'seq': seq, 'at_ms': fake.now_ms(), 'frame_id': 'lidar_origin_coarse_body_heading',
                                 'ranges': ranges})
                if sensors:
                    scan['sim_imu'] = [car.kappa*car.v, car.v]  # yaw rate, integral of longitudinal acceleration from rest
                fake.emit(scan)
                wall, cone = course.body_gaps(car.x, car.y, car.psi)
                truth.write(json.dumps({'t': round(t, 3), 'seq': seq, 'x': car.x, 'y': car.y, 'psi': car.psi,
                                        'v': car.v, 'motor': motor, 'servo': servo, 'wall_gap': wall,
                                        'cone_gap': cone, 'injected_dropout':dropout})+'\n')
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
