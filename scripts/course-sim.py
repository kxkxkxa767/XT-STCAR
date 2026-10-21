#!/usr/bin/env python3
"""Closed-loop course simulation of the explicit route (offline, never contacts the car).

The REAL service (web/vehicle-console/server.py Console, demo mode: body gate,
output checks, ACK sequencing, budgets, quality gates) runs the REAL
ManeuverSequence every 20 ms, exactly as the console does, on ray-cast scans of
a synthetic course of the rules' shape. The vehicle bridge's native geometry
(corridors, finite walls, left junction/turn goal, front boundary) comes from
tools/scan-geometry, the Mac build of the same Rust code.

Course (rules figure, sizes are parameters, not the competition's):
x from the start-end wall (0) to the finish-end wall (L), y from the start
lane's outer wall (0) to the light lane's outer wall (W). Start lane y in
[0, start_lane]; its divider at y=start_lane from x=0 to start_divider. Light
lane y in [W-light_lane, W]; its divider from x=L-light_divider to L. Finish
zone: the last .8 m of the light lane. Cones are circles.

Vehicle model (measured where available, see agent.md section 5): speed per
motor PWM with a first-order response, neutral rolling decel, curvature
(servo-1500)/160 per m with a pure delay plus first-order lag, body box from
the lidar (front .20, rear .18, sides .14). Each scan bin is cast from the
pose at its own time inside the 100 ms revolution (rotation distortion).

This is a model, not a certificate and not a vehicle replay.

Usage (repository root):
    cargo build --release -q --manifest-path tools/scan-geometry/Cargo.toml
    python3 scripts/course-sim.py --layout rules8 [--png out.png] [--log out.tsv]
    python3 scripts/course-sim.py --matrix
"""
import argparse
import importlib.util
import json
import math
from pathlib import Path
import random
import struct
import subprocess
import sys
import tempfile
import types
from unittest.mock import patch
import zlib

ROOT = Path(__file__).resolve().parents[1]
CONSOLE = ROOT/'web'/'vehicle-console'
TOOL = ROOT/'tools'/'scan-geometry'/'target'/'release'/'xt-stcar-scan-geometry'
FRONT, REAR, SIDE = .20, .18, .14
BOARD = .02

def layout(L, W, lane, divider, *, first=None, second=None, start=None):
    """A course of the construction document's shape (sizes are parameters).

    Document text (赛道搭建细节): boards around a rectangle 5-8 m x 4-6 m, two
    half dividers of about half the length, the lower one attached to the
    start-end wall and the upper one to the finish-end wall; worked example
    8 x 4 m, dividers 3.2 m, lanes .9 m, cones on the middle line 2 m from the
    left/right walls. The cone default scales that example (L/4, 3L/4 on the
    middle line); the first cone in route order is the one nearer the finish
    end. Figure proportions are never used as dimensions.
    """
    return dict(L=L, W=W, start_lane=lane, light_lane=lane, start_divider=divider, light_divider=divider,
                cones=[first or (.75*L, W/2), second or (.25*L, W/2)], start=start)


LAYOUTS = {
    # Practice course laid with 6.4 m side straights (trials up to 62),
    # cones where the trial-60 map shows them, start at the divider end.
    'practice': dict(L=8.1, W=4.15, start_lane=1.05, light_lane=1.05, start_divider=6.4, light_divider=6.4,
                     cones=[(6.0, 2.0), (2.0, 1.95)], start=(5.85, .52, 0.)),
    # The construction document's worked example (nominal, not official coordinates).
    'doc8x4': layout(8., 4., .9, 3.2),
    # The user's re-laid course: practice outer frame and lanes, 3.2 m dividers.
    'user8': layout(8.1, 4.15, 1.05, 3.2),
    'mid64': layout(6.4, 5., 1., 3.2),
    'short5': layout(5., 4., .9, 2.5),
    'wide8x6': layout(8., 6., 1.2, 4.),
}


def variations():
    """The rules' range: lengths, widths, lanes, divider fractions, cone offsets."""
    for L in (5., 6.4, 8.):
        for W in (4., 5., 6.):
            for lane in (.9, 1.05):
                for fraction in (.4, .5):
                    for dx, dy in ((0., 0.), (-.4, .3), (.4, -.3)):
                        yield (f'L{L}W{W}lane{lane}div{fraction}cone{dx:+}{dy:+}',
                               layout(L, W, lane, round(fraction*L, 2),
                                      first=(.75*L+dx, W/2+dy), second=(.25*L-dx, W/2-dy)))


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Course:
    def __init__(self, L, W, start_lane, light_lane, start_divider, light_divider, cones, start=None,
                 cone_radius=.10):
        self.L, self.W = L, W
        self.start_lane, self.light_lane = start_lane, light_lane
        self.cones = [(x, y, cone_radius) for x, y in cones]
        self.segments = [((0, 0), (L, 0)), ((L, 0), (L, W)), ((L, W), (0, W)), ((0, W), (0, 0))]
        self.boards = []
        for x0, x1, y in ((0., start_divider, start_lane), (L-light_divider, L, W-light_lane)):
            box = [((x0, y-BOARD/2), (x1, y-BOARD/2)), ((x1, y-BOARD/2), (x1, y+BOARD/2)),
                   ((x1, y+BOARD/2), (x0, y+BOARD/2)), ((x0, y+BOARD/2), (x0, y-BOARD/2))]
            self.boards.append(box)
            self.segments.extend(box)
        # Lidar in the 80 cm start zone, rear ~.2 m clear of the start-end wall.
        self.start = start or (.40, start_lane/2, 0.)
        self.finish_x = L-.8

    def cast(self, ox, oy, angle, *, surface=False):
        """Ray range; optionally the acute ray-to-wall tangent angle (0 = grazing).

        Circular cone hits have no planar angle and are never subject to the
        optional grazing-wall dropout. The default follows the old arithmetic.
        """
        dx, dy = math.cos(angle), math.sin(angle)
        best = None
        grazing_angle = None
        for (x1, y1), (x2, y2) in self.segments:
            ex, ey = x2-x1, y2-y1
            den = dx*ey-dy*ex
            if abs(den) < 1e-12:
                continue
            t = ((x1-ox)*ey-(y1-oy)*ex)/den
            u = ((x1-ox)*dy-(y1-oy)*dx)/den
            if t > 1e-6 and -1e-9 <= u <= 1+1e-9 and (best is None or t < best):
                best = t
                if surface:
                    grazing_angle = math.degrees(math.asin(min(1., abs(den)/math.hypot(ex, ey))))
        for cx, cy, r in self.cones:
            fx, fy = ox-cx, oy-cy
            b = fx*dx+fy*dy
            disc = b*b-(fx*fx+fy*fy-r*r)
            if disc >= 0:
                t = -b-math.sqrt(disc)
                if t > 1e-6 and (best is None or t < best):
                    best = t
                    grazing_angle = None
        return (best, grazing_angle) if surface else best

    def body_gaps(self, x, y, psi):
        """Smallest distance from the body box to a board/wall and to a cone."""
        c, s = math.cos(psi), math.sin(psi)

        def local(px, py):
            return c*(px-x)+s*(py-y), -s*(px-x)+c*(py-y)

        def box_distance(lx, ly):
            return math.hypot(max(-REAR-lx, 0., lx-FRONT), max(abs(ly)-SIDE, 0.))

        wall = math.inf
        for (x1, y1), (x2, y2) in self.segments:
            if min(abs(x1-x), abs(x2-x)) > 2.5 and (x1-x)*(x2-x) > 0:
                continue
            if min(abs(y1-y), abs(y2-y)) > 2.5 and (y1-y)*(y2-y) > 0:
                continue
            n = max(2, int(math.dist((x1, y1), (x2, y2))/.01))
            for k in range(n+1):
                px, py = x1+(x2-x1)*k/n, y1+(y2-y1)*k/n
                if abs(px-x) < 1. and abs(py-y) < 1.:
                    wall = min(wall, box_distance(*local(px, py)))
        cone = math.inf
        for cx, cy, r in self.cones:
            for k in range(24):
                a = k*math.pi/12
                cone = min(cone, box_distance(*local(cx+r*math.cos(a), cy+r*math.sin(a))))
        return wall, cone


class Geometry:
    """Persistent tools/scan-geometry process (same Rust as the vehicle bridge)."""
    def __init__(self):
        if not TOOL.exists():
            subprocess.run(['cargo', 'build', '--release', '-q', '--manifest-path',
                            str(ROOT/'tools'/'scan-geometry'/'Cargo.toml')], check=True)
        self.proc = subprocess.Popen([str(TOOL)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)

    def __call__(self, scan):
        self.proc.stdin.write(json.dumps({'ranges': scan['ranges'], 'at_ms': scan['at_ms']})+'\n')
        self.proc.stdin.flush()
        row = json.loads(self.proc.stdout.readline())
        for key in ('front_boundary_m', 'left_junction', 'corridor_candidates', 'wall_candidates',
                    'left_turn_goal'):
            scan[key] = row[key]
        return scan

    def close(self):
        self.proc.stdin.close()
        self.proc.wait()


class Vehicle:
    def __init__(self, x, y, psi, *, speed_scale=1., decel=.5, delay=.15, steer_tau=.1, seed=0,
                 right_curvature_max=1.875, steer_backlash=0.):
        if not math.isfinite(right_curvature_max) or right_curvature_max <= 0:
            raise ValueError('right_curvature_max must be finite and positive')
        if not math.isfinite(steer_backlash) or not 0 <= steer_backlash < 100:
            raise ValueError('steer_backlash must be finite and non-negative')
        self.x, self.y, self.psi, self.v = x, y, psi, 0.
        self.speed_scale, self.decel, self.delay, self.steer_tau = speed_scale, decel, delay, steer_tau
        self.servo_queue = []  # (time, servo) commands, applied after the delay
        self.servo_phys = 1500.
        self.right_curvature_max = right_curvature_max
        # Steering play (PWM): on the ground the wheels stay where they were
        # until the servo has moved through the play (user, 2026-10-13: after
        # a turn, 1500 leaves the wheels toward that turn; lifted, straight).
        # A hypothesis for robustness runs; the real play is not measured.
        self.steer_backlash = steer_backlash
        self.wheel_pwm = 1500.
        # Keep the measured steady right turn (1350 -> about .94/m, agent.md
        # section 5) when the wheels lag the servo by the play while turning.
        self.pwm_per_curvature = 160*(150-steer_backlash)/150
        self.rng = random.Random(seed)

    def target_speed(self, motor):
        # 1600 (round 100 start-lane launch): extrapolated from 1560/1570, not measured.
        return {1550: 1.0, 1560: 1.35, 1570: 1.5, 1600: 1.95}.get(motor, 0.)*self.speed_scale

    def step(self, t, dt, motor, servo):
        self.servo_queue.append((t, servo))
        applied = self.servo_phys
        while self.servo_queue and self.servo_queue[0][0] <= t-self.delay:
            applied = self.servo_queue.pop(0)[1]
            self._delayed = applied
        target = getattr(self, '_delayed', 1500)
        self.servo_phys += (target-self.servo_phys)*min(1., dt/self.steer_tau)
        # Nominal extension of the existing PWM model to the static1200
        # endpoint. This is not a measured right-turn calibration.
        self.wheel_pwm = min(self.servo_phys+self.steer_backlash, max(self.servo_phys-self.steer_backlash,
                                                                       self.wheel_pwm))
        kappa = max(-self.right_curvature_max, min(1.375, (self.wheel_pwm-1500)/self.pwm_per_curvature))
        vt = self.target_speed(motor)
        if vt > 0 and (self.v > .05 or motor >= 1560):
            self.v += max(-2., min(2., (vt-self.v)/.35))*dt
        else:
            self.v = max(0., self.v-self.decel*dt)
        n = 4
        for _ in range(n):
            self.psi += kappa*self.v*dt/n
            self.x += self.v*math.cos(self.psi)*dt/n
            self.y += self.v*math.sin(self.psi)*dt/n


# Steering fitted on the track (rounds 70-78, play 40, ~155 PWM per 1/m both
# sides; docs/vehicle-dynamic-turn-validation-20261007.json:
# turn79_steer_calibration_20261019), the console's steer_calibration table;
# beyond the right planning limit it saturates as on the 10-16 tiles (1200
# -1.35 /m). Speed and coast: tiles 2026-10-13/16 (reverse_brake_test_20261016).
# Wheel PWM -> curvature (1/m) after the steering play; values between points
# are interpolated.
MEASURED_STEER_TABLE = ((1200, -1.35), (1314, -1.2), (1500, 0.), (1720, 1.42))
# Steady speed (m/s) per motor PWM on tiles, straight; speed keeps rising for
# seconds (1560: 1.2 -> 1.9 m/s over 1.3 s), so a first-order lag.
# 1600 (round 100 start-lane launch): extrapolated (+.5 per 10 PWM), not
# measured; the response is limited to 2.5 m/s^2 below.
MEASURED_TARGET_SPEED = {1550: 1.1, 1560: 2.2, 1570: 2.7, 1580: 3.2, 1600: 4.2}


def _interp(table, x):
    if x <= table[0][0]:
        return table[0][1]
    for (x0, y0), (x1, y1) in zip(table, table[1:]):
        if x <= x1:
            return y0+(y1-y0)*(x-x0)/(x1-x0)
    return table[-1][1]


class MeasuredVehicle(Vehicle):
    """Measured car: steering play, table gain, ESC lag both ways, slow coast.

    play_pwm: wheels follow the servo only outside +-play (track fit 40).
    esc_lag_s: the motor responds that much later to any command, also to the
    cut (the car kept pushing ~.5 s after a stop). coast_decel: on tiles
    ~.6-.7 m/s^2 after the lag. drift_per_mps: slight left drift at speed (user and runs 11-13).
    turn_drag: steady speed divided by (1 + turn_drag*|curvature|).
    """
    def __init__(self, x, y, psi, *, speed_scale=1., decel=.7, delay=.15, steer_tau=.1, seed=0,
                 play_pwm=40., gain_scale=1., esc_lag_s=.45, drift_per_mps=.025, turn_drag=.5,
                 speed_tau_s=1.1, near_center_gain=1., near_center_width_pwm=100., initial_wheel_pwm=1500.,
                 **_):
        super().__init__(x, y, psi, speed_scale=speed_scale, decel=decel, delay=delay, steer_tau=steer_tau,
                         seed=seed)
        # Where the wheels sit inside the play at the start (round 68: 1500
        # kept the last turn's side, about -.17 /m); default centred as before.
        if not math.isfinite(initial_wheel_pwm) or abs(initial_wheel_pwm-1500) > play_pwm:
            raise ValueError('initial_wheel_pwm must lie inside the play')
        self.wheel_pwm = float(initial_wheel_pwm)
        self.play_pwm, self.gain_scale, self.esc_lag_s = play_pwm, gain_scale, esc_lag_s
        self.drift_per_mps, self.turn_drag, self.speed_tau_s = drift_per_mps, turn_drag, speed_tau_s
        if not math.isfinite(near_center_gain) or near_center_gain <= 0:
            raise ValueError('near_center_gain must be finite and positive')
        if not math.isfinite(near_center_width_pwm) or near_center_width_pwm <= 0:
            raise ValueError('near_center_width_pwm must be finite and positive')
        self.near_center_gain = near_center_gain
        self.near_center_width_pwm = near_center_width_pwm
        self.motor_queue = []
        self.motor_phys = 1500
        self.kappa = 0.

    def target_speed(self, motor):
        return MEASURED_TARGET_SPEED.get(motor, 0.)*self.speed_scale

    def step(self, t, dt, motor, servo):
        self.servo_queue.append((t, servo))
        while self.servo_queue and self.servo_queue[0][0] <= t-self.delay:
            self._delayed = self.servo_queue.pop(0)[1]
        target = getattr(self, '_delayed', 1500)
        self.servo_phys += (target-self.servo_phys)*min(1., dt/self.steer_tau)
        self.wheel_pwm = min(self.servo_phys+self.play_pwm, max(self.servo_phys-self.play_pwm, self.wheel_pwm))
        steering_curvature = _interp(MEASURED_STEER_TABLE, self.wheel_pwm)
        if self.near_center_gain != 1.:
            # Hypothesis, not a fitted calibration: monotonically warp wheel
            # position near centre, keeping both endpoint table values. Unlike
            # multiplying curvature by a taper, this cannot invert the slope.
            # Play remains real play: commands within it do NOT move the wheel.
            offset = self.wheel_pwm-1500
            u = abs(offset)/self.near_center_width_pwm
            if u < 1.:
                warped = self.near_center_gain*u/(1.+(self.near_center_gain-1.)*u)
                wheel = 1500+math.copysign(self.near_center_width_pwm*warped, offset)
                steering_curvature = _interp(MEASURED_STEER_TABLE, wheel)
        self.kappa = steering_curvature*self.gain_scale+self.drift_per_mps*self.v
        self.motor_queue.append((t, motor))
        while self.motor_queue and self.motor_queue[0][0] <= t-self.esc_lag_s:
            self.motor_phys = self.motor_queue.pop(0)[1]
        vt = self.target_speed(self.motor_phys)/(1+self.turn_drag*abs(self.kappa))
        if vt > 0 and (self.v > .05 or self.motor_phys >= 1560):
            self.v += max(-self.decel, min(2.5, (vt-self.v)/self.speed_tau_s))*dt
        else:
            self.v = max(0., self.v-self.decel*dt)
        n = 4
        for _ in range(n):
            self.psi += self.kappa*self.v*dt/n
            self.x += self.v*math.cos(self.psi)*dt/n
            self.y += self.v*math.sin(self.psi)*dt/n


def grazing_dropouts(ranges, surface_angles, *, angle_deg, probability, rng):
    """Drop only planar hits at an acute ray-to-wall angle below the threshold.

    A dedicated RNG avoids changing range noise or depth-edge dropout draws.
    This acts after ray casting and ordinary noise; it never invents a return.
    """
    for i, angle in enumerate(surface_angles):
        if (ranges[i] is not None and angle is not None and angle < angle_deg
                and rng.random() < probability):
            ranges[i] = None


def make_console(server):
    temp = tempfile.TemporaryDirectory()
    console = server.Console(types.SimpleNamespace(access_file=str(Path(temp.name)/'access.json'),
                                                   output=str(Path(temp.name)/'out'), demo=True,
                                                   allow_reverse=False))
    console.coast_worker = server.CoastMotionWorker(autostart=False)
    console.healthy = console.autonomy_healthy = lambda: True
    console.sensor_ages = lambda now: {'camera': .01, 'lidar': now-console.scan_at, 'control': .01}
    console.status = {'control': {'armed': False, 'motor': 1500, 'servo': 1500, 'seq': 0, 'tick': 1000}}
    console._temp = temp
    return console


def run(course, *, max_s=45., speed_scale=1., decel=.5, delay=.15, distortion=1., noise=.008, seed=0,
        verbose=False, initial_presteer_pwm=None, max_drive_s=40., trace=None, right_curvature_max=1.875,
        steer_backlash=0., vehicle='nominal', vehicle_params=None, edge_dropout=0.,
        grazing_angle_deg=0., grazing_dropout=1.):
    if not math.isfinite(grazing_angle_deg) or not 0 <= grazing_angle_deg <= 90:
        raise ValueError('grazing_angle_deg must be finite and in 0..90')
    if not math.isfinite(grazing_dropout) or not 0 <= grazing_dropout <= 1:
        raise ValueError('grazing_dropout must be finite and in 0..1')
    sys.path.insert(0, str(CONSOLE))
    server = load('course_sim_server', CONSOLE/'server.py')
    console = make_console(server)
    geometry = Geometry()
    rng = random.Random(seed)
    edge_rng = random.Random(seed*7919+13)
    grazing_rng = random.Random(seed*104729+29)
    if vehicle == 'measured':
        params = dict(vehicle_params or {})
        speed_scale = params.pop('speed_scale', speed_scale)
        car = MeasuredVehicle(*course.start, speed_scale=speed_scale, delay=delay, seed=seed, **params)
    elif vehicle == 'nominal':
        car = Vehicle(*course.start, speed_scale=speed_scale, decel=decel, delay=delay, seed=seed,
                      right_curvature_max=right_curvature_max, steer_backlash=steer_backlash)
    else:
        raise ValueError('vehicle must be nominal or measured')
    history = [(0., car.x, car.y, car.psi)]
    clock = {'now': 0.}
    outputs = []

    def emit(op, motor=1500, servo=1500, tick=0):
        console.sequence += 1
        control = console.status['control']
        control.update(seq=console.sequence, motor=motor)
        if op == 'arm':
            control['armed'] = True
        elif op == 'stop':
            control.update(armed=False, motor=1500, servo=1500)
        control['servo'] = servo
        # The console's steering calibration (nominal -> physical) as the real
        # emit applies it; the bridge feedback is translated back to nominal.
        control['physical_servo'] = (console.output_servo(op, servo, console.sequence)
                                     if op == 'drive' else servo)
        console.control_at = clock['now']
        outputs.append((clock['now'], op, motor, servo))
    console.emit = emit

    def pose_at(t):
        for (t0, x0, y0, p0), (t1, x1, y1, p1) in zip(reversed(history[:-1]), reversed(history[1:])):
            if t0 <= t <= t1:
                a = (t-t0)/(t1-t0) if t1 > t0 else 0.
                return x0+(x1-x0)*a, y0+(y1-y0)*a, p0+(p1-p0)*a
        return history[0][1:] if t < history[0][0] else history[-1][1:]

    def make_scan(seq, t):
        ranges = []
        surface_angles = []
        for i in range(360):
            x, y, psi = pose_at(t-distortion*(359-i)/360*.1)
            if grazing_angle_deg:
                r, angle = course.cast(x, y, psi-math.radians(i), surface=True)
                surface_angles.append(angle)
            else:
                r = course.cast(x, y, psi-math.radians(i))
            if r is None or r > 12 or r < .02 or rng.random() < .002:
                ranges.append(None)
            else:
                ranges.append(round(r+rng.gauss(0, noise), 4))
        if edge_dropout:
            # The real lidar loses runs of returns at depth steps (a near edge
            # against a far wall; round 66: 5 bins between the divider's open
            # end and the end wall 7 m away). Separate generator: the other
            # noise is the same with or without this model.
            raw = list(ranges)
            for i in range(360):
                a, b = raw[i], raw[(i+1) % 360]
                if (a is not None and b is not None and abs(a-b) > 1. and max(a, b) > 2.
                        and edge_rng.random() < edge_dropout):
                    run = edge_rng.randint(1, 5)
                    for k in range(run):  # on the far side of the step
                        ranges[((i+1+k) if b > a else (i-k)) % 360] = None
        if grazing_angle_deg:
            grazing_dropouts(ranges, surface_angles, angle_deg=grazing_angle_deg,
                             probability=grazing_dropout, rng=grazing_rng)
        return geometry({'seq': seq, 'at_ms': round(t*1000), 'frame_id': 'lidar_origin_coarse_body_heading',
                         'ranges': ranges})

    seq = 1
    console.scan, console.scan_at, console.control_at = make_scan(seq, 0.), 0., 0.
    log, contacts, events = [], [], []
    start_error = None
    data = {'op': 'turn_cone_start', 'boot': console.boot, 'epoch': console.control_epoch,
            'tick': console.status['control']['tick'], 'placement_confirmed': True, 'continue_route': True,
            'max_drive_s': max_drive_s}  # the trial helper's --max-seconds
    if initial_presteer_pwm is not None:
        data['initial_presteer_pwm'] = initial_presteer_pwm
    with patch.object(server.time, 'monotonic', return_value=0.):
        try:
            console.autonomy_command(data)
        except ValueError as error:
            start_error = str(error)
    dt = .02
    t = 0.
    stage_seen = []
    min_wall = min_cone = math.inf
    min_wall_at = min_cone_at = None
    while start_error is None and t < max_s:
        t = round(t+dt, 6)
        clock['now'] = t
        control = console.status['control']
        motor = control['motor'] if control.get('armed') else 1500
        servo = ((control.get('physical_servo', control['servo']) if vehicle == 'measured' else control['servo'])
                 if control.get('armed') else 1500)
        car.step(t, dt, motor, servo)
        history.append((t, car.x, car.y, car.psi))
        del history[:-40]
        if round(t*1000) % 100 == 0:
            seq += 1
            console.scan, console.scan_at = make_scan(seq, t), t
        console.owner_at = t
        console.control_at = t
        control['tick'] = 1000+round(t*1000)
        if console.auto_session is not None:
            with patch.object(server.time, 'monotonic', return_value=t):
                console.autonomy_tick(t)
            console.coast_worker.run_pending()
        session = console.auto_session
        motion = session.get('turn_motion') if session else None
        stage = getattr(motion, 'route_stage', None) if motion else None
        phase = session.get('phase') if session else 'ended'
        reason = getattr(motion, 'reason', '') if motion else (console.auto_result or {}).get('reason', '')
        if not stage_seen or stage_seen[-1][1:] != (stage, phase):
            stage_seen.append((t, stage, phase))
        wall, cone = course.body_gaps(car.x, car.y, car.psi)
        if wall < min_wall:
            min_wall_at = (round(t, 2), round(car.x, 3), round(car.y, 3), round(car.psi, 4))
        if cone < min_cone:
            min_cone_at = (round(t, 2), round(car.x, 3), round(car.y, 3), round(car.psi, 4))
        min_wall, min_cone = min(min_wall, wall), min(min_cone, cone)
        if wall <= 0 and not any(c[1] == 'board' for c in contacts):
            contacts.append((t, 'board', round(car.x, 2), round(car.y, 2)))
        if cone <= 0 and not any(c[1] == 'cone' and t-c[0] < 1 for c in contacts):
            contacts.append((t, 'cone', round(car.x, 2), round(car.y, 2)))
        if round(t*1000) % 100 == 0:
            row = (t, car.x, car.y, math.degrees(car.psi), car.v, motor, servo, stage, phase, reason, wall, cone)
            log.append(row)
            if verbose:
                print('%6.2f x %5.2f y %5.2f psi %7.1f v %4.2f m %d s %d %-14s %-12s %-44s wall %.3f cone %.3f' % row)
            if trace and motion is not None:
                values = {key: getattr(motion, key, None) for key in trace}
                print('       ', json.dumps(values, ensure_ascii=False, default=str))
        if session is None and car.v < .01:
            break
        if any(c[1] == 'board' for c in contacts) and car.v < .01:
            break
    geometry.close()
    console.coast_worker.close()
    result = console.auto_result or {}
    front_x = car.x+FRONT*math.cos(car.psi)
    in_finish = (front_x >= course.finish_x and car.y >= course.W-course.light_lane
                 and abs(math.degrees(car.psi)) < 45)
    summary = {'start_error': start_error, 'end_reason': result.get('reason'),
               'stages': [(round(a, 2), b, c) for a, b, c in stage_seen],
               'contacts': contacts, 'min_board_gap_m': round(min_wall, 3), 'min_cone_gap_m': round(min_cone, 3),
               'min_board_gap_at': min_wall_at, 'min_cone_gap_at': min_cone_at,
               'final': (round(car.x, 2), round(car.y, 2), round(math.degrees(car.psi), 1)),
               'final_front_x': round(front_x, 2), 'in_finish_zone': in_finish, 'time_s': t,
               'success': in_finish and not any(c[1] == 'board' for c in contacts),
               # Keep the historical position-based score for comparisons,
               # but do not count a safety stop in the zone as route completion.
               'route_completed': (in_finish and not any(c[1] == 'board' for c in contacts)
                                   and result.get('reason') == 'route_finish_zone_stop'),
               'vehicle_model': {'right_curvature_max_per_m': right_curvature_max,
                                 'right_1200_dynamically_calibrated': False,
                                 'steer_backlash_pwm': steer_backlash,
                                 'pwm_per_curvature': car.pwm_per_curvature,
                                 'vehicle': vehicle, 'vehicle_params': vehicle_params or {}}}
    if grazing_angle_deg:
        summary['grazing_model'] = {'angle_deg': grazing_angle_deg, 'probability': grazing_dropout,
                                    'angle_definition': 'acute ray-to-planar-wall tangent; 0 = parallel'}
    sys.path.remove(str(CONSOLE))
    return summary, log, history


# Glyphs the trial-map bitmap font lacks (dimension labels).
EXTRA_GLYPHS = {'X': ('101', '101', '010', '101', '101'), ',': ('000', '000', '000', '010', '100'),
                ':': ('000', '010', '000', '010', '000'), '=': ('000', '111', '000', '111', '000'),
                '(': ('010', '100', '100', '100', '010'), ')': ('010', '001', '001', '001', '010'),
                '/': ('001', '001', '010', '100', '100'), '+': ('000', '010', '111', '010', '000'),
                'J': ('001', '001', '001', '101', '010')}
STAGE_COLORS = {'drive': (0, 150, 0), 'presteer': (40, 90, 230), 'quality_wait': (240, 140, 0),
                'coast': (150, 60, 200), 'ended': (220, 0, 0)}


def render(course, log, path, *, name='', summary=None, scale=90):
    """Top view with a 1 m grid and the course dimensions (metres)."""
    tm = load('course_sim_map', ROOT/'scripts'/'trial-map.py')
    tm.FONT.update(EXTRA_GLYPHS)
    left, right, top, bottom = 1.25, 1.05, 1.4, 1.05
    right = max(right, 960/scale-course.L-left)  # header and legend fit on narrow courses
    w, h = int((course.L+left+right)*scale), int((course.W+top+bottom)*scale)
    img = tm.Image(w, h)
    dim_color, grid_color, label_color = (200, 30, 30), (228, 228, 228), (90, 90, 90)
    char_w = 8  # text scale 2: 4*2 px per character

    def uv(x, y):
        return int((x+left)*scale), int((course.W+top-y)*scale)

    def label(x, y, text, color=dim_color, anchor='center'):
        u, v = uv(x, y)
        text = text.upper()
        width = len(text)*char_w
        u = u-width//2 if anchor == 'center' else u-width if anchor == 'right' else u
        img.text(u, v-5, text, color)

    def dimension(a, b, offset, text, side=None):
        """Dimension line parallel to a-b, shifted by offset, with end ticks."""
        (ax, ay), (bx, by), (ox, oy) = a, b, offset
        pa, pb = (ax+ox, ay+oy), (bx+ox, by+oy)
        img.line(uv(*a), uv(*pa), (235, 170, 170))
        img.line(uv(*b), uv(*pb), (235, 170, 170))
        img.line(uv(*pa), uv(*pb), dim_color)
        length = math.dist(a, b)
        tx, ty = ((bx-ax)/length, (by-ay)/length) if length else (1, 0)
        for px, py in (pa, pb):
            img.line(uv(px-.05*(tx+ty*0), py-.05*(ty)), uv(px+.05*tx, py+.05*ty), dim_color)
            img.line(uv(px-.06*ty, py+.06*tx), uv(px+.06*ty, py-.06*tx), dim_color)
        mx, my = (pa[0]+pb[0])/2, (pa[1]+pb[1])/2
        if side == 'left':
            label(mx-.08, my, text, anchor='right')
        elif side == 'right':
            label(mx+.08, my, text, anchor='left')
        elif side == 'below':
            label(mx, my-.14, text)
        else:
            label(mx, my+.14, text)

    # 1 m grid with coordinates from the start-end wall (x) and the start-lane outer wall (y).
    for k in range(int(math.floor(course.L))+1):
        img.line(uv(k, 0), uv(k, course.W), grid_color)
        label(k, -.15, str(k), label_color)
    for k in range(int(math.floor(course.W))+1):
        img.line(uv(0, k), uv(course.L, k), grid_color)
        label(-.1, k, str(k), label_color, anchor='right')
    label(course.L+.12, -.15, 'X M', label_color, anchor='left')
    label(-.1, course.W+.15, 'Y M', label_color, anchor='right')
    for (x1, y1), (x2, y2) in course.segments:
        img.line(uv(x1, y1), uv(x2, y2), (40, 40, 40))
    # Overall size, lanes, dividers.
    # Dimensions outside the field: dividers in their own rows (the lower one
    # is attached to the start-end wall, the upper one to the finish-end wall).
    sl, ll = course.start_lane, course.light_lane
    start_div = max(x for (x1, _), (x, _) in course.segments[4:8] for x in (x1, x))
    light_div = course.L-min(x for (x1, _), (x, _) in course.segments[8:12] for x in (x1, x))
    dimension((0, sl), (start_div, sl), (0, -sl-.42), f'START DIVIDER {start_div:.2f}', side='below')
    dimension((0, 0), (course.L, 0), (0, -.8), f'L {course.L:.2f}', side='below')
    dimension((0, 0), (0, course.W), (-1.05, 0), f'W {course.W:.2f}', side='right')
    dimension((course.L, 0), (course.L, sl), (.15, 0), f'LANE {sl:.2f}', side='right')
    dimension((course.L, course.W-ll), (course.L, course.W), (.15, 0), f'LANE {ll:.2f}', side='right')
    dimension((course.L-light_div, course.W-ll), (course.L, course.W-ll), (0, ll+.25),
              f'LIGHT DIVIDER {light_div:.2f}')
    # Cones with coordinates and their distance to the nearer end wall.
    for index, (cx, cy, r) in enumerate(course.cones, 1):
        img.dot(*uv(cx, cy), (30, 60, 220), r=max(2, int(r*scale)))
        label(cx, cy+r+.17, f'C{index} ({cx:.2f},{cy:.2f})', (30, 60, 220))
        label(cx, cy-r-.17, f'R {r:.2f}', (30, 60, 220))
    # Start zone (first .8 m of the start lane).
    img.line(uv(.8, 0), uv(.8, sl), (40, 90, 230))
    u, v = uv(.83, sl-.08)
    img.text(u, v, 'START ZONE 0.80', (40, 90, 230), scale=1)
    # Finish zone (last .8 m of the light lane).
    fx = course.finish_x
    img.line(uv(fx, course.W-ll), uv(fx, course.W), (0, 160, 0))
    dimension((fx, course.W), (course.L, course.W), (0, .62), f'FINISH {course.L-fx:.2f}')
    # Path (colour = service phase), body box every .5 s, time every 1 s.
    previous = None
    for k, row in enumerate(log):
        t, x, y, psi = row[:4]
        point = uv(x, y)
        if previous:
            img.line(previous, point, STAGE_COLORS.get(row[8], (120, 120, 120)))
        previous = point
        if k % 5 == 0:
            c, s = math.cos(math.radians(psi)), math.sin(math.radians(psi))
            corners = [(x+c*a-s*b, y+s*a+c*b) for a, b in ((FRONT, SIDE), (FRONT, -SIDE), (-REAR, -SIDE), (-REAR, SIDE))]
            for a, b in zip(corners, corners[1:]+corners[:1]):
                img.line(uv(*a), uv(*b), (170, 170, 170))
        if k % 10 == 0:
            img.text(point[0]+4, point[1]-12, f'{t:.0f}', (0, 0, 0), scale=1)
    if summary:
        for key, gap_key, color, tag in (('min_board_gap_at', 'min_board_gap_m', (220, 0, 0), 'MIN BOARD'),
                                         ('min_cone_gap_at', 'min_cone_gap_m', (30, 60, 220), 'MIN CONE')):
            at = summary.get(key)
            if at:
                _, x, y, psi = at
                u, v = uv(x, y)
                for d in range(-6, 7):
                    img.put(u+d, v+d, color)
                    img.put(u+d, v-d, color)
                img.text(u+8, v+6, f'{tag} {summary[gap_key]:.3f}', color, scale=1)
        head = (f"{name} {'SUCCESS' if summary.get('success') else 'FAIL'}  "
                f"BOARD MIN {summary['min_board_gap_m']:.3f}  CONE MIN {summary['min_cone_gap_m']:.3f}  "
                f"T {summary['time_s']:.1f}S  END {summary.get('end_reason') or summary.get('start_error')}")
        img.text(8, 8, head.upper().replace('_', '-'), (0, 0, 0))
    u = 8
    for phase, color in STAGE_COLORS.items():
        for a in range(10):
            for b in range(10):
                img.put(u+a, 28+b, color)
        img.text(u+14, 28, phase.upper().replace('_', '-'), color)
        u += 14+len(phase)*char_w+16
    img.text(u+8, 28, 'UNIT M  GRID 1 M  TIME LABELS S', label_color)
    img.save(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--layout', default='doc8x4', choices=sorted(LAYOUTS))
    parser.add_argument('--cones', help='x1,y1,x2,y2 override')
    parser.add_argument('--start', help='x,y,heading_deg override')
    parser.add_argument('--speed-scale', type=float, default=1.)
    parser.add_argument('--decel', type=float, default=.5)
    parser.add_argument('--delay', type=float, default=.15)
    parser.add_argument('--right-curvature-max', type=float, default=1.875,
                        help='unmeasured nominal1200 limit (1/m); use .94 for old-cap sensitivity')
    parser.add_argument('--vehicle', choices=('nominal', 'measured'), default='nominal',
                        help='measured: track steering fit (rounds 70-78) and 2026-10-13/16 tile ESC lag and coast')
    parser.add_argument('--vehicle-param', action='append', default=[], metavar='NAME=VALUE',
                        help='measured model parameter, e.g. play_pwm=70 esc_lag_s=.6 (repeatable)')
    parser.add_argument('--steer-backlash', type=float, default=0.,
                        help='hypothetical steering play (PWM) for robustness runs; not measured')
    parser.add_argument('--edge-dropout', type=float, default=0.,
                        help='chance per depth step (>1 m, far side >2 m) of losing 1-5 returns there (real lidar; default off)')
    parser.add_argument('--grazing-angle-deg', type=float, default=0.,
                        help='lose planar wall returns below this ray-to-wall tangent angle; 0 disables (default)')
    parser.add_argument('--grazing-dropout', type=float, default=1.,
                        help='probability per grazing planar return; independent RNG, default 1')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--max-s', type=float, default=45.)
    parser.add_argument('--png')
    parser.add_argument('--log')
    parser.add_argument('-v', '--verbose', action='store_true')
    parser.add_argument('--trace', action='append', help='print this motion attribute every 100 ms (repeatable)')
    parser.add_argument('--matrix', action='store_true', help='run every named layout with vehicle variations')
    parser.add_argument('--range', action='store_true', help='run the rules-range variations (nominal vehicle)')
    parser.add_argument('--variant', help='run one --range variation by name (e.g. L8.0W6.0lane0.9div0.5cone+0.0+0.0)')
    parser.add_argument('--png-dir', help='with --range: write each variation\'s picture here')
    parser.add_argument('--only-failures', action='store_true', help='with --png-dir: pictures of failures only')
    args = parser.parse_args()
    vehicle = {'vehicle': args.vehicle, 'vehicle_params': {k: float(v) for k, v in (a.split('=', 1) for a in args.vehicle_param)}}
    sensors = {'edge_dropout': args.edge_dropout, 'grazing_angle_deg': args.grazing_angle_deg,
               'grazing_dropout': args.grazing_dropout}
    keys = ('success', 'route_completed', 'start_error', 'end_reason', 'contacts', 'min_board_gap_m', 'min_cone_gap_m', 'final', 'time_s',
            'vehicle_model')
    if args.matrix:
        for name in sorted(LAYOUTS):
            for speed, decel, delay in ((1., .5, .15), (1.2, .35, .3), (.8, .7, .15)):
                summary, _, _ = run(Course(**LAYOUTS[name]), speed_scale=speed, decel=decel, delay=delay,
                                    max_s=args.max_s, seed=args.seed, right_curvature_max=args.right_curvature_max,
                                    steer_backlash=args.steer_backlash, **sensors, **vehicle)
                print(name, f'speed{speed} decel{decel} delay{delay}',
                      json.dumps({k: summary[k] for k in keys}, ensure_ascii=False), flush=True)
        return
    if args.range:
        passed = total = 0
        for name, spec in variations():
            course = Course(**spec)
            summary, log, _ = run(course, max_s=args.max_s, seed=args.seed,
                                  right_curvature_max=args.right_curvature_max,
                                  steer_backlash=args.steer_backlash, **sensors, **vehicle)
            if args.png_dir and not (args.only_failures and summary['success']):
                Path(args.png_dir).mkdir(parents=True, exist_ok=True)
                render(course, log, str(Path(args.png_dir)/f'{name}.png'), name=name, summary=summary)
            total += 1
            passed += summary['success']
            print(name, json.dumps({k: summary[k] for k in keys}, ensure_ascii=False), flush=True)
        print(f'success {passed}/{total}')
        return
    if args.variant:
        spec = dict(dict(variations())[args.variant])
        args.layout = args.variant
    else:
        spec = dict(LAYOUTS[args.layout])
    if args.cones:
        v = [float(a) for a in args.cones.split(',')]
        spec['cones'] = [(v[0], v[1]), (v[2], v[3])]
    if args.start:
        v = [float(a) for a in args.start.split(',')]
        spec['start'] = (v[0], v[1], math.radians(v[2]))
    course = Course(**spec)
    summary, log, _ = run(course, max_s=args.max_s, speed_scale=args.speed_scale, decel=args.decel,
                          delay=args.delay, seed=args.seed, verbose=args.verbose, trace=args.trace,
                          right_curvature_max=args.right_curvature_max,
                          steer_backlash=args.steer_backlash, **sensors, **vehicle)
    print(json.dumps(summary, ensure_ascii=False, indent=1))
    if args.log:
        Path(args.log).write_text(''.join('\t'.join(str(round(v, 3)) if isinstance(v, float) else str(v)
                                                    for v in row)+'\n' for row in log))
    if args.png:
        render(course, log, args.png, name=args.layout, summary=summary)


if __name__ == '__main__':
    main()
