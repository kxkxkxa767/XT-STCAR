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

    def cast(self, ox, oy, angle):
        dx, dy = math.cos(angle), math.sin(angle)
        best = None
        for (x1, y1), (x2, y2) in self.segments:
            ex, ey = x2-x1, y2-y1
            den = dx*ey-dy*ex
            if abs(den) < 1e-12:
                continue
            t = ((x1-ox)*ey-(y1-oy)*ex)/den
            u = ((x1-ox)*dy-(y1-oy)*dx)/den
            if t > 1e-6 and -1e-9 <= u <= 1+1e-9 and (best is None or t < best):
                best = t
        for cx, cy, r in self.cones:
            fx, fy = ox-cx, oy-cy
            b = fx*dx+fy*dy
            disc = b*b-(fx*fx+fy*fy-r*r)
            if disc >= 0:
                t = -b-math.sqrt(disc)
                if t > 1e-6 and (best is None or t < best):
                    best = t
        return best

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
    def __init__(self, x, y, psi, *, speed_scale=1., decel=.5, delay=.15, steer_tau=.1, seed=0):
        self.x, self.y, self.psi, self.v = x, y, psi, 0.
        self.speed_scale, self.decel, self.delay, self.steer_tau = speed_scale, decel, delay, steer_tau
        self.servo_queue = []  # (time, servo) commands, applied after the delay
        self.servo_phys = 1500.
        self.rng = random.Random(seed)

    def target_speed(self, motor):
        return {1550: 1.0, 1560: 1.35, 1570: 1.5}.get(motor, 0.)*self.speed_scale

    def step(self, t, dt, motor, servo):
        self.servo_queue.append((t, servo))
        applied = self.servo_phys
        while self.servo_queue and self.servo_queue[0][0] <= t-self.delay:
            applied = self.servo_queue.pop(0)[1]
            self._delayed = applied
        target = getattr(self, '_delayed', 1500)
        self.servo_phys += (target-self.servo_phys)*min(1., dt/self.steer_tau)
        kappa = max(-.94, min(1.375, (self.servo_phys-1500)/160))
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
        verbose=False, initial_presteer_pwm=None, max_drive_s=24.):
    sys.path.insert(0, str(CONSOLE))
    server = load('course_sim_server', CONSOLE/'server.py')
    console = make_console(server)
    geometry = Geometry()
    rng = random.Random(seed)
    car = Vehicle(*course.start, speed_scale=speed_scale, decel=decel, delay=delay, seed=seed)
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
        for i in range(360):
            x, y, psi = pose_at(t-distortion*(359-i)/360*.1)
            r = course.cast(x, y, psi-math.radians(i))
            if r is None or r > 12 or r < .02 or rng.random() < .002:
                ranges.append(None)
            else:
                ranges.append(round(r+rng.gauss(0, noise), 4))
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
    while start_error is None and t < max_s:
        t = round(t+dt, 6)
        clock['now'] = t
        control = console.status['control']
        motor = control['motor'] if control.get('armed') else 1500
        servo = control['servo'] if control.get('armed') else 1500
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
               'final': (round(car.x, 2), round(car.y, 2), round(math.degrees(car.psi), 1)),
               'final_front_x': round(front_x, 2), 'in_finish_zone': in_finish, 'time_s': t,
               'success': in_finish and not any(c[1] == 'board' for c in contacts)}
    sys.path.remove(str(CONSOLE))
    return summary, log, history


def render(course, log, path, scale=90):
    tm = load('course_sim_map', ROOT/'scripts'/'trial-map.py')
    pad = .3
    w, h = int((course.L+2*pad)*scale), int((course.W+2*pad)*scale)
    img = tm.Image(w, h)

    def uv(x, y):
        return int((x+pad)*scale), int((course.W+pad-y)*scale)
    for (x1, y1), (x2, y2) in course.segments:
        img.line(uv(x1, y1), uv(x2, y2), (60, 60, 60))
    for cx, cy, r in course.cones:
        img.dot(*uv(cx, cy), (30, 60, 220), r=max(2, int(r*scale)))
    fx = course.finish_x
    img.line(uv(fx, course.W-course.light_lane), uv(fx, course.W), (0, 160, 0))
    colors = {'drive': (0, 150, 0), 'presteer': (40, 90, 230), 'quality_wait': (240, 140, 0),
              'coast': (150, 60, 200), 'ended': (220, 0, 0)}
    previous = None
    for k, row in enumerate(log):
        t, x, y, psi = row[:4]
        point = uv(x, y)
        if previous:
            img.line(previous, point, colors.get(row[8], (120, 120, 120)))
        previous = point
        if k % 5 == 0:
            c, s = math.cos(math.radians(psi)), math.sin(math.radians(psi))
            corners = [(x+c*a-s*b, y+s*a+c*b) for a, b in ((FRONT, SIDE), (FRONT, -SIDE), (-REAR, -SIDE), (-REAR, SIDE))]
            for a, b in zip(corners, corners[1:]+corners[:1]):
                img.line(uv(*a), uv(*b), (170, 170, 170))
        if k % 10 == 0:
            img.text(point[0]+4, point[1]-12, f'{t:.0f}', (0, 0, 0), scale=1)
    img.save(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--layout', default='doc8x4', choices=sorted(LAYOUTS))
    parser.add_argument('--cones', help='x1,y1,x2,y2 override')
    parser.add_argument('--start', help='x,y,heading_deg override')
    parser.add_argument('--speed-scale', type=float, default=1.)
    parser.add_argument('--decel', type=float, default=.5)
    parser.add_argument('--delay', type=float, default=.15)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--max-s', type=float, default=45.)
    parser.add_argument('--png')
    parser.add_argument('--log')
    parser.add_argument('-v', '--verbose', action='store_true')
    parser.add_argument('--matrix', action='store_true', help='run every named layout with vehicle variations')
    parser.add_argument('--range', action='store_true', help='run the rules-range variations (nominal vehicle)')
    args = parser.parse_args()
    keys = ('success', 'start_error', 'end_reason', 'contacts', 'min_board_gap_m', 'min_cone_gap_m', 'final', 'time_s')
    if args.matrix:
        for name in sorted(LAYOUTS):
            for speed, decel, delay in ((1., .5, .15), (1.2, .35, .3), (.8, .7, .15)):
                summary, _, _ = run(Course(**LAYOUTS[name]), speed_scale=speed, decel=decel, delay=delay,
                                    max_s=args.max_s, seed=args.seed)
                print(name, f'speed{speed} decel{decel} delay{delay}',
                      json.dumps({k: summary[k] for k in keys}, ensure_ascii=False), flush=True)
        return
    if args.range:
        passed = total = 0
        for name, spec in variations():
            summary, _, _ = run(Course(**spec), max_s=args.max_s, seed=args.seed)
            total += 1
            passed += summary['success']
            print(name, json.dumps({k: summary[k] for k in keys}, ensure_ascii=False), flush=True)
        print(f'success {passed}/{total}')
        return
    spec = dict(LAYOUTS[args.layout])
    if args.cones:
        v = [float(a) for a in args.cones.split(',')]
        spec['cones'] = [(v[0], v[1]), (v[2], v[3])]
    if args.start:
        v = [float(a) for a in args.start.split(',')]
        spec['start'] = (v[0], v[1], math.radians(v[2]))
    course = Course(**spec)
    summary, log, _ = run(course, max_s=args.max_s, speed_scale=args.speed_scale, decel=args.decel,
                          delay=args.delay, seed=args.seed, verbose=args.verbose)
    print(json.dumps(summary, ensure_ascii=False, indent=1))
    if args.log:
        Path(args.log).write_text(''.join('\t'.join(str(round(v, 3)) if isinstance(v, float) else str(v)
                                                    for v in row)+'\n' for row in log))
    if args.png:
        render(course, log, args.png)


if __name__ == '__main__':
    main()
