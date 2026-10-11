#!/usr/bin/env python3
"""Rebuild a trial's driven path from its saved lidar scans (read-only analysis).

Usage (from the repository root):
    python3 -I scripts/trial-map.py work/vehicle-dynamic-deploy-20261008/left-cone-trial-20261011-49
    python3 -I scripts/trial-map.py <trial-dir> --png /tmp/t49-map.png --panels 5.3,5.9,6.6

Every saved scan (before-lidar.json, feedback.jsonl, after.json) is matched to
the map built from the earlier ones with 2-D point-to-point ICP. The world frame
is the car frame at the first scan: +x = the start heading (up in the PNG),
+y = left of the start heading (left in the PNG). It never contacts the vehicle
and needs no third-party package (the PNG is written with zlib).

Output:
- a table per scan: time from the first scan (lidar at_ms), lidar position,
  heading (left positive), speed, yaw rate, measured path curvature
  (yaw change / distance, left positive), phase, stage, motor, servo, ICP fit.
  The curvature next to the commanded servo is the only measured
  steering-to-path relation we have; use it instead of guessing PWM gains.
- --png: top-down map. Gray = all walls/cones seen, black = the last at-rest
  scan, colored line = lidar path by phase, boxes = measured body outline
  (front .20, rear .18, sides .14 m from the lidar) with time labels, red
  squares = first target positions, blue squares = second target positions.
- --panels t1,t2: car-frame snapshots at those times (forward up, left left,
  green = body box, pink rays = null bins, red/blue = tracked targets).

Caveats: the path drifts a few cm per meter, worse where only parallel walls
are visible (sliding along a corridor is unobservable). Frames with fit < 0.6
or rms > .04 m are flagged with '!' and are not added to the map. The
observer saved about every other scan, so speed/yaw are over ~150 ms.
"""
import argparse
import json
import math
from pathlib import Path
import struct
import zlib

FRONT, REAR, SIDE = .20, .18, .14
PHASE_COLORS = {'presteer': (40, 90, 230), 'drive': (0, 150, 0), 'quality_wait': (240, 140, 0),
                'coast': (150, 60, 200), 'locked': (220, 0, 0), 'idle': (120, 120, 120)}
FONT = {
    '0': ('111', '101', '101', '101', '111'), '1': ('010', '110', '010', '010', '111'),
    '2': ('111', '001', '111', '100', '111'), '3': ('111', '001', '111', '001', '111'),
    '4': ('101', '101', '111', '001', '001'), '5': ('111', '100', '111', '001', '111'),
    '6': ('111', '100', '111', '101', '111'), '7': ('111', '001', '001', '001', '001'),
    '8': ('111', '101', '111', '101', '111'), '9': ('111', '101', '111', '001', '111'),
    '.': ('000', '000', '000', '000', '010'), '-': ('000', '000', '111', '000', '000'),
    ' ': ('000', '000', '000', '000', '000'), '_': ('000', '000', '000', '000', '111'),
    'A': ('010', '101', '111', '101', '101'), 'B': ('110', '101', '110', '101', '110'),
    'C': ('011', '100', '100', '100', '011'), 'D': ('110', '101', '101', '101', '110'),
    'E': ('111', '100', '110', '100', '111'), 'F': ('111', '100', '110', '100', '100'),
    'G': ('011', '100', '101', '101', '011'), 'H': ('101', '101', '111', '101', '101'),
    'I': ('111', '010', '010', '010', '111'), 'K': ('101', '101', '110', '101', '101'),
    'L': ('100', '100', '100', '100', '111'), 'M': ('101', '111', '111', '101', '101'),
    'N': ('110', '101', '101', '101', '101'), 'O': ('010', '101', '101', '101', '010'),
    'P': ('110', '101', '110', '100', '100'), 'Q': ('010', '101', '101', '110', '011'),
    'R': ('110', '101', '110', '101', '101'), 'S': ('011', '100', '010', '001', '110'),
    'T': ('111', '010', '010', '010', '010'), 'U': ('101', '101', '101', '101', '111'),
    'V': ('101', '101', '101', '101', '010'), 'W': ('101', '101', '111', '111', '101'),
    'Y': ('101', '101', '010', '010', '010'), 'Z': ('111', '001', '010', '100', '111')}


class Image:
    def __init__(self, width, height):
        self.w, self.h = width, height
        self.px = bytearray([255])*(width*height*3)

    def put(self, u, v, color):
        if 0 <= u < self.w and 0 <= v < self.h:
            i = (v*self.w+u)*3
            self.px[i:i+3] = bytes(color)

    def dot(self, u, v, color, r=1):
        for du in range(-r, r+1):
            for dv in range(-r, r+1):
                self.put(u+du, v+dv, color)

    def line(self, a, b, color):
        steps = max(abs(b[0]-a[0]), abs(b[1]-a[1]), 1)
        for k in range(steps+1):
            self.put(round(a[0]+(b[0]-a[0])*k/steps), round(a[1]+(b[1]-a[1])*k/steps), color)

    def text(self, u, v, s, color, scale=2):
        for ch in s.upper():
            glyph = FONT.get(ch, FONT[' '])
            for row, bits in enumerate(glyph):
                for col, bit in enumerate(bits):
                    if bit == '1':
                        for a in range(scale):
                            for b in range(scale):
                                self.put(u+col*scale+a, v+row*scale+b, color)
            u += 4*scale

    def save(self, path):
        raw = b''.join(b'\x00'+bytes(self.px[y*self.w*3:(y+1)*self.w*3]) for y in range(self.h))

        def chunk(tag, data):
            return struct.pack('>I', len(data))+tag+data+struct.pack('>I', zlib.crc32(tag+data) & 0xffffffff)
        Path(path).write_bytes(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR', struct.pack('>IIBBBBB', self.w, self.h, 8, 2, 0, 0, 0))
                               + chunk(b'IDAT', zlib.compress(raw, 9))+chunk(b'IEND', b''))


def car_points(ranges, stride=1):
    """Lidar bin i is i degrees clockwise from forward; car frame x forward, y left."""
    points = []
    for i in range(0, len(ranges), stride):
        r = ranges[i]
        if r is None or not .12 < r < 6.:
            continue
        a = math.radians(i)
        points.append((r*math.cos(a), -r*math.sin(a)))
    return points


def to_world(pose, p):
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    return (x+c*p[0]-s*p[1], y+s*p[0]+c*p[1])


class Grid:
    """Voxel-filtered point map with bucketed nearest-neighbor search."""
    def __init__(self, voxel=.03, cell=.25):
        self.voxel, self.cell = voxel, cell
        self.voxels, self.buckets = set(), {}

    def add(self, p):
        key = (math.floor(p[0]/self.voxel), math.floor(p[1]/self.voxel))
        if key in self.voxels:
            return
        self.voxels.add(key)
        self.buckets.setdefault((math.floor(p[0]/self.cell), math.floor(p[1]/self.cell)), []).append(p)

    def nearest(self, p, gate):
        cx, cy = math.floor(p[0]/self.cell), math.floor(p[1]/self.cell)
        best, best_d2 = None, gate*gate
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for q in self.buckets.get((cx+dx, cy+dy), ()):
                    d2 = (q[0]-p[0])**2+(q[1]-p[1])**2
                    if d2 < best_d2:
                        best, best_d2 = q, d2
        return best, best_d2


def icp(points, grid, pose, iterations=30):
    x, y, th = pose
    for it in range(iterations):
        gate = .30 if it < 6 else .15 if it < 12 else .07
        c, s = math.cos(th), math.sin(th)
        pairs = []
        for p in points:
            w = (x+c*p[0]-s*p[1], y+s*p[0]+c*p[1])
            q, d2 = grid.nearest(w, gate)
            if q is not None:
                pairs.append((d2, w, q))
        if len(pairs) < 15:
            break
        pairs.sort(key=lambda t: t[0])
        pairs = pairs[:max(15, int(len(pairs)*.85))]
        n = len(pairs)
        mx, my = sum(t[1][0] for t in pairs)/n, sum(t[1][1] for t in pairs)/n
        qx, qy = sum(t[2][0] for t in pairs)/n, sum(t[2][1] for t in pairs)/n
        sxx = sum((t[1][0]-mx)*(t[2][0]-qx) for t in pairs)
        sxy = sum((t[1][0]-mx)*(t[2][1]-qy) for t in pairs)
        syx = sum((t[1][1]-my)*(t[2][0]-qx) for t in pairs)
        syy = sum((t[1][1]-my)*(t[2][1]-qy) for t in pairs)
        dth = math.atan2(sxy-syx, sxx+syy)
        cd, sd = math.cos(dth), math.sin(dth)
        nx, ny = cd*(x-mx)-sd*(y-my)+qx, sd*(x-mx)+cd*(y-my)+qy
        moved = math.hypot(nx-x, ny-y)
        x, y, th = nx, ny, th+dth
        if it >= 12 and moved < 5e-4 and abs(dth) < 1e-3:
            break
    c, s = math.cos(th), math.sin(th)
    inliers = []
    for p in points:
        q, d2 = grid.nearest((x+c*p[0]-s*p[1], y+s*p[0]+c*p[1]), .06)
        if q is not None:
            inliers.append(d2)
    fit = len(inliers)/max(1, len(points))
    rms = math.sqrt(sum(inliers)/len(inliers)) if inliers else None
    return (x, y, th), fit, rms


def load_frames(base):
    frames, seen = [], set()

    def add(scan, state, source):
        if not isinstance(scan, dict) or scan.get('seq') in seen or not isinstance(scan.get('ranges'), list):
            return
        seen.add(scan['seq'])
        frames.append({'scan': scan, 'state': state, 'source': source})
    before_lidar, before = base/'before-lidar.json', base/'before.json'
    if before_lidar.exists():
        add(json.loads(before_lidar.read_text()).get('scan'),
            json.loads(before.read_text()) if before.exists() else None, 'before')
    for line in (base/'feedback.jsonl').read_text().splitlines():
        row = json.loads(line)
        if 'state' in row and 'lidar' in row:
            add(row['lidar'].get('scan'), row['state'], 'feedback')
    after = base/'after.json'
    if after.exists():
        for row in json.loads(after.read_text()):
            add(row['lidar'].get('scan'), row['state'], 'after')
    frames.sort(key=lambda f: f['scan']['seq'])
    return frames


def frame_info(state):
    info = {'phase': 'idle', 'stage': '', 'reason': '', 'motor': None, 'servo': None, 'first': None, 'second': None}
    if not isinstance(state, dict):
        return info
    control = (state.get('status') or {}).get('control') or {}
    info['motor'], info['servo'] = control.get('motor'), control.get('servo')
    active = (state.get('autonomy') or {}).get('active')
    if not isinstance(active, dict):
        info['phase'] = 'locked' if (state.get('autonomy') or {}).get('mode') == 'locked' else 'idle'
        return info
    turn = active.get('turn') or {}
    info.update(phase=turn.get('phase') or 'idle', stage=turn.get('route_stage') or '', reason=turn.get('reason') or '')
    for key, name in (('compact_target', 'first'), ('second_target', 'second')):
        target = turn.get(key)
        if isinstance(target, dict) and isinstance(target.get('point_left_m'), list):
            info[name] = tuple(target['point_left_m'][:2])
    return info


def track(frames):
    grid = Grid()
    poses, prev, prev2 = [], None, None
    for k, frame in enumerate(frames):
        points = car_points(frame['scan']['ranges'])
        if k == 0:
            pose, fit, rms = (0., 0., 0.), 1., 0.
        else:
            if prev2 is None:
                guess = prev
            else:
                # constant velocity in the previous car frame
                dx, dy = prev[0]-prev2[0], prev[1]-prev2[1]
                c, s = math.cos(-prev2[2]), math.sin(-prev2[2])
                lx, ly, dth = c*dx-s*dy, s*dx+c*dy, prev[2]-prev2[2]
                c, s = math.cos(prev[2]), math.sin(prev[2])
                guess = (prev[0]+c*lx-s*ly, prev[1]+s*lx+c*ly, prev[2]+dth)
            sub = points[::2]
            pose, fit, rms = icp(sub, grid, guess)
            if fit < .6:
                for offset in (-.17, .17, -.35, .35):
                    trial = icp(sub, grid, (guess[0], guess[1], guess[2]+offset))
                    if trial[1] > fit:
                        pose, fit, rms = trial
        good = fit >= .6 and (rms is None or rms <= .04)
        if k == 0 or good:
            for p in points:
                grid.add(to_world(pose, p))
        poses.append({'pose': pose, 'fit': fit, 'rms': rms, 'good': k == 0 or good, 'points': points})
        prev2, prev = prev, pose
    return poses


def table(frames, poses):
    t0 = frames[0]['scan']['at_ms']
    rows = []
    for k, (frame, est) in enumerate(zip(frames, poses)):
        info = frame_info(frame['state'])
        t = (frame['scan']['at_ms']-t0)/1000
        speed = yaw = curvature = None
        if k > 0:
            dt = (frame['scan']['at_ms']-frames[k-1]['scan']['at_ms'])/1000
            (x0, y0, th0), (x1, y1, th1) = poses[k-1]['pose'], est['pose']
            ds = math.hypot(x1-x0, y1-y0)
            dth = math.atan2(math.sin(th1-th0), math.cos(th1-th0))
            if dt > 0:
                speed, yaw = ds/dt, math.degrees(dth)/dt
            if ds > .03:
                # sign of travel: forward motion along the mean heading
                forward = (x1-x0)*math.cos((th0+th1)/2)+(y1-y0)*math.sin((th0+th1)/2)
                curvature = dth/ds if forward > 0 else -dth/ds
        rows.append({'t': t, 'seq': frame['scan']['seq'], 'pose': est['pose'], 'speed': speed, 'yaw': yaw,
                     'curvature': curvature, 'fit': est['fit'], 'rms': est['rms'], 'good': est['good'],
                     'source': frame['source'], **info})
    return rows


def print_table(rows):
    print(' t(s)   seq   x(m)   y(m) head(deg) v(m/s) yaw(deg/s) kappa(1/m) phase        stage        motor servo fit   reason')
    for r in rows:
        x, y, th = r['pose']
        fmt = lambda v, f: ('%'+f) % v if v is not None else ' '*int(f.split('.')[0])
        print(f"{r['t']:5.2f} {r['seq']:5d} {x:6.2f} {y:6.2f} {math.degrees(th):8.1f} {fmt(r['speed'], '6.2f')} "
              f"{fmt(r['yaw'], '9.1f')} {fmt(r['curvature'], '9.2f')}  {r['phase'][:12]:12s} {r['stage'][:12]:12s} "
              f"{str(r['motor']):5s} {str(r['servo']):5s} {r['fit']:.2f}{'' if r['good'] else '!'} {r['reason'][:46]}")


def body_outline(pose):
    corners = [(FRONT, SIDE), (FRONT, -SIDE), (-REAR, -SIDE), (-REAR, SIDE)]
    return [to_world(pose, c) for c in corners]


def render_map(frames, poses, rows, path):
    world, final = [], []
    for k, est in enumerate(poses):
        pts = [to_world(est['pose'], p) for p in est['points']]
        if est['good']:
            world.extend(pts)
        if k == len(poses)-1:
            final = pts
    traj = [r['pose'] for r in rows]
    near = [p for p in world if min((p[0]-q[0])**2+(p[1]-q[1])**2 for q in traj[::3]) < 3.5**2]
    xs = [p[0] for p in near]+[q[0] for q in traj]
    ys = [p[1] for p in near]+[q[1] for q in traj]
    xmin, xmax, ymin, ymax = min(xs)-.3, max(xs)+.3, min(ys)-.3, max(ys)+.3
    scale = min(1100/max(xmax-xmin, .1), 1100/max(ymax-ymin, .1), 160)
    legend_h = 40
    img = Image(int((ymax-ymin)*scale)+1, int((xmax-xmin)*scale)+1+legend_h)

    def uv(p):
        return int((ymax-p[1])*scale), int((xmax-p[0])*scale)+legend_h
    # 0.5 m grid
    gx = math.floor(xmin*2)/2
    while gx <= xmax:
        for yy in range(int(ymin*100), int(ymax*100), 3):
            u, v = uv((gx, yy/100))
            img.put(u, v, (228, 228, 228))
        gx += .5
    gy = math.floor(ymin*2)/2
    while gy <= ymax:
        for xx in range(int(xmin*100), int(xmax*100), 3):
            u, v = uv((xx/100, gy))
            img.put(u, v, (228, 228, 228))
        gy += .5
    for p in near:
        u, v = uv(p)
        img.put(u, v, (165, 165, 165))
    for p in final:
        u, v = uv(p)
        img.dot(u, v, (0, 0, 0), 1)
    last_label = -9.
    for k, r in enumerate(rows):
        color = PHASE_COLORS.get(r['phase'], (120, 120, 120))
        outline = [uv(c) for c in body_outline(r['pose'])]
        light = tuple(min(255, c+(255-c)*2//3) for c in color)
        labelled = r['t']-last_label >= .45 or k == len(rows)-1
        for a, b in zip(outline, outline[1:]+outline[:1]):
            img.line(a, b, color if labelled else light)
        if k:
            img.line(uv(rows[k-1]['pose']), uv(r['pose']), color)
            img.line((uv(rows[k-1]['pose'])[0]+1, uv(rows[k-1]['pose'])[1]), (uv(r['pose'])[0]+1, uv(r['pose'])[1]), color)
        if labelled:
            u, v = uv(to_world(r['pose'], (FRONT+.05, 0)))
            img.text(u+3, v-5, f"{r['t']:.1f}", color)
            last_label = r['t']
        for key, tcolor in (('first', (220, 0, 0)), ('second', (0, 60, 230))):
            if r[key] is not None:
                u, v = uv(to_world(r['pose'], r[key]))
                img.dot(u, v, tcolor, 2)
    u = 6
    for name in ('presteer', 'drive', 'quality_wait', 'coast', 'locked'):
        img.dot(u+4, 14, PHASE_COLORS[name], 4)
        img.text(u+12, 9, name.replace('quality_wait', 'wait'), PHASE_COLORS[name])
        u += 12+4*2*len(name.replace('quality_wait', 'wait'))+14
    img.text(u, 9, '1ST', (220, 0, 0))
    img.text(u+34, 9, '2ND', (0, 60, 230))
    img.text(6, 26, 'GRID 0.5 M  UP = START HEADING  BLACK = REST SCAN', (90, 90, 90))
    img.save(path)
    return img.w, img.h


def render_panels(frames, rows, times, path):
    W, H, S = 360, 400, 90
    picks = [min(range(len(rows)), key=lambda k: abs(rows[k]['t']-t)) for t in times]
    img = Image(W*len(picks)+4*(len(picks)-1), H)
    for n, k in enumerate(picks):
        ox = n*(W+4)

        def put(x, y, color, r=0):
            img.dot(ox+int(W/2-y*S), int(H-(x+1.2)*S), color, r)
        for gx in range(-1, 4):
            for yy in range(-200, 201, 4):
                put(gx, yy/100, (225, 225, 225))
        for gy in range(-2, 3):
            for xx in range(-120, 321, 4):
                put(xx/100, gy, (225, 225, 225))
        for xx in range(-18, 21):
            put(xx/100, SIDE, (0, 160, 0)), put(xx/100, -SIDE, (0, 160, 0))
        for yy in range(-14, 15):
            put(FRONT, yy/100, (0, 160, 0)), put(-REAR, yy/100, (0, 160, 0))
        for i, d in enumerate(frames[k]['scan']['ranges']):
            a = math.radians(i)
            if d is None:
                for rr in range(5, 60, 6):
                    put(rr/100*math.cos(a), -rr/100*math.sin(a), (255, 170, 170))
            elif d < 6:
                put(d*math.cos(a), -d*math.sin(a), (0, 0, 0), 1)
        for key, color in (('first', (220, 0, 0)), ('second', (0, 60, 230))):
            if rows[k][key] is not None:
                put(rows[k][key][0], rows[k][key][1], color, 4)
        img.text(ox+6, 6, f"{rows[k]['t']:.2f} {rows[k]['phase'][:12]}", (60, 60, 60))
        if n:
            for v in range(H):
                for du in range(1, 5):
                    img.put(ox-du, v, (64, 64, 64))
    img.save(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('trial_dir')
    parser.add_argument('--png', help='write the top-down map PNG here')
    parser.add_argument('--panels', help='comma-separated times (s, same clock as the table) for car-frame snapshots')
    args = parser.parse_args()
    base = Path(args.trial_dir)
    frames = load_frames(base)
    poses = track(frames)
    rows = table(frames, poses)
    print_table(rows)
    if args.png:
        w, h = render_map(frames, poses, rows, args.png)
        print('map', args.png, f'{w}x{h}')
    if args.panels:
        out = (args.png or str(base.name)+'-map.png').replace('.png', '')+'-panels.png'
        render_panels(frames, rows, [float(t) for t in args.panels.split(',')], out)
        print('panels', out)


if __name__ == '__main__':
    main()
