#!/usr/bin/env python3
"""Before a run (round y11, 2026-10-31): is the glued camera still at its
calibrated pitch? Read only: one console /camera.jpg and one /api/lidar from
the car over the SSH control socket (the access token is read on the car and
never printed), on the Mac the base of the board straight ahead (the lowest
blue run above row 340 in the centre columns) against the lidar's distance to
it gives the camera's pitch with the calibrated lens geometry and mount.
10-31 on the track: 8.50 deg (start, board 7.70 m from the camera) and 8.60
(light trigger, 3.92 m); vision-log ranges with 9.1 for race4's box bottoms.
A change of more than PITCH_TOLERANCE_DEG moves the crosswalk distances
(~.5 m per degree at 2.3 m): recalibrate before the run.

  .venv/bin/python scripts/camera-pitch-check.py [OUT.jpg]
  .venv/bin/python scripts/camera-pitch-check.py --image JPG --lidar-m M   (offline: a saved frame)
"""
import base64
import json
import math
from pathlib import Path
import statistics
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
SSH = ['ssh', '-S', str(ROOT/'work/vehicle-test-ssh.sock'), '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5',
       'bianbu@192.168.0.156', 'python3', '-']
CAMERA_AHEAD_OF_LIDAR_M = .159  # front .20 ahead of the lidar, the lens .041 behind the front
PHYSICAL_PITCH_DEG = 8.55  # the two board bases of 10-31
PITCH_TOLERANCE_DEG = .4
REMOTE = r'''
import base64, json, urllib.request
from pathlib import Path
access = json.loads((Path('/home/bianbu/xt-stcar-console')/'access.json').read_text())
opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
def get(path):
    r = urllib.request.Request('http://127.0.0.1:%d%s' % (access['port'], path), headers={'X-Control-Token': access['token']})
    with opener.open(r, timeout=3) as f: return f.read()
lidar = json.loads(get('/api/lidar'))
print(json.dumps({'jpg': base64.b64encode(get('/camera.jpg')).decode(), 'lidar': lidar}))
'''


def undistort(u, v, intrinsics, distortion):
    """Normalized undistorted image point of pixel (u, v), or None where the
    fixed-point iteration does not reproduce the pixel (the bottom corners)."""
    fx, fy, cx, cy = intrinsics
    k1, k2, p1, p2, k3 = distortion
    xd, yd = (u-cx)/fx, (v-cy)/fy
    x, y = xd, yd
    for _ in range(40):
        r2 = x*x+y*y
        radial = 1+r2*(k1+r2*(k2+r2*k3))
        x, y = (xd-2*p1*x*y-p2*(r2+2*x*x))/radial, (yd-p1*(r2+2*y*y)-2*p2*x*y)/radial
    r2 = x*x+y*y
    radial = 1+r2*(k1+r2*(k2+r2*k3))
    if (abs(fx*(x*radial+2*p1*x*y+p2*(r2+2*x*x))+cx-u) > 1e-3
            or abs(fy*(y*radial+p1*(r2+2*y*y)+2*p2*x*y)+cy-v) > 1e-3):
        return None
    return x, y


def pitch_from_board(u, v, distance, calibration):
    lens = calibration['intrinsics']
    point = undistort(u, v, lens['fx_fy_cx_cy'], lens['distortion_k1_k2_p1_p2_k3'])
    if point is None or distance <= 0:
        return None
    height = calibration['extrinsics']['lens_centre_height_m']
    return math.degrees(math.atan(point[1]) - math.atan(height / distance))


def main():
    calibration = json.loads((ROOT/"config/camera-race4-20261031.json").read_text())
    lens = calibration["intrinsics"]
    import cv2
    import numpy as np
    if sys.argv[1:2] == ['--image']:
        data, lidar_m = Path(sys.argv[2]).read_bytes(), float(sys.argv[sys.argv.index('--lidar-m')+1])
    else:
        got = json.loads(subprocess.run(SSH, input=REMOTE, capture_output=True, text=True, check=True,
                                        timeout=30).stdout)
        data = base64.b64decode(got['jpg'])
        if len(sys.argv) > 1:
            Path(sys.argv[1]).write_bytes(data)
        scan = got['lidar']['scan']
        ranges = scan['ranges'] if isinstance(scan, dict) else scan
        ahead = [ranges[i % 360] for i in range(-3, 4) if ranges[i % 360]]
        if len(ahead) < 4:
            raise SystemExit('no lidar return straight ahead')
        lidar_m = statistics.median(ahead)
    image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR).astype(int)
    rows = []
    for u in range(330, 391, 10):
        blue = (image[200:400, u-1:u+2, 0]-image[200:400, u-1:u+2, 2]).mean(axis=1) > 45
        idx = [i+200 for i in range(len(blue)) if blue[i]]
        runs, run = [], []
        for i in idx:
            if run and i != run[-1]+1:
                runs.append(run)
                run = []
            run.append(i)
        if run:
            runs.append(run)
        runs = [r for r in runs if len(r) > 5 and r[-1] < 340]
        if runs:
            rows.append(runs[-1][-1]+.5)  # the board's base: its lowest blue row and a half
    if len(rows) < 4:
        raise SystemExit(f'no board base found in the centre columns ({rows})')
    row = statistics.median(rows)
    distance = lidar_m-CAMERA_AHEAD_OF_LIDAR_M
    pitch = pitch_from_board(lens['fx_fy_cx_cy'][2], row, distance, calibration)
    if pitch is None:
        raise SystemExit('board base is outside the calibrated image or lidar range')
    result = {'board_base_row': row, 'rows': rows, 'lidar_ahead_m': round(lidar_m, 3),
              'pitch_deg': round(pitch, 2), 'calibrated_pitch_deg': PHYSICAL_PITCH_DEG,
              'ok': abs(pitch-PHYSICAL_PITCH_DEG) <= PITCH_TOLERANCE_DEG}
    print(json.dumps(result))
    return 0 if result['ok'] else 1


if __name__ == '__main__':
    sys.exit(main())
