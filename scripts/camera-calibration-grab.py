"""Car side, read only (round y11, 2026-10-31): camera calibration frames
through the console's own /camera.jpg (the console keeps the camera; the
access token is read here and never printed). Moves nothing.

  python3 camera-calibration-grab.py check OUT_DIR             one frame: find the 9x6 board, draw it
  python3 camera-calibration-grab.py collect OUT_DIR SECONDS   keep frames whose board pose differs (>12 px)
  python3 camera-calibration-grab.py files OUT_DIR IMAGE ...   the board in given images (console captures)

OUT_DIR is new. Copy the OUT_DIRs to the Mac and solve there
(scripts/camera-calibration-solve.py; the car's CPU took >9 min for what the
Mac does in seconds). The board: 9x6 inner corners, 25 mm squares (A4).
"""
import json
import os
import sys
import time
import urllib.request
from pathlib import Path

os.environ['OPENCV_OPENCL_DEVICE'] = 'disabled'  # its kernels fail to build here
import cv2
import numpy as np

BASE = Path('/home/bianbu/xt-stcar-console')
PATTERN, SQUARE_M, SIZE = (9, 6), .025, (640, 480)
access = json.loads((BASE/'access.json').read_text())
opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def grab():
    request = urllib.request.Request('http://127.0.0.1:%d/camera.jpg' % access['port'],
                                     headers={'X-Control-Token': access['token']})
    with opener.open(request, timeout=3) as response:
        return response.read()


cv2.ocl.setUseOpenCL(False)


def detect(gray):
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
    ok, corners = cv2.findChessboardCorners(gray, PATTERN, flags)
    if ok:
        criteria = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 40, .01)
        return cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria), 'classic'
    ok, corners = cv2.findChessboardCornersSB(gray, PATTERN, cv2.CALIB_CB_ACCURACY)
    if ok:
        return corners, 'sb'
    return None, None


def describe(corners):
    c = corners.reshape(-1, 2)
    x0, y0 = c.min(0)
    x1, y1 = c.max(0)
    return {'box': [round(float(v), 1) for v in (x0, y0, x1, y1)], 'centre': [round(float(v), 1) for v in c.mean(0)],
            'square_px': round(float(np.linalg.norm(c[1]-c[0])), 1)}


def check(out):
    out.mkdir(mode=0o700)
    data = grab()
    (out/'check.jpg').write_bytes(data)
    image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    corners, how = detect(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
    if corners is not None:
        cv2.drawChessboardCorners(image, PATTERN, corners, True)
        np.save(out/'check-corners.npy', corners)
    cv2.imwrite(str(out/'check-ann.jpg'), image, [cv2.IMWRITE_JPEG_QUALITY, 85])
    print(json.dumps({'found': corners is not None, 'how': how, **(describe(corners) if corners is not None else {})}))


def collect(out, seconds):
    out.mkdir(mode=0o700)
    kept, tried, last, started = [], 0, None, time.monotonic()
    while time.monotonic()-started < seconds:
        data = grab()
        if data == last:
            time.sleep(.05)
            continue
        last, tried = data, tried+1
        image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        corners, how = detect(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
        if tried % 10 == 0:
            print(json.dumps({'tried': tried, 't': round(time.monotonic()-started, 1), 'kept': len(kept)}), flush=True)
        if corners is None:
            if tried % 15 == 0:
                (out/('miss-%04d.jpg' % tried)).write_bytes(data)
            continue
        c = corners.reshape(-1, 2)
        # a new pose: some corner moved more than 12 px from every kept view
        if any(np.abs(c-k).max() < 12 for k in kept):
            continue
        kept.append(c)
        name = 'view-%03d' % len(kept)
        (out/(name+'.jpg')).write_bytes(data)
        np.save(out/(name+'.npy'), corners)
        print(json.dumps({'kept': len(kept), 't': round(time.monotonic()-started, 1), 'how': how, **describe(corners)}),
              flush=True)
    print(json.dumps({'tried': tried, 'kept': len(kept), 'coverage': coverage(kept)}), flush=True)


def coverage(views):
    """Corners per cell of a 4x3 grid over the image (the corners matter most)."""
    grid = np.zeros((3, 4), int)
    for c in views:
        for x, y in c:
            grid[min(int(y/SIZE[1]*3), 2), min(int(x/SIZE[0]*4), 3)] += 1
    return grid.tolist()


if __name__ == '__main__':
    mode = sys.argv[1]
    if mode == 'check':
        check(Path(sys.argv[2]))
    elif mode == 'files':
        files(Path(sys.argv[2]), sys.argv[3:])
    elif mode == 'collect':
        collect(Path(sys.argv[2]), float(sys.argv[3]))
    else:
        solve(sys.argv[2:])
