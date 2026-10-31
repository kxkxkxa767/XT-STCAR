"""Mac (round y11, 2026-10-31): camera height, pitch, roll and distance from
the board taped flat on a vertical wall, with the lens model of
scripts/camera-calibration-solve.py. The board's top row may be cut off by the
frame: the visible inner corners are found with SB 'larger' and ordered by
image position (the lowest visible row is the board's lowest inner row, one
square above its edge). 10-31: lowest squares' edge .33 m above the floor,
front .415 m and glass .454 m from the wall: height .1924, pitch 6.38 deg -
the pitch 2.2 deg below every track reference (the board likely leaned off
the wall): check the pitch on the track (far board bases at lidar distances).

  .venv/bin/python scripts/camera-wall-pose.py SOLVE_RESULT_JSON EDGE_M WALL_TO_GLASS_M WALL_TO_FRONT_M IMAGE ...
"""
import json
import sys
from pathlib import Path

import cv2
import numpy as np

SQUARE_M = .025
LOWEST_EDGE_M, WALL_TO_GLASS_M, WALL_TO_FRONT_M = (float(v) for v in sys.argv[2:5])
model = json.loads(Path(sys.argv[1]).read_text())['models']['pinhole_k5']
K, dist = np.array(model['K']), np.array(model['dist'])
rows_out = []
for path in sys.argv[5:]:
    gray = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    ok, corners, meta = cv2.findChessboardCornersSBWithMeta(
        gray, (6, 5), cv2.CALIB_CB_LARGER | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
    if not ok:
        rows_out.append({'image': Path(path).name, 'found': False})
        continue
    c = corners.reshape(-1, 2).astype(np.float64)
    n_rows, n_cols = meta.shape
    grid = c.reshape(n_rows, n_cols, 2)
    # make axis 0 the image-vertical direction, lowest row last
    if np.ptp(grid[:, 0, 1]) < np.ptp(grid[0, :, 1]):
        grid = grid.transpose(1, 0, 2)
    if grid[0, 0, 1] > grid[-1, 0, 1]:
        grid = grid[::-1]
    if grid[0, 0, 0] > grid[0, -1, 0]:
        grid = grid[:, ::-1]
    nv, nh = grid.shape[:2]
    image_points, object_points = [], []
    for i in range(nv):
        height = LOWEST_EDGE_M+SQUARE_M*(nv-i)   # lowest image row: one square above the edge
        for j in range(nh):
            image_points.append(grid[i, j])
            object_points.append((j*SQUARE_M, height, 0.))  # x right, y up, z out of the wall
    image_points = np.array(image_points, np.float64)
    object_points = np.array(object_points, np.float64)
    ok, rvec, tvec = cv2.solvePnP(object_points, image_points, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    projected, _ = cv2.projectPoints(object_points, rvec, tvec, K, dist)
    rms = float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2)-image_points)**2, axis=1))))
    R, _ = cv2.Rodrigues(rvec)
    centre = (-R.T @ tvec).ravel()            # camera centre in the wall frame (y = height above the floor)
    z_axis, x_axis = R[2, :], R[0, :]         # optical axis and image right in the wall frame
    up = np.array([0., 1., 0.])
    rows_out.append({
        'image': Path(path).name, 'found': True, 'corners': f'{nv}x{nh}', 'rms_px': round(rms, 3),
        'height_m': round(float(centre[1]), 4), 'wall_distance_m': round(float(centre[2]), 4),
        'pitch_up_deg': round(float(np.degrees(np.arcsin(z_axis @ up))), 3),
        'roll_deg': round(float(np.degrees(np.arcsin(-(x_axis @ up)))), 3),
        'yaw_deg': round(float(np.degrees(np.arctan2(z_axis[0], -z_axis[2]))), 2),
        'centre_glass_m': round(float(centre[2])-WALL_TO_GLASS_M, 4),
        'camera_behind_front_m': round(float(centre[2])-WALL_TO_FRONT_M, 4)})
for row in rows_out:
    print(json.dumps(row))
found = [r for r in rows_out if r['found']]
keys = ('height_m', 'wall_distance_m', 'pitch_up_deg', 'roll_deg', 'yaw_deg', 'camera_behind_front_m')
print(json.dumps({'frames': len(found), 'median': {k: round(float(np.median([r[k] for r in found])), 4) for k in keys},
                  'spread': {k: round(float(np.ptp([r[k] for r in found])), 4) for k in keys}}))
