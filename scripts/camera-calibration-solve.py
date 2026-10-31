"""Mac (round y11, 2026-10-31): calibrate the car camera (640x480) from the
9x6 board views kept by scripts/camera-calibration-grab.py (view-*.npy in the
sub-directories of DIR). Drops degenerate and duplicate views and outliers,
reports the spread over random 70 % subsets, writes DIR/solve-result.json.

  .venv/bin/python scripts/camera-calibration-solve.py DIR
"""
import glob, json, random, sys
from pathlib import Path
import cv2, numpy as np
HERE = Path(sys.argv[1]).resolve()
paths = sorted(glob.glob(str(HERE/'*'/'view-*.npy')))
grid = np.zeros((54, 3), np.float32); grid[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)*.025
names, pts = [], []
for p in paths:
    c = np.load(p).astype(np.float32)
    g = c.reshape(6, 9, 2)
    step = min(np.linalg.norm(np.diff(g, axis=1), axis=2).min(), np.linalg.norm(np.diff(g, axis=0), axis=2).min())
    if step < 3 or any(np.abs(c-q).max() < 1.5 for q in pts):
        continue  # degenerate or the same pose twice
    names.append(str(Path(p).relative_to(HERE))); pts.append(c)
def run(idx, flags):
    return cv2.calibrateCameraExtended([grid]*len(idx), [pts[i] for i in idx], (640, 480), None, None, flags=flags)
result = {'views_found': len(paths), 'views_distinct': len(pts), 'models': {}}
for model, flags in (('pinhole_k5', 0), ('rational_k8', cv2.CALIB_RATIONAL_MODEL)):
    idx = list(range(len(pts)))
    for _ in range(4):
        rms, K, d, rv, tv, sdi, sde, per = run(idx, flags)
        per = per.ravel(); lim = max(3*float(np.median(per)), .8)
        if per.max() <= lim: break
        idx = [i for i, e in zip(idx, per) if e <= lim]
    dist = sorted(float(np.linalg.norm(t)) for t in tv)
    tilt = sorted(float(np.degrees(np.arccos(abs(cv2.Rodrigues(r)[0][2, 2])))) for r in rv)
    cover = np.zeros((3, 4), int)
    for i in idx:
        for x, y in pts[i].reshape(-1, 2):
            cover[min(int(y/160), 2), min(int(x/160), 3)] += 1
    random.seed(1); fx, k1 = [], []
    for _ in range(12):
        s = random.sample(idx, int(.7*len(idx)))
        _, K2, d2, *_ = cv2.calibrateCamera([grid]*len(s), [pts[i] for i in s], (640, 480), None, None, flags=flags)
        fx.append(K2[0, 0]); k1.append(d2.ravel()[0])
    result['models'][model] = {
        'rms_px': round(rms, 4), 'views_used': len(idx), 'worst_view_px': round(float(per.max()), 3),
        'fx_fy_cx_cy': [round(float(v), 2) for v in (K[0, 0], K[1, 1], K[0, 2], K[1, 2])],
        'sd_fx_fy_cx_cy': np.round(sdi.ravel()[:4], 2).tolist(),
        'dist': [round(float(v), 6) for v in d.ravel()], 'sd_dist': np.round(sdi.ravel()[4:4+d.size], 5).tolist(),
        'board_m_min_med_max': [round(dist[0], 2), round(dist[len(dist)//2], 2), round(dist[-1], 2)],
        'tilt_deg_min_med_max': [round(tilt[0]), round(tilt[len(tilt)//2]), round(tilt[-1])],
        'coverage_3x4': cover.tolist(), 'subset70_fx': [round(min(fx), 1), round(max(fx), 1)],
        'subset70_k1': [round(min(k1), 4), round(max(k1), 4)], 'K': K.tolist(),
        'dropped': [names[i] for i in range(len(names)) if i not in idx]}
print(json.dumps({k: v for k, v in result.items() if k != 'models'}))
for m, v in result['models'].items():
    print(m, json.dumps({k: v[k] for k in v if k not in ('K', 'dropped')}), 'dropped', len(v['dropped']))
(HERE/'solve-result.json').write_text(json.dumps(result, indent=2)+'\n')
