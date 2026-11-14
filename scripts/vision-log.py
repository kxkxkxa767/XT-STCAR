#!/usr/bin/env python3
"""Read-only camera perception log for the race4 YOLO26n model (round y1).

  python3 vision-log.py --model-dir DIR --out NEW_DIR [--camera /dev/video20] [--threads 2]
      [--max-s 150] [--conf .25] [--save-every 10] [--max-saved 400] [--light-state PATH]

Runs on the car beside `route-engine race` (scripts/race-trial.py --vision-*), or
alone for a bench check. It never opens the car's control or lidar devices and
sends nothing anywhere: it reads the camera (V4L2 MJPG 640x480, as the console
does), runs DIR/race4.onnx with ONNX Runtime on the CPU and writes, per frame,
one JSON line to NEW_DIR/detections.jsonl: the camera sequence, the capture time
on CLOCK_MONOTONIC (the clock race-trial.py records the program start on), the
inference time and every detection at or above --conf (class name, score, box
in camera pixels). JPEGs go to NEW_DIR/frames/: every frame with a crosswalk or
a light and every --save-every-th frame, at most --max-saved. NEW_DIR/summary.json
is written at the end (SIGTERM, SIGINT or --max-s). The first stdout line is
{"kind": "ready", ...} once the model and the camera work.

Round y2: with --light-state PATH, after every frame PATH is replaced
atomically (a temporary file renamed over it) by one JSON object: the frame
number, its capture time and the write time on this process's CLOCK_MONOTONIC
("captured_s", "written_s") and the highest red_light / green_light score of
the frame ("red", "green"; 0 without one). `route-engine race --light-state
PATH` reads it as the traffic light's state; nothing else is sent anywhere.
Round y5: with a crosswalk in the frame also its best score ("crosswalk")
and that box's bottom row in image pixels ("crosswalk_row"), for the start
lane's crosswalk stop.
Round y11: and the distance from the car's front to the stripes' near edge
("crosswalk_m", metres along the ground) by the calibrated camera model
(crosswalk_front_m), when the box's bottom stands clear of the frame's.
Round y35: and, when both are found, the undistorted image heights of the
stripes' near edge and of the board straight ahead's base ("crosswalk_edge_y",
"board_base_y"); the race program ranges the stripes against that board
(round y36: measured from the best crosswalk box scoring at least .15).

Before anything else the model must match its handoff files: sha256 of
race4.onnx equal to race4.provenance.json's, the class order of race4.json
(crosswalk, blue_cone, red_light, green_light), input images [1,3,320,320]
float32, output output0 [1,300,6] (x1,y1,x2,y2,score,class; one-to-one, no
NMS). Preprocessing follows crates/vision (letterbox): bilinear resize with
the aspect kept, centred integer padding filled with 114, RGB, 0-1.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import sys
import time

CLASSES = ['crosswalk', 'blue_cone', 'red_light', 'green_light']
SIZE = 320
FILL = 114
LOGGED = ('crosswalk', 'red_light', 'green_light')
# Round y11: the car camera's model (2026-10-31, config/camera-race4-20261031.json;
# the camera fixed with hot glue at the angle chosen at the light). Lens:
# OpenCV's pinhole k1 k2 p1 p2 k3 from 138 views of a 9x6 board (.42 px rms;
# 70 % subsets fx 424-434). Mount: the lens centre .195 m above the floor (tape;
# a board on a wall gave .1924) and .041 m behind the front (the wall pose and
# the tape at the glass, 45.4 vs 41.5 cm). The ranging pitch fits race4's box
# bottoms on the track (front 1.00/2.31-2.34 m from the near edge by tape and
# the board behind: 8.94-9.21 deg); the stripes' edge and the far boards'
# bases (lidar 3.92/7.70 m) put the camera's own pitch at 8.5-8.8 deg, the box
# bottoms sitting 2-3 rows below the edge.
CAMERA_FX_FY_CX_CY = (426.55, 424.47, 349.59, 227.85)
CAMERA_DISTORTION = (-.371764, .172259, -.000092, -.000226, -.042575)
CAMERA_HEIGHT_M = .195
CAMERA_BEHIND_FRONT_M = .041
CAMERA_RANGE_PITCH_DEG = 9.1
# A crosswalk box this low touches the frame's bottom (480): race4 ended the
# boxes of stripes running out of the frame at rows 476-477 (front .21 m from
# the edge, 10-31); the edge itself stays in view to ~.38 m (row ~474).
CROSSWALK_CLIPPED_ROW = 472.


def undistort(u, v):
    """Normalized undistorted image point of pixel (u, v), or None where the
    fixed-point iteration does not reproduce the pixel (the bottom corners)."""
    fx, fy, cx, cy = CAMERA_FX_FY_CX_CY
    k1, k2, p1, p2, k3 = CAMERA_DISTORTION
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


def crosswalk_front_m(box):
    """Round y11: metres from the car's front to the crosswalk's near edge on
    the floor at box's bottom row (in the column nearest the image centre:
    the edge's lowest point there), or None (bottom at the frame's, or no
    floor point)."""
    x1, _, x2, y2 = box
    if y2 >= CROSSWALK_CLIPPED_ROW:
        return None
    point = undistort(min(max(CAMERA_FX_FY_CX_CY[2], x1), x2), y2)
    if point is None:
        return None
    pitch = math.radians(CAMERA_RANGE_PITCH_DEG)
    down = point[1]*math.cos(pitch)-math.sin(pitch)
    if down <= 1e-6:
        return None
    return CAMERA_HEIGHT_M*(math.cos(pitch)+point[1]*math.sin(pitch))/down-CAMERA_BEHIND_FRONT_M


# Round y35 (user, 2026-11-03): the crosswalk stop is planned at rest against
# the board straight ahead (the left-turn exit board). Both heights below are
# undistorted image heights (tan of the angle below the optical axis); route-engine
# race takes the board's lidar range for its base, which fixes the pitch of this
# frame, so the mount's pitch (9.1 above; 8.18 measured at the start on 11-03)
# drops out of the stripes' distance.
BOARD_BASE_COLUMNS = range(330, 391, 10)  # scripts/camera-pitch-check.py's
BOARD_BLUE_MIN = 45  # blue minus red: the boards
EDGE_CONTRAST_MIN = 25  # grey levels between the paper and the floor just below it
# Round y36: y35's start frames scored the crosswalk about .25 (y34 .35, y32 .29), at the
# --conf edge; the edge is measured from the best box down to this, the logs keep --conf.
CROSSWALK_PLAN_MIN_SCORE = .15


def board_base_y(frame):
    """Undistorted height of the board ahead's base (camera-pitch-check.py: in
    the centre columns the lowest blue run longer than five rows ending above
    row 340, its last row and a half), or None."""
    rows = []
    for u in BOARD_BASE_COLUMNS:
        strip = frame[200:400, u-1:u+2].astype(int)
        blue = (strip[:, :, 0]-strip[:, :, 2]).mean(axis=1) > BOARD_BLUE_MIN
        best, start = None, None
        for i, b in enumerate(list(blue)+[False]):
            if b and start is None:
                start = i
            elif not b and start is not None:
                if i-start > 5 and 200+i-1 < 340:
                    best = 200+i-1
                start = None
        if best is not None:
            rows.append(best+.5)
    if len(rows) < 4:
        return None
    rows.sort()
    row = rows[len(rows)//2] if len(rows) % 2 else (rows[len(rows)//2-1]+rows[len(rows)//2])/2
    point = undistort(CAMERA_FX_FY_CX_CY[2], row)
    return None if point is None else point[1]


def crosswalk_edge_y(gray, box):
    """Undistorted height of the stripes' near edge: in every second column of
    the box, down from the brightest row near the box's bottom (the paper) to
    where the brightness falls half way to the floor just below it; the median
    over the columns with that contrast (gaps between papers have none), or None."""
    x1, _, x2, y2 = box
    top, bottom = max(0, int(y2)-24), min(gray.shape[0]-1, int(y2)+12)
    heights = []
    for u in range(max(1, int(x1)+2), min(gray.shape[1]-1, int(x2)-1), 2):
        col = gray[top:bottom+1, u-1:u+2].mean(axis=1)
        peak = int(col.argmax())
        floor = col[peak:].min()
        if col[peak]-floor < EDGE_CONTRAST_MIN:
            continue
        half = (col[peak]+floor)/2
        i = peak
        while i+1 < len(col) and col[i+1] >= half:
            i += 1
        if i+1 >= len(col):
            continue
        row = top+i+(col[i]-half)/(col[i]-col[i+1])
        point = undistort(u, row) if row < CROSSWALK_CLIPPED_ROW else None
        if point is not None:
            heights.append(point[1])
    if len(heights) < 5:
        return None
    heights.sort()
    return heights[len(heights)//2]


def check_model(model_dir):
    """The handoff's own checks; returns (onnx path, spec)."""
    spec = json.loads((model_dir/'race4.json').read_text())
    provenance = json.loads((model_dir/'race4.provenance.json').read_text())
    onnx = model_dir/'race4.onnx'
    digest = hashlib.sha256(onnx.read_bytes()).hexdigest()
    problems = []
    if digest != provenance.get('sha256'):
        problems.append('race4.onnx sha256 differs from race4.provenance.json')
    if spec.get('class_names') != CLASSES or [provenance.get('names', {}).get(str(i)) for i in range(4)] != CLASSES:
        problems.append('class order is not crosswalk, blue_cone, red_light, green_light')
    if (spec.get('input_width'), spec.get('input_height'), spec.get('output_layout')) != (SIZE, SIZE, 'B_N_XYXY_SCORE_CLASS'):
        problems.append('input size or output layout differs from the race4 contract')
    if problems:
        raise SystemExit('model check failed: ' + '; '.join(problems))
    return onnx, spec, digest


def letterbox(frame_bgr, cv2, np):
    """crates/vision preprocess: (tensor, scale, pad_left, pad_top)."""
    h, w = frame_bgr.shape[:2]
    scale = min(SIZE/w, SIZE/h)
    rw, rh = round(w*scale), round(h*scale)
    left, top = (SIZE-rw)//2, (SIZE-rh)//2
    canvas = np.full((SIZE, SIZE, 3), FILL, dtype=np.uint8)
    canvas[top:top+rh, left:left+rw] = cv2.resize(frame_bgr, (rw, rh), interpolation=cv2.INTER_LINEAR)
    rgb = canvas[:, :, ::-1].astype(np.float32)/255.
    return np.ascontiguousarray(rgb.transpose(2, 0, 1)[None]), scale, left, top, w, h


def decode(output, conf, scale, left, top, w, h):
    detections = []
    for x1, y1, x2, y2, score, cls in output[0]:
        if not score >= conf or not 0 <= int(cls) < len(CLASSES):
            continue
        box = [min(max((x1-left)/scale, 0.), w), min(max((y1-top)/scale, 0.), h),
               min(max((x2-left)/scale, 0.), w), min(max((y2-top)/scale, 0.), h)]
        if box[2]-box[0] < 1 or box[3]-box[1] < 1:
            continue  # padding only
        detections.append({'cls': CLASSES[int(cls)], 'score': round(float(score), 4),
                           'box': [round(float(v), 1) for v in box]})
    return detections


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--model-dir', required=True)
    parser.add_argument('--out', required=True)
    parser.add_argument('--camera', default='/dev/video20')
    parser.add_argument('--threads', type=int, default=2)
    parser.add_argument('--phase-state', help='Rust race resource hint: drive or light_wait')
    parser.add_argument('--wait-threads', type=int, default=4, help='inference threads only while Rust reports light_wait')
    parser.add_argument('--max-s', type=float, default=150.)
    parser.add_argument('--conf', type=float, default=.25)
    parser.add_argument('--save-every', type=int, default=10)
    parser.add_argument('--max-saved', type=int, default=400)
    parser.add_argument('--images', nargs='*', help='bench: these image files instead of the camera')
    parser.add_argument('--light-state', help='round y2: the light state file for route-engine race')
    args = parser.parse_args()
    onnx, spec, digest = check_model(Path(args.model_dir))
    out = Path(args.out)
    out.mkdir(mode=0o700)  # a new directory, never reused
    (out/'frames').mkdir()
    stop = []
    signal.signal(signal.SIGTERM, lambda *_: stop.append('sigterm'))
    signal.signal(signal.SIGINT, lambda *_: stop.append('sigint'))
    try:
        os.nice(10)
    except OSError:
        pass
    import cv2
    import numpy as np
    import onnxruntime as ort
    def make_session(threads):
        options = ort.SessionOptions()
        options.intra_op_num_threads = threads
        options.inter_op_num_threads = 1
        return ort.InferenceSession(str(onnx), options, providers=['CPUExecutionProvider'])
    session = make_session(args.threads)
    inputs, outputs = session.get_inputs(), session.get_outputs()
    if ([i.name for i in inputs], list(inputs[0].shape), [o.name for o in outputs], list(outputs[0].shape)) != (
            ['images'], [1, 3, SIZE, SIZE], ['output0'], [1, 300, 6]):
        raise SystemExit('ONNX interface differs from the race4 contract')
    sessions = {args.threads: session}
    wait_profile_error = None
    if args.phase_state and args.wait_threads != args.threads:
        try:
            sessions[args.wait_threads] = make_session(args.wait_threads)
            warm = np.zeros((1, 3, SIZE, SIZE), dtype=np.float32)
            for profile in sessions.values():
                profile.run(['output0'], {'images': warm})
        except Exception as error:
            # An optional throughput profile must not disable the working detector.
            sessions = {args.threads: session}
            wait_profile_error = str(error)
    phase_path = Path(args.phase_state) if args.phase_state else None
    phase = 'drive'
    profile_times = {threads: [] for threads in sessions}
    cap = None
    if args.images is None:
        cap = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise SystemExit('camera open failed: ' + args.camera)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    started = time.monotonic()
    print(json.dumps({'kind': 'ready', 'model_sha256': digest, 'onnxruntime': ort.__version__,
                      'threads': args.threads, 'waiting_threads': args.wait_threads if args.wait_threads in sessions else args.threads,
                      'source': args.camera if cap is not None else 'images',
                      'started_monotonic_s': started}), flush=True)
    stats = {'frames': 0, 'saved': 0, 'infer_ms': [], 'counts': {c: 0 for c in CLASSES}, 'first_s': {}, 'last_s': {},
             'best_score': {}}
    images = list(args.images or [])
    with (out/'detections.jsonl').open('w') as log:
        while not stop and time.monotonic()-started < args.max_s:
            if phase_path is not None:
                try:
                    hint = phase_path.read_text().strip()
                except OSError:
                    hint = None
                if hint in ('drive', 'light_wait'):
                    phase = hint
            threads = args.wait_threads if phase == 'light_wait' and args.wait_threads in sessions else args.threads
            session = sessions[threads]
            if cap is not None:
                ok, frame = cap.read()
                captured = time.monotonic()
                if not ok:
                    stop.append('camera_read_failed')
                    break
            else:
                if not images:
                    break
                path = images.pop(0)
                frame = cv2.imread(path)
                captured = time.monotonic()
                if frame is None:
                    continue
            tensor, scale, left, top, w, h = letterbox(frame, cv2, np)
            t0 = time.monotonic()
            result = session.run(['output0'], {'images': tensor})[0]
            infer_ms = (time.monotonic()-t0)*1000
            profile_times[threads].append(infer_ms)
            candidates = decode(result, min(args.conf, CROSSWALK_PLAN_MIN_SCORE), scale, left, top, w, h)
            detections = [d for d in candidates if d['score'] >= args.conf]
            stats['frames'] += 1
            stats['infer_ms'].append(infer_ms)
            seen = {d['cls'] for d in detections}
            for d in detections:
                c = d['cls']
                stats['counts'][c] += 1
                stats['first_s'].setdefault(c, round(captured-started, 3))
                stats['last_s'][c] = round(captured-started, 3)
                stats['best_score'][c] = max(stats['best_score'].get(c, 0.), d['score'])
            saved = None
            if stats['saved'] < args.max_saved and (seen & set(LOGGED) or stats['frames'] % args.save_every == 1):
                saved = f'frame-{stats["frames"]:06d}.jpg'
                cv2.imwrite(str(out/'frames'/saved), frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                stats['saved'] += 1
            if args.light_state:
                def best(cls):
                    return max((d['score'] for d in detections if d['cls'] == cls), default=0.)
                state = {'frame': stats['frames'], 'captured_s': round(captured, 4),
                         'written_s': round(time.monotonic(), 4), 'red': best('red_light'),
                         'green': best('green_light'), 'inference_threads': threads, 'vision_phase': phase}
                crosswalks = [d for d in detections if d['cls'] == 'crosswalk']
                if crosswalks:
                    # round y5: the best crosswalk and its box's bottom row (image pixels)
                    top = max(crosswalks, key=lambda d: d['score'])
                    state.update(crosswalk=top['score'], crosswalk_row=round(top['box'][3], 1))
                    metres = crosswalk_front_m(top['box'])
                    if metres is not None:
                        state['crosswalk_m'] = round(metres, 3)  # round y11
                # round y35: the stripes' edge and the board ahead's base in this frame
                # (y36: from the best crosswalk box down to CROSSWALK_PLAN_MIN_SCORE)
                planned = [d for d in candidates if d['cls'] == 'crosswalk']
                if planned:
                    best_box = max(planned, key=lambda d: d['score'])['box']
                    edge = crosswalk_edge_y(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), best_box)
                    base = board_base_y(frame)
                    if edge is not None and base is not None:
                        state.update(crosswalk_edge_y=round(edge, 6), board_base_y=round(base, 6))
                light_state = Path(args.light_state)
                temporary = light_state.with_name(light_state.name+'.tmp')
                temporary.write_text(json.dumps(state))
                os.replace(temporary, light_state)
            log.write(json.dumps({'frame': stats['frames'], 'captured_monotonic_s': round(captured, 4),
                                  'infer_ms': round(infer_ms, 1), 'detections': detections, 'saved': saved,
                                  'inference_threads': threads, 'vision_phase': phase,
                                  'source': None if cap is not None else path}) + '\n')
            log.flush()
    if cap is not None:
        cap.release()
    ms = sorted(stats.pop('infer_ms'))
    elapsed = time.monotonic()-started
    summary = {**stats, 'ended': stop[0] if stop else ('max_s' if cap is not None else 'images_done'),
               'elapsed_s': round(elapsed, 2), 'fps': round(stats['frames']/elapsed, 2) if elapsed > 0 else None,
               'infer_ms_median': round(ms[len(ms)//2], 1) if ms else None,
               'infer_ms_max': round(ms[-1], 1) if ms else None, 'model_sha256': digest,
               'conf': args.conf, 'started_monotonic_s': started, 'physical_output': False}
    summary['inference_profiles'] = {str(n): {'frames': len(values),
        'infer_ms_median': round(sorted(values)[len(values)//2], 1) if values else None}
        for n, values in profile_times.items()}
    summary['wait_profile_error'] = wait_profile_error
    (out/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps({'kind': 'summary', **{k: summary[k] for k in ('frames', 'fps', 'infer_ms_median', 'counts', 'ended')}}),
          flush=True)
    return 0 if not stop or stop[0] in ('sigterm', 'sigint') else 1


if __name__ == '__main__':
    sys.exit(main())
