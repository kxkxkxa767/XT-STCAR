#!/usr/bin/env python3
"""Read-only camera perception log for the race4 YOLO26n model (round y1).

  python3 vision-log.py --model-dir DIR --out NEW_DIR [--camera /dev/video20] [--threads 2]
      [--max-s 150] [--conf .25] [--save-every 10] [--max-saved 400]

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
import os
from pathlib import Path
import signal
import sys
import time

CLASSES = ['crosswalk', 'blue_cone', 'red_light', 'green_light']
SIZE = 320
FILL = 114
LOGGED = ('crosswalk', 'red_light', 'green_light')


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
    parser.add_argument('--max-s', type=float, default=150.)
    parser.add_argument('--conf', type=float, default=.25)
    parser.add_argument('--save-every', type=int, default=10)
    parser.add_argument('--max-saved', type=int, default=400)
    parser.add_argument('--images', nargs='*', help='bench: these image files instead of the camera')
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
    options = ort.SessionOptions()
    options.intra_op_num_threads = args.threads
    options.inter_op_num_threads = 1
    session = ort.InferenceSession(str(onnx), options, providers=['CPUExecutionProvider'])
    inputs, outputs = session.get_inputs(), session.get_outputs()
    if ([i.name for i in inputs], list(inputs[0].shape), [o.name for o in outputs], list(outputs[0].shape)) != (
            ['images'], [1, 3, SIZE, SIZE], ['output0'], [1, 300, 6]):
        raise SystemExit('ONNX interface differs from the race4 contract')
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
                      'threads': args.threads, 'source': args.camera if cap is not None else 'images',
                      'started_monotonic_s': started}), flush=True)
    stats = {'frames': 0, 'saved': 0, 'infer_ms': [], 'counts': {c: 0 for c in CLASSES}, 'first_s': {}, 'last_s': {},
             'best_score': {}}
    images = list(args.images or [])
    with (out/'detections.jsonl').open('w') as log:
        while not stop and time.monotonic()-started < args.max_s:
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
            detections = decode(result, args.conf, scale, left, top, w, h)
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
            log.write(json.dumps({'frame': stats['frames'], 'captured_monotonic_s': round(captured, 4),
                                  'infer_ms': round(infer_ms, 1), 'detections': detections, 'saved': saved,
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
    (out/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps({'kind': 'summary', **{k: summary[k] for k in ('frames', 'fps', 'infer_ms_median', 'counts', 'ended')}}),
          flush=True)
    return 0 if not stop or stop[0] in ('sigterm', 'sigint') else 1


if __name__ == '__main__':
    sys.exit(main())
