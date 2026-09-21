"""Repeat recorded JPEG inference in a persistent process; never opens devices."""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'web/vehicle-console'))
from vision_shadow import VisionShadow


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--image', required=True, type=Path)
    p.add_argument('--binary', type=Path, default=ROOT/'target/release/vision-shadow')
    p.add_argument('--model', type=Path, default=ROOT/'models/yolo26n.onnx')
    p.add_argument('--spec', type=Path, default=ROOT/'config/yolo26n.json')
    p.add_argument('--road-config', type=Path, default=ROOT/'config/road-perception-sim.json')
    p.add_argument('--runtime-lib', type=Path, default=ROOT/'toolchains/onnxruntime-macos-arm64-1.29.0/lib/libonnxruntime.1.29.0.dylib')
    p.add_argument('--frames', type=int, default=15)
    p.add_argument('--output', required=True, type=Path)
    args = p.parse_args()
    if not 6 <= args.frames <= 1000 or not args.image.is_file() or args.image.stat().st_size > 2*1024*1024:
        p.error('requires 6..1000 frames and an existing JPEG <=2 MiB')
    jpeg = args.image.read_bytes()
    worker = VisionShadow([str(x.resolve()) for x in [args.binary, args.road_config, args.model, args.runtime_lib, args.spec]])
    worker.start()
    try:
        for sequence in range(1, args.frames+1):
            if not worker.submit(jpeg, time.monotonic(), sequence):
                raise RuntimeError('worker rejected source frame')
            deadline = time.monotonic() + (45 if sequence == 1 else 8)
            while time.monotonic() < deadline:
                state = worker.snapshot()
                if state['error']: raise RuntimeError(state['error'])
                if state['result'] and state['result']['sequence'] == sequence: break
                time.sleep(.01)
            else: raise RuntimeError('no result before deadline')
        report = state['result']
        report.pop('jpeg_base64')
        report.update(scope='Recorded JPEG, sequential persistent native ORT, no camera or vehicle access',
            host=platform.system()+'/'+platform.machine(), frames=args.frames, model_sha256=hashlib.sha256(args.model.read_bytes()).hexdigest(),
            image_sha256=hashlib.sha256(jpeg).hexdigest(), native_intra_threads=2)
        args.output.write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps({'frames':args.frames, 'detections':len(report['diagnostics']['detections']), 'performance':report['performance']}))
    finally:
        worker.close()


if __name__ == '__main__':
    main()
