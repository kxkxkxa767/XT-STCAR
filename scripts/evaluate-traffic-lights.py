"""Compare the deployed Rust pixel detector and a weather model on labelled historical frames."""
import argparse
from collections import Counter
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile

import cv2

ROOT = Path(__file__).resolve().parents[1]
LABELS = ROOT / 'docs/y114-traffic-light-frames-20261120.jsonl'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model-dir', required=True, type=Path)
    parser.add_argument('--evidence-root', type=Path, default=ROOT/'work/vehicle-dynamic-deploy-20261008')
    parser.add_argument('--out', type=Path, required=True, help='new local directory for this comparison')
    args = parser.parse_args()
    rows = [json.loads(line) for line in LABELS.read_text().splitlines()]
    rows = [row for row in rows if row['set'] == 'validation']
    paths = []
    for row in rows:
        run, filename = row['source'].split('/')
        paths.append((args.evidence_root / ('left-cone-trial-' + run) / 'vision/frames' / filename).resolve())
    args.out.mkdir(parents=True, exist_ok=False)
    yolo = args.out / 'yolo'
    subprocess.run([sys.executable, '-B', str(ROOT/'scripts/vision-log.py'), '--model-dir', str(args.model_dir),
                    '--out', str(yolo), '--threads', '2', '--max-s', '1800', '--max-saved', '0', '--images',
                    *map(str, paths)], check=True)
    totals = Counter()
    with tempfile.TemporaryDirectory(dir=args.out) as tmp:
        executable = Path(tmp)/'traffic-lamp-frames'
        subprocess.run(['rustc', '--edition', '2024', '-O', str(ROOT/'scripts/traffic-lamp-frames.rs'),
                        '-o', str(executable)], check=True)
        with (yolo/'detections.jsonl').open() as results, (args.out/'frames.jsonl').open('w') as output, \
                subprocess.Popen([executable], stdin=subprocess.PIPE, stdout=subprocess.PIPE) as detector:
            for label, path, line in zip(rows, paths, results, strict=True):
                bgr = cv2.imread(str(path))
                if bgr is None:
                    raise ValueError(f'could not read {path}')
                height, width = bgr.shape[:2]
                detector.stdin.write(struct.pack('<II', width, height) + bgr.tobytes())
                detector.stdin.flush()
                r, g, elapsed_ns = map(int, detector.stdout.readline().split())
                observed = json.loads(line)
                if observed['source'] != str(path):
                    raise ValueError(f'frame order changed at {path}')
                signals = {box['cls'] for box in observed['detections']}
                votes = {'rust': (bool(r), bool(g)), 'yolo': ('red_light' in signals, 'green_light' in signals)}
                votes['fused'] = (votes['rust'][0] or votes['yolo'][0], votes['rust'][1] or votes['yolo'][1])
                record = {'source': label['source'], 'observed': label['observed'],
                          'rust_ms': elapsed_ns/1e6, 'yolo_ms': observed['infer_ms']}
                for name, (red, green) in votes.items():
                    predicted = 'red' if red else 'green' if green else 'none'
                    record[name] = predicted
                    totals[name, label['observed'], predicted] += 1
                output.write(json.dumps(record) + '\n')
            detector.stdin.close()
            if detector.wait() != 0:
                raise RuntimeError('Rust detector exited during replay')
    for (name, actual, predicted), count in sorted(totals.items()):
        print(f'{name:5}  {actual:9} -> {predicted:5}  {count}')


if __name__ == '__main__':
    main()
