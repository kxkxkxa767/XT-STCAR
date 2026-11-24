"""Replay labelled historical images through the competition Rust pixel detector."""
import argparse
from collections import Counter
import json
from pathlib import Path
import struct
import subprocess
import tempfile

import cv2

ROOT = Path(__file__).resolve().parents[1]
LABELS = ROOT / 'docs/y114-traffic-light-frames-20261120.jsonl'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
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
    totals = Counter()
    with tempfile.TemporaryDirectory(dir=args.out) as tmp:
        executable = Path(tmp)/'traffic-lamp-frames'
        subprocess.run(['rustc', '--edition', '2024', '-O', str(ROOT/'scripts/traffic-lamp-frames.rs'),
                        '-o', str(executable)], check=True)
        with (args.out/'frames.jsonl').open('w') as output, \
                subprocess.Popen([executable], stdin=subprocess.PIPE, stdout=subprocess.PIPE) as detector:
            for label, path in zip(rows, paths, strict=True):
                bgr = cv2.imread(str(path))
                if bgr is None:
                    raise ValueError(f'could not read {path}')
                height, width = bgr.shape[:2]
                detector.stdin.write(struct.pack('<II', width, height) + bgr.tobytes())
                detector.stdin.flush()
                r, g, elapsed_ns = map(int, detector.stdout.readline().split())
                predicted = 'red' if r else 'green' if g else 'none'
                record = {'source': label['source'], 'observed': label['observed'], 'rust': predicted, 'rust_ms': elapsed_ns/1e6}
                totals['rust', label['observed'], predicted] += 1
                output.write(json.dumps(record) + '\n')
            detector.stdin.close()
            if detector.wait() != 0:
                raise RuntimeError('Rust detector exited during replay')
    for (name, actual, predicted), count in sorted(totals.items()):
        print(f'{name:5}  {actual:9} -> {predicted:5}  {count}')


if __name__ == '__main__':
    main()
