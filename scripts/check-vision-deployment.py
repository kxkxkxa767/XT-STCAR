#!/usr/bin/env python3
"""Read-only deployment check. Opens no camera, serial port or chassis bridge."""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys

from validate_yolo26 import load_spec, read_regular_file


def check(config_path, release_dir, static_only=False):
    config = json.loads(read_regular_file(config_path, 65536))
    if config.get('schema_version') != 1 or config.get('enabled') is not True:
        raise ValueError('vision.json requires schema_version=1 and enabled=true for this check')
    fields = ['binary', 'road_config', 'model', 'runtime_lib', 'model_spec']
    for key in fields:
        if not isinstance(config.get(key), str) or not Path(config[key]).is_absolute() or not Path(config[key]).is_file():
            raise ValueError('missing absolute file path: ' + key)
    missing = [name for name in ['app.js', 'index.html', 'server.py', 'style.css', 'vision_shadow.py', 'vehicle-bridge']
               if not (release_dir/name).is_file()]
    if missing:
        raise ValueError('missing console files: ' + ', '.join(missing))
    spec = load_spec(config['model_spec'])
    model = Path(config['model'])
    provenance = json.loads(read_regular_file(model.with_suffix('.provenance.json'), 1024*1024))
    model_hash = hashlib.sha256(read_regular_file(model, 64*1024*1024)).hexdigest()
    if provenance.get('sha256') != model_hash:
        raise ValueError('ONNX SHA256 differs from model provenance')
    labels = [provenance.get('names', {}).get(str(i)) for i in range(spec['class_count'])]
    if (provenance.get('class_count') != spec['class_count'] or any(not n for n in labels)
        or (spec.get('class_names') and spec['class_names'] != labels)):
        raise ValueError('model class names/order differ from model spec')
    if spec.get('class_names') and not provenance.get('training'):
        raise ValueError('custom model requires export-generated training provenance')
    report = {'static_checks_passed': True, 'native_load_checked': False,
              'physical_devices_opened': False, 'motor_commands_sent': False,
              'class_names': labels, 'model_sha256': model_hash, 'host': platform.machine()}
    if static_only:
        report['note'] = 'Static files only; runtime ABI and actual ONNX interface still unchecked'
        return report
    import cv2
    import numpy
    result = subprocess.run([config[k] for k in fields], input=b'', stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, timeout=45)
    if result.returncode != 0:
        raise ValueError('native model startup failed: ' + result.stderr.decode(errors='replace')[-2000:])
    if json.loads(result.stdout) != {'kind': 'ready', 'physical_output_enabled': False}:
        raise ValueError('unexpected vision-shadow startup handshake')
    report.update(native_load_checked=True, opencv=cv2.__version__, numpy=numpy.__version__,
                  note='Persistent session started then closed by EOF; no images, camera or motion')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vision-config', type=Path, default=Path.home()/'xt-stcar-console/vision.json')
    parser.add_argument('--release-dir', type=Path, default=Path.home()/'xt-stcar-console/20260917')
    parser.add_argument('--static-only', action='store_true')
    args = parser.parse_args()
    try:
        print(json.dumps(check(args.vision_config, args.release_dir, args.static_only), ensure_ascii=False, indent=2))
    except Exception as error:
        print('Deployment check failed: ' + str(error), file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
