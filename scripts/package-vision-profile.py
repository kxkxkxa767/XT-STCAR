#!/usr/bin/env python3
"""Build a named visual profile on an already-built Rust engine package.

Only normalizes deployment filenames; ONNX data and declared class IDs stay
unchanged. The profile uses camera crosswalk positioning and its own field file.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[1]
WORK = ROOT/'work/vehicle-dynamic-deploy-20261008'
BASE = '/home/bianbu/xt-stcar-console'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('baseline')
    parser.add_argument('label')
    parser.add_argument('profile', choices=('dark', 'light'))
    parser.add_argument('handoff', type=Path)
    args = parser.parse_args()
    with zipfile.ZipFile(args.handoff) as archive:
        models = [n for n in archive.namelist() if n.endswith('.onnx')]
        if len(models) != 1:
            raise SystemExit('The profile handoff must contain one ONNX model')
        stem = models[0][:-5]
        files = {'race4'+suffix: archive.read(stem+suffix) for suffix in ('.onnx', '.json', '.provenance.json')}
    digest = hashlib.sha256(files['race4.onnx']).hexdigest()
    provenance = json.loads(files['race4.provenance.json'])
    if provenance.get('sha256') != digest:
        raise SystemExit('Handoff model and provenance do not match')
    files['README.md'] = (f'{args.profile} visual profile from {args.handoff.name}.\n'
                          f'Original ONNX SHA256: {digest}. Deployment filenames normalized; class names unchanged.\n').encode()
    files['SHA256SUMS'] = ''.join(hashlib.sha256(data).hexdigest()+'  '+name+'\n'
                                 for name, data in files.items()).encode()
    normalized = WORK/f'{args.profile}-handoff-{digest[:8]}.zip'
    with zipfile.ZipFile(normalized, 'w', compression=zipfile.ZIP_DEFLATED) as archive:
        for name, data in files.items():
            archive.writestr(name, data)
    subprocess.run([sys.executable, '-B', str(WORK/'build-vision.py'), args.baseline, args.label,
                    str(normalized), '--light-stop'], cwd=ROOT, check=True)
    path = WORK/f'claude-{args.label}-package.json'
    package = json.loads(path.read_text())
    config = BASE+'/field-test-'+args.profile+'.json'
    fixed = package['helper_fixed_args'] + ['--field-config', config]
    helper = Path(package['helper'])
    text, count = re.subn(r'^FIXED_ARGS = .*$', lambda _: 'FIXED_ARGS = '+repr(fixed)+'  # '+args.profile+' profile',
                         helper.read_text(), flags=re.M)
    if count != 1:
        raise RuntimeError('Profile helper has no unique fixed-argument declaration')
    compile(text, str(helper), 'exec')
    helper.write_text(text)
    helper_sha = hashlib.sha256(helper.read_bytes()).hexdigest()
    package.update(start_profile=args.profile, field_config=config,
                   helper_fixed_args=fixed, helper_sha256=helper_sha,
                   original_model_handoff=str(args.handoff.resolve()))
    package['vision_files'][f'trial{args.label}.py']['sha256'] = helper_sha
    path.write_text(json.dumps(package, indent=2)+'\n')
    print(json.dumps({'profile':args.profile,'package':str(path),'label':args.label,
                      'model_sha256':digest,'field_config':config,'motion_requested':False}))


if __name__ == '__main__':
    main()
