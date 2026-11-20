#!/usr/bin/env python3
"""Stage dark/light/mix models for one Rust race launcher; no vehicle operation."""
import argparse
import datetime
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import zipfile
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parents[1]
WORK = ROOT/'work/vehicle-dynamic-deploy-20261008'
BASE = '/home/bianbu/xt-stcar-console'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('engine', help='already built engine package, e.g. claude-turn209')
    parser.add_argument('label')
    parser.add_argument('weather_zip', type=Path)
    args = parser.parse_args()
    source = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    package = json.loads((WORK/(args.engine+'-package.json')).read_text())
    if package['source_commit'] != source:
        raise SystemExit('Build the engine from the current source first')
    now = datetime.datetime.now(ZoneInfo('Asia/Taipei'))
    directory = WORK/f'weather-{args.label}-{source[:7]}'
    directory.mkdir()
    stage = BASE+f'/stage-weather-{args.label}-'+now.strftime('%Y%m%d-%H%M%S')
    shutil.copy2(ROOT/'scripts/vision-log.py', directory/'vision-log.py')
    configs = {}
    for profile, prior in [('dark', 'y101')]:
        old = json.loads((WORK/f'claude-{prior}-package.json').read_text())
        dest = directory/'models'/profile
        dest.mkdir(parents=True)
        for name in ['race4.onnx', 'race4.json', 'race4.provenance.json']:
            shutil.copy2(old['vision_files']['race4/'+name]['path'], dest/name)
    with zipfile.ZipFile(args.weather_zip) as archive:
        for profile, member in [('light', 'sunny_three_class'), ('mix', 'five_class_all_weather')]:
            models = [n for n in archive.namelist() if '/'+member+'/' in n and n.endswith('.onnx')]
            if len(models) != 1:
                raise SystemExit('Expected exactly one model in '+member)
            stem = models[0][:-5]
            dest = directory/'models'/profile
            dest.mkdir(parents=True)
            for suffix in ['.onnx', '.json', '.provenance.json']:
                (dest/('race4'+suffix)).write_bytes(archive.read(stem+suffix))
    spec = importlib.util.spec_from_file_location('vision_log', ROOT/'scripts/vision-log.py')
    vision = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(vision)
    for profile in ['dark', 'light', 'mix']:
        _, model_spec, digest = vision.check_model(directory/'models'/profile)
        configs[profile] = {'model_dir':stage+'/models/'+profile, 'model_sha256':digest,
            'classes':model_spec['class_names'], 'camera':'/dev/video20'}
    launch = {'label':args.label, 'base':BASE, 'source_commit':source,
        'evidence_prefix':f'left-cone-trial-{now:%Y%m%d}-{args.label}',
        'bridge':BASE+'/20260917/vehicle-bridge', 'vision_script':stage+'/vision-log.py', 'profiles':configs}
    (directory/'race-launch.json').write_text(json.dumps(launch,indent=2)+'\n')
    files = {str(p.relative_to(directory)):{'path':str(p),'sha256':sha(p)} for p in directory.rglob('*') if p.is_file()}
    record = {'label':args.label,'source_commit':source,'engine_package':args.engine,
        'route_engine_sha256':package['route_engine_sha256'], 'stage':stage,'weather_files':files,
        'rust_launcher':{'program':BASE+'/20260917/route-engine','config':BASE+'/race-launch.json','profiles':configs},
        'model_handoff':str(args.weather_zip.resolve()), 'motion_requested':False}
    (WORK/f'claude-{args.label}-package.json').write_text(json.dumps(record,indent=2)+'\n')
    print(json.dumps({'label':args.label,'stage':stage,'profiles':configs,'files':len(files)},indent=2))


if __name__ == '__main__':
    main()
