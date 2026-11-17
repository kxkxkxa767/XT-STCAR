#!/usr/bin/env python3
"""Upload the unified launcher model set after installing its Rust engine."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
WORK = ROOT/'work/vehicle-dynamic-deploy-20261008'
SOCKET = ROOT/'work/vehicle-test-ssh.sock'
HOST = 'bianbu@192.168.0.156'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('label')
    args = parser.parse_args()
    package = json.loads((WORK/f'claude-{args.label}-package.json').read_text())
    stage = package['stage']
    ssh = ['ssh','-S',str(SOCKET),'-o','BatchMode=yes','-o','ConnectTimeout=5',HOST]
    profiles = list(package['rust_launcher']['profiles'])
    subprocess.run(ssh+['mkdir','-p',stage+'/models',*[stage+'/models/'+p for p in profiles]],check=True)
    for name, entry in package['weather_files'].items():
        assert hashlib.sha256(Path(entry['path']).read_bytes()).hexdigest() == entry['sha256']
        subprocess.run(['scp','-q','-o','ControlPath='+str(SOCKET),'-o','BatchMode=yes',entry['path'],HOST+':'+stage+'/'+name],check=True,timeout=90)
    script = f'''from pathlib import Path
import hashlib,json
stage=Path({stage!r})
files={package['weather_files']!r}
for name, entry in files.items():
 assert hashlib.sha256((stage/name).read_bytes()).hexdigest()==entry['sha256'],name
print(json.dumps({{'verified_files':len(files),'motion_requested':False}}))
'''
    subprocess.run(ssh+['python3 -B -'],input=script,text=True,check=True,timeout=15)
    subprocess.run([sys.executable,'-B',str(ROOT/'scripts/install-vehicle-start.py'),args.label],check=True)


if __name__ == '__main__':
    main()
