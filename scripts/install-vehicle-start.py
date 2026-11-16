#!/usr/bin/env python3
"""Install/update the SSH-terminal start command after staging a vision helper.

Called by deploy-vision.py after its normal upload completes. Does not run a trial.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
WORK = ROOT / 'work/vehicle-dynamic-deploy-20261008'
HOST = 'bianbu@192.168.0.156'
SOCKET = ROOT / 'work/vehicle-test-ssh.sock'
BASE = '/home/bianbu/xt-stcar-console'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('label', help='deployed vision round, e.g. y90')
    parser.add_argument('--profile', choices=('dark', 'light', 'radar'))
    args = parser.parse_args()
    package = json.loads((WORK / f'claude-{args.label}-package.json').read_text())
    profile = args.profile or package.get('start_profile')
    record = {'label': args.label, 'source_commit': package['source_commit'],
              'helper': f"{package['stage']}/trial{args.label}.py"}
    if profile:
        record.update(profile=profile, source_commit_vision=package.get('source_commit_vision'),
                      model=package.get('vision_model'), field_config=package['field_config'])
    ssh = ['ssh', '-S', str(SOCKET), '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5', HOST]
    subprocess.run(['scp', '-q', '-o', f'ControlPath={SOCKET}', '-o', 'BatchMode=yes',
                    str(ROOT / 'scripts/start-vehicle.py'), f'{HOST}:{BASE}/start-vehicle.py'],
                   check=True, timeout=30)
    # Serialize values as Python literals into stdin; no shell interpolation.
    script = f'''from pathlib import Path
import json,shutil
base=Path({BASE!r})
record={record!r}
profile={profile!r}
assert Path(record['helper']).is_file(), 'vision helper must be staged first'
marker=base/('current-race-'+profile+'.json' if profile else 'current-race.json')
if profile:
 config=Path(record['field_config'])
 if not config.exists():
  config.write_text(json.dumps({{'enabled':True,'crosswalk_mode':'vision','light_mode':'fixed10s'}},indent=2)+'\\n')
tmp=marker.with_suffix('.tmp')
tmp.write_text(json.dumps(record,indent=2)+'\\n')
tmp.replace(marker)
rc=Path.home()/'.bashrc'
text=rc.read_text() if rc.exists() else ''
lines=['start() {{ python3 -B /home/bianbu/xt-stcar-console/start-vehicle.py "$@"; }}']
if profile:
 lines += ['start_'+profile+'() {{ python3 -B /home/bianbu/xt-stcar-console/start-vehicle.py --profile '+profile+' "$@"; }}',
           'stop_'+profile+'() {{ python3 -B /home/bianbu/xt-stcar-console/start-vehicle.py --profile '+profile+' --stop; }}']
missing=[line for line in lines if line not in text]
if missing:
 backup=base/'work/start-command/bashrc-before-start'
 backup.parent.mkdir(parents=True,exist_ok=True)
 if rc.exists() and not backup.exists(): shutil.copy2(rc,backup)
 rc.write_text(text.rstrip()+'\\n\\n# XT-STCAR: one explicit start per trial\\n'+'\\n'.join(missing)+'\\n')
print(json.dumps({{'installed':True,'label':record['label'],'profile':profile,'motion_requested':False}}))
'''
    subprocess.run(ssh + ['python3 -'], input=script, text=True, check=True, timeout=20)


if __name__ == '__main__':
    main()
