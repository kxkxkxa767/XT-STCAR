#!/usr/bin/env python3
"""Start one built and deployed round on the car from this Mac (not on the car).

  python3 scripts/start-round.py N

Typing it is the operator's go: the car must already stand at the start. Each
round runs once: it refuses when work/.../trialN-stdout.json or the local copy
of the round's evidence directory already exists. Steps:
1. the car's fresh state (work/.../read-fresh-engine.py) must be six fresh
   locked-neutral ready samples whose 17 installed hashes match
   claude-turnN-package.json (validate-fresh-engine.py); otherwise nothing starts;
2. the round's packaged helper STAGE/trialN.py runs once on the car (it checks
   the hashes, the console's fresh locked neutral and its start check again,
   runs `route-engine race` and always restores the console);
3. the evidence directory is copied back and checked file by file (sha256), the
   console's state after the run is read, and a short summary is printed.
SSH uses work/vehicle-test-ssh.sock while its master runs, else plain ssh (which
may ask for the password in this terminal). Nothing secret is printed.
"""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
WORK = ROOT/'work'/'vehicle-dynamic-deploy-20261008'
HOST = 'bianbu@192.168.0.156'
SOCK = ROOT/'work'/'vehicle-test-ssh.sock'


def say(text):
    print(time.strftime('%H:%M:%S'), text, flush=True)


def fail(text):
    say('停止：' + text)
    raise SystemExit(1)


def ssh_base():
    if SOCK.exists() and subprocess.run(['ssh', '-S', str(SOCK), '-O', 'check', HOST],
                                        capture_output=True).returncode == 0:
        return (['ssh', '-S', str(SOCK), '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5', HOST],
                ['scp', '-o', f'ControlPath={SOCK}'])
    return ['ssh', '-o', 'ConnectTimeout=10', HOST], ['scp']


def read_fresh(ssh, out):
    with open(WORK/'read-fresh-engine.py', 'rb') as script, open(out, 'wb') as dump:
        if subprocess.run(ssh+['python3 -'], stdin=script, stdout=dump, timeout=60).returncode != 0:
            fail(f'读不到车上状态（{out.name}）')
    return json.loads(Path(out).read_text())


def main():
    if len(sys.argv) != 2 or not sys.argv[1].isdigit():
        raise SystemExit(__doc__)
    n = int(sys.argv[1])
    package = WORK/f'claude-turn{n}-package.json'
    if not package.is_file():
        fail(f'没有第{n}轮的部署包记录 {package.name}')
    pkg = json.loads(package.read_text())
    args = pkg['helper_fixed_args']
    remote_evidence = args[args.index('--evidence')+1]
    evidence = WORK/Path(remote_evidence).name
    stdout_path = WORK/f'trial{n}-stdout.json'
    if stdout_path.exists() or evidence.exists():
        fail(f'第{n}轮已经跑过（{stdout_path.name} 或 {evidence.name} 已存在），每轮只跑一次')
    ssh, scp = ssh_base()

    say(f'第{n}轮：读车上状态并核对部署（车须锁定、中位、在起点）')
    pre = WORK/f'pre{n}-run-fresh.json'
    if pre.exists():
        pre = WORK/f'pre{n}-run-fresh-{time.strftime("%H%M%S")}.json'
    fresh = read_fresh(ssh, pre)
    check = subprocess.run([sys.executable, str(WORK/'validate-fresh-engine.py'), str(pre), str(package)],
                           capture_output=True, text=True)
    if check.returncode != 0:
        fail('车上状态或安装文件与第%d轮部署不符，没有启动：%s' % (n, (check.stderr or check.stdout).strip()[-300:]))
    ranges = fresh['samples'][-1]['lidar']['scan']['ranges']
    side = lambda i: '-' if ranges[i] is None else f'{ranges[i]:.2f}m'
    say(f'核对通过：{check.stdout.strip()}')
    say(f'起点读数：左 {side(270)}，右 {side(90)}，前方 {side(0)}，后方 {side(180)}')

    say('开跑（约30秒，跑完车自动锁定）')
    with open(stdout_path, 'wb') as out, open(WORK/f'trial{n}-stderr.txt', 'wb') as err:
        code = subprocess.run(ssh+[f"python3 {pkg['stage']}/trial{n}.py"], stdout=out, stderr=err,
                              timeout=300).returncode
    try:
        helper = json.loads(stdout_path.read_text().strip().splitlines()[-1])
    except (ValueError, IndexError):
        helper = {}
    say(f"车上助手退出码 {code}，结束原因 {helper.get('reason')}")

    say('取回证据')
    listing = subprocess.run(ssh+[f'cd {remote_evidence} && find . -type f | sort | xargs sha256sum'],
                             capture_output=True, text=True, timeout=60)
    if listing.returncode != 0:
        fail('车上证据目录读不到，请让 Claude 检查')
    (WORK/f'trial{n}-remote-sha256.txt').write_text(listing.stdout)
    if subprocess.run(scp+['-p', '-r', '-q', f'{HOST}:{remote_evidence}', str(WORK)+'/'], timeout=120).returncode != 0:
        fail('证据复制失败，请让 Claude 检查')
    lines = [l.split(None, 1) for l in listing.stdout.splitlines() if l.strip()]
    bad = [name for digest, name in lines
           if hashlib.sha256((evidence/name.strip()).read_bytes()).hexdigest() != digest]
    say(f'证据 {len(lines)-len(bad)}/{len(lines)} 个文件校验一致' + (f'，不一致：{bad}' if bad else ''))
    after = read_fresh(ssh, WORK/f'after{n}-trial-fresh.json')
    control = after['samples'][-1]['state']['status']['control']
    say(f"跑后车况：armed={control['armed']} 电机 {control['motor']} 舵机 {control['servo']}")

    try:
        result = json.loads((evidence/'run'/'summary.json').read_text())['result']
    except (OSError, ValueError, KeyError):
        fail('没有 run/summary.json，请让 Claude 检查')
    turn = result.get('turn') or {}
    events = turn.get('route_events') or {}
    seq = {name: event['scan']['seq'] for name, event in events.items()}
    say(f"路线到达：{turn.get('route_stage')}，经过阶段 {list(seq)}")
    finish = turn.get('finish_evidence') or {}
    if 'finish_stop' in seq and 'start_lane' in seq:
        say(f"起步到停车约 {(seq['finish_stop']-seq['start_lane'])/10:.1f} 秒；离终点板 {finish.get('front_gap_m', 0):.3f}m，"
            f"依据 {finish.get('basis')}，补进 {finish.get('creeps')} 次")
    say(f"质量等待计数 {result.get('quality_counts') or {}}；把结果告诉 Claude 即可继续分析")
    return 0 if code == 0 and not bad else 1


if __name__ == '__main__':
    sys.exit(main())
