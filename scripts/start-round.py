#!/usr/bin/env python3
"""Start one built and deployed round on the car from this Mac (not on the car).

  python3 scripts/start-round.py N        (round N: work/.../claude-turnN-package.json)
  python3 scripts/start-round.py yK       (round yK with the YOLO log: claude-yK-package.json)

Typing it is the operator's go: the car must already stand at the start. A
round can run any number of times (user, 10-24): the car-side helper gives each
run a new evidence directory (left-cone-trial-<date>-N, then -N-2, -N-3, ...),
and this Mac's files of a run carry the time it started (preN-<time>-fresh.json,
trialN-<time>-stdout.json, ...), so nothing is overwritten. Steps:
1. the car's fresh state (work/.../read-fresh-engine.py) must be six fresh
   locked-neutral ready samples whose 17 installed hashes match
   claude-turnN-package.json (validate-fresh-engine.py); otherwise nothing starts;
2. the round's packaged helper STAGE/trialN.py runs on the car (it checks
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
import re
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
    if len(sys.argv) != 2 or not re.fullmatch(r'\d+|y\d+', sys.argv[1]):
        raise SystemExit(__doc__)
    n = sys.argv[1]
    package = WORK/(f'claude-turn{n}-package.json' if n.isdigit() else f'claude-{n}-package.json')
    if not package.is_file():
        fail(f'没有第{n}轮的部署包记录 {package.name}')
    pkg = json.loads(package.read_text())
    stamp = time.strftime('%Y%m%d-%H%M%S')
    stdout_path = WORK/f'trial{n}-{stamp}-stdout.json'
    if stdout_path.exists():
        fail(f'{stdout_path.name} 已存在，过一秒再试')
    ssh, scp = ssh_base()

    say(f'第{n}轮：读车上状态并核对部署（车须锁定、中位、在起点）')
    pre = WORK/f'pre{n}-{stamp}-fresh.json'
    fresh = read_fresh(ssh, pre)
    check = subprocess.run([sys.executable, str(WORK/'validate-fresh-engine.py'), str(pre), str(package)],
                           capture_output=True, text=True)
    if check.returncode != 0:
        fail('车上状态或安装文件与第%s轮部署不符，没有启动：%s' % (n, (check.stderr or check.stdout).strip()[-300:]))
    ranges = fresh['samples'][-1]['lidar']['scan']['ranges']
    side = lambda i: '-' if ranges[i] is None else f'{ranges[i]:.2f}m'
    say(f'核对通过：{check.stdout.strip()}')
    say(f'起点读数：左 {side(270)}，右 {side(90)}，前方 {side(0)}，后方 {side(180)}')

    say('开跑（约30秒，跑完车自动锁定）')
    stderr_path = WORK/f'trial{n}-{stamp}-stderr.txt'
    with open(stdout_path, 'wb') as out, open(stderr_path, 'wb') as err:
        code = subprocess.run(ssh+[f"python3 {pkg['stage']}/trial{n}.py"], stdout=out, stderr=err,
                              timeout=300).returncode
    printed = []
    for line in stdout_path.read_text().splitlines():
        try:
            printed.append(json.loads(line))
        except ValueError:
            pass
    helper = printed[-1] if printed and 'reason' in printed[-1] else {}
    remote_evidence = next((row['evidence_dir'] for row in printed if 'evidence_dir' in row), None)
    say(f"车上助手退出码 {code}，结束原因 {helper.get('reason')}")
    if not helper:
        say('车上助手的提示：' + stderr_path.read_text().strip()[-300:])
    if remote_evidence is None:
        fail('车上没有建证据目录、没有启动：' + stderr_path.read_text().strip()[-300:])
    evidence = WORK/Path(remote_evidence).name
    if evidence.exists():
        fail(f'本机已有 {evidence.name}，没有覆盖，请让 Claude 检查')
    say(f'这次的证据目录：{evidence.name}')

    say('取回证据')
    listing = subprocess.run(ssh+[f'cd {remote_evidence} && find . -type f | sort | xargs sha256sum'],
                             capture_output=True, text=True, timeout=60)
    if listing.returncode != 0:
        fail('车上证据目录读不到，请让 Claude 检查')
    (WORK/f'trial{n}-{stamp}-remote-sha256.txt').write_text(listing.stdout)
    if subprocess.run(scp+['-p', '-r', '-q', f'{HOST}:{remote_evidence}', str(WORK)+'/'], timeout=120).returncode != 0:
        fail('证据复制失败，请让 Claude 检查')
    lines = [l.split(None, 1) for l in listing.stdout.splitlines() if l.strip()]
    bad = [name for digest, name in lines
           if hashlib.sha256((evidence/name.strip()).read_bytes()).hexdigest() != digest]
    say(f'证据 {len(lines)-len(bad)}/{len(lines)} 个文件校验一致' + (f'，不一致：{bad}' if bad else ''))
    after = read_fresh(ssh, WORK/f'after{n}-{stamp}-fresh.json')
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
    vision = helper.get('vision_summary')
    if vision or helper.get('vision_exit') is not None:
        say(f"识别记录（只读）：{vision.get('frames')} 帧，{vision.get('fps')} 帧/秒，推理中位 {vision.get('infer_ms_median')} ms，"
            f"各类检出 {vision.get('counts')}" if vision else f"识别记录没有小结（退出码 {helper.get('vision_exit')}），请让 Claude 检查")
    say(f"质量等待计数 {result.get('quality_counts') or {}}；把结果告诉 Claude 即可继续分析")
    return 0 if code == 0 and not bad else 1


if __name__ == '__main__':
    sys.exit(main())
