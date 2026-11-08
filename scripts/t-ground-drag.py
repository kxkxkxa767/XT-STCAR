#!/usr/bin/env python3
"""Measure a floor's drag before a race, from this Mac (user, 2026-11-08).

  python3 scripts/t-ground-drag.py --length 4        one measurement (t1, t2, ...): the car moves
  python3 scripts/t-ground-drag.py --show            today's results, nothing moves
  python3 scripts/t-ground-drag.py --apply           write today's result to the car for the next race runs
  python3 scripts/t-ground-drag.py --clear           remove it: back to no initial compensation

Options: --motor 1570 (1550-1600), --push 1.6 (seconds at most), --drop t2,t3 (leave runs out).

Typing a measurement is the operator's go: the car must stand locked on a straight with at
least --length metres free ahead. It rests 2 s, pushes, coasts to a stop and rests again
(`route-engine tdrag`, in Rust on the car; the gyro holds the heading). Measure both ways along
the straight: a slope cancels in the average.

The result is the extra load against the indoor tile: the coast's deceleration less .60 m/s2.
On nine saved indoor launches (y59-y71) the coast read .578-.649, median .607, so it needs no
correction; the powered shortfall against race_surface.rs ran from -.89 to +.12 there, so it is
only printed. --apply puts the result in surface-initial.json on the car; race-trial.py hands it
to `route-engine race`, whose load learner starts from it. Below the .08 deadband it changes nothing.

Evidence: work/t-ground/tN-<time>/ (never overwritten). SSH as in scripts/start-round.py.
"""
import argparse
import bisect
import hashlib
import json
from pathlib import Path
import re
import statistics as st
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
WORK = ROOT/'work'/'t-ground'
HOST = 'bianbu@192.168.0.156'
SOCK = ROOT/'work'/'vehicle-test-ssh.sock'
BASE = '/home/bianbu/xt-stcar-console'
INITIAL = BASE+'/surface-initial.json'
CALIBRATION = ROOT/'config'/'imu-planar-y25-20261102.json'
# route-engine/src/race_surface.rs
REFERENCE_COAST = 0.60
REFERENCE_ACCEL_MAX = 4.14
DEADBAND = 0.08
RESPONSE = [(1550, 1.814, 2.98), (1570, 2.340, 1.00), (1600, 3.837, 0.70)]
ESC_ONSET_S = 0.25
COAST_MIN_MPS = 0.25
DRIFT_WARN_MPS = 0.30
SLOPE_WARN_MPS2 = 0.05

CAR_HELPER = r'''
import fcntl, json, subprocess, sys, time, urllib.request
from pathlib import Path
out, length, motor, push = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
BASE = Path('/home/bianbu/xt-stcar-console')
ACTIVE = BASE/'20260917'
SERVICE = 'xt-stcar-console.service'
def say(**row):
    print(json.dumps(row), flush=True)
program = str(ACTIVE/'route-engine')
check = subprocess.run([program, 'tdrag'], stdin=subprocess.DEVNULL, capture_output=True, text=True, timeout=10)
if 'route-engine tdrag' not in check.stderr:
    say(error='the installed route-engine has no tdrag; deploy a build from 2026-11-08 or later')
    sys.exit(3)
lock = (BASE/'race-trial.lock').open('w')
try:
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
except BlockingIOError:
    say(error='another run is in progress')
    sys.exit(3)
access = json.loads((BASE/'access.json').read_text())
opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
def state():
    req = urllib.request.Request(f"http://127.0.0.1:{access['port']}/api/state", headers={'X-Control-Token': access['token']})
    with opener.open(req, timeout=2) as r:
        return json.load(r)
def neutral(s):
    c = s['status']['control']
    return (s['healthy'] is True and s['owner'] is None and s['autonomy']['active'] is None and s['autonomy']['mode'] == 'locked'
            and c['armed'] is False and c['motor'] == c['servo'] == 1500 and 0 <= s['ages']['control'] < .2)
if not neutral(state()):
    say(error='the console is not at a fresh locked neutral')
    sys.exit(3)
root = Path(out)
root.mkdir(mode=0o700)
say(evidence_dir=out)
result = {}
devices = ('/dev/car', '/dev/laser')
busy = lambda: [d for d in devices if subprocess.run(['fuser', d], capture_output=True).returncode != 1]
subprocess.run(['systemctl', '--user', 'stop', SERVICE], check=True, timeout=30)
try:
    deadline = time.monotonic()+15
    while busy() and time.monotonic() < deadline:
        time.sleep(.2)
    if busy():
        raise RuntimeError('devices still busy after stopping the console')
    with (root/'stdout.txt').open('w') as stdout:
        p = subprocess.run([program, 'tdrag', '--bridge', str(ACTIVE/'vehicle-bridge'), '--out', str(root/'run'),
                            '--length-m', length, '--motor', motor, '--push-s', push],
                           stdout=stdout, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, timeout=60)
    result['program_exit'] = p.returncode
    time.sleep(1)
finally:
    subprocess.run(['systemctl', '--user', 'start', SERVICE], timeout=30)
    final, deadline = [], time.monotonic()+90
    while len(final) < 6 and time.monotonic() < deadline:
        try:
            s = state()
            if neutral(s) and (not final or s['status']['control']['tick'] > final[-1]['status']['control']['tick']):
                final.append(s)
            elif not neutral(s):
                final = []
        except Exception:
            pass
        time.sleep(.3)
    (root/'after-console.json').write_text(json.dumps(final, indent=2)+'\n')
    result['console_neutral_samples'] = len(final)
    (root/'helper-result.json').write_text(json.dumps(result, indent=2)+'\n')
    say(**result)
'''


def say(text):
    print(time.strftime('%H:%M:%S'), text, flush=True)


def fail(text):
    say('停止：'+text)
    raise SystemExit(1)


def ssh_base():
    if SOCK.exists() and subprocess.run(['ssh', '-S', str(SOCK), '-O', 'check', HOST], capture_output=True).returncode == 0:
        return (['ssh', '-S', str(SOCK), '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5', HOST],
                ['scp', '-o', f'ControlPath={SOCK}'])
    return ['ssh', '-o', 'ConnectTimeout=10', HOST], ['scp']


def imu_series(path):
    """(seconds, forward acceleration, yaw rate), calibrated as race_imu.rs does on the car."""
    p = json.loads(CALIBRATION.read_text())['parameters']
    forward, up = p['body_axes_in_sensor']['forward'], p['body_axes_in_sensor']['up']
    rest, bias, ga, gy = p['stationary_specific_force_mps2'], p['gyro_bias_radps'], p['forward_acceleration_gain'], p['yaw_rate_gain']
    out = []
    for line in open(path):
        r = json.loads(line)
        a, w = r['acceleration_mps2'], r['angular_velocity_radps']
        out.append((r['captured_at']/1000, ga*sum(forward[i]*(a['xyz'[i]]-rest[i]) for i in range(3)),
                    gy*sum(up[i]*(w['xyz'[i]]-bias[i]) for i in range(3))))
    return out


def reference_accel(pwm, speed):
    target, response = RESPONSE[-1][1], RESPONSE[-1][2]
    for (p0, t0, r0), (p1, t1, r1) in zip(RESPONSE, RESPONSE[1:]):
        if pwm <= p1:
            w = max(0., min(1., (pwm-p0)/(p1-p0)))
            target, response = t0+w*(t1-t0), r0+w*(r1-r0)
            break
    return max(-REFERENCE_COAST, min(REFERENCE_ACCEL_MAX, (target-speed)/response))


def mean_in(imu, a, b, index=1):
    v = [row[index] for row in imu if a <= row[0] <= b]
    return st.fmean(v) if v else None


def slope(points):
    mx, my = st.fmean(p[0] for p in points), st.fmean(p[1] for p in points)
    return sum((x-mx)*(y-my) for x, y in points)/sum((x-mx)**2 for x, _ in points)


def analyse(imu, phases, pwm_at):
    """Speeds, the coast's deceleration and the powered response's shortfall of one push and coast.

    phases: {'rest1': (a, b), 'push': (a, b), 'coast': (a, b), 'rest2': (a, b)}; pwm_at(t): the push's PWM.
    """
    rest1, push, coast, rest2 = phases['rest1'], phases['push'], phases['coast'], phases.get('rest2')
    bias1 = mean_in(imu, rest1[0]+.3, rest1[1])
    bias2 = mean_in(imu, rest2[0]+.3, rest2[1]) if rest2 else None
    if bias1 is None:
        return {'error': '起步前静止段没有 IMU 数据'}
    bias2 = bias1 if bias2 is None else bias2
    # The stop: the deceleration ends abruptly; the mean over .25 s rises to about the rest value.
    t0 = rest1[1]
    t_stop = coast[1]
    after = [row for row in imu if coast[0]+.5 <= row[0] <= (rest2[1] if rest2 else coast[1])]
    for row in after:
        window = [r[1]-bias2 for r in after if row[0] <= r[0] <= row[0]+.25]
        if len(window) >= 20 and st.fmean(window) > -.2 and max(abs(r[2]) for r in after if row[0] <= r[0] <= row[0]+.25) < .05:
            t_stop = row[0]
            break
    series, v, last = [], 0., t0
    for t, a, w in imu:
        if t < t0 or t > t_stop:
            continue
        b = bias1+(bias2-bias1)*(t-t0)/max(1e-6, t_stop-t0)
        v += (a-b)*(t-last)
        last = t
        series.append((t, v))
    if len(series) < 20:
        return {'error': '运动段的 IMU 数据太少'}
    drift = series[-1][1]
    span = max(1e-6, t_stop-t0)
    speed = [(t, x-drift*(t-t0)/span) for t, x in series]
    times = [p[0] for p in speed]

    def speed_at(t):
        i = max(0, min(len(times)-1, bisect.bisect_left(times, t)))
        return speed[i][1]

    window = [p for p in speed if push[1]+ESC_ONSET_S <= p[0] <= t_stop and p[1] >= COAST_MIN_MPS]
    decel = -slope(window) if len(window) >= 20 and window[-1][0]-window[0][0] >= .5 else None
    extras, t = [], push[0]+ESC_ONSET_S
    while t+.1 <= push[1]:
        a = mean_in(imu, t, t+.1)
        b = bias1+(bias2-bias1)*(t+.05-t0)/span
        v_mid = speed_at(t+.05)
        if a is not None and v_mid >= COAST_MIN_MPS:
            extras.append(reference_accel(pwm_at(t+.05), v_mid)-(a-b))
        t += .1
    distance = sum((b[0]-a[0])*(a[1]+b[1])/2 for a, b in zip(speed, speed[1:]))
    return {'coast_decel': decel, 'coast_extra': None if decel is None else decel-REFERENCE_COAST,
            'power_extra': st.fmean(extras) if extras else None, 'power_samples': len(extras),
            'peak_speed': max(p[1] for p in speed), 'distance': distance, 'drift': drift,
            'bias_before': bias1, 'bias_after': bias2, 'stopped_at': t_stop-t0,
            'coast_window_s': (window[-1][0]-window[0][0]) if window else 0.}


def analyse_dir(path):
    summary = json.loads((path/'run'/'summary.json').read_text())
    if summary.get('reason') != 'done':
        return {'error': f"车上结束原因 {summary.get('reason')}（{summary.get('detail') or summary.get('guard')}）"}
    motor = summary['motor']
    phases = {name: (a, b) for name, a, b in summary['phases']}
    if 'push' not in phases or 'coast' not in phases:
        return {'error': f'没有完整的推进和滑行：{list(phases)}（{summary.get("guard")}）'}
    if 'brake' in phases:
        return {'error': f'中途刹车（{summary.get("guard")}），这次不算；换个更空的地方或把 --length 改小'}
    result = analyse(imu_series(path/'run'/'imu.jsonl'), phases, lambda _: motor)
    result.update(motor=motor, guard=summary.get('guard'), heading_deg=summary.get('peak_heading_rad', 0)*57.2958)
    return result


def label_of(path):
    return path.name.split('-')[0]


def runs_of_today():
    day = time.strftime('%Y%m%d')
    found = [p for p in WORK.glob(f't*-{day}-*') if (p/'run'/'summary.json').is_file()]
    return sorted(found, key=lambda p: int(label_of(p)[1:]))


def combine(drop):
    rows = []
    for path in runs_of_today():
        if label_of(path) not in drop:
            rows.append((label_of(path), analyse_dir(path)))
    good = [r for _, r in rows if 'error' not in r and r.get('coast_decel') is not None]
    coast = st.median(r['coast_extra'] for r in good) if good else None
    power = [r['power_extra'] for r in good if r.get('power_extra') is not None]
    power = st.median(power) if power else None
    load = None if coast is None else max(0., coast)
    return rows, coast, power, load, st.median(r['coast_decel'] for r in good) if good else None


def show(drop):
    rows, coast, power, load, decel = combine(drop)
    if not rows:
        say('今天还没有测量')
        return None
    for label, r in rows:
        if 'error' in r:
            say(f'{label}：无效，{r["error"]}')
            continue
        d = 'n/a' if r.get('coast_decel') is None else f"{r['coast_decel']:.3f}"
        p = 'n/a' if r.get('power_extra') is None else f"{r['power_extra']:+.3f}"
        notes = []
        if abs(r['drift']) > DRIFT_WARN_MPS:
            notes.append(f"IMU 末速残差 {r['drift']:+.2f} m/s，这次不太可信")
        if abs(r['bias_after']-r['bias_before']) > SLOPE_WARN_MPS2:
            notes.append('前后静止加速度不同，地面可能有坡')
        if r.get('guard'):
            notes.append(f"中途触发 {r['guard']}")
        say(f"{label}：滑行减速度 {d} m/s²，动力差（仅参考）{p} m/s²，最高 {r['peak_speed']:.2f} m/s，行程 {r['distance']:.2f} m，"
            f"航向偏差 {r['heading_deg']:.1f}°"+(f"（{'；'.join(notes)}）" if notes else ''))
    if load is None:
        say('没有有效的滑行数据，算不出结果')
        return None
    say(f'今天 {len(rows)} 次的中位数：滑行减速度 {decel:.3f} m/s²（室内瓷砖 {REFERENCE_COAST:.2f}），'
        f'滑行多出 {coast:+.3f}，动力差（仅参考）{"n/a" if power is None else f"{power:+.3f}"}')
    say(f'建议初始阻力补偿 {load:.3f} m/s²'+('（低于 0.08 的死区，比赛时不会加 PWM）' if load <= DEADBAND else ''))
    if decel < REFERENCE_COAST-.1:
        say('注意：这块地比室内滑得更远，斑马线收油用的 0.6 可能偏大，车会多滑；把这个数告诉 Claude')
    return load, coast, power, decel


def measure(args):
    WORK.mkdir(parents=True, exist_ok=True)
    numbers = [int(m.group(1)) for p in WORK.iterdir() if (m := re.match(r't(\d+)-', p.name))]
    label = f't{max(numbers, default=0)+1}'
    stamp = time.strftime('%Y%m%d-%H%M%S')
    name = f'{label}-{stamp}'
    remote = f'{BASE}/t-ground-{stamp}'
    ssh, scp = ssh_base()
    say(f'{label}：车须锁定、停在直道上，前方至少 {args.length:g} m 空地；开始（约 12 秒）')
    done = subprocess.run(ssh+['python3', '-', remote, str(args.length), str(args.motor), str(args.push)],
                          input=CAR_HELPER, capture_output=True, text=True, timeout=180)
    printed = []
    for line in done.stdout.splitlines():
        try:
            printed.append(json.loads(line))
        except ValueError:
            pass
    errors = [row['error'] for row in printed if 'error' in row]
    if errors or not any('evidence_dir' in row for row in printed):
        fail((errors[0] if errors else done.stderr.strip()[-300:]) or '车上助手没有输出')
    say('取回证据')
    listing = subprocess.run(ssh+[f'cd {remote} && find . -type f | sort | xargs sha256sum'], capture_output=True, text=True, timeout=60)
    if listing.returncode != 0:
        fail('车上证据读不到')
    local = WORK/name
    if subprocess.run(scp+['-p', '-r', '-q', f'{HOST}:{remote}', str(local)], timeout=120).returncode != 0:
        fail('证据复制失败')
    lines = [line.split(None, 1) for line in listing.stdout.splitlines() if line.strip()]
    bad = [n for digest, n in lines if hashlib.sha256((local/n.strip()).read_bytes()).hexdigest() != digest]
    say(f'证据 {len(lines)-len(bad)}/{len(lines)} 个文件校验一致'+(f'，不一致：{bad}' if bad else ''))
    if printed[-1].get('console_neutral_samples') != 6:
        say('注意：跑后没有确认车已锁定中位，请检查车')
    summary = local/'run'/'summary.json'
    if not summary.is_file():
        fail('没有 run/summary.json，车可能没动：'+(local/'stdout.txt').read_text()[-300:])
    show(set())


def apply(drop):
    result = show(drop)
    if result is None:
        fail('没有结果可写')
    load, coast, power, decel = result
    ssh, _ = ssh_base()
    labels = [label_of(p) for p in runs_of_today() if label_of(p) not in drop]
    row = {'extra_load_mps2': round(load, 3), 'coast_decel_mps2': round(decel, 3), 'coast_extra_mps2': round(coast, 3),
           'power_extra_mps2': None if power is None else round(power, 3), 'source': ','.join(labels),
           'measured': time.strftime('%Y-%m-%d %H:%M')}
    if subprocess.run(ssh+[f'cat > {INITIAL}'], input=json.dumps(row)+'\n', text=True, timeout=20).returncode != 0:
        fail('写到车上失败')
    say(f'已写到车上：之后每轮比赛的初始阻力补偿 {row["extra_load_mps2"]:.3f} m/s²（来自 {row["source"]}）')


def clear():
    ssh, _ = ssh_base()
    if subprocess.run(ssh+[f'rm -f {INITIAL}'], timeout=20).returncode != 0:
        fail('车上删除失败')
    say('已清除：之后每轮比赛的初始阻力补偿为 0')


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--length', type=float)
    parser.add_argument('--motor', type=int, default=1570)
    parser.add_argument('--push', type=float, default=1.6)
    parser.add_argument('--show', action='store_true')
    parser.add_argument('--apply', action='store_true')
    parser.add_argument('--clear', action='store_true')
    parser.add_argument('--drop', default='')
    args = parser.parse_args()
    drop = {x for x in args.drop.split(',') if x}
    if args.clear:
        clear()
    elif args.apply:
        apply(drop)
    elif args.show:
        show(drop)
    elif args.length:
        if not 1.5 <= args.length <= 30 or not 1550 <= args.motor <= 1600 or not 0 < args.push <= 3:
            fail('--length 1.5-30，--motor 1550-1600，--push 0-3')
        measure(args)
    else:
        raise SystemExit(__doc__)


if __name__ == '__main__':
    main()
