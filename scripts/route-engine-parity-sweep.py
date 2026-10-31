#!/usr/bin/env python3
"""Parity sweep: course-sim cases through route-engine-parity.py, in parallel.

  python3 scripts/route-engine-parity-sweep.py OUT_DIR [all|range|matrix|measured|sensors|light|replay] [--supervisor]

light (round y2): the traffic light (course-sim --gantry --light) with the
lamps off or green before the trigger, a camera losing the light near it, no
light seen at all, short and long reds, a faster car; round y3: the green lamp
out of view near it (green_min_ahead_m), with a 5 s red, and a 6 s red with the
camera losing the light near it; round y5: a crosswalk 2.46 or 1.4 m before
the start (else the start lane's fallback stop in the first turn); round y11:
the glued camera's view (lamps to .60 m from the lidar) is the default, the
light lost from .80 instead, and crosswalks 3.0 m ahead and ranged 10 % long.
replay: route-replay.py over every recorded car trial from round 64 on
(work/vehicle-dynamic-deploy-20261008/left-cone-trial-*-N, N >= 64).

One line per case (OK, or FAIL with the first difference); each case's log and,
on a mismatch, the call's arguments (JSON) are written to OUT_DIR. With
--supervisor each case runs under route-engine-supervisor-parity.py (the
console's route supervision against its Rust port) instead.
"""
import concurrent.futures as cf, importlib.util, subprocess, sys, json, time
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
supervisor = '--supervisor' in sys.argv[1:]
argv = [a for a in sys.argv[1:] if a != '--supervisor']
OUT = Path(argv[0]); OUT.mkdir(parents=True, exist_ok=True)
which = argv[1] if len(argv) > 1 else 'all'
HARNESS = ROOT/'scripts'/('route-engine-supervisor-parity.py' if supervisor else 'route-engine-parity.py')
spec = importlib.util.spec_from_file_location('cs_par', ROOT/'scripts/course-sim.py')
cs = importlib.util.module_from_spec(spec); spec.loader.exec_module(cs)
cases = []
if which in ('all', 'range'):
    cases += [(f'range-{n}', ['--variant', n]) for n, _ in cs.variations()]
if which in ('all', 'matrix'):
    for n in sorted(cs.LAYOUTS):
        for sp, de, dl in ((1., .5, .15), (1.2, .35, .3), (.8, .7, .15)):
            cases.append((f'matrix-{n}-{sp}-{de}-{dl}', ['--layout', n, '--speed-scale', str(sp), '--decel', str(de), '--delay', str(dl)]))
if which in ('all', 'measured'):
    for n in sorted(cs.LAYOUTS):
        cases.append((f'measured-{n}', ['--layout', n, '--vehicle', 'measured']))
    for n in ('user8', 'doc8x4'):
        for sc, d in ((1.0, .3), (1.3, .15), (1.3, .3)):
            cases.append((f'measured-{n}-{sc}-{d}', ['--layout', n, '--vehicle', 'measured', '--vehicle-param', f'speed_scale={sc}', '--delay', str(d)]))
if which in ('all', 'sensors'):
    for n in ('user8', 'doc8x4', 'short5'):
        for seed in (1, 2):
            cases.append((f'sensors-{n}-edge-{seed}', ['--layout', n, '--edge-dropout', '.15', '--seed', str(seed)]))
            cases.append((f'sensors-{n}-graze-{seed}', ['--layout', n, '--grazing-angle-deg', '25', '--grazing-dropout', '.5', '--seed', str(seed)]))
            cases.append((f'sensors-{n}-backlash-{seed}', ['--layout', n, '--steer-backlash', '30', '--seed', str(seed)]))
if which in ('all', 'light'):
    lights = (('default', 0., '{}', ()), ('red3-at1', 1., '{"red_s": 3}', ()), ('green-at2', 2., '{"pre": "green"}', ()),
              ('near.8', 0., '{"min_ahead_m": 0.8}', ()), ('unseen', 0., '{"score": 0.0}', ()),
              ('greennear.6', 0., '{"green_min_ahead_m": 0.6}', ()),
              ('greennear.6-red5', 0., '{"green_min_ahead_m": 0.6, "red_s": 5}', ()),
              ('near.8-red6', 0., '{"min_ahead_m": 0.8, "red_s": 6}', ()),
              ('crosswalk2.46', 0., '{"crosswalk_m": 2.46}', ()),
              ('crosswalk1.4', 0., '{"crosswalk_m": 1.4}', ()),
              ('crosswalk3.0', 0., '{"crosswalk_m": 3.0}', ()),
              ('crosswalk2.46-long1.1', 0., '{"crosswalk_m": 2.46, "crosswalk_scale": 1.1}', ()),
              ('fast', 0., '{}', ('--vehicle-param', 'speed_scale=1.3', '--vehicle-param', 'gain_scale=1.3')))
    for n in ('user8', 'doc8x4', 'short5'):
        for tag, at, light, extra in lights:
            cases.append((f'light-{n}-{tag}', ['--layout', n, '--vehicle', 'measured', '--gantry', '--gantry-at', str(at),
                                               '--light', light, '--max-s', '75', *extra]))

cases = [(name, ['scripts/course-sim.py', *args]) for name, args in cases]
if which in ('all', 'replay'):
    trials = [p for p in (ROOT/'work/vehicle-dynamic-deploy-20261008').glob('left-cone-trial-*-*')
              if p.is_dir() and p.name.rsplit('-', 1)[1].isdigit()]
    for trial in sorted(trials, key=lambda p: int(p.name.rsplit('-', 1)[1])):
        if int(trial.name.rsplit('-', 1)[1]) >= 64 and (trial/'feedback.jsonl').is_file():
            cases.append((f'replay-{trial.name}', ['scripts/route-replay.py', str(trial)]))

def one(case):
    name, (tool, *args) = case
    t = time.time()
    p = subprocess.run([sys.executable, str(HARNESS), '--dump', str(OUT/f'{name}.json'),
                        '--', str(ROOT/tool), *args], capture_output=True, text=True, cwd=ROOT)
    (OUT/f'{name}.log').write_text(p.stdout+p.stderr)
    lines = [l for l in p.stderr.splitlines() if 'PARITY' in l or 'MISMATCH' in l or l.startswith('  python')
             or l.startswith('  rust') or 'calls compared' in l or 'Error' in l]
    return name, p.returncode, round(time.time()-t, 1), lines[-4:]

bad = 0
with cf.ThreadPoolExecutor(9) as ex:
    for name, code, dt, lines in ex.map(one, cases):
        status = 'OK' if code == 0 else f'FAIL({code})'
        bad += code != 0
        print(status, name, dt, ' | '.join(lines) if code else (lines[-1] if lines else ''), flush=True)
print(f'{len(cases)-bad}/{len(cases)} matched')
