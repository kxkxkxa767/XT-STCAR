#!/usr/bin/env python3
"""Parity sweep: course-sim cases through route-engine-parity.py, in parallel.

  python3 scripts/route-engine-parity-sweep.py OUT_DIR [all|range|matrix|measured|sensors]

One line per case (OK, or FAIL with the first difference); each case's log and,
on a mismatch, the call's arguments (JSON) are written to OUT_DIR.
"""
import concurrent.futures as cf, importlib.util, subprocess, sys, json, time
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
OUT = Path(sys.argv[1]); OUT.mkdir(parents=True, exist_ok=True)
which = sys.argv[2] if len(sys.argv) > 2 else 'all'
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

def one(case):
    name, args = case
    t = time.time()
    p = subprocess.run([sys.executable, str(ROOT/'scripts/route-engine-parity.py'), '--dump', str(OUT/f'{name}.json'),
                        '--', str(ROOT/'scripts/course-sim.py'), *args], capture_output=True, text=True, cwd=ROOT)
    (OUT/f'{name}.log').write_text(p.stdout+p.stderr)
    lines = [l for l in p.stderr.splitlines() if 'PARITY' in l or l.startswith('  python') or l.startswith('  rust') or 'calls compared' in l or 'Error' in l]
    return name, p.returncode, round(time.time()-t, 1), lines[-4:]

bad = 0
with cf.ThreadPoolExecutor(9) as ex:
    for name, code, dt, lines in ex.map(one, cases):
        status = 'OK' if code == 0 else f'FAIL({code})'
        bad += code != 0
        print(status, name, dt, ' | '.join(lines) if code else (lines[-1] if lines else ''), flush=True)
print(f'{len(cases)-bad}/{len(cases)} matched')
