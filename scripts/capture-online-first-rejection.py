#!/usr/bin/env python3
"""Capture the first small-field async refusal in two isolated native-host builds.

Pinned historical evidence, not an alternate production controller. Both runs
must retain their expected nonzero race-failure exit. No network or device I/O.
"""
import argparse
import hashlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tarfile

REVISION = 'bf2e8e4be0416909d109e803522a88929ce391f3'
TIMES = (33180, 33260, 33380)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + '\n')


def ledger(source):
    return {str(p.relative_to(source)): sha(p) for p in sorted(source.rglob('*'))
            if p.is_file() and 'target' not in p.relative_to(source).parts
            and (p.suffix == '.rs' or p.name in ('Cargo.toml', 'Cargo.lock')
                 or ('fixtures' in p.parts and p.suffix == '.json'))}


def differences(a, b, path=''):
    if type(a) is not type(b):
        return [path]
    if isinstance(a, dict):
        result = []
        for key in sorted(a.keys() | b.keys()):
            result.extend([path + '/' + key] if key not in a or key not in b
                          else differences(a[key], b[key], path + '/' + key))
        return result
    if isinstance(a, list):
        if len(a) != len(b):
            return [path + '/length']
        return [p for i, (x, y) in enumerate(zip(a, b))
                for p in differences(x, y, path + '/' + str(i))]
    return [] if a == b else [path]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='work/online-first-rejection-recapture')
    parser.add_argument('--prepare-only', action='store_true')
    args = parser.parse_args()
    root = Path(__file__).resolve().parent.parent
    out = (root / args.output).resolve()
    work = (root / 'work').resolve()
    if not out.is_relative_to(work) or out == work or out.exists():
        parser.error('--output must be a fresh directory below repository work/')
    archive = subprocess.check_output(['git', 'archive', REVISION], cwd=root)
    out.mkdir(parents=True)
    hook = root / 'scripts/capture-online-first-rejection/instrument.py'
    report = {'schema_version': 1, 'source_commit': REVISION,
              'scope': 'Native Mac synthetic async capture; no RISC-V execution or physical output. No runtime safety/geometry/TTL/budget change.',
              'script_sha256': sha(Path(__file__).resolve()), 'hook_sha256': sha(hook),
              'archive_sha256': hashlib.sha256(archive).hexdigest(), 'runs': {}}
    for name in ('baseline', 'instrumented'):
        run = out / name
        source = run / 'source'
        source.mkdir(parents=True)
        with tarfile.open(fileobj=io.BytesIO(archive)) as tar:
            for member in tar.getmembers():
                if not (source / member.name).resolve().is_relative_to(source) or not (member.isfile() or member.isdir()):
                    raise ValueError('unsupported archive member: ' + member.name)
            tar.extractall(source, filter='data')
        if name == 'instrumented':
            subprocess.run([sys.executable, str(hook), str(source)], check=True)
        entry = {'source_before': ledger(source)}
        report['runs'][name] = entry
        if args.prepare_only:
            continue
        env = dict(os.environ, CARGO_TARGET_DIR=str(source / 'target'))
        build = ['cargo', 'build', '--release', '--locked', '--offline',
                 '-p', 'xt-stcar-robot-runner', '--example', 'online_matrix']
        entry['build_command'] = build
        with (run / 'build.log').open('wb') as log:
            subprocess.run(build, cwd=source, env=env, stdout=log, stderr=subprocess.STDOUT, check=True)
        binary = source / 'target/release/examples/online_matrix'
        entry['binary_sha256'] = sha(binary)
        command = [str(binary), 'async-trace', 'small_5x4']
        entry['command'] = command
        with (run / 'trace.json').open('wb') as stdout, (run / 'stderr.log').open('wb') as stderr:
            entry['exit_code'] = subprocess.run(command, cwd=source, env=env, stdout=stdout, stderr=stderr).returncode
        entry['source_after'] = ledger(source)
        assert entry['source_after'] == entry['source_before']
        entry['files_sha256'] = {f: sha(run / f) for f in ('trace.json', 'stderr.log', 'build.log')}
        assert entry['exit_code'] == 1, 'Small-field failure must remain visible'
        save(out / 'provenance.json', report)
    if args.prepare_only:
        save(out / 'provenance.json', report)
        return
    baseline = json.loads((out / 'baseline/trace.json').read_text())
    observed = json.loads((out / 'instrumented/trace.json').read_text())
    diff = differences(baseline, observed)
    assert set(diff) <= {'/results/0/summary/wall_time_ms'}, diff
    count = len(observed['results'][0]['offline_plan_trace'])
    assert count == 439 and observed['results'][0]['omitted_trace_plans'] == 0
    report['comparison'] = {'different_paths': diff, 'plans': count,
                            'all_functional_fields_equal': True}
    frames = []
    for time in TIMES:
        path = out / f'instrumented/capture-Timestamp({time}).json'
        report.setdefault('capture_sha256', {})[str(time)] = sha(path)
        data = json.loads(path.read_text())
        assert data['at'] == time
        if time != 33260:
            data['event_count'] = len(data.pop('events'))
        frames.append(data)
    fixture = {'schema_version': 1, 'source_commit': REVISION,
               'scope': 'Actual native-host small_5x4 async worker capture. Complete raw input, Navigator state before boundary and before/after plan. Full terminal/primitive event stream at first refusal; neighbor event streams remain in local capture. This is failure evidence, not race acceptance.',
               'trace_comparison': 'All 439 offline plans and every JSON field agree with uninstrumented run except summary.wall_time_ms.',
               'frames': frames}
    save(out / 'fixture.json', fixture)
    report['fixture_sha256'] = sha(out / 'fixture.json')
    save(out / 'provenance.json', report)
    print(json.dumps({'output': str(out), 'plans': count, 'functional_fields_equal': True,
                      'race_completed': False, 'fixture_sha256': report['fixture_sha256']}))


if __name__ == '__main__':
    main()
