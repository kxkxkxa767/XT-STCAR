#!/usr/bin/env python3
"""CPU-only, frozen-binary batch simulations. No vehicle I/O or automatic code edits."""
import argparse
import concurrent.futures
import gzip
import hashlib
import itertools
import json
import os
from pathlib import Path
import shutil
import sqlite3
import subprocess
import time
import fcntl

ROOT = Path(__file__).resolve().parents[1]
AXES = {
    'length_m': [5 + i * .5 for i in range(7)],
    'width_m': [4 + i * .5 for i in range(5)],
    'bottom_straight_span_m': [3 + i * .5 for i in range(7)],
    'top_straight_span_m': [3 + i * .5 for i in range(7)],
    'bottom_lane_width_m': [1, 1.5, 2],
    'top_lane_width_m': [1, 1.5, 2],
}


def encode(value):
    return json.dumps(value, sort_keys=True, ensure_ascii=False, separators=(',', ':')).encode()


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def source_digest():
    names = sorted(set(subprocess.check_output(['git', 'ls-files', 'Cargo*', 'crates', 'scripts/local-sim-sweep.py'], cwd=ROOT, text=True).splitlines()) | {str(p.relative_to(ROOT)) for p in (ROOT / 'crates').rglob('*.rs')} | {'scripts/local-sim-sweep.py'})
    return hashlib.sha256(b''.join(encode(n) + bytes.fromhex(digest(ROOT / n)) for n in names if (ROOT / n).is_file())).hexdigest()


def connect(out):
    db = sqlite3.connect(out / 'queue.sqlite')
    db.execute('CREATE TABLE IF NOT EXISTS jobs (id TEXT PRIMARY KEY, family TEXT, mode TEXT, input TEXT, status TEXT, accepted INTEGER, seconds REAL, error TEXT)')
    return db


def initialize(out, binary):
    out.mkdir(parents=True, exist_ok=True)
    if (out / 'identity.json').exists():
        raise SystemExit('Existing round: use run to resume, or a different output directory.')
    subprocess.run([str(binary), 'example'], check=True, stdout=subprocess.DEVNULL)
    shutil.copy2(binary, out / 'lidar_sweep')
    template = json.loads(subprocess.check_output([str(out / 'lidar_sweep'), 'example']))
    template['scoped_ideal_semantics'] = False
    identity = {'binary_sha256': digest(out / 'lidar_sweep'), 'source_sha256': source_digest(),
                'base_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
                'axes': AXES, 'scope': 'finite 0.5m geometry grid, cone/visibility/noise samples; not continuous-range proof or actual YOLO',
                'workers': 2, 'created': time.time(), 'host_timeout_seconds': 900}
    snapshot = out / 'source_snapshot'
    names = subprocess.check_output(['git', 'ls-files'], cwd=ROOT, text=True).splitlines()
    names += [str(p.relative_to(ROOT)) for p in (ROOT/'crates').rglob('*.rs')] + ['scripts/local-sim-sweep.py']
    manifest = {}
    for name in sorted(set(names)):
        src = ROOT / name
        if src.is_file():
            dst = snapshot / name
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dst)
            manifest[name] = digest(dst)
    (out / 'source-manifest.json').write_bytes(encode(manifest))
    identity['source_manifest_sha256'] = digest(out/'source-manifest.json')
    rules = ROOT / '赛项三-RISC-V_轻量无人车赛比赛规则（初稿）.pdf'
    identity['rules_sha256'] = digest(rules) if rules.exists() else None
    db = connect(out)

    def add(family, scene):
        for mode in ['sync', 'async']:
            key = hashlib.sha256(encode([identity['binary_sha256'], mode, scene])).hexdigest()[:24]
            db.execute('INSERT OR IGNORE INTO jobs VALUES (?,?,?,?,?,NULL,NULL,NULL)', (key, family, mode, encode(scene).decode(), 'pending'))

    # Smoke scenarios precede exhaustive enumeration. Exact duplicates share one key.
    for length, width, bottom, top in [(7,5,4.5,4.5),(5,4,3,3),(8,6,6,3),(8,4,3,6),(7,5,3,6),(7,5,6,3)]:
        for walls in [False, True]:
            scene = json.loads(json.dumps(template))
            scene['spec'].update(length_m=length, width_m=width, bottom_straight_span_m=bottom, top_straight_span_m=top,
                                 light_front_x_m=length-.8, right_cone_from_bottom_m=width/2)
            scene['corridor_walls'] = walls
            add('smoke', scene)
    for walls in [False, True]:
        scene = json.loads(json.dumps(template))
        scene.update(scoped_ideal_semantics=True, corridor_walls=walls)
        add('restricted-visibility', scene)
    # Sweep cone offsets, crossing location, red duration and bounded lidar noise independently.
    for left, right, top in itertools.product([1,1.5,2,2.5,3], [1,1.5,2,2.5,3], [2,2.5,3]):
        scene = json.loads(json.dumps(template))
        scene['spec'].update(left_cone_from_left_m=left, right_cone_from_right_m=right, left_cone_from_top_m=top)
        add('cone-range', scene)
    for crossing, red, noise, walls in itertools.product([1.2,2.5,4.2], [3000,5000,10000], [0,.002,.03], [False,True]):
        scene = json.loads(json.dumps(template))
        scene.update(crosswalk_near_x_m=crossing, light_red_duration_ms=red, lidar_range_noise_m=noise, corridor_walls=walls)
        add('stress', scene)
    for values in itertools.product(*AXES.values()):
        dims = dict(zip(AXES, values))
        for walls in [False, True]:
            scene = json.loads(json.dumps(template))
            scene['spec'].update(dims)
            scene['spec'].update(light_front_x_m=dims['length_m']-.8, right_cone_from_bottom_m=dims['width_m']/2)
            scene['corridor_walls'] = walls
            add('geometry', scene)
    db.commit()
    identity['jobs'] = db.execute('SELECT COUNT(*) FROM jobs').fetchone()[0]
    (out / 'identity.json').write_bytes(encode(identity))
    db.close()
    print(json.dumps(identity, ensure_ascii=False, indent=2))


def execute(out, row):
    key, family, mode, input_json = row
    case = out / 'cases' / key
    case.mkdir(parents=True, exist_ok=True)
    (case / 'input.json').write_text(input_json)
    started = time.monotonic()
    error = None
    try:
        with (case / 'stdout.json').open('wb') as stdout, (case / 'stderr.txt').open('wb') as stderr:
            proc = subprocess.run([str(out / 'lidar_sweep'), mode, str(case / 'input.json'), str(case)], stdout=stdout, stderr=stderr, timeout=900)
        record = json.loads((case / 'stdout.json').read_bytes())
        accepted = record.get('accepted') is True and proc.returncode == 0
        status = 'accepted' if accepted else record.get('status', 'invalid_output')
        if status == 'finished':
            status = 'failed'
        record.update(returncode=proc.returncode, family=family, job_id=key)
        with gzip.open(case / 'result.json.gz', 'wb') as stream:
            stream.write(encode(record))
        summary = record.get('summary', {})
        brief = {k: summary.get(k) for k in ['completed','fault','elapsed_ms','distance_m','final_pose','online_referee','crosswalk_hold_ms','green_observed_ms','final_actual_speed_mps','final_actual_curvature_per_m','physical_output_enabled']}
        (case / 'result.json').write_bytes(encode(dict(status=status, accepted=accepted, returncode=proc.returncode, summary=brief)))
    except subprocess.TimeoutExpired:
        status, accepted, error = 'host_timeout', False, 'host 900-second limit; not simulated timeout or pass'
    except Exception as exc:
        status, accepted, error = 'runner_error', False, str(exc)
    for name in ['events.jsonl', 'compiled.json', 'trajectory.json', 'stdout.json']:
        path = case / name
        if path.exists():
            with path.open('rb') as source, gzip.open(str(path) + '.gz', 'wb') as target:
                shutil.copyfileobj(source, target)
            path.unlink()
    return key, status, int(accepted), time.monotonic()-started, error


def progress(db, out, state):
    counts = dict(db.execute('SELECT status, COUNT(*) FROM jobs GROUP BY status'))
    completed, seconds = db.execute('SELECT COUNT(*), COALESCE(SUM(seconds),0) FROM jobs WHERE seconds IS NOT NULL').fetchone()
    value = {'state': state, 'pid': os.getpid(), 'updated': time.time(), 'counts': counts,
             'completed': completed, 'mean_process_seconds': seconds/completed if completed else None,
             'rough_remaining_wall_hours_at_two_workers': counts.get('pending',0)*seconds/completed/7200 if completed else None}
    temp = out / 'progress.tmp'
    temp.write_bytes(encode(value))
    temp.replace(out / 'progress.json')
    return value


def run(out, limit):
    identity = json.loads((out / 'identity.json').read_bytes())
    if digest(out / 'lidar_sweep') != identity['binary_sha256']:
        raise SystemExit('Frozen binary changed; refusing to reuse results.')
    with (out / 'worker.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        db = connect(out)
        db.execute("UPDATE jobs SET status='pending' WHERE status='running'")
        db.commit()
        done = 0
        paused = False
        pending = {}
        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
            while pending or (limit is None or done < limit):
                if shutil.disk_usage(out).free < 10 * 1024**3:
                    paused = True
                    progress(db, out, 'paused_low_disk')
                    # Drain running subprocesses before leaving resumable queue.
                    for future in list(pending):
                        key,status,accepted,seconds,error = future.result()
                        db.execute('UPDATE jobs SET status=?,accepted=?,seconds=?,error=? WHERE id=?',(status,accepted,seconds,error,key))
                        db.commit()
                    break
                while len(pending) < 2 and (limit is None or done + len(pending) < limit):
                    row = db.execute("SELECT id,family,mode,input FROM jobs WHERE status='pending' ORDER BY rowid LIMIT 1").fetchone()
                    if row is None:
                        break
                    db.execute("UPDATE jobs SET status='running' WHERE id=?", (row[0],))
                    db.commit()
                    pending[pool.submit(execute,out,row)] = row[0]
                progress(db,out,'running')
                if not pending:
                    break
                finished,_ = concurrent.futures.wait(pending, timeout=30, return_when=concurrent.futures.FIRST_COMPLETED)
                for future in finished:
                    pending.pop(future)
                    key,status,accepted,seconds,error = future.result()
                    db.execute('UPDATE jobs SET status=?,accepted=?,seconds=?,error=? WHERE id=?',(status,accepted,seconds,error,key))
                    db.commit()
                    done += 1
            state = 'paused_low_disk' if paused else 'complete' if not db.execute("SELECT 1 FROM jobs WHERE status='pending' LIMIT 1").fetchone() else 'batch_stopped'
            print(json.dumps(progress(db,out,state),ensure_ascii=False,indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['init','run','status'])
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--binary',type=Path,default=ROOT/'target/release/examples/lidar_sweep')
    parser.add_argument('--limit',type=int)
    args = parser.parse_args()
    out = args.out.resolve()
    if args.limit is not None and args.limit <= 0:
        parser.error('limit must be positive')
    if args.action == 'init':
        initialize(out,args.binary.resolve())
    elif args.action == 'run':
        run(out,args.limit)
    else:
        db = connect(out)
        print(json.dumps({'counts':dict(db.execute('SELECT status,COUNT(*) FROM jobs GROUP BY status')), 'progress':json.loads((out/'progress.json').read_bytes()) if (out/'progress.json').exists() else None},ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
