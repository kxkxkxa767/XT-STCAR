#!/usr/bin/env python3
"""Vehicle-local timed video capture from the console's existing camera.

Run as a user systemd service so neither SSH logout nor a CPE disconnect owns
its lifetime. No arm, drive, stop, camera reconfiguration or image downloads.
"""
import argparse
import http.client
import json
from pathlib import Path
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=600)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    base = Path.home()/'xt-stcar-console'
    access = json.loads((base/'access.json').read_text())
    args.out.mkdir(parents=True, exist_ok=False)
    video = args.out/'camera.mkv'
    state = {'duration_requested_s': args.seconds, 'frames': 0, 'read_errors': 0,
             'source': 'vehicle-local console camera JPEG', 'fps': 10,
             'ssh_independent': True, 'video': str(video), 'status': 'starting'}
    def save():
        temporary = args.out/'status.tmp'
        temporary.write_text(json.dumps(state, indent=2)+'\n')
        temporary.replace(args.out/'status.json')
    save()
    connection = http.client.HTTPConnection('127.0.0.1', access['port'], timeout=3)
    headers = {'X-Control-Token': access['token']}
    with (args.out/'ffmpeg.log').open('wb') as log, (args.out/'frames.jsonl').open('w') as frames:
        process = subprocess.Popen(['ffmpeg', '-hide_banner', '-nostdin', '-loglevel', 'warning',
            '-f', 'mjpeg', '-framerate', '10', '-i', 'pipe:0', '-map', '0:v:0',
            '-c:v', 'copy', '-f', 'matroska', str(video)], stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL, stderr=log)
        started = None
        try:
            while started is None or time.monotonic()-started < args.seconds:
                tick = time.monotonic()
                try:
                    connection.request('GET', '/camera.jpg', headers=headers)
                    response = connection.getresponse()
                    jpeg = response.read()
                    if response.status != 200:
                        raise RuntimeError('camera response '+str(response.status))
                except (OSError, http.client.HTTPException, RuntimeError) as error:
                    state['read_errors'] += 1
                    state['last_error'] = str(error)
                    connection.close()
                    if started is None:
                        raise
                    save()
                    time.sleep(.1)
                    continue
                if started is None:
                    started = tick
                    state.update(status='recording', started_unix_s=time.time(), started_monotonic_s=tick)
                process.stdin.write(jpeg)
                process.stdin.flush()
                state['frames'] += 1
                state['elapsed_s'] = round(time.monotonic()-started, 3)
                frames.write(json.dumps({'frame': state['frames'], 'captured_monotonic_s': tick})+'\n')
                if state['frames'] % 10 == 0 or state['frames'] == 1:
                    frames.flush()
                    save()
                time.sleep(max(0, .1-(time.monotonic()-tick)))
        finally:
            connection.close()
            process.stdin.close()
            code = process.wait(timeout=20)
            state.update(status='completed' if code == 0 and started is not None else 'failed',
                         ffmpeg_exit=code, ended_unix_s=time.time(), video_bytes=video.stat().st_size if video.exists() else 0)
            save()


if __name__ == '__main__':
    main()
