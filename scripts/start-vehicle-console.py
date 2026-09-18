#!/usr/bin/env python3
"""Run on the vehicle: reuse or start console, never arm or drive."""
import json, os, re, subprocess, sys, time, urllib.error, urllib.request
CHECK_ONLY = globals().get("CHECK_ONLY", "--check-only" in sys.argv)
from pathlib import Path
base = Path.home() / 'xt-stcar-console'
release = base / '20260917'
access = base / 'access.json'
opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
def read_state():
    data = json.loads(access.read_text())
    if access.stat().st_mode & 0o077:
        raise RuntimeError('access.json permissions must be 600')
    if data.get('port') != 8081 or not re.fullmatch(r'[A-Za-z0-9_-]{8,128}', data.get('token', '')):
        raise RuntimeError('invalid console credentials')
    req = urllib.request.Request('http://127.0.0.1:8081/api/state', headers={'X-Control-Token': data['token']})
    state = json.loads(opener.open(req, timeout=2).read())
    return {'token': data['token'], 'boot': state['boot'], 'healthy': state['healthy'], 'armed': state['status']['control']['armed']}
try:
    result = read_state()
except urllib.error.HTTPError:
    raise RuntimeError('8081 authentication failed; refusing to replace existing service')
except (FileNotFoundError, urllib.error.URLError):
    if CHECK_ONLY:
        raise RuntimeError('console unavailable; check-only did not start anything')
    unit = Path.home()/'.config/systemd/user/xt-stcar-console.service'
    managed = unit.is_file()
    if managed:
        subprocess.run(['systemctl', '--user', 'start', 'xt-stcar-console.service'], check=True, timeout=15)
    elif access.exists():
        old = json.loads(access.read_text())
        pid = old.get('pid', -1)
        command = Path('/proc/' + str(pid) + '/cmdline')
        if command.exists() and b'xt-stcar-console' in command.read_bytes():
            raise RuntimeError('recorded console process still exists; inspect it before restarting')
    for name in ['server.py', 'vehicle-bridge', 'index.html', 'app.js', 'style.css']:
        if not (release/name).is_file():
            raise RuntimeError('console not deployed: ' + str(release/name))
    if not os.access(release/'vehicle-bridge', os.X_OK):
        raise RuntimeError('vehicle-bridge is not executable')
    base.mkdir(mode=0o700, exist_ok=True)
    proc = None
    if not managed:
        with (base/'server.log').open('ab') as log:
            proc = subprocess.Popen(['python3', '-u', str(release/'server.py'), '--bridge', str(release/'vehicle-bridge'), '--allow-reverse', '--bind', '127.0.0.1', '--lan-bind', os.environ['SSH_CONNECTION'].split()[2], '--port', '8081', '--output', str(base/'recordings'), '--access-file', str(access)], stdin=subprocess.DEVNULL, stdout=log, stderr=log, start_new_session=True)
    for _ in range(40):
        if proc is not None and proc.poll() is not None:
            raise RuntimeError('console exited; inspect ~/xt-stcar-console/server.log')
        time.sleep(.25)
        try:
            result = read_state()
            break
        except (FileNotFoundError, urllib.error.URLError):
            pass
    else:
        raise RuntimeError('console did not become reachable; inspect server.log')
if '--url' in sys.argv:
    host = os.environ.get('SSH_CONNECTION', '').split()
    address = host[2] if len(host) == 4 else '192.168.0.156'
    print('驾驶台服务可访问；没有发送解锁或行驶指令。')
    print('http://' + address + ':8081/#token=' + result['token'])
else:
    print(json.dumps(result))
