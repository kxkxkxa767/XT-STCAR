#!/usr/bin/env python3
"""Open the deployed vehicle console via SSH; never arm or send drive commands."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import socket
import subprocess
import sys
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
REMOTE = r'''
import json, os, re, subprocess, time, urllib.error, urllib.request
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
    if access.exists():
        old = json.loads(access.read_text())
        if Path('/proc/' + str(old.get('pid', -1))).exists():
            raise RuntimeError('recorded process still exists; inspect it before restarting')
    for name in ['server.py', 'vehicle-bridge', 'index.html', 'app.js', 'style.css']:
        if not (release/name).is_file():
            raise RuntimeError('console not deployed: ' + str(release/name))
    if not os.access(release/'vehicle-bridge', os.X_OK):
        raise RuntimeError('vehicle-bridge is not executable')
    base.mkdir(mode=0o700, exist_ok=True)
    with (base/'server.log').open('ab') as log:
        proc = subprocess.Popen(['python3', '-u', str(release/'server.py'), '--bridge', str(release/'vehicle-bridge'), '--bind', '127.0.0.1', '--lan-bind', os.environ['SSH_CONNECTION'].split()[2], '--port', '8081', '--output', str(base/'recordings'), '--access-file', str(access)], stdin=subprocess.DEVNULL, stdout=log, stderr=log, start_new_session=True)
    for _ in range(40):
        if proc.poll() is not None:
            raise RuntimeError('console exited; inspect ~/xt-stcar-console/server.log')
        time.sleep(.25)
        try:
            result = read_state()
            break
        except (FileNotFoundError, urllib.error.URLError):
            pass
    else:
        raise RuntimeError('console did not become reachable; inspect server.log')
print(json.dumps(result))
'''


def run(argv, **options):
    return subprocess.run(argv, check=True, **options)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host', default='bianbu@192.168.0.156', help='vehicle SSH user@host')
    p.add_argument('--port', type=int, default=8081, help='local port, vehicle port remains 8081')
    p.add_argument('--check-only', action='store_true', help='read existing connections only; no startup/tunnel/browser')
    p.add_argument('--no-open', action='store_true', help='prepare service/tunnel but do not open browser')
    args = p.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_.-]+@[A-Za-z0-9_.-]+', args.host) or not 1024 <= args.port <= 65535:
        p.error('invalid SSH host or local port')
    if sys.platform != 'darwin' and not (args.no_open or args.check_only):
        p.error('double-click launcher supports macOS; use --no-open for connection checks')
    work = ROOT/'work'
    work.mkdir(exist_ok=True)
    selected = None
    own_socket = work / ('vehicle-console-ssh.sock' if args.host == 'bianbu@192.168.0.156' else 'console-' + hashlib.sha256(args.host.encode()).hexdigest()[:12] + '.sock')
    candidates = [work/'vehicle-test-ssh.sock', own_socket] if args.host == 'bianbu@192.168.0.156' else [own_socket]
    for candidate in candidates:
        check = subprocess.run(['ssh', '-S', str(candidate), '-O', 'check', args.host], capture_output=True, timeout=5)
        if check.returncode == 0:
            selected = candidate
            break
    if selected is None:
        if args.check_only:
            raise RuntimeError('没有可复用的SSH连接；检查模式未尝试登录。')
        selected = own_socket
        print('需要连接车辆。若提示密码，请在此终端输入；不会保存密码。', flush=True)
        run(['ssh', '-M', '-N', '-f', '-S', str(selected), '-o', 'ControlPersist=600', '-o', 'ConnectTimeout=8', '-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=2', args.host])
    print('正在检查车端驾驶台…', flush=True)
    result = run(['ssh', '-S', str(selected), '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', args.host, 'python3 -'], input='CHECK_ONLY = '+repr(args.check_only)+'\n'+REMOTE, text=True, capture_output=True, timeout=25)
    data = json.loads(result.stdout)
    if not re.fullmatch(r'[A-Za-z0-9_-]{8,128}', data.get('token', '')):
        raise RuntimeError('车端返回了无效的访问令牌。')
    local = 'http://127.0.0.1:'+str(args.port)
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def same_console():
        try:
            request = urllib.request.Request(local+'/api/state', headers={'X-Control-Token': data['token']})
            with opener.open(request, timeout=2) as response:
                return json.loads(response.read())['boot'] == data['boot']
        except Exception:
            return False

    if not same_console():
        if args.check_only:
            raise RuntimeError('本机隧道尚未连到该驾驶台；检查模式未创建转发。')
        with socket.socket() as check:
            if check.connect_ex(('127.0.0.1', args.port)) == 0:
                raise RuntimeError('本机端口已被其他服务或旧隧道占用；可用 --port 8082 选择其他本机端口。')
        run(['ssh', '-S', str(selected), '-O', 'forward', '-L', '127.0.0.1:'+str(args.port)+':127.0.0.1:8081', args.host], capture_output=True, text=True, timeout=8)
        if not same_console():
            raise RuntimeError('隧道建立后未能核对车端会话，未打开网页。')
    print('驾驶台与隧道已就绪；未发送解锁或运动指令。', flush=True)
    if not data['healthy']:
        print('部分传感器未就绪，请查看网页提示。', flush=True)
    if args.check_only or args.no_open:
        return
    run(['open', local+'/#token='+data['token']])
    print('已在默认浏览器打开驾驶台。此终端窗口可以关闭。', flush=True)


if __name__ == '__main__':
    try:
        main()
    except subprocess.CalledProcessError as error:
        # SSH diagnostics are useful; never echo successful stdout containing credentials.
        print('连接失败：'+(error.stderr or 'SSH或浏览器命令失败'), file=sys.stderr)
        sys.exit(1)
    except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as error:
        print('打开失败：'+str(error), file=sys.stderr)
        sys.exit(1)
