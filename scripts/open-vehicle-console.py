#!/usr/bin/env python3
"""Open the deployed console over CPE or Bluetooth SSH; never arm or drive."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import socket
import subprocess
import sys
import time
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
REMOTE = (ROOT/'scripts/start-vehicle-console.py').read_text()


def connection_kind(candidate, host):
    try:
        check = subprocess.run(['ssh', '-S', str(candidate), '-O', 'check', host], capture_output=True, timeout=5)
        if check.returncode:
            return None
        result = subprocess.run(['ssh', '-S', str(candidate), '-o', 'BatchMode=yes',
                                 '-o', 'ProxyCommand=false', host, 'printenv SSH_CONNECTION'],
                                capture_output=True, text=True, timeout=8)
    except subprocess.TimeoutExpired:
        return None
    parts = result.stdout.split()
    if result.returncode or len(parts) != 4:
        return None
    return 'bluetooth' if parts[0] in ('127.0.0.1', '::1') else 'cpe'


def run(argv, **options):
    return subprocess.run(argv, check=True, **options)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host', default='bianbu@192.168.0.156', help='vehicle SSH user@host')
    p.add_argument('--link', choices=('auto', 'cpe', 'bluetooth'), default='auto')
    p.add_argument('--port', type=int, help='local port; default CPE 8081 / Bluetooth 8082')
    p.add_argument('--check-only', action='store_true', help='read existing connections only; no startup/tunnel/browser')
    p.add_argument('--no-open', action='store_true', help='prepare service/tunnel but do not open browser')
    args = p.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_.-]+@[A-Za-z0-9_.-]+', args.host) or (args.port is not None and not 1024 <= args.port <= 65535):
        p.error('invalid SSH host or local port')
    if args.link == 'bluetooth' and args.host != 'bianbu@192.168.0.156':
        p.error('Bluetooth uses the paired XT-STCAR SSH identity')
    if sys.platform != 'darwin' and not (args.no_open or args.check_only):
        p.error('double-click launcher supports macOS; use --no-open for connection checks')
    work = ROOT/'work'
    work.mkdir(exist_ok=True)
    selected = None
    own_socket = work / ('vehicle-console-ssh.sock' if args.host == 'bianbu@192.168.0.156' else 'console-' + hashlib.sha256(args.host.encode()).hexdigest()[:12] + '.sock')
    bluetooth_socket = work/'vehicle-web-bluetooth.sock'
    candidates = [work/'vehicle-test-ssh.sock', own_socket, bluetooth_socket] if args.host == 'bianbu@192.168.0.156' else [own_socket]
    for candidate in candidates:
        kind = connection_kind(candidate, args.host)
        if kind and args.link in ('auto', kind):
            selected = candidate
            break
    if selected is None:
        if args.check_only:
            raise RuntimeError('没有可复用的SSH连接；检查模式未尝试登录。')
        kind = 'bluetooth' if args.link == 'bluetooth' else 'cpe'
        selected = bluetooth_socket if kind == 'bluetooth' else own_socket
        print('需要连接车辆。若提示密码，请在此终端输入；不会保存密码。', flush=True)
        if kind == 'bluetooth':
            run([sys.executable, '-B', str(ROOT/'scripts/connect-vehicle-bluetooth.py'),
                 '--socket', str(selected), '--background'])
        else:
            run(['ssh', '-M', '-N', '-f', '-S', str(selected), '-o', 'ControlPersist=600', '-o', 'ConnectTimeout=8', '-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=2', args.host])
    if args.port is None:
        args.port = 8082 if kind == 'bluetooth' else 8081
    print('正在检查车端驾驶台…', flush=True)
    result = run(['ssh', '-S', str(selected), '-o', 'BatchMode=yes', '-o', 'ProxyCommand=false', args.host, 'python3 -'], input='CHECK_ONLY = '+repr(args.check_only)+'\n'+REMOTE, text=True, capture_output=True, timeout=25)
    data = json.loads(result.stdout)
    if not re.fullmatch(r'[A-Za-z0-9_-]{8,128}', data.get('token', '')):
        raise RuntimeError('车端返回了无效的访问令牌。')
    local = 'http://127.0.0.1:'+str(args.port)
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def same_console():
        try:
            request = urllib.request.Request(local+'/api/control-state', headers={'X-Control-Token': data['token']})
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
        # SSH acknowledges the listener before its first forwarded connection is ready.
        deadline = time.monotonic() + 8
        while not same_console():
            if time.monotonic() >= deadline:
                raise RuntimeError('隧道已建立，但8秒内未能核对车端会话；可重新双击入口重试，或检查车端server.log。')
            time.sleep(.2)
    url = local+'/?link='+kind+'#token='+data['token']
    print(('蓝牙' if kind == 'bluetooth' else 'CPE')+'驾驶台与隧道已就绪；未发送解锁或运动指令。', flush=True)
    if kind == 'bluetooth':
        print('照片只保存在车上，拍完后再统一取回。', flush=True)
    if not data['healthy']:
        print('部分传感器未就绪，请查看网页提示。', flush=True)
    if args.check_only or args.no_open:
        return
    run(['open', url])
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
