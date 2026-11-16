#!/usr/bin/env python3
"""Mac: Bluetooth SSH to the car, retaining Wi-Fi and a separate Bluetooth socket.

Run without arguments for the vehicle terminal. --check inspects an existing connection.
"""
import argparse
import json
import os
from pathlib import Path
import plistlib
import shlex
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
SOCKET = ROOT/'work/vehicle-web-bluetooth.sock'
HOST = 'bianbu@192.168.0.156'  # SSH host identity; Bluetooth ProxyCommand supplies the transport.
WORK = ROOT/'work/bluetooth-client'


def build_proxy():
    source = ROOT/'scripts/bluetooth-ssh-proxy.m'
    binary = WORK/'bluetooth-ssh-proxy'
    if binary.exists() and binary.stat().st_mtime >= source.stat().st_mtime:
        return binary
    WORK.mkdir(parents=True, exist_ok=True)
    (WORK/'module-cache').mkdir(exist_ok=True)
    (WORK/'tmp').mkdir(exist_ok=True)
    info = WORK/'Info.plist'
    info.write_bytes(plistlib.dumps({
        'CFBundleIdentifier': 'org.xtstcar.bluetoothssh',
        'CFBundleName': 'XT-STCAR Bluetooth SSH',
        'CFBundleExecutable': binary.name,
        'CFBundlePackageType': 'APPL',
        'NSBluetoothAlwaysUsageDescription': '通过蓝牙连接已配对的 XT-STCAR，承载 SSH 终端和文件传输。',
        'LSUIElement': True,
    }))
    env = dict(os.environ, CLANG_MODULE_CACHE_PATH=str(WORK/'module-cache'), TMPDIR=str(WORK/'tmp'))
    subprocess.run(['xcrun', 'clang', '-fobjc-arc', '-O2', '-framework', 'Foundation',
        '-framework', 'IOBluetooth', '-Wl,-sectcreate,__TEXT,__info_plist,'+str(info),
        str(source), '-o', str(binary)], env=env, check=True)
    subprocess.run(['codesign', '-s', '-', '--force', '--identifier', 'org.xtstcar.bluetoothssh',
                    str(binary)], check=True, capture_output=True)
    return binary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--address', help='vehicle Bluetooth address; normally read from project work configuration')
    parser.add_argument('--check', action='store_true', help='check an existing Bluetooth SSH master; no new connection or motion')
    parser.add_argument('--socket', type=Path, default=SOCKET)
    parser.add_argument('--command', help='optional remote command instead of an interactive shell')
    parser.add_argument('--background', action='store_true', help='open the shared SSH connection without a shell')
    args = parser.parse_args()
    config = WORK/'vehicle.json'
    address = args.address or (json.loads(config.read_text())['address'] if config.exists() else None)
    if not address:
        raise SystemExit('请先用 --address 指定这台车的蓝牙地址。')
    args.socket.parent.mkdir(parents=True, exist_ok=True)
    shared = ['ssh', '-S', str(args.socket)]
    alive = subprocess.run(shared+['-O', 'check', HOST], capture_output=True).returncode == 0
    if args.check and not alive:
        raise SystemExit('尚未建立共享连接，请先运行连接命令。')
    if alive:
        transport = subprocess.run(shared+['-o', 'BatchMode=yes', HOST, 'printenv SSH_CONNECTION'],
                                   capture_output=True, text=True, timeout=10)
        if args.check:
            if not transport.stdout.startswith('127.0.0.1 '):
                raise SystemExit('当前共享连接仍走 CPE，不是蓝牙。')
            print('蓝牙 SSH 共享连接有效。')
            return
        if not transport.stdout.startswith('127.0.0.1 '):
            # Keep existing terminals alive; replace only their multiplex listener.
            subprocess.run(shared+['-O', 'stop', HOST], check=True, capture_output=True)
            alive = False
    if not alive and args.socket.exists():
        args.socket.unlink()
    binary = build_proxy()
    command = shared + ['-o', 'ControlMaster=auto', '-o', 'ControlPersist=yes',
        '-o', 'ServerAliveInterval=15', '-o', 'ServerAliveCountMax=3',
        '-o', 'ProxyCommand='+shlex.join([str(binary), address])]
    if args.background:
        command += ['-fN']
    elif not args.command:
        command += ['-t']
    command += [HOST]
    if args.command:
        command += [args.command]
    print('通过蓝牙连接车辆；Mac Wi-Fi 保持原样。', flush=True)
    os.execvp(command[0], command)


if __name__ == '__main__':
    main()
