#!/usr/bin/env python3
"""Systemd console entry for CPE/wired LAN and loopback; no Bluetooth binding."""
import argparse
import ipaddress
import json
import os
from pathlib import Path
import subprocess
import time


def private_address(value):
    try:
        address = ipaddress.ip_address(value)
    except ValueError:
        return None
    return str(address) if address.version == 4 and address.is_private and not address.is_loopback and not address.is_unspecified else None


def lan_address():
    def lan_interface(name):
        return name.startswith(('wlan', 'wl', 'eth', 'en'))
    route = subprocess.run(['ip', '-j', '-4', 'route', 'get', '1.1.1.1'], capture_output=True, text=True)
    if route.returncode == 0:
        for row in json.loads(route.stdout):
            if lan_interface(row.get('dev', '')) and (address := private_address(row.get('prefsrc', row.get('src', '')))):
                return address
    # CPE may provide only a local network, without an Internet default route.
    interfaces = json.loads(subprocess.check_output(['ip', '-j', '-4', 'address', 'show', 'scope', 'global'], text=True))
    for interface in interfaces:
        if not lan_interface(interface.get('ifname', '')):
            continue
        for row in interface.get('addr_info', []):
            if address := private_address(row.get('local', '')):
                return address
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--show', action='store_true', help='print selected address without opening devices')
    args = parser.parse_args()
    if args.show:
        print(json.dumps({'loopback': '127.0.0.1', 'lan': lan_address(), 'motion_requested': False}))
        return
    base = Path.home()/'xt-stcar-console'
    release = base/'20260917'
    address = None
    for attempt in range(30):
        devices_ready = all(Path(p).exists() for p in ['/dev/car', '/dev/laser', '/dev/video20'])
        try:
            address = lan_address()
        except (OSError, ValueError, subprocess.CalledProcessError):
            address = None
        if devices_ready and (address is not None or attempt == 29):
            break
        if attempt == 29:
            raise RuntimeError('Vehicle devices did not appear')
        time.sleep(1)
    vision = ['--vision-config', str(base/'vision.json')] if (base/'vision.json').is_file() else []
    lan = ['--lan-bind', address] if address else []
    os.execv('/usr/bin/python3', ['python3', '-u', str(release/'server.py'),
        '--bridge', str(release/'vehicle-bridge'), '--allow-reverse',
        '--bind', '127.0.0.1', *lan, '--port', '8081',
        '--output', str(base/'recordings'), '--access-file', str(base/'access.json'), *vision])


if __name__ == '__main__':
    main()
