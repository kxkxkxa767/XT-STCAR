#!/usr/bin/env python3
"""Foreground systemd entrypoint; chooses a LAN address, never arms control."""
import ipaddress
import json
import os
from pathlib import Path
import subprocess
import time


def lan_address():
    # Route lookup only: does not send traffic to the lookup destination.
    routes = json.loads(subprocess.check_output(['ip', '-j', '-4', 'route', 'get', '1.1.1.1'], text=True))
    for route in routes:
        value = route.get('prefsrc', route.get('src', ''))
        try:
            address = ipaddress.ip_address(value)
        except ValueError:
            continue
        if address.version == 4 and address.is_private and not address.is_loopback and not address.is_unspecified:
            return str(address)
    raise RuntimeError('No private LAN route available')


def main():
    base = Path.home() / 'xt-stcar-console'
    release = base / '20260917'
    for attempt in range(30):
        try:
            address = lan_address()
            if not all(Path(p).exists() for p in ['/dev/car', '/dev/laser', '/dev/video20']):
                raise RuntimeError('Waiting for vehicle devices')
            break
        except (RuntimeError, OSError, ValueError, subprocess.CalledProcessError):
            if attempt == 29:
                raise
            time.sleep(1)
    os.execv('/usr/bin/python3', ['python3', '-u', str(release/'server.py'),
        '--bridge', str(release/'vehicle-bridge'), '--allow-reverse',
        '--bind', '127.0.0.1', '--lan-bind', address, '--port', '8081',
        '--output', str(base/'recordings'), '--access-file', str(base/'access.json')])


if __name__ == '__main__':
    main()
