#!/usr/bin/env python3
"""Validate staged RTL8852BS files before installing them; run via sudo on vehicle."""
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import tempfile
import time

EXPECTED = {
    'rtk_hciattach': '1a807e8408bd27212c7414b91072591c2168ddcd55cebf338f6afbbf150f65d8',
    'firmware/rtlbt/rtl8852bs_fw': '295488bd7d494e656d4c7941ddd45abde60363f31f90015f411e6cfa409dec74',
    'firmware/rtlbt/rtl8852bs_config': 'efa8915db59c5bc30aaa23e1f264656bbf425fcaf091db0954c27752a1d8f7a0',
}
TARGETS = {
    'rtk_hciattach': Path('/usr/local/bin/rtk_hciattach'),
    'firmware/rtlbt/rtl8852bs_fw': Path('/lib/firmware/rtlbt/rtl8852bs_fw'),
    'firmware/rtlbt/rtl8852bs_config': Path('/lib/firmware/rtlbt/rtl8852bs_config'),
}
UNIT = 'xt-stcar-bt-validation.service'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(args, timeout=20, check=True):
    result = subprocess.run(args, capture_output=True, text=True, timeout=timeout,
                            env={**os.environ, 'LC_ALL': 'C'})
    if check and result.returncode:
        raise RuntimeError(f'{args[0]} failed: {result.stdout} {result.stderr}')
    return result


def reset_bluetooth():
    state = Path('/sys/class/rfkill/rfkill0/state')
    if Path('/sys/class/rfkill/rfkill0/type').read_text().strip() != 'bluetooth':
        raise RuntimeError('rfkill0 identity changed')
    if Path('/sys/class/rfkill/rfkill0/name').read_text().strip() != 'rf-pwrseq:bt-pwrseq':
        raise RuntimeError('Unexpected Bluetooth power control')
    # Match the factory startup sequence. Closing the HCI UART does not reset
    # firmware or the chip's negotiated 1.5-Mbaud link back to boot defaults.
    try:
        state.write_text('0\n')
        time.sleep(1)
    finally:
        state.write_text('1\n')
    time.sleep(1)


def wait_controller(timeout=25):
    until = time.monotonic() + timeout
    while time.monotonic() < until:
        controllers = list(Path('/sys/class/bluetooth').glob('hci*'))
        if controllers:
            return controllers[0].name
        time.sleep(0.5)
    raise RuntimeError('No Bluetooth controller appeared within 25 seconds')


def wait_bluez(timeout=15):
    # Kernel hci0 appears before BlueZ publishes a usable Adapter1/default
    # controller. Wait for the userspace API actually used by power/scan.
    until = time.monotonic() + timeout
    last = 'controller not published'
    while time.monotonic() < until:
        try:
            result = run(['bluetoothctl', 'show'], timeout=3, check=False)
            last = (result.stdout + result.stderr).strip()
            if result.returncode == 0 and re.search(r'Controller [0-9A-Fa-f:]{17}', result.stdout):
                return
        except subprocess.TimeoutExpired:
            last = 'bluetoothctl show timed out'
        time.sleep(0.5)
    raise RuntimeError('BlueZ controller not ready within 15 seconds: ' + last)


def scan(report, label):
    # The factory FIFO may deliver a pending off/on transition after reattach.
    # Retry only the post-install check; every pass still needs live scan events.
    limit = 3 if label == 'system_scan' else 1
    attempts = report.setdefault(label + '_attempts', [])
    for number in range(limit):
        record = {}
        attempts.append(record)
        try:
            wait_bluez()
            report[label + '_bluez_ready'] = True
            power = run(['bluetoothctl', 'power', 'on'], timeout=5)
            if 'succeeded' not in power.stdout:
                raise RuntimeError('Bluetooth power-on did not report success: ' + power.stdout)
            result = run(['bluetoothctl', '--timeout', '10', 'scan', 'on'], timeout=15)
            clean = re.sub(r'\x1b\[[0-?]*[ -/]*[@-~]', '', result.stdout)
            devices = set(re.findall(r'\[(?:NEW|CHG)\] Device ([0-9A-Fa-f:]{17})', clean))
            record.update(scan_started='Discovery started' in clean,
                          observed_device_count=len(devices), exit=result.returncode,
                          stdout=result.stdout, stderr=result.stderr)
            report[label] = {key: record[key] for key in ('scan_started', 'observed_device_count', 'exit')}
            if not record['scan_started'] or not devices:
                raise RuntimeError('Scan did not both start and receive a nearby device advertisement')
            # Ensure the adapter remains present after the scan, not just before.
            wait_bluez(timeout=5)
            print(f'{label}: scan received {len(devices)} nearby devices', flush=True)
            return
        except (RuntimeError, subprocess.TimeoutExpired) as error:
            record['error'] = str(error)
            if number + 1 == limit:
                raise
            print('Controller changed during scan; waiting before bounded retry.', flush=True)
            time.sleep(3)


def main():
    if os.geteuid() != 0:
        raise SystemExit('Run with sudo in your own SSH terminal; no password is stored.')
    base = Path(__file__).resolve().parent
    if os.uname().machine != 'riscv64':
        raise SystemExit('This installer is only for the verified RISC-V vehicle.')
    if b'MUSE-Pi-Pro' not in Path('/proc/device-tree/model').read_bytes():
        raise SystemExit('Unexpected board model')
    if Path('/sys/class/rfkill/rfkill0/type').read_text().strip() != 'bluetooth':
        raise SystemExit('rfkill0 is not Bluetooth; refusing to run fixed vendor helper')
    if list(Path('/sys/class/bluetooth').glob('hci*')):
        raise SystemExit('A controller already exists; inspect it before running repair.')
    if run(['systemctl', 'is-active', '--quiet', 'realtek-bt.service'], check=False).returncode == 0:
        raise SystemExit('Factory attach service is already active')
    if run(['systemctl', 'is-active', '--quiet', UNIT], check=False).returncode == 0:
        raise SystemExit('A validation service is already active')
    for relative, expected in EXPECTED.items():
        if digest(base / relative) != expected:
            raise SystemExit(f'Artifact hash mismatch: {relative}')
        target = TARGETS[relative]
        if target.exists() or target.is_symlink():
            raise SystemExit(f'Refusing to overwrite existing file: {target}')
    if run(['fuser', '/dev/ttyS2'], check=False).returncode == 0:
        raise SystemExit('Bluetooth UART is occupied')
    stage = Path(tempfile.mkdtemp(prefix='xt-stcar-bt-', dir='/var/tmp'))
    stage.chmod(0o700)
    report = {'schema_version': 1, 'system_install_completed': False,
              'source_hashes': EXPECTED, 'temporary_root_stage': str(stage),
              'temporary_service_deadline_seconds': 60}
    installed = []
    factory_started = False
    validation_started = False
    success = False
    for sig in (signal.SIGHUP, signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        for relative, expected in EXPECTED.items():
            destination = stage / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(base / relative, destination)
            destination.chmod(0o755 if relative == 'rtk_hciattach' else 0o644)
            if digest(destination) != expected:
                raise RuntimeError('Root-owned staging copy hash mismatch')
        print('Testing staged files in a private mount namespace; system files unchanged.', flush=True)
        # The exact production binary reads its standard firmware path, but only
        # this temporary service sees the staged directory at that path.
        reset_bluetooth()
        report['temporary_bluetooth_power_reset'] = True
        validation_started = True
        run(['systemd-run', '--quiet', '--collect', '--unit=' + UNIT,
             '--property=RuntimeMaxSec=60', '--property=PrivateMounts=yes',
             '--property=NoNewPrivileges=yes',
             '--property=BindReadOnlyPaths=' + str(stage / 'firmware') + ':' + os.path.realpath('/lib/firmware'),
             '--property=StandardOutput=append:' + str(stage / 'attach.log'),
             '--property=StandardError=append:' + str(stage / 'attach.log'),
             str(stage / 'rtk_hciattach'), '-n', '-t', '20', '-s', '115200', '/dev/ttyS2', 'rtk_h5'])
        report['temporary_controller'] = wait_controller()
        scan(report, 'temporary_scan')
        run(['systemctl', 'stop', UNIT])
        validation_started = False
        for _ in range(20):
            if not list(Path('/sys/class/bluetooth').glob('hci*')):
                break
            time.sleep(0.25)
        else:
            raise RuntimeError('Temporary controller did not detach; installation skipped')
        print('Temporary scan passed. Installing three verified files into system directories.', flush=True)
        for relative, target in TARGETS.items():
            target.parent.mkdir(parents=True, exist_ok=True)
            # Exclusive creation: never replace an existing file or symlink.
            with target.open('xb') as output:
                installed.append((relative, target))
                output.write((stage / relative).read_bytes())
            target.chmod(0o755 if relative == 'rtk_hciattach' else 0o644)
            if digest(target) != EXPECTED[relative]:
                raise RuntimeError('Installed file hash mismatch')
        factory_started = True
        run(['systemctl', 'restart', 'realtek-bt.service'])
        report['system_controller'] = wait_controller()
        scan(report, 'system_scan')
        run(['systemctl', 'is-active', '--quiet', 'realtek-bt.service'])
        report['system_install_completed'] = True
        report['installed_paths'] = [str(path) for _, path in installed]
        success = True
        print('Installed and verified. Factory Bluetooth service remains running.', flush=True)
    except BaseException as error:
        report['error'] = str(error) or type(error).__name__
        print('Repair not completed: ' + report['error'], flush=True)
    finally:
        if validation_started:
            run(['systemctl', 'stop', UNIT], check=False)
        if not success and factory_started:
            run(['systemctl', 'stop', 'realtek-bt.service'], check=False)
        if not success:
            for relative, target in reversed(installed):
                # These files were exclusively created by this invocation.
                if target.is_file() and not target.is_symlink():
                    target.unlink()
            report['system_files_rolled_back'] = bool(installed)
        log = stage / 'attach.log'
        if log.exists():
            shutil.copyfile(log, base / 'temporary-attach.log')
        (base / 'repair-result.json').write_text(json.dumps(report, indent=2) + '\n')
        shutil.rmtree(stage)
    if not success:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
