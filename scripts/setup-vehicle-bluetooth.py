#!/usr/bin/env python3
"""Install the car's native Bluetooth PAN profile and a scoped maintenance command.

Run on the car with sudo python3 -I FILE --apply. No motion or firmware flashing.
No arguments print the changes. BlueZ pairing still requires matching passkeys.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import uuid

PROFILE = Path('/etc/NetworkManager/system-connections/xt-stcar-bluetooth.nmconnection')
WRAPPER = Path('/usr/local/sbin/xt-stcar-bluetooth')
SUDOERS = Path('/etc/sudoers.d/xt-stcar-bluetooth')
PROFILE_ID = 'XT-STCAR Bluetooth'
ADDRESS = '10.77.0.1/24'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    plan = {'profile': str(PROFILE), 'interface': 'btnap0', 'address': ADDRESS,
            'autoconnect': True, 'commands': ['status', 'on', 'off', 'pairing'],
            'sudo_scope': 'bianbu may call only these fixed Bluetooth maintenance actions',
            'wifi_changed': False, 'motion_requested': False}
    print(json.dumps(plan, indent=2), flush=True)
    if not args.apply:
        return
    if os.geteuid() != 0:
        raise SystemExit('Run this installer with sudo; do not send the password in chat.')
    import gi
    gi.require_version('NM', '1.0')
    from gi.repository import NM
    connection = NM.SimpleConnection.new()
    setting = NM.SettingConnection.new()
    setting.props.id = PROFILE_ID
    old_uuid = None
    if PROFILE.exists():
        for line in PROFILE.read_text().splitlines():
            if line.startswith('uuid='):
                old_uuid = line.partition('=')[2]
    profile_uuid = old_uuid or str(uuid.uuid4())
    setting.props.uuid = profile_uuid
    setting.props.type = 'bluetooth'
    setting.props.interface_name = 'btnap0'
    setting.props.autoconnect = True
    connection.add_setting(setting)
    bt = NM.SettingBluetooth.new()
    bt.props.type = 'nap'
    connection.add_setting(bt)
    bridge = NM.SettingBridge.new()
    bridge.props.stp = False
    connection.add_setting(bridge)
    ipv4 = NM.SettingIP4Config.new()
    ipv4.props.method = 'shared'
    ipv4.props.never_default = True
    ipv4.add_address(NM.IPAddress.new(2, ADDRESS.split('/')[0], 24))
    connection.add_setting(ipv4)
    ipv6 = NM.SettingIP6Config.new()
    ipv6.props.method = 'disabled'
    connection.add_setting(ipv6)
    connection.verify()
    keyfile = NM.keyfile_write(connection, NM.KeyfileHandlerFlags.NONE, None, None)
    PROFILE.write_text(keyfile.to_data()[0])
    PROFILE.chmod(0o600)

    # Root-owned, self-contained wrapper; no imports or programs from user-writable paths.
    wrapper = '''#!/usr/bin/python3 -I
import subprocess,sys
if len(sys.argv)!=2 or sys.argv[1] not in ('status','on','off','pairing'):
    raise SystemExit('usage: xt-stcar-bluetooth status|on|off|pairing')
def run(args): subprocess.run(args,check=True,timeout=30)
profile=PROFILE_UUID
action=sys.argv[1]
if action=='on':
    run(['/usr/bin/bluetoothctl','power','on'])
    run(['/usr/bin/nmcli','connection','up','uuid',profile])
elif action=='off':
    run(['/usr/bin/nmcli','connection','down','uuid',profile])
elif action=='pairing':
    run(['/usr/bin/bluetoothctl','power','on'])
    run(['/usr/bin/bluetoothctl','system-alias','XT-STCAR'])
    run(['/usr/bin/busctl','set-property','org.bluez','/org/bluez/hci0','org.bluez.Adapter1','DiscoverableTimeout','u','180'])
    run(['/usr/bin/bluetoothctl','pairable','on'])
    run(['/usr/bin/bluetoothctl','discoverable','on'])
else:
    run(['/usr/bin/bluetoothctl','show'])
    run(['/usr/bin/nmcli','-f','GENERAL,IP4','device','show','btnap0'])
'''.replace('PROFILE_UUID', repr(profile_uuid))
    WRAPPER.write_text(wrapper)
    WRAPPER.chmod(0o755)
    rules = '\n'.join('bianbu ALL=(root) NOPASSWD: '+str(WRAPPER)+' '+action
                      for action in ('status', 'on', 'off', 'pairing'))+'\n'
    candidate = Path(__file__).resolve().parent/'sudoers-candidate'
    candidate.write_text(rules)
    candidate.chmod(0o440)
    subprocess.run(['/usr/sbin/visudo', '-cf', str(candidate)], check=True)
    SUDOERS.write_text(rules)
    SUDOERS.chmod(0o440)
    subprocess.run(['nmcli', 'connection', 'load', str(PROFILE)], check=True)
    subprocess.run([str(WRAPPER), 'on'], check=True)
    subprocess.run([str(WRAPPER), 'pairing'], check=True)
    print(json.dumps({'installed': True, 'profile_uuid': profile_uuid,
                      'pairing_window_seconds': 180, 'motion_requested': False}), flush=True)


if __name__ == '__main__':
    main()
