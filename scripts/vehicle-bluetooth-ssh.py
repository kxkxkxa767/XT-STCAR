#!/usr/bin/env python3
"""Vehicle user service: authenticated Bluetooth RFCOMM carries the existing SSH socket."""
import argparse
import select
import signal
import socket
import threading

import dbus
import dbus.service
from dbus.mainloop.glib import DBusGMainLoop
from gi.repository import GLib

SERVICE_UUID = '78de4e4b-909b-45e1-a987-40c6d40c0b4e'
PROFILE = '/org/xtstcar/ssh'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--peer', required=True, help='paired and trusted Mac Bluetooth address')
    parser.add_argument('--channel', type=int, default=23)
    args = parser.parse_args()
    peer = 'dev_' + args.peer.upper().replace(':', '_')
    DBusGMainLoop(set_as_default=True)
    bus = dbus.SystemBus()
    loop = GLib.MainLoop()
    connections = {}

    def relay(device, radio, tcp):
        totals = [0, 0]
        try:
            sockets = [radio, tcp]
            while True:
                for source in select.select(sockets, [], [])[0]:
                    data = source.recv(65536)
                    if not data:
                        return
                    direction = sockets.index(source)
                    sockets[1-direction].sendall(data)
                    totals[direction] += len(data)
        except OSError as error:
            print(f'Bluetooth SSH closed: {error}', flush=True)
        finally:
            # BlueZ also owns a descriptor for the radio socket. Close the
            # connection itself, so the Mac can open a fresh SSH session later.
            for connection in (radio, tcp):
                try:
                    connection.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                connection.close()
            def forget():
                connections.pop(radio, None)
                return False
            GLib.idle_add(forget)
            print(f'Bluetooth SSH bytes: upload={totals[0]} download={totals[1]}', flush=True)

    class Profile(dbus.service.Object):
        @dbus.service.method('org.bluez.Profile1', in_signature='', out_signature='')
        def Release(self):
            loop.quit()

        @dbus.service.method('org.bluez.Profile1', in_signature='oha{sv}', out_signature='')
        def NewConnection(self, device, descriptor, properties):
            radio = socket.socket(fileno=descriptor.take())
            radio.setblocking(True)  # BlueZ passes a nonblocking fd; kernel backpressure must wait, not close SSH.
            if str(device).rsplit('/', 1)[-1] != peer:
                radio.close()
                raise dbus.exceptions.DBusException('Unexpected peer', name='org.bluez.Error.Rejected')
            try:
                tcp = socket.create_connection(('127.0.0.1', 22), timeout=5)
                tcp.settimeout(None)
                tcp.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            except OSError:
                radio.close()
                raise
            connections[radio] = (device, tcp)
            print('Bluetooth SSH connected to local sshd', flush=True)
            threading.Thread(target=relay, args=(device, radio, tcp), daemon=True).start()

        @dbus.service.method('org.bluez.Profile1', in_signature='o', out_signature='')
        def RequestDisconnection(self, device):
            for radio, (owner, tcp) in list(connections.items()):
                if owner == device:
                    connections.pop(radio, None)
                    for connection in (radio, tcp):
                        try:
                            connection.shutdown(socket.SHUT_RDWR)
                        except OSError:
                            pass
                        connection.close()

    profile = Profile(bus, PROFILE)
    manager = dbus.Interface(bus.get_object('org.bluez', '/org/bluez'), 'org.bluez.ProfileManager1')
    manager.RegisterProfile(PROFILE, SERVICE_UUID, {
        'Name': 'XT-STCAR SSH', 'Role': 'server', 'Channel': dbus.UInt16(args.channel),
        'RequireAuthentication': dbus.Boolean(True), 'RequireAuthorization': dbus.Boolean(True),
        'ServiceRecord': f'''<record>
          <attribute id="0x0001"><sequence><uuid value="{SERVICE_UUID}"/></sequence></attribute>
          <attribute id="0x0004"><sequence>
            <sequence><uuid value="0x0100"/></sequence>
            <sequence><uuid value="0x0003"/><uint8 value="{args.channel}"/></sequence>
          </sequence></attribute>
          <attribute id="0x0005"><sequence><uuid value="0x1002"/></sequence></attribute>
          <attribute id="0x0100"><text value="XT-STCAR SSH"/></attribute>
        </record>''',
    })
    signal.signal(signal.SIGTERM, lambda *_: loop.quit())
    signal.signal(signal.SIGINT, lambda *_: loop.quit())
    print(f'Bluetooth SSH ready on RFCOMM {args.channel}; no vehicle control interface', flush=True)
    try:
        loop.run()
    finally:
        manager.UnregisterProfile(PROFILE)
        for device, _ in list(connections.values()):
            profile.RequestDisconnection(device)


if __name__ == '__main__':
    main()
