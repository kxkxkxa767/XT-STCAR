#!/usr/bin/env python3
"""Single-operator console. Rust owns chassis deadlines. No arm/movement on start."""
import argparse
import base64
import collections
import fcntl
import hashlib
import hmac
import ipaddress
import json
import math
import os
from pathlib import Path
import secrets
import select
import shutil
import signal
import socket
import socketserver
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit
import zipfile

ROOT = Path(__file__).resolve().parent


class Console:
    def __init__(self, args):
        self.args = args
        self.lock = threading.RLock()
        self.send_lock = threading.Lock()
        self.stop = threading.Event()
        self.token = secrets.token_urlsafe(32)
        access = Path(args.access_file)
        if access.exists():
            existing = json.loads(access.read_text()).get('token')
            if not isinstance(existing, str) or not 8 <= len(existing) <= 128 or any(c not in 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_-' for c in existing):
                raise ValueError('invalid stored access token')
            if access.stat().st_mode & 0o077:
                raise ValueError('access file must be private (chmod 600)')
            self.token = existing
        self.boot = secrets.token_urlsafe(16)
        self.owner = None
        self.owner_at = 0
        self.arm_sequence = 0
        self.last_stop = None
        self.stop_latched = True
        self.stopped_tick = -1
        self.sequence = 0
        self.browser_sequence = -1
        self.status = {}
        self.control_at = 0
        self.scan = None
        self.scan_at = 0
        self.jpeg = None
        self.camera_at = 0
        self.camera_seq = 0
        self.vision = None
        if getattr(args, 'vision_shadow', None):
            from vision_shadow import VisionShadow
            self.vision = VisionShadow(args.vision_shadow)
        self.errors = {}
        self.events = collections.deque(maxlen=2000)
        self.record = None
        self.record_error = None
        self.saving = False
        self.storage_lock = threading.Lock()
        self.processes = []
        self.output = Path(args.output).resolve()
        self.output.mkdir(parents=True, exist_ok=True, mode=0o700)
        os.chmod(self.output, 0o700)
        self.settings = {'forward': 1550, 'reverse': 1450, 'left': 1650, 'right': 1350}

    def spawn(self, mode, device):
        cmd = [self.args.bridge, mode]
        if not self.args.demo:
            if os.path.realpath(device) != {'control': '/dev/ttyUSB1', 'lidar': '/dev/ttyACM0'}[mode]:
                raise RuntimeError('device identity changed: ' + device)
            check = subprocess.run(['fuser', device], capture_output=True, timeout=3)
            if check.returncode != 1:
                raise RuntimeError('device busy or owner check failed: ' + device)
            cmd += ['--device', device, '--execute']
        if mode == 'control' and self.args.allow_reverse:
            cmd += ['--allow-reverse']
        proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)
        os.set_blocking(proc.stdin.fileno(), False)
        self.processes.append(proc)
        return proc

    def start(self):
        if self.vision:
            self.vision.start()
        self.control = self.spawn('control', '/dev/car')
        self.lidar = self.spawn('lidar', '/dev/laser')
        threading.Thread(target=self.read_bridge, args=(self.control, 'control'), daemon=True).start()
        threading.Thread(target=self.read_bridge, args=(self.lidar, 'lidar'), daemon=True).start()
        threading.Thread(target=self.camera, daemon=True).start()
        threading.Thread(target=self.recorder, daemon=True).start()
        threading.Thread(target=self.health_watch, daemon=True).start()

    def read_bridge(self, proc, kind):
        try:
            while not self.stop.is_set():
                line = proc.stdout.readline(65537)
                if not line or len(line) > 65536:
                    raise RuntimeError(kind + ' bridge ended')
                value = json.loads(line)
                with self.lock:
                    if kind == 'control':
                        self.status = value
                        self.control_at = time.monotonic()
                        control = value.get('control', {})
                        if self.owner and not control.get('armed') and control.get('seq', -1) >= self.arm_sequence:
                            self.halt('bridge_' + control.get('reason', 'locked'))
                    else:
                        self.scan = value
                        self.scan_at = time.monotonic()
        except Exception as error:
            with self.lock:
                self.errors[kind] = str(error)
            self.halt('bridge_failure')

    def healthy(self):
        now = time.monotonic()
        return (now - self.camera_at < 1 and now - self.scan_at < 1
                and now - self.control_at < .2 and not self.errors)

    def health_watch(self):
        while not self.stop.wait(.05):
            with self.lock:
                unsafe = not self.healthy() or (self.owner is not None and time.monotonic() - self.owner_at > .35)
                armed = self.status.get('control', {}).get('armed', False)
                if unsafe and (armed or self.owner is not None):
                    self.halt('sensor_stale' if not self.healthy() else 'browser_timeout')

    def emit(self, op, motor=1500, servo=1500, tick=0):
        # Nonblocking writes: a blocked/killed bridge must never block the web watchdog.
        with self.send_lock:
            self.sequence += 1
            row = {'op': op, 'seq': self.sequence, 'tick': tick, 'motor': motor, 'servo': servo}
            data = (json.dumps(row) + '\n').encode()
            try:
                if os.write(self.control.stdin.fileno(), data) != len(data):
                    raise RuntimeError('short bridge write')
            except Exception as error:
                self.errors['control_write'] = str(error)
                raise RuntimeError('control bridge unavailable') from error
            self.events.append({'unix_s': time.time(), 'request': row})

    def halt(self, reason):
        with self.lock:
            # Keep the first cause; late requests must not mask a watchdog stop.
            if not self.stop_latched or self.last_stop is None:
                self.last_stop = {'reason': reason, 'unix_s': time.time()}
            self.stop_latched = True
            self.owner = None
            self.stopped_tick = self.status.get('control', {}).get('tick', -1)
            self.events.append({'unix_s': time.time(), 'stop_reason': reason})
            try:
                self.emit('stop')
            except Exception:
                pass

    def command(self, data):
        with self.lock:
            op = data.get('op')
            if op == 'stop':
                self.halt('operator_stop')
                return {'ok': True}
            if self.stop.is_set():
                raise ValueError('server is shutting down')
            if data.get('boot') != self.boot:
                raise ValueError('server restarted; refresh state and unlock again')
            client = data.get('client')
            seq = data.get('seq')
            tick = data.get('tick')
            if not isinstance(client, str) or not 16 <= len(client) <= 80 or type(seq) is not int or type(tick) is not int:
                raise ValueError('invalid client/sequence/tick')
            if self.owner is not None and client != self.owner:
                raise ValueError('another tab owns control')
            if op == 'arm':
                if self.owner is not None or self.status.get('control', {}).get('armed'):
                    raise ValueError('already armed; stop first')
                if not self.healthy():
                    raise ValueError('camera/lidar/control not fresh')
                current_tick = self.status.get('control', {}).get('tick', -1)
                if not 0 <= current_tick - tick < 180 or tick <= self.stopped_tick:
                    raise ValueError('stale arm request')
                self.stop_latched = False
                self.owner = client
                self.browser_sequence = seq
                self.owner_at = time.monotonic()
                self.emit('arm', tick=tick)
                self.arm_sequence = self.sequence
            elif op == 'drive':
                if self.owner != client or seq <= self.browser_sequence:
                    self.halt('stale_or_unowned_request')
                    raise ValueError('stale/unowned request; unlock again')
                if not self.healthy():
                    self.halt('sensor_stale')
                    raise ValueError('sensors stale')
                keys = data.get('keys')
                if not isinstance(keys, list) or len(keys) > 4 or any(k not in ['up', 'down', 'left', 'right'] for k in keys):
                    self.halt('invalid_keys')
                    raise ValueError('invalid keys')
                if 'down' in keys and not self.args.allow_reverse:
                    self.halt('reverse_not_calibrated')
                    raise ValueError('reverse is disabled until calibrated')
                motor = 1500
                servo = 1500
                if ('up' in keys) != ('down' in keys):
                    motor = self.settings['forward' if 'up' in keys else 'reverse']
                if ('left' in keys) != ('right' in keys):
                    servo = self.settings['left' if 'left' in keys else 'right']
                self.browser_sequence = seq
                self.owner_at = time.monotonic()
                self.emit('drive', motor, servo, tick)
            else:
                raise ValueError('unknown control operation')
            return {'ok': True, 'bridge_seq': self.sequence}

    def configure(self, data):
        with self.lock:
            if self.owner or self.status.get('control', {}).get('armed'):
                raise ValueError('stop and lock before editing PWM')
            ranges = {'forward': (1500, 1620), 'reverse': (1350, 1500), 'left': (1500, 1650), 'right': (1350, 1500)}
            if set(data) != set(ranges):
                raise ValueError('four PWM settings required')
            for k, (lo, hi) in ranges.items():
                if type(data[k]) is not int or not lo <= data[k] <= hi:
                    raise ValueError(k + ' is outside server limits')
            self.settings = dict(data)
            return self.settings

    def camera(self):
        cap = None
        try:
            if not self.args.demo:
                check = subprocess.run(['fuser', self.args.camera], capture_output=True, timeout=3)
                if check.returncode != 1:
                    raise RuntimeError('camera busy or owner check failed')
                import cv2
                cap = cv2.VideoCapture(self.args.camera, cv2.CAP_V4L2)
                if not cap.isOpened():
                    raise RuntimeError('camera open failed')
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            encoded = 0
            while not self.stop.is_set():
                if cap is not None:
                    ok, frame = cap.read()
                    if not ok:
                        raise RuntimeError('camera read failed')
                    captured_at = time.monotonic()
                    now = captured_at
                    if now - encoded < .1:
                        continue
                    ok, image = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
                    if not ok:
                        raise RuntimeError('JPEG failed')
                    image = image.tobytes()
                else:
                    image = b'<svg xmlns="http://www.w3.org/2000/svg" width="640" height="480"><rect width="640" height="480" fill="#162d35"/><text x="130" y="250" font-size="32" fill="#76e7c0">DEMO / NO VEHICLE</text></svg>'
                    time.sleep(.1)
                    captured_at = time.monotonic()
                with self.lock:
                    self.jpeg = image
                    self.camera_at = captured_at
                    self.camera_seq += 1
                    sequence = self.camera_seq
                if self.vision and cap is not None:
                    self.vision.submit(image, captured_at, sequence)
                encoded = time.monotonic()
        except Exception as error:
            with self.lock:
                self.errors['camera'] = str(error)
            self.halt('camera_failed')
        finally:
            if cap is not None:
                cap.release()

    def state(self):
        with self.lock:
            now = time.monotonic()
            return {'demo': self.args.demo, 'boot': self.boot, 'status': self.status, 'settings': self.settings,
                    'reverse_enabled': self.args.allow_reverse, 'healthy': self.healthy(),
                    'ages': {'camera': now - self.camera_at, 'lidar': now - self.scan_at, 'control': now - self.control_at},
                    'errors': dict(self.errors), 'recording': self.record is not None, 'saving': self.saving,
                    'record_error': self.record_error, 'owner': self.owner, 'last_stop': self.last_stop,
                    'files': sorted(x.name for x in self.output.iterdir() if x.suffix in ('.zip', '.jpg', '.png', '.svg'))[-30:]}

    def storage_available(self):
        total = sum(x.stat().st_size for x in self.output.rglob('*') if x.is_file())
        return total < 2 * 1024**3 and shutil.disk_usage(self.output).free > 512 * 1024**2

    def start_record(self):
        with self.lock:
            if self.record or self.saving:
                raise ValueError('recording or saving already active')
            if not self.healthy():
                raise ValueError('sensors not ready')
            if not self.storage_available():
                raise ValueError('recording storage full; download and clear old files on vehicle')
            name = time.strftime('%Y%m%d-%H%M%S') + '-' + secrets.token_hex(3)
            folder = self.output / name
            folder.mkdir(mode=0o700)
            self.record = {'folder': folder, 'started': time.monotonic(), 'frames': 0, 'bytes': 0, 'last_camera': -1, 'last_scan': -1}
            self.record_error = None
            return {'name': name}

    def finish_record(self):
        with self.lock:
            rec = self.record
            if not rec:
                return {'ok': True}
            self.record = None
            self.saving = True
            saved_events = list(self.events)
            saved_settings = dict(self.settings)
        try:
            with self.storage_lock:
                folder = rec['folder']
                (folder / 'control-events.json').write_text(json.dumps(saved_events, ensure_ascii=False))
                (folder / 'metadata.json').write_text(json.dumps({'created_unix_s': time.time(), 'duration_s': time.monotonic() - rec['started'], 'frames': rec['frames'], 'camera_format': 'SVG demo' if self.args.demo else 'JPEG sequence at up to 10 fps', 'lidar': 'raw visualization bins, NOT navigation-certified', 'clock': 'host receive monotonic; not hardware synchronized', 'settings': saved_settings, 'record_error': self.record_error}, ensure_ascii=False, indent=2))
                temporary = self.output / (folder.name + '.zip.part')
                with zipfile.ZipFile(temporary, 'w', compression=zipfile.ZIP_STORED) as z:
                    for file in folder.iterdir():
                        z.write(file, file.name)
                final = temporary.with_suffix('')
                temporary.rename(final)
                shutil.rmtree(folder)
            return {'file': final.name}
        except Exception as error:
            self.record_error = str(error)
            raise
        finally:
            self.saving = False

    def recorder(self):
        while not self.stop.wait(.1):
            finish = False
            try:
                with self.storage_lock:
                    with self.lock:
                        rec = self.record
                        if rec is None:
                            continue
                        image, seq, scan = self.jpeg, self.camera_seq, self.scan
                        camera_at, scan_at = self.camera_at, self.scan_at
                    now = time.monotonic()
                    if image and seq != rec['last_camera'] and now - camera_at < 1:
                        ext = 'svg' if self.args.demo else 'jpg'
                        (rec['folder'] / f'camera-{rec["frames"]:06d}.{ext}').write_bytes(image)
                        with (rec['folder'] / 'frames.jsonl').open('a') as f:
                            f.write(json.dumps({'frame': rec['frames'], 'seq': seq, 'received_monotonic_s': camera_at}) + '\n')
                        rec['frames'] += 1
                        rec['bytes'] += len(image)
                        rec['last_camera'] = seq
                    if scan and scan['seq'] != rec['last_scan'] and now - scan_at < 1:
                        row = json.dumps({'received_monotonic_s': scan_at, **scan}) + '\n'
                        with (rec['folder'] / 'lidar.jsonl').open('a') as f:
                            f.write(row)
                        rec['bytes'] += len(row)
                        rec['last_scan'] = scan['seq']
                    finish = now - rec['started'] >= 60 or rec['bytes'] >= 250 * 1024**2
                    if rec['frames'] % 20 == 0 and not self.storage_available():
                        self.record_error = 'storage quota / free space limit'
                        finish = True
            except Exception as error:
                self.record_error = str(error)
                finish = True
            if finish:
                try:
                    self.finish_record()
                except Exception:
                    pass

    def snapshot(self):
        with self.lock:
            if not self.healthy():
                raise ValueError('no fresh sensor data')
            image, scan, status = self.jpeg, dict(self.scan), dict(self.status)
        name = time.strftime('snapshot-%Y%m%d-%H%M%S-') + secrets.token_hex(3) + '.zip'
        with self.storage_lock:
            if not self.storage_available():
                raise ValueError('storage full')
            temporary = self.output / (name + '.part')
            with zipfile.ZipFile(temporary, 'w') as z:
                z.writestr('camera.svg' if self.args.demo else 'camera.jpg', image)
                z.writestr('lidar.json', json.dumps(scan))
                z.writestr('status.json', json.dumps(status))
                dots = []
                for angle, distance in enumerate(scan['ranges']):
                    if distance is not None:
                        # Display forward up, left left; N10 factory y=-r*sin(angle).
                        x = 320 + math.sin(math.radians(angle)) * distance * 24
                        y = 320 - math.cos(math.radians(angle)) * distance * 24
                        dots.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="2" fill="#68e6be"/>')
                z.writestr('lidar.svg', '<svg xmlns="http://www.w3.org/2000/svg" width="640" height="640"><rect width="640" height="640" fill="#10252d"/>' + ''.join(dots) + '</svg>')
            temporary.rename(self.output / name)
        return {'file': name}

    def photo(self, kind):
        if kind == 'vision':
            state = self.vision.snapshot() if self.vision else None
            result = state.get('result') if state else None
            if not result or state.get('error') or result['age_ms'] >= 1500:
                raise ValueError('vision result unavailable or stale')
            payload = base64.b64decode(result['jpeg_base64'], validate=True)
            return self.write_photo('vision-' + str(result['sequence']), 'jpg', payload)
        if kind not in ('camera', 'lidar', 'combined'):
            raise ValueError('unknown photo type')
        with self.lock:
            now = time.monotonic()
            if kind != 'lidar' and (self.jpeg is None or now - self.camera_at >= 1):
                raise ValueError('camera frame stale')
            if kind != 'camera' and (self.scan is None or now - self.scan_at >= 1):
                raise ValueError('lidar frame stale')
            image, scan = self.jpeg, self.scan
        ext = 'jpg' if kind == 'camera' else 'png'
        if self.args.demo:
            if kind != 'camera':
                raise ValueError('PNG rendering requires vehicle OpenCV; demo camera SVG is available')
            ext = 'svg'
            payload = image
        elif kind == 'camera':
            payload = image
        else:
            import cv2
            import numpy as np
            radar = np.full((480, 640, 3), (36, 28, 13), dtype=np.uint8)
            for radius in range(2, 13, 2):
                cv2.circle(radar, (320, 255), radius * 18, (77, 65, 41), 1)
                cv2.putText(radar, str(radius) + 'm', (325, 255-radius*18+14), cv2.FONT_HERSHEY_SIMPLEX, .4, (158, 146, 120), 1)
            cv2.line(radar, (320, 20), (320, 465), (77, 65, 41), 1)
            cv2.line(radar, (30, 255), (610, 255), (77, 65, 41), 1)
            for angle, distance in enumerate(scan['ranges']):
                if distance is not None:
                    x = round(320 + math.sin(math.radians(angle))*distance*18)
                    y = round(255 - math.cos(math.radians(angle))*distance*18)
                    cv2.circle(radar, (x, y), 2, (188, 226, 114), -1)
            cv2.putText(radar, 'FRONT', (296, 18), cv2.FONT_HERSHEY_SIMPLEX, .45, (188, 226, 114), 1)
            cv2.circle(radar, (320, 255), 5, (188, 226, 114), -1)
            result = radar
            if kind == 'combined':
                camera = cv2.imdecode(np.frombuffer(image, dtype=np.uint8), cv2.IMREAD_COLOR)
                if camera is None:
                    raise ValueError('camera JPEG decode failed')
                result = np.vstack((np.full((40, 1280, 3), (36, 28, 13), dtype=np.uint8), np.hstack((cv2.resize(camera, (640,480)), radar))))
                cv2.putText(result, 'XT-STCAR | CAMERA + LIDAR | '+time.strftime('%Y-%m-%d %H:%M:%S'), (18,27), cv2.FONT_HERSHEY_SIMPLEX, .55, (230,240,240), 1)
            ok, encoded = cv2.imencode('.png', result)
            if not ok:
                raise ValueError('PNG encoding failed')
            payload = encoded.tobytes()
        return self.write_photo(kind, ext, payload)

    def write_photo(self, kind, ext, payload):
        name = kind+'-'+time.strftime('%Y%m%d-%H%M%S')+'-'+secrets.token_hex(3)+'.'+ext
        with self.storage_lock:
            if not self.storage_available():
                raise ValueError('storage full')
            temporary = self.output/(name+'.part')
            temporary.write_bytes(payload)
            temporary.rename(self.output/name)
        return {'file': name}

    def close(self):
        self.stop.set()
        self.halt('server_shutdown')
        if self.vision:
            self.vision.close()
        if hasattr(self, 'control'):
            self.control.stdin.close()  # Bridge independently observes EOF and sends neutral frames.
            try:
                self.control.wait(timeout=3)
            except subprocess.TimeoutExpired:
                pass  # Never kill the neutral-sending owner as an automatic fallback.
        if hasattr(self, 'lidar'):
            self.lidar.terminate()
        try:
            self.finish_record()
        except Exception:
            pass


def serve(args):
    app = Console(args)
    limiter = threading.BoundedSemaphore(12)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass  # URLs may contain the access token; never log them.

        def setup(self):
            super().setup()
            self.connection.settimeout(3)
            self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        def reply(self, status, body, content='application/json'):
            if not isinstance(body, bytes):
                body = json.dumps(body, ensure_ascii=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', content)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Referrer-Policy', 'no-referrer')
            self.send_header('Content-Security-Policy', "default-src 'self'; img-src 'self' blob:; style-src 'self'; script-src 'self'; frame-ancestors 'none'; base-uri 'none'")
            self.end_headers()
            self.wfile.write(body)

        def authorized(self):
            supplied = self.headers.get('X-Control-Token', '') or parse_qs(urlsplit(self.path).query).get('token', [''])[0]
            return hmac.compare_digest(supplied, app.token)

        def do_GET(self):
            if not limiter.acquire(False):
                self.reply(503, {'error': 'busy'})
                return
            try:
                path = urlsplit(self.path).path
                if path in ['/', '/app.js', '/style.css']:
                    file = ROOT / {'/': 'index.html', '/app.js': 'app.js', '/style.css': 'style.css'}[path]
                    kind = {'/': 'text/html; charset=utf-8', '/app.js': 'text/javascript', '/style.css': 'text/css'}[path]
                    self.reply(200, file.read_bytes(), kind)
                    return
                if not self.authorized():
                    self.reply(403, {'error': 'access token required'})
                    return
                if path == '/api/state':
                    self.reply(200, app.state())
                elif path == '/api/vision':
                    self.reply(200, app.vision.snapshot() if app.vision else {'enabled': False})
                elif path == '/api/lidar':
                    with app.lock:
                        snapshot = {'scan': app.scan, 'age_s': time.monotonic() - app.scan_at}
                    self.reply(200, snapshot)
                elif path == '/camera.jpg':
                    with app.lock:
                        image, age = app.jpeg, time.monotonic() - app.camera_at
                    if image is None or age > 1:
                        self.reply(503, {'error': 'camera stale'})
                    else:
                        self.reply(200, image, 'image/svg+xml' if args.demo else 'image/jpeg')
                elif path.startswith('/download/'):
                    name = path[len('/download/'):]
                    if '/' in name or '\\' in name or Path(name).suffix not in ('.zip', '.jpg', '.png', '.svg') or name.startswith('.'):
                        self.reply(404, {'error': 'not found'})
                        return
                    file = app.output / name
                    if not file.is_file():
                        self.reply(404, {'error': 'not found'})
                        return
                    self.send_response(200)
                    self.send_header('Content-Type', {'.zip':'application/zip', '.jpg':'image/jpeg', '.png':'image/png', '.svg':'image/svg+xml'}[file.suffix])
                    self.send_header('Content-Length', str(file.stat().st_size))
                    self.send_header('Content-Disposition', 'attachment; filename="' + name + '"')
                    self.send_header('Cache-Control', 'no-store')
                    self.end_headers()
                    with file.open('rb') as f:
                        shutil.copyfileobj(f, self.wfile, 65536)
                else:
                    self.reply(404, {'error': 'not found'})
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                pass
            finally:
                limiter.release()

        def do_POST(self):
            if not self.authorized():
                self.reply(403, {'error': 'access token required'})
                return
            origin = self.headers.get('Origin')
            if origin and origin != 'http://' + self.headers.get('Host', ''):
                self.reply(403, {'error': 'cross-origin request rejected'})
                return
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 2048 or self.headers.get('Content-Type') != 'application/json':
                    raise ValueError('bounded JSON body required')
                data = json.loads(self.rfile.read(size))
                if not isinstance(data, dict):
                    raise ValueError('JSON object required')
                path = urlsplit(self.path).path
                if path == '/api/control': result = app.command(data)
                elif path == '/api/settings': result = app.configure(data)
                elif path == '/api/snapshot': result = app.snapshot()
                elif path == '/api/photo': result = app.photo(data.get('kind'))
                elif path == '/api/record/start': result = app.start_record()
                elif path == '/api/record/stop': result = app.finish_record()
                else: raise ValueError('unknown action')
                self.reply(200, result)
            except (ValueError, RuntimeError, OSError) as error:
                self.reply(400, {'error': str(error)})

    class Server(ThreadingHTTPServer):
        daemon_threads = True
        request_queue_size = 8
        def server_bind(self):
            # Avoid reverse DNS lookup delaying startup on offline vehicle networks.
            socketserver.TCPServer.server_bind(self)
            self.server_name = self.server_address[0]
            self.server_port = self.server_address[1]
        def process_request(self, request, client_address):
            # Bound thread creation, not just route work. Reserve capacity for stop.
            if not self.slots.acquire(False):
                request.close()
                return
            try:
                super().process_request(request, client_address)
            except Exception:
                self.slots.release()
                raise
        def process_request_thread(self, request, client_address):
            try:
                super().process_request_thread(request, client_address)
            finally:
                self.slots.release()

    servers = []
    try:
        for address in dict.fromkeys([args.bind] + ([args.lan_bind] if args.lan_bind else [])):
            server = Server((address, args.port), Handler)
            server.slots = threading.BoundedSemaphore(20)
            server.timeout = .2
            servers.append(server)
    except Exception:
        for server in servers:
            server.server_close()
        raise
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *_: app.stop.set())
    access = Path(args.access_file)
    access.parent.mkdir(parents=True, exist_ok=True)
    fd = os.open(access, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    os.fchmod(fd, 0o600)
    with os.fdopen(fd, 'w') as f:
        json.dump({'token': app.token, 'port': args.port, 'pid': os.getpid()}, f)
    try:
        app.start()
        print(f'Console listening on {args.bind}:{args.port}; credentials in {access}', flush=True)
        while not app.stop.is_set():
            for ready in select.select(servers, [], [], .2)[0]:
                ready.handle_request()
    finally:
        app.close()
        for server in servers:
            server.server_close()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bridge', required=True)
    p.add_argument('--bind', default='127.0.0.1')
    p.add_argument('--lan-bind', help='Additional explicit private IPv4 address for LAN browsers')
    p.add_argument('--port', type=int, default=8081)
    p.add_argument('--camera', default='/dev/video20')
    p.add_argument('--output', default='./recordings')
    p.add_argument('--access-file', default='./access.json')
    p.add_argument('--demo', action='store_true')
    vision_group = p.add_mutually_exclusive_group()
    vision_group.add_argument('--vision-config', type=Path, help='optional persistent shadow configuration JSON')
    vision_group.add_argument('--vision-shadow', nargs=5, metavar=('BIN', 'ROAD_CONFIG', 'MODEL', 'ORT_LIB', 'MODEL_SPEC'), help='optional perception-only process; reuses camera frames, never drives')
    p.add_argument('--allow-reverse', action='store_true', help='Only after supervised ESC reverse calibration')
    args = p.parse_args()
    if args.vision_config:
        from vision_shadow import load_config
        try:
            args.vision_shadow = load_config(args.vision_config)
        except (OSError, ValueError) as error:
            p.error(str(error))
    if args.demo and args.vision_shadow:
        p.error('demo SVG camera cannot feed JPEG perception; use recorded JPEG IPC for offline tests')
    for bind in [args.bind] + ([args.lan_bind] if args.lan_bind else []):
        address = ipaddress.ip_address(bind)
        if address.version != 4 or address.is_unspecified or address.is_multicast or not (address.is_private or address.is_loopback):
            p.error('explicit private/loopback IPv4 bind required')
    lock = open('/tmp/xt-stcar-console-demo-' + str(args.port) + '.lock' if args.demo else '/tmp/xt-stcar-console.lock', 'a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    serve(args)


if __name__ == '__main__':
    main()
