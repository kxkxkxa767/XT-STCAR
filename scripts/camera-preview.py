#!/usr/bin/env python3
"""Local-network camera preview only. No chassis, ROS, or motion commands."""
import argparse
import fcntl
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import ipaddress
import json
import signal
import subprocess
import threading
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', default='/dev/video20')
    parser.add_argument('--bind', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--fps', type=int, default=5, help='JPEG preview limit, not camera capture FPS')
    args = parser.parse_args()
    address = ipaddress.ip_address(args.bind)
    if address.version != 4 or not (address.is_private or address.is_loopback) or address.is_unspecified:
        parser.error('--bind must be an explicit private/loopback IPv4 address')
    if not (1 <= args.port <= 65535 and 1 <= args.fps <= 15):
        parser.error('invalid port or preview FPS (1..15)')
    if (args.width, args.height) not in [(640, 480), (1280, 720), (1920, 1080)]:
        parser.error('unsupported preview dimensions')
    if not args.device.startswith('/dev/') or '..' in args.device.split('/'):
        parser.error('device must be an explicit /dev path')
    # Cooperative process lock plus existing device-owner check. A separate,
    # non-cooperating camera client can still contend; stop preview before tests.
    lock = open('/tmp/xt-stcar-camera-preview.lock', 'a', encoding='utf-8')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        parser.error('another camera preview process holds the lock')
    owner = subprocess.run(['fuser', args.device], capture_output=True, timeout=3)
    if owner.returncode == 0:
        parser.error('camera is already owned by another process')
    import cv2
    stop = threading.Event()
    ready = threading.Event()
    condition = threading.Condition()
    state = {'jpeg': None, 'seq': 0, 'at': 0.0, 'shape': None, 'error': None}
    clients = threading.BoundedSemaphore(2)

    def capture():
        cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
        try:
            if not cap.isOpened():
                raise RuntimeError('camera open failed')
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
            cap.set(cv2.CAP_PROP_FPS, 30)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            encoded_at = 0.0
            while not stop.is_set():
                ok, frame = cap.read()
                if not ok:
                    raise RuntimeError('camera read failed')
                now = time.monotonic()
                if now - encoded_at < 1.0 / args.fps:
                    continue
                ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if not ok or jpeg.nbytes > 4 * 1024 * 1024:
                    raise RuntimeError('JPEG encoding failed or frame too large')
                with condition:
                    state.update(jpeg=jpeg.tobytes(), seq=state['seq'] + 1,
                                 at=now, shape=list(frame.shape))
                    condition.notify_all()
                encoded_at = now
                ready.set()
        except Exception as error:
            with condition:
                state['error'] = str(error)
                condition.notify_all()
            ready.set()
            stop.set()
        finally:
            cap.release()

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.0'

        def setup(self):
            super().setup()
            self.connection.settimeout(5)

        def reply(self, code, kind, payload):
            self.send_response(code)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(payload)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(payload)

        def do_GET(self):
            try:
                if self.path == '/':
                    html = '''<!doctype html><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>XT-STCAR Camera</title><style>body{background:#111827;color:#e5e7eb;font:18px system-ui;margin:24px}img{max-width:100%;height:auto;border-radius:8px}a{color:#7dd3fc}</style><h1>XT-STCAR 相机预览</h1><p>仅图像预览 · 不控制车辆</p><img src="/stream.mjpg" alt="相机画面"><p><a href="/snapshot.jpg">查看当前单帧</a> · <a href="/health">状态</a></p>'''
                    self.reply(200, 'text/html; charset=utf-8', html.encode())
                elif self.path == '/health':
                    with condition:
                        data = {'device': args.device, 'shape': state['shape'],
                                'frames_encoded': state['seq'],
                                'frame_age_s': time.monotonic() - state['at'],
                                'error': state['error'], 'motion_control': False}
                    self.reply(200, 'application/json', json.dumps(data).encode())
                elif self.path in ('/snapshot.jpg', '/stream.mjpg'):
                    if not clients.acquire(blocking=False):
                        self.reply(503, 'text/plain', b'Two image clients already connected')
                        return
                    try:
                        if self.path == '/snapshot.jpg':
                            with condition:
                                jpeg = state['jpeg']
                                fresh = time.monotonic() - state['at'] < 3
                            if not jpeg or not fresh:
                                self.reply(503, 'text/plain', b'No fresh camera frame')
                            else:
                                self.reply(200, 'image/jpeg', jpeg)
                            return
                        self.send_response(200)
                        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                        self.send_header('Cache-Control', 'no-store')
                        self.end_headers()
                        previous = -1
                        while not stop.is_set():
                            with condition:
                                condition.wait_for(lambda: state['seq'] != previous or stop.is_set(), timeout=3)
                                if stop.is_set() or time.monotonic() - state['at'] >= 3:
                                    break
                                jpeg, previous = state['jpeg'], state['seq']
                            self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ' + str(len(jpeg)).encode() + b'\r\n\r\n' + jpeg + b'\r\n')
                            self.wfile.flush()
                    finally:
                        clients.release()
                else:
                    self.reply(404, 'text/plain', b'Not found')
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                pass

    class Server(ThreadingHTTPServer):
        daemon_threads = True
        request_queue_size = 4

    server = Server((args.bind, args.port), Handler)
    server.timeout = 0.5
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    worker = threading.Thread(target=capture, daemon=True)
    worker.start()
    try:
        if not ready.wait(15) or state['error']:
            raise RuntimeError(state['error'] or 'camera startup timeout')
        print(f'Camera preview: http://{args.bind}:{args.port}/', flush=True)
        while not stop.is_set():
            server.handle_request()
        if state['error']:
            raise RuntimeError(state['error'])
    finally:
        stop.set()
        with condition:
            condition.notify_all()
        server.server_close()
        worker.join(timeout=2)
        lock.close()


if __name__ == '__main__':
    main()
