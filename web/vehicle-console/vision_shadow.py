"""One active + one latest JPEG, isolated native perception; no actuator API."""
import base64
import collections
import json
import math
import os
import select
import statistics
import subprocess
import threading
import time
from pathlib import Path


def load_config(path):
    if not path.is_file() or path.stat().st_size > 65536:
        raise ValueError("vision config must be a regular JSON file <=64 KiB")
    config = json.loads(path.read_text())
    fields = ["binary", "road_config", "model", "runtime_lib", "model_spec"]
    if (not isinstance(config, dict) or set(config) != set(fields + ["schema_version", "enabled"])
        or type(config["schema_version"]) is not int or config["schema_version"] != 1 or type(config["enabled"]) is not bool):
        raise ValueError("invalid vision configuration schema")
    if not config["enabled"]:
        return None
    if any(not isinstance(config[k], str) or not Path(config[k]).is_absolute() or not Path(config[k]).is_file() for k in fields):
        raise ValueError("vision paths must be existing absolute files")
    if not os.access(config["binary"], os.X_OK):
        raise ValueError("vision binary must be executable")
    return [config[k] for k in fields]


class VisionShadow:
    def __init__(self, command):
        self.command = list(command)
        self.condition = threading.Condition()
        self.pending = None
        self.closed = False
        self.dropped = 0
        self.frames = 0
        self.result = None
        self.error = None
        self.process = None
        self.samples = collections.deque(maxlen=120)
        self.thread = threading.Thread(target=self.run, daemon=True)

    def start(self):
        self.thread.start()

    def submit(self, jpeg, captured_at, sequence):
        if len(jpeg) > 2 * 1024 * 1024 or not self.condition.acquire(False):
            return False
        try:
            if self.closed or self.error:
                return False
            if self.pending is not None:
                self.dropped += 1
            self.pending = (jpeg, captured_at, sequence)
            self.condition.notify()
            return True
        finally:
            self.condition.release()

    def snapshot(self):
        with self.condition:
            result = dict(self.result) if self.result else None
            if result:
                result['age_ms'] = max(0, time.monotonic() * 1000 - result['captured_at_ms'])
            return {'enabled': True, 'error': self.error, 'frames': self.frames,
                    'dropped': self.dropped, 'result': result}

    def close(self):
        with self.condition:
            self.closed = True
            self.pending = None
            self.condition.notify()
        proc = self.process
        if proc and proc.poll() is None:
            proc.terminate()  # Perception-only process; never the chassis owner.
        self.thread.join(timeout=2)

    def transfer(self, data, timeout):
        """Nonblocking pipe IO with a wall deadline, including writes and partial lines."""
        proc = self.process
        end = time.monotonic() + timeout
        offset, response = 0, bytearray()
        while time.monotonic() < end and not self.closed:
            if proc.poll() is not None:
                raise RuntimeError('vision process exited: ' + str(proc.returncode))
            readable, writable, _ = select.select([proc.stdout], [proc.stdin] if offset < len(data) else [], [], .1)
            if writable:
                try:
                    offset += os.write(proc.stdin.fileno(), memoryview(data)[offset:offset + 65536])
                except BlockingIOError:
                    pass
            if readable:
                chunk = os.read(proc.stdout.fileno(), 65536)
                if not chunk:
                    raise RuntimeError('vision process EOF')
                response.extend(chunk)
                if len(response) > 128 * 1024:
                    raise RuntimeError('oversized vision result')
                if b'\n' in response:
                    if not response.endswith(b'\n') or response.count(b'\n') != 1 or offset != len(data):
                        raise RuntimeError('invalid vision response framing')
                    return json.loads(response)
        raise RuntimeError('vision process timeout/stopped')

    @staticmethod
    def annotate(jpeg, value):
        import cv2
        import numpy as np
        image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError('shadow JPEG decode failed')
        diagnostics = value['diagnostics']
        names = diagnostics['class_names']
        for d in diagnostics['detections']:
            x0, y0, x1, y1 = map(lambda x: int(round(x)), d['xyxy'])
            label = names[d['class_id']] if d['class_id'] < len(names) else 'class ' + str(d['class_id'])
            cv2.rectangle(image, (x0, y0), (x1, y1), (100, 240, 100), 2)
            cv2.putText(image, f'{label} {d["confidence"]:.2f}', (x0, max(14, y0 - 5)), cv2.FONT_HERSHEY_SIMPLEX, .45, (100, 240, 100), 1)
        cv2.putText(image, f'SHADOW #{value["sequence"]} / NO AUTO DRIVE', (8, 22), cv2.FONT_HERSHEY_SIMPLEX, .5, (50, 220, 255), 1)
        ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 75])
        if not ok:
            raise RuntimeError('shadow JPEG encode failed')
        return base64.b64encode(encoded).decode('ascii')

    def run(self):
        try:
            self.process = subprocess.Popen(self.command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)
            for stream in (self.process.stdin, self.process.stdout):
                os.set_blocking(stream.fileno(), False)
            ready = self.transfer(b'', 40)
            if ready != {'kind': 'ready', 'physical_output_enabled': False}:
                raise RuntimeError('invalid vision startup handshake')
            while True:
                with self.condition:
                    self.condition.wait_for(lambda: self.pending is not None or self.closed)
                    if self.closed:
                        return
                    jpeg, captured_at, sequence = self.pending
                    self.pending = None
                header = {'sequence': sequence, 'captured_at_ms': int(captured_at * 1000), 'jpeg_bytes': len(jpeg)}
                started = time.monotonic()
                value = self.transfer((json.dumps(header) + '\n').encode() + jpeg, 5)
                if (value.get('kind') != 'vision_shadow' or value.get('physical_output_enabled') is not False
                    or value.get('sequence') != sequence or value.get('captured_at_ms') != header['captured_at_ms']):
                    raise RuntimeError('vision result source identity mismatch')
                value['jpeg_base64'] = self.annotate(jpeg, value)
                value['worker_ms'] = (time.monotonic() - started) * 1000
                value['publish_age_ms'] = (time.monotonic() - captured_at) * 1000
                with self.condition:
                    self.frames += 1
                    if self.frames > 5:
                        self.samples.append(value['worker_ms'])
                    ordered = sorted(self.samples)
                    value['performance'] = {'warmup_frames': 5, 'window_frames': len(ordered),
                        'median_ms': statistics.median(ordered) if ordered else None,
                        'p95_ms': ordered[math.ceil(len(ordered)*.95)-1] if ordered else None,
                        'max_ms': max(ordered) if ordered else None}
                    self.result = value
        except Exception as error:
            with self.condition:
                if not self.closed:
                    self.error = str(error)
                self.pending = None
        finally:
            proc = self.process
            if proc:
                if proc.poll() is None:
                    proc.kill()
                proc.wait(timeout=2)
                proc.stdin.close()
                proc.stdout.close()
