"""Native custom-model IPC and latest-frame integration, synthetic JPEGs only."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import cv2
import numpy as np
import onnx
from onnx import helper, numpy_helper, TensorProto
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT / 'web/vehicle-console'))
from validate_yolo26 import load_spec, load_and_validate_model
from vision_shadow import VisionShadow
from export_yolo26 import EXPORT_ARGS

BIN = ROOT / 'target/debug/vision-shadow'
ORT = ROOT / 'toolchains/onnxruntime-macos-arm64-1.29.0/lib/libonnxruntime.1.29.0.dylib'


@pytest.fixture
def custom(tmp_path):
    spec = load_spec(ROOT / 'config/yolo26-race4.json')
    values = np.zeros((1, 300, 6), dtype=np.float32)
    values[0, 0] = [85, 135, 155, 245, .9, 0]  # 40 px letterbox padding
    graph = helper.make_graph([
        helper.make_node('Constant', [], ['output0'], value=numpy_helper.from_array(values))],
        'SYNTHETIC_TEST_ONLY', [helper.make_tensor_value_info('images', TensorProto.FLOAT, [1, 3, 320, 320])],
        [helper.make_tensor_value_info('output0', TensorProto.FLOAT, [1, 300, 6])])
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid('', 17)], ir_version=10)
    helper.set_model_props(model, {'task':'detect', 'head':'Detect', 'version':'8.4.142',
        'end2end':'True', 'names':repr(dict(enumerate(spec['class_names'])))})
    path = tmp_path / 'synthetic.onnx'
    onnx.save(model, path)
    _, record = load_and_validate_model(path, spec)
    record['training'] = {'schema_version':1, 'model_family':'yolo26n',
        'weights_sha256':hashlib.sha256(b'synthetic fixture, not trained weights').hexdigest(),
        'dataset_version':'SYNTHETIC_TEST_ONLY', 'class_names':spec['class_names']}
    record['weights_sha256'] = record['training']['weights_sha256']
    record['export_args'] = EXPORT_ARGS
    path.with_suffix('.provenance.json').write_text(json.dumps(record))
    return path, record


def command(model, spec=None):
    if not BIN.exists() or not ORT.exists():
        pytest.skip('build vision-shadow and install the locked Mac native ORT runtime first')
    return [str(BIN), str(ROOT / 'config/road-perception-sim.json'), str(model), str(ORT),
            str(spec or ROOT / 'config/yolo26-race4.json')]


def jpeg():
    frame = np.full((240, 320, 3), 35, dtype=np.uint8)
    cv2.fillPoly(frame, [np.array([[120, 100], [95, 199], [145, 199]], dtype=np.int32)], (0,0,255))
    return cv2.imencode('.jpg', frame)[1].tobytes()


def packet(image, seq, at):
    return (json.dumps({'sequence':seq, 'captured_at_ms':at, 'jpeg_bytes':len(image)})+'\n').encode()+image


def test_native_custom_model_restores_pixels_preserves_source_and_visual_only(custom):
    proc = subprocess.run(command(custom[0]), input=packet(jpeg(), 1, 1200)+packet(jpeg(), 2, 1300), capture_output=True, timeout=15)
    assert proc.returncode == 0, proc.stderr.decode()
    ready, first, second = map(json.loads, proc.stdout.splitlines())
    assert ready['physical_output_enabled'] is False
    for result, at in [(first,1200), (second,1300)]:
        assert result['physical_output_enabled'] is False
        assert result['captured_at_ms'] == result['road']['elements']['captured_at'] == at
        assert result['diagnostics']['detections'][0]['xyxy'] == [85.,95.,155.,205.]
        cone = result['road']['elements']['observations'][0]
        assert cone['source'] == 'ground_projection'
        assert cone['color'] == 'red'
        assert abs(cone['position_body_m']['x_m'] - .8333) < .05
        assert result['diagnostics']['simulation_only'] is True


@pytest.mark.parametrize('change', ['class_order', 'missing_training', 'model_hash', 'stale_time', 'huge_header'])
def test_native_rejects_unbound_models_and_bad_input(custom, tmp_path, change):
    path, record = custom
    spec = ROOT / 'config/yolo26-race4.json'
    data = packet(jpeg(), 1, 1200)
    if change == 'class_order':
        config = json.loads(spec.read_text()); config['class_names'].reverse()
        spec = tmp_path / 'reordered.json'; spec.write_text(json.dumps(config))
    elif change == 'missing_training':
        del record['training'];path.with_suffix('.provenance.json').write_text(json.dumps(record))
    elif change == 'model_hash':
        record['sha256'] = '0'*64;path.with_suffix('.provenance.json').write_text(json.dumps(record))
    elif change == 'stale_time': data += packet(jpeg(), 2, 1200)
    elif change == 'huge_header': data = b'x'*4097
    proc = subprocess.run(command(path,spec), input=data, capture_output=True, timeout=15)
    assert proc.returncode != 0


def test_latest_worker_retains_original_timestamp_and_paired_jpeg(custom):
    worker = VisionShadow(command(custom[0]))
    source = time.monotonic()
    # Queueing before startup deterministically replaces stale pending frames.
    assert worker.submit(jpeg(), source - .2, 1)
    assert worker.submit(jpeg(), source, 2)
    assert worker.dropped == 1
    worker.start()
    try:
        deadline = time.monotonic()+10
        while time.monotonic()<deadline:
            state=worker.snapshot()
            if state['error'] or state['result']: break
            time.sleep(.02)
        assert state['error'] is None
        result = state['result']
        assert result['sequence'] == 2
        assert result['captured_at_ms'] == int(source*1000)
        assert result['age_ms'] >= 0
        assert result['jpeg_base64']
        assert result['performance']['window_frames'] == 0  # warmup excluded
    finally:
        worker.close()
    assert not worker.thread.is_alive()


def test_persistent_config_requires_explicit_paths_and_can_stay_disabled(tmp_path):
    from vision_shadow import load_config
    config = json.loads((ROOT/'config/vision-shadow.example.json').read_text())
    path = tmp_path/'vision.json';path.write_text(json.dumps(config))
    assert load_config(path) is None
    config['enabled'] = True;path.write_text(json.dumps(config))
    with pytest.raises(ValueError, match='absolute files'): load_config(path)
    config['enabled'] = 'yes';path.write_text(json.dumps(config))
    with pytest.raises(ValueError, match='schema'): load_config(path)


def test_faulted_native_worker_latches_error_without_accepting_more_frames(custom):
    worker=VisionShadow(command(custom[0]))
    worker.start()
    try:
        worker.submit(b'not JPEG',time.monotonic(),1)
        deadline=time.monotonic()+10
        while time.monotonic()<deadline and worker.snapshot()['error'] is None: time.sleep(.02)
        assert worker.snapshot()['error']
        assert worker.submit(jpeg(),time.monotonic(),2) is False
        assert worker.snapshot()['result'] is None
    finally: worker.close()


def test_console_single_vision_photo_uses_same_result_image_and_rejects_stale(tmp_path):
    import argparse, base64
    import server
    app = server.Console(argparse.Namespace(access_file=str(tmp_path/'access.json'), output=str(tmp_path/'files')))
    payload = jpeg()
    result = {'age_ms':20, 'sequence':42, 'jpeg_base64':base64.b64encode(payload).decode()}
    class ResultSource:
        def snapshot(self): return {'result':result, 'error':None}
    app.vision = ResultSource()
    name = app.photo('vision')['file']
    assert name.startswith('vision-42-') and name.endswith('.jpg')
    assert (tmp_path/'files'/name).read_bytes() == payload
    result['age_ms'] = 2000
    with pytest.raises(ValueError, match='stale'): app.photo('vision')
