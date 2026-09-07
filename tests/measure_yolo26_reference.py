"""Run the locked bus.jpg PyTorch/ONNX reference comparison; no vehicle is accessed."""

import hashlib
import json
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
os.environ['YOLO_CONFIG_DIR'] = str(ROOT / 'tmp/ultralytics')
os.environ['YOLO_AUTOINSTALL'] = 'false'
import cv2
import numpy as np
import torch
from ultralytics import YOLO
from ultralytics.utils import ASSETS
from ultralytics.data.augment import LetterBox
from onnx_worker import run
from validate_yolo26 import load_and_validate_model, load_spec

outdir = ROOT / 'tmp/yolo26-reference'
outdir.mkdir(parents=True, exist_ok=True)
image_path = ASSETS / 'bus.jpg'
image = cv2.imread(str(image_path))
assert image is not None
letterbox = LetterBox(new_shape=(320, 320), auto=False, scale_fill=False, scaleup=True, center=True)
params = letterbox.get_params({'img': image})
canvas = letterbox(image=image)
tensor = np.ascontiguousarray(canvas[..., ::-1].transpose(2, 0, 1)[None], dtype=np.float32) / 255.0
input_path = outdir / 'bus-cv.f32le'
input_path.write_bytes(tensor.astype('<f4').tobytes())
output_path = outdir / 'bus-ort.json'
start = time.perf_counter()
run(ROOT / 'models/yolo26n.onnx', ROOT / 'config/yolo26n.json', input_path, output_path)
worker_seconds = time.perf_counter() - start
ort_payload = json.loads(output_path.read_text())
ort_values = np.array(ort_payload['values'], dtype=np.float32).reshape(ort_payload['shape'])
model = YOLO(str(ROOT / 'models/yolo26n.pt')).model.cpu().eval()
model.end2end = True
with torch.no_grad():
    result = model(torch.from_numpy(tensor))
    pt_values = (result[0] if isinstance(result, tuple) else result).cpu().numpy()
assert pt_values.shape == (1, 300, 6)
pt_high = pt_values[0][pt_values[0, :, 4] > 0.25]
ort_high = ort_values[0][ort_values[0, :, 4] > 0.25]
assert len(pt_high) == len(ort_high)
np.testing.assert_array_equal(pt_high[:, 5], ort_high[:, 5])
np.testing.assert_allclose(pt_high[:, :4], ort_high[:, :4], rtol=0, atol=0.01)
np.testing.assert_allclose(pt_high[:, 4], ort_high[:, 4], rtol=0, atol=0.0001)
_, validation = load_and_validate_model(ROOT / 'models/yolo26n.onnx', load_spec(ROOT / 'config/yolo26n.json'))
report = {
    'date': '2026-09-07', 'scope': 'Mac standard ONNX Runtime CPU reference; no vehicle execution or performance claim',
    'model_sha256': validation['sha256'],
    'image': str(image_path), 'image_source': 'Ultralytics 8.4.142 package assets/bus.jpg',
    'image_sha256': hashlib.sha256(image_path.read_bytes()).hexdigest(),
    'original_size_wh': [image.shape[1], image.shape[0]],
    'letterbox': params, 'input_f32le': str(input_path),
    'input_sha256': hashlib.sha256(input_path.read_bytes()).hexdigest(),
    'output': str(output_path), 'shape': list(ort_values.shape),
    'threshold_rule': 'score > 0.25', 'detections_count': len(ort_high),
    'detections': [{'box_xyxy_input': row[:4].tolist(), 'score': float(row[4]), 'class_id': int(row[5]), 'class_name': model.names[int(row[5])]} for row in ort_high],
    'pytorch_one_to_one_same_tensor': {
        'high_confidence_count_equal': True, 'high_confidence_classes_equal': True,
        'max_box_abs_error_pixels': float(np.max(np.abs(pt_high[:, :4] - ort_high[:, :4]))),
        'max_score_abs_error': float(np.max(np.abs(pt_high[:, 4] - ort_high[:, 4]))),
        'all_300_rows_class_equal': bool(np.array_equal(pt_values[..., 5], ort_values[..., 5])),
        'all_values_max_abs_error': float(np.max(np.abs(pt_values - ort_values))),
        'box_atol': 0.01, 'score_atol': 0.0001,
    },
    'cold_reference_worker_inprocess_seconds': worker_seconds,
}
(ROOT / 'models/yolo26n.reference-validation.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(report, indent=2))
