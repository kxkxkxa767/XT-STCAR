"""Deployment checks use files and a no-frame native session; no device access."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
loader=importlib.util.spec_from_file_location('deployment_check',ROOT/'scripts/check-vision-deployment.py')
module=importlib.util.module_from_spec(loader);loader.loader.exec_module(module)

@pytest.fixture
def bundle(tmp_path):
    release=tmp_path/'console';release.mkdir()
    for name in ['app.js','index.html','server.py','style.css','vision_shadow.py','vehicle-bridge']:
        (release/name).write_text('fixture')
    spec=json.loads((ROOT/'config/yolo26-blue1.example.json').read_text())
    (tmp_path/'spec.json').write_text(json.dumps(spec))
    (tmp_path/'model.onnx').write_bytes(b'synthetic-static-fixture')
    (tmp_path/'model.provenance.json').write_text(json.dumps({'sha256':hashlib.sha256(b'synthetic-static-fixture').hexdigest(), 'class_count':1,'names':{'0':'cone_blue'},'training':{'dataset_version':'fixture'}}))
    config={'schema_version':1,'enabled':True,'binary':str(release/'vehicle-bridge'),
            'model':str(tmp_path/'model.onnx'),'runtime_lib':str(release/'server.py'),
            'model_spec':str(tmp_path/'spec.json'),'road_config':str(ROOT/'config/road-perception-sim.json')}
    path=tmp_path/'vision.json';path.write_text(json.dumps(config))
    return path,release,config


def test_static_checks_do_not_execute_or_claim_native_validation(bundle):
    path,release,_=bundle
    result=module.check(path,release,True)
    assert result['static_checks_passed'] and not result['native_load_checked']
    assert not result['motor_commands_sent'] and not result['physical_devices_opened']

@pytest.mark.parametrize('fault',['missing_module','wrong_sha','wrong_class'])
def test_mixed_or_unbound_deployment_is_rejected(bundle,fault):
    path,release,config=bundle
    if fault=='missing_module': (release/'vision_shadow.py').unlink()
    elif fault=='wrong_sha': Path(config['model']).write_bytes(b'changed')
    else:
        spec=json.loads(Path(config['model_spec']).read_text());spec['class_names']=['blue_cone'];spec['road_classes']={'blue_cone':'cone_blue'}
        Path(config['model_spec']).write_text(json.dumps(spec))
    with pytest.raises(ValueError): module.check(path,release,True)


def test_native_ready_handshake_uses_real_official_model_without_frames(bundle):
    path,release,config=bundle
    binary=ROOT/'target/debug/vision-shadow'
    runtime=ROOT/'toolchains/onnxruntime-macos-arm64-1.29.0/lib/libonnxruntime.1.29.0.dylib'
    if not binary.is_file() or not runtime.is_file() or not (ROOT/'models/yolo26n.onnx').is_file():
        pytest.skip('locked Mac runtime and official model required')
    config.update(binary=str(binary),model=str(ROOT/'models/yolo26n.onnx'),runtime_lib=str(runtime),model_spec=str(ROOT/'config/yolo26n.json'))
    path.write_text(json.dumps(config))
    result=module.check(path,release)
    assert result['native_load_checked'] and not result['motor_commands_sent']
