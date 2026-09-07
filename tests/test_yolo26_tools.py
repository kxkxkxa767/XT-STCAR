"""Contract rejection and worker protocol tests; fixtures are explicitly synthetic."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from onnx_worker import encode_output, read_input  # noqa: E402
from export_yolo26 import export  # noqa: E402
from validate_yolo26 import ContractError, load_and_validate_model, load_spec, validate_model  # noqa: E402


@pytest.fixture
def spec():
    return load_spec(ROOT / "config/yolo26n.json")


@pytest.fixture
def model():
    values = np.zeros((1, 300, 6), dtype=np.float32)
    values[0, 0] = [10, 20, 30, 40, 0.75, 2]
    values[0, 1] = [10, 20, 30, 40, 0.25, 2]
    graph = helper.make_graph(
        [helper.make_node("Constant", [], ["output0"], value=numpy_helper.from_array(values))],
        "synthetic-contract-fixture",
        [helper.make_tensor_value_info("images", TensorProto.FLOAT, [1, 3, 320, 320])],
        [helper.make_tensor_value_info("output0", TensorProto.FLOAT, [1, 300, 6])],
    )
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 17)], ir_version=10)
    helper.set_model_props(model, {
        "task": "detect", "head": "Detect", "version": "8.4.142", "end2end": "True",
        "names": repr({index: f"fixture_{index}" for index in range(80)}),
    })
    return model


def set_metadata(model, key, value):
    props = {entry.key: entry.value for entry in model.metadata_props}
    props[key] = value
    helper.set_model_props(model, props)


def test_contract_fixture_passes(model, spec):
    report = validate_model(model, spec)
    assert report["end2end"] is True
    assert report["output"]["shape"] == [1, 300, 6]
    assert report["class_count"] == 80


def test_rejects_unknown_spec_fields(tmp_path):
    config = json.loads((ROOT / "config/yolo26n.json").read_text())
    config["typo_width"] = 640
    path = tmp_path / "spec.json"
    path.write_text(json.dumps(config))
    with pytest.raises(ContractError, match="unknown spec fields"):
        load_spec(path)


def test_rejects_duplicate_spec_fields(tmp_path):
    source = (ROOT / "config/yolo26n.json").read_text()
    path = tmp_path / "duplicate.json"
    path.write_text(source.replace('"schema_version": 1,', '"schema_version": 1, "schema_version": 1,'))
    with pytest.raises(ContractError, match="duplicate JSON field"):
        load_spec(path)


def test_export_rejects_unverified_weights_before_loading_torch(tmp_path):
    path = tmp_path / "yolo26n.pt"
    path.write_bytes(b"This filename is not a model identity")
    with pytest.raises(ContractError, match="unsupported weights SHA256"):
        export(path, ROOT / "config/yolo26n.json")


@pytest.mark.parametrize(("key", "value"), [
    ("task", "segment"), ("head", "Segment"), ("version", "8.4.141"),
    ("end2end", "False"), ("end2end", "1"), ("end2end", "not valid"),
    ("names", repr({index: str(index) for index in range(79)})),
    ("names", repr({str(index): str(index) for index in range(80)})),
    ("names", repr({index: "" for index in range(80)})),
])
def test_rejects_wrong_metadata(model, spec, key, value):
    set_metadata(model, key, value)
    with pytest.raises(ContractError):
        validate_model(model, spec)


def test_rejects_duplicate_metadata(model, spec):
    model.metadata_props.add(key="end2end", value="True")
    with pytest.raises(ContractError, match="duplicate"):
        validate_model(model, spec)


@pytest.mark.parametrize("shape", [[1, 84, 2100], [1, 6, 300], [2, 300, 6], [1, "dynamic", 6]])
def test_rejects_raw_dynamic_or_wrong_output(model, spec, shape):
    model.graph.output[0].CopyFrom(helper.make_tensor_value_info("output0", TensorProto.FLOAT, shape))
    with pytest.raises(ContractError, match="static"):
        validate_model(model, spec)


@pytest.mark.parametrize("which", ["input", "output"])
def test_rejects_fp16(model, spec, which):
    getattr(model.graph, which)[0].type.tensor_type.elem_type = TensorProto.FLOAT16
    with pytest.raises(ContractError, match="FLOAT32"):
        validate_model(model, spec)


@pytest.mark.parametrize("which", ["input", "output"])
def test_rejects_tensor_name(model, spec, which):
    getattr(model.graph, which)[0].name = "unexpected"
    with pytest.raises(ContractError, match="tensor name"):
        validate_model(model, spec)


def test_rejects_nms_in_nested_graph(model, spec):
    branch = helper.make_graph([helper.make_node("NonMaxSuppression", [], ["bad"])], "nested-nms", [], [])
    model.graph.node.append(helper.make_node("If", ["condition"], [], then_branch=branch, else_branch=branch))
    with pytest.raises(ContractError, match="NonMaxSuppression"):
        validate_model(model, spec)


def test_rejects_nms_in_local_function(model, spec):
    model.functions.append(helper.make_function(
        "fixture", "HiddenNms", [], [], [helper.make_node("NonMaxSuppression", [], ["bad"])],
        [helper.make_opsetid("", 17)],
    ))
    with pytest.raises(ContractError, match="NonMaxSuppression"):
        validate_model(model, spec)


def test_rejects_wrong_opset(model, spec):
    model.opset_import[0].version = 18
    with pytest.raises(ContractError, match="opset 17"):
        validate_model(model, spec)


def test_rejects_external_tensor_in_constant_attribute(tmp_path, model, spec):
    tensor = model.graph.node[0].attribute[0].t
    tensor.data_location = TensorProto.EXTERNAL
    tensor.external_data.add(key="location", value="outside-model.data")
    path = tmp_path / "external.onnx"
    path.write_bytes(model.SerializeToString())
    with pytest.raises(ContractError, match="external ONNX tensor"):
        load_and_validate_model(path, spec)


def test_reads_little_endian_input(tmp_path, spec):
    path = tmp_path / "input.f32le"
    values = np.linspace(0, 1, 3 * 320 * 320, dtype=np.float32).reshape(1, 3, 320, 320)
    path.write_bytes(values.astype("<f4").tobytes())
    np.testing.assert_array_equal(read_input(path, spec), values)


def test_rejects_wrong_byte_count(tmp_path, spec):
    path = tmp_path / "short.f32le"
    path.write_bytes(b"\x00" * 12)
    with pytest.raises(ContractError, match="exactly 1228800"):
        read_input(path, spec)


@pytest.mark.parametrize("bad", [np.nan, np.inf, -0.1, 1.1])
def test_rejects_invalid_input_values(tmp_path, spec, bad):
    values = np.zeros(3 * 320 * 320, dtype="<f4")
    values[0] = bad
    path = tmp_path / "invalid.f32le"
    path.write_bytes(values.tobytes())
    with pytest.raises(ContractError, match="finite RGB"):
        read_input(path, spec)


@pytest.mark.parametrize(("column", "bad"), [(0, np.nan), (4, 1.1), (4, -0.1), (5, 0.5), (5, 80), (5, -1)])
def test_rejects_invalid_runtime_detections(spec, column, bad):
    values = np.zeros((1, 300, 6), dtype=np.float32)
    values[0, 0, column] = bad
    with pytest.raises(ContractError):
        encode_output(values, spec)


def test_worker_preserves_low_confidence_regression_geometry(spec):
    values = np.zeros((1, 300, 6), dtype=np.float32)
    values[0, 0] = [20, 30, 10, 15, 0.25, 2]
    payload = encode_output(values, spec)
    assert payload["values"][:6] == [20, 30, 10, 15, 0.25, 2]


def test_worker_protocol_roundtrip(tmp_path, model):
    model_path = tmp_path / "fixture.onnx"
    input_path = tmp_path / "input.f32le"
    output_path = tmp_path / "output.json"
    onnx.save(model, model_path)
    input_path.write_bytes(np.zeros((1, 3, 320, 320), dtype="<f4").tobytes())
    result = subprocess.run([
        sys.executable, str(ROOT / "scripts/onnx_worker.py"),
        "--model", str(model_path), "--spec", str(ROOT / "config/yolo26n.json"),
        "--input", str(input_path), "--output", str(output_path),
    ], capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stderr
    payload = json.loads(output_path.read_text())
    assert payload["shape"] == [1, 300, 6]
    assert len(payload["values"]) == 1800
    # The worker preserves both scores; the Rust decoder owns strict thresholding.
    assert payload["values"][:12] == [10, 20, 30, 40, 0.75, 2, 10, 20, 30, 40, 0.25, 2]
    assert "CPU reference" in result.stderr
