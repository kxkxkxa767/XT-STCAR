#!/usr/bin/env python3
"""Validate the locked YOLO26n Detect ONNX contract without creating an ORT session."""

from __future__ import annotations

import argparse
import ast
import hashlib
import json
import math
import os
from pathlib import Path
import stat
import sys

ULTRALYTICS_VERSION = "8.4.142"
SPEC_FIXED = {
    "schema_version": 1,
    "model_family": "yolo26n",
    "task": "detect",
    "input_name": "images",
    "output_name": "output0",
    "input_width": 320,
    "input_height": 320,
    "input_layout": "NCHW_RGB_0_1",
    "input_dtype": "float32",
    "output_layout": "B_N_XYXY_SCORE_CLASS",
    "max_detections": 300,
    "class_count": 80,
}


class ContractError(ValueError):
    """A model, tensor or configuration does not satisfy the supported contract."""


def read_regular_file(path: str | Path, max_bytes: int) -> bytes:
    """Bound actual bytes and reject FIFO/device input before a blocking read."""
    flags = os.O_RDONLY | getattr(os, "O_NONBLOCK", 0) | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOCTTY", 0)
    fd = os.open(path, flags)
    with os.fdopen(fd, "rb") as stream:
        metadata = os.fstat(stream.fileno())
        if not stat.S_ISREG(metadata.st_mode):
            raise ContractError(f"input {path} must be a regular file")
        if metadata.st_size > max_bytes:
            raise ContractError(f"input {path} exceeds {max_bytes} bytes")
        data = stream.read(max_bytes + 1)
    if len(data) > max_bytes:
        raise ContractError(f"input {path} exceeds {max_bytes} bytes")
    return data


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ContractError(f"duplicate JSON field: {key}")
        result[key] = value
    return result


def load_spec(path: str | Path) -> dict:
    spec = json.loads(read_regular_file(path, 1024 * 1024).decode("utf-8"), object_pairs_hook=_unique_object)
    if not isinstance(spec, dict):
        raise ContractError("spec must be a JSON object")
    unknown = set(spec) - (set(SPEC_FIXED) | {"confidence_threshold"})
    if unknown:
        raise ContractError(f"unknown spec fields: {sorted(unknown)}")
    for key, expected in SPEC_FIXED.items():
        value = spec.get(key)
        if type(value) is not type(expected) or value != expected:
            raise ContractError(f"spec {key}: expected {expected!r}, got {value!r}")
    threshold = spec.get("confidence_threshold")
    if type(threshold) not in (int, float) or not math.isfinite(threshold) or not 0 <= threshold <= 1:
        raise ContractError("confidence_threshold must be finite and in [0, 1]")
    return spec


def _literal(metadata: dict, key: str):
    if key not in metadata:
        raise ContractError(f"missing ONNX metadata: {key}")
    try:
        return ast.literal_eval(metadata[key])
    except (ValueError, SyntaxError) as error:
        raise ContractError(f"invalid ONNX metadata: {key}") from error


def _nodes(container):
    """Include control-flow subgraphs and local functions when looking for NMS."""
    import onnx

    for node in container.node:
        yield node
        for attribute in node.attribute:
            if attribute.type == onnx.AttributeProto.GRAPH:
                yield from _nodes(attribute.g)
            elif attribute.type == onnx.AttributeProto.GRAPHS:
                for graph in attribute.graphs:
                    yield from _nodes(graph)


def _tensor(value, expected_name: str, expected_shape: list[int]) -> dict:
    import onnx

    if value.name != expected_name:
        raise ContractError(f"tensor name: expected {expected_name!r}, got {value.name!r}")
    tensor = value.type.tensor_type
    if tensor.elem_type != onnx.TensorProto.FLOAT:
        raise ContractError(f"{value.name}: expected FLOAT32, got dtype {tensor.elem_type}")
    shape = [dim.dim_value if dim.HasField("dim_value") else None for dim in tensor.shape.dim]
    if shape != expected_shape:
        raise ContractError(
            f"{value.name}: expected static {expected_shape}, got {shape}; raw heads are unsupported"
        )
    return {"name": value.name, "dtype": "float32", "shape": shape}


def validate_model(model, spec: dict) -> dict:
    import onnx

    pairs = [(entry.key, entry.value) for entry in model.metadata_props]
    metadata = dict(pairs)
    if len(pairs) != len(metadata):
        raise ContractError("duplicate ONNX metadata keys")
    for key, expected in {"task": "detect", "head": "Detect", "version": ULTRALYTICS_VERSION}.items():
        if metadata.get(key) != expected:
            raise ContractError(f"ONNX metadata {key}: expected {expected!r}, got {metadata.get(key)!r}")
    if _literal(metadata, "end2end") is not True:
        raise ContractError("ONNX end2end metadata must be True; raw/one-to-many heads are unsupported")
    names = _literal(metadata, "names")
    if not isinstance(names, dict) or set(names) != set(range(spec["class_count"])):
        raise ContractError("ONNX names must map integer class IDs 0..79 to 80 labels")
    if any(type(key) is not int for key in names) or any(not isinstance(v, str) or not v.strip() for v in names.values()):
        raise ContractError("ONNX names must contain integer IDs and nonempty string labels")
    if len(model.graph.input) != 1 or len(model.graph.output) != 1:
        raise ContractError("expected exactly one ONNX input and one output")
    input_info = _tensor(model.graph.input[0], spec["input_name"], [1, 3, 320, 320])
    output_info = _tensor(model.graph.output[0], spec["output_name"], [1, 300, 6])
    opsets = {item.domain: item.version for item in model.opset_import}
    if opsets.get("", opsets.get("ai.onnx")) != 17:
        raise ContractError(f"expected ONNX opset 17, got {opsets}")
    nodes = list(_nodes(model.graph))
    for function in model.functions:
        nodes.extend(_nodes(function))
    if any(node.op_type == "NonMaxSuppression" for node in nodes):
        raise ContractError("NonMaxSuppression is forbidden for this one-to-one contract")
    onnx.checker.check_model(model, full_check=True)
    return {
        "validated": True,
        "contract": "yolo26n-detect-one-to-one-fp32-static320",
        "identity_scope": "interface and exporter metadata only; provenance requires a trusted weights/model SHA256",
        "ultralytics_version": metadata["version"],
        "metadata": metadata,
        "task": metadata["task"],
        "head": metadata["head"],
        "end2end": True,
        "class_count": len(names),
        "names": names,
        "input": input_info,
        "output": output_info,
        "opset": 17,
        "non_max_suppression_nodes": 0,
        "node_count": len(nodes),
    }


def load_and_validate_model(path: str | Path, spec: dict):
    import onnx

    path = Path(path)
    # This initial baseline is a self-contained ONNX file. External weights must
    # receive an explicit packaging/provenance design before they are supported.
    data = read_regular_file(path, 64 * 1024 * 1024)
    model = onnx.load_model_from_string(data)
    messages = [model]
    while messages:
        message = messages.pop()
        if isinstance(message, onnx.TensorProto) and (
            message.data_location == onnx.TensorProto.EXTERNAL or message.external_data
        ):
            raise ContractError("external ONNX tensor files are unsupported; use a self-contained export")
        for field, value in message.ListFields():
            if field.message_type is not None:
                messages.extend(value if field.is_repeated else [value])
    report = validate_model(model, spec)
    report["model"] = str(path.resolve())
    report["sha256"] = hashlib.sha256(data).hexdigest()
    return model, report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True, type=Path)
    parser.add_argument("--spec", required=True, type=Path)
    args = parser.parse_args()
    try:
        _, report = load_and_validate_model(args.model, load_spec(args.spec))
        print(json.dumps(report, ensure_ascii=False, indent=2))
    except Exception as error:
        print(f"YOLO26 validation failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
