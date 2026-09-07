#!/usr/bin/env python3
"""One-shot standard ONNX Runtime CPU reference backend, verified on Mac only.

This is not an adapter for python3-spacemit-ort. Only --output is the protocol;
stdout and stderr may contain diagnostics. The caller owns process timeouts.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
import tempfile

from validate_yolo26 import ContractError, load_and_validate_model, load_spec


def read_input(path: Path, spec: dict):
    import numpy as np

    shape = (1, 3, spec["input_height"], spec["input_width"])
    expected = int(np.prod(shape)) * 4
    data = path.read_bytes()
    if len(data) != expected:
        raise ContractError(f"input must contain exactly {expected} bytes of little-endian float32, got {len(data)}")
    tensor = np.frombuffer(data, dtype="<f4").reshape(shape)
    if not np.isfinite(tensor).all() or (tensor < 0).any() or (tensor > 1).any():
        raise ContractError("input must contain finite RGB values normalized to [0, 1]")
    return np.ascontiguousarray(tensor, dtype=np.float32)


def encode_output(output, spec: dict) -> dict:
    import numpy as np

    expected = (1, spec["max_detections"], 6)
    if output.dtype != np.float32 or output.shape != expected:
        raise ContractError(f"runtime output must be float32 {expected}; got {output.dtype} {output.shape}")
    if not np.isfinite(output).all():
        raise ContractError("runtime output contains NaN or infinity")
    scores, classes = output[..., 4], output[..., 5]
    if (scores < 0).any() or (scores > 1).any():
        raise ContractError("runtime scores must be probabilities in [0, 1]")
    if (classes < 0).any() or (classes >= spec["class_count"]).any() or (classes != np.floor(classes)).any():
        raise ContractError("runtime class IDs must be integers in [0, 79]")
    # Geometry belongs to the Rust decoder after its strict confidence filter.
    # Unconstrained low-confidence regression rows may contain empty/reversed boxes.
    # Do not threshold, sigmoid, multiply objectness or run NMS here.
    return {"shape": list(output.shape), "values": output.reshape(-1).tolist()}


def write_output(path: Path, payload: dict) -> None:
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent, prefix=".onnx-output-", delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(payload, stream, allow_nan=False, separators=(",", ":"))
            stream.write("\n")
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def run(model_path: Path, spec_path: Path, input_path: Path, output_path: Path) -> None:
    spec = load_spec(spec_path)
    model, report = load_and_validate_model(model_path, spec)
    tensor = read_input(input_path, spec)
    # Validate the graph before importing/constructing the reference runtime.
    import onnxruntime as ort

    session = ort.InferenceSession(model.SerializeToString(), providers=["CPUExecutionProvider"])
    output = session.run([spec["output_name"]], {spec["input_name"]: tensor})[0]
    write_output(output_path, encode_output(output, spec))
    print(f"standard ONNX Runtime {ort.__version__}, CPU reference; model SHA256 {report['sha256']}", file=sys.stderr)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("model", "spec", "input", "output"):
        parser.add_argument(f"--{name}", required=True, type=Path)
    args = parser.parse_args()
    try:
        paths = [path.resolve() for path in (args.model, args.spec, args.input, args.output)]
        if len(set(paths)) != len(paths):
            raise ContractError("model, spec, input and output paths must be distinct")
        run(args.model, args.spec, args.input, args.output)
    except Exception as error:
        print(f"ONNX reference worker failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
