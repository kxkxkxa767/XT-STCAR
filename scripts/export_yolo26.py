#!/usr/bin/env python3
"""Export trusted YOLO26n weights with the Ultralytics 8.4.142 one-to-one contract."""

from __future__ import annotations

import argparse
import hashlib
from importlib.metadata import version
import json
import os
from pathlib import Path
import sys

from validate_yolo26 import ContractError, ULTRALYTICS_VERSION, load_and_validate_model, load_spec

OFFICIAL_WEIGHTS_SHA256 = "9b09cc8bf347f0fc8a5f7657480587f25db09b34bf33b0652110fb03a8ad4fef"
OFFICIAL_WEIGHTS_URL = "https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26n.pt"
SOURCE_COMMIT = "2c5d376eee77b3d2217db609ce7b85837c1b191e"
EXPORT_ARGS = {
    "format": "onnx", "imgsz": 320, "batch": 1, "dynamic": False, "nms": False,
    "quantize": 32, "max_det": 300, "device": "cpu", "simplify": False, "opset": 17,
}


def export(weights: Path, spec_path: Path) -> dict:
    spec = load_spec(spec_path)
    if not weights.is_file():
        raise ContractError(f"weights not found: {weights}; download the official yolo26n.pt asset explicitly")
    with weights.open("rb") as stream:
        weights_sha256 = hashlib.file_digest(stream, "sha256").hexdigest()
    if weights_sha256 != OFFICIAL_WEIGHTS_SHA256:
        raise ContractError(
            f"unsupported weights SHA256 {weights_sha256}; this baseline only accepts the verified official yolo26n.pt asset from {OFFICIAL_WEIGHTS_URL}"
        )
    # Keep Ultralytics user settings in the project, and prohibit auto-install
    # changes to this locked environment while exporting.
    os.environ.setdefault("YOLO_CONFIG_DIR", str(Path(__file__).resolve().parents[1] / "tmp" / "ultralytics"))
    Path(os.environ["YOLO_CONFIG_DIR"]).expanduser().mkdir(parents=True, exist_ok=True)
    os.environ["YOLO_AUTOINSTALL"] = "false"
    import ultralytics
    from ultralytics import YOLO

    if ultralytics.__version__ != ULTRALYTICS_VERSION:
        raise ContractError(f"requires ultralytics=={ULTRALYTICS_VERSION}; got {ultralytics.__version__}")
    model = YOLO(str(weights))
    if model.task != "detect" or type(model.model.model[-1]).__name__ != "Detect":
        raise ContractError("only YOLO26n Detect weights are supported")
    head = model.model.model[-1]
    if len(model.names) != 80 or getattr(head, "one2one_cv2", None) is None or getattr(head, "one2one_cv3", None) is None:
        raise ContractError("weights must provide 80 classes and an available one-to-one head")
    output = Path(model.export(**EXPORT_ARGS))
    _, report = load_and_validate_model(output, spec)
    report["weights"] = str(weights.resolve())
    report["weights_sha256"] = weights_sha256
    report["weights_source"] = OFFICIAL_WEIGHTS_URL
    report["source_commit"] = SOURCE_COMMIT
    report["export_args"] = EXPORT_ARGS
    report["versions"] = {
        name: version(name) for name in ("ultralytics", "torch", "torchvision", "onnx", "onnxruntime", "numpy", "opencv-python")
    }
    provenance = output.with_suffix(".provenance.json")
    provenance.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--weights", type=Path, default=Path("models/yolo26n.pt"))
    parser.add_argument("--spec", type=Path, default=Path("config/yolo26n.json"))
    args = parser.parse_args()
    try:
        print(json.dumps(export(args.weights, args.spec), ensure_ascii=False, indent=2))
    except Exception as error:
        print(f"YOLO26 export failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
