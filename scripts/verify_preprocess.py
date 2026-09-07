#!/usr/bin/env python3
"""Compare Rust preprocessing against OpenCV uint8 INTER_LINEAR on fixed fixtures.

This checks RGB/NCHW/normalization, nominal scale, ties-to-even resized dimensions,
and asymmetric center padding. It measures interpolation differences; it does not
claim bitwise equivalence for all possible images or model accuracy equivalence.
"""

import argparse
import json
import subprocess
import tempfile
from pathlib import Path

import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, default=Path("target/debug/xt-stcar"))
    parser.add_argument("--config", type=Path, default=Path("config/yolo26n.json"))
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    binary, config_path = args.binary.resolve(), args.config.resolve()
    spec = json.loads(config_path.read_text())
    iw, ih = spec["input_width"], spec["input_height"]
    rng = np.random.default_rng(260907)
    sizes = [(320, 320), (640, 480), (641, 480), (480, 641), (1920, 1080),
             (321, 319), (319, 321), (32, 17), (7, 11), (1, 2), (1280, 642),
             (1280, 646), (1023, 777)]
    results = []
    with tempfile.TemporaryDirectory(prefix="xt-stcar-preprocess-") as directory:
        work = Path(directory)
        for width, height in sizes:
            yy, xx = np.indices((height, width))
            fixtures = {
                "random": rng.integers(0, 256, (height, width, 3), dtype=np.uint8),
                "pattern": np.stack([(xx * 17) % 256, (yy * 31) % 256,
                                     ((xx + yy) % 2) * 255], axis=2).astype(np.uint8),
            }
            for kind, rgb in fixtures.items():
                source, tensor, metadata = work / "source.png", work / "input.f32", work / "transform.json"
                if not cv2.imwrite(str(source), rgb[:, :, ::-1]):
                    raise RuntimeError("cannot write PNG fixture")
                process = subprocess.run(
                    [str(binary), "preprocess", "--config", str(config_path), "--image", str(source),
                     "--tensor", str(tensor), "--transform", str(metadata)],
                    capture_output=True, text=True, timeout=30, check=False,
                )
                if process.returncode:
                    raise RuntimeError(f"Rust preprocess failed: {process.stderr}")
                scale = min(iw / width, ih / height)
                rw, rh = round(width * scale), round(height * scale)
                left, top = round((iw - rw) / 2 - 0.1), round((ih - rh) / 2 - 0.1)
                expected_metadata = dict(source_width=width, source_height=height,
                                         input_width=iw, input_height=ih,
                                         resized_width=rw, resized_height=rh,
                                         pad_left=left, pad_top=top, scale=scale)
                actual_metadata = json.loads(metadata.read_text())
                if actual_metadata != expected_metadata:
                    raise AssertionError(f"letterbox mismatch {width}x{height}: {actual_metadata} != {expected_metadata}")
                expected = np.full((ih, iw, 3), 114, dtype=np.uint8)
                expected[top:top + rh, left:left + rw] = cv2.resize(rgb, (rw, rh), interpolation=cv2.INTER_LINEAR)
                raw = np.fromfile(tensor, dtype="<f4")
                if raw.size != 3 * iw * ih or not np.isfinite(raw).all():
                    raise AssertionError("invalid Rust tensor size/values")
                actual = raw.reshape(3, ih, iw).transpose(1, 2, 0)
                quantized = np.rint(actual * 255).astype(np.int16)
                if not np.array_equal(actual, quantized.astype(np.float32) / np.float32(255)):
                    raise AssertionError("Rust tensor is not normalized uint8 RGB")
                difference = np.abs(quantized - expected.astype(np.int16))
                maximum = int(difference.max())
                if maximum > 1:
                    raise AssertionError(f"OpenCV difference exceeds one uint8 level: {width}x{height} {kind} max={maximum}")
                results.append(dict(width=width, height=height, fixture=kind,
                                    max_uint8_difference=maximum,
                                    mean_uint8_difference=float(difference.mean()),
                                    differing_channel_values=int(np.count_nonzero(difference))))
    report = dict(status="passed", reference=f"OpenCV {cv2.__version__} INTER_LINEAR uint8",
                  numpy_version=np.__version__, seed=260907, case_count=len(results),
                  max_uint8_difference=max(row["max_uint8_difference"] for row in results),
                  scope="Fixed PNG fixtures only; not universal bitwise or model-accuracy equivalence.",
                  cases=results)
    serialized = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(serialized)
    print(serialized, end="")


if __name__ == "__main__":
    main()
