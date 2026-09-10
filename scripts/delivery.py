#!/usr/bin/env python3
"""Allowlisted package creation and explicit, dry-run-by-default upload."""

import argparse
import datetime
import gzip
import hashlib
import io
import ipaddress
import json
from pathlib import Path, PurePosixPath
import re
import shlex
import subprocess
import sys
import tarfile

from inspect_elf import ElfError, inspect, inspect_bytes


ROOT = Path(__file__).resolve().parent.parent
TARGET = "riscv64gc-unknown-linux-gnu.2.38"
BINARY = ROOT / "target/riscv64gc-unknown-linux-gnu/release/xt-stcar"
BINARY_REPORTS = {"xt-stcar": "elf.json", "xt-stcar-robot": "robot-elf.json"}
MODEL_LIMIT = 64 * 1024 * 1024  # Same baseline limit as the Rust native backend.
PROVENANCE_LIMIT = 1024 * 1024
PACKAGE_LIMIT = 768 * 1024 * 1024
BASE_FILES = {
    "bin/xt-stcar", "bin/xt-stcar-robot", "config/yolo26n.json", "scripts/onnx_worker.py", "scripts/validate_yolo26.py",
    "scripts/check-vehicle.sh", "docs/部署包使用说明.md", "build.json", "elf.json",
    "robot-elf.json", "config/robot-sim.json", "examples/robot-sim.jsonl", "docs/机器人模块.md",
    "config/n10-replay.json", "config/robot-n10-replay.json", "examples/robot-n10-replay.jsonl",
    "config/chassis-calibration-sim.json", "config/serial-imu-capture.json", "config/serial-n10-capture.json",
    "docs/N10协议依据.md", "docs/底盘标定映射.md", "docs/Rust串口采集.md",
    "config/imu-replay.json", "config/robot-imu-replay.json", "examples/robot-imu-replay.jsonl", "docs/厂商协议Rust适配.md",
    "config/competition-sim.json", "config/competition-controller-sim.json", "config/road-perception-sim.json",
    "config/scan-assembly-sim.json", "config/laser-localization-sim.json", "docs/Rust比赛自主闭环.md",
    "Mac与车端命令手册.txt",
    "docs/competition-validation.json", "docs/competition-simulation-summary.json",
    "docs/运动控制设计与对比.md", "docs/motion-control-validation.json", "docs/motion-control-tracking-comparison.json",
    "docs/motion-control-competition-comparison.json",
    "docs/导航预测与失败首因修正.md", "docs/motion-v2-validation.json",
    "docs/motion-v2-before.json", "docs/motion-v2-competition-comparison.json",
    "docs/运动执行与通过点修正.md", "docs/motion-v3-validation.json",
    "docs/motion-v3-before.json", "docs/motion-v3-competition-comparison.json",
}
MODEL_FILES = {"models/yolo26n.onnx", "model-validation.json", "models/yolo26n.provenance.json",
               "licenses/Ultralytics-LICENSE"}
RUNTIME_REQUIREMENTS = {
    "default_inference": "Rust native ONNX Runtime C API, loaded dynamically",
    "riscv_onnxruntime_library_included": False,
    "riscv_onnxruntime_required_for_inference": True,
    "vendor_runtime_compatibility_verified": False,
    "python_reference_backend": "explicit --backend python-reference only",
    "offline_replay_and_self_check_require_onnxruntime": False,
}


def sha(data):
    return hashlib.sha256(data).hexdigest()


def read_regular(path, limit=PACKAGE_LIMIT):
    path = Path(path)
    if path.is_symlink() or not path.is_file() or path.stat().st_size > limit:
        raise ValueError(f"Expected a regular non-symlink file smaller than {limit} bytes: {path}")
    return path.read_bytes()


def source_files():
    return {"Cargo.lock", "Cargo.toml", "rust-toolchain.toml", *(
        str(path.relative_to(ROOT)) for pattern in ("crates/**/Cargo.toml", "crates/**/*.rs")
        for path in ROOT.glob(pattern)
    )}


def check_model_evidence(model_data, validation, provenance):
    if not isinstance(validation, dict) or not isinstance(provenance, dict):
        raise ValueError("Model validation and provenance must be JSON objects")
    digest = sha(model_data)
    if validation.get("validated") is not True or validation.get("sha256") != digest:
        raise ValueError("Packaged model does not match its validation evidence")
    if provenance.get("validated") is not True or provenance.get("sha256") != digest:
        raise ValueError("Packaged model does not match its provenance")
    fields = ("contract", "ultralytics_version", "metadata", "task", "head", "end2end", "class_count", "names",
              "input", "output", "opset", "non_max_suppression_nodes")
    for field in fields:
        if field not in validation or field not in provenance or json.dumps(validation[field], sort_keys=True) != json.dumps(provenance[field], sort_keys=True):
            raise ValueError(f"Model provenance differs from ONNX validation: {field}")


def verify_archive(path):
    archive_data = read_regular(path)
    files = {}
    with tarfile.open(fileobj=io.BytesIO(archive_data), mode="r:gz") as archive:
        total = 0
        for item in archive:
            name = PurePosixPath(item.name)
            if not item.isfile() or str(name) != item.name or name.is_absolute() or ".." in name.parts:
                raise ValueError(f"Unsafe archive member: {item.name}")
            if item.name in files:
                raise ValueError(f"Duplicate archive member: {item.name}")
            if item.name not in BASE_FILES | MODEL_FILES | {"manifest.json", "SHA256SUMS"}:
                raise ValueError(f"Archive member is outside the deployment allowlist: {item.name}")
            expected_mode = 0o755 if item.name in {f"bin/{name}" for name in BINARY_REPORTS} or item.name.endswith(".sh") else 0o644
            if item.mode != expected_mode:
                raise ValueError(f"Unexpected archive permissions: {item.name}")
            total += item.size
            if item.size > PACKAGE_LIMIT or total > PACKAGE_LIMIT:
                raise ValueError("Expanded package exceeds size limit")
            handle = archive.extractfile(item)
            if handle is None:
                raise ValueError(f"Unreadable archive member: {item.name}")
            files[item.name] = handle.read()
    if "manifest.json" not in files or "SHA256SUMS" not in files:
        raise ValueError("Package manifest and SHA256SUMS are required")
    manifest = json.loads(files["manifest.json"])
    if not isinstance(manifest, dict) or type(manifest.get("schema_version")) is not int or manifest.get("schema_version") != 1 or manifest.get("target") != TARGET:
        raise ValueError("Unsupported deployment package manifest")
    if type(manifest.get("model_included")) is not bool:
        raise ValueError("Manifest model_included must be boolean")
    if manifest.get("runtime_requirements") != RUNTIME_REQUIREMENTS:
        raise ValueError("Manifest must declare the separately required RISC-V inference runtime")
    expected = BASE_FILES | (MODEL_FILES if manifest.get("model_included") is True else set())
    if set(files) != expected | {"manifest.json", "SHA256SUMS"}:
        raise ValueError("Package contents differ from the deployment allowlist")
    entries = manifest.get("files", {})
    if not isinstance(entries, dict) or set(entries) != expected:
        raise ValueError("Manifest paths do not match package contents")
    for name in expected:
        if entries[name] != {"sha256": sha(files[name]), "size_bytes": len(files[name])}:
            raise ValueError(f"Manifest checksum/size mismatch: {name}")
    expected_sums = "".join(f"{sha(files[name])}  {name}\n" for name in sorted(expected | {"manifest.json"}))
    if files["SHA256SUMS"].decode() != expected_sums:
        raise ValueError("SHA256SUMS differs from package contents")
    # Run the same ELF checks on package bytes without trusting the manifest's claim.
    build = json.loads(files["build.json"])
    if not isinstance(build, dict) or not isinstance(build.get("binaries"), dict) or set(build["binaries"]) != set(BINARY_REPORTS):
        raise ValueError("Build evidence must record both executables")
    if build.get("target") != manifest["target"]:
        raise ValueError("Build target differs from deployment manifest target")
    for name, evidence_name in BINARY_REPORTS.items():
        data = files[f"bin/{name}"]
        report = inspect_bytes(data, name)
        elf = json.loads(files[evidence_name])
        if not isinstance(elf, dict):
            raise ValueError("ELF evidence must be a JSON object")
        if not isinstance(build["binaries"][name], dict) or sha(data) != build["binaries"][name].get("sha256") or sha(data) != elf.get("sha256"):
            raise ValueError(f"Binary does not match saved build/ELF evidence: {name}")
        if not elf.get("passed") or not report["passed"]:
            raise ValueError(f"Package ELF failed verification ({name}): {report['errors']}")
    if build.get("binary_sha256") != sha(files["bin/xt-stcar"]):
        raise ValueError("Primary binary does not match saved build evidence")
    if manifest.get("model_included") is True:
        if len(files["models/yolo26n.onnx"]) > MODEL_LIMIT or len(files["models/yolo26n.provenance.json"]) > PROVENANCE_LIMIT:
            raise ValueError("Model/provenance exceeds the Rust native backend size limits")
        validation = json.loads(files["model-validation.json"])
        provenance = json.loads(files["models/yolo26n.provenance.json"])
        check_model_evidence(files["models/yolo26n.onnx"], validation, provenance)
    return {"archive": str(Path(path).resolve()), "sha256": sha(archive_data), "size_bytes": len(archive_data),
            "file_count": len(files), "manifest": manifest}


def package(args):
    binaries = {name: BINARY.with_name(name) for name in BINARY_REPORTS}
    elf_reports = {name: inspect(path) for name, path in binaries.items()}
    for name, elf in elf_reports.items():
        if not elf["passed"]:
            raise ValueError(f"ELF check failed ({name}): {elf['errors']}")
    elf = elf_reports["xt-stcar"]
    build_data = read_regular(BINARY.with_name("xt-stcar.build.json"))
    build = json.loads(build_data)
    if not isinstance(build, dict):
        raise ValueError("Build evidence must be a JSON object")
    if build.get("target") != TARGET or build.get("binary_sha256") != elf["sha256"]:
        raise ValueError("Build evidence is missing/stale; run scripts/build-riscv.sh")
    if not isinstance(build.get("binaries"), dict) or set(build["binaries"]) != set(BINARY_REPORTS):
        raise ValueError("Build evidence must record both executables; run scripts/build-riscv.sh")
    for name, report in elf_reports.items():
        if not isinstance(build["binaries"][name], dict) or build["binaries"][name].get("sha256") != report["sha256"]:
            raise ValueError(f"Build evidence is missing/stale for {name}; run scripts/build-riscv.sh")
    if not build.get("rustc", "").startswith("rustc 1.97.1 ") or build.get("zig") != "0.15.2" or build.get("cargo_zigbuild") != "0.23.4":
        raise ValueError("Build evidence does not match locked tool versions")
    sources = build.get("source_sha256", {})
    if not isinstance(sources, dict) or set(sources) != source_files():
        raise ValueError("Source changed since build: Rust source set differs; rebuild first")
    for name, digest in sources.items():
        path = PurePosixPath(name)
        if path.is_absolute() or ".." in path.parts or sha(read_regular(ROOT / name)) != digest:
            raise ValueError(f"Source changed since build: {name}; rebuild first")
    files = {"build.json": build_data}
    for name, path in binaries.items():
        files[f"bin/{name}"] = read_regular(path)
        files[BINARY_REPORTS[name]] = (json.dumps(elf_reports[name], indent=2) + "\n").encode()
    for name in BASE_FILES - files.keys():
        files[name] = read_regular(ROOT / name)
    if args.model:
        model_path = Path(args.model).resolve()
        # Validate the exact bytes that will be packaged, after rejecting symlink input.
        model_data = read_regular(Path(args.model), MODEL_LIMIT)
        python = args.python or str(ROOT / ".venv-model/bin/python")
        result = subprocess.run([python, str(ROOT / "scripts/validate_yolo26.py"),
                                 "--model", str(model_path), "--spec", str(ROOT / "config/yolo26n.json")],
                                check=False, capture_output=True, text=True)
        if result.returncode:
            raise ValueError(f"Model validation failed: {result.stderr.strip()}")
        if read_regular(model_path, MODEL_LIMIT) != model_data:
            raise ValueError("Model changed during validation")
        validation = json.loads(result.stdout)
        if validation.get("validated") is not True or validation.get("sha256") != sha(model_data):
            raise ValueError("Model validator did not attest to the packaged model bytes")
        files["models/yolo26n.onnx"] = model_data
        files["model-validation.json"] = (json.dumps(validation, indent=2) + "\n").encode()
        provenance_data = read_regular(model_path.with_suffix(".provenance.json"), PROVENANCE_LIMIT)
        check_model_evidence(model_data, validation, json.loads(provenance_data))
        files["models/yolo26n.provenance.json"] = provenance_data
        files["licenses/Ultralytics-LICENSE"] = read_regular(ROOT / "资料/官方参考/Ultralytics_8.4.142/LICENSE")
    manifest = {
        "schema_version": 1, "project": "XT-STCAR", "target": TARGET,
        "created_at_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "model_included": bool(args.model), "vehicle_execution_verified": False,
        "runtime_requirements": RUNTIME_REQUIREMENTS,
        "files": {name: {"sha256": sha(data), "size_bytes": len(data)} for name, data in sorted(files.items())},
    }
    files["manifest.json"] = (json.dumps(manifest, indent=2, ensure_ascii=False) + "\n").encode()
    files["SHA256SUMS"] = "".join(f"{sha(data)}  {name}\n" for name, data in sorted(files.items())).encode()
    dist = ROOT / "dist"
    if dist.is_symlink():
        raise ValueError("dist must not be a symlink")
    dist.mkdir(exist_ok=True)
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    variant = "with-model" if args.model else "core"
    output = dist / f"xt-stcar-riscv64-{variant}-{stamp}-{elf['sha256'][:12]}.tar.gz"
    with output.open("xb") as raw:
        with gzip.GzipFile(filename="", fileobj=raw, mode="wb", mtime=0) as compressed:
            with tarfile.open(fileobj=compressed, mode="w", format=tarfile.USTAR_FORMAT) as archive:
                for name, data in sorted(files.items()):
                    item = tarfile.TarInfo(name)
                    item.size = len(data)
                    item.mode = 0o755 if name in {f"bin/{program}" for program in BINARY_REPORTS} or name.endswith(".sh") else 0o644
                    archive.addfile(item, io.BytesIO(data))
    report = verify_archive(output)
    checksum = output.with_name(output.name + ".sha256")
    checksum.write_text(f"{report['sha256']}  {output.name}\n")
    print(json.dumps({key: value for key, value in report.items() if key != "manifest"}, indent=2))


REMOTE_MKDIR = """import os,pathlib,stat,sys
p=pathlib.Path(sys.argv[1]); home=pathlib.Path.home()
if p.parent.parent.parent != home or p.parent.parent.name != 'xt-stcar' or p.parent.name != 'releases':
    raise SystemExit('Destination must be in the SSH account home/xt-stcar/releases')
if home.is_symlink() or not home.is_dir(): raise SystemExit('Unsafe account home')
for part in (home/'xt-stcar', home/'xt-stcar'/'releases'):
    try: os.mkdir(part, 0o700)
    except FileExistsError: pass
    s=os.lstat(part)
    if not stat.S_ISDIR(s.st_mode) or s.st_uid != os.getuid(): raise SystemExit('Unsafe release parent')
os.mkdir(p, 0o700)
print(p)
"""
REMOTE_HASH = """import hashlib,pathlib,sys
p=pathlib.Path(sys.argv[1])/'bundle.tar.gz'
actual=hashlib.sha256(p.read_bytes()).hexdigest()
if actual != sys.argv[2]: raise SystemExit('Remote checksum mismatch')
print('Uploaded archive verified: '+actual)
"""


def upload(args):
    # Argument validation and package verification happen before any subprocess/network call.
    if not args.host or not args.user or not args.remote_dir:
        raise ValueError("Explicit --host IP, --user and --remote-dir are required; no connection attempted")
    host = ipaddress.ip_address(args.host)
    if host.is_unspecified or host.is_multicast:
        raise ValueError("Host must be a unicast destination IP")
    if not re.fullmatch(r"[a-z_][a-z0-9_-]{0,31}", args.user) or args.user == "root":
        raise ValueError("Use an explicit non-root Unix account")
    prefix = f"/home/{args.user}/xt-stcar/releases/"
    if not args.remote_dir.startswith(prefix) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,79}", args.remote_dir[len(prefix):]):
        raise ValueError(f"--remote-dir must be {prefix}<new-release-name> (letters/digits/_/- only)")
    report = verify_archive(args.archive)
    destination = f"{args.user}@{host}"
    options = ["-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes", "-o", "ConnectTimeout=10"]
    mkdir_command = shlex.join(["python3", "-c", REMOTE_MKDIR, args.remote_dir])
    hash_command = shlex.join(["python3", "-c", REMOTE_HASH, args.remote_dir, report["sha256"]])
    scp_host = f"[{host}]" if host.version == 6 else str(host)
    commands = [
        ["ssh", *options, "-p", str(args.port), destination, mkdir_command],
        ["scp", *options, "-P", str(args.port), report["archive"], f"{args.user}@{scp_host}:{args.remote_dir}/bundle.tar.gz"],
        ["ssh", *options, "-p", str(args.port), destination, hash_command],
    ]
    print(json.dumps({"dry_run": not args.execute, "host": str(host), "user": args.user,
                      "remote_dir": args.remote_dir, "archive_sha256": report["sha256"],
                      "actions": ["Create a new private release directory (fail if it exists)",
                                  "Upload bundle.tar.gz", "Verify remote SHA256"],
                      "extract_or_run": False}, indent=2))
    if not args.execute:
        return
    for command in commands:
        subprocess.run(command, check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    pack = commands.add_parser("package", help="Create a minimal checked archive in project dist/")
    pack.add_argument("--model", help="Explicitly include a validated YOLO26n ONNX model")
    pack.add_argument("--python", help="Python with onnx for optional model validation")
    pack.set_defaults(func=package)
    verify = commands.add_parser("verify", help="Verify an archive without extracting or running it")
    verify.add_argument("archive", type=Path)
    verify.set_defaults(func=lambda args: print(json.dumps(verify_archive(args.archive), indent=2, ensure_ascii=False)))
    send = commands.add_parser("upload", help="Default dry-run; transfer only with --execute")
    send.add_argument("--archive", type=Path, required=True)
    send.add_argument("--host", help="Explicit new vehicle IP; DNS/SSH aliases are not accepted")
    send.add_argument("--user", help="Explicit non-root new vehicle account")
    send.add_argument("--remote-dir", help="/home/USER/xt-stcar/releases/NEW_NAME")
    send.add_argument("--port", type=int, default=22)
    send.add_argument("--execute", action="store_true", help="Execute the displayed upload actions")
    send.set_defaults(func=upload)
    args = parser.parse_args()
    try:
        if getattr(args, "port", 22) not in range(1, 65536):
            raise ValueError("Port must be 1..65535")
        args.func(args)
    except (OSError, ValueError, KeyError, TypeError, tarfile.TarError, subprocess.CalledProcessError, ElfError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
