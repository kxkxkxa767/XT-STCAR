#!/usr/bin/env bash
# Build only with the project-locked toolchain; never replace global defaults.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
cd "$ROOT"
OFFLINE=""
case "${1:-}" in
  "") ;;
  --offline) OFFLINE=--offline; shift ;;
  -h|--help) printf '%s\n' 'Usage: scripts/build-riscv.sh [--offline]' 'Runs locked fmt/test/clippy, cross-links release, checks ELF and saves build evidence.'; exit 0 ;;
  *) printf 'Unknown option: %s\n' "$1" >&2; exit 2 ;;
esac
[[ $# -eq 0 ]] || { printf '%s\n' 'Unexpected arguments' >&2; exit 2; }
[[ -f Cargo.lock ]] || { printf '%s\n' 'Cargo.lock is required; create and review it before building.' >&2; exit 1; }
export RUSTUP_TOOLCHAIN=1.97.1
export CARGO_TARGET_DIR="$ROOT/target"
export CARGO_ZIGBUILD_ZIG_PATH="$ROOT/toolchains/zig-aarch64-macos-0.15.2/zig"
export CARGO_ZIGBUILD_CACHE_DIR="$ROOT/tmp/cargo-zigbuild-cache"
export ZIG_GLOBAL_CACHE_DIR="$ROOT/tmp/zig-cache"
ZIGBUILD="$ROOT/toolchains/cargo-zigbuild-0.23.4/cargo-zigbuild"
PYTHON="${PYTHON:-python3}"
[[ "$(rustup run 1.97.1 rustc --version)" == 'rustc 1.97.1 '* ]] || { printf '%s\n' 'Rust 1.97.1 required' >&2; exit 1; }
[[ "$(rustup run 1.97.1 cargo --version)" == 'cargo 1.97.1 '* ]] || { printf '%s\n' 'Cargo 1.97.1 required' >&2; exit 1; }
[[ "$("$CARGO_ZIGBUILD_ZIG_PATH" version)" == '0.15.2' ]] || { printf '%s\n' 'Project Zig 0.15.2 required' >&2; exit 1; }
[[ "$("$ZIGBUILD" --version)" == 'cargo-zigbuild 0.23.4' ]] || { printf '%s\n' 'Project cargo-zigbuild 0.23.4 required' >&2; exit 1; }
rustup run 1.97.1 cargo fmt --all --check
rustup run 1.97.1 cargo test --workspace --locked ${OFFLINE:+$OFFLINE}
rustup run 1.97.1 cargo clippy --workspace --all-targets --locked ${OFFLINE:+$OFFLINE} -- -D warnings
"$ZIGBUILD" zigbuild --manifest-path "$ROOT/Cargo.toml" \
  --package xt-stcar --package xt-stcar-robot-runner --bins \
  --release --locked ${OFFLINE:+$OFFLINE} --target riscv64gc-unknown-linux-gnu.2.38
for PROGRAM in xt-stcar xt-stcar-robot; do
  BINARY="$ROOT/target/riscv64gc-unknown-linux-gnu/release/$PROGRAM"
  "$PYTHON" "$ROOT/scripts/inspect_elf.py" "$BINARY" --output "$BINARY.elf.json"
done
"$PYTHON" - "$ROOT" <<'PY'
import datetime
import hashlib
import json
from pathlib import Path
import subprocess
import sys
root = Path(sys.argv[1])
binary = root / "target/riscv64gc-unknown-linux-gnu/release/xt-stcar"
binaries = {name: binary.with_name(name) for name in ("xt-stcar", "xt-stcar-robot")}
sources = sorted({root / "Cargo.lock", root / "Cargo.toml", root / "rust-toolchain.toml",
                  *root.glob("crates/**/Cargo.toml"), *root.glob("crates/**/*.rs")})
report = {
    "schema_version": 1,
    "built_at_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
    "target": "riscv64gc-unknown-linux-gnu.2.38",
    "rustc": subprocess.check_output(["rustup", "run", "1.97.1", "rustc", "--version"], text=True).strip(),
    "cargo": subprocess.check_output(["rustup", "run", "1.97.1", "cargo", "--version"], text=True).strip(),
    "zig": "0.15.2", "cargo_zigbuild": "0.23.4",
    "checks": ["cargo fmt --all --check", "cargo test --workspace --locked",
               "cargo clippy --workspace --all-targets --locked -- -D warnings", "ELF ABI/GLIBC inspection"],
    "binary_sha256": hashlib.sha256(binary.read_bytes()).hexdigest(),
    "binaries": {name: {"sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                        "path": str(path.relative_to(root)), "elf_report": name + ".elf.json"}
                 for name, path in binaries.items()},
    "source_sha256": {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
    "vehicle_execution_verified": False,
    "onnxruntime_library_included": False,
    "inference_runtime": "Requires a separately verified RISC-V standard ONNX Runtime C API dynamic library",
}
binary.with_name(binary.name + ".build.json").write_text(json.dumps(report, indent=2) + "\n")
print("Build evidence:", binary.with_name(binary.name + ".build.json"))
PY
