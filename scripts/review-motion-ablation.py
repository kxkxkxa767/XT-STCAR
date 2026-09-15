#!/usr/bin/env python3
"""Reproduce v9's three pinned native-host ablations; never execute target ELFs.

No network or repository mutations. Each variant gets a new git archive and a
separate Cargo target. Old clocks are isolated evidence, not a production option.
"""
from __future__ import annotations

import argparse
import hashlib
import io
import json
import os
from pathlib import Path
import subprocess
import tarfile

OLD = "7d28761c40f7d7e69a8ac77977f5b6710dd5a0c6"
CURRENT = "a6236380a68431342ab9e76f468c587c9dd775f3"
NAV = "crates/robot-core/src/navigation.rs"
ASYNC = "crates/runner/src/async_simulation.rs"
OBSERVER = "crates/runner/examples/v9_ablation.rs"
EXPECTED = {
    "old_navigation": "1b2b0070a92be2423d470f62b2ab83c312822b44b886050452e38907db377dff",
    "ordered_navigation": "72046e4d04c8034118fb2e6e30a4c4cc21def47606b7f47ce8c8297817724f21",
    "current_navigation": "ed61d03769e63b8e7d4bdc64f13f67c000699a2873bd92ac1f650f26da8c3b8d",
    "old_async": "b40bfefad7f7b928f038597a5f632f1a9749475059226d33255790a897ebeb44",
    "observed_async": "c9427be299ce01ad485e1369064f60692de53e58470a20b3c4b1d386e38bab9c",
    "observer": "fc52dc1952b5527e5cd5a1aadd7ca4438663bd305d5a27e588f6b34e8cf57355",
    "patch": "aecde158d41d1f8e64097842163bb3cc18639401c895ee2e3eee304efe0b2d94",
}


def sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def checked(data: bytes, name: str) -> bytes:
    if sha(data) != EXPECTED[name]:
        raise ValueError(f"{name}: source/asset hash differs from pinned review")
    return data


def report(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default="work/motion-v9-ablation-reproduction",
                        help="fresh directory below this repository's work/")
    parser.add_argument("--prepare-only", action="store_true",
                        help="verify/extract/apply pinned assets without compiling or running")
    args = parser.parse_args()
    root = Path(__file__).resolve().parent.parent
    assets = root / "scripts/review-motion-ablation"
    out = (root / args.output).resolve()
    if not out.is_relative_to((root / "work").resolve()) or out == (root / "work").resolve():
        parser.error("--output must be a new directory below repository work/")
    if out.exists():
        parser.error("output already exists; choose a fresh path (no overwrite/resume)")
    observer = checked((assets / "observer.rs").read_bytes(), "observer")
    patch = assets / "order-only.patch"
    checked(patch.read_bytes(), "patch")
    async_source = checked(subprocess.check_output(["git", "show", CURRENT + ":" + ASYNC],
                                                   cwd=root), "observed_async")
    out.mkdir(parents=True)
    provenance = {
        "schema_version": 1,
        "scope": "Native-host offline fixed LQR [40,60] input / [3,7,9] adoption. "
                 "Old clocks used only in isolated attribution builds. Same read-only observer; "
                 "no timing-performance assertion and no device or RISC-V execution.",
        "expected_source_and_asset_sha256": EXPECTED,
        "prepare_only": args.prepare_only,
        "variants": {},
    }
    for name, revision in (("old", OLD), ("order_only", OLD), ("current", CURRENT)):
        source = out / name
        source.mkdir()
        archive = subprocess.check_output(["git", "archive", revision], cwd=root)
        with tarfile.open(fileobj=io.BytesIO(archive)) as tar:
            # Git's tracked archive only; reject special files/links and extraction escapes.
            for member in tar.getmembers():
                destination = (source / member.name).resolve()
                if not destination.is_relative_to(source) or not (member.isdir() or member.isfile()):
                    raise ValueError(f"unsupported archive member: {member.name}")
            tar.extractall(source, filter="data")
        checked((source / NAV).read_bytes(),
                "current_navigation" if name == "current" else "old_navigation")
        checked((source / ASYNC).read_bytes(),
                "observed_async" if name == "current" else "old_async")
        # This exact a623 file differs from the pinned old file only by the public
        # observer wrapper and four post-poll calls (git diff OLD CURRENT -- ASYNC).
        (source / ASYNC).write_bytes(async_source)
        (source / OBSERVER).write_bytes(observer)
        env = os.environ.copy()
        # Prevent discovery of the enclosing production repository during git apply.
        env["GIT_CEILING_DIRECTORIES"] = str(out)
        if name == "order_only":
            for command in (["git", "apply", "--check", str(patch)],
                            ["git", "apply", str(patch)]):
                subprocess.run(command, cwd=source, env=env, check=True)
            checked((source / NAV).read_bytes(), "ordered_navigation")
        target = out / (name + "-target")
        env["CARGO_TARGET_DIR"] = str(target)
        details = {
            "source_commit": revision, "source_directory": str(source),
            "archive_sha256": sha(archive), "cargo_target_directory": str(target),
            "compiled_source_sha256": {
                str(p.relative_to(source)): sha(p.read_bytes())
                for p in sorted(source.glob("crates/**/*"))
                if p.is_file() and (p.suffix == ".rs" or p.name == "Cargo.toml"
                                    or (p.suffix == ".json" and "fixtures" in p.parts))
            },
            "workspace_manifest_sha256": sha((source / "Cargo.toml").read_bytes()),
            "lockfile_sha256": sha((source / "Cargo.lock").read_bytes()),
        }
        provenance["variants"][name] = details
        if not args.prepare_only:
            command = ["cargo", "build", "--release", "--locked", "--offline",
                       "-p", "xt-stcar-robot-runner", "--example", "v9_ablation"]
            details["build_command"] = command
            print(f"building {name} (independent target)", flush=True)
            with (out / (name + "-build.log")).open("w") as log:
                subprocess.run(command, cwd=source, env=env, stdout=log,
                               stderr=subprocess.STDOUT, check=True)
            executable = target / "release/examples/v9_ablation"
            details["native_executable_sha256"] = sha(executable.read_bytes())
            output = out / (name + ".json")
            print(f"running {name} (native host only)", flush=True)
            with output.open("w") as data, (out / (name + "-run.log")).open("w") as log:
                subprocess.run([str(executable)], cwd=source, env=env,
                               stdout=data, stderr=log, check=True)
            result = json.loads(output.read_text())
            details["result_sha256"] = sha(output.read_bytes())
            details["elapsed_ms"] = result["summary"]["elapsed_ms"]
            details["distance_m"] = result["summary"]["distance_m"]
            details["trace_omitted"] = result["omitted"]
        report(out / "provenance.json", provenance)
    if not args.prepare_only:
        old = json.loads((out / "old.json").read_text())
        order = json.loads((out / "order_only.json").read_text())
        current = json.loads((out / "current.json").read_text())
        evidence = {
            "order_only_commands_and_states_exactly_equal": old["commands"] == order["commands"],
            "order_only_plan_commands_equal":
                [p["report"]["command"] for p in old["plans"]]
                == [p["report"]["command"] for p in order["plans"]],
            "all_completed": all(r["summary"]["completed"] for r in (old, order, current)),
            "traces_complete": all(r["omitted"] == 0 for r in (old, order, current)),
            "scope": "Inspect the full per-version JSON for geometry and phase attribution. "
                     "Exact point differences alone do not prove extra route or topology.",
        }
        report(out / "comparison.json", evidence)
        if not all(v for v in evidence.values() if isinstance(v, bool)):
            raise RuntimeError("ablation invariants failed; inspect saved results")
    print(out)


if __name__ == "__main__":
    main()
