"""Local-only artifact boundary tests. No target program, SSH or SCP is executed."""

import argparse
import contextlib
import hashlib
import io
import json
from pathlib import Path
import struct
import sys
import tarfile
import tempfile
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import delivery
import inspect_elf


def target_fixture():
    candidates = [ROOT / "target/riscv64gc-unknown-linux-gnu/release/xt-stcar",
                  ROOT / "tmp/riscv-hello/target/riscv64gc-unknown-linux-gnu/release/xt-stcar-cross-smoke"]
    for candidate in candidates:
        if candidate.is_file():
            return candidate.read_bytes()
    raise unittest.SkipTest("Build a RISC-V target first to run ELF/package regression checks")


class ElfTests(unittest.TestCase):
    def test_rejects_non_elf_and_truncated_tables(self):
        for data in (b"Mac ARM64", b"\x7fELF\x02\x01\x01" + b"\0" * 57):
            with self.subTest(data=data), self.assertRaises((ValueError, struct.error)):
                inspect_elf.inspect_bytes(data)

    def test_target_version_references_are_table_based(self):
        data = target_fixture()
        report = inspect_elf.inspect_bytes(data)
        self.assertTrue(report["passed"])
        self.assertLessEqual(tuple(map(int, report["maximum_glibc"].split("."))), (2, 38))
        self.assertEqual(report["glibc_baseline"], "2.38")
        self.assertTrue(report["versioned_undefined_symbols"])
        # An arbitrary string in debug/trailing data is not a dynamic requirement.
        appended = inspect_elf.inspect_bytes(data + b"GLIBC_99.99\0")
        self.assertEqual(appended["glibc_versions"], report["glibc_versions"])
        # Modifying the actual version string in the ELF table must fail baseline checking.
        original = f"GLIBC_{report['maximum_glibc']}\0".encode()
        self.assertEqual(len(original), len(b"GLIBC_2.38\0"))
        boundary = inspect_elf.inspect_bytes(data.replace(original, b"GLIBC_2.38\0"))
        self.assertTrue(boundary["passed"])
        self.assertEqual(boundary["maximum_glibc"], "2.38")
        modified = inspect_elf.inspect_bytes(data.replace(original, b"GLIBC_2.39\0"))
        self.assertFalse(modified["passed"])
        self.assertEqual(modified["maximum_glibc"], "2.39")

    def test_rejects_wrong_architecture_abi_and_loader(self):
        data = target_fixture()
        variants = []
        arch = bytearray(data)
        struct.pack_into("<H", arch, 18, 183)  # AArch64, not the car's RISC-V.
        variants.append(bytes(arch))
        abi = bytearray(data)
        struct.pack_into("<I", abi, 48, 1)  # Soft float, not LP64D.
        variants.append(bytes(abi))
        variants.append(data.replace(b"/lib/ld-linux-riscv64-lp64d.so.1", b"/tmp/ld-linux-riscv64-lp64d.so.1"))
        for value in variants:
            with self.subTest():
                self.assertFalse(inspect_elf.inspect_bytes(value)["passed"])


class DeliveryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(dir=ROOT / "tmp")
        self.root = Path(self.temp.name)
        self.binary = self.root / "target/riscv64gc-unknown-linux-gnu/release/xt-stcar"
        self.binary.parent.mkdir(parents=True)
        self.binary.write_bytes(target_fixture())
        self.robot = self.binary.with_name("xt-stcar-robot")
        self.robot.write_bytes(target_fixture() + b"robot fixture\n")
        for name in ("Cargo.lock", "Cargo.toml", "rust-toolchain.toml"):
            (self.root / name).write_text("# Locked test fixture\n")
        build = {"target": delivery.TARGET, "binary_sha256": delivery.sha(self.binary.read_bytes()),
                 "binaries": {name: {"sha256": delivery.sha(path.read_bytes())}
                              for name, path in {"xt-stcar": self.binary, "xt-stcar-robot": self.robot}.items()},
                 "rustc": "rustc 1.97.1 (test)", "zig": "0.15.2", "cargo_zigbuild": "0.23.4",
                 "source_sha256": {name: delivery.sha((self.root / name).read_bytes())
                                   for name in ("Cargo.lock", "Cargo.toml", "rust-toolchain.toml")}}
        self.binary.with_name("xt-stcar.build.json").write_text(json.dumps(build))
        for name in delivery.BASE_FILES - {"bin/xt-stcar", "bin/xt-stcar-robot", "elf.json", "robot-elf.json", "build.json"}:
            path = self.root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text("# Test resource\n")
        self.patches = [mock.patch.object(delivery, "ROOT", self.root),
                        mock.patch.object(delivery, "BINARY", self.binary)]
        for patch in self.patches:
            patch.start()

    def tearDown(self):
        for patch in reversed(self.patches):
            patch.stop()
        self.temp.cleanup()

    def package(self):
        with contextlib.redirect_stdout(io.StringIO()):
            delivery.package(argparse.Namespace(model=None, python=None))
        return next((self.root / "dist").glob("*.tar.gz"))

    def test_minimal_package_and_checksums(self):
        # Secrets/caches outside the allowlist must never be copied.
        (self.root / ".venv").mkdir()
        (self.root / ".venv/private-token").write_text("not a deliverable")
        archive = self.package()
        report = delivery.verify_archive(archive)
        self.assertFalse(report["manifest"]["model_included"])
        self.assertEqual(set(report["manifest"]["files"]), delivery.BASE_FILES)
        self.assertEqual(report["file_count"], len(delivery.BASE_FILES) + 2)
        self.assertFalse(report["manifest"]["runtime_requirements"]["riscv_onnxruntime_library_included"])

    def test_stale_source_or_binary_evidence_is_rejected(self):
        (self.root / "Cargo.lock").write_text("changed")
        with self.assertRaisesRegex(ValueError, "Source changed"):
            self.package()

    def test_old_glibc_build_target_requires_rebuild(self):
        path = self.binary.with_name("xt-stcar.build.json")
        build = json.loads(path.read_text())
        build["target"] = "riscv64gc-unknown-linux-gnu.2.27"
        path.write_text(json.dumps(build))
        with self.assertRaisesRegex(ValueError, "Build evidence is missing/stale"):
            self.package()

    def test_archive_build_target_must_match_even_with_valid_checksums(self):
        path = self.package()
        with tarfile.open(path, "r:gz") as original:
            members = {item.name: (item, original.extractfile(item).read()) for item in original}
        build = json.loads(members["build.json"][1])
        build["target"] = "riscv64gc-unknown-linux-gnu.2.27"
        data = json.dumps(build).encode()
        members["build.json"] = (members["build.json"][0], data)
        manifest = json.loads(members["manifest.json"][1])
        manifest["files"]["build.json"] = {"sha256": delivery.sha(data), "size_bytes": len(data)}
        members["manifest.json"] = (members["manifest.json"][0], json.dumps(manifest).encode())
        sums = "".join(f"{delivery.sha(members[name][1])}  {name}\n"
                       for name in sorted(members) if name != "SHA256SUMS").encode()
        members["SHA256SUMS"] = (members["SHA256SUMS"][0], sums)
        with tarfile.open(path, "w:gz") as rewritten:
            for item, data in members.values():
                item.size = len(data)
                rewritten.addfile(item, io.BytesIO(data))
        with self.assertRaisesRegex(ValueError, "Build target differs"):
            delivery.verify_archive(path)

    def test_archive_checksum_is_for_compressed_archive_not_a_member(self):
        archive = self.package()
        report = delivery.verify_archive(archive)
        digest = hashlib.sha256(archive.read_bytes()).hexdigest()
        self.assertEqual(report["sha256"], digest)
        self.assertEqual(report["size_bytes"], archive.stat().st_size)
        checksum = archive.with_name(archive.name + ".sha256")
        self.assertEqual(checksum.read_text(), f"{digest}  {archive.name}\n")

    def test_added_rust_source_requires_rebuild(self):
        source = self.root / "crates/fixture/src/lib.rs"
        source.parent.mkdir(parents=True)
        source.write_text("pub fn newly_added() {}\n")
        with self.assertRaisesRegex(ValueError, "Rust source set differs"):
            self.package()

    def test_second_binary_requires_matching_build_evidence(self):
        self.robot.write_bytes(self.robot.read_bytes() + b"changed\n")
        with self.assertRaisesRegex(ValueError, "missing/stale for xt-stcar-robot"):
            self.package()

    def test_second_binary_architecture_is_independently_checked(self):
        changed = bytearray(self.robot.read_bytes())
        struct.pack_into("<H", changed, 18, 183)
        self.robot.write_bytes(changed)
        with self.assertRaisesRegex(ValueError, "ELF check failed .*xt-stcar-robot"):
            self.package()

    def test_model_provenance_metadata_must_match_validation(self):
        model = b"synthetic model evidence fixture"
        fields = {key: "fixture" for key in ("contract", "ultralytics_version", "task", "head", "input", "output")}
        evidence = {**fields, "sha256": delivery.sha(model), "validated": True,
                    "metadata": {"end2end": "True", "names": "{0: 'fixture'}"},
                    "names": {"0": "fixture"},
                    "end2end": True, "class_count": 80, "opset": 17, "non_max_suppression_nodes": 0}
        delivery.check_model_evidence(model, evidence, dict(evidence))
        wrong = {**evidence, "metadata": {"end2end": "False"}}
        with self.assertRaisesRegex(ValueError, "provenance differs.*metadata"):
            delivery.check_model_evidence(model, evidence, wrong)
        with self.assertRaisesRegex(ValueError, "provenance"):
            delivery.check_model_evidence(model, evidence, {**evidence, "sha256": "wrong"})
        with self.assertRaisesRegex(ValueError, "provenance"):
            delivery.check_model_evidence(model, evidence, {**evidence, "validated": False})

    def test_no_implicit_model_and_reject_symlink_resource(self):
        resource = self.root / "scripts/onnx_worker.py"
        resource.unlink()
        resource.symlink_to(self.root / "Cargo.lock")
        with self.assertRaisesRegex(ValueError, "non-symlink"):
            self.package()

    def test_tar_path_traversal_and_symlink_are_rejected(self):
        for name, kind in (("../outside", tarfile.REGTYPE), ("bin/xt-stcar", tarfile.SYMTYPE)):
            path = self.root / "bad.tar.gz"
            with tarfile.open(path, "w:gz") as archive:
                item = tarfile.TarInfo(name)
                item.type = kind
                item.linkname = "/etc/passwd" if kind == tarfile.SYMTYPE else ""
                archive.addfile(item, io.BytesIO())
            with self.subTest(name=name), self.assertRaisesRegex(ValueError, "Unsafe archive"):
                delivery.verify_archive(path)

    def test_archive_content_tampering_is_rejected(self):
        path = self.package()
        with tarfile.open(path, "r:gz") as original:
            members = [(item, original.extractfile(item).read()) for item in original]
        with tarfile.open(path, "w:gz") as rewritten:
            for item, data in members:
                if item.name == "scripts/onnx_worker.py":
                    data += b"changed\n"
                    item.size = len(data)
                rewritten.addfile(item, io.BytesIO(data))
        with self.assertRaisesRegex(ValueError, "checksum/size mismatch"):
            delivery.verify_archive(path)

    def args(self, **updates):
        values = {"archive": self.package(), "host": "192.0.2.10", "user": "vehicle",
                  "remote_dir": "/home/vehicle/xt-stcar/releases/local-test", "port": 22, "execute": False}
        values.update(updates)
        return argparse.Namespace(**values)

    def test_dry_run_never_starts_any_subprocess(self):
        args = self.args()
        with mock.patch.object(delivery.subprocess, "run", side_effect=AssertionError("network forbidden")):
            with contextlib.redirect_stdout(io.StringIO()) as output:
                delivery.upload(args)
        self.assertTrue(json.loads(output.getvalue())["dry_run"])

    def test_missing_connection_info_and_injection_rejected_before_network(self):
        args = self.args()
        cases = [{"host": None}, {"host": "vehicle.local"}, {"host": "192.0.2.10;id"},
                 {"user": "root"}, {"user": "vehicle;id"}, {"remote_dir": "/opt/vendor"},
                 {"remote_dir": "/home/vehicle/xt-stcar/releases/../vendor"},
                 {"remote_dir": "/home/vehicle/xt-stcar/releases/name$(id)"}]
        for updates in cases:
            case = argparse.Namespace(**(vars(args) | updates))
            with self.subTest(updates=updates), mock.patch.object(delivery.subprocess, "run", side_effect=AssertionError("network forbidden")):
                with self.assertRaises(ValueError):
                    delivery.upload(case)

    def test_remote_creation_refuses_existing_release_and_symlink_parent(self):
        home = self.root / "home"
        home.mkdir()
        remote = home / "xt-stcar/releases/new-release"
        def run_creation():
            with mock.patch.object(Path, "home", return_value=home), mock.patch.object(sys, "argv", ["remote", str(remote)]):
                with contextlib.redirect_stdout(io.StringIO()):
                    exec(delivery.REMOTE_MKDIR, {})
        run_creation()
        with self.assertRaises(FileExistsError):
            run_creation()
        remote.rmdir()
        remote.parent.rmdir()
        outside = self.root / "outside"
        outside.mkdir()
        remote.parent.symlink_to(outside, target_is_directory=True)
        with self.assertRaises(SystemExit):
            run_creation()
        self.assertFalse((outside / "new-release").exists())


if __name__ == "__main__":
    unittest.main()
