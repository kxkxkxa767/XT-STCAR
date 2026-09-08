"""Local-only artifact boundary tests. No target program, SSH or SCP is executed."""

import argparse
import contextlib
import hashlib
import io
import json
import os
from pathlib import Path
import stat
import struct
import subprocess
import sys
import tarfile
import tempfile
import unittest
from unittest import mock
from types import SimpleNamespace

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


def elf_sections(data):
    offset = struct.unpack_from("<Q", data, 40)[0]
    count = struct.unpack_from("<H", data, 60)[0]
    return [(offset + index * 64, struct.unpack_from("<IIQQQQIIQQ", data, offset + index * 64))
            for index in range(count)]


def elf_programs(data):
    offset = struct.unpack_from("<Q", data, 32)[0]
    count = struct.unpack_from("<H", data, 56)[0]
    return [(offset + index * 56, struct.unpack_from("<IIQQQQQQ", data, offset + index * 56))
            for index in range(count)]


def section_string_table_bypass(original):
    """Keep a benign section view while the loaded DT_STRTAB requires 2.39."""
    data = bytearray(original)
    sections = elf_sections(data)
    _, verneed = next(entry for entry in sections if entry[1][1] == 0x6FFFFFFE)
    header, strings = sections[verneed[6]]
    old_strings = bytes(data[strings[4]:strings[4] + strings[5]])
    highest = inspect_elf.inspect_bytes(original)["maximum_glibc"]
    name = f"GLIBC_{highest}\0".encode()
    if len(name) != len(b"GLIBC_2.39\0"):
        raise AssertionError("Fixture GLIBC version must use equal-length replacement")
    position = strings[4] + old_strings.index(name)
    data[position:position + len(name)] = b"GLIBC_2.39\0"
    old_strings_offset = len(data)
    data.extend(old_strings)
    struct.pack_into("<Q", data, header + 24, old_strings_offset)
    return bytes(data)


class ElfTests(unittest.TestCase):
    def test_cli_report_rejects_input_path_and_hardlink_aliases(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            binary = directory / "input.elf"
            original = target_fixture()
            binary.write_bytes(original)
            hardlink = directory / "alias.elf"
            os.link(binary, hardlink)
            (directory / "nested").mkdir()
            for output in (binary, directory / "nested/../input.elf", hardlink):
                with self.subTest(output=str(output)):
                    result = subprocess.run([sys.executable, str(ROOT / "scripts/inspect_elf.py"),
                                             str(binary), "--output", str(output)],
                                            capture_output=True, text=True, timeout=5)
                    self.assertNotEqual(result.returncode, 0)
                    self.assertFalse(json.loads(result.stdout)["passed"])
                    self.assertEqual(binary.read_bytes(), original)
                    self.assertEqual(hardlink.read_bytes(), original)

    @unittest.skipUnless(hasattr(os, "mkfifo"), "POSIX output type tests")
    def test_cli_report_rejects_fifo_directory_and_symlink_outputs(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            binary = directory / "input.elf"
            original = target_fixture()
            binary.write_bytes(original)
            fifo = directory / "report.fifo"
            os.mkfifo(fifo)
            old_report = directory / "old.json"
            old_report.write_text("previous report\n")
            symlink = directory / "report-link.json"
            symlink.symlink_to(old_report)
            for output in (fifo, directory, symlink):
                with self.subTest(output=str(output)):
                    result = subprocess.run([sys.executable, str(ROOT / "scripts/inspect_elf.py"),
                                             str(binary), "--output", str(output)],
                                            capture_output=True, text=True, timeout=5)
                    self.assertNotEqual(result.returncode, 0)
                    self.assertIn("regular non-symlink", json.loads(result.stdout)["errors"][0])
                    self.assertEqual(binary.read_bytes(), original)
                    self.assertEqual(old_report.read_text(), "previous report\n")
            self.assertTrue(stat.S_ISFIFO(fifo.lstat().st_mode))

    def test_cli_report_commits_success_and_preserves_existing_report_on_bad_elf(self):
        with tempfile.TemporaryDirectory() as temp:
            binary, output = Path(temp) / "input.elf", Path(temp) / "report.json"
            original = target_fixture()
            binary.write_bytes(original)
            output.write_text("previous report\n")
            command = [sys.executable, str(ROOT / "scripts/inspect_elf.py"), str(binary), "--output", str(output)]
            result = subprocess.run(command, capture_output=True, text=True, timeout=5)
            self.assertEqual(result.returncode, 0, result.stderr)
            saved = output.read_bytes()
            self.assertTrue(json.loads(saved)["passed"])
            self.assertEqual(json.loads(saved), json.loads(result.stdout))
            self.assertEqual(binary.read_bytes(), original)
            # An ABI check failure returns diagnostics without replacing the last passed report.
            changed = bytearray(original)
            struct.pack_into("<H", changed, 18, 183)
            binary.write_bytes(changed)
            result = subprocess.run(command, capture_output=True, text=True, timeout=5)
            self.assertNotEqual(result.returncode, 0)
            self.assertFalse(json.loads(result.stdout)["passed"])
            self.assertEqual(output.read_bytes(), saved)
            self.assertEqual(binary.read_bytes(), changed)

    def test_failed_atomic_report_commit_preserves_prior_file_and_cleans_temp(self):
        with tempfile.TemporaryDirectory() as temp:
            binary, output = Path(temp) / "input.elf", Path(temp) / "report.json"
            original = target_fixture()
            binary.write_bytes(original)
            output.write_text("previous report\n")
            with mock.patch.object(sys, "argv", ["inspect_elf", str(binary), "--output", str(output)]), \
                    mock.patch.object(inspect_elf.os, "replace", side_effect=OSError("synthetic rename failure")), \
                    contextlib.redirect_stdout(io.StringIO()) as printed:
                status = inspect_elf.main()
            self.assertEqual(status, 1)
            self.assertIn("synthetic rename failure", json.loads(printed.getvalue())["errors"][0])
            self.assertEqual(binary.read_bytes(), original)
            self.assertEqual(output.read_text(), "previous report\n")
            self.assertEqual(list(Path(temp).glob(".elf-report-*")), [])

    @unittest.skipUnless(hasattr(os, "mkfifo"), "POSIX FIFO test")
    def test_fifo_is_rejected_without_waiting_for_a_writer(self):
        with tempfile.TemporaryDirectory() as temp:
            fifo = Path(temp) / "input.fifo"
            os.mkfifo(fifo)
            output = subprocess.run([sys.executable, str(ROOT / "scripts/inspect_elf.py"), str(fifo)],
                                    capture_output=True, text=True, timeout=5)
            self.assertNotEqual(output.returncode, 0)
            report = json.loads(output.stdout)
            self.assertFalse(report["passed"])
            self.assertIn("regular non-symlink", report["errors"][0])

    def test_elf_size_limit_checks_stat_and_actual_read_length(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "oversized.elf"
            with path.open("wb") as handle:
                handle.truncate(inspect_elf.MAX_ELF_BYTES + 1)
            output = subprocess.run([sys.executable, str(ROOT / "scripts/inspect_elf.py"), str(path)],
                                    capture_output=True, text=True, timeout=5)
            self.assertNotEqual(output.returncode, 0)
            self.assertIn("size limit", json.loads(output.stdout)["errors"][0])
            # Simulate growth after a small fstat without allocating 64 MiB.
            path.write_bytes(b"x" * 129)
            with mock.patch.object(inspect_elf, "MAX_ELF_BYTES", 128), \
                    mock.patch.object(inspect_elf.os, "fstat", return_value=SimpleNamespace(st_mode=stat.S_IFREG, st_size=1)):
                with self.assertRaisesRegex(inspect_elf.ElfError, "size limit"):
                    inspect_elf.inspect(path)

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

    def test_section_only_string_table_cannot_hide_loaded_glibc_239(self):
        with self.assertRaisesRegex(inspect_elf.ElfError, "dynamic LOAD mapping"):
            inspect_elf.inspect_bytes(section_string_table_bypass(target_fixture()))

    def test_truncated_section_sizes_cannot_hide_dynamic_symbols(self):
        data = bytearray(target_fixture())
        sections = elf_sections(data)
        sym_header, symbols = next(entry for entry in sections if entry[1][1] == 11)
        ver_header, versions = next(entry for entry in sections if entry[1][1] == 0x6FFFFFFF)
        struct.pack_into("<Q", data, sym_header + 32, symbols[5] - 24)
        struct.pack_into("<Q", data, ver_header + 32, versions[5] - 2)
        with self.assertRaisesRegex(inspect_elf.ElfError, "Dynamic symbol table section differs"):
            inspect_elf.inspect_bytes(data)

    def test_dynamic_version_and_symbol_pointers_must_match_sections(self):
        original = target_fixture()
        _, dynamic = next(entry for entry in elf_programs(original) if entry[1][0] == 2)
        entries = {struct.unpack_from("<q", original, offset)[0]: offset
                   for offset in range(dynamic[2], dynamic[2] + dynamic[5], 16)}
        for tag in (6, 0x6FFFFFF0, 0x6FFFFFFE):
            with self.subTest(tag=hex(tag)):
                data = bytearray(original)
                position = entries[tag] + 8
                address = struct.unpack_from("<Q", data, position)[0]
                struct.pack_into("<Q", data, position, address + 8)
                with self.assertRaisesRegex(inspect_elf.ElfError, "dynamic LOAD mapping"):
                    inspect_elf.inspect_bytes(data)

    def test_loaded_dynamic_table_cannot_be_replaced_by_file_only_copy(self):
        data = bytearray(target_fixture())
        header, dynamic = next(entry for entry in elf_programs(data) if entry[1][0] == 2)
        copy = data[dynamic[2]:dynamic[2] + dynamic[5]]
        new_offset = len(data)
        data.extend(copy)
        struct.pack_into("<Q", data, header + 8, new_offset)
        with self.assertRaisesRegex(inspect_elf.ElfError, "PT_DYNAMIC differs"):
            inspect_elf.inspect_bytes(data)

    def test_even_identical_overlapping_load_mappings_are_rejected(self):
        data = bytearray(target_fixture())
        programs = elf_programs(data)
        header, _ = next(entry for entry in programs if entry[1][0] not in (1, 2, 3))
        _, load = next(entry for entry in programs if entry[1][0] == 1)
        struct.pack_into("<IIQQQQQQ", data, header, *load)
        with self.assertRaisesRegex(inspect_elf.ElfError, "Overlapping LOAD"):
            inspect_elf.inspect_bytes(data)

    def test_sysv_and_gnu_dynamic_hash_symbol_counts_are_supported(self):
        original = target_fixture()
        _, dynamic = next(entry for entry in elf_programs(original) if entry[1][0] == 2)
        for removed in (4, 0x6FFFFEF5):
            data = bytearray(original)
            for offset in range(dynamic[2], dynamic[2] + dynamic[5], 16):
                if struct.unpack_from("<q", data, offset)[0] == removed:
                    struct.pack_into("<q", data, offset, 21)  # Harmless DT_DEBUG entry.
                    break
            else:
                self.fail("Target fixture must have both dynamic hash styles")
            with self.subTest(removed=hex(removed)):
                self.assertTrue(inspect_elf.inspect_bytes(data)["passed"])

    def test_verneed_remains_a_requirement_without_a_symbol_reference(self):
        original = target_fixture()
        data = bytearray(original)
        highest = inspect_elf.inspect_bytes(original)["maximum_glibc"]
        data = data.replace(f"GLIBC_{highest}\0".encode(), b"GLIBC_2.39\0")
        _, version_symbols = next(entry for entry in elf_sections(data) if entry[1][1] == 0x6FFFFFFF)
        # Erase every symbol's version index; the loaded verneed table survives.
        for offset in range(version_symbols[4], version_symbols[4] + version_symbols[5], 2):
            struct.pack_into("<H", data, offset, 1)
        report = inspect_elf.inspect_bytes(data)
        self.assertEqual(report["versioned_undefined_symbols"], [])
        self.assertEqual(report["maximum_glibc"], "2.39")
        self.assertFalse(report["passed"])
        self.assertTrue(any("2.38 baseline" in error for error in report["errors"]))

    def test_verneed_count_cannot_hide_an_unterminated_auxiliary_chain(self):
        data = bytearray(target_fixture())
        _, requirement = next(entry for entry in elf_sections(data) if entry[1][1] == 0x6FFFFFFE)
        self.assertGreater(struct.unpack_from("<H", data, requirement[4] + 2)[0], 1)
        struct.pack_into("<H", data, requirement[4] + 2, 1)
        with self.assertRaisesRegex(inspect_elf.ElfError, "exceeds its declared count"):
            inspect_elf.inspect_bytes(data)


class DeliveryTests(unittest.TestCase):
    def setUp(self):
        fixture = target_fixture()  # Skip before allocating resources if no target was built.
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.binary = self.root / "target/riscv64gc-unknown-linux-gnu/release/xt-stcar"
        self.binary.parent.mkdir(parents=True)
        self.binary.write_bytes(fixture)
        self.robot = self.binary.with_name("xt-stcar-robot")
        self.robot.write_bytes(fixture + b"robot fixture\n")
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

    def test_rehashed_archive_still_rejects_section_only_elf_version_bypass(self):
        path = self.package()
        with tarfile.open(path, "r:gz") as archive:
            members = {item.name: (item, archive.extractfile(item).read()) for item in archive}
        disguised = section_string_table_bypass(members["bin/xt-stcar"][1])
        members["bin/xt-stcar"] = (members["bin/xt-stcar"][0], disguised)
        build = json.loads(members["build.json"][1])
        build["binary_sha256"] = delivery.sha(disguised)
        build["binaries"]["xt-stcar"]["sha256"] = delivery.sha(disguised)
        members["build.json"] = (members["build.json"][0], json.dumps(build).encode())
        elf = json.loads(members["elf.json"][1])
        elf["sha256"] = delivery.sha(disguised)
        members["elf.json"] = (members["elf.json"][0], json.dumps(elf).encode())
        manifest = json.loads(members["manifest.json"][1])
        for name, (_, data) in members.items():
            if name in manifest["files"]:
                manifest["files"][name] = {"sha256": delivery.sha(data), "size_bytes": len(data)}
        members["manifest.json"] = (members["manifest.json"][0], json.dumps(manifest).encode())
        sums = "".join(f"{delivery.sha(members[name][1])}  {name}\n"
                       for name in sorted(members) if name != "SHA256SUMS").encode()
        members["SHA256SUMS"] = (members["SHA256SUMS"][0], sums)
        with tarfile.open(path, "w:gz") as archive:
            for item, data in members.values():
                item.size = len(data)
                archive.addfile(item, io.BytesIO(data))
        with self.assertRaisesRegex(inspect_elf.ElfError, "dynamic LOAD mapping"):
            delivery.verify_archive(path)

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


class FreshCheckoutTests(unittest.TestCase):
    def test_missing_target_and_tmp_directory_skip_cleanly(self):
        script = """import json,pathlib,sys,unittest
sys.path.insert(0, sys.argv[1])
import test_delivery
test_delivery.ROOT = pathlib.Path(sys.argv[2])
result = unittest.TestResult()
test_delivery.DeliveryTests('test_minimal_package_and_checksums').run(result)
print(json.dumps({'skipped':len(result.skipped),'errors':len(result.errors),'failures':len(result.failures)}))
"""
        with tempfile.TemporaryDirectory() as fresh:
            output = subprocess.run([sys.executable, "-c", script, str(ROOT / "tests"), fresh],
                                    capture_output=True, text=True, timeout=5)
            self.assertEqual(output.returncode, 0, output.stderr)
            self.assertEqual(json.loads(output.stdout), {"skipped": 1, "errors": 0, "failures": 0})
            self.assertFalse((Path(fresh) / "tmp").exists())


if __name__ == "__main__":
    unittest.main()
