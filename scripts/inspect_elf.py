#!/usr/bin/env python3
"""Inspect ELF tables directly; never infer required GLIBC versions with strings.

ELF layout: https://www.sco.com/developers/gabi/latest/contents.html
RISC-V flags: https://riscv-non-isa.github.io/riscv-elf-psabi-doc/
GNU version tables: glibc elf/elf.h and elf/dl-version.c.
https://gabi.xinuos.com/elf/08-dynamic.html
https://github.com/bminor/glibc/blob/glibc-2.38/elf/dl-version.c

Only the project's supported ELF layout is accepted. This is not a general ELF
loader, instruction-set validator, or proof that the program runs on the car.
ELF inputs are limited to 64 MiB; path inspection requires POSIX nonblocking open.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import struct
import sys
import tempfile


GLIBC_BASELINE = "2.38"
MAX_ELF_BYTES = 64 * 1024 * 1024
LOADER = "/lib/ld-linux-riscv64-lp64d.so.1"
ALLOWED_NEEDED = {
    "libc.so.6", "libpthread.so.0", "libdl.so.2", "libm.so.6", "librt.so.1",
    "libgcc_s.so.1", "ld-linux-riscv64-lp64d.so.1",
}


class ElfError(ValueError):
    """Invalid or unsupported ELF layout."""


class Elf:
    def __init__(self, data):
        self.data = data

    def take(self, offset, size):
        if offset < 0 or size < 0 or offset + size > len(self.data):
            raise ElfError("ELF table is outside file bounds")
        return self.data[offset:offset + size]

    def unpack(self, fmt, offset):
        return struct.unpack(fmt, self.take(offset, struct.calcsize(fmt)))

    def string(self, table, offset):
        if not 0 <= offset < len(table):
            raise ElfError("Invalid ELF string offset")
        end = table.find(b"\0", offset)
        if end < 0:
            raise ElfError("Unterminated ELF string")
        return table[offset:end].decode("utf-8", errors="strict")

    def inspect(self):
        ident = self.take(0, 16)
        if ident[:7] != b"\x7fELF\x02\x01\x01":
            raise ElfError("Expected ELF64 little-endian version 1")
        (etype, machine, version, entry, phoff, shoff, flags, ehsize, phsize,
         phnum, shsize, shnum, _) = self.unpack("<HHIQQQIHHHHHH", 16)
        if version != 1 or ehsize != 64 or phsize != 56 or shsize != 64:
            raise ElfError("Unsupported ELF header/table sizes")
        if not phnum or not shnum:
            raise ElfError("Program and section tables are required")
        programs = [self.unpack("<IIQQQQQQ", phoff + i * phsize) for i in range(phnum)]
        sections = [self.unpack("<IIQQQQIIQQ", shoff + i * shsize) for i in range(shnum)]
        loads = sorted((p for p in programs if p[0] == 1), key=lambda p: p[3])
        for index, load in enumerate(loads):
            if load[5] > load[6]:
                raise ElfError("LOAD file size exceeds memory size")
            self.take(load[2], load[5])
            if index and load[3] < loads[index - 1][3] + loads[index - 1][6]:
                raise ElfError("Overlapping LOAD segments are not supported")

        def file_offset(address, size):
            offsets = []
            for ph in loads:
                if ph[3] <= address and address + size <= ph[3] + ph[5]:
                    offsets.append(ph[2] + address - ph[3])
            if len(offsets) != 1:
                raise ElfError("Dynamic virtual address must have one unambiguous LOAD mapping")
            offset = offsets.pop()
            self.take(offset, size)
            return offset

        def at_address(address, size):
            return self.take(file_offset(address, size), size)

        def mapped_section(section, address, size, label):
            # Section headers are not the loader's source of truth. Refuse an
            # alternate file-only table even if its contents look well formed.
            if (section[3] != address or section[5] != size or not section[2] & 2
                    or section[4] != file_offset(address, size)):
                raise ElfError(f"{label} section differs from the dynamic LOAD mapping")

        interps = [self.take(p[2], p[5]) for p in programs if p[0] == 3]
        if len(interps) != 1 or not interps[0].endswith(b"\0"):
            raise ElfError("Exactly one terminated PT_INTERP is required")
        loader = interps[0][:-1].decode("ascii")
        dynamics = [p for p in programs if p[0] == 2]
        if len(dynamics) != 1:
            raise ElfError("Exactly one PT_DYNAMIC is required")
        dynamic = {}
        ph = dynamics[0]
        if ph[5] % 16 or ph[2] != file_offset(ph[3], ph[5]):
            raise ElfError("PT_DYNAMIC differs from its LOAD mapping")
        terminated = False
        for offset in range(ph[2], ph[2] + ph[5], 16):
            tag, value = self.unpack("<qQ", offset)
            if tag == 0:
                terminated = True
                break
            dynamic.setdefault(tag, []).append(value)
        if not terminated:
            raise ElfError("Unterminated dynamic table")
        def singleton(tag, label):
            if len(dynamic.get(tag, [])) != 1:
                raise ElfError(f"Missing or duplicate dynamic {label}")
            return dynamic[tag][0]

        str_address = singleton(5, "string table")
        str_size = singleton(10, "string table size")
        sym_address = singleton(6, "symbol table")
        if singleton(11, "symbol entry size") != 24:
            raise ElfError("Invalid dynamic symbol entry size")
        versym_address = singleton(0x6FFFFFF0, "symbol versions")
        verneed_address = singleton(0x6FFFFFFE, "version requirements")
        verneed_count = singleton(0x6FFFFFFF, "version requirement count")
        dynstr = at_address(str_address, str_size)
        needed = [self.string(dynstr, value) for value in dynamic.get(1, [])]

        # Determine the symbol count from loaded hash tables, not a mutable
        # section size which could hide later versioned undefined symbols.
        symbol_counts = []
        if 4 in dynamic:  # DT_HASH, ELF64 still uses 32-bit hash words.
            address = singleton(4, "SysV hash table")
            buckets, chains = struct.unpack("<II", at_address(address, 8))
            if not buckets or not chains:
                raise ElfError("Invalid SysV dynamic hash dimensions")
            at_address(address, 8 + 4 * (buckets + chains))
            symbol_counts.append(chains)
        if 0x6FFFFEF5 in dynamic:  # DT_GNU_HASH
            address = singleton(0x6FFFFEF5, "GNU hash table")
            buckets, symbol_offset, bloom_size, _ = struct.unpack("<IIII", at_address(address, 16))
            if not buckets or not bloom_size:
                raise ElfError("Invalid GNU dynamic hash dimensions")
            bucket_address = address + 16 + bloom_size * 8
            bucket_data = at_address(bucket_address, buckets * 4)
            starts = [entry[0] for entry in struct.iter_unpack("<I", bucket_data)]
            if any(start and start < symbol_offset for start in starts):
                raise ElfError("Invalid GNU dynamic hash bucket")
            last = max(starts)
            if last:
                chain_address = bucket_address + buckets * 4
                while True:
                    (value,) = struct.unpack("<I", at_address(chain_address + 4 * (last - symbol_offset), 4))
                    last += 1
                    if value & 1:
                        break
                symbol_counts.append(last)
            else:
                symbol_counts.append(symbol_offset)
        if not symbol_counts or not symbol_counts[0] or len(set(symbol_counts)) != 1:
            raise ElfError("Missing or inconsistent dynamic symbol counts")
        symbol_count = symbol_counts[0]

        versions = {}
        verneeds = [s for s in sections if s[1] == 0x6FFFFFFE]
        if len(verneeds) != 1:
            raise ElfError("Exactly one GNU version requirement table is required")
        vn = verneeds[0]
        mapped_section(vn, verneed_address, vn[5], "GNU verneed")
        if not verneed_count or vn[7] != verneed_count:
            raise ElfError("GNU verneed count differs from DT_VERNEEDNUM")
        if vn[6] >= len(sections) or sections[vn[6]][1] != 3:
            raise ElfError("Invalid GNU verneed string table link")
        mapped_section(sections[vn[6]], str_address, str_size, "GNU verneed string table")
        vnstr = dynstr
        cursor = vn[4]
        visited = set()
        while True:
            if cursor in visited or not vn[4] <= cursor <= vn[4] + vn[5] - 16:
                raise ElfError("Invalid GNU verneed chain")
            visited.add(cursor)
            vn_version, count, library_offset, aux_offset, next_offset = self.unpack("<HHIII", cursor)
            if vn_version != 1 or not count:
                raise ElfError("Unsupported GNU verneed record")
            library = self.string(vnstr, library_offset)
            if library not in needed:
                raise ElfError("GNU version requirement library is absent from DT_NEEDED")
            aux = cursor + aux_offset
            aux_visited = set()
            for index in range(count):
                if aux in aux_visited or not vn[4] <= aux <= vn[4] + vn[5] - 16:
                    raise ElfError("Invalid GNU vernaux chain")
                aux_visited.add(aux)
                _, _, other, name, next_aux = self.unpack("<IHHII", aux)
                version_index = other & 0x7FFF
                if version_index in versions or version_index < 2:
                    raise ElfError("Duplicate or invalid GNU version index")
                versions[version_index] = {"library": library, "version": self.string(vnstr, name)}
                if index + 1 < count and not next_aux:
                    raise ElfError("Truncated GNU vernaux chain")
                if index + 1 == count and next_aux:
                    raise ElfError("GNU vernaux chain exceeds its declared count")
                aux += next_aux
            if not next_offset:
                break
            cursor += next_offset
        if len(visited) != verneed_count:
            raise ElfError("GNU verneed chain differs from DT_VERNEEDNUM")

        dynsyms = [(i, s) for i, s in enumerate(sections) if s[1] == 11]
        if len(dynsyms) != 1:
            raise ElfError("Exactly one dynamic symbol table is required")
        symbol_index, symbols = dynsyms[0]
        if symbols[9] != 24 or symbols[6] >= len(sections) or sections[symbols[6]][1] != 3:
            raise ElfError("Invalid dynamic symbol table")
        mapped_section(symbols, sym_address, symbol_count * 24, "Dynamic symbol table")
        mapped_section(sections[symbols[6]], str_address, str_size, "Dynamic symbol string table")
        symstr = dynstr
        versyms = [s for s in sections if s[1] == 0x6FFFFFFF and s[6] == symbol_index]
        if len(versyms) != 1:
            raise ElfError("Invalid dynamic symbol version table")
        mapped_section(versyms[0], versym_address, symbol_count * 2, "Dynamic symbol version table")
        references = []
        for i in range(symbols[5] // 24):
            name, _, _, shndx, _, _ = self.unpack("<IBBHQQ", symbols[4] + i * 24)
            (index,) = self.unpack("<H", versyms[0][4] + i * 2)
            index &= 0x7FFF
            if shndx == 0 and index > 1:
                if index not in versions:
                    raise ElfError("Undefined symbol refers to absent GNU version")
                references.append({"symbol": self.string(symstr, name), **versions[index]})
        # The loader checks DT_VERNEED, including entries with no remaining
        # undefined symbol reference. Keep the symbol list as separate evidence.
        requirements = sorted(versions.values(), key=lambda r: (r["library"], r["version"]))
        glibc_versions = sorted({r["version"] for r in requirements if r["version"].startswith("GLIBC_")})
        numeric_versions = []
        for value in glibc_versions:
            if not re.fullmatch(r"GLIBC_[0-9]+(?:\.[0-9]+)+", value):
                raise ElfError(f"Unsupported nonnumeric GLIBC requirement: {value}")
            numeric_versions.append(tuple(map(int, value[6:].split("."))))
        maximum = max(numeric_versions, default=())
        return {
            "elf_class": 64, "endianness": "little", "machine": machine,
            "architecture": "riscv64" if machine == 243 else f"ELF machine {machine}",
            "elf_type": etype, "entry": entry, "flags": flags,
            "rvc": bool(flags & 1), "abi": "lp64d" if flags & 6 == 4 else "unsupported",
            "pie": etype == 3 and bool(dynamic.get(0x6FFFFFFB, [0])[0] & 0x08000000),
            "loader": loader, "needed": needed,
            "rpath": [self.string(dynstr, v) for tag in (15, 29) for v in dynamic.get(tag, [])],
            "glibc_versions": glibc_versions,
            "maximum_glibc": ".".join(map(str, maximum)) if maximum else None,
            "version_requirements": requirements,
            "versioned_undefined_symbols": sorted(references, key=lambda r: (r["library"], r["symbol"])),
        }


def inspect_bytes(data, name="xt-stcar"):
    if len(data) > MAX_ELF_BYTES:
        raise ElfError("ELF input exceeds the 64 MiB size limit")
    report = Elf(data).inspect()
    errors = []
    for condition, error in (
        (report["machine"] == 243, "ELF machine must be RISC-V (243)"),
        (report["flags"] == 5, "Expected RVC + double float ABI flags (0x5) only"),
        (report["rvc"] and report["abi"] == "lp64d", "Expected RVC / LP64D"),
        (report["pie"] and report["entry"] != 0, "Expected PIE executable with a nonzero entry"),
        (report["loader"] == LOADER, f"Expected loader {LOADER}"),
        ("libc.so.6" in report["needed"], "Expected dynamic libc dependency"),
        (not set(report["needed"]) - ALLOWED_NEEDED, "Unreviewed dynamic library dependency"),
        (not report["rpath"], "RPATH/RUNPATH is not permitted in this deployment package"),
        (bool(report["glibc_versions"]), "No GNU GLIBC version requirements found"),
    ):
        if not condition:
            errors.append(error)
    if report["maximum_glibc"] and tuple(map(int, report["maximum_glibc"].split("."))) > tuple(map(int, GLIBC_BASELINE.split("."))):
        errors.append(f"Required GLIBC exceeds the locked {GLIBC_BASELINE} baseline")
    report.update(schema_version=1, glibc_baseline=GLIBC_BASELINE, file=name, sha256=hashlib.sha256(data).hexdigest(),
                  size_bytes=len(data), passed=not errors, errors=errors)
    return report


def inspect(path):
    if not hasattr(os, "O_NONBLOCK") or not hasattr(os, "O_NOFOLLOW"):
        raise ElfError("Path inspection requires POSIX nonblocking, no-follow open")
    flags = os.O_RDONLY | os.O_NONBLOCK | os.O_NOFOLLOW | getattr(os, "O_CLOEXEC", 0)
    descriptor = os.open(path, flags)
    with os.fdopen(descriptor, "rb") as handle:
        info = os.fstat(handle.fileno())
        if not stat.S_ISREG(info.st_mode):
            raise ElfError("ELF input must be a regular non-symlink file")
        if info.st_size > MAX_ELF_BYTES:
            raise ElfError("ELF input exceeds the 64 MiB size limit")
        # Check the actual bytes too: a regular file may grow after fstat.
        data = handle.read(MAX_ELF_BYTES + 1)
    return inspect_bytes(data, Path(path).name)


def checked_output(binary, output):
    """Validate without opening an output stream (a FIFO must never block)."""
    binary, output = Path(binary), Path(output)
    try:
        info = output.lstat()
    except FileNotFoundError:
        info = None
    if info is not None and not stat.S_ISREG(info.st_mode):
        raise ElfError("ELF report output must be a regular non-symlink file")
    if binary.resolve() == output.resolve():
        raise ElfError("ELF report output must not overwrite the input binary")
    if info is not None and os.path.samestat(binary.stat(), info):
        raise ElfError("ELF report output must not alias the input binary inode")
    parent = output.parent.resolve(strict=True)
    if not parent.is_dir():
        raise ElfError("ELF report parent must be a directory")
    return parent / output.name


def write_report(binary, output, rendered):
    output = checked_output(binary, output)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=output.parent,
                                         prefix=".elf-report-", delete=False) as handle:
            temporary = Path(handle.name)
            handle.write(rendered)
            handle.flush()
            os.fsync(handle.fileno())
        # Recheck before committing; never truncate an existing output inode.
        checked_output(binary, output)
        os.replace(temporary, output)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("binary", type=Path)
    parser.add_argument("--output", type=Path, help="Atomically save a passed JSON report; preserve it on failure")
    args = parser.parse_args()
    try:
        if args.output:
            checked_output(args.binary, args.output)
        report = inspect(args.binary)
        if args.output and report["passed"]:
            write_report(args.binary, args.output, json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    except (OSError, ValueError, RuntimeError, struct.error) as exc:
        report = {"schema_version": 1, "file": str(args.binary), "passed": False, "errors": [str(exc)]}
    rendered = json.dumps(report, indent=2, ensure_ascii=False) + "\n"
    print(rendered, end="")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
