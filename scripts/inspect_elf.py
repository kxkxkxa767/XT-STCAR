#!/usr/bin/env python3
"""Inspect ELF tables directly; never infer required GLIBC versions with strings.

ELF layout: https://www.sco.com/developers/gabi/latest/contents.html
RISC-V flags: https://riscv-non-isa.github.io/riscv-elf-psabi-doc/
GNU version tables: glibc elf/elf.h and elf/dl-version.c.
"""

import argparse
import hashlib
import json
from pathlib import Path
import re
import struct
import sys


GLIBC_BASELINE = "2.38"
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

        def section_data(section):
            return self.take(section[4], section[5])

        def at_address(address, size):
            for ph in programs:
                if ph[0] == 1 and ph[3] <= address and address + size <= ph[3] + ph[5]:
                    return self.take(ph[2] + address - ph[3], size)
            raise ElfError("Dynamic virtual address is not backed by a LOAD segment")

        interps = [self.take(p[2], p[5]) for p in programs if p[0] == 3]
        if len(interps) != 1 or not interps[0].endswith(b"\0"):
            raise ElfError("Exactly one terminated PT_INTERP is required")
        loader = interps[0][:-1].decode("ascii")
        dynamics = [p for p in programs if p[0] == 2]
        if len(dynamics) != 1:
            raise ElfError("Exactly one PT_DYNAMIC is required")
        dynamic = {}
        ph = dynamics[0]
        terminated = False
        for offset in range(ph[2], ph[2] + ph[5], 16):
            tag, value = self.unpack("<qQ", offset)
            if tag == 0:
                terminated = True
                break
            dynamic.setdefault(tag, []).append(value)
        if not terminated:
            raise ElfError("Unterminated dynamic table")
        for tag in (5, 10):
            if len(dynamic.get(tag, [])) != 1:
                raise ElfError("Missing or duplicate dynamic string table")
        dynstr = at_address(dynamic[5][0], dynamic[10][0])
        needed = [self.string(dynstr, value) for value in dynamic.get(1, [])]

        versions = {}
        verneeds = [s for s in sections if s[1] == 0x6FFFFFFE]
        if len(verneeds) != 1:
            raise ElfError("Exactly one GNU version requirement table is required")
        vn = verneeds[0]
        if vn[6] >= len(sections):
            raise ElfError("Invalid GNU verneed string table link")
        vnstr = section_data(sections[vn[6]])
        cursor = vn[4]
        visited = set()
        while True:
            if cursor in visited or not vn[4] <= cursor <= vn[4] + vn[5] - 16:
                raise ElfError("Invalid GNU verneed chain")
            visited.add(cursor)
            vn_version, count, file_offset, aux_offset, next_offset = self.unpack("<HHIII", cursor)
            if vn_version != 1 or not count:
                raise ElfError("Unsupported GNU verneed record")
            library = self.string(vnstr, file_offset)
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
                aux += next_aux
            if not next_offset:
                break
            cursor += next_offset

        dynsyms = [(i, s) for i, s in enumerate(sections) if s[1] == 11]
        if len(dynsyms) != 1:
            raise ElfError("Exactly one dynamic symbol table is required")
        symbol_index, symbols = dynsyms[0]
        if symbols[9] != 24 or symbols[5] % 24 or symbols[6] >= len(sections):
            raise ElfError("Invalid dynamic symbol table")
        symstr = section_data(sections[symbols[6]])
        versyms = [s for s in sections if s[1] == 0x6FFFFFFF and s[6] == symbol_index]
        if len(versyms) != 1 or versyms[0][5] != (symbols[5] // 24) * 2:
            raise ElfError("Invalid dynamic symbol version table")
        references = []
        for i in range(symbols[5] // 24):
            name, _, _, shndx, _, _ = self.unpack("<IBBHQQ", symbols[4] + i * 24)
            (index,) = self.unpack("<H", versyms[0][4] + i * 2)
            index &= 0x7FFF
            if shndx == 0 and index > 1:
                if index not in versions:
                    raise ElfError("Undefined symbol refers to absent GNU version")
                references.append({"symbol": self.string(symstr, name), **versions[index]})
        glibc_versions = sorted({r["version"] for r in references if r["version"].startswith("GLIBC_")})
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
            "versioned_undefined_symbols": sorted(references, key=lambda r: (r["library"], r["symbol"])),
        }


def inspect_bytes(data, name="xt-stcar"):
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
        (bool(report["glibc_versions"]), "No versioned undefined GLIBC symbols found"),
    ):
        if not condition:
            errors.append(error)
    if report["maximum_glibc"] and tuple(map(int, report["maximum_glibc"].split("."))) > tuple(map(int, GLIBC_BASELINE.split("."))):
        errors.append(f"Required GLIBC exceeds the locked {GLIBC_BASELINE} baseline")
    report.update(schema_version=1, glibc_baseline=GLIBC_BASELINE, file=name, sha256=hashlib.sha256(data).hexdigest(),
                  size_bytes=len(data), passed=not errors, errors=errors)
    return report


def inspect(path):
    return inspect_bytes(Path(path).read_bytes(), Path(path).name)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("binary", type=Path)
    parser.add_argument("--output", type=Path, help="Also save the JSON report")
    args = parser.parse_args()
    try:
        report = inspect(args.binary)
    except (OSError, ValueError, struct.error) as exc:
        report = {"schema_version": 1, "file": str(args.binary), "passed": False, "errors": [str(exc)]}
    rendered = json.dumps(report, indent=2, ensure_ascii=False) + "\n"
    if args.output:
        args.output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
