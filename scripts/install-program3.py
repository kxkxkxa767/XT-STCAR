#!/usr/bin/env python3
"""Install the conservative N10P binary beside the numbered race programs."""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import subprocess

ROOT = Path(__file__).resolve().parents[1]
COPY_BASE = "0492cdc397ae6a4afd8d1cde56bd7a466416c8a1"
HOST = "bianbu@192.168.0.156"

BASH_BINDING = r'''# BEGIN XT-STCAR PROGRAM 3
if [ -n "${BASH_VERSION:-}" ]; then
    __xt_stcar_numbered_entry() {
        local runner="$HOME/xt-stcar-console/race-programs/current/program-2"
        case "$1" in start_3_*) runner="$HOME/xt-stcar-console/race-programs/program3-current/program-3" ;; esac
        LD_LIBRARY_PATH="/opt/ros/humble/lib/riscv64-linux-gnu:/opt/ros/humble/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
            "$runner" select-start "$@"
    }
fi
# END XT-STCAR PROGRAM 3
'''

REMOTE = r'''
import hashlib, json, os, shutil, stat, subprocess, sys, tempfile
from pathlib import Path

digest, revision, base_revision, binding = sys.argv[1:]
home = Path.home()
programs = home / "xt-stcar-console/race-programs"
normal = programs / "current"
active = programs / "program3-current"
release = programs / ("p3-1560-" + digest[:16])
config = json.loads((normal / "program-2.json").read_bytes())
config.update(program_number=3, label="y115-1560", source_commit=revision, base_source=base_revision,
              forward_pwm_max=1560, evidence_prefix="left-cone-trial-y115-1560-program3")
if active.exists() and not active.is_symlink():
    raise RuntimeError("program3-current already exists as a user file")

rc = home / ".bashrc"
original = rc.read_text()
clean = original
begin, end = "# BEGIN XT-STCAR PROGRAM 3", "# END XT-STCAR PROGRAM 3\n"
if begin in clean:
    i = clean.index(begin)
    clean = clean[:i] + clean[clean.index(end, i) + len(end):]
base_end = "# END XT-STCAR NUMBERED RACE\n"
pos = clean.index(base_end) + len(base_end)
updated = clean[:pos] + binding + clean[pos:]

shortcuts = [home / ".local/bin" / ("start_3_" + str(s)) for s in range(61)]
target = active / "program-3"
for link in shortcuts:
    if (link.is_symlink() or link.exists()) and not (link.is_symlink() and os.readlink(link) == str(target)):
        raise RuntimeError("existing user command: " + str(link))

with tempfile.TemporaryDirectory(prefix=".install-program3-", dir=programs) as scratch:
    tmp = Path(scratch)
    payload = tmp / "payload"
    payload.mkdir()
    binary = payload / "program-3"
    with binary.open("xb") as stream:
        shutil.copyfileobj(sys.stdin.buffer, stream)
    if hashlib.sha256(binary.read_bytes()).hexdigest() != digest:
        raise RuntimeError("program 3 transfer incomplete")
    binary.chmod(0o755)
    settings = payload / "program-3.json"
    settings.write_text(json.dumps(config, indent=2) + "\n")
    # The selector can also delegate explicitly named 1/2 calls to their normal release.
    for n in (1, 2):
        for suffix in ("", ".json"):
            name = f"program-{n}{suffix}"
            (payload / name).symlink_to(Path("..") / "current" / name)
    shown = json.loads(subprocess.check_output([str(binary), "start", "--config", str(settings),
        "--profile", "mix", "--light-max-wait-s", "7", "--show"], text=True))
    if (shown["program"], shown["forward_pwm_max"], shown["light_max_wait_s"]) != (3, 1560, 7):
        raise RuntimeError("wrong program 3 binary or settings")
    if not release.exists():
        os.rename(payload, release)
    elif hashlib.sha256((release / "program-3").read_bytes()).hexdigest() != digest or json.loads((release / "program-3.json").read_bytes()) != config:
        raise RuntimeError("existing program 3 release differs")

    rc_new = tmp / "bashrc"
    rc_new.write_text(updated)
    rc_new.chmod(stat.S_IMODE(rc.stat().st_mode))
    subprocess.run(["/bin/bash", "-n", str(rc_new)], check=True)
    old = os.readlink(active) if active.is_symlink() else None
    backup = None
    if updated != original:
        backups = programs / "shell-backups"
        backups.mkdir(exist_ok=True, mode=0o700)
        with tempfile.NamedTemporaryFile(dir=backups, prefix="before-program3-", delete=False) as f:
            backup = Path(f.name)
            f.write(original.encode())
    new_link = tmp / "activate"
    new_link.symlink_to(release.name)
    if rc.read_text() != original:
        raise RuntimeError("Bash configuration changed during installation")
    os.replace(new_link, active)
    created = []
    try:
        if updated != original:
            os.replace(rc_new, rc)
        for link in shortcuts:
            if not link.is_symlink():
                link.symlink_to(target)
                created.append(link)
    except BaseException:
        for link in created:
            link.unlink()
        if old is None:
            active.unlink()
        else:
            new_link.symlink_to(old)
            os.replace(new_link, active)
        if backup is not None:
            rc_new.write_text(original)
            rc_new.chmod(stat.S_IMODE(rc.stat().st_mode))
            os.replace(rc_new, rc)
        raise
print(json.dumps({"program":3, "release":str(release), "source_commit":revision, "sha256":digest,
                  "forward_pwm_max":1560, "normal_release":os.readlink(normal), "started":False}))
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("binary", type=Path)
    parser.add_argument("--socket", type=Path, required=True)
    parser.add_argument("--source", required=True, help="Git revision used to build the ELF")
    args = parser.parse_args()
    binary = args.binary.read_bytes()
    if binary[:6] != b"\x7fELF\x02\x01" or int.from_bytes(binary[18:20], "little") != 243:
        parser.error("expected a RISC-V 64-bit ELF")
    revision = subprocess.check_output(
        ["git", "rev-parse", "--verify", args.source + "^{commit}"], cwd=ROOT, text=True
    ).strip()
    command = shlex.join(["python3", "-B", "-c", REMOTE, hashlib.sha256(binary).hexdigest(),
                          revision, COPY_BASE, BASH_BINDING])
    subprocess.run(["ssh", "-S", str(args.socket.resolve()), "-o", "BatchMode=yes", "-o", "ConnectTimeout=5",
                    HOST, command], input=binary, check=True, timeout=100)


if __name__ == "__main__":
    main()
