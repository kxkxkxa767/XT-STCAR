#!/usr/bin/env python3
"""Install programs 4 and 5 alongside the existing numbered releases, without starting a race."""
import argparse
import hashlib
import io
import json
from pathlib import Path
import shlex
import subprocess
import tarfile
import tempfile

ROOT = Path(__file__).resolve().parents[1]
PATH_ENTRY = r'''#!/bin/sh
export LD_LIBRARY_PATH="/opt/ros/humble/lib/riscv64-linux-gnu:/opt/ros/humble/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$HOME/xt-stcar-console/race-programs/vision-current/program-4" select-start "${0##*/}" "$@"
'''
BINDING = r'''# BEGIN XT-STCAR VISION PROGRAMS
if [ -n "${BASH_VERSION:-}" ]; then
    __xt_stcar_numbered_entry() {
        local runner
        case "$1" in
            start_4_*|start_5_*) runner="$HOME/xt-stcar-console/race-programs/vision-current/program-4" ;;
            start_3_*) runner="$HOME/xt-stcar-console/race-programs/program3-current/program-3" ;;
            *) runner="$HOME/xt-stcar-console/race-programs/current/program-2" ;;
        esac
        LD_LIBRARY_PATH="/opt/ros/humble/lib/riscv64-linux-gnu:/opt/ros/humble/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
            "$runner" select-start "$@"
    }
fi
# END XT-STCAR VISION PROGRAMS
'''
REMOTE = r'''
import fcntl, hashlib, json, os, shutil, stat, subprocess, sys, tarfile, tempfile
from pathlib import Path
home=Path.home()
programs=home/"xt-stcar-console/race-programs"
install_lock=os.open(programs,os.O_RDONLY)
fcntl.flock(install_lock,fcntl.LOCK_EX)
manifest=json.loads(sys.argv[1])
entry,binding=sys.argv[2:]
active=programs/"vision-current"
if active.exists() and not active.is_symlink():
    raise RuntimeError("vision-current is a user file")
sha=lambda data:hashlib.sha256(data).hexdigest()
env=os.environ.copy()
env["LD_LIBRARY_PATH"]="/opt/ros/humble/lib/riscv64-linux-gnu:/opt/ros/humble/lib"+(":"+env["LD_LIBRARY_PATH"] if env.get("LD_LIBRARY_PATH") else "")
configs={}
for n,parent,directory in (("4",2,"current"),("5",3,"program3-current")):
    prior=json.loads((programs/directory/f"program-{parent}.json").read_text())
    cfg={k:prior[k] for k in ("base","bridge")}
    cfg.update(program_number=int(n),label=manifest[n]["label"],source_commit=manifest[n]["source"],
               base_source=manifest[n]["parent_source"],camera=prior["profiles"]["mix"].get("camera","/dev/video20"),
               vision_backend="rust_v4l2",evidence_prefix="left-cone-trial-vision-program"+n)
    if n=="5": cfg["forward_pwm_max"]=1560
    configs[n]=cfg
release=programs/("vision-"+sha(json.dumps([manifest,configs,entry],sort_keys=True).encode())[:24])
rc=home/".bashrc"
old_rc=rc.read_text()
start="# BEGIN XT-STCAR VISION PROGRAMS"
end="# END XT-STCAR VISION PROGRAMS"
if start in old_rc:
    i=old_rc.index(start); j=old_rc.index(end,i)+len(end)
    updated=old_rc[:i]+binding.rstrip("\n")+old_rc[j:]
else:
    old_end="# END XT-STCAR PROGRAM 3\n" if "# END XT-STCAR PROGRAM 3\n" in old_rc else "# END XT-STCAR NUMBERED RACE\n"
    i=old_rc.index(old_end)+len(old_end)
    updated=old_rc[:i]+binding+old_rc[i:]
links=home/".local/bin"
links.mkdir(parents=True,exist_ok=True)
target=active/"path-entry"
to_create=[]
for n in ("4","5"):
    for sec in range(61):
        link=links/f"start_{n}_{sec}"
        if link.is_symlink() and os.readlink(link)==str(target):continue
        if link.is_symlink() or link.exists():raise RuntimeError("existing user command "+str(link))
        to_create.append(link)
old_active=os.readlink(active) if active.is_symlink() else None
with tempfile.TemporaryDirectory(dir=programs,prefix=".install-vision-") as tmp:
    tmp=Path(tmp);payload=tmp/"payload";payload.mkdir(mode=0o755)
    with tarfile.open(fileobj=sys.stdin.buffer,mode="r|gz") as archive:
        seen=set()
        for member in archive:
            if member.name not in ("program-4","program-5") or member.name in seen or not member.isfile():raise RuntimeError("bad upload member")
            seen.add(member.name);dst=payload/member.name
            with dst.open("xb") as out,archive.extractfile(member) as src:shutil.copyfileobj(src,out)
            dst.chmod(0o755)
        if len(seen)!=2:raise RuntimeError("incomplete program pair")
    (payload/"path-entry").write_text(entry);(payload/"path-entry").chmod(0o755)
    subprocess.run(["/bin/sh","-n",str(payload/"path-entry")],check=True)
    for n,cfg in configs.items():
        name="program-"+n;exe=payload/name;settings=payload/(name+".json")
        if sha(exe.read_bytes())!=manifest[n]["sha256"]:raise RuntimeError("binary checksum "+n)
        settings.write_text(json.dumps(cfg,indent=2)+"\n")
        shown=json.loads(subprocess.check_output([str(exe),"start","--config",str(settings),"--profile","rust","--light-max-wait-s","7","--show"],env=env,text=True))
        if (shown["program"],shown["vision_backend"],shown.get("model"),shown.get("forward_pwm_max"),shown["source_commit"])!=(int(n),"rust_v4l2",None,cfg.get("forward_pwm_max"),cfg["source_commit"]):raise RuntimeError("wrong installed program "+n)
    if release.exists():
        if any(sha(p.read_bytes())!=sha((release/p.name).read_bytes()) for p in payload.iterdir()):raise RuntimeError("release collision")
    else:os.rename(payload,release)
    rc_new=tmp/"bashrc";rc_new.write_text(updated);rc_new.chmod(stat.S_IMODE(rc.stat().st_mode))
    subprocess.run(["/bin/bash","-n",str(rc_new)],check=True)
    if rc.read_text()!=old_rc:raise RuntimeError("shell configuration changed")
    if old_rc!=updated:
        backups=programs/"shell-backups";backups.mkdir(mode=0o700,exist_ok=True)
        with tempfile.NamedTemporaryFile(dir=backups,delete=False,prefix="before-vision-") as backup:backup.write(old_rc.encode())
    switch=tmp/"active";switch.symlink_to(release.name);os.replace(switch,active)
    created=[];rc_changed=False
    try:
        if updated!=old_rc:os.replace(rc_new,rc);rc_changed=True
        for link in to_create:link.symlink_to(target);created.append(link)
    except BaseException:
        for link in reversed(created):link.unlink()
        if rc_changed:
            rc_new.write_text(old_rc);rc_new.chmod(stat.S_IMODE(rc.stat().st_mode));os.replace(rc_new,rc)
        if old_active is None:active.unlink()
        else:switch.symlink_to(old_active);os.replace(switch,active)
        raise
print(json.dumps({"release":str(release),"programs":manifest,"source_uploaded":False,"race_started":False}))
'''

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket",type=Path,required=True)
    for n in (4,5):
        parser.add_argument(f"program{n}",type=Path)
        parser.add_argument(f"--source{n}",required=True)
    args=parser.parse_args()
    manifest={}
    with tempfile.TemporaryFile() as data:
        with tarfile.open(fileobj=data,mode="w:gz") as archive:
            for n in (4,5):
                binary=getattr(args,f"program{n}").read_bytes()
                if binary[:6]!=b"\x7fELF\x02\x01" or int.from_bytes(binary[18:20],"little")!=243:
                    parser.error(f"program {n} needs a 64-bit RISC-V ELF")
                rev=subprocess.check_output(["git","rev-parse","--verify",getattr(args,f"source{n}")+"^{commit}"],cwd=ROOT,text=True).strip()
                manifest[str(n)]={"source":rev,"sha256":hashlib.sha256(binary).hexdigest(),"label":"y115-rust"+str(n),
                                  "parent_source":"1466a7276fc4378f706f2c7175c8808a3f7dbfd0" if n==4 else "e75ea79cbd920c35905ef92a24a8c8031c6711ec"}
                info=tarfile.TarInfo("program-"+str(n));info.size=len(binary);archive.addfile(info,io.BytesIO(binary))
        data.seek(0)
        command=shlex.join(["python3","-B","-c",REMOTE,json.dumps(manifest),PATH_ENTRY,BINDING])
        subprocess.run(["ssh","-S",str(args.socket.resolve()),"-o","BatchMode=yes","-o","ConnectTimeout=5","bianbu@192.168.0.156",command],stdin=data,check=True,timeout=120)
if __name__=="__main__":main()
