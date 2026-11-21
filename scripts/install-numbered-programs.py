#!/usr/bin/env python3
"""Install two prebuilt race ELFs and the numbered Bash entry, without starting a race."""
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
HOST = "bianbu@192.168.0.156"
SOCKET = ROOT / "work/vehicle-test-ssh.sock"

PATH_ENTRY = r'''#!/bin/sh
export LD_LIBRARY_PATH="/opt/ros/humble/lib/riscv64-linux-gnu:/opt/ros/humble/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$HOME/xt-stcar-console/race-programs/current/program-2" select-start "${0##*/}" "$@"
'''

# Bash handles unbounded numeric command names. Common whole-second names
# also work in a terminal opened before installation through normal PATH lookup.
BINDING = r'''# BEGIN XT-STCAR NUMBERED RACE
if [ -n "${BASH_VERSION:-}" ]; then
    if declare -F command_not_found_handle >/dev/null && ! declare -f command_not_found_handle | grep -q '__xt_stcar_numbered_entry'; then
        eval "$(declare -f command_not_found_handle | sed '1s/command_not_found_handle/__xt_stcar_previous_not_found/')"
    fi
    __xt_stcar_numbered_entry() {
        LD_LIBRARY_PATH="/opt/ros/humble/lib/riscv64-linux-gnu:/opt/ros/humble/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
            "$HOME/xt-stcar-console/race-programs/current/program-2" select-start "$@"
    }
    command_not_found_handle() {
        case "$1" in
            start_[0-9]*_*) __xt_stcar_numbered_entry "$@" ;;
            *) if declare -F __xt_stcar_previous_not_found >/dev/null; then
                   __xt_stcar_previous_not_found "$@"
               else
                   printf '%s: command not found\n' "$1" >&2
                   return 127
               fi ;;
        esac
    }
    for __xt_name in start start_dark start_light start_mix; do
        case "$(declare -f "$__xt_name" 2>/dev/null)" in
            *'/xt-stcar-console/'*) unset -f "$__xt_name" ;;
        esac
    done
    unset __xt_name
fi
# END XT-STCAR NUMBERED RACE
'''

REMOTE = r'''
from pathlib import Path
import copy, datetime, fcntl, hashlib, json, os, shutil, stat, subprocess, sys, tarfile, tempfile
home=Path.home()
base=home/'xt-stcar-console'
root=base/'race-programs'
root.mkdir(exist_ok=True)
# Both installers update the same Bash file and read the active release.
install_lock=os.open(root,os.O_RDONLY)
fcntl.flock(install_lock,fcntl.LOCK_EX)
manifest=json.loads(sys.argv[1])
binding,path_entry=sys.argv[2:]
entry_hash=hashlib.sha256(path_entry.encode()).hexdigest()[:12]
release=root/('p1-'+manifest['1']['sha256'][:12]+'-p2-'+manifest['2']['sha256'][:12]+'-entry-'+entry_hash)
active=root/'current'
if active.exists() and not active.is_symlink(): raise RuntimeError('existing current is not a release link')
launch=json.loads((base/'race-launch.json').read_bytes())
if 'mix' not in launch['profiles']: raise RuntimeError('installed mix vision profile missing')
if not Path(launch['bridge']).is_file() or not Path(launch['vision_script']).is_file(): raise RuntimeError('installed runtime assets missing')
for profile in launch['profiles'].values():
    model=Path(profile['model_dir'])/'race4.onnx'
    if not model.is_file(): raise RuntimeError('installed weather model missing')
def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1<<20),b''): h.update(block)
    return h.hexdigest()
rc=home/'.bashrc';original=rc.read_text()
start='# BEGIN XT-STCAR NUMBERED RACE';end='# END XT-STCAR NUMBERED RACE'
clean=''.join(line for line in original.splitlines(keepends=True)
              if not (line.startswith(('start()','start_dark()','start_light()','start_mix()')) and '/xt-stcar-console/' in line))
if start in clean:
    i=clean.index(start);j=clean.index(end,i)+len(end)
    updated=clean[:i]+binding.rstrip('\n')+clean[j:]
else:
    updated=binding.rstrip()+'\n\n'+clean
old=Path(os.readlink(active)) if active.is_symlink() else None
with tempfile.TemporaryDirectory(prefix='.install-',dir=root) as tmp:
    temp=Path(tmp);payload=temp/'programs';payload.mkdir(mode=0o755)
    with tarfile.open(fileobj=sys.stdin.buffer,mode='r|gz') as archive:
        names=set()
        for member in archive:
            if member.name not in ('program-1','program-2') or member.name in names or not member.isfile():
                raise RuntimeError('unexpected upload member')
            names.add(member.name);f=payload/member.name
            with f.open('xb') as dst,archive.extractfile(member) as src:shutil.copyfileobj(src,dst)
            os.chmod(f,0o755)
        if len(names)!=2: raise RuntimeError('missing ELF')
    for n,meta in manifest.items():
        exe=payload/('program-'+n)
        if sha(exe)!=meta['sha256']: raise RuntimeError('binary digest mismatch '+n)
        config=copy.deepcopy(launch)
        config.update(program_number=int(n),label=meta['label'],source_commit=meta['source'],base_source=meta['base'],
            evidence_prefix='left-cone-trial-'+datetime.date.today().strftime('%Y%m%d')+'-'+meta['label']+'-program'+n)
        (payload/('program-'+n+'.json')).write_text(json.dumps(config,indent=2)+'\n')
    entry=payload/'path-entry'
    entry.write_text(path_entry);entry.chmod(0o755)
    subprocess.run(['/bin/sh','-n',str(entry)],check=True)
    if not release.exists(): os.rename(payload,release)
    else:
        entry=release/'path-entry'
        if entry.read_text()!=path_entry or not os.access(entry,os.X_OK): raise RuntimeError('release entry differs')
        for n,meta in manifest.items():
            if sha(release/('program-'+n))!=meta['sha256']: raise RuntimeError('release collision '+n)
            cfg=json.loads((release/('program-'+n+'.json')).read_bytes())
            if cfg['source_commit']!=meta['source'] or cfg['profiles']!=launch['profiles']: raise RuntimeError('release metadata differs '+n)
    # Both target paths and the shell configuration are staged before activation.
    rc_tmp=temp/'bashrc.new'
    rc_tmp.write_text(updated);os.chmod(rc_tmp,stat.S_IMODE(rc.stat().st_mode))
    subprocess.run(['/bin/bash','-n',str(rc_tmp)],check=True)
    links=home/'.local/bin';links.mkdir(parents=True,exist_ok=True)
    target=active/'path-entry'
    previous={}
    for n in (1,2):
        for seconds in range(61):
            shortcut=links/f'start_{n}_{seconds}'
            if shortcut.is_symlink():
                prior=Path(os.readlink(shortcut))
                if prior not in (active/'program-2',target):
                    raise RuntimeError('existing user command '+str(shortcut))
            elif shortcut.exists():
                raise RuntimeError('existing user command '+str(shortcut))
            else:
                prior=None
            if prior!=target: previous[shortcut]=prior
    if original!=updated:
        saved=root/'shell-backups';saved.mkdir(mode=0o700,exist_ok=True)
        backup=saved/('bashrc-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
        shutil.copy2(rc,backup);backup.chmod(0o600)
    if rc.read_text()!=original: raise RuntimeError('Bash configuration changed during installation')
    link=temp/'current';link.symlink_to(release.name)
    with tempfile.TemporaryDirectory(prefix='.numbered-',dir=links) as link_tmp:
        staged=Path(link_tmp)/'next'
        changed=[]
        os.replace(link,active)
        try:
            if original!=updated:os.replace(rc_tmp,rc)
            for shortcut,prior in previous.items():
                if prior is None:
                    shortcut.symlink_to(target)
                else:
                    staged.symlink_to(target)
                    os.replace(staged,shortcut)
                changed.append(shortcut)
        except BaseException:
            staged.unlink(missing_ok=True)
            for shortcut in reversed(changed):
                prior=previous[shortcut]
                if prior is None: shortcut.unlink()
                else:
                    staged.symlink_to(prior);os.replace(staged,shortcut)
            if old is None:active.unlink(missing_ok=True)
            else:
                link.symlink_to(old);os.replace(link,active)
            if original!=updated:
                rc_tmp.write_text(original);os.chmod(rc_tmp,stat.S_IMODE(rc.stat().st_mode));os.replace(rc_tmp,rc)
            raise
print(json.dumps({'release':str(release),'programs':manifest,'source_uploaded':False,'race_started':False}))
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("program1", "program2"):
        parser.add_argument(name, type=Path)
        parser.add_argument("--"+name+"-source", required=True)
    args = parser.parse_args()
    release = {}
    with tempfile.TemporaryFile() as stream:
        with tarfile.open(fileobj=stream, mode="w:gz") as archive:
            for n, label, base in [(1,"y114","41337e111e8dc0bcb442d306b1aa5cec9b353f85"),
                                    (2,"y115","e35ea8026efb39fc1cd163bf6ce4b7828d81ce92")]:
                binary = getattr(args, f"program{n}").read_bytes()
                if binary[:6] != b"\x7fELF\x02\x01" or int.from_bytes(binary[18:20],"little") != 243:
                    raise ValueError(f"program {n} is not a RISC-V 64-bit ELF")
                revision = subprocess.check_output(["git","rev-parse","--verify",getattr(args,f"program{n}_source")+"^{commit}"],cwd=ROOT,text=True).strip()
                release[str(n)] = {"label":label,"source":revision,"base":base,"sha256":hashlib.sha256(binary).hexdigest()}
                member = tarfile.TarInfo(f"program-{n}"); member.mode = 0o755; member.size = len(binary)
                archive.addfile(member,io.BytesIO(binary))
        stream.seek(0)
        command = shlex.join(["python3","-B","-c",REMOTE,json.dumps(release),BINDING,PATH_ENTRY])
        subprocess.run(["ssh","-S",str(SOCKET),"-o","BatchMode=yes","-o","ConnectTimeout=5",HOST,command],
                       cwd=ROOT,stdin=stream,check=True,timeout=100)


if __name__ == "__main__":
    main()
