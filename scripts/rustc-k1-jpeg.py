#!/usr/bin/env python3
"""Select the K1 CPU for the target JPEG crates in the vehicle build."""
import os
import sys

compiler, *args = sys.argv[1:]
if '--target' in args and '--crate-name' in args:
    target = args[args.index('--target') + 1]
    crate = args[args.index('--crate-name') + 1]
    if target == 'riscv64gc-unknown-linux-gnu' and crate in ('xt_stcar_camera_jpeg', 'zune_jpeg', 'zune_core'):
        args.extend(['-C', 'target-cpu=spacemit-x60'])
    elif target == 'riscv64gc-unknown-linux-gnu' and crate == 'route_engine':
        if attributes := os.environ.get('XT_STCAR_K1_JPEG_ARCH_OBJECT'):
            args.extend(['-C', 'link-arg=' + attributes])

command = [compiler, *args]
if wrapper := os.environ.get('XT_STCAR_INNER_RUSTC_WRAPPER'):
    command.insert(0, wrapper)
os.execvp(command[0], command)
