#!/usr/bin/env bash
# Run locally on the vehicle after reviewing. No SSH, package installs, drivers or motion.
set -u
check() {
  printf '\n$'
  printf ' %q' "$@"
  printf '\n'
  "$@" 2>&1 || printf '%s\n' '(unavailable or failed; recorded only)'
}
check uname -a
check cat /etc/os-release
check getconf GNU_LIBC_VERSION
check dpkg --print-architecture
check python3 --version
check ls -ld /lib/ld-linux-riscv64-lp64d.so.1 /opt/ros /opt/bros
check printenv ROS_DISTRO
check lsusb
check ls -l /dev/serial/by-id/ /dev/v4l/by-id/
check systemctl --type=service --state=running --no-pager
check dpkg-query -W python3-spacemit-ort
check python3 -c 'import importlib.util; print({name: importlib.util.find_spec(name) is not None for name in ("numpy", "onnx", "onnxruntime", "spacemit_ort", "cv2")})'
