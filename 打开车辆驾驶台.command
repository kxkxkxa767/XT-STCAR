#!/bin/zsh
# Finder 双击入口；不包含访问令牌，不发送车辆运动指令。
cd -- "${0:A:h}" || exit 1
python3 scripts/open-vehicle-console.py "$@"
result=$?
if (( result != 0 )); then
  print '\n请检查车辆电源、网络及SSH连接。按回车关闭窗口。'
  read -r
fi
exit "$result"
