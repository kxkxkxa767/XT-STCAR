#!/usr/bin/env python3
"""Vehicle terminal entry: each explicit `start` launches one installed trial."""
import argparse
import json
import os
from pathlib import Path
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--show', action='store_true', help='print the installed round without starting it')
    args = parser.parse_args()
    package = json.loads((Path(__file__).resolve().parent / 'current-race.json').read_text())
    if args.show:
        print(package['label'])
        return
    print(f"启动 {package['label']}；一轮结束后可再次输入 start。", flush=True)
    os.execv(sys.executable, [sys.executable, '-B', package['helper']])


if __name__ == '__main__':
    main()
