#!/usr/bin/env python3
"""Remove installed ROS bag packages on the vehicle; default is an apt simulation.

Run on the car after the operator returns. --apply requires root and removes
only the reviewed apt plan. No autoremove, ROS startup, or vehicle motion.
"""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import time

BAG = re.compile(r'^(?:ros-[a-z0-9]+-(?:ros2bag|rosbag2(?:-.*)?|rosbag)|ros2bag|rosbag2(?:-.*)?|python3?-rosbag)$')
META = re.compile(r'^ros-[a-z0-9]+-(?:ros-base|desktop|desktop-full)$')


def command(args):
    return subprocess.run(args, check=True, text=True, capture_output=True).stdout


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true')
    parser.add_argument('--allow-dependent', action='append', default=[], metavar='PACKAGE',
                        help='exact package name of a reviewed, unused bag-dependent component')
    parser.add_argument('--out', type=Path, default=Path('work/bag-cleanup'))
    args = parser.parse_args()
    listing = command(['dpkg-query', '-W', '-f=${binary:Package}\t${db:Status-Status}\n'])
    packages = sorted({line.split('\t')[0] for line in listing.splitlines()
                       if line.endswith('\tinstalled') and BAG.fullmatch(line.split('\t')[0].split(':')[0])})
    if not packages:
        print('没有已安装的 ROS bag 软件包；手工安装残留见环境核对说明。')
        return
    env = dict(os.environ, LC_ALL='C')
    plan = subprocess.run(['apt-get', '-s', 'purge', *packages], env=env,
                          check=True, text=True, capture_output=True).stdout
    removed = sorted(set(re.findall(r'^(?:Remv|Purg)\s+(\S+)', plan, re.M)))
    extra = [p for p in removed if not BAG.fullmatch(p.split(':')[0])
             and not META.fullmatch(p.split(':')[0]) and p not in args.allow_dependent]
    args.out.mkdir(parents=True, exist_ok=True)
    prefix = args.out / time.strftime('%Y%m%d-%H%M%S')
    prefix.with_suffix('.plan.txt').write_text(plan)
    prefix.with_suffix('.json').write_text(json.dumps({'requested': packages, 'removal_plan': removed,
        'additional_packages': extra, 'allowed_dependents': args.allow_dependent,
        'apply_requested': args.apply}, indent=2) + '\n')
    print(plan)
    if extra:
        raise SystemExit('apt 还会移除上述 bag/聚合包以外的软件；保留现场，先解决依赖。')
    if args.apply:
        if os.geteuid() != 0:
            raise SystemExit('实际卸载请用 sudo python3 scripts/remove-bag-tools.py --apply。')
        subprocess.run(['apt-get', '-y', 'purge', *packages], env=env, check=True)
        print('bag 包已卸载；未执行 autoremove。')


if __name__ == '__main__':
    main()
