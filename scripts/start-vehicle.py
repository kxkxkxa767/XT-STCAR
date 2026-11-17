#!/usr/bin/env python3
"""Vehicle terminal entry: each explicit `start` launches one installed trial."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

UNIT = 'xt-stcar-race'


def launch_background(base, package, profile, unit=UNIT):
    command = ['systemd-run', '--user', '--unit='+unit, '--collect',
               '--property=Type=exec', '--property=KillSignal=SIGINT',
               '--property=WorkingDirectory='+str(base),
               sys.executable, '-B', package['helper']]
    subprocess.run(command, check=True)
    print(f"已提交 {package['label']} / {profile}，后台运行一轮；断网或退出SSH不影响运行。", flush=True)
    print('结束后可重新摆放、复位，再输入同一启动命令。', flush=True)


def main():
    base = Path(__file__).resolve().parent
    if (base/'race-launch.json').is_file():
        program = str(base/'20260917/route-engine')
        environment = dict(os.environ, LD_LIBRARY_PATH='/opt/ros/humble/lib/riscv64-linux-gnu:/opt/ros/humble/lib')
        os.execve(program, [program, 'start', *sys.argv[1:]], environment)
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group()
    action.add_argument('--show', action='store_true', help='print the installed round without starting it')
    action.add_argument('--test', action='store_true', help='configure the provincial practice layout; does not start')
    action.add_argument('--normal', action='store_true', help='disable practice overrides; does not start')
    action.add_argument('--test-status', action='store_true', help='show practice configuration; does not start')
    action.add_argument('--stop', action='store_true', help='stop the background round')
    action.add_argument('--status', action='store_true', help='show the background round status')
    parser.add_argument('--profile', choices=('default', 'radar', 'dark', 'light'), default='default')
    parser.add_argument('--test-lights', choices=('fixed10s', 'vision'), help='set practice light handling; does not start')
    args = parser.parse_args()
    base = Path(__file__).resolve().parent
    marker = base/('current-race.json' if args.profile == 'default' else f'current-race-{args.profile}.json')
    if not marker.is_file():
        raise SystemExit(f'{args.profile} 版本尚未安装。')
    package = json.loads(marker.read_text())
    path = Path(package.get('field_config', base/'field-test.json'))
    if args.stop:
        subprocess.run(['systemctl', '--user', 'stop', UNIT+'.service'], check=True)
        print('已请求结束后台运行。')
        return
    if args.status:
        subprocess.run(['systemctl', '--user', 'status', UNIT+'.service', '--no-pager'], check=False)
        return
    if args.test or args.normal or args.test_status or args.test_lights:
        config = json.loads(path.read_text()) if path.is_file() else {'enabled': False}
        if args.test:
            config = ({'enabled': True, 'crosswalk_mode': 'vision', 'light_mode': 'fixed10s'}
                      if args.profile in ('dark', 'light') else
                      {'enabled': True, 'start_zone_m': 0.8, 'start_line_to_crosswalk_back_edge_m': 2.0,
                       'crosswalk_length_m': 0.297, 'start_straight_m': 3.2, 'light_mode': 'fixed10s'})
        if args.normal:
            config['enabled'] = False
        if args.test_lights:
            if args.normal or args.test_status or args.show or not config.get('enabled'):
                parser.error('--test-lights requires an enabled test configuration; use --test first')
            config['light_mode'] = args.test_lights
        if not args.test_status:
            temporary = path.with_suffix('.tmp')
            temporary.write_text(json.dumps(config, indent=2)+'\n')
            temporary.replace(path)
        print(json.dumps(config, indent=2, ensure_ascii=False))
        print('仅配置/查看，没有发车。')
        return
    if args.show:
        print(package['label'])
        return
    if args.profile != 'default':
        launch_background(base, package, args.profile)
        return
    print(f"启动 {package['label']}；一轮结束后可再次输入 start。", flush=True)
    os.execv(sys.executable, [sys.executable, '-B', package['helper']])


if __name__ == '__main__':
    main()
