#!/usr/bin/env python3
"""Vehicle terminal entry: each explicit `start` launches one installed trial."""
import argparse
import json
import os
from pathlib import Path
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group()
    action.add_argument('--show', action='store_true', help='print the installed round without starting it')
    action.add_argument('--test', action='store_true', help='configure the provincial practice layout; does not start')
    action.add_argument('--normal', action='store_true', help='disable practice overrides; does not start')
    action.add_argument('--test-status', action='store_true', help='show practice configuration; does not start')
    parser.add_argument('--test-lights', choices=('fixed10s', 'vision'), help='set practice light handling; does not start')
    args = parser.parse_args()
    base = Path(__file__).resolve().parent
    path = base/'field-test.json'
    if args.test or args.normal or args.test_status or args.test_lights:
        config = json.loads(path.read_text()) if path.is_file() else {'enabled': False}
        if args.test:
            config = {'enabled': True, 'start_zone_m': 0.8, 'start_line_to_crosswalk_back_edge_m': 2.0,
                      'crosswalk_length_m': 0.297, 'start_straight_m': 3.2, 'light_mode': 'fixed10s'}
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
    package = json.loads((base / 'current-race.json').read_text())
    if args.show:
        print(package['label'])
        return
    print(f"启动 {package['label']}；一轮结束后可再次输入 start。", flush=True)
    os.execv(sys.executable, [sys.executable, '-B', package['helper']])


if __name__ == '__main__':
    main()
