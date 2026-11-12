#!/usr/bin/env python3
"""Use the local round builder for a ROS2-featured package, never a native fallback.

The project-local build-engine.py must support XT_STCAR_ROUTE_FEATURES and the
ros2-humble ELF profile. This wrapper checks the produced feature manifest.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('baseline')
parser.add_argument('label')
parser.add_argument('round')
parser.add_argument('record_key')
parser.add_argument('--ros-prefix', type=Path, required=True)
parser.add_argument('--interfaces', type=Path, required=True)
args = parser.parse_args()
base = ROOT/'work/vehicle-dynamic-deploy-20261008'
builder = base/'build-engine.py'
if 'XT_STCAR_ROUTE_FEATURES' not in builder.read_text():
    raise SystemExit('The local round builder has no ROS2 feature support; use build-ros2-race.py or update the builder explicitly.')
env = dict(os.environ, XT_STCAR_ROUTE_FEATURES='ros2', ROS_DISTRO='humble',
    AMENT_PREFIX_PATH=os.pathsep.join(map(str, [args.interfaces.resolve(), args.ros_prefix.resolve()])),
    RUSTFLAGS='-C link-arg=-Wl,-rpath,/opt/ros/humble/lib -C link-arg=-Wl,--disable-new-dtags -C link-arg=-Wl,--allow-shlib-undefined')
subprocess.run([sys.executable, '-B', str(base/'build-race.py'), args.baseline, args.label, args.round, args.record_key],
    env=env, cwd=ROOT, check=True)
path = base/(args.label+'-package.json')
package = json.loads(path.read_text())
manifest = json.loads((Path(package['bundle'])/'manifest.json').read_text())
if manifest['route_engine_build'].get('features') != ['ros2']:
    raise SystemExit('Produced binary is not declared ROS2-enabled; refusing ROS2 package label.')
helper = Path(package['helper'])
fixed = package['helper_fixed_args'] + ['--ros2-setup', '/opt/ros/humble/setup.bash']
text, count = re.subn(r'^FIXED_ARGS = .*$', lambda _: 'FIXED_ARGS = '+repr(fixed)+'  # ROS2 Humble race',
    helper.read_text(), flags=re.M)
assert count == 1
compile(text, str(helper), 'exec')
helper.write_text(text)
package.update(helper_fixed_args=fixed, helper_sha256=hashlib.sha256(helper.read_bytes()).hexdigest(),
    ros2={'distro': 'humble', 'rclrs': '0.8.0', 'rmw': 'rmw_fastrtps_cpp', 'hardware_nodes': True})
overlay = base/(args.label+'-ros2')
subprocess.run([sys.executable, '-B', str(ROOT/'scripts/build-ros2-race.py'),
    '--ros-prefix', str(args.ros_prefix), '--interfaces', str(args.interfaces), '--out', str(overlay),
    '--binary', str(Path(package['bundle'])/'payload/route-engine'), '--require-clean'], env=env, cwd=ROOT, check=True)
package['ros2_install_archive'] = str(overlay/'xt-stcar-ros2-install.tar.gz')
path.write_text(json.dumps(package, indent=2)+'\n')
print(json.dumps({'package': str(path), 'source_commit': package['source_commit'],
    'ros2_install_archive': package['ros2_install_archive'], 'helper_ros2_setup': fixed[-1]}, indent=2))
