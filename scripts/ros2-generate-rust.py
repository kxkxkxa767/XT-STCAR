#!/usr/bin/env python3
"""Generate real Rust interfaces from the target ROS IDL into a project overlay.

Requires the upstream rosidl (matching ROS distro) and rosidl_rust source trees,
plus that distro's EmPy/lark dependencies in a project-local Python environment.
Does not install or change the target ROS installation.
"""
import argparse
import json
import os
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--ros-prefix', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--rosidl', type=Path, required=True)
parser.add_argument('--rosidl-rust', type=Path, required=True)
parser.add_argument('--distro', default='humble')
args = parser.parse_args()
os.environ['ROS_DISTRO'] = args.distro
sys.path[:0] = [str(args.rosidl.resolve()/'rosidl_parser'), str(args.rosidl.resolve()/'rosidl_cmake'),
               str(args.rosidl.resolve()/'rosidl_pycommon'), str(args.rosidl_rust.resolve()/'rosidl_generator_rs')]
from rosidl_generator_rs import generate_rs

prefix, output = args.ros_prefix.resolve(), args.output.resolve()
packages = ['builtin_interfaces', 'rcl_interfaces', 'rosgraph_msgs', 'unique_identifier_msgs', 'action_msgs', 'std_msgs', 'std_srvs']
output.mkdir(parents=True, exist_ok=True)
for package in packages:
    source = prefix/'share'/package
    interfaces = sorted(source.glob('*/*.idl'))
    if not interfaces:
        raise SystemExit(f'Missing target IDL: {source}')
    dependencies = set()
    for interface in interfaces:
        dependencies.update(re.findall(r'#include "([a-zA-Z0-9_]+)/', interface.read_text()))
    dependencies.discard(package)
    config = {
        'package_name': package, 'package_version': ET.parse(source/'package.xml').findtext('version'),
        'idl_tuples': [str(source)+':'+str(f.relative_to(source)) for f in interfaces],
        'ros_interface_dependencies': [p+':unused' for p in sorted(dependencies)],
        'template_dir': str(args.rosidl_rust.resolve()/'rosidl_generator_rs/resource'),
        'output_dir': str(output/'share'/package), 'target_dependencies': list(map(str, interfaces)),
    }
    path = output/(package+'-generator.json')
    path.write_text(json.dumps(config, indent=2)+'\n')
    generate_rs(str(path), 'rosidl_typesupport_c')
    print(f'{package}: {len(interfaces)} IDL files')
