#!/usr/bin/env python3
"""Run console unit test modules by name (web/vehicle-console/tests), e.g. under
route-engine-supervisor-parity.py --keep-going. Offline.

  python3 scripts/run-console-tests.py test_turn_control test_route_control ...
"""
from pathlib import Path
import sys
import unittest

TESTS = Path(__file__).resolve().parents[1]/'web'/'vehicle-console'/'tests'
sys.path.insert(0, str(TESTS))
suite = unittest.defaultTestLoader.loadTestsFromNames(sys.argv[1:])
result = unittest.TextTestRunner(verbosity=1).run(suite)
sys.exit(0 if result.wasSuccessful() else 1)
