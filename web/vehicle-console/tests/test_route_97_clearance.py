"""Round 97: the console's body-clearance check runs in the Rust route engine.

route-engine/src/clearance.rs ports autonomy_live.probe_clearance and
ClearanceReuse; with the engine's turn motion the autonomy tick sends the
check in one request with the current contact target (route_engine
RustManeuverSequence.clearance_probe). Equality over whole routes and random
real-scan cases is checked by scripts/route-engine-parity(-sweep).py and
scripts/route-engine-clearance-parity.py; these tests cover the proxy (kept
results as shallow copies, the body-stop exception with its clearance, the
reuse token) and the console path against the Python one. They need the Mac
build of route-engine/ and are skipped without it. Synthetic scans and
clocks; no physical claims.
"""
import copy
import json
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).parents[1]))
import autonomy_live as live
import route_engine
import test_route_contact_control as contact
import test_turn_control as control

ENGINE = Path(__file__).resolve().parents[3]/'route-engine'/'target'/'release'/'route-engine'
AGES = {'camera': .01, 'lidar': .05, 'control': .01}
ROUTE_OPTIONS = dict(rear_launch=False, camera_required=False, clearance_profile='maneuver',
                     relaxed_turn_left=False, front_lane=None, front_side_walls=True, rear_start=False)


def text(value):
    """Exact comparison: types, float bits and key order."""
    return json.dumps(value)


@unittest.skipUnless(ENGINE.is_file(), 'route-engine not built (cargo build --release in route-engine/)')
class EngineClearanceProxyTests(unittest.TestCase):
    def setUp(self):
        self.engine = route_engine.RouteEngine(ENGINE)
        self.motion = route_engine.RustManeuverSequence(self.engine, 0., continue_route=True)

    def tearDown(self):
        self.engine.close()

    def both(self, reuse, scan, ages=AGES, **options):
        options = {**ROUTE_OPTIONS, **options}
        python = reuse.probe(live.probe_clearance, scan, ages, False, False, body_contact_target=None, **options)
        target, rust, count = self.motion.clearance_probe(scan, 0., False, ages, False, False, **options)
        self.assertIsNone(target)
        self.assertEqual(text(rust), text(python))
        self.assertEqual(count, reuse.reused)
        return python, rust

    def test_results_and_reuse_follow_clearance_reuse(self):
        reuse = live.ClearanceReuse()
        first = control.TurnControlTests.scan(1)
        self.both(reuse, first)
        self.both(reuse, first)
        self.both(reuse, copy.deepcopy(first))  # equal content is the same scan
        self.assertEqual(reuse.reused, 2)
        self.both(reuse, first, {**AGES, 'lidar': .31})  # across the lidar limit: checked again
        self.both(reuse, first, front_side_walls=False)  # other arguments: checked again
        lane = first['corridor_candidates'][0]
        self.both(reuse, first, front_lane=lane, front_side_walls=False)
        self.both(reuse, first, front_lane=copy.deepcopy(lane), front_side_walls=False)
        self.both(reuse, control.TurnControlTests.scan(2), front_lane=None)
        self.assertEqual(reuse.reused, 3)

    def test_kept_result_is_handed_out_as_a_shallow_copy(self):
        reuse = live.ClearanceReuse()
        scan = control.TurnControlTests.scan(1)
        python_fresh, rust_fresh = self.both(reuse, scan)
        for result in (python_fresh, rust_fresh):
            result['quality_issues'] = ['set_by_the_service']  # the console sets top-level fields
        python_again, rust_again = self.both(reuse, scan)
        python_third, rust_third = self.both(reuse, scan)
        self.assertEqual(rust_again['quality_issues'], [])
        for again, third in ((python_again, python_third), (rust_again, rust_third)):
            self.assertIsNot(again, third)
            self.assertIs(again['body_proximity'], third['body_proximity'])

    def test_body_stop_carries_its_clearance_and_is_never_reused(self):
        reuse = live.ClearanceReuse()
        scan = control.TurnControlTests.scan(1)
        scan['ranges'][0] = .25  # five cm beyond the front face, inside the 8 cm gate
        for _ in range(2):
            with self.assertRaises(live.ProbeClearanceError) as python:
                reuse.probe(live.probe_clearance, scan, AGES, False, False, body_contact_target=None,
                            **ROUTE_OPTIONS)
            with self.assertRaises(live.ProbeClearanceError) as rust:
                self.motion.clearance_probe(scan, 0., False, AGES, False, False, **ROUTE_OPTIONS)
            self.assertEqual(str(rust.exception), 'probe_obstacle_close_body')
            self.assertEqual(text(rust.exception.clearance), text(python.exception.clearance))
            self.assertEqual(rust.exception.clearance['body_proximity']['stop_trigger_point']['angle_deg'], 0)
        self.assertEqual(reuse.reused, 0)
        _, _, count = self.motion.clearance_probe(control.TurnControlTests.scan(2), 0., False, AGES, False, False,
                                                  **ROUTE_OPTIONS)
        self.assertEqual(count, 0)

    def test_other_refusals_keep_their_type_and_message(self):
        scan = control.TurnControlTests.scan(1)
        cases = ((AGES | {'control': .2}, {}, 'probe_sensor_unavailable'),
                 (AGES, {'clearance_profile': 'straight'}, 'probe_invalid_front_lane'),
                 (AGES, {'rear_start': 1}, 'probe_invalid_rear_start'),
                 ({'lidar': -1, 'control': .01}, {}, 'probe_invalid_sensor_age'))
        for ages, change, message in cases:
            with self.subTest(message=message):
                with self.assertRaises(ValueError) as rust:
                    self.motion.clearance_probe(scan, 0., False, ages, False, False, **{**ROUTE_OPTIONS, **change})
                self.assertIs(type(rust.exception), ValueError)
                self.assertEqual(str(rust.exception), message)
                with self.assertRaisesRegex(ValueError, f'^{message}$'):
                    live.probe_clearance(scan, ages, False, False, **{**ROUTE_OPTIONS, **change})

    def test_a_reuse_answer_without_the_kept_result_is_a_protocol_error(self):
        scan = control.TurnControlTests.scan(1)
        self.motion.clearance_probe(scan, 0., False, AGES, False, False, **ROUTE_OPTIONS)
        self.motion.__dict__['_clearance'] = (-1, {})
        with self.assertRaisesRegex(RuntimeError, '^route_engine_protocol_error$'):
            self.motion.clearance_probe(scan, 0., False, AGES, False, False, **ROUTE_OPTIONS)


@unittest.skipUnless(ENGINE.is_file(), 'route-engine not built (cargo build --release in route-engine/)')
class EngineConsoleClearanceTests(unittest.TestCase):
    """The tick's own path: a bound cone into contact continues, a wall point stops."""

    def scenario(self, engine):
        h = contact.RouteContactControlTests('test_service_maintains_real_bound_target_into_contact_then_stops_for_wall')
        h.setUp()
        try:
            h.console.route_engine = engine
            _, session = h.enter_compact_orbit(continue_route=True)
            ticks = []
            for distance in (.78, .55, .36, .25, .17):
                value = h.compact_scan(h.console.scan['seq']+1, observed=False)
                half_span = round(math.degrees(math.atan2(.06, distance)))
                value['ranges'][270-half_span:271+half_span] = [distance]*(2*half_span+1)
                h.observe(round(h.now+.1, 6), value)
                self.assertIs(h.console.auto_session, session)
                report = session['report']
                ticks.append([report['clearance_current'], report.get('clearance_reused_count'),
                              report.get('target_contact_tolerated_count')])
            uses_python_reuse = 'clearance_reuse' in session
            value = copy.deepcopy(value)
            value['seq'] += 1
            value['at_ms'] += 100
            value['ranges'][0] = .25
            h.tick(round(h.now+.02, 6), value)
            self.assertIsNone(h.console.auto_session)
            return ticks, list(h.outputs), h.console.auto_result, uses_python_reuse
        finally:
            h.tearDown()

    def test_engine_check_in_the_tick_matches_python(self):
        engine = route_engine.RouteEngine(ENGINE)
        try:
            rust_ticks, rust_outputs, rust_result, rust_reuse = self.scenario(engine)
        finally:
            engine.close()
        python_ticks, python_outputs, python_result, python_reuse = self.scenario(None)
        self.assertEqual((rust_reuse, python_reuse), (False, True))  # the engine keeps its own reuse
        self.assertEqual(text(rust_ticks), text(python_ticks))
        self.assertEqual(rust_outputs, python_outputs)
        self.assertEqual(rust_outputs[-1]['op'], 'stop')
        body = rust_ticks[-1][0]['body_proximity']
        self.assertTrue(body['target_contact_tolerated'])
        self.assertEqual(body['target_contact_evidence']['role'], 'cone_like_support')
        self.assertGreater(rust_ticks[-1][1], 0)  # repeated frames were reused
        self.assertEqual((rust_result['reason'], python_result['reason']), ('probe_obstacle_close_body',) * 2)
        self.assertEqual(text(rust_result['clearance_current']), text(python_result['clearance_current']))
        self.assertEqual(rust_result['clearance_current']['body_proximity']['stop_trigger_point']['angle_deg'], 0)
        self.assertEqual(rust_result['route_controller'], 'rust_engine')


if __name__ == '__main__':
    unittest.main()
