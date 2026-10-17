"""Round 64 car run (2026-10-17): one start-lane tick took .24 s on the car,
held the control lock past the 300 ms heartbeat lease and stopped the run.
These tests pin the faster forms to the original ones (same outputs), the
per-scan caches to the scan content, and the LAN read pause while an
autonomous run owns the car."""
import json
import math
from pathlib import Path
import random
import sys
import unittest

sys.path.insert(0, str(Path(__file__).parents[1]))
import maneuver_sequence as ms  # noqa: E402
from maneuver_sequence import (ManeuverSequence, _axis_lines, _plan_loop, _loop_extent, _scan_points,
                               OPEN_LINE_BAND_M, OPEN_LINE_MIN_POINTS, OPEN_LANE_MIN_SPAN_M, OPEN_LINE_GAP_M,
                               OPEN_LINE_GAP_SCALE, OPEN_LINE_REACH_M, OPEN_INNER_GAP_M, OPEN_OUTER_GAP_M,
                               OPEN_APPROACH_SLOPE, OPEN_RADIUS_PREFERENCE, OPEN_RADIUS_STEP_M,
                               OPEN_RIGHT_RADIUS_M, OPEN_LEFT_RADIUS_M, SIDE_BODY_EXTENT_M)
import autonomy_live  # noqa: E402
import server  # noqa: E402

FIXTURES = Path(__file__).parent/'fixtures'


def axis_lines_reference(points, heading):
    """The round 64 build's _axis_lines, verbatim (every window re-sorted)."""
    c, s = math.cos(-heading), math.sin(-heading)
    rows = sorted((-s*x+c*y, c*x+s*y) for x, y in points)
    lines, start = [], 0
    for end in range(len(rows)):
        while rows[end][0]-rows[start][0] > OPEN_LINE_BAND_M:
            start += 1
        group = rows[start:end+1]
        if len(group) < OPEN_LINE_MIN_POINTS:
            continue
        lateral = sorted(l for l, _ in group)[len(group)//2]
        along = sorted(a for _, a in group)
        runs, run = [], [along[0]]
        for a, b in zip(along, along[1:]):
            gap = max(OPEN_LINE_GAP_M, OPEN_LINE_GAP_SCALE*max(a*a, b*b)/max(abs(lateral), .3))
            if b-a > gap:
                runs.append(run)
                run = []
            run.append(b)
        runs.append(run)
        for run in runs:
            if len(run) < OPEN_LINE_MIN_POINTS or run[-1]-run[0] < OPEN_LANE_MIN_SPAN_M:
                continue
            line = {'lateral_m': lateral, 'along_m': [run[0], run[-1]], 'points': len(run), 'source': 'raw_returns'}
            same = [i for i, old in enumerate(lines) if abs(old['lateral_m']-lateral) <= OPEN_LINE_BAND_M
                    and old['along_m'][0] <= run[-1] and run[0] <= old['along_m'][1]]
            if not same:
                lines.append(line)
            elif len(run) > lines[same[0]]['points']:
                lines[same[0]] = line
    return lines


def plan_loop_reference(free, support_radius, radii, approach=None):
    """The round 64 build's _plan_loop, verbatim (no memo, _loop_extent calls)."""
    lo = support_radius+SIDE_BODY_EXTENT_M+OPEN_INNER_GAP_M
    bands = [(theta, distance-SIDE_BODY_EXTENT_M-OPEN_OUTER_GAP_M) for theta, distance in free]
    inner_floor = support_radius+SIDE_BODY_EXTENT_M if any(hi < lo for _, hi in bands) else lo

    def score(radius, a, b):
        if math.hypot(a, b) > radius-inner_floor:
            return -math.inf
        worst = math.inf
        for theta, hi in bands:
            t = _loop_extent(a, b, radius, theta)
            worst = min(worst, min(t-lo, hi-t) if hi >= lo else -abs(t-hi))
        if approach is not None and a-approach[0] > 0:
            worst -= max(0., OPEN_APPROACH_SLOPE*abs(approach[1]-(b-radius))-(a-approach[0]))
        return worst-.01*math.hypot(a, b)-OPEN_RADIUS_PREFERENCE*(radii[1]-radius)

    best = None
    radius = radii[0]
    while radius <= radii[1]+1e-9:
        a = b = 0.
        offsets = tuple(k*.05 for k in range(-math.ceil(radius/.05), math.ceil(radius/.05)+1))
        for _ in range(2):
            b = max(offsets, key=lambda v: score(radius, a, v))
            a = max(offsets, key=lambda v: score(radius, v, b))
        value = score(radius, a, b)
        if best is None or value > best[0]:
            best = (value, radius, a, b)
        radius += OPEN_RADIUS_STEP_M
    value, radius, a, b = best
    return {'radius_m': round(radius, 3), 'offset_axis_m': round(a, 3), 'offset_side_m': round(b, 3),
            'min_slack_m': value, 'inner_bound_m': lo, 'inner_floor_m': inner_floor,
            'board_slack_m': min(hi-_loop_extent(a, b, radius, theta) for theta, hi in bands),
            'extents_m': [round(_loop_extent(a, b, radius, theta), 3) for theta, _ in free]}


def recorded_ranges():
    """Every 360-bin scan in the JSON fixtures (real recordings)."""
    out = []

    def walk(value):
        if isinstance(value, dict):
            ranges = value.get('ranges')
            if isinstance(ranges, list) and len(ranges) == 360:
                out.append(ranges)
            for v in value.values():
                walk(v)
        elif isinstance(value, list):
            for v in value:
                walk(v)
    for path in sorted(FIXTURES.glob('*.json')):
        walk(json.loads(path.read_text()))
    return out


def board_scan(rng, heading):
    """Synthetic course-like returns: parallel boards, a crossing board, cones, noise."""
    points = []
    for lateral, start, end in ((.46, -1., 3.5), (-.45, -1., 2.6), (1.4, .5, 5.5)):
        for k in range(int((end-start)/.03)):
            along = start+k*.03
            if rng.random() < .1:
                continue
            l = lateral+rng.gauss(0., .008)
            points.append((along*math.cos(heading)-l*math.sin(heading), along*math.sin(heading)+l*math.cos(heading)))
    for k in range(40):
        points.append((3.+rng.gauss(0., .01), -1.+k*.05))
    for cx, cy in ((2.2, .9), (4.1, -.6)):
        for k in range(8):
            a = math.pi+k*.2
            points.append((cx+.1*math.cos(a), cy+.1*math.sin(a)))
    points += [(rng.uniform(-3, 5), rng.uniform(-3, 3)) for _ in range(15)]
    return [p for p in points if .02 <= math.hypot(*p) <= OPEN_LINE_REACH_M]


class AxisLinesEquivalence(unittest.TestCase):
    def test_recorded_scans_match_reference(self):
        scans = recorded_ranges()
        self.assertGreater(len(scans), 30)
        rng = random.Random(64)
        for ranges in scans:
            points = _scan_points(ranges, OPEN_LINE_REACH_M)
            for heading in (0., math.pi, .04, rng.uniform(-math.pi, math.pi)):
                self.assertEqual(_axis_lines(points, heading), axis_lines_reference(points, heading))

    def test_synthetic_boards_match_reference(self):
        rng = random.Random(7)
        found = 0
        for _ in range(60):
            heading = rng.uniform(-math.pi, math.pi)
            points = board_scan(rng, heading)
            for probe in (heading, heading+rng.gauss(0., .03), heading-math.pi):
                lines = _axis_lines(points, probe)
                self.assertEqual(lines, axis_lines_reference(points, probe))
                found += len(lines)
        self.assertGreater(found, 100)  # boards were actually found, not only empty lists


class PlanLoopEquivalence(unittest.TestCase):
    def test_random_probes_match_reference(self):
        rng = random.Random(11)
        for k in range(40):
            radii = OPEN_RIGHT_RADIUS_M if k % 2 else OPEN_LEFT_RADIUS_M
            free = [(theta, rng.uniform(.3, 2.5)) for theta in (-math.pi/2, -math.pi/4, 0., math.pi/4, math.pi/2)]
            approach = None if k % 3 == 0 else (rng.uniform(-2.5, .5), rng.uniform(-1.5, 1.5))
            support = rng.uniform(.05, .15)
            self.assertEqual(_plan_loop(free, support, radii, approach),
                             plan_loop_reference(free, support, radii, approach))


def scan(seq, ranges, at_ms=None):
    return {'seq': seq, 'at_ms': seq*100 if at_ms is None else at_ms, 'ranges': ranges}


class PerScanCaches(unittest.TestCase):
    def setUp(self):
        self.motion = ManeuverSequence(0., continue_route=True, max_drive_s=40.)
        self.ranges = recorded_ranges()[0]

    def test_axis_lines_cached_per_scan_and_heading(self):
        first = self.motion._scan_axis_lines(scan(5, self.ranges), .1)
        self.assertEqual(first, _axis_lines(_scan_points(self.ranges, OPEN_LINE_REACH_M), .1))
        self.assertIs(self.motion._scan_axis_lines(scan(5, list(self.ranges)), .1), first)
        self.assertEqual(self.motion._scan_axis_lines(scan(5, self.ranges), -.3),
                         _axis_lines(_scan_points(self.ranges, OPEN_LINE_REACH_M), -.3))

    def test_axis_lines_recomputed_when_the_content_changes_under_the_same_seq(self):
        # The first loop previews the second support on a copy of the frame
        # with the first support's returns removed, under the same seq.
        self.motion._scan_axis_lines(scan(5, self.ranges), 0.)
        edited = [None if 0 <= i < 40 else r for i, r in enumerate(self.ranges)]
        self.assertEqual(self.motion._scan_axis_lines(scan(5, edited), 0.),
                         _axis_lines(_scan_points(edited, OPEN_LINE_REACH_M), 0.))

    def test_route_proximity_follows_scan_content_and_first_pass(self):
        ranges = [2.]*360
        ranges[0] = .25  # head-on, inside the avoidance gap
        ranges[300] = .2  # left side: ignored before the first pass
        fresh = ManeuverSequence(0., continue_route=True, max_drive_s=40.)
        before = self.motion._route_proximity(scan(9, ranges))
        self.assertEqual(before, fresh._route_proximity_scan(scan(9, ranges), ranges))
        self.motion.first_pass_evidence = {'track_id': 1}
        fresh.first_pass_evidence = {'track_id': 1}
        after = self.motion._route_proximity(scan(9, ranges))
        self.assertEqual(after, fresh._route_proximity_scan(scan(9, ranges), ranges))
        self.assertNotEqual(before, after)
        moved = list(ranges)
        moved[0] = 2.
        self.assertEqual(self.motion._route_proximity(scan(9, moved)),
                         fresh._route_proximity_scan(scan(9, moved), moved))


class ProbeClearanceTables(unittest.TestCase):
    def test_bin_tables_equal_the_per_call_values(self):
        for angle in range(360):
            self.assertEqual(autonomy_live._BIN_COS[angle], math.cos(math.radians(angle)))
            self.assertEqual(autonomy_live._BIN_SIN[angle], math.sin(math.radians(angle)))


class LanReadPause(unittest.TestCase):
    def test_only_lan_reads_pause_and_only_during_a_run(self):
        session = {'report': {}}
        self.assertTrue(server.lan_read_paused('192.168.0.103', session))
        self.assertFalse(server.lan_read_paused('127.0.0.1', session))
        self.assertFalse(server.lan_read_paused('::1', session))
        self.assertFalse(server.lan_read_paused('192.168.0.103', None))
        self.assertTrue(server.lan_read_paused('not-an-address', session))


if __name__ == '__main__':
    unittest.main()
