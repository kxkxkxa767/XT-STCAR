"""Round 74 (2026-10-18): the open transfer after the first loop passed the
.12 s control gap (a .100 s tick + a fixed 20 ms wait). The component search
is shared per frame content, the transfer's board lines are computed only for
band candidates, and the loop waits the rest of 20 ms (at least 5 ms)."""
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).parents[1]))
import compact_target as ct  # noqa: E402
import server  # noqa: E402

FIXTURES = Path(__file__).parent/'fixtures'


def recorded_ranges():
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


class GroupCache(unittest.TestCase):
    def test_cached_components_equal_fresh_ones(self):
        scans = recorded_ranges()
        self.assertGreater(len(scans), 30)
        for ranges in scans:
            for kw in ({'tolerant': True}, {}, {'tolerant': True, 'allow_boundary_gap': True},
                       {'tolerant': True, 'max_range_m': ct.EXTENDED_MAX_RANGE_M}):
                ct._GROUP_CACHE.clear()
                fresh = ct._candidates(ranges, **kw)
                again = ct._candidates(ranges, **kw)  # served from the cache
                self.assertEqual(fresh, again)
                ct._GROUP_CACHE.clear()
                self.assertEqual(ct._candidates(list(ranges), **kw), fresh)

    def test_changed_content_is_recomputed(self):
        ranges = recorded_ranges()[0]
        ct._candidates(ranges, tolerant=True)
        hidden = [None if 250 <= i <= 290 else r for i, r in enumerate(ranges)]
        ct._GROUP_CACHE.clear()
        expected = ct._candidates(hidden, tolerant=True)
        ct._candidates(ranges, tolerant=True)
        self.assertEqual(ct._candidates(hidden, tolerant=True), expected)


class LoopPacing(unittest.TestCase):
    def watch(self, clock):
        console = server.Console.__new__(server.Console)
        console.lock = __import__('threading').Lock()
        console.auto_session = {'report': {'run_id': 'r'}}
        console.auto_result = None
        waits = []
        console.stop = type('S', (), {'wait': lambda self, t: (waits.append(t), len(waits) > 2)[1]})()
        console.autonomy_tick = lambda now: None
        with patch.object(server.time, 'monotonic', side_effect=clock):
            console.autonomy_watch()
        return waits

    def test_long_tick_waits_only_the_minimum(self):
        waits = self.watch([1., 1.001, 1.101, 2., 2.001, 2.006])
        self.assertAlmostEqual(waits[0], server.AUTONOMY_TICK_S)
        self.assertAlmostEqual(waits[1], server.AUTONOMY_MIN_IDLE_S)  # .101 s tick
        self.assertAlmostEqual(waits[2], server.AUTONOMY_TICK_S-.006)  # 6 ms tick


if __name__ == '__main__':
    unittest.main()
