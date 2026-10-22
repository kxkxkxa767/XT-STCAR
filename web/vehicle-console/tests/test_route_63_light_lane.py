"""Round 63 (Claude): light-lane straight like the start straight, earlier
hand-over from the light turn, coasting profile to the finish (round 101: aimed
at a whole-car stop, closed loop with an ESC delay), and a cheaper (bitwise
identical) free-space computation.
Real trials 60-62 light frames: fixtures/route-60-62-light-lane.json (saved
scans only). Real-frame stage tests re-time the saved 5 Hz frames to 100 ms;
geometry is unchanged. Closed-loop stops are a kinematic model, no physical
claim."""
import copy
import json
import math
from pathlib import Path
import time
import unittest

import test_maneuver_route as route
import maneuver_sequence as ms
from maneuver_sequence import (ManeuverSequence, LIGHT_HEADINGS_DEG, LIGHT_FREE_HORIZON_M, LIGHT_BRIDGE_MAX_BINS,
                               LIGHT_UNKNOWN_BLOCK_M, LIGHT_TURN_MIN_S, LIGHT_LANE_ENTRY_HEADING_RAD,
                               ROUTE_SLOW_MOTOR, FRONT_BODY_EXTENT_M, REAR_BODY_EXTENT_M, SIDE_BODY_EXTENT_M,
                               MANEUVER_BODY_CLEARANCE_M, _wrap)
from turn_motion import TRIAL_MOTOR
from test_route_58_light_lane import box

FIXTURE = json.loads((Path(__file__).parent/'fixtures'/'route-60-62-light-lane.json').read_text())


def frames(trial, first=None, last=None):
    """Saved frames of one trial in order, re-timed to consecutive 100 ms."""
    rows = [copy.deepcopy(v) for k, v in sorted(FIXTURE['scans'][str(trial)].items(), key=lambda kv: int(kv[0]))
            if (first is None or int(k) >= first) and (last is None or int(k) <= last)]
    for i, scan in enumerate(rows):
        scan['at_ms'], scan['received_at'] = 1000+100*i, 1.+.1*i
    return rows


def reference_free_space(scan, previous_heading=None):
    """The round-58 implementation, verbatim except for the module constants."""
    ranges = list(scan['ranges'])
    half = SIDE_BODY_EXTENT_M+MANEUVER_BODY_CLEARANCE_M
    step = LIGHT_HEADINGS_DEG[1]-LIGHT_HEADINGS_DEG[0]
    free = [LIGHT_FREE_HORIZON_M]*len(LIGHT_HEADINGS_DEG)
    left_clear = right_clear = None
    usable = any(r is not None for r in ranges)
    reach = LIGHT_HEADINGS_DEG[-1]+10
    start = None
    for angle in range(-reach, reach+2):
        missing = angle <= reach and ranges[(-angle) % 360] is None
        if missing and start is None:
            start = angle
        elif not missing and start is not None:
            edges = [ranges[(-(start-1)) % 360], ranges[(-angle) % 360]]
            bracketed = angle-start <= LIGHT_BRIDGE_MAX_BINS and None not in edges
            fill = min(edges) if bracketed else LIGHT_UNKNOWN_BLOCK_M
            for unknown in range(start, angle):
                ranges[(-unknown) % 360] = fill
            start = None
    for index, r in enumerate(ranges):
        if r is None:
            continue
        bearing = math.degrees(_wrap(-math.radians(index)))
        x, y = r*math.cos(math.radians(bearing)), r*math.sin(math.radians(bearing))
        if -REAR_BODY_EXTENT_M <= x <= 1.:
            if y > SIDE_BODY_EXTENT_M:
                left_clear = y if left_clear is None else min(left_clear, y)
            elif y < -SIDE_BODY_EXTENT_M:
                right_clear = -y if right_clear is None else min(right_clear, -y)
        if r >= LIGHT_FREE_HORIZON_M+half:
            continue
        spread = 90. if r <= half else math.degrees(math.asin(half/r))
        first = max(0, math.ceil((bearing-spread-LIGHT_HEADINGS_DEG[0])/step))
        last = min(len(LIGHT_HEADINGS_DEG)-1, math.floor((bearing+spread-LIGHT_HEADINGS_DEG[0])/step))
        for k in range(first, last+1):
            delta = math.radians(LIGHT_HEADINGS_DEG[k]-bearing)
            along = r*math.cos(delta)
            if along > 0 and abs(r*math.sin(delta)) <= half and along < free[k]:
                free[k] = along
    if not usable:
        return None
    deepest = max(free)
    windows, run = [], []
    for k, f in enumerate(free):
        if f >= .9*deepest:
            run.append(k)
        elif run:
            windows.append(run)
            run = []
    if run:
        windows.append(run)
    chosen = None
    if previous_heading is not None:
        previous = math.degrees(previous_heading)
        chosen = next((w for w in windows
                       if LIGHT_HEADINGS_DEG[w[0]]-step <= previous <= LIGHT_HEADINGS_DEG[w[-1]]+step), None)
    if chosen is None:
        chosen = max(windows, key=len)
    best = math.radians((LIGHT_HEADINGS_DEG[chosen[0]]+LIGHT_HEADINGS_DEG[chosen[-1]])/2)
    return {'best_heading_rad': best, 'deepest_free_m': deepest,
            'free_ahead_m': free[LIGHT_HEADINGS_DEG.index(0)],
            'left_clear_m': left_clear, 'right_clear_m': right_clear,
            'center_error_m': (None if left_clear is None or right_clear is None
                               else (left_clear-right_clear)/2)}


def lane_motion(stage='light_lane'):
    helper = route.ManeuverRouteTests()
    helper.setUp()
    motion = helper.route('second_orbit')
    motion.route_stage, motion.light_since = stage, 1.
    motion.second_pass_evidence = {'track_id': 2, 'source_seq': 4}
    # The route's own budget (the helper's 10 s trial default is not it): the
    # finish waits out the ESC lag between pushes (2026-10-16 measured car).
    motion.max_drive_s = ms.ROUTE_MAX_DRIVE_S
    return motion


# Round 101: front wheels over the finish line (zone .80 m, front overhang
# ~.08 m) and the whole car (.38 m) inside the zone.
LINE_GAP_M = .72
WHOLE_CAR_GAP_M = .80-(FRONT_BODY_EXTENT_M+REAR_BODY_EXTENT_M)


class FinishStopTests(unittest.TestCase):
    """Round 101 (user: 最后那段直道想办法让他一口气跑完不要停顿): a closed-loop lane
    end with the motor acting a pure ESC delay late both ways (round 64-94
    tests moved the car the scan a command was sent), coasting at
    ``decel``, 1550 a first-order lag to ``cruise`` (``tau``), 1560 to 2.2
    m/s; 100 ms scans of a centered 1.05 m lane, the car entering it
    powered at ``v0`` with ``start`` m of front gap. A kinematic model, no
    physical claim."""
    @staticmethod
    def stop(lag, decel, start, v0, *, cruise=1.15, tau=1.1, noise=0., seed=0):
        import random
        rng = random.Random(seed)
        motion = lane_motion()
        motion.servo = motion.steering_target = 1500
        motion.lane_motor = ROUTE_SLOW_MOTOR
        motion._power_since, motion._power_cmd_at = 0., .98  # the light turn drove at 1550
        motion._power_cmd_motor = ROUTE_SLOW_MOTOR
        gap, v, t, cmd, seq = start, v0, 1., ROUTE_SLOW_MOTOR, 5
        queue = [(0., ROUTE_SLOW_MOTOR)]
        ended, first_stop, moving, kicks, log = None, None, True, [], []
        for step in range(1500):
            t = round(1.+.02*step, 4)
            if step % 5 == 0 and ended is None:
                value = route.observation(seq, heading=0.)
                value['ranges'] = box(gap+FRONT_BODY_EXTENT_M+(rng.gauss(0, noise) if noise else 0.), half=.525)
                value['corridor_candidates'][0].update(center_offset_left_m=0., width_m=1.05)
                value['at_ms'], value['received_at'] = round(1000*t), t
                result = motion.update(value, 0., t, route.feedback(seq, motion.servo, cmd))
                new = result['motor'] if result['phase'] == 'drive' else 1500
                if new == TRIAL_MOTOR and cmd != TRIAL_MOTOR:
                    kicks.append([t, 0])
                if new == TRIAL_MOTOR:
                    kicks[-1][1] += 1
                if result['phase'] != 'drive':
                    ended = motion.reason
                cmd = new
                log.append((t, round(gap, 3), round(v, 3), cmd))
                seq += 1
            queue.append((t, cmd))
            while len(queue) > 1 and queue[1][0] <= t-lag:
                queue.pop(0)
            motor = queue[0][1]
            if motor == ROUTE_SLOW_MOTOR and v > .05:
                v += max(-decel, min(2.5, (cruise-v)/tau))*.02
            elif motor == TRIAL_MOTOR:
                v += max(-decel, min(2.5, (2.2-v)/1.1))*.02
            else:
                v = max(0., v-decel*.02)
            gap -= v*.02
            if v <= 0 and moving and first_stop is None:
                first_stop = gap
            moving = v > 0
            if gap <= 0 or (ended is not None and v <= 0):
                break
        return {'gap': gap, 'first_stop': first_stop, 'ended': ended, 'kicks': kicks, 'log': log,
                'evidence': motion.finish_evidence}

    @staticmethod
    def feasible(lag, decel, start, v0):
        """Whether a cut on entry still stops before the board."""
        return v0*lag+v0*v0/(2*decel) < start

    def test_one_go_to_a_whole_car_stop_without_contact(self):
        rows = []
        for lag in (.2, .3, .45):
            for decel in (.5, .58, .65):
                for cruise, tau in ((1.15, 1.1), (1.5, 3.5)):
                    for start, v0 in ((3.3, 1.5), (3.1, 1.7), (3.4, 1.5), (4.5, 1.2), (5.5, 1.), (2.5, 1.)):
                        if not self.feasible(lag, decel, start, v0):
                            continue
                        row = self.stop(lag, decel, start, v0, cruise=cruise, tau=tau)
                        rows.append(row)
                        with self.subTest(lag=lag, decel=decel, cruise=cruise, start=start, v0=v0):
                            self.assertEqual(row['ended'], 'route_finish_zone_stop')
                            self.assertGreater(row['gap'], 0.)  # no board contact
                            # No standstill before the front wheels cross the line.
                            self.assertLessEqual(row['first_stop'], LINE_GAP_M)
                            self.assertLess(row['gap'], .5)  # the front well inside the zone
        # 75 % at the time of writing (84 % before late pushes were guarded at
        # LANE_PUSH_LATE_ACCEL_MAX_MPS2); the rest stopped .42-.5 m from the board
        # (front well in, rear wheels just outside), none before the line.
        whole = sum(row['gap'] <= WHOLE_CAR_GAP_M for row in rows)
        self.assertGreaterEqual(whole, .7*len(rows), (whole, len(rows)))

    def test_noisy_ranges_still_never_reach_the_board(self):
        for seed in range(2):
            for lag in (.2, .45):
                for decel in (.5, .62):
                    for start, v0 in ((3.3, 1.5), (4.5, 1.2), (2.5, 1.)):
                        row = self.stop(lag, decel, start, v0, noise=.01, seed=seed)
                        with self.subTest(seed=seed, lag=lag, decel=decel, start=start, v0=v0):
                            self.assertGreater(row['gap'], 0.)
                            self.assertLessEqual(row['first_stop'], LINE_GAP_M)

    def test_restarts_are_full_kicks_rested_apart(self):
        # Round 94 (car runs 91/93): 2-frame pulses barely moved the car; every
        # 1560 restart is one LANE_KICK_FRAMES kick; round 101: the next one
        # no sooner than LANE_KICK_REST_S after it (a moving car read 0 m/s).
        for decel in (.5, .7, 1.):
            row = self.stop(.3, decel, 1.9, .2, cruise=.6)
            with self.subTest(decel=decel):
                self.assertEqual(row['ended'], 'route_finish_zone_stop')
                self.assertGreater(row['gap'], 0.)
                self.assertTrue(all(frames <= ms.LANE_KICK_FRAMES for _, frames in row['kicks']), row['kicks'])
                starts = [at for at, _ in row['kicks']]
                self.assertTrue(all(b-a >= ms.LANE_KICK_REST_S for a, b in zip(starts, starts[1:])), starts)

    def test_weaker_or_stronger_coast_never_reaches_the_board(self):
        for decel in (.35, .8, 1.):
            for lag in (.2, .45):
                for start, v0 in ((4.5, 1.2), (5.5, 1.), (1.9, .6)):
                    row = self.stop(lag, decel, start, v0)
                    with self.subTest(decel=decel, lag=lag, start=start, v0=v0):
                        self.assertGreater(row['gap'], 0.)
                        self.assertEqual(row['ended'], 'route_finish_zone_stop')

    def test_far_part_is_driven_at_1550_and_the_measured_coast_aims_the_stop(self):
        row = self.stop(.3, .58, 5.5, 1.)  # box() returns at most 6 m
        # Far = beyond the guard's reach at ~1.1 m/s before a coast is measured.
        far = [motor for t, gap, v, motor in row['log'] if gap > 3.6]
        self.assertTrue(len(far) > 10 and all(motor == ROUTE_SLOW_MOTOR for motor in far[1:]), far)
        evidence = row['evidence']
        self.assertGreater(row['gap'], 0.)
        self.assertLess(evidence['front_gap_m'], .5)
        self.assertTrue(evidence['coast_decel_measured'])
        self.assertAlmostEqual(evidence['coast_decel_mps2'], .58, delta=.06)
        self.assertFalse(evidence['finish_zone_certified'])


if __name__ == '__main__':
    unittest.main()
