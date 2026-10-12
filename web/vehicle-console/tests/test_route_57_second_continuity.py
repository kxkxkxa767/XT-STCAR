"""Round 57 (Claude): second-support identity continuity and lane-frame bypass.

Real trial56 frames come from fixtures/route-56-second-identity.json (saved
scans only; raw 2244/2245/2247/2248 were never saved and are not invented).
The closed-loop check at the end is synthetic kinematics, not a car replay.
"""
import copy
import json
import math
from pathlib import Path
import unittest

import test_maneuver_route as route
import compact_target as ct
from compact_target import CompactTargetTracker, _associated, _candidates
from maneuver_sequence import ManeuverSequence, ROUTE_RETURN_RELEASE_PWM_STEP

FIXTURE = json.loads((Path(__file__).parent/'fixtures'/'route-56-second-identity.json').read_text())


def real(seq, received=None):
    scan = copy.deepcopy(FIXTURE['scans'][str(seq)])
    scan['received_at'] = scan['at_ms']/1000 if received is None else received
    return scan


def second_tracker():
    # The route's second-role tracker configuration (maneuver_sequence._new_second_tracker).
    return CompactTargetTracker(acquisition_bearing_rad=(-math.pi/2, math.pi/2),
                                maintenance_bearing_rad=(-math.pi, math.pi/2), select_associated=True,
                                unconfirmed_boundary_gap=True, tolerant=True)


class SecondIdentityContinuityTests(unittest.TestCase):
    def confirmed_far(self, at_ms):
        """Synthetic confirmation of a support where real scan2225's cluster
        is (2.18 m, -6 deg), ending 100 ms before the real frame."""
        tracker = second_tracker()
        for k, dt in enumerate((-.35, -.2, -.1)):
            t = at_ms/1000+dt
            scan = route.compact_scan(k+1, round(t*1000), t, bearing=-6., distance=2.18, size=5)
            scan['ranges'] = [6. if r == 3. else r for r in scan['ranges']]
            target = tracker.update(scan, t)
        self.assertTrue(target['confirmed'])
        return tracker, target

    def test_real_three_return_far_frames_keep_a_confirmed_identity_only(self):
        # Trial56 2225/2227: 3 returns, valid diameter and farther background.
        for seq in (2225, 2227):
            r = real(seq)['ranges']
            self.assertEqual(_candidates(r, bearing_range_rad=(-math.pi/2, math.pi/2), tolerant=True,
                                         allow_boundary_gap=True), [])  # acquisition unchanged
        tracker, old = self.confirmed_far(real(2225)['at_ms'])
        for seq in (2225, 2227, 2229):
            scan = real(seq)
            current = tracker.update(scan, scan['received_at']+.01, acquire=False)
            self.assertIsNotNone(current, (seq, tracker.reason))
            self.assertEqual(current['track_id'], old['track_id'])
            self.assertTrue(current['confirmed'])
            self.assertFalse(current['tracking_only_boundary_gap'])
        # An unconfirmed tracker never acquires from the same 3-return frames.
        fresh = second_tracker()
        for seq in (2223, 2225, 2227):
            scan = real(seq)
            self.assertIsNone(fresh.update(scan, scan['received_at']+.01))

    def test_real_near_support_passing_the_body_associates_by_distance(self):
        # Trial56: last complete 2243 (.378 m, -52.5 deg) and complete 2246
        # (.399 m, -121.8 deg) .30 s later: moved .44 m, bearing 69.3 deg.
        def support(seq, bearing):
            found = [c for c in ct._candidates(real(seq)['ranges'], bearing_range_rad=(-math.pi, math.pi),
                                               tolerant=True, allow_boundary_gap=True, maintenance=True,
                                               min_range_m=.02)
                     if abs(math.degrees(c['bearing_left_rad'])-bearing) < 2]
            self.assertEqual(len(found), 1)
            return found[0]
        old, new = support(2243, -52.5), support(2246, -121.8)
        slack = (real(2246)['at_ms']-real(2243)['at_ms'])/1000-.1
        delta = abs(math.degrees(new['bearing_left_rad']-old['bearing_left_rad']))
        self.assertGreater(delta, 20+100*slack)  # the old fixed angle gate rejected it
        self.assertLess(math.dist(new['point_left_m'], old['point_left_m']), .30+1.5*slack)
        self.assertTrue(_associated(new, old, slack))
        # A far support with the same bearing change is still rejected (distance).
        far = {**new, 'point_left_m': [2.*math.cos(new['bearing_left_rad']), 2.*math.sin(new['bearing_left_rad'])]}
        far_old = {**old, 'point_left_m': [2.*math.cos(old['bearing_left_rad']), 2.*math.sin(old['bearing_left_rad'])]}
        self.assertFalse(_associated(far, far_old, slack))

    def test_range_dependent_gate_leaves_far_association_unchanged(self):
        # Beyond ~.86 m the rotation gate (20 deg) is the larger bound.
        old = {'point_left_m': [2., 0.], 'bearing_left_rad': 0., 'diameter_m': .15}
        for bearing, expected in ((5., True), (25., False)):
            current = {'point_left_m': [2.*math.cos(math.radians(bearing)), 2.*math.sin(math.radians(bearing))],
                       'bearing_left_rad': math.radians(bearing), 'diameter_m': .15}
            self.assertEqual(_associated(current, old), expected and math.dist(current['point_left_m'], old['point_left_m']) <= .30)


class LaneFrameBypassTests(unittest.TestCase):
    def motion(self, *, width=2.1, car_y=0., heading=0.):
        """``heading`` is the lane axis relative to the car (deg, left +)."""
        h = route.ManeuverRouteTests()
        h.setUp()
        motion = h.route('second_orbit')
        motion.second_track_id, motion.second_orbit_since = 2, .4
        motion.second_center_ack = copy.deepcopy(motion._route_center_ack)
        self.width, self.car_y, self.heading = width, car_y, math.radians(heading)
        return motion

    def scan(self, seq, bearing, distance):
        value = route.observation(seq, heading=math.degrees(self.heading))
        value['ranges'] = route.compact_scan(bearing=bearing, distance=distance, size=5 if distance > 1.5 else 9)['ranges']
        value['ranges'] = [6. if r == 3. else r for r in value['ranges']]
        value['corridor_candidates'][0].update(center_offset_left_m=-self.car_y, width_m=self.width)
        return value

    def update(self, motion, seq, bearing, distance, lane=True):
        value = self.scan(seq, bearing, distance)
        if not lane:
            value['corridor_candidates'] = []
        c = _candidates(value['ranges'], bearing_range_rad=(-math.pi/2, math.pi/2), tolerant=True)[0]
        motion.second_target = {**c, 'track_id': 2, 'confirmed': True, 'source_seq': seq,
                                'source_at_ms': seq*100, 'source_received_at': seq/10}
        return motion._second_orbit_update(value, seq/10, route.feedback(seq, motion.servo, 1550))

    def test_far_support_ahead_takes_the_offset_early_and_is_never_weaker_than_pursuit(self):
        result = self.update(self.motion(), 5, -4., 2.3)
        feedback = result['second_feedback']
        lane = feedback['lane_bypass']
        self.assertIsNotNone(lane)
        pursuit = 1500+round(205*feedback['pursuit_curvature_per_m'])
        self.assertGreater(lane['nominal_target_pwm'], pursuit)  # trial56: pursuit ~1534 at 2.3 m
        self.assertEqual(feedback['nominal_target_pwm'], max(pursuit, lane['nominal_target_pwm']))
        # Near and not yet offset: the pursuit's strong left still governs (trial48).
        near = self.update(self.motion(), 6, 1., .9)['second_feedback']
        self.assertGreaterEqual(near['nominal_target_pwm'], 1680)

    def test_on_the_offset_line_the_car_runs_parallel_and_no_lane_keeps_pursuit(self):
        # Support .55 m to the right of the car path: the .40 m line is reached.
        motion = self.motion()
        result = self.update(motion, 5, -math.degrees(math.atan2(.6, 1.2)), math.hypot(.6, 1.2))
        self.assertLessEqual(result['second_feedback']['lane_bypass']['nominal_target_pwm'], 1500)
        result = self.update(self.motion(), 5, -4., 2.3, lane=False)
        self.assertIsNone(result['second_feedback']['lane_bypass'])
        self.assertEqual(result['second_feedback']['scope'], 'current_support_relative_point_trial')

    def test_lane_room_limits_the_bypass_offset(self):
        motion = self.motion(width=1.0)  # room left of center: .5-.22 = .28 m
        lane = self.update(motion, 5, -2., 2.)['second_feedback']['lane_bypass']
        self.assertAlmostEqual(lane['desired_offset_m'], lane['lane_room_left_m'])
        self.assertLess(lane['desired_offset_m'], lane['wanted_offset_m'])

    def test_bypass_stops_adding_left_once_the_car_is_turned_past_the_heading_limit(self):
        # Car already 40 deg left of the lane, support .9 m ahead: no more left
        # (a board, not the cone, would end the run); aligned car unchanged.
        turned = self.update(self.motion(heading=-40.), 5, 1., .9)['second_feedback']
        self.assertLessEqual(turned['nominal_target_pwm'], 1500)
        self.assertIn('heading_limited_target_pwm', turned)
        aligned = self.update(self.motion(heading=-3.), 5, 1., .9)['second_feedback']
        self.assertGreaterEqual(aligned['nominal_target_pwm'], 1680)
        self.assertNotIn('heading_limited_target_pwm', aligned)

    def test_return_release_steps_are_larger_but_release_only(self):
        motion = self.motion()
        motion.servo, motion.steering_target = 1720, 1500
        motion.second_return_preparing = True
        motion.handover_observed = True
        motion.last_change = 0.
        motion._current_scan = self.scan(5, -100., .5)
        motion._slew(1.)
        self.assertEqual(motion.servo, 1720-ROUTE_RETURN_RELEASE_PWM_STEP)
        motion.second_return_preparing = False
        motion.servo, motion.steering_target, motion.last_change = 1720, 1500, 0.
        motion._slew(1.)
        self.assertEqual(motion.servo, 1720-40)


class SyntheticBypassClosedLoopTests(unittest.TestCase):
    """Straight lane, support ahead, .3 s steering lag, residual right turn.
    Kinematic model only: curvature (servo-1500)/160 per m, 1.25 m/s."""

    @staticmethod
    def cast(X, Y, psi, cone, W, end_x):
        ranges = []
        for i in range(360):
            a = psi-math.radians(i)
            dx, dy = math.cos(a), math.sin(a)
            best = 6.
            for wy in (W/2, -W/2):
                if abs(dy) > 1e-9:
                    t = (wy-Y)/dy
                    if t > 0 and -3 <= X+t*dx <= end_x:
                        best = min(best, t)
            if abs(dx) > 1e-9:
                t = (end_x-X)/dx
                if t > 0 and abs(Y+t*dy) <= W/2:
                    best = min(best, t)
            fx, fy = X-cone[0], Y-cone[1]
            b, c = fx*dx+fy*dy, fx*fx+fy*fy-cone[2]**2
            if b*b-c >= 0 and -b-math.sqrt(b*b-c) > 0:
                best = min(best, -b-math.sqrt(b*b-c))
            ranges.append(round(best, 3))
        return ranges

    def run_bypass(self, lane_law, start=2.3):
        h = route.ManeuverRouteTests()
        h.setUp()
        motion = h.route('second_orbit')
        motion.second_track_id, motion.second_orbit_since = 2, 0.
        motion.second_center_ack = copy.deepcopy(motion._route_center_ack)
        if not lane_law:
            # Old behavior: no lane frame, no heading limit (pursuit only).
            motion._current_lane = lambda *args: (None, None)
        W, V, cone = 2.1, 1.25, (4., 0., .1)
        X, Y, psi = cone[0]-start, 0., 0.
        max_heading = 0.
        motion.servo = motion.steering_target = 1500
        pipeline = [1460]*3  # .3 s lag, still turning right from the exit
        abeam, min_gap = None, 9.
        for k in range(1, 40):
            scan = {'frame_id': 'lidar_origin_coarse_body_heading', 'seq': k, 'at_ms': 100*k,
                    'received_at': .1*k, 'ranges': self.cast(X, Y, psi, cone, W, 6.)}
            c = route.corridor(0.)
            c.update(heading_left_rad=-psi, center_offset_left_m=-Y, width_m=W)
            scan['corridor_candidates'] = [c]
            local = (math.cos(-psi)*(cone[0]-X)-math.sin(-psi)*(cone[1]-Y),
                     math.sin(-psi)*(cone[0]-X)+math.cos(-psi)*(cone[1]-Y))
            found = _candidates(scan['ranges'], bearing_range_rad=(-math.pi, math.pi), tolerant=True,
                                allow_boundary_gap=True, maintenance=True, min_range_m=.02)
            if not found:
                break  # support merged with the body outline: contact, stop the check
            target = min(found, key=lambda d: math.dist(d['point_left_m'], local))
            motion.second_target = {**target, 'track_id': 2, 'confirmed': True, 'source_seq': k,
                                    'source_at_ms': 100*k, 'source_received_at': .1*k}
            motion._current_scan = scan
            result = motion._second_orbit_update(scan, .1*k, route.feedback(k, motion.servo, 1550))
            applied = pipeline.pop(0)
            pipeline.append(motion.servo)
            kappa = (applied-1500)/160
            for _ in range(10):
                psi += kappa*V*.01
                X += V*.01*math.cos(psi)
                Y += V*.01*math.sin(psi)
            for j in range(36):
                px, py = cone[0]+.1*math.cos(j*math.pi/18), cone[1]+.1*math.sin(j*math.pi/18)
                x = math.cos(-psi)*(px-X)-math.sin(-psi)*(py-Y)
                y = math.sin(-psi)*(px-X)+math.cos(-psi)*(py-Y)
                min_gap = min(min_gap, math.hypot(max(-.18-x, 0, x-.20), max(abs(y)-.14, 0)))
            max_heading = max(max_heading, math.degrees(psi))
            if abeam is None and X >= cone[0]:
                abeam = math.degrees(psi)
            if result['route_stage'] != 'second_orbit' or X > cone[0]+.8:
                break
        return abeam, min_gap, max_heading

    def test_lane_frame_bypass_passes_more_parallel_with_clearance(self):
        new_heading, new_gap, _ = self.run_bypass(True)
        old_heading, old_gap, _ = self.run_bypass(False)
        self.assertGreaterEqual(new_gap, .15)
        self.assertLess(abs(new_heading), 10.)
        self.assertLess(abs(new_heading), abs(old_heading)-5.)

    def test_late_confirmation_turns_the_car_far_less_toward_the_board(self):
        # Trial56 class: support confirmed only at ~1.45 m.
        _, _, new_max = self.run_bypass(True, start=1.45)
        _, _, old_max = self.run_bypass(False, start=1.45)
        self.assertLess(new_max, 80.)
        self.assertLess(new_max, old_max-30.)


if __name__ == '__main__':
    unittest.main()
