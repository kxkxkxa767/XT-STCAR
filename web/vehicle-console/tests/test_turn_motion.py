"""No hardware: native geometry, adoption evidence and bounded left trial checks."""
import importlib.util
import json
import math
from pathlib import Path
import unittest

SPEC = importlib.util.spec_from_file_location('turn_motion', Path(__file__).parents[1]/'turn_motion.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def corridor(heading=60., offset=0., width=1.2):
    return {'heading_left_rad': math.radians(heading), 'center_offset_left_m': offset,
            'width_m': width, 'left_wall_points': 40, 'right_wall_points': 45,
            'support_span_m': .8, 'fit_error_m': .02, 'origin_between_walls': True,
            'candidate_only': True, 'turn_path_certified': False}


def scan(seq, heading=60., offset=0., width=1.2, published=None):
    forward = corridor(heading, offset, width)
    backward = corridor(heading-180., -offset, width)
    return {'seq': seq, 'at_ms': seq*100 if published is None else published,
            'frame_id': 'lidar_origin_coarse_body_heading', 'navigation_validated': False,
            'corridor_candidates': [forward, backward]}


def control(seq, servo=1500, armed=True, motor=1500, tick=None):
    return {'seq': seq, 'tick': seq*100 if tick is None else tick,
            'armed': armed, 'motor': motor, 'servo': servo}


def wall(heading=60., rho=-1.8, start=1., end=3.):
    theta = math.radians(heading)
    def point(t):
        return {'x_m': math.cos(theta)*t-math.sin(theta)*rho,
                'y_m': math.sin(theta)*t+math.cos(theta)*rho}
    return {'heading_left_rad': theta, 'rho_left_m': rho,
            'support_start_left_m': point(start), 'support_end_left_m': point(end),
            'support_span_m': end-start, 'points': 32, 'fit_error_m': .02, 'candidate_only': True}


def opening_scan(seq, endpoint_index=254):
    value = dict(scan(seq), corridor_candidates=[])
    theta, target_range = math.radians(30.), 2.4
    outer = wall()
    value['ranges'] = [3.2]*360
    end_angle = MODULE._wrap(math.radians(-endpoint_index))
    end_range = .5/math.sin(end_angle)
    end_point = {'x_m': end_range*math.cos(end_angle), 'y_m': .5}
    value['ranges'][endpoint_index] = end_range
    value['wall_candidates'] = [outer]
    value['left_turn_goal'] = {'heading_left_rad': math.radians(60.), 'center_offset_left_m': -1.2,
                              'width_m': 1.2, 'target_point_left_m': {'x_m': target_range*math.cos(theta),
                                                                   'y_m': target_range*math.sin(theta)},
                              'target_support_ray': {'index': 330, 'angle_left_rad': theta,
                                                     'range_m': 3.2, 'target_range_m': target_range},
                              'outer_wall': outer, 'incoming_heading_left_rad': 0., 'front_wall_m': 2.2,
                              'incoming_left_end_m': end_point['x_m'], 'origin_between_exit_walls': False,
                              'incoming_left_end_support': {'index': endpoint_index, 'angle_left_rad': end_angle,
                                                           'range_m': end_range, 'point_left_m': end_point},
                              'known_open_fraction': 1.,
                              'candidate_only': True, 'turn_path_certified': False,
                              'observation_type': 'left_opening_outer_wall_alignment'}
    return value


def recorded_opening_scan(seq, actual_board=False):
    # Compact actual observations from the 2026-10-07 trial, not vehicle pose
    # or wheel feedback. Only the goal-support ray is needed by this adapter;
    # the full raw scans remain in ignored work/.
    records = {
        5403: {
            'at_ms': 539722,
            'outer': (1.5646674578789872, -2.1964654010888034, 1.554855844854365,
                      0.018679049943563054, 27, (2.2011344707415534, 0.8893160527693753),
                      (2.2007705643142836, 2.44420333099515)),
            'goal': (1.6230809417647114, -1.3849249302064477, 0.022178004105570397,
                     2.157825186262612, 0.5347442444979006, (1.3929118987384266, 1.2989113592342365),
                     317, 3.04, 1.9045666900360356)},
        5411: {
            'at_ms': 540522,
            'outer': (1.5656449523392322, -2.1965240856932935, 1.5548647729048408,
                      0.02746331575072961, 26, (2.201134470741553, 0.8893160527693753),
                      (2.200770564314284, 2.44420333099515)),
            'goal': (1.6218090389922744, -1.3856195661971564, 0.024438574913648002,
                     2.1578817650840327, 0.5360727260917584, (1.3925655029858237, 1.3447848744169772),
                     316, 3.087, 1.9358938603567206)},
        5412: {
            'at_ms': 540621,
            'outer': (1.5580785672907735, -2.187710719998273, 1.5718263374867365,
                      0.021324689176498685, 26, (2.1872267129230516, 0.8836969538681367),
                      (2.2108075234096662, 2.455350503377311)),
            'goal': (1.6241504404991778, -1.3756354997486842, 0.020639609241906953,
                     2.1579890453384896, 0.5338386048393119, (1.392259147086357, 1.2983026584711803),
                     317, 3.04, 1.9036741647768833)}
    }
    records.update({4129: {'at_ms': 412301,
            'outer': (1.5541714539283245,
                      -2.184040846535792,
                      1.2449977539018375,
                      0.02169008347781507,
                      24,
                      (2.199276206844011, 0.7572715272673459),
                      (2.2234893178283643, 2.0020387742257033)),
            'goal': (1.6377899545469763,
                     -1.365145869262304,
                     -0.0030299029696582136,
                     2.169437960584625,
                     0.5316480060376487,
                     (1.3846521173625657, 1.1618610809877687),
                     320,
                     2.87,
                     1.807534967194467)}})
    records[886] = {'at_ms': 88530,
            'outer': (1.5831126523694827, -2.1938025609561134, 1.3305570554044794,
                      0.018386116451866493, 25, (2.183644617576955, 0.8382226339864579),
                      (2.168696497899141, 2.168696497899141)),
            'goal': (1.6304507692778052, -1.3785771763172108, 0.01832409115258497,
                     2.1676119650966146, 0.5371611958188093, (1.3630263756039962, 1.271042658054264),
                     317, 2.974, 1.8637033935650316)}
    # Same-scan endpoint rays derived from the captured native end scalar.
    end_support = {5403: (311, 0.7949999999999999), 5411: (311, 0.7949999999999999), 5412: (311, 0.7949999999999999), 4129: (317, 0.729), 886: (314, .759)}
    record = records[seq]
    heading, rho, span, error, points, start, end = record['outer']
    outer = {'heading_left_rad': heading, 'rho_left_m': rho, 'support_span_m': span,
             'fit_error_m': error, 'points': points, 'candidate_only': True,
             'support_start_left_m': dict(zip(('x_m', 'y_m'), start)),
             'support_end_left_m': dict(zip(('x_m', 'y_m'), end))}
    width, offset, incoming, front, left_end, target, index, distance, target_distance = record['goal']
    ray_angle = (math.pi-math.radians(index)) % (2*math.pi)-math.pi
    ranges = [None]*360
    ranges[index] = distance
    end_index, end_distance = end_support[seq]
    end_angle = MODULE._wrap(math.radians(-end_index))
    ranges[end_index] = end_distance
    endpoint = {'index': end_index, 'angle_left_rad': end_angle, 'range_m': end_distance,
                'point_left_m': {'x_m': end_distance*math.cos(end_angle),
                                 'y_m': end_distance*math.sin(end_angle)}}
    if actual_board:
        if seq != 5412:
            raise ValueError('only the captured 5412 board trace is included')
        # Actual quantized board/corner returns, including unknown bins. Other
        # sectors stay unknown; this is evidence for surface identity only.
        board = (
            0.667, 0.6829999999999999, 0.6829999999999999, 0.7, 0.7, 0.715, 0.731, 0.747, 0.747, 0.763,
            0.78, 0.7949999999999999, 3.304, 3.273, 3.181, 3.134, 3.087, 3.04, 2.962, 2.915,
            2.887, 2.84, 2.777, 2.761, 2.73, 2.715, 2.652, 2.637, 2.605, 2.589,
            2.543, 2.527, 2.497, 2.483, 2.436, 2.42, 2.405, 2.39, 2.359, 2.343,
            2.343, 2.328, 2.296, 2.296, 2.282, 2.266, 2.251, 2.251, 2.251, 2.235,
            2.219, 2.219, 2.204, 2.204, 2.188, 2.173, 2.173, 2.158, 2.158, 2.158,
            2.158, 2.158, 2.158, 2.158, 2.158, 2.158, 2.158, 2.173, 2.173, 2.172,
            2.172, None, None, 2.188, 2.204, 2.204, 2.142, 2.034, 1.942, 1.8639999999999999,
        )
        for position, board_distance in enumerate(board):
            ranges[(300+position) % 360] = board_distance
    goal = {'heading_left_rad': heading, 'center_offset_left_m': offset, 'width_m': width,
            'incoming_heading_left_rad': incoming, 'front_wall_m': front, 'incoming_left_end_m': left_end,
            'outer_wall': outer, 'target_point_left_m': dict(zip(('x_m', 'y_m'), target)),
            'incoming_left_end_support': endpoint, 'known_open_fraction': 1.,
            'target_support_ray': {'index': index, 'angle_left_rad': ray_angle,
                                   'range_m': distance, 'target_range_m': target_distance},
            'candidate_only': True, 'turn_path_certified': False, 'origin_between_exit_walls': False,
            'observation_type': 'left_opening_outer_wall_alignment'}
    walls = [outer]
    if seq == 5412:
        walls.append({'candidate_only': True,
                      'fit_error_m': 0.014604957177516793,
                      'heading_left_rad': 1.5138636694540555,
                      'points': 24,
                      'rho_left_m': -2.1570072453451923,
                      'support_end_left_m': {'x_m': 2.193599190131244, 'y_m': 0.6290044459743911},
                      'support_span_m': 0.8944721136357403,
                      'support_start_left_m': {'x_m': 2.156802787516593, 'y_m': -0.26482208321938533}})
    return {'seq': seq, 'at_ms': record['at_ms'], 'frame_id': 'lidar_origin_coarse_body_heading',
            'navigation_validated': False, 'ranges': ranges, 'corridor_candidates': [],
            'wall_candidates': walls, 'left_turn_goal': goal}


def surface_scan(theta_deg=90., kind='curve', noise=0., separation=.03):
    """Raycast observed surfaces and fit real subsets; no physical pose feedback."""
    theta = math.radians(theta_deg)
    c, s = math.cos(theta), math.sin(theta)
    ranges, points = [None]*360, []
    for i in range(360):
        angle = math.radians(-i)
        normal, along_ray = math.sin(angle-theta), math.cos(angle-theta)
        if normal >= -.02:
            continue
        distance = -2/normal
        for _ in range(10):
            along = distance*along_ray
            if kind in ('curve', 'gap'):
                offset, derivative = -2+.04*along*along, .08*along*along_ray
            elif kind == 'kink':
                slope = math.tan(math.radians(17.5))
                offset, derivative = -2+abs(along)*slope, math.copysign(slope, along)*along_ray
            else:
                offset, derivative = -2., 0.
            distance -= (distance*normal-offset)/(normal-derivative)
        along = distance*along_ray
        if not .02 <= distance <= 12 or not -1.5 <= along <= 1.5:
            continue
        if kind == 'gap' and abs(along) < .1:
            continue
        distance += noise*math.sin(i*2.3)
        if kind == 'parallel':
            distance = (2+(separation if i % 2 else 0))/-normal
        ranges[i] = distance
        points.append((i, distance*math.cos(angle), distance*math.sin(angle), distance*along_ray))

    def fitted(low, high, parity=None):
        values = [p for p in points if low <= p[3] <= high and (parity is None or p[0] % 2 == parity)]
        xs, ys = [c*p[1]+s*p[2] for p in values], [-s*p[1]+c*p[2] for p in values]
        x, y = sum(xs)/len(xs), sum(ys)/len(ys)
        slope = sum((a-x)*(b-y) for a, b in zip(xs, ys))/sum((a-x)**2 for a in xs)
        intercept = y-slope*x
        heading = MODULE._wrap(theta+math.atan(slope))
        hc, hs = math.cos(heading), math.sin(heading)
        rho = intercept/math.hypot(1, slope)
        error = max(abs(-hs*p[1]+hc*p[2]-rho) for p in values)
        first = min(values, key=lambda p: hc*p[1]+hs*p[2])
        last = max(values, key=lambda p: hc*p[1]+hs*p[2])
        return {'candidate_only': True, 'heading_left_rad': heading, 'rho_left_m': rho,
                'fit_error_m': error, 'points': len(values),
                'support_span_m': hc*(last[1]-first[1])+hs*(last[2]-first[2]),
                'support_start_left_m': {'x_m': first[1], 'y_m': first[2]},
                'support_end_left_m': {'x_m': last[1], 'y_m': last[2]}}
    if kind == 'curve':
        candidates = [fitted(-1.4, .5), fitted(-.5, 1.4)]
    elif kind in ('gap', 'kink'):
        candidates = [fitted(-1.4, -.15), fitted(.15, 1.4)]
    else:
        candidates = [fitted(-1.4, 1.4, 0), fitted(-1.4, 1.4, 1)]
    motion = MODULE.TurnMotion(0.)
    motion.outer_wall = wall(theta_deg, -2., -1.4, 1.4)
    return motion, candidates, {'ranges': ranges, 'combined_fit': fitted(-1.4, 1.4)}


class TurnMotionTests(unittest.TestCase):
    def drive(self, heading=60., offset=0., max_drive_s=10., initial_presteer_pwm=None):
        motion = MODULE.TurnMotion(0., max_drive_s=max_drive_s, initial_presteer_pwm=initial_presteer_pwm)
        result = {'servo': 1500}
        outputs = []
        for seq in range(1, 49):
            now = (seq-1)*.1
            result = motion.update(scan(seq, heading, offset), .01, now, control(seq, result['servo']))
            outputs.append(result)
            if result['phase'] == 'drive':
                return motion, seq, now, result, outputs
        self.fail('fresh continuous adopted presteer should reach bounded trial drive')

    def test_neutral_preview_never_ramps_or_requests_positive_motor(self):
        motion = MODULE.TurnMotion(10.)
        result = motion.update(scan(1), .01, 10., control(1, armed=False))
        self.assertTrue(result['start_ready'])
        self.assertEqual((result['motor'], result['servo']), (1500, 1500))
        self.assertFalse(result['physical_steering_confirmed'])

    def test_pwm_gradual_presteer_and_actual_adoption_allowance(self):
        motion, _, now, result, outputs = self.drive()
        self.assertGreaterEqual(now, 1.2)
        self.assertTrue(all(r['motor'] == 1500 for r in outputs[:-1]))
        self.assertEqual(result['motor'], 1560)
        self.assertTrue(all(0 <= b['servo']-a['servo'] <= 10 for a, b in zip(outputs, outputs[1:])))
        self.assertTrue(all(1500 <= r['servo'] <= 1720 for r in outputs))
        self.assertGreaterEqual(result['steering_settle_elapsed_s'], 1.2-1e-9)
        self.assertGreaterEqual(result['steering_settle_feedback_ticks'], 3)
        self.assertFalse(result['physical_steering_confirmed'])
        self.assertFalse(result['completed'])

    def test_initial_1700_override_is_only_presteer_and_drive_uses_live_target(self):
        motion, seq, now, previous, outputs = self.drive(initial_presteer_pwm=1700)
        self.assertTrue(all(r['motor'] == 1500 for r in outputs[:-1]))
        self.assertTrue(all(r['initial_presteer_active'] for r in outputs[:-1]))
        self.assertTrue(all(r['steering_target'] == 1700 for r in outputs))
        self.assertTrue(all(0 <= b['servo']-a['servo'] <= 10 for a, b in zip(outputs, outputs[1:])))
        self.assertEqual((previous['servo'], previous['motor']), (1700, 1560))
        self.assertFalse(previous['initial_presteer_active'])
        self.assertGreaterEqual(previous['steering_settle_feedback_ticks'], 3)
        self.assertGreaterEqual(previous['steering_settle_elapsed_s'], 1.2-1e-9)
        first_drive = motion.drive_since
        result = motion.update(scan(seq+1, heading=55.), .01, now+.1,
                               control(seq+1, 1700, motor=1560))
        self.assertEqual((result['phase'], result['motor'], result['servo']), ('drive', 1560, 1690))
        self.assertEqual(result['steering_target'], result['natural_steering_target'])
        self.assertLess(result['steering_target'], 1700)
        self.assertFalse(result['initial_presteer_active'])
        self.assertEqual(motion.drive_since, first_drive)

    def test_initial_override_cannot_bypass_current_neutral_target_or_deadline(self):
        motion = MODULE.TurnMotion(0., initial_presteer_pwm=1700)
        for seq in range(1, 51):
            result = motion.update(scan(seq, heading=15.1, offset=-.65, width=2.), .01,
                                   (seq-1)*.1, control(seq, armed=seq > 1))
            self.assertEqual((result['motor'], result['servo'], result['steering_target']), (1500, 1500, 1500))
            self.assertFalse(result['start_ready'])
            self.assertFalse(result['initial_presteer_active'])
            self.assertEqual(result['natural_steering_target'], 1500)
            self.assertEqual(result['steering_settle_feedback_ticks'], 0)
        result = motion.update(scan(51, heading=15.1, offset=-.65, width=2.), .01, 5., control(51))
        self.assertEqual((result['phase'], result['motor']), ('locked', 1500))
        self.assertEqual(result['reason'], 'left_turn_presteer_timeout')

    def test_initial_override_needs_its_exact_acknowledged_fresh_adoption(self):
        for fault in ('wrong_servo', 'unacked', 'same_tick'):
            with self.subTest(fault=fault):
                motion, previous = MODULE.TurnMotion(0., initial_presteer_pwm=1700), {'servo': 1500}
                for seq in range(1, 49):
                    feedback = control(seq, 1647 if fault == 'wrong_servo' else previous['servo'],
                                       tick=100 if fault == 'same_tick' else seq*100)
                    if fault == 'unacked': feedback['command_acked'] = False
                    previous = motion.update(scan(seq), .01, (seq-1)*.1, feedback)
                    self.assertEqual(previous['motor'], 1500)
                self.assertLessEqual(previous['steering_settle_feedback_ticks'], 1)

    def test_initial_override_quality_wait_holds_servo_and_restarts_allowance(self):
        motion, previous = MODULE.TurnMotion(0., initial_presteer_pwm=1700), {'servo': 1500}
        for seq in range(1, 49):
            now = (seq-1)*.1
            previous = motion.update(scan(seq), .01, now, control(seq, previous['servo']))
            if previous['steering_settle_feedback_ticks'] == 4:
                break
        waited = motion.update(scan(seq+1), .01, now+.1, control(seq+1, 1700), presteer_wait=True)
        self.assertEqual((waited['motor'], waited['servo'], waited['turn_stage']), (1500, 1700, 'presteer_wait'))
        self.assertFalse(waited['initial_presteer_active'])
        self.assertEqual(waited['steering_settle_elapsed_s'], 0)
        for i in range(2, 14):
            result = motion.update(scan(seq+i), .01, now+i*.1, control(seq+i, 1700))
            self.assertEqual(result['motor'], 1500)
            self.assertTrue(result['initial_presteer_active'])
        result = motion.update(scan(seq+14), .01, now+1.4, control(seq+14, 1700))
        self.assertEqual((result['phase'], result['motor']), ('drive', 1560))
        self.assertGreaterEqual(result['steering_settle_elapsed_s'], 1.2-1e-9)

    def test_initial_override_validator_rejects_noninteger_or_outside_candidates(self):
        for value in (None, 1650, 1700, 1720):
            self.assertEqual(MODULE.validate_initial_presteer_pwm(value), value)
            self.assertEqual(MODULE.TurnMotion(0., initial_presteer_pwm=value).initial_presteer_pwm, value)
        for value in (True, False, 1700., math.nan, math.inf, '1700', 1649, 1721, 1500, 1270):
            with self.subTest(value=value):
                with self.assertRaisesRegex(ValueError, 'invalid_initial_presteer_pwm'):
                    MODULE.validate_initial_presteer_pwm(value)
                with self.assertRaisesRegex(ValueError, 'invalid_initial_presteer_pwm'):
                    MODULE.TurnMotion(0., initial_presteer_pwm=value)

    def test_initial_override_cannot_restart_motor_after_coast_or_safety_lock(self):
        for stop in ('coast', 'safety'):
            with self.subTest(stop=stop):
                motion, seq, now, previous, _ = self.drive(initial_presteer_pwm=1700)
                if stop == 'coast': motion.begin_coast('planned_stop', now)
                for i in range(1, 25):
                    previous = motion.update(scan(seq+i), .01, now+i*.1,
                                             control(seq+i, previous['servo'], motor=1500), safe=stop == 'coast')
                    self.assertEqual(previous['motor'], 1500)
                    self.assertFalse(previous['initial_presteer_active'])
                self.assertEqual(previous['servo'], 1500)
                self.assertEqual(previous['phase'], 'coast' if stop == 'coast' else 'locked')

    def test_motor_neutral_ack_with_wrong_servo_never_starts_drive(self):
        motion = MODULE.TurnMotion(0.)
        for seq in range(1, 49):
            result = motion.update(scan(seq), .01, (seq-1)*.1, control(seq))
            self.assertEqual(result['motor'], 1500)
        self.assertEqual(result['steering_settle_elapsed_s'], 0)

    def test_same_adoption_tick_cannot_mature_settle(self):
        motion = MODULE.TurnMotion(0.)
        result = {'servo': 1500}
        for seq in range(1, 48):
            result = motion.update(scan(seq), .01, (seq-1)*.1,
                                   control(1, result['servo'], tick=100))
            self.assertEqual(result['motor'], 1500)
        self.assertLessEqual(result['steering_settle_feedback_ticks'], 1)

    def test_repeated_frames_do_not_ramp_or_refresh_observation(self):
        motion = MODULE.TurnMotion(0.)
        value = scan(1)
        first = motion.update(value, .01, 0., control(1))
        repeated = motion.update(value, .01, .2, control(2, first['servo']))
        self.assertEqual(repeated['servo'], first['servo'])
        self.assertEqual(repeated['steering_settle_elapsed_s'], 0)
        stale = motion.update(value, .01, .3, control(3, first['servo']))
        self.assertEqual(stale['terminal_reason'], 'turn_scan_not_advancing')
        self.assertEqual(stale['motor'], 1500)

    def test_scan_content_at_same_identity_cannot_change(self):
        motion = MODULE.TurnMotion(0.)
        motion.update(scan(1), .01, 0., control(1))
        result = motion.update(scan(1, heading=61.), .01, .1, control(2))
        self.assertEqual(result['terminal_reason'], 'duplicate_turn_scan_changed_content')

    def test_actual_ranges_at_same_scan_identity_cannot_change(self):
        motion = MODULE.TurnMotion(0.)
        value = dict(scan(1), ranges=[None]*360)
        motion.update(value, .01, 0., control(1))
        value['ranges'][270] = 1.1
        result = motion.update(value, .01, .1, control(2))
        self.assertEqual(result['terminal_reason'], 'duplicate_turn_scan_changed_content')
        self.assertEqual((result['motor'], result['servo']), (1500, 1500))

    def test_unknown_initial_left_target_refuses_start(self):
        for value in [dict(scan(1), corridor_candidates=[]), scan(1, heading=0.)]:
            with self.subTest(value=value):
                result = MODULE.TurnMotion(0.).update(value, .01, 0., control(1, armed=False))
                self.assertFalse(result['start_ready'])
                self.assertEqual(result['motor'], 1500)
                self.assertEqual(result['terminal_reason'], 'left_corridor_unknown')

    def test_ambiguous_initial_or_tracked_candidates_never_drive(self):
        for started in [False, True]:
            with self.subTest(started=started):
                motion = MODULE.TurnMotion(0.)
                if started:
                    motion.update(scan(1), .01, 0., control(1))
                value = scan(2 if started else 1)
                value['corridor_candidates'].append(corridor(62.))
                result = motion.update(value, .01, .1 if started else 0., control(2))
                self.assertEqual(result['terminal_reason'], 'left_corridor_ambiguous')
                self.assertEqual(result['motor'], 1500)

    def test_no_fixed_90_degree_target_and_unobserved_origin_rejected(self):
        for heading in [20., 45., 115.]:
            result = MODULE.TurnMotion(0.).update(scan(1, heading), .01, 0., control(1, armed=False))
            self.assertTrue(result['start_ready'])
            self.assertAlmostEqual(result['corridor']['heading_left_rad'], math.radians(heading))
        value = scan(1)
        value['corridor_candidates'][0]['origin_between_walls'] = False
        result = MODULE.TurnMotion(0.).update(value, .01, 0., control(1))
        self.assertEqual(result['terminal_reason'], 'invalid_native_corridor_candidates')

    def test_narrow_turn_corridor_uses_body_clearance_and_range_allowance(self):
        self.assertAlmostEqual(MODULE.TURN_LATERAL_MARGIN_M, .25)
        available = .6/2-MODULE.TURN_LATERAL_MARGIN_M
        for sign in (-1, 1):
            for offset, accepted in [(available-1e-6, True), (available, False),
                                     (available+1e-6, False)]:
                with self.subTest(sign=sign, offset=offset):
                    result = MODULE.TurnMotion(0.).update(
                        scan(1, heading=20., offset=sign*offset, width=.6), .01, 0.,
                        control(1, armed=False))
                    self.assertEqual(result['start_ready'], accepted)
                    self.assertEqual(result['lock_requested'], not accepted)
                    self.assertEqual((result['motor'], result['servo']), (1500, 1500))
                    if accepted:
                        self.assertGreater(result['steering_target'], 1500)
                    else:
                        self.assertEqual(result['reason'], 'invalid_native_corridor_candidates')

    def test_narrow_turn_corridor_presteers_then_confirms_alignment_at_new_margin(self):
        motion, previous, outputs = MODULE.TurnMotion(0.), {'servo': 1500}, []
        for seq in range(1, 49):
            now = (seq-1)*.1
            previous = motion.update(scan(seq, heading=20., offset=.04, width=.6), .01,
                                     now, control(seq, previous['servo']))
            outputs.append(previous)
            if previous['phase'] == 'drive':
                break
        self.assertEqual(previous['phase'], 'drive')
        self.assertTrue(all(r['motor'] == 1500 for r in outputs[:-1]))
        self.assertGreater(previous['servo'], 1500)
        self.assertGreaterEqual(previous['steering_settle_feedback_ticks'], 3)
        self.assertGreaterEqual(previous['steering_settle_elapsed_s'], 1.2-1e-9)
        for i, heading in enumerate((12., 4., 3., 2., 1.), start=1):
            previous = motion.update(scan(seq+i, heading=heading, offset=.04, width=.6), .01,
                                     now+i*.1, control(seq+i, previous['servo'], motor=1560))
        self.assertTrue(previous['observed_alignment'])
        self.assertEqual((previous['phase'], previous['motor']), ('coast', 1500))
        self.assertFalse(previous['completed'])

    def test_narrow_turn_coast_correction_obeys_the_same_lateral_boundary(self):
        motion = MODULE.TurnMotion(0.)
        available = .6/2-MODULE.TURN_LATERAL_MARGIN_M
        for sign in (-1, 1):
            with self.subTest(sign=sign):
                inner = motion._coast_target(corridor(sign*5., sign*(available-1e-6), .6))
                self.assertGreater(sign*(inner-1500), 0)
                self.assertTrue(1445 <= inner <= 1555)
                for offset in (available, available+1e-6):
                    self.assertEqual(motion._coast_target(corridor(sign*5., sign*offset, .6)), 1500)

    def test_stale_reordered_jump_and_unknown_lock_immediately(self):
        for fault in ['stale', 'reordered', 'jump', 'publication']:
            with self.subTest(fault=fault):
                motion, seq, now, old, _ = self.drive()
                value = scan(seq+1)
                age = .3 if fault == 'stale' else .01
                if fault == 'reordered': value = scan(seq-1)
                if fault == 'jump': value = scan(seq+1, heading=120.)
                if fault == 'publication': value['at_ms'] = seq*100
                result = motion.update(value, age, now+.1, control(seq+1, old['servo'], motor=1560))
                self.assertTrue(result['lock_requested'])
                self.assertEqual((result['motor'], result['servo']), (1500, 1500))

    def test_geometry_empty_grace_keeps_old_target_without_renewing_evidence(self):
        motion, seq, now, previous, _ = self.drive()
        source_seq, source_at = previous['geometry_source_seq'], previous['geometry_source_at_ms']
        target = previous['steering_target']
        for i in range(1, 4):
            result = motion.update(dict(scan(seq+i), corridor_candidates=[]), .01, now+i*.1,
                                   control(seq+i, previous['servo'], motor=1560))
            self.assertFalse(result['lock_requested'])
            self.assertEqual(result['steering_target'], target)
            self.assertEqual(result['geometry_source_seq'], source_seq)
            self.assertEqual(result['geometry_source_at_ms'], source_at)
            self.assertEqual(result['alignment_confirmations'], 0)
            self.assertTrue(result['geometry_missing'])
            previous = result
        result = motion.update(dict(scan(seq+4), corridor_candidates=[]), .01, now+.4,
                               control(seq+4, previous['servo'], motor=1560))
        self.assertEqual(result['terminal_reason'], 'left_turn_geometry_lost')
        self.assertEqual(result['motor'], 1500)

    def test_unacknowledged_servo_command_cannot_start_settle_or_drive(self):
        motion = MODULE.TurnMotion(0.)
        previous = {'servo': 1500}
        for seq in range(1, 48):
            feedback = dict(control(seq, previous['servo']), command_acked=False)
            previous = motion.update(scan(seq), .01, (seq-1)*.1, feedback)
            self.assertEqual(previous['motor'], 1500)
            self.assertEqual(previous['steering_settle_feedback_ticks'], 0)

    def test_actual_ray_goal_allows_origin_before_exit_without_fixed_right_angle(self):
        motion = MODULE.TurnMotion(0.)
        result = motion.update(opening_scan(1), .01, 0., control(1, armed=False))
        self.assertTrue(result['start_ready'])
        self.assertEqual(result['geometry_mode'], 'opening_wall')
        self.assertFalse(result['corridor']['origin_between_walls'])
        self.assertAlmostEqual(result['corridor']['heading_left_rad'], math.radians(60.))
        self.assertEqual((result['motor'], result['servo']), (1500, 1500))

    def test_recorded_third_and_fourth_trial_starts_preview_dynamic_left_presteer(self):
        for seq in (4129, 886):
            with self.subTest(seq=seq):
                value = recorded_opening_scan(seq)
                result = MODULE.TurnMotion(0.).update(value, .01, 0., control(1, armed=False))
                self.assertTrue(result['start_ready'])
                self.assertEqual(result['turn_stage'], 'presteer')
                self.assertGreater(result['steering_target'], 1500)
                self.assertLess(result['steering_target'], 1720)
                self.assertEqual((result['motor'], result['servo']), (1500, 1500))
                self.assertTrue(result['incoming_endpoint_current'])
                self.assertEqual(result['incoming_endpoint']['source_seq'], seq)
                self.assertFalse(result['turn_path_certified'])
                if seq == 886:
                    self.assertEqual(result['steering_target'], 1681)

    def test_recorded_fourth_start_cannot_drive_before_left_adoption_and_allowance(self):
        motion, previous, outputs = MODULE.TurnMotion(0.), {'servo': 1500}, []
        # Synthetic fresh observations of the captured start geometry exercise
        # command adoption, without treating them as a recorded motion trace.
        for i in range(49):
            value = recorded_opening_scan(886)
            value.update(seq=886+i, at_ms=88530+i*100)
            feedback = control(i+1, previous['servo'])
            previous = motion.update(value, .01, i*.1, feedback)
            outputs.append(previous)
            if previous['phase'] == 'drive':
                self.assertEqual(feedback['servo'], previous['steering_target'])
                self.assertEqual(previous['servo'], 1681)
                self.assertGreaterEqual(previous['steering_settle_feedback_ticks'], 3)
                self.assertGreaterEqual(previous['steering_settle_elapsed_s'], 1.2-1e-9)
                break
            self.assertEqual(previous['motor'], 1500)
            self.assertEqual(previous['turn_stage'], 'presteer')
        self.assertEqual(previous['phase'], 'drive')
        self.assertEqual(previous['turn_stage'], 'drive')
        self.assertTrue(all(abs(b['servo']-a['servo']) <= 10 for a, b in zip(outputs, outputs[1:])))

    def test_recorded_fourth_start_wrong_actual_servo_keeps_motor_neutral(self):
        motion = MODULE.TurnMotion(0.)
        for i in range(49):
            value = recorded_opening_scan(886)
            value.update(seq=886+i, at_ms=88530+i*100)
            result = motion.update(value, .01, i*.1, control(i+1, 1500))
            self.assertEqual(result['phase'], 'presteer')
            self.assertEqual(result['motor'], 1500)
            self.assertEqual(result['steering_settle_feedback_ticks'], 0)
        self.assertEqual(result['servo'], 1681)

    def test_current_target_change_restarts_presteer_adoption(self):
        motion, previous = MODULE.TurnMotion(0.), {'servo': 1500}
        for seq in range(1, 49):
            now = (seq-1)*.1
            previous = motion.update(scan(seq), .01, now, control(seq, previous['servo']))
            if previous['steering_settle_feedback_ticks'] == 4:
                break
        old_target = previous['steering_target']
        result = motion.update(scan(seq+1, heading=70.), .01, now+.1,
                               control(seq+1, old_target))
        self.assertGreater(result['steering_target'], old_target+10)
        self.assertEqual((result['phase'], result['motor']), ('presteer', 1500))
        self.assertEqual(result['steering_settle_feedback_ticks'], 0)
        self.assertEqual(result['steering_settle_elapsed_s'], 0)
        for i in range(2, 8):
            result = motion.update(scan(seq+i, heading=70.), .01, now+i*.1,
                                   control(seq+i, old_target))
            self.assertEqual(result['motor'], 1500)
            self.assertEqual(result['steering_settle_feedback_ticks'], 0)

    def test_target_releasing_to_neutral_during_presteer_cannot_start_straight(self):
        motion, previous = MODULE.TurnMotion(0.), {'servo': 1500}
        for seq in range(1, 49):
            heading = 20. if seq == 1 else 10. if seq == 2 else 0.
            previous = motion.update(scan(seq, heading=heading), .01, (seq-1)*.1,
                                     control(seq, previous['servo']))
            self.assertEqual((previous['phase'], previous['motor']), ('presteer', 1500))
        self.assertEqual(previous['steering_target'], 1500)
        self.assertFalse(previous['start_ready'])
        self.assertEqual(previous['steering_settle_feedback_ticks'], 0)

    def test_legal_corridor_with_neutral_initial_target_is_a_bounded_hold(self):
        motion = MODULE.TurnMotion(0.)
        for seq in range(1, 51):
            result = motion.update(scan(seq, heading=15.1, offset=-.65, width=2.), .01,
                                   (seq-1)*.1, control(seq, armed=seq > 1))
            self.assertEqual((result['motor'], result['servo']), (1500, 1500))
            self.assertFalse(result['start_ready'])
            self.assertEqual(result['steering_settle_feedback_ticks'], 0)
        self.assertEqual(result['phase'], 'presteer')
        result = motion.update(scan(51, heading=15.1, offset=-.65, width=2.), .01,
                               5., control(51))
        self.assertEqual(result['reason'], 'left_turn_presteer_timeout')
        self.assertEqual((result['phase'], result['motor']), ('locked', 1500))

    def test_endpoint_diagnostic_distinguishes_current_ray_from_cached_goal(self):
        motion = MODULE.TurnMotion(0.)
        current = motion.update(opening_scan(1, 280), .01, 0., control(1, armed=False))
        self.assertTrue(current['incoming_endpoint_current'])
        tracked = dict(opening_scan(2, 280), left_turn_goal=None)
        result = motion.update(tracked, .01, .1, control(2, armed=False))
        self.assertFalse(result['incoming_endpoint_current'])
        self.assertIsNone(result['incoming_endpoint'])
        self.assertEqual(result['turn_goal']['incoming_left_end_m'],
                         current['incoming_endpoint']['incoming_left_end_m'])
        self.assertGreater(result['steering_target'], 1500)
        missing = dict(opening_scan(3, 280), left_turn_goal=None, wall_candidates=[])
        result = motion.update(missing, .01, .2, control(3, armed=False))
        self.assertFalse(result['incoming_endpoint_current'])
        self.assertIsNone(result['incoming_endpoint'])
        self.assertEqual(result['geometry_source_seq'], 2)

    def test_presteer_wait_holds_adopted_servo_and_clears_settle_without_extending_deadline(self):
        motion = MODULE.TurnMotion(0.)
        previous = {'servo': 1500}
        for seq in range(1, 12):
            previous = motion.update(opening_scan(seq), .01, (seq-1)*.1, control(seq, previous['servo']))
        held = previous['servo']
        self.assertGreater(held, 1500)
        result = motion.update(opening_scan(12), .01, 1.1, control(12, held), presteer_wait=True)
        self.assertEqual((result['phase'], result['motor'], result['servo']), ('presteer', 1500, held))
        self.assertEqual(result['turn_stage'], 'presteer_wait')
        self.assertFalse(result['start_ready'])
        self.assertEqual(result['steering_settle_feedback_ticks'], 0)
        self.assertEqual(result['geometry_source_seq'], 12)
        missing = dict(opening_scan(13), left_turn_goal=None, wall_candidates=[], corridor_candidates=[])
        result = motion.update(missing, .01, 1.2, control(13, held), presteer_wait=True)
        self.assertEqual(result['servo'], held)
        self.assertEqual(result['geometry_source_seq'], 12)
        self.assertEqual(result['turn_stage'], 'presteer_wait')
        for seq in range(14, 52):
            result = motion.update(opening_scan(seq), .01, (seq-1)*.1, control(seq, held), presteer_wait=True)
        self.assertEqual(result['reason'], 'left_turn_presteer_timeout')
        self.assertEqual((result['motor'], result['servo']), (1500, 1500))

    def test_presteer_quality_wait_requires_a_new_full_adoption_allowance(self):
        motion, previous = MODULE.TurnMotion(0.), {'servo': 1500}
        for seq in range(1, 49):
            now = (seq-1)*.1
            previous = motion.update(opening_scan(seq), .01, now, control(seq, previous['servo']))
            if previous['steering_settle_feedback_ticks'] == 4:
                break
        held = previous['servo']
        waited = motion.update(opening_scan(seq+1), .01, now+.1, control(seq+1, held), presteer_wait=True)
        self.assertEqual((waited['motor'], waited['servo'], waited['turn_stage']), (1500, held, 'presteer_wait'))
        self.assertEqual(waited['steering_settle_elapsed_s'], 0)
        for i in range(2, 14):
            result = motion.update(opening_scan(seq+i), .01, now+i*.1, control(seq+i, held))
            self.assertEqual((result['motor'], result['turn_stage']), (1500, 'presteer'))
        result = motion.update(opening_scan(seq+14), .01, now+1.4, control(seq+14, held))
        self.assertEqual((result['phase'], result['motor']), ('drive', 1560))
        self.assertGreaterEqual(result['steering_settle_elapsed_s'], 1.2-1e-9)

    def test_presteer_wait_does_not_mask_invalid_current_feedback_or_geometry(self):
        for fault in ['motor', 'stale', 'ambiguous']:
            with self.subTest(fault=fault):
                motion = MODULE.TurnMotion(0.)
                motion.update(opening_scan(1), .01, 0., control(1))
                value, age, feedback = opening_scan(2), .01, control(2)
                if fault == 'motor': feedback['motor'] = 1560
                if fault == 'stale': age = .3
                if fault == 'ambiguous':
                    value['wall_candidates'].append(wall(60., -1.68))
                result = motion.update(value, age, .1, feedback, presteer_wait=True)
                self.assertTrue(result['lock_requested'])
                self.assertEqual((result['motor'], result['servo']), (1500, 1500))

    def test_endpoint_contract_rejects_missing_unknown_mismatched_and_right_side_evidence(self):
        for fault in ['missing', 'unknown', 'range', 'angle', 'point', 'scalar', 'heading', 'fraction',
                      'right_side', 'native_window']:
            with self.subTest(fault=fault):
                value = opening_scan(1)
                goal = value['left_turn_goal']
                endpoint = goal['incoming_left_end_support']
                if fault == 'missing': del goal['incoming_left_end_support']
                if fault == 'unknown': value['ranges'][endpoint['index']] = None
                if fault == 'range': endpoint['range_m'] += .05
                if fault == 'angle': endpoint['angle_left_rad'] += .02
                if fault == 'point': endpoint['point_left_m']['x_m'] += .05
                if fault == 'scalar': goal['incoming_left_end_m'] += .05
                if fault == 'heading': goal['incoming_heading_left_rad'] = 4.
                if fault == 'fraction': goal['known_open_fraction'] = .89
                if fault in ('right_side', 'native_window'):
                    angle = math.radians(-105) if fault == 'right_side' else math.radians(135)
                    index = 105 if fault == 'right_side' else 225
                    distance = .5/abs(math.sin(angle))
                    point = {'x_m': distance*math.cos(angle), 'y_m': distance*math.sin(angle)}
                    value['ranges'][index] = distance
                    goal['incoming_left_end_m'] = point['x_m']
                    goal['incoming_left_end_support'] = {'index': index, 'angle_left_rad': angle,
                                                         'range_m': distance, 'point_left_m': point}
                result = MODULE.TurnMotion(0.).update(value, .01, 0., control(1, armed=False))
                self.assertEqual(result['reason'], 'invalid_native_turn_wall_goal')
                self.assertEqual((result['motor'], result['servo']), (1500, 1500))

    def test_outer_wall_reprojects_target_and_handover_requires_real_double_walls(self):
        motion = MODULE.TurnMotion(0.)
        previous = {'servo': 1500}
        for seq in range(1, 49):
            now = (seq-1)*.1
            previous = motion.update(opening_scan(seq), .01, now, control(seq, previous['servo']))
            if previous['phase'] == 'drive': break
        self.assertEqual(previous['phase'], 'drive')
        old_point = previous['turn_goal']['target_point_left_m']
        initial_target, first_drive = previous['steering_target'], motion.drive_since
        targets = []
        for i in range(1, 13):
            heading, rho = 60.-i*5, -1.8+i*.1
            value = dict(scan(seq+i), corridor_candidates=[], wall_candidates=[wall(heading, rho)], left_turn_goal=None)
            if i == 12:
                value['corridor_candidates'] = [corridor(0., 0., 1.2)]
            result = motion.update(value, .01, now+i*.1, control(seq+i, previous['servo'], motor=1560))
            self.assertLessEqual(abs(result['servo']-previous['servo']), 10)
            previous = result
            self.assertFalse(previous['lock_requested'])
            self.assertEqual((previous['motor'], previous['turn_stage']), (1560, 'drive'))
            self.assertEqual(motion.drive_since, first_drive)
            self.assertFalse(previous['incoming_endpoint_current'])
            targets.append(previous['steering_target'])
            if i < 12:
                self.assertEqual(previous['geometry_mode'], 'opening_wall')
                self.assertFalse(previous['observed_alignment'])
        self.assertLess(targets[0], initial_target)
        self.assertEqual(targets[-4:], [1500]*4)
        self.assertNotEqual(previous['turn_goal']['target_point_left_m'], old_point)
        self.assertEqual(previous['geometry_mode'], 'corridor')
        for i in range(13, 16):
            previous = motion.update(scan(seq+i, heading=0.), .01, now+i*.1,
                                     control(seq+i, previous['servo'], motor=1560))
        self.assertTrue(previous['observed_alignment'])
        self.assertTrue(previous['test_sequence_finished'])
        self.assertEqual((previous['motor'], previous['servo'], previous['turn_stage']), (1500, 1500, 'coast'))
        self.assertFalse(previous['completed'])

    def test_overlapping_wall_fits_are_one_identity_but_different_parallel_walls_are_not(self):
        for ambiguous in [False, True]:
            with self.subTest(ambiguous=ambiguous):
                motion = MODULE.TurnMotion(0.)
                motion.update(opening_scan(1), .01, 0., control(1))
                second = dict(opening_scan(2), left_turn_goal=None)
                second['wall_candidates'].append(wall(60.1, -1.7 if ambiguous else -1.799, 1.2, 2.6))
                result = motion.update(second, .01, .1, control(2))
                self.assertEqual(result['lock_requested'], ambiguous)

    def test_recorded_nonoverlapping_near_wall_does_not_make_selected_outer_support_ambiguous(self):
        motion = MODULE.TurnMotion(0.)
        previous = motion.update(recorded_opening_scan(5411), .01, 0., control(1, armed=False))
        self.assertTrue(previous['start_ready'])
        current = recorded_opening_scan(5412)
        result = motion.update(current, .01, .099, control(2))
        self.assertEqual(result['phase'], 'presteer')
        self.assertEqual(result['reason'], 'left_turn_presteering')
        self.assertEqual((result['motor'], result['servo']), (1500, 1510))
        self.assertGreater(result['steering_target'], 1500)
        self.assertTrue(result['incoming_endpoint_current'])
        self.assertEqual(result['geometry_source_seq'], 5412)
        self.assertEqual(result['outer_wall'], current['left_turn_goal']['outer_wall'])
        self.assertEqual(result['steering_settle_feedback_ticks'], 0)
        self.assertFalse(result['completed'])

    def test_recorded_before_wall_association_is_unique_but_skipped_frames_do_not_refresh_clock(self):
        motion = MODULE.TurnMotion(0.)
        motion.update(recorded_opening_scan(5403), .01, 0., control(1, armed=False))
        current = recorded_opening_scan(5412)
        matches = motion._wall_matches(current['wall_candidates'], .899)
        self.assertEqual(matches, [current['left_turn_goal']['outer_wall']])
        result = motion.update(current, .01, .899, control(2))
        self.assertEqual(result['reason'], 'turn_scan_clock_gap')
        self.assertEqual((result['motor'], result['servo']), (1500, 1500))
        self.assertEqual(result['geometry_source_seq'], 5403)

    def test_finite_support_association_works_for_rotated_walls_and_reversed_endpoint_order(self):
        for heading in [-170., -45., 20., 75., 145.]:
            with self.subTest(heading=heading):
                motion = MODULE.TurnMotion(0.)
                motion.outer_wall = wall(heading, -1.8, 1., 3.)
                selected = wall(heading+1., -1.79, 1.05, 3.05)
                selected['support_start_left_m'], selected['support_end_left_m'] = (
                    selected['support_end_left_m'], selected['support_start_left_m'])
                other_segment = wall(heading-2.5, -1.78, -.8, .6)
                self.assertEqual(motion._wall_matches([other_segment, selected], .1), [selected])

    def test_nonoverlapping_wall_cannot_replace_selected_finite_support(self):
        motion = MODULE.TurnMotion(0.)
        motion.update(opening_scan(1), .01, 0., control(1, armed=False))
        current = dict(opening_scan(2), left_turn_goal=None, wall_candidates=[wall(60., -1.8, 4., 6.)])
        result = motion.update(current, .01, .1, control(2))
        self.assertEqual(result['reason'], 'left_turn_outer_wall_identity_jump')
        self.assertEqual((result['motor'], result['servo']), (1500, 1500))
        self.assertEqual(result['geometry_source_seq'], 1)

    def test_smooth_deformed_surface_merges_different_fitted_angles_with_real_returns(self):
        for heading in [-145., -30., 0., 90., 170.]:
            for noise in [0., .02]:
                with self.subTest(heading=heading, noise=noise):
                    motion, walls, actual = surface_scan(heading, noise=noise)
                    self.assertTrue(all(MODULE._valid_wall(w) for w in walls))
                    self.assertGreater(abs(walls[0]['heading_left_rad']-walls[1]['heading_left_rad']),
                                       math.radians(2))
                    # Reverse endpoint order without changing the actual surface.
                    walls[1]['support_start_left_m'], walls[1]['support_end_left_m'] = (
                        walls[1]['support_end_left_m'], walls[1]['support_start_left_m'])
                    self.assertEqual(len(motion._wall_matches(walls, .1, actual)), 1)
                    self.assertEqual(len(motion._wall_matches(walls, .1)), 2)

    def test_recorded_quantized_board_support_and_measured_window_bridge_are_smooth(self):
        actual = recorded_opening_scan(5412, actual_board=True)
        outer, near = actual['wall_candidates']
        self.assertGreater(abs(outer['heading_left_rad']-near['heading_left_rad']), math.radians(2))
        # Although the fitted finite windows stop on either side of the gap,
        # actual returns in that interval witness one gently bending board.
        supports = MODULE._surface_supports(actual, [outer, near])
        self.assertIs(MODULE._same_surface(*supports), True)
        alias = dict(outer, heading_left_rad=outer['heading_left_rad']+.0017,
                     rho_left_m=outer['rho_left_m']-.001)
        motion = MODULE.TurnMotion(0.)
        motion.outer_wall = outer
        self.assertEqual(len(motion._wall_matches([outer, alias], .1, actual)), 1)

    def test_recorded_1491_missing_edge_bins_keep_real_mature_same_surface_support(self):
        fixture = json.loads((Path(__file__).parent/'fixtures'/'left-turn-1491-fragmented-board.json').read_text())
        self.assertEqual(fixture['recorded_stage'], 'drive')
        actual, walls = fixture['scan'], fixture['scan']['wall_candidates']
        self.assertEqual((actual['seq'], actual['at_ms']), (1491, 148921))
        self.assertEqual([i for i, value in enumerate(actual['ranges']) if value is None], [8, 9])
        self.assertGreater(abs(walls[0]['heading_left_rad']-walls[1]['heading_left_rad']), math.radians(2))
        self.assertGreater(abs(walls[0]['rho_left_m']-walls[1]['rho_left_m']), .05)
        supports = MODULE._surface_supports(actual, walls)
        self.assertTrue(all(support and support[0] for support in supports))
        self.assertTrue(all(min(group[0].bit_count() for group in support[0].values()) >= 16
                            for support in supports))
        self.assertIs(MODULE._same_surface(*supports), True)
        motion = MODULE.TurnMotion(0.)
        motion.outer_wall = fixture['previous_selected_outer_wall']
        selected = motion._wall_matches(walls, fixture['publication_dt_s'], actual)
        self.assertEqual(len(selected), 1)
        self.assertEqual(motion._wall_matches(list(reversed(walls)), fixture['publication_dt_s'], actual), selected)
        self.assertEqual(len(motion._wall_matches(walls, fixture['publication_dt_s'])), 2)

    def test_recorded_1491_recovery_cannot_bridge_a_new_unknown_or_out_of_line_return(self):
        fixture = json.loads((Path(__file__).parent/'fixtures'/'left-turn-1491-fragmented-board.json').read_text())
        motion = MODULE.TurnMotion(0.)
        motion.outer_wall = fixture['previous_selected_outer_wall']
        for fault in ('unknown', 'out_of_line'):
            with self.subTest(fault=fault):
                actual = dict(fixture['scan'], ranges=list(fixture['scan']['ranges']))
                actual['ranges'][20] = None if fault == 'unknown' else actual['ranges'][20]+.08
                walls = actual['wall_candidates']
                self.assertIsNot(MODULE._same_surface(*MODULE._surface_supports(actual, walls)), True)
                self.assertEqual(len(motion._wall_matches(walls, fixture['publication_dt_s'], actual)), 2)

    def test_basic_run_edges_do_not_reduce_six_point_or_sixteen_support_requirements(self):
        measured = wall(90., -2., -.8, .8)
        for first in (0, 352):
            for length in (5, 6, 11, 12, 15, 16):
                with self.subTest(first=first, length=length):
                    ranges = [None]*360
                    for offset in range(length):
                        index = (first+offset) % 360
                        ranges[index] = 2/math.cos(math.radians(index))
                    supports = MODULE._surface_supports({'ranges': ranges}, [measured, measured])
                    if length < 16:
                        self.assertEqual(supports, [None, None])
                    else:
                        self.assertEqual([group[0].bit_count() for group in supports[0][0].values()], [16])
                        self.assertIs(MODULE._same_surface(*supports), True)

    def test_complete_circular_basic_run_keeps_wrap_windows_real(self):
        measured = dict(wall(0., 3.2, -.6, .6), fit_error_m=.06)
        supports = MODULE._surface_supports({'ranges': [3.2]*360}, [measured, measured])
        self.assertIs(MODULE._same_surface(*supports), True)
        self.assertGreaterEqual(sum(group[0].bit_count() for group in supports[0][0].values()), 16)

    def test_short_kink_arm_at_measured_run_edge_remains_a_separate_candidate(self):
        for short_count in range(1, 7):
            with self.subTest(short_count=short_count):
                motion, walls, actual = surface_scan(90., 'kink')
                negative = sorted((i for i, r in enumerate(actual['ranges'])
                                   if r is not None and -r*math.sin(math.radians(i)) < 0),
                                  key=lambda i: abs(actual['ranges'][i]*math.sin(math.radians(i))))
                keep = set(negative[:short_count])
                actual['ranges'] = [r if r is None or -r*math.sin(math.radians(i)) >= 0 or i in keep else None
                                    for i, r in enumerate(actual['ranges'])]
                self.assertEqual(len(motion._wall_matches(walls, .2, actual)), 2)

    def test_same_surface_still_needs_four_shared_returns_and_full_connecting_interval(self):
        left = (1 << 16)-1
        for shift, expected in ((12, True), (13, False)):
            right = ((1 << 16)-1) << shift
            a = ({0: (left, 0, 15)}, left)
            b = ({0: (right, shift, shift+15)}, right)
            self.assertIs(MODULE._same_surface(a, b), expected)
        # Two real mature supports with four shared returns can still have an
        # unexplained intervening return, which must not become a wall witness.
        left = ((1 << 18)-1) & ~(1 << 5) & ~(1 << 9)
        right = ((1 << 16)-1) << 14
        self.assertIs(MODULE._same_surface(({0: (left, 0, 17)}, left),
                                          ({0: (right, 14, 29)}, right)), False)

    def test_identical_angle_separate_parallel_returns_stay_ambiguous(self):
        for heading in [-30., 90., 170.]:
            for separation in [.03, .15]:
                with self.subTest(heading=heading, separation=separation):
                    motion, walls, actual = surface_scan(heading, 'parallel', separation=separation)
                    self.assertTrue(all(MODULE._valid_wall(w) for w in walls))
                    self.assertAlmostEqual(walls[0]['heading_left_rad'], walls[1]['heading_left_rad'])
                    self.assertEqual(len(motion._wall_matches(walls, .1, actual)), 2)

    def test_broad_fit_cannot_transitively_merge_separate_parallel_surfaces(self):
        motion, walls, actual = surface_scan(kind='parallel')
        broad = actual['combined_fit']
        self.assertTrue(MODULE._valid_wall(broad))
        self.assertGreaterEqual(broad['support_span_m'], max(w['support_span_m'] for w in walls))
        # The combined fit individually agrees with either set of returns, but
        # their precise fits witness two genuinely separate parallel surfaces.
        self.assertEqual(len(motion._wall_matches([broad]+walls, .1, actual)), 2)

    def test_maximum_duplicate_candidates_preserve_alias_and_parallel_results(self):
        for kind, expected in [('curve', 1), ('parallel', 2)]:
            with self.subTest(kind=kind):
                motion, walls, actual = surface_scan(kind=kind)
                repeated = [dict(walls[i % 2]) for i in range(64)]
                self.assertEqual(motion._wall_matches(repeated, .1, actual),
                                 motion._wall_matches(walls, .1, actual))
                self.assertEqual(len(motion._wall_matches(repeated, .1, actual)), expected)

    def test_actual_kink_and_unknown_gap_cannot_be_merged_as_smooth_board(self):
        for heading in [-30., 90., 170.]:
            for kind in ['kink', 'gap']:
                with self.subTest(heading=heading, kind=kind):
                    motion, walls, actual = surface_scan(heading, kind)
                    self.assertTrue(all(MODULE._valid_wall(w) for w in walls))
                    self.assertEqual(len(motion._wall_matches(walls, .2, actual)), 2)

    def test_missing_malformed_or_short_raw_support_cannot_relax_alias_rule(self):
        for fault in ['missing', 'short', 'malformed', 'sparse']:
            with self.subTest(fault=fault):
                motion, walls, actual = surface_scan()
                if fault == 'missing': actual = {}
                if fault == 'short': actual['ranges'] = actual['ranges'][:-1]
                if fault == 'malformed': actual['ranges'][0] = True
                if fault == 'sparse':
                    keep = [i for i, r in enumerate(actual['ranges']) if r is not None][:15]
                    actual['ranges'] = [r if i in keep else None for i, r in enumerate(actual['ranges'])]
                self.assertEqual(len(motion._wall_matches(walls, .1, actual)), 2)

    def test_turn_goal_without_real_support_ray_or_inconsistent_reference_refuses(self):
        for fault in ['ray', 'mirror', 'line', 'flag']:
            with self.subTest(fault=fault):
                value = opening_scan(1)
                if fault == 'ray': value['ranges'][330] = None
                if fault == 'mirror': value['left_turn_goal']['target_point_left_m']['y_m'] *= -1
                if fault == 'line': value['left_turn_goal']['center_offset_left_m'] += .3
                if fault == 'flag': value['left_turn_goal']['turn_path_certified'] = True
                result = MODULE.TurnMotion(0.).update(value, .01, 0., control(1, armed=False))
                self.assertFalse(result['start_ready'])
                self.assertTrue(result['lock_requested'])

    def test_target_only_manual_or_simulated_source_is_accepted_without_fake_pose(self):
        for source in ['manual', 'simulated']:
            record = {'schema_version': 1, 'goal_id': 'trial-1', 'source_kind': source, 'goal_type': 'point_stop',
                      'frame': 'run_start_lidar_reference', 'coordinate_convention': 'x_forward_y_left_yaw_left_positive',
                      'max_seconds': 10., 'target_point_left_m': {'x_m': 2., 'y_m': 1.}}
            result = MODULE.parse_trial_goal(record)
            self.assertTrue(result['target_only'])
            self.assertEqual(result['execution_rejection'], 'real_pose_missing')
            self.assertFalse(result['physical_control_ready'])
            for field in ['sim_pose', 'vehicle_pose', 'source_pose', 'speed_mps']:
                with self.assertRaises(ValueError): MODULE.parse_trial_goal(dict(record, **{field: 0}))
        record['goal_type'] = 'turn_exit_align'
        self.assertEqual(MODULE.parse_trial_goal(record)['execution_rejection'], 'native_left_turn_geometry_required')
        for change in [{'max_seconds': 11}, {'frame': 'world'}, {'coordinate_convention': 'x_forward_y_right'}]:
            with self.assertRaises(ValueError): MODULE.parse_trial_goal(dict(record, **change))

    def test_local_and_publication_epochs_are_independent(self):
        motion = MODULE.TurnMotion(900.)
        a = motion.update(scan(1, published=123456000), .01, 900., control(1))
        b = motion.update(scan(2, published=123456100), .01, 900.1, control(2, a['servo']))
        self.assertFalse(b['lock_requested'])
        self.assertEqual(b['servo']-a['servo'], 10)

    def test_pwm_interval_does_not_catch_up_after_pause(self):
        motion = MODULE.TurnMotion(0.)
        first = motion.update(scan(1), .01, 0., control(1))
        value = scan(2, published=150)
        short = motion.update(value, .01, .05, control(2, first['servo']))
        self.assertEqual(short['servo'], first['servo'])
        third = motion.update(scan(3, published=300), .01, .2, control(3, first['servo']))
        self.assertEqual(third['servo']-first['servo'], 10)

    def test_smaller_heading_error_and_trend_release_old_turn(self):
        motion, seq, now, previous, _ = self.drive()
        original = previous['servo']
        for i, heading in enumerate([55., 50., 45., 40., 35., 30.], start=1):
            result = motion.update(scan(seq+i, heading), .01, now+i*.1,
                                   control(seq+i, previous['servo'], motor=1560))
            self.assertLessEqual(abs(result['servo']-previous['servo']), 10)
            self.assertGreaterEqual(result['servo'], 1500)
            previous = result
        self.assertLess(result['servo'], original)
        self.assertLess(result['steering_target'], 1500+round(140*math.radians(30)))

    def test_alignment_requires_distinct_source_and_receive_time(self):
        motion, seq, now, previous, _ = self.drive(heading=20.)
        for i, heading in enumerate([12., 4., 3., 2., 1.], start=1):
            previous = motion.update(scan(seq+i, heading), .01, now+i*.1,
                                     control(seq+i, previous['servo'], motor=1560))
        self.assertTrue(previous['observed_alignment'])
        self.assertTrue(previous['request_coast'])
        self.assertEqual(previous['alignment_evidence'], 'double_wall')
        self.assertFalse(previous['entry_confirmed'])
        self.assertFalse(previous['completed'])
        self.assertEqual(previous['motor'], 1500)

    def test_coast_latches_neutral_even_after_new_targets_and_time(self):
        motion, seq, now, previous, _ = self.drive()
        motion.begin_coast('test_stop', now)
        for i in range(1, 30):
            result = motion.update(scan(seq+i, heading=60.), .01, now+i*.1,
                                   control(seq+i, previous['servo'], motor=1500))
            self.assertEqual(result['motor'], 1500)
            self.assertEqual(result['phase'], 'coast')
            self.assertLessEqual(abs(result['servo']-previous['servo']), 10)
            previous = result
        self.assertEqual(previous['servo'], 1500)

    def test_coast_consumes_fresh_geometry_for_bounded_correction_without_drive(self):
        motion, seq, now, previous, _ = self.drive(heading=20.)
        motion.begin_coast('planned', now)
        targets = []
        for i in range(1, 26):
            heading, offset = (15., .12) if i == 1 else (10., -.12)
            result = motion.update(scan(seq+i, heading, offset), .01, now+i*.1,
                                   control(seq+i, previous['servo'], motor=1500))
            self.assertEqual((result['phase'], result['motor']), ('coast', 1500))
            self.assertEqual(result['geometry_source_seq'], seq+i)
            self.assertLessEqual(abs(result['servo']-previous['servo']), 10)
            self.assertTrue(1445 <= result['steering_target'] <= 1555)
            self.assertFalse(result['completed'])
            targets.append(result['steering_target'])
            previous = result
        self.assertGreater(targets[0], 1500)
        self.assertLess(targets[1], 1500)
        self.assertEqual(previous['servo'], previous['steering_target'])

    def test_coast_missing_geometry_slews_neutral_without_reusing_old_geometry(self):
        motion, seq, now, previous, _ = self.drive()
        source_seq = previous['geometry_source_seq']
        motion.begin_coast('planned', now)
        for i in range(1, 41):
            result = motion.update(dict(scan(seq+i), corridor_candidates=[]), .01, now+i*.1,
                                   control(seq+i, previous['servo'], motor=1500))
            self.assertEqual((result['phase'], result['motor']), ('coast', 1500))
            self.assertEqual(result['steering_target'], 1500)
            self.assertEqual(result['terminal_reason'], 'planned')
            self.assertEqual(result['geometry_source_seq'], source_seq)
            self.assertTrue(result['geometry_missing'])
            self.assertLessEqual(abs(result['servo']-previous['servo']), 10)
            previous = result
        self.assertEqual(previous['servo'], 1500)

    def test_coast_faults_including_duplicate_scan_feedback_lock_immediately(self):
        for fault in ['stale', 'changed', 'reordered_scan', 'clock_gap', 'unarmed', 'old_tick', 'old_seq', 'malformed']:
            with self.subTest(fault=fault):
                motion, seq, now, previous, _ = self.drive()
                motion.begin_coast('planned', now)
                value, age, received = scan(seq), .01, now+.1
                feedback = control(seq+1, previous['servo'], motor=1500)
                if fault == 'stale': age = .3
                if fault == 'changed': value = scan(seq, heading=61.)
                if fault == 'reordered_scan': value = scan(seq-1)
                if fault == 'clock_gap': value, received = scan(seq+4), now+.4
                if fault == 'unarmed': feedback['armed'] = False
                if fault == 'old_tick': feedback['tick'] = 0
                if fault == 'old_seq': feedback['seq'] = 0
                if fault == 'malformed': feedback['servo'] = True
                result = motion.update(value, age, received, feedback)
                self.assertTrue(result['lock_requested'])
                self.assertEqual((result['motor'], result['servo']), (1500, 1500))
                returned = motion.update(scan(seq+5), .01, now+.5, control(seq+5))
                self.assertEqual(returned['phase'], 'locked')

    def test_tracked_wall_behind_car_does_not_restore_left_and_safe_exit_can_align(self):
        motion = MODULE.TurnMotion(0.)
        previous = {'servo': 1500}
        for seq in range(1, 49):
            now = (seq-1)*.1
            previous = motion.update(opening_scan(seq), .01, now, control(seq, previous['servo']))
            if previous['phase'] == 'drive': break
        for i in range(1, 17):
            heading = max(0., 60.-i*5)
            rho = -1.8+min(i, 12)*.1+.12
            # A continuous slide of observed finite support, rather than a
            # one-frame four-metre jump, eventually puts it behind the car.
            support = wall(heading, rho, 1.-.35*i, 3.-.35*i)
            value = dict(scan(seq+i), corridor_candidates=[], wall_candidates=[support], left_turn_goal=None)
            result = motion.update(value, .01, now+i*.1, control(seq+i, previous['servo'], motor=1560))
            self.assertFalse(result['lock_requested'])
            if i >= 12:
                self.assertLessEqual(result['steering_target'], 1528)
                self.assertLessEqual(abs(result['servo']-previous['servo']), 10)
            previous = result
        self.assertTrue(previous['observed_alignment'])
        self.assertEqual(previous['alignment_evidence'], 'tracked_goal_wall')
        self.assertEqual(previous['geometry_mode'], 'opening_wall')
        self.assertAlmostEqual(previous['corridor']['center_offset_left_m'], .12)
        self.assertFalse(previous['entry_confirmed'])
        self.assertFalse(previous['completed'])

    def test_drive_deadline_uses_trial_time_and_cannot_extend(self):
        motion, seq, now, previous, _ = self.drive(max_drive_s=.2)
        for i in range(1, 4):
            result = motion.update(scan(seq+i), .01, now+i*.1, control(seq+i, previous['servo']))
            previous = result
        self.assertEqual(result['terminal_reason'], 'left_turn_drive_timeout')
        self.assertTrue(result['request_coast'])
        self.assertEqual(result['motor'], 1500)

    def test_stale_scan_at_drive_deadline_still_locks_immediately(self):
        motion, seq, now, previous, _ = self.drive(max_drive_s=.1)
        result = motion.update(scan(seq+1), .3, now+.1, control(seq+1, previous['servo']))
        self.assertTrue(result['lock_requested'])
        self.assertEqual((result['motor'], result['servo']), (1500, 1500))

    def test_presteer_deadline_cannot_wait_forever(self):
        motion = MODULE.TurnMotion(0.)
        result = motion.update(scan(1), .01, 5., control(1))
        self.assertEqual(result['terminal_reason'], 'left_turn_presteer_timeout')

    def test_close_obstacle_overrides_presteer_drive_and_coast(self):
        for phase in ['presteer', 'drive', 'coast']:
            with self.subTest(phase=phase):
                if phase == 'presteer':
                    motion, seq, now, previous = MODULE.TurnMotion(0.), 0, 0., {'servo': 1500}
                else:
                    motion, seq, now, previous, _ = self.drive()
                    if phase == 'coast': motion.begin_coast('planned', now)
                result = motion.update(scan(seq+1), .01, now+.1, control(seq+1, previous['servo']), safe=False)
                self.assertTrue(result['lock_requested'])
                self.assertEqual((result['motor'], result['servo']), (1500, 1500))

    def test_entry_evidence_must_match_fresh_native_scan_and_still_is_trial(self):
        for valid in [True, False]:
            with self.subTest(valid=valid):
                motion, seq, now, previous, _ = self.drive()
                evidence = {'kind': 'cone', 'source_seq': seq+1 if valid else seq,
                            'source_at_ms': (seq+1)*100, 'distance_forward_m': 1.2,
                            'error_m': .04, 'candidate_only': True}
                result = motion.update(scan(seq+1), .01, now+.1,
                                       control(seq+1, previous['servo']), entry_stop=evidence)
                self.assertEqual(result['entry_confirmed'], valid)
                self.assertEqual(result['request_coast'], valid)
                self.assertEqual(result['lock_requested'], not valid)
                self.assertFalse(result['completed'])
                self.assertEqual(result['motor'], 1500)

    def test_fault_never_restarts_when_geometry_returns(self):
        motion = MODULE.TurnMotion(0.)
        motion.update(dict(scan(1), corridor_candidates=[]), .01, 0., control(1))
        for seq in range(2, 10):
            result = motion.update(scan(seq), .01, seq*.1, control(seq))
            self.assertEqual(result['phase'], 'locked')
            self.assertEqual(result['motor'], 1500)

    def test_software_feedback_loss_or_reordering_latches_stop(self):
        for fault in ['unarmed', 'old_tick', 'old_seq', 'malformed']:
            with self.subTest(fault=fault):
                motion, seq, now, previous, _ = self.drive()
                feedback = control(seq+1, previous['servo'], motor=1560)
                if fault == 'unarmed': feedback['armed'] = False
                if fault == 'old_tick': feedback['tick'] = 0
                if fault == 'old_seq': feedback['seq'] = 0
                if fault == 'malformed': feedback['servo'] = True
                result = motion.update(scan(seq+1), .01, now+.1, feedback)
                self.assertTrue(result['lock_requested'])
                self.assertEqual((result['motor'], result['servo']), (1500, 1500))

    def test_invalid_configuration_and_source_finite_values(self):
        for kwargs in [{'started_at': False}, {'started_at': 0, 'max_drive_s': 11},
                       {'started_at': 0, 'motor_pwm': 1580}, {'started_at': 0, 'max_presteer_s': 6}]:
            with self.assertRaises(ValueError): MODULE.TurnMotion(**kwargs)
        for bad in [math.nan, math.inf, True]:
            value = scan(1)
            value['corridor_candidates'][0]['heading_left_rad'] = bad
            result = MODULE.TurnMotion(0.).update(value, .01, 0., control(1))
            self.assertTrue(result['lock_requested'])


if __name__ == '__main__':
    unittest.main()
