"""Conservative scan-to-scan low-motion evidence; no hardware or wheel odometry.

at_ms is the bridge whole-scan publication clock, not a point acquisition clock.
At least two associated nonparallel extended planes must support a rigid motion.
Parallel corridor walls cannot observe along-corridor translation. Unknown
geometry, stale/reordered data, ambiguous plane identity or inconsistent fits
never certify stationary. This assumes static planes and fixed lidar extrinsics;
it does not establish braking distance or compensate scan acquisition distortion.
"""
import math
import statistics
import threading
import time


SOFTWARE_THRESHOLDS = {
    'min_publication_stable_s': .5,
    'min_receive_stable_s': .5,
    'max_clock_progress_difference_s': .15,
    'min_low_motion_pairs': 5,
    'max_low_speed_mps': .05,
    'max_low_yaw_rate_rps': .03,
    'max_low_pair_translation_m': .008,
    'max_stable_total_translation_m': .02,
    'max_stable_total_rotation_rad': .02,
}


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _wrap(angle):
    return (angle+math.pi) % (2*math.pi)-math.pi


def _fit(points):
    # Coordinates are floats; fmean avoids exact Fraction arithmetic per fit
    # while retaining an accurately summed floating-point centroid.
    cx = statistics.fmean(p[0] for p in points)
    cy = statistics.fmean(p[1] for p in points)
    xx = sum((x-cx)**2 for x, y in points)
    xy = sum((x-cx)*(y-cy) for x, y in points)
    yy = sum((y-cy)**2 for x, y in points)
    angle = .5*math.atan2(2*xy, xx-yy)
    tx, ty = math.cos(angle), math.sin(angle)
    nx, ny = -ty, tx
    rho = nx*cx+ny*cy
    if rho < 0:
        nx, ny, rho = -nx, -ny, -rho
    residuals = [nx*x+ny*y-rho for x, y in points]
    projections = [tx*x+ty*y for x, y in points]
    return {'nx': nx, 'ny': ny, 'rho': rho, 'angle': math.atan2(ny, nx),
            'points': points, 'span': max(projections)-min(projections),
            'rmse': math.sqrt(sum(r*r for r in residuals)/len(points)), 'residuals': residuals}


def _planes(ranges):
    groups, group, previous_angle = [], [], None
    for angle, r in enumerate(ranges):
        if r is None:
            continue
        p = r*math.cos(math.radians(angle)), r*math.sin(math.radians(angle))
        if group and (angle-previous_angle > 3 or math.dist(group[-1], p) > .30):
            groups.append(group); group = []
        group.append(p); previous_angle = angle
    if group:
        groups.append(group)
    if len(groups) > 1 and math.dist(groups[-1][-1], groups[0][0]) <= .30:
        groups[0] = groups.pop()+groups[0]
    lines = []
    for group in groups:
        circular = math.dist(group[0], group[-1]) <= .30 and len(group) > 96
        extended = group+group[:95] if circular else group
        for start in range(0, len(group), 8):
            for size in [12, 24, 48, 96]:
                points = extended[start:start+size]
                if len(points) < size:
                    continue
                line = _fit(points)
                if (line['span'] >= .35 and line['rmse'] <= .015
                        and max(map(abs, line['residuals'])) <= .035):
                    lines.append(line)
    merged = []
    for line in sorted(lines, key=lambda p: len(p['points']), reverse=True):
        for index, old in enumerate(merged):
            if abs(_wrap(line['angle']-old['angle'])) < math.radians(2) and abs(line['rho']-old['rho']) < .035:
                joined = _fit(list(dict.fromkeys(old['points']+line['points'])))
                if joined['rmse'] <= .015 and max(map(abs, joined['residuals'])) <= .035:
                    merged[index] = joined
                    break
        else:
            merged.append(line)
    # Many local tangents on a smooth curve are not stable plane identities.
    # Never truncate an ambiguous cloud into a convenient apparent room.
    return merged if len(merged) <= 12 else []


def _register(previous, current, dt):
    matches = []
    for line in current:
        candidates = [old for old in previous
                      if abs(_wrap(old['angle']-line['angle'])) <= .15
                      and abs(old['rho']-line['rho']) <= min(.80, 5*dt+.035)]
        if len(candidates) == 1:
            matches.append((candidates[0], line))
    # One-to-one identity, rather than a convenient nearest-line match.
    ids = [id(old) for old, _ in matches]
    if len(ids) != len(set(ids)):
        return None, 'ambiguous_plane_identity'
    if len(matches) != len(previous) or len(matches) != len(current):
        return None, 'plane_identity_unmatched'
    if len(matches) < 2:
        return None, 'insufficient_associated_planes'
    if not any(abs(a['nx']*b['ny']-a['ny']*b['nx']) >= .5
               for a, _ in matches for b, _ in matches):
        return None, 'parallel_planes_unobservable'
    rotations = [_wrap(old['angle']-line['angle']) for old, line in matches]
    rotation = statistics.median(rotations)
    if max(abs(_wrap(r-rotation)) for r in rotations) > math.radians(1.5):
        return None, 'inconsistent_plane_rotation'
    aa = sum(old['nx']**2 for old, _ in matches)
    ab = sum(old['nx']*old['ny'] for old, _ in matches)
    bb = sum(old['ny']**2 for old, _ in matches)
    ax = sum(old['nx']*(old['rho']-line['rho']) for old, line in matches)
    bx = sum(old['ny']*(old['rho']-line['rho']) for old, line in matches)
    determinant = aa*bb-ab*ab
    if determinant/(aa+bb)**2 < .03:
        return None, 'ill_conditioned_plane_geometry'
    tx, ty = (ax*bb-bx*ab)/determinant, (bx*aa-ax*ab)/determinant
    c, s = math.cos(rotation), math.sin(rotation)
    errors, inliers, count = [], 0, 0
    for old, line in matches:
        transformed = [(c*x-s*y+tx, s*x+c*y+ty) for x, y in line['points']]
        residuals = [old['nx']*x+old['ny']*y-old['rho'] for x, y in transformed]
        errors.append(sum(r*r for r in residuals)/len(residuals))
        inliers += sum(abs(r) <= .035 for r in residuals); count += len(residuals)
        tangent = -old['ny'], old['nx']
        old_along = [tangent[0]*x+tangent[1]*y for x, y in old['points']]
        new_along = [tangent[0]*x+tangent[1]*y for x, y in transformed]
        overlap = min(max(old_along), max(new_along))-max(min(old_along), min(new_along))
        if overlap < .20 or overlap < .4*min(old['span'], line['span']):
            return None, 'plane_support_changed'
    rmse = math.sqrt(statistics.mean(errors))
    if rmse > .020 or inliers < .90*count:
        return None, 'rigid_residual_too_large'
    translation = math.hypot(tx, ty)
    if translation > .80 or translation/dt > 5 or abs(rotation)/dt > 1.5:
        return None, 'motion_jump_exceeds_limits'
    return {'translation_m': translation, 'rotation_rad': rotation,
            'speed_mps': translation/dt, 'yaw_rate_rps': rotation/dt,
            'matched_planes': len(matches), 'rmse_m': rmse,
            'inlier_fraction': inliers/count}, None


class CoastMotion:
    """Both publication and receive clocks need >=0.5 s of low-motion evidence.

    Their accumulated progress must agree within 0.15 s. Five reliable pairs
    are also required; buffered scans cannot mature the window in wall time.
    stationary names an uncalibrated lidar low-motion criterion, not proof of
    physical standstill. The returned software_thresholds describe that limit.
    """
    def __init__(self):
        self.previous = None
        self.last_seq = None
        self.last_publication = None
        self.last_now = None
        self.last_receive = None
        self.last_ranges = None
        self.cached = None
        self._clear_stable()

    def _clear_stable(self):
        self.stable_since = None
        self.stable_receive_since = None
        self.stable_pairs = 0
        self.stable_translation = 0.
        self.stable_rotation = 0.

    def _diagnostics(self, publication_elapsed=0., receive_elapsed=0.):
        return {'stable_ms': round(min(publication_elapsed, receive_elapsed)*1000),
                'stable_publication_ms': round(publication_elapsed*1000),
                'stable_receive_ms': round(receive_elapsed*1000),
                'stable_pairs': self.stable_pairs,
                'stable_translation_m': self.stable_translation,
                'stable_rotation_rad': self.stable_rotation,
                'evidence_type': 'uncalibrated_lidar_low_motion',
                'software_thresholds': dict(SOFTWARE_THRESHOLDS)}

    def _unknown(self, reason, reset_reference=True):
        self._clear_stable()
        if reset_reference:
            self.previous = None
        self.cached = {'observable': False, 'stationary': False, 'low_motion': False, 'translation_m': None,
                       'rotation_rad': None, 'speed_mps': None, 'yaw_rate_rps': None,
                       'reason': reason, **self._diagnostics()}
        return dict(self.cached)

    def update(self, scan, lidar_age_s, now):
        if not _finite(now) or now < 0 or (self.last_now is not None and now < self.last_now):
            return self._unknown('invalid_receive_clock')
        self.last_now = now
        if not _finite(lidar_age_s) or not 0 <= lidar_age_s < .30:
            return self._unknown('lidar_stale_or_invalid_age')
        if not isinstance(scan, dict) or scan.get('frame_id') != 'lidar_origin_coarse_body_heading':
            return self._unknown('unknown_lidar_frame')
        seq, at = scan.get('seq'), scan.get('at_ms')
        if type(seq) is not int or seq < 0 or type(at) is not int or at < 0:
            return self._unknown('invalid_scan_clock')
        if self.last_seq is not None and seq == self.last_seq:
            if at != self.last_publication:
                return self._unknown('duplicate_sequence_changed_clock')
            if not isinstance(scan.get('ranges'), list) or tuple(scan['ranges']) != self.last_ranges:
                return self._unknown('duplicate_sequence_changed_ranges')
            # Cached evidence cannot mature because server wall time passed.
            result = dict(self.cached) if self.cached else self._unknown('repeated_scan')
            result['reason'] = 'repeated_scan'
            return result
        if self.last_seq is not None and (seq < self.last_seq or at <= self.last_publication):
            return self._unknown('reordered_scan_or_publication_clock')
        publication_s = at/1000
        old_publication = self.last_publication
        old_receive = self.last_receive
        self.last_seq, self.last_publication, self.last_receive = seq, at, now
        ranges = scan.get('ranges')
        if (not isinstance(ranges, list) or len(ranges) != 360
                or any(r is not None and (not _finite(r) or not .02 <= r <= 12) for r in ranges)):
            return self._unknown('invalid_scan_ranges')
        self.last_ranges = tuple(ranges)
        if sum(r is not None for r in ranges) < 324:
            return self._unknown('scan_coverage_insufficient')
        current = _planes(ranges)
        if len(current) < 2:
            return self._unknown('insufficient_extended_planes')
        if not any(abs(a['nx']*b['ny']-a['ny']*b['nx']) >= .5 for a in current for b in current):
            return self._unknown('parallel_planes_unobservable')
        if self.previous is None:
            self.previous = publication_s, current
            return self._unknown('reference_initialized', reset_reference=False)
        previous_publication_s, previous = self.previous
        dt = publication_s-previous_publication_s
        if not .05 <= dt < .30:
            self.previous = publication_s, current
            return self._unknown('scan_gap_or_rate_invalid', reset_reference=False)
        if old_publication is not None and old_receive is not None and abs((at-old_publication)/1000-(now-old_receive)) > .15:
            self.previous = publication_s, current
            return self._unknown('publication_receive_clock_disagreement', reset_reference=False)
        motion, error = _register(previous, current, dt)
        self.previous = publication_s, current
        if error:
            return self._unknown(error, reset_reference=False)
        limits = SOFTWARE_THRESHOLDS
        low = (motion['speed_mps'] <= limits['max_low_speed_mps']
               and abs(motion['yaw_rate_rps']) <= limits['max_low_yaw_rate_rps']
               and motion['translation_m'] <= limits['max_low_pair_translation_m'])
        reset_reason = None
        if not low:
            self._clear_stable()
        else:
            if self.stable_since is None:
                self.stable_since = previous_publication_s
                self.stable_receive_since = old_receive
            self.stable_pairs += 1
            self.stable_translation += motion['translation_m']
            self.stable_rotation += abs(motion['rotation_rad'])
            if (self.stable_translation > limits['max_stable_total_translation_m']
                    or self.stable_rotation > limits['max_stable_total_rotation_rad']):
                self._clear_stable()
                reset_reason = 'stable_total_motion_exceeds_limits'
        elapsed = 0 if self.stable_since is None else publication_s-self.stable_since
        received = 0 if self.stable_receive_since is None else now-self.stable_receive_since
        if abs(elapsed-received) > limits['max_clock_progress_difference_s']:
            self._clear_stable()
            elapsed, received = 0., 0.
            reset_reason = 'stable_clock_progress_disagreement'
        stationary = (self.stable_pairs >= limits['min_low_motion_pairs']
                      and elapsed >= limits['min_publication_stable_s']
                      and received >= limits['min_receive_stable_s'])
        self.cached = {'observable': True, 'stationary': stationary, 'low_motion': low, **motion,
                       'reason': reset_reason or ('stable_observed_low_motion' if stationary else
                                 'collecting_low_motion_evidence' if low else 'observed_motion'),
                       **self._diagnostics(elapsed, received)}
        return dict(self.cached)


class CoastMotionWorker:
    """One fit in progress and at most one latest pending scan, without control locks.

    Generation changes revoke both queued and completed evidence. Results are
    usable only for the exact currently received scan, within its original age
    limit; a stationary result from an older scan cannot stop a newer motion.
    """
    def __init__(self, estimator_factory=CoastMotion, autostart=True):
        self._condition = threading.Condition()
        self._factory = estimator_factory
        self._generation = 0
        self._closed = False
        self._pending = None
        self._inflight = None
        self._result = None
        self._last_input = None
        self._motion = None
        self._motion_generation = None
        self._thread = None
        if autostart:
            self._thread = threading.Thread(target=self._run, name='coast-motion', daemon=True)
            self._thread.start()

    @staticmethod
    def unknown(reason):
        return CoastMotion()._unknown(reason)

    @staticmethod
    def _identity(scan, received):
        return (scan.get('seq'), scan.get('at_ms'), scan.get('frame_id'),
                received, tuple(scan.get('ranges', [])))

    @staticmethod
    def _input_error(scan, age, received, now):
        if not isinstance(scan, dict) or not isinstance(scan.get('ranges'), list):
            return 'invalid_scan_ranges'
        if (not _finite(received) or received < 0 or not _finite(now)
                or not _finite(age) or not 0 <= age < .30
                or not 0 <= now-received < .30):
            return 'motion_result_stale_receive'
        if abs((now-received)-age) > .15:
            return 'motion_result_receive_age_disagreement'
        if (type(scan.get('seq')) is not int or scan['seq'] < 0
                or type(scan.get('at_ms')) is not int or scan['at_ms'] < 0):
            return 'invalid_scan_clock'
        return None

    def reset(self):
        with self._condition:
            self._generation += 1
            self._pending = self._result = self._last_input = None
            self._condition.notify_all()
            return self._generation

    def close(self):
        with self._condition:
            self._closed = True
            self._generation += 1
            self._pending = self._result = self._last_input = None
            self._condition.notify_all()
        # Never wait for fitting while the caller holds the control lock.

    def submit(self, generation, scan, age, received, now):
        error = self._input_error(scan, age, received, now)
        if error:
            return error
        identity = self._identity(scan, received)
        with self._condition:
            if self._closed or generation != self._generation:
                return 'motion_worker_generation_changed'
            if identity != self._last_input:
                # Only immutable identity and a private range list cross threads.
                snapshot = {key: scan.get(key) for key in ('seq', 'at_ms', 'frame_id')}
                snapshot['ranges'] = list(scan['ranges'])
                self._pending = (generation, identity, snapshot, age, received, now)
                self._last_input = identity
                self._condition.notify()
        return None

    def result(self, generation, scan, age, received, now):
        error = self._input_error(scan, age, received, now)
        if error:
            return self.unknown(error)
        identity = self._identity(scan, received)
        with self._condition:
            if self._closed or generation != self._generation:
                return self.unknown('motion_worker_generation_changed')
            result = self._result
            if result is None:
                return self.unknown('motion_estimator_pending')
            if result[0] != generation or result[1] != identity:
                return self.unknown('motion_result_superseded')
            if not 0 <= now-result[2] < .30:
                return self.unknown('motion_result_stale_submission')
            return dict(result[3])

    def _take_pending(self):
        with self._condition:
            if self._closed or self._pending is None or self._inflight is not None:
                return None
            item, self._pending = self._pending, None
            self._inflight = item[:2]
            return item

    def _compute(self, item):
        generation, identity, scan, age, received, submitted = item
        if self._motion_generation != generation:
            self._motion, self._motion_generation = self._factory(), generation
        started = time.perf_counter()
        try:
            # Receive progress is the original accepted scan's clock, never the
            # time a slow fit completes or a repeated control tick polls it.
            motion = self._motion.update(scan, age, received)
        except Exception:
            self._motion = self._factory()
            motion = self.unknown('motion_estimator_failed')
        motion = {**motion, 'source_scan_seq': scan['seq'], 'source_publication_at_ms': scan['at_ms'],
                  'source_receive_monotonic_s': received,
                  'processing_ms': (time.perf_counter()-started)*1000}
        with self._condition:
            self._inflight = None
            if not self._closed and generation == self._generation:
                self._result = generation, identity, submitted, motion
            self._condition.notify_all()

    def run_pending(self):
        """Advance an explicitly unscheduled worker in deterministic offline tests."""
        if self._thread is not None:
            raise RuntimeError('worker thread already owns fitting')
        item = self._take_pending()
        if item is not None:
            self._compute(item)

    def _run(self):
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._closed or self._pending is not None)
                if self._closed:
                    return
            item = self._take_pending()
            if item is not None:
                self._compute(item)
