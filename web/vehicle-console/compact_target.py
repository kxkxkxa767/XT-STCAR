"""Fresh, isolated compact lidar objects for a bounded first-target trial.

These are experimental shape/association gates, not measured cone dimensions,
semantic classification, vehicle motion or a free-space/path certificate. Ranges
use the bridge's calibrated 360 clockwise bins; output coordinates are x forward,
y left. The caller must attach the actual scan receive time as ``received_at``
(server.scan_at), never a new time for an old observation.
"""
import copy
import math


FRAME = 'lidar_origin_coarse_body_heading'
MAX_AGE_S = .30
MIN_CONFIRM_S = .25
MIN_POINTS = 5
MAX_POINT_GAP_M = .18
MAX_BOUNDARY_GAP_BINS = 3
MIN_DIAMETER_M = .06
MAX_DIAMETER_M = .35
MIN_RANGE_M = .35
MAX_RANGE_M = 3.
MIN_BEARING_RAD = math.radians(45)
MAX_BEARING_RAD = math.radians(135)
_DIRECTIONS = tuple((math.cos(math.radians(i)), -math.sin(math.radians(i)))
                    for i in range(360))


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


def _clusters(points):
    """Whole contiguous components, including the beam-359/0 edge."""
    def cut(a, b):
        return points[a] is None or points[b] is None or math.dist(points[a], points[b]) > MAX_POINT_GAP_M
    start = next((i for i in range(360) if cut((i-1) % 360, i)), None)
    if start is None:
        return []  # A continuous enclosure cannot supply a compact target.
    groups, group = [], []
    for offset in range(360):
        i = (start+offset) % 360
        if group and cut((i-1) % 360, i):
            groups.append(group)
            group = []
        if points[i] is not None:
            group.append(i)
    if group:
        groups.append(group)
    return groups


def _candidates(ranges, *, allow_boundary_gap=False):
    # Build components BEFORE range/sector filtering. Filtering first would
    # manufacture a small object from the end of an extended wall.
    points = [None if r is None else (r*d[0], r*d[1]) for r, d in zip(ranges, _DIRECTIONS)]
    result = []
    for group in _clusters(points):
        if not MIN_POINTS <= len(group) <= 128:
            continue
        values = [ranges[i] for i in group]
        if min(values) < MIN_RANGE_M or max(values) > MAX_RANGE_M:
            continue
        # Acquisition needs both immediate neighbors. A confirmed track may
        # cross up to three nulls at one edge, or exactly one at each edge,
        # with real farther returns beyond both edges. The tracker also limits
        # this to the last full isolation's original lease. Nulls are neither
        # object support nor free space; the whole current component stays intact.
        unknown, background, gap_counts = [], [], []
        for edge, direction in [(group[0], -1), (group[-1], 1)]:
            index = (edge+direction) % 360
            value = ranges[index]
            edge_gaps = 0
            while value is None and allow_boundary_gap and edge_gaps < MAX_BOUNDARY_GAP_BINS:
                unknown.append(index)
                edge_gaps += 1
                index = (index+direction) % 360
                value = ranges[index]
            gap_counts.append(edge_gaps)
            if value is None or value <= max(values)+MAX_POINT_GAP_M:
                break
            background.append({'index': index, 'range_m': value})
        if len(background) != 2 or (all(gap_counts) and gap_counts != [1, 1]):
            continue
        support = [points[i] for i in group]
        if (max(p[0] for p in support)-min(p[0] for p in support) > MAX_DIAMETER_M
                or max(p[1] for p in support)-min(p[1] for p in support) > MAX_DIAMETER_M):
            continue
        diameter = max(math.dist(a, b) for index, a in enumerate(support) for b in support[index+1:])
        if not MIN_DIAMETER_M <= diameter <= MAX_DIAMETER_M:
            continue
        center = [sum(p[axis] for p in support)/len(support) for axis in (0, 1)]
        distance, bearing = math.hypot(*center), math.atan2(center[1], center[0])
        if MIN_RANGE_M <= distance <= MAX_RANGE_M and MIN_BEARING_RAD <= bearing <= MAX_BEARING_RAD:
            result.append({'point_left_m': center, 'range_m': distance, 'bearing_left_rad': bearing,
                           'diameter_m': diameter, 'point_count': len(group), 'support_bins': list(group),
                           'boundary_unknown_bins': unknown, 'boundary_far_returns': background,
                           'tracking_only_boundary_gap': bool(unknown)})
    return result


class CompactTargetTracker:
    """Return a current unknown object, or None; never coast on a cached target.

    At least three distinct scans and 250 ms of BOTH publication and receive time
    confirm an unbroken, unique association with full boundary observations.
    An already confirmed identity may tolerate up to three null bins at one
    boundary, or exactly one at each boundary, using a unique current component
    and real farther returns beyond both edges. Partial frames cannot
    initialize/mature a track or renew the last FULL
    isolation's 300 ms publication/receive lease. Repeat reads also respect that
    lease, as well as the current scan's age; they cannot mature or renew either.
    ``reason`` supplies diagnostics when update returns None.
    """
    def __init__(self):
        self.reason = 'compact_target_not_observed'
        self._last_now = None
        self._last_seq = self._last_at = self._last_received = self._signature = None
        self._target = None
        self._next_id = 1
        self._first_at = self._first_received = None

    def _forget(self, reason):
        self.reason, self._target = reason, None
        self._first_at = self._first_received = None
        return None

    def update(self, scan, now):
        if not _number(now) or now < 0 or (self._last_now is not None and now < self._last_now):
            return self._forget('compact_target_invalid_receive_clock')
        self._last_now = now
        if (not isinstance(scan, dict) or scan.get('frame_id') != FRAME
                or type(scan.get('seq')) is not int or scan['seq'] < 0
                or type(scan.get('at_ms')) is not int or scan['at_ms'] < 0
                or not _number(scan.get('received_at')) or not 0 <= scan['received_at'] <= now):
            return self._forget('compact_target_invalid_scan')
        seq, published, received = scan['seq'], scan['at_ms'], scan['received_at']
        ranges = scan.get('ranges')
        valid = (isinstance(ranges, list) and len(ranges) == 360
                 and all(r is None or (_number(r) and .02 <= r <= 12) for r in ranges))
        signature = tuple(ranges) if valid else None
        if self._last_seq is not None and seq <= self._last_seq:
            if seq < self._last_seq:
                return self._forget('compact_target_reordered_scan')
            if (published != self._last_at or received != self._last_received or signature != self._signature):
                return self._forget('compact_target_duplicate_changed')
            if now-received >= MAX_AGE_S:
                return self._forget('compact_target_stale')
            if (self._target is not None and self._target['tracking_only_boundary_gap']
                    and now-self._target['last_full_isolation_received_at'] >= MAX_AGE_S):
                return self._forget('compact_target_boundary_gap_expired')
            return copy.deepcopy(self._target)
        if self._last_at is not None and (published <= self._last_at or received <= self._last_received):
            return self._forget('compact_target_reordered_clock')
        publication_dt = None if self._last_at is None else (published-self._last_at)/1000
        receive_dt = None if self._last_received is None else received-self._last_received
        self._last_seq, self._last_at, self._last_received, self._signature = seq, published, received, signature
        if not valid:
            return self._forget('compact_target_invalid_ranges')
        if now-received >= MAX_AGE_S:
            return self._forget('compact_target_stale')
        if publication_dt is not None:
            if publication_dt >= MAX_AGE_S or receive_dt >= MAX_AGE_S:
                self._forget('compact_target_observation_gap')
            elif abs(publication_dt-receive_dt) > .15:
                return self._forget('compact_target_clock_gap')
        old = self._target
        allow_boundary_gap = old is not None and old['confirmed']
        candidates = _candidates(ranges, allow_boundary_gap=allow_boundary_gap)
        if len(candidates) != 1:
            return self._forget('compact_target_ambiguous' if candidates else 'compact_target_missing')
        current = candidates[0]
        # These bounded association gates are not speed, odometry or static
        # object certification. A jump creates a new, unconfirmed identity.
        associated = (old is not None and math.dist(current['point_left_m'], old['point_left_m']) <= .30
                      and abs(current['bearing_left_rad']-old['bearing_left_rad']) <= math.radians(20)
                      and abs(current['diameter_m']-old['diameter_m']) <= .12)
        boundary_gap = current['tracking_only_boundary_gap']
        if boundary_gap and not associated:
            return self._forget('compact_target_boundary_gap_unassociated')
        if boundary_gap and ((published-old['last_full_isolation_at_ms'])/1000 >= MAX_AGE_S
                             or now-old['last_full_isolation_received_at'] >= MAX_AGE_S):
            return self._forget('compact_target_boundary_gap_expired')
        if associated:
            track_id = old['track_id']
            count = old['confirmation_count'] + (0 if boundary_gap else 1)
        else:
            track_id, count = self._next_id, 1
            self._next_id += 1
            self._first_at, self._first_received = published, received
        confirmed = (count >= 3 and published-self._first_at >= round(MIN_CONFIRM_S*1000)
                     and received-self._first_received+1e-9 >= MIN_CONFIRM_S)
        self.reason = ('compact_target_tracked_with_boundary_gap' if boundary_gap else
                       'compact_target_confirmed' if confirmed else 'compact_target_unconfirmed')
        self._target = {**current, 'source_seq': seq, 'source_at_ms': published,
                        'source_received_at': received, 'confirmed': confirmed,
                        'confirmation_count': count, 'semantic_class': 'unknown',
                        'kind': 'lidar_compact_object', 'track_id': track_id,
                        'last_full_isolation_seq': old['last_full_isolation_seq'] if boundary_gap else seq,
                        'last_full_isolation_at_ms': old['last_full_isolation_at_ms'] if boundary_gap else published,
                        'last_full_isolation_received_at': old['last_full_isolation_received_at'] if boundary_gap else received,
                        'candidate_only': True, 'physical_identity_verified': False}
        return copy.deepcopy(self._target)
