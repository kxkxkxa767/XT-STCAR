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
# Opt-in (route start lane, round 72): the first cone's approach begins at
# 3.5 m; size, isolation and return-count rules are unchanged.
EXTENDED_MAX_RANGE_M = 3.5
# Only an explicitly bound, already confirmed role may maintain its original
# identity below the acquisition range. This is the existing valid lidar
# lower bound, not an added near-object acquisition or a semantic cone claim.
CLOSE_MAINTENANCE_MIN_RANGE_M = .02
MIN_BEARING_RAD = math.radians(45)
MAX_BEARING_RAD = math.radians(135)
# Opt-in ``tolerant`` detection (explicit route). Trials 41/47/50/51/52: a
# support ~.2 m wide at .5-2 m often has 1-2 null bins at an edge (beam split
# at the depth step), an intermediate "mixed" return there, or a null inside
# it (the 5-17 deg right dropout band). Strict detection then flickers every
# other scan and trackers reset. Tolerant detection keeps the same size/range
# gates but: bridges <= TOLERANT_INTERNAL_NULLS null bins inside a support
# whose neighbors are within MAX_POINT_GAP_M; treats <= TOLERANT_EDGE_BINS
# null or not-far bins at each edge as unknown when a real farther return
# lies beyond; and bounds the unknown edge extent: diameter plus its arc must
# stay within MAX_DIAMETER_M. Such frames count as full observations.
TOLERANT_INTERNAL_NULLS = 2
TOLERANT_EDGE_BINS = 2
# Trial52 scan429: second support at 1.5 m kept 4 returns beside 2 edge nulls;
# real farther returns on both sides still bound it.
TOLERANT_MIN_POINTS = 4
# Tolerant trackers keep an identity through missing/ambiguous frames for the
# same 300 ms lease, widening the association gate by the elapsed time
# (static object, moving car: ~1.5 m/s and ~100 deg/s observed in turns).
TOLERANT_GATE_M_PER_S = 1.5
TOLERANT_GATE_RAD_PER_S = math.radians(100)
# Maintenance of an already confirmed identity (round 55, Claude). Saved
# trials 40-54: at the moment a confirmed support was lost, the commonest
# cause was a 3-5+ bin null run at its silhouette edge (the lidar drops beams
# split between a support at .6-.9 m and the far background, e.g. trial51
# scan366 bins 289-292, trial48 scan471 289-293), more than the 2-3 bins
# allowed. The unknown run is bounded by the existing size rule (observed
# diameter plus the arc of all unknown bins <= MAX_DIAMETER_M) and still needs
# a real farther return beyond it, so even if every unknown beam were occupied
# the object would still be compact and isolated: such a frame counts as a
# full observation. Acquisition keeps TOLERANT_EDGE_BINS.
TOLERANT_MAINTENANCE_EDGE_BINS = 8
# Round 57 (Claude): the same confirmed-maintenance rule accepts a 3-return
# support. Trial56 scans 2223/2225/2227: the confirmed second support at
# 2.3-2.8 m kept only 3 returns (diameter and farther background on both
# sides valid); MIN 4 dropped the identity, it was re-confirmed only at
# 1.43 m. Acquisition still needs 4 (tolerant) / 5 (strict) returns.
TOLERANT_MAINTENANCE_MIN_POINTS = 3
# Body-contact exemption: only a support that faces the sensor (its principal
# axis at >= 45 deg to the line of sight). Audit of every saved trial frame
# with a compact support within 12 cm of the body: real cones (trials 46, 48,
# 52) lie at 73-90 deg, rms .006-.02 m about the line; incoming board ends
# seen past the rear-left corner while turning (trials 23, 25, 29, 32, 34, 49)
# form a straight piece along the ray at 4-8 deg, rms <= .0033 m. A board seen
# across the ray is longer than MAX_DIAMETER_M and never compact.
CONTACT_MIN_FACING_RAD = math.radians(45)
_DIRECTIONS = tuple((math.cos(math.radians(i)), -math.sin(math.radians(i)))
                    for i in range(360))


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


def _bearing_range(value, name):
    if (not isinstance(value, (tuple, list)) or len(value) != 2
            or not all(_number(bound) for bound in value)
            or not -math.pi <= value[0] < value[1] <= math.pi):
        raise ValueError(f'{name} must be an ordered bearing interval within [-pi, pi]')
    return tuple(value)


def _associated(current, old, slack_s=0.):
    if old is None:
        return False
    bearing_delta = abs(current['bearing_left_rad']-old['bearing_left_rad'])
    if bearing_delta > math.pi:
        bearing_delta = 2*math.pi-bearing_delta
    slack_s = max(0., slack_s)
    distance_gate = .30+TOLERANT_GATE_M_PER_S*slack_s
    # Range-dependent bearing gate (round 57, Claude). A displacement within
    # distance_gate changes the bearing of a support at range r by up to
    # asin(distance_gate/r) (any angle once r <= distance_gate). Far away the
    # rotation gate is the larger one and is unchanged; near the sensor the
    # fixed angle alone rejected the same support passing the body: trial56
    # 2243 -> 2246, .378 -> .399 m, moved .442 m (gate .603 m) but 69.3 deg.
    near = min(math.hypot(*current['point_left_m']), math.hypot(*old['point_left_m']))
    translation_bearing = (math.pi if near <= distance_gate
                           else math.asin(distance_gate/near))
    bearing_gate = max(math.radians(20)+TOLERANT_GATE_RAD_PER_S*slack_s, translation_bearing)
    return (math.dist(current['point_left_m'], old['point_left_m']) <= distance_gate
            and bearing_delta <= bearing_gate
            and abs(current['diameter_m']-old['diameter_m']) <= .12)


def rotated_observation(observation, rotation_rad):
    """An observation as it appears after the car yawed ``rotation_rad`` (left +).

    Round 64 (Claude): association reference only. A fixed support's car-frame
    position turns by -rotation; on the open course the car yaws ~70 deg/s in
    a loop, which moved a support 2.4 m away by ~.27 m between two scans and
    broke the .30 m gate. Nothing else of the observation changes.
    """
    if observation is None or not _number(rotation_rad):
        return observation
    c, s = math.cos(-rotation_rad), math.sin(-rotation_rad)
    x, y = observation['point_left_m']
    point = [c*x-s*y, s*x+c*y]
    return {**observation, 'point_left_m': point, 'bearing_left_rad': math.atan2(point[1], point[0])}


def _clusters_bridged(points, bridge_nulls):
    """Components over returns only; <= bridge_nulls null bins between two
    returns within MAX_POINT_GAP_M do not split a component (nulls are never
    support). bridge_nulls=0 reproduces _clusters."""
    present = [i for i in range(360) if points[i] is not None]
    if not present:
        return []
    if len(present) == 1:
        return [present]

    def linked(a, b):
        return (b-a) % 360 <= 1+bridge_nulls and math.dist(points[a], points[b]) <= MAX_POINT_GAP_M
    count = len(present)
    start = next((k for k in range(count) if not linked(present[k-1], present[k])), None)
    if start is None:
        return []  # A continuous enclosure cannot supply a compact target.
    groups, group = [], []
    for offset in range(count):
        k = (start+offset) % count
        if group and not linked(present[k-1], present[k]):
            groups.append(group)
            group = []
        group.append(present[k])
    if group:
        groups.append(group)
    return groups


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


def _tolerant_edges(ranges, group, values, diameter, allow_boundary_gap, maintenance=False):
    """Tolerant edge rule: (unknown bins, background, gap counts, tracking_only) or None."""
    limit = (TOLERANT_MAINTENANCE_EDGE_BINS if maintenance
             else MAX_BOUNDARY_GAP_BINS if allow_boundary_gap else TOLERANT_EDGE_BINS)
    far = max(values)+MAX_POINT_GAP_M
    unknown, background, gap_counts = [], [], []
    for edge, direction in [(group[0], -1), (group[-1], 1)]:
        found, count = None, 0
        for step in range(1, limit+2):
            index = (edge+direction*step) % 360
            value = ranges[index]
            if value is not None and value > far:
                found = {'index': index, 'range_m': value}
                break
            if step > limit:
                break
            unknown.append(index)  # null, or a not-far (mixed / nearer) return
            count += 1
        if found is None:
            return None
        background.append(found)
        gap_counts.append(count)
    if any(gap_counts) and diameter+max(values)*math.radians(sum(gap_counts)) > MAX_DIAMETER_M:
        return None
    return unknown, background, gap_counts, not maintenance and max(gap_counts) > TOLERANT_EDGE_BINS


def _candidates(ranges, *, allow_boundary_gap=False,
                bearing_range_rad=(MIN_BEARING_RAD, MAX_BEARING_RAD), tolerant=False,
                min_range_m=MIN_RANGE_M, maintenance=False, min_points=None, max_range_m=MAX_RANGE_M):
    bearing_min, bearing_max = _bearing_range(bearing_range_rad, 'bearing_range_rad')
    if type(allow_boundary_gap) is not bool or type(tolerant) is not bool or type(maintenance) is not bool:
        raise ValueError('allow_boundary_gap, tolerant and maintenance must be bools')
    if maintenance and not tolerant:
        raise ValueError('maintenance edges are a tolerant-detection rule')
    if not _number(min_range_m) or not CLOSE_MAINTENANCE_MIN_RANGE_M <= min_range_m <= MIN_RANGE_M:
        raise ValueError('min_range_m must be within the valid lidar and acquisition bounds')
    if not _number(max_range_m) or not MAX_RANGE_M <= max_range_m <= EXTENDED_MAX_RANGE_M:
        raise ValueError('max_range_m must be within the acquisition bounds')
    # Build components BEFORE range/sector filtering. Filtering first would
    # manufacture a small object from the end of an extended wall.
    points = [None if r is None else (r*d[0], r*d[1]) for r, d in zip(ranges, _DIRECTIONS)]
    result = []
    for group in (_clusters_bridged(points, TOLERANT_INTERNAL_NULLS) if tolerant else _clusters(points)):
        minimum = (min_points if min_points is not None
                   else TOLERANT_MAINTENANCE_MIN_POINTS if maintenance
                   else TOLERANT_MIN_POINTS if tolerant else MIN_POINTS)
        if not minimum <= len(group) <= 128:
            continue
        values = [ranges[i] for i in group]
        if min(values) < min_range_m or max(values) > max_range_m:
            continue
        support = [points[i] for i in group]
        if (max(p[0] for p in support)-min(p[0] for p in support) > MAX_DIAMETER_M
                or max(p[1] for p in support)-min(p[1] for p in support) > MAX_DIAMETER_M):
            continue
        diameter = max(math.dist(a, b) for index, a in enumerate(support) for b in support[index+1:])
        if not MIN_DIAMETER_M <= diameter <= MAX_DIAMETER_M:
            continue
        if tolerant:
            edges = _tolerant_edges(ranges, group, values, diameter, allow_boundary_gap, maintenance)
            if edges is None:
                continue
            unknown, background, gap_counts, tracking_only = edges
            center = [sum(p[axis] for p in support)/len(support) for axis in (0, 1)]
            distance, bearing = math.hypot(*center), math.atan2(center[1], center[0])
            if min_range_m <= distance <= max_range_m and bearing_min <= bearing <= bearing_max:
                internal = [i for i in range(group[0], group[0]+(group[-1]-group[0]) % 360+1)
                            if i % 360 not in group]
                result.append({'point_left_m': center, 'range_m': distance, 'bearing_left_rad': bearing,
                               'diameter_m': diameter, 'point_count': len(group), 'support_bins': list(group),
                               'boundary_unknown_bins': unknown, 'boundary_far_returns': background,
                               'internal_null_bins': [i % 360 for i in internal],
                               'boundary_gap_projected_span_m': (max(values)*math.radians(sum(gap_counts))
                                                                 if any(gap_counts) else None),
                               'tracking_only_boundary_gap': tracking_only, 'tolerant_detection': True})
            continue
        # Acquisition needs both immediate neighbors. A confirmed track may
        # cross up to three nulls at one edge, or exactly one at each edge,
        # with real farther returns beyond both edges. Longer single-edge gaps
        # need the bounded projection below, with no gap at the opposite edge.
        # Nulls are neither support nor free space; the current component stays intact.
        unknown, background, gap_counts = [], [], []
        for edge, direction in [(group[0], -1), (group[-1], 1)]:
            index = (edge+direction) % 360
            value = ranges[index]
            edge_gaps = 0
            while value is None and allow_boundary_gap and edge_gaps < len(ranges):
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
        projected_gap = None
        if max(gap_counts) > MAX_BOUNDARY_GAP_BINS:
            # Arc at the farthest ACTUAL support range, spanning the boundary
            # beams. This only bounds an association projection, not the physical
            # contents/depth of missing beams or an observed object diameter.
            projected_gap = max(values)*math.radians(max(gap_counts)+1)
            if (projected_gap > min(MAX_POINT_GAP_M, diameter)
                    or diameter+projected_gap > MAX_DIAMETER_M):
                continue
        center = [sum(p[axis] for p in support)/len(support) for axis in (0, 1)]
        distance, bearing = math.hypot(*center), math.atan2(center[1], center[0])
        if min_range_m <= distance <= max_range_m and bearing_min <= bearing <= bearing_max:
            result.append({'point_left_m': center, 'range_m': distance, 'bearing_left_rad': bearing,
                           'diameter_m': diameter, 'point_count': len(group), 'support_bins': list(group),
                           'boundary_unknown_bins': unknown, 'boundary_far_returns': background,
                           'boundary_gap_projected_span_m': projected_gap,
                           'tracking_only_boundary_gap': bool(unknown)})
    return result


def _close_candidates(ranges, *, bearing_range_rad=(-math.pi, math.pi)):
    """Current near (< MIN_RANGE_M) supports under the confirmed-maintenance rules.

    Round 55 (Claude): the earlier strict near rule (no null, no mixed edge,
    no range step) lost the real cone in trial54 scan328 (.33-.38 m with one
    null and .42-.49 m mixed edge returns). Near supports now use the same
    tolerant rules as far ones; size and isolation bounds are unchanged. The
    tracker still uses them only for a bound, already confirmed identity.
    """
    return [candidate for candidate in _candidates(
                ranges, bearing_range_rad=bearing_range_rad, tolerant=True, allow_boundary_gap=True,
                maintenance=True, min_range_m=CLOSE_MAINTENANCE_MIN_RANGE_M)
            if any(ranges[i] < MIN_RANGE_M for i in candidate['support_bins'])]


def faces_sensor(ranges, bins):
    """Principal axis of the support points at >= CONTACT_MIN_FACING_RAD to the mean ray."""
    points = [(ranges[i]*_DIRECTIONS[i][0], ranges[i]*_DIRECTIONS[i][1]) for i in bins]
    if len(points) < TOLERANT_MIN_POINTS:
        return False
    mx = sum(p[0] for p in points)/len(points)
    my = sum(p[1] for p in points)/len(points)
    sxx = sum((p[0]-mx)**2 for p in points)
    syy = sum((p[1]-my)**2 for p in points)
    sxy = sum((p[0]-mx)*(p[1]-my) for p in points)
    axis = .5*math.atan2(2*sxy, sxx-syy)
    ray = math.atan2(my, mx)
    angle = abs((axis-ray+math.pi/2) % math.pi-math.pi/2)
    return angle >= CONTACT_MIN_FACING_RAD


def contact_supports(ranges):
    """Every current isolated compact support that faces the sensor (cone-like).

    Used only for the explicit route's body-contact exemption (user,
    2026-10-12: a cone may be hit and the run continues; boards still stop).
    Tolerant detection with the confirmed-track edge allowance (<= 3 bins),
    any bearing, down to the valid lidar range. Not a semantic classification.
    """
    return [candidate for candidate in _candidates(
                ranges, bearing_range_rad=(-math.pi, math.pi), tolerant=True, allow_boundary_gap=True,
                min_range_m=CLOSE_MAINTENANCE_MIN_RANGE_M)
            if faces_sensor(ranges, candidate['support_bins'])]


class CompactTargetTracker:
    """Return a current unknown object, or None; never coast on a cached target.

    At least three distinct scans and 250 ms of BOTH publication and receive time
    confirm an unbroken, unique association with full boundary observations.
    An already confirmed identity may tolerate up to three null bins at one
    boundary, or exactly one at each boundary. A longer single-boundary gap must
    have a range/angle projection no larger than both the actual component's
    diameter and the point-gap limit, and component diameter plus projection
    cannot exceed the existing diameter limit. All require a unique current
    component and real farther returns beyond both edges. With
    ``select_associated=True``, unrelated components may coexist, but exactly one
    component must pass the original association gates for a confirmed identity.
    The optional maintenance sector applies only to confirmed identities;
    unconfirmed/new identities always use the acquisition sector. Partial frames cannot
    initialize/mature a track or renew the last FULL isolation's 300 ms
    publication/receive lease. Repeat reads also respect that
    lease, as well as the current scan's age; they cannot mature or renew either.
    ``reason`` supplies diagnostics when update returns None.
    """
    def __init__(self, *, acquisition_bearing_rad=(MIN_BEARING_RAD, MAX_BEARING_RAD),
                 maintenance_bearing_rad=None, select_associated=False, unconfirmed_boundary_gap=False,
                 tolerant=False, acquisition_min_points=None):
        self._acquisition_bearing_rad = _bearing_range(acquisition_bearing_rad, 'acquisition_bearing_rad')
        self._maintenance_bearing_rad = (None if maintenance_bearing_rad is None else
                                         _bearing_range(maintenance_bearing_rad, 'maintenance_bearing_rad'))
        if type(select_associated) is not bool:
            raise ValueError('select_associated must be a bool')
        if type(unconfirmed_boundary_gap) is not bool or type(tolerant) is not bool:
            raise ValueError('unconfirmed_boundary_gap and tolerant must be bools')
        self._tolerant = tolerant
        if acquisition_min_points is not None and (type(acquisition_min_points) is not int
                or not TOLERANT_MAINTENANCE_MIN_POINTS <= acquisition_min_points <= MIN_POINTS or not tolerant):
            raise ValueError('acquisition_min_points must be a tolerant 3..5 return count')
        self._acquisition_min_points = acquisition_min_points
        self._acquisition_max_range_m = MAX_RANGE_M
        self.held = None  # tolerant: identity kept through this missing/ambiguous frame
        self._select_associated = select_associated
        # Opt-in (explicit route first target). Trial51: the first support at
        # ~1 m had 1-2 null bins at one edge in about every other scan (the
        # beam split between the near support and the far board), so an
        # unconfirmed identity was forgotten each time and three full frames
        # in a row never occurred. The same bounded boundary-gap rule used for
        # confirmed identities may keep an UNCONFIRMED identity alive; such a
        # frame never counts toward confirmation or renews full isolation.
        self._unconfirmed_boundary_gap = unconfirmed_boundary_gap
        self.reason = 'compact_target_not_observed'
        self._last_now = None
        self._last_seq = self._last_at = self._last_received = self._signature = None
        self._target = None
        self._next_id = 1
        self._first_at = self._first_received = None
        self._close_maintenance_track_id = None

    def extend_acquisition_range(self, max_range_m):
        """Opt-in farther acquisition (route first target from the start lane)."""
        if not _number(max_range_m) or not MAX_RANGE_M <= max_range_m <= EXTENDED_MAX_RANGE_M:
            raise ValueError('max_range_m must be within the acquisition bounds')
        self._acquisition_max_range_m = max_range_m

    def widen_acquisition(self, bearing_range_rad):
        """Replace the acquisition sector (route first target after handover)."""
        self._acquisition_bearing_rad = _bearing_range(bearing_range_rad, 'acquisition_bearing_rad')

    def enable_close_maintenance(self, track_id):
        """Bind near-range maintenance to a caller-selected confirmed role ID.

        Acquisition remains at MIN_RANGE_M. Near observations use the tolerant
        maintenance rules (round 55) and the same association gates as far
        ones; they never create or rebind an identity. The body-contact
        exemption is separate (contact_supports), recomputed from each scan.
        """
        target = self._target
        if (type(track_id) is not int or not isinstance(target, dict)
                or target['track_id'] != track_id or target['confirmed'] is not True
                or target['tracking_only_boundary_gap'] is not False):
            return False
        self._close_maintenance_track_id = track_id
        return True

    def _forget(self, reason):
        self.reason, self._target = reason, None
        self._first_at = self._first_received = None
        self._close_maintenance_track_id = None
        return None

    def update(self, scan, now, *, acquire=True, heading_rad=None):
        """Use only current support; ``acquire=False`` cannot create a new ID.

        ``heading_rad`` (round 64, open course only): the compass heading of
        this scan. Association then compares with the previous observation
        turned by the heading change since it (rotated_observation); without
        it the behaviour is unchanged.
        """
        if type(acquire) is not bool:
            raise ValueError('acquire must be a bool')
        if heading_rad is not None and not _number(heading_rad):
            raise ValueError('heading_rad must be a number')
        self.held = None
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
        confirmed_old = old is not None and old['confirmed']
        allow_boundary_gap = confirmed_old or (old is not None and self._unconfirmed_boundary_gap)
        bearing_range = (self._maintenance_bearing_rad
                         if confirmed_old and self._maintenance_bearing_rad is not None
                         else self._acquisition_bearing_rad)
        # A bound confirmed role is maintained down to the valid lidar range
        # (trials 52/54: real supports at .28-.38 m). Maintenance of any
        # confirmed tolerant identity uses the bounded edge allowance.
        close_bound = confirmed_old and old['track_id'] == self._close_maintenance_track_id
        maintenance = self._tolerant and confirmed_old
        candidates = _candidates(ranges, allow_boundary_gap=allow_boundary_gap,
                                 bearing_range_rad=bearing_range, tolerant=self._tolerant,
                                 maintenance=maintenance,
                                 min_range_m=CLOSE_MAINTENANCE_MIN_RANGE_M if close_bound else MIN_RANGE_M,
                                 min_points=None if maintenance else self._acquisition_min_points,
                                 max_range_m=self._acquisition_max_range_m)
        candidates = [{**candidate, 'close_maintenance_observation':
                       any(ranges[i] < MIN_RANGE_M for i in candidate['support_bins'])}
                      for candidate in candidates]
        # Tolerant: the gate widens with the time since the identity was last
        # observed beyond one nominal scan period (held frames in between).
        slack = (max(0., (published-old['source_at_ms'])/1000-.1)
                 if self._tolerant and old is not None else 0.)
        reference = old
        if heading_rad is not None and old is not None and _number(old.get('compass_heading_rad')):
            reference = rotated_observation(old, heading_rad-old['compass_heading_rad'])
        if self._select_associated and allow_boundary_gap:
            # Confirmed role trackers must not switch to an unrelated object
            # merely because it is the only component left in their sector.
            candidates = [candidate for candidate in candidates if _associated(candidate, reference, slack)]
        self.held = None
        if len(candidates) != 1:
            reason = 'compact_target_ambiguous' if candidates else 'compact_target_missing'
            if (self._tolerant and old is not None
                    and (published-old['source_at_ms'])/1000 < MAX_AGE_S
                    and received-old['source_received_at'] < MAX_AGE_S):
                # Keep the identity (not its observation) for the lease; this
                # frame returns None and never counts toward confirmation.
                self.reason, self.held = reason+'_held', copy.deepcopy(old)
                return None
            return self._forget(reason)
        current = candidates[0]
        # These bounded association gates are not speed, odometry or static
        # object certification. A jump creates a new, unconfirmed identity.
        close_observation = current.get('close_maintenance_observation') is True
        associated = _associated(current, reference, slack)
        boundary_gap = current['tracking_only_boundary_gap']
        if boundary_gap and not associated:
            return self._forget('compact_target_boundary_gap_unassociated')
        if boundary_gap and ((published-old['last_full_isolation_at_ms'])/1000 >= MAX_AGE_S
                             or now-old['last_full_isolation_received_at'] >= MAX_AGE_S):
            return self._forget('compact_target_boundary_gap_expired')
        if not associated and not acquire:
            return self._forget('compact_target_acquisition_disabled')
        if (not associated
                and not self._acquisition_bearing_rad[0] <= current['bearing_left_rad'] <= self._acquisition_bearing_rad[1]):
            return self._forget('compact_target_outside_acquisition_sector')
        if associated:
            track_id = old['track_id']
            count = old['confirmation_count'] + (0 if boundary_gap else 1)
        else:
            track_id, count = self._next_id, 1
            self._next_id += 1
            self._first_at, self._first_received = published, received
            self._close_maintenance_track_id = None
        confirmed = (count >= 3 and published-self._first_at >= round(MIN_CONFIRM_S*1000)
                     and received-self._first_received+1e-9 >= MIN_CONFIRM_S)
        self.reason = ('compact_target_tracked_with_boundary_gap' if boundary_gap else
                       'compact_target_confirmed' if confirmed else 'compact_target_unconfirmed')
        self._target = {**current, 'source_seq': seq, 'source_at_ms': published,
                        'source_received_at': received, 'confirmed': confirmed,
                        'close_maintenance_observation': close_observation,
                        'confirmation_count': count, 'semantic_class': 'unknown',
                        'kind': 'lidar_compact_object', 'track_id': track_id,
                        'last_full_isolation_seq': old['last_full_isolation_seq'] if boundary_gap else seq,
                        'last_full_isolation_at_ms': old['last_full_isolation_at_ms'] if boundary_gap else published,
                        'last_full_isolation_received_at': old['last_full_isolation_received_at'] if boundary_gap else received,
                        'candidate_only': True, 'physical_identity_verified': False}
        if heading_rad is not None:
            self._target['compass_heading_rad'] = heading_rad
        return copy.deepcopy(self._target)
