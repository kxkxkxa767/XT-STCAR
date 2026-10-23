"""Bounded left turn and opt-in observed corridor / second-object continuation.

The compact object's semantic identity is unknown. This module observes no
global pose, speed, physical steering angle, passed cone count or complete lap.
The service retains fresh body-clearance checks, ownership, heartbeat and ACKs.
"""
from bisect import bisect_left
import copy
import math

from compact_target import (CompactTargetTracker, _associated, _candidates, valid_ranges, contact_supports, MAX_DIAMETER_M,
                            MIN_RANGE_M, TOLERANT_MIN_POINTS, TOLERANT_MAINTENANCE_MIN_POINTS,
                            rotated_observation, EXTENDED_MAX_RANGE_M)
from autonomy_live import (FRONT_BODY_EXTENT_M, REAR_BODY_EXTENT_M, SIDE_BODY_EXTENT_M,
                           MANEUVER_BODY_CLEARANCE_M, MIN_FORWARD_PWM, CorridorSteering, corridor_walls,
                           side_wall_fit)
from steer_calibration import NOMINAL_RIGHT_LIMIT_PWM, STEER_PLAY_PWM
from turn_motion import (TurnMotion, NEUTRAL, TRIAL_MOTOR, SERVO_MIN, SERVO_MAX, SCAN_AGE_S, MAX_DRIVE_S,
                         PWM_STEP, PWM_INTERVAL_S, _FRAME, _freeze, _number, _valid_candidate,
                         _valid_wall, _wrap, _surface_supports, _same_surface)

ORBIT_ENTRY_MAX_S = 3.0
COAST_MAX_S = 5.0
PRESTEER_MAX_S = 8.0
DEFAULT_INITIAL_PWM = 1670
INITIAL_PWM_MIN = 1670
PRESTEER_PWM_STEP = 20
LEFT_RELEASE_PWM_STEP = 20
ORBIT_BEARING_RAD = math.pi/2
ORBIT_BEARING_GAIN = 100.0
ORBIT_RANGE_GAIN = 70.0
LEFT_TRIAL_GAIN = 180.0
MAX_TREND_RELEASE_FRACTION = .20
MAX_INNER_RELEASE_FRACTION = .20
MAX_EXIT_WIDTH_CHANGE_FRACTION = .10
# Nominal right limit the wheels reach (steer_calibration: -1.2 /m measured,
# 2026-10-16); the console maps nominal commands to physical servo PWM.
RIGHT_EXIT_MIN_PWM = NOMINAL_RIGHT_LIMIT_PWM
RIGHT_EXIT_MAX_S = 3.0
# Explicit continue_route only: once the current first-target support is back
# at the measured body front, every further left step only extends the orbit.
# Release toward neutral (never past it) faster than the generic 20 PWM step.
# Rightward steps and every other phase keep their existing limits.
ROUTE_EXIT_RELEASE_PWM_STEP = 40
# Consecutive strictly fresh full-support frames for the body-front release.
ROUTE_EXIT_PREPARE_FRAMES = 2
# Explicit continue_route only: trial40 had two fresh permitting endpoint
# bearings (1947/1948) but lost the native opening before the third, so the
# entry cap coasted the left turn. Same frame count as the exit prepare.
ROUTE_ENTRY_RELEASE_FRAMES = 2
# Explicit continue_route only. After the first-target pass every route
# steering change (exit alignment, corridor follow, second target) uses the
# route exit release step already exercised on the car in trial43. Trial43
# needed a left swing around the second support within about one second;
# trial45 needed a quick left away from a right wall .22 m from the body.
# Center crossing and the right-after-center-ACK rule are unchanged.
ROUTE_POST_PASS_PWM_STEP = 40
# Explicit continue_route only: after the first-target handover, LEFT increases
# also use 40 PWM/100 ms. Trial49 (path rebuilt from saved scans): the left turn
# had released to 1636 just before handover, the orbit asked 1707 but the 10
# PWM step reached only 1676 in .5 s; the U-turn radius grew from ~.95 to
# ~1.1 m and put the body .08 m from the outer wall. Releases are unchanged.
ROUTE_ORBIT_LEFT_PWM_STEP = ROUTE_POST_PASS_PWM_STEP
# Explicit continue_route only. After the first-target pass the motor uses the
# operator's manual forward PWM instead of 1560: trial43 reached ~1.8 m/s at
# the second target, and the traffic-light section must be driven slowly
# because the countdown starts when the body crosses the infrared sensor.
ROUTE_SLOW_MOTOR = MIN_FORWARD_PWM
# Round 100 (user: 刚开始直道那一段你可以试试1600的pwm): the open start lane
# launches at 1600 (start-zone entry, before any handover only; the server
# allows it nowhere else). Run 99: 1560 took 1.1 s to reach 1.0 m/s.
START_LAUNCH_MOTOR = 1600
DRIVE_MOTORS = (TRIAL_MOTOR, ROUTE_SLOW_MOTOR, START_LAUNCH_MOTOR)
POST_PASS_STAGES = ('right_align', 'corridor_follow', 'second_orbit', 'light_turn', 'light_lane',
                    'open_transfer', 'second_loop', 'light_entry')
# Route mode alone may extend the cumulative drive budget (user permitted a
# measured relaxation): the light-lane turn follows the two targets. The
# total deadline, coast and every stage budget still apply.
# Round 58 (Claude): the route now continues through the light lane to the
# finish zone (trial57 reached the light turn after ~7.7 s of drive).
# Round 64 (Claude): with 3.2 m side straights the run starts in the start
# zone and covers the whole lap (~22 m; simulation 22-24 s at the 1550
# speed model of 1.0 m/s, and 1550 measured .6-1.1 m/s), so 24 s cannot
# finish. The budget is a ceiling; total deadline, coast and holds remain.
ROUTE_MAX_DRIVE_S = 40.0
# Exit alignment / corridor pursuit: lookahead about one corridor width plus a
# heading-rate damping term (trial46 oscillated with a 1 m lookahead).
ROUTE_ALIGN_LOOKAHEAD_MAX_M = 2.0
ROUTE_ALIGN_DAMPING_S = .30
ROUTE_ALIGN_HEADING_RAD = math.radians(8)
# Light-lane turn after the second target: follow the deepest free direction
# for the measured body plus clearance, then hold once a long free lane is
# straight ahead and the side clearances balance (camera faces the lane axis,
# where the traffic light hangs over the center). No course constants.
LIGHT_STAGE_MAX_S = 4.0
LIGHT_FREE_HORIZON_M = 3.0
LIGHT_ALIGNED_FREE_M = 2.5
LIGHT_ALIGNED_HEADING_RAD = math.radians(6)
LIGHT_CENTER_TOLERANCE_M = .20
LIGHT_HEADINGS_DEG = tuple(range(-100, 101, 2))
# Unknown is never free: a front null run of <= 5 bins between two known
# returns blocks at the nearer one (the clearance bridging rule); any longer
# or unbracketed run blocks at this short range so no heading or alignment
# is chosen through it.
LIGHT_UNKNOWN_BLOCK_M = .60
# Trial41/47: the scan_incomplete frame that triggers the orbit wait can hide
# the first support's edge, and the tracker may then re-acquire the same
# support under a new ID. The wait may start from the last FULL isolation of
# the orbit support if it is fresh, holds through missing frames, and re-binds
# a new identity only if its first sighting lies within the tracker's own
# association gate of that last full sighting and it then confirms.
QUALITY_WAIT_REBIND_M = .30
QUALITY_WAIT_REBIND_WINDOW_S = .60
# Trial50: the coasting wait lost the first support at ~.9 m; the tracker then
# found the same support as new IDs (.75 m, then .37 m once stopped) that the
# association gate above never matched, and the orbit budget locked a car that
# was well placed. In a first-target wait, the nearest compact candidate on the
# left, confirmed and fully isolated within this range, is the orbited support.
ORBIT_REBIND_MAX_RANGE_M = 1.2
# Explicit route only: steer away from close known returns before the service
# body gate (8 cm, unchanged) is reached. Within this gap a return on the right
# (or a head-on return with the left front more open) demands at least
# ROUTE_AVOID_LEFT_PWM; one on the left caps steering at center. Before the
# first-target pass the inside (left) is left to the turn/orbit laws, which
# already keep their own inside clearance. Steps use the route step.
ROUTE_AVOID_GAP_M = .15
ROUTE_AVOID_LEFT_PWM = NEUTRAL+150
ROUTE_AVOID_PWM_STEP = 40
# Route waits (motor at neutral, car may still roll): the held left is
# released only while it exceeds the closest inside return's clearance-bearing
# demand by more than this margin (trial50: 1710 held while the support closed
# from .9 to .37 m on the inside). A static scene never changes the hold.
ROUTE_WAIT_INSIDE_MARGIN_PWM = 60
# Explicit route only. Trial51: the first support never confirmed during the
# left turn, the turn passed its 90 deg exit axis by 8 deg and the overshoot
# coast centered the steering at ~1.5 m/s; the car rolled straight into the
# outer board of the next lane (same spot as trial49). Instead the adopted left
# is held at drive power while the first-target handover is checked every
# frame; only after this long does the held-left coast (no centering) follow.
ROUTE_LEFT_OVERSHOOT_HOLD_S = 1.0
# Explicit route, after the first-target handover (round 53): check the stage's
# steering against the predicted body sweep along its arc, independent of any
# object recognition (trial52: the second support was not recognized and the
# car drove straight into it). Curvature per PWM is the value measured from
# the rebuilt paths of trials 49/50/52 (scripts/trial-map.py): ~1.0-1.1 1/m at
# 1655-1665, ~1.3 1/m at 1710 slow. If the stage target's arc meets a known
# return (body side .14 m, +4 cm ahead of the front face) within the
# lookahead, the nearest clear candidate is used; corridor and second-target
# stages prefer passing an obstacle on its left (task rule: the second cone is
# passed clockwise). Right of center only where the route already authorizes it.
ROUTE_PWM_PER_CURVATURE = 160.
ROUTE_PATH_LOOKAHEAD_M = .70
ROUTE_PATH_MARGIN_M = .04
ROUTE_PATH_RANGE_M = 2.
ROUTE_PATH_LEFT_CANDIDATES = tuple(range(NEUTRAL, SERVO_MAX, 20))+(SERVO_MAX,)
ROUTE_PATH_RIGHT_CANDIDATES = (RIGHT_EXIT_MIN_PWM, 1350, 1390, 1430, 1470)
_DIRECTION_COS = tuple(math.cos(math.radians(i)) for i in range(360))
_DIRECTION_SIN = tuple(math.sin(math.radians(i)) for i in range(360))
LIGHT_BRIDGE_MAX_BINS = 5
# Round 63 (Claude): _free_space tables. The same expressions as the former
# per-call math, evaluated once (bitwise identical results); trials 60-62
# light-turn ticks took .087-.095 s on the car, ~60% of it in _free_space,
# against the .12 s control-interval limit.
_FREE_BEARING_DEG = tuple(math.degrees(_wrap(-math.radians(i))) for i in range(360))
_FREE_POINT_COS = tuple(math.cos(math.radians(b)) for b in _FREE_BEARING_DEG)
_FREE_POINT_SIN = tuple(math.sin(math.radians(b)) for b in _FREE_BEARING_DEG)
_FREE_COS = tuple(tuple(math.cos(math.radians(h-b)) for h in LIGHT_HEADINGS_DEG) for b in _FREE_BEARING_DEG)
_FREE_SIN = tuple(tuple(math.sin(math.radians(h-b)) for h in LIGHT_HEADINGS_DEG) for b in _FREE_BEARING_DEG)
# Round 58 (Claude): light lane to the finish, no vision yet (user: run the
# whole route first). Neutral coasting only (no reverse braking; nothing below
# 1550 starts the car). Speed is the closing rate of the board straight ahead
# (median of 3 scan-to-scan rates); a 1560 restart below
# LANE_SPEED_RESTART_MPS (rules: stopping > 10 s outside light/crosswalk fails).
# Finish (rules: stop inside the 80 cm zone at the lane end, board contact
# +10 s): cut power once the board ahead is within body front + gap + the
# neutral coasting distance v^2/(2a) + v*reaction. Unknown speed uses the
# fastest measured 1550 speed.
# Round 63 (Claude). User: "后续直道的部分可以参考发车区直道的，快一点，但要
# 保证能不撞板" / "红绿灯后的直道，到尽头就停下". The straight is driven at
# 1550 (~.9-1.3 m/s; round 58 regulated the whole lane to .45-.70). Once the
# board ahead is within the current speed's cut plus LANE_SLOW_TRAVEL_M, a
# latched slow zone keeps the round-58 band (coast above LANE_SLOW_HIGH_MPS,
# 1550 below LANE_SLOW_LOW_MPS), so the final cut happens from a low speed and
# the uncertain decel acts on a short coast only. A smooth coasting profile
# was tried first: with only 1550/neutral and a median-of-3 speed, a 1550 push
# overshot to ~1.2 m/s and the cut fired from there (closed-loop test). The
# cut uses LANE_STOP_DECEL_MPS2, below the ~.5 m/s^2 rolling decel measured
# from rebuilt paths (trials 49-51): at .6 m/s the coast is .51 m at .35 vs
# .36 m at .5 m/s^2. A speed missing for one frame (front range jump) uses the
# last measured one (+.1 m/s) for .5 s before the 1.3 m/s fallback. Trial62
# entered the lane end at 1.14-1.27 m/s without any cut (still in the light
# turn) and touched the board after the control-gap lock.
LANE_SPEED_RESTART_MPS = .15
LANE_SPEED_UNKNOWN_MPS = 1.3
# Above any speed this car reaches in the route (straight 1560 on tiles keeps
# rising to ~2-2.4 m/s, 1580 ~2.7, 2026-10-16); a larger closing rate is a
# different return on the ray (turning car), not speed.
LANE_MAX_CLOSING_MPS = 3.5
LANE_COAST_DECEL_MPS2 = .5
LANE_STOP_DECEL_MPS2 = .5
LANE_SPEED_HOLD_S = .5
LANE_SPEED_HOLD_MARGIN_MPS = .1
# Control/perception reaction plus the ESC lag: the car keeps its last motor
# command ~.4-.5 s after any new one, also after a cut (2026-10-16 tiles; the
# bridge already sends 1500 every 20 ms then).
ESC_LAG_S = .45
LANE_REACTION_BASE_S = .2
LANE_REACTION_S = LANE_REACTION_BASE_S+ESC_LAG_S  # where the last push time is not tracked
# Speed still rising while a cut waits out the ESC lag (1560 straight on tiles:
# about 1 m/s^2 at 1-1.5 m/s).
LANE_POWER_ACCEL_MPS2 = 1.
LANE_LAUNCH_S = .4
LANE_FINISH_GAP_M = .15
# Round 64 (Claude): finish. A cut from cruise speed cannot land inside the
# .8 m zone over the plausible coast decels (simulation: a scan-seam jump read
# 1.15 m/s at 1.0 and the car stopped 1.2 m short; the round-63 closed-loop
# tests failed). Now: (1) never power within the distance the weakest coast
# (LANE_STOP_DECEL_MIN_MPS2) needs from the estimated speed plus a margin
# (more while powered: the estimate lags a push); (2) in the slow zone hold
# LANE_SLOW_LOW..HIGH with neutral/1550 (no push within LANE_PUSH_CLEAR_M of
# the cut); (3) end only at <= LANE_SLOW_HIGH where a coast at the nominal
# decel (the measured ~.5 m/s^2) stops LANE_FINISH_TARGET_GAP_M from the board,
# or when stopped with the front inside the zone. Parameters chosen on the
# round-63 closed-loop model (decel .25-1.0, start 3.5-6 m, cruise .9/1.27).
# Round 101 replaced (2) and (3), see LANE_FINISH_BODY_GAP_M.
# Round 92 (user: 为什么最后那么点距离，你频繁的停车再加pwm): light-lane coasts
# on the car stopped after v0^2/(2d) = .46 (run 91: 1.1 m/s, 1.31 m), ~.45
# (run 80) and ~.51 m/s^2 (run 88); at .25 the cut came ~1 m early and the car
# stopped short of the zone, then crept on in pulses (run 91: 6.5 s). .35
# keeps ~22 % below the weakest measured coast.
# Round 101: the guard now models the ESC lag's travel apart (see
# LANE_FINISH_BODY_GAP_M): .45 keeps ~25 % below the .57-.61 coasts of runs 98-100.
LANE_STOP_DECEL_MIN_MPS2 = .45
# Round 82: a coast decel measured over at least this span and speed drop
# replaces the weakest one, clamped to [LANE_STOP_DECEL_MIN_MPS2,
# LANE_COAST_DECEL_CAP_MPS2]. Round 86: the cap was the nominal .5 and the
# measured car with 15 % stronger steering (user8) stopped .055 m from the
# end board on the body gate; at .375 (halfway) all sixteen measured-car
# finish cases (user8/doc8x4) stop in the zone, board >= .15 m.
LANE_COAST_MIN_SPAN_S = .4
LANE_COAST_MIN_DROP_MPS = .1
# Round 92: at most the weakest coast measured on the car (~.45, see above).
# Round 101: light-lane coasts on the car, runs 98-100, ~.55-.6 m/s^2 (1.5-1.7
# m/s at the cut, 2.0-2.5 m to the stop); the measured one now also aims the
# coast (below), so the cap only bounds a bad fit.
LANE_COAST_DECEL_CAP_MPS2 = .70
# Front in the zone (the stage's old end; the finish line lies ~.05 m beyond).
LANE_FINISH_ZONE_GAP_M = .75
# Round 101 (Claude; user: 最后那段直道想办法让他一口气跑完不要停顿，这个太耗时间了).
# Runs 98-100 entered the lane at 1.5-1.7 m/s with 3.1-3.4 m to go, coasted at
# once on the weakest-coast guard (.35 m/s^2 plus .3 m/s of margins), stopped
# 1.0-1.3 m from the board and crept on in 1560 kicks (run 100: three, ~3 s);
# the end at a front gap of .74-.76 m left the rear wheels outside the 80 cm
# zone (rules 2.4.2: the time ends when the front wheels cross the finish
# line; at the stop all four wheels must be in the zone, else +5 s; board
# contact +10 s). Those coasts, fitted with the ESC lag apart, ran at .57-.61
# m/s^2 (speed residual ~.03 m/s); the older .45-.51 (runs 80/88/91) were
# v0^2/(2 d) with the lag's travel inside d. The cut came ~.2-.3 s before the
# coast, run 100's restart kick moved the car ~.3 s after it was sent.
# Now the coast is aimed at a stop with the whole car in the zone, and a car
# that would stop short is pushed again (1550) while it still rolls:
# - aim (_lane_stop): the front gap less the travel over LANE_AIM_REACTION_S
#   (while powered), the outputs still in the ESC pipeline (_lane_pipeline:
#   LANE_ESC_LAG_NOMINAL_S both ways, 1550 at LANE_PUSH_ACCEL_MPS2) and a coast
#   at the lane's measured decel (_lane_coast_observe), LANE_STOP_DECEL_MPS2
#   before one is measured; the median-rate speed is brought to now
#   (_lane_speed_now: it is half its window old);
# - guard (_lane_worst_gap): the worse of a LANE_ESC_LAG_MIN_S and an
#   ESC_LAG_S lag, each LANE_ESC_SKEW_S early on and late off, the speed plus
#   LANE_SPEED_GUARD_MARGIN_MPS, power at LANE_PUSH_ACCEL_MAX_MPS2 (inside the
#   zone LANE_PUSH_LATE_ACCEL_MAX_MPS2, a 1560 kick LANE_KICK_ACCEL_MAX_MPS2) up
#   to LANE_PUSH_MAX_MPS, then the measured decel
#   less two standard errors (at least LANE_COAST_GUARD_RATIO of it;
#   LANE_STOP_DECEL_MIN_MPS2 before one is measured); never within
#   LANE_FINISH_MIN_GAP_M of the board. Roughly two standard deviations of
#   each, not all worst cases at once: compounded they ruled out any push
#   inside ~1.5 m at .8 m/s (closed-loop simulation with a pure ESC delay)
#   while the aim already keeps .25 m from the board. A push starts only when
#   its first scan (LANE_PUSH_MIN_S) passes the guard and the car still rolls.
LANE_FINISH_BODY_GAP_M = .40  # zone .80 less the car's .38 m, .02 inside
LANE_FINISH_MIN_GAP_M = .05
LANE_FINISH_TARGET_GAP_M = .25
# Push again only when the aimed coast stops this far short of the target.
LANE_TOPUP_BAND_M = .10
# 1550 moves a car that still rolls at this speed when it acts (run 93: .3 ->
# .62 m/s in 1 s, ~.32 m/s^2), not a car at rest (~.7 s, 2026-10-16); .25
# (simulation: .3 left 13 % more stops with the rear outside the zone).
LANE_PUSH_ROLLING_MPS = .25
LANE_PUSH_ACCEL_MPS2 = .35
LANE_PUSH_ACCEL_MAX_MPS2 = .5
# Round 101: inside the zone (past the finish line) a push only brings the rear
# wheels in, while the board costs +10 s: guarded at this (course-sim's nominal
# car, no ESC lag and 1550 up to ~1.6 m/s^2 at .45 m/s, met the board after a
# late .5 m/s^2-guarded push).
LANE_PUSH_LATE_ACCEL_MAX_MPS2 = 1.
# A 1560 restart kick from rest reached ~.47 m/s within ~.3 s of power (run 91).
LANE_KICK_ACCEL_MAX_MPS2 = 1.6
# 1550 cruises at ~1.5 m/s on the track (runs 98-100 light entry), ~1.1 on tiles.
LANE_PUSH_MAX_MPS = 1.7
LANE_PUSH_MIN_S = .1  # one scan
LANE_ESC_LAG_NOMINAL_S = .3
# The guard takes the worse of a LANE_ESC_LAG_MIN_S and an ESC_LAG_S lag, each
# reaching the wheels LANE_ESC_SKEW_S early and leaving them that much late.
LANE_ESC_LAG_MIN_S = .2
LANE_ESC_SKEW_S = .05
LANE_AIM_REACTION_S = .1
# The coasting speed's residual on the car was ~.03 m/s (runs 98-100).
LANE_SPEED_GUARD_MARGIN_MPS = .05
LANE_COAST_GUARD_RATIO = .9
LANE_COAST_TRACK_POINTS = 12
# A fit is used once its slope's standard error is at most this; the guard and
# the rolling end then take two standard errors (at least LANE_COAST_GUARD_RATIO).
LANE_COAST_DECEL_SE_MAX_MPS2 = .08
# Restart from rest: a LANE_KICK_FRAMES 1560 kick moved the car .08-.36 m
# (runs 98-100); none when that could reach LANE_FINISH_MIN_GAP_M.
LANE_KICK_TRAVEL_M = .45
# No restart kick until a push has left the ESC pipeline and the speed window
# (the estimate lags a kick: a moving car still read 0 and got a second kick).
LANE_KICK_REST_S = 1.
# A measured coast decel is kept within these (the prior LANE_STOP_DECEL_MIN_MPS2
# bounds the guard only before one is measured).
LANE_COAST_DECEL_FLOOR_MPS2 = .2
# Near the end power came in pulses (LANE_PULSE_FRAMES = 2 powered frames, then
# LANE_REST_FRAMES neutral): the closing-speed estimate lags a push, and a 1560
# restart held until it read .15 m/s had reached .93 m/s (simulation, 8x6
# course: then cut late and hit the end board). Round 94: every 1560 restart
# is a LANE_KICK_FRAMES kick and a rolling 1550 push is not chopped (see
# _light_lane_update).
# Rest at least the ESC lag between pushes, so the speed a push produces is
# measured before the next one (2026-10-16 measured car: a stopped car got
# 1560 for 1 s before it moved, then ran into the end board at 1.2 m/s); round
# 101: LANE_KICK_REST_S (was LANE_REST_FRAMES = 5 scans).
# A restart kick at 1560 anywhere in the lane is a pulse too.
LANE_KICK_FRAMES = 3
# Round 105 (Claude; car runs 103/104 stopped .404/.459 m from the end board:
# the front in the zone, the rear out (rules 2.4.2: +5 s), inside the kick's
# LANE_FINISH_MIN_GAP_M+LANE_KICK_TRAVEL_M). A car stopped there gets a
# LANE_CREEP_FRAMES 1560 creep instead, at most LANE_CREEP_MAX of them (the time
# has already ended at the finish line). Its travel bound scales the kick's
# (.45 m for 3 frames; the car's 3-frame kicks moved .08-.34 m, runs 98-103)
# by the squared speed reached: LANE_SPEED_GUARD_MARGIN_MPS plus
# LANE_KICK_ACCEL_MAX_MPS2 over the frames and the ESC skew on both ends,
# (.53/.69)^2 x .45 = .266 m; from the zone's rear edge it stops >= .13 m short.
LANE_CREEP_FRAMES = 2
LANE_CREEP_TRAVEL_M = .27
LANE_CREEP_MAX = 3

LANE_SPEED_CUT_MARGIN_MPS = .15
# While powered the speed rises faster than the ~.3 s estimate follows.
LANE_SPEED_POWER_MARGIN_MPS = .15
# Speed: closing rate over ~.2 s baselines (median), so one scan-seam jump of
# the ray ahead (~.1 m at 1 m/s) moves it by ~.5 m/s instead of ~1 m/s and
# the median drops it; longer baselines lagged too much while coasting.
LANE_SPEED_BASELINE = 2
LANE_SPEED_HISTORY = 7
LANE_FRONT_HALF_BINS = 3
LANE_ALIGNED_HEADING_RAD = math.radians(8)
# Round 63 (Claude): the light turn hands over to the light lane as soon as
# a lane is measured within this heading, once the turn has run at least
# LIGHT_TURN_MIN_S (the old corridor can still be near straight ahead when
# the turn starts). Trials 60-62: the light lane (~1.05 m wide) was measured
# at 8-18 deg for seconds while the free-space law weaved 1408-1571 and the
# 8 deg / .3 s alignment never matured; the car reached the lane end still in
# the light turn.
LIGHT_LANE_ENTRY_HEADING_RAD = math.radians(20)
LIGHT_TURN_MIN_S = 1.0
# Lateral gap from the lidar origin to the support's current left edge at the
# bypass point: keeps the support above the detector's minimum range while it
# is abeam (trial43 lost the second support below .35 m) and leaves .26 m to
# the measured body side, above the existing .08 m maneuver clearance.
SECOND_BYPASS_SUPPORT_GAP_M = MIN_RANGE_M+.05
# Trial48: steering by bearing (180 PWM/rad) asked only ~1590 with the second
# support .9 m ahead, while reaching the bypass line there needs a ~1 m path
# radius. Steer by pursuit curvature kappa = 2y/(x^2+y^2) instead, with the
# uncalibrated PWM-per-curvature seen in the first-target orbits (~1705 at
# ~1 m radius). Not a measured wheel angle.
SECOND_CURVATURE_GAIN_PWM_M = 205.
# Round 55 (Claude): stages whose task rule is to keep the obstacle on the
# car's right (second support clockwise). An obstacle correction there may add
# left from a neutral or slightly right stage target (trial52 scans 436/438:
# corridor target ~1494-1500 straight at the unconfirmed second support; the
# round-53 check chose 1580/1700, the first round-55 bounds kept 1494/1500).
# It never invents right. Exit/light turns keep the strict bounds.
TASK_LEFT_BYPASS_STAGES = ('corridor_follow', 'second_orbit', 'open_transfer', 'second_loop')
# Second-role recovery after an identity loss: the last full observation is a
# car-frame prior, and the car keeps moving (~1 m/s at 1550, up to ~100 deg/s
# yaw). Its association gate widens at the tolerant tracker rates with the
# time since that observation, capped here (gate <= 1.2 m / 80 deg). With a
# fixed gate a support re-seen after >~.3 m of travel was never recovered.
SECOND_RECOVERY_MAX_SLACK_S = .6
# Round 79 (Claude): the prior also moves with the car. Round 78: the car
# rolled ~1.2-1.5 m and turned ~57 deg in the post-pass pause, beyond that
# rotation-only gate. When the first support (still tracked) was current in
# the prior's own scan and is current now, the prior is its present position
# plus the inter-support vector turned by the compass change, and the gate
# widens by at most this slack (.6 m / 40 deg).
SECOND_LANDMARK_SLACK_S = .2
# Round 57 (Claude): second-support bypass in the current lane frame. Trial56:
# the confirmed second support stayed 3.5-7 deg from straight ahead from 2.3 m
# to .87 m. The support-relative pursuit point sits at the support's own
# distance, so it asked only ~1540-1620 while far and 1720 at .65 m: the car
# reached the support turned hard left, pointing at the far board, and could
# not swing back in time. With a measured lane, steer toward the line
# SECOND_BYPASS_SUPPORT_GAP_M left of the support's left edge (limited by the
# lane's left wall less the body clearance), reached within at most this
# lookahead along the lane: the offset is taken early and the car passes
# parallel to the lane. Same corridor gain as exit/corridor following. The
# command is the larger of this and the support pursuit (trial48: a near
# support whose offset is not yet taken still needs the pursuit's strong
# left). No measured lane (or a lane at more than SECOND_LANE_MAX_HEADING_RAD):
# the support-relative pursuit alone, unchanged.
SECOND_LANE_LOOKAHEAD_M = 1.0
SECOND_LANE_MAX_HEADING_RAD = math.radians(45)
# While bypassing the second support, do not turn the car more than this
# left of the measured lane: a cone contact continues (user rule), a board
# contact ends the run. Trial56 re-confirmed the support only at 1.43 m and
# the pursuit then drove 1720 until the car faced the far board. Synthetic
# lane model with .3 s steering lag (work/claude-turn57-*/bypass_sim*):
# start 1.8 m -> max heading 55 -> 44 deg, 1.43 m -> 120 -> 71 deg; a start
# at 2.3 m is unchanged. Gain chosen so that a car aligned with the lane
# (trial48 scan509, lane -3 deg, support .9 m ahead) keeps the >= 1680 left
# the pursuit needs (limit 1712).
SECOND_BYPASS_MAX_HEADING_RAD = math.radians(30)
SECOND_HEADING_HOLD_GAIN = 450.
# Round 59 (Claude): the heading limit acts on the heading predicted one
# steering lag ahead (lane-heading rate over consecutive fresh frames).
# Trial58: 445 at 12 deg left with 72 deg/s yaw was still allowed 1646; the
# car reached 35 deg left of the lane (left board) before the return acted.
# trial-map: commands act on the trajectory ~.2-.3 s later.
SECOND_STEER_LAG_S = .3
# Round 59 (Claude): the second role (corridor proof, its tracker and identity
# recovery) acquires a 3-return support. At 2.5-2.9 m a ~.17 m support spans
# ~3.5 one-degree bins, so frames alternate 3/4 returns and an unconfirmed
# identity kept restarting (trial58: confirmed only at 1.79 m, the near
# pursuit then turned the car into the left board). Saved trials 47-58, ahead
# sector: first confirmation 52 1.72->2.28 m, 55 1.75->2.01, 56 1.20->2.04,
# 57 2.20->2.40, 58 1.68->2.46; no extra identities on boards. Size,
# isolation, association, corridor uniqueness and 3-frame confirmation stay.
SECOND_ACQUISITION_MIN_POINTS = 3
SECOND_MAX_HEADING_RATE_RAD_S = math.radians(150)
# Round 60 (Claude): after the first pass, the exit alignment and corridor
# following steer toward a line this far LEFT of the measured lane center
# instead of the center itself (task rule: the second support is passed
# clockwise, i.e. on the car's left side). Trial59: the car followed the
# center (.10 m left of it), the second support was confirmed at ~1.4-1.8 m
# about on the center line, and the remaining .30 m bypass offset at 1.27 m/s
# took 1611-1639 and ~40 deg of heading toward the separator board, whose end
# is ~.9 m left of that support. The line is the bypass offset for a support
# whose left edge is on the center; it is bounded by the measured width (body
# side + maneuver clearance + SECOND_APPROACH_WALL_MARGIN_M to the left wall).
# The second-support bypass law itself (lane frame + pursuit) is unchanged.
SECOND_APPROACH_LEFT_M = SECOND_BYPASS_SUPPORT_GAP_M
SECOND_APPROACH_WALL_MARGIN_M = .15
# Round 60 (Claude): stage bound of the second orbit (handover -> matured
# pass), separate from the first orbit's ORBIT_ENTRY_MAX_S. The second
# support is acquired up to MAX_RANGE_M (3 m) ahead and the pass needs it
# behind the rear plus .25 s: at the slowest measured 1550 speed (~.6 m/s)
# that is ~5.5 s. Trial57 used 2.79 s of the old 3.0 s from 2.49 m at
# ~.9 m/s; the round-59 3-return acquisition confirms earlier (up to ~2.5 m)
# and a timeout there drops back to corridor following with the support
# already beside the car. The cumulative route drive budget still applies.
SECOND_ORBIT_MAX_S = 6.0
# Releasing left toward center while the second return is being prepared
# (round 57): trial56 needed ~.25 s per adopted 40-PWM step, 1720 -> 1600 in
# .53 s, still not centered at the board .7 s after preparing. Release only;
# each step still waits for the previous command's adoption.
ROUTE_RETURN_RELEASE_PWM_STEP = 80
# Same-frame native wall-window pairing reuses the native corridor gates
# (corridor.rs): parallel 12 deg, width .65..2.5 m, shared support .35 m.
PAIR_MAX_ANGLE_RAD = math.radians(12)
PAIR_MIN_WIDTH_M, PAIR_MAX_WIDTH_M = .65, 2.5
PAIR_MIN_SHARED_SUPPORT_M = .35
# Several windows of the same two physical boards differ by a few degrees
# (trial39 scan480: 6 deg). The merge tolerance reuses the parallel gate; a
# different physical axis (corner/T) differs far more and stays ambiguous.
PAIR_MERGE_HEADING_RAD = PAIR_MAX_ANGLE_RAD
PAIR_MERGE_OFFSET_M = .15
# Round 64 (Claude, layout_3p2_route_redesign_20261012). The course is laid
# with 3.2 m side straights (user; construction document: two half dividers
# of about half the length): the first U-turn opening is ~L-3.2 m and the
# dividers do not overlap, so the native left junction (front wall <= 3 m,
# opening <= 2.25 m) and the native corridor (<= 2.5 m) do not exist where the
# narrow practice course had them. Every run starts in the start zone.
#
# Wall compass: the car heading relative to the start-lane axis, tracked from
# the native finite walls of every scan. The course is a board rectangle with
# dividers parallel to its sides (rules), so each long straight wall votes its
# direction modulo 90 deg around the predicted heading; the span-weighted
# median corrects the prediction. Trials 57-62 (work/layout-3p2-analysis-
# 20261012/compass-real.log): against every frame with one native corridor
# the error was p50 .7-1.0, p90 1.4-2.0, max 2.7 deg, through both U-turns.
COMPASS_MIN_SPAN_M = .5
COMPASS_MAX_FIT_M = .04
COMPASS_MIN_POINTS = 20
COMPASS_GATE_RAD = math.radians(30)
COMPASS_MIN_SUPPORT_M = 1.
COMPASS_MAX_RATE_RAD_S = 2.5
COMPASS_MAX_PREDICT_S = .3
COMPASS_RATE_WINDOW_S = (.05, .5)
COMPASS_MAX_STALE_S = 1.
# Start lane (stage start_lane): admitted in a measured straight lane with no
# native left opening ahead; follows the lane walls with the start-straight
# law (autonomy_live.CorridorSteering), else holds the compass heading.
LANE_START_MAX_HEADING_RAD = math.radians(15)
LANE_START_WIDTH_M = (.65, 2.2)
LANE_START_MIN_FRONT_M = 1.
LANE_START_SETTLE_S = .3
LANE_START_HOLD_PWM = 55
# Start-lane wall law (2026-10-18, replaces CorridorSteering in this stage).
# Rounds 64/66/67/68 weaved +-8 deg: the real wheels answer a near-centre
# command 1.5-3x the table and ~.35 s late, so the old law (about 200 nominal
# PWM per rad, slow 10 PWM ramps, no lead) is beyond its stability limit there
# (MATLAB margins: unstable at gain 3 / delay .35 / 1.5 m/s). This law corrects
# toward the lane by the sign and size of the measured error and leads it by
# the measured wall-heading trend, so it needs no model of the steering; its
# gain is small enough to stay stable over gain 1-3 and delay .15-.35 s.
LANE_WALL_LOOKAHEAD_M = 1.2  # offset weight: 1/LANE_WALL_LOOKAHEAD_M rad per m
LANE_WALL_LEAD_S = .45  # steering + scan delay the trend is extrapolated over
# Rounds 64/68/70 fit ~2x the 10-16 table's curvature near centre; the
# round-79 calibration (steer_calibration) maps that out, so the nominal gain
# is now the real one. 100 PWM/rad (cap 33) corrected drift at 1x in
# simulation (75 drifted into a board); with the track fit (G .8-1.4, lag
# .3-.5 s) its worst phase margin is 66 deg, 125 26 deg, 150 unstable
# (turn79_steer_calibration_20261019). An online estimate from ~3 s of start
# lane behind the play was too noisy and was not adopted (turn70 record).
LANE_WALL_GAIN_PWM_PER_RAD = 100.
LANE_WALL_CAP_PWM = 33
LANE_WALL_HISTORY = 3
LANE_WALL_GAP_S = .35  # older publications are not one trend
LANE_TRACK_SPEED_POINTS = 5
# A single side board standing in for the lane must run along the compass axis.
LANE_SIDE_WALL_HEADING_RAD = math.radians(5)
# Play crossed by the start-lane wall law's commands: the measured one (it
# was the then measured 50 in turn69_start_lane_law_20261018, chosen over real
# play 0-70 in simulation; the track fit gives 40).
LANE_WALL_PLAY_PWM = STEER_PLAY_PWM


class LaneWallSteering:
    """Start lane: nominal servo from the current lane walls, led by their trend.

    error = wall heading (rad, + = lane turns right of the nose) + offset/L
    (+ = lane centre to the right); the command turns toward the lane in
    proportion to error + LANE_WALL_LEAD_S * d(heading)/dt. The trend is the
    median of the last segments of consecutive fresh publications; a clock,
    width, offset or heading discontinuity restarts it. Output stays a nominal
    command (the console's steering calibration still applies).
    """
    def __init__(self):
        self.servo = NEUTRAL
        self.history = []
        self.last_scan = self.last_publication = None
        self.evidence = None

    def _reset(self):
        self.history, self.servo = [], NEUTRAL
        return self.servo

    def update(self, walls, scan_seq, now, motion_ready=True, scan_at_ms=None):
        if not motion_ready or walls is None or type(scan_seq) is not int:
            self.last_scan = scan_seq if type(scan_seq) is int else None
            return self._reset()
        if self.last_scan is not None and scan_seq < self.last_scan:
            self.last_scan = scan_seq
            return self._reset()
        if scan_seq == self.last_scan:
            return self.servo
        published = scan_at_ms/1000 if type(scan_at_ms) is int and scan_at_ms >= 0 else now
        if self.last_publication is not None and published <= self.last_publication:
            self.last_scan, self.last_publication = scan_seq, published
            return self._reset()
        self.last_scan, self.last_publication = scan_seq, published
        heading = math.radians(walls['heading_right_deg'])
        offset, width = walls['offset_right_m'], walls['width_m']
        previous = self.history[-1] if self.history else None
        if previous and (published-previous[0] > LANE_WALL_GAP_S or abs(width-previous[3]) > .1
                         or abs(offset-previous[2]) > .15 or abs(heading-previous[1]) > math.radians(8)):
            self.history = []
        self.history.append((published, heading, offset, width))
        del self.history[:-LANE_WALL_HISTORY]
        rates = [(b[1]-a[1])/(b[0]-a[0]) for a, b in zip(self.history, self.history[1:]) if b[0]-a[0] >= .05]
        rates.sort()
        trend = ((rates[(len(rates)-1)//2]+rates[len(rates)//2])/2
                 if len(self.history) == LANE_WALL_HISTORY and rates else 0.)
        error = heading+offset/LANE_WALL_LOOKAHEAD_M
        predicted = error+LANE_WALL_LEAD_S*trend
        correction = max(-LANE_WALL_CAP_PWM, min(LANE_WALL_CAP_PWM, round(LANE_WALL_GAIN_PWM_PER_RAD*predicted)))
        # Positive y is right; lower PWM turns right.
        self.servo = NEUTRAL-correction
        self.evidence = {'error_rad': error, 'trend_rad_s': trend, 'predicted_rad': predicted,
                         'correction_pwm': correction, 'history': len(self.history)}
        return self.servo


# First support from the lane (task rule: first support anticlockwise, i.e. on
# the car's left). It is acquired ahead-left (past the start divider's open
# end, left of the start lane) and approached on its planned loop (see the
# open-course loops below) once the approach strip is clear; the first-target
# stage takes over inside the usable window (from 45 deg, <= 3 m) with an
# adopted left command.
# Ahead to abeam only: the first support in route order is the one nearer the
# finish end, i.e. ahead along the start lane (simulation, 6.4 x 6 m course:
# the second support was behind-left at 124 deg while the first was ahead at
# 72 deg, and the car looped the second one first).
FIRST_ACQUIRE_BEARING_RAD = (math.radians(10), math.radians(100))
OPEN_FIRST_MIN_ALONG_M = -1.2  # the second support lies metres behind; a far first one may be just passed
FIRST_APPROACH_MAX_RANGE_M = 3.5
FIRST_HANDOVER_BEARING_RAD = math.radians(45)
# The loop stage's loss handling stops the car; hand over only once the support
# is well inside the 3 m compact-target range (simulation, 6 m wide course:
# handed over at ~2.9 m, lost at 3.0 m, stopped 3.07 m away and never
# re-acquired). Before that the start lane keeps steering the same loop plan
# and tolerates a lost frame.
OPEN_HANDOVER_MAX_RANGE_M = 2.5
# Without a first support, stop in front of a board with the measured rolling
# decel (agent.md section 5) plus a reaction allowance.
LANE_START_STOP_DECEL_MPS2 = .5
# 1560 is a power level, not a speed: on tiles the straight speed keeps rising
# past 2 m/s (2026-10-16). Once the measured lane speed reaches this cruise
# the start lane continues at 1550 (steady ~1.1 m/s on tiles).
# Until the first support is approached the start lane drives at the speed
# that still stops (reaction, ESC lag, nominal coast) LANE_START_SEARCH_M short
# of the stopping range, at most LANE_START_MAX_MPS: fast on a long lane,
# creeping near the end of a short one while the support is searched for.
# Above it the car coasts, well below it 1560, else 1550 (short courses: with
# the ESC lag a fixed cruise reached the stopping range before any support).
# Round 99 (user: 刚开始那个直道速度可以快点): run 98 cruised at 1.1-1.2 m/s
# on the open start lane (with the full ESC-lag rise, any 1560 frame above
# ~1.05 m/s was followed by a coast at the 1.5 cap); 1.8 only lets the lane
# go faster while the board ahead is far (the stopping taper above is
# unchanged and binds from ~5.5 m), so the first-loop approach speed is
# unchanged. course-sim: the same 98/110 courses succeed, gaps unchanged.
LANE_START_MAX_MPS = 1.8
LANE_START_SEARCH_M = 1.6
LANE_START_PUSH_BAND_MPS = .4
# Round 100: 1600 only while the measured speed is below START_LAUNCH_MAX_MPS
# and its full ESC-lag rise stays under the allowed speed; after a 1600
# command the rise (governor and stop check) uses START_LAUNCH_ACCEL_MPS2, an
# assumed upper bound (no 1600 straight measurement: 1560 reached ~1.4 m/s^2
# from rest in run 99), so the cut comes early rather than late.
START_LAUNCH_MAX_MPS = .2
START_LAUNCH_ACCEL_MPS2 = 2.5
# After the last 1600 frame: no 1560 for the ESC lag plus this (the closing-
# rate estimate trails a hard launch; course-sim with a .6 s ESC lag: 1560
# right after the burst reached 2.1 m/s and the stopping range cut the lane).
START_LAUNCH_SETTLE_S = .3
# Open-course loops (round 64, entry_kind 'lane' only). Without overlapping
# dividers there is no corridor round a support, so each support is looped
# on a planned circle: the task side (first support on the left,
# anticlockwise; second on the right, clockwise), the loop radius and the
# circle centre's offset from the support are chosen from the free distance
# measured from the support centre along five directions of the loop half
# (approach side, diagonals, far side, return side; compass frame), keeping
# the body as far as possible inside [support edge + inner gap, other return -
# outer gap] in every direction. Radii: left .80-.95 m (model 1/1.375 m at
# 1720); right .75-1.40 m with the statically accepted1200 endpoint (nominal
# 1.875/m, NOT dynamically measured). The car follows the tangent line plus that circle by pure pursuit
# (agent.md section 5: (servo-1500)/160 per m); a loop ends once the compass
# shows the heading reversed against its approach axis (first: start-lane
# axis 0 -> pi; second: pi -> 0). No course length, coordinate or time.
# Round 81 (Claude): with the track calibration the left wheels reach ~1.16 /m
# at the physical 1720 (radius ~.86 m), so a .80 m circle was not drivable;
# the lower bound is .90. A larger upper bound (1.10) chose 1.05 m on the
# round-80 free space but hit the start-end wall on the 5 m simulation course.
OPEN_LEFT_RADIUS_M = (.90, .95)
# Round 81 tried a first-loop turn-in lead (.75 of _response_pose until the
# car is on the circle); round 86 removed it: car runs 82/84/85 passed the
# first support at .060-.076 m body gap (the user saw the wheel touch it),
# 79/80/81/83 at .12-.14 m: entering the circle tighter put the car inside it.
OPEN_RIGHT_RADIUS_M = (.90, 1.40)  # measured right limit 1.2 /m (.83 m) plus margin
OPEN_RADIUS_STEP_M = .05
# Slack (m) traded per metre of loop radius: a larger loop keeps steering
# authority. This preference is soft; it does not establish physical curvature.
OPEN_RADIUS_PREFERENCE = .10
OPEN_APPROACH_SLOPE = 1.5
OPEN_INNER_GAP_M = .15
OPEN_OUTER_GAP_M = .15
OPEN_PROBE_HALF_WIDTH_M = SIDE_BODY_EXTENT_M+.10
OPEN_PROBE_REACH_M = 3.
OPEN_PROBE_DIRECTIONS_RAD = tuple(math.radians(a) for a in (-90, -45, 0, 45, 90))
OPEN_PROBE_QUANTUM_M = .05
# Free-space probes run from the support across other bearings; a scan taken
# while yawing fast is distorted (each bin from a different heading), so only
# frames below this compass rate refine them (a direction never probed yet
# takes the first frame).
OPEN_PROBE_MAX_RATE_RAD_S = .6
OPEN_SUPPORT_SKIP_M = .08
OPEN_LOOKAHEAD_M = 1.1
# Round 95 (car runs 92/94): the second loop's approach line lay ~.7 m to the
# side with the carrot 1.1 m ahead (.82 /m, 1338); with the ~.3 s steering lag
# at ~1.4 m/s the car turned ~28 deg past it, the support came dead ahead and
# the loop went round it in an S. On the approach line the carrot is at least
# this many times the car's offset from the line ahead (convergence <= ~22
# deg, curvature <= ~.63 /m), but at most half the way to the tangent point:
# up to the tangent point itself the car reached it unconverged on the 5 m
# courses and the loop touched the board (simulation). Sweep of k 1.6/2/2.5 x
# cap .5/.7/1 over 20 layouts/variants/measured cases: only cap .5 kept every
# case the baseline completes; 2.5/.5 cut steering reversals 17 -> 12, board
# gaps >= .16 m (short5 .11 -> .17, doc8x4 x1.3/.3 .13 -> .20).
OPEN_APPROACH_CONVERGE_RATIO = 2.5
OPEN_APPROACH_CONVERGE_CAP = .5
OPEN_CARROT_SUPPORT_GAP_M = .10
OPEN_EXIT_MARGIN_RAD = math.radians(25)
# A support lost this close to the reversed heading ends its loop (the next
# law steers by the compass); earlier the usual recovery applies.
OPEN_LATE_EXIT_RAD = math.radians(60)
OPEN_APPROACH_MARGIN_M = .05
OPEN_APPROACH_CHECK_M = 1.2
OPEN_LOST_HOLD_S = .5
# After the first loop: hold the reversed axis until one distinct support is
# ahead inside this band (lateral half width, along-axis reach).
OPEN_SECOND_BAND_HALF_M = 1.6
OPEN_SECOND_BAND_REACH_M = 3.2
OPEN_SECOND_BEYOND_FIRST_M = .8
# Round 88 (car run 88): the second support was first tracked at 3.16 m with
# 3 points (the most a ~.11 m support gives at 1 deg per bin there) and the
# board-ahead hold came .15 s later, before its confirmation (>= 3 scans over
# >= 250 ms, the next ~.3 s: <= .48 m at the measured open-stage 1.6 m/s).
# The hold then rolled 1.1 m at ~1 m/s and the track restarted (1 s lost).
# While that support is tracked in the current scan and lies nearer than the
# board, the hold waits until the board is this far inside its stopping range.
OPEN_SECOND_CONFIRM_GRACE_M = .5
# The second support is looked for from this first-loop progress on, and the
# first loop may hand over to the second loop's tangent from the next.
OPEN_SECOND_LOOKOUT_RAD = math.pi/2
OPEN_HANDOFF_MIN_PROGRESS_RAD = 2*math.pi/3
# The second loop may hand over to the light-lane entry from this progress on
# (the second support abeam or behind on the right).
OPEN_LIGHT_HANDOFF_PROGRESS_RAD = math.pi/2
OPEN_HEADING_HOLD_PWM = 120
# Transfer toward the second support's channel: the second support is passed
# clockwise, i.e. between it and the start divider. While it is not yet
# confirmed, aim at the start divider's open end (a board parallel to the axis
# on the left whose near end lies ahead), half a start-lane width inside it.
# Simulation, 4 m wide courses: diving only after the confirmation (~2.5 m)
# overshot ~.2 m toward the divider and the body gate stopped the car.
OPEN_CHANNEL_OFFSET_M = (.40, .60)
OPEN_CHANNEL_MAX_LATERAL_M = 3.5
# The pursuit point lies at least this many times its lateral offset ahead
# (descent <= ~34 deg; simulation 6 m course: aiming at the open end 2.85 m
# aside and 1.3 m ahead dove at ~65 deg past the second support).
OPEN_CHANNEL_SLOPE = 1.5
# Steering step of the open-course pursuit (approach, loops, transfer, light
# entry). Simulation (rotation-compensated tracking): 40 completed all named
# courses; 80 overshot the 5 m course's crossing into the start divider.
OPEN_PWM_STEP = 40
# Round 79 (Claude): after the first pass (transfer, second loop approach
# line, light entry) a pursuit command comes from the pose the car reaches
# before a new command acts whenever, seen from there, the pursuit point lies
# at centre or on the other side (_overshoot_guard: the turn would carry the
# car past it even if released now); and a release toward centre moves 80 PWM
# per adopted step (turning harder keeps OPEN_PWM_STEP; every step still waits
# for its adoption).
# Rounds 78/79: planned from the current pose, the second loop asked full
# right until the car faced its approach line; the wheels then needed the .3 s
# servo lag (rounds 70-78 fit) plus ~.13-.15 s per adopted 40-PWM step to
# come back, and at the open stages' ~1.3 m/s (median 1.27-1.61 m/s, rounds
# 77-79) the car turned another 60-100 deg and crossed in front of the second
# support; after the first pass the 40-PWM release from 1713 let it turn ~60
# deg past the transfer axis. Horizon: the lag plus half the time to release
# the current command to centre; the measured yaw rate is held along an arc
# at the nominal speed. Simulation (measured car x1.3 speed, delays .15/.3,
# user8/doc8x4, and the two nominal-car regression variants): all complete;
# a full lead (always pursuing from the predicted pose) also passed the
# measured cases but trimmed non-overshooting turns of the faster nominal car
# (5 m course: start-end wall), as did a guard at half the current turn.
OPEN_STEER_LAG_S = .3
OPEN_RELEASE_PWM_STEP = ROUTE_RETURN_RELEASE_PWM_STEP
OPEN_RELEASE_PWM_PER_S = 550.  # 80 PWM per adopted step every ~.14 s
OPEN_PREDICT_SPEED_MPS = 1.3
# Light lane entry: the lane between the outermost wall on the left and the
# next parallel wall inside it; without that inner wall the start lane's
# measured width is the prior. Pursuit point at the divider's open end
# (bounded) on the lane centre line.
OPEN_LANE_PARALLEL_RAD = math.radians(12)
OPEN_LANE_MIN_SPAN_M = .5
OPEN_LANE_LOOKAHEAD_M = (.8, 1.6)
OPEN_LANE_WIDTH_M = (.65, 2.2)
OPEN_LANE_ENTRY_HEADING_RAD = math.radians(20)
OPEN_LANE_INSIDE_MARGIN_M = .05
# Round 102: a blocked light-entry line moves to a free one; gaps narrower
# than this (lateral room for the line, body strip excluded) are taken only
# when no wider one is in reach (_open_entry_free_lateral).
OPEN_LANE_SHIFT_MIN_GAP_M = .20
OPEN_LINE_BAND_M = .06
OPEN_LINE_MIN_POINTS = 8
OPEN_LINE_GAP_M = .3
OPEN_LINE_GAP_SCALE = .03
OPEN_LINE_REACH_M = 6.
OPEN_STAGES = ('open_transfer', 'second_loop', 'light_entry')


def _wrap90(angle):
    quarter = math.pi/2
    return (angle+quarter/2) % quarter-quarter/2


class WallCompass:
    """Car heading relative to the start-lane axis, from native finite walls.

    The heading is continuous (never wrapped): the first U-turn reads about
    +pi, the second (clockwise) back about 0. A frame without enough wall
    support only predicts with the last measured rate; ``current`` reports
    no heading once the last measurement is older than COMPASS_MAX_STALE_S.
    No coordinate, course size or turn time is involved.
    """
    def __init__(self):
        self.heading = None
        self.rate = 0.
        self.at_s = None
        self.measured_at_s = None
        self._last_measurement = None
        self.evidence = None

    def update(self, scan):
        at = scan['at_ms']/1000
        if self.at_s is not None and at <= self.at_s:
            return self.heading
        predicted = 0. if self.heading is None else self.heading+self.rate*min(at-self.at_s, COMPASS_MAX_PREDICT_S)
        votes = self._votes(scan, predicted)
        support = sum(span for _, span in votes)
        median = None
        if support >= COMPASS_MIN_SUPPORT_M:
            votes.sort()
            accumulated = 0.
            for residual, span in votes:
                accumulated += span
                if accumulated >= support/2:
                    median = residual
                    break
            heading = predicted-median
            if self._last_measurement is not None:
                dt = at-self._last_measurement[1]
                if COMPASS_RATE_WINDOW_S[0] <= dt <= COMPASS_RATE_WINDOW_S[1]:
                    self.rate = max(-COMPASS_MAX_RATE_RAD_S, min(COMPASS_MAX_RATE_RAD_S,
                                                                 (heading-self._last_measurement[0])/dt))
            self._last_measurement = (heading, at)
            self.heading, self.measured_at_s = heading, at
        elif self.heading is not None:
            self.heading = predicted
        self.at_s = at
        self.evidence = {'source_at_ms': scan['at_ms'], 'heading_rad': self.heading, 'rate_rad_s': self.rate,
                         'predicted_rad': predicted, 'votes': len(votes), 'support_m': support,
                         'median_residual_rad': median, 'measured': median is not None,
                         'basis': 'native_wall_directions_mod_90_vs_start_lane_axis'}
        return self.heading

    @staticmethod
    def _votes(scan, predicted):
        votes = []
        for wall in scan.get('wall_candidates') or []:
            if (not _valid_wall(wall) or wall['support_span_m'] < COMPASS_MIN_SPAN_M
                    or wall['fit_error_m'] > COMPASS_MAX_FIT_M or wall['points'] < COMPASS_MIN_POINTS):
                continue
            residual = _wrap90(predicted+wall['heading_left_rad'])
            if abs(residual) <= COMPASS_GATE_RAD:
                votes.append((residual, wall['support_span_m']))
        return votes

    def measurable(self, scan):
        """Read-only: this scan's native walls would measure the heading (as update does)."""
        if self.heading is None or self.at_s is None or not isinstance(scan, dict) or type(scan.get('at_ms')) is not int:
            return False
        predicted = self.heading+self.rate*min(max(0., scan['at_ms']/1000-self.at_s), COMPASS_MAX_PREDICT_S)
        return sum(span for _, span in self._votes(scan, predicted)) >= COMPASS_MIN_SUPPORT_M

    def current(self, scan):
        """Heading for this scan's publication, or None when stale/uninitialised."""
        if (self.heading is None or self.measured_at_s is None or not isinstance(scan, dict)
                or type(scan.get('at_ms')) is not int):
            return None
        at = scan['at_ms']/1000
        if not 0 <= at-self.measured_at_s <= COMPASS_MAX_STALE_S:
            return None
        return self.heading+self.rate*min(max(0., at-self.at_s), COMPASS_MAX_PREDICT_S)


def _axis_frame(axis, heading, side=1):
    """Approach axis e1 and the turn-side normal e2 in the car frame (left +)."""
    angle = axis-heading
    e1 = (math.cos(angle), math.sin(angle))
    return e1, (-side*e1[1], side*e1[0])


def _scan_points(ranges, reach):
    return [(r*_DIRECTION_COS[i], -r*_DIRECTION_SIN[i]) for i, r in enumerate(ranges)
            if _number(r) and .02 <= r <= reach]


def _support_center(ranges, bins, diameter):
    """Centre and radius of a round support from its visible arc (car frame).

    The visible half of a circle has its centroid 2r/pi in front of the
    centre, so the centre lies that far beyond the arc centroid on its ray.
    """
    points = [(ranges[i]*_DIRECTION_COS[i], -ranges[i]*_DIRECTION_SIN[i]) for i in bins]
    x, y = sum(p[0] for p in points)/len(points), sum(p[1] for p in points)/len(points)
    radius = max(.05, min(MAX_DIAMETER_M/2, (diameter if _number(diameter) else .2)/2))
    distance = math.hypot(x, y)
    if distance > 1e-6:
        x, y = x+2*radius/math.pi*x/distance, y+2*radius/math.pi*y/distance
    return (x, y), radius


def _probe_free(points, origin, direction, skip):
    """Free distance from origin along direction to the first return in a body-wide strip."""
    ox, oy = origin
    ux, uy = direction
    best = math.inf
    for x, y in points:
        dx, dy = x-ox, y-oy
        if dx*dx+dy*dy <= skip*skip:
            continue
        along = dx*ux+dy*uy
        if 0 < along < best and along <= OPEN_PROBE_REACH_M and abs(dy*ux-dx*uy) <= OPEN_PROBE_HALF_WIDTH_M:
            best = along
    return best


def _loop_extent(a, b, radius, theta):
    """Distance from the support centre along axis-frame direction theta to a
    loop circle of this radius whose centre is offset (a, b) from it."""
    projection = a*math.cos(theta)+b*math.sin(theta)
    return projection+math.sqrt(max(0., projection*projection-(a*a+b*b)+radius*radius))


_PLAN_MARGINS = []  # (probes/support/bounds, {(radius, a, b): smallest probe margin}), last 4


def _plan_margins(context):
    for known, table in _PLAN_MARGINS:
        if known == context:
            return table
    table = {}
    _PLAN_MARGINS.append((context, table))
    del _PLAN_MARGINS[:-4]
    return table


def _plan_loop(free, support_radius, radii, approach=None):
    """Loop radius and centre offset (a along the approach axis, b toward the
    turn side) maximising the smallest clearance slack over the probes.

    ``free`` holds (theta, free distance) in the approach-axis frame. A body
    centre inside [lo, hi] keeps the inner gap to the support and the outer
    gap to the probed return. If both comfort margins do not fit, reduce
    cone comfort first instead of centring the violation against the board;
    the circle still encloses the support plus the body side. Offsets are
    bounded by this circle, not an arbitrary fixed displacement.
    ``approach`` (the car's (along, lateral) from the support in that frame)
    penalises an approach line the car cannot reach before its tangent point
    at a descent of at most 1/OPEN_APPROACH_SLOPE (simulation: a line 1.5 m
    below the car 1.9 m before the tangent point made the car dive, overshoot
    and run wide at the right steering bound).
    """
    lo = support_radius+SIDE_BODY_EXTENT_M+OPEN_INNER_GAP_M
    bands = [(theta, distance-SIDE_BODY_EXTENT_M-OPEN_OUTER_GAP_M) for theta, distance in free]
    inner_floor = support_radius+SIDE_BODY_EXTENT_M if any(hi < lo for _, hi in bands) else lo

    # The coordinate search revisits the same (radius, a, b); _loop_extent is
    # inlined with the probe directions' cos/sin computed once, and offsets
    # outside the circle (score -inf) are skipped in the sweeps. Same
    # arithmetic, so the same plan. Round 64 car run: ~2000 evaluations per
    # scan were the slowest step of the second loop.
    # Round 87 (car run 87: the second-loop entry tick took .085 s of CPU):
    # the smallest probe margin of (radius, a, b) does not depend on the
    # approach, so it is kept across plans with the same probes, support and
    # bounds (_plan_margins); a repeated sweep reuses its choice; a sweep only
    # visits offsets that can lie inside the circle (the exact test stays);
    # max/min calls are written as the equal comparisons. Identical plans on
    # 4606 recorded and random inputs; ~.29 instead of ~.52 ms (Mac).
    trig = [(math.cos(theta), math.sin(theta), hi, hi >= lo) for theta, hi in bands]
    hypot, sqrt, inf = math.hypot, math.sqrt, math.inf
    memo, sweeps = {}, {}
    margins = _plan_margins((lo, inner_floor, tuple(trig)))

    def score(radius, a, b):
        key = (radius, a, b)
        value = memo.get(key)
        if value is not None:
            return value
        if hypot(a, b) > radius-inner_floor:
            memo[key] = -inf
            return -inf
        worst = margins.get(key)
        if worst is None:
            worst = inf
            ab, rr = a*a+b*b, radius*radius
            for c, s, hi, comfort in trig:
                projection = a*c+b*s
                d = projection*projection-ab+rr
                t = projection+sqrt(d if d > 0. else 0.)
                # Cone comfort is soft; a board must not be traded for it when
                # both comfort margins do not fit. The raw body gate is unchanged.
                if comfort:
                    inner, outer = t-lo, hi-t
                    margin = outer if outer < inner else inner
                else:
                    margin = -abs(t-hi)
                if margin < worst:
                    worst = margin
            margins[key] = worst
        if approach is not None and a-approach[0] > 0:
            descent = OPEN_APPROACH_SLOPE*abs(approach[1]-(b-radius))-(a-approach[0])
            worst -= descent if descent > 0. else 0.
        value = memo[key] = worst-.01*hypot(a, b)-OPEN_RADIUS_PREFERENCE*(radii[1]-radius)
        return value

    def sweep(radius, offsets, a=None, b=None):
        """max(offsets, key=score) over the free coordinate (first maximum);
        offsets with an infinite (outside) score cannot be it unless all are."""
        key = (radius, a, b)
        choice = sweeps.get(key)
        if choice is not None:
            return choice
        limit = radius-inner_floor
        choice, top = offsets[0], None
        fixed = b if a is None else a
        reach = limit*limit-fixed*fixed
        if reach >= 0.:
            middle = len(offsets)//2  # offsets[middle] == 0
            k = int((sqrt(reach)+1e-6)/.05)+1
            for v in offsets[max(0, middle-k):middle+k+1]:
                pa, pb = (v, b) if a is None else (a, v)
                if hypot(pa, pb) > limit:
                    continue
                value = score(radius, pa, pb)
                if top is None or value > top:
                    choice, top = v, value
        sweeps[key] = choice
        return choice

    best = None
    radius = radii[0]
    while radius <= radii[1]+1e-9:
        a = b = 0.
        offsets = tuple(k*.05 for k in range(-math.ceil(radius/.05), math.ceil(radius/.05)+1))
        for _ in range(2):
            b = sweep(radius, offsets, a=a)
            a = sweep(radius, offsets, b=b)
        value = score(radius, a, b)
        if best is None or value > best[0]:
            best = (value, radius, a, b)
        radius += OPEN_RADIUS_STEP_M
    value, radius, a, b = best
    return {'radius_m': round(radius, 3), 'offset_axis_m': round(a, 3), 'offset_side_m': round(b, 3),
            'min_slack_m': value, 'inner_bound_m': lo,
            'inner_floor_m': inner_floor,
            'board_slack_m': min(hi-_loop_extent(a, b, radius, theta) for theta, hi in bands),
            'extents_m': [round(_loop_extent(a, b, radius, theta), 3) for theta, _ in free]}


def _segment_distance(point, end):
    """Distance from ``point`` to the segment from the origin to ``end``."""
    length2 = end[0]**2+end[1]**2
    t = 0. if length2 < 1e-12 else max(0., min(1., (point[0]*end[0]+point[1]*end[1])/length2))
    return math.hypot(point[0]-t*end[0], point[1]-t*end[1])


def _loop_carrot(center, radius, side, lookahead, axis_dir, turn_normal, support=None):
    """Pure-pursuit point on the approach line, then the loop circle.

    The approach line runs along the approach axis and touches the circle on
    its approach side (away from the turn); the car first converges onto it
    like onto a lane, then follows the circle (side +1 anticlockwise, -1
    clockwise). A tangent drawn from wherever the car happens to be made steep
    dives when a support was confirmed late (simulation: the second loop dove
    past the start divider's open end into the start lane). When the straight
    way to that point passes the support (``support`` = (centre, clearance))
    closer than its clearance (simulation: a car far above the line passed
    .02 m from it), the tangent from the car is used instead, which never
    enters the circle round the support.
    Car at the origin heading +x. Returns (point, mode, distance before the
    tangent point along the axis).
    """
    ox, oy = center
    ex, ey = axis_dir
    nx, ny = turn_normal
    tx, ty = ox-radius*nx, oy-radius*ny
    before = tx*ex+ty*ey
    distance = math.hypot(ox, oy)
    if before > 0:
        if before > lookahead:
            offset = abs(ey*tx-ex*ty)  # the car's distance from the approach line
            lookahead = max(lookahead, min(OPEN_APPROACH_CONVERGE_CAP*before, OPEN_APPROACH_CONVERGE_RATIO*offset))
        s = lookahead-before
        if s <= 0:
            point, mode = (tx+s*ex, ty+s*ey), 'approach_line'
        else:
            phi = math.atan2(ty-oy, tx-ox)+side*s/radius
            point, mode = (ox+radius*math.cos(phi), oy+radius*math.sin(phi)), 'approach_line_to_circle'
        if support is None or _segment_distance(support[0], point) >= support[1]:
            return point, mode, before
    if distance > radius+1e-6:
        tangent = math.sqrt(distance*distance-radius*radius)
        theta = math.atan2(oy, ox)-side*math.asin(radius/distance)
        if tangent >= lookahead:
            return (lookahead*math.cos(theta), lookahead*math.sin(theta)), 'tangent', max(before, 0.)
        px, py = tangent*math.cos(theta), tangent*math.sin(theta)
        phi = math.atan2(py-oy, px-ox)+side*(lookahead-tangent)/radius
        return (ox+radius*math.cos(phi), oy+radius*math.sin(phi)), 'tangent_to_circle', max(before, 0.)
    phi = math.atan2(-oy, -ox)+side*lookahead/radius
    return (ox+radius*math.cos(phi), oy+radius*math.sin(phi)), 'circle', 0.


def _in_pose(point, pose):
    """A car-frame point as seen from ``pose`` (x, y, heading change, ...)."""
    dx, dy = point[0]-pose[0], point[1]-pose[1]
    c, s = math.cos(-pose[2]), math.sin(-pose[2])
    return (c*dx-s*dy, s*dx+c*dy)


def _turned(vector, turn):
    c, s = math.cos(-turn), math.sin(-turn)
    return (c*vector[0]-s*vector[1], s*vector[0]+c*vector[1])


def _overshoot_guard(pwm, predicted):
    """The command to send: ``predicted`` (pursuit from the response pose) only
    when the turn would carry the car past its pursuit point even if released
    now (the predicted point lies at centre or on the other side), else ``pwm``.
    Round 79: a full lead also trimmed turns that were not overshooting
    (simulation, 5 m course: the second loop then started too close to its
    support and ended at the start-end wall)."""
    if pwm != NEUTRAL and (predicted-NEUTRAL)*(pwm-NEUTRAL) <= 0:
        return predicted
    return pwm


def _pursuit_pwm(point):
    curvature = 2*point[1]/max(1e-6, point[0]**2+point[1]**2)
    return curvature, max(RIGHT_EXIT_MIN_PWM, min(SERVO_MAX, NEUTRAL+round(ROUTE_PWM_PER_CURVATURE*curvature)))


def _strip_clear(points, end, skip_center, skip_radius):
    """No return in the body-wide strip from the lidar to ``end`` (plus the front)."""
    length = math.hypot(*end)
    if length < 1e-6:
        return True
    ux, uy = end[0]/length, end[1]/length
    half = SIDE_BODY_EXTENT_M+OPEN_APPROACH_MARGIN_M
    for x, y in points:
        if skip_center is not None and math.dist((x, y), skip_center) <= skip_radius:
            continue
        along = x*ux+y*uy
        if -REAR_BODY_EXTENT_M < along <= length+FRONT_BODY_EXTENT_M and abs(y*ux-x*uy) <= half:
            return False
    return True


def _axis_lines(points, heading):
    """Boards parallel to the compass axis from raw returns: (lateral, along span).

    The native finite walls miss a divider seen at a grazing angle from the
    open area (simulation, 8x6 course: never reported), so returns are
    grouped by their lateral offset in the axis frame, then split into runs
    contiguous along the axis (a stray return of a crossing board at the same
    offset once stretched a divider ahead to 2.4 m behind the car). The
    allowed gap grows with the grazing bin spacing (~r^2 dtheta / lateral). A
    run needs OPEN_LINE_MIN_POINTS and OPEN_LANE_MIN_SPAN_M; a compact support
    is far shorter and never qualifies.
    """
    c, s = math.cos(-heading), math.sin(-heading)
    rows = sorted((-s*x+c*y, c*x+s*y) for x, y in points)
    # Round 64 car run: the plain form (every window re-sorted and re-split,
    # quadratic in a board's points) took ~3 ms per call in simulation, about
    # .1 s on the car. The window's along values are kept sorted as rows enter
    # and leave it, and only gaps wider than OPEN_LINE_GAP_M (the smallest
    # possible split) are tracked; the splits, medians and the replacement
    # rule are unchanged (identical output on 2241 recorded and 879 simulated
    # scans).
    lines, start = [], 0
    window = []   # along values of rows[start:end+1], sorted
    wide = set()  # adjacent (a, b) in window with b-a > OPEN_LINE_GAP_M
    for end in range(len(rows)):
        value = rows[end][1]
        j = bisect_left(window, value)
        if 0 < j < len(window) and window[j]-window[j-1] > OPEN_LINE_GAP_M:
            wide.discard((window[j-1], window[j]))
        if j > 0 and value-window[j-1] > OPEN_LINE_GAP_M:
            wide.add((window[j-1], value))
        if j < len(window) and window[j]-value > OPEN_LINE_GAP_M:
            wide.add((value, window[j]))
        window.insert(j, value)
        while rows[end][0]-rows[start][0] > OPEN_LINE_BAND_M:
            value = rows[start][1]
            j = bisect_left(window, value)
            left = window[j-1] if j > 0 else None
            right = window[j+1] if j+1 < len(window) else None
            if left is not None and value-left > OPEN_LINE_GAP_M:
                wide.discard((left, value))
            if right is not None and right-value > OPEN_LINE_GAP_M:
                wide.discard((value, right))
            if left is not None and right is not None and right-left > OPEN_LINE_GAP_M:
                wide.add((left, right))
            del window[j]
            start += 1
        count = end+1-start
        if count < OPEN_LINE_MIN_POINTS or window[-1]-window[0] < OPEN_LANE_MIN_SPAN_M:
            continue
        lateral = rows[start+count//2][0]  # rows are sorted by lateral first
        scale = max(abs(lateral), .3)
        runs, first = [], 0
        for a, b in sorted(wide):
            if b-a > max(OPEN_LINE_GAP_M, OPEN_LINE_GAP_SCALE*max(a*a, b*b)/scale):
                k = bisect_left(window, b)
                runs.append((first, k-1))
                first = k
        runs.append((first, len(window)-1))
        for i0, i1 in runs:
            n = i1-i0+1
            if n < OPEN_LINE_MIN_POINTS or window[i1]-window[i0] < OPEN_LANE_MIN_SPAN_M:
                continue
            line = {'lateral_m': lateral, 'along_m': [window[i0], window[i1]], 'points': n, 'source': 'raw_returns'}
            same = [i for i, old in enumerate(lines) if abs(old['lateral_m']-lateral) <= OPEN_LINE_BAND_M
                    and old['along_m'][0] <= window[i1] and window[i0] <= old['along_m'][1]]
            if not same:
                lines.append(line)
            elif n > lines[same[0]]['points']:
                lines[same[0]] = line
    return lines


# A compact return lying on an axis-parallel board run (or just beyond its end)
# is that board's end face, not a support (simulation, measured car on the
# re-laid course: the start divider's open end, seen end-on, became the
# second support at 2.9 m and was lost as the view changed).
OPEN_BOARD_END_LATERAL_M = .15
OPEN_BOARD_END_ALONG_M = .5
# A round support's visible arc spans at most about its radius in range; four
# returns receding .09 m per bin (.27 m) were a board face at a grazing angle
# next to its open end (same simulation). Cones here are ~.1 m in radius.
OPEN_SUPPORT_MAX_DEPTH_M = .15


def _support_depth(ranges, bins):
    values = [ranges[i % 360] for i in bins if _number(ranges[i % 360])]
    return max(values)-min(values) if values else 0.


def _on_board_run(point, lines, heading):
    """Whether a car-frame point lies on (or just past the end of) an _axis_lines run."""
    c, s = math.cos(-heading), math.sin(-heading)
    x, y = point
    lateral, along = -s*x+c*y, c*x+s*y
    return any(abs(line['lateral_m']-lateral) <= OPEN_BOARD_END_LATERAL_M
               and line['along_m'][0]-OPEN_BOARD_END_ALONG_M <= along <= line['along_m'][1]+OPEN_BOARD_END_ALONG_M
               for line in lines)


def validate_maneuver_initial_pwm(value):
    if type(value) is not int or not INITIAL_PWM_MIN <= value <= SERVO_MAX:
        raise ValueError('invalid_maneuver_initial_presteer_pwm')
    return value


class ManeuverSequence(TurnMotion):
    """Single arm / cumulative budget; terminal coast never restarts.

    Explicit route mode may perform one fresh-evidence neutral quality wait.
    It does not reset a drive/phase budget or certify a physical two-cone path.
    """
    def __init__(self, started_at, max_drive_s=10.0, initial_presteer_pwm=DEFAULT_INITIAL_PWM,
                 continue_route=False):
        if type(continue_route) is not bool:
            raise ValueError('invalid_continue_route')
        validate_maneuver_initial_pwm(initial_presteer_pwm)
        route_drive_s = None
        if (continue_route and _number(max_drive_s)
                and MAX_DRIVE_S < max_drive_s <= ROUTE_MAX_DRIVE_S):
            route_drive_s, max_drive_s = max_drive_s, MAX_DRIVE_S
        super().__init__(started_at, max_drive_s=max_drive_s,
                         initial_presteer_pwm=initial_presteer_pwm)
        if route_drive_s is not None:
            self.max_drive_s = route_drive_s
        # User-selected neutral preparation budget for this experiment only.
        # Parent construction and default TurnMotion retain their five seconds.
        self.max_presteer_s = PRESTEER_MAX_S
        self.continue_route = continue_route
        self.target_tracker = (CompactTargetTracker(maintenance_bearing_rad=(-math.pi, math.pi),
                                                   select_associated=True, unconfirmed_boundary_gap=True,
                                                   tolerant=True)
                               if continue_route else CompactTargetTracker())
        self.compact_target = None
        self._route_left_hold_since = None
        self.left_overshoot_evidence = None
        self._last_seen_compact = None  # (receive time, copy) of the latest first-tracker observation
        self._last_confirmed_compact_target = None
        self.compact_target_error = None
        self.compact_loss_evidence = None
        self.handover_observed = False
        self.orbit_since = self.orbit_reference_range_m = None
        self.orbit_bias_pwm = None
        self.orbit_feedback = None
        self.orbit_left_entry_boost = False
        self._orbit_abeam_count = 0
        self._orbit_abeam_publication = self._orbit_abeam_receive = None
        self.left_turn_feedback = None
        self.left_handover_preparing = False
        self.handover_control = None
        self.handover_wait_since = None
        self.orbit_track_id = None
        self.coast_since = None
        self.wall_ambiguity_hold = False
        self._last_actual_left_target = None
        self._presteer_adoption_context = None
        self._current_scan = None
        self.inner_clearance = None
        self.quality_coast_servo = None
        self.quality_wait_refusal = None
        self.left_turn_pause_used = False
        self.left_turn_pause_evidence = None
        self.left_turn_pause_geometry_error = None
        # Self-recovery (trial47 operator): recoverable stops hold at neutral
        # and keep re-checking whether the current task can resume; only the
        # total deadline and cumulative drive budget bound how often.
        self.neutral_pause_kind = None
        self.recovery_count = 0
        self.recovery_evidence = None
        self.post_pass_pause_evidence = None
        self.quality_coast_reason = None
        self.entry_bearing_required = False
        self.entry_bearing_released = False
        self.entry_bearing = None
        self._incoming_association = None
        self._entry_last_receive = self._entry_last_publication = None
        self._entry_ready_count = 0
        self._entry_ready_receive = self._entry_ready_publication = None
        self._entry_ready_observation_type = None
        self._entry_ready_history = []
        self.adopted_presteer_pwm = None
        self.first_pass_preparing = False
        self.first_pass_evidence = None
        self.first_pass_progress = None
        self._first_pass_history = []
        self._first_exit_prepare_history = []
        self._quality_exit_prepare_ready = False
        # Neutral-wait release: decided from current full first-target support
        # (or a release already latched before the wait). Release only.
        self.quality_wait_release = None
        self.quality_wait_previous_servo = None
        self._wait_release_history = []
        self.quality_wait_anchor = None
        self.quality_wait_rebind = None
        self._wait_rebind_candidate = None
        self.first_exit_prepare_evidence = None
        self.right_exit_since = None
        self.right_exit_center_ack = None
        self.right_exit_adoption_wait_since = None
        self.right_exit_geometry = None
        self._right_exit_last_receive = self._right_exit_last_publication = None
        self.route_stage = 'first_target'
        self.route_events = {}
        self.quality_wait_since = self.quality_wait_servo = None
        self.quality_wait_used = False
        self.quality_wait_evidence = None
        self.quality_resume_pending = False
        self.quality_resume_since = None
        self._quality_good_history = []
        self._quality_full_streak = 0
        self._quality_pass_history = []
        self._route_alignment_history = []
        self.route_alignment_evidence = None
        self._route_center_ack = None
        self.second_tracker = self._new_second_tracker() if continue_route else None
        self.second_target = None
        self.second_track_id = None
        self.second_target_handover_observed = False
        self.second_center_ack = None
        self.second_orbit_since = None
        self.second_pass_evidence = None
        self.second_feedback = None
        self.second_return_preparing = False
        self.second_return_evidence = None
        self.route_end_coast_servo = None
        self.light_since = None
        self.light_turn_started = None  # second pass time; not renewed by a pause
        self._lane_steering = CorridorSteering()
        self._lane_speed_last = None
        self._lane_pulse = 0
        self._lane_creep = False  # round 105: the current 1560 pulse is a creep
        self._lane_creeps = 0
        self._light_history = []
        self._light_heading = None
        self.light_feedback = None
        self.light_alignment_evidence = None
        self.lane_motor = ROUTE_SLOW_MOTOR
        self._lane_coast_track = []
        self._lane_coast_decel = None
        self._lane_coast_decel_se = None
        self._lane_ranges = []
        self.lane_feedback = None
        self.finish_evidence = None
        self._exit_heading = None
        self.role_distinct_evidence = None
        self.second_acquisition_reason = None
        self._second_proof_at = None
        self._second_pass_history = []
        self._second_role_anchor = None
        self._second_recovering = False
        self._second_recovery_last = None
        self.second_recovery_prior = None
        self.second_rebind_evidence = None
        self._pause_stage_held = False
        self._pause_steer_target = None
        # Route: current-scan proximity avoidance and steering inside waits.
        self._avoid = None
        # Per-scan results reused by the 20 ms ticks of one scan (round 64 car
        # run: repeated per-tick work held the control lock long enough to
        # starve the heartbeat). Keyed by seq/at_ms and the ranges' content.
        self._proximity_cache = None
        self._axis_cache = None
        self._exit_release_cache = None
        self._path_cache = None
        self.path_evidence = None
        self.path_override_count = 0
        self.avoid_count = 0
        self.avoid_evidence = []
        self.wait_steering_changes = 0
        # Round 64 (Claude): wall compass (route mode) and the start-lane entry.
        self.compass = WallCompass()
        self.entry_kind = None  # 'junction' (native left opening at the start) or 'lane'
        self._lane_start_steering = LaneWallSteering()
        self._lane_start_settle_since = None
        self._lane_start_ticks = 0
        self._lane_start_slow = False
        # Searching for the first support near the start lane's stopping range.
        self._start_motor = TRIAL_MOTOR
        self._start_speed_measured = False
        self._start_speed_last = None  # round 100: (time, speed) of the last start-lane measurement
        self.start_lane_coast_steering = None
        self.start_side_wall = None
        self._front_track = None
        self._track_history = []
        self._power_cmd_at = None  # last output frame with a drive motor (ESC lag)
        self._power_since = None  # first output frame of that powered run (round 101)
        self._power_prev_end = None  # last output frame of the powered run before it
        self._launch_cmd_at = None  # round 100: last output frame at START_LAUNCH_MOTOR
        self._power_cmd_motor = None
        self._launch_until = None
        self.start_lane_feedback = None
        self._lane_start_ok = False
        self.first_approach = None
        self._start_lane_width = None
        # Open-course loops (entry_kind 'lane').
        self._open_slots = {}  # loop side (+1 first, -1 second) -> probes and plan
        self._open_lost_since = None
        self._second_plan_pending = None  # (seq, at_ms) of the scan that confirmed the second support
        self.open_loop = None
        self.open_next_loop = None
        self.open_feedback = None
        self._open_lookout_seq = None

    @staticmethod
    def _new_second_tracker():
        # Trial43: the second support first appears ahead on either side;
        # once confirmed it moves from the front through the right to behind
        # the rear, where the pass is matured over .25 s.
        tracker = CompactTargetTracker(acquisition_bearing_rad=(-math.pi/2, math.pi/2),
            maintenance_bearing_rad=(-math.pi, math.pi/2), select_associated=True,
            unconfirmed_boundary_gap=True, tolerant=True,
            acquisition_min_points=SECOND_ACQUISITION_MIN_POINTS)
        # Round 77: the second cone was first seen at 2.81 m (a 3 m limit) and
        # confirmed too late; acquire from 3.5 m as the first one does.
        tracker.extend_acquisition_range(EXTENDED_MAX_RANGE_M)
        return tracker

    def _route_event(self, name, scan, control, evidence=None):
        # One snapshot per transition, never an unbounded in-control log.
        if name not in self.route_events:
            self.route_events[name] = {'scan': copy.deepcopy({k: scan.get(k) for k in
                ('frame_id', 'seq', 'at_ms', 'received_at', 'ranges', 'corridor_candidates')}),
                'control': copy.deepcopy(control), 'evidence': copy.deepcopy(evidence)}

    @staticmethod
    def _full_target_points(target, scan, identity):
        if (not isinstance(target, dict) or target.get('track_id') != identity
                or target.get('confirmed') is not True
                or target.get('tracking_only_boundary_gap') is not False
                or target.get('source_seq') != scan.get('seq')
                or target.get('source_at_ms') != scan.get('at_ms')
                or target.get('source_received_at') != scan.get('received_at')):
            return None
        bins, ranges = target.get('support_bins'), scan.get('ranges')
        if (not isinstance(bins, list) or len(bins) < TOLERANT_MAINTENANCE_MIN_POINTS or len(set(bins)) != len(bins)
                or not isinstance(ranges, list) or len(ranges) != 360
                or any(type(i) is not int or not 0 <= i < 360 or not _number(ranges[i]) for i in bins)):
            return None
        points = [(ranges[i]*math.cos(math.radians(i)), -ranges[i]*math.sin(math.radians(i))) for i in bins]
        unknown = target.get('boundary_unknown_bins') or []
        if any(type(i) is not int or not 0 <= i < 360 for i in unknown):
            return None
        # Size-bounded unknown edge bins (compact_target maintenance rule) may
        # hide part of the support. Count them at the nearest observed support
        # range: forward/lateral extents used for release, pass and bypass
        # decisions can then only be conservative, never optimistic.
        near = min(ranges[i] for i in bins)
        points += [(near*math.cos(math.radians(i)), -near*math.sin(math.radians(i))) for i in unknown]
        return points

    def current_contact_target(self, scan, now):
        """Exact current returns of every cone-like support, for the body gate.

        User 2026-10-12: a cone may be hit and the run continues; boards
        still stop. Any isolated compact support facing the sensor in this
        fresh scan (compact_target.contact_supports) is a witness, whether or
        not it is the support being orbited or already confirmed. The service
        recomputes the same supports from the raw scan before exempting them.
        Read-only: no tracker or stage state changes.
        """
        if (not self.continue_route or self.phase not in ('drive', 'quality_wait')
                or self.drive_since is None or not isinstance(scan, dict)
                or not _number(now) or not _number(scan.get('received_at'))
                or not 0 <= now-scan['received_at'] < SCAN_AGE_S
                or type(scan.get('seq')) is not int or type(scan.get('at_ms')) is not int
                or not isinstance(scan.get('ranges'), list) or len(scan['ranges']) != 360):
            return None
        try:
            supports = contact_supports(scan['ranges'])
        except (ValueError, TypeError, KeyError):
            return None
        if not supports:
            return None
        bins = sorted({i for support in supports for i in support['support_bins']})
        return {'role': 'cone_like_support', 'track_id': None, 'confirmed': False, 'full_observation': True,
                'source_seq': scan['seq'], 'source_at_ms': scan['at_ms'],
                'source_received_at': scan['received_at'], 'support_bins': bins,
                'support_ranges_m': [scan['ranges'][i] for i in bins], 'semantic_class': 'unknown',
                'objects': [{'support_bins': list(support['support_bins']), 'range_m': support['range_m'],
                             'bearing_left_rad': support['bearing_left_rad'],
                             'diameter_m': support['diameter_m']} for support in supports],
                'orbit_track_id': self.orbit_track_id, 'second_track_id': self.second_track_id}

    @staticmethod
    def _mature_observation(history, scan, duration):
        row = (scan['seq'], scan['at_ms'], scan['received_at'])
        if history and row[0] == history[-1][0]:
            return False
        if history and (row[0] <= history[-1][0] or row[1] <= history[-1][1]
                        or row[2] <= history[-1][2]
                        or (row[1]-history[-1][1])/1000 >= SCAN_AGE_S
                        or row[2]-history[-1][2] >= SCAN_AGE_S
                        or abs((row[1]-history[-1][1])/1000-(row[2]-history[-1][2])) > .15):
            history.clear()
        history.append(row)
        del history[:-6]
        return (len(history) >= 3 and (row[1]-history[0][1])/1000+1e-9 >= duration
                and row[2]-history[0][2]+1e-9 >= duration)

    @staticmethod
    def _observation_lease(scan, now, published, received):
        """A missing frame may preserve history, never count or renew a lease."""
        if not (_number(published) and _number(received) and isinstance(scan, dict)
                and _number(scan.get('at_ms')) and _number(scan.get('received_at'))):
            return False
        dt, rt = (scan['at_ms']-published)/1000, scan['received_at']-received
        return (0 <= dt < SCAN_AGE_S and 0 <= rt < SCAN_AGE_S
                and 0 <= now-received < SCAN_AGE_S and abs(dt-rt) <= .15)

    def _target_gap_held(self, tracker, identity, scan, now):
        held = getattr(tracker, 'held', None)
        return (isinstance(held, dict) and held.get('confirmed') is True
                and held.get('track_id') == identity
                and self._observation_lease(scan, now, held.get('last_full_isolation_at_ms'),
                                            held.get('last_full_isolation_received_at')))

    def _clear_target_recovery_history(self):
        # A new ID cannot inherit another identity's readiness or pass window.
        self._quality_good_history.clear()
        self._quality_full_streak = 0
        self._quality_pass_history.clear()
        self._first_pass_history.clear()
        self._second_pass_history.clear()

    def _mature_recovery_observation(self, scan):
        previous = self._quality_good_history[-1][0] if self._quality_good_history else None
        mature = self._mature_observation(self._quality_good_history, scan, .30)
        if scan['seq'] != previous:
            self._quality_full_streak = (1 if len(self._quality_good_history) == 1
                                         else self._quality_full_streak+1)
        # A retained window is not evidence that the missing frame was good.
        # Require two current full frames after any held gap; alternating
        # isolated good/missing frames cannot trigger powered recovery.
        return mature and self._quality_full_streak >= 2

    @staticmethod
    def _consecutive_fresh(history, scan, count=ROUTE_EXIT_PREPARE_FRAMES, max_gap_s=SCAN_AGE_S):
        """Same freshness/ordering resets as _mature_observation, frame count only.

        A repeated scan never counts twice; a gap or clock mismatch restarts.
        """
        row = (scan['seq'], scan['at_ms'], scan['received_at'])
        if history and row[0] == history[-1][0]:
            return len(history) >= count
        if history and (row[0] <= history[-1][0] or row[1] <= history[-1][1]
                        or row[2] <= history[-1][2]
                        or (row[1]-history[-1][1])/1000 >= max_gap_s
                        or row[2]-history[-1][2] >= max_gap_s
                        or abs((row[1]-history[-1][1])/1000-(row[2]-history[-1][2])) > .15):
            history.clear()
        history.append(row)
        del history[:-6]
        return len(history) >= count

    def _route_exit_release_active(self):
        return (self.continue_route and self.handover_observed
                and ((self.phase == 'drive' and (self.first_pass_preparing
                                                 or self.first_pass_evidence is not None))
                     or (self.phase == 'quality_wait' and self.quality_wait_release is not None)))

    @staticmethod
    def _paired_window_corridor(scan):
        """Pair same-frame native wall windows into one forward corridor.

        Only current native windows are used; nothing is cached or extended.
        Gates mirror corridor.rs. Two distinct axes are ambiguous (None, True).
        Returns (candidate | None, ambiguous).
        """
        walls = scan.get('wall_candidates')
        if not isinstance(walls, list) or len(walls) > 64:
            return None, False
        forward = [w for w in walls if _valid_wall(w) and abs(w['heading_left_rad']) < math.pi/2]
        pairs = []
        for left in forward:
            if left['rho_left_m'] <= 0:
                continue
            for right in forward:
                if right['rho_left_m'] >= 0:
                    continue
                difference = _wrap(left['heading_left_rad']-right['heading_left_rad'])
                if abs(difference) > PAIR_MAX_ANGLE_RAD:
                    continue
                heading = _wrap(right['heading_left_rad']+difference/2)
                if abs(heading) >= math.pi/2:
                    continue
                c, s = math.cos(heading), math.sin(heading)
                ends = {}
                for name, wall in (('left', left), ('right', right)):
                    points = [wall['support_start_left_m'], wall['support_end_left_m']]
                    normal = [-s*p['x_m']+c*p['y_m'] for p in points]
                    along = [c*p['x_m']+s*p['y_m'] for p in points]
                    ends[name] = (sum(normal)/2, min(along), max(along))
                width = ends['left'][0]-ends['right'][0]
                shared = min(ends['left'][2], ends['right'][2])-max(ends['left'][1], ends['right'][1])
                if (not ends['left'][0] > 0 > ends['right'][0]
                        or not PAIR_MIN_WIDTH_M <= width <= PAIR_MAX_WIDTH_M
                        or shared < PAIR_MIN_SHARED_SUPPORT_M):
                    continue
                pairs.append({'heading_left_rad': heading,
                    'center_offset_left_m': (ends['left'][0]+ends['right'][0])/2,
                    'width_m': width, 'left_wall_points': left['points'],
                    'right_wall_points': right['points'], 'support_span_m': shared,
                    'fit_error_m': max(left['fit_error_m'], right['fit_error_m']),
                    'origin_between_walls': True, 'candidate_only': True,
                    'turn_path_certified': False})
        if not pairs:
            return None, False
        base = max(pairs, key=lambda p: p['support_span_m'])
        for p in pairs:
            # Several windows of the same two physical walls merge; any other
            # axis/width/offset is a second interpretation, never a choice.
            if (abs(_wrap(p['heading_left_rad']-base['heading_left_rad'])) > PAIR_MERGE_HEADING_RAD
                    or abs(p['width_m']-base['width_m']) > max(.15, MAX_EXIT_WIDTH_CHANGE_FRACTION*base['width_m'])
                    or abs(p['center_offset_left_m']-base['center_offset_left_m']) > PAIR_MERGE_OFFSET_M):
                return None, True
        return {**base, 'source_kind': 'native_wall_window_pair', 'pair_count': len(pairs)}, False

    def _open_loop_orbit(self):
        """Open-course first loop after the handover, before its pass.

        Round 73: the loop ends on its own compass rule; the lane-route pass
        preparation (support at the body front -> release left, latched again
        at a quality resume and stepped on repeated frames) pulled the loop's
        1720 back toward 1500 and the car ran wide into a board.
        """
        return (self.continue_route and self.entry_kind == 'lane' and self.handover_observed
                and self.route_stage == 'first_target' and self.first_pass_evidence is None)

    def _clear_exit_prepare_window(self):
        self._first_exit_prepare_history.clear()
        self._quality_exit_prepare_ready = False

    def _latch_exit_prepare(self, scan, control, *, basis, history=None):
        points = self._full_target_points(self.compact_target, scan, self.orbit_track_id)
        history = self._first_exit_prepare_history if history is None else history
        if points is None or not history:
            return
        self.first_pass_preparing = True
        self.orbit_left_entry_boost = False
        self.first_exit_prepare_evidence = {
            'source_seq': scan['seq'], 'source_at_ms': scan['at_ms'],
            'received_at': scan['received_at'],
            'center_x_m': sum(p[0] for p in points)/len(points),
            'track_id': self.orbit_track_id, 'support_bins': list(self.compact_target['support_bins']),
            'observed_frontmost_x_m': max(p[0] for p in points),
            'observed_left_body_gap_m': min(p[1] for p in points)-SIDE_BODY_EXTENT_M,
            'observation_count': len(history), 'first_source_seq': history[0][0],
            'last_preparation_source_seq': history[-1][0],
            'publication_span_s': (history[-1][1]-history[0][1])/1000,
            'receive_span_s': history[-1][2]-history[0][2],
            'adopted_control': {k: control.get(k) for k in ('motor', 'servo', 'tick', 'seq', 'command_acked')},
            'basis': basis, 'action': 'release_left_toward_neutral_only',
            'observed_rear_pass': False, 'right_steering_authorized': False,
            'physical_cone_pass_certified': False}
        self._route_event('first_exit_prepare', scan, control, self.first_exit_prepare_evidence)

    def begin_quality_wait(self, now, control, *, scan=None):
        """One opt-in neutral wait; this never reopens a terminal coast."""
        def refuse(reason):
            # Trial41: the refusal reason was not recorded; keep it for analysis.
            self.quality_wait_refusal = {'reason': reason, 'at': now if _number(now) else None,
                'control_servo': control.get('servo'), 'command_acked': control.get('command_acked'),
                'servo': self.servo, 'scan_seq': scan.get('seq') if isinstance(scan, dict) else None}
            return False
        if (not self.continue_route or self.phase != 'drive'
                or not self.handover_observed or self.first_pass_evidence is not None
                or self.right_exit_since is not None or self.route_stage != 'first_target'
                or not _number(now) or now < self.last_now):
            return refuse('route_state')
        if (control.get('armed') is not True or control.get('motor') not in DRIVE_MOTORS
                or control.get('command_acked') is not True
                or not NEUTRAL < control.get('servo', 0) == self.servo <= SERVO_MAX):
            return refuse('control_not_acked_left')
        scan = self._current_scan if scan is None else scan
        if (not isinstance(scan, dict) or not _number(scan.get('received_at'))
                or not 0 <= now-scan['received_at'] < SCAN_AGE_S):
            return refuse('scan_stale')
        previous = self.compact_target  # the last frame's target, before this scan
        observed = self.target_tracker.update(scan, now, acquire=True)
        if self._full_target_points(observed, scan, self.orbit_track_id) is not None:
            anchor = {'track_id': observed['track_id'], 'point_left_m': list(observed['point_left_m']),
                      'received_at': scan['received_at'], 'basis': 'trigger_frame_full_isolation'}
        else:
            last = (previous if isinstance(previous, dict) and previous.get('confirmed') is True
                    else self._last_confirmed_compact_target)
            if (isinstance(last, dict) and last.get('track_id') == self.orbit_track_id
                    and _number(last.get('last_full_isolation_received_at'))
                    and 0 <= now-last['last_full_isolation_received_at'] < SCAN_AGE_S):
                anchor = {'track_id': last['track_id'], 'point_left_m': list(last['point_left_m']),
                          'received_at': last['last_full_isolation_received_at'],
                          'basis': 'previous_fresh_full_isolation'}
            else:
                # Trial50: wait without an anchor rather than ending the
                # route; the nearest-left rebind can still recover the support.
                anchor = None
        self.compact_target = observed
        self.neutral_pause_kind = None
        self.recovery_count += 1
        self.quality_wait_anchor = anchor
        self.quality_wait_rebind = None
        self._wait_rebind_candidate = None
        self._current_scan = scan
        self.quality_wait_used = True
        self.quality_wait_since, self.quality_wait_servo = now, self.servo
        self.quality_wait_evidence = {'entry_seq': scan['seq'], 'entry_at_ms': scan['at_ms'],
            'entry_received_at': scan['received_at'], 'held_servo': self.servo,
            'target_id': self.orbit_track_id, 'maximum_resumes': 1}
        self._quality_good_history.clear()
        self._quality_pass_history.clear()
        self._first_pass_history.clear()
        self._clear_exit_prepare_window()
        self._wait_release_history.clear()
        self.quality_wait_previous_servo = None
        # A release decided from healthy drive frames before this wait stays a
        # release-only intention; it never carries observation windows.
        self.quality_wait_release = ({'basis': 'release_latched_before_neutral_wait',
                                      'source_seq': self.first_exit_prepare_evidence.get('source_seq')
                                      if isinstance(self.first_exit_prepare_evidence, dict) else None,
                                      'held_servo_at_entry': self.servo}
                                     if self.first_pass_preparing else None)
        self.first_pass_preparing = False
        self.phase, self.reason = 'quality_wait', 'route_quality_wait_neutral'
        self.steering_target = self.servo
        self._route_event('quality_wait', scan, control, self.quality_wait_evidence)
        return True

    def _nearest_left_rebind(self, scan, control):
        target = self.compact_target
        if (self.route_stage != 'first_target' or self.first_pass_evidence is not None
                or not isinstance(target, dict) or target.get('confirmed') is not True
                or target.get('track_id') in (None, self.orbit_track_id)
                or not _number(target.get('bearing_left_rad')) or not _number(target.get('range_m'))
                or not 0 < target['bearing_left_rad'] < math.pi
                or not target['range_m'] <= ORBIT_REBIND_MAX_RANGE_M):
            return None
        points = self._full_target_points(target, scan, target['track_id'])
        if points is None:
            return None
        try:
            others = _candidates(scan['ranges'], bearing_range_rad=(0., math.pi), tolerant=True)
        except (ValueError, TypeError, KeyError):
            return None
        if any(_number(o.get('range_m')) and o['range_m'] < target['range_m']-.05 for o in others):
            return None
        self.quality_wait_rebind = {'old_track_id': self.orbit_track_id, 'new_track_id': target['track_id'],
            'source_seq': scan['seq'], 'anchor': copy.deepcopy(self.quality_wait_anchor),
            'range_m': target['range_m'], 'bearing_left_rad': target['bearing_left_rad'],
            'basis': 'nearest_confirmed_full_compact_on_left_within_1.2m_during_first_target_wait',
            'physical_identity_verified': False}
        self.orbit_track_id = target['track_id']
        self.target_tracker.enable_close_maintenance(self.orbit_track_id)
        self._clear_target_recovery_history()
        self._route_event('quality_wait_target_rebind', scan, control, self.quality_wait_rebind)
        return points

    def _quality_wait_rebind(self, scan, control):
        target, anchor = self.compact_target, self.quality_wait_anchor
        if (not isinstance(target, dict) or target.get('track_id') == self.orbit_track_id
                or not isinstance(target.get('point_left_m'), list) or not _number(target.get('source_received_at'))):
            return None
        if anchor is None:
            return self._nearest_left_rebind(scan, control)
        if target.get('confirmation_count') == 1:
            if (math.dist(target['point_left_m'], anchor['point_left_m']) <= QUALITY_WAIT_REBIND_M
                    and 0 <= target['source_received_at']-anchor['received_at'] < QUALITY_WAIT_REBIND_WINDOW_S):
                self._wait_rebind_candidate = target['track_id']
            return None
        if target.get('confirmed') is not True or target['track_id'] != self._wait_rebind_candidate:
            return self._nearest_left_rebind(scan, control)
        points = self._full_target_points(target, scan, target['track_id'])
        if points is None:
            return None
        self.quality_wait_rebind = {'old_track_id': self.orbit_track_id, 'new_track_id': target['track_id'],
            'source_seq': scan['seq'], 'anchor': copy.deepcopy(anchor),
            'basis': 'new_identity_first_seen_within_association_gate_of_last_full_orbit_support',
            'physical_identity_verified': False}
        self.orbit_track_id = target['track_id']
        self.target_tracker.enable_close_maintenance(self.orbit_track_id)
        self._clear_target_recovery_history()
        self._route_event('quality_wait_target_rebind', scan, control, self.quality_wait_rebind)
        return points

    def _wait_steer(self, now, servo_feedback, neutral, base):
        """Route wait/pause steering while the car may still be rolling (trial49:
        ~.5 m/s^2 rolling decel; trial50: a held 1710 curled into the support).
        One ACKed step at a time toward base, plus proximity avoidance; the
        motor stays at neutral and every step is recorded for the service."""
        self.steering_target = self.quality_wait_servo
        if not neutral or servo_feedback != self.quality_wait_servo or self.quality_wait_previous_servo is not None:
            return
        lower = RIGHT_EXIT_MIN_PWM if self.neutral_pause_kind in ('post_pass', 'recovery') else NEUTRAL
        before = self.servo
        self.steering_target = max(lower, min(SERVO_MAX, base))
        self._slew(now)
        if not lower <= self.servo <= SERVO_MAX:
            self.servo = before
        if self.servo != before:
            self.quality_wait_previous_servo = before
            self.quality_wait_servo = self.servo
            self.wait_steering_changes += 1
        self.steering_target = self.quality_wait_servo

    def _exit_release_target(self, scan, now):
        """Release-only target once the first support is at the body front.

        Round 53 (trial49): centering at heading ~145 deg of the first U-turn
        drove the car into the outer board while the exit corridor axis was
        still well to the left. Route mode releases toward the measured exit
        corridor pursuit target instead, never right of center and never above
        the current command; center when no corridor is measured.
        """
        if not self.continue_route or not isinstance(scan, dict):
            return NEUTRAL
        cached = self._exit_release_cache
        if cached is not None and cached[0] == scan.get('seq'):
            return max(NEUTRAL, min(self.servo, cached[1]))
        # Measure only: the corridor identity used after the first pass must
        # start fresh exactly as before (round-53 replay of trial52: a U-turn
        # corridor kept as identity was rejected at the first pass -> pause).
        saved = (self.right_exit_geometry, self._right_exit_last_receive, self._right_exit_last_publication)
        target, rejection = self._right_exit_target(scan, now)
        self.right_exit_geometry, self._right_exit_last_receive, self._right_exit_last_publication = saved
        value = NEUTRAL if rejection is not None or not _number(target) else max(NEUTRAL, int(target))
        self._exit_release_cache = (scan.get('seq'), value)
        return max(NEUTRAL, min(self.servo, value))

    def _wait_inside_base(self, lookahead):
        """Held left, released only for a close inside return (no creep)."""
        held = self.quality_wait_servo
        self._limit_left_for_known_points(held, lookahead)
        demand = (self.inner_clearance or {}).get('clearance_bearing_demand_pwm')
        if not _number(demand):
            return held
        return min(held, round(demand)+ROUTE_WAIT_INSIDE_MARGIN_PWM)

    def _quality_wait_update(self, scan, now, control, advancing, quality_clear, resume_ready):
        self.steering_target = self.quality_wait_servo
        # Trial50: the orbit-entry (3 s) and wait (5 s) clocks locked a car that
        # was stopped well placed beside the support. Only the cumulative drive
        # budget (and the service's total deadline) bound a route wait now.
        if now-self.drive_since >= self.max_drive_s:
            self.lock('route_quality_wait_original_budget_expired')
            return self._result(now)
        servo_feedback = control.get('servo')
        if servo_feedback == self.quality_wait_servo:
            self.quality_wait_previous_servo = None
        # Only the immediately preceding release step may still be reported
        # while its command is not yet ACKed; nothing else may change steering.
        pending_release = (self.quality_wait_previous_servo is not None
                           and servo_feedback == self.quality_wait_previous_servo
                           and control.get('command_acked') is False)
        if ((servo_feedback != self.quality_wait_servo and not pending_release)
                or control.get('motor') not in (NEUTRAL,)+DRIVE_MOTORS
                or (control.get('neutral_acked') is True and control['motor'] != NEUTRAL)):
            self.lock('route_quality_wait_feedback_changed')
            return self._result(now)
        neutral = control.get('neutral_acked') is True and control['motor'] == NEUTRAL
        if not neutral and now-self.quality_wait_since >= SCAN_AGE_S:
            self.lock('route_quality_wait_neutral_ack_timeout')
            return self._result(now)
        points = self._full_target_points(self.compact_target, scan, self.orbit_track_id)
        if points is None:
            points = self._quality_wait_rebind(scan, control)
        if points is None:
            # Missing/partial support: power never resumes without the
            # confirmed, fully isolated orbit support. While still rolling,
            # steering may only release left for close inside returns or move
            # away from close known returns.
            self._clear_exit_prepare_window()
            self._quality_full_streak = 0
            if not (quality_clear is True and neutral
                    and self._target_gap_held(self.target_tracker, self.orbit_track_id, scan, now)):
                self._quality_good_history.clear()
                self._quality_pass_history.clear()
            # Round 54 (trial53): a support last fully seen at/behind the body
            # front (or a release latched before the wait) keeps releasing
            # toward the exit while the car coasts; otherwise hold, releasing
            # only for close inside returns.
            anchor = self.quality_wait_anchor
            anchored_front = (isinstance(anchor, dict) and isinstance(anchor.get('point_left_m'), list)
                              and anchor['point_left_m'][0] <= FRONT_BODY_EXTENT_M)
            base = (self._exit_release_target(scan, now)
                    if (self.quality_wait_release is not None or anchored_front) and not self._open_loop_orbit()
                    else self._wait_inside_base(1.))
            self._wait_steer(now, servo_feedback, neutral, base)
            self.reason = 'route_quality_wait_target_reacquiring'
            return self._result(now)
        self.quality_wait_anchor = {'track_id': self.orbit_track_id,
                                    'point_left_m': list(self.compact_target['point_left_m']),
                                    'received_at': scan['received_at'], 'basis': 'wait_frame_full_isolation'}
        if quality_clear is not True or not neutral:
            self._clear_exit_prepare_window()
            self._quality_good_history.clear()
        front, side = max(p[0] for p in points), min(p[1] for p in points)-SIDE_BODY_EXTENT_M
        # Release-only decision in the neutral wait. Degraded scan quality does
        # not hide this complete, isolated same-ID support; it never advances a
        # pass, authorizes right steering or restores motor power.
        if advancing and neutral and self.quality_wait_release is None and not self._open_loop_orbit():
            if front <= FRONT_BODY_EXTENT_M and side >= MANEUVER_BODY_CLEARANCE_M:
                if self._consecutive_fresh(self._wait_release_history, scan):
                    self.quality_wait_release = {'basis': 'neutral_wait_current_support_at_body_front',
                        'source_seq': scan['seq'], 'source_at_ms': scan['at_ms'],
                        'received_at': scan['received_at'], 'track_id': self.orbit_track_id,
                        'observed_frontmost_x_m': front, 'observed_left_body_gap_m': side,
                        'observation_count': len(self._wait_release_history),
                        'first_source_seq': self._wait_release_history[0][0],
                        'held_servo_at_decision': self.quality_wait_servo,
                        'action': 'release_left_toward_neutral_only',
                        'right_steering_authorized': False, 'physical_cone_pass_certified': False}
                    self._route_event('quality_wait_release', scan, control, self.quality_wait_release)
            else:
                self._wait_release_history.clear()
        def hold():
            # A steering step is issued only on ticks that do not request the
            # powered resume, so that output always holds an ACKed command.
            if self.phase == 'quality_wait':
                before = self.servo
                base = (self._exit_release_target(scan, now) if self.quality_wait_release is not None
                        else self._wait_inside_base(max(.35, min(1., self.compact_target['range_m']))))
                self._wait_steer(now, servo_feedback, neutral, base)
                if self.servo != before and self.quality_wait_release is not None and self.servo < before:
                    self.reason = 'route_quality_wait_neutral_release_left'
            return self._result(now)
        if not advancing:
            return hold()
        passed = front <= -REAR_BODY_EXTENT_M and side >= MANEUVER_BODY_CLEARANCE_M
        if passed:
            pass_ready = self._mature_observation(self._quality_pass_history, scan, .25)
        else:
            self._quality_pass_history.clear()
            pass_ready = False
        if quality_clear is not True or not neutral:
            return hold()
        # Rebuild only from NEW, fully healthy observations in this neutral
        # wait. No pre-wait window or degraded frame can mature the intention.
        if front <= FRONT_BODY_EXTENT_M and side >= MANEUVER_BODY_CLEARANCE_M:
            self._quality_exit_prepare_ready = self._mature_observation(
                self._first_exit_prepare_history, scan, .25)
        else:
            self._clear_exit_prepare_window()
        good = self._mature_recovery_observation(scan)
        if (not good or resume_ready is not True
                or servo_feedback != self.quality_wait_servo or self.quality_wait_previous_servo is not None):
            # Resume only on an ACKed, unchanged neutral-wait steering command.
            return hold()
        if passed:
            if not pass_ready:
                return hold()
            _, rejection = self._right_exit_target(scan, now)
            if rejection is not None:
                # The target pass is current, but it does not manufacture an
                # exit corridor. Keep neutral and retry from fresh geometry.
                self.right_exit_geometry = None
                self._right_exit_last_receive = self._right_exit_last_publication = None
                self.quality_wait_evidence['last_corridor_rejection'] = rejection
                hold()
                self.reason = 'route_quality_wait_pass_corridor_unavailable'
                return self._result(now)
            self.first_pass_preparing = True
            self.first_pass_evidence = {'source_seq': scan['seq'], 'source_at_ms': scan['at_ms'],
                'received_at': scan['received_at'], 'track_id': self.orbit_track_id,
                'support_bins': list(self.compact_target['support_bins']),
                'observed_frontmost_x_m': front, 'observed_left_body_gap_m': side,
                'basis': 'current_complete_support_behind_rear_during_neutral_wait',
                'physical_cone_pass_certified': False, 'swept_path_certified': False}
            self.right_exit_since, self.route_stage = now, 'right_align'
            self._route_event('first_pass', scan, control, self.first_pass_evidence)
        self.phase, self.reason = 'drive', 'route_quality_resume_waiting_ack'
        self.quality_resume_pending, self.quality_resume_since = True, now
        self.quality_wait_evidence.update(resume_seq=scan['seq'], resume_at_ms=scan['at_ms'],
            resume_received_at=scan['received_at'], good_frame_count=len(self._quality_good_history))
        self._first_pass_history.clear()
        if not self._quality_exit_prepare_ready:
            self._clear_exit_prepare_window()
        self._route_event('quality_resume_request', scan, control, self.quality_wait_evidence)
        # First powered output holds the actually ACKed neutral-wait steering.
        return self._result(now)

    def _measurement(self, scan, candidates, publication_dt):
        candidate = super()._measurement(scan, candidates, publication_dt)
        if self.geometry_mode == 'opening_wall':
            self.entry_bearing_required = True
        return candidate

    def _endpoint_observation(self, scan):
        super()._endpoint_observation(scan)
        if not self.entry_bearing_required or self.entry_bearing_released:
            return
        # Called only after native geometry and its same-scan ray binding have
        # been validated. The cached turn_goal may contain an OLD endpoint.
        opening = scan.get('left_turn_goal')
        observation = ({'endpoint_return': opening['incoming_left_end_support'],
                        'heading_left_rad': opening['incoming_heading_left_rad'],
                        'observation_type': 'full_opening', 'wall': None}
                       if opening is not None else self._current_incoming_endpoint(scan))
        if observation is None:
            self.entry_bearing = None
            self._entry_ready_count = 0
            self._entry_ready_receive = self._entry_ready_publication = None
            self._entry_ready_observation_type = None
            self._entry_ready_history.clear()
            return
        point = observation['endpoint_return']['point_left_m']
        heading = observation['heading_left_rad']
        c, s = math.cos(heading), math.sin(heading)
        # This separate descriptor associates the next CURRENT observation.
        # It never supplies a missing point, heading, opening width or outer wall.
        received = scan.get('received_at')
        prior = self._incoming_association
        if (_number(received) and received >= 0 and 0 <= self.last_now-received < SCAN_AGE_S
                and (prior is None or (scan['seq'] > prior['source_seq']
                    and scan['at_ms'] > prior['source_at_ms']
                    and received > prior['source_received_at']))):
            self._incoming_association = {'source_seq': scan['seq'], 'source_at_ms': scan['at_ms'],
                'source_received_at': received, 'heading_left_rad': heading,
                'rho_left_m': -s*point['x_m']+c*point['y_m'],
                'endpoint_return': copy.deepcopy(observation['endpoint_return'])}
        corners = [(x, y) for x in (-REAR_BODY_EXTENT_M, FRONT_BODY_EXTENT_M)
                   for y in (-SIDE_BODY_EXTENT_M, SIDE_BODY_EXTENT_M)]
        # Project the measured body, with the existing 8 cm allowance, onto
        # the current incoming-wall axes. Neither distance is a course length.
        front = max(c*x+s*y for x, y in corners)+MANEUVER_BODY_CLEARANCE_M
        side = max(-s*x+c*y for x, y in corners)+MANEUVER_BODY_CLEARANCE_M
        forward_gap = c*point['x_m']+s*point['y_m']-front
        lateral_gap = -s*point['x_m']+c*point['y_m']-side
        bearing = heading+math.atan2(max(0., lateral_gap), max(0., forward_gap))
        cap = max(NEUTRAL, min(SERVO_MAX, NEUTRAL+round(LEFT_TRIAL_GAIN*bearing)))
        if lateral_gap <= 0:
            cap = NEUTRAL
        # The native full-junction candidate can disappear as the car rotates.
        # Trial25 already put the CURRENT endpoint at the inflated front
        # projection, but a second 250 ms maturity wait delayed the handoff
        # until that candidate was gone. This direct same-frame geometry can
        # end the early cap; it does not assert rear/body or path clearance.
        front_projection_reached = forward_gap <= 0 and lateral_gap > 0 and cap > NEUTRAL
        permits_prepared_left = cap >= self.initial_presteer_pwm
        observation_clock = (received if observation['observation_type'] == 'tracked_current_incoming_wall'
                             else self.last_now)
        # Keep the existing full-opening maturity clock. A change of source
        # starts a new maturity interval rather than mixing decision/receipt time.
        if self._entry_ready_observation_type != observation['observation_type']:
            self._entry_ready_count = 0
            self._entry_ready_receive = self._entry_ready_publication = None
            self._entry_ready_history.clear()
        self._entry_ready_observation_type = observation['observation_type']
        if permits_prepared_left:
            if self._entry_ready_count == 0:
                self._entry_ready_receive = observation_clock
                self._entry_ready_publication = scan['at_ms']
            self._entry_ready_count += 1
            if (self._entry_ready_count >= 3
                    and observation_clock-self._entry_ready_receive >= .25
                    and (scan['at_ms']-self._entry_ready_publication)/1000 >= .25):
                self.entry_bearing_released = True
            # Route mode: the same permitting bearing in two strictly fresh,
            # consistently clocked frames; a repeat, gap or clock skew restarts.
            if (self.continue_route and _number(received)
                    and self._consecutive_fresh(self._entry_ready_history, scan,
                                                ROUTE_ENTRY_RELEASE_FRAMES)):
                self.entry_bearing_released = True
        else:
            self._entry_ready_count = 0
            self._entry_ready_receive = self._entry_ready_publication = None
            self._entry_ready_history.clear()
        if front_projection_reached:
            self.entry_bearing_released = True
        self._entry_last_receive, self._entry_last_publication = observation_clock, scan['at_ms']
        self.entry_bearing = {'source_seq': scan['seq'], 'source_at_ms': scan['at_ms'],
            'source_received_at': received,
            'endpoint_return': copy.deepcopy(observation['endpoint_return']),
            'observation_type': observation['observation_type'],
            'current_incoming_wall': copy.deepcopy(observation['wall']),
            'current_near_support_bins': list(observation.get('near_support_bins', [])),
            'adjacent_far_return': copy.deepcopy(observation.get('adjacent_far_return')),
            'incoming_heading_left_rad': heading, 'body_forward_support_m': front,
            'body_left_support_m': side, 'forward_gap_m': forward_gap,
            'lateral_gap_m': lateral_gap, 'clearance_bearing_left_rad': bearing,
            'steering_cap_pwm': cap, 'prepared_left_pwm': self.initial_presteer_pwm,
            'confirmation_count': self._entry_ready_count,
            'released': self.entry_bearing_released,
            'release_basis': ('current_endpoint_at_body_front_projection' if front_projection_reached
                              else 'prepared_left_bearing_matured' if self.entry_bearing_released else None),
            'route_fresh_frames': len(self._entry_ready_history) if self.continue_route else None,
            'scope': 'early_turn_endpoint_bearing_heuristic',
            'endpoint_passed_proven': False, 'swept_path_certified': False,
            'physical_curvature_calibrated': False}

    def _current_incoming_endpoint(self, scan):
        """Associate a currently observed wall end after a full opening was seen.

        A native fit may stop at a sampling window. Its forward end must bind
        to a current return, followed immediately by a real farther return,
        and retain a contiguous, actually observed inlier chain behind it.
        """
        old = self._incoming_association
        if self.phase != 'drive' or old is None:
            return None
        publication_dt = (scan['at_ms']-old['source_at_ms'])/1000
        received = scan.get('received_at')
        if (scan['seq'] <= old['source_seq'] or not 0 < publication_dt < SCAN_AGE_S
                or not _number(received) or received < 0 or not 0 <= self.last_now-received < SCAN_AGE_S
                or received <= old['source_received_at']
                or not 0 <= self.last_now-old['source_received_at'] < SCAN_AGE_S
                or abs(publication_dt-(received-old['source_received_at'])) > .15):
            return None
        ranges, walls = scan.get('ranges'), scan.get('wall_candidates')
        if (not isinstance(ranges, list) or len(ranges) != 360
                or any(r is not None and (not _number(r) or not .02 <= r <= 12) for r in ranges)
                or not isinstance(walls, list) or len(walls) > 64
                or any(not _valid_wall(w) for w in walls)):
            return None
        gate = min(math.radians(30), .12+math.radians(90)*publication_dt)
        previous = old['endpoint_return']['point_left_m']
        matches = []
        for original in walls:
            wall = copy.deepcopy(original)
            heading = wall['heading_left_rad']
            if abs(_wrap(heading-old['heading_left_rad'])) > math.pi/2:
                heading = wall['heading_left_rad'] = _wrap(heading+math.pi)
                wall['rho_left_m'] = -wall['rho_left_m']
                wall['support_start_left_m'], wall['support_end_left_m'] = (
                    wall['support_end_left_m'], wall['support_start_left_m'])
            end = wall['support_end_left_m']
            if (abs(_wrap(heading-old['heading_left_rad'])) > gate or wall['rho_left_m'] <= 0
                    or abs(wall['rho_left_m']-old['rho_left_m']) > .30
                    or math.hypot(end['x_m']-previous['x_m'], end['y_m']-previous['y_m']) > .30):
                continue
            observation = self._observed_wall_endpoint(wall, ranges)
            if observation is not None and not any(wall == item['wall'] for item in matches):
                matches.append(observation)
        if not matches:
            return None
        supports = _surface_supports(scan, [item['wall'] for item in matches])
        groups = []
        for observation, support in zip(matches, supports):
            if support is None:
                continue
            # Merge only aliases of the same real endpoint AND current surface.
            # Similar headings alone never merge distinct parallel boards.
            group = next((g for g in groups if all(
                observation['endpoint_return']['index'] == prior['endpoint_return']['index']
                and _same_surface(support, evidence) is True for prior, evidence in g)), None)
            if group is None:
                groups.append([(observation, support)])
            else:
                group.append((observation, support))
        return groups[0][0][0] if len(groups) == 1 else None

    @staticmethod
    def _observed_wall_endpoint(wall, ranges):
        heading, end = wall['heading_left_rad'], wall['support_end_left_m']
        c, s = math.cos(heading), math.sin(heading)
        along = lambda p: c*p['x_m']+s*p['y_m']
        normal = lambda p: -s*p['x_m']+c*p['y_m']
        low, high = along(wall['support_start_left_m']), along(end)
        if low >= high or abs(normal(end)-wall['rho_left_m']) > .05:
            return None
        index = round(-math.degrees(math.atan2(end['y_m'], end['x_m']))) % 360
        distance = ranges[index]
        if distance is None:
            return None
        point = lambda i: {'x_m': ranges[i]*math.cos(math.radians(i)),
                           'y_m': -ranges[i]*math.sin(math.radians(i))}
        measured = point(index)
        if math.hypot(measured['x_m']-end['x_m'], measured['y_m']-end['y_m']) > 1e-6:
            return None
        for direction in (-1, 1):
            outside = (index+direction) % 360
            far = ranges[outside]
            if far is None or far <= distance+.18:
                continue
            # Require the adjacent beam to continue forward along this wall's
            # orientation, independently of its much farther observed range.
            adjacent = {'x_m': distance*math.cos(math.radians(outside)),
                        'y_m': -distance*math.sin(math.radians(outside))}
            if along(adjacent) <= high or along(point(outside)) <= high:
                continue
            previous = measured
            support_bins = [index]
            # Match the native wall's minimum actual support count at THIS end;
            # a separate mature patch elsewhere on the line is insufficient.
            for step in range(1, 16):
                inside = (index-direction*step) % 360
                if ranges[inside] is None:
                    break
                current = point(inside)
                if (not low-1e-6 <= along(current) < high
                        or abs(normal(current)-wall['rho_left_m']) > .05
                        or math.hypot(current['x_m']-previous['x_m'], current['y_m']-previous['y_m']) > .18):
                    break
                support_bins.append(inside)
                previous = current
            if len(support_bins) == 16:
                return {'endpoint_return': {'index': index,
                            'angle_left_rad': _wrap(-math.radians(index)),
                            'range_m': distance, 'point_left_m': measured},
                        'heading_left_rad': heading,
                        'observation_type': 'tracked_current_incoming_wall', 'wall': wall,
                        'near_support_bins': support_bins,
                        'adjacent_far_return': {'index': outside, 'range_m': far}}
        return None

    def _entry_target(self, target):
        if not self.entry_bearing_required or self.entry_bearing_released:
            return target
        if self.entry_bearing is not None:
            return min(target, self.entry_bearing['steering_cap_pwm'])
        if self.phase == 'presteer':
            return NEUTRAL  # Missing current endpoint cannot mature adoption.
        # A brief missing endpoint only holds the already commanded steering;
        # it cannot authorize more left, target handover or a renewed lease.
        if (self._entry_last_receive is not None
                and self.last_now-self._entry_last_receive < SCAN_AGE_S
                and (self.last_publication-self._entry_last_publication)/1000 < SCAN_AGE_S):
            return self.servo
        self.begin_coast('left_turn_entry_endpoint_lost', self.last_now)
        return NEUTRAL

    def _presteer_target(self, target):
        return self._entry_target(super()._presteer_target(target))

    def begin_quality_coast(self, now, control):
        return self._begin_adopted_left_coast('turn_perception_unavailable', now, control)

    def begin_left_turn_quality_pause(self, now, control, *, scan=None):
        """Explicit route only: one bounded neutral pause before target handover.

        Trial44: one front_sparse frame during the left turn ended the run in
        the terminal held coast .16 s before the first target confirmed; the
        next frame was clean. Hold the ACKed left with neutral motor instead.
        Power returns only after clean quality matures over .30 s together
        with a current left-turn geometry measurement. Once per run; the
        original drive budget keeps running; nothing here steers.
        """
        scan = self._current_scan if scan is None else scan
        if (not self.continue_route or self.phase != 'drive'
                or self.handover_observed or self.route_stage != 'first_target'
                or self.first_pass_evidence is not None or self.right_exit_since is not None
                or self.quality_resume_pending
                or not _number(now) or now < self.last_now
                or control.get('armed') is not True or control.get('motor') != TRIAL_MOTOR
                or control.get('command_acked') is not True
                or not NEUTRAL < control.get('servo', 0) == self.servo <= SERVO_MAX
                or not isinstance(scan, dict) or not _number(scan.get('received_at'))
                or not 0 <= now-scan['received_at'] < SCAN_AGE_S):
            return False
        self.left_turn_pause_used = True
        self.left_turn_pause_geometry_error = None
        self._enter_neutral_pause('left_turn', 'route_left_turn_quality_pause_neutral', now, scan, control,
                                  'quality', 'clean quality over .30 s and current left-turn geometry')
        return True

    def begin_post_pass_quality_pause(self, now, control, *, scan=None):
        """Explicit route after the first pass: quality faults pause, not halt."""
        scan = self._current_scan if scan is None else scan
        if (not self.continue_route or self.phase != 'drive' or self.first_pass_evidence is None
                or self.route_stage not in POST_PASS_STAGES or self.quality_resume_pending
                or not _number(now) or now < self.last_now
                or control.get('armed') is not True
                or control.get('motor') not in ((NEUTRAL,)+DRIVE_MOTORS if self.route_stage == 'light_lane'
                                                else DRIVE_MOTORS)
                or control.get('command_acked') is not True
                or not RIGHT_EXIT_MIN_PWM <= control.get('servo', 0) == self.servo <= SERVO_MAX
                or not isinstance(scan, dict) or not _number(scan.get('received_at'))
                or not 0 <= now-scan['received_at'] < SCAN_AGE_S):
            return False
        self._enter_neutral_pause('post_pass', 'route_post_pass_quality_pause_neutral', now, scan, control,
                                  'quality', 'clean quality over .30 s and the stage input observed again')
        return True

    def _begin_recovery_hold(self, cause, now, control):
        """Explicit route after the first pass: a recoverable stop holds at
        neutral and keeps re-checking whether the stage can resume. Anything
        else (or an un-ACKed command) keeps the previous coast."""
        scan = self._current_scan
        if (self.continue_route and self.phase == 'drive' and self.first_pass_evidence is not None
                and self.route_stage in POST_PASS_STAGES and not self.quality_resume_pending
                and control.get('armed') is True and control.get('command_acked') is True
                and control.get('servo') == self.servo and RIGHT_EXIT_MIN_PWM <= self.servo <= SERVO_MAX
                and isinstance(scan, dict) and _number(scan.get('received_at'))
                and 0 <= self.last_now-scan['received_at'] < SCAN_AGE_S):
            # The corridor identity restarts from the next fresh observation.
            self.right_exit_geometry = None
            self._right_exit_last_receive = self._right_exit_last_publication = None
            self._route_alignment_history.clear()
            self._enter_neutral_pause('recovery', 'route_recovery_hold_neutral', now, scan, control,
                                      cause, 'clean quality over .30 s and the stage input observed again')
            return True
        self.begin_coast(cause, now)
        return False

    def _enter_neutral_pause(self, kind, reason, now, scan, control, cause, resume_requires):
        self.neutral_pause_kind = kind
        # Round 81 (car run 80): a light-entry hold steps toward a current
        # target while the car still rolls (holding the second loop's full
        # right turned it ~70 deg past the lane); round 84: the lane axis.
        self._pause_steer_target = None
        self.recovery_count += 1
        self.quality_wait_since, self.quality_wait_servo = now, self.servo
        self.quality_wait_previous_servo = None
        self.quality_wait_release = None
        self._quality_good_history.clear()
        evidence = {'kind': kind, 'cause': cause, 'stage': self.route_stage, 'entry_seq': scan['seq'],
                    'entry_at_ms': scan['at_ms'], 'entry_received_at': scan['received_at'],
                    'held_servo': self.servo, 'recovery_count': self.recovery_count,
                    'resume_requires': resume_requires}
        if kind == 'left_turn':
            self.left_turn_pause_evidence = evidence
        elif kind == 'post_pass':
            self.post_pass_pause_evidence = evidence
        self.recovery_evidence = evidence
        self.phase, self.reason = 'quality_wait', reason
        self.steering_target = self.servo
        self._route_event('neutral_pause_'+kind, scan, control, evidence)

    def _left_turn_pause_geometry(self, scan, now):
        """Refresh left-turn geometry from this real scan; never steers."""
        candidates = scan.get('corridor_candidates')
        if (not isinstance(candidates, list) or len(candidates) > 24
                or any(not _valid_candidate(c) for c in candidates)):
            self.lock('invalid_native_corridor_candidates')
            return False
        self.incoming_endpoint = None
        geometry_dt = (0 if self.last_geometry_publication is None
                       else (scan['at_ms']-self.last_geometry_publication)/1000)
        try:
            candidate = self._measurement(scan, candidates, geometry_dt)
        except ValueError as error:
            self.left_turn_pause_geometry_error = str(error)
            return False
        # Same bookkeeping as a successful TurnMotion measurement.
        self.last_geometry_receive, self.last_geometry_publication, self.last_geometry_seq = (
            now, scan['at_ms'], scan['seq'])
        self.geometry_missing = False
        self.corridor, self.selected, self.start_ready = candidate, True, True
        self._endpoint_observation(scan)
        self.left_turn_pause_geometry_error = None
        return True

    def _pause_stage_ready(self, scan, now):
        """Whether the paused task can be executed again from this frame."""
        kind, stage = self.neutral_pause_kind, self.route_stage
        self._pause_stage_held = False
        if kind == 'left_turn':
            return self._left_turn_pause_geometry(scan, now)
        if (stage == 'corridor_follow' and self.second_target_handover_observed
                and self._full_target_points(self.second_target, scan, self.second_track_id) is not None):
            # A drive loss can return to corridor following. Its recovery uses
            # the same role proof as an orbit pause, including right-rear support.
            self.route_stage = stage = 'second_orbit'
            self.second_orbit_since = now
        if stage in ('right_align', 'corridor_follow'):
            _, rejection = self._right_exit_target(scan, now)
            if rejection is not None:
                self._pause_stage_held = (
                    rejection in ('right_exit_corridor_missing', 'right_exit_corridor_ambiguous')
                    and self._observation_lease(scan, now, self._right_exit_last_publication,
                                                self._right_exit_last_receive))
                if not self._pause_stage_held:
                    self.right_exit_geometry = None  # restart identity after lease expiry
                self.recovery_evidence['last_readiness_rejection'] = rejection
                return False
            # The service body gate (8 cm, raw returns) still stops a real
            # contact; a model-margin deficit alone must not keep a stopped car
            # waiting forever. Resume once the corridor model does not overlap.
            return self._corridor_body_gap(self.right_exit_geometry) >= 0.
        if stage in ('open_transfer', 'light_entry'):
            # Resume only where the stage would not stop again at once: each
            # failed resume is a 1560 ACK pulse with the held command, which
            # crept the car round in circles (simulation).
            heading = self.compass.current(scan)
            if heading is None:
                return False
            ahead, _, stop_at = self._open_front_stop(scan)
            board_ahead = ahead is not None and ahead <= stop_at
            if stage == 'light_entry':
                pursuit = self._open_light_pursuit(scan, heading)
                if pursuit is None:
                    return False
                lane, point, _, _ = pursuit
                # Round 84 (car run 83): while the car still rolls, keep it
                # parallel to the lane axis (the light ahead must be faced,
                # and the boards are near); steering toward the pursuit point
                # turned the coasting car from 9 to 15 deg left into the lane.
                self._pause_steer_target = self._light_entry_pause_hold(scan)
                hit = self._open_entry_path_hit(scan, point, heading, stop_at, ahead)
                # Round 102: also where the drive would take a free parallel line.
                return ((hit is None or hit > stop_at)
                        or self._open_light_lane_entry(scan, heading, lane) is not None
                        or self._open_entry_shift(scan, heading, lane, None, stop_at, ahead, hit) is not None)
            if stage == 'open_transfer' and self.second_track_id is None:
                # Round 77: the hold waited for a confirmed second support but
                # never observed it (the tracker froze at its first sighting).
                self._open_second_observation(scan, now, heading)
            second = self._open_confirmed_second(scan) if stage == 'open_transfer' else None
            if second is not None:
                # Round 83: plan the second loop while stopped, so the resume
                # tick reuses it (car run 82: resume + first plan .111 s).
                self._open_loop_geometry(scan, second, second['track_id'], -1, math.pi)
            return not board_ahead or second is not None
        if stage == 'second_loop':
            target = self.second_target
            self._pause_stage_held = self._target_gap_held(self.second_tracker, self.second_track_id, scan, now)
            return (isinstance(target, dict) and target.get('confirmed') is True
                    and target.get('track_id') == self.second_track_id
                    and self._full_target_points(target, scan, self.second_track_id) is not None
                    and self.compass.current(scan) is not None)
        if stage == 'light_turn':
            return self._free_space(scan, self._light_heading) is not None
        if stage == 'light_lane':
            return (self._current_lane(scan, now)[1] is not None
                    or self._free_space(scan, self._light_heading) is not None)
        if stage == 'second_orbit':
            target = self.second_target
            self._pause_stage_held = self._target_gap_held(self.second_tracker, self.second_track_id, scan, now)
            return (isinstance(target, dict) and target.get('confirmed') is True
                    and target.get('track_id') == self.second_track_id
                    and self._full_target_points(target, scan, self.second_track_id) is not None)
        return False

    def _pause_wait_base(self, kind):
        """Where the wait steering steps while the paused car may still roll."""
        if self.route_stage == 'second_orbit' and self.second_return_preparing:
            return min(self.quality_wait_servo, NEUTRAL)
        if self.route_stage == 'light_entry' and self._pause_steer_target is not None:
            # Lane-axis heading hold (rounds 81/84/85); round 91: any pause kind.
            return self._pause_steer_target
        return self.quality_wait_servo

    def _light_entry_pause_hold(self, scan):
        """Lane-axis heading hold for a light-entry pause, from the response pose.

        Round 91 (car runs 88/90): the paused car rolled 1.1-1.65 m at 1-1.9
        m/s; the plain hold swung the heading from +6.6 to -16 deg (88: to -21)
        and a quality pause in the same stage kept 1580 (90: 30 deg left at
        the lane start, left-front contact). As in the open pursuits (round
        79), the hold of the predicted heading is used when it already lies
        at centre or on the other side (_overshoot_guard).
        """
        heading = self.compass.current(scan)
        if heading is None:
            return None
        pose = self._response_pose()
        return _overshoot_guard(self._open_heading_hold(heading, 0.),
                                self._open_heading_hold(heading+pose[2], 0.))

    def _neutral_pause_update(self, scan, lidar_age_s, now, control, *, safe, presteer_wait,
                              quality_clear, resume_ready):
        advancing = self._orbit_inputs(scan, lidar_age_s, now, control, safe, presteer_wait)
        if self.phase == 'locked':
            return self._result(self.last_now)
        self.steering_target = self.quality_wait_servo
        kind = self.neutral_pause_kind
        # Trial50: only the cumulative drive budget (and the service's total
        # deadline) bound a route pause; it keeps re-checking until then.
        if now-self.drive_since >= self.max_drive_s:
            self.lock(f'route_{kind}_pause_original_budget_expired')
            return self._result(now)
        servo_feedback = control.get('servo')
        if servo_feedback == self.quality_wait_servo:
            self.quality_wait_previous_servo = None
        pending_step = (self.quality_wait_previous_servo is not None
                        and servo_feedback == self.quality_wait_previous_servo
                        and control.get('command_acked') is False)
        if ((servo_feedback != self.quality_wait_servo and not pending_step)
                or control.get('motor') not in (NEUTRAL,)+DRIVE_MOTORS):
            self.lock(f'route_{kind}_pause_feedback_changed')
            return self._result(now)
        if self.quality_resume_pending:
            if (control.get('resume_acked') is True and control['motor'] == TRIAL_MOTOR
                    and control.get('command_acked') is True):
                self.quality_resume_pending = False
                self.recovery_evidence['resume_ack'] = {k: control.get(k) for k in ('motor', 'servo', 'seq', 'tick')}
                # Resume the same task with fresh stage clocks; the total
                # deadline and cumulative drive budget are never renewed.
                if self.route_stage == 'right_align' and kind != 'left_turn':
                    self.right_exit_since = now
                if self.route_stage == 'light_turn':
                    self.light_since = now
                self.neutral_pause_kind = None
            elif now-self.quality_resume_since >= SCAN_AGE_S:
                self.lock(f'route_{kind}_pause_resume_ack_timeout')
            # Even the ACK observation keeps the held command on this output.
            return self._result(now)
        if control.get('neutral_acked') is True and control['motor'] != NEUTRAL:
            self.lock(f'route_{kind}_pause_feedback_changed')
            return self._result(now)
        neutral = control.get('neutral_acked') is True and control['motor'] == NEUTRAL
        if not neutral and now-self.quality_wait_since >= SCAN_AGE_S:
            self.lock(f'route_{kind}_pause_neutral_ack_timeout')
            return self._result(now)
        if not advancing:
            return self._result(now)
        if self.route_stage == 'light_entry':
            # Round 91: every pause kind in this stage, each new scan, whether
            # or not the stage inputs are ready (quality pauses held 1580).
            hold = self._light_entry_pause_hold(scan)
            if hold is not None:
                self._pause_steer_target = hold
        stage_ready = self._pause_stage_ready(scan, now) if quality_clear is True and neutral else False
        if quality_clear is not True or not neutral or not stage_ready:
            self._quality_full_streak = 0
            if not (quality_clear is True and neutral and self._pause_stage_held):
                self._quality_good_history.clear()
            self._wait_steer(now, servo_feedback, neutral, self._pause_wait_base(kind))
            return self._result(now)
        if (not self._mature_recovery_observation(scan) or resume_ready is not True
                or servo_feedback != self.quality_wait_servo or self.quality_wait_previous_servo is not None):
            # Round 85 (car run 84): this branch too (it held 1397 while the
            # car turned 23 -> -8 deg past the light-lane axis).
            self._wait_steer(now, servo_feedback, neutral, self._pause_wait_base(kind))
            return self._result(now)
        self.phase, self.reason = 'drive', f'route_{kind}_pause_resume_waiting_ack'
        self.quality_resume_pending, self.quality_resume_since = True, now
        self.recovery_evidence.update(resume_seq=scan['seq'], resume_at_ms=scan['at_ms'],
            resume_received_at=scan['received_at'], good_frame_count=len(self._quality_good_history))
        self._route_event(f'{kind}_pause_resume_request', scan, control, self.recovery_evidence)
        return self._result(now)

    def begin_route_quality_coast(self, now, control):
        """Route fallback for bounded missing returns in the first-target stage.

        Trial41 halted, snapping 1689 to center while still rolling, and hit
        the outer board; trial42 halted the same way on front_sparse during
        the left turn. Cut power and hold the ACKed left instead, as the
        default sequence has since trial20, before or after handover. An
        un-ACKed step keeps the old halt (a late ACK would change the held
        feedback).
        """
        if not self.continue_route or self.route_stage != 'first_target':
            return False
        return self._begin_adopted_left_coast('route_quality_coast_adopted_left', now, control,
                                              require_handover=False)

    def _begin_adopted_left_coast(self, reason, now, control, *, require_handover=True):
        """Cut power while retaining only an ACKed left orbit-entry command."""
        if (self.phase != 'drive' or (require_handover and not self.handover_observed)
                or self.first_pass_evidence is not None or self.right_exit_since is not None
                or control.get('armed') is not True
                or control.get('motor') not in (DRIVE_MOTORS if self.continue_route else (TRIAL_MOTOR,))
                or control.get('command_acked') is not True
                or type(control.get('servo')) is not int
                or not NEUTRAL < control['servo'] == self.servo <= SERVO_MAX):
            return False
        self.begin_coast(reason, now)
        if self.phase != 'coast':
            return False
        self.quality_coast_servo = control['servo']
        self.quality_coast_reason = reason
        self.steering_target = self.quality_coast_servo
        return True

    def begin_coast(self, reason, now):
        seen = self._last_seen_compact
        target_on_left = (seen is not None and _number(now) and 0 <= now-seen[0] < SCAN_AGE_S
                          and _number(seen[1].get('bearing_left_rad')) and _number(seen[1].get('range_m'))
                          and 0 < seen[1]['bearing_left_rad'] < math.pi
                          and seen[1]['range_m'] <= ORBIT_REBIND_MAX_RANGE_M+.3)
        if (reason == 'left_turn_heading_overshoot' and self.continue_route and self.phase == 'drive'
                and not self.handover_observed and self.route_stage == 'first_target'
                and self._route_left_hold_since is None and NEUTRAL < self.servo <= SERVO_MAX
                and target_on_left and now >= self.last_now):
            # Trial51: a (not yet confirmed) first-target identity was just
            # observed on the left; the rule passes it anticlockwise, so keep
            # the adopted left while its confirmation/handover is checked.
            # Without such live evidence the original coast applies (no
            # course-shaped assumption that the route always keeps turning).
            self._route_left_hold_since = now
            self.left_overshoot_evidence = {'at': now, 'held_servo': self.servo,
                'scan_seq': (self._current_scan or {}).get('seq'),
                'last_seen_first_target': copy.deepcopy(seen[1]), 'last_seen_age_s': now-seen[0],
                'tracker_reason': getattr(self.target_tracker, 'reason', None),
                'basis': 'left_turn_past_exit_axis_with_live_first_target_identity_on_left'}
            self.steering_target = self.servo
            self.reason = 'route_left_turn_overshoot_hold_left'
            return
        if self.phase not in ('coast', 'locked') and _number(now) and now >= self.last_now:
            self.coast_since = now
        super().begin_coast(reason, now)

    def _route_left_hold_update(self, scan, lidar_age_s, now, control, safe, presteer_wait):
        """Held left at drive power after a route left-turn overshoot (trial51)."""
        advancing = self._orbit_inputs(scan, lidar_age_s, now, control, safe, presteer_wait)
        if self.phase == 'locked':
            return self._result(self.last_now)
        held = self.left_overshoot_evidence['held_servo']
        if (now-self.drive_since+1e-9 >= self.max_drive_s
                or now-self._route_left_hold_since+1e-9 >= ROUTE_LEFT_OVERSHOOT_HOLD_S):
            reason = ('maneuver_cumulative_drive_timeout' if now-self.drive_since >= self.max_drive_s
                      else 'route_left_turn_overshoot_hold_expired')
            self._route_left_hold_since = None
            if not self._begin_adopted_left_coast(reason, now, control, require_handover=False):
                self.begin_coast(reason, now)
            self._slew(now)
            return self._result(now)
        if advancing:
            self.steering_target = held
            self.reason = 'route_left_turn_overshoot_hold_left'
            self._slew(now)
        return self._result(now)

    def _exit_width_change_limit(self, previous_width):
        # Operator permits moderate changes in the live exit-width estimate.
        # Trial21's same-wall pair changed 8.5%, just above the old 15 cm gate.
        # Scale with the previous accepted observation, not a course dimension;
        # unique actual wall association is still required before this check.
        return max(super()._exit_width_change_limit(previous_width),
                   MAX_EXIT_WIDTH_CHANGE_FRACTION*previous_width)

    def _continue_with_tracked_width(self):
        # The operator confirms the exit remains present when the soft board
        # moves. An abrupt width estimate alone must not center an active turn.
        # Reuse the existing current-wall-only path construction with the last
        # accepted observed width; never invent a new wall or renew a missing
        # wall. No new-width target is adopted. Current body gates stay outside.
        return self.phase == 'drive'

    def _scan_axis_lines(self, scan, heading):
        """_axis_lines of this scan's returns, once per scan and heading (read-only)."""
        ranges = scan['ranges']
        key = (scan.get('seq'), scan.get('at_ms'))
        cache = self._axis_cache
        if cache is None or cache[0] != key or cache[1] != ranges:
            cache = self._axis_cache = (key, list(ranges), {})
        lines = cache[2].get(heading)
        if lines is None:
            lines = cache[2][heading] = _axis_lines(_scan_points(ranges, OPEN_LINE_REACH_M), heading)
        return lines

    def _route_proximity(self, scan):
        """Nearest known return within ROUTE_AVOID_GAP_M of the body box, or None."""
        ranges = scan.get('ranges') if isinstance(scan, dict) else None
        if not isinstance(ranges, list) or len(ranges) != 360:
            return None
        key = (scan.get('seq'), scan.get('at_ms'), self.first_pass_evidence is None)
        cache = self._proximity_cache
        if cache is not None and cache[0] == key and cache[1] == ranges:
            return cache[2]
        result = self._route_proximity_scan(scan, ranges)
        self._proximity_cache = (key, list(ranges), result)
        return result

    def _route_proximity_scan(self, scan, ranges):
        best = None
        for index, distance in enumerate(ranges):
            if not _number(distance) or not .02 <= distance <= 12:
                continue
            angle = math.radians(index)
            x, y = distance*math.cos(angle), -distance*math.sin(angle)
            head_on = x > FRONT_BODY_EXTENT_M and abs(y) <= SIDE_BODY_EXTENT_M/2
            if y > 0 and not head_on and self.first_pass_evidence is None:
                continue  # inside of the left turn / first orbit: their own laws
            gap = math.hypot(max(-REAR_BODY_EXTENT_M-x, 0., x-FRONT_BODY_EXTENT_M),
                             max(abs(y)-SIDE_BODY_EXTENT_M, 0.))
            if gap < ROUTE_AVOID_GAP_M and x >= -REAR_BODY_EXTENT_M and (best is None or gap < best['gap_m']):
                best = {'gap_m': gap, 'point_left_m': [x, y], 'bin': index, 'head_on': head_on}
        if best is None:
            return None
        if best['head_on']:
            def nearest(bins):
                values = [ranges[i] for i in bins if _number(ranges[i])]
                return min(values) if values else 12.
            side = 'front_left_open' if nearest(range(270, 340)) >= nearest(range(20, 91)) else 'front_right_open'
        else:
            side = 'left' if best['point_left_m'][1] > 0 else 'right'
        return {**best, 'side': side, 'source_seq': scan.get('seq')}

    @staticmethod
    def _path_points(scan):
        ranges = scan.get('ranges') if isinstance(scan, dict) else None
        if not isinstance(ranges, list) or len(ranges) != 360:
            return None
        points = []
        for index, distance in enumerate(ranges):
            if _number(distance) and .02 <= distance <= ROUTE_PATH_RANGE_M:
                x, y = _DIRECTION_COS[index]*distance, -_DIRECTION_SIN[index]*distance
                if x > -REAR_BODY_EXTENT_M-.2:
                    points.append((x, y))
        return points

    @staticmethod
    def _path_free_distance(points, servo):
        """Lidar travel before the body meets a known return on this arc (inf: clear)."""
        kappa = (servo-NEUTRAL)/ROUTE_PWM_PER_CURVATURE
        best = math.inf
        for x, y in points:
            if abs(kappa) < 1e-3:
                along, normal = x, y
            else:
                radius = 1/abs(kappa)
                side = y if kappa > 0 else -y
                rho = math.hypot(x, radius-side)
                normal = radius-rho
                along = radius*math.atan2(x, radius-side)
            if along+REAR_BODY_EXTENT_M < 0:
                continue  # behind the rear face: never reached driving forward
            half = SIDE_BODY_EXTENT_M+(ROUTE_PATH_MARGIN_M if along > FRONT_BODY_EXTENT_M else 0.)
            if abs(normal) > half:
                continue
            best = min(best, max(0., along-FRONT_BODY_EXTENT_M))
        return best

    def _route_right_authorized(self):
        return (self.continue_route and self.phase == 'drive' and not self.quality_resume_pending
                and self.first_pass_evidence is not None and self._route_center_ack is not None
                and ((self.route_stage in ('right_align', 'corridor_follow') and self.right_exit_center_ack is not None)
                     or (self.route_stage == 'second_orbit' and self.second_center_ack is not None)
                     or self.route_stage in ('light_turn', 'light_lane') or self.route_stage in OPEN_STAGES))

    def _route_override_bounds(self, target):
        """An obstacle correction may release a turn, never invent its opposite.

        The stage target has already passed its center-ACK gate. In particular
        a neutral target is a command, not permission for this layer to choose
        a new direction. During the first orbit, a hold/release must also not
        become a renewed left boost. These are task-relative bounds, with no
        assumption about where the support or exit lies on the course.
        """
        if self.route_stage == 'second_orbit' and self.second_return_preparing:
            # The stage already checks each return arc against the entire
            # support and the current boards. Another arc has not passed that
            # check. In particular, keep its neutral ACK request at neutral.
            return target, target
        if self.route_stage in TASK_LEFT_BYPASS_STAGES:
            lower = RIGHT_EXIT_MIN_PWM if target < NEUTRAL else NEUTRAL
            upper = SERVO_MAX
            cap = getattr(self, '_task_left_upper', None)
            scan = self._current_scan
            if cap is not None and isinstance(scan, dict) and cap[0] == scan.get('seq'):
                # Never turn the car past the lane-heading limit to dodge an
                # obstacle: a cone contact continues, a board contact ends.
                upper = max(target, min(SERVO_MAX, cap[1]))
            return lower, max(lower, upper)
        if (self.entry_kind == 'lane' and self.route_stage in ('start_lane', 'first_target')
                and self.first_pass_evidence is None):
            # Round 64 open first loop/approach: any clear arc between center
            # and full left (no right before the first pass).
            return NEUTRAL, SERVO_MAX
        if target == NEUTRAL:
            return NEUTRAL, NEUTRAL
        if target < NEUTRAL:
            return RIGHT_EXIT_MIN_PWM, NEUTRAL
        upper = SERVO_MAX
        if (self.handover_observed and self.route_stage == 'first_target'
                and self.first_pass_evidence is None and target <= self.servo):
            upper = target
        return NEUTRAL, upper

    def _route_path_target(self, target):
        """Stage target, or the nearest candidate whose arc is clear (round 53).

        Trial53: with the car pointing at a board after the first pass, no arc
        was clear and the old fallback took the arc with the longest free
        distance (1-3 cm more, always left). That held the servo off center,
        so the center ACK that must precede the right turn never came and the
        car turned into the board. The check therefore never overrides when
        no candidate is clear, and never while the stage waits for its center
        ACK before steering right; it may only pick a clear arc consistent
        with the stage's direction and first-target hold/release intention.
        """
        scan = self._current_scan
        natural = self.natural_steering_target
        # A right turn waiting for its center ACK is left alone (trial53),
        # except where the task side of an obstacle is left anyway.
        pending_right = (_number(natural) and natural < NEUTRAL and not self._route_right_authorized()
                         and self.route_stage not in TASK_LEFT_BYPASS_STAGES)
        lower, upper = self._route_override_bounds(target)
        key = (scan.get('seq') if isinstance(scan, dict) else None, target, self.route_stage,
               self._route_right_authorized(), pending_right, lower, upper)
        if self._path_cache is not None and self._path_cache[0] == key:
            return self._path_cache[1]
        points = self._path_points(scan)
        choice = target
        if points and not pending_right:
            free = self._path_free_distance(points, target)
            if free < ROUTE_PATH_LOOKAHEAD_M:
                candidates = list(ROUTE_PATH_LEFT_CANDIDATES)
                if key[3]:
                    candidates += list(ROUTE_PATH_RIGHT_CANDIDATES)
                # Filter before scoring: clamping a clear opposite arc would
                # produce a different arc which has never been checked.
                candidates = [c for c in candidates if lower <= c <= upper]
                scored = {c: self._path_free_distance(points, c) for c in candidates}
                clear = [c for c, f in scored.items() if f >= ROUTE_PATH_LOOKAHEAD_M]
                prefer_left = self.route_stage in TASK_LEFT_BYPASS_STAGES
                if clear:
                    above = [c for c in clear if c >= target]
                    pool = above if prefer_left and above else clear
                    choice = min(pool, key=lambda c: (abs(c-target), -c))
                    self.path_override_count += int(choice != target)
                self.path_evidence = {'source_seq': key[0], 'stage': self.route_stage, 'stage_target': target,
                    'stage_free_m': free, 'chosen': choice,
                    'chosen_free_m': scored.get(choice, free), 'clear_candidates': len(clear),
                    'overridden': choice != target, 'right_authorized': key[3],
                    'stage_override_bounds': [lower, upper],
                    'pwm_per_curvature': ROUTE_PWM_PER_CURVATURE, 'swept_path_certified': False}
        self._path_cache = (key, choice)
        return choice

    def _avoid_target(self, target):
        avoid = self._avoid
        if avoid is None:
            return target
        if avoid['side'] in ('left', 'front_right_open'):
            choice = min(target, NEUTRAL)
        else:
            choice = max(target, ROUTE_AVOID_LEFT_PWM)
        lower, upper = self._route_override_bounds(target)
        return max(lower, min(upper, choice))

    def _slew(self, now):
        if (self.continue_route and self.phase == 'drive' and not self.quality_resume_pending
                and (self.handover_observed or (self.route_stage == 'start_lane'
                                                and self.first_approach is not None))):
            avoid_target = self._route_path_target(self.steering_target)
        elif self.continue_route and self.phase in ('drive', 'quality_wait'):
            avoid_target = self._avoid_target(self.steering_target)
        else:
            avoid_target = self.steering_target
        if now-self.last_change+1e-9 >= PWM_INTERVAL_S and self.servo != avoid_target:
            target = avoid_target
            reducing_left = self.servo > NEUTRAL and target < self.servo
            # Round 58: light turn/lane included (trial57 turned right at the
            # generic 10 PWM/100 ms and hit the left-front board).
            post_pass = (self.continue_route and self.phase == 'drive'
                         and self.first_pass_evidence is not None
                         and self.route_stage in POST_PASS_STAGES)
            orbit_left = (self.continue_route and self.phase == 'drive' and self.handover_observed
                          and self.route_stage == 'first_target' and self.first_pass_evidence is None
                          and target > self.servo >= NEUTRAL)
            # Round 64: the first-support approach from the start lane uses the
            # route step both ways (the orbit it hands over to uses 40 too).
            # 2026-10-18: the start-lane wall law too (a 10 PWM/100 ms ramp
            # behind a .35 s steering delay made the old law overshoot).
            lane_approach = (self.continue_route and self.phase == 'drive'
                             and ((self.route_stage == 'start_lane'
                                   and (self._lane_start_slow
                                        or self.reason in ('start_lane_lane_walls', 'start_lane_side_wall')))
                                  or (self.route_stage == 'first_target' and self.entry_kind == 'lane'
                                      and self.handover_observed)))
            # Round 102 (car run 101): a light-entry pause steps its lane-axis
            # hold at the open-stage steps (10 PWM per ACK left the rolling car
            # turning from +16 to -23 deg on the loop's right).
            light_pause = (self.continue_route and self.phase == 'quality_wait'
                           and self.first_pass_evidence is not None and self.route_stage == 'light_entry'
                           and self._pause_steer_target is not None)
            open_steps = light_pause or (post_pass and self.route_stage in OPEN_STAGES)
            crossing = (target-NEUTRAL)*(self.servo-NEUTRAL) < 0
            releasing = self.servo != NEUTRAL and (crossing or abs(target-NEUTRAL) < abs(self.servo-NEUTRAL))
            if (post_pass and reducing_left and self.route_stage == 'second_orbit'
                    and self.second_return_preparing):
                post_step = ROUTE_RETURN_RELEASE_PWM_STEP
            elif open_steps and releasing:
                post_step = OPEN_RELEASE_PWM_STEP  # round 79: release fast, to centre first
                if crossing:
                    target = NEUTRAL
            elif lane_approach or open_steps:
                post_step = OPEN_PWM_STEP
            else:
                post_step = ROUTE_POST_PASS_PWM_STEP
            step = (PRESTEER_PWM_STEP if self.phase == 'presteer'
                    else post_step if post_pass or lane_approach or light_pause else
                    (ROUTE_EXIT_RELEASE_PWM_STEP if self._route_exit_release_active() else LEFT_RELEASE_PWM_STEP)
                    if reducing_left else ROUTE_ORBIT_LEFT_PWM_STEP if orbit_left else PWM_STEP)
            if avoid_target != self.steering_target:
                step = max(step, ROUTE_AVOID_PWM_STEP)
            if reducing_left and target < NEUTRAL:
                target = NEUTRAL  # First release to center; a later right step stays at 10.
            delta = max(-step, min(step, target-self.servo))
            self.servo += delta
            self.last_change = now  # One step only; delayed ticks never catch up.
            self.settle_since = None

    def lock(self, reason):
        # Only this experimental sequence may hold its last actually commanded
        # servo briefly while wall aliases resolve. Neither geometry clock nor
        # alignment evidence advances; a compact observation is still required
        # for handover. Default TurnMotion is unchanged.
        adoption = self._presteer_adoption_context
        if (reason == 'left_turn_outer_wall_ambiguous' and self.phase == 'presteer'
                and self.selected and self.last_geometry_receive is not None
                and self.last_geometry_publication is not None
                and self._last_actual_left_target is not None
                and self._last_actual_left_target > NEUTRAL
                and adoption is not None and adoption['fresh_feedback']
                and adoption['control'].get('armed') is True
                and adoption['control'].get('motor') == NEUTRAL
                and adoption['control'].get('servo') == self.servo
                and adoption['control'].get('command_acked', True) is True):
            # Neutral presteer may wait within its original preparation budget.
            # Hold only an already adopted servo, not the unseen wall target.
            # All scan/control validation has run before this measurement error.
            # Fresh unique geometry must be observed again before either slew
            # or the full adoption/response allowance can restart.
            self.wall_ambiguity_hold = self.geometry_missing = True
            self.steering_target = self.servo
            self._presteer_hold('left_turn_presteer_wall_ambiguity_hold')
            self.alignment_count = 0
            self.alignment_evidence = None
            self.alignment_publication = self.alignment_receive = None
            return
        if (reason == 'left_turn_outer_wall_ambiguous' and self.phase == 'drive'
                and self.last_geometry_receive is not None
                and self.last_geometry_publication is not None
                and 0 <= self.last_now-self.last_geometry_receive <= SCAN_AGE_S+1e-9
                and 0 <= (self.last_publication-self.last_geometry_publication)/1000 <= SCAN_AGE_S+1e-9):
            self.wall_ambiguity_hold = self.geometry_missing = True
            self.start_ready = False
            self.reason = 'left_turn_wall_ambiguity_hold'
            self.steering_target = self.servo
            self.alignment_count = 0
            self.alignment_evidence = None
            self.alignment_publication = self.alignment_receive = None
            self.last_error = None
            return
        super().lock(reason)

    def _snap(self, key, value):
        """Round 87: _result deep-copied ~45 evidence values every 20 ms tick
        (~.05 ms on the Mac, a third of a tick on an unchanged scan once the
        clearance check is reused). The copy is handed out again while the
        live value still equals it (C-level compare, same type); any change,
        in place or not, takes a new copy. Results only read these copies."""
        memo = self.__dict__.setdefault('_snapshots', {})
        old = memo.get(key, memo)
        if old is not memo and type(old) is type(value) and old == value:
            return old
        memo[key] = new = copy.deepcopy(value)
        return new

    def _result(self, now):
        if self.phase == 'drive' and self.adopted_presteer_pwm is None:
            self.adopted_presteer_pwm = self.servo
        result = super()._result(now)
        if result['motor'] == TRIAL_MOTOR:
            result['motor'] = self._route_motor()
        if result['motor'] not in DRIVE_MOTORS:
            self._launch_until = None  # the next push is a launch again
        elif self._launch_until is None:
            self._launch_until = now+ESC_LAG_S+LANE_LAUNCH_S
            self._power_since, self._power_prev_end = now, self._power_cmd_at
        if (result['motor'] == ROUTE_SLOW_MOTOR and self.continue_route and self.entry_kind == 'lane'
                and now < self._launch_until
                and self.route_stage in ('first_target', 'open_transfer', 'second_loop', 'light_entry')):
            # 1550 does not start a car at rest (2026-10-16: ~.7 s before it
            # moved; in simulation never; a one-tick 1560 resume ACK then 1550
            # left it standing). Every push after an unpowered output launches
            # at 1560 for the ESC lag plus LANE_LAUNCH_S.
            result['motor'] = TRIAL_MOTOR
        if result['motor'] in DRIVE_MOTORS:
            self._power_cmd_at, self._power_cmd_motor = now, result['motor']
            if result['motor'] == START_LAUNCH_MOTOR:
                self._launch_cmd_at = now
        stage = ('orbit_entry' if self.handover_observed and self.phase == 'drive'
                 else 'left_turn' if self.phase == 'drive' else result['turn_stage'])
        if self.phase == 'drive' and self.first_pass_preparing:
            stage = 'right_exit' if self.first_pass_evidence is not None else 'first_pass_prepare'
        result.update(turn_stage=stage, trial_scope='first_lidar_compact_target_orbit_entry',
                      compact_target=self._snap('compact_target', self.compact_target),
                      compact_target_reason=getattr(self.target_tracker, 'reason', None),
                      compact_loss_evidence=self._snap('compact_loss_evidence', self.compact_loss_evidence),
                      first_pass_preparing=self.first_pass_preparing,
                      first_pass_progress=self._snap('first_pass_progress', self.first_pass_progress),
                      first_exit_prepare_evidence=self._snap('first_exit_prepare_evidence', self.first_exit_prepare_evidence),
                      first_pass_evidence=self._snap('first_pass_evidence', self.first_pass_evidence),
                      right_exit_center_ack=self._snap('right_exit_center_ack', self.right_exit_center_ack),
                      right_exit_geometry=self._snap('right_exit_geometry', self.right_exit_geometry),
                      right_exit_min_pwm=RIGHT_EXIT_MIN_PWM,
                      right_exit_elapsed_s=0 if self.right_exit_since is None else max(0, now-self.right_exit_since),
                      right_exit_max_s=RIGHT_EXIT_MAX_S,
                      second_target_handover_observed=self.second_target_handover_observed,
                      full_two_cone_s_supported=False,
                      handover_observed=self.handover_observed,
                      orbit_reference_range_m=self.orbit_reference_range_m,
                      orbit_bias_pwm=self.orbit_bias_pwm,
                      orbit_feedback=self._snap('orbit_feedback', self.orbit_feedback),
                      orbit_left_entry_boost=self.orbit_left_entry_boost,
                      left_turn_feedback=self._snap('left_turn_feedback', self.left_turn_feedback),
                      handover_control=self._snap('handover_control', self.handover_control),
                      orbit_entry_elapsed_s=0 if self.orbit_since is None else max(0, now-self.orbit_since),
                      orbit_entry_max_s=ORBIT_ENTRY_MAX_S, second_orbit_max_s=SECOND_ORBIT_MAX_S,
                      presteer_max_s=self.max_presteer_s,
                      left_turn_servo_cap=SERVO_MAX,
                      steering_step_policy={'presteer': PRESTEER_PWM_STEP,
                          'reduce_left': LEFT_RELEASE_PWM_STEP,
                          'route_exit_reduce_left': ROUTE_EXIT_RELEASE_PWM_STEP if self.continue_route else None,
                          'route_post_pass': ROUTE_POST_PASS_PWM_STEP if self.continue_route else None,
                          'route_orbit_increase_left': ROUTE_ORBIT_LEFT_PWM_STEP if self.continue_route else None,
                          'increase_left': PWM_STEP,
                          'center_to_right': PWM_STEP, 'interval_s': PWM_INTERVAL_S},
                      object_semantic_verified=False, passed_cones=None,
                      competition_supported=False, completed=False,
                      test_sequence_finished=False,
                      inner_clearance=self._snap('inner_clearance', self.inner_clearance),
                      entry_bearing=self._snap('entry_bearing', self.entry_bearing),
                      entry_bearing_required=self.entry_bearing_required,
                      entry_bearing_released=self.entry_bearing_released,
                      adopted_presteer_pwm=self.adopted_presteer_pwm,
                      quality_coast_servo=self.quality_coast_servo,
                      quality_wait_refusal=self._snap('quality_wait_refusal', self.quality_wait_refusal),
                      quality_coast_reason=self.quality_coast_reason,
                      quality_coast_hold_active=(self.phase == 'coast'
                                                and self.quality_coast_servo is not None),
                      wall_ambiguity_hold=self.wall_ambiguity_hold)
        result.update(continue_route=self.continue_route, route_stage=self.route_stage,
            quality_wait_active=self.phase == 'quality_wait',
            quality_wait_servo=self.quality_wait_servo,
            quality_wait_used=self.quality_wait_used,
            quality_wait_anchor=self._snap('quality_wait_anchor', self.quality_wait_anchor),
            quality_wait_rebind=self._snap('quality_wait_rebind', self.quality_wait_rebind),
            left_turn_pause_used=self.left_turn_pause_used,
            neutral_pause_kind=self.neutral_pause_kind,
            recovery_count=self.recovery_count,
            recovery_evidence=self._snap('recovery_evidence', self.recovery_evidence),
            route_left_hold_active=self._route_left_hold_since is not None,
            left_overshoot_evidence=self._snap('left_overshoot_evidence', self.left_overshoot_evidence),
            route_avoid=self._snap('_avoid', self._avoid), avoid_count=self.avoid_count,
            path_evidence=self._snap('path_evidence', self.path_evidence), path_override_count=self.path_override_count,
            avoid_evidence=self._snap('avoid_evidence', self.avoid_evidence),
            wait_steering_changes=self.wait_steering_changes,
            post_pass_pause_evidence=self._snap('post_pass_pause_evidence', self.post_pass_pause_evidence),
            left_turn_pause_evidence=self._snap('left_turn_pause_evidence', self.left_turn_pause_evidence),
            left_turn_pause_geometry_error=self.left_turn_pause_geometry_error,
            quality_wait_evidence=self._snap('quality_wait_evidence', self.quality_wait_evidence),
            quality_resume_pending=self.quality_resume_pending,
            quality_wait_release=self._snap('quality_wait_release', self.quality_wait_release),
            quality_wait_previous_servo=self.quality_wait_previous_servo,
            route_alignment_evidence=self._snap('route_alignment_evidence', self.route_alignment_evidence),
            second_target=self._snap('second_target', self.second_target),
            second_rebind_evidence=self._snap('second_rebind_evidence', self.second_rebind_evidence),
            second_identity_recovering=self._second_recovering,
            second_center_ack=self._snap('second_center_ack', self.second_center_ack),
            second_pass_evidence=self._snap('second_pass_evidence', self.second_pass_evidence),
            second_feedback=self._snap('second_feedback', self.second_feedback),
            second_return_preparing=self.second_return_preparing,
            second_return_evidence=self._snap('second_return_evidence', self.second_return_evidence),
            route_end_coast_servo=self.route_end_coast_servo,
            light_feedback=self._snap('light_feedback', self.light_feedback),
            light_alignment_evidence=self._snap('light_alignment_evidence', self.light_alignment_evidence),
            lane_feedback=self._snap('lane_feedback', self.lane_feedback), lane_motor=self.lane_motor,
            finish_evidence=self._snap('finish_evidence', self.finish_evidence),
            second_acquisition_reason=self.second_acquisition_reason,
            second_recovery_prior=self._snap('second_recovery_prior', self.second_recovery_prior),
            role_distinct_evidence=self._snap('role_distinct_evidence', self.role_distinct_evidence),
            # Each event is a deep snapshot taken once (_route_event) and never
            # changed; re-copying them every 20 ms tick cost ~.2 ms (Mac).
            route_events=dict(self.route_events),
            two_target_observed_pass_complete=self.second_pass_evidence is not None,
            route_right_output_authorized=(self.continue_route and self.phase == 'drive'
                and not self.quality_resume_pending and self._route_center_ack is not None
                and ((self.first_pass_evidence is not None
                      and ((self.route_stage in ('right_align', 'corridor_follow')
                            and self.right_exit_center_ack is not None)
                           or (self.route_stage == 'second_orbit' and self.second_center_ack is not None)
                           or self.route_stage in ('light_turn', 'light_lane')
                           or self.route_stage in OPEN_STAGES))
                     # Round 64: start-lane wall corrections, after an actual center ACK.
                     or (self.route_stage == 'start_lane' and self.entry_kind == 'lane'))),
            route_center_ack=self._snap('_route_center_ack', self._route_center_ack),
            entry_kind=self.entry_kind, start_lane_feedback=self._snap('start_lane_feedback', self.start_lane_feedback),
            first_approach=self._snap('first_approach', self.first_approach),
            open_loop=self._snap('open_loop', self.open_loop), open_next_loop=self._snap('open_next_loop', self.open_next_loop),
            open_feedback=self._snap('open_feedback', self.open_feedback),
            compass=self._snap('compass.evidence', self.compass.evidence) if self.continue_route else None)
        if self.continue_route and self.phase == 'drive' and self.route_stage != 'first_target':
            result['turn_stage'] = self.route_stage
        if self.continue_route:
            result['trial_scope'] = 'opt_in_observed_two_target_route'
        return result

    def _usable_target(self, target, scan):
        if self.entry_bearing_required and not self.entry_bearing_released:
            return False
        if not isinstance(target, dict) or target.get('confirmed') is not True:
            return False
        close_bound_role = (self.continue_route and self.handover_observed
                            and self.first_pass_evidence is None
                            and target.get('close_maintenance_observation') is True
                            and self._full_target_points(target, scan, self.orbit_track_id) is not None)
        point = target.get('point_left_m')
        return (target.get('kind') == 'lidar_compact_object'
                and target.get('semantic_class') == 'unknown'
                and target.get('candidate_only') is True
                and type(target.get('source_seq')) is int and target['source_seq'] == scan.get('seq')
                and type(target.get('source_at_ms')) is int and target['source_at_ms'] == scan.get('at_ms')
                and isinstance(point, list) and len(point) == 2 and all(_number(v) for v in point)
                and _number(target.get('range_m'))
                and (.02 if close_bound_role else MIN_RANGE_M) <= target['range_m'] <= 3.0
                and _number(target.get('bearing_left_rad'))
                and math.radians(45) <= target['bearing_left_rad'] <= math.radians(135)
                and abs(math.hypot(*point)-target['range_m']) <= 1e-6
                and abs(math.atan2(point[1], point[0])-target['bearing_left_rad']) <= 1e-6
                and target.get('track_id') is not None)

    def _capture_compact_loss(self, scan, now, control, reason):
        # A one-shot in-memory snapshot from the actual control decision.
        # HTTP observers can skip the failing frame between successive reads.
        # Later scans must not overwrite this evidence. No disk IO or new
        # actuator behavior is introduced into the control tick.
        if self.compact_loss_evidence is None:
            self.compact_loss_evidence = {
                'decision_at': now, 'reason': reason,
                'tracker_reason': getattr(self.target_tracker, 'reason', None),
                'tracker_error': self.compact_target_error,
                'scan': copy.deepcopy({k: scan.get(k) for k in
                    ('frame_id', 'seq', 'at_ms', 'received_at', 'ranges')}),
                'previous_confirmed_target': copy.deepcopy(self._last_confirmed_compact_target),
                'current_target': copy.deepcopy(self.compact_target),
                'control': copy.deepcopy({k: control.get(k) for k in
                    ('armed', 'motor', 'servo', 'tick', 'seq', 'command_acked')}),
                'physical_identity_verified': False}

    @staticmethod
    def _held_support_points(held):
        """Support points of a tracker-held identity's last observation (car frame then)."""
        if not isinstance(held, dict) or not isinstance(held.get('point_left_m'), list):
            return []
        x, y = held['point_left_m']
        radius = (held.get('diameter_m') or 0.)/2
        return [(x-radius, y), (x+radius, y)]

    def _observe_first_pass(self, scan, now, control):
        """Prepare command reversal from current target/body geometry.

        Relative point motion is an observation trend, NOT vehicle speed or
        odometry. The lead covers commanded PWM steps only, not measured servo
        lag. Partial boundary observations cannot establish this new phase.
        """
        target = self.compact_target
        bins = target.get('support_bins') if isinstance(target, dict) else None
        if self.continue_route and (self._full_target_points(target, scan, self.orbit_track_id) is None
                or not _number(scan.get('received_at'))
                or not 0 <= now-scan['received_at'] < SCAN_AGE_S):
            self._clear_exit_prepare_window()
            self._first_pass_history.clear()
            return
        if (target is None or target.get('track_id') != self.orbit_track_id
                or not target.get('confirmed') or target.get('tracking_only_boundary_gap') is not False
                or not isinstance(bins, list) or len(bins) < TOLERANT_MAINTENANCE_MIN_POINTS
                or any(type(i) is not int or not 0 <= i < 360 for i in bins)):
            self._first_pass_history = []
            self._clear_exit_prepare_window()
            return
        ranges = scan['ranges']
        if any(not _number(ranges[i]) for i in bins):
            self._first_pass_history = []
            self._clear_exit_prepare_window()
            return
        points = [(ranges[i]*math.cos(math.radians(i)), -ranges[i]*math.sin(math.radians(i))) for i in bins]
        x = sum(p[0] for p in points)/len(points)
        if self.continue_route:
            # Route: extents include size-bounded unknown edges (conservative).
            points = self._full_target_points(target, scan, self.orbit_track_id) or points
        # Always derive extents from current raw support, never a cone radius.
        front = max(p[0] for p in points)
        side = min(p[1] for p in points)-SIDE_BODY_EXTENT_M
        row = {'source_seq': scan['seq'], 'source_at_ms': scan['at_ms'],
               'received_at': scan['received_at'], 'center_x_m': x}
        history = self._first_pass_history
        if history and (scan['seq'] <= history[-1]['source_seq']
                        or scan['at_ms'] <= history[-1]['source_at_ms']
                        or scan['received_at'] <= history[-1]['received_at']):
            return
        history.append(row)
        self._first_pass_history = history = history[-6:]
        pub_span = (row['source_at_ms']-history[0]['source_at_ms'])/1000
        receive_span = row['received_at']-history[0]['received_at']
        approaching = (len(history) >= 3 and pub_span >= .25 and receive_span >= .25
                       and all(b['center_x_m'] < a['center_x_m'] for a, b in zip(history, history[1:])))
        adopted = (control.get('command_acked') is True
                   and control.get('motor') in (DRIVE_MOTORS if self.continue_route else (TRIAL_MOTOR,))
                   and control.get('servo') == self.servo and NEUTRAL <= self.servo <= SERVO_MAX)
        lead = (math.ceil((self.servo-NEUTRAL)/LEFT_RELEASE_PWM_STEP)+1)*PWM_INTERVAL_S
        closing = (min((history[0]['center_x_m']-x)/pub_span,
                       (history[0]['center_x_m']-x)/receive_span) if approaching else None)
        prediction_prepare = (adopted and approaching and x <= FRONT_BODY_EXTENT_M
                   and side >= MANEUVER_BODY_CLEARANCE_M
                   and front+REAR_BODY_EXTENT_M <= closing*lead)
        # Ending the continuing-left demand is distinct from observing a pass.
        # Sustained orbit geometry can make relative x motion approach zero,
        # so the predictive rear-crossing test alone can wait indefinitely.
        # This opt-in gate only releases toward center; rear passage and the
        # current-corridor / center-ACK conditions still own all right steering.
        geometric_ready = False
        eligible = (self.continue_route and self.handover_observed is True
                    and self.phase == 'drive' and self.route_stage == 'first_target'
                    and front <= FRONT_BODY_EXTENT_M
                    and side >= MANEUVER_BODY_CLEARANCE_M)
        if eligible:
            # Trial37..39: three frames over .25 s plus the 20 PWM release let the
            # orbit continue far past the target. Two strictly fresh current
            # full-support frames are required now; still release-only.
            # Round 54 (trial53): one missed scan between the two body-front
            # observations (tracker-held frame) must not restart the count.
            geometric_ready = self._consecutive_fresh(self._first_exit_prepare_history, scan,
                                                      max_gap_s=SCAN_AGE_S+.15 if self.continue_route else SCAN_AGE_S)
        else:
            self._clear_exit_prepare_window()
        geometric_prepare = geometric_ready and adopted
        if geometric_prepare and not self.first_pass_preparing:
            self._latch_exit_prepare(scan, control,
                basis='current_first_target_complete_support_behind_body_front')
        prepare = prediction_prepare or geometric_prepare
        self.first_pass_progress = {**row, 'observed_frontmost_x_m': front,
            'observed_left_body_gap_m': side, 'command_center_lead_s': lead,
            'relative_x_closing_mps': closing, 'relative_trend_is_vehicle_speed': False,
            'prepare_condition': prepare, 'prediction_prepare_condition': prediction_prepare,
            'geometric_exit_prepare_condition': geometric_prepare,
            'body_rear_x_m': -REAR_BODY_EXTENT_M,
            'physical_steering_confirmed': False}
        if prepare:
            self.first_pass_preparing = True
            self.orbit_left_entry_boost = False
        if (self.first_pass_preparing and self.first_pass_evidence is None
                and front <= -REAR_BODY_EXTENT_M and side >= MANEUVER_BODY_CLEARANCE_M):
            self.first_pass_evidence = {**self.first_pass_progress,
                'track_id': target['track_id'], 'support_bins': list(bins),
                'basis': 'current_confirmed_target_observed_support_behind_body_rear',
                'semantic_class': 'unknown', 'physical_cone_pass_certified': False,
                'swept_path_certified': False}
            self.right_exit_since = now
            if self.continue_route:
                self.route_stage = 'right_align'
                self._route_event('first_pass', scan, control, self.first_pass_evidence)

    # ---- Round 64 (Claude): start-zone entry -------------------------------
    def _decide_entry(self, scan):
        """A native left opening at the start keeps the original entry; a
        measured straight lane without one starts in the start lane."""
        if not isinstance(scan, dict):
            return
        if scan.get('left_turn_goal') is not None:
            self.entry_kind = 'junction'
            return
        ok, evidence = self._lane_start_geometry(scan)
        self.start_lane_feedback = evidence
        if ok:
            self.entry_kind, self.route_stage = 'lane', 'start_lane'
            self._start_lane_width = evidence['width_m']
            # Ahead-left as well as abeam: the support past the divider's open
            # end is seen long before it is abeam (still left of the lane).
            self.target_tracker.widen_acquisition(FIRST_ACQUIRE_BEARING_RAD)
            # Round 72: the first cone had 4 returns at 3.03 m (beyond the 3 m
            # acquisition) when the car stopped; its approach starts at 3.5 m.
            self.target_tracker.extend_acquisition_range(FIRST_APPROACH_MAX_RANGE_M)

    @staticmethod
    def _forward_lanes(values, *, validated=False):
        """Valid native lanes ahead; near-duplicates of one lane merge into it.

        ``validated`` is only for internal callers that have just validated
        every candidate with _valid_candidate, without changing the inputs.
        """
        forward = [c for c in values if (validated or _valid_candidate(c))
                   and abs(c['heading_left_rad']) < math.pi/2]
        if len(forward) > 1:
            base = forward[0]
            if all(abs(_wrap(c['heading_left_rad']-base['heading_left_rad'])) <= PAIR_MERGE_HEADING_RAD
                   and abs(c['center_offset_left_m']-base['center_offset_left_m']) <= PAIR_MERGE_OFFSET_M
                   and abs(c['width_m']-base['width_m']) <= .15 for c in forward[1:]):
                forward = [max(forward, key=lambda c: c['left_wall_points']+c['right_wall_points'])]
        return forward

    def start_lane_quality_geometry(self, scan):
        """Read-only: driving the start lane with current native lane geometry.

        Round 66 car run: 0.4 s after the start a 5-bin front gap (returns lost
        at the edge between the divider's open end and the far end wall, ~7 m)
        stopped the run, because the bounded-gap allowance the route already
        grants after the handover (or in a started left turn with current
        geometry) did not cover the start lane. The server uses this as the
        same allowance's evidence; the body gate and every limit are unchanged.
        """
        if (not self.continue_route or self.entry_kind != 'lane' or self.route_stage != 'start_lane'
                or self.phase != 'drive' or self.drive_since is None or not isinstance(scan, dict)):
            return False
        values = scan.get('corridor_candidates')
        if (not isinstance(values, list) or len(values) > 24
                or any(not _valid_candidate(candidate) for candidate in values)):
            return False
        # Either the start-lane law's own lane, or (lane briefly unreported,
        # e.g. returns also lost on a side board) the side boards still
        # measuring the compass heading in this scan.
        return len(self._forward_lanes(values)) == 1 or self.compass.measurable(scan)

    def _lane_start_geometry(self, scan):
        """One forward native lane around the car, nearly aligned, room ahead."""
        values = scan.get('corridor_candidates') if isinstance(scan, dict) else None
        ranges = scan.get('ranges') if isinstance(scan, dict) else None
        if (not isinstance(values, list) or len(values) > 24 or not isinstance(ranges, list)
                or len(ranges) != 360):
            return False, {'reason': 'start_lane_invalid_scan'}
        if scan.get('left_turn_goal') is not None:
            return False, {'reason': 'start_lane_native_left_opening_present'}
        forward = self._forward_lanes(values)
        if len(forward) != 1:
            return False, {'reason': 'start_lane_unique_lane_missing', 'forward_lanes': len(forward)}
        lane = forward[0]
        front = [ranges[i % 360] for i in range(-10, 11) if _number(ranges[i % 360])]
        evidence = {'source_seq': scan.get('seq'), 'source_at_ms': scan.get('at_ms'),
                    'heading_left_rad': lane['heading_left_rad'],
                    'center_offset_left_m': lane['center_offset_left_m'], 'width_m': lane['width_m'],
                    'body_gap_m': self._corridor_body_gap(lane), 'front_m': min(front) if front else None,
                    'compass_heading_rad': self.compass.current(scan)}
        if abs(lane['heading_left_rad']) > LANE_START_MAX_HEADING_RAD:
            return False, {**evidence, 'reason': 'start_lane_heading_too_large'}
        if not LANE_START_WIDTH_M[0] <= lane['width_m'] <= LANE_START_WIDTH_M[1]:
            return False, {**evidence, 'reason': 'start_lane_width_out_of_range'}
        if evidence['body_gap_m'] < MANEUVER_BODY_CLEARANCE_M:
            return False, {**evidence, 'reason': 'start_lane_body_gap'}
        if evidence['front_m'] is None or evidence['front_m'] < LANE_START_MIN_FRONT_M:
            return False, {**evidence, 'reason': 'start_lane_front_blocked'}
        if evidence['compass_heading_rad'] is None:
            return False, {**evidence, 'reason': 'start_lane_compass_unavailable'}
        return True, {**evidence, 'reason': None}

    def _start_lane_presteer(self, scan, lidar_age_s, now, control, *, safe, presteer_wait):
        """Neutral start: adopt 1500 for three fresh feedback ticks and .3 s."""
        self.presteer_waiting = False
        if not (isinstance(control, dict) and control.get('armed') is True):
            # Neutral preview before arming: admission from this scan only.
            self.last_now = now
            ok, evidence = self._lane_start_geometry(scan)
            self.start_lane_feedback = evidence
            self.start_ready, self.steering_target = ok, NEUTRAL
            self.reason = 'start_lane_neutral_preview' if ok else evidence['reason']
            return self._result(now)
        if now-self.started_at+1e-9 >= self.max_presteer_s:
            self.lock('left_turn_presteer_timeout')
            return self._result(now)
        new_feedback = type(control.get('tick')) is int and (self.last_control_tick is None
                                                             or control['tick'] > self.last_control_tick)
        advancing = self._orbit_inputs(scan, lidar_age_s, now, control, safe, False)
        if self.phase == 'locked':
            return self._result(self.last_now)
        if advancing:
            ok, evidence = self._lane_start_geometry(scan)
            self.start_lane_feedback = evidence
            self._lane_start_ok = ok
        ok = getattr(self, '_lane_start_ok', False)
        self.steering_target = NEUTRAL
        if presteer_wait or not ok:
            self._lane_start_settle_since, self._lane_start_ticks = None, 0
            self.presteer_waiting = presteer_wait
            self.start_ready = False
            self.reason = ('start_lane_presteer_quality_hold' if presteer_wait
                           else self.start_lane_feedback.get('reason') or 'start_lane_geometry_missing')
            return self._result(now)
        self.start_ready = True
        adopted = (self.servo == NEUTRAL and control['servo'] == NEUTRAL and control['motor'] == NEUTRAL
                   and control.get('command_acked', True) is True)
        if not adopted:
            self._lane_start_settle_since, self._lane_start_ticks = None, 0
            self.reason = 'start_lane_waiting_neutral_adoption'
            return self._result(now)
        if new_feedback:
            if self._lane_start_settle_since is None:
                self._lane_start_settle_since = now
            self._lane_start_ticks += 1
        self.reason = 'start_lane_neutral_allowance'
        if (new_feedback and self._lane_start_ticks >= 3
                and now-self._lane_start_settle_since+1e-9 >= LANE_START_SETTLE_S):
            self.phase, self.drive_since, self.presteer_ended_at = 'drive', now, now
            self.reason = 'start_lane_drive'
            self._route_event('start_lane', scan, control, self.start_lane_feedback)
        return self._result(now)

    def _start_lane_handover_ready(self, scan, control):
        target = self.compact_target
        return (isinstance(scan, dict) and scan.get('seq') != self.last_seq
                and self.first_approach is not None and self._usable_target(target, scan)
                and target['track_id'] == self.first_approach.get('track_id')
                and target['range_m'] <= OPEN_HANDOVER_MAX_RANGE_M
                and target['bearing_left_rad'] >= FIRST_HANDOVER_BEARING_RAD
                and isinstance(control, dict) and control.get('command_acked') is True
                and control.get('motor') in DRIVE_MOTORS and control.get('servo') == self.servo > NEUTRAL)

    def _start_side_wall(self, scan, heading):
        """Lane walls from one side board when the corridor fit fails.

        The board comes from this frame (side_wall_fit); the lane centre is
        half the start lane's width (measured at admission in this run) from
        it. It must run along the compass axis (within 5 deg) when the compass
        is measured. The output has corridor_walls' shape for the wall law.
        """
        width = getattr(self, '_start_lane_width', None)
        if not _number(width):
            return None
        fits = [f for f in (side_wall_fit(scan['ranges'], side) for side in ('right', 'left')) if f is not None]
        if heading is not None:
            fits = [f for f in fits if abs(math.atan(f['slope'])-heading) <= LANE_SIDE_WALL_HEADING_RAD]
        if not fits:
            return None
        fit = max(fits, key=lambda f: f['points'])
        centre = fit['intercept_m']-width/2 if fit['side'] == 'right' else fit['intercept_m']+width/2
        self.start_side_wall = {'source_seq': scan['seq'], **fit, 'lane_width_m': width}
        return {'offset_right_m': centre, 'heading_right_deg': math.degrees(math.atan(fit['slope'])),
                'slope': fit['slope'], 'width_m': width, 'side_wall': fit['side']}

    def lane_wall_output_active(self):
        """The current command comes from the start-lane wall law (drive or coast)."""
        return (self.continue_route and self.route_stage == 'start_lane'
                and (self.phase == 'drive' and self.reason in ('start_lane_lane_walls', 'start_lane_side_wall')
                     or self.phase == 'coast' and self.start_lane_coast_steering is not None))

    def _start_lane_update(self, scan, lidar_age_s, now, control, *, safe, presteer_wait):
        """Start lane: wall law (else compass hold), then the first approach."""
        if self.phase == 'presteer':
            return self._start_lane_presteer(scan, lidar_age_s, now, control, safe=safe,
                                             presteer_wait=presteer_wait)
        advancing = self._orbit_inputs(scan, lidar_age_s, now, control, safe, presteer_wait)
        if self.phase == 'locked':
            return self._result(self.last_now)
        if self.phase == 'drive' and now-self.drive_since+1e-9 >= self.max_drive_s:
            self.begin_coast('maneuver_cumulative_drive_timeout', now)
        if self.phase == 'coast':
            if self.coast_since is not None and now-self.coast_since+1e-9 >= COAST_MAX_S:
                self.lock('coast_standstill_unconfirmed')
            else:
                if advancing:
                    # 2026-10-18: the car rolls on for the ESC lag and more;
                    # keep following the current lane walls (no walls: centre)
                    # instead of holding one command into a side board.
                    walls = corridor_walls(scan)
                    target = (NEUTRAL if walls is None else
                              self._lane_start_steering.update(walls, scan['seq'], now, True, scan['at_ms']))
                    if target < NEUTRAL and (self._route_center_ack is None or self.servo > NEUTRAL):
                        target = NEUTRAL  # right only after a current actual center ACK
                    self.steering_target = target
                    self.start_lane_coast_steering = {'source_seq': scan['seq'], 'walls': walls,
                                                      'target_pwm': target, 'reason': self.reason}
                self._slew(now)
            return self._result(now)
        if self.phase != 'drive':
            return self._result(now)
        if not advancing:
            self._slew(now)
            return self._result(now)
        if not self._route_adopted(control, now):
            return self._result(now)
        heading = self.compass.current(scan)
        approach = self._open_first_approach(scan)
        held = getattr(self.target_tracker, 'held', None)
        walls = None
        if approach is not None:
            self.first_approach, self._lane_start_slow = approach, True
            natural, source = approach['target_pwm'], 'first_support_approach'
        elif (self.first_approach is not None and isinstance(held, dict)
              and held.get('track_id') == self.first_approach['track_id']):
            natural, source = self.servo, 'first_support_missing_frame_hold'
        else:
            walls = corridor_walls(scan)
            if walls is not None:
                self._lane_start_steering.servo = max(NEUTRAL-LANE_START_HOLD_PWM,
                                                      min(NEUTRAL+LANE_START_HOLD_PWM, self.servo))
                natural = self._lane_start_steering.update(walls, scan['seq'], now, True, scan['at_ms'])
                source = 'lane_walls'
            elif (walls := self._start_side_wall(scan, heading)) is not None:
                # Round 72: the left divider ended; keep the lane centre from
                # the right board (or the left one) measured in this frame.
                self._lane_start_steering.servo = max(NEUTRAL-LANE_START_HOLD_PWM,
                                                      min(NEUTRAL+LANE_START_HOLD_PWM, self.servo))
                natural = self._lane_start_steering.update(walls, scan['seq'], now, True, scan['at_ms'])
                source = 'side_wall'
            elif heading is not None:
                natural = NEUTRAL+max(-LANE_START_HOLD_PWM, min(LANE_START_HOLD_PWM,
                                                               round(LEFT_TRIAL_GAIN*(-heading))))
                source = 'compass_heading_hold'
            else:
                natural, source = NEUTRAL, 'no_geometry_center'
        target = natural
        if target < NEUTRAL and (self._route_center_ack is None or self.servo > NEUTRAL):
            target = NEUTRAL  # right only after a current actual center ACK
        ahead = self._lane_front_range(scan)
        # Start lane: at least three rates, so the median rejects one jump of
        # the far ray ahead (2026-10-18 simulation, .94 m lane: rates 2.7/-.3
        # gave 2.65 m/s at .9 m/s and cut the lane); unknown speed meanwhile
        # is the conservative LANE_SPEED_UNKNOWN_MPS.
        speed = self._track_speed(scan, self._lane_front_track(scan, ahead))
        stop_at = allowed = None
        if approach is None and source != 'first_support_missing_frame_hold':
            # A cut now still pushes for the rest of the ESC lag, the speed
            # rising meanwhile; once coasting that long, neither applies.
            reaction = self._reaction_s(now)
            # Only 1560 still accelerates through the lag; 1550 holds ~1.1 m/s
            # and slows a faster car.
            rise = (LANE_POWER_ACCEL_MPS2*(reaction-LANE_REACTION_BASE_S)
                    if self._power_cmd_motor == TRIAL_MOTOR else
                    START_LAUNCH_ACCEL_MPS2*(reaction-LANE_REACTION_BASE_S)
                    if self._power_cmd_motor == START_LAUNCH_MOTOR else 0.)
            if self._launch_cmd_at is not None:
                # Round 100: a 1600 frame still in the ESC lag keeps adding its
                # rise after a later 1560/1550 command.
                rise = max(rise, START_LAUNCH_ACCEL_MPS2*max(0., min(ESC_LAG_S, ESC_LAG_S-(now-self._launch_cmd_at))))
            held = self._start_speed_last
            if speed is not None:
                base = speed
            elif (self._power_cmd_motor == START_LAUNCH_MOTOR and held is not None
                  and 0 <= now-held[0] <= LANE_SPEED_HOLD_S):
                # Round 100: within a 1600 launch's ESC lag an unknown speed
                # is its last measurement (1.3 m/s plus the 1600 rise would
                # put the stopping range past most start lanes).
                base = held[1]+LANE_SPEED_HOLD_MARGIN_MPS
            else:
                base = LANE_SPEED_UNKNOWN_MPS
            v = base+rise
            stop_at = (FRONT_BODY_EXTENT_M+LANE_FINISH_GAP_M+v*v/(2*LANE_START_STOP_DECEL_MPS2)
                       + v*reaction)
            allowed = self._start_allowed_speed(ahead)
            if speed is not None:
                self._start_speed_measured = True
                self._start_speed_last = (now, speed)
            settling = (self._launch_cmd_at is not None
                        and now-self._launch_cmd_at < ESC_LAG_S+START_LAUNCH_SETTLE_S)
            self._start_motor = self._start_governor(speed, rise, allowed, self._start_speed_measured, settling)
        else:
            # The approach drives on at 1550 (a stopped car restarts at 1560).
            self._start_motor = (TRIAL_MOTOR if speed is not None and speed < LANE_SPEED_RESTART_MPS
                                 else ROUTE_SLOW_MOTOR)
        # Round 103 (car run 102): a stopping range that reaches the board
        # ahead before the first support is found cuts the power for this scan
        # only; the lane law keeps steering and the next scan checks again (the
        # governor's own speed cuts always worked so). It was a terminal coast:
        # a closing speed read off the end board ~7 m ahead (1.77 m/s at ~1.3)
        # ended run 102 .3 m after the start. Without a support in reach the
        # governor still allows no speed inside LANE_START_SEARCH_M of the board.
        cut = stop_at is not None and ahead is not None and ahead <= stop_at
        if cut:
            self._start_motor = NEUTRAL
            self._route_event('start_lane_board_ahead_cut', scan, control,
                              {'source_seq': scan['seq'], 'front_range_m': ahead, 'front_stop_range_m': stop_at,
                               'speed_mps': speed, 'basis': 'stopping_range_reaches_board_without_first_support'})
        self.start_lane_feedback = {'source_seq': scan['seq'], 'steering_source': source,
            'nominal_target_pwm': natural, 'walls': walls, 'compass_heading_rad': heading,
            'first_approach': approach, 'front_range_m': ahead, 'speed_mps': speed,
            'front_stop_range_m': stop_at, 'allowed_speed_mps': allowed, 'slow': self._lane_start_slow,
            'motor': self._start_motor, 'board_ahead_cut': cut, 'swept_path_certified': False}
        self.steering_target, self.natural_steering_target = target, natural
        self.reason = 'start_lane_' + source
        self._slew(now)
        if self.servo > NEUTRAL:
            self._route_center_ack = None
        return self._result(now)

    # ---- Round 64 (Claude): open-course loops, transfer and light-lane entry --
    def _open_loop_geometry(self, scan, target, identity, side, axis, pose=None):
        """Plan the loop round this support and its pursuit command, or None.

        ``pose`` (round 79, _response_pose): on the approach line the command
        may pursue from that predicted pose (_overshoot_guard); the plan and
        its approach state stay those of the current pose.

        One state per loop (side +1 first/anticlockwise, -1 second/clockwise),
        so the second loop can be planned while the first still steers. The
        free distances are kept as their running minimum from low-yaw frames
        (boards do not move; a direction once seen blocked stays blocked), so
        the plan changes only when new evidence narrows it.

        Once following a circle, a small score change must not move its centre
        across the support and reverse the task turn while the prior circle
        remains feasible. Approach replanning and genuinely narrower bounds
        retain their existing behavior. This is not a swept-path certificate.
        """
        heading = self.compass.current(scan)
        if heading is None or self._full_target_points(target, scan, identity) is None:
            return None
        slot = self._open_slots.setdefault(side, {'free': {}, 'plan': None, 'key': None, 'radius': None})
        ranges = scan['ranges']
        center, radius = _support_center(ranges, target['support_bins'], target.get('diameter_m'))
        slot['radius'] = radius = max(radius, slot['radius'] or 0.)
        e1, e2 = _axis_frame(axis, heading, side)
        points = _scan_points(ranges, math.hypot(*center)+OPEN_PROBE_REACH_M)
        skip = radius+OPEN_SUPPORT_SKIP_M
        free_memory, changed = slot['free'], False
        for theta in OPEN_PROBE_DIRECTIONS_RAD:
            known = theta in free_memory
            if known and abs(self.compass.rate) > OPEN_PROBE_MAX_RATE_RAD_S:
                continue
            c, s = math.cos(theta), math.sin(theta)
            free = _probe_free(points, center, (c*e1[0]+s*e2[0], c*e1[1]+s*e2[1]), skip)
            if free != math.inf:
                free = math.floor(free/OPEN_PROBE_QUANTUM_M)*OPEN_PROBE_QUANTUM_M
            if not known or free < free_memory[theta]:
                free_memory[theta], changed = free, True
        key = (identity, round(radius, 2))
        approach = (-(center[0]*e1[0]+center[1]*e1[1]), -(center[0]*e2[0]+center[1]*e2[1]))
        plan_update = None
        # Round 83 (car run 82: a .111 s tick): the same inputs give the same
        # plan; a stopped car (approach within 1 cm) reuses it.
        plan_inputs = (key, tuple(sorted(free_memory.items())), round(approach[0], 2), round(approach[1], 2))
        if slot['plan'] is not None and slot.get('plan_inputs') == plan_inputs:
            changed = False
        elif changed or slot['plan'] is None or slot['key'] != key or slot.get('approaching', True):
            # While the car is still before the tangent point the plan also
            # depends on where the car is (approach term), so it is redone.
            candidate = _plan_loop([(t, free_memory.get(t, math.inf)) for t in OPEN_PROBE_DIRECTIONS_RAD],
                                      radius, OPEN_LEFT_RADIUS_M if side > 0 else OPEN_RIGHT_RADIUS_M,
                                      approach=approach)
            # Test the existing circle against the latest conservative probe
            # memory, not its old optimizer score. Those distances can include
            # earlier low-yaw observations; they are not all current-frame rays.
            old_plan = slot['plan']
            reject_reversal = False
            plan_update = {'checked_source_seq': scan['seq'],
                'constraint_basis': 'existing_low_yaw_probe_history_running_minimum',
                'prior_plan_margins_m': None, 'candidate_curvature_per_m': None,
                'prior_curvature_per_m': None, 'rejected_task_direction_reversal': False,
                'swept_path_certified': False}
            if (old_plan is not None and slot['key'] is not None and slot['key'][0] == identity
                    and not slot.get('approaching', True)):
                old_r = old_plan['radius_m']
                old_a, old_b = old_plan['offset_axis_m'], old_plan['offset_side_m']
                inner = radius+SIDE_BODY_EXTENT_M+OPEN_INNER_GAP_M
                extents = [(t, _loop_extent(old_a, old_b, old_r, t)) for t in OPEN_PROBE_DIRECTIONS_RAD]
                margins = {'enclosing_support_margin_m': old_r-math.hypot(old_a, old_b)-inner,
                    'minimum_inner_probe_margin_m': min(extent-inner for _, extent in extents),
                    'minimum_outer_probe_margin_m': min(free_memory.get(t, math.inf)-SIDE_BODY_EXTENT_M
                                                       -OPEN_OUTER_GAP_M-extent for t, extent in extents)}
                plan_update['prior_plan_margins_m'] = {
                    k: (value if math.isfinite(value) else None) for k, value in margins.items()}
                old_feasible = (old_r-math.hypot(old_a, old_b) >= inner
                    and all(inner <= extent <= free_memory.get(t, math.inf)-SIDE_BODY_EXTENT_M-OPEN_OUTER_GAP_M
                            for t, extent in extents))
                if old_feasible:
                    def plan_carrot(value):
                        pa, pb = value['offset_axis_m'], value['offset_side_m']
                        pc = (center[0]+pa*e1[0]+pb*e2[0], center[1]+pa*e1[1]+pb*e2[1])
                        return _loop_carrot(pc, value['radius_m'], side, OPEN_LOOKAHEAD_M, e1, e2,
                            support=(center, radius+SIDE_BODY_EXTENT_M+OPEN_CARROT_SUPPORT_GAP_M))[0]
                    candidate_carrot, prior_carrot = plan_carrot(candidate), plan_carrot(old_plan)
                    reject_reversal = side*candidate_carrot[1] < 0 <= side*prior_carrot[1]
                    plan_update.update(candidate_curvature_per_m=_pursuit_pwm(candidate_carrot)[0],
                                       prior_curvature_per_m=_pursuit_pwm(prior_carrot)[0])
            plan_update['rejected_task_direction_reversal'] = reject_reversal
            if not reject_reversal:
                slot['plan'] = candidate
                slot['key'] = key
                slot['plan_source_seq'] = scan['seq']
                slot['plan_inputs'] = plan_inputs
        plan = slot['plan']
        a, b = plan['offset_axis_m'], plan['offset_side_m']
        loop_center = (center[0]+a*e1[0]+b*e2[0], center[1]+a*e1[1]+b*e2[1])
        carrot, mode, before = _loop_carrot(loop_center, plan['radius_m'], side, OPEN_LOOKAHEAD_M, e1, e2,
            support=(center, radius+SIDE_BODY_EXTENT_M+OPEN_CARROT_SUPPORT_GAP_M))
        slot['approaching'] = before > 0
        curvature, pwm = _pursuit_pwm(carrot)
        current_pose_pwm = pwm
        command = None
        if pose is not None and mode == 'approach_line':
            # Only on the approach line: on the circle the support is close and
            # the current-pose carrot keeps its clearance check (simulation:
            # a predicted carrot there cut into the support).
            command = _loop_carrot(_in_pose(loop_center, pose), plan['radius_m'], side, OPEN_LOOKAHEAD_M,
                _turned(e1, pose[2]), _turned(e2, pose[2]),
                support=(_in_pose(center, pose), radius+SIDE_BODY_EXTENT_M+OPEN_CARROT_SUPPORT_GAP_M))[0]
            guarded = _overshoot_guard(pwm, _pursuit_pwm(command)[1])
            if guarded != pwm:
                curvature, pwm = _pursuit_pwm(command)
        return {'source_seq': scan['seq'], 'track_id': identity,
            'direction': 'anticlockwise' if side > 0 else 'clockwise', 'axis_rad': axis,
            'compass_heading_rad': heading, 'progress_rad': side*(heading-axis),
            'support_center_left_m': [round(v, 3) for v in center], 'support_radius_m': round(radius, 3),
            'free_m': {round(math.degrees(t)): (None if v == math.inf else v) for t, v in free_memory.items()},
            'plan': {k: (None if isinstance(v, float) and not math.isfinite(v) else v) for k, v in plan.items()},
            'plan_metrics_basis': 'selection_time_optimizer_score_and_probe_margins',
            'plan_selection_source_seq': slot.get('plan_source_seq'), 'plan_update': plan_update,
            'loop_center_left_m': [round(v, 3) for v in loop_center], 'pursuit_mode': mode,
            'before_tangent_point_m': round(before, 3), 'carrot_left_m': [round(v, 3) for v in carrot],
            'response_pose': None if pose is None else [round(v, 3) for v in pose],
            'response_carrot_left_m': None if command is None else [round(v, 3) for v in command],
            'pursuit_curvature_per_m': curvature, 'target_pwm': pwm, 'current_pose_pwm': current_pose_pwm,
            'basis': 'support_loop_planned_from_probed_free_space', 'swept_path_certified': False}

    def _tracking_heading(self, scan):
        """Compass heading for rotation-compensated association (open course only)."""
        return self.compass.current(scan) if self.entry_kind == 'lane' else None

    @staticmethod
    def _open_target_usable(target, scan, identity, side):
        """Current confirmed support of this identity anywhere on the task side.

        The orbit window of the narrow course (45-135 deg) does not hold on an
        open loop: a support 2.4 m to the side (wide course) is at 44 deg on
        the tangent approach, which ended the loop in a recovery wait.
        """
        return (isinstance(target, dict) and target.get('confirmed') is True
                and target.get('track_id') == identity and identity is not None
                and target.get('kind') == 'lidar_compact_object'
                and type(target.get('source_seq')) is int and target['source_seq'] == scan.get('seq')
                and type(target.get('source_at_ms')) is int and target['source_at_ms'] == scan.get('at_ms')
                and _number(target.get('bearing_left_rad'))
                and 0 <= side*target['bearing_left_rad'] <= math.pi)

    def _open_reset_loop(self, side=None):
        if side is None:
            self._open_slots = {}
        else:
            self._open_slots.pop(side, None)
        self._open_lost_since = None

    def _open_first_approach(self, scan):
        """First-support pursuit from the start lane (anticlockwise loop plan).

        It starts only once the body-wide strip along the planned tangent is
        clear of every return (the start divider is still beside the car
        before its open end), then continues until the handover.
        """
        target = self.compact_target
        if (not isinstance(target, dict) or target.get('confirmed') is not True
                or target.get('track_id') is None or not _number(target.get('range_m'))
                or not _number(target.get('bearing_left_rad'))
                or not 0 < target['bearing_left_rad'] <= FIRST_ACQUIRE_BEARING_RAD[1]
                or target['range_m'] > FIRST_APPROACH_MAX_RANGE_M):
            return None
        heading = self.compass.current(scan)
        if heading is not None:
            e1, _ = _axis_frame(0., heading)
            x, y = target['point_left_m']
            if e1[0]*x+e1[1]*y < OPEN_FIRST_MIN_ALONG_M:
                return None  # behind along the start axis: not the first support
            if (_on_board_run((x, y), self._scan_axis_lines(scan, heading), heading)
                    or _support_depth(scan['ranges'], target.get('support_bins', [])) > OPEN_SUPPORT_MAX_DEPTH_M):
                return None  # a board's end or grazing face, not a support
        if self.first_approach is not None and self.first_approach.get('track_id') != target['track_id']:
            self._open_reset_loop(1)
        loop = self._open_loop_geometry(scan, target, target['track_id'], 1, 0.)
        if loop is None:
            return None
        clear = True
        if self.first_approach is None:
            # The strip along the pursuit direction, OPEN_APPROACH_CHECK_M long.
            cx, cy = loop['carrot_left_m']
            length = math.hypot(cx, cy) or 1.
            end = (OPEN_APPROACH_CHECK_M*cx/length, OPEN_APPROACH_CHECK_M*cy/length)
            clear = _strip_clear(_scan_points(scan['ranges'], OPEN_APPROACH_CHECK_M+1.), end,
                                 tuple(loop['support_center_left_m']), loop['support_radius_m']+OPEN_SUPPORT_SKIP_M)
        self.open_loop = loop
        approach = {**loop, 'range_m': target['range_m'], 'bearing_left_rad': target['bearing_left_rad'],
                    'approach_strip_clear': clear,
                    'basis': 'first_support_left_of_car_anticlockwise_rule_loop_plan'}
        return approach if clear else None

    def _open_second_maturing(self, scan, ahead, stop_at):
        """A second support tracked in this scan, not yet confirmed, nearer than
        the board ahead, while the board is within OPEN_SECOND_CONFIRM_GRACE_M
        of the stopping range (see there). The body gate is unchanged.

        Round 106 (car runs 98/99/101/105): the support standing on the
        transfer axis was itself the return ahead (2.87-2.98 m, its centre
        range ~1 cm farther), so it was never "nearer than the board" and the
        hold stopped the car for .4-.7 s just before its confirmation. The
        board is then the nearest axis return that is not one of its own.
        """
        target = self.second_target
        if not (ahead is not None and ahead > stop_at-OPEN_SECOND_CONFIRM_GRACE_M
                and isinstance(target, dict) and target.get('confirmed') is False
                and type(target.get('track_id')) is int and target.get('source_seq') == scan.get('seq')
                and _number(target.get('range_m'))
                and self.role_distinct_evidence is not None
                and self.role_distinct_evidence.get('source_seq') == scan.get('seq')):
            return False
        if target['range_m'] < ahead:
            return True
        bins = target.get('support_bins')
        if not isinstance(bins, list) or not isinstance(scan.get('ranges'), list):
            return False
        board = self._lane_front_range(scan, exclude=set(bins))
        return board is None or target['range_m'] < board

    def _open_confirmed_second(self, scan):
        """The second support when confirmed with its complete support this frame."""
        target = self.second_target
        if not isinstance(target, dict) or target.get('confirmed') is not True:
            return None
        identity = self.second_track_id if self.second_track_id is not None else target.get('track_id')
        return target if self._full_target_points(target, scan, identity) is not None else None

    def _open_first_loop_update(self, scan, now, control):
        """First loop after the handover (anticlockwise).

        It ends when the compass shows the heading reversed, or earlier once
        the second support is confirmed and the car already points along the
        second loop's tangent (left arc - straight - right arc: continuing
        left past that tangent only steepens the crossing; simulation, 5 m
        course: the second loop then ran into the start divider).
        """
        heading = self.compass.current(scan)
        progress = None if heading is None else heading
        if progress is not None and progress >= OPEN_SECOND_LOOKOUT_RAD and self.second_track_id is None:
            self._open_second_observation(scan, now, heading)
        if progress is not None and progress >= math.pi-OPEN_EXIT_MARGIN_RAD:
            return self._open_first_pass(scan, now, control, 'compass_heading_reversed_in_loop')
        target = self.compact_target
        held = getattr(self.target_tracker, 'held', None)
        if (target is None and isinstance(held, dict) and held.get('track_id') == self.orbit_track_id
                and progress is not None):
            self.steering_target = self.natural_steering_target = self.servo
            self.reason = 'open_first_loop_missing_frame_hold'
            self._slew(now)
            return self._result(now)
        if not self._open_target_usable(target, scan, self.orbit_track_id, 1):
            if progress is not None and progress >= math.pi-OPEN_LATE_EXIT_RAD:
                return self._open_first_pass(scan, now, control, 'support_lost_near_heading_reversal')
            self._capture_compact_loss(scan, now, control, 'compact_target_lost_or_ambiguous')
            if self.begin_quality_wait(now, control, scan=scan):
                self.reason = 'route_orbit_target_recovery_wait'
            elif not self._begin_adopted_left_coast('compact_target_lost_or_ambiguous', now, control):
                self.begin_coast('compact_target_lost_or_ambiguous', now)
            self._slew(now)
            return self._result(now)
        loop = self._open_loop_geometry(scan, target, self.orbit_track_id, 1, 0.)
        if loop is None:
            # No compass or incomplete support this frame: keep the command.
            self.steering_target = self.natural_steering_target = self.servo
            self.reason = 'open_first_loop_geometry_hold'
            self._slew(now)
            return self._result(now)
        self.open_loop = loop
        second = self._open_confirmed_second(scan)
        if second is not None and progress >= OPEN_HANDOFF_MIN_PROGRESS_RAD:
            following = self._open_loop_geometry(scan, second, second['track_id'], -1, math.pi)
            self.open_next_loop = following
            # Round 81: compare the current-pose pursuits of both loops.
            if following is not None and following['current_pose_pwm'] <= loop['current_pose_pwm']:
                # Left arc - straight - right arc: the second loop now asks for
                # less left than the first; turning on only steepens the crossing.
                return self._open_first_pass(scan, now, control, 'second_loop_needs_less_left')
        # No right before the first pass (service rule); a release is allowed.
        self.natural_steering_target = loop['target_pwm']
        self.steering_target = max(NEUTRAL, loop['target_pwm'])
        self.reason = 'open_first_loop'
        self._slew(now)
        return self._result(now)

    def _open_first_pass(self, scan, now, control, basis):
        self.first_pass_evidence = {'source_seq': scan['seq'], 'source_at_ms': scan['at_ms'],
            'received_at': scan['received_at'], 'track_id': self.orbit_track_id, 'basis': basis,
            'compass': copy.deepcopy(self.compass.evidence), 'loop': copy.deepcopy(self.open_loop),
            'next_loop': copy.deepcopy(self.open_next_loop),
            'semantic_class': 'unknown', 'physical_cone_pass_certified': False, 'swept_path_certified': False}
        self.right_exit_since = now
        self._open_reset_loop(1)
        self._route_event('first_pass', scan, control, self.first_pass_evidence)
        if self._open_confirmed_second(scan) is not None:
            self._open_begin_second_loop(scan, now, control)
            return self._open_second_loop_update(scan, now, control, adopted=True)
        self.route_stage = 'open_transfer'
        self.steering_target = self.natural_steering_target = self.servo
        self.reason = 'open_first_pass_keep_command'
        return self._result(now)

    def _open_begin_second_loop(self, scan, now, control):
        target = self.second_target
        self.second_track_id = target['track_id']
        self.second_tracker.enable_close_maintenance(self.second_track_id)
        self._second_role_anchor = self._second_prior(target, scan)
        self._second_recovering = False
        self.second_target_handover_observed = True
        self.second_orbit_since, self.route_stage = now, 'second_loop'
        self._open_lost_since = None
        self.open_next_loop = None
        self._route_event('second_handover', scan, control, self.role_distinct_evidence)

    def _open_heading_hold(self, heading, axis):
        error = _wrap(axis-heading)
        return NEUTRAL+max(-OPEN_HEADING_HOLD_PWM, min(OPEN_HEADING_HOLD_PWM, round(LEFT_TRIAL_GAIN*error)))

    def _open_right_gate(self, target):
        if target < NEUTRAL and (self._route_center_ack is None or self.servo > NEUTRAL):
            return NEUTRAL  # right only after a current actual center ACK
        return target

    def _open_front_stop(self, scan):
        """Board straight ahead along the axis and its coast distance.

        A fallback only: while the car turns the axis ray samples different
        returns and the closing-rate estimate is meaningless (simulation:
        2.3 m/s at 1550), so the conservative nominal speed is used.
        """
        ahead = self._lane_front_range(scan)
        speed = self._lane_speed(scan, ahead)  # history only: the light lane continues it
        v = LANE_SPEED_UNKNOWN_MPS
        stop_at = FRONT_BODY_EXTENT_M+LANE_FINISH_GAP_M+v*v/(2*LANE_START_STOP_DECEL_MPS2)+v*LANE_REACTION_S
        return ahead, speed, stop_at

    def _open_first_center(self, scan):
        """First support position in this car frame (current or tracker-held), or None."""
        target = self.compact_target
        if (isinstance(target, dict) and target.get('track_id') == self.orbit_track_id
                and target.get('source_seq') == scan.get('seq') and isinstance(target.get('point_left_m'), list)):
            return target['point_left_m']
        held = getattr(self.target_tracker, 'held', None)
        if isinstance(held, dict) and held.get('track_id') == self.orbit_track_id:
            return held.get('point_left_m')
        return None

    def _open_second_observation(self, scan, now, heading):
        """One distinct compact support ahead inside the band along the reversed axis.

        It must also lie OPEN_SECOND_BEYOND_FIRST_M farther along that axis
        than the first support (task order: the second is passed after
        returning from the first), so a flickering first track can never
        become the second role.
        """
        e1, _ = _axis_frame(math.pi, heading)
        first = self._full_target_points(self.compact_target, scan, self.orbit_track_id)
        first_center = self._open_first_center(scan)
        first_along = (None if not isinstance(first_center, list) or len(first_center) != 2
                       else e1[0]*first_center[0]+e1[1]*first_center[1])
        ahead = []
        boards = None
        for target in _candidates(scan['ranges'], bearing_range_rad=(-math.pi/2, math.pi/2), tolerant=True,
                                  min_points=SECOND_ACQUISITION_MIN_POINTS, max_range_m=EXTENDED_MAX_RANGE_M):
            x, y = target['point_left_m']
            along, lateral = e1[0]*x+e1[1]*y, e1[0]*y-e1[1]*x
            # Cheap band test first; the board lines (the costliest part of a
            # transfer tick on the car, round 74) only for band candidates.
            if not (x > FRONT_BODY_EXTENT_M and FRONT_BODY_EXTENT_M < along <= OPEN_SECOND_BAND_REACH_M
                    and abs(lateral) <= OPEN_SECOND_BAND_HALF_M
                    and (first_along is None or along >= first_along+OPEN_SECOND_BEYOND_FIRST_M)):
                continue
            if boards is None:
                boards = self._scan_axis_lines(scan, heading)
            if (_on_board_run((x, y), boards, heading)
                    or _support_depth(scan['ranges'], target['support_bins']) > OPEN_SUPPORT_MAX_DEPTH_M):
                continue
            ahead.append((target, along, lateral))
        proof = None
        if len(ahead) == 1:
            target, along, lateral = ahead[0]
            bins = target['support_bins']
            points = [(scan['ranges'][i]*math.cos(math.radians(i)),
                       -scan['ranges'][i]*math.sin(math.radians(i))) for i in bins]
            separation = None if first is None else min(math.dist(a, b) for a in first for b in points)
            if first is None or (not set(bins).intersection(self.compact_target['support_bins'])
                                 and separation > MAX_DIAMETER_M):
                proof = {'source_seq': scan['seq'], 'source_at_ms': scan['at_ms'],
                    'source_received_at': scan['received_at'], 'first_track_id': self.orbit_track_id,
                    'first_support_visible': first is not None, 'second_support_bins': list(bins),
                    'actual_support_separation_m': separation, 'compass_heading_rad': heading,
                    'second_along_axis_m': along, 'second_lateral_m': lateral,
                    'first_along_axis_m': first_along, 'route_stage': self.route_stage,
                    'basis': 'single_support_ahead_in_band_along_reversed_axis_beyond_first',
                    'physical_identity_verified': False}
        if proof is None and self._second_proof_at is not None and 0 <= now-self._second_proof_at < SCAN_AGE_S:
            self.second_target = None
            self.second_acquisition_reason = 'second_role_proof_gap_held'
            return
        if proof is None:
            self._second_proof_at = None
            self.second_tracker = self._new_second_tracker()
            self.second_target = None
            self.role_distinct_evidence = None
            self.second_acquisition_reason = 'second_role_band_support_missing'
            return
        self._second_proof_at = now
        tracked = scan
        if self.first_pass_evidence is None:
            # Lookout inside the first loop: the first support is still within
            # the second tracker's own +-90 deg window and made it ambiguous
            # (simulation, 5 m course: four frames lost). The proof above has
            # already singled out the candidate; the tracker gets this frame
            # without the first support's own returns, or none at all.
            if first is None:
                self.second_acquisition_reason = 'second_lookout_first_support_not_current'
                return
            hidden = set(self.compact_target['support_bins'])
            hidden.update(self.compact_target.get('boundary_unknown_bins') or [])
            tracked = {**scan, 'ranges': [None if i in hidden else r for i, r in enumerate(scan['ranges'])]}
            self._open_lookout_seq = scan['seq']
        self.second_target = self.second_tracker.update(tracked, now, acquire=True, heading_rad=heading)
        self.role_distinct_evidence = proof
        self.second_acquisition_reason = self.second_tracker.reason

    def _open_channel_target(self, scan, heading, pose=None):
        """Pursuit point at the start divider's open end on the channel line, or None.

        ``pose`` (round 79, _response_pose): the command may come from the
        same construction ahead of that pose (_overshoot_guard); the returned
        point is the current-pose one.
        """
        reverse = heading-math.pi
        lines = [line for line in self._scan_axis_lines(scan, reverse)
                 if SIDE_BODY_EXTENT_M < line['lateral_m'] <= OPEN_CHANNEL_MAX_LATERAL_M
                 and line['along_m'][0] > FRONT_BODY_EXTENT_M]
        if not lines:
            return None
        divider = min(lines, key=lambda line: line['lateral_m'])
        width = self._start_lane_width if _number(self._start_lane_width) else 2*OPEN_CHANNEL_OFFSET_M[0]
        offset = max(OPEN_CHANNEL_OFFSET_M[0], min(OPEN_CHANNEL_OFFSET_M[1], width/2))
        lateral = divider['lateral_m']-offset
        u = (math.cos(-reverse), math.sin(-reverse))

        def channel_point(moved_along=0., moved_lateral=0.):
            along = max(OPEN_LANE_LOOKAHEAD_M[0],
                        min(OPEN_LANE_LOOKAHEAD_M[1], divider['along_m'][0]-moved_along),
                        OPEN_CHANNEL_SLOPE*abs(lateral-moved_lateral))+moved_along
            return (along*u[0]-lateral*u[1], along*u[1]+lateral*u[0])
        point = channel_point()
        curvature, pwm = self._guarded_pursuit(point, channel_point, pose, u)
        return {'divider_lateral_m': divider['lateral_m'], 'divider_open_end_along_m': divider['along_m'][0],
                'channel_offset_m': offset, 'pursuit_point_left_m': [round(v, 3) for v in point],
                'response_pose': None if pose is None else [round(v, 3) for v in pose],
                'pursuit_curvature_per_m': curvature, 'target_pwm': pwm}

    @staticmethod
    def _guarded_pursuit(point, build, pose, u):
        """(curvature, pwm) toward ``point``, or from ``pose`` toward the point
        ``build(along, lateral)`` makes ahead of it when _overshoot_guard says so."""
        curvature, pwm = _pursuit_pwm(point)
        if pose is None:
            return curvature, pwm
        moved = (pose[0]*u[0]+pose[1]*u[1], pose[1]*u[0]-pose[0]*u[1])  # along, left of the axis
        predicted = _pursuit_pwm(_in_pose(build(*moved), pose))
        return predicted if _overshoot_guard(pwm, predicted[1]) != pwm else (curvature, pwm)

    def _response_pose(self):
        """(x, y, heading change, horizon s) reached before a new command acts.

        Round 79: the measured yaw rate held for OPEN_STEER_LAG_S plus half the
        time to release the current command to centre, along an arc at
        OPEN_PREDICT_SPEED_MPS (car frame, left +).
        """
        rate = self.compass.rate if _number(self.compass.rate) else 0.
        horizon = OPEN_STEER_LAG_S+abs(self.servo-NEUTRAL)/OPEN_RELEASE_PWM_PER_S/2
        turn = rate*horizon
        distance = OPEN_PREDICT_SPEED_MPS*horizon
        if abs(turn) < 1e-6:
            return (distance, 0., 0., horizon)
        radius = distance/turn
        return (radius*math.sin(turn), radius*(1-math.cos(turn)), turn, horizon)

    def _open_transfer_update(self, scan, now, control):
        """After the first loop: reversed axis until the second support is confirmed."""
        if not self._route_adopted(control, now):
            return self._result(now)
        heading = self.compass.current(scan)
        if heading is None:
            self._begin_recovery_hold('open_compass_unavailable', now, control)
            return self._result(now)
        self._open_second_observation(scan, now, heading)
        if self._open_confirmed_second(scan) is not None:
            self._open_begin_second_loop(scan, now, control)
            # Round 87 (car run 87: confirmation plus the first loop plan took
            # .085 s of CPU in one tick, the next control gap .122 s): this tick
            # keeps the current command and the loop is planned on the next
            # control tick of this same scan (~25 ms later on the car, one
            # 20 ms tick in simulation); a newer scan plans it instead.
            self._second_plan_pending = (scan['seq'], scan['at_ms'])
            self.reason = 'open_second_loop_plan_next_tick'
            return self._result(now)
        pose = self._response_pose()
        channel = self._open_channel_target(scan, heading, pose)
        natural = (channel['target_pwm'] if channel is not None
                   else _overshoot_guard(self._open_heading_hold(heading, math.pi),
                                         self._open_heading_hold(heading+pose[2], math.pi)))
        ahead, speed, stop_at = self._open_front_stop(scan)
        self.open_feedback = {'source_seq': scan['seq'], 'stage': 'open_transfer', 'compass_heading_rad': heading,
            'channel': channel, 'nominal_target_pwm': natural, 'front_range_m': ahead, 'speed_mps': speed,
            'front_stop_range_m': stop_at, 'second_acquisition_reason': self.second_acquisition_reason}
        maturing = self._open_second_maturing(scan, ahead, stop_at)
        self.open_feedback['second_confirm_grace'] = maturing
        if ahead is not None and ahead <= stop_at and not maturing:
            self._begin_recovery_hold('open_board_ahead_without_second_support', now, control)
            return self._result(now)
        self.natural_steering_target = natural
        self.steering_target = self._open_right_gate(natural)
        self.reason = 'open_transfer_channel_entry' if channel is not None else 'open_transfer_reversed_axis_hold'
        self._slew(now)
        return self._result(now)

    def _open_second_loop_update(self, scan, now, control, *, adopted=False):
        """Second loop (clockwise, the support on the right); ends on the compass."""
        self._second_plan_pending = None
        if not adopted and not self._route_adopted(control, now):
            return self._result(now)
        heading = self.compass.current(scan)
        progress = None if heading is None else math.pi-heading
        if progress is not None and progress >= math.pi-OPEN_EXIT_MARGIN_RAD:
            return self._open_second_pass(scan, now, control, 'compass_heading_reversed_in_loop')
        target = self.second_target
        pose = self._response_pose()
        loop = (None if self._full_target_points(target, scan, self.second_track_id) is None
                else self._open_loop_geometry(scan, target, self.second_track_id, -1, math.pi, pose))
        if loop is None:
            if progress is not None and progress >= math.pi-OPEN_LATE_EXIT_RAD:
                return self._open_second_pass(scan, now, control, 'support_lost_near_heading_reversal')
            if self._open_lost_since is None:
                self._open_lost_since = now
            if now-self._open_lost_since < OPEN_LOST_HOLD_S:
                self.steering_target = self.natural_steering_target = self.servo
                self.reason = 'open_second_loop_missing_frame_hold'
                self._slew(now)
                return self._result(now)
            self._begin_recovery_hold('open_second_support_lost', now, control)
            return self._result(now)
        self._open_lost_since = None
        self.open_loop = loop
        if progress is not None and progress >= OPEN_LIGHT_HANDOFF_PROGRESS_RAD:
            # Right arc - straight - right arc: once the light-lane pursuit asks
            # for less right than the loop, continuing the loop only lowers the
            # car below the lane it must climb to (simulation, wide courses:
            # the light divider's open end was hit after a full loop).
            pursuit = self._open_light_pursuit(scan, heading, pose)
            if pursuit is not None:
                # Also hand over when continuing the adopted right arc would
                # carry us beyond the measured lane centre before heading is
                # aligned. Waiting only for "less right" misses the opposite
                # failure: a tighter entry is needed before the outer board.
                kappa = (NEUTRAL-self.servo)/ROUTE_PWM_PER_CURVATURE
                remaining_lateral = ((1-math.cos(heading))/kappa
                                     if kappa > 0 and 0 <= heading <= math.pi/2 else None)
                reaches_lane = (remaining_lateral is not None
                                and remaining_lateral >= pursuit[0]['center_lateral_m'])
                if pursuit[3] >= loop['target_pwm'] or reaches_lane:
                    return self._open_second_pass(scan, now, control,
                        'adopted_arc_reaches_light_lane' if reaches_lane else 'light_lane_pursuit_needs_less_right')
        self.natural_steering_target = loop['target_pwm']
        self.steering_target = self._open_right_gate(loop['target_pwm'])
        self.reason = 'open_second_loop'
        self._slew(now)
        return self._result(now)

    def _open_second_pass(self, scan, now, control, basis):
        self.second_pass_evidence = {'source_seq': scan['seq'], 'source_at_ms': scan['at_ms'],
            'received_at': scan['received_at'], 'track_id': self.second_track_id, 'basis': basis,
            'compass': copy.deepcopy(self.compass.evidence), 'loop': copy.deepcopy(self.open_loop),
            'semantic_class': 'unknown', 'physical_cone_pass_certified': False, 'swept_path_certified': False}
        self.route_stage = 'light_entry'
        self.light_since = self.light_turn_started = now
        self._open_reset_loop()
        self._route_event('second_pass', scan, control, self.second_pass_evidence)
        self.steering_target = self.natural_steering_target = self.servo
        self.reason = 'open_second_pass_keep_command'
        return self._result(now)

    def _open_lane_target(self, scan, heading):
        """Light-lane centre line (lateral, left +) from boards parallel to the axis.

        Boards: native finite walls parallel to the axis plus raw-return lines
        (_axis_lines). Outer = the leftmost board reaching ahead; inner = the
        next parallel board inside it within the lane-width range (its near
        end is the divider's open end); without one the start lane's measured
        width is the prior.
        """
        u = (math.cos(-heading), math.sin(-heading))
        n = (-u[1], u[0])
        walls = []
        for wall in scan.get('wall_candidates') or []:
            if (not _valid_wall(wall) or wall['support_span_m'] < OPEN_LANE_MIN_SPAN_M
                    or abs((wall['heading_left_rad']+heading+math.pi/2) % math.pi-math.pi/2) > OPEN_LANE_PARALLEL_RAD):
                continue
            ends = [(p['x_m'], p['y_m']) for p in (wall['support_start_left_m'], wall['support_end_left_m'])]
            walls.append({'lateral_m': sum(n[0]*p[0]+n[1]*p[1] for p in ends)/2,
                          'along_m': sorted(u[0]*p[0]+u[1]*p[1] for p in ends), 'source': 'native_wall'})
        walls += self._scan_axis_lines(scan, heading)
        outer = max((w for w in walls if w['lateral_m'] > SIDE_BODY_EXTENT_M and w['along_m'][1] > FRONT_BODY_EXTENT_M),
                    key=lambda w: w['lateral_m'], default=None)
        if outer is None:
            return None
        inner = max((w for w in walls if outer['lateral_m']-OPEN_LANE_WIDTH_M[1] <= w['lateral_m']
                     <= outer['lateral_m']-OPEN_LANE_WIDTH_M[0] and w['along_m'][1] > FRONT_BODY_EXTENT_M),
                    key=lambda w: w['lateral_m'], default=None)
        prior = self._start_lane_width if _number(self._start_lane_width) else OPEN_LANE_WIDTH_M[0]
        width = outer['lateral_m']-inner['lateral_m'] if inner is not None else prior
        return {'outer_lateral_m': outer['lateral_m'], 'outer_source': outer['source'],
                'inner_lateral_m': None if inner is None else inner['lateral_m'],
                'inner_source': None if inner is None else inner['source'],
                'inner_start_along_m': None if inner is None else inner['along_m'][0],
                'width_m': width, 'width_source': 'measured' if inner is not None else 'start_lane_prior',
                'center_lateral_m': outer['lateral_m']-width/2, 'walls': len(walls)}

    def _open_light_pursuit(self, scan, heading, pose=None, lateral=None):
        """Pursuit of the light-lane centre line: (lane, point, curvature, pwm) or None.

        ``pose`` (round 79, _response_pose): the command may come from the same
        construction ahead of that pose (_overshoot_guard); the point is the
        current-pose one (the path check models the arc from the current pose).
        ``lateral`` (round 102): a line parallel to it instead (_open_entry_shift).
        """
        lane = self._open_lane_target(scan, heading)
        if lane is None:
            return None
        u = (math.cos(-heading), math.sin(-heading))
        lateral = lane['center_lateral_m'] if lateral is None else lateral

        def lane_point(moved_along=0., moved_lateral=0.):
            start = lane['inner_start_along_m']
            start = None if start is None else start-moved_along
            along = (OPEN_LANE_LOOKAHEAD_M[1] if start is None or start <= FRONT_BODY_EXTENT_M
                     else max(OPEN_LANE_LOOKAHEAD_M[0], min(OPEN_LANE_LOOKAHEAD_M[1], start)))
            # As the line is approached, shorten the pursuit horizon so the
            # remaining cross-lane motion is removed before releasing the turn.
            along = min(along, max(OPEN_LANE_LOOKAHEAD_M[0], OPEN_LOOKAHEAD_M+abs(lateral-moved_lateral)))
            along += moved_along
            return (along*u[0]-lateral*u[1], along*u[1]+lateral*u[0])
        point = lane_point()
        curvature, pwm = self._guarded_pursuit(point, lane_point, pose, u)
        return lane, point, curvature, pwm

    def _open_entry_path_hit(self, scan, point, heading, reach, fallback=None):
        """Nearest return on the pursuit's own path, as path length, or None.

        The axis ray from a car still below the lane line meets the divider's
        open end that the pursuit is about to pass above; with the ESC lag in
        the stopping range that held the car short of the lane for good
        (simulation, measured car). The path is the pure-pursuit arc to its
        point on the lane line, then that line along the axis; a return counts
        inside the body strip (side + margin) round it, up to ``reach``
        beyond the body front. Without raw returns: ``fallback`` (axis ray).
        """
        if not isinstance(scan.get('ranges'), list):
            return fallback
        half = SIDE_BODY_EXTENT_M+OPEN_APPROACH_MARGIN_M
        limit = reach+FRONT_BODY_EXTENT_M
        px, py = point
        curvature = 2*py/max(1e-6, px*px+py*py)
        if abs(curvature) < 1e-3:
            arc_to_point = math.hypot(px, py)
        else:
            radius = 1/curvature
            arc_to_point = abs(radius)*((math.atan2(px, radius-py) if curvature > 0
                                         else math.atan2(px, py-radius)) % (2*math.pi))
        u = (math.cos(-heading), math.sin(-heading))
        best = None
        for x, y in _scan_points(scan['ranges'], limit+1.):
            hit = None
            if abs(curvature) < 1e-3:
                along, offset = x, y
            else:
                radius = 1/curvature
                offset = abs(radius)-math.hypot(x, y-radius)
                along = abs(radius)*((math.atan2(x, radius-y) if curvature > 0
                                      else math.atan2(x, y-radius)) % (2*math.pi))
            if 0 < along <= min(limit, arc_to_point) and abs(offset) <= half:
                hit = along
            else:
                dx, dy = x-px, y-py
                along = dx*u[0]+dy*u[1]
                if 0 <= along <= limit-arc_to_point and abs(dy*u[0]-dx*u[1]) <= half:
                    hit = arc_to_point+along
            if hit is not None:
                best = hit if best is None else min(best, hit)
        return best

    def _open_entry_free_lateral(self, scan, heading, lane, reach):
        """Lateral (left +) of a lane-axis line whose body strip is free ahead, or None.

        Round 102 (car run 101; the user's photo and the car's lidar facing the
        light, 2026-10-22): the traffic-light gantry's two legs stand in the
        light-lane mouth (.96 m apart in a 1.14 m lane). The divider not yet in
        view, the start-lane width put the line .12 m out and the path met the
        outer leg 2.9 m ahead: the car held at 1.5 m/s and turned past the axis.
        Every return from the near pursuit horizon to the path reach along the
        axis blocks the body strip (side + margin) round it. Of the free gaps
        within half a lane of the measured line (never outside the outer
        board) the nearest one at least OPEN_LANE_SHIFT_MIN_GAP_M wide is taken
        (else the nearest; ties: the outer side); in it a lane's span from its
        near edge is centred (between two returns: the middle of the gap).
        """
        if not isinstance(scan.get('ranges'), list):
            return None
        half = SIDE_BODY_EXTENT_M+OPEN_APPROACH_MARGIN_M
        limit = reach+FRONT_BODY_EXTENT_M
        u = (math.cos(-heading), math.sin(-heading))
        blocks = []
        for x, y in _scan_points(scan['ranges'], limit+1.):
            if OPEN_LANE_LOOKAHEAD_M[0] <= x*u[0]+y*u[1] <= limit:
                lateral = y*u[0]-x*u[1]
                blocks.append((lateral-half, lateral+half))
        blocks.sort()
        gaps, edge = [], -math.inf
        for lo, hi in blocks:
            if lo > edge:
                gaps.append((edge, lo))
            edge = max(edge, hi)
        gaps.append((edge, math.inf))
        centre, width = lane['center_lateral_m'], lane['width_m']
        top = lane['outer_lateral_m']-half
        low, high = centre-width/2, min(centre+width/2, top)
        best = None
        for lo, hi in gaps:
            hi = min(hi, top)
            if max(lo, low) >= min(hi, high):
                continue
            distance = 0. if lo <= centre <= hi else min(abs(lo-centre), abs(hi-centre))
            key = (hi-lo < OPEN_LANE_SHIFT_MIN_GAP_M, distance, -hi)
            if best is None or key < best[0]:
                best = (key, lo, hi)
        if best is None:
            return None
        _, lo, hi = best
        span = max(0., width-2*half)
        if centre > hi:
            a, b = max(lo, hi-span), hi
        elif centre < lo:
            a, b = lo, min(hi, lo+span)
        else:
            a, b = max(lo, centre-span/2), min(hi, centre+span/2)
        return (a+b)/2

    def _open_entry_shift(self, scan, heading, lane, pose, stop_at, fallback, blocked):
        """Round 102: the light-entry pursuit on a free parallel line.

        (pursuit, path hit, evidence) when the moved path is clear of the stop
        range, else None (the stage holds as before).
        """
        lateral = self._open_entry_free_lateral(scan, heading, lane, stop_at)
        moved = None if lateral is None else self._open_light_pursuit(scan, heading, pose, lateral)
        if moved is None:
            return None
        ahead = self._open_entry_path_hit(scan, moved[1], heading, stop_at, fallback)
        if ahead is not None and ahead <= stop_at:
            return None
        return moved, ahead, {'line_lateral_m': lane['center_lateral_m'], 'lateral_m': lateral,
                              'blocked_front_range_m': blocked, 'basis': 'nearest_free_body_strip_along_axis'}

    def _open_light_lane_entry(self, scan, heading, lane):
        """The same measured lane admission for driving and paused recovery."""
        walls = corridor_walls(scan)
        # Inside the lane: the outer board on the left, the inner board (the
        # divider) on the right and within the near pursuit horizon, nearly
        # aligned. Requiring the divider alongside can deadlock before its
        # open end when the end-board stopping distance is reached first.
        inside = (lane['inner_lateral_m'] is not None
                  and lane['inner_lateral_m'] <= -(SIDE_BODY_EXTENT_M+OPEN_LANE_INSIDE_MARGIN_M)
                  and lane['outer_lateral_m'] >= SIDE_BODY_EXTENT_M+OPEN_LANE_INSIDE_MARGIN_M
                  and lane['inner_start_along_m'] <= OPEN_LANE_LOOKAHEAD_M[0])
        fitted = (walls is not None and OPEN_LANE_WIDTH_M[0] <= walls['width_m'] <= OPEN_LANE_WIDTH_M[1]
                  and abs(lane['center_lateral_m']) <= walls['width_m']/2-SIDE_BODY_EXTENT_M)
        if abs(heading) <= OPEN_LANE_ENTRY_HEADING_RAD and (inside or fitted):
            return {'source_seq': scan['seq'], 'walls': walls,
                'lane_target': lane, 'compass_heading_rad': heading,
                'basis': ('car_between_outer_board_and_near_divider_lines' if inside
                          else 'two_parallel_boards_round_car_on_light_lane_line')}
        return None

    def _open_light_entry_update(self, scan, now, control):
        """Turn onto the light-lane centre line along the outer boundary."""
        if not self._route_adopted(control, now):
            return self._result(now)
        heading = self.compass.current(scan)
        pose = self._response_pose()
        pursuit = None if heading is None else self._open_light_pursuit(scan, heading, pose)
        if pursuit is None:
            self._begin_recovery_hold('open_light_lane_unmeasured', now, control)
            return self._result(now)
        lane, point, curvature, natural = pursuit
        entry = self._open_light_lane_entry(scan, heading, lane)
        if entry is not None:
            return self._begin_light_lane(scan, now, control, entry)
        walls = corridor_walls(scan)
        axis_ahead, speed, stop_at = self._open_front_stop(scan)
        ahead = self._open_entry_path_hit(scan, point, heading, stop_at, axis_ahead)
        shift = None
        if ahead is not None and ahead <= stop_at:
            shift = self._open_entry_shift(scan, heading, lane, pose, stop_at, axis_ahead, ahead)
            if shift is not None:
                (lane, point, curvature, natural), ahead, shift = shift
        self.light_feedback = {'source_seq': scan['seq'], 'stage': 'light_entry', 'compass_heading_rad': heading,
            'lane_target': lane, 'pursuit_point_left_m': [round(v, 3) for v in point],
            'pursuit_curvature_per_m': curvature, 'nominal_target_pwm': natural, 'walls': walls,
            'front_range_m': ahead, 'axis_front_range_m': axis_ahead, 'speed_mps': speed,
            'front_stop_range_m': stop_at, 'line_shift': shift, 'swept_path_certified': False}
        if ahead is not None and ahead <= stop_at:
            self._begin_recovery_hold('open_board_ahead_before_light_lane', now, control)
            return self._result(now)
        self.natural_steering_target = natural
        self.steering_target = self._open_right_gate(natural)
        self.reason = 'open_light_entry'
        self._slew(now)
        return self._result(now)

    def _right_exit_target(self, scan, now):
        """Use a current native forward corridor, never a guessed second cone."""
        values = scan.get('corridor_candidates')
        if (not isinstance(values, list) or len(values) > 24
                or any(not _valid_candidate(c, lateral_margin=SIDE_BODY_EXTENT_M+MANEUVER_BODY_CLEARANCE_M)
                       for c in values)):
            return None, 'right_exit_invalid_corridor'
        forward = [c for c in values if -math.pi/2 < c['heading_left_rad'] < math.pi/2]
        if len(forward) > 1 and self.continue_route:
            # Trial34 604 / trial45: the native fit can report the same
            # corridor twice a few degrees apart. Aliases within the existing
            # window-merge tolerances are one axis (keep the best-supported
            # one); a different axis (corner/T) stays ambiguous.
            base = forward[0]
            if all(abs(_wrap(c['heading_left_rad']-base['heading_left_rad'])) <= PAIR_MERGE_HEADING_RAD
                   and abs(c['center_offset_left_m']-base['center_offset_left_m']) <= PAIR_MERGE_OFFSET_M
                   and abs(c['width_m']-base['width_m']) <= max(.15, MAX_EXIT_WIDTH_CHANGE_FRACTION*base['width_m'])
                   for c in forward[1:]):
                forward = [max(forward, key=lambda c: (c.get('left_wall_points', 0)+c.get('right_wall_points', 0),
                                                       -c.get('fit_error_m', 0.)))]
        if len(forward) > 1:
            return None, 'right_exit_corridor_ambiguous'
        if forward:
            c = {**forward[0], 'source_kind': 'native_corridor'}
        elif self.continue_route:
            # A nearby compact object or a wall that ends inside one side
            # sector can defeat the whole-sector fit while the same frame still
            # holds two parallel native wall windows (trials38/39).
            c, ambiguous = self._paired_window_corridor(scan)
            if ambiguous:
                return None, 'right_exit_corridor_ambiguous'
            if c is None or not _valid_candidate(c, lateral_margin=SIDE_BODY_EXTENT_M+MANEUVER_BODY_CLEARANCE_M):
                return None, 'right_exit_corridor_missing'
        else:
            return None, 'right_exit_corridor_missing'
        previous = self.right_exit_geometry
        if previous is not None:
            # Bound each new corridor against the preceding real observation.
            dt = (scan['at_ms']-previous['source_at_ms'])/1000
            receive_dt = scan['received_at']-previous['received_at']
            if (dt <= 0 or dt >= SCAN_AGE_S
                    or (self.continue_route and (not 0 < receive_dt < SCAN_AGE_S
                        or now-previous['received_at'] >= SCAN_AGE_S
                        or abs(dt-receive_dt) > .15))
                    or abs(c['heading_left_rad']-previous['heading_left_rad']) > math.radians(25)
                    or abs(c['center_offset_left_m']-previous['center_offset_left_m']) > .30
                    or abs(c['width_m']-previous['width_m']) > max(.15, .1*previous['width_m'])):
                return None, 'right_exit_corridor_identity_changed'
        lookahead = max(FRONT_BODY_EXTENT_M+REAR_BODY_EXTENT_M,
                        min(ROUTE_ALIGN_LOOKAHEAD_MAX_M, c['width_m']) if self.continue_route
                        else min(1., c['width_m']/2))
        heading_rate = 0.
        if self.continue_route and previous is not None:
            # Trial46 overshot both sides with a 1 m lookahead: damp with the
            # observed corridor-heading rate (the car yawing left makes it fall).
            heading_rate = (_wrap(c['heading_left_rad']-previous['heading_left_rad'])
                            / ((scan['at_ms']-previous['source_at_ms'])/1000))
        approach = self._approach_offset(c)
        error = (c['heading_left_rad']+math.atan2(c['center_offset_left_m']+approach, lookahead)
                 + ROUTE_ALIGN_DAMPING_S*heading_rate)
        # Trial45: after the first pass the corridor axis was 9.4 deg LEFT and
        # its center .66 m left (trial43: axis 10 deg right). The exit side is
        # observed, never assumed: route mode pursues the current corridor
        # center line either way; right still needs a current center ACK.
        upper = SERVO_MAX if self.continue_route else NEUTRAL
        target = max(RIGHT_EXIT_MIN_PWM, min(upper, NEUTRAL+round(LEFT_TRIAL_GAIN*error)))
        self.right_exit_geometry = {**copy.deepcopy(c), 'source_seq': scan['seq'],
            'source_at_ms': scan['at_ms'], 'received_at': scan['received_at'],
            'lookahead_m': lookahead, 'steering_target_pwm': target, 'heading_rate_rad_s': heading_rate,
            'approach_offset_left_m': approach,
            'source': 'current_native_corridor', 'second_target_confirmed': False}
        self._right_exit_last_receive, self._right_exit_last_publication = scan['received_at'], scan['at_ms']
        return target, None

    def _approach_offset(self, corridor):
        """Left offset of the steered line from the lane center (round 60).

        Only in route mode between the first pass and the second handover
        (exit alignment, corridor following); zero everywhere else, including
        the second orbit's lane measurement and return target.
        """
        if not (self.continue_route and self.first_pass_evidence is not None
                and self.route_stage in ('right_align', 'corridor_follow')):
            return 0.
        room = (corridor['width_m']/2-(SIDE_BODY_EXTENT_M+MANEUVER_BODY_CLEARANCE_M)
                - SECOND_APPROACH_WALL_MARGIN_M)
        return max(0., min(SECOND_APPROACH_LEFT_M, room))

    @staticmethod
    def _corridor_body_gap(candidate):
        heading = candidate['heading_left_rad']
        projections = [-math.sin(heading)*x+math.cos(heading)*y
                       for x in (-REAR_BODY_EXTENT_M, FRONT_BODY_EXTENT_M)
                       for y in (-SIDE_BODY_EXTENT_M, SIDE_BODY_EXTENT_M)]
        offset, half = candidate['center_offset_left_m'], candidate['width_m']/2
        return min(half+offset-max(projections), min(projections)-(offset-half))

    def _route_motor(self):
        """Commanded drive PWM: slow from the first-target handover in route mode.

        Trials 49-51 (rebuilt paths): ~1.3-1.6 m/s at 1560 through the first
        U-turn and ~.5 m/s^2 rolling decel; 1550 measured ~.6-1.1 m/s. The
        left turn keeps 1560; a powered resume keeps its 1560 ACK sequence.
        """
        slow_stage = ((self.handover_observed and self.route_stage == 'first_target')
                      or (self.first_pass_evidence is not None and self.route_stage in POST_PASS_STAGES))
        if (self.continue_route and self.phase == 'drive' and self.route_stage == 'light_lane'
                and not self.quality_resume_pending):
            return self.lane_motor  # slow light lane: 1550 / neutral coast / 1560 restart
        if self.continue_route and self.phase == 'drive' and self.route_stage == 'start_lane':
            # Round 64: the start lane at 1560; slow from the first approach on.
            return self._start_motor
        return (ROUTE_SLOW_MOTOR if (self.continue_route and self.phase == 'drive' and slow_stage
                                     and not self.quality_resume_pending)
                else TRIAL_MOTOR)

    def _route_adopted(self, control, now):
        if control.get('servo', NEUTRAL) > NEUTRAL:
            self._route_center_ack = None
            self.second_center_ack = None
            if self.servo < NEUTRAL:
                self.lock('route_actual_steering_crossed_center')
                return False
        motors = ((NEUTRAL,)+DRIVE_MOTORS if self.route_stage in ('light_lane', 'start_lane')
                  else DRIVE_MOTORS)
        adopted = (control.get('motor') in motors and control.get('servo') == self.servo
                   and control.get('command_acked') is True)
        if not adopted:
            if self.right_exit_adoption_wait_since is None:
                self.right_exit_adoption_wait_since = now
            if now-self.right_exit_adoption_wait_since >= SCAN_AGE_S:
                self.begin_coast('route_adoption_unconfirmed', now)
            else:
                self.steering_target = self.servo
                self.reason = 'route_waiting_adopted_command'
            return False
        self.right_exit_adoption_wait_since = None
        if self.servo > NEUTRAL:
            self._route_center_ack = None
        elif self.servo == NEUTRAL and (control.get('motor') in DRIVE_MOTORS
                                        or (self.route_stage == 'light_lane' and control.get('motor') == NEUTRAL)):
            # A center ACK is recorded under drive power (service check); a
            # neutral coasting pulse keeps the previous one. Round 82: in the
            # light lane an ACKed centre while coasting counts too (car runs
            # 80/81: the lane coasts to the finish, right stayed blocked for
            # .6-2.8 s, the car drifted to ~.08 m from / into the left board).
            self._route_center_ack = {k: control.get(k) for k in ('servo', 'motor', 'tick', 'seq')}
        return True

    def _right_exit_update(self, scan, now, control):
        if now-self.right_exit_since+1e-9 >= RIGHT_EXIT_MAX_S:
            if self.continue_route:
                self._begin_recovery_hold('first_target_right_exit_trial_timeout', now, control)
            else:
                self.begin_coast('first_target_right_exit_trial_timeout', now)
            self._slew(now)
            return self._result(now)
        target, rejection = self._right_exit_target(scan, now)
        if target is None:
            # Missing geometry only retains an already selected command under
            # its original observation lease; it cannot advance a right turn.
            lease_receive, lease_publication = self._right_exit_last_receive, self._right_exit_last_publication
            if (lease_receive is None and self.continue_route and self.servo > NEUTRAL
                    and isinstance(self.first_pass_evidence, dict)):
                # Before any corridor: only the actual first-pass frame's own
                # lease, and only to keep releasing toward neutral.
                lease_receive = self.first_pass_evidence.get('received_at')
                lease_publication = self.first_pass_evidence.get('source_at_ms')
            # Trial45: one ambiguous frame coasted the exit straight into a
            # wall; route mode gives it the same lease as a missing corridor.
            held = ((rejection == 'right_exit_corridor_missing'
                     or (self.continue_route and rejection == 'right_exit_corridor_ambiguous'))
                    and _number(lease_receive) and _number(lease_publication)
                    and now-lease_receive < SCAN_AGE_S
                    and (scan['at_ms']-lease_publication)/1000 < SCAN_AGE_S)
            if held and self.continue_route and self.servo > NEUTRAL:
                # Trial39 489: holding residual left while the corridor was
                # missing continued the orbit. Release toward neutral only;
                # missing geometry still never authorizes a right command.
                self.steering_target = NEUTRAL
                self.reason = 'right_exit_corridor_missing_release_left'
                self._slew(now)
            elif held:
                self.steering_target = self.servo
                self.reason = 'right_exit_corridor_missing_hold'
            elif self.continue_route:
                self._begin_recovery_hold(rejection, now, control)
                self._slew(now)
            else:
                self.begin_coast(rejection, now)
                self._slew(now)
            return self._result(now)
        if self.continue_route and not self._route_adopted(control, now):
            return self._result(now)
        adopted = (control.get('motor') in (DRIVE_MOTORS if self.continue_route else (TRIAL_MOTOR,))
                   and control.get('servo') == self.servo and control.get('command_acked') is True)
        if not adopted:
            if self.right_exit_adoption_wait_since is None:
                self.right_exit_adoption_wait_since = now
            if now-self.right_exit_adoption_wait_since+1e-9 >= SCAN_AGE_S:
                self.begin_coast('right_exit_adoption_unconfirmed', now)
                self._slew(now)
            else:
                self.steering_target = self.servo
                self.reason = 'right_exit_waiting_adopted_command'
            return self._result(now)
        self.right_exit_adoption_wait_since = None
        if self.right_exit_center_ack is None:
            if (self.servo == NEUTRAL and control.get('servo') == NEUTRAL
                    and control.get('motor') in (DRIVE_MOTORS if self.continue_route else (TRIAL_MOTOR,))
                    and control.get('command_acked') is True):
                self.right_exit_center_ack = {k: control.get(k) for k in ('servo', 'motor', 'tick', 'seq')}
                self._route_center_ack = dict(self.right_exit_center_ack)
            elif not self.continue_route:
                target = NEUTRAL
        natural = target
        if self.continue_route and target < NEUTRAL and (
                self.right_exit_center_ack is None or self._route_center_ack is None or self.servo > NEUTRAL):
            # Left toward the observed center line is direct; right still
            # needs a current actual center ACK after any left command.
            target = NEUTRAL
        if self.continue_route:
            # Trial46: the second support was already visible during exit
            # alignment; the same role proof and confirmation apply here.
            self._second_role_observation(scan, now)
            if self.second_target is not None and self.second_target.get('confirmed') is True:
                return self._begin_second_orbit(scan, now, control)
            c = self.right_exit_geometry
            aligned = (abs(c['heading_left_rad']) <= ROUTE_ALIGN_HEADING_RAD
                       and self._corridor_body_gap(c) >= MANEUVER_BODY_CLEARANCE_M)
            if aligned and self._mature_observation(self._route_alignment_history, scan, .25):
                self.route_alignment_evidence = {**copy.deepcopy(c),
                    'body_normal_gap_m': self._corridor_body_gap(c),
                    'physical_alignment_certified': False}
                self.route_stage = 'corridor_follow'
                self._route_event('corridor_follow', scan, control, self.route_alignment_evidence)
                self.steering_target = self.servo
                self.reason = 'route_corridor_alignment_observed'
                return self._result(now)
            if not aligned:
                self._route_alignment_history.clear()
        self.steering_target, self.natural_steering_target = target, natural
        self.reason = ('first_target_exit_alignment_left' if target > NEUTRAL
                       else 'first_target_right_exit_tracking' if self.right_exit_center_ack is not None
                       else 'first_target_right_exit_waiting_center_ack')
        self._slew(now)
        return self._result(now)

    def _start_second_identity_recovery(self):
        self._clear_second_return()
        if self._second_role_anchor is None:
            return
        self._second_recovering = True
        self._second_recovery_last = None
        # Acquisition after role handover includes the right rear, not the
        # passed first target's left-rear sector. Role proof below is still
        # required; this does not acquire any arbitrary right-side object.
        self.second_tracker.widen_acquisition((-math.pi, math.pi/2))

    def _second_recovery_observation(self, scan, now, *, fallback=False):
        """Fresh reconfirmation shared by drive loss and neutral-wait loss.

        The last full second observation is an identity prior, not current
        support. Its association gate widens with the time since it, capped
        (SECOND_RECOVERY_MAX_SLACK_S). A new ID needs the tracker's original
        multi-frame confirmation, with every current candidate distinct from
        any currently observed first support. Returns True when exactly one
        proven support advanced the tracker this frame. With ``fallback`` a
        frame without one leaves the tracker untouched for the caller's own
        corridor role proof (driving: the support may now be ahead again).
        """
        anchor = self._second_role_anchor
        if not self._second_recovering or anchor is None or self.first_pass_evidence is None:
            return False
        previous = self._second_recovery_last
        reference = (previous if isinstance(previous, dict)
                     and self._observation_lease(scan, now, previous.get('source_at_ms'),
                                                 previous.get('source_received_at')) else anchor)
        candidates = _candidates(scan['ranges'], bearing_range_rad=(-math.pi, math.pi/2), tolerant=True,
                                 min_points=SECOND_ACQUISITION_MIN_POINTS)
        first = self.compact_target
        first_bins = (set(first['support_bins']) if isinstance(first, dict)
                      and first.get('track_id') == self.orbit_track_id
                      and first.get('source_seq') == scan.get('seq') else set())
        first_points = [(scan['ranges'][i]*math.cos(math.radians(i)),
                         -scan['ranges'][i]*math.sin(math.radians(i))) for i in first_bins]
        slack = min(SECOND_RECOVERY_MAX_SLACK_S,
                    max(0., (scan['at_ms']-reference['source_at_ms'])/1000-.1))
        heading = self._tracking_heading(scan)
        landmark = self._first_landmark(scan)
        prior_landmark = reference.get('first_landmark_left_m')
        if (heading is not None and _number(reference.get('compass_heading_rad'))
                and landmark is not None and isinstance(prior_landmark, list)):
            # Round 79: the car also moved; the first support is the landmark.
            turn = -(heading-reference['compass_heading_rad'])
            c, s = math.cos(turn), math.sin(turn)
            dx = reference['point_left_m'][0]-prior_landmark[0]
            dy = reference['point_left_m'][1]-prior_landmark[1]
            point = [landmark[0]+c*dx-s*dy, landmark[1]+s*dx+c*dy]
            reference = {**reference, 'point_left_m': point, 'bearing_left_rad': math.atan2(point[1], point[0])}
            slack = min(slack, SECOND_LANDMARK_SLACK_S)
            self.second_recovery_prior = {'basis': 'first_support_landmark', 'source_seq': scan['seq'],
                                          'point_left_m': point, 'slack_s': slack}
        elif heading is not None and _number(reference.get('compass_heading_rad')):
            # Open course: the prior turned with the car (round 64).
            reference = rotated_observation(reference, heading-reference['compass_heading_rad'])
            self.second_recovery_prior = {'basis': 'compass_rotation', 'source_seq': scan['seq'],
                                          'point_left_m': reference['point_left_m'], 'slack_s': slack}
        def proven(candidate):
            if (not _associated(candidate, reference, slack)
                    or first_bins.intersection(candidate['support_bins'])):
                return False
            points = [(scan['ranges'][i]*math.cos(math.radians(i)),
                       -scan['ranges'][i]*math.sin(math.radians(i))) for i in candidate['support_bins']]
            return not first_points or min(math.dist(a, b) for a in first_points for b in points) > MAX_DIAMETER_M
        matches = [candidate for candidate in candidates if proven(candidate)]
        if len(matches) != 1 and fallback:
            self.second_acquisition_reason = 'second_role_recovery_unproven_corridor_proof_next'
            return False
        if len(matches) != 1:
            if candidates:
                # An unrelated/ambiguous current object cannot advance a new
                # identity, even if acquire=False would keep associating it.
                self.second_tracker._forget('second_role_recovery_unproven')
                self._second_recovery_last = None
            self.second_tracker.update(scan, now, acquire=False, heading_rad=heading)
            self.second_target = None
            self.second_acquisition_reason = 'second_role_recovery_current_support_unproven'
            return False
        # Only the proven support may start or continue the identity: the
        # first support in the same sector is not an ambiguity (round 79).
        target = self.second_tracker.update(scan, now, acquire=True, heading_rad=heading,
                                            only_support_bins=matches[0]['support_bins'])
        self.second_target = target
        if target is not None and not target['tracking_only_boundary_gap']:
            self._second_recovery_last = self._second_prior(target, scan)
        self.second_acquisition_reason = self.second_tracker.reason
        if (target is None or set(target['support_bins']) != set(matches[0]['support_bins'])
                or self._full_target_points(target, scan, target['track_id']) is None):
            return True
        old_id = self.second_track_id
        self.second_track_id = target['track_id']
        self.second_tracker.enable_close_maintenance(self.second_track_id)
        self._second_recovering = False
        self.second_target_handover_observed = True
        self._second_role_anchor = self._second_prior(target, scan)
        self.second_orbit_since = now  # new confirmed attempt; cumulative deadline is unchanged
        self._clear_target_recovery_history()
        self._clear_second_return()
        self.second_center_ack = None
        self.second_rebind_evidence = {
            'old_track_id': old_id, 'new_track_id': target['track_id'],
            'source_seq': scan['seq'], 'source_at_ms': scan['at_ms'],
            'source_received_at': scan['received_at'], 'anchor_source_seq': anchor['source_seq'],
            'support_bins': list(target['support_bins']),
            'basis': 'current_confirmed_support_associated_with_last_complete_second_observation',
            'physical_identity_verified': False, 'observed_rear_pass': False}
        return True

    def _first_landmark(self, scan):
        """Current confirmed first-support centre in this scan (left frame), or None."""
        first = self.compact_target
        if (self.orbit_track_id is not None and isinstance(first, dict)
                and first.get('track_id') == self.orbit_track_id and first.get('confirmed') is True
                and first.get('source_seq') == scan.get('seq') and isinstance(first.get('point_left_m'), list)):
            return list(first['point_left_m'])
        return None

    def _second_prior(self, observation, scan):
        """Copy of a second observation, with the first support seen in the same scan."""
        prior = copy.deepcopy(observation)
        landmark = self._first_landmark(scan)
        if isinstance(prior, dict) and landmark is not None and prior.get('source_seq') == scan.get('seq'):
            prior['first_landmark_left_m'] = landmark
        return prior

    def _update_second_tracking(self, scan, now):
        if self._open_lookout_seq is not None and scan.get('seq') == self._open_lookout_seq:
            # Round 64: this publication already went to the tracker without the
            # first support's returns (first-loop lookout); the same seq with
            # different content would read as a changed duplicate and drop it.
            return
        if self._second_recovering:
            self._second_recovery_observation(scan, now)
            return
        self.second_target = self.second_tracker.update(scan, now, acquire=False,
                                                        heading_rad=self._tracking_heading(scan))
        if self._full_target_points(self.second_target, scan, self.second_track_id) is not None:
            self._second_role_anchor = self._second_prior(self.second_target, scan)
        elif self.second_target is None and not self._target_gap_held(
                self.second_tracker, self.second_track_id, scan, now):
            self._start_second_identity_recovery()

    def _second_role_observation(self, scan, now):
        """One distinct compact support ahead inside the current corridor.

        Trial43: the second target appeared at the left front while the first
        was already behind; the old right-half-only, first-still-visible proof
        never matched. The first support has a recorded pass behind the rear
        and the car follows a forward corridor, so a support ahead of the body
        front inside that corridor is a different object. If the first support
        is still fully visible it must also share no bins and be separated.
        """
        # Driving after a second-role loss: the anchored recovery first; when
        # it proves nothing this frame, the original corridor proof below may
        # acquire the support again ahead (trial52 class: the car moved on).
        if self._second_recovering and self._second_recovery_observation(scan, now, fallback=True):
            return
        c = self.right_exit_geometry
        proof = None
        if self.first_pass_evidence is not None and isinstance(c, dict):
            first = self._full_target_points(self.compact_target, scan, self.orbit_track_id)
            heading = c['heading_left_rad']
            bound = c['width_m']/2-MANEUVER_BODY_CLEARANCE_M
            ahead = []
            for target in _candidates(scan['ranges'], bearing_range_rad=(-math.pi/2, math.pi/2), tolerant=True,
                                      min_points=SECOND_ACQUISITION_MIN_POINTS):
                x, y = target['point_left_m']
                along = math.cos(heading)*x+math.sin(heading)*y
                normal = -math.sin(heading)*x+math.cos(heading)*y
                if (x > FRONT_BODY_EXTENT_M and along > FRONT_BODY_EXTENT_M
                        and abs(normal-c['center_offset_left_m']) < bound):
                    ahead.append((target, along, normal))
            if len(ahead) == 1:
                target, along, normal = ahead[0]
                bins = target['support_bins']
                points = [(scan['ranges'][i]*math.cos(math.radians(i)),
                           -scan['ranges'][i]*math.sin(math.radians(i))) for i in bins]
                separation = (None if first is None else
                              min(math.dist(a, b) for a in first for b in points))
                distinct = first is None or (
                    not set(bins).intersection(self.compact_target['support_bins'])
                    and separation > MAX_DIAMETER_M)
                if distinct:
                    proof = {'source_seq': scan['seq'], 'source_at_ms': scan['at_ms'],
                        'source_received_at': scan['received_at'], 'first_track_id': self.orbit_track_id,
                        'first_support_visible': first is not None,
                        'first_support_bins': (list(self.compact_target['support_bins'])
                                               if first is not None else None),
                        'first_pass_source_seq': self.first_pass_evidence.get('source_seq'),
                        'second_support_bins': list(bins), 'actual_support_separation_m': separation,
                        'corridor_heading_left_rad': heading, 'second_along_m': along,
                        'second_normal_m': normal, 'second_side': 'left' if target['point_left_m'][1] > 0 else 'right',
                        'basis': 'single_support_ahead_inside_current_corridor_after_first_pass',
                        'physical_identity_verified': False}
        if (proof is None and self._second_proof_at is not None
                and 0 <= now-self._second_proof_at < SCAN_AGE_S):
            # Trial52: the second support flickered (dropout band) and every
            # missed frame discarded its tracker. Within one lease a frame
            # without proof neither advances (counts) nor discards the tracker.
            self.second_target = None
            self.second_acquisition_reason = 'second_role_proof_gap_held'
            return
        if proof is None:
            # acquire=False alone would still mature an existing unconfirmed
            # association. No role proof means discard that entire interval.
            self._second_proof_at = None
            if not self._second_recovering:
                self.second_tracker = self._new_second_tracker()
            self.second_target = None
            self.role_distinct_evidence = None
            self.second_acquisition_reason = 'second_role_current_distinct_support_missing'
            return
        self._second_proof_at = now
        self.second_target = self.second_tracker.update(scan, now, acquire=True)
        self.role_distinct_evidence = proof
        self.second_acquisition_reason = self.second_tracker.reason

    def _corridor_follow_update(self, scan, now, control):
        if (self.second_target_handover_observed
                and self._full_target_points(self.second_target, scan, self.second_track_id) is not None):
            return self._begin_second_orbit(scan, now, control)
        _, rejection = self._right_exit_target(scan, now)
        if rejection is not None:
            # Trial45: a single missing/ambiguous corridor frame only keeps the
            # adopted command within the last observation lease; no new
            # steering authority, and expiry still coasts.
            lease_receive, lease_publication = self._right_exit_last_receive, self._right_exit_last_publication
            if (rejection in ('right_exit_corridor_missing', 'right_exit_corridor_ambiguous')
                    and _number(lease_receive) and _number(lease_publication)
                    and now-lease_receive < SCAN_AGE_S
                    and (scan['at_ms']-lease_publication)/1000 < SCAN_AGE_S):
                self.steering_target = self.servo
                self.reason = 'route_corridor_' + rejection.rsplit('_', 1)[1] + '_hold'
                return self._result(now)
            self._begin_recovery_hold(rejection, now, control)
            self._slew(now)
            return self._result(now)
        if self._corridor_body_gap(self.right_exit_geometry) < 0.:
            # Model overlap only; a small model margin keeps pursuing the
            # center line (away from the near board) with proximity avoidance.
            self._begin_recovery_hold('route_corridor_body_margin_unavailable', now, control)
            return self._result(now)
        if not self._route_adopted(control, now):
            return self._result(now)
        self._second_role_observation(scan, now)
        if self.second_target is not None and self.second_target.get('confirmed') is True:
            return self._begin_second_orbit(scan, now, control)
        c = self.right_exit_geometry
        self._lane_left_limit(scan, c['heading_left_rad'], c.get('heading_rate_rad_s'))
        error = (c['heading_left_rad']
                 + math.atan2(c['center_offset_left_m']+c.get('approach_offset_left_m', 0.), c['lookahead_m'])
                 + ROUTE_ALIGN_DAMPING_S*c.get('heading_rate_rad_s', 0.))
        target = max(NEUTRAL-55, min(NEUTRAL+55, NEUTRAL+round(LEFT_TRIAL_GAIN*error)))
        natural = target
        if target < NEUTRAL and self._route_center_ack is None:
            target = NEUTRAL
        self.steering_target, self.natural_steering_target = target, natural
        self.reason = 'route_corridor_follow_current_geometry'
        self._slew(now)
        if self.servo > NEUTRAL:
            self._route_center_ack = None
        return self._result(now)

    def _clear_second_return(self):
        self.second_return_preparing = False
        self.second_return_evidence = None

    @staticmethod
    def _second_return_support_clear(points, servo):
        """Keep the complete support to the RIGHT of the proposed forward arc.

        Unlike the short all-obstacle path check, this checks every target
        point, including a target beyond that horizon. An arc passing on its
        wrong side is not a clockwise bypass, even if it misses the target.
        This uses the existing uncalibrated path model, not a physical proof.
        """
        curvature = (servo-NEUTRAL)/ROUTE_PWM_PER_CURVATURE
        if curvature > 0:
            return False
        half = SIDE_BODY_EXTENT_M+MANEUVER_BODY_CLEARANCE_M
        for x, y in points:
            if abs(curvature) < 1e-9:
                along, right = x, -y
            else:
                radius = -1/curvature
                along = radius*math.atan2(x, radius+y)
                right = radius-math.hypot(x, radius+y)
            if along+REAR_BODY_EXTENT_M >= 0 and right < half:
                return False
        return True

    def _lane_left_limit(self, scan, heading, rate):
        """Left PWM allowed by the lane-heading limit, predicted one lag ahead.

        Round 59 (Claude). Also bounds a task-left path correction in this
        frame (trial58 442/445: the path check raised 1612/1625 to 1632/1646
        while the car was already yawing 72 deg/s toward the left board).
        """
        predicted = heading+(rate*SECOND_STEER_LAG_S if rate is not None and rate < 0 else 0.)
        limit = NEUTRAL+round(SECOND_HEADING_HOLD_GAIN*(predicted+SECOND_BYPASS_MAX_HEADING_RAD))
        self._task_left_upper = (scan.get('seq'), limit)
        return predicted, limit

    def _current_lane(self, scan, now):
        """This frame's corridor measurement, without keeping its identity."""
        saved = (self.right_exit_geometry, self._right_exit_last_receive,
                 self._right_exit_last_publication)
        self.right_exit_geometry = None
        self._right_exit_last_receive = self._right_exit_last_publication = None
        try:
            target, rejection = self._right_exit_target(scan, now)
            corridor = copy.deepcopy(self.right_exit_geometry)
        finally:
            (self.right_exit_geometry, self._right_exit_last_receive,
             self._right_exit_last_publication) = saved
        return (None, None) if rejection is not None else (target, corridor)

    def _second_lane_bypass(self, scan, now, points):
        """Lane-frame bypass target left of the second support, or None."""
        _, lane = self._current_lane(scan, now)
        self._second_lane_heading = None if lane is None else lane['heading_left_rad']
        previous = getattr(self, '_second_lane_previous', None)
        self._second_lane_rate = None
        if lane is not None:
            row = (scan['at_ms']/1000, scan['received_at'], lane['heading_left_rad'])
            if previous is not None and .05 <= row[0]-previous[0] <= SCAN_AGE_S and .05 <= row[1]-previous[1] <= SCAN_AGE_S:
                rate = _wrap(row[2]-previous[2])/(row[0]-previous[0])
                if abs(rate) <= SECOND_MAX_HEADING_RATE_RAD_S:
                    self._second_lane_rate = rate
            self._second_lane_previous = row
        else:
            self._second_lane_previous = None
        if lane is None or abs(lane['heading_left_rad']) > SECOND_LANE_MAX_HEADING_RAD:
            return None
        heading, offset, width = lane['heading_left_rad'], lane['center_offset_left_m'], lane['width_m']
        c, s = math.cos(heading), math.sin(heading)
        along = [c*x+s*y for x, y in points]
        normal = [-s*x+c*y for x, y in points]
        wanted = max(normal)+SECOND_BYPASS_SUPPORT_GAP_M
        room = offset+width/2-(SIDE_BODY_EXTENT_M+MANEUVER_BODY_CLEARANCE_M)
        desired = min(wanted, room)
        ahead = min(along)
        lookahead = (max(FRONT_BODY_EXTENT_M+REAR_BODY_EXTENT_M, min(SECOND_LANE_LOOKAHEAD_M, ahead))
                     if ahead > 0 else SECOND_LANE_LOOKAHEAD_M)
        error = heading+math.atan2(desired, lookahead)
        target = max(RIGHT_EXIT_MIN_PWM, min(SERVO_MAX, NEUTRAL+round(LEFT_TRIAL_GAIN*error)))
        return {'source_seq': scan['seq'], 'lane_heading_left_rad': heading,
                'lane_center_offset_left_m': offset, 'lane_width_m': width,
                'support_left_normal_m': max(normal), 'support_nearest_along_m': ahead,
                'wanted_offset_m': wanted, 'lane_room_left_m': room, 'desired_offset_m': desired,
                'lookahead_m': lookahead, 'heading_error_rad': error, 'nominal_target_pwm': target,
                'scope': 'current_lane_frame_bypass', 'swept_path_certified': False}

    def _second_return_target(self, scan, now, points, bypass_target):
        """Prepare from present support/corridor geometry, before final pass.

        A confirmed support already clear of the right flank can stop asking
        for a left waypoint ahead of the car. Measure the current corridor
        without renewing or inheriting the older first-exit corridor lease.
        Missing geometry may release left, but cannot create a new right step.
        """
        if min(-p[1] for p in points)-SIDE_BODY_EXTENT_M < MANEUVER_BODY_CLEARANCE_M:
            self._clear_second_return()
            return None
        saved = (self.right_exit_geometry, self._right_exit_last_receive,
                 self._right_exit_last_publication)
        self.right_exit_geometry = None
        self._right_exit_last_receive = self._right_exit_last_publication = None
        try:
            native, _ = self._right_exit_target(scan, now)
            corridor = copy.deepcopy(self.right_exit_geometry)
        finally:
            (self.right_exit_geometry, self._right_exit_last_receive,
             self._right_exit_last_publication) = saved
        if native is not None and native > NEUTRAL:
            # A different live layout still calls for left; no memorized
            # turn direction or permanent release latch overrides it.
            self._clear_second_return()
            return None
        candidates = []
        if native is not None:
            # Preserve a valid clockwise request already made by the support
            # pursuit law. A straight corridor must not suppress that orbit.
            requested = min(native, bypass_target)
            all_points = self._path_points(scan)
            candidates = [pwm for pwm in {*ROUTE_PATH_RIGHT_CANDIDATES, requested, NEUTRAL}
                          if requested <= pwm <= NEUTRAL
                          and self._second_return_support_clear(points, pwm)
                          and all_points
                          and self._path_free_distance(all_points, pwm) >= ROUTE_PATH_LOOKAHEAD_M]
        if not candidates:
            # Do not reuse old geometry to increase right steering. The
            # existing near-body and stale-frame gates remain authoritative.
            if self.second_return_preparing:
                return min(self.servo, NEUTRAL)
            return None
        choice = min(candidates, key=lambda pwm: abs(pwm-requested))
        self.second_return_preparing = True
        self.second_return_evidence = {
            'source_seq': scan['seq'], 'source_at_ms': scan['at_ms'],
            'received_at': scan['received_at'], 'track_id': self.second_track_id,
            'support_bins': list(self.second_target['support_bins']),
            'observed_frontmost_x_m': max(p[0] for p in points),
            'observed_right_body_gap_m': min(-p[1] for p in points)-SIDE_BODY_EXTENT_M,
            'corridor': corridor, 'corridor_target_pwm': native,
            'support_bypass_target_pwm': bypass_target, 'requested_target_pwm': requested,
            'selected_target_pwm': choice, 'clear_candidates': sorted(candidates),
            'all_obstacle_lookahead_m': ROUTE_PATH_LOOKAHEAD_M,
            'basis': 'current_complete_support_right_of_current_corridor_return_arc',
            'pass_proven': False, 'swept_path_certified': False,
            'physical_curvature_calibrated': False}
        return choice

    def _begin_second_orbit(self, scan, now, control):
        self._clear_second_return()
        self.second_track_id = self.second_target['track_id']
        self.second_tracker.enable_close_maintenance(self.second_track_id)
        self._second_role_anchor = self._second_prior(self.second_target, scan)
        self._second_recovering = False
        self.second_target_handover_observed = True
        self.second_orbit_since, self.route_stage = now, 'second_orbit'
        # An uninterrupted right command may retain its actual center ACK.
        # A new left-to-right change must acquire a new center ACK below.
        if self.servo <= NEUTRAL and self._route_center_ack is not None:
            self.second_center_ack = dict(self._route_center_ack)
        self.steering_target = self.servo
        self.reason = 'second_target_handover_keep_adopted_command'
        self._route_event('second_handover', scan, control, self.role_distinct_evidence)
        return self._result(now)

    def _second_orbit_update(self, scan, now, control):
        target = self.second_target
        held = getattr(self.second_tracker, 'held', None)
        if (target is None and isinstance(held, dict) and held.get('track_id') == self.second_track_id
                and now-self.second_orbit_since < SECOND_ORBIT_MAX_S):
            # Tolerant tracker kept the identity through a missing frame:
            # keep the selected command, or continue an adopted left release.
            if self.second_return_preparing and not self._route_adopted(control, now):
                return self._result(now)
            self.steering_target = (min(self.servo, NEUTRAL)
                                    if self.second_return_preparing else self.servo)
            self.natural_steering_target = self.steering_target
            self.reason = 'second_target_missing_frame_hold'
            self._slew(now)
            return self._result(now)
        lost = (target is None or target.get('track_id') != self.second_track_id
                or not target.get('confirmed'))
        if now-self.second_orbit_since >= SECOND_ORBIT_MAX_S or lost:
            self._clear_second_return()
            # Self-recovery: go back to corridor following, which re-acquires
            # the second support (or continues) from the current position.
            self.route_stage = 'corridor_follow'
            # The second orbit does not refresh the corridor estimate; restart
            # its identity from the next fresh observation (no stale-identity
            # rejection and recovery pause on return).
            self.right_exit_geometry = None
            self._right_exit_last_receive = self._right_exit_last_publication = None
            if lost:
                self._start_second_identity_recovery()
            else:
                self.second_tracker = self._new_second_tracker()
                self._second_recovering = False
            self.second_target = None
            self.second_center_ack = None
            self.second_target_handover_observed = False
            self.reason = ('second_target_entry_timeout_back_to_corridor' if not lost
                           else 'second_target_lost_back_to_corridor')
            self.steering_target = self.servo
            self._route_event('second_target_recovery', scan, control,
                              {'source_seq': scan['seq'], 'reason': self.reason})
            return self._result(now)
        if not self._route_adopted(control, now):
            return self._result(now)
        if self.second_center_ack is None and self.servo == NEUTRAL:
            self.second_center_ack = {k: control.get(k) for k in ('servo', 'motor', 'tick', 'seq')}
        points = self._full_target_points(target, scan, self.second_track_id)
        passed = (points is not None and max(p[0] for p in points) <= -REAR_BODY_EXTENT_M
                  and min(-p[1] for p in points)-SIDE_BODY_EXTENT_M >= MANEUVER_BODY_CLEARANCE_M)
        if passed and self._mature_observation(self._second_pass_history, scan, .25):
            self.second_pass_evidence = {'source_seq': scan['seq'], 'source_at_ms': scan['at_ms'],
                'received_at': scan['received_at'], 'track_id': self.second_track_id,
                'support_bins': list(target['support_bins']),
                'observed_frontmost_x_m': max(p[0] for p in points),
                'observed_right_body_gap_m': min(-p[1] for p in points)-SIDE_BODY_EXTENT_M,
                'basis': 'current_complete_support_behind_rear',
                'physical_cone_pass_certified': False, 'swept_path_certified': False}
            self._route_event('second_pass', scan, control, self.second_pass_evidence)
            # Trial46 operator: after the second target turn on toward the
            # traffic light and end facing it. Direction comes from free space.
            self.route_stage, self.light_since = 'light_turn', now
            self.light_turn_started = now
            self.second_return_preparing = False
            self._light_history.clear()
            self._light_heading = self._exit_heading  # continue the next-lane window
            self.steering_target = self.servo
            self.reason = 'light_turn_started_keep_adopted_command'
            self._route_event('light_turn', scan, control, {'source_seq': scan['seq']})
            return self._result(now)
        if not passed:
            self._second_pass_history.clear()
        if points is None:
            # Partial boundaries maintain only the already selected command;
            # they cannot define a new bypass point or prove a pass.
            self.steering_target = (min(self.servo, NEUTRAL)
                                    if self.second_return_preparing else self.servo)
            self.natural_steering_target = self.steering_target
            self.reason = 'second_target_partial_support_hold'
            if self.second_return_preparing:
                self._slew(now)
            return self._result(now)
        body_length = FRONT_BODY_EXTENT_M+REAR_BODY_EXTENT_M
        front = max(p[0] for p in points)
        near = min(p[0] for p in points)
        left_edge = max(p[1] for p in points)
        # Reach the bypass line at the support's near face, not beyond its far
        # face: a pursuit arc to a point past the support cuts inside it. The
        # receding point then keeps that gap while the support is abeam.
        waypoint = [max(body_length, near), left_edge+SECOND_BYPASS_SUPPORT_GAP_M]
        bearing = math.atan2(waypoint[1], waypoint[0])
        curvature = 2*waypoint[1]/(waypoint[0]**2+waypoint[1]**2)
        # Trial43: the bypass point left of a left-front support needs left;
        # right still requires a current actual center ACK after any left.
        nominal = max(RIGHT_EXIT_MIN_PWM, min(SERVO_MAX,
            NEUTRAL+round(SECOND_CURVATURE_GAIN_PWM_M*curvature)))
        self.second_feedback = {'source_seq': scan['seq'], 'source_at_ms': scan['at_ms'],
            'current_support_left_edge_y_m': left_edge, 'current_support_frontmost_x_m': front,
            'current_support_nearest_x_m': near, 'bypass_support_gap_m': SECOND_BYPASS_SUPPORT_GAP_M,
            'relative_bypass_point_left_m': waypoint, 'bearing_left_rad': bearing,
            'pursuit_curvature_per_m': curvature,
            'nominal_target_pwm': nominal,
            'scope': 'current_support_relative_point_trial',
            'swept_path_certified': False, 'physical_curvature_calibrated': False}
        lane = self._second_lane_bypass(scan, now, points)
        self.second_feedback['lane_bypass'] = lane
        if lane is not None:
            # Early offset from the lane frame; the support pursuit still
            # governs a near support whose offset is not yet taken (trial48:
            # .9 m ahead needs >= 1680). Both relax once the car is on the line.
            nominal = max(nominal, lane['nominal_target_pwm'])
            self.second_feedback['nominal_target_pwm'] = nominal
            self.second_feedback['scope'] = 'max_of_support_pursuit_and_lane_frame_bypass'
        lane_heading = getattr(self, '_second_lane_heading', None)
        if lane_heading is not None and nominal > NEUTRAL:
            predicted, limit = self._lane_left_limit(scan, lane_heading, getattr(self, '_second_lane_rate', None))
            self.second_feedback['predicted_lane_heading_rad'] = predicted
            if nominal > limit:
                nominal = max(RIGHT_EXIT_MIN_PWM, limit)
                self.second_feedback['heading_limited_target_pwm'] = nominal
                self.second_feedback['nominal_target_pwm'] = nominal
        # Round 58 (Claude): once the support is beside/behind the body front
        # with the right clearance, the rest of the clockwise orbit is the turn
        # into the next lane. Trial57 576-581: the return target went back to
        # the old lane axis (1413 -> 1486) while that lane ended in a board
        # ahead; the light turn then started late. Take the free-space target
        # when it is further right (task rule: second support clockwise);
        # support clearance, path check and the center ACK still apply.
        right_gap = min(-p[1] for p in points)-SIDE_BODY_EXTENT_M
        if front <= FRONT_BODY_EXTENT_M and right_gap >= MANEUVER_BODY_CLEARANCE_M:
            space = self._free_space(scan, self._exit_heading)
            if space is not None:
                self._exit_heading = space['best_heading_rad']
                exit_target = self._free_space_target(space)
                self.second_feedback['next_lane_free_space'] = {
                    'best_heading_rad': space['best_heading_rad'], 'free_ahead_m': space['free_ahead_m'],
                    'target_pwm': exit_target, 'applied': exit_target < nominal}
                if exit_target < nominal:
                    nominal = exit_target
                    self.second_feedback['nominal_target_pwm'] = nominal
        was_preparing = self.second_return_preparing
        returning = self._second_return_target(scan, now, points, nominal)
        if returning is not None:
            nominal = returning
            if not was_preparing:
                self._route_event('second_return_prepare', scan, control, self.second_return_evidence)
        self.natural_steering_target = nominal
        if nominal < NEUTRAL and (self.second_center_ack is None or self.servo > NEUTRAL):
            self.steering_target = NEUTRAL
            self.reason = 'second_target_waiting_center_ack'
        else:
            self.steering_target = nominal
            self.reason = ('second_target_left_of_support_entry' if nominal > NEUTRAL
                           else 'second_target_clockwise_bounded_entry')
        if self.second_return_preparing:
            self.reason = ('second_target_return_waiting_center_ack'
                           if nominal < NEUTRAL and self.steering_target == NEUTRAL
                           else 'second_target_return_current_geometry'
                           if self.second_return_evidence['source_seq'] == scan['seq']
                           else 'second_target_return_geometry_hold')
        self._slew(now)
        return self._result(now)

    @staticmethod
    def _free_space(scan, previous_heading=None):
        """Deepest free direction for the measured body plus clearance.

        Polar footprint: each known return blocks the headings whose body
        corridor (half width = side extent + .08 m) it overlaps, at its
        projected distance. A front null is never free: it blocks at its
        nearer known neighbour. Returns None without any usable return.
        """
        ranges = list(scan['ranges'])
        half = SIDE_BODY_EXTENT_M+MANEUVER_BODY_CLEARANCE_M
        step = LIGHT_HEADINGS_DEG[1]-LIGHT_HEADINGS_DEG[0]
        free = [LIGHT_FREE_HORIZON_M]*len(LIGHT_HEADINGS_DEG)
        left_clear = right_clear = None
        usable = any(r is not None for r in ranges)
        # Fill front nulls conservatively before the footprint pass.
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
            bearing = _FREE_BEARING_DEG[index]  # left positive
            x, y = r*_FREE_POINT_COS[index], r*_FREE_POINT_SIN[index]
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
            cos_row, sin_row = _FREE_COS[index], _FREE_SIN[index]
            for k in range(first, last+1):
                along = r*cos_row[k]
                if along > 0 and abs(r*sin_row[k]) <= half and along < free[k]:
                    free[k] = along
        if not usable:
            return None
        deepest = max(free)
        # Follow the gap: aim at the middle of a contiguous near-deepest window
        # (in a lane that is its axis). Keep the window holding the previous
        # choice so two similar openings do not flip the command; otherwise
        # take the widest window.
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

    def _end_light_stage(self, reason, now, control):
        self.begin_coast(reason, now)
        if (self.phase == 'coast' and control.get('servo') == self.servo
                and control.get('command_acked') is True):
            # Never recenter while rolling; the final command is ACKed.
            self.route_end_coast_servo = self.servo
            self.steering_target = self.servo
        self._slew(now)
        return self._result(now)

    @staticmethod
    def _free_space_target(space):
        """Light-turn free-space law: deepest free heading, centered when nearly lined up."""
        best, center = space['best_heading_rad'], space['center_error_m']
        center_term = (math.atan2(center, 1.5)
                       if center is not None and abs(best) <= math.radians(20) else 0.)
        return max(RIGHT_EXIT_MIN_PWM, min(SERVO_MAX, NEUTRAL+round(LEFT_TRIAL_GAIN*(best+center_term))))

    def _lane_front_range(self, scan, exclude=()):
        """Nearest return straight ahead (+-LANE_FRONT_HALF_BINS deg), or None;
        bins in ``exclude`` are skipped (round 106: a support's own returns).

        Round 64 (lane entry): measured along the compass axis instead, as
        the distance along that axis. An angled car's own straight-ahead ray
        meets a side board long before the end board (round-63 simulation:
        the finish cut fired early on an angled lane entry).
        """
        heading = self.compass.current(scan) if self.entry_kind == 'lane' else None
        if heading is not None:
            axis = math.pi if self.route_stage == 'open_transfer' else 0.
            center = round(math.degrees(_wrap(heading-axis)))
            values = [scan['ranges'][i % 360]*math.cos(math.radians(i-center))
                      for i in range(center-LANE_FRONT_HALF_BINS, center+LANE_FRONT_HALF_BINS+1)
                      if _number(scan['ranges'][i % 360]) and i % 360 not in exclude]
            return min(values) if values else None
        values = [scan['ranges'][i % 360] for i in range(-LANE_FRONT_HALF_BINS, LANE_FRONT_HALF_BINS+1)
                  if i % 360 not in exclude]
        values = [r for r in values if _number(r)]
        return min(values) if values else None

    def _lane_front_track(self, scan, ahead):
        """Start-lane distance to go for the speed estimate only.

        The nearest return of the +-LANE_FRONT_HALF_BINS sector can switch
        between surfaces from one frame to the next (2026-10-18 simulation: a
        .15 m depth step 6-7 m ahead, or a side board met at grazing incidence
        at the sector edge), so the minimum jumps and the closing speed went
        unknown or read 2+ m/s. Each direction of the sector (relative to the
        compass axis) is compared with the same direction on the previous frame
        and the median change moves the track. Stopping still uses `ahead`.
        """
        heading = self.compass.current(scan)
        previous, self._front_track = self._front_track, None
        if heading is None or ahead is None:
            return ahead
        center = round(math.degrees(_wrap(heading)))
        sector = {}
        for j in range(-LANE_FRONT_HALF_BINS, LANE_FRONT_HALF_BINS+1):
            r = scan['ranges'][(center+j) % 360]
            if _number(r):
                sector[j] = r*math.cos(math.radians(center+j)-heading)
        value = ahead
        if previous is not None and 0 < scan['at_ms']-previous[0] <= (SCAN_AGE_S+.05)*1000:
            deltas = sorted(previous[1][j]-sector[j] for j in sector.keys() & previous[1].keys())
            if len(deltas) >= 3:
                value = previous[2]-(deltas[(len(deltas)-1)//2]+deltas[len(deltas)//2])/2
        self._front_track = (scan['at_ms'], sector, value)
        return value

    def _track_speed(self, scan, track):
        """Start-lane closing speed: median of the pairwise slopes of the last
        LANE_TRACK_SPEED_POINTS track values (.4 s). The median of .2 s
        baselines lagged ~.3 s; a least-squares slope read 2.11 m/s at
        1.2 m/s in round 71 from one stale frame (all sector bins .35 m too
        far for one scan) and cut the start lane; one bad frame of five
        cannot move the pairwise median."""
        history = self._track_history
        if track is None:
            history.clear()
            return None
        row = (scan['at_ms']/1000, scan['received_at'], track)
        if history:
            dt = row[0]-history[-1][0]
            rate = (history[-1][2]-track)/dt if dt > 0 else None
            if (not .05 <= dt <= SCAN_AGE_S+.05 or not .05 <= row[1]-history[-1][1] <= SCAN_AGE_S+.05
                    or rate is None or not -.4 <= rate <= LANE_MAX_CLOSING_MPS):
                history.clear()
        history.append(row)
        del history[:-LANE_TRACK_SPEED_POINTS]
        if len(history) < LANE_TRACK_SPEED_POINTS:
            return None
        slopes = sorted((b[2]-a[2])/(b[0]-a[0]) for i, a in enumerate(history) for b in history[i+1:])
        slope = (slopes[(len(slopes)-1)//2]+slopes[len(slopes)//2])/2
        return max(0., -slope)

    def _lane_speed(self, scan, ahead, *, min_rates=2):
        """Closing speed on the board ahead: median rate over ~.3 s baselines."""
        history = self._lane_ranges
        if ahead is None:
            history.clear()
            return None
        row = (scan['at_ms']/1000, scan['received_at'], ahead)
        if history:
            dt = row[0]-history[-1][0]
            rate = (history[-1][2]-ahead)/dt if dt > 0 else None
            if (not .05 <= dt <= SCAN_AGE_S+.05 or not .05 <= row[1]-history[-1][1] <= SCAN_AGE_S+.05
                    or rate is None or not -.4 <= rate <= LANE_MAX_CLOSING_MPS):
                history.clear()
        history.append(row)
        del history[:-LANE_SPEED_HISTORY]
        if len(history) < 3 or len(history)-min(LANE_SPEED_BASELINE, len(history)-2) < min_rates:
            return None
        step = min(LANE_SPEED_BASELINE, len(history)-2)
        rates = sorted((a[2]-b[2])/(b[0]-a[0]) for a, b in zip(history, history[step:]))
        return max(0., rates[len(rates)//2])

    def _begin_light_lane(self, scan, now, control, evidence):
        self.light_alignment_evidence = evidence
        self._route_event('light_aligned', scan, control, evidence)
        self.route_stage = 'light_lane'
        self.lane_motor = ROUTE_SLOW_MOTOR  # speed history from the aligned frames is kept
        self._lane_steering = CorridorSteering()
        self._lane_pulse = 0
        self._route_event('light_lane', scan, control, {'source_seq': scan['seq']})
        self.steering_target = self.servo
        self.reason = 'light_lane_started_keep_adopted_command'
        return self._result(now)

    @staticmethod
    def _start_allowed_speed(ahead):
        """Highest speed that still stops LANE_START_SEARCH_M short of the board ahead."""
        if ahead is None:
            return None
        room = ahead-FRONT_BODY_EXTENT_M-LANE_FINISH_GAP_M-LANE_START_SEARCH_M
        if room <= 0:
            return 0.
        a, r = LANE_START_STOP_DECEL_MPS2, LANE_REACTION_S
        return min(LANE_START_MAX_MPS, -r*a+math.sqrt((r*a)**2+2*a*room))

    @staticmethod
    def _start_governor(speed, rise, allowed, measured_before=False, settling=False):
        """Start-lane motor for the measured speed against the allowed speed."""
        cap = LANE_START_MAX_MPS if allowed is None else allowed
        launch_rise = START_LAUNCH_ACCEL_MPS2*ESC_LAG_S
        if speed is None:
            if cap < LANE_SPEED_UNKNOWN_MPS*.6:
                return NEUTRAL
            # Launch at 1560; once a speed was measured, an unknown one cruises
            # at 1550 (2026-10-18 simulation: the speed stayed unknown behind a
            # jumping range ahead and 1560 drove the car past the 1.5 m/s cap).
            return ROUTE_SLOW_MOTOR if measured_before else TRIAL_MOTOR
        if speed+rise >= cap:
            return NEUTRAL
        if settling:
            return ROUTE_SLOW_MOTOR  # round 100: just after a 1600 burst
        # Round 100: 1600 once a speed is measured, while slow and while its
        # full ESC-lag rise stays under the allowed speed.
        if speed < START_LAUNCH_MAX_MPS and speed+launch_rise < cap:
            return START_LAUNCH_MOTOR
        # Round 104 (car run 103): a new 1560 push also keeps its own full
        # ESC-lag rise under the allowed speed, as 1600 does. One 1560 scan at
        # 1.23 m/s (allowed 1.66, no rise yet) put speed + rise at 1.31 + .43
        # against 1.62 the next scan and cut four scans (.4 s at neutral). A
        # push under way (rise near its full value) keeps the band rule.
        push_rise = max(rise, LANE_POWER_ACCEL_MPS2*ESC_LAG_S)
        if speed < LANE_SPEED_RESTART_MPS or (speed+rise < cap-LANE_START_PUSH_BAND_MPS
                                               and speed+push_rise < cap):
            return TRIAL_MOTOR
        return ROUTE_SLOW_MOTOR

    def _reaction_s(self, now):
        """Reaction plus what is left of the ESC lag after the last push."""
        if self._power_cmd_at is None:
            return LANE_REACTION_BASE_S
        return LANE_REACTION_BASE_S+max(0., min(ESC_LAG_S, ESC_LAG_S-(now-self._power_cmd_at)))

    def _lane_decels(self):
        """(aim, guard, strong) coast decels: the lane's measured one, less and more
        by two standard errors (at least by LANE_COAST_GUARD_RATIO); else
        LANE_STOP_DECEL_MPS2, the weakest LANE_STOP_DECEL_MIN_MPS2 and None."""
        decel, se = self._lane_coast_decel, self._lane_coast_decel_se
        if decel is None:
            return LANE_STOP_DECEL_MPS2, LANE_STOP_DECEL_MIN_MPS2, None
        return (decel, max(LANE_COAST_DECEL_FLOOR_MPS2, min(LANE_COAST_GUARD_RATIO*decel, decel-2*se)),
                max(decel/LANE_COAST_GUARD_RATIO, decel+2*se))

    def _lane_pipeline(self, now, onset, offset, push_s=0.):
        """(coast_s, power_s) the wheels still get if the motor goes neutral now, or
        after ``push_s`` more of power: power from ``onset`` after the powered run
        began (now, for a push after it has left the pipeline) until ``offset``
        after its last output."""
        since, last = self._power_since, self._power_cmd_at
        if last is not None and since is None:
            since = last
        if push_s > 0:
            if last is None or last+offset <= now:
                since = now
            last = now+push_s
        if last is None:
            return 0., 0.
        start, end = since+onset, last+offset
        if end <= now or end <= start:
            return 0., 0.
        return max(0., start-now), end-max(now, start)

    def _lane_speed_now(self, speed, now, onset, offset, decel, accel):
        """The closing speed now from the median rate, which is the speed half its
        window ago: over that half window the wheels coasted at ``decel`` except
        while power reached them (``onset`` after a powered run began until
        ``offset`` after its last output), at ``accel``."""
        history = self._lane_ranges
        lag = (history[-1][0]-history[0][0])/2 if history else 0.
        powered_s = 0.
        if self._power_cmd_at is not None:
            since = self._power_cmd_at if self._power_since is None else self._power_since
            powered_s = max(0., min(now, self._power_cmd_at+offset)-max(now-lag, since+onset))
            if self._power_prev_end is not None:
                # the run before this one (a push that has not reached the wheels yet)
                powered_s += max(0., min(now, self._power_prev_end+offset, since+onset)-(now-lag))
        powered_s = min(lag, powered_s)
        return max(0., speed+accel*powered_s-decel*(lag-powered_s))

    def _lane_worst_gap(self, gap, speed, now, decel, accel, reaction, push_s=0.):
        """Guard: the smaller front gap at the stop over a LANE_ESC_LAG_MIN_S and an
        ESC_LAG_S lag (each skewed by LANE_ESC_SKEW_S), with the speed brought to
        now plus LANE_SPEED_GUARD_MARGIN_MPS."""
        worst = None
        for lag in (LANE_ESC_LAG_MIN_S, ESC_LAG_S):
            onset, offset = lag-LANE_ESC_SKEW_S, lag+LANE_ESC_SKEW_S
            v = self._lane_speed_now(speed, now, onset, offset, decel, accel)+LANE_SPEED_GUARD_MARGIN_MPS
            coast_s, power_s = self._lane_pipeline(now, onset, offset, push_s)
            value = self._lane_stop(gap, v, decel, reaction, coast_s, power_s, accel)
            worst = value if worst is None else min(worst, value)
        return worst

    @staticmethod
    def _lane_stop(gap, speed, decel, reaction, coast_s, power_s, accel):
        """Front gap where the car stops: ``speed`` over ``reaction``, ``coast_s``
        coasting, ``power_s`` of power at ``accel`` (to LANE_PUSH_MAX_MPS, travelled
        at the speed reached), then a coast at ``decel``."""
        travel = speed*reaction
        if speed <= decel*coast_s:
            return gap-travel-speed*speed/(2*decel)
        coasted = speed-decel*coast_s
        peak = min(coasted+accel*power_s, max(coasted, LANE_PUSH_MAX_MPS))
        return gap-travel-(speed+coasted)/2*coast_s-peak*power_s-peak*peak/(2*decel)

    def _lane_coast_observe(self, now, speed):
        """Coast decel: least-squares slope of the speed while its whole window coasts.

        Round 101: the newest three rates of the median (its last five scans)
        must start ESC_LAG_S after the last powered output: the median of five
        rates whose newest three fall is the middle one, as in a whole coast;
        fitted over up to LANE_COAST_TRACK_POINTS scans (was the end points of
        LANE_SPEED_HISTORY) once its slope's standard error is at most
        LANE_COAST_DECEL_SE_MAX_MPS2; a speed under LANE_SPEED_RESTART_MPS (at or
        near rest reads 0) does not enter the fit."""
        coasting = (self.lane_motor == NEUTRAL and speed is not None and len(self._lane_ranges) >= 3
                    and (self._power_cmd_at is None
                         or self._lane_ranges[max(0, len(self._lane_ranges)-5)][1]-self._power_cmd_at >= ESC_LAG_S))
        if not coasting:
            self._lane_coast_track = []
            return
        if speed < LANE_SPEED_RESTART_MPS:
            return
        track = self._lane_coast_track
        if track and (now <= track[-1][0] or speed > track[-1][1]+LANE_SPEED_CUT_MARGIN_MPS):
            track.clear()  # a jump up is not a coast
        track.append((now, speed))
        del track[:-LANE_COAST_TRACK_POINTS]
        t0, v0 = track[0]
        t1, v1 = track[-1]
        if t1-t0 >= LANE_COAST_MIN_SPAN_S and v0-v1 >= LANE_COAST_MIN_DROP_MPS and len(track) >= 4:
            mean_t = sum(t for t, _ in track)/len(track)
            mean_v = sum(v for _, v in track)/len(track)
            sxx = sum((t-mean_t)*(t-mean_t) for t, _ in track)
            slope = sum((t-mean_t)*(v-mean_v) for t, v in track)/sxx
            residual = sum((v-mean_v-slope*(t-mean_t))*(v-mean_v-slope*(t-mean_t)) for t, v in track)
            se = math.sqrt(residual/(len(track)-2)/sxx)
            if se <= LANE_COAST_DECEL_SE_MAX_MPS2:
                self._lane_coast_decel = max(LANE_COAST_DECEL_FLOOR_MPS2, min(LANE_COAST_DECEL_CAP_MPS2, -slope))
                self._lane_coast_decel_se = se

    def _light_lane_update(self, scan, now, control):
        """Follow the light-lane straight, then stop inside the finish zone.

        Round 58 (Claude); round 63 (Claude): steering is the start-straight
        wall law (autonomy_live.CorridorSteering: two fitted side boards,
        hysteresis, trend damping, +-55 PWM, slow steps, release before
        reversing), else the measured-lane corridor law, else free space.
        Right still needs a current actual center ACK. No traffic-light vision.
        """
        if not self._route_adopted(control, now):
            return self._result(now)
        _, lane = self._current_lane(scan, now)
        space = None
        walls = corridor_walls(scan)
        heading = self.compass.current(scan) if self.entry_kind == 'lane' else None
        open_pursuit = None if heading is None else self._open_light_pursuit(scan, heading, self._response_pose())
        if open_pursuit is not None:
            natural = open_pursuit[3]
            source = 'open_lane_axis_pursuit'
        elif walls is not None:
            # Continue from the actually commanded servo (inside the law's own
            # +-55 range; the post-pass slew moves a larger turn there).
            self._lane_steering.servo = max(NEUTRAL-55, min(NEUTRAL+55, self.servo))
            natural = self._lane_steering.update(walls, scan['seq'], now, True, scan['at_ms'])
            source = 'straight_lane_walls'
        elif lane is not None and abs(lane['heading_left_rad']) <= SECOND_LANE_MAX_HEADING_RAD:
            lookahead = max(FRONT_BODY_EXTENT_M+REAR_BODY_EXTENT_M, min(ROUTE_ALIGN_LOOKAHEAD_MAX_M, lane['width_m']))
            error = lane['heading_left_rad']+math.atan2(lane['center_offset_left_m'], lookahead)
            natural = max(NEUTRAL-55, min(NEUTRAL+55, NEUTRAL+round(LEFT_TRIAL_GAIN*error)))
            source = 'measured_lane'
        else:
            space = self._free_space(scan, self._light_heading)
            if space is None:
                self._begin_recovery_hold('light_lane_geometry_unavailable', now, control)
                return self._result(now)
            self._light_heading = space['best_heading_rad']
            natural = self._free_space_target(space)
            source = 'free_space'
        target = natural
        if target < NEUTRAL and (self._route_center_ack is None or self.servo > NEUTRAL):
            target = NEUTRAL  # right only after a current actual center ACK
        ahead = self._lane_front_range(scan)
        speed = self._lane_speed(scan, ahead)
        self._lane_coast_observe(now, speed)
        held = getattr(self, '_lane_speed_last', None)
        speed_known = True
        if speed is not None:
            self._lane_speed_last = (now, speed)
            v = speed
        elif held is not None and 0 <= now-held[0] <= LANE_SPEED_HOLD_S:
            v = held[1]+LANE_SPEED_HOLD_MARGIN_MPS
        else:
            v, speed_known = LANE_SPEED_UNKNOWN_MPS, False
        gap = None if ahead is None else ahead-FRONT_BODY_EXTENT_M
        decel, guard_decel, strong_decel = self._lane_decels()
        powered = self.lane_motor in DRIVE_MOTORS
        aim_coast, aim_power = self._lane_pipeline(now, LANE_ESC_LAG_NOMINAL_S, LANE_ESC_LAG_NOMINAL_S)
        # nothing left in the ESC pipeline even with the longest lag
        coasting = not powered and (self._power_cmd_at is None
                                    or now >= self._power_cmd_at+ESC_LAG_S+LANE_ESC_SKEW_S)
        kick = self._power_cmd_motor == TRIAL_MOTOR
        aim_accel = LANE_POWER_ACCEL_MPS2 if kick else LANE_PUSH_ACCEL_MPS2
        # Past the finish line a push only brings the rear wheels in (the time
        # has ended; the board costs +10 s): LANE_PUSH_LATE_ACCEL_MAX_MPS2.
        late = gap is not None and gap <= LANE_FINISH_ZONE_GAP_M
        guard_accel = (LANE_KICK_ACCEL_MAX_MPS2 if kick else LANE_PUSH_LATE_ACCEL_MAX_MPS2 if late
                       else LANE_PUSH_ACCEL_MAX_MPS2)
        v_aim = v
        if speed is not None:
            v_aim = self._lane_speed_now(v, now, LANE_ESC_LAG_NOMINAL_S, LANE_ESC_LAG_NOMINAL_S, decel, aim_accel)
        guard_reaction = LANE_REACTION_BASE_S if powered else LANE_AIM_REACTION_S
        stop_gap = worst_gap = None
        if gap is not None:
            stop_gap = self._lane_stop(gap, v_aim, decel, LANE_AIM_REACTION_S if powered else 0.,
                                       aim_coast, aim_power, aim_accel)
            worst_gap = self._lane_worst_gap(gap, v, now, guard_decel, guard_accel, guard_reaction)
        stopped = speed is not None and speed < LANE_SPEED_RESTART_MPS
        in_zone = gap is not None and gap <= LANE_FINISH_ZONE_GAP_M
        body_in_zone = gap is not None and gap <= LANE_FINISH_BODY_GAP_M
        kick_room = gap is not None and gap >= LANE_FINISH_MIN_GAP_M+LANE_KICK_TRAVEL_M
        rested = self._power_cmd_at is None or now-self._power_cmd_at >= LANE_KICK_REST_S
        # Round 105: the front in the zone, the rear out, no room for the kick:
        # creep, until LANE_CREEP_MAX creeps have been sent and the last one
        # has rested (its ESC lag and the speed window behind it).
        creep_room = (in_zone and not body_in_zone and not kick_room
                      and gap >= LANE_FINISH_MIN_GAP_M+LANE_CREEP_TRAVEL_M
                      and (self._lane_creeps < LANE_CREEP_MAX or not rested))
        # Round 101: end with the whole car in the zone: stopped there, or still
        # coasting where even a coast at the strong measured decel (_lane_decels)
        # stops in it (and the aimed one short of the board); stopped with only
        # the front in it where neither a kick nor a creep may go on, end there.
        basis = None
        if speed_known and stopped and body_in_zone:
            basis = 'stopped_with_the_whole_car_in_the_finish_zone'
        elif speed_known and stopped and in_zone and not kick_room and not creep_room:
            basis = ('stopped_front_in_finish_zone_creeps_used' if self._lane_creeps
                     else 'stopped_front_in_finish_zone_restart_could_reach_the_board')
        elif (speed_known and coasting and not stopped and strong_decel is not None and gap is not None
              and stop_gap >= LANE_FINISH_MIN_GAP_M
              and self._lane_stop(gap, v_aim, strong_decel, 0., 0., 0., 0.)
              <= LANE_FINISH_BODY_GAP_M):
            basis = 'coast_stops_the_whole_car_in_the_finish_zone'
        self.lane_feedback = {'source_seq': scan['seq'], 'steering_source': source,
            'nominal_target_pwm': natural, 'lane': lane, 'free_space': space, 'walls': walls,
            'front_range_m': ahead, 'front_gap_m': gap, 'speed_mps': speed, 'speed_for_cut_mps': v,
            'speed_aim_mps': v_aim, 'speed_known': speed_known, 'speed_basis': 'closing_rate_on_board_ahead_axis',
            'predicted_stop_gap_m': stop_gap, 'worst_stop_gap_m': worst_gap, 'in_finish_zone': in_zone,
            'whole_car_in_finish_zone': body_in_zone, 'pipeline_power_s': aim_power,
            'coast_decel_mps2': decel, 'guard_decel_mps2': guard_decel, 'strong_decel_mps2': strong_decel,
            'coast_decel_se_mps2': self._lane_coast_decel_se,
            'coast_decel_measured': self._lane_coast_decel is not None,
            'motor_before': self.lane_motor, 'swept_path_certified': False, 'traffic_light_vision': False,
            'creep_room': creep_room, 'creeps': self._lane_creeps}
        if basis is not None:
            self.finish_evidence = {**self.lane_feedback, 'received_at': scan['received_at'],
                                    'basis': basis, 'finish_zone_certified': False}
            self._route_event('finish_stop', scan, control, self.finish_evidence)
            return self._end_light_stage('route_finish_zone_stop', now, control)
        if gap is not None and (stop_gap <= LANE_FINISH_TARGET_GAP_M or worst_gap <= LANE_FINISH_MIN_GAP_M):
            self.lane_motor = NEUTRAL  # the coast from here reaches the target (or the guard)
        elif speed is None:
            self.lane_motor = ROUTE_SLOW_MOTOR
        elif stopped:
            self.lane_motor = (TRIAL_MOTOR if (kick_room or creep_room) and (rested or self._lane_pulse > 0)
                               else NEUTRAL)
        elif self.lane_motor == ROUTE_SLOW_MOTOR:
            pass  # the straight, or a push, until the coast is aimed
        elif (self.lane_motor == NEUTRAL and stop_gap > LANE_FINISH_TARGET_GAP_M+LANE_TOPUP_BAND_M
              and v_aim-decel*LANE_ESC_LAG_NOMINAL_S >= LANE_PUSH_ROLLING_MPS
              and self._lane_worst_gap(gap, v, now, guard_decel, guard_accel, guard_reaction,
                                       LANE_PUSH_MIN_S) > LANE_FINISH_MIN_GAP_M):
            self.lane_motor = ROUTE_SLOW_MOTOR  # it would stop short: push again while it rolls
        else:
            self.lane_motor = NEUTRAL  # coasting, or a kick once the car rolls
        # Round 94 (user: 最后不到半米的距离一直停车再动): near the end a 2-frame
        # pulse barely moved the car (run 93: 1550 pulse at .86 m, then 0 m/s
        # and .17 m in 1.1 s; run 91: 1560 pulses crept 1.04 -> .87 m in 2 s),
        # while a 3-frame 1560 kick from rest reached .47 m/s and ~.3-.4 m (run
        # 91). A restart from rest is that kick everywhere; a 1550 push of a
        # car still rolling is not chopped (its guard is checked every scan).
        # Round 101: the next kick no sooner than LANE_KICK_REST_S after it (was
        # LANE_REST_FRAMES scans after the kick, restarted by any other output).
        if self.lane_motor == TRIAL_MOTOR and self._lane_pulse >= (LANE_CREEP_FRAMES if self._lane_creep
                                                                   else LANE_KICK_FRAMES):
            self.lane_motor, self._lane_pulse, self._lane_creep = NEUTRAL, 0, False
        elif self.lane_motor == TRIAL_MOTOR:
            if self._lane_pulse == 0 and not kick_room and creep_room:
                self._lane_creep = True
                self._lane_creeps += 1
            self._lane_pulse += 1
        else:
            self._lane_pulse, self._lane_creep = 0, False
        self.lane_feedback.update(motor=self.lane_motor, creeps=self._lane_creeps)
        self.steering_target, self.natural_steering_target = target, natural
        self.reason = 'light_lane_follow_straight'
        self._slew(now)
        return self._result(now)

    def _light_update(self, scan, now, control):
        if now-self.light_since+1e-9 >= LIGHT_STAGE_MAX_S:
            self._begin_recovery_hold('light_stage_timeout', now, control)
            return self._result(now)
        if not self._route_adopted(control, now):
            return self._result(now)
        space = self._free_space(scan, self._light_heading)
        if space is None:
            self._begin_recovery_hold('light_free_space_unavailable', now, control)
            return self._result(now)
        best = space['best_heading_rad']
        self._light_heading = best
        center = space['center_error_m']
        _, lane = self._current_lane(scan, now)
        # Round 63: a measured lane within LIGHT_LANE_ENTRY_HEADING_RAD hands
        # over once the turn has run LIGHT_TURN_MIN_S; the lane stage then
        # straightens the car (trials 60-62 weaved at 8-18 deg here).
        turned = (self.light_turn_started is not None
                  and now-self.light_turn_started+1e-9 >= LIGHT_TURN_MIN_S)
        lane_aligned = (turned and lane is not None
                        and abs(lane['heading_left_rad']) <= LIGHT_LANE_ENTRY_HEADING_RAD
                        and self._corridor_body_gap(lane) >= MANEUVER_BODY_CLEARANCE_M)
        aligned = lane_aligned or (abs(best) <= LIGHT_ALIGNED_HEADING_RAD
                                   and space['free_ahead_m'] >= LIGHT_ALIGNED_FREE_M
                                   and center is not None and abs(center) <= LIGHT_CENTER_TOLERANCE_M)
        if aligned:
            # Heading is steady while aligned: start measuring the closing speed
            # now so the lane (and its finish cut) begins with a known speed.
            self._lane_speed(scan, self._lane_front_range(scan))
        else:
            self._lane_ranges.clear()
        if aligned and self._mature_observation(self._light_history, scan, .20):
            # Round 58: aligned with the light lane -> drive it slowly to the
            # finish (was: terminal coast here).
            return self._begin_light_lane(scan, now, control, {**space,
                'source_seq': scan['seq'], 'source_at_ms': scan['at_ms'],
                'received_at': scan['received_at'], 'frames': len(self._light_history),
                'measured_lane': lane, 'basis': ('measured_lane_aligned' if lane_aligned
                                                 else 'long_free_lane_ahead_and_balanced_side_clearance'),
                'camera_facing_certified': False})
        if not aligned:
            self._light_history.clear()
        # Toward the deepest free direction; once nearly lined up, also center.
        target = self._free_space_target(space)
        natural = target
        if target < NEUTRAL and (self._route_center_ack is None or self.servo > NEUTRAL):
            target = NEUTRAL  # right only after a current actual center ACK
        self.steering_target, self.natural_steering_target = target, natural
        self.light_feedback = {**space, 'source_seq': scan['seq'], 'nominal_target_pwm': natural,
                               'scope': 'free_space_follow_trial', 'swept_path_certified': False}
        self.reason = 'light_turn_follow_free_space'
        self._slew(now)
        return self._result(now)

    def _relative_target_pwm(self, target):
        # This is the bounded entry stage, not a settled circular orbit. In
        # trial18 the object was still ahead of the lateral axis and approaching;
        # two negative terms erased the adopted left demand immediately after
        # handover. Trial19 additionally inherited an already released 1538.
        # Continue from the configured left reference, rather than treating
        # the wall controller's final release as an orbit feedforward value.
        # The first handover output still holds the current adopted command;
        # subsequent increases obey the normal slew. A target moving behind
        # or farther away adds left correction; current inside-edge clearance
        # can still independently release steering in the caller.
        # Gains are trial PWM feedback, not physical curvature or a course radius.
        reference = self.orbit_reference_range_m
        bearing_term = ORBIT_BEARING_GAIN*(target['bearing_left_rad']-ORBIT_BEARING_RAD)
        range_term = ORBIT_RANGE_GAIN*(target['range_m']-reference)/reference
        correction = max(0., bearing_term) + max(0., range_term)
        bias = self.orbit_bias_pwm if self.orbit_bias_pwm is not None else self.initial_presteer_pwm
        entry_base = max(bias, self.initial_presteer_pwm)
        requested = max(NEUTRAL, min(SERVO_MAX, round(entry_base+correction)))
        # Trial26's accepted initial turn handed over with the real object
        # still left/front. Both old correction terms stayed negative, so the
        # command stopped increasing at1670 and the operator hit the right
        # wall. For this bounded entry only, continue requesting the existing
        # left limit while that same confirmed object has not reached abeam.
        # Actual commands still increase at10/100ms, and current inside points
        # may release left independently. This is not a calibrated orbit law.
        if self.orbit_left_entry_boost:
            if target['bearing_left_rad'] >= ORBIT_BEARING_RAD:
                if self._orbit_abeam_count == 0:
                    self._orbit_abeam_publication = target['source_at_ms']
                    self._orbit_abeam_receive = self.last_now
                self._orbit_abeam_count += 1
                if (self._orbit_abeam_count >= 3
                        and (target['source_at_ms']-self._orbit_abeam_publication)/1000 >= .25
                        and self.last_now-self._orbit_abeam_receive >= .25):
                    self.orbit_left_entry_boost = False
            else:
                self._orbit_abeam_count = 0
                self._orbit_abeam_publication = self._orbit_abeam_receive = None
            if self.orbit_left_entry_boost:
                requested = SERVO_MAX
        self.orbit_feedback = {'source_seq': target.get('source_seq'),
            'source_at_ms': target.get('source_at_ms'), 'scope': 'bounded_orbit_entry_only',
            'adopted_handover_bias_pwm': bias, 'prepared_left_pwm': self.initial_presteer_pwm,
            'entry_base_pwm': entry_base, 'bearing_term_pwm': bearing_term,
            'range_term_pwm': range_term, 'applied_entry_correction_pwm': correction,
            'left_entry_boost_active': self.orbit_left_entry_boost,
            'abeam_confirmations': self._orbit_abeam_count,
            'boost_scope': 'first_target_ahead_at_handover_until_current_abeam_evidence',
            'nominal_target_pwm': requested, 'inside_edge_release_remains_independent': True,
            'physical_curvature_calibrated': False}
        return requested

    def _limit_left_for_known_points(self, nominal, lookahead):
        """Apply a bounded release while a measured inside edge is still nearby.

        This is a relative-point steering heuristic, not a swept-path certificate
        or a mapping from PWM to wheel angle. The reference is the measured rear
        body edge plus the existing clearance, never an assumed rear axle.
        Its clearance bearing is only a correction signal: treating that small
        bearing as an absolute PWM erased the needed exit-turn demand in trial15.
        The hard current-body gate remains independent in the service.
        """
        scan = self._current_scan
        if not isinstance(scan, dict):
            return nominal
        ranges = scan.get('ranges')
        if not isinstance(ranges, list) or len(ranges) != 360:
            return nominal
        rear = REAR_BODY_EXTENT_M + MANEUVER_BODY_CLEARANCE_M
        side = SIDE_BODY_EXTENT_M + MANEUVER_BODY_CLEARANCE_M
        selected = None
        target = nominal
        for index in range(181, 360):
            distance = ranges[index]
            if not _number(distance) or not .02 <= distance <= 12:
                continue  # No interpolation and no free-space assertion.
            angle = math.radians(index)
            x, y = distance*math.cos(angle), -distance*math.sin(angle)
            if not -rear < x <= lookahead or y <= 0:
                continue
            clearance_bearing = math.atan2(max(0., y-side), x+rear)
            point_target = min(SERVO_MAX, NEUTRAL+round(LEFT_TRIAL_GAIN*clearance_bearing))
            if point_target < target:
                target = point_target
                selected = {'index': index, 'range_m': distance,
                            'point_left_m': {'x_m': x, 'y_m': y},
                            'clearance_bearing_rad': clearance_bearing}
        bearing_demand = target
        # Retain the live exit/relative-target demand. Near points can release
        # at most this fraction of the requested left offset from center; this
        # is a trial gain, not a fixed minimum PWM or a physical steering limit.
        target = nominal-round(MAX_INNER_RELEASE_FRACTION*(nominal-bearing_demand))
        self.inner_clearance = {
            'source_seq': scan.get('seq'), 'source_at_ms': scan.get('at_ms'),
            'scope': 'current_known_left_returns_only',
            'nominal_target_pwm': nominal, 'limited_target_pwm': target,
            'clearance_bearing_demand_pwm': bearing_demand,
            'max_release_fraction': MAX_INNER_RELEASE_FRACTION,
            'active': target < nominal, 'limiting_return': selected,
            'lookahead_m': lookahead, 'rear_reference_m': -rear,
            'left_reference_m': side, 'swept_path_certified': False,
            'physical_steering_confirmed': False}
        return target

    def _target(self, candidate, publication_dt):
        # Use the same actual heading/finite-point error as TurnMotion. In this
        # sequence the trend term may release at most 20% of the current error,
        # so a rapidly changing wall estimate does not erase most left steering
        # while the exit is still far from forward. No minimum fixed PWM is
        # introduced: a genuinely smaller current error still releases toward
        # center before the continuation rule below. These are trial gains,
        # not a physical curvature calibration.
        heading, offset = candidate['heading_left_rad'], candidate['center_offset_left_m']
        lookahead = max(.35, min(1., candidate['width_m']/2))
        if self.geometry_mode == 'opening_wall':
            point = self.turn_goal['target_point_left_m']
            error = (.65*heading+.35*math.atan2(point['y_m'], point['x_m']) if point['x_m'] > 0
                     else heading+math.atan2(offset, lookahead))
        else:
            error = heading+math.atan2(offset, lookahead)
        damped = error
        if self.last_error is not None and .05 <= publication_dt < SCAN_AGE_S:
            rate = (error-self.last_error)/publication_dt
            if abs(rate) <= math.radians(90):
                release = error+.35*rate
                if error > 0:
                    damped = max((1-MAX_TREND_RELEASE_FRACTION)*error, min(error, release))
        self.last_error = error
        # Current measured turn error may request the full 1720 bound. The
        # adopted preparation value also anchors the handover phase below;
        # _slew retains the phase/direction step limit per 100 ms.
        target = max(NEUTRAL, min(SERVO_MAX, NEUTRAL+round(LEFT_TRIAL_GAIN*damped)))
        alignment_target = target
        opening_drive = self.phase == 'drive' and self.geometry_mode == 'opening_wall'
        transition_trigger = (opening_drive and heading > 0
                              and self.turn_goal['target_point_left_m']['y_m'] <= 0)
        if transition_trigger:
            self.left_handover_preparing = True
        continuing_left = (opening_drive and heading > 0
                           and (self.left_handover_preparing or self.entry_bearing_released))
        if continuing_left:
            # Opening-wall alignment is not a request to straighten before the
            # next left orbit. Trial19's projected target moved to the right
            # while exit heading was still left, driving its command near1500.
            # The current endpoint-bearing gate can also establish this demand
            # after the early approach. A small projection sign change cannot
            # release it again. Neither condition proves the endpoint is passed.
            # Current geometry, original leases and all hard gates still apply.
            # Initial PWM is the configured continuation reference; the entry
            # gate may have lowered actual neutral preparation, recorded as
            # adopted_presteer_pwm. All increases retain the normal slew.
            target = max(target, self.initial_presteer_pwm)
        self.left_turn_feedback = {
            'source_seq': self._current_scan.get('seq') if self._current_scan else None,
            'source_at_ms': self._current_scan.get('at_ms') if self._current_scan else None,
            'geometry_mode': self.geometry_mode,
            'opening_left_continuation_active': continuing_left,
            'handover_preparation_trigger_current': transition_trigger,
            'alignment_target_pwm': alignment_target,
            'prepared_left_pwm': self.initial_presteer_pwm,
            'nominal_target_pwm': target,
            'inside_edge_release_remains_independent': True,
            'physical_curvature_calibrated': False}
        if self.phase == 'drive':
            target = self._limit_left_for_known_points(target, lookahead)
        target = self._entry_target(target)
        self.left_turn_feedback['endpoint_limited_target_pwm'] = target
        self._last_actual_left_target = target if target > NEUTRAL else None
        return target

    def _orbit_inputs(self, scan, lidar_age_s, now, control, safe, presteer_wait):
        """Same advancing raw-scan/control contract, without a fabricated wall."""
        if not _number(now) or now < self.last_now:
            self.lock('invalid_turn_receive_clock')
            return False
        self.last_now = now
        if safe is not True:
            self.lock('turn_safety_rejected')
            return False
        if type(presteer_wait) is not bool or presteer_wait:
            self.lock('turn_presteer_wait_requires_neutral')
            return False
        received = scan.get('received_at') if isinstance(scan, dict) else None
        ranges = scan.get('ranges') if isinstance(scan, dict) else None
        if (not _number(lidar_age_s) or not 0 <= lidar_age_s < SCAN_AGE_S
                or not isinstance(scan, dict) or scan.get('frame_id') != _FRAME
                or type(scan.get('seq')) is not int or scan['seq'] < 0
                or type(scan.get('at_ms')) is not int or scan['at_ms'] < 0
                or not _number(received) or not 0 <= now-received < SCAN_AGE_S
                or not valid_ranges(ranges)):
            self.lock('turn_scan_stale_or_invalid')
            return False
        if (not isinstance(control, dict) or type(control.get('armed')) is not bool
                or type(control.get('motor')) is not int or type(control.get('servo')) is not int
                or type(control.get('tick')) is not int or control['tick'] < 0
                or type(control.get('seq')) is not int or control['seq'] < 0
                or not min(SERVO_MIN, RIGHT_EXIT_MIN_PWM) <= control['servo'] <= SERVO_MAX):
            self.lock('invalid_turn_adoption_feedback')
            return False
        if self.last_control_tick is not None and (control['tick'] < self.last_control_tick
                or control['seq'] < self.last_control_seq):
            self.lock('reordered_turn_adoption_feedback')
            return False
        self.last_control_tick, self.last_control_seq = control['tick'], control['seq']
        if not control['armed']:
            self.lock('turn_bridge_locked')
            return False
        # Ranges are a validated flat list: tuple() is the same value as the
        # recursive freeze, at C speed (round 83).
        signature = (_freeze(scan.get('corridor_candidates')), _freeze(scan.get('wall_candidates', [])),
                     _freeze(scan.get('left_turn_goal')), tuple(ranges))
        seq, published = scan['seq'], scan['at_ms']
        if self.last_seq is not None and seq == self.last_seq:
            if published != self.last_publication or signature != self.last_signature:
                self.lock('duplicate_turn_scan_changed_content')
            elif now-self.last_receive >= SCAN_AGE_S:
                self.lock('turn_scan_not_advancing')
            return False
        publication_dt = 0 if self.last_publication is None else (published-self.last_publication)/1000
        if self.last_seq is not None and (seq < self.last_seq or publication_dt <= 0):
            self.lock('reordered_turn_scan')
            return False
        if self.last_receive is not None and (publication_dt >= SCAN_AGE_S
                or now-self.last_receive >= SCAN_AGE_S
                or abs(publication_dt-(now-self.last_receive)) > .15):
            self.lock('turn_scan_clock_gap')
            return False
        self.last_seq, self.last_publication, self.last_receive = seq, published, now
        self.last_signature = signature
        return True

    def _orbit_update(self, scan, lidar_age_s, now, control, *, safe, presteer_wait,
                      quality_clear=False, quality_resume_ready=False):
        advancing = self._orbit_inputs(scan, lidar_age_s, now, control, safe, presteer_wait)
        if self.phase == 'locked':
            self._clear_exit_prepare_window()
            return self._result(self.last_now)
        if self.phase == 'quality_wait':
            return self._quality_wait_update(scan, now, control, advancing,
                                             quality_clear, quality_resume_ready)
        if self.phase == 'drive' and now-self.drive_since+1e-9 >= self.max_drive_s:
            self.begin_coast('maneuver_cumulative_drive_timeout', now)
        if self.phase == 'coast':
            if self.coast_since is not None and now-self.coast_since+1e-9 >= COAST_MAX_S:
                self.lock('coast_standstill_unconfirmed')
            else:
                self.steering_target = (self.quality_coast_servo if self.quality_coast_servo is not None
                                        else self.route_end_coast_servo if self.route_end_coast_servo is not None
                                        else NEUTRAL)
                self._slew(now)
            return self._result(now)
        if self.quality_resume_pending:
            points = self._full_target_points(self.compact_target, scan, self.orbit_track_id)
            if (quality_clear is not True or points is None
                    or max(p[0] for p in points) > FRONT_BODY_EXTENT_M
                    or min(p[1] for p in points)-SIDE_BODY_EXTENT_M < MANEUVER_BODY_CLEARANCE_M):
                self._clear_exit_prepare_window()
            if self.first_pass_evidence is not None and advancing:
                _, rejection = self._right_exit_target(scan, now)
                if rejection is not None:
                    self.lock('route_quality_resume_corridor_unconfirmed')
                    return self._result(now)
            if points is None:
                self.lock('route_quality_resume_target_unconfirmed')
            elif (control.get('servo') != self.quality_wait_servo
                    or control.get('motor') not in (NEUTRAL, TRIAL_MOTOR)):
                self.lock('route_quality_resume_feedback_changed')
            elif (control.get('resume_acked') is True and control['motor'] == TRIAL_MOTOR
                  and control.get('command_acked') is True):
                self.quality_resume_pending = False
                self.quality_wait_evidence['resume_ack'] = {k: control.get(k) for k in
                    ('motor', 'servo', 'seq', 'tick')}
                if (self._quality_exit_prepare_ready and self.first_pass_evidence is None
                        and not self._open_loop_orbit()):
                    self._latch_exit_prepare(scan, control,
                        basis='healthy_neutral_wait_body_front_support_rechecked_at_resume_ack')
                    self._quality_exit_prepare_ready = False
                if (self.quality_wait_release is not None and self.first_pass_evidence is None
                        and not self._open_loop_orbit()):
                    # The wait already released left; resumed power must not
                    # restart the orbit demand. Release-only, no right PWM.
                    if not self.first_pass_preparing:
                        self._latch_exit_prepare(scan, control,
                            basis='neutral_wait_release_continued_at_resume_ack',
                            history=self._wait_release_history or [(scan['seq'], scan['at_ms'],
                                                                    scan['received_at'])])
                    self.first_pass_preparing = True
                    self.orbit_left_entry_boost = False
            elif now-self.quality_resume_since >= SCAN_AGE_S:
                self.lock('route_quality_resume_ack_timeout')
            else:
                self.steering_target = self.quality_wait_servo
            if self.phase == 'locked':
                self._clear_exit_prepare_window()
            # Even the ACK observation cannot change steering on this output.
            return self._result(now)
        if not advancing:
            if (self._second_plan_pending == (scan.get('seq'), scan.get('at_ms'))
                    and self.continue_route and self.phase == 'drive' and self.route_stage == 'second_loop'
                    and not self.quality_resume_pending and 0 <= now-scan['received_at'] < SCAN_AGE_S):
                # Round 87: the second loop's first plan, deferred from the
                # tick that confirmed the support on this same scan (that tick
                # had the adopted command; it sent no new one).
                return self._open_second_loop_update(scan, now, control, adopted=True)
            # A repeated fresh observation cannot establish preparation or a
            # pass. It may execute ONE already-authorized release step once
            # the previous actual command is ACKed and its 100 ms is due.
            loop_step = (self._open_loop_orbit() and self.phase == 'drive' and self.reason == 'open_first_loop'
                         and 0 <= now-scan['received_at'] < SCAN_AGE_S
                         and control.get('command_acked') is True
                         and control.get('motor') in DRIVE_MOTORS and control.get('servo') == self.servo)
            if loop_step:
                # The latest fresh frame's loop command, one step per 100 ms,
                # so the left build-up does not wait for a fresh-frame phase.
                self._slew(now)
            elif (self.continue_route and self.phase == 'drive' and not self._open_loop_orbit()
                    and self.handover_observed and self.route_stage == 'first_target'
                    and self.first_pass_preparing and self.first_pass_evidence is None
                    and NEUTRAL <= self.servo <= SERVO_MAX
                    and self._full_target_points(self.compact_target, scan, self.orbit_track_id) is not None
                    and 0 <= now-scan['received_at'] < SCAN_AGE_S
                    and control.get('command_acked') is True
                    and control.get('motor') in DRIVE_MOTORS and control.get('servo') == self.servo):
                # Route mode: no orbit-entry clock (trial50); the cumulative
                # drive budget and the service deadline still bound it. A
                # repeated frame reuses the last measured release target.
                cached = self._exit_release_cache
                self.steering_target = self.natural_steering_target = (
                    max(NEUTRAL, min(self.servo, cached[1])) if cached is not None else NEUTRAL)
                self.reason = 'first_target_pass_preparing_neutral'
                self._slew(now)
            return self._result(now)
        if self.continue_route and self.route_stage == 'light_turn':
            return self._light_update(scan, now, control)
        if self.continue_route and self.route_stage == 'light_lane':
            return self._light_lane_update(scan, now, control)
        if self.continue_route and self.route_stage == 'corridor_follow':
            return self._corridor_follow_update(scan, now, control)
        if self.continue_route and self.route_stage == 'second_orbit':
            return self._second_orbit_update(scan, now, control)
        if self.continue_route and self.route_stage == 'open_transfer':
            return self._open_transfer_update(scan, now, control)
        if self.continue_route and self.route_stage == 'second_loop':
            return self._open_second_loop_update(scan, now, control)
        if self.continue_route and self.route_stage == 'light_entry':
            return self._open_light_entry_update(scan, now, control)
        if self.first_pass_evidence is not None:
            return self._right_exit_update(scan, now, control)
        if (self.continue_route and self.entry_kind == 'lane' and self.handover_observed
                and self.route_stage == 'first_target'):
            return self._open_first_loop_update(scan, now, control)
        held = getattr(self.target_tracker, 'held', None)
        if (self.continue_route and self.compact_target is None and self.handover_observed
                and isinstance(held, dict) and held.get('track_id') == self.orbit_track_id):
            # Tolerant tracker kept the orbited identity through a missing
            # frame (trial50/51 flicker): keep the current command this frame,
            # except that a support last seen at/behind the body front (or an
            # already latched release) keeps releasing toward the exit
            # (trial53: holding 1691 here over-rotated the car 66 deg past the
            # lane). The body-front window is kept, not cleared.
            last_front = max((p[0] for p in self._held_support_points(held)), default=None)
            if self.first_pass_preparing or (last_front is not None and last_front <= FRONT_BODY_EXTENT_M):
                self.steering_target = self._exit_release_target(scan, now)
                self.reason = 'first_target_missing_frame_release'
            else:
                self.steering_target = self.servo
                self.reason = 'first_target_missing_frame_hold'
            self._slew(now)
            return self._result(now)
        if not self._usable_target(self.compact_target, scan):
            self._clear_exit_prepare_window()
            self._capture_compact_loss(scan, now, control, 'compact_target_lost_or_ambiguous')
            # Losing object identity removes the drive request, not the
            # already adopted left steering. Keep that command only in the
            # existing bounded neutral coast; never reacquire powered motion.
            if self.continue_route and self.begin_quality_wait(now, control, scan=scan):
                # Self-recovery: hold at neutral and re-find the same support.
                self.reason = 'route_orbit_target_recovery_wait'
            elif not self._begin_adopted_left_coast('compact_target_lost_or_ambiguous', now, control):
                self.begin_coast('compact_target_lost_or_ambiguous', now)
            self._slew(now)
            return self._result(now)
        just_handed_over = False
        if not self.handover_observed:
            # Continue the adopted left command across normal stage handover.
            # A target never bypasses an ACK, neutral/protection stop or re-arms
            # the bridge. This is command feedback, not physical wheel angle.
            adopted = (control.get('motor') in (DRIVE_MOTORS if self.continue_route else (TRIAL_MOTOR,))
                       and control.get('servo') == self.servo > NEUTRAL
                       and control.get('command_acked') is True)
            if not adopted:
                if self.handover_wait_since is None:
                    self.handover_wait_since = now
                if now-self.handover_wait_since+1e-9 >= SCAN_AGE_S:
                    self.begin_coast('compact_handover_adoption_unconfirmed', now)
                    self._slew(now)
                else:
                    self.reason = 'compact_handover_waiting_adopted_left'
                return self._result(now)
            self.handover_observed = True
            self._route_left_hold_since = None
            if self.continue_route:
                # Trial54: the car passed the support while the tracker lost it
                # beside the body; it reappeared behind-left (128-163 deg) where
                # no new identity could be acquired, so the pass was never
                # observed and the wait stalled. After handover the first
                # support is anywhere on the left (task rule: anticlockwise),
                # so re-acquisition covers the rear-left too.
                self.target_tracker.widen_acquisition((math.radians(45), math.pi))
            just_handed_over = True
            self.orbit_since = now
            self.orbit_reference_range_m = self.compact_target['range_m']
            self.orbit_bias_pwm = control['servo']
            self.orbit_left_entry_boost = self.compact_target['bearing_left_rad'] < ORBIT_BEARING_RAD
            self.handover_control = {key: control[key] for key in ('motor', 'servo', 'tick', 'seq')}
            self.handover_control.update(command_acked=True, physical_steering_confirmed=False,
                target_source_seq=scan['seq'], target_source_at_ms=scan['at_ms'])
            self.orbit_track_id = self.compact_target['track_id']
            if self.continue_route:
                self.target_tracker.enable_close_maintenance(self.orbit_track_id)
            self.wall_ambiguity_hold = False
            self.geometry_missing = True
            self.alignment_count = 0
            self.alignment_evidence = None
            self.alignment_publication = self.alignment_receive = None
        elif self.compact_target['track_id'] != self.orbit_track_id:
            self._clear_exit_prepare_window()
            self._capture_compact_loss(scan, now, control, 'compact_target_identity_changed')
            if self.continue_route and self.begin_quality_wait(now, control, scan=scan):
                self.reason = 'route_orbit_target_recovery_wait'
            elif not self._begin_adopted_left_coast('compact_target_identity_changed', now, control):
                self.begin_coast('compact_target_identity_changed', now)
            self._slew(now)
            return self._result(now)
        if not self.continue_route and now-self.orbit_since+1e-9 >= ORBIT_ENTRY_MAX_S:
            self.begin_coast('first_relative_object_entry_trial_timeout', now)
            self._slew(now)
            return self._result(now)
        if just_handed_over:
            self.steering_target = self.natural_steering_target = self.orbit_bias_pwm
            self.reason = 'first_relative_object_handover_keep_adopted_left'
            return self._result(now)
        self._observe_first_pass(scan, now, control)
        if self.first_pass_evidence is not None:
            return self._right_exit_update(scan, now, control)
        if self.first_pass_preparing:
            self.steering_target = self.natural_steering_target = self._exit_release_target(scan, now)
            self.reason = 'first_target_pass_preparing_neutral'
            self._slew(now)
            return self._result(now)
        self.steering_target = self._limit_left_for_known_points(
            self._relative_target_pwm(self.compact_target),
            max(.35, min(1., self.compact_target['range_m'])))
        self.natural_steering_target = self.steering_target
        self.reason = 'first_relative_object_left_orbit_entry'
        self._slew(now)
        return self._result(now)

    def update(self, scan, lidar_age_s, now, control, *, safe=True, entry_stop=None, presteer_wait=False,
               quality_clear=False, quality_resume_ready=False):
        # Expose only this call's raw returns; cached wall endpoints are never
        # used to manufacture current inside-edge evidence.
        self._current_scan = scan
        self._avoid = self._route_proximity(scan) if self.continue_route and self.phase != 'locked' else None
        if self._avoid is not None:
            self.avoid_count += 1
            if len(self.avoid_evidence) < 40:
                self.avoid_evidence.append({**self._avoid, 'phase': self.phase, 'route_stage': self.route_stage,
                                            'servo': self.servo})
        if (not isinstance(scan, dict) or self.inner_clearance is None
                or self.inner_clearance['source_seq'] != scan.get('seq')
                or self.inner_clearance['source_at_ms'] != scan.get('at_ms')):
            self.inner_clearance = None
        if self.phase == 'locked':
            return self._result(self.last_now)
        if not _number(now) or now < self.last_now:
            self.lock('invalid_turn_receive_clock')
            return self._result(self.last_now)
        if self.continue_route and isinstance(scan, dict) and type(scan.get('at_ms')) is int:
            # Round 64: one compass step per newly published scan (repeats ignored).
            self.compass.update(scan)
        if self.compact_target is not None and self.compact_target.get('confirmed') is True:
            self._last_confirmed_compact_target = self.compact_target
        self.compact_target_error = None
        try:
            self.compact_target = (self.target_tracker.update(scan, now, acquire=self.first_pass_evidence is None,
                                                              heading_rad=self._tracking_heading(scan))
                                   if self.continue_route else self.target_tracker.update(scan, now))
            # Corridor following while driving runs its own recovery plus
            # corridor proof on this scan (_second_role_observation); feeding
            # the tracker here first would consume the frame.
            corridor_drive = (self.route_stage == 'corridor_follow' and self.phase == 'drive'
                              and not self.quality_resume_pending)
            if self.continue_route and (self.route_stage in ('second_orbit', 'second_loop')
                                        or (self._second_recovering and not corridor_drive)):
                self._update_second_tracking(scan, now)
        except (ValueError, TypeError, KeyError) as error:
            self.compact_target = None
            self.compact_target_error = type(error).__name__
        if isinstance(self.compact_target, dict) and _number(now):
            self._last_seen_compact = (now, copy.deepcopy(self.compact_target))
        if entry_stop is not None:
            # The new trial is target tracking, not the old optional stop input.
            self.lock('unsupported_maneuver_entry_stop')
            return self._result(self.last_now)
        if (self.continue_route and self.neutral_pause_kind is not None
                and (self.phase == 'quality_wait' or self.quality_resume_pending)):
            return self._neutral_pause_update(scan, lidar_age_s, now, control, safe=safe,
                presteer_wait=presteer_wait, quality_clear=quality_clear,
                resume_ready=quality_resume_ready)
        if self.continue_route and self.entry_kind is None and self.phase == 'presteer':
            self._decide_entry(scan)
        if self.continue_route and self.route_stage == 'start_lane':
            if not (self.phase == 'drive' and self._start_lane_handover_ready(scan, control)):
                return self._start_lane_update(scan, lidar_age_s, now, control, safe=safe,
                                               presteer_wait=presteer_wait)
            # The support is round on the left with an adopted left command:
            # the existing first-target orbit takes over on this same frame.
            self.route_stage = 'first_target'
            self._route_event('first_approach_handover', scan, control, self.first_approach)
        # A held quality coast (route mode may start one in the left turn)
        # only validates advancing scans/control and keeps its ACKed servo;
        # no geometry association can recenter it while still rolling.
        if (self.handover_observed
                or (self.phase == 'coast' and (self.wall_ambiguity_hold
                                               or self.quality_coast_servo is not None))
                or (self.phase == 'drive' and self._usable_target(self.compact_target, scan))):
            if self.handover_observed:
                self._route_left_hold_since = None
            return self._orbit_update(scan, lidar_age_s, now, control,
                                      safe=safe, presteer_wait=presteer_wait,
                                      quality_clear=quality_clear, quality_resume_ready=quality_resume_ready)
        if self._route_left_hold_since is not None and self.phase == 'drive':
            return self._route_left_hold_update(scan, lidar_age_s, now, control, safe, presteer_wait)
        self.wall_ambiguity_hold = False
        feedback_tick = control.get('tick') if isinstance(control, dict) else None
        self._presteer_adoption_context = {
            'control': dict(control) if isinstance(control, dict) else {},
            'fresh_feedback': (type(feedback_tick) is int
                               and (self.last_control_tick is None or feedback_tick > self.last_control_tick))}
        try:
            return super().update(scan, lidar_age_s, now, control, safe=safe,
                                  presteer_wait=presteer_wait)
        finally:
            self._presteer_adoption_context = None
