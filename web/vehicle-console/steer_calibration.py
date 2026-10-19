"""Measured steering of this car: nominal route PWM -> physical servo PWM.

ManeuverSequence and the console checks reason in a nominal linear servo
space (agent.md section 5: curvature = (servo-1500)/160 per m). The real
steering has play and lags:

- play: the wheels follow the servo only once it has moved through about
  +-40 PWM (after a right turn 1500 keeps the wheels right of centre, after a
  left turn left of it);
- gain: on the track (rounds 70-78, 176 drive samples: path curvature rebuilt
  by scripts/trial-map.py against the logged physical servo with .3 s lag;
  docs/vehicle-dynamic-turn-validation-20261007.json round79_analysis_20261019
  and turn79_steer_calibration_20261019) the wheel position maps to curvature
  close to linearly, about 155 PWM per 1/m on both sides (play 40, centre 0;
  every leave-one-round-out fit: play 40, 150). The 10-16 tile table
  (1250 -1.2, 1350 -.38, 1450 -.16, 1550 .18, 1600 .4, 1670 1.6) understated
  moderate turns about 2x and made SteerOutput over-drive them (start-lane
  weave, round 78 second-loop cross-over). Right is kept at the -1.2 /m the
  planners count on (nominal 1308); left ends where the physical range ends.

SteerOutput turns each nominal command into the physical command that puts
the wheels at the curvature the nominal command means, crossing the play in
the direction of the change. It keeps its own estimate of the wheel position
from the commands it issued. Nominal 1500 at rest stays physical 1500. This is
a vehicle calibration, not a course parameter.
"""
import math

NEUTRAL = 1500
NOMINAL_PWM_PER_CURVATURE = 160.
# Wheel PWM (servo position after the play) -> curvature, 1/m (155 PWM per
# 1/m, track fit).
STEER_TABLE = ((1314, -1.2), (1500, 0.), (1720, 1.42))
STEER_PLAY_PWM = 40
PHYSICAL_RANGE = (1200, 1720)
# Nominal commands beyond what the wheels reach saturate there (planners must
# not count on more): right -1.2 /m (nominal 1308); left the physical 1720
# leaves the wheels at 1680 after the play, about 1.16 /m (nominal ~1686; the
# track samples at physical 1680-1700 measured 1.12-1.16 /m).
NOMINAL_RIGHT_LIMIT_PWM = NEUTRAL+math.ceil(STEER_TABLE[0][1]*NOMINAL_PWM_PER_CURVATURE)


def wheel_for_curvature(kappa):
    """Inverse of STEER_TABLE (monotonic), clamped to its ends."""
    if kappa <= STEER_TABLE[0][1]:
        return float(STEER_TABLE[0][0])
    for (w0, k0), (w1, k1) in zip(STEER_TABLE, STEER_TABLE[1:]):
        if kappa <= k1:
            return w0+(w1-w0)*(kappa-k0)/(k1-k0)
    return float(STEER_TABLE[-1][0])


def curvature_for_wheel(wheel):
    if wheel <= STEER_TABLE[0][0]:
        return STEER_TABLE[0][1]
    for (w0, k0), (w1, k1) in zip(STEER_TABLE, STEER_TABLE[1:]):
        if wheel <= w1:
            return k0+(k1-k0)*(wheel-w0)/(w1-w0)
    return STEER_TABLE[-1][1]


class SteerOutput:
    """Stateful nominal -> physical servo mapping with play compensation."""
    def __init__(self, play=STEER_PLAY_PWM):
        if not 0 <= play < 150:
            raise ValueError('steering play out of range')
        self.play = play
        self.wheel = float(NEUTRAL)  # estimated wheel position (servo PWM units)
        self.last = NEUTRAL

    def physical(self, nominal, play=None):
        """``play``: the play to cross for this command (default the measured
        one); the wheel estimate follows the same assumption."""
        if type(nominal) is not int:
            raise ValueError('nominal servo must be an int PWM')
        play = self.play if play is None else play
        if not 0 <= play <= self.play:
            raise ValueError('steering play out of range')
        target = wheel_for_curvature((nominal-NEUTRAL)/NOMINAL_PWM_PER_CURVATURE)
        if target > self.wheel+.5:
            command = target+play
        elif target < self.wheel-.5:
            command = target-play
        else:
            # Wheels already there: keep pushing with the same command (the
            # servo lags; releasing to the target after one frame of the
            # overshoot left the wheels short of it). Rounding allowance 1 PWM.
            command = self.last if abs(self.last-target) <= play+1 else target
        command = int(round(max(PHYSICAL_RANGE[0], min(PHYSICAL_RANGE[1], command))))
        self.wheel = min(command+play, max(command-play, self.wheel))
        self.last = command
        return command
