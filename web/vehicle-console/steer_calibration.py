"""Measured steering of this car: nominal route PWM -> physical servo PWM.

ManeuverSequence and the console checks reason in a nominal linear servo
space (agent.md section 5: curvature = (servo-1500)/160 per m). The real
steering (tile floor, 2026-10-13/16, docs/vehicle-dynamic-turn-validation-
20261007.json: steering_arc_after_repair_20261016) is neither linear nor free
of play:

- play: the wheels follow the servo only once it has moved through about
  +-50 PWM (after a right turn 1500 keeps about -.2 /m, after a left turn about
  +.2 /m; a 1585 kick over-corrected);
- gain: the wheel position maps to curvature through the measured table;
  right saturates near 1250 (-1.2 /m), left near 1670 (+1.6 /m).

SteerOutput turns each nominal command into the physical command that puts
the wheels at the curvature the nominal command means, crossing the play in
the direction of the change. It keeps its own estimate of the wheel position
from the commands it issued. Nominal 1500 at rest stays physical 1500. This is
a vehicle calibration, not a course parameter.
"""
import math

NEUTRAL = 1500
NOMINAL_PWM_PER_CURVATURE = 160.
# Wheel PWM (servo position after the play) -> curvature, 1/m.
STEER_TABLE = ((1250, -1.2), (1350, -.38), (1450, -.16), (1500, 0.),
               (1550, .18), (1600, .4), (1670, 1.6))
STEER_PLAY_PWM = 50
PHYSICAL_RANGE = (1200, 1720)
# Nominal commands beyond what the wheels reach saturate there (planners must
# not count on more): right -1.2 /m (nominal 1308), left +1.6 /m (beyond the
# nominal 1720 = 1.375 /m, so the nominal range is reachable on the left).
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

    def physical(self, nominal):
        if type(nominal) is not int:
            raise ValueError('nominal servo must be an int PWM')
        target = wheel_for_curvature((nominal-NEUTRAL)/NOMINAL_PWM_PER_CURVATURE)
        if target > self.wheel+.5:
            command = target+self.play
        elif target < self.wheel-.5:
            command = target-self.play
        else:
            # Wheels already there: keep pushing with the same command (the
            # servo lags; releasing to the target after one frame of the
            # overshoot left the wheels short of it). Rounding allowance 1 PWM.
            command = self.last if abs(self.last-target) <= self.play+1 else target
        command = int(round(max(PHYSICAL_RANGE[0], min(PHYSICAL_RANGE[1], command))))
        self.wheel = min(command+self.play, max(command-self.play, self.wheel))
        self.last = command
        return command
