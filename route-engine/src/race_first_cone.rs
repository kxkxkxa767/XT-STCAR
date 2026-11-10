//! A cone close to the end wall ahead, or hidden behind the divider's end.
//!
//! Rule-range simulation (11-08): when the end wall stood under about 2 m past the divider the
//! car hit it. Steering climbs 40 PWM per 0.1 s here, so full lock took 0.55 s, plus the servo's
//! own lag: about 1.2 m at 1.5 m/s. This turns as late as the wall allows (which also keeps
//! the most room to a cone standing near that wall), steps faster, and stops pushing.
//! Cone one turns left off the lane axis, cone two right off the reversed axis.
use crate::helpers::*;
use crate::maneuver::*;
use crate::py::*;

/// Full left as commanded in the first loop's output gain (about 0.73 m); less in the lane.
const FULL_LEFT_RADIUS_M: f64 = 0.75;
/// Full right: 1.2 per metre nominal, about 0.7-0.83 m on the wheels.
const FULL_RIGHT_RADIUS_M: f64 = 0.80;
pub const WALL_TURN_PWM_STEP: i64 = 120;
const STEER_RESPONSE_S: f64 = 0.30;
const WALL_MARGIN_M: f64 = 0.10;
/// A quarter of the full-left circle must be clear before committing to it.
const ARC_CLEAR_M: f64 = 0.9;
const STOP_PUSHING_ABOVE_MPS: f64 = 1.1;
const SPEED_IF_UNKNOWN_MPS: f64 = 1.5;
/// Past this the wall no longer needs the turn; a blind turn levels off here.
const TURNED_RAD: f64 = 75. * PI / 180.;

/// A cone first seen this far round the car stood behind the divider's end (rules Fig. 5).
pub const LATE_SIGHT_BEARING_RAD: f64 = 60. * PI / 180.;
/// Round a late cone at the speed that tracks a wide approach to its circle.
pub const LATE_ENTRY_MPS: f64 = 1.2;

#[derive(Clone, Debug, Default)]
pub struct WallTurn {
    /// Full left is being commanded because of the wall.
    pub active: bool,
    /// It started with no cone in view, so it continues until one appears.
    blind: bool,
    speed: Option<f64>,
    /// Cone one came into view already abreast of the car.
    pub late_sight: bool,
}

impl WallTurn {
    /// Coast instead of pushing into the turn.
    pub fn stop_pushing(&self) -> bool {
        self.active && self.speed.is_none_or(|v| v > STOP_PUSHING_ABOVE_MPS)
    }

    pub fn diagnostics(&self) -> V {
        crate::dict! {"active" => self.active, "blind" => self.blind, "late_sight" => self.late_sight}
    }

    /// Cone one's entry speed.
    pub fn first_entry_mps(&self) -> f64 {
        if self.late_sight { LATE_ENTRY_MPS } else { crate::race_crosswalk::FIRST_ENTRY_MPS }
    }

    /// Turn in at the emergency step rate: into either lock for the wall, leftward for a late cone.
    pub fn fast_steering(&self, route_stage: &str, target: i64, servo: i64) -> bool {
        self.active || (self.late_sight && matches!(route_stage, "start_lane" | "first_target") && target > servo)
    }

    /// Hold the loop speed down until the cone is round.
    pub fn slow_loop(&self, speed: Option<f64>) -> bool {
        self.late_sight && speed.is_some_and(|v| v > LATE_ENTRY_MPS)
    }
}

/// Loop one in a race: a car running inside its plan's circle pursues a wider one.
///
/// y53-y74, 15 runs: between headings 45 and 150 degrees the car held 0.02-0.19 m inside the
/// plan's circle in every run (median 0.11) and passed cone one that much nearer than planned:
/// 0.27-0.47 m from its centre where the plan meant 0.50 (y74 0.29, a 0.052 m body gap; y57
/// 0.27). With the play taken up the wheels turn the car about 1.3 times the steering table (54
/// settled frames, quartiles 1.26-1.33, single runs to 1.48) against the 1.2 this loop's output
/// assumes, and a pursuit whose car over-turns settles inside its circle. A car within
/// RADIAL_DEADBAND_M of the plan (y53, y58, y61, y71) is left as it is; loop two is balanced
/// between its cone and the outer wall and is not touched. A cone that came into view late is
/// rounded as before: no real run has done that, and in the model it only moved cases around.
pub const RADIAL_GAIN: f64 = 1.5;
pub const RADIAL_DEADBAND_M: f64 = 0.10;
pub const RADIAL_MAX_M: f64 = 0.25;
/// The loop is entered from inside its circle; the car has settled on its arc by this heading.
pub const RADIAL_FROM_RAD: f64 = 35. * PI / 180.;

/// The radius the carrot is placed on for a car `r_now` from the plan's centre.
pub fn held_radius(plan_radius: f64, r_now: f64) -> f64 {
    plan_radius + (RADIAL_GAIN * (plan_radius - RADIAL_DEADBAND_M - r_now)).clamp(0., RADIAL_MAX_M)
}

/// Metres to spare before a full-lock arc starting after the steering response would no longer
/// clear the wall ahead. `ahead` is measured along the axis, `turned` from it toward the turn.
pub fn wall_room(ahead: f64, turned: f64, speed: f64, servo: i64, full: i64, radius: f64) -> f64 {
    let outer = hypot(radius + SIDE_BODY_EXTENT_M, FRONT_BODY_EXTENT_M);
    // The part of the wheel travel still to come sets the wait: wheels already nearly at full
    // lock answer at once.
    let remaining = (full - servo).abs() as f64;
    let response_s = STEER_RESPONSE_S * remaining / (full - NEUTRAL).abs() as f64 + remaining / WALL_TURN_PWM_STEP as f64 * PWM_INTERVAL_S;
    ahead + radius * turned.psin() - outer - speed * response_s * turned.pcos().max(0.)
}

/// The distance to the wall along the axis: the second nearest return around it, skipping the
/// cone's own bins (a cone on the axis is what the loop goes round, not the wall to clear). One
/// stray near return must not turn the car.
fn axis_front_range(ranges: &[V], axis_bin: i64, cone_bins: &[i64]) -> Option<f64> {
    let mut near: Vec<f64> = (axis_bin - LANE_FRONT_HALF_BINS..=axis_bin + LANE_FRONT_HALF_BINS)
        .filter(|i| !cone_bins.contains(&imod(*i, 360)))
        .filter_map(|i| ranges[imod(i, 360) as usize].of().map(|r| r * radians((i - axis_bin) as f64).pcos()))
        .collect();
    near.sort_by(f64::total_cmp);
    near.get(1).copied()
}

impl Maneuver {
    /// The steering the end wall demands in this scan, if any. `side` is +1 for the left turn
    /// off the lane axis, -1 for the right turn off the reversed one.
    pub(crate) fn wall_turn_command(&mut self, scan: &V, now: f64, side: i64, cone: &V) -> Option<i64> {
        self.wall_turn.active = false;
        let input = self.race_inputs?;
        if self.crosswalk_state != Some("done") || (side > 0 && self.first_pass_evidence.is_some()) {
            return None;
        }
        let heading = self.compass.current(scan)?;
        let axis = if side > 0 { 0. } else { PI };
        let turned = side as f64 * wrap(heading - axis);
        let (full, radius) = if side > 0 { (SERVO_MAX, FULL_LEFT_RADIUS_M) } else { (nominal_right_limit_pwm(), FULL_RIGHT_RADIUS_M) };
        let speed = self.race_surface_speed.speed(now, input.forward_integral)
            .or(if side > 0 { self.race_crosswalk.imu_speed(input.forward_integral) } else { None });
        self.wall_turn.speed = speed;
        let cone_in_view = cone.is_dict();
        if cone_in_view {
            self.wall_turn.blind = false;
        }
        let center = round(degrees(wrap(heading - axis)));
        let cone_bins = if cone_in_view { ints_of(cone.get("support_bins")) } else { Vec::new() };
        let needed = turned < TURNED_RAD && axis_front_range(scan.idx("ranges").l(), center, &cone_bins).is_some_and(|ahead| {
            wall_room(ahead, turned, speed.unwrap_or(SPEED_IF_UNKNOWN_MPS), self.servo, full, radius) <= WALL_MARGIN_M
        });
        if needed {
            let points = Self::path_points(scan)?;
            let arc = Self::path_free_distance(&points, full);
            if arc >= ARC_CLEAR_M || arc >= Self::path_free_distance(&points, NEUTRAL) {
                self.wall_turn.active = true;
                self.wall_turn.blind = !cone_in_view;
                return Some(full);
            }
            return None;
        }
        if !self.wall_turn.blind {
            return None;
        }
        // No cone yet: keep turning, then run along the wall until the tracker has one.
        self.wall_turn.active = turned < TURNED_RAD;
        Some(if self.wall_turn.active {
            full
        } else {
            // Right steering is not available before the first pass.
            clampi(NEUTRAL + round(LEFT_TRIAL_GAIN * (TURNED_RAD - turned)), NEUTRAL, NEUTRAL + OPEN_HEADING_HOLD_PWM)
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_room_shrinks_with_speed_and_grows_with_the_turn_already_made() {
        let room = |ahead, turned, speed, servo| wall_room(ahead, turned, speed, servo, SERVO_MAX, FULL_LEFT_RADIUS_M);
        let straight = room(1.8, 0., 1.5, NEUTRAL);
        assert!(room(1.8, 0., 1.0, NEUTRAL) > straight);
        assert!(room(1.8, 0.5, 1.5, NEUTRAL) > straight);
        assert!(room(1.8, 0., 1.5, SERVO_MAX) > straight);
    }

    #[test]
    fn a_car_well_inside_its_circle_is_held_on_a_wider_one() {
        assert_eq!(held_radius(0.9, 0.86), 0.9); // y71
        assert_eq!(held_radius(0.9, 1.05), 0.9);
        let y74 = held_radius(0.9, 0.70);
        assert!((y74 - 1.05).abs() < 1e-9, "{y74}");
        assert_eq!(held_radius(0.9, 0.5), 0.9 + RADIAL_MAX_M);
    }

    #[test]
    fn at_rate_it_needs_about_a_metre_and_three_quarters() {
        let room = wall_room(1.75, 0., 1.5, NEUTRAL, SERVO_MAX, FULL_LEFT_RADIUS_M);
        assert!(room.abs() < 0.15, "{room}");
    }
}
