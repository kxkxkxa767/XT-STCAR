"""Read-only diagnostic hooks for a disposable, pinned source archive only."""
from pathlib import Path
import sys
root=Path(sys.argv[1]).resolve()
def edit(path,old,new):
 p=root/path;s=p.read_text();assert s.count(old)==1,(path,old[:120],s.count(old));p.write_text(s.replace(old,new))
nav='crates/robot-core/src/navigation.rs'
edit(nav,'pub struct Navigator {','#[derive(Serialize)]\npub struct Navigator {')
edit(nav,'#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]\npub enum TargetPolicy','#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize)]\npub enum TargetPolicy')
edit('crates/robot-core/src/navigation/continuity.rs','#[derive(Clone, Copy, Debug)]\npub(super) struct CachedTerminalSeed','#[derive(Clone, Copy, Debug, Serialize)]\npub(super) struct CachedTerminalSeed')
t='crates/robot-core/src/navigation/terminal.rs'
edit(t,'#[derive(Clone, Copy, Debug, Default)]\nstruct WorkAllowance','#[derive(Clone, Copy, Debug, Default, Serialize)]\nstruct WorkAllowance')
edit(t,'pub(super) struct TerminalBudget {','#[derive(Serialize)]\npub(super) struct TerminalBudget {')
edit(t,'#[derive(Clone, Copy, Debug)]\n#[cfg_attr(test, derive(serde::Deserialize, serde::Serialize))]\npub(super) struct TwoArcSeed','#[derive(Clone, Copy, Debug, Serialize)]\n#[cfg_attr(test, derive(serde::Deserialize))]\npub(super) struct TwoArcSeed')
edit(t,'        let mut work = self.work.get();\n        if work.budget_exhausted || self.cold_deferred.get() {','        let mut work = self.work.get();\n        super::capture_event(|| serde_json::json!({"kind":"terminal_charge_before", "work":work,"request":[solvers,iterations,samples],"cold_ceiling":self.cold_ceiling.get(),"cold_deferred":self.cold_deferred.get()}));\n        if work.budget_exhausted || self.cold_deferred.get() {')
# This module exists only in an isolated archived source copy.
edit(nav,'impl Navigator {','''std::thread_local! {
    static CAPTURE: std::cell::RefCell<Option<Vec<serde_json::Value>>> = const { std::cell::RefCell::new(None) };
}
pub fn capture_begin() { CAPTURE.with(|v| *v.borrow_mut() = Some(Vec::new())); }
pub fn capture_end() -> Vec<serde_json::Value> { CAPTURE.with(|v| v.borrow_mut().take().unwrap_or_default()) }
fn capture_event(f: impl FnOnce() -> serde_json::Value) {
    CAPTURE.with(|v| { if let Some(events) = v.borrow_mut().as_mut() { assert!(events.len() < 100_000); events.push(f()); } });
}
impl Navigator {''')
edit(nav,'    let count = (length / (grid.resolution / 3.0).min(0.025))','''    capture_event(|| serde_json::json!({"kind":"primitive_begin", "start":start, "initial_curvature":initial_curvature,"target_curvature":target_curvature,"length":length,"error":[error.position_m,error.heading_rad],"recovery":grid.recovery_active()}));
    let count = (length / (grid.resolution / 3.0).min(0.025))''')
r='crates/robot-core/src/navigation/recovery.rs'
old='''        self.oriented_motion_transition_clear(from, to, length, error)
            || (self.recovery_active()'''
edit(r,old,'''        let accepted = self.oriented_motion_transition_clear(from, to, length, error)
            || (self.recovery_active()''')
old='''                        .oriented_boundary_with_padding(from, to, length, error, padding)
                }))
    }'''
edit(r,old,'''                        .oriented_boundary_with_padding(from, to, length, error, padding)
                }));
        if !accepted {
            super::capture_event(|| {
                let arc_padding = self.recovery.maximum_curvature * length.powi(2) / 8.0 + error.position_m;
                let chord_padding = model.and_then(|(motion, duration)| body_chord_padding(self.recovery.footprint, motion, duration));
                serde_json::json!({"kind":"motion_rejected", "from":from,"to":to,"length":length,"error":[error.position_m,error.heading_rad],
                    "recovery":self.recovery_active(), "oriented":self.recovery.oriented_boundary,
                    "capsule":self.motion_transition_clear_with_error(from.point(),to.point(),length,error.position_m),
                    "bounds_and_obstacles":self.recovery.segment_clear_domain(from.point(),to.point(),arc_padding,None),
                    "first_order":self.recovery.oriented_boundary_clear(from,to,length,error),
                    "chord_padding":chord_padding,
                    "second_order":chord_padding.is_some_and(|p| self.recovery.oriented_boundary_with_padding(from,to,length,error,p)),
                    "boundary":self.recovery.boundary, "capsule_radius":self.recovery.radius+arc_padding,
                    "model":model.map(|(m,dt)| format!("{m:?}, duration={dt:?}"))})
            });
        }
        accepted
    }''')
edit('crates/runner/src/control_execution.rs','#[derive(Clone, Debug)]\npub struct PlanningContext','#[derive(Clone, Debug, serde::Serialize)]\npub struct PlanningContext')
a='crates/runner/src/autonomy.rs'
edit(a,'        self.navigation.set_travel_boundary(travel_boundary);','''        let capture = matches!(at.0, 33180 | 33260 | 33380);
        let before_boundary = capture.then(|| serde_json::to_value(&self.navigation).unwrap());
        self.navigation.set_travel_boundary(travel_boundary);''')
edit(a,'                let decision = self.navigation.plan_with_arrival(','                let before_plan = capture.then(|| serde_json::to_value(&self.navigation).unwrap());\n                if capture { xt_stcar_robot_core::navigation::capture_begin(); }\n                let decision = self.navigation.plan_with_arrival(')
edit(a,'                let decision = decision.map_err(|e| e.to_string())?;','''                let decision = decision.map_err(|e| e.to_string())?;
                if capture {
                    let events = xt_stcar_robot_core::navigation::capture_end();
                    let data = serde_json::json!({"at":at,"source":pose,"scan":scan,"road":road_frame,"projection":projection,"hint_dt_s":hint_dt_s,
                        "mission":mission,"online":online_report,"travel_boundary":travel_boundary,"before_boundary":before_boundary,"before_plan":before_plan,
                        "estimate":navigation_pose,"obstacles":obstacles,"obstacles_at":scan.captured_at.min(road.captured_at),"goal":point,"goal_heading_rad":heading,"speed_limit_mps":max_speed_mps,"arrival":arrival,
                        "decision":decision,"after_plan":self.navigation,"events":events});
                    std::fs::write(format!("../capture-{at:?}.json"), serde_json::to_vec_pretty(&data).unwrap()).unwrap();
                }''')
