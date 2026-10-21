#![forbid(unsafe_code)]
//! Route control engine: one ManeuverSequence per console session, driven
//! over stdin/stdout JSON lines by web/vehicle-console/route_engine.py.
//!
//! Request:  {"op": "new" | "call" | "get" | "dump", ...}
//! Response: {"ok": true, "r": result, "d": decision delta, "x": removed
//!            decision keys, "s": changed mirrored attributes}
//!           or {"ok": false, "exc": "ValueError", "msg": "...", "s": ...}
//! A scan argument is {"$base": scan, "$with": extra} (the scan plus extra
//! keys, e.g. received_at) or {"$same": 1, "$with": extra} (the previous
//! base again, equal content). Every request names its session ("sid");
//! {"op": "drop"} ends one, and "drops": [sid, ...] ends others first.
//! A known-body stop of `clearance_probe` answers {"ok": false, "exc":
//! "ProbeClearanceError", "msg": ..., "clearance": ...}.

pub mod clearance;
pub mod compact_target;
pub mod dump;
pub mod helpers;
pub mod maneuver;
pub mod maneuver_open;
pub mod maneuver_path;
pub mod maneuver_post;
pub mod maneuver_tm;
pub mod maneuver_wait;
pub mod py;

use std::io::{BufRead, BufWriter, Write};

use clearance::{ClearanceReuse, Probe};
use maneuver::Maneuver;
use py::*;

/// One console session's sequence and the last values sent for it.
#[derive(Default)]
struct Session {
    motion: Option<Maneuver>,
    last_scan: V,
    last_decision: V,
    last_mirror: V,
    clearance: ClearanceReuse,
}

#[derive(Default)]
struct Engine {
    sessions: Vec<(i64, Session)>,
}

/// Top-level keys of `new` that differ from `old` (same values and types),
/// plus the keys `old` has and `new` lacks.
fn delta(old: &V, new: &V) -> (V, Vec<V>) {
    let mut changed = V::dict();
    for (k, v) in new.items() {
        if !(old.has(k) && same(old.get(k), v)) {
            changed.set(k, v.clone());
        }
    }
    let removed = if old.is_dict() {
        old.items().iter().filter(|(k, _)| !new.has(k)).map(|(k, _)| V::from(k.clone())).collect()
    } else {
        vec![]
    };
    (changed, removed)
}

impl Engine {
    fn session(&mut self, sid: i64) -> &mut Session {
        if !self.sessions.iter().any(|(k, _)| *k == sid) {
            self.sessions.push((sid, Session::default()));
        }
        &mut self.sessions.iter_mut().find(|(k, _)| *k == sid).unwrap().1
    }

    fn handle(&mut self, req: &V) -> V {
        let sid = if req.get("sid").is_int() { req.get("sid").i() } else { 0 };
        if req.get("drops").is_list() {
            let drops: Vec<i64> = req.get("drops").l().iter().filter(|v| v.is_int()).map(|v| v.i()).collect();
            self.sessions.retain(|(k, _)| !drops.contains(k));
        }
        if req.get("op").os() == Some("drop") {
            self.sessions.retain(|(k, _)| *k != sid);
            return crate::dict! {"ok" => true, "r" => V::None};
        }
        if req.get("op").os() == Some("fn") {
            return crate::dict! {"ok" => true, "r" => function(req.idx("name").s(), req.idx("args").l())};
        }
        self.session(sid).handle(req)
    }

    fn mirror_delta(&mut self, req: &V) -> V {
        let sid = if req.get("sid").is_int() { req.get("sid").i() } else { 0 };
        match self.sessions.iter_mut().find(|(k, _)| *k == sid) {
            Some((_, s)) => s.mirror_delta(),
            None => V::dict(),
        }
    }
}

impl Session {
    fn motion(&mut self) -> &mut Maneuver {
        self.motion.as_mut().unwrap_or_else(|| raise(Exc::ValueError, "route_engine_no_session"))
    }

    fn scan_arg(&mut self, value: &V) -> V {
        if !value.is_dict() {
            return value.clone();
        }
        if value.has("$base") {
            self.last_scan = value.get("$base").clone();
        } else if !value.get("$same").truthy() {
            return value.clone();
        }
        let extra = value.get("$with");
        if extra.is_dict() && !extra.items().is_empty() { self.last_scan.merged(extra) } else { self.last_scan.clone() }
    }

    fn mirror_delta(&mut self) -> V {
        let Some(motion) = &self.motion else { return V::dict() };
        let mirror = motion.mirrored();
        let (changed, _) = delta(&self.last_mirror, &mirror);
        self.last_mirror = mirror;
        changed
    }

    fn handle(&mut self, req: &V) -> V {
        let op = req.get("op").os().unwrap_or("").to_string();
        match op.as_str() {
            "new" => {
                let motion = Maneuver::new(
                    req.get("started_at"),
                    req.get("max_drive_s"),
                    req.get("initial_presteer_pwm"),
                    req.get("continue_route"),
                );
                self.motion = Some(motion);
                self.last_scan = V::None;
                self.last_decision = V::dict();
                self.last_mirror = V::dict();
                self.clearance = ClearanceReuse::default();
                crate::dict! {"ok" => true, "r" => V::None}
            }
            "call" => self.call(req),
            "get" => {
                let dump = self.motion().dump();
                let mut out = V::dict();
                for name in req.idx("attrs").l() {
                    let n = name.s();
                    if dump.has(n) {
                        out.set(n, dump.get(n).clone());
                    }
                }
                crate::dict! {"ok" => true, "r" => out}
            }
            "dump" => {
                let dump = self.motion().dump();
                crate::dict! {"ok" => true, "r" => dump}
            }
            other => raise(Exc::ValueError, format!("route_engine_unknown_op_{other}")),
        }
    }

    fn call(&mut self, req: &V) -> V {
        let method = req.idx("m").s().to_string();
        let a = req.get("a").clone();
        let arg = |k: &str| -> V { a.get(k).clone() };
        let mut decision = V::None;
        let result: V = match method.as_str() {
            "update" => {
                let scan = self.scan_arg(&arg("scan"));
                let motion = self.motion();
                let d = motion.update(
                    &scan,
                    &arg("lidar_age_s"),
                    &arg("now"),
                    &arg("control"),
                    &arg("safe"),
                    arg("entry_stop").is_some(),
                    &arg("presteer_wait"),
                    &arg("quality_clear"),
                    &arg("quality_resume_ready"),
                );
                decision = d;
                V::None
            }
            "begin_quality_wait" | "begin_left_turn_quality_pause" | "begin_post_pass_quality_pause" => {
                let scan_v = arg("scan");
                let scan = if scan_v.is_none() { None } else { Some(self.scan_arg(&scan_v)) };
                let (now, control) = (arg("now"), arg("control"));
                let motion = self.motion();
                let ok = match method.as_str() {
                    "begin_quality_wait" => motion.begin_quality_wait(&now, &control, scan.as_ref()),
                    "begin_left_turn_quality_pause" => motion.begin_left_turn_quality_pause(&now, &control, scan.as_ref()),
                    _ => motion.begin_post_pass_quality_pause(&now, &control, scan.as_ref()),
                };
                V::Bool(ok)
            }
            "begin_route_quality_coast" => {
                let (now, control) = (arg("now"), arg("control"));
                V::Bool(self.motion().begin_route_quality_coast(&now, &control))
            }
            "begin_quality_coast" => {
                let (now, control) = (arg("now"), arg("control"));
                V::Bool(self.motion().begin_quality_coast(&now, &control))
            }
            "begin_coast" => {
                let now = arg("now");
                let now = if now.is_num() { now.f() } else { f64::NAN };
                let reason = arg("reason").s().to_string();
                self.motion().begin_coast(&reason, now);
                V::None
            }
            "current_contact_target" => {
                let scan = self.scan_arg(&arg("scan"));
                let now = arg("now");
                self.motion().current_contact_target(&scan, &now)
            }
            "clearance_probe" => {
                // server.autonomy_tick's body-clearance check: the current
                // contact target (route sessions) as its witness, then
                // ClearanceReuse(probe_clearance) as the console ran it.
                let scan = self.scan_arg(&arg("scan"));
                let now = arg("now");
                let contact = if arg("contact").is_true() {
                    self.motion().current_contact_target(&scan, &now)
                } else {
                    V::None
                };
                let mut kw = arg("kw");
                kw.set("body_contact_target", contact.clone());
                let reuse = &mut self.clearance;
                match reuse.probe(&scan, &arg("ages"), &arg("demo"), &arg("centering"), &kw) {
                    Ok(Probe::Reused) => crate::dict! {
                        "contact" => contact, "reused" => true, "token" => reuse.token, "count" => reuse.reused,
                    },
                    Ok(Probe::Fresh { clearance, stored }) => crate::dict! {
                        "contact" => contact, "reused" => false,
                        "token" => if stored { V::Int(reuse.token) } else { V::None },
                        "count" => reuse.reused, "clearance" => clearance,
                    },
                    Err(clearance) => {
                        return crate::dict! {
                            "ok" => false, "exc" => "ProbeClearanceError", "msg" => "probe_obstacle_close_body",
                            "clearance" => clearance,
                        };
                    }
                }
            }
            "start_lane_quality_geometry" => {
                let scan = self.scan_arg(&arg("scan"));
                V::Bool(self.motion().start_lane_quality_geometry(&scan))
            }
            "lane_wall_output_active" => V::Bool(self.motion().lane_wall_output_active()),
            "probe_measurement" => {
                // server.turn_route_quality_scope: _measurement on an isolated
                // copy, so the check cannot refresh a live lease or history.
                let scan = self.scan_arg(&arg("scan"));
                let (candidates, dt) = (arg("candidates"), arg("dt"));
                let mut probe = self.motion().clone();
                let ok = except(VTK, || probe.measurement(&scan, &candidates, dt.f()));
                match ok {
                    Ok(_) => V::Bool(true),
                    Err(e) => raise(e.kind, e.msg),
                }
            }
            other => raise(Exc::ValueError, format!("route_engine_unknown_method_{other}")),
        };
        let mut response = crate::dict! {"ok" => true, "r" => result};
        if decision.is_dict() {
            let (changed, removed) = delta(&self.last_decision, &decision);
            self.last_decision = decision;
            response.set("d", changed);
            response.set("x", V::list(removed));
        }
        response
    }
}

/// Module-level functions, for checking one computation against Python.
fn function(name: &str, args: &[V]) -> V {
    match name {
        "hypot" => V::Float(hypot(args[0].f(), args[1].f())),
        "directions" => V::list(compact_target::directions().into_iter().map(V::from).collect()),
        "scan_points" => V::list(
            compact_target::scan_points(&compact_target::floats(&args[0])).into_iter().map(|p| p.map(V::from).unwrap_or(V::None)).collect(),
        ),
        "contact_supports" => V::list(compact_target::contact_supports(&args[0])),
        "probe_clearance" => match clearance::probe_clearance(&args[0], &args[1], &args[2], &args[3], &args[4]) {
            Ok(result) => crate::dict! {"clearance" => result},
            Err(result) => crate::dict! {"body_stop" => result},
        },
        _ => raise(Exc::ValueError, format!("route_engine_unknown_function_{name}")),
    }
}

fn main() {
    install_panic_hook();
    let stdin = std::io::stdin();
    let stdout = std::io::stdout();
    let mut out = BufWriter::with_capacity(1 << 16, stdout.lock());
    let mut engine = Engine::default();
    let mut line = String::new();
    let mut input = stdin.lock();
    loop {
        line.clear();
        match input.read_line(&mut line) {
            Ok(0) | Err(_) => break,
            Ok(_) => {}
        }
        if line.trim().is_empty() {
            continue;
        }
        let (mut response, req) = match parse_json(line.trim_end()) {
            Err(e) => (crate::dict! {"ok" => false, "exc" => "ProtocolError", "msg" => e}, V::None),
            Ok(req) => match py_try(|| engine.handle(&req)) {
                Ok(r) => (r, req),
                Err(e) => (crate::dict! {"ok" => false, "exc" => e.kind.name(), "msg" => e.msg}, req),
            },
        };
        if req.is_dict() && req.get("op").os() != Some("drop") {
            let mirror = engine.mirror_delta(&req);
            response.set("s", mirror);
        }
        let mut text = to_json(&response);
        text.push('\n');
        if out.write_all(text.as_bytes()).is_err() || out.flush().is_err() {
            break;
        }
    }
}
