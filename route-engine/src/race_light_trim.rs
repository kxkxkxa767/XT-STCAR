//! Deliberate inch-back *after* the compulsory stop. Positive travel here means
//! backwards. Both a physical wall displacement and a currently registered map pose
//! must describe it. Never derive reverse distance from a clamped forward speed.
use crate::helpers::{NEUTRAL, SCAN_AGE_S};
use crate::py::V;

pub const TRIM_PWM: i64=1480;
const PACE: f64=0.08; // requested reverse m/s, regulated by signed range changes
const GOOD_LIP: f64=0.25; // lower edge / image height: seven good stopped images .254..327

#[derive(Clone,Copy,Debug)]
pub struct Fix {pub at:f64,pub wall:f64,pub mapped:f64,pub front:f64,pub rear:f64,pub rear_free:f64}

#[derive(Clone,Copy,Default,Debug,PartialEq,Eq)]
enum Phase {#[default] Idle, Armed, Back, Settle, Done}

#[derive(Clone,Default,Debug)]
pub struct Trim {
    phase:Phase, anchor:Option<(Fix,f64)>, previous:Option<Fix>, view_at:Option<f64>, lip:Option<f64>,
    saw_panel:bool, clipped:bool, started:f64, still:Option<(f64,f64,f64)>, pub signed_forward:f64,
    pub distance:f64,pub rear_room:Option<f64>,pub issued:bool, pub brake_acked:bool, last_low:Option<f64>,until:Option<f64>,pub reason:&'static str,
    edge_origin:bool, pub before_stop:bool,
}
impl Trim {
    pub fn view(&mut self,at:f64,lip:Option<f64>) {
        if self.view_at.is_some_and(|t|at<=t){return}
        self.view_at=Some(at);self.lip=lip;
        if let Some(y)=lip {self.saw_panel=true; self.clipped=y<GOOD_LIP;}
    }
    pub fn arm(&mut self,fix:Fix,plane:f64){self.anchor=Some((fix,plane));self.previous=Some(fix);self.phase=Phase::Armed;}
    pub fn arm_tip(&mut self,fix:Fix,plane:f64,recover:bool){self.arm(fix,plane);self.edge_origin=true;self.before_stop=recover;}
    pub fn cropped(&self)->bool {self.saw_panel && self.clipped}
    pub fn active(&self)->bool{matches!(self.phase,Phase::Back|Phase::Settle)}
    pub fn needs_scan(&self)->bool{self.active() || self.phase==Phase::Armed && self.saw_panel}
    pub fn authorized(&self,now:f64)->bool{self.phase==Phase::Back && self.until.is_some_and(|t|now<t)}
    pub fn late_ack(&self,now:f64)->bool{self.last_low.is_some_and(|at|(0. ..=SCAN_AGE_S).contains(&(now-at)))}
    pub fn release(&mut self){self.until=None;self.phase=Phase::Done;}
    fn settle(&mut self,why:&'static str){self.phase=Phase::Settle;self.until=None;self.reason=why;}
    // Loss of the second position source cancels the PWM, not the original clock.
    // The same physical board anchored at the certified stop can still prove that
    // coasting has ended inside the zone. This path can never issue reverse.
    pub fn wall_settle(&mut self, at:f64, wall:Option<(f64,f64,f64)>,ack:Option<i64>){
        if !self.active(){return}
        self.settle("missing_current_geometry");
        let Some(((w,front,rear),(o,plane)))=wall.zip(self.anchor) else{self.still=None;return};
        let d=w-o.wall;let base=if self.edge_origin {plane} else {o.mapped};
        let in_zone=plane.min(base)+d-front>0. && 1.-plane.max(base)-d-rear>0.06;
        let fresh=self.previous.is_some_and(|f|at>=f.at && (w-f.wall).abs()<=0.15+(at-f.at).min(SCAN_AGE_S)*PACE);
        if !in_zone || !fresh || ack!=Some(NEUTRAL){self.still=None;return}
        let s=self.still.get_or_insert((at,w,w));
        if (w-s.1).abs()>0.012 {*s=(at,w,w)}
        if at-s.0>=crate::coast::MIN_RECEIVE_STABLE_S {self.phase=Phase::Done;self.signed_forward=0.;}
    }
    pub fn update(&mut self,now:f64,fix:Option<Fix>,ack:Option<i64>,due:bool)->i64 {
        let Some((origin,plane))=self.anchor else{return NEUTRAL};
        if !self.needs_scan(){return NEUTRAL}
        let fresh_view=self.view_at.is_some_and(|t|(0. ..=crate::maneuver::LIGHT_FRAME_MAX_AGE_S).contains(&(now-t)));
        let Some(f)=fix.filter(|f|(0. ..SCAN_AGE_S).contains(&(now-f.at))) else {
            if self.active(){self.settle("missing_current_geometry");self.still=None}return NEUTRAL;
        };
        let dw=f.wall-origin.wall;let dm=f.mapped-origin.mapped;
        // At a confirmed physical opening, transport both displacements from
        // that measured origin. Otherwise keep the original two absolute checks.
        let localized=if self.edge_origin {plane+dm} else {f.mapped};
        let nose=(plane+dw).max(localized)-f.front;
        let room=1.-(plane+dw).max(localized)-f.rear;
        self.distance=(dw+dm)*0.5;self.rear_room=Some(room);
        let legal=(plane+dw).min(localized)-f.front>=0. && room>0.;
        let consistent=(dw-dm).abs()<=0.06 && (self.edge_origin || (origin.mapped-plane).abs()<=0.12);
        let old=self.previous.replace(f);let dt=old.map_or(0.,|o|f.at-o.at);
        let step=old.filter(|_|dt>0. && dt<=SCAN_AGE_S).map(|o|((f.wall-o.wall)/dt,(f.mapped-o.mapped)/dt));
        if let Some((a,b))=step {self.signed_forward=-(a+b)*0.5;}
        let closing=step.map_or(0.,|(a,b)| a.max(b).max(0.));
        // Preserve rear clearance through the acquisition, command and coast tails;
        // use the smaller of the rule-boundary and actual rear obstacle budgets.
        let v=closing.max(PACE);let tail=v*(SCAN_AGE_S+crate::motion_sensors::ESC_COAST_DELAY_S)+v*v/(2.*0.6)+0.02;
        let fits=(legal || self.before_stop) && consistent && room>0.06+tail && f.rear_free>crate::helpers::MANEUVER_BODY_CLEARANCE_M+tail;
        let visually_close=fresh_view && self.saw_panel && (self.clipped || (self.lip.is_none() && nose<0.40));
        if self.phase==Phase::Armed {
            if due || !visually_close || !fits || nose>=0.55-tail || ack!=Some(NEUTRAL){return NEUTRAL}
            self.phase=Phase::Back;self.started=now;self.issued=true;self.reason="back_to_panel_view";
        }
        if self.phase==Phase::Back {
            if due || !fits || !fresh_view || self.lip.is_some_and(|y|y>=GOOD_LIP) || nose>=0.55-tail {
                self.settle(if due {"time_limit"}else if !fits {"rear_or_reference_budget"}else if !fresh_view {"camera_stale"}else{"target_view_or_position"});
            } else if now-self.started>=crate::coast::MIN_RECEIVE_STABLE_S && dw.abs().max(dm.abs())<0.012 {
                self.settle("no_measured_reverse_response");
            } else if ack.is_some_and(|p|p!=NEUTRAL && p!=TRIM_PWM) || (dt>0. && step.is_none()) {
                self.settle("lost_motion_ack");
            } else if ack.is_none() { self.until=None; }
            else if closing<=PACE && (dt>0. || now==self.started) {
                // -4: 19/39 ms at 1480 was shorter than the ESC's response;
                // the physical ranges never moved. Renew the low gear through
                // the response only while well below the measured creep pace.
                self.last_low=Some(now);self.until=Some(f.at+if closing<0.5*PACE {crate::race_light_brake::LIGHT_BRAKE_RESPONSE_S}else{0.019});return TRIM_PWM;
            } else if dt==0. && self.authorized(now){return TRIM_PWM}
            else {self.until=None;}
        }
        if self.phase==Phase::Settle {
            if legal && ack==Some(NEUTRAL) {
                let s=self.still.get_or_insert((f.at,f.wall,f.mapped));
                if dt>SCAN_AGE_S || (f.wall-s.1).abs().max((f.mapped-s.2).abs())>0.012 {*s=(f.at,f.wall,f.mapped)}
                if f.at-s.0>=crate::coast::MIN_RECEIVE_STABLE_S {self.phase=Phase::Done;self.signed_forward=0.;}
            }else{self.still=None}
        }
        NEUTRAL
    }
    pub fn diagnostics(&self)->V {crate::dict!{"phase"=>match self.phase{Phase::Idle=>"idle",Phase::Armed=>"armed",Phase::Back=>"back",Phase::Settle=>"settle",Phase::Done=>"done"},
        "reason"=>self.reason,"issued"=>self.issued,"before_legal_stop"=>self.before_stop,"housing_lip_y"=>self.lip,"frame_at"=>self.view_at,
        "rear_past_beam_m"=>self.rear_room,"signed_forward_mps"=>self.signed_forward,"reverse_displacement_m"=>self.distance}}
}
