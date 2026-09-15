use xt_stcar_robot_core::tracking::TrackingConfig;
use xt_stcar_robot_core::{MotionOutput, Timestamp};
use xt_stcar_robot_runner::async_simulation::{AsyncSimulationOptions, simulate_async_observed};
use xt_stcar_robot_runner::simulation::SimulationConfig;
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut config=SimulationConfig::example();
    config.autonomy.navigation.tracking=TrackingConfig::Lqr {q_lateral:4.0,q_heading:2.0,r_curvature:1.0,min_speed_mps:0.03,max_heading_error_rad:0.7,max_lateral_error_m:0.5};
    let timing=AsyncSimulationOptions {input_delays_ms:[40,60],adoption_delays_ms:[3,7,9],..Default::default()};
    let mut commands=Vec::new(); let mut plans=Vec::new();
    let mut last_command=None; let mut last_plan=None; let mut last_at=Timestamp(0); let mut stop_ms=0u64; let mut omitted=0u64;
    let summary=simulate_async_observed(&config,&timing,&mut std::io::sink(),|poll,pose,speed,curvature| {
        if last_command==Some(MotionOutput::Stop) {stop_ms+=poll.at.0.saturating_sub(last_at.0);}
        last_at=poll.at;
        if last_command.as_ref()!=Some(&poll.command) {
            last_command=Some(poll.command.clone());
            if commands.len()<16384 {commands.push(serde_json::json!({"at":poll.at,"command":poll.command,"pose":pose,"speed_mps":speed,"curvature_per_m":curvature}));} else {omitted+=1;}
        }
        if let Some(plan)=&poll.observed_plan && last_plan!=Some(plan.planned_at) {
            last_plan=Some(plan.planned_at);
            if plans.len()<2048 {plans.push(serde_json::json!({"source_at":plan.source_at,"planned_at":plan.planned_at,"observed_at":poll.at,"actual_pose":pose,"actual_speed_mps":speed,"actual_curvature_per_m":curvature,"output_command":poll.command,"adoption_rejection":poll.adoption_rejection,"report":plan.report.as_ref()}));} else {omitted+=1;}
        }
    })?;
    serde_json::to_writer(std::io::stdout().lock(),&serde_json::json!({"schema_version":1,"physical_output_enabled":false,"config":config,"timing":timing,"trace_limits":{"commands":16384,"plans":2048},"omitted":omitted,"stop_duration_ms":stop_ms,"commands":commands,"plans":plans,"summary":summary}))?;
    println!();
    if !summary.completed || omitted!=0 {return Err("incomplete or truncated ablation run".into());}
    Ok(())
}
