//! Console bridge. Default dry run; explicit --execute opens a selected device.
use std::{
    io::{self, BufRead, Read, Write},
    path::PathBuf,
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
        mpsc,
    },
    thread,
    time::{Duration, Instant},
};
use xt_stcar_device_io::SerialPort;
use xt_stcar_robot_core::{
    FrameId, Timestamp,
    protocol::{
        chassis::PwmCommand,
        n10::{N10Config, N10Decoder},
    },
    teleop::{Guard, Request},
};
fn output() -> mpsc::SyncSender<serde_json::Value> {
    let (tx, rx) = mpsc::sync_channel::<serde_json::Value>(1);
    thread::spawn(move || {
        let mut out = io::stdout().lock();
        for row in rx {
            if serde_json::to_writer(&mut out, &row).is_err()
                || writeln!(out).is_err()
                || out.flush().is_err()
            {
                break;
            }
        }
    });
    tx
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let mode = args.next().ok_or("control or lidar required")?;
    let mut device = None;
    let mut execute = false;
    let mut reverse = false;
    while let Some(a) = args.next() {
        match a.as_str() {
            "--device" => {
                device = Some(PathBuf::from(args.next().ok_or("missing device")?));
            }
            "--execute" => execute = true,
            "--allow-reverse" => reverse = true,
            _ => return Err("unknown argument".into()),
        }
    }
    if !matches!(mode.as_str(), "control" | "lidar") {
        return Err("unknown mode".into());
    }
    if execute && device.is_none() {
        return Err("--execute requires --device".into());
    }
    let mut port = if execute {
        Some(SerialPort::open(
            &device.unwrap(),
            if mode == "control" { 38400 } else { 230400 },
        )?)
    } else {
        None
    };
    let out = output();
    let start = Instant::now();
    if mode == "lidar" {
        let mut decoder = N10Decoder::new(N10Config {
            frame_id: FrameId("laser_link".into()),
            range_min_m: 0.02,
            range_max_m: 12.0,
            max_packet_age_ms: 50,
        })?;
        let mut bins = vec![None::<f64>; 360];
        let mut seen = vec![false; 360];
        let mut prev = 0;
        let mut synced = false;
        let mut seq = 0u64;
        loop {
            if let Some(p) = port.as_mut() {
                let mut buf = [0u8; 4096];
                if let Some(n) =
                    p.read_until(&mut buf, Instant::now() + Duration::from_millis(20))?
                {
                    for packet in decoder
                        .feed(&buf[..n], Timestamp(start.elapsed().as_millis() as u64))?
                        .packets
                    {
                        if prev > packet.start_angle_cdeg.saturating_add(18000) {
                            if synced {
                                seq += 1;
                                let valid = bins.iter().filter(|r| r.is_some()).count();
                                let coverage = seen.iter().filter(|v| **v).count();
                                let row = serde_json::json!({"seq":seq,"at_ms":start.elapsed().as_millis(),"ranges":bins,"valid_fraction":valid as f64/360.0,"coverage":coverage as f64/360.0,"navigation_validated":false});
                                if matches!(
                                    out.try_send(row),
                                    Err(mpsc::TrySendError::Disconnected(_))
                                ) {
                                    return Ok(());
                                }
                            }
                            bins.fill(None);
                            seen.fill(false);
                            synced = true;
                        }
                        prev = packet.start_angle_cdeg;
                        let span = (i32::from(packet.end_angle_cdeg)
                            - i32::from(packet.start_angle_cdeg))
                        .rem_euclid(36000) as f64;
                        for (i, p) in packet.points.iter().enumerate() {
                            let angle = (f64::from(packet.start_angle_cdeg)
                                + span * i as f64 / 15.0)
                                / 100.0;
                            let bin = (angle.round() as usize) % 360;
                            seen[bin] = true;
                            if let Some(r) = p.range_m {
                                bins[bin] = Some(bins[bin].map_or(r, |old| old.min(r)));
                            }
                        }
                    }
                }
            } else {
                seq += 1;
                let ranges: Vec<Option<f64>> = (0..360)
                    .map(|i| Some(3.0 + (i as f64 / 30.0).sin() * 0.3))
                    .collect();
                if out.try_send(serde_json::json!({"seq":seq,"ranges":ranges,"valid_fraction":1.0,"coverage":1.0,"navigation_validated":false})).is_err(){thread::sleep(Duration::from_millis(100));}
                thread::sleep(Duration::from_millis(100));
            }
        }
    }
    let (tx, rx) = mpsc::sync_channel::<Request>(4);
    let failed = Arc::new(AtomicBool::new(false));
    let reader_failed = failed.clone();
    thread::spawn(move || {
        let mut input = io::stdin().lock();
        loop {
            let mut line = Vec::new();
            match input.by_ref().take(1025).read_until(b'\n', &mut line) {
                Ok(0) | Err(_) => break,
                Ok(_) if line.len() > 1024 => break,
                _ => {}
            }
            match serde_json::from_slice(&line) {
                Ok(req) => {
                    if tx.try_send(req).is_err() {
                        break;
                    }
                }
                Err(_) => break,
            }
        }
        reader_failed.store(true, Ordering::SeqCst);
    });
    let mut guard = Guard::new(reverse);
    let mut problem = None;
    loop {
        let now = start.elapsed().as_millis() as u64;
        guard.advance(now);
        if failed.load(Ordering::SeqCst) {
            guard.stop("input_closed");
            break;
        }
        // Bounded drain, each request independently checked against the bridge clock.
        for _ in 0..4 {
            if let Ok(req) = rx.try_recv() {
                guard.apply(req, now);
            } else {
                break;
            }
        }
        if let Some(p) = port.as_mut()
            && let Err(e) = p.write_packet_until(
                &PwmCommand::new(guard.status.motor, guard.status.servo)?.encode(),
                Instant::now() + Duration::from_millis(15),
            )
        {
            problem = Some(e);
            break;
        }
        let row = serde_json::json!({"control":guard.status,"physical_output":execute,"reverse_enabled":reverse});
        if out.try_send(row).is_err() {
            guard.stop("status_backpressure");
        }
        thread::sleep(Duration::from_millis(20));
    }
    if let Some(p) = port.as_mut() {
        for _ in 0..20 {
            let _ = p.write_packet_until(
                &PwmCommand::new(1500, 1500)?.encode(),
                Instant::now() + Duration::from_millis(15),
            );
            thread::sleep(Duration::from_millis(20));
        }
    }
    if let Some(e) = problem {
        return Err(e.into());
    }
    Ok(())
}
