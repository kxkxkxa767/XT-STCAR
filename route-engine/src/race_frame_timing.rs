//! Per-frame measurements for locked checks. All workers use Linux CLOCK_MONOTONIC.
use std::fs::OpenOptions;
use std::io::{LineWriter, Write};
use std::path::PathBuf;

pub(crate) const DIRECTORY: &str = "XT_STCAR_FRAME_TIMING_DIR";

pub(crate) struct FrameLog(LineWriter<std::fs::File>);

impl FrameLog {
    pub(crate) fn open(name: &str) -> Option<Self> {
        let directory = PathBuf::from(std::env::var_os(DIRECTORY)?);
        match OpenOptions::new()
            .create(true)
            .append(true)
            .open(directory.join(name))
        {
            Ok(file) => Some(Self(LineWriter::new(file))),
            Err(error) => {
                eprintln!("frame timing {name}: {error}");
                None
            }
        }
    }

    pub(crate) fn record(&mut self, row: serde_json::Value) {
        let _ = writeln!(self.0, "{row}");
    }
}

pub(crate) fn monotonic_s() -> f64 {
    let t = rustix::time::clock_gettime(rustix::time::ClockId::Monotonic);
    t.tv_sec as f64 + t.tv_nsec as f64 * 1e-9
}
