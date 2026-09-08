//! Bounded, memory-only JSONL journaling for offline runs.
//!
//! Records are serialized in memory; no files are opened and no per-tick flush occurs.
//! The caller selects phase/fault events, appends its terminal result after `write_to`,
//! and decides when to write the completed run. Logging errors are diagnostics, never
//! a reason to alter control commands. Optional trace pressure only drops trace data.
use serde::{Deserialize, Serialize};
use std::io::{self, Write};

pub const DEFAULT_TRACE_BYTES: usize = 1024 * 1024;
pub const MAX_TRACE_BYTES: usize = 8 * 1024 * 1024;
pub const MAX_IMPORTANT_BYTES: usize = 256 * 1024;
pub const MAX_IMPORTANT_RECORDS: usize = 1024;

#[derive(Clone, Copy, Debug, Default, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum TelemetryMode {
    /// The caller selects only faults and essential results as important.
    Summary,
    /// The caller also selects mission phase transitions as important.
    #[default]
    Transitions,
    /// Also retain ordinary diagnostic records up to the trace byte budget.
    Trace,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct TelemetryConfig {
    pub mode: TelemetryMode,
    pub max_trace_bytes: usize,
}

impl Default for TelemetryConfig {
    fn default() -> Self {
        Self {
            mode: TelemetryMode::Transitions,
            max_trace_bytes: DEFAULT_TRACE_BYTES,
        }
    }
}

impl TelemetryConfig {
    pub fn validate(&self) -> Result<(), String> {
        if !(1..=MAX_TRACE_BYTES).contains(&self.max_trace_bytes) {
            return Err("max_trace_bytes must be between 1 byte and 8 MiB".into());
        }
        Ok(())
    }
}

pub struct RunJournal {
    config: TelemetryConfig,
    bytes: Vec<u8>,
    trace_bytes: usize,
    important_bytes: usize,
    important_records: usize,
    dropped_records: u64,
}

impl RunJournal {
    pub fn new(config: TelemetryConfig) -> Result<Self, String> {
        config.validate()?;
        // One contiguous allocation avoids unbounded per-record metadata overhead.
        let capacity = MAX_IMPORTANT_BYTES
            + if config.mode == TelemetryMode::Trace {
                config.max_trace_bytes
            } else {
                0
            };
        Ok(Self {
            config,
            bytes: Vec::with_capacity(capacity),
            trace_bytes: 0,
            important_bytes: 0,
            important_records: 0,
            dropped_records: 0,
        })
    }

    pub fn config(&self) -> &TelemetryConfig {
        &self.config
    }

    /// Append one complete JSONL record without disk access.
    ///
    /// `important` bypasses trace filtering and has a separate finite reserve. Such
    /// records are never silently dropped or evicted: exhaustion/serialization
    /// failure returns an error and preserves every earlier record. The caller must
    /// report that logging error separately, without turning it into a control fault.
    /// Summary versus Transitions selection belongs to the caller; both preserve all
    /// records explicitly marked important. Filtered ordinary records return false
    /// without incrementing `dropped_records`; only trace budget drops are counted.
    pub fn record<T: Serialize + ?Sized>(
        &mut self,
        record: &T,
        important: bool,
    ) -> Result<bool, String> {
        if !important && self.config.mode != TelemetryMode::Trace {
            return Ok(false);
        }
        if important && self.important_records == MAX_IMPORTANT_RECORDS {
            return Err("important telemetry record reserve exhausted".into());
        }
        let remaining = if important {
            MAX_IMPORTANT_BYTES - self.important_bytes
        } else {
            self.config.max_trace_bytes - self.trace_bytes
        };
        let previous_len = self.bytes.len();
        // Include the newline in the budget. Serialize through a limited writer so
        // an oversized value cannot allocate an unbounded temporary JSON buffer.
        let mut limited = LimitedWriter {
            bytes: &mut self.bytes,
            remaining: remaining.saturating_sub(1),
            exceeded: remaining == 0,
        };
        let result = serde_json::to_writer(&mut limited, record);
        let exceeded = limited.exceeded;
        if let Err(error) = result {
            self.bytes.truncate(previous_len);
            if exceeded {
                if important {
                    return Err("important telemetry byte reserve exhausted".into());
                }
                self.dropped_records = self.dropped_records.saturating_add(1);
                return Ok(false);
            }
            return Err(format!("telemetry serialization failed: {error}"));
        }
        if exceeded || self.bytes.len() == previous_len {
            self.bytes.truncate(previous_len);
            if important {
                return Err("important telemetry record is empty or exceeds its reserve".into());
            }
            self.dropped_records = self.dropped_records.saturating_add(1);
            return Ok(false);
        }
        self.bytes.push(b'\n');
        let added = self.bytes.len() - previous_len;
        if important {
            self.important_bytes += added;
            self.important_records += 1;
        } else {
            self.trace_bytes += added;
        }
        Ok(true)
    }

    pub fn dropped_records(&self) -> u64 {
        self.dropped_records
    }

    pub fn buffered_bytes(&self) -> usize {
        self.bytes.len()
    }

    /// Batch output at the caller's chosen end-of-run boundary. Does not flush or
    /// fsync and does not consume the buffer, so a failed destination can be retried.
    /// A `Write` implementation may require more than one underlying short write.
    pub fn write_to(&self, writer: &mut impl Write) -> io::Result<()> {
        writer.write_all(&self.bytes)
    }

    /// Empty the journal and counters for reuse, retaining its fixed allocation.
    pub fn clear(&mut self) {
        self.bytes.clear();
        self.trace_bytes = 0;
        self.important_bytes = 0;
        self.important_records = 0;
        self.dropped_records = 0;
    }
}

struct LimitedWriter<'a> {
    bytes: &'a mut Vec<u8>,
    remaining: usize,
    exceeded: bool,
}

impl Write for LimitedWriter<'_> {
    fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
        if self.exceeded || bytes.len() > self.remaining {
            self.exceeded = true;
            return Err(io::Error::other("telemetry byte budget exhausted"));
        }
        self.bytes.extend_from_slice(bytes);
        self.remaining -= bytes.len();
        Ok(bytes.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}
