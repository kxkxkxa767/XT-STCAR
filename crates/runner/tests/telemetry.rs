use serde::ser::{Error, SerializeSeq};
use serde::{Serialize, Serializer};
use serde_json::{Value, json};
use std::io::{self, Write};
use xt_stcar_robot_runner::telemetry::{
    DEFAULT_TRACE_BYTES, MAX_IMPORTANT_BYTES, MAX_IMPORTANT_RECORDS, MAX_TRACE_BYTES, RunJournal,
    TelemetryConfig, TelemetryMode,
};

fn journal(mode: TelemetryMode, max_trace_bytes: usize) -> RunJournal {
    RunJournal::new(TelemetryConfig {
        mode,
        max_trace_bytes,
    })
    .unwrap()
}

fn records(journal: &RunJournal) -> Vec<Value> {
    let mut bytes = Vec::new();
    journal.write_to(&mut bytes).unwrap();
    String::from_utf8(bytes)
        .unwrap()
        .lines()
        .map(|line| serde_json::from_str(line).unwrap())
        .collect()
}

#[test]
fn trace_budget_drops_only_optional_records_and_keeps_important_order() {
    let trace = json!({"kind":"trace","speed":0.3});
    let budget = serde_json::to_vec(&trace).unwrap().len() + 1;
    let mut log = journal(TelemetryMode::Trace, budget);
    assert!(
        log.record(&json!({"kind":"phase","phase":"approach"}), true)
            .unwrap()
    );
    assert!(log.record(&trace, false).unwrap());
    assert!(!log.record(&trace, false).unwrap());
    assert!(
        log.record(&json!({"kind":"fault","reason":"expired laser"}), true)
            .unwrap()
    );
    assert!(!log.record(&trace, false).unwrap());
    assert_eq!(log.dropped_records(), 2);
    let output = records(&log);
    assert_eq!(output.len(), 3);
    assert_eq!(output[0]["kind"], "phase");
    assert_eq!(output[1], trace);
    assert_eq!(output[2]["kind"], "fault");
    assert!(log.buffered_bytes() <= budget + MAX_IMPORTANT_BYTES);
}

#[test]
fn default_and_summary_filter_optional_records_without_hiding_important_ones() {
    assert_eq!(TelemetryConfig::default().mode, TelemetryMode::Transitions);
    assert_eq!(
        TelemetryConfig::default().max_trace_bytes,
        DEFAULT_TRACE_BYTES
    );
    for mode in [TelemetryMode::Summary, TelemetryMode::Transitions] {
        let mut log = journal(mode, 1);
        assert!(!log.record(&json!({"debug":"not selected"}), false).unwrap());
        assert!(log.record(&json!({"fault":"pose expired"}), true).unwrap());
        assert_eq!(log.dropped_records(), 0);
        assert_eq!(records(&log), vec![json!({"fault":"pose expired"})]);
    }
}

/// Streaming this value would produce enormous JSON if the journal did not bound
/// serialization itself, rather than inspecting a fully allocated output string.
struct HugeStream;
impl Serialize for HugeStream {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let mut seq = serializer.serialize_seq(None)?;
        for _ in 0..1_000_000 {
            seq.serialize_element("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")?;
        }
        seq.end()
    }
}

#[test]
fn oversized_serialization_rolls_back_and_cannot_consume_important_reserve() {
    let mut log = journal(TelemetryMode::Trace, 64);
    assert!(!log.record(&HugeStream, false).unwrap());
    assert_eq!(log.buffered_bytes(), 0);
    assert_eq!(log.dropped_records(), 1);
    assert!(log.record(&json!({"phase":"wait_green"}), true).unwrap());
    let saved = log.buffered_bytes();
    assert!(
        log.record(&HugeStream, true)
            .unwrap_err()
            .contains("reserve")
    );
    assert_eq!(log.buffered_bytes(), saved);
    assert_eq!(log.dropped_records(), 1);
    assert_eq!(records(&log), vec![json!({"phase":"wait_green"})]);
    assert!(log.record(&json!({"result":"stopped"}), true).unwrap());
}

struct FailingValue;
impl Serialize for FailingValue {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let mut seq = serializer.serialize_seq(None)?;
        seq.serialize_element(&7)?;
        Err(S::Error::custom("deliberate serialization failure"))
    }
}

#[test]
fn serialization_failure_preserves_prior_records_and_is_not_a_trace_drop() {
    let mut log = journal(TelemetryMode::Trace, 128);
    log.record(&json!({"phase":1}), true).unwrap();
    let bytes = log.buffered_bytes();
    for important in [false, true] {
        assert!(
            log.record(&FailingValue, important)
                .unwrap_err()
                .contains("deliberate")
        );
        assert_eq!(log.buffered_bytes(), bytes);
    }
    assert_eq!(log.dropped_records(), 0);
    assert_eq!(records(&log), vec![json!({"phase":1})]);
}

#[derive(Default)]
struct WriterSpy {
    bytes: Vec<u8>,
    writes: usize,
    flushes: usize,
}
impl Write for WriterSpy {
    fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
        self.writes += 1;
        self.bytes.extend_from_slice(bytes);
        Ok(bytes.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        self.flushes += 1;
        Ok(())
    }
}

#[test]
fn output_is_one_batch_with_no_flush_and_clear_resets_a_run() {
    let mut log = journal(TelemetryMode::Trace, 2);
    log.record(&0, false).unwrap();
    log.record(&0, false).unwrap();
    log.record(&json!({"phase":"completed"}), true).unwrap();
    let mut sink = WriterSpy::default();
    assert_eq!(sink.writes, 0);
    log.write_to(&mut sink).unwrap();
    assert_eq!(sink.writes, 1);
    assert_eq!(sink.flushes, 0);
    assert_eq!(sink.bytes.len(), log.buffered_bytes());
    log.clear();
    assert_eq!(log.buffered_bytes(), 0);
    assert_eq!(log.dropped_records(), 0);
    assert!(log.record(&0, false).unwrap());
    assert_eq!(records(&log), vec![json!(0)]);
}

struct UnavailableWriter;
impl Write for UnavailableWriter {
    fn write(&mut self, _: &[u8]) -> io::Result<usize> {
        Err(io::Error::new(
            io::ErrorKind::BrokenPipe,
            "destination lost",
        ))
    }

    fn flush(&mut self) -> io::Result<()> {
        panic!("journaling must not flush the destination")
    }
}

#[test]
fn failed_batch_output_keeps_the_complete_run_for_a_new_destination() {
    let mut log = journal(TelemetryMode::Transitions, 1);
    let event = json!({"phase":"crosswalk_stop"});
    log.record(&event, true).unwrap();
    assert_eq!(
        log.write_to(&mut UnavailableWriter).unwrap_err().kind(),
        io::ErrorKind::BrokenPipe
    );
    assert_eq!(records(&log), vec![event]);
}

#[test]
fn important_record_count_exhaustion_is_explicit_and_never_evicts() {
    let mut log = journal(TelemetryMode::Transitions, 1);
    for value in 0..MAX_IMPORTANT_RECORDS {
        assert!(log.record(&value, true).unwrap());
    }
    let before = records(&log);
    let bytes = log.buffered_bytes();
    assert!(
        log.record(&json!({"fault":"one more"}), true)
            .unwrap_err()
            .contains("reserve")
    );
    assert_eq!(log.buffered_bytes(), bytes);
    assert_eq!(records(&log), before);
    assert_eq!(log.dropped_records(), 0);
    log.clear();
    assert!(log.record(&json!({"phase":"new run"}), true).unwrap());
}

#[test]
fn byte_reserves_include_newlines_and_remain_independent_at_the_boundary() {
    let mut log = journal(TelemetryMode::Trace, MAX_TRACE_BYTES);
    // JSON string quotes + newline occupy three additional bytes.
    assert!(log.record(&"x".repeat(MAX_TRACE_BYTES - 3), false).unwrap());
    assert!(!log.record(&0, false).unwrap());
    assert!(
        log.record(&"i".repeat(MAX_IMPORTANT_BYTES - 3), true)
            .unwrap()
    );
    assert_eq!(log.buffered_bytes(), MAX_TRACE_BYTES + MAX_IMPORTANT_BYTES);
    assert!(log.record(&0, true).is_err());
    assert_eq!(log.buffered_bytes(), MAX_TRACE_BYTES + MAX_IMPORTANT_BYTES);
    assert_eq!(log.dropped_records(), 1);
}

#[test]
fn unknown_modes_fields_and_out_of_range_budgets_are_rejected() {
    for text in [
        r#"{"mode":"disk","max_trace_bytes":1024}"#,
        r#"{"mode":"trace","max_trace_bytes":1024,"fsync":true}"#,
        r#"{"mode":"trace","max_trace_bytes":-1}"#,
        r#"{"mode":"trace"}"#,
    ] {
        assert!(serde_json::from_str::<TelemetryConfig>(text).is_err());
    }
    for budget in [0, MAX_TRACE_BYTES + 1, usize::MAX] {
        assert!(
            RunJournal::new(TelemetryConfig {
                mode: TelemetryMode::Trace,
                max_trace_bytes: budget,
            })
            .is_err()
        );
    }
    assert_eq!(
        serde_json::to_value(TelemetryConfig::default()).unwrap(),
        json!({
            "mode":"transitions", "max_trace_bytes":DEFAULT_TRACE_BYTES
        })
    );
}
