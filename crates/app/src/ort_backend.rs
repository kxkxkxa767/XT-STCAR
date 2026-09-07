//! Standard ONNX Runtime C API through ort's runtime loader; no target library is linked.
//! A trusted validator sidecar binds graph checks to the exact ONNX bytes. It is
//! a provenance record, not a digital signature or proof of vendor compatibility.
use ort::session::{RunOptions, Session};
use ort::value::{TensorElementType, TensorRef, ValueType};
use serde::Deserialize;
use sha2::{Digest, Sha256};
use std::collections::BTreeMap;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::{Mutex, mpsc};
use std::thread;
use std::time::{Duration, Instant};
use xt_stcar_vision::{InferenceBackend, InputTensor, ModelSpec, OutputTensor, Result};

const CONTRACT: &str = "yolo26n-detect-one-to-one-fp32-static320";
const EXPORTER: &str = "8.4.142";
static RUNTIME_LIBRARY: Mutex<Option<PathBuf>> = Mutex::new(None);

#[derive(Debug, Deserialize)]
struct TensorRecord {
    name: String,
    dtype: String,
    shape: Vec<usize>,
}

#[derive(Debug, Deserialize)]
struct Provenance {
    validated: bool,
    contract: String,
    sha256: String,
    ultralytics_version: String,
    task: String,
    head: String,
    end2end: bool,
    class_count: usize,
    names: BTreeMap<String, String>,
    input: TensorRecord,
    output: TensorRecord,
    opset: u32,
    non_max_suppression_nodes: usize,
    metadata: BTreeMap<String, String>,
}

fn validate_spec(spec: &ModelSpec) -> Result<()> {
    spec.validate()?;
    if spec.input_shape() != [1, 3, 320, 320]
        || spec.input_name != "images"
        || spec.output_name != "output0"
        || spec.max_detections != 300
        || spec.class_count != 80
    {
        return Err(
            "native ORT requires the locked static320/80-class/300-row images->output0 contract"
                .into(),
        );
    }
    Ok(())
}

fn validate_provenance(record: &Provenance, model: &[u8]) -> Result<()> {
    let digest = format!("{:x}", Sha256::digest(model));
    if record.sha256 != digest {
        return Err("model SHA256 does not match its provenance; validate/export this exact ONNX file first".into());
    }
    if !record.validated
        || record.contract != CONTRACT
        || record.ultralytics_version != EXPORTER
        || record.task != "detect"
        || record.head != "Detect"
        || !record.end2end
        || record.class_count != 80
        || record.opset != 17
        || record.non_max_suppression_nodes != 0
        || record.input.name != "images"
        || record.input.dtype != "float32"
        || record.input.shape != [1, 3, 320, 320]
        || record.output.name != "output0"
        || record.output.dtype != "float32"
        || record.output.shape != [1, 300, 6]
        || record.names.len() != 80
        || (0..80).any(|id| {
            record
                .names
                .get(&id.to_string())
                .is_none_or(|name| name.trim().is_empty())
        })
    {
        return Err(
            "provenance does not attest the locked YOLO26n one-to-one FP32 contract".into(),
        );
    }
    for (key, expected) in [
        ("task", "detect"),
        ("head", "Detect"),
        ("version", EXPORTER),
        ("end2end", "True"),
    ] {
        if record.metadata.get(key).map(String::as_str) != Some(expected) {
            return Err(format!("provenance ONNX metadata {key} must be {expected}"));
        }
    }
    if record
        .metadata
        .get("names")
        .is_none_or(|value| value.trim().is_empty())
    {
        return Err("provenance must retain the original ONNX names metadata".into());
    }
    Ok(())
}

fn initialize_runtime(path: &Path) -> Result<()> {
    let path = path
        .canonicalize()
        .map_err(|e| format!("runtime library {}: {e}", path.display()))?;
    if !path.is_file() {
        return Err("runtime library must be an existing standard ORT C library file".into());
    }
    let mut selected = RUNTIME_LIBRARY
        .lock()
        .map_err(|_| "runtime initialization lock was poisoned")?;
    if let Some(existing) = selected.as_ref() {
        return if *existing == path {
            Ok(())
        } else {
            Err("the process already initialized a different ORT runtime library".into())
        };
    }
    let committed = ort::init_from(&path)
        .map_err(|e| format!("load standard ORT C runtime {}: {e}", path.display()))?
        .with_telemetry(false)
        .commit();
    if !committed {
        return Err(
            "ORT was already initialized outside NativeOrtBackend; runtime identity is ambiguous"
                .into(),
        );
    }
    *selected = Some(path);
    Ok(())
}

/// The timeout requests cancellation through the standard ORT C API. A native
/// runtime that does not return from cancellation cannot be forcibly killed in
/// process; this is deliberately distinct from the Python subprocess deadline.
fn with_deadline<T>(
    timeout: Duration,
    operation: impl FnOnce() -> Result<T>,
    cancel: impl FnOnce() + Send,
) -> Result<T> {
    if timeout.is_zero() {
        return Err("native ORT timeout must be positive".into());
    }
    thread::scope(|scope| {
        let started = Instant::now();
        let (done, received) = mpsc::channel();
        let watchdog = scope.spawn(move || {
            if received.recv_timeout(timeout) == Err(mpsc::RecvTimeoutError::Timeout) {
                cancel();
                true
            } else {
                false
            }
        });
        let result = operation();
        let _ = done.send(());
        let cancellation_requested = watchdog
            .join()
            .map_err(|_| "native ORT timeout watchdog panicked")?;
        if cancellation_requested || started.elapsed() >= timeout {
            return Err(format!(
                "native ORT timed out after {:.3}s; cancellation is cooperative",
                timeout.as_secs_f64()
            ));
        }
        result
    })
}

fn validate_outlet(outlet: &ort::value::Outlet, name: &str, expected: &[i64]) -> Result<()> {
    if outlet.name() != name {
        return Err(format!(
            "runtime tensor name must be {name}, got {}",
            outlet.name()
        ));
    }
    match outlet.dtype() {
        ValueType::Tensor {
            ty: TensorElementType::Float32,
            shape,
            ..
        } if &shape[..] == expected => Ok(()),
        actual => Err(format!(
            "runtime {name} must be static FLOAT32 {expected:?}, got {actual:?}"
        )),
    }
}

pub struct NativeOrtBackend {
    session: Session,
    timeout: Duration,
    model_sha256: String,
    runtime_info: String,
}

impl NativeOrtBackend {
    pub fn new(model_path: &Path, runtime_lib: &Path, spec: &ModelSpec) -> Result<Self> {
        Self::with_options(
            model_path,
            runtime_lib,
            &model_path.with_extension("provenance.json"),
            spec,
            Duration::from_secs(30),
        )
    }

    pub fn with_options(
        model_path: &Path,
        runtime_lib: &Path,
        provenance_path: &Path,
        spec: &ModelSpec,
        timeout: Duration,
    ) -> Result<Self> {
        validate_spec(spec)?;
        if timeout.is_zero() {
            return Err("native ORT timeout must be positive".into());
        }
        if fs::metadata(model_path)
            .map_err(|e| format!("model {}: {e}", model_path.display()))?
            .len()
            > 64 * 1024 * 1024
        {
            return Err("baseline ONNX model exceeds 64 MiB".into());
        }
        if fs::metadata(provenance_path)
            .map_err(|e| format!("provenance {}: {e}", provenance_path.display()))?
            .len()
            > 1024 * 1024
        {
            return Err("provenance JSON exceeds 1 MiB".into());
        }
        let model = fs::read(model_path).map_err(|e| format!("read model: {e}"))?;
        let record: Provenance = serde_json::from_slice(
            &fs::read(provenance_path).map_err(|e| format!("read provenance: {e}"))?,
        )
        .map_err(|e| format!("parse provenance: {e}"))?;
        validate_provenance(&record, &model)?;
        initialize_runtime(runtime_lib)?;
        let mut builder = Session::builder()
            .map_err(|e| format!("create ORT session: {e}"))?
            .with_intra_threads(2)
            .map_err(|e| e.to_string())?;
        let canceler = builder.canceler();
        // Load the hashed bytes, preventing a path replacement between validation and load.
        let session = with_deadline(
            timeout,
            || {
                builder
                    .commit_from_memory(&model)
                    .map_err(|e| format!("load ONNX model: {e}"))
            },
            move || {
                let _ = canceler.cancel();
            },
        )?;
        if session.inputs().len() != 1 || session.outputs().len() != 1 {
            return Err("runtime must expose exactly one input and one output".into());
        }
        validate_outlet(&session.inputs()[0], "images", &[1, 3, 320, 320])?;
        validate_outlet(&session.outputs()[0], "output0", &[1, 300, 6])?;
        let metadata = session
            .metadata()
            .map_err(|e| format!("read runtime model metadata: {e}"))?;
        for key in ["task", "head", "version", "end2end", "names"] {
            if metadata.custom(key).as_ref() != record.metadata.get(key) {
                return Err(format!(
                    "runtime model metadata {key} does not match validated provenance"
                ));
            }
        }
        drop(metadata);
        Ok(Self {
            session,
            timeout,
            model_sha256: record.sha256,
            runtime_info: ort::info().to_owned(),
        })
    }

    pub fn model_sha256(&self) -> &str {
        &self.model_sha256
    }
    pub fn runtime_info(&self) -> &str {
        &self.runtime_info
    }
}

impl InferenceBackend for NativeOrtBackend {
    fn infer(&mut self, input: &InputTensor, spec: &ModelSpec) -> Result<OutputTensor> {
        validate_spec(spec)?;
        if input.shape != spec.input_shape()
            || input.values.len() != 3 * 320 * 320
            || input
                .values
                .iter()
                .any(|v| !v.is_finite() || !(0.0..=1.0).contains(v))
        {
            return Err("native ORT input must be finite normalized FLOAT32 [1,3,320,320]".into());
        }
        let tensor = TensorRef::from_array_view((input.shape, input.values.as_slice()))
            .map_err(|e| format!("create NCHW input: {e}"))?;
        let options = RunOptions::new().map_err(|e| e.to_string())?;
        with_deadline(
            self.timeout,
            || {
                let outputs = self
                    .session
                    .run_with_options(ort::inputs!["images" => tensor], &options)
                    .map_err(|e| format!("native ORT inference failed: {e}"))?;
                let output = outputs.get("output0").ok_or("runtime omitted output0")?;
                let (shape, values) = output
                    .try_extract_tensor::<f32>()
                    .map_err(|e| format!("extract FLOAT32 output: {e}"))?;
                if shape[..] != [1, 300, 6] || values.len() != 1800 {
                    return Err("runtime returned a nonconforming output tensor".into());
                }
                Ok(OutputTensor {
                    shape: vec![1, 300, 6],
                    values: values.to_vec(),
                })
            },
            || {
                let _ = options.terminate();
            },
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicBool, Ordering};

    fn provenance(model: &[u8]) -> Provenance {
        Provenance {
            validated: true,
            contract: CONTRACT.into(),
            sha256: format!("{:x}", Sha256::digest(model)),
            ultralytics_version: EXPORTER.into(),
            task: "detect".into(),
            head: "Detect".into(),
            end2end: true,
            class_count: 80,
            names: (0..80)
                .map(|id| (id.to_string(), format!("class{id}")))
                .collect(),
            input: TensorRecord {
                name: "images".into(),
                dtype: "float32".into(),
                shape: vec![1, 3, 320, 320],
            },
            output: TensorRecord {
                name: "output0".into(),
                dtype: "float32".into(),
                shape: vec![1, 300, 6],
            },
            opset: 17,
            non_max_suppression_nodes: 0,
            metadata: [
                ("task", "detect"),
                ("head", "Detect"),
                ("version", EXPORTER),
                ("end2end", "True"),
                ("names", "synthetic metadata fixture"),
            ]
            .into_iter()
            .map(|(key, value)| (key.into(), value.into()))
            .collect(),
        }
    }

    #[test]
    fn provenance_binds_digest_and_rejects_non_end_to_end_or_raw_heads() {
        let model = b"synthetic byte fixture, not an ONNX model";
        let mut record = provenance(model);
        assert!(validate_provenance(&record, model).is_ok());
        assert!(
            validate_provenance(&record, b"modified model")
                .unwrap_err()
                .contains("SHA256")
        );
        record.end2end = false;
        assert!(validate_provenance(&record, model).is_err());
        record.end2end = true;
        record.output.shape = vec![1, 84, 2100];
        assert!(validate_provenance(&record, model).is_err());
        record.output.shape = vec![1, 300, 6];
        record.non_max_suppression_nodes = 1;
        assert!(validate_provenance(&record, model).is_err());
        record.non_max_suppression_nodes = 0;
        record.metadata.remove("names");
        assert!(validate_provenance(&record, model).is_err());
    }

    #[test]
    fn deadline_does_not_cancel_a_completed_operation() {
        let canceled = AtomicBool::new(false);
        assert_eq!(
            with_deadline(
                Duration::from_secs(1),
                || Ok(42),
                || {
                    canceled.store(true, Ordering::SeqCst);
                }
            )
            .unwrap(),
            42
        );
        assert!(!canceled.load(Ordering::SeqCst));
    }

    #[test]
    fn deadline_requests_cancellation_and_rejects_late_results() {
        let canceled = AtomicBool::new(false);
        let result = with_deadline(
            Duration::from_millis(10),
            || {
                while !canceled.load(Ordering::SeqCst) {
                    thread::yield_now();
                }
                Ok(42)
            },
            || {
                canceled.store(true, Ordering::SeqCst);
            },
        );
        assert!(result.unwrap_err().contains("timed out"));
    }
}
