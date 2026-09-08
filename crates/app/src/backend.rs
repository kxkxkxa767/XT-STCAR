//! File-based reference backend; no native inference or vendor ABI is linked.
use crate::file_io::{
    MAX_CONFIG_BYTES, MAX_MODEL_BYTES, open_regular_file, read_regular_file, validate_output_file,
};
use std::fs::{self, File};
use std::io::{BufWriter, Read, Write};
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::thread;
use std::time::{Duration, Instant};
use xt_stcar_vision::{InferenceBackend, InputTensor, ModelSpec, OutputTensor, Result};

pub struct PythonReferenceBackend {
    pub python: PathBuf,
    pub worker: PathBuf,
    pub model: PathBuf,
    pub timeout: Duration,
}

/// Replace the destination directory entry, preserving any input inode reached
/// through another hard link. A failed write leaves the previous output intact.
pub fn write_atomic(
    path: &Path,
    write: impl FnOnce(&mut File) -> std::io::Result<()>,
) -> Result<()> {
    validate_output_file(path)?;
    let parent = path
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
        .unwrap_or_else(|| Path::new("."));
    let mut temporary = tempfile::NamedTempFile::new_in(parent)
        .map_err(|e| format!("create temporary output beside {}: {e}", path.display()))?;
    write(temporary.as_file_mut()).map_err(|e| format!("write {}: {e}", path.display()))?;
    temporary
        .as_file_mut()
        .flush()
        .map_err(|e| format!("flush {}: {e}", path.display()))?;
    temporary
        .persist(path)
        .map_err(|e| format!("replace {}: {e}", path.display()))?;
    Ok(())
}

pub fn write_f32le(path: &Path, input: &InputTensor) -> Result<()> {
    write_atomic(path, |file| {
        let mut writer = BufWriter::new(file);
        for value in &input.values {
            writer.write_all(&value.to_le_bytes())?;
        }
        writer.flush()
    })
}

pub fn read_output(path: &Path) -> Result<OutputTensor> {
    // The supported output has at most 1,800 scalar values. Bound malformed files.
    let bytes = read_regular_file(path, MAX_CONFIG_BYTES)?;
    serde_json::from_slice(&bytes).map_err(|e| format!("parse {}: {e}", path.display()))
}

fn diagnostics(path: &Path) -> String {
    let mut bytes = Vec::new();
    if let Ok(file) = open_regular_file(path) {
        let _ = file.take(8192).read_to_end(&mut bytes);
    }
    String::from_utf8_lossy(&bytes).trim().to_owned()
}

impl InferenceBackend for PythonReferenceBackend {
    fn infer(&mut self, input: &InputTensor, spec: &ModelSpec) -> Result<OutputTensor> {
        spec.validate()?;
        if self.timeout.is_zero() {
            return Err("worker timeout must be positive".into());
        }
        if input.shape != spec.input_shape()
            || input.values.len() != input.shape.iter().product::<usize>()
            || input
                .values
                .iter()
                .any(|v| !v.is_finite() || !(0.0..=1.0).contains(v))
        {
            return Err(
                "backend input must be finite normalized FP32 NCHW matching the spec".into(),
            );
        }
        if !self.model.is_file() || !self.worker.is_file() {
            return Err("model and worker must be existing files".into());
        }
        let model = open_regular_file(&self.model)?;
        if model.metadata().map_err(|e| e.to_string())?.len() > MAX_MODEL_BYTES {
            return Err("reference ONNX model exceeds 64 MiB".into());
        }
        drop(model);
        let temp = tempfile::Builder::new()
            .prefix("xt-stcar-infer-")
            .tempdir()
            .map_err(|e| format!("create worker temporary directory: {e}"))?;
        let input_path = temp.path().join("input.f32le");
        let spec_path = temp.path().join("spec.json");
        let output_path = temp.path().join("output.json");
        let stdout_path = temp.path().join("stdout.log");
        let stderr_path = temp.path().join("stderr.log");
        write_f32le(&input_path, input)?;
        fs::write(
            &spec_path,
            serde_json::to_vec(spec).map_err(|e| e.to_string())?,
        )
        .map_err(|e| format!("write temporary spec: {e}"))?;
        let stdout = File::create(&stdout_path).map_err(|e| e.to_string())?;
        let stderr = File::create(&stderr_path).map_err(|e| e.to_string())?;
        // Arguments are passed directly, never interpolated into a shell command.
        // Logs use files so a verbose worker cannot block on a full pipe.
        let mut child = Command::new(&self.python)
            .arg(&self.worker)
            .arg("--model")
            .arg(&self.model)
            .arg("--spec")
            .arg(&spec_path)
            .arg("--input")
            .arg(&input_path)
            .arg("--output")
            .arg(&output_path)
            .stdin(Stdio::null())
            .stdout(stdout)
            .stderr(stderr)
            .spawn()
            .map_err(|e| {
                format!(
                    "start reference worker using {}: {e}",
                    self.python.display()
                )
            })?;
        let started = Instant::now();
        let status = loop {
            match child.try_wait() {
                Ok(Some(status)) => break status,
                Ok(None) => {}
                Err(e) => {
                    let _ = child.kill();
                    let _ = child.wait();
                    return Err(format!("wait for reference worker: {e}"));
                }
            }
            if started.elapsed() >= self.timeout {
                let kill = child.kill();
                let reap = child.wait();
                return Err(format!(
                    "reference worker timed out after {:.3}s; kill={kill:?}; reap={reap:?}; stderr: {}",
                    self.timeout.as_secs_f64(),
                    diagnostics(&stderr_path)
                ));
            }
            thread::sleep(
                Duration::from_millis(10).min(self.timeout.saturating_sub(started.elapsed())),
            );
        };
        if !status.success() {
            return Err(format!(
                "reference worker failed ({status}); stdout: {}; stderr: {}",
                diagnostics(&stdout_path),
                diagnostics(&stderr_path)
            ));
        }
        for (label, path) in [("stdout", &stdout_path), ("stderr", &stderr_path)] {
            let message = diagnostics(path);
            if !message.is_empty() {
                eprintln!("reference worker {label}: {message}");
            }
        }
        // A failure, timeout, or missing output is never interpreted as detections.
        read_output(&output_path)
    }
}
