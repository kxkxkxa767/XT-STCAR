//! A persistent Rust ORT session reused across file-backed camera frames.
use crate::input::{Result, load_frame};
use serde::Serialize;
use std::path::{Path, PathBuf};
use std::time::Instant;
use xt_stcar::NativeOrtBackend;
use xt_stcar_vision::{Detection, InferenceBackend, Letterbox, ModelSpec, decode, preprocess};

pub struct VisionOptions {
    pub model: PathBuf,
    pub runtime_lib: PathBuf,
    pub spec: PathBuf,
}

pub struct VisionPipeline {
    backend: NativeOrtBackend,
    spec: ModelSpec,
}

#[derive(Serialize)]
pub struct VisionReport {
    pub backend: &'static str,
    pub inference_performed: bool,
    pub source: PathBuf,
    pub sequence: u64,
    pub frame_id: String,
    pub elapsed_ms: f64,
    pub transform: Letterbox,
    pub detections: Vec<Detection>,
}

impl VisionPipeline {
    pub fn new(options: &VisionOptions) -> Result<Self> {
        let bytes = std::fs::read(&options.spec).map_err(|e| format!("vision config: {e}"))?;
        let spec: ModelSpec = serde_json::from_slice(&bytes).map_err(|e| e.to_string())?;
        spec.validate()?;
        let backend = NativeOrtBackend::new(&options.model, &options.runtime_lib, &spec)?;
        Ok(Self { backend, spec })
    }

    pub fn infer(&mut self, path: &Path, sequence: u64, frame_id: &str) -> Result<VisionReport> {
        let started = Instant::now();
        let rgb = load_frame(path)?;
        let input = preprocess(&rgb, &self.spec)?;
        let tensor = self.backend.infer(&input, &self.spec)?;
        let detections = decode(&tensor, &self.spec, &input.transform)?;
        Ok(VisionReport {
            backend: "rust-onnxruntime",
            inference_performed: true,
            source: path.to_owned(),
            sequence,
            frame_id: frame_id.to_owned(),
            elapsed_ms: started.elapsed().as_secs_f64() * 1000.0,
            transform: input.transform,
            detections,
        })
    }
}
