use image::{ImageReader, Rgb, RgbImage};
use serde::Serialize;
use std::collections::BTreeMap;
use std::ffi::OsString;
use std::fs::File;
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};
use xt_stcar::NativeOrtBackend;
use xt_stcar::backend::{PythonReferenceBackend, read_output, write_atomic, write_f32le};
use xt_stcar_vision::{
    Detection, InferenceBackend, Letterbox, ModelSpec, OutputTensor, Result, decode, preprocess,
};

const HELP: &str = "XT-STCAR offline vision CLI (no camera, ROS, or actuator control)

Usage:
  xt-stcar self-check [--config FILE] [--output FILE]
  xt-stcar preprocess --image FILE --tensor FILE --transform FILE [--config FILE]
  xt-stcar replay --image FILE --tensor FILE [--config FILE] [--output FILE]
  xt-stcar infer --image FILE --model FILE --runtime-lib FILE
                [--provenance FILE] [--config FILE] [--output FILE] [--timeout-secs SECONDS]
  xt-stcar infer --backend python-reference --image FILE --model FILE
                [--python EXECUTABLE] [--worker FILE] [--config FILE] [--output FILE]
                [--timeout-secs SECONDS]

Defaults: --backend native-ort, --config config/yolo26n.json, --python python3,
          --worker scripts/onnx_worker.py, --timeout-secs 30 (0 < seconds <= 3600).
Paths are relative to the current directory. PNG and JPEG input are supported.
preprocess writes contiguous little-endian float32 NCHW and a Letterbox JSON file.
replay reads {\"shape\":[1,N,6],\"values\":[...]} tensor JSON; it performs no inference.
self-check uses a synthetic image and synthetic detections; it performs no inference.
infer defaults to an in-process Rust ORT session using the explicit standard C runtime.
Its trusted model.provenance.json sidecar binds validator checks to the ONNX SHA256.
Native timeout requests cooperative ORT cancellation; Python timeout kills its worker.
Both backends require the fixed FP32 320/80-class/300-row model contract.
Standard ORT API compatibility does not establish vendor or vehicle compatibility.
Reports are JSON on stdout, or --output FILE. Diagnostics go to stderr.
";

struct Options(BTreeMap<String, OsString>);

impl Options {
    fn parse(args: impl Iterator<Item = OsString>) -> Result<Self> {
        let mut args = args;
        let mut values = BTreeMap::new();
        while let Some(key) = args.next() {
            let key = key
                .into_string()
                .map_err(|_| "option names must be UTF-8")?;
            if !key.starts_with("--") {
                return Err(format!("expected --option, got {key}"));
            }
            let value = args
                .next()
                .ok_or_else(|| format!("missing value for {key}"))?;
            if value.to_string_lossy().starts_with("--") || value.is_empty() {
                return Err(format!("missing value for {key}"));
            }
            if values.insert(key.clone(), value).is_some() {
                return Err(format!("duplicate option {key}"));
            }
        }
        Ok(Self(values))
    }

    fn take(&mut self, key: &str) -> Option<PathBuf> {
        self.0.remove(key).map(PathBuf::from)
    }

    fn required(&mut self, key: &str) -> Result<PathBuf> {
        self.take(key)
            .ok_or_else(|| format!("missing required option {key}"))
    }

    fn finish(self) -> Result<()> {
        match self.0.first_key_value() {
            Some((key, _)) => Err(format!("unknown or inapplicable option {key}")),
            None => Ok(()),
        }
    }
}

#[derive(Serialize)]
struct DetectionReport {
    schema_version: u32,
    mode: &'static str,
    backend: &'static str,
    inference_performed: bool,
    note: &'static str,
    elapsed_ms: f64,
    input_shape: [usize; 4],
    transform: Letterbox,
    detections: Vec<Detection>,
    #[serde(skip_serializing_if = "Option::is_none")]
    model_sha256: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    runtime_info: Option<String>,
}

enum BackendConfig {
    Python(PythonReferenceBackend),
    Native {
        model: PathBuf,
        runtime_lib: PathBuf,
        provenance: PathBuf,
        timeout: Duration,
    },
}

fn write_json<T: Serialize>(path: Option<&Path>, value: &T) -> Result<()> {
    let mut bytes = serde_json::to_vec_pretty(value).map_err(|e| e.to_string())?;
    bytes.push(b'\n');
    match path {
        Some(path) => write_atomic(path, |file| file.write_all(&bytes)),
        None => io::stdout()
            .lock()
            .write_all(&bytes)
            .map_err(|e| format!("write stdout: {e}")),
    }
}

fn load_spec(path: &Path) -> Result<ModelSpec> {
    let file = File::open(path).map_err(|e| format!("open config {}: {e}", path.display()))?;
    let spec: ModelSpec = serde_json::from_reader(file)
        .map_err(|e| format!("parse config {}: {e}", path.display()))?;
    spec.validate()?;
    Ok(spec)
}

fn load_image(path: &Path) -> Result<RgbImage> {
    let mut reader = ImageReader::open(path)
        .map_err(|e| format!("open image {}: {e}", path.display()))?
        .with_guessed_format()
        .map_err(|e| e.to_string())?;
    let mut limits = image::Limits::default();
    limits.max_alloc = Some(512 * 1024 * 1024);
    reader.limits(limits);
    // Check area before decoding, including unusually long one-pixel images.
    let (width, height) = ImageReader::open(path)
        .map_err(|e| e.to_string())?
        .with_guessed_format()
        .map_err(|e| e.to_string())?
        .into_dimensions()
        .map_err(|e| format!("read image dimensions: {e}"))?;
    if width == 0 || height == 0 || u64::from(width) * u64::from(height) > 64_000_000 {
        return Err("source dimensions must be positive and at most 64 megapixels".into());
    }
    reader
        .decode()
        .map(|image| image.to_rgb8())
        .map_err(|e| format!("decode image {}: {e}", path.display()))
}

fn ensure_distinct(paths: &[&Path]) -> Result<()> {
    // Canonicalize existing files/parents so outputs cannot silently overwrite inputs.
    let current = std::env::current_dir().map_err(|e| e.to_string())?;
    let mut normalized = Vec::new();
    for path in paths {
        let absolute = if path.is_absolute() {
            path.to_path_buf()
        } else {
            current.join(path)
        };
        let resolved = if absolute.exists() {
            absolute.canonicalize().map_err(|e| e.to_string())?
        } else {
            let parent = absolute.parent().ok_or("output path has no parent")?;
            parent
                .canonicalize()
                .map_err(|e| format!("output directory {}: {e}", parent.display()))?
                .join(absolute.file_name().ok_or("output path has no filename")?)
        };
        if normalized.contains(&resolved) {
            return Err("input, configuration, and output paths must be distinct".into());
        }
        normalized.push(resolved);
    }
    Ok(())
}

fn run() -> Result<()> {
    let mut args = std::env::args_os().skip(1);
    let Some(command) = args.next() else {
        print!("{HELP}");
        return Ok(());
    };
    let rest: Vec<OsString> = args.collect();
    if command == "--help"
        || command == "-h"
        || (rest.len() == 1 && (rest[0] == "--help" || rest[0] == "-h"))
    {
        print!("{HELP}");
        return Ok(());
    }
    let command = command.to_str().ok_or("command must be UTF-8")?;
    if !["self-check", "preprocess", "replay", "infer"].contains(&command) {
        return Err(format!("unknown command {command}; use --help"));
    }
    let mut options = Options::parse(rest.into_iter())?;
    let config = options
        .take("--config")
        .unwrap_or_else(|| "config/yolo26n.json".into());
    let output = if command != "preprocess" {
        options.take("--output")
    } else {
        None
    };
    let image_path = if command != "self-check" {
        Some(options.required("--image")?)
    } else {
        None
    };
    let tensor_path = if command == "preprocess" || command == "replay" {
        Some(options.required("--tensor")?)
    } else {
        None
    };
    let transform_path = if command == "preprocess" {
        Some(options.required("--transform")?)
    } else {
        None
    };
    let backend = if command == "infer" {
        let kind = options
            .0
            .remove("--backend")
            .unwrap_or_else(|| "native-ort".into());
        let model = options.required("--model")?;
        let timeout = match options.0.remove("--timeout-secs") {
            Some(value) => value
                .to_str()
                .ok_or("timeout must be a number")?
                .parse::<f64>()
                .map_err(|_| "timeout must be a number")?,
            None => 30.0,
        };
        if !timeout.is_finite() || timeout <= 0.0 || timeout > 3600.0 {
            return Err("timeout must be finite and 0 < seconds <= 3600".into());
        }
        let timeout = Duration::from_secs_f64(timeout);
        Some(match kind.to_str() {
            Some("native-ort") => BackendConfig::Native {
                runtime_lib: options.required("--runtime-lib")?,
                provenance: options
                    .take("--provenance")
                    .unwrap_or_else(|| model.with_extension("provenance.json")),
                model,
                timeout,
            },
            Some("python-reference") => BackendConfig::Python(PythonReferenceBackend {
                python: options.take("--python").unwrap_or_else(|| "python3".into()),
                worker: options
                    .take("--worker")
                    .unwrap_or_else(|| "scripts/onnx_worker.py".into()),
                model,
                timeout,
            }),
            _ => return Err("--backend must be native-ort or python-reference".into()),
        })
    } else {
        None
    };
    options.finish()?;
    let mut paths = vec![config.as_path()];
    for path in [&output, &image_path, &tensor_path, &transform_path]
        .into_iter()
        .flatten()
    {
        paths.push(path.as_path());
    }
    if let Some(backend) = &backend {
        match backend {
            BackendConfig::Python(backend) => {
                paths.push(&backend.worker);
                paths.push(&backend.model);
            }
            BackendConfig::Native {
                model,
                runtime_lib,
                provenance,
                ..
            } => {
                paths.extend([model.as_path(), runtime_lib.as_path(), provenance.as_path()]);
            }
        }
    }
    ensure_distinct(&paths)?;
    let spec = load_spec(&config)?;
    let started = Instant::now();
    let image = match image_path {
        Some(path) => load_image(&path)?,
        None => RgbImage::from_pixel(640, 480, Rgb([255, 0, 128])),
    };
    let input = preprocess(&image, &spec)?;
    if command == "preprocess" {
        let tensor = tensor_path.as_deref().ok_or("missing tensor path")?;
        let transform = transform_path.as_deref().ok_or("missing transform path")?;
        write_f32le(tensor, &input)?;
        write_json(Some(transform), &input.transform)?;
        return write_json(
            None,
            &serde_json::json!({
                "schema_version": 1, "mode": "preprocess", "backend": "none",
                "inference_performed": false, "input_shape": input.shape,
                "tensor": tensor, "transform": transform,
                "element_count": input.values.len(), "byte_order": "little-endian",
            }),
        );
    }
    let mut model_sha256 = None;
    let mut runtime_info = None;
    let (tensor, mode, backend_name, note) = match command {
        "self-check" => {
            let mut values = vec![0.0; spec.max_detections * 6];
            let t = &input.transform;
            values[..6].copy_from_slice(&[
                t.pad_left as f32,
                t.pad_top as f32,
                (t.pad_left + t.resized_width) as f32,
                (t.pad_top + t.resized_height) as f32,
                1.0,
                0.0,
            ]);
            (
                OutputTensor {
                    shape: vec![1, spec.max_detections, 6],
                    values,
                },
                "self-check",
                "synthetic",
                "Synthetic image and tensor only; no model was loaded or executed.",
            )
        }
        "replay" => (
            read_output(tensor_path.as_deref().ok_or("missing tensor path")?)?,
            "replay",
            "saved-tensor",
            "Saved tensor decoding only; no model was loaded or executed.",
        ),
        "infer" => match backend.ok_or("missing backend")? {
            BackendConfig::Python(mut backend) => (
                backend.infer(&input, &spec)?,
                "infer",
                "python-onnxruntime-reference",
                "Explicit Python reference worker; vendor/vehicle compatibility is unverified.",
            ),
            BackendConfig::Native {
                model,
                runtime_lib,
                provenance,
                timeout,
            } => {
                let mut backend = NativeOrtBackend::with_options(
                    &model,
                    &runtime_lib,
                    &provenance,
                    &spec,
                    timeout,
                )?;
                let result = backend.infer(&input, &spec)?;
                model_sha256 = Some(backend.model_sha256().to_owned());
                runtime_info = Some(backend.runtime_info().to_owned());
                (
                    result,
                    "infer",
                    "rust-onnxruntime",
                    "Rust calls the standard ORT C API in process; vendor/vehicle compatibility is unverified.",
                )
            }
        },
        _ => unreachable!("commands were validated above"),
    };
    let detections = decode(&tensor, &spec, &input.transform)?;
    let report = DetectionReport {
        schema_version: 1,
        mode,
        backend: backend_name,
        inference_performed: command == "infer",
        note,
        elapsed_ms: started.elapsed().as_secs_f64() * 1000.0,
        input_shape: input.shape,
        transform: input.transform,
        detections,
        model_sha256,
        runtime_info,
    };
    write_json(output.as_deref(), &report)
}

fn main() -> std::process::ExitCode {
    match run() {
        Ok(()) => std::process::ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("xt-stcar: {error}");
            std::process::ExitCode::from(2)
        }
    }
}
