//! Opt-in test against an actual standard ORT C library and validated model.
//! XT_STCAR_TEST_MODEL=/.../yolo26n.onnx XT_STCAR_TEST_RUNTIME=/.../libonnxruntime...
//! cargo test -p xt-stcar --test native_runtime -- --ignored
use image::{Rgb, RgbImage};
use std::path::PathBuf;
use std::time::Duration;
use xt_stcar::NativeOrtBackend;
use xt_stcar_vision::{InferenceBackend, ModelSpec, decode, preprocess};

#[test]
#[ignore = "requires an explicit real ONNX model, validator provenance, and standard ORT C library"]
fn native_session_is_reused_and_deadlines_cancel_model_loading() {
    let model =
        PathBuf::from(std::env::var_os("XT_STCAR_TEST_MODEL").expect("XT_STCAR_TEST_MODEL"));
    let runtime =
        PathBuf::from(std::env::var_os("XT_STCAR_TEST_RUNTIME").expect("XT_STCAR_TEST_RUNTIME"));
    let spec: ModelSpec =
        serde_json::from_str(include_str!("../../../config/yolo26n.json")).unwrap();
    let input = preprocess(&RgbImage::from_pixel(640, 480, Rgb([114, 114, 114])), &spec).unwrap();
    let mut backend = NativeOrtBackend::new(&model, &runtime, &spec).unwrap();
    let first = backend.infer(&input, &spec).unwrap();
    let second = backend.infer(&input, &spec).unwrap();
    assert_eq!(first.shape, [1, 300, 6]);
    assert_eq!(
        first.values, second.values,
        "the same session/input must reproduce CPU output"
    );
    decode(&first, &spec, &input.transform).unwrap();
    let timed = NativeOrtBackend::with_options(
        &model,
        &runtime,
        &model.with_extension("provenance.json"),
        &spec,
        Duration::from_nanos(1),
    );
    assert!(
        timed
            .err()
            .expect("tiny deadline must fail")
            .contains("timed out")
    );
}
