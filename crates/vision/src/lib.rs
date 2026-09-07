//! YOLO26 Detect one-to-one interface. No actuator or ROS dependencies.
use image::RgbImage;
use serde::{Deserialize, Serialize};

pub type Result<T> = std::result::Result<T, String>;

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct ModelSpec {
    pub schema_version: u32,
    pub model_family: String,
    pub task: String,
    pub input_name: String,
    pub output_name: String,
    pub input_width: u32,
    pub input_height: u32,
    pub input_layout: String,
    pub input_dtype: String,
    pub output_layout: String,
    pub max_detections: usize,
    pub class_count: u32,
    pub confidence_threshold: f32,
}

impl ModelSpec {
    pub fn validate(&self) -> Result<()> {
        if self.schema_version != 1 || self.model_family != "yolo26n" || self.task != "detect" {
            return Err("only schema 1 / yolo26n / detect is supported".into());
        }
        if self.input_layout != "NCHW_RGB_0_1"
            || self.input_dtype != "float32"
            || self.output_layout != "B_N_XYXY_SCORE_CLASS"
        {
            return Err(
                "expected FP32 RGB NCHW input and end-to-end XYXY/score/class output".into(),
            );
        }
        if self.input_name.is_empty() || self.output_name.is_empty() {
            return Err("tensor names must be explicit".into());
        }
        for size in [self.input_width, self.input_height] {
            if !(32..=1280).contains(&size) || size % 32 != 0 {
                return Err("input dimensions must be multiples of 32 in 32..=1280".into());
            }
        }
        if !(1..=1000).contains(&self.class_count) || !(1..=300).contains(&self.max_detections) {
            return Err("invalid class_count or max_detections".into());
        }
        if !self.confidence_threshold.is_finite()
            || !(0.0..=1.0).contains(&self.confidence_threshold)
        {
            return Err("confidence_threshold must be finite and in [0,1]".into());
        }
        Ok(())
    }

    pub fn input_shape(&self) -> [usize; 4] {
        [1, 3, self.input_height as usize, self.input_width as usize]
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Letterbox {
    pub source_width: u32,
    pub source_height: u32,
    pub input_width: u32,
    pub input_height: u32,
    pub resized_width: u32,
    pub resized_height: u32,
    pub pad_left: u32,
    pub pad_top: u32,
    pub scale: f64,
}

impl Letterbox {
    pub fn new(width: u32, height: u32, spec: &ModelSpec) -> Result<Self> {
        spec.validate()?;
        if width == 0 || height == 0 || u64::from(width) * u64::from(height) > 64_000_000 {
            return Err("source dimensions must be positive and at most 64 megapixels".into());
        }
        let scale = (f64::from(spec.input_width) / f64::from(width))
            .min(f64::from(spec.input_height) / f64::from(height));
        // Python round() uses ties-to-even; center with integer left/top padding.
        let rw = (f64::from(width) * scale).round_ties_even() as u32;
        let rh = (f64::from(height) * scale).round_ties_even() as u32;
        if rw == 0 || rh == 0 {
            return Err("source aspect ratio would resize one dimension to zero".into());
        }
        Ok(Self {
            source_width: width,
            source_height: height,
            input_width: spec.input_width,
            input_height: spec.input_height,
            resized_width: rw,
            resized_height: rh,
            pad_left: (spec.input_width - rw) / 2,
            pad_top: (spec.input_height - rh) / 2,
            scale,
        })
    }
}

pub struct InputTensor {
    pub shape: [usize; 4],
    pub values: Vec<f32>,
    pub transform: Letterbox,
}

/// Bilinear resize with half-pixel sampling, RGB channel order, fill=114.
/// OpenCV's optimized fixed-point interpolation can differ by one intensity level.
pub fn preprocess(rgb: &RgbImage, spec: &ModelSpec) -> Result<InputTensor> {
    let t = Letterbox::new(rgb.width(), rgb.height(), spec)?;
    let plane = (t.input_width * t.input_height) as usize;
    let mut values = vec![114.0 / 255.0; 3 * plane];
    for dy in 0..t.resized_height {
        let sy = ((f64::from(dy) + 0.5) * f64::from(rgb.height()) / f64::from(t.resized_height)
            - 0.5)
            .max(0.0);
        let y0 = (sy.floor() as u32).min(rgb.height() - 1);
        let y1 = (y0 + 1).min(rgb.height() - 1);
        let fy = sy - f64::from(y0);
        for dx in 0..t.resized_width {
            let sx = ((f64::from(dx) + 0.5) * f64::from(rgb.width()) / f64::from(t.resized_width)
                - 0.5)
                .max(0.0);
            let x0 = (sx.floor() as u32).min(rgb.width() - 1);
            let x1 = (x0 + 1).min(rgb.width() - 1);
            let fx = sx - f64::from(x0);
            let index = ((dy + t.pad_top) * t.input_width + dx + t.pad_left) as usize;
            for c in 0..3 {
                let top = f64::from(rgb.get_pixel(x0, y0)[c]) * (1.0 - fx)
                    + f64::from(rgb.get_pixel(x1, y0)[c]) * fx;
                let bottom = f64::from(rgb.get_pixel(x0, y1)[c]) * (1.0 - fx)
                    + f64::from(rgb.get_pixel(x1, y1)[c]) * fx;
                values[c * plane + index] = (top * (1.0 - fy) + bottom * fy).round() as f32 / 255.0;
            }
        }
    }
    Ok(InputTensor {
        shape: spec.input_shape(),
        values,
        transform: t,
    })
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct OutputTensor {
    pub shape: Vec<usize>,
    pub values: Vec<f32>,
}

/// Backends must perform inference only. Pre/postprocessing belongs to this crate.
pub trait InferenceBackend {
    fn infer(&mut self, input: &InputTensor, spec: &ModelSpec) -> Result<OutputTensor>;
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct Detection {
    pub class_id: u32,
    pub confidence: f32,
    /// Original-image pixel coordinates; not ground-plane distance.
    pub xyxy: [f32; 4],
}

/// Decode only native end-to-end output. Do not apply a second NMS pass.
pub fn decode(output: &OutputTensor, spec: &ModelSpec, t: &Letterbox) -> Result<Vec<Detection>> {
    spec.validate()?;
    let expected = Letterbox::new(t.source_width, t.source_height, spec)?;
    if t.input_width != expected.input_width
        || t.input_height != expected.input_height
        || t.resized_width != expected.resized_width
        || t.resized_height != expected.resized_height
        || t.pad_left != expected.pad_left
        || t.pad_top != expected.pad_top
        || !t.scale.is_finite()
        || (t.scale - expected.scale).abs() > 1e-9
    {
        return Err("letterbox metadata does not match model/source dimensions".into());
    }
    if output.shape != [1, spec.max_detections, 6] || output.values.len() != spec.max_detections * 6
    {
        return Err(format!(
            "expected [1,{},6] end-to-end output; raw YOLO11/one-to-many output is unsupported",
            spec.max_detections
        ));
    }
    let mut detections = Vec::new();
    for row in output.values.chunks_exact(6) {
        if row.iter().any(|v| !v.is_finite()) || !(0.0..=1.0).contains(&row[4]) {
            return Err("model output has non-finite values or invalid confidence".into());
        }
        let class = row[5];
        if class < 0.0 || class >= spec.class_count as f32 || class.fract() != 0.0 {
            return Err("model output class must be an integer within class_count".into());
        }
        if row[4] <= spec.confidence_threshold {
            continue;
        }
        if row[2] <= row[0] || row[3] <= row[1] {
            return Err("model output contains an inverted or empty confident box".into());
        }
        let x = |v: f32| {
            ((f64::from(v) - f64::from(t.pad_left)) / t.scale).clamp(0.0, f64::from(t.source_width))
                as f32
        };
        let y = |v: f32| {
            ((f64::from(v) - f64::from(t.pad_top)) / t.scale).clamp(0.0, f64::from(t.source_height))
                as f32
        };
        let xyxy = [x(row[0]), y(row[1]), x(row[2]), y(row[3])];
        if xyxy[2] <= xyxy[0] || xyxy[3] <= xyxy[1] {
            continue;
        }
        detections.push(Detection {
            class_id: class as u32,
            confidence: row[4],
            xyxy,
        });
    }
    Ok(detections)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn spec() -> ModelSpec {
        serde_json::from_str(include_str!("../../../config/yolo26n.json")).unwrap()
    }
    fn output(row: [f32; 6]) -> OutputTensor {
        let mut values = vec![0.0; 300 * 6];
        values[..6].copy_from_slice(&row);
        OutputTensor {
            shape: vec![1, 300, 6],
            values,
        }
    }
    #[test]
    fn letterbox_rgb_normalization_and_padding() {
        let s = spec();
        let image = RgbImage::from_pixel(640, 480, image::Rgb([255, 0, 128]));
        let input = preprocess(&image, &s).unwrap();
        assert_eq!(input.shape, [1, 3, 320, 320]);
        assert_eq!((input.transform.pad_left, input.transform.pad_top), (0, 40));
        assert_eq!(input.values[0], 114.0 / 255.0);
        assert_eq!(input.values[40 * 320], 1.0);
        assert_eq!(input.values[320 * 320 + 40 * 320], 0.0);
        assert_eq!(input.values[2 * 320 * 320 + 40 * 320], 128.0 / 255.0);
    }
    #[test]
    fn unletterbox_threshold_and_no_extra_nms() {
        let s = spec();
        let t = Letterbox::new(640, 480, &s).unwrap();
        let mut o = output([50.0, 65.0, 150.0, 165.0, 0.8, 2.0]);
        o.values[6..12].copy_from_slice(&[50.0, 65.0, 150.0, 165.0, 0.7, 2.0]);
        let boxes = decode(&o, &s, &t).unwrap();
        assert_eq!(boxes.len(), 2);
        assert_eq!(boxes[0].xyxy, [100.0, 50.0, 300.0, 250.0]);
        o.values[4] = 0.1;
        assert_eq!(decode(&o, &s, &t).unwrap().len(), 1);
    }
    #[test]
    fn reject_wrong_layout_malformed_output_and_transform() {
        let s = spec();
        let mut t = Letterbox::new(640, 480, &s).unwrap();
        let mut o = output([50.0, 65.0, 150.0, 165.0, 0.8, 2.0]);
        o.shape = vec![1, 84, 2100];
        assert!(decode(&o, &s, &t).is_err());
        o.shape = vec![1, 300, 6];
        for value in [f32::NAN, f32::INFINITY, -1.0, 80.0, 2.5] {
            o.values[5] = value;
            assert!(decode(&o, &s, &t).is_err());
        }
        o.values[5] = 2.0;
        t.pad_top += 1;
        assert!(decode(&o, &s, &t).is_err());
    }
    #[test]
    fn clip_boxes_and_drop_padding_only() {
        let s = spec();
        let t = Letterbox::new(640, 480, &s).unwrap();
        assert_eq!(
            decode(&output([-20.0, 20.0, 350.0, 300.0, 0.8, 0.0]), &s, &t).unwrap()[0].xyxy,
            [0.0, 0.0, 640.0, 480.0]
        );
        assert!(
            decode(&output([5.0, 1.0, 10.0, 30.0, 0.8, 0.0]), &s, &t)
                .unwrap()
                .is_empty()
        );
    }
    #[test]
    fn odd_padding_and_invalid_dimensions() {
        let s = spec();
        let t = Letterbox::new(641, 480, &s).unwrap();
        assert_eq!(t.resized_height, 240);
        assert_eq!(t.pad_top, 40);
        assert!(Letterbox::new(0, 480, &s).is_err());
        assert!(Letterbox::new(1, 64_000_000, &s).is_err());
        let mut bad = s;
        bad.confidence_threshold = f32::NAN;
        assert!(bad.validate().is_err());
    }

    #[test]
    fn confidence_is_strictly_greater_than_threshold() {
        let mut s = spec();
        let t = Letterbox::new(320, 320, &s).unwrap();
        let row = |confidence| output([0.0, 0.0, 20.0, 20.0, confidence, 0.0]);
        assert!(decode(&row(0.25), &s, &t).unwrap().is_empty());
        assert!(
            decode(&row(0.25_f32.next_down()), &s, &t)
                .unwrap()
                .is_empty()
        );
        assert_eq!(decode(&row(0.25_f32.next_up()), &s, &t).unwrap().len(), 1);
        s.confidence_threshold = 0.0;
        assert!(decode(&row(0.0), &s, &t).unwrap().is_empty());
        s.confidence_threshold = 1.0;
        assert!(decode(&row(1.0), &s, &t).unwrap().is_empty());
    }

    #[test]
    fn reject_invalid_scores_and_confident_boxes() {
        let s = spec();
        let t = Letterbox::new(320, 320, &s).unwrap();
        for confidence in [-0.1, 1.1, f32::NAN, f32::INFINITY] {
            assert!(decode(&output([0.0, 0.0, 20.0, 20.0, confidence, 0.0]), &s, &t).is_err());
        }
        for row in [
            [20.0, 0.0, 0.0, 20.0, 0.5, 0.0],
            [0.0, 20.0, 20.0, 20.0, 0.5, 0.0],
            [f32::NAN, 0.0, 20.0, 20.0, 0.0, 0.0],
        ] {
            assert!(decode(&output(row), &s, &t).is_err());
        }
        assert!(
            decode(
                &output([20.0, 20.0, 0.0, 0.0, s.confidence_threshold, 0.0]),
                &s,
                &t
            )
            .unwrap()
            .is_empty()
        );
        let mut truncated = output([0.0; 6]);
        truncated.values.pop();
        assert!(decode(&truncated, &s, &t).is_err());
    }

    #[test]
    fn ties_to_even_and_asymmetric_padding() {
        let mut s = spec();
        s.input_width = 32;
        s.input_height = 32;
        let t = Letterbox::new(128, 10, &s).unwrap();
        assert_eq!((t.resized_width, t.resized_height, t.pad_top), (32, 2, 15));
        let t = Letterbox::new(128, 14, &s).unwrap();
        assert_eq!((t.resized_height, t.pad_top), (4, 14));
        let t = Letterbox::new(128, 12, &s).unwrap();
        assert_eq!((t.resized_height, t.pad_top), (3, 14));
        assert_eq!(t.input_height - t.resized_height - t.pad_top, 15);
    }

    #[test]
    fn resize_uses_half_pixel_sampling_and_rgb_planes() {
        let mut s = spec();
        s.input_width = 32;
        s.input_height = 32;
        let image = RgbImage::from_fn(64, 64, |x, y| image::Rgb([x as u8, y as u8, 255]));
        let input = preprocess(&image, &s).unwrap();
        assert_eq!(input.values[0], 1.0 / 255.0);
        assert_eq!(input.values[31], 63.0 / 255.0);
        assert_eq!(input.values[1024 + 31 * 32], 63.0 / 255.0);
        assert!(input.values[2048..].iter().all(|value| *value == 1.0));
    }
}
