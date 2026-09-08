//! Reusable Rust vision backends. Native ORT sessions can serve multiple frames.
pub mod backend;
pub mod file_io;
pub mod ort_backend;

pub use ort_backend::NativeOrtBackend;
