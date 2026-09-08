//! Bounded static inputs and ordinary output paths shared by both CLIs.
use rustix::fs::{FileType, Mode, OFlags, fstat, open};
use std::fs::File;
use std::io::Read;
use std::path::Path;
use xt_stcar_vision::Result;

pub const MAX_CONFIG_BYTES: u64 = 1024 * 1024;
pub const MAX_MODEL_BYTES: u64 = 64 * 1024 * 1024;

/// Open without waiting for a FIFO peer, then inspect the actual descriptor.
/// A pathname check alone would race with replacement before open.
pub fn open_regular_file(path: &Path) -> Result<File> {
    let fd = open(
        path,
        OFlags::RDONLY | OFlags::NONBLOCK | OFlags::CLOEXEC | OFlags::NOCTTY,
        Mode::empty(),
    )
    .map_err(|e| format!("open {}: {e}", path.display()))?;
    let metadata = fstat(&fd).map_err(|e| format!("stat {}: {e}", path.display()))?;
    if FileType::from_raw_mode(metadata.st_mode) != FileType::RegularFile {
        return Err(format!("input {} must be a regular file", path.display()));
    }
    Ok(File::from(fd))
}

/// Limit bytes actually read, including when a regular input grows after stat.
pub fn read_regular_file(path: &Path, max_bytes: u64) -> Result<Vec<u8>> {
    let file = open_regular_file(path)?;
    let too_large = || format!("input {} exceeds {max_bytes} bytes", path.display());
    if file.metadata().map_err(|e| e.to_string())?.len() > max_bytes {
        return Err(too_large());
    }
    let limit = max_bytes
        .checked_add(1)
        .ok_or("input byte limit overflow")?;
    let mut bytes = Vec::new();
    file.take(limit)
        .read_to_end(&mut bytes)
        .map_err(|e| format!("read {}: {e}", path.display()))?;
    if bytes.len() as u64 > max_bytes {
        return Err(too_large());
    }
    Ok(bytes)
}

/// Atomic replacement must not destroy a FIFO, directory, or device entry.
pub fn validate_output_file(path: &Path) -> Result<()> {
    match path.metadata() {
        Ok(metadata) if metadata.is_file() => Ok(()),
        Ok(_) => Err(format!("output {} must be a regular file", path.display())),
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(()),
        Err(error) => Err(format!("output {}: {error}", path.display())),
    }
}
