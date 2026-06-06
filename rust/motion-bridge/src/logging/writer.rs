use std::fs::{File, OpenOptions};
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

pub const DEFAULT_MAX_BYTES: u64 = 32 * 1024 * 1024;
pub const DEFAULT_BACKUP_COUNT: u32 = 5;
pub const FSYNC_INTERVAL: Duration = Duration::from_secs(15);

pub struct RotatingJsonlWriter {
    path: PathBuf,
    file: File,
    written: u64,
    max_bytes: u64,
    backup_count: u32,
    last_fsync: Instant,
    fsync_interval: Duration,
}

impl std::fmt::Debug for RotatingJsonlWriter {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("RotatingJsonlWriter")
            .field("path", &self.path)
            .field("written", &self.written)
            .finish_non_exhaustive()
    }
}

impl RotatingJsonlWriter {
    pub fn new(
        path: impl AsRef<Path>,
        max_bytes: u64,
        backup_count: u32,
        fsync_interval: Duration,
    ) -> io::Result<Self> {
        let path = path.as_ref().to_path_buf();
        if let Some(dir) = path.parent() {
            std::fs::create_dir_all(dir)?;
        }
        let file = OpenOptions::new().create(true).append(true).open(&path)?;
        let written = file.metadata()?.len();
        Ok(RotatingJsonlWriter {
            path,
            file,
            written,
            max_bytes,
            backup_count,
            last_fsync: Instant::now(),
            fsync_interval,
        })
    }

    fn rotated_path(&self, n: u32) -> PathBuf {
        let mut s = self.path.as_os_str().to_os_string();
        s.push(format!(".{n}"));
        PathBuf::from(s)
    }

    fn rotate(&mut self) -> io::Result<()> {
        self.file.flush()?;
        self.file.sync_all()?;
        let oldest = self.rotated_path(self.backup_count);
        if oldest.exists() {
            std::fs::remove_file(&oldest)?;
        }
        for n in (1..self.backup_count).rev() {
            let src = self.rotated_path(n);
            if src.exists() {
                std::fs::rename(&src, self.rotated_path(n + 1))?;
            }
        }
        if self.path.exists() {
            std::fs::rename(&self.path, self.rotated_path(1))?;
        }
        self.file = OpenOptions::new()
            .create(true)
            .append(true)
            .open(&self.path)?;
        self.written = 0;
        self.last_fsync = Instant::now();
        Ok(())
    }
}

impl Write for RotatingJsonlWriter {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        if self.written + buf.len() as u64 > self.max_bytes && self.written > 0 {
            self.rotate()?;
        }
        let n = self.file.write(buf)?;
        self.written += n as u64;
        Ok(n)
    }

    fn flush(&mut self) -> io::Result<()> {
        self.file.flush()?;
        if self.last_fsync.elapsed() >= self.fsync_interval {
            self.file.sync_all()?;
            self.last_fsync = Instant::now();
        }
        Ok(())
    }
}

impl Drop for RotatingJsonlWriter {
    fn drop(&mut self) {
        let _ = self.file.flush();
        let _ = self.file.sync_all();
    }
}

#[cfg(test)]
mod tests;
