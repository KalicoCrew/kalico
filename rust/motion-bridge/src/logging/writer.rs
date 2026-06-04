//! Size-based rotating NDJSON writer for `host-rust.jsonl`. Mirrors the Stage 1
//! Python `JsonlSink`: uncompressed rotation (`.1`..`.N`), flush per write, a
//! periodic fsync backstop, and fsync on rotate/close. Single-threaded: owned
//! by the `tracing-appender` worker, so no locking.

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

    /// Close + fsync the current file, shift `.N-1`->`.N` ... base->`.1`, reopen.
    fn rotate(&mut self) -> io::Result<()> {
        self.file.flush()?;
        self.file.sync_all()?; // fsync before rotate so no partial tail is lost
        // Drop the oldest, then cascade.
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
        // Best-effort durable close. Errors at drop cannot be propagated; this
        // is the documented exception to fail-loudly (matches Python close()).
        let _ = self.file.flush();
        let _ = self.file.sync_all();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tmp(name: &str) -> PathBuf {
        let mut p = std::env::temp_dir();
        // Per-test unique dir; std::process::id avoids cross-test collisions.
        p.push(format!("kalico-jsonl-test-{}-{name}", std::process::id()));
        let _ = std::fs::remove_dir_all(&p);
        std::fs::create_dir_all(&p).unwrap();
        p.push("host-rust.jsonl");
        p
    }

    #[test]
    fn writes_lines_to_base_file() {
        let path = tmp("basic");
        let mut w = RotatingJsonlWriter::new(&path, 1024, 3, FSYNC_INTERVAL).unwrap();
        w.write_all(b"{\"a\":1}\n").unwrap();
        w.flush().unwrap();
        let contents = std::fs::read_to_string(&path).unwrap();
        assert_eq!(contents, "{\"a\":1}\n");
    }

    #[test]
    fn rotates_when_exceeding_max_bytes() {
        let path = tmp("rotate");
        // tiny max so the 2nd write triggers a rotation
        let mut w = RotatingJsonlWriter::new(&path, 8, 3, FSYNC_INTERVAL).unwrap();
        w.write_all(b"AAAAAAA\n").unwrap(); // 8 bytes -> fills
        w.write_all(b"BBBBBBB\n").unwrap(); // triggers rotate before write
        w.flush().unwrap();
        let base = std::fs::read_to_string(&path).unwrap();
        // rotated_path appends ".1" to full filename: host-rust.jsonl -> host-rust.jsonl.1
        let mut rotated_name = path.as_os_str().to_os_string();
        rotated_name.push(".1");
        let rotated = std::fs::read_to_string(PathBuf::from(&rotated_name)).unwrap();
        assert_eq!(base, "BBBBBBB\n");
        assert_eq!(rotated, "AAAAAAA\n");
    }

    #[test]
    fn drops_oldest_beyond_backup_count() {
        let path = tmp("cascade");
        let mut w = RotatingJsonlWriter::new(&path, 4, 2, FSYNC_INTERVAL).unwrap();
        for i in 0..5u8 {
            w.write_all(&[b'0' + i, b'\n', b'x', b'\n']).unwrap();
        }
        w.flush().unwrap();
        // backup_count=2 => base + .1 + .2 exist, no .3
        assert!(path.exists());
        let mut p1 = path.as_os_str().to_os_string();
        p1.push(".1");
        assert!(PathBuf::from(&p1).exists());
        let mut p2 = path.as_os_str().to_os_string();
        p2.push(".2");
        assert!(PathBuf::from(&p2).exists());
        let mut p3 = path.as_os_str().to_os_string();
        p3.push(".3");
        assert!(!PathBuf::from(&p3).exists());
    }
}
