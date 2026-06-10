use std::fs::File;
use std::io::Write;
use std::path::PathBuf;
use std::sync::mpsc::{sync_channel, Receiver, SyncSender, TrySendError};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

pub const ERR_CAPTURE_ACTIVE: i32 = -320;
pub const ERR_CAPTURE_NOT_ACTIVE: i32 = -321;
pub const ERR_CAPTURE_FILE: i32 = -322;
pub const ERR_CAPTURE_OVERFLOW: i32 = -323;
pub const ERR_CAPTURE_BAD_ARG: i32 = -324;

pub const CAPTURE_RING_CAPACITY: usize = 4096;
pub const RECORD_SIZE: usize = 31;
pub const FLAG_TORQUE_ENABLED: u8 = 1 << 0;
pub const FLAG_MOTION_ACTIVE: u8 = 1 << 1;

const WRITER_SYNC_INTERVAL: Duration = Duration::from_secs(1);
const WRITER_RECV_TIMEOUT: Duration = Duration::from_millis(100);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DriveSample {
    pub target_counts: i32,
    pub position_demand: i32,
    pub position_actual: i32,
    pub following_error: i32,
    pub torque_actual: i16,
    pub statusword: u16,
    pub error_code: u16,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CaptureRecord {
    pub cycle_index: u64,
    pub flags: u8,
    pub drive: DriveSample,
}

#[derive(Debug, Clone, PartialEq)]
pub struct CaptureConfig {
    pub path: String,
    pub started_utc: String,
    pub drive_name: String,
    pub cycle_ns: i64,
    pub counts_per_mm: f64,
    pub started_mono_ns: u64,
}

#[derive(Debug, PartialEq, Eq)]
pub struct StopOutcome {
    pub result: i32,
    pub samples: u64,
    pub overflow_cycle: Option<u64>,
}

pub fn encode_record(r: &CaptureRecord) -> [u8; RECORD_SIZE] {
    let mut b = [0u8; RECORD_SIZE];
    b[0..8].copy_from_slice(&r.cycle_index.to_le_bytes());
    b[8] = r.flags;
    b[9..13].copy_from_slice(&r.drive.target_counts.to_le_bytes());
    b[13..17].copy_from_slice(&r.drive.position_demand.to_le_bytes());
    b[17..21].copy_from_slice(&r.drive.position_actual.to_le_bytes());
    b[21..25].copy_from_slice(&r.drive.following_error.to_le_bytes());
    b[25..27].copy_from_slice(&r.drive.torque_actual.to_le_bytes());
    b[27..29].copy_from_slice(&r.drive.statusword.to_le_bytes());
    b[29..31].copy_from_slice(&r.drive.error_code.to_le_bytes());
    b
}

fn json_string_safe(s: &str) -> bool {
    s.chars()
        .all(|c| (c.is_ascii_graphic() || c == ' ') && c != '"' && c != '\\')
}

pub fn header_json(cfg: &CaptureConfig) -> String {
    format!(
        concat!(
            "{{\"version\":1,\"cycle_ns\":{},\"record_size\":{},",
            "\"started_utc\":\"{}\",\"started_mono_ns\":{},",
            "\"drives\":[{{\"name\":\"{}\",\"counts_per_mm\":{}}}],",
            "\"channels\":[",
            "{{\"name\":\"cycle_index\",\"dtype\":\"u64\",\"offset\":0}},",
            "{{\"name\":\"flags\",\"dtype\":\"u8\",\"offset\":8}},",
            "{{\"name\":\"target_counts\",\"dtype\":\"i32\",\"offset\":9}},",
            "{{\"name\":\"position_demand\",\"dtype\":\"i32\",\"offset\":13}},",
            "{{\"name\":\"position_actual\",\"dtype\":\"i32\",\"offset\":17}},",
            "{{\"name\":\"following_error\",\"dtype\":\"i32\",\"offset\":21}},",
            "{{\"name\":\"torque_actual\",\"dtype\":\"i16\",\"offset\":25}},",
            "{{\"name\":\"statusword\",\"dtype\":\"u16\",\"offset\":27}},",
            "{{\"name\":\"error_code\",\"dtype\":\"u16\",\"offset\":29}}",
            "]}}\n",
        ),
        cfg.cycle_ns,
        RECORD_SIZE,
        cfg.started_utc,
        cfg.started_mono_ns,
        cfg.drive_name,
        cfg.counts_per_mm,
    )
}

struct ActiveCapture {
    tx: SyncSender<CaptureRecord>,
    writer: JoinHandle<Result<u64, String>>,
    path: PathBuf,
    failure: Option<(u64, i32)>,
}

pub struct Capture {
    capacity: usize,
    active: Option<ActiveCapture>,
}

impl Default for Capture {
    fn default() -> Self {
        Self::new()
    }
}

impl Capture {
    pub fn new() -> Self {
        Self::with_capacity(CAPTURE_RING_CAPACITY)
    }

    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            capacity,
            active: None,
        }
    }

    pub fn is_active(&self) -> bool {
        self.active.is_some()
    }

    pub fn start(&mut self, cfg: CaptureConfig) -> i32 {
        self.start_inner(cfg, None)
    }

    #[cfg(test)]
    pub(crate) fn start_gated(&mut self, cfg: CaptureConfig, gate: Receiver<()>) -> i32 {
        self.start_inner(cfg, Some(gate))
    }

    fn start_inner(&mut self, cfg: CaptureConfig, gate: Option<Receiver<()>>) -> i32 {
        if self.active.is_some() {
            return ERR_CAPTURE_ACTIVE;
        }
        if !json_string_safe(&cfg.drive_name) || !json_string_safe(&cfg.started_utc) {
            return ERR_CAPTURE_BAD_ARG;
        }
        let path = PathBuf::from(&cfg.path);
        if let Some(parent) = path.parent() {
            if !parent.as_os_str().is_empty() && std::fs::create_dir_all(parent).is_err() {
                return ERR_CAPTURE_FILE;
            }
        }
        let file = match File::create(&path) {
            Ok(f) => f,
            Err(_) => return ERR_CAPTURE_FILE,
        };
        let header = header_json(&cfg);
        let (tx, rx) = sync_channel(self.capacity);
        let writer = std::thread::Builder::new()
            .name("capture-writer".into())
            .spawn(move || writer_thread(rx, file, header, gate))
            .expect("spawn capture writer thread");
        self.active = Some(ActiveCapture {
            tx,
            writer,
            path,
            failure: None,
        });
        0
    }

    pub fn push(&mut self, record: CaptureRecord) {
        let Some(active) = self.active.as_mut() else {
            return;
        };
        if active.failure.is_some() {
            return;
        }
        match active.tx.try_send(record) {
            Ok(()) => {}
            Err(TrySendError::Full(r)) => {
                active.failure = Some((r.cycle_index, ERR_CAPTURE_OVERFLOW));
            }
            Err(TrySendError::Disconnected(r)) => {
                active.failure = Some((r.cycle_index, ERR_CAPTURE_FILE));
            }
        }
    }

    pub fn stop(&mut self) -> StopOutcome {
        let Some(active) = self.active.take() else {
            return StopOutcome {
                result: ERR_CAPTURE_NOT_ACTIVE,
                samples: 0,
                overflow_cycle: None,
            };
        };
        drop(active.tx);
        let written = active.writer.join().expect("capture writer panicked");
        let (mut result, mut overflow_cycle) = (0i32, None);
        if let Some((cycle, code)) = active.failure {
            result = code;
            overflow_cycle = Some(cycle);
        }
        let samples = match written {
            Ok(n) => n,
            Err(_) if result == 0 => {
                result = ERR_CAPTURE_FILE;
                0
            }
            Err(_) => 0,
        };
        if result != 0 {
            let failed = active.path.with_extension("failed.scap");
            if std::fs::rename(&active.path, &failed).is_err() && result == ERR_CAPTURE_OVERFLOW {
                result = ERR_CAPTURE_FILE;
            }
        }
        StopOutcome {
            result,
            samples,
            overflow_cycle,
        }
    }
}

fn writer_thread(
    rx: Receiver<CaptureRecord>,
    mut file: File,
    header: String,
    gate: Option<Receiver<()>>,
) -> Result<u64, String> {
    file.write_all(header.as_bytes())
        .map_err(|e| format!("capture header write: {e}"))?;
    if let Some(g) = gate {
        let _ = g.recv();
    }
    let mut written = 0u64;
    let mut last_sync = Instant::now();
    loop {
        match rx.recv_timeout(WRITER_RECV_TIMEOUT) {
            Ok(r) => {
                file.write_all(&encode_record(&r))
                    .map_err(|e| format!("capture record write: {e}"))?;
                written += 1;
            }
            Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {}
            Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => break,
        }
        if last_sync.elapsed() >= WRITER_SYNC_INTERVAL {
            file.sync_data()
                .map_err(|e| format!("capture fsync: {e}"))?;
            last_sync = Instant::now();
        }
    }
    file.sync_data()
        .map_err(|e| format!("capture final fsync: {e}"))?;
    Ok(written)
}

#[cfg(test)]
mod tests;
