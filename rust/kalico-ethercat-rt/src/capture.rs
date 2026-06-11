use std::fs::File;
use std::io::Write;
use std::path::PathBuf;
use std::sync::mpsc::{sync_channel, Receiver, SyncSender, TrySendError};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use crate::thread_prio::demote_to_normal_scheduling;

pub const ERR_CAPTURE_ACTIVE: i32 = -320;
pub const ERR_CAPTURE_NOT_ACTIVE: i32 = -321;
pub const ERR_CAPTURE_FILE: i32 = -322;
pub const ERR_CAPTURE_OVERFLOW: i32 = -323;
pub const ERR_CAPTURE_BAD_ARG: i32 = -324;

pub const CAPTURE_RING_CAPACITY: usize = 4096;
pub const RECORD_SIZE: usize = 31;
pub const FLAG_TORQUE_ENABLED: u8 = 1 << 0;
pub const FLAG_MOTION_ACTIVE: u8 = 1 << 1;

const OFF_CYCLE_INDEX: usize = 0;
const OFF_FLAGS: usize = 8;
const OFF_TARGET_COUNTS: usize = 9;
const OFF_POSITION_DEMAND: usize = 13;
const OFF_POSITION_ACTUAL: usize = 17;
const OFF_FOLLOWING_ERROR: usize = 21;
const OFF_TORQUE_ACTUAL: usize = 25;
const OFF_STATUSWORD: usize = 27;
const OFF_ERROR_CODE: usize = 29;

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
    b[OFF_CYCLE_INDEX..OFF_CYCLE_INDEX + 8].copy_from_slice(&r.cycle_index.to_le_bytes());
    b[OFF_FLAGS] = r.flags;
    b[OFF_TARGET_COUNTS..OFF_TARGET_COUNTS + 4]
        .copy_from_slice(&r.drive.target_counts.to_le_bytes());
    b[OFF_POSITION_DEMAND..OFF_POSITION_DEMAND + 4]
        .copy_from_slice(&r.drive.position_demand.to_le_bytes());
    b[OFF_POSITION_ACTUAL..OFF_POSITION_ACTUAL + 4]
        .copy_from_slice(&r.drive.position_actual.to_le_bytes());
    b[OFF_FOLLOWING_ERROR..OFF_FOLLOWING_ERROR + 4]
        .copy_from_slice(&r.drive.following_error.to_le_bytes());
    b[OFF_TORQUE_ACTUAL..OFF_TORQUE_ACTUAL + 2]
        .copy_from_slice(&r.drive.torque_actual.to_le_bytes());
    b[OFF_STATUSWORD..OFF_STATUSWORD + 2].copy_from_slice(&r.drive.statusword.to_le_bytes());
    b[OFF_ERROR_CODE..OFF_ERROR_CODE + 2].copy_from_slice(&r.drive.error_code.to_le_bytes());
    b
}

fn json_string_safe(s: &str) -> bool {
    s.chars()
        .all(|c| (c.is_ascii_graphic() || c == ' ') && c != '"' && c != '\\')
}

pub fn header_json(cfg: &CaptureConfig) -> String {
    let mut channels = String::new();
    for (name, dtype, offset) in [
        ("cycle_index", "u64", OFF_CYCLE_INDEX),
        ("flags", "u8", OFF_FLAGS),
        ("target_counts", "i32", OFF_TARGET_COUNTS),
        ("position_demand", "i32", OFF_POSITION_DEMAND),
        ("position_actual", "i32", OFF_POSITION_ACTUAL),
        ("following_error", "i32", OFF_FOLLOWING_ERROR),
        ("torque_actual", "i16", OFF_TORQUE_ACTUAL),
        ("statusword", "u16", OFF_STATUSWORD),
        ("error_code", "u16", OFF_ERROR_CODE),
    ] {
        if !channels.is_empty() {
            channels.push(',');
        }
        channels.push_str(&format!(
            "{{\"name\":\"{name}\",\"dtype\":\"{dtype}\",\"offset\":{offset}}}"
        ));
    }
    format!(
        concat!(
            "{{\"version\":1,\"cycle_ns\":{},\"record_size\":{},",
            "\"started_utc\":\"{}\",\"started_mono_ns\":{},",
            "\"drives\":[{{\"name\":\"{}\",\"counts_per_mm\":{}}}],",
            "\"channels\":[{}]}}\n",
        ),
        cfg.cycle_ns,
        RECORD_SIZE,
        cfg.started_utc,
        cfg.started_mono_ns,
        cfg.drive_name,
        cfg.counts_per_mm,
        channels,
    )
}

enum WriterHook {
    None,
    Gate(Receiver<()>),
    #[cfg(test)]
    FailAfterHeader(SyncSender<()>),
}

struct ActiveCapture {
    tx: SyncSender<CaptureRecord>,
    writer: JoinHandle<Result<u64, (u64, String)>>,
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
        self.start_inner(cfg, WriterHook::None)
    }

    #[cfg(test)]
    pub(crate) fn start_gated(&mut self, cfg: CaptureConfig, gate: Receiver<()>) -> i32 {
        self.start_inner(cfg, WriterHook::Gate(gate))
    }

    #[cfg(test)]
    pub(crate) fn start_writer_fails(&mut self, cfg: CaptureConfig) -> (i32, Receiver<()>) {
        let (done_tx, done_rx) = sync_channel(1);
        let result = self.start_inner(cfg, WriterHook::FailAfterHeader(done_tx));
        (result, done_rx)
    }

    fn start_inner(&mut self, cfg: CaptureConfig, hook: WriterHook) -> i32 {
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
            .spawn(move || writer_thread(rx, file, header, hook))
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

    /// Blocking stop — for the stub endpoint and tests, where there is no
    /// realtime cycle to starve.
    pub fn stop(&mut self) -> StopOutcome {
        self.stop_async().wait()
    }

    /// Finalizing a capture joins the writer through its final fsync —
    /// hundreds of ms on an SD card. The DC thread must never wait on that
    /// (drive latches ErC1.1 / AL 0x001a when cyclic frames pause), so the
    /// join runs on a demoted finalizer thread and the outcome arrives
    /// through the returned handle; poll it from the cycle.
    pub fn stop_async(&mut self) -> PendingStop {
        let (tx, rx) = sync_channel(1);
        let Some(active) = self.active.take() else {
            let _ = tx.send(StopOutcome {
                result: ERR_CAPTURE_NOT_ACTIVE,
                samples: 0,
                overflow_cycle: None,
            });
            return PendingStop { rx };
        };
        std::thread::Builder::new()
            .name("capture-finalizer".into())
            .spawn(move || {
                demote_to_normal_scheduling();
                let _ = tx.send(finalize(active));
            })
            .expect("spawn capture finalizer thread");
        PendingStop { rx }
    }
}

pub struct PendingStop {
    rx: Receiver<StopOutcome>,
}

impl PendingStop {
    pub fn try_take(&self) -> Option<StopOutcome> {
        self.rx.try_recv().ok()
    }

    pub fn wait(self) -> StopOutcome {
        self.rx.recv().expect("capture finalizer died")
    }
}

fn finalize(active: ActiveCapture) -> StopOutcome {
    drop(active.tx);
    let written = match active.writer.join() {
        Ok(w) => w,
        Err(_) => {
            eprintln!("ec-rt: capture writer panicked — aborting");
            std::process::abort();
        }
    };
    let (mut result, mut overflow_cycle) = (0i32, None);
    if let Some((cycle, code)) = active.failure {
        result = code;
        overflow_cycle = Some(cycle);
    }
    let samples = match written {
        Ok(n) => n,
        Err((n, _)) if result == 0 => {
            result = ERR_CAPTURE_FILE;
            n
        }
        Err((n, _)) => n,
    };
    if result != 0 {
        let failed = active.path.with_extension("failed.scap");
        if std::fs::rename(&active.path, &failed).is_err() {
            result = ERR_CAPTURE_FILE;
        }
    }
    StopOutcome {
        result,
        samples,
        overflow_cycle,
    }
}

fn writer_thread(
    rx: Receiver<CaptureRecord>,
    mut file: File,
    header: String,
    hook: WriterHook,
) -> Result<u64, (u64, String)> {
    demote_to_normal_scheduling();
    file.write_all(header.as_bytes())
        .map_err(|e| (0u64, format!("capture header write: {e}")))?;
    match hook {
        WriterHook::None => {}
        WriterHook::Gate(g) => {
            let _ = g.recv();
        }
        #[cfg(test)]
        WriterHook::FailAfterHeader(done_tx) => {
            let _ = done_tx.send(());
            return Err((0, "injected".into()));
        }
    }
    let mut written = 0u64;
    let mut last_sync = Instant::now();
    loop {
        match rx.recv_timeout(WRITER_RECV_TIMEOUT) {
            Ok(r) => {
                file.write_all(&encode_record(&r))
                    .map_err(|e| (written, format!("capture record write: {e}")))?;
                written += 1;
            }
            Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {}
            Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => break,
        }
        if last_sync.elapsed() >= WRITER_SYNC_INTERVAL {
            file.sync_data()
                .map_err(|e| (written, format!("capture fsync: {e}")))?;
            last_sync = Instant::now();
        }
    }
    file.sync_data()
        .map_err(|e| (written, format!("capture final fsync: {e}")))?;
    Ok(written)
}

#[cfg(test)]
mod tests;
