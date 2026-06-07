use runtime::piece_ring::PieceEntry;
use std::collections::{BTreeMap, BTreeSet, HashMap, VecDeque};
use std::sync::Arc;
use std::sync::Weak;
use std::sync::mpsc::{Receiver, RecvError, RecvTimeoutError};
use std::time::Duration;

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug, PartialOrd, Ord)]
pub struct AxisKey {
    pub mcu_id: u32,
    pub axis: u8,
}

#[derive(Debug)]
pub struct AxisQueue {
    pub pieces: VecDeque<(PieceEntry, f64)>,
    pub pushed: u32,
    pub retired: u32,
    pub ring_depth: u32,
    pub physical_write_cursor: u32,
}

impl AxisQueue {
    pub fn new(ring_depth: u32) -> Self {
        Self {
            pieces: VecDeque::new(),
            pushed: 0,
            retired: 0,
            ring_depth,
            physical_write_cursor: 0,
        }
    }
    pub fn room(&self) -> u32 {
        let in_flight = self.pushed.wrapping_sub(self.retired);
        self.ring_depth.saturating_sub(in_flight)
    }
    pub fn advance_write_cursor(&mut self, n: u32) {
        if self.ring_depth == 0 {
            return;
        }
        self.physical_write_cursor = (self.physical_write_cursor + n) % self.ring_depth;
    }
}

#[derive(Debug)]
pub struct FramePlan {
    pub key: AxisKey,
    pub pieces: Vec<PieceEntry>,
    pub start_slot: u16,
}

#[derive(Debug)]
pub enum Schedule {
    Send(Vec<FramePlan>),
    StallFull(AxisKey),
    StallAhead(AxisKey),
    Idle,
}

/// Select the globally earliest-host-time piece across all queues, then emit
/// the same-MCU prefix as one frame batch.
///
/// ## Invariants preserved
///
/// 1. **Global gating is intentional.** A stalled MCU (StallFull / StallAhead)
///    gates issuance to all other MCUs — cross-MCU issue-side coherence requires
///    that a blocked MCU is never overtaken.
///
/// 2. **Ordering must use host time.** `start_time` values are raw MCU clock
///    ticks in per-MCU clock domains (H7: ~520 MHz / own epoch; F446: ~180 MHz /
///    own epoch). Comparing ticks across MCUs is meaningless; F446 values are
///    numerically smaller and would always win, starving the H7.  The `f64`
///    sidecar in each `(PieceEntry, f64)` pair carries the minting host time
///    (`t0 + u_start`, seconds) and is the only valid cross-queue ordering key.
#[must_use]
pub fn schedule(
    queues: &BTreeMap<AxisKey, AxisQueue>,
    max_per_frame: usize,
    horizon_of: impl Fn(u32) -> Option<u64>,
) -> Schedule {
    let mut stall_ahead_candidate: Option<AxisKey> = None;

    let head = queues
        .iter()
        .filter(|(_, q)| !q.pieces.is_empty())
        .min_by(|(ka, qa), (kb, qb)| {
            let ha = qa.pieces.front().unwrap().1;
            let hb = qb.pieces.front().unwrap().1;
            ha.total_cmp(&hb).then(ka.cmp(kb))
        });
    let (&head_key, head_q) = match head {
        None => return Schedule::Idle,
        Some(h) => h,
    };
    let (head_entry, _head_host) = head_q.pieces.front().unwrap();
    let head_start_ticks = head_entry.start_time;

    if head_q.room() == 0 {
        return Schedule::StallFull(head_key);
    }

    if let Some(horizon) = horizon_of(head_key.mcu_id) {
        if head_start_ticks > horizon {
            return Schedule::StallAhead(head_key);
        }
    }

    let mut taken: BTreeMap<AxisKey, usize> = BTreeMap::new();
    let mut maxed: BTreeSet<AxisKey> = BTreeSet::new();
    loop {
        let next = queues
            .iter()
            .filter_map(|(k, q)| {
                if maxed.contains(k) {
                    return None;
                }
                let already = taken.get(k).copied().unwrap_or(0);
                q.pieces
                    .get(already)
                    .map(|&(ref p, host)| (*k, p.start_time, host))
            })
            .min_by(|(ka, _, ha), (kb, _, hb)| ha.total_cmp(hb).then(ka.cmp(kb)));
        let (k, start_ticks, _host) = match next {
            Some(n) => n,
            None => break,
        };
        if k.mcu_id != head_key.mcu_id {
            break;
        }
        let already = taken.get(&k).copied().unwrap_or(0);
        let q = &queues[&k];
        let room = q.room() as usize;
        if already >= room || already >= max_per_frame {
            maxed.insert(k);
            continue;
        }
        if let Some(horizon) = horizon_of(k.mcu_id) {
            if start_ticks > horizon {
                if stall_ahead_candidate.is_none() {
                    stall_ahead_candidate = Some(k);
                }
                maxed.insert(k);
                continue;
            }
        }
        *taken.entry(k).or_insert(0) += 1;
    }

    if taken.is_empty() {
        if let Some(k) = stall_ahead_candidate {
            return Schedule::StallAhead(k);
        }
        return Schedule::StallFull(head_key);
    }

    let frames: Vec<FramePlan> = taken
        .into_iter()
        .filter(|(_, n)| *n > 0)
        .map(|(k, n)| FramePlan {
            key: k,
            pieces: queues[&k].pieces.iter().take(n).map(|(p, _)| *p).collect(),
            start_slot: 0,
        })
        .collect();
    debug_assert!(!frames.is_empty());
    Schedule::Send(frames)
}

#[cfg(test)]
mod sched_tests;

#[cfg(test)]
mod tests;

pub struct EnqueueMsg {
    pub key: AxisKey,
    /// Each entry pairs the `PieceEntry` with its minting host time (`t0 + u_start`, seconds).
    /// The host time is the ordering key used by `schedule()`; the raw `start_time` tick is
    /// MCU-clock-domain-specific and incomparable across MCUs.
    pub pieces: Vec<(PieceEntry, f64)>,
    pub fresh_stream: bool,
}

pub struct HeartbeatMsg {
    pub mcu_id: u32,
    pub retired_counts: Vec<u32>,
}

pub enum PumpMsg {
    Enqueue(EnqueueMsg),
    Heartbeat(HeartbeatMsg),
    Shutdown,
}

/// Error from [`PieceSink::send_frame`].
///
/// `Fatal` means the transport is permanently broken and the caller must not
/// retry — the process should abort or restart.  `Transient` means the frame
/// was not delivered but the transport may recover; the caller can back off and
/// retry.
#[derive(Debug)]
pub enum SendError {
    /// Unrecoverable transport failure (broken pipe, connection reset, peer
    /// closed).  Callers that receive this must invoke their fatal-fault action
    /// immediately; retrying will not help.
    Fatal(String),
    /// Recoverable or non-transport error (MCU rejected the frame, ring full,
    /// etc.).
    Transient(String),
}

impl std::fmt::Display for SendError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Fatal(s) => write!(f, "fatal: {s}"),
            Self::Transient(s) => write!(f, "transient: {s}"),
        }
    }
}

pub trait PieceSink: Send {
    fn send_frame(
        &self,
        key: AxisKey,
        pieces: &[PieceEntry],
        start_slot: u16,
        new_head: u32,
    ) -> Result<i32, SendError>;
}

/// Compute the tick-domain and host-domain gaps at a batch boundary.
///
/// Returns `(tick_jump_us, host_jump_us)` where negative values indicate overlap.
/// The difference between the two isolates clock-projection error from planner-intent gaps.
pub fn junction_jumps(
    first_start_ticks: u64,
    first_host: f64,
    prev_end_ticks: u64,
    prev_end_host: f64,
    approx_freq_hz: f64,
) -> (f64, f64) {
    let tick_jump_us =
        (first_start_ticks as i64 - prev_end_ticks as i64) as f64 / approx_freq_hz * 1e6;
    let host_jump_us = (first_host - prev_end_host) * 1e6;
    (tick_jump_us, host_jump_us)
}

const MAX_LEAD_SECS: f64 = 1.0;

/// Run the piece pump loop.
///
/// `on_fatal_transport` is called (at most once) when [`SendError::Fatal`] is
/// returned by the sink, indicating an unrecoverable transport failure.  The
/// production call site passes an `abort()`-based action; tests inject a
/// channel-send or a flag-set so they can assert detection without terminating
/// the process.  After the callback returns the pump loop exits.
pub fn run_pump<S, F, C, A>(
    rx: Receiver<PumpMsg>,
    sink: S,
    ring_depth_of: F,
    mcu_clock_of: C,
    on_fatal_transport: A,
) where
    S: PieceSink,
    F: Fn(AxisKey) -> u32,
    C: Fn(u32) -> Option<(u64, f64)>,
    A: Fn(AxisKey) + Send + 'static,
{
    let mut queues: BTreeMap<AxisKey, AxisQueue> = BTreeMap::new();
    let mut junction_ends: BTreeMap<AxisKey, (u64, f64)> = BTreeMap::new();
    const MAX_PER_FRAME: usize = 32;

    let apply = |msg: PumpMsg,
                 queues: &mut BTreeMap<AxisKey, AxisQueue>,
                 junction_ends: &mut BTreeMap<AxisKey, (u64, f64)>|
     -> bool {
        match msg {
            PumpMsg::Shutdown => return false,
            PumpMsg::Enqueue(EnqueueMsg {
                key,
                pieces,
                fresh_stream,
            }) => {
                if fresh_stream {
                    junction_ends.remove(&key);
                }
                if !pieces.is_empty() {
                    // Clock not yet synced — skip junction bookkeeping; µs math is
                    // meaningless without a real frequency and end_time needs it too.
                    if let Some((_ack_now, freq)) = mcu_clock_of(key.mcu_id) {
                        let (first_entry, first_host) = &pieces[0];
                        if let Some(&(prev_end_ticks, prev_end_host)) = junction_ends.get(&key) {
                            let (tick_jump_us, host_jump_us) = junction_jumps(
                                first_entry.start_time,
                                *first_host,
                                prev_end_ticks,
                                prev_end_host,
                                freq,
                            );
                            let anomalous =
                                tick_jump_us < -50.0 || (tick_jump_us - host_jump_us).abs() > 50.0;
                            if fresh_stream || !anomalous {
                                log::debug!(
                                    "[junction] key={:?} tick_jump_us={:.1} host_jump_us={:.1} fresh={}",
                                    key,
                                    tick_jump_us,
                                    host_jump_us,
                                    fresh_stream,
                                );
                            } else {
                                let reason = if tick_jump_us < -50.0 {
                                    "overlap_risk"
                                } else {
                                    "projection_divergence"
                                };
                                log::warn!(
                                    "[junction] key={:?} tick_jump_us={:.1} host_jump_us={:.1} fresh={} reason={}",
                                    key,
                                    tick_jump_us,
                                    host_jump_us,
                                    fresh_stream,
                                    reason,
                                );
                            }
                        }
                        let (last_entry, last_host) = pieces.last().unwrap();
                        #[allow(clippy::cast_possible_truncation)]
                        let last_end_ticks = last_entry.end_time(freq as f32);
                        let last_end_host = last_host + last_entry.duration as f64;
                        junction_ends.insert(key, (last_end_ticks, last_end_host));
                    }
                }
                let q = queues
                    .entry(key)
                    .or_insert_with(|| AxisQueue::new(ring_depth_of(key)));
                q.pieces.extend(pieces);
            }
            PumpMsg::Heartbeat(HeartbeatMsg {
                mcu_id,
                retired_counts,
            }) => {
                // retired_counts[i] is axis index i; same numbering as PushPieces.axis_idx in runtime_ffi.rs — do not reorder either side.
                for (axis, &c) in retired_counts.iter().enumerate() {
                    let key = AxisKey {
                        mcu_id,
                        axis: axis as u8,
                    };
                    if let Some(q) = queues.get_mut(&key) {
                        q.retired = c;
                    }
                }
            }
        }
        true
    };

    let horizon_of = |mcu_id: u32| -> Option<u64> {
        let (ack_now, freq) = mcu_clock_of(mcu_id)?;
        #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
        Some(ack_now + (MAX_LEAD_SECS * freq) as u64)
    };

    let mut holding_ahead = false;

    loop {
        // If we are holding pieces that are time-gated, poll every 50 ms so
        // the horizon sweeps forward (ack_now advances with the MCU clock).
        // Otherwise block indefinitely — a heartbeat or enqueue will wake us.
        let first = if holding_ahead {
            match rx.recv_timeout(Duration::from_millis(50)) {
                Ok(m) => Some(m),
                Err(RecvTimeoutError::Timeout) => None,
                Err(RecvTimeoutError::Disconnected) => return,
            }
        } else {
            match rx.recv() {
                Ok(m) => Some(m),
                Err(RecvError) => return,
            }
        };

        if let Some(msg) = first {
            if !apply(msg, &mut queues, &mut junction_ends) {
                return;
            }
            while let Ok(m) = rx.try_recv() {
                if !apply(m, &mut queues, &mut junction_ends) {
                    return;
                }
            }
        }

        holding_ahead = false;
        'send: loop {
            match schedule(&queues, MAX_PER_FRAME, &horizon_of) {
                Schedule::Idle => break 'send,
                Schedule::StallFull(_stall_key) => {
                    break 'send;
                }
                Schedule::StallAhead(_stall_key) => {
                    holding_ahead = true;
                    break 'send;
                }
                Schedule::Send(frames) => {
                    if frames.is_empty() {
                        break 'send;
                    }
                    for mut f in frames {
                        let n = f.pieces.len() as u32;
                        let new_head = {
                            let q = queues.get(&f.key).expect("planned key exists");
                            debug_assert!(
                                q.ring_depth <= u32::from(u16::MAX),
                                "ring_depth {} exceeds u16::MAX; start_slot cast is lossy",
                                q.ring_depth
                            );
                            f.start_slot = q.physical_write_cursor as u16;
                            q.pushed.wrapping_add(n)
                        };
                        match sink.send_frame(f.key, &f.pieces, f.start_slot, new_head) {
                            Ok(_) => {
                                let q = queues.get_mut(&f.key).expect("planned key exists");
                                for _ in 0..f.pieces.len() {
                                    q.pieces.pop_front();
                                }
                                q.pushed = q.pushed.wrapping_add(n);
                                q.advance_write_cursor(n);
                            }
                            Err(SendError::Fatal(ref e)) => {
                                log::error!(
                                    "pump send_frame FATAL transport error for {:?}: {e} \
                                     — invoking fatal-transport action",
                                    f.key
                                );
                                on_fatal_transport(f.key);
                                return;
                            }
                            Err(SendError::Transient(ref e)) => {
                                log::error!("pump send_frame failed for {:?}: {e}", f.key);
                                break 'send;
                            }
                        }
                    }
                }
            }
        }
    }
}

pub enum McuTransport {
    Serial(Weak<kalico_host_rt::host_io::KalicoHostIo>),
    EtherCat(Weak<kalico_host_rt::unix_native_conn::UnixNativeConn>),
}

impl std::fmt::Debug for McuTransport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Serial(_) => write!(f, "McuTransport::Serial"),
            Self::EtherCat(_) => write!(f, "McuTransport::EtherCat"),
        }
    }
}

pub struct WireSink {
    pub transports: HashMap<u32, McuTransport>,
    pub timeout: Duration,
    /// Live per-MCU clock frequency (Hz), queried from the router per frame.
    /// `None` while the MCU clock is not yet synced. Single source of truth for
    /// the `[transit-diag]` µs conversion — no second freq table.
    pub freq_of: Arc<dyn Fn(u32) -> Option<f64> + Send + Sync>,
}

impl WireSink {
    /// Call `PushPieces` on the transport for the given axis.
    ///
    /// Returns `Err(SendError::Fatal(...))` for EtherCAT transport errors that
    /// represent permanent connection loss (`Closed` or `Io`); all other
    /// failures map to `Err(SendError::Transient(...))`.
    ///
    /// The reader thread owns all socket reads; `WouldBlock`/`TimedOut` never
    /// surface here.  `TransportError::Timeout` (deadline exhausted on the
    /// caller's `recv_timeout`) is transient — the session may still be alive.
    fn call_push_pieces(
        &self,
        key: AxisKey,
        pieces: &[PieceEntry],
        start_slot: u16,
        new_head: u32,
    ) -> Result<kalico_protocol::messages::PushPiecesResponse, SendError> {
        use kalico_host_rt::transport::TransportError;

        let mut pieces_bytes = Vec::with_capacity(std::mem::size_of_val(pieces));
        for p in pieces {
            pieces_bytes.extend_from_slice(&p.to_le_bytes());
        }

        let msg = kalico_protocol::messages::PushPieces {
            axis_idx: key.axis,
            piece_count: pieces.len() as u8,
            start_slot,
            new_head,
            pieces_bytes,
        };
        let mut body = Vec::with_capacity(8 + std::mem::size_of_val(pieces));
        kalico_protocol::codec::Encode::encode(&msg, &mut body);

        let transport = self.transports.get(&key.mcu_id).ok_or_else(|| {
            SendError::Transient(format!(
                "WireSink: no transport for mcu_id {} (axis {}); \
                     this is a logic bug in init_planner — the axis was enqueued \
                     without registering its transport",
                key.mcu_id, key.axis
            ))
        })?;

        let resp_body = match transport {
            McuTransport::Serial(weak) => {
                let io = weak.upgrade().ok_or_else(|| {
                    SendError::Transient(format!("KalicoHostIo for mcu {} detached", key.mcu_id))
                })?;
                let (_kind, b) = io
                    .kalico_call_on_channel(
                        kalico_protocol::KALICO_CHANNEL_PIECES,
                        kalico_protocol::MessageKind::PushPieces,
                        body,
                        self.timeout,
                    )
                    .map_err(|e| {
                        SendError::Transient(format!("serial PushPieces mcu {}: {e:?}", key.mcu_id))
                    })?;
                b
            }
            McuTransport::EtherCat(weak) => {
                let conn = weak.upgrade().ok_or_else(|| {
                    // The endpoint conn was released (last strong Arc dropped):
                    // session is gone, treat as fatal so the pump exits rather
                    // than spinning on a dead axis.
                    SendError::Fatal(format!(
                        "ethercat conn for mcu {} detached (released)",
                        key.mcu_id
                    ))
                })?;
                let (_kind, b) = conn
                    .kalico_call_on_channel(
                        kalico_protocol::KALICO_CHANNEL_PIECES,
                        kalico_protocol::MessageKind::PushPieces,
                        body,
                        self.timeout,
                    )
                    .map_err(|e| {
                        if matches!(&e, TransportError::Closed | TransportError::Io(_)) {
                            SendError::Fatal(format!(
                                "ethercat PushPieces mcu {}: {e:?}",
                                key.mcu_id
                            ))
                        } else {
                            SendError::Transient(format!(
                                "ethercat PushPieces mcu {}: {e:?}",
                                key.mcu_id
                            ))
                        }
                    })?;
                b
            }
        };

        use kalico_protocol::codec::Decode as _;
        kalico_protocol::messages::PushPiecesResponse::decode(&resp_body).map_err(|e| {
            SendError::Transient(format!(
                "decode PushPiecesResponse mcu {}: {e:?}",
                key.mcu_id
            ))
        })
    }
}

impl PieceSink for WireSink {
    fn send_frame(
        &self,
        key: AxisKey,
        pieces: &[PieceEntry],
        start_slot: u16,
        new_head: u32,
    ) -> Result<i32, SendError> {
        debug_assert!(
            pieces.len() <= 255,
            "PushPieces frame exceeds u8 piece_count; schedule() must cap at MAX_PER_FRAME"
        );

        let host_front_start_time: u64 = pieces.first().map(|p| p.start_time).unwrap_or(0);

        let r = self.call_push_pieces(key, pieces, start_slot, new_head)?;

        {
            let arrival_lead_ticks = r.front_start_time as i64 - r.arrival_clock as i64;
            let approx_freq_hz = (self.freq_of)(key.mcu_id);
            let host_send_secs = {
                use std::time::{SystemTime, UNIX_EPOCH};
                SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .map(|d| d.as_secs_f64())
                    .unwrap_or(0.0)
            };
            let zero_st = host_front_start_time == 0;
            let past_arrival = arrival_lead_ticks < 0;
            // Clock not yet synced -> the µs conversion is meaningless; render
            // N/A. Alert gating uses arrival_lead_ticks (tick domain), so the
            // ALERT still fires without a frequency.
            let arrival_lead_us = approx_freq_hz
                .map(|f| format!("{:.1}", (arrival_lead_ticks as f64 / f) * 1e6))
                .unwrap_or_else(|| "N/A".to_owned());
            if zero_st || past_arrival {
                log::warn!(
                    "[transit-diag] mcu={} axis={} \
                     host_front_start_time={} mcu_front_start_time={} \
                     arrival_clock={} \
                     arrival_lead_ticks={} arrival_lead_us={} \
                     host_send_unix_secs={:.6} \
                     ALERT: {}",
                    key.mcu_id,
                    key.axis,
                    host_front_start_time,
                    r.front_start_time,
                    r.arrival_clock,
                    arrival_lead_ticks,
                    arrival_lead_us,
                    host_send_secs,
                    if zero_st && past_arrival {
                        "host_start_time=0 (clock-sync gap) AND piece in MCU past"
                    } else if zero_st {
                        "host_start_time=0 (router clock_freq=0 at dispatch — clock-sync gap)"
                    } else {
                        "piece arrived in MCU past (arrival_lead<0) — PieceStartInPast risk"
                    },
                );
            } else {
                log::info!(
                    "[transit-diag] mcu={} axis={} \
                     host_front_start_time={} mcu_front_start_time={} \
                     arrival_clock={} \
                     arrival_lead_ticks={} arrival_lead_us={} \
                     host_send_unix_secs={:.6}",
                    key.mcu_id,
                    key.axis,
                    host_front_start_time,
                    r.front_start_time,
                    r.arrival_clock,
                    arrival_lead_ticks,
                    arrival_lead_us,
                    host_send_secs,
                );
            }
        }

        if r.result != kalico_protocol::result_codes::OK {
            return Err(SendError::Transient(format!(
                "MCU rejected PushPieces (mcu {} axis {}): {}",
                key.mcu_id, key.axis, r.result
            )));
        }
        Ok(r.result)
    }
}

#[cfg(test)]
mod wire_sink_tests;
