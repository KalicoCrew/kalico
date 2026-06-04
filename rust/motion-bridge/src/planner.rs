//! Planner thread core.
//!
//! Receives `PlannerMsg` messages and runs the streaming-shaper pipeline:
//! every `PlannerMsg::Move(m)` triggers `ShaperState::append_and_replan` over
//! the un-committed tail, followed by `ShaperState::emit_committed` to
//! dispatch any newly-eligible shaped output.
//!
//! `window_capacity` on [`crate::config::PlannerConfig`] is retained on the
//! PyO3 surface for forward compatibility but silently ignored — replan +
//! emit happens per `submit_move`.
//!
//! `emit_committed` holds back the trailing decel-to-zero region
//! speculatively. A follow-on move re-anchors that region further out; if no
//! follow-on arrives the quiescence-commit timer dispatches it. Tests that
//! submit a single short move and immediately flush will see less shaped
//! output than a buffered-batch shaper would produce.

use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, RecvTimeoutError, Sender, unbounded};
use nurbs::algebra::PiecewisePolynomialKernel;

use crate::classify::ClassifiedMove;
use crate::config::{PlannerConfig, PlannerLimits};
use trajectory::plan_velocity::{PlanShaper, SafetyMode};
use trajectory::streaming::{EmitContext, ReplanContext, ShaperState};
use trajectory::{AxisShaper, EHalo, RequiredShaper, ShapedSegment, ShaperConfig};

// ---------------------------------------------------------------------------
// Quiescence-commit timer
// ---------------------------------------------------------------------------

/// Sentinel timeout used when there is no held-back tail. Using `recv_timeout`
/// (rather than `recv`) keeps the loop at a single message-handling site while
/// allowing clean cancellation via channel `Disconnected`.
const T_IDLE: Duration = Duration::from_secs(3600);

/// Lead time (s) inserted between planner time 0 and `host_now` at first
/// dispatch. Must equal `anchor::DEFAULT_LEAD_SECS`; duplicated here so
/// `run_loop` can compute clock-derived deadlines without depending on
/// anchor's private constant. Keep in sync with anchor.rs.
const LEAD: f64 = 0.25;

/// Safety margin (s) for the decel-commit deadline: the commit must reach the
/// MCU at least this long before the on-wire buffer drains
/// (`t_dispatched + LEAD`). Covers shaping + dispatch + pump + wire latency.
/// Starting value per spec §G; tune on hardware.
const SAFETY_MARGIN: f64 = 0.050;

// ---------------------------------------------------------------------------
// Messages
// ---------------------------------------------------------------------------

/// Clock-sync bias snapshot — the `(freq, offset, last_clock)` triple the
/// bridge's per-MCU clock-sync driver maintains. Matches
/// `kalico_host_rt::router_state::Router::set_clock_est_from_sample`'s
/// argument shape so callers can forward it verbatim.
#[derive(Debug, Clone, Copy)]
pub struct ClockBias {
    /// Estimated MCU clock frequency in Hz.
    pub freq: f64,
    /// Host-time offset (seconds) of the most recent clock-sync sample.
    pub offset_s: f64,
    /// MCU clock counter (ticks) at the time of the most recent sample.
    pub last_clock: u64,
}

#[derive(Debug)]
pub enum PlannerMsg {
    Move(ClassifiedMove),
    Dwell {
        duration_s: f64,
        notify: Sender<()>,
    },
    Flush {
        /// Planner sends the wall-clock `Instant` at which all committed
        /// motion finishes executing (`sync_instant + t_appended + LEAD`), or
        /// `None` if nothing is in flight. The caller waits until then before
        /// returning from `flush`.
        notify: Sender<Option<Instant>>,
    },
    UpdateLimits(PlannerLimits),
    UpdateShaper(ShaperConfig),
    Shutdown,
    /// Reset `ShaperState` to `home_pos` on `kalico_stream_open`.
    KalicoStreamOpen {
        home_pos: [f64; 4],
    },
    /// Reset `ShaperState` to `home_pos` on homing / `SET_KINEMATIC_POSITION`.
    Homing {
        home_pos: [f64; 4],
    },
    /// Engine `Underrun` fault. Planner-side handler is wired (resets state to
    /// `recovered_pos`); bridge-side detection is deferred — klippy reconnect
    /// is the current load-bearing recovery path.
    Underrun {
        recovered_pos: [f64; 4],
    },
    /// Engine `force_idle` fault. Same handling as `Underrun`.
    ForceIdle {
        recovered_pos: [f64; 4],
    },
    /// Clock-sync re-arm: drain held-back shaped output under the old bias
    /// before the bias swap takes effect. Planner-side handler is wired;
    /// bridge-side wiring (calling this before `set_clock_est_from_sample`)
    /// is a follow-up. The `new_bias` payload is forwarded verbatim so the
    /// bridge can apply it post-barrier without re-deriving the sample.
    ClockSyncRearm {
        new_bias: ClockBias,
    },
}

// ---------------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------------

#[derive(Debug)]
pub enum PlannerError {
    Shape(trajectory::ShapeError),
    ChannelClosed,
    /// Dispatch callback (e.g. wire push) returned an error.
    Dispatch(DispatchError),
}

impl std::fmt::Display for PlannerError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Shape(e) => write!(f, "shape pipeline error: {e}"),
            Self::ChannelClosed => write!(f, "planner channel closed"),
            Self::Dispatch(s) => write!(f, "dispatch error: {s}"),
        }
    }
}

impl std::error::Error for PlannerError {}

/// Failures the wire-side dispatch closure can surface to the planner
/// thread. Each variant carries enough structured context that future
/// telemetry / retry policy can discriminate transient (clock-sync,
/// transport hiccup) from terminal (caps-exceeded) cases without
/// string-matching.
#[derive(Debug, thiserror::Error)]
pub enum DispatchError {
    #[error(
        "motion-bridge: curve for mcu {mcu_id} exceeds caps \
         (pieces {pieces} > {max_pieces}); \
         logical-move splitting not yet implemented (Task 13 follow-up)."
    )]
    CapsExceeded {
        mcu_id: u32,
        pieces: usize,
        max_pieces: usize,
    },
    #[error("compute_ack_clock: {0}")]
    ComputeAckClock(String),
    #[error(
        "compute_ack_clock returned 0 after 5s — \
         clock-sync didn't establish for mcu {mcu_id} (mcu_h={mcu_handle:?})"
    )]
    ClockSyncTimeout {
        mcu_id: u32,
        mcu_handle: kalico_host_rt::passthrough_queue::McuHandle,
    },
    /// The `Arc<KalicoHostIo>` for the given MCU was dropped (e.g. by
    /// `attach_serial` during a FIRMWARE_RESTART) before this dispatch
    /// completed. The dispatch closure holds only a `Weak` reference and
    /// `upgrade()` returned `None`. The segment was not sent.
    #[error("MCU {0}: connection dropped during dispatch")]
    ConnectionDropped(u32),
    /// The piece pump's receiver was dropped (pump thread exited/panicked) — a dispatch send had nowhere to go.
    #[error("piece pump thread is gone; cannot dispatch")]
    PumpGone,
}

// ---------------------------------------------------------------------------
// Handle
// ---------------------------------------------------------------------------

#[allow(missing_debug_implementations)]
pub struct PlannerHandle {
    sender: Sender<PlannerMsg>,
    join_handle: Option<JoinHandle<()>>,
    /// Single-slot: a second error before the caller observes the first overwrites it.
    error: Arc<Mutex<Option<PlannerError>>>,
    /// Latest "last move time" snapshot — bits of an f64.
    last_move_time_bits: Arc<AtomicU64>,
    /// Monotonic counter incremented every time the quiescence-commit timer
    /// fires and calls `ShaperState::commit_decel_to_zero`. Cheap
    /// observability hook on the commit integration point.
    commit_fire_count: Arc<AtomicU32>,
}

impl PlannerHandle {
    pub fn spawn(
        config: PlannerConfig,
        dispatch: Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync>,
    ) -> Self {
        let (tx, rx) = unbounded();
        let error = Arc::new(Mutex::new(None));
        let last_move_time_bits = Arc::new(AtomicU64::new(0u64));
        let commit_fire_count = Arc::new(AtomicU32::new(0));

        // `ShaperState` owns the per-axis queues + un-committed tail; the
        // per-iteration `ReplanContext` / `EmitContext` are rebuilt on every
        // shaper/limits update so live config changes take effect on the next
        // `submit_move`.
        let shapers = shaper_config_to_axis_shapers(&config.shaper);
        let state = ShaperState::new([0.0; 4], &shapers);

        let error_thread = Arc::clone(&error);
        let last_thread = Arc::clone(&last_move_time_bits);
        let commit_thread = Arc::clone(&commit_fire_count);
        let join = thread::Builder::new()
            .name("kalico-planner".to_string())
            .spawn(move || {
                run_loop(
                    rx,
                    config,
                    state,
                    dispatch,
                    error_thread,
                    last_thread,
                    commit_thread,
                );
            })
            .expect("spawn planner thread");

        Self {
            sender: tx,
            join_handle: Some(join),
            error,
            last_move_time_bits,
            commit_fire_count,
        }
    }

    fn check_error(&self) -> Result<(), PlannerError> {
        let mut guard = self.error.lock().unwrap_or_else(|p| p.into_inner());
        if let Some(e) = guard.take() {
            return Err(e);
        }
        Ok(())
    }

    pub fn submit_move(&self, m: ClassifiedMove) -> Result<(), PlannerError> {
        tracing::debug!(
            subsystem = "motion",
            event = "submit_move_enter",
            nominal_s = m.nominal_duration(),
            distance_mm = m.distance_mm,
            feedrate_mm_s = m.segment.feedrate_mm_s,
            "planner.submit_move enter"
        );
        self.check_error()?;

        // Advance `last_move_time_bits` by the nominal duration before the
        // channel send so klippy can schedule inline events (M106, SET_PIN
        // AT_TIME, etc.) against the updated time immediately after
        // `submit_move` returns. The planner thread rectifies if the actual
        // shaped duration differs (see the Move arm in `run_loop`).
        //
        // CAS rather than bare load+store: the planner thread's rectification
        // path and `Dwell` arm are concurrent writers on the same atomic.
        // Klippy's normal path is single-threaded (toolhead lock), so the CAS
        // is uncontended in practice; the loop guards against call-sites that
        // bypass serialisation (e.g. `submit_homing_move`).
        //
        // Advance *before* the channel send so the atomic is fresh by the time
        // `submit_move` returns. `Release` pairs with klippy's `Acquire` load
        // in `last_move_time()`.
        let nominal = m.nominal_duration();
        advance_last_move_time(&self.last_move_time_bits, nominal);

        self.sender
            .send(PlannerMsg::Move(m))
            .map_err(|_| PlannerError::ChannelClosed)
    }

    pub fn flush(&self) -> Result<(), PlannerError> {
        self.check_error()?;
        let (tx, rx) = crossbeam_channel::bounded(1);
        self.sender
            .send(PlannerMsg::Flush { notify: tx })
            .map_err(|_| PlannerError::ChannelClosed)?;
        match rx.recv() {
            Ok(finish) => {
                if let Some(deadline) = finish {
                    let now = Instant::now();
                    if deadline > now {
                        std::thread::sleep(deadline - now);
                    }
                }
                self.check_error()
            }
            Err(_) => {
                self.check_error()?;
                Err(PlannerError::ChannelClosed)
            }
        }
    }

    pub fn dwell(&self, duration_s: f64) -> Result<(), PlannerError> {
        self.check_error()?;
        let (tx, rx) = crossbeam_channel::bounded(1);
        self.sender
            .send(PlannerMsg::Dwell {
                duration_s,
                notify: tx,
            })
            .map_err(|_| PlannerError::ChannelClosed)?;
        match rx.recv() {
            Ok(()) => self.check_error(),
            Err(_) => {
                // Sender dropped: either pipeline error trashed pending_dwell,
                // or thread exited. Surface the stored error if present.
                self.check_error()?;
                Err(PlannerError::ChannelClosed)
            }
        }
    }

    pub fn update_limits(&self, l: PlannerLimits) -> Result<(), PlannerError> {
        self.sender
            .send(PlannerMsg::UpdateLimits(l))
            .map_err(|_| PlannerError::ChannelClosed)
    }

    pub fn update_shaper(&self, s: ShaperConfig) -> Result<(), PlannerError> {
        self.sender
            .send(PlannerMsg::UpdateShaper(s))
            .map_err(|_| PlannerError::ChannelClosed)
    }

    /// Resets the planner's `ShaperState` to `home_pos` on stream open.
    /// Fire-and-forget: callers needing a sync barrier should follow with
    /// `flush()`.
    pub fn kalico_stream_open(&self, home_pos: [f64; 4]) -> Result<(), PlannerError> {
        self.sender
            .send(PlannerMsg::KalicoStreamOpen { home_pos })
            .map_err(|_| PlannerError::ChannelClosed)
    }

    /// Resets the planner's `ShaperState` to `home_pos` on homing /
    /// `SET_KINEMATIC_POSITION`. Named separately from `kalico_stream_open`
    /// because the bridge wires them to distinct klippy hooks even though the
    /// planner response is identical (a position re-anchor).
    pub fn homing(&self, home_pos: [f64; 4]) -> Result<(), PlannerError> {
        self.sender
            .send(PlannerMsg::Homing { home_pos })
            .map_err(|_| PlannerError::ChannelClosed)
    }

    /// Resets `ShaperState` to `recovered_pos` after an engine `Underrun`
    /// fault. Fire-and-forget, no barrier semantics.
    ///
    /// Bridge-side detection (the `StatusEvent` → `PlannerMsg::Underrun`
    /// routing) is deferred; klippy reconnect is the current load-bearing
    /// recovery handle. This entry point exists so the bridge can wire it
    /// once the dispatched-curve-pool lookup machinery lands.
    pub fn underrun(&self, recovered_pos: [f64; 4]) -> Result<(), PlannerError> {
        self.sender
            .send(PlannerMsg::Underrun { recovered_pos })
            .map_err(|_| PlannerError::ChannelClosed)
    }

    /// Resets `ShaperState` to `recovered_pos` after a `ForceIdle` event.
    /// Same shape and semantics as [`Self::underrun`].
    pub fn force_idle(&self, recovered_pos: [f64; 4]) -> Result<(), PlannerError> {
        self.sender
            .send(PlannerMsg::ForceIdle { recovered_pos })
            .map_err(|_| PlannerError::ChannelClosed)
    }

    /// Drains any held-back shaped output to the wire before the clock-bias
    /// swap takes effect (spec §3.7). Must be called *before*
    /// `Router::set_clock_est_from_sample` — calling after would shape the
    /// drained tail under the new bias instead of the old one. Bridge-side
    /// wiring of this ordering is a follow-up.
    pub fn clock_sync_rearm(&self, new_bias: ClockBias) -> Result<(), PlannerError> {
        self.sender
            .send(PlannerMsg::ClockSyncRearm { new_bias })
            .map_err(|_| PlannerError::ChannelClosed)
    }

    /// Snapshot of the current "last move time" (cumulative print_time, seconds).
    pub fn last_move_time(&self) -> f64 {
        f64::from_bits(self.last_move_time_bits.load(Ordering::Acquire))
    }

    /// Number of times the decel-commit deadline has fired on the planner
    /// thread (i.e. the quiescence timer expired with a held-back tail and
    /// `ShaperState::commit_decel_to_zero` was called).
    pub fn commit_fire_count(&self) -> u32 {
        self.commit_fire_count.load(Ordering::Acquire)
    }

    pub fn shutdown(&mut self) {
        let _ = self.sender.send(PlannerMsg::Shutdown);
        if let Some(h) = self.join_handle.take() {
            let _ = h.join();
        }
    }
}

impl Drop for PlannerHandle {
    fn drop(&mut self) {
        if self.join_handle.is_some() {
            self.shutdown();
        }
    }
}

// ---------------------------------------------------------------------------
// Loop
// ---------------------------------------------------------------------------

/// Below this magnitude (seconds) the `actual − nominal` delta is
/// indistinguishable from floating-point noise; 1 µs is well below the MCU
/// tick rate (~10 µs) and the host stepper-dispatch quantum.
const RECTIFICATION_TOLERANCE_S: f64 = 1e-6;

/// Retry bound for the rectification CAS loop. The only concurrent writer is
/// `submit_move`, which issues at most one CAS per call; 100 attempts cover
/// any plausible burst.
const RECTIFICATION_CAS_MAX_ATTEMPTS: usize = 100;

/// Atomically add `delta` to `last_move_time_bits` via a bounded CAS loop.
/// Used by the `Move` arm's rectification path — the caller-side
/// `submit_move` advance and this rectification are concurrent writers, so
/// `compare_exchange` is required to avoid clobbering an in-flight
/// caller-side advance from a follow-on `submit_move`.
///
/// Returns `true` if the CAS landed within `RECTIFICATION_CAS_MAX_ATTEMPTS`,
/// `false` on give-up (a debug warning is emitted).
fn rectify_last_move_time(last_move_time_bits: &AtomicU64, delta: f64) -> bool {
    for _ in 0..RECTIFICATION_CAS_MAX_ATTEMPTS {
        let cur = last_move_time_bits.load(Ordering::Acquire);
        let next = (f64::from_bits(cur) + delta).to_bits();
        if last_move_time_bits
            .compare_exchange(cur, next, Ordering::Release, Ordering::Acquire)
            .is_ok()
        {
            return true;
        }
    }
    log::debug!(
        "planner: rectification CAS contended for >{} attempts (delta {} s) — \
         giving up on this delta; the atomic will reflect the next \
         caller-side advance only",
        RECTIFICATION_CAS_MAX_ATTEMPTS,
        delta,
    );
    false
}

/// Atomically add `delta` to `last_move_time_bits` for non-`Move` arms
/// (Dwell / Flush / quiescence-commit). These also race the caller-side
/// `submit_move` advance, so the same CAS loop applies.
fn advance_last_move_time(last_move_time_bits: &AtomicU64, delta: f64) {
    rectify_last_move_time(last_move_time_bits, delta);
}

/// Drive `ShaperState::commit_decel_to_zero` to completion: shape the
/// held-back tail, dispatch each segment, and book-keep print-time + the
/// commit counter. Shared by the quiescence-timer and `PlannerMsg::Flush`
/// paths so both have byte-identical accounting behaviour.
///
/// The held-back tail's time was not covered by the nominal advance in
/// `submit_move` (the nominal is the cruise estimate; the post-cruise
/// decel-to-zero ramp is held speculatively). This advance is therefore
/// additive on top of the nominal.
///
/// Returns `true` if commit succeeded (idempotent on an already-committed
/// queue). Returns `false` if a pipeline error was stored; callers should
/// not advance further state in that case.
fn run_commit_and_dispatch(
    state: &mut ShaperState,
    thread_state: &PlannerThreadState,
    dispatch: &Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync>,
    error: &Arc<Mutex<Option<PlannerError>>>,
    last_move_time_bits: &AtomicU64,
    commit_fire_count: &AtomicU32,
) -> bool {
    let t_app_before = state.t_appended;
    let t_disp_before = state.t_dispatched;
    let commit_start = Instant::now();
    let drained = match state.commit_decel_to_zero(&thread_state.emit_ctx()) {
        Ok(out) => out,
        Err(e) => {
            tracing::error!(subsystem = "motion", event = "commit_decel_error", error = ?e, "run_commit_and_dispatch: commit_decel_to_zero failed");
            *error.lock().unwrap_or_else(|p| p.into_inner()) = Some(PlannerError::Shape(e));
            return false;
        }
    };
    let commit_us = commit_start.elapsed().as_micros();
    let batch_dur: f64 = drained.iter().map(|s| s.t_end - s.t_start).sum();
    tracing::debug!(
        subsystem = "motion",
        event = "commit_drained",
        drained = drained.len(),
        batch_dur_s = batch_dur,
        t_app_before,
        t_disp_before,
        "run_commit_and_dispatch drained"
    );
    advance_last_move_time(last_move_time_bits, batch_dur);
    for s in &drained {
        if let Err(detail) = dispatch(s) {
            tracing::error!(subsystem = "motion", event = "dispatch_error", error = ?detail, "run_commit_and_dispatch: dispatch failed");
            *error.lock().unwrap_or_else(|p| p.into_inner()) = Some(PlannerError::Dispatch(detail));
            break;
        }
    }
    commit_fire_count.fetch_add(1, Ordering::AcqRel);
    log::debug!(
        "[planner-trace] commit drained={} dur_s={:.6} commit_us={} t_app={:.6} t_disp_before={:.6} t_disp_after={:.6}",
        drained.len(),
        batch_dur,
        commit_us,
        t_app_before,
        t_disp_before,
        state.t_dispatched,
    );
    true
}

/// Per-thread context buffers consumed by `ShaperState::append_and_replan`
/// and `ShaperState::emit_committed`. `EmitContext` borrows from the kernel
/// array + halo list, so both are owned here and reborrowed on every call.
/// Rebuilt on `UpdateShaper` so live shaper-config changes propagate to the
/// next `submit_move`.
struct PlannerThreadState {
    emit_kernels: [Option<PiecewisePolynomialKernel<f64>>; 4],
    /// Empty: streaming `emit_committed` never has E-only gaps. Retained so
    /// the borrow plumbing into `EmitContext` is straight-line.
    e_halos: Vec<EHalo>,
    replan_ctx: ReplanContext,
}

impl PlannerThreadState {
    fn build(config: &PlannerConfig) -> Self {
        let emit_kernels = shaper_config_to_emit_kernels(&config.shaper);
        let replan_ctx = build_replan_context(config);
        Self {
            emit_kernels,
            e_halos: Vec::new(),
            replan_ctx,
        }
    }

    fn rebuild(&mut self, config: &PlannerConfig) {
        let next = Self::build(config);
        self.emit_kernels = next.emit_kernels;
        self.e_halos = next.e_halos;
        self.replan_ctx = next.replan_ctx;
    }

    fn emit_ctx(&self) -> EmitContext<'_> {
        EmitContext {
            kernels: &self.emit_kernels,
            e_halos: &self.e_halos,
        }
    }
}

fn run_loop(
    rx: Receiver<PlannerMsg>,
    mut config: PlannerConfig,
    mut state: ShaperState,
    dispatch: Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync>,
    error: Arc<Mutex<Option<PlannerError>>>,
    last_move_time_bits: Arc<AtomicU64>,
    commit_fire_count: Arc<AtomicU32>,
) {
    let mut thread_state = PlannerThreadState::build(&config);

    {
        let tl = config.limits.to_temporal_limits();
        log::debug!(
            "[planner-trace] startup limits v_max={:?} a_max={:?} j_max={:?} a_centripetal_max={} shaper={:?}",
            tl.v_max,
            tl.a_max,
            tl.j_max,
            tl.a_centripetal_max,
            config.shaper,
        );
    }

    let mut last_recv_time: Option<Instant> = None;
    // Re-set to `None` on every reset (stream-open, homing, Underrun,
    // ForceIdle); re-captured at the next first dispatch. Same OS monotonic
    // clock as the projection's host-time input, so `elapsed_since_sync`
    // carries no drift.
    let mut sync_instant: Option<Instant> = None;

    loop {
        // Clock-derived decel-commit deadline. The MCU starts executing
        // planner-time 0 at elapsed_since_sync == LEAD and plays forward 1:1,
        // so the on-wire buffer (ending at t_dispatched) drains at
        // elapsed_since_sync == t_dispatched + LEAD. Commit SAFETY_MARGIN
        // before that. When there is no held-back tail, sleep on T_IDLE.
        let next_timeout = if state.t_dispatched < state.t_appended - 1e-12 {
            let esc = sync_instant.map_or(0.0, |t| t.elapsed().as_secs_f64());
            let remaining = (state.t_dispatched + LEAD - SAFETY_MARGIN) - esc;
            // `try_from_secs_f64` handles NaN / infinite / negative in one
            // call — never panic the planner thread on a degenerate deadline;
            // fall back to an immediate wake (the commit guard re-checks).
            Duration::try_from_secs_f64(remaining.max(0.0)).unwrap_or(Duration::ZERO)
        } else {
            T_IDLE
        };

        let msg = match rx.recv_timeout(next_timeout) {
            Ok(m) => {
                let now = Instant::now();
                let gap_us = last_recv_time
                    .map(|t| now.saturating_duration_since(t).as_micros() as i64)
                    .unwrap_or(-1);
                last_recv_time = Some(now);
                let tag = match &m {
                    PlannerMsg::Move(_) => "Move",
                    PlannerMsg::Flush { .. } => "Flush",
                    PlannerMsg::Dwell { .. } => "Dwell",
                    PlannerMsg::UpdateLimits(_) => "UpdateLimits",
                    PlannerMsg::UpdateShaper(_) => "UpdateShaper",
                    PlannerMsg::KalicoStreamOpen { .. } => "KalicoStreamOpen",
                    PlannerMsg::Homing { .. } => "Homing",
                    PlannerMsg::Underrun { .. } => "Underrun",
                    PlannerMsg::ForceIdle { .. } => "ForceIdle",
                    PlannerMsg::ClockSyncRearm { .. } => "ClockSyncRearm",
                    PlannerMsg::Shutdown => "Shutdown",
                };
                tracing::debug!(
                    subsystem = "motion",
                    event = "planner_recv_gap",
                    tag,
                    gap_us,
                    "planner recv"
                );
                m
            }
            Err(RecvTimeoutError::Timeout) => {
                // Decel-commit deadline: the on-wire buffer is about to drain;
                // commit the held-back decel-to-zero so the MCU stops cleanly.
                // DO NOT reset the timeline — the next move self-places via
                // max(t_appended, elapsed_since_sync).
                if state.t_dispatched < state.t_appended - 1e-12 {
                    let _ok = run_commit_and_dispatch(
                        &mut state,
                        &thread_state,
                        &dispatch,
                        &error,
                        &last_move_time_bits,
                        &commit_fire_count,
                    );
                }
                continue;
            }
            Err(RecvTimeoutError::Disconnected) => return,
        };

        match msg {
            PlannerMsg::Move(m) => {
                // `submit_move` already advanced `last_move_time_bits` by
                // `m.nominal_duration()`. The planner thread runs the real
                // plan and rectifies if actual diverges from nominal by more
                // than `RECTIFICATION_TOLERANCE_S` via a CAS loop — the
                // caller-side advance is a concurrent writer on the same
                // atomic, so a blind `store` would race.
                //
                // Rectification is against `t_appended_after − t_appended_before`
                // (the appended duration), not the dispatched-this-round
                // duration. The trailing decel-to-zero is held back past
                // `t_decel_start − max_h` until a commit fires; rectifying
                // against `drained` would chronically under-rectify.
                let nominal = m.nominal_duration();
                let prior_t_appended = state.t_appended;
                let prior_t_decel = state.t_decel_start;
                let prior_t_disp = state.t_dispatched;
                let move_dist = m.distance_mm;
                let move_feed = m.segment.feedrate_mm_s;

                // Placement rule (spec §A): if the clock has run past the plan
                // tail, the toolhead is genuinely idle. Commit any held-back
                // tail first (so the MCU gets the prior decel-to-zero), then
                // insert a rest-hold advancing t_appended to "now" so the new
                // move starts at elapsed_since_sync instead of overlapping the
                // prior committed tail.
                //
                // Skip the rest-hold if commit/dispatch failed — advancing the
                // timeline then would violate advance_idle's fully-committed
                // precondition or silently drop the uncommitted tail.
                let esc = sync_instant.map_or(0.0, |t| t.elapsed().as_secs_f64());
                if esc > state.t_appended + 1e-6 {
                    let committed_ok = if state.t_dispatched < state.t_appended - 1e-12 {
                        run_commit_and_dispatch(
                            &mut state,
                            &thread_state,
                            &dispatch,
                            &error,
                            &last_move_time_bits,
                            &commit_fire_count,
                        )
                    } else {
                        true
                    };
                    if committed_ok {
                        state.advance_idle(esc);
                    }
                }

                let replan_start = Instant::now();
                if let Err(e) = state.append_and_replan(m.segment, &thread_state.replan_ctx) {
                    tracing::error!(subsystem = "motion", event = "move_arm_error", phase = "append_and_replan", error = ?e, "Move arm: append_and_replan failed");
                    *error.lock().unwrap_or_else(|p| p.into_inner()) = Some(PlannerError::Shape(e));
                    continue;
                }
                let replan_us = replan_start.elapsed().as_micros();
                let emit_start = Instant::now();
                let drained = match state.emit_committed(&thread_state.emit_ctx()) {
                    Ok(out) => out,
                    Err(e) => {
                        tracing::error!(subsystem = "motion", event = "move_arm_error", phase = "emit_committed", error = ?e, "Move arm: emit_committed failed");
                        *error.lock().unwrap_or_else(|p| p.into_inner()) =
                            Some(PlannerError::Shape(e));
                        continue;
                    }
                };
                tracing::debug!(
                    subsystem = "motion",
                    event = "move_arm_drained",
                    drained = drained.len(),
                    t_app_before = prior_t_appended,
                    t_app_after = state.t_appended,
                    t_decel_before = prior_t_decel,
                    t_decel_after = state.t_decel_start,
                    t_disp_before = prior_t_disp,
                    t_disp_after = state.t_dispatched,
                    "Move arm: drained"
                );
                let emit_us = emit_start.elapsed().as_micros();
                let drained_dur: f64 = drained.iter().map(|s| s.t_end - s.t_start).sum();
                log::debug!(
                    "[planner-trace] Move dist={:.3}mm feed={:.1} nominal_s={:.6} replan_us={} emit_us={} drained={} drained_dur_s={:.6} t_app:{:.6}->{:.6} (+{:.6}) t_decel:{:.6}->{:.6} t_disp:{:.6}->{:.6}",
                    move_dist,
                    move_feed,
                    nominal,
                    replan_us,
                    emit_us,
                    drained.len(),
                    drained_dur,
                    prior_t_appended,
                    state.t_appended,
                    state.t_appended - prior_t_appended,
                    prior_t_decel,
                    state.t_decel_start,
                    prior_t_disp,
                    state.t_dispatched,
                );

                for s in &drained {
                    if let Err(detail) = dispatch(s) {
                        *error.lock().unwrap_or_else(|p| p.into_inner()) =
                            Some(PlannerError::Dispatch(detail));
                        break;
                    }
                }

                // `sync_instant` is captured only on a non-empty dispatch, not
                // at append. Capturing before dispatch would run
                // `elapsed_since_sync` ahead of the MCU playhead and
                // spuriously trip idle-detection during continuous printing.
                // A sub-`LEAD` first move whose `emit_committed` yields nothing
                // leaves `sync_instant` as `None` until the held tail commits —
                // until then `esc` reads 0.0 and the deadline / Flush-wait
                // degrade to no-ops (both idempotent / harmless).
                if sync_instant.is_none() && !drained.is_empty() {
                    sync_instant = Some(Instant::now());
                }

                let actual = state.t_appended - prior_t_appended;
                let delta = actual - nominal;
                if delta.abs() > RECTIFICATION_TOLERANCE_S {
                    rectify_last_move_time(&last_move_time_bits, delta);
                }

                // The quiescence-commit timer is armed by the cursor guard
                // `state.t_dispatched < state.t_appended - 1e-12` in
                // `next_timeout` — no separate tracking variable needed.
            }

            PlannerMsg::Flush { notify } => {
                // Commit the held-back decel-to-zero (spec §E); idempotent
                // when already committed. No timeline reset.
                if state.t_dispatched < state.t_appended - 1e-12 {
                    let _ok = run_commit_and_dispatch(
                        &mut state,
                        &thread_state,
                        &dispatch,
                        &error,
                        &last_move_time_bits,
                        &commit_fire_count,
                    );
                }
                // The last committed piece ends at planner-time t_appended and
                // executes at wall-clock sync_instant + t_appended + LEAD.
                // `None` when nothing was ever dispatched.
                // `try_from_secs_f64` handles degenerate t_appended — never
                // panic the run-loop; a degenerate value yields ZERO and the
                // latched error (if any) surfaces via the caller's
                // `check_error()`.
                let finish = sync_instant.map(|t| {
                    t + Duration::try_from_secs_f64(state.t_appended + LEAD)
                        .unwrap_or(Duration::ZERO)
                });
                let _ = notify.send(finish);
            }

            PlannerMsg::Dwell { duration_s, notify } => {
                // The caller-side `submit_move` advance is a concurrent writer
                // on the same atomic; routing through the CAS-loop helper
                // avoids clobbering an in-flight caller-side advance from a
                // follow-on `submit_move` racing this `Dwell`.
                advance_last_move_time(&last_move_time_bits, duration_s);
                let _ = notify.send(());
            }

            PlannerMsg::UpdateLimits(l) => {
                config.limits = l;
                thread_state.rebuild(&config);
            }

            PlannerMsg::UpdateShaper(s) => {
                // Cross-axis barrier (spec §3.7): drain any held-back shaped
                // output under the *old* kernels before swapping in new ones.
                // Without this, the trailing tail of the prior trajectory would
                // be shaped by a kernel it was never planned against —
                // producing the post-shape `|ẍ_shaped|` overshoot the shaper
                // is supposed to prevent.
                //
                // After the drain, `ShaperState` is re-seeded with the new `h`
                // values (the kernel half-support drives the left-pad span —
                // see `build_axis_queue`). The prior committed-history left-pad
                // is lost, but the re-seeded queue starts from `v = 0`
                // (the drained terminus) so the prior history is moot.
                //
                // The drain covers *all* axes regardless of which changed
                // because the wire format carries a multi-axis `ShaperConfig`
                // and we can't single-axis the drain on the receive side.
                if state.t_dispatched < state.t_appended - 1e-12 {
                    let _ok = run_commit_and_dispatch(
                        &mut state,
                        &thread_state,
                        &dispatch,
                        &error,
                        &last_move_time_bits,
                        &commit_fire_count,
                    );
                }
                config.shaper = s;
                let shapers = shaper_config_to_axis_shapers(&config.shaper);
                state = ShaperState::new([0.0; 4], &shapers);
                thread_state.rebuild(&config);
            }

            PlannerMsg::KalicoStreamOpen { home_pos } | PlannerMsg::Homing { home_pos } => {
                // Position re-anchor — `ShaperState::reset` preserves per-axis
                // kernels (not a shaper config swap, so no `thread_state`
                // rebuild needed). After reset, `t_dispatched == t_appended ==
                // 0` so the cursor guard reads "no held-back tail" and no
                // spurious commit fires. `commit_fire_count` is kept monotonic
                // across resets so test/diagnostic counters reflect cumulative
                // history.
                sync_instant = None; // re-captured at next first dispatch
                state.reset(home_pos);
            }

            PlannerMsg::Underrun { recovered_pos } | PlannerMsg::ForceIdle { recovered_pos } => {
                // Engine fault: the MCU has stopped from wherever it was; the
                // planned-but-undispatched tail is no longer valid. Reset to
                // the MCU's last-confirmed position (spec §3.7: "Reset queues
                // to the MCU's last-confirmed position, host-derived from
                // `current_segment_id` + dispatched curve pool").
                // Per-axis kernels are preserved (position re-anchor, not a
                // shaper swap). After reset the cursor guard correctly reads
                // "no held-back tail."
                sync_instant = None; // re-captured at next first dispatch
                state.reset(recovered_pos);
            }

            PlannerMsg::ClockSyncRearm { new_bias: _ } => {
                // Pre-swap barrier (spec §3.7): flush held-back shaped output
                // under the old clock bias to the wire before the bias swap.
                // Ordering: this must run *before*
                // `Router::set_clock_est_from_sample`; calling after would
                // shape the drained tail under the new bias. Bridge-side
                // wiring of that ordering is a follow-up (see
                // `clock_sync_rearm` doc). `new_bias` is not consumed here —
                // the variant is forward-compatible for when the bridge takes
                // the pre-barrier path and needs to apply it post-drain.
                if state.t_dispatched < state.t_appended - 1e-12 {
                    let _ok = run_commit_and_dispatch(
                        &mut state,
                        &thread_state,
                        &dispatch,
                        &error,
                        &last_move_time_bits,
                        &commit_fire_count,
                    );
                }
            }

            PlannerMsg::Shutdown => return,
        }
    }
}

// ---------------------------------------------------------------------------
// Context construction helpers
// ---------------------------------------------------------------------------

/// Build a `ReplanContext` from the current `PlannerConfig`. Rebuilt on
/// `UpdateLimits` / `UpdateShaper`.
fn build_replan_context(config: &PlannerConfig) -> ReplanContext {
    ReplanContext {
        limits: config.limits.to_temporal_limits(),
        kernels: shaper_config_to_plan_shapers(&config.shaper),
        fit_tolerance_mm: config.fit_tolerance_mm,
        beta_max_iters: config.beta_max_iters,
        beta_convergence_ratio: config.beta_convergence_ratio,
        e_limits: config.e_limits,
        // Slicer-supplied per-segment in the full pipeline; the streaming
        // planner does not currently have a per-move plumb, so we use the
        // same default (`0.05 mm` = 50 µm) the legacy `run_pipeline` used
        // when it built `ShapeSegmentInput.trailing_junction_chord_tolerance_mm`.
        junction_chord_tolerance_mm: 0.05,
        worker_threads: config.worker_threads,
        grid_strategy: temporal::multi::GridStrategy::Adaptive {
            min_n: 20,
            max_n: 200,
            target_grid_spacing_mm: 0.5,
        },
        // Fallback fires only when the cursor sits outside the pieces' domain
        // (e.g. the very first `append_and_replan` after a fresh
        // `ShaperState::new`). At-rest startup is the right default.
        fallback_initial_v: 0.0,
        // The trailing decel-to-zero is speculative until the next move
        // arrives or quiescence commit fires — always use worst-case-future.
        safety_mode: SafetyMode::WorstCaseFuture,
    }
}

/// Materialize the per-axis `PiecewisePolynomialKernel`s that
/// `emit_committed`'s convolution consumes. E slot is `None` (extruder
/// follows the shaped XY arc-length, not separately shaped).
fn shaper_config_to_emit_kernels(
    cfg: &ShaperConfig,
) -> [Option<PiecewisePolynomialKernel<f64>>; 4] {
    [
        Some(required_to_kernel(cfg.x)),
        Some(required_to_kernel(cfg.y)),
        cfg.z.to_kernel(),
        None,
    ]
}

fn required_to_kernel(req: RequiredShaper) -> PiecewisePolynomialKernel<f64> {
    req.to_kernel()
}

/// Map the planner-side `ShaperConfig` to the `[Option<PlanShaper>; 4]` form
/// `ReplanContext.kernels` expects. X and Y are always populated (the
/// `RequiredShaper` types statically guarantee this); E is always `None`.
fn shaper_config_to_plan_shapers(cfg: &ShaperConfig) -> [Option<PlanShaper>; 4] {
    [
        Some(required_to_plan(cfg.x)),
        Some(required_to_plan(cfg.y)),
        Some(axis_to_plan(cfg.z)),
        None,
    ]
}

fn required_to_plan(req: RequiredShaper) -> PlanShaper {
    match req {
        RequiredShaper::SmoothZv { frequency_hz } => PlanShaper::SmoothZv { frequency_hz },
        RequiredShaper::SmoothMzv { frequency_hz } => PlanShaper::SmoothMzv { frequency_hz },
    }
}

fn axis_to_plan(ax: AxisShaper) -> PlanShaper {
    match ax {
        AxisShaper::SmoothZv { frequency_hz } => PlanShaper::SmoothZv { frequency_hz },
        AxisShaper::SmoothMzv { frequency_hz } => PlanShaper::SmoothMzv { frequency_hz },
        AxisShaper::Passthrough => PlanShaper::Passthrough,
    }
}

/// Convert the planner-config-level `ShaperConfig` (X/Y required + Z
/// optional) into the per-axis `[Option<AxisShaper>; 4]` array
/// `streaming::ShaperState::new` consumes (X, Y, Z, E). E slot is `None` —
/// extruder follows the shaped XY arc-length and is not separately shaped.
fn shaper_config_to_axis_shapers(cfg: &ShaperConfig) -> [Option<AxisShaper>; 4] {
    [
        Some(required_to_axis(cfg.x)),
        Some(required_to_axis(cfg.y)),
        Some(cfg.z),
        None,
    ]
}

fn required_to_axis(req: RequiredShaper) -> AxisShaper {
    match req {
        RequiredShaper::SmoothZv { frequency_hz } => AxisShaper::SmoothZv { frequency_hz },
        RequiredShaper::SmoothMzv { frequency_hz } => AxisShaper::SmoothMzv { frequency_hz },
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests;
