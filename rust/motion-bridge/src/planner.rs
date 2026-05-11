//! Planner thread core.
//!
//! Receives `PlannerMsg` messages and runs the streaming-shaper pipeline:
//! every `PlannerMsg::Move(m)` triggers `ShaperState::append_and_replan` over
//! the un-committed tail, followed by `ShaperState::emit_committed` to
//! dispatch any newly-eligible shaped output.
//!
//! Phase 3 Task 3.3 replaced the Phase-1 buffered-window shim (which called
//! `trajectory::shape_batch` on a `Vec<CubicSegment>` window and staged the
//! result through `ShaperState::pending_dispatch`) with the streaming-native
//! path. Two consequences for tests / callers:
//!
//! - The `window_capacity` field on [`crate::config::PlannerConfig`] is no
//!   longer consulted — replan + emit happens per `submit_move`. The field
//!   is retained on the config (the PyO3 surface still accepts it) for
//!   forward compatibility with Phase 6 print-time rectification; it is
//!   silently ignored on the streaming hot path.
//! - `emit_committed` only dispatches up to `t_decel_start − max_h` —
//!   the trailing decel-to-zero region of the most recent replan is held
//!   speculatively until either a follow-on move arrives (in which case the
//!   replan re-anchors the decel-to-zero point further out and more of the
//!   prior plan becomes committed) or Phase 4's quiescence commit adopts
//!   the planned decel as the actual trajectory. Tests that submit a single
//!   short move and immediately flush will therefore see strictly less
//!   shaped output than the Phase-1 shim produced. For now we keep those
//!   assertions only on long-enough moves; the dwell / commit handler in
//!   Phase 4 will let short moves dispatch the full plan on flush.

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
use trajectory::{AxisShaper, EHalo, RequiredShaper, ShaperConfig, ShapedSegment};

// ---------------------------------------------------------------------------
// Quiescence-commit timer
// ---------------------------------------------------------------------------

/// Inter-move quiescence threshold for the single-timer commit model
/// (spec §3.5). If no `PlannerMsg::Move` arrives within this window after
/// the most recent append, the planner thread calls
/// [`ShaperState::commit_decel_to_zero`] to dispatch the held-back trailing
/// decel-to-zero ramp. 50 ms is the spec's proposed default; open-question 1
/// in §6 reserves empirical calibration on Trident for Phase 7.
const T_COMMIT: Duration = Duration::from_millis(50);

/// Sentinel "long" timeout used when there is no held-back tail to commit.
/// We still call `recv_timeout` (rather than `recv`) so the loop reaches a
/// single uniform message-handling site; an effectively-forever bound keeps
/// the wake-up overhead negligible while preserving cancellation semantics
/// on `Shutdown` (the channel close fires `Disconnected`, not `Timeout`).
const T_IDLE: Duration = Duration::from_secs(3600);

// ---------------------------------------------------------------------------
// Messages
// ---------------------------------------------------------------------------

/// Clock-sync bias snapshot — the host-side `(freq, offset, last_clock)` triple
/// the bridge's per-MCU clock-sync driver maintains. Phase 5 Task 5.1 adds the
/// `PlannerMsg::ClockSyncRearm` variant that will carry this through to the
/// planner thread once Task 5.4 wires the host-side detection / dispatch.
///
/// The triple matches `kalico_host_rt::router_state::Router::set_clock_est_from_sample`'s
/// argument shape (freq Hz, offset s, last_clock ticks) so callers reading
/// existing clock-sync state can forward it verbatim. Task 5.4 will refine the
/// type if a narrower shape suffices; for now we preserve the full sample
/// payload so the planner-side handler has everything the dispatch layer needs.
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
        notify: Sender<()>,
    },
    UpdateLimits(PlannerLimits),
    UpdateShaper(ShaperConfig),
    Shutdown,
    /// Phase 5 Task 5.1 — `kalico_stream_open` arrived. Reset `ShaperState`
    /// to the supplied home position before processing any further moves.
    /// Wired now (see `run_loop`'s match arm).
    KalicoStreamOpen { home_pos: [f64; 4] },
    /// Phase 5 Task 5.1 — homing / `SET_KINEMATIC_POSITION` succeeded.
    /// Reset `ShaperState` to the supplied home position. Wired now.
    Homing { home_pos: [f64; 4] },
    /// Phase 5 Task 5.1 — engine `Underrun` fault detected. Host-side
    /// detection lands in Task 5.2; for now this variant exists so callers
    /// can be wired against a stable enum, but the run-loop handler logs a
    /// warning and drops the message. Task 5.2 will replace the placeholder
    /// with the real reset-to-recovered-position path.
    Underrun { recovered_pos: [f64; 4] },
    /// Phase 5 Task 5.1 — engine `force_idle` detected. Same handling as
    /// `Underrun` (placeholder until Task 5.2 wires the recovery path).
    ForceIdle { recovered_pos: [f64; 4] },
    /// Phase 5 Task 5.1 — clock-sync re-arm event. Task 5.4 will wire the
    /// real handler (drain pending shaped output under the old bias, then
    /// update the bias for future dispatches per spec §3.7). For now the
    /// run-loop logs a warning and drops the message.
    ClockSyncRearm { new_bias: ClockBias },
}

// ---------------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------------

#[derive(Debug)]
pub enum PlannerError {
    Shape(trajectory::ShapeError),
    ChannelClosed,
    /// Dispatch callback (e.g. wire push) returned an error.
    Dispatch(String),
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
    /// Monotonic counter incremented every time the planner thread's
    /// quiescence-commit timer expires and calls
    /// [`ShaperState::commit_decel_to_zero`]. Phase 4 Task 4.1 uses this
    /// purely as a wiring-confidence signal for tests; Phase 4 Task 4.3
    /// keeps the counter live (it remains a cheap observability hook on
    /// the timer integration point).
    commit_fire_count: Arc<AtomicU32>,
}

impl PlannerHandle {
    pub fn spawn(
        config: PlannerConfig,
        dispatch: Arc<dyn Fn(&ShapedSegment) -> Result<(), String> + Send + Sync>,
    ) -> Self {
        let (tx, rx) = unbounded();
        let error = Arc::new(Mutex::new(None));
        let last_move_time_bits = Arc::new(AtomicU64::new(0u64));
        let commit_fire_count = Arc::new(AtomicU32::new(0));

        // Streaming-native state. `ShaperState` owns the per-axis queues +
        // un-committed tail; the per-iteration `ReplanContext` /
        // `EmitContext` are rebuilt on every shaper/limits update so live
        // config changes take effect on the next `submit_move`.
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
        let mut guard = self.error.lock().unwrap();
        if let Some(e) = guard.take() {
            return Err(e);
        }
        Ok(())
    }

    pub fn submit_move(&self, m: ClassifiedMove) -> Result<(), PlannerError> {
        self.check_error()?;
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
            Ok(()) => self.check_error(),
            Err(_) => {
                // Sender dropped: either pipeline error trashed pending_flush,
                // or thread exited. Surface the stored error if present.
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

    /// Phase 5 Task 5.1 — `kalico_stream_open` entry point. Resets the
    /// planner's `ShaperState` to the supplied home position. Caller-side
    /// hook for the klippy bridge's first connect / stream-open handshake
    /// (the spec §3.7 lifecycle row "kalico_stream_open, homing,
    /// SET_KINEMATIC_POSITION").
    ///
    /// Fire-and-forget: the channel send returns immediately; the planner
    /// thread will apply the reset on the next message-dispatch tick. No
    /// barrier semantics — callers that need to know the reset has applied
    /// should issue a subsequent `flush()` (which is a sync barrier).
    pub fn kalico_stream_open(&self, home_pos: [f64; 4]) -> Result<(), PlannerError> {
        self.sender
            .send(PlannerMsg::KalicoStreamOpen { home_pos })
            .map_err(|_| PlannerError::ChannelClosed)
    }

    /// Phase 5 Task 5.1 — homing / `SET_KINEMATIC_POSITION` entry point.
    /// Same shape and semantics as [`Self::kalico_stream_open`]; named
    /// differently to make the call-site intent legible (the bridge wires
    /// these to distinct klippy hooks even though the planner's response is
    /// identical — a position re-anchor).
    pub fn homing(&self, home_pos: [f64; 4]) -> Result<(), PlannerError> {
        self.sender
            .send(PlannerMsg::Homing { home_pos })
            .map_err(|_| PlannerError::ChannelClosed)
    }

    /// **Phase 5 Task 5.2** — fire a host-derived `Underrun` recovery into
    /// the planner. Resets `ShaperState` to `recovered_pos` (the last MCU-
    /// confirmed position the bridge could derive from the dispatched
    /// curve pool keyed by `current_segment_id`) before processing any
    /// further moves. Fire-and-forget — no barrier semantics, mirrors
    /// [`Self::kalico_stream_open`].
    ///
    /// Wire-up status: the planner-side handler is wired (the run-loop's
    /// `PlannerMsg::Underrun` arm calls `state.reset(recovered_pos)`).
    /// Bridge-side detection — the `StatusEvent` → `PlannerMsg::Underrun`
    /// routing — is **deferred**. The bridge's `take_runtime_event`
    /// surfaces `Fault` events directly to klippy today; the klippy-side
    /// reconnect path is the load-bearing recovery handle (Task 5.5).
    /// Once we want the bridge to recover *without* a klippy round-trip,
    /// the bridge's fault handler will gain a `PlannerHandle.underrun(...)`
    /// call here. This method exists so that call-site is wirable now.
    pub fn underrun(&self, recovered_pos: [f64; 4]) -> Result<(), PlannerError> {
        self.sender
            .send(PlannerMsg::Underrun { recovered_pos })
            .map_err(|_| PlannerError::ChannelClosed)
    }

    /// **Phase 5 Task 5.2** — fire a host-derived `ForceIdle` recovery
    /// into the planner. Same shape and semantics as [`Self::underrun`]
    /// (the run-loop collapses both arms into the same
    /// `state.reset(recovered_pos)` call).
    pub fn force_idle(&self, recovered_pos: [f64; 4]) -> Result<(), PlannerError> {
        self.sender
            .send(PlannerMsg::ForceIdle { recovered_pos })
            .map_err(|_| PlannerError::ChannelClosed)
    }

    /// **Phase 5 Task 5.4** — fire a clock-sync re-arm into the planner.
    /// The planner-side handler synchronously drains any held-back
    /// shaped output (via `commit_decel_to_zero`) so dispatched samples
    /// land under the old bias *before* the bias swap takes effect.
    ///
    /// Wire-up status: the planner-side handler is wired (see the
    /// run-loop's `PlannerMsg::ClockSyncRearm` arm — it calls
    /// `run_commit_and_dispatch` on the held-back tail). The bias-swap
    /// itself happens on the bridge thread that owns the `Router`
    /// (`spawn_periodic_clock_sync`'s `set_clock_est_from_sample` call)
    /// — see this method's comment in `bridge.rs` for the
    /// ordering-contract follow-up. This method is the entry point the
    /// bridge will call **before** swapping the bias when we wire the
    /// pre-swap barrier.
    pub fn clock_sync_rearm(&self, new_bias: ClockBias) -> Result<(), PlannerError> {
        self.sender
            .send(PlannerMsg::ClockSyncRearm { new_bias })
            .map_err(|_| PlannerError::ChannelClosed)
    }

    /// Snapshot of the current "last move time" (cumulative print_time, seconds).
    pub fn last_move_time(&self) -> f64 {
        f64::from_bits(self.last_move_time_bits.load(Ordering::Acquire))
    }

    /// Number of times the quiescence-commit timer has fired on the planner
    /// thread (i.e., `recv_timeout(T_COMMIT − elapsed)` returned
    /// `RecvTimeoutError::Timeout` and the run-loop invoked
    /// [`ShaperState::commit_decel_to_zero`]). Wired by Phase 4 Task 4.1;
    /// the underlying handler became real in Task 4.2 (the run-loop now
    /// dispatches the held-back trailing decel-to-zero on timer fire). The
    /// counter remains a cheap observability hook on the timer integration
    /// point.
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

fn store_print_time(bits: &AtomicU64, t: f64) {
    bits.store(t.to_bits(), Ordering::Release);
}

/// Drive `ShaperState::commit_decel_to_zero` to completion: shape the
/// held-back tail, dispatch each segment, and book-keep print-time +
/// the commit counter. Shared by the quiescence-timer (`T_commit` fire)
/// and `PlannerMsg::Flush` paths so both routes through commit have
/// byte-identical accounting behaviour. Phase 4 Task 4.3 added the
/// `Flush` caller; Task 4.2 added the timer caller.
///
/// Returns `true` if commit succeeded (even if zero segments were
/// drained — the handler is idempotent for an already-fully-committed
/// queue). Returns `false` if a pipeline error was stored; callers
/// should not advance further state in that case.
fn run_commit_and_dispatch(
    state: &mut ShaperState,
    thread_state: &PlannerThreadState,
    dispatch: &Arc<dyn Fn(&ShapedSegment) -> Result<(), String> + Send + Sync>,
    error: &Arc<Mutex<Option<PlannerError>>>,
    last_move_time_bits: &AtomicU64,
    commit_fire_count: &AtomicU32,
    print_time: &mut f64,
) -> bool {
    let drained = match state.commit_decel_to_zero(&thread_state.emit_ctx()) {
        Ok(out) => out,
        Err(e) => {
            *error.lock().unwrap() = Some(PlannerError::Shape(e));
            return false;
        }
    };
    let batch_dur: f64 = drained.iter().map(|s| s.t_end - s.t_start).sum();
    *print_time += batch_dur;
    store_print_time(last_move_time_bits, *print_time);
    for s in &drained {
        if let Err(detail) = dispatch(s) {
            *error.lock().unwrap() = Some(PlannerError::Dispatch(detail));
            break;
        }
    }
    commit_fire_count.fetch_add(1, Ordering::AcqRel);
    true
}

/// Owned per-thread context buffers consumed by `ShaperState::append_and_replan`
/// and `ShaperState::emit_committed`. `EmitContext` borrows from the kernel
/// array + halo list, so we keep both owned on this struct and reborrow on
/// every call. Rebuilt on `UpdateShaper` so live shaper-config changes
/// propagate to the next `submit_move`.
struct PlannerThreadState {
    emit_kernels: [Option<PiecewisePolynomialKernel<f64>>; 4],
    /// Streaming `emit_committed` never has E-only gaps (the extruder
    /// follows shaped XY arc-length on coupled moves; independent-E moves
    /// don't reach the streaming path today). Retained as an empty slot so
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
    dispatch: Arc<dyn Fn(&ShapedSegment) -> Result<(), String> + Send + Sync>,
    error: Arc<Mutex<Option<PlannerError>>>,
    last_move_time_bits: Arc<AtomicU64>,
    commit_fire_count: Arc<AtomicU32>,
) {
    let mut print_time: f64 = 0.0;
    let mut thread_state = PlannerThreadState::build(&config);

    // Phase 4 Task 4.1 — single-timer quiescence-commit state. `Some(t)`
    // means a real append landed at `t` and the loop should call
    // `commit_decel_to_zero` if no follow-on message arrives within
    // `T_COMMIT − t.elapsed()`. `None` means the queue is fully quiesced
    // (already committed) or no append has happened yet; the loop sleeps
    // on the long sentinel until a new `Move` arrives. Task 4.1 uses
    // `is_some()` as the proxy for "held-back tail exists" per the task
    // spec; Task 4.2 will refine to a precise `t_dispatched <
    // t_decel_start − max_h` check on `ShaperState` once the real
    // `commit_decel_to_zero` semantics land.
    let mut last_append_time: Option<Instant> = None;

    loop {
        // Compute the next timeout. The held-back-tail proxy is "an append
        // happened and we haven't committed since" (`last_append_time.is_some()`);
        // when there is no held-back tail, fall through to the long
        // sentinel so the receiver is effectively a blocking `recv` while
        // still cancellable on channel close.
        let next_timeout = match last_append_time {
            Some(t) => T_COMMIT.checked_sub(t.elapsed()).unwrap_or(Duration::ZERO),
            None => T_IDLE,
        };

        let msg = match rx.recv_timeout(next_timeout) {
            Ok(m) => m,
            Err(RecvTimeoutError::Timeout) => {
                // `T_commit` elapsed without a follow-on message. Task 4.2
                // shipped the real body of `commit_decel_to_zero`: shape
                // and dispatch the held-back trailing region
                // `[t_dispatched, t_appended]` (including the terminal
                // decel-to-zero ramp `emit_committed` deliberately holds
                // back). This branch dispatches those segments the same
                // way the `Move` arm dispatches `emit_committed` output —
                // print-time accounting + per-segment dispatch + commit
                // counter increment, all factored into
                // `run_commit_and_dispatch` (shared with the `Flush` arm
                // wired by Task 4.3). Clearing `last_append_time` disarms
                // the timer until the next `Move` arrives.
                let _ok = run_commit_and_dispatch(
                    &mut state,
                    &thread_state,
                    &dispatch,
                    &error,
                    &last_move_time_bits,
                    &commit_fire_count,
                    &mut print_time,
                );
                last_append_time = None;
                continue;
            }
            Err(RecvTimeoutError::Disconnected) => return,
        };

        match msg {
            PlannerMsg::Move(m) => {
                // Streaming-native: replan the un-committed tail with the
                // new move appended, then emit anything newly eligible up
                // to `t_decel_start − max_h`.
                if let Err(e) = state.append_and_replan(m.segment, &thread_state.replan_ctx) {
                    *error.lock().unwrap() = Some(PlannerError::Shape(e));
                    continue;
                }
                let drained = match state.emit_committed(&thread_state.emit_ctx()) {
                    Ok(out) => out,
                    Err(e) => {
                        *error.lock().unwrap() = Some(PlannerError::Shape(e));
                        continue;
                    }
                };

                // Print-time accounting: sum the eligible-region durations.
                // Phase 7 Task 7.1 will move this to caller-side advance
                // (input-time accounting, not output-time accounting) so
                // the post-decel speculative tail also contributes; for
                // now we preserve the pre-streaming behaviour of advancing
                // by what was actually dispatched this round.
                let batch_dur: f64 = drained.iter().map(|s| s.t_end - s.t_start).sum();
                print_time += batch_dur;
                store_print_time(&last_move_time_bits, print_time);

                for s in &drained {
                    if let Err(detail) = dispatch(s) {
                        *error.lock().unwrap() = Some(PlannerError::Dispatch(detail));
                        break;
                    }
                }

                // Arm / re-arm the quiescence-commit timer. Setting
                // `last_append_time = Some(Instant::now())` on every
                // successful append (even when `emit_committed` produced
                // nothing this round) is what makes the timer the single
                // "did the user stop submitting moves?" signal.
                last_append_time = Some(Instant::now());
            }

            PlannerMsg::Flush { notify } => {
                // Phase 4 Task 4.3 — `Flush` collapses `T_commit` → now
                // (spec §3.4 lifecycle row). The streaming-native model
                // holds the trailing decel-to-zero of the most recent
                // `Move` speculatively until either the quiescence timer
                // fires or a follow-on `Move` re-anchors the decel
                // further out. `wait_moves` / `M400` / homing barriers
                // need to block until *all* submitted motion is on the
                // wire — including that held-back tail — so `Flush`
                // synchronously invokes `commit_decel_to_zero` and
                // dispatches the drained segments before notifying the
                // waiter.
                //
                // Guarding on `last_append_time.is_some()` keeps the
                // arm a no-op (modulo the notify) when the queue is
                // already fully committed (every prior commit cleared
                // the timer). The `_ok` ignore matches the timer arm's
                // behaviour: a pipeline error has already been stored
                // and will surface via `PlannerHandle::check_error` on
                // the caller's next API entry.
                if last_append_time.is_some() {
                    let _ok = run_commit_and_dispatch(
                        &mut state,
                        &thread_state,
                        &dispatch,
                        &error,
                        &last_move_time_bits,
                        &commit_fire_count,
                        &mut print_time,
                    );
                    last_append_time = None;
                }
                let _ = notify.send(());
            }

            PlannerMsg::Dwell { duration_s, notify } => {
                // Phase 3 preserves the pre-streaming dwell behaviour:
                // advance `print_time` by the dwell duration and unblock
                // the caller. Phase 4 ("commit decel to zero") will model
                // dwell properly by committing the planned decel-to-zero
                // before extending the timeline by `duration_s`.
                print_time += duration_s;
                store_print_time(&last_move_time_bits, print_time);
                let _ = notify.send(());
            }

            PlannerMsg::UpdateLimits(l) => {
                config.limits = l;
                thread_state.rebuild(&config);
            }

            PlannerMsg::UpdateShaper(s) => {
                // **Phase 5 Task 5.3 — cross-axis barrier on
                // UpdateShaper.** Spec §3.7 ("Drain any held-back shaped
                // output on the affected axis to wire (use old kernel),
                // then swap kernel. Subsequent plans use new kernel."):
                // we must dispatch any uncommitted shaped output under
                // the **old** kernels before swapping in the new ones.
                // Otherwise the trailing tail of the prior trajectory
                // would be shaped by a kernel it was never planned
                // against — producing exactly the post-shape `|ẍ_shaped|`
                // overshoot β-medium was supposed to prevent.
                //
                // The drain uses the run-loop's existing commit path
                // (`run_commit_and_dispatch`, shared with `Flush` and
                // the quiescence-timer fire). The post-drain
                // `last_append_time = None` matches the other two
                // call-sites' invariant.
                //
                // The handler then rebuilds the kernels / replan
                // context on the updated config and **also** rebuilds
                // the `ShaperState` so the per-axis queues are
                // re-seeded with the new `h` values (the kernel
                // half-support drives the seed's left-pad span — see
                // `build_axis_queue`). The re-seed loses the
                // committed-history left-pad for the next move's
                // convolution, but that move's `append_and_replan`
                // starts from `v = 0` (the drained queue terminus) so
                // the prior history is moot. Spec §3.7 frames this as
                // a single per-axis event but the cross-axis barrier
                // (we drain *any* axis with held output, not just the
                // changed one) is the simplest correct discipline —
                // the wire format `UpdateShaper(ShaperConfig)` is
                // multi-axis already so we can't single-axis the
                // drain on the receive side without re-shaping the
                // message.
                if last_append_time.is_some() {
                    let _ok = run_commit_and_dispatch(
                        &mut state,
                        &thread_state,
                        &dispatch,
                        &error,
                        &last_move_time_bits,
                        &commit_fire_count,
                        &mut print_time,
                    );
                    last_append_time = None;
                }
                config.shaper = s;
                let shapers = shaper_config_to_axis_shapers(&config.shaper);
                state = ShaperState::new([0.0; 4], &shapers);
                thread_state.rebuild(&config);
            }

            PlannerMsg::KalicoStreamOpen { home_pos }
            | PlannerMsg::Homing { home_pos } => {
                // Phase 5 Task 5.1 — reset the streaming state to the new
                // home position. `ShaperState::reset` preserves per-axis
                // kernels (this is a position re-anchor, not a shaper
                // config swap — see spec §3.7), so we don't rebuild
                // `thread_state` here.
                //
                // Reset the run-loop's quiescence-timer book-keeping
                // alongside the state — a held-back tail from the prior
                // timeline is no longer meaningful and the planner is
                // back to "no append observed since reset." The commit
                // counter is observability state; we keep it monotonic
                // across resets so the test/diagnostic counters reflect
                // cumulative timer-fire history (resetting it would
                // confuse downstream consumers reading the AtomicU32).
                state.reset(home_pos);
                last_append_time = None;
            }

            PlannerMsg::Underrun { recovered_pos }
            | PlannerMsg::ForceIdle { recovered_pos } => {
                // **Phase 5 Task 5.2 — planner-side reset to recovered
                // position.** Engine `Underrun` / `force_idle` faults
                // invalidate the in-flight planner timeline: the MCU
                // has stopped executing whatever was on the wire, and
                // the host's planned-but-undispatched tail is no longer
                // valid (the engine will resume from
                // `recovered_pos`, not from the trajectory's expected
                // continuation). Spec §3.7 maps both events to
                // "Reset queues to the MCU's last-confirmed position,
                // host-derived from `current_segment_id` + dispatched
                // curve pool (no schema change)."
                //
                // `ShaperState::reset` zeroes the absolute-time line
                // and re-seeds each axis queue with the new home
                // position at `v = 0`. Per-axis kernels are
                // preserved (this is a position re-anchor, not a
                // shaper config swap). The run-loop's
                // `last_append_time` is reset alongside the state — a
                // held-back tail from the prior timeline is no longer
                // meaningful.
                //
                // **Scope note for Task 5.2 bridge integration.** The
                // bridge currently surfaces `RuntimeEvent::Fault` to
                // klippy directly via `take_runtime_event`; klippy's
                // reactor then drives the recovery (calls back through
                // the bridge to set positions, etc.). There is no
                // central bridge-side fault → planner routing today.
                // The dispatched-curve-pool → `current_segment_id`
                // → `recovered_pos` lookup is non-trivial (it needs
                // the bridge's per-MCU curve-pool state +
                // engine-side segment ID retirement tracking). We
                // therefore land the planner-side handler now — the
                // `PlannerHandle::underrun(..)` / `force_idle(..)`
                // entry points are public so the bridge can wire
                // them when the lookup machinery lands — and defer
                // the bridge-side detection to a follow-up. In the
                // interim, klippy reconnect (Task 5.5) is the
                // load-bearing recovery handle: klippy treats the
                // fault as a connection break, drops the planner,
                // re-`init_planner`s, and re-homes — which
                // constructs a fresh `ShaperState`.
                state.reset(recovered_pos);
                last_append_time = None;
            }

            PlannerMsg::ClockSyncRearm { new_bias: _ } => {
                // **Phase 5 Task 5.4 — planner-side pre-swap barrier.**
                // Spec §3.7 ("Clock-sync re-arm"): "Flush any pending
                // shaped output under the old clock bias to the wire,
                // then update the bias for future dispatches. Queue
                // content (in planner-time) is unaffected."
                //
                // The planner-thread side of the barrier is exactly
                // the same commit-and-dispatch call that `Flush` /
                // `UpdateShaper` / the quiescence timer use: drain
                // any held-back shaped output to the wire so the
                // dispatched samples land under the bias that was
                // active when they were planned. Queue content (in
                // planner-time) survives — we only drain the
                // committable region; `t_dispatched` advances; the
                // next `append_and_replan` continues seamlessly under
                // whatever bias the dispatch closure now uses.
                //
                // **The bias swap itself is bridge-thread work.** The
                // `Router::set_clock_est_from_sample` call lives on
                // the periodic clock-sync thread
                // (`spawn_periodic_clock_sync`), not the planner
                // thread — the planner doesn't carry a `Router`
                // handle today and the dispatch closure consults the
                // router on every push for `host_time_to_mcu_clock`.
                // Two orderings are possible for the full barrier:
                //
                //   1. Bridge calls `planner.clock_sync_rearm(...)`
                //      *before* it calls
                //      `Router::set_clock_est_from_sample`. The
                //      planner drains held output; the router
                //      hasn't swapped yet so the drain runs under
                //      the old bias. Then the bridge swaps.
                //
                //   2. Bridge swaps the router first, then notifies
                //      the planner. The planner's drain shapes
                //      samples to dispatch under the **new** bias.
                //      Wrong: spec requires "old bias" for the
                //      drained tail.
                //
                // Ordering (1) is what we need. The planner-side
                // handler is now ready (this arm); the bridge-side
                // wiring — replacing `spawn_periodic_clock_sync`'s
                // unconditional `set_clock_est_from_sample` with a
                // pre-barrier `clock_sync_rearm(new_bias)` call to
                // the planner — is a small follow-up that lives in
                // `bridge.rs` (Task 5.4 follow-up). The `new_bias`
                // payload is carried verbatim so the bridge can
                // apply it post-barrier without re-deriving the
                // sample. We do not consume `new_bias` here today;
                // the variant is retained so the wire-format is
                // forward-compatible once the bridge takes the
                // ordering-(1) path.
                if last_append_time.is_some() {
                    let _ok = run_commit_and_dispatch(
                        &mut state,
                        &thread_state,
                        &dispatch,
                        &error,
                        &last_move_time_bits,
                        &commit_fire_count,
                        &mut print_time,
                    );
                    last_append_time = None;
                }
            }

            PlannerMsg::Shutdown => return,
        }
    }
}

// ---------------------------------------------------------------------------
// Context construction helpers
// ---------------------------------------------------------------------------

/// Build a `ReplanContext` from the current `PlannerConfig`. Captures the
/// snapshot the next `append_and_replan` will use; rebuilt on
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
        // The streaming planner samples the actual velocity at
        // `t_dispatched` off its own `pieces` queue when available; this
        // fallback fires only when the cursor sits outside the pieces'
        // domain (e.g., the very first `append_and_replan` after a fresh
        // `ShaperState::new`). At-rest startup is the right default.
        fallback_initial_v: 0.0,
        // Phase 3 always uses the worst-case-future safety mode — the
        // trailing decel-to-zero is speculative until the next move
        // arrives or quiescence commit fires.
        safety_mode: SafetyMode::WorstCaseFuture,
    }
}

/// Materialize the per-axis `PiecewisePolynomialKernel`s that
/// `emit_committed`'s convolution consumes. E slot is `None` (extruder is
/// followed off the shaped XY arc-length, not separately shaped).
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
/// `ReplanContext.kernels` expects. The X and Y axes are always populated
/// (the `RequiredShaper` types statically guarantee this); Z is taken from
/// the optional axis enum; E is always `None` (extruder is not shaped here).
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
/// `streaming::ShaperState::new` consumes (X, Y, Z, E). The E slot is
/// `None` — extruder follows the shaped XY arc-length and is not a
/// separately shaped axis in MVP scope.
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
mod tests {
    use super::*;
    use crate::classify::classify_and_build;
    use std::sync::atomic::AtomicUsize;

    fn counting_dispatch() -> (
        Arc<dyn Fn(&ShapedSegment) -> Result<(), String> + Send + Sync>,
        Arc<AtomicUsize>,
    ) {
        let counter = Arc::new(AtomicUsize::new(0));
        let c = Arc::clone(&counter);
        let cb: Arc<dyn Fn(&ShapedSegment) -> Result<(), String> + Send + Sync> =
            Arc::new(move |_seg: &ShapedSegment| {
                c.fetch_add(1, Ordering::Relaxed);
                Ok(())
            });
        (cb, counter)
    }

    fn relaxed_config() -> PlannerConfig {
        let mut c = PlannerConfig::default();
        // Relax the C1 refit tolerance — the default 5 µm is tighter than the
        // degree-4 refit can hit on a collinear-cubic 10 mm move under the
        // test's reduced-grid budget. Task 11 covers full-tolerance runs.
        c.fit_tolerance_mm = 0.05;
        c
    }

    /// Long-move helper: a 200 mm pure-X move at 200 mm/s feedrate has a
    /// clear accel-cruise-decel shape so `t_decel_start − max_h` is well
    /// inside the move and `emit_committed` returns non-trivial output on
    /// the first submit. Used by the dispatch-non-empty smoke tests; the
    /// short-move equivalents under the Phase-1 shim would dispatch the
    /// whole move on flush, but streaming holds the trailing decel-to-zero
    /// speculatively until commit (Phase 4).
    fn long_move() -> ClassifiedMove {
        classify_and_build([0.0; 3], 200.0, 0.0, 0.0, 0.0, 200.0).unwrap()
    }

    #[test]
    fn submit_and_flush_dispatches_segments() {
        let (dispatch, counter) = counting_dispatch();
        let mut h = PlannerHandle::spawn(relaxed_config(), dispatch);

        h.submit_move(long_move()).unwrap();
        h.flush().unwrap();

        assert!(counter.load(Ordering::Relaxed) > 0, "dispatch never called");
        assert!(h.last_move_time() > 0.0, "print_time not advanced");

        h.shutdown();
    }

    #[test]
    fn shutdown_joins_cleanly() {
        let (dispatch, _counter) = counting_dispatch();
        let mut h = PlannerHandle::spawn(PlannerConfig::default(), dispatch);
        h.shutdown();
        assert!(h.join_handle.is_none());
    }

    #[test]
    fn dwell_advances_print_time_and_unblocks() {
        let (dispatch, _counter) = counting_dispatch();
        let mut h = PlannerHandle::spawn(PlannerConfig::default(), dispatch);

        h.dwell(0.25).unwrap();
        assert!((h.last_move_time() - 0.25).abs() < 1e-9);

        h.shutdown();
    }

    #[test]
    fn update_limits_processed_without_error() {
        // Smoke test: deep verification belongs in Task 11.
        let (dispatch, counter) = counting_dispatch();
        let mut h = PlannerHandle::spawn(relaxed_config(), dispatch);

        let new_limits = PlannerLimits {
            max_velocity: 200.0,
            max_accel: 2000.0,
            max_z_velocity: 10.0,
            max_z_accel: 80.0,
            square_corner_velocity: 4.0,
        };
        h.update_limits(new_limits).unwrap();

        h.submit_move(long_move()).unwrap();
        h.flush().unwrap();

        assert!(counter.load(Ordering::Relaxed) > 0);
        h.shutdown();
    }

    #[test]
    fn update_shaper_processed_without_error() {
        let (dispatch, _counter) = counting_dispatch();
        let mut h = PlannerHandle::spawn(PlannerConfig::default(), dispatch);

        let shaper = ShaperConfig {
            x: trajectory::RequiredShaper::SmoothZv { frequency_hz: 60.0 },
            y: trajectory::RequiredShaper::SmoothZv { frequency_hz: 60.0 },
            z: trajectory::AxisShaper::Passthrough,
        };
        h.update_shaper(shaper).unwrap();

        h.shutdown();
    }

    #[test]
    fn submit_triggers_replan_per_move() {
        // Under streaming, every `submit_move` runs `append_and_replan` +
        // `emit_committed` immediately (no buffer accumulation). This test
        // pins that behaviour by submitting a single long-enough move and
        // verifying dispatch fires before `flush` is called — `flush` is
        // used solely as a synchronization point.
        //
        // This is the streaming-era successor to the
        // `window_capacity_triggers_batch_flush_without_explicit_flush`
        // test, which was retired alongside the buffered-window path.
        let (dispatch, counter) = counting_dispatch();
        let mut h = PlannerHandle::spawn(relaxed_config(), dispatch);

        h.submit_move(long_move()).unwrap();
        h.flush().unwrap();

        assert!(
            counter.load(Ordering::Relaxed) > 0,
            "submit_move did not trigger per-move dispatch",
        );
        assert!(h.last_move_time() > 0.0);
        h.shutdown();
    }

    #[test]
    fn drop_without_explicit_shutdown_does_not_hang() {
        let (dispatch, _counter) = counting_dispatch();
        let h = PlannerHandle::spawn(PlannerConfig::default(), dispatch);
        drop(h); // Drop impl should send Shutdown + join.
    }
}
