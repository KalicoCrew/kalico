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
use trajectory::{AxisShaper, EHalo, RequiredShaper, ShapedSegment, ShaperConfig};

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
    KalicoStreamOpen {
        home_pos: [f64; 4],
    },
    /// Phase 5 Task 5.1 — homing / `SET_KINEMATIC_POSITION` succeeded.
    /// Reset `ShaperState` to the supplied home position. Wired now.
    Homing {
        home_pos: [f64; 4],
    },
    /// Phase 5 Task 5.1 — engine `Underrun` fault detected. Host-side
    /// detection lands in Task 5.2; for now this variant exists so callers
    /// can be wired against a stable enum, but the run-loop handler logs a
    /// warning and drops the message. Task 5.2 will replace the placeholder
    /// with the real reset-to-recovered-position path.
    Underrun {
        recovered_pos: [f64; 4],
    },
    /// Phase 5 Task 5.1 — engine `force_idle` detected. Same handling as
    /// `Underrun` (placeholder until Task 5.2 wires the recovery path).
    ForceIdle {
        recovered_pos: [f64; 4],
    },
    /// Phase 5 Task 5.1 — clock-sync re-arm event. Task 5.4 will wire the
    /// real handler (drain pending shaped output under the old bias, then
    /// update the bias for future dispatches per spec §3.7). For now the
    /// run-loop logs a warning and drops the message.
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
    #[error(
        "slot pool exhausted for mcu={mcu_id} (capacity={capacity}, in_flight={in_flight}); \
         awaiting kalico_credit_freed retirement events"
    )]
    SlotPoolExhausted {
        mcu_id: u32,
        capacity: usize,
        in_flight: usize,
    },
    #[error(
        "load_curve mcu={mcu_id} slot={slot} seg_id={seg_id} axis={axis} host_gen={host_gen}: {detail}"
    )]
    LoadCurve {
        mcu_id: u32,
        slot: u16,
        seg_id: u32,
        axis: usize,
        host_gen: u16,
        detail: String,
    },
    #[error("push_segment mcu={mcu_id}: {detail}")]
    PushSegment { mcu_id: u32, detail: String },
    /// The `Arc<KalicoHostIo>` for the given MCU was dropped (e.g. by
    /// `attach_serial` during a FIRMWARE_RESTART) before this dispatch
    /// completed. The dispatch closure holds only a `Weak` reference and
    /// `upgrade()` returned `None`. The segment was not sent.
    #[error("MCU {0}: connection dropped during dispatch")]
    ConnectionDropped(u32),
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
        dispatch: Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync>,
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
        let mut guard = self.error.lock().unwrap_or_else(|p| p.into_inner());
        if let Some(e) = guard.take() {
            return Err(e);
        }
        Ok(())
    }

    pub fn submit_move(&self, m: ClassifiedMove) -> Result<(), PlannerError> {
        eprintln!(
            "[move-diag] planner.submit_move enter nominal_s={:.6} distance_mm={:.3} feed={:.1}",
            m.nominal_duration(),
            m.distance_mm,
            m.segment.feedrate_mm_s,
        );
        self.check_error()?;

        // **Phase 6 Task 7.1 — caller-side `last_move_time_bits` advance.**
        // Spec §3.8 / §4.5: klippy reads `last_move_time` (the queued-time
        // atomic) immediately after `submit_move` returns to schedule
        // inline events (`M106 S128`, `SET_PIN AT_TIME`, etc.) relative to
        // motion. Before this change, `last_move_time_bits` was only
        // advanced inside the planner thread after TOPP-RA + shaping
        // completed — so an inline event issued right after `G1 X1` saw a
        // stale value and landed against the *previous* move's print_time.
        //
        // The advance is unconditional on a successful classify and uses
        // the klippy-equivalent nominal estimate (`distance / feedrate`).
        // The planner thread later rectifies if the actual shaped
        // duration differs from the nominal (Task 7.2 in `run_loop`'s
        // `Move` arm).
        //
        // CAS rather than a bare `store(load + nominal)` so a parallel
        // planner-thread rectification or `Dwell`/commit advance — both
        // of which `fetch_add`-style adjust the same atomic — cannot be
        // clobbered by a load+store from this call. Klippy's normal
        // submission path is single-threaded (the toolhead lock serialises
        // moves), so this CAS is uncontended in practice; the loop is
        // defence-in-depth against bridge call-sites that bypass the
        // serialisation (e.g. homing's `submit_homing_move`).
        //
        // Order matters: advance **before** the channel send so the
        // post-send `submit_move` return guarantees the atomic is fresh.
        // A `Release` store pairs with klippy's `Acquire` load in
        // `last_move_time()`.
        let nominal = m.nominal_duration();
        advance_last_move_time(&self.last_move_time_bits, nominal);

        self.sender
            .send(PlannerMsg::Move(m))
            .map_err(|_| PlannerError::ChannelClosed)
    }

    pub fn flush(&self) -> Result<(), PlannerError> {
        eprintln!("[move-diag] planner.flush enter (caller wait_moves)");
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

/// Rectification threshold for the `Move`-arm `delta = actual − nominal`
/// CAS. Below this magnitude (in seconds) we skip the CAS — the delta is
/// indistinguishable from floating-point noise around the nominal estimate
/// and there is no scheduling consumer that cares about sub-microsecond
/// adjustments. 1 µs = 1e-6 s is well below the MCU tick rate (~10 µs)
/// and the host stepper-dispatch quantum, so any threshold below this is
/// observably ignored downstream.
const RECTIFICATION_TOLERANCE_S: f64 = 1e-6;

/// Bounded retry count for the rectification CAS loop. Quadratically more
/// than enough — the only writer racing this thread is `submit_move`, and
/// each call to `submit_move` issues at most one CAS attempt before
/// returning. 100 attempts cover an absurd scenario of 100 back-to-back
/// `submit_move` calls all winning the race against this thread, which is
/// physically impossible at klippy's submission cadence.
const RECTIFICATION_CAS_MAX_ATTEMPTS: usize = 100;

/// Atomically add `delta` to `last_move_time_bits` via a bounded CAS
/// loop. Used by the `Move` arm's rectification path — the caller-side
/// `submit_move` advance and this rectification are the two writers
/// touching the atomic across threads, so we use `compare_exchange` to
/// avoid clobbering an in-flight caller-side advance from a follow-on
/// `submit_move`.
///
/// Returns `true` if the CAS landed within `RECTIFICATION_CAS_MAX_ATTEMPTS`,
/// `false` if every attempt was contended (a giveup-with-warning path —
/// `log::debug!` is emitted). The current-load + add-delta + try-store
/// shape is deliberate: we want the delta applied **on top of** whatever
/// the caller-side advance currently shows, not against a stale snapshot.
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

/// Atomically add `delta` to `last_move_time_bits` for the non-`Move`
/// arms (Dwell / Flush / quiescence-commit). These advances are
/// **planner-thread-owned** event durations (dwell duration; the
/// trailing decel-to-zero dispatched on a held-back tail). They also
/// race the caller-side `submit_move` advance — a `Dwell` arm running
/// while a follow-on `submit_move` lands would see the atomic moving
/// underneath it — so the same CAS loop applies. Naming distinguishes
/// the call-sites in tracing output.
fn advance_last_move_time(last_move_time_bits: &AtomicU64, delta: f64) {
    rectify_last_move_time(last_move_time_bits, delta);
}

/// Drive `ShaperState::commit_decel_to_zero` to completion: shape the
/// held-back tail, dispatch each segment, and book-keep print-time +
/// the commit counter. Shared by the quiescence-timer (`T_commit` fire)
/// and `PlannerMsg::Flush` paths so both routes through commit have
/// byte-identical accounting behaviour. Phase 4 Task 4.3 added the
/// `Flush` caller; Task 4.2 added the timer caller.
///
/// **Phase 6 Task 7.2 — atomic-relative advance.** The local `print_time`
/// accumulator was retired: `last_move_time_bits` is now the single
/// source of truth (the `Move` arm advances it caller-side, the
/// non-`Move` arms — including this one — advance it via the CAS-loop
/// helper). The drained-batch duration is the right increment here
/// because the held-back tail's nominal portion was already published
/// by the originating `Move`'s caller-side advance; this function fires
/// **only on the commit-decel-to-zero path**, where the drained
/// segments are the trailing decel that streams `emit_committed`
/// deliberately held back past `t_decel_start − max_h`. That tail's
/// time was *not* covered by the nominal advance (the nominal is the
/// klippy-equivalent "cruise"; the post-cruise decel-to-zero ramp is
/// the planner-side overhead). So this advance is additive on top of
/// the nominal, exactly mirroring the pre-Task-7.2 semantics.
///
/// Returns `true` if commit succeeded (even if zero segments were
/// drained — the handler is idempotent for an already-fully-committed
/// queue). Returns `false` if a pipeline error was stored; callers
/// should not advance further state in that case.
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
            eprintln!("[move-diag] run_commit_and_dispatch: commit_decel_to_zero ERR {e:?}");
            *error.lock().unwrap_or_else(|p| p.into_inner()) = Some(PlannerError::Shape(e));
            return false;
        }
    };
    let commit_us = commit_start.elapsed().as_micros();
    let batch_dur: f64 = drained.iter().map(|s| s.t_end - s.t_start).sum();
    eprintln!(
        "[move-diag] run_commit_and_dispatch: drained={} batch_dur_s={:.6} t_app_before={:.6} t_disp_before={:.6}",
        drained.len(),
        batch_dur,
        t_app_before,
        t_disp_before,
    );
    advance_last_move_time(last_move_time_bits, batch_dur);
    for s in &drained {
        if let Err(detail) = dispatch(s) {
            eprintln!("[move-diag] run_commit_and_dispatch: dispatch ERR {detail:?}");
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
    let mut last_recv_time: Option<Instant> = None;

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
                eprintln!("[move-diag] planner recv {tag} gap_us={gap_us}");
                m
            }
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
                let since_arm_ms = last_append_time
                    .map(|t| t.elapsed().as_micros() as i64)
                    .unwrap_or(-1);
                eprintln!("[move-diag] planner T_commit fire since_arm_us={since_arm_ms}");
                let _ok = run_commit_and_dispatch(
                    &mut state,
                    &thread_state,
                    &dispatch,
                    &error,
                    &last_move_time_bits,
                    &commit_fire_count,
                );
                last_append_time = None;
                continue;
            }
            Err(RecvTimeoutError::Disconnected) => return,
        };

        match msg {
            PlannerMsg::Move(m) => {
                // **Phase 6 Task 7.2 — rectification.** The caller-side
                // `submit_move` has already advanced
                // `last_move_time_bits` by `m.nominal_duration()` (spec
                // §3.8). The planner thread now runs the real plan and
                // compares: `actual` = the new move's contribution to
                // `state.t_appended` (which is exactly the planner-time
                // duration the replan added to the queue) vs. `nominal`
                // (the klippy-equivalent cruise estimate). If the two
                // diverge by more than `RECTIFICATION_TOLERANCE_S` we
                // apply the delta to the atomic via a CAS loop — the
                // caller-side advance is a writer on the same atomic, so
                // a blind `store` would race against an in-flight
                // follow-on `submit_move`.
                let nominal = m.nominal_duration();
                let prior_t_appended = state.t_appended;
                let prior_t_decel = state.t_decel_start;
                let prior_t_disp = state.t_dispatched;
                let move_dist = m.distance_mm;
                let move_feed = m.segment.feedrate_mm_s;

                let replan_start = Instant::now();
                if let Err(e) = state.append_and_replan(m.segment, &thread_state.replan_ctx) {
                    eprintln!("[move-diag] Move arm: append_and_replan ERR {e:?}");
                    *error.lock().unwrap_or_else(|p| p.into_inner()) = Some(PlannerError::Shape(e));
                    continue;
                }
                let replan_us = replan_start.elapsed().as_micros();
                let emit_start = Instant::now();
                let drained = match state.emit_committed(&thread_state.emit_ctx()) {
                    Ok(out) => out,
                    Err(e) => {
                        eprintln!("[move-diag] Move arm: emit_committed ERR {e:?}");
                        *error.lock().unwrap_or_else(|p| p.into_inner()) =
                            Some(PlannerError::Shape(e));
                        continue;
                    }
                };
                eprintln!(
                    "[move-diag] Move arm: drained={} t_app:{:.6}->{:.6} t_decel:{:.6}->{:.6} t_disp:{:.6}->{:.6}",
                    drained.len(),
                    prior_t_appended,
                    state.t_appended,
                    prior_t_decel,
                    state.t_decel_start,
                    prior_t_disp,
                    state.t_dispatched,
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

                // Compute `actual = t_appended_after − t_appended_before`.
                // For the **streaming** path this is the right measure of
                // "what duration did this move add to the queue?" — not
                // the `drained` batch's duration (which is only the
                // committed-to-wire portion; the post-decel speculative
                // tail is still part of the appended-but-not-yet-drained
                // region). The spec §3.8 rectification is against the
                // *appended* duration, not the dispatched-this-round
                // duration. The latter would chronically under-rectify
                // because the trailing decel-to-zero is held back until
                // a commit fires (timer / Flush / UpdateShaper /
                // ClockSyncRearm).
                let actual = state.t_appended - prior_t_appended;
                let delta = actual - nominal;
                if delta.abs() > RECTIFICATION_TOLERANCE_S {
                    rectify_last_move_time(&last_move_time_bits, delta);
                }

                // Arm / re-arm the quiescence-commit timer. Setting
                // `last_append_time = Some(Instant::now())` on every
                // successful append (even when `emit_committed` produced
                // nothing this round) is what makes the timer the single
                // "did the user stop submitting moves?" signal.
                last_append_time = Some(Instant::now());
            }

            PlannerMsg::Flush { notify } => {
                eprintln!(
                    "[move-diag] Flush arm: last_append_time.is_some={} t_app={:.6} t_disp={:.6}",
                    last_append_time.is_some(),
                    state.t_appended,
                    state.t_dispatched,
                );
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
                    );
                    last_append_time = None;
                }
                let _ = notify.send(());
            }

            PlannerMsg::Dwell { duration_s, notify } => {
                // Advance the queued-time atomic by `duration_s` and
                // unblock the caller. Phase 6 Task 7.2 routed this
                // through the CAS-loop helper because the caller-side
                // `submit_move` advance is now a parallel writer on the
                // same atomic; the previous unconditional `store` would
                // have clobbered an in-flight caller-side advance from
                // a follow-on `submit_move` racing this `Dwell`.
                advance_last_move_time(&last_move_time_bits, duration_s);
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
                    );
                    last_append_time = None;
                }
                config.shaper = s;
                let shapers = shaper_config_to_axis_shapers(&config.shaper);
                state = ShaperState::new([0.0; 4], &shapers);
                thread_state.rebuild(&config);
            }

            PlannerMsg::KalicoStreamOpen { home_pos } | PlannerMsg::Homing { home_pos } => {
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

            PlannerMsg::Underrun { recovered_pos } | PlannerMsg::ForceIdle { recovered_pos } => {
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
        Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync>,
        Arc<AtomicUsize>,
    ) {
        let counter = Arc::new(AtomicUsize::new(0));
        let c = Arc::clone(&counter);
        let cb: Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync> =
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

    // ---------------------------------------------------------------------------
    // Bug regression: Z-only move after XY homing produces non-constant X/Y
    // shaped output (axes[0] and axes[1] deviate from constant by 751 mm and
    // 73 mm — massive, not numerical noise).
    //
    // Root: after `state.reset(home_pos)` the per-axis queues are re-seeded at
    // the new home position but `planned_fitted` / `planned_meta` are cleared.
    // `emit_committed` (and `commit_decel_to_zero`) rebuilds per-axis history
    // from `axes[i].pieces` — which are correct — but the shaping kernel for X
    // and Y is applied to the Z-only move's unshaped plan. For a Z-only segment
    // the X and Y components of `planned_fitted[*].axes[{0,1}]` are constant at
    // the reset's home X/Y. After a flush/commit the shaped X and Y curves must
    // therefore be constant (the convolution of a constant with any kernel is the
    // same constant).
    //
    // This test FAILS on the current code (demonstrating the bug) and should
    // pass after the fix is applied.
    //
    // Sequence mirrors a CoreXY G28 homing cycle:
    //   1. Reset to X homing force-position.
    //   2. X homing move (fast, large dx).
    //   3. Reset to X endstop position.
    //   4. X retract.
    //   5. X slow approach moves.
    //   6. Reset to XY homing force-position.
    //   7. XY diagonal move (to home_xy position).
    //   8. Reset to Z homing start (toolhead at home_xy, Z near top).
    //   9. Z-only homing move (slow descent).
    //  10. Flush: commit_decel_to_zero drains the held-back tail.
    //  11. Assert: every shaped segment from step 9 has constant X and Y.
    //
    // The test drives `ShaperState` inline (no `PlannerHandle` thread) using
    // the same internal helpers the planner run-loop uses, so the shaped output
    // is fully deterministic and directly inspectable.
    #[test]
    fn z_only_move_after_homing_xy_shaped_axes_are_constant() {
        use crate::classify::classify_and_build;
        use crate::dispatch::is_trivially_constant;

        // ---- Shaper config matching real Trident: smooth_mzv @ 186 Hz on X,
        //      smooth_mzv @ 122 Hz on Y, passthrough on Z. ----
        let shaper_cfg = ShaperConfig {
            x: trajectory::RequiredShaper::SmoothMzv {
                frequency_hz: 186.0,
            },
            y: trajectory::RequiredShaper::SmoothMzv {
                frequency_hz: 122.0,
            },
            z: trajectory::AxisShaper::Passthrough,
        };

        // Build a PlannerConfig that uses the Trident shapers but relaxed
        // fit tolerance so the test converges reliably on short homing moves.
        let mut cfg = PlannerConfig::default();
        cfg.limits.max_velocity = 1000.0;
        cfg.limits.max_accel = 70000.0;
        cfg.limits.max_z_velocity = 5.0;
        cfg.limits.max_z_accel = 100.0;
        cfg.shaper = shaper_cfg;

        // Construct ShaperState + contexts exactly as the run-loop does.
        let shapers = shaper_config_to_axis_shapers(&cfg.shaper);
        let mut state = ShaperState::new([0.0; 4], &shapers);

        let replan_ctx = build_replan_context(&cfg);
        let emit_kernels = shaper_config_to_emit_kernels(&cfg.shaper);
        let e_halos: Vec<trajectory::EHalo> = Vec::new();
        let emit_ctx = EmitContext {
            kernels: &emit_kernels,
            e_halos: &e_halos,
        };

        // Helper: append one move, immediately emit committed, discard output.
        // Mirrors the run-loop's per-Move arm (append_and_replan + emit_committed).
        let do_move =
            |state: &mut ShaperState,
             start: [f64; 3],
             dx: f64,
             dy: f64,
             dz: f64,
             feed: f64| {
                let m = classify_and_build(start, dx, dy, dz, 0.0, feed)
                    .expect("classify_and_build should succeed for valid moves");
                state
                    .append_and_replan(m.segment, &replan_ctx)
                    .expect("append_and_replan should succeed");
                state
                    .emit_committed(&emit_ctx)
                    .expect("emit_committed should succeed")
            };

        // Helper: flush (commit decel tail) and collect all emitted segments.
        let do_flush = |state: &mut ShaperState| -> Vec<ShapedSegment> {
            state
                .commit_decel_to_zero(&emit_ctx)
                .expect("commit_decel_to_zero should succeed")
        };

        // Sequence from actual klippy log during G28 on the Trident.
        // reset() = klippy's set_position (homing boundaries only).
        // Regular moves chain through append_and_replan without reset.
        //
        // X homing (sensorless, positive_dir, endstop at 300):
        state.reset([-154.5, 0.0, 0.0, 0.0]);
        let _ = do_move(&mut state, [-154.5, 0.0, 0.0], 454.5, 0.0, 0.0, 100.0);
        let _ = do_flush(&mut state);
        // Endstop triggered → set_position(haltpos ≈ 300)
        state.reset([300.0, 0.0, 0.0, 0.0]);
        // Retract + safe-X moves: regular moves, no reset between them.
        let _ = do_move(&mut state, [300.0, 0.0, 0.0], -5.0, 0.0, 0.0, 100.0);
        let _ = do_move(&mut state, [295.0, 0.0, 0.0], -100.0, 0.0, 0.0, 100.0);
        let _ = do_move(&mut state, [195.0, 0.0, 0.0], -100.0, 0.0, 0.0, 100.0);
        // home_rails flush_step_generation at end of X homing
        let _ = do_flush(&mut state);

        // Y homing (sensorless, positive_dir, endstop at 302):
        state.reset([95.0, -151.5, 0.0, 0.0]);
        let _ = do_move(&mut state, [95.0, -151.5, 0.0], 0.0, 453.5, 0.0, 100.0);
        let _ = do_flush(&mut state);
        // Endstop triggered → set_position(haltpos ≈ [95, 302])
        state.reset([95.0, 302.0, 0.0, 0.0]);
        // Retract + move to beacon home: regular moves, no reset between them.
        let _ = do_move(&mut state, [95.0, 302.0, 0.0], 0.0, -5.0, 0.0, 100.0);
        let _ = do_move(
            &mut state,
            [95.0, 297.0, 0.0],
            55.0,   // dx: to X=150
            -165.0, // dy: to Y=132
            0.0,
            300.0,
        );
        // home_rails flush_step_generation at end of Y homing
        let _ = do_flush(&mut state);

        // Z homing setup: set_position with Z at top of travel
        state.reset([150.0, 132.0, 344.0, 0.0]);

        // ---- Step 9: Z-only homing move (slow descent) ----
        // This is the move that triggers the bug: dx=0, dy=0, dz=-342.
        //
        // On the real printer, the planner's T_commit timer fires every
        // ~50ms, calling emit_committed hundreds of times over the 43s
        // Z descent. Each call dispatches a small window and updates the
        // shaper history with the shaped output. If the shaped output has
        // even a tiny X/Y deviation, it becomes the history for the next
        // emit — compounding across hundreds of calls to produce the
        // 751mm deviation seen on hardware.
        //
        // Simulate this by calling append_and_replan once, then calling
        // emit_committed in a loop (advancing t_decel_start each time
        // to simulate the commit timer opening the dispatch window).
        let z_move = classify_and_build(
            [150.0, 150.0, 344.0], 0.0, 0.0, -342.0, 0.0, 8.0,
        ).expect("classify Z move");
        state
            .append_and_replan(z_move.segment, &replan_ctx)
            .expect("append Z move");

        // Collect all segments via incremental emit_committed calls
        // that mimic the planner's T_commit-driven dispatch.
        let mut z_segments: Vec<trajectory::ShapedSegment> = Vec::new();
        // First emit_committed (immediate, pre-decel region)
        z_segments.extend(
            state.emit_committed(&emit_ctx).expect("emit_committed"),
        );
        // Final flush (decel tail)
        z_segments.extend(
            state.commit_decel_to_zero(&emit_ctx).expect("commit_decel_to_zero"),
        );

        // For the assertion we only need at least one segment.
        assert!(
            !z_segments.is_empty(),
            "commit_decel_to_zero must produce at least one segment for a 342 mm Z move",
        );

        // ---- Step 11: assert X and Y shaped axes are constant ----
        //
        // For a Z-only move the toolhead does not move in X or Y. The
        // unshaped X and Y trajectories in `planned_fitted` are constant
        // (all control points equal the reset home position: X=150, Y=150).
        // The shaper convolution of a constant function is the same constant.
        // Therefore every `ShapedSegment` produced by this move must have
        // axes[0] (X) and axes[1] (Y) trivially constant.
        //
        // On buggy code the X and Y axes have 751 mm and 73 mm maximum
        // control-point deviation from constant — a visible sign that the
        // shaper was operating on wrong history state left over from the
        // prior XY moves.
        let mut any_non_constant_x = false;
        let mut any_non_constant_y = false;
        let mut max_dev_x: f64 = 0.0;
        let mut max_dev_y: f64 = 0.0;

        for (i, seg) in z_segments.iter().enumerate() {
            let x_const = is_trivially_constant(&seg.axes[0]);
            let y_const = is_trivially_constant(&seg.axes[1]);
            let cps_x = seg.axes[0].control_points();
            let cps_y = seg.axes[1].control_points();

            // Measure deviation from the expected constant (150.0),
            // not from first CP (which itself may be wrong).
            let dev_from_expected_x = cps_x.iter()
                .map(|c| (c - 150.0).abs())
                .fold(0.0_f64, f64::max);
            let dev_from_expected_y = cps_y.iter()
                .map(|c| (c - 150.0).abs())
                .fold(0.0_f64, f64::max);
            let min_x = cps_x.iter().copied().fold(f64::INFINITY, f64::min);
            let max_x = cps_x.iter().copied().fold(f64::NEG_INFINITY, f64::max);
            let min_y = cps_y.iter().copied().fold(f64::INFINITY, f64::min);
            let max_y = cps_y.iter().copied().fold(f64::NEG_INFINITY, f64::max);

            eprintln!(
                "[z_only_constant] seg[{i}]: t=[{:.3},{:.3}] duration={:.3}s",
                seg.t_start, seg.t_end, seg.t_end - seg.t_start,
            );
            eprintln!(
                "  X: n_cps={} const={} range=[{:.3},{:.3}] dev_from_150={:.3}mm",
                cps_x.len(), x_const, min_x, max_x, dev_from_expected_x,
            );
            eprintln!(
                "  Y: n_cps={} const={} range=[{:.3},{:.3}] dev_from_150={:.3}mm",
                cps_y.len(), y_const, min_y, max_y, dev_from_expected_y,
            );

            if !x_const {
                any_non_constant_x = true;
                let first_x = cps_x[0];
                let dev_x = cps_x.iter()
                    .map(|c| (c - first_x).abs())
                    .fold(0.0_f64, f64::max);
                max_dev_x = max_dev_x.max(dev_from_expected_x);
            }

            if !y_const {
                any_non_constant_y = true;
                max_dev_y = max_dev_y.max(dev_from_expected_y);
            }
        }

        assert!(
            !any_non_constant_x,
            "Z-only move after XY homing produced non-constant X shaped output \
             (max deviation from constant: {max_dev_x:.3} mm). \
             Expected: X should be trivially constant at 150.0 mm throughout \
             the Z descent. Bug: shaper history state left over from prior XY \
             moves bleeds into the Z-only move's shaped X axis.",
        );

        assert!(
            !any_non_constant_y,
            "Z-only move after XY homing produced non-constant Y shaped output \
             (max deviation from constant: {max_dev_y:.3} mm). \
             Expected: Y should be trivially constant at 150.0 mm throughout \
             the Z descent. Bug: shaper history state left over from prior XY \
             moves bleeds into the Z-only move's shaped Y axis.",
        );
    }
}
