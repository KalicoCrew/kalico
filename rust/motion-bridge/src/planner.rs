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
                // counter increment. Clearing `last_append_time` disarms
                // the timer until the next `Move` arrives.
                let drained = match state.commit_decel_to_zero(&thread_state.emit_ctx()) {
                    Ok(out) => out,
                    Err(e) => {
                        *error.lock().unwrap() = Some(PlannerError::Shape(e));
                        last_append_time = None;
                        continue;
                    }
                };
                let batch_dur: f64 = drained.iter().map(|s| s.t_end - s.t_start).sum();
                print_time += batch_dur;
                store_print_time(&last_move_time_bits, print_time);
                for s in &drained {
                    if let Err(detail) = dispatch(s) {
                        *error.lock().unwrap() = Some(PlannerError::Dispatch(detail));
                        break;
                    }
                }
                commit_fire_count.fetch_add(1, Ordering::AcqRel);
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
                // No buffer to drain under streaming — replan + emit happen
                // per `submit_move`. `Flush` remains a synchronization
                // barrier so callers can wait for prior messages on the
                // queue to be processed before continuing.
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
                config.shaper = s;
                // Rebuild the kernels / replan context so the next
                // `append_and_replan` sees the new shaper config. We also
                // re-seed the `ShaperState` itself (matching the prior
                // Phase-1 behaviour); a future cross-axis-barrier
                // implementation (Phase 5 Task 5.3) will drain in-flight
                // moves under the old shaper first.
                let shapers = shaper_config_to_axis_shapers(&config.shaper);
                state = ShaperState::new([0.0; 4], &shapers);
                thread_state.rebuild(&config);
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
