//! Planner thread core.
//!
//! Receives `PlannerMsg` messages, accumulates moves in a window, runs the
//! reduce → temporal → trajectory pipeline (via `trajectory::shape_batch`),
//! and dispatches shaped segments through a callback (per Task 6 of the
//! Phase-2 motion-bridge plan).
//!
//! Actual MCU push logic comes in Task 7. For now, the dispatch callback is
//! `Arc<dyn Fn(&trajectory::ShapedSegment) + Send + Sync>`.
//!
//! ## API divergences from the plan snippet
//!
//! - The trajectory crate exports `ShapeError` (not `ShapeBatchError`) as the
//!   top-level error from `shape_batch`. We map it through
//!   `PlannerError::Shape(trajectory::ShapeError)`.
//! - `shape_batch` returns `ShapeBatchOutput { segments, .. }`; we extract
//!   `segments` per the plan.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};

use crossbeam_channel::{Receiver, Sender, unbounded};
use geometry::segment::CubicSegment;

use crate::classify::ClassifiedMove;
use crate::config::{PlannerConfig, PlannerLimits};
use trajectory::streaming::ShaperState;
use trajectory::{AxisShaper, RequiredShaper, ShaperConfig, ShapedSegment};

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
}

impl PlannerHandle {
    pub fn spawn(
        config: PlannerConfig,
        dispatch: Arc<dyn Fn(&ShapedSegment) -> Result<(), String> + Send + Sync>,
    ) -> Self {
        let (tx, rx) = unbounded();
        let error = Arc::new(Mutex::new(None));
        let last_move_time_bits = Arc::new(AtomicU64::new(0u64));

        // Phase 1 plumbing: construct a `ShaperState` from the planner config
        // and seed it at home_pos = [0; 4]. Homing wires the real home pose
        // later (Phase 5). The state is owned by the planner thread.
        let shapers = shaper_config_to_axis_shapers(&config.shaper);
        let state = ShaperState::new([0.0; 4], &shapers);

        let error_thread = Arc::clone(&error);
        let last_thread = Arc::clone(&last_move_time_bits);
        let join = thread::Builder::new()
            .name("kalico-planner".to_string())
            .spawn(move || {
                run_loop(rx, config, state, dispatch, error_thread, last_thread);
            })
            .expect("spawn planner thread");

        Self {
            sender: tx,
            join_handle: Some(join),
            error,
            last_move_time_bits,
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

fn run_loop(
    rx: Receiver<PlannerMsg>,
    mut config: PlannerConfig,
    mut state: ShaperState,
    dispatch: Arc<dyn Fn(&ShapedSegment) -> Result<(), String> + Send + Sync>,
    error: Arc<Mutex<Option<PlannerError>>>,
    last_move_time_bits: Arc<AtomicU64>,
) {
    let mut print_time: f64 = 0.0;

    loop {
        // Block until at least one message arrives.
        let first = match rx.recv() {
            Ok(m) => m,
            Err(_) => return,
        };

        let mut buffer: Vec<CubicSegment> = Vec::new();
        let mut pending_flush: Option<Sender<()>> = None;
        let mut pending_dwell: Option<(f64, Sender<()>)> = None;
        let mut pending_config_update: Option<ConfigUpdate> = None;
        let mut shutdown = false;

        handle_msg(
            first,
            &mut buffer,
            &mut pending_flush,
            &mut pending_dwell,
            &mut pending_config_update,
            &mut shutdown,
            &mut config,
            &mut state,
        );

        // Drain until window is full or we hit a sync barrier.
        while !(buffer.len() >= config.window_capacity
            || pending_flush.is_some()
            || pending_dwell.is_some()
            || pending_config_update.is_some()
            || shutdown)
        {
            match rx.try_recv() {
                Ok(m) => handle_msg(
                    m,
                    &mut buffer,
                    &mut pending_flush,
                    &mut pending_dwell,
                    &mut pending_config_update,
                    &mut shutdown,
                    &mut config,
                    &mut state,
                ),
                Err(_) => break,
            }
        }

        // Run pipeline if we have moves.
        if !buffer.is_empty() {
            match run_pipeline(&buffer, &config) {
                Ok(shaped) => {
                    // Phase 1 plumbing: route the existing pipeline's output
                    // through `ShaperState` to establish the seam. The shim's
                    // `append_batch` would re-shape an already-shaped segment
                    // (it ingests `FittedSegment`, the trajectory crate's
                    // *pre-shape* per-segment form, which is not exposed
                    // through the public `shape_batch` API), so for Phase 1
                    // we stage the already-shaped output directly into
                    // `pending_dispatch` and drain through the same handle
                    // Phase 2+ will use. Behaviour is byte-identical to the
                    // pre-Phase-1 direct-dispatch path; the only difference
                    // is that `state.drain_committed()` is now the single
                    // source of truth for committed shaped segments.
                    state.pending_dispatch.extend(shaped);
                    let drained = state.drain_committed();

                    // shape_batch emits zero-relative t_start/t_end (batch_t_start = 0.0
                    // in trajectory::beta), so the last segment's t_end is this batch's
                    // total duration. Sum is robust against future API changes.
                    let batch_dur: f64 =
                        drained.iter().map(|s| s.t_end - s.t_start).sum();
                    print_time += batch_dur;
                    store_print_time(&last_move_time_bits, print_time);
                    for s in &drained {
                        if let Err(msg) = dispatch(s) {
                            *error.lock().unwrap() = Some(PlannerError::Dispatch(msg));
                            pending_flush = None;
                            pending_dwell = None;
                            break;
                        }
                    }
                }
                Err(e) => {
                    *error.lock().unwrap() = Some(e);
                    // Drop pending notifies — caller will see error on next op.
                    pending_flush = None;
                    pending_dwell = None;
                }
            }
        }

        if let Some(tx) = pending_flush.take() {
            let _ = tx.send(());
        }
        if let Some((dur, tx)) = pending_dwell.take() {
            print_time += dur;
            store_print_time(&last_move_time_bits, print_time);
            let _ = tx.send(());
        }

        // Apply the config update *after* the buffered window has been
        // shaped & dispatched, so subsequent moves see the new config but
        // already-buffered moves are processed under the old one.
        if let Some(update) = pending_config_update.take() {
            match update {
                ConfigUpdate::Limits(l) => config.limits = l,
                ConfigUpdate::Shaper(s) => {
                    config.shaper = s;
                    // Phase 1: rebuild the streaming-shaper kernels from the
                    // new config. We fully re-seed at home_pos = [0; 4] for
                    // now (homing wires the real pose later in Phase 5);
                    // since `pending_dispatch` was drained immediately above
                    // and the per-axis queues are not yet history-aware,
                    // this is behaviour-equivalent to the prior direct-call
                    // path, which simply re-built kernels inside the next
                    // `shape_batch`.
                    let shapers = shaper_config_to_axis_shapers(&config.shaper);
                    state = ShaperState::new([0.0; 4], &shapers);
                }
            }
        }

        if shutdown {
            return;
        }
    }
}

enum ConfigUpdate {
    Limits(PlannerLimits),
    Shaper(ShaperConfig),
}

fn handle_msg(
    msg: PlannerMsg,
    buffer: &mut Vec<CubicSegment>,
    pending_flush: &mut Option<Sender<()>>,
    pending_dwell: &mut Option<(f64, Sender<()>)>,
    pending_config_update: &mut Option<ConfigUpdate>,
    shutdown: &mut bool,
    config: &mut PlannerConfig,
    state: &mut ShaperState,
) {
    match msg {
        PlannerMsg::Move(m) => buffer.push(m.segment),
        PlannerMsg::Flush { notify } => *pending_flush = Some(notify),
        PlannerMsg::Dwell { duration_s, notify } => {
            *pending_dwell = Some((duration_s, notify));
        }
        PlannerMsg::UpdateLimits(l) => {
            if buffer.is_empty() {
                // No moves buffered: apply immediately.
                config.limits = l;
            } else {
                // Moves buffered: defer so they shape under the old config,
                // then apply the update as a barrier before further messages.
                *pending_config_update = Some(ConfigUpdate::Limits(l));
            }
        }
        PlannerMsg::UpdateShaper(s) => {
            if buffer.is_empty() {
                config.shaper = s;
                // Phase 1 plumbing: re-seed the streaming-shaper state so its
                // per-axis kernels match the new config. Behaviour-equivalent
                // to the prior path (which rebuilt kernels inside the next
                // `shape_batch`) because Phase 1 doesn't yet rely on history.
                let shapers = shaper_config_to_axis_shapers(&config.shaper);
                *state = ShaperState::new([0.0; 4], &shapers);
            } else {
                *pending_config_update = Some(ConfigUpdate::Shaper(s));
            }
        }
        PlannerMsg::Shutdown => *shutdown = true,
    }
}

// ---------------------------------------------------------------------------
// ShaperState construction helper
// ---------------------------------------------------------------------------

/// Convert the planner-config-level `ShaperConfig` (X/Y required + Z
/// optional) into the per-axis `[Option<AxisShaper>; 4]` array
/// `streaming::ShaperState::new` consumes (X, Y, Z, E). The E slot is `None`
/// in Phase 1 — extruder follows the shaped XY arc-length and is not a
/// separately shaped axis.
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
// Pipeline
// ---------------------------------------------------------------------------

fn run_pipeline(
    segments: &[CubicSegment],
    config: &PlannerConfig,
) -> Result<Vec<ShapedSegment>, PlannerError> {
    let limits = config.limits.to_temporal_limits();
    let seg_inputs: Vec<trajectory::ShapeSegmentInput<'_>> = segments
        .iter()
        .map(|seg| trajectory::ShapeSegmentInput {
            temporal: temporal::multi::SegmentInput {
                curve: &seg.xyz,
                limits,
                trailing_junction_chord_tolerance_mm: 0.05,
            },
            e_mode: seg.e_mode,
            extrusion_per_xy_mm: seg.extrusion_per_xy_mm,
            e_independent: seg.e_independent.as_ref(),
            feedrate_mm_s: seg.feedrate_mm_s,
        })
        .collect();

    let input = trajectory::ShapeBatchInput {
        segments: &seg_inputs,
        grid_strategy: temporal::multi::GridStrategy::Adaptive {
            min_n: 20,
            max_n: 200,
            target_grid_spacing_mm: 0.5,
        },
        worker_threads: config.worker_threads,
        shaper: config.shaper.clone(),
        fit_tolerance_mm: config.fit_tolerance_mm,
        beta_max_iters: config.beta_max_iters,
        beta_convergence_ratio: config.beta_convergence_ratio,
        e_limits: config.e_limits,
    };

    let output = trajectory::shape_batch(&input).map_err(PlannerError::Shape)?;
    Ok(output.segments)
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

    #[test]
    fn submit_and_flush_dispatches_segments() {
        let (dispatch, counter) = counting_dispatch();
        let mut h = PlannerHandle::spawn(relaxed_config(), dispatch);

        let m = classify_and_build([0.0; 3], 10.0, 0.0, 0.0, 0.0, 100.0).unwrap();
        h.submit_move(m).unwrap();
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

        let m = classify_and_build([0.0; 3], 10.0, 0.0, 0.0, 0.0, 100.0).unwrap();
        h.submit_move(m).unwrap();
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
    fn window_capacity_triggers_batch_flush_without_explicit_flush() {
        let (dispatch, counter) = counting_dispatch();
        let mut c = relaxed_config();
        c.window_capacity = 1;
        let mut h = PlannerHandle::spawn(c, dispatch);

        // Submit one move; window_capacity=1 forces immediate batch flush
        // without an explicit Flush message.
        let m = classify_and_build([0.0; 3], 10.0, 0.0, 0.0, 0.0, 100.0).unwrap();
        h.submit_move(m).unwrap();

        // Use Flush only as a synchronization point (waits for the worker
        // to drain the queue). The capacity-triggered batch must have
        // dispatched already.
        h.flush().unwrap();

        assert!(
            counter.load(Ordering::Relaxed) > 0,
            "window-capacity batch never dispatched"
        );
        assert!(h.last_move_time() > 0.0);

        h.shutdown();
    }

    #[test]
    fn two_adjacent_moves_shape_in_one_batch() {
        let c = relaxed_config();
        let m0 = classify_and_build([0.0; 3], 50.0, 0.0, 0.0, 0.0, 1000.0).unwrap();
        let m1 = classify_and_build([50.0, 0.0, 0.0], 50.0, 0.0, 0.0, 0.0, 1000.0).unwrap();
        let segments = vec![m0.segment, m1.segment];

        let shaped = run_pipeline(&segments, &c).unwrap();

        assert_eq!(shaped.len(), 2);
        assert_eq!(shaped[0].t_end, shaped[1].t_start);
    }

    #[test]
    fn drop_without_explicit_shutdown_does_not_hang() {
        let (dispatch, _counter) = counting_dispatch();
        let h = PlannerHandle::spawn(PlannerConfig::default(), dispatch);
        drop(h); // Drop impl should send Shutdown + join.
    }
}
