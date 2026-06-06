use std::sync::Arc;
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, RecvTimeoutError, Sender, unbounded};
use nurbs::algebra::PiecewisePolynomialKernel;

use crate::classify::ClassifiedMove;
use crate::config::{PlannerConfig, PlannerLimits};
use trajectory::plan_velocity::{PlanShaper, SafetyMode};
use trajectory::streaming::{EmitContext, ReplanContext, ReplanReport, ShaperState};
use trajectory::{AxisShaper, EHalo, RequiredShaper, ShapedSegment, ShaperConfig};

const T_IDLE: Duration = Duration::from_secs(3600);

/// Must equal `anchor::DEFAULT_LEAD_SECS`. Keep in sync with anchor.rs.
const LEAD: f64 = 0.25;

/// Safety margin for the decel-commit deadline: covers shaping + dispatch + pump + wire latency.
const SAFETY_MARGIN: f64 = 0.050;

const REPLAN_WARN_BUDGET_US: u64 = ((LEAD - SAFETY_MARGIN) * 1e6) as u64;

#[derive(Debug, Clone, Copy)]
pub struct ClockBias {
    pub freq: f64,
    pub offset_s: f64,
    pub last_clock: u64,
}

#[derive(Debug)]
pub enum PlannerMsg {
    Move(ClassifiedMove),
    Dwell { duration_s: f64, notify: Sender<()> },
    Flush { notify: Sender<Option<Instant>> },
    UpdateLimits(PlannerLimits),
    UpdateShaper(ShaperConfig),
    Shutdown,
    KalicoStreamOpen { home_pos: [f64; 4] },
    Homing { home_pos: [f64; 4] },
    Underrun { recovered_pos: [f64; 4] },
    ForceIdle { recovered_pos: [f64; 4] },
    ClockSyncRearm { new_bias: ClockBias },
}

#[derive(Debug)]
pub enum PlannerError {
    Shape(trajectory::ShapeError),
    ChannelClosed,
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
    #[error("MCU {0}: connection dropped during dispatch")]
    ConnectionDropped(u32),
    #[error("piece pump thread is gone; cannot dispatch")]
    PumpGone,
    #[error(
        "planner stream starvation: segment (stream t={seg_t_start:.3}s) scheduled \
         {gap_s:.3}s in the past; refusing to silently re-anchor — planner failed \
         to keep ahead of playback"
    )]
    SegmentLate { gap_s: f64, seg_t_start: f64 },
}

#[allow(missing_debug_implementations)]
pub struct PlannerHandle {
    sender: Sender<PlannerMsg>,
    join_handle: Option<JoinHandle<()>>,
    last_move_time_bits: Arc<AtomicU64>,
    commit_fire_count: Arc<AtomicU32>,
}

impl PlannerHandle {
    pub fn spawn(
        config: PlannerConfig,
        dispatch: Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync>,
    ) -> Self {
        let (tx, rx) = unbounded();
        let last_move_time_bits = Arc::new(AtomicU64::new(0u64));
        let commit_fire_count = Arc::new(AtomicU32::new(0));

        let shapers = shaper_config_to_axis_shapers(&config.shaper);
        let state = ShaperState::new([0.0; 4], &shapers);

        let last_thread = Arc::clone(&last_move_time_bits);
        let commit_thread = Arc::clone(&commit_fire_count);
        let join = thread::Builder::new()
            .name("kalico-planner".to_string())
            .spawn(move || {
                run_loop(rx, config, state, dispatch, last_thread, commit_thread);
            })
            .expect("spawn planner thread");

        Self {
            sender: tx,
            join_handle: Some(join),
            last_move_time_bits,
            commit_fire_count,
        }
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

        // Advance before channel send so the atomic is fresh when submit_move returns.
        // CAS guards against concurrent rectification from the planner thread.
        let nominal = m.nominal_duration();
        advance_last_move_time(&self.last_move_time_bits, nominal);

        self.sender
            .send(PlannerMsg::Move(m))
            .map_err(|_| PlannerError::ChannelClosed)
    }

    pub fn flush(&self) -> Result<(), PlannerError> {
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
                Ok(())
            }
            Err(_) => Err(PlannerError::ChannelClosed),
        }
    }

    pub fn dwell(&self, duration_s: f64) -> Result<(), PlannerError> {
        let (tx, rx) = crossbeam_channel::bounded(1);
        self.sender
            .send(PlannerMsg::Dwell {
                duration_s,
                notify: tx,
            })
            .map_err(|_| PlannerError::ChannelClosed)?;
        match rx.recv() {
            Ok(()) => Ok(()),
            Err(_) => Err(PlannerError::ChannelClosed),
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

    pub fn kalico_stream_open(&self, home_pos: [f64; 4]) -> Result<(), PlannerError> {
        self.sender
            .send(PlannerMsg::KalicoStreamOpen { home_pos })
            .map_err(|_| PlannerError::ChannelClosed)
    }

    pub fn homing(&self, home_pos: [f64; 4]) -> Result<(), PlannerError> {
        self.sender
            .send(PlannerMsg::Homing { home_pos })
            .map_err(|_| PlannerError::ChannelClosed)
    }

    pub fn underrun(&self, recovered_pos: [f64; 4]) -> Result<(), PlannerError> {
        self.sender
            .send(PlannerMsg::Underrun { recovered_pos })
            .map_err(|_| PlannerError::ChannelClosed)
    }

    pub fn force_idle(&self, recovered_pos: [f64; 4]) -> Result<(), PlannerError> {
        self.sender
            .send(PlannerMsg::ForceIdle { recovered_pos })
            .map_err(|_| PlannerError::ChannelClosed)
    }

    /// Must be called *before* `Router::set_clock_est_from_sample` to drain
    /// held-back output under the old bias first.
    pub fn clock_sync_rearm(&self, new_bias: ClockBias) -> Result<(), PlannerError> {
        self.sender
            .send(PlannerMsg::ClockSyncRearm { new_bias })
            .map_err(|_| PlannerError::ChannelClosed)
    }

    pub fn last_move_time(&self) -> f64 {
        f64::from_bits(self.last_move_time_bits.load(Ordering::Acquire))
    }

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

/// Below this magnitude the `actual − nominal` delta is indistinguishable from
/// floating-point noise (well below the MCU tick rate ~10 µs).
const RECTIFICATION_TOLERANCE_S: f64 = 1e-6;

const RECTIFICATION_CAS_MAX_ATTEMPTS: usize = 100;

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

fn advance_last_move_time(last_move_time_bits: &AtomicU64, delta: f64) {
    rectify_last_move_time(last_move_time_bits, delta);
}

fn fatal(e: &PlannerError) -> ! {
    eprintln!("kalico planner fatal error: {e}");
    tracing::error!(
        subsystem = "motion",
        event = "planner_fatal",
        error = %e,
        "planner encountered an unrecoverable error — aborting"
    );
    // tracing_appender uses non_blocking; sleep lets the worker drain the fatal
    // line to disk before abort skips the WorkerGuard flush.
    std::thread::sleep(Duration::from_millis(100));
    std::process::abort();
}

fn run_commit_and_dispatch(
    state: &mut ShaperState,
    thread_state: &PlannerThreadState,
    dispatch: &Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync>,
    last_move_time_bits: &AtomicU64,
    commit_fire_count: &AtomicU32,
) {
    let t_app_before = state.t_appended;
    let t_disp_before = state.t_dispatched;
    let commit_start = Instant::now();
    let drained = match state.commit_decel_to_zero(&thread_state.emit_ctx()) {
        Ok(out) => out,
        Err(e) => {
            tracing::error!(subsystem = "motion", event = "commit_decel_error", error = ?e, "run_commit_and_dispatch: commit_decel_to_zero failed");
            fatal(&PlannerError::Shape(e));
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
            fatal(&PlannerError::Dispatch(detail));
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
}

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
    let mut sync_instant: Option<Instant> = None;

    loop {
        let next_timeout = if state.t_dispatched < state.t_appended - 1e-12 {
            let esc = sync_instant.map_or(0.0, |t| t.elapsed().as_secs_f64());
            let remaining = (state.t_dispatched + LEAD - SAFETY_MARGIN) - esc;
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
                if state.t_dispatched < state.t_appended - 1e-12 {
                    run_commit_and_dispatch(
                        &mut state,
                        &thread_state,
                        &dispatch,
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
                // `submit_move` already advanced `last_move_time_bits` by the nominal.
                // Rectify if actual diverges: use CAS since submit_move is a concurrent writer.
                // Rectify against t_appended delta (not dispatched): the decel tail is held back.
                let nominal = m.nominal_duration();
                let prior_t_appended = state.t_appended;
                let prior_t_decel = state.t_decel_start;
                let prior_t_disp = state.t_dispatched;
                let move_dist = m.distance_mm;
                let move_feed = m.segment.feedrate_mm_s;

                // Commit held-back tail before inserting an idle rest-hold when clock outran plan.
                let esc = sync_instant.map_or(0.0, |t| t.elapsed().as_secs_f64());
                if esc > state.t_appended + 1e-6 {
                    if state.t_dispatched < state.t_appended - 1e-12 {
                        run_commit_and_dispatch(
                            &mut state,
                            &thread_state,
                            &dispatch,
                            &last_move_time_bits,
                            &commit_fire_count,
                        );
                    }
                    state.advance_idle(esc);
                }

                let replan_start = Instant::now();
                let report = match state.append_and_replan(m.segment, &thread_state.replan_ctx) {
                    Ok(r) => r,
                    Err(e) => {
                        tracing::error!(subsystem = "motion", event = "move_arm_error", phase = "append_and_replan", error = ?e, "Move arm: append_and_replan failed");
                        fatal(&PlannerError::Shape(e));
                    }
                };
                let replan_us = replan_start.elapsed().as_micros() as u64;
                let emit_start = Instant::now();
                let drained = match state.emit_committed(&thread_state.emit_ctx()) {
                    Ok(out) => out,
                    Err(e) => {
                        tracing::error!(subsystem = "motion", event = "move_arm_error", phase = "emit_committed", error = ?e, "Move arm: emit_committed failed");
                        fatal(&PlannerError::Shape(e));
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
                let emit_us = emit_start.elapsed().as_micros() as u64;
                let ReplanReport {
                    split_us,
                    solve_us,
                    rebuild_us,
                    window_segments,
                    plan,
                } = report;
                let beta_iters = plan.beta_iterations;
                let beta_converged = plan.beta_converged;
                tracing::debug!(
                    subsystem = "motion",
                    event = "replan_stats",
                    replan_us,
                    split_us,
                    solve_us,
                    rebuild_us,
                    window_segments,
                    beta_iters,
                    beta_converged,
                    emit_us,
                    drained = drained.len(),
                    dist_mm = move_dist,
                    feed_mm_s = move_feed,
                    nominal_s = nominal,
                    "replan stats"
                );
                if replan_us > REPLAN_WARN_BUDGET_US {
                    tracing::warn!(
                        subsystem = "motion",
                        event = "replan_overrun",
                        replan_us,
                        split_us,
                        solve_us,
                        rebuild_us,
                        window_segments,
                        beta_iters,
                        beta_converged,
                        emit_us,
                        drained = drained.len(),
                        dist_mm = move_dist,
                        feed_mm_s = move_feed,
                        nominal_s = nominal,
                        "replan overran its real-time budget"
                    );
                }

                for s in &drained {
                    if let Err(detail) = dispatch(s) {
                        tracing::error!(
                            subsystem = "motion",
                            event = "dispatch_error",
                            phase = "move_arm",
                            error = ?detail,
                            "Move arm: dispatch failed"
                        );
                        fatal(&PlannerError::Dispatch(detail));
                    }
                }

                // Capture sync_instant only on non-empty dispatch, not at append.
                // Capturing at append would make elapsed_since_sync run ahead of the MCU playhead.
                if sync_instant.is_none() && !drained.is_empty() {
                    sync_instant = Some(Instant::now());
                }

                let actual = state.t_appended - prior_t_appended;
                let delta = actual - nominal;
                if delta.abs() > RECTIFICATION_TOLERANCE_S {
                    rectify_last_move_time(&last_move_time_bits, delta);
                }
            }

            PlannerMsg::Flush { notify } => {
                // Commit held-back decel-to-zero; idempotent when already committed.
                if state.t_dispatched < state.t_appended - 1e-12 {
                    run_commit_and_dispatch(
                        &mut state,
                        &thread_state,
                        &dispatch,
                        &last_move_time_bits,
                        &commit_fire_count,
                    );
                }
                let finish = sync_instant.map(|t| {
                    t + Duration::try_from_secs_f64(state.t_appended + LEAD)
                        .unwrap_or(Duration::ZERO)
                });
                let _ = notify.send(finish);
            }

            PlannerMsg::Dwell { duration_s, notify } => {
                advance_last_move_time(&last_move_time_bits, duration_s);
                let _ = notify.send(());
            }

            PlannerMsg::UpdateLimits(l) => {
                config.limits = l;
                thread_state.rebuild(&config);
            }

            PlannerMsg::UpdateShaper(s) => {
                // Drain the held-back tail under the old kernels before switching.
                if state.t_dispatched < state.t_appended - 1e-12 {
                    run_commit_and_dispatch(
                        &mut state,
                        &thread_state,
                        &dispatch,
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
                sync_instant = None;
                state.reset(home_pos);
            }

            PlannerMsg::Underrun { recovered_pos } | PlannerMsg::ForceIdle { recovered_pos } => {
                sync_instant = None;
                state.reset(recovered_pos);
            }

            PlannerMsg::ClockSyncRearm { new_bias: _ } => {
                // Pre-swap barrier: flush held-back output under the old clock bias.
                // Must run before Router::set_clock_est_from_sample — calling after
                // would shape the drained tail under the new bias.
                if state.t_dispatched < state.t_appended - 1e-12 {
                    run_commit_and_dispatch(
                        &mut state,
                        &thread_state,
                        &dispatch,
                        &last_move_time_bits,
                        &commit_fire_count,
                    );
                }
            }

            PlannerMsg::Shutdown => return,
        }
    }
}

fn build_replan_context(config: &PlannerConfig) -> ReplanContext {
    ReplanContext {
        limits: config.limits.to_temporal_limits(),
        kernels: shaper_config_to_plan_shapers(&config.shaper),
        fit_tolerance_mm: config.fit_tolerance_mm,
        beta_max_iters: config.beta_max_iters,
        beta_convergence_ratio: config.beta_convergence_ratio,
        e_limits: config.e_limits,
        junction_chord_tolerance_mm: 0.05,
        worker_threads: config.worker_threads,
        grid_strategy: temporal::multi::GridStrategy::Adaptive {
            min_n: 20,
            max_n: 200,
            target_grid_spacing_mm: 0.5,
        },
        fallback_initial_v: 0.0,
        safety_mode: SafetyMode::WorstCaseFuture,
    }
}

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

#[cfg(test)]
mod tests;
