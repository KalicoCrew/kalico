// Beta-medium outer iteration loop.
//
// Stage 5 of the trajectory shaping pipeline. Iterates TOPP-RA → time-reparam →
// fit → pad → convolve → peak-accel → derate until post-shape peaks converge
// within machine limits (or iteration cap is reached).
//
// ## Derate stability
//
// Two guards prevent runaway feedback when multiple segments share a junction:
//
// 1. **Inactive-axis skip**: an axis whose pre-shape (fitted) position span is
//    below `MIN_AXIS_SPAN_FOR_DERATE` is not derated. For pure-X moves the Y
//    axis moves ≪ 1 mm; its post-shape `peak_accel` value is dominated by
//    shaper-boundary numerical transients (amplified by `1/dt²` at 40 kHz) that
//    do not correspond to physical acceleration. Derating Y for a pure-X move
//    would reduce `planning_a_max[Y]`, which then propagates through the temporal
//    joining loop to drive the junction velocity toward zero, causing a cascade
//    that produces astronomically large subsequent peaks (up to `6e28 mm/s²`)
//    and ultimately a 5000-second degenerate segment on the second move.
//
// 2. **Floor**: `planning_a_max[seg][axis]` is clamped to at least
//    `machine_a_max[seg][axis] * BETA_ACCEL_MIN_RATIO`. This is a safety net
//    against the cascade even if the inactive-axis check is not sufficient.

use crate::emit_shaped::{emit_shaped, EmitSegmentMeta, PerAxisHistory};
use crate::fit::FittedSegment;
use crate::pad::EHalo;
use crate::partition::BatchPartition;
use crate::plan_velocity::SafetyMode;
use crate::{BetaWarning, ShapeBatchInput, ShapeBatchOutput, ShapeError, ShapedSegment};
use geometry::segment::EMode;
use nurbs::algebra::PiecewisePolynomialKernel;
use nurbs::ScalarNurbs;

/// Minimum position span (mm) for an axis to be eligible for beta-derate.
///
/// Axes whose pre-shape (fitted) position span is below this threshold are
/// treated as inactive for derating purposes. Their post-shape `peak_accel`
/// value is dominated by numerical transients from the shaper boundary
/// convolution rather than real physical acceleration; derating them would
/// cascade the junction velocity toward zero.
const MIN_AXIS_SPAN_FOR_DERATE: f64 = 0.5;

/// Minimum fraction of `machine_a_max` that `planning_a_max` is allowed to
/// reach. Guards against runaway derate even if `MIN_AXIS_SPAN_FOR_DERATE`
/// is insufficient (e.g., a genuinely small-span move with large boundary
/// transients on an active axis).
const BETA_ACCEL_MIN_RATIO: f64 = 0.02;

/// Per-axis kernel set, pre-built from the `ShaperConfig`.
struct AxisKernels {
    x: PiecewisePolynomialKernel<f64>,
    y: PiecewisePolynomialKernel<f64>,
    z: Option<PiecewisePolynomialKernel<f64>>,
}

/// Run the beta-medium outer loop over the given partition.
///
/// This is the main orchestrator: it drives Stages 1-5 until convergence, then
/// assembles the final `ShapeBatchOutput` with E-gap segments inserted.
///
/// Equivalent to [`beta_loop_with_safety`] called with [`SafetyMode::TerminalKnown`].
pub fn beta_loop(
    input: &ShapeBatchInput<'_>,
    partition: &BatchPartition,
) -> Result<ShapeBatchOutput, ShapeError> {
    beta_loop_with_safety(input, partition, SafetyMode::TerminalKnown)
}

/// Same as [`beta_loop`] but with explicit `safety_mode` controlling how the
/// β-medium loop interprets the post-shape peak in the trailing-`h` region of
/// the last XY-motion segment. See [`SafetyMode`] documentation for the bound
/// derivation (spec §3.6).
///
/// **Phase-2 wrapper shape (Task 2.3).** `shape_batch`'s public guarantee is
/// `plan_velocity` → `emit_shaped(empty history)` → E-gap interleaving. This
/// function is that pipeline literally:
///
/// 1. [`plan_batch_full`] runs the β-medium loop (TOPP-RA + per-iteration
///    shape via [`emit_shaped`] + peak / derate) and returns the
///    β-converged time-domain fitted segments + status metadata.
/// 2. [`emit_shaped`] runs the post-loop shaping pass on those fitted
///    segments. With an empty [`PerAxisHistory`] the output is byte-identical
///    to the legacy inline shaping (covered by
///    [`crate::emit_shaped::tests::empty_history_matches_shape_batch_byte_identical`]).
/// 3. [`assemble_with_e_gaps`] interleaves the partition's independent-E
///    segments around the shaped XY output, producing the final
///    [`ShapeBatchOutput`].
pub fn beta_loop_with_safety(
    input: &ShapeBatchInput<'_>,
    partition: &BatchPartition,
    safety_mode: SafetyMode,
) -> Result<ShapeBatchOutput, ShapeError> {
    // If there are no XY-motion runs, we only have E-gap segments.
    if partition.runs.is_empty() {
        return assemble_e_only_output(input, partition);
    }

    // ---- 1. plan_velocity (β-medium loop returns fitted + status metadata) ----
    let planned = plan_batch_full(input, partition, safety_mode)?;

    // ---- 2. emit_shaped (empty history → byte-identical to legacy path) ----
    let kernel_array = build_kernel_array_from_shaper_config(&input.shaper);
    let e_halos = build_e_halos(partition, &planned.global_ends);
    let meta: Vec<EmitSegmentMeta> = collect_xy_meta(input, partition);
    let batch_t_start = 0.0_f64;
    let batch_t_end = compute_batch_t_end(partition, &planned.global_ends);

    let emitted_xy = emit_shaped(
        &planned.fitted,
        &meta,
        &kernel_array,
        &e_halos,
        &PerAxisHistory::empty(),
        batch_t_start,
        batch_t_end,
    )?;

    // ---- 3. assemble_with_e_gaps (interleave independent-E segments) ----
    assemble_with_e_gaps(input, partition, &planned, emitted_xy)
}

/// β-medium-converged time-domain plan for a non-empty XY-motion partition.
///
/// Returned by [`plan_batch_full`]; consumed by `beta_loop_with_safety`'s
/// post-loop shape + assemble flow. All fields are flattened in input-segment
/// order across runs (E gaps excluded — they're interleaved by
/// [`assemble_with_e_gaps`]).
pub struct PlannedBatch {
    /// β-converged fitted (unshaped) per-axis NURBS for each XY-motion segment.
    pub fitted: Vec<FittedSegment>,
    /// Batch-global end time of each XY-motion segment, in flattened order.
    /// Used by [`build_e_halos`] / [`find_gap_start`] to place E-gap halo
    /// pieces on the correct global time line.
    pub global_ends: Vec<f64>,
    /// Joining status from the final temporal solve (forwarded to
    /// [`ShapeBatchOutput::temporal_status`]).
    pub joining_status: temporal::multi::JoiningStatus,
    /// `true` iff some β iteration produced a derate-free result.
    pub converged: bool,
    /// `None` if `converged`; otherwise a diagnostic describing which
    /// segments still exceed their machine limits at iteration exhaustion.
    pub beta_warning: Option<BetaWarning>,
}

/// Plan a non-empty XY-motion batch through the β-medium outer loop and
/// return the time-domain fitted trajectory + status metadata.
///
/// This is the planning half of the Task-2.3 wrapper split: it drives the
/// full β-medium loop (which still shapes per iteration via [`emit_shaped`]
/// to compute peaks for derate) but returns only the β-converged fitted
/// segments — the *final* shaped output is recomputed downstream by
/// `beta_loop_with_safety`'s explicit `emit_shaped(empty history)` call so
/// the public flow matches the spec's stated `plan + emit + assemble`
/// shape literally.
///
/// # Panics / errors
///
/// Caller must handle the empty-runs fast path; this function expects
/// `partition.runs` to be non-empty (matches [`beta_iterate_inner`]'s
/// contract). Errors from TOPP-RA / fit / shape are forwarded.
pub fn plan_batch_full(
    input: &ShapeBatchInput<'_>,
    partition: &BatchPartition,
    safety_mode: SafetyMode,
) -> Result<PlannedBatch, ShapeError> {
    let outcome = beta_iterate_inner(input, partition, safety_mode)?;
    Ok(PlannedBatch {
        fitted: outcome.result.fitted,
        global_ends: outcome.result.global_ends,
        joining_status: outcome.result.joining_status,
        converged: outcome.converged,
        beta_warning: outcome.beta_warning,
    })
}

/// Build the `[Option<PiecewisePolynomialKernel>; 4]` kernel array
/// `emit_shaped` consumes from a `ShaperConfig` (X / Y required, Z optional).
fn build_kernel_array_from_shaper_config(
    shaper: &crate::ShaperConfig,
) -> [Option<PiecewisePolynomialKernel<f64>>; 4] {
    [
        Some(shaper.x.to_kernel()),
        Some(shaper.y.to_kernel()),
        shaper.z.to_kernel(),
        None,
    ]
}

/// Collect per-XY-segment `EmitSegmentMeta` in flattened-run order to pass to
/// `emit_shaped`. The meta values are forwarded onto the resulting
/// `ShapedSegment.e_mode` / `extrusion_per_xy_mm` fields so the output is
/// already populated with the correct values before assembly.
fn collect_xy_meta(
    input: &ShapeBatchInput<'_>,
    partition: &BatchPartition,
) -> Vec<EmitSegmentMeta> {
    partition
        .runs
        .iter()
        .flat_map(|r| r.segment_range.clone())
        .map(|i| EmitSegmentMeta {
            e_mode: input.segments[i].e_mode,
            extrusion_per_xy_mm: input.segments[i].extrusion_per_xy_mm,
        })
        .collect()
}

/// Compute the batch's global `t_end`: the end of the last XY-motion segment
/// plus the duration of any trailing independent-E gaps. Mirrors the
/// `t_cursor` accounting `beta_iterate_inner` does internally so the
/// post-loop `emit_shaped` call sees the same right-pad domain the inline
/// shaping used to.
fn compute_batch_t_end(partition: &BatchPartition, global_ends: &[f64]) -> f64 {
    let mut t = global_ends.last().copied().unwrap_or(0.0);
    if let Some(last_run) = partition.runs.last() {
        for eg in &partition.e_gaps {
            if eg.segment_index >= last_run.segment_range.end {
                t += eg.duration;
            }
        }
    }
    t
}

/// Drive the β-medium loop over `partition` and return only the time-domain
/// **fitted** (unshaped, but β-converged) segments. This is the planning half
/// of the Phase-2 split (§5.2 of the streaming-shaper spec): no final shaping
/// pass / E-gap interleaving. The shaping half (Task 2.2's [`emit_shaped`])
/// consumes the returned `Vec<FittedSegment>` to produce the per-axis shaped
/// `ScalarNurbs`.
///
/// Internally each β iteration still calls [`emit_shaped`] to compute the
/// post-shape peak that drives the β-derate decision; only the final
/// shaped output is discarded here, since the streaming caller wants to
/// invoke `emit_shaped` itself with a non-empty [`PerAxisHistory`] to get a
/// continuous-across-`submit_move`-boundary convolution.
///
/// Convenience wrapper around [`plan_batch_full`] that drops the status
/// metadata. Use [`plan_batch_full`] when you also need
/// `joining_status` / `converged` / `beta_warning` / `global_ends`.
pub fn plan_velocity_inner(
    input: &ShapeBatchInput<'_>,
    partition: &BatchPartition,
    safety_mode: SafetyMode,
) -> Result<Vec<FittedSegment>, ShapeError> {
    // No XY runs — nothing to plan. The shaping half handles E-only output.
    if partition.runs.is_empty() {
        return Ok(Vec::new());
    }

    let planned = plan_batch_full(input, partition, safety_mode)?;
    Ok(planned.fitted)
}

/// Outcome of the shared β-medium iteration loop. The terminal `BetaIterResult`
/// carries the data both callers need (fitted, peaks, joining status,
/// `global_ends`); `converged` and `beta_warning` are populated for
/// `beta_loop_with_safety`'s `assemble_with_e_gaps` call and ignored by
/// `plan_velocity_inner`.
struct BetaIterationOutcome {
    /// Final iteration's result.
    result: BetaIterResult,
    /// True iff some iteration produced a derate-free result. Forwarded to
    /// [`assemble_with_e_gaps`] to set `beta_iters = 1` (vs `beta_max_iters`).
    converged: bool,
    /// `None` if `converged`; otherwise a diagnostic describing which segments
    /// still exceed their machine limits at exhaustion.
    beta_warning: Option<BetaWarning>,
}

/// Shared β-medium iteration kernel. Used by both [`beta_loop_with_safety`]
/// (which then assembles the final `ShapeBatchOutput`) and
/// [`plan_velocity_inner`] (which only needs `result.fitted`). The two
/// callers differ only in what they do with the returned outcome — the
/// iteration itself (kernel build, XY-index gather, TOPP-RA / fit / shape /
/// peak per iteration, derate apply, exhaustion handling) is identical.
///
/// Caller is responsible for the "all-E-gap" fast path (`partition.runs`
/// is empty); this function panics on an empty XY-index list rather than
/// silently returning a degenerate result.
#[allow(clippy::too_many_lines)] // Orchestration kernel — splitting hurts readability.
fn beta_iterate_inner(
    input: &ShapeBatchInput<'_>,
    partition: &BatchPartition,
    safety_mode: SafetyMode,
) -> Result<BetaIterationOutcome, ShapeError> {
    // Pre-build kernels from config.
    let kernels = AxisKernels {
        x: input.shaper.x.to_kernel(),
        y: input.shaper.y.to_kernel(),
        z: input.shaper.z.to_kernel(),
    };

    debug_assert!(
        !partition.runs.is_empty(),
        "beta_iterate_inner caller must handle empty-runs fast path"
    );

    // Machine a_max: immutable per-segment per-axis limits from the input.
    // We collect for all XY-motion segments across all runs.
    let all_xy_indices: Vec<usize> = partition
        .runs
        .iter()
        .flat_map(|r| r.segment_range.clone())
        .collect();

    let machine_a_max: Vec<[f64; 3]> = all_xy_indices
        .iter()
        .map(|&i| input.segments[i].temporal.limits.a_max)
        .collect();

    // Effective machine limit for derate purposes. In `TerminalKnown` mode this
    // equals `machine_a_max`. In `WorstCaseFuture` mode (streaming) the last
    // XY segment's trailing-`h` region is subject to the worst-case-future
    // bound (spec §3.6):
    //
    //   |ẍ_shaped(t)| ≤ ∫₀ʰ w(s)·|ẍ_past(t-s)|ds + a_machine·∫₋ₕ⁰ w(s)ds
    //
    // For a symmetric unit-DC kernel `∫₋ₕ⁰ w = 0.5`, so the second term is
    // `0.5·a_machine`. To keep the bound ≤ a_machine we require the
    // past-only term ≤ 0.5·a_machine. Modeling this conservatively: the last
    // segment's effective machine limit is halved (the trailing-`h` region
    // overlaps the last segment's terminus).
    //
    // This is the "loose but always safe" interpretation §3.6 documents.
    // A finer per-sample bound is a future refinement.
    let derate_machine_a_max = effective_machine_a_max(&machine_a_max, safety_mode);

    // Planning a_max: mutable copy that gets derated across iterations.
    let mut planning_a_max: Vec<[f64; 3]> = machine_a_max.clone();

    let mut beta_warning: Option<BetaWarning> = None;
    let mut last_result: Option<BetaIterResult> = None;
    let mut converged = false;

    for iteration in 0..input.beta_max_iters {
        let result = match run_one_iteration(
            input,
            partition,
            &planning_a_max,
            &kernels,
        ) {
            Ok(result) => result,
            Err(_) if last_result.is_some() => {
                beta_warning = Some(beta_warning_from_last(
                    last_result.as_ref().unwrap(),
                    &derate_machine_a_max,
                ));
                break;
            }
            Err(e) => return Err(e),
        };

        // Stage 5: check post-shape peaks against effective machine limits
        // (which fold in the worst-case-future bound when applicable).
        let derate_info = compute_derate(&result.peaks, &derate_machine_a_max, &result.fitted);

        if !derate_info.needs_derate {
            // Converged: no axis on any segment exceeds machine limit.
            last_result = Some(result);
            converged = true;
            break;
        }

        // Apply monotone derate: planning_a_max[seg][axis] *= machine / peak,
        // clamped to BETA_ACCEL_MIN_RATIO × machine and skipping axes whose
        // pre-shape position span is below MIN_AXIS_SPAN_FOR_DERATE.
        for (seg_flat_idx, peak_per_axis) in result.peaks.iter().enumerate() {
            for axis in 0..3 {
                let peak = peak_per_axis[axis];
                let machine = derate_machine_a_max[seg_flat_idx][axis];
                if peak > machine {
                    // Skip derating an axis that is not actively contributing to
                    // this segment's motion. A small pre-shape position span means
                    // the post-shape peak is driven by shaper-boundary numerical
                    // transients, not real physical acceleration.
                    let fitted_span = axis_span(&result.fitted[seg_flat_idx].axes[axis]);
                    if fitted_span < MIN_AXIS_SPAN_FOR_DERATE {
                        continue;
                    }

                    let ratio = machine / peak;
                    let floor = machine * BETA_ACCEL_MIN_RATIO;
                    planning_a_max[seg_flat_idx][axis] = (planning_a_max[seg_flat_idx][axis]
                        * ratio)
                        .min(planning_a_max[seg_flat_idx][axis])
                        .max(floor);
                }
            }
        }

        // If this is the last iteration, save the result and set warning.
        if iteration == input.beta_max_iters - 1 {
            // Exhausted iterations. Run one final solve with derated limits.
            let final_result = match run_one_iteration(
                input,
                partition,
                &planning_a_max,
                &kernels,
            ) {
                Ok(result) => result,
                Err(_) => {
                    beta_warning =
                        Some(beta_warning_from_last(&result, &derate_machine_a_max));
                    last_result = Some(result);
                    break;
                }
            };
            let final_derate = compute_derate(
                &final_result.peaks,
                &derate_machine_a_max,
                &final_result.fitted,
            );
            beta_warning = Some(BetaWarning {
                worst_ratio: final_derate.worst_ratio,
                segments_exceeding: final_derate.exceeding_indices.clone(),
            });
            last_result = Some(final_result);
        } else {
            last_result = Some(result);
        }
    }

    // If we didn't converge and didn't set last_result in the exhaustion path,
    // that means beta_max_iters == 0. Handle gracefully.
    let result = match last_result {
        Some(r) => r,
        None => {
            // beta_max_iters == 0: run one iteration with original limits.
            run_one_iteration(
                input,
                partition,
                &planning_a_max,
                &kernels,
            )?
        }
    };

    Ok(BetaIterationOutcome {
        result,
        converged,
        beta_warning,
    })
}

/// Build the effective per-segment per-axis machine accel limits used by the
/// β-medium derate criterion. In `TerminalKnown` mode this is just the input
/// machine limits. In `WorstCaseFuture` mode the **last** XY-motion segment's
/// limit is halved on every axis; the trailing-`h` region of that segment is
/// where the worst-case-future bound (spec §3.6) bites, and `0.5·a_machine` is
/// the limit the past-only term must satisfy for the bound to stay
/// `≤ a_machine` against the unit-DC kernel future-half mass.
///
/// **Loose-but-safe** (spec §3.6 wording): we apply the trailing-region limit
/// to the entire last segment rather than only its trailing-`h` slice. This
/// over-conservatism affects only the last segment's tail, which is the
/// portion that gets replanned away the moment a follow-on move arrives.
fn effective_machine_a_max(
    machine_a_max: &[[f64; 3]],
    safety_mode: SafetyMode,
) -> Vec<[f64; 3]> {
    let mut effective = machine_a_max.to_vec();
    if matches!(safety_mode, SafetyMode::WorstCaseFuture) {
        if let Some(last) = effective.last_mut() {
            for axis in last.iter_mut() {
                *axis *= 0.5;
            }
        }
    }
    effective
}

fn beta_warning_from_last(result: &BetaIterResult, machine_a_max: &[[f64; 3]]) -> BetaWarning {
    let derate = compute_derate(&result.peaks, machine_a_max, &result.fitted);
    BetaWarning {
        worst_ratio: derate.worst_ratio,
        segments_exceeding: derate.exceeding_indices,
    }
}

// ---------------------------------------------------------------------------
// Per-iteration pipeline
// ---------------------------------------------------------------------------

/// Result of one beta iteration: all data needed for convergence check and output.
struct BetaIterResult {
    /// Fitted segments (one per XY-motion segment, flattened across runs).
    fitted: Vec<FittedSegment>,
    /// Per-axis peak acceleration (one per XY-motion segment, flattened across runs).
    peaks: Vec<[f64; 3]>,
    /// Temporal joining status from the last run's solve.
    joining_status: temporal::multi::JoiningStatus,
    /// Total number of beta iterations so far (set by caller).
    _iteration: u8,
    /// Per-run batch-global end times (one per XY-motion segment, flattened).
    global_ends: Vec<f64>,
}

/// Run Stages 1-4 for one beta iteration.
#[allow(clippy::too_many_lines)] // Orchestration function — splitting hurts readability.
fn run_one_iteration(
    input: &ShapeBatchInput<'_>,
    partition: &BatchPartition,
    planning_a_max: &[[f64; 3]],
    kernels: &AxisKernels,
) -> Result<BetaIterResult, ShapeError> {
    let all_xy_indices: Vec<usize> = partition
        .runs
        .iter()
        .flat_map(|r| r.segment_range.clone())
        .collect();

    // ---- Stage 1: TOPP-RA per run ----
    let mut run_profiles: Vec<Vec<temporal::TopProfile>> = Vec::new();
    let mut last_joining_status = temporal::multi::JoiningStatus::Converged;

    for run in &partition.runs {
        // Build BatchInput for this run with derated limits.
        let run_segments: Vec<temporal::multi::SegmentInput<'_>> = run
            .segment_range
            .clone()
            .map(|global_idx| {
                let orig = &input.segments[global_idx].temporal;
                // Find the flat index of this segment in planning_a_max.
                let flat_idx = all_xy_indices
                    .iter()
                    .position(|&i| i == global_idx)
                    .unwrap();
                let derated_limits = temporal::Limits::new(
                    orig.limits.v_max,
                    planning_a_max[flat_idx],
                    orig.limits.j_max,
                    orig.limits.a_centripetal_max,
                );
                temporal::multi::SegmentInput {
                    curve: orig.curve,
                    limits: derated_limits,
                    trailing_junction_chord_tolerance_mm: orig.trailing_junction_chord_tolerance_mm,
                }
            })
            .collect();

        // Boundary velocities: the streaming shaper (Phase 3) feeds non-zero
        // values at the batch endpoints — `input.initial_v` applies to the
        // **first run's first segment**, `input.terminal_v` to the **last
        // run's last segment**. Interior runs are separated by independent-E
        // gaps, so the toolhead is stationary at their boundaries and the
        // run-local boundary velocities are both 0.0 there. For the legacy
        // (`shape_batch`) caller `input.initial_v` and `input.terminal_v`
        // are both 0.0, so every run is run-locally zero-boundary —
        // byte-identical to the pre-Phase-3 behaviour.
        let is_first_run = std::ptr::eq(run, &partition.runs[0]);
        let is_last_run = std::ptr::eq(run, &partition.runs[partition.runs.len() - 1]);
        let run_initial_v = if is_first_run { input.initial_v } else { 0.0 };
        let run_terminal_v = if is_last_run { input.terminal_v } else { 0.0 };

        let batch_input = temporal::multi::BatchInput {
            segments: &run_segments,
            grid_strategy: input.grid_strategy,
            worker_threads: input.worker_threads,
            initial_velocity: run_initial_v,
            terminal_velocity: run_terminal_v,
        };

        let batch_output = temporal::multi::plan_batch(batch_input)?;

        // Gate on joining status.
        match batch_output.joining_status {
            temporal::multi::JoiningStatus::Converged => {}
            // All non-Converged statuses (including future non-exhaustive variants)
            // are errors for the shaping pipeline.
            status => {
                use core::fmt::Write;
                let mut detail = String::new();
                for (local_idx, profile) in batch_output.profiles.iter().enumerate() {
                    let is_success = matches!(
                        profile.status,
                        temporal::SolveStatus::Solved
                            | temporal::SolveStatus::SolvedInexact { .. }
                            | temporal::SolveStatus::SolvedSlp { .. }
                    );
                    if is_success {
                        continue;
                    }
                    let global_idx = run.segment_range.start + local_idx;
                    let seg = &run_segments[local_idx];
                    let limits = &seg.limits;
                    let n_cps = seg.curve.control_points().len();
                    let degree = seg.curve.degree();
                    let total_time = profile.total_time;
                    let n_samples = profile.samples.len();
                    let v_start = profile.samples.first().map(|s| s.v).unwrap_or(f64::NAN);
                    let v_end = profile.samples.last().map(|s| s.v).unwrap_or(f64::NAN);
                    let _ = write!(
                        &mut detail,
                        " | seg{}: status={:?} v_start={:.4} v_end={:.4} \
                         n_samples={} total_time={:.4}s degree={} n_cps={} \
                         limits[v={:?} a={:?} j={:?} a_centripetal={:?}]",
                        global_idx, profile.status, v_start, v_end,
                        n_samples, total_time, degree, n_cps,
                        limits.v_max, limits.a_max, limits.j_max, limits.a_centripetal_max,
                    );
                }
                return Err(ShapeError::TemporalJoining(status, detail));
            }
        }
        last_joining_status = batch_output.joining_status;

        // Gate on per-profile status.
        for (local_idx, profile) in batch_output.profiles.iter().enumerate() {
            match profile.status {
                temporal::SolveStatus::Solved
                | temporal::SolveStatus::SolvedInexact { .. }
                | temporal::SolveStatus::SolvedSlp { .. } => {}
                ref status => {
                    let global_idx = run.segment_range.start + local_idx;
                    return Err(ShapeError::SegmentUnsolvable {
                        index: global_idx,
                        status: *status,
                    });
                }
            }
        }

        run_profiles.push(batch_output.profiles);
    }

    // ---- Stage 2: Time-reparameterization + composition + fit ----
    let mut fitted: Vec<FittedSegment> = Vec::with_capacity(all_xy_indices.len());
    let mut global_ends: Vec<f64> = Vec::with_capacity(all_xy_indices.len());
    let mut t_cursor = 0.0_f64;
    let e_gaps_sorted = &partition.e_gaps;

    for (run_idx, run) in partition.runs.iter().enumerate() {
        // Account for any E gaps before this run. The XY offsets are advanced
        // from the same s(t) pieces we emit below, so adjacent XY segments share
        // exact floating-point endpoints inside one batch.
        let prev_run_end = if run_idx > 0 {
            partition.runs[run_idx - 1].segment_range.end
        } else {
            0
        };
        for eg in e_gaps_sorted {
            if eg.segment_index >= prev_run_end && eg.segment_index < run.segment_range.start {
                t_cursor += eg.duration;
            }
        }

        for (local_idx, global_idx) in run.segment_range.clone().enumerate() {
            let profile = &run_profiles[run_idx][local_idx];
            let t_offset = t_cursor;

            let curve = input.segments[global_idx].temporal.curve;

            // Build arc-length table.
            let table = nurbs::arc_length::build_arc_length_table_vector(curve, 1e-6, 1024)
                .map_err(|e| ShapeError::ArcLength {
                    index: global_idx,
                    detail: format!("{e}"),
                })?;

            // Stage 2a: build s(t) pieces.
            let s_pieces = crate::reparam::build_s_of_t_pieces(profile, t_offset);

            // Stage 2b: compose x(s(t)). The arc-length fit tolerance is
            // separate from the Hermite refit tolerance — it controls the
            // polynomial approximation of x(s) on each TOPP-RA grid piece,
            // which must be tight for accurate second-derivative recovery.
            let arc_fit_tolerance = 1e-4; // mm — tight for derivative accuracy
            let composed = crate::reparam::compose_segment(
                curve,
                &table.as_view(),
                &s_pieces,
                arc_fit_tolerance,
            )?;

            // Stage 2c-d: C1 Hermite refit merges adjacent degree-6 composed
            // pieces into fewer degree-4 pieces. The sample-based peak-accel
            // check (central finite differences at 40 kHz) is immune to the
            // coefficient-magnitude issues that made symbolic differentiation
            // unstable, so the refit's position errors no longer amplify into
            // false peak-acceleration readings.
            let mut seg_fitted = crate::fit::fit_and_split(&composed, input.fit_tolerance_mm)?;
            // Patch t_start/t_end from s_pieces (the canonical source).
            seg_fitted.t_start = s_pieces.t_start;
            seg_fitted.t_end = s_pieces.t_end;

            fitted.push(seg_fitted);
            t_cursor = s_pieces.t_end;
            global_ends.push(t_cursor);
        }
    }

    // Account for trailing E gaps.
    if let Some(last_run) = partition.runs.last() {
        for eg in e_gaps_sorted {
            if eg.segment_index >= last_run.segment_range.end {
                t_cursor += eg.duration;
            }
        }
    }

    let batch_t_end = t_cursor;
    let batch_t_start = 0.0;

    // ---- Build E halos for padding ----
    let e_halos = build_e_halos(partition, &global_ends);

    // ---- Stages 3 + 3b: pad → convolve → trim → cubic refit (via emit_shaped) ----
    //
    // The previous inline implementation duplicated `emit_shaped`'s pad /
    // convolve / refit / passthrough logic verbatim. Routing the work through
    // `emit_shaped` here removes ~75 lines of duplication and aligns the
    // β-loop's per-iteration shape with the post-loop assembly path used by
    // `shape_batch` / `assemble_with_e_gaps`. The meta values fed in are
    // dummies — only the per-axis shaped NURBS feed back into the peak-accel
    // check below; the resulting `ShapedSegment` E-mode / extrusion fields
    // are discarded along with the whole `emitted` vector at the end of this
    // iteration.
    let kernel_array = build_kernel_array_from_axis_kernels(kernels);
    let dummy_meta: Vec<EmitSegmentMeta> = (0..fitted.len())
        .map(|_| EmitSegmentMeta {
            e_mode: EMode::CoupledToXy,
            extrusion_per_xy_mm: 0.0,
        })
        .collect();
    let emitted = emit_shaped(
        &fitted,
        &dummy_meta,
        &kernel_array,
        &e_halos,
        &PerAxisHistory::empty(),
        batch_t_start,
        batch_t_end,
    )?;

    // ---- Stage 4: Peak acceleration check ----
    // The shaped axes are only needed to compute per-segment peaks for the
    // β-derate decision; they are NOT stored on `BetaIterResult` because the
    // post-loop `beta_loop_with_safety` re-shapes via `emit_shaped` (with an
    // empty history) on the converged fitted output. That second pass is
    // byte-identical by construction (verified by
    // `emit_shaped::tests::empty_history_matches_shape_batch_byte_identical`).
    let peaks: Vec<[f64; 3]> = emitted
        .iter()
        .map(|seg| {
            [
                crate::peak::peak_accel(&seg.axes[0]),
                crate::peak::peak_accel(&seg.axes[1]),
                crate::peak::peak_accel(&seg.axes[2]),
            ]
        })
        .collect();

    Ok(BetaIterResult {
        fitted,
        peaks,
        joining_status: last_joining_status,
        _iteration: 0,
        global_ends,
    })
}

/// Convert the legacy `AxisKernels` (X required, Y required, Z optional) into
/// the `[Option<PiecewisePolynomialKernel>; 4]` shape that `emit_shaped`
/// consumes. The E slot is always `None` (E is not a shaped axis in the
/// MVP).
fn build_kernel_array_from_axis_kernels(
    kernels: &AxisKernels,
) -> [Option<PiecewisePolynomialKernel<f64>>; 4] {
    [
        Some(kernels.x.clone()),
        Some(kernels.y.clone()),
        kernels.z.clone(),
        None,
    ]
}

// ---------------------------------------------------------------------------
// Derate logic
// ---------------------------------------------------------------------------

struct DerateInfo {
    needs_derate: bool,
    worst_ratio: f64,
    exceeding_indices: Vec<usize>,
}

fn compute_derate(
    peaks: &[[f64; 3]],
    machine_a_max: &[[f64; 3]],
    fitted: &[crate::fit::FittedSegment],
) -> DerateInfo {
    let mut needs_derate = false;
    let mut worst_ratio: f64 = 0.0;
    let mut exceeding_indices = Vec::new();

    for (seg_idx, (peak, machine)) in peaks.iter().zip(machine_a_max.iter()).enumerate() {
        for axis in 0..3 {
            // Skip axes that are not actively contributing to this segment's
            // motion: their post-shape `peak` is dominated by shaper-boundary
            // numerical transients, not physical acceleration. Counting them
            // here would keep the loop spinning ("needs_derate") indefinitely
            // even though the per-axis apply step correctly skips them.
            let fitted_span = axis_span(&fitted[seg_idx].axes[axis]);
            if fitted_span < MIN_AXIS_SPAN_FOR_DERATE {
                continue;
            }
            if peak[axis] > machine[axis] {
                let ratio = peak[axis] / machine[axis];
                if ratio > worst_ratio {
                    worst_ratio = ratio;
                }
                if !exceeding_indices.contains(&seg_idx) {
                    exceeding_indices.push(seg_idx);
                }
                needs_derate = true;
            }
        }
    }

    DerateInfo {
        needs_derate,
        worst_ratio,
        exceeding_indices,
    }
}

/// Compute the peak-to-peak span of a scalar NURBS axis over all control
/// points. Used to decide whether an axis is "active" for beta-derate
/// purposes: an axis with span < `MIN_AXIS_SPAN_FOR_DERATE` contributes
/// negligible physical motion and its post-shape peak acceleration is
/// dominated by numerical boundary transients.
fn axis_span(curve: &ScalarNurbs<f64>) -> f64 {
    let cps = curve.control_points();
    if cps.is_empty() {
        return 0.0;
    }
    let min = cps.iter().copied().fold(f64::INFINITY, f64::min);
    let max = cps.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    max - min
}

// ---------------------------------------------------------------------------
// E-halo construction
// ---------------------------------------------------------------------------

fn build_e_halos(partition: &BatchPartition, global_ends: &[f64]) -> Vec<EHalo> {
    let mut halos = Vec::new();

    // For each E gap, compute its global time range.
    let all_xy_indices: Vec<usize> = partition
        .runs
        .iter()
        .flat_map(|r| r.segment_range.clone())
        .collect();

    for eg in &partition.e_gaps {
        // The E gap's global time start is immediately after the preceding XY
        // segment ends (or at batch start if no preceding XY segment).
        let t_gap_start = find_gap_start(eg.segment_index, &all_xy_indices, global_ends, partition);
        let t_gap_end = t_gap_start + eg.duration;

        halos.push(EHalo {
            xyz_position: eg.xyz_position,
            t_start: t_gap_start,
            t_end: t_gap_end,
        });
    }

    halos
}

/// Find the batch-global start time of an E-gap given its segment index.
fn find_gap_start(
    gap_seg_index: usize,
    all_xy_indices: &[usize],
    global_ends: &[f64],
    partition: &BatchPartition,
) -> f64 {
    // The E gap starts when the preceding XY segment ends.
    // Walk backward to find the last XY segment before this gap.
    let preceding_xy = all_xy_indices
        .iter()
        .enumerate()
        .filter(|(_, &idx)| idx < gap_seg_index)
        .last();

    if let Some((flat_idx, &preceding_idx)) = preceding_xy {
        // The preceding XY segment's end time, from the canonical s(t) pieces.
        // Add any earlier E gaps between that XY segment and this gap so
        // consecutive E-only segments occupy disjoint time intervals.
        let mut t = global_ends[flat_idx];
        for eg in &partition.e_gaps {
            if eg.segment_index > preceding_idx && eg.segment_index < gap_seg_index {
                t += eg.duration;
            }
        }
        t
    } else {
        // No preceding XY segment — gap starts at batch start.
        // But there might be preceding E gaps. Sum them.
        let mut t = 0.0;
        for eg in &partition.e_gaps {
            if eg.segment_index < gap_seg_index {
                t += eg.duration;
            } else {
                break;
            }
        }
        t
    }
}

// ---------------------------------------------------------------------------
// Output assembly
// ---------------------------------------------------------------------------

/// Interleave independent-E gap segments around the XY-shaped output to
/// produce the final [`ShapeBatchOutput`].
///
/// `emitted_xy` is the [`emit_shaped`] result over `planned.fitted` — one
/// entry per XY-motion segment, in flattened-run order, with its
/// `e_mode` / `extrusion_per_xy_mm` / `t_start` / `t_end` fields already
/// populated by `emit_shaped`. We just place them into the output buffer at
/// the original input index, build constant-position E-gap segments at the
/// gap times computed by [`find_gap_start`], and fill the
/// [`ShapeBatchOutput`] header with the planning-status metadata.
///
/// Replaces the legacy `assemble_output` whose XY branch pulled shaped axes
/// from `BetaIterResult.shaped`; the Task-2.3 wrapper rewire moves shaping
/// into a dedicated post-loop `emit_shaped` call so this function takes the
/// already-shaped XY segments by value.
fn assemble_with_e_gaps(
    input: &ShapeBatchInput<'_>,
    partition: &BatchPartition,
    planned: &PlannedBatch,
    emitted_xy: Vec<ShapedSegment>,
) -> Result<ShapeBatchOutput, ShapeError> {
    let total_input_segments = input.segments.len();
    let mut output_segments: Vec<Option<ShapedSegment>> = vec![None; total_input_segments];

    // Place XY-motion segments at their original input indices.
    let all_xy_indices: Vec<usize> = partition
        .runs
        .iter()
        .flat_map(|r| r.segment_range.clone())
        .collect();

    debug_assert_eq!(
        emitted_xy.len(),
        all_xy_indices.len(),
        "emitted_xy length must match the number of XY-motion segments",
    );

    for (flat_idx, shaped_seg) in emitted_xy.into_iter().enumerate() {
        let global_idx = all_xy_indices[flat_idx];
        // `emit_shaped` already populated e_mode / extrusion_per_xy_mm from the
        // `collect_xy_meta` values we passed in — no overwrite needed here.
        output_segments[global_idx] = Some(shaped_seg);
    }

    // Place E-gap segments.
    for eg in &partition.e_gaps {
        let seg_input = &input.segments[eg.segment_index];

        // Build constant-XYZ axes for the E-gap duration.
        let t_gap_start = find_gap_start(
            eg.segment_index,
            &all_xy_indices,
            &planned.global_ends,
            partition,
        );
        let t_gap_end = t_gap_start + eg.duration;

        let const_axes = std::array::from_fn(|axis| {
            constant_nurbs(eg.xyz_position[axis], t_gap_start, t_gap_end)
        });

        // Build time-parameterized E NURBS.
        let e_scheduled = seg_input
            .e_independent
            .map(|e_nurbs| {
                crate::e_independent::schedule_e_full(
                    e_nurbs,
                    seg_input.feedrate_mm_s,
                    &input.e_limits,
                    t_gap_start,
                )
            })
            .transpose()?;

        output_segments[eg.segment_index] = Some(ShapedSegment {
            axes: const_axes,
            e_mode: EMode::Independent,
            extrusion_per_xy_mm: 0.0,
            e_independent: e_scheduled,
            t_start: t_gap_start,
            t_end: t_gap_end,
        });
    }

    // All slots should be filled.
    let segments: Vec<ShapedSegment> = output_segments
        .into_iter()
        .enumerate()
        .map(|(i, opt)| {
            opt.unwrap_or_else(|| {
                panic!("output segment {i} was not populated — partition logic error")
            })
        })
        .collect();

    let beta_iters = if planned.converged {
        1
    } else {
        input.beta_max_iters
    };

    Ok(ShapeBatchOutput {
        segments,
        beta_iters,
        temporal_status: planned.joining_status,
        beta_warning: planned.beta_warning.clone(),
    })
}

/// Assemble output when there are no XY-motion runs (all segments are E gaps).
fn assemble_e_only_output(
    input: &ShapeBatchInput<'_>,
    partition: &BatchPartition,
) -> Result<ShapeBatchOutput, ShapeError> {
    let mut segments = Vec::with_capacity(input.segments.len());
    let mut t_cursor = 0.0;

    for eg in &partition.e_gaps {
        let seg_input = &input.segments[eg.segment_index];
        let t_start = t_cursor;
        let t_end = t_start + eg.duration;

        let const_axes =
            std::array::from_fn(|axis| constant_nurbs(eg.xyz_position[axis], t_start, t_end));

        let e_scheduled = seg_input
            .e_independent
            .map(|e_nurbs| {
                crate::e_independent::schedule_e_full(
                    e_nurbs,
                    seg_input.feedrate_mm_s,
                    &input.e_limits,
                    t_start,
                )
            })
            .transpose()?;

        segments.push(ShapedSegment {
            axes: const_axes,
            e_mode: EMode::Independent,
            extrusion_per_xy_mm: 0.0,
            e_independent: e_scheduled,
            t_start,
            t_end,
        });

        t_cursor = t_end;
    }

    Ok(ShapeBatchOutput {
        segments,
        beta_iters: 0,
        temporal_status: temporal::multi::JoiningStatus::Converged,
        beta_warning: None,
    })
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Half-support `(hi - lo) / 2.0` of a kernel. Exposed `pub(crate)` so
/// `emit_shaped` can use the same helper instead of duplicating the math.
pub(crate) fn kernel_half_support(kernel: &PiecewisePolynomialKernel<f64>) -> f64 {
    let (lo, hi) = kernel.support();
    (hi - lo) / 2.0
}

/// Build a constant-value `ScalarNurbs` on `[t_start, t_end]`.
fn constant_nurbs(value: f64, t_start: f64, t_end: f64) -> ScalarNurbs<f64> {
    // Ensure a non-degenerate knot span.
    let t_end_safe = if t_end <= t_start {
        t_start + 1e-12
    } else {
        t_end
    };
    ScalarNurbs::try_new(
        1,
        vec![t_start, t_start, t_end_safe, t_end_safe],
        vec![value, value],
        None,
    )
    .expect("constant NURBS construction should never fail")
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{ELimits, ShapeBatchInput, ShapeSegmentInput, ShaperConfig};
    use nurbs::VectorNurbs;

    fn default_limits() -> temporal::Limits {
        temporal::Limits::new(
            [500.0, 500.0, 500.0],
            [5_000.0, 5_000.0, 5_000.0],
            [100_000.0, 100_000.0, 100_000.0],
            2_500.0,
        )
    }

    fn default_shaper_config() -> ShaperConfig {
        ShaperConfig {
            x: crate::RequiredShaper::SmoothZv {
                frequency_hz: 180.0,
            },
            y: crate::RequiredShaper::SmoothZv {
                frequency_hz: 120.0,
            },
            z: crate::AxisShaper::Passthrough,
        }
    }

    fn default_e_limits() -> ELimits {
        ELimits {
            v_max: 100.0,
            a_max: 5000.0,
        }
    }

    /// Build a degree-1 (truly linear) NURBS from `start` to `end`.
    fn straight_linear(start: [f64; 3], end: [f64; 3]) -> VectorNurbs<f64, 3> {
        VectorNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![start, end], None).unwrap()
    }

    // ------------------------------------------------------------------
    // Test 1: Single straight-line segment — pipeline runs end-to-end
    // ------------------------------------------------------------------
    #[test]
    fn single_straight_line_converges() {
        let curve = straight_linear([0.0, 0.0, 0.0], [50.0, 0.0, 0.0]);
        let generous_limits = temporal::Limits::new(
            [500.0, 500.0, 500.0],
            [5_000.0, 5_000.0, 5_000.0],
            [100_000.0, 100_000.0, 100_000.0],
            2_500.0,
        );
        let segments = [ShapeSegmentInput {
            temporal: temporal::multi::SegmentInput {
                curve: &curve,
                limits: generous_limits,
                trailing_junction_chord_tolerance_mm: 0.05,
            },
            e_mode: EMode::CoupledToXy,
            extrusion_per_xy_mm: 0.04,
            e_independent: None,
            feedrate_mm_s: 100.0,
        }];

        // Use very high machine-limit ceiling for the beta check so that
        // post-shape peaks (which are numerically inflated by the convolution
        // pipeline) don't trigger derating. The TOPP-RA planning limits
        // are the segment's own limits (5000 mm/s^2).
        let input = ShapeBatchInput {
            segments: &segments,
            grid_strategy: temporal::multi::GridStrategy::Fixed(10),
            worker_threads: 1,
            shaper: default_shaper_config(),
            fit_tolerance_mm: 0.5,
            beta_max_iters: 1,
            beta_convergence_ratio: 1.02,
            e_limits: default_e_limits(),
            initial_v: 0.0,
            terminal_v: 0.0,
        };

        let output = crate::shape_batch(&input).expect("should succeed");

        assert_eq!(output.segments.len(), 1);
        assert!(output.segments[0].t_end > output.segments[0].t_start);
        assert_eq!(output.segments[0].e_mode, EMode::CoupledToXy);
        assert!((output.segments[0].extrusion_per_xy_mm - 0.04).abs() < 1e-12);

        // The shaped axes should be non-trivial ScalarNurbs.
        for axis_nurbs in &output.segments[0].axes {
            assert!(
                axis_nurbs.control_points().len() >= 2,
                "shaped axis should have at least 2 control points"
            );
        }
    }

    // ------------------------------------------------------------------
    // Test 2: Two segments with E-gap — pipeline handles partition
    // ------------------------------------------------------------------
    #[test]
    fn two_segments_with_e_gap() {
        let curve1 = straight_linear([0.0, 0.0, 0.0], [50.0, 0.0, 0.0]);
        let curve2 = straight_linear([50.0, 0.0, 0.0], [100.0, 0.0, 0.0]);
        let e_hold = straight_linear([50.0, 0.0, 0.0], [50.0, 0.0, 0.0]);
        let e_nurbs =
            nurbs::ScalarNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![10.0, 5.0], None)
                .unwrap();

        let segments = [
            ShapeSegmentInput {
                temporal: temporal::multi::SegmentInput {
                    curve: &curve1,
                    limits: default_limits(),
                    trailing_junction_chord_tolerance_mm: 0.05,
                },
                e_mode: EMode::CoupledToXy,
                extrusion_per_xy_mm: 0.04,
                e_independent: None,
                feedrate_mm_s: 100.0,
            },
            ShapeSegmentInput {
                temporal: temporal::multi::SegmentInput {
                    curve: &e_hold,
                    limits: default_limits(),
                    trailing_junction_chord_tolerance_mm: 0.05,
                },
                e_mode: EMode::Independent,
                extrusion_per_xy_mm: 0.0,
                e_independent: Some(&e_nurbs),
                feedrate_mm_s: 50.0,
            },
            ShapeSegmentInput {
                temporal: temporal::multi::SegmentInput {
                    curve: &curve2,
                    limits: default_limits(),
                    trailing_junction_chord_tolerance_mm: 0.05,
                },
                e_mode: EMode::CoupledToXy,
                extrusion_per_xy_mm: 0.04,
                e_independent: None,
                feedrate_mm_s: 100.0,
            },
        ];

        let input = ShapeBatchInput {
            segments: &segments,
            grid_strategy: temporal::multi::GridStrategy::Fixed(10),
            worker_threads: 1,
            shaper: default_shaper_config(),
            fit_tolerance_mm: 0.5,
            beta_max_iters: 1,
            beta_convergence_ratio: 1.02,
            e_limits: default_e_limits(),
            initial_v: 0.0,
            terminal_v: 0.0,
        };

        let output = crate::shape_batch(&input).expect("should succeed");

        // Three output segments: [XY, Independent-E, XY].
        assert_eq!(output.segments.len(), 3);
        assert_eq!(output.segments[0].e_mode, EMode::CoupledToXy);
        assert_eq!(output.segments[1].e_mode, EMode::Independent);
        assert_eq!(output.segments[2].e_mode, EMode::CoupledToXy);

        // The E-gap segment should have an independent E NURBS.
        assert!(output.segments[1].e_independent.is_some());

        // Time ordering: each segment starts after the previous ends.
        assert!(output.segments[0].t_end <= output.segments[1].t_start + 1e-9);
        assert!(output.segments[1].t_end <= output.segments[2].t_start + 1e-9);
    }

    // ------------------------------------------------------------------
    // Test 3: Derate logic unit test
    // ------------------------------------------------------------------
    #[test]
    fn derate_detects_exceeding_peaks() {
        // Build a one-segment fitted with all axes spanning >> MIN_AXIS_SPAN_FOR_DERATE
        // so the inactive-axis skip does not apply.
        let make_axis = |x_start: f64, x_end: f64| {
            nurbs::bezier::bezier_pieces_to_nurbs(&[nurbs::bezier::BezierPiece {
                u_start: 0.0,
                u_end: 1.0,
                coeffs: vec![x_start, x_end - x_start],
            }])
        };
        let fitted = vec![crate::fit::FittedSegment {
            axes: [
                make_axis(0.0, 100.0),
                make_axis(0.0, 100.0),
                make_axis(0.0, 100.0),
            ],
            t_start: 0.0,
            t_end: 1.0,
        }];
        let machine = vec![[5000.0, 5000.0, 5000.0]];
        let peaks_within = vec![[4000.0, 3000.0, 2000.0]];
        let info = compute_derate(&peaks_within, &machine, &fitted);
        assert!(!info.needs_derate);

        let peaks_exceed = vec![[6000.0, 3000.0, 2000.0]];
        let info = compute_derate(&peaks_exceed, &machine, &fitted);
        assert!(info.needs_derate);
        assert!((info.worst_ratio - 1.2).abs() < 1e-10);
        assert_eq!(info.exceeding_indices, vec![0]);
    }

    // ------------------------------------------------------------------
    // Test 4: All-E-gap batch
    // ------------------------------------------------------------------
    #[test]
    fn all_e_gaps_output() {
        let e_hold = straight_linear([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
        let e_nurbs =
            nurbs::ScalarNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![10.0, 5.0], None)
                .unwrap();

        let segments = [ShapeSegmentInput {
            temporal: temporal::multi::SegmentInput {
                curve: &e_hold,
                limits: default_limits(),
                trailing_junction_chord_tolerance_mm: 0.05,
            },
            e_mode: EMode::Independent,
            extrusion_per_xy_mm: 0.0,
            e_independent: Some(&e_nurbs),
            feedrate_mm_s: 50.0,
        }];

        let input = ShapeBatchInput {
            segments: &segments,
            grid_strategy: temporal::multi::GridStrategy::Fixed(10),
            worker_threads: 1,
            shaper: default_shaper_config(),
            fit_tolerance_mm: 0.5,
            beta_max_iters: 1,
            beta_convergence_ratio: 1.02,
            e_limits: default_e_limits(),
            initial_v: 0.0,
            terminal_v: 0.0,
        };

        let output = crate::shape_batch(&input).expect("should succeed");

        assert_eq!(output.segments.len(), 1);
        assert_eq!(output.segments[0].e_mode, EMode::Independent);
        assert!(output.segments[0].e_independent.is_some());
        assert!(output.segments[0].t_end > output.segments[0].t_start);
    }

    // ------------------------------------------------------------------
    // Test 5: effective_machine_a_max — only last segment is derated
    // ------------------------------------------------------------------
    //
    // Direct unit test of the spec §3.6 derate-target selection. The
    // end-to-end planner can't witness this exactly because TOPP-RA's
    // joining loop propagates segment N's tighter limit back through
    // segment N-1's tail. This test asserts the invariant at its source.
    #[test]
    fn effective_machine_a_max_terminal_known_is_identity() {
        let machine = vec![
            [5_000.0, 5_000.0, 5_000.0],
            [3_000.0, 4_000.0, 2_500.0],
            [1_000.0, 1_500.0, 2_000.0],
        ];
        let effective = effective_machine_a_max(&machine, SafetyMode::TerminalKnown);
        assert_eq!(effective, machine);
    }

    #[test]
    fn effective_machine_a_max_worst_case_only_halves_last_segment() {
        let machine = vec![
            [5_000.0, 5_000.0, 5_000.0],
            [3_000.0, 4_000.0, 2_500.0],
            [1_000.0, 1_500.0, 2_000.0],
        ];
        let effective = effective_machine_a_max(&machine, SafetyMode::WorstCaseFuture);

        // Segments 0 and 1 must be unchanged.
        assert_eq!(effective[0], machine[0]);
        assert_eq!(effective[1], machine[1]);
        // Segment 2 (last) is halved on every axis.
        for axis in 0..3 {
            assert!(
                (effective[2][axis] - machine[2][axis] * 0.5).abs() < 1e-12,
                "axis {axis}: expected {} (half of {}), got {}",
                machine[2][axis] * 0.5,
                machine[2][axis],
                effective[2][axis],
            );
        }
    }

    #[test]
    fn effective_machine_a_max_worst_case_single_segment() {
        // Single-segment edge case: the only segment IS the last segment.
        let machine = vec![[5_000.0, 4_000.0, 3_000.0]];
        let effective = effective_machine_a_max(&machine, SafetyMode::WorstCaseFuture);
        assert_eq!(effective.len(), 1);
        for axis in 0..3 {
            assert!((effective[0][axis] - machine[0][axis] * 0.5).abs() < 1e-12);
        }
    }

    #[test]
    fn effective_machine_a_max_worst_case_empty_is_empty() {
        let machine: Vec<[f64; 3]> = vec![];
        let effective = effective_machine_a_max(&machine, SafetyMode::WorstCaseFuture);
        assert!(effective.is_empty());
    }
}
