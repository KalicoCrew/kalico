use crate::emit_shaped::{emit_shaped, EmitSegmentMeta, PerAxisHistory};
use crate::fit::FittedSegment;
use crate::pad::EHalo;
use crate::partition::BatchPartition;
use crate::plan_velocity::SafetyMode;
use crate::{BetaWarning, ShapeBatchInput, ShapeBatchOutput, ShapeError, ShapedSegment};
use geometry::segment::EMode;
use nurbs::algebra::PiecewisePolynomialKernel;
use nurbs::ScalarNurbs;

const MIN_AXIS_SPAN_FOR_DERATE: f64 = 0.5;
const BETA_ACCEL_MIN_RATIO: f64 = 0.02;

struct AxisKernels {
    x: PiecewisePolynomialKernel<f64>,
    y: PiecewisePolynomialKernel<f64>,
    z: Option<PiecewisePolynomialKernel<f64>>,
}

pub fn beta_loop(
    input: &ShapeBatchInput<'_>,
    partition: &BatchPartition,
) -> Result<ShapeBatchOutput, ShapeError> {
    beta_loop_with_safety(input, partition, SafetyMode::TerminalKnown)
}

pub fn beta_loop_with_safety(
    input: &ShapeBatchInput<'_>,
    partition: &BatchPartition,
    safety_mode: SafetyMode,
) -> Result<ShapeBatchOutput, ShapeError> {
    if partition.runs.is_empty() {
        return assemble_e_only_output(input, partition);
    }

    let planned = plan_batch_full(input, partition, safety_mode)?;

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

    assemble_with_e_gaps(input, partition, &planned, emitted_xy)
}

pub struct PlannedBatch {
    pub fitted: Vec<FittedSegment>,
    pub global_ends: Vec<f64>,
    pub joining_status: temporal::multi::JoiningStatus,
    pub converged: bool,
    pub beta_iterations: u8,
    pub beta_warning: Option<BetaWarning>,
}

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
        beta_iterations: outcome.iterations,
        beta_warning: outcome.beta_warning,
    })
}

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

#[derive(Debug, Clone, Copy)]
pub struct PlanStats {
    pub beta_iterations: u8,
    pub beta_converged: bool,
    pub segments: usize,
}

#[derive(Debug)]
pub struct PlanOutput {
    pub fitted: Vec<FittedSegment>,
    pub stats: PlanStats,
}

pub fn plan_velocity_inner(
    input: &ShapeBatchInput<'_>,
    partition: &BatchPartition,
    safety_mode: SafetyMode,
) -> Result<PlanOutput, ShapeError> {
    if partition.runs.is_empty() {
        return Ok(PlanOutput {
            fitted: Vec::new(),
            stats: PlanStats {
                beta_iterations: 0,
                beta_converged: true,
                segments: 0,
            },
        });
    }

    let planned = plan_batch_full(input, partition, safety_mode)?;
    let segments = planned.fitted.len();
    Ok(PlanOutput {
        fitted: planned.fitted,
        stats: PlanStats {
            beta_iterations: planned.beta_iterations,
            beta_converged: planned.converged,
            segments,
        },
    })
}

struct BetaIterationOutcome {
    result: BetaIterResult,
    converged: bool,
    iterations: u8,
    beta_warning: Option<BetaWarning>,
}

#[allow(clippy::too_many_lines)]
fn beta_iterate_inner(
    input: &ShapeBatchInput<'_>,
    partition: &BatchPartition,
    safety_mode: SafetyMode,
) -> Result<BetaIterationOutcome, ShapeError> {
    let kernels = AxisKernels {
        x: input.shaper.x.to_kernel(),
        y: input.shaper.y.to_kernel(),
        z: input.shaper.z.to_kernel(),
    };

    debug_assert!(
        !partition.runs.is_empty(),
        "beta_iterate_inner caller must handle empty-runs fast path"
    );

    let all_xy_indices: Vec<usize> = partition
        .runs
        .iter()
        .flat_map(|r| r.segment_range.clone())
        .collect();

    let machine_a_max: Vec<[f64; 3]> = all_xy_indices
        .iter()
        .map(|&i| input.segments[i].temporal.limits.a_max)
        .collect();

    let derate_machine_a_max = effective_machine_a_max(&machine_a_max, safety_mode);

    let mut planning_a_max: Vec<[f64; 3]> = machine_a_max.clone();

    let mut beta_warning: Option<BetaWarning> = None;
    let mut last_result: Option<BetaIterResult> = None;
    let mut converged = false;
    let mut iterations: u8 = 0;

    for iteration in 0..input.beta_max_iters {
        let result = match run_one_iteration(input, partition, &planning_a_max, &kernels) {
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
        iterations = iterations.saturating_add(1);

        let derate_info = compute_derate(&result.peaks, &derate_machine_a_max, &result.fitted);

        if !derate_info.needs_derate {
            last_result = Some(result);
            converged = true;
            break;
        }

        for (seg_flat_idx, peak_per_axis) in result.peaks.iter().enumerate() {
            for axis in 0..3 {
                let peak = peak_per_axis[axis];
                let machine = derate_machine_a_max[seg_flat_idx][axis];
                if peak > machine {
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

        if iteration == input.beta_max_iters - 1 {
            let final_result = match run_one_iteration(input, partition, &planning_a_max, &kernels)
            {
                Ok(result) => result,
                Err(_) => {
                    beta_warning = Some(beta_warning_from_last(&result, &derate_machine_a_max));
                    last_result = Some(result);
                    break;
                }
            };
            iterations = iterations.saturating_add(1);
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

    let result = match last_result {
        Some(r) => r,
        None => {
            debug_assert_eq!(input.beta_max_iters, 0);
            let r = run_one_iteration(input, partition, &planning_a_max, &kernels)?;
            iterations = 1;
            converged = true;
            r
        }
    };

    Ok(BetaIterationOutcome {
        result,
        converged,
        iterations,
        beta_warning,
    })
}

/// In `WorstCaseFuture` mode the last XY segment's limit is halved: for a
/// symmetric unit-DC kernel the past-only term must be ≤ 0.5·a_machine for
/// the convolution bound to stay ≤ a_machine. Applied to the whole segment
/// for simplicity; only the trailing-h region actually bites.
fn effective_machine_a_max(machine_a_max: &[[f64; 3]], safety_mode: SafetyMode) -> Vec<[f64; 3]> {
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

struct BetaIterResult {
    fitted: Vec<FittedSegment>,
    peaks: Vec<[f64; 3]>,
    joining_status: temporal::multi::JoiningStatus,
    _iteration: u8,
    global_ends: Vec<f64>,
}

#[allow(clippy::too_many_lines)]
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

    let mut run_profiles: Vec<Vec<temporal::TopProfile>> = Vec::new();
    let mut last_joining_status = temporal::multi::JoiningStatus::Converged;

    for run in &partition.runs {
        let run_segments: Vec<temporal::multi::SegmentInput<'_>> = run
            .segment_range
            .clone()
            .map(|global_idx| {
                let orig = &input.segments[global_idx].temporal;
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

        match batch_output.joining_status {
            temporal::multi::JoiningStatus::Converged => {}
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
                        global_idx,
                        profile.status,
                        v_start,
                        v_end,
                        n_samples,
                        total_time,
                        degree,
                        n_cps,
                        limits.v_max,
                        limits.a_max,
                        limits.j_max,
                        limits.a_centripetal_max,
                    );
                }
                return Err(ShapeError::TemporalJoining(status, detail));
            }
        }
        last_joining_status = batch_output.joining_status;

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

    let mut fitted: Vec<FittedSegment> = Vec::with_capacity(all_xy_indices.len());
    let mut global_ends: Vec<f64> = Vec::with_capacity(all_xy_indices.len());
    let mut t_cursor = 0.0_f64;
    let e_gaps_sorted = &partition.e_gaps;

    for (run_idx, run) in partition.runs.iter().enumerate() {
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

            let table = nurbs::arc_length::build_arc_length_table_vector(curve, 1e-6, 1024)
                .map_err(|e| ShapeError::ArcLength {
                    index: global_idx,
                    detail: format!("{e}"),
                })?;

            let s_pieces = crate::reparam::build_s_of_t_pieces(profile, t_offset);

            let arc_fit_tolerance = 1e-4; // mm
            let composed = crate::reparam::compose_segment(
                curve,
                &table.as_view(),
                &s_pieces,
                arc_fit_tolerance,
            )?;

            let mut seg_fitted = crate::fit::fit_and_split(&composed, input.fit_tolerance_mm)?;
            seg_fitted.t_start = s_pieces.t_start;
            seg_fitted.t_end = s_pieces.t_end;

            fitted.push(seg_fitted);
            t_cursor = s_pieces.t_end;
            global_ends.push(t_cursor);
        }
    }

    if let Some(last_run) = partition.runs.last() {
        for eg in e_gaps_sorted {
            if eg.segment_index >= last_run.segment_range.end {
                t_cursor += eg.duration;
            }
        }
    }

    let batch_t_end = t_cursor;
    let batch_t_start = 0.0;

    let e_halos = build_e_halos(partition, &global_ends);

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

fn axis_span(curve: &ScalarNurbs<f64>) -> f64 {
    let cps = curve.control_points();
    if cps.is_empty() {
        return 0.0;
    }
    let min = cps.iter().copied().fold(f64::INFINITY, f64::min);
    let max = cps.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    max - min
}

fn build_e_halos(partition: &BatchPartition, global_ends: &[f64]) -> Vec<EHalo> {
    let mut halos = Vec::new();

    let all_xy_indices: Vec<usize> = partition
        .runs
        .iter()
        .flat_map(|r| r.segment_range.clone())
        .collect();

    for eg in &partition.e_gaps {
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

fn find_gap_start(
    gap_seg_index: usize,
    all_xy_indices: &[usize],
    global_ends: &[f64],
    partition: &BatchPartition,
) -> f64 {
    let preceding_xy = all_xy_indices
        .iter()
        .enumerate()
        .filter(|(_, &idx)| idx < gap_seg_index)
        .last();

    if let Some((flat_idx, &preceding_idx)) = preceding_xy {
        let mut t = global_ends[flat_idx];
        for eg in &partition.e_gaps {
            if eg.segment_index > preceding_idx && eg.segment_index < gap_seg_index {
                t += eg.duration;
            }
        }
        t
    } else {
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

fn assemble_with_e_gaps(
    input: &ShapeBatchInput<'_>,
    partition: &BatchPartition,
    planned: &PlannedBatch,
    emitted_xy: Vec<ShapedSegment>,
) -> Result<ShapeBatchOutput, ShapeError> {
    let total_input_segments = input.segments.len();
    let mut output_segments: Vec<Option<ShapedSegment>> = vec![None; total_input_segments];

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
        output_segments[global_idx] = Some(shaped_seg);
    }

    for eg in &partition.e_gaps {
        let seg_input = &input.segments[eg.segment_index];
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

pub(crate) fn kernel_half_support(kernel: &PiecewisePolynomialKernel<f64>) -> f64 {
    let (lo, hi) = kernel.support();
    (hi - lo) / 2.0
}

fn constant_nurbs(value: f64, t_start: f64, t_end: f64) -> ScalarNurbs<f64> {
    let t_end_safe = if t_end <= t_start {
        t_start + 1e-12
    } else {
        t_end
    };
    ScalarNurbs::try_new(
        1,
        vec![t_start, t_start, t_end_safe, t_end_safe],
        vec![value, value],
    )
    .expect("constant NURBS construction should never fail")
}

#[cfg(test)]
mod tests;
