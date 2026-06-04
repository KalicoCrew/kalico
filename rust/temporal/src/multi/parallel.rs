use crate::multi::joining::SegmentState;
use crate::multi::{BatchError, SegmentInput};
use crate::topp::{ToleranceMode, schedule_segment_with_tolerance};
use crate::{GridConfig, SolveStatus, TopProfile};
use std::sync::Mutex;
use std::thread;

/// Re-solve all `dirty` segments in parallel across `n_threads` workers.
///
/// Only clears `dirty` on verifier-feasible success statuses: `Solved`,
/// `SolvedInexact`, and `SolvedSlp`. `SolvedSlp` must be included — it is
/// the actual termination path on curved geometry. Treating it as failure
/// would leave every SLP-required-cuts segment dirty forever.
pub(crate) fn fan_out_solves(
    inputs: &[SegmentInput<'_>],
    states: &mut [SegmentState],
    grids: &[GridConfig],
    n_threads: usize,
) -> Result<(), BatchError> {
    let dirty_indices: Vec<usize> = states
        .iter()
        .enumerate()
        .filter_map(|(i, s)| if s.dirty { Some(i) } else { None })
        .collect();
    if dirty_indices.is_empty() {
        return Ok(());
    }

    let queue = Mutex::new(dirty_indices);
    let results: Mutex<Vec<(usize, Result<crate::TopProfile, crate::ScheduleError>)>> =
        Mutex::new(Vec::new());

    let v_starts: Vec<f64> = states.iter().map(|s| s.v_start).collect();
    let v_ends: Vec<f64> = states.iter().map(|s| s.v_end).collect();

    thread::scope(|s| {
        for _ in 0..n_threads {
            s.spawn(|| {
                loop {
                    let Some(idx) = queue.lock().unwrap().pop() else {
                        break;
                    };
                    let r = solve_with_boundary_fallback(
                        inputs[idx].curve,
                        &inputs[idx].limits,
                        grids[idx],
                        v_starts[idx],
                        v_ends[idx],
                    );
                    results.lock().unwrap().push((idx, r));
                }
            });
        }
    });

    for (idx, r) in results.into_inner().unwrap() {
        match r {
            Ok(profile) => {
                let success = is_success(profile.status);
                // Always sync v_start/v_end to the actual profile endpoints so
                // the forward/reverse sweeps propagate the achieved velocity
                // even on infeasible solves.
                if let Some(first) = profile.samples.first() {
                    states[idx].v_start = first.v;
                }
                if let Some(last) = profile.samples.last() {
                    states[idx].v_end = last.v;
                }
                states[idx].profile = Some(profile);
                if success {
                    states[idx].dirty = false;
                }
            }
            Err(e) => return Err(BatchError::Segment(idx, e)),
        }
    }
    Ok(())
}

fn solve_with_boundary_fallback(
    curve: &nurbs::VectorNurbs<f64, 3>,
    limits: &crate::Limits,
    grid: GridConfig,
    v_start: f64,
    v_end: f64,
) -> Result<TopProfile, crate::ScheduleError> {
    const VEL_NEAR_ZERO: f64 = 1e-6;
    let initial =
        schedule_segment_with_tolerance(curve, limits, &grid, v_start, v_end, ToleranceMode::Auto)?;
    if is_success(initial.status) {
        return Ok(initial);
    }

    let mut lo = 0.0_f64;
    let mut hi = 1.0_f64;
    let mut best: Option<TopProfile> = None;

    for _ in 0..24 {
        let mid = (lo + hi) * 0.5;
        let candidate = schedule_segment_with_tolerance(
            curve,
            limits,
            &grid,
            v_start * mid,
            v_end * mid,
            ToleranceMode::Auto,
        )?;
        if is_success(candidate.status) {
            lo = mid;
            best = Some(candidate);
        } else {
            hi = mid;
        }
    }

    if let Some(profile) = best {
        return Ok(profile);
    }

    if v_start.abs() > VEL_NEAR_ZERO || v_end.abs() > VEL_NEAR_ZERO {
        return schedule_segment_with_tolerance(
            curve,
            limits,
            &grid,
            0.0,
            0.0,
            ToleranceMode::Auto,
        );
    }
    let base_v_max = limits.v_max;
    let mut vlo = 0.0_f64;
    let mut vhi = 1.0_f64;
    let mut vbest: Option<TopProfile> = None;
    for _ in 0..24 {
        let mid = (vlo + vhi) * 0.5;
        let scaled_v_max = [
            base_v_max[0] * mid,
            base_v_max[1] * mid,
            base_v_max[2] * mid,
        ];
        let scaled_limits = crate::Limits::new(
            scaled_v_max,
            limits.a_max,
            limits.j_max,
            limits.a_centripetal_max,
        );
        let candidate = schedule_segment_with_tolerance(
            curve,
            &scaled_limits,
            &grid,
            v_start * mid,
            v_end * mid,
            ToleranceMode::Auto,
        )?;
        if is_success(candidate.status) {
            vlo = mid;
            vbest = Some(candidate);
        } else {
            vhi = mid;
        }
    }

    if let Some(profile) = vbest {
        Ok(profile)
    } else {
        let zero_v_max = [0.0, 0.0, 0.0];
        let scaled_limits = crate::Limits::new(
            zero_v_max,
            limits.a_max,
            limits.j_max,
            limits.a_centripetal_max,
        );
        schedule_segment_with_tolerance(curve, &scaled_limits, &grid, 0.0, 0.0, ToleranceMode::Auto)
    }
}

fn is_success(status: SolveStatus) -> bool {
    matches!(
        status,
        SolveStatus::Solved | SolveStatus::SolvedInexact { .. } | SolveStatus::SolvedSlp { .. }
    )
}

#[cfg(test)]
mod tests;
