//! 3-thread fan-out for re-solving dirty segments. Per spec §2.6.

use crate::multi::joining::SegmentState;
use crate::multi::{BatchError, SegmentInput};
use crate::topp::{schedule_segment_with_tolerance, ToleranceMode};
use crate::{GridConfig, SolveStatus, TopProfile};
use std::sync::Mutex;
use std::thread;

/// Re-solve all `dirty` segments in parallel across `n_threads` workers using
/// `std::thread::scope` (no unsafe; works because Rust 1.63+ scoped threads
/// borrow for the scope lifetime, which encloses the call). MSRV is 1.85.
///
/// Per Codex review-1 finding I + kalico-plan-reviewer #8 + Codex review-3 +
/// kalico-verifier confirmation: a profile returned from `schedule_segment` is
/// `Ok(_)` even when the SOCP returned `MaxIter`, `Infeasible`, or the SLP outer
/// loop returned `DivergedSlp` / `MaxIterSlp`. We MUST inspect the public
/// `SolveStatus` and only clear `dirty` on the verifier-feasible success
/// statuses: `Solved`, `SolvedInexact`, AND `SolvedSlp`.
///
/// `SolvedSlp` is critical to include — it represents a feasible solve where
/// the SLP outer loop materially required cuts (the actual termination path on
/// curved geometry like the cubic-with-endpoint-κ class). Verified via
/// kalico-verifier (this session): `SolvedSlp` is only reachable when both
/// (a) the inner solver returned `Solved`/`SolvedInexact` and (b) `verify::check`
/// passed feasibility at `ε_feas` = 1e-3. Treating it as failure would leave
/// every SLP-required-cuts segment dirty forever, breaking convergence on
/// curved geometry.
///
/// Failure statuses (`Infeasible`, `MaxIter`, `DivergedSlp`, `MaxIterSlp`)
/// fall into the catch-all `_` arm and leave `dirty=true` for the caller to
/// notice. The convergence loop (Task 6) detects this via early-bail when
/// velocities have stabilized and surfaces it as
/// `JoiningStatus::StalledOnInfeasibleSegment { last_dirty_count }` (distinct
/// from `JoiningStatus::CappedAtMaxSweeps`, which signals genuine `MAX_SWEEPS` exhaustion
/// from joining oscillation — different and worse failure mode).
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

    // Snapshot endpoint velocities into thread-shared Vec (avoids passing
    // &states across the scope boundary).
    let v_starts: Vec<f64> = states.iter().map(|s| s.v_start).collect();
    let v_ends: Vec<f64> = states.iter().map(|s| s.v_end).collect();

    thread::scope(|s| {
        for _ in 0..n_threads {
            s.spawn(|| loop {
                let Some(idx) = queue.lock().unwrap().pop() else {
                    break;
                };
                let r = solve_with_boundary_fallback(
                    inputs[idx].curve,
                    &inputs[idx].limits,
                    &grids[idx],
                    v_starts[idx],
                    v_ends[idx],
                );
                results.lock().unwrap().push((idx, r));
            });
        }
    });

    // Apply results. Per Codex review-1: only clear dirty on actual success.
    for (idx, r) in results.into_inner().unwrap() {
        match r {
            Ok(profile) => {
                let success = is_success(profile.status);
                // Always sync v_start/v_end to the actual profile endpoints.
                // For infeasible/non-success solves the profile endpoint may be
                // lower than the requested v_end (e.g., the solver returned a
                // near-zero velocity on a too-short segment). Propagating the
                // actual achieved endpoint lets the forward/reverse sweeps reduce
                // v_jct caps correctly even when the initial cap was infeasible.
                // Per spec §2.3: the joining loop works on the actual achievable
                // velocity at each junction, not the upfront cap.
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
                // else: leave dirty=true so join_until_converged knows the segment
                // didn't actually solve; the convergence loop's MAX_SWEEPS cap
                // will catch persistent failures.
            }
            Err(e) => return Err(BatchError::Segment(idx, e)),
        }
    }
    Ok(())
}

fn solve_with_boundary_fallback(
    curve: &nurbs::VectorNurbs<f64, 3>,
    limits: &crate::Limits,
    grid: &GridConfig,
    v_start: f64,
    v_end: f64,
) -> Result<TopProfile, crate::ScheduleError> {
    let initial =
        schedule_segment_with_tolerance(curve, limits, grid, v_start, v_end, ToleranceMode::Auto)?;
    if is_success(initial.status) {
        return Ok(initial);
    }

    // Stage 1: bisect on endpoint velocities. The upfront junction cap can
    // exceed what a short segment can reach under jerk/accel limits; relaxing
    // endpoints toward zero opens up feasibility.
    let mut lo = 0.0_f64;
    let mut hi = 1.0_f64;
    let mut best: Option<TopProfile> = None;

    for _ in 0..24 {
        let mid = (lo + hi) * 0.5;
        let candidate = schedule_segment_with_tolerance(
            curve,
            limits,
            grid,
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

    // Stage 2: endpoint scaling didn't help. Bisect on v_max instead —
    // useful when the infeasibility sits at the peak-speed boundary (e.g.,
    // a feedrate-capped v_max for a rest-to-rest segment where SLP9 lands
    // exactly on the post-shape-jerk threshold and the SOCP can't converge
    // precisely). Gated on v_start ≈ v_end ≈ 0 so the more-common
    // junction-cap-too-high case (handled by stage 1) doesn't trigger an
    // unnecessary v_max derate that the joining loop would have to chase.
    const VEL_NEAR_ZERO: f64 = 1e-6;
    if v_start.abs() > VEL_NEAR_ZERO || v_end.abs() > VEL_NEAR_ZERO {
        // Endpoint scaling exhausted on a non-rest-to-rest segment — fall
        // back to the historical zero-zero solve. The joining sweep will
        // see the achieved zero endpoints and propagate.
        return schedule_segment_with_tolerance(
            curve, limits, grid, 0.0, 0.0, ToleranceMode::Auto,
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
            grid,
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
        // Last resort: rest-to-rest at zero v_max — produces a zero-velocity
        // profile but never fails the SOCP.
        let zero_v_max = [0.0, 0.0, 0.0];
        let scaled_limits = crate::Limits::new(
            zero_v_max,
            limits.a_max,
            limits.j_max,
            limits.a_centripetal_max,
        );
        schedule_segment_with_tolerance(curve, &scaled_limits, grid, 0.0, 0.0, ToleranceMode::Auto)
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
