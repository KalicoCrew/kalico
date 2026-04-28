//! 3-thread fan-out for re-solving dirty segments. Per spec §2.6.

use crate::GridConfig;
use crate::SolveStatus;
use crate::multi::joining::SegmentState;
use crate::multi::{BatchError, SegmentInput};
use crate::topp::{ToleranceMode, schedule_segment_with_tolerance};
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
            s.spawn(|| {
                loop {
                    let Some(idx) = queue.lock().unwrap().pop() else {
                        break;
                    };
                    let r = schedule_segment_with_tolerance(
                        inputs[idx].curve,
                        &inputs[idx].limits,
                        &grids[idx],
                        v_starts[idx],
                        v_ends[idx],
                        ToleranceMode::Auto,
                    );
                    results.lock().unwrap().push((idx, r));
                }
            });
        }
    });

    // Apply results. Per Codex review-1: only clear dirty on actual success.
    for (idx, r) in results.into_inner().unwrap() {
        match r {
            Ok(profile) => {
                let success = matches!(
                    profile.status,
                    SolveStatus::Solved
                        | SolveStatus::SolvedInexact { .. }
                        | SolveStatus::SolvedSlp { .. }
                );
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::multi::SegmentInput;
    use crate::{GridConfig, GridScheme, Limits};
    use nurbs::VectorNurbs;

    fn straight() -> VectorNurbs<f64, 3> {
        VectorNurbs::<f64, 3>::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![[0.0, 0.0, 0.0], [50.0, 0.0, 0.0]],
            None,
        )
        .unwrap()
    }

    fn limits() -> Limits {
        Limits::new([500.0; 3], [5_000.0; 3], [100_000.0; 3], 2_500.0)
    }

    #[test]
    fn fan_out_processes_all_dirty() {
        let curves: Vec<_> = (0..4).map(|_| straight()).collect();
        let inputs: Vec<SegmentInput> = curves
            .iter()
            .map(|c| SegmentInput {
                curve: c,
                limits: limits(),
                trailing_junction_chord_tolerance_mm: 0.05,
            })
            .collect();
        let grids = vec![
            GridConfig {
                scheme: GridScheme::UniformArclength,
                n: 20
            };
            4
        ];
        let mut states: Vec<_> = (0..4)
            .map(|_| SegmentState {
                v_start: 0.0,
                v_end: 0.0,
                profile: None,
                dirty: true,
            })
            .collect();
        fan_out_solves(&inputs, &mut states, &grids, 3).unwrap();
        for s in &states {
            assert!(s.profile.is_some());
            assert!(!s.dirty);
        }
    }
}
