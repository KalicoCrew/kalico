use crate::multi::BatchError;
use crate::multi::joining::ChainState;
use crate::topp::chain::ChainGrid;
use crate::topp::constraints::{
    COMP_FLOOR, boundary_reachable_b_lower, boundary_reachable_b_upper,
};
use crate::topp::{EndpointConditions, ToleranceMode, schedule_chain_with_tolerance};
use crate::{BoundarySide, InfeasibleReason, SolveStatus, TopProfile};
use std::sync::Mutex;
use std::thread;

const BISECT_VEL_RESOLUTION_MM_S: f64 = 0.1;

pub(crate) fn fan_out_solves(
    chain_grids: &[ChainGrid],
    states: &mut [ChainState],
    n_threads: usize,
) -> Result<(), BatchError> {
    let n_chains = states.len();
    let dirty_indices: Vec<usize> = states
        .iter()
        .enumerate()
        .filter_map(|(i, s)| if s.dirty { Some(i) } else { None })
        .collect();
    if dirty_indices.is_empty() {
        return Ok(());
    }

    let queue = Mutex::new(dirty_indices);
    let results: Mutex<Vec<(usize, Result<TopProfile, crate::ScheduleError>)>> =
        Mutex::new(Vec::new());

    let v_starts: Vec<f64> = states.iter().map(|s| s.v_start).collect();
    let v_ends: Vec<f64> = states.iter().map(|s| s.v_end).collect();
    let a_starts: Vec<Option<f64>> = states.iter().map(|s| s.a_start).collect();

    thread::scope(|s| {
        for _ in 0..n_threads {
            s.spawn(|| {
                loop {
                    let Some(idx) = queue.lock().unwrap().pop() else {
                        break;
                    };
                    let pin_start = idx == 0;
                    let pin_end = idx + 1 == n_chains;
                    let r = solve_with_boundary_fallback(
                        &chain_grids[idx],
                        v_starts[idx],
                        v_ends[idx],
                        a_starts[idx],
                        pin_start,
                        pin_end,
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

                let start_is_pinned_boundary = idx == 0;
                let end_is_pinned_boundary = idx + 1 == n_chains;
                let kinematic_boundary_end = matches!(
                    profile.status,
                    SolveStatus::Infeasible {
                        reason: InfeasibleReason::BoundaryBelowMinReachable {
                            side: BoundarySide::End,
                            ..
                        } | InfeasibleReason::BoundaryAboveMaxReachable {
                            side: BoundarySide::End,
                            ..
                        },
                        ..
                    }
                );
                if !start_is_pinned_boundary && success {
                    if let Some(first) = profile.samples.first() {
                        states[idx].v_start = first.v.min(v_starts[idx]);
                    }
                }
                if !end_is_pinned_boundary && (success || kinematic_boundary_end) {
                    if let Some(last) = profile.samples.last() {
                        states[idx].v_end = last.v.min(v_ends[idx]);
                    }
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

pub(crate) fn solve_with_boundary_fallback(
    chain: &ChainGrid,
    v_start: f64,
    v_end: f64,
    a_start: Option<f64>,
    pin_start: bool,
    pin_end: bool,
) -> Result<TopProfile, crate::ScheduleError> {
    debug_assert!(
        a_start.is_none() || pin_start,
        "a_start pin without a pinned v_start — the bisection would silently re-plan a different boundary state"
    );

    let initial = schedule_chain_with_tolerance(
        chain,
        EndpointConditions {
            v_start,
            v_end,
            a_start,
        },
        ToleranceMode::Auto,
    )?;
    if is_success(initial.status) {
        return Ok(initial);
    }

    if pin_start && pin_end {
        return Ok(initial);
    }

    match initial.status {
        SolveStatus::Infeasible {
            reason:
                InfeasibleReason::BoundaryAboveMaxReachable {
                    side: BoundarySide::End,
                    max_b,
                },
            ..
        } if pin_start && !pin_end => {
            let max_v = max_b.max(0.0).sqrt();
            let s_total = chain.s.last().copied().unwrap_or(0.0);
            let (a_env, j_env) = chain_envelope_aj(chain);
            let a0_clamped = a_start.unwrap_or(0.0).clamp(-a_env, a_env);
            let min_b = crate::topp::constraints::boundary_reachable_b_lower(
                s_total, v_start, a0_clamped, a_env, j_env,
            );
            let min_v = min_b.max(0.0).sqrt();
            let mut lo_v = min_v;
            let mut hi_v = max_v;
            let mut best: Option<TopProfile> = None;
            for _ in 0..24 {
                if hi_v - lo_v < BISECT_VEL_RESOLUTION_MM_S {
                    break;
                }
                let mid_v = (lo_v + hi_v) * 0.5;
                let candidate = schedule_chain_with_tolerance(
                    chain,
                    EndpointConditions {
                        v_start,
                        v_end: mid_v,
                        a_start,
                    },
                    ToleranceMode::Auto,
                )?;
                if is_success(candidate.status) {
                    lo_v = mid_v;
                    best = Some(candidate);
                } else {
                    hi_v = mid_v;
                }
            }
            if let Some(profile) = best {
                return Ok(profile);
            }
            return Ok(initial);
        }
        SolveStatus::Infeasible {
            reason:
                InfeasibleReason::BoundaryBelowMinReachable {
                    side: BoundarySide::End,
                    min_b,
                },
            ..
        } if pin_start && !pin_end => {
            let min_v = min_b.max(0.0).sqrt();
            let v_max_end = chain
                .limits
                .last()
                .map(|l| l.v_max.iter().copied().fold(f64::INFINITY, f64::min))
                .unwrap_or(f64::INFINITY);
            let mut lo_v = min_v;
            let mut hi_v = v_max_end.min(v_start.max(v_end) * 2.0 + min_v + 1.0);
            let mut best: Option<TopProfile> = None;
            for _ in 0..24 {
                if hi_v - lo_v < BISECT_VEL_RESOLUTION_MM_S {
                    break;
                }
                let mid_v = (lo_v + hi_v) * 0.5;
                let candidate = schedule_chain_with_tolerance(
                    chain,
                    EndpointConditions {
                        v_start,
                        v_end: mid_v,
                        a_start,
                    },
                    ToleranceMode::Auto,
                )?;
                if is_success(candidate.status) {
                    hi_v = mid_v;
                    best = Some(candidate);
                } else {
                    lo_v = mid_v;
                }
            }
            if let Some(profile) = best {
                return Ok(profile);
            }
            return Ok(initial);
        }
        _ => {}
    }

    let scaled_mag = match (pin_start, pin_end) {
        (false, false) => v_start.abs().max(v_end.abs()),
        (false, true) => v_start.abs(),
        (true, false) => v_end.abs(),
        (true, true) => unreachable!("both-pinned handled above"),
    };

    let mut lo = 0.0_f64;
    let mut hi = 1.0_f64;
    let mut best: Option<TopProfile> = None;

    for _ in 0..24 {
        if (hi - lo) * scaled_mag < BISECT_VEL_RESOLUTION_MM_S {
            break;
        }
        let mid = (lo + hi) * 0.5;
        let vs = if pin_start { v_start } else { v_start * mid };
        let ve = if pin_end { v_end } else { v_end * mid };
        let candidate = schedule_chain_with_tolerance(
            chain,
            EndpointConditions {
                v_start: vs,
                v_end: ve,
                a_start,
            },
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

    if pin_start || pin_end {
        return Ok(initial);
    }

    const VEL_NEAR_ZERO: f64 = 1e-6;
    if v_start.abs() > VEL_NEAR_ZERO || v_end.abs() > VEL_NEAR_ZERO {
        return schedule_chain_with_tolerance(
            chain,
            EndpointConditions {
                v_start: 0.0,
                v_end: 0.0,
                a_start: None,
            },
            ToleranceMode::Auto,
        );
    }
    let base_v_max = chain.limits[0].v_max;
    let mut vlo = 0.0_f64;
    let mut vhi = 1.0_f64;
    let mut vbest: Option<TopProfile> = None;
    for _ in 0..24 {
        if (vhi - vlo) < BISECT_VEL_RESOLUTION_MM_S / base_v_max[0].max(1e-9) {
            break;
        }
        let mid = (vlo + vhi) * 0.5;
        let scaled = scale_chain_v_max(chain, mid);
        let candidate = schedule_chain_with_tolerance(
            &scaled,
            EndpointConditions {
                v_start: 0.0,
                v_end: 0.0,
                a_start: None,
            },
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
        let scaled = scale_chain_v_max(chain, 0.0);
        schedule_chain_with_tolerance(
            &scaled,
            EndpointConditions {
                v_start: 0.0,
                v_end: 0.0,
                a_start: None,
            },
            ToleranceMode::Auto,
        )
    }
}

fn chain_envelope_aj(chain: &crate::topp::chain::ChainGrid) -> (f64, f64) {
    let mut a_env = 0.0_f64;
    let mut j_env = 0.0_f64;
    for (node_idx, geom) in chain.geom.iter().enumerate() {
        let lim = chain.limits_at(node_idx);
        let g = &geom.c_prime;
        let mut a_tan = f64::INFINITY;
        let mut j_tan = f64::INFINITY;
        let mut active = false;
        for ax in 0..3 {
            let gabs = g[ax].abs();
            if gabs > COMP_FLOOR {
                a_tan = a_tan.min(lim.a_max[ax] / gabs);
                j_tan = j_tan.min(lim.j_max[ax] / gabs);
                active = true;
            }
        }
        if active {
            a_env = a_env.max(a_tan);
            j_env = j_env.max(j_tan);
        }
    }
    (a_env.max(1e-9), j_env.max(1e-9))
}

fn scale_chain_v_max(chain: &ChainGrid, factor: f64) -> ChainGrid {
    let mut scaled = chain.clone();
    for l in &mut scaled.limits {
        *l = crate::Limits::new(
            l.v_max.map(|v| v * factor),
            l.a_max,
            l.j_max,
            l.a_centripetal_max,
        );
    }
    scaled
}

pub(crate) fn is_success(status: SolveStatus) -> bool {
    matches!(
        status,
        SolveStatus::Solved | SolveStatus::SolvedInexact { .. } | SolveStatus::SolvedSlp { .. }
    )
}

#[cfg(test)]
mod tests;
