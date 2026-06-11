use crate::TopProfile;
use crate::multi::parallel::fan_out_solves;
use crate::multi::{BatchError, JoiningStatus};
use crate::topp::chain::ChainGrid;

const MAX_SWEEPS: u32 = 10;

pub(crate) fn join_until_converged(
    chain_grids: &[ChainGrid],
    states: &mut [ChainState],
    corner_caps: &[f64],
    n_threads: usize,
) -> Result<(u32, JoiningStatus), BatchError> {
    for sweep in 1..=MAX_SWEEPS {
        let dirty_count = bidirectional_junction_sweep(states, corner_caps);
        if dirty_count == 0 {
            if states.iter().all(|s| !s.dirty) {
                return Ok((sweep, JoiningStatus::Converged));
            }
            let last_dirty_count = states.iter().filter(|s| s.dirty).count();
            return Ok((
                sweep,
                JoiningStatus::StalledOnInfeasibleSegment { last_dirty_count },
            ));
        }
        fan_out_solves(chain_grids, states, n_threads)?;
    }
    let last_dirty = states.iter().filter(|s| s.dirty).count();
    Ok((
        MAX_SWEEPS,
        JoiningStatus::CappedAtMaxSweeps {
            last_dirty_count: last_dirty,
        },
    ))
}

pub(crate) struct ChainState {
    pub v_start: f64,
    pub v_end: f64,
    pub a_start: Option<f64>,
    pub profile: Option<TopProfile>,
    pub dirty: bool,
}

pub(crate) fn bidirectional_junction_sweep(
    states: &mut [ChainState],
    corner_caps: &[f64],
) -> usize {
    const EPS_VEL: f64 = 1e-3;
    let mut dirty_count = 0;

    for k in 0..corner_caps.len() {
        let target = corner_caps[k]
            .min(states[k].v_end)
            .min(states[k + 1].v_start);

        if (target - states[k].v_end).abs() > EPS_VEL {
            states[k].v_end = target;
            states[k].dirty = true;
            dirty_count += 1;
        }
        if (target - states[k + 1].v_start).abs() > EPS_VEL {
            states[k + 1].v_start = target;
            states[k + 1].dirty = true;
            dirty_count += 1;
        }
    }

    dirty_count
}

#[cfg(test)]
pub(crate) fn forward_sweep(states: &mut [ChainState], corner_caps: &[f64]) -> usize {
    const EPS_VEL: f64 = 1e-3;
    let mut dirty_count = 0;
    for k in 1..states.len() {
        let proposed_v_start = corner_caps[k - 1].min(states[k - 1].v_end);
        if (proposed_v_start - states[k].v_start).abs() > EPS_VEL {
            states[k].v_start = proposed_v_start;
            states[k].dirty = true;
            dirty_count += 1;
        }
    }
    dirty_count
}

#[cfg(test)]
pub(crate) fn reverse_sweep(states: &mut [ChainState], corner_caps: &[f64]) -> usize {
    const EPS_VEL: f64 = 1e-3;
    if states.len() < 2 {
        return 0;
    }
    let mut dirty_count = 0;
    for k in (0..states.len() - 1).rev() {
        let proposed_v_end = corner_caps[k].min(states[k + 1].v_start);
        if (proposed_v_end - states[k].v_end).abs() > EPS_VEL {
            states[k].v_end = proposed_v_end;
            states[k].dirty = true;
            dirty_count += 1;
        }
    }
    dirty_count
}

#[cfg(test)]
mod tests;
