use crate::multi::junction::JunctionKind;
use crate::{GridSample, TopProfile};
use std::ops::RangeInclusive;

/// Maximal runs of segments joined by Smooth junctions. `kinds[k]` is the
/// junction between segments k and k+1.
pub(crate) fn partition_chains(
    n_segments: usize,
    kinds: &[JunctionKind],
) -> Vec<RangeInclusive<usize>> {
    debug_assert_eq!(kinds.len() + 1, n_segments);
    let mut chains = Vec::new();
    let mut start = 0;
    for (k, kind) in kinds.iter().enumerate() {
        if *kind == JunctionKind::Corner {
            chains.push(start..=k);
            start = k + 1;
        }
    }
    chains.push(start..=n_segments - 1);
    chains
}

/// Slice one chain profile into per-segment profiles. The junction sample is
/// duplicated into both neighbors; per-segment `s` is rebased to start at 0;
/// per-segment time is the trapezoid over the slice (same formula as
/// output::assemble).
pub(crate) fn slice_chain_profile(
    chain: &TopProfile,
    segment_ranges: &[(usize, usize)],
) -> Vec<TopProfile> {
    segment_ranges
        .iter()
        .map(|&(lo, hi)| {
            let s0 = chain.samples[lo].s;
            let samples: Vec<GridSample> = chain.samples[lo..=hi]
                .iter()
                .map(|smp| GridSample { s: smp.s - s0, ..*smp })
                .collect();
            let mut total_time = 0.0;
            for w in samples.windows(2) {
                let ds = w[1].s - w[0].s;
                let v_sum = w[0].v + w[1].v;
                total_time += if v_sum > 1e-12 {
                    ds * 2.0 / v_sum
                } else {
                    ds / 1e-9_f64.max(w[0].v.max(w[1].v))
                };
            }
            TopProfile {
                samples,
                status: chain.status,
                grid_scheme: chain.grid_scheme,
                total_time,
            }
        })
        .collect()
}

#[cfg(test)]
mod tests;
