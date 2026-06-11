use crate::Limits;
use crate::topp::path::ArclengthGrid;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PointGeom {
    pub c_prime: [f64; 3],
    pub c_double_prime: [f64; 3],
    pub c_triple_prime: [f64; 3],
    pub kappa: f64,
}

/// Right-side geometry/limits of a shared junction point. The primary
/// per-point arrays carry the left side.
#[derive(Debug, Clone, Copy)]
pub struct JunctionDual {
    pub idx: usize,
    pub geom: PointGeom,
    pub limits_idx: usize,
}

#[derive(Debug, Clone)]
pub struct ChainGrid {
    /// Cumulative arclength along the chain, len M.
    pub s: Vec<f64>,
    pub geom: Vec<PointGeom>,
    /// Per-interval spacing, len M−1. Uniform within a segment, changes at
    /// junction indices.
    pub h_intervals: Vec<f64>,
    /// Index into `limits` per point (junction points → left segment).
    pub limits_idx: Vec<usize>,
    pub limits: Vec<Limits>,
    pub junctions: Vec<JunctionDual>,
    /// Inclusive (start, end) point-index range per segment; consecutive
    /// ranges share their boundary index.
    pub segment_ranges: Vec<(usize, usize)>,
    /// Interior κ samples for each chain interval `[i, i+1]`, len M−1.
    /// Each entry is a vec of `(θ, κ)` pairs with θ ∈ (0,1).
    pub inter_kappa: Vec<Vec<(f64, f64)>>,
}

pub(crate) const MAX_JUNCTION_SPACING_RATIO: f64 = 16.0;

impl ChainGrid {
    pub fn from_segment_grids(grids: Vec<ArclengthGrid>, limits: Vec<Limits>) -> Self {
        let n = grids.len();
        Self::from_segment_grids_with_absorbed(grids, limits, &vec![false; n])
    }

    /// Concatenate per-segment grids into one chain. Adjacent grids must be
    /// geometrically continuous (the caller guarantees tangent continuity —
    /// that's what made them one chain). Segments marked in `absorbed` have
    /// their arclength folded into a single degenerate interval shared with
    /// the preceding segment; they contribute no interior grid nodes. Panics
    /// on empty input: an empty chain is a caller bug.
    pub fn from_segment_grids_with_absorbed(
        grids: Vec<ArclengthGrid>,
        limits: Vec<Limits>,
        absorbed: &[bool],
    ) -> Self {
        assert_eq!(grids.len(), limits.len());
        assert_eq!(grids.len(), absorbed.len());
        assert!(!grids.is_empty(), "empty chain");

        for j_idx in 1..grids.len() {
            if absorbed[j_idx] {
                continue;
            }
            let Some(prev_non_absorbed) = (0..j_idx).rev().find(|&k| !absorbed[k]) else {
                continue;
            };
            let hl = grids[prev_non_absorbed].s[1] - grids[prev_non_absorbed].s[0];
            let hr = grids[j_idx].s[1] - grids[j_idx].s[0];
            let ratio = (hl / hr).max(hr / hl);
            assert!(
                ratio <= MAX_JUNCTION_SPACING_RATIO,
                "junction spacing ratio {ratio:.1} (hl={hl:.4}, hr={hr:.4}) — \
                 grid construction bug; the non-uniform stencil conditioning \
                 degrades with the spacing ratio"
            );
        }

        let mut s = Vec::new();
        let mut geom = Vec::new();
        let mut h_intervals = Vec::new();
        let mut limits_idx = Vec::new();
        let mut junctions = Vec::new();
        let mut segment_ranges = Vec::new();
        let mut inter_kappa = Vec::new();
        let mut s_offset = 0.0;

        for (seg, g) in grids.iter().enumerate() {
            let n = g.s.len();
            debug_assert!(n >= 2);

            if absorbed[seg] {
                let pinned = if s.is_empty() { 0 } else { s.len() - 1 };
                segment_ranges.push((pinned, pinned));
                s_offset += g.total_length;
                continue;
            }

            let h_seg = g.s[1] - g.s[0];
            let is_first_real = s.is_empty();
            let start_point = if is_first_real { 0 } else { 1 };
            let range_start = if is_first_real { 0 } else { s.len() - 1 };

            if !is_first_real {
                junctions.push(JunctionDual {
                    idx: s.len() - 1,
                    geom: point_geom(g, 0),
                    limits_idx: seg,
                });
            }
            for i in start_point..n {
                s.push(s_offset + g.s[i]);
                geom.push(point_geom(g, i));
                limits_idx.push(seg);
            }
            for interval_idx in 0..n - 1 {
                h_intervals.push(h_seg);
                inter_kappa.push(g.inter_kappa[interval_idx].clone());
            }
            segment_ranges.push((range_start, s.len() - 1));
            s_offset += g.total_length;
        }

        Self {
            s,
            geom,
            h_intervals,
            limits_idx,
            limits,
            junctions,
            segment_ranges,
            inter_kappa,
        }
    }

    pub fn n_points(&self) -> usize {
        self.s.len()
    }

    pub fn limits_at(&self, i: usize) -> &Limits {
        &self.limits[self.limits_idx[i]]
    }
}

fn point_geom(g: &ArclengthGrid, i: usize) -> PointGeom {
    PointGeom {
        c_prime: g.c_prime[i],
        c_double_prime: g.c_double_prime[i],
        c_triple_prime: g.c_triple_prime[i],
        kappa: g.kappa[i],
    }
}

#[cfg(test)]
pub(crate) mod tests_support;

#[cfg(test)]
mod tests;
