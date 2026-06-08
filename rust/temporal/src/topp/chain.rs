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
}

const MAX_JUNCTION_SPACING_RATIO: f64 = 16.0;

impl ChainGrid {
    /// Concatenate per-segment grids into one chain. Adjacent grids must be
    /// geometrically continuous (the caller guarantees tangent continuity —
    /// that's what made them one chain). Panics on empty input: an empty
    /// chain is a caller bug.
    pub fn from_segment_grids(grids: Vec<ArclengthGrid>, limits: Vec<Limits>) -> Self {
        assert_eq!(grids.len(), limits.len());
        assert!(!grids.is_empty(), "empty chain");

        for j_idx in 1..grids.len() {
            let hl = grids[j_idx - 1].s[1] - grids[j_idx - 1].s[0];
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
        let mut s_offset = 0.0;

        for (seg, g) in grids.iter().enumerate() {
            let n = g.s.len();
            debug_assert!(n >= 2);
            let h_seg = g.s[1] - g.s[0];
            let start_point = if seg == 0 { 0 } else { 1 };
            let range_start = if seg == 0 { 0 } else { s.len() - 1 };

            if seg > 0 {
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
            for _ in 0..n - 1 {
                h_intervals.push(h_seg);
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
