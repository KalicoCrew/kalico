use crate::Limits;
use crate::topp::path::ArclengthGrid;
use crate::topp::solver::SolverResult;

/// Clarabel's QDLDL factorization loses enough precision to stall
/// (InsufficientProgress, false certificates) when b = v² reaches ~1e6 in raw
/// mm units; mapping the dominant v_max to ~10 units/s puts every machine
/// config in the empirically best-conditioned regime (b ≈ 1e2).
const V_TARGET_UNITS_PER_S: f64 = 10.0;

pub struct SolverScale {
    pub(crate) mm_per_unit: f64,
}

impl SolverScale {
    pub fn for_limits(limits: &Limits) -> Self {
        let sigma = limits
            .v_max
            .iter()
            .copied()
            .filter(|v| v.is_finite() && *v > 0.0)
            .fold(f64::NEG_INFINITY, f64::max);
        if sigma <= 0.0 || !sigma.is_finite() {
            return Self::identity();
        }
        Self {
            mm_per_unit: sigma / V_TARGET_UNITS_PER_S,
        }
    }

    pub fn identity() -> Self {
        Self { mm_per_unit: 1.0 }
    }

    pub(crate) fn sigma(&self) -> f64 {
        self.mm_per_unit
    }

    pub(crate) fn scale_limits(&self, limits: &Limits) -> Limits {
        let s = self.sigma();
        Limits {
            v_max: limits.v_max.map(|v| v / s),
            a_max: limits.a_max.map(|a| a / s),
            j_max: limits.j_max.map(|j| j / s),
            a_centripetal_max: limits.a_centripetal_max / s,
        }
    }

    pub(crate) fn scale_grid(&self, grid: &ArclengthGrid) -> ArclengthGrid {
        let s = self.sigma();
        ArclengthGrid {
            s: grid.s.iter().map(|v| v / s).collect(),
            u: grid.u.clone(),
            c: grid.c.iter().map(|p| p.map(|v| v / s)).collect(),
            c_prime: grid.c_prime.clone(),
            c_double_prime: grid.c_double_prime.iter().map(|p| p.map(|v| v * s)).collect(),
            c_triple_prime: grid
                .c_triple_prime
                .iter()
                .map(|p| p.map(|v| v * s * s))
                .collect(),
            kappa: grid.kappa.iter().map(|k| k * s).collect(),
            total_length: grid.total_length / s,
        }
    }

    pub(crate) fn scale_velocity(&self, v: f64) -> f64 {
        v / self.sigma()
    }

    pub(crate) fn unscale_result(&self, result: &mut SolverResult) {
        let s2 = self.sigma() * self.sigma();
        let s = self.sigma();
        for b in &mut result.b {
            *b *= s2;
        }
        for a in &mut result.a {
            *a *= s;
        }
    }

    pub(crate) fn unscale_b(&self, b: f64) -> f64 {
        b * self.sigma() * self.sigma()
    }

    pub(crate) fn to_scaled_b(&self, b: f64) -> f64 {
        let s2 = self.sigma() * self.sigma();
        b / s2
    }

    pub(crate) fn to_scaled_accel(&self, a: f64) -> f64 {
        a / self.sigma()
    }

    pub(crate) fn to_scaled_kappa(&self, kappa: f64) -> f64 {
        kappa * self.sigma()
    }

    pub fn for_chain(chain: &crate::topp::chain::ChainGrid) -> Self {
        let sigma = chain
            .limits
            .iter()
            .flat_map(|l| l.v_max.iter().copied())
            .filter(|v| v.is_finite() && *v > 0.0)
            .fold(f64::NEG_INFINITY, f64::max);
        if sigma <= 0.0 || !sigma.is_finite() {
            return Self::identity();
        }
        Self {
            mm_per_unit: sigma / V_TARGET_UNITS_PER_S,
        }
    }

    pub(crate) fn scale_chain_grid(
        &self,
        chain: &crate::topp::chain::ChainGrid,
    ) -> crate::topp::chain::ChainGrid {
        let s = self.sigma();
        let scale_geom = |g: &crate::topp::chain::PointGeom| crate::topp::chain::PointGeom {
            c_prime: g.c_prime,
            c_double_prime: g.c_double_prime.map(|v| v * s),
            c_triple_prime: g.c_triple_prime.map(|v| v * s * s),
            kappa: g.kappa * s,
        };
        crate::topp::chain::ChainGrid {
            s: chain.s.iter().map(|v| v / s).collect(),
            geom: chain.geom.iter().map(scale_geom).collect(),
            h_intervals: chain.h_intervals.iter().map(|h| h / s).collect(),
            limits_idx: chain.limits_idx.clone(),
            limits: chain.limits.iter().map(|l| self.scale_limits(l)).collect(),
            junctions: chain
                .junctions
                .iter()
                .map(|j| crate::topp::chain::JunctionDual {
                    idx: j.idx,
                    geom: scale_geom(&j.geom),
                    limits_idx: j.limits_idx,
                })
                .collect(),
            segment_ranges: chain.segment_ranges.clone(),
        }
    }
}

#[cfg(test)]
mod tests;
