use super::*;
use crate::topp::path::ArclengthGrid;
use crate::topp::solver::{SolverResult, SolverStatus};
use crate::topp::verify::VerifyReport;
use crate::{BindingConstraint, GridConfig, GridScheme};

fn dummy_grid(n: usize, length: f64) -> ArclengthGrid {
    let s: Vec<f64> = (0..n).map(|i| length * i as f64 / (n - 1) as f64).collect();
    let u = s.clone();
    let c = s.iter().map(|si| [*si, 0.0, 0.0]).collect();
    let c_prime = vec![[1.0, 0.0, 0.0]; n];
    let c_double_prime = vec![[0.0, 0.0, 0.0]; n];
    let c_triple_prime = vec![[0.0, 0.0, 0.0]; n];
    let kappa = vec![0.0; n];
    ArclengthGrid {
        s,
        u,
        c,
        c_prime,
        c_double_prime,
        c_triple_prime,
        kappa,
        total_length: length,
    }
}

#[test]
fn assembles_samples_and_total_time() {
    let grid = dummy_grid(3, 10.0);
    let result = SolverResult {
        b: vec![0.0, 100.0, 0.0],
        a: vec![10.0, 0.0, -10.0],
        status: SolverStatus::Solved,
    };
    let verify = VerifyReport {
        binding_per_grid: vec![
            BindingConstraint::Boundary,
            BindingConstraint::None,
            BindingConstraint::Boundary,
        ],
        worst_violation: 0.0,
        worst_violation_grid: 0,
        feasible: true,
    };
    let cfg = GridConfig {
        scheme: GridScheme::UniformArclength,
        n: 3,
    };
    let p = assemble(
        &grid.s,
        &result,
        &verify,
        cfg,
        SlpOutcome::Converged { outer_iters: 0 },
    );
    assert_eq!(p.samples.len(), 3);
    assert!((p.samples[1].v - 10.0).abs() < 1e-9);
    assert!(matches!(p.status, SolveStatus::Solved));
    // Trapezoidal time over the two intervals: 2·5/(0+10) + 2·5/(10+0) = 2.0 s.
    assert!((p.total_time - 2.0).abs() < 1e-9);
}
