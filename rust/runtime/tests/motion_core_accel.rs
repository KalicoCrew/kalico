#![allow(
    clippy::cast_sign_loss,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::float_cmp
)]

use runtime::motion_core::{eval_accel, eval_horner};
use runtime::piece_ring::PieceEntry;

fn entry() -> PieceEntry {
    PieceEntry {
        start_time: 5_000_000,
        coeffs: [1.0, 2.5, -3.0, 4.0],
        duration: 0.5,
        _reserved: 0,
    }
}

#[test]
fn accel_matches_velocity_finite_difference() {
    let (mono, vel) = entry().to_monomial();
    let cps = 1.0e9_f32;
    let start = entry().start_time;
    for &t_s in &[0.0_f32, 0.05, 0.2, 0.45] {
        let now = start + (t_s * cps) as u64;
        let h_cycles = 1_000_u64;
        let h_s = h_cycles as f32 / cps;
        let (_, v0) = eval_horner(&mono, &vel, start, now, cps);
        let (_, v1) = eval_horner(&mono, &vel, start, now + h_cycles, cps);
        let fd = (v1 - v0) / h_s;
        let a = eval_accel(&vel, start, now, cps);
        assert!(
            (a - fd).abs() <= 0.05 * fd.abs().max(1.0),
            "t={t_s}: accel {a} vs finite-diff {fd}"
        );
    }
}

#[test]
fn accel_has_no_quadratic_term() {
    let (_, vel) = entry().to_monomial();
    let cps = 1.0e9_f32;
    let a0 = eval_accel(&vel, 0, 0, cps);
    let a1 = eval_accel(&vel, 0, (0.1 * cps) as u64, cps);
    let a2 = eval_accel(&vel, 0, (0.2 * cps) as u64, cps);
    assert!((a2 - a1 - (a1 - a0)).abs() < 1e-3, "{a0} {a1} {a2}");
}

#[test]
fn accel_clamps_before_piece_start() {
    let (_, vel) = entry().to_monomial();
    assert_eq!(eval_accel(&vel, 1000, 500, 1.0e9), vel[1]);
}
