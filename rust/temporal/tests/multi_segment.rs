//! Layer 2 multi-segment integration tests. Per spec §5.1.

use nurbs::VectorNurbs;
use temporal::{
    plan_batch, BatchInput, GridStrategy, JoiningStatus, JunctionBindingCap,
    Limits, SegmentInput,
};

fn textbook_limits() -> Limits {
    // Use Limits::new(...) — `Limits` is `#[non_exhaustive]` (Task 0), so
    // struct-literal construction is forbidden across the integration-test
    // crate boundary. Per review-2 (Codex BLOCKER + kalico-plan-reviewer
    // advisory).
    Limits::new(
        [500.0; 3],
        [5_000.0; 3],
        [100_000.0; 3],
        2_500.0,
    )
}

fn adaptive() -> GridStrategy {
    GridStrategy::Adaptive { min_n: 10, max_n: 200, target_grid_spacing_mm: 0.5 }
}

/// Spec §6.2 acceptance: every junction's `v_end[k]` ≈ `v_start[k+1]` ≈ `v_junction`
/// within `ε_velocity` = 1 mm/s. Reusable across all multi-segment fixtures
/// (review-1 finding F9: previously only fixture 1 enforced this).
fn assert_junction_continuity_for_all(
    output: &temporal::BatchOutput,
    eps_mm_s: f64,
) {
    for (k, junction) in output.junctions.iter().enumerate() {
        let v_jct = junction.v_junction;
        let v_end_left = output.profiles[k].samples.last().unwrap().v;
        let v_start_right = output.profiles[k + 1].samples[0].v;
        assert!(
            (v_end_left - v_jct).abs() < eps_mm_s,
            "junction {k}: v_end_left={v_end_left} vs v_jct={v_jct} (ε={eps_mm_s})",
        );
        assert!(
            (v_start_right - v_jct).abs() < eps_mm_s,
            "junction {k}: v_start_right={v_start_right} vs v_jct={v_jct} (ε={eps_mm_s})",
        );
    }
}

mod fixture_1_two_g1_sharp_corner {
    use super::*;

    #[test]
    fn fixture_1() {
        let left = VectorNurbs::<f64, 3>::try_new(
            1, vec![0.0, 0.0, 1.0, 1.0],
            vec![[0.0, 0.0, 0.0], [50.0, 0.0, 0.0]], None,
        ).unwrap();
        let right = VectorNurbs::<f64, 3>::try_new(
            1, vec![0.0, 0.0, 1.0, 1.0],
            vec![[50.0, 0.0, 0.0], [50.0, 50.0, 0.0]], None,
        ).unwrap();
        let limits = textbook_limits();
        let segments = [
            SegmentInput { curve: &left, limits, trailing_junction_chord_tolerance_mm: 0.05 },
            SegmentInput { curve: &right, limits, trailing_junction_chord_tolerance_mm: 0.05 },
        ];
        let input = BatchInput { segments: &segments, grid_strategy: adaptive(), worker_threads: 3 };
        let output = plan_batch(input).expect("should succeed");

        // Acceptance §6.1: each profile passes its own per-segment feasibility check
        // (already enforced by schedule_segment -> verify::check).
        assert_eq!(output.profiles.len(), 2);

        // Acceptance §6.2: junction continuity. v_end of seg 0 ≈ v_start of seg 1 ≈ v_junction.
        // Use shared helper (review-1 finding F9).
        assert_junction_continuity_for_all(&output, 1.0);
        let v_jct = output.junctions[0].v_junction;

        // §6.2: sharp-corner cap. Expected ≈ sqrt(2500 · 0.05 · 2.414) ≈ 17.4 mm/s.
        let expected = (2_500.0_f64 * 0.05 * 2.414_213_562).sqrt();
        assert!((v_jct - expected).abs() < 0.1, "v_jct {v_jct} vs expected {expected}");
        assert!(matches!(output.junctions[0].binding_cap, JunctionBindingCap::SharpCornerChord));

        // §6.5: convergence in ≤3 sweeps.
        assert!(output.joining_sweeps <= 3);
        assert!(matches!(output.joining_status, JoiningStatus::Converged));
    }
}
