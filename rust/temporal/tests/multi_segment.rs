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
        let expected = (2_500.0_f64 * 0.05 * 2.414_213_562).sqrt(); // 2.414... = 1/(1 - cos(π/4)) for a 90° deviation angle (spec §2.2)
        assert!((v_jct - expected).abs() < 0.1, "v_jct {v_jct} vs expected {expected}");
        assert!(matches!(output.junctions[0].binding_cap, JunctionBindingCap::SharpCornerChord));

        // §6.5: convergence in ≤3 sweeps.
        assert!(output.joining_sweeps <= 3);
        assert!(matches!(output.joining_status, JoiningStatus::Converged));
    }
}

mod fixture_2_g1_to_g5_smooth {
    use super::*;

    #[test]
    fn fixture_2() {
        // G1 ending at (50, 0, 0) with tangent +X.
        let left = VectorNurbs::<f64, 3>::try_new(
            1, vec![0.0, 0.0, 1.0, 1.0],
            vec![[0.0, 0.0, 0.0], [50.0, 0.0, 0.0]], None,
        ).unwrap();
        // Cubic G5-style with tangent matching at u=0 (also +X), curving away.
        let right = VectorNurbs::<f64, 3>::try_new(
            3, vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![
                [50.0, 0.0, 0.0],
                [60.0, 0.0, 0.0],     // CP1: tangent direction at u=0 = +X (matches left)
                [70.0, 30.0, 0.0],
                [100.0, 50.0, 0.0],
            ], None,
        ).unwrap();
        let limits = textbook_limits();
        let segments = [
            SegmentInput { curve: &left, limits, trailing_junction_chord_tolerance_mm: 0.05 },
            SegmentInput { curve: &right, limits, trailing_junction_chord_tolerance_mm: 0.05 },
        ];
        let input = BatchInput { segments: &segments, grid_strategy: adaptive(), worker_threads: 3 };
        let output = plan_batch(input).expect("should succeed");

        // §6.2: smooth-κ branch. Junction κ on right side > 0 (G5 has curvature at u=0).
        let j = &output.junctions[0];
        assert!(j.kappa_right.abs() > 1e-6, "G5 should have nonzero κ at u=0, got {}", j.kappa_right);
        // Expect Centripetal cap, not SharpCornerChord.
        assert!(matches!(j.binding_cap, JunctionBindingCap::Centripetal | JunctionBindingCap::PerAxisVelocity | JunctionBindingCap::GlobalVMax),
            "smooth junction should not trigger SharpCornerChord, got {:?}", j.binding_cap);

        assert!(output.joining_sweeps <= 3);
        assert!(matches!(output.joining_status, JoiningStatus::Converged));

        // §6.2 (review-1 helper): junction continuity.
        assert_junction_continuity_for_all(&output, 1.0);
    }
}

mod fixture_3_long_straight_then_corner {
    use super::*;
    use temporal::{schedule_segment_with_tolerance, GridConfig, GridScheme, ToleranceMode};

    #[test]
    fn fixture_3() {
        let straight = VectorNurbs::<f64, 3>::try_new(
            1, vec![0.0, 0.0, 1.0, 1.0],
            vec![[0.0, 0.0, 0.0], [100.0, 0.0, 0.0]], None,
        ).unwrap();
        let corner_right = VectorNurbs::<f64, 3>::try_new(
            1, vec![0.0, 0.0, 1.0, 1.0],
            vec![[100.0, 0.0, 0.0], [100.0, 50.0, 0.0]], None,
        ).unwrap();
        let limits = textbook_limits();
        let segments = [
            SegmentInput { curve: &straight, limits, trailing_junction_chord_tolerance_mm: 0.05 },
            SegmentInput { curve: &corner_right, limits, trailing_junction_chord_tolerance_mm: 0.05 },
        ];
        let input = BatchInput { segments: &segments, grid_strategy: adaptive(), worker_threads: 3 };
        let output = plan_batch(input).expect("should succeed");

        // §6.3 lookahead: profile of seg 0 at u=1 has v < v_max (decel happening).
        let v_end_seg0 = output.profiles[0].samples.last().unwrap().v;
        assert!(v_end_seg0 < 499.0, "seg 0 should be braking, v_end = {v_end_seg0}");

        // §6.3: total time of seg 0 in joined batch > seg 0 in isolation with v_end=v_max.
        // Solve seg 0 alone with v_end=v_max for comparison.
        let solo_grid = GridConfig {
            scheme: GridScheme::UniformArclength,
            n: 200,  // fixed grid for the comparison solve
        };
        let solo = schedule_segment_with_tolerance(
            &straight, &limits, &solo_grid, 0.0, 500.0, ToleranceMode::Auto,
        ).expect("solo solve");
        let t_joined = output.profiles[0].total_time;
        let t_solo = solo.total_time;
        assert!(t_joined > t_solo,
            "joined seg 0 should take longer (decel for corner): joined={t_joined} solo={t_solo}");

        // §6.2 (review-2 fix): junction continuity helper applied to fixture 3
        // too — has 2 segments + 1 junction, same as fixture 1.
        assert_junction_continuity_for_all(&output, 1.0);

        // §6.5 convergence (review-2 fix): fixture 3 should also satisfy ≤3 sweeps.
        assert!(output.joining_sweeps <= 3,
            "lookahead fixture should converge in ≤3 sweeps");
        assert!(matches!(output.joining_status, temporal::JoiningStatus::Converged));
    }
}

mod fixture_4_per_segment_limits_change {
    use super::*;

    #[test]
    fn fixture_4() {
        let segments_curves: Vec<_> = (0..3_usize).map(|i| {
            VectorNurbs::<f64, 3>::try_new(
                1, vec![0.0, 0.0, 1.0, 1.0],
                vec![
                    [i as f64 * 50.0, 0.0, 0.0],
                    [(i + 1) as f64 * 50.0, 0.0, 0.0],
                ], None,
            ).unwrap()
        }).collect();
        let normal_limits = textbook_limits();
        let mut reduced_limits = normal_limits;
        reduced_limits.a_max = [2_500.0; 3];  // halved a_max for seg 1
        let segments = [
            SegmentInput { curve: &segments_curves[0], limits: normal_limits, trailing_junction_chord_tolerance_mm: 0.05 },
            SegmentInput { curve: &segments_curves[1], limits: reduced_limits, trailing_junction_chord_tolerance_mm: 0.05 },
            SegmentInput { curve: &segments_curves[2], limits: normal_limits, trailing_junction_chord_tolerance_mm: 0.05 },
        ];
        let input = BatchInput { segments: &segments, grid_strategy: adaptive(), worker_threads: 3 };
        let output = plan_batch(input).expect("should succeed");

        // §6.4: seg 1 profile peak |s̈| ≤ 2500 (1+ε).
        let max_a_seg1 = output.profiles[1].samples.iter()
            .map(|s| s.a.abs())
            .fold(0.0_f64, f64::max);
        assert!(max_a_seg1 <= 2_500.0 * 1.001,
            "seg 1 peak accel {max_a_seg1} exceeds reduced a_max 2500");

        // §6.4 (review-1 fix): seg 0 / seg 2 actually reach textbook a_max,
        // confirming they're using their own (looser) limits, not the reduced
        // ones from seg 1. If joining incorrectly propagated reduced limits
        // outside seg 1's range, this would catch it.
        let max_a_seg0 = output.profiles[0].samples.iter()
            .map(|s| s.a.abs())
            .fold(0.0_f64, f64::max);
        let max_a_seg2 = output.profiles[2].samples.iter()
            .map(|s| s.a.abs())
            .fold(0.0_f64, f64::max);
        // Sanity: seg 0/2 peak accel should be much closer to 5000 (textbook)
        // than to 2500 (reduced). Allow 5% slack for adaptive-N quantization.
        assert!(max_a_seg0 > 2_500.0 * 1.5,
            "seg 0 peak accel {max_a_seg0} suggests reduced limits leaked outside seg 1");
        assert!(max_a_seg2 > 2_500.0 * 1.5,
            "seg 2 peak accel {max_a_seg2} suggests reduced limits leaked outside seg 1");

        // §6.2 (review-1 helper): junction continuity at both interior junctions.
        assert_junction_continuity_for_all(&output, 1.0);

        // §6.5 convergence (review-2 fix): fixture 4 also expects ≤3 sweeps.
        assert!(output.joining_sweeps <= 3);
        assert!(matches!(output.joining_status, JoiningStatus::Converged));
    }
}

mod fixture_5_star_pattern {
    use super::*;

    #[test]
    fn fixture_5() {
        // 5-pointed star: 5 segments, alternating outward-spike + inward-cusp.
        // Use 5 short G1 segments forming a star-like pattern.
        let r_outer: f64 = 30.0;
        let r_inner: f64 = 12.0;
        let n_points = 5_usize;
        let mut points: Vec<[f64; 3]> = Vec::new();
        for i in 0..n_points * 2 {
            let theta = i as f64 * std::f64::consts::PI / n_points as f64;
            let r = if i % 2 == 0 { r_outer } else { r_inner };
            points.push([r * theta.cos(), r * theta.sin(), 0.0]);
        }
        let curves: Vec<_> = points.windows(2).map(|w| {
            VectorNurbs::<f64, 3>::try_new(
                1, vec![0.0, 0.0, 1.0, 1.0],
                vec![w[0], w[1]], None,
            ).unwrap()
        }).collect();
        let limits = textbook_limits();
        let segments: Vec<_> = curves.iter().map(|c| SegmentInput {
            curve: c, limits, trailing_junction_chord_tolerance_mm: 0.05,
        }).collect();
        let input = BatchInput { segments: &segments, grid_strategy: adaptive(), worker_threads: 3 };
        let output = plan_batch(input).expect("should succeed");

        // §6.5: converges in ≤5 sweeps.
        assert!(output.joining_sweeps <= 5, "joining took {} sweeps", output.joining_sweeps);
        assert!(matches!(output.joining_status, JoiningStatus::Converged));

        // §6.2 (review-1 helper): junction continuity at every junction.
        // Star pattern: 10 points → 9 segments → 8 junctions.
        assert_junction_continuity_for_all(&output, 1.0);
    }
}

mod fixture_6_long_realistic_chain {
    use super::*;
    use std::time::Instant;

    fn realistic_machine_limits() -> Limits {
        // Limits::new because integration tests are external to temporal crate
        // (review-2 fix).
        Limits::new(
            [1_000.0; 3],
            [65_000.0; 3],
            [50_000_000.0; 3],
            65_000.0,
        )
    }

    #[test]
    fn fixture_6() {
        // 10 segments: 6 G1 straights + 2 G5 cubics + 2 G2 quarter-arcs.
        // All segments are geometrically connected end-to-end.
        // Minimum segment length chosen so v_jct=1000, a_max=65000 is reachable
        // (requires ≥ 7.7 mm to accelerate from 0 to 1000 mm/s at 65k mm/s²).
        let mut curves: Vec<VectorNurbs<f64, 3>> = Vec::new();

        // Track current position — ensures connectivity across all segment types.
        let mut px = 0.0_f64;
        let mut py = 0.0_f64;

        // 6 G1 straights along +X, lengths 20..45 mm.
        for i in 0..6_usize {
            let len = 20.0 + i as f64 * 5.0;
            curves.push(VectorNurbs::<f64, 3>::try_new(
                1, vec![0.0, 0.0, 1.0, 1.0],
                vec![[px, py, 0.0], [px + len, py, 0.0]], None,
            ).unwrap());
            px += len;
        }

        // 2 G5 cubics: go out +Y by 20 mm and return, length ~40 mm each.
        for _ in 0..2 {
            let p0 = [px, py, 0.0];
            let p1 = [px + 10.0, py + 20.0, 0.0];
            let p2 = [px + 30.0, py + 20.0, 0.0];
            let p3 = [px + 40.0, py, 0.0];
            curves.push(VectorNurbs::<f64, 3>::try_new(
                3, vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
                vec![p0, p1, p2, p3], None,
            ).unwrap());
            px += 40.0;
        }

        // 2 G2 quarter-arcs: rational quadratic, radius 20 mm.
        // Each arc goes from [px, py] toward [px+20, py+20] (quarter-circle in +X/+Y).
        // Endpoint of arc: [px+20, py+20]. The intermediate CP is [px+20, py] (right angle).
        // After each arc we advance both px by 20 and py by 20.
        let w = std::f64::consts::FRAC_1_SQRT_2;
        for _ in 0..2 {
            let p0 = [px, py, 0.0];
            let p_mid = [px + 20.0, py, 0.0];
            let p2 = [px + 20.0, py + 20.0, 0.0];
            curves.push(VectorNurbs::<f64, 3>::try_new(
                2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
                vec![p0, p_mid, p2],
                Some(vec![1.0, w, 1.0]),
            ).unwrap());
            // Arc endpoint is p2; advance current position there.
            px += 20.0;
            py += 20.0;
        }

        let limits = realistic_machine_limits();
        let segments: Vec<_> = curves.iter().map(|c| SegmentInput {
            curve: c, limits, trailing_junction_chord_tolerance_mm: 0.05,
        }).collect();
        let input = BatchInput { segments: &segments, grid_strategy: adaptive(), worker_threads: 3 };

        let t0 = Instant::now();
        let output = plan_batch(input).expect("should succeed");
        let elapsed_ms = t0.elapsed().as_secs_f64() * 1000.0;

        // §6.5: convergence in ≤3 sweeps.
        assert!(output.joining_sweeps <= 3);

        // §6.2 (review-1 helper): junction continuity at every junction.
        assert_junction_continuity_for_all(&output, 1.0);

        // §6.6: performance sanity log (not acceptance). Expect <100ms on Pi 5.
        eprintln!("fixture_6 wall-clock: {elapsed_ms:.2} ms (no acceptance threshold)");
    }
}
