// Tests for the streaming-shaper module.
//
// Split out of the per-file inline `#[cfg(test)] mod tests` after the module
// directory split. Covers:
//
// - Phase 1 byte-identity contract (`shim_matches_direct_pipeline_*`).
// - Construction seeding (`new_seeds_axis_queues_*`).
// - Phase 3 Task 3.1 `append_and_replan` semantics.
// - Phase 3 Task 3.2 `emit_committed` semantics.

use geometry::segment::CubicSegment;
use nurbs::algebra::PiecewisePolynomialKernel;
use nurbs::bezier::{bezier_pieces_to_nurbs, BezierPiece};
use nurbs::ScalarNurbs;

use super::{EmitContext, ReplanContext, ShaperState};
use crate::fit::FittedSegment;
use crate::kernel::build_smooth_zv_kernel;
use crate::pad::{pad_segment_axis, EHalo};
use crate::plan_velocity::{PlanShaper, SafetyMode};
use crate::refit::{refit_to_cubic, REFIT_TOLERANCE_MM};
use crate::shaper::shape_axis;
use crate::{AxisShaper, ELimits, RequiredShaper};

// ---------------------------------------------------------------------------
// Shared helpers
// ---------------------------------------------------------------------------

/// Build a simple linear-move `FittedSegment`: X linear from 0 → 10,
/// Y and Z constant at 0, on `t ∈ [0, 1]`.
fn linear_segment() -> FittedSegment {
    let x_nurbs = bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![0.0, 10.0],
    }]);
    let y_nurbs = bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![0.0],
    }]);
    let z_nurbs = bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![0.0],
    }]);
    FittedSegment {
        axes: [x_nurbs, y_nurbs, z_nurbs],
        t_start: 0.0,
        t_end: 1.0,
    }
}

/// Byte-equivalent NURBS comparator: same degree, same knots, same
/// control points, same weight presence. We compare with `==` on `f64`
/// (NaN-free in this pipeline) to make "byte-equivalent" literal.
fn assert_nurbs_byte_equal(a: &ScalarNurbs<f64>, b: &ScalarNurbs<f64>, label: &str) {
    assert_eq!(a.degree(), b.degree(), "{label}: degree differs");
    assert_eq!(a.knots(), b.knots(), "{label}: knots differ");
    assert_eq!(
        a.control_points(),
        b.control_points(),
        "{label}: control points differ"
    );
    assert_eq!(
        a.weights().is_some(),
        b.weights().is_some(),
        "{label}: weight presence differs"
    );
    if let (Some(wa), Some(wb)) = (a.weights(), b.weights()) {
        assert_eq!(wa, wb, "{label}: weights differ");
    }
}

// ---------------------------------------------------------------------------
// Phase 1 byte-identity tests
// ---------------------------------------------------------------------------

#[test]
#[allow(clippy::float_cmp)] // Time bounds and cursor zeros are exact-by-construction.
fn shim_matches_direct_pipeline_for_single_linear_move() {
    let fitted = linear_segment();
    let freq = 60.0;
    let h = 0.8025 / freq / 2.0;
    let kernel = build_smooth_zv_kernel(0.8025 / freq);

    // ---- Method A: streaming shim ----
    let shapers: [Option<AxisShaper>; 4] = [
        Some(AxisShaper::SmoothZv { frequency_hz: freq }),
        Some(AxisShaper::SmoothZv { frequency_hz: freq }),
        Some(AxisShaper::Passthrough),
        Some(AxisShaper::Passthrough),
    ];
    let mut state = ShaperState::new([0.0, 0.0, 0.0, 0.0], &shapers);
    state.append_batch(&fitted).expect("shim should succeed");
    let shim_out = state.drain_committed();
    assert_eq!(shim_out.len(), 1, "shim should emit exactly one segment");
    let shim_seg = &shim_out[0];

    // After draining, `pending_dispatch` is empty.
    assert!(state.pending_dispatch.is_empty());
    // Re-draining yields nothing.
    assert!(state.drain_committed().is_empty());

    // ---- Method B: direct call sequence (mirrors `beta::run_one_iteration`) ----
    let fitted_slice = std::slice::from_ref(&fitted);

    // X: shaped + refit.
    let x_padded = pad_segment_axis(0, 0, fitted_slice, &[], h, 0.0, 1.0);
    let x_shaped = shape_axis(&x_padded, &kernel, 0.0, 1.0).unwrap();
    let x_refit = refit_to_cubic(&x_shaped, REFIT_TOLERANCE_MM).unwrap();

    // Y: shaped + refit (Y also SmoothZv at the same freq → same kernel).
    let y_padded = pad_segment_axis(0, 1, fitted_slice, &[], h, 0.0, 1.0);
    let y_shaped = shape_axis(&y_padded, &kernel, 0.0, 1.0).unwrap();
    let y_refit = refit_to_cubic(&y_shaped, REFIT_TOLERANCE_MM).unwrap();

    // Z: passthrough → still refit.
    let z_passthrough = fitted.axes[2].clone();
    let z_refit = refit_to_cubic(&z_passthrough, REFIT_TOLERANCE_MM).unwrap();

    // ---- Compare byte-for-byte ----
    assert_nurbs_byte_equal(&shim_seg.axes[0], &x_refit, "X");
    assert_nurbs_byte_equal(&shim_seg.axes[1], &y_refit, "Y");
    assert_nurbs_byte_equal(&shim_seg.axes[2], &z_refit, "Z");

    // Time bounds match the input.
    assert_eq!(shim_seg.t_start, 0.0);
    assert_eq!(shim_seg.t_end, 1.0);
}

#[test]
#[allow(clippy::float_cmp)] // Cursor zeros and h=0 for passthrough are exact-by-construction.
fn new_seeds_axis_queues_with_rest_extension() {
    let shapers: [Option<AxisShaper>; 4] = [
        Some(AxisShaper::SmoothZv {
            frequency_hz: 100.0,
        }),
        Some(AxisShaper::SmoothMzv {
            frequency_hz: 80.0,
        }),
        Some(AxisShaper::Passthrough),
        None,
    ];
    let state = ShaperState::new([1.0, 2.0, 3.0, 4.0], &shapers);

    // Active axes get a single seed piece spanning `2h` of the past
    // (`δ_safety = h`).
    let h_x = 0.8025 / 100.0 / 2.0;
    assert_eq!(state.axes[0].pieces.len(), 1);
    let seed_x = &state.axes[0].pieces[0];
    assert!((seed_x.u_start - (-2.0 * h_x)).abs() < 1e-15);
    assert_eq!(seed_x.u_end, 0.0);
    assert_eq!(seed_x.coeffs, vec![1.0]);
    assert!((state.axes[0].h - h_x).abs() < 1e-15);
    assert!(state.axes[0].kernel.is_some());

    let h_y = 0.95625 / 80.0 / 2.0;
    assert_eq!(state.axes[1].pieces.len(), 1);
    let seed_y = &state.axes[1].pieces[0];
    assert!((seed_y.u_start - (-2.0 * h_y)).abs() < 1e-15);
    assert_eq!(seed_y.coeffs, vec![2.0]);

    // Passthrough — h = 0, no seed piece, no kernel.
    assert!(state.axes[2].pieces.is_empty());
    assert_eq!(state.axes[2].h, 0.0);
    assert!(state.axes[2].kernel.is_none());

    // None — same as Passthrough for the seed/kernel; recorded for E.
    assert!(state.axes[3].pieces.is_empty());
    assert_eq!(state.axes[3].h, 0.0);
    assert!(state.axes[3].kernel.is_none());

    // Cursors start at zero.
    assert_eq!(state.t_appended, 0.0);
    assert_eq!(state.t_decel_start, 0.0);
    assert_eq!(state.t_shaped, 0.0);
    assert_eq!(state.t_dispatched, 0.0);
    assert!(state.pending_dispatch.is_empty());
}

#[test]
fn required_shaper_h_matches_axis_shaper_h() {
    // Sanity: the half-support computation matches `RequiredShaper::to_kernel`'s
    // own conversion (`0.8025 / freq` → support `[-h, h]`).
    let shapers: [Option<AxisShaper>; 4] = [
        Some(AxisShaper::SmoothZv {
            frequency_hz: 186.0,
        }),
        Some(AxisShaper::SmoothMzv {
            frequency_hz: 122.0,
        }),
        Some(AxisShaper::Passthrough),
        None,
    ];
    let state = ShaperState::new([0.0; 4], &shapers);

    let kernel_x = RequiredShaper::SmoothZv {
        frequency_hz: 186.0,
    }
    .to_kernel();
    let (lo_x, hi_x) = kernel_x.support();
    let expected_h_x = (hi_x - lo_x) / 2.0;
    assert!((state.axes[0].h - expected_h_x).abs() < 1e-15);

    let kernel_y = RequiredShaper::SmoothMzv {
        frequency_hz: 122.0,
    }
    .to_kernel();
    let (lo_y, hi_y) = kernel_y.support();
    let expected_h_y = (hi_y - lo_y) / 2.0;
    assert!((state.axes[1].h - expected_h_y).abs() < 1e-15);
}

// -----------------------------------------------------------------
// Phase 3 Task 3.1 — append_and_replan tests
// -----------------------------------------------------------------

/// Standard shaper set for the replan tests: SmoothMZV at 60 Hz on X
/// and Y, passthrough on Z, none on E. Matches the production MVP
/// `motion-bridge::config::PlannerConfig::default()` shape but with a
/// lower (more permissive) shaper frequency so short test moves can
/// converge β-medium under the relaxed tolerance budget.
fn replan_shapers() -> [Option<AxisShaper>; 4] {
    [
        Some(AxisShaper::SmoothMzv { frequency_hz: 60.0 }),
        Some(AxisShaper::SmoothMzv { frequency_hz: 60.0 }),
        Some(AxisShaper::Passthrough),
        None,
    ]
}

/// Mirrors `replan_shapers` but with the `plan_velocity::PlanShaper`
/// shape `ReplanContext` requires.
fn replan_kernels_planshaper() -> [Option<PlanShaper>; 4] {
    [
        Some(PlanShaper::SmoothMzv { frequency_hz: 60.0 }),
        Some(PlanShaper::SmoothMzv { frequency_hz: 60.0 }),
        Some(PlanShaper::Passthrough),
        None,
    ]
}

/// `emit_committed` consumes materialized `PiecewisePolynomialKernel`s.
/// Build them from the same shaper config as `replan_shapers`.
fn replan_kernels_piecewise() -> [Option<PiecewisePolynomialKernel<f64>>; 4] {
    [
        Some(
            RequiredShaper::SmoothMzv {
                frequency_hz: 60.0,
            }
            .to_kernel(),
        ),
        Some(
            RequiredShaper::SmoothMzv {
                frequency_hz: 60.0,
            }
            .to_kernel(),
        ),
        None,
        None,
    ]
}

fn replan_limits() -> temporal::Limits {
    temporal::Limits::new([500.0; 3], [5_000.0; 3], [100_000.0; 3], 2_500.0)
}

fn replan_context() -> ReplanContext {
    ReplanContext {
        limits: replan_limits(),
        kernels: replan_kernels_planshaper(),
        // Match the production `motion-bridge::config::PlannerConfig`
        // default (`0.005 mm`). The C¹ Hermite refit's L∞ tolerance gates
        // the cross-emission seam residue under the streaming flow: a
        // looser bound here lets two β-converged plans for the same path
        // tail differ by the full tolerance, which the post-shape kernel
        // then propagates into shaped output. 5 µm is the
        // `Phase 3 Task 3.1.5` test target and matches the production
        // pipeline.
        fit_tolerance_mm: 0.005,
        beta_max_iters: 5,
        beta_convergence_ratio: 1.02,
        e_limits: ELimits {
            v_max: 100.0,
            a_max: 5_000.0,
        },
        junction_chord_tolerance_mm: 0.05,
        worker_threads: 1,
        grid_strategy: temporal::multi::GridStrategy::Fixed(20),
        fallback_initial_v: 0.0,
        safety_mode: SafetyMode::WorstCaseFuture,
    }
}

/// Construct a pure-X `CubicSegment` from `(start_x, end_x)` at unit
/// feedrate. Inlines the collinear-cubic-Bézier formula
/// (control points at 0, 1/3, 2/3, 1 lerp) so the trajectory crate's
/// test harness doesn't have to depend on `motion-bridge` or `compat`.
fn linear_x_segment(start_x: f64, end_x: f64, feedrate: f64) -> CubicSegment {
    use geometry::segment::{EMode, SourceRange};
    use nurbs::VectorNurbs;

    let p0 = [start_x, 0.0, 0.0];
    let p3 = [end_x, 0.0, 0.0];
    let lerp = |t: f64| -> [f64; 3] {
        [
            p0[0] + (p3[0] - p0[0]) * t,
            p0[1] + (p3[1] - p0[1]) * t,
            p0[2] + (p3[2] - p0[2]) * t,
        ]
    };
    let cps = vec![p0, lerp(1.0 / 3.0), lerp(2.0 / 3.0), p3];
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        cps,
        None,
    )
    .unwrap();
    CubicSegment::try_new(
        xyz,
        EMode::Travel,
        0.0,
        None,
        feedrate,
        SourceRange {
            start_line: 0,
            end_line: 0,
        },
        None,
    )
    .unwrap()
}

/// Spec §3.4 single-move acceptance: after the first `append_and_replan`
/// call on a fresh state, the planner has built an accel-cruise-decel
/// profile (so `t_decel_start` is strictly between 0 and `t_appended`)
/// and the un-committed tail is materialized in the per-axis queues.
#[test]
fn single_move_append_planning_completes() {
    let mut state = ShaperState::new([0.0; 4], &replan_shapers());
    let ctx = replan_context();
    let seg = linear_x_segment(0.0, 1.0, 100.0);

    state
        .append_and_replan(seg, &ctx)
        .expect("first append should succeed");

    assert!(
        state.t_appended > 0.0,
        "t_appended must advance past 0.0 on first append, got {}",
        state.t_appended,
    );
    assert!(
        state.t_decel_start > 0.0,
        "t_decel_start must be strictly positive (the planner produced \
         a non-degenerate accel-cruise/peak-decel profile), got {}",
        state.t_decel_start,
    );
    assert!(
        state.t_decel_start < state.t_appended,
        "t_decel_start ({}) must lie strictly between 0 and t_appended ({}) — \
         the decel-to-zero ramp is the trailing portion of the plan",
        state.t_decel_start,
        state.t_appended,
    );
    // Per-axis queues are non-empty for X and Y.
    let x_pieces_after = state.axes[0]
        .pieces
        .iter()
        .filter(|p| p.u_start >= 0.0)
        .count();
    let y_pieces_after = state.axes[1]
        .pieces
        .iter()
        .filter(|p| p.u_start >= 0.0)
        .count();
    assert!(x_pieces_after > 0, "X queue must contain new plan's pieces");
    assert!(y_pieces_after > 0, "Y queue must contain new plan's pieces");
    // One UncommittedMove record per submitted move.
    assert_eq!(state.uncommitted_moves.len(), 1);
    assert!(state.uncommitted_moves[0].t_end > 0.0);
    // Plan cache is populated for emit_committed's use.
    assert_eq!(state.planned_fitted.len(), 1);
    assert_eq!(state.planned_meta.len(), 1);
}

/// Spec §3.4 chained-replan acceptance: after move 2 is appended,
/// the planner's velocity profile across the move-1/move-2 boundary
/// does **not** decelerate to zero — TOPP-RA picks a non-zero junction
/// velocity, allowing the toolhead to chain through.
#[test]
fn two_move_replan_chains_smoothly() {
    let mut state = ShaperState::new([0.0; 4], &replan_shapers());
    let ctx = replan_context();

    let m1 = linear_x_segment(0.0, 1.0, 100.0);
    state.append_and_replan(m1, &ctx).expect("move 1");
    let t_decel_after_move_1 = state.t_decel_start;
    let t_appended_after_move_1 = state.t_appended;

    let m2 = linear_x_segment(1.0, 2.0, 100.0);
    state.append_and_replan(m2, &ctx).expect("move 2");

    // After move 2, the move-1/move-2 boundary is in the interior of
    // the un-committed tail. The path-speed at the junction time
    // (where move-1's geometry ends and move-2's begins) must be
    // strictly positive — the planner is chaining, not stopping.
    assert_eq!(state.uncommitted_moves.len(), 2);
    let t_junction = state.uncommitted_moves[0].t_end;
    assert!(t_junction > 0.0 && t_junction < state.t_appended);

    let v_junction = state.read_path_speed_at(t_junction, -1.0);
    assert!(
        v_junction > 5.0,
        "junction speed must be strictly positive (chaining junction), got {} mm/s",
        v_junction,
    );

    // The chained plan covers move 1 + move 2 (2 mm total) and so
    // takes strictly longer than the move-1-only plan (1 mm). The
    // **start of the terminal decel ramp** (`t_decel_start`) sits
    // at the cruise-to-decel boundary; for a longer path with a
    // longer cruise plateau it can land further into the plan than
    // for a shorter path. What we can assert unconditionally is
    // that the decel ramp itself runs from `t_decel_start` all the
    // way to `t_appended` and occupies a non-empty trailing region.
    assert!(
        state.t_appended > t_appended_after_move_1,
        "two-move plan must take longer than one-move plan: \
         one-move {}, two-move {}",
        t_appended_after_move_1,
        state.t_appended,
    );
    assert!(
        state.t_decel_start < state.t_appended,
        "decel ramp must occupy a non-empty tail of the plan",
    );
    // Sanity reference; not asserted-on directly because the
    // move-1-only vs chained decel-start relationship depends on
    // TOPP-RA's peak shape under each input.
    let _ = t_decel_after_move_1;
}

/// Spec §3.4 history-preservation acceptance: when the dispatch cursor
/// has advanced past part of the planned trajectory, a follow-on
/// `append_and_replan` only replaces the un-committed portion of the
/// per-axis pieces. Pre-`t_dispatched` history is retained.
#[test]
fn append_after_committed_dispatch_keeps_history() {
    let mut state = ShaperState::new([0.0; 4], &replan_shapers());
    let ctx = replan_context();

    let m1 = linear_x_segment(0.0, 1.0, 100.0);
    state.append_and_replan(m1, &ctx).expect("move 1");

    // Simulate Phase-3 `emit_committed` advancing `t_dispatched` into
    // the middle of move 1 (between `0` and `t_decel_start`). For the
    // test we just write the cursor directly.
    let t_dispatched_synth = state.t_decel_start * 0.4;
    assert!(t_dispatched_synth > 0.0);
    state.t_dispatched = t_dispatched_synth;

    // Capture the X-axis piece set that's strictly behind the cursor
    // (history) so we can compare after the replan.
    let history_before: Vec<BezierPiece<f64>> = state.axes[0]
        .pieces
        .iter()
        .filter(|p| p.u_end <= t_dispatched_synth + 1e-12)
        .cloned()
        .collect();
    assert!(
        !history_before.is_empty(),
        "must have some history to preserve"
    );

    let m2 = linear_x_segment(1.0, 2.0, 100.0);
    state.append_and_replan(m2, &ctx).expect("move 2");

    let history_after: Vec<BezierPiece<f64>> = state.axes[0]
        .pieces
        .iter()
        .filter(|p| p.u_end <= t_dispatched_synth + 1e-12)
        .cloned()
        .collect();
    assert_eq!(
        history_before, history_after,
        "pre-t_dispatched X history must be preserved byte-identically across replan",
    );

    // And the queue must still extend past `t_appended` for the
    // un-committed (replanned) tail.
    let pieces_past_cursor = state.axes[0]
        .pieces
        .iter()
        .filter(|p| p.u_start >= t_dispatched_synth)
        .count();
    assert!(
        pieces_past_cursor > 0,
        "replan must have appended fresh pieces to the un-committed tail",
    );

    // The first move is still tracked as uncommitted (its old end-time
    // got rewritten by the new plan; the original t_end is no longer
    // a meaningful cursor). Both moves are present.
    assert_eq!(state.uncommitted_moves.len(), 2);
}

/// Spec §3.2 correctness: on a move with a real cruise plateau,
/// `t_decel_start` must land at the **start of the terminal decel
/// ramp**, not at the end-of-accel / start-of-cruise boundary.
///
/// Why this matters: the dispatch boundary is `t_decel_start - max_h`.
/// If `t_decel_start` landed at end-of-accel (the bug-symptom on
/// the prior `find_decel_start_time` implementation, which reported
/// the time of global path-speed maximum), almost the entire
/// cruise + decel region would be held back from dispatch on a
/// long-cruise jog. Throughput suffers measurably.
///
/// **Move:** 2000 mm pure-X. With the test context's `v_max =
/// 500 mm/s`, `a_max = 5_000 mm/s²` the unshaped profile cruises
/// at 500 mm/s for the bulk of the move; post-shape (smooth-MZV
/// @ 60 Hz, β-medium derated, C¹ refit at 0.5 mm tolerance) the
/// plateau remains a clearly dominant flat region (~3 s out of
/// ~4.3 s total). On a shorter move (~200 mm at the same limits)
/// the smooth-shaper / refit pipeline smears the accel and decel
/// ramps into the cruise and there is no recognizable plateau —
/// the test would degenerate into pinning a pure-bell shape and
/// neither the old nor the new `find_decel_start_time` would
/// have a meaningfully different answer there.
///
/// That gives us the cruise plateau the bug-symptom would have
/// hidden behind the held-back boundary.
#[test]
fn t_decel_start_lands_on_actual_decel_for_cruise_move() {
    let mut state = ShaperState::new([0.0; 4], &replan_shapers());
    // Bump the grid density so TOPP-RA resolves the long cruise
    // plateau cleanly under Fixed-grid sampling.
    let mut ctx = replan_context();
    ctx.grid_strategy = temporal::multi::GridStrategy::Fixed(50);
    let seg = linear_x_segment(0.0, 2000.0, 500.0);

    state
        .append_and_replan(seg, &ctx)
        .expect("long-cruise append should succeed");

    let t_appended = state.t_appended;
    let t_decel_start = state.t_decel_start;

    assert!(t_decel_start > 0.0 && t_decel_start < t_appended);

    // Sample path-speed on a dense uniform grid across the whole
    // move. 400 samples → ~1-2 ms resolution on a ~500 ms move.
    const N: usize = 400;
    let dt = t_appended / (N as f64);
    let samples: Vec<(f64, f64)> = (0..=N)
        .map(|i| {
            let t = (i as f64) * dt;
            (t, state.read_path_speed_at(t, 0.0))
        })
        .collect();

    // (1) Samples strictly after `t_decel_start` must be
    //     monotonically non-increasing — this is the decel ramp.
    //     Allow a small absolute tolerance (1e-3 mm/s) so the C¹
    //     refit's piece-boundary wobble doesn't falsely trip the
    //     check.
    let decel_samples: Vec<&(f64, f64)> = samples
        .iter()
        .filter(|(t, _)| *t > t_decel_start + 1e-9 && *t <= t_appended)
        .collect();
    assert!(
        decel_samples.len() >= 4,
        "must have at least 4 samples on the decel ramp; got {}",
        decel_samples.len(),
    );
    for w in decel_samples.windows(2) {
        let (t_a, v_a) = *w[0];
        let (t_b, v_b) = *w[1];
        assert!(
            v_a >= v_b - 1e-3,
            "decel ramp must be monotonically non-increasing: \
             v({}) = {} mm/s but v({}) = {} mm/s — that is forward-accel \
             inside the supposed decel region",
            t_a,
            v_a,
            t_b,
            v_b,
        );
    }

    // (2) There must exist a clear cruise plateau **before**
    //     `t_decel_start` — a contiguous stretch where path-speed
    //     varies by less than 1 mm/s across the window. We search
    //     for the longest such stretch within [0, t_decel_start]
    //     and assert it is non-trivial: at least 10% of the total
    //     move duration. (The bug-symptom on the prior
    //     implementation reported the time of global path-speed
    //     maximum, which on a long-cruise plateau is the **first**
    //     sample to hit v_peak — i.e., somewhere in or just past
    //     the accel ramp. With the bug, there would be at most a
    //     single-piece-wide "plateau" candidate; the new behaviour
    //     gives a multi-sample wide one.)
    let mut best_len_samples = 0usize;
    let mut best_window: Option<(f64, f64, f64)> = None; // (t_start, t_end, v_avg)
    let plateau_v_tol = 1.0f64; // mm/s — flat-enough threshold
    let mut i = 0usize;
    while i < samples.len() {
        // Only consider plateau windows that end at or before t_decel_start.
        if samples[i].0 > t_decel_start {
            break;
        }
        let v_i = samples[i].1;
        // Plateau samples must also be at the high-velocity regime,
        // so we don't accidentally find a "plateau" at the start
        // where v ≈ 0 and stays near 0 for a few samples.
        if v_i < 50.0 {
            i += 1;
            continue;
        }
        let mut j = i + 1;
        while j < samples.len()
            && samples[j].0 <= t_decel_start + 1e-9
            && (samples[j].1 - v_i).abs() < plateau_v_tol
        {
            j += 1;
        }
        let len = j - i;
        if len > best_len_samples {
            best_len_samples = len;
            let t_start = samples[i].0;
            let t_end = samples[j - 1].0;
            let v_avg: f64 = samples[i..j].iter().map(|(_, v)| *v).sum::<f64>() / (len as f64);
            best_window = Some((t_start, t_end, v_avg));
        }
        i = j.max(i + 1);
    }

    let (plateau_start, plateau_end, plateau_v) =
        best_window.expect("a cruise plateau must exist before t_decel_start");
    let plateau_duration = plateau_end - plateau_start;
    assert!(
        plateau_duration > t_appended * 0.10,
        "cruise plateau must span >10% of the move ({:.4} s); \
         found ({:.4}, {:.4}) = {:.4} s at v ≈ {:.2} mm/s — this is \
         the bug-symptom (t_decel_start landed at end-of-accel so the \
         cruise plateau got bundled into the decel side and shrank).",
        t_appended,
        plateau_start,
        plateau_end,
        plateau_duration,
        plateau_v,
    );

    // (3) The plateau must end at or just before `t_decel_start`,
    //     and the immediate post-plateau direction must be decel.
    //     This is the crisp positive form of "`t_decel_start` sits
    //     between cruise and decel."
    assert!(
        plateau_end <= t_decel_start + 1e-6,
        "plateau end {} must be at or before t_decel_start {}",
        plateau_end,
        t_decel_start,
    );
    assert!(
        plateau_end > t_decel_start - dt * 4.0,
        "plateau end {} must be within a few samples of t_decel_start {} \
         — the decel ramp starts right where cruise stops",
        plateau_end,
        t_decel_start,
    );
    // Plateau speed should be the high-regime value, not near zero.
    assert!(
        plateau_v > 50.0,
        "plateau speed must be at the high-regime cruise value, got {:.2} mm/s",
        plateau_v,
    );
}

// -----------------------------------------------------------------
// Phase 3 Task 3.2 — emit_committed tests
// -----------------------------------------------------------------

fn emit_context_default<'a>(
    kernels: &'a [Option<PiecewisePolynomialKernel<f64>>; 4],
    e_halos: &'a [EHalo],
) -> EmitContext<'a> {
    EmitContext { kernels, e_halos }
}

/// Fresh state, no appends — the boundary `t_decel_start − max_h` is
/// non-positive, so `emit_committed` returns empty and the cursors stay
/// at zero.
#[test]
fn emit_committed_returns_empty_when_target_not_advanced() {
    let mut state = ShaperState::new([0.0; 4], &replan_shapers());
    let kernels = replan_kernels_piecewise();
    let halos: Vec<EHalo> = Vec::new();
    let ctx = emit_context_default(&kernels, &halos);

    let out = state
        .emit_committed(&ctx)
        .expect("fresh-state emit_committed should not error");
    assert!(out.is_empty(), "fresh state must emit nothing");
    assert_eq!(state.t_dispatched, 0.0);
    assert_eq!(state.t_shaped, 0.0);
}

/// After a single `append_and_replan` of an accel-cruise-decel move,
/// `emit_committed` returns shaped segment(s) covering
/// `[0, t_decel_start − max_h]`, advances `t_dispatched` to that
/// boundary, and trims old per-axis history.
#[test]
fn emit_committed_after_single_append_dispatches_pre_decel_region() {
    let mut state = ShaperState::new([0.0; 4], &replan_shapers());
    let ctx_replan = replan_context();

    // A long enough move that the plan has a clear accel-cruise-decel
    // shape, so `t_decel_start` is well-interior to the segment.
    let seg = linear_x_segment(0.0, 200.0, 200.0);
    state.append_and_replan(seg, &ctx_replan).expect("append");

    let max_h: f64 = state.axes.iter().map(|a| a.h).fold(0.0_f64, f64::max);
    let target = state.t_decel_start - max_h;
    assert!(target > 0.0, "target must be positive for this test");

    let kernels = replan_kernels_piecewise();
    let halos: Vec<EHalo> = Vec::new();
    let ctx = emit_context_default(&kernels, &halos);
    let out = state
        .emit_committed(&ctx)
        .expect("single-append emit_committed should succeed");

    assert!(
        !out.is_empty(),
        "single-append emit must dispatch at least one shaped segment",
    );

    // Each returned segment must end at or before the dispatch target.
    for (i, s) in out.iter().enumerate() {
        assert!(
            s.t_end <= target + 1e-9,
            "seg[{i}]: t_end {} exceeds target {}",
            s.t_end,
            target,
        );
        assert!(
            s.t_start >= 0.0 - 1e-12,
            "seg[{i}]: t_start {} preceeds initial dispatch",
            s.t_start,
        );
    }
    // The last returned segment must reach right up to the target —
    // the convolution at `t == target` reads exactly through
    // `t_decel_start`, the rightmost committed point.
    let last = out.last().unwrap();
    assert!(
        (last.t_end - target).abs() < 1e-9,
        "last shaped segment must end at target {} (got {})",
        target,
        last.t_end,
    );

    // Cursors advanced to the boundary.
    assert!(
        (state.t_dispatched - target).abs() < 1e-12,
        "t_dispatched ({}) must advance to target ({})",
        state.t_dispatched,
        target,
    );
    assert!(
        (state.t_shaped - target).abs() < 1e-12,
        "t_shaped ({}) must advance to target ({})",
        state.t_shaped,
        target,
    );

    // History trim: any seed piece fully before `t_dispatched − max_h
    // − δ_safety` is gone. With δ_safety = max_h the cutoff is
    // `t_dispatched − 2·max_h`. The original seed spans
    // `[−2·max_h, 0]`. Since `t_dispatched − 2·max_h > 0 > 0 - 2·max_h`
    // (because `t_dispatched > 0` for any non-degenerate dispatch),
    // the seed's right edge `0` is strictly before the cutoff and so
    // the seed gets trimmed. Verify on X.
    assert!(
        !state.axes[0].pieces.iter().any(|p| p.u_end <= 0.0 + 1e-12),
        "the original seed piece must have been trimmed once dispatch advanced",
    );
}

/// Two-append chaining: emit_committed after append 1 dispatches part
/// of move 1; append 2; emit_committed again now extends dispatch
/// into the previously-held-back trailing region of move 1 (now
/// shapeable because move 2 supplies real future content). The two
/// emissions are continuous in position at the seam.
#[test]
fn emit_committed_chains_across_two_appends() {
    let mut state = ShaperState::new([0.0; 4], &replan_shapers());
    let ctx_replan = replan_context();

    // Move 1: 200 mm pure-X.
    let m1 = linear_x_segment(0.0, 200.0, 200.0);
    state
        .append_and_replan(m1, &ctx_replan)
        .expect("append move 1");
    let max_h: f64 = state.axes.iter().map(|a| a.h).fold(0.0_f64, f64::max);
    let target_1 = state.t_decel_start - max_h;
    assert!(target_1 > 0.0);

    let kernels = replan_kernels_piecewise();
    let halos: Vec<EHalo> = Vec::new();
    let ctx_emit = emit_context_default(&kernels, &halos);
    let out_1 = state
        .emit_committed(&ctx_emit)
        .expect("first emit_committed");
    assert!(!out_1.is_empty(), "first emit must produce output");
    let t_dispatched_after_1 = state.t_dispatched;
    assert!(
        (t_dispatched_after_1 - target_1).abs() < 1e-9,
        "first emit must advance t_dispatched to target_1",
    );

    // Record the position at the seam between move 1's emit and what
    // will follow it. Position is `axes[0]` (X) of the last segment
    // evaluated at the segment's `t_end`.
    let last_1 = out_1.last().unwrap();
    let x_at_seam_1 = nurbs::eval::eval(&last_1.axes[0], last_1.t_end);

    // Move 2: continues past 200 mm.
    let m2 = linear_x_segment(200.0, 400.0, 200.0);
    state
        .append_and_replan(m2, &ctx_replan)
        .expect("append move 2");

    // `t_decel_start` must have advanced — move 2 extends the plan,
    // and the new terminal decel is at the end of move 2.
    let target_2 = state.t_decel_start - max_h;
    assert!(
        target_2 > t_dispatched_after_1,
        "second emit target ({}) must advance past first dispatch ({})",
        target_2,
        t_dispatched_after_1,
    );

    let out_2 = state
        .emit_committed(&ctx_emit)
        .expect("second emit_committed");
    assert!(!out_2.is_empty(), "second emit must produce output");
    let first_2 = out_2.first().unwrap();
    let x_at_seam_2 = nurbs::eval::eval(&first_2.axes[0], first_2.t_start);

    // Cross-emission position continuity (spec §4.2): the X position
    // at the seam between emit-1 and emit-2 must match within refit
    // noise.
    //
    // **Phase 3 Task 3.1.5.** When `t_dispatched` lands interior to a
    // still-uncommitted move, `append_and_replan` rewrites that move's
    // `segment.xyz` to the right-half cubic (split at the parameter
    // matching the dispatched position) before feeding `plan_velocity`.
    // The new unshaped trajectory therefore starts at the *same X*
    // the prior unshaped plan placed there, and the post-shape kernel
    // sees no value-jump across the seam.
    //
    // The residual budget (50 µm) is dominated by the C¹ Hermite
    // refit's L∞ position error on each side of the seam (5 µm budget
    // per refit) plus the shaper convolution's response to that error.
    let seam_diff = (x_at_seam_1 - x_at_seam_2).abs();
    assert!(
        seam_diff < 0.05,
        "cross-emission X discontinuity at seam exceeds the refit \
         noise budget (50 µm): emit-1 ends at {} mm, emit-2 starts at \
         {} mm (diff {}). See Phase 3 Task 3.1.5.",
        x_at_seam_1,
        x_at_seam_2,
        seam_diff,
    );

    // And the second emit's segments are all in the new eligible
    // range.
    for (i, s) in out_2.iter().enumerate() {
        assert!(
            s.t_end <= target_2 + 1e-9,
            "out_2 seg[{i}]: t_end {} exceeds target_2 {}",
            s.t_end,
            target_2,
        );
        assert!(
            s.t_start >= t_dispatched_after_1 - 1e-9,
            "out_2 seg[{i}]: t_start {} precedes first dispatch {}",
            s.t_start,
            t_dispatched_after_1,
        );
    }
}

/// Phase 3 Task 3.1.5 — when `t_dispatched` lands interior to a
/// partially-committed move, the new plan produced by `append_and_replan`
/// must place the toolhead at the same unshaped X position the old plan
/// had there. Concretely: read X(t_dispatched) off the old plan's
/// per-axis cache, run `append_and_replan` for a follow-on move, and
/// confirm the new plan's per-axis cache still produces the same X at
/// `t_dispatched`.
///
/// This is the direct invariant the split-at-`s_dispatched` fix
/// preserves; without the fix the new plan starts at `X = 0` at
/// `t_dispatched`, blowing the assertion by ~94 mm on this 200 mm
/// cruise.
#[test]
fn t_dispatched_interior_to_move_replan_preserves_position() {
    let mut state = ShaperState::new([0.0; 4], &replan_shapers());
    let ctx_replan = replan_context();
    let kernels = replan_kernels_piecewise();
    let halos: Vec<EHalo> = Vec::new();
    let ctx_emit = emit_context_default(&kernels, &halos);

    // Move 1: 200 mm pure-X. Plan it, then partially dispatch so
    // `t_dispatched` lands interior to the move (somewhere in the
    // cruise plateau on the way to 200 mm).
    let m1 = linear_x_segment(0.0, 200.0, 200.0);
    state
        .append_and_replan(m1, &ctx_replan)
        .expect("append move 1");
    let _ = state
        .emit_committed(&ctx_emit)
        .expect("emit move 1 (partial)");
    let t_d = state.t_dispatched;
    assert!(t_d > 0.0, "emit must have advanced t_dispatched");

    // Read X(t_dispatched) off the prior (unshaped) plan's per-axis
    // queue. By the queue's invariant this is the same value the
    // post-shape kernel was about to convolve through at the seam.
    let x_pre_replan = read_axis_value_at(&state, 0, t_d)
        .expect("axes[0] must cover t_dispatched after emit");
    // For a 200 mm move with `feedrate = 200 mm/s`, the unshaped profile
    // cruises near 200 mm/s for most of the move; t_dispatched (at
    // `t_decel_start − max_h`) lands somewhere in or past the cruise
    // plateau. Pre-feedrate-cap (when v_max wasn't bound by F) the
    // dispatch boundary landed near X ≈ 94 mm; post-cap the longer
    // cruise pushes it further (~186 mm). Either way, we just want a
    // "well-interior to the move" sanity check that catches a
    // regression dropping dispatch back to the move's origin.
    assert!(
        x_pre_replan > 50.0 && x_pre_replan < 199.0,
        "t_dispatched should land interior to move 1's plan; \
         got X(t_d) = {x_pre_replan} mm",
    );

    // Append move 2. The split-at-`s_dispatched` logic should rewrite
    // move 1's `xyz` to the right-half cubic *before* feeding
    // `plan_velocity`. After the call, the new plan's queue at the
    // same absolute time `t_d` must match `x_pre_replan` within
    // refit noise.
    let m2 = linear_x_segment(200.0, 400.0, 200.0);
    state
        .append_and_replan(m2, &ctx_replan)
        .expect("append move 2");
    let x_post_replan = read_axis_value_at(&state, 0, t_d)
        .expect("axes[0] must still cover t_dispatched after replan");

    // Refit-noise budget: the C¹ Hermite fit's L∞ tolerance is
    // `REFIT_TOLERANCE_MM` (5 µm); we allow 50 µm here to absorb the
    // composite of fit error on both sides of the seam plus the
    // arc-length / parameter-inversion residual.
    let diff = (x_post_replan - x_pre_replan).abs();
    assert!(
        diff < 0.05,
        "post-replan X({t_d}) = {x_post_replan} mm deviates from \
         pre-replan X = {x_pre_replan} mm by {diff} mm (50 µm budget). \
         If this regresses, `split_partially_committed_at_t_dispatched` \
         is either not running or computing the wrong split parameter.",
    );
}

/// Read the value of axis `axis_idx`'s queued piece active at absolute
/// time `t`. Tie-break: prefer the right-hand piece on a boundary tie
/// (`u_start ≤ t < u_end`); on the very last piece's terminus,
/// extrapolate by clamping `t` to `u_end`.
fn read_axis_value_at(state: &ShaperState, axis_idx: usize, t: f64) -> Option<f64> {
    let pieces = &state.axes[axis_idx].pieces;
    if pieces.is_empty() {
        return None;
    }
    let last = pieces.back().unwrap();
    if t >= last.u_end && t <= last.u_end + 1e-12 {
        return Some(last.evaluate(last.u_end));
    }
    for p in pieces {
        if p.u_start - 1e-12 <= t && t < p.u_end {
            return Some(p.evaluate(t));
        }
    }
    None
}

/// **Phase 3 Task 3.1.5 cleanup — Critical 1 regression.** After a
/// failed `append_and_replan` call (i.e., `plan_velocity` returned `Err`),
/// the state must be byte-equivalent to its pre-call value so the next
/// `append_and_replan` sees consistent `planned_fitted` /
/// `uncommitted_moves` alignment. The original implementation snapshotted
/// `uncommitted_moves` / `t_appended` / `t_decel_start` but *not*
/// `planned_fitted` / `planned_meta`; this test guards against a regression
/// to that incomplete rollback.
///
/// **Setup.** Append move 1; `emit_committed` to advance `t_dispatched`
/// interior to move 1; then call `append_and_replan` with a deliberately
/// broken `ReplanContext` (Passthrough X kernel → `UnsupportedShaperOnXY`).
/// After the failure, every snapshot-able field must equal its pre-failure
/// value, including `planned_fitted` and `planned_meta`. Finally, append
/// move 2 with the *good* context and verify it succeeds and produces the
/// same outcome as a fresh `append_and_replan` would have (no leftover
/// corruption from the failed attempt).
#[test]
#[allow(clippy::float_cmp)] // Byte-equivalence rollback check requires exact comparison.
fn append_and_replan_rolls_back_planned_caches_on_plan_velocity_error() {
    let mut state = ShaperState::new([0.0; 4], &replan_shapers());
    let ctx_good = replan_context();
    let kernels = replan_kernels_piecewise();
    let halos: Vec<EHalo> = Vec::new();
    let ctx_emit = emit_context_default(&kernels, &halos);

    // 1. Append move 1 and partially dispatch so we have a populated
    //    `planned_fitted` cache + a `t_dispatched` interior to move 1.
    let m1 = linear_x_segment(0.0, 200.0, 200.0);
    state
        .append_and_replan(m1, &ctx_good)
        .expect("append move 1 (good context)");
    let _ = state
        .emit_committed(&ctx_emit)
        .expect("emit move 1 (partial)");
    assert!(state.t_dispatched > 0.0);

    // 2. Snapshot every public field the rollback contract covers, plus
    //    the private `planned_fitted` / `planned_meta` caches.
    let snap_uncommitted = state.uncommitted_moves.clone();
    let snap_t_appended = state.t_appended;
    let snap_t_decel_start = state.t_decel_start;
    let snap_planned_fitted_len = state.planned_fitted.len();
    let snap_planned_meta_len = state.planned_meta.len();
    let snap_planned_fitted_bounds: Vec<(f64, f64)> = state
        .planned_fitted
        .iter()
        .map(|f| (f.t_start, f.t_end))
        .collect();
    let snap_planned_meta_extrusion: Vec<f64> = state
        .planned_meta
        .iter()
        .map(|m| m.extrusion_per_xy_mm)
        .collect();

    // 3. Build a *broken* context whose only difference is `Passthrough`
    //    on X — `plan_velocity` rejects this with `UnsupportedShaperOnXY`
    //    before any state mutation in TOPP-RA. This is the cleanest
    //    reachable error for the rollback test: we want a deterministic
    //    failure that exercises the `Err` arm of the `match plan_velocity`
    //    in `append_and_replan` without depending on TOPP-RA's
    //    convergence-failure conditions.
    let mut ctx_bad = ctx_good;
    ctx_bad.kernels[0] = Some(PlanShaper::Passthrough);

    let m_broken = linear_x_segment(200.0, 400.0, 200.0);
    let bad_result = state.append_and_replan(m_broken, &ctx_bad);
    assert!(
        bad_result.is_err(),
        "append with Passthrough-X context must fail",
    );

    // 4. Verify byte-equivalence of every snapshot-able field.
    assert_eq!(
        state.uncommitted_moves.len(),
        snap_uncommitted.len(),
        "uncommitted_moves length changed across failed append",
    );
    for (i, (a, b)) in state
        .uncommitted_moves
        .iter()
        .zip(snap_uncommitted.iter())
        .enumerate()
    {
        assert_eq!(
            a.t_start, b.t_start,
            "uncommitted_moves[{i}].t_start changed across failed append",
        );
        assert_eq!(
            a.t_end, b.t_end,
            "uncommitted_moves[{i}].t_end changed across failed append",
        );
    }
    assert_eq!(
        state.t_appended, snap_t_appended,
        "t_appended changed across failed append",
    );
    assert_eq!(
        state.t_decel_start, snap_t_decel_start,
        "t_decel_start changed across failed append",
    );
    assert_eq!(
        state.planned_fitted.len(),
        snap_planned_fitted_len,
        "planned_fitted length changed across failed append \
         (the headline regression Critical 1 was about)",
    );
    assert_eq!(
        state.planned_meta.len(),
        snap_planned_meta_len,
        "planned_meta length changed across failed append",
    );
    for (i, (a, b)) in state
        .planned_fitted
        .iter()
        .zip(snap_planned_fitted_bounds.iter())
        .enumerate()
    {
        assert_eq!(
            a.t_start, b.0,
            "planned_fitted[{i}].t_start changed across failed append",
        );
        assert_eq!(
            a.t_end, b.1,
            "planned_fitted[{i}].t_end changed across failed append",
        );
    }
    for (i, (a, b)) in state
        .planned_meta
        .iter()
        .zip(snap_planned_meta_extrusion.iter())
        .enumerate()
    {
        assert_eq!(
            a.extrusion_per_xy_mm, *b,
            "planned_meta[{i}].extrusion_per_xy_mm changed across failed append",
        );
    }

    // 5. A follow-on good append must still work — and the
    //    `split_partially_committed_at_t_dispatched` resolver must find
    //    a consistent (`planned_fitted`, `uncommitted_moves`) pair. We
    //    verify behaviour-equivalence by reading X at `t_dispatched`
    //    before and after, mirroring
    //    `t_dispatched_interior_to_move_replan_preserves_position`'s
    //    assertion.
    let t_d = state.t_dispatched;
    let x_pre = read_axis_value_at(&state, 0, t_d)
        .expect("axes[0] must cover t_dispatched after the failed call");

    let m2 = linear_x_segment(200.0, 400.0, 200.0);
    state
        .append_and_replan(m2, &ctx_good)
        .expect("good append after failed append must succeed");
    let x_post = read_axis_value_at(&state, 0, t_d)
        .expect("axes[0] must still cover t_dispatched after replan");
    let diff = (x_post - x_pre).abs();
    assert!(
        diff < 0.05,
        "post-rollback replan must preserve X(t_dispatched) within \
         refit budget (50 µm): pre = {x_pre} mm, post = {x_post} mm, \
         diff = {diff} mm. If this regresses, the failed-append rollback \
         left `planned_fitted` out of sync with `uncommitted_moves`, and \
         the partial-commit split picked the wrong target.",
    );
}

/// History trim: pieces whose right edge is strictly before
/// `t_dispatched − max_h − δ_safety` get dropped. After several
/// append/emit rounds, no axis-queue piece should violate that
/// invariant.
#[test]
fn emit_committed_trims_old_history() {
    let mut state = ShaperState::new([0.0; 4], &replan_shapers());
    let ctx_replan = replan_context();
    let kernels = replan_kernels_piecewise();
    let halos: Vec<EHalo> = Vec::new();
    let ctx_emit = emit_context_default(&kernels, &halos);

    // Two append-and-emit rounds, so the second round's emit has
    // the first round's pieces sitting fully behind it.
    let m1 = linear_x_segment(0.0, 200.0, 200.0);
    state.append_and_replan(m1, &ctx_replan).expect("append 1");
    let _ = state.emit_committed(&ctx_emit).expect("emit 1");

    let m2 = linear_x_segment(200.0, 400.0, 200.0);
    state.append_and_replan(m2, &ctx_replan).expect("append 2");
    let _ = state.emit_committed(&ctx_emit).expect("emit 2");

    // After two rounds, `t_dispatched > 0` and per-axis pieces have
    // been trimmed.
    let max_h: f64 = state.axes.iter().map(|a| a.h).fold(0.0_f64, f64::max);
    let trim_cutoff = state.t_dispatched - max_h - max_h;
    for (axis_idx, axis) in state.axes.iter().enumerate() {
        for p in &axis.pieces {
            assert!(
                p.u_end >= trim_cutoff - 1e-12,
                "axis {axis_idx}: piece with u_end {} survived after trim cutoff {}",
                p.u_end,
                trim_cutoff,
            );
        }
    }
}

// -----------------------------------------------------------------
// Phase 5 Task 5.1 — ShaperState::reset
// -----------------------------------------------------------------

/// Spec §3.7 reset acceptance: after submitting a move, advancing
/// cursors via the normal append/emit cycle, and then calling
/// `reset(new_home)`, every observable field of `ShaperState` must
/// match what a freshly-constructed `ShaperState::new(new_home,
/// &same_shapers)` would produce — except for `kernel` / `h` which are
/// deliberately preserved (reset is a position re-anchor, not a shaper
/// reconfiguration; see [`ShaperState::reset`]'s doc-comment).
#[test]
fn reset_after_motion_clears_state_and_reseeds_at_home() {
    let shapers = replan_shapers();
    let mut state = ShaperState::new([0.0; 4], &shapers);
    let ctx_replan = replan_context();
    let kernels = replan_kernels_piecewise();
    let halos: Vec<EHalo> = Vec::new();
    let ctx_emit = emit_context_default(&kernels, &halos);

    // Submit a move and advance cursors via append + emit so the state
    // is meaningfully non-fresh before the reset.
    let m1 = linear_x_segment(0.0, 200.0, 200.0);
    state.append_and_replan(m1, &ctx_replan).expect("append 1");
    let _ = state.emit_committed(&ctx_emit).expect("emit 1");
    assert!(state.t_appended > 0.0, "precondition: t_appended advanced");
    assert!(
        state.t_dispatched > 0.0,
        "precondition: t_dispatched advanced",
    );
    assert!(
        !state.uncommitted_moves.is_empty(),
        "precondition: uncommitted_moves non-empty",
    );
    assert!(
        !state.planned_fitted.is_empty(),
        "precondition: planned_fitted populated",
    );

    // Reset to a new home position.
    let new_home = [10.0, 20.0, 30.0, 0.0];
    state.reset(new_home);

    // Build the reference fresh state with the **same** shaper config.
    let fresh = ShaperState::new(new_home, &shapers);

    // Cursors and queues must equal the fresh state.
    assert_eq!(state.t_appended, fresh.t_appended);
    assert_eq!(state.t_decel_start, fresh.t_decel_start);
    assert_eq!(state.t_shaped, fresh.t_shaped);
    assert_eq!(state.t_dispatched, fresh.t_dispatched);
    assert!(state.uncommitted_moves.is_empty());
    assert!(state.pending_dispatch.is_empty());
    assert!(state.planned_fitted.is_empty());
    assert!(state.planned_meta.is_empty());

    // Per-axis: the queue's seed piece must match the fresh state's
    // seed piece byte-for-byte (same `home_pos`, same `h`-derived
    // span, same constant-polynomial coeffs). `kernel` and `h` are
    // preserved across reset by construction; we cross-check that
    // they didn't drift.
    for axis_idx in 0..4 {
        let s = &state.axes[axis_idx];
        let f = &fresh.axes[axis_idx];
        assert_eq!(
            s.pieces.len(),
            f.pieces.len(),
            "axis {axis_idx}: piece count mismatch after reset",
        );
        for (sp, fp) in s.pieces.iter().zip(f.pieces.iter()) {
            assert_eq!(sp.u_start, fp.u_start, "axis {axis_idx} u_start");
            assert_eq!(sp.u_end, fp.u_end, "axis {axis_idx} u_end");
            assert_eq!(sp.coeffs, fp.coeffs, "axis {axis_idx} coeffs");
        }
        assert_eq!(s.h, f.h, "axis {axis_idx}: h preserved across reset");
        assert_eq!(
            s.kernel.is_some(),
            f.kernel.is_some(),
            "axis {axis_idx}: kernel presence preserved across reset",
        );
    }
}

/// Regression test for the path-frame jerk min() bug fixed in
/// `per_segment_limits`. Pre-fix, the SOCP used
/// `min(j_max[X], j_max[Y], j_max[Z])` for the path-frame jerk bound;
/// with `j_max[Z]=200` (from `max_z_accel=100*2`), a pure-X jog ran
/// ~700× slower than the X-axis is capable of. Pre-fix: 50mm @ 100mm/s
/// took 1.45s. Fix: inactive axes' `j_max` bumped to max across active
/// axes so the SOCP's `min()` reduces to active-axis only.
#[test]
fn live_limits_50mm_pure_x_completes_quickly() {
    let mut state = ShaperState::new([0.0; 4], &replan_shapers());
    let live = temporal::Limits::new(
        [1000.0, 1000.0, 5.0],
        [70000.0, 70000.0, 100.0],
        [140000.0, 140000.0, 200.0],
        5.0_f64.powi(2) / (70000.0 * 0.5),
    );
    let mut ctx = replan_context();
    ctx.limits = live;

    let seg = linear_x_segment(0.0, 50.0, 100.0);
    state
        .append_and_replan(seg, &ctx)
        .expect("50mm pure-X jog should plan");

    eprintln!(
        "[regression] live-limits 50mm pure-X: t_appended={:.6}s t_decel={:.6}s",
        state.t_appended, state.t_decel_start,
    );
    assert!(
        state.t_appended < 0.8,
        "50mm pure-X jog took {:.4}s — pre-fix was 1.447s (j_max[Z]=200 bound)",
        state.t_appended,
    );
}

