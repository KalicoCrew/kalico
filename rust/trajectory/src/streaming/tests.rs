#![allow(
    clippy::doc_markdown,
    clippy::uninlined_format_args,
    clippy::too_many_lines,
    clippy::items_after_statements,
    clippy::float_cmp,
    clippy::unreadable_literal
)]

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
use crate::{AxisShaper, ELimits};

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

fn assert_nurbs_near_equal(a: &ScalarNurbs<f64>, b: &ScalarNurbs<f64>, label: &str) {
    assert_eq!(a.degree(), b.degree(), "{label}: degree differs");
    assert_eq!(
        a.knots().len(),
        b.knots().len(),
        "{label}: knot count differs"
    );
    let max_knot_diff = a
        .knots()
        .iter()
        .zip(b.knots().iter())
        .map(|(ka, kb)| (ka - kb).abs())
        .fold(0.0_f64, f64::max);
    assert!(
        max_knot_diff < 1e-12,
        "{label}: knots differ by {max_knot_diff:.2e}",
    );
    assert_eq!(
        a.control_points().len(),
        b.control_points().len(),
        "{label}: control point count differs",
    );
    let max_cp_diff = a
        .control_points()
        .iter()
        .zip(b.control_points().iter())
        .map(|(ca, cb)| (ca - cb).abs())
        .fold(0.0_f64, f64::max);
    assert!(
        max_cp_diff < 1e-12,
        "{label}: control points differ by {max_cp_diff:.2e} mm",
    );
}

#[test]
#[allow(clippy::float_cmp)] // Time bounds and cursor zeros are exact-by-construction.
fn shim_matches_direct_pipeline_for_single_linear_move() {
    let fitted = linear_segment();
    let freq = 60.0;
    let h = 0.8025 / freq / 2.0;
    let kernel = build_smooth_zv_kernel(0.8025 / freq);

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

    assert!(state.pending_dispatch.is_empty());
    assert!(state.drain_committed().is_empty());

    let fitted_slice = std::slice::from_ref(&fitted);

    let x_padded = pad_segment_axis(0, 0, fitted_slice, &[], h, 0.0, 1.0);
    let x_shaped = shape_axis(&x_padded, &kernel, 0.0, 1.0);
    let x_refit = refit_to_cubic(&x_shaped, REFIT_TOLERANCE_MM).unwrap();

    let y_padded = pad_segment_axis(0, 1, fitted_slice, &[], h, 0.0, 1.0);
    let y_shaped = shape_axis(&y_padded, &kernel, 0.0, 1.0);
    let y_refit = refit_to_cubic(&y_shaped, REFIT_TOLERANCE_MM).unwrap();

    let z_passthrough = fitted.axes[2].clone();
    let z_refit = refit_to_cubic(&z_passthrough, REFIT_TOLERANCE_MM).unwrap();

    assert_nurbs_near_equal(&shim_seg.axes[0], &x_refit, "X");
    assert_nurbs_near_equal(&shim_seg.axes[1], &y_refit, "Y");
    assert_nurbs_near_equal(&shim_seg.axes[2], &z_refit, "Z");

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
        Some(AxisShaper::SmoothMzv { frequency_hz: 80.0 }),
        Some(AxisShaper::Passthrough),
        None,
    ];
    let state = ShaperState::new([1.0, 2.0, 3.0, 4.0], &shapers);

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

    assert!(state.axes[2].pieces.is_empty());
    assert_eq!(state.axes[2].h, 0.0);
    assert!(state.axes[2].kernel.is_none());

    assert!(state.axes[3].pieces.is_empty());
    assert_eq!(state.axes[3].h, 0.0);
    assert!(state.axes[3].kernel.is_none());

    assert_eq!(state.t_appended, 0.0);
    assert_eq!(state.t_decel_start, 0.0);
    assert_eq!(state.t_shaped, 0.0);
    assert_eq!(state.t_dispatched, 0.0);
    assert!(state.pending_dispatch.is_empty());
}

#[test]
fn required_shaper_h_matches_axis_shaper_h() {
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

    let kernel_x = AxisShaper::SmoothZv {
        frequency_hz: 186.0,
    }
    .to_kernel()
    .unwrap();
    let (lo_x, hi_x) = kernel_x.support();
    let expected_h_x = (hi_x - lo_x) / 2.0;
    assert!((state.axes[0].h - expected_h_x).abs() < 1e-15);

    let kernel_y = AxisShaper::SmoothMzv {
        frequency_hz: 122.0,
    }
    .to_kernel()
    .unwrap();
    let (lo_y, hi_y) = kernel_y.support();
    let expected_h_y = (hi_y - lo_y) / 2.0;
    assert!((state.axes[1].h - expected_h_y).abs() < 1e-15);
}

fn replan_shapers() -> [Option<AxisShaper>; 4] {
    [
        Some(AxisShaper::SmoothMzv { frequency_hz: 60.0 }),
        Some(AxisShaper::SmoothMzv { frequency_hz: 60.0 }),
        Some(AxisShaper::Passthrough),
        None,
    ]
}

fn replan_kernels_planshaper() -> [Option<PlanShaper>; 4] {
    [
        Some(PlanShaper::SmoothMzv { frequency_hz: 60.0 }),
        Some(PlanShaper::SmoothMzv { frequency_hz: 60.0 }),
        Some(PlanShaper::Passthrough),
        None,
    ]
}

fn replan_kernels_piecewise() -> [Option<PiecewisePolynomialKernel<f64>>; 4] {
    [
        AxisShaper::SmoothMzv { frequency_hz: 60.0 }.to_kernel(),
        AxisShaper::SmoothMzv { frequency_hz: 60.0 }.to_kernel(),
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
    let xyz = VectorNurbs::<f64, 3>::try_new(3, vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0], cps)
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
    assert_eq!(state.uncommitted_moves.len(), 1);
    assert!(state.uncommitted_moves[0].t_end > 0.0);
    assert_eq!(state.planned_fitted.len(), 1);
    assert_eq!(state.planned_meta.len(), 1);
}

#[test]
fn two_move_replan_chains_smoothly() {
    let mut state = ShaperState::new([0.0; 4], &replan_shapers());
    let ctx = replan_context();

    let m1 = linear_x_segment(0.0, 1.0, 100.0);
    state.append_and_replan(m1, &ctx).expect("move 1");

    let m2 = linear_x_segment(1.0, 2.0, 100.0);
    state.append_and_replan(m2, &ctx).expect("move 2");

    assert_eq!(state.uncommitted_moves.len(), 2);
    let t_junction = state.uncommitted_moves[0].t_end;
    assert!(t_junction > 0.0 && t_junction < state.t_appended);

    let v_junction = state.read_path_speed_at(t_junction, -1.0);
    assert!(
        v_junction > 5.0,
        "junction speed must be strictly positive (chaining junction), got {} mm/s",
        v_junction,
    );

    assert!(
        state.t_appended > 0.0,
        "two-move plan must have positive duration",
    );
    assert!(
        state.t_decel_start < state.t_appended,
        "decel ramp must occupy a non-empty tail of the plan",
    );
}

#[test]
fn append_after_committed_dispatch_keeps_history() {
    let mut state = ShaperState::new([0.0; 4], &replan_shapers());
    let ctx = replan_context();

    let m1 = linear_x_segment(0.0, 1.0, 100.0);
    state.append_and_replan(m1, &ctx).expect("move 1");

    let t_dispatched_synth = state.t_decel_start * 0.4;
    assert!(t_dispatched_synth > 0.0);
    state.t_dispatched = t_dispatched_synth;

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

    let pieces_past_cursor = state.axes[0]
        .pieces
        .iter()
        .filter(|p| p.u_start >= t_dispatched_synth)
        .count();
    assert!(
        pieces_past_cursor > 0,
        "replan must have appended fresh pieces to the un-committed tail",
    );

    assert_eq!(state.uncommitted_moves.len(), 2);
}

#[test]
fn t_decel_start_lands_on_actual_decel_for_cruise_move() {
    let mut state = ShaperState::new([0.0; 4], &replan_shapers());
    let mut ctx = replan_context();
    ctx.grid_strategy = temporal::multi::GridStrategy::Fixed(50);
    let seg = linear_x_segment(0.0, 2000.0, 500.0);

    state
        .append_and_replan(seg, &ctx)
        .expect("long-cruise append should succeed");

    let t_appended = state.t_appended;
    let t_decel_start = state.t_decel_start;

    assert!(t_decel_start > 0.0 && t_decel_start < t_appended);

    const N: usize = 400;
    let dt = t_appended / (N as f64);
    let samples: Vec<(f64, f64)> = (0..=N)
        .map(|i| {
            let t = (i as f64) * dt;
            (t, state.read_path_speed_at(t, 0.0))
        })
        .collect();

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

    let mut best_len_samples = 0usize;
    let mut best_window: Option<(f64, f64, f64)> = None;
    let plateau_v_tol = 1.0f64;
    let mut i = 0usize;
    while i < samples.len() {
        if samples[i].0 > t_decel_start {
            break;
        }
        let v_i = samples[i].1;
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
    assert!(
        plateau_v > 50.0,
        "plateau speed must be at the high-regime cruise value, got {:.2} mm/s",
        plateau_v,
    );
}

fn emit_context_default<'a>(
    kernels: &'a [Option<PiecewisePolynomialKernel<f64>>; 4],
    e_halos: &'a [EHalo],
) -> EmitContext<'a> {
    EmitContext { kernels, e_halos }
}

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

#[test]
fn emit_committed_after_single_append_dispatches_pre_decel_region() {
    let mut state = ShaperState::new([0.0; 4], &replan_shapers());
    let ctx_replan = replan_context();
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
    let last = out.last().unwrap();
    assert!(
        (last.t_end - target).abs() < 1e-9,
        "last shaped segment must end at target {} (got {})",
        target,
        last.t_end,
    );

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

    assert!(
        !state.axes[0].pieces.iter().any(|p| p.u_end <= 0.0 + 1e-12),
        "the original seed piece must have been trimmed once dispatch advanced",
    );
}

#[test]
fn emit_committed_chains_across_two_appends() {
    let mut state = ShaperState::new([0.0; 4], &replan_shapers());
    let ctx_replan = replan_context();

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

    let last_1 = out_1.last().unwrap();
    let x_at_seam_1 = nurbs::eval::eval(&last_1.axes[0], last_1.t_end);

    let m2 = linear_x_segment(200.0, 400.0, 200.0);
    state
        .append_and_replan(m2, &ctx_replan)
        .expect("append move 2");

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

#[test]
fn t_dispatched_interior_to_move_replan_preserves_position() {
    let mut state = ShaperState::new([0.0; 4], &replan_shapers());
    let ctx_replan = replan_context();
    let kernels = replan_kernels_piecewise();
    let halos: Vec<EHalo> = Vec::new();
    let ctx_emit = emit_context_default(&kernels, &halos);

    let m1 = linear_x_segment(0.0, 200.0, 200.0);
    state
        .append_and_replan(m1, &ctx_replan)
        .expect("append move 1");
    let _ = state
        .emit_committed(&ctx_emit)
        .expect("emit move 1 (partial)");
    let t_d = state.t_dispatched;
    assert!(t_d > 0.0, "emit must have advanced t_dispatched");

    let x_pre_replan =
        read_axis_value_at(&state, 0, t_d).expect("axes[0] must cover t_dispatched after emit");
    assert!(
        x_pre_replan > 50.0 && x_pre_replan < 199.0,
        "t_dispatched should land interior to move 1's plan; \
         got X(t_d) = {x_pre_replan} mm",
    );

    let m2 = linear_x_segment(200.0, 400.0, 200.0);
    state
        .append_and_replan(m2, &ctx_replan)
        .expect("append move 2");
    let x_post_replan = read_axis_value_at(&state, 0, t_d)
        .expect("axes[0] must still cover t_dispatched after replan");

    let diff = (x_post_replan - x_pre_replan).abs();
    assert!(
        diff < 0.05,
        "post-replan X({t_d}) = {x_post_replan} mm deviates from \
         pre-replan X = {x_pre_replan} mm by {diff} mm (50 µm budget). \
         If this regresses, `split_partially_committed_at_t_dispatched` \
         is either not running or computing the wrong split parameter.",
    );
}

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

#[test]
#[allow(clippy::float_cmp)] // Byte-equivalence rollback check requires exact comparison.
fn append_and_replan_rolls_back_planned_caches_on_plan_velocity_error() {
    let mut state = ShaperState::new([0.0; 4], &replan_shapers());
    let ctx_good = replan_context();
    let kernels = replan_kernels_piecewise();
    let halos: Vec<EHalo> = Vec::new();
    let ctx_emit = emit_context_default(&kernels, &halos);

    let m1 = linear_x_segment(0.0, 200.0, 200.0);
    state
        .append_and_replan(m1, &ctx_good)
        .expect("append move 1 (good context)");
    let _ = state
        .emit_committed(&ctx_emit)
        .expect("emit move 1 (partial)");
    assert!(state.t_dispatched > 0.0);

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

    let mut ctx_bad = ctx_good;
    ctx_bad.limits = temporal::Limits::new([1e-10; 3], [5_000.0; 3], [100_000.0; 3], 2_500.0);

    let m_broken = linear_x_segment(200.0, 400.0, 200.0);
    let bad_result = state.append_and_replan(m_broken, &ctx_bad);
    assert!(
        bad_result.is_err(),
        "append with Passthrough-X context must fail",
    );

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

#[test]
fn emit_committed_trims_old_history() {
    let mut state = ShaperState::new([0.0; 4], &replan_shapers());
    let ctx_replan = replan_context();
    let kernels = replan_kernels_piecewise();
    let halos: Vec<EHalo> = Vec::new();
    let ctx_emit = emit_context_default(&kernels, &halos);

    let m1 = linear_x_segment(0.0, 200.0, 200.0);
    state.append_and_replan(m1, &ctx_replan).expect("append 1");
    let _ = state.emit_committed(&ctx_emit).expect("emit 1");

    let m2 = linear_x_segment(200.0, 400.0, 200.0);
    state.append_and_replan(m2, &ctx_replan).expect("append 2");
    let _ = state.emit_committed(&ctx_emit).expect("emit 2");

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

#[test]
fn reset_after_motion_clears_state_and_reseeds_at_home() {
    let shapers = replan_shapers();
    let mut state = ShaperState::new([0.0; 4], &shapers);
    let ctx_replan = replan_context();
    let kernels = replan_kernels_piecewise();
    let halos: Vec<EHalo> = Vec::new();
    let ctx_emit = emit_context_default(&kernels, &halos);

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

    let new_home = [10.0, 20.0, 30.0, 0.0];
    state.reset(new_home);

    let fresh = ShaperState::new(new_home, &shapers);

    assert_eq!(state.t_appended, fresh.t_appended);
    assert_eq!(state.t_decel_start, fresh.t_decel_start);
    assert_eq!(state.t_shaped, fresh.t_shaped);
    assert_eq!(state.t_dispatched, fresh.t_dispatched);
    assert!(state.uncommitted_moves.is_empty());
    assert!(state.pending_dispatch.is_empty());
    assert!(state.planned_fitted.is_empty());
    assert!(state.planned_meta.is_empty());

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

#[test]
fn current_position_reads_settled_endpoint_after_motion() {
    let shapers = replan_shapers();
    let mut state = ShaperState::new([0.0; 4], &shapers);
    let ctx_replan = replan_context();
    let kernels = replan_kernels_piecewise();
    let halos: Vec<EHalo> = Vec::new();
    let ctx_emit = emit_context_default(&kernels, &halos);

    let m1 = linear_x_segment(0.0, 200.0, 200.0);
    state.append_and_replan(m1, &ctx_replan).expect("append");
    let _ = state.emit_committed(&ctx_emit).expect("emit");
    assert!(state.t_appended > 0.0, "precondition: t_appended advanced");

    let pos = state.current_position();
    assert!(
        (pos[0] - 200.0).abs() < 1e-2,
        "X should settle at endpoint 200, got {}",
        pos[0]
    );
    assert!(
        (pos[1] - 0.0).abs() < 1e-2,
        "Y stays at home 0, got {}",
        pos[1]
    );
}

#[test]
fn current_position_on_fresh_shaped_state_reads_seed() {
    let shapers = replan_shapers();
    let state = ShaperState::new([7.0, 9.0, 5.0, 3.0], &shapers);
    let pos = state.current_position();
    assert!((pos[0] - 7.0).abs() < 1e-12, "X seed, got {}", pos[0]);
    assert!((pos[1] - 9.0).abs() < 1e-12, "Y seed, got {}", pos[1]);
    assert_eq!(
        pos[2], 0.0,
        "passthrough Z falls back to 0.0, got {}",
        pos[2]
    );
    assert_eq!(
        pos[3], 0.0,
        "none-shaper E falls back to 0.0, got {}",
        pos[3]
    );
}

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

#[test]
fn advance_idle_is_noop_when_target_not_past_t_appended() {
    let mut state = ShaperState::new([0.0; 4], &replan_shapers());
    let ctx_replan = replan_context();
    state
        .append_and_replan(linear_x_segment(0.0, 200.0, 200.0), &ctx_replan)
        .expect("append");
    let t_app_before = state.t_appended;
    let pieces_x_before = state.axes[0].pieces.len();

    state.advance_idle(state.t_appended * 0.5);

    assert!(
        (state.t_appended - t_app_before).abs() < 1e-12,
        "queued-ahead: t_appended must not change"
    );
    assert_eq!(
        state.axes[0].pieces.len(),
        pieces_x_before,
        "queued-ahead: no piece inserted"
    );
}

#[test]
fn advance_idle_when_drained_extends_to_target_preserving_position() {
    let mut state = ShaperState::new([0.0; 4], &replan_shapers());
    let ctx_replan = replan_context();
    state
        .append_and_replan(linear_x_segment(0.0, 200.0, 200.0), &ctx_replan)
        .expect("append");
    let kernels = replan_kernels_piecewise();
    let halos: Vec<EHalo> = Vec::new();
    let ctx_emit = emit_context_default(&kernels, &halos);
    let _ = state.emit_committed(&ctx_emit).expect("emit");
    let _ = state.commit_decel_to_zero(&ctx_emit).expect("commit");

    let t_app_before = state.t_appended;
    let pos_before = state.current_position();

    let target = t_app_before + 0.3;
    state.advance_idle(target);

    assert!(
        (state.t_appended - target).abs() < 1e-12,
        "t_appended -> target"
    );
    assert!(
        (state.t_decel_start - target).abs() < 1e-12,
        "t_decel_start -> target"
    );
    assert!(
        (state.t_dispatched - target).abs() < 1e-12,
        "t_dispatched must advance to target"
    );
    let pos_after = state.current_position();
    for i in 0..4 {
        assert!(
            (pos_after[i] - pos_before[i]).abs() < 1e-6,
            "axis {i} position must be continuous across the rest-hold"
        );
    }
    let last_x = state.axes[0].pieces.back().unwrap();
    assert!(
        (last_x.u_end - target).abs() < 1e-12,
        "hold piece u_end must equal target"
    );
    assert!(
        (last_x.u_start - t_app_before).abs() < 1e-12,
        "hold piece u_start must equal t_app_before"
    );
}

#[test]
fn commit_decel_to_zero_advances_t_dispatched_to_t_appended_and_is_idempotent() {
    let mut state = ShaperState::new([0.0; 4], &replan_shapers());
    let ctx_replan = replan_context();
    state
        .append_and_replan(linear_x_segment(0.0, 200.0, 200.0), &ctx_replan)
        .expect("append");
    let kernels = replan_kernels_piecewise();
    let halos: Vec<EHalo> = Vec::new();
    let ctx_emit = emit_context_default(&kernels, &halos);

    let partial = state.emit_committed(&ctx_emit).expect("emit");
    assert!(!partial.is_empty());
    assert!(
        state.t_dispatched < state.t_appended,
        "tail held back before commit"
    );

    let committed = state.commit_decel_to_zero(&ctx_emit).expect("commit");
    assert!(!committed.is_empty(), "commit emits the decel tail");
    assert!(
        (state.t_dispatched - state.t_appended).abs() < 1e-12,
        "after commit t_dispatched == t_appended"
    );

    let again = state.commit_decel_to_zero(&ctx_emit).expect("commit2");
    assert!(again.is_empty(), "second commit is a no-op");
    assert!((state.t_dispatched - state.t_appended).abs() < 1e-12);
}

#[test]
fn piece_stamps_monotone_across_idle_gap() {
    let mut state = ShaperState::new([0.0; 4], &replan_shapers());
    let ctx = replan_context();
    let kernels = replan_kernels_piecewise();
    let halos: Vec<EHalo> = Vec::new();
    let ctx_emit = emit_context_default(&kernels, &halos);

    state
        .append_and_replan(linear_x_segment(0.0, 200.0, 200.0), &ctx)
        .expect("m1");
    let _ = state.emit_committed(&ctx_emit).expect("emit1");
    let _ = state.commit_decel_to_zero(&ctx_emit).expect("commit1");
    let t_after_m1 = state.t_appended;

    state.advance_idle(t_after_m1 + 0.5);
    state
        .append_and_replan(linear_x_segment(200.0, 400.0, 200.0), &ctx)
        .expect("m2");
    let _ = state.emit_committed(&ctx_emit).expect("emit2");

    let stamps: Vec<f64> = state.axes[0].pieces.iter().map(|p| p.u_start).collect();
    for w in stamps.windows(2) {
        assert!(
            w[1] >= w[0] - 1e-12,
            "u_start went backward: {} -> {}",
            w[0],
            w[1]
        );
    }
}

#[test]
fn advance_idle_then_append_places_new_move_at_target() {
    let mut state = ShaperState::new([0.0; 4], &replan_shapers());
    let ctx = replan_context();
    let kernels = replan_kernels_piecewise();
    let halos: Vec<EHalo> = Vec::new();
    let ctx_emit = emit_context_default(&kernels, &halos);
    state
        .append_and_replan(linear_x_segment(0.0, 200.0, 200.0), &ctx)
        .expect("m1");
    let _ = state.emit_committed(&ctx_emit).expect("emit1");
    let _ = state.commit_decel_to_zero(&ctx_emit).expect("commit1");
    let t_after_m1 = state.t_appended;

    let target = t_after_m1 + 0.5;
    state.advance_idle(target);
    assert!(
        (state.t_dispatched - target).abs() < 1e-12,
        "t_dispatched must advance to target"
    );

    state
        .append_and_replan(linear_x_segment(200.0, 400.0, 200.0), &ctx)
        .expect("m2");
    let m2_start = state.uncommitted_moves.front().expect("m2 queued").t_start;
    assert!(
        (m2_start - target).abs() < 1e-9,
        "new move must start at target (now), got {m2_start} vs target {target}"
    );
}
