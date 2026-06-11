use super::*;
use crate::fit::{fit_and_split, FittedSegment};
use crate::{
    plan_velocity, AxisShaper, ELimits, PlanInput, PlanSegment, PlanShaper, SafetyMode,
    ShapeBatchInput, ShapeSegmentInput, ShaperConfig,
};
use geometry::segment::EMode;
use nurbs::bezier::{bezier_pieces_to_nurbs, extract_bezier_pieces, BezierPiece};
use nurbs::VectorNurbs;

fn straight_linear(start: [f64; 3], end: [f64; 3]) -> VectorNurbs<f64, 3> {
    VectorNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![start, end]).unwrap()
}

fn default_limits() -> temporal::Limits {
    temporal::Limits::new(
        [500.0, 500.0, 500.0],
        [5_000.0, 5_000.0, 5_000.0],
        [100_000.0, 100_000.0, 100_000.0],
        2_500.0,
    )
}

fn default_e_limits() -> ELimits {
    ELimits {
        v_max: 100.0,
        a_max: 5_000.0,
    }
}

fn default_shaper_config() -> ShaperConfig {
    ShaperConfig {
        x: AxisShaper::SmoothZv {
            frequency_hz: 180.0,
        },
        y: AxisShaper::SmoothZv {
            frequency_hz: 120.0,
        },
        z: AxisShaper::Passthrough,
    }
}

fn default_kernels() -> [Option<PlanShaper>; 4] {
    [
        Some(PlanShaper::SmoothZv {
            frequency_hz: 180.0,
        }),
        Some(PlanShaper::SmoothZv {
            frequency_hz: 120.0,
        }),
        Some(PlanShaper::Passthrough),
        None,
    ]
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
        "{label}: knots differ by {max_knot_diff:.2e}"
    );
    assert_eq!(
        a.control_points().len(),
        b.control_points().len(),
        "{label}: control point count differs"
    );
    let max_cp_diff = a
        .control_points()
        .iter()
        .zip(b.control_points().iter())
        .map(|(ca, cb)| (ca - cb).abs())
        .fold(0.0_f64, f64::max);
    assert!(
        max_cp_diff < 1e-12,
        "{label}: control points differ by {max_cp_diff:.2e} mm"
    );
}

#[test]
fn empty_history_matches_shape_batch_byte_identical() {
    let curve = straight_linear([0.0, 0.0, 0.0], [50.0, 0.0, 0.0]);
    let plan_segs = [PlanSegment {
        temporal: temporal::multi::SegmentInput {
            curve: &curve,
            limits: default_limits(),
            trailing_junction_chord_tolerance_mm: 0.05,
        },
        e_mode: EMode::CoupledToXy,
        extrusion_per_xy_mm: 0.04,
        e_independent: None,
        feedrate_mm_s: 100.0,
    }];

    let plan_input = PlanInput {
        segments: &plan_segs,
        grid_strategy: temporal::multi::GridStrategy::Fixed(10),
        worker_threads: 1,
        kernels: default_kernels(),
        fit_tolerance_mm: 0.5,
        beta_max_iters: 5,
        beta_convergence_ratio: 1.02,
        e_limits: default_e_limits(),
        initial_v: 0.0,
        initial_a: 0.0,
        terminal_v: 0.0,
        safety_mode: SafetyMode::TerminalKnown,
        start_d2_override: None,
    };
    let planned = plan_velocity(&plan_input)
        .expect("plan_velocity should succeed")
        .fitted;
    assert_eq!(planned.len(), 1);

    let kernels: [Option<PiecewisePolynomialKernel<f64>>; 4] = [
        AxisShaper::SmoothZv {
            frequency_hz: 180.0,
        }
        .to_kernel(),
        AxisShaper::SmoothZv {
            frequency_hz: 120.0,
        }
        .to_kernel(),
        None,
        None,
    ];
    let meta = [EmitSegmentMeta {
        e_mode: EMode::CoupledToXy,
        extrusion_per_xy_mm: 0.04,
    }];

    let batch_t_start = 0.0;
    let batch_t_end = planned[0].t_end;

    let emitted = emit_shaped(
        &planned,
        &meta,
        &kernels,
        &[],
        &PerAxisHistory::empty(),
        batch_t_start,
        batch_t_end,
    )
    .expect("emit_shaped should succeed");

    let segs = [ShapeSegmentInput {
        temporal: plan_segs[0].temporal,
        e_mode: plan_segs[0].e_mode,
        extrusion_per_xy_mm: plan_segs[0].extrusion_per_xy_mm,
        e_independent: plan_segs[0].e_independent,
        feedrate_mm_s: plan_segs[0].feedrate_mm_s,
    }];
    let shape_input = ShapeBatchInput {
        segments: &segs,
        grid_strategy: temporal::multi::GridStrategy::Fixed(10),
        worker_threads: 1,
        shaper: default_shaper_config(),
        fit_tolerance_mm: 0.5,
        beta_max_iters: 5,
        beta_convergence_ratio: 1.02,
        e_limits: default_e_limits(),
        initial_v: 0.0,
        initial_a: 0.0,
        terminal_v: 0.0,
        start_d2_override: None,
    };
    let reference = crate::shape_batch(&shape_input).expect("shape_batch should succeed");

    assert_eq!(emitted.len(), reference.segments.len());
    for (i, (a, b)) in emitted.iter().zip(reference.segments.iter()).enumerate() {
        assert_nurbs_near_equal(&a.axes[0], &b.axes[0], &format!("seg{i} X"));
        assert_nurbs_near_equal(&a.axes[1], &b.axes[1], &format!("seg{i} Y"));
        assert_nurbs_near_equal(&a.axes[2], &b.axes[2], &format!("seg{i} Z"));
        assert_eq!(a.e_mode, b.e_mode, "seg{i}: e_mode differs");
        assert!(
            (a.extrusion_per_xy_mm - b.extrusion_per_xy_mm).abs() < 1e-15,
            "seg{i}: extrusion_per_xy_mm differs",
        );
        #[allow(clippy::float_cmp)]
        {
            assert_eq!(a.t_start, b.t_start, "seg{i}: t_start differs");
            assert_eq!(a.t_end, b.t_end, "seg{i}: t_end differs");
        }
    }
}

#[test]
fn pad_segment_axis_with_history_seam_reads_history_tail() {
    let x_nurbs = bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: 1.0,
        u_end: 2.0,
        coeffs: vec![10.0, 20.0],
    }]);
    let y_nurbs = bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: 1.0,
        u_end: 2.0,
        coeffs: vec![0.0],
    }]);
    let z_nurbs = bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: 1.0,
        u_end: 2.0,
        coeffs: vec![0.0],
    }]);
    let fitted = vec![FittedSegment {
        axes: [x_nurbs, y_nurbs, z_nurbs],
        t_start: 1.0,
        t_end: 2.0,
    }];

    let history_x = vec![BezierPiece {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![0.0, 10.0],
    }];

    let t_sm_half = 0.3;
    let padded = crate::pad::pad_segment_axis_with_history(
        0,
        0,
        &fitted,
        &[],
        &history_x,
        t_sm_half,
        1.0,
        2.0,
    );

    let pieces = extract_bezier_pieces(&padded);
    assert!(
        pieces[0].u_start <= 0.7 + 1e-12,
        "padded must cover at least back to 0.7, got {}",
        pieces[0].u_start,
    );

    let val_08 = pieces
        .iter()
        .find(|p| 0.8 >= p.u_start - 1e-12 && 0.8 <= p.u_end + 1e-12)
        .expect("padded curve should cover t = 0.8")
        .evaluate(0.8);
    assert!(
        (val_08 - 8.0).abs() < 1e-9,
        "expected 8.0 from history at t=0.8, got {val_08}",
    );

    let val_10 = pieces
        .iter()
        .find(|p| 1.0 >= p.u_start - 1e-12 && 1.0 <= p.u_end + 1e-12)
        .expect("padded curve should cover t = 1.0")
        .evaluate(1.0);
    assert!(
        (val_10 - 10.0).abs() < 1e-9,
        "expected 10.0 at seam, got {val_10}",
    );

    let padded_no_history = crate::pad::pad_segment_axis(0, 0, &fitted, &[], t_sm_half, 1.0, 2.0);
    let pieces_no_history = extract_bezier_pieces(&padded_no_history);
    let val_08_no_history = pieces_no_history
        .iter()
        .find(|p| 0.8 >= p.u_start - 1e-12 && 0.8 <= p.u_end + 1e-12)
        .expect("padded curve should cover t = 0.8")
        .evaluate(0.8);
    // With no history the left pad continues at the segment's entry velocity (slope 20 through
    // position 10 at the t=1.0 seam) rather than holding the start position: at t=0.8 that is
    // 10 + 20*(0.8 - 1.0) = 6.0.
    assert!(
        (val_08_no_history - 6.0).abs() < 1e-9,
        "no-history path should extrapolate at entry velocity to 6.0 at t=0.8, got {val_08_no_history}",
    );
    assert!(
        (val_08 - val_08_no_history).abs() > 1.0,
        "history vs no-history must disagree at t=0.8 (history 8.0 vs vel-extrapolated 6.0)",
    );
}

#[test]
fn empty_history_pad_matches_legacy() {
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
    let fitted = vec![FittedSegment {
        axes: [x_nurbs, y_nurbs, z_nurbs],
        t_start: 0.0,
        t_end: 1.0,
    }];

    let t_sm_half = 0.1;
    for axis in 0..3 {
        let with_history = crate::pad::pad_segment_axis_with_history(
            0,
            axis,
            &fitted,
            &[],
            &[],
            t_sm_half,
            0.0,
            1.0,
        );
        let legacy = crate::pad::pad_segment_axis(0, axis, &fitted, &[], t_sm_half, 0.0, 1.0);
        assert_nurbs_near_equal(&with_history, &legacy, &format!("axis {axis}"));
    }
}

#[test]
fn constant_y_axis_emits_cubic_matching_moving_x_corexy_degree_invariant() {
    // Regression: when the fitter produces a degree-5 FittedSegment and Y is
    // bitwise-constant, emit_shaped returned the fitter's native degree-5 curve
    // for Y while fitting X to degree-3 via fit_c2_cubic. The degree mismatch
    // caused add_with_knot_union to return KnotMismatch and panicked at
    // motion-bridge/src/enqueue.rs:30 on any CoreXY dispatch.
    //
    // Trigger condition: pure-X jogs queued back-to-back while the first is
    // in flight; the terminal-decel splice rebuilds Y as bitwise-constant.

    let x_composed: Vec<[BezierPiece<f64>; 3]> = (0..4)
        .map(|i| {
            let s = f64::from(i);
            [
                BezierPiece {
                    u_start: s,
                    u_end: s + 1.0,
                    coeffs: vec![s * 10.0, 10.0, 0.5, 0.1],
                },
                BezierPiece {
                    u_start: s,
                    u_end: s + 1.0,
                    coeffs: vec![0.0],
                },
                BezierPiece {
                    u_start: s,
                    u_end: s + 1.0,
                    coeffs: vec![0.0],
                },
            ]
        })
        .collect();

    let fitted_from_fitter =
        fit_and_split(&x_composed, 0.005, None).expect("fit_and_split must succeed");

    let degree5_x = &fitted_from_fitter.axes[0];
    assert!(
        degree5_x.degree() >= 4,
        "expected fitter to produce degree >= 4 for X, got {}",
        degree5_x.degree(),
    );

    let y_constant_val = 25.0_f64;
    let degree5_constant_y = bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: fitted_from_fitter.t_start,
        u_end: fitted_from_fitter.t_end,
        coeffs: vec![y_constant_val, 0.0, 0.0, 0.0, 0.0, 0.0],
    }]);
    assert_eq!(
        degree5_constant_y.degree(),
        degree5_x.degree(),
        "test setup: Y must be same degree as X to match the live crash precondition",
    );

    let constant_cps = degree5_constant_y.control_points();
    let all_equal = constant_cps
        .iter()
        .all(|c| (c - constant_cps[0]).abs() < 1e-12);
    assert!(
        all_equal,
        "test setup: Y control points must be bitwise-constant to trigger the bug branch",
    );

    let constant_z = bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: fitted_from_fitter.t_start,
        u_end: fitted_from_fitter.t_end,
        coeffs: vec![0.0; degree5_x.degree() as usize + 1],
    }]);

    let fitted = FittedSegment {
        axes: [degree5_x.clone(), degree5_constant_y, constant_z],
        t_start: fitted_from_fitter.t_start,
        t_end: fitted_from_fitter.t_end,
    };

    let kernels: [Option<PiecewisePolynomialKernel<f64>>; 4] = [
        AxisShaper::SmoothZv {
            frequency_hz: 186.0,
        }
        .to_kernel(),
        None,
        None,
        None,
    ];
    let meta = [EmitSegmentMeta {
        e_mode: EMode::CoupledToXy,
        extrusion_per_xy_mm: 0.04,
    }];

    let emitted = emit_shaped(
        &[fitted],
        &meta,
        &kernels,
        &[],
        &PerAxisHistory::empty(),
        fitted_from_fitter.t_start,
        fitted_from_fitter.t_end,
    )
    .expect("emit_shaped must not return an error");

    for (i, seg) in emitted.iter().enumerate() {
        assert_eq!(
            seg.axes[0].degree(),
            seg.axes[1].degree(),
            "segment {i}: X degree {} != Y degree {} — CoreXY motor-union \
             add_with_knot_union will panic with KnotMismatch (constant-Y \
             axis must be refit to cubic, not returned as-is from the fitter)",
            seg.axes[0].degree(),
            seg.axes[1].degree(),
        );
    }
}
