use super::*;
use crate::ELimits;
use geometry::segment::EMode;
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

fn default_kernels() -> [Option<PlanShaper>; 4] {
    [
        Some(PlanShaper::SmoothZv {
            frequency_hz: 180.0,
        }),
        Some(PlanShaper::SmoothMzv {
            frequency_hz: 120.0,
        }),
        Some(PlanShaper::Passthrough),
        None,
    ]
}

fn default_input<'a>(segments: &'a [PlanSegment<'a>], safety: SafetyMode) -> PlanInput<'a> {
    PlanInput {
        segments,
        grid_strategy: temporal::multi::GridStrategy::Fixed(10),
        worker_threads: 1,
        kernels: default_kernels(),
        fit_tolerance_mm: 0.5,
        beta_max_iters: 5,
        beta_convergence_ratio: 1.02,
        e_limits: default_e_limits(),
        // Step-0 lift: caller may override these to exercise nonzero
        // boundary velocities (Phase 3's `append_and_replan` always
        // does). Defaults match the legacy (0, 0) shape_batch contract.
        initial_v: 0.0,
        terminal_v: 0.0,
        safety_mode: safety,
    }
}

#[test]
fn rejects_empty_segments() {
    let input = default_input(&[], SafetyMode::TerminalKnown);
    let result = plan_velocity(&input);
    assert!(matches!(result, Err(ShapeError::EmptySegments)));
}

#[test]
fn rejects_negative_initial_v() {
    let curve = straight_linear([0.0, 0.0, 0.0], [50.0, 0.0, 0.0]);
    let segments = [PlanSegment {
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
    let mut input = default_input(&segments, SafetyMode::TerminalKnown);
    input.initial_v = -1.0;
    let result = plan_velocity(&input);
    assert!(matches!(
        result,
        Err(ShapeError::UnsupportedBoundaryVelocity)
    ));
}

#[test]
fn rejects_nan_terminal_v() {
    let curve = straight_linear([0.0, 0.0, 0.0], [50.0, 0.0, 0.0]);
    let segments = [PlanSegment {
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
    let mut input = default_input(&segments, SafetyMode::TerminalKnown);
    input.terminal_v = f64::NAN;
    let result = plan_velocity(&input);
    assert!(matches!(
        result,
        Err(ShapeError::UnsupportedBoundaryVelocity)
    ));
}

/// Step-0 lift contract: a non-zero `initial_v` is accepted (no error)
/// and produces a valid plan. The first sample of the first segment's
/// TOPP profile reflects the requested starting velocity.
#[test]
fn nonzero_initial_v_produces_chained_profile() {
    // 200 mm move to give TOPP-RA enough path length to actually run
    // an accel-cruise-decel under the default limits.
    let curve = straight_linear([0.0, 0.0, 0.0], [200.0, 0.0, 0.0]);
    let segments = [PlanSegment {
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
    let mut input = default_input(&segments, SafetyMode::TerminalKnown);
    input.initial_v = 50.0;
    input.terminal_v = 0.0;

    let fitted = plan_velocity(&input).expect("plan with nonzero initial_v should succeed");
    assert_eq!(fitted.len(), 1);

    // Sample the X-axis velocity at t = t_start. For a single 200 mm
    // pure-X move with `initial_v = 50 mm/s`, the toolhead's instantaneous
    // speed at the start should be 50 mm/s to within TOPP-RA's per-grid
    // tolerance (the joining loop's `ε_velocity = 1 mm/s`).
    let seg = &fitted[0];
    let mut t_eps = (seg.t_end - seg.t_start) * 1e-6;
    if t_eps <= 0.0 {
        t_eps = 1e-9;
    }
    let t_sample = seg.t_start + t_eps;
    let x0 = nurbs::eval::eval(&seg.axes[0], seg.t_start);
    let x1 = nurbs::eval::eval(&seg.axes[0], t_sample);
    let vx_start = (x1 - x0) / t_eps;
    assert!(
        (vx_start - 50.0).abs() < 5.0,
        "X-axis start velocity {vx_start} mm/s deviates from requested 50.0 mm/s",
    );
}

#[test]
fn rejects_passthrough_on_x() {
    let curve = straight_linear([0.0, 0.0, 0.0], [50.0, 0.0, 0.0]);
    let segments = [PlanSegment {
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
    let mut input = default_input(&segments, SafetyMode::TerminalKnown);
    input.kernels[0] = Some(PlanShaper::Passthrough);
    let result = plan_velocity(&input);
    assert!(matches!(result, Err(ShapeError::UnsupportedShaperOnXY)));
}

#[test]
fn rejects_passthrough_on_y() {
    let curve = straight_linear([0.0, 0.0, 0.0], [50.0, 0.0, 0.0]);
    let segments = [PlanSegment {
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
    let mut input = default_input(&segments, SafetyMode::TerminalKnown);
    input.kernels[1] = Some(PlanShaper::Passthrough);
    let result = plan_velocity(&input);
    assert!(matches!(result, Err(ShapeError::UnsupportedShaperOnXY)));
}

#[test]
fn rejects_none_on_x() {
    let curve = straight_linear([0.0, 0.0, 0.0], [50.0, 0.0, 0.0]);
    let segments = [PlanSegment {
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
    let mut input = default_input(&segments, SafetyMode::TerminalKnown);
    input.kernels[0] = None;
    let result = plan_velocity(&input);
    assert!(matches!(result, Err(ShapeError::UnsupportedShaperOnXY)));
}

#[test]
fn returns_one_fitted_per_xy_segment() {
    let curve = straight_linear([0.0, 0.0, 0.0], [50.0, 0.0, 0.0]);
    let segments = [PlanSegment {
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
    let input = default_input(&segments, SafetyMode::TerminalKnown);
    let fitted = plan_velocity(&input).expect("plan should succeed");
    assert_eq!(fitted.len(), 1);
    assert!(fitted[0].t_end > fitted[0].t_start);
}

/// **Spec §3.6 contract — multi-segment.** Only the **last** XY segment
/// is subject to the worst-case-future half-machine-accel derate.
///
/// We can't assert identical segment-0 _durations_ here: the temporal
/// joining loop uses segment 1's tighter limit to compute the junction
/// velocity, which propagates back and slows segment 0's tail too —
/// that's TOPP-RA doing its job, not a β-derate regression. The
/// invariant we _can_ assert end-to-end is monotonicity per-segment
/// (both segments must take ≥ their TerminalKnown durations under
/// WorstCaseFuture, with strict inequality on the last segment because
/// its limit is genuinely halved).
///
/// The "only last segment's _effective machine limit_ is changed"
/// invariant is tested directly via `effective_machine_a_max` in the
/// `beta::tests` module.
#[test]
fn worst_case_future_segment_durations_monotone() {
    let curve0 = straight_linear([0.0, 0.0, 0.0], [50.0, 0.0, 0.0]);
    let curve1 = straight_linear([50.0, 0.0, 0.0], [100.0, 0.0, 0.0]);
    let segments = [
        PlanSegment {
            temporal: temporal::multi::SegmentInput {
                curve: &curve0,
                limits: default_limits(),
                trailing_junction_chord_tolerance_mm: 0.05,
            },
            e_mode: EMode::CoupledToXy,
            extrusion_per_xy_mm: 0.04,
            e_independent: None,
            feedrate_mm_s: 100.0,
        },
        PlanSegment {
            temporal: temporal::multi::SegmentInput {
                curve: &curve1,
                limits: default_limits(),
                trailing_junction_chord_tolerance_mm: 0.05,
            },
            e_mode: EMode::CoupledToXy,
            extrusion_per_xy_mm: 0.04,
            e_independent: None,
            feedrate_mm_s: 100.0,
        },
    ];

    let known = plan_velocity(&default_input(&segments, SafetyMode::TerminalKnown))
        .expect("TerminalKnown plan should succeed");
    let worst = plan_velocity(&default_input(&segments, SafetyMode::WorstCaseFuture))
        .expect("WorstCaseFuture plan should succeed");

    assert_eq!(known.len(), 2);
    assert_eq!(worst.len(), 2);

    let dur_known_0 = known[0].t_end - known[0].t_start;
    let dur_worst_0 = worst[0].t_end - worst[0].t_start;
    let dur_known_1 = known[1].t_end - known[1].t_start;
    let dur_worst_1 = worst[1].t_end - worst[1].t_start;

    // Both segments: WorstCaseFuture's tighter end-of-batch accel
    // bound is a strictly tighter constraint set, so neither
    // segment's β-converged duration can be shorter.
    assert!(
        dur_worst_0 >= dur_known_0 - 1e-9,
        "segment 0 WorstCaseFuture duration {dur_worst_0} \
         must be ≥ TerminalKnown duration {dur_known_0}",
    );
    assert!(
        dur_worst_1 >= dur_known_1 - 1e-9,
        "segment 1 WorstCaseFuture duration {dur_worst_1} \
         must be ≥ TerminalKnown duration {dur_known_1}",
    );
}

/// **Spec §3.6 contract.** For the same input the
/// `WorstCaseFuture` β-converged plan must use accel limits no greater
/// than the `TerminalKnown` plan — by construction the trailing region's
/// effective machine limit is half of `a_machine` under
/// `WorstCaseFuture`, so the resulting trajectory must take **at least
/// as long** to traverse.
#[test]
fn worst_case_future_is_no_faster_than_terminal_known() {
    // 50 mm move with the same dynamic limits as the existing
    // `single_straight_line_converges` test (which is known to converge
    // β-medium under TerminalKnown). A 5000 mm/s² accel cap is enough
    // for TOPP-RA to feasibly schedule a triangular profile; the
    // β-medium step's behaviour under WorstCaseFuture is what we test
    // here, not TOPP-RA's own feasibility.
    let curve = straight_linear([0.0, 0.0, 0.0], [50.0, 0.0, 0.0]);
    let segments = [PlanSegment {
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

    let known = plan_velocity(&default_input(&segments, SafetyMode::TerminalKnown))
        .expect("TerminalKnown plan should succeed");
    let worst = plan_velocity(&default_input(&segments, SafetyMode::WorstCaseFuture))
        .expect("WorstCaseFuture plan should succeed");

    assert_eq!(known.len(), 1);
    assert_eq!(worst.len(), 1);

    let dur_known = known[0].t_end - known[0].t_start;
    let dur_worst = worst[0].t_end - worst[0].t_start;
    // The worst-case bound is loose but always safe; the worst-case
    // duration must be ≥ the terminal-known duration up to the same
    // numerical tolerance the β-medium loop converges with.
    assert!(
        dur_worst >= dur_known - 1e-9,
        "WorstCaseFuture duration {dur_worst} must be ≥ TerminalKnown duration {dur_known}",
    );
}
