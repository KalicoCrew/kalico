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

#[test]
fn nonzero_initial_v_produces_chained_profile() {
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

    let out = plan_velocity(&input).expect("plan with nonzero initial_v should succeed");
    assert_eq!(out.fitted.len(), 1);

    let seg = &out.fitted[0];
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
fn passthrough_on_x_is_valid() {
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
    assert!(result.is_ok(), "passthrough on X must succeed, got: {result:?}");
}

#[test]
fn passthrough_on_y_is_valid() {
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
    assert!(result.is_ok(), "passthrough on Y must succeed, got: {result:?}");
}

#[test]
fn none_on_x_treated_as_passthrough() {
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
    assert!(result.is_ok(), "None on X must be treated as passthrough, got: {result:?}");
}

#[test]
fn all_passthrough_produces_fitted_output() {
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
    input.kernels = [None, None, None, None];
    let out = plan_velocity(&input).expect("all-passthrough plan must succeed");
    assert_eq!(out.fitted.len(), 1);
    assert!(out.fitted[0].t_end > out.fitted[0].t_start);
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
    let out = plan_velocity(&input).expect("plan should succeed");
    assert_eq!(out.fitted.len(), 1);
    assert!(out.fitted[0].t_end > out.fitted[0].t_start);
}

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

    assert_eq!(known.fitted.len(), 2);
    assert_eq!(worst.fitted.len(), 2);

    let dur_known_0 = known.fitted[0].t_end - known.fitted[0].t_start;
    let dur_worst_0 = worst.fitted[0].t_end - worst.fitted[0].t_start;
    let dur_known_1 = known.fitted[1].t_end - known.fitted[1].t_start;
    let dur_worst_1 = worst.fitted[1].t_end - worst.fitted[1].t_start;

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

#[test]
fn worst_case_future_is_no_faster_than_terminal_known() {
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

    assert_eq!(known.fitted.len(), 1);
    assert_eq!(worst.fitted.len(), 1);

    let dur_known = known.fitted[0].t_end - known.fitted[0].t_start;
    let dur_worst = worst.fitted[0].t_end - worst.fitted[0].t_start;
    assert!(
        dur_worst >= dur_known - 1e-9,
        "WorstCaseFuture duration {dur_worst} must be ≥ TerminalKnown duration {dur_known}",
    );
}
