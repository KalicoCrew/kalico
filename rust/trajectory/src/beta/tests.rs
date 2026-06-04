use super::*;
use crate::{ELimits, ShapeBatchInput, ShapeSegmentInput, ShaperConfig};
use nurbs::VectorNurbs;

fn default_limits() -> temporal::Limits {
    temporal::Limits::new(
        [500.0, 500.0, 500.0],
        [5_000.0, 5_000.0, 5_000.0],
        [100_000.0, 100_000.0, 100_000.0],
        2_500.0,
    )
}

fn default_shaper_config() -> ShaperConfig {
    ShaperConfig {
        x: crate::RequiredShaper::SmoothZv {
            frequency_hz: 180.0,
        },
        y: crate::RequiredShaper::SmoothZv {
            frequency_hz: 120.0,
        },
        z: crate::AxisShaper::Passthrough,
    }
}

fn default_e_limits() -> ELimits {
    ELimits {
        v_max: 100.0,
        a_max: 5000.0,
    }
}

/// Build a degree-1 (truly linear) NURBS from `start` to `end`.
fn straight_linear(start: [f64; 3], end: [f64; 3]) -> VectorNurbs<f64, 3> {
    VectorNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![start, end]).unwrap()
}

// ------------------------------------------------------------------
// Test 1: Single straight-line segment — pipeline runs end-to-end
// ------------------------------------------------------------------
#[test]
fn single_straight_line_converges() {
    let curve = straight_linear([0.0, 0.0, 0.0], [50.0, 0.0, 0.0]);
    let generous_limits = temporal::Limits::new(
        [500.0, 500.0, 500.0],
        [5_000.0, 5_000.0, 5_000.0],
        [100_000.0, 100_000.0, 100_000.0],
        2_500.0,
    );
    let segments = [ShapeSegmentInput {
        temporal: temporal::multi::SegmentInput {
            curve: &curve,
            limits: generous_limits,
            trailing_junction_chord_tolerance_mm: 0.05,
        },
        e_mode: EMode::CoupledToXy,
        extrusion_per_xy_mm: 0.04,
        e_independent: None,
        feedrate_mm_s: 100.0,
    }];

    // Use very high machine-limit ceiling for the beta check so that
    // post-shape peaks (which are numerically inflated by the convolution
    // pipeline) don't trigger derating. The TOPP-RA planning limits
    // are the segment's own limits (5000 mm/s^2).
    let input = ShapeBatchInput {
        segments: &segments,
        grid_strategy: temporal::multi::GridStrategy::Fixed(10),
        worker_threads: 1,
        shaper: default_shaper_config(),
        fit_tolerance_mm: 0.5,
        beta_max_iters: 1,
        beta_convergence_ratio: 1.02,
        e_limits: default_e_limits(),
        initial_v: 0.0,
        terminal_v: 0.0,
    };

    let output = crate::shape_batch(&input).expect("should succeed");

    assert_eq!(output.segments.len(), 1);
    assert!(output.segments[0].t_end > output.segments[0].t_start);
    assert_eq!(output.segments[0].e_mode, EMode::CoupledToXy);
    assert!((output.segments[0].extrusion_per_xy_mm - 0.04).abs() < 1e-12);

    // The shaped axes should be non-trivial ScalarNurbs.
    for axis_nurbs in &output.segments[0].axes {
        assert!(
            axis_nurbs.control_points().len() >= 2,
            "shaped axis should have at least 2 control points"
        );
    }
}

// ------------------------------------------------------------------
// Test 2: Two segments with E-gap — pipeline handles partition
// ------------------------------------------------------------------
#[test]
fn two_segments_with_e_gap() {
    let curve1 = straight_linear([0.0, 0.0, 0.0], [50.0, 0.0, 0.0]);
    let curve2 = straight_linear([50.0, 0.0, 0.0], [100.0, 0.0, 0.0]);
    let e_hold = straight_linear([50.0, 0.0, 0.0], [50.0, 0.0, 0.0]);
    let e_nurbs =
        nurbs::ScalarNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![10.0, 5.0]).unwrap();

    let segments = [
        ShapeSegmentInput {
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
        ShapeSegmentInput {
            temporal: temporal::multi::SegmentInput {
                curve: &e_hold,
                limits: default_limits(),
                trailing_junction_chord_tolerance_mm: 0.05,
            },
            e_mode: EMode::Independent,
            extrusion_per_xy_mm: 0.0,
            e_independent: Some(&e_nurbs),
            feedrate_mm_s: 50.0,
        },
        ShapeSegmentInput {
            temporal: temporal::multi::SegmentInput {
                curve: &curve2,
                limits: default_limits(),
                trailing_junction_chord_tolerance_mm: 0.05,
            },
            e_mode: EMode::CoupledToXy,
            extrusion_per_xy_mm: 0.04,
            e_independent: None,
            feedrate_mm_s: 100.0,
        },
    ];

    let input = ShapeBatchInput {
        segments: &segments,
        grid_strategy: temporal::multi::GridStrategy::Fixed(10),
        worker_threads: 1,
        shaper: default_shaper_config(),
        fit_tolerance_mm: 0.5,
        beta_max_iters: 1,
        beta_convergence_ratio: 1.02,
        e_limits: default_e_limits(),
        initial_v: 0.0,
        terminal_v: 0.0,
    };

    let output = crate::shape_batch(&input).expect("should succeed");

    // Three output segments: [XY, Independent-E, XY].
    assert_eq!(output.segments.len(), 3);
    assert_eq!(output.segments[0].e_mode, EMode::CoupledToXy);
    assert_eq!(output.segments[1].e_mode, EMode::Independent);
    assert_eq!(output.segments[2].e_mode, EMode::CoupledToXy);

    // The E-gap segment should have an independent E NURBS.
    assert!(output.segments[1].e_independent.is_some());

    // Time ordering: each segment starts after the previous ends.
    assert!(output.segments[0].t_end <= output.segments[1].t_start + 1e-9);
    assert!(output.segments[1].t_end <= output.segments[2].t_start + 1e-9);
}

// ------------------------------------------------------------------
// Test 3: Derate logic unit test
// ------------------------------------------------------------------
#[test]
fn derate_detects_exceeding_peaks() {
    // Build a one-segment fitted with all axes spanning >> MIN_AXIS_SPAN_FOR_DERATE
    // so the inactive-axis skip does not apply.
    let make_axis = |x_start: f64, x_end: f64| {
        nurbs::bezier::bezier_pieces_to_nurbs(&[nurbs::bezier::BezierPiece {
            u_start: 0.0,
            u_end: 1.0,
            coeffs: vec![x_start, x_end - x_start],
        }])
    };
    let fitted = vec![crate::fit::FittedSegment {
        axes: [
            make_axis(0.0, 100.0),
            make_axis(0.0, 100.0),
            make_axis(0.0, 100.0),
        ],
        t_start: 0.0,
        t_end: 1.0,
    }];
    let machine = vec![[5000.0, 5000.0, 5000.0]];
    let peaks_within = vec![[4000.0, 3000.0, 2000.0]];
    let info = compute_derate(&peaks_within, &machine, &fitted);
    assert!(!info.needs_derate);

    let peaks_exceed = vec![[6000.0, 3000.0, 2000.0]];
    let info = compute_derate(&peaks_exceed, &machine, &fitted);
    assert!(info.needs_derate);
    assert!((info.worst_ratio - 1.2).abs() < 1e-10);
    assert_eq!(info.exceeding_indices, vec![0]);
}

// ------------------------------------------------------------------
// Test 4: All-E-gap batch
// ------------------------------------------------------------------
#[test]
fn all_e_gaps_output() {
    let e_hold = straight_linear([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
    let e_nurbs =
        nurbs::ScalarNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![10.0, 5.0]).unwrap();

    let segments = [ShapeSegmentInput {
        temporal: temporal::multi::SegmentInput {
            curve: &e_hold,
            limits: default_limits(),
            trailing_junction_chord_tolerance_mm: 0.05,
        },
        e_mode: EMode::Independent,
        extrusion_per_xy_mm: 0.0,
        e_independent: Some(&e_nurbs),
        feedrate_mm_s: 50.0,
    }];

    let input = ShapeBatchInput {
        segments: &segments,
        grid_strategy: temporal::multi::GridStrategy::Fixed(10),
        worker_threads: 1,
        shaper: default_shaper_config(),
        fit_tolerance_mm: 0.5,
        beta_max_iters: 1,
        beta_convergence_ratio: 1.02,
        e_limits: default_e_limits(),
        initial_v: 0.0,
        terminal_v: 0.0,
    };

    let output = crate::shape_batch(&input).expect("should succeed");

    assert_eq!(output.segments.len(), 1);
    assert_eq!(output.segments[0].e_mode, EMode::Independent);
    assert!(output.segments[0].e_independent.is_some());
    assert!(output.segments[0].t_end > output.segments[0].t_start);
}

// ------------------------------------------------------------------
// Test 5: effective_machine_a_max — only last segment is derated
// ------------------------------------------------------------------
//
// Direct unit test of the spec §3.6 derate-target selection. The
// end-to-end planner can't witness this exactly because TOPP-RA's
// joining loop propagates segment N's tighter limit back through
// segment N-1's tail. This test asserts the invariant at its source.
#[test]
fn effective_machine_a_max_terminal_known_is_identity() {
    let machine = vec![
        [5_000.0, 5_000.0, 5_000.0],
        [3_000.0, 4_000.0, 2_500.0],
        [1_000.0, 1_500.0, 2_000.0],
    ];
    let effective = effective_machine_a_max(&machine, SafetyMode::TerminalKnown);
    assert_eq!(effective, machine);
}

#[test]
fn effective_machine_a_max_worst_case_only_halves_last_segment() {
    let machine = vec![
        [5_000.0, 5_000.0, 5_000.0],
        [3_000.0, 4_000.0, 2_500.0],
        [1_000.0, 1_500.0, 2_000.0],
    ];
    let effective = effective_machine_a_max(&machine, SafetyMode::WorstCaseFuture);

    // Segments 0 and 1 must be unchanged.
    assert_eq!(effective[0], machine[0]);
    assert_eq!(effective[1], machine[1]);
    // Segment 2 (last) is halved on every axis.
    for axis in 0..3 {
        assert!(
            (effective[2][axis] - machine[2][axis] * 0.5).abs() < 1e-12,
            "axis {axis}: expected {} (half of {}), got {}",
            machine[2][axis] * 0.5,
            machine[2][axis],
            effective[2][axis],
        );
    }
}

#[test]
fn effective_machine_a_max_worst_case_single_segment() {
    // Single-segment edge case: the only segment IS the last segment.
    let machine = vec![[5_000.0, 4_000.0, 3_000.0]];
    let effective = effective_machine_a_max(&machine, SafetyMode::WorstCaseFuture);
    assert_eq!(effective.len(), 1);
    for axis in 0..3 {
        assert!((effective[0][axis] - machine[0][axis] * 0.5).abs() < 1e-12);
    }
}

#[test]
fn effective_machine_a_max_worst_case_empty_is_empty() {
    let machine: Vec<[f64; 3]> = vec![];
    let effective = effective_machine_a_max(&machine, SafetyMode::WorstCaseFuture);
    assert!(effective.is_empty());
}
