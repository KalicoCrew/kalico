use super::*;
use crate::{ELimits, ShapeSegmentInput};
use geometry::segment::EMode;
use nurbs::{ScalarNurbs, VectorNurbs};

/// Build a simple cubic Bezier XYZ curve from `start` to `end` (straight line).
fn make_xyz_curve(start: [f64; 3], end: [f64; 3]) -> VectorNurbs<f64, 3> {
    // Cubic Bezier with collinear control points at 1/3 and 2/3 lerp.
    let cp1 = [
        start[0] + (end[0] - start[0]) / 3.0,
        start[1] + (end[1] - start[1]) / 3.0,
        start[2] + (end[2] - start[2]) / 3.0,
    ];
    let cp2 = [
        start[0] + 2.0 * (end[0] - start[0]) / 3.0,
        start[1] + 2.0 * (end[1] - start[1]) / 3.0,
        start[2] + 2.0 * (end[2] - start[2]) / 3.0,
    ];
    VectorNurbs::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![start, cp1, cp2, end],
    )
    .unwrap()
}

/// Build a linear E NURBS for retraction/prime.
fn make_e_nurbs(e_start: f64, e_end: f64) -> ScalarNurbs<f64> {
    ScalarNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![e_start, e_end]).unwrap()
}

fn default_limits() -> temporal::Limits {
    temporal::Limits::new(
        [500.0, 500.0, 500.0],
        [10_000.0, 10_000.0, 10_000.0],
        [100_000.0, 100_000.0, 100_000.0],
        5_000.0,
    )
}

fn default_e_limits() -> ELimits {
    ELimits {
        v_max: 100.0,
        a_max: 5000.0,
    }
}

fn make_xy_segment(
    curve: &VectorNurbs<f64, 3>,
    e_mode: EMode,
    extrusion_per_xy_mm: f64,
) -> ShapeSegmentInput<'_> {
    ShapeSegmentInput {
        temporal: temporal::multi::SegmentInput {
            curve,
            limits: default_limits(),
            trailing_junction_chord_tolerance_mm: 0.05,
        },
        e_mode,
        extrusion_per_xy_mm,
        e_independent: None,
        feedrate_mm_s: 100.0,
    }
}

fn make_independent_segment<'a>(
    curve: &'a VectorNurbs<f64, 3>,
    e_nurbs: &'a ScalarNurbs<f64>,
) -> ShapeSegmentInput<'a> {
    ShapeSegmentInput {
        temporal: temporal::multi::SegmentInput {
            curve,
            limits: default_limits(),
            trailing_junction_chord_tolerance_mm: 0.05,
        },
        e_mode: EMode::Independent,
        extrusion_per_xy_mm: 0.0,
        e_independent: Some(e_nurbs),
        feedrate_mm_s: 50.0,
    }
}

#[test]
fn partition_all_xy_single_run() {
    // All segments are CoupledToXy — should produce 1 run, 0 gaps.
    let c1 = make_xyz_curve([0.0, 0.0, 0.0], [10.0, 0.0, 0.0]);
    let c2 = make_xyz_curve([10.0, 0.0, 0.0], [20.0, 0.0, 0.0]);
    let c3 = make_xyz_curve([20.0, 0.0, 0.0], [30.0, 0.0, 0.0]);

    let segments = vec![
        make_xy_segment(&c1, EMode::CoupledToXy, 0.04),
        make_xy_segment(&c2, EMode::CoupledToXy, 0.04),
        make_xy_segment(&c3, EMode::Travel, 0.0),
    ];
    let e_limits = default_e_limits();
    let result = partition_batch(&segments, &e_limits);

    assert_eq!(result.runs.len(), 1);
    assert_eq!(result.runs[0].segment_range, 0..3);
    assert_eq!(result.e_gaps.len(), 0);
}

#[test]
fn partition_with_retraction() {
    // [CoupledToXy, Independent, CoupledToXy] -> 2 runs, 1 gap.
    let c1 = make_xyz_curve([0.0, 0.0, 0.0], [10.0, 5.0, 0.0]);
    let c_hold = make_xyz_curve([10.0, 5.0, 0.0], [10.0, 5.0, 0.0]); // stationary
    let c2 = make_xyz_curve([10.0, 5.0, 0.0], [20.0, 5.0, 0.0]);
    let e_retract = make_e_nurbs(10.0, 5.0); // 5mm retraction

    let segments = vec![
        make_xy_segment(&c1, EMode::CoupledToXy, 0.04),
        make_independent_segment(&c_hold, &e_retract),
        make_xy_segment(&c2, EMode::CoupledToXy, 0.04),
    ];
    let e_limits = default_e_limits();
    let result = partition_batch(&segments, &e_limits);

    assert_eq!(result.runs.len(), 2);
    assert_eq!(result.runs[0].segment_range, 0..1);
    assert_eq!(result.runs[1].segment_range, 2..3);
    assert_eq!(result.e_gaps.len(), 1);
    assert_eq!(result.e_gaps[0].segment_index, 1);
    assert!(result.e_gaps[0].duration > 0.0);
    // XYZ position should be the endpoint of segment 0: [10, 5, 0].
    let pos = result.e_gaps[0].xyz_position;
    assert!((pos[0] - 10.0).abs() < 1e-10);
    assert!((pos[1] - 5.0).abs() < 1e-10);
    assert!((pos[2] - 0.0).abs() < 1e-10);
}

#[test]
fn partition_leading_e() {
    // [Independent, CoupledToXy] -> 1 run, 1 gap (leading E before first run).
    let c_hold = make_xyz_curve([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
    let c1 = make_xyz_curve([0.0, 0.0, 0.0], [10.0, 0.0, 0.0]);
    let e_prime = make_e_nurbs(5.0, 10.0); // 5mm prime

    let segments = vec![
        make_independent_segment(&c_hold, &e_prime),
        make_xy_segment(&c1, EMode::CoupledToXy, 0.04),
    ];
    let e_limits = default_e_limits();
    let result = partition_batch(&segments, &e_limits);

    assert_eq!(result.runs.len(), 1);
    assert_eq!(result.runs[0].segment_range, 1..2);
    assert_eq!(result.e_gaps.len(), 1);
    assert_eq!(result.e_gaps[0].segment_index, 0);
    assert!(result.e_gaps[0].duration > 0.0);
    // No preceding segment — xyz_position should be [0, 0, 0].
    #[allow(clippy::float_cmp)]
    {
        assert_eq!(result.e_gaps[0].xyz_position, [0.0, 0.0, 0.0]);
    }
}

#[test]
fn partition_trailing_e() {
    // [CoupledToXy, Independent] -> 1 run, 1 gap (trailing E after last run).
    let c1 = make_xyz_curve([0.0, 0.0, 0.0], [10.0, 0.0, 0.0]);
    let c_hold = make_xyz_curve([10.0, 0.0, 0.0], [10.0, 0.0, 0.0]);
    let e_retract = make_e_nurbs(10.0, 5.0);

    let segments = vec![
        make_xy_segment(&c1, EMode::CoupledToXy, 0.04),
        make_independent_segment(&c_hold, &e_retract),
    ];
    let e_limits = default_e_limits();
    let result = partition_batch(&segments, &e_limits);

    assert_eq!(result.runs.len(), 1);
    assert_eq!(result.runs[0].segment_range, 0..1);
    assert_eq!(result.e_gaps.len(), 1);
    assert_eq!(result.e_gaps[0].segment_index, 1);
    let pos = result.e_gaps[0].xyz_position;
    assert!((pos[0] - 10.0).abs() < 1e-10);
}

#[test]
fn partition_consecutive_e_gaps() {
    // [CoupledToXy, Independent, Independent, CoupledToXy]
    // -> 2 runs, 2 gaps (retraction then prime).
    let c1 = make_xyz_curve([0.0, 0.0, 0.0], [10.0, 0.0, 0.0]);
    let c_hold1 = make_xyz_curve([10.0, 0.0, 0.0], [10.0, 0.0, 0.0]);
    let c_hold2 = make_xyz_curve([10.0, 0.0, 0.0], [10.0, 0.0, 0.0]);
    let c2 = make_xyz_curve([10.0, 0.0, 0.0], [20.0, 0.0, 0.0]);
    let e_retract = make_e_nurbs(10.0, 5.0);
    let e_prime = make_e_nurbs(5.0, 10.0);

    let segments = vec![
        make_xy_segment(&c1, EMode::CoupledToXy, 0.04),
        make_independent_segment(&c_hold1, &e_retract),
        make_independent_segment(&c_hold2, &e_prime),
        make_xy_segment(&c2, EMode::CoupledToXy, 0.04),
    ];
    let e_limits = default_e_limits();
    let result = partition_batch(&segments, &e_limits);

    assert_eq!(result.runs.len(), 2);
    assert_eq!(result.runs[0].segment_range, 0..1);
    assert_eq!(result.runs[1].segment_range, 3..4);
    assert_eq!(result.e_gaps.len(), 2);
    assert_eq!(result.e_gaps[0].segment_index, 1);
    assert_eq!(result.e_gaps[1].segment_index, 2);
}

#[test]
fn partition_empty_input() {
    let result = partition_batch(&[], &default_e_limits());
    assert!(result.runs.is_empty());
    assert!(result.e_gaps.is_empty());
}

#[test]
fn partition_all_independent() {
    // All E segments — no runs.
    let c_hold = make_xyz_curve([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
    let e1 = make_e_nurbs(10.0, 5.0);
    let e2 = make_e_nurbs(5.0, 10.0);

    let segments = vec![
        make_independent_segment(&c_hold, &e1),
        make_independent_segment(&c_hold, &e2),
    ];
    let e_limits = default_e_limits();
    let result = partition_batch(&segments, &e_limits);

    assert!(result.runs.is_empty());
    assert_eq!(result.e_gaps.len(), 2);
}

#[test]
fn e_gap_duration_matches_schedule() {
    // Verify the duration in the E gap matches what schedule_e_duration returns.
    let c1 = make_xyz_curve([0.0, 0.0, 0.0], [10.0, 0.0, 0.0]);
    let c_hold = make_xyz_curve([10.0, 0.0, 0.0], [10.0, 0.0, 0.0]);
    let e_retract = make_e_nurbs(10.0, 5.0);
    let e_limits = default_e_limits();

    let expected_dur = crate::e_independent::schedule_e_duration(&e_retract, 50.0, &e_limits);

    let segments = vec![
        make_xy_segment(&c1, EMode::CoupledToXy, 0.04),
        make_independent_segment(&c_hold, &e_retract),
    ];
    let result = partition_batch(&segments, &e_limits);

    assert!(
        (result.e_gaps[0].duration - expected_dur).abs() < 1e-15,
        "expected {expected_dur}, got {}",
        result.e_gaps[0].duration
    );
}
