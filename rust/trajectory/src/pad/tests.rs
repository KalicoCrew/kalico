use super::*;
use nurbs::bezier::BezierPiece;

/// Build a simple `FittedSegment` with linear motion on axis 0 (X),
/// constant on axes 1 and 2.
fn linear_segment(x_start: f64, x_end: f64, t_start: f64, t_end: f64) -> FittedSegment {
    let dt = t_end - t_start;
    let slope = (x_end - x_start) / dt;
    // X axis: linear in Pascal-shifted basis.
    let x_nurbs = bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: t_start,
        u_end: t_end,
        coeffs: vec![x_start, slope],
    }]);
    // Y and Z: constant at 0.
    let y_nurbs = bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: t_start,
        u_end: t_end,
        coeffs: vec![0.0],
    }]);
    let z_nurbs = bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: t_start,
        u_end: t_end,
        coeffs: vec![0.0],
    }]);
    FittedSegment {
        axes: [x_nurbs, y_nurbs, z_nurbs],
        t_start,
        t_end,
    }
}

#[test]
fn pad_single_segment_extends_with_constants() {
    // Single segment from t=0 to t=1, X goes from 0 to 10.
    let fitted = vec![linear_segment(0.0, 10.0, 0.0, 1.0)];
    let t_sm_half = 0.1;

    let padded = pad_segment_axis(0, 0, &fitted, &[], t_sm_half, 0.0, 1.0);
    let pieces = extract_bezier_pieces(&padded);

    // Should have padding on both sides.
    assert!(
        pieces.len() >= 3,
        "expected at least 3 pieces, got {}",
        pieces.len()
    );

    // First piece should start before t=0.
    assert!(
        pieces[0].u_start < 0.0,
        "first piece should start before 0, starts at {}",
        pieces[0].u_start
    );

    // Last piece should end after t=1.
    assert!(
        pieces.last().unwrap().u_end > 1.0,
        "last piece should end after 1, ends at {}",
        pieces.last().unwrap().u_end
    );

    // Value at t=0 on the left pad should be the start value (0.0).
    let left_val = pieces[0].evaluate(pieces[0].u_start);
    assert!(
        left_val.abs() < 1e-10,
        "left pad should hold 0.0, got {left_val}"
    );

    // Value at the right pad should be the end value (10.0).
    let right_val = pieces
        .last()
        .unwrap()
        .evaluate(pieces.last().unwrap().u_end);
    assert!(
        (right_val - 10.0).abs() < 1e-10,
        "right pad should hold 10.0, got {right_val}"
    );
}

#[test]
fn pad_middle_segment_uses_neighbors() {
    // Three segments:
    // seg 0: t=[0, 1], X=[0, 10]
    // seg 1: t=[1, 2], X=[10, 30]
    // seg 2: t=[2, 3], X=[30, 35]
    let fitted = vec![
        linear_segment(0.0, 10.0, 0.0, 1.0),
        linear_segment(10.0, 30.0, 1.0, 2.0),
        linear_segment(30.0, 35.0, 2.0, 3.0),
    ];
    let t_sm_half = 0.3;

    let padded = pad_segment_axis(1, 0, &fitted, &[], t_sm_half, 0.0, 3.0);
    let pieces = extract_bezier_pieces(&padded);

    // Padded curve should cover [1.0 - 0.3, 2.0 + 0.3] = [0.7, 2.3].
    let first = &pieces[0];
    let last = pieces.last().unwrap();
    assert!(
        (first.u_start - 0.7).abs() < 1e-10,
        "expected start ~0.7, got {}",
        first.u_start
    );
    assert!(
        (last.u_end - 2.3).abs() < 1e-10,
        "expected end ~2.3, got {}",
        last.u_end
    );

    // Value at t=0.7 should come from seg 0: x = 0 + 10*(0.7) = 7.0.
    assert!(
        (first.evaluate(0.7) - 7.0).abs() < 1e-6,
        "expected 7.0 at t=0.7, got {}",
        first.evaluate(0.7)
    );
}

#[test]
fn pad_with_e_halo_gap() {
    // Two segments with an E-gap halo between them.
    // seg 0: t=[0, 1], X=[0, 10]
    // E-gap: t=[1, 1.5], xyz_position=[10, 0, 0]
    // seg 1: t=[1.5, 2.5], X=[10, 20]
    let fitted = vec![
        linear_segment(0.0, 10.0, 0.0, 1.0),
        linear_segment(10.0, 20.0, 1.5, 2.5),
    ];
    let e_halos = vec![EHalo {
        xyz_position: [10.0, 0.0, 0.0],
        t_start: 1.0,
        t_end: 1.5,
    }];
    let t_sm_half = 0.3;

    // Pad segment 1 — should pick up the E-gap halo.
    let padded = pad_segment_axis(1, 0, &fitted, &e_halos, t_sm_half, 0.0, 2.5);
    let pieces = extract_bezier_pieces(&padded);

    // Should start at 1.5 - 0.3 = 1.2, which is inside the E-gap.
    let first = &pieces[0];
    assert!(
        (first.u_start - 1.2).abs() < 1e-10,
        "expected start ~1.2, got {}",
        first.u_start
    );

    // Value at t=1.2 should be the halo value (10.0).
    assert!(
        (first.evaluate(1.2) - 10.0).abs() < 1e-6,
        "expected 10.0 at t=1.2 (halo), got {}",
        first.evaluate(1.2)
    );
}

#[test]
fn padded_pieces_are_contiguous() {
    let fitted = vec![
        linear_segment(0.0, 5.0, 0.0, 0.5),
        linear_segment(5.0, 15.0, 0.5, 1.5),
        linear_segment(15.0, 18.0, 1.5, 2.0),
    ];
    let t_sm_half = 0.2;

    for seg_idx in 0..fitted.len() {
        let padded = pad_segment_axis(seg_idx, 0, &fitted, &[], t_sm_half, 0.0, 2.0);
        let pieces = extract_bezier_pieces(&padded);
        for w in pieces.windows(2) {
            assert!(
                (w[0].u_end - w[1].u_start).abs() < 1e-12,
                "non-contiguous pieces in segment {seg_idx}: {} vs {}",
                w[0].u_end,
                w[1].u_start
            );
        }
    }
}
