use super::*;
use nurbs::bezier::BezierPiece;

fn linear_segment(x_start: f64, x_end: f64, t_start: f64, t_end: f64) -> FittedSegment {
    let dt = t_end - t_start;
    let slope = (x_end - x_start) / dt;
    let x_nurbs = bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: t_start,
        u_end: t_end,
        coeffs: vec![x_start, slope],
    }]);
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
fn pad_single_segment_extends_left_by_velocity_right_by_constant() {
    let fitted = vec![linear_segment(0.0, 10.0, 0.0, 1.0)];
    let t_sm_half = 0.1;

    let padded = pad_segment_axis(0, 0, &fitted, &[], t_sm_half, 0.0, 1.0);
    let pieces = extract_bezier_pieces(&padded);

    assert!(
        pieces.len() >= 3,
        "expected at least 3 pieces, got {}",
        pieces.len()
    );

    assert!(
        pieces[0].u_start < 0.0,
        "first piece should start before 0, starts at {}",
        pieces[0].u_start
    );

    assert!(
        pieces.last().unwrap().u_end > 1.0,
        "last piece should end after 1, ends at {}",
        pieces.last().unwrap().u_end
    );

    // The left pad is a constant-velocity continuation of the entry motion (slope 10 through
    // position 0 at the t=0 seam), NOT a constant-position hold: holding position injects a
    // phantom velocity step that the shaper convolves into a spurious acceleration spike.
    let left = &pieces[0];
    assert!(
        left.evaluate(0.0).abs() < 1e-9,
        "left pad must meet the segment start at position 0, got {}",
        left.evaluate(0.0),
    );
    let left_slope = left.differentiate().evaluate(left.u_start);
    assert!(
        (left_slope - 10.0).abs() < 1e-6,
        "left pad must continue at the entry velocity 10, got slope {left_slope}",
    );

    // The trailing edge faces an unknown future, so it still holds the end position constant.
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
    let fitted = vec![
        linear_segment(0.0, 10.0, 0.0, 1.0),
        linear_segment(10.0, 30.0, 1.0, 2.0),
        linear_segment(30.0, 35.0, 2.0, 3.0),
    ];
    let t_sm_half = 0.3;

    let padded = pad_segment_axis(1, 0, &fitted, &[], t_sm_half, 0.0, 3.0);
    let pieces = extract_bezier_pieces(&padded);

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

    assert!(
        (first.evaluate(0.7) - 7.0).abs() < 1e-6,
        "expected 7.0 at t=0.7, got {}",
        first.evaluate(0.7)
    );
}

#[test]
fn pad_with_e_halo_gap() {
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

    let padded = pad_segment_axis(1, 0, &fitted, &e_halos, t_sm_half, 0.0, 2.5);
    let pieces = extract_bezier_pieces(&padded);

    let first = &pieces[0];
    assert!(
        (first.u_start - 1.2).abs() < 1e-10,
        "expected start ~1.2, got {}",
        first.u_start
    );

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
