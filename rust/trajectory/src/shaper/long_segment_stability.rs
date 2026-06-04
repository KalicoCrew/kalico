use crate::fit::FittedSegment;
use crate::kernel::build_smooth_mzv_kernel;
use crate::pad::pad_segment_axis;
use crate::shaper::shape_axis;
use nurbs::bezier::{bezier_pieces_to_nurbs, extract_bezier_pieces, BezierPiece};

fn constant_segment_69s(x_val: f64) -> FittedSegment {
    FittedSegment {
        axes: [
            bezier_pieces_to_nurbs(&[BezierPiece {
                u_start: 0.0,
                u_end: 69.0,
                coeffs: vec![x_val],
            }]),
            bezier_pieces_to_nurbs(&[BezierPiece {
                u_start: 0.0,
                u_end: 69.0,
                coeffs: vec![0.0],
            }]),
            bezier_pieces_to_nurbs(&[BezierPiece {
                u_start: 0.0,
                u_end: 69.0,
                coeffs: vec![0.0],
            }]),
        ],
        t_start: 0.0,
        t_end: 69.0,
    }
}

fn eval_at(pieces: &[BezierPiece<f64>], t: f64) -> f64 {
    for p in pieces {
        if t >= p.u_start - 1e-15 && t <= p.u_end + 1e-15 {
            return p.evaluate(t);
        }
    }
    panic!("t={t} not in any piece");
}

#[test]
fn constant_69s_near_zero_deviation() {
    let freq = 186.0;
    let t_sm = 0.95625 / freq;
    let t_sm_half = t_sm / 2.0;
    let kernel = build_smooth_mzv_kernel(t_sm);

    let x_val = 150.0;
    let fitted = vec![constant_segment_69s(x_val)];
    let padded = pad_segment_axis(0, 0, &fitted, &[], t_sm_half, 0.0, 69.0);

    let shaped = shape_axis(&padded, &kernel, 0.0, 69.0);
    let pieces = extract_bezier_pieces(&shaped);

    let mut max_dev = 0.0_f64;
    for i in 0..=20 {
        let t = 69.0 * (i as f64) / 20.0;
        let val = eval_at(&pieces, t.clamp(0.0, 69.0));
        max_dev = max_dev.max((val - x_val).abs());
    }

    assert!(
        max_dev < 1e-3,
        "max deviation from {x_val} = {max_dev:.6} mm; expected < 1µm"
    );
}

#[test]
fn stable_where_nurbs_convolve_fails() {
    let freq = 186.0;
    let t_sm = 0.95625 / freq;
    let t_sm_half = t_sm / 2.0;
    let kernel = build_smooth_mzv_kernel(t_sm);

    let x_val = 150.0;
    let fitted = vec![constant_segment_69s(x_val)];
    let padded = pad_segment_axis(0, 0, &fitted, &[], t_sm_half, 0.0, 69.0);

    let shaped = shape_axis(&padded, &kernel, 0.0, 69.0);
    let pieces = extract_bezier_pieces(&shaped);

    let mut max_dev = 0.0_f64;
    for i in 0..=50 {
        let t = 69.0 * (i as f64) / 50.0;
        let val = eval_at(&pieces, t.clamp(0.0, 69.0));
        max_dev = max_dev.max((val - x_val).abs());
    }

    assert!(
        max_dev < 1e-3,
        "max dev = {max_dev:.6} mm on 69s constant input"
    );
}
