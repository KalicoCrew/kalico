use super::*;
use crate::fit::FittedSegment;
use crate::kernel::build_smooth_zv_kernel;
use crate::pad::pad_segment_axis;
use nurbs::algebra::convolve;
use nurbs::bezier::{bezier_pieces_to_nurbs, extract_bezier_pieces, BezierPiece};

fn constant_segment(x: f64, y: f64, z: f64, t_start: f64, t_end: f64) -> FittedSegment {
    let make_axis = |val: f64| {
        bezier_pieces_to_nurbs(&[BezierPiece {
            u_start: t_start,
            u_end: t_end,
            coeffs: vec![val],
        }])
    };
    FittedSegment {
        axes: [make_axis(x), make_axis(y), make_axis(z)],
        t_start,
        t_end,
    }
}

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
fn shape_constant_is_constant() {
    let freq = 150.0;
    let t_sm = 0.8025 / freq;
    let t_sm_half = t_sm / 2.0;
    let kernel = build_smooth_zv_kernel(t_sm);

    let x_val = 42.0;
    let fitted = vec![constant_segment(x_val, 0.0, 0.0, 0.0, 1.0)];

    let padded = pad_segment_axis(0, 0, &fitted, &[], t_sm_half, 0.0, 1.0);
    let shaped = shape_axis(&padded, &kernel, 0.0, 1.0);

    let pieces = extract_bezier_pieces(&shaped);
    for &t in &[0.0, 0.25, 0.5, 0.75, 1.0] {
        let val = eval_at(&pieces, t);
        assert!(
            (val - x_val).abs() < 1e-4,
            "at t={t}: expected {x_val}, got {val}"
        );
    }
}

#[test]
fn pad_trim_matches_global_convolve() {
    let freq = 10.0;
    let t_sm = 0.8025 / freq;
    let t_sm_half = t_sm / 2.0;
    let kernel = build_smooth_zv_kernel(t_sm);

    let fitted = vec![
        linear_segment(0.0, 10.0, 0.0, 1.0),
        linear_segment(10.0, 30.0, 1.0, 2.0),
        linear_segment(30.0, 35.0, 2.0, 3.0),
    ];
    let batch_t_start = 0.0;
    let batch_t_end = 3.0;

    let mut shaped_per_seg: Vec<ScalarNurbs<f64>> = Vec::new();
    for seg_idx in 0..3 {
        let padded = pad_segment_axis(
            seg_idx,
            0,
            &fitted,
            &[],
            t_sm_half,
            batch_t_start,
            batch_t_end,
        );
        let shaped = shape_axis(
            &padded,
            &kernel,
            fitted[seg_idx].t_start,
            fitted[seg_idx].t_end,
        );
        shaped_per_seg.push(shaped);
    }

    let mut global_pieces: Vec<BezierPiece<f64>> = Vec::new();

    global_pieces.push(BezierPiece {
        u_start: -t_sm_half,
        u_end: 0.0,
        coeffs: vec![0.0, 0.0],
    });

    for seg in &fitted {
        global_pieces.extend(extract_bezier_pieces(&seg.axes[0]));
    }

    global_pieces.push(BezierPiece {
        u_start: 3.0,
        u_end: 3.0 + t_sm_half,
        coeffs: vec![35.0, 0.0],
    });

    let global_nurbs = bezier_pieces_to_nurbs(&global_pieces);
    let global_convolved = convolve(&global_nurbs, &kernel).unwrap();

    for seg_idx in 0..3 {
        let seg = &fitted[seg_idx];
        let per_seg_pieces = extract_bezier_pieces(&shaped_per_seg[seg_idx]);
        let global_pieces = extract_bezier_pieces(&global_convolved);

        let n_samples = 10;
        for i in 1..n_samples {
            let t = seg.t_start + (seg.t_end - seg.t_start) * (f64::from(i) / f64::from(n_samples));
            let val_per_seg = eval_at(&per_seg_pieces, t);
            let val_global = eval_at(&global_pieces, t);
            assert!(
                (val_per_seg - val_global).abs() < 1e-4,
                "seg {seg_idx}, t={t}: per_seg={val_per_seg}, global={val_global}, diff={}",
                (val_per_seg - val_global).abs()
            );
        }
    }
}

#[test]
fn batch_edge_constant_extension() {
    let freq = 150.0;
    let t_sm = 0.8025 / freq;
    let t_sm_half = t_sm / 2.0;
    let kernel = build_smooth_zv_kernel(t_sm);

    let x_start = 5.0;
    let x_end = 15.0;
    let fitted = vec![linear_segment(x_start, x_end, 0.0, 1.0)];

    let padded = pad_segment_axis(0, 0, &fitted, &[], t_sm_half, 0.0, 1.0);
    let pieces = extract_bezier_pieces(&padded);

    assert!(pieces[0].u_start < 0.0, "padding should extend before t=0");
    assert!(
        pieces.last().unwrap().u_end > 1.0,
        "padding should extend past t=1"
    );

    let shaped = shape_axis(&padded, &kernel, 0.0, 1.0);
    let shaped_pieces = extract_bezier_pieces(&shaped);

    let val_at_0 = eval_at(&shaped_pieces, 0.0);
    assert!(
        (val_at_0 - x_start).abs() < 0.5,
        "at t=0: expected ~{x_start}, got {val_at_0}"
    );

    let val_at_1 = eval_at(&shaped_pieces, 1.0);
    assert!(
        (val_at_1 - x_end).abs() < 0.5,
        "at t=1: expected ~{x_end}, got {val_at_1}"
    );

    let n_samples = 50;
    let mut prev = f64::NEG_INFINITY;
    for i in 0..=n_samples {
        let t = f64::from(i) / f64::from(n_samples);
        let val = eval_at(&shaped_pieces, t);
        assert!(
            val >= prev - 1e-10,
            "not monotone at t={t}: prev={prev}, val={val}"
        );
        prev = val;
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
fn shaped_signal_eval_matches_convolve_output_samples() {
    use crate::kernel::build_smooth_zv_kernel;
    use nurbs::bezier::{bezier_pieces_to_nurbs, BezierPiece};
    use nurbs::eval::eval;

    // Smooth input s(t) on [0, 0.5].
    let t_end = 0.5_f64;
    let s = bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: 0.0,
        u_end: t_end,
        coeffs: vec![0.0, 0.0, 300.0 / t_end.powi(2), -200.0 / t_end.powi(3)],
    }]);
    let kernel = build_smooth_zv_kernel(0.8025 / 40.0);

    let linear = shape_axis(&s, &kernel, 0.0, t_end);
    let sig = ShapedSignal::new(&s, &kernel, 0.0, t_end);

    // The dense-linear output samples lie on the smooth convolution: ShapedSignal
    // must agree with the linear curve at its own knots (where linear == fir_at).
    for &u in linear.knots().iter() {
        if u >= 0.0 && u <= t_end {
            assert!(
                (sig.eval(u) - eval(&linear.as_view(), u)).abs() < 1e-9,
                "mismatch at u={u}",
            );
        }
    }
}
