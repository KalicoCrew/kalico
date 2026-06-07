use nurbs::algebra::{PiecewisePolynomialKernel, convolve, restrict_to_domain};
use nurbs::bezier::{BezierPiece, bezier_pieces_to_nurbs};
use nurbs::eval::eval;

#[test]
fn convolve_then_restrict_preserves_endpoint_on_offset_second_segment() {
    let f_hz = 50.0_f64;
    let t_sm = 0.95625 / f_hz;
    let h = t_sm / 2.0;
    let c = 15.0 / (16.0 * h.powi(5));
    let abs_coeffs = vec![c * h.powi(4), 0.0, -2.0 * c * h * h, 0.0, c];
    let kernel = PiecewisePolynomialKernel::single_poly_from_absolute(abs_coeffs, (-h, h));

    let t_start = 0.728_351_736_901_697_1_f64;
    let t_end = 2.069_205_513_403_982_6_f64;
    let n_ramp_pieces = 18;
    let mut pieces: Vec<BezierPiece<f64>> = Vec::new();
    pieces.push(BezierPiece::<f64> {
        u_start: t_start - h,
        u_end: t_start,
        coeffs: vec![50.0, 0.0, 0.0, 0.0, 0.0],
    });
    let total = t_end - t_start;
    let slope = 50.0 / total;
    let modulation_amp = 1.0e-3;
    let modulation_freq = 30.0;
    for i in 0..n_ramp_pieces {
        let u0 = t_start + total * f64::from(i) / f64::from(n_ramp_pieces);
        let u1 = t_start + total * f64::from(i + 1) / f64::from(n_ramp_pieces);
        let u_lin = u0;
        let omega = 2.0 * std::f64::consts::PI * modulation_freq;
        let s = (omega * u_lin).sin();
        let cs = (omega * u_lin).cos();
        let c0 = 50.0 + slope * (u0 - t_start) + modulation_amp * s;
        let c1 = slope + modulation_amp * omega * cs;
        let c2 = -modulation_amp * omega.powi(2) / 2.0 * s;
        let c3 = -modulation_amp * omega.powi(3) / 6.0 * cs;
        let c4 = modulation_amp * omega.powi(4) / 24.0 * s;
        pieces.push(BezierPiece::<f64> {
            u_start: u0,
            u_end: u1,
            coeffs: vec![c0, c1, c2, c3, c4],
        });
    }
    pieces.push(BezierPiece::<f64> {
        u_start: t_end,
        u_end: t_end + h,
        coeffs: vec![100.0, 0.0, 0.0, 0.0, 0.0],
    });

    let input = bezier_pieces_to_nurbs(&pieces);
    let convolved = convolve(&input, &kernel).unwrap();
    let restricted = restrict_to_domain(&convolved, t_start, t_end).unwrap();

    let v_end = eval(&restricted.as_view(), t_end);
    let v_just_below = eval(&restricted.as_view(), t_end - 1e-9);
    assert!(
        (v_end - v_just_below).abs() < 1e-6,
        "discontinuity at final knot: eval({t_end})={v_end}, \
         eval({t_end}−1e-9)={v_just_below}, diff={}",
        v_end - v_just_below,
    );

    let last_cp = *restricted.control_points().last().unwrap();
    assert!(
        (last_cp - v_end).abs() < 1e-6,
        "last control point ({last_cp}) does not match polynomial value at \
         t_end ({v_end}) — degenerate trailing piece, diff={}",
        last_cp - v_end,
    );

    assert!(
        (v_end - 100.0).abs() < 1.0,
        "endpoint {v_end} more than 1 mm off expected ~100; pre-fix was ~114",
    );
}
