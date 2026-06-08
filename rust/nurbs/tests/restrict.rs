use nurbs::algebra::restrict_to_domain;
use nurbs::bezier::{BezierPiece, bezier_pieces_to_nurbs, extract_bezier_pieces};

#[test]
fn restrict_single_piece() {
    let piece = BezierPiece::<f64> {
        u_start: 0.0,
        u_end: 4.0,
        coeffs: vec![1.0, 2.0, 3.0],
    };
    let curve = bezier_pieces_to_nurbs(&[piece]);
    let restricted = restrict_to_domain(&curve, 1.0, 3.0).unwrap();
    let pieces = extract_bezier_pieces(&restricted);
    for &u in &[1.0, 1.5, 2.0, 2.5, 3.0] {
        let original: f64 = 1.0 + 2.0 * u + 3.0 * u * u;
        let val = pieces
            .iter()
            .find(|p| p.u_start <= u + 1e-12 && u <= p.u_end + 1e-12)
            .map(|p| p.evaluate(u))
            .unwrap();
        assert!(
            (original - val).abs() < 1e-10,
            "at u={u}: expected {original}, got {val}"
        );
    }
}

#[test]
fn restrict_multi_piece() {
    let p1 = BezierPiece::<f64> {
        u_start: 0.0,
        u_end: 2.0,
        coeffs: vec![1.0, 1.0],
    };
    let p2 = BezierPiece::<f64> {
        u_start: 2.0,
        u_end: 4.0,
        coeffs: vec![3.0, -1.0],
    };
    let curve = bezier_pieces_to_nurbs(&[p1, p2]);
    let restricted = restrict_to_domain(&curve, 1.0, 3.0).unwrap();
    let pieces = extract_bezier_pieces(&restricted);
    assert_eq!(pieces.len(), 2);
    assert!((pieces[0].u_start - 1.0_f64).abs() < 1e-12);
    assert!((pieces[0].u_end - 2.0_f64).abs() < 1e-12);
    assert!((pieces[1].u_start - 2.0_f64).abs() < 1e-12);
    assert!((pieces[1].u_end - 3.0_f64).abs() < 1e-12);
}

#[test]
fn restrict_invalid_domain() {
    let piece = BezierPiece::<f64> {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![1.0],
    };
    let curve = bezier_pieces_to_nurbs(&[piece]);
    assert!(restrict_to_domain(&curve, 1.0, 0.5).is_err());
}

#[test]
fn restrict_exact_domain_is_identity() {
    let piece = BezierPiece::<f64> {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![1.0, 2.0, 3.0],
    };
    let curve = bezier_pieces_to_nurbs(&[piece]);
    let restricted = restrict_to_domain(&curve, 0.0, 1.0).unwrap();
    let pieces = extract_bezier_pieces(&restricted);
    assert_eq!(pieces.len(), 1);
    for &u in &[0.0_f64, 0.25, 0.5, 0.75, 1.0] {
        let orig: f64 = 1.0 + 2.0 * u + 3.0 * u * u;
        assert!((pieces[0].evaluate(u) - orig).abs() < 1e-12);
    }
}
