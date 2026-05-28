#![allow(
    clippy::ref_as_ptr,
    clippy::float_cmp,
    clippy::cast_sign_loss,
    clippy::cast_lossless,
    clippy::too_many_lines,
    clippy::uninlined_format_args,
    clippy::doc_markdown
)]

use runtime::cubic_curve::{CubicLoadError, LoadedCubicCurve, WirePiece, populate_from_wire};

fn make_wire(bp: [f32; 4], dur: f32) -> WirePiece {
    WirePiece {
        bp0_bits: bp[0].to_bits(),
        bp1_bits: bp[1].to_bits(),
        bp2_bits: bp[2].to_bits(),
        bp3_bits: bp[3].to_bits(),
        duration_bits: dur.to_bits(),
    }
}

#[test]
fn single_piece_linear_load() {
    let mut curve = LoadedCubicCurve::empty();
    let wire = [make_wire([0.0, 10.0 / 3.0, 20.0 / 3.0, 10.0], 25e-6)];
    assert_eq!(populate_from_wire(&mut curve, &wire), Ok(()));
    assert_eq!(curve.piece_count, 1);
    // Seconds-domain c1 = (10mm) / (25e-6 s) = 4e5 mm/s.
    assert!((curve.pieces[0].coeffs[1] - 4e5).abs() < 1.0);
    assert!((curve.pieces[0].duration - 25e-6).abs() < 1e-12);
}

#[test]
fn rejects_zero_pieces() {
    let mut curve = LoadedCubicCurve::empty();
    let wire: [WirePiece; 0] = [];
    assert_eq!(
        populate_from_wire(&mut curve, &wire),
        Err(CubicLoadError::PieceCountOutOfRange)
    );
    // No mutation on rejection.
    assert_eq!(curve.piece_count, 0);
}

#[test]
fn rejects_too_many_pieces() {
    let mut curve = LoadedCubicCurve::empty();
    let one = make_wire([0.0, 0.333, 0.667, 1.0], 1e-3);
    let wire = vec![one; runtime::cubic_curve::MAX_PIECES_PER_CURVE + 1];
    assert_eq!(
        populate_from_wire(&mut curve, &wire),
        Err(CubicLoadError::PieceCountOutOfRange)
    );
    assert_eq!(curve.piece_count, 0);
}

#[test]
fn rejects_non_finite_bernstein() {
    let mut curve = LoadedCubicCurve::empty();
    let wire = [make_wire([0.0, f32::NAN, 0.667, 1.0], 1e-3)];
    assert_eq!(
        populate_from_wire(&mut curve, &wire),
        Err(CubicLoadError::NonFiniteBernstein)
    );
    assert_eq!(curve.piece_count, 0);
}

#[test]
fn rejects_zero_duration() {
    let mut curve = LoadedCubicCurve::empty();
    let wire = [make_wire([0.0, 0.333, 0.667, 1.0], 0.0)];
    assert_eq!(
        populate_from_wire(&mut curve, &wire),
        Err(CubicLoadError::NonPositiveDuration)
    );
    assert_eq!(curve.piece_count, 0);
}

#[test]
fn rejects_negative_duration() {
    let mut curve = LoadedCubicCurve::empty();
    let wire = [make_wire([0.0, 0.333, 0.667, 1.0], -1e-6)];
    assert_eq!(
        populate_from_wire(&mut curve, &wire),
        Err(CubicLoadError::NonPositiveDuration)
    );
}

#[test]
fn multi_piece_load_all_fifteen() {
    let mut curve = LoadedCubicCurve::empty();
    let one = make_wire([0.0, 0.333, 0.667, 1.0], 1e-3);
    let wire = vec![one; 15];
    assert_eq!(populate_from_wire(&mut curve, &wire), Ok(()));
    assert_eq!(curve.piece_count, 15);
    for i in 0..15 {
        assert!((curve.pieces[i].duration - 1e-3).abs() < 1e-12);
    }
    // Pieces 15-16 (out of count) should still be zero from `empty()`.
    assert_eq!(curve.pieces[15].duration, 0.0);
}
