use geometry::{CubicSegment, EMode, GeometryError, SourceRange};
use nurbs::VectorNurbs;

fn valid_cubic_xyz() -> VectorNurbs<f64, 3> {
    VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [3.0, 0.0, 0.0],
        ],
        None,
    )
    .expect("valid cubic")
}

fn dummy_source() -> SourceRange {
    SourceRange {
        start_line: 1,
        end_line: 1,
    }
}

#[test]
fn try_new_rejects_non_cubic() {
    let linear = VectorNurbs::<f64, 3>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
        None,
    )
    .expect("valid linear");
    let result = CubicSegment::try_new(
        linear,
        EMode::Travel,
        0.0,
        None,
        100.0,
        dummy_source(),
        None,
    );
    assert!(matches!(
        result,
        Err(GeometryError::NotSinglePieceCubic { .. })
    ));
}

#[test]
fn try_new_rejects_weighted() {
    let weighted = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [3.0, 0.0, 0.0],
        ],
        Some(vec![1.0, 0.5, 0.5, 1.0]),
    )
    .expect("valid weighted cubic");
    let result = CubicSegment::try_new(
        weighted,
        EMode::Travel,
        0.0,
        None,
        100.0,
        dummy_source(),
        None,
    );
    assert!(matches!(
        result,
        Err(GeometryError::NotSinglePieceCubic { .. })
    ));
}

#[test]
fn try_new_accepts_valid_travel() {
    let result = CubicSegment::try_new(
        valid_cubic_xyz(),
        EMode::Travel,
        0.0,
        None,
        100.0,
        dummy_source(),
        None,
    );
    assert!(result.is_ok());
}

#[test]
fn try_new_accepts_coupled_signed_ratio() {
    // Negative ratio = retract-during-XY-motion / wipe / coast.
    let result = CubicSegment::try_new(
        valid_cubic_xyz(),
        EMode::CoupledToXy,
        -0.05,
        None,
        100.0,
        dummy_source(),
        None,
    );
    assert!(result.is_ok());
}

#[test]
fn try_new_rejects_travel_with_nonzero_ratio() {
    let result = CubicSegment::try_new(
        valid_cubic_xyz(),
        EMode::Travel,
        0.05,
        None,
        100.0,
        dummy_source(),
        None,
    );
    assert!(matches!(
        result,
        Err(GeometryError::EModeInvariantViolation { .. })
    ));
}

#[test]
fn try_new_rejects_independent_without_e_curve() {
    let result = CubicSegment::try_new(
        valid_cubic_xyz(),
        EMode::Independent,
        0.0,
        None,
        100.0,
        dummy_source(),
        None,
    );
    assert!(matches!(
        result,
        Err(GeometryError::EModeInvariantViolation { .. })
    ));
}
