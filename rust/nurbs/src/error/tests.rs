use super::*;

#[test]
fn construct_error_converts_to_nurbs_error() {
    let e = ConstructError::DegreeExceeded {
        actual: 25,
        max: 20,
    };
    let n: NurbsError<f32> = e.into();
    assert!(matches!(
        n,
        NurbsError::Construct(ConstructError::DegreeExceeded { .. })
    ));
}

#[test]
fn wire_error_wraps_construct_error() {
    let e = ConstructError::KnotsNotMonotone;
    let w: WireError = e.into();
    assert!(matches!(w, WireError::Construct(_)));
}

#[test]
fn nurbs_error_implements_error_trait() {
    let e: NurbsError<f32> = ConstructError::KnotsNotClamped.into();
    let _: &dyn core::error::Error = &e;
}

#[test]
fn display_renders_messages() {
    let e: NurbsError<f32> = ConstructError::DegreeExceeded {
        actual: 30,
        max: 20,
    }
    .into();
    let s = format!("{e}");
    assert!(s.contains("30"));
    assert!(s.contains("20"));
}

#[test]
fn knot_error_converts_to_nurbs_error() {
    let e = KnotError::BoundaryInsertion;
    let n: NurbsError<f64> = e.into();
    assert!(matches!(n, NurbsError::Knot(KnotError::BoundaryInsertion)));
}

#[test]
fn knot_error_displays_clearly() {
    let e = KnotError::MultiplicityExceeded {
        existing: 2,
        requested: 2,
        max: 3,
    };
    let s = format!("{e}");
    assert!(s.contains("multiplicity"));
    assert!(s.contains('2'));
    assert!(s.contains('3'));
}

#[test]
fn rational_not_supported_displays_with_workaround() {
    let e = AlgebraError::RationalNotSupported {
        operation: "multiply",
        workaround: "use polynomial_refit",
    };
    let s = format!("{e}");
    assert!(s.contains("multiply"));
    assert!(s.contains("polynomial_refit"));
}
