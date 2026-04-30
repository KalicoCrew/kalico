use compat::modal::{ModalState, Plane};

/// Default construction matches power-on state.
#[test]
fn initial_state() {
    let s = ModalState::new();
    assert_eq!(s.position, [0.0, 0.0, 0.0]);
    assert_eq!(s.input_e, 0.0);
    assert_eq!(s.output_e, 0.0);
    assert_eq!(s.feedrate_mm_min, None);
    assert!(s.absolute_xyz, "should default to G90 (absolute XYZ)");
    assert!(s.absolute_e, "should default to M82 (absolute E)");
    assert_eq!(s.active_plane, Plane::XY);
    assert!(s.prev_g5_pq.is_none());
    assert!(s.prev_tangent.is_none());
}

/// G90 mode: explicit XYZ values are taken as absolute coordinates.
#[test]
fn resolve_position_absolute() {
    let mut s = ModalState::new();
    s.position = [10.0, 20.0, 30.0];
    s.absolute_xyz = true;

    let end = s.resolve_position(Some(5.0), Some(15.0), Some(25.0));
    assert_eq!(end, [5.0, 15.0, 25.0]);
}

/// G91 mode: values are added to the current position.
#[test]
fn resolve_position_relative() {
    let mut s = ModalState::new();
    s.position = [10.0, 20.0, 30.0];
    s.absolute_xyz = false;

    let end = s.resolve_position(Some(1.0), Some(-2.0), Some(0.5));
    assert!((end[0] - 11.0).abs() < 1e-12);
    assert!((end[1] - 18.0).abs() < 1e-12);
    assert!((end[2] - 30.5).abs() < 1e-12);
}

/// Absent parameters inherit from the current position in both G90 and G91.
#[test]
fn resolve_position_modal_inherit() {
    let mut s = ModalState::new();
    s.position = [3.0, 7.0, 1.5];

    // G90: absent axes stay at current absolute position.
    s.absolute_xyz = true;
    let end_abs = s.resolve_position(Some(9.0), None, None);
    assert!((end_abs[0] - 9.0).abs() < 1e-12);
    assert!((end_abs[1] - 7.0).abs() < 1e-12);
    assert!((end_abs[2] - 1.5).abs() < 1e-12);

    // G91: absent axes contribute 0 delta → same as current position.
    s.absolute_xyz = false;
    let end_rel = s.resolve_position(None, Some(2.0), None);
    assert!((end_rel[0] - 3.0).abs() < 1e-12);
    assert!((end_rel[1] - 9.0).abs() < 1e-12);
    assert!((end_rel[2] - 1.5).abs() < 1e-12);
}

/// M82 (absolute E): the parameter is taken as-is.
#[test]
fn resolve_e_absolute() {
    let mut s = ModalState::new();
    s.input_e = 5.0;
    s.absolute_e = true;

    let result = s.resolve_input_e(Some(12.0));
    assert_eq!(result, Some(12.0));
}

/// M83 (relative E): the parameter is added to the current input_e accumulator.
#[test]
fn resolve_e_relative() {
    let mut s = ModalState::new();
    s.input_e = 5.0;
    s.absolute_e = false;

    let result = s.resolve_input_e(Some(3.0));
    assert_eq!(result, Some(8.0));
}

/// A missing E word resolves to None regardless of mode.
#[test]
fn resolve_e_absent() {
    let mut s = ModalState::new();
    s.input_e = 5.0;

    s.absolute_e = true;
    assert_eq!(s.resolve_input_e(None), None);

    s.absolute_e = false;
    assert_eq!(s.resolve_input_e(None), None);
}

/// Endpoint displaced in XY from current position → has_xy_motion returns true.
#[test]
fn has_xy_motion_true() {
    let mut s = ModalState::new();
    s.position = [1.0, 2.0, 3.0];

    assert!(s.has_xy_motion(&[1.01, 2.0, 3.0]));
    assert!(s.has_xy_motion(&[1.0, 2.01, 3.0]));
}

/// Endpoint only differs in Z (or is identical) → has_xy_motion returns false.
#[test]
fn has_xy_motion_false() {
    let mut s = ModalState::new();
    s.position = [1.0, 2.0, 3.0];

    // Same XY, different Z.
    assert!(!s.has_xy_motion(&[1.0, 2.0, 10.0]));

    // Exactly the same point.
    assert!(!s.has_xy_motion(&[1.0, 2.0, 3.0]));
}
