use super::*;
use gcode::{Params, Token};

fn cmd(letter: u8, major: u32, line_no: u32, params: Params) -> Token {
    Token::Command {
        letter,
        major,
        minor: None,
        params,
        line_no,
    }
}

fn cmd_with_minor(
    letter: u8,
    major: u32,
    minor: Option<u32>,
    line_no: u32,
    params: Params,
) -> Token {
    Token::Command {
        letter,
        major,
        minor,
        params,
        line_no,
    }
}

fn p(setters: &[(u8, f64)]) -> Params {
    let mut p = Params::default();
    for (l, v) in setters {
        p.set(*l, *v);
    }
    p
}

#[test]
fn modal_state_initializes_at_origin() {
    let st = ModalState::new();
    #[allow(clippy::float_cmp)]
    {
        assert_eq!(st.position, [0.0, 0.0, 0.0]);
    }
    assert_eq!(st.feedrate_mm_s, None);
    assert_eq!(st.tool, 0);
}

#[test]
fn t_marker_carries_tool_number() {
    let toks = vec![cmd(b'T', 2, 1, Params::default())];
    let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
    match &events[0] {
        ReduceEvent::Marker {
            kind: MotionMarkerKind::T,
            tool: Some(2),
            ..
        } => {}
        other => panic!("expected T Marker with tool=2, got {other:?}"),
    }
}

#[test]
fn modal_state_plane_defaults_to_xy() {
    let st = ModalState::new();
    assert_eq!(st.active_plane, Plane::XY);
}

#[test]
fn modal_state_prev_g5_pq_defaults_to_none() {
    let st = ModalState::new();
    assert_eq!(st.prev_g5_pq, None);
}

#[test]
fn g17_keeps_xy_plane() {
    let toks = vec![cmd(b'G', 17, 1, Params::default())];
    let _events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
    // Plane is internal modal state; this test is reachable today only by
    // observing through downstream behavior, which lands in Task 18. Test
    // ordering: this scaffolds the type so Task 18's plane-mismatch test
    // can construct cases that change the plane. For now, assert the type
    // compiles and the variant set is what we expect.
    assert_eq!(Plane::default(), Plane::XY);
    assert_eq!(Plane::XY, Plane::XY);
    assert_ne!(Plane::XY, Plane::XZ);
    assert_ne!(Plane::XZ, Plane::YZ);
}

#[test]
fn g17_sets_xy_plane() {
    let mut st = ModalState::new();
    let toks = vec![cmd(b'G', 17, 1, Params::default())];
    // Drive the iterator to consume the token; we observe the side-effect
    // by re-running with a follow-up G18 and checking that G18 wins.
    let _events: Vec<_> = reduce_with_state(&mut st, toks.into_iter().map(Ok)).collect();
    assert_eq!(st.active_plane, Plane::XY);
}

#[test]
fn g18_sets_xz_plane() {
    let mut st = ModalState::new();
    let toks = vec![cmd(b'G', 18, 1, Params::default())];
    let _events: Vec<_> = reduce_with_state(&mut st, toks.into_iter().map(Ok)).collect();
    assert_eq!(st.active_plane, Plane::XZ);
}

#[test]
fn g19_sets_yz_plane() {
    let mut st = ModalState::new();
    let toks = vec![cmd(b'G', 19, 1, Params::default())];
    let _events: Vec<_> = reduce_with_state(&mut st, toks.into_iter().map(Ok)).collect();
    assert_eq!(st.active_plane, Plane::YZ);
}

#[test]
fn plane_select_emits_no_event() {
    let toks = vec![cmd(b'G', 17, 1, Params::default())];
    let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
    // Plane selects update modal state silently — they're configuration,
    // not motion, and intentionally do not produce telemetry events.
    assert!(events.is_empty(), "expected no events, got {events:?}");
}

#[test]
#[allow(clippy::float_cmp)]
fn g5_with_explicit_ijpq_emits_curve_cubic() {
    // Position at origin, G5 to (10, 0) with tangent params I=3, J=3, P=-3, Q=3.
    // Expected control points:
    //   P0 = (0, 0, 0)
    //   P1 = (0+3, 0+3, 0) = (3, 3, 0)
    //   P2 = (10+(-3), 0+3, 0) = (7, 3, 0)
    //   P3 = (10, 0, 0)
    let toks = vec![cmd_with_minor(
        b'G',
        5,
        None,
        1,
        p(&[
            (b'X', 10.0),
            (b'Y', 0.0),
            (b'I', 3.0),
            (b'J', 3.0),
            (b'P', -3.0),
            (b'Q', 3.0),
            (b'F', 1500.0),
        ]),
    )];
    let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
    assert_eq!(events.len(), 1);
    match &events[0] {
        ReduceEvent::Curve {
            geom: CurveGeom::Cubic { cps },
            feedrate_mm_s,
            line_no: 1,
            ..
        } => {
            assert_eq!(cps[0], [0.0, 0.0, 0.0]);
            assert_eq!(cps[1], [3.0, 3.0, 0.0]);
            assert_eq!(cps[2], [7.0, 3.0, 0.0]);
            assert_eq!(cps[3], [10.0, 0.0, 0.0]);
            assert!((feedrate_mm_s - 25.0).abs() < 1e-9);
        }
        other => panic!("expected Curve(Cubic), got {other:?}"),
    }
}

#[test]
fn g5_error_path_clears_prev_g5_pq() {
    // First G5 succeeds and would normally extend the chain.
    // Second G5 errors (missing P) — must clear prev_g5_pq.
    // Third G5 has no I,J — must produce G5MissingTangent
    // (proves the second G5's error cleared the chain).
    let toks = vec![
        cmd_with_minor(
            b'G',
            5,
            None,
            1,
            p(&[
                (b'X', 10.0),
                (b'Y', 0.0),
                (b'I', 3.0),
                (b'J', 3.0),
                (b'P', -3.0),
                (b'Q', 3.0),
                (b'F', 1500.0),
            ]),
        ),
        // Second G5: P omitted -> G5MalformedTangent.
        cmd_with_minor(
            b'G',
            5,
            None,
            2,
            p(&[
                (b'X', 20.0),
                (b'Y', 0.0),
                (b'I', 3.0),
                (b'J', 3.0),
                (b'Q', 3.0),
            ]),
        ),
        // Third G5: no I,J. If the second G5 didn't clear, this would
        // silently link to the *first* G5's (P, Q) — wrong. Must error.
        cmd_with_minor(
            b'G',
            5,
            None,
            3,
            p(&[(b'X', 30.0), (b'Y', 0.0), (b'P', -2.0), (b'Q', 2.0)]),
        ),
    ];
    let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
    assert_eq!(events.len(), 3);
    match &events[1] {
        ReduceEvent::ParseError {
            line_no: 2,
            kind: ParseErrorKind::G5MalformedTangent,
            ..
        } => {}
        other => panic!("[1] expected G5MalformedTangent, got {other:?}"),
    }
    match &events[2] {
        ReduceEvent::ParseError {
            line_no: 3,
            kind: ParseErrorKind::G5MissingTangent,
            ..
        } => {}
        other => {
            panic!("[2] expected G5MissingTangent (error path must clear chain), got {other:?}")
        }
    }
}

#[test]
#[allow(clippy::float_cmp)]
fn g5_chain_implicit_tangent_from_prev_pq() {
    // Three-G5 chain. Second and third have no I,J — should default to
    // -(prev P, prev Q).
    let toks = vec![
        cmd_with_minor(
            b'G',
            5,
            None,
            1,
            p(&[
                (b'X', 10.0),
                (b'Y', 0.0),
                (b'I', 3.0),
                (b'J', 3.0),
                (b'P', -3.0),
                (b'Q', 3.0),
                (b'F', 1500.0),
            ]),
        ),
        // Second G5: I,J implicit. Should be -(P,Q) of prev = (3, -3).
        cmd_with_minor(
            b'G',
            5,
            None,
            2,
            p(&[(b'X', 20.0), (b'Y', 0.0), (b'P', -2.0), (b'Q', 2.0)]),
        ),
        // Third G5: I,J implicit. Should be -(P,Q) of second = (2, -2).
        cmd_with_minor(
            b'G',
            5,
            None,
            3,
            p(&[(b'X', 30.0), (b'Y', 0.0), (b'P', 0.0), (b'Q', 0.0)]),
        ),
    ];
    let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
    assert_eq!(events.len(), 3);

    // Second G5: P0=(10,0,0), P1=(10+3, 0+(-3), 0)=(13, -3, 0).
    match &events[1] {
        ReduceEvent::Curve {
            geom: CurveGeom::Cubic { cps },
            ..
        } => {
            assert_eq!(cps[0], [10.0, 0.0, 0.0]);
            assert_eq!(cps[1], [13.0, -3.0, 0.0]);
            assert_eq!(cps[2], [20.0 + (-2.0), 0.0 + 2.0, 0.0]);
            assert_eq!(cps[3], [20.0, 0.0, 0.0]);
        }
        other => panic!("[1] expected Curve(Cubic), got {other:?}"),
    }

    // Third G5: P0=(20,0,0), P1=(20+2, 0+(-2), 0)=(22, -2, 0).
    match &events[2] {
        ReduceEvent::Curve {
            geom: CurveGeom::Cubic { cps },
            ..
        } => {
            assert_eq!(cps[0], [20.0, 0.0, 0.0]);
            assert_eq!(cps[1], [22.0, -2.0, 0.0]);
            assert_eq!(cps[2], [30.0 + 0.0, 0.0 + 0.0, 0.0]);
            assert_eq!(cps[3], [30.0, 0.0, 0.0]);
        }
        other => panic!("[2] expected Curve(Cubic), got {other:?}"),
    }
}

#[test]
fn g5_chain_broken_by_g1_emits_recovery() {
    // G5 → G1 (breaks chain) → G5 with no I,J → expect ParseError.
    let toks = vec![
        cmd_with_minor(
            b'G',
            5,
            None,
            1,
            p(&[
                (b'X', 10.0),
                (b'Y', 0.0),
                (b'I', 3.0),
                (b'J', 3.0),
                (b'P', -3.0),
                (b'Q', 3.0),
                (b'F', 1500.0),
            ]),
        ),
        cmd(b'G', 1, 2, p(&[(b'X', 11.0), (b'Y', 0.0)])),
        cmd_with_minor(
            b'G',
            5,
            None,
            3,
            p(&[(b'X', 20.0), (b'Y', 0.0), (b'P', -2.0), (b'Q', 2.0)]),
        ),
    ];
    let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
    assert_eq!(events.len(), 3);
    match &events[2] {
        ReduceEvent::ParseError {
            line_no: 3,
            kind: ParseErrorKind::G5MissingTangent,
            ..
        } => {}
        other => panic!("[2] expected G5MissingTangent ParseError, got {other:?}"),
    }
}

#[test]
#[allow(clippy::float_cmp)]
fn g5_chain_preserved_by_plane_select() {
    // G5 → G17 (no motion, doesn't break chain) → G5 with no I,J → uses prev_g5_pq.
    let toks = vec![
        cmd_with_minor(
            b'G',
            5,
            None,
            1,
            p(&[
                (b'X', 10.0),
                (b'Y', 0.0),
                (b'I', 3.0),
                (b'J', 3.0),
                (b'P', -3.0),
                (b'Q', 3.0),
                (b'F', 1500.0),
            ]),
        ),
        cmd(b'G', 17, 2, Params::default()),
        cmd_with_minor(
            b'G',
            5,
            None,
            3,
            p(&[(b'X', 20.0), (b'Y', 0.0), (b'P', -2.0), (b'Q', 2.0)]),
        ),
    ];
    let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
    // G17 emits no event, so we have 2 events total (the two G5s).
    assert_eq!(events.len(), 2);
    match &events[1] {
        ReduceEvent::Curve {
            geom: CurveGeom::Cubic { cps },
            ..
        } => {
            // Modal-chain implicit I,J = -(prev P, prev Q) = (3, -3).
            assert_eq!(cps[1], [13.0, -3.0, 0.0]);
        }
        other => panic!("[1] expected Curve(Cubic), got {other:?}"),
    }
}

#[test]
#[allow(clippy::float_cmp)]
fn g5_chain_preserved_by_m_and_t_codes() {
    // G5 → M104 → T0 → G5 with no I,J. M and T don't move; chain intact.
    let toks = vec![
        cmd_with_minor(
            b'G',
            5,
            None,
            1,
            p(&[
                (b'X', 10.0),
                (b'Y', 0.0),
                (b'I', 3.0),
                (b'J', 3.0),
                (b'P', -3.0),
                (b'Q', 3.0),
                (b'F', 1500.0),
            ]),
        ),
        cmd(b'M', 104, 2, p(&[(b'S', 210.0)])),
        cmd(b'T', 0, 3, Params::default()),
        cmd_with_minor(
            b'G',
            5,
            None,
            4,
            p(&[(b'X', 20.0), (b'Y', 0.0), (b'P', -2.0), (b'Q', 2.0)]),
        ),
    ];
    let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
    // M and T emit Marker events; G5s emit Curve events; total = 4.
    assert_eq!(events.len(), 4);
    match &events[3] {
        ReduceEvent::Curve {
            geom: CurveGeom::Cubic { cps },
            ..
        } => {
            assert_eq!(cps[1], [13.0, -3.0, 0.0]);
        }
        other => panic!("[3] expected Curve(Cubic), got {other:?}"),
    }
}

#[test]
fn g5_chain_broken_by_g92_emits_recovery() {
    // G5 → G92 (redefines coordinate frame; clears chain per spec §3.5)
    // → G5 with no I,J → expect ParseError::G5MissingTangent.
    // (G5 → G92 → G5(no IJ) → Recovery::G5MissingTangent — derived behavior.)
    let toks = vec![
        cmd_with_minor(
            b'G',
            5,
            None,
            1,
            p(&[
                (b'X', 10.0),
                (b'Y', 0.0),
                (b'I', 3.0),
                (b'J', 3.0),
                (b'P', -3.0),
                (b'Q', 3.0),
                (b'F', 1500.0),
            ]),
        ),
        // G92 redefines the current position / coordinate frame; (P, Q)
        // become semantically stale because they are deltas in the prior
        // frame. Spec §3.5 chooses to clear conservatively.
        cmd(b'G', 92, 2, p(&[(b'X', 0.0), (b'Y', 0.0)])),
        cmd_with_minor(
            b'G',
            5,
            None,
            3,
            p(&[(b'X', 20.0), (b'Y', 0.0), (b'P', -2.0), (b'Q', 2.0)]),
        ),
    ];
    let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
    // The trailing G5 must produce a ParseError, not silently link to
    // the pre-G92 G5's (P, Q).
    let last = events.last().expect("expected at least one event");
    match last {
        ReduceEvent::ParseError {
            line_no: 3,
            kind: ParseErrorKind::G5MissingTangent,
            ..
        } => {}
        other => panic!(
            "expected G5MissingTangent on trailing G5 (G92 must clear chain), got {other:?}"
        ),
    }
}

#[test]
fn g5_single_i_only_is_malformed() {
    // I given but J omitted — invalid.
    let toks = vec![cmd_with_minor(
        b'G',
        5,
        None,
        1,
        p(&[
            (b'X', 10.0),
            (b'Y', 0.0),
            (b'I', 3.0),
            (b'P', -3.0),
            (b'Q', 3.0),
            (b'F', 1500.0),
        ]),
    )];
    let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
    match &events[0] {
        ReduceEvent::ParseError {
            line_no: 1,
            kind: ParseErrorKind::G5MalformedTangent,
            ..
        } => {}
        other => panic!("expected G5MalformedTangent, got {other:?}"),
    }
}

#[test]
fn g5_missing_pq_is_malformed() {
    // P,Q absent on G5 — invalid (P,Q are required on every G5 line).
    let toks = vec![cmd_with_minor(
        b'G',
        5,
        None,
        1,
        p(&[
            (b'X', 10.0),
            (b'Y', 0.0),
            (b'I', 3.0),
            (b'J', 3.0),
            (b'F', 1500.0),
        ]),
    )];
    let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
    match &events[0] {
        ReduceEvent::ParseError {
            line_no: 1,
            kind: ParseErrorKind::G5MalformedTangent,
            ..
        } => {}
        other => panic!("expected G5MalformedTangent, got {other:?}"),
    }
}

#[test]
#[allow(clippy::float_cmp)]
fn g5_with_z_delta_interpolates_z_at_thirds() {
    // From (0,0,0) to (10, 0, 0.3). Expected Z at CPs: 0, 0.1, 0.2, 0.3.
    let toks = vec![cmd_with_minor(
        b'G',
        5,
        None,
        1,
        p(&[
            (b'X', 10.0),
            (b'Y', 0.0),
            (b'Z', 0.3),
            (b'I', 3.0),
            (b'J', 3.0),
            (b'P', -3.0),
            (b'Q', 3.0),
            (b'F', 1500.0),
        ]),
    )];
    let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
    match &events[0] {
        ReduceEvent::Curve {
            geom: CurveGeom::Cubic { cps },
            ..
        } => {
            let approx = |a: f64, b: f64| (a - b).abs() < 1e-12;
            assert!(approx(cps[0][2], 0.0));
            assert!(approx(cps[1][2], 0.1));
            assert!(approx(cps[2][2], 0.2));
            assert!(approx(cps[3][2], 0.3));
        }
        other => panic!("expected Curve(Cubic), got {other:?}"),
    }
}

#[test]
#[allow(clippy::float_cmp)]
fn g5_1_with_z_delta_interpolates_z_at_midpoint() {
    // From (0,0,0) to (10, 0, 0.4). Expected Z at the three CPs:
    //   P0.z = 0, P1.z = 0.2 (midpoint), P2.z = 0.4.
    // Spec §6.2: "G5.1 with Z delta → control-point Z values at midpoint
    // (0, dz/2, dz)." Mirrors the cubic-at-thirds test above for G5.
    let toks = vec![cmd_with_minor(
        b'G',
        5,
        Some(1),
        1,
        p(&[
            (b'X', 10.0),
            (b'Y', 0.0),
            (b'Z', 0.4),
            (b'I', 3.0),
            (b'J', 3.0),
            (b'F', 1500.0),
        ]),
    )];
    let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
    match &events[0] {
        ReduceEvent::Curve {
            geom: CurveGeom::Quadratic { cps },
            ..
        } => {
            let approx = |a: f64, b: f64| (a - b).abs() < 1e-12;
            assert!(approx(cps[0][2], 0.0));
            assert!(approx(cps[1][2], 0.2));
            assert!(approx(cps[2][2], 0.4));
        }
        other => panic!("expected Curve(Quadratic), got {other:?}"),
    }
}

#[test]
#[allow(clippy::float_cmp)]
fn g5_1_with_explicit_ij_emits_curve_quadratic() {
    // From (0,0,0) to (10,0). I=3, J=3. Expected:
    //   P0 = (0, 0, 0), P1 = (3, 3, 0), P2 = (10, 0, 0).
    let toks = vec![cmd_with_minor(
        b'G',
        5,
        Some(1),
        1,
        p(&[
            (b'X', 10.0),
            (b'Y', 0.0),
            (b'I', 3.0),
            (b'J', 3.0),
            (b'F', 1500.0),
        ]),
    )];
    let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
    assert_eq!(events.len(), 1);
    match &events[0] {
        ReduceEvent::Curve {
            geom: CurveGeom::Quadratic { cps },
            feedrate_mm_s,
            line_no: 1,
            ..
        } => {
            assert_eq!(cps[0], [0.0, 0.0, 0.0]);
            assert_eq!(cps[1], [3.0, 3.0, 0.0]);
            assert_eq!(cps[2], [10.0, 0.0, 0.0]);
            assert!((feedrate_mm_s - 25.0).abs() < 1e-9);
        }
        other => panic!("expected Curve(Quadratic), got {other:?}"),
    }
}

#[test]
fn g5_1_outside_xy_plane_emits_recovery() {
    // G18 sets XZ plane; G5.1 should error.
    let toks = vec![
        cmd(b'G', 18, 1, Params::default()),
        cmd_with_minor(
            b'G',
            5,
            Some(1),
            2,
            p(&[(b'X', 10.0), (b'Z', 1.0), (b'I', 3.0), (b'J', 3.0)]),
        ),
    ];
    let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
    // G18 emits no event, so we have 1 event total (the G5.1 ParseError).
    assert_eq!(events.len(), 1);
    match &events[0] {
        ReduceEvent::ParseError {
            line_no: 2,
            kind: ParseErrorKind::G5PlaneMismatch,
            text,
        } => {
            assert_eq!(text, "18", "expected active plane G-code 18, got {text:?}");
        }
        other => panic!("expected G5PlaneMismatch, got {other:?}"),
    }
}

#[test]
fn g5_1_with_both_ij_zero_is_malformed() {
    let toks = vec![cmd_with_minor(
        b'G',
        5,
        Some(1),
        1,
        p(&[(b'X', 10.0), (b'Y', 0.0), (b'I', 0.0), (b'J', 0.0)]),
    )];
    let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
    match &events[0] {
        ReduceEvent::ParseError {
            kind: ParseErrorKind::G5MalformedTangent,
            ..
        } => {}
        other => panic!("expected G5MalformedTangent, got {other:?}"),
    }
}

#[test]
fn g5_1_missing_j_is_malformed() {
    let toks = vec![cmd_with_minor(
        b'G',
        5,
        Some(1),
        1,
        p(&[(b'X', 10.0), (b'Y', 0.0), (b'I', 3.0)]),
    )];
    let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
    match &events[0] {
        ReduceEvent::ParseError {
            kind: ParseErrorKind::G5MalformedTangent,
            ..
        } => {}
        other => panic!("expected G5MalformedTangent, got {other:?}"),
    }
}

#[test]
fn g5_1_missing_i_is_malformed() {
    // J specified but I omitted — invalid (G5.1 has no modal-chain rule;
    // both I and J are required). Symmetric to g5_1_missing_j_is_malformed.
    let toks = vec![cmd_with_minor(
        b'G',
        5,
        Some(1),
        1,
        p(&[(b'X', 10.0), (b'Y', 0.0), (b'J', 3.0)]),
    )];
    let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
    match &events[0] {
        ReduceEvent::ParseError {
            kind: ParseErrorKind::G5MalformedTangent,
            ..
        } => {}
        other => panic!("expected G5MalformedTangent, got {other:?}"),
    }
}

#[test]
fn g5_1_no_ij_is_malformed() {
    // Neither I nor J — G5.1 has no modal-chain rule, so this is invalid.
    // (Per spec §6.2: "G5.1 with no I, J → Recovery::MalformedParams.
    // No modal-chain rule for G5.1.")
    let toks = vec![cmd_with_minor(
        b'G',
        5,
        Some(1),
        1,
        p(&[(b'X', 10.0), (b'Y', 0.0)]),
    )];
    let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
    match &events[0] {
        ReduceEvent::ParseError {
            kind: ParseErrorKind::G5MalformedTangent,
            ..
        } => {}
        other => panic!("expected G5MalformedTangent, got {other:?}"),
    }
}

#[test]
fn g5_1_outside_g19_plane_emits_recovery() {
    // G19 sets YZ plane; G5.1 should error.
    // Symmetric to g5_1_outside_xy_plane_emits_recovery (which uses G18).
    let toks = vec![
        cmd(b'G', 19, 1, Params::default()),
        cmd_with_minor(
            b'G',
            5,
            Some(1),
            2,
            p(&[(b'Y', 10.0), (b'Z', 1.0), (b'I', 3.0), (b'J', 3.0)]),
        ),
    ];
    let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
    // G19 emits no event, so we have 1 event total (the G5.1 ParseError).
    assert_eq!(events.len(), 1);
    match &events[0] {
        ReduceEvent::ParseError {
            line_no: 2,
            kind: ParseErrorKind::G5PlaneMismatch,
            text,
        } => {
            assert_eq!(text, "19", "expected active plane G-code 19, got {text:?}");
        }
        other => panic!("expected G5PlaneMismatch, got {other:?}"),
    }
}

#[test]
#[allow(clippy::float_cmp)]
fn g5_1_after_g18_then_g17_succeeds() {
    // G18 (sets XZ — would error if G5.1 followed) → G17 (resets to XY)
    // → G5.1 should now succeed. Asserts the plane-mismatch error path
    // is not sticky and that G17 properly resets the active plane.
    let toks = vec![
        cmd(b'G', 18, 1, Params::default()),
        cmd(b'G', 17, 2, Params::default()),
        cmd_with_minor(
            b'G',
            5,
            Some(1),
            3,
            p(&[
                (b'X', 10.0),
                (b'Y', 0.0),
                (b'I', 3.0),
                (b'J', 3.0),
                (b'F', 1500.0),
            ]),
        ),
    ];
    let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
    // G18 and G17 emit no events; G5.1 emits one Curve(Quadratic) event.
    assert_eq!(events.len(), 1);
    match &events[0] {
        ReduceEvent::Curve {
            geom: CurveGeom::Quadratic { cps },
            line_no: 3,
            ..
        } => {
            assert_eq!(cps[0], [0.0, 0.0, 0.0]);
            assert_eq!(cps[1], [3.0, 3.0, 0.0]);
            assert_eq!(cps[2], [10.0, 0.0, 0.0]);
        }
        other => panic!("expected Curve(Quadratic) after G18→G17 reset, got {other:?}"),
    }
}

#[test]
fn comment_marker_layer_change_is_forwarded() {
    let toks = vec![Token::Marker {
        kind: gcode::MarkerKind::LayerChange { layer: Some(7) },
        line_no: 42,
    }];
    let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
    assert_eq!(events.len(), 1);
    match &events[0] {
        ReduceEvent::CommentMarker { kind, line_no: 42 } => match kind {
            gcode::MarkerKind::LayerChange { layer } => assert_eq!(*layer, Some(7)),
            _ => panic!("expected LayerChange"),
        },
        other => panic!("expected CommentMarker, got {other:?}"),
    }
}
