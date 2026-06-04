use geometry::{CubicSegment, EMode, GeometryError, SourceRange};
use nurbs::{ScalarNurbs, VectorNurbs, eval::vector_eval};

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

#[test]
fn live_reduce_rejects_g1() {
    use geometry::{Fatal, FitterParams, GeometryPipeline, Item, TelemetryEvent};

    let mut events: Vec<TelemetryEvent> = vec![];
    let items: Vec<Item> = {
        let mut pipeline = GeometryPipeline::new(FitterParams::default());
        let mut sink = |evt: TelemetryEvent| events.push(evt);
        pipeline.process("G1 X10 Y10 F1000\n", &mut sink).collect()
    };

    assert!(
        items.iter().any(|item| matches!(
            item,
            Item::Fatal(Fatal::UnsupportedGcode {
                gcode_kind: "G0/G1",
                ..
            })
        )),
        "G1 input should produce Item::Fatal(Fatal::UnsupportedGcode {{ gcode_kind: \"G0/G1\" }}); got {items:#?}"
    );
}

#[test]
fn degree_elevation_preserves_curve() {
    use geometry::degree_elevate_2_to_3;

    let q = VectorNurbs::<f64, 3>::try_new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [1.0, 1.0, 0.0], [2.0, 0.0, 0.0]],
    )
    .unwrap();

    let cubic = degree_elevate_2_to_3(&q);

    // Sample 100 points; quadratic and cubic must agree to f64 round-off.
    for i in 0..=100 {
        let u = f64::from(i) / 100.0;
        let q_val = vector_eval(&q, u);
        let c_val = vector_eval(&cubic, u);
        for axis in 0..3 {
            assert!(
                (q_val[axis] - c_val[axis]).abs() < 1e-12,
                "axis {axis} mismatch at u={u}: q={q_val:?} c={c_val:?}",
            );
        }
    }
}

#[test]
fn live_reduce_rejects_g2() {
    use geometry::{Fatal, FitterParams, GeometryPipeline, Item, TelemetryEvent};

    let mut events: Vec<TelemetryEvent> = vec![];
    let items: Vec<Item> = {
        let mut pipeline = GeometryPipeline::new(FitterParams::default());
        let mut sink = |evt: TelemetryEvent| events.push(evt);
        pipeline
            .process("G2 X10 Y10 I5 J5 F1000\n", &mut sink)
            .collect()
    };

    assert!(
        items.iter().any(|item| matches!(
            item,
            Item::Fatal(Fatal::UnsupportedGcode {
                gcode_kind: "G2/G3",
                ..
            })
        )),
        "G2 input should produce Item::Fatal(Fatal::UnsupportedGcode {{ gcode_kind: \"G2/G3\" }}); got {items:#?}"
    );
}

#[test]
fn live_g0_then_g5_aborts_before_emitting_stale_cubic() {
    use geometry::{Fatal, FitterParams, GeometryPipeline, Item, Segment, TelemetryEvent};

    // Pre-fix bug: G0 X10 was rejected without updating state.position, so
    // the subsequent G5 emitted a cubic with cps[0] = [0,0,0] instead of
    // [10,0,0] — silent 10mm geometric corruption. Post-fix: G0 produces
    // Item::Fatal which terminates the iterator before the G5 is processed.
    let mut p = GeometryPipeline::new(FitterParams::default());
    let mut sink = |_e: TelemetryEvent| {};
    let src = "G0 X10 Y0\nG5 X20 Y0 I3 J3 P-3 Q3 F1000\n";
    let items: Vec<_> = p.process(src, &mut sink).collect();

    // Must contain Item::Fatal for the G0 line.
    assert!(
        items.iter().any(|item| matches!(
            item,
            Item::Fatal(Fatal::UnsupportedGcode {
                gcode_kind: "G0/G1",
                ..
            })
        )),
        "expected Item::Fatal(UnsupportedGcode), got {items:#?}"
    );

    // Must NOT contain a Segment::Cubic (G5 should not have been processed
    // because the iterator went terminal on the G0 Fatal).
    let any_cubic = items
        .iter()
        .any(|item| matches!(item, Item::Segment(Segment::Cubic(_))));
    assert!(
        !any_cubic,
        "post-Fatal cubic emission would mean stale-state corruption; got {items:#?}"
    );
}

#[test]
fn live_reduce_rejects_z_plus_e_as_helical() {
    use geometry::{Fatal, FitterParams, GeometryPipeline, Item, TelemetryEvent};

    // G5 Z move with E delta (pure-Z+E, no XY motion). Must be rejected as
    // helical extrusion — pre-fix this leaked through as EMode::Independent
    // and the splitter would have cloned the full E curve into every child.
    // Round-5 review fix: helical rejection now produces `Item::Fatal` (not
    // `Recovered`) because reduce-stage already committed modal state before
    // the pipeline classified, so a recoverable rejection let subsequent G5s
    // start from the rejected endpoint.
    let mut p = GeometryPipeline::new(FitterParams::default());
    let mut sink = |_e: TelemetryEvent| {};
    let src = "G5 Z10 E5 I0 J0 P0 Q0 F1500\n";
    let items: Vec<_> = p.process(src, &mut sink).collect();

    assert!(
        items
            .iter()
            .any(|item| matches!(item, Item::Fatal(Fatal::HelicalExtrusionUnsupported { .. }))),
        "pure-Z+E G5 should produce Item::Fatal(Fatal::HelicalExtrusionUnsupported); got {items:#?}"
    );
}

#[test]
fn helical_rejection_aborts_before_subsequent_g5_can_inherit_stale_state() {
    use geometry::{Fatal, FitterParams, GeometryPipeline, Item, Segment, TelemetryEvent};

    // Pre-fix bug (round-5 Claim G): the first G5's helical rejection did
    // NOT terminate the pipeline; reduce-stage had already committed
    // state.position = [10, 0, 5] and state.e = 2 before pipeline
    // classified the move as helical. The follow-up valid G5 then started
    // its cubic from [10, 0, 5] instead of [0, 0, 0].
    //
    // Post-fix: helical rejection produces Item::Fatal, the iterator goes
    // terminal, the second G5 never reaches handle_event.
    let mut p = GeometryPipeline::new(FitterParams::default());
    let mut sink = |_e: TelemetryEvent| {};
    let src = "G5 X10 Y0 Z5 I0 J3 P0 Q-3 E2 F1500\nG5 X20 Y0 I3 J3 P-3 Q3 F1500\n";
    let items: Vec<_> = p.process(src, &mut sink).collect();

    // Must contain Item::Fatal for the first (helical) G5.
    assert!(
        items
            .iter()
            .any(|item| matches!(item, Item::Fatal(Fatal::HelicalExtrusionUnsupported { .. }))),
        "expected Item::Fatal(HelicalExtrusionUnsupported), got {items:#?}"
    );

    // Must NOT contain a Segment::Cubic from the second G5 — the iterator
    // should have gone terminal on the Fatal.
    let cubic_count = items
        .iter()
        .filter(|item| matches!(item, Item::Segment(Segment::Cubic(_))))
        .count();
    assert_eq!(
        cubic_count, 0,
        "post-Fatal cubic emission means the iterator continued past the rejection; got {cubic_count} cubics in {items:#?}"
    );
}

#[test]
fn g92_resets_modal_position_for_subsequent_g5() {
    use geometry::{FitterParams, GeometryPipeline, Item, Segment, TelemetryEvent};
    use nurbs::eval::vector_eval;

    // Pre-fix bug: G92 X10 Y20 didn't update state.position. The subsequent
    // G5 emitted P0 = [0,0,0] instead of [10,20,0] — silent geometric
    // corruption. Post-fix: G92 binds params and writes them through to
    // state.position before the marker break.
    let mut p = GeometryPipeline::new(FitterParams::default());
    let mut sink = |_: TelemetryEvent| {};
    let src = "G92 X10 Y20\nG5 X20 Y30 I0 J5 P0 Q-5 F1500\n";
    let items: Vec<_> = p.process(src, &mut sink).collect();

    let cubic = items
        .iter()
        .find_map(|item| match item {
            Item::Segment(Segment::Cubic(c)) => Some(c.clone()),
            _ => None,
        })
        .expect("expected one Segment::Cubic after G92 + G5");

    // P0 must be the post-G92 position, not the default origin.
    let p0 = vector_eval(&cubic.xyz, 0.0);
    assert!(
        (p0[0] - 10.0).abs() < 1e-9 && (p0[1] - 20.0).abs() < 1e-9,
        "post-G92 G5 P0 should be [10, 20, *], got {p0:?}"
    );
}

#[test]
fn g92_e0_resets_modal_e_for_subsequent_g5_e_delta() {
    use geometry::{EMode, FitterParams, GeometryPipeline, Item, Segment, TelemetryEvent};

    // Pre-fix: G92 E0 didn't update state.e. The next G5 with E5 computed
    // e_delta = 5 - state.e (whatever was there before), instead of 5 - 0.
    let mut p = GeometryPipeline::new(FitterParams::default());
    let mut sink = |_: TelemetryEvent| {};
    // G5 X10 sets state.e implicitly via E word? Actually G5 with no E word
    // doesn't change state.e. Set state.e via an E-only G1 first — but G1 is
    // unsupported in live mode. So just rely on the default state.e = 0.0
    // and verify the E delta from G92 E0 reset.
    //
    // Sequence: default e = 0. G92 E5 sets state.e = 5. Then G5 with E10
    // produces e_delta = 10 - 5 = 5. The CoupledToXy classifier then computes
    // extrusion_per_xy_mm = 5 / xy_arc_length.
    let src = "G92 E5\nG5 X10 Y0 I3 J3 P-3 Q3 E10 F1500\n";
    let items: Vec<_> = p.process(src, &mut sink).collect();

    let cubic = items
        .iter()
        .find_map(|item| match item {
            Item::Segment(Segment::Cubic(c)) => Some(c.clone()),
            _ => None,
        })
        .expect("expected one Segment::Cubic after G92 E + G5");

    // CoupledToXy with positive ratio (10 - 5 = 5 mm of extrusion).
    assert_eq!(cubic.e_mode, EMode::CoupledToXy);
    assert!(
        cubic.extrusion_per_xy_mm > 0.0,
        "expected positive extrusion ratio (e_delta=5), got {}",
        cubic.extrusion_per_xy_mm
    );
}

#[test]
fn g18_then_g5_emits_plane_mismatch_recovery() {
    use geometry::{FitterParams, GeometryPipeline, Item, Recovery, TelemetryEvent};

    // Pre-fix: G5 ignored active_plane and emitted a CurveGeom::Cubic as if
    // XY. Post-fix: mirrors G5.1's plane check; emits Recovery::G5PlaneMismatch.
    let mut p = GeometryPipeline::new(FitterParams::default());
    let mut sink = |_: TelemetryEvent| {};
    let src = "G18\nG5 X10 Y0 I3 J3 P-3 Q3 F1500\n";
    let items: Vec<_> = p.process(src, &mut sink).collect();

    assert!(
        items.iter().any(|item| matches!(
            item,
            Item::Recovered(
                _,
                Recovery::G5PlaneMismatch {
                    active_plane_g_code: 18,
                    ..
                }
            )
        )),
        "G18 + G5 should produce Recovery::G5PlaneMismatch {{ 18 }}, got {items:#?}"
    );
}

#[test]
fn g19_then_g5_emits_plane_mismatch_recovery() {
    use geometry::{FitterParams, GeometryPipeline, Item, Recovery, TelemetryEvent};

    let mut p = GeometryPipeline::new(FitterParams::default());
    let mut sink = |_: TelemetryEvent| {};
    let src = "G19\nG5 X10 Y0 I3 J3 P-3 Q3 F1500\n";
    let items: Vec<_> = p.process(src, &mut sink).collect();

    assert!(
        items.iter().any(|item| matches!(
            item,
            Item::Recovered(
                _,
                Recovery::G5PlaneMismatch {
                    active_plane_g_code: 19,
                    ..
                }
            )
        )),
        "G19 + G5 should produce Recovery::G5PlaneMismatch {{ 19 }}, got {items:#?}"
    );
}

#[test]
fn nan_g5_produces_malformed_params_recovery_not_silent_drop() {
    use geometry::{FitterParams, GeometryPipeline, Item, Recovery, TelemetryEvent};

    // Pre-Fix-H: silent ZeroMotion drop. Rust's f64::FromStr accepts "NaN",
    // so the lexer surfaced the move with NaN-poisoned XY params. The
    // pipeline's ZeroMotion classifier then dropped the move (NaN > 1e-6
    // is false), and modal state.position became NaN-poisoned for every
    // subsequent G5 — silent geometric corruption with zero telemetry.
    //
    // Post-Fix-H: lexer rejects NaN as MalformedNumber, the geometry
    // pipeline maps the parse error to Recovery::MalformedParams via the
    // existing handle_event::ParseError path.
    let mut p = GeometryPipeline::new(FitterParams::default());
    let mut sink = |_e: TelemetryEvent| {};
    let src = "G5 XNaN Y0 I0 J3 P0 Q-3 F1500\n";
    let items: Vec<_> = p.process(src, &mut sink).collect();

    assert!(
        items
            .iter()
            .any(|item| matches!(item, Item::Recovered(_, Recovery::MalformedParams { .. }))),
        "expected Item::Recovered(_, MalformedParams), got {items:#?}"
    );
}

#[test]
fn try_new_rejects_non_finite_control_point() {
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [f64::NAN, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [3.0, 0.0, 0.0],
        ],
    )
    .expect("VectorNurbs accepts NaN at the type level; CubicSegment::try_new must catch it");
    let result = CubicSegment::try_new(
        xyz,
        EMode::Travel,
        0.0,
        None,
        100.0,
        SourceRange {
            start_line: 1,
            end_line: 1,
        },
        None,
    );
    assert!(matches!(
        result,
        Err(GeometryError::NotSinglePieceCubic { .. })
    ));
}

#[test]
fn try_new_rejects_non_finite_feedrate() {
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [3.0, 0.0, 0.0],
        ],
    )
    .unwrap();
    let result = CubicSegment::try_new(
        xyz,
        EMode::Travel,
        0.0,
        None,
        f64::INFINITY,
        SourceRange {
            start_line: 1,
            end_line: 1,
        },
        None,
    );
    assert!(matches!(
        result,
        Err(GeometryError::EModeInvariantViolation { .. })
    ));
}

#[test]
fn try_new_rejects_non_finite_extrusion_ratio() {
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [3.0, 0.0, 0.0],
        ],
    )
    .unwrap();
    let result = CubicSegment::try_new(
        xyz,
        EMode::CoupledToXy,
        f64::NAN,
        None,
        100.0,
        SourceRange {
            start_line: 1,
            end_line: 1,
        },
        None,
    );
    assert!(matches!(
        result,
        Err(GeometryError::EModeInvariantViolation { .. })
    ));
}

#[test]
fn try_new_rejects_non_finite_e_independent_curve() {
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![[0.0; 3]; 4],
    )
    .unwrap();
    let bad_e =
        ScalarNurbs::<f64>::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, f64::INFINITY]).unwrap();
    let result = CubicSegment::try_new(
        xyz,
        EMode::Independent,
        0.0,
        Some(bad_e),
        100.0,
        SourceRange {
            start_line: 1,
            end_line: 1,
        },
        None,
    );
    assert!(matches!(
        result,
        Err(GeometryError::EModeInvariantViolation { .. })
    ));
}
