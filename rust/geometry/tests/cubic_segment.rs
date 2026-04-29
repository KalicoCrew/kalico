use geometry::{CubicSegment, EMode, GeometryError, SourceRange};
use nurbs::{VectorNurbs, eval::vector_eval};

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

#[cfg(not(feature = "legacy-reference"))]
#[test]
fn live_reduce_rejects_g1() {
    use geometry::{Fatal, FitterParams, GeometryPipeline, Item, TelemetryEvent};

    let mut events: Vec<TelemetryEvent> = vec![];
    let items: Vec<Item> = {
        let mut pipeline = GeometryPipeline::new(FitterParams::default());
        let mut sink = |evt: TelemetryEvent| events.push(evt);
        pipeline
            .process("G1 X10 Y10 F1000\n", &mut sink)
            .collect()
    };

    assert!(
        items.iter().any(|item| matches!(
            item,
            Item::Fatal(Fatal::UnsupportedGcode { gcode_kind: "G0/G1", .. })
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
        None,
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

#[cfg(not(feature = "legacy-reference"))]
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
            Item::Fatal(Fatal::UnsupportedGcode { gcode_kind: "G2/G3", .. })
        )),
        "G2 input should produce Item::Fatal(Fatal::UnsupportedGcode {{ gcode_kind: \"G2/G3\" }}); got {items:#?}"
    );
}

#[test]
#[cfg(not(feature = "legacy-reference"))]
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
            Item::Fatal(Fatal::UnsupportedGcode { gcode_kind: "G0/G1", .. })
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
    use geometry::{FitterParams, GeometryPipeline, Item, Recovery, TelemetryEvent};

    // G5 Z move with E delta (pure-Z+E, no XY motion). Must be rejected as
    // helical extrusion — pre-fix this leaked through as EMode::Independent
    // and the splitter would have cloned the full E curve into every child.
    let mut p = GeometryPipeline::new(FitterParams::default());
    let mut sink = |_e: TelemetryEvent| {};
    let src = "G5 Z10 E5 I0 J0 P0 Q0 F1500\n";
    let items: Vec<_> = p.process(src, &mut sink).collect();

    assert!(
        items.iter().any(|item| matches!(
            item,
            Item::Recovered(_, Recovery::HelicalExtrusionUnsupported { .. })
        )),
        "pure-Z+E G5 should produce Recovery::HelicalExtrusionUnsupported, got {items:#?}"
    );
}
