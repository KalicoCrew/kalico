//! End-to-end integration tests for G5 / G5.1 reduction.
//! Black-box: drives `GeometryPipeline::process` against synthetic G-code
//! strings and asserts on the public `Item` / `Segment` / `Recovery` /
//! `TelemetryEvent` surface.
//!
//! Per Task 1.6 (build-order Step 7-pre), G5 / G5.1 emit `Segment::Cubic`
//! (G5.1 via exact degree-elevation 2→3). These tests run only under
//! `legacy-reference` because they prefix moves with `G1 X0 Y0 F1500` to
//! seat feedrate, which the live pipeline rejects as `UnsupportedGcode`.

#![cfg(feature = "legacy-reference")]

use geometry::{
    CubicSegment, FitterParams, GeometryPipeline, Item, Recovery, Segment, TelemetryEvent,
};

fn process(text: &str) -> (Vec<Item>, Vec<TelemetryEvent>) {
    let mut p = GeometryPipeline::new(FitterParams::default());
    let mut events = vec![];
    let items: Vec<_> = {
        let mut sink = |e: TelemetryEvent| events.push(e);
        p.process(text, &mut sink).collect()
    };
    (items, events)
}

fn approx(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-12
}

#[test]
fn single_g5_emits_one_cubic_segment() {
    let (items, _events) = process("G1 X0 Y0 F1500\nG5 X10 Y0 I3 J3 P-3 Q3\n");
    let cubics: Vec<&CubicSegment> = items
        .iter()
        .filter_map(|it| match it {
            Item::Segment(Segment::Cubic(c)) => Some(c),
            _ => None,
        })
        .collect();
    assert_eq!(
        cubics.len(),
        1,
        "expected exactly one Segment::Cubic, got {} in {items:#?}",
        cubics.len()
    );
    let c = cubics[0];
    let cps = c.xyz.control_points();
    assert!(approx(cps[1][0], 3.0) && approx(cps[1][1], 3.0));
    assert!(approx(cps[2][0], 7.0) && approx(cps[2][1], 3.0));
    assert!(c.xyz.weights().is_none());
}

#[test]
fn single_g5_1_emits_one_non_rational_cubic_via_degree_elevation() {
    let (items, _events) = process("G1 X0 Y0 F1500\nG5.1 X10 Y0 I3 J3\n");
    let cubics: Vec<&CubicSegment> = items
        .iter()
        .filter_map(|it| match it {
            Item::Segment(Segment::Cubic(c)) => Some(c),
            _ => None,
        })
        .collect();
    assert_eq!(
        cubics.len(),
        1,
        "expected exactly one Segment::Cubic from G5.1, got {} in {items:#?}",
        cubics.len()
    );
    let c = cubics[0];
    // Post-elevation: degree 3, 4 CPs, non-rational.
    assert_eq!(c.xyz.degree(), 3);
    assert_eq!(c.xyz.control_points().len(), 4);
    assert!(
        c.xyz.weights().is_none(),
        "G5.1 → cubic must remain non-rational (distinguishes from Arc)"
    );
}

#[test]
fn g5_chain_three_lines_no_junctions_between() {
    let (items, _events) = process(
        "G1 X0 Y0 F1500\n\
         G5 X10 Y0 I3 J3 P-3 Q3\n\
         G5 X20 Y0 P-2 Q2\n\
         G5 X30 Y0 P0 Q0\n",
    );
    let cubics_count = items
        .iter()
        .filter(|it| matches!(it, Item::Segment(Segment::Cubic(_))))
        .count();
    let junctions_count = items
        .iter()
        .filter(|it| matches!(it, Item::Segment(Segment::Junction(_))))
        .count();
    assert_eq!(cubics_count, 3, "expected 3 cubic G5 segments");
    assert_eq!(
        junctions_count, 0,
        "G5↔G5 boundaries should produce no junctions"
    );
}

#[test]
fn g5_followed_by_g1_breaks_chain_no_junction() {
    let (items, _events) = process(
        "G1 X0 Y0 F1500\n\
         G5 X10 Y0 I3 J3 P-3 Q3\n\
         G1 X20 Y0\n",
    );
    let junctions_count = items
        .iter()
        .filter(|it| matches!(it, Item::Segment(Segment::Junction(_))))
        .count();
    assert_eq!(
        junctions_count, 0,
        "G5→G1 boundary should not produce a junction"
    );
}

#[test]
fn g5_chain_break_then_implicit_tangent_emits_recovery() {
    let (items, events) = process(
        "G1 X0 Y0 F1500\n\
         G5 X10 Y0 I3 J3 P-3 Q3\n\
         G1 X11 Y0\n\
         G5 X20 Y0 P-2 Q2\n",
    );
    let recoveries: Vec<_> = items
        .iter()
        .filter_map(|it| match it {
            Item::Recovered(_, r @ Recovery::G5MissingTangent { .. }) => Some(r.clone()),
            _ => None,
        })
        .collect();
    assert_eq!(
        recoveries.len(),
        1,
        "expected one G5MissingTangent recovery, got {items:#?}"
    );
    let recovery_in_sink = events.iter().any(|e| {
        matches!(
            e,
            TelemetryEvent::Recovery(Recovery::G5MissingTangent { .. })
        )
    });
    assert!(
        recovery_in_sink,
        "Recovery should also appear in sink (dual-emit)"
    );
}

#[test]
fn g5_1_outside_g17_plane_emits_recovery() {
    let (items, _events) = process("G18\nG5.1 X10 Z1 I3 J3\n");
    let recoveries: Vec<_> = items
        .iter()
        .filter_map(|it| match it {
            Item::Recovered(_, r @ Recovery::G5PlaneMismatch { .. }) => Some(r.clone()),
            _ => None,
        })
        .collect();
    assert_eq!(recoveries.len(), 1, "expected one G5PlaneMismatch recovery");
    match &recoveries[0] {
        Recovery::G5PlaneMismatch {
            active_plane_g_code: 18,
            line_no: 2,
        } => {}
        other => panic!("expected G5PlaneMismatch with active_plane_g_code=18, got {other:?}"),
    }
}

#[test]
fn g5_with_z_motion_rejected_as_helical_extrusion_when_e_present() {
    // G5 with both Z and E → helical extrusion (design-rejected by classifier).
    // Round-5 review fix: surfaces as Item::Fatal (not Recovered) because
    // reduce-stage commits modal state before classification — recoverable
    // rejection would let subsequent G5s start from the rejected endpoint.
    use geometry::Fatal;
    let (items, _events) = process("G1 X0 Y0 Z0 F1500\nG5 X10 Y0 Z0.3 E0.5 I3 J3 P-3 Q3\n");
    let helical = items
        .iter()
        .find(|it| matches!(it, Item::Fatal(Fatal::HelicalExtrusionUnsupported { .. })));
    assert!(
        helical.is_some(),
        "expected Item::Fatal(HelicalExtrusionUnsupported), got {items:#?}"
    );
}

#[test]
fn g5_with_z_motion_no_e_emits_travel_cubic() {
    let (items, _events) = process("G1 X0 Y0 Z0 F1500\nG5 X10 Y0 Z0.3 I3 J3 P-3 Q3\n");
    let c = items
        .iter()
        .find_map(|it| match it {
            Item::Segment(Segment::Cubic(c)) => Some(c),
            _ => None,
        })
        .expect("expected a Segment::Cubic");
    let cps = c.xyz.control_points();
    // Z linearly interpolated at thirds: 0, 0.1, 0.2, 0.3.
    assert!(approx(cps[0][2], 0.0));
    assert!(approx(cps[1][2], 0.1));
    assert!(approx(cps[2][2], 0.2));
    assert!(approx(cps[3][2], 0.3));
    // No E → Travel.
    assert_eq!(c.e_mode, geometry::EMode::Travel);
}

#[test]
fn g5_chain_preserved_by_m_codes_and_t_codes() {
    let (items, _events) = process(
        "G1 X0 Y0 F1500\n\
         G5 X10 Y0 I3 J3 P-3 Q3\n\
         M104 S210\n\
         T0\n\
         G5 X20 Y0 P-2 Q2\n",
    );
    let cubics_count = items
        .iter()
        .filter(|it| matches!(it, Item::Segment(Segment::Cubic(_))))
        .count();
    assert_eq!(
        cubics_count, 2,
        "expected 2 cubics — modal chain should survive M and T"
    );
    let recoveries_count = items
        .iter()
        .filter(|it| matches!(it, Item::Recovered(_, Recovery::G5MissingTangent { .. })))
        .count();
    assert_eq!(
        recoveries_count, 0,
        "expected no missing-tangent recoveries"
    );
}
