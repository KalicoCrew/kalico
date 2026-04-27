//! End-to-end integration tests for G5 / G5.1 reduction (build-order Step 3).
//! Black-box: drives `GeometryPipeline::process` against synthetic G-code
//! strings and asserts on the public `Item` / `Segment` / `Recovery` /
//! `TelemetryEvent` surface.

use geometry::{
    FittedSegment, FitterParams, GeometryPipeline, Item, Recovery, Segment,
    TelemetryEvent,
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

fn approx(a: f64, b: f64) -> bool { (a - b).abs() < 1e-12 }

#[test]
fn single_g5_emits_one_cubic_fitted_segment() {
    let (items, _events) = process("G1 X0 Y0 F1500\nG5 X10 Y0 I3 J3 P-3 Q3\n");
    let cubics: Vec<&FittedSegment> = items
        .iter()
        .filter_map(|it| match it {
            Item::Segment(Segment::Fitted(f)) if f.degree == 3 => Some(f),
            _ => None,
        })
        .collect();
    assert_eq!(cubics.len(), 1, "expected exactly one degree-3 Fitted, got {} in {items:#?}", cubics.len());
    let f = cubics[0];
    let cps = f.xyz.control_points();
    assert!(approx(cps[1][0], 3.0) && approx(cps[1][1], 3.0));
    assert!(approx(cps[2][0], 7.0) && approx(cps[2][1], 3.0));
    assert!(f.xyz.weights().is_none());
    assert!(approx(f.max_residual_mm, 0.0));
}

#[test]
fn single_g5_1_emits_one_quadratic_non_rational_fitted_segment() {
    let (items, _events) = process("G1 X0 Y0 F1500\nG5.1 X10 Y0 I3 J3\n");
    let quads: Vec<&FittedSegment> = items
        .iter()
        .filter_map(|it| match it {
            Item::Segment(Segment::Fitted(f)) if f.degree == 2 => Some(f),
            _ => None,
        })
        .collect();
    assert_eq!(quads.len(), 1, "expected exactly one degree-2 Fitted, got {} in {items:#?}", quads.len());
    let f = quads[0];
    let cps = f.xyz.control_points();
    assert!(approx(cps[1][0], 3.0) && approx(cps[1][1], 3.0));
    assert!(f.xyz.weights().is_none(), "G5.1 must be non-rational (distinguishes from Arc)");
}

#[test]
fn g5_chain_three_lines_no_junctions_between() {
    let (items, _events) = process(
        "G1 X0 Y0 F1500\n\
         G5 X10 Y0 I3 J3 P-3 Q3\n\
         G5 X20 Y0 P-2 Q2\n\
         G5 X30 Y0 P0 Q0\n",
    );
    let cubics_count = items.iter().filter(|it| matches!(it, Item::Segment(Segment::Fitted(f)) if f.degree == 3)).count();
    let junctions_count = items.iter().filter(|it| matches!(it, Item::Segment(Segment::Junction(_)))).count();
    assert_eq!(cubics_count, 3, "expected 3 cubic G5 segments");
    assert_eq!(junctions_count, 0, "G5↔G5 boundaries should produce no junctions");
}

#[test]
fn g5_followed_by_g1_breaks_chain_no_junction() {
    let (items, _events) = process(
        "G1 X0 Y0 F1500\n\
         G5 X10 Y0 I3 J3 P-3 Q3\n\
         G1 X20 Y0\n",
    );
    let junctions_count = items.iter().filter(|it| matches!(it, Item::Segment(Segment::Junction(_)))).count();
    assert_eq!(junctions_count, 0, "G5→G1 boundary should not produce a junction");
}

#[test]
fn g5_chain_break_then_implicit_tangent_emits_recovery() {
    let (items, events) = process(
        "G1 X0 Y0 F1500\n\
         G5 X10 Y0 I3 J3 P-3 Q3\n\
         G1 X11 Y0\n\
         G5 X20 Y0 P-2 Q2\n",
    );
    let recoveries: Vec<_> = items.iter().filter_map(|it| match it {
        Item::Recovered(_, r @ Recovery::G5MissingTangent { .. }) => Some(r.clone()),
        _ => None,
    }).collect();
    assert_eq!(recoveries.len(), 1, "expected one G5MissingTangent recovery, got {items:#?}");
    let recovery_in_sink = events.iter().any(|e| matches!(e, TelemetryEvent::Recovery(Recovery::G5MissingTangent { .. })));
    assert!(recovery_in_sink, "Recovery should also appear in sink (dual-emit)");
}

#[test]
fn g5_1_outside_g17_plane_emits_recovery() {
    let (items, _events) = process("G18\nG5.1 X10 Z1 I3 J3\n");
    let recoveries: Vec<_> = items.iter().filter_map(|it| match it {
        Item::Recovered(_, r @ Recovery::G5PlaneMismatch { .. }) => Some(r.clone()),
        _ => None,
    }).collect();
    assert_eq!(recoveries.len(), 1, "expected one G5PlaneMismatch recovery");
    match &recoveries[0] {
        Recovery::G5PlaneMismatch { active_plane_g_code: 18, line_no: 2 } => {}
        other => panic!("expected G5PlaneMismatch with active_plane_g_code=18, got {other:?}"),
    }
}

#[test]
fn g5_with_z_motion_interpolates_z_at_thirds() {
    let (items, _events) = process(
        "G1 X0 Y0 Z0 F1500\nG5 X10 Y0 Z0.3 I3 J3 P-3 Q3\n",
    );
    let f = items.iter().find_map(|it| match it {
        Item::Segment(Segment::Fitted(f)) if f.degree == 3 => Some(f),
        _ => None,
    }).expect("expected a degree-3 Fitted");
    let cps = f.xyz.control_points();
    assert!(approx(cps[0][2], 0.0));
    assert!(approx(cps[1][2], 0.1));
    assert!(approx(cps[2][2], 0.2));
    assert!(approx(cps[3][2], 0.3));
}

#[test]
fn g5_1_with_z_motion_interpolates_z_at_midpoint() {
    let (items, _events) = process(
        "G1 X0 Y0 Z0 F1500\nG5.1 X10 Y0 Z0.4 I3 J3\n",
    );
    let f = items.iter().find_map(|it| match it {
        Item::Segment(Segment::Fitted(f)) if f.degree == 2 => Some(f),
        _ => None,
    }).expect("expected a degree-2 Fitted");
    let cps = f.xyz.control_points();
    assert!(approx(cps[0][2], 0.0));
    assert!(approx(cps[1][2], 0.2));
    assert!(approx(cps[2][2], 0.4));
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
    let cubics_count = items.iter().filter(|it| matches!(it, Item::Segment(Segment::Fitted(f)) if f.degree == 3)).count();
    assert_eq!(cubics_count, 2, "expected 2 cubics — modal chain should survive M and T");
    let recoveries_count = items.iter().filter(|it| matches!(it, Item::Recovered(_, Recovery::G5MissingTangent { .. }))).count();
    assert_eq!(recoveries_count, 0, "expected no missing-tangent recoveries");
}
