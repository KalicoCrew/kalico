use super::*;
use crate::{Item, Recovery, Segment, TelemetryEvent};

fn collect(text: &str) -> Vec<Item> {
    let mut p = GeometryPipeline::new(FitterParams::default());
    let mut sink = |_: crate::TelemetryEvent| {};
    p.process(text, &mut sink).collect()
}

#[test]
fn empty_input_yields_no_items() {
    let mut p = GeometryPipeline::new(FitterParams::default());
    let mut sink = |_e: crate::TelemetryEvent| {};
    let items: Vec<_> = p.process("", &mut sink).collect();
    assert!(items.is_empty());
}

#[test]
fn whitespace_input_yields_no_items() {
    let mut p = GeometryPipeline::new(FitterParams::default());
    let mut sink = |_e: crate::TelemetryEvent| {};
    let items: Vec<_> = p.process("\n\n   \n", &mut sink).collect();
    assert!(items.is_empty());
}

#[test]
fn layer_change_marker_fires_telemetry() {
    let mut events = vec![];
    let mut p = GeometryPipeline::new(FitterParams::default());
    let _items: Vec<_> = {
        let mut sink = |e: TelemetryEvent| events.push(e);
        p.process(";LAYER:5\n", &mut sink).collect()
    };
    assert!(matches!(
        events.as_slice(),
        [TelemetryEvent::LayerChange {
            layer: Some(5),
            line_no: 1
        }]
    ));
}

#[test]
fn tool_change_fires_telemetry() {
    let mut events = vec![];
    let mut p = GeometryPipeline::new(FitterParams::default());
    let _items: Vec<_> = {
        let mut sink = |e: TelemetryEvent| events.push(e);
        p.process("T1\n", &mut sink).collect()
    };
    assert!(matches!(
        events.as_slice(),
        [TelemetryEvent::ToolChange {
            tool: 1,
            line_no: 1
        }]
    ));
}

#[test]
fn g5_emits_cubic_segment() {
    let items = collect("G5 X10 Y0 I3 J3 P-3 Q3\n");
    let cubic_seg = items.iter().find_map(|it| match it {
        Item::Segment(Segment::Cubic(c)) => Some(c),
        _ => None,
    });
    let c = cubic_seg.expect("expected a Segment::Cubic");
    assert_eq!(c.xyz.degree(), 3);
    let cps = c.xyz.control_points();
    assert_eq!(cps.len(), 4);
    let approx = |a: f64, b: f64| (a - b).abs() < 1e-12;
    assert!(approx(cps[0][0], 0.0) && approx(cps[0][1], 0.0));
    assert!(approx(cps[1][0], 3.0) && approx(cps[1][1], 3.0));
    assert!(approx(cps[2][0], 7.0) && approx(cps[2][1], 3.0));
    assert!(approx(cps[3][0], 10.0) && approx(cps[3][1], 0.0));
    let knots = c.xyz.knots();
    assert_eq!(knots, &[0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]);
    assert_eq!(c.e_mode, crate::EMode::Travel);
    assert!(
        !items
            .iter()
            .any(|it| matches!(it, Item::Segment(Segment::Junction(_)))),
        "G5 must not emit a junction here, got {items:#?}"
    );
}

#[test]
fn g5_1_emits_cubic_via_degree_elevation() {
    let items = collect("G5.1 X10 Y0 I3 J3\n");
    let cubic_seg = items.iter().find_map(|it| match it {
        Item::Segment(Segment::Cubic(c)) => Some(c),
        _ => None,
    });
    let c = cubic_seg.expect("expected a Segment::Cubic from G5.1");
    assert_eq!(c.xyz.degree(), 3);
    assert_eq!(c.xyz.control_points().len(), 4);
    let knots = c.xyz.knots();
    assert_eq!(knots, &[0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]);
    let cps = c.xyz.control_points();
    let approx = |a: f64, b: f64| (a - b).abs() < 1e-12;
    assert!(approx(cps[0][0], 0.0) && approx(cps[0][1], 0.0));
    assert!(approx(cps[3][0], 10.0) && approx(cps[3][1], 0.0));
    assert_eq!(c.e_mode, crate::EMode::Travel);
}

#[test]
fn g5_1_outside_xy_plane_yields_recovered() {
    let mut events = vec![];
    let mut p = GeometryPipeline::new(FitterParams::default());
    let items: Vec<_> = {
        let mut sink = |e: TelemetryEvent| events.push(e);
        p.process("G18\nG5.1 X10 Z1 I3 J3\n", &mut sink).collect()
    };
    let recovered = items.iter().find_map(|it| match it {
        Item::Recovered(
            _,
            Recovery::G5PlaneMismatch {
                line_no: 2,
                active_plane_g_code: 18,
            },
        ) => Some(()),
        _ => None,
    });
    assert!(
        recovered.is_some(),
        "expected G5PlaneMismatch, got {items:#?}"
    );
    assert!(matches!(
        events.last(),
        Some(TelemetryEvent::Recovery(Recovery::G5PlaneMismatch {
            line_no: 2,
            active_plane_g_code: 18
        }))
    ));
}
