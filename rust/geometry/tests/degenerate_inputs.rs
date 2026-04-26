//! Degenerate input handling: empty file, single G1, comment-only file,
//! malformed line. Each should surface through Item or Recovery without
//! panicking.

use geometry::{FitterParams, GeometryPipeline, Item, Recovery, TelemetryEvent};

fn run(text: &str) -> (Vec<Item>, Vec<TelemetryEvent>) {
    let mut events = vec![];
    let mut p = GeometryPipeline::new(FitterParams::default());
    let items: Vec<_> = {
        let mut sink = |e: TelemetryEvent| events.push(e);
        p.process(text, &mut sink).collect()
    };
    (items, events)
}

#[test]
fn empty_input() {
    let (items, events) = run("");
    assert!(items.is_empty());
    assert!(events.is_empty());
}

#[test]
fn comment_only_file_with_layer() {
    let (items, events) = run(";LAYER:0\n; just a comment\n");
    assert!(items.is_empty(), "comments alone produce no segments");
    assert_eq!(events.len(), 1);
    assert!(matches!(events[0], TelemetryEvent::LayerChange { layer: Some(0), .. }));
}

#[test]
fn single_g1_no_junction_emitted() {
    let (items, _) = run("G1 X10 F1500\n");
    assert_eq!(items.len(), 1);
    // First G1 has no preceding G1, so no junction.
}

#[test]
fn malformed_line_yields_recovered() {
    let (items, _) = run("G1 X1.2.3\n");
    assert_eq!(items.len(), 1);
    assert!(matches!(items[0], Item::Recovered(_, Recovery::MalformedParams { .. })));
}

#[test]
fn unknown_command_yields_recovered() {
    let (items, _) = run("123 X1\n");
    assert_eq!(items.len(), 1);
    assert!(matches!(items[0], Item::Recovered(_, Recovery::UnrecognizedCommand { .. })));
}
