//! End-to-end smoke test on the `OrcaSlicer` corpus. Phase 1 emits one
//! `FittedSegment` per G1, plus `JunctionDeviation` between consecutive G1s,
//! plus `ArcSegment`s for G2/G3. Test:
//!  - Pipeline runs to completion without panic.
//!  - Segment counts are within sane order-of-magnitude.
//!  - Telemetry sees expected events.

use geometry::{
    FitterParams, GeometryPipeline, Item, Segment, TelemetryEvent,
};
use std::path::Path;

const CORPUS_DIR: &str = "../../scripts/fitter_prototype/corpus";

fn read_corpus_file(name: &str) -> Option<String> {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join(CORPUS_DIR).join(name);
    std::fs::read_to_string(&path).ok()
}

#[derive(Default)]
struct Counts {
    fitted: u64,
    arc: u64,
    junction: u64,
    corner_blend: u64,
    recovered: u64,
    fatal: u64,
    layer_changes: u64,
    tool_changes: u64,
    retractions: u64,
}

fn run_corpus(name: &str) -> Option<Counts> {
    let text = read_corpus_file(name)?;
    let mut counts = Counts::default();
    let mut p = GeometryPipeline::new(FitterParams::default());
    let mut sink = |e: TelemetryEvent| match e {
        TelemetryEvent::LayerChange { .. } => counts.layer_changes += 1,
        TelemetryEvent::ToolChange { .. } => counts.tool_changes += 1,
        TelemetryEvent::Retraction { .. } => counts.retractions += 1,
        _ => {}
    };
    for item in p.process(&text, &mut sink) {
        match item {
            Item::Segment(seg) => match seg {
                Segment::Fitted(_) => counts.fitted += 1,
                Segment::Arc(_) => counts.arc += 1,
                Segment::Junction(_) => counts.junction += 1,
                Segment::CornerBlend(_) => counts.corner_blend += 1,
                _ => {}
            },
            Item::Recovered(_, _) => counts.recovered += 1,
            Item::Fatal(_) => {
                counts.fatal += 1;
                break;
            }
            _ => {}
        }
    }
    Some(counts)
}

#[test]
fn arc_fitted_corpus_runs_end_to_end() {
    let Some(c) = run_corpus("voron_cube_arc_fitted.gcode") else {
        eprintln!("skipping: corpus file not present");
        return;
    };
    eprintln!(
        "arc_fitted: fitted={} arc={} junction={} cornerblend={} recovered={} \
         fatal={} layers={} tools={} retracts={}",
        c.fitted, c.arc, c.junction, c.corner_blend, c.recovered,
        c.fatal, c.layer_changes, c.tool_changes, c.retractions,
    );
    assert_eq!(c.fatal, 0, "Phase 1 should not fatal on legitimate input");
    assert_eq!(c.corner_blend, 0, "Phase 1 emits no CornerBlendSlot");
    assert!(c.fitted > 100_000, "expected > 100k FittedSegments, got {}", c.fitted);
    assert!(c.arc > 5_000, "expected > 5k ArcSegments (corpus has ~9710 G2/G3), got {}", c.arc);
    assert!(c.layer_changes >= 1, "expected at least one LayerChange");
}

#[test]
fn straight_line_corpus_runs_end_to_end() {
    let Some(c) = run_corpus("voron_cube_straight_line.gcode") else {
        eprintln!("skipping: corpus file not present");
        return;
    };
    eprintln!(
        "straight_line: fitted={} arc={} junction={} cornerblend={} recovered={} \
         fatal={} layers={} tools={} retracts={}",
        c.fitted, c.arc, c.junction, c.corner_blend, c.recovered,
        c.fatal, c.layer_changes, c.tool_changes, c.retractions,
    );
    assert_eq!(c.fatal, 0);
    assert_eq!(c.arc, 0, "straight-line corpus has no G2/G3");
    assert!(c.fitted > 150_000, "expected > 150k FittedSegments, got {}", c.fitted);
}
