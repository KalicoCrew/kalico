# `geometry`

Layer 1 geometry pipeline for the kalico motion planner. Token stream →
typed segments. Phase 1 (current): degree-1 NURBS for G1, 3D rational
quadratic for G2/G3, JunctionDeviation between consecutive G1s. Phase 2
adds the LSPIA fitter, classifier, and corner-blend slot construction.

See `docs/superpowers/specs/2026-04-26-layer-1-rust-architecture-design.md`.

## Public surface

```rust
let mut pipeline = GeometryPipeline::new(FitterParams::default());
let mut sink = |event: TelemetryEvent| { /* observability */ };
for item in pipeline.process(&gcode_text, &mut sink) {
    match item {
        Item::Segment(s) => { /* normal */ }
        Item::Recovered(s, recovery) => { /* anomaly + segment */ }
        Item::Fatal(f) => { /* terminal */ break; }
    }
}
```

## Phase 1 vs Phase 2

Public API is identical across phases. Phase 1 produces a subset of segment
kinds:

- Emitted in Phase 1: `Segment::Fitted` (degree 1 only), `Segment::Arc`,
  `Segment::Junction`.
- Defined but never produced in Phase 1: `Segment::CornerBlend`,
  `Recovery::WindowCapHit`, `Recovery::DegenerateSlotFallback`,
  `Recovery::ToleranceExceeded`, `Recovery::LspiaNotConverged`,
  `TelemetryEvent::WindowFlush`, `TelemetryEvent::FitObservation`.

Consumers handle the full enum from day one — no API churn at the phase
boundary.
