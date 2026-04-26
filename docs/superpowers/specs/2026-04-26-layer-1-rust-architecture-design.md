# Layer 1 Rust Architecture — Design

**Date:** 2026-04-26
**Status:** Spec — design approved, implementation plan to follow
**Layer:** 1 (Geometry pipeline)
**Driver:** Rust-side architecture for the kalico motion-planner rewrite

## 1. Context

This spec defines the Rust architecture for Layer 1 of the kalico motion
planner — the geometry pipeline that ingests a complete G-code file, parses
it, reduces G2/G3 to exact NURBS, classifies G1 vertices, fits smooth runs as
B-splines, and emits a typed segment stream consumable by Layer 2 (TOPP-RA).

**Algorithm calls are settled by the spike**
(`docs/superpowers/spikes/2026-04-26-layer-1-fitter-spike.md`): LSPIA +
chord-bound for smooth-run fits, CMLT-style classifier with `θ_smooth = 15°` /
`θ_hard = 60°` defaults, parameterized cubic-Bezier corner-blend slots,
junction-deviation as the third fallback, G2/G3 passthrough as exact rational
quadratic NURBS. This spec does not re-litigate them.

**What is genuinely new** in this spec is the Rust architecture: crate
layout, public API shape, segment type design (built on the Layer 0 `nurbs`
crate), error model, telemetry-sink pattern, phasing across the build-order
MVP boundary (step 6 ships parser + reduce + junction-deviation; step 7 adds
classifier + fitter + corner-blend slot), and testing strategy.

### 1.1 Non-goals

- Layer 3 corner-blend shape finalization
- TOPP-RA, kinematic-limit enforcement
- Async or threading
- Multi-slicer corpus expansion (deferred per spike's open questions)
- Wire-protocol commitments to MCU
- Performance benchmarks (revisit at the future `geometry-c-api/` boundary)

### 1.2 Driving constraints

- **Offline-file premise.** Pipeline ingests a complete G-code buffer
  (`&str`), not a live wire stream. Re-running on the same gcode mid-print
  is not a supported scenario; pipeline is one-shot per file.
- **Receive-time on host** (Pi 5 class), single-threaded, f64-only.
- **Algebraic-closure pipeline.** Layer 1 emits NURBS in their natural
  parameter `u` (chord-length-parameterized for fits, knot-domain for arc
  reductions). Layer 0 provides arc-length tooling (`ArcLengthTable`) for
  downstream consumers; Layer 2 invokes it on segments it receives,
  controlling its own `s`-discretization for TOPP-RA. Layer 3 reparameterizes
  to time. **Layer 1 does no parameterization beyond what the fitter
  naturally produces.**
- **Per-axis kinematic decoupling.** Z and E have their own speed/accel
  limits. The data model preserves per-coordinate addressability — XYZ in a
  single `VectorNurbs<f64, 3>`, E as a separate parallel scalar NURBS — so
  Layer 2 can extract per-axis dynamics without Layer 1 baking in any
  isotropic assumption.
- **Telemetry as a first-class subsystem**, but layer-internal events go
  through layer-defined sink traits, not a shared dependency. Each layer
  defines its observability event types and exposes a sink trait; the
  binary wires up real implementations (same pattern as `tracing`).

## 2. Crate layout

Two new workspace members alongside the existing `nurbs/` and
`nurbs-c-api/`:

```
rust/
├── nurbs/             [existing] Layer 0 substrate
├── nurbs-c-api/       [existing] C ABI for Klipper integration
├── gcode/             [NEW] G-code parser, no NURBS dep
└── geometry/          [NEW] Layer 1 pipeline: reduce, classify, fit,
                              corner_blend, segment types, telemetry sink
```

### 2.1 `rust/gcode/`

Pure text → typed-token state machine. **Zero dependency on `nurbs`.** Public
surface: a `Token` enum, a parser entry point returning an iterator, a
`ParseError` type, and slicer-dialect marker matchers. Reusable outside the
planner — fuzz target via `cargo fuzz`, candidate for klipper-sim/replay-tooling
consumption, candidate for eventual standalone publication.

The parser is **lexically G-code-aware but motion-semantics-agnostic.** It
knows line structure, comments, parameter words, line numbers, optional
checksums, and the slicer-comment dialect (recognizable `;LAYER:N` patterns
etc.). It does not know what G2 means. Motion-semantic interpretation is
`geometry::reduce`'s job.

`Cargo.toml`:
```toml
[dependencies]
thiserror = "2"
```

### 2.2 `rust/geometry/`

Layer 1 proper. Depends on `nurbs` (for `VectorNurbs<f64, 3>`,
`ScalarNurbs<f64>`, `KnotVector`, `BezierPiece`) and on `gcode` (for `Token`).
Owns reduce, classify, fit, corner-blend slot construction, the public
pipeline struct, segment types, telemetry sink, and the error taxonomy.

`Cargo.toml`:
```toml
[dependencies]
nurbs = { path = "../nurbs" }
gcode = { path = "../gcode" }
thiserror = "2"
```

Workspace lints inherited.

### 2.3 Future `rust/geometry-c-api/`

The workspace shape accommodates a future `rust/geometry-c-api/` parallel to
`nurbs-c-api/`, for exposing the pipeline through C FFI to Klipper or
standalone consumers. Not built today; layout reserves the slot.

### 2.4 What stays inside `geometry/`

Segment types live in `rust/geometry/` for now. Carving out a separate
`trajectory-types/` crate is deferred until Layer 2 has a concrete reason to
avoid the dep. YAGNI applies.

### 2.5 `thiserror` as a deliberate workspace choice

Both new crates use `thiserror` for error enums. `gcode/` heads toward ~10–15
ParseError variants long-term; `geometry/` has Recovery (~10) + Fatal
(small) + internal-construction errors. Hand-rolling Display+Error for ~30
variants across the workspace buys nothing. MSRV is fine — workspace pins
Rust 1.85; thiserror 2.x's MSRV is 1.61.

## 3. Public API surface

### 3.1 Top-level `geometry/` re-exports

```rust
pub use crate::pipeline::{GeometryPipeline, Segments, Item};
pub use crate::error::{Recovery, SlotDegeneracy, Fatal, InternalKind};
pub use crate::segment::{
    Segment, FittedSegment, ArcSegment, CornerBlendSlot,
    JunctionDeviation, BlendFamily, SourceRange,
};
pub use crate::telemetry::TelemetryEvent;
pub use crate::params::FitterParams;
```

### 3.2 The pipeline

```rust
pub struct GeometryPipeline {
    params: FitterParams,
    // (no scratch buffers for now — YAGNI; can amortize across a future
    //  `reset()` if reuse becomes a thing)
}

impl GeometryPipeline {
    pub fn new(params: FitterParams) -> Self;

    /// Process a complete G-code buffer. Returns a borrowing iterator over
    /// the segment stream. Sink receives observability events synchronously
    /// during processing.
    ///
    /// One-shot per file by convention; calling `process` more than once on
    /// the same instance is allowed but provides no state-reset guarantees
    /// between calls (each call reinitializes internal state).
    pub fn process<'a>(
        &'a mut self,
        text: &'a str,
        sink: &'a mut dyn FnMut(TelemetryEvent),
    ) -> Segments<'a>;
}

pub struct Segments<'a> {
    // Internal: parser cursor, modal state, accumulating polyline buffer,
    // pending-emit queue (small VecDeque<Item>).
    // Borrows pipeline + text + sink.
}

impl<'a> Iterator for Segments<'a> {
    type Item = Item;
    fn next(&mut self) -> Option<Item>;
}

#[non_exhaustive]
pub enum Item {
    Segment(Segment),
    Recovered(Segment, Recovery),
    Fatal(Fatal),
}
```

### 3.3 Three deliberate design choices

**1. `&mut self` on `process`, not consuming `self`.** One-shot semantics
by convention, but if scratch-buffer amortization becomes worth it later, no
API break needed.

**2. Sink as `&mut dyn FnMut(TelemetryEvent)`, not a trait.** The geometry
crate isn't in the business of telling the sink to flush or check levels —
those are the consumer's concerns and naturally belong on the consumer's
side. Closure form saves the `NoopSink` type and per-consumer `impl
TelemetrySink for MyCounter` boilerplate, and composes naturally with
captured-state closures (counter, Vec, logger) — the common case — where
a trait would force the consumer to invent a struct. If we ever genuinely
need a multi-method observability interface, adding it as a separate
parameter is non-breaking.

**Trade acknowledged.** A consumer wanting to fan events to multiple sinks
(counter + logger) writes the fan-out themselves rather than registering two
trait impls. This is composition vs. registration, and we're choosing
composition; consumers needing fan-out can wrap with one closure that
forwards to multiple captured sinks.

**3. Sink is non-optional.** Forces consumers to make an explicit "I don't
want telemetry" choice (`&mut |_| {}`) rather than silently dropping
observability events. Same principle as `#[must_use]` on `Result`.

### 3.4 Iterator contract

After `Item::Fatal(_)` is yielded, the pipeline transitions to a terminal
state: subsequent `next()` calls return `None` **and emit no further sink
events**. Documented as part of the `Item::Fatal` doc-comment:

> *Telemetry events emitted during the call to `next()` that produced this
> `Fatal` are still delivered to the sink before `Fatal` is returned.
> Consumers should drain the sink for diagnostic events on the failure path;
> telemetry is independent of the iterator's success state. After `Fatal`
> is yielded, no further sink events fire and `next()` returns `None`.*

The single unified `'a` lifetime is the simplest form that works for the
common case (consumer drives a pipeline against a buffer with a sink in the
same scope). If a real consumer hits a lifetime mismatch, generalize to
three independent lifetimes; for the common case the unified form is correct.

### 3.5 Stability commitment

`Iterator<Item = Item>` is **Layer 1's stable contract** for consumers.
Layer 2 (TOPP-RA) will buffer segments on top of this iterator for its own
lookahead window — composition, not API hedging. If Layer 2 ends up wanting
a richer shape, it wraps the iterator; Layer 1 doesn't change.

## 4. Segment types

The product of the iterator. Layer 2 reads these forever.

```rust
#[non_exhaustive]
pub enum Segment {
    Fitted(FittedSegment),
    Arc(ArcSegment),
    CornerBlend(CornerBlendSlot),
    Junction(JunctionDeviation),
}

pub struct FittedSegment {
    /// Geometric path in 3D Euclidean space. Built on Layer 0's
    /// `VectorNurbs` directly — no wrapper. Z is bundled (vase mode and
    /// non-planar printing just work); for layer-by-layer prints Z is
    /// constant within a run and the fitter represents that with equal Z
    /// across control points.
    ///
    /// **Parameterization invariant.** `xyz` is in its natural NURBS
    /// parameter `u` — chord-length-parameterized at fit time, monotone on
    /// `[0, 1]`. Layer 1 does **not** compute or attach arc-length
    /// reparameterization. Arc-length tooling lives in Layer 0
    /// (`ArcLengthTable`); Layer 2 invokes it on the segments it receives,
    /// controlling its own `s`-discretization for TOPP-RA.
    pub xyz: VectorNurbs<f64, 3>,

    /// Extruder profile parameterized over the same `u` parameter as `xyz`,
    /// with its own knot vector (E can have independent resolution; bundling
    /// E into a 4D NURBS would mix mm-of-travel with mm-of-filament in the
    /// chord-error metric, which is wrong).
    /// `None` for non-extruding runs (rare; usually G0 marker-breaks them).
    pub e: Option<ScalarNurbs<f64>>,

    /// Coalesce-min of all F-words seen during the run. Layer 2 uses as a
    /// velocity cap. Coalesce-min under-caps but never over-caps; TOPP-RA
    /// can always reduce velocity from the cap, but cannot recognize that
    /// the cap was set too high.
    ///
    /// **Measured on the OrcaSlicer arc-fitted corpus** (132k G1 lines,
    /// 4,309 distinct F values, 31,766 F-words): 49.3% of marker-bounded
    /// streaks have constant F (coalesce-min exact), 22.7% have 2 F values,
    /// 30.0% have 3+. After classifier-driven smooth-run segmentation
    /// (median run = 3 vertices per spike findings), within-run F-changes
    /// are bounded by 2-3 in the worst case. Coalesce-min systematically
    /// under-caps on variable-speed-extrusion sections (PA-aware
    /// outer-wall speed modulation) by an estimated 5-15% on those
    /// sections; net print-time impact estimated 2-5% on representative
    /// prints — material but not catastrophic.
    ///
    /// **Generalization trigger.** Generalize to
    /// `Vec<(s_position, feedrate_mm_s)>` — *not* `Vec<(line_no, ...)>`,
    /// since by the time the run is fitted, source line structure is gone
    /// and the transition is naturally placed at an arc-length position
    /// along the curve — when **either**:
    /// - (a) end-to-end print-time vs target throughput on a representative
    ///   print exceeds 5% slowdown attributable to feedrate coalescing, **or**
    /// - (b) Layer 2 reports velocity-cap-only constraints producing
    ///   noticeable corner-velocity artifacts that per-vertex feedrate
    ///   would resolve.
    pub feedrate_mm_s: f64,

    /// Fitter degree used for this segment (1 for very-short runs, 3 by
    /// default). Degree selection is a decision, not a failure — segment
    /// metadata, not Recovery.
    pub degree: u8,

    /// Maximum chord-error of the fit (mm). Quality metric for consumer
    /// telemetry; available on the happy path. Per the design's category
    /// distinction: Recovery is for anomalies, metadata is for measurements.
    pub max_residual_mm: f64,

    pub source: SourceRange,
}

pub struct ArcSegment {
    /// Exact rational quadratic NURBS in 3D. Helical G2/G3 (with Z-delta) is
    /// fully representable — Z is linearly interpolated across control
    /// points; same weights as the 2D arc. No Recovery for Z-varying arcs.
    /// Layer 0's `VectorNurbs<f64, 3>` supports weights for the rational
    /// quadratic encoding.
    pub xyz: VectorNurbs<f64, 3>,

    /// Extruder profile (`None` for non-extruding arcs).
    pub e: Option<ScalarNurbs<f64>>,

    pub feedrate_mm_s: f64,
    pub source: SourceRange,  // single line for arcs
}

pub struct CornerBlendSlot {
    /// 3D corner position.
    pub position: [f64; 3],

    /// 3D unit tangents (3D-normalized). Layer 3 uses directly to
    /// reconstruct blend geometry. Classifier uses 3D tangents internally
    /// for the angle threshold; XY-projection would mis-classify Z-doubling-
    /// back paths as smooth, which is the opposite of safety.
    pub t_in: [f64; 3],
    pub t_out: [f64; 3],

    /// Available segment lengths for control-point placement.
    pub seg_len_in: f64,
    pub seg_len_out: f64,

    /// Deviation budget — Layer 3 must produce a curve within this distance
    /// of the corner vertex.
    pub tolerance_budget_mm: f64,

    /// Hint to Layer 3. Default `CubicBezier`; Layer 3 may override per
    /// dynamic-limit shape selection (Tajima & Sencer 2016).
    pub default_family: BlendFamily,

    pub feedrate_mm_s: f64,
    pub source: SourceRange,
}

pub struct JunctionDeviation {
    pub position: [f64; 3],
    pub angle_deg: f64,
    pub feedrate_mm_s: f64,
    pub source: SourceRange,
}

/// Discriminated future-extension point. Layer 3 will iterate on blend
/// shapes as throughput tuning at high accel matures. Candidate future
/// variants: `Clothoid` (constant-jerk traversal), `QuinticBezier`
/// (C³ continuity at the seam), `Composite` (clothoid–arc–clothoid,
/// jerk-bounded with arc center). `#[non_exhaustive]` makes extension
/// non-breaking.
#[non_exhaustive]
pub enum BlendFamily {
    CubicBezier,
}

pub struct SourceRange {
    pub start_line: u32,
    pub end_line: u32,  // inclusive; equal to start_line for single-line segments
}
```

### 4.1 Design notes

1. **No newtype wrappers around Layer 0 types.** `FittedSegment::xyz` is a
   `VectorNurbs<f64, 3>` directly. Wrappers cost a conversion barrier and
   buy nothing.
2. **E is `Option<ScalarNurbs<f64>>`.** Naturally encodes "this is a travel
   run, no extrusion" without a sentinel value.
3. **`feedrate_mm_s` per segment**, coalesced via min over the run's F-words.
4. **`max_residual_mm` and `degree` always present** on `FittedSegment` —
   measurements, not anomalies.
5. **`SourceRange::end_line` is inclusive.** Matches Rust's `..=` and
   matches user expectation when saying "lines 100 to 120."
6. **3D tangents in `CornerBlendSlot`.** Drop XY-projection assumption.
7. **`#[non_exhaustive]` on `Segment`** — future variants (a hypothetical
   `SplineExtrusion`, or other curve representations) shouldn't break
   consumers' match arms.
8. **3D tangents in `CornerBlendSlot` are an architectural commitment, not
   a vase-mode special case.** No special-casing of XY-only data lets the
   classifier behave consistently across layer-by-layer prints (where the
   Z-component is tiny except at z-hops, which are marker-broken anyway),
   vase mode (continuous Z), and any future non-planar printing — the same
   classifier code, the same data shape. The vase-mode safety story is a
   downstream consequence of the consistency commitment, not its driver.

## 5. Telemetry events

The closure sink (`&mut dyn FnMut(TelemetryEvent)`) receives
observability-pure events. Six variants, all by-value (no borrows in the
event), `#[non_exhaustive]`:

```rust
#[non_exhaustive]
pub enum TelemetryEvent {
    /// Slicer comment indicated a layer change (`;LAYER:5` or equivalent).
    /// Detected by the parser's slicer-dialect matcher; consumed by reduce
    /// and forwarded as this event. Pluggable per-slicer matchers TBD if
    /// F0 corpora demand it; default set covers OrcaSlicer / PrusaSlicer /
    /// BambuStudio.
    LayerChange { layer: u32, line_no: u32 },

    /// T-word changed the active tool.
    ToolChange { tool: u32, line_no: u32 },

    /// E-only G1 detected — typically retraction or unretraction.
    /// `e_delta_mm` is signed (negative = retract).
    Retraction { e_delta_mm: f64, line_no: u32 },

    /// Smooth run force-flushed because it hit `max_window_vertices`.
    /// Spike: rare on slicer-G1 (max observed ~214 vertices); vase-mode
    /// safety guard.
    WindowFlush { run_vertex_count: u32, line_no: u32 },

    /// Dual-emitted alongside `Item::Recovered`. Recovery itself carries
    /// source info, so no separate `line_no` field.
    Recovery(Recovery),

    /// Per-FittedSegment quality observation, fired on the happy path
    /// (and within Recovery-wrapped emissions). Enables histogram-building
    /// without re-iterating segments.
    FitObservation {
        residual_mm: f64,
        tolerance_mm: f64,
        run_vertex_count: u32,
        piece_count: u32,
        degree: u8,
    },
}
```

### 5.1 Dual-emit ordering contract

Sink events fire synchronously **at processing time** — when the pipeline
determines the corresponding Item, not when the iterator yields it.

For Items emerging in batches (a fit may produce a `FittedSegment` plus a
`CornerBlendSlot` in the same processing step), **all sink events for the
batch fire before any Item in the batch is yielded.** Within a batch, sink
event order matches the Item yield order. Consumers correlating both
channels can rely on per-Item ordering.

Three observable points (processing → queue → yield), not two.

### 5.2 Three deliberate design choices

1. **No lifecycle events** (`Started`, `FinishedOk`, `FinishedFatal`). The
   consumer brackets their own iteration; introducing pipeline-emitted
   lifecycle events tangles drop semantics with telemetry semantics.
2. **`Recovery` is dual-emitted** (iterator + sink). Consumers driving with
   a closure shouldn't need to also walk the segment stream to count
   anomalies; consumers pattern-matching items shouldn't need to also wire
   telemetry routing. Both call sites are cheap.
3. **`FitObservation` fires per fit, not per "interesting" fit.** Letting
   the consumer decide what's interesting (via filtering inside the
   closure) is cheaper and more flexible than baking a notion of
   "interesting" into the geometry crate.

### 5.3 What's deliberately not in `TelemetryEvent`

- **Per-vertex classifier labels.** Heavy (millions of events for a print),
  and aggregable post-hoc from segment-stream structure.
- **Per-segment "this is a normal Fitted" notification.** Redundant with
  the iterator.
- **Modal-state shifts (G92, G53, G54).** F0 treats G92 as a marker break
  and ignores other modal-state codes; if those become consequential, add
  a `ModalShift` event later.

## 6. Configuration

```rust
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct FitterParams {
    // --- Classifier (CMLT) ---
    pub theta_smooth_deg: f64,         // default 15.0
    pub theta_hard_deg: f64,            // default 60.0
    pub seg_len_collapse_mm: f64,       // default 0.05 — collapse adjacent
                                        //   corners separated by very-short
                                        //   segments

    // --- Fitter (LSPIA + chord-bound) ---
    pub degree: u8,                     // default 3
    /// Initial interior-knot count for LSPIA. Capped per-run inside
    /// `fit.rs` to `max(0, vertex_count - degree - 1)` to avoid the
    /// under-determination bug surfaced by the spike (8-CP fits on 3-vertex
    /// runs produced control points many orders of magnitude away from the
    /// input). The cap is enforced by:
    /// - `debug_assert!(effective_n_init <= vertex_count.saturating_sub(degree as u32 + 1))`
    ///   in `fit_smooth_run`.
    /// - A T2 unit test (`test_n_init_cap_short_run`) constructing a
    ///   3-vertex run with `n_init_interior = 4` and asserting the
    ///   effective LSPIA system is well-determined and produces residuals
    ///   below `eps_chord_mm`.
    pub n_init_interior: u32,            // default 4
    pub eps_chord_mm: f64,              // default 0.025
    pub eps_iter_mm: f64,               // default 1e-9
    pub max_lspia_iter: u32,            // default 100
    pub max_refine_iter: u32,           // default 20
    pub n_chord_samples: u32,           // default 50 — sample-based chord-bound;
                                        //   replace with analytical convex-hull
                                        //   bound (see §6.3 trigger)

    // --- Lookahead window ---
    pub max_window_vertices: u32,       // default 64 — vase-mode safety guard;
                                        //   never observed binding on slicer-G1

    // --- Corner blend ---
    pub blend_tolerance_mm: f64,        // default 0.050 — 2× eps_chord,
                                        //   half-margin convention
}

impl Default for FitterParams { /* spike-validated defaults above */ }
```

### 6.1 Construction

All fields `pub`, `Copy + Clone + Debug + PartialEq`. **Production
construction uses `Default` + struct-update syntax:**

```rust
let params = FitterParams {
    theta_hard_deg: 45.0,
    ..Default::default()
};
```

Full struct literals are for tests/dev only — they break on field additions;
struct-update with `Default` survives them.

`FitterParams` is **intentionally NOT `#[non_exhaustive]`** because consumers
genuinely want to set fields. The `Default + ..Default::default()` idiom is
the right pattern for this stability profile.

### 6.2 Validation

`GeometryPipeline::new(params)` runs `debug_assert!`-level invariant checks
(`degree >= 1 && degree <= 5`, `0 < theta_smooth_deg < theta_hard_deg < 180`,
`eps_chord_mm > 0`, `max_window_vertices >= degree as u32 + 2`).

In release builds, bad params degrade naturally — LSPIA may emit
`Recovery::LspiaNotConverged`, fitter may emit `Recovery::ToleranceExceeded`,
etc. Production-grade `Result`-returning validation is deferred to F0+1 if
it becomes worth the API cost.

Defaults are known-valid by construction; the `debug_assert` catches misuse
during development. The natural-degradation path actually produces useful
telemetry events that diagnose the misuse.

### 6.3 F0+1 trigger: analytical chord-bound

Replace the `n_chord_samples` sample-based chord-bound with the analytical
convex-hull bound when **either**:

- (a) corpus measurements show sample-based bound under-estimates true chord
  error by > 5% on realistic input, **or**
- (b) sample-based evaluation becomes a measurable fraction (>10%) of fit
  time on benchmarks.

Either trigger fires; otherwise sample-based is fine. The trigger discipline
matters more than the version target — without a trigger, F0+1 is
aspirational.

### 6.4 Config-file format

**Not Layer 1's job.** CLAUDE.md flags configuration as cross-cutting; the
eventual config crate (TOML, JSON, or whatever lands) reads its file format
and produces a `FitterParams`. `geometry/` exposes the struct, nothing more.

### 6.5 `Send + Sync` and concurrency model

`FitterParams` is `Send + Sync` trivially via `Copy`. The pipeline is
single-threaded **per file**: a `GeometryPipeline` instance plus its
`Segments<'_>` iterator are not `Sync`, and the lifetime-bound borrow of
pipeline + text + sink prevents concurrent access from another thread.

**Multiple files in parallel** is supported the simple way: clone
`FitterParams` (cheap, `Copy`), construct two `GeometryPipeline`s, process
on two threads. The crate doesn't manage threading; it provides
thread-locality through ownership.

## 7. Internal module layout

### 7.1 `rust/gcode/`

```
src/
├── lib.rs        — re-exports, top-level docs
├── lexer.rs      — &str → Iterator<Item = Result<Token, ParseError>>
├── token.rs      — Token, MarkerKind, Params (indexed [Option<f64>; 26])
├── marker.rs     — slicer-dialect comment-pattern matchers
└── error.rs      — ParseError variants (thiserror)
fuzz/
└── fuzz_targets/lex.rs   — cargo-fuzz target on raw &[u8] → tokenize
tests/
└── golden_corpus_lex.rs  — tokenize the OrcaSlicer corpus, snapshot test
```

Token model:

```rust
pub enum Token {
    Command { letter: u8, major: u32, minor: Option<u32>,  // G5.1 → (5, Some(1))
              params: Params, line_no: u32 },
    Comment { text: SmallString, line_no: u32 },             // unrecognized
    Marker  { kind: MarkerKind, line_no: u32 },              // recognized dialect
}

pub struct Params {
    words: [Option<f64>; 26],  // indexed by letter - b'A';
                                // O(1) accessors x()/y()/z()/e()/f()/i()/j()/r()/p()/q()
}

#[non_exhaustive]
pub enum MarkerKind {
    LayerChange { layer: u32 },
    LayerType   { name: SmallString },     // ;TYPE:WALL-OUTER, ;TYPE:INFILL, etc.
    EndOfPrint,
}
```

`SmallString` is an inline-or-heap representation (TBD: hand-rolled or via
e.g. `compact_str`; deferred to implementation phase, not architectural).

### 7.2 `rust/geometry/`

```
src/
├── lib.rs           — re-exports, top-level docs
├── pipeline.rs      — GeometryPipeline, Segments<'_>, Item, drives the others
├── reduce.rs        — Token stream → (Polyline | ArcDescriptor | MotionMarker) events
├── classify.rs      — Polyline → Vec<VertexLabel> using full 3D tangents
├── fit.rs           — fit_smooth_run(&[Vec3], &[Option<f64>], &FitterParams) -> FitOutcome
├── corner_blend.rs  — make_slot(prev, corner, next, &FitterParams) -> Result<CornerBlendSlot, SlotError>
├── segment.rs       — segment types
├── telemetry.rs     — TelemetryEvent
├── params.rs        — FitterParams + Default
└── error.rs         — Recovery, SlotDegeneracy, Fatal, InternalKind
tests/
├── integration_orca.rs       — corpus run, segment-count sanity checks
├── synthetic_unit_circle.rs  — known curve recovery tests
├── helical_arc_3d.rs         — Z-delta G2/G3 → 3D rational quadratic invariant
├── degenerate_inputs.rs      — degenerate slot, single-vertex run, etc.
├── vase_mode_smoke.rs        — synthesized vase-mode → classifier on 3D tangents
└── cross_check_python.rs     — vs prototype on a fixed seed corpus
```

### 7.3 Module privacy as stability commitment

`pub` on `pipeline`, `segment`, `telemetry`, `params`, `error`. **`pub(crate)`
on `reduce`, `classify`, `fit`, `corner_blend`** — deliberate stability
mechanism: `pub` is committed-stable to external consumers, `pub(crate)` is
freely-refactorable. Exposing a piece externally is a deliberate decision
(public re-export), not a leak.

**`reduce`'s internal event types** (`Polyline`, `ArcDescriptor`,
`MotionMarker`) are `pub(crate)` and exist solely as the internal interface
between `reduce` and `pipeline`. Tests of `reduce` operate at this boundary
(`reduce::reduce_tokens(tokens) -> Vec<ReduceEvent>` is exposed to the
crate's `tests/` module via `#[cfg(test)] pub use`); external consumers do
not see these types and may not depend on them.

### 7.4 Dataflow

```
&str text
   │
   ▼
gcode::lexer            → Iterator<Item = Result<Token, ParseError>>
   │
   ▼
geometry::reduce        → modal-state machine, emits internal events:
   │                       Polyline { points: Vec<Vec3>, e: Vec<Option<f64>>, f: f64 }
   │                       ArcDescriptor { start, end, center, ccw, e_delta, z_delta, f }
   │                       MotionMarker { kind: ZOnly | EOnly | G0 | G92 | M | T | EndOfFile }
   │                       (also forwards gcode::Token::Marker to the sink as
   │                        TelemetryEvent::LayerChange / LayerType / etc.)
   │
   ▼
geometry::pipeline      → consumes events, dispatches:
   │                       Polyline → classify → split into smooth-runs →
   │                                  fit each → emit FittedSegment(s) +
   │                                  CornerBlendSlot(s) + JunctionDeviation(s)
   │                       ArcDescriptor → emit ArcSegment (3D rational quadratic)
   │                       MotionMarker → flush state, emit telemetry
   │
   ▼
Iterator<Item = Item>
```

### 7.5 Internal queue discipline

The smooth-run fit happens once the run terminator is known. The pipeline's
`next()` may consume many tokens before producing one segment (whole run-up
to next corner/marker), then queue several segments at once (fit + slot, or
fit + junction). Internal queue is a small `VecDeque<Item>`; pipeline yields
from the queue first, advances the parser when empty.

**Bounded queue size.** Worst case ~3 items in practice (fit + trailing
slot + queued recovery for a prior-but-deferred run). Implementation
`debug_assert!`s queue length never exceeds a small bound (e.g. 8). If the
queue grows unbounded, that's a bug worth catching at the assertion.

## 8. Error model

```rust
#[non_exhaustive]
pub enum Recovery {
    /// Parser saw a G/M/T head not in the modeled set; reduce treats as marker.
    UnrecognizedCommand { line_no: u32, head: String },

    /// Parameter parse failed for a recognized motion command.
    MalformedParams { line_no: u32, raw: String },

    /// Smooth run hit `max_window_vertices` cap; force-emitted as a fit at
    /// the cap, next run starts at the cap vertex.
    WindowCapHit { source: SourceRange, run_vertex_count: u32 },

    /// Smoothable corner couldn't be parameterized — falls back to
    /// `JunctionDeviation`.
    DegenerateSlotFallback { line_no: u32, reason: SlotDegeneracy },

    /// Chord-bound refinement exhausted `max_refine_iter` without reaching
    /// `eps_chord_mm`. Curve emitted with surfaced residual.
    ToleranceExceeded { source: SourceRange, actual_mm: f64, budget_mm: f64 },

    /// LSPIA exhausted `max_lspia_iter` without reaching `eps_iter_mm`.
    /// Curve emitted with last-update magnitude surfaced.
    LspiaNotConverged { source: SourceRange, last_update_mm: f64 },
}

#[non_exhaustive]
pub enum SlotDegeneracy {
    BacktrackingCorner,     // t_in · t_out below -threshold (near 180°)
    ZeroIncidentLength,     // |t_in| or |t_out| below numerical floor
    ColinearTangents,       // t_in × t_out below numerical floor
}

/// Fatal payload — NOT #[non_exhaustive]; consumers must handle every
/// variant. Different stability posture from Recovery: Fatal additions are
/// version-bumping changes.
pub enum Fatal {
    /// Internal invariant violation — surfaced rather than panicking so the
    /// consumer can release resources, log diagnostics, decide whether to
    /// abort the print or just stop the pipeline cleanly.
    Internal(Box<InternalDetails>),  // boxed: keeps Item compact on hot path
}

pub struct InternalDetails {
    pub kind: InternalKind,
    pub context: String,
    pub backtrace: std::backtrace::Backtrace,  // captured at construction
}

/// Closed enum — variants here represent invariants we've identified.
/// Adding a variant is a deliberate version bump.
pub enum InternalKind {
    NonMonotoneKnotVector,
    NaNDetected { stage: &'static str },
    KnotInsertionFailed,
    BasisMatrixSingular,
    DegreeOutOfBounds,
}
```

### 8.1 Recovery doc-header

> *`Recovery` is for anomalies — "tried X, X didn't work, here's the
> fallback I took instead." Per-segment quality measurements (residual,
> degree, tolerance utilization) live on segment metadata, not Recovery.
> Adversarial or corrupted input may produce Recovery events at line-rate
> (thousands per second); sink implementations that accumulate or forward
> should consider rate-limiting or sampling.*

### 8.2 Stability posture summary

| Type             | `#[non_exhaustive]` | Rationale                                      |
|------------------|---------------------|------------------------------------------------|
| `Item`           | yes                 | Future variant additions shouldn't break match |
| `Segment`        | yes                 | Future segment kinds shouldn't break consumers |
| `Recovery`       | yes                 | Fitter is high-risk; new modes will appear     |
| `SlotDegeneracy` | yes                 | New degeneracy modes are non-breaking          |
| `MarkerKind`     | yes                 | New slicer-dialect markers will be recognized  |
| `BlendFamily`    | yes                 | Layer 3 will iterate on blend shapes           |
| `TelemetryEvent` | yes                 | Observability event types grow over time       |
| `Fatal`          | **no**              | Consumers must handle every case               |
| `InternalKind`   | **no** (closed)     | Invariants are version-bumping additions       |
| `FitterParams`   | **no**              | Consumers genuinely want to set fields         |

### 8.3 Backtrace capture

`std::backtrace::Backtrace::capture()` at the construction site of
`Fatal::Internal`. Stable since Rust 1.65; well within MSRV 1.85.
`RUST_BACKTRACE=1` enables; off by default, so production builds don't pay
until a fatal fires. `Backtrace` is large-ish and `!Copy`; the
`Box<InternalDetails>` keeps `Item`'s stack size compact for the iterator's
hot path.

### 8.4 Removed / deferred

- **`DegradedDegree`** (was Recovery) — moved to `FittedSegment::degree`
  metadata. Degree selection is a decision, not a failure.
- **`FitQuality`** (was Recovery) — `max_residual_mm` is segment metadata.
  Quality measurements aren't anomalies.
- **`Fatal::EncodingError`** — not reserved. "Fatal additions are version
  bumps" is the policy; following it for the case that actually arises is
  cleaner than carrying speculative reservations forever.

### 8.5 Panic policy

`Item::Fatal` rather than `panic!` for invariant violations is the right
shape because:

- A panic in a streaming pipeline is hostile to the consumer (`catch_unwind`
  is gross; UnwindSafe is its own can of worms).
- Invariant violations are likely diagnosable, not unrecoverable; structured
  error capture is a better debugging experience.
- The line between "internal invariant violation" and "subtle malformed
  input we didn't anticipate" is usually invisible at the panic site.
  `Fatal` gives flexibility about classifying things later; panic locks the
  classification at first trigger, often poorly.

`panic!` / `unreachable!()` / `debug_assert!` remain appropriate for truly
impossible states (post-exhaustive-match unreachables, memory-corruption
indicators), not "geometry pipeline produced something it shouldn't have."

## 9. Phasing

The spec covers all of Layer 1; implementation lands in two phases mirroring
CLAUDE.md's build-order steps 6 and 7.

### 9.0 Phase 1 viability calculation

A back-of-envelope sanity check that "every G1 vertex becomes a
JunctionDeviation + degree-1 segment" is tractable for Layer 2:

- Spike corpus: ~196k segments per print (straight-line) / ~140k per print
  (arc-fitted). A real 100MB print is ~20× the corpus → **3–8M segments**.
- Planning happens at receive time, not bulk: 1000 mm/s × 0.5mm/move ≈
  **2000 moves/sec → 500 µs per-move budget**.
- TOPP-RA on degree-1 segments + junction-deviation cornering is
  ~10–50 floating-point operations per segment; even at 1 µs per segment
  with cache-cold lookups, **comfortably within budget** (1–5 µs vs 500 µs).

Phase 1 is a viable MVP, not just a correctness scaffold. The throughput
deficit comes from corner-velocity-reduction at every junction, not from
planner compute.

### 9.1 Phase 1 — Foundations (build-order step 6)

Ships into the MVP first-print:

- `rust/gcode/` complete: lexer, token model, marker matchers, ParseError
  taxonomy, fuzz target.
- `rust/geometry/` minimal: `pipeline`, `segment`, `reduce`, `params`,
  `error`, `telemetry`.
- Reduce emits **degree-1 NURBS for each G1 segment** (no fitting) and
  **`JunctionDeviation` at every G1 vertex** (no classifier, no
  smoothability test).
- G2/G3 → full 3D rational quadratic `ArcSegment` (helical-capable from day
  one).
- Telemetry minimum: `LayerChange`, `ToolChange`, `Retraction`. No
  `FitObservation`, no `WindowFlush` (no fitter, no window).
- `FitterParams` exists with full default; phase-1 only consults the
  classifier-related and arc-related fields.

This drives MVP. Output is verbose-but-correct: lots of degree-1 segments,
every corner is a junction-deviation, no fitting. CLAUDE.md is explicit:
corner velocities will be conservative, but parts print.

**Performance characteristics at Phase 1 mirror existing junction-deviation
planners (regular-Klipper-class). The throughput benefits of NURBS-internal
planning materialize at Phase 2 when smooth runs collapse to fitted
segments. Phase 1's purpose is correctness validation across the full
pipeline; performance is not a Phase 1 goal.**

### 9.2 Phase 2 — Fitter and classifier (build-order step 7)

Brings Layer 1 to feature complete:

- `geometry::classify` — full 3D-tangent CMLT classifier with the spike's
  measured thresholds.
- `geometry::fit` — LSPIA + sample-based chord-bound, with the spike's
  `n_init_interior` per-run cap.
- `geometry::corner_blend::make_slot` — parameterized cubic-Bezier slot.
- Full `Recovery` taxonomy active. Full `TelemetryEvent` surface active.

During Phase 2 (and through build-order step 8 when Layer 3 lands),
`CornerBlendSlot` is emitted but Layer 2 treats it equivalently to
`JunctionDeviation`. That's Layer 2's concern; Layer 1's API is stable
across the transition.

### 9.3 Phase 3 — Hardening

Not part of F0 ship-readiness:

- Analytical convex-hull chord-bound (replaces sample-based; triggered per
  §6.3 conditions).
- Helical-arc-3d test corpus + vase-mode model corpus.
- BambuStudio + PrusaSlicer corpus expansion (per spike's open question).
- Fuzz battery on `gcode/` integrated into CI.

### 9.4 Phase boundary contract

The public API of `geometry/` is identical across phases. Phase 1 produces a
subset of segment kinds (no Fitted-with-degree>1, no `CornerBlendSlot`);
Phase 2 expands the produced set. Consumers handle the full enum from day
one — no API churn at the phase boundary.

## 10. Testing strategy

Six tiers, in roughly increasing cost / required-toolchain order:

### T1 — Unit tests, `gcode/`

- Per-Token-kind round-trip on synthesized G-code lines.
- Edge cases: comments before/after commands, missing parameters, integer
  vs decimal heads (G2 vs G5.1), multiple slicer-comment dialects,
  modal-state-bare-param lines.
- Property tests via `proptest`: arbitrary parameter words, arbitrary line
  orderings, arbitrary whitespace; assert tokenizer never panics, always
  terminates.

### T2 — Unit tests, `geometry/` modules

- `reduce`: Token sequences → expected internal events; modal-state
  correctness; G92 marker treatment.
- `classify`: synthetic polylines with known angles; threshold transitions;
  degenerate inputs (zero-length segments, colinear).
- `fit`: known curves (unit circle, published Bezier, sinusoid sample) →
  recovery within tolerance, piece count near analytical minimum.
- `corner_blend::make_slot`: tangent extraction, length computation,
  degeneracy detection (each `SlotDegeneracy` variant has a constructed-
  input test).

### T3 — Anchor tests

Small, fast, end-to-end on canonical inputs. Run with every `cargo test`.

- `helical_arc_3d.rs` — synthesize G2 with Z-delta, verify `ArcSegment` 3D
  control points match analytical helix. Locks the full-3D commitment as a
  tested invariant.
- `degenerate_inputs.rs` — single-vertex polyline, zero-length segment,
  exact 180° corner, run hitting `max_window_vertices`. One test per
  Recovery variant.
- `vase_mode_smoke.rs` — synthesized vase-mode input (continuous Z-progress
  within a polyline). Smoke-tests the classifier's full-3D-tangent decision.

### T4 — Integration tests, OrcaSlicer corpus

- Run pipeline on `voron_cube_arc_fitted.gcode` and
  `voron_cube_straight_line.gcode`.
- Assert: all vertices classified, all smooth runs fit within
  `eps_chord_mm`, all G2/G3 produce `ArcSegment`s, no crashes, output
  JSON-deserializable, segment counts within ±5% of measured spike numbers
  (after the under-determination fix).
- Telemetry assertions: `FitObservation` count == `FittedSegment` count;
  `LayerChange` count matches expected layer count from the gcode.

### T5 — Cross-check against Python prototype

Run Rust pipeline + Python prototype on a fixed-seed corpus (the existing
`scripts/fitter_prototype/corpus/voron_cube_*`). Compare outputs.

Tolerances per quantity:

- **Classifier labels:** bit-exact on inputs constructed away from
  threshold boundaries (synthetic tests choose angles far from 15° and
  60°). Corpus-based tests treat "label difference only on near-threshold
  inputs" as expected.
- **Piece counts per fit:** **exact ±1**, with telemetry diff'd to identify
  disagreement points and **the direction of the discrepancy recorded
  per-fit** (Rust higher / Python higher). Consistent directional bias
  (e.g. Rust always +1) indicates a systematic divergence in the LSPIA
  convergence criterion or chord-bound discretization worth investigating;
  symmetric ±1 is benign last-bit noise. ±1 reflects iterative-refinement
  last-bit divergence between scipy and our LSPIA, not a bug. Diffs > 1 are
  real divergences worth investigating.
- **Control points:** equivalent within `1e-6 mm`. Cross-language
  bit-exactness for iterative numerical algorithms is rarely achievable;
  asserting it produces flaky tests.
- **`max_residual_mm`:** equivalent within `1e-9 mm`.

This is the prototype-as-oracle pattern from the spike. Lives in
`tests/cross_check_python.rs`, gated behind a `--features python-cross-check`
flag (requires Python toolchain).

**Framing:** the cross-check's value is "does our Rust port match our Python
prototype on the same algorithm," not "do we agree with off-the-shelf
libraries solving a different problem." `scipy.splprep` and `ArcWelder` fail
in characteristic ways — `splprep` doesn't enforce chord-error tolerance
directly; `ArcWelder` is biased toward arc fitting. Reproducing their
failure modes isn't useful, and asserting they match would either gate our
fitter on bugs they have or require constant carve-outs as their behavior
changes. They are explicitly out of scope.

### T6 — Fuzz, `gcode/`

- `cargo-fuzz` target on `&[u8] → tokenize`. Run nightly in CI; treat any
  panic or hang as P0.
- Corpus seeded from OrcaSlicer corpus (legitimate input baseline) plus
  synthesized adversarial: bit-flipped lines, truncated lines, deeply-
  nested comments, malicious unicode in comments.

### 10.1 Test corpus versioning

Both T4 (OrcaSlicer corpus) and T5 (cross-check) reference corpus files in
`scripts/fitter_prototype/corpus/`. As OrcaSlicer evolves, real-world
output may diverge from the committed corpus. Lock the corpus version with
a `README.md` in the corpus directory naming, per file:

- Slicer name and version (e.g. "OrcaSlicer 2.3.2")
- Source model
- Profile / settings used to generate
- Generation date

A future maintainer regenerating the corpus knows what they're comparing
against; a future maintainer noticing a behavioral change can localize
"is this a slicer-output change or a fitter-behavior change?"

### 10.2 Out of scope for F0

- Performance benchmarks (revisit at the future `geometry-c-api/` boundary,
  when Klipper integration is real and end-to-end latency is measurable).
- Full corpus expansion (BambuStudio, PrusaSlicer; deferred per spike's
  open question).
- Comparison against `scipy.splprep` / ArcWelder.

## 11. Alternatives considered and rejected

A short non-exhaustive list of decisions where the obvious-looking
alternative was actively rejected, captured so future contributors know the
choices were deliberate.

**Async pipeline / `tokio::mpsc`-based.** Rejected. Pipeline is one-shot,
CPU-bound, and operates on a complete in-memory buffer. Async machinery
adds runtime dependency, tangles drop semantics, and saves nothing for a
host-side single-thread workload.

**Trait objects over enum (`Box<dyn TrajectorySegment>` instead of
`enum Segment`).** Rejected. Enum gives exhaustiveness checking on match
arms, monomorphization for variant-specific code in Layer 2, and better
cache behavior (control points and metadata co-located with the variant
discriminant). Trait objects would force vtable indirection for every
geometric operation Layer 2 performs on a stream of millions of segments.
The flexibility of trait objects (polymorphism via dyn) doesn't earn its
cost when the variant set is small and stable.

**One mega-crate or three+ crates instead of `gcode/` + `geometry/`.**
Rejected. One mega-crate (`planner/`) couples Layer 2's iteration speed to
Layer 1's stability through the same compilation unit; module boundaries are
weaker contracts than crate boundaries. Three+ crates (e.g. splitting
`geometry/` further into `reduce/` + `fit/` + `corner-blend/`) is YAGNI —
they're tightly coupled, no second consumer exists, premature splitting is
its own pain. Two crates is the pragmatic minimum that earns its keep
through `gcode/`'s reusability and fuzz-target isolation.

**Trait-based `TelemetrySink`.** Rejected in favor of `&mut dyn FnMut`. See
§3.3.2 for the rationale and trade.

**`Result<Self, ParamsError>` on `GeometryPipeline::new`.** Rejected in
favor of `debug_assert!`-level validation with natural release-mode
degradation through Recovery. See §6.2.

**Per-vertex feedrate in F0 (`Vec<(s_position, feedrate_mm_s)>`).** Deferred
to F0+1 with a measured, concrete trigger. See `FittedSegment::feedrate_mm_s`
doc-comment.

## 12. Layer 2 evolution policy

`Iterator<Item = Item>` is Layer 1's stable contract; Layer 2 builds on top.
If Layer 2 wants something Layer 1 didn't expose:

- **Layer 1 grows.** New segment metadata, new telemetry events, new
  Recovery variants — all additive on `#[non_exhaustive]` types.
- **Layer 2 does not reach into `pub(crate)` modules** via feature flags or
  side channels. The privacy boundary is a stability commitment.
- **The iterator contract does not get hedged.** No "this is a placeholder
  API"; the iterator is the API.

Future contributors faced with "Layer 2 needs X" should propose extending
Layer 1's public surface, not bypassing the privacy boundary.

## 13. Open questions for implementation

Tracked as design-time TBDs to resolve during implementation, not blockers
on the spec:

- **`SmallString` representation** (hand-rolled, `compact_str`, `Box<str>`,
  or `String`) — implementation choice in `gcode::token`. Default: start
  with `String`; optimize if profiling shows allocator pressure.
- **Per-slicer marker matcher pluggability** — F0 hardcodes the
  Orca/Prusa/Bambu pattern set; deciding whether to expose a public
  registration API is deferred until a real consumer asks.
- **Pipeline `reset()` for cross-file reuse** — not needed for F0
  (one-shot-per-file is sufficient); add if amortizing scratch buffers ever
  matters.
- **Three independent lifetimes on `process`** — single `'a` is the simplest
  form that works; generalize only if a real consumer hits a mismatch.

## 14. Cross-cutting follow-up

Worth promoting to a CLAUDE.md addendum after this spec lands: the **"each
layer defines its event types and a sink trait, the binary wires real sinks
at the top"** pattern is general (Layer 2's TOPP-RA telemetry and Layer 3's
shaping-process telemetry will follow the same shape). Capturing as a
follow-up rather than blocking on it.

## 15. References

### 15.1 Internal

- Spike findings:
  `docs/superpowers/spikes/2026-04-26-layer-1-fitter-spike.md`
- Python prototype design:
  `docs/superpowers/specs/2026-04-26-layer-1-fitter-prototype-design.md`
- Layer 0 NURBS evaluation library:
  `docs/superpowers/specs/2026-04-26-nurbs-evaluation-library-design.md`
- Layer 0 NURBS algebra:
  `docs/superpowers/specs/2026-04-26-nurbs-algebra-design.md`
- High-level architecture and build order: `CLAUDE.md`

### 15.2 External (algorithm provenance)

Full bibliography lives in the spike findings (§9 of
`2026-04-26-layer-1-fitter-spike.md`). Load-bearing for this spec:

- Bi, Huang, Lu, Zhu, Ding (2019). "A general, fast and robust B-spline
  fitting scheme for micro-line tool path under chord error constraint."
  *Sci. China Tech. Sci.* 62. — LSPIA + chord-bound refinement.
- Sun, Yu, Wang, Xie (2018). "A smooth tool path generation and real-time
  interpolation algorithm based on B-spline curves." *Adv. Mech. Eng.* 10.
  — CMLT classifier.
- Tajima, Sencer (2016). "Kinematic corner smoothing for high-speed machine
  tools." *Int. J. Mach. Tools Manuf.* 108. — Layer 3 dynamic-limit-aware
  shape selection.
- Pateloup, Duc, Ray (2004). "Corner optimization for pocket machining."
  *Int. J. Mach. Tools Manuf.* — Cubic Bezier corner placeholder default.
