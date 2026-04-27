# Layer 1 — G5 / G5.1 Reduction Design

**Date:** 2026-04-27
**Status:** Spec — design under brainstorm review; implementation plan to follow on green-light
**Layer:** 1 (Geometry pipeline)
**Driver:** Build-order Step 3 — close the remaining gap in geometric reduction so Layer 1 produces exact non-rational NURBS for G5 (degree-3, 4 control points) and G5.1 (degree-2, 3 control points) per the LinuxCNC RS274NGC convention, with the modal-chain implicit-tangent rule for G5 and active-plane validation for G5.1.

## 1. Context

The Layer 1 Phase-1 foundations (`docs/superpowers/specs/2026-04-26-layer-1-rust-architecture-design.md` + `docs/superpowers/plans/2026-04-26-layer-1-phase-1-foundations.md`) shipped a working geometry pipeline: `gcode/` lexer (already tokenizes G5 / G5.1 with `minor: Option<u32>`), `geometry/reduce` modal-state machine, `geometry/pipeline` segment-emission iterator, full Recovery / Telemetry surface. **Reduce currently silently drops G5 / G5.1.** Step 3 closes that gap.

This is a *small follow-up* to Step 2 in build-order terms — no new crates, no new public API surface beyond two `Recovery` variants. The work is mostly in `rust/geometry/src/{reduce.rs, pipeline.rs}` plus a tightly-scoped extension to `error.rs` and one new integration-test file.

**What's genuinely new** in this spec:

1. The G-code semantics for G5 / G5.1 under the LinuxCNC convention (verified during brainstorming against Marlin / RRF / grblHAL / Fanuc — LinuxCNC is the only meaningful spec).
2. The RS274NGC §3.5.5 modal-chain implicit-tangent rule for G5 — a piece of modal state (`prev_g5_pq: Option<[f64; 2]>`) and a clearing discipline tied to motion-producing G-codes.
3. Minimal active-plane tracking (`Plane` enum + `active_plane: Plane` in `ModalState`), wired to G17 / G18 / G19, used to validate G5.1 invocation.
4. A refactor of the internal `pub(crate) ReduceEvent` enum from per-source-G-code variants (`G1Move`, `Arc`) to a unified `Curve(CurveGeom, …)` shape with fixed-size-array variants. This is internal — no external API impact — but governs how G5 / G5.1 are introduced (as new `CurveGeom` variants alongside the migrated existing ones).
5. The reduce stage now constructs the rational-quadratic Bézier control points for arcs (G2 / G3) inline; pipeline consumes NURBS-form data, not center-form descriptions. The arc-NURBS construction helper that currently lives in `pipeline.rs` moves to `reduce.rs` as part of the refactor.

**What this spec does not re-litigate:**

- Algorithms or kinematic limits (no Layer 2 / Layer 3 work here).
- Crate layout, public API shape, error model envelope, telemetry sink pattern (all settled by `2026-04-26-layer-1-rust-architecture-design.md`).
- The lexer's tokenization of G5 / G5.1 (already shipped in Phase 1, Task 6).

### 1.1 Non-goals

- **Plane-aware G2 / G3.** Phase 1 implements G2 / G3 as XY-only regardless of `active_plane`. Re-gating arcs by plane is a deliberate non-goal of this step — it would touch existing arc tests with no relation to G5 / G5.1. Plane support for arcs is appropriate for a future "RS274NGC plane completeness" item if the user wants it.
- **G5 / G5.1 in non-XY planes for G5.1.** LinuxCNC § G5.1 is plane-restricted by spec; we accept G5.1 only in G17 (XY). G18 / G19 invocations of G5.1 emit `Recovery::G5PlaneMismatch`. (G5 itself is XY-only by spec — there is no G5-equivalent in XZ / YZ.)
- **Fitter-output corner blends.** Step 8 territory. G5 endpoints break the G1-tangent chain; Layer 2 evaluates curvature at endpoints from the NURBS itself (per the curvature-continuity principle codified in `CLAUDE.md` 2026-04-27).
- **G5.2 / G5.3** (LinuxCNC's NURBS-block control-point streaming). Step 8-or-later territory; the user has noted future plans for a G5-emitting slicer but the wire format is undefined.
- **Real-world G5 corpus integration.** No slicer or common CAM tool emits G5 today (PrusaSlicer / Orca / Bambu / Cura linearize splines or arc-fit; Fusion / Vectric / Carbide don't emit G5 in stock posts; user plans to write a G5-emitting slicer later, out of scope here). Synthetic-only test corpus.
- **Pipeline performance benchmarking.** Same disposition as the prior spec §1.1 — revisit at the `geometry-c-api/` boundary when end-to-end latency is measurable.

### 1.2 Driving constraints (inherited and reaffirmed)

- **Offline-file premise.** One-shot per file; modal state initializes on `process()`.
- **Receive-time on host** (Pi 5 class), single-threaded, f64-only. Reduce + pipeline see G5 / G5.1 lines at slicer-emit rates if they ever appear (thousands per second worst case for hypothetical future slicer output) — fixed-size arrays and zero-allocation per segment matter.
- **Algebraic-closure pipeline.** G5 / G5.1 are exact non-rational Béziers; the pipeline emits NURBS in their natural Bézier parameter `u ∈ [0, 1]` with clamped knot vectors. No reparameterization.
- **Curvature-continuity-based junction handling** (CLAUDE.md 2026-04-27). Layer 1's job is to preserve geometry exactly; Layer 2 derives end-tangents and end-curvatures from each segment's NURBS at the junction parameter `u = 1` of segment N and `u = 0` of segment N+1. Layer 1 must therefore *break* the G1-tangent chain at any non-G1 segment endpoint — including G5 and G5.1 — because the only correct end-tangent at a NURBS endpoint is the NURBS's own derivative there, computed in Layer 2.

## 2. G-code semantics

### 2.1 LinuxCNC RS274NGC G5 (degree-3, cubic Bézier)

Source: LinuxCNC G-code reference, RS274NGC §3.5.5 (`G5 X… Y… I… J… P… Q…`).

**Control points** (3D, with Z linearly interpolated across the four CPs at parameter values 0, ⅓, ⅔, 1):

```
P0 = (current.x,        current.y,        current.z + 0)
P1 = (current.x + I,    current.y + J,    current.z + dz/3)
P2 = (end.x + P,        end.y + Q,        current.z + 2·dz/3)
P3 = (end.x,            end.y,            end.z)            // = current.z + dz
```

where `dz = end.z − current.z`. The XY geometry is exactly the planar cubic Bézier; Z is linear in the parameter.

**Knot vector** (clamped, single piece): `[0, 0, 0, 0, 1, 1, 1, 1]`.

**Weights:** none — non-rational. (The `nurbs::VectorNurbs::try_new` `weights` argument is `None`.)

**Required and optional parameters:**

- `X, Y` — modal: inherit from current position if absent.
- `Z` — modal: inherit from current position if absent (no implicit Z change).
- `I, J` — XY tangent offset at start. **Required**, with one exception: the modal-chain rule (§2.3) lets a G5 omit both I and J when it immediately follows another G5 — they default to `−(prev P, prev Q)` for C¹ continuity. Single I or single J specified is invalid.
- `P, Q` — XY tangent offset at end. **Always required and explicit on every G5.** No defaulting. Both required, single P or single Q invalid.
- `F` — feedrate, modal across the gcode stream.
- `E` — extruder, modal.

**Plane:** G5 is XY-only by spec. There is no defined behavior in G18 / G19 for G5; we treat G18 / G19 as "no-op for the G5 plane check" — G5 always uses XY tangent offsets regardless of `active_plane`. (G5.1 is plane-restricted; G5 is not, per LinuxCNC's distinct treatment.)

**Source [KNOWLEDGE — research-confirmed]:** LinuxCNC G-code reference (LinuxCNC §G5); user-confirmed in brainstorming Q1 round 1.

### 2.2 LinuxCNC G5.1 (degree-2, quadratic Bézier — non-rational)

Source: LinuxCNC G-code reference, § G5.1 (`G5.1 X… Y… I… J…`).

**Control points** (3D, with Z linearly interpolated across the three CPs at parameter values 0, ½, 1):

```
P0 = (current.x,        current.y,        current.z)
P1 = (current.x + I,    current.y + J,    current.z + dz/2)
P2 = (end.x,            end.y,            end.z)
```

**Knot vector** (clamped, single piece): `[0, 0, 0, 1, 1, 1]`.

**Weights:** none — non-rational. **This is the load-bearing distinction from G2 / G3 arcs** (which are rational quadratic with `weights = [1, cos(half_sweep), 1]`). At the `Segment::Fitted` level, G5.1 has `xyz.weights() == None` and degree 2; G2 / G3 land in `Segment::Arc` with `xyz.weights() == Some(_)`.

**Required parameters:**

- `X, Y, Z` — modal as for G5.
- `I, J` — both required and explicit. No modal chain (LinuxCNC § G5.1 specifies no implicit-tangent rule for G5.1; the modal-chain rule is G5-only). At least one of I, J must be non-zero — both zero collapses the curve to a degenerate line, which is rejected as malformed.
- `F, E` — modal.

**Plane:** G5.1 is restricted to the active plane per § G5.1. Phase 1 supports only G17 (XY); G5.1 issued while `active_plane != XY` emits `Recovery::G5PlaneMismatch { line_no, active_plane_g_code }` and is dropped (no segment emitted).

**Source [KNOWLEDGE — research-confirmed]:** LinuxCNC G-code reference (LinuxCNC § G5.1); cross-verified during brainstorming that Marlin doesn't implement G5.1, RRF doesn't implement G5 / G5.1, grblHAL matches LinuxCNC, Fanuc's `G05.1 Q1` is an unrelated AICC mode toggle. **LinuxCNC is the only meaningful spec for G5 / G5.1 in the open-source space.** User-confirmed in brainstorming Q1 round 1.

### 2.3 RS274NGC §3.5.5 G5 modal-chain implicit-tangent rule

When a G5 immediately follows another G5 with both I and J omitted, default to:

```
I := −prev_P
J := −prev_Q
```

This is the C¹ continuity rule: the next segment's start tangent at P0 is the *mirror* of the previous segment's end tangent at P3 across the seam, so that the cubic Bézier basis-function derivatives at the seam match in direction and magnitude.

**Implementation discipline:**

- Modal state grows one slot: `prev_g5_pq: Option<[f64; 2]>`. Initialized `None`. Set to `Some([P, Q])` at the end of every successful G5 emission. Cleared by every motion-producing g-code other than G5: G0, G1, G2, G3, G5.1.
- Non-motion-producing g-codes do **not** clear it: G10 (any L variant), G53, G54–G59.3, G17 / G18 / G19, M-codes, T-codes. They don't move the machine, so the previous G5 is still the *most recent motion*.
- G5 with both I and J omitted **and** `prev_g5_pq.is_none()` → `Recovery::G5MissingTangent { line_no }` is emitted; no segment is produced. This is the strict-error stance — fabricating a tangent (e.g. zero) masks bugs in upstream G-code.
- Single I OR single J specified on G5 → `Recovery::MalformedParams` (via `ParseErrorKind::G5MalformedTangent`).
- G5 missing P or Q (or both) → also `Recovery::MalformedParams`. **P and Q are never inferred** from any modal source.

**Why this is the right default for kalico:** Professional CAM output that emits G5 chains assumes the rule. The cost is one `Option<[f64; 2]>` slot in `ModalState`. Alternatives considered and rejected:

- *Silent default to zero on missing I, J.* Rejected — masks broken G-code; downstream curvature evaluation against a zero-tangent cubic produces a degenerate curve indistinguishable from a malformed input.
- *Require explicit I, J always.* Rejected — rejects RS274NGC-conformant input.

**Source [DIRECTION + RESEARCH]:** User confirmation (round 1 Q2) + LinuxCNC RS274NGC §3.5.5 + brainstorming-time research subagent report.

### 2.4 Z handling — linear-in-parameter across CPs

For G5 cubic, Z values at the four CPs are at parameter positions 0, ⅓, ⅔, 1 — i.e. evenly spaced, matching the *Greville abscissae* of a degree-3 Bézier on knots `[0,0,0,0,1,1,1,1]` for a non-rational Bézier where Z is the projection. Concretely:

```
P0.z = current.z
P1.z = current.z + (end.z − current.z) / 3
P2.z = current.z + 2·(end.z − current.z) / 3
P3.z = end.z
```

For G5.1 quadratic, Z values at the three CPs are at parameter positions 0, ½, 1 (Greville abscissae of a degree-2 Bézier on knots `[0,0,0,1,1,1]`):

```
P0.z = current.z
P1.z = (current.z + end.z) / 2
P2.z = end.z
```

This makes the resulting NURBS evaluate exactly to a *planar Bézier in XY × linear-in-time in Z* — the curve passes through any z-value strictly between current.z and end.z exactly once, monotonically, at the same parameter where the XY position is the planar Bézier's evaluation. This matches what G2 / G3 already do for helical arcs (`pipeline::build_arc_nurbs` interpolates Z at the midpoint of the three CPs; G5 / G5.1 follow the same convention generalized to four / three CPs respectively).

**Source [KNOWLEDGE]:** Standard Bézier-NURBS basis-function math; matches existing G2 / G3 helical-arc convention in `2026-04-26-layer-1-phase-1-foundations.md` (Task 20 verified by `g2_helical_yields_z_linear_control_points`).

### 2.5 Junction handling — break the G1-tangent chain

Both G5 and G5.1 set `prev_g1_end = None`, `prev_g1_dir = None`, `prev_g1_feedrate = None` after emitting their segment, identical to the existing G2 / G3 behavior. Consequence: a G1 immediately following a G5 or G5.1 produces no `JunctionDeviation` from Layer 1.

This is correct because Layer 2 will compute junction velocity from curvature on both sides of the boundary — `κ(u=1)` of the G5 / G5.1 segment and `κ(u=0)` of the next segment. The G5 / G5.1 NURBS carries the necessary information; fabricating a "virtual G1 direction" at a smooth-curve endpoint would be geometrically wrong.

**Source [DIRECTION + KNOWLEDGE]:** User direction round 1 Q3, codified in CLAUDE.md 2026-04-27 as the "Junction velocity from curvature continuity" Layer-2 principle: *"Implication for Layer 1: do not fabricate 'virtual G1 directions' at smooth-curve endpoints to feed JD — break the G1-tangent chain at any non-G1 segment, and let Layer 2 evaluate end-tangents and end-curvatures from the NURBS itself."*

## 3. Internal-API design

### 3.1 `CurveGeom` enum

A new `pub(crate)` inner enum in `rust/geometry/src/reduce.rs` carrying the geometry payload of a `ReduceEvent::Curve`:

```rust
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum CurveGeom {
    Linear            { cps: [[f64; 3]; 2] },                       // G0 / G1
    Quadratic         { cps: [[f64; 3]; 3] },                       // G5.1, non-rational
    RationalQuadratic { cps: [[f64; 3]; 3], weights: [f64; 3] },    // G2 / G3, rational
    Cubic             { cps: [[f64; 3]; 4] },                       // G5
}
```

Design choices:

- **Fixed-size arrays per variant.** Zero per-segment heap allocation; the type system enforces the correct CP count for each variant. No `Vec` in the hot path.
- **`Quadratic` and `RationalQuadratic` are distinct variants** — not a single `Quadratic { cps, weights: Option<[f64; 3]> }`. Two reasons: (a) the ontology is real — G5.1 (non-rational quadratic) and G2 / G3 (rational quadratic) are distinct curve-mathematical kinds; (b) consuming code that branches on rational vs non-rational doesn't have to inspect an Option, just exhaustive-match.
- **Source-G-code-driven, not mathematical-class-driven.** A variant per source g-code's natural curve form. Future `G5.2 / G5.3` (LinuxCNC's NURBS-block CP-streaming control-flow) would add a single `CurveGeom::Nurbs { cps: SmallVec<…>, weights: Option<…>, knots: SmallVec<…>, degree: u8 }` variant when implemented; existing variants and downstream code don't change.

**Source [DIRECTION]:** User decision in brainstorming round 1 Q5 (the `(a.i)` shape, with explicit `Quadratic` ≠ `RationalQuadratic` per the ontological argument).

### 3.2 `ReduceEvent::Curve(CurveGeom, …)` variant + migration

```rust
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum ReduceEvent {
    Curve {
        geom: CurveGeom,
        e_delta: Option<f64>,
        feedrate_mm_s: f64,
        line_no: u32,
    },
    Marker { /* unchanged */ kind: MotionMarkerKind, line_no: u32, tool: Option<u32>, e_delta_mm: Option<f64> },
    CommentMarker { /* unchanged */ kind: gcode::MarkerKind, line_no: u32 },
    ParseError { /* extended kind set */ line_no: u32, kind: ParseErrorKind, text: String },
    // existing G1Move / Arc removed after migration
}
```

Migration: existing `ReduceEvent::G1Move { from, to, … }` → `ReduceEvent::Curve { geom: CurveGeom::Linear { cps: [from, to] }, … }`. Existing `ReduceEvent::Arc { start, end, center, clockwise, …, z_delta }` → `ReduceEvent::Curve { geom: CurveGeom::RationalQuadratic { cps, weights }, … }` where `cps`, `weights` are computed by an arc-NURBS-construction helper that *moves from `pipeline::build_arc_nurbs` to `reduce::build_arc_curve`*.

**Why move arc NURBS construction into reduce?** Reduce's stable contract becomes "g-code → NURBS-form geometry"; pipeline consumes NURBS, not center-form descriptions. The split-of-responsibility today (reduce emits center form, pipeline reconstructs) is an accident of how G2 / G3 was implemented before the `Curve` shape existed; the `Curve(RationalQuadratic)` type forces the right factoring.

**Backward compatibility:** `ReduceEvent` is `pub(crate)`, per the prior architecture spec §7.3. No external consumers — the `gcode` integration tests don't depend on it. Refactor is internal-only.

**Source [DIRECTION + KNOWLEDGE]:** User decision Q5 + prior spec §7.3 (`pub(crate)` privacy boundary for `reduce::*` is freely refactorable).

### 3.3 `ParseErrorKind` extensions

Three new sub-kinds carried in `ReduceEvent::ParseError`:

```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ParseErrorKind {
    MalformedNumber,
    UnrecognizedHead,
    EmptyCommand,
    DuplicateParam,
    /// G5 line missing both I,J with no previous G5 in modal chain.
    G5MissingTangent,
    /// G5.1 line with the active plane != G17 (Phase 1: XY only).
    /// `text` carries the active plane G-code as a literal ("18" or "19").
    G5PlaneMismatch,
    /// G5/G5.1 with malformed I,J,P,Q (single I, single J, both zero on G5.1, etc.).
    G5MalformedTangent,
}
```

Pipeline maps each to a `Recovery` variant (§3.4).

### 3.4 `Recovery` extensions (the only public-API change)

Two new variants on the `pub enum Recovery` (still `#[non_exhaustive]`):

```rust
pub enum Recovery {
    // existing variants…
    /// G5 with both I,J omitted but no previous G5 in modal chain.
    /// Per RS274NGC §3.5.5 the implicit-tangent rule requires `prev_g5_pq`
    /// to be set; when it is not, we reject the line rather than fabricate
    /// a tangent.
    G5MissingTangent { line_no: u32 },
    /// G5.1 issued while the active plane is not the only supported plane
    /// (XY in Phase 1). The G-code number of the active plane is included
    /// (17 = XY, 18 = XZ, 19 = YZ).
    G5PlaneMismatch { line_no: u32, active_plane_g_code: u32 },
}
```

`G5MalformedTangent` does **not** get its own Recovery variant — it maps to the existing `Recovery::MalformedParams { line_no, raw }` with a descriptive `raw` string. Rationale: keeps the public Recovery surface minimal; consumers that want to special-case G5 malformations can grep `raw`. `MissingTangent` and `PlaneMismatch` are sufficiently distinct semantic categories to warrant their own variants (especially `MissingTangent`, which is *not* a malformed line — it's a perfectly-well-formed G5 that happens to be in a chain-broken context, a genuinely different category).

**Source [DIRECTION]:** Following the prior spec's stability posture (Recovery is `#[non_exhaustive]` — additions are non-breaking; new categories are appropriate when the consumer's response differs).

### 3.5 `ModalState` extensions

```rust
pub(crate) struct ModalState {
    pub position: [f64; 3],
    pub e: f64,
    pub feedrate_mm_s: Option<f64>,
    pub tool: u32,
    pub active_plane: Plane,         // NEW — default Plane::XY
    pub prev_g5_pq: Option<[f64; 2]>, // NEW — default None
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub(crate) enum Plane {
    #[default]
    XY,
    XZ,
    YZ,
}
```

`Plane` exists for G5.1 plane validation. G17 / G18 / G19 in the token stream update `active_plane` silently (no `ReduceEvent` emitted — these are configuration commands per RS274NGC §3.5.1, not motion).

`prev_g5_pq` exists for the G5 modal chain (§2.3). Cleared by motion-producing g-codes other than G5; preserved across plane selects, M-codes, T-codes. (Concretely in the implementation: every existing arm of `next_event` that produces a `ReduceEvent::Curve { geom: !Cubic, … }` sets `state.prev_g5_pq = None;` immediately before returning. The G5 arm sets it. The Marker arms — M, T, G92, EOnly, ZOnly — are mixed: G92 and ZOnly should clear it; M and T should preserve it. Decision below.)

**Clearing discipline — concrete table:**

| G-code | Motion? | Clears `prev_g5_pq`? | Rationale |
|---|---|---|---|
| G0 | Yes | Yes | Rapid is non-G5 motion |
| G1 (XY move) | Yes | Yes | Linear is non-G5 motion |
| G1 (Z-only marker) | Z-axis only | Yes | Z-only is still motion |
| G1 (E-only marker) | Filament only | Yes | E-only still alters extruder modal state, treat as non-G5 motion to be conservative |
| G2 / G3 | Yes | Yes | Arcs are non-G5 motion |
| G5 (success) | Yes | Yes — but immediately re-set to current `(P, Q)` | The chain links forward |
| G5 (error path) | No | Yes | Don't carry stale `(P, Q)` past an error |
| G5.1 | Yes | Yes | G5.1 is non-G5 motion (different curve order) |
| G17 / G18 / G19 | No | No | Plane select; configuration |
| G92 | No (sets origin) | Yes | G92 *redefines* current position; even though no machine motion, the chain semantics break because P0 of any subsequent G5 is now in a different frame |
| M-codes | No | No | Hardware control; no motion |
| T-codes | No | No | Tool change; no motion (tool offset is applied later) |
| Comment markers | No | No | Layer markers etc. |
| Parse errors | No | No | Leave the chain alone; the next valid G5 may continue it |

The G92 case is subtle. RS274NGC §3.5.5 doesn't explicitly address G92's interaction with the modal chain; conservative reading is "G92 changes the coordinate frame, so prev_g5_pq's components — which are deltas in the current frame — become semantically stale." Choose to clear. The risk of clearing when we shouldn't is one extra `Recovery::G5MissingTangent` on a hand-edited G5-after-G92 line; the risk of *not* clearing is silently fabricating a wrong tangent. Clear.

**Source [KNOWLEDGE — derived from spec + first principles]:** Behavior matrix is derived; user-confirmed in round 1 Q2 the broad shape ("any motion-producing g-code other than G5 clears it; configuration / hardware-control commands do not").

### 3.6 Pipeline `handle_event` refactor

Today's `handle_event` matches on `ReduceEvent::G1Move`, `ReduceEvent::Arc`, `ReduceEvent::Marker`, `ReduceEvent::CommentMarker`, `ReduceEvent::ParseError`. After the refactor, the curve-handling collapses into one arm dispatching on `CurveGeom`:

```rust
fn handle_event(&mut self, event: ReduceEvent) {
    match event {
        ReduceEvent::Curve { geom, e_delta: _, feedrate_mm_s, line_no } => {
            self.handle_curve(geom, feedrate_mm_s, line_no);
        }
        ReduceEvent::CommentMarker { … } => { /* unchanged */ }
        ReduceEvent::Marker { … } => { /* unchanged */ }
        ReduceEvent::ParseError { … } => { /* extended kind→Recovery mapping */ }
    }
}

fn handle_curve(&mut self, geom: CurveGeom, feedrate_mm_s: f64, line_no: u32) {
    match geom {
        CurveGeom::Linear { cps }            => /* JD-against-prev-G1 + emit FittedSegment(degree=1) */,
        CurveGeom::Quadratic { cps }         => /* emit FittedSegment(degree=2, weights=None); break G1 chain */,
        CurveGeom::RationalQuadratic { cps, weights } => /* emit ArcSegment(weights=Some); break G1 chain */,
        CurveGeom::Cubic { cps }             => /* emit FittedSegment(degree=3, weights=None); break G1 chain */,
    }
}
```

The `Linear` arm preserves the existing JD-against-previous-G1 behavior (§2.5 of `2026-04-26-layer-1-phase-1-foundations.md` Task 19); all other arms break the G1 chain.

### 3.7 `Segment::Fitted` reuse for G5 / G5.1

G5 and G5.1 emit `Segment::Fitted` (not `Segment::Arc`, not a new variant), with degree 3 and 2 respectively, `max_residual_mm = 0.0` (exact construction, no fit residual), and `e: None` (Phase 1 E-pipeline convention).

Why reuse `Fitted` rather than introduce `Segment::G5Bezier` / `Segment::CubicBezier` / similar:

- The prior architecture spec §4 documents `FittedSegment::degree: u8` precisely so segments of different geometric degrees coexist under one segment kind. Step 8's smooth-run fitter will emit degree 3 (default) or degree 1 (very-short runs); G5 also emits degree 3 — they're indistinguishable to Layer 2 because they're geometrically identical in form.
- `max_residual_mm = 0.0` accurately encodes "exact construction" — the same convention G1 uses today (`degree: 1, max_residual_mm: 0.0`).
- `Segment::Arc` is reserved for the rational-quadratic (weights-bearing) curve form. G5.1 is *non-rational quadratic* — sharing the variant would require teaching downstream consumers to inspect `weights()` to distinguish, which is exactly what the `Quadratic` / `RationalQuadratic` split in `CurveGeom` was introduced to avoid.

**Source [DIRECTION + KNOWLEDGE]:** Prior architecture spec §4; this spec §3.1.

## 4. Module-level changes

### 4.1 `rust/geometry/src/reduce.rs`

- Add `Plane` enum + `active_plane: Plane` field to `ModalState`. Wire G17 / G18 / G19 to update silently.
- Add `prev_g5_pq: Option<[f64; 2]>` field to `ModalState`. Clearing discipline per §3.5 table.
- Define `CurveGeom` (§3.1) and migrate `ReduceEvent` to use `Curve(CurveGeom, …)` (§3.2). Remove the legacy `G1Move` / `Arc` variants once production paths are migrated.
- Move `build_arc_nurbs`'s control-point math from `pipeline.rs` into a `reduce::build_arc_curve(start, end, center, clockwise) -> CurveGeom::RationalQuadratic` helper. Phase-1 limitations preserved (single-piece for sweeps `< π`, clamp at `π − ε` for larger sweeps; multi-piece is a future item).
- Add G5 reduction arm: explicit-IJ path, modal-chain implicit-IJ path, error paths for missing tangent / malformed tangent / missing P, Q.
- Add G5.1 reduction arm: plane check first, then explicit-IJ-or-error, then non-rational quadratic emission. Degenerate-input check (both I, J zero) → `G5MalformedTangent`.

### 4.2 `rust/geometry/src/pipeline.rs`

- Refactor `handle_event` per §3.6: one `Curve` arm dispatching on `CurveGeom`.
- Replace `degree_1_nurbs` with `nurbs_from_linear(cps: [[f64; 3]; 2])`; replace inline arc construction (now removed; `reduce::build_arc_curve` does the math) with a thin `nurbs_from_rational_quadratic(cps: [[f64; 3]; 3], weights: [f64; 3])`. Add `nurbs_from_quadratic(cps: [[f64; 3]; 3])` (degree-2 non-rational, knots `[0,0,0,1,1,1]`, weights `None`) and `nurbs_from_cubic(cps: [[f64; 3]; 4])` (degree-3 non-rational, knots `[0,0,0,0,1,1,1,1]`, weights `None`).
- Extend the `ReduceEvent::ParseError` arm to map the new sub-kinds to `Recovery::G5MissingTangent` / `Recovery::G5PlaneMismatch` / `Recovery::MalformedParams` (for `G5MalformedTangent`).

### 4.3 `rust/geometry/src/error.rs`

- Add `Recovery::G5MissingTangent { line_no: u32 }` and `Recovery::G5PlaneMismatch { line_no: u32, active_plane_g_code: u32 }` (§3.4). The enum's `#[non_exhaustive]` posture means consumers' match arms with `_ =>` continue to compile.

### 4.4 New file: `rust/geometry/tests/g5_reduction.rs`

End-to-end integration tests at the public API boundary (driving `GeometryPipeline::process` against synthetic G-code strings; black-box, no `pub(crate)` references). Coverage per §6.

### 4.5 Files NOT touched

- `rust/gcode/*` — lexer already tokenizes G5 / G5.1.
- `rust/geometry/src/{segment.rs, params.rs, telemetry.rs, lib.rs}` — `Segment::Fitted` already supports arbitrary degrees; no segment-type changes; no params changes; no new telemetry events.

## 5. Telemetry

**No new event types.** `Recovery` is dual-emitted via the existing `TelemetryEvent::Recovery(Recovery)` mechanism; the new `Recovery::G5MissingTangent` and `Recovery::G5PlaneMismatch` flow through that path. G5 / G5.1 emissions on the happy path produce `Segment::Fitted` items; no new `FitObservation`-class observability is needed (the curves are exactly constructed, not fitted — `max_residual_mm = 0.0` is the only quality metric and it's encoded on the segment).

This is a **deliberate non-addition.** The pattern in §5.3 of the prior architecture spec (events live where consumers benefit; redundancy with segment metadata is rejected) applies — there's nothing to observe at the G5 / G5.1 boundary that isn't already on the segment.

## 6. Validation rules and tests

Tests live in three layers, mirroring the prior spec's testing-tier discipline:

### 6.1 T1 — `gcode/` lexer

No new tests. Lexer's G5 / G5.1 tokenization is already covered by Phase 1 Task 6.

### 6.2 T2 — `geometry/` unit tests

Per-module tests inside `reduce.rs` and `pipeline.rs`. Coverage enumerated:

**Modal state:**
- `Plane` enum default = `XY`.
- `prev_g5_pq` initial value = `None`.
- G17 / G18 / G19 update `active_plane` silently (no event emitted).
- G17 / G18 / G19 do not clear `prev_g5_pq`.

**G5 explicit (no chain):**
- `G5 X10 Y0 I3 J3 P-3 Q3` from origin → `Curve(Cubic { cps: [(0,0,0), (3,3,0), (7,3,0), (10,0,0)] })`, feedrate carried, line number carried.
- G5 with Z delta → control-point Z values at thirds (0, dz/3, 2·dz/3, dz).

**G5 modal chain:**
- Three-G5 chain, second and third with implicit (no I, J) → P1 of each computed as `−(prev P, prev Q)`.
- G5 → G1 → G5(no IJ) → `Recovery::G5MissingTangent` on the last G5; G1 in the middle clears `prev_g5_pq`.
- G5 → G17 → G5(no IJ) → second G5 succeeds (plane select preserves chain).
- G5 → M104 → T0 → G5(no IJ) → second G5 succeeds (M / T preserve chain).
- G5 → G92 → G5(no IJ) → `Recovery::G5MissingTangent` (G92 clears chain — derived behavior per §3.5).

**G5 malformations:**
- G5 with single I or single J → `Recovery::MalformedParams` (mapped from `G5MalformedTangent`).
- G5 missing P → `Recovery::MalformedParams`.
- G5 missing Q → `Recovery::MalformedParams`.
- G5 missing both P and Q → `Recovery::MalformedParams`.

**G5.1 explicit:**
- `G5.1 X10 Y0 I3 J3` from origin → `Curve(Quadratic { cps: [(0,0,0), (3,3,0), (10,0,0)] })`.
- G5.1 with Z delta → control-point Z values at midpoint (0, dz/2, dz).

**G5.1 plane mismatch:**
- G18 then G5.1 → `Recovery::G5PlaneMismatch { active_plane_g_code: 18 }`.
- G19 then G5.1 → `Recovery::G5PlaneMismatch { active_plane_g_code: 19 }`.
- G18 then G17 then G5.1 → success (plane reset).

**G5.1 malformations:**
- G5.1 with single I → `Recovery::MalformedParams`.
- G5.1 with single J → `Recovery::MalformedParams`.
- G5.1 with both I and J zero → `Recovery::MalformedParams`.
- G5.1 with no I, J → `Recovery::MalformedParams`. (No modal-chain rule for G5.1.)

**Refactor regression (the legacy variants are gone):**
- All existing `g1_xy_emits_g1move`, `g2_emits_arc_clockwise`, `g3_emits_arc_counter_clockwise`, `modal_position_persists_across_g1s`, `g2_with_z_delta_yields_z_delta_field` tests rewritten to match against `ReduceEvent::Curve { geom: CurveGeom::Linear { … } }` / `CurveGeom::RationalQuadratic { … }` shapes. Numeric assertions unchanged.

### 6.3 T3 — Integration, end-to-end

In `rust/geometry/tests/g5_reduction.rs`. Driving `GeometryPipeline::process` against synthetic strings; only the public API surface visible:

- **`single_g5_emits_one_cubic_fitted_segment`** — G1 + G5 produces exactly one `Segment::Fitted { degree: 3, weights: None, max_residual_mm: 0.0 }`.
- **`single_g5_1_emits_one_quadratic_non_rational_fitted_segment`** — G1 + G5.1 produces exactly one `Segment::Fitted { degree: 2, weights: None, max_residual_mm: 0.0 }`. Crucial assertion: `xyz.weights().is_none()` (this is what distinguishes G5.1 from a rational-quadratic Arc in Layer 2's eyes).
- **`g5_chain_three_lines_no_junctions_between`** — Three-G5 chain produces three cubic Fitted segments and zero Junction segments. Locks the curvature-continuity break-the-chain principle.
- **`g5_followed_by_g1_breaks_chain_no_junction`** — G1 → G5 → G1 produces no `Junction` between the G5 and the trailing G1.
- **`g5_chain_break_then_implicit_tangent_emits_recovery`** — G1 → G5 → G1 → G5(no IJ) produces `Recovery::G5MissingTangent`. Both via `Item::Recovered` *and* via the sink (dual-emit per `2026-04-26-layer-1-rust-architecture-design.md` §5.1).
- **`g5_1_outside_g17_plane_emits_recovery`** — G18 → G5.1 produces `Recovery::G5PlaneMismatch { active_plane_g_code: 18 }`.
- **`g5_with_z_motion_interpolates_z_at_thirds`** — G5 with `Z=0.3` from `Z=0` produces CP Z values `[0, 0.1, 0.2, 0.3]`.
- **`g5_1_with_z_motion_interpolates_z_at_midpoint`** — G5.1 with `Z=0.4` from `Z=0` produces CP Z values `[0, 0.2, 0.4]`.
- **`g5_chain_preserved_by_m_codes_and_t_codes`** — G5 → M104 → T0 → G5(no IJ) produces two cubic segments, no Recovery.

### 6.4 T4 — Cross-firmware corpus

**Out of scope.** No real-world G5 corpus exists today; deferred to future G5-emitting-slicer work (per round 1 Q4).

### 6.5 Acceptance criterion for the spec

- All T2 / T3 tests pass.
- `cargo test --workspace --manifest-path rust/Cargo.toml` passes.
- `cargo clippy --workspace --manifest-path rust/Cargo.toml --all-targets -- -D warnings` passes.
- Concretely: `G5 X10 Y0 I3 J3 P-3 Q3` after `G1 X0 Y0` produces `Segment::Fitted { degree: 3, max_residual_mm: 0.0 }` with control points `[(0,0,0), (3,3,0), (7,3,0), (10,0,0)]` and knot vector `[0,0,0,0,1,1,1,1]`, no `Junction` after.
- `G5.1 X10 Y0 I3 J3` after `G1 X0 Y0` produces `Segment::Fitted { degree: 2, max_residual_mm: 0.0 }` with control points `[(0,0,0), (3,3,0), (10,0,0)]`, knot vector `[0,0,0,1,1,1]`, `weights().is_none()`.

## 7. Risks and mitigations

CLAUDE.md does not flag G5 reduction as a high-risk item — Step 3 is explicitly described as a *small follow-up to Step 2*. Three minor risks specific to this work:

1. **G5 modal-chain semantics rarely tested in practice.** Almost no existing G-code emitting tool uses G5 chains — the rule may have firmware-divergent interpretations in obscure CAM exports. *Mitigation:* synthetic-only test corpus per §6.2; the LinuxCNC RS274NGC text is the canonical reference and we follow it exactly. If a future real-world corpus surfaces incompatibilities, the rule is encapsulated in one arm of `next_event` and is straightforward to adjust.

2. **Latent bug surface: G5 error paths must clear `prev_g5_pq`.** A G5 line that fails validation (missing P, Q, single I, etc.) returns `ReduceEvent::ParseError` *before* the success-path `state.prev_g5_pq = Some([pp, qq])` runs. A subsequent G5 in chain would then implicit-tangent against stale state. *Mitigation:* every G5-arm error-return path explicitly sets `state.prev_g5_pq = None` immediately before returning. T2 tests include a regression case (G5(success) → G5(missing P, error) → G5(no IJ) → must produce `G5MissingTangent`, not silently link to the first G5).

3. **`f64::midpoint` MSRV.** Used in existing `pipeline::build_arc_nurbs` (Z-midpoint for helical arcs). The G5.1 Z-midpoint computation reuses it. `f64::midpoint` stabilized in Rust 1.85; workspace MSRV is 1.85 per the prior spec §2.5. *Mitigation:* none required, just noting that the helper continues to be available.

## 8. Open questions deferred to implementation

These are design-time TBDs to resolve during implementation, not blockers on the spec:

- **`VectorNurbs::knots()` accessor.** The integration tests in §6.3 reference `xyz.knots()` to assert exact knot-vector contents. If the Layer 0 public API exposes the knot vector under a different name (`knot_vector()`, `knots_slice()`, etc.), the assertion adapts to the actual API; if no public accessor exists, the literal-vector check is dropped (degree + control points + weights are fully sufficient to characterize the curve, given the reduce-stage construction is deterministic). *Trigger to revisit:* during plan-Task implementation, run `grep -n 'knots\|knot_vector' rust/nurbs/src/lib.rs` and adapt accordingly.

- **`Token::Marker` line numbering on comment-only lines.** §6.3's `g5_1_outside_g17_plane_emits_recovery` test assumes `G18\nG5.1 …\n` produces line 1 = G18, line 2 = G5.1. *Trigger to revisit:* at test-write time, verify the lexer's line-number convention (1-indexed) by reading `gcode::lex` output for a two-line input; trivial confirmation.

- **G5 immediately after G92 — clear `prev_g5_pq` or preserve?** Spec §3.5 chooses to clear (conservative). RS274NGC §3.5.5 doesn't address it explicitly. *Trigger to revisit:* if real-world G-code surfaces post-G92 G5 chains expecting the rule to survive, revisit. Until then, clear.

- **Plan-task granularity for the `ReduceEvent::Curve` refactor.** The implementation plan should split the refactor into commit-sized steps (define `CurveGeom` → add `Curve` variant → migrate G1 → migrate G2 / G3 → migrate pipeline → delete legacy variants), each green-tested at the workspace level. *Trigger to revisit:* during plan writing — not a spec concern.

## 9. Alternatives considered and rejected

**Marlin-faithful G5.1 (degenerate cubic with implicit P, Q = 0).** Rejected. Marlin doesn't actually implement G5.1; the "degenerate cubic" interpretation appears in some community discussions but produces a curve with a degenerate / undefined end tangent (P2 = P3), which is geometrically wrong and would break Layer 2's curvature-at-endpoint computation. LinuxCNC's non-rational-quadratic interpretation is the canonical spec.

**Single `Quadratic` `CurveGeom` variant with `weights: Option<[f64; 3]>`.** Rejected (per round 1 Q5). G5.1 (non-rational) and G2 / G3 (rational) are distinct curve-mathematical kinds; folding them together via `Option` muddles the ontology and forces every consumer that branches on rationality to inspect an Option rather than exhaustive-match. The variant cost is free.

**Vec-of-control-points `CurveGeom::Curve { cps: Vec<[f64; 3]> }`.** Rejected (per round 1 Q5 research findings). Allocator pressure at slicer-emit rates; LinuxCNC's `TC_STRUCT` precedent uses fixed-size geometry per variant; the unification value is already realized at `VectorNurbs<f64, 3>` one level down (where `FittedSegment` / `ArcSegment` already share the type). Pushing it up to `ReduceEvent` adds cost without value.

**New `Segment::G5Bezier` / `Segment::CubicBezier` variant on the public `Segment` enum.** Rejected. `FittedSegment::degree: u8` was *designed* for this — Step 8's smooth-run fitter will emit degree-3 fits indistinguishable from G5 emissions. A new `Segment` variant would require Layer 2 to handle two cases for what is geometrically the same thing.

**Silent default to zero on missing G5 I, J when chain is broken.** Rejected (per round 1 Q2). Masks broken G-code; produces a degenerate curve.

**Require explicit I, J on every G5 (no modal chain).** Rejected (per round 1 Q2). Rejects RS274NGC-conformant input from CAM tools that emit G5 chains.

**Plane-restrict G2 / G3 in this step too.** Rejected as out-of-scope expansion. The user explicitly scoped Step 3 as a small follow-up to Step 2; touching G2 / G3 plane handling is a separate item.

**New `TelemetryEvent` variant for G5 emissions.** Rejected. No observability gap — `max_residual_mm = 0.0` on `Segment::Fitted` already encodes the "exact construction" quality metric, and Recovery dual-emission handles the error paths.

## 10. References

### 10.1 Internal

- Layer 1 architecture: `docs/superpowers/specs/2026-04-26-layer-1-rust-architecture-design.md`
- Phase 1 implementation plan: `docs/superpowers/plans/2026-04-26-layer-1-phase-1-foundations.md`
- Build order and architectural principles: `CLAUDE.md`
  - **Layer 2 — "Junction velocity from curvature continuity" bullet** (added 2026-04-27): governs §2.5 of this spec.
  - **Build-order Step 3** (rewritten 2026-04-27): the build-order item this spec implements.
  - **Plan changes log 2026-04-27**: documents the rewrites this spec depends on.
- Layer 0 NURBS evaluation library: `docs/superpowers/specs/2026-04-26-nurbs-evaluation-library-design.md`

### 10.2 External (g-code semantics provenance)

- LinuxCNC G-code reference, **§G5** (cubic non-rational Bézier with I, J, P, Q) and **§G5.1** (quadratic non-rational Bézier with I, J, plane-restricted). Authoritative spec for kalico's G5 / G5.1 handling.
- LinuxCNC RS274NGC §3.5.5 "Cubic Spline" — the modal-chain implicit-tangent rule for G5.
- LinuxCNC `TC_STRUCT` (trajectory-controller queue element) — ontological precedent for fixed-size-geometry-per-variant tagged-union form, motivating §3.1's `CurveGeom` enum shape.

### 10.3 Cross-firmware confirmation

- **Marlin:** does not implement G5.1; G5 implementation matches the cubic Bézier form but has no modal chain.
- **RepRapFirmware:** does not implement G5 or G5.1.
- **grblHAL:** matches LinuxCNC for both G5 and G5.1.
- **Fanuc:** `G05.1 Q1` is an unrelated AICC mode toggle on a colliding number; not a curve command.

Established during round 1 Q1 research; documented in CLAUDE.md plan-changes log 2026-04-27.
