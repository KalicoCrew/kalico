---
title: Step 7-pre — Layer 0/1 prep for the cubic-Bézier-only live pipeline
date: 2026-04-29
status: design
supersedes: none
related:
  - docs/superpowers/plan-changes-log.md (2026-04-29 entries — round 1 + round 2)
  - CLAUDE.md (G5-only feature scope, Layer 1, build-order Step 7 / Step 13)
  - docs/research/bspline-polynomial-convolution.md
  - docs/research/layer3-time-polynomial-fit-bounds.md
  - docs/research/single-polynomial-fit-per-segment-conditioning.md
---

# Step 7-pre — Layer 0/1 prep for the cubic-Bézier-only live pipeline

## 1. Overview

Step 7-A (the Layer-3 minimum that lands smooth-ZV/MZV pre-bake + β-medium shaper-aware TOPP-RA + math-exact T-A time-reparameterization + E-follows-shaped-XY) sits on top of two new Layer-0/Layer-1 primitives and a Layer-1 structural refactor that the existing codebase does not yet provide:

1. **A polynomial composition primitive** (`nurbs::algebra::compose_vector_piece`) that does math-exact polynomial-of-polynomial substitution on Bézier pieces. T-A's per-TOPP-RA-grid-piece work is a cubic-Bézier-of-quadratic-Bézier composition; without this primitive, T-A would re-implement composition inline in Layer 3.
2. **A gcode-side N≤25 segment splitter** (`geometry::pipeline::split_segment_to_cap`) that bounds per-MCU-segment piece count by capping path length at 12.5 mm. Without this, a long G5-collinear (post-compat-layer) input from a legacy slicer could produce a NURBS with 200 grid pieces, blowing the bumped-but-still-bounded curve-pool slot budget on the H723.
3. **A Layer-1 structural refactor** that the round-2 brainstorm locked in: `FittedSegment` renamed to `CubicSegment` with a single-piece-cubic-Bézier invariant; G0/G1/G2/G3 reduce paths retired from the live pipeline (those move to Step 13's compat layer); `Segment::Arc` variant retired; `SplitInfo` metadata added for sub-segment provenance.

This spec captures the design of the three items together because they're tightly coupled: the splitter operates on the post-rename `CubicSegment`, the compose primitive's caller pattern depends on the splitter's output structure, and all three exist to *prepare* the pipeline for 7-A's math.

## 2. Scope

### 2.1 In scope

- **Layer 0:** Single new primitive `compose_vector_piece` in `rust/nurbs/src/algebra.rs` (or a new `rust/nurbs/src/composition.rs` module) for polynomial Bézier-piece composition. Leverages existing `nurbs::bezier::BezierPiece` infrastructure.
- **Layer 1:** Single new primitive `split_segment_to_cap` in `rust/geometry/src/pipeline.rs` (or a new `rust/geometry/src/splitter.rs` module) for path-length-capped subdivision of cubic segments.
- **Layer 1 structural:**
  - Rename `FittedSegment` → `CubicSegment` in `rust/geometry/src/segment.rs`.
  - Add single-piece-cubic-Bézier invariant (runtime assertion at constructor / primitive entry).
  - Retire `Segment::Arc` variant.
  - Retire G0/G1/G2/G3 reduce code paths from `rust/geometry/src/reduce.rs` (preserve as a basis for Step 13 compat layer; either move or comment out per implementation preference).
  - Add `SplitInfo { sub_index, sub_count, s_lo_mm, s_hi_mm }` to `segment.rs`; populate on splitter output.
- **Tests:** Unit tests for compose + splitter + structural changes. Integration sanity test (synthetic G5 → reduce → splitter → Layer 2) confirming end-to-end shape works.

### 2.2 Out of scope (deferred)

- **Polynomial-refit primitive (rational quadratic → polynomial):** moves to Step 13 compat-layer crate. The Goldapp 1991 closed-form circular-arc-to-Bézier is the right tool there; 7-pre doesn't need it because the live pipeline doesn't see rationals.
- **Step 13 compat layer itself:** offline G0/G1/G2/G3 → G5 normalizer. Build-order Step 13.
- **Spline-fitter (Tajima/Sencer 2016, Beudaert 2012):** post-MVP, Step 13 sub-component for users who want smoother corners than collinear-cubic G1-by-G1 produces.
- **C¹ knot-multiplicity cleanup:** post-stitch deduplication of redundant knots that `bezier_pieces_to_nurbs` produces (always emits multiplicity p, giving C0 even when math says C¹). Optional follow-up; not blocking 7-pre.
- **MCU curve-pool resizing (`MAX_DEGREE` / `MAX_CONTROL_POINTS` / `MAX_KNOT_VECTOR_LEN` bumps):** that's Step 7-B scope. 7-pre's outputs land in the existing-bound curve-pool which won't actually accept post-shape NURBS until 7-B sizes it up. The integration sanity test in this spec runs the pipeline up through Layer 2 only, not all the way through Layer 4.
- **Production `MsgProtoParser` + `host_io.rs` robustness:** Step 7-C scope (Plan-decision-C deferrals from Step 6).
- **Knot insertion as a public Layer 0 primitive:** not needed by 7-pre's specific use cases; the existing `split_piece_at` covers the splitter's needs.

## 3. Architecture context

The locked architectural state from the round-1 + round-2 brainstorm sessions (CLAUDE.md feature scope and plan-changes-log entries):

- **Live pipeline accepts G5 / G5.1 only.** Uniform cubic Bézier polynomial NURBS through Layer 1 / 2 / 3 / 4. No rational NURBS, no mixed-degree dispatch, no source-gcode-type special-cases.
- **G5 → cubic Bézier direct.** G5.1 → cubic via exact degree-elevation (degree 2 → 3, +1 control point, no fit error).
- **Legacy G0 / G1 / G2 / G3 normalize offline via Step 13 compat layer.** That layer uses Goldapp 1991 closed-form circular-arc-to-Bézier (~2 cubic pieces per quarter-arc at 0.1 µm L∞ position error), G1 → cubic with collinear control points (degree elevation, exact), G5.1 → cubic (degree elevation, exact). Optional spline-fitter for smoother corners. Output is G5-only G-code consumable by the live pipeline.
- **T-A math-exact time-reparameterization** (per-TOPP-RA-grid-piece, modulo a sub-µm per-piece linear `u(s)` approximation that the geometric NURBS's natural-parameter-vs-arc-length non-polynomial relationship forces — the verifier round flagged this as the "meaningful caveat the brief glosses over"; documenting honestly here).
- **Per-axis scalar NURBS storage on the MCU** post-shape (independent X(t), Y(t), Z(t) curves; E(t) derived per-sample from `v_xy`, no separate E NURBS for COUPLED_TO_XY mode).
- **β-medium shaper-aware TOPP-RA in MVP** (closed-form post-shape peak `|ẍ_shaped|` from polynomial extremum of shaped NURBS; outer iteration on TOPP-RA accel limits to converge).

## 4. Layer 0 primitive: `compose_vector_piece`

### 4.1 Purpose

Math-exact polynomial-of-polynomial composition for T-A's time-reparameterization step. Computes `composed(t) = outer(inner(t))` where both inputs are Bézier pieces in Bernstein form.

### 4.2 Interface

```rust
pub fn compose_vector_piece<const D: usize>(
    outer: &[&BezierPiece<f64>; D],   // single-piece per axis (X, Y, Z; D=3 typical)
    inner: &BezierPiece<f64>,         // single-piece scalar (s(t) per TOPP-RA piece)
) -> Result<[BezierPiece<f64>; D], AlgebraError>;
```

- **`outer`**: an array of D single-piece scalar Bézier pieces, one per output axis. Each is a polynomial in some local parameter `s` (the geometric NURBS's parameter, after a linear `u(s)` approximation has been applied per piece — see §4.5 caveat).
- **`inner`**: a single-piece scalar Bézier piece, polynomial in t. The TOPP-RA-derived `s(t)` per grid piece, exactly degree-2 (per the verifier-confirmed closed-form `s(t) = √b_k·(t−t_k) + (b₁/4)·(t−t_k)²`).
- **Output**: an array of D single-piece scalar Bézier pieces in Bernstein form on `inner`'s domain interval `[t_k, t_{k+1}]`. Each output piece has degree `outer.degree() × inner.degree()` (for our use case: 3 × 2 = 6).

### 4.3 Algorithm

Direct Bernstein-basis substitution and re-collection (per Codex review confirmation that this is canonical CAGD for low-degree polynomial composition; alternatives like blossom/de-Casteljau-recursion or Chebyshev sample-and-refit are either stylistic improvements with negligible benefit at our degrees or strictly worse).

For each axis independently (sharing the same `inner`):

1. Normalize `inner`'s parameter range to `[0, 1]` for the substitution. Coefficients are unchanged in Bernstein form; only the domain interpretation shifts.
2. Substitute `inner(t) = Σ_j inner_coef_j × B_j^d_inner(t')` where `t' ∈ [0, 1]` is the local parameter, into `outer(s) = Σ_i outer_coef_i × B_i^d_outer(s)`. The result is `outer(inner(t')) = Σ_i outer_coef_i × B_i^d_outer(Σ_j inner_coef_j × B_j^d_inner(t'))`.
3. Expand each `B_i^d_outer(...)` term using the multinomial expansion of the inner sum, distribute, and re-collect into Bernstein form on `t' ∈ [0, 1]` of degree `d_outer × d_inner`.
4. Map back to `inner`'s actual `[t_k, t_{k+1}]` domain by setting the output piece's `u_start = inner.u_start`, `u_end = inner.u_end`. Coefficients are unchanged in Bernstein form.

Complexity: `O(d_outer × d_inner × d_outer × d_inner)` per axis = `O(36)` for our case, plus per-axis-independent so `O(D × 36)` = trivial. Well under f64 round-off concern at our degrees per Codex's analysis.

### 4.4 Implementation choices

- **Bernstein basis**: input and output both in Bernstein form. Avoids monomial-basis Vandermonde conditioning issues. Existing `BezierPiece` already stores Bernstein coefficients per the codebase's convention — no conversion needed at primitive boundary.
- **Single-piece API**: primitive operates on `BezierPiece`, not `VectorNurbs`. Multi-piece handling is the caller's concern (per Codex review concern flagged in Section B v2 review). Caller extracts pieces from any multi-piece input via existing `extract_bezier_pieces` before calling.
- **Per-axis loop**: D iterations of identical scalar work. Easily parallelizable but probably not worth thread-pool overhead at D=3-4.
- **No fit, no tolerance**: composition is algebraically exact at the polynomial-of-polynomial level. The only error source is f64 round-off in the substitute-and-collect step (~tens to hundreds of ulps per Codex's analysis; far below planning-visible scale).

### 4.5 Honest precision caveat (T-A integration)

The T-A "math-exact" framing has a subtle precision floor that I owe explicit documentation of:

- For a general G5 cubic Bézier, `x(u)` is cubic in u (the natural NURBS parameter). The arc-length `s(u) = ∫₀^u ‖x'(u')‖ du'` involves `√(polynomial)` and is *not* polynomial in u. Hence `u(s)` (its inverse) is also non-polynomial.
- T-A's "math-exact polynomial-of-polynomial composition" claim therefore requires a per-piece *linear u(s) approximation*: `u(s) ≈ u_k + (u_{k+1} - u_k) / Δs · (s - s_k)` over each TOPP-RA grid piece `[s_k, s_{k+1}]`.
- Linear u(s) per-piece error is bounded by the cubic Bézier's parameterization quality (how uniform the curve's speed is across the piece). For Goldapp-placed cubic Béziers from the Step-13 compat layer (Goldapp 1991 specifically chooses control-point placements to minimize parameterization-speed-non-uniformity), this is **sub-µm error per piece**. For kalico-aware-slicer G5 emission (where the slicer ideally picks similarly-uniform parameterizations), expectation is similar.
- Total trajectory error from this approximation: well below the 0.1 µm budget I documented earlier in CLAUDE.md, well below the f32 quantization floor on the MCU (~20 nm at end-of-bed), and well below mechanical printer tolerance (~50-100 µm).
- **The compose primitive itself is unaffected** by this caveat. It composes whatever polynomials the caller hands it. The precision story is about how Layer 3's T-A code prepares the `outer` argument to compose.

CLAUDE.md feature-scope bullet on T-A should be updated to clarify "math-exact polynomial-of-polynomial composition modulo a sub-µm per-piece linear-u(s) approximation," not "zero by construction." Filed as a doc-update follow-up.

### 4.6 Tests

- **Identity composition.** `compose(outer, inner=identity)` == `outer` to f64 round-off, where `identity = BezierPiece` with coefs `[0, 1/d, 2/d, ..., 1]` representing `t(t) = t`. Sanity check.
- **Linear inner (degree 1).** `compose(outer, inner=linear)` is equivalent to a parameter rescaling on `outer`. Compare against direct rescaled evaluation at sample points.
- **Cubic-outer × quadratic-inner.** The T-A case. Synthetic outer = cubic `[1, 2, 3, 4]`-CP curve; synthetic inner = quadratic `[0.1, 0.5, 0.9]`-CP curve. Evaluate composition at 100 sample points; compare against direct evaluation `outer(inner(t))` computed via two separate Bernstein evaluations. Match to f64 ULP-tolerance.
- **Sympy cross-check.** Generate a sympy-symbolic version of the composition for a fixed input pair; verify the Bernstein coefficients of our compose primitive match the sympy-derived coefficients to 1e-12 absolute.
- **Vector composition consistency.** For a vector outer with D=3 axes (synthetic XY-arc + Z-linear case), verify each per-axis output is bit-exact to the corresponding scalar composition. (D=3 is just D parallel scalar applications.)

### 4.7 Code size estimate

~150 LOC for the primitive itself + ~80 LOC of tests + ~50 LOC of fixture / cross-check helpers. Total ~280 LOC.

## 5. Layer 1 primitive: `split_segment_to_cap`

### 5.1 Purpose

Bound per-MCU-segment path length at a fixed cap (default 12.5 mm) so the post-shape per-axis NURBS fits within the bumped curve-pool slot's `MAX_KNOT_VECTOR_LEN` / `MAX_CONTROL_POINTS` budget on the H723. Operates at Layer 1 / `geometry::pipeline`, after `geometry::reduce` produces a `CubicSegment` and before it's emitted to Layer 2.

### 5.2 Interface

```rust
pub fn split_segment_to_cap(
    segment: &CubicSegment,
    max_arc_length_mm: f64,    // default 12.5 mm
) -> Vec<CubicSegment>;
```

- **`segment`**: a single-piece cubic Bézier `CubicSegment` with metadata (feedrate, e_mode, extrusion ratio, source range).
- **Output**: a vector of `CubicSegment`s, each with `arc_length ≤ max_arc_length_mm` and the parent's metadata propagated. Each child segment carries `Some(SplitInfo)` if the parent was split; `None` if passthrough.

### 5.3 Algorithm

Adapts the existing `algebra.rs::refine_pieces_to_breakpoints` global-domain split pattern (per Codex review confirmation that this is the right primitive vs. de Casteljau parameter-rescaling bookkeeping):

1. Build the arc-length table for the parent cubic *once* via `nurbs::arc_length::build_arc_length_table_vector`. Compute total `L`.
2. If `L ≤ max_arc_length_mm`: return `vec![segment.clone()]` with `SplitInfo: None`. No work.
3. Compute `k = ⌈L / max_arc_length_mm⌉` and the `k-1` target arc-lengths `[L/k, 2L/k, …, (k-1)L/k]`.
4. Convert each target to a parameter `u_i` via `nurbs::arc_length::param_from_arc_length(table, target_s)`. Outputs are sorted (table is monotone-by-construction).
5. Walk the global-domain split pattern, carrying the right piece forward:
   ```rust
   let mut current = parent_piece;
   for &u in &u_breaks {
       let (left, right) = nurbs::bezier::split_piece_at(&current, u)?;
       out.push(left);
       current = right;
   }
   out.push(current);
   ```
6. Wrap each piece as a `CubicSegment` with parent metadata propagated, plus `SplitInfo { sub_index: i as u32, sub_count: k as u32, s_lo_mm: i × L/k, s_hi_mm: (i+1) × L/k }`. Source range preserved unchanged on each child.

### 5.4 Edge cases

- `L < max_arc_length_mm`: passthrough, `SplitInfo: None`.
- `L = max_arc_length_mm` exactly: passthrough (single segment of exactly the cap length).
- `L = max_arc_length_mm + ε`: produces 2 sub-segments; both ≤ cap.
- `L = k × max_arc_length_mm` exactly: produces k sub-segments of equal length.
- `e_mode = INDEPENDENT` (retraction / prime, no XY motion): arc length in XYZ space is 0; passthrough.
- Multi-piece input: out-of-spec; primitive panics or returns `Err` (the runtime assertion documented in §6.1).

### 5.5 Implementation choices

- **Build arc-length table once on parent**, not per child (per Codex review note Q7). Existing `build_arc_length_table_vector` is 5-point Gauss-Legendre with doubling — moderate cost. Single call per `split_segment_to_cap` invocation regardless of `k`.
- **Use existing `split_piece_at`**, not de Casteljau directly. The codebase's `split_piece_at` uses monomial-basis re-shifting; functionally equivalent to de Casteljau for cubic, fewer surprises.
- **Sequential right-piece carry** (per `algebra.rs::refine_pieces_to_breakpoints` template). Avoids parameter-rescaling bookkeeping.

### 5.6 Tests

- **Passthrough.** `L < cap`: output is a single-element Vec equal to input; `SplitInfo: None`.
- **Boundary.** `L = cap` exactly: passthrough.
- **Two-piece split.** `L = 25 mm`, cap = 12.5: 2 sub-segments, each 12.5 mm; sum-of-arc-lengths = 25 within table tolerance (~1e-6 mm).
- **Eight-piece split.** `L = 100 mm`, cap = 12.5: 8 sub-segments. Each has SplitInfo populated correctly.
- **Continuity.** For a `L = 50 mm` test segment, evaluate parent at `s = i × 12.5` and compare against child `i+1`'s start point — bit-exact (`split_piece_at` produces this by construction).
- **Metadata propagation.** Feedrate, e_mode, extrusion_per_xy_mm, source_range preserved unchanged on each child.
- **Arc-length-table caching.** Mock `build_arc_length_table_vector` to count invocations; verify exactly 1 call per `split_segment_to_cap` invocation regardless of `k`.
- **`e_mode = INDEPENDENT`.** Passthrough verification.

### 5.7 Code size estimate

~80 LOC for the primitive itself + ~100 LOC of tests + ~30 LOC of fixture helpers. Total ~210 LOC.

## 6. Layer 1 structural changes

### 6.1 `FittedSegment` → `CubicSegment` rename + invariant

**Old shape (in `rust/geometry/src/segment.rs`):**
```rust
pub struct FittedSegment {
    pub xyz: VectorNurbs<f64, 3>,
    pub e: Option<ScalarNurbs<f64>>,
    pub feedrate_mm_s: f64,
    pub degree: u8,
    pub max_residual_mm: f64,
    pub source: SourceRange,
}
```

**New shape:**
```rust
pub struct CubicSegment {
    /// Single-piece cubic Bézier in arc-length-related parameter space.
    /// Invariant: degree == 3, control_points.len() == 4, no weights.
    /// Enforced by constructor (returns Result on violation) and checked
    /// at primitive entry points (compose, splitter).
    pub xyz: VectorNurbs<f64, 3>,
    pub e_mode: EMode,                    // COUPLED_TO_XY | INDEPENDENT
    pub extrusion_per_xy_mm: f64,         // valid when e_mode == COUPLED_TO_XY
    pub e_independent: Option<ScalarNurbs<f64>>,  // valid when e_mode == INDEPENDENT
    pub feedrate_mm_s: f64,
    pub source: SourceRange,
    pub split_info: Option<SplitInfo>,    // None on un-split, Some on splitter output
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EMode {
    CoupledToXy,
    Independent,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SplitInfo {
    pub sub_index: u32,
    pub sub_count: u32,
    pub s_lo_mm: f64,
    pub s_hi_mm: f64,
}

impl CubicSegment {
    pub fn try_new(...) -> Result<Self, GeometryError> {
        // Validates: xyz is single-piece cubic (degree 3, 4 CPs, no weights, clamped knots).
        // Returns Err(NotSinglePieceCubic) on violation.
        ...
    }
}
```

**Removed fields:** `degree` (always 3, redundant), `max_residual_mm` (only meaningful for fitter output, which moves to Step 13 compat layer; live pipeline doesn't fit). **Renamed/added:** `e` → `e_mode + extrusion_per_xy_mm + e_independent` (per Section A's E-follows-XY architecture from round-1 brainstorm); `split_info` (per Section C splitter output).

### 6.2 `Segment` enum simplification

**Old:**
```rust
pub enum Segment {
    Fitted(FittedSegment),
    Arc(ArcSegment),
    CornerBlend(CornerBlendSlot),
    Junction(JunctionDeviation),
}
```

**New:**
```rust
pub enum Segment {
    Cubic(CubicSegment),
    CornerBlend(CornerBlendSlot),
    Junction(JunctionDeviation),
}
```

`Arc` variant retired. `FittedSegment`/`Fitted` renamed to `CubicSegment`/`Cubic`.

### 6.3 Retire G0/G1/G2/G3 reduce paths from live `geometry::reduce`

Per the round-2 brainstorm decision: live pipeline accepts G5 / G5.1 only.

- The existing G0, G1, G2, G3 handling code in `rust/geometry/src/reduce.rs` is preserved for *reference* (will be the basis for Step 13 compat-layer logic) but **removed from the live reduce-stage match**.
- Live `reduce.rs` accepts only G5 / G5.1 token kinds. G5 → cubic Bézier direct. G5.1 → cubic via exact degree-elevation (degree 2 → 3, +1 control point at the midpoint of P0 and P2 of the original quadratic, no fit error).
- Encountering G0/G1/G2/G3 in the live parser is an error (`GeometryError::UnsupportedGcode`) telling the user to run their G-code through the Step-13 compat layer first. (Or, depending on implementation preference, the error is at parse time — `gcode::parser` rejects G0/G1/G2/G3 in live mode.)
- **Disposition of removed code:** Two options the implementer can choose:
  1. **Comment out + preserve for Step 13.** Keeps the code in-tree; Step 13 imports and adapts.
  2. **Move to a new `rust/geometry-compat/` crate (or similar).** Cleaner separation; Step 13 builds on this crate. Imposes a workspace member move now for an item that doesn't ship until Step 13.
  
  Recommended: option (1) for minimal-diff during 7-pre, with a clear comment block stating "Step 13 compat-layer code; do not invoke from live pipeline." Option (2) when Step 13 is actively brainstormed.

### 6.4 Removed: `ArcSegment` struct + `Segment::Arc` constructor sites

`ArcSegment` is dead-code in the live pipeline (no producers post round-2 brainstorm). Remove the struct entirely. Update any test fixtures or callsites that referenced it.

### 6.5 Updated: G5.1 → cubic via degree-elevation

`reduce.rs` G5.1 path currently emits a degree-2 polynomial NURBS. New path: degree-elevate to degree-3 immediately. Bernstein degree-elevation formula: for a degree-2 Bézier with control points `[Q_0, Q_1, Q_2]`, the equivalent degree-3 Bézier has control points `[Q_0, (1/3) Q_0 + (2/3) Q_1, (2/3) Q_1 + (1/3) Q_2, Q_2]`. Exact, no fit error.

### 6.6 G5 reduction unchanged

The existing G5 reduction in `reduce.rs` (per the existing 2026-04-27 plan) already produces a degree-3 single-piece NURBS with 4 control points. Stays as-is.

## 7. Test strategy

### 7.1 Unit tests

- **Compose primitive** (§4.6).
- **Splitter primitive** (§5.6).
- **CubicSegment constructor invariant.** Reject multi-piece, reject degree != 3, reject != 4 CPs, reject rational (weighted) input.
- **Reduce stage G5.1 → cubic.** Verify degree-elevation produces correct cubic; evaluate at sample points; bit-exact match to original quadratic Bézier.
- **Live reduce rejects G0/G1/G2/G3.** Error message points to compat layer.

### 7.2 Integration tests

- **Synthetic G5 → reduce → splitter → Layer 2.** Generate a long G5 G-code (synthetic 50 mm cubic Bézier), parse, reduce, split, plan_batch on the resulting child segments, verify continuity at sub-segment boundaries (junction velocities reconcile across splits).
- **Composed-NURBS shape end-to-end.** For a test input (synthetic G5 + synthetic TOPP-RA-output `b(s)`), run the full T-A composition (§4.5 caveat: linear-u(s)-per-piece + compose primitive) and verify the resulting per-axis NURBS is degree-6, multi-piece, with continuity at TOPP-RA-grid joints. *Note: this is a 7-A integration test that depends on 7-pre primitives — co-developed with 7-A.*

### 7.3 Regression tests

- **Existing tests involving G1/G2/G3 reduce paths** (e.g., `g5_reduction.rs`, `multi_segment.rs` with G2/G3 fixtures) need disposition:
  - Tests of the G2/G3 reduce code itself: move to the (yet-to-exist) Step-13 compat-layer crate. For 7-pre, mark `#[ignore]` with a comment pointing to Step 13 follow-up.
  - Tests of Layer 2 multi-segment integration that *consume* G2/G3 output: rebuild fixtures from G5-only inputs (synthetic cubic Béziers in place of G2/G3 → rational quadratics). Most of these tests are about Layer 2 algorithms (junction-velocity, lookahead-window joining), not about G2/G3-specific reduction; rebuilding fixtures preserves coverage.

## 8. Implementation sequence

The three deliverables can develop in parallel after the structural rename, but a sensible serialization is:

1. **Layer 0 compose primitive** (`compose_vector_piece`). Independent of all Layer 1 changes; depends only on existing `BezierPiece` infrastructure. Can land first.
2. **Layer 1 structural rename** (`FittedSegment` → `CubicSegment`, retire `ArcSegment`, simplify `Segment` enum). Touches `segment.rs` + every callsite. Independent of compose primitive but should land before splitter so splitter operates on the new types.
3. **Layer 1 SplitInfo** addition. Tiny diff; lands with structural rename.
4. **Layer 1 splitter primitive** (`split_segment_to_cap`). Operates on the post-rename types. Lands after structural rename.
5. **Live reduce-stage cleanup** (drop G0/G1/G2/G3 paths). Tiny diff but cross-cuts existing tests; coordinate with the test-disposition step (§7.3).
6. **Test updates and regression-test disposition.** Final cleanup pass.

Total estimated implementation effort: ~2 weeks for one developer (compose ~3 days, splitter ~3 days, structural rename ~2 days, test updates and integration ~3 days, slack ~3 days).

## 9. Open questions / assumptions

- **Q1: Should `compose_vector_piece` live in `nurbs::algebra` or a new `nurbs::composition` module?** Defaulting to `nurbs::algebra::compose_vector_piece` for simplicity (one less module). Implementer can split into a separate module if the algebra module grows past readable size during 7-pre.
- **Q2: Should the runtime assertion on `CubicSegment` invariants live at `try_new` time only, or be re-checked at every primitive entry?** Defaulting to constructor-only (`try_new`). The type is constructed once and consumed by primitives; re-checking at primitive entry would be paranoid. Document the invariant clearly in the type's doc-comment.
- **Q3: Disposition of `geometry::reduce`'s G0/G1/G2/G3 code:** comment-out-and-preserve (§6.3 option 1) vs. move-to-new-crate (option 2). Defaulting to option 1 for 7-pre's minimal-diff scope; revisit when Step 13 is actively brainstormed.
- **Q4: Should the compat-layer's existence be advertised in error messages?** When the live parser sees G0/G1/G2/G3, the error message should mention "run input through Step-13 compat layer." Stronger: maybe *recommend* a specific tool or command. Defer the exact wording to implementation; the architectural decision is just that the error is *expressive* not silent.
- **Q5: T-A precision-caveat documentation update.** §4.5 documents the sub-µm linear-u(s)-per-piece approximation honestly. CLAUDE.md's existing T-A bullets should be updated to reflect this caveat. Defaulting to "fold this into 7-A's spec rather than retroactively patching CLAUDE.md again now" — the round-2 plan-changes-log entry has the round-1 + round-2 record; 7-A's spec will refine the precision claim.
- **Q6: Knot-multiplicity cleanup pass.** Codex flagged that `bezier_pieces_to_nurbs` emits knots at multiplicity p (degree), giving C0 stitching even when math says C¹. Not a correctness issue — extra storage. Defer to a small follow-up after 7-pre lands; not blocking.

## 10. References

- CLAUDE.md (2026-04-29 state) — feature scope, Layer 1, build-order Step 7 / Step 13, critical-path observations.
- docs/superpowers/plan-changes-log.md — 2026-04-29 round-1 + round-2 entries documenting the brainstorm decisions.
- docs/research/bspline-polynomial-convolution.md — Minkowski-sum knot-vector growth from convolution; degree formula `d_input + d_kernel + 1`.
- docs/research/layer3-time-polynomial-fit-bounds.md — T-B verifier artifact (T-B was deprecated in favor of T-A but the analysis of fit error scales remains useful reference).
- docs/research/single-polynomial-fit-per-segment-conditioning.md — T-C verifier artifact (T-C refuted via Jackson convergence floor on C¹ x(t)).
- Codex review of Section B (composition primitive design) — flagged single-piece API, blossom alternative, C0-stitched-knot redundancy.
- Codex review of Section C (splitter design) — flagged use of existing `split_piece_at` + `refine_pieces_to_breakpoints` template, `param_from_arc_length` API name correction, `ArcLengthTable` caching, `CubicSegment` type concern.
- kalico-researcher CAGD literature survey (2026-04-29) — Sederberg-Kakimoto 1991, Wang-Sederberg-Chen 1997, Floater 1995/2006, Hu-Wang-Jin 2008, Lewanowicz-Woźny-Keller 2012/2015, Goldapp 1991, Vavpetič-Žagar 2015/2017, Kim 2023 hexic. Used to refine Section A's algorithm choice (eventually retired) and to confirm Section B's direct-Bernstein-substitution approach as canonical.
- Piegl & Tiller, *The NURBS Book* (2nd ed., 1997), §5.2 (Bernstein convex-hull property), §5.4 (knot insertion / degree elevation).
- Goldapp 1991, "Approximation of circular arcs by cubic polynomials," CAGD 8:227 — closed-form cubic Bézier approximation of circular arcs (used by Step 13 compat layer, not by 7-pre directly).
