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

Step 7-A (the Layer-3 minimum that lands smooth-ZV/MZV pre-bake + β-medium shaper-aware TOPP-RA + T-A time-reparameterization + E-follows-shaped-XY) sits on top of three new Layer-0/Layer-1 primitives and a Layer-1 structural refactor that the existing codebase does not yet provide:

1. **A polynomial composition primitive** (`nurbs::algebra::compose_vector_piece`) that does polynomial-of-polynomial substitution on Bézier pieces. T-A's per-TOPP-RA-grid-piece work is a per-axis-polynomial-of-quadratic-Bézier composition; without this primitive, T-A would re-implement composition inline in Layer 3.
2. **A per-piece arc-length-domain polynomial fit primitive** (`nurbs::algebra::fit_x_to_arc_length_piece`) that produces a per-axis polynomial in arc-length s on a given TOPP-RA grid piece, *sample-verified* against a target L∞ position-error budget. Required because the cubic Bézier `x(u)`'s arc-length parameterization `x(s)` is non-polynomial and the round-1 "math-exact via linear u(s)" framing was refuted on round-1 spec review (see §4.5).
3. **A gcode-side N≤25 segment splitter** (`geometry::pipeline::split_segment_to_cap`) that bounds per-MCU-segment piece count by capping path length at 12.5 mm. Without this, a long G5-collinear (post-compat-layer) input from a legacy slicer could produce a NURBS with 200 grid pieces, blowing the bumped-but-still-bounded curve-pool slot budget on the H723.
4. **A Layer-1 structural refactor** that the round-2 brainstorm locked in: `FittedSegment` renamed to `CubicSegment` with a single-piece-cubic-Bézier invariant; G0/G1/G2/G3 reduce paths retired from the live pipeline (those move to Step 13's compat layer); `Segment::Arc` variant retired; `SplitInfo` metadata added for sub-segment provenance; explicit E-mode semantics for travel / extrusion / E-only / Z-only motion classes.

This spec captures the design of the three items together because they're tightly coupled: the splitter operates on the post-rename `CubicSegment`, the compose primitive's caller pattern depends on the splitter's output structure, and all three exist to *prepare* the pipeline for 7-A's math.

## 2. Scope

### 2.1 In scope

- **Layer 0:** Two new primitives: `compose_vector_piece` (polynomial Bézier-piece composition) and `fit_x_to_arc_length_piece` (adaptive polynomial fit of x(s) sample-verified to a target position-error tolerance per TOPP-RA grid piece). Plus a small sibling primitive `xy_arc_length` (scalar one-shot query). All in `rust/nurbs/src/{algebra,arc_length}.rs`. Leverage existing `BezierPiece`, `arc_length::param_from_arc_length`, and the vector-NURBS evaluator.
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
- **T-A time-reparameterization** (per-TOPP-RA-grid-piece). Closed-form `s(t)` is *exactly* degree-2 polynomial in t per piece (per the TOPP-RA piecewise-linear-b convention; verifier-confirmed in round 1). The geometric `x(s)` per piece, however, is *not* polynomial — `x(u)` is cubic in the natural parameter u, but `u(s)` is non-polynomial (square-root integral) and a linear `u(s)` approximation per piece was refuted on round-1 review as ≥ 2-7 µm error on typical FDM cubics (see §4.5). Practical T-A: per-piece adaptive polynomial fit of `x(s)` *sample-verified* to a configurable L∞ position-error tolerance (default 1 µm; sample-based bound, not certified L∞ — see §9 Q7), composed with `s(t)` to yield `x(t)` polynomial per piece. Output position error is sample-verified at the configured tolerance, *not* zero — but well below mechanical tolerance and below the smooth-shaper L¹ amplification floor.
- **Per-axis scalar NURBS storage on the MCU** post-shape (independent X(t), Y(t), Z(t) curves; E(t) derived per-sample from `v_xy`, no separate E NURBS for COUPLED_TO_XY mode).
- **β-medium shaper-aware TOPP-RA in MVP** (closed-form post-shape peak `|ẍ_shaped|` from polynomial extremum of shaped NURBS; outer iteration on TOPP-RA accel limits to converge).

## 4. Layer 0 primitives: `compose_vector_piece` and `fit_x_to_arc_length_piece`

### 4.1 `compose_vector_piece` — purpose

Polynomial-of-polynomial composition for T-A's time-reparameterization step. Computes `composed(t) = outer(inner(t))` where both inputs are Bézier pieces. The error in the *composition itself* is f64 round-off only (~tens of ulps); any precision floor in T-A's overall trajectory output comes from how the caller prepares `outer` (see §4.5).

### 4.2 `compose_vector_piece` — interface

```rust
pub fn compose_vector_piece<const D: usize>(
    outer: &[&BezierPiece<f64>; D],   // single-piece per axis (X, Y, Z; D=3 typical)
    inner: &BezierPiece<f64>,         // single-piece scalar (s(t) per TOPP-RA piece)
) -> Result<[BezierPiece<f64>; D], AlgebraError>;
```

- **`outer`**: an array of D single-piece scalar Bézier pieces, one per output axis. Each is a polynomial in arc-length `s` on the TOPP-RA grid piece's `[s_k, s_{k+1}]`. Caller produces these via `fit_x_to_arc_length_piece` (§4.5) — *not* via "linear u(s) substitution," which round-1 review refuted.
- **`inner`**: a single-piece scalar Bézier piece, polynomial in t. The TOPP-RA-derived `s(t)` per grid piece, exactly degree-2 (per the verifier-confirmed closed-form `s(t) = √b_k·(t−t_k) + (b₁/4)·(t−t_k)²`).
- **Output**: an array of D single-piece scalar Bézier pieces on `inner`'s domain interval `[t_k, t_{k+1}]`. Each output piece has degree `outer.degree() × inner.degree()` — for typical T-A usage (`outer.degree() = 6`, `inner.degree() = 2`): **degree 12 per piece pre-shape, degree 17 post-shape after smooth-shaper convolution**.

### 4.3 `compose_vector_piece` — algorithm

Direct substitution-and-collect in the **Pascal-shifted monomial basis** that `BezierPiece` actually uses (`p(u) = Σ coeffs[k] × (u − u_start)^k` per `rust/nurbs/src/bezier.rs:1-15`). Round-1 spec review (codex) caught a CRITICAL error in v1: the spec previously claimed Bernstein storage, which is false. `BezierPiece` exposes `to_bernstein` / `from_bernstein` converters at lines 41-99 if a Bernstein-form intermediate is desired, but native monomial-basis substitution is more direct.

For each axis independently (sharing the same `inner`):

1. Normalize `inner` to its native local basis: `inner(t) = Σ_j inner.coeffs[j] × (t − inner.u_start)^j` for `t ∈ [inner.u_start, inner.u_end]`.
2. The geometric `outer(s) = Σ_i outer.coeffs[i] × (s − outer.u_start)^i` for `s ∈ [outer.u_start, outer.u_end]`. **Precondition for the caller:** `outer.u_start = inner.evaluate(inner.u_start)` and `outer.u_end = inner.evaluate(inner.u_end)` — i.e., `outer`'s s-domain range exactly matches the s-image of `inner`'s t-domain range. Without this affine alignment, the substitution lands `outer` outside its valid domain. (Caller responsibility — see §4.6.)
3. Compute `outer(inner(t)) = Σ_i outer.coeffs[i] × (inner(t) − outer.u_start)^i`. Each `(inner(t) − outer.u_start)^i` is a polynomial in `(t − inner.u_start)` of degree `i × inner.degree()`. Expand via multinomial, multiply by `outer.coeffs[i]`, sum across i.
4. Output `BezierPiece` has `u_start = inner.u_start`, `u_end = inner.u_end`, and `coeffs[k]` = the collected polynomial coefficient on `(t − inner.u_start)^k`. Length = `outer.degree() × inner.degree() + 1`.

Two implementation options for the substitute-and-collect step:

- **(A) Native monomial-basis loop**: directly compute `(inner(t) − outer.u_start)^i` powers iteratively (each new power = previous power × (inner − outer.u_start), polynomial multiplication via convolution of coefficient arrays). Then sum with outer.coeffs[i] weights.
- **(B) Round-trip via Bernstein**: convert `outer` and `inner` to Bernstein form on their respective domains, do Bernstein-basis composition (a more standard CAGD path with cleaner conditioning theory), convert result back to monomial-basis BezierPiece via `from_bernstein`.

(A) is simpler to implement and avoids two basis conversions per call. (B) has cleaner numerical conditioning theory at higher degrees but is overkill for our typical case (fit-degree-6 × inner-degree-2 = degree-12 output) in f64. **Recommendation: (A) as v1 implementation; switch to (B) only if numerical issues surface in tests at degree 12 or higher.**

Complexity: `O((d_outer × d_inner)^2)` per axis = `O(144)` for our typical case, per-axis-independent so `O(D × 144)`. Well under f64 round-off concern at our degrees per Codex review.

### 4.4 `compose_vector_piece` — implementation choices

- **Native Pascal-shifted monomial basis**: input and output both use `BezierPiece`'s native basis. Existing `BezierPiece::to_bernstein`/`from_bernstein` available if conversion is wanted internally.
- **Single-piece API**: primitive operates on `BezierPiece`, not `VectorNurbs`. Multi-piece handling is the caller's concern (per Codex review concern flagged in Section B v2 review). Caller extracts pieces from any multi-piece input via existing `extract_bezier_pieces` before calling, or constructs `BezierPiece`s directly via `fit_x_to_arc_length_piece`.
- **Per-axis loop**: D iterations of identical scalar work. Easily parallelizable but probably not worth thread-pool overhead at D=3-4.
- **No fit, no tolerance** (within the primitive itself): composition is algebraically exact at the polynomial-of-polynomial level. The only error source is f64 round-off in the substitute-and-collect step (~tens of ulps per Codex's analysis; far below planning-visible scale).
- **Affine domain precondition** (§4.3 step 2): the caller must align `outer`'s s-domain with `inner`'s s-image before calling. The primitive can debug-assert this at entry; release-mode trusts the caller. See §4.6 for the caller pattern that establishes this alignment.

### 4.5 `fit_x_to_arc_length_piece` — purpose and precision story

**Round-1 spec review (codex HIGH 1) and the kalico-verifier verification round** (artifact: `docs/research/linear-us-approximation-cubic-bezier-error.md`) refuted the v1 spec's claim that a per-piece *linear* `u(s)` approximation yields sub-µm position error for typical FDM cubic Béziers. The actual error scaling for linear `u(s)` is

```
ε_max ≤ Δs² / 8 · max |v'(u) / v²(u)|,    where v(u) = ‖x'(u)‖
```

which delivers concrete worst-case numbers across realistic input classes:

| Input class | Linear-u(s) error per Δs = 0.5 mm piece |
|---|---|
| Long gentle 50 mm Bézier (speed ratio ~ 1.5×) | ~ 2.5 µm |
| Typical S-curve, speed ratio ~ 2.15× | ~ 4.4 µm |
| Tight arc R = 1 mm (R ≤ 2 mm features in real prints) | ~ 5-50 µm |
| Near-cusp / clustered control points | 100 µm — diverges at v→0 |
| Large-radius arc R ≥ 10 mm, θ ≤ 90° | sub-µm |

The verifier also confirmed Goldapp 1991 minimizes geometric L∞ chord error, *not* parameterization-speed-uniformity (per Rababah 2016) — the v1 spec's claim that Goldapp placement yielded benign u-vs-s mapping was factually wrong. For arc-style cubics emerging from the Step-13 compat layer, sub-µm holds only at large radii.

**Practical T-A approach: per-piece adaptive polynomial fit of `x(s)` directly.** Sample `x` at d+1 Chebyshev-spaced s-values within each TOPP-RA grid piece, fit a polynomial of degree d in s through them, sample-verify residual against truth at oversampled points, raise degree or split piece on failure. Sample-verified position error at a configurable tolerance (per §4.5 step 4 + §4.5.1).

**Interface:**

```rust
pub fn fit_x_to_arc_length_piece<const D: usize>(
    geometry: &VectorNurbs<f64, D>,            // parent cubic Bézier in u-domain
    table: &ArcLengthTableRef<'_, f64>,        // built once for the parent segment
    s_lo: f64, s_hi: f64,                      // arc-length range of the TOPP-RA grid piece
    target_degree: u8,                          // default 6
    max_degree: u8,                             // default 10 (cap before triggering split)
    tolerance_mm: f64,                          // default 1 µm L∞
) -> Result<[BezierPiece<f64>; D], FitError>;

pub enum FitError {
    /// Reached max_degree without satisfying tolerance — caller should split the piece and recurse.
    ToleranceNotReached { achieved_mm: f64, at_degree: u8 },
    /// Pathological input — table inversion or geometry evaluation failed.
    DegenerateInput { reason: &'static str },
}
```

**Algorithm:**

1. Generate d+1 Chebyshev-of-the-second-kind nodes in s on `[s_lo, s_hi]`: `s_i = (s_lo + s_hi)/2 + (s_hi - s_lo)/2 · cos(i·π / d)` for `i ∈ [0, d]`. (Endpoints are included; interior nodes are well-conditioned for polynomial interpolation.)
2. For each s_i, query `u_i = arc_length::param_from_arc_length(table, s_i)` and evaluate `x_i = vector_eval(geometry, u_i)` (returns a D-vector).
3. For each axis a ∈ [0, D), solve Lagrange interpolation: find polynomial `p_a(s)` in Pascal-shifted-monomial basis on `[s_lo, s_hi]` with `p_a(s_i) = x_i[a]` for all i. (Standard Vandermonde-like solve at degree d ≤ 10 — well-conditioned at Chebyshev nodes.)
4. **Verification step (revised per round-2 review).** Codex round-2 flagged that midpoint-only sampling can miss oscillatory peaks in the residual function `r(s) = x(u(s)) − p_a(s)` between Chebyshev nodes — for high-degree interpolation of high-curvature inputs, the residual peaks may not coincide with the simple midpoints. Revised verification: oversample the residual at `4·(d+1)` points (4× the interpolation-node count), uniformly distributed across `[s_lo, s_hi]`. Take `max_residual = max_a max_i |x(u(s_i))[a] − p_a(s_i)|`. If `max_residual > tolerance`: increase d by 1, return to step 1. Cap at `max_degree`. **Disclaimer on the bound:** this is an *empirical sample-based bound*, not a certified L∞. Pathological residual shapes (oscillating with period < `Δs / (4·(d+1))`) could in principle still hide a peak between samples — but this would require ‖x⁽ᵈ⁺¹⁾‖∞ to be enormously high while remaining bounded, which doesn't happen for cubic Bézier inputs at any practical scale. Section 9 (Open Questions) flags a possible follow-up to upgrade to a Bernstein-coefficient-based certified bound if pathological cases ever surface.
5. If verification still fails at `max_degree`: return `FitError::ToleranceNotReached { achieved_mm, at_degree }`. Caller (T-A loop or splitter) bisects the TOPP-RA grid piece and recurses with two halves. **Termination policy** (added per round-2 MEDIUM): caller's bisection is bounded by `max_recursion_depth = 8` (giving minimum sub-piece width `Δs / 256 = 1.95 µm` at the default 0.5 mm grid) — past which the caller returns a hard error to the planner ("input geometry is pathologically curved; reject the segment"). This prevents pathological inputs from looping or exploding piece count.

**Why this works.** Polynomial-fit error at Chebyshev nodes scales as `Δs^(d+1) / 2^d / (d+1)! × ‖x⁽ᵈ⁺¹⁾(s)‖∞`. For Δs = 0.5 mm and d = 6, this is `(0.5)⁷ / 2⁶ / 7! × ‖x⁽⁷⁾‖∞ ≈ 2.4×10⁻⁸ × ‖x⁽⁷⁾‖∞`. For non-pathological cubic Béziers, ‖x⁽⁷⁾‖∞ stays bounded, giving sub-µm at d = 6 effortlessly. Pathological cases (near-cusps, R<0.5 mm features, slicer-emitted bizarre control points) trigger the adaptive path; if even max_degree+max_recursion-depth bisection doesn't converge, the segment is rejected.

**Precondition contract for `compose_vector_piece` callers** (resolves §4.4's affine alignment requirement): `fit_x_to_arc_length_piece` returns `BezierPiece`s with `u_start = s_lo`, `u_end = s_hi`. The TOPP-RA `inner = s(t)` Bézier piece has `u_start = t_k`, `u_end = t_{k+1}` and evaluates to `s_lo` at t_k, `s_hi` at t_{k+1} (by construction of the closed-form `s(t)`). So composing them is well-defined and the affine-alignment precondition is satisfied automatically.

**Tests for `fit_x_to_arc_length_piece`:**

- **Linear input.** For a degree-1 cubic-Bézier representing a straight line (collinear control points at 1/3, 2/3 lerp), `fit_x_to_arc_length_piece` produces a linear polynomial in s. L∞ error is f64 round-off only.
- **Tight arc.** For a Goldapp-style cubic of an R = 1 mm quarter-arc, fit at default tolerance triggers the adaptive path; verify converged degree and L∞ error ≤ 1 µm.
- **Pathological near-cusp.** Cubic Bézier with `P_1 = P_0 + ε`, `P_2 = P_3 - ε`, `ε = 10⁻⁶`. Verify either: (a) high-degree fit succeeds, or (b) `FitError::ToleranceNotReached` returned and caller falls back gracefully.
- **Boundary integrity.** For all non-error cases: `eval(fit, s_lo) == x(u(s_lo))` to f64 round-off; same at `s_hi`. (Chebyshev-of-2nd-kind includes endpoints, so this is by construction.)
- **Degenerate-input rejection.** Zero-length piece, NaN coordinates, table inversion failure → returns `FitError::DegenerateInput`.

**Code size estimate for `fit_x_to_arc_length_piece`:** ~120 LOC + ~80 LOC tests.

### 4.5.1 Combined precision picture

After `fit_x_to_arc_length_piece` produces `outer` and `compose_vector_piece` composes with `inner = s(t)`, the per-axis polynomial-in-t per piece has:

- **Position error vs. ideal x(t)**: *sample-verified* at the configured tolerance (default 1 µm) by the fit step's `4·(d+1)` oversampling check. Round-3 review (codex MEDIUM-5) corrected v2's framing: this is an empirical sample-verified bound, *not* a certified L∞. Pathological residual shapes oscillating with period below `Δs / (4·(d+1))` could in principle hide a peak between samples, but this would require ‖x⁽ᵈ⁺¹⁾‖∞ to be enormously high — doesn't happen for cubic Bézier inputs at any practical scale.
- **f64 round-off in compose**: ~tens of ulps, ≤ 10⁻¹³ mm scale. Negligible.
- **Total per-piece (sample-verified)**: ≤ tolerance.

After downstream smooth-shaper convolution, the L¹ kernel norm amplifies: ≤ tolerance × ‖kernel‖₁ ≈ tolerance × 1.0–1.2. After float32 conversion on MCU at end of bed: + ~20 nm.

**Total trajectory representation error (sample-verified): ≤ ~1.2 µm at default tolerance.** Well below mechanical printer tolerance (50–100 µm), well below stepper quantization (12.5 µm at 80 steps/mm), well below smooth-shaper amplification floor.

This is honest. It's not "math-exact" and the bound is sample-verified rather than certified L∞. CLAUDE.md's feature-scope bullet on T-A should be updated to reflect this; deferred to 7-A's spec for the precise CLAUDE.md re-wording. A possible follow-up to upgrade to a certified L∞ via Bernstein-coefficient-bound on the residual function is filed in §9 Q7.

### 4.6 T-A caller pattern (using both Layer-0 primitives)

The Layer-3 T-A loop, per TOPP-RA grid piece `[s_k, s_{k+1}]` on parent geometric segment with already-built `arc_length::ArcLengthTable`:

```rust
// outer: per-axis polynomial-in-s on [s_k, s_{k+1}], adaptive degree, L∞ error ≤ tolerance.
let outer: [BezierPiece<f64>; 3] = nurbs::algebra::fit_x_to_arc_length_piece(
    &geometry, &arc_length_table, s_k, s_kp1,
    /*target_degree=*/6, /*max_degree=*/10, /*tolerance_mm=*/1e-3,
)?;

// inner: TOPP-RA closed-form s(t) on [t_k, t_{k+1}], exactly degree 2.
//   coefs[0] = s_k, coefs[1] = sqrt(b_k), coefs[2] = b_1 / 4
let inner: BezierPiece<f64> = BezierPiece {
    u_start: t_k,
    u_end: t_kp1,
    coeffs: vec![s_k, sqrt(b_k), b_1 / 4.0],
};

// composed: per-axis polynomial-in-t on [t_k, t_kp1]. Degree = 6 × 2 = 12 per piece.
//   By fit_x_to_arc_length_piece's contract, outer.u_start = s_k = inner.evaluate(t_k),
//   so the affine-alignment precondition holds.
let composed: [BezierPiece<f64>; 3] = nurbs::algebra::compose_vector_piece(
    &[&outer[0], &outer[1], &outer[2]],
    &inner,
)?;
```

The caller iterates this over all N TOPP-RA grid pieces and stitches the per-axis composed pieces into a multi-piece NURBS via `bezier::bezier_pieces_to_nurbs`. The arc-length table is built *once* per parent segment (in §5's splitter or as a Layer-3-loop precondition), reused across all N grid pieces.

### 4.7 Tests for `compose_vector_piece`

- **Identity composition.** `compose(outer, inner=identity)` == `outer` to f64 round-off, where `identity` is the linear polynomial `p(t) = t` represented as a `BezierPiece`. Note that under Pascal-shifted-monomial storage, `p(t) = coeffs[0] + coeffs[1] * (t - u_start)`, so identity requires `coeffs = [u_start, 1]` (not `[0, 1]` unless `u_start == 0`). Set `u_start` and `u_end` to `outer`'s s-domain endpoints for the test. Sanity check.
- **Linear inner (degree 1).** `compose(outer, inner=linear)` is equivalent to a parameter rescaling on `outer`. Compare against direct rescaled evaluation at sample points.
- **Cubic-outer × quadratic-inner.** The T-A case. Synthetic outer = cubic in s with monomial-shifted coeffs `[1, 2, 3, 4]`; synthetic inner = quadratic in t with coeffs `[0.1, 0.5, 0.9]`. Verify the affine-alignment precondition holds (outer's u_start matches inner.evaluate(inner.u_start)). Evaluate composition at 100 sample points; compare against direct evaluation `outer(inner(t))` computed via two separate `BezierPiece::evaluate` calls. Match to f64 ULP-tolerance.
- **Sympy cross-check.** Generate a sympy-symbolic version of the composition for a fixed input pair; verify the monomial-shifted coefficients of our compose primitive match the sympy-derived coefficients to 1e-12 absolute.
- **Vector composition consistency.** For D=3 axes, verify each per-axis output is bit-exact to the corresponding scalar composition (D=3 is just D parallel scalar applications).
- **Affine-alignment debug-assert.** Pass `outer` whose `u_start ≠ inner.evaluate(inner.u_start)`; verify debug-mode panics, release-mode produces (predictably wrong) output without panic.

### 4.8 Code size estimate (combined)

`compose_vector_piece`: ~150 LOC + ~80 LOC tests + ~50 LOC of fixture / cross-check helpers. ~280 LOC.

`fit_x_to_arc_length_piece`: ~120 LOC + ~80 LOC tests + ~30 LOC of fixture helpers. ~230 LOC.

**Combined Layer-0 prep code size**: ~510 LOC.

## 5. Layer 1 primitive: `split_segment_to_cap`

### 5.1 Purpose

Bound per-MCU-segment path length at a fixed cap (default 12.5 mm) so the post-shape per-axis NURBS fits within the bumped curve-pool slot's `MAX_KNOT_VECTOR_LEN` / `MAX_CONTROL_POINTS` budget on the H723. Operates at Layer 1 / `geometry::pipeline`, after `geometry::reduce` produces a `CubicSegment` and before it's emitted to Layer 2.

### 5.2 Interface

```rust
pub fn split_segment_to_cap(
    segment: &CubicSegment,
    max_arc_length_mm: f64,    // default 12.5 mm
) -> Result<Vec<CubicSegment>, SplitError>;

pub enum SplitError {
    /// Input violated the single-piece-cubic invariant (e.g., wrong degree, multi-piece NURBS,
    /// has weights). Should never happen if caller used `CubicSegment::try_new` correctly,
    /// but the splitter re-checks defensively in release.
    NotSinglePieceCubic,
    /// Arc-length-table build failed (degenerate curve, NaN inputs, etc.).
    ArcLengthTableBuildFailed { reason: &'static str },
}
```

- **`segment`**: a single-piece cubic Bézier `CubicSegment` with metadata (feedrate, e_mode, extrusion ratio, source range).
- **Output (Ok case)**: a vector of `CubicSegment`s, each with `arc_length ≤ max_arc_length_mm` and the parent's metadata propagated. Each child segment carries `Some(SplitInfo)` if the parent was split; `None` if passthrough.
- **Output (Err case)**: returned when the input invariant is violated or the arc-length table can't be built. Round-4 review (LOW-5) caught that the v1 signature `Vec<CubicSegment>` contradicted §5.4's claim of `Err` returns — corrected to `Result<Vec<...>, SplitError>` here.
- **`SplitError` completeness:** the splitter does not perform polynomial fitting (that's `fit_x_to_arc_length_piece` in Layer 0) or shaper convolution. So `SplitError` doesn't need a `ToleranceNotReached`-style variant — the only ways the splitter can fail are invariant violation or arc-length-table build failure. Both cases above are exhaustive.

### 5.3 Algorithm

Adapts the existing `algebra.rs::refine_pieces_to_breakpoints` global-domain split pattern (per Codex review confirmation that this is the right primitive vs. de Casteljau parameter-rescaling bookkeeping). Round-1 review (codex CRITICAL 2 + MEDIUM 1) flagged two boundary-handling concerns folded into the algorithm below:

1. **Zero-length / degenerate-input passthrough (FIRST).** Round-2 review (codex HIGH) caught that gating on `e_mode == INDEPENDENT` would let helical segments bypass the cap. Round-3 review (codex HIGH-1) caught that chord-length-only gating would let a *closed or near-closed cubic loop* (chord ≈ 0, arc length large, real motion) silently bypass — though uncommon in slicer output, it's theoretically possible and would blow the curve-pool budget. Revised gate uses **control-polygon length** as the zero-motion proxy, which is robust against loops:
    - Compute `cp_polygon_length = ‖P_1 − P_0‖ + ‖P_2 − P_1‖ + ‖P_3 − P_2‖` for the cubic's four control points in XYZ.
    - Compute `midpoint_speed = ‖xyz'(0.5)‖` (parametric-speed at u = 0.5).
    - Fast-path passthrough applies if and only if **both** are below thresholds: `cp_polygon_length < ε_cp_polygon` (default `3e-6 mm` ≈ ε per CP gap) AND `midpoint_speed < MIN_PARAMETRIC_SPEED`. Both conditions together robustly catch retraction / prime / pathological near-zero inputs without false-positives on closed loops. Closed loops have nonzero `cp_polygon_length` (the control polygon traces around the loop), so they fall through to step 2 and split per actual arc length.
    - Helical extrusion (XY motion + Z motion + E motion in same segment) does NOT reach the splitter at all — it's rejected upstream by `CubicSegment::try_new` per §6.1's `GeometryError::HelicalExtrusionUnsupported` row. The splitter never sees a helical-Independent segment in the live pipeline.
2. **Build the arc-length table for the parent cubic once** via `nurbs::arc_length::build_arc_length_table_vector`. Compute total `L = table.s_max()`.
3. If `L ≤ max_arc_length_mm`: return `vec![segment.clone()]` with `SplitInfo: None`. No work.
4. Compute `k = ⌈L / max_arc_length_mm⌉` and the `k-1` target arc-lengths `[L/k, 2L/k, …, (k-1)L/k]`.
5. Convert each target to a parameter `u_i` via `nurbs::arc_length::param_from_arc_length(table, target_s)`. Outputs are sorted (table is monotone-by-construction). Note that `param_from_arc_length` clamps endpoints in release mode (per `arc_length.rs:155`), so a target near `L` could return `u = u_end` exactly.
6. **Epsilon-filter the breakpoints** to satisfy `split_piece_at`'s strict-interior assertion (`bezier.rs:212` requires `u_split > piece.u_start && u_split < piece.u_end`). Define `EPS_U = 1e-9`. For the carried `current` piece on each iteration:
    - If `u_i ≤ current.u_start + EPS_U`: skip this break (the next sub-segment would be near-zero-length; merge into the current).
    - If `u_i ≥ current.u_end − EPS_U`: skip this break (current piece already extends to or near the break; finish current and emit).
    - Otherwise: split at `u_i`.
   If filtering reduces the effective break count below `k − 1`, the actual `sub_count` in the produced `SplitInfo`s is the realized count, not the originally-planned `k`. Document this in `SplitInfo`'s doc-comment.
7. Walk the global-domain split pattern, carrying the right piece forward:
   ```rust
   let mut current = parent_piece;
   let mut emitted = Vec::with_capacity(k);
   for &u in &u_breaks_filtered {
       // Defensive: filter step (6) should have already excluded near-boundary u values, but
       // re-check here for paranoia in case parent_piece's effective bounds shifted.
       if u <= current.u_start + EPS_U || u >= current.u_end - EPS_U {
           continue;
       }
       let (left, right) = nurbs::bezier::split_piece_at(&current, u);
       emitted.push(left);
       current = right;
   }
   emitted.push(current);
   ```
8. Wrap each piece as a `CubicSegment` with parent metadata propagated, plus `SplitInfo { sub_index: i, sub_count: emitted.len(), s_lo_mm: arc_length_at_u_start(piece), s_hi_mm: arc_length_at_u_end(piece) }`. Source range preserved unchanged on each child. Compute `s_lo_mm` / `s_hi_mm` by querying the arc-length table at each piece's `u_start` / `u_end` — *not* by reusing the originally-planned `i × L/k` targets, which may not match after epsilon filtering.

### 5.4 Edge cases

- **Retraction / prime / filament-change with no XYZ motion:** `cp_polygon_length < ε_cp_polygon` AND `‖xyz'(0.5)‖ < MIN_PARAMETRIC_SPEED`; passthrough at step 1, *before* arc-length-table build. (This is the typical INDEPENDENT case; see round-3 fix for why the gate is on control-polygon length + midpoint speed instead of `e_mode == INDEPENDENT` or chord length alone.)
- **Helical extrusion (XYZ motion + E motion in same segment):** rejected before reaching the splitter — see §6.1's classification table row that returns `Err(GeometryError::HelicalExtrusionUnsupported)`. The splitter never receives a helical-extrusion segment in the live pipeline. (Step 13's compat layer can convert helical extrusion to `Independent` with explicit E NURBS upstream — that conversion happens before the live pipeline sees the segment.)
- **Degenerate XYZ geometry (e.g., all control points collinear at the same point, or near-zero-length cubic):** `build_arc_length_table_vector` returns `DegenerateCurve`; step 1's parametric-speed check catches this before table build.
- `L < max_arc_length_mm`: passthrough, `SplitInfo: None`.
- `L = max_arc_length_mm` exactly: passthrough (single segment of exactly the cap length).
- `L = max_arc_length_mm + ε`: produces 2 sub-segments; both ≤ cap.
- `L = k × max_arc_length_mm` exactly: produces k sub-segments of equal length.
- **Near-endpoint targets after epsilon filter:** if a target arc-length lands `< EPS_U` from `current.u_start` or `current.u_end`, the break is dropped (per §5.3 step 6). Effective sub-segment count may be < planned k. `SplitInfo.sub_count` reflects the *realized* count, not the planned one.
- **Multi-piece input:** out-of-spec; primitive panics in debug mode or returns `Err(SplitError::NotSinglePieceCubic)` in release mode (per the §5.2 signature; defensive re-check beyond `CubicSegment::try_new`'s invariant).

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
- **Pure E-only `Independent` passthrough.** `e_mode = INDEPENDENT` with all four XYZ control points coincident (cp_polygon_length ≈ 0, midpoint_speed ≈ 0) and a non-trivial `e_independent` E NURBS. Verify fast-path passthrough at step 1; arc-length table NOT built; SplitInfo: None. (This is the canonical retraction / prime / filament-change case — `Independent` segments in the live pipeline never have real XYZ motion, because helical extrusion is rejected upstream per §6.1.)
- **Closed-loop edge case.** Cubic Bézier with P_0 ≈ P_3 (chord ≈ 0) but non-trivial control polygon (cp_polygon_length > threshold) and arc length L > cap. Verify fast-path is NOT taken (chord-only gate would have failed here); segment splits per actual arc length. Confirms the round-3 fix that gates passthrough on cp_polygon_length, not endpoint chord.
- **Helical-extrusion rejection regression test.** Construct a `CubicSegment` builder that would attempt to classify a helical-extrusion input. Verify `CubicSegment::try_new` returns `Err(GeometryError::HelicalExtrusionUnsupported)`. The splitter is never reached.

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
    pub e_mode: EMode,
    pub extrusion_per_xy_mm: f64,         // valid when e_mode is CoupledToXy or Travel
    pub e_independent: Option<ScalarNurbs<f64>>,  // Some(curve) iff e_mode is Independent
    pub feedrate_mm_s: f64,
    pub source: SourceRange,
    pub split_info: Option<SplitInfo>,    // None on un-split, Some on splitter output
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EMode {
    /// Extrusion proportional to actual XY shaped motion: `e_actual(t) = ratio × ∫|v_xy| dt`.
    /// `extrusion_per_xy_mm` is nonzero and signed (positive for normal extrusion;
    /// negative for retract-during-XY-motion / wipe / coast). Used for moves with
    /// `ΔXY > ε_xyz`, `ΔZ ≤ ε_z`, and `abs(ΔE) > ε_e`.
    CoupledToXy,
    /// Travel move: XY motion with no extrusion. Equivalent to `CoupledToXy` with
    /// `extrusion_per_xy_mm = 0`. Modeled as a distinct variant for clarity in logs/telemetry
    /// and to allow a future plan layer to skip per-sample E integration when the ratio is
    /// definitionally zero. Used for G5 with Δ_XY > 0 and Δ_E = 0.
    Travel,
    /// E motion not coupled to XY: own E NURBS carries the trajectory in time. **In
    /// 7-pre's live pipeline, `Independent` always implies null `xyz` motion**
    /// (`cp_polygon_length < ε_cp_polygon` AND midpoint speed below `MIN_PARAMETRIC_SPEED`)
    /// — used for retraction, prime, filament-change moves with no XY motion
    /// (`Δ_XY ≈ 0` and `Δ_E ≠ 0`). Helical extrusion (XYZ motion + E motion) is
    /// rejected upstream by `CubicSegment::try_new` via `GeometryError::HelicalExtrusionUnsupported`
    /// and never produces an `Independent` segment in 7-pre. Step-13's compat layer is
    /// out of scope here; whatever extension Step-13 makes to the `Independent` invariant
    /// is Step-13's spec to define, not 7-pre's.
    Independent,
}
```

**Classification rules** (Layer 1 reduce stage applies these when constructing `CubicSegment` from G5/G5.1 input). Round-2 review (codex CRITICAL + HIGH on signed-E) corrected v1's two correctness bugs: (a) the extrusion-ratio denominator must be the *XY arc length of the cubic Bézier*, not the endpoint chord (chord ≠ arc length for curved cubics; using chord would over-/under-extrude on heavily-curved G5s); (b) classification must use `abs(ΔE)`, not `ΔE`, otherwise XY+retract wipe/coast moves with negative ΔE silently lose their commanded E motion.

| ΔXY (mm) | ΔZ (mm) | abs(ΔE) (mm) | e_mode | extrusion_per_xy_mm | e_independent |
|---|---|---|---|---|---|
| > ε_xyz | ≤ ε_z | > ε_e | `CoupledToXy` | `ΔE / xy_arc_length(xyz)` *(signed; can be negative for retract-during-motion)* | `None` |
| > ε_xyz | > ε_z | > ε_e | **`Err(GeometryError::HelicalExtrusionUnsupported)`** *(round-4 fix; see note below)* | — | — |
| > ε_xyz | any | ≤ ε_e | `Travel` | `0.0` | `None` |
| ≤ ε_xyz | > ε_z | > ε_e | `Independent` | `0.0` (unused) | `Some(linear_e_curve)` |
| ≤ ε_xyz | ≤ ε_z | > ε_e | `Independent` | `0.0` (unused) | `Some(linear_e_curve)` |
| ≤ ε_xyz | > ε_z | ≤ ε_e | `Travel` | `0.0` | `None` |
| ≤ ε_xyz | ≤ ε_z | ≤ ε_e | (rejected as zero-motion segment) | — | — |

Thresholds (`ε_xyz`, `ε_z`, `ε_e`) are configurable; defaults `ε_xyz = ε_z = ε_e = 1e-6 mm`. **Threshold semantics:** Δ-quantities are *endpoint-delta* magnitudes (e.g., `ΔZ = |xyz_end[2] − xyz_start[2]|`), not accumulating drift across the curve's interior. The 1 µm threshold is comfortably above f64 round-off accumulating across normal slicer-emitted G5 endpoint coordinates and well below any deliberately-emitted Z lift.

**`xy_arc_length(xyz)` computation:** the XY arc length of the cubic Bézier (length of its projection onto the machine X/Y plane — *not* the G-code active-plane setting; kalico's machine frame is always X/Y for shaping and CoreXY transform purposes). Concrete API:

```rust
pub fn xy_arc_length<const D: usize>(xyz: &VectorNurbs<f64, D>) -> f64
where
    // D ≥ 2 (X is index 0, Y is index 1 by codebase convention).
{
    // 5-point Gauss-Legendre quadrature integrating sqrt(x'(u)² + y'(u)²) over u ∈ [0, 1].
    // No table built; this is a scalar one-shot integration. Cheaper than the 3D table.
    // For pure-XY moves (Δz = 0), result equals the 3D arc length to f64 round-off.
}
```

This lives in `nurbs::arc_length` alongside `build_arc_length_table_vector`. Sibling, not extension — it's a separate scalar query, no table rebuild needed. Implementation cost: ~30 LOC + ~30 LOC tests:

- **Pure XY (Δz = 0):** `xy_arc_length` matches the 3D arc length to f64 round-off.
- **Constant-pitch helix fixture:** for a curve where Z increases linearly with arc length and XY traces a fixed-radius arc with constant tangential speed (the `√(L_3d² − L_z²)` identity *would* hold for this special case — but the v1 test text claimed this for general helical input, which is false; the round-4 review caught this. Use only the special-case fixture and document the constant-direction-ratio assumption explicitly.).
- **General helical (varying Z-XY direction ratio):** *direct numerical projection fixture* — choose a known cubic Bézier with hand-computed XY-projection arc length (computed via independent quadrature); compare `xy_arc_length`'s output to the hand-computed value.
- **Loops / closed curves:** verify XY projection length is non-zero for a closed cubic loop in the XY plane (chord ≈ 0 but real motion).

**Why XY arc length not 3D arc length** (for the extrusion ratio): the slicer's commanded ΔE per move is per-XY-distance (extrusion volume scales with in-plane motion, not Z lift). For helical extrusion (rare; Z motion mid-extrusion-stroke), some slicers use 3D-distance basis instead — those slicers' helical segments would be silently mis-extruded if the live pipeline accepted them as `CoupledToXy`.

**Round-4 fix (MEDIUM): live pipeline rejects helical-extrusion outright** rather than silently mis-classifying. The new table row above with `ΔXY > ε_xyz && ΔZ > ε_z && abs(ΔE) > ε_e` returns `Err(GeometryError::HelicalExtrusionUnsupported)` from `CubicSegment::try_new`. Users encountering this error must either (a) re-slice without helical extrusion (most slicers don't emit it by default), or (b) wait for the Step-13 compat layer's helical-detection override which converts to `Independent` with an explicit E NURBS (Step-13 scope, not 7-pre). The error message includes a clear path forward: `"helical extrusion (XY motion + Z motion + E motion in same segment) not yet supported in live pipeline; pre-process via Step-13 compat layer or disable helical extrusion in slicer."`

**Note on negative `extrusion_per_xy_mm`:** wipe/coast/retract-during-XY-motion is a real slicer pattern. The signed ratio `ΔE / xy_arc_length` is preserved (negative when retracting); MCU per-sample integration `e_acc += ratio × v_xy × dt` handles it correctly because `v_xy ≥ 0` and `ratio` carries the sign.

**Pure-Z-with-extrusion row:** Z motion without XY motion — routed through `Independent` because `CoupledToXy` would integrate `v_xy = 0` and produce no extrusion (wrong if the slicer commanded ΔE ≠ 0).

**Splitter propagation rules** (when `split_segment_to_cap` produces multiple sub-segments from one parent):

- `e_mode`, `extrusion_per_xy_mm`: copied unchanged on each sub-segment. **Conservation argument** (clarified per round-3 review codex MEDIUM-3): the splitter splits by 3D arc length (since the curve-pool budget is what we're capping, and 3D arc length is what determines piece count after time-reparameterization). The ratio is in XY-arc-length units. The MCU integrates `e_acc += ratio × |v_xy| × dt` per sample regardless of how the parent was split — total extrusion across sub-segments = `ratio × Σ child_xy_arc_lengths`. Child XY arc lengths sum to parent's XY arc length by additivity of arc length over a partition. So total extrusion is preserved. The 3D-vs-XY split-basis mismatch is harmless because the conservation is over XY arc length on both sides of the equation. **Note**: for `CoupledToXy` segments the parent has `ΔZ ≤ ε_z` per the §6.1 classification rule (the helical row is rejected upstream), so 3D and XY arc lengths are equal anyway — the conservation argument is the same in either domain.
- `e_independent`: in the live pipeline, `Independent` segments are only retraction / prime / filament-change moves with `cp_polygon_length < ε_cp_polygon` — they never reach the splitter (caught by §5.3 step 1's fast-path). So splitter `e_independent` propagation is a no-op in practice. Defensive code that subdivides `e_independent` proportionally is not required for 7-pre's MVP; the splitter can debug-assert that any `Independent` segment it sees has already been fast-pathed (i.e., it should never reach the multi-piece-emission code path with `e_mode == INDEPENDENT`).
- `feedrate_mm_s`, `source`: copied unchanged.
- `split_info`: populated per §5.3 step 8.

```rust
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SplitInfo {
    /// 0-indexed position of this child within the parent's sub-segment sequence.
    pub sub_index: u32,
    /// Total sub-segments produced from the parent. May be < the originally-planned `k`
    /// if epsilon-filtering at §5.3 step 6 dropped near-boundary breakpoints.
    pub sub_count: u32,
    /// Arc-length range this sub-segment occupies in the parent's arc-length domain.
    /// Computed at split time by querying the parent's arc-length table at the child's
    /// `xyz.u_start` and `xyz.u_end`.
    pub s_lo_mm: f64,
    pub s_hi_mm: f64,
}

impl CubicSegment {
    pub fn try_new(
        xyz: VectorNurbs<f64, 3>,
        e_mode: EMode,
        extrusion_per_xy_mm: f64,
        e_independent: Option<ScalarNurbs<f64>>,
        feedrate_mm_s: f64,
        source: SourceRange,
        split_info: Option<SplitInfo>,
    ) -> Result<Self, GeometryError> {
        // Validates: xyz is single-piece cubic (degree 3, 4 CPs, no weights, clamped knots).
        // Validates: e_mode == Independent ⇔ e_independent.is_some().
        // Validates: e_mode != CoupledToXy ⇒ extrusion_per_xy_mm == 0.0 (or unused).
        // Returns Err(NotSinglePieceCubic | EModeInvariantViolation | ...) on violation.
        ...
    }
}
```

**Removed fields:** `degree` (always 3, redundant), `max_residual_mm` (only meaningful for fitter output, which moves to Step 13 compat layer; live pipeline doesn't fit). **Renamed/added:** `e` → `e_mode + extrusion_per_xy_mm + e_independent` (per Section A's E-follows-XY architecture from round-1 brainstorm, with the explicit classification rules above); `split_info` (per Section C splitter output).

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
- Live `reduce.rs` accepts only G5 / G5.1 token kinds. G5 → cubic Bézier direct. G5.1 → cubic via exact Bernstein degree-elevation per §6.5 (degree 2 → 3, +1 control point, no fit error).
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
- **Composed-NURBS shape end-to-end.** For a test input (synthetic G5 + synthetic TOPP-RA-output `b(s)`), run the full T-A pipeline (`fit_x_to_arc_length_piece` per TOPP-RA grid piece + `compose_vector_piece` against the closed-form quadratic `s(t)`). Verify the resulting per-axis NURBS is degree-12 per piece pre-shape (`fit.degree() = 6` × `s(t).degree() = 2`), multi-piece across the segment, with continuity at TOPP-RA-grid joints. Verify max position error against direct evaluation `x(u(s(t)))` is below the configured tolerance. *Note: this is a 7-A integration test that depends on 7-pre primitives — co-developed with 7-A.*

### 7.3 Regression tests

Round-1 review (codex HIGH 4) flagged that the test blast radius is significantly larger than v1 framing implied — existing tests directly exercise Phase-1 behavior on >100k `FittedSegment`s and >5k `ArcSegment`s in the OrcaSlicer corpus tests, and `multi_segment.rs` fixture 6 *intentionally* mixes degree-1, cubic, and rational-quadratic curves to exercise SLP behavior on rational arcs. Blanket `#[ignore]`-ing those tests would lose real coverage. Revised disposition:

**Strategy: split test suites into two named groups, neither one disabled, both maintained.**

- **`live-pipeline-g5-only` test suite (default `cargo test`):**
  - All compose / splitter / CubicSegment unit tests from §7.1.
  - `multi_segment.rs` fixtures rebuilt with synthetic G5-only inputs (cubic Béziers replacing G2/G3 → rational quadratics where the original test was about Layer 2 algorithms — junction-velocity continuity, lookahead-window joining, limit-change invalidation — which are curve-type-agnostic). Each rebuilt fixture documents in a comment what the original G2/G3 fixture exercised and confirms the cubic-Bézier replacement preserves the algorithmic intent.
  - Integration-test on the OrcaSlicer corpus: tests run on G5-normalized output produced by the future Step-13 compat layer. *During 7-pre's development, the corpus tests are gated behind a `step-13-available` cfg flag and run only when the compat layer is wired up.* Default: skipped with a clear comment pointing to the Step-13 dependency.
- **`legacy-reference` test suite (gated, `cargo test --features legacy-reference`):**
  - The original G1 / G2 / G3 reduce tests (e.g., `g5_reduction.rs`'s G2/G3 cases, `multi_segment.rs` fixture 6's rational-quadratic SLP behavior) preserved as a reference suite.
  - Compiles only when `legacy-reference` feature is enabled. When the Step-13 compat-layer crate lands, those tests *move* to that crate; until then, they live in `geometry::tests::legacy` (or similar) with documentation that they exercise the compat-layer's reduction code.
- **`fixture-6-rational-arc` (Layer 2 SLP behavior on rational arcs):** kept in `legacy-reference` as a *reference* of how the SOCP/SLP behaves on rational geometry. Live pipeline doesn't see rationals, but the test serves as documentation for Step-13 compat-layer designers about the algorithmic edge cases.
- **OrcaSlicer-corpus tests (`integration_orca.rs`):** the >100k `FittedSegment` + >5k `ArcSegment` assertion content moves to the `legacy-reference` suite, with a parallel `integration_orca_g5_normalized.rs` added to the `live-pipeline-g5-only` suite (to be populated when Step 13 compat layer can produce G5-normalized output of the corpus).
- **Implementer ergonomics:** `cargo test` runs the live suite, `cargo test --features legacy-reference` adds the legacy suite. CI runs both. When Step 13 lands, the gated suite migrates to the compat-layer crate cleanly.

## 8. Implementation sequence

Round-1 review (codex MEDIUM 3) flagged that the v1 ordering had an implicit dependency hazard: the splitter was listed as parallelizable with the structural rename, but it operates on `CubicSegment` (post-rename) and works against the live reduce stage's output. If splitter lands before live-reduce-cleanup, it can encounter `CurveGeom::Linear / Quadratic / RationalQuadratic` from the existing G0/G1/G2/G3 paths (`pipeline.rs:220`) — which would silently mis-handle. Revised ordering:

**Phase 1: structural foundation (must land together).**
1. **Layer 1 structural rename** (`FittedSegment` → `CubicSegment` with the new E-mode shape, retire `ArcSegment`, simplify `Segment` enum). Touches `segment.rs` + every callsite. The classification rules (§6.1) and `try_new` invariant land here.
2. **Layer 1 G5.1 → cubic degree-elevation** in `reduce.rs` (per §6.5).
3. **Layer 1 live-reduce rejection** of G0/G1/G2/G3 (per §6.3 — those tokens become `GeometryError::UnsupportedGcode`). The legacy code is preserved per Q3's option 1.
4. **`SplitInfo` type addition** to `segment.rs`.

These four changes form one cohesive PR. After this lands, the live pipeline accepts only G5/G5.1 and emits only `CubicSegment`s — no rationals, no mixed-degree dispatch, no source-gcode-type special-cases.

**Phase 2: parallel-able primitives (after Phase 1).**
5. **Layer 0 `compose_vector_piece`** primitive. Independent of all Layer 1 changes (operates on `BezierPiece` directly); could even start before Phase 1 if pure-algebra unit testing is sufficient. Co-developable in parallel with #6.
6. **Layer 0 `fit_x_to_arc_length_piece`** primitive. Independent of all Layer 1 changes (operates on `VectorNurbs` + `ArcLengthTable`). Co-developable in parallel with #5.
7. **Layer 1 `split_segment_to_cap`** primitive. Operates on the post-Phase-1 `CubicSegment`. Lands after Phase 1.

**Phase 3: integration (after Phase 1 + Phase 2).**
8. **Test suite split** per §7.3 (live-pipeline-g5-only vs. legacy-reference); legacy tests gated behind `legacy-reference` feature flag.
9. **Integration sanity test:** synthetic G5 → reduce → splitter → Layer 2 (`plan_batch`). Verifies the end-to-end shape of the pipeline works with the new types.

**Effort estimate:** ~2.5 weeks for one developer (~5 working days for Phase 1 + the test-suite split, ~5 days for Phase 2's two primitives in parallel-developable chunks, ~3 days for Phase 3 integration + slack).

## 9. Open questions / assumptions

- **Q1: Should `compose_vector_piece` live in `nurbs::algebra` or a new `nurbs::composition` module?** Defaulting to `nurbs::algebra::compose_vector_piece` for simplicity (one less module). Implementer can split into a separate module if the algebra module grows past readable size during 7-pre.
- **Q2: Should the runtime assertion on `CubicSegment` invariants live at `try_new` time only, or be re-checked at every primitive entry?** Defaulting to constructor-only (`try_new`). The type is constructed once and consumed by primitives; re-checking at primitive entry would be paranoid. Document the invariant clearly in the type's doc-comment.
- **Q3: Disposition of `geometry::reduce`'s G0/G1/G2/G3 code:** comment-out-and-preserve (§6.3 option 1) vs. move-to-new-crate (option 2). Defaulting to option 1 for 7-pre's minimal-diff scope; revisit when Step 13 is actively brainstormed.
- **Q4: Should the compat-layer's existence be advertised in error messages?** When the live parser sees G0/G1/G2/G3, the error message should mention "run input through Step-13 compat layer." Stronger: maybe *recommend* a specific tool or command. Defer the exact wording to implementation; the architectural decision is just that the error is *expressive* not silent.
- **Q5: T-A precision-caveat documentation update.** §4.5 documents the per-piece adaptive polynomial fit (`fit_x_to_arc_length_piece`) with sample-verified position error — *not* a sub-µm linear-u(s) approximation, which round-1 review refuted. CLAUDE.md's existing T-A bullets should be updated to reflect that T-A's "math-exact" framing was wrong: actual T-A is "polynomial fit per TOPP-RA grid piece, sample-verified at configurable position-error tolerance (default 1 µm)." Defaulting to "fold this into 7-A's spec" since 7-A is where the CLAUDE.md re-wording naturally lives — 7-pre's plan-changes-log entry will record the round-1 review's finding so 7-A's author has the context.
- **Q6: Knot-multiplicity cleanup pass.** Codex flagged that `bezier_pieces_to_nurbs` emits knots at multiplicity p (degree), giving C0 stitching even when math says C¹. Not a correctness issue — extra storage. Defer to a small follow-up after 7-pre lands; not blocking.
- **Q7: Certified L∞ bound on `fit_x_to_arc_length_piece` residual.** §4.5 step 4 uses an empirical sample-based bound (4·(d+1) oversampling). For pathologically-curved inputs that don't exist in real FDM workloads, residual peaks could in principle hide between samples. A rigorous follow-up would compute the residual function in Bernstein form and bound L∞ via the convex hull of its Bernstein coefficients (per Piegl & Tiller §5.2's convex-hull property). This upgrades the empirical bound to a certified L∞ at the cost of ~50 extra LOC. Defer to a small follow-up if pathological inputs ever surface in field telemetry; not blocking 7-pre.

## 10. References

- CLAUDE.md (2026-04-29 state) — feature scope, Layer 1, build-order Step 7 / Step 13, critical-path observations.
- docs/superpowers/plan-changes-log.md — 2026-04-29 round-1 + round-2 entries documenting the brainstorm decisions, plus the round-1-review-applied entry that records the precision-claim correction.
- docs/research/bspline-polynomial-convolution.md — Minkowski-sum knot-vector growth from convolution; degree formula `d_input + d_kernel + 1`.
- docs/research/layer3-time-polynomial-fit-bounds.md — T-B verifier artifact; analysis of fit error scales for adaptive multi-piece polynomial approximation of x(t).
- docs/research/single-polynomial-fit-per-segment-conditioning.md — T-C verifier artifact (T-C refuted via Jackson convergence floor on C¹ x(t)).
- **docs/research/linear-us-approximation-cubic-bezier-error.md** — kalico-verifier round-1-review artifact (2026-04-29) refuting the v1 spec's sub-µm linear-u(s) approximation claim. Provides the analytical bound `ε_max ≤ Δs²/8 · max|v'/v²|` and concrete worst-case numerical examples (S-curve 4.4 µm, R=1mm arc 7.1 µm, near-cusp 100+ µm). Drove the §4.5 rewrite to `fit_x_to_arc_length_piece`.
- Round-1 codex review of the spec (2026-04-29) — flagged: BezierPiece basis (Pascal-shifted monomial, not Bernstein); split_piece_at strict-interior assert vs. param_from_arc_length endpoint clamping; G5.1 elevation contradiction; CubicSegment E-mode classification gaps; structural-refactor blast radius; zero-length splitting ordering; compose-caller affine-reparameterization missing; §8 ordering hazard. All applied in the round-1-applied edits.
- Codex review of Section B (composition primitive design, brainstorm phase) — flagged single-piece API, blossom alternative, C0-stitched-knot redundancy.
- Codex review of Section C (splitter design, brainstorm phase) — flagged use of existing `split_piece_at` + `refine_pieces_to_breakpoints` template, `param_from_arc_length` API name correction, `ArcLengthTable` caching, `CubicSegment` type concern.
- kalico-researcher CAGD literature survey (2026-04-29) — Sederberg-Kakimoto 1991, Wang-Sederberg-Chen 1997, Floater 1995/2006, Hu-Wang-Jin 2008, Lewanowicz-Woźny-Keller 2012/2015, Goldapp 1991, Vavpetič-Žagar 2015/2017, Kim 2023 hexic. Used to scope Section A's algorithm choice (eventually retired) and to confirm composition's substitute-and-collect approach as canonical.
- Rababah 2016, "The Best Uniform Cubic Approximation of Circular Arcs with High Accuracy" — primary CAGD-literature source confirming all GCk cubic-arc methods (including Goldapp 1991) optimize geometric L∞ position error, *not* parameterization-speed-uniformity. Contradicts a v1 claim that the round-1 verifier surfaced.
- Piegl & Tiller, *The NURBS Book* (2nd ed., 1997), §5.2 (Bernstein convex-hull property), §5.4 (knot insertion / degree elevation).
- Goldapp 1991, "Approximation of circular arcs by cubic polynomials," CAGD 8:227 — closed-form cubic Bézier approximation of circular arcs (used by Step 13 compat layer, not by 7-pre directly).
