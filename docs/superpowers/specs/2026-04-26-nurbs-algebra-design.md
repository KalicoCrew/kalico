# NURBS Algebra Library — v1 Design

**Date:** 2026-04-26
**Status:** Spec — design approved, implementation plan to follow
**Layer:** 0 (Mathematical foundations)
**Driver:** Layer 3 smooth-shaper pre-bake (CLAUDE.md build step 8)

## 1. Context

Layer 0 of the motion-planner rewrite is ~95% complete. NURBS evaluation
(`rust/nurbs/src/eval.rs`) and arc-length parameterization
(`rust/nurbs/src/arc_length.rs`) are production-ready. Three algebra primitives
remain stubbed in `rust/nurbs/src/algebra.rs`:

1. `convolve_with_polynomial_kernel` — load-bearing for Layer 3 smooth-shaper bake.
2. `multiply` — pointwise NURBS product (Piegl & Tiller Ch. 5).
3. `add` weighted case — homogeneous lift.

This spec covers items 1 and 2. Item 3 (weighted add) is deferred — no
identified downstream consumer in the rewrite, and homogeneous-lift
semantics aren't necessarily what callers want anyway.

## 2. Scope

**Approach:** Lean v1 — minimum primitives to unblock Layer 3 smooth-shaper
bake. Foundation primitives (knot insertion, Bézier extraction) built first;
multiply and convolve layered on top.

**In scope:**
- `KnotVector<T>` — separable type extracted from current bare `Vec<T>` in
  `ScalarNurbs`/`VectorNurbs`.
- `insert_knot` (Boehm), `remove_knot` (Tiller) — public knot manipulation.
- `BezierPiece<T>` — Pascal-shifted monomial-basis polynomial piece, with
  Bernstein interop.
- `extract_bezier_pieces`, `bezier_pieces_to_nurbs` — round-trip between NURBS
  and piecewise-Bézier representations.
- `multiply` — pointwise product of two scalar polynomial NURBS.
- `convolve` — convolution of a scalar polynomial NURBS with a piecewise
  polynomial kernel.
- Sympy-based oracle corpus + property tests + one Klipper cross-check.

**Out of scope (deferred to v1.5 or later):**
- Weighted (rational) NURBS addition via homogeneous lift.
- Rational input to multiply/convolve. Returns `RationalNotSupported` with
  `polynomial_refit` (a Layer 3 utility) as the workaround.
- Vector convenience wrappers (`vector_multiply`, `vector_convolve`). Layer 3
  N-D callers iterate per axis for v1.
- Oslo bulk knot insertion. Repeated single-insertion is O(degree²) per knot
  for our degree range (≤ 5), measured in microseconds.
- Degree elevation. Not on the v1 critical path.
- Direct B-spline-basis multiplication (Mørken blossom-based). Bezier-extract
  path is correct, well-documented, and adequate at our degree range.

## 3. Layer 3 contract clarification

The Layer 3 spec previously read "convolve base NURBS with polynomial kernel
analytically." This was ambiguous about parameterization. Resolution:

- **Layer 3 reparameterizes geometric NURBS to time** using v(s) from TOPP-RA
  before convolving. The s→t reparameterization is a piecewise polynomial
  composition (NURBS-of-piecewise-polynomial), which expands to ~3–7 pieces
  per segment per jerk-bound phase but stays piecewise polynomial.
- **Layer 0 `convolve` is domain-agnostic.** It convolves polynomial pieces
  with a polynomial kernel without knowing whether the input is
  parameterized in s, t, or anything else. The caller (Layer 3) chooses the
  domain.

Rationale: convolution is mathematically pure only in the time domain
(matches resonance-frequency math). Convolving in s with a velocity-varying
window is approximate at best and breaks down where it matters most — the
high-acceleration phase boundaries. The spec note has been amended in
`CLAUDE.md` to make the reparameterize-then-convolve stage explicit.

## 4. Module layout

Three new files. All host-only (`#[cfg(feature = "host")]` gated).

```
rust/nurbs/src/
├── algebra.rs           ← expand: implement multiply + convolve
├── bezier.rs            ← NEW: BezierPiece type + extraction/recompose
├── knot.rs              ← NEW: KnotVector type + insert/remove
├── eval.rs              ← migrate find_knot_span → KnotVector::find_span
├── scalar.rs            ← refactor: knots: Vec<T> → KnotVector<T>
└── vector.rs            ← refactor: knots: Vec<T> → KnotVector<T>
```

## 5. Public API

### `knot.rs`

```rust
pub struct KnotVector<T: Float> {
    knots: Vec<T>,
}

impl<T: Float> KnotVector<T> {
    pub fn try_new(knots: Vec<T>) -> Result<Self, KnotError>;
    pub fn as_slice(&self) -> &[T];
    pub fn len(&self) -> usize;
    pub fn find_span(&self, u: T, degree: u8) -> usize;
    pub fn multiplicity_at(&self, u: T) -> usize;

    /// Crate-internal helper used by `extract_bezier_pieces`. Returns a knot
    /// vector with every interior breakpoint raised to multiplicity = degree.
    pub(crate) fn refined_to_full_multiplicity(&self, degree: u8) -> Self;
}

/// Boehm knot insertion. Inserts ū with the given multiplicity into a
/// curve, producing a new curve with `multiplicity` additional control
/// points. Geometrically invariant.
///
/// Errors:
/// - `BoundaryInsertion` if ū equals a clamped endpoint.
/// - `MultiplicityExceeded` if `existing + multiplicity > degree`.
/// - `OutOfRange` if ū is outside the knot vector domain.
pub fn insert_knot<T: Float>(
    curve: &ScalarNurbs<T>, u: T, multiplicity: usize,
) -> Result<ScalarNurbs<T>, KnotError>;

/// Tiller knot removal (Piegl & Tiller §5.4). Removes knot ū up to `count`
/// times if removal is possible within tolerance ε (chord-error in CP space).
/// Returns the new curve and the number of removals actually performed.
pub fn remove_knot<T: Float>(
    curve: &ScalarNurbs<T>, u: T, count: usize, tol: T,
) -> (ScalarNurbs<T>, usize);

pub enum KnotError {
    BoundaryInsertion,
    MultiplicityExceeded { existing: u8, requested: u8, max: u8 },
    OutOfRange,
    Invalid,  // non-monotone / wrong length on KnotVector::try_new
}
```

### `bezier.rs`

```rust
/// One Bézier piece as a polynomial in the *Pascal-shifted monomial basis*:
/// p(u) = Σ_{k=0..d} coeffs[k] * (u - u_start)^k
///
/// Chosen over Bernstein basis for two reasons:
/// (1) algebra ops (multiply, convolve) are coefficient arithmetic in this basis;
/// (2) reuses the analytical derivation in `bs_polynomial_composer.md`.
/// For external interop (visualization, serialization), use
/// `to_bernstein` / `from_bernstein`.
pub struct BezierPiece<T: Float> {
    pub u_start: T,
    pub u_end: T,
    pub coeffs: Vec<T>,  // length = degree + 1
}

impl<T: Float> BezierPiece<T> {
    pub fn degree(&self) -> usize;
    pub fn evaluate(&self, u: T) -> T;                // Horner, O(d)
    pub fn to_bernstein(&self) -> Vec<T>;             // O(d²)
    pub fn from_bernstein(bernstein: &[T], u_start: T, u_end: T) -> Self;

    /// Zero polynomial of the given degree on [u_start, u_end]. Used inside
    /// convolve as the accumulator for summing (i, j) contributions.
    pub fn zero(u_start: T, u_end: T, degree: usize) -> Self;
}

/// Sum two pieces with matching support. Used inside convolve to accumulate
/// (input piece, kernel piece) contributions to one output sub-interval.
/// Returns SupportMismatch if u_start/u_end differ.
impl<T: Float> std::ops::Add<&BezierPiece<T>> for &BezierPiece<T> {
    type Output = Result<BezierPiece<T>, AlgebraError>;
    fn add(self, rhs: &BezierPiece<T>) -> Self::Output;
}

/// Decompose a polynomial NURBS into its constituent Bézier pieces in
/// Pascal-shifted monomial form. Uses Boehm to raise interior knot
/// multiplicities to `degree`, then converts each Bernstein piece to monomial.
pub fn extract_bezier_pieces<T: Float>(
    curve: &ScalarNurbs<T>,
) -> Vec<BezierPiece<T>>;

/// Recompose contiguous Bézier pieces (in monomial form) into a single NURBS.
/// Inverse of extract_bezier_pieces. Panics if pieces are non-contiguous or
/// have inconsistent degrees.
pub fn bezier_pieces_to_nurbs<T: Float>(
    pieces: &[BezierPiece<T>],
) -> ScalarNurbs<T>;

/// Split a Bézier piece at an interior point `u_split`. Used by `multiply`
/// to refine pieces to a common breakpoint set.
pub fn split_piece_at<T: Float>(
    piece: &BezierPiece<T>, u_split: T,
) -> (BezierPiece<T>, BezierPiece<T>);
```

### `algebra.rs`

```rust
// Existing — kept as-is.
pub fn scalar_multiply<T: Float>(curve: &ScalarNurbs<T>, scalar: T) -> ScalarNurbs<T>;
pub fn add<T: Float>(a: &ScalarNurbs<T>, b: &ScalarNurbs<T>)
    -> Result<ScalarNurbs<T>, AlgebraError>;  // unweighted only; weighted returns NotImplemented

// New — replaces the existing NotImplemented stub.
pub fn multiply<T: Float>(
    a: &ScalarNurbs<T>, b: &ScalarNurbs<T>,
) -> Result<ScalarNurbs<T>, AlgebraError>;

/// Replaces the existing PolynomialKernel — the single-poly case is one piece.
pub struct PiecewisePolynomialKernel<T: Float> {
    pub pieces: Vec<BezierPiece<T>>,  // contiguous, ordered
}

impl<T: Float> PiecewisePolynomialKernel<T> {
    /// Build a single-piece kernel from monomial coeffs on the given support.
    /// Convenience for the bleeding-edge-v2 single-polynomial smoothers.
    pub fn single_poly(coeffs: Vec<T>, support: (T, T)) -> Self;
}

// New — replaces the existing NotImplemented stub.
pub fn convolve<T: Float>(
    curve: &ScalarNurbs<T>, kernel: &PiecewisePolynomialKernel<T>,
) -> Result<ScalarNurbs<T>, AlgebraError>;

/// Crate-internal post-pass used by both `multiply` and `convolve`. Iterates
/// over interior knots and applies `remove_knot` with a tight tolerance,
/// dropping knots whose removal preserves the curve within `tol`. Exposes
/// the natural smoothness of the result instead of carrying maximum-multiplicity
/// knots from the Bézier-piece recomposition.
pub(crate) fn knot_remove_redundant<T: Float>(curve: &mut ScalarNurbs<T>, tol: T);

pub enum AlgebraError {
    KnotMismatch,
    NotImplemented(&'static str),                       // kept for weighted-add stub
    RationalNotSupported {                              // NEW
        operation: &'static str,                        // "multiply" or "convolve"
        workaround: &'static str,                       // "use polynomial_refit (Layer 3 utility) before calling"
    },
    SupportMismatch,                                    // NEW — for BezierPiece add
}
```

## 6. Algorithms

### 6.1 Boehm knot insertion (`knot.rs`)

To insert ū with multiplicity m at span k (`U[k] ≤ ū < U[k+1]`):

1. Compute `s = multiplicity of ū already in U`.
2. Precondition check: `m + s ≤ degree`. Otherwise return `MultiplicityExceeded`.
3. Reject ū at clamped boundaries (`u == U[0]` or `u == U[last]`) → `BoundaryInsertion`.
4. Apply Piegl & Tiller A5.3 (fused multi-insertion). Single-insertion form:

   ```
   α_i = (ū - U[i]) / (U[i+degree] - U[i])
   Q[i] = (1 - α_i) · P[i-1] + α_i · P[i]    for i = k-degree+1 ..= k-s
   ```

   Note the upper bound is `k-s`, not `k`, when ū already has multiplicity s
   in the knot vector (avoids divide-by-zero on `U[i+degree] - U[i]`).

The new knot vector inserts ū at position k+1; existing knots shift right.
Other control points pass through.

**Reference:** Piegl & Tiller 2nd ed. §5.2 pp. 141–151, A5.1 p. 151, A5.3 p. 155.

### 6.2 Bézier extraction (`bezier.rs`)

```
extract_bezier_pieces(curve):
    refined = curve.knots().refined_to_full_multiplicity()
    aligned = curve_with_knots(curve, refined)
    for each interior breakpoint pair (u_i, u_{i+1}):
        bernstein_cps = aligned.control_points()[piece_range_i]
        coeffs = bernstein_to_monomial(bernstein_cps, u_i, u_{i+1})
        emit BezierPiece { u_start: u_i, u_end: u_{i+1}, coeffs }
```

`bernstein_to_monomial` formula (Pascal-shifted basis at `u_start`):

```
c_k = C(d, k) · Σ_{i=0..k} (-1)^{k-i} · C(k, i) · B_i / (u_end - u_start)^k
```

Pascal shift is numerically meaningful (not just bookkeeping): evaluating
`Σ c_k (u - u_start)^k` via Horner with small `(u - u_start)` avoids
catastrophic cancellation that plagues unshifted monomials. For segment
lengths < 1m and degrees ≤ 10, this is comfortably well-conditioned in f64.

`bezier_pieces_to_nurbs` is the inverse: per piece, `monomial_to_bernstein`,
then concatenate Bernstein control points (shared boundary CPs identified —
last CP of piece i equals first CP of piece i+1 by construction).

**References:** Piegl & Tiller §5.4 / A5.6 (Bézier decomposition); §1.1
(Bernstein/monomial); Farin §5.7. Higham *Accuracy and Stability of
Numerical Algorithms* §5 (Horner conditioning).

### 6.3 Multiply (`algebra.rs`)

```
multiply(a, b) -> ScalarNurbs:
    if a.has_weights() || b.has_weights():
        return Err(RationalNotSupported {
            operation: "multiply",
            workaround: "use polynomial_refit (Layer 3 utility) before calling",
        })

    a_pieces = extract_bezier_pieces(a)
    b_pieces = extract_bezier_pieces(b)

    // Refine to common breakpoint set
    breakpoints = sorted_union of {a piece breakpoints} ∪ {b piece breakpoints}
    a_refined = split_pieces_at(a_pieces, breakpoints)
    b_refined = split_pieces_at(b_pieces, breakpoints)

    out_pieces = for each (a_p, b_p) in zip(a_refined, b_refined):
        // Both polynomials in (u - u_start), same u_start, same u_end.
        // Standard polynomial coefficient convolution.
        out_coeffs = poly_multiply(a_p.coeffs, b_p.coeffs)  // O(d_a · d_b)
        BezierPiece { u_start: a_p.u_start, u_end: a_p.u_end, coeffs: out_coeffs }

    result = bezier_pieces_to_nurbs(&out_pieces)

    // Knot-removal post-pass: at a breakpoint where `a` has knot multiplicity
    // m_a and `b` has m_b, the product's natural multiplicity is
    // max(m_a + d_b, m_b + d_a). Remove redundant interior knots within tol.
    knot_remove_redundant(&mut result, tol = 1e-12)

    Ok(result)
```

**Continuity invariant:** if `a ∈ C^k, b ∈ C^l` at a breakpoint, then
`c = a·b ∈ C^min(k,l)`. Encoded via output knot multiplicity
`max(m_a + d_b, m_b + d_a)` per Mørken Thm 3.1.

**References:** Piegl & Tiller §5.6.3 (product of two B-splines), §5.4
(knot removal, Tiller's algorithm). Mørken K. (1991) "Some identities
for products and degree raising of splines" *Constructive Approximation*
7:195–208.

### 6.4 Convolve (`algebra.rs`)

```
convolve(curve, kernel) -> ScalarNurbs:
    if curve.has_weights():
        return Err(RationalNotSupported { operation: "convolve", ... })

    x_pieces = extract_bezier_pieces(curve)
    w_pieces = kernel.pieces

    // Output breakpoints: cross-sum of input and kernel breakpoints.
    // y(u) changes analytic form whenever any kernel boundary crosses any
    // input boundary, i.e. when u = x_b + w_b for some pair.
    out_breaks = sorted_dedupe({xb + wb : xb ∈ x_breaks, wb ∈ w_breaks})

    out_pieces = for each consecutive (α, β) in out_breaks:
        accum = BezierPiece::zero(α, β, degree = d_x + d_w + 1)
        for each (i, j) such that the integration range over u ∈ [α, β]
                                    is non-empty:
            // For u ∈ [α, β]: kernel piece j is active where
            // u - s ∈ [w_j.u_start, w_j.u_end], i.e. s ∈ [u - w_j.u_end, u - w_j.u_start].
            // Intersect with x_pieces[i] support to get integration limits
            // [s_lo(u), s_hi(u)]. Both linear in u by construction of out_breaks.
            contribution = integrate_product_piece(x_pieces[i], w_pieces[j], α, β)
            accum = (&accum + &contribution)?  // same-support polynomial add
        accum

    result = bezier_pieces_to_nurbs(&out_pieces)

    // Knot-removal post-pass: convolution of C^a with C^b gives C^(a+b+1)
    // (gain one order from integration). Output as constructed carries
    // multiplicity-1 interior knots that are smoother than the basis demands.
    knot_remove_redundant(&mut result, tol = 1e-12)

    Ok(result)
```

`integrate_product_piece(x_i, w_j, α, β)`:

1. Express `w_j(u-s)` in the s-basis with coefficients polynomial in u
   (binomial expansion of `(u - s - w_j.u_start)^k` terms).
2. Multiply by `x_i(s)` as polynomial in s with u-dependent coefficients.
   Reuses `poly_multiply` from `multiply`.
3. Integrate `s^k → s^{k+1}/(k+1)`, evaluate at the integration limits
   `s_lo(u), s_hi(u)`. Both are linear in u, so `s_lo^{k+1}(u)` is polynomial
   in u of degree k+1.
4. Result: polynomial in u of degree `d_x_i + d_w_j + 1` over `[α, β]`,
   in Pascal-shifted basis at α.

**Output piece count:** ~`N_x + N_w − 1` (sliding-window argument,
consistent with `B_m * B_n` cardinal-spline convolution).

**Output domain:** `[x_min + w_min, x_max + w_max]` (Minkowski sum of
input and kernel supports). Layer 0 contract: zero-extend input outside
its support; caller (Layer 3) handles cross-segment stitching via overlap-add.

**References:** `docs/superpowers/plans/plan8-research/bs_polynomial_composer.md`
(this codebase's analytical derivation, magnum-opus branch). Unser, Aldroubi,
Eden "B-Spline Signal Processing" parts I/II, IEEE TSP 1993. Schoenberg
*Cardinal Spline Interpolation* (SIAM 1973). de Boor *A Practical Guide to
Splines* §IX–X.

## 7. Complexity & sanity check

For the realistic case (input: 5-piece NURBS degree 3; kernel: bs-3 = 4-piece
degree 2 — most expensive smooth shaper):

- Output: ~16 breakpoints, ~15 sub-intervals, polynomial degree ~6 per piece,
  ~5 (i, j) contributions per sub-interval.
- Total: ~75 inner integrations per axis per move. ~15 ms per axis per move
  upper bound. Within receive-time budget for a 10 mm move at 1000 mm/s.

For the simpler case (input: 1-piece linear from G1; kernel: smooth_zv =
1-piece degree 4):

- Output: 2 sub-intervals (full inside is one piece; two boundary pieces).
- ~1 ms total.

Spline-fitter output (Layer 1) typically dominates piece count, not per-piece
cost.

## 8. Error handling

Two error types, separated by abstraction layer:

- **`KnotError`** — public, in `knot.rs`. Returned by user-facing knot ops.
- **`AlgebraError`** — public, in `algebra.rs`. Returned by user-facing algebra
  ops.

Internal call sites (algebra ops calling `KnotVector::insert` on intermediate
curves we constructed) use `.expect("algebra-internal: invariants guaranteed")`.
Failure here is a bug, not a runtime condition.

`remove_knot` returns `(curve, count_actually_removed)` — tolerance-bounded
partial removal is communicated via the count, not an error.

## 9. Validation strategy

Three test layers, in order of stringency:

### 9.1 Property tests (`rust/nurbs/tests/algebra_proptest.rs`)

Using `proptest` (already in dev-deps).

| Op | Properties |
|---|---|
| `insert_knot` | Eval at sample points unchanged after insertion (geometric invariance); CP count ↑ by inserted multiplicity |
| `extract_bezier_pieces` | Per-piece eval matches original at sample points; piece count = interior-breakpoint count + 1; round-trip extract → recompose preserves eval |
| `remove_knot` | If insert(u) then remove(u), get back the original (within tol); never raises smoothness above what's in the original |
| `multiply` | `(a·b).eval(u) == a.eval(u) · b.eval(u)`; degree = `d_a + d_b`; associativity, commutativity, distributivity over add; identity (×1.0) |
| `convolve` | `convolve(a+b, k) == convolve(a, k) + convolve(b, k)`; degree = `d_x + d_w + 1`; support = Minkowski sum; convolve with constant kernel = scaled integral of input |

### 9.2 Sympy oracle corpus (`rust/nurbs/tests/algebra_oracle.rs`)

Generated by `scripts/generate_algebra_corpus.py` (parallel to the existing
`generate_geomdl_corpus.py`). Sympy does exact symbolic NURBS multiplication
and polynomial convolution; we generate ~100–200 small fixtures (low-degree,
integer/rational coefficients) and the Rust harness compares `multiply` /
`convolve` output against the symbolic reference at sample points to ~`1e-12`
in f64.

### 9.3 Klipper cross-check (one focused test, `rust/tests/`)

Take a simple time-parameterized input trajectory (e.g., a single polynomial
accel phase), convolve with bleeding-edge-v2's `smooth_zv` kernel, and compare
against Klipper's existing runtime convolution path in
`klippy/chelper/integrate.c`. Both should produce the same shaped trajectory.

This is the highest-value test because it validates against a battle-tested
reference, not just a symbolic one.

**Out of v1 testing scope:** fuzzing, coverage targets, performance benchmarks.
Add later if needed.

## 10. Build order

Five steps. Each ends with green tests; each is its own commit-worthy unit.

1. **Refactor: extract `KnotVector<T>`** (~1 day). Replaces bare `Vec<T>` in
   `ScalarNurbs`/`VectorNurbs`/`eval.rs`. Migrate `find_knot_span` from
   `eval.rs` to `KnotVector::find_span`. Pure refactor — existing tests stay
   green, no behavior change. Resist the temptation to "while I'm in here,
   also fix that one thing in find_span." Any behavior change deserves its
   own commit with its own test justification.

2. **`knot.rs` primitives** (~2 days). `insert_knot` (Boehm with multiplicity
   guard + boundary rejection, fused multi-insertion form), `remove_knot`
   (Tiller), `refined_to_full_multiplicity` helper, multiplicity queries.
   Property tests for invariants.

3. **`bezier.rs` primitives** (~2 days). `BezierPiece` struct, `evaluate`
   (Horner), `to_bernstein` / `from_bernstein`, `Add` impl with
   `SupportMismatch` guard, `extract_bezier_pieces`, `bezier_pieces_to_nurbs`,
   `split_piece_at`. Property tests.

4. **`algebra.rs` ops + sympy oracle live** (~3 days). Implement `multiply`
   (replaces `NotImplemented` stub, includes knot-removal post-pass).
   Implement `convolve` (replaces stub, takes new `PiecewisePolynomialKernel`,
   includes knot-removal post-pass). Add `RationalNotSupported` and
   `SupportMismatch` variants. Build the sympy corpus generator and minimal
   Rust oracle harness (~10–20 fixtures) alongside, so the oracle is live
   during implementation, not after. End state: algebra ops correct against
   symbolic reference.

5. **Test infrastructure expansion** (~2 days). Corpus expansion (more
   fixtures, edge cases), Klipper cross-check integration test, harness
   polish. End state: all three validation tiers running CI.

**Total: ~10 working days (2 weeks).**

## 11. References

**Knot insertion / removal / Bézier extraction:**
- Piegl & Tiller, *The NURBS Book*, 2nd ed., Springer 1997.
  - §5.2 pp. 141–151, A5.1 p. 151, A5.3 p. 155 — Boehm knot insertion.
  - §5.4, A5.6 — Bézier decomposition.
  - §5.4 — Tiller knot removal.
  - §1.1 — Bernstein / monomial bases.
- Farin, *Curves and Surfaces for CAGD*, 5th ed., Morgan Kaufmann 2002.
  - §5.7 — monomial form of a Bézier curve.
  - §8.3–8.5 — knot insertion / extraction.
- Boehm W. (1980) "Inserting new knots into B-spline curves" *CAD* 12(4):199–201.

**Multiplication:**
- Piegl & Tiller §5.6.3 — product of two B-splines.
- Mørken K. (1991) "Some identities for products and degree raising of splines"
  *Constructive Approximation* 7:195–208.
- Che, Wang, Goldman (2011) "Computing the product of two B-splines" — blossom-based.

**Convolution:**
- `docs/superpowers/plans/plan8-research/bs_polynomial_composer.md` — analytical
  derivation in this codebase (magnum-opus branch).
- Unser, Aldroubi, Eden "B-Spline Signal Processing" parts I/II, IEEE TSP 1993.
- Schoenberg, *Cardinal Spline Interpolation*, SIAM 1973.
- de Boor, *A Practical Guide to Splines*, 2001 rev. ed., Ch. IX–X.

**Numerical conditioning:**
- Higham N. *Accuracy and Stability of Numerical Algorithms*, 2nd ed., SIAM 2002.
  - §5 — Horner / monomial evaluation.
- Farouki & Rajan (1987) "On the numerical condition of polynomials in
  Bernstein form" — relevant if we ever need higher degrees on wider domains.
