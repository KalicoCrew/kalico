# NURBS Algebra v1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement Layer 0 NURBS algebra primitives — knot insertion/removal, Bézier extraction, NURBS multiplication, and polynomial-kernel convolution — to unblock Layer 3 smooth-shaper pre-bake.

**Architecture:** Three new host-only modules (`knot.rs`, `bezier.rs`, expanded `algebra.rs`). Foundation primitives first (`KnotVector` type, Boehm insertion, Tiller removal, Bézier extraction in Pascal-shifted monomial basis), then layered ops (multiply, convolve). Sympy oracle corpus + property tests + one Klipper cross-check.

**Tech Stack:** Rust 2021, generic over `Float` trait (f64 host / f32 MCU), `proptest` for property tests, sympy + Python for symbolic oracle corpus. Klipper cross-check uses existing `klippy/chelper/integrate.c` as reference.

**Spec:** `docs/superpowers/specs/2026-04-26-nurbs-algebra-design.md`

**Branch:** Currently on `sota-motion`. Consider creating a worktree (`git worktree add ../kalico-algebra sota-motion-algebra`) if you want isolation; not required.

**Conventions:**
- TDD throughout — failing test first, run to confirm fail, minimal impl, run to confirm pass, commit.
- Run tests from `rust/`: `cargo test -p nurbs --features host <pattern>`.
- All new code is host-only (`#[cfg(feature = "host")]`).
- Commit after every passing test or short logical group. Never amend.

---

## Phase 1: KnotVector type extraction (refactor, ~1 day)

Pure refactor at the type-system level. No behavior change; existing tests stay green throughout.

### Task 1.1: Add `KnotVector<T>` type with `try_new` and basic accessors

**Files:**
- Create: `rust/nurbs/src/knot.rs`
- Modify: `rust/nurbs/src/lib.rs`

- [ ] **Step 1: Write the failing test**

Append to `rust/nurbs/src/knot.rs` (creating the file):
```rust
//! Knot vector type and host-only knot operations (insertion, removal, span queries).
//! See `docs/superpowers/specs/2026-04-26-nurbs-algebra-design.md` §4–§6.

#![cfg(feature = "host")]

use crate::{ConstructError, Float};

/// Owned knot vector. Validates `non-decreasing` invariant on construction.
/// Clamping and length-vs-degree invariants are enforced by `ScalarNurbs::try_new`
/// where applicable; this type holds knots independent of any single curve.
#[derive(Debug, Clone, PartialEq)]
pub struct KnotVector<T: Float> {
    knots: Vec<T>,
}

impl<T: Float> KnotVector<T> {
    pub fn try_new(knots: Vec<T>) -> Result<Self, ConstructError> {
        if knots.len() < 2 {
            return Err(ConstructError::KnotCountMismatch {
                expected: 2,
                got: knots.len(),
            });
        }
        for window in knots.windows(2) {
            if window[1] < window[0] {
                return Err(ConstructError::KnotsNotMonotone);
            }
        }
        Ok(Self { knots })
    }

    pub fn as_slice(&self) -> &[T] {
        &self.knots
    }

    pub fn len(&self) -> usize {
        self.knots.len()
    }

    pub fn is_empty(&self) -> bool {
        self.knots.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn try_new_accepts_monotone_knots() {
        let kv = KnotVector::<f64>::try_new(vec![0.0, 0.0, 0.5, 1.0, 1.0]).unwrap();
        assert_eq!(kv.len(), 5);
        assert_eq!(kv.as_slice(), &[0.0, 0.0, 0.5, 1.0, 1.0]);
    }

    #[test]
    fn try_new_rejects_non_monotone() {
        let result = KnotVector::<f64>::try_new(vec![0.0, 0.5, 0.3, 1.0]);
        assert!(matches!(result, Err(ConstructError::KnotsNotMonotone)));
    }

    #[test]
    fn try_new_rejects_too_short() {
        let result = KnotVector::<f64>::try_new(vec![0.0]);
        assert!(matches!(result, Err(ConstructError::KnotCountMismatch { .. })));
    }
}
```

Add to `rust/nurbs/src/lib.rs` after `pub mod algebra;`:
```rust
#[cfg(feature = "host")]
pub mod knot;
#[cfg(feature = "host")]
pub use knot::KnotVector;
```

- [ ] **Step 2: Run test to verify it fails (compile or test failure)**

Run: `cargo test -p nurbs --features host knot::tests`
Expected: PASS (since impl is included in this step). If RED, fix the impl.

- [ ] **Step 3: Run all existing tests**

Run: `cargo test -p nurbs --features host`
Expected: All existing tests still pass; new `knot::tests::*` pass.

- [ ] **Step 4: Commit**

```bash
git add rust/nurbs/src/knot.rs rust/nurbs/src/lib.rs
git commit -m "nurbs: add KnotVector<T> type with monotone validation"
```

---

### Task 1.2: Move `find_knot_span` to `knot.rs` as a free function + add `KnotVector::find_span` method

**Files:**
- Modify: `rust/nurbs/src/knot.rs` (add `find_knot_span` free fn + method)
- Modify: `rust/nurbs/src/eval.rs` (re-export from knot for transitional callers)

- [ ] **Step 1: Add `find_knot_span` to knot.rs**

Append to `rust/nurbs/src/knot.rs` (above the `#[cfg(test)]` block):
```rust
/// Find the knot span `k` such that `knots[k] <= u < knots[k+1]`, with the
/// clamped-end special case mapping `u >= knots[n]` to the last span.
/// Reference: Piegl & Tiller "The NURBS Book" Algorithm A2.1.
///
/// Free function form for callers that have raw `&[T]`. See also
/// `KnotVector::find_span` for owned-type callers.
pub fn find_knot_span<T: Float>(knots: &[T], p: usize, n: usize, u: T) -> usize {
    debug_assert!(knots.len() == n + p + 1);
    if u >= knots[n] {
        return n - 1;
    }
    if u <= knots[p] {
        return p;
    }
    let mut low = p;
    let mut high = n;
    let mut mid = (low + high) / 2;
    while u < knots[mid] || u >= knots[mid + 1] {
        if u < knots[mid] {
            high = mid;
        } else {
            low = mid;
        }
        mid = (low + high) / 2;
    }
    mid
}

impl<T: Float> KnotVector<T> {
    /// Find the knot span containing `u` for a curve of given degree `p` with
    /// `n` control points. Delegates to the free function `find_knot_span`.
    pub fn find_span(&self, u: T, p: usize, n: usize) -> usize {
        find_knot_span(&self.knots, p, n, u)
    }
}
```

- [ ] **Step 2: Add a smoke test for `find_knot_span` in knot.rs**

Append inside the existing `#[cfg(test)] mod tests`:
```rust
    #[test]
    fn find_knot_span_returns_correct_span() {
        let knots = [0.0_f64, 0.0, 0.5, 1.0, 1.0];
        // degree 1, n = 3 cps. Span at u=0.25 is 1 (between knots[1]=0.0 and knots[2]=0.5).
        assert_eq!(find_knot_span(&knots, 1, 3, 0.25), 1);
        // u >= knots[n] returns n-1.
        assert_eq!(find_knot_span(&knots, 1, 3, 1.0), 2);
        // u <= knots[p] returns p.
        assert_eq!(find_knot_span(&knots, 1, 3, 0.0), 1);
    }

    #[test]
    fn knot_vector_find_span_delegates() {
        let kv = KnotVector::<f64>::try_new(vec![0.0, 0.0, 0.5, 1.0, 1.0]).unwrap();
        assert_eq!(kv.find_span(0.25, 1, 3), 1);
    }
```

- [ ] **Step 3: Replace `eval.rs`'s local `find_knot_span` with re-export**

In `rust/nurbs/src/eval.rs`, replace lines 6–34 (the `find_knot_span` definition) with:
```rust
// Re-export from knot module for transitional internal use. Eventually
// callers should import directly from `crate::knot::find_knot_span`.
#[cfg(feature = "host")]
pub(crate) use crate::knot::find_knot_span;

// MCU build needs an inline copy since knot module is host-only.
#[cfg(not(feature = "host"))]
#[inline]
pub(crate) fn find_knot_span<T: Float>(knots: &[T], p: usize, n: usize, u: T) -> usize {
    debug_assert!(knots.len() == n + p + 1);
    if u >= knots[n] {
        return n - 1;
    }
    if u <= knots[p] {
        return p;
    }
    let mut low = p;
    let mut high = n;
    let mut mid = (low + high) / 2;
    while u < knots[mid] || u >= knots[mid + 1] {
        if u < knots[mid] {
            high = mid;
        } else {
            low = mid;
        }
        mid = (low + high) / 2;
    }
    mid
}
```

- [ ] **Step 4: Run all tests across both feature sets**

Run: `cargo test -p nurbs --features host`
Expected: PASS (eval tests still green, new knot tests pass).

Run: `cargo test -p nurbs --no-default-features --features mcu-h7`
Expected: PASS (MCU build still works with inline copy).

- [ ] **Step 5: Commit**

```bash
git add rust/nurbs/src/knot.rs rust/nurbs/src/eval.rs
git commit -m "nurbs: move find_knot_span to knot module, keep MCU inline copy"
```

---

### Task 1.3: Refactor `ScalarNurbs` to hold `KnotVector<T>` internally

**Files:**
- Modify: `rust/nurbs/src/scalar.rs`

- [ ] **Step 1: Change the field type and `try_new`**

In `rust/nurbs/src/scalar.rs`, modify the struct (around line 11) and `try_new` (around line 21):

```rust
#[cfg(feature = "host")]
#[derive(Debug, Clone, PartialEq)]
pub struct ScalarNurbs<T: Float> {
    degree: u8,
    knots: crate::knot::KnotVector<T>,  // was: Vec<T>
    control_points: Vec<T>,
    weights: Option<Vec<T>>,
}

#[cfg(feature = "host")]
impl<T: Float> ScalarNurbs<T> {
    pub fn try_new(
        degree: u8,
        knots: Vec<T>,
        control_points: Vec<T>,
        weights: Option<Vec<T>>,
    ) -> Result<Self, ConstructError> {
        validate(degree, &knots, control_points.len(), weights.as_deref())?;
        let knot_vector = crate::knot::KnotVector::try_new(knots)
            .expect("validate already ensured monotone + length");
        Ok(Self {
            degree,
            knots: knot_vector,
            control_points,
            weights,
        })
    }
```

- [ ] **Step 2: Update `knots()` accessor to return `&[T]` via `KnotVector::as_slice()`**

In `rust/nurbs/src/scalar.rs`, modify the inherent and trait `knots()` methods:

```rust
    #[must_use]
    pub fn knots(&self) -> &[T] {
        self.knots.as_slice()
    }
```

And in the `NurbsView` impl:
```rust
    #[inline]
    fn knots(&self) -> &[T] {
        self.knots.as_slice()
    }
```

- [ ] **Step 3: Update `as_view` and `into_parts`**

In `as_view`:
```rust
    pub fn as_view(&self) -> ScalarNurbsRef<'_, T> {
        ScalarNurbsRef {
            degree: self.degree,
            knots: self.knots.as_slice(),
            control_points: &self.control_points,
            weights: self.weights.as_deref(),
        }
    }
```

In `into_parts` — to keep the public signature stable, expose the underlying `Vec<T>`:
```rust
    pub fn into_parts(self) -> (u8, Vec<T>, Vec<T>, Option<Vec<T>>) {
        (self.degree, self.knots.into_inner(), self.control_points, self.weights)
    }
```

- [ ] **Step 4: Add `into_inner` to `KnotVector`**

Append to the `impl<T: Float> KnotVector<T>` block in `rust/nurbs/src/knot.rs`:
```rust
    /// Consume the wrapper, returning the underlying `Vec<T>`.
    pub fn into_inner(self) -> Vec<T> {
        self.knots
    }
```

- [ ] **Step 5: Run tests**

Run: `cargo test -p nurbs --features host`
Expected: All existing scalar tests still pass (signatures unchanged); new tests pass.

- [ ] **Step 6: Commit**

```bash
git add rust/nurbs/src/scalar.rs rust/nurbs/src/knot.rs
git commit -m "nurbs: ScalarNurbs holds KnotVector internally; public accessor unchanged"
```

---

### Task 1.4: Refactor `VectorNurbs` to hold `KnotVector<T>` internally

**Files:**
- Modify: `rust/nurbs/src/vector.rs`

- [ ] **Step 1: Apply the same field-type refactor as Task 1.3**

In `rust/nurbs/src/vector.rs`, change the field:

```rust
pub struct VectorNurbs<T: Float, const N: usize> {
    degree: u8,
    knots: crate::knot::KnotVector<T>,  // was: Vec<T>
    // ... other fields unchanged
}
```

- [ ] **Step 2: Update `try_new` to wrap into `KnotVector`**

In `try_new`, after the `validate(...)` call, wrap:
```rust
        let knot_vector = crate::knot::KnotVector::try_new(knots)
            .expect("validate already ensured monotone + length");
        Ok(Self {
            degree,
            knots: knot_vector,
            // ... rest unchanged
        })
```

- [ ] **Step 3: Update `knots()` accessor and `as_view` and `into_parts`**

Mirror exactly the changes from Task 1.3 — `self.knots.as_slice()` everywhere `&self.knots` was used in slice context.

- [ ] **Step 4: Run all tests**

Run: `cargo test -p nurbs --features host`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/nurbs/src/vector.rs
git commit -m "nurbs: VectorNurbs holds KnotVector internally; public accessor unchanged"
```

---

### Task 1.5: Confirm Phase 1 leaves the public API and behavior unchanged

- [ ] **Step 1: Run the full test suite under all feature combos**

```bash
cd rust
cargo test -p nurbs --features host
cargo test -p nurbs --no-default-features --features mcu-h7
cargo test -p nurbs --no-default-features --features mcu-f4
```
Expected: All pass.

- [ ] **Step 2: Run clippy under host**

```bash
cargo clippy -p nurbs --features host -- -D warnings
```
Expected: Clean.

- [ ] **Step 3: Confirm no public-API changes via `cargo doc`**

```bash
cargo doc -p nurbs --features host --no-deps
```
Expected: Builds without errors. Spot-check `target/doc/nurbs/struct.ScalarNurbs.html` — `knots()` still returns `&[T]`.

---

## Phase 2: `knot.rs` primitives — insertion + removal (~2 days)

### Task 2.1: Add `KnotError` enum and wire to `NurbsError`

**Files:**
- Modify: `rust/nurbs/src/error.rs`
- Modify: `rust/nurbs/src/lib.rs`

- [ ] **Step 1: Write the failing test**

Append to `rust/nurbs/src/error.rs` (inside the `#[cfg(test)] mod tests`):
```rust
    #[test]
    fn knot_error_converts_to_nurbs_error() {
        let e = KnotError::BoundaryInsertion;
        let n: NurbsError<f64> = e.into();
        assert!(matches!(n, NurbsError::Knot(KnotError::BoundaryInsertion)));
    }

    #[test]
    fn knot_error_displays_clearly() {
        let e = KnotError::MultiplicityExceeded { existing: 2, requested: 2, max: 3 };
        let s = format!("{e}");
        assert!(s.contains("multiplicity"));
        assert!(s.contains("2"));
        assert!(s.contains("3"));
    }
```

- [ ] **Step 2: Run to verify it fails (KnotError doesn't exist)**

Run: `cargo test -p nurbs --features host error::tests::knot_error`
Expected: FAIL — `cannot find type 'KnotError'`.

- [ ] **Step 3: Add `KnotError` and integrate**

Append to `rust/nurbs/src/error.rs` after the existing `AlgebraError` definition:
```rust
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum KnotError {
    BoundaryInsertion,
    MultiplicityExceeded { existing: u8, requested: u8, max: u8 },
    OutOfRange,
    Invalid,
}

impl fmt::Display for KnotError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::BoundaryInsertion => {
                write!(f, "cannot insert knot at clamped boundary")
            }
            Self::MultiplicityExceeded { existing, requested, max } => {
                write!(f, "knot multiplicity {existing} + {requested} exceeds max {max}")
            }
            Self::OutOfRange => write!(f, "knot value out of knot vector range"),
            Self::Invalid => write!(f, "knot vector violates monotone or length invariants"),
        }
    }
}

impl core::error::Error for KnotError {}
```

Extend `NurbsError`:
```rust
#[derive(Debug, Clone, PartialEq)]
pub enum NurbsError<T: Float> {
    Construct(ConstructError),
    Wire(WireError),
    ArcLength(ArcLengthError<T>),
    Algebra(AlgebraError),
    Knot(KnotError),  // NEW
}
```

Extend the `Display` impl for `NurbsError` to include `Knot(e) => write!(f, "{e}")`, and add a `From` impl:
```rust
impl<T: Float> From<KnotError> for NurbsError<T> {
    fn from(e: KnotError) -> Self {
        Self::Knot(e)
    }
}
```

- [ ] **Step 4: Re-export from lib.rs**

In `rust/nurbs/src/lib.rs`, change:
```rust
pub use error::{AlgebraError, ArcLengthError, ConstructError, NurbsError, WireError};
```
to include `KnotError`:
```rust
pub use error::{AlgebraError, ArcLengthError, ConstructError, KnotError, NurbsError, WireError};
```

- [ ] **Step 5: Run tests to confirm pass**

Run: `cargo test -p nurbs --features host`
Expected: All pass including new error tests.

- [ ] **Step 6: Commit**

```bash
git add rust/nurbs/src/error.rs rust/nurbs/src/lib.rs
git commit -m "nurbs: add KnotError type and wire it to NurbsError"
```

---

### Task 2.2: `KnotVector::multiplicity_at`

**Files:**
- Modify: `rust/nurbs/src/knot.rs`

- [ ] **Step 1: Write the failing test**

Add to the `#[cfg(test)] mod tests` block in `knot.rs`:
```rust
    #[test]
    fn multiplicity_at_counts_repeated_knots() {
        let kv = KnotVector::<f64>::try_new(vec![0.0, 0.0, 0.5, 0.5, 1.0, 1.0]).unwrap();
        assert_eq!(kv.multiplicity_at(0.0), 2);
        assert_eq!(kv.multiplicity_at(0.5), 2);
        assert_eq!(kv.multiplicity_at(1.0), 2);
        assert_eq!(kv.multiplicity_at(0.25), 0);
    }
```

- [ ] **Step 2: Run, expect compile failure**

Run: `cargo test -p nurbs --features host knot::tests::multiplicity_at`
Expected: FAIL — `no method multiplicity_at`.

- [ ] **Step 3: Implement**

Append to the `impl<T: Float> KnotVector<T>` block in `knot.rs`:
```rust
    /// Count consecutive equal knots at value `u`. Returns 0 if `u` is not present.
    pub fn multiplicity_at(&self, u: T) -> usize {
        self.knots.iter().filter(|k| **k == u).count()
    }
```

- [ ] **Step 4: Run, expect pass**

Run: `cargo test -p nurbs --features host knot::tests::multiplicity_at`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/nurbs/src/knot.rs
git commit -m "nurbs/knot: add multiplicity_at"
```

---

### Task 2.3: Boehm `insert_knot` — happy path (no existing multiplicity)

**Files:**
- Modify: `rust/nurbs/src/knot.rs`

- [ ] **Step 1: Write the failing test**

Add to the test module in `knot.rs`:
```rust
    use crate::ScalarNurbs;
    use crate::eval::eval;

    #[test]
    fn insert_knot_into_simple_curve_preserves_evaluation() {
        // Linear curve from 0 to 2 over [0, 1]. Insert knot at u=0.5.
        let curve = ScalarNurbs::<f64>::try_new(
            1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 2.0], None,
        ).unwrap();

        let inserted = insert_knot(&curve, 0.5, 1).unwrap();

        assert_eq!(inserted.knots(), &[0.0, 0.0, 0.5, 1.0, 1.0]);
        assert_eq!(inserted.control_points().len(), 3);  // was 2, now 3
        // Geometric invariance: eval at sample points unchanged.
        for u in [0.0, 0.1, 0.25, 0.5, 0.75, 1.0] {
            let before = eval(&curve.as_view(), u);
            let after = eval(&inserted.as_view(), u);
            assert!((before - after).abs() < 1e-12, "u={u}: before={before}, after={after}");
        }
    }
```

- [ ] **Step 2: Run, expect compile failure**

Run: `cargo test -p nurbs --features host knot::tests::insert_knot_into_simple`
Expected: FAIL — `cannot find function 'insert_knot'`.

- [ ] **Step 3: Implement Boehm single-insertion**

Append to `knot.rs` (above the test module):
```rust
use crate::{KnotError, ScalarNurbs};

/// Insert ū into a curve with the given multiplicity (number of repeated insertions).
///
/// Boehm's algorithm (Piegl & Tiller §5.2, Algorithm A5.1 / A5.3). The inserted
/// knot does not change the curve geometrically — eval is invariant. The
/// number of control points grows by `multiplicity`.
///
/// Errors:
/// - `BoundaryInsertion` if ū equals a clamped endpoint.
/// - `MultiplicityExceeded` if `existing + multiplicity > degree`.
/// - `OutOfRange` if ū is outside the knot vector range.
pub fn insert_knot<T: Float>(
    curve: &ScalarNurbs<T>,
    u: T,
    multiplicity: usize,
) -> Result<ScalarNurbs<T>, KnotError> {
    let p = curve.degree() as usize;
    let knots = curve.knots();
    let cps = curve.control_points();
    let weights = curve.weights();

    // Validate u is in (knots[0], knots[last]) — strictly interior.
    if u <= knots[0] || u >= knots[knots.len() - 1] {
        return Err(KnotError::BoundaryInsertion);
    }
    if u < knots[0] || u > knots[knots.len() - 1] {
        return Err(KnotError::OutOfRange);
    }

    // Existing multiplicity at u.
    let existing = curve.knots().iter().filter(|k| **k == u).count();
    if existing + multiplicity > p {
        return Err(KnotError::MultiplicityExceeded {
            existing: existing as u8,
            requested: multiplicity as u8,
            max: p as u8,
        });
    }

    let n = cps.len();
    let k = find_knot_span(knots, p, n, u);

    // Build new knot vector: insert `multiplicity` copies of u at position k+1.
    let mut new_knots = Vec::with_capacity(knots.len() + multiplicity);
    new_knots.extend_from_slice(&knots[..=k]);
    for _ in 0..multiplicity {
        new_knots.push(u);
    }
    new_knots.extend_from_slice(&knots[k + 1..]);

    // Apply A5.3 fused multi-insertion to control points.
    // For non-rational case: lift weights to homogeneous (cps * w, w), insert, project back.
    // For non-rational only (rational rejected by callers in v1 algebra), simpler form:
    let new_cps = if let Some(w) = weights {
        // Homogeneous lift: (cp * w, w), insert, project.
        let homo: Vec<(T, T)> = cps.iter().zip(w.iter()).map(|(c, w)| (*c * *w, *w)).collect();
        let new_homo = boehm_insert_homogeneous(&homo, knots, p, k, u, existing, multiplicity);
        // Project back: cp = homo.0 / homo.1.
        new_homo.into_iter().map(|(num, w)| num / w).collect::<Vec<T>>()
    } else {
        boehm_insert_unweighted(cps, knots, p, k, u, existing, multiplicity)
    };

    let new_weights = if let Some(w) = weights {
        // Recompute new weights: also Boehm-blended.
        let dummy_cps: Vec<T> = vec![T::ZERO; cps.len()];  // placeholder — see below
        // Simpler: re-lift, the projection above gave us new_cps; we still need new_weights.
        // Run Boehm on weights alone.
        Some(boehm_insert_unweighted(w, knots, p, k, u, existing, multiplicity))
    } else {
        None
    };

    ScalarNurbs::try_new(curve.degree(), new_knots, new_cps, new_weights)
        .map_err(|_| KnotError::Invalid)
}

/// Single-insertion fused as r-fold (P&T A5.3) for unweighted control points.
fn boehm_insert_unweighted<T: Float>(
    cps: &[T],
    knots: &[T],
    p: usize,
    k: usize,
    u: T,
    existing: usize,
    r: usize,  // number of insertions
) -> Vec<T> {
    let n = cps.len();
    let new_n = n + r;
    let mut new_cps = vec![T::ZERO; new_n];

    // Unaffected CPs pass through.
    for i in 0..=k - p {
        new_cps[i] = cps[i];
    }
    for i in (k - existing)..n {
        new_cps[i + r] = cps[i];
    }

    // Working buffer for the r-fold blend.
    let mut work: Vec<T> = (0..=p - existing).map(|i| cps[k - p + i]).collect();

    // r-fold insertion (A5.3).
    for j in 1..=r {
        let l = k - p + j;
        for i in 0..=p - j - existing {
            let denom = knots[l + i + p] - knots[l + i];
            let alpha = if denom > T::ZERO {
                (u - knots[l + i]) / denom
            } else {
                T::ZERO
            };
            work[i] = (T::ONE - alpha) * work[i] + alpha * work[i + 1];
        }
        new_cps[l] = work[0];
        new_cps[k + r - j - existing] = work[p - j - existing];
    }

    // Remaining middle CPs.
    for i in (k - p + r)..(k - existing) {
        new_cps[i] = work[i - (k - p + r)];
    }

    new_cps
}

/// Homogeneous variant: blends (num, w) tuples.
fn boehm_insert_homogeneous<T: Float>(
    homo: &[(T, T)],
    knots: &[T],
    p: usize,
    k: usize,
    u: T,
    existing: usize,
    r: usize,
) -> Vec<(T, T)> {
    // Apply Boehm component-wise.
    let nums: Vec<T> = homo.iter().map(|(n, _)| *n).collect();
    let ws: Vec<T> = homo.iter().map(|(_, w)| *w).collect();
    let new_nums = boehm_insert_unweighted(&nums, knots, p, k, u, existing, r);
    let new_ws = boehm_insert_unweighted(&ws, knots, p, k, u, existing, r);
    new_nums.into_iter().zip(new_ws).collect()
}
```

- [ ] **Step 4: Run, expect pass**

Run: `cargo test -p nurbs --features host knot::tests::insert_knot_into_simple`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/nurbs/src/knot.rs
git commit -m "nurbs/knot: insert_knot via Boehm A5.3 (happy path)"
```

---

### Task 2.4: `insert_knot` boundary rejection

**Files:**
- Modify: `rust/nurbs/src/knot.rs`

- [ ] **Step 1: Write the failing test**

Add to the test module:
```rust
    #[test]
    fn insert_knot_rejects_clamped_boundary() {
        let curve = ScalarNurbs::<f64>::try_new(
            1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0], None,
        ).unwrap();

        assert!(matches!(insert_knot(&curve, 0.0, 1), Err(KnotError::BoundaryInsertion)));
        assert!(matches!(insert_knot(&curve, 1.0, 1), Err(KnotError::BoundaryInsertion)));
    }
```

- [ ] **Step 2: Run, expect pass (already implemented)**

Run: `cargo test -p nurbs --features host knot::tests::insert_knot_rejects_clamped`
Expected: PASS — boundary check already in `insert_knot`.

- [ ] **Step 3: Commit**

```bash
git add rust/nurbs/src/knot.rs
git commit -m "nurbs/knot: test boundary rejection for insert_knot"
```

---

### Task 2.5: `insert_knot` multiplicity-exceeded rejection

**Files:**
- Modify: `rust/nurbs/src/knot.rs`

- [ ] **Step 1: Write the failing test**

```rust
    #[test]
    fn insert_knot_rejects_multiplicity_exceeded() {
        // Quadratic curve with interior knot at 0.5 (multiplicity 1, so we can add 1 more).
        let curve = ScalarNurbs::<f64>::try_new(
            2, vec![0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0], vec![0.0, 1.0, 2.0, 3.0], None,
        ).unwrap();

        // Insert 2 more at u=0.5: existing=1 + 2 = 3 > degree 2.
        let result = insert_knot(&curve, 0.5, 2);
        assert!(matches!(
            result,
            Err(KnotError::MultiplicityExceeded { existing: 1, requested: 2, max: 2 })
        ));
    }
```

- [ ] **Step 2: Run, expect pass (already implemented)**

Run: `cargo test -p nurbs --features host knot::tests::insert_knot_rejects_multiplicity`
Expected: PASS.

- [ ] **Step 3: Commit**

```bash
git add rust/nurbs/src/knot.rs
git commit -m "nurbs/knot: test multiplicity-exceeded rejection for insert_knot"
```

---

### Task 2.6: `insert_knot` with existing multiplicity > 0

**Files:**
- Modify: `rust/nurbs/src/knot.rs`

- [ ] **Step 1: Write the failing test**

```rust
    #[test]
    fn insert_knot_at_existing_multiplicity_preserves_evaluation() {
        // Quadratic curve with interior knot at 0.5 (multiplicity 1).
        let curve = ScalarNurbs::<f64>::try_new(
            2, vec![0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0], vec![0.0, 1.0, 2.0, 3.0], None,
        ).unwrap();

        // Insert one more at u=0.5: existing=1 + 1 = 2 == degree, allowed.
        let inserted = insert_knot(&curve, 0.5, 1).unwrap();
        assert_eq!(inserted.knots(), &[0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0]);

        for u in [0.0, 0.25, 0.5, 0.75, 1.0] {
            let before = eval(&curve.as_view(), u);
            let after = eval(&inserted.as_view(), u);
            assert!((before - after).abs() < 1e-12, "u={u}: before={before}, after={after}");
        }
    }
```

- [ ] **Step 2: Run, debug if needed**

Run: `cargo test -p nurbs --features host knot::tests::insert_knot_at_existing`
Expected: PASS. If FAIL, the `existing` clamp in `boehm_insert_unweighted` is incorrect — review the loop bounds in A5.3.

- [ ] **Step 3: Commit**

```bash
git add rust/nurbs/src/knot.rs
git commit -m "nurbs/knot: verify insert_knot handles existing multiplicity correctly"
```

---

### Task 2.7: `KnotVector::refined_to_full_multiplicity`

**Files:**
- Modify: `rust/nurbs/src/knot.rs`

- [ ] **Step 1: Write the failing test**

```rust
    #[test]
    fn refined_to_full_multiplicity_raises_interior_knots() {
        // Cubic with one interior knot at 0.5 (multiplicity 1).
        let curve = ScalarNurbs::<f64>::try_new(
            3, vec![0.0, 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0, 1.0],
            vec![0.0, 1.0, 2.0, 3.0, 4.0], None,
        ).unwrap();

        let refined = refined_to_full_multiplicity(&curve);

        // Interior knot 0.5 should now have multiplicity = degree = 3.
        assert_eq!(refined.knots(), &[0.0, 0.0, 0.0, 0.0, 0.5, 0.5, 0.5, 1.0, 1.0, 1.0, 1.0]);
        // Geometric invariance.
        for u in [0.0, 0.25, 0.5, 0.75, 1.0] {
            let before = eval(&curve.as_view(), u);
            let after = eval(&refined.as_view(), u);
            assert!((before - after).abs() < 1e-10, "u={u}: before={before}, after={after}");
        }
    }
```

- [ ] **Step 2: Run, expect compile failure**

Run: `cargo test -p nurbs --features host knot::tests::refined_to_full`
Expected: FAIL — `cannot find function`.

- [ ] **Step 3: Implement**

Append to `knot.rs` (above the test module):
```rust
/// Raise every interior knot's multiplicity to `degree`, producing a curve
/// whose representation decomposes cleanly into Bézier pieces. Geometric
/// invariance preserved.
pub fn refined_to_full_multiplicity<T: Float>(curve: &ScalarNurbs<T>) -> ScalarNurbs<T> {
    let p = curve.degree() as usize;
    let mut current = curve.clone();

    // Collect unique interior knot values.
    let knots_snapshot: Vec<T> = current.knots().to_vec();
    let mut interior: Vec<T> = Vec::new();
    let mut i = p + 1;
    while i < knots_snapshot.len() - p - 1 {
        let u = knots_snapshot[i];
        if !interior.contains(&u) {
            interior.push(u);
        }
        i += 1;
    }

    for u in interior {
        let existing = current.knots().iter().filter(|k| **k == u).count();
        if existing < p {
            current = insert_knot(&current, u, p - existing)
                .expect("refined_to_full_multiplicity: insertion should be valid");
        }
    }

    current
}
```

- [ ] **Step 4: Run, expect pass**

Run: `cargo test -p nurbs --features host knot::tests::refined_to_full`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/nurbs/src/knot.rs
git commit -m "nurbs/knot: refined_to_full_multiplicity — Bezier decomposition prep"
```

---

### Task 2.8: Tiller `remove_knot` — happy path (insert-then-remove round-trip)

**Files:**
- Modify: `rust/nurbs/src/knot.rs`

- [ ] **Step 1: Write the failing test**

```rust
    #[test]
    fn remove_knot_undoes_insertion_within_tolerance() {
        let curve = ScalarNurbs::<f64>::try_new(
            2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], vec![0.0, 1.0, 2.0], None,
        ).unwrap();

        let inserted = insert_knot(&curve, 0.5, 1).unwrap();
        let (removed, count) = remove_knot(&inserted, 0.5, 1, 1e-10);

        assert_eq!(count, 1);
        assert_eq!(removed.knots(), curve.knots());
        for (a, b) in removed.control_points().iter().zip(curve.control_points()) {
            assert!((a - b).abs() < 1e-10, "cp mismatch: {a} vs {b}");
        }
    }
```

- [ ] **Step 2: Run, expect compile failure**

Run: `cargo test -p nurbs --features host knot::tests::remove_knot_undoes`
Expected: FAIL.

- [ ] **Step 3: Implement Tiller knot removal (P&T §5.4 / A5.8)**

Append to `knot.rs`:
```rust
/// Tiller knot removal (P&T §5.4, Algorithm A5.8). Removes knot ū up to
/// `count` times if removal preserves the curve within chord-error `tol` in
/// control-point space. Returns the new curve and the number of removals
/// actually performed (may be less than `count`).
///
/// For unweighted curves only in v1; weighted (rational) curves return the
/// input unchanged with count 0 (no error — caller can detect via the count).
pub fn remove_knot<T: Float>(
    curve: &ScalarNurbs<T>,
    u: T,
    count: usize,
    tol: T,
) -> (ScalarNurbs<T>, usize) {
    if curve.weights().is_some() {
        // v1: rational removal not supported; return input unchanged.
        return (curve.clone(), 0);
    }
    let p = curve.degree() as usize;
    let knots = curve.knots();
    let cps = curve.control_points();
    let n = cps.len();

    // Find span and existing multiplicity.
    let s = knots.iter().filter(|k| **k == u).count();
    if s == 0 {
        return (curve.clone(), 0);  // u not in knot vector
    }
    let r = find_knot_span(knots, p, n, u);

    let mut new_cps = cps.to_vec();
    let mut new_knots = knots.to_vec();
    let mut removed = 0;
    let mut current_s = s;

    while removed < count && current_s > 0 {
        // Try one removal (A5.8).
        let first = r - p;
        let last = r - current_s;
        let mut temp = vec![T::ZERO; (last - first + 2).max(2)];

        temp[0] = new_cps[first - 1];
        temp[last - first + 1] = new_cps[last + 1];

        let mut i = first;
        let mut j = last;
        let mut ii = 1;
        let mut jj = last - first;
        let mut converged = true;

        while j - i > 0 {
            let alpha_i = (u - new_knots[i]) / (new_knots[i + p + 1] - new_knots[i]);
            let alpha_j = (u - new_knots[j]) / (new_knots[j + p + 1] - new_knots[j]);

            temp[ii] = (new_cps[i] - (T::ONE - alpha_i) * temp[ii - 1]) / alpha_i;
            temp[jj] = (new_cps[j] - alpha_j * temp[jj + 1]) / (T::ONE - alpha_j);

            i += 1; ii += 1; j -= 1; jj -= 1;
        }

        // Convergence check: chord-error tolerance.
        if j - i < 1 {
            let err = (temp[ii - 1] - temp[jj + 1]).abs();
            if err > tol {
                converged = false;
            }
        }

        if !converged {
            break;
        }

        // Apply: shift CPs down, drop one knot.
        let mut i2 = first;
        let mut j2 = last;
        while j2 - i2 > 0 {
            new_cps[i2] = temp[i2 - first + 1];
            new_cps[j2] = temp[j2 - first + 1];
            i2 += 1; j2 -= 1;
        }
        // Remove one cp (the duplicate at center) and one knot.
        new_cps.remove((first + last) / 2 + 1);
        new_knots.remove(r);

        removed += 1;
        current_s -= 1;
    }

    let new_curve = ScalarNurbs::try_new(curve.degree(), new_knots, new_cps, None)
        .expect("remove_knot: result invariants should hold");
    (new_curve, removed)
}
```

- [ ] **Step 4: Run test**

Run: `cargo test -p nurbs --features host knot::tests::remove_knot_undoes`
Expected: PASS. If FAIL, debug — A5.8 is fiddly; cross-check P&T pp. 184–186.

- [ ] **Step 5: Commit**

```bash
git add rust/nurbs/src/knot.rs
git commit -m "nurbs/knot: remove_knot via Tiller A5.8"
```

---

### Task 2.9: `remove_knot` rejects when tolerance not met

**Files:**
- Modify: `rust/nurbs/src/knot.rs`

- [ ] **Step 1: Write the failing test**

```rust
    #[test]
    fn remove_knot_returns_zero_when_tolerance_not_met() {
        // A real C^0 corner at u=0.5: knot at multiplicity 2 (== degree), and
        // CPs chosen so removal would visibly displace.
        let curve = ScalarNurbs::<f64>::try_new(
            2, vec![0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0],
            vec![0.0, 1.0, 5.0, 0.0, 1.0],  // sharp jump at the corner
            None,
        ).unwrap();

        let (result, removed) = remove_knot(&curve, 0.5, 1, 1e-9);
        assert_eq!(removed, 0);
        // Curve unchanged.
        assert_eq!(result.knots(), curve.knots());
    }
```

- [ ] **Step 2: Run**

Run: `cargo test -p nurbs --features host knot::tests::remove_knot_returns_zero`
Expected: PASS.

- [ ] **Step 3: Commit**

```bash
git add rust/nurbs/src/knot.rs
git commit -m "nurbs/knot: test remove_knot tolerance rejection"
```

---

## Phase 3: `bezier.rs` — Bézier piece type and extraction (~2 days)

### Task 3.1: `BezierPiece` struct + `evaluate` (Horner)

**Files:**
- Create: `rust/nurbs/src/bezier.rs`
- Modify: `rust/nurbs/src/lib.rs`

- [ ] **Step 1: Create file with struct + test**

Create `rust/nurbs/src/bezier.rs`:
```rust
//! Bézier piece in Pascal-shifted monomial basis. Host-only.
//! See `docs/superpowers/specs/2026-04-26-nurbs-algebra-design.md` §5.

#![cfg(feature = "host")]

use crate::{AlgebraError, Float};

/// One Bézier piece as a polynomial in the *Pascal-shifted monomial basis*:
/// p(u) = Σ_{k=0..d} coeffs[k] * (u - u_start)^k
#[derive(Debug, Clone, PartialEq)]
pub struct BezierPiece<T: Float> {
    pub u_start: T,
    pub u_end: T,
    pub coeffs: Vec<T>,  // length = degree + 1
}

impl<T: Float> BezierPiece<T> {
    /// Polynomial degree (= coeffs.len() - 1).
    pub fn degree(&self) -> usize {
        self.coeffs.len().saturating_sub(1)
    }

    /// Evaluate p(u) by Horner's method on the Pascal-shifted basis.
    pub fn evaluate(&self, u: T) -> T {
        let dx = u - self.u_start;
        let mut acc = T::ZERO;
        for c in self.coeffs.iter().rev() {
            acc = acc * dx + *c;
        }
        acc
    }

    /// Zero polynomial of the given degree on [u_start, u_end].
    /// Used as the accumulator inside `convolve`.
    pub fn zero(u_start: T, u_end: T, degree: usize) -> Self {
        Self {
            u_start,
            u_end,
            coeffs: vec![T::ZERO; degree + 1],
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn evaluate_constant_polynomial_is_constant() {
        let p = BezierPiece::<f64> { u_start: 0.0, u_end: 1.0, coeffs: vec![3.5] };
        assert_eq!(p.evaluate(0.0), 3.5);
        assert_eq!(p.evaluate(0.5), 3.5);
        assert_eq!(p.evaluate(1.0), 3.5);
    }

    #[test]
    fn evaluate_linear_polynomial() {
        // p(u) = 1 + 2 * (u - 0)
        let p = BezierPiece::<f64> { u_start: 0.0, u_end: 1.0, coeffs: vec![1.0, 2.0] };
        assert_eq!(p.evaluate(0.0), 1.0);
        assert_eq!(p.evaluate(0.5), 2.0);
        assert_eq!(p.evaluate(1.0), 3.0);
    }

    #[test]
    fn evaluate_uses_shifted_basis() {
        // p(u) = 1 + 2 * (u - 5), so p(5) = 1, p(6) = 3.
        let p = BezierPiece::<f64> { u_start: 5.0, u_end: 7.0, coeffs: vec![1.0, 2.0] };
        assert_eq!(p.evaluate(5.0), 1.0);
        assert_eq!(p.evaluate(6.0), 3.0);
        assert_eq!(p.evaluate(7.0), 5.0);
    }

    #[test]
    fn zero_creates_zero_polynomial_of_given_degree() {
        let p = BezierPiece::<f64>::zero(0.0, 1.0, 3);
        assert_eq!(p.coeffs, vec![0.0, 0.0, 0.0, 0.0]);
        assert_eq!(p.degree(), 3);
        assert_eq!(p.evaluate(0.5), 0.0);
    }
}
```

Add to `rust/nurbs/src/lib.rs` after the `knot` module:
```rust
#[cfg(feature = "host")]
pub mod bezier;
#[cfg(feature = "host")]
pub use bezier::BezierPiece;
```

- [ ] **Step 2: Run tests**

Run: `cargo test -p nurbs --features host bezier::tests`
Expected: PASS.

- [ ] **Step 3: Commit**

```bash
git add rust/nurbs/src/bezier.rs rust/nurbs/src/lib.rs
git commit -m "nurbs: add BezierPiece type with Horner evaluate"
```

---

### Task 3.2: `BezierPiece::to_bernstein` and `from_bernstein`

**Files:**
- Modify: `rust/nurbs/src/bezier.rs`

- [ ] **Step 1: Write the failing test**

Add to the test module:
```rust
    #[test]
    fn bernstein_round_trip_preserves_polynomial() {
        // Quadratic in monomial form: p(u) = 1 + 2u + 3u^2 on [0, 1].
        let monom = BezierPiece::<f64> {
            u_start: 0.0, u_end: 1.0, coeffs: vec![1.0, 2.0, 3.0],
        };
        let bernstein = monom.to_bernstein();
        let back = BezierPiece::from_bernstein(&bernstein, 0.0, 1.0);

        for u in [0.0, 0.25, 0.5, 0.75, 1.0] {
            let exp = monom.evaluate(u);
            let got = back.evaluate(u);
            assert!((exp - got).abs() < 1e-12, "u={u}: exp={exp}, got={got}");
        }
    }

    #[test]
    fn from_bernstein_to_monomial_for_known_case() {
        // Bernstein control points for line from 0 to 1 on [0, 1]: B_0=0, B_1=1.
        let p = BezierPiece::from_bernstein(&[0.0_f64, 1.0], 0.0, 1.0);
        // Equivalent monomial: p(u) = u, so coeffs = [0, 1].
        assert!((p.coeffs[0] - 0.0).abs() < 1e-12);
        assert!((p.coeffs[1] - 1.0).abs() < 1e-12);
    }
```

- [ ] **Step 2: Run, expect compile failure**

Run: `cargo test -p nurbs --features host bezier::tests::bernstein_round`
Expected: FAIL.

- [ ] **Step 3: Implement**

Append to the `impl<T: Float> BezierPiece<T>` block:
```rust
    /// Convert this monomial-basis polynomial to Bernstein control points on
    /// [u_start, u_end]. Length = degree + 1.
    /// Formula: B_k = Σ_{i=0..k} C(k,i) / C(d,i) * c_i_norm, where c_i_norm = c_i * h^i, h = u_end - u_start.
    /// (Per Farin §5.7.)
    pub fn to_bernstein(&self) -> Vec<T> {
        let d = self.degree();
        let h = self.u_end - self.u_start;
        // Normalize monomial coefficients to the [0, 1] domain.
        let mut h_pow = T::ONE;
        let normalized: Vec<T> = self.coeffs.iter().map(|c| {
            let v = *c * h_pow;
            h_pow = h_pow * h;
            v
        }).collect();

        // Convert normalized monomial to Bernstein.
        let mut bernstein = vec![T::ZERO; d + 1];
        for k in 0..=d {
            let mut acc = T::ZERO;
            for i in 0..=k {
                let num = T::from_f64(binomial(k, i) as f64);
                let den = T::from_f64(binomial(d, i) as f64);
                acc = acc + (num / den) * normalized[i];
            }
            bernstein[k] = acc;
        }
        bernstein
    }

    /// Build a Bézier piece from Bernstein control points on [u_start, u_end].
    /// Inverse of `to_bernstein`. Length of `bernstein` = degree + 1.
    /// Formula: c_k = C(d,k) * Σ_{i=0..k} (-1)^{k-i} * C(k,i) * B_i / h^k.
    pub fn from_bernstein(bernstein: &[T], u_start: T, u_end: T) -> Self {
        let d = bernstein.len() - 1;
        let h = u_end - u_start;

        let mut h_pow = T::ONE;
        let mut coeffs = vec![T::ZERO; d + 1];
        for k in 0..=d {
            let mut acc = T::ZERO;
            for i in 0..=k {
                let sign = if (k - i) % 2 == 0 { T::ONE } else { -T::ONE };
                let c_d_k = T::from_f64(binomial(d, k) as f64);
                let c_k_i = T::from_f64(binomial(k, i) as f64);
                acc = acc + sign * c_d_k * c_k_i * bernstein[i];
            }
            coeffs[k] = acc / h_pow;
            h_pow = h_pow * h;
        }
        Self { u_start, u_end, coeffs }
    }
}

/// Binomial coefficient C(n, k). Integer-valued; safe for k, n ≤ 30 or so.
/// `pub(crate)` so `algebra.rs` can reuse it (DRY — defined here, used in convolve too).
pub(crate) fn binomial(n: usize, k: usize) -> u64 {
    if k > n { return 0; }
    let k = k.min(n - k);
    let mut result: u64 = 1;
    for i in 0..k {
        result = result * (n - i) as u64 / (i + 1) as u64;
    }
    result
}
```

- [ ] **Step 4: Run, expect pass**

Run: `cargo test -p nurbs --features host bezier::tests::bernstein_round_trip`
Expected: PASS. Iterate on formula sign or normalization if FAIL.

- [ ] **Step 5: Commit**

```bash
git add rust/nurbs/src/bezier.rs
git commit -m "nurbs/bezier: to_bernstein / from_bernstein basis conversions"
```

---

### Task 3.3: `BezierPiece` `Add` impl (same-support case)

**Files:**
- Modify: `rust/nurbs/src/bezier.rs`
- Modify: `rust/nurbs/src/error.rs` (add `SupportMismatch` variant to `AlgebraError`)

- [ ] **Step 1: Add `SupportMismatch` to `AlgebraError`**

In `rust/nurbs/src/error.rs`, modify the `AlgebraError` enum:
```rust
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AlgebraError {
    DegreeExceeded { result_degree: u8, max: u8 },
    KnotMismatch,
    NotImplemented(&'static str),
    SupportMismatch,  // NEW
}
```

And the Display impl:
```rust
            Self::SupportMismatch => write!(f, "Bezier pieces have mismatched support"),
```

- [ ] **Step 2: Write the failing test**

Add to `rust/nurbs/src/bezier.rs` test module:
```rust
    #[test]
    fn add_two_pieces_same_support() {
        let a = BezierPiece::<f64> { u_start: 0.0, u_end: 1.0, coeffs: vec![1.0, 2.0] };
        let b = BezierPiece::<f64> { u_start: 0.0, u_end: 1.0, coeffs: vec![3.0, 4.0] };
        let sum = (&a + &b).unwrap();
        assert_eq!(sum.coeffs, vec![4.0, 6.0]);
        assert_eq!(sum.u_start, 0.0);
        assert_eq!(sum.u_end, 1.0);
    }

    #[test]
    fn add_two_pieces_mismatched_degrees_pads_with_zero() {
        let a = BezierPiece::<f64> { u_start: 0.0, u_end: 1.0, coeffs: vec![1.0, 2.0, 3.0] };
        let b = BezierPiece::<f64> { u_start: 0.0, u_end: 1.0, coeffs: vec![1.0] };
        let sum = (&a + &b).unwrap();
        assert_eq!(sum.coeffs, vec![2.0, 2.0, 3.0]);
    }

    #[test]
    fn add_two_pieces_mismatched_support_errors() {
        let a = BezierPiece::<f64> { u_start: 0.0, u_end: 1.0, coeffs: vec![1.0] };
        let b = BezierPiece::<f64> { u_start: 0.5, u_end: 1.0, coeffs: vec![1.0] };
        assert!(matches!(&a + &b, Err(AlgebraError::SupportMismatch)));
    }
```

- [ ] **Step 3: Run, expect compile failure**

Run: `cargo test -p nurbs --features host bezier::tests::add_two_pieces`
Expected: FAIL — `Add` not implemented.

- [ ] **Step 4: Implement `Add`**

Append to `rust/nurbs/src/bezier.rs`:
```rust
impl<T: Float> std::ops::Add<&BezierPiece<T>> for &BezierPiece<T> {
    type Output = Result<BezierPiece<T>, AlgebraError>;
    fn add(self, rhs: &BezierPiece<T>) -> Self::Output {
        if self.u_start != rhs.u_start || self.u_end != rhs.u_end {
            return Err(AlgebraError::SupportMismatch);
        }
        let max_len = self.coeffs.len().max(rhs.coeffs.len());
        let mut coeffs = vec![T::ZERO; max_len];
        for (i, c) in self.coeffs.iter().enumerate() { coeffs[i] = coeffs[i] + *c; }
        for (i, c) in rhs.coeffs.iter().enumerate() { coeffs[i] = coeffs[i] + *c; }
        Ok(BezierPiece { u_start: self.u_start, u_end: self.u_end, coeffs })
    }
}
```

- [ ] **Step 5: Run, expect pass**

Run: `cargo test -p nurbs --features host bezier::tests`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add rust/nurbs/src/bezier.rs rust/nurbs/src/error.rs
git commit -m "nurbs/bezier: same-support Add for BezierPiece + SupportMismatch error"
```

---

### Task 3.4: `extract_bezier_pieces` for an already-Bézier input

**Files:**
- Modify: `rust/nurbs/src/bezier.rs`

- [ ] **Step 1: Write the failing test**

Add to test module:
```rust
    use crate::ScalarNurbs;

    #[test]
    fn extract_single_bezier_piece_from_clamped_curve() {
        // Quadratic with no interior knots — already a single Bezier piece.
        let curve = ScalarNurbs::<f64>::try_new(
            2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], vec![0.0, 1.0, 4.0], None,
        ).unwrap();

        let pieces = extract_bezier_pieces(&curve);
        assert_eq!(pieces.len(), 1);
        let p = &pieces[0];
        assert_eq!(p.u_start, 0.0);
        assert_eq!(p.u_end, 1.0);
        assert_eq!(p.degree(), 2);
        // Eval at sample points matches.
        for u in [0.0, 0.25, 0.5, 0.75, 1.0] {
            let exp = crate::eval::eval(&curve.as_view(), u);
            let got = p.evaluate(u);
            assert!((exp - got).abs() < 1e-12, "u={u}: exp={exp}, got={got}");
        }
    }
```

- [ ] **Step 2: Run, expect compile failure**

Run: `cargo test -p nurbs --features host bezier::tests::extract_single`
Expected: FAIL.

- [ ] **Step 3: Implement**

Append to `bezier.rs`:
```rust
use crate::ScalarNurbs;

/// Decompose a polynomial NURBS into its constituent Bézier pieces in the
/// Pascal-shifted monomial basis. Internally raises every interior knot to
/// multiplicity = degree (Boehm), then converts each Bernstein piece to monomial.
pub fn extract_bezier_pieces<T: Float>(curve: &ScalarNurbs<T>) -> Vec<BezierPiece<T>> {
    assert!(curve.weights().is_none(), "extract_bezier_pieces: rational input not supported in v1");

    let refined = crate::knot::refined_to_full_multiplicity(curve);
    let p = refined.degree() as usize;
    let knots = refined.knots();
    let cps = refined.control_points();

    // Identify unique breakpoints (excluding clamping at endpoints, only counted once).
    let mut breakpoints: Vec<T> = Vec::new();
    let mut last: Option<T> = None;
    for k in knots {
        if last.map_or(true, |l| *k != l) {
            breakpoints.push(*k);
            last = Some(*k);
        }
    }

    let mut pieces = Vec::with_capacity(breakpoints.len() - 1);
    let mut cp_idx = 0;
    for window in breakpoints.windows(2) {
        let u_start = window[0];
        let u_end = window[1];
        let bernstein: Vec<T> = cps[cp_idx..cp_idx + p + 1].to_vec();
        pieces.push(BezierPiece::from_bernstein(&bernstein, u_start, u_end));
        cp_idx += p;  // Shared boundary CP between adjacent pieces.
    }

    pieces
}
```

- [ ] **Step 4: Run, expect pass**

Run: `cargo test -p nurbs --features host bezier::tests::extract_single`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/nurbs/src/bezier.rs
git commit -m "nurbs/bezier: extract_bezier_pieces for single-piece curves"
```

---

### Task 3.5: `extract_bezier_pieces` for a multi-piece (interior-knot) curve

**Files:**
- Modify: `rust/nurbs/src/bezier.rs`

- [ ] **Step 1: Write the failing test**

```rust
    #[test]
    fn extract_two_bezier_pieces_from_curve_with_interior_knot() {
        // Quadratic with an interior knot at 0.5.
        let curve = ScalarNurbs::<f64>::try_new(
            2,
            vec![0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0],
            vec![0.0, 1.0, 2.0, 3.0],
            None,
        ).unwrap();

        let pieces = extract_bezier_pieces(&curve);
        assert_eq!(pieces.len(), 2);
        assert_eq!(pieces[0].u_start, 0.0);
        assert_eq!(pieces[0].u_end, 0.5);
        assert_eq!(pieces[1].u_start, 0.5);
        assert_eq!(pieces[1].u_end, 1.0);
        // Eval continuity: pieces[0].evaluate(0.5) == pieces[1].evaluate(0.5).
        let mid_left = pieces[0].evaluate(0.5);
        let mid_right = pieces[1].evaluate(0.5);
        assert!((mid_left - mid_right).abs() < 1e-12);
        // Each piece evaluates correctly.
        for u in [0.0, 0.25, 0.5] {
            let exp = crate::eval::eval(&curve.as_view(), u);
            let got = pieces[0].evaluate(u);
            assert!((exp - got).abs() < 1e-12);
        }
        for u in [0.5, 0.75, 1.0] {
            let exp = crate::eval::eval(&curve.as_view(), u);
            let got = pieces[1].evaluate(u);
            assert!((exp - got).abs() < 1e-12);
        }
    }
```

- [ ] **Step 2: Run, debug if needed**

Run: `cargo test -p nurbs --features host bezier::tests::extract_two`
Expected: PASS. If FAIL, the cp_idx stride between pieces (currently `cp_idx += p`) is wrong; verify against P&T A5.6.

- [ ] **Step 3: Commit**

```bash
git add rust/nurbs/src/bezier.rs
git commit -m "nurbs/bezier: verify extract_bezier_pieces handles interior knots"
```

---

### Task 3.6: `bezier_pieces_to_nurbs` (recompose)

**Files:**
- Modify: `rust/nurbs/src/bezier.rs`

- [ ] **Step 1: Write the failing test (round-trip)**

```rust
    #[test]
    fn bezier_pieces_to_nurbs_round_trips_extraction() {
        let original = ScalarNurbs::<f64>::try_new(
            2,
            vec![0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0],
            vec![0.0, 1.0, 2.0, 3.0],
            None,
        ).unwrap();

        let pieces = extract_bezier_pieces(&original);
        let recomposed = bezier_pieces_to_nurbs(&pieces);

        // Eval-equivalence at sample points (knot vector may differ in multiplicity).
        for u in [0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0] {
            let exp = crate::eval::eval(&original.as_view(), u);
            let got = crate::eval::eval(&recomposed.as_view(), u);
            assert!((exp - got).abs() < 1e-10, "u={u}: exp={exp}, got={got}");
        }
    }
```

- [ ] **Step 2: Run, expect compile failure**

Run: `cargo test -p nurbs --features host bezier::tests::bezier_pieces_to_nurbs_round`
Expected: FAIL.

- [ ] **Step 3: Implement**

Append to `bezier.rs`:
```rust
/// Recompose contiguous Bézier pieces into a single NURBS. Inverse of
/// `extract_bezier_pieces` (modulo knot-multiplicity, which is `degree` at
/// each interior breakpoint = piecewise-Bézier representation).
///
/// Panics if pieces are non-contiguous or have inconsistent degrees.
pub fn bezier_pieces_to_nurbs<T: Float>(pieces: &[BezierPiece<T>]) -> ScalarNurbs<T> {
    assert!(!pieces.is_empty(), "bezier_pieces_to_nurbs: empty input");
    let p = pieces[0].degree();
    for w in pieces.windows(2) {
        assert!(w[0].u_end == w[1].u_start, "non-contiguous Bezier pieces");
        assert!(w[1].degree() == p, "inconsistent degrees");
    }

    // Build knot vector: u_start[0] repeated p+1 times, then each interior
    // boundary repeated p times, then u_end[last] repeated p+1 times.
    let mut knots = Vec::with_capacity((pieces.len() + 1) * p + 2);
    for _ in 0..=p { knots.push(pieces[0].u_start); }
    for piece in &pieces[..pieces.len() - 1] {
        for _ in 0..p { knots.push(piece.u_end); }
    }
    for _ in 0..=p { knots.push(pieces[pieces.len() - 1].u_end); }

    // Build CPs: each piece's Bernstein CPs, with shared boundaries.
    let mut cps: Vec<T> = Vec::with_capacity(pieces.len() * p + 1);
    for (i, piece) in pieces.iter().enumerate() {
        let bernstein = piece.to_bernstein();
        if i == 0 {
            cps.extend_from_slice(&bernstein);
        } else {
            // Skip first CP (shared boundary with previous piece's last).
            cps.extend_from_slice(&bernstein[1..]);
        }
    }

    ScalarNurbs::try_new(p as u8, knots, cps, None)
        .expect("bezier_pieces_to_nurbs: invariants should hold")
}
```

- [ ] **Step 4: Run, expect pass**

Run: `cargo test -p nurbs --features host bezier::tests::bezier_pieces_to_nurbs_round`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/nurbs/src/bezier.rs
git commit -m "nurbs/bezier: bezier_pieces_to_nurbs (recompose)"
```

---

### Task 3.7: `split_piece_at`

**Files:**
- Modify: `rust/nurbs/src/bezier.rs`

- [ ] **Step 1: Write the failing test**

```rust
    #[test]
    fn split_piece_at_preserves_evaluation_on_each_side() {
        // p(u) = 1 + 2 * (u - 0) on [0, 1].
        let original = BezierPiece::<f64> { u_start: 0.0, u_end: 1.0, coeffs: vec![1.0, 2.0] };
        let (left, right) = split_piece_at(&original, 0.4);

        assert_eq!(left.u_start, 0.0);
        assert_eq!(left.u_end, 0.4);
        assert_eq!(right.u_start, 0.4);
        assert_eq!(right.u_end, 1.0);

        // Evaluation matches on each side.
        for u in [0.0, 0.2, 0.4] {
            let exp = original.evaluate(u);
            let got = left.evaluate(u);
            assert!((exp - got).abs() < 1e-12);
        }
        for u in [0.4, 0.7, 1.0] {
            let exp = original.evaluate(u);
            let got = right.evaluate(u);
            assert!((exp - got).abs() < 1e-12);
        }
    }
```

- [ ] **Step 2: Run, expect compile failure**

Run: `cargo test -p nurbs --features host bezier::tests::split_piece_at`
Expected: FAIL.

- [ ] **Step 3: Implement (basis re-shift via Pascal triangle)**

Append to `bezier.rs`:
```rust
/// Split a Bézier piece at an interior point `u_split`, producing two pieces
/// covering [u_start, u_split] and [u_split, u_end] with the same polynomial
/// degree. The polynomial value is preserved on each side.
pub fn split_piece_at<T: Float>(
    piece: &BezierPiece<T>,
    u_split: T,
) -> (BezierPiece<T>, BezierPiece<T>) {
    assert!(u_split > piece.u_start && u_split < piece.u_end, "u_split must be strictly interior");
    let d = piece.degree();

    // Left piece: same monomial coefficients (basis at u_start unchanged); just narrower support.
    let left = BezierPiece {
        u_start: piece.u_start,
        u_end: u_split,
        coeffs: piece.coeffs.clone(),
    };

    // Right piece: re-shift the basis from u_start to u_split.
    // p(u) = Σ c_k (u - u_start)^k. Substitute (u - u_start) = (u - u_split) + delta where delta = u_split - u_start.
    // Expand via binomial: (u - u_start)^k = Σ_{i=0..k} C(k,i) (u - u_split)^i delta^{k-i}.
    // So new_coeff[i] = Σ_{k=i..d} c_k * C(k,i) * delta^{k-i}.
    let delta = u_split - piece.u_start;
    let mut right_coeffs = vec![T::ZERO; d + 1];
    let mut delta_pow = vec![T::ONE; d + 1];
    for k in 1..=d { delta_pow[k] = delta_pow[k - 1] * delta; }

    for i in 0..=d {
        let mut acc = T::ZERO;
        for k in i..=d {
            let c_k_i = T::from_f64(binomial(k, i) as f64);
            acc = acc + piece.coeffs[k] * c_k_i * delta_pow[k - i];
        }
        right_coeffs[i] = acc;
    }

    let right = BezierPiece {
        u_start: u_split,
        u_end: piece.u_end,
        coeffs: right_coeffs,
    };

    (left, right)
}
```

- [ ] **Step 4: Run, expect pass**

Run: `cargo test -p nurbs --features host bezier::tests::split_piece_at`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/nurbs/src/bezier.rs
git commit -m "nurbs/bezier: split_piece_at via Pascal-shifted basis"
```

---

## Phase 4: `algebra.rs` ops + sympy oracle live (~3 days)

### Task 4.1: Add `RationalNotSupported` to `AlgebraError`

**Files:**
- Modify: `rust/nurbs/src/error.rs`

- [ ] **Step 1: Add variant**

In `rust/nurbs/src/error.rs`, modify the `AlgebraError` enum:
```rust
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AlgebraError {
    DegreeExceeded { result_degree: u8, max: u8 },
    KnotMismatch,
    NotImplemented(&'static str),
    SupportMismatch,
    RationalNotSupported {
        operation: &'static str,
        workaround: &'static str,
    },
}
```

And the Display impl:
```rust
            Self::RationalNotSupported { operation, workaround } => {
                write!(f, "{operation} does not support rational input; {workaround}")
            }
```

- [ ] **Step 2: Add a smoke test**

In the existing `error::tests` module, append:
```rust
    #[test]
    fn rational_not_supported_displays_with_workaround() {
        let e = AlgebraError::RationalNotSupported {
            operation: "multiply",
            workaround: "use polynomial_refit",
        };
        let s = format!("{e}");
        assert!(s.contains("multiply"));
        assert!(s.contains("polynomial_refit"));
    }
```

- [ ] **Step 3: Run, expect pass**

Run: `cargo test -p nurbs --features host error::tests`
Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add rust/nurbs/src/error.rs
git commit -m "nurbs: add AlgebraError::RationalNotSupported with operation + workaround"
```

---

### Task 4.2: `multiply` — reject rational input

**Files:**
- Modify: `rust/nurbs/src/algebra.rs`

- [ ] **Step 1: Replace the `multiply` stub**

In `rust/nurbs/src/algebra.rs`, replace the existing `multiply` function (lines 71–79) with:
```rust
/// Multiply two scalar NURBS pointwise: `c(u) = a(u) * b(u)`.
/// Result degree = `degree(a) + degree(b)`.
///
/// Polynomial inputs only in v1; rational inputs return RationalNotSupported.
#[cfg(feature = "host")]
pub fn multiply<T: Float>(
    a: &crate::ScalarNurbs<T>,
    b: &crate::ScalarNurbs<T>,
) -> Result<crate::ScalarNurbs<T>, AlgebraError> {
    if a.weights().is_some() || b.weights().is_some() {
        return Err(AlgebraError::RationalNotSupported {
            operation: "multiply",
            workaround: "use polynomial_refit (Layer 3 utility) before calling",
        });
    }
    todo!("multiply: per-piece product implementation")
}
```

- [ ] **Step 2: Update the existing `multiply_returns_not_implemented_error` test**

The existing test in `algebra.rs` checks for `NotImplemented`. Update it:
```rust
    #[test]
    fn multiply_rejects_rational_input() {
        let a = crate::ScalarNurbs::try_new(
            1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0], Some(vec![1.0, 1.0]),
        ).unwrap();
        let b = a.clone();
        let result = multiply(&a, &b);
        assert!(matches!(
            result,
            Err(crate::AlgebraError::RationalNotSupported { operation: "multiply", .. })
        ));
    }
```
Delete the old `multiply_returns_not_implemented_error` test.

- [ ] **Step 3: Run, expect pass + `todo!()` panic for non-rational unimplemented case**

Run: `cargo test -p nurbs --features host algebra::tests::multiply_rejects_rational`
Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add rust/nurbs/src/algebra.rs
git commit -m "nurbs/algebra: multiply rejects rational input with RationalNotSupported"
```

---

### Task 4.3: `multiply` — happy path for two single-piece linear curves

**Files:**
- Modify: `rust/nurbs/src/algebra.rs`

- [ ] **Step 1: Write the failing test**

Add to `algebra.rs` test module:
```rust
    use crate::eval::eval;

    #[test]
    fn multiply_two_linear_curves_gives_quadratic() {
        // a(u) = u, b(u) = 2u + 1, expected c(u) = u(2u + 1) = 2u^2 + u.
        let a = crate::ScalarNurbs::<f64>::try_new(
            1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0], None,
        ).unwrap();
        let b = crate::ScalarNurbs::<f64>::try_new(
            1, vec![0.0, 0.0, 1.0, 1.0], vec![1.0, 3.0], None,
        ).unwrap();
        let c = multiply(&a, &b).unwrap();
        assert_eq!(c.degree(), 2);
        for u in [0.0, 0.25, 0.5, 0.75, 1.0] {
            let exp = eval(&a.as_view(), u) * eval(&b.as_view(), u);
            let got = eval(&c.as_view(), u);
            assert!((exp - got).abs() < 1e-12, "u={u}: exp={exp}, got={got}");
        }
    }
```

- [ ] **Step 2: Run, expect `todo!()` panic**

Run: `cargo test -p nurbs --features host algebra::tests::multiply_two_linear`
Expected: FAIL with todo!() panic.

- [ ] **Step 3: Implement the per-piece product (single-piece case)**

Replace the `todo!()` body in `multiply` with:
```rust
    let a_pieces = crate::bezier::extract_bezier_pieces(a);
    let b_pieces = crate::bezier::extract_bezier_pieces(b);

    // Refine to common breakpoint set.
    let breakpoints = union_breakpoints(&a_pieces, &b_pieces);
    let a_refined = refine_pieces_to_breakpoints(&a_pieces, &breakpoints);
    let b_refined = refine_pieces_to_breakpoints(&b_pieces, &breakpoints);
    debug_assert_eq!(a_refined.len(), b_refined.len());

    // Per-piece product.
    let mut out_pieces = Vec::with_capacity(a_refined.len());
    for (a_p, b_p) in a_refined.iter().zip(b_refined.iter()) {
        let coeffs = poly_multiply(&a_p.coeffs, &b_p.coeffs);
        out_pieces.push(crate::bezier::BezierPiece {
            u_start: a_p.u_start,
            u_end: a_p.u_end,
            coeffs,
        });
    }

    let result = crate::bezier::bezier_pieces_to_nurbs(&out_pieces);
    Ok(result)
}

/// Compute the union of distinct breakpoints from two piecewise representations.
#[cfg(feature = "host")]
fn union_breakpoints<T: Float>(
    a: &[crate::bezier::BezierPiece<T>],
    b: &[crate::bezier::BezierPiece<T>],
) -> Vec<T> {
    let mut breaks: Vec<T> = Vec::new();
    let mut push_unique = |u: T, breaks: &mut Vec<T>| {
        if !breaks.iter().any(|x| *x == u) {
            breaks.push(u);
        }
    };
    for piece in a {
        push_unique(piece.u_start, &mut breaks);
        push_unique(piece.u_end, &mut breaks);
    }
    for piece in b {
        push_unique(piece.u_start, &mut breaks);
        push_unique(piece.u_end, &mut breaks);
    }
    breaks.sort_by(|x, y| x.partial_cmp(y).unwrap());
    breaks
}

/// Refine a list of contiguous Bézier pieces so that the piece boundaries
/// coincide with the given (sorted) breakpoints.
#[cfg(feature = "host")]
fn refine_pieces_to_breakpoints<T: Float>(
    pieces: &[crate::bezier::BezierPiece<T>],
    breakpoints: &[T],
) -> Vec<crate::bezier::BezierPiece<T>> {
    let mut result: Vec<crate::bezier::BezierPiece<T>> = Vec::new();
    for piece in pieces {
        let mut current = piece.clone();
        // Find any breakpoints strictly inside (current.u_start, current.u_end).
        let mut interior: Vec<T> = breakpoints
            .iter()
            .filter(|&&b| b > current.u_start && b < current.u_end)
            .copied()
            .collect();
        interior.sort_by(|x, y| x.partial_cmp(y).unwrap());
        for u in interior {
            let (left, right) = crate::bezier::split_piece_at(&current, u);
            result.push(left);
            current = right;
        }
        result.push(current);
    }
    result
}

/// Polynomial coefficient convolution: out[k] = Σ_{i+j=k} a[i] * b[j].
#[cfg(feature = "host")]
fn poly_multiply<T: Float>(a: &[T], b: &[T]) -> Vec<T> {
    let mut out = vec![T::ZERO; a.len() + b.len() - 1];
    for (i, ai) in a.iter().enumerate() {
        for (j, bj) in b.iter().enumerate() {
            out[i + j] = out[i + j] + *ai * *bj;
        }
    }
    out
}
```

- [ ] **Step 4: Run, expect pass**

Run: `cargo test -p nurbs --features host algebra::tests::multiply_two_linear`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/nurbs/src/algebra.rs
git commit -m "nurbs/algebra: multiply via Bezier extraction + per-piece poly product"
```

---

### Task 4.4: `multiply` — multi-piece input (interior knots in operands)

**Files:**
- Modify: `rust/nurbs/src/algebra.rs`

- [ ] **Step 1: Write the failing test**

```rust
    #[test]
    fn multiply_curves_with_different_interior_knots() {
        // a has interior knot at 0.4, b has interior knot at 0.7.
        let a = crate::ScalarNurbs::<f64>::try_new(
            2,
            vec![0.0, 0.0, 0.0, 0.4, 1.0, 1.0, 1.0],
            vec![0.0, 1.0, 2.0, 3.0],
            None,
        ).unwrap();
        let b = crate::ScalarNurbs::<f64>::try_new(
            2,
            vec![0.0, 0.0, 0.0, 0.7, 1.0, 1.0, 1.0],
            vec![1.0, 2.0, 0.0, 1.0],
            None,
        ).unwrap();
        let c = multiply(&a, &b).unwrap();
        assert_eq!(c.degree(), 4);
        for u in [0.0, 0.2, 0.4, 0.5, 0.7, 0.9, 1.0] {
            let exp = eval(&a.as_view(), u) * eval(&b.as_view(), u);
            let got = eval(&c.as_view(), u);
            assert!((exp - got).abs() < 1e-10, "u={u}: exp={exp}, got={got}");
        }
    }
```

- [ ] **Step 2: Run**

Run: `cargo test -p nurbs --features host algebra::tests::multiply_curves_with_different`
Expected: PASS (already implemented via union breakpoints in Task 4.3).

- [ ] **Step 3: Commit**

```bash
git add rust/nurbs/src/algebra.rs
git commit -m "nurbs/algebra: verify multiply handles different interior knots"
```

---

### Task 4.5: Add `knot_remove_redundant` helper

**Files:**
- Modify: `rust/nurbs/src/algebra.rs`

- [ ] **Step 1: Write the failing test**

```rust
    #[test]
    fn knot_remove_redundant_simplifies_overproduct() {
        // Square of a degree-1 curve: produces degree-2 with a needless interior knot if any.
        let a = crate::ScalarNurbs::<f64>::try_new(
            1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0], None,
        ).unwrap();
        let b = a.clone();
        let mut c = multiply(&a, &b).unwrap();
        let initial_knot_count = c.knots().len();

        knot_remove_redundant(&mut c, 1e-10);

        // For a single-piece input, no interior knots to remove; result unchanged.
        assert_eq!(c.knots().len(), initial_knot_count);
        // Eval still correct.
        for u in [0.0, 0.5, 1.0] {
            let exp = eval(&a.as_view(), u) * eval(&b.as_view(), u);
            let got = eval(&c.as_view(), u);
            assert!((exp - got).abs() < 1e-10);
        }
    }
```

- [ ] **Step 2: Run, expect compile failure**

Run: `cargo test -p nurbs --features host algebra::tests::knot_remove_redundant`
Expected: FAIL.

- [ ] **Step 3: Implement**

Append to `algebra.rs`:
```rust
/// Iterate over interior knots and apply `remove_knot` with the given tolerance,
/// dropping knots whose removal preserves the curve within `tol`. Used by
/// `multiply` and `convolve` to expose natural smoothness of the result.
#[cfg(feature = "host")]
pub(crate) fn knot_remove_redundant<T: Float>(curve: &mut crate::ScalarNurbs<T>, tol: T) {
    let p = curve.degree() as usize;
    loop {
        // Snapshot interior knot values.
        let knots: Vec<T> = curve.knots().to_vec();
        let interior: Vec<T> = {
            let mut seen: Vec<T> = Vec::new();
            for &k in &knots[p + 1..knots.len() - p - 1] {
                if !seen.contains(&k) {
                    seen.push(k);
                }
            }
            seen
        };

        let mut removed_any = false;
        for u in interior {
            let (new_curve, count) = crate::knot::remove_knot(curve, u, 1, tol);
            if count > 0 {
                *curve = new_curve;
                removed_any = true;
            }
        }
        if !removed_any { break; }
    }
}
```

- [ ] **Step 4: Run, expect pass**

Run: `cargo test -p nurbs --features host algebra::tests::knot_remove_redundant`
Expected: PASS.

- [ ] **Step 5: Wire into `multiply`**

In `multiply`, after `let result = bezier_pieces_to_nurbs(&out_pieces);`, replace `Ok(result)` with:
```rust
    let mut result = bezier_pieces_to_nurbs(&out_pieces);
    knot_remove_redundant(&mut result, T::from_f64(1e-12));
    Ok(result)
```

- [ ] **Step 6: Run all multiply tests**

Run: `cargo test -p nurbs --features host algebra::tests::multiply`
Expected: PASS.

- [ ] **Step 7: Commit**

```bash
git add rust/nurbs/src/algebra.rs
git commit -m "nurbs/algebra: knot_remove_redundant post-pass for multiply"
```

---

### Task 4.6: Add `PiecewisePolynomialKernel` type

**Files:**
- Modify: `rust/nurbs/src/algebra.rs`

- [ ] **Step 1: Write the failing test**

Add to `algebra.rs` test module:
```rust
    #[test]
    fn single_poly_kernel_constructs_one_piece() {
        let k = PiecewisePolynomialKernel::single_poly(vec![1.0, 0.5_f64], (-1.0, 1.0));
        assert_eq!(k.pieces.len(), 1);
        assert_eq!(k.pieces[0].u_start, -1.0);
        assert_eq!(k.pieces[0].u_end, 1.0);
        assert_eq!(k.pieces[0].coeffs, vec![1.0, 0.5]);
    }
```

- [ ] **Step 2: Run, expect compile failure**

Run: `cargo test -p nurbs --features host algebra::tests::single_poly_kernel`
Expected: FAIL.

- [ ] **Step 3: Implement (replaces existing `PolynomialKernel` struct)**

In `rust/nurbs/src/algebra.rs`, replace the existing `PolynomialKernel` struct and impl (~lines 50–64) with:
```rust
/// Polynomial kernel for convolution. Pieces are contiguous and ordered.
/// Each piece is a polynomial in the Pascal-shifted monomial basis.
#[cfg(feature = "host")]
#[derive(Debug, Clone)]
pub struct PiecewisePolynomialKernel<T: Float> {
    pub pieces: Vec<crate::bezier::BezierPiece<T>>,
}

#[cfg(feature = "host")]
impl<T: Float> PiecewisePolynomialKernel<T> {
    /// Build a single-piece kernel from monomial coefficients
    /// `coeffs[k] * (u - u_start)^k` on the interval `support`.
    pub fn single_poly(coeffs: Vec<T>, support: (T, T)) -> Self {
        let piece = crate::bezier::BezierPiece {
            u_start: support.0,
            u_end: support.1,
            coeffs,
        };
        Self { pieces: vec![piece] }
    }

    /// Total support of the kernel: from first piece's u_start to last piece's u_end.
    pub fn support(&self) -> (T, T) {
        (self.pieces.first().unwrap().u_start, self.pieces.last().unwrap().u_end)
    }
}
```

- [ ] **Step 4: Update the old `convolve_with_polynomial_kernel` test**

In `algebra.rs`, replace the existing `convolve_returns_not_implemented_error` test with:
```rust
    #[test]
    fn kernel_support_returns_endpoints() {
        let k = PiecewisePolynomialKernel::single_poly(vec![1.0_f64], (-0.5, 0.5));
        assert_eq!(k.support(), (-0.5, 0.5));
    }
```

Delete the old `convolve_returns_not_implemented_error` and the old `convolve_with_polynomial_kernel` stub function.

- [ ] **Step 5: Run, expect pass**

Run: `cargo test -p nurbs --features host algebra::tests::single_poly_kernel algebra::tests::kernel_support`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add rust/nurbs/src/algebra.rs
git commit -m "nurbs/algebra: PiecewisePolynomialKernel replaces PolynomialKernel stub"
```

---

### Task 4.7: `convolve` — reject rational input + minimal scaffold

**Files:**
- Modify: `rust/nurbs/src/algebra.rs`

- [ ] **Step 1: Write the failing test**

```rust
    #[test]
    fn convolve_rejects_rational_input() {
        let curve = crate::ScalarNurbs::try_new(
            1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0_f64, 1.0], Some(vec![1.0, 1.0]),
        ).unwrap();
        let kernel = PiecewisePolynomialKernel::single_poly(vec![1.0_f64], (-0.1, 0.1));
        let result = convolve(&curve, &kernel);
        assert!(matches!(
            result,
            Err(AlgebraError::RationalNotSupported { operation: "convolve", .. })
        ));
    }
```

- [ ] **Step 2: Run, expect compile failure**

Run: `cargo test -p nurbs --features host algebra::tests::convolve_rejects_rational`
Expected: FAIL.

- [ ] **Step 3: Implement the rejection scaffold**

Append to `algebra.rs`:
```rust
/// Convolve a polynomial NURBS with a piecewise polynomial kernel:
/// `y(u) = ∫ x(s) w(u - s) ds`.
///
/// Output domain = Minkowski sum of input and kernel supports. Caller
/// (Layer 3) handles cross-segment stitching for trajectories.
///
/// Polynomial inputs only in v1.
#[cfg(feature = "host")]
pub fn convolve<T: Float>(
    curve: &crate::ScalarNurbs<T>,
    kernel: &PiecewisePolynomialKernel<T>,
) -> Result<crate::ScalarNurbs<T>, AlgebraError> {
    if curve.weights().is_some() {
        return Err(AlgebraError::RationalNotSupported {
            operation: "convolve",
            workaround: "use polynomial_refit (Layer 3 utility) before calling",
        });
    }
    todo!("convolve: piecewise integration implementation")
}
```

- [ ] **Step 4: Run, expect pass**

Run: `cargo test -p nurbs --features host algebra::tests::convolve_rejects_rational`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/nurbs/src/algebra.rs
git commit -m "nurbs/algebra: convolve scaffold (rejects rational input)"
```

---

### Task 4.8: `convolve` — implement `integrate_product_piece` helper

**Files:**
- Modify: `rust/nurbs/src/algebra.rs`

- [ ] **Step 1: Write a focused test for the inner integration**

```rust
    #[test]
    fn integrate_product_constant_input_constant_kernel_yields_linear_result() {
        // x(s) = 2 on [0, 1], w(t) = 3 on [-0.5, 0.5].
        // y(u) = ∫ x(s) w(u - s) ds, integration range = intersection of
        // [u - 0.5, u + 0.5] with [0, 1].
        // For u ∈ [0.5, 0.5] (single point), y = 2*3*1 = 6.
        // Generally y(u) = 6 * (length of overlap window).

        let x = crate::bezier::BezierPiece::<f64> {
            u_start: 0.0, u_end: 1.0, coeffs: vec![2.0],  // constant 2
        };
        let w = crate::bezier::BezierPiece::<f64> {
            u_start: -0.5, u_end: 0.5, coeffs: vec![3.0],  // constant 3
        };

        // Integrate over output sub-interval [0.5, 1.0] where the kernel window
        // shrinks linearly (from full overlap to half overlap at u=1.0).
        let contribution = integrate_product_piece(&x, &w, 0.5, 1.0);

        // Expected: y(u) = 6 * (1.0 - (u - 0.5)) for u ∈ [0.5, 1.0]
        //                = 6 * (1.5 - u)
        //                = 9 - 6u
        // In Pascal-shifted basis at α = 0.5: y(u) = 9 - 6u = 9 - 6*(0.5 + (u - 0.5))
        //                                          = 6 - 6 * (u - 0.5)
        // So coeffs at u_start = 0.5 should be [6.0, -6.0].
        assert!((contribution.coeffs[0] - 6.0).abs() < 1e-10);
        assert!((contribution.coeffs[1] - (-6.0)).abs() < 1e-10);
    }
```

- [ ] **Step 2: Run, expect compile failure**

Run: `cargo test -p nurbs --features host algebra::tests::integrate_product_constant`
Expected: FAIL.

- [ ] **Step 3: Implement `integrate_product_piece`**

The math, restated:
- `x(s)` polynomial in `(s - x.u_start)` of degree `d_x`.
- `w(u-s)` polynomial in `(u - s - w.u_start)` of degree `d_w`.
- Integration limits: `s_lo(u) = max(x.u_start, u - w.u_end)`, `s_hi(u) = min(x.u_end, u - w.u_start)`. Both linear in `u`.
- Output polynomial in `(u - α)`, degree `d_x + d_w + 1`.

Append to `algebra.rs`:
```rust
/// Integrate `∫ x(s) w(u - s) ds` over the (s, u) region where x's piece i and
/// w's piece j are simultaneously active, for u in [α, β]. Returns the
/// contribution as a BezierPiece on [α, β] with degree d_x + d_w + 1.
///
/// Algorithm sketch (per spec §6.4):
/// 1. Re-express w(u-s) in s-basis with u-dependent coefficients (binomial expansion).
/// 2. Multiply by x(s); result is a polynomial in s with u-dependent coefficients.
/// 3. Integrate s^k → s^(k+1)/(k+1), evaluate at s_hi(u) and s_lo(u).
/// 4. Both s_lo and s_hi are linear in u, so output is polynomial in u.
#[cfg(feature = "host")]
fn integrate_product_piece<T: Float>(
    x: &crate::bezier::BezierPiece<T>,
    w: &crate::bezier::BezierPiece<T>,
    alpha: T,
    beta: T,
) -> crate::bezier::BezierPiece<T> {
    let d_x = x.degree();
    let d_w = w.degree();
    let out_degree = d_x + d_w + 1;

    // Integration limits as polynomials in u (degree 1, in absolute u, NOT shifted).
    // s_lo(u) = max(x.u_start, u - w.u_end)
    // s_hi(u) = min(x.u_end,   u - w.u_start)
    //
    // For u in [α, β] by construction of out_breaks, the active branch of max/min
    // is constant; we can determine it from the value at u = (α + β) / 2.
    let u_mid = (alpha + beta) * T::from_f64(0.5);
    let lo_branch_curve = u_mid - w.u_end > x.u_start;  // true → s_lo(u) = u - w.u_end
    let hi_branch_curve = u_mid - w.u_start < x.u_end;  // true → s_hi(u) = u - w.u_start

    // s_lo(u) and s_hi(u) as (constant, linear-in-u-coeff) tuples.
    let (s_lo_c, s_lo_u): (T, T) = if lo_branch_curve {
        (-w.u_end, T::ONE)
    } else {
        (x.u_start, T::ZERO)
    };
    let (s_hi_c, s_hi_u): (T, T) = if hi_branch_curve {
        (-w.u_start, T::ONE)
    } else {
        (x.u_end, T::ZERO)
    };

    // The integrand is x(s) * w(u - s).
    // Express w_j(u - s) = w_j(t) where t = u - s - w.u_start (Pascal-shifted basis arg of w):
    //   w(u - s) = Σ_j w.coeffs[j] * (u - s - w.u_start)^j.
    // To multiply by x(s) = Σ_i x.coeffs[i] * (s - x.u_start)^i, change x's basis to absolute s
    // and w's argument to expansion in (s, u).

    // Step A: Convert x.coeffs to absolute-s monomial basis.
    let x_abs = pascal_shift_to_absolute(&x.coeffs, x.u_start);

    // Step B: Convert w.coeffs to absolute-(u-s) monomial basis (in z = u-s).
    let w_abs_z = pascal_shift_to_absolute(&w.coeffs, w.u_start);
    // Then expand each z^j as (u - s)^j via binomial, giving polynomial in u and s.
    // w_abs_z[j] * (u - s)^j = w_abs_z[j] * Σ_l C(j, l) * u^(j-l) * (-s)^l
    //                        = Σ_l w_abs_z[j] * C(j, l) * (-1)^l * u^(j-l) * s^l

    // Build a 2D coefficient table: integrand[m][n] = coefficient of u^m * s^n.
    // After multiplying by x_abs (polynomial in s only):
    //   integrand[m][k] = Σ_{l: l + i = k} x_abs[i] * (Σ_{j ≥ l} w_abs_z[j] * C(j, l) * (-1)^l * δ_{j-l, m})
    let max_m = d_w;
    let max_n = d_x + d_w;
    let mut integrand = vec![vec![T::ZERO; max_n + 1]; max_m + 1];

    for j in 0..=d_w {
        for l in 0..=j {
            let m = j - l;
            let sign = if l % 2 == 0 { T::ONE } else { -T::ONE };
            let c_jl = T::from_f64(binomial(j, l) as f64);
            let coef = sign * c_jl * w_abs_z[j];
            for i in 0..=d_x {
                let n = l + i;
                integrand[m][n] = integrand[m][n] + coef * x_abs[i];
            }
        }
    }

    // Step C: Integrate s^n → s^(n+1) / (n+1), evaluate at s_hi(u) - s_lo(u).
    // For each (m, n) in the integrand, the contribution to y(u) is:
    //   (integrand[m][n] / (n+1)) * (s_hi(u)^(n+1) - s_lo(u)^(n+1)) * u^m
    // s_hi(u)^(n+1) and s_lo(u)^(n+1) are polynomials in u of degree n+1.
    let mut y_abs = vec![T::ZERO; out_degree + 1];
    for m in 0..=max_m {
        for n in 0..=max_n {
            if integrand[m][n] == T::ZERO { continue; }
            let inv = integrand[m][n] / T::from_f64((n + 1) as f64);
            // (c + a*u)^(n+1) expanded as polynomial in u.
            let hi_pow = power_of_linear(s_hi_c, s_hi_u, n + 1);
            let lo_pow = power_of_linear(s_lo_c, s_lo_u, n + 1);
            // Multiply each by u^m and accumulate.
            for k in 0..hi_pow.len() {
                let target = k + m;
                if target <= out_degree {
                    y_abs[target] = y_abs[target] + inv * (hi_pow[k] - lo_pow[k]);
                }
            }
        }
    }

    // Convert from absolute-u monomial to Pascal-shifted-at-α basis.
    let y_shifted = absolute_to_pascal_shift(&y_abs, alpha);
    crate::bezier::BezierPiece {
        u_start: alpha,
        u_end: beta,
        coeffs: y_shifted,
    }
}

/// Expand (c + a*u)^p as a polynomial in u (length p+1, ascending power).
fn power_of_linear<T: Float>(c: T, a: T, p: usize) -> Vec<T> {
    let mut out = vec![T::ZERO; p + 1];
    let mut c_pow = vec![T::ONE; p + 1];
    let mut a_pow = vec![T::ONE; p + 1];
    for k in 1..=p { c_pow[k] = c_pow[k - 1] * c; a_pow[k] = a_pow[k - 1] * a; }
    for k in 0..=p {
        let bin = T::from_f64(binomial(p, k) as f64);
        out[k] = bin * c_pow[p - k] * a_pow[k];
    }
    out
}

/// Convert Pascal-shifted-at-`shift` coefficients to absolute monomial.
/// p(u) = Σ c_k * (u - shift)^k → Σ c'_n * u^n
fn pascal_shift_to_absolute<T: Float>(shifted: &[T], shift: T) -> Vec<T> {
    let d = shifted.len() - 1;
    let mut out = vec![T::ZERO; d + 1];
    for k in 0..=d {
        let exp = power_of_linear(-shift, T::ONE, k);  // (u - shift)^k = (-shift + u)^k
        for n in 0..exp.len() {
            out[n] = out[n] + shifted[k] * exp[n];
        }
    }
    out
}

/// Inverse: convert absolute monomial to Pascal-shifted-at-`shift`.
/// Σ c_n * u^n → Σ c'_k * (u - shift)^k where u^n = Σ_k C(n, k) * shift^(n-k) * (u - shift)^k.
fn absolute_to_pascal_shift<T: Float>(absolute: &[T], shift: T) -> Vec<T> {
    let d = absolute.len() - 1;
    let mut out = vec![T::ZERO; d + 1];
    let mut shift_pow = vec![T::ONE; d + 1];
    for k in 1..=d { shift_pow[k] = shift_pow[k - 1] * shift; }
    for n in 0..=d {
        for k in 0..=n {
            let bin = T::from_f64(binomial(n, k) as f64);
            out[k] = out[k] + absolute[n] * bin * shift_pow[n - k];
        }
    }
    out
}

// `binomial` is defined as `pub(crate)` in `bezier.rs` (Task 3.2). Import it
// at the top of algebra.rs: `use crate::bezier::binomial;` — and bump the
// bezier.rs binomial visibility from private `fn` to `pub(crate) fn` in this task.
```

- [ ] **Step 4: Run, debug if needed**

Run: `cargo test -p nurbs --features host algebra::tests::integrate_product_constant`
Expected: PASS. If FAIL, this is a complex integration; trace the basis conversions step-by-step against the test's expected `[6.0, -6.0]` output. The `binomial` may collide with the bezier.rs version — change one to `pub(crate)` and import.

- [ ] **Step 5: Commit**

```bash
git add rust/nurbs/src/algebra.rs
git commit -m "nurbs/algebra: integrate_product_piece + basis-conversion helpers"
```

---

### Task 4.9: `convolve` — wire up the outer loop

**Files:**
- Modify: `rust/nurbs/src/algebra.rs`

- [ ] **Step 1: Write the failing test (simple convolve case)**

```rust
    #[test]
    fn convolve_constant_input_with_constant_kernel_gives_triangle() {
        // x(s) = 2 on [0, 1], w(t) = 3 on [-0.5, 0.5].
        // Convolution support: [0 + (-0.5), 1 + 0.5] = [-0.5, 1.5].
        // Output: triangle peaking in [0.5, 0.5] at value 6, sloping linearly to 0 at boundaries.
        let x = crate::ScalarNurbs::<f64>::try_new(
            1, vec![0.0, 0.0, 1.0, 1.0], vec![2.0, 2.0], None,
        ).unwrap();
        let kernel = PiecewisePolynomialKernel::single_poly(vec![3.0_f64], (-0.5, 0.5));

        let y = convolve(&x, &kernel).unwrap();

        // Spot-check: at u = 0.5, the kernel window [0.0, 1.0] is fully inside x's support,
        // so y(0.5) = ∫_{0}^{1} 2 * 3 ds = 6.
        let val = eval(&y.as_view(), 0.5);
        assert!((val - 6.0).abs() < 1e-10, "y(0.5) = {val}, expected 6");

        // At u = -0.5 (left boundary of output), y = 0.
        let val_lo = eval(&y.as_view(), -0.5);
        assert!(val_lo.abs() < 1e-10, "y(-0.5) = {val_lo}, expected 0");

        // At u = 1.5 (right boundary), y = 0.
        let val_hi = eval(&y.as_view(), 1.5);
        assert!(val_hi.abs() < 1e-10, "y(1.5) = {val_hi}, expected 0");
    }
```

- [ ] **Step 2: Run, expect `todo!()` panic**

Run: `cargo test -p nurbs --features host algebra::tests::convolve_constant_input`
Expected: FAIL with todo!() panic.

- [ ] **Step 3: Wire the outer loop**

Replace the `todo!()` body in `convolve` with:
```rust
    let x_pieces = crate::bezier::extract_bezier_pieces(curve);
    let w_pieces = &kernel.pieces;

    // Compute output breakpoints: cross-sum of input and kernel breakpoints.
    let x_breaks: Vec<T> = {
        let mut v: Vec<T> = Vec::new();
        for p in &x_pieces { if !v.contains(&p.u_start) { v.push(p.u_start); } }
        v.push(x_pieces.last().unwrap().u_end);
        v
    };
    let w_breaks: Vec<T> = {
        let mut v: Vec<T> = Vec::new();
        for p in w_pieces { if !v.contains(&p.u_start) { v.push(p.u_start); } }
        v.push(w_pieces.last().unwrap().u_end);
        v
    };
    let mut out_breaks: Vec<T> = Vec::new();
    for xb in &x_breaks {
        for wb in &w_breaks {
            let s = *xb + *wb;
            if !out_breaks.iter().any(|x| *x == s) {
                out_breaks.push(s);
            }
        }
    }
    out_breaks.sort_by(|a, b| a.partial_cmp(b).unwrap());

    let degree = x_pieces[0].degree() + w_pieces[0].degree() + 1;

    let mut out_pieces: Vec<crate::bezier::BezierPiece<T>> = Vec::with_capacity(out_breaks.len() - 1);
    for win in out_breaks.windows(2) {
        let alpha = win[0];
        let beta = win[1];
        let mut accum = crate::bezier::BezierPiece::<T>::zero(alpha, beta, degree);

        for x_p in &x_pieces {
            for w_p in w_pieces {
                // For u in [alpha, beta], the integration range over s is
                // [s_lo(u), s_hi(u)] = [max(x_p.u_start, u - w_p.u_end), min(x_p.u_end, u - w_p.u_start)].
                // Non-empty if s_lo(u) < s_hi(u) somewhere in [alpha, beta].
                // Sufficient check: at u_mid, the range is non-empty.
                let u_mid = (alpha + beta) * T::from_f64(0.5);
                let s_lo = (x_p.u_start).max(u_mid - w_p.u_end);
                let s_hi = (x_p.u_end).min(u_mid - w_p.u_start);
                if s_lo >= s_hi { continue; }

                let contribution = integrate_product_piece(x_p, w_p, alpha, beta);
                accum = (&accum + &contribution).expect("same-support accumulation");
            }
        }
        out_pieces.push(accum);
    }

    let mut result = crate::bezier::bezier_pieces_to_nurbs(&out_pieces);
    knot_remove_redundant(&mut result, T::from_f64(1e-12));
    Ok(result)
}
```

- [ ] **Step 4: Run, expect pass**

Run: `cargo test -p nurbs --features host algebra::tests::convolve_constant_input`
Expected: PASS. If FAIL, the `integrate_product_piece` may have basis bugs; isolate by adding intermediate eval prints.

- [ ] **Step 5: Commit**

```bash
git add rust/nurbs/src/algebra.rs
git commit -m "nurbs/algebra: convolve outer loop with cross-sum breakpoints"
```

---

### Task 4.10: `convolve` — non-trivial polynomial input

**Files:**
- Modify: `rust/nurbs/src/algebra.rs`

- [ ] **Step 1: Write the failing test**

```rust
    #[test]
    fn convolve_linear_input_with_constant_kernel_yields_correct_integral() {
        // x(s) = s on [0, 1], w(t) = 1 on [-0.25, 0.25].
        // y(u) = ∫_{u-0.25}^{u+0.25} s ds = ((u+0.25)^2 - (u-0.25)^2) / 2 = u/2
        // for u in [0.25, 0.75] (kernel window fully inside x's support).
        // Output value at u=0.5 should be 0.5/2... wait, let me recompute:
        //   y(0.5) = ∫_{0.25}^{0.75} s ds = (0.75^2 - 0.25^2) / 2 = (0.5625 - 0.0625) / 2 = 0.25
        // and 0.5 * 0.5 = 0.25 ✓ (since width = 0.5 and average value of s in window = 0.5).
        let x = crate::ScalarNurbs::<f64>::try_new(
            1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0], None,
        ).unwrap();
        let kernel = PiecewisePolynomialKernel::single_poly(vec![1.0_f64], (-0.25, 0.25));

        let y = convolve(&x, &kernel).unwrap();

        let val = eval(&y.as_view(), 0.5);
        assert!((val - 0.25).abs() < 1e-10, "y(0.5) = {val}, expected 0.25");
    }
```

- [ ] **Step 2: Run**

Run: `cargo test -p nurbs --features host algebra::tests::convolve_linear_input`
Expected: PASS (the existing implementation handles this).

- [ ] **Step 3: Commit**

```bash
git add rust/nurbs/src/algebra.rs
git commit -m "nurbs/algebra: verify convolve handles linear input"
```

---

### Task 4.11: Sympy oracle corpus generator script

**Files:**
- Create: `rust/nurbs/tests/scripts/generate_algebra_corpus.py`
- Create: `rust/nurbs/tests/data/algebra_corpus.json` (committed; regenerated only when adding fixtures)

- [ ] **Step 1: Write the script**

Create `rust/nurbs/tests/scripts/generate_algebra_corpus.py`:
```python
#!/usr/bin/env python3
"""Generate symbolic-reference corpus for NURBS algebra ops via sympy.

Each fixture provides:
- multiply(a, b): NURBS a, NURBS b, sample evaluations of c = a*b
- convolve(curve, kernel): NURBS, kernel, sample evaluations of y = curve*kernel

Run with:
    pip install sympy
    python rust/nurbs/tests/scripts/generate_algebra_corpus.py > rust/nurbs/tests/data/algebra_corpus.json
"""

import json
import sympy as sp


def linear_curve_data():
    # a(u) = u over [0, 1].
    return {
        "degree": 1,
        "knots": [0.0, 0.0, 1.0, 1.0],
        "control_points": [0.0, 1.0],
        "weights": None,
    }


def quadratic_curve_data():
    # b(u) = u^2 over [0, 1] expressed as a degree-2 NURBS.
    # Need cps = [0, 0, 1] for u^2 in degree-2 Bernstein on [0, 1].
    return {
        "degree": 2,
        "knots": [0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        "control_points": [0.0, 0.0, 1.0],
        "weights": None,
    }


def multiply_fixture_linear_x_linear():
    """a(u) = u, b(u) = u, expected c(u) = u^2."""
    a = linear_curve_data()
    b = linear_curve_data()
    samples_u = [0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0]
    samples = [{"u": u, "value": u * u} for u in samples_u]
    return {
        "name": "multiply_linear_squared",
        "operation": "multiply",
        "a": a,
        "b": b,
        "samples": samples,
    }


def convolve_fixture_constant_x_constant():
    """x(s) = 2 on [0, 1], w(t) = 3 on [-0.5, 0.5]."""
    samples = []
    # Symbolic computation:
    s, u = sp.symbols('s u', real=True)
    x_sym = sp.Piecewise((2, sp.And(s >= 0, s <= 1)), (0, True))
    # Compute y(u) numerically at sample points by integration.
    for u_val in [-0.5, -0.25, 0.0, 0.25, 0.5, 0.75, 1.0, 1.25, 1.5]:
        s_lo = max(0, u_val - 0.5)
        s_hi = min(1, u_val + 0.5)
        y = max(0, (s_hi - s_lo) * 2 * 3)
        samples.append({"u": u_val, "value": y})
    return {
        "name": "convolve_constant_x_constant",
        "operation": "convolve",
        "curve": {
            "degree": 1,
            "knots": [0.0, 0.0, 1.0, 1.0],
            "control_points": [2.0, 2.0],
            "weights": None,
        },
        "kernel": {
            "pieces": [
                {"u_start": -0.5, "u_end": 0.5, "coeffs": [3.0]},
            ],
        },
        "samples": samples,
    }


def main():
    fixtures = [
        multiply_fixture_linear_x_linear(),
        convolve_fixture_constant_x_constant(),
    ]
    print(json.dumps({"fixtures": fixtures}, indent=2))


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Run the generator**

```bash
python3 rust/nurbs/tests/scripts/generate_algebra_corpus.py > rust/nurbs/tests/data/algebra_corpus.json
```
Expected: Creates `algebra_corpus.json`.

- [ ] **Step 3: Commit**

```bash
git add rust/nurbs/tests/scripts/generate_algebra_corpus.py rust/nurbs/tests/data/algebra_corpus.json
git commit -m "nurbs/tests: sympy-based algebra corpus generator + initial fixtures"
```

---

### Task 4.12: Rust oracle harness for the algebra corpus

**Files:**
- Create: `rust/nurbs/tests/algebra_oracle.rs`

- [ ] **Step 1: Write the harness**

Create `rust/nurbs/tests/algebra_oracle.rs`:
```rust
//! Cross-check our algebra ops against a sympy-generated oracle corpus.
//! Corpus file: `tests/data/algebra_corpus.json` (regenerated via the
//! Python script in `tests/scripts/`).

#![cfg(feature = "host")]

use serde_json::Value;
use std::fs;
use std::path::PathBuf;

const TOLERANCE: f64 = 1e-9;

fn parse_curve(v: &Value) -> nurbs::ScalarNurbs<f64> {
    let degree = v["degree"].as_u64().unwrap() as u8;
    let knots: Vec<f64> = v["knots"].as_array().unwrap()
        .iter().map(|x| x.as_f64().unwrap()).collect();
    let cps: Vec<f64> = v["control_points"].as_array().unwrap()
        .iter().map(|x| x.as_f64().unwrap()).collect();
    let weights: Option<Vec<f64>> = if v["weights"].is_null() {
        None
    } else {
        Some(v["weights"].as_array().unwrap()
            .iter().map(|x| x.as_f64().unwrap()).collect())
    };
    nurbs::ScalarNurbs::try_new(degree, knots, cps, weights).unwrap()
}

fn parse_kernel(v: &Value) -> nurbs::algebra::PiecewisePolynomialKernel<f64> {
    let pieces: Vec<nurbs::BezierPiece<f64>> = v["pieces"].as_array().unwrap()
        .iter().map(|p| {
            let u_start = p["u_start"].as_f64().unwrap();
            let u_end = p["u_end"].as_f64().unwrap();
            let coeffs: Vec<f64> = p["coeffs"].as_array().unwrap()
                .iter().map(|c| c.as_f64().unwrap()).collect();
            nurbs::BezierPiece { u_start, u_end, coeffs }
        }).collect();
    nurbs::algebra::PiecewisePolynomialKernel { pieces }
}

#[test]
fn algebra_oracle_matches_for_corpus() {
    let path: PathBuf = [
        env!("CARGO_MANIFEST_DIR"), "tests", "data", "algebra_corpus.json",
    ].iter().collect();
    let raw = fs::read_to_string(&path).expect("corpus must exist");
    let v: Value = serde_json::from_str(&raw).expect("valid JSON");

    for fixture in v["fixtures"].as_array().unwrap() {
        let name = fixture["name"].as_str().unwrap();
        let op = fixture["operation"].as_str().unwrap();
        let result = match op {
            "multiply" => {
                let a = parse_curve(&fixture["a"]);
                let b = parse_curve(&fixture["b"]);
                nurbs::algebra::multiply(&a, &b)
                    .unwrap_or_else(|e| panic!("{name}: multiply failed: {e:?}"))
            }
            "convolve" => {
                let curve = parse_curve(&fixture["curve"]);
                let kernel = parse_kernel(&fixture["kernel"]);
                nurbs::algebra::convolve(&curve, &kernel)
                    .unwrap_or_else(|e| panic!("{name}: convolve failed: {e:?}"))
            }
            other => panic!("unknown operation: {other}"),
        };

        for sample in fixture["samples"].as_array().unwrap() {
            let u = sample["u"].as_f64().unwrap();
            let expected = sample["value"].as_f64().unwrap();
            let got = nurbs::eval::eval(&result.as_view(), u);
            let diff = (got - expected).abs();
            assert!(
                diff < TOLERANCE,
                "{name} u={u}: got {got} expected {expected} (diff {diff})"
            );
        }
    }
}
```

- [ ] **Step 2: Run**

Run: `cargo test -p nurbs --features host --test algebra_oracle`
Expected: PASS for both fixtures.

- [ ] **Step 3: Commit**

```bash
git add rust/nurbs/tests/algebra_oracle.rs
git commit -m "nurbs/tests: algebra oracle harness against sympy corpus"
```

---

## Phase 5: Test infrastructure expansion (~2 days)

### Task 5.1: Property tests for `insert_knot` (geometric invariance)

**Files:**
- Create: `rust/nurbs/tests/algebra_proptest.rs`

- [ ] **Step 1: Write the proptest**

Create `rust/nurbs/tests/algebra_proptest.rs`:
```rust
//! Property-based tests for NURBS algebra primitives.
//! These exercise random inputs and check structural invariants.

#![cfg(feature = "host")]

use proptest::prelude::*;

fn arb_degree() -> impl Strategy<Value = u8> {
    1u8..=4
}

fn arb_simple_polynomial_curve() -> impl Strategy<Value = nurbs::ScalarNurbs<f64>> {
    arb_degree().prop_flat_map(|p| {
        let n = p as usize + 1;
        let cps = prop::collection::vec(-5.0..5.0_f64, n);
        cps.prop_map(move |cps_vec| {
            let pad = p as usize + 1;
            let mut knots = vec![0.0; pad];
            knots.extend(vec![1.0; pad]);
            nurbs::ScalarNurbs::try_new(p, knots, cps_vec, None).unwrap()
        })
    })
}

proptest! {
    #[test]
    fn insert_knot_preserves_evaluation(
        curve in arb_simple_polynomial_curve(),
        u in 0.01..0.99_f64,
    ) {
        let inserted = nurbs::knot::insert_knot(&curve, u, 1).unwrap();
        for sample_u in [0.0, 0.1, 0.3, 0.5, 0.7, 0.9, 1.0] {
            let before = nurbs::eval::eval(&curve.as_view(), sample_u);
            let after = nurbs::eval::eval(&inserted.as_view(), sample_u);
            prop_assert!(
                (before - after).abs() < 1e-9,
                "u={sample_u}: before={before}, after={after}"
            );
        }
    }
}
```

- [ ] **Step 2: Run**

Run: `cargo test -p nurbs --features host --test algebra_proptest`
Expected: PASS (~256 random cases).

- [ ] **Step 3: Commit**

```bash
git add rust/nurbs/tests/algebra_proptest.rs
git commit -m "nurbs/tests: proptest for insert_knot geometric invariance"
```

---

### Task 5.2: Property tests for `multiply` (degree formula + pointwise eval)

**Files:**
- Modify: `rust/nurbs/tests/algebra_proptest.rs`

- [ ] **Step 1: Append proptests**

Add to the `proptest!` block:
```rust
    #[test]
    fn multiply_degree_equals_sum(
        a in arb_simple_polynomial_curve(),
        b in arb_simple_polynomial_curve(),
    ) {
        let c = nurbs::algebra::multiply(&a, &b).unwrap();
        prop_assert_eq!(c.degree(), a.degree() + b.degree());
    }

    #[test]
    fn multiply_eval_matches_pointwise_product(
        a in arb_simple_polynomial_curve(),
        b in arb_simple_polynomial_curve(),
    ) {
        let c = nurbs::algebra::multiply(&a, &b).unwrap();
        for u in [0.0, 0.1, 0.3, 0.5, 0.7, 0.9, 1.0] {
            let exp = nurbs::eval::eval(&a.as_view(), u) * nurbs::eval::eval(&b.as_view(), u);
            let got = nurbs::eval::eval(&c.as_view(), u);
            prop_assert!(
                (exp - got).abs() < 1e-9,
                "u={u}: a*b={exp}, multiply={got}"
            );
        }
    }
```

- [ ] **Step 2: Run**

Run: `cargo test -p nurbs --features host --test algebra_proptest`
Expected: PASS.

- [ ] **Step 3: Commit**

```bash
git add rust/nurbs/tests/algebra_proptest.rs
git commit -m "nurbs/tests: proptest for multiply degree + pointwise eval"
```

---

### Task 5.3: Property tests for `convolve` (degree formula + support Minkowski sum)

**Files:**
- Modify: `rust/nurbs/tests/algebra_proptest.rs`

- [ ] **Step 1: Append proptests**

```rust
    fn arb_single_poly_kernel() -> impl Strategy<Value = nurbs::algebra::PiecewisePolynomialKernel<f64>> {
        (1usize..=4, 0.05..0.4_f64).prop_map(|(d, half)| {
            let coeffs: Vec<f64> = (0..=d).map(|i| (i as f64 + 1.0) * 0.5).collect();
            nurbs::algebra::PiecewisePolynomialKernel::single_poly(coeffs, (-half, half))
        })
    }

    #[test]
    fn convolve_degree_equals_input_plus_kernel_plus_one(
        curve in arb_simple_polynomial_curve(),
        kernel in arb_single_poly_kernel(),
    ) {
        let y = nurbs::algebra::convolve(&curve, &kernel).unwrap();
        let expected = curve.degree() as usize + kernel.pieces[0].degree() + 1;
        prop_assert_eq!(y.degree() as usize, expected);
    }

    #[test]
    fn convolve_support_is_minkowski_sum(
        curve in arb_simple_polynomial_curve(),
        kernel in arb_single_poly_kernel(),
    ) {
        let y = nurbs::algebra::convolve(&curve, &kernel).unwrap();
        let (k_lo, k_hi) = kernel.support();
        let expected_lo = curve.knots()[0] + k_lo;
        let expected_hi = curve.knots()[curve.knots().len() - 1] + k_hi;
        prop_assert!((y.knots()[0] - expected_lo).abs() < 1e-12);
        prop_assert!((y.knots()[y.knots().len() - 1] - expected_hi).abs() < 1e-12);
    }
```

- [ ] **Step 2: Run**

Run: `cargo test -p nurbs --features host --test algebra_proptest`
Expected: PASS.

- [ ] **Step 3: Commit**

```bash
git add rust/nurbs/tests/algebra_proptest.rs
git commit -m "nurbs/tests: proptest for convolve degree + Minkowski-sum support"
```

---

### Task 5.4: Expand sympy corpus — multiply edge cases

**Files:**
- Modify: `rust/nurbs/tests/scripts/generate_algebra_corpus.py`
- Modify: `rust/nurbs/tests/data/algebra_corpus.json` (regenerated)

- [ ] **Step 1: Add fixtures**

In `rust/nurbs/tests/scripts/generate_algebra_corpus.py`, add:
```python
def multiply_fixture_quadratic_x_linear():
    """a(u) = u^2, b(u) = u, expected c(u) = u^3."""
    a = quadratic_curve_data()
    b = linear_curve_data()
    samples_u = [0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0]
    samples = [{"u": u, "value": u**3} for u in samples_u]
    return {
        "name": "multiply_quadratic_x_linear",
        "operation": "multiply",
        "a": a,
        "b": b,
        "samples": samples,
    }


def multiply_fixture_with_interior_knot():
    """a has interior knot at 0.5; b is plain quadratic. Verify product."""
    a = {
        "degree": 2,
        "knots": [0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0],
        "control_points": [0.0, 1.0, 2.0, 3.0],
        "weights": None,
    }
    b = quadratic_curve_data()  # b(u) = u^2
    # Sympy: compute a(u) symbolically, then a(u) * u^2.
    from sympy import symbols, Piecewise, And, Rational, simplify
    u = symbols('u', real=True)
    # a(u) on [0, 0.5] and [0.5, 1] — use de Boor or exploit Bezier extraction.
    # For brevity, we compute samples numerically via the Rust ground truth:
    # since this fixture cross-checks Rust against itself for now, use ascending
    # sample interpolation. Better: use geomdl in a follow-up. For initial
    # bring-up, accept that this fixture validates structure, not closed-form.
    samples = []
    for u_val in [0.1, 0.3, 0.5, 0.7, 0.9]:
        # Compute a(u_val) using Bezier extraction logic (skip for v1; placeholder).
        # FOR INITIAL IMPLEMENTATION: skip this fixture from main() until a
        # geomdl-based or fully-symbolic ground truth is wired.
        pass
    return None  # not yet wired — placeholder
```

Update `main()` to skip `None` fixtures:
```python
def main():
    fixtures = [f for f in [
        multiply_fixture_linear_x_linear(),
        multiply_fixture_quadratic_x_linear(),
        convolve_fixture_constant_x_constant(),
    ] if f is not None]
    print(json.dumps({"fixtures": fixtures}, indent=2))
```

- [ ] **Step 2: Regenerate corpus**

```bash
python3 rust/nurbs/tests/scripts/generate_algebra_corpus.py > rust/nurbs/tests/data/algebra_corpus.json
```

- [ ] **Step 3: Run oracle test**

Run: `cargo test -p nurbs --features host --test algebra_oracle`
Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add rust/nurbs/tests/scripts/generate_algebra_corpus.py rust/nurbs/tests/data/algebra_corpus.json
git commit -m "nurbs/tests: expand sympy corpus with quadratic*linear multiply"
```

---

### Task 5.5: Expand sympy corpus — convolve with smooth_zv kernel

**Files:**
- Modify: `rust/nurbs/tests/scripts/generate_algebra_corpus.py`
- Modify: `rust/nurbs/tests/data/algebra_corpus.json`

- [ ] **Step 1: Add fixture using bleeding-edge-v2 smooth_zv kernel**

In `generate_algebra_corpus.py`, add:
```python
def convolve_fixture_smooth_zv_x_linear():
    """x(s) = s on [0, 1], w = bleeding-edge-v2 smooth_zv kernel.
    Verifies our convolve matches the analytical reference."""
    from sympy import symbols, integrate, Piecewise, Rational, And

    # smooth_zv coefficients (from bleeding-edge-v2 shaper_defs.py):
    # init_smoother normalizes them; here we use the post-normalization form
    # for shaper_freq=1 → t_sm = 0.8025.
    # Coeffs (low-to-high in t) for the normalized polynomial on [-t_sm/2, +t_sm/2]:
    # We pre-normalize for unit smooth time; the actual values come from the
    # init_smoother formula.
    raw_coeffs = [-118.4265334338076, 5.861885495127615, 29.52796003014231,
                  -1.465471373781904, 0.01966833207740377]
    smooth_time = 0.8025  # for shaper_freq = 1
    inv_t = 1.0 / smooth_time
    inv_t_n = inv_t
    n = len(raw_coeffs)
    c = [0.0] * n
    for i in range(n - 1, -1, -1):
        c[n - i - 1] = raw_coeffs[i] * inv_t_n
        inv_t_n *= inv_t
    # Now `c` is in DESCENDING order? Check init_smoother — the comment says
    # ASCENDING. Verify against shaper_defs.py.
    # init_smoother returns (list(reversed(coeffs)), smooth_time) with
    # normalization, then reverses → so the returned list is ASCENDING in t.
    # Thus c (above) is now ASCENDING.

    s, u = symbols('s u', real=True)
    # x(s) = s on [0, 1].
    x_sym = Piecewise((s, And(s >= 0, s <= 1)), (0, True))
    # w(t) = Σ c_i * t^i on [-T/2, T/2], else 0.
    half = smooth_time / 2
    t = symbols('t', real=True)
    w_poly = sum(c[i] * t**i for i in range(n))
    # Sample y(u) = ∫ x(s) * w(u - s) ds.
    samples = []
    for u_val in [0.0, 0.5, 1.0]:
        # Substitute t = u_val - s, integrate in s over the overlap interval.
        s_lo = max(0, u_val - half)
        s_hi = min(1, u_val + half)
        if s_lo >= s_hi:
            samples.append({"u": u_val, "value": 0.0})
            continue
        integrand = s * w_poly.subs(t, u_val - s)
        y_val = float(integrate(integrand, (s, s_lo, s_hi)))
        samples.append({"u": u_val, "value": y_val})

    kernel = {
        "pieces": [
            {"u_start": -half, "u_end": half, "coeffs": c},
        ],
    }
    return {
        "name": "convolve_smooth_zv_x_linear",
        "operation": "convolve",
        "curve": linear_curve_data(),
        "kernel": kernel,
        "samples": samples,
    }
```

Add `convolve_fixture_smooth_zv_x_linear()` to `main()`.

- [ ] **Step 2: Regenerate and run**

```bash
python3 rust/nurbs/tests/scripts/generate_algebra_corpus.py > rust/nurbs/tests/data/algebra_corpus.json
cargo test -p nurbs --features host --test algebra_oracle
```
Expected: PASS for the new fixture.

- [ ] **Step 3: Commit**

```bash
git add rust/nurbs/tests/scripts/generate_algebra_corpus.py rust/nurbs/tests/data/algebra_corpus.json
git commit -m "nurbs/tests: sympy corpus — convolve with bleeding-edge-v2 smooth_zv kernel"
```

---

### Task 5.6: Klipper cross-check integration test

**Files:**
- Create: `rust/tests/klipper_convolve_crosscheck.rs`
- Modify: `rust/Cargo.toml` (add as integration test if not auto-discovered)

- [ ] **Step 1: Write a test that calls Klipper's runtime convolution via FFI or a separate Python script**

Since Klipper's runtime convolution lives in `klippy/chelper/integrate.c` and is C-based, the cleanest cross-check is:
1. Write a Python harness that calls Klipper's existing input_shaper smoother on a synthetic trajectory.
2. Dump the shaped trajectory to JSON.
3. Have the Rust test load that JSON and compare against `convolve` output.

Create `rust/tests/scripts/generate_klipper_reference.py`:
```python
#!/usr/bin/env python3
"""Generate Klipper-shaped trajectory reference for convolve cross-check.

Loads the existing klippy shaper machinery and applies smooth_zv to a simple
input trajectory, dumping samples to JSON for the Rust oracle.
"""

import json
import sys
sys.path.insert(0, 'klippy/extras')
from shaper_defs import get_zv_smoother, init_smoother

def main():
    # Single accel phase: x(t) = 0.5 * a * t^2 for t in [0, 0.5], a = 100.
    # Input is parameterized in time directly. Use shaper_freq = 30.
    coeffs, t_sm = get_zv_smoother(30.0)
    # Compute Klipper's shaped position by evaluating the convolution kernel
    # against the input. This requires reproducing Klipper's smoother application:
    # x_sm(T) = Σ_i C_i * I_i(T), where I_i are time-integrals of x(t) against
    # t^i kernel components. See klippy/chelper/integrate.c.
    #
    # For initial bring-up: numerically integrate using sympy or scipy.
    import scipy.integrate as si

    a = 100.0
    def x_input(t):
        return 0.5 * a * t**2 if 0 <= t <= 0.5 else 0.0

    def kernel(t):
        # Polynomial w(t) = Σ coeffs[i] * t^i on [-t_sm/2, t_sm/2], else 0.
        if abs(t) > t_sm / 2:
            return 0.0
        return sum(c * t**i for i, c in enumerate(coeffs))

    samples = []
    for T in [0.05, 0.1, 0.2, 0.3, 0.4, 0.45]:
        # y(T) = ∫ x(s) w(T - s) ds over s ∈ [T - t_sm/2, T + t_sm/2] ∩ [0, 0.5]
        s_lo = max(0, T - t_sm / 2)
        s_hi = min(0.5, T + t_sm / 2)
        if s_lo >= s_hi:
            samples.append({"T": T, "value": 0.0})
            continue
        y, _ = si.quad(lambda s: x_input(s) * kernel(T - s), s_lo, s_hi)
        samples.append({"T": T, "value": y})

    out = {
        "kernel_coeffs": list(coeffs),
        "kernel_t_sm": t_sm,
        "input_accel": a,
        "input_t_end": 0.5,
        "samples": samples,
    }
    print(json.dumps(out, indent=2))


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Generate the reference**

```bash
mkdir -p rust/tests/data rust/tests/scripts
mv rust/tests/scripts/generate_klipper_reference.py rust/tests/scripts/  # if needed
python3 rust/tests/scripts/generate_klipper_reference.py > rust/tests/data/klipper_smooth_zv_reference.json
```

- [ ] **Step 3: Write the Rust integration test**

Create `rust/tests/klipper_convolve_crosscheck.rs`:
```rust
//! Cross-check our convolve against scipy/Klipper-derived numerical reference.
//! Reference file: `rust/tests/data/klipper_smooth_zv_reference.json`.

use serde_json::Value;
use std::fs;
use std::path::PathBuf;

const TOLERANCE: f64 = 1e-4;  // numerical-quadrature reference, not exact

#[test]
fn convolve_matches_scipy_reference_for_smooth_zv_kernel() {
    let path: PathBuf = [
        env!("CARGO_MANIFEST_DIR"), "tests", "data", "klipper_smooth_zv_reference.json",
    ].iter().collect();
    let raw = fs::read_to_string(&path).expect("reference file must exist");
    let v: Value = serde_json::from_str(&raw).expect("valid JSON");

    let kernel_coeffs: Vec<f64> = v["kernel_coeffs"].as_array().unwrap()
        .iter().map(|c| c.as_f64().unwrap()).collect();
    let t_sm = v["kernel_t_sm"].as_f64().unwrap();
    let accel = v["input_accel"].as_f64().unwrap();
    let t_end = v["input_t_end"].as_f64().unwrap();

    // Build the input as a quadratic NURBS: x(t) = 0.5 * a * t^2 on [0, t_end].
    // Bezier control points for 0.5 * a * t^2 on [0, t_end] in degree-2 Bernstein:
    // a_b * 1 + b_b * 2t(1-t)/t_end + c_b * (t/t_end)^2 with proper scaling — easier to
    // construct in monomial form and convert.
    let mono = nurbs::BezierPiece::<f64> {
        u_start: 0.0,
        u_end: t_end,
        coeffs: vec![0.0, 0.0, 0.5 * accel],
    };
    let bernstein = mono.to_bernstein();
    let curve = nurbs::ScalarNurbs::try_new(
        2,
        vec![0.0, 0.0, 0.0, t_end, t_end, t_end],
        bernstein,
        None,
    ).unwrap();

    let kernel = nurbs::algebra::PiecewisePolynomialKernel::single_poly(
        kernel_coeffs, (-t_sm / 2.0, t_sm / 2.0),
    );
    let y = nurbs::algebra::convolve(&curve, &kernel).unwrap();

    for sample in v["samples"].as_array().unwrap() {
        let t = sample["T"].as_f64().unwrap();
        let expected = sample["value"].as_f64().unwrap();
        let got = nurbs::eval::eval(&y.as_view(), t);
        let diff = (got - expected).abs();
        assert!(
            diff < TOLERANCE,
            "T={t}: got {got}, expected {expected} (diff {diff})"
        );
    }
}
```

- [ ] **Step 4: Run**

Run: `cargo test --test klipper_convolve_crosscheck`
Expected: PASS within `1e-4` of scipy's quadrature reference.

- [ ] **Step 5: Commit**

```bash
git add rust/tests/scripts/generate_klipper_reference.py rust/tests/data/klipper_smooth_zv_reference.json rust/tests/klipper_convolve_crosscheck.rs
git commit -m "nurbs/tests: Klipper cross-check — convolve vs scipy quadrature reference"
```

---

### Task 5.7: Final clippy / fmt / full-test pass

- [ ] **Step 1: Run clippy under host**

Run: `cargo clippy -p nurbs --features host -- -D warnings`
Expected: Clean. Fix any warnings inline.

- [ ] **Step 2: Run clippy under MCU configurations**

Run: `cargo clippy -p nurbs --no-default-features --features mcu-h7 -- -D warnings`
Run: `cargo clippy -p nurbs --no-default-features --features mcu-f4 -- -D warnings`
Expected: Clean for both.

- [ ] **Step 3: Run cargo fmt**

Run: `cargo fmt --all`
Then: `git diff --stat`

- [ ] **Step 4: Run full test suite**

Run: `cargo test -p nurbs --features host`
Run: `cargo test --test klipper_convolve_crosscheck`
Expected: All pass.

- [ ] **Step 5: Final commit if anything moved**

```bash
git add -A
git commit -m "nurbs: clippy + fmt cleanups for algebra phase"
```

---

## Implementation complete

All five phases done. Layer 0 algebra is unblocked for Layer 3 smooth-shaper bake (build step 8 in CLAUDE.md).

**Verification checklist:**
- [ ] `cargo test -p nurbs --features host` passes (unit + proptest + sympy oracle)
- [ ] `cargo test --test klipper_convolve_crosscheck` passes
- [ ] `cargo clippy` clean under host, mcu-h7, mcu-f4
- [ ] No `todo!()` or `unimplemented!()` in algebra/knot/bezier modules
- [ ] Public API matches spec section 5
- [ ] All five build-order steps each landed as a green commit set
