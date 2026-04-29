# Step 7-pre: Cubic-Bézier-only live pipeline prep — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Land the Layer-0 / Layer-1 prep work that Step 7-A's T-A time-reparameterization sits on: two new Layer-0 primitives (`compose_vector_piece`, `fit_x_to_arc_length_piece`), a small Layer-0 helper (`xy_arc_length`), one Layer-1 primitive (`split_segment_to_cap`), and a Layer-1 structural refactor (FittedSegment → CubicSegment with explicit E-mode classification, retire `Segment::Arc`, drop G0/G1/G2/G3 from live reduce).

**Architecture:** Live pipeline becomes G5/G5.1-only with uniform cubic-Bézier-polynomial-NURBS internal representation. Layer 0 gains polynomial-of-polynomial composition + adaptive-degree per-piece x(s) fit + scalar XY arc-length query. Layer 1 gains a path-length cap via knot-insertion-based subdivision. Existing G0/G1/G2/G3 reduction code is preserved behind a `legacy-reference` feature flag for the future Step-13 compat layer.

**Tech Stack:** Rust 2024 edition; `nurbs` and `geometry` workspace crates; existing `BezierPiece` Pascal-shifted-monomial-basis machinery; `arc_length::ArcLengthTable` Gauss-Legendre infrastructure; cargo + standard test runner; `cargo clippy --all-targets -- -D warnings` clean.

**Spec:** [`docs/superpowers/specs/2026-04-29-step7-pre-cubic-pipeline-prep-design.md`](../specs/2026-04-29-step7-pre-cubic-pipeline-prep-design.md) (7-round adversarial-review converged).

---

## File Structure

**Layer 0 — `rust/nurbs/`:**
- Modify: `rust/nurbs/src/algebra.rs` — add `compose_vector_piece`, `fit_x_to_arc_length_piece`, `FitError`. Remove `RationalNotSupported` workaround text in `convolve` doc-comment (no longer relevant).
- Modify: `rust/nurbs/src/arc_length.rs` — add `xy_arc_length` scalar one-shot query.
- Create: `rust/nurbs/tests/compose_vector_piece.rs` — composition tests.
- Create: `rust/nurbs/tests/fit_x_to_arc_length_piece.rs` — fit tests.
- Create: `rust/nurbs/tests/xy_arc_length.rs` — XY-arc-length tests.

**Layer 1 — `rust/geometry/`:**
- Modify: `rust/geometry/Cargo.toml` — add `legacy-reference` feature flag.
- Modify: `rust/geometry/src/lib.rs` — re-export `CubicSegment`, `EMode`, `SplitInfo`, `GeometryError`, `split_segment_to_cap`.
- Modify: `rust/geometry/src/segment.rs` — add `CubicSegment`, `EMode`, `SplitInfo`; deprecate `FittedSegment` and `ArcSegment` behind `cfg(feature = "legacy-reference")`; update `Segment` enum.
- Create: `rust/geometry/src/error.rs` — `GeometryError` enum (or extend existing).
- Modify: `rust/geometry/src/reduce.rs` — gate G0/G1/G2/G3 paths behind `cfg(feature = "legacy-reference")`; emit `Cubic` for G5/G5.1.
- Modify: `rust/geometry/src/pipeline.rs` — emit `Segment::Cubic`, classification logic, helical-rejection.
- Create: `rust/geometry/src/splitter.rs` — `split_segment_to_cap` primitive.
- Create: `rust/geometry/tests/cubic_segment.rs` — invariant + classification tests.
- Create: `rust/geometry/tests/split_segment_to_cap.rs` — splitter tests.
- Create: `rust/geometry/tests/integration_g5_only.rs` — end-to-end G5 → reduce → split → Layer 2 sanity.
- Modify: `rust/geometry/tests/g5_reduction.rs`, `rust/temporal/tests/multi_segment.rs` — gate or rebuild legacy fixtures.

**Workspace level:**
- Modify: `rust/Cargo.toml` — no member changes; `legacy-reference` is a per-crate feature.

---

## Phase 1 — Layer-1 structural foundation

The four sub-tasks in Phase 1 must land in one PR (per spec §8). The live pipeline must be self-consistent at the end of Phase 1: G5/G5.1 in → `Segment::Cubic` out, G0/G1/G2/G3 → error, no `Segment::Arc` references anywhere live.

### Task 1.1: Add `EMode`, `SplitInfo` types to segment.rs

**Files:**
- Modify: `rust/geometry/src/segment.rs`
- Modify: `rust/geometry/src/lib.rs` (re-export)

- [ ] **Step 1: Open the file and add the new types after the `BlendFamily` enum** (`rust/geometry/src/segment.rs`):

```rust
/// E-axis classification per CLAUDE.md feature scope. `CubicSegment::try_new`
/// applies the §6.1 classification rules to derive this from raw `(ΔX, ΔY, ΔZ, ΔE)`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EMode {
    /// Extrusion proportional to actual XY shaped motion: `e_actual(t) = ratio × ∫|v_xy| dt`.
    /// `extrusion_per_xy_mm` is nonzero and signed (positive for normal extrusion;
    /// negative for retract-during-XY-motion / wipe / coast). Used for moves with
    /// `ΔXY > ε_xyz`, `ΔZ ≤ ε_z`, and `abs(ΔE) > ε_e`.
    CoupledToXy,
    /// Travel move: XY motion with no extrusion. Equivalent to `CoupledToXy` with
    /// `extrusion_per_xy_mm = 0`. Modeled distinctly for clarity in logs/telemetry
    /// and to allow a future plan layer to skip per-sample E integration when the
    /// ratio is definitionally zero.
    Travel,
    /// E motion not coupled to XY: own E NURBS carries the trajectory in time.
    /// In 7-pre's live pipeline, `Independent` always implies null `xyz` motion
    /// (cp_polygon_length and midpoint parametric speed both below thresholds).
    /// Helical extrusion (XYZ + E) is rejected upstream; never produces `Independent`
    /// in the live pipeline.
    Independent,
}

/// Sub-segment provenance, populated by `split_segment_to_cap` (geometry::splitter).
/// `None` when the segment was not split.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SplitInfo {
    /// 0-indexed position of this child within the parent's sub-segment sequence.
    pub sub_index: u32,
    /// Total sub-segments produced from the parent. May be < the originally-planned
    /// `k` if epsilon-filtering at splitter step 6 dropped near-boundary breakpoints.
    pub sub_count: u32,
    /// Arc-length range this sub-segment occupies in the parent's arc-length domain.
    /// Computed at split time by querying the parent's arc-length table at the child's
    /// `xyz.u_start` and `xyz.u_end`.
    pub s_lo_mm: f64,
    pub s_hi_mm: f64,
}
```

- [ ] **Step 2: Re-export from lib.rs** (`rust/geometry/src/lib.rs`):

Find the existing `pub use segment::{...};` and extend:
```rust
pub use segment::{
    ArcSegment, BlendFamily, CornerBlendSlot, CubicSegment, EMode, FittedSegment, /* ← will fix in 1.6 */
    JunctionDeviation, Segment, SourceRange, SplitInfo,
};
```

(`CubicSegment` doesn't exist yet — we're adding it in Task 1.2. This step is a no-op for now; the `pub use` line will be updated in Task 1.2.)

- [ ] **Step 3: Verify compile**

```bash
cd /Users/daniladergachev/Developer/kalico
cargo check -p geometry
```

Expected: clean (the new types are independent additions).

- [ ] **Step 4: Commit**

```bash
git add rust/geometry/src/segment.rs rust/geometry/src/lib.rs
git commit -m "geometry/segment: add EMode and SplitInfo types"
```

### Task 1.2: Add `CubicSegment` struct and constructor invariants

**Files:**
- Modify: `rust/geometry/src/segment.rs`
- Create: `rust/geometry/src/error.rs`
- Modify: `rust/geometry/src/lib.rs`

- [ ] **Step 1: Create `rust/geometry/src/error.rs`** with the geometry-error enum:

```rust
//! Geometry-layer errors. Layer-1 reduce + segment-construction surface these
//! to the pipeline, which converts them to telemetry events / fatal-segment
//! markers per the existing `Fatal` / `Recovery` machinery.

use thiserror::Error;

#[derive(Debug, Clone, Copy, PartialEq, Error)]
pub enum GeometryError {
    /// Live pipeline received G0/G1/G2/G3; user must run input through the
    /// Step-13 compat layer first to normalize to G5/G5.1.
    #[error(
        "live pipeline accepts G5/G5.1 only; received G{0:?}. \
         Run input through Step-13 compat layer or use a kalico-aware slicer."
    )]
    UnsupportedGcode(u8),

    /// Helical extrusion (XY motion + Z motion + E motion in same segment)
    /// is not yet supported in the live pipeline; pre-process via the Step-13
    /// compat layer or disable helical extrusion in the slicer.
    #[error(
        "helical extrusion (XY motion + Z motion + E motion in same segment) \
         not yet supported in live pipeline; pre-process via Step-13 compat \
         layer or disable helical extrusion in slicer"
    )]
    HelicalExtrusionUnsupported,

    /// `xyz` did not satisfy the single-piece-cubic-Bézier invariant
    /// (degree != 3, != 4 control points, has weights, or knot vector not clamped).
    #[error("xyz NURBS is not a single-piece cubic Bézier: {reason}")]
    NotSinglePieceCubic { reason: &'static str },

    /// E-mode/E-fields invariant violated.
    #[error("EMode invariant violated: {reason}")]
    EModeInvariantViolation { reason: &'static str },

    /// Zero-motion segment (all of ΔXY, ΔZ, ΔE below thresholds). Caller should
    /// drop this segment without emitting it.
    #[error("zero-motion segment rejected")]
    ZeroMotion,
}
```

- [ ] **Step 2: Wire into `lib.rs`** (`rust/geometry/src/lib.rs`):

```rust
pub mod error;
pub use error::GeometryError;
```

- [ ] **Step 3: Add `CubicSegment` to `segment.rs`** (after `JunctionDeviation`):

```rust
/// Live-pipeline cubic-Bézier segment. Single-piece cubic Bézier in `xyz` (degree 3,
/// 4 control points, no weights, clamped knot vector). E classification per `EMode`.
#[derive(Debug, Clone, PartialEq)]
pub struct CubicSegment {
    /// XYZ trajectory in u-domain. **Invariant** (enforced by `try_new`): single-piece
    /// cubic Bézier — degree 3, 4 control points, no weights, clamped knot vector.
    pub xyz: VectorNurbs<f64, 3>,
    pub e_mode: EMode,
    /// Valid when `e_mode == CoupledToXy`. Signed: negative for retract-during-XY-motion
    /// / wipe / coast. Zero when `e_mode == Travel`. Unused when `e_mode == Independent`
    /// (use `e_independent` instead).
    pub extrusion_per_xy_mm: f64,
    /// `Some(curve)` iff `e_mode == Independent`; carries the E trajectory for
    /// retraction / prime / filament-change segments.
    pub e_independent: Option<ScalarNurbs<f64>>,
    pub feedrate_mm_s: f64,
    pub source: SourceRange,
    /// `None` on un-split segments; `Some` on splitter output.
    pub split_info: Option<SplitInfo>,
}

impl CubicSegment {
    /// Construct a CubicSegment, validating invariants. Returns `Err` on:
    /// - `NotSinglePieceCubic`: xyz is not single-piece cubic (degree != 3,
    ///   != 4 CPs, has weights, or knots are not clamped `[0,0,0,0,1,1,1,1]`).
    /// - `EModeInvariantViolation`: `e_mode` and the corresponding fields disagree.
    pub fn try_new(
        xyz: VectorNurbs<f64, 3>,
        e_mode: EMode,
        extrusion_per_xy_mm: f64,
        e_independent: Option<ScalarNurbs<f64>>,
        feedrate_mm_s: f64,
        source: SourceRange,
        split_info: Option<SplitInfo>,
    ) -> Result<Self, crate::GeometryError> {
        // xyz must be single-piece cubic Bézier.
        if xyz.degree() != 3 {
            return Err(crate::GeometryError::NotSinglePieceCubic {
                reason: "degree != 3",
            });
        }
        if xyz.control_points().len() != 4 {
            return Err(crate::GeometryError::NotSinglePieceCubic {
                reason: "control_points.len() != 4",
            });
        }
        if xyz.weights().is_some() {
            return Err(crate::GeometryError::NotSinglePieceCubic {
                reason: "weights present (must be polynomial)",
            });
        }
        let expected_knots: [f64; 8] = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
        if xyz.knots() != expected_knots.as_slice() {
            return Err(crate::GeometryError::NotSinglePieceCubic {
                reason: "knot vector not clamped [0,0,0,0,1,1,1,1]",
            });
        }

        // EMode invariants.
        match e_mode {
            EMode::CoupledToXy => {
                if e_independent.is_some() {
                    return Err(crate::GeometryError::EModeInvariantViolation {
                        reason: "CoupledToXy must have e_independent: None",
                    });
                }
                // extrusion_per_xy_mm: any value (signed); zero is Travel territory but
                // the choice is left to the classifier; `try_new` only validates structure.
            }
            EMode::Travel => {
                if extrusion_per_xy_mm != 0.0 {
                    return Err(crate::GeometryError::EModeInvariantViolation {
                        reason: "Travel must have extrusion_per_xy_mm == 0.0",
                    });
                }
                if e_independent.is_some() {
                    return Err(crate::GeometryError::EModeInvariantViolation {
                        reason: "Travel must have e_independent: None",
                    });
                }
            }
            EMode::Independent => {
                if e_independent.is_none() {
                    return Err(crate::GeometryError::EModeInvariantViolation {
                        reason: "Independent must have e_independent: Some(_)",
                    });
                }
                if extrusion_per_xy_mm != 0.0 {
                    return Err(crate::GeometryError::EModeInvariantViolation {
                        reason: "Independent must have extrusion_per_xy_mm == 0.0",
                    });
                }
            }
        }

        Ok(Self {
            xyz,
            e_mode,
            extrusion_per_xy_mm,
            e_independent,
            feedrate_mm_s,
            source,
            split_info,
        })
    }
}
```

- [ ] **Step 4: Add `thiserror` to geometry's Cargo.toml** (if not already present):

Check `rust/geometry/Cargo.toml`. If `thiserror` is not in `[dependencies]`, add:
```toml
thiserror = { workspace = true }
```

- [ ] **Step 5: Update `lib.rs` re-exports**:

```rust
pub use segment::{
    BlendFamily, CornerBlendSlot, CubicSegment, EMode, JunctionDeviation, Segment,
    SourceRange, SplitInfo,
};
```

(Note: `FittedSegment` and `ArcSegment` re-exports will be gated in Task 1.6.)

- [ ] **Step 6: Verify compile**

```bash
cargo check -p geometry
```

Expected: clean. Some downstream warnings about the still-existing `FittedSegment`/`ArcSegment` are fine — those are removed in Task 1.6.

- [ ] **Step 7: Write the invariant test**

Create `rust/geometry/tests/cubic_segment.rs`:

```rust
use geometry::{CubicSegment, EMode, GeometryError, SourceRange};
use nurbs::VectorNurbs;

fn valid_cubic_xyz() -> VectorNurbs<f64, 3> {
    VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [3.0, 0.0, 0.0]],
        None,
    )
    .expect("valid cubic")
}

fn dummy_source() -> SourceRange {
    SourceRange { start_line: 1, end_line: 1 }
}

#[test]
fn try_new_rejects_non_cubic() {
    let linear = VectorNurbs::<f64, 3>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
        None,
    )
    .expect("valid linear");
    let result = CubicSegment::try_new(
        linear,
        EMode::Travel,
        0.0,
        None,
        100.0,
        dummy_source(),
        None,
    );
    assert!(matches!(result, Err(GeometryError::NotSinglePieceCubic { .. })));
}

#[test]
fn try_new_rejects_weighted() {
    let weighted = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [3.0, 0.0, 0.0]],
        Some(vec![1.0, 0.5, 0.5, 1.0]),
    )
    .expect("valid weighted cubic");
    let result = CubicSegment::try_new(
        weighted,
        EMode::Travel,
        0.0,
        None,
        100.0,
        dummy_source(),
        None,
    );
    assert!(matches!(result, Err(GeometryError::NotSinglePieceCubic { .. })));
}

#[test]
fn try_new_accepts_valid_travel() {
    let result = CubicSegment::try_new(
        valid_cubic_xyz(),
        EMode::Travel,
        0.0,
        None,
        100.0,
        dummy_source(),
        None,
    );
    assert!(result.is_ok());
}

#[test]
fn try_new_accepts_coupled_signed_ratio() {
    // Negative ratio = retract-during-XY-motion / wipe / coast.
    let result = CubicSegment::try_new(
        valid_cubic_xyz(),
        EMode::CoupledToXy,
        -0.05,
        None,
        100.0,
        dummy_source(),
        None,
    );
    assert!(result.is_ok());
}

#[test]
fn try_new_rejects_travel_with_nonzero_ratio() {
    let result = CubicSegment::try_new(
        valid_cubic_xyz(),
        EMode::Travel,
        0.05,
        None,
        100.0,
        dummy_source(),
        None,
    );
    assert!(matches!(result, Err(GeometryError::EModeInvariantViolation { .. })));
}

#[test]
fn try_new_rejects_independent_without_e_curve() {
    let result = CubicSegment::try_new(
        valid_cubic_xyz(),
        EMode::Independent,
        0.0,
        None,
        100.0,
        dummy_source(),
        None,
    );
    assert!(matches!(result, Err(GeometryError::EModeInvariantViolation { .. })));
}
```

- [ ] **Step 8: Run the test**

```bash
cargo test -p geometry --test cubic_segment
```

Expected: all 6 tests pass.

- [ ] **Step 9: Commit**

```bash
git add rust/geometry/src/error.rs rust/geometry/src/segment.rs rust/geometry/src/lib.rs \
        rust/geometry/Cargo.toml rust/geometry/tests/cubic_segment.rs
git commit -m "geometry: add CubicSegment with single-piece-cubic + EMode invariants"
```

### Task 1.3: Add `legacy-reference` feature flag to geometry crate

**Files:**
- Modify: `rust/geometry/Cargo.toml`

- [ ] **Step 1: Add the feature flag**

Find the `[features]` block (or add one) in `rust/geometry/Cargo.toml`:

```toml
[features]
default = []
# Gates legacy G0/G1/G2/G3 reduction code + ArcSegment/FittedSegment types.
# Enabled by tests that exercise the pre-compat-layer reduction paths and
# eventually by the Step-13 compat-layer crate.
legacy-reference = []
```

- [ ] **Step 2: Verify**

```bash
cargo check -p geometry
cargo check -p geometry --features legacy-reference
```

Both should be clean (no code changes yet — feature is just declared).

- [ ] **Step 3: Commit**

```bash
git add rust/geometry/Cargo.toml
git commit -m "geometry: declare legacy-reference feature flag"
```

### Task 1.4: Update `Segment` enum, gate legacy variants behind feature

**Files:**
- Modify: `rust/geometry/src/segment.rs`
- Modify: `rust/geometry/src/lib.rs`

- [ ] **Step 1: Edit the `Segment` enum** in `rust/geometry/src/segment.rs`:

```rust
#[non_exhaustive]
#[derive(Debug, Clone, PartialEq)]
pub enum Segment {
    /// Live-pipeline cubic Bézier segment with E-mode classification. Produced by
    /// reduce.rs from G5/G5.1 input.
    Cubic(CubicSegment),
    CornerBlend(CornerBlendSlot),
    Junction(JunctionDeviation),

    /// **Legacy.** Multi-degree polynomial NURBS segment from the pre-G5-only reduce
    /// stage. Step-13 compat-layer territory; not produced in the live pipeline.
    #[cfg(feature = "legacy-reference")]
    Fitted(FittedSegment),

    /// **Legacy.** Rational quadratic NURBS segment from G2/G3 reduction. Step-13
    /// compat-layer territory; not produced in the live pipeline.
    #[cfg(feature = "legacy-reference")]
    Arc(ArcSegment),
}
```

- [ ] **Step 2: Gate `FittedSegment` and `ArcSegment` definitions**:

Wrap their `struct` definitions:

```rust
#[cfg(feature = "legacy-reference")]
#[derive(Debug, Clone, PartialEq)]
pub struct FittedSegment {
    // ... unchanged fields ...
}

#[cfg(feature = "legacy-reference")]
#[derive(Debug, Clone, PartialEq)]
pub struct ArcSegment {
    // ... unchanged fields ...
}
```

- [ ] **Step 3: Update `lib.rs` re-exports to gate legacy types**:

```rust
pub use segment::{
    BlendFamily, CornerBlendSlot, CubicSegment, EMode, JunctionDeviation, Segment,
    SourceRange, SplitInfo,
};

#[cfg(feature = "legacy-reference")]
pub use segment::{ArcSegment, FittedSegment};
```

- [ ] **Step 4: Update existing tests in `segment.rs`**:

Find the `#[cfg(test)] mod tests` block at the end. Wrap any `FittedSegment` / `ArcSegment` usage in `#[cfg(feature = "legacy-reference")]`:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use nurbs::VectorNurbs;

    #[test]
    #[cfg(feature = "legacy-reference")]
    fn segment_variants_construct() {
        // ... existing test body, unchanged
    }

    #[test]
    fn cubic_variant_constructs() {
        // Quick sanity test that doesn't need legacy-reference.
        let xyz = VectorNurbs::<f64, 3>::try_new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [3.0, 0.0, 0.0]],
            None,
        ).expect("valid");
        let cs = CubicSegment::try_new(
            xyz, EMode::Travel, 0.0, None, 100.0,
            SourceRange { start_line: 1, end_line: 1 }, None,
        ).expect("valid travel");
        let seg: Segment = Segment::Cubic(cs);
        assert!(matches!(seg, Segment::Cubic(_)));
    }
}
```

- [ ] **Step 5: Verify both feature configurations compile**

```bash
cargo check -p geometry
cargo check -p geometry --features legacy-reference
cargo test -p geometry --test cubic_segment
cargo test -p geometry segment::tests::cubic_variant_constructs
cargo test -p geometry --features legacy-reference segment::tests::segment_variants_construct
```

All should pass / be clean. The `geometry` crate's downstream consumers (`pipeline.rs`, `reduce.rs`) will fail to compile if they reference `FittedSegment` / `ArcSegment` outside of legacy-reference cfg — that's expected and gets fixed in subsequent tasks.

- [ ] **Step 6: Commit**

(Don't worry about pipeline.rs / reduce.rs compile errors yet — we'll fix in Task 1.5 onwards.)

```bash
git add rust/geometry/src/segment.rs rust/geometry/src/lib.rs
git commit -m "geometry/segment: gate FittedSegment/ArcSegment behind legacy-reference"
```

### Task 1.5: Gate legacy reduce.rs paths behind feature

**Files:**
- Modify: `rust/geometry/src/reduce.rs`

- [ ] **Step 1: Read the existing reduce.rs to find the dispatch surface**

```bash
grep -n "fn reduce\|enum CurveGeom\|Linear\|Quadratic\|RationalQuadratic\|Cubic" rust/geometry/src/reduce.rs | head -30
```

The existing `CurveGeom` enum carries variants for each gcode primitive. `Cubic` (G5) and `Quadratic` (G5.1) stay live; `Linear` (G1), `RationalQuadratic` (G2/G3) are gated.

- [ ] **Step 2: Identify the variants to gate**

In the `CurveGeom` enum (and any pattern-matches against it), wrap the legacy variants with `#[cfg(feature = "legacy-reference")]`. **The exact variant names depend on the existing enum** — read the file. As a guide:

```rust
pub enum CurveGeom {
    Cubic(/* G5 fields */),
    Quadratic(/* G5.1 fields */),

    #[cfg(feature = "legacy-reference")]
    Linear(/* G0/G1 fields */),

    #[cfg(feature = "legacy-reference")]
    RationalQuadratic(/* G2/G3 fields */),
}
```

- [ ] **Step 3: Gate the corresponding reduction handlers in the `reduce()` function**

For each pattern arm that matches a Linear/RationalQuadratic input or that produces a Linear/RationalQuadratic CurveGeom variant, wrap with `#[cfg(feature = "legacy-reference")]`. For the **non-cfg** path, replace with an explicit error emission:

```rust
match token {
    Token::G0 { .. } | Token::G1 { .. } => {
        // Live pipeline: error out.
        // (Specific shape depends on existing reduce-error machinery.
        //  The intent: emit a `ReduceEvent::ParseError(ParseErrorKind::UnsupportedGcode)`
        //  with the offending kind label.)
        emit_unsupported_gcode_error(token, "G0/G1");
    }
    Token::G2 { .. } | Token::G3 { .. } => {
        emit_unsupported_gcode_error(token, "G2/G3");
    }
    Token::G5 { .. } => { /* unchanged G5 handling */ }
    Token::G5_1 { .. } => { /* unchanged G5.1 handling */ }
    // ... other non-motion tokens unchanged ...
}
```

The exact wording depends on the existing `ParseErrorKind` enum. Add a new variant `UnsupportedGcode { kind: &'static str }` if needed.

- [ ] **Step 4: Verify both configurations compile**

```bash
cargo check -p geometry
cargo check -p geometry --features legacy-reference
```

Both should be clean. Existing legacy tests (in `tests/`) will fail without `--features legacy-reference` — that's fixed in Task 4.1.

- [ ] **Step 5: Add a test for the live-pipeline rejection**

Add to `rust/geometry/tests/cubic_segment.rs`:

```rust
#[test]
fn live_reduce_rejects_g1() {
    use geometry::{GeometryPipeline, FitterParams, Item};

    let mut pipeline = GeometryPipeline::new(FitterParams::default());
    let mut events = vec![];
    let mut sink = |evt| events.push(evt);

    let g1_input = "G1 X10 Y10 F1000\n";
    let result: Vec<_> = pipeline.process(g1_input, &mut sink).collect();

    // Expect at least one Item::Fatal or Item::Recovered with UnsupportedGcode error.
    assert!(
        result.iter().any(|item| matches!(item, Item::Fatal(_))),
        "G1 input should produce a Fatal item in live pipeline"
    );
}

#[test]
fn live_reduce_rejects_g2() {
    use geometry::{GeometryPipeline, FitterParams, Item};

    let mut pipeline = GeometryPipeline::new(FitterParams::default());
    let mut events = vec![];
    let mut sink = |evt| events.push(evt);

    let g2_input = "G2 X10 Y10 I5 J5 F1000\n";
    let result: Vec<_> = pipeline.process(g2_input, &mut sink).collect();
    assert!(result.iter().any(|item| matches!(item, Item::Fatal(_))));
}
```

(`FitterParams::default()` may not exist — add `#[derive(Default)]` to it if missing, or construct explicitly.)

- [ ] **Step 6: Run tests**

```bash
cargo test -p geometry --test cubic_segment
```

Expected: all pass, including the two new rejection tests.

- [ ] **Step 7: Commit**

```bash
git add rust/geometry/src/reduce.rs rust/geometry/tests/cubic_segment.rs
git commit -m "geometry/reduce: gate G0/G1/G2/G3 paths behind legacy-reference; reject in live"
```

### Task 1.6: Update `pipeline.rs` to emit `Segment::Cubic` for G5/G5.1

**Files:**
- Modify: `rust/geometry/src/pipeline.rs`

- [ ] **Step 1: Identify the existing G5 handler**

```bash
grep -n "Cubic\|FittedSegment\|handle_curve\|G5" rust/geometry/src/pipeline.rs | head -20
```

Find the function (likely `handle_curve` or similar) that produces `FittedSegment` from a `CurveGeom::Cubic`.

- [ ] **Step 2: Replace `FittedSegment` emission with `CubicSegment`**

The existing handler probably looks like:
```rust
fn handle_curve(&mut self, geom: CurveGeom, ...) {
    match geom {
        CurveGeom::Cubic { xyz, e_delta, .. } => {
            self.queue.push_back(Item::Segment(Segment::Fitted(FittedSegment {
                xyz, e: None, feedrate_mm_s: ..., degree: 3,
                max_residual_mm: 0.0, source: ...,
            })));
        }
        // ... other variants ...
    }
}
```

Replace with classification logic. Pseudocode:

```rust
use crate::{CubicSegment, EMode, GeometryError};

fn classify_e_mode(
    xyz: &VectorNurbs<f64, 3>,
    e_delta: Option<f64>,
) -> Result<(EMode, f64, Option<ScalarNurbs<f64>>), GeometryError> {
    const EPS_XYZ: f64 = 1e-6;
    const EPS_Z: f64 = 1e-6;
    const EPS_E: f64 = 1e-6;

    let cps = xyz.control_points();
    let dx = (cps[3][0] - cps[0][0]).abs();
    let dy = (cps[3][1] - cps[0][1]).abs();
    let dz = (cps[3][2] - cps[0][2]).abs();
    let dxy_sq = dx * dx + dy * dy;
    let dxy = dxy_sq.sqrt();
    let de = e_delta.unwrap_or(0.0);
    let abs_de = de.abs();

    let xyz_motion = dxy > EPS_XYZ;
    let z_motion = dz > EPS_Z;
    let e_motion = abs_de > EPS_E;

    match (xyz_motion, z_motion, e_motion) {
        (true, false, true) => {
            // CoupledToXy: signed ratio.
            let ratio = de / xy_arc_length_of_cubic(xyz);
            Ok((EMode::CoupledToXy, ratio, None))
        }
        (true, true, true) => Err(GeometryError::HelicalExtrusionUnsupported),
        (true, _, false) => Ok((EMode::Travel, 0.0, None)),
        (false, _, true) => {
            let e_curve = build_linear_e_curve(de);
            Ok((EMode::Independent, 0.0, Some(e_curve)))
        }
        (false, false, false) => Err(GeometryError::ZeroMotion),
        (false, true, false) => Ok((EMode::Travel, 0.0, None)),
    }
}

fn xy_arc_length_of_cubic(xyz: &VectorNurbs<f64, 3>) -> f64 {
    nurbs::arc_length::xy_arc_length(xyz)
    // (Layer-0 helper added in Task 2.1.)
}

fn build_linear_e_curve(e_delta: f64) -> ScalarNurbs<f64> {
    // Linear NURBS [0 → e_delta] over u ∈ [0, 1].
    ScalarNurbs::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, e_delta],
        None,
    )
    .expect("linear E curve always valid")
}
```

(Note: `xy_arc_length` is added in Task 2.1; the function compiles after that task lands. Phase 2 may need to land before this part of Phase 1 in execution order — see Phase 1's note about parallel-able primitive work.)

- [ ] **Step 3: Wire the classification into the G5 handler**:

```rust
match geom {
    CurveGeom::Cubic { xyz, e_delta, source, feedrate_mm_s } => {
        let classification = classify_e_mode(&xyz, e_delta);
        match classification {
            Ok((e_mode, extrusion_per_xy_mm, e_independent)) => {
                let cubic_seg = CubicSegment::try_new(
                    xyz, e_mode, extrusion_per_xy_mm, e_independent,
                    feedrate_mm_s, source, None,
                );
                match cubic_seg {
                    Ok(seg) => self.queue.push_back(Item::Segment(Segment::Cubic(seg))),
                    Err(err) => self.queue.push_back(Item::Fatal(Fatal::from(err))),
                }
            }
            Err(GeometryError::ZeroMotion) => {
                // Drop zero-motion segments silently.
            }
            Err(err) => {
                self.queue.push_back(Item::Fatal(Fatal::from(err)));
            }
        }
    }
    CurveGeom::Quadratic { xyz, e_delta, source, feedrate_mm_s } => {
        // Degree-elevate G5.1 quadratic to cubic per spec §6.5.
        let cubic_xyz = degree_elevate_2_to_3(&xyz);
        // Then same classification + emission as Cubic.
        // (Refactor: extract a helper "emit_cubic_segment" for the shared logic.)
        emit_cubic_segment(self, cubic_xyz, e_delta, source, feedrate_mm_s);
    }
    // ... legacy variants remain gated ...
}
```

- [ ] **Step 4: Implement `degree_elevate_2_to_3`**:

```rust
/// Bernstein degree-elevation from a degree-2 polynomial NURBS to degree-3,
/// preserving the curve exactly (no fit error). For Bézier control points
/// `[Q_0, Q_1, Q_2]`, the equivalent degree-3 has CPs:
///   `[Q_0, (1/3)Q_0 + (2/3)Q_1, (2/3)Q_1 + (1/3)Q_2, Q_2]`
/// (per Piegl & Tiller §5.5).
fn degree_elevate_2_to_3(quadratic: &VectorNurbs<f64, 3>) -> VectorNurbs<f64, 3> {
    debug_assert_eq!(quadratic.degree(), 2);
    debug_assert_eq!(quadratic.control_points().len(), 3);
    debug_assert!(quadratic.weights().is_none(), "G5.1 is non-rational");
    let q = quadratic.control_points();
    let p0 = q[0];
    let p1 = [
        (1.0 / 3.0) * q[0][0] + (2.0 / 3.0) * q[1][0],
        (1.0 / 3.0) * q[0][1] + (2.0 / 3.0) * q[1][1],
        (1.0 / 3.0) * q[0][2] + (2.0 / 3.0) * q[1][2],
    ];
    let p2 = [
        (2.0 / 3.0) * q[1][0] + (1.0 / 3.0) * q[2][0],
        (2.0 / 3.0) * q[1][1] + (1.0 / 3.0) * q[2][1],
        (2.0 / 3.0) * q[1][2] + (1.0 / 3.0) * q[2][2],
    ];
    let p3 = q[2];
    VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![p0, p1, p2, p3],
        None,
    )
    .expect("degree-elevation always valid")
}
```

- [ ] **Step 5: Add the `Fatal::from(GeometryError)` impl**:

In `rust/geometry/src/error.rs` (or wherever `Fatal` is defined):

```rust
impl From<GeometryError> for Fatal {
    fn from(err: GeometryError) -> Fatal {
        Fatal::Geometry(err.to_string())
        // (or whatever shape Fatal has; adapt to the existing enum)
    }
}
```

- [ ] **Step 6: Verify compile (after Task 2.1 lands)**

```bash
cargo check -p geometry
```

If `xy_arc_length` is not yet available from `nurbs`, this step waits on Task 2.1. To unblock Phase 1, you can use a placeholder `xy_arc_length_of_cubic` that integrates inline (5-point Gauss-Legendre on the cubic's `√(x'² + y'²)`). That is exactly the implementation Task 2.1 will land in `nurbs::arc_length`; landing it inline first and refactoring later is a valid sequencing.

- [ ] **Step 7: Add a degree-elevation test**

Add to `rust/geometry/tests/cubic_segment.rs`:

```rust
#[test]
fn degree_elevation_preserves_curve() {
    // Test that G5.1 → cubic via degree-elevation is exact.
    use nurbs::{VectorNurbs, eval::vector_eval};

    let q = VectorNurbs::<f64, 3>::try_new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [1.0, 1.0, 0.0], [2.0, 0.0, 0.0]],
        None,
    ).unwrap();

    let cubic = degree_elevate_2_to_3(&q);

    // Sample 100 points; quadratic and cubic must agree to f64 round-off.
    for i in 0..=100 {
        let u = i as f64 / 100.0;
        let q_val = vector_eval(&q, u);
        let c_val = vector_eval(&cubic, u);
        for axis in 0..3 {
            assert!(
                (q_val[axis] - c_val[axis]).abs() < 1e-12,
                "axis {axis} mismatch at u={u}: q={:?} c={:?}",
                q_val, c_val,
            );
        }
    }
}
```

The `degree_elevate_2_to_3` function will need to be exposed or moved to a place where the test can reach it. Either:
- Make it `pub(crate)` and keep the test inside `pipeline.rs` as `#[cfg(test)]`.
- Make it `pub` and re-export from `lib.rs`.

For 7-pre, `pub` re-export keeps it testable without surgical access; the function is a useful utility that may be needed elsewhere later.

- [ ] **Step 8: Run all geometry tests**

```bash
cargo test -p geometry
```

All should pass.

- [ ] **Step 9: Commit**

```bash
git add rust/geometry/src/pipeline.rs rust/geometry/src/error.rs rust/geometry/tests/cubic_segment.rs
git commit -m "geometry/pipeline: emit Segment::Cubic with E-mode classification + G5.1 degree elevation"
```

### Task 1.7: Verify Phase 1 self-consistency

**Files:** none (verification step)

- [ ] **Step 1: Run all geometry tests in both feature configurations**

```bash
cargo test -p geometry
cargo test -p geometry --features legacy-reference
```

Both must pass.

- [ ] **Step 2: Run clippy**

```bash
cargo clippy -p geometry --all-targets -- -D warnings
cargo clippy -p geometry --all-targets --features legacy-reference -- -D warnings
```

Both must be clean.

- [ ] **Step 3: Sanity-check from the workspace root**

```bash
cargo check --workspace
cargo check --workspace --features geometry/legacy-reference
```

Both must be clean. (The temporal crate's tests reference the pipeline and may need feature-gate updates — handle in Task 4.x.)

- [ ] **Step 4: No commit needed** — Phase 1 is verification-only at this point.

---

## Phase 2 — Layer-0 primitives

Phase 2 lands the three new primitives in `nurbs`. Tasks 2.1, 2.2, 2.3 are independent of each other and can be developed in parallel by separate agents.

### Task 2.1: Add `xy_arc_length` scalar primitive

**Files:**
- Modify: `rust/nurbs/src/arc_length.rs`
- Create: `rust/nurbs/tests/xy_arc_length.rs`

- [ ] **Step 1: Read existing arc-length implementation for reference**

```bash
grep -n "build_arc_length_table_vector\|gauss_legendre\|GAUSS_5" rust/nurbs/src/arc_length.rs | head
```

Identify the 5-point Gauss-Legendre quadrature helper the existing code uses. Reuse it.

- [ ] **Step 2: Write the failing test first**

Create `rust/nurbs/tests/xy_arc_length.rs`:

```rust
use nurbs::{VectorNurbs, arc_length::xy_arc_length};

/// Pure-XY straight-line cubic: XY arc length should equal endpoint distance to f64 round-off.
#[test]
fn pure_xy_straight_line_collinear_cubic() {
    // P0=(0,0,0), P1=(1,0,0), P2=(2,0,0), P3=(3,0,0): straight line of length 3.
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [3.0, 0.0, 0.0]],
        None,
    ).unwrap();

    let l = xy_arc_length(&xyz);
    assert!((l - 3.0).abs() < 1e-9, "expected ~3.0, got {l}");
}

/// Pure-Z motion: XY arc length should be zero.
#[test]
fn pure_z_motion_xy_length_zero() {
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 2.0], [0.0, 0.0, 3.0]],
        None,
    ).unwrap();

    let l = xy_arc_length(&xyz);
    assert!(l.abs() < 1e-9, "expected ~0.0, got {l}");
}

/// Diagonal X+Y straight line of length sqrt(2)*3 ≈ 4.2426; pure-XY case.
#[test]
fn diagonal_xy_straight_line() {
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [1.0, 1.0, 0.0], [2.0, 2.0, 0.0], [3.0, 3.0, 0.0]],
        None,
    ).unwrap();

    let l = xy_arc_length(&xyz);
    let expected = 3.0 * std::f64::consts::SQRT_2;
    assert!((l - expected).abs() < 1e-9, "expected ~{expected}, got {l}");
}

/// Pure-XY curve = match the 3D arc length to f64 round-off.
#[test]
fn pure_xy_curve_matches_3d_length() {
    // Quarter-arc-shaped cubic Bézier in the XY plane, Z=0 throughout.
    // Approximation of a unit quarter-arc: standard cubic Bézier control points.
    let k = 4.0 / 3.0 * (std::f64::consts::PI / 8.0).tan();
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [1.0, 0.0, 0.0],
            [1.0, k, 0.0],
            [k, 1.0, 0.0],
            [0.0, 1.0, 0.0],
        ],
        None,
    ).unwrap();

    let xy_l = xy_arc_length(&xyz);
    // Build the 3D arc-length table for cross-check.
    let table_3d = nurbs::arc_length::build_arc_length_table_vector(&xyz, 1e-9, 64).unwrap();
    let l_3d = table_3d.s_max();

    assert!(
        (xy_l - l_3d).abs() < 1e-9,
        "pure-XY: xy_arc_length should match 3D arc length, got xy={xy_l} vs 3d={l_3d}"
    );
}

/// Loop closure (XY): chord-zero but real XY motion. xy_arc_length must be nonzero.
#[test]
fn xy_loop_chord_zero_arc_length_nonzero() {
    // Cubic Bézier returning to its start: P0 ≈ P3 (different control points P1, P2).
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [1.0, 1.0, 0.0], [-1.0, 1.0, 0.0], [0.0, 0.0, 0.0]],
        None,
    ).unwrap();

    let l = xy_arc_length(&xyz);
    assert!(l > 0.5, "loop should have nonzero XY arc length, got {l}");
}
```

- [ ] **Step 3: Verify the test fails to compile**

```bash
cargo test -p nurbs --test xy_arc_length
```

Expected: compile error: `xy_arc_length` not found in `nurbs::arc_length`.

- [ ] **Step 4: Implement `xy_arc_length`**

In `rust/nurbs/src/arc_length.rs`, add:

```rust
/// Compute the XY-projection arc length of a 3D vector NURBS via 5-point Gauss-Legendre
/// quadrature with adaptive doubling until residual < `1e-9 * total_length` (capped at
/// 64 subintervals).
///
/// Integrates `√(x'(u)² + y'(u)²) du` over `u ∈ [knots.first(), knots.last()]`.
///
/// For pure-XY input (Δz = 0 across the curve), this matches the 3D arc length to
/// f64 round-off. For helical input (varying Z), this gives the in-plane projection
/// length used by 7-pre's E-coupled-to-XY classification (`extrusion_per_xy_mm`).
///
/// Cost: ~30 quadrature evaluations for typical cubic Bézier inputs. No table built.
#[cfg(feature = "host")]
pub fn xy_arc_length<const D: usize>(xyz: &crate::VectorNurbs<f64, D>) -> f64
where
    [(); D]:,
{
    use crate::eval::vector_derivative;
    debug_assert!(D >= 2, "xy_arc_length requires D >= 2 (X, Y axes present)");

    // 5-point Gauss-Legendre nodes and weights on [-1, 1].
    const GL5_NODES: [f64; 5] = [
        -0.9061798459386640,
        -0.5384693101056831,
         0.0,
         0.5384693101056831,
         0.9061798459386640,
    ];
    const GL5_WEIGHTS: [f64; 5] = [
        0.2369268850561891,
        0.4786286704993665,
        0.5688888888888889,
        0.4786286704993665,
        0.2369268850561891,
    ];

    fn integrate_xy<const D: usize>(
        xyz: &crate::VectorNurbs<f64, D>,
        u_lo: f64,
        u_hi: f64,
    ) -> f64 {
        let half = (u_hi - u_lo) * 0.5;
        let mid = (u_lo + u_hi) * 0.5;
        let deriv = vector_derivative(xyz);
        let mut sum = 0.0;
        for (i, &node) in GL5_NODES.iter().enumerate() {
            let u = mid + half * node;
            let d = crate::eval::vector_eval(&deriv, u);
            // XY projection: only axes 0 and 1.
            let speed_xy = (d[0] * d[0] + d[1] * d[1]).sqrt();
            sum += GL5_WEIGHTS[i] * speed_xy;
        }
        sum * half
    }

    let knots = xyz.knots();
    let u_start = knots[0];
    let u_end = *knots.last().unwrap();

    let mut estimate = integrate_xy(xyz, u_start, u_end);
    let mut prev_estimate;
    let mut n_subintervals: usize = 1;
    let max_subintervals: usize = 64;
    let tol_rel: f64 = 1e-9;

    loop {
        prev_estimate = estimate;
        n_subintervals *= 2;
        if n_subintervals > max_subintervals {
            break;
        }
        let mut sum = 0.0;
        for k in 0..n_subintervals {
            let u_lo = u_start + (u_end - u_start) * (k as f64) / (n_subintervals as f64);
            let u_hi = u_start + (u_end - u_start) * ((k + 1) as f64) / (n_subintervals as f64);
            sum += integrate_xy(xyz, u_lo, u_hi);
        }
        estimate = sum;
        if (estimate - prev_estimate).abs() < tol_rel * estimate.abs().max(1e-12) {
            break;
        }
    }
    estimate
}
```

- [ ] **Step 5: Run the test**

```bash
cargo test -p nurbs --test xy_arc_length
```

Expected: all 5 tests pass. (Note: `build_arc_length_table_vector`'s exact signature may differ; adjust the cross-check call as needed.)

- [ ] **Step 6: Run clippy**

```bash
cargo clippy -p nurbs --all-targets -- -D warnings
```

Expected: clean.

- [ ] **Step 7: Commit**

```bash
git add rust/nurbs/src/arc_length.rs rust/nurbs/tests/xy_arc_length.rs
git commit -m "nurbs/arc_length: add xy_arc_length scalar one-shot query"
```

### Task 2.2: Add `compose_vector_piece` primitive

**Files:**
- Modify: `rust/nurbs/src/algebra.rs`
- Create: `rust/nurbs/tests/compose_vector_piece.rs`

- [ ] **Step 1: Write the failing test**

Create `rust/nurbs/tests/compose_vector_piece.rs`:

```rust
use nurbs::algebra::compose_vector_piece;
use nurbs::bezier::BezierPiece;

/// Identity composition: outer ∘ identity = outer.
#[test]
fn identity_composition_returns_outer() {
    let outer_x = BezierPiece::<f64> {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![1.0, 2.0, 3.0, 4.0],  // p(s) = 1 + 2s + 3s² + 4s³ on [0,1]
    };
    let outer_y = BezierPiece::<f64> {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![0.0, 1.0, 0.0, 0.0],  // p(s) = s
    };
    let outer_z = BezierPiece::<f64> {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![5.0, 0.0, 0.0, 0.0],  // p(s) = 5
    };
    // identity(t) = t in Pascal-shifted basis on [0, 1] is [0, 1].
    let inner = BezierPiece::<f64> {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![0.0, 1.0],
    };

    let composed = compose_vector_piece::<3>(
        &[&outer_x, &outer_y, &outer_z],
        &inner,
    ).unwrap();

    // Sample at 100 points and check.
    for i in 0..=100 {
        let t = i as f64 / 100.0;
        let composed_x = composed[0].evaluate(t);
        let composed_y = composed[1].evaluate(t);
        let composed_z = composed[2].evaluate(t);
        let expected_x = outer_x.evaluate(inner.evaluate(t));
        let expected_y = outer_y.evaluate(inner.evaluate(t));
        let expected_z = outer_z.evaluate(inner.evaluate(t));
        assert!((composed_x - expected_x).abs() < 1e-10, "x mismatch at t={t}");
        assert!((composed_y - expected_y).abs() < 1e-10, "y mismatch at t={t}");
        assert!((composed_z - expected_z).abs() < 1e-10, "z mismatch at t={t}");
    }
}

/// Linear inner: outer ∘ linear-rescaling = outer composed with rescaling.
#[test]
fn linear_inner_is_parameter_rescaling() {
    let outer = BezierPiece::<f64> {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![0.0, 1.0, 0.0, 1.0],  // p(s) = s + s³
    };
    // inner(t) = 0.5 * t = t/2: maps [0, 1] → [0, 0.5].
    let inner = BezierPiece::<f64> {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![0.0, 0.5],
    };
    let outer_subdomain = BezierPiece::<f64> {
        u_start: 0.0,
        u_end: 0.5,
        coeffs: outer.coeffs.clone(),
    };

    // outer's domain after composition with [0, 0.5]-mapping inner is [0, 0.5].
    let composed = compose_vector_piece::<1>(&[&outer_subdomain], &inner).unwrap();

    for i in 0..=50 {
        let t = i as f64 / 100.0;  // t in [0, 0.5]
        let composed_val = composed[0].evaluate(t);
        let expected = outer_subdomain.evaluate(inner.evaluate(t));
        assert!(
            (composed_val - expected).abs() < 1e-10,
            "mismatch at t={t}: composed={composed_val} expected={expected}"
        );
    }
}

/// Cubic outer × quadratic inner = degree-6 polynomial in t.
#[test]
fn cubic_outer_quadratic_inner_yields_degree_6() {
    // outer(s) = 1 + s + s² + s³ on s ∈ [0, 1].
    let outer = BezierPiece::<f64> {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![1.0, 1.0, 1.0, 1.0],
    };
    // inner(t) = t² on t ∈ [0, 1] (Pascal-shifted around 0): coeffs = [0, 0, 1].
    let inner = BezierPiece::<f64> {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![0.0, 0.0, 1.0],
    };

    let composed = compose_vector_piece::<1>(&[&outer], &inner).unwrap();

    assert_eq!(composed[0].degree(), 6, "expected degree 6, got {}", composed[0].degree());

    // Sample values must match outer(inner(t)) = 1 + t² + t⁴ + t⁶.
    for i in 0..=100 {
        let t = i as f64 / 100.0;
        let composed_val = composed[0].evaluate(t);
        let expected = 1.0 + t * t + t * t * t * t + t * t * t * t * t * t;
        assert!(
            (composed_val - expected).abs() < 1e-10,
            "mismatch at t={t}: got {composed_val} expected {expected}"
        );
    }
}
```

- [ ] **Step 2: Verify test fails to compile**

```bash
cargo test -p nurbs --test compose_vector_piece
```

Expected: `compose_vector_piece` not found.

- [ ] **Step 3: Implement `compose_vector_piece`**

In `rust/nurbs/src/algebra.rs`:

```rust
/// Compose a vector of polynomial Bézier pieces (in s-domain) with a scalar polynomial
/// Bézier piece (in t-domain). Returns per-axis polynomial pieces in t-domain on
/// `inner`'s `[u_start, u_end]` interval.
///
/// **Mathematical operation**: for each axis a, compute the polynomial
/// `outer_a(inner(t))`. Output degree per axis = `outer.degree() × inner.degree()`.
/// For T-A's typical use (outer.degree() = 6, inner.degree() = 2): output degree 12.
///
/// **Precondition** (debug-asserted): `outer[a].u_start == inner.evaluate(inner.u_start)`
/// and `outer[a].u_end == inner.evaluate(inner.u_end)` for all axes a — the outer
/// polynomial's s-domain must align with the inner's s-image. Caller is responsible
/// for ensuring this alignment (typically via `fit_x_to_arc_length_piece` whose output
/// has `u_start = s_lo` and `u_end = s_hi` matching the TOPP-RA grid piece).
///
/// **Storage basis**: input and output use the Pascal-shifted monomial basis native
/// to `BezierPiece` — `p(u) = Σ coeffs[k] × (u − u_start)^k`. Algorithm is direct
/// substitution-and-collect in this basis.
#[cfg(feature = "host")]
pub fn compose_vector_piece<const D: usize>(
    outer: &[&crate::bezier::BezierPiece<f64>; D],
    inner: &crate::bezier::BezierPiece<f64>,
) -> Result<[crate::bezier::BezierPiece<f64>; D], AlgebraError> {
    use crate::bezier::BezierPiece;

    // Debug-assert affine alignment per spec §4.4.
    debug_assert!(
        (0..D).all(|a| {
            (outer[a].u_start - inner.evaluate(inner.u_start)).abs() < 1e-9
                && (outer[a].u_end - inner.evaluate(inner.u_end)).abs() < 1e-9
        }),
        "compose_vector_piece: outer.u_start/u_end must match inner's s-image"
    );

    let d_inner = inner.degree();
    let mut result: Vec<BezierPiece<f64>> = Vec::with_capacity(D);

    for &outer_axis in outer.iter() {
        let d_outer = outer_axis.degree();
        let result_degree = d_outer * d_inner;

        // Algorithm: outer(s) = Σ_i outer.coeffs[i] × (s − outer.u_start)^i,
        // where s = inner(t) = Σ_j inner.coeffs[j] × (t − inner.u_start)^j.
        // Substitute s into the i-th outer term, expand, collect by power of (t − inner.u_start).

        // We first compute (inner(t) − outer.u_start) as a polynomial in (t − inner.u_start):
        //   inner(t) − outer.u_start = (inner.coeffs[0] − outer.u_start)
        //                            + Σ_{j≥1} inner.coeffs[j] × (t − inner.u_start)^j
        let mut shifted_inner: Vec<f64> = inner.coeffs.clone();
        shifted_inner[0] -= outer_axis.u_start;

        // Build powers of `shifted_inner` up to power d_outer, where power 0 = [1.0],
        // power 1 = shifted_inner, power i = shifted_inner × power (i-1).
        let mut powers: Vec<Vec<f64>> = vec![vec![1.0]];
        for i in 1..=d_outer {
            let prev = &powers[i - 1];
            let next = poly_multiply(prev, &shifted_inner);
            powers.push(next);
        }

        // Sum: result_coeffs[k] = Σ_i outer.coeffs[i] × powers[i][k].
        let mut result_coeffs = vec![0.0; result_degree + 1];
        for i in 0..=d_outer {
            let p = &powers[i];
            for (k, &c) in p.iter().enumerate() {
                result_coeffs[k] += outer_axis.coeffs[i] * c;
            }
        }

        result.push(BezierPiece {
            u_start: inner.u_start,
            u_end: inner.u_end,
            coeffs: result_coeffs,
        });
    }

    // Convert Vec<BezierPiece> into [BezierPiece; D] via try_into.
    Ok(result.try_into().expect("D pieces produced"))
}
```

(`poly_multiply` is the existing private helper at `algebra.rs:399` — already does coefficient convolution. If it's not pub-visible at the call site, either make it `pub(crate)` or duplicate inline.)

- [ ] **Step 4: Run the test**

```bash
cargo test -p nurbs --test compose_vector_piece
```

Expected: all 3 tests pass.

- [ ] **Step 5: Add a sympy cross-check fixture (optional but recommended)**

Create `rust/nurbs/tests/scripts/generate_compose_corpus.py` that generates a JSON of `(outer, inner, expected_output_at_sample_points)` tuples using sympy. Cross-check in a Rust test that loads the JSON. (Optional follow-up; the inline tests above cover most cases.)

- [ ] **Step 6: Commit**

```bash
git add rust/nurbs/src/algebra.rs rust/nurbs/tests/compose_vector_piece.rs
git commit -m "nurbs/algebra: add compose_vector_piece (polynomial-of-polynomial in monomial basis)"
```

### Task 2.3: Add `fit_x_to_arc_length_piece` primitive

**Files:**
- Modify: `rust/nurbs/src/algebra.rs`
- Create: `rust/nurbs/tests/fit_x_to_arc_length_piece.rs`

- [ ] **Step 1: Add `FitError` enum to `algebra.rs`**

```rust
#[cfg(feature = "host")]
#[derive(Debug, Clone, PartialEq)]
pub enum FitError {
    /// Reached `max_degree` without satisfying tolerance — caller should split
    /// the piece (recurse with two halves) or return a hard planner error if at
    /// `max_recursion_depth`.
    ToleranceNotReached { achieved_mm: f64, at_degree: u8 },
    /// Pathological input — table inversion or geometry evaluation failed.
    DegenerateInput { reason: &'static str },
}
```

- [ ] **Step 2: Write the failing tests**

Create `rust/nurbs/tests/fit_x_to_arc_length_piece.rs`:

```rust
use nurbs::algebra::{fit_x_to_arc_length_piece, FitError};
use nurbs::{VectorNurbs, arc_length::build_arc_length_table_vector};

fn cubic_straight_line() -> VectorNurbs<f64, 3> {
    // Line from (0,0,0) to (10,0,0).
    VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [10.0/3.0, 0.0, 0.0], [20.0/3.0, 0.0, 0.0], [10.0, 0.0, 0.0]],
        None,
    ).unwrap()
}

#[test]
fn straight_line_fits_at_low_degree() {
    let xyz = cubic_straight_line();
    let table = build_arc_length_table_vector(&xyz, 1e-9, 64).unwrap();
    let table_ref = table.as_view();
    // Fit a 0.5 mm piece in the middle of the line.
    let result = fit_x_to_arc_length_piece::<3>(
        &xyz, &table_ref, 4.0, 4.5,
        /*target_degree=*/3, /*max_degree=*/10, /*tolerance_mm=*/1e-3,
    );
    assert!(result.is_ok(), "expected Ok, got {:?}", result);
    let pieces = result.unwrap();
    // Verify each piece's u_start and u_end match the s-domain.
    for axis in 0..3 {
        assert!((pieces[axis].u_start - 4.0).abs() < 1e-9);
        assert!((pieces[axis].u_end - 4.5).abs() < 1e-9);
    }
}

#[test]
fn quarter_arc_fits_at_low_degree() {
    // Cubic Bézier approximation of a quarter circle, R = 10.
    let r = 10.0;
    let k = 4.0 / 3.0 * (std::f64::consts::PI / 8.0).tan();
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [r, 0.0, 0.0],
            [r, r * k, 0.0],
            [r * k, r, 0.0],
            [0.0, r, 0.0],
        ],
        None,
    ).unwrap();

    let table = build_arc_length_table_vector(&xyz, 1e-9, 64).unwrap();
    let table_ref = table.as_view();
    let s_max = table.s_max();

    // Fit a 0.5 mm piece in the middle of the quarter-arc.
    let result = fit_x_to_arc_length_piece::<3>(
        &xyz, &table_ref, s_max * 0.4, s_max * 0.4 + 0.5,
        /*target_degree=*/3, /*max_degree=*/10, /*tolerance_mm=*/1e-3,
    );
    assert!(result.is_ok(), "quarter arc fit failed: {:?}", result);
}
```

- [ ] **Step 3: Verify test fails to compile**

```bash
cargo test -p nurbs --test fit_x_to_arc_length_piece
```

Expected: `fit_x_to_arc_length_piece` not found.

- [ ] **Step 4: Implement `fit_x_to_arc_length_piece`**

In `rust/nurbs/src/algebra.rs`, add:

```rust
/// Adaptive polynomial fit of `x(s)` on a TOPP-RA grid piece `[s_lo, s_hi]`,
/// sample-verified to a configurable L∞ position-error tolerance.
///
/// Returns one `BezierPiece` per axis, all sharing the s-domain `[s_lo, s_hi]`,
/// in Pascal-shifted-monomial basis (matching `BezierPiece`'s storage convention).
///
/// **Algorithm (per spec §4.5):**
/// 1. Generate `target_degree + 1` Chebyshev-of-the-second-kind nodes in `[s_lo, s_hi]`.
/// 2. For each node, query u via `arc_length::param_from_arc_length`, evaluate `x` via
///    `vector_eval(geometry, u)`.
/// 3. Solve Lagrange interpolation per axis (Vandermonde-like solve at degree d ≤ 10
///    on Chebyshev nodes — well-conditioned).
/// 4. **Verification step**: oversample residual at `4·(d+1)` uniform points, take L∞.
///    If above tolerance: increase d by 1, return to step 1. Cap at `max_degree`.
/// 5. If verification fails at `max_degree`: return `FitError::ToleranceNotReached`.
///    Caller should bisect the piece and recurse (bounded at `max_recursion_depth = 8`).
#[cfg(feature = "host")]
pub fn fit_x_to_arc_length_piece<const D: usize>(
    geometry: &crate::VectorNurbs<f64, D>,
    table: &crate::ArcLengthTableRef<'_, f64>,
    s_lo: f64,
    s_hi: f64,
    target_degree: u8,
    max_degree: u8,
    tolerance_mm: f64,
) -> Result<[crate::bezier::BezierPiece<f64>; D], FitError>
where
    [(); D]:,
{
    use crate::bezier::BezierPiece;
    use crate::eval::vector_eval;

    if !(s_hi > s_lo) {
        return Err(FitError::DegenerateInput { reason: "s_hi <= s_lo" });
    }
    if !s_lo.is_finite() || !s_hi.is_finite() {
        return Err(FitError::DegenerateInput { reason: "s endpoints not finite" });
    }
    if target_degree > max_degree {
        return Err(FitError::DegenerateInput { reason: "target_degree > max_degree" });
    }

    let mut d = target_degree;
    loop {
        // Generate d+1 Chebyshev-of-2nd-kind nodes in [s_lo, s_hi]:
        //   s_i = (s_lo + s_hi)/2 + (s_hi − s_lo)/2 × cos(i × π / d)
        let n = d as usize + 1;
        let mut s_nodes = Vec::with_capacity(n);
        let mid = (s_lo + s_hi) * 0.5;
        let half = (s_hi - s_lo) * 0.5;
        for i in 0..n {
            let cos_arg = (i as f64) * std::f64::consts::PI / (d as f64);
            s_nodes.push(mid + half * cos_arg.cos());
        }
        s_nodes.sort_by(|a, b| a.partial_cmp(b).unwrap());

        // Evaluate x at each node.
        let mut samples: Vec<[f64; D]> = Vec::with_capacity(n);
        for &s in s_nodes.iter() {
            let u = crate::arc_length::param_from_arc_length(table, s);
            let x = vector_eval(geometry, u);
            samples.push(x);
        }

        // Solve Lagrange interpolation per axis to produce monomial-shifted coefficients
        // on [s_lo, s_hi]. Approach: build the Vandermonde-style matrix A_ij = (s_nodes[i] - s_lo)^j
        // and solve A * coeffs = samples for each axis.
        let coeffs_per_axis = lagrange_interpolation_pascal_shifted(
            &s_nodes, &samples, s_lo,
        );

        // Verification step: oversample at 4×(d+1) points; max axis residual.
        let n_verify = 4 * (d as usize + 1);
        let mut max_err: f64 = 0.0;
        for k in 0..n_verify {
            let t = k as f64 / (n_verify - 1).max(1) as f64;
            let s = s_lo + (s_hi - s_lo) * t;
            let u = crate::arc_length::param_from_arc_length(table, s);
            let truth = vector_eval(geometry, u);
            for axis in 0..D {
                let p_val = horner_pascal_shifted(&coeffs_per_axis[axis], s, s_lo);
                let err = (truth[axis] - p_val).abs();
                if err > max_err {
                    max_err = err;
                }
            }
        }

        if max_err <= tolerance_mm {
            // Pack into [BezierPiece; D].
            let mut pieces: Vec<BezierPiece<f64>> = Vec::with_capacity(D);
            for axis in 0..D {
                pieces.push(BezierPiece {
                    u_start: s_lo,
                    u_end: s_hi,
                    coeffs: coeffs_per_axis[axis].clone(),
                });
            }
            return Ok(pieces.try_into().expect("D pieces produced"));
        }

        if d >= max_degree {
            return Err(FitError::ToleranceNotReached {
                achieved_mm: max_err,
                at_degree: d,
            });
        }
        d += 1;
    }
}

#[cfg(feature = "host")]
fn lagrange_interpolation_pascal_shifted<const D: usize>(
    s_nodes: &[f64],
    samples: &[[f64; D]],
    s_origin: f64,
) -> Vec<Vec<f64>> {
    // Build Vandermonde A[i][j] = (s_nodes[i] − s_origin)^j and solve A × coeffs = samples
    // per axis. Use Gauss elimination with partial pivoting (tiny matrix, ~10×10 worst case).
    let n = s_nodes.len();
    let mut matrix = vec![vec![0.0; n + 1]; n];
    let mut coeffs_per_axis: Vec<Vec<f64>> = (0..D).map(|_| vec![0.0; n]).collect();

    for axis in 0..D {
        // Build augmented matrix [A | b] where b[i] = samples[i][axis].
        for i in 0..n {
            let mut x_pow = 1.0;
            let dx = s_nodes[i] - s_origin;
            for j in 0..n {
                matrix[i][j] = x_pow;
                x_pow *= dx;
            }
            matrix[i][n] = samples[i][axis];
        }
        // Gauss elimination with partial pivoting.
        for k in 0..n {
            // Pivot.
            let mut pivot = k;
            for i in (k + 1)..n {
                if matrix[i][k].abs() > matrix[pivot][k].abs() {
                    pivot = i;
                }
            }
            matrix.swap(k, pivot);
            // Eliminate.
            for i in (k + 1)..n {
                let factor = matrix[i][k] / matrix[k][k];
                for j in k..=n {
                    matrix[i][j] -= factor * matrix[k][j];
                }
            }
        }
        // Back substitution.
        for i in (0..n).rev() {
            let mut sum = matrix[i][n];
            for j in (i + 1)..n {
                sum -= matrix[i][j] * coeffs_per_axis[axis][j];
            }
            coeffs_per_axis[axis][i] = sum / matrix[i][i];
        }
    }
    coeffs_per_axis
}

#[cfg(feature = "host")]
fn horner_pascal_shifted(coeffs: &[f64], s: f64, s_origin: f64) -> f64 {
    let dx = s - s_origin;
    let mut acc = 0.0;
    for &c in coeffs.iter().rev() {
        acc = acc * dx + c;
    }
    acc
}
```

- [ ] **Step 5: Run the test**

```bash
cargo test -p nurbs --test fit_x_to_arc_length_piece
```

Expected: both tests pass.

- [ ] **Step 6: Commit**

```bash
git add rust/nurbs/src/algebra.rs rust/nurbs/tests/fit_x_to_arc_length_piece.rs
git commit -m "nurbs/algebra: add fit_x_to_arc_length_piece (adaptive Chebyshev fit)"
```

### Task 2.4: Phase 2 verification

**Files:** none

- [ ] **Step 1: Run all nurbs tests**

```bash
cargo test -p nurbs
cargo clippy -p nurbs --all-targets -- -D warnings
```

Both must pass.

---

## Phase 3 — Layer-1 splitter

### Task 3.1: Add `split_segment_to_cap` primitive

**Files:**
- Create: `rust/geometry/src/splitter.rs`
- Modify: `rust/geometry/src/lib.rs`
- Create: `rust/geometry/tests/split_segment_to_cap.rs`

- [ ] **Step 1: Write the failing tests**

Create `rust/geometry/tests/split_segment_to_cap.rs`:

```rust
use geometry::{CubicSegment, EMode, SourceRange, split_segment_to_cap};
use nurbs::VectorNurbs;

fn straight_cubic(length_mm: f64) -> CubicSegment {
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [length_mm / 3.0, 0.0, 0.0],
            [2.0 * length_mm / 3.0, 0.0, 0.0],
            [length_mm, 0.0, 0.0],
        ],
        None,
    ).unwrap();
    CubicSegment::try_new(
        xyz, EMode::Travel, 0.0, None, 100.0,
        SourceRange { start_line: 1, end_line: 1 }, None,
    ).unwrap()
}

#[test]
fn passthrough_when_below_cap() {
    let seg = straight_cubic(5.0);
    let out = split_segment_to_cap(&seg, 12.5).unwrap();
    assert_eq!(out.len(), 1);
    assert!(out[0].split_info.is_none());
}

#[test]
fn passthrough_at_exact_cap() {
    let seg = straight_cubic(12.5);
    let out = split_segment_to_cap(&seg, 12.5).unwrap();
    assert_eq!(out.len(), 1);
    assert!(out[0].split_info.is_none());
}

#[test]
fn splits_into_two_at_25mm() {
    let seg = straight_cubic(25.0);
    let out = split_segment_to_cap(&seg, 12.5).unwrap();
    assert_eq!(out.len(), 2);
    for (i, child) in out.iter().enumerate() {
        let info = child.split_info.expect("split_info populated");
        assert_eq!(info.sub_index, i as u32);
        assert_eq!(info.sub_count, 2);
    }
}

#[test]
fn splits_into_eight_at_100mm() {
    let seg = straight_cubic(100.0);
    let out = split_segment_to_cap(&seg, 12.5).unwrap();
    assert_eq!(out.len(), 8);
}

#[test]
fn metadata_propagates() {
    let seg = straight_cubic(50.0);
    let out = split_segment_to_cap(&seg, 12.5).unwrap();
    for child in out.iter() {
        assert_eq!(child.feedrate_mm_s, seg.feedrate_mm_s);
        assert_eq!(child.e_mode, seg.e_mode);
        assert_eq!(child.extrusion_per_xy_mm, seg.extrusion_per_xy_mm);
        assert_eq!(child.source, seg.source);
    }
}

#[test]
fn boundary_continuity_bit_exact() {
    use nurbs::eval::vector_eval;
    let seg = straight_cubic(50.0);
    let out = split_segment_to_cap(&seg, 12.5).unwrap();
    for window in out.windows(2) {
        let left_end = vector_eval(&window[0].xyz, 1.0);
        let right_start = vector_eval(&window[1].xyz, 0.0);
        for axis in 0..3 {
            assert_eq!(
                left_end[axis], right_start[axis],
                "boundary mismatch axis {axis}: {left_end:?} vs {right_start:?}"
            );
        }
    }
}

#[test]
fn pure_e_only_independent_passthrough() {
    use nurbs::ScalarNurbs;
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![[0.0; 3]; 4],  // all four CPs at origin → cp_polygon_length == 0
        None,
    ).unwrap();
    let e_curve = ScalarNurbs::<f64>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, -2.0],  // retraction
        None,
    ).unwrap();
    let seg = CubicSegment::try_new(
        xyz, EMode::Independent, 0.0, Some(e_curve), 100.0,
        SourceRange { start_line: 1, end_line: 1 }, None,
    ).unwrap();
    let out = split_segment_to_cap(&seg, 12.5).unwrap();
    assert_eq!(out.len(), 1);
    assert!(out[0].split_info.is_none());
}
```

- [ ] **Step 2: Verify tests fail to compile**

```bash
cargo test -p geometry --test split_segment_to_cap
```

Expected: `split_segment_to_cap` not found.

- [ ] **Step 3: Create `rust/geometry/src/splitter.rs`**:

```rust
//! Path-length-capped subdivision of cubic-Bézier segments. Bounds per-MCU-segment
//! piece count for downstream Layer 3 (T-A) by capping arc length at `max_arc_length_mm`
//! (default 12.5 mm). See `docs/superpowers/specs/2026-04-29-step7-pre-cubic-pipeline-prep-design.md` §5.

use crate::{CubicSegment, EMode, SourceRange, SplitInfo};
use nurbs::{
    arc_length::{build_arc_length_table_vector, param_from_arc_length},
    bezier::{BezierPiece, extract_bezier_pieces, split_piece_at},
    eval::vector_derivative,
    VectorNurbs,
};

const EPS_CP_POLYGON: f64 = 3e-6;
const EPS_U: f64 = 1e-9;
const MIN_PARAMETRIC_SPEED_FOR_SPLITTER: f64 = 1e-9;

#[derive(Debug, Clone, PartialEq)]
pub enum SplitError {
    /// Input violated the single-piece-cubic invariant (e.g., wrong degree, multi-piece NURBS).
    NotSinglePieceCubic,
    /// Arc-length-table build failed.
    ArcLengthTableBuildFailed { reason: &'static str },
}

/// Subdivide a cubic-Bézier `CubicSegment` into sub-segments each ≤ `max_arc_length_mm`
/// of XYZ arc length. See spec §5.
///
/// Returns:
/// - `Ok(vec![segment.clone()])` with `SplitInfo: None` when the segment is below cap
///   (pass-through).
/// - `Ok(Vec<CubicSegment>)` with each sub-segment carrying `Some(SplitInfo)` populated
///   per spec §5.3 step 8.
/// - `Err(SplitError)` on invariant violation or arc-length-table build failure.
pub fn split_segment_to_cap(
    segment: &CubicSegment,
    max_arc_length_mm: f64,
) -> Result<Vec<CubicSegment>, SplitError> {
    debug_assert!(max_arc_length_mm > 0.0, "max_arc_length_mm must be positive");

    // Step 1: zero-motion / degenerate-input passthrough.
    if is_zero_motion(&segment.xyz) {
        return Ok(vec![segment.clone()]);
    }

    // Step 2: build the arc-length table once.
    let table = build_arc_length_table_vector(&segment.xyz, 1e-9, 64)
        .map_err(|_| SplitError::ArcLengthTableBuildFailed { reason: "build failed" })?;
    let table_ref = table.as_view();
    let l = table.s_max();

    // Step 3: passthrough if below cap.
    if l <= max_arc_length_mm {
        return Ok(vec![segment.clone()]);
    }

    // Step 4: compute target arc-lengths.
    let k_planned = (l / max_arc_length_mm).ceil() as usize;
    let mut targets = Vec::with_capacity(k_planned - 1);
    for i in 1..k_planned {
        targets.push(l * (i as f64) / (k_planned as f64));
    }

    // Step 5: convert each target to a parameter via param_from_arc_length.
    let mut u_breaks: Vec<f64> = targets.iter()
        .map(|&s| param_from_arc_length(&table_ref, s))
        .collect();

    // Step 6: epsilon-filter breakpoints (deferred until the carried-piece is known).
    // Step 7: walk the global-domain split pattern.
    let parent_pieces = extract_bezier_pieces_vector(&segment.xyz);
    debug_assert_eq!(parent_pieces.len(), 3, "expected D=3 axes");
    debug_assert!(
        parent_pieces.iter().all(|axis_pieces| axis_pieces.len() == 1),
        "single-piece-cubic invariant"
    );

    // Track the carried "current" piece per axis.
    let mut current_pieces: [BezierPiece<f64>; 3] = [
        parent_pieces[0][0].clone(),
        parent_pieces[1][0].clone(),
        parent_pieces[2][0].clone(),
    ];

    let mut emitted_axes: [Vec<BezierPiece<f64>>; 3] = Default::default();

    for &u in &u_breaks {
        // Epsilon filter against current piece bounds.
        let u_start = current_pieces[0].u_start;
        let u_end = current_pieces[0].u_end;
        if u <= u_start + EPS_U || u >= u_end - EPS_U {
            continue;
        }
        for axis in 0..3 {
            let (left, right) = split_piece_at(&current_pieces[axis], u);
            emitted_axes[axis].push(left);
            current_pieces[axis] = right;
        }
    }
    for axis in 0..3 {
        emitted_axes[axis].push(current_pieces[axis].clone());
    }

    let n_emitted = emitted_axes[0].len();

    // Step 8: wrap each emitted-piece-tuple into a CubicSegment with SplitInfo.
    let mut output: Vec<CubicSegment> = Vec::with_capacity(n_emitted);
    for i in 0..n_emitted {
        let xyz = vector_nurbs_from_pieces([
            &emitted_axes[0][i],
            &emitted_axes[1][i],
            &emitted_axes[2][i],
        ]);
        let s_lo = nurbs::arc_length::arc_length_from_param(&table_ref, emitted_axes[0][i].u_start);
        let s_hi = nurbs::arc_length::arc_length_from_param(&table_ref, emitted_axes[0][i].u_end);
        let split_info = SplitInfo {
            sub_index: i as u32,
            sub_count: n_emitted as u32,
            s_lo_mm: s_lo,
            s_hi_mm: s_hi,
        };
        let child = CubicSegment::try_new(
            xyz,
            segment.e_mode,
            segment.extrusion_per_xy_mm,
            segment.e_independent.clone(),
            segment.feedrate_mm_s,
            segment.source,
            Some(split_info),
        )
        .map_err(|_| SplitError::NotSinglePieceCubic)?;
        output.push(child);
    }
    Ok(output)
}

/// Check whether the segment has effectively no XYZ motion (retraction / prime /
/// degenerate). Returns true when the splitter should pass-through without building
/// an arc-length table.
fn is_zero_motion(xyz: &VectorNurbs<f64, 3>) -> bool {
    let cps = xyz.control_points();
    let cp_polygon_length = (1..4)
        .map(|i| {
            let dx = cps[i][0] - cps[i - 1][0];
            let dy = cps[i][1] - cps[i - 1][1];
            let dz = cps[i][2] - cps[i - 1][2];
            (dx * dx + dy * dy + dz * dz).sqrt()
        })
        .sum::<f64>();
    let mid_speed = midpoint_parametric_speed(xyz);
    cp_polygon_length < EPS_CP_POLYGON && mid_speed < MIN_PARAMETRIC_SPEED_FOR_SPLITTER
}

fn midpoint_parametric_speed(xyz: &VectorNurbs<f64, 3>) -> f64 {
    let deriv = vector_derivative(xyz);
    let d = nurbs::eval::vector_eval(&deriv, 0.5);
    (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
}

fn extract_bezier_pieces_vector(
    xyz: &VectorNurbs<f64, 3>,
) -> [Vec<BezierPiece<f64>>; 3] {
    let mut out: [Vec<BezierPiece<f64>>; 3] = Default::default();
    for axis in 0..3 {
        // Project onto axis, extract pieces.
        let scalar = project_axis_to_scalar(xyz, axis);
        out[axis] = extract_bezier_pieces(&scalar);
    }
    out
}

fn project_axis_to_scalar(xyz: &VectorNurbs<f64, 3>, axis: usize) -> nurbs::ScalarNurbs<f64> {
    let cps: Vec<f64> = xyz.control_points().iter().map(|cp| cp[axis]).collect();
    nurbs::ScalarNurbs::try_new(
        xyz.degree(),
        xyz.knots().to_vec(),
        cps,
        xyz.weights().map(<[f64]>::to_vec),
    )
    .expect("projection always valid")
}

fn vector_nurbs_from_pieces(pieces: [&BezierPiece<f64>; 3]) -> VectorNurbs<f64, 3> {
    // Convert each piece (Pascal-shifted-monomial) to Bernstein control points,
    // then construct a VectorNurbs with clamped knots [0,0,0,0,1,1,1,1] (after rescaling).
    debug_assert!(pieces.iter().all(|p| p.degree() == 3));
    debug_assert!(pieces.iter().all(|p| {
        (p.u_start - pieces[0].u_start).abs() < 1e-12
            && (p.u_end - pieces[0].u_end).abs() < 1e-12
    }));
    let bern_x = pieces[0].to_bernstein();
    let bern_y = pieces[1].to_bernstein();
    let bern_z = pieces[2].to_bernstein();
    let cps: Vec<[f64; 3]> = (0..4)
        .map(|i| [bern_x[i], bern_y[i], bern_z[i]])
        .collect();
    VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        cps,
        None,
    )
    .expect("valid cubic from pieces")
}
```

(`arc_length::arc_length_from_param` already exists per the inventory above.)

- [ ] **Step 4: Wire into `lib.rs`**

```rust
mod splitter;
pub use splitter::{SplitError, split_segment_to_cap};
```

- [ ] **Step 5: Run the tests**

```bash
cargo test -p geometry --test split_segment_to_cap
```

Expected: all 7 tests pass.

- [ ] **Step 6: Commit**

```bash
git add rust/geometry/src/splitter.rs rust/geometry/src/lib.rs rust/geometry/tests/split_segment_to_cap.rs
git commit -m "geometry/splitter: split_segment_to_cap path-length cap subdivision"
```

### Task 3.2: Closed-loop edge case test

**Files:**
- Modify: `rust/geometry/tests/split_segment_to_cap.rs`

- [ ] **Step 1: Add the closed-loop test**

```rust
#[test]
fn closed_loop_chord_zero_splits_by_arc_length() {
    // Cubic Bézier returning to its start point but with real arc length.
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [50.0, 50.0, 0.0],
            [-50.0, 50.0, 0.0],
            [0.0, 0.0, 0.0],
        ],
        None,
    ).unwrap();
    let seg = CubicSegment::try_new(
        xyz, EMode::Travel, 0.0, None, 100.0,
        SourceRange { start_line: 1, end_line: 1 }, None,
    ).unwrap();

    let out = split_segment_to_cap(&seg, 12.5).unwrap();
    assert!(out.len() > 1, "closed loop should split, not passthrough");
}
```

- [ ] **Step 2: Run**

```bash
cargo test -p geometry --test split_segment_to_cap
```

Expected: pass.

- [ ] **Step 3: Commit**

```bash
git add rust/geometry/tests/split_segment_to_cap.rs
git commit -m "geometry/splitter: closed-loop edge case test"
```

---

## Phase 4 — Test reorganization + integration

### Task 4.1: Gate legacy fixture tests

**Files:**
- Modify: `rust/geometry/tests/g5_reduction.rs`
- Modify: `rust/temporal/tests/multi_segment.rs` (and any others that consume legacy `Segment::Fitted` / `Segment::Arc`)

- [ ] **Step 1: Identify legacy-test files**

```bash
grep -rln "FittedSegment\|ArcSegment\|Segment::Fitted\|Segment::Arc" rust/{geometry,temporal}/tests/
```

- [ ] **Step 2: Gate them with `#![cfg(feature = "legacy-reference")]`** (file-level attribute at top):

For each identified test file, add at the top (after the `//!` doc-comment, if any):

```rust
#![cfg(feature = "legacy-reference")]
```

- [ ] **Step 3: For temporal tests that consume legacy types, also gate the `legacy-reference` feature in temporal's Cargo.toml**:

```toml
[features]
default = []
legacy-reference = ["geometry/legacy-reference"]
```

- [ ] **Step 4: Verify both configurations**

```bash
cargo test --workspace
cargo test --workspace --features geometry/legacy-reference
```

Default config: tests compile without the legacy fixtures.
Legacy config: legacy fixtures included and pass.

- [ ] **Step 5: Commit**

```bash
git add rust/geometry/tests/g5_reduction.rs rust/temporal/Cargo.toml rust/temporal/tests/multi_segment.rs
git commit -m "tests: gate G2/G3-legacy fixtures behind legacy-reference feature"
```

### Task 4.2: Add 7-pre integration sanity test

**Files:**
- Create: `rust/geometry/tests/integration_g5_only.rs`

- [ ] **Step 1: Write the integration test**

```rust
//! Synthetic G5 → reduce → split → Layer 2 sanity test.

use geometry::{GeometryPipeline, FitterParams, Item, Segment, split_segment_to_cap};

#[test]
fn synthetic_long_g5_reduces_splits_and_plans() {
    // 50 mm G5: from (0,0,0) to (50,0,0) via cubic Bézier.
    let g5_input = "G5 X50 Y0 I16.66 J0 P33.33 Q0 F1000\n";

    let mut pipeline = GeometryPipeline::new(FitterParams::default());
    let mut events = vec![];
    let mut sink = |evt| events.push(evt);

    let items: Vec<_> = pipeline.process(g5_input, &mut sink).collect();

    // Find the Cubic segment.
    let cubic = items.iter().find_map(|item| match item {
        Item::Segment(Segment::Cubic(c)) => Some(c.clone()),
        _ => None,
    }).expect("expected at least one Cubic segment");

    // Split it.
    let split = split_segment_to_cap(&cubic, 12.5).expect("split ok");
    assert!(split.len() >= 4, "50mm split into ≥4 sub-segments at 12.5mm cap");

    // Verify boundary continuity.
    use nurbs::eval::vector_eval;
    for w in split.windows(2) {
        let lend = vector_eval(&w[0].xyz, 1.0);
        let rstart = vector_eval(&w[1].xyz, 0.0);
        for axis in 0..3 {
            assert_eq!(lend[axis], rstart[axis], "boundary continuity on axis {axis}");
        }
    }
}
```

- [ ] **Step 2: Run**

```bash
cargo test -p geometry --test integration_g5_only
```

Expected: pass.

- [ ] **Step 3: Commit**

```bash
git add rust/geometry/tests/integration_g5_only.rs
git commit -m "geometry/tests: integration sanity G5 → reduce → split"
```

### Task 4.3: Final workspace verification

**Files:** none (verification step)

- [ ] **Step 1: Full workspace test, default features**

```bash
cargo test --workspace
```

Expected: all pass.

- [ ] **Step 2: Full workspace test, legacy-reference enabled**

```bash
cargo test --workspace --features geometry/legacy-reference
```

Expected: all pass (including legacy fixtures).

- [ ] **Step 3: Full clippy**

```bash
cargo clippy --workspace --all-targets -- -D warnings
cargo clippy --workspace --all-targets --features geometry/legacy-reference -- -D warnings
```

Both must be clean.

- [ ] **Step 4: Final commit message** (no code changes; this is a marker commit if anything was missed)

```bash
git status   # should be clean
```

If clean, no commit. If anything's left over, address and commit appropriately.

---

## Self-review checklist

Once Phase 4 is complete:

- [ ] **Spec coverage:** every spec section §4-§8 has at least one task implementing it.
- [ ] **No placeholders:** no "TBD" / "implement later" / "fill in" anywhere in the plan.
- [ ] **Type consistency:** `CubicSegment`, `EMode`, `SplitInfo`, `GeometryError`, `FitError`, `SplitError` are referenced consistently between tasks.
- [ ] **Test coverage:** invariant rejection, helical rejection, classification rules, fit primitive at multiple inputs, splitter at edge cases, integration sanity.
- [ ] **Both feature configurations green:** default and legacy-reference both pass cargo test + cargo clippy.

---

## Open implementation choice points (per spec §9)

These are for the implementer to decide; not blocking:

- **Q1**: `compose_vector_piece` lives in `nurbs::algebra` (chosen here) vs. new `nurbs::composition` module.
- **Q2**: Runtime-assertion of `CubicSegment` invariant at `try_new` only (chosen) vs. re-checking at every primitive entry.
- **Q3**: Disposition of legacy reduce code: gated in-place (chosen) vs. moved to a new `geometry-compat` crate (Step-13 future).
- **Q7**: Bernstein-coefficient certified L∞ bound on fit residual (post-MVP follow-up; not blocking).

---

## References

- Spec: `docs/superpowers/specs/2026-04-29-step7-pre-cubic-pipeline-prep-design.md`
- Plan-changes log: `docs/superpowers/plan-changes-log.md` (2026-04-29 round-1 + round-2 + 7-round-review-loop entries)
- Linear u(s) verifier artifact: `docs/research/linear-us-approximation-cubic-bezier-error.md`
- B-spline convolution research: `docs/research/bspline-polynomial-convolution.md`
- Piegl & Tiller, *The NURBS Book* (2nd ed., 1997), §5.2 (Bernstein convex-hull), §5.5 (degree elevation).
