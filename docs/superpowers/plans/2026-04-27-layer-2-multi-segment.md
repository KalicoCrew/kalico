# Layer 2 Multi-Segment Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the offline-batch multi-segment planner on top of Step 4's single-segment SOCP kernel — junction velocity from curvature continuity, forward/reverse joining with SOCP-per-iteration option (A), per-segment limits handling, adaptive N, 3-thread parallel batch executor.

**Architecture:** New `multi/` module under `rust/temporal/`. Public entry point `plan_batch(BatchInput) -> BatchOutput` is a function (no stateful object); takes a `Vec` of `(NurbsSegment, Limits)` plus a grid strategy + worker count, returns per-segment profiles + junction diagnostics. Joining computes junction velocities once, then iterates forward+reverse sweeps re-solving dirty segments via Step-4's `schedule_segment` until convergence. Adaptive-tolerance fallback added to `schedule_segment` for the cubic-class-fragility case.

**Tech Stack:** Rust 1.85, `nurbs` workspace crate (degree-3 NURBS evaluation + arclength), `geometry` workspace crate (G-code reduction), Step-4's `temporal::topp::schedule_segment` Clarabel-based SOCP, `std::thread` for parallelism (no rayon dep).

**Spec:** `docs/superpowers/specs/2026-04-27-layer-2-multi-segment-design.md`. **Read it before starting any task.** Key decisions are recorded there with rationale; this plan implements without re-litigating.

---

## Pre-Flight

Before Task 0: read the spec end-to-end. Particularly:
- §2.2 junction velocity formula (in particular the half-angle-identity numerical-safety note for the JD branch)
- §2.5 adaptive-N policy
- §3.2 public API surface (every type listed there must end up in `multi/mod.rs`)
- §6 acceptance criteria (each fixture's pass/fail spec)
- §7 risks (read all of these — implementation may surface edge cases the tests don't cover)
- The "Post-review revisions" sections at the end — they record bugs we already fixed in earlier drafts; don't re-introduce them

**Hard prerequisites (per review-1 of this plan):**

1. **Step 4 / Step 9 must be fully committed before starting Task 7.** Task 7 modifies `topp::mod::schedule_segment`'s public signature; the parallel Step-4 agent has been actively committing to `topp/mod.rs` and `topp/solver.rs`. Pre-flight check:
   ```bash
   git status   # rust/temporal/src/topp/* must NOT show as modified
   git log --oneline -5   # most recent commit on Step 4/9 should be present
   ```
   If `topp/mod.rs`, `topp/solver.rs`, or `tests/prototype.rs` shows uncommitted, stop and coordinate with the Step-4-agent session first. Optionally, run this plan in a worktree:
   ```bash
   git worktree add ../kalico-step-4.5 sota-motion
   cd ../kalico-step-4.5
   ```
2. **Verify the workspace builds clean before starting:**
   ```bash
   cd rust && cargo test -p temporal --release 2>&1 | tail -10
   ```
   Expected: all tests pass. If any fail, do not start — the failure is likely an in-flight test from Step 4/9 that needs to be resolved first.

---

### Task 0: NURBS API audit + `Limits` hardening (added per review-1)

**Files:**
- Read-only: `rust/nurbs/src/lib.rs`, `rust/nurbs/src/eval.rs`, `rust/nurbs/src/vector.rs`
- Modify: `rust/temporal/src/limits.rs`

**Why:** review-1 of this plan caught that the `derivative_at` API the original Task 3 assumed does not exist; the actual surface is `nurbs::eval::vector_derivative` (degree-lowering) + `nurbs::eval::vector_eval` (point evaluation on a view) + `nurbs::eval::curvature_from_derivs` (precomputed-derivative curvature). Lock the API choice up-front so Task 3 doesn't get blocked.

Also: `Limits` should be `#[non_exhaustive]` so Step 9 can additively add a shaper-aware acceleration constraint without breaking Step 4.5 callers (spec §7.3).

- [ ] **Step 1: Audit nurbs::eval public surface**

```bash
grep -nE "^pub fn" rust/nurbs/src/eval.rs
```

Expected to find at least:
- `pub fn vector_eval<T, V: VectorNurbsView<T, N>, const N>(curve: &V, u: T) -> [T; N]` (point evaluation on a view)
- `pub fn vector_derivative<T, const N>(curve: &VectorNurbs<T, N>) -> VectorNurbs<T, N>` (degree-lowering — returns a new owned NURBS of degree p-1)
- `pub fn curvature_from_derivs<T, const N>(first_deriv: &VectorNurbs, second_deriv: &VectorNurbs, u: T) -> T` (κ from precomputed derivatives)

If the API is materially different, **stop and update Task 3 of this plan** rather than guessing. Don't proceed until the API contract is locked.

- [ ] **Step 2: Make `Limits` `#[non_exhaustive]` + verify it derives `Copy`**

In `rust/temporal/src/limits.rs`:

```rust
//! Per-axis kinematic limits and centripetal cap. Pure data.
//!
//! Spec §4.4. Per-axis centripetal limits are deferred (§4.4 / §11).
//! `#[non_exhaustive]` per Step-4.5 spec §7.3: Step 9 will additively add
//! a shaper-aware acceleration constraint field.

#[non_exhaustive]
#[derive(Debug, Clone, Copy)]
pub struct Limits {
    /// Per-axis [X, Y, Z] velocity bound, mm/s.
    pub v_max: [f64; 3],
    /// Per-axis [X, Y, Z] acceleration bound, mm/s².
    pub a_max: [f64; 3],
    /// Per-axis [X, Y, Z] jerk bound, mm/s³.
    pub j_max: [f64; 3],
    /// Centripetal-acceleration cap, mm/s² (scalar; per-axis deferred).
    pub a_centripetal_max: f64,
}
```

Add `#[non_exhaustive]` if not already present. Confirm `derive(Copy)` is present (it should be — current code already has it).

**Internal-construction note:** `#[non_exhaustive]` forbids constructing `Limits { v_max: ..., ... }` from outside the `temporal` crate. All external callers must use a constructor or the `..` rest-syntax. Add a constructor for outside use:

```rust
impl Limits {
    /// Construct a `Limits` from all required fields. The struct is
    /// `#[non_exhaustive]` to allow Step 9 additive extension; external
    /// callers must use this constructor (or `..` rest-syntax inside the
    /// crate).
    #[must_use]
    pub fn new(
        v_max: [f64; 3],
        a_max: [f64; 3],
        j_max: [f64; 3],
        a_centripetal_max: f64,
    ) -> Self {
        Self { v_max, a_max, j_max, a_centripetal_max }
    }
}
```

- [ ] **Step 3: Verify all existing internal `Limits` literals still work**

```bash
cd rust && cargo build -p temporal --release 2>&1 | tail -3
```

Expected: clean build. `#[non_exhaustive]` permits literal construction inside the defining crate.

- [ ] **Step 4: Run existing tests**

```bash
cd rust && cargo test -p temporal --release 2>&1 | grep -E "test result|FAILED" | tail -5
```

Expected: all previously-passing tests still pass.

- [ ] **Step 5: Commit**

```bash
git add rust/temporal/src/limits.rs
git commit -m "temporal/limits: #[non_exhaustive] + new() constructor (spec §7.3)"
```

---

### Task 1: Scaffold the `multi/` module + public API types

**Files:**
- Create: `rust/temporal/src/multi/mod.rs`
- Modify: `rust/temporal/src/lib.rs:1-15` (add `pub mod multi;` + re-exports)

**Spec sections:** §3.1 module layout, §3.2 public API.

- [ ] **Step 1: Create empty multi/ module**

```bash
mkdir -p rust/temporal/src/multi
touch rust/temporal/src/multi/mod.rs
```

- [ ] **Step 2: Write the public-API type definitions in `multi/mod.rs`**

```rust
//! Layer 2 multi-segment integration. See spec
//! `docs/superpowers/specs/2026-04-27-layer-2-multi-segment-design.md`.

use crate::{Limits, TopProfile};
use nurbs::VectorNurbs;
use thiserror::Error;

#[non_exhaustive]
#[derive(Debug, Clone, Copy)]
pub enum GridStrategy {
    /// Fixed-N for every segment. Step 4 backward-compatible.
    Fixed(usize),
    /// Adaptive N per segment per spec §2.5.
    Adaptive {
        min_n: usize,
        max_n: usize,
        target_grid_spacing_mm: f64,
    },
}

pub struct SegmentInput<'a> {
    pub curve: &'a VectorNurbs<f64, 3>,
    pub limits: Limits,
    /// Per-junction chord-error tolerance for the *trailing* junction
    /// (between this segment and the next). Slicer-supplied for sharp
    /// G1↔G1 corners; ignored for smooth-κ junctions per spec §2.2.
    pub trailing_junction_chord_tolerance_mm: f64,
}

pub struct BatchInput<'a> {
    pub segments: &'a [SegmentInput<'a>],
    pub grid_strategy: GridStrategy,
    /// Default 3 on Pi 5 per spec §2.6 (avoids Klipper contention on cores 0-1).
    pub worker_threads: usize,
}

pub struct BatchOutput {
    pub profiles: Vec<TopProfile>,
    pub junctions: Vec<JunctionInfo>,
    pub joining_sweeps: u32,
    pub joining_status: JoiningStatus,
}

#[non_exhaustive]
#[derive(Debug, Clone, Copy)]
pub enum JoiningStatus {
    Converged,
    CappedAtMaxSweeps { last_dirty_count: usize },
}

#[derive(Debug, Clone, Copy)]
pub struct JunctionInfo {
    /// Indices of the two segments this junction sits between.
    pub between_segments: (usize, usize),
    pub v_junction: f64,
    pub binding_cap: JunctionBindingCap,
    pub kappa_left: f64,
    pub kappa_right: f64,
}

#[non_exhaustive]
#[derive(Debug, Clone, Copy)]
pub enum JunctionBindingCap {
    PerAxisVelocity,
    Centripetal,
    GlobalVMax,
    SharpCornerChord,
}

#[derive(Debug, Error)]
pub enum BatchError {
    #[error("empty segment buffer")]
    EmptySegments,
    #[error("worker_threads must be ≥ 1")]
    InvalidThreads,
    #[error("segment {0}: {1}")]
    Segment(usize, crate::topp::ScheduleError),
}

// Stub — real implementation in Task 9.
pub fn plan_batch(_input: BatchInput<'_>) -> Result<BatchOutput, BatchError> {
    unimplemented!("plan_batch lands in Task 9")
}

mod grid;
mod junction;
mod joining;
mod parallel;
```

- [ ] **Step 3: Wire `multi` into `lib.rs`**

```rust
// In rust/temporal/src/lib.rs, near the top with other module decls:
pub mod multi;

// And re-export the public surface:
pub use multi::{
    plan_batch, BatchError, BatchInput, BatchOutput, GridStrategy,
    JoiningStatus, JunctionBindingCap, JunctionInfo, SegmentInput,
};
```

- [ ] **Step 4: Create empty placeholder files for the submodules**

```bash
touch rust/temporal/src/multi/grid.rs
touch rust/temporal/src/multi/junction.rs
touch rust/temporal/src/multi/joining.rs
touch rust/temporal/src/multi/parallel.rs
```

Each file gets a one-line module doc to silence missing-docs warnings:

```rust
//! Stub — real implementation lands in subsequent tasks.
```

- [ ] **Step 5: Build and verify scaffolding compiles**

```bash
cd rust && cargo build -p temporal --release 2>&1 | tail -3
```

Expected: `Finished release profile [optimized + debuginfo] target(s)`. Warnings about unused imports / dead code are OK (we'll fill them in).

- [ ] **Step 6: Commit**

```bash
git add rust/temporal/src/multi/ rust/temporal/src/lib.rs
git commit -m "temporal/multi: scaffold module + public API types (spec §3.1, §3.2)"
```

---

### Task 2: Adaptive-N grid policy (`multi::grid`)

**Files:**
- Modify: `rust/temporal/src/multi/grid.rs`

**Spec sections:** §2.5 adaptive-N policy.

- [ ] **Step 1: Write the failing test for `Fixed` strategy**

In `rust/temporal/src/multi/grid.rs`:

```rust
//! Adaptive-N policy per spec §2.5.

use crate::multi::GridStrategy;
use nurbs::VectorNurbs;

pub(crate) fn compute_n(strategy: &GridStrategy, curve: &VectorNurbs<f64, 3>) -> usize {
    match *strategy {
        GridStrategy::Fixed(n) => n,
        GridStrategy::Adaptive { min_n, max_n, target_grid_spacing_mm } => {
            let l = arclength_mm(curve);
            let n = (l / target_grid_spacing_mm).ceil() as usize;
            n.clamp(min_n, max_n)
        }
    }
}

/// Approximate arclength via control-polygon length (cheap upper-bound estimate;
/// exact arclength would require Layer 0's quadrature which we don't need at this
/// granularity — the policy clamps anyway).
fn arclength_mm(curve: &VectorNurbs<f64, 3>) -> f64 {
    let cps = curve.control_points();
    cps.windows(2)
        .map(|w| {
            let dx = w[1][0] - w[0][0];
            let dy = w[1][1] - w[0][1];
            let dz = w[1][2] - w[0][2];
            (dx*dx + dy*dy + dz*dz).sqrt()
        })
        .sum()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn straight_100mm() -> VectorNurbs<f64, 3> {
        VectorNurbs::<f64, 3>::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![[0.0, 0.0, 0.0], [100.0, 0.0, 0.0]],
            None,
        ).unwrap()
    }

    #[test]
    fn fixed_strategy_returns_n_unchanged() {
        let curve = straight_100mm();
        assert_eq!(compute_n(&GridStrategy::Fixed(50), &curve), 50);
        assert_eq!(compute_n(&GridStrategy::Fixed(200), &curve), 200);
    }
}
```

- [ ] **Step 2: Run test to verify it passes**

```bash
cd rust && cargo test -p temporal --release multi::grid::tests::fixed_strategy 2>&1 | tail -5
```

Expected: `test result: ok. 1 passed`.

- [ ] **Step 3: Add the adaptive-N test case (1mm segment → MIN_N=10)**

Add to the same `tests` module:

```rust
#[test]
fn adaptive_short_segment_floors_to_min_n() {
    let curve = VectorNurbs::<f64, 3>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],  // 1mm
        None,
    ).unwrap();
    let strategy = GridStrategy::Adaptive {
        min_n: 10,
        max_n: 200,
        target_grid_spacing_mm: 0.5,
    };
    // 1mm / 0.5mm = 2; clamped to MIN_N = 10.
    assert_eq!(compute_n(&strategy, &curve), 10);
}
```

- [ ] **Step 4: Run; expect pass**

```bash
cd rust && cargo test -p temporal --release multi::grid::tests::adaptive_short 2>&1 | tail -5
```

Expected: PASS.

- [ ] **Step 5: Add adaptive-N test for typical (50mm) and ceiling (200mm) cases**

```rust
#[test]
fn adaptive_typical_segment_scales_with_arclength() {
    let curve_50 = VectorNurbs::<f64, 3>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [50.0, 0.0, 0.0]],
        None,
    ).unwrap();
    let strategy = GridStrategy::Adaptive {
        min_n: 10,
        max_n: 200,
        target_grid_spacing_mm: 0.5,
    };
    // 50mm / 0.5mm = 100.
    assert_eq!(compute_n(&strategy, &curve_50), 100);
}

#[test]
fn adaptive_long_segment_caps_to_max_n() {
    let curve_200mm = VectorNurbs::<f64, 3>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [200.0, 0.0, 0.0]],
        None,
    ).unwrap();
    let strategy = GridStrategy::Adaptive {
        min_n: 10,
        max_n: 200,
        target_grid_spacing_mm: 0.5,
    };
    // 200mm / 0.5mm = 400; clamped to MAX_N = 200.
    assert_eq!(compute_n(&strategy, &curve_200mm), 200);
}
```

- [ ] **Step 6: Run all grid tests**

```bash
cd rust && cargo test -p temporal --release multi::grid 2>&1 | tail -5
```

Expected: 4 passed.

- [ ] **Step 7: Commit**

```bash
git add rust/temporal/src/multi/grid.rs
git commit -m "temporal/multi/grid: adaptive-N policy v1 (spec §2.5)"
```

---

### Task 3: Junction velocity computation (`multi::junction`)

**Files:**
- Modify: `rust/temporal/src/multi/junction.rs`

**Spec sections:** §2.2 (the unified centripetal-against-curvature formula + JD sub-case + numerical-safety note).

**Reference:** `docs/research/junction-deviation-cornering-formula.md` for formula derivation. **Read the half-angle-identity safety note in spec §2.2 carefully** — implementations that compose `arccos(dot)` then `cos(α/2)` will hit `NaN` on tangent-normalize ULP overshoot. Use the direct half-angle form.

- [ ] **Step 1: Write the JD-formula test for collinear (must NOT brake)**

In `rust/temporal/src/multi/junction.rs`:

```rust
//! Junction velocity from curvature continuity. Per spec §2.2.

use crate::multi::JunctionBindingCap;
use crate::Limits;
use nurbs::VectorNurbs;

/// Numerical floor on κ — below this the centripetal cap is treated as ∞ and
/// we fall back to the JD sharp-corner sub-case. Per spec §2.2.
const KAPPA_FLOOR: f64 = 1e-12;
/// Numerical ceiling on `b = ṡ²` for "no centripetal cap" cases. Per spec §2.2
/// + matching `constraints.rs::B_MAX_CENT_CAP`. ~10⁴ mm/s.
const B_MAX_CENT_CAP: f64 = 1e8;
/// Threshold below which the JD branch returns ∞ (no corner cap). Per spec §2.2.
const ALPHA_COLLINEAR_THRESHOLD: f64 = 1e-3;
/// Threshold above which the JD branch caps v_jd at a small positive floor
/// (avoid exact-zero boundary conditions confusing downstream solver).
const ALPHA_REVERSAL_THRESHOLD: f64 = std::f64::consts::PI * 0.99;
/// Floor v_jd at this value at near-reversal junctions. Per spec §2.2.
const V_JD_REVERSAL_FLOOR_MM_S: f64 = 1.0;

pub(crate) fn compute_junction_velocity(
    left: &VectorNurbs<f64, 3>,
    right: &VectorNurbs<f64, 3>,
    left_limits: &Limits,
    right_limits: &Limits,
    chord_tolerance_mm: f64,
) -> JunctionResult {
    let t_left = forward_unit_tangent_at_end(left);
    let t_right = forward_unit_tangent_at_start(right);

    let kappa_left = curvature_at_end(left);
    let kappa_right = curvature_at_start(right);

    // Cap 1: per-axis MVC from tangent direction at junction.
    // Since |t| = 1, |dx_axis/dt| = |t_axis| · |ṡ|, so |ṡ| ≤ v_max,axis / |t_axis|.
    // Apply on both sides; take the more-restrictive limits.
    let cap_per_axis = per_axis_velocity_cap(&t_left, left_limits)
        .min(per_axis_velocity_cap(&t_right, right_limits));

    // Cap 2: centripetal cap.
    let cap_centripetal = centripetal_cap(kappa_left, left_limits)
        .min(centripetal_cap(kappa_right, right_limits));

    // Cap 3: sharp-corner JD when both sides are below the κ floor.
    let cap_sharp = if kappa_left.abs() <= KAPPA_FLOOR && kappa_right.abs() <= KAPPA_FLOOR {
        sharp_corner_jd_cap(&t_left, &t_right, &left_limits, chord_tolerance_mm)
    } else {
        f64::INFINITY
    };

    // Cap 4: global per-axis v_max (each axis independently).
    let cap_v_max = left_limits.v_max.iter().chain(right_limits.v_max.iter())
        .copied().fold(f64::INFINITY, f64::min);

    // Take the minimum and tag which cap was binding.
    let (v, binding) = min_with_tag([
        (cap_per_axis, JunctionBindingCap::PerAxisVelocity),
        (cap_centripetal, JunctionBindingCap::Centripetal),
        (cap_sharp, JunctionBindingCap::SharpCornerChord),
        (cap_v_max, JunctionBindingCap::GlobalVMax),
    ]);

    JunctionResult {
        v_junction: v,
        binding_cap: binding,
        kappa_left,
        kappa_right,
    }
}

pub(crate) struct JunctionResult {
    pub v_junction: f64,
    pub binding_cap: JunctionBindingCap,
    pub kappa_left: f64,
    pub kappa_right: f64,
}

fn per_axis_velocity_cap(t: &[f64; 3], limits: &Limits) -> f64 {
    let mut cap = f64::INFINITY;
    for axis in 0..3 {
        let t_abs = t[axis].abs();
        if t_abs > 1e-12 {
            cap = cap.min(limits.v_max[axis] / t_abs);
        }
    }
    cap
}

fn centripetal_cap(kappa: f64, limits: &Limits) -> f64 {
    let k = kappa.abs();
    if k <= KAPPA_FLOOR {
        B_MAX_CENT_CAP.sqrt()
    } else {
        (limits.a_centripetal_max / k).sqrt()
    }
}

/// Sharp-corner JD cap. Per spec §2.2 sharp-corner sub-case.
///
/// Uses the deviation-angle convention (α = 0 collinear, α = π reversal) and
/// computes `cos(α/2)` directly via the half-angle identity to avoid the
/// `arccos(dot)`-then-`cos(α/2)` NaN trap in f64 (see spec §2.2 numerical-safety
/// note + docs/research/junction-deviation-cornering-formula.md).
fn sharp_corner_jd_cap(
    t_left: &[f64; 3],
    t_right: &[f64; 3],
    limits: &Limits,
    chord_tolerance_mm: f64,
) -> f64 {
    let dot = (t_left[0] * t_right[0] + t_left[1] * t_right[1] + t_left[2] * t_right[2])
        .clamp(-1.0, 1.0);
    // Half-angle identity: cos(α/2) = sqrt((1 + dot)/2). Always non-negative.
    let cos_half_alpha = ((1.0 + dot) * 0.5).max(0.0).sqrt();
    // Compute α only for the threshold checks (stable form via asin half-angle).
    let sin_half_alpha = ((1.0 - dot) * 0.5).max(0.0).sqrt();
    let alpha = 2.0 * sin_half_alpha.asin();
    if alpha <= ALPHA_COLLINEAR_THRESHOLD {
        return B_MAX_CENT_CAP.sqrt();
    }
    if alpha >= ALPHA_REVERSAL_THRESHOLD {
        return V_JD_REVERSAL_FLOOR_MM_S;
    }
    // v_jd² = a · δ · cos(α/2) / (1 − cos(α/2))
    let denom = 1.0 - cos_half_alpha;
    if denom <= 1e-15 {
        return B_MAX_CENT_CAP.sqrt();
    }
    (limits.a_centripetal_max * chord_tolerance_mm * cos_half_alpha / denom).sqrt()
}

fn min_with_tag(caps: [(f64, JunctionBindingCap); 4]) -> (f64, JunctionBindingCap) {
    let mut best = caps[0];
    for &(v, tag) in &caps[1..] {
        if v < best.0 {
            best = (v, tag);
        }
    }
    best
}

// Forward-tangent + curvature evaluation. Stub for now — wire to nurbs Layer 0
// after we know the API. Will be filled in Step 5.
fn forward_unit_tangent_at_end(_curve: &VectorNurbs<f64, 3>) -> [f64; 3] {
    todo!("Layer 0 NURBS tangent evaluation — wire in Step 5")
}
fn forward_unit_tangent_at_start(_curve: &VectorNurbs<f64, 3>) -> [f64; 3] {
    todo!("Layer 0 NURBS tangent evaluation — wire in Step 5")
}
fn curvature_at_end(_curve: &VectorNurbs<f64, 3>) -> f64 {
    todo!("Layer 0 NURBS curvature evaluation — wire in Step 5")
}
fn curvature_at_start(_curve: &VectorNurbs<f64, 3>) -> f64 {
    todo!("Layer 0 NURBS curvature evaluation — wire in Step 5")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn textbook_limits() -> Limits {
        Limits {
            v_max: [500.0, 500.0, 500.0],
            a_max: [5_000.0, 5_000.0, 5_000.0],
            j_max: [100_000.0, 100_000.0, 100_000.0],
            a_centripetal_max: 2_500.0,
        }
    }

    #[test]
    fn jd_collinear_no_cap() {
        let t_x = [1.0, 0.0, 0.0];
        let cap = sharp_corner_jd_cap(&t_x, &t_x, &textbook_limits(), 0.05);
        // Collinear should give ∞ (or B_MAX_CENT_CAP.sqrt() = 10000 mm/s).
        assert!(cap >= 9999.9, "collinear should give ~10000 mm/s cap, got {cap}");
    }
}
```

- [ ] **Step 2: Run; expect collinear test pass (the only one with non-todo!() helpers)**

```bash
cd rust && cargo test -p temporal --release multi::junction::tests::jd_collinear 2>&1 | tail -5
```

Expected: PASS.

- [ ] **Step 3: Add JD test for 90° corner**

```rust
#[test]
fn jd_90_degree_corner_matches_klipper() {
    let t_x = [1.0, 0.0, 0.0];
    let t_y = [0.0, 1.0, 0.0];
    let limits = textbook_limits();
    // a · δ = 2500 · 0.05 = 125. v² = 125 · 2.414 = 301.75. v = 17.37 mm/s.
    let cap = sharp_corner_jd_cap(&t_x, &t_y, &limits, 0.05);
    let expected = (limits.a_centripetal_max * 0.05 * 2.414213562).sqrt();
    assert!(
        (cap - expected).abs() < 0.05,
        "90° JD: got {cap}, expected ~{expected}",
    );
}
```

- [ ] **Step 4: Run; expect pass**

```bash
cd rust && cargo test -p temporal --release multi::junction::tests::jd_90 2>&1 | tail -5
```

Expected: PASS within ε.

- [ ] **Step 5: Wire Layer-0 NURBS tangent + curvature evaluation (using audited nurbs::eval API)**

Per Task 0's audit: actual API is `nurbs::eval::vector_derivative` (degree-lowering — returns a new NURBS of degree p-1) + `nurbs::eval::vector_eval` (point evaluation on a view) + `nurbs::eval::curvature_from_derivs`. Replace the four `todo!()` stubs:

```rust
use nurbs::eval::{vector_derivative, vector_eval, curvature_from_derivs};

fn forward_unit_tangent_at_end(curve: &VectorNurbs<f64, 3>) -> [f64; 3] {
    let u_end = curve.knots()[curve.knots().len() - 1];
    let d1 = vector_derivative(curve);
    let t = vector_eval(&d1.as_view(), u_end);
    normalize_3(t)
}

fn forward_unit_tangent_at_start(curve: &VectorNurbs<f64, 3>) -> [f64; 3] {
    let u_start = curve.knots()[0];
    let d1 = vector_derivative(curve);
    let t = vector_eval(&d1.as_view(), u_start);
    normalize_3(t)
}

fn curvature_at_end(curve: &VectorNurbs<f64, 3>) -> f64 {
    if curve.degree() < 2 {
        return 0.0;  // degree-1 NURBS (G1 segment) has zero curvature.
    }
    let u_end = curve.knots()[curve.knots().len() - 1];
    let d1 = vector_derivative(curve);
    let d2 = vector_derivative(&d1);
    curvature_from_derivs(&d1, &d2, u_end)
}

fn curvature_at_start(curve: &VectorNurbs<f64, 3>) -> f64 {
    if curve.degree() < 2 {
        return 0.0;
    }
    let u_start = curve.knots()[0];
    let d1 = vector_derivative(curve);
    let d2 = vector_derivative(&d1);
    curvature_from_derivs(&d1, &d2, u_start)
}

#[inline]
fn normalize_3(v: [f64; 3]) -> [f64; 3] {
    let m = (v[0]*v[0] + v[1]*v[1] + v[2]*v[2]).sqrt();
    if m < 1e-12 {
        [0.0; 3]
    } else {
        [v[0]/m, v[1]/m, v[2]/m]
    }
}
```

**Implementation efficiency note:** computing `d1` and `d2` per-junction is O(p) work each via the degree-lowering algorithm — fine for the ~1 junction-velocity-call-per-segment pattern. A future optimization could cache `(d1, d2)` per segment in `SegmentInput`, but unnecessary for Step 4.5 prototype.

**Degree-1 special case:** G1 segments have zero curvature by definition. The `degree() < 2` guard avoids calling `vector_derivative` on a degree-0 result (which would have no derivative). For G1↔G1 junctions both sides report κ=0, the JD sharp-corner sub-case fires correctly.

- [ ] **Step 6: Add end-to-end test using two G1 segments**

```rust
#[test]
fn compute_junction_velocity_g1_to_g1_90deg() {
    let left = VectorNurbs::<f64, 3>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [50.0, 0.0, 0.0]],
        None,
    ).unwrap();
    let right = VectorNurbs::<f64, 3>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[50.0, 0.0, 0.0], [50.0, 50.0, 0.0]],
        None,
    ).unwrap();
    let limits = textbook_limits();
    let result = compute_junction_velocity(&left, &right, &limits, &limits, 0.05);
    let expected = (limits.a_centripetal_max * 0.05 * 2.414213562).sqrt();
    assert!(
        (result.v_junction - expected).abs() < 0.05,
        "got {}, expected ~{}", result.v_junction, expected
    );
    assert!(matches!(result.binding_cap, JunctionBindingCap::SharpCornerChord));
}
```

- [ ] **Step 7: Run; verify all junction tests pass**

```bash
cd rust && cargo test -p temporal --release multi::junction 2>&1 | tail -5
```

Expected: 3+ passed.

- [ ] **Step 8: Commit**

```bash
git add rust/temporal/src/multi/junction.rs
git commit -m "temporal/multi/junction: unified centripetal+JD junction-velocity (spec §2.2)"
```

---

### Task 4: Forward sweep (`multi::joining::forward_sweep`)

**Files:**
- Modify: `rust/temporal/src/multi/joining.rs`

**Spec sections:** §2.3 lookahead-window joining.

- [ ] **Step 1: Write the failing test for accel-feasibility-bounded forward propagation**

In `rust/temporal/src/multi/joining.rs`:

```rust
//! Lookahead joining via SOCP-per-iteration (option A). Spec §2.3.

use crate::multi::junction::JunctionResult;
use crate::TopProfile;

/// Per-segment scratch state during joining.
pub(crate) struct SegmentState {
    pub v_start: f64,
    pub v_end: f64,
    pub profile: Option<TopProfile>,
    pub dirty: bool,
}

/// Propagate junction velocities forward, marking dirty any segment whose
/// `v_start` changed beyond `EPS_VEL` since the last forward sweep.
pub(crate) fn forward_sweep(
    states: &mut [SegmentState],
    junctions: &[JunctionResult],
) -> usize {
    const EPS_VEL: f64 = 1e-3;
    let mut dirty_count = 0;
    for k in 1..states.len() {
        let proposed_v_start = junctions[k - 1].v_junction.min(states[k - 1].v_end);
        if (proposed_v_start - states[k].v_start).abs() > EPS_VEL {
            states[k].v_start = proposed_v_start;
            states[k].dirty = true;
            dirty_count += 1;
        }
    }
    dirty_count
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::multi::JunctionBindingCap;

    fn make_state(v_start: f64, v_end: f64) -> SegmentState {
        SegmentState { v_start, v_end, profile: None, dirty: false }
    }

    fn make_junction(v: f64) -> JunctionResult {
        JunctionResult {
            v_junction: v,
            binding_cap: JunctionBindingCap::Centripetal,
            kappa_left: 0.0,
            kappa_right: 0.0,
        }
    }

    #[test]
    fn forward_propagates_v_end_to_next_v_start() {
        let mut states = vec![
            make_state(0.0, 100.0),
            make_state(0.0, 200.0),
        ];
        let junctions = vec![make_junction(150.0)];
        let dirty = forward_sweep(&mut states, &junctions);
        // junctions[0] = 150, states[0].v_end = 100; min = 100. New v_start[1] = 100.
        assert_eq!(dirty, 1);
        assert!((states[1].v_start - 100.0).abs() < 1e-6);
        assert!(states[1].dirty);
    }
}
```

- [ ] **Step 2: Run; expect pass**

```bash
cd rust && cargo test -p temporal --release multi::joining::tests::forward_propagates 2>&1 | tail -5
```

Expected: PASS.

- [ ] **Step 3: Add test for "no-change" case (no dirty segments)**

```rust
#[test]
fn forward_no_change_no_dirty() {
    let mut states = vec![
        make_state(0.0, 150.0),
        make_state(150.0, 200.0),
    ];
    let junctions = vec![make_junction(150.0)];
    let dirty = forward_sweep(&mut states, &junctions);
    assert_eq!(dirty, 0);
    assert!(!states[1].dirty);
}
```

- [ ] **Step 4: Run; expect pass**

```bash
cd rust && cargo test -p temporal --release multi::joining::tests 2>&1 | tail -5
```

Expected: 2 passed.

- [ ] **Step 5: Commit**

```bash
git add rust/temporal/src/multi/joining.rs
git commit -m "temporal/multi/joining: forward sweep (spec §2.3)"
```

---

### Task 5: Reverse sweep (`multi::joining::reverse_sweep`)

**Files:**
- Modify: `rust/temporal/src/multi/joining.rs`

**Spec sections:** §2.3 lookahead-window joining (reverse pass).

- [ ] **Step 1: Add reverse_sweep function**

```rust
/// Propagate junction velocities backward, marking dirty any segment whose
/// `v_end` changed beyond `EPS_VEL` since the last reverse sweep.
pub(crate) fn reverse_sweep(
    states: &mut [SegmentState],
    junctions: &[JunctionResult],
) -> usize {
    const EPS_VEL: f64 = 1e-3;
    let mut dirty_count = 0;
    for k in (0..states.len() - 1).rev() {
        let proposed_v_end = junctions[k].v_junction.min(states[k + 1].v_start);
        if (proposed_v_end - states[k].v_end).abs() > EPS_VEL {
            states[k].v_end = proposed_v_end;
            states[k].dirty = true;
            dirty_count += 1;
        }
    }
    dirty_count
}
```

- [ ] **Step 2: Add test**

```rust
#[test]
fn reverse_propagates_v_start_to_prev_v_end() {
    let mut states = vec![
        make_state(0.0, 200.0),
        make_state(100.0, 200.0),
    ];
    let junctions = vec![make_junction(150.0)];
    let dirty = reverse_sweep(&mut states, &junctions);
    // junctions[0] = 150, states[1].v_start = 100; min = 100. New v_end[0] = 100.
    assert_eq!(dirty, 1);
    assert!((states[0].v_end - 100.0).abs() < 1e-6);
}
```

- [ ] **Step 3: Run**

```bash
cd rust && cargo test -p temporal --release multi::joining::tests::reverse 2>&1 | tail -5
```

Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add rust/temporal/src/multi/joining.rs
git commit -m "temporal/multi/joining: reverse sweep (spec §2.3)"
```

---

### Task 6: Joining convergence loop

**Files:**
- Modify: `rust/temporal/src/multi/joining.rs`

**Spec sections:** §2.3 (convergence loop), §6.5 (acceptance ≤3 sweeps for normal fixtures, ≤5 for star).

- [ ] **Step 1: Add the convergence loop with in-loop re-solves (corrected per review-1)**

The convergence loop **must invoke `fan_out_solves` between sweeps**, not just at the end. Spec §2.3 says: "if `v_start_proposed[k]` is higher than the SOCP can absorb given the desired `v_end`, mark the segment dirty and recompute via `schedule_segment`." Original draft of this task only ran sweeps without re-solves, which would converge on stale velocity caps + leave profiles inconsistent with the final boundary velocities (review-1 finding F8 / Codex finding I).

```rust
use crate::multi::{BatchError, JoiningStatus, SegmentInput};
use crate::multi::parallel::fan_out_solves;
use crate::GridConfig;

/// Hard cap on joining sweeps. Per spec §2.3 + §6.5: typical convergence is
/// 1–3 sweeps; cap at 10 to detect bugs.
const MAX_SWEEPS: u32 = 10;

pub(crate) fn join_until_converged(
    inputs: &[SegmentInput<'_>],
    grids: &[GridConfig],
    states: &mut [SegmentState],
    junctions: &[JunctionResult],
    n_threads: usize,
) -> Result<(u32, JoiningStatus), BatchError> {
    for sweep in 1..=MAX_SWEEPS {
        let f_dirty = forward_sweep(states, junctions);
        let r_dirty = reverse_sweep(states, junctions);
        if f_dirty == 0 && r_dirty == 0 {
            // Velocity propagation has stabilized — no segment's joining-decided
            // (v_start, v_end) changed in either sweep direction. Three cases:
            if states.iter().all(|s| !s.dirty) {
                // (1) All segments solved cleanly. Done.
                return Ok((sweep, JoiningStatus::Converged));
            }
            // (2) Some segments still have dirty=true because their last
            // fan_out_solves returned a non-success status (Infeasible /
            // MaxIter / DivergedSlp / MaxIterSlp — all of which return
            // Ok(profile) with non-success SolveStatus, leaving dirty=true).
            // Per kalico-verifier review-3: schedule_segment is deterministic
            // (Clarabel 0.11.1 with kalico's default-features uses single-
            // threaded QDLDL; SLP loops have no RNG; constraint construction
            // is deterministic), so re-solving with unchanged inputs would
            // produce the same non-success status. Bail early rather than
            // spin to MAX_SWEEPS.
            let last_dirty_count = states.iter().filter(|s| s.dirty).count();
            return Ok((sweep, JoiningStatus::CappedAtMaxSweeps { last_dirty_count }));
        }
        fan_out_solves(inputs, states, grids, n_threads)?;
    }
    // (3) Reached MAX_SWEEPS without velocity stabilization — pathological
    // joining behavior (shouldn't happen on the test fixtures; surfaces as
    // CappedAtMaxSweeps for diagnostic).
    let last_dirty = states.iter().filter(|s| s.dirty).count();
    Ok((MAX_SWEEPS, JoiningStatus::CappedAtMaxSweeps { last_dirty_count: last_dirty }))
}
```

**Why the re-solve goes inside the loop:** when forward/reverse sweep changes `v_start[k]` or `v_end[k]`, segment k's existing `profile` is stale relative to the new boundary conditions. The next sweep iteration would propagate from `state[k].v_end` — but that field reflects the *previously-solved* SOCP output. To get a true convergence, we must re-solve stale segments before re-checking propagation. (For the simple case where the initial junction-velocity seeds are already feasible everywhere, the loop terminates in 1 iteration with no re-solves — same outcome as the original buggy version, but correctness preserved on the harder cases.)

**Note on `fan_out_solves` dirty-clearing semantics:** Task 8 must clear `dirty` only on the verifier-feasible public success statuses — `SolveStatus::Solved`, `SolveStatus::SolvedInexact`, AND `SolveStatus::SolvedSlp` (kalico-verifier round-3 confirmed: SolvedSlp passes the same verifier::check feasibility gate as the others). Failures (`MaxIter`, `Infeasible`, `DivergedSlp`, `MaxIterSlp`) leave dirty=true. Codex review-1 found that returning `Ok(profile)` from `schedule_segment` doesn't imply the profile is feasible — non-success statuses still construct a `TopProfile`. Task 8 implementation must inspect `profile.status` and propagate failures.

- [ ] **Step 2: Add test — converges-in-one-sweep on consistent input + signature change**

```rust
#[test]
fn converges_in_one_sweep_on_already_consistent() {
    // Stub test — full plan_batch test in Task 9. Direct join_until_converged
    // requires SegmentInput + GridConfig setup, which is integration-test scope.
    // Unit-test path: assert forward_sweep + reverse_sweep both no-op on a
    // pre-balanced state.
    let mut states = vec![
        make_state(0.0, 150.0),
        make_state(150.0, 200.0),
    ];
    let junctions = vec![make_junction(150.0)];
    let f_dirty = forward_sweep(&mut states, &junctions);
    let r_dirty = reverse_sweep(&mut states, &junctions);
    assert_eq!(f_dirty, 0);
    assert_eq!(r_dirty, 0);
    // join_until_converged would return Converged in one sweep with no re-solves.
}
```

(The full join_until_converged integration test moves to Task 9; this unit test only exercises the sweep helpers, which is what `joining.rs` exposes for unit-testing.)

- [ ] **Step 3: Run**

```bash
cd rust && cargo test -p temporal --release multi::joining::tests 2>&1 | tail -5
```

Expected: 3 passed (forward_propagates, reverse_propagates, converges_in_one_sweep_on_already_consistent).

- [ ] **Step 4: Commit**

```bash
git add rust/temporal/src/multi/joining.rs
git commit -m "temporal/multi/joining: convergence loop with in-loop re-solve (spec §2.3, §6.5; review-1 fix)"
```

---

### Task 7: Adaptive-tolerance fallback API — backward-compatible wrapper (revised per review-1)

**Files:**
- Modify: `rust/temporal/src/topp/mod.rs` — add `ToleranceMode` enum + `schedule_segment_with_tolerance` wrapper
- Modify: `rust/temporal/src/topp/solver.rs` — plumb `tol` through `slp_solve_with_axis_jerk` (the actual entry called by `schedule_segment`) AND every Clarabel `DefaultSettings` construction site
- Modify: `rust/temporal/src/lib.rs` — re-export `ToleranceMode`

**Spec sections:** §2.1 (option A); the Pi 5 throughput investigation `Recommendation for Step 4.5` item 2 + Finding 2 SAFETY UPDATE.

**Hard prerequisite:** Step 4 / Step 9 fully committed before starting this task. See Pre-Flight #1.

**Why this isn't optional:** without it, `plan_batch` either (a) uses default 1e-8 tolerance everywhere and burns ~10× the planning latency on the regime where 1e-5 is safe, or (b) uses 1e-5 unconditionally and breaks fixture 4 (G5 cubic with non-zero endpoint κ). Adaptive fallback gets us the 11×-on-bench speedup on the convergent regime + correctness on the fragile regime.

**Backward-compatibility approach** (per Codex review-1): rather than mutate the existing `schedule_segment` signature, add a new `schedule_segment_with_tolerance(...)` that takes a `ToleranceMode`, and have `schedule_segment(...)` delegate with `ToleranceMode::Tight` (preserving existing behavior). This avoids touching every existing test caller and reduces merge-conflict risk with the parallel Step 4/9 work.

- [ ] **Step 1: Add `ToleranceMode` enum to `topp::mod`**

In `rust/temporal/src/topp/mod.rs`, near the top:

```rust
/// Solver tolerance strategy. Per Pi 5 throughput investigation Finding 2 +
/// Step-4.5 spec §2.1 (Codex review-1 corrected version).
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ToleranceMode {
    /// Default Clarabel tolerances (1e-8). Always safe, slowest.
    Tight,
    /// Loosened tolerances (1e-5). ~11× faster on convergent geometry.
    /// May trigger DivergedSlp on fragile geometry (G5 cubic with endpoint κ).
    Fast,
    /// Try `Fast` first; on any non-success status, fall back to `Tight`.
    /// **Default for `schedule_segment_with_tolerance`.**
    Auto,
}

impl Default for ToleranceMode {
    fn default() -> Self {
        ToleranceMode::Auto
    }
}
```

- [ ] **Step 2: Add `schedule_segment_with_tolerance` wrapper; existing `schedule_segment` delegates with Tight**

Keep the existing 5-arg `schedule_segment(curve, limits, grid, v_start, v_end)` signature unchanged. Refactor its body to call the new 6-arg form:

```rust
/// Backward-compatible: equivalent to
/// `schedule_segment_with_tolerance(..., ToleranceMode::Tight)`.
pub fn schedule_segment(
    curve: &VectorNurbs<f64, 3>,
    limits: &Limits,
    grid: &GridConfig,
    v_start: f64,
    v_end: f64,
) -> Result<TopProfile, ScheduleError> {
    schedule_segment_with_tolerance(curve, limits, grid, v_start, v_end, ToleranceMode::Tight)
}

/// New entry point with adaptive-tolerance support. Per Step-4.5 spec §2.1.
pub fn schedule_segment_with_tolerance(
    curve: &VectorNurbs<f64, 3>,
    limits: &Limits,
    grid: &GridConfig,
    v_start: f64,
    v_end: f64,
    tolerance: ToleranceMode,
) -> Result<TopProfile, ScheduleError> {
    // ... existing setup-time validation: NaN/negative endpoint velocities,
    //     unsupported grid scheme — copy from current schedule_segment ...

    // Stage 1-2 unchanged: arclength grid + constraint bundle.
    let arc_grid = path::sample_arclength_grid(curve, grid.n)
        .map_err(|e| ScheduleError::PathParam(format!("{e}")))?;
    let bundle = match build(&arc_grid, limits, EndpointVelocities { v_start, v_end }) {
        BuildOutcome::Ok(b) => b,
        BuildOutcome::Boundary(BoundaryInfeasibility::StartAboveMvc { mvc_b }) => {
            return Ok(boundary_infeasible_profile(
                &arc_grid, *grid, crate::BoundarySide::Start, mvc_b, 0,
            ));
        }
        BuildOutcome::Boundary(BoundaryInfeasibility::EndAboveMvc { mvc_b }) => {
            let last = arc_grid.s.len() - 1;
            return Ok(boundary_infeasible_profile(
                &arc_grid, *grid, crate::BoundarySide::End, mvc_b, last,
            ));
        }
    };

    // Stage 3 (modified): adaptive-tolerance routing through slp_solve_with_axis_jerk.
    let (solver_result, slp_outcome) = match tolerance {
        ToleranceMode::Tight => solver::slp_solve_with_axis_jerk(&bundle, &arc_grid, limits, 1e-8)
            .map_err(|e| ScheduleError::SolverSetup(format!("{e}")))?,
        ToleranceMode::Fast => solver::slp_solve_with_axis_jerk(&bundle, &arc_grid, limits, 1e-5)
            .map_err(|e| ScheduleError::SolverSetup(format!("{e}")))?,
        ToleranceMode::Auto => {
            let (fast_result, fast_outcome) =
                solver::slp_solve_with_axis_jerk(&bundle, &arc_grid, limits, 1e-5)
                .map_err(|e| ScheduleError::SolverSetup(format!("{e}")))?;
            if solver_outcome_is_success(&fast_result, &fast_outcome) {
                (fast_result, fast_outcome)
            } else {
                // Fallback: re-solve at tight tolerance. Codex review-1: trigger
                // on ANY non-success status, not just DivergedSlp.
                solver::slp_solve_with_axis_jerk(&bundle, &arc_grid, limits, 1e-8)
                    .map_err(|e| ScheduleError::SolverSetup(format!("{e}")))?
            }
        }
    };

    // Stage 4-5 unchanged: verify + assemble.
    let verify_report = verify::check(&arc_grid, &solver_result, limits);
    Ok(output::assemble(&arc_grid, &solver_result, &verify_report, *grid, slp_outcome))
}

/// Per Codex review-1: `is_success` check uses solver-internal `SolverStatus`
/// (and `SlpOutcome`), not the public `SolveStatus`. The two differ — public
/// `SolveStatus` is set later by `output::assemble`.
fn solver_outcome_is_success(
    result: &solver::SolverResult,
    outcome: &solver::SlpOutcome,
) -> bool {
    let status_ok = matches!(
        result.status,
        solver::SolverStatus::Solved | solver::SolverStatus::SolvedInexact { .. }
    );
    // Per Codex review-1: any non-success outcome triggers fallback. Update this
    // match if `SlpOutcome` adds new success variants.
    let outcome_ok = matches!(outcome, solver::SlpOutcome::Converged { .. });
    status_ok && outcome_ok
}
```

**Important:** the `solver::SolverStatus` type is `pub(crate)` per `rust/temporal/src/topp/solver.rs:161`, distinct from `crate::SolveStatus` (the public-API enum at `lib.rs:71`). The `is_success` helper must inspect the internal type, not the public one. Codex review-1 caught this — the original draft of this task referenced the wrong type.

- [ ] **Step 3: Plumb `tol` through `slp_solve_with_axis_jerk` AND every Clarabel construction site**

In `rust/temporal/src/topp/solver.rs`, change the signature of `slp_solve_with_axis_jerk` to accept `tol: f64`, and pass it through to:

1. The inner `slp_solve` (path-jerk SLP loop) — also extend its signature with `tol`.
2. `solve_with_cuts` and `solve_with_cuts_and_trust_region` — these construct `DefaultSettings` and need the tolerance.
3. ANY OTHER Clarabel construction site — search:
   ```bash
   grep -n "DefaultSettings::<f64>" rust/temporal/src/topp/solver.rs
   ```
   Each match needs the tolerance threaded through.

For each `DefaultSettings` construction, replace:
```rust
let settings = DefaultSettings::<f64> {
    verbose: false,
    max_iter: 1000,
    ..Default::default()
};
```
with:
```rust
let settings = DefaultSettings::<f64> {
    verbose: false,
    max_iter: 1000,
    tol_gap_abs: tol,
    tol_gap_rel: tol,
    tol_feas: tol,
    ..Default::default()
};
```

**Codex review-1 finding F2 / kalico-plan-reviewer #2:** the original draft of this task only mentioned plumbing through `solve_with_cuts`, missing the per-axis SLP path's Clarabel construction site. ALL sites must be plumbed or `Auto` mode silently uses 1e-8 in some code paths.

- [ ] **Step 4: Re-export from `lib.rs`**

```rust
pub use topp::{schedule_segment_with_tolerance, ToleranceMode};
```

- [ ] **Step 5: Update existing tests — NO CHANGES NEEDED**

The backward-compat shim means existing 5-arg `schedule_segment(...)` callers continue to work unchanged with `ToleranceMode::Tight` semantics. Verify:

```bash
cd rust && cargo test -p temporal --release 2>&1 | grep -E "test result|FAILED" | tail -5
```

Expected: same set of tests pass as before (no caller-side updates required).

- [ ] **Step 6: Add tests for `Auto` mode**

In a new file `rust/temporal/tests/adaptive_tolerance.rs`:

```rust
//! Adaptive-tolerance regression tests. Spec §2.1 + Pi 5 investigation Finding 2.

use nurbs::VectorNurbs;
use temporal::{
    schedule_segment_with_tolerance, GridConfig, GridScheme, Limits,
    SolveStatus, ToleranceMode,
};

fn textbook_limits() -> Limits {
    Limits::new(
        [500.0; 3],
        [5_000.0; 3],
        [100_000.0; 3],
        2_500.0,
    )
}

#[test]
fn auto_succeeds_on_straight_line() {
    let curve = VectorNurbs::<f64, 3>::try_new(
        1, vec![0.0, 0.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [100.0, 0.0, 0.0]], None,
    ).unwrap();
    let grid = GridConfig { scheme: GridScheme::UniformArclength, n: 50 };
    let profile = schedule_segment_with_tolerance(
        &curve, &textbook_limits(), &grid, 0.0, 0.0, ToleranceMode::Auto,
    ).expect("Auto should succeed on straight line");
    assert!(matches!(
        profile.status,
        SolveStatus::Solved | SolveStatus::SolvedInexact { .. } | SolveStatus::SolvedSlp { .. }
    ));
}

#[test]
fn auto_falls_back_on_fixture_4_class() {
    // G5-style cubic with non-zero endpoint curvature — the Pi 5 investigation
    // Finding 2 SAFETY UPDATE failure case at tol=1e-5. Auto must fall back to
    // 1e-8 silently and succeed.
    let curve = VectorNurbs::<f64, 3>::try_new(
        3, vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [10.0, 30.0, 0.0],
            [40.0, 30.0, 0.0],
            [50.0, 0.0, 0.0],
        ], None,
    ).unwrap();
    let grid = GridConfig { scheme: GridScheme::UniformArclength, n: 100 };
    // Endpoint velocity at the centripetal cap (use a small fraction; full
    // setup is fixture-4 territory in tests/prototype.rs).
    let v_endpoint = 30.0;
    let profile = schedule_segment_with_tolerance(
        &curve, &textbook_limits(), &grid, v_endpoint, v_endpoint, ToleranceMode::Auto,
    ).expect("Auto should fall back on fixture-4-class geometry");
    assert!(matches!(
        profile.status,
        SolveStatus::Solved | SolveStatus::SolvedInexact { .. } | SolveStatus::SolvedSlp { .. }
    ));
}
```

- [ ] **Step 7: Run all tests**

```bash
cd rust && cargo test -p temporal --release 2>&1 | grep -E "test result|FAILED" | tail -5
```

Expected: all previous tests pass + 2 new adaptive_tolerance tests pass.

- [ ] **Step 8: Commit**

```bash
git add rust/temporal/src/topp/mod.rs rust/temporal/src/topp/solver.rs rust/temporal/src/lib.rs rust/temporal/tests/adaptive_tolerance.rs
git commit -m "temporal/topp: ToleranceMode + Auto fallback via wrapper (spec §2.1, review-1)"
```

---

### Task 8: 3-thread parallel executor (`multi::parallel`)

**Files:**
- Modify: `rust/temporal/src/multi/parallel.rs`

**Spec sections:** §2.6 + §3.3.

- [ ] **Step 1: Write the parallel-fan-out function (scoped-threads only — review-1 dropped the unsafe alternative)**

In `rust/temporal/src/multi/parallel.rs`:

```rust
//! 3-thread fan-out for re-solving dirty segments. Per spec §2.6.

use crate::multi::joining::SegmentState;
use crate::multi::{BatchError, SegmentInput};
use crate::topp::{schedule_segment_with_tolerance, ToleranceMode};
use crate::SolveStatus;
use crate::GridConfig;
use std::sync::Mutex;
use std::thread;

/// Re-solve all `dirty` segments in parallel across `n_threads` workers using
/// `std::thread::scope` (no unsafe; works because Rust 1.63+ scoped threads
/// borrow for the scope lifetime, which encloses the call). MSRV is 1.85.
///
/// Per Codex review-1 finding I + kalico-plan-reviewer #8 + Codex review-3 +
/// kalico-verifier confirmation: a profile returned from `schedule_segment` is
/// `Ok(_)` even when the SOCP returned `MaxIter`, `Infeasible`, or the SLP outer
/// loop returned `DivergedSlp` / `MaxIterSlp`. We MUST inspect the public
/// `SolveStatus` and only clear `dirty` on the verifier-feasible success
/// statuses: `Solved`, `SolvedInexact`, AND `SolvedSlp`.
///
/// `SolvedSlp` is critical to include — it represents a feasible solve where
/// the SLP outer loop materially required cuts (the actual termination path on
/// curved geometry like the cubic-with-endpoint-κ class). Verified via
/// kalico-verifier (this session): SolvedSlp is only reachable when both
/// (a) the inner solver returned Solved/SolvedInexact and (b) verify::check
/// passed feasibility at ε_feas = 1e-3. Treating it as failure would leave
/// every SLP-required-cuts segment dirty forever, breaking convergence on
/// curved geometry.
///
/// Failure statuses (`Infeasible`, `MaxIter`, `DivergedSlp`, `MaxIterSlp`)
/// fall into the catch-all `_` arm and leave dirty=true for the caller to
/// notice. The convergence loop's `MAX_SWEEPS` cap (Task 6) catches persistent
/// failures and surfaces them via `JoiningStatus::CappedAtMaxSweeps`.
pub(crate) fn fan_out_solves(
    inputs: &[SegmentInput<'_>],
    states: &mut [SegmentState],
    grids: &[GridConfig],
    n_threads: usize,
) -> Result<(), BatchError> {
    let dirty_indices: Vec<usize> = states.iter().enumerate()
        .filter_map(|(i, s)| if s.dirty { Some(i) } else { None })
        .collect();
    if dirty_indices.is_empty() {
        return Ok(());
    }

    let queue = Mutex::new(dirty_indices);
    let results: Mutex<Vec<(usize, Result<crate::TopProfile, crate::ScheduleError>)>>
        = Mutex::new(Vec::new());

    // Snapshot endpoint velocities into thread-shared Vec (avoids passing
    // &states across the scope boundary).
    let v_starts: Vec<f64> = states.iter().map(|s| s.v_start).collect();
    let v_ends: Vec<f64> = states.iter().map(|s| s.v_end).collect();

    thread::scope(|s| {
        for _ in 0..n_threads {
            s.spawn(|| {
                loop {
                    let idx = match queue.lock().unwrap().pop() {
                        Some(i) => i,
                        None => break,
                    };
                    let r = schedule_segment_with_tolerance(
                        inputs[idx].curve,
                        &inputs[idx].limits,
                        &grids[idx],
                        v_starts[idx],
                        v_ends[idx],
                        ToleranceMode::Auto,
                    );
                    results.lock().unwrap().push((idx, r));
                }
            });
        }
    });

    // Apply results. Per Codex review-1: only clear dirty on actual success.
    for (idx, r) in results.into_inner().unwrap() {
        match r {
            Ok(profile) => {
                let success = matches!(
                    profile.status,
                    SolveStatus::Solved
                    | SolveStatus::SolvedInexact { .. }
                    | SolveStatus::SolvedSlp { .. }
                );
                states[idx].profile = Some(profile);
                if success {
                    states[idx].dirty = false;
                }
                // else: leave dirty=true so join_until_converged knows the segment
                // didn't actually solve; the convergence loop's MAX_SWEEPS cap
                // will catch persistent failures.
            }
            Err(e) => return Err(BatchError::Segment(idx, e)),
        }
    }
    Ok(())
}
```

**Lock contention is not a concern at 3 workers** (per Codex review-1 finding C): each worker locks only to pop one index or push one result; SOCP solve dominates wall-clock by orders of magnitude.

**The unsafe raw-pointer alternative shown in earlier draft of this task is REMOVED** per review-1 finding C — it was unsound (`'static` lifetime forgery) and unnecessary given MSRV 1.85.

- [ ] **Step 2: Add a unit test that fan-out completes for 4 dirty segments**

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use crate::multi::SegmentInput;
    use crate::{GridConfig, GridScheme, Limits, TopProfile};
    use nurbs::VectorNurbs;

    fn straight() -> VectorNurbs<f64, 3> {
        VectorNurbs::<f64, 3>::try_new(
            1, vec![0.0, 0.0, 1.0, 1.0],
            vec![[0.0, 0.0, 0.0], [50.0, 0.0, 0.0]], None,
        ).unwrap()
    }

    fn limits() -> Limits {
        Limits {
            v_max: [500.0; 3], a_max: [5_000.0; 3], j_max: [100_000.0; 3],
            a_centripetal_max: 2_500.0,
        }
    }

    #[test]
    fn fan_out_processes_all_dirty() {
        let curves: Vec<_> = (0..4).map(|_| straight()).collect();
        let inputs: Vec<SegmentInput> = curves.iter().map(|c| SegmentInput {
            curve: c, limits: limits(), trailing_junction_chord_tolerance_mm: 0.05,
        }).collect();
        let grids = vec![GridConfig { scheme: GridScheme::UniformArclength, n: 20 }; 4];
        let mut states: Vec<_> = (0..4).map(|_| SegmentState {
            v_start: 0.0, v_end: 0.0, profile: None, dirty: true,
        }).collect();
        fan_out_solves(&inputs, &mut states, &grids, 3).unwrap();
        for s in &states {
            assert!(s.profile.is_some());
            assert!(!s.dirty);
        }
    }
}
```

- [ ] **Step 3: Run**

```bash
cd rust && cargo test -p temporal --release multi::parallel 2>&1 | tail -5
```

Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add rust/temporal/src/multi/parallel.rs
git commit -m "temporal/multi/parallel: 3-thread fan-out (spec §2.6)"
```

---

### Task 9: `plan_batch` end-to-end pipeline

**Files:**
- Modify: `rust/temporal/src/multi/mod.rs`

**Spec sections:** §3.3 internal pipeline.

- [ ] **Step 1: Replace the `unimplemented!()` with the full pipeline**

```rust
pub fn plan_batch(input: BatchInput<'_>) -> Result<BatchOutput, BatchError> {
    use crate::multi::{grid, joining, junction, parallel};
    use crate::GridConfig;

    if input.segments.is_empty() {
        return Err(BatchError::EmptySegments);
    }
    if input.worker_threads == 0 {
        return Err(BatchError::InvalidThreads);
    }

    let k = input.segments.len();

    // Stage 1: per-segment grid sizes.
    let grids: Vec<GridConfig> = input.segments.iter()
        .map(|s| GridConfig {
            scheme: crate::GridScheme::UniformArclength,
            n: grid::compute_n(&input.grid_strategy, s.curve),
        })
        .collect();

    // Stage 2: junction velocities (k-1 junctions).
    let junctions: Vec<junction::JunctionResult> = (0..k - 1)
        .map(|i| junction::compute_junction_velocity(
            input.segments[i].curve,
            input.segments[i + 1].curve,
            &input.segments[i].limits,
            &input.segments[i + 1].limits,
            input.segments[i].trailing_junction_chord_tolerance_mm,
        ))
        .collect();

    // Stage 3: seed per-segment states.
    let mut states: Vec<joining::SegmentState> = (0..k).map(|i| {
        let v_start = if i == 0 { 0.0 } else { junctions[i - 1].v_junction };
        let v_end = if i == k - 1 { 0.0 } else { junctions[i].v_junction };
        joining::SegmentState { v_start, v_end, profile: None, dirty: true }
    }).collect();

    // Stage 4: initial fan-out (all dirty).
    parallel::fan_out_solves(input.segments, &mut states, &grids, input.worker_threads)?;

    // Stage 5: joining loop with in-loop re-solves (review-1 corrected algorithm).
    let (sweeps, joining_status) = joining::join_until_converged(
        input.segments, &grids, &mut states, &junctions, input.worker_threads,
    )?;

    // Stage 6: assemble output.
    let profiles: Vec<_> = states.into_iter()
        .map(|s| s.profile.expect("all profiles solved by stage 5"))
        .collect();
    let junction_infos: Vec<JunctionInfo> = junctions.into_iter().enumerate()
        .map(|(i, j)| JunctionInfo {
            between_segments: (i, i + 1),
            v_junction: j.v_junction,
            binding_cap: j.binding_cap,
            kappa_left: j.kappa_left,
            kappa_right: j.kappa_right,
        })
        .collect();
    Ok(BatchOutput {
        profiles,
        junctions: junction_infos,
        joining_sweeps: sweeps,
        joining_status,
    })
}
```

- [ ] **Step 2: Add a sanity-test to `multi/mod.rs` that 1-segment-batch works**

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use crate::Limits;
    use nurbs::VectorNurbs;

    fn straight_50mm() -> VectorNurbs<f64, 3> {
        VectorNurbs::<f64, 3>::try_new(
            1, vec![0.0, 0.0, 1.0, 1.0],
            vec![[0.0, 0.0, 0.0], [50.0, 0.0, 0.0]], None,
        ).unwrap()
    }

    fn textbook_limits() -> Limits {
        Limits {
            v_max: [500.0; 3], a_max: [5_000.0; 3], j_max: [100_000.0; 3],
            a_centripetal_max: 2_500.0,
        }
    }

    #[test]
    fn plan_batch_single_segment_works() {
        let curve = straight_50mm();
        let segment = SegmentInput {
            curve: &curve,
            limits: textbook_limits(),
            trailing_junction_chord_tolerance_mm: 0.05,
        };
        let input = BatchInput {
            segments: &[segment],
            grid_strategy: GridStrategy::Adaptive {
                min_n: 10, max_n: 200, target_grid_spacing_mm: 0.5,
            },
            worker_threads: 1,
        };
        let output = plan_batch(input).expect("should succeed");
        assert_eq!(output.profiles.len(), 1);
        assert!(output.junctions.is_empty());
        // Single segment endpoints both 0.
        assert!(output.profiles[0].samples[0].v < 1e-3);
    }
}
```

- [ ] **Step 3: Run**

```bash
cd rust && cargo test -p temporal --release multi::tests::plan_batch_single 2>&1 | tail -5
```

Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add rust/temporal/src/multi/mod.rs
git commit -m "temporal/multi: plan_batch end-to-end pipeline (spec §3.3)"
```

---

### Task 10: Fixture 1 — Two G1 segments, sharp corner

**Files:**
- Create: `rust/temporal/tests/multi_segment.rs`

**Spec sections:** §5.1 fixture 1, §6 acceptance criteria.

- [ ] **Step 1: Create fixture 1 test**

```rust
//! Layer 2 multi-segment integration tests. Per spec §5.1.

use nurbs::VectorNurbs;
use temporal::{
    plan_batch, BatchInput, GridStrategy, JoiningStatus, JunctionBindingCap,
    Limits, SegmentInput,
};

fn textbook_limits() -> Limits {
    // Use Limits::new(...) — `Limits` is `#[non_exhaustive]` (Task 0), so
    // struct-literal construction is forbidden across the integration-test
    // crate boundary. Per review-2 (Codex BLOCKER + kalico-plan-reviewer
    // advisory).
    Limits::new(
        [500.0; 3],
        [5_000.0; 3],
        [100_000.0; 3],
        2_500.0,
    )
}

fn adaptive() -> GridStrategy {
    GridStrategy::Adaptive { min_n: 10, max_n: 200, target_grid_spacing_mm: 0.5 }
}

/// Spec §6.2 acceptance: every junction's v_end[k] ≈ v_start[k+1] ≈ v_junction
/// within ε_velocity = 1 mm/s. Reusable across all multi-segment fixtures
/// (review-1 finding F9: previously only fixture 1 enforced this).
fn assert_junction_continuity_for_all(
    output: &temporal::BatchOutput,
    eps_mm_s: f64,
) {
    for (k, junction) in output.junctions.iter().enumerate() {
        let v_jct = junction.v_junction;
        let v_end_left = output.profiles[k].samples.last().unwrap().v;
        let v_start_right = output.profiles[k + 1].samples[0].v;
        assert!(
            (v_end_left - v_jct).abs() < eps_mm_s,
            "junction {k}: v_end_left={v_end_left} vs v_jct={v_jct} (ε={eps_mm_s})",
        );
        assert!(
            (v_start_right - v_jct).abs() < eps_mm_s,
            "junction {k}: v_start_right={v_start_right} vs v_jct={v_jct} (ε={eps_mm_s})",
        );
    }
}

mod fixture_1_two_g1_sharp_corner {
    use super::*;

    #[test]
    fn fixture_1() {
        let left = VectorNurbs::<f64, 3>::try_new(
            1, vec![0.0, 0.0, 1.0, 1.0],
            vec![[0.0, 0.0, 0.0], [50.0, 0.0, 0.0]], None,
        ).unwrap();
        let right = VectorNurbs::<f64, 3>::try_new(
            1, vec![0.0, 0.0, 1.0, 1.0],
            vec![[50.0, 0.0, 0.0], [50.0, 50.0, 0.0]], None,
        ).unwrap();
        let limits = textbook_limits();
        let segments = [
            SegmentInput { curve: &left, limits, trailing_junction_chord_tolerance_mm: 0.05 },
            SegmentInput { curve: &right, limits, trailing_junction_chord_tolerance_mm: 0.05 },
        ];
        let input = BatchInput { segments: &segments, grid_strategy: adaptive(), worker_threads: 3 };
        let output = plan_batch(input).expect("should succeed");

        // Acceptance §6.1: each profile passes its own per-segment feasibility check
        // (already enforced by schedule_segment -> verify::check).
        assert_eq!(output.profiles.len(), 2);

        // Acceptance §6.2: junction continuity. v_end of seg 0 ≈ v_start of seg 1 ≈ v_junction.
        // Use shared helper (review-1 finding F9).
        assert_junction_continuity_for_all(&output, 1.0);
        let v_jct = output.junctions[0].v_junction;

        // §6.2: sharp-corner cap. Expected ≈ sqrt(2500 · 0.05 · 2.414) ≈ 17.4 mm/s.
        let expected = (2500.0_f64 * 0.05 * 2.414213562).sqrt();
        assert!((v_jct - expected).abs() < 0.1, "v_jct {} vs expected {}", v_jct, expected);
        assert!(matches!(output.junctions[0].binding_cap, JunctionBindingCap::SharpCornerChord));

        // §6.5: convergence in ≤3 sweeps.
        assert!(output.joining_sweeps <= 3);
        assert!(matches!(output.joining_status, JoiningStatus::Converged));
    }
}
```

- [ ] **Step 2: Run**

```bash
cd rust && cargo test -p temporal --release fixture_1 2>&1 | tail -10
```

Expected: PASS.

- [ ] **Step 3: Commit**

```bash
git add rust/temporal/tests/multi_segment.rs
git commit -m "temporal/tests: fixture 1 — G1+G1 sharp corner (spec §5.1)"
```

---

### Task 11: Fixture 2 — G1 → G5 smooth junction

**Files:**
- Modify: `rust/temporal/tests/multi_segment.rs`

**Spec sections:** §5.1 fixture 2.

- [ ] **Step 1: Add fixture 2 test**

```rust
mod fixture_2_g1_to_g5_smooth {
    use super::*;

    #[test]
    fn fixture_2() {
        // G1 ending at (50, 0, 0) with tangent +X.
        let left = VectorNurbs::<f64, 3>::try_new(
            1, vec![0.0, 0.0, 1.0, 1.0],
            vec![[0.0, 0.0, 0.0], [50.0, 0.0, 0.0]], None,
        ).unwrap();
        // Cubic G5-style with tangent matching at u=0 (also +X), curving away.
        let right = VectorNurbs::<f64, 3>::try_new(
            3, vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![
                [50.0, 0.0, 0.0],
                [60.0, 0.0, 0.0],     // CP1: tangent direction at u=0 = +X (matches left)
                [70.0, 30.0, 0.0],
                [100.0, 50.0, 0.0],
            ], None,
        ).unwrap();
        let limits = textbook_limits();
        let segments = [
            SegmentInput { curve: &left, limits, trailing_junction_chord_tolerance_mm: 0.05 },
            SegmentInput { curve: &right, limits, trailing_junction_chord_tolerance_mm: 0.05 },
        ];
        let input = BatchInput { segments: &segments, grid_strategy: adaptive(), worker_threads: 3 };
        let output = plan_batch(input).expect("should succeed");

        // §6.2: smooth-κ branch. Junction κ on right side > 0 (G5 has curvature at u=0).
        let j = &output.junctions[0];
        assert!(j.kappa_right.abs() > 1e-6, "G5 should have nonzero κ at u=0, got {}", j.kappa_right);
        // Expect Centripetal cap, not SharpCornerChord.
        assert!(matches!(j.binding_cap, JunctionBindingCap::Centripetal | JunctionBindingCap::PerAxisVelocity | JunctionBindingCap::GlobalVMax),
            "smooth junction should not trigger SharpCornerChord, got {:?}", j.binding_cap);

        assert!(output.joining_sweeps <= 3);
        assert!(matches!(output.joining_status, JoiningStatus::Converged));

        // §6.2 (review-1 helper): junction continuity.
        assert_junction_continuity_for_all(&output, 1.0);
    }
}
```

- [ ] **Step 2: Run**

```bash
cd rust && cargo test -p temporal --release fixture_2 2>&1 | tail -10
```

Expected: PASS.

- [ ] **Step 3: Commit**

```bash
git add rust/temporal/tests/multi_segment.rs
git commit -m "temporal/tests: fixture 2 — G1+G5 smooth junction (spec §5.1)"
```

---

### Task 12: Fixture 3 — Long straight + corner (lookahead)

**Files:**
- Modify: `rust/temporal/tests/multi_segment.rs`

**Spec sections:** §5.1 fixture 3, §6.3 lookahead correctness.

- [ ] **Step 1: Add fixture 3 test**

```rust
mod fixture_3_long_straight_then_corner {
    use super::*;
    use temporal::{schedule_segment_with_tolerance, GridConfig, GridScheme, ToleranceMode};

    #[test]
    fn fixture_3() {
        let straight = VectorNurbs::<f64, 3>::try_new(
            1, vec![0.0, 0.0, 1.0, 1.0],
            vec![[0.0, 0.0, 0.0], [100.0, 0.0, 0.0]], None,
        ).unwrap();
        let corner_right = VectorNurbs::<f64, 3>::try_new(
            1, vec![0.0, 0.0, 1.0, 1.0],
            vec![[100.0, 0.0, 0.0], [100.0, 50.0, 0.0]], None,
        ).unwrap();
        let limits = textbook_limits();
        let segments = [
            SegmentInput { curve: &straight, limits, trailing_junction_chord_tolerance_mm: 0.05 },
            SegmentInput { curve: &corner_right, limits, trailing_junction_chord_tolerance_mm: 0.05 },
        ];
        let input = BatchInput { segments: &segments, grid_strategy: adaptive(), worker_threads: 3 };
        let output = plan_batch(input).expect("should succeed");

        // §6.3 lookahead: profile of seg 0 at u=1 has v < v_max (decel happening).
        let v_end_seg0 = output.profiles[0].samples.last().unwrap().v;
        assert!(v_end_seg0 < 499.0, "seg 0 should be braking, v_end = {}", v_end_seg0);

        // §6.3: total time of seg 0 in joined batch > seg 0 in isolation with v_end=v_max.
        // Solve seg 0 alone with v_end=v_max for comparison.
        let solo_grid = GridConfig {
            scheme: GridScheme::UniformArclength,
            n: 200,  // fixed grid for the comparison solve
        };
        let solo = schedule_segment_with_tolerance(
            &straight, &limits, &solo_grid, 0.0, 500.0, ToleranceMode::Auto,
        ).expect("solo solve");
        let t_joined = output.profiles[0].total_time;
        let t_solo = solo.total_time;
        assert!(t_joined > t_solo,
            "joined seg 0 should take longer (decel for corner): joined={} solo={}",
            t_joined, t_solo);

        // §6.2 (review-2 fix): junction continuity helper applied to fixture 3
        // too — has 2 segments + 1 junction, same as fixture 1.
        assert_junction_continuity_for_all(&output, 1.0);

        // §6.5 convergence (review-2 fix): fixture 3 should also satisfy ≤3 sweeps.
        assert!(output.joining_sweeps <= 3,
            "lookahead fixture should converge in ≤3 sweeps");
        assert!(matches!(output.joining_status, temporal::JoiningStatus::Converged));
    }
}
```

- [ ] **Step 2: Run**

```bash
cd rust && cargo test -p temporal --release fixture_3 2>&1 | tail -10
```

Expected: PASS.

- [ ] **Step 3: Commit**

```bash
git add rust/temporal/tests/multi_segment.rs
git commit -m "temporal/tests: fixture 3 — lookahead test (spec §5.1, §6.3)"
```

---

### Task 13: Fixture 4 — Per-segment limits change

**Files:**
- Modify: `rust/temporal/tests/multi_segment.rs`

**Spec sections:** §5.1 fixture 4, §6.4 per-segment limits correctness.

- [ ] **Step 1: Add fixture 4 test**

```rust
mod fixture_4_per_segment_limits_change {
    use super::*;

    #[test]
    fn fixture_4() {
        let segments_curves: Vec<_> = (0..3).map(|i| {
            VectorNurbs::<f64, 3>::try_new(
                1, vec![0.0, 0.0, 1.0, 1.0],
                vec![
                    [(i as f64) * 50.0, 0.0, 0.0],
                    [((i + 1) as f64) * 50.0, 0.0, 0.0],
                ], None,
            ).unwrap()
        }).collect();
        let normal_limits = textbook_limits();
        let mut reduced_limits = normal_limits;
        reduced_limits.a_max = [2_500.0; 3];  // halved a_max for seg 1
        let segments = [
            SegmentInput { curve: &segments_curves[0], limits: normal_limits, trailing_junction_chord_tolerance_mm: 0.05 },
            SegmentInput { curve: &segments_curves[1], limits: reduced_limits, trailing_junction_chord_tolerance_mm: 0.05 },
            SegmentInput { curve: &segments_curves[2], limits: normal_limits, trailing_junction_chord_tolerance_mm: 0.05 },
        ];
        let input = BatchInput { segments: &segments, grid_strategy: adaptive(), worker_threads: 3 };
        let output = plan_batch(input).expect("should succeed");

        // §6.4: seg 1 profile peak |s̈| ≤ 2500 (1+ε).
        let max_a_seg1 = output.profiles[1].samples.iter()
            .map(|s| s.a.abs())
            .fold(0.0_f64, f64::max);
        assert!(max_a_seg1 <= 2_500.0 * 1.001,
            "seg 1 peak accel {} exceeds reduced a_max 2500", max_a_seg1);

        // §6.4 (review-1 fix): seg 0 / seg 2 actually reach textbook a_max,
        // confirming they're using their own (looser) limits, not the reduced
        // ones from seg 1. If joining incorrectly propagated reduced limits
        // outside seg 1's range, this would catch it.
        let max_a_seg0 = output.profiles[0].samples.iter()
            .map(|s| s.a.abs())
            .fold(0.0_f64, f64::max);
        let max_a_seg2 = output.profiles[2].samples.iter()
            .map(|s| s.a.abs())
            .fold(0.0_f64, f64::max);
        // Sanity: seg 0/2 peak accel should be much closer to 5000 (textbook)
        // than to 2500 (reduced). Allow 5% slack for adaptive-N quantization.
        assert!(max_a_seg0 > 2_500.0 * 1.5,
            "seg 0 peak accel {} suggests reduced limits leaked outside seg 1", max_a_seg0);
        assert!(max_a_seg2 > 2_500.0 * 1.5,
            "seg 2 peak accel {} suggests reduced limits leaked outside seg 1", max_a_seg2);

        // §6.2 (review-1 helper): junction continuity at both interior junctions.
        assert_junction_continuity_for_all(&output, 1.0);

        // §6.5 convergence (review-2 fix): fixture 4 also expects ≤3 sweeps.
        assert!(output.joining_sweeps <= 3);
        assert!(matches!(output.joining_status, JoiningStatus::Converged));
    }
}
```

- [ ] **Step 2: Run**

```bash
cd rust && cargo test -p temporal --release fixture_4 2>&1 | tail -10
```

Expected: PASS.

- [ ] **Step 3: Commit**

```bash
git add rust/temporal/tests/multi_segment.rs
git commit -m "temporal/tests: fixture 4 — per-segment limits change (spec §5.1, §6.4)"
```

---

### Task 14: Fixture 5 — Star pattern (joining stress)

**Files:**
- Modify: `rust/temporal/tests/multi_segment.rs`

**Spec sections:** §5.1 fixture 5, §6.5 (≤5 sweeps).

- [ ] **Step 1: Add fixture 5 test**

```rust
mod fixture_5_star_pattern {
    use super::*;

    #[test]
    fn fixture_5() {
        // 5-pointed star: 5 segments, alternating outward-spike + inward-cusp.
        // Use 5 short G1 segments forming a star-like pattern.
        let r_outer: f64 = 30.0;
        let r_inner: f64 = 12.0;
        let n_points = 5;
        let mut points: Vec<[f64; 3]> = Vec::new();
        for i in 0..n_points * 2 {
            let theta = (i as f64) * std::f64::consts::PI / (n_points as f64);
            let r = if i % 2 == 0 { r_outer } else { r_inner };
            points.push([r * theta.cos(), r * theta.sin(), 0.0]);
        }
        let curves: Vec<_> = points.windows(2).map(|w| {
            VectorNurbs::<f64, 3>::try_new(
                1, vec![0.0, 0.0, 1.0, 1.0],
                vec![w[0], w[1]], None,
            ).unwrap()
        }).collect();
        let limits = textbook_limits();
        let segments: Vec<_> = curves.iter().map(|c| SegmentInput {
            curve: c, limits, trailing_junction_chord_tolerance_mm: 0.05,
        }).collect();
        let input = BatchInput { segments: &segments, grid_strategy: adaptive(), worker_threads: 3 };
        let output = plan_batch(input).expect("should succeed");

        // §6.5: converges in ≤5 sweeps.
        assert!(output.joining_sweeps <= 5, "joining took {} sweeps", output.joining_sweeps);
        assert!(matches!(output.joining_status, JoiningStatus::Converged));

        // §6.2 (review-1 helper): junction continuity at every junction.
        // Star pattern has 9 junctions (n_points*2 - 1 segments).
        assert_junction_continuity_for_all(&output, 1.0);
    }
}
```

- [ ] **Step 2: Run**

```bash
cd rust && cargo test -p temporal --release fixture_5 2>&1 | tail -10
```

Expected: PASS.

- [ ] **Step 3: Commit**

```bash
git add rust/temporal/tests/multi_segment.rs
git commit -m "temporal/tests: fixture 5 — star pattern (spec §5.1, §6.5)"
```

---

### Task 15: Fixture 6 — Long realistic chain (perf sanity)

**Files:**
- Modify: `rust/temporal/tests/multi_segment.rs`

**Spec sections:** §5.1 fixture 6, §6.6 performance (sanity log only, no acceptance).

- [ ] **Step 1: Add fixture 6 test**

```rust
mod fixture_6_long_realistic_chain {
    use super::*;
    use std::time::Instant;

    fn realistic_machine_limits() -> Limits {
        // Limits::new because integration tests are external to temporal crate
        // (review-2 fix).
        Limits::new(
            [1000.0; 3],
            [65_000.0; 3],
            [50_000_000.0; 3],
            65_000.0,
        )
    }

    #[test]
    fn fixture_6() {
        // 10 segments mixed: 6 G1 straights of varying length, 2 G5 cubics, 2 G2 arcs.
        let mut curves: Vec<VectorNurbs<f64, 3>> = Vec::new();
        let mut x = 0.0;
        for i in 0..6 {
            let len = 5.0 + (i as f64) * 3.0;
            curves.push(VectorNurbs::<f64, 3>::try_new(
                1, vec![0.0, 0.0, 1.0, 1.0],
                vec![[x, 0.0, 0.0], [x + len, 0.0, 0.0]], None,
            ).unwrap());
            x += len;
        }
        // 2 G5 cubics (degree-3 with 4 CPs).
        for _ in 0..2 {
            curves.push(VectorNurbs::<f64, 3>::try_new(
                3, vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
                vec![
                    [x, 0.0, 0.0],
                    [x + 5.0, 10.0, 0.0],
                    [x + 15.0, 10.0, 0.0],
                    [x + 20.0, 0.0, 0.0],
                ], None,
            ).unwrap());
            x += 20.0;
        }
        // 2 G2 arcs (rational quadratic).
        let w = std::f64::consts::FRAC_1_SQRT_2;
        for _ in 0..2 {
            curves.push(VectorNurbs::<f64, 3>::try_new(
                2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
                vec![[x, 0.0, 0.0], [x + 10.0, 0.0, 0.0], [x + 10.0, 10.0, 0.0]],
                Some(vec![1.0, w, 1.0]),
            ).unwrap());
            x += 15.708;  // approximate quarter-circle arclength
        }

        let limits = realistic_machine_limits();
        let segments: Vec<_> = curves.iter().map(|c| SegmentInput {
            curve: c, limits, trailing_junction_chord_tolerance_mm: 0.05,
        }).collect();
        let input = BatchInput { segments: &segments, grid_strategy: adaptive(), worker_threads: 3 };

        let t0 = Instant::now();
        let output = plan_batch(input).expect("should succeed");
        let elapsed_ms = t0.elapsed().as_secs_f64() * 1000.0;

        // §6.5: convergence in ≤3 sweeps.
        assert!(output.joining_sweeps <= 3);

        // §6.2 (review-1 helper): junction continuity at every junction.
        assert_junction_continuity_for_all(&output, 1.0);

        // §6.6: performance sanity log (not acceptance). Expect <100ms on Pi 5.
        eprintln!("fixture_6 wall-clock: {:.2} ms (no acceptance threshold)", elapsed_ms);
    }
}
```

- [ ] **Step 2: Run**

```bash
cd rust && cargo test -p temporal --release fixture_6 -- --nocapture 2>&1 | tail -10
```

Expected: PASS + log line with wall-clock.

- [ ] **Step 3: Commit**

```bash
git add rust/temporal/tests/multi_segment.rs
git commit -m "temporal/tests: fixture 6 — long realistic chain perf sanity (spec §5.1, §6.6)"
```

---

### Task 16: Fixture 7 — Curvature-spike inter-grid sanity (v1 vs v2 gate)

**Files:**
- Modify: `rust/temporal/tests/multi_segment.rs`

**Spec sections:** §5.1 fixture 7, §6.6.5 inter-grid sanity methodology.

- [ ] **Step 1: Add fixture 7 test with cubic Hermite + per-axis Cartesian (v, a) + centripetal checks** (per-axis-jerk deferred to v2; see plan §"Spec §6.6.5 conformance" below)

```rust
mod fixture_7_curvature_spike_intergrid_sanity {
    use super::*;
    use nurbs::eval::{vector_derivative, vector_eval, curvature_from_derivs};
    use temporal::{
        schedule_segment_with_tolerance, GridConfig, GridScheme, GridSample, Limits, ToleranceMode,
    };

    #[test]
    fn fixture_7() {
        // Hand-rolled degree-3 NURBS with two close interior CPs producing a κ spike.
        let curve = VectorNurbs::<f64, 3>::try_new(
            3, vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![
                [0.0, 0.0, 0.0],
                [1.0, 5.0, 0.0],     // sharp interior bump
                [1.5, 5.0, 0.0],
                [3.0, 0.0, 0.0],
            ], None,
        ).unwrap();
        let limits = textbook_limits();

        // Force MIN_N=10 (explicitly NOT bumping to fix the test).
        let grid = GridConfig {
            scheme: GridScheme::UniformArclength,
            n: 10,
        };
        let profile = schedule_segment_with_tolerance(
            &curve, &limits, &grid, 0.0, 0.0, ToleranceMode::Auto,
        ).expect("schedule_segment_with_tolerance");

        // Pre-compute derivative NURBSes once for the entire resampling pass.
        // d3 (third derivative) intentionally NOT computed here — see the
        // per-axis-jerk discussion below + spec deferral note.
        let d1 = vector_derivative(&curve);
        let d2 = vector_derivative(&d1);

        // §6.6.5 methodology (v1, jerk deferred): re-evaluate per-axis Cartesian
        // velocity + acceleration + centripetal at 4× density via piecewise-cubic
        // Hermite of (v_i, a_i) pairs from solver. Compute geometric κ and
        // tangent direction directly from NURBS at each resampled point (NOT
        // interpolated κ). Per-axis-jerk validation deferred — see comment in
        // the per-axis check loop below + the conformance summary at section end.
        let n_resampled = 4 * profile.samples.len();
        let mut violations = Vec::new();
        let u_start = curve.knots()[0];
        let u_end = curve.knots()[curve.knots().len() - 1];
        for k in 0..n_resampled {
            let t = (k as f64) / (n_resampled as f64 - 1.0);
            let (v_path, a_path) = hermite_interp(&profile.samples, t);

            // Map normalized t ∈ [0,1] → u via uniform-in-u proxy. Approximation:
            // the solver grid is uniform-in-arclength, but for this spike-at-the-
            // middle test the segment is short enough that u ≈ s/L is acceptable.
            // For longer segments, replace with arclength→u inverse from Layer 0.
            let u = u_start + (u_end - u_start) * t;

            // Geometric quantities at u.
            let r1 = vector_eval(&d1.as_view(), u);     // dC/du
            let r2 = vector_eval(&d2.as_view(), u);     // d²C/du²
            let kappa = curvature_from_derivs(&d1, &d2, u);
            let speed_param = mag_3(r1);                 // |dC/du|
            if speed_param < 1e-12 { continue; }

            // Per-axis Cartesian time-derivatives at this resampled point.
            // Chain rule with arclength parameterization (|C'(s)|=1 in the
            // path-arclength frame; here we work in u-frame and convert via
            // |dC/du| for tangent magnitude):
            //   T(u) = r1 / |r1|     (unit tangent in motion direction)
            //   dx/dt   = T · v_path
            //   d²x/dt² = T · a_path + (curvature term · v_path²)
            //   d³x/dt³ = T · j_path + (mixed terms with κ, v_path²·a_path,
            //                           and the third derivative of position)
            let inv_speed = 1.0 / speed_param;
            let tangent = [r1[0] * inv_speed, r1[1] * inv_speed, r1[2] * inv_speed];
            // Normal-direction component of acceleration: a_n = κ · v² along the
            // principal normal. Direction: (r2 - (r2·T)T) / |...|. We just need
            // its per-axis projection onto each axis = perpendicular_component[axis].
            let r2_dot_t = r2[0]*tangent[0] + r2[1]*tangent[1] + r2[2]*tangent[2];
            let r2_perp = [
                r2[0] - r2_dot_t * tangent[0],
                r2[1] - r2_dot_t * tangent[1],
                r2[2] - r2_dot_t * tangent[2],
            ];
            let r2_perp_mag = mag_3(r2_perp);
            let normal_dir = if r2_perp_mag < 1e-12 {
                [0.0; 3]
            } else {
                [r2_perp[0]/r2_perp_mag, r2_perp[1]/r2_perp_mag, r2_perp[2]/r2_perp_mag]
            };
            let v_squared = v_path * v_path;
            let a_axis = [
                tangent[0] * a_path + normal_dir[0] * kappa * v_squared,
                tangent[1] * a_path + normal_dir[1] * kappa * v_squared,
                tangent[2] * a_path + normal_dir[2] * kappa * v_squared,
            ];
            // Per-axis jerk: NOT checked in v1 fixture 7 (per review-2 fix).
            // The full Cartesian per-axis jerk in arclength parameterization is
            //   j_axis_i = C'''_i · v³ + 3 · C''_i · v · a + C'_i · j
            // Implementing this correctly requires (a) computing d3 = third
            // NURBS derivative and (b) doing arclength→u inversion to map the
            // resampled t to the right u (so that C'(s)·... terms are valid in
            // the arclength frame). An earlier draft of this fixture used a
            // coarse bound `|j_path| + κ·|v·a_path|` that omitted the C''' term
            // and had the wrong factor on the middle term — Codex review-2
            // correctly flagged it as anti-conservative (could PASS when actual
            // jerk exceeds limit, defeating the sentinel's purpose). Rather
            // than ship a wrong bound, we drop the per-axis-jerk check from
            // v1's fixture 7. Centripetal + per-axis velocity + per-axis
            // acceleration are still strong enough to detect the κ-spike
            // under-resolution failure mode this fixture is designed to gate.
            // **Follow-up:** add a separate fixture (or extend fixture 7) once
            // arclength-inversion API is exposed from Layer 0 + we've audited
            // the full per-axis-jerk derivation. Until then, the per-axis-jerk
            // gap in this fixture is a known limitation, documented here.

            // Per-axis velocity + acceleration checks.
            for axis in 0..3 {
                let v_axis = tangent[axis].abs() * v_path;
                if v_axis > limits.v_max[axis] * 1.001 {
                    violations.push(format!(
                        "v_axis at u={u}, axis={axis}: {v_axis} > v_max={}",
                        limits.v_max[axis],
                    ));
                }
                if a_axis[axis].abs() > limits.a_max[axis] * 1.001 {
                    violations.push(format!(
                        "a_axis at u={u}, axis={axis}: {} > a_max={}",
                        a_axis[axis].abs(), limits.a_max[axis],
                    ));
                }
            }
            // Centripetal check.
            if v_squared * kappa > limits.a_centripetal_max * 1.001 {
                violations.push(format!(
                    "centripetal at u={u}: v²·κ={} > a_cent={}",
                    v_squared * kappa, limits.a_centripetal_max,
                ));
            }
        }

        if !violations.is_empty() {
            panic!(
                "v1 adaptive-N policy under-resolved curvature spikes — escalate to v2:\n{}",
                violations.join("\n"),
            );
        }
    }

    /// Piecewise-cubic Hermite interpolation of (v, a) solver samples at
    /// normalized parameter t ∈ [0,1]. Per spec §6.6.5 item 2.
    ///
    /// Treats sample.v as the function value and sample.a (path acceleration =
    /// dv/dt) as its time-derivative. The Hermite basis on [0,1] is:
    ///   h00(s) = 2s³ − 3s² + 1
    ///   h10(s) = s³ − 2s² + s
    ///   h01(s) = −2s³ + 3s²
    ///   h11(s) = s³ − s²
    /// f(s) = h00·v_i + h10·dt·a_i + h01·v_{i+1} + h11·dt·a_{i+1}
    /// where `dt` is the time between samples (≈ sample arclength / mean v).
    ///
    /// Returns (v_interp, a_interp). The third return slot (`j_interp`) was
    /// dropped per review-3 cleanup since v1 fixture 7 doesn't validate
    /// per-axis Cartesian jerk (deferred to v2 once arclength→u inversion +
    /// third-derivative APIs are exposed from Layer 0). When v2 lands and
    /// per-axis-jerk validation is implemented, recompute `j_interp` here as
    /// the closed-form second-derivative of the Hermite (formula left in the
    /// commit history at b03684c5 if needed for reference).
    fn hermite_interp(samples: &[GridSample], t: f64) -> (f64, f64) {
        let n = samples.len();
        if n < 2 {
            return (samples.first().map_or(0.0, |s| s.v), 0.0);
        }
        let pos = t * ((n - 1) as f64);
        let i = (pos.floor() as usize).min(n - 2);
        let s = pos - (i as f64);  // fractional [0,1) within segment i..i+1

        let v_i = samples[i].v;
        let v_ip1 = samples[i + 1].v;
        let a_i = samples[i].a;
        let a_ip1 = samples[i + 1].a;

        // Approximate Δt between samples from arclength + average speed.
        let ds = samples[i + 1].s - samples[i].s;
        let v_avg = 0.5 * (v_i + v_ip1).max(1e-9);  // avoid div0 at zero speed
        let dt = ds / v_avg;

        let s2 = s * s;
        let s3 = s2 * s;
        let h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
        let h10 = s3 - 2.0 * s2 + s;
        let h01 = -2.0 * s3 + 3.0 * s2;
        let h11 = s3 - s2;
        let v_interp = h00 * v_i + h10 * dt * a_i + h01 * v_ip1 + h11 * dt * a_ip1;

        // Derivatives of Hermite basis (w.r.t. s, then chain-rule by 1/dt).
        let dh00 = 6.0 * s2 - 6.0 * s;
        let dh10 = 3.0 * s2 - 4.0 * s + 1.0;
        let dh01 = -6.0 * s2 + 6.0 * s;
        let dh11 = 3.0 * s2 - 2.0 * s;
        let dv_ds = dh00 * v_i + dh10 * dt * a_i + dh01 * v_ip1 + dh11 * dt * a_ip1;
        let a_interp = dv_ds / dt;

        (v_interp, a_interp)
    }

    #[inline]
    fn mag_3(v: [f64; 3]) -> f64 {
        (v[0]*v[0] + v[1]*v[1] + v[2]*v[2]).sqrt()
    }
}
```

**Spec §6.6.5 conformance** (review-1 findings F5/F6 + Codex review-1 D/F + Codex review-2 jerk-bound math correction):

- **Centripetal**: `v²·κ ≤ a_centripetal_max` checked directly.
- **Per-axis Cartesian velocity**: `|t_axis · v_path| ≤ v_max,axis` checked.
- **Per-axis Cartesian acceleration**: full path-frame decomposition `T·a_path + N·κ·v²` with per-axis projection checked.
- **Per-axis Cartesian jerk**: **NOT checked in v1 fixture 7** (review-2 deferral). Implementing the correct upper bound requires `C'''_i · v³ + 3·C''_i · v · a + C'_i · j` in the arclength frame, which needs (a) third-derivative NURBS computation and (b) arclength→u inversion API from Layer 0. An earlier draft used a coarse bound that omitted `C'''` and had a wrong middle-term coefficient — Codex review-2 correctly flagged it as anti-conservative (could PASS while actual jerk exceeded limits). Rather than ship a wrong gate, defer per-axis-jerk validation. **The other checks (centripetal + per-axis v + per-axis a) are still strong enough to detect the κ-spike under-resolution failure mode this fixture is designed to gate** — the spike's primary signature is centripetal-cap violation, not jerk violation.
- **Snap / jerk-of-jerk**: NOT checked (per spec §6.6.5 item 5 — not in our constraint set).
- **Cubic-Hermite interpolation** of (v, a) solver samples: implemented per spec §6.6.5 item 2.
- **Geometric κ resampled** from NURBS via Layer 0's `curvature_from_derivs` (NOT interpolated): implemented.

**Known limitations** (acknowledged in spec §6.6.5 framing as "sentinel, not proof"):
1. Per-axis-jerk gap noted above.
2. Resampled `u` is computed via `u ≈ u_start + (u_end − u_start) · t` — uniform-in-`u` proxy for arclength-uniform `t`. For Fixture 7's short curvature-spike geometry this is acceptable (spike is broad enough to span multiple resampled points) but for sharper localized spikes a proper arclength→u inversion would be more robust. Defer to a v2 fixture.

- [ ] **Step 2: Run**

```bash
cd rust && cargo test -p temporal --release fixture_7 2>&1 | tail -15
```

Expected (if v1 policy is OK on this geometry): PASS.
Expected (if v1 policy under-resolves): FAIL with violation list. **In that case, escalate to v2 per spec §10 before merging Step 4.5** — implement curvature-aware adaptive N in `multi/grid.rs` and re-run.

- [ ] **Step 3: Commit**

```bash
git add rust/temporal/tests/multi_segment.rs
git commit -m "temporal/tests: fixture 7 — curvature-spike inter-grid sanity (spec §5.1, §6.6.5)"
```

---

### Task 17: CLAUDE.md plan-changes-log entry

**Files:**
- Modify: `CLAUDE.md`

- [ ] **Step 1: Add plan-changes-log entry**

Append to the `# Plan changes log` section:

```markdown
---

**Build-order Step 4.5 (Layer 2 multi-segment integration): completed.**
Implementation per `docs/superpowers/plans/2026-04-27-layer-2-multi-segment.md`.
New `multi/` module under `rust/temporal/` with `plan_batch` entry point;
junction velocity from unified centripetal-against-curvature formula
(subsumes Sonny-Jeon JD as G1↔G1 degenerate case); option-(A) joining
with SOCP-per-iteration via Step-4 `schedule_segment`; adaptive-N policy
(arclength-based v1, fixture 7 gates v2 escalation); 3-thread parallel
batch executor; per-call adaptive-tolerance (ToleranceMode::Auto with
fallback from 1e-5 to 1e-8 on any non-success status). Six fixtures
land in `rust/temporal/tests/multi_segment.rs` exercising sharp G1↔G1
corners, smooth G1↔G5 junctions, lookahead, per-segment limits, joining
convergence stress, long realistic chains. Fixture 7 inter-grid sanity
test passed with v1 policy on the curvature-spike fixture; if it had
failed, v2 (curvature-aware adaptive N) would have been required first.

**Evidence:** Plan + 17 tasks committed on this branch. Spec at
`docs/superpowers/specs/2026-04-27-layer-2-multi-segment-design.md`;
Pi 5 throughput investigation at
`docs/research/pi5-socp-throughput-investigation.md`; JD formula
verification at `docs/research/junction-deviation-cornering-formula.md`.
Multi-segment integration tests at `rust/temporal/tests/multi_segment.rs`.
Top-level code review by `superpowers:code-reviewer` (opus): TBD on
plan execution.
```

- [ ] **Step 2: Commit**

```bash
git add CLAUDE.md
git commit -m "CLAUDE.md plan-changes-log: Step 4.5 Layer 2 multi-segment completed"
```

---

## Self-Review

**Spec coverage check:**

- §2.1 option (A) joining with SOCP-per-iter → Tasks 4–6, 7 (ToleranceMode::Auto), 9 ✓
- §2.2 unified junction velocity formula → Task 3 ✓
- §2.3 forward/reverse joining + convergence → Tasks 4, 5, 6 ✓
- §2.4 per-segment limits as input data → Task 9 (consumes per-segment Limits in BatchInput) ✓
- §2.5 adaptive N → Task 2 ✓
- §2.6 3-thread parallel executor → Task 8 ✓
- §3.1 module layout → Task 1 ✓
- §3.2 public API → Task 1 (types) + Task 9 (plan_batch impl) ✓
- §3.3 internal pipeline → Task 9 ✓
- §5.1 six fixtures → Tasks 10–15 ✓
- §5.1 fixture 7 (post-Codex addition) → Task 16 ✓
- §6.1 per-segment correctness → enforced by schedule_segment, validated in fixtures ✓
- §6.2 junction velocity correctness → Task 10 (fixture 1) explicit asserts ✓
- §6.3 lookahead correctness → Task 12 (fixture 3) ✓
- §6.4 per-segment limits correctness → Task 13 (fixture 4) ✓
- §6.5 joining convergence ≤3/≤5 → Tasks 10–15 explicit asserts ✓
- §6.6 perf sanity log → Task 15 (fixture 6) ✓
- §6.6.5 inter-grid sanity → Task 16 (fixture 7) ✓
- Adaptive-tolerance API (Codex review-2 follow-up) → Task 7 ✓
- CLAUDE.md plan-changes-log update → Task 17 ✓

**Placeholder scan:** Searched for "TBD" / "TODO" / "implement later" / "Similar to Task N":
- Task 16 has `unimplemented in the actual fixture` — replaced with explicit "Implementation note" callout pointing to spec §6.6.5 item 2; the simplified Hermite stub is acceptable for plan-readability but the executor must implement the real cubic Hermite. Flagged in Step 1 prose.
- No other placeholders.

**Type consistency:** Cross-checked across tasks:
- `SegmentInput` used identically in Tasks 1, 8, 9, 10–16 ✓
- `BatchInput` / `BatchOutput` / `JunctionInfo` consistent across Tasks 1, 9, 10–16 ✓
- `JunctionResult` (internal) consistent across Tasks 3, 4, 9 ✓
- `SegmentState` consistent across Tasks 4, 5, 6, 8, 9 ✓
- `ToleranceMode` consistent across Tasks 7, 8, 9, 12, 16 ✓
- `compute_n` signature: `(&GridStrategy, &VectorNurbs<f64, 3>) -> usize` — consistent in Tasks 2 and 9 ✓

**Real-life caveat:** ~~the exact `nurbs::VectorNurbs::derivative_at` API is referenced but unverified~~ **(superseded by review-1)**: Task 0 now locks the audited nurbs API surface (`vector_derivative` + `vector_eval` + `curvature_from_derivs` per `rust/nurbs/src/eval.rs`) before Task 3 begins, and Task 3 Step 5 uses those exact functions. No remaining unverified API caveat.

---

## Post-review revisions (Plan review-1: kalico-plan-reviewer + Codex)

Both reviewers ran in parallel and converged on the same critical findings. Substantial revisions applied:

**Algorithmic correctness fixes:**
- **Task 6 + Task 9 joining loop**: original draft ran sweeps without re-solving dirty segments between them, which would converge on stale velocity caps + leave profiles inconsistent with final boundary velocities. This collapses option (A) "SOCP per joining iteration" into something closer to (B) "SOCP at finalize" — exactly what the throughput-non-negotiable principle disallows. Fixed: `join_until_converged` now invokes `fan_out_solves` between each forward+reverse pair, signature extended to take inputs/grids/n_threads.
- **Task 7 plumbing**: original draft called `solver::slp_solve(&bundle, tol)` but the actual entry point used by `schedule_segment` is `solver::slp_solve_with_axis_jerk` (added by the parallel Step 4/9 agent during this session). The original would have silently bypassed Step 9's per-axis Cartesian jerk SLP, regressing verifier feasibility. Fixed: tolerance plumbs through the actual entry point AND every Clarabel `DefaultSettings` construction site (search command added to ensure no site is missed).
- **Task 7 `is_success` types**: original draft used `crate::SolveStatus` (the public enum), but `SolverResult.status` is the internal `solver::SolverStatus` (different type). Wouldn't have compiled. Fixed.
- **Task 8 `fan_out_solves` dirty-clearing**: original cleared `dirty` on any `Ok(profile)` from `schedule_segment`, but `Ok` is returned even for non-success statuses (`MaxIter`, `Infeasible`, `DivergedSlp`). Fixed: only clear `dirty` on `Solved` / `SolvedInexact`; non-success leaves dirty=true so the convergence loop catches it.

**Compilation fixes:**
- **Task 3 nurbs API**: original draft used `curve.derivative_at(u, n)` which doesn't exist. Actual nurbs API is `nurbs::eval::vector_derivative` (degree-lowering returns new NURBS) + `nurbs::eval::vector_eval` (point evaluation on a view) + `nurbs::eval::curvature_from_derivs`. Rewrote Task 3 against the audited surface. Added Task 0 to lock the audit before Task 3 starts.

**Architectural-stability fixes:**
- **Task 7 backward-compat wrapper**: original draft mutated `schedule_segment`'s 5-arg signature to 6-arg, breaking every existing caller (~10 sites in tests + unit tests). Codex review-1 suggested keeping the 5-arg form unchanged and adding a new `schedule_segment_with_tolerance` wrapper. Adopted — eliminates merge-conflict risk with parallel Step 4/9 work, eliminates the test-mass-edit step, preserves backward compat.
- **Task 0 `Limits` `#[non_exhaustive]`**: spec §7.3 calls for additive-extension safety so Step 9 can add a shaper-aware acceleration constraint field without breaking Step 4.5 callers. Added Task 0 sub-step + a `Limits::new` constructor for external use.

**Acceptance-gate fixes:**
- **Task 16 fixture 7**: original draft stubbed Hermite interpolation as linear ("simplified linear interp for plan readability; replace with cubic Hermite in actual implementation") and explicitly omitted per-axis Cartesian checks ("omitted in this draft for brevity"). Both are placeholders the writing-plans skill prohibits, AND fixture 7 is the v1-vs-v2 gate — a linear-interp stub would systematically under-detect the inter-grid violations the gate is designed to catch. Spelled out the cubic Hermite formulas (h00/h10/h01/h11 basis with `dt = ds/v_avg` time scaling), added per-axis velocity / acceleration / jerk-bound checks, kept centripetal check, explicitly excluded snap/jerk-of-jerk per spec §6.6.5 item 5.
- **§6.2 acceptance helper**: original draft only enforced junction-velocity continuity in fixture 1 (Task 10). Lifted to a shared `assert_junction_continuity_for_all` helper used by every multi-segment fixture (Tasks 11, 13, 14, 15).
- **§6.4 seg 0/2 assertions**: Task 13 originally said "No assertion needed beyond existing per-segment feasibility" for the unchanged-limits segments. That misses the spec §6.4 implicit requirement: seg 0/2 must reach textbook a_max (confirming reduced limits did NOT leak outside seg 1's range). Added explicit `max_a_seg0/2 > 2_500.0 * 1.5` assertion.

**Code-quality fixes:**
- **Task 8 unsafe parallel option dropped**: original presented two implementations of `fan_out_solves` — one with `unsafe` raw-pointer slice reconstruction (transmutes `'a` lifetime to `'static`, unsound), one with `std::thread::scope`. Codex correctly flagged the unsafe variant as unsound + unnecessary at MSRV 1.85. Removed entirely; only scoped-threads form remains.

**Pre-Flight hardening:**
- Added explicit hard prerequisite that Step 4/9 must be committed before Task 7 starts. Added optional `git worktree add` recipe for executing Step 4.5 in isolation if the parallel Step 4/9 work is still in flight.

**Findings reviewers raised that this revision did NOT change** (intentionally):
- Some reviewer-suggested style tightenings (e.g., extracting `2.414213562` magic number to a constant) deferred to executor's discretion — small enough to not warrant explicit plan instruction.
- The MAX_SWEEPS = 10 cap kept as-is even though spec §10 says "might tighten to 5 once we have real-fixture data." Keeping looser cap during initial implementation; can tighten once fixtures pass with smaller sweep counts.

---

## Post-review revisions (Plan review-3: kalico-plan-reviewer APPROVED + Codex NEEDS-REVISION → all findings verified + applied)

Round-3 dual review found two MAJORs (one Codex-only, one missed by reviewer-1's local grep) and three MINORs. Critically, this round adopted a **per-Codex-finding kalico-verifier verification** pattern: rather than applying Codex's suggestions blindly, each load-bearing claim was independently adversarially verified before applying.

**Verified-then-applied:**

1. **`SolvedSlp` missing from Task 8 dirty-clearing success arm** — Codex MAJOR. Verified by kalico-verifier (`docs/research/...verifier transcript`): SolvedSlp is a fully verifier-feasible success status (only emitted when both `verify::check` passes AND inner solver returned Solved/SolvedInexact AND SLP cuts were materially required). The original `Solved | SolvedInexact` success match would have left every SLP-required-cuts segment dirty forever, breaking convergence on cubic-class geometry. Fixed: success arm is now `SolveStatus::Solved | SolvedInexact { .. } | SolvedSlp { .. }`. The verifier also caught two corollary issues Codex missed: (a) the prose mistakenly named `SolverStatus::Solved` (internal type) when it should be `SolveStatus::Solved` (public); (b) `MaxIterSlp` falls into the catch-all `_` arm correctly. Both fixed. Same updates applied to the `auto_succeeds_on_*` test asserts in Task 7's adaptive-tolerance test file.

2. **Joining-loop dirty-spin early-bail** — Codex MAJOR + kalico-plan-reviewer MAJOR. Both reviewers proposed slightly different early-bail conditions; verified by kalico-verifier that the simpler Codex variant ("after fan_out_solves, if velocities have stabilized AND segments are still dirty, immediately return CappedAtMaxSweeps") is sound. Verification chain: (i) Clarabel 0.11.1 with kalico's default features (no `faer-sparse`) uses single-threaded QDLDL → fully deterministic IPM; (ii) SLP outer loops have no RNG / no time / no parallelism; (iii) constraint matrix construction is deterministic; (iv) `forward_sweep`/`reverse_sweep` only read `state.v_start`/`state.v_end` (joining-decided), never `state.profile.last_sample.v` (SOCP-actual), so post-fan_out velocity stability genuinely implies no further propagation. Conclusion: re-solving a still-dirty segment with unchanged inputs always produces the same non-success status; bailing immediately is strictly correct. Fixed: `join_until_converged` now returns `CappedAtMaxSweeps` after the first stable-velocities + still-dirty observation, bounding waste at ~1 sweep × |dirty| instead of 10 × N. Misleading comment ("dirty == true ... should already be a returned error") replaced with accurate three-case explanation.

**Compile blockers found and fixed:**

3. **`crate::topp::TopProfile` at plan line 1369** — Codex MAJOR. Round-2 grep only matched `use` statements; missed inline type annotation in the `Mutex<Vec<(usize, Result<crate::topp::TopProfile, crate::topp::ScheduleError>)>>` declaration. Fixed.

**Cleanup edits (no verification needed):**

4. **Hermite helper's `j_interp` is dead code** since per-axis-jerk check was deferred. Both reviewers flagged it. Helper now returns `(f64, f64)` instead of `(f64, f64, f64)`; `_j_path` destructuring at the call site dropped. Old formula left in commit history (b03684c5) for reference when v2 jerk validation lands.
5. **Task 16 Step 1 heading** still said "per-axis Cartesian checks (v, a, j)" despite the jerk drop. Updated to "per-axis Cartesian (v, a) + centripetal checks; per-axis-jerk deferred to v2."
6. **Self-review's stale `derivative_at` caveat** at the bottom of the plan — superseded by Task 0's NURBS API audit. Annotated as superseded.
7. **Spec §6.6.5 deferral note added** — kalico-plan-reviewer flagged that the spec listed per-axis Cartesian jerk as required without a matching deferral note; future reader sees mismatch. Added explicit deferral paragraph in the spec referencing this round-3 review.

**Findings reviewers raised that this revision did NOT change** (with rationale):

- **Codex's "consider pinning Clarabel determinism"** — verifier suggested explicitly setting `direct_solve_method: "qdldl"` + `max_threads: 1` in `DefaultSettings` (`solver.rs:811`) as insurance against future Clarabel feature-flag changes. Defensible, but lives in upstream `solver.rs` (Step 4/9 surface), not in Step 4.5's new code. Treating as a follow-up to coordinate with the Step-4-agent rather than a Step-4.5 plan correction.

**Net result:** Round 3 was meaningfully productive — caught one real correctness bug (SolvedSlp omission would have broken convergence on cubic-class fixtures), one real efficiency bug (dirty-spin to MAX_SWEEPS without progress), one missed compile-blocker, and three cleanup items. **All Codex's substantive claims that I applied went through kalico-verifier first**, which both validated them AND surfaced corollary issues Codex missed (SolverStatus naming, MaxIterSlp categorization, comment correctness, determinism insurance). The verify-each-Codex-finding pattern paid off and should be reused.

---

## Post-review revisions (Plan review-2: kalico-plan-reviewer APPROVED + Codex NEEDS REVISION)

Round-2 re-review of the post-review-1 plan. **Both reviewers converged on the same compile-blocker findings** — strong signal these were real:

**Compile blockers found and fixed:**

1. **`crate::topp::TopProfile` import wrong** — `TopProfile` is defined in `lib.rs` and re-exported at crate root, not under `topp::`. Fixed at three sites: Task 1 (`multi/mod.rs`), Task 4 (`multi/joining.rs::SegmentState`), Task 8 (`multi/parallel.rs` unit-test mod). Now use `crate::TopProfile`.

2. **`crate::topp::SolveStatus` import wrong** — same root issue. Fixed in Task 7 — now `crate::SolveStatus`.

3. **`temporal::topp::GridSample` import wrong** — `GridSample` is at `temporal::GridSample` (root), not under `topp::`. Fixed in Task 16.

4. **Fixture 3 calling `schedule_segment` with 6 args** — review-1 added the backward-compat wrapper but missed updating fixture 3's solo-comparison call site. Now uses `schedule_segment_with_tolerance(..., ToleranceMode::Auto)`.

5. **`#[non_exhaustive] Limits` external-literal construction** — Task 0 made `Limits` non-exhaustive, but the integration-test helpers `textbook_limits()` (Task 10) and `realistic_machine_limits()` (Task 15) used struct-literal construction, which is forbidden across the test-crate boundary. Both rewritten to `Limits::new(...)`.

**Math correctness fix (Codex review-2 caught, kalico-plan-reviewer didn't):**

6. **Fixture 7 per-axis jerk bound was anti-conservative** — original draft used `j_bound = |j_path| + κ·|v·a_path|` as the per-axis Cartesian jerk upper bound. Codex review-2 correctly observed: the full Cartesian formula in arclength parameterization is `C'''_i · v³ + 3·C''_i · v · a + C'_i · j` — my bound omitted the `C'''_i · v³` term entirely AND had factor 1 instead of 3 on the middle term. This means the bound could report PASS when actual jerk exceeded the limit — defeating the v1-vs-v2 gate's purpose. **Fixed by dropping the per-axis-jerk check from v1 fixture 7 entirely with explicit "deferred to v2" documentation.** Implementing the full formula correctly requires the third NURBS derivative + arclength→u inversion from Layer 0, neither of which are needed for the centripetal + per-axis-v + per-axis-a checks that ARE the primary signatures of κ-spike under-resolution. The fixture remains effective at its intended job; the per-axis-jerk gate becomes a follow-up fixture once the supporting Layer 0 API is exposed.

**Coverage gaps fixed:**

7. **Fixture 3 missing junction-continuity helper** — added.
8. **Fixture 3 + Fixture 4 missing convergence (≤3 sweeps + Converged) assertions** — added.

**Findings reviewers raised that this revision did NOT change** (with rationale):

- **Dirty-on-non-success can spin to MAX_SWEEPS** — kalico-plan-reviewer ruled it acceptable (bounded by MAX_SWEEPS × dirty count, surfaces failure via `JoiningStatus::CappedAtMaxSweeps { last_dirty_count }`); Codex called it MAJOR. Both have merit. We're keeping current behavior because (a) MAX_SWEEPS = 10 caps the waste at ~10× per-segment cost on persistently-infeasible segments, (b) the failure surfaces explicitly to the caller via the `joining_status` field, (c) implementing endpoint-relaxation-on-failure is a more substantial algorithmic change appropriate for a v2 of Step 4.5 once we have real-fixture data on how often this matters in practice.
- **Codex's "placeholder scan inaccurate" claim re Task 3 Step 1's `todo!()` stubs** — these get replaced in Step 5 of the same task; standard TDD scaffolding within a task's flow, not a cross-task placeholder. Acceptable.

**Net result:** kalico-plan-reviewer round-2 returned APPROVED with these advisories; Codex returned NEEDS REVISION with these blockers. Both reviewers' load-bearing findings now addressed. Plan is ready to execute.

---

## Plan complete

**Saved to** `docs/superpowers/plans/2026-04-27-layer-2-multi-segment.md`. Revised after dual review-1 + dual review-2 (kalico-plan-reviewer + Codex, both rounds run in parallel for cross-validation). Ready to commit and execute.
