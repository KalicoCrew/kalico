# Junction Accel Continuity (Condensed Smooth-Chain SOCP) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. All Rust work goes to `rust-engineer` subagents (do not specify a model). All `cargo` commands run from `rust/`.

**Goal:** Fuse tangent-continuous segments into single condensed TOPP SOCPs so junction velocity *and* acceleration are emergent results of one optimization (full `j_max` across smooth junctions), and carry `(v, a)` across the streaming replan boundary.

**Architecture:** A new `ChainGrid` (concatenated per-segment arclength grids, shared junction points with dual geometry, per-interval spacing) replaces `(ArclengthGrid, Limits)` as the input to `constraints::build`, the SLP solver, and the verifier — a single segment is a chain of length 1, one code path. The `multi/` joining layer partitions windows into smooth chains at corner junctions, fans out chain solves, and sweeps only corner velocities. `trajectory` plumbs `initial_a` from the shaped profile at `t_dispatched` down to the first chain's `a_0` pin.

**Tech Stack:** Rust workspace at `rust/`; Clarabel 0.11 SOCP solver; spec at `docs/superpowers/specs/2026-06-07-junction-accel-continuity-design.md`; math derivations at `docs/research/condensed-smooth-chain-socp-junction.md`.

---

## Context primer (read before any task)

**Variable layout** (pinned in `constraints.rs`, relied on by `solver.rs:176-181`): for a grid of `M` points, `x[0..M]` = `b_i` (= ṡ²), `x[M..2M]` = `a_i` (= s̈), then `n_interior = M−2` each of `t`, `x1`, `x2`. Offsets `off_b = 0`, `off_a = M`, `off_t = 2M`, `off_x1 = off_t + n_interior`, `off_x2 = off_x1 + n_interior`.

**Sign conventions:** bundle rows are `A_k·x + b_rhs ∈ K`; Clarabel rows are negated (`A_clarabel = −A_k`, see `solver.rs:1-10`). The SOC time chain (block h) encodes `t_i ≥ h̄_i/√b_i` via three 3-dim SOC cones per interior point.

**Non-uniform stencils** (verified in `docs/research/condensed-smooth-chain-socp-junction.md`; exact for quadratics). For point `i` with left spacing `hl = s_i − s_{i−1}` and right spacing `hr = s_{i+1} − s_i`, with `D = hl·hr·(hl+hr)`:

```text
b′(s_i) ≈ [−hr²·b_{i−1} + (hr²−hl²)·b_i + hl²·b_{i+1}] / D          (a_i = b′_i / 2)
b″(s_i) ≈ [ 2hr·b_{i−1} − 2(hl+hr)·b_i + 2hl·b_{i+1}] / D
```

Uniform reduction `hl = hr = h`: `b′ = (b_{i+1}−b_{i−1})/(2h)`, `b″ = Δ²b/h²`. **Cell width** `h̄_i = (h_{i−1} + h_i)/2` for interior points (used by blocks f/g/h and the objective — verifier item 5a: never reuse a single scalar `h` across a spacing change).

**Traps (each is a verified failure mode — do not "simplify" these away):**
1. Never pin `a_0` when `v_start == 0`: the pin row `b_1 = b_0 + 2h·a_0` with `b_0 = 0, a_0 = 0` forces `b_1 = 0` and wastes ~9% trajectory time. Rest starts keep free FD accel + block (e2) envelope.
2. Never pin accel at corner junctions: sweep monotonicity (and hence joining-layer optimality) *depends* on corner accel being free.
3. `j_path` in block (h) must remain a single scalar per chain (min over all segments' axes) — the SOC chain is only convex for scalar J.
4. Block (e2) gates on `endpoint v == 0` (either edge), not on "is batch edge".
5. Junction-spanning rows inherit the row-∞-norm scaling discipline from `append_axis_jerk_cut_to_clarabel` (`solver.rs:371-385`).

## File structure

| File | Change |
|---|---|
| `temporal/src/topp/stencil.rs` | + weight-based non-uniform `b′`/`b″` helpers; `s_dddot_at` generalizes to weights |
| `temporal/src/topp/chain.rs` | **new** — `PointGeom`, `JunctionDual`, `ChainGrid`, assembly from per-segment `ArclengthGrid`s |
| `temporal/src/topp/constraints.rs` | `build` consumes `&ChainGrid`; per-point `h̄`; junction dual rows; `a_start` pin; `EndpointVelocities` → `EndpointConditions` |
| `temporal/src/topp/scaling.rs` | + `for_chain`, `scale_chain_grid` |
| `temporal/src/topp/solver.rs` | weights-based cuts (3-case stencil match collapses); per-point limits; junction dual-geometry scans |
| `temporal/src/topp/verify.rs` | chain-aware `check` (per-point limits, dual geometry, weights `s⃛`) |
| `temporal/src/topp/mod.rs` | + `schedule_chain_with_tolerance`; `schedule_segment*` become chain-of-1 wrappers |
| `temporal/src/multi/junction.rs` | + `JunctionKind::{Smooth, Corner}` classification |
| `temporal/src/multi/chain.rs` | **new** — chain partitioning, per-chain `ChainGrid` build, profile slicing |
| `temporal/src/multi/joining.rs` | sweep over `ChainState`s (corner junctions only) |
| `temporal/src/multi/parallel.rs` | fan-out over chains; bisection scoped to corner endpoints |
| `temporal/src/multi/mod.rs` | `BatchInput.initial_accel`; `JunctionBindingCap::ChainInterior`; chain wiring |
| `trajectory/src/lib.rs` | `ShapeBatchInput.initial_a` |
| `trajectory/src/plan_velocity.rs` | `PlanInput.initial_a` + validation |
| `trajectory/src/beta.rs` | first-run `initial_accel` pass-through |
| `trajectory/src/streaming/state.rs` | + `read_path_accel_at`; carry in `append_and_replan` |
| `motion-bridge/src/planner/tests.rs` | RED replan-boundary test |
| `temporal/src/multi/tests.rs` | RED junction-impulse test |

---

### Task 1: Spec amendment + RED integration tests

**Files:**
- Modify: `docs/superpowers/specs/2026-06-07-junction-accel-continuity-design.md`
- Test: `rust/temporal/src/multi/tests.rs`
- Test: `rust/motion-bridge/src/planner/tests.rs`

- [ ] **Step 1: Amend the spec with the rest-start carve-out** (Trap 1 was found while planning). In the "Chain edges" bullet, change:

```text
- **Batch start**: `b_0 = v₀²` and now `a_0 = a₀` via one Zero-cone row
  `b_1 = b_0 + 2h·a_0` (convexity untouched; verified during the
  rest-boundary investigation).
```
to:
```text
- **Batch start**: `b_0 = v₀²` and, **only when `v₀ > 0`**, `a_0 = a₀` via one
  Zero-cone row `b_1 = b_0 + 2h·a_0` (convexity untouched; verified during the
  rest-boundary investigation). At a rest start the pin would force `b_1 = 0`
  — the same trap as the rejected terminal rest pin — so `initial_accel`
  must be 0 there and the (e2) envelope governs instead.
```

- [ ] **Step 2: Write the RED junction-impulse test.** Append to `rust/temporal/src/multi/tests.rs`:

```rust
/// Two cubic Béziers forming one smooth 180° U-turn (tangent-continuous,
/// high κ at the junction). The time-optimal profile decelerates into the
/// junction and accelerates out (V-shape), so with free boundary accels the
/// junction shows a ~2·a_max accel step — an unbounded-jerk impulse.
fn smooth_u_turn() -> (VectorNurbs<f64, 3>, VectorNurbs<f64, 3>) {
    // Quarter-circle-ish cubics of radius 5 mm meeting at the apex (5, 5).
    // Left: (0,0) → (5,5), tangent +X at start, +Y at end.
    // Right: (5,5) → (10,0), tangent +Y at start... NO — for tangent
    // continuity through the apex both tangents at the junction must match.
    // Use the standard circle-approximation control offset k = r·4(√2−1)/3.
    let r = 5.0;
    let k = r * 4.0 * (std::f64::consts::SQRT_2 - 1.0) / 3.0;
    // Left quarter: from (0,0) heading +X, curving up to apex (r, r) heading +Y.
    let left = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [k, 0.0, 0.0],
            [r, r - k, 0.0],
            [r, r, 0.0],
        ],
    )
    .unwrap();
    // Right quarter: from apex (r, r) heading +Y... mirror so it continues
    // the turn: heads +Y at junction, curls to (2r, 2r) heading +X? That
    // changes turn direction. Correct smooth continuation with the SAME
    // junction tangent (+Y): curve from (r,r) to (0, 2r) heading -X (turn
    // continues counterclockwise).
    let right = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [r, r, 0.0],
            [r, r + k, 0.0],
            [k, 2.0 * r, 0.0],
            [0.0, 2.0 * r, 0.0],
        ],
    )
    .unwrap();
    (left, right)
}

#[test]
#[ignore = "RED until the condensed-chain solver lands (Task 11)"]
fn smooth_junction_has_no_accel_impulse() {
    let (left, right) = smooth_u_turn();
    let limits = textbook_limits();
    let segs = [
        SegmentInput {
            curve: &left,
            limits,
            trailing_junction_chord_tolerance_mm: 0.05,
        },
        SegmentInput {
            curve: &right,
            limits,
            trailing_junction_chord_tolerance_mm: 0.05,
        },
    ];
    let out = plan_batch(BatchInput {
        segments: &segs,
        grid_strategy: GridStrategy::Fixed(32),
        worker_threads: 1,
        initial_velocity: 0.0,
        terminal_velocity: 0.0,
    })
    .expect("plan_batch");

    let a_end_left = out.profiles[0].samples.last().unwrap().a;
    let a_start_right = out.profiles[1].samples[0].a;

    // Pre-fix: independent FD endpoints, V-profile makes them differ by
    // O(a_max) — expect this assert to fail with step ≈ 1e3..1e4.
    // Post-fix: structural check only — slicing duplicates the single shared
    // junction variable into both profiles, so step == 0 by construction.
    // The PHYSICAL test is the contract-(b) jerk assertion below.
    let step = (a_end_left - a_start_right).abs();
    assert!(
        step < 1.0,
        "junction accel step {step:.1} mm/s² — boundary accels are decoupled"
    );

    // Contract (b): the junction-spanning discrete jerk obeys j_max. Build
    // the spanning second difference from the two slices (junction sample
    // duplicated, so left[n-2], junction, right[1]).
    let l = &out.profiles[0].samples;
    let r = &out.profiles[1].samples;
    let (bl, bj, br) = (l[l.len() - 2].b, l[l.len() - 1].b, r[1].b);
    let hl = l[l.len() - 1].s - l[l.len() - 2].s;
    let hr = r[1].s - r[0].s;
    let d = hl * hr * (hl + hr);
    let b_dd = (2.0 * hr * bl - 2.0 * (hl + hr) * bj + 2.0 * hl * br) / d;
    let jerk = bj.max(0.0).sqrt() * b_dd / 2.0;
    let j_path = limits.j_max[0].min(limits.j_max[1]).min(limits.j_max[2]);
    assert!(
        jerk.abs() <= j_path * 1.10,
        "junction-spanning jerk {jerk:.0} exceeds j_path {j_path:.0}"
    );
}
```

- [ ] **Step 3: Run it (ignored tests run with `--ignored`) and verify it fails for the right reason:**

Run: `cargo test -p temporal --lib smooth_junction_has_no_accel_impulse -- --ignored`
Expected: FAIL with `junction accel step <large> mm/s² — boundary accels are decoupled` (step on the order of 1e3–1e4). If it fails on geometry construction instead (tangent mismatch, plan error), fix the fixture, not the assertion.

- [ ] **Step 4: Write the RED replan-boundary test.** Append to `rust/motion-bridge/src/planner/tests.rs`, following the harness used by `motion_at_velocity_limit_cruises_at_limit` (same file — `ShaperState`, `classify_and_build`, `append_and_replan`, `emit_committed`).

**Scenario reasoning (load-bearing — do not change the geometry without re-deriving):** the streaming machinery commits only up to `t_decel_start − max_h`, so `t_dispatched` always lands *before* the old plan's decel ramp — the impulse cannot be staged there. The divergence case is the opposite polarity: old plan still *accelerating* at `t_dispatched` (short triangular move A, apex = `t_decel_start`, smoothing shaper so `max_h > 0` puts `t_dispatched` just before the apex, `a_old ≈ +a_max`), then append a long **slow** move B (low feedrate → `per_segment_limits` caps its `v_max` far below A's speed) sized so the new optimal plan must begin shedding speed immediately: free `a_0` flips to ≈ `−a_max`. Post-fix feasibility is guaranteed by the spec's argument (the old plan's own decel-to-zero remainder is always an admissible continuation), so the pinned re-plan decelerates jerk-continuously instead of erroring.

```rust
#[test]
#[ignore = "RED until the replan accel carry lands (Task 13)"]
fn replan_boundary_carries_acceleration() {
    // A: 20 mm at F600 with a_max=5000 → triangular profile (never cruises);
    // emit_committed parks t_dispatched just before the apex where a = +a_max.
    // B: 200 mm at F30 → slow; the new window wants immediate decel.
    let (mut state, ctx) = single_axis_harness(600.0, 5_000.0);
    append_x_move(&mut state, &ctx, 20.0, 600.0);
    let t_split = emit_partial_window(&mut state, &ctx);

    let a_old = sampled_path_accel(&state, t_split);
    assert!(
        a_old > 0.5 * 5_000.0,
        "precondition: t_dispatched must land mid-acceleration \
         (got a={a_old:.0}); resize move A or the emit window, \
         not the assertion"
    );

    append_x_move(&mut state, &ctx, 200.0, 30.0);
    let a_new = sampled_path_accel(&state, t_split);

    assert!(
        (a_new - a_old).abs() < 100.0,
        "replan accel step at t_dispatched: {a_old:.0} -> {a_new:.0} mm/s²"
    );
}
```

Helpers, extracted from the `peak_speed_of_single_x_move` machinery already in this file:
- `single_axis_harness(v_max, a_max)` — the existing `ShaperState` + config construction with a **smoothing** shaper on X (so `max_h > 0`; the existing tests' shaper setup already provides one).
- `append_x_move(&mut state, &ctx, dist_mm, feedrate)` — existing classify/append path with an X-only G5 move.
- `emit_partial_window(&mut state, &ctx) -> f64` — call `emit_committed` once (it advances `t_dispatched` to `t_decel_start − max_h` on its own; no target time is needed) and return `state.t_dispatched`.
- `sampled_path_accel(&state, t) -> f64` — sample the **pre-shape planned profile** (`planned_fitted`: find the `FittedSegment` with `t_start ≤ t < t_end`, evaluate each axis's second derivative via `nurbs::eval`, project tangentially: `(vx·ax + vy·ay)/speed`). The pre-shape profile is what the SOCP pin governs (Task 13) — do NOT sample the shaped axes here.

The precondition assert makes mis-staging loud instead of silently green.

- [ ] **Step 5: Run it and verify the precondition holds and the final assert fails:**

Run: `cargo test -p motion-bridge --lib replan_boundary_carries_acceleration -- --ignored`
Expected: FAIL on the final assert with `a_old ≈ +5000`, `a_new ≈ −5000`. If the precondition fails instead, resize move A (shorter → apex later relative to commit horizon) until `t_dispatched` is mid-acceleration.

- [ ] **Step 6: Commit**

```bash
git add docs/superpowers/specs/2026-06-07-junction-accel-continuity-design.md \
        rust/temporal/src/multi/tests.rs rust/motion-bridge/src/planner/tests.rs
git commit -m "test: RED junction-impulse and replan-accel-carry integration tests

Both #[ignore]-tagged with the un-ignoring task named; both verified to
fail for the intended physical reason (decoupled boundary accels)."
```

---

### Task 2: Non-uniform stencil weights in `stencil.rs`

**Files:**
- Modify: `rust/temporal/src/topp/stencil.rs`
- Test: `rust/temporal/src/topp/stencil/tests.rs`

- [ ] **Step 1: Write failing tests** (append to `stencil/tests.rs`):

```rust
#[test]
fn b_dd_weights_exact_on_quadratic_nonuniform() {
    // b(s) = 3s² − 2s + 1 → b″ = 6 everywhere, any spacing.
    let b = |s: f64| 3.0 * s * s - 2.0 * s + 1.0;
    let (hl, hr) = (0.3, 0.7);
    let s_i = 1.0;
    let w = b_dd_weights(hl, hr);
    let approx = w[0] * b(s_i - hl) + w[1] * b(s_i) + w[2] * b(s_i + hr);
    assert!((approx - 6.0).abs() < 1e-10, "got {approx}");
}

#[test]
fn b_d_weights_exact_on_quadratic_nonuniform() {
    let b = |s: f64| 3.0 * s * s - 2.0 * s + 1.0; // b′(1) = 4
    let (hl, hr) = (0.3, 0.7);
    let w = b_d_weights(hl, hr);
    let approx = w[0] * b(1.0 - hl) + w[1] * b(1.0) + w[2] * b(1.0 + hr);
    assert!((approx - 4.0).abs() < 1e-10, "got {approx}");
}

#[test]
fn weights_reduce_to_uniform() {
    let h = 0.5;
    let wd = b_d_weights(h, h);
    assert!((wd[0] - (-1.0 / (2.0 * h))).abs() < 1e-12);
    assert!(wd[1].abs() < 1e-12);
    assert!((wd[2] - 1.0 / (2.0 * h)).abs() < 1e-12);
    let wdd = b_dd_weights(h, h);
    assert!((wdd[0] - 1.0 / (h * h)).abs() < 1e-12);
    assert!((wdd[1] - (-2.0 / (h * h))).abs() < 1e-12);
    assert!((wdd[2] - 1.0 / (h * h)).abs() < 1e-12);
}

#[test]
fn s_dddot_weights_matches_legacy_uniform() {
    let b = vec![100.0, 144.0, 196.0, 256.0, 324.0];
    let h = 0.25;
    let h_intervals = vec![h; 4];
    for i in 0..5 {
        let legacy = s_dddot_at(&b, i, h);
        let general = s_dddot_at_weights(&b, i, &h_intervals);
        assert!(
            (legacy - general).abs() < 1e-9,
            "i={i}: {legacy} vs {general}"
        );
    }
}
```

- [ ] **Step 2: Run to verify failure**

Run: `cargo test -p temporal --lib stencil`
Expected: compile FAIL — `b_dd_weights` not found.

- [ ] **Step 3: Implement** (append to `stencil.rs`):

```rust
/// 3-point stencil weights for b′(s_i) over spacings (hl, hr); exact for
/// quadratics; order is [w_{i−1}, w_i, w_{i+1}].
pub fn b_d_weights(hl: f64, hr: f64) -> [f64; 3] {
    debug_assert!(hl > 0.0 && hr > 0.0);
    let d = hl * hr * (hl + hr);
    [-hr * hr / d, (hr * hr - hl * hl) / d, hl * hl / d]
}

/// 3-point stencil weights for b″(s_i) over spacings (hl, hr); exact for
/// quadratics; O(h) truncation when hl ≠ hr, O(h²) when equal.
pub fn b_dd_weights(hl: f64, hr: f64) -> [f64; 3] {
    debug_assert!(hl > 0.0 && hr > 0.0);
    let d = hl * hr * (hl + hr);
    [2.0 * hr / d, -2.0 * (hl + hr) / d, 2.0 * hl / d]
}

/// Stencil index triple and spacings for point `i` of a grid with
/// per-interval spacings `h_intervals` (len = n−1). Boundary points return
/// the 3-point stencil anchored at the edge — for b″ this matches the legacy
/// one-sided second difference (3-point second-difference weights are
/// anchor-position-independent); for b′ it is a forward/backward 3-point
/// approximation, NOT the 2-point one-sided difference block (b) uses for
/// its edge rows (those stay 2-point on purpose — bit-equivalence with the
/// legacy bundle).
pub fn stencil_at(i: usize, n: usize, h_intervals: &[f64]) -> ([usize; 3], f64, f64) {
    debug_assert!(n >= 3 && i < n && h_intervals.len() == n - 1);
    if i == 0 {
        ([0, 1, 2], h_intervals[0], h_intervals[1])
    } else if i == n - 1 {
        ([n - 3, n - 2, n - 1], h_intervals[n - 3], h_intervals[n - 2])
    } else {
        ([i - 1, i, i + 1], h_intervals[i - 1], h_intervals[i])
    }
}

/// `s‴_i = √b_i · b″(s_i) / 2` with non-uniform-capable weights.
pub fn s_dddot_at_weights(b: &[f64], i: usize, h_intervals: &[f64]) -> f64 {
    let n = b.len();
    let (idx, hl, hr) = stencil_at(i, n, h_intervals);
    let w = b_dd_weights(hl, hr);
    let b_dd = w[0] * b[idx[0]] + w[1] * b[idx[1]] + w[2] * b[idx[2]];
    b[i].max(0.0).sqrt() * b_dd / 2.0
}
```

- [ ] **Step 4: Run to verify pass**

Run: `cargo test -p temporal --lib stencil`
Expected: PASS (all new + existing stencil tests).

- [ ] **Step 5: Commit**

```bash
git add rust/temporal/src/topp/stencil.rs rust/temporal/src/topp/stencil/tests.rs
git commit -m "temporal: non-uniform 3-point stencil weights for b', b'', s-dddot"
```

---

### Task 3: `ChainGrid` and chain assembly (`topp/chain.rs`)

**Files:**
- Create: `rust/temporal/src/topp/chain.rs`
- Create: `rust/temporal/src/topp/chain/tests.rs`
- Modify: `rust/temporal/src/topp/mod.rs` (add `pub mod chain;`)
- Modify: `rust/temporal/src/topp/scaling.rs` (+ `for_chain`, `scale_chain_grid`)
- Test: `rust/temporal/src/topp/scaling/tests.rs`

- [ ] **Step 1: Write failing tests** (`chain/tests.rs`):

```rust
use super::*;
use crate::Limits;
use crate::topp::path::sample_arclength_grid;
use nurbs::VectorNurbs;

fn line(from: [f64; 3], to: [f64; 3]) -> VectorNurbs<f64, 3> {
    VectorNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![from, to]).unwrap()
}

fn lim(v: f64) -> Limits {
    Limits {
        v_max: [v; 3],
        a_max: [5_000.0; 3],
        j_max: [100_000.0; 3],
        a_centripetal_max: 2_500.0,
    }
}

#[test]
fn single_segment_chain_mirrors_arclength_grid() {
    let c = line([0.0; 3], [50.0, 0.0, 0.0]);
    let g = sample_arclength_grid(&c, 11).unwrap();
    let chain = ChainGrid::from_segment_grids(vec![g.clone()], vec![lim(300.0)]);
    assert_eq!(chain.s, g.s);
    assert_eq!(chain.h_intervals.len(), 10);
    assert!(chain.junctions.is_empty());
    assert_eq!(chain.segment_ranges, vec![(0, 10)]);
    assert_eq!(chain.geom[3].c_prime, g.c_prime[3]);
    assert!(chain.limits_idx.iter().all(|&i| i == 0));
}

#[test]
fn two_segment_chain_shares_junction_point() {
    let a = line([0.0; 3], [40.0, 0.0, 0.0]);
    let b = line([40.0, 0.0, 0.0], [100.0, 0.0, 0.0]);
    let ga = sample_arclength_grid(&a, 11).unwrap(); // h = 4
    let gb = sample_arclength_grid(&b, 13).unwrap(); // h = 5
    let chain =
        ChainGrid::from_segment_grids(vec![ga, gb], vec![lim(300.0), lim(200.0)]);

    // M = 11 + 13 − 1 shared point.
    assert_eq!(chain.s.len(), 23);
    assert_eq!(chain.h_intervals.len(), 22);
    // s is cumulative and strictly increasing through the junction.
    assert!((chain.s[10] - 40.0).abs() < 1e-9);
    assert!((chain.s[22] - 100.0).abs() < 1e-9);
    assert!((chain.h_intervals[9] - 4.0).abs() < 1e-9);
    assert!((chain.h_intervals[10] - 5.0).abs() < 1e-9);
    // One junction marker at the shared index carrying the right side.
    assert_eq!(chain.junctions.len(), 1);
    assert_eq!(chain.junctions[0].idx, 10);
    assert_eq!(chain.junctions[0].limits_idx, 1);
    // Primary arrays carry the LEFT side at the junction.
    assert_eq!(chain.limits_idx[10], 0);
    assert_eq!(chain.segment_ranges, vec![(0, 10), (10, 22)]);
}
```

- [ ] **Step 2: Run to verify compile failure**

Run: `cargo test -p temporal --lib topp::chain`
Expected: FAIL — module/type not found.

- [ ] **Step 3: Implement `chain.rs`:**

```rust
use crate::Limits;
use crate::topp::path::ArclengthGrid;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PointGeom {
    pub c_prime: [f64; 3],
    pub c_double_prime: [f64; 3],
    pub c_triple_prime: [f64; 3],
    pub kappa: f64,
}

/// Right-side geometry/limits of a shared junction point. The primary
/// per-point arrays carry the left side.
#[derive(Debug, Clone, Copy)]
pub struct JunctionDual {
    pub idx: usize,
    pub geom: PointGeom,
    pub limits_idx: usize,
}

#[derive(Debug, Clone)]
pub struct ChainGrid {
    /// Cumulative arclength along the chain, len M.
    pub s: Vec<f64>,
    pub geom: Vec<PointGeom>,
    /// Per-interval spacing, len M−1. Uniform within a segment, changes at
    /// junction indices.
    pub h_intervals: Vec<f64>,
    /// Index into `limits` per point (junction points → left segment).
    pub limits_idx: Vec<usize>,
    pub limits: Vec<Limits>,
    pub junctions: Vec<JunctionDual>,
    /// Inclusive (start, end) point-index range per segment; consecutive
    /// ranges share their boundary index.
    pub segment_ranges: Vec<(usize, usize)>,
}

impl ChainGrid {
    /// Concatenate per-segment grids into one chain. Adjacent grids must be
    /// geometrically continuous (the caller guarantees tangent continuity —
    /// that's what made them one chain). Panics on empty input: an empty
    /// chain is a caller bug.
    pub fn from_segment_grids(grids: Vec<ArclengthGrid>, limits: Vec<Limits>) -> Self {
        assert_eq!(grids.len(), limits.len());
        assert!(!grids.is_empty(), "empty chain");

        let mut s = Vec::new();
        let mut geom = Vec::new();
        let mut h_intervals = Vec::new();
        let mut limits_idx = Vec::new();
        let mut junctions = Vec::new();
        let mut segment_ranges = Vec::new();
        let mut s_offset = 0.0;

        for (seg, g) in grids.iter().enumerate() {
            let n = g.s.len();
            debug_assert!(n >= 2);
            let h_seg = g.s[1] - g.s[0];
            let start_point = if seg == 0 { 0 } else { 1 };
            let range_start = if seg == 0 { 0 } else { s.len() - 1 };

            if seg > 0 {
                // The shared point already exists with LEFT geometry; record
                // the right side as a dual.
                junctions.push(JunctionDual {
                    idx: s.len() - 1,
                    geom: point_geom(g, 0),
                    limits_idx: seg,
                });
            }
            for i in start_point..n {
                s.push(s_offset + g.s[i]);
                geom.push(point_geom(g, i));
                limits_idx.push(seg);
            }
            for _ in 0..n - 1 {
                h_intervals.push(h_seg);
            }
            segment_ranges.push((range_start, s.len() - 1));
            s_offset += g.total_length;
        }

        Self {
            s,
            geom,
            h_intervals,
            limits_idx,
            limits,
            junctions,
            segment_ranges,
        }
    }

    pub fn n_points(&self) -> usize {
        self.s.len()
    }

    pub fn limits_at(&self, i: usize) -> &Limits {
        &self.limits[self.limits_idx[i]]
    }
}
```

Add the spec's spacing-ratio guard at the end of `from_segment_grids`, before constructing `Self` (an extreme ratio is a grid-construction bug — fail loudly, per spec "Verification" §Claim 2):

```rust
const MAX_JUNCTION_SPACING_RATIO: f64 = 16.0;
for j_idx in 1..grids.len() {
    let hl = grids[j_idx - 1].s[1] - grids[j_idx - 1].s[0];
    let hr = grids[j_idx].s[1] - grids[j_idx].s[0];
    let ratio = (hl / hr).max(hr / hl);
    assert!(
        ratio <= MAX_JUNCTION_SPACING_RATIO,
        "junction spacing ratio {ratio:.1} (hl={hl:.4}, hr={hr:.4}) — \
         grid construction bug; the non-uniform stencil conditioning \
         degrades with the spacing ratio"
    );
}
```

with a `#[should_panic(expected = "junction spacing ratio")]` test pairing two grids of wildly different `n` over similar lengths. Also add the shared test-fixture module at the bottom of `chain.rs` (used by constraints/verify/topp/scaling tests across Tasks 4–8):

```rust
#[cfg(test)]
pub(crate) mod tests_support {
    use super::ChainGrid;
    use crate::Limits;
    use nurbs::VectorNurbs;

    pub(crate) fn line(from: [f64; 3], to: [f64; 3]) -> VectorNurbs<f64, 3> {
        VectorNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![from, to]).unwrap()
    }

    pub(crate) fn line_50mm() -> VectorNurbs<f64, 3> {
        line([0.0; 3], [50.0, 0.0, 0.0])
    }

    /// 40 mm + 60 mm collinear lines with different v_max per side
    /// (300 / 150 mm/s); 11 + 13 grid points → junction at index 10,
    /// h = 4 mm left, 5 mm right.
    pub(crate) fn two_segment_chain_with_junction() -> ChainGrid {
        let ga =
            crate::topp::path::sample_arclength_grid(&line([0.0; 3], [40.0, 0.0, 0.0]), 11)
                .unwrap();
        let gb = crate::topp::path::sample_arclength_grid(
            &line([40.0, 0.0, 0.0], [100.0, 0.0, 0.0]),
            13,
        )
        .unwrap();
        let lim = |v: f64| Limits {
            v_max: [v; 3],
            a_max: [5_000.0; 3],
            j_max: [100_000.0; 3],
            a_centripetal_max: 2_500.0,
        };
        ChainGrid::from_segment_grids(vec![ga, gb], vec![lim(300.0), lim(150.0)])
    }
}

fn point_geom(g: &ArclengthGrid, i: usize) -> PointGeom {
    PointGeom {
        c_prime: g.c_prime[i],
        c_double_prime: g.c_double_prime[i],
        c_triple_prime: g.c_triple_prime[i],
        kappa: g.kappa[i],
    }
}

#[cfg(test)]
mod tests;
```

Register in `topp/mod.rs` next to the other modules: `pub mod chain;`.

- [ ] **Step 4: Run to verify pass**

Run: `cargo test -p temporal --lib topp::chain`
Expected: PASS.

- [ ] **Step 5: Scaling support — failing test first** (append to `scaling/tests.rs`):

```rust
#[test]
fn chain_grid_scaling_matches_arclength_grid_scaling() {
    let c = crate::topp::chain::tests_support::line_50mm();
    let g = crate::topp::path::sample_arclength_grid(&c, 9).unwrap();
    let lims = crate::Limits {
        v_max: [1000.0; 3],
        a_max: [50_000.0; 3],
        j_max: [100_000.0; 3],
        a_centripetal_max: 50_000.0,
    };
    let chain =
        crate::topp::chain::ChainGrid::from_segment_grids(vec![g.clone()], vec![lims]);
    let scale = SolverScale::for_chain(&chain);
    let sg = scale.scale_grid(&g);
    let sc = scale.scale_chain_grid(&chain);
    assert_eq!(sc.s, sg.s);
    for i in 0..sc.n_points() {
        assert_eq!(sc.geom[i].c_double_prime, sg.c_double_prime[i]);
        assert_eq!(sc.geom[i].c_triple_prime, sg.c_triple_prime[i]);
        assert_eq!(sc.geom[i].kappa, sg.kappa[i]);
    }
    assert!((sc.h_intervals[0] - (sg.s[1] - sg.s[0])).abs() < 1e-15);
}
```

(Add a tiny `pub(crate) mod tests_support` in `chain.rs` with `line_50mm()` so fixtures are shared, or inline the curve construction — either is fine; do not duplicate the magic numbers more than once.)

- [ ] **Step 6: Implement scaling** (append to `scaling.rs`):

```rust
impl SolverScale {
    pub fn for_chain(chain: &crate::topp::chain::ChainGrid) -> Self {
        let sigma = chain
            .limits
            .iter()
            .flat_map(|l| l.v_max.iter().copied())
            .filter(|v| v.is_finite() && *v > 0.0)
            .fold(f64::NEG_INFINITY, f64::max);
        if sigma <= 0.0 || !sigma.is_finite() {
            return Self::identity();
        }
        Self {
            mm_per_unit: sigma / V_TARGET_UNITS_PER_S,
        }
    }

    pub(crate) fn scale_chain_grid(
        &self,
        chain: &crate::topp::chain::ChainGrid,
    ) -> crate::topp::chain::ChainGrid {
        let s = self.sigma();
        let scale_geom = |g: &crate::topp::chain::PointGeom| crate::topp::chain::PointGeom {
            c_prime: g.c_prime,
            c_double_prime: g.c_double_prime.map(|v| v * s),
            c_triple_prime: g.c_triple_prime.map(|v| v * s * s),
            kappa: g.kappa * s,
        };
        crate::topp::chain::ChainGrid {
            s: chain.s.iter().map(|v| v / s).collect(),
            geom: chain.geom.iter().map(scale_geom).collect(),
            h_intervals: chain.h_intervals.iter().map(|h| h / s).collect(),
            limits_idx: chain.limits_idx.clone(),
            limits: chain.limits.iter().map(|l| self.scale_limits(l)).collect(),
            junctions: chain
                .junctions
                .iter()
                .map(|j| crate::topp::chain::JunctionDual {
                    idx: j.idx,
                    geom: scale_geom(&j.geom),
                    limits_idx: j.limits_idx,
                })
                .collect(),
            segment_ranges: chain.segment_ranges.clone(),
        }
    }
}
```

- [ ] **Step 7: Run to verify pass, then commit**

Run: `cargo test -p temporal --lib`
Expected: PASS.

```bash
git add rust/temporal/src/topp/chain.rs rust/temporal/src/topp/chain/tests.rs \
        rust/temporal/src/topp/mod.rs rust/temporal/src/topp/scaling.rs \
        rust/temporal/src/topp/scaling/tests.rs
git commit -m "temporal: ChainGrid — concatenated multi-segment grid with junction duals"
```

---

### Task 4: `constraints::build_chain` — chain-of-1 equivalence first

**Files:**
- Modify: `rust/temporal/src/topp/constraints.rs`
- Test: `rust/temporal/src/topp/constraints/tests.rs`

This task ports `build()` to consume `&ChainGrid` as `build_chain()`, with general per-point stencils and per-point `h̄`, **emitting bit-identical bundles for single uniform segments**. The legacy `build()` stays alive until Task 8.

- [ ] **Step 1: Write the failing equivalence test** (append to `constraints/tests.rs`):

```rust
#[test]
fn build_chain_of_one_emits_identical_bundle() {
    let curve = crate::topp::chain::tests_support::line_50mm();
    let grid = crate::topp::path::sample_arclength_grid(&curve, 16).unwrap();
    let limits = crate::Limits {
        v_max: [300.0; 3],
        a_max: [5_000.0; 3],
        j_max: [100_000.0; 3],
        a_centripetal_max: 2_500.0,
    };
    let scale = SolverScale::identity();
    let endpoints = EndpointConditions {
        v_start: 10.0,
        v_end: 0.0,
        a_start: None,
    };
    let legacy = match build(
        &grid,
        &limits,
        EndpointVelocities {
            v_start: 10.0,
            v_end: 0.0,
        },
        &scale,
    ) {
        BuildOutcome::Ok(b) => b,
        other => panic!("legacy build failed: {other:?}"),
    };
    let chain = crate::topp::chain::ChainGrid::from_segment_grids(vec![grid], vec![limits]);
    let new = match build_chain(&chain, endpoints, &scale) {
        BuildOutcome::Ok(b) => b,
        other => panic!("chain build failed: {other:?}"),
    };
    assert_eq!(legacy.n_vars, new.n_vars);
    assert_eq!(legacy.cones, new.cones);
    assert_eq!(legacy.b_rhs.len(), new.b_rhs.len());
    for (i, (lr, nr)) in legacy.a_rows.iter().zip(&new.a_rows).enumerate() {
        for (j, (lv, nv)) in lr.iter().zip(nr).enumerate() {
            assert!(
                (lv - nv).abs() < 1e-12,
                "row {i} col {j}: {lv} vs {nv}"
            );
        }
    }
    for (i, (lv, nv)) in legacy.b_rhs.iter().zip(&new.b_rhs).enumerate() {
        assert!((lv - nv).abs() < 1e-12, "rhs {i}: {lv} vs {nv}");
    }
}
```

- [ ] **Step 2: Run to verify compile failure** (`EndpointConditions`, `build_chain` missing).

Run: `cargo test -p temporal --lib constraints`

- [ ] **Step 3: Implement.** In `constraints.rs`:

(3a) New endpoint type (keep `EndpointVelocities` until Task 8 deletes it with legacy `build`):

```rust
/// Chain-edge boundary conditions. `a_start = Some(_)` is only legal with
/// `v_start > 0` — pinning accel at a rest start forces `b_1 = 0` (the
/// rejected rest-pin trap); `build_chain` panics on it as a caller bug.
#[derive(Debug, Clone, Copy)]
pub struct EndpointConditions {
    pub v_start: f64,
    pub v_end: f64,
    pub a_start: Option<f64>,
}
```

(3b) `build_chain(chain: &ChainGrid, endpoints: EndpointConditions, scale: &SolverScale) -> BuildOutcome`, mirroring `build()` (constraints.rs:85-492) with these exact deltas — everything not listed is a verbatim port with `grid.X[i]` → `chain.geom[i].X` / `chain.s[i]` and `limits` → `chain.limits_at(i)`:

```rust
// Replaces the scalar `h` (legacy line 171):
let n = chain.n_points();
let h = &chain.h_intervals; // len n−1
let h_bar = |i: usize| -> f64 {
    if i == 0 {
        h[0]
    } else if i == n - 1 {
        h[n - 2]
    } else {
        0.5 * (h[i - 1] + h[i])
    }
};

// b_max_cent (legacy 96-106): per-point limits; junction duals tighten.
let mut b_max_cent: Vec<f64> = (0..n)
    .map(|i| {
        let k = chain.geom[i].kappa;
        if k.abs() < kappa_floor {
            b_cap
        } else {
            (chain.limits_at(i).a_centripetal_max / k.abs()).min(b_cap)
        }
    })
    .collect();
for j in &chain.junctions {
    let k = j.geom.kappa;
    let cap = if k.abs() < kappa_floor {
        b_cap
    } else {
        (chain.limits[j.limits_idx].a_centripetal_max / k.abs()).min(b_cap)
    };
    b_max_cent[j.idx] = b_max_cent[j.idx].min(cap);
}

// a_env/j_env scan (legacy 113-138): same loop, but inner uses
// chain.geom[i].c_prime and chain.limits_at(i); afterwards extend with one
// extra pass over chain.junctions (dual geometry + dual limits) so the
// envelope stays an over-estimate at junction points.

// j_path (legacy 178): min over ALL chain segments' axes:
let j_path = chain
    .limits
    .iter()
    .flat_map(|l| l.j_max.iter().copied())
    .fold(f64::INFINITY, f64::min);
```

Block (a) — unchanged, plus the pin:

```rust
// Block (a): boundary equalities + optional a_start pin.
{
    let mut count = 0_usize;
    push_row(&mut a_rows, &mut b_rhs, &[(off_b, 1.0)], -b_start);
    count += 1;
    push_row(&mut a_rows, &mut b_rhs, &[(off_b + n - 1, 1.0)], -b_end);
    count += 1;
    if let Some(a0) = endpoints.a_start {
        assert!(
            endpoints.v_start > 0.0,
            "a_start pin at a rest start forces b_1 = 0 (rejected trap); \
             rest starts use the (e2) envelope"
        );
        // b_1 − b_0 − 2·h_0·a_0 = 0  (Zero cone; b_0 already pinned).
        push_row(
            &mut a_rows,
            &mut b_rhs,
            &[(off_b + 1, 1.0), (off_b, -1.0)],
            -2.0 * h[0] * a0,
        );
        count += 1;
    }
    cones.push((Cone::Zero, count));
}
```

Block (b) — general weights everywhere (replaces legacy 206-245). The one-sided edge rows keep the legacy forward/backward form (they are the `stencil_at` boundary triple with `b_d` reduced — emit exactly the legacy two-coefficient rows at `i = 0` and `i = n−1` so the equivalence test passes bit-exactly; document that they are the `hl = hr` one-sided first-difference):

```rust
// Block (b): a_i = ½·b′(s_i); interior uses non-uniform central weights,
// edges keep one-sided first differences over their adjacent interval.
{
    let count = n;
    push_row(
        &mut a_rows,
        &mut b_rhs,
        &[
            (off_a, 1.0),
            (off_b + 1, -1.0 / (2.0 * h[0])),
            (off_b, 1.0 / (2.0 * h[0])),
        ],
        0.0,
    );
    for i in 1..n - 1 {
        let w = crate::topp::stencil::b_d_weights(h[i - 1], h[i]);
        push_row(
            &mut a_rows,
            &mut b_rhs,
            &[
                (off_a + i, 1.0),
                (off_b + i - 1, -w[0] / 2.0),
                (off_b + i, -w[1] / 2.0),
                (off_b + i + 1, -w[2] / 2.0),
            ],
            0.0,
        );
    }
    push_row(
        &mut a_rows,
        &mut b_rhs,
        &[
            (off_a + n - 1, 1.0),
            (off_b + n - 1, -1.0 / (2.0 * h[n - 2])),
            (off_b + n - 2, 1.0 / (2.0 * h[n - 2])),
        ],
        0.0,
    );
    cones.push((Cone::Zero, count));
}
```

(The uniform interior reduction gives `w[1] = 0` and `∓1/(2h)` — identical rows to legacy, so the equivalence test stays bit-exact within 1e-12.)

Blocks (c), (d), (e) — same loops over `chain.geom[i]` + `chain.limits_at(i)`. Junction dual rows are part of Task 5 (so this task stays bit-equivalent for chains of 1).

Block (e2) — same two loops, with `d` computed from `chain.s` and the per-point envelope unchanged.

Block (f) — general weights + local `h̄` (replaces legacy 361-398):

```rust
// Block (f): t_i ≥ ±h̄_i·b″_i/(2·J_path), b″ via non-uniform weights.
// Uniform reduction: ±Δ²b/(2hJ) — identical to legacy.
{
    let mut count = 0_usize;
    for k in 0..n_interior {
        let i = k + 1;
        let t_idx = off_t + k;
        let w = crate::topp::stencil::b_dd_weights(h[i - 1], h[i]);
        let c = h_bar(i) / (2.0 * j_path);
        for sign in [1.0_f64, -1.0] {
            push_row(
                &mut a_rows,
                &mut b_rhs,
                &[
                    (t_idx, 1.0),
                    (off_b + i - 1, -sign * c * w[0]),
                    (off_b + i, -sign * c * w[1]),
                    (off_b + i + 1, -sign * c * w[2]),
                ],
                0.0,
            );
            count += 1;
        }
    }
    if count > 0 {
        cones.push((Cone::Nonneg, count));
    }
}
```

Block (g) — unchanged. Block (h) — replace every `h` with the point-local `h̄(k+1)` (`sqrt_h` becomes per-point; the `2.0 * h` RHS in H3 becomes `2.0 * h_bar(k + 1)`; H1's `±h` likewise). Objective — unchanged (`Σ t`); the time-meaning now comes from per-point `h̄` inside the cones (verifier item 5a).

`ConstraintBundle` — replace `pub h: f64` with `pub h_intervals: Vec<f64>` (legacy `build` fills it with `vec![h; n-1]`), keep `j_path`. Fix **all four** legacy `bundle.h` reads in solver.rs — lines 496 (`solve_with_cuts_and_trust_region`), 708 (`slp_solve`), 925 and 948 (`slp_solve_with_axis_jerk`) — each with:

```rust
let h = bundle.h_intervals[0];
debug_assert!(
    bundle.h_intervals.iter().all(|&hi| (hi - h).abs() < 1e-12),
    "legacy SLP path requires uniform spacing"
);
```

(`topp/mod.rs`'s `h_phys` reads `arc_grid` directly — unaffected.) Also in this task's block (d) primary loop: the `BLOCK_D_SAFETY` vacuous-row gate computes `a_cap_i = b_cap_i / (2.0 * h)` in legacy — in `build_chain` it must use `b_cap_i / (2.0 * h_bar(i))` (the gate's accel scale is the point-local cell width; the junction dual rows in Task 5 use the same).

- [ ] **Step 4: Run the equivalence test + full temporal suite**

Run: `cargo test -p temporal`
Expected: PASS — bit-exact bundle equivalence, and every existing fixture still green via the untouched legacy path.

- [ ] **Step 5: Commit**

```bash
git add rust/temporal/src/topp/constraints.rs rust/temporal/src/topp/constraints/tests.rs \
        rust/temporal/src/topp/solver.rs rust/temporal/src/topp/mod.rs
git commit -m "temporal: build_chain — chain-grid SOCP build, bit-equivalent for chains of 1

Interval-local h in blocks (b)/(f)/(g)/(h) and the SOC time chain per the
verifier's item 5a; a_start Zero-cone pin with the rest-start trap asserted."
```

---

### Task 5: Junction dual rows + e2 gating + pin tests

**Files:**
- Modify: `rust/temporal/src/topp/constraints.rs`
- Test: `rust/temporal/src/topp/constraints/tests.rs`

- [ ] **Step 1: Failing tests** (append to `constraints/tests.rs`; `two_segment_chain_with_junction` comes from `crate::topp::chain::tests_support` — import it, don't redefine):

```rust
use crate::topp::chain::tests_support::two_segment_chain_with_junction;

#[test]
fn junction_point_gets_dual_velocity_rows() {
    let chain = two_segment_chain_with_junction();
    let bundle = match build_chain(
        &chain,
        EndpointConditions { v_start: 0.0, v_end: 0.0, a_start: None },
        &SolverScale::identity(),
    ) {
        BuildOutcome::Ok(b) => b,
        other => panic!("{other:?}"),
    };
    // The junction b-column must appear in at least two block-(c)-style
    // velocity rows with DIFFERENT RHS (300² vs 150² projected): count rows
    // whose only nonzero is −1 at off_b+10 with rhs in {300², 150²}.
    let jidx = 10;
    let mut rhs_seen = vec![];
    for (row, rhs) in bundle.a_rows.iter().zip(&bundle.b_rhs) {
        let nz: Vec<usize> = row
            .iter()
            .enumerate()
            .filter(|(_, v)| **v != 0.0)
            .map(|(c, _)| c)
            .collect();
        if nz == vec![jidx] && (row[jidx] + 1.0).abs() < 1e-12 {
            rhs_seen.push(*rhs);
        }
    }
    assert!(
        rhs_seen.iter().any(|r| (r - 300.0_f64.powi(2)).abs() < 1e-6),
        "missing left-side velocity row at junction: {rhs_seen:?}"
    );
    assert!(
        rhs_seen.iter().any(|r| (r - 150.0_f64.powi(2)).abs() < 1e-6),
        "missing right-side velocity row at junction: {rhs_seen:?}"
    );
}

#[test]
#[should_panic(expected = "a_start pin at a rest start")]
fn a_start_pin_at_rest_panics() {
    let chain = two_segment_chain_with_junction();
    let _ = build_chain(
        &chain,
        EndpointConditions { v_start: 0.0, v_end: 0.0, a_start: Some(100.0) },
        &SolverScale::identity(),
    );
}

#[test]
fn a_start_pin_emits_zero_cone_row() {
    let chain = two_segment_chain_with_junction();
    let bundle = match build_chain(
        &chain,
        EndpointConditions { v_start: 50.0, v_end: 0.0, a_start: Some(-2_000.0) },
        &SolverScale::identity(),
    ) {
        BuildOutcome::Ok(b) => b,
        other => panic!("{other:?}"),
    };
    // Zero cone now has 3 rows (b_0, b_{N−1}, pin).
    assert_eq!(bundle.cones[0], (Cone::Zero, 3));
    let pin = &bundle.a_rows[2];
    assert!((pin[1] - 1.0).abs() < 1e-12 && (pin[0] + 1.0).abs() < 1e-12);
    let h0 = chain.h_intervals[0];
    assert!((bundle.b_rhs[2] - (-2.0 * h0 * -2_000.0)).abs() < 1e-9);
}
```

- [ ] **Step 2: Run, verify the dual-rows test fails** (the pin tests pass already from Task 4 — that is fine; the dual-row emission is what's missing).

Run: `cargo test -p temporal --lib constraints`

- [ ] **Step 3: Implement dual rows.** In `build_chain`, immediately after the primary block (c) loop, add a junction pass; same for (d). Block (e) needs no extra rows — `b_max_cent` already took the min of both sides in Task 4:

```rust
// Junction duals: the shared point belongs to both segments; emit the
// right side's velocity and accel rows too (per-axis caps with the right
// segment's geometry AND limits).
for j in &chain.junctions {
    let i = j.idx;
    let lims = &chain.limits[j.limits_idx];
    for ax in 0..3 {
        let g = j.geom.c_prime[ax];
        if g.abs() < COMP_FLOOR {
            continue;
        }
        let rhs = (lims.v_max[ax] / g).powi(2);
        if rhs > b_cap {
            continue;
        }
        push_row(&mut a_rows, &mut b_rhs, &[(off_b + i, -1.0)], rhs);
        count += 1;
    }
}
```

(and the analogous loop inside block (d) using `j.geom.c_prime` / `j.geom.c_double_prime` / `lims.a_max`, with the same `BLOCK_D_SAFETY` vacuous-row skip computed from the junction's own `b_max_cent[i]` and `h_bar(i)`).

Block (e2): confirm the gate reads `endpoints.v_start == 0.0` / `endpoints.v_end == 0.0` — it already does (ported in Task 4); add a test only if you changed it.

- [ ] **Step 4: Run to verify pass**

Run: `cargo test -p temporal`
Expected: PASS, including the Task-4 equivalence test (a chain of 1 has no junctions — no dual rows — still bit-exact).

- [ ] **Step 5: Commit**

```bash
git add rust/temporal/src/topp/constraints.rs rust/temporal/src/topp/constraints/tests.rs
git commit -m "temporal: junction dual rows — both sides' velocity/accel caps at shared points"
```

---

### Task 6: Solver generalization — weights-based cuts, per-point limits

**Files:**
- Modify: `rust/temporal/src/topp/solver.rs`
- Test: `rust/temporal/src/topp/solver/tests.rs`, `rust/temporal/tests/step9_cut_identity.rs`

The three-case `AxisJerkStencil` match (`solver.rs:278-361`) collapses into one weights-based path; the `AxisJerkCut` struct gains the stencil triple and weights. New chain-aware SLP entry points are added **alongside** the legacy ones (deleted in Task 8).

- [ ] **Step 1: Failing test — cut identity at a non-uniform junction.** Append to `rust/temporal/tests/step9_cut_identity.rs` (follow the existing identity-test pattern in that file: evaluate the analytic per-axis jerk at a perturbed iterate and compare against the linearized cut's prediction):

```rust
// NOTE: `topp::solver` is `pub(crate)` — the gradient helper is re-exported
// from `topp`, so the path here is temporal::topp::axis_jerk_gradient_for_test
// (NOT ...::solver::...). `topp::stencil` is already `pub mod`.
#[test]
fn axis_jerk_cut_identity_nonuniform_spacing() {
    // Anchor point i with hl=0.3, hr=0.7; random-ish but fixed iterate.
    let (hl, hr) = (0.3, 0.7);
    let h_intervals = [hl, hr];
    let b_bars = [900.0, 1_000.0, 1_150.0];
    let a_bar = 180.0;
    let (cp, cpp, cppp) = (0.8, 0.05, 0.002);

    let j_at = |b: [f64; 3], a: f64| -> f64 {
        let w = temporal::topp::stencil::b_dd_weights(hl, hr);
        let b_dd = w[0] * b[0] + w[1] * b[1] + w[2] * b[2];
        let s = b[1].max(0.0).sqrt();
        cppp * s * s * s + 3.0 * cpp * a * s + cp * (s * b_dd / 2.0)
    };

    // The cut is the first-order Taylor expansion at (b̄, ā): for small
    // perturbations the cut's linear prediction must match the analytic
    // jerk to second order.
    let f0 = j_at(b_bars, a_bar);
    let grad = temporal::topp::axis_jerk_gradient_for_test(
        &b_bars, a_bar, cp, cpp, cppp, &h_intervals,
    );
    let db = [1.5, -2.0, 1.0];
    let da = 0.5;
    let pred = f0
        + grad.b[0] * db[0]
        + grad.b[1] * db[1]
        + grad.b[2] * db[2]
        + grad.a * da;
    let actual = j_at(
        [b_bars[0] + db[0], b_bars[1] + db[1], b_bars[2] + db[2]],
        a_bar + da,
    );
    assert!(
        (pred - actual).abs() < 1e-2 * actual.abs().max(1.0),
        "linearization off: pred {pred}, actual {actual}"
    );
}
```

- [ ] **Step 2: Run to verify compile failure** (`axis_jerk_gradient_for_test` missing).

Run: `cargo test -p temporal --test step9_cut_identity`

- [ ] **Step 3: Implement the weights-based cut.** In `solver.rs`:

(3a) Rework `AxisJerkCut`:

```rust
#[derive(Debug, Clone, Copy)]
pub(crate) struct AxisJerkCut {
    /// Anchor grid index (where the jerk is evaluated).
    pub i: usize,
    #[allow(dead_code)]
    pub axis: usize,
    /// The three stencil indices (from `stencil::stencil_at`).
    pub idx: [usize; 3],
    /// b″ weights for the stencil (from `stencil::b_dd_weights`).
    pub w: [f64; 3],
    /// Iterate b̄ at the three stencil indices.
    pub b_bars: [f64; 3],
    pub a_bar_i: f64,
    pub cp: f64,
    pub cpp: f64,
    pub cppp: f64,
    pub j_lim_inflated: f64,
}
```

(3b) Replace the body of `append_axis_jerk_cut_to_clarabel` (`solver.rs:258-407`) with the unified weight form. With `S = √(max(b̄_anchor, b_floor))`, `S3 = b̄_anchor·S`, `b̄″ = w·b̄` (dot product), and `anchor_pos` = position of `cut.i` inside `cut.idx`:

```text
coefficient on b at idx[k], k ≠ anchor_pos:   c′·S·w[k]/2
coefficient on b at the anchor:               (3/2)·c‴·S + 3·c″·ā/(2S)
                                              + c′·[w[anchor_pos]·S/2 + b̄″/(4S)]
coefficient on a_i:                           3·c″·S
K (constant):                                 −(1/2)·c‴·S3 − (3/2)·c″·ā·S − c′·b̄″·S/4
rows:  ±(coeffs)·x ≤ j_lim_inflated ∓ K   (two Nonneg rows, exactly as today)
```

Keep the row-∞-norm scaling block (`solver.rs:371-385`) verbatim — it applies unchanged to the new coefficients. Uniform-spacing sanity: with `w = (1/h², −2/h², 1/h²)` these reduce exactly to the legacy three-case coefficients (the legacy `D₂/h²` is `b̄″`).

(3c) Export the gradient for the identity test:

```rust
pub struct AxisJerkGradient {
    pub b: [f64; 3],
    pub a: f64,
}

/// Test-only: the cut's gradient at an iterate (same formulas the cut rows
/// use). h_intervals are the two spacings around the anchor.
pub fn axis_jerk_gradient_for_test(
    b_bars: &[f64; 3],
    a_bar: f64,
    cp: f64,
    cpp: f64,
    cppp: f64,
    h_intervals: &[f64; 2],
) -> AxisJerkGradient { /* compute from the (3b) formulas with b_floor = 0 */ }
```

(Make `topp::solver` and `topp::stencil` reachable from the integration test: `pub mod` in `topp/mod.rs` for `stencil` is already there; for `solver` expose only this one `pub fn` via a `pub use` in `topp/mod.rs` guarded as a documented test support export.)

(3d) Chain-aware SLP entry points, added alongside the legacy ones:

- `find_jerk_violators_chain(b: &[f64], h_intervals: &[f64], j_path: f64)` — ratio per point `|b″_i|·√b_i / (2·j_path)` with `b″` via `stencil_at` + `b_dd_weights`.
- `max_axis_ratio_chain(result, chain: &ChainGrid)` — the legacy loop with `chain.geom[i]` + `chain.limits_at(i)` + `s_dddot_at_weights`, **plus** a second pass over `chain.junctions` evaluating the dual geometry/limits at `j.idx` (the junction point must satisfy both sides' jerk limits).
- `build_axis_jerk_cuts_chain(result, chain, target_ratio)` — same placement rule (`SLP9_CUT_PLACEMENT_FRACTION`), emitting weight-based cuts; junction duals get their own cuts (different `cp/cpp/cppp/j_lim`, same stencil triple).
- `append_path_jerk_cut_weights` — a new weight-form variant **alongside** the legacy `append_path_jerk_cut_to_clarabel` (the legacy fn keeps serving the legacy `slp_solve` untouched until Task 8 deletes both): row `3J/√b̄ − α·b_i − Σ w_k·b_k ≥ 0` with `α = J/b̄^{3/2}` (a positive scaling of the legacy rows — feasible-set identical; the legacy h²-multiplied form is this times h². Do NOT switch the legacy path to the new scaling: identical fixture numerics through Task 7 are part of the regression evidence).
- `solve_with_cuts_and_trust_region` reads `bundle.h_intervals` (passes weights down instead of scalar `h`).
- `slp_solve_chain(bundle, tol, scale)` and `slp_solve_with_axis_jerk_chain(bundle, chain, tol, scale)` — same control flow as the legacy pair (`solver.rs:703-809`, `901-1042`), calling the chain variants.

- [ ] **Step 4: Run to verify pass**

Run: `cargo test -p temporal`
Expected: PASS — identity tests (uniform AND non-uniform), full fixture suite still green through the legacy path.

- [ ] **Step 5: Commit**

```bash
git add rust/temporal/src/topp/solver.rs rust/temporal/src/topp/solver/tests.rs \
        rust/temporal/tests/step9_cut_identity.rs rust/temporal/src/topp/mod.rs
git commit -m "temporal: weights-based SLP cuts — one formula replaces the 3-case stencil match

Chain-aware violator scans and cut builders run per-point limits and
junction dual geometry; row normalization unchanged."
```

---

### Task 7: Chain-aware verify + output

**Files:**
- Modify: `rust/temporal/src/topp/verify.rs`, `rust/temporal/src/topp/output.rs`
- Test: `rust/temporal/src/topp/verify/tests.rs`

- [ ] **Step 1: Failing test** (append to `verify/tests.rs`): construct the two-segment chain from Task 5's helper, hand it a synthetic `SolverResult` where the junction point violates the RIGHT side's velocity limit only (b at junction = 200² — legal for the 300 mm/s left side, illegal for the 150 mm/s right side), and assert `check_chain` reports infeasible at the junction index:

```rust
#[test]
fn junction_dual_limits_are_verified() {
    let chain = two_segment_chain_with_junction(); // share via tests_support
    let n = chain.n_points();
    let mut b = vec![100.0; n];
    b[10] = 200.0_f64.powi(2); // junction: ok for v_max=300, violates 150
    let a = vec![0.0; n];
    let result = SolverResult { b, a, status: SolverStatus::Solved };
    let report = check_chain(&chain, &result);
    assert!(!report.feasible, "right-side junction limits not checked");
    assert_eq!(report.worst_violation_grid, 10);
}
```

- [ ] **Step 2: Run to verify compile failure.** Run: `cargo test -p temporal --lib verify`

- [ ] **Step 3: Implement `check_chain(chain: &ChainGrid, result: &SolverResult) -> VerifyReport`:** the legacy loop (`verify.rs:142-217`) with `chain.geom[i]` / `chain.limits_at(i)` / `s_dddot_at_weights(&result.b, i, &chain.h_intervals)`, plus a junction pass: for each `JunctionDual`, run `ratios_at` again with the dual geometry/limits at `j.idx` and merge (worst ratio wins the binding tag for that point). `output::assemble` needs only its `grid: &ArclengthGrid` parameter loosened: it reads `grid.s` — change the parameter to `s: &[f64]` and **update the legacy caller in `topp/mod.rs` in this same step** (one line: `assemble(&arc_grid.s, …)`) so this commit compiles; the chain caller (Task 8) passes `&chain.s`.

- [ ] **Step 4: Run to verify pass.** Run: `cargo test -p temporal`

- [ ] **Step 5: Commit**

```bash
git add rust/temporal/src/topp/verify.rs rust/temporal/src/topp/verify/tests.rs \
        rust/temporal/src/topp/output.rs rust/temporal/src/topp/mod.rs
git commit -m "temporal: chain-aware verifier — junction points checked against both sides"
```

---

### Task 8: `schedule_chain` entry + flip + delete legacy

**Files:**
- Modify: `rust/temporal/src/topp/mod.rs`, `rust/temporal/src/topp/constraints.rs`, `rust/temporal/src/topp/solver.rs`, `rust/temporal/src/topp/verify.rs`

- [ ] **Step 1: Failing test** (append to `topp/tests.rs`): chain entry solves the Task-5 two-segment chain from rest to rest, status success, junction sample continuous:

```rust
#[test]
fn schedule_chain_two_collinear_segments_solves() {
    let chain = crate::topp::chain::tests_support::two_segment_chain_with_junction();
    let profile = schedule_chain_with_tolerance(
        &chain,
        EndpointConditions { v_start: 0.0, v_end: 0.0, a_start: None },
        ToleranceMode::Auto,
    )
    .expect("setup ok");
    assert!(
        matches!(
            profile.status,
            crate::SolveStatus::Solved
                | crate::SolveStatus::SolvedInexact { .. }
                | crate::SolveStatus::SolvedSlp { .. }
        ),
        "status: {:?}",
        profile.status
    );
    assert_eq!(profile.samples.len(), chain.n_points());
    assert!(profile.samples[0].v < 1e-3);
    assert!(profile.samples.last().unwrap().v < 1e-3);
}
```

- [ ] **Step 2: Run to verify compile failure.** Run: `cargo test -p temporal --lib topp`

- [ ] **Step 3: Implement.** In `topp/mod.rs` — reuse `constraints::EndpointConditions` as the public boundary-condition type (one type, no `ChainEndpoints` duplicate; re-export it from `topp`):

```rust
pub use constraints::EndpointConditions;

pub fn schedule_chain_with_tolerance(
    chain: &chain::ChainGrid,
    endpoints: EndpointConditions,
    tolerance: ToleranceMode,
) -> Result<TopProfile, ScheduleError> { ... }
```

Body mirrors `schedule_segment_with_tolerance` (`topp/mod.rs:51-…`): validate endpoints (finite, ≥ 0; `a_start` finite, and `Some` only with `v_start > 0` — return `ScheduleError::InvalidEndpointVelocity` variants, adding `InvalidEndpointAccel(&'static str)` to the error enum), `SolverScale::for_chain`, `scale_chain_grid`, scaled `EndpointConditions` (accel scales by `scale_velocity`-equivalent: `a / σ` — use `to_scaled_accel`), `build_chain`, boundary-infeasible mapping (chain edges → `BoundarySide::Start/End` with unscaled `mvc_b`), `slp_solve_with_axis_jerk_chain`, `unscale_result`, `check_chain` on the **physical** chain (unscaled result), `output::assemble(&chain.s, …)`. The `GridConfig`/`grid_scheme` argument to `assemble`: pass `GridScheme::UniformArclength` (per-segment uniform; the profile is sliced per segment downstream).

Then flip `schedule_segment_with_tolerance` to:

```rust
pub fn schedule_segment_with_tolerance(
    curve: &VectorNurbs<f64, 3>,
    limits: &Limits,
    grid: &GridConfig,
    v_start: f64,
    v_end: f64,
    tolerance: ToleranceMode,
) -> Result<TopProfile, ScheduleError> {
    // (existing validation + grid sampling unchanged)
    let arc_grid = path::sample_arclength_grid(curve, grid.n)
        .map_err(|e| ScheduleError::PathParam(format!("{e}")))?;
    let chain = chain::ChainGrid::from_segment_grids(vec![arc_grid], vec![*limits]);
    schedule_chain_with_tolerance(
        &chain,
        EndpointConditions { v_start, v_end, a_start: None },
        tolerance,
    )
}
```

Then **delete**: legacy `build` + `EndpointVelocities` (constraints.rs), legacy `slp_solve`/`slp_solve_with_axis_jerk`/`max_axis_ratio`/`build_axis_jerk_cuts`/`find_jerk_violators`/`append_path_jerk_cut_to_clarabel` + `AxisJerkStencil` (solver.rs), legacy `check` (verify.rs), `stencil::s_dddot_at` + `stencil_for` (stencil.rs), the Task-4 equivalence test, and Task 2's `s_dddot_weights_matches_legacy_uniform` comparison test (both compared new-vs-legacy; their job ends with the legacy path — keep the pure-weights polynomial-exactness tests). Update `solver/tests.rs`, `constraints/tests.rs`, `verify/tests.rs`, `output/tests.rs` call sites to the chain forms (mechanical: wrap single grids via `from_segment_grids`). **Also migrate `temporal/tests/step9_cut_identity.rs`**: it imports `stencil::s_dddot_at` (line 1) and ground-truths the legacy three-case cut helpers — port its `j_axis_at_iterate` ground truth to the weight-based formula (`s_dddot_at_weights` + uniform `h_intervals`); the uniform-spacing identity assertions must keep passing against the new cut path (they verify the same math through the collapsed formula).

- [ ] **Step 4: Run the FULL workspace suite — this is the regression gate for the whole rewrite.**

Run: `cargo test --workspace`
Expected: PASS everywhere except the 3 known-red sentinels (prototype fixture_4, multi_segment fixture_7, and the two `#[ignore]`d RED tests from Task 1). Any other failure is a porting bug in Tasks 4–8 — fix before committing. Pay attention to the SLP fixtures (1–6): they now run through `build_chain` + chain SLP; identical numerics are expected (bit-equivalence held at the bundle level), but `SolvedSlp` iteration counts may shift by ±1 — amend fixture assertions only if the trajectory time is unchanged within 1e-6.

- [ ] **Step 5: Commit**

```bash
git add -A rust/temporal
git commit -m "temporal: schedule_chain entry; schedule_segment is a chain of 1; legacy path deleted

One code path for 1..K segments. Full suite green through the chain build."
```

---

### Task 9: Junction classification (`JunctionKind`)

**Files:**
- Modify: `rust/temporal/src/multi/junction.rs`
- Test: `rust/temporal/src/multi/junction/tests.rs`

- [ ] **Step 1: Failing tests** (`junction/tests.rs` has `textbook_limits()` but no `line()`; add the two fixtures at the top of the new test block):

```rust
fn line(from: [f64; 3], to: [f64; 3]) -> VectorNurbs<f64, 3> {
    VectorNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![from, to]).unwrap()
}

fn lims() -> Limits {
    textbook_limits()
}

#[test]
fn collinear_junction_is_smooth() {
    let a = line([0.0; 3], [40.0, 0.0, 0.0]);
    let b = line([40.0, 0.0, 0.0], [100.0, 0.0, 0.0]);
    let r = compute_junction_velocity(&a, &b, &lims(), &lims(), 0.05);
    assert!(matches!(r.kind, JunctionKind::Smooth));
}

#[test]
fn right_angle_junction_is_corner() {
    let a = line([0.0; 3], [40.0, 0.0, 0.0]);
    let b = line([40.0, 0.0, 0.0], [40.0, 60.0, 0.0]);
    let r = compute_junction_velocity(&a, &b, &lims(), &lims(), 0.05);
    assert!(matches!(r.kind, JunctionKind::Corner));
}
```

- [ ] **Step 2: Run to verify compile failure.** Run: `cargo test -p temporal --lib junction`

- [ ] **Step 3: Implement.** In `junction.rs`:

```rust
/// Fuse threshold: junctions with tangent disagreement at or below this are
/// chain-fused (treated G1-continuous). At 1000 mm/s a 1e-3 rad kink is a
/// ~1 mm/s lateral step — far inside the scv impulse budget.
const THETA_FUSE_RAD: f64 = 1e-3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum JunctionKind {
    Smooth,
    Corner,
}
```

`JunctionResult` gains `pub kind: JunctionKind`. In `compute_junction_velocity`, after computing `t_left`/`t_right` (junction.rs:26-27), compute the angle via the same half-angle identity used by `sharp_corner_jd_cap` (junction.rs:96-102) and set `kind = if alpha <= THETA_FUSE_RAD { Smooth } else { Corner }`. Degenerate tangents (`normalize_3` returned zeros) → `Corner` (fail-safe: never fuse what we can't prove smooth).

- [ ] **Step 4: Run to verify pass.** Run: `cargo test -p temporal --lib junction`

- [ ] **Step 5: Commit**

```bash
git add rust/temporal/src/multi/junction.rs rust/temporal/src/multi/junction/tests.rs
git commit -m "temporal: classify junctions Smooth/Corner by tangent continuity"
```

---

### Task 10: Chain partitioning + profile slicing (`multi/chain.rs`)

**Files:**
- Create: `rust/temporal/src/multi/chain.rs`
- Create: `rust/temporal/src/multi/chain/tests.rs`
- Modify: `rust/temporal/src/multi/mod.rs` (register `mod chain;`)

- [ ] **Step 1: Failing tests:**

```rust
#[test]
fn partition_splits_only_at_corners() {
    // smooth, corner, smooth → chains [0..=1], [2..=3].
    let kinds = [JunctionKind::Smooth, JunctionKind::Corner, JunctionKind::Smooth];
    let chains = partition_chains(4, &kinds);
    assert_eq!(chains, vec![0..=1, 2..=3]);
}

#[test]
fn slice_duplicates_junction_sample_and_splits_time() {
    // Synthetic 2-segment chain profile: ranges (0,2) and (2,4), 5 samples.
    let ranges = vec![(0usize, 2usize), (2, 4)];
    let samples: Vec<GridSample> = (0..5)
        .map(|i| GridSample {
            s: i as f64,
            v: 10.0,
            a: 0.0,
            b: 100.0,
            binding: BindingConstraint::None,
        })
        .collect();
    let chain_profile = TopProfile {
        samples,
        status: SolveStatus::Solved,
        grid_scheme: GridScheme::UniformArclength,
        total_time: 0.4,
    };
    let per_segment = slice_chain_profile(&chain_profile, &ranges);
    assert_eq!(per_segment.len(), 2);
    assert_eq!(per_segment[0].samples.len(), 3);
    assert_eq!(per_segment[1].samples.len(), 3);
    // Junction sample appears in both, with per-segment s rebased to 0.
    assert_eq!(per_segment[0].samples[2].v, per_segment[1].samples[0].v);
    assert!((per_segment[1].samples[0].s - 0.0).abs() < 1e-12);
    // Trapezoid time over each slice: 2 mm at 10 mm/s each → 0.2 s.
    assert!((per_segment[0].total_time - 0.2).abs() < 1e-9);
    assert!((per_segment[1].total_time - 0.2).abs() < 1e-9);
}
```

- [ ] **Step 2: Run to verify compile failure.** Run: `cargo test -p temporal --lib multi::chain`

- [ ] **Step 3: Implement `multi/chain.rs`:**

```rust
use crate::multi::junction::JunctionKind;
use crate::{GridSample, SolveStatus, TopProfile};
use std::ops::RangeInclusive;

/// Maximal runs of segments joined by Smooth junctions. `kinds[k]` is the
/// junction between segments k and k+1.
pub(crate) fn partition_chains(
    n_segments: usize,
    kinds: &[JunctionKind],
) -> Vec<RangeInclusive<usize>> {
    debug_assert_eq!(kinds.len() + 1, n_segments);
    let mut chains = Vec::new();
    let mut start = 0;
    for (k, kind) in kinds.iter().enumerate() {
        if *kind == JunctionKind::Corner {
            chains.push(start..=k);
            start = k + 1;
        }
    }
    chains.push(start..=n_segments - 1);
    chains
}

/// Slice one chain profile back into per-segment profiles. The junction
/// sample is duplicated into both neighbors; per-segment `s` is rebased to
/// start at 0; per-segment time is the trapezoid over the slice.
pub(crate) fn slice_chain_profile(
    chain: &TopProfile,
    segment_ranges: &[(usize, usize)],
) -> Vec<TopProfile> {
    segment_ranges
        .iter()
        .map(|&(lo, hi)| {
            let s0 = chain.samples[lo].s;
            let samples: Vec<GridSample> = chain.samples[lo..=hi]
                .iter()
                .map(|smp| GridSample { s: smp.s - s0, ..*smp })
                .collect();
            // Same trapezoid + floors as output::assemble (output.rs:29-39)
            // — extract a shared helper there rather than duplicating.
            let mut total_time = 0.0;
            for w in samples.windows(2) {
                let ds = w[1].s - w[0].s;
                let v_sum = w[0].v + w[1].v;
                total_time += if v_sum > 1e-12 {
                    ds * 2.0 / v_sum
                } else {
                    ds / 1e-9_f64.max(w[0].v.max(w[1].v))
                };
            }
            TopProfile {
                samples,
                status: chain.status,
                grid_scheme: chain.grid_scheme,
                total_time,
            }
        })
        .collect()
}

#[cfg(test)]
mod tests;
```

- [ ] **Step 4: Run to verify pass, commit.**

Run: `cargo test -p temporal --lib multi::chain`

```bash
git add rust/temporal/src/multi/chain.rs rust/temporal/src/multi/chain/tests.rs \
        rust/temporal/src/multi/mod.rs
git commit -m "temporal: chain partitioning at corners + chain-profile slicing"
```

---

### Task 11: Rewire `plan_batch` over chains; un-ignore RED test 1

**Files:**
- Modify: `rust/temporal/src/multi/mod.rs`, `rust/temporal/src/multi/joining.rs`, `rust/temporal/src/multi/parallel.rs`
- Test: `rust/temporal/src/multi/tests.rs`, `rust/temporal/src/multi/joining/tests.rs`, `rust/temporal/src/multi/parallel/tests.rs`, `rust/temporal/tests/multi_segment.rs`

This is the joining-layer rewrite. The structure of every piece survives — the unit of work changes from segment to chain.

- [ ] **Step 1: Un-ignore the RED test** (delete the `#[ignore]` line on `smooth_junction_has_no_accel_impulse`) and re-confirm it fails:

Run: `cargo test -p temporal --lib smooth_junction_has_no_accel_impulse`
Expected: FAIL (accel step). **Run this BEFORE step 2** — step 2 adds `initial_accel` to `BatchInput` and breaks this test's struct literal until step 6 adds `initial_accel: 0.0` to every construction site. The test must be green (and un-ignored) by the end of this task.

- [ ] **Step 2: API changes in `multi/mod.rs`:**

```rust
pub struct BatchInput<'a> {
    pub segments: &'a [SegmentInput<'a>],
    pub grid_strategy: GridStrategy,
    pub worker_threads: usize,
    pub initial_velocity: f64,
    /// Path accel at the batch start. Pinned in the SOCP only when
    /// `initial_velocity > 0`; at a rest start it MUST be 0.0 (asserted —
    /// standstill implies zero accel) and the rest envelope governs.
    pub initial_accel: f64,
    pub terminal_velocity: f64,
}
```

`JunctionBindingCap` gains `ChainInterior` (it is `#[non_exhaustive]` — backward compatible).

- [ ] **Step 3: Rewire `plan_batch` (multi/mod.rs:83-174):**

```text
1. validation (+ assert!(initial_velocity > 0.0 || initial_accel == 0.0))
2. junction analysis: compute_junction_velocity for all K−1 junctions (as today)
3. chains = chain::partition_chains(k, &kinds)
4. per chain: sample per-segment ArclengthGrids (grid::compute_n per segment,
   as today at mod.rs:96-103), ChainGrid::from_segment_grids — built ONCE,
   reused across sweeps (geometry never changes; only endpoint velocities do)
5. ChainState per chain: v_start = batch initial_velocity for chain 0 else
   corner cap; v_end = terminal for last else corner cap; a_start =
   Some(initial_accel) for chain 0 iff initial_velocity > 0; dirty = true
6. parallel::fan_out_solves over chains (Step 4 below)
7. joining::join_until_converged over chains + corner junctions only
8. slice every chain profile (chain::slice_chain_profile) → flat
   Vec<TopProfile> per segment (order preserved)
9. JunctionInfo per original junction:
   - Corner: as today — v from the left slice's last sample, binding from
     the upfront cap tag
   - Smooth: v from the shared junction sample; binding = the upfront cap
     tag if v is within 1e-3 of that cap, else ChainInterior
```

- [ ] **Step 4: `parallel.rs` — fan-out over chains.** `SegmentState` → `ChainState { v_start, v_end, a_start: Option<f64>, profile: Option<TopProfile>, dirty }` (joining.rs). `fan_out_solves` takes `chain_grids: &[ChainGrid]` + `states: &mut [ChainState]`; the worker calls `solve_with_boundary_fallback(&chain_grids[idx], v_start, v_end, a_start, pin_start, pin_end)` where `pin_start = idx == 0`, `pin_end = idx + 1 == n_chains`. Inside `solve_with_boundary_fallback`, every `schedule_segment_with_tolerance(curve, limits, grid, vs, ve, mode)` call becomes `schedule_chain_with_tolerance(chain_grid, ChainEndpoints { v_start: vs, v_end: ve, a_start }, mode)` — `a_start` rides along untouched through the bisection — it is only ever `Some` on chain 0, whose `v_start` is pinned and therefore never bisected, and the zero-zero retry / v_max ladder (parallel.rs:147-209) are unreachable for pinned-start chains (the `pin_start || pin_end` early return at parallel.rs:147 guards them). Encode the invariant in code, not prose: `debug_assert!(a_start.is_none() || pin_start, "a_start pin without a pinned v_start — the bisection would silently re-plan a different boundary state");`. The v_max-ladder fallback rebuilds limits — with chains the ladder scales each `ChainGrid.limits` entry; the helper, matching legacy ladder semantics exactly (velocity-only bisection — scaling accel/jerk would change the problem class):

```rust
/// v_max-ladder support: scale ONLY v_max, preserving per-segment feedrate
/// derating ratios; a_max/j_max/a_centripetal_max untouched.
fn scale_chain_v_max(chain: &ChainGrid, factor: f64) -> ChainGrid {
    let mut scaled = chain.clone();
    for l in &mut scaled.limits {
        *l = crate::Limits::new(
            l.v_max.map(|v| v * factor),
            l.a_max,
            l.j_max,
            l.a_centripetal_max,
        );
    }
    scaled
}
``` Endpoint sync after solve (parallel.rs:74-79) reads the chain profile's first/last samples — unchanged logic.

- [ ] **Step 5: `joining.rs` — sweep over corner junctions.** `bidirectional_junction_sweep(states: &mut [ChainState], corner_caps: &[f64])` — identical algorithm (joining.rs:59-85), indices now over chains; `corner_caps[k]` is the `v_junction` of the k-th **corner** junction (the only junctions between chains). `join_until_converged` unchanged in structure. The `#[cfg(test)]` `forward_sweep`/`reverse_sweep` update mechanically.

- [ ] **Step 6: Fix the multi test fallout.** Existing `multi/tests.rs`, `joining/tests.rs`, `parallel/tests.rs` construct `SegmentState`s and `BatchInput`s: add `initial_accel: 0.0` everywhere, rename to `ChainState` where needed. The semantics of every existing assertion are preserved — collinear test fixtures (`straight_50mm` chains) now fuse into ONE chain, so tests that asserted per-segment solve counts or sweep counts need their expectations re-derived: a fused chain converges in 1 sweep with 0 corner junctions. Re-derive, don't delete — each existing test still guards a real behavior (boundary velocity threading, stall reporting, profile endpoint sync). `multi_segment.rs` integration fixtures: collinear multi-segment fixtures fuse (assert times improve or stay equal — never regress by more than 1e-6); fixture_7's κ-spike inter-grid sentinel stays red with its existing message.

- [ ] **Step 7: Run the gates.**

Run: `cargo test -p temporal`
Expected: `smooth_junction_has_no_accel_impulse` GREEN (the chain solver makes the junction sample shared — the accel step is structurally zero, and the spanning jerk row holds). Corner tests green. fixture_7 red (pre-existing sentinel), fixture_4 red (pre-existing sentinel).

Run: `cargo test --workspace`
Expected: trajectory/motion-bridge compile breaks on `BatchInput.initial_accel` — fix the construction sites (the `BatchInput` literal at `trajectory/src/beta.rs:376-382` gets `initial_accel: 0.0` plainly — Task 12 threads the real value; a `TODO(task-12)` marker is not needed, 0.0 is the physical truth for every caller until the carry exists). All green except the two sentinels + the Task-1 streaming RED (still ignored).

- [ ] **Step 8: Commit**

```bash
git add -A rust/temporal rust/trajectory
git commit -m "temporal: joining layer over condensed chains

Smooth junctions dissolve into chain SOCPs (shared point, spanning jerk);
corners keep scv caps with monotone velocity sweep; interior bisection
ladder scoped to corner endpoints. Junction accel impulse test green."
```

---

### Task 12: Trajectory plumb — `initial_a` end to end

**Files:**
- Modify: `rust/trajectory/src/lib.rs`, `rust/trajectory/src/plan_velocity.rs`, `rust/trajectory/src/beta.rs`
- Test: `rust/trajectory/src/tests.rs` (or the existing module test homes for `plan_velocity`/`beta`)

- [ ] **Step 1: Failing test** — validation contract:

```rust
// Use the existing builder `default_input(segments, safety)` at
// plan_velocity/tests.rs:39 (it gains `initial_a: 0.0` in this task). There
// is NO named segment fixture in this file — every existing test constructs
// its PlanSegment inline; copy the segment construction from the simplest
// existing single-segment test in this file verbatim.
#[test]
fn plan_velocity_rejects_accel_at_rest_start() {
    let segs = /* inline PlanSegment construction copied from the simplest
                  existing test in this file */;
    let mut input = default_input(&segs, /* same SafetyMode the copied test uses */);
    input.initial_v = 0.0;
    input.initial_a = 100.0;
    let err = plan_velocity(&input).unwrap_err();
    assert!(matches!(err, ShapeError::UnsupportedBoundaryAccel));
}

#[test]
fn plan_velocity_rejects_nonfinite_accel() {
    let segs = /* same construction */;
    let mut input = default_input(&segs, /* same SafetyMode */);
    input.initial_v = 50.0;
    input.initial_a = f64::NAN;
    let err = plan_velocity(&input).unwrap_err();
    assert!(matches!(err, ShapeError::UnsupportedBoundaryAccel));
}
```

(Both validations run before segments are touched — `plan_velocity.rs:94-99` — so any segment fixture works; correctness does not depend on the geometry.)

- [ ] **Step 2: Run to verify compile failure.** Run: `cargo test -p trajectory --lib plan_velocity`

- [ ] **Step 3: Implement.**
  - `trajectory/src/lib.rs`: `ShapeBatchInput` gains `pub initial_a: f64` (after `initial_v`, lib.rs:34); `ShapeError` gains `#[error("unsupported boundary accel: initial_a must be finite, and 0.0 when initial_v is 0.0")] UnsupportedBoundaryAccel`.
  - `plan_velocity.rs`: `PlanInput` gains `pub initial_a: f64` (line ~76); validation after the existing `initial_v` check (line 94-99): non-finite → error; `initial_v == 0.0 && initial_a != 0.0` → error; thread into the `ShapeBatchInput` construction (line ~123).
  - `beta.rs`: where `run_initial_v` is computed (beta.rs:373), add `let run_initial_a = if is_first_run { input.initial_a } else { 0.0 };` and set `initial_accel: run_initial_a` in the `BatchInput` (replacing Task 11's literal 0.0). Later runs start at rest — 0.0 is the physical truth there, not a placeholder.
  - All other `ShapeBatchInput`/`PlanInput` construction sites (tests, `shape_batch` callers): `initial_a: 0.0` — they are rest starts.

- [ ] **Step 4: Run to verify pass.** Run: `cargo test -p trajectory`

- [ ] **Step 5: Commit**

```bash
git add rust/trajectory
git commit -m "trajectory: thread initial_a from PlanInput to the first run's BatchInput"
```

---

### Task 13: Streaming accel carry; un-ignore RED test 2

**Files:**
- Modify: `rust/trajectory/src/streaming/state.rs`
- Test: `rust/trajectory/src/streaming/tests.rs`, `rust/motion-bridge/src/planner/tests.rs`

**Source decision (review finding, load-bearing):** the pin feeds the **pre-shape** temporal SOCP, so the accel must be sampled from the **pre-shape planned profile** — `self.planned_fitted: Vec<FittedSegment>` (time-domain fitted axes, exactly what the SOCP profile was fitted to) — NOT from the shaped axis pieces. (`read_path_speed_at` samples the shaped pieces — a pre-existing approximation convention for `v` that this task deliberately does not relitigate; the `(v, a)` pair being sourced from two layers is the same order of approximation as today's `v` alone and is documented at the call site.)

- [ ] **Step 1: Failing unit test for the sampler** (append to `streaming/tests.rs`; build a `FittedSegment` whose x-axis is a known quadratic — mirror the `linear_segment()` fixture at tests.rs:24, which already constructs `FittedSegment`s from Bézier pieces):

```rust
#[test]
fn read_path_accel_at_matches_analytic() {
    // x(t) = 5t² on t ∈ [0,1] → vx = 10t, ax = 10; straight line → path
    // accel = 10. Quadratic Bézier coeffs for 5t²: [0.0, 0.0, 5.0].
    let mut state = empty_state(); // existing ShaperState fixture pattern
    state.planned_fitted = vec![quadratic_x_segment()]; // like linear_segment()
    let a = state.read_path_accel_at(0.5, f64::NAN);
    assert!((a - 10.0).abs() < 1e-9, "got {a}");
}

#[test]
fn read_path_accel_at_zero_speed_returns_fallback() {
    let mut state = empty_state();
    state.planned_fitted = vec![quadratic_x_segment()];
    // At t=0 speed is 0 — the tangential direction is undefined; fallback.
    let a = state.read_path_accel_at(0.0, 0.0);
    assert_eq!(a, 0.0);
}
```

- [ ] **Step 2: Run to verify compile failure.** Run: `cargo test -p trajectory --lib streaming`

- [ ] **Step 3: Implement** (in `state.rs`, next to `read_path_speed_at` at line 238):

```rust
/// Tangential (path) acceleration of the PRE-SHAPE planned profile at time
/// `t`: a_path = (v⃗·a⃗)/|v⃗| over the planned_fitted XY axes. This is the
/// quantity the temporal SOCP's a_0 pin governs — the shaped axes include
/// shaper transients and would pin the wrong layer. Below SPEED_FLOOR the
/// tangential direction is undefined; standstill implies a_path = 0, so the
/// fallback is returned.
pub(crate) fn read_path_accel_at(&self, t: f64, fallback: f64) -> f64 {
    const SPEED_FLOOR: f64 = 1e-9;
    let Some(seg) = self
        .planned_fitted
        .iter()
        .find(|s| s.t_start <= t && t < s.t_end)
        .or_else(|| {
            self.planned_fitted
                .last()
                .filter(|s| (t - s.t_end).abs() <= TIME_LOOKUP_TOLERANCE)
        })
    else {
        return fallback;
    };
    let d = |axis: usize| {
        let d1 = nurbs::eval::derivative(&seg.axes[axis]);
        let d2 = nurbs::eval::derivative(&d1);
        (
            nurbs::eval::evaluate(&d1.as_view(), t),
            nurbs::eval::evaluate(&d2.as_view(), t),
        )
    };
    let (vx, ax) = d(0);
    let (vy, ay) = d(1);
    let speed = (vx * vx + vy * vy).sqrt();
    if speed < SPEED_FLOOR {
        fallback
    } else {
        (vx * ax + vy * ay) / speed
    }
}
```

(Match the actual `nurbs` scalar derivative/eval function names used elsewhere in this crate — `refit.rs` and `emit_shaped.rs` differentiate and evaluate `ScalarNurbs`; reuse their exact call pattern. `FittedSegment.axes` is `[ScalarNurbs<f64>; 3]` per `fit.rs`.)

- [ ] **Step 4: Wire the carry in `append_and_replan`** (state.rs:101):

```rust
let initial_v = self.read_path_speed_at(self.t_dispatched, ctx.fallback_initial_v);
let initial_a = if initial_v > 0.0 {
    self.read_path_accel_at(self.t_dispatched, 0.0)
} else {
    0.0
};
```

and `initial_a` into the `PlanInput` construction (state.rs:158-170, next to `initial_v`).

- [ ] **Step 5: Un-ignore `replan_boundary_carries_acceleration`** in `motion-bridge/src/planner/tests.rs` and run:

Run: `cargo test -p trajectory --lib streaming && cargo test -p motion-bridge --lib replan_boundary_carries_acceleration`
Expected: PASS — the new plan's accel at `t_dispatched` is pinned to the previously-planned value.

- [ ] **Step 6: Commit**

```bash
git add rust/trajectory/src/streaming rust/motion-bridge/src/planner/tests.rs
git commit -m "trajectory: carry (v, a) across the replan boundary

read_path_accel_at samples the planned tangential accel at t_dispatched;
appends can no longer re-plan with a free start accel mid-motion."
```

---

### Task 14: Full-suite gate, limit-speed chain test, bench

**Files:**
- Test: `rust/temporal/src/multi/tests.rs`
- Create: `rust/temporal/tests/chain_bench.rs` (an `#[ignore]`d timing harness, not CI-gating)

- [ ] **Step 1: Limit-speed chain test** (this branch's signature scenario, now multi-segment — append to `multi/tests.rs`):

```rust
#[test]
fn chain_at_1000mms_50k_accel_solves() {
    // Three collinear 200 mm segments fused into one chain at v_max=1000,
    // a_max=50k — the conditioning regime that motivated nondimensionalization.
    let segs: Vec<VectorNurbs<f64, 3>> = (0..3)
        .map(|i| {
            line(
                [200.0 * i as f64, 0.0, 0.0],
                [200.0 * (i + 1) as f64, 0.0, 0.0],
            )
        })
        .collect();
    let limits = Limits {
        v_max: [1000.0, 1000.0, 15.0],
        a_max: [50_000.0, 50_000.0, 100.0],
        j_max: [100_000.0; 3],
        a_centripetal_max: 50_000.0,
    };
    let inputs: Vec<SegmentInput<'_>> = segs
        .iter()
        .map(|c| SegmentInput {
            curve: c,
            limits,
            trailing_junction_chord_tolerance_mm: 0.05,
        })
        .collect();
    let out = plan_batch(BatchInput {
        segments: &inputs,
        grid_strategy: GridStrategy::Fixed(24),
        worker_threads: 1,
        initial_velocity: 0.0,
        initial_accel: 0.0,
        terminal_velocity: 0.0,
    })
    .expect("plan_batch");
    let peak = out
        .profiles
        .iter()
        .flat_map(|p| p.samples.iter())
        .map(|s| s.v)
        .fold(0.0_f64, f64::max);
    assert!(
        (peak - 1000.0).abs() < 15.0,
        "expected cruise at 1000 mm/s, peaked at {peak:.1}"
    );
}
```

- [ ] **Step 2: Run it.** Run: `cargo test -p temporal --lib chain_at_1000mms`
Expected: PASS (if InsufficientProgress reappears at chain scale, the σ for the chain is wrong — check `SolverScale::for_chain`).

- [ ] **Step 3: Bench harness** (`rust/temporal/tests/chain_bench.rs`):

```rust
//! Wall-time probe, not a CI gate (machine-dependent). Run explicitly:
//!   cargo test --release -p temporal --test chain_bench -- --ignored --nocapture
use std::time::Instant;
use temporal::multi::{plan_batch, BatchInput, GridStrategy, SegmentInput};
use nurbs::VectorNurbs;

/// Quarter-circle cubic from `start`, entering along `dir_in`, exiting along
/// the +90°-rotated direction; radius r. Chaining 50 of these with the exit
/// tangent feeding the next entry gives a tangent-continuous serpentine
/// (same control-point pattern as multi/tests.rs::smooth_u_turn).
fn quarter(start: [f64; 3], dir_in: [f64; 2], r: f64, flip: f64) -> VectorNurbs<f64, 3> {
    let k = r * 4.0 * (std::f64::consts::SQRT_2 - 1.0) / 3.0;
    let n = [-dir_in[1] * flip, dir_in[0] * flip]; // rotated ±90°
    let p0 = start;
    let p1 = [start[0] + k * dir_in[0], start[1] + k * dir_in[1], 0.0];
    let p3 = [start[0] + r * (dir_in[0] + n[0]), start[1] + r * (dir_in[1] + n[1]), 0.0];
    let p2 = [p3[0] - k * n[0], p3[1] - k * n[1], 0.0];
    VectorNurbs::try_new(3, vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0], vec![p0, p1, p2, p3])
        .unwrap()
}

#[test]
#[ignore = "wall-time probe, run with --ignored --nocapture"]
fn chain_bench_50_segment_serpentine() {
    // Alternate flip = ±1 so curvature alternates (serpentine, not a circle);
    // track position and tangent through the 50 quarters.
    // ... build Vec<VectorNurbs>, then SegmentInputs (textbook limits,
    // chord tol 0.05), plan_batch with GridStrategy::Fixed(16), workers 4,
    // rest-to-rest, initial_accel 0.0; time 5 calls with Instant, print
    // min/median. Numbers go into the commit message — the spec's posture is
    // measure-and-optimize, never silently degrade.
}
```

- [ ] **Step 4: Full workspace gate.**

Run: `cargo test --workspace 2>&1 | grep -E "test result|FAILED"`
Expected: green everywhere except prototype fixture_4 and multi_segment fixture_7 (pre-existing sentinels, unchanged failure messages). Both Task-1 RED tests now green and un-ignored.

- [ ] **Step 5: Run the bench, record numbers, commit.**

```bash
git add rust/temporal/tests/chain_bench.rs rust/temporal/src/multi/tests.rs
git commit -m "temporal: limit-speed chain test + chain-solve bench harness

Bench (50-seg smooth serpentine, N=16, 4 workers, release): <numbers> per
plan_batch on <machine>."
```

---

## Self-review checklist (run after writing, before execution)

- Spec coverage: chain partitioning (T9, T10), condensed SOCP w/ junction duals + interval-local h (T4, T5), spanning jerk + SLP (T6), schedule_chain + one-code-path (T8), joining over chains + ChainInterior + initial_accel API (T11), replan carry (T12, T13), e2 v==0 gate (T4), corner-free-accel preserved (no accel pins anywhere except chain 0 start — T11 step 3), error handling (build_chain assert, plan_velocity validation, fail-loud stall paths unchanged), tests incl. RED-first (T1), chain-of-1 equivalence (T4), corner preservation (existing joining tests, T11), stencil exactness (T2), limit-speed chain (T14), bench (T14).
- Known intentional gaps: fixture_4 / fixture_7 stay red (out of scope per spec); junction `binding_cap` attribution uses cap-proximity, not dual-variable inspection (documented in T11).
