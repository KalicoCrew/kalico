# Layer 2 — Multi-Segment Integration Design

**Date:** 2026-04-27
**Status:** Spec — design under brainstorm review; implementation plan to follow on green-light
**Layer:** 2 (Temporal scheduling)
**Driver:** Build-order Step 4.5 — "Layer 2 multi-segment integration on synthetic input." Closes the remaining three Layer-2 bullets after Step 4's single-segment SOCP kernel: junction velocity from curvature continuity (subsumes Sonny-Jeon JD as the G1↔G1 degenerate case), lookahead-window joining (forward/reverse sweep), and limit-change invalidation. Operates on synthetic multi-segment NURBS buffers; live Layer-1 wiring is implicit in build-order Step 7 (MVP).

---

## 1. Context

Layer 2 sits between Layer 1's NURBS output and Layer 3's trajectory transformations. CLAUDE.md describes Layer 2 in four bullets:

1. **TOPP-RA implementation** — single-segment time-optimal velocity profile. ⇒ **Build-order Step 4 (in flight, other agent).** Implements the Consolini-Locatelli 2024 SOCP via Clarabel + Lee 2024 SLP outer loop, exposing `schedule_segment(curve, limits, grid, v_start, v_end) -> TopProfile`.
2. **Junction velocity from curvature continuity** — v_max at every segment boundary derives from the centripetal-acceleration-against-curvature formulation. JD is the degenerate G1↔G1 case. ⇒ **This spec.**
3. **Lookahead-window joining** — two-pass forward/reverse smoothing across the segment buffer to reconcile end-of-N velocities with start-of-N+1 velocities. ⇒ **This spec.**
4. **Limit-change invalidation logic** — M-code limit changes affect subsequent segments' limits. ⇒ **This spec.**

What's genuinely new in this spec, surfaced during brainstorming this session:

1. **Architecture is offline-batch, not streaming.** CLAUDE.md's "streaming / receive-time interface" phrasing for Step 4.5 was conflated with "must keep up with motion rate." It does not — only the MCU runtime is real-time. The host has unlimited per-file latency budget; planner finishes ahead of motion and feeds the MCU's segment buffer. So the public API is a function, not a stateful streaming object.
2. **Joining algorithm is option (A) — SOCP per joining iteration**, not option (B) — cheap-kinematic-joining + SOCP-at-finalize. The throughput-non-negotiable principle (CLAUDE.md "Non-negotiable constraints" section, added 2026-04-27) disallows (B)'s 3–8% trajectory-time regression on ramp-bound segments. The Pi 5 throughput investigation (`docs/research/pi5-socp-throughput-investigation.md`) established that (A) is hardware-feasible at MVP throughput on the actual target host.
3. **N is adaptive per-segment, not fixed at 200.** Investigation showed fixed N=200 was over-resolved (5 µm grid spacing on 1 mm segments) and computationally pathological at the upper end (cubic@N=200 baseline = 1.6 s pre-tolerance-patch; 142 ms post-patch but still 22× slower than N=20 at 6.5 ms). Adaptive N policy targets ~0.5 mm grid spacing per segment.
4. **3-thread parallel batch executor**, not 4-thread. Investigation showed 4-thread scaling collapses at large N due to BCM2712 memory-bandwidth saturation on shared L3, plus contention with Klipper's cores 0-1 background activity. 3-thread is the empirical sweet spot.
5. **Per-segment limits as input data, not a stateful FSM.** Each segment in the input buffer carries its own `Limits`. M-code limit changes from the slicer are baked into per-segment limits at parse time. No mid-stream `update_limits` API; the "invalidation" framing was a streaming-pattern artifact.
6. **Junction velocity is a unified centripetal-against-curvature formula.** G1↔G1 (Sonny-Jeon JD with chord-error budget) is the degenerate κ=0-with-delta-at-corner case. G1↔G5, G5↔G5, G2↔G3, fitter-output↔anything all flow through the same computation; only the curvature-evaluation source changes.

What this spec does not re-litigate:

- Step 4's single-segment SOCP/SLP (Consolini-Locatelli + Lee 2024 + Clarabel) — we consume `schedule_segment` as a black box.
- Layer 0 NURBS evaluation, derivative computation, arclength reparameterization (we consume Step 4's path-grid sampler).
- Layer 1's geometric reduction or G5 NURBS construction (we consume reduce-pipeline output).
- Shaper-aware acceleration constraint (Step 8, the renumbered "Smooth shapers + shaper-aware TOPP-RA + corner-blend finalization" item).
- Step 7 MVP integration (wiring real Layer-1 segment stream to `plan_batch`; this spec operates on synthetic buffers).
- Cache layer (discussed during brainstorming, not committed; would compose with `plan_batch` as an outer wrapper if/when added).

### 1.1 Non-goals

- **Streaming / receive-time interface.** The public API is a single batch call. Live G-code (interactive console commands during a print) is not in scope; if needed later, can be supported by repeated `plan_batch` calls on small buffer chunks.
- **Cross-MCU trajectory streaming.** That's Layer 5 (comms) territory; `plan_batch` produces per-segment profiles, the comms layer slices for per-MCU consumption.
- **Per-file caching.** Discussed during brainstorming as a UX win for re-prints; explicitly not committed in this spec. If added later, lives as a wrapper around `plan_batch` (`cache_or_plan_batch(file_hash, machine_config_hash, ...)`), not inside it.
- **M220 / M221 runtime overrides.** Speed/flow live overrides are runtime scale factors applied at the MCU layer, not re-planning triggers. Out of scope for Step 4.5.
- **Production performance bar.** Throughput investigation established (A) is hardware-feasible at MVP throughput; precise per-batch wall-clock budget is not part of acceptance, only logged as a sanity check.
- **Bed mesh / thermal compensation / probing offsets.** Per CLAUDE.md these are runtime per-axis offsets applied outside the planner; out of Step 4.5 scope entirely.

### 1.2 Driving constraints (inherited)

- **Rust end-to-end, host-side, f64.** Layer 2 runs on the Pi-5-class host.
- **NURBS-native pipeline.** Input is a sequence of NURBS segments from Layer 1.
- **Third-order motion as primary profile.** Inherited from Step 4's SOCP formulation; no change.
- **Curvature-continuity-based junction handling** (CLAUDE.md 2026-04-27). Layer 2 derives end-tangents and end-curvatures from each segment's NURBS at u=0 / u=1; no virtual G1 directions for smooth-curve endpoints.
- **Throughput is non-negotiable** (CLAUDE.md 2026-04-27 "Non-negotiable constraints"). The planner never produces a slower trajectory than the math-optimal one. Drives the (A) vs (B) decision below.

## 2. Algorithm choice

### 2.1 Joining: option (A) SOCP per joining iteration

[DIRECTION-confirmed, brainstorm round 3 + Pi 5 investigation]: **Option (A)** — re-solve the per-segment SOCP whenever joining changes a segment's `(v_start, v_end)`, until convergence.

The other option considered (and rejected):

- **Option (B)** — cheap-kinematic forward/reverse joining of `(v_start, v_end)` caps using closed-form `v² = v₀² + 2·a·L`-style propagation, with the per-segment SOCP invoked exactly once per segment at finalization time. The kalico-verifier analysis (this session) established that (B) yields a measurable 3–8% trajectory-time regression vs (A) on ramp-bound segments (the dominant case in real slicer output). The throughput-non-negotiable principle disallows that trade.

(B) was the more conservative choice when (A)'s hardware-feasibility was unknown. The Pi 5 investigation closes that question:

- (A) needs ~1.5–3 cores at 100% sustained at MVP-target 1000 push/sec with adaptive N=20 typical and 4-core parallelism. Available: 4 cores. Headroom: 1–2.5 cores spare for Klipper + comms + telemetry.
- See `docs/research/pi5-socp-throughput-investigation.md` "(A) joining-with-SOCP-per-iter feasibility math" section for the full derivation.

### 2.2 Junction velocity: unified centripetal-against-curvature formula

At every segment boundary `(seg_k, seg_{k+1})`, junction velocity `v_junction` is the minimum of two caps, both evaluated against the path geometry on each side of the junction:

**Cap 1 — per-axis maximum velocity.** From the path tangent direction at the junction, evaluate per-axis MVC:

```
v_max,axis_eq(side) = v_max,axis / |T_axis(side)|     for axis ∈ {X, Y, Z}
v_max,perAxis_cap = min over both sides, over axes (v_max,axis_eq)
```

where `T(side)` is the unit tangent of the segment evaluated at u=1 (left side) or u=0 (right side). Per-axis cap collapses to `v_max` at most cardinal-aligned junctions; tighter at diagonal-aligned junctions because both X and Y axes contribute.

**Cap 2 — centripetal cap.** From the curvature on each side at the junction:

```
v_cent_cap(side)  = sqrt(a_centripetal_max / κ(side))     for κ(side) > κ_floor
v_cent_cap(side)  = ∞                                       for κ(side) ≤ κ_floor
v_centripetal_cap = min over both sides (v_cent_cap)
```

with `κ_floor = 1e-12 mm⁻¹` per the toppra-issue-#244 robustness pattern. For a smooth join (κ_left = κ_right = some finite value), this gives the standard centripetal cap. For a sharp G1↔G1 corner (κ = 0 on each side except a delta at the corner), the formula degenerates and Cap 2 alone gives `v = ∞`.

**Sharp-corner sub-case (G1↔G1).** When both sides report κ ≤ κ_floor at the junction (the G1↔G1 degenerate case), kick into chord-error mode: junction velocity is bounded by the Sonny-Jeon JD formula

```
v_jd = sqrt(2 · a_centripetal_max · δ_chord · (1 - cos(θ/2)) / (1 + cos(θ/2)))
                                            -- or equivalently --
v_jd = sqrt(a_centripetal_max · δ_chord · tan²(θ/2))
```

where `θ` is the corner angle (between left tangent and right tangent) and `δ_chord` is the chord-error tolerance budget. **`δ_chord` is a per-junction quantity supplied by the input**, not a kalico-internal constant — the slicer (parallel workstream) is the source of truth for per-feature tolerance hints. Default if unsupplied: a conservative value derived from CLAUDE.md (e.g., 50 µm); finalized at implementation time.

**Implementation note.** The formula is unified in the sense that it reduces to the same call-site per junction; whether the cap comes from "smooth κ" or "sharp-corner JD" is a runtime branch on `max(κ_left, κ_right) > κ_floor`. There is one `compute_junction_velocity` function in the codebase, not three.

**Final junction velocity:**

```
v_junction = min(v_max,perAxis_cap, v_centripetal_cap, v_max,xyz)
```

(The plain `v_max,xyz` cap is included so isolated-axis high-speed segments still respect machine maxima.)

[DIRECTION-clarified, brainstorming this session]: **`δ_chord` is a Layer-2 input from above (Layer 1's slicer-supplied or default-applied per-junction hint), not a Layer-2-internal constant.** This is consistent with the spec §1's "kalico-aware slicer" parallel workstream, where the slicer emits feature-tagged tolerance hints.

### 2.3 Lookahead-window joining: bidirectional sweep

Standard Klipper-style two-pass:

**Forward sweep (left-to-right).** For each segment k from 0 to K-1:
- `v_start_proposed[k] = v_end[k-1]` (junction velocity from previous segment, modulo cap)
- Cap `v_end[k]` so segment k's SOCP is feasible from `v_start_proposed[k]` (enforce accel-feasibility: `v_end ≤ achievable_v_end_from_v_start_under_dynamic_limits`)
- Note: "achievable" here is *not* a cheap closed-form — it's the SOCP's actual feasibility envelope. So if `v_end[k]` was previously revised lower by a downstream sweep, we honor that; if `v_start_proposed[k]` is higher than the SOCP can absorb given the desired `v_end`, mark the segment dirty and recompute via `schedule_segment`.

**Reverse sweep (right-to-left).** For each segment k from K-1 to 0:
- `v_end_proposed[k] = v_start[k+1]` (junction velocity from next segment)
- Cap `v_start[k]` similarly (decel-feasibility)
- Mark dirty segments

**Iterate** until no segment is marked dirty in a full forward+reverse sweep, or until a hard iteration cap (e.g., 10) is reached. In practice, brainstorming surfaced and the verifier confirmed: convergence is typically within 1–3 sweeps.

**Convergence detection.** A segment is "clean" when its `(v_start, v_end)` matches the junction velocities supplied by both neighbors AND the resulting profile passes Step-4's post-solve verification. Otherwise dirty, re-solve via `schedule_segment`.

### 2.4 Per-segment limits

[DIRECTION-confirmed, brainstorm + Pi 5 investigation]: **Each segment in the input buffer carries its own `Limits`.** Slicer-side M-code limit changes (M201 / M203 / M204 / M205 — set per-axis accel / velocity / jerk / centripetal) are baked into the per-segment limits at G-code parse time (Layer 1 responsibility, not Layer 2). The Layer 2 input is `&[(NurbsSegment, Limits)]`; segments with different limits in adjacent positions are completely fine — joining respects each segment's individual limits.

**No FSM.** No `update_limits` mid-stream. No "dirty range" tracking. The "limit-change invalidation" CLAUDE.md bullet is satisfied by the per-segment-limits-in-input model: when limits change at G-code position K, the slicer / Layer 1 emits subsequent segments with the new limits; the Layer 2 batch sees one homogeneous-limits sequence with a discontinuity at K, and joining/lookahead handle the discontinuity exactly the same as any other limit-discontinuity (different Limits at adjacent segments).

### 2.5 Adaptive N per-segment

[DIRECTION-confirmed, Pi 5 investigation]: **N is computed per-segment**, not fixed.

Default policy (v1):

```
N(seg) = clamp(
    MIN_N = 10,
    ceil(arclength(seg) / TARGET_GRID_SPACING_MM = 0.5),
    MAX_N = 200,
)
```

Examples:
- 1 mm G1 segment ⇒ N = 10 (MIN_N floor)
- 5 mm arc ⇒ N = 10
- 50 mm long G5 ⇒ N = 100
- 200 mm long G5 ⇒ N = 200 (MAX_N cap)

`MAX_N = 200` cap protects against the cubic-class catastrophic regime documented in the Pi 5 investigation (cubic@N=200 = 142 ms at tol=1e-5; pre-patch was 1.6 s).

Future-extensions (not v1):
- Curvature-aware densification: if `max(κ) / mean(κ) > THRESHOLD`, bump N proportionally. Defer to Step 4.5 v2 once we measure on real slicer output.
- Knot-aware grids per Beudaert 2012: align grid points to NURBS knot positions for tight-relaxation properties. v3 territory.

The adaptive-N policy is encoded in a `GridStrategy::Adaptive { ... }` variant on the existing `GridScheme` enum in Step 4 (which currently has only `UniformArclength`). The variant is `#[non_exhaustive]` so future strategies fit without breaking the API.

### 2.6 Per-segment parallelism

[DIRECTION-confirmed, Pi 5 investigation]: **3-thread batch executor.**

Pi 5 has 4× Cortex-A76 cores. Empirical measurement (per the throughput investigation) shows:
- 4-thread scaling collapses at large N due to BCM2712 shared-L3 memory-bandwidth saturation
- 4th thread also fights Klipper's background activity on cores 0-1
- 3-thread is the sweet spot: near-linear scaling at small N (the regime adaptive N produces), no Klipper contention

Pattern: dedicate cores 1-3 for kalico planner; leave core 0 for Klipper + OS. Use `taskset` or `pthread_setaffinity_np` to pin threads.

After joining converges, the per-segment SOCPs are embarrassingly parallel — fan out across the 3 worker threads. Use `std::thread` (no rayon dependency for the prototype).

Implementation choice: a simple work-stealing queue over `Vec<&mut Segment>` plus 3 worker threads, joined at the end. Keeps the dependency surface minimal.

## 3. Architecture

### 3.1 Module layout

New module under existing `temporal` crate:

```
rust/temporal/
├── Cargo.toml                                   # unchanged from Step 4
├── src/
│   ├── lib.rs                                   # re-exports `plan_batch`
│   ├── limits.rs                                # unchanged from Step 4
│   ├── topp/                                    # Step 4 (single-segment) — unchanged
│   │   ├── mod.rs                               # `schedule_segment` (Step 4)
│   │   ├── path.rs
│   │   ├── constraints.rs
│   │   ├── solver.rs
│   │   ├── verify.rs
│   │   └── output.rs
│   └── multi/                                   # Step 4.5 (multi-segment) — new
│       ├── mod.rs                               # `plan_batch` entry + pipeline
│       ├── junction.rs                          # `compute_junction_velocity`
│       ├── joining.rs                           # forward/reverse sweep
│       ├── grid.rs                              # adaptive-N policy
│       └── parallel.rs                          # 3-thread work-stealing fan-out
└── tests/
    └── multi_segment.rs                         # synthetic-fixture tests (§5)
```

`lib.rs` re-exports the public API; `multi/` modules are `pub(crate)` by default.

### 3.2 Public API

```rust
// Existing Step 4 re-exports (unchanged):
pub use limits::Limits;
pub use topp::{schedule_segment, ScheduleError, SolveStatus, ...};

// New for Step 4.5:
pub use multi::{plan_batch, BatchInput, BatchOutput, GridStrategy, JunctionInfo};

// In multi/mod.rs:

#[non_exhaustive]
pub enum GridStrategy {
    /// Fixed-N for every segment. Step 4 backward-compatible.
    Fixed(usize),
    /// Adaptive N per segment per §2.5. v1 policy: arclength-only.
    Adaptive {
        min_n: usize,
        max_n: usize,
        target_grid_spacing_mm: f64,
    },
    // Future: AdaptiveCurvature, KnotAware, ...
}

pub struct BatchInput<'a> {
    /// One entry per segment, in path order.
    /// Each segment carries its own `Limits` and per-junction tolerance hint.
    pub segments: &'a [SegmentInput<'a>],
    /// Adaptive-N policy.
    pub grid_strategy: GridStrategy,
    /// Number of worker threads for parallel SOCP fan-out. Default 3 on Pi 5.
    pub worker_threads: usize,
}

pub struct SegmentInput<'a> {
    pub curve: &'a nurbs::VectorNurbs<f64, 3>,
    pub limits: Limits,
    /// Per-junction chord-error tolerance for the *trailing* junction (the
    /// junction between this segment and the next). Slicer-supplied for
    /// sharp G1↔G1 corners; ignored for smooth κ junctions per §2.2.
    /// Default if unsupplied: 50 µm (placeholder; revisit per §11).
    pub trailing_junction_chord_tolerance_mm: f64,
}

pub struct BatchOutput {
    /// One profile per input segment, in path order.
    pub profiles: Vec<TopProfile>,
    /// Junction-velocity diagnostics for telemetry / debugging.
    pub junctions: Vec<JunctionInfo>,
    /// Number of joining sweeps performed before convergence (or cap-hit).
    pub joining_sweeps: u32,
    /// Whether convergence was reached or cap-hit.
    pub joining_status: JoiningStatus,
}

#[non_exhaustive]
pub enum JoiningStatus {
    Converged,
    CappedAtMaxSweeps { last_dirty_count: usize },
}

pub struct JunctionInfo {
    /// The grid index in the path (`0` is start of segment 0, last is end of segment K-1).
    pub between_segments: (usize, usize),
    pub v_junction: f64,
    pub binding_cap: JunctionBindingCap,
    /// Source of κ on each side; useful when debugging G1↔G1 vs smooth-κ paths.
    pub kappa_left: f64,
    pub kappa_right: f64,
}

#[non_exhaustive]
pub enum JunctionBindingCap {
    PerAxisVelocity,
    Centripetal,
    GlobalVMax,
    SharpCornerChord,
}

#[derive(Debug, thiserror::Error)]
pub enum BatchError {
    #[error("empty segment buffer")]
    EmptySegments,
    #[error("worker_threads must be ≥ 1")]
    InvalidThreads,
    #[error("segment {0}: {1}")]
    Segment(usize, ScheduleError),
}

pub fn plan_batch(input: BatchInput<'_>) -> Result<BatchOutput, BatchError>;
```

`#[non_exhaustive]` on every public enum so we can add `JoiningStatus::FailedNumeric`, new `JunctionBindingCap` variants, future `GridStrategy` cases without breaking downstream.

### 3.3 Internal pipeline

```
plan_batch(input):
    1. Validate: input.segments not empty; worker_threads ≥ 1.
    2. For each junction k in 0..K-1:
         compute_junction_velocity(segments[k], segments[k+1])
       → seed v_start[0] = 0, v_end[K-1] = 0 (boundary; revisit for live G-code)
    3. For each segment k:
         N(k) = grid_strategy.compute_n(segments[k])
       Initial per-segment SOCP: solo solves with seed v_start/v_end (parallel
       fan-out across worker_threads).
    4. Joining loop (max 10 sweeps):
         a. forward sweep: enforce accel-feasibility, mark dirty
         b. reverse sweep: enforce decel-feasibility, mark dirty
         c. if no dirty, break
         d. parallel fan-out: re-solve dirty segments
    5. Final post-joining: collect TopProfiles + junction info + joining_status
```

Each stage is unit-testable in isolation. Pipeline stages have no cross-cutting state.

### 3.4 Why no streaming / stateful object

[DIRECTION-confirmed, brainstorm + Pi 5 investigation]: A single batch call is sufficient because:

- File-source G-code (the MVP target) is fully available upfront.
- Live G-code (interactive console commands during a print) is rare and can be supported by repeated `plan_batch` calls on small chunks.
- M-code limit changes are baked into per-segment `Limits` at parse time (Layer 1 responsibility), not handled by a Layer-2 FSM.
- The eventual MCU runtime is what's real-time; the planner just has to feed the MCU's segment buffer ahead of motion.

If interactive G-code ever needs sub-second-latency planning, that's a Step-7+ concern; can be added as a streaming wrapper around `plan_batch` without changing this Step's design.

## 4. Discretization

Per-segment grid is `UniformArclength` (Step 4 default; only supported scheme), with N computed adaptively per §2.5. Cross-segment grids do not align (no global "supergrid"). Each segment's SOCP is independent in its grid; only `(v_start, v_end)` couple across segments.

### 4.1 Why not multi-segment SOCP across the buffer

[DIRECTION-confirmed, Pi 5 investigation, Step-4 spec §11 deferred item closed]:

The deferred "Cross-segment relaxation effects" item asked whether one SOCP across the whole window would amortize Clarabel setup. Investigation showed:

- 1×(N=200) cubic = 142 ms single-thread
- 10×(N=20) cubic = 65 ms single-thread (2.2× faster)
- With 4-core parallelism: 5.7× faster

Per-segment is structurally better than multi-segment because (a) SOCP cost scales superlinearly in problem size, (b) splitting into K small problems gives each a tiny KKT system that fits in cache and converges faster, (c) the per-segment shape lets us trivially fan out across cores.

**Step-4 spec §11 "Cross-segment relaxation effects" item is closed**: per-segment with adaptive N + parallelism wins.

## 5. Synthetic input fixtures

Six fixtures, designed to exercise each new behavior in isolation. Each is a function in `tests/multi_segment.rs` producing a `BatchInput` and asserting on the `BatchOutput`.

### 5.1 Fixture catalog

**Fixture 1 — Two G1 segments, sharp corner.** Two degree-1 NURBS meeting at a 90° corner (e.g., (0,0)→(50,0)→(50,50)). Tests the JD degenerate-case junction-velocity formula.
- Limits: textbook (Step 4 §6.5).
- Junction tolerance: 50 µm (default placeholder).
- Acceptance: `v_junction` matches the JD formula prediction within 1%; profile at end of seg 0 + start of seg 1 both equal `v_junction` within ε; per-segment post-solve feasibility passes.

**Fixture 2 — G1 → G5 smooth junction.** Degree-1 NURBS into a degree-3 NURBS that's tangent at the junction. Tests the "smooth κ" branch of junction velocity.
- Limits: textbook.
- Acceptance: `v_junction` matches the centripetal cap from G5's κ at u=0; per-segment feasibility passes.

**Fixture 3 — Long straight then sharp corner.** A 100 mm straight followed by a 90° G1↔G1 corner. Tests lookahead — the brake from the corner velocity must propagate back through the straight.
- Limits: textbook.
- Acceptance: profile of seg 0 has a clearly-visible decel ramp ending at `v_junction` (not at `v_max`); total time of seg 0 is greater than a free `v_start=0, v_end=v_max` solo solve would predict (i.e., the decel is real).

**Fixture 4 — Per-segment limits change.** Three segments, middle one with sharply-reduced `a_max` (simulating a slicer M204 mid-chain). Tests per-segment limits handling.
- Limits: seg 0 = textbook, seg 1 = textbook with a_max halved, seg 2 = textbook.
- Acceptance: profile of seg 1 respects the reduced a_max; profiles of seg 0 / seg 2 use full textbook limits; junction velocities at both interior boundaries are caps using the more-restrictive side's limits.

**Fixture 5 — Star pattern (alternating sharp corners).** Five short straight segments forming a star pattern with sharp corners between. Stress-tests joining convergence.
- Limits: textbook.
- Acceptance: all profiles pass per-segment feasibility; joining converges in ≤ 5 sweeps; total time matches a hand-derived expected value within 5%.

**Fixture 6 — Long realistic chain (10 segments mixed types).** A representative mix: 6 G1 straights of varying lengths, 2 G5 cubics, 2 G2 arcs. Stress-tests the parallel batch executor + adaptive N.
- Limits: realistic-target-machine (Step 4 §6.5).
- Per-segment N: adaptive default (§2.5).
- Acceptance: all profiles pass feasibility; joining converges in ≤ 3 sweeps; sanity-log total batch wall-clock (no acceptance threshold) — expectation: <100 ms on a Pi 5.

### 5.2 Skipped on purpose

- **Hundreds-of-segments-long buffer.** Would test scale but not new behaviors; defer to Step 7 MVP integration tests.
- **Shaper-aware constraints in the buffer.** Step 8 territory.
- **Live limit changes (mid-batch `update_limits` style).** Per §2.4, this is a non-feature; per-segment limits handle the case as input data.

## 6. Acceptance criteria

[DIRECTION-confirmed, brainstorm round 3 + Pi 5 investigation]: numerical thresholds frozen here as acceptance criteria; performance is a non-goal sanity log.

### 6.1 Per-segment correctness

Every profile in `BatchOutput.profiles` must pass Step 4's existing post-solve feasibility check (ε_feas = 1e-3). This is enforced by `plan_batch` itself — Step 4's `verify::check` is invoked on each segment as part of `schedule_segment`.

### 6.2 Junction velocity correctness

For each junction `(seg_k, seg_{k+1})`:

- Profile of seg_k at u=1 has `v_end` matching `v_junction` within ε_velocity = 1 mm/s.
- Profile of seg_{k+1} at u=0 has `v_start` matching `v_junction` within ε_velocity = 1 mm/s.
- `v_junction` ≤ each cap (per-axis MVC, centripetal, global v_max, sharp-corner JD as applicable) + ε_feas.

ε_velocity = 1 mm/s is generous because Step 4's solver-internal tolerance can produce small drift. Tightening to 0.1 mm/s post-empirical if real fixtures show better.

### 6.3 Lookahead correctness

For Fixture 3 (long straight + corner) specifically:

- Profile of seg 0 (straight) at u=1 has `v < v_max` (i.e., the brake is happening).
- Total time of seg 0 in the joined batch is strictly greater than total time of seg 0 in isolation (with `v_start=0, v_end=v_max`) — confirms lookahead is reducing throughput at this junction.

### 6.4 Per-segment limits correctness

For Fixture 4 (per-segment limits change):

- Profile of seg 1 (reduced a_max) has peak `|s̈|` ≤ seg 1's `a_max` × (1 + ε_feas).
- Profile of seg 0 / seg 2 reach peaks consistent with textbook a_max.

### 6.5 Joining convergence

- Fixtures 1–4: converges in ≤ 3 sweeps.
- Fixture 5 (star, stress test): converges in ≤ 5 sweeps.
- Fixture 6 (long realistic chain): converges in ≤ 3 sweeps.

If any fixture caps at the hard 10-sweep maximum, that's a test failure indicating either the joining algorithm has a bug or the convergence criterion is too tight. Investigate — don't bump the cap silently.

### 6.6 Performance: non-goal sanity log

Wall-clock per batch is logged to test output but is **not** an acceptance criterion. Expectation: Fixture 6 (10 mixed segments at adaptive N) finishes in <100 ms on a Pi 5 with 3 worker threads. Investigate if observed runtime exceeds ~300 ms (3× margin), but no specific bar is set. Production performance budgets are a Step-7 concern.

## 7. Risks and watch-outs

### 7.1 Joining fails to converge in 10 sweeps

Possible on pathological geometry. **Mitigation:** the `JoiningStatus::CappedAtMaxSweeps { last_dirty_count }` variant on `BatchOutput` lets the test fail explicitly (vs hanging). Diagnosis path: dump the dirty-segment trail and the per-iteration `(v_start, v_end)` history. Bug report, not a "bump the cap" fix.

### 7.2 SOCP returns `Infeasible` on a junction-velocity that should be feasible

Edge cases at very-low-curvature junctions where the boundary equality `b_0 = v_start²` runs into Clarabel's tolerance floor. **Mitigation:** Pre-validate junction velocity against the per-segment MVC at u=0 / u=1 before calling `schedule_segment`; if the supplied v exceeds the MVC, clamp downward and emit a warning.

### 7.3 The other agent's Step-9 work might conflict with Step 4.5

Step 9 (smooth shapers + shaper-aware TOPP-RA + corner-blend finalization) extends `Limits` with a shaper-aware acceleration constraint. **Mitigation:** the `Limits` struct in Step 4 is `#[non_exhaustive]`-friendly (struct fields allow additive extension via `..`); Step 9 fields are added without breaking Step 4.5's API. Coordinate via this spec — Step-9 design should explicitly address composition with `plan_batch`.

### 7.4 Memory pressure on 2 GB Pi

**Mitigation:** at adaptive N (typical N=20–50) per segment × ~8 KB per profile × max ~1000 segments per batch buffer = ~8 MB per batch. Plus working-set per SOCP solve = ~few hundred KB per worker thread × 3 threads = ~1 MB. Total ≪ 2 GB. No concern.

### 7.5 Per-segment SOCP cost scaling on cubic-class geometry

Investigation showed cubic@N=200 = 142 ms at tol=1e-5. **Mitigation:** the `MAX_N = 200` cap in the adaptive-N policy bounds worst-case per-segment cost. If a long G5 segment hits the cap, total batch cost is bounded; doesn't affect correctness, just throughput. For very-long G5 segments where N=200 is genuinely under-resolved, splitting into multiple sub-segments at NURBS knot positions is a Step 4.5 v2 refinement.

### 7.6 3-thread parallelism vs Klipper background activity

Investigation showed 4-thread fights Klipper on cores 0-1. **Mitigation:** default `worker_threads = 3` with thread affinity to cores 1-3. Configurable via `BatchInput.worker_threads` for hosts where Klipper isn't running (developer benchmarking, simulator).

### 7.7 Adaptive-N policy may under-resolve some segments

Default `TARGET_GRID_SPACING_MM = 0.5` is conservative for typical print quality but might be too loose for high-curvature short segments where 0.5 mm grid spacing samples the curvature poorly. **Mitigation:** the MAX_N cap (200) ensures we never under-resolve egregiously; v2 policy adds a curvature-aware density factor. v1 acceptable based on per-segment feasibility check (which would catch egregious under-resolution).

## 8. What deferred / future work picks up

**Step 7 (MVP integration):**
- Wire real Layer-1 segment stream to `plan_batch`. Likely a thin wrapper that buffers Layer-1 output in chunks and calls `plan_batch` per chunk. Cache layer (if added) lives between the two.

**Step 8 (smooth shapers + shaper-aware TOPP-RA + corner-blend finalization):**
- Extends `Limits` with shaper-aware acceleration constraint (additive field; existing Step 4.5 API unchanged).
- Wraps `plan_batch` in an outer iteration that adjusts shaper-aware limits based on observed post-shaping peak acceleration.
- Layer 3 corner-blend finalization replaces the synthetic G5 input with shape-selected NURBS based on dynamic limits.

**Step 4.5 v2 (deferred refinements):**
- Curvature-aware adaptive N (§2.5)
- Skip-base-SOCP heuristic for cubic-class geometry (per Pi 5 investigation: ~30% additional savings)
- Sub-segment splitting at NURBS knot positions for very-long G5 segments
- O(nnz) constraint-matrix construction (per Pi 5 investigation: 7-13% additional savings)

**Cache layer (discussed in brainstorm; not committed):**
- File-level: hash(gcode + machine_config) → cached MCU-ready trajectory stream
- Lives outside `plan_batch` as a wrapper

## 9. Implementation plan envelope

The plan (forthcoming, separate document) will decompose into SDD-worker-sized items roughly:

1. New `multi/` module scaffolding + public API types (`BatchInput`, `BatchOutput`, `GridStrategy`, `JunctionInfo`, etc.).
2. `multi::grid` adaptive-N policy implementation + unit tests (clamp + pure-arclength formula).
3. `multi::junction` `compute_junction_velocity` — per-axis MVC, centripetal cap, sharp-corner JD branch + unit tests on synthetic 2-segment cases.
4. `multi::joining` forward sweep + unit test.
5. `multi::joining` reverse sweep + unit test.
6. `multi::joining` convergence loop (sweeps, dirty-tracking, cap-detection) + unit test.
7. `multi::parallel` 3-thread work-stealing fan-out + unit test on a small batch.
8. `multi::mod::plan_batch` end-to-end pipeline orchestration + integration test on Fixture 1.
9. Fixture 2 (G1+G5 smooth junction).
10. Fixture 3 (long straight + corner — lookahead test).
11. Fixture 4 (per-segment limits change).
12. Fixture 5 (star pattern — convergence stress).
13. Fixture 6 (long realistic chain — performance sanity log).
14. Update CLAUDE.md plan-changes-log on completion.

Items 1–8 are roughly sequential (each pipeline layer); 9–13 are parallel-friendly once 1–8 are in place.

## 10. Open questions / future work

- **`δ_chord` default value** — placeholder is 50 µm; should be revisited once kalico-aware slicer is emitting real per-feature tolerance hints. Fixture comments flag this.
- **Joining cap (10 sweeps)** — empirical; might tighten to 5 once we have real-fixture data showing 1–3 is the typical case.
- **`MAX_N = 200` per-segment cap** — empirical; could be raised if a future tolerance-tuning pass makes large-N solves cheaper.
- **Adaptive N curvature-density factor** — v2 (§2.5).
- **Sub-segment splitting at NURBS knots** — for very-long G5 segments where N=200 is genuinely under-resolved (§7.5).
- **Skip-base-SOCP heuristic** — per Pi 5 investigation, ~30% savings on cubic-class geometry by detecting "this geometry needs SLP cuts" up-front and starting with cuts. Algorithm work; deferred.
- **`O(nnz)` constraint-matrix construction** — per Pi 5 investigation, 7-13% savings at N=200; defer until Step 4.5 lands and we have profiling data on real fixtures.
- **Worker-thread count auto-detect** — currently default 3; should auto-detect Klipper presence + adjust (3 if Klipper is on cores 0-1, else `num_cpus`).
- **Composition with cache layer** — when (if) cache layer lands, define the contract between cache lookups and `plan_batch` invocation.
- **Composition with Step 8 shaper-aware iteration** — Step 8 design should explicitly address how the outer iteration loop interacts with `plan_batch`.

## 11. References

- Consolini & Locatelli, "Is time-optimal speed planning under jerk constraints a convex problem?" *Automatica* 2024, arXiv:2310.07583. (Per-segment SOCP formulation.)
- Lee, Bylard, Sun, Sentis 2024, arXiv:2404.07889. (SLP outer iteration.)
- Sonny Jeon's junction-deviation algorithm (grbl, smoothieware lineage). Sharp-corner G1↔G1 chord-error formula reference.
- Klipper's lookahead implementation (https://www.klipper3d.org/Kinematics.html). Reference for two-pass forward/reverse sweep architecture.
- Beudaert, Lavernhe, Tournier, "Feedrate interpolation with axis jerk constraints on 5-axis NURBS and G1 tool path," *IJMTM* 2012, 57:73–82. (Curve-aware joining pattern; not adopted because we picked option (A) "SOCP per joining iteration" instead of cheap-kinematic joining.)
- Step 4 spec: `docs/superpowers/specs/2026-04-27-layer-2-topp-prototype-design.md`. (Per-segment SOCP API + algorithm.)
- Pi 5 throughput investigation: `docs/research/pi5-socp-throughput-investigation.md`. (Hardware-feasibility of (A); adaptive-N policy; 3-thread parallelism; tolerance patch; multi-seg SOCP analysis.)
- CLAUDE.md (this repo), 2026-04-27 updates: throughput non-negotiable principle; Layer 2 curvature-continuity framing; Step 4 / Step 4.5 split; spline-fitter demotion; build-order renumbering.

---

## Self-review

**Placeholder scan.** Three acknowledged placeholders, each explicitly flagged:
- `δ_chord` default = 50 µm (§2.2, §10) — placeholder until slicer parallel workstream produces real per-feature tolerance hints.
- `TARGET_GRID_SPACING_MM` = 0.5 (§2.5) — empirical-derived; v1 only.
- `MAX_N = 200` per-segment cap (§2.5, §7.5) — empirical-derived from Pi 5 investigation; revisit if future tolerance-tuning makes large-N solves cheaper.

No other "TBD" / "TODO" / vague requirements.

**Internal consistency.** Cross-checked:
- §2.1 (option A) ↔ §2.3 (joining algorithm) ↔ §3.3 (pipeline): consistent on "re-solve dirty segments via `schedule_segment`."
- §2.2 (junction velocity) ↔ §6.2 (junction acceptance) ↔ §5.1 fixtures 1, 2: consistent on the unified centripetal-vs-sharp-corner branching.
- §2.5 (adaptive N) ↔ §3.2 (`GridStrategy` API) ↔ §5.1 fixtures: consistent on `GridStrategy::Adaptive` being the default.
- §2.6 (3-thread) ↔ §3.2 (`worker_threads` API) ↔ §7.6 (Klipper coexistence): consistent.

**Scope check.** Multi-segment batch executor with one new module under `temporal`, six fixtures, frozen acceptance. Sized for one implementation plan with ~14 SDD-worker items. Not too large.

**Ambiguity check.**
- "Achievable v_end from v_start under dynamic limits" in §2.3 forward sweep — ambiguous whether this is a closed-form pre-check or a SOCP solve. Resolution: SOCP solve. The cheap-form check would re-introduce option (B)'s anti-conservative regime per the verifier's analysis. §2.3 wording could be tightened.
- "Per-segment limits at parse time" (§2.4) — ambiguous about which layer parses M-codes. Resolution: Layer 1 (G-code processor); Layer 2 just consumes. Out of Step 4.5 scope; flagged for Layer 1 / Step 7 spec.
- "δ_chord supplied by the input" (§2.2) — ambiguous about default-application path. Resolution: Layer 1 supplies a default if slicer doesn't (parser fills in `trailing_junction_chord_tolerance_mm` per the API in §3.2); Layer 2 just consumes. Default is the placeholder 50 µm.
- Worker-thread CPU pinning: mentioned in §2.6 as `taskset` / `pthread_setaffinity_np`. Implementation will pick one; current preference is `pthread_setaffinity_np` for portability (works regardless of how the binary is invoked). Plan-level detail.

No remaining ambiguities flagged.
