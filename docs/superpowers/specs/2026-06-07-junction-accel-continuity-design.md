# Junction accel continuity — condensed smooth-chain SOCP

## Problem

The planner enforces accel and jerk limits *inside* a segment but not *across*
segment boundaries. Three boundary types, two of them broken:

1. **Intra-batch junctions.** `bidirectional_junction_sweep`
   (`rust/temporal/src/multi/joining.rs`) propagates a single scalar
   `v_j = min(cap, v_left_end, v_right_start)` per junction. Both segments'
   boundary accelerations stay free SOCP variables (block (b) one-sided
   diffs), so a segment can end at `−a_max` while the next begins at `+a_max`:
   a `2·a_max` accel step — an unbounded-jerk impulse — at any junction, at
   every junction density real slicer output produces.
2. **The replan boundary.** `append_and_replan`
   (`rust/trajectory/src/streaming/state.rs`) carries only
   `initial_v = read_path_speed_at(t_dispatched)`. The machine is physically
   mid-profile at some accel `a ≠ 0`, but every fresh plan gets a free `a_0`.
   A replan fires on every appended segment, so this impulse fires
   continuously while printing.
3. **Rest boundaries** (`v = 0` run edges in `rust/trajectory/src/beta.rs`)
   are already governed by the rest-boundary reachable-envelope rows
   (block (e2), commit `e195bf534`). Covered; out of scope here.

## Contract (user-approved)

- **Full `j_max` enforcement across smooth junctions** — not merely accel
  continuity. The junction-spanning jerk stencil and the SLP per-axis jerk
  cuts get the same treatment as interior points. The jerk limit means the
  same thing everywhere on the toolpath; there is no boundary layer where it
  quietly degrades.
- **Continuity is tangential.** Cartesian accel is `c″·b + c′·a`; this work
  makes the path-scalar contribution continuous. Normal-accel steps from
  curvature jumps at G1-but-not-G2 junctions remain, bounded by
  `a_centripetal_max`; fixing them is geometry-layer G2 smoothing, out of
  scope.
- **Corners keep their semantics.** A junction whose forward unit tangents
  disagree by more than `θ_fuse` (1e-3 rad, matching the spirit of
  `ALPHA_COLLINEAR_THRESHOLD`) keeps today's model: sharp-corner/JD velocity
  cap, pinned scalar `v_j`, free accel. The corner model deliberately budgets
  a velocity-direction impulse (scv/junction deviation); accel continuity
  across a kink is not physically meaningful.
- **The replan boundary carries `(v, a)`**, not just `v`.

## Design

### Chain partitioning

`junction.rs` classifies each junction `JunctionKind::Smooth | Corner` by the
tangent test above. A **chain** is a maximal run of segments joined by smooth
junctions. Corners are the only interior chain boundaries — rests never occur
inside a `plan_batch` window (beta's run partition splits there first).

### The condensed chain SOCP

One solve per chain over the concatenated grid. The junction grid point is a
**single shared variable** — it is the same physical point — so
`M = Σ nᵢ − #junctions` points, each segment keeping its own arclength spacing
`hᵢ` (adaptive N untouched). Existing blocks carry over per-point with that
segment's `Limits` (per-segment feedrate derating works unchanged). New
structure at junctions:

- **Accel linkage** at the shared point uses the non-uniform 3-point
  first-derivative stencil over `(h_L, h_R)`; `a_m = b′_m / 2`.
- **Jerk block (f)** and **SLP per-axis cuts** wherever the 3-point stencil
  support crosses the junction (the shared point and its two neighbors) use
  the non-uniform second-difference stencil with RHS scaled so the bound means
  the same `J` on both sides. This is the contract's teeth. Truncation at the
  junction is O(h) when `h_L ≠ h_R` (leading error `(h_R−h_L)/3 · b‴`) — the
  same order as the one-sided endpoint diffs it replaces. Junction rows get
  the same row-∞-norm scaling as axis-jerk cuts, and chain build asserts a
  bounded `h_L/h_R` ratio — an extreme ratio is a grid-construction bug and
  fails loudly (verifier, Claim 2).
- **Both sides' geometry and limits apply at the shared point**: per-axis
  velocity and accel rows are emitted twice (left `c′, c″` with left `Limits`,
  right with right), plus two centripetal rows (`κ_left`, `κ_right`).
  Reproduces today's min-of-both-sides cap semantics as rows on a *variable*
  instead of a precomputed scalar pin. The single spanning `a_m` is *tighter*
  than two free one-sided accels — correctly so: bounded jerk makes `s̈(t)`
  Lipschitz, so tangential accel is continuous through any junction; the old
  decoupled accels were the looser relaxation (verifier, Claim 1).
- **Time chain and objective use interval-local `h_i`**: block (g/h) rows
  become `t_i·b_i ≥ h_i²` and the `Σt` cost is built per-interval. The
  current code hardcodes one scalar `h`; reused verbatim across a spacing
  change it mis-costs time by `(h_local/h)²` and the optimizer slows the
  trajectory "for free" (verifier item 5a — the one correctness hazard
  found).

Junction `v` and `a` are never explicitly represented, chosen, or propagated
at smooth junctions — they are emergent results of the joint optimum. The
jerk-aware junction velocity cap contemplated for fixture_4-class inputs is
subsumed: if jerk cannot support entering a junction at the MVC, the solver
backs `b_j` off automatically.

Chain edges:

- **Batch start**: `b_0 = v₀²` and now `a_0 = a₀` via one Zero-cone row
  `b_1 = b_0 + 2h·a_0` (convexity untouched; verified during the
  rest-boundary investigation).
- **Batch end at rest**: terminal `v = 0` pin plus block (e2) envelope, FD
  accel free — pinning `a = 0` at rest recreates the `b_1 = 0` time-waste
  trap already rejected in the envelope work. A `terminal_velocity > 0`
  (generic `plan_batch` API, never produced by streaming) keeps today's
  velocity-pin-only semantics: accel free, no envelope.
- **Corner edges**: `b` pinned to the swept corner velocity, accel free.
  Free corner accel is **load-bearing**, not just physical taste: chain
  traversal time is monotone in corner velocity *because* the accel there is
  free (brake-down-and-replay argument), and that monotonicity is what makes
  the min-propagation sweep exact. Pinning corner accel would make the
  propagated state two-dimensional and break the sweep (verifier, Claim 3).
- **Any chain edge whose pinned velocity is 0** — batch end at rest *or* a
  corner swept down to zero — gets the (e2) envelope rows. The gate is
  "endpoint v == 0", not "batch edge".

### SLP and scaling

`slp_solve_with_axis_jerk` is structurally unchanged — bigger bundle, same
trust regions, homotopy, cut normalization and placement (commit `21abab957`
is what makes chain-sized cut systems viable). `SolverScale` takes σ from the
max `v_max` over the chain's segments. `constraints::build` generalizes from
`(grid, limits)` to a chain problem: per-interval `h`, per-point limits
reference, junction markers, optional `a_0` pin. **A single segment is a
chain of length 1 — one code path, no special-casing.** `schedule_segment`
keeps its signature as a thin wrapper.

### Joining layer after the rewrite

- `multi/chain.rs` (new): partitions junctions into chains, builds the chain
  problem, slices the solved profile back into per-segment `TopProfile`s
  (junction sample duplicated into both sides, `total_time` split
  per-segment's intervals).
- `joining.rs`: the bidirectional sweep survives but iterates over **chains**,
  propagating only corner velocities — scalar, monotone min-propagation, the
  regime where the sweep is exact. `MAX_SWEEPS` anxiety disappears for smooth
  junctions entirely.
- `parallel.rs`: fan-out over chains. The interior-junction bisection
  fallback ladder dies — interior infeasibility cannot exist when junction
  states are variables. Corner-endpoint bisection stays, now correctly scoped
  to the only place reachability failures can still occur.
- `JunctionInfo` still reported per junction; smooth ones get
  `binding_cap: ChainInterior` (new `#[non_exhaustive]` variant) unless the
  solved `b_j` sits on a cap row.

### Replan-boundary carry

`BatchInput` gains `initial_accel: f64`. Terminal stays velocity-only:
streaming always ends decel-to-zero and rest is governed by the envelope —
YAGNI on `terminal_accel`. Plumbing: `read_path_speed_at` gets an accel twin
sampling the same planned object's derivative at `t_dispatched` →
`PlanInput`/`ShapeBatchInput.initial_a` → beta's first run → first chain's
`a_0` pin row. Later runs start at rest (`initial_a = 0` trivially).

Feasibility: an append never edits prefix geometry and strictly extends the
path, so the carried `(v₀, a₀)` was part of a feasible plan and remains
feasible. An infeasible pin therefore indicates a planner bug and is a
**hard error**, never a fallback case.

## Error handling

- Chain solve non-success → chain stays dirty → `StalledOnInfeasibleSegment`
  → beta errors out, as today. No new silent paths.
- Infeasible replan-boundary pin → hard error.
- Residual infeasibility modes after interior failures disappear (verifier,
  Claim 4): a corner cap above what the adjacent chain can reach — covered by
  the retained corner-endpoint bisection; a short chain infeasible with
  *both* corner pins — fails loudly, never silently relaxed.
- Chain latency is answered by measurement and optimization, never by
  silently degrading the formulation. If a chain ever exceeds the replan
  budget, that surfaces as an explicit follow-up (overlap splitting), not a
  quiet fallback.

## Testing (TDD order)

1. **RED — junction impulse, temporal level**: two tangent-continuous
   segments forming a smooth high-κ junction (decelerate in, accelerate out —
   V-shaped profile). Assert discrete cross-junction jerk ≤ `j_max·(1+tol)`.
   Fails today by construction.
2. **RED — replan boundary, streaming level**: append mid-decel so the fresh
   plan wants `+a` at `t_dispatched`; assert no accel step between the
   committed profile and the new plan.
3. **Chain-of-1 equivalence**: single-segment results unchanged — the entire
   existing topp suite is this guard.
4. **Corner preservation**: right-angle corner still gets the scv-cap `v_j`,
   chains split there, sweep converges — existing joining tests stay green.
5. **Non-uniform stencil exactness** on polynomial `b(s)` across an
   `h_L ≠ h_R` junction.
6. **Limit-speed chain**: this branch's 1000 mm/s / 50k accel scenario
   extended to a multi-segment chain.
7. **Bench, not gate**: representative 50-segment chain wall-time logged
   against the fan-out baseline on the workstation; Pi measurement at next
   flash.

## Out of scope / known limitations

- **fixture_4** (ℓ1-penalty SLP robustness) and **fixture_7** (adaptive-N
  κ-spikes) stay separate work. Chains likely help fixture_4-class inputs,
  but the single-segment pinned-MVC sentinel itself is a solver-robustness
  issue.
- **Normal-accel steps** from κ-jumps at G1-but-not-G2 junctions: bounded by
  `a_centripetal_max`; geometry-layer fix.
- **Slow smooth junctions**: block (f)'s `√b` weight goes soft at low `b_j` —
  the same blind spot as rest, but `b_j` is a free variable here, so the
  exposure is discretization fidelity, not a structural hole. If a probe test
  shows real jerk spikes at deep-slow junctions, the mechanism is
  SLP-iterate-anchored envelope cuts. Documented hook, not built now.

## Verification

Adversarial math review (kalico-verifier, 2026-06-07) of the four load-bearing
claims. Full derivations:
[`docs/research/condensed-smooth-chain-socp-junction.md`](../../research/condensed-smooth-chain-socp-junction.md).

- **Claim 1 — condensation exactness/convexity: VERIFIED.** The attack
  (curvature-sign-flip junction where left/right block-(d) accel intervals
  become disjoint) lands on *tightening*, which is physically correct, not on
  lost optimality. Residual modeling error O(h), same as today.
- **Claim 2 — non-uniform stencils: VERIFIED with caveats**, folded into the
  design above (O(h) truncation at `h_L ≠ h_R`, row normalization, bounded
  spacing ratio).
- **Claim 3 — corner-sweep monotonicity: VERIFIED with caveats.** Holds
  because corner accel stays free (now marked load-bearing in the design).
  Finite-N non-monotonicity (Pham, TOPP-RA) is a pre-existing caveat of
  today's per-segment sweep, not worsened by chains.
- **Claim 4 — interior infeasibility removal: VERIFIED with caveats**, folded
  into Error handling (corner-cap bisection; both-pinned short chain fails
  loudly).
- **Item 5a — correctness hazard**: block (g/h) and the objective must use
  interval-local `h_i`; folded into the design above.
- Watch item: SLP trust-region/homotopy heuristics are tuned on per-segment
  problem sizes; chain-sized behavior is covered by the testing plan's
  fixtures and bench rather than assumed.
