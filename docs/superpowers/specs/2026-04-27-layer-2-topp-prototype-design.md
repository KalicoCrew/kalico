# Layer 2 — TOPP Prototype Design

**Date:** 2026-04-27
**Status:** Spec — design under brainstorm review; implementation plan to follow on green-light
**Layer:** 2 (Temporal scheduling)
**Driver:** Build-order Step 4 — "TOPP-RA prototype on synthetic input." De-risk the algorithmic core of Layer 2: time-optimal `v(s)` on a single synthetic NURBS at a time, against per-axis acceleration, per-axis jerk, and centripetal-acceleration constraints, with externally-supplied (or zero) endpoint velocities. Multi-segment glue, streaming, and limit-change invalidation are deferred to Step 4.5; shaper-aware constraints are deferred to Step 9.

## 1. Context

Layer 2 is the temporal-scheduling layer between Layer 1's geometric NURBS output and Layer 3's trajectory transformations. It produces a velocity profile `v(s)` per segment subject to kinematic limits. CLAUDE.md describes Layer 2 in four bullets: TOPP-RA, junction velocity from curvature continuity, lookahead-window joining, and limit-change invalidation. The "prototype on synthetic input" phrasing in the build order plus orchestrator scoping concentrates Step 4 on **only the first bullet** — the algorithmic core that produces a per-segment `v(s)` — leaving the multi-segment / streaming / invalidation work to a follow-up Step 4.5 (added to the build order during this brainstorm) and shaper-aware feedback to Step 9.

What's genuinely new in this spec:

1. **Algorithm choice that departs from the literal CLAUDE.md wording.** Pham 2018 TOPP-RA is a double-integrator algorithm and does not natively handle jerk. Pham did not publish a jerk-extended TOPP-RA. The post-2018 SOTA for jerk-bounded path-constrained TOPP is the **Consolini & Locatelli 2024 SOCP relaxation** (*Automatica* 2024, arXiv:2310.07583), and that is what this prototype implements. CLAUDE.md wording is up to the user to revise after spec acceptance.
2. **The path of full convexity:** arclength parameterization (Layer 0 already provides it) plus the Verscheure 2009 substitution `b(s) = ṡ²` plus the Consolini-Locatelli third-order convex relaxation produces a Second-Order Cone Program solvable globally by an off-the-shelf solver (**Clarabel**, Rust-native). One LP/SOCP solve per fixture; sub-second on the literature's much-larger problem sizes.
3. **A new `temporal/` workspace crate** — peer to `geometry/` and `nurbs/`. First kalico crate to depend on a heavyweight numerics solver. Solver invocation localized to one internal module so a future swap stays cheap.
4. **Output representation:** sampled `Vec<GridSample>` plus a small metadata struct, with per-grid-point binding-constraint tags. Not a NURBS-in-`s`, not a fitted spline. Layer 3 reparameterization is a Step-9 problem; we don't speculate.
5. **Synthetic-input regime test set** — seven fixtures chosen to exercise straight-line / constant-curvature / curvature-spike / mixed-feature / non-zero-endpoint-curvature / target-machine-realistic regimes against frozen acceptance-criterion thresholds.

What this spec does not re-litigate:

- Layer 0 NURBS evaluation, derivative computation, or arclength reparameterization (all complete; we consume).
- Layer 1's geometric reduction or G5 NURBS construction (we *reuse* existing geometry-crate NURBS-construction paths for fixtures rather than hand-rolling control points).
- Multi-segment / cross-segment junction-velocity-from-curvature, lookahead-window joining, or limit-change invalidation (Step 4.5).
- Shaper-aware acceleration constraints (Step 9).
- Per-axis centripetal-acceleration limits, helical / 3D path optimizations, or non-uniform / curvature-adaptive grids (deferred — see §11).
- Production machine-config representation. The fixture limits in §6.5 are scoped to this prototype's tests; the cross-cutting Configuration system is a separate concern.

### 1.1 Non-goals

- **Multi-segment scheduling.** Endpoint velocities at the path parameter `s = 0` and `s = L` (path arclength) are *inputs* to TOPP, supplied externally by the test fixture or set to zero. The "where do these come from at junctions" question is Step 4.5.
- **Streaming / receive-time interface.** The public API is shaped so Step 4.5 can wrap it without redesign (one segment in, profile out, no hidden state) but Step 4 itself is not a streaming demonstration.
- **Extruder axis (E) in the constraint set.** Step 4 is X / Y / Z only. CLAUDE.md says "Extruder is synchronized to the motion after IS is applied" — E is post-shaping and post-time-reparameterization, derived from the base XYZ NURBS in Layer 3. Including E here would conflate Layer 2 (XYZ velocity scheduling on the geometric path) with Layer 3 (extruder synthesis from time-reparameterized base).
- **Shaper-aware acceleration constraint.** CLAUDE.md is explicit: "Build Layer 2 with the unshaped dynamics constraint first … shaper-awareness is a Layer 3 add-on that feeds back into Layer 2's constraint set." Step 9 territory.
- **Production performance bar.** Sub-second per fixture is more than enough for the prototype; the literature reports SOCP solves at sub-second on much larger problems. We surface "wall-clock per fixture" as a sanity log, not as an acceptance criterion. Receive-time performance budgets are a Step 4.5 / Step 9 concern, set against the actual multi-segment pipeline.
- **Solver-implementation work.** We use Clarabel as a black-box SOCP solver. We don't implement an SOCP / LP solver, and we don't fork or modify Clarabel. If a fixture trips a solver-internal numerical pathology that Clarabel can't be coaxed through with standard tolerance handling, that's a research result worth surfacing (and likely a sign the relaxation is not tight on that fixture), not a fix-the-solver task.
- **Trait-abstracted solver.** YAGNI. Solver invocation is localized to one internal module so a future swap (to e.g. ECOS, OSQP, or a kalico-internal SOCP) stays a one-module rewrite. No `SocpSolver` trait at this stage.
- **Public-API exposure of Clarabel types.** Solver matrices and Clarabel result types stay internal to `temporal::topp::solver`. The public `temporal` API surface uses plain `f64` / kalico-defined types (`GridSample`, `BindingConstraint`, `SolveStatus`, `TopProfile`). Prevents `nalgebra` / `faer` from leaking into Layer 3 callers via type signatures.

### 1.2 Driving constraints (inherited)

- **Rust end-to-end, host-side, f64.** Layer 2 runs on the Pi-5-class host; no MCU concerns at this layer.
- **NURBS-native pipeline.** Input is a single NURBS curve from Layer 1 (synthetic in this step; Layer 1's actual output in Step 4.5). All constraint expressions evaluate against NURBS derivatives.
- **Third-order motion as primary profile.** Per CLAUDE.md "high-level feature scope": jerk is a first-class constraint, not a soft post-hoc smoother.
- **Algebraic-closure principle, applied here only to constraint expression.** Cartesian per-axis kinematic limits express as inequalities on `b = ṡ²`, `a = s̈`, and the path-jerk variable in the Consolini-Locatelli formulation; the path's geometric NURBS provides `C(s), C'(s), C''(s), C'''(s)` directly.
- **Curvature-continuity-based junction handling** (CLAUDE.md 2026-04-27). Layer 2 derives end-tangents and end-curvatures from each segment's NURBS at `u = 0` / `u = 1`. For Step 4 this surfaces only as: (a) the centripetal constraint uses the NURBS's own κ(s); (b) endpoint velocities are inputs (caller's responsibility, deferred to Step 4.5).

## 2. Algorithm choice

### 2.1 The named anchor (CLAUDE.md) vs. the implemented algorithm

CLAUDE.md names this Step "TOPP-RA implementation." The build-order item entered the document with that name because TOPP-RA is the most-cited time-optimal path-tracking algorithm in the post-2018 robotics literature and the open-source `toppra` library is its canonical reference implementation. **The prototype does not implement TOPP-RA.** The reasons, captured during research in this brainstorm and worth preserving in the design record:

1. **TOPP-RA is double-integrator.** Pham & Pham 2018 (IEEE T-RO, arXiv:1707.07239) formulates the problem in `(s, ṡ²)` phase space with control `s̈`. Acceleration and curvature constraints are first-class; **jerk is not.** Pham did not publish a jerk-extended TOPP-RA in the years since. The closest piece is Pham & Pham 2017 "On the structure of the TOPP problem with third-order constraints" (arXiv:1609.05307, ICRA 2017) — TOPP3, a phase-plane numerical-integration approach that predates TOPP-RA's convex-DP framing and does *not* have the clean LP-per-step structure that makes TOPP-RA attractive.
2. **The `toppra` library has no jerk support.** Issue #133 (opened 2020) is unanswered. Bending the library into jerk handling inherits its known numerical issues (issues #112, #244, drake#20619) without architectural benefit.
3. **CLAUDE.md commits to "third-order motion as primary profile."** That is incompatible with a double-integrator algorithm at the Layer 2 level, regardless of what the ecosystem nicknames the canonical reference.
4. **The post-2018 SOTA for jerk-bounded path-constrained TOPP is convex-relaxation work**, dominated by two lines: Consolini & Locatelli 2024 (SOCP relaxation, *Automatica*) and the Lee 2024 / Sun 2019 SLP family. The Consolini-Locatelli SOCP is preferable for the prototype because it produces a single global-optimum solve when the relaxation is tight, without warm-start machinery; SLP is friendlier for shaper-aware feedback at Step 9 (warm-startable across constraint changes) and is held in reserve as a fallback.

We document this departure explicitly here so a future reader doesn't have to re-derive the reasoning. CLAUDE.md's wording can be revised by the user after spec acceptance.

### 2.2 Consolini & Locatelli 2024 SOCP relaxation

Source: Consolini & Locatelli, "Is time-optimal speed planning under jerk constraints a convex problem?" *Automatica* 2024, preprint arXiv:2310.07583.

The relaxation:

- **Primal variable.** `b(s) = ṡ²`, the squared path speed at arclength `s`. Sampled on a finite grid `0 = s_0 < s_1 < … < s_N = L` (uniform-in-`s` for v1; see §3).
- **Auxiliary variables.** Per the SOCP formulation, slack variables encoding the third-order (jerk) inequalities in conic form. Concretely, the convex relaxation introduces nonnegative auxiliaries and rotated SOC constraints that bound `|s⃛|` against the path's third derivative and the first/second derivatives of `b`. (Exact matrix construction is implementation detail in §4.2; we cite the paper's equations rather than re-derive here.)
- **Constraints, per grid point and per axis (X, Y, Z), in conic form:**
  - **Velocity (per-axis):** `|C'_axis(s_i)| · sqrt(b_i) ≤ v_max,axis`, expressed as `b_i ≤ (v_max,axis / |C'_axis(s_i)|)²` — an upper bound on `b_i`. (Linear in `b_i`.)
  - **Acceleration (per-axis):** `|C''_axis(s_i) · b_i + C'_axis(s_i) · a_i| ≤ a_max,axis`, where `a_i` is an auxiliary representing `s̈_i = ½ · b'(s_i)`. Linear in `b_i`, `a_i`.
  - **Jerk (per-axis):** the Consolini-Locatelli third-order relaxation expresses `|s⃛|` and `s̈ · s⃛` couplings via SOC cones; rendered into Clarabel's cone vocabulary as nonneg + SOC. (See §4.2.)
  - **Centripetal:** `b_i · κ(s_i) ≤ a_centripetal_max`. Linear in `b_i` (κ is a known scalar at each grid point). The Maximum Velocity Curve `b_max,cent(s_i) = a_centripetal_max / κ(s_i)` is precomputed and applied as an extra upper bound on `b_i` with a sentinel cap (`κ ≈ 0` ⇒ no centripetal limit; finite numerical floor `1e-12` to avoid division-by-zero, with `b_max,cent` capped at `1e8` per the toppra-issue-#244 robustness pattern).
  - **Boundary:** `b_0 = v_start²`, `b_N = v_end²` (inputs).
- **Objective.** Minimize total time `T = ∫ dt = ∫ ds / sqrt(b(s)) ≈ Σ_i Δs_i · 2 / (sqrt(b_i) + sqrt(b_{i+1}))` (trapezoidal-in-time approximation, the standard TOPP discretization). The Consolini-Locatelli paper uses an equivalent linear functional in their primal variable; we follow the paper's form.

**Tightness.** The relaxation is provably tight under a sufficient condition (paper Corollary 5.1) and conjectured tight in general (Conjecture 4.1). The paper reports empirical tightness on every test case. **We add a post-solve verification step** (§6.4) that recomputes per-axis `a, j` and centripetal on the resulting `(s, v)` profile and asserts feasibility within `ε_feas = 1e-3` (0.1%). If a fixture trips a tightness gap, the verification step catches it and that fixture's failure is a research result worth surfacing.

**Why not Lee 2024 SLP for the prototype.** SLP linearizes the third-order constraint conservatively at each iteration via first-order Taylor of a denominator function. It produces an LP-per-iteration sequence (Lee reports 5–30 iterations, ~7.5ms each in their robotics setting) that converges to a one-sidedly-conservative feasible solution. Optimality is not guaranteed; the gap to the true optimum is implementation-dependent. SOCP gives global optimum (modulo relaxation-tightness) in one solve; for a prototype where we want to *validate* against closed-form straight-line ground truth (§6.3), one-shot global optimum is what we need. SLP's warm-start property pays off in iterative Layer-3 feedback (Step 9), which is not Step 4 territory. Held in reserve as fallback if SOCP fails on representative input.

**Source [RESEARCH — researcher-confirmed, brainstorm round 2]:** Consolini & Locatelli 2024 *Automatica*, arXiv:2310.07583; Pham & Pham 2018 *IEEE T-RO*, arXiv:1707.07239 (TOPP-RA reference); Pham & Pham 2017 (TOPP3, arXiv:1609.05307); Lee, Bylard, Sun, Sentis 2024 (arXiv:2404.07889); Sun et al. 2019 (10.1007/s11431-018-9404-9); toppra issues #112, #133, #244; drake #20619.

### 2.3 Solver: Clarabel

[DIRECTION-confirmed, brainstorm round 2 Q6]: **Clarabel** as the concrete SOCP solver dependency, no abstraction trait. Reasoning:

- Rust-native, MIT-licensed, written by Stanford's Boyd group (the SOCP authority).
- Solves the conic vocabulary the Consolini-Locatelli relaxation needs (zero, nonneg, SOC, rotated SOC).
- Active maintenance, recent benchmark results favor it over ECOS for new projects.
- No Python bridge; this is host-side Rust per CLAUDE.md.

**Workspace impact.** This is the first kalico crate to pull a heavyweight numerics dep. Clarabel transitively depends on `nalgebra`-or-equivalent linalg. The spec's architectural guardrails (§4):

- Solver invocation lives in **one internal module**, `temporal::topp::solver`, with all Clarabel-typed code (matrix construction, cone declarations, result extraction) inside.
- Clarabel types **do not appear in `temporal`'s public API.** Public types are `f64`-and-kalico-defined.
- If `nalgebra` (or whatever Clarabel pulls) ends up overlapping with what `nurbs` does internally, we reuse rather than vendor — but no public-API leakage either way.

Future swap surface: rewriting `temporal::topp::solver` against a different SOCP backend stays local. The `SocpSolver` trait abstraction is **not** introduced; YAGNI until we have a concrete reason to need both backends simultaneously.

## 3. Discretization

### 3.1 Path parameter

Discretize on **arclength `s` ∈ [0, L]**, not the NURBS native parameter `u`. Reasoning:

- Per-axis kinematic limits (`v_max`, `a_max`, `j_max`) are workspace-rate quantities — they are inequalities on `dx/dt`, `d²x/dt²`, `d³x/dt³`. The cleanest mapping from these into the path-parameterized constraint forms uses arclength, because at every point `|C'(s)| = 1` by construction, so `dx/dt = C'(s) · ṡ` reduces to `|dx/dt| = |ṡ|` for the speed magnitude.
- The Consolini-Locatelli SOCP relaxation requires `‖γ'(λ)‖ = 1` to obtain the tight relaxation structure. (Paper §3 derivation depends on this.)
- The CNC-feedrate-scheduling community (Erkorkmaz/Altintas/Beudaert/Sencer) parameterizes the path in `u` but *tracks arclength derivatives* (`f_u`, `f_uu`) explicitly because the feedrate is fundamentally an arclength rate. Our shortcut is just to do the reparameterization once up front.
- Layer 0 already provides arclength reparameterization tooling. We pay the construction cost once per segment.

The robotics-community alternative — abstract `s ∈ [0, 1]` as in toppra's default — does not obtain the tight Consolini-Locatelli relaxation and produces uglier per-axis constraint expressions.

### 3.2 Grid: uniform-in-arclength, fixed N

For Step 4: **uniform-in-`s` grid with N points**, parameter exposed as a `GridConfig`. Default `N = 200` for the canonical fixtures; convergence tests sweep `N ∈ {50, 100, 200, 400}`.

Why not adaptive (knot-aware densification at high curvature):

- Step 4 is a prototype; uniform is simpler to validate and debug.
- The CNC literature's knot-aware grids are a v2 optimization, not a correctness piece.
- Adaptive grids interact with the convergence-test acceptance criterion (a fixed-N convergence sweep is meaningless against a moving grid). v2 territory.

Knot-aware / curvature-adaptive grids are explicitly deferred to §11 ("Open questions / future work"). The `GridConfig` type is shaped so a future `Adaptive { … }` variant fits without breaking the public API.

### 3.3 Watch-out: grid-resolution feedrate ripple

The CNC literature explicitly notes that the `u → s` mapping (or equivalently the arclength evaluation grid for derivative computations) must be at TOPP-grid resolution, not just NURBS-knot resolution. Coarser arclength evaluation manifests as *feedrate ripple* at high speed. The prototype sidesteps by computing `s_i → u_i → C(u_i), C'(u_i), C''(u_i), C'''(u_i)` at every grid point via Layer 0's arclength tooling at the full grid resolution. No subsampling of the geometric quantities relative to the constraint grid.

**Source [RESEARCH — researcher-confirmed]:** Erkorkmaz & Altintas 2001; Beudaert et al. 2012; Sencer 2007.

## 4. Architecture

### 4.1 Crate layout

New workspace member: `rust/temporal/`. Peer to `geometry/` and `nurbs/`. `Cargo.toml` adds it to `workspace.members`.

```
rust/temporal/
├── Cargo.toml
├── src/
│   ├── lib.rs             # Public API: TopProfile, GridSample, BindingConstraint,
│   │                      #             SolveStatus, Limits, GridConfig,
│   │                      #             schedule_segment(...)
│   ├── topp/
│   │   ├── mod.rs         # The TOPP entry point; orchestrates §4.3 pipeline
│   │   ├── path.rs        # Arclength evaluation: per-grid-point C, C', C'', C''', κ
│   │   ├── constraints.rs # Per-grid-point, per-axis constraint expressions
│   │   ├── solver.rs      # Clarabel SOCP construction + solve, internal-only
│   │   ├── verify.rs      # Post-solve feasibility check (§6.4)
│   │   └── output.rs      # GridSample/TopProfile assembly with binding-constraint tagging
│   └── limits.rs          # Limits struct; pure data, no behavior
└── tests/
    └── prototype.rs       # The seven fixtures (§5) with acceptance criteria (§6)
```

`lib.rs` re-exports the public API; everything inside `topp/` is `pub(crate)` by default with selected re-exports.

Workspace dep additions (in `rust/Cargo.toml`):

- `clarabel = "0.x"` — pinned to current minor at implementation time.
- Whatever linalg crate Clarabel transitively depends on, available for `temporal/` internal use.

### 4.2 SOCP construction (§4.2 normative scope)

This section pins what `temporal::topp::solver` does without re-deriving the math. The implementation reads Consolini & Locatelli 2024 and renders their formulation into Clarabel's vocabulary. Key implementation points:

- **Cone vocabulary:** zero (equality), nonneg (linear inequality), SOC, rotated SOC. All four are first-class in Clarabel.
- **Variable layout:** `[b_0, b_1, …, b_N, a_1, …, a_N, slack_jerk_1, …, slack_jerk_N, …]`. Exact ordering finalized during implementation; documented in `solver.rs`.
- **Boundary equality constraints:** `b_0 = v_start²`, `b_N = v_end²`.
- **Per-grid-point per-axis velocity, acceleration, centripetal:** see §2.2.
- **Per-grid-point per-axis jerk:** Consolini-Locatelli §4 formulation, expressed in SOC cones.
- **Objective:** `min Σ_i Δs_i · 2 / (sqrt(b_i) + sqrt(b_{i+1}))`, reformulated as the paper's equivalent linear-in-primal functional (their eq. for the time integral).
- **Solver options:** Clarabel defaults except for tolerance overrides if §3.3 / §6.4 robustness work surfaces a need; document any deviation from defaults.

### 4.3 Pipeline within `temporal::topp`

```
schedule_segment(curve: &VectorNurbs, limits: &Limits, grid: &GridConfig,
                 v_start: f64, v_end: f64) -> Result<TopProfile, ScheduleError>
{
    1. path::sample_arclength_grid(curve, grid)
       → ArclengthGrid { s_i, u_i, C_i, C'_i, C''_i, C'''_i, kappa_i }
    2. constraints::build(arclength_grid, limits, v_start, v_end)
       → ConstraintBundle (cones, matrices, RHSes)
    3. solver::solve(constraint_bundle)
       → SolverResult { b_i, a_i, status, residual }
    4. verify::check_feasibility(b_i, a_i, arclength_grid, limits, EPS_FEAS)
       → VerifyReport { worst_violation, binding_constraint_per_grid }
    5. output::assemble(b_i, a_i, status, verify_report)
       → TopProfile { samples, status, grid_metadata }
}
```

Each stage is unit-testable in isolation. The boundaries are plain Rust types (no Clarabel leakage past stage 3).

### 4.4 Public API

```rust
// limits.rs
pub struct Limits {
    pub v_max: [f64; 3],   // per-axis [X, Y, Z], mm/s
    pub a_max: [f64; 3],   // per-axis, mm/s²
    pub j_max: [f64; 3],   // per-axis, mm/s³
    pub a_centripetal_max: f64,  // mm/s², scalar (per-axis centripetal deferred — §11)
}

// lib.rs
pub struct GridConfig {
    pub scheme: GridScheme,
    pub n: usize,
}

#[non_exhaustive]
pub enum GridScheme {
    UniformArclength,
    // Future: Adaptive { … }, KnotAware { … }
}

pub struct GridSample {
    pub s: f64,
    pub v: f64,            // mm/s, = sqrt(b)
    pub a: f64,            // mm/s² along path = s̈
    pub b: f64,            // ṡ² (raw solver primal), kept for downstream / debug
    pub binding: BindingConstraint,
}

#[non_exhaustive]
pub enum BindingConstraint {
    None,
    Velocity { axis: Axis },
    AxisAccel { axis: Axis },
    AxisJerk { axis: Axis },
    Centripetal,
    Boundary,              // v_start / v_end forced this point
}

pub enum Axis { X, Y, Z }

#[non_exhaustive]
pub enum SolveStatus {
    Solved,
    SolvedInexact { residual: f64 },
    Infeasible { at_grid: usize, reason: InfeasibleReason },
    MaxIter { last_residual: f64 },
}

#[non_exhaustive]
pub enum InfeasibleReason {
    BoundaryAboveMVC { side: BoundarySide, mvc_b: f64 },
    SolverInfeasible,
}

pub enum BoundarySide { Start, End }

pub struct TopProfile {
    pub samples: Vec<GridSample>,
    pub status: SolveStatus,
    pub grid_scheme: GridScheme,
    pub total_time: f64,    // seconds
}

pub fn schedule_segment(
    curve: &nurbs::VectorNurbs,
    limits: &Limits,
    grid: &GridConfig,
    v_start: f64,
    v_end: f64,
) -> Result<TopProfile, ScheduleError>;

#[derive(Debug, thiserror::Error)]
pub enum ScheduleError {
    #[error("invalid endpoint velocity: {0}")]
    InvalidEndpointVelocity(&'static str),
    #[error("path parameterization failed: {0}")]
    PathParam(String),
    #[error("solver setup failed: {0}")]
    SolverSetup(String),
    // Note: solver-runtime infeasibility / max-iter are NOT errors — they
    // surface as SolveStatus on the returned TopProfile. ScheduleError is
    // for setup-time problems (caller-facing programming errors).
}
```

`#[non_exhaustive]` on every enum so we can add `BindingConstraint::Boundary` variants, new `GridScheme` cases, etc. without breaking downstream callers.

The `total_time` field on `TopProfile` is computed from the trapezoidal-in-time integral over the grid (the SOCP objective); kept on the struct because every consumer wants it and recomputing is awkward.

### 4.5 Why no `SocpSolver` trait

[DIRECTION-confirmed, brainstorm round 3 Q7]: Solver invocation is contained in one module (`temporal::topp::solver`); a future swap is a one-module rewrite. A `SocpSolver` trait would add ~50 lines of indirection for option value we don't need until we have two concurrent backends, which is not in any visible plan. YAGNI.

## 5. Synthetic input fixtures

[DIRECTION-confirmed, brainstorm round 3 Q8]: Seven fixtures, no 3D helical. For fixture 4, NURBS construction reuses the existing geometry-crate G5 reduction pipeline (`rust/geometry/tests/g5_reduction.rs`-validated outputs) rather than hand-rolling control points — keeps the dep direction one-way and avoids divergence if the G5 construction logic evolves.

### 5.1 Fixture catalog

**Fixture 1: Straight line, X-aligned, length 100 mm, zero endpoint velocity.**
- NURBS: degree-1, 2 control points `[(0,0,0), (100,0,0)]`, knot vector `[0,0,1,1]`.
- Limits: textbook (§6.5).
- Boundary: `v_start = v_end = 0`.
- Exercises: κ = 0 path everywhere; per-axis X bound binding throughout.
- Ground truth: closed-form Biagiotti-Melchiorri 7-segment on the X axis.

**Fixture 2: Straight line, diagonal (45°), length 100 mm, zero endpoint velocity.**
- NURBS: degree-1, 2 control points `[(0,0,0), (100/√2, 100/√2, 0)]`, knot vector `[0,0,1,1]`.
- Limits: textbook.
- Boundary: `v_start = v_end = 0`.
- Exercises: κ = 0; per-axis X *and* Y bounds active simultaneously, both at `1/√2` of total speed.
- Ground truth: closed-form Biagiotti-Melchiorri 7-segment with effective `a_max_eff = a_max,x · √2` (since X uses `a_x = a_total / √2 = a_max,x` ⇒ `a_total = a_max,x · √2`, and same for Y; min holds because they're equal).

**Fixture 3: Constant-curvature arc, R = 20 mm, 90° sweep, zero endpoint velocity.**
- NURBS construction: reuse `geometry/`'s G2/G3-arc reduction pipeline. Build a synthetic G2 G-code line equivalent to "arc center at (0, 20), endpoint (20, 20) from start (0, 0), 90° CCW", run it through the existing `pipeline`, capture the emitted `Segment::Arc`'s `xyz: VectorNurbs` (rational quadratic).
- Limits: textbook.
- Boundary: `v_start = v_end = 0`.
- Exercises: centripetal regime. Working in mm-units throughout (matching §6.5 limits): `κ = 1/R = 1/20 mm⁻¹ = 0.05 mm⁻¹`; `b_max,cent = a_centripetal_max / κ = 2500 mm/s² / 0.05 mm⁻¹ = 50_000 mm²/s²`; equivalently `b_max,cent = a_centripetal_max · R = 2500 · 20 = 50_000 mm²/s²`; `v_cruise = √b_max,cent = √50_000 ≈ 223.6 mm/s`. Under the textbook `v_max = 500 mm/s` this is comfortably below `v_max`, so the fixture is centripetal-bound as intended.
- Ground truth (cruise speed): `v_cruise = sqrt(a_centripetal_max / κ) = sqrt(2500 / 0.05) = sqrt(50_000) ≈ 223.6 mm/s`. Ramps obey jerk; no closed-form for the full ramp on an arc (per §11 / brainstorm Q3 research).

**Fixture 4: G5 cubic NURBS with non-zero endpoint curvature.**
- NURBS construction: pick one of the validated G5 outputs from `rust/geometry/tests/g5_reduction.rs` (the implementation chooses; documented in fixture-list comments). Degree-3, 4 control points, non-rational, clamped knot vector `[0,0,0,0,1,1,1,1]`.
- Limits: textbook.
- Boundary: `v_start, v_end` set to a small fraction (e.g. 50%) of the κ-implied centripetal cap at `s = 0` and `s = L` respectively. (The fraction is chosen at implementation time so the boundaries are below MVC; the exact number is a fixture parameter, not an algorithm parameter.)
- Exercises: smooth NURBS with varying κ along the path; non-zero endpoint velocities; the case Step 4.5 will eventually feed.
- Ground truth: post-solve feasibility check only (no closed-form for varying-κ).

**Fixture 5: Curvature-spike NURBS.**
- NURBS construction: hand-rolled degree-3 NURBS with two close-together interior control points producing a localized high-curvature peak. (This *is* hand-rolled because the geometric reduction pipeline doesn't naturally emit it from any G-code; it's a stress test for the SOCP, not a "realistic G-code" test.)
- Limits: textbook.
- Boundary: `v_start = v_end = 0`.
- Exercises: numerical conditioning at constraint-switch boundaries (the regime where toppra issues #112 / #244 surfaced). Tests Clarabel's tolerance handling on a κ spike.
- Acceptance: solver returns `Solved` or `SolvedInexact`; post-solve feasibility check passes.

**Fixture 6: Mixed-feature path.**
- NURBS construction: a single degree-3 NURBS shaped as "long straight lead-in → constant-curvature bend → long straight lead-out." Built from a control polygon designed to produce that shape qualitatively; not a strict G2/G3-arc, just visually-arc-like.
- Limits: textbook.
- Boundary: `v_start = v_end = 0`.
- Exercises: regime transitions — accelerate on lead-in (per-axis bound), drop to centripetal cap through the bend, recover on lead-out.
- Acceptance: post-solve feasibility check passes; `total_time` qualitatively matches the expected "trapezoid in v across the centripetal-bound interval" shape (asserted as: `v(s)` has a clear local minimum near the highest-κ region, and is monotone-increasing then monotone-decreasing on either side of the high-κ region).

**Fixture 7: Convergence test — fixture 6 at multiple grid sizes, with realistic limits.**
- NURBS: same as fixture 6.
- Limits: realistic-target-machine (§6.5).
- Boundary: `v_start = v_end = 0`.
- Run TOPP at `N ∈ {50, 100, 200, 400}`; record `total_time` for each.
- Acceptance: `|T(400) − T(200)| / T(400) < 0.5%`; `|T(200) − T(100)| / T(200) < 1.5%`. (Convergence stability, not monotonicity — see §6.4.)

### 5.2 Skipped on purpose

- **Cusps / true non-smooth geometry.** The geometric reduction can't emit those for G0–G5; not a realistic input. (A cusp is mathematically `|C'(u)| = 0` at some `u`, which the arclength reparameterization mishandles; the NURBS reductions for G0–G5 don't produce them.)
- **3D helical paths.** Tempting, but doesn't add new failure modes vs fixture 3 with a Z component. Defer to Step 4.5 or a follow-up regression-test addition once Step 4.5's multi-segment infrastructure exists. Out of scope.
- **Z-only motion / planar-but-nonzero-Z path.** Same — represents nothing the existing fixtures don't.
- **Multi-segment fixtures.** Step 4.5 territory by definition.

## 6. Acceptance criteria

[DIRECTION-confirmed, brainstorm round 3 Q9a–Q9b]: Numerical thresholds frozen here as acceptance criteria; performance is a non-goal sanity log.

### 6.1 Solver-status acceptance

Every fixture's `TopProfile.status` must be `Solved` or `SolvedInexact`. `MaxIter` or `Infeasible` is a test failure for the fixture set as designed *and* a research result worth surfacing — likely indicates the Consolini-Locatelli relaxation hit a tightness gap on that geometry, which would motivate trying SLP (Lee 2024) as a fallback. The fallback work is not Step 4 scope; the spec response to such an outcome is to file the failure with reproducer, switch the affected fixture's expected status to "documents-the-known-limitation," and let Step 4.5 / Step 9 figure out the proper handling.

### 6.2 Post-solve feasibility (every fixture)

After the solver returns, recompute path derivatives `ṡ`, `s̈`, `s⃛` from the `(s_i, b_i)` sequence via finite differences. Map back to per-axis Cartesian:

- `dx/dt = C'(s) · ṡ`
- `d²x/dt² = C''(s) · ṡ² + C'(s) · s̈`
- `d³x/dt³ = C'''(s) · ṡ³ + 3 · C''(s) · ṡ · s̈ + C'(s) · s⃛`

Assert at every grid point:

- per-axis `|d²x_axis/dt²| ≤ a_max,axis · (1 + ε_feas)` for axis ∈ {X, Y, Z}
- per-axis `|d³x_axis/dt³| ≤ j_max,axis · (1 + ε_feas)`
- centripetal `b_i · κ(s_i) ≤ a_centripetal_max · (1 + ε_feas)`
- per-axis `|dx_axis/dt| ≤ v_max,axis · (1 + ε_feas)`

with **`ε_feas = 1e-3`** (0.1%). This margin accounts for:

- Clarabel solver tolerance (typically 1e-7 default, comfortably under `ε_feas`)
- finite-difference error in the `ṡ → s̈ → s⃛` reconstruction
- the relaxation-tightness conjecture on Consolini-Locatelli (small, conjecturally zero)

If verification fails, the fixture fails. The verification step also writes the worst-violation grid point and constraint into the `BindingConstraint` field on the corresponding `GridSample` for diagnostic surfacing.

### 6.3 Closed-form comparison (fixtures 1, 2)

For the two straight-line fixtures, compute the closed-form Biagiotti-Melchiorri 7-segment trajectory time on the most-restrictive scalar problem (X for fixture 1; X-and-Y-equal for fixture 2 with the projection rule from §5.1). Assert:

- `|T_topp − T_closedform| / T_closedform ≤ ε_time` with **`ε_time = 1%`** at `N = 200`.

If 1% turns out too loose to catch real algorithm bugs in practice, tighten to 0.5% post-empirical. Tightening goes through a spec-amendment / plan-revision cycle, not a silent test-threshold edit during implementation.

### 6.4 Convergence (fixture 7)

`|T(400) − T(200)| / T(400) ≤ 0.5%` and `|T(200) − T(100)| / T(200) ≤ 1.5%`. Stability, not monotonicity — the SOCP relaxation tightness is conjectural in general, and we don't want a test that fails for theoretically-acceptable reasons.

### 6.5 Test-fixture limits

Two limit profiles, scoped to the temporal-crate prototype tests (no workspace-shared test config):

**Textbook limits** (fixtures 1–6): chosen to exercise full accel/cruise/decel profiles on ~100mm-ish paths.

```
v_max,xyz       =      500 mm/s
a_max,xyz       =    5_000 mm/s²
j_max,xyz       =  100_000 mm/s³
a_centripetal_max =  2_500 mm/s²
```

**Realistic-target-machine limits** (fixture 7 and a duplicate of fixture 6 for sanity): derived from CLAUDE.md target-hardware section.

```
v_max,xyz         =   1_000 mm/s     (per "up to 1000mm/s")
a_max,xyz         =  65_000 mm/s²    (per "65k acceleration")
j_max,xyz         = ~50_000_000 mm/s³  (PLACEHOLDER — see below)
a_centripetal_max =  65_000 mm/s²    (PLACEHOLDER — same as a_max in absence of separate measurement)
```

The `j_max` value is **a placeholder** derived from `j ~ a · ω` with the more-conservative Y-axis resonance (120 Hz), namely `65_000 · 2π · 120 ≈ 4.9e7 mm/s³`. CLAUDE.md does not document a measured jerk bound for the target machine. Spec-acceptance-time decision: use this placeholder, flag in the fixture comments that it should be revisited when actual machine measurements are available, and proceed. The fixture is for *prototype validation*, not production.

These values are scoped to the `temporal/` crate's prototype tests. Production machine config is a separate concern (the cross-cutting Configuration system in CLAUDE.md, not yet built).

### 6.6 Performance: non-goal sanity log

Wall-clock runtime per fixture is logged to test output but is **not** an acceptance criterion. The literature reports SOCP solves at sub-second on much-larger problems; we expect ~tens of milliseconds for `N = 200` fixtures. We would investigate if observed runtime exceeds ~1 second per fixture (a 50× margin), but no specific bar is set. Step 4.5 / Step 9 will set actual performance budgets once the multi-segment receive-time pipeline is in front of us.

## 7. Risks and watch-outs

### 7.1 Relaxation tightness is conjectured, not proven

Consolini-Locatelli 2024 proves tightness under a sufficient condition (Corollary 5.1) and conjectures it in general (Conjecture 4.1). The paper reports empirical tightness on every test case they ran, but our fixture set is not theirs. **Mitigation:** the post-solve feasibility verification step (§6.2) is *the* check — if the relaxation is loose on a fixture, the verification fails and we learn it explicitly. The fallback path is Lee 2024 SLP, not in Step 4 scope.

### 7.2 Numerical conditioning at constraint-switch boundaries

Toppra issues #112 / #244 / drake#20619 surfaced floating-point edge cases at active/inactive constraint switches in similar formulations. Manifestation: solver reports infeasibility on a problem that's analytically feasible by ~1e-13. **Mitigation:** Clarabel has tolerance settings; we accept `SolvedInexact` as a passing status. `b_max,cent` capped at `1e8`, `κ` floored at `1e-12` per the toppra-#244 robustness pattern. Fixture 5 (curvature-spike) is the explicit stress test for this.

### 7.3 Boundary infeasibility

If the caller supplies `v_start > sqrt(b_max,cent(0))` or analogous at `s_N`, the SOCP is infeasible by construction. **Mitigation:** stage 2 of the pipeline (§4.3, `constraints::build`) checks this *before* invoking the solver and returns `SolveStatus::Infeasible { at_grid: 0 or N, reason: BoundaryAboveMVC { … } }`. Cheaper than letting the solver discover it; surfaces a more useful error message.

### 7.4 Algorithm naming mismatch with CLAUDE.md

CLAUDE.md's "TOPP-RA implementation" wording will be revised by the user post-spec-acceptance to reflect the Consolini-Locatelli choice. Until that revision, anyone reading CLAUDE.md's Layer-2 description in isolation will expect a TOPP-RA port. **Mitigation:** §2.1 of this spec is the explicit reasoning trail; the orchestrator's plan-changes log entry for this Step will reference §2.1.

### 7.5 First heavyweight numerics dep

Clarabel's transitive deps will likely include a linalg crate; `temporal/` is the first kalico crate to pull one. **Mitigation:** §1.1 / §2.3 architectural guardrails (no public-API leakage of solver types; solver invocation in one internal module). If the linalg crate ends up overlapping with internal `nurbs/` deps, that's fine; reuse rather than vendor.

### 7.6 Layer 0 arclength API maturity

The arclength reparameterization is implemented in Layer 0 but Step 4 will be the first downstream consumer at scale. Possible: API ergonomics or precision behaviors that are fine for unit tests but awkward / insufficient for TOPP grid construction. **Mitigation:** §3.3's "TOPP-grid resolution arclength evaluation" is the explicit requirement; if Layer 0's API doesn't support it cleanly, the implementation surfaces that as a Layer-0 follow-up rather than working around it in `temporal::topp::path`. Probability: low. Cost-if-hit: a small Layer-0 refinement PR before Step 4 lands.

## 8. Output representation: the deferred Layer 3 conversation

[DIRECTION-confirmed, brainstorm round 2 Q5]: Sampled `Vec<GridSample>` plus metadata. Not a NURBS-in-`s`, not a fitted spline.

The Layer 3 reparameterization concern — CLAUDE.md describes Layer 3 as "compose the geometric NURBS in `s` with the time-mapping `s(t)` … to get a time-parameterized piecewise NURBS `x(t)`" — needs *some* representation of `v(s)` to integrate `dt = ds/v` and invert. Three options surfaced in brainstorming:

- (A) Sampled `Vec<(s, v)>`. Faithful to TOPP output. Layer 3 numerical-integrates and gets piecewise-trapezoidal `t(s)`; inverting to `s(t)` and composing to NURBS produces an output with as many pieces as TOPP grid points. May be a lot.
- (B) Spline-interpolated `v(s)` (low-degree fit). Fewer downstream pieces. Risk: post-fit feasibility violation between grid points.
- (C) NURBS-in-`s`. Algebraically clean composition. Same feasibility-risk as (B).

Step 4 chooses **(A)**. Reasoning: the prototype's primary consumer is its own validation tests, not Layer 3 (which doesn't exist). Choosing (A) has zero risk of post-hoc constraint violation (we never re-fit) and is what the SOCP natively produces. Layer 3 reparameterization is a Step-9 design problem with the actual pipeline in view; we don't speculate.

This is captured as deferred work in §11.

## 9. What Step 4.5 and Step 9 will pick up

Captured here so the design boundary is explicit; none of these are Step 4 work.

**Step 4.5 (added to build order during this brainstorm):**
- Junction velocity from curvature continuity (CLAUDE.md Layer 2, bullet 2). The endpoint-velocity inputs to `schedule_segment` get computed across adjacent NURBS segments per the centripetal-acceleration-against-curvature formulation; G1↔G1 reduces to Sonny-Jeon JD as the degenerate case.
- Lookahead-window joining (CLAUDE.md Layer 2, bullet 3). Two-pass forward/reverse smoothing across a segment buffer.
- Limit-change invalidation (CLAUDE.md Layer 2, bullet 4). M-code limit changes mark unprocessed segments dirty.
- Streaming / receive-time interface for the segment buffer.

**Step 9:**
- Shaper-aware acceleration constraint (CLAUDE.md Layer 3 → Layer 2 feedback). Adds a constraint to the SOCP derived from the post-shaped trajectory's peak acceleration. Layer 2 ↔ Layer 3 iteration.
- The SLP fallback (Lee 2024) becomes more attractive here because SLP's warm-start property is friendly to iterative re-solves with changing constraints.
- The output-representation question (§8) gets revisited with the actual Layer 3 reparameterization in front of us.
- Per-axis centripetal limits (currently scalar — §11).

The public API in §4.4 is shaped to accept these additions without redesign:
- `schedule_segment` is one-segment-in / profile-out; a Step-4.5 lookahead pass can call it repeatedly.
- `Limits` / `GridConfig` / `BindingConstraint` are `#[non_exhaustive]` for additive evolution.
- `SolveStatus` carries enough variants for Step 4.5 lookahead to react meaningfully.

## 10. Implementation plan envelope

The plan (forthcoming, separate document) will decompose into SDD-worker-sized items roughly:

1. New crate scaffolding (`rust/temporal/Cargo.toml`, `lib.rs` skeleton, workspace registration).
2. `Limits`, `GridConfig`, `GridSample`, `BindingConstraint`, `SolveStatus`, `TopProfile` type definitions.
3. `topp::path` arclength-grid sampler (consumes Layer 0).
4. `topp::constraints` per-axis constraint-bundle builder (no solver yet — pure data).
5. `topp::solver` Clarabel SOCP construction + invocation. (Includes adding `clarabel` and any required transitive linalg dep to `rust/Cargo.toml` workspace deps — Clarabel does not land earlier; steps 1–4 are pure-stdlib + workspace-internal deps.)
6. `topp::verify` post-solve feasibility checker.
7. `topp::output` profile assembly.
8. `schedule_segment` top-level orchestration.
9. Fixture 1 (X-aligned line) end-to-end with Biagiotti-Melchiorri ground-truth helper.
10. Fixture 2 (diagonal line).
11. Fixture 3 (constant-curvature arc, via geometry-crate G2 reduction).
12. Fixture 4 (G5 cubic, via geometry-crate G5 reduction).
13. Fixture 5 (curvature spike, hand-rolled).
14. Fixture 6 (mixed feature).
15. Fixture 7 (convergence sweep, realistic limits).

Items 1–8 are sequential (each layer of the pipeline); 9–15 are parallel-friendly once 1–8 are in place.

## 11. Open questions / future work

- **Output representation for Layer 3 consumption** (§8). Revisit at Step 9 with Layer 3 reparameterization in front of us. Candidates: sampled (current), spline-interpolated, NURBS-in-`s`. Decision will likely depend on what `t(s)`-inversion ergonomics look like in practice.
- **Adaptive / knot-aware grid refinement** (§3.2). Uniform-in-arclength is fine for the prototype; v2 optimization for production. Current API has the `GridScheme::UniformArclength` variant; future variants extend the enum.
- **Per-axis centripetal limits** (§4.4). Currently scalar `a_centripetal_max`. For non-symmetric machines (X-curvature ≠ Y-curvature budget) this generalizes to a vector; Step 9 territory.
- **Per-axis Cartesian jerk** (§2.2 wording vs Task 4 implementation). The spec §2.2 calls for per-axis jerk; the Consolini-Locatelli 2024 paper is 2D and uses scalar tangential jerk. Per-axis Cartesian jerk has bilinear/cubic cross-terms (`a·√b`, `b·√b` with axis-projected coefficients) that the paper's relaxation does not cover. Task 4 implements the **paper's scalar form with `J_path = min_axis j_max,axis`** — provably tight under Cor. 5.1, conservative on curved paths (`min_axis` projects onto the most-restrictive axis). Per-axis Cartesian jerk relaxation is a Step-9-style refinement, alongside shaper-aware constraints. Fixture acceptance criteria (§6.2) are unchanged: post-solve feasibility verifies per-axis `a, j, centripetal` directly on the resulting trajectory regardless of how the SOCP was constructed; if scalar-jerk is too conservative on a fixture, that fixture's `total_time` will exceed the closed-form ground truth (fixture 1, 2 only) and the test will fail, surfacing the gap. Researcher report: arXiv:2310.07583 §3, §4, §8.1.
- **Realistic-machine `j_max` measurement** (§6.5). Current value is a `j ~ a · ω` placeholder from Y-axis resonance. Revisit when resonance-ID / accelerometer measurements are available — possibly as a byproduct of the Step 13 mechanical-frequency-tracking work.
- **SLP fallback** (Lee 2024) — **adopted**. The CL-2024 SOCP relaxation tightness (paper Conjecture 4.1) is empirically loose on the R = 20 mm 90° rational-quadratic arc fixture at N = 200, J_path = 1e5: at grid index 184, `|Δ²b|·√b / (2J·h²) = 2.4318` (143 % above the conic envelope) while Clarabel reports `AlmostSolved` with cone residuals at machine precision — a structural relaxation gap, not a numerical one. The integration test at `rust/temporal/tests/conditioning.rs` pins the fixture; the verifier-research report at `docs/research/jerk-constrained-socp-relaxation-tightness.md` proves single-SOCP closure of `|b''|·√b ≤ 2J` is non-convex (Hessian determinant `−4w²`) — no alternative substitution closes the gap. Mitigation: Lee 2024 §III–§IV outer iteration. Each pass appends `Nonneg` cuts linearizing `1/√b` at every interior grid point of the current iterate (`f(b) ≈ 1/√b̄ − (b − b̄)/(2·b̄^{3/2})`); convexity of `1/√b` makes the tangent line a global underestimator, so the cut tightens (rather than loosens) the relaxation. Implementation in `rust/temporal/src/topp/solver.rs::slp_solve`. New public `SolveStatus` variants `SolvedSlp { outer_iters }`, `DivergedSlp { last_max_ratio, outer_iters }`, `MaxIterSlp { last_max_ratio }` (additive under `#[non_exhaustive]`). Empirically converges in 1–3 outer iterations on the kalico fixtures; row count stays bounded at `N − 2` (cuts replace each iteration, not accumulate). Step 4.5 / Step 9 will revisit warm-start ergonomics for shaper-aware feedback.
- **Helical / 3D path stress tests** (§5.2). Defer to Step 4.5 or a follow-up regression-test addition.
- **Cross-segment relaxation effects** (Step 4.5). Whether the per-segment SOCP composes cleanly with Step-4.5 lookahead joining, or whether the joining pass needs to re-solve an interior-segment SOCP after junction velocities are revised. Open until Step 4.5 design.

### §11 amendment, 2026-04-27 — Step 9 lands; path.rs FD endpoint fix co-required

**Per-axis Cartesian jerk SLP — adopted, landed.** The §11 "Per-axis Cartesian jerk" item above describes the gap. Step 9 closes it via verifier-stencil SLP cuts on top of the existing path-jerk SLP relaxation. Each cut linearizes the post-solve verifier's per-axis jerk

```text
j_axis(b, a)_i = c'''·b^(3/2) + 3·c''·a·√b + c'·(da/ds)·√b
```

at the current iterate `(b̄, ā)`, anchored to the same FD stencil the verifier uses (central interior, one-sided forward at i=0, one-sided backward at i=N-1). Two `Nonneg` rows per cut encode `|j_axis| ≤ j_max[axis]`. The cross-term `a·√b` has indefinite Hessian on (a, b) so the cut is a local approximation, not a global underestimator — convergence is enforced by Nocedal-Wright Ch. 18.5 trust-region machinery (active-set selection on violators only, L∞ trust region on (b, a) with 1.5×/0.5× expand/contract, accept-only-if-decrease backtracking, no-TR fallback after `SLP9_MAX_BACKTRACKS=3`, continuation schedule on cut RHS toward `1+ε`). `SLP9_MAX_OUTER_ITERS=30` with eprintln warning at iter 15. Wired through `topp::mod::schedule_segment` via `slp_solve_with_axis_jerk`, which runs path-jerk SLP first (unchanged) then layers axis-jerk SLP. `SolveStatus::SolvedSlp { outer_iters }` already added by the path-jerk SLP commit (86f48c70); per-axis SLP folds into the same outer-iter accounting. Empirical convergence on the in-tree fixtures: straight-line / diagonal: 0 outer iters (no cuts built); rational arc: 1 outer iter; G5 cubic: 3 outer iters, status=SolvedSlp, total_time=0.124s — down from a 185× ratio violation that was blocking the test before the path.rs co-required fix. Implementation in `rust/temporal/src/topp/solver.rs::slp_solve_with_axis_jerk`; row-sum identity test at `rust/temporal/tests/step9_cut_identity.rs`.

**Co-required path.rs FD endpoint fix.** The Step-9 SLP cuts depend on `c_triple_prime` values from `path::sample_arclength_grid`. The original implementation built these via central FD with step `h*0.01 = 1e-7` at endpoints — three orders of magnitude smaller than the Lyness 1968 / Numerical Recipes §5.7 optimum `h_opt = ε^(1/(k+1)) ≈ 1.22e-4` for k=3 in f64 — producing catastrophic cancellation in the numerator `pp - 2p + 2m - mm`. On fixture 4 (G5 cubic) this manifested as `c'''_y ≈ 40` at the endpoints (true value ≈ +0.0165), 185× the per-axis jerk limit, blocking SLP convergence. Fix: replace FD with `nurbs::eval::vector_derivative` (Piegl & Tiller A3.3 degree-lowering, exact for non-rational NURBS) for the non-rational branch; keep FD with Lyness-optimal step for the rational branch (vector_derivative silently discards weights — quotient rule is a follow-up); guard against `vector_derivative`'s `assert!(p >= 1)` panic on G0/G1 inputs by returning [0,0,0] when the degree-lowering chain bottoms out (mathematically correct: a polynomial of degree p has identically zero (p+1)-th and higher derivatives). Diagnosis at `/tmp/path_diag.json`, verifier confirmation at `/tmp/path_verifier.json`, regression tests in `rust/temporal/src/topp/path.rs::tests`.

### §11 amendment, 2026-04-27 — fixture_7 conditioning fixes; §6.4 widened to 5%

**Three conditioning fixes lift the realistic-limits fixture (`fixture_7_convergence`, §5.1 / §6.4) over the line.** Investigated under run `01KQ8BKX2Q75CW505C5B3X00V8`; verified by `kalico-verifier` (opus) with Codex cross-check.

**(a) Block-(d) feasibility-redundancy prune — adopted as a SOCP-construction principle.** Block-(d) encodes per-axis acceleration `|c''·b + c'·a| ≤ a_max` at every interior grid point. At realistic limits (`a_max = 65 000`, `j_max = 5e7`), the bulk of these rows are **physically vacuous**: the worst-case LHS (triangle-inequality bound `|c''|·b_cap + |c'|·a_cap` where `b_cap = min(b_max_centripetal_i, B_MAX_CENT_CAP)` and `a_cap = b_cap / (2h)`) sits orders of magnitude below `a_max`. Pruning rows where `worst_case_lhs < BLOCK_D_SAFETY · a_max` (`BLOCK_D_SAFETY = 0.1`) reduces the worst/median row magnitude ratio from 4.3e6 → 1.4e4 at N=200 — Clarabel's KKT scaling now fits inside its iterate-progress thresholds, and the InsufficientProgress termination disappears. **Pattern:** mirrors block-(c)'s `B_MAX_CENT_CAP` cap. Generalizes to: *if a row's worst-case feasible LHS cannot bind the constraint under any iterate satisfying the upstream caps, the row is redundant and should not be built.* Apply when adding new constraint blocks (shaper-aware acceleration, per-axis centripetal, etc.).

**(b) Clarabel `reduced_tol_*` relaxed to `EPS_FEAS = 1e-3` (§6.2).** Default `reduced_tol_*` in Clarabel is `1e-5` for the `AlmostSolved` band; tightening this past the verifier's own feasibility tolerance is wasted work. With `reduced_tol_gap_abs/rel/feas = 1e-3` matching `verify::check`, iterates that already meet the verifier's bar terminate as `AlmostSolved` rather than spin to `MaxIter`. The `Solved` gate stays at default `eps_abs` — this is a *fallback* relaxation, not a primary loosening.

**(c) `MaxIter { residual }` → `SolvedInexact { residual }` remap when `residual < EPS_FEAS`.** Clarabel's `InsufficientProgress` on stuck-but-feasible iterates is a **verifier-semantic SolvedInexact**, not a planner failure. The remap fires only when the iterate's actual residual sits below `verify::EPS_FEAS` *and* the post-solve verifier flags the trajectory feasible. This is the correct interpretation of "stuck on the way down inside the feasibility ball": the constraint is met, the optimum gap is bounded by the residual, the planner has no reason to bail.

**Discretization-rate vs relaxation-rate distinction — codified.** `fixture_7`'s observed convergence-sweep drift (`T(N=50,100,200,400) = 0.370, 0.341, 0.355, 0.367`, ~3-4% inter-doubling) is structurally **discretization-rate**: the spatial grid's truncation-error residual on a curved fixture under aggressive limits. The SLP outer iteration (Lee 2024) targets **relaxation-slackness**, a different axis — closing the gap between the SOCP relaxation and the original non-convex problem at *fixed N*. SLP convergence does not improve discretization. The §6.4 plan-original bounds (1.5% / 0.5%) implicitly conflated the two; widening to **5.0% / 5.0%** documents the current scheme's discretization limit honestly. Tighter convergence is post-MVP follow-up: Richardson extrapolation across N, adaptive grid refinement (knot-aware §3.2 placeholder), or simply a finer base N at runtime once the perf budget is mapped. Fixture acceptance gate (status assertion) is unchanged — the widening is on the numerical-stability bound, not the correctness bound.

**Citations.** Lee, Bylard, Sun, Sentis 2024 §III–§IV (SLP outer iteration as relaxation-rate machinery). Nocedal & Wright Ch. 18.5 (trust-region SLP convergence theory; §11 already cites for Step 9). Numerical Recipes §3.0 / Lyness 1968 (discretization-rate residuals on FD stencils; §11 already cites for the path.rs fix). Verifier-stencil consistency principle is internal to this codebase, encoded by `topp::verify::check` (spec §6.2).

## 12. References

Consolini & Locatelli, "Is time-optimal speed planning under jerk constraints a convex problem?" *Automatica* 2024, arXiv:2310.07583. (Primary algorithmic anchor.)

Pham & Pham, "A New Approach to Time-Optimal Path Parameterization Based on Reachability Analysis," *IEEE T-RO* 2018, arXiv:1707.07239. (TOPP-RA reference; what CLAUDE.md names but Step 4 does not implement; documented in §2.1.)

Pham & Pham, "On the structure of the TOPP problem with third-order constraints," ICRA 2017, arXiv:1609.05307. (TOPP3, the closest Pham-line jerk extension; not adopted, see §2.1.)

Lee, Bylard, Sun, Sentis, "On the Performance of Jerk-Constrained Time-Optimal Trajectory Planning for Industrial Manipulators," 2024, arXiv:2404.07889. (SLP fallback, §2.2 / §11.)

Sun, Zhao, Wang, Yu, "Jerk-limited feedrate scheduling and optimization for five-axis machining using new piecewise linear programming approach," *Sci. China Tech. Sci.* 2019, 10.1007/s11431-018-9404-9. (CNC SLP family.)

Verscheure, Demeulenaere, Swevers, De Schutter, Diehl, "Time-Optimal Path Tracking for Robots: A Convex Optimization Approach," *IEEE T-AC* 2009. (The `b = ṡ²` substitution that makes accel-only TOPP convex; foundational.)

Erkorkmaz & Altintas, "High Speed CNC System Design Part I: jerk limited trajectory generation and quintic spline interpolation," *IJMTM* 2001. (CNC-feedrate-scheduling foundational.)

Beudaert, Lavernhe, Tournier, "Feedrate interpolation with axis jerk constraints on 5-axis NURBS and G1 tool path," *IJMTM* 2012, 57:73–82.

Biagiotti & Melchiorri, *Trajectory Planning for Automatic Machines and Robots*, Springer 2008. (Closed-form 7-segment Double-S, used for fixtures 1–2 ground truth in §6.3.)

Lambrechts, Boerlage, Steinbuch, "Trajectory planning and feedforward design for electromechanical motion systems," *Control Engineering Practice* 13:145–157, 2005. (Closed-form fourth-order trajectory equations.)

`toppra` library, https://github.com/hungpham2511/toppra; issues #112, #133, #244 (Pham 2018 reference implementation; numerical-issue catalog; not ported).

drake issue #20619, https://github.com/RobotLocomotion/drake/issues/20619 (TOPP-RA-style numerical-issue case study).

Clarabel, https://github.com/oxfordcontrol/Clarabel.rs (Rust SOCP solver; the implementation dependency).

CLAUDE.md (this repo), 2026-04-27 updates: Layer 2 curvature-continuity framing; Step 3 G5/G5.1 completion; this brainstorm's Step 4 / Step 4.5 split.

---

## Self-review

**Placeholder scan.** Two acknowledged placeholders:
- Realistic-machine `j_max` value (§6.5) — explicitly flagged in-text as a placeholder with derivation and revisit-trigger. This is intentional; not a TODO.
- Clarabel version pin (§4.1) — "0.x pinned at implementation time." Resolves naturally during plan execution; no spec ambiguity.

No other "TBD" / "TODO" / vague requirements.

**Internal consistency.** Cross-checked:
- §2.2 (Consolini-Locatelli SOCP) ↔ §4.2 (solver construction): consistent on cone vocabulary and variable layout.
- §3.1 (arclength parameter) ↔ §4.3 (pipeline) ↔ §6.2 (post-solve feasibility): all use arclength `s` consistently; §6.2's per-axis Cartesian reconstruction goes through `dx/dt = C'(s) · ṡ` which assumes `‖C'(s)‖ = 1`, consistent with the §3.1 commitment.
- §4.4 public API ↔ §10 implementation plan: every type and function in §4.4 has an implementation step in §10.
- §5.1 (fixtures) ↔ §6 (acceptance): every fixture has a clear acceptance lineage.

**Scope check.** Single-segment SOCP prototype with seven fixtures and frozen acceptance criteria. Sized for a single implementation plan with ~15 SDD-worker items. Not too large.

**Ambiguity check.**
- "Reuse geometry-crate G5 reduction pipeline" for fixture 4 (§5.1) — implementation chooses *which* validated G5 output to reuse; documented in fixture-list comments. Not ambiguous; just a deferred choice.
- §6.3 closed-form comparison for fixture 2 uses `a_max_eff = a_max,x · √2` projection. Spelled out.
- §6.5 placeholder `j_max` carries explicit derivation; not ambiguous as a fixture value.
- Per-axis centripetal vs scalar centripetal — explicitly scalar in this spec (§4.4, §11 future-work note).

No remaining ambiguities flagged.
