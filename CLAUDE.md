We are working on a complete rewrite of the motion planner and more:

# Non-negotiable constraints

- **Print throughput is non-negotiable.** The planner never knowingly chooses a cheaper algorithmic architecture that produces a measurably slower trajectory than the best one we can compute on the active hardware. "Best we can compute" is realistic — finite discretization N, local-optimum convergence (SLP for the non-convex jerk relaxation; the Consolini-Locatelli SOCP itself is convex but not a closed-form), tolerance settings tuned to the hardware budget. Within those engineering realities, the planner aims for the tightest trajectory it can; we do not give up trajectory time to make planning easier. Host compute is something we spend in service of trajectory optimality — not the other way around. If the Pi can't keep up, the answer is to optimize the implementation, parallelize across cores, or upgrade the host; the answer is never to ship a cheaper algorithm that produces a measurably slower trajectory on representative slicer output. State-of-the-art is the target, not safe-and-good-enough.

# High level feature scope
- Rust end-to-end for new code; single source compiled f64 host / f32 MCU. Rust links as staticlib into Klipper's existing C MCU build, which stays C for now.
- NURBS-native internal primitive through the planner. **Uniform cubic Bézier (degree-3 polynomial) representation** across Layer 1 / 2 / 3 / 4 — no rational NURBS anywhere live, no mixed-degree dispatch, no source-gcode-type special-cases.
- **G5 / G5.1 only — no legacy G-code anywhere in the planner.** G5 → cubic Bézier direct; G5.1 → cubic via exact degree-elevation. The planner has zero internal handling for G0 / G1 / G2 / G3 — no reduction code paths, no `Linear` / `RationalQuadratic` / `FittedSegment` / `ArcSegment` types, no feature-flagged "legacy mode." Anything that reaches the planner is G5 or G5.1; anything else is rejected at the lexer/reduce boundary as a hard error. The compatibility layer (build-order Step 13 — see below) is a separate offline tool that consumes G-code text and emits G-code text: G0/G1/G2/G3-bearing input file in, G5-only file out. It does not link into the planner crates and does not extend the planner's internal types. Step 13's reductions: G1 → single-piece cubic Bézier with collinear control points (exact, no fit error); G2 / G3 → multi-piece cubic Bézier via Goldapp 1991 closed-form circular-arc-to-Bézier (~2 pieces per quarter-arc at 0.1 µm L∞); G5.1 → G5 by exact degree-elevation; optional G1-sequence spline-fitting (Tajima-Sencer 2016, Beudaert 2012) for smoother corners. Kalico-aware slicers emit G5 directly and never invoke the compat layer; legacy slicers' G-code passes through it once offline before printing.
- Phase stepping with open loop steppers with BTT Octopus pro and similar (H723 chip)
- EtherCAT support as a future backend, with the planner architecturally designed to accommodate it
- Regular stepping for non-phase-capable drivers (e.g. 2209 on Z)
- Only smooth shaper support, pre-baked into NURBS. MVP scope is `smooth_zv` and `smooth_mzv` (bleeding-edge-v2 `init_smoother` polynomial kernels); other smooth families (`smooth_ei`, `smooth_2hump_ei`, `smooth_zvd_ei`, `smooth_si`) are post-MVP. Possibly impulse shapers in the future as composition.
- Per-axis kernels: X and Y configurable and required; Z configurable and default-off (passthrough); E is **not** a separately-shaped axis — see next bullet.
- **Extruder is a follower of the shaped toolhead motion, not an independent axis.** During extruding moves, E_actual(t) = `extrusion_per_xy_mm` × ∫₀ᵗ √(Ẋ_shaped(τ)² + Ẏ_shaped(τ)²) dτ — derived per-sample on the MCU from the same shaped XY trajectory the steppers are tracking. This is the "no desync, by construction" foundation: per-axis IS dispersion gets absorbed because E follows actual toolhead arc-length-traveled. No separate "extruder smoother" family; bleeding-edge-v2's `extruder_smoother.py` synchronization-hack does not apply. Retraction / prime / filament-change segments (E motion with no XY) carry their own un-shaped E NURBS.
- Non-linear PA from bleeding-edge kalico (Step 9) layers cleanly onto the E-follows-XY architecture: PA adds `+ advance × ratio_per_xy_mm × |v_xy(t)|` to the integrated extrusion. Asymmetric PA (separate K_accel vs K_decel) dispatches on `sign(v̇_xy)`. No second IS-PA synchronization step — IS and PA share the same shaped-velocity source.
- Axis limits are calculated against shaped dynamics (shaper-aware TOPP-RA via β-medium outer iteration: solve TOPP-RA, pre-bake smooth shaper, evaluate closed-form post-shape peak |ẍ_shaped| from x(t)'s analytic derivatives, derate accel and re-solve until peak ≤ a_machine — no fixed de-rating constant)
- Third order motion as primary profile
- User configurable corner rounding. Optimal blend shape (curve family + control parameters) is genuinely dynamic-limit-dependent — the curve that minimizes time through a corner at a given tolerance differs across accel/jerk regimes — so shape selection happens in Layer 3 with full dynamic-limit context, not at geometric receive time.
- Real time communication with MCUs, no queue-based offload.
- Trajectory evaluation on MCU at modulation rate (20-40kHz) for true phase stepping. MCU receives the shape with PA and IS already baked in, to reduce load.
- Telemetry as a first-class subsystem
- Explicit position/step decoupling. For future closed loop support.
- Real-time per-axis offset applied outside the planner, for bed mesh, thermal expansion compensation, and probing.

# Reference docs

- **Target hardware + nice-to-haves:** [`docs/kalico-rewrite/hardware.md`](docs/kalico-rewrite/hardware.md)
- **Layered dependency graph + completed-step detail + critical-path observations:** [`docs/kalico-rewrite/dependency-graph.md`](docs/kalico-rewrite/dependency-graph.md). Read this when reasoning about cross-layer impact, picking up a layer for the first time, or when a step's historical detail matters.

# Build order (current status)

1. [x] **NURBS library** (host + MCU) and arc-length tools — Layer 0
2. [x] **G-code parser and geometric reduction** (G0/G1/G2/G3 NURBS reduction; G5/G5.1 deferred) — partial Layer 1
3. [x] **G5 / G5.1 reduction** — closes the remaining gap in step 2
4. [x] **TOPP-RA prototype on synthetic input** — partial Layer 2
4.5. [x] **Layer 2 multi-segment integration on synthetic input** — completes Layer 2
5. [x] **MCU framework with stub NURBS evaluator and basic kinematics** — partial Layer 4
6. [x] **Communication protocol and clock sync** — Layer 5
7. [ ] **First-print MVP: end-to-end on G5-only live pipeline (uniform cubic Bézier), with junction-deviation between sharp corners, smooth-ZV/smooth-MZV pre-bake, β-medium shaper-aware TOPP-RA, math-exact time-reparameterization (T-A), per-axis-scalar MCU storage, E-follows-shaped-XY foundation. No PA, no other smooth-shaper families, no corner-blend finalization, no live G0/G1/G2/G3 (handled by Step-13 offline compat layer).** Decomposes into sub-projects:
   - [x] **7-pre — Layer 0 / Layer 1 prep (small):** composition primitive (`x(s(t))` polynomial-of-polynomial in `nurbs::algebra`), gcode-side N≤25-grid-piece cap splitter (Layer 1 reduce stage). G2/G3 / G1 reduction code paths in `geometry::reduce` retired from the live pipeline; their substance moves to Step-13 / compat-layer scope.
   - [x] **7-A — Layer 3 minimum (host):** `trajectory` crate (`rust/trajectory/`). Time-reparam (fit x(s) + exact composition with degree-2 s(t)), C¹ Hermite refit (degree 4, ≤5 µm), per-axis smooth-ZV/smooth-MZV convolution via pad-and-trim + `nurbs::algebra::convolve`, sample-based peak-accel check (40 kHz finite differences), β-medium outer iteration on TOPP-RA accel limits, E-follows-XY metadata emission, independent E trapezoidal scheduling. 62 tests.
   - [x] **7-B — Layer 4 (MCU):** per-axis-scalar curve-pool refactor (bumped MAX_DEGREE / MAX_CONTROL_POINTS / MAX_KNOT_VECTOR_LEN to fit post-shape NURBS), per-sample evaluator with `v_xy` integration for COUPLED_TO_XY E mode, INDEPENDENT-mode E NURBS path for retraction, real step-output replacing trace-only stub, homing/endstops, hybrid stepping for MVP (phase stepping in Step 10).
   - **7-C — klippy bridge + production host I/O:** Python ↔ Rust integration so existing Klipper configs route motion through kalico's planner; production `MsgProtoParser` (data-dictionary JSON parse) and `host_io.rs` (NAK retransmit, async event dispatch, reconnect recovery) — closing Step-6 Plan-decision-C deferrals; `arm_all_mcus` request_id correlation; `ArmError::QualityGate` detail.
   - **7-D — Hardware bring-up + first print:** Surface-C cycle-budget actuals, F4x integration for Z, M1/M2/M3 soaks, calibration, physical first print.

   **Test G-code source:** either the parallel kalico-aware-slicer workstream (which emits G5 directly) or legacy slicer G-code passed through Step-13 compat layer. The OrcaSlicer test corpus in `scripts/fitter_prototype/corpus/` (240-layer Voron cubes, G1-dense and arc-fitted) requires Step-13 normalization before the live pipeline can consume it.
8. [ ] **Corner-blend shape finalization + smooth-shaper-family expansion** — `smooth_ei`, `smooth_2hump_ei`, `smooth_zvd_ei`, `smooth_si` added to the per-axis kernel inventory; corner-blend NURBS shape selection (curve family + control-point placement) per Tajima & Sencer 2016 under full dynamic-limit context. Completes Layer 3.
9. [ ] **Tanh nonlinear PA + asymmetric PA on MCU** — refines Layer 4. Per-segment params (advance_accel, advance_decel, transition shape) layered onto the COUPLED_TO_XY E integration: `e_actual(t) = ratio × ∫|v_xy| dτ + advance(sign(v̇_xy)) × ratio × |v_xy(t)|`. INDEPENDENT-mode E (retraction) gets its own PA path operating on its independent NURBS. No "base E NURBS" emitted for COUPLED segments — the PA velocity term shares the same shaped-XY-velocity source the MVP integration uses.
10. [ ] **Phase stepping current synthesis** — completes Layer 4
11. [ ] **Skip detection and telemetry** — Layer 4 acquisition + Layer 5 transport + cross-cutting events
12. [ ] **Mechanical-frequency tracking** — Layer 6
13. [ ] **Compatibility layer — offline legacy-G-code → G5-only normalizer.** Separate offline tool (its own binary; can live in the same Cargo workspace and share the `gcode` lexer, which already accepts G0/G1/G2/G3 input) that takes legacy slicer output (G0 / G1 / G2 / G3 / G5 / G5.1 mixed) and emits G5-only G-code consumable by the live pipeline. **Pure G-code-text → G-code-text transform.** Step 13 does not import the `geometry` / `nurbs` planner crates' internal segment / NURBS types and does not extend them — those remain G5 / cubic-Bézier-only. Three reduction paths: **G1 → G5** as a single-piece cubic Bézier with collinear control points at 1/3 / 2/3 lerp (degree-elevation, exact); **G2 / G3 → G5** as multi-piece cubic Bézier via Goldapp 1991 closed-form circular-arc-to-Bézier (~2 pieces per quarter-arc at 0.1 µm L∞ — published per-degree constants, no LSQ); **G5.1 → G5** by exact degree-elevation. Optional **G1-sequence spline-fitter** for users who want smoother corners than collinear-cubic G1-by-G1 produces (Tajima/Sencer 2016, Beudaert 2012; standard offline-CNC literature applies — none of the streaming/windowed/online-tolerance properties of the originally-framed live fitter are required). Output is a fresh G5-only G-code file that flows through the live pipeline like any other G5 source. Optional: kalico-aware slicers emit G5 directly and never invoke this.
14. [ ] **EtherCAT backend** — Layer 6, after everything else

# Plan changes log

The running log of build-order/spec/constraint changes lives at [`docs/superpowers/plan-changes-log.md`](docs/superpowers/plan-changes-log.md). Format per entry: date, what changed, why, evidence link.

# General instructions

When calling subagents, if you decide it needs opus model, just omit the model parameter completely, it will select opus by default
