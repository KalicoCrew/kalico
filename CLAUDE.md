We are working on a complete rewrite of the motion planner and more:

# Non-negotiable constraints

- **Print throughput is non-negotiable.** The planner never knowingly chooses a cheaper algorithmic architecture that produces a measurably slower trajectory than the best one we can compute on the active hardware. "Best we can compute" is realistic — finite discretization N, local-optimum convergence (SLP for the non-convex jerk relaxation; the Consolini-Locatelli SOCP itself is convex but not a closed-form), tolerance settings tuned to the hardware budget. Within those engineering realities, the planner aims for the tightest trajectory it can; we do not give up trajectory time to make planning easier. Host compute is something we spend in service of trajectory optimality — not the other way around. If the Pi can't keep up, the answer is to optimize the implementation, parallelize across cores, or upgrade the host; the answer is never to ship a cheaper algorithm that produces a measurably slower trajectory on representative slicer output. State-of-the-art is the target, not safe-and-good-enough.

# High level feature scope
- Rust end-to-end for new code; single source compiled f64 host / f32 MCU. Rust links as staticlib into Klipper's existing C MCU build, which stays C for now.
- NURBS-native, internal primitive through the planner.
- Support for G2, G3, G5, G5.1. spline-fitting for older slicers that emit g1-dense gcode.
- Phase stepping with open loop steppers with BTT Octopus pro and similar (H723 chip)
- EtherCAT support as a future backend, with the planner architecturally designed to accommodate it
- Regular stepping for non-phase-capable drivers (e.g. 2209 on Z)
- Only smooth shaper support, pre-baked into NURBS. Possibly impulse shapers in the future as composition.
- Extruder is synchronized to the motion after IS is applied.
- Non-linear PA from bleeding-edge kalico, applied IS-then-PA
- Axis limits are calculated against shaped dynamics (shaper aware TOPP-RA, not fixed de-rating)
- Third order motion as primary profile
- User configurable corner rounding. Optimal blend shape (curve family + control parameters) is genuinely dynamic-limit-dependent — the curve that minimizes time through a corner at a given tolerance differs across accel/jerk regimes — so shape selection happens in Layer 3 with full dynamic-limit context, not at geometric receive time.
- Real time communication with MCUs, no queue-based offload.
- Trajectory evaluation on MCU at modulation rate (20-40kHz) for true phase stepping. MCU receives the shape with PA and IS already baked in, to reduce load.
- Telemetry as a first-class subsystem
- Explicit position/step decoupling. For future closed loop support.
- Real-time per-axis offset applied outside the planner, for bed mesh, thermal expansion compensation, and probing.
- Asymmetric PA (separate K for accel vs decel)


# Target hardware
- A rigid machine with single spike on each axis resonance graph. 120hz on Y and 180hz on X
- With regular klipper it could achieve motion up to 1000mm/s and 65k acceleration with 65scv before skipping steps.
- Extruder could achieve roughly 50k with acceptable pressure advance before acceleration becomes too high.
- Max flow of about 80mm cubic.
- Host: Pi 5
- MCU1: Octopus Pro with H723, 4 5160 drivers for AB steppers, 1 more 5160 for extruder
- MCU2: Octopus with F4x chip, 2209 for Z

# Nice to have
- A mechanical-frequency tracking system separate from the shaper, alerting on drift without auto-applying changes
- Ability to gerenate a look up table of rotor angles for a given
  microstep for phase stepping using toolhead mounted accelerometer. 



Following Dependency graph is AI generated and might contain small
errors, please point out if you notice one.

# Dependency Graph

High-performance FDM with NURBS-native planning, phase stepping, and EtherCAT-ready architecture.

## Overview

The build is organized into seven layers (0–6). Each layer depends on everything below it; items **within** a layer can be developed in parallel. The unusual structure comes from the algebraic-closure principle of NURBS: linear/rational operations bake into the trajectory at receive time on the host; transcendental operations defer to MCU runtime evaluation. This split determines what depends on what.

## Layer 0 — Mathematical foundations

Pure libraries with no firmware coupling. Unit-testable in isolation against synthetic input. **Nothing else can start until these exist.**

- **NURBS evaluation library.** de Boor's algorithm, derivative computation (degree-lowering), curvature κ(u) from first and second derivatives. Both host (double precision) and MCU (single precision, M7-optimized) implementations. The MCU version is the hot path — every cycle here costs you on every sample.
- **Arc-length parameterization.** u(s) computation for a given NURBS, via Gaussian quadrature with caching or precomputed monotone tables. Host-only.
- **NURBS algebraic operations.** Multiplication-by-scalar, sum-of-NURBS, NURBS multiplication (pointwise product, Piegl & Tiller ch. 5), and *convolution-with-polynomial-kernel* (this last one is what makes smooth-shaper pre-bake possible and is the least standard operation — likely something to implement from the basis-function math, builds on NURBS multiplication). Host-only.

## Layer 1 — Geometry pipeline

Depends on Layer 0. Produces NURBS segments from g-code input.

- **G-code parser** with G0/G1/G2/G3/G5 support and standard CNC features (work coordinates, override characters).
- **Geometric reduction:** G0/G1 → degree-1 NURBS, G2/G3 → exact rational quadratic NURBS, G5 → direct.
- **G1-sequence spline fitter.** Windowed B-spline fitting with configurable tolerance. **This is the highest-risk item in the spec by a meaningful margin** — offline fitting literature exists (Tajima/Sencer, Beudaert) but the streaming/windowed/real-time-tolerance case in a hobby-firmware context is genuinely novel research, not just porting. Build a prototype early.
- **Junction-deviation fallback** for G1 sequences the fitter can't smooth (very short, deliberate sharp corners, explicit non-smoothable). The machine drives through the geometric corner with velocity reduction; geometry stays exact.
- **Parameterized corner-blend slots** as a third path for deliberately sharp corners that need smoothing but aren't fittable as continuous curves. Layer 1 emits the slot — in/out tangents, tolerance budget, segment-length context — but defers curve-family choice and control-point placement to Layer 3 where dynamic limits are known. Cubic Bezier is the default family (degenerate cubic NURBS, integrates cleanly); per Tajima & Sencer 2016, optimal-time-through-corner shape genuinely varies with accel/jerk ratio, so this is not a fixed-geometry path.

The three corner paths form a fallback chain: **fitter handles what it can → cubic Bezier blends explicitly-marked sharp corners → junction-deviation handles the rest.** Output: a stream of NURBS segments with metadata about source g-code.

## Layer 2 — Temporal scheduling

Depends on Layer 1 NURBS output. Produces v(s) per segment.

- **TOPP-RA implementation.** Time-optimal velocity scheduling against acceleration, jerk, and curvature constraints. Host-side, runs at receive time. Porting + adaptation work — `toppra` Python library exists but is offline-robotics-oriented; needs adaptation for streaming use, not novel research.
- **Junction velocity from curvature continuity.** v_max at any segment boundary derives from the curvature on both sides at the junction parameter (u=1 of segment N, u=0 of segment N+1) under the centripetal-acceleration constraint v²·κ ≤ a_max. The Sonny-Jeon junction-deviation algorithm is the **degenerate special case** for G1↔G1 boundaries — both sides have zero curvature except a delta at the corner; JD computes the deviation budget against a chord-error tolerance. G1↔G5, G5↔G5, fitter-output↔anything, and future G6.2 NURBS↔anything all flow through the same computation; only the curvature-evaluation source changes (zero for G1, NURBS κ(u) for any smooth segment). Implication for Layer 1: do not fabricate "virtual G1 directions" at smooth-curve endpoints to feed JD — break the G1-tangent chain at any non-G1 segment, and let Layer 2 evaluate end-tangents and end-curvatures from the NURBS itself.
- **Lookahead-window joining.** Two-pass forward/reverse smoothing across the segment buffer to reconcile end-of-N velocities with start-of-N+1 velocities. Standard planner work.
- **Limit-change invalidation logic.** Mark unprocessed segments dirty on M-code limit changes, recompute v(s) only for them.

Build Layer 2 with the unshaped dynamics constraint first — that gets you a working planner. Shaper-awareness is a Layer 3 add-on that feeds back into Layer 2's constraint set.

## Layer 3 — Trajectory transformations (pre-bake)

Depends on Layers 1 and 2. This is where the algebraic-closure principle plays out. Linear/rational transformations bake here at receive time; transcendental ones get punted to Layer 4 for runtime evaluation.

### Pre-bakes on the host

- **Corner-blend shape finalization.** Take Layer 1's parameterized blend slots (tolerance budget + tangent + segment-length context) and select curve family + control-point placement to minimize time through the corner under current dynamic limits and ringing budget. Output replaces the slot with a finalized NURBS in the segment stream. Per Tajima & Sencer 2016. Runs before TOPP-RA — geometry must be finalized before v(s) is computed against it.
- **Impulse-shaper application:** produce per-axis impulse table that travels with the segment.
- **Reparameterize geometry to time.** After TOPP-RA produces v(s), compose the geometric NURBS in s with the time-mapping s(t) (inverse of t(s) = ∫ds/v) to get a time-parameterized piecewise NURBS x(t). This is a NURBS-of-piecewise-polynomial composition; result has more pieces per segment (~3–7) but stays piecewise-polynomial. Required because the shaper math is time-domain.
- **Smooth-shaper application:** convolve the time-reparameterized NURBS x(t) with the polynomial kernel w(t) analytically, produce shaped (higher-degree) NURBS in t. Kernel support is a few ms; output piece count grows by O(input pieces × kernel pieces).
- **Shaper-aware acceleration constraint:** because x(t) is known in closed form post-shaping, peak shaped acceleration is derivable from its derivatives directly. Feed this back to TOPP-RA as a constraint. The "shaper-overshoot factor" is a derived quantity, not a magic number. This is the Layer 2 ↔ Layer 3 feedback. Implement Layer 2 first without it; add as refinement.

### Defers to Layer 4 (does not pre-bake)

- **Tanh/Kalico nonlinear PA** — send base E + PA params, MCU evaluates at runtime.
- **Same-shaper-on-extruder for tanh PA** — runtime evaluation since the underlying PA is runtime.

## Layer 4 — MCU runtime

Depends on Layer 0 (NURBS eval, MCU side) and Layer 3 (knows what arrives over the wire). Receives trajectory descriptions, evaluates at modulation rate (~40 kHz).

- **Real-time MCU framework.** Sample-rate clock at 40 kHz, segment buffer holding 2–3 adjacent segments for shaper-boundary handling.
- **Per-axis evaluator.** Composes (in order): base or pre-shaped NURBS evaluation, kinematic transform (CoreXY/Cartesian), runtime PA tanh evaluation if applicable, runtime shaper application if applicable (only for E with nonlinear PA; XY and linear-PA E are already pre-baked).
- **Phase-stepping current synthesis.** Electrical-angle map from mechanical position, sin/cos current setpoints, driver SPI/UART output. Tightly coupled with the per-axis evaluator.
- **Hybrid stepping for non-phase-capable axes.** Trajectory evaluation produces position; digitize to step events for TMC2209-class drivers.
- **Skip detection acquisition.** MSCNT or encoder reading at ~100 Hz, threshold check, event emission.

## Layer 5 — Communication infrastructure

The protocol must exist before MCU and host can talk end-to-end. Stub early; schema can evolve. **Layer 4 and Layer 5 co-evolve** — Layer 4's data needs drive what the protocol must carry, and the protocol schema is a hard prerequisite for Layer 4 integration testing.

- **Self-describing protocol** (Klipper-style data dictionary). Carries trajectory descriptions, config, telemetry events, skip events.
- **Multi-MCU clock synchronization.** Continuous frequency estimation per MCU.
- **Telemetry transport** (event types defined cross-cuttingly, transported here).

## Layer 6 — External features and UX

Depends on most layers being functional.

- **Mechanical-frequency tracking.** Accelerometer-based continuous resonance ID parallel to the shaper, alerts on drift.
- **EtherCAT backend.** **Replaces Layer 4**, not added on top. Layers 1–3 unchanged. Cyclic-RT host-side trajectory evaluation feeds EtherCAT slaves; slaves do local interpolation to current-loop rate. Requires RT_PREEMPT host with proper IRQ affinity / CPU isolation.
- **Slicer integration / config UI.** Once you know what config knobs you actually need.

## Cross-cutting concerns

These don't fit cleanly into a single layer because they touch multiple layers throughout development.

- **Telemetry.** Hooks at every layer (planner-state events from Layers 2–3, MCU-state events from Layer 4). Define event types, format, and emission points cross-cuttingly. Transport happens in Layer 5; alerting/logging/visualization in Layer 6.
- **Configuration system.** Touches all layers. **Pick a representation early** — YAML on flash (FluidNC-style), TOML/INI (Klipper/LinuxCNC), JSON object model (RRF/g2core). The choice affects Layer 5 (protocol carries config queries) and Layer 6 (UI integration). Don't follow Marlin's `Configuration.h` preprocessor model.

## Dependency diagram

```
                    ┌─────────────────────────────────┐
                    │  Layer 6: External & UX         │
                    │  - Frequency tracking           │
                    │  - EtherCAT (replaces L4)       │
                    │  - Slicer integration / UI      │
                    └────────────────┬────────────────┘
                                     │
            ┌────────────────────────┴────────────────────────┐
            │                                                 │
            ▼                                                 ▼
  ┌─────────────────────┐                       ┌─────────────────────┐
  │ Layer 4: MCU runtime │ ◄────────────────────►│ Layer 5: Comms      │
  │ - Real-time MCU FW   │                       │ - Self-describing   │
  │ - Per-axis evaluator │                       │   protocol          │
  │ - Phase synthesis    │                       │ - Clock sync        │
  │ - Hybrid stepping    │                       │ - Telemetry         │
  │ - Tanh PA runtime    │                       │   transport         │
  │ - Skip detection     │                       │                     │
  │   (acquisition)      │                       │                     │
  └──────────┬───────────┘                       └─────────────────────┘
             │
             ▼
  ┌─────────────────────────────────────────────────────────┐
  │ Layer 3: Trajectory transformations (pre-bake)          │
  │ - Corner-blend shape finalization (host)                │
  │ - Impulse-shaper application (host)                     │
  │ - Smooth-shaper convolution into NURBS (host)           │
  │ - Shaper-aware accel constraint ──┐ feedback to Layer 2 │
  │ - Tanh PA params (passes through to Layer 4)            │
  └──────────┬──────────────────────────┴────────────────────┘
             │                          │
             ▼                          ▼
  ┌─────────────────────────┐  ┌─────────────────────────┐
  │ Layer 2: Temporal       │  │ Layer 1: Geometry       │
  │ - TOPP-RA               │◄─│ - G-code parser         │
  │ - Lookahead joining     │  │ - Geometric reduction   │
  │ - Invalidation logic    │  │ - Spline fitter         │
  │                         │  │ - Junction-deviation    │
  │                         │  │   fallback              │
  │                         │  │ - Corner-blend slots    │
  └──────────┬──────────────┘  └────────────┬────────────┘
             │                              │
             └──────────────┬───────────────┘
                            ▼
  ┌─────────────────────────────────────────────────────────┐
  │ Layer 0: Mathematical foundations                       │
  │ - NURBS evaluation (host + MCU)                         │
  │ - Arc-length parameterization                           │
  │ - NURBS algebraic ops (sum, mul, convolution)           │
  └─────────────────────────────────────────────────────────┘

  Cross-cutting (touches all layers): Telemetry, Configuration
```

## Critical-path observations

- **Layer 0 NURBS evaluation on the MCU is the most performance-critical code in the entire stack.** Every cycle saved on de Boor pays back at 40 kHz × axes × impulses. Optimize this last but design the API early — the rest of Layer 4 has to assume it exists and call it heavily.
- **The spline fitter (Layer 1) has been demoted to an optional / offline G1-compatibility addon (build-order Step 13).** Original framing — streaming/windowed/real-time-tolerance fitting as the highest-risk item in the spec — was driven by the assumption that all input would be G1-dense forever. With kalico-aware slicer emission of G5 directly (parallel workstream by user, see below), the fitter becomes a one-shot offline file pre-processor only used to support legacy slicers, where the standard CNC literature (Tajima/Sencer 2016, Beudaert 2012) applies directly and the streaming/online-tolerance properties are not needed. Risk: low. Critical-path: no longer.
- **Shaper-aware TOPP-RA (Layer 3 → Layer 2 feedback) is the highest-leverage throughput optimization in the spec, but it's a refinement, not an independent feature.** Build the planner first without it; add the shaper-aware constraint once you have something running. Don't try to implement them simultaneously.
- **Phase stepping (Layer 4) requires Layer 0 MCU NURBS eval but is otherwise independent of higher math layers.** Build a "dumb" version that takes pre-computed step times and does phase modulation, validate the phase-stepping firmware on its own, then integrate with the trajectory evaluator. De-risks two complex things developing in parallel.
- **EtherCAT is genuinely additive, not coupled.** Layers 1–3 don't change for EtherCAT; only Layer 4 swaps. The architectural commitment ("planner output is curve + v(s), backend evaluates") is what makes this true. Don't build EtherCAT until phase stepping works end-to-end.

## Suggested build order

1. [x] **NURBS library** (host + MCU) and arc-length tools — Layer 0
2. [x] **G-code parser and geometric reduction** (no fitting yet, just direct NURBS from G0/G1/G2/G3/G5) — partial Layer 1
   - G0/G1 → degree-1 NURBS, G2/G3 → 3D rational quadratic NURBS (helical-capable), JunctionDeviation between consecutive G1s. Telemetry routing for LayerChange / ToolChange / Retraction. **G5/G5.1 not yet handled** — lexer parses them but reduce silently drops; deferred until needed.
3. [x] **G5 / G5.1 reduction** — closes the remaining gap in step 2. Lexer already tokenizes G5/G5.1 (Task 6 of the Phase 1 plan). Per LinuxCNC RS274NGC convention: **G5** → degree-3 single-piece NURBS with 4 control points (P0=current, P1=current+I,J, P2=end+P,Q, P3=end); **G5.1** → degree-2 single-piece NURBS with 3 control points (P0=current, P1=current+I,J, P2=end), restricted to the active plane (G17/G18/G19). Implement the RS274NGC modal-chain implicit-tangent rule for G5: when a G5 immediately follows another G5 with both I,J omitted, default I,J to −(prev P, prev Q) for C¹ continuity; emit a parser error if the implicit tangent is unavailable (chain broken by intervening motion-producing g-code) and explicit I,J are missing. Both G5 and G5.1 break the G1-tangent chain — Layer 2 derives endpoint curvature from the NURBS per the curvature-continuity principle (Layer 2 description above). Small follow-up to step 2; should land before step 7's spline-fitter work begins.
4. [ ] **TOPP-RA prototype on synthetic input** — partial Layer 2. De-risk the algorithm itself: time-optimal v(s) on a single synthetic NURBS at a time, against accel + jerk + curvature constraints, with externally-supplied (or zero) endpoint velocities. No cross-segment glue, no streaming/invalidation. Validates jerk-bounded TOPP-RA on a NURBS path with proper discretization before it gets wired to multi-segment input.
4.5. [ ] **Layer 2 multi-segment integration on synthetic input** — completes Layer 2. Junction velocity from curvature continuity (subsumes Sonny-Jeon JD as the G1↔G1 degenerate case), lookahead-window joining (two-pass forward/reverse smoothing across the segment buffer), and limit-change invalidation logic. Operates on a synthetic multi-segment NURBS buffer; wiring to live Layer 1 output is implicit in step 7. Must precede step 7 (MVP needs JD-quality cornering for G1↔G1, which is now a degenerate path through this same machinery).
5. [ ] **MCU framework with stub NURBS evaluator and basic kinematics** — partial Layer 4, with the runtime-evaluation slots designed in even if unused
6. [ ] **Communication protocol and clock sync** — Layer 5
7. [ ] **First-print MVP: end-to-end with junction-deviation on G1, plus G2/G3 native, plus ZV shaper. No PA, no fitting, no smooth shapers.** Prints from existing slicers — corner velocities will be conservative on G1-dense input (lots of velocity reductions at slicer-emitted G1 vertices). If the parallel kalico-aware-slicer workstream (see below) is ready by MVP time, the same MVP also prints kalico-slicer output with G5-rich corners that look better; the wording above is the floor MVP guarantees, not the ceiling.
8. [ ] **Smooth shapers, shaper-aware TOPP-RA, and corner-blend shape finalization** — completes Layer 3 and refines Layer 2.
9. [ ] **Tanh PA on MCU** (runtime evaluation against base E NURBS) — refines Layer 4
10. [ ] **Phase stepping current synthesis** — completes Layer 4
11. [ ] **Skip detection and telemetry** — Layer 4 acquisition + Layer 5 transport + cross-cutting events
12. [ ] **Mechanical-frequency tracking** — Layer 6
13. [ ] **Spline fitter — optional, offline G1-compatibility addon.** Pre-processes G1-dense input from legacy (non-kalico-aware) slicers into G5-rich form as a one-shot file pass. Standard CNC literature applies (Tajima/Sencer 2016, Beudaert 2012) — none of the streaming/windowed/online-tolerance properties of the original framing are required. Output flows through the normal Layer 1+ pipeline. Optional: users on a kalico-aware slicer never invoke this.
14. [ ] **EtherCAT backend** — Layer 6, after everything else

**Parallel workstream (user, not on the kalico build-order critical path):** kalico-aware slicer fork emitting G5 directly for smooth toolpath segments and G1+tolerance hints for sharp corners (Layer 3 picks blend NURBS shape under dynamic limits per the existing CLAUDE.md feature-scope bullet). Independent of the kalico-side numbering above; affects MVP's corner quality (better with kalico-aware slicer output) but does not gate any kalico-side build-order item.

Step 7 is the minimum viable proof of concept — a printer that prints, with most things in their final architectural shape but limited features. **Note that PA is deliberately absent from MVP.** The user committed to tanh PA (nonlinear, runtime-evaluated), so introducing a linear-PA path that gets thrown away would be dead code. First-print validation can be done without PA; it just shows blob/zit at corners until step 9 lands.

Steps 8–10 are where it becomes high-performance. Steps 11–14 are polish and future-proofing. **The transition from step 7 to step 8 is psychologically the hardest** — you have something that works, and you're tearing into it to add features that may break it. Plan for that.

# Plan changes log

Appended by the kalico orchestrator (`/kalico-orchestrate`) when build-order items, layer scopes, or constraints change. Each entry: date, what changed, why, evidence link.

<!-- entries below -->

## 2026-04-27

**Changed:**
- **Layer 2:** added the "Junction velocity from curvature continuity" bullet — formalizes that v_max at every segment boundary derives from the same centripetal-acceleration-against-curvature formulation; junction-deviation is the degenerate G1↔G1 case, not a separate algorithm.
- **Build-order Step 3 (G5 / G5.1 reduction):** rewrote to specify per-LinuxCNC semantics — G5 is degree-3 with 4 CPs, G5.1 is degree-2 with 3 CPs (restricted to the active plane). Added the RS274NGC modal-chain implicit-tangent rule for G5. Removed the `Segment::Fitted { degree: 3 }` wire-format hint (that's plan-level detail; build-order items stay at semantic-spec granularity).

**Why:** The original Step 3 wording conflated G5 and G5.1 into a single degree-3 recipe, which is wrong for G5.1 under the canonical LinuxCNC convention (G5.1 is a non-rational quadratic B-spline, not a degenerate cubic). Research during brainstorming confirmed Marlin lacks G5.1 entirely, RepRapFirmware lacks both, grblHAL matches LinuxCNC, and Fanuc's `G05.1 Q1` is an unrelated AICC mode toggle on a colliding number — LinuxCNC is the only meaningful spec for G5/G5.1 in the open-source space, so kalico adopts it. The curvature-continuity framing was articulated during Q3 of Step 3 brainstorming as the unifying architectural principle behind G1↔G1 (JD), G5↔G1, G5↔G5, and future fitter-output↔anything junction handling; recorded in CLAUDE.md so it governs all subsequent planner-stage design.

**Evidence:** `brainstormer-step-3` round 1 transcript; two `kalico-researcher` reports (G5.1 cross-firmware semantics; RS274NGC §G5 modal-chain rule); user direction confirmation in this session.

---

**Build-order Step 3 (G5 / G5.1 reduction): completed.** Implementation per `docs/superpowers/plans/2026-04-27-layer-1-g5-reduction.md`. Reduce + pipeline now construct exact non-rational NURBS for G5 (degree-3, 4 CPs) and G5.1 (degree-2, 3 CPs); G5 modal-chain implicit-tangent rule, G5.1 active-plane validation, defensive `prev_g5_pq` clearing on every G5 error path, and curvature-continuity G1-chain break all in place. `ReduceEvent` shape refactored to `ReduceEvent::Curve(CurveGeom, …)` with fixed-size-array variants per the Q5 brainstorm decision (no per-segment heap allocation, distinct `Quadratic` vs `RationalQuadratic` variants per the user-chosen ontology).

**Evidence:** Plan + 18 commits on this branch (range `9c21b59f..` head). Spec at `docs/superpowers/specs/2026-04-27-layer-1-g5-reduction-design.md`. Integration tests at `rust/geometry/tests/g5_reduction.rs` (9/9 passing). Top-level code review by `superpowers:code-reviewer` (opus): approved.

---

**Build-order Step 4 split into Step 4 + Step 4.5.** Step 4 is now narrowed to "TOPP-RA core on single-segment synthetic NURBS" — a de-risk milestone for the algorithm itself (jerk-bounded RA, NURBS-path discretization, convex-program structure). Step 4.5, newly inserted, captures the remaining Layer 2 bullets (junction velocity from curvature continuity, lookahead-window joining, limit-change invalidation) on synthetic multi-segment input. Step 4.5 must precede Step 7 (MVP) since MVP requires JD-quality G1↔G1 cornering, which now flows through the unified curvature-continuity machinery.

**Why:** The build-order phrase "prototype on synthetic input" reads as de-risk, not feature-complete Layer 2. Folding all four Layer-2 bullets into one step diluted what the prototype was validating and risked making it a months-long effort that no longer functioned as an early algorithm-de-risk milestone. Splitting also keeps each step independently reviewable and lets Step 4.5/5/6 develop in parallel across layers without reordering the rest of the plan. User-confirmed direction call recorded by orchestrator (`brainstormer-step-4` round 1, Q1 `[DIRECTION]`).

---

**Added top-level "Non-negotiable constraints" section: print throughput is non-negotiable.** The planner never produces a slower trajectory than the math-optimal one for given geometry, dynamic limits, and shaper. Host compute is spent in service of trajectory optimality, not the other way around — if the host can't keep up the answer is to optimize/parallelize/upgrade, never to ship a cheaper algorithm that produces a slower trajectory.

**Why:** During Step 4.5 (Layer 2 multi-segment) brainstorming, two architectures surfaced: (A) per-segment SOCP re-solve on every joining iteration (math-optimal trajectory; potentially expensive on host), vs. (B) cheap-kinematic forward/reverse joining + SOCP-once-at-finalization (decoupled, ~3–8% slower trajectory than (A) on ramp-bound segments per kalico-verifier analysis, which dominate real slicer output). User direction: a measurable trajectory-time regression vs. math-optimal is never an acceptable trade. The principle generalizes beyond Step 4.5; recording at top level so it governs all subsequent algorithmic-vs-implementation-cost trades.

**Evidence:** Step 4.5 brainstorming this session; two `kalico-verifier` reports (one on M-code-handling option (i)/(ii)/(iii), one on joining-vs-solving option (A)/(B)/(C)). The (B) verification (INCONCLUSIVE — directional correction) explicitly quantifies the 3–8% throughput gap on ramp-bound segments and notes that with kalico's realistic limits (a=65000, j=5e7, v=1000) ramp-bound segments dominate any slicer output with sub-25mm segments.

---

**Spline fitter (formerly build-order Step 8) demoted to optional / offline G1-compatibility addon (now Step 13).** No longer the highest-risk item in the spec; no longer critical-path; no longer ahead of MVP. Standard CNC literature (Tajima/Sencer 2016, Beudaert 2012) applies directly because none of the streaming/windowed/online-tolerance properties from the original framing are required for an offline file pre-processor. Build-order Steps 9–14 renumbered down by one to fill the gap (smooth-shapers/PA/phase/skip/mech-freq), and the EtherCAT backend stays at the end. Critical-path-observation about the fitter being highest-risk replaced with a note that its risk evaporates under the offline framing.

**Added parallel workstream note** (kalico-aware slicer fork emitting G5 directly for smooth paths + G1+tolerance for sharp corners). Documented as a non-build-order parallel item driven by the user; affects MVP corner-quality but does not gate any kalico-side step. CLAUDE.md's existing feature-scope bullet about "Layer 3 picks the optimal blend NURBS under dynamic limits given a slicer-supplied tolerance" continues to govern the slicer↔kalico contract.

**MVP (Step 7) wording adjusted** to clarify that the JD-only-on-G1 description is the *floor* MVP guarantees, not the ceiling — if the parallel slicer workstream is ready in time, the same MVP also handles G5-rich slicer output with better corners, no MVP-side rework required.

**Why:** Conversation surfaced that future kalico-aware slicers will emit G5 directly with proper corner-tolerance hints, eliminating the need for a streaming windowed real-time-tolerance fitter on the kalico host. The fitter remains useful as a one-shot offline pre-processor for legacy G1-only slicer output (PrusaSlicer / Orca / Super / Cura users without the kalico fork), but at that scope it inherits offline literature directly and the framing as "the highest-risk item by a meaningful margin" no longer holds. Critical-path observation accordingly rewritten; build-order Step 8 demoted to Step 13.

**Evidence:** Brainstorming this session — user direction call. Spline-fitter risk reframing was triggered by user observation that "spline fitting should be like an addon you can enable to add compatibility with older gcode," and the parallel-slicer-workstream commitment ("I will work on the slicer in parallel, to emit proper splines"). Edits scoped tightly to user-confirmed changes; broader exploration (cache layer, offline-batch reframe, throughput-as-discretization-bounded) discussed in the same conversation but not adopted.

---

**Build-order Step 4 (TOPP-RA prototype) — Step 9 lands per-axis Cartesian jerk via verifier-stencil SLP, with a co-required path.rs FD endpoint fix.** Step 9 implements the §11 "Per-axis Cartesian jerk" follow-up by layering verifier-stencil SLP cuts on top of the existing path-jerk SLP (commit ce5e962f). Each cut linearizes `j_axis(b, a)_i = c'''·b^(3/2) + 3·c''·a·√b + c'·(da/ds)·√b` at the current iterate against the same FD stencil the post-solve verifier uses. Active-set + L∞ trust region + accept-only-if-decrease backtracking + continuation schedule on cut RHS, with `SLP9_MAX_OUTER_ITERS=30` and a soft warning at iter 15. Empirical convergence: straight-line / diagonal fixtures take 0 outer iters (no cuts built — path-jerk iterate already feasible); rational arc 1 outer iter; G5 cubic 3 outer iters with `total_time = 0.124s`. Co-required fix in `rust/temporal/src/topp/path.rs` (commit 269498ed): the `eval_kth_deriv` central-FD k=3 stencil suffered catastrophic cancellation at endpoints (avail_h floored to 1e-7 vs Lyness-1968 / NR-§5.7 optimum h_opt ≈ 1.22e-4) producing `c'''_y ≈ 40` on fixture 4 endpoints — a 185× ratio violation that blocked SLP convergence. Replaced with `nurbs::eval::vector_derivative` for non-rational, FD with Lyness-optimal step for rational, and a degree-too-low guard returning [0,0,0] for G0/G1 inputs.

**Why:** The §11 future-work item explicitly flagged per-axis Cartesian jerk as Step-9 territory; landing it now closes the spec §6.2 post-solve-feasibility gap on curved-path fixtures and is a precondition for Step 4.5 multi-segment work where every junction's b-iterate is curved. The path.rs FD bug was not visible at Step 4 because the only test exercising endpoint c''' (`cubic_bezier_pins_third_derivative_at_start`) was passing on round-off coincidence at u=0 with 5% tolerance (verified by `kalico-verifier`); the bug surfaced as a hard convergence failure once Step-9 SLP started consuming endpoint c''' values. Both fixes co-shipped in a four-commit sequence (path.rs fix → row-sum identity test → SLP wire-up → spec amendment).

**Evidence:** Diagnosis at `/tmp/path_diag.json`, verifier confirmation at `/tmp/path_verifier.json`. Spec §11 amendment recorded in `docs/superpowers/specs/2026-04-27-layer-2-topp-prototype-design.md` (this commit). Plan at `docs/superpowers/plans/2026-04-27-layer-2-topp-prototype.md`. Commits: `269498ed` (path.rs fix), `03aa47bc` (cut-identity test), `ce5e962f` (SLP wire-up + fixture 4 widened acceptance).

# Babysitter

Babysitter wraps the existing `kalico-orchestrate → kalico-brainstormer → kalico-researcher → kalico-plan-reviewer` flow into a persistent run-state with retry/refine loops, so brainstorm/research/plan-review cycles run to completion without user nagging. The eventual fix is a bespoke `kalico/ceo-brainstorm` babysitter process that replaces the `/kalico-orchestrate` slash command end-to-end (auto-routing technical questions to `kalico-researcher`, surfacing only intent-level questions). Until that process is authored, `cradle/project-install` has produced the encoded project profile at `.a5c/project-profile.json` and stop-gap reference processes are listed below.

## Recommended commands

- `babysitter:project-install` — already run; produces / updates `.a5c/project-profile.json`. Re-run only if project state changes materially (new layer started, build order revised, methodology swap).
- *Future* `kalico/ceo-brainstorm` — to be designed in a follow-up. Replaces `/kalico-orchestrate` for the brainstorm/research/plan-review pipeline. Build with `babysitter:assimilate` + `specializations/meta/process-creation.js` as references.
- Stop-gap reference processes adopted until the custom process lands:
  - `methodologies/spec-kit` — primary spine. Maps onto the existing `docs/superpowers/{specs,plans}/YYYY-MM-DD-<topic>` flow; CLAUDE.md grand-plan + Plan-changes-log slot in as the constitution.
  - `methodologies/gsd/iterative-convergence` — quality-gated implement→test→score→refine loop, intended for the highest-risk research items (notably the streaming windowed spline fitter; build-order step 8).

## Behavior notes (CEO-mode)

- Breakpoints fire to the user only on **vision questions** ("why are we building this") and **scope-boundary decisions**. Implementation-approach choices, operator-visible behavior knobs, and CLAUDE.md plan-changes-log additions are auto-routed to `kalico-researcher` or answered by the orchestrator.
- Reviewer subagents (`kalico-plan-reviewer`, `superpowers:code-reviewer`) always run on opus.
- No CI/CD integration. Babysitter is host-only; sota-motion's linear-rebase discipline stays the integration story.

Full encoded profile: `.a5c/project-profile.json`. Adopted-process detail per run: `artifacts/tool-recommendations.json` inside each run dir under `.a5c/runs/`.

