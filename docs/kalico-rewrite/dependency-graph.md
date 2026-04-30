# Dependency Graph

> AI generated and might contain small errors — please point out if you notice one.

High-performance FDM with NURBS-native planning, phase stepping, and EtherCAT-ready architecture.

## Overview

The build is organized into seven layers (0–6). Each layer depends on everything below it; items **within** a layer can be developed in parallel. The unusual structure comes from the algebraic-closure principle of NURBS: linear/rational operations bake into the trajectory at receive time on the host; transcendental operations defer to MCU runtime evaluation. This split determines what depends on what.

## Layer 0 — Mathematical foundations

Pure libraries with no firmware coupling. Unit-testable in isolation against synthetic input. **Nothing else can start until these exist.**

- **NURBS evaluation library.** de Boor's algorithm, derivative computation (degree-lowering), curvature κ(u) from first and second derivatives. Both host (double precision) and MCU (single precision, M7-optimized) implementations. The MCU version is the hot path — every cycle here costs you on every sample.
- **Arc-length parameterization.** u(s) computation for a given NURBS, via Gaussian quadrature with caching or precomputed monotone tables. Host-only.
- **NURBS algebraic operations.** Multiplication-by-scalar, sum-of-NURBS, NURBS multiplication (pointwise product, Piegl & Tiller ch. 5), and *convolution-with-polynomial-kernel* (this last one is what makes smooth-shaper pre-bake possible and is the least standard operation — likely something to implement from the basis-function math, builds on NURBS multiplication). Host-only.

## Layer 1 — Geometry pipeline

Depends on Layer 0. Produces uniform cubic Bézier NURBS segments from g-code input.

- **G-code parser (live pipeline)** accepts G5 / G5.1 (and the standard non-motion CNC machinery — work coordinates, override characters, comments, M-codes routed to telemetry). Legacy G0 / G1 / G2 / G3 are not handled by the live parser; the compatibility layer (Step 13) normalizes those offline to G5-only before the file enters the live pipeline.
- **Geometric reduction:** G5 → cubic Bézier polynomial NURBS direct; G5.1 → cubic via exact degree-elevation (degree 2 → 3, +1 control point, no fit error).
- **Junction-deviation fallback** for sharp corners between consecutive G5 segments where the slicer marked the junction as non-smoothable. The machine drives through the geometric corner with velocity reduction; geometry stays exact. Subsumes the historical G1↔G1 case (post-compat-layer, all corners are G5-collinear↔G5-collinear, treated identically).
- **Parameterized corner-blend slots** for deliberately sharp corners marked with a tolerance budget. Layer 1 emits the slot — in/out tangents, tolerance budget, segment-length context — and Layer 3 (Step 8) selects curve family + control-point placement under full dynamic-limit context, per Tajima & Sencer 2016. Default family is cubic Bézier (matches the live pipeline's uniform-cubic invariant; degenerate forms integrate cleanly).

Output: a stream of cubic Bézier polynomial NURBS segments with metadata (source-g-code line range, feedrate, e-mode, extrusion ratio).

## Layer 2 — Temporal scheduling

Depends on Layer 1 NURBS output. Produces v(s) per segment.

- **TOPP-RA implementation.** Time-optimal velocity scheduling against acceleration, jerk, and curvature constraints. Host-side, runs at receive time. Porting + adaptation work — `toppra` Python library exists but is offline-robotics-oriented; needs adaptation for streaming use, not novel research.
- **Junction velocity from curvature continuity.** v_max at any segment boundary derives from the curvature on both sides at the junction parameter (u=1 of segment N, u=0 of segment N+1) under the centripetal-acceleration constraint v²·κ ≤ a_max. The Sonny-Jeon junction-deviation algorithm is the **degenerate special case** for G1↔G1 boundaries — both sides have zero curvature except a delta at the corner; JD computes the deviation budget against a chord-error tolerance. G1↔G5, G5↔G5, fitter-output↔anything, and future G6.2 NURBS↔anything all flow through the same computation; only the curvature-evaluation source changes (zero for G1, NURBS κ(u) for any smooth segment). Implication for Layer 1: do not fabricate "virtual G1 directions" at smooth-curve endpoints to feed JD — break the G1-tangent chain at any non-G1 segment, and let Layer 2 evaluate end-tangents and end-curvatures from the NURBS itself.
- **Lookahead-window joining.** Two-pass forward/reverse smoothing across the segment buffer to reconcile end-of-N velocities with start-of-N+1 velocities. Standard planner work.
- **Limit-change invalidation logic.** Mark unprocessed segments dirty on M-code limit changes, recompute v(s) only for them.

Layer 2 was built first against unshaped dynamics (Steps 4 + 4.5 — that's a working planner). Shaper-awareness is a Layer 3 add-on that feeds back into Layer 2's constraint set; in MVP it lands as the β-medium outer iteration over Step-4.5's `plan_batch` (see Layer 3 / Step 7).

## Layer 3 — Trajectory transformations (pre-bake)

Depends on Layers 1 and 2. This is where the algebraic-closure principle plays out. Linear/rational transformations bake here at receive time; transcendental ones get punted to Layer 4 for runtime evaluation.

### Pre-bakes on the host

- **Corner-blend shape finalization** *(Step 8, not MVP).* Take Layer 1's parameterized blend slots (tolerance budget + tangent + segment-length context) and select curve family + control-point placement to minimize time through the corner under current dynamic limits and ringing budget. Output replaces the slot with a finalized NURBS in the segment stream. Per Tajima & Sencer 2016. Runs before TOPP-RA — geometry must be finalized before v(s) is computed against it.
- **Reparameterize geometry to time — math-exact (T-A).** After TOPP-RA produces b(s)=v²(s) at N grid points, on each piece b is piecewise-linear in s, so the per-piece time map `s(t) = √b_k·(t−t_k) + (b₁/4)·(t−t_k)²` is *exactly* degree-2 in t (closed-form, not approximated). Composition with polynomial geometry x(s) (degree 1/2/3 for G1/G2-G3/G5) gives `x(t)` as N pieces of degree `2·d_x_geom` ≤ 6 per segment, C¹ at TOPP-RA grid joints (a/jerk/snap discontinuous there). Per-axis scalar storage on the MCU; per-segment N is capped via gcode-side splitting (default cap N≤25 grid pieces ≈ 12.5 mm path-length per "MCU segment") to bound curve-pool slot size. Position-error budget vs. the math-exact reference: 0 by construction.
- **Smooth-shaper application:** convolve the time-reparameterized NURBS x(t) with the polynomial kernel w(t) analytically (per-axis, `smooth_zv` / `smooth_mzv` from bleeding-edge-v2 `init_smoother` — single-piece degree-4 polynomial of compact support `[-T_sm/2, T_sm/2]`, T_sm = 0.8025/f or 0.95625/f). Output is a piecewise-polynomial NURBS in t per axis with breakpoints at the Minkowski sum of input and kernel breakpoints, degree raised by `kernel_degree + 1` (= 5). See `docs/research/bspline-polynomial-convolution.md` for the knot-vector / degree / support bookkeeping.
- **Shaper-aware acceleration constraint (β-medium outer iteration, MVP).** Because shaped x(t) is closed-form piecewise-polynomial, peak `|ẍ_shaped(t)|` per axis is computable in closed form via polynomial extremum / root-finding on the analytic derivative — no L¹-norm bound, no scalar derating constant. Outer loop: solve TOPP-RA → pre-bake smooth shaper → check post-shape peak per axis → if `peak > a_machine`, scale accel limit by `a_machine/peak` and re-solve → iterate to convergence (typically 2–3 outer iterations). Math-optimal trajectory at convergence, modulo TOPP-RA grid discretization. This is the Layer 2 ↔ Layer 3 feedback; in MVP it is wired in, not deferred.
- **E-follows-XY metadata.** For each segment, emit `e_mode ∈ {COUPLED_TO_XY, INDEPENDENT}` plus a scalar `extrusion_per_xy_mm = (ΔE) / √(ΔX² + ΔY²)` for COUPLED segments. INDEPENDENT segments (retraction, prime, filament-change) carry their own un-shaped E NURBS through the normal Layer 3 → Layer 4 pipeline. No per-axis E shaper kernel — by design.

### Defers to Layer 4 (does not pre-bake)

- **Tanh/Kalico nonlinear PA (Step 9, not MVP).** Per-segment params (advance_accel, advance_decel, transition shape) + the shaped XY trajectory. MCU computes `e_actual(t) = ratio_per_xy_mm × ∫|v_xy_actual(τ)| dτ + advance(sign(v̇_xy)) × ratio × |v_xy_actual(t)|` at sample rate. PA shares the same shaped-XY-velocity source the COUPLED_TO_XY E integration uses; no second synchronization layer.

## Layer 4 — MCU runtime

Depends on Layer 0 (NURBS eval, MCU side) and Layer 3 (knows what arrives over the wire). Receives trajectory descriptions, evaluates at modulation rate (~40 kHz).

- **Real-time MCU framework.** Sample-rate clock at 40 kHz, segment buffer holding 2–3 adjacent segments for shaper-boundary handling (kernel support widens segment-edge data dependencies — host-side pre-bake produces a shaped NURBS that is locally exact within `[t_start, t_end]` provided neighboring unshaped segments were available at convolution time).
- **Per-axis evaluator (per sample).** Evaluate pre-shaped per-axis NURBS for X(t), Y(t), Z(t) — each axis is its own scalar NURBS in the curve-pool (per-axis-scalar storage; X uses `smooth_zv` / `smooth_mzv` kernel, Y uses its own kernel, Z is passthrough by default). Apply the kinematic transform (CoreXY / Cartesian) to (X, Y) → (A, B) stepper space. **E in `COUPLED_TO_XY` mode** (extruding moves): `v_xy = √(Ẋ_shaped² + Ẏ_shaped²)`; `e_acc += ratio_per_xy_mm × v_xy × dt`; `e_t = e_acc`. **E in `INDEPENDENT` mode** (retraction / prime): evaluate E's own NURBS directly. Step 9 PA layers in here as `e_t += advance_for(sign(v̇_xy)) × ratio × v_xy` — same shaped-velocity source, no separate runtime shaper.
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
  │ - Time-reparam (math-exact per TOPP grid; T-A)          │
  │ - Smooth-shaper convolution into NURBS (host)           │
  │ - β-medium shaper-aware accel ──┐ outer-iter to Layer 2 │
  │ - E-follows-XY metadata (passes through to Layer 4)     │
  │ - Corner-blend finalization (Step 8, post-MVP)          │
  │ - Tanh PA params (Step 9, passes through to Layer 4)    │
  └──────────┬──────────────────────────┴────────────────────┘
             │                          │
             ▼                          ▼
  ┌─────────────────────────┐  ┌─────────────────────────┐
  │ Layer 2: Temporal       │  │ Layer 1: Geometry       │
  │ - TOPP-RA               │◄─│ - G5/G5.1 parser (live) │
  │ - Lookahead joining     │  │ - Cubic Bézier reduce   │
  │ - Invalidation logic    │  │ - Junction-deviation    │
  │                         │  │ - Corner-blend slots    │
  │                         │  │ - (Step 13 compat layer │
  │                         │  │   normalizes G0/G1/G2/  │
  │                         │  │   G3 offline → G5)      │
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
- **The live pipeline accepts G5 / G5.1 only; legacy G0 / G1 / G2 / G3 normalize to G5 via the offline compatibility layer (build-order Step 13).** This collapses the live pipeline to a uniform cubic Bézier representation with no rational NURBS, no mixed-degree dispatch, no source-gcode-type special-cases. The compat layer subsumes what was originally framed as the "streaming/windowed/real-time-tolerance G1-sequence spline fitter — highest-risk item in the spec" — that streaming framing is no longer needed. As an offline pre-processor, the standard CNC literature (Tajima/Sencer 2016, Beudaert 2012, Goldapp 1991) applies directly. Kalico-aware slicers emit G5 directly and never invoke the compat layer. Risk: low. Critical-path: no longer.
- **Shaper-aware TOPP-RA (Layer 3 → Layer 2 feedback) is wired in MVP via β-medium outer iteration**, not deferred. The original "build planner first, add shaper-aware as refinement" framing was written against an impulse-ZV-runtime MVP; the move to smooth-shaper pre-bake makes closed-form post-shape peak `|ẍ_shaped|` available analytically (polynomial extremum on the shaped NURBS's derivative), so shaper-aware feedback collapses to "solve TOPP-RA → check post-shape peak → derate accel and re-solve" — typically 2–3 outer iterations per segment, embarrassingly parallel across the existing temporal::multi 3-thread batch executor. Math-optimal trajectory at convergence (modulo TOPP-RA grid discretization). Step 8 keeps corner-blend finalization but loses shaper-aware TOPP-RA from its scope.
- **Phase stepping (Layer 4) requires Layer 0 MCU NURBS eval but is otherwise independent of higher math layers.** Build a "dumb" version that takes pre-computed step times and does phase modulation, validate the phase-stepping firmware on its own, then integrate with the trajectory evaluator. De-risks two complex things developing in parallel.
- **EtherCAT is genuinely additive, not coupled.** Layers 1–3 don't change for EtherCAT; only Layer 4 swaps. The architectural commitment ("planner output is curve + v(s), backend evaluates") is what makes this true. Don't build EtherCAT until phase stepping works end-to-end.

## Completed build-order steps (historical detail)

The current build-order checklist lives in `CLAUDE.md`. Detail on completed steps:

1. **NURBS library** (host + MCU) and arc-length tools — Layer 0.
2. **G-code parser and geometric reduction** (no fitting yet, just direct NURBS from G0/G1/G2/G3/G5) — partial Layer 1. G0/G1 → degree-1 NURBS, G2/G3 → 3D rational quadratic NURBS (helical-capable), JunctionDeviation between consecutive G1s. Telemetry routing for LayerChange / ToolChange / Retraction. **G5/G5.1 not yet handled** at this step — lexer parses them but reduce silently drops; deferred until needed.
3. **G5 / G5.1 reduction** — closes the remaining gap in step 2. Per LinuxCNC RS274NGC convention: **G5** → degree-3 single-piece NURBS with 4 control points (P0=current, P1=current+I,J, P2=end+P,Q, P3=end); **G5.1** → degree-2 single-piece NURBS with 3 control points (P0=current, P1=current+I,J, P2=end), restricted to the active plane (G17/G18/G19). Implements the RS274NGC modal-chain implicit-tangent rule for G5: when a G5 immediately follows another G5 with both I,J omitted, default I,J to −(prev P, prev Q) for C¹ continuity; emit a parser error if the implicit tangent is unavailable (chain broken by intervening motion-producing g-code) and explicit I,J are missing. Both G5 and G5.1 break the G1-tangent chain — Layer 2 derives endpoint curvature from the NURBS per the curvature-continuity principle.
4. **TOPP-RA prototype on synthetic input** — partial Layer 2. De-risks the algorithm itself: time-optimal v(s) on a single synthetic NURBS at a time, against accel + jerk + curvature constraints, with externally-supplied (or zero) endpoint velocities. No cross-segment glue, no streaming/invalidation. Validates jerk-bounded TOPP-RA on a NURBS path with proper discretization before it gets wired to multi-segment input.
4.5. **Layer 2 multi-segment integration on synthetic input** — completes Layer 2. Junction velocity from curvature continuity (subsumes Sonny-Jeon JD as the G1↔G1 degenerate case), lookahead-window joining (two-pass forward/reverse smoothing across the segment buffer), and limit-change invalidation logic. Operates on a synthetic multi-segment NURBS buffer; wiring to live Layer 1 output is implicit in step 7. Must precede step 7 (MVP needs JD-quality cornering for G1↔G1, which is now a degenerate path through this same machinery).
5. **MCU framework with stub NURBS evaluator and basic kinematics** — partial Layer 4, with the runtime-evaluation slots designed in even if unused.
6. **Communication protocol and clock sync** — Layer 5.
7-pre. **Layer 0 / Layer 1 prep (small):** composition primitive (`x(s(t))` polynomial-of-polynomial in `nurbs::algebra`), gcode-side N≤25-grid-piece cap splitter (Layer 1 reduce stage). G2/G3 / G1 reduction code paths in `geometry::reduce` retired from the live pipeline; their substance moves to Step-13 / compat-layer scope.
7-A. **Layer 3 minimum (host):** `trajectory` crate (`rust/trajectory/`). Time-reparam (fit x(s) + exact composition with degree-2 s(t)), C¹ Hermite refit (degree 4, ≤5 µm), per-axis smooth-ZV/smooth-MZV convolution via pad-and-trim + `nurbs::algebra::convolve`, sample-based peak-accel check (40 kHz finite differences), β-medium outer iteration on TOPP-RA accel limits, E-follows-XY metadata emission, independent E trapezoidal scheduling. 62 tests.

## Closing notes

**Parallel workstream (user, not on the kalico build-order critical path):** kalico-aware slicer fork emitting G5 directly for smooth toolpath segments and G1+tolerance hints for sharp corners (Layer 3 picks blend NURBS shape under dynamic limits per the existing CLAUDE.md feature-scope bullet). Independent of the kalico-side numbering above; affects MVP's corner quality (better with kalico-aware slicer output) but does not gate any kalico-side build-order item.

Step 7 is the minimum viable proof of concept — a printer that prints, with most things in their final architectural shape and several features that *would* normally be deferred (smooth-shaper pre-bake, shaper-aware TOPP-RA, math-exact time-reparameterization, E-follows-shaped-XY) wired in MVP because they belong to the foundation, not the polish layer. **Note that PA is deliberately absent from MVP.** The user committed to tanh PA (nonlinear, runtime-evaluated, Step 9), so introducing a linear-PA path that gets thrown away would be dead code. First-print validation can be done without PA; it just shows blob/zit at corners until step 9 lands. The E-follows-shaped-XY foundation means Step 9 PA layers in cleanly without an "extruder smoother" synchronization hack.

Steps 8–10 are where it becomes high-performance. Steps 11–14 are polish and future-proofing. **The transition from step 7 to step 8 is psychologically the hardest** — you have something that works, and you're tearing into it to add features that may break it. Plan for that.
