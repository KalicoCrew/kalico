We are working on a complete rewrite of the motion planner and more:

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
- **The spline fitter (Layer 1) is the highest-risk item in the spec.** Streaming/windowed/real-time-tolerance fitting in a hobby-firmware context is genuinely novel. TOPP-RA (Layer 2) is porting work by comparison — well-published with reference implementations. Build the fitter prototype early in Python or similar before committing to C/C++.
- **Shaper-aware TOPP-RA (Layer 3 → Layer 2 feedback) is the highest-leverage throughput optimization in the spec, but it's a refinement, not an independent feature.** Build the planner first without it; add the shaper-aware constraint once you have something running. Don't try to implement them simultaneously.
- **Phase stepping (Layer 4) requires Layer 0 MCU NURBS eval but is otherwise independent of higher math layers.** Build a "dumb" version that takes pre-computed step times and does phase modulation, validate the phase-stepping firmware on its own, then integrate with the trajectory evaluator. De-risks two complex things developing in parallel.
- **EtherCAT is genuinely additive, not coupled.** Layers 1–3 don't change for EtherCAT; only Layer 4 swaps. The architectural commitment ("planner output is curve + v(s), backend evaluates") is what makes this true. Don't build EtherCAT until phase stepping works end-to-end.

## Suggested build order

1. **NURBS library** (host + MCU) and arc-length tools — Layer 0
2. **G-code parser and geometric reduction** (no fitting yet, just direct NURBS from G0/G1/G2/G3/G5) — partial Layer 1
3. **TOPP-RA prototype on synthetic input** — partial Layer 2
4. **MCU framework with stub NURBS evaluator and basic kinematics** — partial Layer 4, with the runtime-evaluation slots designed in even if unused
5. **Communication protocol and clock sync** — Layer 5
6. **First-print MVP: end-to-end with junction-deviation on G1, plus G2/G3 native, plus ZV shaper. No PA, no fitting, no smooth shapers.** This actually prints from existing slicers — the corner velocities will be conservative (no fitting means lots of velocity reductions at slicer-emitted G1 vertices) but it produces parts.
7. **Spline fitter and parameterized corner-blend emission** — completes Layer 1's geometric output. Until Layer 3 corner-blend finalization lands (step 8), corner-blend slots fall back to junction-deviation.
8. **Smooth shapers, shaper-aware TOPP-RA, and corner-blend shape finalization** — completes Layer 3 and refines Layer 2.
9. **Tanh PA on MCU** (runtime evaluation against base E NURBS) — refines Layer 4
10. **Phase stepping current synthesis** — completes Layer 4
11. **Skip detection and telemetry** — Layer 4 acquisition + Layer 5 transport + cross-cutting events
12. **Mechanical-frequency tracking** — Layer 6
13. **EtherCAT backend** — Layer 6, after everything else

Step 6 is the minimum viable proof of concept — a printer that prints, with most things in their final architectural shape but limited features. **Note that PA is deliberately absent from MVP.** The user committed to tanh PA (nonlinear, runtime-evaluated), so introducing a linear-PA path that gets thrown away would be dead code. First-print validation can be done without PA; it just shows blob/zit at corners until step 9 lands.

Steps 7–10 are where it becomes high-performance. Steps 11–13 are polish and future-proofing. **The transition from step 6 to step 7 is psychologically the hardest** — you have something that works, and you're tearing into it to add features that may break it. Plan for that.
