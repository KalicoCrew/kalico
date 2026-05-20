We are working on a complete rewrite of the motion planner and more:

# Non-negotiable constraints

- **Print throughput is non-negotiable.** The planner never knowingly chooses a cheaper algorithmic architecture that produces a measurably slower trajectory than the best one we can compute on the active hardware. "Best we can compute" is realistic — finite discretization N, local-optimum convergence (SLP for the non-convex jerk relaxation; the Consolini-Locatelli SOCP itself is convex but not a closed-form), tolerance settings tuned to the hardware budget. Within those engineering realities, the planner aims for the tightest trajectory it can; we do not give up trajectory time to make planning easier. Host compute is something we spend in service of trajectory optimality — not the other way around. If the Pi can't keep up, the answer is to optimize the implementation, parallelize across cores, or upgrade the host; the answer is never to ship a cheaper algorithm that produces a measurably slower trajectory on representative slicer output. State-of-the-art is the target, not safe-and-good-enough.

# High level feature scope
- Rust for new code by default; single source compiled f64 host / f32 MCU. Rust links as staticlib into Klipper's existing C MCU build, which stays C for now. **C is acceptable for low-level building blocks where Rust's borrow / aliasing abstractions misbehave or obscure debugging** — e.g., the MCU-side segment SPSC queue is a C struct in `.axi_bss` because LLVM miscompiled the borrow-projected `heapless::spsc::Consumer` pattern (2026-05-18 bench: `qlen_sd=6 qlen_ps=1` from the same `Consumer` instance across two call sites, with the Producer's enqueues visible from one site but not the other). The rule is "Rust for the engine, C where the engine's primitives need to be trivially debuggable."
- NURBS-native internal primitive through the planner. **Uniform cubic Bézier (degree-3 polynomial) representation** across Layer 1 / 2 / 3 / 4 — no rational NURBS anywhere live, no mixed-degree dispatch, no source-gcode-type special-cases.
- **G5 / G5.1 only — no legacy G-code in the planner reduce stage.** G5 → cubic Bézier direct; G5.1 → cubic via exact degree-elevation. The planner reduce stage (`rust/geometry`'s reducer + `rust/temporal` + `rust/trajectory`) has zero internal handling for G0 / G1 / G2 / G3 — no reduction code paths, no `Linear` / `RationalQuadratic` / `FittedSegment` / `ArcSegment` types, no feature-flagged "legacy mode." Anything reaching reduce is G5 or G5.1; anything else is rejected at the reduce boundary as a hard error.

  The `rust/gcode` lexer remains capable of tokenizing legacy G-code, because the `compat` crate (Step 13's normalizer) and the bridge's live-G1-conversion path both depend on it. Tokenization is not the rejection boundary.

  The `compat` crate has two callers: the offline Step-13 binary (file → file) and the live bridge (terminal/macro G1/G2/G3 conversion via `compat::collinear::to_collinear_g5`, `compat::arc::arc_to_g5`, `compat::degree_elev::elevate_g51_to_g5`). Both share the lexer.

  Step 13's reductions: G1 → single-piece cubic Bézier with collinear control points (exact, no fit error); G2 / G3 → multi-piece cubic Bézier via Goldapp 1991 closed-form circular-arc-to-Bézier (~2 pieces per quarter-arc at 0.1 µm L∞); G5.1 → G5 by exact degree-elevation; optional G1-sequence spline-fitting (Tajima-Sencer 2016, Beudaert 2012) for smoother corners. Kalico-aware slicers emit G5 directly and never invoke the compat layer; legacy slicers' G-code passes through it once offline before printing.
- Phase stepping with open loop steppers with BTT Octopus pro and similar (H723 chip)
- EtherCAT support as a future backend, with the planner architecturally designed to accommodate it
- Regular stepping for non-phase-capable drivers (e.g. 2209 on Z)
- Only smooth shaper support, pre-baked into the cubic Bézier piece stream the MCU consumes. MVP scope is `smooth_zv` and `smooth_mzv` (bleeding-edge-v2 `init_smoother` polynomial kernels); other smooth families (`smooth_ei`, `smooth_2hump_ei`, `smooth_zvd_ei`, `smooth_si`) are post-MVP. Possibly impulse shapers in the future as composition.
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
- **MCU C/Rust boundary — architectural invariant:** [`docs/kalico-rewrite/mcu-c-rust-boundary.md`](docs/kalico-rewrite/mcu-c-rust-boundary.md). Read this before adding shared state between C and Rust on the MCU, or before reaching for `#[link_section]` on a Rust static. Rules: C owns boot, safety-critical paths, and all shared-memory placement; Rust owns the motion engine; the seam is `extern "C"` + `#[repr(C)]` only.
