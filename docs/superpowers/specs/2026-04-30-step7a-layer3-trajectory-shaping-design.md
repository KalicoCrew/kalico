# Step 7-A: Layer 3 trajectory shaping — design spec

Build-order step: **7-A** (Layer 3 minimum for first-print MVP).

## Scope

Time-reparameterization, smooth-shaper convolution, and β-medium
shaper-aware TOPP-RA feedback. This is the host-side trajectory
transformation pipeline that sits between Layer 2 (temporal scheduling)
and Layer 4 (MCU runtime evaluation).

### In scope

- New `trajectory` crate (`rust/trajectory/`) wrapping
  `temporal::plan_batch`.
- Per-axis smooth-ZV / smooth-MZV shaper convolution via
  `nurbs::algebra::convolve`.
- Time-reparameterization: adaptive polynomial fit of x(s) via
  `fit_x_to_arc_length_piece`, then exact composition with degree-2
  s(t) per TOPP-RA grid piece. The composition step is exact; the
  x(s) fit carries bounded error (≤ fit tolerance).
- C¹-constrained adaptive polynomial refit (degree 4, ≤5 µm L∞
  position error) to reduce piece count before convolution.
- β-medium outer iteration: TOPP-RA → shape → peak-check → derate →
  re-solve.
- E-follows-XY metadata passthrough for COUPLED segments.
- Independent E time-domain scheduling for retraction/prime segments.
- Shaper kernel generation ported from bleeding-edge-v2 `init_smoother`.

### Out of scope

- Corner-blend finalization (Step 8).
- Smooth-shaper families beyond smooth_zv / smooth_mzv (Step 8).
- Tanh nonlinear PA (Step 9).
- Phase stepping (Step 10).
- Layer 4 curve-pool refactor (Step 7-B — hard prerequisite for
  end-to-end integration but not Layer 3's responsibility).

## Crate structure

```
rust/trajectory/
├── Cargo.toml          # deps: nurbs, temporal, geometry
├── src/
│   ├── lib.rs          # public API: shape_batch
│   ├── reparam.rs      # Stage 2: s(t) construction, composition
│   ├── fit.rs          # Stage 2c: C¹-constrained Hermite refit
│   ├── pad.rs          # Stage 3a: variable-width neighbor padding
│   ├── shaper.rs       # Stage 3b-c: convolution + trim
│   ├── peak.rs         # Stage 4: polynomial peak-check
│   ├── beta.rs         # Stage 5: β-medium convergence loop
│   ├── kernel.rs       # Shaper kernel generation from frequency
│   └── e_independent.rs # Stage 6: E-only segment scheduling
└── tests/
    ├── reparam.rs
    ├── fit.rs
    ├── shaper.rs
    ├── peak.rs
    ├── beta_convergence.rs
    └── end_to_end.rs
```

## Public API

### Entry point

```rust
pub fn shape_batch(
    input: &ShapeBatchInput<'_>,
) -> Result<ShapeBatchOutput, ShapeError>
// Note: BetaNotConverged is a warning, not a hard error — the output
// is included in the Ok variant with a warning flag. See ShapeBatchOutput.
```

### Input types

```rust
pub struct ShapeBatchInput<'a> {
    /// Segments with temporal + geometry metadata.
    pub segments: &'a [ShapeSegmentInput<'a>],
    pub grid_strategy: temporal::multi::GridStrategy,
    pub worker_threads: usize,
    pub shaper: ShaperConfig,
    /// L∞ position tolerance for the adaptive refit. Default 0.005 mm.
    pub fit_tolerance_mm: f64,
    /// Maximum β-medium outer iterations. Default 5.
    pub beta_max_iters: u8,
    /// β convergence threshold: stop when all derate ratios are within
    /// this fraction of 1.0. Default 0.01 (1%).
    pub beta_convergence_ratio: f64,
    /// E-axis limits for independent E segments (retraction/prime).
    pub e_limits: ELimits,
}

/// Per-segment input combining temporal scheduling data with Layer 1
/// geometry metadata. Constructed from a CubicSegment + Limits pair.
pub struct ShapeSegmentInput<'a> {
    /// Temporal scheduling input (curve, limits, junction tolerance).
    pub temporal: temporal::multi::SegmentInput<'a>,
    /// E-mode classification from Layer 1.
    pub e_mode: geometry::segment::EMode,
    /// Extrusion ratio for COUPLED_TO_XY segments.
    pub extrusion_per_xy_mm: f64,
    /// Independent E NURBS for retraction/prime segments.
    pub e_independent: Option<&'a nurbs::ScalarNurbs<f64>>,
    /// Feedrate from G-code (mm/s). Used for independent E scheduling.
    pub feedrate_mm_s: f64,
}

/// E-axis kinematic limits for independent E scheduling (Stage 6).
pub struct ELimits {
    pub v_max: f64,
    pub a_max: f64,
}

pub struct ShaperConfig {
    /// X-axis shaper. Required (Passthrough rejected at validation).
    pub x: RequiredShaper,
    /// Y-axis shaper. Required (Passthrough rejected at validation).
    pub y: RequiredShaper,
    /// Z-axis shaper. Passthrough by default.
    pub z: AxisShaper,
}

pub enum RequiredShaper {
    SmoothZv { frequency_hz: f64 },
    SmoothMzv { frequency_hz: f64 },
}

pub enum AxisShaper {
    SmoothZv { frequency_hz: f64 },
    SmoothMzv { frequency_hz: f64 },
    Passthrough,
}
```

### Output types

```rust
pub struct ShapeBatchOutput {
    pub segments: Vec<ShapedSegment>,
    /// Number of β-medium iterations executed.
    pub beta_iters: u8,
    /// Joining status from the final temporal::plan_batch call.
    pub temporal_status: temporal::multi::JoiningStatus,
    /// Non-empty when β-medium loop hit max iterations without full
    /// convergence. The output is still usable — segments may slightly
    /// exceed a_machine. Caller decides whether to accept or reject.
    pub beta_warning: Option<BetaWarning>,
}

pub struct BetaWarning {
    pub worst_ratio: f64,
    pub segments_exceeding: Vec<usize>,
}

pub struct ShapedSegment {
    /// Per-axis shaped trajectory in the time domain.
    /// X and Y: post-convolution piecewise polynomial.
    /// Z: fitted but unshaped (passthrough by default).
    pub axes: [nurbs::ScalarNurbs<f64>; 3],
    pub e_mode: geometry::segment::EMode,
    pub extrusion_per_xy_mm: f64,
    /// For INDEPENDENT E segments only (retraction/prime).
    pub e_independent: Option<nurbs::ScalarNurbs<f64>>,
    pub t_start: f64,
    pub t_end: f64,
}
```

### Error types

```rust
pub enum ShapeError {
    /// temporal::plan_batch returned an error.
    TemporalBatch(temporal::multi::BatchError),
    /// Joining loop did not converge (StalledOnInfeasibleSegment or
    /// CappedAtMaxSweeps). Contains the joining status.
    TemporalJoining(temporal::multi::JoiningStatus),
    /// A segment's TopProfile has non-success status after all
    /// temporal retries.
    SegmentUnsolvable { index: usize, status: temporal::SolveStatus },
    /// Fitting failed on a segment (tolerance not reached after
    /// adaptive subdivision).
    FitFailure { index: usize, detail: nurbs::algebra::FitError },
    /// An algebra primitive (compose, convolve, restrict) failed.
    Algebra { index: usize, detail: nurbs::algebra::AlgebraError },
    /// Empty segment buffer.
    EmptySegments,
}
```

## Pipeline

Six stages per β iteration, plus the outer convergence loop.

### Stage 0 — Batch partitioning

Split the input segments into **runs**: contiguous groups of XY-motion
segments (`CoupledToXy` or `Travel`), separated by independent E
segments. Each run is a self-contained batch for `plan_batch`. Independent
E segments are scheduled separately in Stage 6.

Timeline construction: each run gets a global time offset. Between runs,
the independent E segment's duration (from Stage 6 trapezoidal
scheduling) is inserted. Junction velocities at run boundaries are
forced to zero (the machine is at rest during retraction/prime).

### Stage 1 — TOPP-RA solve (per run)

Construct `temporal::multi::BatchInput` from the current run's segments
and (potentially derated) limits. Call `temporal::plan_batch()`.

Gate on `BatchOutput.joining_status`:

- `Converged` → proceed.
- `StalledOnInfeasibleSegment` → return `ShapeError::TemporalJoining`.
- `CappedAtMaxSweeps` → return `ShapeError::TemporalJoining`.

Per-profile status check: only `Solved`, `SolvedInexact`, and `SolvedSlp`
are success statuses. Profiles with any other status (`Infeasible`,
`MaxIter`, `DivergedSlp`, `MaxIterSlp`) return
`ShapeError::SegmentUnsolvable` with the segment index.

### Stage 2 — Time-reparameterization + fit (parallel)

Parallel over segments using a scoped-thread executor (mutex work queue +
`std::thread::scope`, same pattern as `temporal::multi::parallel`).

#### 2a. Construct s(t) per TOPP-RA grid piece

From each `TopProfile`'s `Vec<GridSample>`, for consecutive samples k
and k+1:

```
v_k     = sample[k].v          // = sqrt(b_k)
a_k     = (sample[k+1].b - sample[k].b) / (2 * Δs_k)
Δs_k    = sample[k+1].s - sample[k].s
Δt_k    = 2 * Δs_k / (v_k + v_{k+1})
t_k     = T_global + cumulative sum of Δt within this segment
```

where `T_global` is the batch-global time offset for this segment,
computed by summing the `total_time` of all preceding segments. All
`BezierPiece` domains use batch-global time so that Stage 3's neighbor
padding can concatenate pieces from adjacent segments directly.

Each grid piece is a `BezierPiece<f64>` in Pascal-shifted monomial basis:

```
s(t) = s_k + v_k·(t - t_k) + (a_k/2)·(t - t_k)²
coeffs = [s_k, v_k, a_k/2]
domain = [t_k, t_{k+1}]
```

**Near-zero velocity special case:** when both `v_k < ε_v` (e.g.,
0.01 mm/s) and `v_{k+1} < ε_v`, the grid piece represents
near-stationary motion. Emit a **constant-position piece** instead:
`coeffs = [s_k, 0, 0]` with `Δt = Δs_k / ε_v`. This avoids the
composition endpoint mismatch that would occur if v_k were clamped but
polynomial coefficients left unclamped. The position error is bounded
by Δs_k (sub-micron for typical 0.5mm grid spacing at near-zero
velocity). The compose step is skipped for constant-position pieces —
the output is constant x(t) = x(s_k) per axis.

#### 2b. Compose x(s(t))

For each TOPP-RA grid piece, fit the geometry x(u) reparameterized by
arc length on `[s_k, s_{k+1}]` using
`nurbs::algebra::fit_x_to_arc_length_piece::<3>`. This produces a
polynomial x(s) per axis (adaptive degree, target degree 3, max degree
5, tolerance = `fit_tolerance_mm`). The fit uses the segment's
arc-length table for the u(s) lookup.

The arc-length table for the u(s) lookup is built per segment using the
same parameters as temporal's internal table (`tolerance = 1e-6`,
`max_intervals = 1024`). The final `s_hi` per grid piece is clamped to
`min(s_{k+1}, table.total_length())` to absorb any sub-1e-9 endpoint
drift between independently-built tables.

Then compose x(s) with s(t) via
`nurbs::algebra::compose_vector_piece::<3>`. The composition step is
exact (polynomial-of-polynomial). The overall x(t) carries the bounded
fit error from x(s).

Output: N grid pieces of degree `d_fit_xs × 2` (typically 6 for
degree-3 x(s) fits), per axis. Domain of each piece is `[t_k, t_{k+1}]`
in batch-global time.

Precondition check: `s(t_k) = s_k` and `s(t_{k+1}) = s_{k+1}` within
1e-9 (compose_vector_piece's runtime guard). Near-zero-velocity
constant-position pieces (from Stage 2a) skip composition.

#### 2c. C¹-constrained adaptive refit

Merge adjacent degree-6 grid pieces into fewer degree-4 pieces:

- **Endpoint constraints (Hermite):** at each piece boundary, match
  position and first derivative from the exact degree-6 trajectory. A
  degree-4 polynomial has 5 coefficients; 4 are consumed by the two
  endpoint pairs (position + velocity at each end), leaving 1 interior
  degree of freedom.
- **Interior optimization:** minimize L∞ residual with the remaining
  DOF (or use a Chebyshev-optimal approach with the endpoint constraints
  baked in).
- **Tolerance check:** verify L∞ residual at 4×(degree+1) uniform
  sample points ≤ `fit_tolerance_mm`. On failure, bisect the merged
  group at a TOPP-RA grid point and retry each half.
- **Boundary constraint:** piece boundaries must be a subset of TOPP-RA
  grid points (per `layer3-time-polynomial-fit-bounds.md` critical
  condition — C¹ discontinuities at TOPP-RA breakpoints invalidate the
  Taylor-remainder error bound if a piece boundary straddles them).

Output: K pieces per axis (typically 4–30 for a 12.5 mm segment at
500 mm/s), degree 4, C¹ at piece boundaries.

#### 2d. Split vector → per-axis scalars

Extract per-axis `ScalarNurbs<f64>` from the fitted `BezierPiece` arrays.
Each axis becomes an independent scalar piecewise polynomial in time.

### Stage 3 — Shaper convolution (parallel, reads neighbors)

Parallel over segments. Each segment reads its own and neighbors' Stage 2
output (read-only).

#### 3a. Variable-width padding

For each segment i, collect neighbor fitted pieces:

- **Left padding:** scan segments i-1, i-2, ... backward, accumulating
  time duration. Stop when accumulated duration ≥ T_sm/2 (kernel
  half-support of the widest active kernel across X/Y). Include all
  fitted pieces from the accumulated segments.
- **Right padding:** scan segments i+1, i+2, ... forward, same logic.
- **Boundary extension:** at the first segment in the batch, extend
  with constant position (zero velocity) for T_sm/2. Same at the last
  segment. This is correct because the shaper kernel is normalized
  (integral = 1, DC gain = 1), so convolving a constant preserves it.
  **Assumption:** batch edges coincide with zero-velocity boundaries
  (print start/end, or pause points). If `shape_batch` is later called
  on streaming lookahead windows, the API will need halo segments or
  boundary-context metadata to avoid baking false stops at batch edges.

Concatenate: left-pad pieces + segment i's pieces + right-pad pieces →
single padded `ScalarNurbs<f64>` per axis.

#### 3b. Per-axis convolution

For each axis:

- `Passthrough` → skip convolution; use the fitted (unshaped)
  `ScalarNurbs` directly.
- `SmoothZv` / `SmoothMzv` → call `nurbs::algebra::convolve(padded_curve, kernel)`.

Kernel is a `PiecewisePolynomialKernel<f64>` with centered support
`[-T_sm/2, T_sm/2]`. Generated once per axis by `AxisShaper::to_kernel()`
at the start of `shape_batch`.

Output degree: `d_fit + d_kernel + 1 = 4 + 4 + 1 = 9` (degree 9 per
piece, lower than the degree-11 from the exact path — a benefit of
fit-then-convolve).

Output piece count: ~2K+1 per axis in the generic case (Minkowski-sum
breakpoints). `knot_remove_redundant` may reduce this where adjacent
pieces are naturally smooth (especially at C¹ input boundaries, which
convolution raises to C² or higher).

#### 3c. Trim to segment time domain

Restrict the convolution output to `[t_start, t_end]` (the segment's
original time domain, not the padded domain).

Implementation: `restrict_to_domain(curve, t_start, t_end)` — extract
all Bézier pieces overlapping the domain, split boundary pieces at
`t_start` and `t_end` via `split_piece_at()`, reassemble into a
`ScalarNurbs`.

### Stage 4 — Peak acceleration check (parallel)

For each segment, for each axis (except `Passthrough`):

1. Compute second derivative of the shaped `ScalarNurbs`: per-piece
   differentiation in Pascal-shifted monomial basis:
   `coeffs'[k] = (k+1) · coeffs[k+1]`. Apply twice for second
   derivative. Degree drops by 2 (from 9 to 7).
2. Find the maximum of `|x''(t)|` on each piece: find roots of the
   first derivative of x'' (degree 6 polynomial), evaluate x'' at
   roots and piece endpoints, take the maximum absolute value.
3. Compare against `a_max[axis]` for this segment.

Root-finding on a degree-6 polynomial: companion-matrix eigenvalue
decomposition, filtering for real roots within the piece domain. Standard
numerical approach; LAPACK not needed at this size (6×6 matrix).

### Stage 5 — β-medium convergence

Outer loop logic in `beta.rs`:

```
// machine_a_max[seg][axis] = original immutable machine limits
// planning_a_max[seg][axis] = mutable limits fed to TOPP-RA

for iter in 0..beta_max_iters:
    run stages 1-4 with planning_a_max
    
    any_derated = false
    for each segment × axis:
        // Compare against MACHINE limits, not planning limits
        if peak > machine_a_max[seg][axis]:
            ratio = machine_a_max[seg][axis] / peak
            new_limit = planning_a_max[seg][axis] * ratio
            // Monotone derate: planning limits only decrease
            planning_a_max[seg][axis] =
                min(planning_a_max[seg][axis], new_limit)
            any_derated = true
    
    if not any_derated:
        return Ok(output)  // converged: all peaks ≤ machine limits
    
    worst_ratio = min(ratio across all derated segment×axis pairs)
    if worst_ratio > 1.0 - beta_convergence_ratio:
        // Derate is tiny — one final solve to bake the derate in
        run stages 1-4 with planning_a_max
        return Ok(output with beta_warning = Some(BetaWarning { ... }))

// Exhausted iterations — final solve with current planning limits
run stages 1-4 with planning_a_max
return Ok(output with beta_warning = Some(BetaWarning { ... }))
```

**Monotone derate guard:** limits never increase across β iterations.
This prevents oscillation where iteration N derates axis X, causing
iteration N+1's velocity profile to load axis Y harder, derating Y,
which then shifts load back to X.

**β-medium non-convergence is a warning, not an error.** The trajectory
from the last iteration is always returned in the `Ok` variant. If the
loop hit `beta_max_iters` without full convergence,
`ShapeBatchOutput.beta_warning` is set with the worst-case ratio and
the list of segments that still exceed `a_machine`. The caller decides
whether to accept the trajectory or reject it.

### Stage 6 — Independent E segments

Segments with `EMode::Independent` (retraction, prime, filament change)
bypass Stages 1–5. Their XYZ motion is zero (or negligible); they carry
an E-axis NURBS from Layer 1.

Time-domain scheduling for independent E:

1. Compute E path length from the `e_independent` NURBS.
2. Apply trapezoidal velocity profile from feedrate and E-axis limits
   (max velocity, max acceleration from config).
3. Time-reparameterize the E NURBS with the trapezoidal s(t) via
   composition.
4. No shaper convolution (E is not a shaped axis).
5. Output: `ShapedSegment` with identity axes (constant position for
   XYZ), `e_independent` set to the time-parameterized E NURBS.

## Shaper kernel module

### Kernel generation

Ported from bleeding-edge-v2 `init_smoother`. Each smooth shaper family
is a degree-4 polynomial of compact support, parameterized by resonance
frequency f:

- **smooth_zv:** T_sm = 0.8025 / f. Single-piece degree-4 polynomial
  on `[-T_sm/2, T_sm/2]`.
- **smooth_mzv:** T_sm = 0.95625 / f. Single-piece degree-4 polynomial
  on `[-T_sm/2, T_sm/2]`.

The polynomial coefficients are a closed-form function of T_sm (derived
from the `init_smoother` convolution of rectangular pulses with
polynomial windowing). The kernel is normalized: integral over support
= 1 (DC gain = 1). The kernel has at least double zeros at the support
boundaries (value and first derivative vanish), ensuring the extended
kernel (zero outside support) is C¹.

```rust
impl AxisShaper {
    pub fn to_kernel(&self) -> Option<PiecewisePolynomialKernel<f64>> {
        match self {
            Self::SmoothZv { frequency_hz } => {
                let t_sm = 0.8025 / frequency_hz;
                Some(build_smooth_zv_kernel(t_sm))
            }
            Self::SmoothMzv { frequency_hz } => {
                let t_sm = 0.95625 / frequency_hz;
                Some(build_smooth_mzv_kernel(t_sm))
            }
            Self::Passthrough => None,
        }
    }
}
```

Constructed via `PiecewisePolynomialKernel::single_poly_from_absolute()`
with the absolute-monomial coefficients from the ported formula.

## New primitives required in `nurbs`

### 1. C¹-constrained Hermite fitter

New function in `nurbs::algebra`:

```rust
pub fn fit_hermite_c1<const D: usize>(
    pieces: &[[BezierPiece<f64>; D]],
    tolerance_mm: f64,
    target_degree: u8,
) -> Result<[Vec<BezierPiece<f64>>; D], FitError>
```

Adaptively merges adjacent exact pieces into fewer pieces with C¹
continuity at boundaries (position + velocity matching). Adaptive
bisection when tolerance is exceeded.

### 2. Trim / restrict-to-domain

New function in `nurbs`:

```rust
pub fn restrict_to_domain<T: Float>(
    curve: &ScalarNurbs<T>,
    t_lo: T,
    t_hi: T,
) -> Result<ScalarNurbs<T>, AlgebraError>
```

Extracts the portion of a `ScalarNurbs` on `[t_lo, t_hi]` via Bézier
piece extraction and boundary splitting.

### 3. Polynomial differentiation

New function in `nurbs::bezier`:

```rust
pub fn differentiate(piece: &BezierPiece<f64>) -> BezierPiece<f64>
```

In Pascal-shifted monomial basis: `coeffs'[k] = (k+1) * coeffs[k+1]`.
Degree drops by 1. Domain unchanged.

### 4. Polynomial root-finding on bounded interval

New function in `nurbs::bezier`:

```rust
pub fn real_roots_in_domain(piece: &BezierPiece<f64>) -> Vec<f64>
```

Finds all real roots of the polynomial within `[u_start, u_end]`.
Companion-matrix eigenvalue decomposition for degree ≤ 10. Filters for
real roots (imaginary part < ε) within the domain.

## Parallelism

Layer 3 owns its own scoped-thread executor, same pattern as
`temporal::multi::parallel` (mutex work queue + `std::thread::scope`).
The two executors are never concurrent — they run in sequential phases
within the β loop:

1. TOPP-RA (temporal's executor, 3 threads)
2. Time-reparam + fit (trajectory's executor, configurable threads)
3. Pad + convolve + trim (trajectory's executor)
4. Peak-check (trajectory's executor)

Thread count is configurable via `ShapeBatchInput::worker_threads`.
Default 3 (same as temporal, avoids contention with Klipper on Pi 5
cores 0–1).

## Degree and piece-count budget

| Stage | Degree | Pieces per segment per axis |
|-------|--------|---------------------------|
| Input (Layer 1) | 3 (cubic Bézier) | 1 |
| After TOPP-RA grid | — | N (10–25 grid pieces) |
| After x(s) fit | 3 (target; up to 5) | N |
| After composition x(s(t)) | 6 (= d_xs × 2; up to 10) | N |
| After C¹ refit | 4 | K (4–30, typically 5–15) |
| After convolution | 9 (= 4 + 4 + 1) | ~2K+1 (9–61, reduced by knot removal) |

Post-convolution degree 9 means 10 coefficients per piece (f32 on MCU).
At ~20 pieces per axis (after knot removal) × 4 axes × 3 segment buffer
× 10 × 4 bytes ≈ 9.6 KB. Fits comfortably in H723's 1 MB SRAM.

MCU evaluation: Horner's for degree 9 is 9 FMA operations per sample.
At 40 kHz × 4 axes = 360K FMA/s. Negligible on Cortex-M7 at 550 MHz.

## Verified mathematical claims

Per kalico-verifier (this session, 2026-04-29):

1. **s(t) is exactly degree-2** per TOPP-RA grid piece, given
   piecewise-linear b(s). Constant db/ds implies constant dv/dt via
   exact algebraic cancellation: dv/dt = (m/(2v))·v = m/2.
2. **Composition degree is 3×2 = 6** (standard polynomial composition
   identity; exact for non-degenerate cubics, at-most-6 for degenerate).
3. **Convolution output degree is d_x + d_w + 1** (integral-form
   convolution; +1 from antidifferentiation with u-dependent limits).
   With fit degree 4 + kernel degree 4: output degree 9.
4. **Output piece count is ~2K+1** for K input pieces with a
   single-piece kernel (Minkowski-sum breakpoints, generic case).

Per kalico-verifier (this session, 2026-04-30):

5. **Pad-and-trim is mathematically exact** — padding with sufficient
   neighbor data (covering T_sm/2 of time) and trimming to the
   segment's original domain produces identical results to convolving
   the full trajectory. Variable-width padding required: short segments
   (duration < T_sm/2) may need multiple neighbors.
6. **Constant-position boundary extension is correct** for normalized
   kernels (integral = 1).
7. **Phase 2 (convolution) is embarrassingly parallel** — no segment's
   convolution output depends on another segment's convolution output.

## Fitting error and smoothness

The C¹-constrained refit introduces ≤5 µm L∞ position error relative
to the exact time-reparameterized trajectory. This is:

- **Below motor resolution** (~2.5 µm for 0.9° stepper at 1/256
  microstep on GT2-20T 40mm/rev).
- **Below the TOPP-RA grid discretization error** (O(h²) where h is
  the 0.5 mm grid spacing).

The C¹ constraint ensures velocity continuity at piece boundaries,
preserving the smoothness of the original trajectory. The exact
trajectory is C¹ at TOPP-RA grid joints; the fit preserves this by
construction. Convolution with the smooth-shaper kernel further raises
continuity at these joints (from C¹ to at least C²).

The peak-acceleration check (Stage 4) runs on the fitted+convolved
trajectory — the same trajectory sent to the MCU. The motor executes
the fitted trajectory, so the peak check validates what the motor
actually tracks.

## Prerequisites

- `nurbs::algebra::compose_vector_piece` — exists (landed in 7-pre).
- `nurbs::algebra::fit_x_to_arc_length_piece` — exists (landed in 7-pre).
- `nurbs::algebra::convolve` — exists (landed in Layer 0).
- `geometry::splitter::split_segment_to_cap` — exists (landed in 7-pre).
- `temporal::plan_batch` — exists (landed in Step 4.5).
- **Layer 4 curve-pool refactor (Step 7-B):** MAX_DEGREE must be bumped
  from 3 to ≥9, MAX_CONTROL_POINTS and MAX_KNOT_VECTOR_LEN sized
  accordingly. Hard prerequisite for end-to-end integration but not for
  Layer 3 development/testing (Layer 3 tests validate output as
  `ScalarNurbs<f64>` without sending to MCU).

## Testing strategy

- **Unit tests per stage:** synthetic single-segment inputs with known
  analytic solutions (straight line at constant velocity, constant-
  velocity arc).
- **Composition correctness:** verify x(s(t)) against direct evaluation
  of x(s) at s=s(t) at dense sample points.
- **Fit quality:** verify L∞ residual ≤ tolerance at dense samples,
  verify C¹ continuity (velocity match) at all piece boundaries.
- **Convolution cross-check:** multi-segment pad-and-trim output vs
  single global-convolve reference at sample points (verifies padding
  correctness).
- **β convergence:** synthetic segment with known post-shape peak >
  a_machine; verify derating converges in ≤3 iterations; verify
  monotone derate (limits never increase).
- **End-to-end:** feed representative G5-only G-code through full
  pipeline; verify all segments pass peak-check; verify total trajectory
  time is within 1% of unshaped TOPP-RA time (shaper should not
  significantly slow the trajectory when limits are not binding).
